"""Short raw-scan calibration of unchanged, current fusion source boxes."""
from __future__ import annotations

import copy
import math
import numpy as np

from perception.obstacle_geometry import (FUSION_OBSTACLE_MIN_DIMENSION_M,
    FUSION_OBSTACLE_DIMENSION_PADDING_M)
from perception.stationary_semantics import metadata, with_metadata
from perception.road_scan_frame import road_scan_frame

MAX_CALIBRATION_AGE_S = .5
ROAD_PADDING_M = .1




def signature(obj):
    shape = metadata(obj)
    if (len(obj) < 12 or obj[3] != 'fusion_obstacle' or obj[4] != 'obstacle'
            or 'road_static_geometry' in shape):
        raise ValueError('not a current fusion source box')
    uid = int(obj[11])
    if isinstance(obj[11], bool) or uid != obj[11] or uid < 0:
        raise ValueError('invalid source identity')
    values = np.asarray([*obj[:3], obj[5], *obj[6:11]], float)
    source = np.asarray(obj[6:8], float)
    raw = np.asarray(shape.get('raw_size_m', source), float)
    padding = float(shape.get('fusion_padding_m', FUSION_OBSTACLE_DIMENSION_PADDING_M))
    height = shape.get('height_m')
    if (values.shape != (9,) or raw.shape != (2,) or not np.isfinite(np.r_[values, raw, padding]).all()
            or abs(float(obj[5])) >= .2 or np.max(np.abs(values[-2:])) >= .2
            or padding != FUSION_OBSTACLE_DIMENSION_PADDING_M
            or np.any(source < FUSION_OBSTACLE_MIN_DIMENSION_M+padding-1e-6)
            or not np.allclose(raw, source, rtol=0., atol=1e-6)
            or (height is not None and not math.isfinite(float(height)))):
        raise ValueError('unverified source dimensions or motion')
    return uid, (obj[3], obj[4], 'raw_size_m' in shape, 'fusion_padding_m' in shape,
        height is not None), np.r_[values, raw, padding, [] if height is None else [float(height)]]


def same_source(a, b):
    return a[0:2] == b[0:2] and a[2].shape == b[2].shape and np.allclose(a[2], b[2], rtol=0., atol=1e-6)


def calibration(source, normalized):
    geometry = metadata(normalized).get('road_static_geometry')
    if not isinstance(geometry, dict) or geometry.get('version') != 1 or geometry.get('source') != 'semantic_fusion':
        return None
    physical = np.asarray(geometry['physical_size_m'], float)
    original = np.asarray(geometry['source_size_m'], float)
    checked = np.asarray(normalized[6:8], float)
    if (any(a.shape != (2,) for a in (physical, original, checked))
            or not np.isfinite(np.r_[physical, original, checked, geometry['observed_points']]).all()
            or float(geometry['observed_points']) < 1.
            or geometry['source_padding_m'] != FUSION_OBSTACLE_DIMENSION_PADDING_M
            or not math.isclose(float(geometry['applied_padding_m']), ROAD_PADDING_M, abs_tol=1e-6)
            or np.any(physical < FUSION_OBSTACLE_MIN_DIMENSION_M-1e-6)
            or not np.allclose(original, source[6:8], rtol=0., atol=1e-6)
            or not np.allclose(checked, physical+ROAD_PADDING_M, rtol=0., atol=1e-6)
            or np.any(checked > original+1e-6)
            or tuple(normalized[:2]) != tuple(source[:2])
            or tuple(normalized[3:6]) != tuple(source[3:6])
            or tuple(normalized[8:12]) != tuple(source[8:12])):
        return None
    return {key: copy.deepcopy(value) for key, value in geometry.items()
            if key not in ('initial_margin_floor_m', 'initial_margin_restore_start_m')}


def has_source_returns(source, points, ego):
    angle = math.radians(float(ego['heading_deg']))
    rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    world = points @ rotation.T + [ego['x'], ego['y']]
    yaw = math.radians(float(source[8]))
    axes = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    relative = (world-np.asarray(source[:2], float)) @ axes
    return bool(np.any((np.abs(relative) <= np.asarray(source[6:8], float)/2.+1e-6).all(axis=1)))


class RoadStaticCalibration:
    def __init__(self):
        self.reset()

    def reset(self, reason='reset'):
        self._tracks = {}
        self._frame = self._now = None
        self.last_diagnostics = dict(restored_n=0, calibrated_n=0, reason=reason)

    def update(self, sources, normalized, ego, points, *, proof, now_wall, active,
               retained_near_field_n=None):
        result = list(normalized)
        if not active:
            self.reset('outside ordinary static policy')
            return result
        try:
            now = float(now_wall)
            frame = road_scan_frame(proof, ego, now)
            cloud = np.asarray(points, float)
            if cloud.ndim != 2 or cloud.shape[1] != 2 or not np.isfinite(cloud).all():
                raise ValueError('invalid current scan')
            if self._now is not None and now < self._now:
                raise ValueError('source clock moved backwards')
            if self._frame is not None and frame != self._frame:
                if frame[0] <= self._frame[0] or frame[1] <= self._frame[1]:
                    self.reset('physical scan restarted or mismatched')
            fresh = frame != self._frame
            try:
                count = int(proof['points_n'])
                retained = int(retained_near_field_n)
                fresh_points_only = (not isinstance(retained_near_field_n, bool)
                    and retained == retained_near_field_n == 0
                    and not isinstance(proof['points_n'], bool)
                    and count == proof['points_n'] == len(cloud))
            except (KeyError, TypeError, ValueError, OverflowError):
                fresh_points_only = False
            self._tracks = {uid: track for uid, track in self._tracks.items()
                            if 0. <= now-track['frame'][0] <= MAX_CALIBRATION_AGE_S}
            restored = calibrated = 0
            if len(sources) != len(result):
                raise ValueError('source/normalization mismatch')
            for index, (source, checked) in enumerate(zip(sources, result)):
                try:
                    key = signature(source)
                except (KeyError, TypeError, ValueError, IndexError, OverflowError):
                    try:
                        self._tracks.pop(int(source[11]), None)
                    except (TypeError, ValueError, IndexError, OverflowError):
                        pass
                    continue
                for uid, track in list(self._tracks.items()):
                    if uid != key[0] and same_source((uid, key[1], key[2]), track['signature']):
                        self._tracks.pop(uid, None)
                old = self._tracks.get(key[0])
                if old and not same_source(key, old['signature']):
                    self._tracks.pop(key[0], None)
                    old = None
                geometry = calibration(source, checked)
                if geometry is not None:
                    if fresh and fresh_points_only and has_source_returns(source, cloud, ego):
                        self._tracks[key[0]] = dict(signature=key, geometry=geometry, frame=frame)
                        calibrated += 1
                    elif old and not np.allclose(geometry['physical_size_m'],
                            old['geometry']['physical_size_m'], rtol=0., atol=1e-6):
                        self._tracks.pop(key[0], None)
                    continue
                associated = has_source_returns(source, cloud, ego)
                if associated:
                    self._tracks.pop(key[0], None)
                if old is None or associated or 'road_static_geometry' in metadata(checked):
                    continue
                geometry = copy.deepcopy(old['geometry'])
                row = list(checked)
                row[6], row[7] = map(float, np.asarray(geometry['physical_size_m'])+ROAD_PADDING_M)
                row[2] = max(row[6:8])
                result[index] = with_metadata(row, road_static_geometry=geometry)
                restored += 1
            self._frame, self._now = frame, now
            self.last_diagnostics = dict(restored_n=restored, calibrated_n=calibrated,
                source_stamp=frame[0], source_sequence=frame[1], reason=None)
            return result
        except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
            self.reset('invalid measured calibration proof')
            return result
