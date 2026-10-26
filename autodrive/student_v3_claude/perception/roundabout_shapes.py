"""Publish compact observed obstacle boxes at the surveyed roundabout only.

Tracking, class assignment and pedestrian-part merging keep their original
geometry. Publication alone can separate route-parallel rail returns from a
compact obstacle. The existing 0.8 m minimum dimensions and 0.3 m total
dimension padding remain; the raw stopping sweep keeps every physical return.
"""
from dataclasses import dataclass
import hashlib
import math

import cv2
import numpy as np


ROUTE_SHA256 = '47c35f037a853123abf86a5e5c0f48562330f698351cfe02a55686a5253a4e2d'
STOP_STATION_M = 885.1006875597407
APPROACH_M = 13.
EXIT_M = 14.
MIN_DIMENSION_M = .8
TOTAL_PADDING_M = .3
CONTAINMENT_TOL_M = 1e-9


def route_barrier_points(world_xy, route, route_tree, route_direction, route_station,
                         *, min_span=6., min_points=15):
    """Preserve the fusion filter's same-offset, along-route line test."""
    points = np.asarray(world_xy, float)
    if (points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all()
            or not math.isfinite(min_span) or min_span <= 0. or len(points) < min_points):
        return np.zeros(len(points), bool)
    _, index = route_tree.query(points)
    relative = points-np.asarray(route)[index]
    direction = np.asarray(route_direction)[index]
    lateral = -np.sin(direction)*relative[:, 0]+np.cos(direction)*relative[:, 1]
    station = np.asarray(route_station)[index]
    result = np.zeros(len(points), bool)
    bins = np.round(lateral/.3).astype(np.int64)
    for band in np.unique(bins):
        selected = np.abs(bins-band) <= 1
        if selected.sum() >= min_points and np.ptp(station[selected]) >= min_span:
            result |= selected
    return result


class RoundaboutShapeRegion:
    """Recognize the saved route and the source ego's roundabout station."""

    def __init__(self, route):
        self.certified = False
        try:
            xy = np.asarray(route, dtype='<f8')
            if (xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2
                    or not np.isfinite(xy).all()
                    or hashlib.sha256(xy.tobytes()).hexdigest() != ROUTE_SHA256):
                return
            delta = np.diff(xy, axis=0)
            lengths = np.linalg.norm(delta, axis=1)
            valid = lengths > 1e-8
            if not valid.any():
                return
            stations = np.r_[0., np.cumsum(lengths)]
            self.starts = xy[:-1][valid].copy()
            self.lengths = lengths[valid].copy()
            self.tangents = delta[valid] / self.lengths[:, None]
            self.stations = stations[:-1][valid].copy()
            for array in (self.starts, self.lengths, self.tangents, self.stations):
                array.setflags(write=False)
            self.certified = True
        except (TypeError, ValueError, OverflowError):
            pass

    def active(self, source_ego):
        """Use the actual selected source sample, without interpolating it."""
        if not self.certified:
            return False
        try:
            stamp, x, y, yaw, speed, yaw_rate = map(float, source_ego)
            if stamp <= 0. or not all(math.isfinite(v) for v in
                                     (stamp, x, y, yaw, speed, yaw_rate)):
                return False
            xy = np.array([x, y])
            along = np.clip(np.sum((xy-self.starts)*self.tangents, axis=1),
                            0., self.lengths)
            projected = self.starts + along[:, None]*self.tangents
            index = int(np.argmin(np.sum((projected-xy)**2, axis=1)))
            station = float(self.stations[index]+along[index])
            return STOP_STATION_M-APPROACH_M <= station <= STOP_STATION_M+EXIT_M
        except (TypeError, ValueError, OverflowError):
            return False


@dataclass(frozen=True)
class PublishGeometry:
    position: tuple
    local_position: tuple
    size: tuple
    yaw: float
    original_position: tuple
    original_size: tuple
    original_yaw: float
    world_points: np.ndarray
    roadside_points_excluded: int = 0


def _contains_with_padding(geometry):
    try:
        points = np.asarray(geometry.world_points, float)
        position = np.asarray(geometry.position, float)
        size = np.asarray(geometry.size, float)
        local = np.asarray(geometry.local_position, float)
        yaw = float(geometry.yaw)
        if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 4
                or position.shape != (2,) or size.shape != (2,) or local.shape != (2,)
                or not np.isfinite(np.r_[points.ravel(), position, size, local, yaw]).all()
                or np.min(size) < MIN_DIMENSION_M+TOTAL_PADDING_M-1e-12):
            return False
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        relative = (points-position) @ rotation
        return bool(np.all(np.abs(relative) <=
                    size/2.-TOTAL_PADDING_M/2.+CONTAINMENT_TOL_M))
    except (TypeError, ValueError, OverflowError):
        return False


def enclosing_publish_geometry(member_xy, source_ego, original, *, roadside_points_excluded=0):
    """Choose the smallest padded area among axis, PCA and min-area candidates.

    OpenCV supplies only an orientation. Reproject every original float64
    point to obtain the bounds and centre, avoiding float32 hull rounding.
    This does not claim a global optimum after minimum dimensions/padding.
    """
    try:
        points = np.asarray(member_xy, dtype=np.float64)
        if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 4
                or not np.isfinite(points).all()):
            return None
        stamp, x, y, yaw, speed, yaw_rate = map(float, source_ego)
        original_pos = np.asarray(original['pos'], float)
        original_size = np.asarray(original['size'], float)
        original_yaw = float(original['yaw'])
        if (stamp <= 0. or original_pos.shape != (2,) or original_size.shape != (2,)
                or not np.isfinite(np.r_[stamp, x, y, yaw, speed, yaw_rate,
                                         original_pos, original_size, original_yaw]).all()
                or np.min(original_size) < MIN_DIMENSION_M+TOTAL_PADDING_M-1e-12):
            return None
        mean = points.mean(axis=0)
        centred = points-mean
        principal = np.linalg.svd(centred, full_matrices=False)[2][0]
        rectangle = cv2.minAreaRect(centred.astype(np.float32))
        corners = cv2.boxPoints(rectangle).astype(np.float64)
        edges = np.diff(np.vstack((corners, corners[:1])), axis=0)
        edge = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))]
        angles = [original_yaw-yaw, 0., math.atan2(principal[1], principal[0])]
        if np.linalg.norm(edge) > 1e-8:
            angles.append(math.atan2(edge[1], edge[0]))
        world_rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                                   [math.sin(yaw), math.cos(yaw)]])
        world_points = points @ world_rotation.T + [x, y]
        world_points.setflags(write=False)
        best, best_area = None, float(np.prod(original_size))
        if not math.isfinite(best_area):
            return None
        for angle in angles:
            rotation = np.array([[math.cos(angle), -math.sin(angle)],
                                 [math.sin(angle), math.cos(angle)]])
            projected = centred @ rotation
            lo, hi = projected.min(axis=0), projected.max(axis=0)
            size = np.maximum(hi-lo, MIN_DIMENSION_M)+TOTAL_PADDING_M
            area = float(np.prod(size))
            if area >= best_area-max(1e-10, abs(best_area)*1e-12):
                continue
            local_position = mean + ((lo+hi)/2.) @ rotation.T
            position = local_position @ world_rotation.T + [x, y]
            candidate = PublishGeometry(tuple(position), tuple(local_position), tuple(size),
                yaw+angle, tuple(original_pos), tuple(original_size), original_yaw, world_points,
                roadside_points_excluded)
            if _contains_with_padding(candidate):
                best, best_area = candidate, area
        return best
    except (KeyError, TypeError, ValueError, IndexError, OverflowError,
            np.linalg.LinAlgError, cv2.error):
        return None


def publication_detection(detection):
    """Apply only to an unmerged, exactly stationary generic obstacle output."""
    candidate = detection.get('publish_geometry')
    if not isinstance(candidate, PublishGeometry) or detection.get('kind') != 'obstacle':
        return detection
    try:
        velocity = np.asarray(detection['vel'], float)
        if (velocity.shape != (2,) or not np.isfinite(velocity).all()
                or not np.all(velocity == 0.) or detection.get('merged_part_ids')
                or not np.array_equal(detection['pos'], candidate.original_position)
                or not np.array_equal(detection['size'], candidate.original_size)
                or float(detection['yaw']) != candidate.original_yaw
                or not _contains_with_padding(candidate)):
            return detection
        return dict(detection, pos=np.asarray(candidate.position),
                    local=np.asarray(candidate.local_position),
                    size=np.asarray(candidate.size), yaw=candidate.yaw,
                    roadside_points_excluded=candidate.roadside_points_excluded)
    except (KeyError, TypeError, ValueError, OverflowError):
        return detection
