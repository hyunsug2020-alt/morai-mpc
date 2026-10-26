"""Briefly retain measured raw targets of a committed normal-road bypass.

The caller supplies only fresh LiDAR boxes that the existing route/footprint
check identifies as the current manoeuvre's hazards. This is additional static
geometry, never a path cache, free-space proof, or permission to bypass a person.
The caller must disable it outside ordinary static roads and whenever people,
moving traffic, junctions, highways, or the tunnel own the manoeuvre.
"""
from __future__ import annotations

import copy
import math

import numpy as np

from perception.road_scan_frame import MAX_FRAME_AGE_S, road_scan_frame

from ego_body import EGO_HALF_LENGTH_M, REAR_AXLE_SHIFT_M

MAX_SOURCE_AGE_S = .5
MATCH_CENTRE_M = .6
MATCH_BOUNDS_M = .15
MAX_ANCHOR_DRIFT_M = .5
REAR_RELEASE_MARGIN_M = .15
MEMORY_KEY = 'road_raw_memory'


def _shape(obj):
    try:
        values = np.asarray([*obj[:2], *obj[6:9]], float)
        if values.shape != (5,) or not np.isfinite(values).all() or np.any(values[2:4] <= 0.):
            return None
        yaw = math.radians(values[4])
        axes = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        corners = (np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]]) *
                   values[2:4]/2.) @ axes + values[:2]
        return dict(centre=values[:2], low=corners.min(axis=0), high=corners.max(axis=0),
                    corners=corners, size=values[2:4])
    except (TypeError, ValueError, IndexError, OverflowError):
        return None


def _matches(a, b):
    return (np.linalg.norm(a['centre']-b['centre']) <= MATCH_CENTRE_M and
            np.all(a['low'] <= b['high']+MATCH_BOUNDS_M) and
            np.all(b['low'] <= a['high']+MATCH_BOUNDS_M))


def _eligible(obj, shape):
    try:
        evidence = obj[12]
        length, width = shape['size']
        longest, thinnest = max(length, width), min(length, width)
        velocity = np.asarray([obj[5], obj[9], obj[10]], float)
        return (str(obj[3]).startswith('tunnel_lidar_') and obj[4] == 'obstacle' and
                isinstance(evidence, dict) and MEMORY_KEY not in evidence and
                np.isfinite(velocity).all() and np.max(np.abs(velocity)) <= .05 and
                math.isfinite(float(evidence.get('point_count', 0.))) and
                float(evidence.get('point_count', 0.)) >= 1. and
                longest <= 8. and not (longest >= 6. and thinnest <= .35))
    except (TypeError, ValueError, IndexError, OverflowError):
        return False


class CommittedRoadRawMemory:
    """Two physical observations, at most 500 ms from the last measurement.

    ``active`` is the caller's ordinary-static-road eligibility proof.
    ``committed`` is its existing validated static manoeuvre, not a timer/model
    action. Observations may accumulate before commitment; only a committed
    manoeuvre receives missing boxes. Reused scans cannot confirm or renew one.
    ``now_wall`` and ``proof['stamp']`` use the road source certificate's clock.
    """
    def __init__(self):
        self.reset()

    def reset(self, reason='reset'):
        self._tracks = []
        self._frame = None
        self._now = None
        self.last_diagnostics = dict(retained_n=0, confirmed_n=0, reason=reason)

    def _proof(self, proof, ego, now):
        return road_scan_frame(proof, ego, now)

    def update(self, target_boxes, ego, *, proof, now_wall, active, committed):
        """Return only missing, still-fresh measured world boxes.

        Returned boxes must be appended after fresh target selection and never
        inserted into the three-second semantic memory or fed back as fresh raw
        observations. Current raw, map, body and human checks remain independent.
        """
        if not active:
            self.reset('outside ordinary committed static policy')
            return []
        try:
            now = float(now_wall)
            frame = self._proof(proof, ego, now)
            if self._now is not None and now < self._now:
                raise ValueError('source clock moved backwards')
            if self._frame is not None and frame != self._frame:
                if frame[0] <= self._frame[0] or frame[1] <= self._frame[1]:
                    self.reset('physical scan restarted or mismatched')
            fresh = frame != self._frame
            targets = []
            for obj in target_boxes:
                shape = _shape(obj)
                if shape is None:
                    continue
                if _eligible(obj, shape):
                    targets.append((obj, shape))
                elif not (len(obj) > 12 and isinstance(obj[12], dict) and MEMORY_KEY in obj[12]):
                    # A newly measured mover/person/changed class cancels its
                    # old raw target, without altering the live object's class.
                    self._tracks = [t for t in self._tracks if not _matches(t['shape'], shape)]
            used = set()
            if fresh:
                for obj, shape in targets:
                    matches = [(np.linalg.norm(t['shape']['centre']-shape['centre']), i)
                               for i, t in enumerate(self._tracks)
                               if i not in used and _matches(t['shape'], shape)]
                    index = min(matches)[1] if matches else len(self._tracks)
                    old = self._tracks[index] if matches else None
                    continuous = bool(old and 0. < frame[0]-old['stamp'] <= MAX_SOURCE_AGE_S and
                        np.linalg.norm(shape['centre']-old['anchor']) <= MAX_ANCHOR_DRIFT_M)
                    track = dict(obj=copy.deepcopy(tuple(obj)), shape=shape, stamp=frame[0],
                                 sequence=frame[1], observations=min(2, old['observations']+1) if continuous else 1,
                                 anchor=old['anchor'] if continuous else shape['centre'].copy())
                    if matches:
                        self._tracks[index] = track
                    else:
                        self._tracks.append(track)
                    used.add(index)
            self._frame, self._now = frame, now
            heading = math.radians(float(ego['heading_deg']))
            forward = np.array([math.cos(heading), math.sin(heading)])
            origin = np.asarray([ego['x'], ego['y']], float)
            rear = EGO_HALF_LENGTH_M-REAR_AXLE_SHIFT_M
            retained, alive = [], []
            for track in self._tracks:
                age = now-track['stamp']
                passed = np.max((track['shape']['corners']-origin) @ forward) < -rear-REAR_RELEASE_MARGIN_M
                if not 0. <= age <= MAX_SOURCE_AGE_S or passed:
                    continue
                alive.append(track)
                present = any(_matches(track['shape'], shape) for _, shape in targets)
                if committed and track['observations'] >= 2 and not present:
                    row = list(copy.deepcopy(track['obj']))
                    row[12] = dict(row[12], **{MEMORY_KEY: dict(source_stamp=track['stamp'],
                        sequence=track['sequence'], observations=track['observations'], age_s=age)})
                    retained.append(tuple(row))
            self._tracks = alive
            self.last_diagnostics = dict(retained_n=len(retained),
                confirmed_n=sum(t['observations'] >= 2 for t in alive),
                source_stamp=frame[0], source_sequence=frame[1], reason=None)
            return retained
        except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
            self.reset('invalid measured raw-target proof')
            return []
