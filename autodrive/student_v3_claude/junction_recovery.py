"""Restore an observed inside-junction state without granting an entry gap.

The bootstrap can be selected only by the first fresh runtime pose. Every
positive continuation still needs the publisher's full NPC/body/person check.
"""
import hashlib
import math
import threading

import numpy as np

from ego_body import body_corners


def _pose(ego, now):
    values = np.asarray([ego[k] for k in ('x', 'y', 'heading_deg', 'speed_kmh', 'stamp', 'age_s')], float)
    reported_age = float(values[-1])
    receipt_age = float(now)-float(ego.get('received', now))
    age = max(reported_age, receipt_age)
    if not (np.isfinite(values).all() and math.isfinite(receipt_age) and values[4] > 0.
            and 0. <= reported_age <= .2 and 0. <= receipt_age <= .2 and float(now) >= 0.):
        raise ValueError('recovery pose unavailable')
    return values, age


def _digest(path):
    points = np.asarray(path, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 2 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError('recovery path unavailable')
    return hashlib.sha256(np.ascontiguousarray(points[:, :2]).tobytes()).hexdigest()[:16]


def _pose_digest(ego):
    values = np.asarray([ego[key] for key in ('x', 'y', 'heading_deg', 'speed_kmh', 'stamp')], float)
    return hashlib.sha256(values.tobytes()).hexdigest()[:16]


def _route_path(path, ego, lanes):
    """A startup continuation follows this route, not a second crossing branch."""
    points = np.asarray(path, float)[:, :2]
    route = np.asarray(lanes.route_xy, float)
    if route.ndim != 2 or route.shape[1] != 2 or len(route) < 3 or not np.isfinite(route).all():
        raise ValueError('recovery route unavailable')
    _, nearest = lanes.route_tree.query(points, k=min(2, len(route)))
    nearest = np.asarray(nearest).reshape(len(points), -1)
    segments = np.clip(np.concatenate((nearest-1, nearest), axis=1), 0, len(route)-2)
    starts, delta = route[segments], route[segments+1]-route[segments]
    length_sq = np.maximum(1e-12, np.sum(delta*delta, axis=2))
    fraction = np.clip(np.sum((points[:, None, :]-starts)*delta, axis=2)/length_sq, 0., 1.)
    projected = starts+fraction[:, :, None]*delta
    distances = np.sum((projected-points[:, None, :])**2, axis=2)
    best = np.argmin(distances, axis=1)
    if np.max(np.sqrt(distances[np.arange(len(points)), best])) > .5:
        raise ValueError('recovery path leaves the route')
    tangent = delta[np.arange(len(points)), best]
    moves = np.diff(points, axis=0)
    alignment = np.sum(moves*tangent[:-1], axis=1)
    norms = np.linalg.norm(moves, axis=1)*np.linalg.norm(tangent[:-1], axis=1)
    if np.any((norms > 1e-8) & (alignment < .5*norms)):
        raise ValueError('recovery path heading differs from the route')
    from ring_entry import PathFrame
    _, lateral, _ = PathFrame(points).project([[ego['x'], ego['y']]])
    if abs(float(lateral[0])) > .5:
        raise ValueError('recovery path misses the actual pose')


def _geometry(ego, lanes, line, region, *, initial):
    xy = np.asarray([ego['x'], ego['y']], float)
    route = np.asarray(lanes.route_xy, float)
    if route.ndim != 2 or route.shape[1] != 2 or len(route) < 3 or not np.isfinite(route).all():
        raise ValueError('recovery route unavailable')
    _, nearby = lanes.route_tree.query(xy, k=min(8, len(route)))
    segments = np.unique(np.clip(np.r_[np.atleast_1d(nearby)-1, np.atleast_1d(nearby)], 0, len(route)-2))
    station, distance, segment = lanes._project_route(xy, segments)
    tangent = route[segment+1]-route[segment]
    yaw = math.radians(float(ego['heading_deg']))
    heading_error = abs((yaw-math.atan2(tangent[1], tangent[0])+math.pi) % (2*math.pi)-math.pi)
    corners = body_corners(*xy, yaw)
    stations = [float(lanes.route_progress(dict(x=float(p[0]), y=float(p[1])))) for p in corners]
    upper = float(line['station_m'])+12. if initial else float(region['end_station_m'])
    if not (np.isfinite(stations).all() and math.isfinite(station) and distance <= .5
            and heading_error <= math.radians(6. if initial else 60.)
            and float(region['start_station_m']) <= station <= upper
            and min(stations) > float(line['station_m'])+.5):
        raise ValueError('recovery outside occupied route')
    return dict(station_m=station, corner_stations_m=stations,
                heading_error_deg=math.degrees(heading_error))


class JunctionRecovery:
    """One runtime bootstrap and source-bound continuation of that same region."""
    def __init__(self):
        self._lock = threading.RLock()
        self.seen_start = False
        self.active = False
        self.line = self.region = self.last_pose = None
        self._controller = None
        self.last_now = None
        self.reason = 'runtime bootstrap pending'

    def _cancel(self, reason):
        self.active = False
        self.line = self.region = None
        self._controller = None
        self.reason = reason

    def observe(self, ego, controller, lanes, regions, now):
        """Record the first fresh pose before any stop-controller update.

        A later teleport into the junction cannot create a second bootstrap.
        Missing this object in an older/test runtime disables recovery.
        """
        with self._lock:
            try:
                values, _ = _pose(ego, now)
            except (KeyError, TypeError, ValueError, OverflowError):
                return False
            if not self.seen_start:
                self.seen_start = True
                self.reason = 'runtime did not start fully inside the junction'
                if (controller is None or not controller.yield_only or controller.active is not None
                        or controller.last_station is not None or controller.released):
                    return False
                for region in regions:
                    for line in controller.lines:
                        if (line['id'] != region.get('line_id') or line['id'] in controller.completed
                                or line['node_id'] != region.get('node_id')):
                            continue
                        try:
                            _geometry(ego, lanes, line, region, initial=True)
                        except (KeyError, TypeError, ValueError, OverflowError, AttributeError):
                            continue
                        self.line, self.region = dict(line), dict(region)
                        self._controller = controller
                        self.last_pose, self.last_now = values, float(now)
                        self.reason = 'cold inside-junction candidate'
                        return True
                return False
            if self.line is None:
                return False
            dt = float(values[4]-self.last_pose[4])
            distance = float(np.linalg.norm(values[:2]-self.last_pose[:2]))
            speed = max(abs(values[3]), abs(self.last_pose[3]))/3.6
            yaw = abs((math.radians(values[2]-self.last_pose[2])+math.pi) % (2*math.pi)-math.pi)
            same = dt > 1e-6 or (distance <= 1e-6 and abs(values[3]-self.last_pose[3]) <= 1e-6
                                 and yaw <= math.radians(.001))
            if (not 0. <= dt <= 1. or float(now) < self.last_now or not same
                    or distance > speed*dt+.25 or yaw > 1.5*dt+.1):
                self._cancel('recovery pose discontinuity')
                return False
            self.last_pose, self.last_now = values, float(now)
            station = float(lanes.route_progress(ego))
            if station > float(self.region['end_station_m']):
                self._cancel('recovery region complete')
                return False
            return True

    def context(self, ego, controller, lanes, regions, now, *, observe_pose=True):
        """A provisional GUI mode; this grants no publisher permission."""
        if observe_pose and not self.observe(ego, controller, lanes, regions, now):
            return None
        with self._lock:
            try:
                _pose(ego, now)
                if self.line is None:
                    return None
                if controller is not self._controller:
                    self._cancel('recovery controller replaced')
                    return None
                _geometry(ego, lanes, self.line, self.region, initial=not self.active)
            except (KeyError, TypeError, ValueError, OverflowError, AttributeError):
                return None
            return dict(line_id=self.line['id'], phase='RECOVERY_FOLLOW', active=self.active)

    def prepare(self, ego, world_path, controller, lanes, regions, now, *, ready, source_ages):
        """Bind the provisional mode to this call's actual path, pose and sources."""
        context = self.context(ego, controller, lanes, regions, now)
        if context is None or not ready:
            return None
        with self._lock:
            try:
                _, pose_age = _pose(ego, now)
                ages = np.asarray(source_ages, float)
                if not len(ages) or not np.isfinite(ages).all() or np.min(ages) < 0. or np.max(ages) > .35:
                    return None
                geometry = _geometry(ego, lanes, self.line, self.region, initial=not self.active)
                digest = _digest(world_path)
                _route_path(world_path, ego, lanes)
                return dict(context, **geometry, checked_at=float(now), pose_stamp=float(ego['stamp']),
                    path_digest=digest, pose_digest=_pose_digest(ego),
                    expires_at=min(float(now)+.2-pose_age, float(now)+.35-float(np.max(ages))))
            except (KeyError, TypeError, ValueError, OverflowError, AttributeError):
                return None

    def matches(self, proof, ego, world_path, now):
        with self._lock:
            try:
                return bool(self.line is not None and proof['line_id'] == self.line['id']
                    and proof['path_digest'] == _digest(world_path)
                    and float(proof['pose_stamp']) == float(ego['stamp'])
                    and proof['pose_digest'] == _pose_digest(ego)
                    and math.isfinite(float(proof['expires_at']))
                    and float(proof['checked_at']) <= float(now) <= float(proof['expires_at']))
            except (KeyError, TypeError, ValueError, OverflowError):
                return False

    def accept(self, proof, ego, world_path, controller, now):
        """Release only controller bookkeeping after the full live guard is clear."""
        with self._lock:
            if not self.matches(proof, ego, world_path, now):
                return False
            if not self.active:
                if not controller.consume_inside_recovery(self.line['id'], self.line['station_m'],
                        proof['station_m'], checked_at=proof['checked_at'], expires_at=proof['expires_at']):
                    return False
                self.active = True
                self.reason = 'observed inside-junction continuation'
            return True
