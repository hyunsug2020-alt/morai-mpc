"""Tunnel-only measured occupancy constraints, with a timestamped ROS protocol.

Geometry is supplied by the existing tunnel wall matcher. This module neither
reads LiDAR scans nor estimates another road plane. Boxed model paths can be
checked against exactly that geometry in the requested current vehicle frame.
These occupied constraints do not replace the existing visibility/braking gate.
"""
from dataclasses import dataclass, field
import json
import math
import threading
import time

import numpy as np
from scipy.spatial import cKDTree

from ego_body import EGO_FRONT_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M, body_centres


TOPIC = '/perception/no_go_areas'
FRAME = 'tunnel_vehicle'
MAP_FRAME = 'map'
MAX_AGE_S = .25
MAX_POINTS = 20000
MAX_WALLS = 256


def _pose(value):
    if not isinstance(value, dict):
        raise ValueError('missing geometry pose')
    pose = {key: float(value[key]) for key in ('x', 'y', 'heading_deg', 'stamp')}
    if not np.isfinite(list(pose.values())).all() or pose['stamp'] <= 0.:
        raise ValueError('invalid geometry pose')
    if value.get('frame_id', MAP_FRAME) != MAP_FRAME:
        raise ValueError('geometry pose map frame mismatch')
    if 'speed_kmh' in value:
        speed = float(value['speed_kmh'])
        if not math.isfinite(speed) or not 0. <= speed <= 200.:
            raise ValueError('invalid geometry pose speed')
        pose['speed_kmh'] = speed
    return pose


def _geometry(value, shape, maximum):
    data = np.asarray(value, dtype=float)
    if data.size == 0:
        data = np.empty((0,)+shape)
    if data.ndim != 1+len(shape) or data.shape[1:] != shape:
        raise ValueError('geometry shape mismatch')
    if len(data) > maximum or not np.isfinite(data).all():
        raise ValueError('invalid geometry coordinates')
    data = np.array(data, copy=True)
    data.setflags(write=False)
    return data


def make_record(occupied_points, walls, frame_pose, stamp, active,
                valid=True, sequence=1):
    """Make one wire record; all active geometry is local to ``frame_pose``.

    ``stamp`` is the actual LiDAR source time. ``frame_pose.stamp`` is the pose
    time at which motion-compensated points and finite physical wall segments
    are expressed. The ESKF/map position may have been wall-aligned already.
    """
    if type(active) is not bool or type(valid) is not bool:
        raise ValueError('active and valid must be booleans')
    if type(sequence) is not int or sequence < 1:
        raise ValueError('invalid no-go sequence')
    record = dict(schema=1, frame=FRAME, map_frame=MAP_FRAME, active=active,
                  valid=bool(valid and active), sequence=sequence,
                  stamp=0., pose=None, occupied_points=[], walls=[])
    if not active or not valid:
        return record
    source = float(stamp)
    pose = _pose(frame_pose)
    if not math.isfinite(source) or source <= 0. or abs(pose['stamp']-source) > MAX_AGE_S:
        raise ValueError('scan and geometry pose time mismatch')
    points = _geometry(occupied_points, (2,), MAX_POINTS)
    segments = _geometry(walls, (2, 2), MAX_WALLS)
    if not len(segments) or np.any(np.linalg.norm(segments[:, 1]-segments[:, 0], axis=1) <= 1e-6):
        raise ValueError('finite physical tunnel walls required')
    record.update(stamp=source, pose=pose, occupied_points=points.tolist(), walls=segments.tolist())
    return record


def _decode(record):
    if isinstance(record, str):
        if len(record) > 4*1024*1024:
            raise ValueError('no-go payload too large')
        record = json.loads(record)
    if not isinstance(record, dict) or type(record.get('schema')) is not int or record['schema'] != 1:
        raise ValueError('unsupported no-go schema')
    if record.get('frame') != FRAME or record.get('map_frame') != MAP_FRAME:
        raise ValueError('no-go frame mismatch')
    # Validate wire arrays once. Reconstructing a publishable list record here
    # would allocate another complete points list before copying it back to
    # numpy on every scan.
    active, valid, sequence = record.get('active'), record.get('valid'), record.get('sequence')
    if type(active) is not bool or type(valid) is not bool or type(sequence) is not int or sequence < 1:
        raise ValueError('invalid no-go flags or sequence')
    normalized = dict(schema=1, frame=FRAME, map_frame=MAP_FRAME, active=active,
                      valid=bool(valid and active), sequence=sequence,
                      stamp=0., pose=None, occupied_points=[], walls=[])
    if normalized['valid']:
        source, pose = float(record['stamp']), _pose(record['pose'])
        if not math.isfinite(source) or source <= 0. or abs(pose['stamp']-source) > MAX_AGE_S:
            raise ValueError('scan and geometry pose time mismatch')
        points = _geometry(record['occupied_points'], (2,), MAX_POINTS)
        walls = _geometry(record['walls'], (2, 2), MAX_WALLS)
        if not len(walls) or np.any(np.linalg.norm(walls[:, 1]-walls[:, 0], axis=1) <= 1e-6):
            raise ValueError('finite physical tunnel walls required')
        normalized.update(stamp=source, pose=pose, occupied_points=points, walls=walls)
    return normalized


def _rotation(pose):
    angle = math.radians(pose['heading_deg'])
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[c, -s], [s, c]])


def _transform(points, old_pose, new_pose):
    shape = points.shape
    world = points.reshape(-1, 2) @ _rotation(old_pose).T + [old_pose['x'], old_pose['y']]
    result = ((world-[new_pose['x'], new_pose['y']]) @ _rotation(new_pose)).reshape(shape)
    result.setflags(write=False)
    return result


def _path_samples(path, horizon):
    data = np.asarray(path, dtype=float)
    if data.ndim != 2 or data.shape[1] < 2 or len(data) < 2 or not np.isfinite(data).all():
        return None
    data = data[:, :2]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(data, axis=0), axis=1))]
    arc, unique = np.unique(arc, return_index=True)
    if len(arc) < 2 or arc[-1] <= 1e-6:
        return None
    end = min(horizon, float(arc[-1]))
    sample = np.unique(np.r_[np.arange(0., end, .25), end])
    if len(sample) < 2:
        return None
    centres = np.column_stack([np.interp(sample, arc, data[unique, axis]) for axis in (0, 1)])
    tangent = np.gradient(centres, axis=0)
    magnitude = np.linalg.norm(tangent, axis=1)
    if np.any(magnitude < 1e-9):
        return None
    tangent /= magnitude[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    return sample, centres, tangent, normal


def _wall_hits(centres, tangent, normal, walls, half_length, half_width):
    """Continuous finite-segment / oriented rectangle intersection (slabs)."""
    delta = walls[None, :, :, :]-centres[:, None, None, :]
    local = np.stack((np.sum(delta*tangent[:, None, None, :], axis=-1),
                      np.sum(delta*normal[:, None, None, :], axis=-1)), axis=-1)
    start, direction = local[:, :, 0], local[:, :, 1]-local[:, :, 0]
    extent = np.column_stack((np.broadcast_to(half_length, len(centres)),
                              np.broadcast_to(half_width, len(centres))))[:, None, :]
    parallel = np.abs(direction) < 1e-12
    outside = (parallel & ((start < -extent) | (start > extent))).any(axis=-1)
    lower = np.full(direction.shape, -np.inf)
    upper = np.full(direction.shape, np.inf)
    first = np.divide(-extent-start, direction, out=np.zeros_like(direction), where=~parallel)
    second = np.divide(extent-start, direction, out=np.zeros_like(direction), where=~parallel)
    lower[~parallel] = np.minimum(first, second)[~parallel]
    upper[~parallel] = np.maximum(first, second)[~parallel]
    enter = np.maximum(0., lower.max(axis=-1))
    leave = np.minimum(1., upper.min(axis=-1))
    return ((enter <= leave+1e-10) & ~outside).any(axis=1)


@dataclass(frozen=True)
class NoGoSnapshot:
    valid: bool
    active: bool
    reason: str
    source_stamp: float = 0.
    pose_stamp: float = 0.
    sequence: int = 0
    age_s: float = math.inf
    occupied_points: object = None
    walls: object = None
    _tree: object = field(default=None, init=False, repr=False, compare=False)

    def __post_init__(self):
        if self.valid and self.occupied_points is not None and len(self.occupied_points):
            object.__setattr__(self, '_tree', cKDTree(self.occupied_points))

    def get(self, key, default=None):
        return getattr(self, key, default)

    def __getitem__(self, key):
        return getattr(self, key)

    def clear_distance(self, path, horizon_m=60., margin=.25):
        """First body collision distance; invalid evidence grants zero distance.

        All front/rear/side points use the competition 4.635m x 1.892m body.
        Finite wall endpoints stay open: no wall is extended beyond the exit.
        Absence of occupied points is not proof of unseen free space; callers
        must retain the independent raw-scan visibility and braking checks.
        """
        if not self.valid or not self.active:
            return 0.
        try:
            horizon, margin = float(horizon_m), float(margin)
            if not math.isfinite(horizon) or horizon <= 0. or not math.isfinite(margin) or not 0. <= margin <= 1.:
                return 0.
            sampled = _path_samples(path, horizon)
            if sampled is None:
                return 0.
            sample, centres, tangent, normal = sampled
            # Path samples are rear-axle positions: the body is centred EGO_CENTRE_M
            # ahead along the heading there and turns about the rear axle (ego_body).
            centres = body_centres(centres, tangent)
            # Bound translation and footprint-corner rotation between checks,
            # including sharply turning candidates. Half a sample alone does
            # not enclose the outside corner during a tight turn.
            angle = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
            turns = np.abs(np.diff(angle))
            turn_bound = np.maximum(np.r_[turns[0], turns], np.r_[turns, turns[-1]])
            reserve = .125 + 2.*math.hypot(EGO_FRONT_M+margin, EGO_HALF_WIDTH_M+margin)*np.sin(
                np.minimum(math.pi, turn_bound)/4.)
            half_length, half_width = EGO_HALF_LENGTH_M+margin+reserve, EGO_HALF_WIDTH_M+margin+reserve
            hits = _wall_hits(centres, tangent, normal, self.walls, half_length, half_width)
            points = self.occupied_points
            if len(points):
                first_wall = np.flatnonzero(hits)
                last = int(first_wall[0])+1 if len(first_wall) else len(centres)
                if len(first_wall) and first_wall[0] == 0:
                    return 0.
                # Stop after the first occupied body, keeping large point
                # lists off later poses that cannot change the answer.
                for begin in range(0, last, 48):
                    end = min(last, begin+48)
                    nearby = self._tree.query_ball_point(centres[begin:end],
                                                        np.hypot(half_length[begin:end], half_width[begin:end]))
                    counts = np.asarray([len(ids) for ids in nearby])
                    if counts.sum():
                        stations = np.repeat(np.arange(begin, end), counts)
                        ids = np.concatenate([ids for ids in nearby if len(ids)])
                        delta = points[ids]-centres[stations]
                        along = np.sum(delta*tangent[stations], axis=1)
                        lateral = np.sum(delta*normal[stations], axis=1)
                        hit = (abs(along) <= half_length[stations]) & (abs(lateral) <= half_width[stations])
                        if hit.any():
                            return float(sample[int(stations[hit].min())])
            first = np.flatnonzero(hits)
            return float(sample[first[0]]) if len(first) else float(sample[-1])
        except (TypeError, ValueError, IndexError, FloatingPointError):
            return 0.

    def path_clear(self, path, horizon_m=None, margin=.25):
        if not self.valid or not self.active:
            return False
        try:
            data = np.asarray(path, dtype=float)
            length = float(np.linalg.norm(np.diff(data[:, :2], axis=0), axis=1).sum())
            horizon = length if horizon_m is None else float(horizon_m)
            if not math.isfinite(length) or length <= 0. or not math.isfinite(horizon) or horizon <= 0.:
                return False
            clearance = self.clear_distance(data, horizon, margin)
            return bool(clearance > 0. and clearance >= min(length, horizon)-1e-6)
        except (TypeError, ValueError, IndexError):
            return False


class TunnelNoGoReceiver:
    """ROS subscriber or offline wire consumer, using the same validation path."""
    def __init__(self, topic=TOPIC, max_age_s=MAX_AGE_S, clock=None, source_clock=None):
        self.max_age_s = float(max_age_s)
        if not 0. < self.max_age_s <= MAX_AGE_S:
            raise ValueError('no-go freshness must be at most .25 seconds')
        self.clock = clock or time.monotonic
        self.source_clock = source_clock or time.time
        self.lock = threading.Lock()
        self.record, self.received, self.error, self.subscriber = None, 0., 'no tunnel occupancy', None
        if topic is not None:
            import rospy
            from std_msgs.msg import String
            self.source_clock = source_clock or rospy.get_time
            self.subscriber = rospy.Subscriber(topic, String, self._on_message, queue_size=1)

    def _on_message(self, message):
        self.ingest(message.data)

    def ingest(self, record):
        try:
            normalized = _decode(record)
            with self.lock:
                previous = self.record
                if (normalized['active'] and previous is not None and previous['active'] and
                        (normalized['sequence'] <= previous['sequence'] or
                         (normalized['valid'] and previous['valid'] and
                          normalized['stamp'] < previous['stamp']-1e-6))):
                    raise ValueError('no-go sequence or source time reversed')
                self.record, self.received, self.error = normalized, self.clock(), None
            return True
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            with self.lock:
                self.record, self.error = None, str(exc)
            return False

    def snapshot(self, ego):
        with self.lock:
            record, received, error = self.record, self.received, self.error
        if record is None:
            return NoGoSnapshot(False, False, error or 'no tunnel occupancy')
        if not record['active']:
            return NoGoSnapshot(False, False, 'tunnel inactive', sequence=record['sequence'])
        if not record['valid']:
            return NoGoSnapshot(False, True, 'invalid tunnel occupancy', sequence=record['sequence'])
        try:
            pose = _pose(ego)
            now = float(self.source_clock())
            receipt_age = float(self.clock()-received)
            scan_age, pose_age = now-record['stamp'], now-record['pose']['stamp']
            if not all(math.isfinite(age) for age in (now, receipt_age, scan_age, pose_age)):
                raise ValueError('invalid no-go clock')
            if not 0. <= receipt_age <= self.max_age_s:
                raise ValueError('tunnel occupancy receipt stale')
            if not -.02 <= scan_age <= self.max_age_s:
                raise ValueError('tunnel occupancy source stale or future')
            if not -.02 <= pose_age <= self.max_age_s or not -.02 <= now-pose['stamp'] <= self.max_age_s:
                raise ValueError('tunnel occupancy/current pose stale or future')
            if abs(pose['stamp']-record['pose']['stamp']) > self.max_age_s:
                raise ValueError('tunnel occupancy ego time mismatch')
            elapsed = abs(pose['stamp']-record['pose']['stamp'])
            speed = max(pose.get('speed_kmh', 0.), record['pose'].get('speed_kmh', 0.))/3.6
            displacement = math.hypot(pose['x']-record['pose']['x'], pose['y']-record['pose']['y'])
            yaw_delta = abs((pose['heading_deg']-record['pose']['heading_deg']+180.) % 360.-180.)
            # A still-fresh packet from before a reset must not become an
            # empty corridor far away. Permit bounded wall corrections and
            # measured short motion, rejecting teleports or frame swaps.
            if displacement > 1.+max(3., speed)*elapsed*1.5 or yaw_delta > 5.+90.*elapsed:
                raise ValueError('tunnel occupancy ego pose jump')
            points = _transform(record['occupied_points'], record['pose'], pose)
            walls = _transform(record['walls'], record['pose'], pose)
            return NoGoSnapshot(True, True, 'fresh tunnel occupancy', record['stamp'],
                record['pose']['stamp'], record['sequence'], max(receipt_age, scan_age, pose_age), points, walls)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            return NoGoSnapshot(False, True, str(exc), sequence=record['sequence'])

    def close(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None


class TunnelNoGoPublisher:
    """Publish existing wall-match geometry, including explicit tunnel leave."""
    def __init__(self, topic=TOPIC):
        self.sequence, self.publisher, self.message_type = 0, None, None
        if topic is not None:
            import rospy
            from std_msgs.msg import String
            self.publisher = rospy.Publisher(topic, String, queue_size=1)
            self.message_type = String

    def update(self, occupied_points, walls, frame_pose, stamp, active, valid=True):
        self.sequence += 1
        try:
            record = make_record(occupied_points, walls, frame_pose, stamp, active, valid, self.sequence)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            record = make_record([], [], None, 0., bool(active), False, self.sequence)
            record['reason'] = str(exc)
        if self.publisher is not None:
            self.publisher.publish(self.message_type(data=json.dumps(record, separators=(',', ':'), allow_nan=False)))
        return record

    def close(self):
        if self.publisher is not None:
            self.publisher.unregister()
            self.publisher = None
