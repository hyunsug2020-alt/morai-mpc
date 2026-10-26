"""Rigid alignment of one measured road scan without renewing its source time."""
from __future__ import annotations

import math
import time

import numpy as np

MAX_ROAD_SCAN_AGE_S = .3


def _pose(value):
    if (not isinstance(value, dict) or value.get('measured') is not True
            or value.get('src') == 'file'):
        raise ValueError('measured ROS pose unavailable')
    pose = np.asarray([value['stamp'], value['x'], value['y'], value['heading_deg']], float)
    if not np.isfinite(pose).all() or pose[0] <= 0.:
        raise ValueError('invalid measured ROS pose')
    return pose


def align_road_scan(scan, ego, pose_at_stamp, *, now_wall, now_mono, captured_at):
    """Return aligned XY and a certificate for the unchanged atomic scan.

    The pose callback must provide a measured pose at the exact source stamp.
    A missing bracket may be retried later, but neither retry nor transform
    renews this scan's stamp, sequence or receipt age. No velocity/RTF-based
    extrapolation or point removal is allowed here.
    """
    proof = dict(valid=False, reason='road scan unavailable')
    try:
        stamp, age = float(scan['stamp']), float(scan['age_s'])
        sequence = int(scan['sequence'])
        now_wall, now_mono, captured_at = map(float, (now_wall, now_mono, captured_at))
        if (not np.isfinite([stamp, age, now_wall, now_mono, captured_at]).all()
                or stamp <= 0. or age < 0. or sequence < 0. or now_mono < captured_at):
            raise ValueError('road scan time invalid')
        effective_age = age+now_mono-captured_at
        wall_age = now_wall-stamp
        proof.update(stamp=stamp, sequence=sequence, source_age_s=age,
                     age_s=effective_age, wall_age_s=wall_age)
        if not 0. <= wall_age <= MAX_ROAD_SCAN_AGE_S or effective_age > MAX_ROAD_SCAN_AGE_S:
            raise ValueError('road scan source stale')
        points = np.asarray(scan['obstacle_points'], float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            raise ValueError('road scan points invalid')
        target = _pose(ego)
        if not 0. <= now_wall-target[0] <= MAX_ROAD_SCAN_AGE_S:
            raise ValueError('road scan target pose stale')
        source_value = pose_at_stamp(stamp)
        if source_value is None:
            proof['awaiting_source_pose'] = True
            raise ValueError('road scan source pose unavailable')
        source = _pose(source_value)
        if abs(source[0]-stamp) > 1e-9:
            raise ValueError('road scan source pose stamp mismatch')
        target_value = pose_at_stamp(float(target[0]))
        if target_value is None:
            raise ValueError('road scan target measurement unavailable')
        target_observation = _pose(target_value)
        target_yaw_error = abs((math.radians(target[3]-target_observation[3])+math.pi)
                               % (2.*math.pi)-math.pi)
        if (abs(target_observation[0]-target[0]) > 1e-9
                or np.linalg.norm(target[1:3]-target_observation[1:3]) > .05
                or target_yaw_error > math.radians(.5)):
            raise ValueError('road scan target measurement mismatch')
        speeds = np.asarray([source_value.get('speed_kmh', 0.), ego.get('speed_kmh', 0.)], float)/3.6
        elapsed = abs(target[0]-source[0])
        yaw_delta = abs((math.radians(target[3]-source[3])+math.pi) % (2.*math.pi)-math.pi)
        # This bounds discontinuities; velocity never supplies the transform.
        if (not np.isfinite(speeds).all()
                or np.linalg.norm(target[1:3]-source[1:3]) > max(abs(speeds))*elapsed+.25
                or yaw_delta > 1.5*elapsed+.1):
            raise ValueError('road scan measured pose discontinuity')
        source_yaw, target_yaw = np.radians([source[3], target[3]])
        c, s = math.cos(source_yaw), math.sin(source_yaw)
        with np.errstate(over='ignore', invalid='ignore'):
            world = points @ np.array([[c, s], [-s, c]])+source[1:3]
        c, s = math.cos(target_yaw), math.sin(target_yaw)
        with np.errstate(over='ignore', invalid='ignore'):
            aligned = (world-target[1:3]) @ np.array([[c, -s], [s, c]])
        if not np.isfinite(aligned).all():
            raise ValueError('road scan transform invalid')
        aligned.setflags(write=False)
        proof.update(valid=True, reason=None, source_pose=source.tolist(),
                     target_pose=target.tolist(), points_n=len(aligned))
        return dict(points=aligned, proof=proof)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError, IndexError) as error:
        proof['reason'] = str(error) if isinstance(error, ValueError) else 'road scan source pose unavailable'
        return dict(points=None, proof=proof)


def align_road_scan_wait(scan, ego, pose_at_stamp, *, captured_at, wait_s=.1):
    """Wait at most 100 ms for a just-arriving measured source bracket.

    Every retry checks the original atomic scan and accumulated age. A stale,
    malformed or discontinuous frame is never retried or replaced by a cache.
    """
    deadline = time.monotonic()+min(.1, max(0., float(wait_s)))
    while True:
        frame = align_road_scan(scan, ego, pose_at_stamp, now_wall=time.time(),
                               now_mono=time.monotonic(), captured_at=captured_at)
        remaining = deadline-time.monotonic()
        if not frame['proof'].get('awaiting_source_pose') or remaining <= 0.:
            return frame
        time.sleep(min(.002, remaining))


MAX_FRAME_AGE_S = .3


def road_scan_frame(proof, ego, now):
    if proof.get('valid') is not True or ego.get('measured') is not True:
        raise ValueError('unproved road source or pose')
    stamp = float(proof['stamp'])
    sequence = int(proof['sequence'])
    if isinstance(proof['sequence'], bool) or sequence != proof['sequence'] or sequence < 0:
        raise ValueError('invalid physical scan sequence')
    source = np.asarray(proof['source_pose'], float)
    target = np.asarray(proof['target_pose'], float)
    pose = np.asarray([ego['stamp'], ego['x'], ego['y'], ego['heading_deg']], float)
    if (source.shape != (4,) or target.shape != (4,) or
            not np.isfinite(np.r_[source, target, pose, now, stamp, proof['age_s']]).all() or
            stamp <= 0. or abs(source[0]-stamp) > 1e-9 or
            not 0. <= now-stamp <= MAX_FRAME_AGE_S or
            not 0. <= float(proof['age_s']) <= MAX_FRAME_AGE_S or
            not 0. <= now-target[0] <= MAX_FRAME_AGE_S or
            abs(target[0]-pose[0]) > 1e-9 or np.linalg.norm(target[1:3]-pose[1:3]) > .05 or
            abs((target[3]-pose[3]+180.) % 360.-180.) > .5):
        raise ValueError('invalid measured source geometry or age')
    return stamp, sequence
