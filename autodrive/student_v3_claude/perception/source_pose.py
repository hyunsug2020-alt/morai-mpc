"""Align sensor frames using measured poses on their shared timestamp clock."""
import math

import numpy as np


def measured_pose_at(history, stamp, *, nearest_limit=.03, bracket_limit=.2):
    """Return a bracketed pose or a nearby measured sample, never extrapolate.

    Native velocity uses simulator seconds; ROS arrival stamps use wall seconds.
    Multiplying those values cannot recover motion when real-time factor varies.
    """
    try:
        records = np.asarray(history, dtype=float)
        stamp = float(stamp)
        if (records.ndim != 2 or records.shape[1] != 6 or not len(records)
                or not np.isfinite(records).all() or not math.isfinite(stamp)
                or stamp <= 0. or np.any(records[:, 0] <= 0.)):
            return None
        records = records[np.argsort(records[:, 0], kind='stable')]
        times = records[:, 0]
        times, first, counts = np.unique(times, return_index=True, return_counts=True)
        selected = records[first+counts-1]
        usable = np.ones(len(times), bool)
        for index in np.flatnonzero(counts > 1):
            revisions = records[first[index]:first[index]+counts[index]]
            displacement = np.linalg.norm(revisions[:, 1:3]-selected[index, 1:3], axis=1)
            yaw = np.abs((revisions[:, 3]-selected[index, 3]+math.pi) % (2*math.pi)-math.pi)
            usable[index] = bool(np.max(displacement) <= .05 and np.max(yaw) <= math.radians(.5))
        # ESKF may refine yaw/speed at an unchanged stamp. Use the latest received
        # revision; an unrelated older conflict must not poison the whole queue.
        records = selected
        right = int(np.searchsorted(times, stamp))
        if right < len(records) and times[right] == stamp:
            return records[right].copy() if usable[right] else None
        if 0 < right < len(records):
            if not (usable[right-1] and usable[right]):
                return None
            before, after = records[right-1], records[right]
            span = float(after[0]-before[0])
            yaw = (after[3]-before[3]+math.pi) % (2*math.pi)-math.pi
            travel = float(np.linalg.norm(after[1:3]-before[1:3]))
            if (span <= bracket_limit and travel <= max(abs(before[4]), abs(after[4]))*span+.25
                    and abs(yaw) <= 1.5*span+.03):
                fraction = (stamp-before[0])/span
                pose = before+(after-before)*fraction
                pose[3] = before[3]+yaw*fraction
                pose[0] = stamp
                return pose
            return None
        index = int(np.argmin(np.abs(times-stamp)))
        nearest = records[index]
        return nearest.copy() if usable[index] and abs(nearest[0]-stamp) <= nearest_limit else None
    except (TypeError, ValueError, OverflowError, IndexError):
        return None


def align_cloud_to_pose(points, cloud_stamp, target_pose, history):
    """Rigid cloud-pose -> world -> target-pose transform; preserve every point."""
    try:
        values = np.asarray(points)
        target = np.asarray(target_pose, dtype=float)
        source = measured_pose_at(history, cloud_stamp)
        if (source is None or target.shape != (6,) or not np.isfinite(target).all()
                or target[0] <= 0. or abs(target[0]-float(cloud_stamp)) > .3
                or values.ndim != 2 or values.shape[1] != 3 or values.dtype.kind != 'f'
                or not np.isfinite(values).all()):
            return None
        target_observation = measured_pose_at(history, target[0])
        if (target_observation is None
                or np.linalg.norm(target[1:3]-target_observation[1:3]) > .05
                or abs((target[3]-target_observation[3]+math.pi) % (2*math.pi)-math.pi)
                    > math.radians(.5)):
            return None
        # Validate target revisions as well, while retaining the caller's exact
        # target frame so subsequent local-to-world publication uses that frame.
        elapsed = abs(float(target[0]-source[0]))
        yaw_delta = abs((target[3]-source[3]+math.pi) % (2*math.pi)-math.pi)
        if (np.linalg.norm(target[1:3]-source[1:3]) >
                max(abs(source[4]), abs(target[4]))*elapsed+.25
                or yaw_delta > 1.5*elapsed+.1):
            return None
        c, s = math.cos(source[3]), math.sin(source[3])
        world = values[:, :2] @ np.array([[c, s], [-s, c]])+source[1:3]
        c, s = math.cos(target[3]), math.sin(target[3])
        local = (world-target[1:3]) @ np.array([[c, -s], [s, c]])
        return np.column_stack((local, values[:, 2])).astype(values.dtype)
    except (TypeError, ValueError, OverflowError, IndexError):
        return None
