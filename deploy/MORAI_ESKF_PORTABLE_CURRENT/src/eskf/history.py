import copy
from dataclasses import dataclass

import numpy as np


@dataclass
class ImuReplayEvent:
    dt: float
    acceleration_body: np.ndarray
    yaw_rate: float
    yaw_measurement: object = None
    yaw_variance: object = None
    apply_nhc: bool = False
    apply_zupt: bool = False

    def __post_init__(self):
        self.dt = float(self.dt)
        self.acceleration_body = np.asarray(
            self.acceleration_body, dtype=float).copy()
        self.yaw_rate = float(self.yaw_rate)
        if self.yaw_measurement is not None:
            self.yaw_measurement = float(self.yaw_measurement)
        if self.yaw_variance is not None:
            self.yaw_variance = float(self.yaw_variance)


@dataclass
class SpeedReplayEvent:
    forward_speed: float
    variance: float

    def __post_init__(self):
        self.forward_speed = float(self.forward_speed)
        self.variance = float(self.variance)


@dataclass
class HistoryFrame:
    stamp: float
    event: object
    snapshot_after: dict


@dataclass
class ReplayResult:
    attempted: bool
    accepted: bool
    replayed_events: int = 0
    reason: str = ""
    update_result: object = None


class FixedLagHistory:
    """IMU journal for bounded delayed-measurement rewind/replay.

    A barrier is installed after every non-IMU correction. Measurements older
    than that barrier are deliberately not replayed, preventing a delayed GPS
    fix from erasing wheel or SLAM corrections that are not in this journal.
    """

    def __init__(self, max_age=1.5, min_delay=0.02, max_events=1000):
        self.max_age = max(0.0, float(max_age))
        self.min_delay = max(0.0, float(min_delay))
        self.max_events = max(1, int(max_events))
        self.base_stamp = None
        self.base_snapshot = None
        self.frames = []
        self.replay_count = 0
        self.replayed_events = 0
        self.fallback_count = 0
        self.last_reason = "not_initialized"

    @staticmethod
    def _copy_snapshot(snapshot):
        return copy.deepcopy(snapshot)

    def seed(self, stamp, snapshot):
        self.base_stamp = float(stamp)
        self.base_snapshot = self._copy_snapshot(snapshot)
        self.frames = []
        self.last_reason = "seeded"

    def mark_barrier(self, stamp, snapshot):
        self.seed(stamp, snapshot)
        self.last_reason = "external_correction_barrier"

    def record(self, stamp, event, snapshot_after):
        stamp = float(stamp)
        if self.base_snapshot is None:
            self.seed(stamp, snapshot_after)
            return
        if stamp < self.base_stamp:
            return
        if self.frames and stamp < self.frames[-1].stamp:
            return
        self.frames.append(HistoryFrame(
            stamp, copy.deepcopy(event), self._copy_snapshot(snapshot_after)))
        cutoff = stamp - self.max_age
        while self.frames and (
                len(self.frames) > self.max_events
                or self.frames[0].stamp < cutoff):
            expired = self.frames.pop(0)
            self.base_stamp = expired.stamp
            self.base_snapshot = self._copy_snapshot(
                expired.snapshot_after)

    def can_replay(self, measurement_stamp, current_stamp):
        measurement_stamp = float(measurement_stamp)
        current_stamp = float(current_stamp)
        lag = current_stamp - measurement_stamp
        if self.base_snapshot is None:
            return False, "not_initialized"
        if lag < self.min_delay:
            return False, "not_delayed"
        if lag > self.max_age:
            return False, "too_old"
        if measurement_stamp < self.base_stamp - 1e-9:
            return False, "before_external_correction_barrier"
        return True, "ready"

    @staticmethod
    def _accepted(update_result):
        if isinstance(update_result, tuple):
            return bool(update_result[0])
        return bool(update_result)

    def replay(self, measurement_stamp, current_stamp, snapshot,
               restore, apply_measurement, apply_event):
        possible, reason = self.can_replay(
            measurement_stamp, current_stamp)
        if not possible:
            self.fallback_count += 1
            self.last_reason = reason
            return ReplayResult(False, False, reason=reason)

        measurement_stamp = float(measurement_stamp)
        current_snapshot = self._copy_snapshot(snapshot())
        past_frames = [
            frame for frame in self.frames
            if frame.stamp <= measurement_stamp]
        future_frames = [
            frame for frame in self.frames
            if frame.stamp > measurement_stamp]
        base_snapshot = (
            past_frames[-1].snapshot_after
            if past_frames else self.base_snapshot)
        restore(self._copy_snapshot(base_snapshot))
        update_result = apply_measurement()
        if not self._accepted(update_result):
            restore(current_snapshot)
            self.last_reason = "measurement_rejected"
            return ReplayResult(
                True, False, reason=self.last_reason,
                update_result=update_result)

        self.base_stamp = measurement_stamp
        self.base_snapshot = self._copy_snapshot(snapshot())
        rebuilt = []
        for frame in future_frames:
            apply_event(copy.deepcopy(frame.event))
            rebuilt.append(HistoryFrame(
                frame.stamp,
                copy.deepcopy(frame.event),
                self._copy_snapshot(snapshot())))
        self.frames = rebuilt
        self.replay_count += 1
        self.replayed_events += len(rebuilt)
        self.last_reason = "replayed"
        return ReplayResult(
            True, True, replayed_events=len(rebuilt),
            reason=self.last_reason, update_result=update_result)

    def diagnostics(self):
        return {
            "gps_replay_count": self.replay_count,
            "gps_replayed_imu_events": self.replayed_events,
            "gps_replay_fallback_count": self.fallback_count,
            "gps_replay_buffer_events": len(self.frames),
            "gps_replay_last_reason": self.last_reason,
        }
