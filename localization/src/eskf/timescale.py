"""Physics/wall clock-ratio estimators for MORAI Real Time Mode.

MORAI stamps every sensor message with the wall-clock arrival time, while
speeds, accelerations and yaw rates are physics quantities.  The ratio
``physics_seconds / wall_seconds`` (the simulator real-time factor) varied
between 0.41 and 0.95 on the development PC and changes by 0.15 within two
seconds.  A wrong ratio makes the filter integrate wheel speed too slowly or
too fast (run 257: 0.70 used while the simulator ran at 0.85 -> 14 m behind).

``GpsWheelTimeScale`` measures the ratio directly: GPS displacement over
about one second (wall time) divided by the wheel-speed/IMU-yaw dead-reckoned
displacement over the same wall interval.  Both inputs are independent of the
estimated state, so the estimate cannot be dragged by a diverging filter.
"""
import math
from collections import deque

import numpy as np


def _interp_angle(history, stamp):
    """Linear interpolation in an (stamp, unwrapped_angle) deque."""
    if not history:
        return None
    if stamp <= history[0][0]:
        return history[0][1]
    if stamp >= history[-1][0]:
        return history[-1][1]
    lo, hi = 0, len(history) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if history[mid][0] <= stamp:
            lo = mid
        else:
            hi = mid
    t0, a0 = history[lo]
    t1, a1 = history[hi]
    if t1 <= t0:
        return a1
    return a0 + (a1 - a0) * (stamp - t0) / (t1 - t0)


class GpsWheelTimeScale:
    def __init__(self, window_sec=1.0, min_window_sec=0.5, max_window_sec=3.0,
                 min_distance_m=3.0, alpha=0.35, minimum=0.05, maximum=2.0,
                 max_direction_residual=0.15, max_age_sec=3.0,
                 gps_lever=(0.0, 0.0)):
        self.window_sec = float(window_sec)
        self.min_window_sec = float(min_window_sec)
        self.max_window_sec = float(max_window_sec)
        self.min_distance = float(min_distance_m)
        self.alpha = float(alpha)
        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.max_direction_residual = float(max_direction_residual)
        self.max_age = float(max_age_sec)
        self.gps_lever = np.asarray(gps_lever, dtype=float)
        self.speeds = deque(maxlen=400)
        self.yaws = deque(maxlen=400)
        self.fixes = deque(maxlen=80)
        self.value = None
        self.last_update = None
        self.updates = 0
        self.rejected = 0
        self.last_measurement = None

    def reset_fixes(self):
        self.fixes.clear()

    def add_speed(self, stamp, speed):
        stamp = float(stamp)
        if self.speeds and stamp <= self.speeds[-1][0]:
            return
        if math.isfinite(speed):
            self.speeds.append((stamp, float(speed)))

    def add_yaw(self, stamp, yaw):
        stamp = float(stamp)
        if self.yaws and stamp <= self.yaws[-1][0]:
            return
        if not math.isfinite(yaw):
            return
        if self.yaws:
            previous = self.yaws[-1][1]
            yaw = previous + math.atan2(
                math.sin(yaw - previous), math.cos(yaw - previous))
        self.yaws.append((stamp, float(yaw)))

    def _base_position(self, stamp, gps_position):
        yaw = _interp_angle(self.yaws, stamp)
        c, s = math.cos(yaw), math.sin(yaw)
        lever = np.array([
            c * self.gps_lever[0] - s * self.gps_lever[1],
            s * self.gps_lever[0] + c * self.gps_lever[1]])
        return np.asarray(gps_position, dtype=float) - lever

    def _dead_reckoned(self, t0, t1):
        """Integral of speed*[cos yaw, sin yaw] d(wall time) over [t0, t1]."""
        samples = [s for s in self.speeds if s[0] < t1]
        if not samples or samples[0][0] > t0 + 0.25:
            return None
        start = 0
        for index, sample in enumerate(samples):
            if sample[0] <= t0:
                start = index
        samples = samples[start:]
        total = np.zeros(2)
        for index, (stamp, speed) in enumerate(samples):
            a = max(stamp, t0)
            b = samples[index + 1][0] if index + 1 < len(samples) else t1
            b = min(b, t1)
            if b <= a:
                continue
            if b - a > 0.3:  # speed stream gap: refuse to extrapolate
                return None
            yaw = _interp_angle(self.yaws, 0.5 * (a + b))
            total += speed * (b - a) * np.array([math.cos(yaw), math.sin(yaw)])
        return total

    def add_gps(self, stamp, gps_position):
        """Add a valid GPS fix; returns the new estimate or None."""
        stamp = float(stamp)
        if not self.yaws or not self.speeds:
            return None
        if self.fixes and stamp <= self.fixes[-1][0] + 1.0e-3:
            return None
        position = self._base_position(stamp, gps_position)
        self.fixes.append((stamp, position))
        while self.fixes and stamp - self.fixes[0][0] > self.max_window_sec:
            self.fixes.popleft()
        if self.speeds[-1][0] < stamp - 0.3 or self.yaws[-1][0] < stamp - 0.3:
            return None
        candidate = None
        for old_stamp, old_position in reversed(self.fixes):
            age = stamp - old_stamp
            if age < self.min_window_sec:
                continue
            candidate = (old_stamp, old_position)
            if (age >= self.window_sec and np.linalg.norm(
                    position - old_position) >= self.min_distance):
                break
        if candidate is None:
            return None
        old_stamp, old_position = candidate
        displacement = position - old_position
        distance = float(np.linalg.norm(displacement))
        if distance < self.min_distance:
            return None
        reckoned = self._dead_reckoned(old_stamp, stamp)
        if reckoned is None:
            return None
        norm_sq = float(reckoned @ reckoned)
        if norm_sq < 1.0e-6:
            return None
        measurement = float(displacement @ reckoned) / norm_sq
        residual = float(np.linalg.norm(
            displacement - measurement * reckoned)) / distance
        if (not math.isfinite(measurement)
                or not self.minimum <= measurement <= self.maximum
                or residual > self.max_direction_residual):
            self.rejected += 1
            return None
        self.last_measurement = measurement
        if self.value is None or self.last_update is None or (
                stamp - self.last_update > self.max_age):
            self.value = measurement
        else:
            self.value += self.alpha * (measurement - self.value)
        self.last_update = stamp
        self.updates += 1
        return self.value

    def get(self, stamp):
        if self.value is None or self.last_update is None:
            return None
        if float(stamp) - self.last_update > self.max_age:
            return None
        return self.value
