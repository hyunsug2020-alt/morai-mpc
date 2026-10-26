"""Simulator units for measured motion: MORAI runs slower than real time.

The real-time factor (RTF) is simulated seconds per wall-clock second. On this
PC MORAI runs at RTF 0.5-0.9 (median 0.67 in follow-20261010T101327Z, varying
within a lap). Ego speed from /localization/ego_status and every control limit
(speed caps, comfort/emergency deceleration, the planners' forecast times) are
simulator units. Perceived motion is not: LiDAR vehicle tracks, semantic
fusion, tunnel clusters and the junction/lead trackers difference positions
over wall-clock stamps, so their velocities are RTF x the true speed. Highway
19:18:06-22 of that run: the 40 km/h lead 30 m ahead read 23 km/h while the
gap stayed constant, so the speed plan held the ego at 40 km/h and 2.7 s.

One rule set, applied wherever object motion meets ego motion:

* Planners work in simulator units. Each planner entry converts measured
  object tuples once with ``to_sim_objects``: velocity and turn rate / rtf.
* Perception-side position propagation (scan alignment, masks, tracker
  prediction, dropout ghosts) stays in wall units: the wall velocity it was
  measured with times the wall age is the same displacement as sim velocity
  times sim age, so converting only one of the two would be wrong.
* A wall-second latency or observation age used purely as an ego reaction or
  observation reserve stays as is. It is >= its simulator duration (rtf <= 1),
  so treating it as simulator seconds is a valid, conservative upper bound.
  This keeps every braking-feasibility margin at least as strict as before.
* Motion-evidence gates that reject measurement noise compare measured (wall)
  speeds. A non-person, non-vehicle observation below
  MEASURED_OBSTACLE_MOTION_MPS is static before conversion (the same 1 m/s
  fused-box jitter bound as path_safety.FUSED_STATIC_MPS); dividing that jitter
  by rtf must not turn standing boxes into movers.
* Pedestrians refined by ped_guard.PedTracker are already simulator units
  (``people=False`` leaves them untouched; never convert them twice).
"""
from __future__ import annotations

import math
from collections import deque

RTF_MIN = .3
RTF_MAX = 1.
# Fusion reported 0.5-0.95 m/s on standing boxes and parked cars (see
# path_safety.FUSED_STATIC_MPS); gui.is_dynamic_obstacle uses the same bound.
MEASURED_OBSTACLE_MOTION_MPS = 1.


def clip_rtf(rtf):
    """A usable conversion factor: unknown/invalid -> 1.0 (no conversion)."""
    try:
        value = float(rtf)
    except (TypeError, ValueError):
        return 1.
    if not math.isfinite(value) or value <= 0.:
        return 1.
    return min(RTF_MAX, max(RTF_MIN, value))


def sim_mps(measured_mps, rtf):
    """Measured (wall-clock) speed -> simulator speed."""
    return float(measured_mps) / clip_rtf(rtf)


def sim_seconds(wall_s, rtf):
    """Wall-clock duration -> simulator duration (never longer than wall)."""
    return float(wall_s) * clip_rtf(rtf)


def to_sim_objects(objects, rtf, people=True):
    """Object tuples with velocity fields in simulator units.

    Index 5 (speed), 9/10 (vx, vy) and a numeric index 12 (turn rate, rad/s)
    are divided by rtf. Index 15 (detector prediction age) stays wall seconds:
    planners use it only as an observation reserve and as a "retained motion"
    flag tied to the detector's wall-clock track lifetime. Positions, sizes,
    headings and circle geometry carry no time unit.
    """
    scale = 1. / clip_rtf(rtf)
    result = []
    for obj in objects or ():
        if len(obj) < 6 or (obj[4] == 'pedestrian' and not people):
            result.append(obj)
            continue
        row = list(obj)
        try:
            speed = float(row[5])
        except (TypeError, ValueError):
            result.append(obj)
            continue
        if obj[4] == 'obstacle' and not abs(speed) >= MEASURED_OBSTACLE_MOTION_MPS:
            # Measurement noise on a standing box/parked body is not motion.
            row[5] = 0. if math.isfinite(speed) else speed
            if len(row) > 10:
                row[9] = row[10] = 0.
        elif scale != 1.:
            row[5] = speed * scale
            if len(row) > 10:
                row[9], row[10] = float(row[9]) * scale, float(row[10]) * scale
            if (len(row) > 12 and isinstance(row[12], (int, float)) and
                    not isinstance(row[12], bool)):
                row[12] = float(row[12]) * scale
        result.append(tuple(row))
    return result


class SimClock:
    """Real-time factor from the ego's own motion: moved distance / reported speed.

    The ESKF pose advances by the distance actually driven per wall second;
    its speed field is simulator speed. Over a 2 s window of motion (>= 2.5 m/s
    throughout, >= 3 m expected travel) their ratio is the RTF (the ESKF's own
    GPS/wheel ratio changed by up to 0.15 within 2 s). Standing or slow driving
    keeps the last estimate (1.0 until the first measurement), so a stop-line
    hold or a queue never invents a new factor.

    Errors are asymmetric: a factor below the truth overstates every measured
    speed (a lead looks faster, the gap closes), one above it only keeps the
    previous conservative reading. A drop is therefore followed more slowly,
    and a window below RATIO_FLOOR (a stalled pose, not a 0.3x simulator; runs
    measured 0.41-0.95) is ignored rather than clipped into the estimate.
    """
    WINDOW_S = 2.
    MIN_SPAN_S = 1.
    MIN_SPEED_MPS = 2.5
    MIN_EXPECTED_M = 3.
    SMOOTHING = .3
    DROP_SMOOTHING = .1
    RATIO_FLOOR = .35
    CHUNK_S = .3

    def __init__(self, rtf=1.):
        self.estimate = clip_rtf(rtf)
        self.valid = False
        self._samples = deque()
        self._last_time = None

    @property
    def rtf(self):
        """Conversion factor in [RTF_MIN, RTF_MAX]."""
        return clip_rtf(self.estimate)

    def reset_samples(self):
        self._samples.clear()
        self._last_time = None

    def update(self, ego, now):
        """Add one pose sample (ego dict with x, y, speed_kmh, optional stamp)."""
        if not ego:
            return self.rtf
        try:
            x, y = float(ego['x']), float(ego['y'])
            speed = abs(float(ego.get('speed_kmh', 0.))) / 3.6
            stamp = float(ego.get('stamp') or 0.)
            now = float(now)
        except (KeyError, TypeError, ValueError):
            return self.rtf
        # The pose's own stamp pairs position and time exactly; the decision
        # clock only replaces it when the source has none.
        when = stamp if stamp > 0. else now
        if not all(math.isfinite(v) for v in (x, y, speed, when)):
            return self.rtf
        if self._last_time is not None:
            if when <= self._last_time:
                if when < self._last_time - 1.:
                    self.reset_samples()   # clock/source restart
                else:
                    return self.rtf        # repeated pose: no new evidence
            elif when - self._last_time > 1.:
                self.reset_samples()       # gap: do not bridge a teleport/reset
        self._last_time = when
        self._samples.append((when, x, y, speed))
        while self._samples and when - self._samples[0][0] > self.WINDOW_S:
            self._samples.popleft()
        samples = list(self._samples)
        if (speed < self.MIN_SPEED_MPS or len(samples) < 4 or
                samples[-1][0] - samples[0][0] < self.MIN_SPAN_S or
                min(s[3] for s in samples) < self.MIN_SPEED_MPS):
            return self.rtf
        steps = [(math.hypot(b[1] - a[1], b[2] - a[2]), .5 * (a[3] + b[3]) * (b[0] - a[0]),
                  b[0] - a[0]) for a, b in zip(samples, samples[1:])]
        # A pose that stops advancing (or jumps) under a moving speed is not
        # the simulator clock: no update while such a stretch is in the window.
        # Judged over >= CHUNK_S stretches so sample timing jitter averages out.
        chunk = [0., 0., 0.]
        for step in steps:
            chunk = [total + value for total, value in zip(chunk, step)]
            if chunk[2] >= self.CHUNK_S:
                if chunk[1] > .3 and not self.RATIO_FLOOR * .7 <= chunk[0] / chunk[1] <= 2.:
                    return self.rtf
                chunk = [0., 0., 0.]
        moved = sum(step[0] for step in steps)
        expected = sum(step[1] for step in steps)
        if expected <= self.MIN_EXPECTED_M:
            return self.rtf
        ratio = moved / expected
        if not ratio >= self.RATIO_FLOOR:
            return self.rtf
        ratio = min(1.2, ratio)
        if not self.valid:
            self.estimate = ratio
        else:
            gain = self.SMOOTHING if ratio >= self.estimate else self.DROP_SMOOTHING
            self.estimate += gain * (ratio - self.estimate)
        self.valid = True
        return self.rtf

    def record(self):
        """Decision-log fields: the factor used and whether it was measured."""
        return dict(rtf=round(self.rtf, 3), rtf_measured=bool(self.valid),
                    rtf_estimate=round(float(self.estimate), 3))


def runtime_rtf(runtime, default=1.):
    """The runtime's current conversion factor (1.0 without a clock)."""
    clock = getattr(runtime, 'sim_clock', None)
    return clock.rtf if clock is not None else clip_rtf(default)
