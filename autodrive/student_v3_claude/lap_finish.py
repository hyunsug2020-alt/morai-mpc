"""One-lap runs: follow route progress from the start pose and stop back at it.

The E2E launch (student_v3_follow_claude.sh) drives exactly one lap of the closed
global route from wherever the car stands at launch, then stops there. There is no
time limit; the launch ends when gui.py writes the lap marker.

Progress is the signed, unwrapped station along the route since the start pose:

* The station of every fix is its projection onto the closed route; consecutive
  stations are differenced modulo the route length, so a start just before the
  route's index wrap (404_contest starts 13 m before it) counts like any other.
* Signed accumulation: a reverse manoeuvre subtracts what it backs up, and
  driving forward again re-adds it - nothing is counted twice. Localization
  jitter at a standstill only wobbles progress around its true value.
* The finish is armed only after half the loop has been driven, so jitter or a
  reverse near the start can never complete a lap.
* A pose jump no car can drive in the elapsed time (a MORAI scenario reset /
  teleport) restarts the lap from the new pose instead of counting it. Fixes far
  off the route (lost localization) are ignored until it is back.

Once armed, the speed cap brakes smoothly (LAP_DECEL_MPS2, not an emergency stop)
to zero at the start station. The lap is done when the car rests within
FINISH_WINDOW_M of it (or past it) for HOLD_S; the cap then stays zero.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np

LAP_DECEL_MPS2 = 1.5          # comfortable braking to the start point (stop line uses 2.0)
LAP_REACTION_S = .3           # command/follower latency reserve, wall seconds (>= sim seconds)
ARM_FRACTION = .5             # the far side of the loop must have been driven
FINISH_WINDOW_M = 5.          # at rest this close to (or past) the start point: lap complete
OVERSHOOT_M = 30.             # never keep driving this far past the start point
STOP_SPEED_MPS = .3
HOLD_S = 1.
MOVED_M = 1.                  # lap clock starts once the car has left the start pose
OFF_ROUTE_M = 10.             # the launch requires the car within 10 m of the route
JUMP_BASE_M = 20.             # plausible travel between fixes: 20 m + 40 m/s x elapsed
JUMP_SPEED_MPS = 40.
SEARCH_WINDOW_M = 150.
SIGNAL_STOP_SPEED_MPS = .5/3.6
SIGNAL_WAIT_MAX_INTERVAL_S = .5


def signal_waiting(ego, signal, now_wall):
    """Exclude only an observed prohibiting signal holding an otherwise usable car.

    Unknown/stale perception, roundabout gaps, independent obstacle stops and
    signal approach braking remain part of the lap. Source timestamps belong
    to the already applied command, rather than a new timing-only observation.
    """
    try:
        return bool(ego.get('measured') is True and 0. <= float(ego['age_s']) <= .2
            and abs(float(ego['speed_kmh']))/3.6 <= SIGNAL_STOP_SPEED_MPS
            and signal.get('enabled') is True and signal.get('active') is True
            and signal.get('fresh') is True and signal.get('stop_required') is True
            and signal.get('raw_state') in ('RED', 'YELLOW', 'RED_LEFT', 'LEFT')
            and signal.get('phase') in ('HOLD', 'BRAKE') and bool(signal.get('line_id'))
            and 0. <= float(now_wall)-float(signal['frame_wall']) <= 1.
            and signal.get('pre_signal_emergency') is False
            and math.isfinite(float(signal['pre_signal_cap_mps']))
            and float(signal['pre_signal_cap_mps']) > 0.
            # The follower rests below its crawl threshold before the gate's
            # ideal zero-distance HOLD point (recorded caps .07–.09 m/s).
            and 0. <= float(signal['cap_mps']) <= SIGNAL_STOP_SPEED_MPS)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return False


class LapTracker:
    def __init__(self, route_xy, decel_mps2=LAP_DECEL_MPS2, arm_fraction=ARM_FRACTION,
                 finish_window_m=FINISH_WINDOW_M, hold_s=HOLD_S):
        xy = np.asarray(route_xy, dtype=np.float64)[:, :2]
        if len(xy) >= 2 and float(np.hypot(*(xy[0] - xy[-1]))) < 1e-6:
            xy = xy[:-1]                     # closed loop stored with a repeated first point
        if len(xy) < 3 or not np.isfinite(xy).all():
            raise ValueError("lap route needs at least three finite points")
        self.xy = xy
        self.seg = np.roll(xy, -1, axis=0) - xy   # last segment closes the loop
        self.seg_len = np.hypot(self.seg[:, 0], self.seg[:, 1])
        self.station = np.concatenate(([0.], np.cumsum(self.seg_len)[:-1]))
        self.length = float(self.seg_len.sum())
        spacing = max(self.length / len(xy), 1e-3)
        self.window = max(8, int(math.ceil(SEARCH_WINDOW_M / spacing)))
        self.decel = float(decel_mps2)
        self.arm_m = float(arm_fraction) * self.length
        self.finish_window_m = float(finish_window_m)
        self.hold_s = float(hold_s)
        self.hint = None
        self.start_station = self.start_xy = None
        self.last_station = self.last_time = None
        self.progress = 0.
        self.armed = self.done = False
        self.overshoot = False
        self.started_at = self.moved_at = self.finished_at = None
        self.sim_s = 0.
        self.signal_wait_s = self.signal_wait_sim_s = 0.
        self.last_signal_wait = False
        self.stopped_since = None
        self.restarts = 0
        self.off_route = 0
        self.last = dict(state="WAIT_START", cap_mps=None, done=False)

    # -- geometry -----------------------------------------------------------------
    def project(self, x, y):
        """(station_m, distance_m, index) of (x, y) on the closed route."""
        p = np.array((x, y), dtype=np.float64)
        n = len(self.xy)
        best = None
        if self.hint is not None:
            idx = (self.hint + np.arange(-self.window, self.window + 1)) % n
            d2 = np.einsum("ij,ij->i", self.xy[idx] - p, self.xy[idx] - p)
            k = int(np.argmin(d2))
            if d2[k] <= OFF_ROUTE_M ** 2:
                best = int(idx[k])
        if best is None:
            delta = self.xy - p
            best = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
        result = None
        for j in ((best - 1) % n, best):
            seg, length = self.seg[j], self.seg_len[j]
            t = 0. if length < 1e-9 else min(1., max(0., float((p - self.xy[j]) @ seg) / (length * length)))
            q = self.xy[j] + t * seg
            dist = float(np.hypot(*(p - q)))
            if result is None or dist < result[1]:
                result = ((float(self.station[j]) + t * float(length)) % self.length, dist, best)
        return result

    def wrap(self, delta):
        half = .5 * self.length
        return (delta + half) % self.length - half

    # -- progress -----------------------------------------------------------------
    def update(self, x, y, speed_mps, now, rtf=1., *, signal_wait=False):
        """Feed one localization fix; returns the lap status (cap_mps None = no cap)."""
        try:
            x, y, speed, now = float(x), float(y), abs(float(speed_mps)), float(now)
        except (TypeError, ValueError):
            return self.status(just_done=False)
        if not all(map(math.isfinite, (x, y, speed, now))):
            return self.status(just_done=False)
        waiting = bool(signal_wait is True and speed <= SIGNAL_STOP_SPEED_MPS)
        if self.done:
            return self.status(just_done=False)
        station, distance, index = self.project(x, y)
        if distance > OFF_ROUTE_M:
            self.off_route += 1
            return self.status(just_done=False)
        self.hint = index
        event = None
        if self.start_station is None:
            self._start(station, x, y, now)
            event = "start"
        else:
            delta = self.wrap(station - self.last_station)
            elapsed = max(0., now - self.last_time)
            if abs(delta) > JUMP_BASE_M + JUMP_SPEED_MPS * elapsed:
                # Teleport / scenario reset: a new lap from the new pose.
                self.restarts += 1
                self._start(station, x, y, now)
                event = "restart"
            else:
                self.progress += delta
                if self.moved_at is not None:
                    try:
                        scale = float(rtf)
                    except (TypeError, ValueError):
                        scale = 1.
                    scale = min(1., max(.3, scale)) if math.isfinite(scale) and scale > 0. else 1.
                    self.sim_s += elapsed * scale
                    if (waiting and self.last_signal_wait
                            and 0. <= elapsed <= SIGNAL_WAIT_MAX_INTERVAL_S):
                        self.signal_wait_s += elapsed
                        self.signal_wait_sim_s += elapsed*scale
        self.last_station, self.last_time = station, now
        self.last_signal_wait = waiting
        if self.moved_at is None and (self.progress >= MOVED_M or self.progress <= -MOVED_M):
            self.moved_at = now
        if not self.armed and self.progress >= self.arm_m:
            self.armed = True
            event = event or "armed"
        just_done = False
        remaining = self.length - self.progress
        if self.armed:
            if remaining <= self.finish_window_m and speed <= STOP_SPEED_MPS:
                if self.stopped_since is None:
                    self.stopped_since = now
                if now - self.stopped_since >= self.hold_s - 1e-9:
                    just_done = True
            else:
                self.stopped_since = None
            if remaining < -OVERSHOOT_M:
                self.overshoot = just_done = True
            if just_done:
                self.done, self.finished_at = True, now
                event = "done"
        return self.status(just_done=just_done, speed=speed, event=event)

    def _start(self, station, x, y, now):
        self.start_station, self.start_xy = station, (x, y)
        self.progress, self.armed, self.stopped_since = 0., False, None
        self.started_at, self.moved_at, self.sim_s = now, None, 0.
        self.signal_wait_s = self.signal_wait_sim_s = 0.
        self.last_signal_wait = False

    def cap_mps(self, speed_mps=0.):
        """Speed cap that reaches zero at the start station (None: no cap yet)."""
        if self.done:
            return 0.
        if not self.armed:
            return None
        remaining = self.length - self.progress
        available = max(0., remaining - abs(float(speed_mps)) * LAP_REACTION_S)
        return math.sqrt(2. * self.decel * available)

    def status(self, just_done=False, speed=0., event=None):
        remaining = self.length - self.progress
        state = ("WAIT_START" if self.start_station is None else "DONE" if self.done else
                 "FINISHING" if self.armed and remaining <= 120. else "ARMED" if self.armed else "LAP")
        self.last = dict(state=state, event=event, done=self.done, just_done=bool(just_done),
                         armed=self.armed, cap_mps=self.cap_mps(speed),
                         progress_m=round(self.progress, 2), remaining_m=round(remaining, 2),
                         fraction=round(self.progress / self.length, 4), route_length_m=round(self.length, 2),
                         restarts=self.restarts, off_route_fixes=self.off_route)
        return self.last

    def lap_time_s(self, now=None):
        if self.moved_at is None:
            return None
        end = self.finished_at if self.finished_at is not None else now
        return None if end is None else max(0., end-self.moved_at-self.signal_wait_s)

    def record(self, x=None, y=None, wall_time=None, run=None):
        """The lap_complete.json payload."""
        stop = None
        if x is not None and y is not None and self.start_xy is not None:
            stop = dict(x=round(float(x), 3), y=round(float(y), 3),
                        distance_from_start_m=round(math.hypot(float(x) - self.start_xy[0],
                                                               float(y) - self.start_xy[1]), 2))
        lap_wall = self.lap_time_s()
        return dict(lap_complete=self.done, run=run, overshoot=self.overshoot,
                    lap_time_s=None if lap_wall is None else round(lap_wall, 2),
                    lap_time_sim_s=round(max(0., self.sim_s-self.signal_wait_sim_s), 2)
                        if lap_wall is not None else None,
                    lap_time_total_s=round(self.finished_at-self.moved_at, 2)
                        if lap_wall is not None else None,
                    lap_time_total_sim_s=round(self.sim_s, 2) if lap_wall is not None else None,
                    signal_wait_s=round(self.signal_wait_s, 2),
                    signal_wait_sim_s=round(self.signal_wait_sim_s, 2),
                    lap_time_excludes_signal_wait=True,
                    route_length_m=round(self.length, 2), progress_m=round(self.progress, 2),
                    finish_offset_m=round(self.progress - self.length, 2),
                    start=dict(x=round(self.start_xy[0], 3), y=round(self.start_xy[1], 3),
                               station_m=round(self.start_station, 2)) if self.start_xy else None,
                    stop=stop, restarts=self.restarts, off_route_fixes=self.off_route,
                    finished_wall=wall_time)


def lap_tracker_from_env(route_xy, environ=None):
    """STUDENT_ONE_LAP=1 (set by the E2E launch) enables the one-lap stop."""
    environ = os.environ if environ is None else environ
    if str(environ.get("STUDENT_ONE_LAP", "0")).strip() != "1":
        return None
    decel = float(environ.get("STUDENT_LAP_DECEL_MPS2", LAP_DECEL_MPS2))
    if not math.isfinite(decel) or not .5 <= decel <= 3.:
        raise ValueError("STUDENT_LAP_DECEL_MPS2 must be within 0.5-3.0")
    return LapTracker(route_xy, decel_mps2=decel)


def lap_marker_path(environ=None):
    environ = os.environ if environ is None else environ
    if environ.get("STUDENT_LAP_MARKER"):
        return Path(environ["STUDENT_LAP_MARKER"])
    if environ.get("STUDENT_LOG_DIR"):
        return Path(environ["STUDENT_LOG_DIR"]) / "lap_complete.json"
    return None


def write_lap_marker(path, record):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name("." + path.name + ".tmp")
    tmp.write_text(json.dumps(record, indent=2))
    os.replace(tmp, path)
