"""Intersection stop and measured empty-entry policies."""
from __future__ import annotations

import math
import threading
import time
from ego_body import EGO_FRONT_M


COMPETITION_STOP_NODE_IDS = frozenset({"A1256W000709"})
# Shared by gap prediction and the published speed ramp. Curvature, headway and
# collision checks may lower this ceiling; it is never a minimum speed.
# 2026-10-03 run: the car reached 2.9 m/s^2 from standstill. At 1.2 m/s^2 and
# 12 km/h a crossing lane took >4 s to clear, so no gap in steady cross traffic
# was ever accepted (90 s creeping at the junction). 2026-10-10 (user): enter firmly
# behind the passing ring car - 3.0 m/s^2 (follow-20261010T113545Z measured 2.85-3.1
# m/s^2 sim behind a faster published ramp; ring_entry.LAUNCH_* use that response).
INTERSECTION_ACCEL_MPS2 = 3.0
INTERSECTION_ENTRY_MPS = 30.0 / 3.6
# Apply the entry ceiling immediately before the full 18 m traffic check.
# Earlier approach commands retain their independent stopping envelope.
INTERSECTION_ENTRY_CAP_FRONT_GAP_M = 20.
STOP_WINDOW_EXTRA_M = float(__import__("os").environ.get("STUDENT_STOP_WINDOW_EXTRA_M", "0.3"))
# At rest this much beyond the stop window still counts as the full stop: the controller's own
# hold (effective distance <= BUFFER_M + .3) parks us there. follow-20261010T092032Z lap 2 came
# to rest at 1.12-1.16 m (window 1.10): the 1 s hold never started, age jitter flipped the stop
# to APPROACH at 18:29:12.16, we crept to 1.09 m and only then held; ~4 s lost.
STOP_REST_EXTRA_M = 0.3
# Once the controller has commanded the stop this close to the line (front bumper to the
# line), it stays a stop until the 1 s hold is done, and wherever the car then comes to
# rest within it is the stop at the line. follow-20261010T130317Z: the stop was commanded
# at 1.71 m (lap 2) / 1.61 m (lap 1) at ~5 km/h, the car rested at 1.54 m - just outside
# the 1.40 m rest window - so APPROACH released a creep, HOLD/APPROACH toggled at 0.3-3
# km/h down to 1.18-1.23 m and every creep over 0.15 m/s restarted the hold (~1.5 s lost;
# lap 2 missed the passing ring car).
STOP_CAPTURE_M = 2.0
ENTRY_MODEL_ACTIONS = frozenset({"KEEP_LANE", "TURN_LEFT", "TURN_RIGHT"})
KNOWN_MODEL_ACTIONS = ENTRY_MODEL_ACTIONS | frozenset({
    "STOP", "YIELD", "UNKNOWN", "LEFT_CHANGE", "RIGHT_CHANGE"})
EMPTY_ENTRY_REASON = "empty junction checked while approaching"
EMPTY_ENTRY_ROLL_MPS = 2.6 / 3.6
# The ring conflict point lies inside the planner's 35 m traffic horizon here.
# Before this window, retain a stopping envelope rather than treating an
# uncovered far-away junction as a checked gap.
YIELD_CHECK_DISTANCE_M = 18.
# The mandatory-stop policy retains its original 0.8 m buffer and 2 m
# capture window. A traffic-only approach completes its braking trajectory
# nearer the physical line, then holds that position until a checked GO.
# A 0.3 m buffer left only 0.035 m in the delayed type-1 pedal model;
# retain another 0.2 m for its tracking/deadband response.
YIELD_BUFFER_M = .5
YIELD_WAIT_CAPTURE_M = .6
# During a blocked near-line approach, retain a requested low-speed stop
# instead of accelerating again as the measured braking reserve recovers.
YIELD_STOP_COMMAND_MPS = .5


def intersection_entry_cap_mps(front_gap):
    """Return the near-entry ceiling; invalid progress keeps its conservative cap."""
    try:
        gap = float(front_gap)
        if math.isfinite(gap) and gap > INTERSECTION_ENTRY_CAP_FRONT_GAP_M:
            return None
    except (TypeError, ValueError, OverflowError):
        pass
    return INTERSECTION_ENTRY_MPS


def yield_line_cap(front_gap, speed_mps, observation_age_s=0.):
    """Brake toward the line only while its traffic gap is unavailable."""
    try:
        gap, speed, age = float(front_gap), abs(float(speed_mps)), float(observation_age_s)
        if not all(math.isfinite(v) for v in (gap, speed, age)) or age < 0.:
            return 0.
        travel = speed*age+.5*INTERSECTION_ACCEL_MPS2*age*age
        room = max(0., gap-travel-YIELD_BUFFER_M)
        brake, response = StopLineController.BRAKE_MPS2, StopLineController.REACTION_S
        # Reserve the response at both the measured speed and the requested
        # speed. At rest the latter prevents a new approach target from using
        # the entire remaining gap before its delayed stop can take effect.
        measured_cap = math.sqrt(2.*brake*max(0., room-speed*response))
        command_cap = math.sqrt((brake*response)**2+2.*brake*room)-brake*response
        return min(measured_cap, command_cap)
    except (TypeError, ValueError, OverflowError):
        return 0.


def intersection_front_station(ego, route_progress):
    """Project the physical front point without changing legacy road footprints."""
    try:
        x, y, heading = float(ego["x"]), float(ego["y"]), math.radians(float(ego["heading_deg"]))
        if not all(math.isfinite(v) for v in (x, y, heading)):
            return math.nan
        front = dict(ego, x=x + EGO_FRONT_M * math.cos(heading),
                     y=y + EGO_FRONT_M * math.sin(heading))
        return float(route_progress(front))
    except (KeyError, TypeError, ValueError, OverflowError):
        return math.nan


def empty_intersection_observation(status):
    """Require a fresh, active detector sample with no current or remembered cars."""
    if not isinstance(status, dict) or status.get("detector_ready") is not True:
        return False
    region = status.get("active_region") or {}
    if not isinstance(region, dict) or region.get("node_id") not in COMPETITION_STOP_NODE_IDS:
        return False
    try:
        age, stamp = float(status["age_s"]), float(status["stamp"])
    except (KeyError, TypeError, ValueError):
        return False
    if not (math.isfinite(age) and 0. <= age <= .35 and math.isfinite(stamp) and stamp > 0.):
        return False
    if type(status.get("car_count")) is not int or status["car_count"] != 0:
        return False
    return all(isinstance(status.get(key), (list, tuple)) and not status[key]
               for key in ("vehicles", "ghost_ids", "inherited_ids", "unknown_ids"))


def competition_stop_controller(lines, *, require_full_stop=False):
    """Competition entry yields to traffic without a mandatory stop at the line."""
    selected = [line for line in lines if line.get("node_id") in COMPETITION_STOP_NODE_IDS]
    missing = COMPETITION_STOP_NODE_IDS - {line["node_id"] for line in selected}
    if missing:
        raise ValueError("Requested intersection stop missing from HD map: " + ", ".join(sorted(missing)))
    return StopLineController(selected, hold_s=0., allow_empty_entry=True,
                              require_full_stop=require_full_stop)


class StopLineController:
    FRONT_M = 2.3175
    BUFFER_M = 0.8
    # Full stop before entry, in simulator seconds (user rule 2026-10-10). The
    # controller clock is wall time; MORAI runs at RTF 0.5-0.9, so one wall second
    # was only ~0.57 simulated seconds of standing (follow-20261010T101327Z).
    HOLD_S = 1.0
    CLEAR_S = 0.0
    REACTION_S = 0.5
    BRAKE_MPS2 = 2.0
    ACCEL_MPS2 = INTERSECTION_ACCEL_MPS2

    def __init__(self, lines, hold_s=None, allow_empty_entry=False, require_full_stop=True):
        self._state_lock = threading.RLock()
        self.allow_empty_entry = bool(allow_empty_entry)
        self.yield_only = not bool(require_full_stop)
        self.yield_approach = False
        self.yield_wait_latched = False
        self.yield_launch_pending = False
        self.empty_approach = False
        self.empty_stop_latched = False
        self.empty_committed = False
        self.empty_prepared = False
        self.empty_rolling_deferred = False
        self.hold_s = self.HOLD_S if hold_s is None else float(hold_s)
        if not math.isfinite(self.hold_s) or self.hold_s < 0.:
            raise ValueError('Stop-line hold duration must be finite and nonnegative')
        self.lines = sorted(lines, key=lambda line: line["station_m"])
        self.completed = set()
        self.active = None
        self.released = False
        self.stopped_since = self.clear_since = None
        self.held_sim_s = 0.
        self.hold_tick = None
        # The mandatory 1 s stop, once done at this line, is never redone (user rule
        # 2026-10-10). follow-20261010T113545Z: decision gaps > 0.5 s and pose ages
        # > 0.5 s reset the completed hold three times at the line (39:50.5, 54.9, 56.9).
        self.hold_latched = False
        # The stop commanded within STOP_CAPTURE_M of the line (hysteresis: no APPROACH again).
        self.stop_latched = False
        self.last_station = self.last_time = None

    @property
    def at_line(self):
        """True once the stop at the active line is commanded or completed (not approaching)."""
        with self._state_lock:
            return bool(self.active is not None and not self.released and (self.stop_latched or self.hold_latched))

    def rolling_entry_available(self, front_distance, speed_mps):
        """Admit rolling preparation before its window, or at measured roll speed.

        A late empty observation during a normal braking approach must not
        introduce a new 2.5 km/h target jump. Finish that stop and use its gap.
        """
        with self._state_lock:
            try:
                from roundabout_empty import rolling_preparation_cap, ROLL_CHECK_FRONT_GAP_M
                gap, speed = float(front_distance), abs(float(speed_mps))
                return bool(self.allow_empty_entry and math.isfinite(gap) and math.isfinite(speed)
                    and not (self.stop_latched or self.hold_latched or self.empty_stop_latched
                             or self.empty_rolling_deferred)
                    and (self.empty_prepared or speed <= EMPTY_ENTRY_ROLL_MPS
                         # Allow only a small measured-speed discrepancy on
                         # first admission; later ticks retain the prepared ramp.
                         or (gap > ROLL_CHECK_FRONT_GAP_M
                             and speed <= rolling_preparation_cap(gap)+.15)))
            except (TypeError, ValueError, OverflowError):
                return False

    def consume_empty_crossing(self, line_id, line_station, front_station, speed_mps, pose_age_s,
                               *, expires_at=None):
        """Store an independently certified crossing of this provisional entry.

        The caller owns the fresh path, traffic and collision proof. Serialize
        consumption with ordinary updates so an older approach cannot undo it.
        Keep pose history intact for the existing teleport/new-lap reset checks.
        """
        with self._state_lock:
            try:
                line, front, speed, age, expiry = map(float,
                    (line_station, front_station, speed_mps, pose_age_s, expires_at))
                if (not all(math.isfinite(v) for v in (line, front, speed, age, expiry))
                        or time.monotonic() > expiry
                        or not self.allow_empty_entry or self.active is None
                        or self.active['id'] != line_id
                        or abs(float(self.active['station_m'])-line) > 1e-6
                        or front < line or not 0. <= speed <= EMPTY_ENTRY_ROLL_MPS
                        or not 0. <= age <= .2):
                    return False
            except (KeyError, TypeError, ValueError, OverflowError):
                return False
            if self.released:
                return self.empty_committed
            if not self.empty_approach or self.stop_latched or self.hold_latched:
                return False
            self.released = self.empty_committed = True
            self.empty_approach = self.empty_stop_latched = False
            self.stopped_since = self.clear_since = self.hold_tick = None
            self.held_sim_s = 0.
            return True

    def consume_yield_crossing(self, line_id, line_station, front_station, speed_mps, pose_age_s,
                               *, expires_at=None):
        """Consume a moving crossing certified by fresh traffic and body checks."""
        return self._consume_yield_entry(line_id, line_station, front_station, speed_mps,
            pose_age_s, expires_at=expires_at, approach_m=0.)

    def consume_yield_launch(self, line_id, line_station, front_station, speed_mps, pose_age_s,
                             *, expires_at=None):
        """Start the freshly checked whole entry within the near-line launch area.

        Waiting until the front crossed restarted the standstill launch forecast
        on every tick, cancelling a safe launch and repeatedly resetting the
        controller's speed ramp. Once accepted, measured following and body/person
        guards own the trajectory, as in the existing committed entry policy.
        """
        return self._consume_yield_entry(line_id, line_station, front_station, speed_mps,
            pose_age_s, expires_at=expires_at, approach_m=self.yield_launch_capture_m)

    @property
    def yield_launch_capture_m(self):
        """Select the physical ring's launch area without changing stop capture."""
        try:
            line = self.active or {}
            distance = float(line.get('yield_launch_capture_m', STOP_CAPTURE_M))
            if (self.yield_only and float(line.get('yield_advance_m', 0.)) > 0.
                    and math.isfinite(distance)):
                return max(STOP_CAPTURE_M, min(3., distance))
        except (TypeError, ValueError, OverflowError):
            pass
        return STOP_CAPTURE_M

    def consume_inside_recovery(self, line_id, line_station, station, *, checked_at, expires_at):
        """Finish line bookkeeping for independently checked cold inside recovery.

        This grants no ring-gap commitment. The live recovery owner retains
        the complete NPC forecast and emergency authority at every publication.
        """
        with self._state_lock:
            try:
                line, rear, checked, expiry = map(float, (line_station, station, checked_at, expires_at))
                now = float(time.monotonic())
                if (not self.yield_only or self.active is None or self.released
                        or self.active['id'] != line_id or line_id in self.completed
                        or abs(float(self.active['station_m'])-line) > 1e-6
                        or not all(math.isfinite(v) for v in (line, rear, checked, expiry, now))
                        or not line+.5 < rear <= line+12. or not checked <= now <= expiry):
                    return False
            except (KeyError, TypeError, ValueError, OverflowError):
                return False
            self.released = True
            self.yield_approach = self.yield_wait_latched = self.yield_launch_pending = False
            return True

    def _consume_yield_entry(self, line_id, line_station, front_station, speed_mps, pose_age_s,
                             *, expires_at, approach_m):
        with self._state_lock:
            try:
                line, front, speed, age, expiry = map(float,
                    (line_station, front_station, speed_mps, pose_age_s, expires_at))
                if (not self.yield_only or self.active is None or not self.yield_approach
                        or self.active['id'] != line_id
                        or not all(math.isfinite(v) for v in (line, front, speed, age, expiry))
                        or abs(float(self.active['station_m'])-line) > 1e-6
                        or front < line-approach_m or speed < 0. or not 0. <= age <= .2
                        or time.monotonic() > expiry):
                    return False
            except (KeyError, TypeError, ValueError, OverflowError):
                return False
            self.released = True
            self.yield_approach = False
            self.yield_wait_latched = False
            self.yield_launch_pending = front < line
            return True

    def update_yield_launch(self, front_station, *, stopped=False):
        """An interrupted launch before crossing needs a new entry proof."""
        with self._state_lock:
            if not self.yield_launch_pending or self.active is None:
                return False
            try:
                front = float(front_station)
            except (TypeError, ValueError, OverflowError):
                front = math.nan
            if math.isfinite(front) and front >= self.active['station_m']:
                self.yield_launch_pending = False
            elif stopped:
                self.yield_launch_pending = self.released = self.yield_approach = False
                self.yield_wait_latched = True
                return True
            return False

    def apply_yield_wait(self, line_id, line_station, front_station, speed_mps, cap_mps,
                         *, go=False, expires_at=None):
        """Keep a traffic stop at its position until the live gap/body proof is GO.

        A GUI GO reserves a restart. Only its fresh publication releases the
        traffic stop; a later live WAIT cannot cause another creep toward the line.
        """
        with self._state_lock:
            try:
                line, front, speed, cap = map(float,
                    (line_station, front_station, speed_mps, cap_mps))
                if (not self.yield_only or self.active is None
                        or self.active['id'] != line_id
                        or abs(float(self.active['station_m'])-line) > 1e-6
                        or not all(math.isfinite(v) for v in (line, front, speed, cap))):
                    return 0.
                if go:
                    expiry = float(expires_at)
                    if not math.isfinite(expiry) or time.monotonic() > expiry or cap <= 0.:
                        return 0.
                    self.yield_wait_latched = False
                elif (self.yield_wait_latched
                        or (line-front <= YIELD_WAIT_CAPTURE_M
                            and (cap <= 0. or abs(speed) <= .15))
                        or (line-front <= STOP_CAPTURE_M
                            and cap <= YIELD_STOP_COMMAND_MPS)):
                    self.yield_wait_latched = True
                    return 0.
                return max(0., cap)
            except (KeyError, TypeError, ValueError, OverflowError):
                return 0.

    def update(self, station, speed_mps, now, clear, observation_age_s=0., time_scale=1.,
               *, empty_entry_clear=False, empty_entry_prepare=False,
               empty_front_station=None, empty_entry_clearance_m=0.):
        """Update the stop policy together with publication-time entry commits."""
        with self._state_lock:
            return self._update(station, speed_mps, now, clear, observation_age_s, time_scale,
                empty_entry_clear=empty_entry_clear, empty_front_station=empty_front_station,
                empty_entry_clearance_m=empty_entry_clearance_m,
                empty_entry_prepare=empty_entry_prepare)

    def _update(self, station, speed_mps, now, clear, observation_age_s=0., time_scale=1.,
               *, empty_entry_clear=False, empty_entry_prepare=False,
               empty_front_station=None, empty_entry_clearance_m=0.):
        """Return a speed cap/status; clear requires fresh streams and no traffic risk.

        ``time_scale`` is the latest simulator real-time factor (sim_time.SimClock).
        Standing time counts wall elapsed x min(time_scale, 1): the hold lasts
        HOLD_S simulated seconds, i.e. HOLD_S / rtf wall seconds, and never less
        than HOLD_S wall seconds. The observation age stays a wall-second travel
        reserve (an upper bound of its simulated duration).
        """
        station, speed_mps, now = float(station), abs(float(speed_mps)), float(now)
        try:
            scale = float(time_scale)
        except (TypeError, ValueError):
            scale = math.nan
        empty_time_ok = math.isfinite(scale) and 0. < scale <= 1.
        scale = min(1., scale) if math.isfinite(scale) and scale > 0. else 1.
        try:
            age = float(observation_age_s)
        except (TypeError, ValueError):
            age = float("nan")
        try:
            front_station = float(empty_front_station)
        except (TypeError, ValueError, OverflowError):
            front_station = math.nan
        if not all(math.isfinite(v) for v in (station, speed_mps, now, age)) or age < 0.:
            # Unusable localization adds no standing time; it does not undo time held.
            self.clear_since = None
            self.hold_tick = None
            return dict(state="HOLD", cap_mps=0., stop=True,
                        reason="stop line localization invalid")
        if self.allow_empty_entry and not math.isfinite(front_station):
            # The opt-in controller uses the physical front for every approach,
            # including a mandatory stop after empty clearance is lost.
            self.clear_since = None
            self.hold_tick = None
            return dict(state="HOLD", cap_mps=0., stop=True,
                        reason="stop line physical front invalid")
        long_tick = False
        if self.last_station is not None:
            dt = now - self.last_time
            # A reset/teleport or a new lap requires a new stop. Small backwards
            # localization jitter must not rearm a line we have just cleared.
            jump = station - self.last_station
            if dt < 0. or jump < -20. or abs(jump) > max(30., 2. * speed_mps * max(dt, 0.) + 10.):
                self.completed.clear()
                self.active, self.released = None, False
                self.stopped_since = self.clear_since = None
                self.hold_latched = self.stop_latched = False
                self.empty_approach = False
                self.empty_stop_latched = False
                self.empty_committed = False
                self.empty_prepared = self.empty_rolling_deferred = False
                self.yield_approach = False
                self.yield_wait_latched = False
                self.yield_launch_pending = False
            elif dt > .5:
                # A long decision gap is not a measured standstill: it adds no time
                # (and needs a fresh clear sample), but keeps the time already held.
                long_tick = True
                self.clear_since = None
        self.last_station, self.last_time = station, now

        # Keep the original rear-pose exit boundary: using front+12 here would
        # end the entry-specific actual-body checks before the rear pose clears.
        if self.active is not None and self.released and station > self.active["station_m"] + 12.:
            self.completed.add(self.active["id"])
            self.active, self.released = None, False
            self.stopped_since = self.clear_since = None
            self.hold_latched = self.stop_latched = False
            self.empty_approach = False
            self.empty_stop_latched = False
            self.empty_committed = False
            self.empty_prepared = self.empty_rolling_deferred = False
            self.yield_approach = False
            self.yield_wait_latched = False
            self.yield_launch_pending = False
        if self.active is None:
            candidates = [line for line in self.lines if line["id"] not in self.completed
                          and (line["station_m"] - front_station if self.allow_empty_entry
                               else line["station_m"] - station - self.FRONT_M) <=
                              80. + (float(line.get('yield_advance_m', 0.)) if self.yield_only else 0.)
                          and (self.yield_only and station <= line["station_m"] + 12.
                               or -.5 <= (line["station_m"] - front_station if self.allow_empty_entry
                                          else line["station_m"] - station - self.FRONT_M))]
            # A restart inside the entry still needs an active controller to
            # consume a freshly checked GO. Restore only that candidate, up to
            # the existing rear-pose exit boundary; never restore a release.
            if not candidates:
                return None
            self.active = candidates[0]
            self.stopped_since = self.clear_since = None
            self.held_sim_s, self.hold_latched, self.stop_latched = 0., False, False
            self.empty_approach = False
            self.empty_stop_latched = False
            self.empty_committed = False
            self.empty_prepared = self.empty_rolling_deferred = False
            self.yield_approach = False
            self.yield_wait_latched = False
            self.yield_launch_pending = False

        distance = (self.active["station_m"] - front_station if self.allow_empty_entry
                    else self.active["station_m"] - station - self.FRONT_M)
        # Position is kept at its observed station. Reserve the travel since
        # that observation, including bounded acceleration, in the speed cap.
        observed_travel = speed_mps * age + .5 * self.ACCEL_MPS2 * age * age
        effective_distance = distance - observed_travel
        result = dict(line_id=self.active["id"], line_xy=self.active["xy"],
                      station_m=self.active["station_m"], distance_m=distance,
                      effective_distance_m=effective_distance, observation_age_s=age)
        if self.released:
            return dict(result, state="ENTER", cap_mps=INTERSECTION_ENTRY_MPS, stop=False,
                        reason="stop line checked; cautious entry")

        observation_fresh = age <= .5
        clear = bool(clear) and observation_fresh
        if self.yield_only:
            # GO is a provisional moving permission. A line cannot force a
            # full stop, hold timer, stop latch, or 2.5 km/h crawl in this mode.
            checked = bool(clear and age <= .2 and empty_time_ok and distance <= YIELD_CHECK_DISTANCE_M)
            self.yield_approach = checked
            if checked:
                # The GUI may refresh its pose past the line after computing
                # GO. Only the independent fresh traffic/body publication can
                # consume the near-line launch and set released.
                return dict(result, state='APPROACH_YIELD',
                    cap_mps=INTERSECTION_ENTRY_MPS, stop=False, gap_checked=True,
                    held_sim_s=0., required_hold_s=0., hold_latched=False,
                    reason='junction gap checked while approaching')
            cap = yield_line_cap(distance, speed_mps, age)
            if (self.yield_wait_latched
                    or (distance <= YIELD_WAIT_CAPTURE_M
                        and (cap <= 0. or speed_mps <= .15))
                    or (distance <= STOP_CAPTURE_M
                        and cap <= YIELD_STOP_COMMAND_MPS)):
                self.yield_wait_latched = True
                cap = 0.
            return dict(result, state='APPROACH_YIELD' if cap > 0. else 'YIELD_WAIT',
                cap_mps=cap, stop=cap <= 0., gap_checked=False,
                held_sim_s=0., required_hold_s=0., hold_latched=False,
                reason='yield for junction traffic' if distance <= YIELD_CHECK_DISTANCE_M
                       else 'approach junction within braking envelope')
        preparing = empty_entry_prepare is True or empty_entry_clear is True
        if self.allow_empty_entry:
            if preparing and not self.rolling_entry_available(distance, speed_mps):
                self.empty_rolling_deferred = True
            if preparing and not self.empty_rolling_deferred:
                self.empty_prepared = True
            elif not preparing:
                self.empty_prepared = False
        if (self.allow_empty_entry and empty_entry_clear is True and clear and empty_time_ok
                and math.isfinite(front_station)
                and self.rolling_entry_available(distance, speed_mps)):
            from roundabout_empty import approach_cap, MAX_POSE_AGE_S
            # Rechecked every decision, including inside the stop window. An empty
            # approach is not a commit: new traffic/readiness loss restores the stop
            # policy until the measured front bumper has actually crossed the line.
            front_distance = self.active["station_m"] - front_station
            cap = approach_cap(front_distance, speed_mps, age, empty_entry_clearance_m)
            if cap > 0.:
                self.stop_latched = self.hold_latched = False
                self.empty_stop_latched = False
                self.stopped_since = self.clear_since = self.hold_tick = None
                self.held_sim_s = 0.
                # Keep the braking envelope at higher measured speed. Only the
                # final, freshly checked empty segment rolls at at most 3 km/h.
                self.released = (front_distance <= 0. and speed_mps <= EMPTY_ENTRY_ROLL_MPS
                                 and age <= MAX_POSE_AGE_S)
                self.empty_approach = not self.released
                self.empty_committed = self.released
                return dict(result, state="ENTER" if self.released else "APPROACH_EMPTY",
                            cap_mps=INTERSECTION_ENTRY_MPS if self.released else cap, stop=False,
                            held_sim_s=0., required_hold_s=self.hold_s,
                            hold_latched=False, empty_entry=True, time_scale=round(scale, 3),
                            empty_front_distance_m=front_distance,
                            reason="empty junction entry checked" if self.released else EMPTY_ENTRY_REASON)
        if self.empty_approach:
            # Lost provisional clearance close to the physical line: hold where
            # we are, rather than resume the approach toward a different window.
            if distance <= STOP_CAPTURE_M:
                self.stop_latched = True
                self.empty_stop_latched = True
            self.empty_approach = False
        # The approach planner now stops at the same line instead of applying
        # a future merge-leader cap before it. Use the 0.8 m buffer plus a small
        # capture window (STOP_REST_EXTRA_M more at rest); a full stop still requires <= 0.15 m/s.
        if self.empty_stop_latched and distance > STOP_CAPTURE_M + 1.:
            self.empty_stop_latched = self.stop_latched = False
        if self.stop_latched and distance > STOP_CAPTURE_M + 1. and not self.empty_stop_latched:
            self.stop_latched = False            # moved/relocalized well away: a new approach
        at_line = (distance <= self.BUFFER_M + STOP_WINDOW_EXTRA_M + STOP_REST_EXTRA_M or
                   (self.stop_latched and distance <= STOP_CAPTURE_M) or
                   (self.empty_stop_latched and distance <= STOP_CAPTURE_M))
        stopped_at_line = at_line and speed_mps <= .15
        if self.hold_latched:
            pass                                 # done at this line: never redone or reset
        elif stopped_at_line:
            if self.stopped_since is None:
                self.stopped_since = now
                self.held_sim_s = 0.
            elif observation_fresh and not long_tick and self.hold_tick is not None:
                # Only fresh, regularly observed standstill counts (simulator seconds);
                # a stale pose or a long tick adds nothing but resets nothing either.
                self.held_sim_s += max(0., now - self.hold_tick) * scale
            self.hold_tick = now
            if (self.held_sim_s >= self.hold_s - 1e-9 and
                    (self.hold_s > 0. or observation_fresh)):
                self.hold_latched = True
        else:
            # Moving (or not yet at the line) before the hold completed: start over.
            self.stopped_since = None
            self.held_sim_s, self.hold_tick = 0., None
        if self.hold_latched or stopped_at_line:
            # The mandatory full stop and timed traffic prediction are separate.
            # Once stopped for one second, accept the first freshly checked gap.
            if clear:
                if self.clear_since is None:
                    self.clear_since = now
            else:
                self.clear_since = None
        else:
            self.clear_since = None
        held = self.hold_latched
        result["held_sim_s"] = (round(max(self.held_sim_s, self.hold_s if held else 0.), 3)
                                if (held or self.stopped_since is not None) else 0.)
        result['required_hold_s'] = self.hold_s
        result["hold_latched"] = bool(held)
        result["time_scale"] = round(scale, 3)
        if held and clear and self.clear_since is not None:
            if now - self.clear_since >= self.CLEAR_S - 1e-9:
                self.released = True
                return dict(result, state="ENTER", cap_mps=INTERSECTION_ENTRY_MPS, stop=False,
                            reason="stop line checked; cautious entry")

        # Reaction travel depends on measured speed, not the smaller requested
        # speed. A slow command cannot retroactively shorten the current coast.
        reaction_travel = speed_mps * self.REACTION_S
        available = max(0., effective_distance - self.BUFFER_M - reaction_travel)
        cap = math.sqrt(2. * self.BRAKE_MPS2 * available)
        if self.allow_empty_entry and preparing and not self.empty_rolling_deferred:
            from roundabout_empty import rolling_preparation_cap
            # Begin preparing before the final five-metre proof window. This
            # applies only to an observed empty entry. With circulating cars,
            # use the ordinary smooth stop instead of crawling the last 5 m.
            cap = min(cap, rolling_preparation_cap(distance))
        # Inside the stop window: stop now. Creeping on at 0.5-0.8 km/h (> 0.15 m/s)
        # restarted the hold four times in 1.5 s (run 228) - one stop, one hold.
        in_window = distance <= self.BUFFER_M + STOP_WINDOW_EXTRA_M
        stop = (held or self.stop_latched or stopped_at_line or in_window
                or effective_distance <= self.BUFFER_M + .3 or available <= 0.)
        if stop and distance <= STOP_CAPTURE_M:
            self.stop_latched = True             # one clean stop: no APPROACH creep after it
        if stop:
            return dict(result, state="WAIT_CLEAR" if held else "HOLD", cap_mps=0., stop=True,
                        reason="stop line waiting for clear traffic" if held else "stop line mandatory full stop")
        return dict(result, state="APPROACH", cap_mps=cap, stop=False,
                    reason="approaching mandatory stop line")


def entry_is_clear(yielding, emergency, fresh):
    """A creeping crossing decision is still a conflict when waiting at a line."""
    if not fresh or emergency:
        return False
    if not yielding:
        return True
    if yielding.get("kind") in ("stop", "slow"):
        return False
    if yielding.get("source") == "crossing_caution":
        return False
    if yielding.get("source") == "traffic":
        return yielding.get("reason") in (
            "lead vehicle headway", "merge lead vehicle headway",
            "follow merging lead with checked headway", "continue checked merge",
            "committed merge with braking reserve",
            EMPTY_ENTRY_REASON,
            "gap accepted with acceleration and clearance",
            "junction committed: follow the car ahead", "junction committed: drive on") \
            and yielding.get("v_mps", 0.) > .3
    return False


def intersection_entry_decision(actions, probabilities, *, lidar_ready,
                                yielding=None, emergency=False, fresh=True,
                                model_fresh=True):
    """Release on measured traffic clearance; model actions are advisory only.

    The mandatory stop is enforced by StopLineController. A learned STOP or
    lane-change label cannot veto a fresh, collision-checked entry opportunity.
    """
    action, probability, valid, winners = None, None, False, ()
    try:
        labels = tuple(actions)
        values = tuple(float(value) for value in probabilities)
        valid = (len(labels) == len(values) and bool(labels)
                 and len(set(labels)) == len(labels)
                 and all(label in KNOWN_MODEL_ACTIONS for label in labels)
                 and all(math.isfinite(value) and 0. <= value <= 1. for value in values)
                 and abs(math.fsum(values) - 1.) <= .001)
        if valid:
            probability = max(values)
            winners = tuple(label for label, value in zip(labels, values)
                            if abs(value - probability) <= 1e-9)
            action = winners[0]
    except (TypeError, ValueError, OverflowError):
        valid = False

    traffic_clear = entry_is_clear(yielding, emergency, fresh)
    result = dict(clear=False, action=action, probability=probability,
                  valid_model=bool(valid), model_fresh=bool(model_fresh),
                  lidar_ready=bool(lidar_ready), traffic_clear=traffic_clear,
                  authority="lidar_traffic", model_advisory=True,
                  model_permits_entry=bool(valid and model_fresh and winners
                      and all(label in ENTRY_MODEL_ACTIONS for label in winners)))
    if emergency:
        reason = "intersection collision risk"
    elif not lidar_ready:
        reason = "intersection LiDAR not ready"
    elif not fresh:
        reason = "intersection sensors stale"
    elif not traffic_clear:
        reason = "intersection traffic conflict"
    else:
        result["clear"] = True
        reason = "intersection LiDAR traffic permits entry"
    result["reason"] = reason
    return result
