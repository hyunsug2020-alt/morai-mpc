#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lightweight HD map lane-change planner.

Input:
  - /localization/ego_status
  - /Object_topic
  - /lane_change_cmd (std_msgs/Int32): target lane number. 0 clears the target.

Output:
  - /avoid_waypoints: path_follower_node compatible waypoints
  - /avoid_target_vel: target speed in m/s
  - /avoid_path: RViz/Foxglove path
  - /lane_change_status: short text status
"""
import csv
import gzip
import json
import math
import os
import sys
import threading
import zipfile
from collections import defaultdict, deque

import rospy
from geometry_msgs.msg import Point, PoseStamped
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

_NODE_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if _NODE_DIRECTORY not in sys.path:
    sys.path.insert(0, _NODE_DIRECTORY)
from local_bypass_core import (  # noqa: E402
    ObstacleOBB,
    build_clamped_bspline_offset_path,
    obb_signed_clearance,
    quintic_smoothstep,
    resample_polyline,
    minimum_quintic_transition_length,
    obb_projected_half_extents,
    project_to_frenet,
    sample_lateral_targets,
    swept_obb_clearance,
    validate_path_kinematics,
)

class FinalRoute(tuple):
    """Route in its published form (immutable shape marker only). It is NOT
    a safety certificate: a positive command also needs a same-tick
    validation certificate for an equal route (_publish enforces this)."""
    __slots__ = ()


SWEEP_CALL_BUDGET = 5000       # OBB evaluations per swept check
SWEEP_TICK_BUDGET = 10000      # OBB evaluations per planning tick

# ── Execution contract (planner_execution_contract): per-tick budget ─────────
TICK_WORK_UNITS = 400000       # deterministic work units per planning tick
TICK_DEADLINE_S = 0.25         # wall-clock watchdog per planning tick
TICK_CHECK_EVERY = 64          # wall clock checked every N work units
REACQUIRE_CHUNK = 40000       # base-route segments scored per tick during global reacquisition
REACQUIRE_POSE_TOL = 0.5      # m, pose change allowed between chunks of one reacquisition (else restart)
RETAINED_SCAN_MAX = 2000       # bounded stop fallback: retained-reference nearest-point scan


class TickBudgetExceeded(BaseException):
    """Raised when the per-tick work or wall-clock budget is exhausted.
    Derives from BaseException so generic 'except Exception' handlers in
    choosers cannot swallow it: the whole tick unwinds to _plan, which
    publishes an explicit stop (no later branch can raise the command)."""


class TickBudget(object):
    """Shared per-tick work token + wall-clock deadline. Every costly loop
    (object broadphase x route segments, swept samples and pair evaluations,
    object lead scans, candidate retries) charges it in small chunks."""

    def __init__(self, work_units=TICK_WORK_UNITS, deadline_s=TICK_DEADLINE_S,
                 clock=None):
        import time as _time
        self._clock = clock or _time.perf_counter
        self.start = self._clock()
        self.deadline = self.start + max(0.0, float(deadline_s))
        self.limit = int(work_units)
        self.used = 0
        self.checks = 0
        self.exhausted = None
        self._since_check = 0

    def spend(self, units=1):
        units = int(units)
        self.used += units
        self._since_check += units
        if self.used > self.limit:
            self.exhausted = "work"
            raise TickBudgetExceeded("work")
        if self._since_check >= TICK_CHECK_EVERY:
            self._since_check = 0
            self.check()

    def check(self):
        self.checks += 1
        if self._clock() > self.deadline:
            self.exhausted = "deadline"
            raise TickBudgetExceeded("deadline")

    def report(self):
        return {"used": self.used, "limit": self.limit,
                "elapsed_ms": 1000.0 * (self._clock() - self.start),
                "deadline_ms": 1000.0 * (self.deadline - self.start),
                "exhausted": self.exhausted, "clock_checks": self.checks}


# ── Execution contract: reachable arrival-time envelope ──────────────────────
ENVELOPE_A_MAX = 2.0           # m/s^2, max acceleration of the executed motion (assumption)
ENVELOPE_B_HI = 8.0            # m/s^2, max deceleration (earliest possible stop)
ENVELOPE_B_LO = 2.0            # m/s^2, guaranteed deceleration for the stop argument
ENVELOPE_T_REACT = 0.5         # s, planner->controller->actuator reaction latency
ENVELOPE_T_TICK = 0.1          # s, planning period
ENVELOPE_T_PRED = 8.0          # s, reachability horizon (stations reachable before it are checked)
ENVELOPE_T_HOLD = 20.0         # s, a stop/hold at any checked station must stay clear until then
ENVELOPE_T_MAX = 60.0          # s, cap of the whole-route reachability window
ENVELOPE_MAX_D0 = 1.5          # m, max reference offset from the checked path
ENVELOPE_MAX_HEADING = math.radians(45.0)
# revision3: the controller feed timeout is part of every stopping proof. After the last received cap the
# controller (revision4, hdmap_path_follower.launch avoid_path_timeout_s) keeps executing it for up to this long
# before its stale-cap stop; every certificate must cover motion + reaction + braking through it.
ENVELOPE_T_FEED = 1.5          # s, controller feed timeout (runtime value validated at startup)
CLOCK_TOLERANCE_S = 1.0e-3     # s, accepted negative receipt age (clock resolution); larger = future receipt


def envelope_policy(t_feed=ENVELOPE_T_FEED):
    """Immutable policy string certificates are bound to (includes the feed timeout)."""
    return ("reachable-envelope-v2 a=%.2f bhi=%.2f blo=%.2f react=%.2f "
            "tick=%.2f pred=%.1f hold=%.1f feed=%.2f" % (
                ENVELOPE_A_MAX, ENVELOPE_B_HI, ENVELOPE_B_LO,
                ENVELOPE_T_REACT, ENVELOPE_T_TICK, ENVELOPE_T_PRED,
                ENVELOPE_T_HOLD, float(t_feed)))


ENVELOPE_POLICY = envelope_policy()


def obstacle_finite(o):
    """Every safety-relevant field of an ObstacleOBB is finite (revision3)."""
    try:
        return all(math.isfinite(float(v)) for v in (o.x, o.y, o.yaw, o.length, o.width, o.vx, o.vy))
    except (TypeError, ValueError):
        return False


class ArrivalEnvelope(object):
    """Reachable station/time envelope of the ego reference point.

    Time origin tau = 0: the ego measurement of this tick's frame; the tick
    runs at tau = ego_age; objects were measured at tau_obj0 = ego_age -
    object_age. Along the checked path the reference starts at the measured
    projection and moves forward only, with speed in [0, v_top],
    acceleration <= a_max and deceleration <= b_hi (stops, dwells and
    restarts allowed). Fastest: accelerate from v0 to v_top then cruise;
    slowest: brake at b_hi from v0 and hold. Any profile satisfying these
    bounds (lower caps, release limiter, controller filters, delayed or
    held commands) stays inside [slowest, fastest]."""

    def __init__(self, v0, v_top, ego_age=0.0, object_age=0.0, v0_lo=None,
                 a_max=ENVELOPE_A_MAX, b_hi=ENVELOPE_B_HI, b_lo=ENVELOPE_B_LO,
                 t_react=ENVELOPE_T_REACT, t_tick=ENVELOPE_T_TICK,
                 t_pred=ENVELOPE_T_PRED, t_hold=ENVELOPE_T_HOLD, t_feed=ENVELOPE_T_FEED):
        # v0: initial speed of the FASTEST profile; v0_lo: of the SLOWEST
        # profile. Equal for a measured speed; an unknown initial speed is
        # the interval [0, v_top] (fastest from v_top, slowest from rest).
        self.v0 = max(0.0, float(v0)) if math.isfinite(v0) else float("nan")
        self.v_top = max(self.v0, max(0.0, float(v_top)))
        lo = self.v0 if v0_lo is None else float(v0_lo)
        self.v0_lo = max(0.0, min(self.v0, lo)) if math.isfinite(lo) else float("nan")
        self.a_max = max(1.0e-3, float(a_max))
        self.b_hi = max(1.0e-3, float(b_hi))
        self.b_lo = max(1.0e-3, float(b_lo))
        self.ego_age = max(0.0, float(ego_age))
        self.tau_obj0 = self.ego_age - max(0.0, float(object_age))
        # revision3: + controller feed timeout (the last cap may be executed that long without a new tick)
        self.t_feed = max(0.0, float(t_feed)) if math.isfinite(float(t_feed)) else float("nan")
        self.policy = envelope_policy(self.t_feed)
        stop_time = t_tick + t_react + self.t_feed + self.v_top / self.b_lo
        self.horizon_s = max(float(t_pred), stop_time)
        self.tau_end = self.ego_age + self.horizon_s
        # A station reachable before tau_end may become a hold position (the
        # vehicle may be stopped there by a later tick): it must stay clear of
        # moving objects until tau_hold.
        self.tau_hold = self.ego_age + max(self.horizon_s, float(t_hold))
        self.tau_stop = self.ego_age + stop_time
        self.required_length = self.fastest_distance(self.tau_stop)

    def valid(self):
        return (math.isfinite(self.v0) and math.isfinite(self.v_top) and
                math.isfinite(self.v0_lo) and math.isfinite(self.t_feed))

    def covering(self, distance, t_max=ENVELOPE_T_MAX):
        """Copy whose reachability window also covers the fastest arrival at
        `distance` (the end of the checked route), capped at t_max: every
        station of the checked route is then certified for every admissible
        profile, not only the stations reachable within t_pred."""
        other = object.__new__(ArrivalEnvelope)
        other.__dict__.update(self.__dict__)
        need = self.t_early(max(0.0, distance))
        horizon = max(self.horizon_s, min(float(t_max), need))
        other.horizon_s = horizon
        other.tau_end = self.ego_age + horizon
        other.tau_hold = max(self.tau_hold, other.tau_end)
        return other

    def fastest_distance(self, tau):
        if tau <= 0.0:
            return 0.0
        ta = (self.v_top - self.v0) / self.a_max
        if tau <= ta:
            return self.v0 * tau + 0.5 * self.a_max * tau * tau
        da = self.v0 * ta + 0.5 * self.a_max * ta * ta
        return da + self.v_top * (tau - ta)

    def t_early(self, distance):
        """Earliest time the reference can be `distance` ahead of s0."""
        if distance <= 0.0:
            return 0.0
        if self.v_top <= 1.0e-9:
            return float("inf")
        ta = (self.v_top - self.v0) / self.a_max
        da = self.v0 * ta + 0.5 * self.a_max * ta * ta
        if distance <= da:
            return (-self.v0 + math.sqrt(self.v0 * self.v0 +
                                         2.0 * self.a_max * distance)) / self.a_max
        return ta + (distance - da) / self.v_top

    def t_late(self, distance):
        """Latest time the reference may still be at `distance` (inf when a
        stop at or before it is possible)."""
        v = self.v0_lo
        if distance <= 0.0 and v <= 1.0e-9:
            return float("inf")
        d_stop = v * v / (2.0 * self.b_hi)
        if distance >= d_stop:
            return float("inf")
        disc = max(0.0, v * v - 2.0 * self.b_hi * distance)
        return (v - math.sqrt(disc)) / self.b_hi

    def summary(self):
        return {"v0": self.v0, "v0_lo": self.v0_lo, "v_top": self.v_top, "a_max": self.a_max,
                "b_hi": self.b_hi, "b_lo": self.b_lo, "ego_age": self.ego_age,
                "tau_obj0": self.tau_obj0, "horizon_s": self.horizon_s,
                "tau_stop": self.tau_stop, "tau_hold": self.tau_hold,
                "required_length": self.required_length, "t_feed": self.t_feed,
                "policy": self.policy}


def _hull_gap(ego_c, ego_h, half_l, half_w, obj_a, obj_b, yaw, obj_hl, obj_hw):
    """Lower bound of the distance between the ego rectangle and the convex
    hull of an object rectangle translated from centre obj_a to obj_b
    (exactly the union of its positions in between). SAT over all edge
    normals of both convex sets; any projected gap bounds the Euclidean
    distance from below, overlap on every axis returns <= 0."""
    ec, es = math.cos(ego_h), math.sin(ego_h)
    oc, os_ = math.cos(yaw), math.sin(yaw)
    ego = [(ego_c[0] + a * ec - b * es, ego_c[1] + a * es + b * ec)
           for a, b in ((half_l, half_w), (half_l, -half_w),
                        (-half_l, -half_w), (-half_l, half_w))]
    obj = []
    for cx, cy in (obj_a, obj_b):
        obj.extend((cx + a * oc - b * os_, cy + a * os_ + b * oc)
                   for a, b in ((obj_hl, obj_hw), (obj_hl, -obj_hw),
                                (-obj_hl, -obj_hw), (-obj_hl, obj_hw)))
    axes = [(ec, es), (-es, ec), (oc, os_), (-os_, oc)]
    dx, dy = obj_b[0] - obj_a[0], obj_b[1] - obj_a[1]
    norm = math.hypot(dx, dy)
    if norm > 1.0e-9:
        axes.append((-dy / norm, dx / norm))
    best = -float("inf")
    for ax, ay in axes:
        pa = [ax * x + ay * y for x, y in ego]
        pb = [ax * x + ay * y for x, y in obj]
        gap = max(min(pb) - max(pa), min(pa) - max(pb))
        if gap > best:
            best = gap
    return best


def envelope_ego_body_clearance(path, obstacles, rear_extent, front_extent,
                                ego_width, envelope, ego_pose,
                                safety_margin=0.0, sample_step=0.2,
                                work_budget=SWEEP_CALL_BUDGET,
                                tick_budget=None):
    """Execution-contract swept check (see ArrivalEnvelope).

    Static objects: every sample of the whole path (time independent), as the
    constant-speed check did. Moving objects: for every path interval
    [s_a, s_b] ahead of the measured reference, the object is replaced by the
    convex hull of its constant-velocity positions over the reachable time
    window [t_early(s_a), t_late(s_b)] clipped to [0, tau_end]; the body at
    both interval ends must clear that hull by the margin, with the same
    Lipschitz interval bound and bisection as the constant-speed check.
    The measured reference offset |d0| is added to the margin; |d0| or the
    heading error above limits fails closed ('ego-off-reference'). The
    reference must be able to stop inside the checked path (fastest
    distance at the stop-guarantee time), else 'envelope-beyond-checked-route'.
    Following traffic behind the rear face (same direction) is excluded
    ONLY for conflicts at in-lane stations (no cut-in) and reported.
    Scope: model bound only; no tracking error during execution, perception
    error, object acceleration or yaw rate."""
    def failed(kind, detail=None):
        return {"valid": False, "minimum_clearance": -float("inf"),
                "closest": {"path_index": None, "time_s": None,
                            "object_id": None, "kind": kind,
                            "clearance": -float("inf"), "between_samples": False,
                            "detail": detail},
                "evaluations": state["evaluations"],
                "budget_exhausted": kind == "work-budget",
                "envelope": envelope.summary() if envelope else None,
                "excluded_following": excluded}
    state = {"evaluations": 0, "minimum": float("inf"), "closest": None,
             "first_conflict_s": float("inf")}
    excluded = []
    dense = []
    for point in resample_polyline(path, sample_step):
        if not dense or math.hypot(point[0] - dense[-1][0],
                                   point[1] - dense[-1][1]) > 1.0e-9:
            dense.append(point)
    if len(dense) < 2:
        raise ValueError("path needs at least two distinct points")
    if envelope is None or not envelope.valid():
        return failed("invalid-envelope")
    count = len(dense)
    arc = [0.0]
    for first, second in zip(dense, dense[1:]):
        arc.append(arc[-1] + math.hypot(second[0] - first[0],
                                        second[1] - first[1]))
    headings = []
    for index in range(count):
        before, after = max(0, index - 1), min(count - 1, index + 1)
        headings.append(math.atan2(dense[after][1] - dense[before][1],
                                   dense[after][0] - dense[before][0]))
    length = rear_extent + front_extent
    shift = 0.5 * (front_extent - rear_extent)
    half_l, half_w = 0.5 * length, 0.5 * ego_width
    ego_radius = math.hypot(half_l, half_w)
    reach = math.hypot(max(rear_extent, front_extent), half_w)

    def spend(units=1):
        if tick_budget is not None:
            tick_budget.spend(units)

    # measured reference projection
    ex, ey, eh = ego_pose
    best = None
    for index in range(count - 1):
        spend(1)
        a, b = dense[index], dense[index + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg = dx * dx + dy * dy
        f = 0.0 if seg <= 1.0e-18 else max(0.0, min(1.0, (
            (ex - a[0]) * dx + (ey - a[1]) * dy) / seg))
        px, py = a[0] + f * dx, a[1] + f * dy
        d = math.hypot(ex - px, ey - py)
        if best is None or d < best[0]:
            best = (d, index, f)
    d0, i0, f0 = best
    s0 = arc[i0] + f0 * (arc[i0 + 1] - arc[i0])
    seg_heading = math.atan2(dense[i0 + 1][1] - dense[i0][1],
                             dense[i0 + 1][0] - dense[i0][0])
    heading_error = abs(_wrap(eh - seg_heading)) if eh is not None else 0.0
    if d0 > ENVELOPE_MAX_D0 or heading_error > ENVELOPE_MAX_HEADING:
        return failed("ego-off-reference",
                      "d0=%.2f heading_err=%.1fdeg" % (d0, math.degrees(heading_error)))
    if s0 + envelope.required_length > arc[-1] + 1.0e-9:
        return failed("envelope-beyond-checked-route",
                      "need=%.1f have=%.1f" % (s0 + envelope.required_length, arc[-1]))
    envelope = envelope.covering(arc[-1] - s0)
    margin = float(safety_margin) + d0

    def ego_corner_center(x, y, yaw):
        return (x + shift * math.cos(yaw), y + shift * math.sin(yaw))

    def note(value, index, time_s, obstacle, between):
        if value != value:              # revision3: NaN clearance = conflict, never 'not negative'
            value = -float("inf")
        if value < state["minimum"]:
            state["minimum"] = value
            state["closest"] = {
                "path_index": index, "time_s": time_s,
                "object_id": obstacle.object_id, "kind": obstacle.kind,
                "clearance": value, "between_samples": between}

    def interval_bound(c0, c1, spread):
        if abs(c0 - c1) >= spread:
            return min(c0, c1)
        return 0.5 * (c0 + c1 - spread)

    def count_exact():
        state["evaluations"] += 1
        spend(1)
        if state["evaluations"] > work_budget:
            raise OverflowError("work-budget")

    # revision3: a non-finite obstacle field can never be read as clear space
    if not all(obstacle_finite(o) for o in obstacles):
        return failed("invalid-obstacle")
    # ── static objects: time independent, whole path ──
    static = [o for o in obstacles if math.hypot(o.vx, o.vy) <= 1.0e-6]
    moving = [o for o in obstacles if math.hypot(o.vx, o.vy) > 1.0e-6]
    ego_move = [
        (arc[i + 1] - arc[i]) + reach * abs(_wrap(headings[i + 1] - headings[i]))
        for i in range(count - 1)]
    try:
        for obstacle in static:
            spend(1)
            object_radius = math.hypot(0.5 * obstacle.length, 0.5 * obstacle.width)
            widest = max(ego_move) if ego_move else 0.0
            values = []
            for index in range(count):
                spend(1)
                x, y, yaw = dense[index][0], dense[index][1], headings[index]
                cx, cy = ego_corner_center(x, y, yaw)
                lower = (math.hypot(cx - obstacle.x, cy - obstacle.y) - ego_radius -
                         object_radius - margin)
                if lower > max(0.0, state["minimum"]) + widest + 1.0e-9:
                    values.append(lower)
                    continue
                count_exact()
                value = obb_signed_clearance(
                    ObstacleOBB(cx, cy, yaw, length, ego_width), obstacle) - margin
                values.append(value)
                note(value, index, None, obstacle, False)
                if not value >= 0.0:
                    state["first_conflict_s"] = min(state["first_conflict_s"], arc[index])
            for index in range(count - 1):
                spend(1)
                c0, c1 = values[index], values[index + 1]
                bound = interval_bound(c0, c1, ego_move[index])
                if bound < 0.0 and c0 >= 0.0 and c1 >= 0.0:
                    # static object: bisect the chord exactly (depth <= 8)
                    bound = _static_refine(dense, headings, index, obstacle,
                                           shift, length, ego_width, margin,
                                           ego_move[index], c0, c1, count_exact)
                if bound < min(c0, c1):
                    note(bound, index, None, obstacle, True)
                    if bound < 0.0:
                        state["first_conflict_s"] = min(state["first_conflict_s"], arc[index])

        # ── moving objects: reachable time windows ahead of s0 ──
        dyn = [(dense[i0][0] + f0 * (dense[i0 + 1][0] - dense[i0][0]),
                dense[i0][1] + f0 * (dense[i0 + 1][1] - dense[i0][1]), 0.0)]
        for index in range(i0 + 1, count):
            if arc[index] - s0 > 1.0e-9:
                dyn.append((dense[index][0], dense[index][1], arc[index] - s0))
        dyn_head = []
        for index in range(len(dyn)):
            before, after = max(0, index - 1), min(len(dyn) - 1, index + 1)
            dyn_head.append(math.atan2(dyn[after][1] - dyn[before][1],
                                       dyn[after][0] - dyn[before][0]))
        dyn_move = [
            (dyn[i + 1][2] - dyn[i][2]) + reach * abs(_wrap(dyn_head[i + 1] - dyn_head[i]))
            for i in range(len(dyn) - 1)]
        h0 = dyn_head[0]
        in_lane = [abs(-math.sin(h0) * (q[0] - dyn[0][0]) + math.cos(h0) * (q[1] - dyn[0][1])) <= 0.5
                   and abs(_wrap(dyn_head[i] - h0)) <= math.radians(10.0)
                   for i, q in enumerate(dyn)]
        tau_end = envelope.tau_end

        for obstacle in moving:
            spend(1)
            object_radius = math.hypot(0.5 * obstacle.length, 0.5 * obstacle.width)
            ohl, ohw = 0.5 * obstacle.length, 0.5 * obstacle.width

            def centre(tau, o=obstacle):
                dt = tau - envelope.tau_obj0
                return (o.x + o.vx * dt, o.y + o.vy * dt)
            # revision2: narrow rear-following responsibility exception (replaces the 08f7 'behind and
            # along >= 0.5 v' rule that exempted diagonal cut-ins). The actor must be a follower NOW: fully
            # behind the ego rear face (by its half diagonal), its centre inside the ego band shrunk by
            # 0.30 m, yaw within 10 deg (mod 180) and velocity within the path direction (along >= 0.9|v|,
            # |lateral v| <= 0.1 m/s). A conflict at window j is excluded only on in-lane stations whose path
            # heading is within 5 deg of the current heading, and only if the actor centre stays inside the
            # ego half width at both window ends in both pose frames while moving forward in that frame.
            # Then the actor enters the body rectangle through the rear face (a point moving forward with
            # |y| <= half width can only cross x = -rear): a rear-end by the follower, which a stop cannot
            # avoid and the cap-only interface cannot certify. Corner tolerance from yaw/lateral drift:
            # <= ohw*sin(10deg) at <= 0.1 m/s lateral, i.e. < 0.05 m for vehicle sizes. Side, diagonal,
            # near-parallel and cut-in entries always keep their conflict (see tests/r2_matrix.py A).
            now = centre(envelope.ego_age)
            rel_x = math.cos(h0) * (now[0] - dyn[0][0]) + math.sin(h0) * (now[1] - dyn[0][1])
            rel_y = -math.sin(h0) * (now[0] - dyn[0][0]) + math.cos(h0) * (now[1] - dyn[0][1])
            speed = math.hypot(obstacle.vx, obstacle.vy)
            along = math.cos(h0) * obstacle.vx + math.sin(h0) * obstacle.vy
            lateral_v = -math.sin(h0) * obstacle.vx + math.cos(h0) * obstacle.vy
            yaw_err = abs(_wrap(2.0 * (obstacle.yaw - h0))) * 0.5
            following = (rel_x < -rear_extent - object_radius and abs(rel_y) <= half_w - 0.30 and
                         yaw_err <= math.radians(10.0) and speed > 0.1 and
                         along >= 0.9 * speed and abs(lateral_v) <= 0.1)

            def rear_end_only(j):
                t0, t1 = window(dyn[j][2], dyn[j + 1][2])
                for fr in (0.0, 1.0):
                    x, y, yaw, _d = pose(j, fr)
                    if abs(_wrap(yaw - h0)) > math.radians(5.0):
                        return False
                    cyaw, syaw = math.cos(yaw), math.sin(yaw)
                    if cyaw * obstacle.vx + syaw * obstacle.vy <= 0.0:
                        return False
                    for cx, cy in (centre(t0), centre(t1)):
                        if abs(-syaw * (cx - x) + cyaw * (cy - y)) > half_w:
                            return False
                return True

            def value_at(x, y, yaw, t0, t1):
                a, b = centre(t0), centre(t1)
                cx, cy = ego_corner_center(x, y, yaw)
                # centre distance to the object's centre segment (cheap bound)
                dx, dy = b[0] - a[0], b[1] - a[1]
                seg = dx * dx + dy * dy
                f = 0.0 if seg <= 1.0e-18 else max(0.0, min(1.0, (
                    (cx - a[0]) * dx + (cy - a[1]) * dy) / seg))
                lower = (math.hypot(cx - a[0] - f * dx, cy - a[1] - f * dy) -
                         ego_radius - object_radius - margin)
                spend(1)
                if lower > 0.0:
                    return lower
                count_exact()
                return _hull_gap((cx, cy), yaw, half_l, half_w, a, b,
                                 obstacle.yaw, ohl, ohw) - margin

            def pose(j, fr):
                a, b = dyn[j], dyn[j + 1]
                return (a[0] + fr * (b[0] - a[0]), a[1] + fr * (b[1] - a[1]),
                        dyn_head[j] + fr * _wrap(dyn_head[j + 1] - dyn_head[j]),
                        a[2] + fr * (b[2] - a[2]))

            def window(da, db):
                # occupied from the earliest arrival until the latest time the
                # reference may still be there: a possible stop at/before db
                # (t_late = inf) keeps the station occupied until tau_hold
                t0 = envelope.t_early(da)
                t1 = min(envelope.t_late(db), envelope.tau_hold)
                return t0, max(t0, t1)

            def refine(j, a0, a1, depth):
                x0, y0, h_0, d_a = pose(j, a0)
                x1, y1, h_1, d_b = pose(j, a1)
                t0, t1 = window(d_a, d_b)
                if t0 > tau_end:
                    return float("inf")
                c0 = value_at(x0, y0, h_0, t0, t1)
                c1 = value_at(x1, y1, h_1, t0, t1)
                bound = interval_bound(c0, c1, dyn_move[j] * (a1 - a0))
                if bound >= 0.0 or depth >= 8 or c0 < 0.0 or c1 < 0.0:
                    return bound
                mid = 0.5 * (a0 + a1)
                return min(refine(j, a0, mid, depth + 1), refine(j, mid, a1, depth + 1))

            for j in range(len(dyn) - 1):
                spend(1)
                if envelope.t_early(dyn[j][2]) > tau_end:
                    break
                bound = refine(j, 0.0, 1.0, 0)
                if 0.0 <= bound < state["minimum"]:
                    note(bound, j, window(dyn[j][2], dyn[j + 1][2])[0], obstacle, True)
                if not bound >= 0.0:        # revision3: NaN bound is a conflict
                    if (following and in_lane[j] and in_lane[j + 1] and
                            rear_end_only(j)):
                        excluded.append({"object_id": obstacle.object_id,
                                         "path_distance": dyn[j][2]})
                        continue
                    note(bound, j, window(dyn[j][2], dyn[j + 1][2])[0], obstacle, True)
                    state["first_conflict_s"] = min(state["first_conflict_s"], s0 + dyn[j][2])
                    break          # this object already invalidates the path
    except OverflowError:
        return failed("work-budget")
    return {"valid": state["minimum"] >= 0.0,
            "minimum_clearance": state["minimum"],
            "closest": state["closest"],
            "evaluations": state["evaluations"], "budget_exhausted": False,
            "envelope": envelope.summary(), "s0": s0, "d0": d0,
            "heading_error_deg": math.degrees(heading_error),
            "excluded_following": excluded,
            "first_conflict_station": state["first_conflict_s"]}


def _static_refine(dense, headings, index, obstacle, shift, length, width,
                   margin, spread, c0, c1, count_exact, depth=0, a0=0.0, a1=1.0):
    def pose(fr):
        a, b = dense[index], dense[index + 1]
        x = a[0] + fr * (b[0] - a[0]); y = a[1] + fr * (b[1] - a[1])
        yaw = headings[index] + fr * _wrap(headings[index + 1] - headings[index])
        return x, y, yaw
    if abs(c0 - c1) >= spread:
        bound = min(c0, c1)
    else:
        bound = 0.5 * (c0 + c1 - spread)
    if bound >= 0.0 or depth >= 8 or c0 < 0.0 or c1 < 0.0:
        return bound
    mid = 0.5 * (a0 + a1)
    x, y, yaw = pose(mid)
    count_exact()
    cm = obb_signed_clearance(ObstacleOBB(x + shift * math.cos(yaw), y + shift * math.sin(yaw),
                                          yaw, length, width), obstacle) - margin
    if cm < 0.0:
        return cm
    return min(_static_refine(dense, headings, index, obstacle, shift, length, width, margin,
                              0.5 * spread, c0, cm, count_exact, depth + 1, a0, mid),
               _static_refine(dense, headings, index, obstacle, shift, length, width, margin,
                              0.5 * spread, cm, c1, count_exact, depth + 1, mid, a1))


def _segment_distance(a, b, c, d):
    """Minimum distance between segments ab and cd (2D)."""
    def point_segment(p, u, v):
        dx, dy = v[0] - u[0], v[1] - u[1]
        length_sq = dx * dx + dy * dy
        t = 0.0 if length_sq <= 1.0e-18 else max(0.0, min(1.0, (
            (p[0] - u[0]) * dx + (p[1] - u[1]) * dy) / length_sq))
        return math.hypot(p[0] - u[0] - t * dx, p[1] - u[1] - t * dy)

    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    if o1 * o2 < 0.0 and o3 * o4 < 0.0:
        return 0.0
    return min(point_segment(a, c, d), point_segment(b, c, d),
               point_segment(c, a, b), point_segment(d, a, b))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def swept_ego_body_clearance(path, obstacles, rear_extent, front_extent,
                             ego_width, speed_mps, safety_margin=0.0,
                             sample_step=0.2, work_budget=SWEEP_CALL_BUDGET,
                             tick_budget=None):
    """Certified lower bound of the body/object signed clearance along a
    path of ego REFERENCE points (rear axle by default), minus margin.

    Model: the reference moves along each resampled chord at constant speed
    (time = reference arc / max(0.1, speed)); heading varies linearly
    between the sample headings; objects move at constant velocity with
    constant yaw. The body spans rear_extent behind / front_extent ahead of
    the reference. Signed OBB clearance is 1-Lipschitz in the displacement
    of either body, so over a sample interval where no ego body point moves
    more than |dp| + reach*|dheading| and the object no more than |v|*dt,
    clearance >= max(c0 - a*D, c1 - (1-a)*D) >= (c0 + c1 - D) / 2. An
    interval whose bound falls below zero while both ends are clear is
    bisected (exact interpolated poses, depth <= 8). Pair values are the
    exact OBB clearance or, far away, the centre-distance lower bound. More
    than work_budget OBB evaluations -> invalid ('work-budget'). This is a
    bound for the stated model only (no perception, yaw-rate, acceleration
    or tracking error)."""
    dense = []
    for point in resample_polyline(path, sample_step):
        # Resampling can end with a point coincident with its predecessor;
        # its centred heading would read atan2(0, 0) = 0 (wrong pose).
        if not dense or math.hypot(point[0] - dense[-1][0],
                                   point[1] - dense[-1][1]) > 1.0e-9:
            dense.append(point)
    if len(dense) < 2:
        raise ValueError("path needs at least two distinct points")
    speed = max(0.1, abs(float(speed_mps)))
    count = len(dense)
    times = [0.0]
    for first, second in zip(dense, dense[1:]):
        times.append(times[-1] + math.hypot(
            second[0] - first[0], second[1] - first[1]) / speed)
    headings = []
    for index in range(count):
        before = max(0, index - 1)
        after = min(count - 1, index + 1)
        headings.append(math.atan2(dense[after][1] - dense[before][1],
                                   dense[after][0] - dense[before][0]))
    length = rear_extent + front_extent
    shift = 0.5 * (front_extent - rear_extent)
    ego_radius = math.hypot(0.5 * length, 0.5 * ego_width)
    reach = math.hypot(max(rear_extent, front_extent), 0.5 * ego_width)
    ego_move = [
        math.hypot(dense[i + 1][0] - dense[i][0],
                   dense[i + 1][1] - dense[i][1]) +
        reach * abs(_wrap(headings[i + 1] - headings[i]))
        for i in range(count - 1)]
    state = {"evaluations": 0, "minimum": float("inf"), "closest": None}

    def pose(index, fraction):
        if fraction == 0.0:
            return dense[index][0], dense[index][1], headings[index], times[index]
        a, b = dense[index], dense[index + 1]
        return (a[0] + fraction * (b[0] - a[0]),
                a[1] + fraction * (b[1] - a[1]),
                headings[index] + fraction * _wrap(
                    headings[index + 1] - headings[index]),
                times[index] + fraction * (times[index + 1] - times[index]))

    def exact(x, y, yaw, time_s, obstacle):
        state["evaluations"] += 1
        if state["evaluations"] > work_budget:
            raise OverflowError("work-budget")
        if tick_budget is not None:
            tick_budget.spend(1)      # revision2: charged incrementally (was one charge after the call)
        ego = ObstacleOBB(x + shift * math.cos(yaw), y + shift * math.sin(yaw),
                          yaw, length, ego_width)
        predicted = ObstacleOBB(
            obstacle.x + obstacle.vx * time_s, obstacle.y + obstacle.vy * time_s,
            obstacle.yaw, obstacle.length, obstacle.width, obstacle.vx,
            obstacle.vy, obstacle.kind, obstacle.object_id)
        return obb_signed_clearance(ego, predicted) - safety_margin

    def note(value, index, time_s, obstacle, between):
        if value != value:              # revision3: NaN clearance = conflict, never 'not negative'
            value = -float("inf")
        if value < state["minimum"]:
            state["minimum"] = value
            state["closest"] = {
                "path_index": index, "time_s": time_s,
                "object_id": obstacle.object_id, "kind": obstacle.kind,
                "clearance": value, "between_samples": between}

    def interval_bound(c0, c1, spread):
        if abs(c0 - c1) >= spread:
            return min(c0, c1)
        return 0.5 * (c0 + c1 - spread)

    def refine(index, a0, a1, c0, c1, spread, depth, obstacle):
        bound = interval_bound(c0, c1, spread)
        if bound >= 0.0 or depth >= 8 or c0 < 0.0 or c1 < 0.0:
            return bound
        middle = 0.5 * (a0 + a1)
        x, y, yaw, time_s = pose(index, middle)
        cm = exact(x, y, yaw, time_s, obstacle)
        if cm < 0.0:
            return cm
        return min(refine(index, a0, middle, c0, cm, 0.5 * spread, depth + 1, obstacle),
                   refine(index, middle, a1, cm, c1, 0.5 * spread, depth + 1, obstacle))

    try:
        for raw in obstacles:
            if isinstance(raw, ObstacleOBB):
                obstacle = raw
            elif isinstance(raw, dict):
                obstacle = ObstacleOBB(
                    x=float(raw["x"]), y=float(raw["y"]),
                    yaw=float(raw.get("yaw", 0.0)),
                    length=float(raw.get("length", 1.0)),
                    width=float(raw.get("width", 1.0)),
                    vx=float(raw.get("vx", 0.0)), vy=float(raw.get("vy", 0.0)),
                    kind=str(raw.get("kind", "Obstacle")),
                    object_id=raw.get("object_id"))
            else:
                raise TypeError("obstacle must be ObstacleOBB or dict")
            if not obstacle_finite(obstacle):
                # revision3: fail closed on any non-finite obstacle field
                state["minimum"] = -float("inf")
                state["closest"] = {"path_index": None, "time_s": None, "object_id": obstacle.object_id,
                                    "kind": "invalid-obstacle", "clearance": -float("inf"), "between_samples": False}
                break
            object_radius = math.hypot(0.5 * obstacle.length, 0.5 * obstacle.width)
            object_speed = math.hypot(obstacle.vx, obstacle.vy)
            spreads = [ego_move[i] + object_speed * (times[i + 1] - times[i])
                       for i in range(count - 1)]
            widest = max(spreads) if spreads else 0.0
            values = []
            if tick_budget is not None:
                tick_budget.spend(1 + count // 16)
            for index in range(count):
                x, y, yaw, time_s = pose(index, 0.0)
                cx, cy = x + shift * math.cos(yaw), y + shift * math.sin(yaw)
                px = obstacle.x + obstacle.vx * time_s
                py = obstacle.y + obstacle.vy * time_s
                lower = (math.hypot(cx - px, cy - py) - ego_radius -
                         object_radius - safety_margin)
                if lower > max(0.0, state["minimum"]) + widest + 1.0e-9:
                    values.append(lower)
                    continue
                value = exact(x, y, yaw, time_s, obstacle)
                values.append(value)
                note(value, index, time_s, obstacle, False)
            for index in range(count - 1):
                c0, c1 = values[index], values[index + 1]
                bound = interval_bound(c0, c1, spreads[index])
                if bound < 0.0 and c0 >= 0.0 and c1 >= 0.0:
                    bound = refine(index, 0.0, 1.0, c0, c1, spreads[index], 0, obstacle)
                if bound < min(c0, c1):
                    note(bound, index, times[index], obstacle, True)
    except OverflowError:
        return {"valid": False, "minimum_clearance": -float("inf"),
                "closest": {"path_index": None, "time_s": None,
                            "object_id": None, "kind": "work-budget",
                            "clearance": -float("inf"), "between_samples": False},
                "evaluations": state["evaluations"], "budget_exhausted": True}
    return {"valid": state["minimum"] >= 0.0,
            "minimum_clearance": state["minimum"],
            "closest": state["closest"],
            "evaluations": state["evaluations"], "budget_exhausted": False}


MAX_HEADING_DIFF = math.radians(70)
FALLBACK_HEADING_DIFF = math.radians(85)
TRAFFIC_RESTRICTIVE_STATES = frozenset(("red", "yellow"))
TRAFFIC_PROCEED_STATES = frozenset(("green", "green_left"))


def norm(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class TrafficLightFilter:
    """Turn YOLO JSON detections into a confirmed, expiring signal state."""

    def __init__(self, confirm_frames=2, timeout_s=1.0,
                 minimum_confidence=0.35, minimum_box_height_px=6.0,
                 roi_min_x_ratio=0.15, roi_max_x_ratio=0.85,
                 roi_min_y_ratio=0.0, roi_max_y_ratio=0.70,
                 track_center_gate_ratio=0.18):
        self.confirm_frames = max(1, int(confirm_frames))
        self.timeout_s = max(0.0, float(timeout_s))
        self.minimum_confidence = float(minimum_confidence)
        self.minimum_box_height_px = max(0.0, float(minimum_box_height_px))
        self.roi_min_x_ratio = float(roi_min_x_ratio)
        self.roi_max_x_ratio = float(roi_max_x_ratio)
        self.roi_min_y_ratio = float(roi_min_y_ratio)
        self.roi_max_y_ratio = float(roi_max_y_ratio)
        self.track_center_gate_ratio = max(
            0.0, float(track_center_gate_ratio))
        self.candidate = None
        self.candidate_count = 0
        self.stable = "unknown"
        self.stable_receipt_s = None
        self.locked_center_ratio = None

    def reset(self):
        """Forget signal state and the box associated with the last stop line."""
        self.candidate = None
        self.candidate_count = 0
        self.stable = "unknown"
        self.stable_receipt_s = None
        self.locked_center_ratio = None

    @staticmethod
    def _normalize_state(value):
        state = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
        aliases = {
            "red_light": "red", "traffic_red": "red", "stop": "red",
            "amber": "yellow", "yellow_light": "yellow",
            "green_light": "green", "traffic_green": "green",
            "left_green": "green_left", "greenarrow": "green_left",
            "green_arrow": "green_left",
        }
        state = aliases.get(state, state)
        return state if state in (TRAFFIC_RESTRICTIVE_STATES | TRAFFIC_PROCEED_STATES) else None

    def _best_detection_state(self, payload):
        if not isinstance(payload, dict):
            return None
        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            return None
        try:
            image_width = float(payload.get("source_width", 0.0))
        except (TypeError, ValueError):
            image_width = 0.0
        try:
            image_height = float(payload.get("source_height", 0.0))
        except (TypeError, ValueError):
            image_height = 0.0
        candidates = []
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            state = self._normalize_state(
                detection.get("traffic_light_state",
                              detection.get("model_class_name",
                                            detection.get("class_name"))))
            if state is None:
                continue
            try:
                confidence = float(detection.get("confidence", 0.0))
            except (TypeError, ValueError):
                continue
            if confidence < self.minimum_confidence:
                continue
            box = detection.get("bbox_xyxy")
            if not isinstance(box, list) or len(box) != 4:
                continue
            try:
                x1, y1, x2, y2 = (float(value) for value in box)
            except (TypeError, ValueError):
                continue
            width = max(0.0, x2 - x1)
            height = max(0.0, y2 - y1)
            if height < self.minimum_box_height_px:
                continue
            center_x = 0.5 * (x1 + x2)
            center_y = 0.5 * (y1 + y2)
            center_x_ratio = center_x / image_width if image_width > 0.0 else 0.5
            center_y_ratio = center_y / image_height if image_height > 0.0 else 0.5
            if image_width > 0.0 and not (
                    self.roi_min_x_ratio <= center_x_ratio <=
                    self.roi_max_x_ratio):
                continue
            if image_height > 0.0 and not (
                    self.roi_min_y_ratio <= center_y_ratio <=
                    self.roi_max_y_ratio):
                continue
            candidates.append({
                "area": width * height,
                "confidence": confidence,
                "state": state,
                "center": (center_x_ratio, center_y_ratio),
            })
        if not candidates:
            return None

        if self.locked_center_ratio is not None:
            locked_x, locked_y = self.locked_center_ratio
            tracked = []
            for candidate in candidates:
                center_x, center_y = candidate["center"]
                distance = math.hypot(
                    center_x - locked_x, center_y - locked_y)
                if distance <= self.track_center_gate_ratio:
                    tracked.append((distance, candidate))
            if not tracked:
                # Keep the association until the route planner disarms the
                # current stop line. Never jump to a newly visible opposite
                # signal merely because the selected light left the frame.
                return None
            _distance, selected = min(
                tracked,
                key=lambda item: (
                    item[0], -item[1]["area"], -item[1]["confidence"]))
        else:
            roi_center = 0.5 * (
                self.roi_min_x_ratio + self.roi_max_x_ratio)
            # Associate the first box with the driven-lane corridor. Size is
            # only a tie breaker, so a large side/opposite signal cannot win
            # over a signal centered on the current approach.
            selected = min(
                candidates,
                key=lambda item: (
                    abs(item["center"][0] - roi_center),
                    -item["area"], -item["confidence"]))

        self.locked_center_ratio = selected["center"]
        return selected["state"]

    def update(self, payload, receipt_s):
        state = self._best_detection_state(payload)
        if state is None:
            self.candidate = None
            self.candidate_count = 0
            return self.current(receipt_s)
        if state == self.candidate:
            self.candidate_count += 1
        else:
            self.candidate = state
            self.candidate_count = 1
        if self.candidate_count >= self.confirm_frames:
            self.stable = state
            self.stable_receipt_s = float(receipt_s)
        return self.current(receipt_s)

    def current(self, now_s):
        if self.stable_receipt_s is None:
            return "unknown"
        age = float(now_s) - self.stable_receipt_s
        if not math.isfinite(age) or age < 0.0 or age > self.timeout_s:
            return "unknown"
        return self.stable


class Link:
    __slots__ = (
        "idx", "fr", "to", "pts", "lane", "road",
        "can_l", "can_r", "dst_l", "dst_r", "length", "vmax",
        "link_type", "width_start", "width_end",
        "lane_mark_left", "lane_mark_right",
    )

    def __init__(self, data):
        self.idx = data["idx"]
        self.fr = data["from_node_idx"]
        self.to = data["to_node_idx"]
        self.pts = [(p[0], p[1]) for p in data["points"]]
        self.lane = data.get("ego_lane")
        self.road = data.get("road_id")
        self.link_type = str(data.get("link_type") or "")
        self.width_start = float(data.get("width_start") or 3.5)
        self.width_end = float(data.get("width_end") or self.width_start)
        self.lane_mark_left = tuple(data.get("lane_mark_left") or ())
        self.lane_mark_right = tuple(data.get("lane_mark_right") or ())
        self.can_l = bool(data.get("can_move_left_lane"))
        self.can_r = bool(data.get("can_move_right_lane"))
        self.dst_l = data.get("left_lane_change_dst_link_idx")
        self.dst_r = data.get("right_lane_change_dst_link_idx")
        self.vmax = (data.get("max_speed") or 50) / 3.6
        self.length = data.get("link_length") or sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(self.pts, self.pts[1:])
        )


class HdmapLaneChange:
    def __init__(self):
        rospy.init_node("hdmap_lane_change")
        hdmap_zip = rospy.get_param(
            "~hdmap_zip", "/home/david/morai-mpc-agent-morai-lio-gps-integration/src/moraimpc/data/hdmap.zip"
        )
        directional_map = rospy.get_param(
            "~directional_map", "")
        lane_policy_file = rospy.get_param(
            "~lane_policy_file", "")
        local_bypass_corridor_file = rospy.get_param(
            "~local_bypass_corridor_file", "")
        self.cruise_mps = float(rospy.get_param("~cruise_mps", 9.0))
        self.route_len = float(rospy.get_param("~route_len_m", 180.0))
        self.transition_m = float(rospy.get_param("~lane_change_trans_m", 28.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.arrive_lat_m = float(rospy.get_param("~arrive_lat_m", 0.8))
        self.arrive_lane_lat_m = float(rospy.get_param("~arrive_lane_lat_m", max(self.arrive_lat_m, 1.5)))
        self.arrive_forced_lane_lat_m = float(rospy.get_param("~arrive_forced_lane_lat_m", self.arrive_lat_m))
        self.path_file = rospy.get_param("~path_file", "")
        self.base_route_file = rospy.get_param("~base_route_file", self.path_file)
        self.use_base_route = bool(rospy.get_param("~use_base_route", True))
        self.loop_hdmap_route = bool(rospy.get_param("~loop_hdmap_route", True))
        self.max_forward_links = int(rospy.get_param("~max_forward_links", 500))
        self.anchor_current_pose = bool(rospy.get_param("~anchor_current_pose", False))
        self.auto_avoid = bool(rospy.get_param("~auto_avoid", True))
        self.detect_dist_m = float(rospy.get_param("~detect_dist_m", 45.0))
        self.block_lat_m = float(rospy.get_param("~block_lat_m", 1.7))
        self.target_front_gap_m = float(rospy.get_param("~target_front_gap_m", 18.0))
        self.target_rear_gap_m = float(rospy.get_param("~target_rear_gap_m", 12.0))
        self.gap_front_min_m = float(rospy.get_param("~gap_front_min_m", 7.0))
        self.gap_rear_min_m = float(rospy.get_param("~gap_rear_min_m", 5.0))
        self.gap_front_time_s = float(rospy.get_param("~gap_front_time_s", 0.55))
        self.gap_rear_time_s = float(rospy.get_param("~gap_rear_time_s", 0.45))
        self.gap_rel_speed_time_s = float(rospy.get_param("~gap_rel_speed_time_s", 0.7))
        self.gap_predict_time_s = float(rospy.get_param("~gap_predict_time_s", 1.8))
        self.gap_compare_epsilon_m = max(
            0.0, float(rospy.get_param("~gap_compare_epsilon_m", 0.10)))
        self.clear_cycles_needed = int(rospy.get_param("~clear_cycles_needed", 8))
        self.avoid_source_pass_margin_m = max(
            0.0, float(rospy.get_param("~avoid_source_pass_margin_m", 6.0)))
        self.brake_decel = float(rospy.get_param("~brake_decel", 5.0))
        self.front_safety_gap = float(rospy.get_param("~front_safety_gap_m", 7.0))
        self.front_safety_lat_m = float(rospy.get_param("~front_safety_lat_m", 3.0))
        self.lane_change_collision_lat_m = float(
            rospy.get_param("~lane_change_collision_lat_m", 2.4))
        self.vulnerable_safety_gap = float(
            rospy.get_param("~vulnerable_safety_gap_m", 12.0))
        self.vulnerable_lateral_m = float(
            rospy.get_param("~vulnerable_lateral_m", 3.5))
        self.vulnerable_brake_decel = float(
            rospy.get_param("~vulnerable_brake_decel_mps2", 3.0))
        self.vulnerable_follow_gain = float(
            rospy.get_param("~vulnerable_follow_gain", 0.40))
        self.obstacle_follow_gain = float(
            rospy.get_param("~obstacle_follow_gain", 0.35))
        self.speed_cap_rise_mps2 = max(
            0.1, float(rospy.get_param("~speed_cap_rise_mps2", 1.0)))
        self.crossing_predict_time_s = max(
            0.5, float(rospy.get_param("~crossing_predict_time_s", 4.0)))
        self.crossing_predict_step_s = max(
            0.05, float(rospy.get_param("~crossing_predict_step_s", 0.2)))
        self.crossing_time_margin_s = max(
            0.2, float(rospy.get_param("~crossing_time_margin_s", 2.0)))
        self.object_heading_diff = float(rospy.get_param("~object_heading_diff_deg", 100.0))
        self.require_rejoin_path = bool(rospy.get_param("~require_rejoin_path", True))
        self.rejoin_check_m = float(rospy.get_param("~rejoin_check_m", 140.0))
        self.urgent_transition_min_m = float(rospy.get_param("~urgent_transition_min_m", 10.0))
        self.urgent_transition_margin_m = float(rospy.get_param("~urgent_transition_margin_m", 4.0))
        self.stopped_avoid_blend_m = max(
            6.0, float(rospy.get_param("~stopped_avoid_blend_m", 10.0)))
        self.stopped_avoid_trigger_m = max(
            self.stopped_avoid_blend_m,
            float(rospy.get_param("~stopped_avoid_trigger_m", 10.0)))
        self.avoid_change_min_mps = float(rospy.get_param("~avoid_change_min_kmh", 6.0)) / 3.6
        self.avoid_change_time_s = max(
            1.0, float(rospy.get_param("~avoid_change_time_s", 2.5)))
        self.straight_lane_change_enable = bool(rospy.get_param(
            "~straight_lane_change_enable", True))
        self.straight_lane_change_lookahead_m = max(
            15.0, float(rospy.get_param(
                "~straight_lane_change_lookahead_m", 45.0)))
        self.straight_lane_change_max_heading = math.radians(max(
            1.0, float(rospy.get_param(
                "~straight_lane_change_max_heading_deg", 5.0))))
        self.straight_lane_change_blend_m = max(
            12.0, float(rospy.get_param(
                "~straight_lane_change_blend_m", 20.0)))
        self.straight_lane_change_time_s = max(
            1.0, float(rospy.get_param(
                "~straight_lane_change_time_s", 1.3)))
        self.straight_fast_min_gap_margin_m = max(
            0.0, float(rospy.get_param(
                "~straight_fast_min_gap_margin_m", 2.0)))
        self.avoid_hard_stop_m = float(rospy.get_param("~avoid_hard_stop_m", 4.0))
        self.avoid_hold_max_mps = float(
            rospy.get_param("~avoid_hold_max_kmh", 20.0)) / 3.6
        self.max_avoid_hops = int(rospy.get_param("~max_avoid_hops", 4))
        self.ego_timeout_s = float(rospy.get_param("~ego_timeout_s", 0.5))
        self.base_reacquire_dist_m = float(rospy.get_param("~base_reacquire_dist_m", 15.0))
        self.base_search_ahead_points = int(rospy.get_param("~base_search_ahead_points", 300))
        self.base_heading_match_margin_m = float(
            rospy.get_param("~base_heading_match_margin_m", 5.0))
        self.global_rejoin_lat_m = float(rospy.get_param("~global_rejoin_lat_m", 0.7))
        self.global_rejoin_trigger_m = float(
            rospy.get_param("~global_rejoin_trigger_m", 1.5))
        self.global_rejoin_trigger_cycles_needed = int(
            rospy.get_param("~global_rejoin_trigger_cycles_needed", 5))
        self.global_rejoin_max_dist_m = float(
            rospy.get_param("~global_rejoin_max_dist_m", 8.0))
        self.global_rejoin_hard_stop_dist_m = float(
            rospy.get_param("~global_rejoin_hard_stop_dist_m", 20.0))
        self.global_rejoin_recovery_mps = float(
            rospy.get_param("~global_rejoin_recovery_kmh", 5.0)) / 3.6
        self.global_rejoin_max_mps = float(
            rospy.get_param("~global_rejoin_max_kmh", 10.0)) / 3.6
        self.global_rejoin_cycles_needed = int(
            rospy.get_param("~global_rejoin_cycles_needed", 8))
        self.global_rejoin_no_lane_max_heading = math.radians(float(
            rospy.get_param("~global_rejoin_no_lane_max_heading_deg", 85.0)))
        self.rejoin_route_refresh_m = float(
            rospy.get_param("~rejoin_route_refresh_m", 3.0))
        self.object_timeout_s = float(rospy.get_param("~object_timeout_s", 0.5))
        self.object_hard_timeout_s = max(
            self.object_timeout_s,
            float(rospy.get_param("~object_hard_timeout_s", 2.0)))
        self.perception_degraded_mps = float(
            rospy.get_param("~perception_degraded_kmh", 10.0)) / 3.6
        self.require_object_feed = bool(rospy.get_param("~require_object_feed", False))
        self.intersection_bypass_enable = bool(
            rospy.get_param("~intersection_bypass_enable", True))
        self.intersection_bypass_offset_m = float(
            rospy.get_param("~intersection_bypass_offset_m", 3.2))
        self.intersection_bypass_clearance_m = float(
            rospy.get_param("~intersection_bypass_clearance_m", 0.35))
        self.path_block_clearance_m = max(
            0.0, float(rospy.get_param(
                "~path_block_clearance_m", 0.35)))
        self.intersection_bypass_min_object_m = float(
            rospy.get_param("~intersection_bypass_min_object_m", 8.0))
        self.intersection_bypass_static_mps = float(
            rospy.get_param("~intersection_bypass_static_mps", 0.8))
        self.intersection_bypass_max_mps = float(
            rospy.get_param("~intersection_bypass_max_kmh", 25.0)) / 3.6
        self.bypass_vehicle_length = max(
            3.0, float(rospy.get_param("~bypass_vehicle_length_m", 4.635)))
        self.bypass_vehicle_width = max(
            1.5, float(rospy.get_param("~bypass_vehicle_width_m", 1.892)))
        self.bypass_wheelbase = max(
            2.0, float(rospy.get_param("~bypass_wheelbase_m", 3.0)))
        # Ego pose / published path reference point. MORAI's vehicle origin
        # is the rear-axle midpoint; "body_center" keeps the old centred
        # footprint for compatibility only.
        self.ego_reference_point = str(rospy.get_param(
            "~ego_reference_point", "rear_axle"))
        self.bypass_front_overhang = float(rospy.get_param(
            "~bypass_front_overhang_m", 0.845))
        self.bypass_rear_overhang = float(rospy.get_param(
            "~bypass_rear_overhang_m", 0.790))
        self.bypass_max_steer_deg = max(
            10.0, float(rospy.get_param("~bypass_max_steer_deg", 35.0)))
        self.bypass_max_steer_rate_deg_s = max(
            10.0, float(rospy.get_param(
                "~bypass_max_steer_rate_deg_s", 50.0)))
        self.bypass_max_lateral_accel = max(
            0.5, float(rospy.get_param(
                "~bypass_max_lateral_accel_mps2", 2.26)))
        self.lane_change_preflight_margin = max(
            0.2, float(rospy.get_param(
                "~lane_change_preflight_margin_m", 0.50)))
        self.delayed_lane_change_max_delay = max(
            0.0, float(rospy.get_param(
                "~delayed_lane_change_max_delay_m", 16.0)))
        self.delayed_lane_change_delay_step = max(
            0.5, float(rospy.get_param(
                "~delayed_lane_change_delay_step_m", 1.0)))
        self.delayed_lane_change_min_transition = max(
            6.0, float(rospy.get_param(
                "~delayed_lane_change_min_transition_m", 8.0)))
        self.delayed_lane_change_max_transition = max(
            self.delayed_lane_change_min_transition,
            float(rospy.get_param(
                "~delayed_lane_change_max_transition_m", 20.0)))
        # Bound only the obstacle-avoidance search. Normal CRUISE planning is
        # intentionally unchanged from the stable baseline.
        self.max_avoid_candidates = max(1, int(rospy.get_param(
            "~max_avoid_candidates", 3)))
        self.max_delayed_transition_candidates = max(1, int(
            rospy.get_param("~max_delayed_transition_candidates", 5)))
        self.free_bypass_enable = bool(
            rospy.get_param("~free_bypass_enable", True))
        # gap passage (free intervals between static objects; see _gap_interval_candidates). Non-finite or
        # out-of-range values fall back to the defaults (never widen the search or raise the speed).
        def _gap_param(name, default, lo, hi):
            try:
                value = float(rospy.get_param("~" + name, default))
            except (TypeError, ValueError):
                return default
            return value if math.isfinite(value) and lo <= value <= hi else default
        self.gap_passage_enable = bool(rospy.get_param("~gap_passage_enable", True))
        self.gap_passage_max_mps = _gap_param("gap_passage_max_mps", 2.0, 0.5, 2.0)
        self.gap_group_join_m = _gap_param("gap_group_join_m", 12.0, 0.0, 30.0)
        self.gap_group_max_objects = int(_gap_param("gap_group_max_objects", 8, 1, 8))
        self.gap_max_candidates = int(_gap_param("gap_max_candidates", 4, 1, 4))
        self.gap_max_objects_scanned = int(_gap_param("gap_max_objects_scanned", 64, 1, 64))
        self.gap_road_margin_m = _gap_param("gap_road_margin_m", 0.15, 0.15, 1.0)
        self.gap_exit_length_m = _gap_param("gap_exit_length_m", 15.0, 10.0, 30.0)
        self.gap_goal_after_exit_m = _gap_param("gap_goal_after_exit_m", 6.0, 0.0, 10.0)
        self.free_bypass_lateral_bias_m = float(
            rospy.get_param("~free_bypass_lateral_bias_m", 0.45))
        self.free_bypass_max_shift_m = float(
            rospy.get_param("~free_bypass_max_shift_m", 7.5))
        self.free_bypass_max_mps = float(
            rospy.get_param("~free_bypass_max_kmh", 25.0)) / 3.6
        self.straight_free_bypass_max_mps = float(rospy.get_param(
            "~straight_free_bypass_max_kmh", 35.0)) / 3.6
        self.local_bypass_nudge_max_m = max(
            0.2, float(rospy.get_param("~local_bypass_nudge_max_m", 1.6)))
        self.local_bypass_nudge_step_m = max(
            0.1, float(rospy.get_param("~local_bypass_nudge_step_m", 0.2)))
        self.local_bypass_tracking_margin = max(
            0.0, float(rospy.get_param(
                "~local_bypass_tracking_margin_m", 0.15)))
        self.local_bypass_gap_max_mps = float(
            rospy.get_param("~local_bypass_gap_max_kmh", 25.0)) / 3.6
        self.marked_edge_bypass_enable = bool(rospy.get_param(
            "~marked_edge_bypass_enable", False))
        self.marked_edge_bypass_max_object_width = max(
            0.4, float(rospy.get_param(
                "~marked_edge_bypass_max_object_width_m", 1.4)))
        self.marked_edge_bypass_max_object_length = max(
            0.4, float(rospy.get_param(
                "~marked_edge_bypass_max_object_length_m", 2.5)))
        self.marked_edge_bypass_max_offset = max(
            1.5, float(rospy.get_param(
                "~marked_edge_bypass_max_offset_m", 2.4)))
        self.marked_edge_bypass_tracking_margin = max(
            0.0, float(rospy.get_param(
                "~marked_edge_bypass_tracking_margin_m", 0.5)))
        self.marked_edge_bypass_min_object = max(
            1.5, float(rospy.get_param(
                "~marked_edge_bypass_min_object_m", 6.5)))
        self.marked_edge_bypass_max_mps = float(rospy.get_param(
            "~marked_edge_bypass_max_kmh", 15.0)) / 3.6
        self.backup_recovery_enable = bool(rospy.get_param(
            "~backup_recovery_enable", True))
        self.backup_recovery_distance_m = max(
            2.0, float(rospy.get_param(
                "~backup_recovery_distance_m", 5.0)))
        self.backup_recovery_trigger_m = max(
            2.0, float(rospy.get_param(
                "~backup_recovery_trigger_m", 9.0)))
        self.backup_recovery_wait_cycles = max(
            3, int(rospy.get_param(
                "~backup_recovery_wait_cycles", 15)))
        self.backup_recovery_max_mps = max(
            0.3, float(rospy.get_param(
                "~backup_recovery_max_kmh", 4.0)) / 3.6)
        self.backup_recovery_rear_clear_m = max(
            self.backup_recovery_distance_m + 1.0,
            float(rospy.get_param(
                "~backup_recovery_rear_clear_m", 8.0)))
        self.backup_recovery_rear_lateral_m = max(
            1.0, float(rospy.get_param(
                "~backup_recovery_rear_lateral_m", 1.8)))
        self.backup_recovery_timeout_cycles = max(
            30, int(rospy.get_param(
                "~backup_recovery_timeout_cycles", 120)))
        self.backup_recovery_start_timeout_cycles = max(
            30, int(rospy.get_param(
                "~backup_recovery_start_timeout_cycles", 150)))
        self.backup_recovery_drive_wait_cycles = max(
            10, int(rospy.get_param(
                "~backup_recovery_drive_wait_cycles", 60)))
        self.backup_recovery_rearm_distance_m = max(
            1.0, float(rospy.get_param(
                "~backup_recovery_rearm_distance_m", 3.0)))
        self.centerline_guard_margin_m = max(
            0.5, float(rospy.get_param(
                "~centerline_guard_margin_m", 1.30)))
        self.centerline_guard_min_opposite_heading = math.radians(max(
            90.0, float(rospy.get_param(
                "~centerline_guard_min_opposite_heading_deg", 145.0))))
        self.white_lane_max_separation_m = max(
            3.0, float(rospy.get_param(
                "~white_lane_max_separation_m", 5.0)))
        self.allow_opposite_last_resort = bool(
            rospy.get_param("~allow_opposite_last_resort", False))
        self.opposite_wait_cycles_needed = int(
            rospy.get_param("~opposite_wait_cycles", 20))
        self.opposite_nearly_stopped_mps = float(
            rospy.get_param("~opposite_nearly_stopped_mps", 0.5))
        self.opposite_max_mps = float(
            rospy.get_param("~opposite_max_kmh", 5.0)) / 3.6
        self.opposite_clear_ahead_m = float(
            rospy.get_param("~opposite_clear_ahead_m", 70.0))
        self.opposite_clear_behind_m = float(
            rospy.get_param("~opposite_clear_behind_m", 20.0))
        self.opposite_corridor_lat_m = float(
            rospy.get_param("~opposite_corridor_lat_m", 2.2))
        self.opposite_max_separation_m = float(
            rospy.get_param("~opposite_max_separation_m", 5.0))
        self.opposite_min_overlap_m = float(
            rospy.get_param("~opposite_min_overlap_m", 20.0))
        self.traffic_stop_enable = bool(
            rospy.get_param("~traffic_stop_enable", False))
        self.traffic_detection_topic = str(rospy.get_param(
            "~traffic_detection_topic",
            "/camera_traffic_light_detection/detections"))
        self.traffic_stop_lookahead_m = float(
            rospy.get_param("~traffic_stop_lookahead_m", 70.0))
        self.traffic_stop_route_tolerance_m = float(
            rospy.get_param("~traffic_stop_route_tolerance_m", 2.2))
        self.traffic_stop_margin_m = float(
            rospy.get_param("~traffic_stop_margin_m", 2.0))
        self.traffic_stop_hard_distance_m = float(
            rospy.get_param("~traffic_stop_hard_distance_m", 0.8))
        self.traffic_stop_decel_mps2 = float(
            rospy.get_param("~traffic_stop_decel_mps2", 2.5))
        self.traffic_stop_signalized_only = bool(
            rospy.get_param("~traffic_stop_signalized_only", False))
        self.traffic_unknown_is_stop = bool(
            rospy.get_param("~traffic_unknown_is_stop", False))
        self.traffic_yellow_is_stop = bool(
            rospy.get_param("~traffic_yellow_is_stop", True))
        self.traffic_green_left_is_go = bool(
            rospy.get_param("~traffic_green_left_is_go", False))
        self.traffic_pass_release_m = float(
            rospy.get_param("~traffic_pass_release_m", 8.0))
        self.traffic_filter = TrafficLightFilter(
            confirm_frames=rospy.get_param("~traffic_confirm_frames", 2),
            timeout_s=rospy.get_param("~traffic_detection_timeout_s", 1.0),
            minimum_confidence=rospy.get_param(
                "~traffic_minimum_confidence", 0.35),
            minimum_box_height_px=rospy.get_param(
                "~traffic_minimum_box_height_px", 6.0),
            roi_min_x_ratio=rospy.get_param(
                "~traffic_roi_min_x_ratio", 0.15),
            roi_max_x_ratio=rospy.get_param(
                "~traffic_roi_max_x_ratio", 0.85),
            roi_min_y_ratio=rospy.get_param(
                "~traffic_roi_min_y_ratio", 0.0),
            roi_max_y_ratio=rospy.get_param(
                "~traffic_roi_max_y_ratio", 0.70),
            track_center_gate_ratio=rospy.get_param(
                "~traffic_track_center_gate_ratio", 0.18),
        )
        self.diagnostics_csv = os.path.abspath(os.path.expanduser(
            rospy.get_param("~diagnostics_csv", "")))

        self.links = {}
        self.stop_line_nodes = []
        self.stop_line_by_id = {}
        self.adj = defaultdict(list)
        self.reverse_adj = defaultdict(list)
        self.directional_same = {}
        self.directional_inferred_same = {}
        self.directional_opposite = {}
        self.link_policies = {}
        self.local_bypass_corridors = {}
        self._load_map(hdmap_zip)
        self._load_directional_map(directional_map)
        self._load_lane_policies(lane_policy_file)
        self._load_local_bypass_corridors(local_bypass_corridor_file)
        self.base_route = self._load_base_route(self.base_route_file)

        self.ego = None
        self.ego_receipt = None
        # Callbacks publish immutable packets; _plan snapshots them once per
        # tick into the planner-owned fields below (see _take_frame).
        self._ego_packet = None
        self._obj_packet = None
        self._ego_version = 0
        self._obj_version = 0
        self._tick_frame = (None, None)
        # revision3 clock epochs (callbacks and the planner thread share them under _clock_lock)
        self._clock_lock = threading.Lock()
        self._clock_epoch = 0
        self._clock_last = None
        self._clock_discard = {}
        self._clock_resets = []
        self.clock_tolerance_s = float(rospy.get_param("~clock_tolerance_s", CLOCK_TOLERANCE_S))
        if not (math.isfinite(self.clock_tolerance_s) and 0.0 <= self.clock_tolerance_s <= 0.05):
            self.clock_tolerance_s = CLOCK_TOLERANCE_S
        self._init_feed_timeout_policy()
        self.ego_v = 0.0
        self.ego_signed_v = 0.0
        self.vehicle_gear = 4
        self.vehicle_status_receipt = None
        self.ego_history = deque(maxlen=300)
        self.objects = []
        self.objects_receipt = None
        self.last_speed_command_mps = None
        self.ref_link = None
        self.last_link = None
        self.target_lane = int(rospy.get_param("~target_lane", 0))
        self.active_target_lane = None
        self.change_dst_idx = None
        self.changing = False
        self.mode = "CRUISE"
        self.home_lane = None
        self.clear_cycles = 0
        self.avoid_source = None
        self.auto_speed_cap = None
        self.change_blend_m = self.transition_m
        self.avoid_hops = 0
        self.base_progress = None
        self.base_match_distance = None
        self.base_match_heading_error = None
        self.base_match_projection = None
        self.base_match_fraction = 0.0
        self.global_rejoin_trigger_cycles = 0
        self.rejoin_stable_cycles = 0
        self.rejoin_route = []
        self.rejoin_refresh_count = 0
        self.rejoin_refresh_last = None
        self.source_return = None
        self.local_bypass_route = []
        self.local_bypass_progress = 0
        self.local_bypass_goal = None
        self.local_bypass_source = None
        self.local_bypass_max_mps = getattr(
            self, "intersection_bypass_max_mps", 15.0 / 3.6)
        self.local_bypass_replan_count = 0
        self.path_revision = 0
        self.last_local_decision = ""
        self.backup_wait_cycles = 0
        self.backup_phase = None
        self.backup_route = []
        self.backup_start = None
        self.backup_cycles = 0
        self.backup_drive_wait = 0
        self.backup_source = None
        self.backup_completed_source = None
        self.backup_motion_started = False
        self.backup_motion_cycles = 0
        self.backup_stall_cycles = 0
        self.backup_travelled_m = 0.0
        self.backup_last_pose = None
        self.backup_fault = None
        self.backup_pending_bypass = None
        self.output_gear = "D"
        self.opposite_wait_cycles = 0
        self.contraflow_active = False
        self.contraflow_link_idx = None
        self.contraflow_source_idx = None
        self.traffic_target_node_id = None
        self.traffic_hold_active = False
        self.traffic_detection_armed = False
        self.diag_stream = None
        self.diag_writer = None
        self.diag_tick = 0
        self.diag_lock = threading.Lock()
        self._open_diagnostics()

        rospy.Subscriber("/localization/ego_status",
                         EgoVehicleStatus, self._ego_cb, queue_size=1)
        rospy.Subscriber("/Competition_topic",
                         EgoVehicleStatus, self._vehicle_status_cb,
                         queue_size=1)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self._obj_cb, queue_size=1)
        rospy.Subscriber("/lane_change_cmd", Int32, self._cmd_cb, queue_size=1)
        if self.traffic_stop_enable:
            rospy.Subscriber(
                self.traffic_detection_topic, String,
                self._traffic_cb, queue_size=1)
        self.pub_wps = rospy.Publisher("/avoid_waypoints", String, queue_size=1)
        self.pub_vel = rospy.Publisher("/avoid_target_vel", Float32, queue_size=1)
        self.pub_path = rospy.Publisher("/avoid_path", Path, queue_size=1)
        self.pub_status = rospy.Publisher("/lane_change_status", String, queue_size=1)
        self.pub_mode = rospy.Publisher("/lane_change_mode", String, queue_size=1)
        self.pub_diagnostics = rospy.Publisher(
            "/avoidance/diagnostics", String, queue_size=1)
        self.pub_debug_markers = rospy.Publisher(
            "/avoidance/debug_markers", MarkerArray, queue_size=1)
        self.pub_traffic = rospy.Publisher(
            "/traffic_stop_status", String, queue_size=1)

        rospy.loginfo("[LaneChange] ON - link=%d stop_nodes=%d route_wp=%d base_route=%s hdmap_loop=%s ego_anchor=%s auto_avoid=%s target_lane=%d directional=%s opposite_last_resort=%s traffic_stop=%s traffic_topic=%s",
                      len(self.links), len(self.stop_line_nodes), len(self.base_route),
                      "ON" if self.use_base_route else "OFF",
                      "ON" if self.loop_hdmap_route else "OFF",
                      "ON" if self.anchor_current_pose else "OFF",
                      "ON" if self.auto_avoid else "OFF", self.target_lane,
                      "ON" if self.directional_same else "OFF",
                      "ON" if self.allow_opposite_last_resort else "OFF",
                      "ON" if self.traffic_stop_enable else "OFF",
                      self.traffic_detection_topic)

    def _load_map(self, hdmap_zip):
        with zipfile.ZipFile(hdmap_zip) as zf:
            raw_nodes = json.loads(zf.read("node_set.json"))
            raw_links = json.loads(zf.read("link_set.json"))
        signalized_nodes = {
            data.get("to_node_idx") for data in raw_links
            if data.get("related_signal")
        }
        for data in raw_nodes:
            point = data.get("point", [])
            if not data.get("on_stop_line") or len(point) < 2:
                continue
            node = {
                "idx": str(data["idx"]),
                "point": (float(point[0]), float(point[1])),
                "traffic_light_id": data.get("traffic_light_id"),
                "signalized": bool(
                    data.get("traffic_light_id") or
                    data.get("idx") in signalized_nodes),
            }
            self.stop_line_nodes.append(node)
            self.stop_line_by_id[node["idx"]] = node
        for data in raw_links:
            link = Link(data)
            if len(link.pts) < 2 or link.lane is None:
                continue
            self.links[link.idx] = link
            self.adj[link.fr].append(link)
            self.reverse_adj[link.to].append(link)

    def _load_directional_map(self, path):
        if not path:
            rospy.logwarn("[LaneChange] directional map disabled; raw HD-map lateral flags used")
            return
        path = os.path.abspath(os.path.expanduser(path))
        try:
            opener = gzip.open if path.endswith(".gz") else open
            with opener(path, "rt", encoding="utf-8") as stream:
                payload = json.load(stream)
            if payload.get("format") != "morai_directional_drivable_map":
                raise ValueError("unexpected format")
            edges = payload.get("edges", {})
            self.directional_same = edges.get(
                "legal_same_direction_lateral", {})
            self.directional_inferred_same = edges.get(
                "inferred_same_direction_parallel", {})
            self.directional_opposite = edges.get(
                "inferred_opposite_direction_parallel", {})
            rospy.loginfo(
                "[LaneChange] directional map loaded: same=%d opposite-last-resort=%d path=%s",
                sum(len(items) for items in self.directional_same.values()),
                sum(len(items) for items in self.directional_opposite.values()),
                path)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logerr("[LaneChange] directional map load failed: %s", exc)
            self.directional_same = {}
            self.directional_inferred_same = {}
            self.directional_opposite = {}

    def _load_lane_policies(self, path):
        if not path:
            rospy.logwarn("[LaneChange] lane policy overrides disabled")
            return
        path = os.path.abspath(os.path.expanduser(path))
        try:
            with open(path, encoding="utf-8") as stream:
                payload = json.load(stream)
            if payload.get("format") != "morai_lane_policy_overrides":
                raise ValueError("unexpected format")
            valid = {"normal", "avoidance", "last_resort", "forbidden"}
            policies = {}
            rejected = 0
            for link_id, entry in payload.get("links", {}).items():
                link = self.links.get(link_id)
                policy = entry.get("policy")
                if link is None or policy not in valid:
                    rejected += 1
                    continue
                lane_no = entry.get("lane_no")
                road_id = str(entry.get("road_id") or "")
                if ((lane_no is not None and int(lane_no) != int(link.lane)) or
                        (road_id and road_id != str(link.road))):
                    rejected += 1
                    rospy.logerr(
                        "[LaneChange] policy metadata mismatch ignored: %s",
                        link_id)
                    continue
                policies[link_id] = policy
            self.link_policies = policies

            same_targets = {
                edge.get("target_link_id")
                for edges in self.directional_same.values()
                for edge in edges
            }
            opposite_targets = {
                edge.get("target_link_id")
                for edges in self.directional_opposite.values()
                for edge in edges
            }
            direction_conflicts = [
                link_id for link_id, policy in policies.items()
                if ((link_id in opposite_targets and policy in ("normal", "avoidance")) or
                    (link_id in same_targets and policy == "last_resort"))
            ]
            for link_id in direction_conflicts:
                rospy.logwarn(
                    "[LaneChange] policy direction conflict kept SAFE/unused: %s policy=%s",
                    link_id, policies[link_id])
            counts = {
                name: sum(value == name for value in policies.values())
                for name in sorted(valid)
            }
            rospy.loginfo(
                "[LaneChange] lane policies loaded: total=%d normal=%d avoidance=%d last_resort=%d forbidden=%d rejected=%d conflicts=%d path=%s",
                len(policies), counts["normal"], counts["avoidance"],
                counts["last_resort"], counts["forbidden"], rejected,
                len(direction_conflicts), path)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logerr("[LaneChange] lane policy load failed: %s", exc)
            self.link_policies = {}

    def _load_local_bypass_corridors(self, path):
        """Load explicitly surveyed intersection corridors.

        A missing or malformed file must fail closed: intersection connectors
        without a valid entry remain stop-only when no mapped lateral lane is
        available.
        """
        self.local_bypass_corridors = {}
        if not path:
            rospy.logwarn("[LaneChange] local bypass corridors disabled")
            return
        path = os.path.abspath(os.path.expanduser(path))
        try:
            with open(path, encoding="utf-8") as stream:
                payload = json.load(stream)
            if payload.get("format") != "morai_local_bypass_corridors":
                raise ValueError("unexpected format")
            if payload.get("coordinate_frame") != "map":
                raise ValueError("coordinate_frame must be map")
            corridors = {}
            rejected = 0
            for raw in payload.get("corridors", []):
                try:
                    link_id = str(raw["link_id"])
                    link = self.links[link_id]
                    sides = tuple(sorted(set(raw["allowed_sides"])))
                    maximum_offset = float(raw["maximum_center_offset_m"])
                    speed_kmh = float(raw["maximum_speed_kmh"])
                    clearance = float(raw["minimum_body_clearance_m"])
                    max_curvature = float(raw["maximum_curvature_inv_m"])
                    entry_margin = float(raw.get("entry_margin_m", 0.0))
                    exit_margin = float(raw.get("exit_margin_m", 0.0))
                    if (str(getattr(link, "link_type", "")) != "1" or
                            not sides or
                            any(side not in ("left", "right") for side in sides) or
                            not 0.5 <= maximum_offset <= 4.0 or
                            not 0.0 < speed_kmh <= 20.0 or
                            not 0.2 <= clearance <= 1.5 or
                            not 0.03 <= max_curvature <= 0.30 or
                            entry_margin < 0.0 or exit_margin < 0.0 or
                            not bool(raw.get("enabled", True))):
                        raise ValueError("unsafe corridor limits")
                    corridors[link_id] = {
                        "id": str(raw.get("id") or link_id),
                        "link_id": link_id,
                        "allowed_sides": sides,
                        "maximum_center_offset_m": maximum_offset,
                        "maximum_speed_mps": speed_kmh / 3.6,
                        "minimum_body_clearance_m": clearance,
                        "maximum_curvature_inv_m": max_curvature,
                        "entry_margin_m": entry_margin,
                        "exit_margin_m": exit_margin,
                        "multi_obstacle": bool(
                            raw.get("multi_obstacle", False)),
                    }
                except (KeyError, TypeError, ValueError, OverflowError):
                    rejected += 1
            self.local_bypass_corridors = corridors
            rospy.loginfo(
                "[LaneChange] local bypass corridors loaded: valid=%d rejected=%d path=%s",
                len(corridors), rejected, path)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logerr("[LaneChange] local bypass corridor load failed: %s", exc)
            self.local_bypass_corridors = {}

    def _open_diagnostics(self):
        if not self.diagnostics_csv:
            return
        try:
            directory = os.path.dirname(self.diagnostics_csv)
            if directory:
                os.makedirs(directory, exist_ok=True)
            self.diag_stream = open(self.diagnostics_csv, "w", newline="", encoding="utf-8")
            fields = [
                "stamp_s", "ego_x", "ego_y", "ego_heading_deg", "ego_speed_mps",
                "mode", "global_index", "global_distance_m", "global_heading_error_deg",
                "link", "road", "lane", "vehicle_count", "route_points",
                "route_first_x", "route_first_y", "route_last_x", "route_last_y", "status",
            ]
            self.diag_writer = csv.DictWriter(self.diag_stream, fieldnames=fields)
            self.diag_writer.writeheader()
            self.diag_stream.flush()
            rospy.on_shutdown(self._close_diagnostics)
            rospy.loginfo("[LaneChange] diagnostics CSV (overwrite): %s", self.diagnostics_csv)
        except OSError as exc:
            rospy.logerr("[LaneChange] diagnostics CSV open failed: %s", exc)
            self.diag_stream = None
            self.diag_writer = None

    def _close_diagnostics(self):
        with self.diag_lock:
            stream = self.diag_stream
            self.diag_writer = None
            self.diag_stream = None
            if stream is not None and not stream.closed:
                stream.flush()
                stream.close()

    def _write_diagnostics(self, cur, route, status):
        lock = getattr(self, "diag_lock", None)
        if lock is None:
            return
        with lock:
            if (self.diag_writer is None or self.diag_stream is None or
                    self.ego is None):
                return
            first = route[0] if route else (float("nan"), float("nan"))
            last = route[-1] if route else first
            self.diag_writer.writerow({
                "stamp_s": "%.9f" % rospy.Time.now().to_sec(),
                "ego_x": "%.6f" % self.ego.position.x,
                "ego_y": "%.6f" % self.ego.position.y,
                "ego_heading_deg": "%.6f" % self.ego.heading,
                "ego_speed_mps": "%.6f" % self.ego_v,
                "mode": self.mode,
                "global_index": "" if self.base_progress is None else self.base_progress,
                "global_distance_m": "" if self.base_match_distance is None else "%.6f" % self.base_match_distance,
                "global_heading_error_deg": "" if self.base_match_heading_error is None else "%.6f" % math.degrees(self.base_match_heading_error),
                "link": cur.idx if cur else "", "road": cur.road if cur else "", "lane": cur.lane if cur else "",
                "vehicle_count": len(self.objects), "route_points": len(route),
                "route_first_x": "%.6f" % first[0], "route_first_y": "%.6f" % first[1],
                "route_last_x": "%.6f" % last[0], "route_last_y": "%.6f" % last[1],
                "status": status,
            })
            self.diag_tick += 1
            if self.diag_tick % max(1, int(self.rate_hz)) == 0:
                self.diag_stream.flush()

    @staticmethod
    def _load_base_route(path_file):
        if not path_file:
            return []
        try:
            out = []
            if path_file.lower().endswith(".json"):
                with open(path_file) as f:
                    data = json.load(f)
                candidates = (
                    (float(w["x"]), float(w["y"]))
                    for w in data.get("waypoints", [])
                    if "x" in w and "y" in w
                )
            else:
                def text_candidates():
                    with open(path_file) as stream:
                        for line_number, line in enumerate(stream, 1):
                            stripped = line.split("#", 1)[0].strip()
                            if not stripped:
                                continue
                            fields = stripped.split()
                            if len(fields) < 2:
                                raise ValueError("line %d has fewer than 2 columns" % line_number)
                            yield float(fields[0]), float(fields[1])
                candidates = text_candidates()
            for x, y in candidates:
                if not math.isfinite(x) or not math.isfinite(y):
                    raise ValueError("route contains a non-finite coordinate")
                if out and math.hypot(x - out[-1][0], y - out[-1][1]) < 1.0e-6:
                    continue
                out.append((x, y))
            if len(out) < 2:
                raise ValueError("route needs at least 2 distinct points")
            return out
        except Exception as exc:
            rospy.logwarn("[LaneChange] base route load failed: %s", exc)
            return []

    def _init_feed_timeout_policy(self):
        """Controller feed timeout used by every certificate (revision3). The configured value is validated
        against the controller parameter when it is readable; the larger value wins; invalid -> default."""
        assumed = rospy.get_param("~controller_feed_timeout_s", ENVELOPE_T_FEED)
        try:
            assumed = float(assumed)
        except (TypeError, ValueError):
            assumed = float("nan")
        source = "planner-param"
        if not (math.isfinite(assumed) and assumed > 0.0):
            rospy.logerr("[LaneChange] invalid controller_feed_timeout_s=%r; using %.2f", assumed, ENVELOPE_T_FEED)
            assumed, source = ENVELOPE_T_FEED, "default-after-invalid"
        observed = None
        for key in rospy.get_param("~controller_feed_timeout_params",
                                   ["/path_follower_node/avoid_path_timeout_s"]) or []:
            try:
                value = rospy.get_param(key, None)
                value = None if value is None else float(value)
            except Exception:
                value = None
            if value is not None and math.isfinite(value) and value > 0.0:
                observed = (key, value)
                break
        if observed is not None and observed[1] > assumed:
            rospy.logwarn("[LaneChange] controller %s=%.2f exceeds planner assumption %.2f; using the controller value",
                          observed[0], observed[1], assumed)
            assumed, source = observed[1], "controller-param " + observed[0]
        self.controller_feed_timeout_s = assumed
        self.controller_feed_timeout_source = source if observed is None or source.startswith("controller") else \
            source + " (controller %s=%.2f)" % observed
        self._envelope_policy = envelope_policy(assumed)

    # ── revision3: clock epochs ───────────────────────────────────────────────
    def _clock_state(self):
        lock = self.__dict__.get("_clock_lock")
        if lock is None:
            lock = self.__dict__["_clock_lock"] = threading.Lock()
            self.__dict__.setdefault("_clock_epoch", 0)
            self.__dict__.setdefault("_clock_last", None)
            self.__dict__.setdefault("_clock_discard", {})
            self.__dict__.setdefault("_clock_resets", [])
        return lock

    def _observe_clock(self, now):
        """Record a ROS-time reading; a backward step beyond the tolerance starts a new input epoch and arms a
        one-packet discard per subscribed topic (queue_size 1: the first packet processed after the step may
        have been queued before it). Returns the current epoch."""
        lock = self._clock_state()
        try:
            t = float(now.to_sec())
        except Exception:
            t = float("nan")
        tol = float(getattr(self, "clock_tolerance_s", CLOCK_TOLERANCE_S))
        with lock:
            last = self._clock_last
            if math.isfinite(t):
                if last is not None and t < last - tol:
                    self._clock_epoch += 1
                    self._clock_discard = {"ego": 1, "obj": 1}
                    self._clock_resets.append((last, t))
                    del self._clock_resets[:-20]
                    self._clock_last = t
                elif last is None or t > last:
                    self._clock_last = t
            return self._clock_epoch

    def _clock_discard_first(self, topic):
        lock = self._clock_state()
        with lock:
            n = self._clock_discard.get(topic, 0)
            if n > 0:
                self._clock_discard[topic] = n - 1
                return True
        return False

    def _receipt_age(self, receipt, now=None):
        """Finite age of a receipt against a started clock, or None (missing, unstarted clock, non-finite, or a
        FUTURE receipt beyond the tolerance). Future receipts are never clamped to zero."""
        if receipt is None:
            return None
        try:
            now = rospy.Time.now() if now is None else now
            t_now = float(now.to_sec())
            age = t_now - float(receipt.to_sec())
        except Exception:
            return None
        if not (t_now > 0.0) or not math.isfinite(age):
            return None
        if age < -float(getattr(self, "clock_tolerance_s", CLOCK_TOLERANCE_S)):
            return None
        return max(0.0, age)

    def _ego_cb(self, msg):
        # One immutable packet per message (single reference assignment);
        # the receipt travels with the data it stamps.
        now = rospy.Time.now()
        epoch = self._observe_clock(now)
        if self._clock_discard_first("ego"):
            return
        self._ego_version = getattr(self, "_ego_version", 0) + 1
        self._ego_packet = (self._ego_version, now, msg,
                            math.hypot(msg.velocity.x, msg.velocity.y),
                            msg.velocity.x, epoch)
        point = (float(msg.position.x), float(msg.position.y))
        if not self.ego_history:
            self.ego_history.append(point)
        else:
            distance = math.hypot(
                point[0] - self.ego_history[-1][0],
                point[1] - self.ego_history[-1][1])
            if distance > 5.0:
                # Scenario reset/teleport: never reverse toward the old pose.
                self.ego_history.clear()
                self.ego_history.append(point)
            elif distance >= 0.25:
                self.ego_history.append(point)

    def _vehicle_status_cb(self, msg):
        now = rospy.Time.now()
        self._observe_clock(now)
        self.vehicle_gear = int(getattr(msg, "gear", 4))
        self.vehicle_status_receipt = now

    def _obj_cb(self, msg):
        now = rospy.Time.now()
        epoch = self._observe_clock(now)
        if self._clock_discard_first("obj"):
            return
        objects = []
        invalid = []
        # Keep identity and orientation.  The former implementation reduced
        # every object to an anonymous point and therefore released a safety
        # cap whenever the nearest list entry changed for one frame.
        for class_name, group in (
                ("Car", msg.npc_list),
                ("Human", msg.pedestrian_list),
                (None, msg.obstacle_list)):
            for obj in group:
                resolved_class = class_name
                name = str(getattr(obj, "name", ""))
                lowered_name = name.lower()
                if resolved_class == "Human" and any(
                        token in lowered_name
                        for token in ("dog", "deer", "animal")):
                    resolved_class = "Animal"
                if resolved_class is None:
                    resolved_class = (
                        "Animal" if any(token in lowered_name for token in (
                            "dog", "deer", "animal")) else "Obstacle")
                # revision3: every safety-relevant field must be finite; an invalid entry is recorded and
                # makes the whole frame an invalid required input (never silently clear space)
                bad = []
                for field, getter in (("position.x", lambda o: o.position.x), ("position.y", lambda o: o.position.y),
                                      ("velocity.x", lambda o: o.velocity.x), ("velocity.y", lambda o: o.velocity.y),
                                      ("heading", lambda o: getattr(o, "heading", 0.0)),
                                      ("size.x", lambda o: getattr(o.size, "x", 1.0)),
                                      ("size.y", lambda o: getattr(o.size, "y", 1.0))):
                    try:
                        ok = math.isfinite(float(getter(obj)))
                    except (TypeError, ValueError, AttributeError):
                        ok = False
                    if not ok:
                        bad.append(field)
                try:
                    uid = int(getattr(obj, "unique_id", -1))
                except (TypeError, ValueError, OverflowError):
                    uid = None
                    bad.append("unique_id")
                if bad:
                    invalid.append((class_name or "obstacle_list", -1 if uid is None else uid, tuple(bad)))
                    continue
                objects.append((
                    obj.position.x,
                    obj.position.y,
                    obj.velocity.x,
                    obj.velocity.y,
                    resolved_class,
                    float(getattr(obj.size, "x", 1.0)),
                    float(getattr(obj.size, "y", 1.0)),
                    float(getattr(obj.size, "z", 1.0)),
                    uid,
                    name,
                    math.radians(float(getattr(obj, "heading", 0.0))),
                ))
        self._obj_version = getattr(self, "_obj_version", 0) + 1
        self._obj_packet = (self._obj_version, now, tuple(objects), epoch, tuple(invalid))

    def _take_frame(self):
        """Snapshot the latest ego and object packets for this tick. Every
        predicate of the tick reads only these planner-owned copies; a
        packet arriving during planning is used at the next tick (reaction
        latency <= one planning period + tick duration)."""
        ego_packet = getattr(self, "_ego_packet", None)
        obj_packet = getattr(self, "_obj_packet", None)
        # revision3: only packets of the CURRENT clock epoch are admissible (a pre-reset packet is invalid)
        epoch = self._observe_clock(rospy.Time.now())
        self._frame_epoch = epoch
        if ego_packet is not None and len(ego_packet) > 5 and ego_packet[5] != epoch:
            ego_packet = None
            self.ego = None; self.ego_receipt = None
        if obj_packet is not None and len(obj_packet) > 3 and obj_packet[3] != epoch:
            obj_packet = None
            self.objects = []; self.objects_receipt = None; self.objects_invalid = ()
        self._frame_ego_current = ego_packet is not None
        self._frame_obj_current = obj_packet is not None
        if ego_packet is not None:
            (_version, self.ego_receipt, self.ego, self.ego_v,
             self.ego_signed_v) = ego_packet[:5]
        if obj_packet is not None:
            self.objects = list(obj_packet[2])
            self.objects_receipt = obj_packet[1]
            self.objects_invalid = tuple(obj_packet[4]) if len(obj_packet) > 4 else ()
            self._objects_ever = True
        self._tick_frame = (
            ego_packet[0] if ego_packet is not None else None,
            obj_packet[0] if obj_packet is not None else None)

    def _traffic_cb(self, msg):
        if not self.traffic_detection_armed:
            return
        try:
            payload = json.loads(msg.data)
            now = rospy.Time.now()
            self._observe_clock(now)
            self.traffic_filter.update(payload, now.to_sec())
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(
                2.0, "[TrafficStop] invalid YOLO detection JSON: %s", exc)

    def _set_traffic_detection_armed(self, armed):
        armed = bool(armed)
        if armed == self.traffic_detection_armed:
            return
        self.traffic_detection_armed = armed
        self.traffic_filter.reset()

    @staticmethod
    def _point_projection_on_route(px, py, route):
        """Return (arc length, lateral distance) for a point on a polyline."""
        if len(route) < 2:
            return None
        best = None
        travelled = 0.0
        for start, end in zip(route, route[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            segment_length = math.hypot(dx, dy)
            if segment_length <= 1.0e-9:
                continue
            fraction = max(0.0, min(
                1.0,
                ((px - start[0]) * dx + (py - start[1]) * dy) /
                (segment_length * segment_length),
            ))
            projection = (
                start[0] + fraction * dx,
                start[1] + fraction * dy,
            )
            distance = math.hypot(px - projection[0], py - projection[1])
            along = travelled + fraction * segment_length
            if best is None or distance < best[1]:
                best = along, distance
            travelled += segment_length
        return best

    def _next_stop_line_on_route(self, route, ex, ey, include_behind_m=0.5):
        ego_projection = self._point_projection_on_route(ex, ey, route)
        if ego_projection is None:
            return None
        ego_along = ego_projection[0]
        best = None
        for node in self.stop_line_nodes:
            if self.traffic_stop_signalized_only and not node["signalized"]:
                continue
            point = node["point"]
            projection = self._point_projection_on_route(point[0], point[1], route)
            if projection is None or projection[1] > self.traffic_stop_route_tolerance_m:
                continue
            distance = projection[0] - ego_along
            if distance < -abs(include_behind_m):
                continue
            if distance > self.traffic_stop_lookahead_m:
                continue
            item = (distance, node, projection[1])
            if best is None or distance < best[0]:
                best = item
        return best

    def _stop_line_distance(self, route, ex, ey, node):
        ego_projection = self._point_projection_on_route(ex, ey, route)
        point = node["point"]
        node_projection = self._point_projection_on_route(
            point[0], point[1], route)
        if ego_projection is None or node_projection is None:
            return None
        if node_projection[1] > 2.0 * self.traffic_stop_route_tolerance_m:
            return None
        return node_projection[0] - ego_projection[0]

    def _traffic_speed_limit(self, route, ex, ey, now_s):
        """Return a speed cap and diagnostic status for the current route."""
        if not getattr(self, "traffic_stop_enable", False):
            return None, "TRAFFIC_STOP_DISABLED"
        upcoming = self._next_stop_line_on_route(route, ex, ey)
        self._set_traffic_detection_armed(
            upcoming is not None or self.traffic_target_node_id is not None)
        state = self.traffic_filter.current(now_s)

        proceed = state == "green" or (
            state == "green_left" and self.traffic_green_left_is_go)
        if proceed:
            released = self.traffic_target_node_id
            self.traffic_target_node_id = None
            self.traffic_hold_active = False
            if upcoming is None:
                return None, "TRAFFIC_GO state=%s" % state
            return None, "TRAFFIC_GO state=%s node=%s dist=%.1f" % (
                state, released or upcoming[1]["idx"], upcoming[0])

        restrictive = state == "red" or (
            state == "yellow" and self.traffic_yellow_is_stop)
        if state == "green_left" and not self.traffic_green_left_is_go:
            restrictive = True
        if state == "unknown" and self.traffic_unknown_is_stop:
            restrictive = True

        # Apply the configured permissive states to an already acquired
        # target as well.  Previously a red target stayed latched forever
        # after the detector expired to ``unknown`` (or a permissive yellow),
        # even when traffic_unknown_is_stop/traffic_yellow_is_stop were false.
        # That left /avoid_target_vel at zero after the route no longer
        # contained the stop node.  Confirmed green still uses TRAFFIC_GO
        # above; this branch keeps the other policy switches consistent.
        if self.traffic_target_node_id is not None and not restrictive:
            released = self.traffic_target_node_id
            was_holding = self.traffic_hold_active
            self.traffic_target_node_id = None
            self.traffic_hold_active = False
            return None, "TRAFFIC_RELEASE state=%s node=%s phase=%s" % (
                state, released, "hold" if was_holding else "approach")

        if self.traffic_target_node_id is None:
            if upcoming is None or not restrictive:
                if upcoming is None:
                    return None, "TRAFFIC_CLEAR state=%s no_stop_line" % state
                return None, "TRAFFIC_CLEAR state=%s node=%s dist=%.1f" % (
                    state, upcoming[1]["idx"], upcoming[0])
            self.traffic_target_node_id = upcoming[1]["idx"]
            self.traffic_hold_active = False

        target = self.stop_line_by_id.get(self.traffic_target_node_id)
        if target is None:
            self.traffic_target_node_id = None
            self.traffic_hold_active = False
            return None, "TRAFFIC_TARGET_MISSING state=%s" % state

        distance = self._stop_line_distance(route, ex, ey, target)
        if distance is None and upcoming is not None:
            # A lane change can replace one lane's stop node with the adjacent
            # lane's node at the same intersection.
            old_point = target["point"]
            new_point = upcoming[1]["point"]
            if math.hypot(
                    old_point[0] - new_point[0],
                    old_point[1] - new_point[1]) <= 8.0:
                target = upcoming[1]
                self.traffic_target_node_id = target["idx"]
                distance = upcoming[0]

        if distance is None:
            if self.traffic_hold_active:
                return 0.0, "TRAFFIC_HOLD state=%s node=%s route_lost" % (
                    state, target["idx"])
            self.traffic_target_node_id = None
            return None, "TRAFFIC_TARGET_LOST state=%s node=%s" % (
                state, target["idx"])

        if distance < -self.traffic_pass_release_m:
            self.traffic_target_node_id = None
            self.traffic_hold_active = False
            return None, "TRAFFIC_PASSED state=%s node=%s dist=%.1f" % (
                state, target["idx"], distance)

        stop_space = distance - self.traffic_stop_margin_m
        if stop_space <= self.traffic_stop_hard_distance_m:
            self.traffic_hold_active = True
        if self.traffic_hold_active:
            return 0.0, "TRAFFIC_HOLD state=%s node=%s dist=%.1f" % (
                state, target["idx"], distance)

        speed_cap = math.sqrt(max(
            0.0, 2.0 * self.traffic_stop_decel_mps2 * stop_space))
        return speed_cap, (
            "TRAFFIC_APPROACH state=%s node=%s dist=%.1f cap=%.1fkm/h"
            % (state, target["idx"], distance, speed_cap * 3.6))

    def _publish_traffic_status(self, status):
        publisher = getattr(self, "pub_traffic", None)
        if publisher is not None:
            publisher.publish(String(data=status))

    def _cmd_cb(self, msg):
        self._cmd_version = getattr(self, "_cmd_version", 0) + 1
        self.target_lane = int(msg.data)
        if self.target_lane <= 0:
            # A zero manual command clears only manual intent. It must not
            # cancel an in-flight automatic AVOID/REJOIN safety maneuver.
            if not self.auto_avoid or self.mode == "MANUAL":
                self.active_target_lane = None
                self.change_dst_idx = None
                self.changing = False
                self.mode = "CRUISE"
                self.home_lane = None
            rospy.loginfo("[LaneChange] 차선변경 목표 해제")
        else:
            rospy.loginfo("[LaneChange] 목표 차선: %d차선", self.target_lane)

    @staticmethod
    def _proj(px, py, link):
        # revision3 (tick budget, identical result): same squared distances, first minimum (as min(range, key))
        squared = [(px - qx) * (px - qx) + (py - qy) * (py - qy) for qx, qy in link.pts]
        nearest_index = squared.index(min(squared))
        best = None
        # Only the two segments adjacent to the nearest vertex can own the
        # local projection. This preserves the old O(points) matching cost
        # while fixing the final-vertex zero-heading bug.
        for segment_index in (nearest_index - 1, nearest_index):
            if segment_index < 0 or segment_index + 1 >= len(link.pts):
                continue
            start = link.pts[segment_index]
            end = link.pts[segment_index + 1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1.0e-12:
                continue
            fraction = max(0.0, min(
                1.0,
                ((px - start[0]) * dx + (py - start[1]) * dy) / length_sq,
            ))
            proj_x = start[0] + fraction * dx
            proj_y = start[1] + fraction * dy
            rx = px - proj_x
            ry = py - proj_y
            distance_sq = rx * rx + ry * ry
            if best is None or distance_sq < best[0]:
                heading = math.atan2(dy, dx)
                lateral = -math.sin(heading) * rx + math.cos(heading) * ry
                best = distance_sq, lateral, heading
        if best is None:
            return 0.0, float("inf"), 0.0
        return best[1], math.sqrt(best[0]), best[2]

    def _current_link(self, ex, ey, eh, want_lane=None):
        best = None
        best_score = 1e18
        fallback = None
        fallback_dist = 1e18
        for link in self.links.values():
            if want_lane is not None and link.lane != want_lane:
                continue
            lat, dist, th = self._proj(ex, ey, link)
            if dist > 8.0:
                continue
            hd = abs(norm(th - eh))
            if want_lane is None and hd < FALLBACK_HEADING_DIFF and dist < fallback_dist:
                fallback = link
                fallback_dist = dist
            if hd > MAX_HEADING_DIFF:
                continue
            # Distance must dominate link hysteresis. Using lateral distance
            # alone can retain an old link almost 8 m behind at link ends.
            score = dist + 0.4 * hd
            if want_lane is None and self.last_link == link.idx:
                score -= 0.7
            if score < best_score:
                best_score = score
                best = link
        if best is None:
            best = fallback
        if best is not None and want_lane is None:
            self.last_link = best.idx
        return best

    def _link_at_point(self, px, py, heading, maximum_distance=8.0):
        """Match route geometry at an arbitrary future point without state."""
        best = None
        best_score = float("inf")
        fallback = None
        fallback_distance = float("inf")
        for link in self.links.values():
            _lateral, distance, link_heading = self._proj(px, py, link)
            if distance > maximum_distance:
                continue
            difference = abs(norm(link_heading - heading))
            if difference < FALLBACK_HEADING_DIFF and distance < fallback_distance:
                fallback = link
                fallback_distance = distance
            if difference > MAX_HEADING_DIFF:
                continue
            score = distance + 0.4 * difference
            if score < best_score:
                best_score = score
                best = link
        return best if best is not None else fallback

    def _blocker_route_link(self, route, blocker):
        if blocker is None or len(route) < 2:
            return None
        frame = self._route_frame_at_distance(route, max(0.0, blocker[0]))
        if frame is None:
            return None
        _point, heading = frame
        return self._link_at_point(blocker[3], blocker[4], heading)

    def _lat_to(self, ex, ey, link):
        lat, _, _ = self._proj(ex, ey, link)
        return abs(lat)

    def _target_lane_reached(self, cur, ex, ey, eh):
        if self.active_target_lane is None:
            return None
        if cur is not None and cur.lane == self.active_target_lane and self._lat_to(ex, ey, cur) < self.arrive_lane_lat_m:
            return cur
        detected = self._current_link(ex, ey, eh, want_lane=self.active_target_lane)
        if detected is not None and detected.lane == self.active_target_lane and self._lat_to(ex, ey, detected) < self.arrive_forced_lane_lat_m:
            return detected
        return None

    def _forward(self, start, length):
        chain = [start]
        total = start.length
        used = {start.idx}
        cur = start
        while total < length and len(chain) < self.max_forward_links:
            nxt = list(self.adj.get(cur.to, []))
            if not self.loop_hdmap_route:
                nxt = [link for link in nxt if link.idx not in used]
            if not nxt:
                break
            same_lane = [link for link in nxt if link.lane == cur.lane]
            same_road = [link for link in nxt if link.road == cur.road]
            pool = same_lane or same_road or nxt
            cur_th = math.atan2(cur.pts[-1][1] - cur.pts[-2][1], cur.pts[-1][0] - cur.pts[-2][0])
            best = min(
                pool,
                key=lambda link: abs(norm(
                    math.atan2(link.pts[1][1] - link.pts[0][1], link.pts[1][0] - link.pts[0][0]) - cur_th
                )),
            )
            best_th = math.atan2(best.pts[1][1] - best.pts[0][1], best.pts[1][0] - best.pts[0][0])
            if abs(norm(best_th - cur_th)) > math.radians(55):
                break
            chain.append(best)
            used.add(best.idx)
            total += best.length
            cur = best
        return chain

    def _against_direction(self, start, length):
        """Follow predecessor links while travelling against their native direction."""
        chain = [start]
        total = start.length
        used = {start.idx}
        cur = start
        reverse_adj = getattr(self, "reverse_adj", {})
        while total < length and len(chain) < self.max_forward_links:
            candidates = [
                link for link in reverse_adj.get(cur.fr, [])
                if link.idx not in used]
            if not candidates:
                break
            same_lane = [link for link in candidates if link.lane == cur.lane]
            same_road = [link for link in candidates if link.road == cur.road]
            pool = same_lane or same_road or candidates
            current_heading = math.atan2(
                cur.pts[0][1] - cur.pts[1][1],
                cur.pts[0][0] - cur.pts[1][0])
            best = min(pool, key=lambda link: abs(norm(
                math.atan2(
                    link.pts[-2][1] - link.pts[-1][1],
                    link.pts[-2][0] - link.pts[-1][0]) - current_heading)))
            best_heading = math.atan2(
                best.pts[-2][1] - best.pts[-1][1],
                best.pts[-2][0] - best.pts[-1][0])
            if abs(norm(best_heading - current_heading)) > math.radians(55.0):
                break
            chain.append(best)
            used.add(best.idx)
            total += best.length
            cur = best
        return chain

    @staticmethod
    def _chain_pts_raw(chain):
        pts = []
        for link in chain:
            for point in link.pts:
                if pts and (pts[-1][0] - point[0]) ** 2 + (pts[-1][1] - point[1]) ** 2 < 0.04:
                    continue
                pts.append(point)
        return pts

    @staticmethod
    def _resample(pts, step=0.5):
        if len(pts) < 2:
            return list(pts)
        out = [pts[0]]
        remaining = step
        segment_start = pts[0]
        for segment_end in pts[1:]:
            sx, sy = segment_start
            ex, ey = segment_end
            distance = math.hypot(ex - sx, ey - sy)
            while distance >= remaining and distance > 1.0e-12:
                ratio = remaining / distance
                point = (sx + ratio * (ex - sx), sy + ratio * (ey - sy))
                out.append(point)
                sx, sy = point
                distance = math.hypot(ex - sx, ey - sy)
                remaining = step
            remaining -= distance
            segment_start = segment_end
        if math.hypot(pts[-1][0] - out[-1][0], pts[-1][1] - out[-1][1]) > 1.0e-9:
            out.append(pts[-1])
        return out

    def _route_from_reference(self, ex, ey, ref, blend_m=None, force_anchor=False):
        if len(ref) < 2:
            return ref
        closest = min(range(len(ref)), key=lambda i: (ref[i][0] - ex) ** 2 + (ref[i][1] - ey) ** 2)
        heading_start = closest if closest + 1 < len(ref) else closest - 1
        heading_end = heading_start + 1
        th0 = math.atan2(
            ref[heading_end][1] - ref[heading_start][1],
            ref[heading_end][0] - ref[heading_start][0])
        d0 = -math.sin(th0) * (ex - ref[closest][0]) + math.cos(th0) * (ey - ref[closest][1])
        blend = max(1.0, blend_m if blend_m is not None else self.transition_m)
        route = []
        if force_anchor or self.anchor_current_pose:
            route.append((ex, ey))
        travelled = 0.0
        prev = ref[closest]
        for i in range(closest, len(ref)):
            if i > closest:
                travelled += math.hypot(ref[i][0] - prev[0], ref[i][1] - prev[1])
            prev = ref[i]
            u = min(1.0, travelled / blend)
            # Quintic smoothstep has zero first and second derivatives at both
            # ends.  This removes the steering/curvature step that made an
            # otherwise successful lane change feel segmented.
            w = u * u * u * (u * (u * 6.0 - 15.0) + 10.0)
            off = d0 * (1.0 - w)
            heading_i = i if i + 1 < len(ref) else i - 1
            j = heading_i + 1
            th = math.atan2(ref[j][1] - ref[heading_i][1], ref[j][0] - ref[heading_i][0])
            point = (ref[i][0] - math.sin(th) * off, ref[i][1] + math.cos(th) * off)
            if not route or (route[-1][0] - point[0]) ** 2 + (route[-1][1] - point[1]) ** 2 > 0.04:
                route.append(point)
        return route

    def _route_to_link(self, ex, ey, link, blend_m=None, force_anchor=False):
        ref = self._resample(
            self._chain_pts_raw(self._forward(link, self.route_len)), 0.5)
        return self._route_from_reference(
            ex, ey, ref, blend_m=blend_m, force_anchor=force_anchor)

    def _contraflow_reference(self, link):
        chain = self._against_direction(link, self.route_len)
        points = []
        for item in chain:
            for point in reversed(item.pts):
                if points and math.hypot(
                        points[-1][0] - point[0], points[-1][1] - point[1]) < 0.2:
                    continue
                points.append(point)
        return self._resample(points, 0.5)

    def _route_to_contraflow(self, ex, ey, link, blend_m=None):
        return self._route_from_reference(
            ex, ey, self._contraflow_reference(link),
            blend_m=blend_m, force_anchor=False)

    def _base_route_geometry(self):
        if len(self.base_route) < 2:
            return 0, False
        closed = math.hypot(
            self.base_route[-1][0] - self.base_route[0][0],
            self.base_route[-1][1] - self.base_route[0][1],
        ) < 1.0
        return len(self.base_route) - 1 if closed else len(self.base_route), closed

    def _base_heading(self, index, unique_count, closed):
        current = index % unique_count
        previous = current - 1
        following = current + 1
        if closed:
            previous %= unique_count
            following %= unique_count
        else:
            previous = max(0, previous)
            following = min(unique_count - 1, following)
        return math.atan2(
            self.base_route[following][1] - self.base_route[previous][1],
            self.base_route[following][0] - self.base_route[previous][0],
        )

    def _match_base_route(self, ex, ey, eh):
        self.base_reacquire_pending = None
        unique_count, closed = self._base_route_geometry()
        if unique_count < 2:
            return None

        def score(absolute_index, ex=ex, ey=ey, eh=eh):
            index = absolute_index % unique_count if closed else absolute_index
            following = (index + 1) % unique_count if closed else index + 1
            start = self.base_route[index]
            end = self.base_route[following]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1.0e-12:
                return float("inf"), float("inf"), math.pi, start, 0.0
            fraction = max(0.0, min(
                1.0,
                ((ex - start[0]) * dx + (ey - start[1]) * dy) / length_sq,
            ))
            projection = (
                start[0] + fraction * dx,
                start[1] + fraction * dy,
            )
            distance = math.hypot(projection[0] - ex, projection[1] - ey)
            heading_error = abs(norm(math.atan2(dy, dx) - eh))
            return (distance + 2.0 * heading_error, distance,
                    heading_error, projection, fraction)

        reacquire = self.base_progress is None
        if not reacquire:
            _, tracked_distance, _, _, _ = score(self.base_progress)
            reacquire = tracked_distance > self.base_reacquire_dist_m

        margin = self.base_heading_match_margin_m
        if reacquire:
            # revision3: global reacquisition is charged and chunked across ticks. Every chunk is scored at the
            # FROZEN start pose; candidates within nearest + margin + 2 x REACQUIRE_POSE_TOL are kept, and on
            # completion the kept candidates are rescored exactly at the CURRENT pose/heading. Because the pose
            # moved at most REACQUIRE_POSE_TOL (else restart), every segment of the one-shot selection at the
            # current pose is in the kept set, so the result equals the one-shot matcher at the current pose.
            # State updates are immutable (a new dict per chunk); _reacq_state is a declared rollback-excluded
            # progress cache (see _PROGRESS_CACHE_KEYS), keyed by route identity/length/version.
            segment_count = unique_count if closed else unique_count - 1
            state = getattr(self, "_reacq_state", None)
            key = (id(self.base_route), len(self.base_route), getattr(self, "_base_route_version", 0))
            if (state is None or state["key"] != key or
                    math.hypot(ex - state["pose"][0], ey - state["pose"][1]) > REACQUIRE_POSE_TOL or
                    abs(norm(eh - state["pose"][2])) > math.radians(5.0)):
                state = {"key": key, "pose": (ex, ey, eh), "next": 0, "nearest": float("inf"), "kept": (),
                         "restarts": (state or {}).get("restarts", 0) + (0 if state is None else 1)}
            fx, fy, fh = state["pose"]
            keep_margin = margin + 2.0 * REACQUIRE_POSE_TOL
            start = state["next"]
            stop = min(segment_count, start + REACQUIRE_CHUNK)
            nearest, kept = state["nearest"], list(state["kept"])
            prune_at = 2 * len(kept) + 256
            for i0 in range(start, stop, 64):
                self._spend(8)
                for i in range(i0, min(stop, i0 + 64)):
                    item = score(i, fx, fy, fh)
                    if item[1] <= nearest + keep_margin:
                        nearest = min(nearest, item[1])
                        kept.append((item, i))
                        if len(kept) > prune_at:
                            # lazy (amortized linear) pruning of candidates no longer within the keep margin
                            self._spend(1 + len(kept) // 64)
                            kept = [k for k in kept if k[0][1] <= nearest + keep_margin]
                            prune_at = 2 * len(kept) + 256
            self._reacq_state = dict(state, next=stop, nearest=nearest, kept=tuple(kept))
            if stop < segment_count:
                self.base_reacquire_pending = (stop, segment_count)
                return None
            self._reacq_state = None
            self.base_reacquire_pending = None
            final = [i for item, i in kept if item[1] <= nearest + keep_margin]
            self._spend(1 + len(final) // 8)
            ranked = [(score(i), i) for i in final]          # exact rescoring at the current pose
        else:
            end = self.base_progress + self.base_search_ahead_points + 1
            if not closed:
                end = min(end, unique_count - 1)
            candidates = range(self.base_progress, end)
            self._spend(1 + (end - self.base_progress) // 8)
            ranked = [(score(i), i) for i in candidates]
        if not ranked:
            return None
        nearest_distance = min(item[0][1] for item in ranked)
        # Heading is only a tie-breaker among spatially nearby branches. A
        # global heading filter used to select an aligned point more than
        # 100 m away after an ESKF reset, sending the rolling path off-screen.
        spatial = [
            item for item in ranked
            if item[0][1] <= nearest_distance + self.base_heading_match_margin_m
        ]
        aligned = [
            item for item in spatial
            if item[0][2] <= FALLBACK_HEADING_DIFF
        ]
        selected = min(aligned or spatial, key=lambda item: item[0][0])
        (_, distance, heading_error, projection, fraction), absolute_index = selected
        self.base_progress = absolute_index
        self.base_match_distance = distance
        self.base_match_heading_error = heading_error
        self.base_match_projection = projection
        self.base_match_fraction = fraction
        return absolute_index, unique_count, closed

    def _base_route_slice(self, ex, ey, eh):
        if len(self.base_route) < 2:
            return []
        match = self._match_base_route(ex, ey, eh)
        if match is None:
            return []
        closest, unique_count, closed = match
        projection = self.base_match_projection
        out = [projection] if projection is not None else []
        total = 0.0
        prev = projection
        for offset in range(1, unique_count + 1):
            index = closest + offset
            if self.loop_hdmap_route and closed:
                index %= unique_count
            elif index >= unique_count:
                break
            point = self.base_route[index]
            if prev is not None:
                segment = math.hypot(point[0] - prev[0], point[1] - prev[1])
                if segment <= 1.0e-6:
                    continue
                total += segment
            out.append(point)
            prev = point
            if total >= self.route_len:
                break
        return out

    def _build_base_rejoin_route(self, ex, ey, eh):
        base = self._base_route_slice(ex, ey, eh)
        if len(base) < 2:
            return base
        # Cubic Hermite/Bezier entry: preserve both the current vehicle tangent
        # and the authoritative global-route tangent. The former offset-only
        # blend ignored ``eh`` and could start with a 70~120 degree jump.
        travelled = 0.0
        merge_index = len(base) - 1
        for index in range(1, len(base)):
            travelled += math.hypot(
                base[index][0] - base[index - 1][0],
                base[index][1] - base[index - 1][1])
            if travelled >= max(6.0, self.transition_m):
                merge_index = index
                break
        end = base[merge_index]
        tangent_index = min(merge_index + 1, len(base) - 1)
        if tangent_index == merge_index:
            tangent_index = max(0, merge_index - 1)
            target_heading = math.atan2(
                end[1] - base[tangent_index][1],
                end[0] - base[tangent_index][0])
        else:
            target_heading = math.atan2(
                base[tangent_index][1] - end[1],
                base[tangent_index][0] - end[0])
        chord = max(1.0, math.hypot(end[0] - ex, end[1] - ey))
        handle = min(max(2.0, chord / 3.0), max(3.0, self.transition_m * 0.5))
        control_1 = (ex + handle * math.cos(eh), ey + handle * math.sin(eh))
        control_2 = (
            end[0] - handle * math.cos(target_heading),
            end[1] - handle * math.sin(target_heading),
        )
        sample_count = max(12, int(math.ceil(chord / 0.5)))
        route = []
        for sample in range(sample_count + 1):
            u = sample / float(sample_count)
            one_minus = 1.0 - u
            candidate = (
                one_minus ** 3 * ex +
                3.0 * one_minus ** 2 * u * control_1[0] +
                3.0 * one_minus * u ** 2 * control_2[0] +
                u ** 3 * end[0],
                one_minus ** 3 * ey +
                3.0 * one_minus ** 2 * u * control_1[1] +
                3.0 * one_minus * u ** 2 * control_2[1] +
                u ** 3 * end[1],
            )
            if not route or math.hypot(
                    candidate[0] - route[-1][0], candidate[1] - route[-1][1]) > 0.05:
                route.append(candidate)
        for point in base[merge_index + 1:]:
            if math.hypot(point[0] - route[-1][0], point[1] - route[-1][1]) > 0.05:
                route.append(point)
        return route

    def _base_rejoin_route(self, ex, ey, eh):
        if self.rejoin_route:
            nearest = min(
                range(len(self.rejoin_route)),
                key=lambda index: (
                    (self.rejoin_route[index][0] - ex) ** 2 +
                    (self.rejoin_route[index][1] - ey) ** 2),
            )
            distance = math.hypot(
                self.rejoin_route[nearest][0] - ex,
                self.rejoin_route[nearest][1] - ey)
            if distance > self.rejoin_route_refresh_m:
                self.rejoin_refresh_count = getattr(self, "rejoin_refresh_count", 0) + 1
                self.rejoin_refresh_last = (distance, float(getattr(self, "ego_v", 0.0)))
                self.rejoin_route = self._build_base_rejoin_route(ex, ey, eh)
            elif nearest > 2:
                self.rejoin_route = self.rejoin_route[nearest - 1:]
            return list(self.rejoin_route)
        self.rejoin_route = self._build_base_rejoin_route(ex, ey, eh)
        return list(self.rejoin_route)

    def _global_rejoin_requested(self):
        distance = self.base_match_distance
        if (distance is None or not math.isfinite(distance) or
                distance <= self.global_rejoin_trigger_m):
            self.global_rejoin_trigger_cycles = 0
            return False
        self.global_rejoin_trigger_cycles += 1
        return self.global_rejoin_trigger_cycles >= max(
            1, self.global_rejoin_trigger_cycles_needed)

    def _rejoin_without_link(self, ex, ey, eh):
        if self.mode != "REJOIN" or not self.use_base_route:
            return None
        distance = self.base_match_distance
        heading_error = self.base_match_heading_error
        if (distance is None or heading_error is None or
                not math.isfinite(distance) or not math.isfinite(heading_error) or
                distance > self.global_rejoin_max_dist_m or
                heading_error > self.global_rejoin_no_lane_max_heading):
            return None
        route = self._base_rejoin_route(ex, ey, eh)
        if len(route) < 2:
            return None
        # Keep recovery controllable when the vehicle is strongly misaligned.
        # A non-zero floor lets the controller steer instead of remaining in a
        # permanent zero-speed/no-path state.
        speed_scale = max(0.2, math.cos(heading_error))
        speed = min(self.cruise_mps, self.global_rejoin_max_mps) * speed_scale
        status = (
            "AUTO_REJOIN_GLOBAL_NO_LANE idx=%s dist=%.2f heading=%.1f"
            % (self.base_progress, distance, math.degrees(heading_error)))
        return route, speed, status

    def _link_reference(self, link):
        ref = self._resample(self._chain_pts_raw(self._forward(link, self.route_len)), 0.5)
        if len(ref) < 2:
            return [], [], []
        s = [0.0]
        th = []
        for i in range(1, len(ref)):
            s.append(s[-1] + math.hypot(ref[i][0] - ref[i - 1][0], ref[i][1] - ref[i - 1][1]))
        for i in range(len(ref)):
            a = max(0, i - 1)
            b = min(len(ref) - 1, i + 1)
            th.append(math.atan2(ref[b][1] - ref[a][1], ref[b][0] - ref[a][0]))
        return ref, s, th

    def _object_matches_link_dir(self, th, vx, vy):
        speed = math.hypot(vx, vy)
        if speed < 0.5:
            return True
        obj_h = math.atan2(vy, vx)
        return abs(norm(obj_h - th)) <= math.radians(self.object_heading_diff)

    @staticmethod
    def _object_fields(raw):
        """Return kinematics and optional perception metadata.

        Four-value tuples remain supported by simulations and legacy callers.
        Runtime tuples also carry class and box dimensions.
        """
        class_name = str(raw[4]) if len(raw) > 4 else "Unknown"
        size = (
            float(raw[5]) if len(raw) > 5 else 1.0,
            float(raw[6]) if len(raw) > 6 else 1.0,
            float(raw[7]) if len(raw) > 7 else 1.0,
        )
        return (float(raw[0]), float(raw[1]), float(raw[2]),
                float(raw[3]), class_name, size)

    @staticmethod
    def _object_metadata(raw):
        """Return stable identity metadata while retaining legacy tuples."""
        return {
            "id": int(raw[8]) if len(raw) > 8 else -1,
            "name": str(raw[9]) if len(raw) > 9 else "",
            "heading": float(raw[10]) if len(raw) > 10 else None,
        }

    @staticmethod
    def _is_vulnerable_class(class_name):
        return str(class_name).lower() in ("human", "animal")

    def _blocker_is_vulnerable(self, blocker):
        return bool(
            blocker is not None and len(blocker) > 5 and
            self._is_vulnerable_class(blocker[5]))

    def _object_path_lateral_limit(
            self, raw, size, route_heading, class_name):
        """Return the body-clearance envelope normal to a driven path.

        ``block_lat_m`` is a lane-association band, not a collision envelope.
        Using that broad band as the final blocker test made roadside barrels
        stop the vehicle even when the published path cleared their OBB.  Use
        the measured ego/object footprints here; vulnerable users retain the
        deliberately wider stop-only corridor.
        """
        metadata = self._object_metadata(raw)
        object_yaw = (
            float(route_heading) if metadata["heading"] is None
            else float(metadata["heading"]))
        _along, object_lateral = obb_projected_half_extents(
            max(0.4, float(size[0])), max(0.4, float(size[1])),
            object_yaw, float(route_heading))
        ego_half_width = 0.5 * float(getattr(
            self, "bypass_vehicle_width", 1.892))
        clearance = float(getattr(
            self, "path_block_clearance_m",
            getattr(self, "intersection_bypass_clearance_m", 0.35)))
        geometry_limit = ego_half_width + object_lateral + clearance
        if self._is_vulnerable_class(class_name):
            return max(
                geometry_limit,
                float(getattr(self, "vulnerable_lateral_m", 3.5)))
        return geometry_limit

    def _remember_avoid_source(self, blocker):
        """Keep the triggering obstacle until ego has physically passed it."""
        if blocker is not None and len(blocker) > 4:
            self.avoid_source = (float(blocker[3]), float(blocker[4]))

    def _avoid_source_passed(self, ex, ey, eh):
        source = getattr(self, "avoid_source", None)
        if source is None:
            return True
        forward = (
            math.cos(eh) * (source[0] - ex) +
            math.sin(eh) * (source[1] - ey))
        margin = getattr(self, "avoid_source_pass_margin_m", 6.0)
        return forward <= -margin

    def _object_lead_on_link(self, link, ex, ey, min_s=2.0, max_s=None):
        max_s = self.detect_dist_m if max_s is None else max_s
        ref, s, th = self._link_reference(link)
        if len(ref) < 2:
            return None
        ego_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ex) ** 2 + (ref[i][1] - ey) ** 2)
        best = None
        for raw in self.objects:
            self._spend(1 + len(ref) // 16)
            ox, oy, vx, vy, class_name, size = self._object_fields(raw)
            obj_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ox) ** 2 + (ref[i][1] - oy) ** 2)
            s_rel = s[obj_i] - s[ego_i]
            if s_rel < min_s or s_rel > max_s:
                continue
            lat = -math.sin(th[obj_i]) * (ox - ref[obj_i][0]) + math.cos(th[obj_i]) * (oy - ref[obj_i][1])
            lateral_limit = self._object_path_lateral_limit(
                raw, size, th[obj_i], class_name)
            if abs(lat) > lateral_limit:
                continue
            if (not self._is_vulnerable_class(class_name) and
                    not self._object_matches_link_dir(th[obj_i], vx, vy)):
                continue
            v_along = vx * math.cos(th[obj_i]) + vy * math.sin(th[obj_i])
            metadata = self._object_metadata(raw)
            item = (s_rel, lat, v_along, ox, oy, class_name, size,
                    metadata["id"], metadata["name"], metadata["heading"])
            if best is None or s_rel < best[0]:
                best = item
        return best

    def _object_lead_on_route(self, route, ex, ey, min_s=2.0, max_s=None,
                              lateral_limit=None):
        """Return the lead vehicle on the path that will actually be published."""
        max_s = self.detect_dist_m if max_s is None else max_s
        lateral_limit = (
            self.block_lat_m if lateral_limit is None else float(lateral_limit))
        ref = self._resample(route, 0.5)
        if len(ref) < 2:
            return None
        distance_along = [0.0]
        headings = []
        for index in range(1, len(ref)):
            distance_along.append(distance_along[-1] + math.hypot(
                ref[index][0] - ref[index - 1][0],
                ref[index][1] - ref[index - 1][1]))
        for index in range(len(ref)):
            start = max(0, index - 1)
            end = min(len(ref) - 1, index + 1)
            headings.append(math.atan2(
                ref[end][1] - ref[start][1],
                ref[end][0] - ref[start][0]))
        ego_index = min(
            range(len(ref)),
            key=lambda index: (
                (ref[index][0] - ex) ** 2 + (ref[index][1] - ey) ** 2))
        best = None
        for raw in self.objects:
            self._spend(1 + len(ref) // 16)
            ox, oy, vx, vy, class_name, size = self._object_fields(raw)
            object_index = min(
                range(len(ref)),
                key=lambda index: (
                    (ref[index][0] - ox) ** 2 + (ref[index][1] - oy) ** 2))
            along = distance_along[object_index] - distance_along[ego_index]
            if along < min_s or along > max_s:
                continue
            heading = headings[object_index]
            lateral = (
                -math.sin(heading) * (ox - ref[object_index][0]) +
                math.cos(heading) * (oy - ref[object_index][1]))
            object_lateral_limit = self._object_path_lateral_limit(
                raw, size, heading, class_name)
            if abs(lateral) > object_lateral_limit:
                continue
            if (not self._is_vulnerable_class(class_name) and
                    not self._object_matches_link_dir(heading, vx, vy)):
                continue
            speed = vx * math.cos(heading) + vy * math.sin(heading)
            metadata = self._object_metadata(raw)
            item = (along, lateral, speed, ox, oy, class_name, size,
                    metadata["id"], metadata["name"], metadata["heading"])
            if best is None or along < best[0]:
                best = item
        predicted = self._predicted_vulnerable_conflict(
            ref, distance_along, headings, ego_index, ex, ey)
        if predicted is not None and (best is None or predicted[0] < best[0]):
            best = predicted
        return best

    def _predicted_vulnerable_conflict(
            self, ref, distance_along, headings, ego_index, ex, ey):
        """Find a Human/Animal trajectory that will cross the driven path.

        Raw lateral-band checks only reacted after a crossing animal was
        already inside the lane.  Canonical object feeds carry map-frame
        velocity, allowing a bounded constant-velocity conflict prediction.
        """
        if len(ref) < 2:
            return None
        horizon = getattr(self, "crossing_predict_time_s", 4.0)
        step = getattr(self, "crossing_predict_step_s", 0.2)
        margin = getattr(self, "crossing_time_margin_s", 2.0)
        ego_speed = max(1.0, float(getattr(self, "ego_v", 0.0)))
        best = None
        for raw in self.objects:
            self._spend(1)
            ox, oy, vx, vy, class_name, size = self._object_fields(raw)
            if not self._is_vulnerable_class(class_name):
                continue
            speed = math.hypot(vx, vy)
            if speed < 0.05:
                continue
            metadata = self._object_metadata(raw)
            count = max(1, int(math.ceil(horizon / step)))
            self._spend(count * (1 + len(ref) // 16))
            for sample in range(1, count + 1):
                future_time = sample * step
                px = ox + vx * future_time
                py = oy + vy * future_time
                index = min(
                    range(len(ref)),
                    key=lambda candidate: (
                        (ref[candidate][0] - px) ** 2 +
                        (ref[candidate][1] - py) ** 2))
                along = distance_along[index] - distance_along[ego_index]
                if along < 1.0 or along > self.detect_dist_m:
                    continue
                heading = headings[index]
                lateral = (
                    -math.sin(heading) * (px - ref[index][0]) +
                    math.cos(heading) * (py - ref[index][1]))
                geometry_limit = (
                    1.05 + 0.5 * min(max(size[1], 0.4), 3.0) + 0.35)
                if abs(lateral) > max(
                        geometry_limit,
                        getattr(self, "vulnerable_lateral_m", 3.5)):
                    continue
                ego_arrival = along / ego_speed
                if abs(ego_arrival - future_time) > margin:
                    continue
                v_along = vx * math.cos(heading) + vy * math.sin(heading)
                item = (
                    along, lateral, v_along, ox, oy, class_name, size,
                    metadata["id"], metadata["name"], metadata["heading"],
                    future_time)
                if best is None or item[0] < best[0]:
                    best = item
                break
        return best

    def _target_lane_clear(self, link, ex, ey):
        ok, _reason, _score = self._target_lane_gap_ok(link, ex, ey)
        return ok

    def _target_lane_gap_ok(self, link, ex, ey, moving_only=False):
        """moving_only (gap passage): non-vulnerable static objects are left to the exact body sweep; moving
        traffic and vulnerable road users keep this lane gate."""
        ref, s, th = self._link_reference(link)
        if len(ref) < 2:
            return False, "no-reference", -1e9
        ego_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ex) ** 2 + (ref[i][1] - ey) ** 2)
        front = None
        rear = None
        for raw in self.objects:
            self._spend(1)
            ox, oy, vx, vy, class_name, _size = self._object_fields(raw)
            if (moving_only and math.hypot(vx, vy) <= self.intersection_bypass_static_mps and
                    not self._is_vulnerable_class(class_name)):
                continue
            obj_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ox) ** 2 + (ref[i][1] - oy) ** 2)
            s_rel = s[obj_i] - s[ego_i]
            lat = -math.sin(th[obj_i]) * (ox - ref[obj_i][0]) + math.cos(th[obj_i]) * (oy - ref[obj_i][1])
            lateral_limit = (
                max(self.block_lat_m + 0.4,
                    getattr(self, "vulnerable_lateral_m", 3.5))
                if self._is_vulnerable_class(class_name)
                else self.block_lat_m + 0.4)
            if abs(lat) >= lateral_limit:
                continue
            if (not self._is_vulnerable_class(class_name) and
                    not self._object_matches_link_dir(th[obj_i], vx, vy)):
                continue
            v_along = vx * math.cos(th[obj_i]) + vy * math.sin(th[obj_i])
            item = (s_rel, v_along)
            if s_rel >= 0.0:
                if front is None or s_rel < front[0]:
                    front = item
            else:
                if rear is None or s_rel > rear[0]:
                    rear = item

        ego_v = max(0.0, self.ego_v)
        front_req = self.gap_front_min_m
        rear_req = self.gap_rear_min_m
        front_gap = self.target_front_gap_m
        rear_gap = self.target_rear_gap_m
        if front is not None:
            front_gap = front[0]
            closing = max(0.0, ego_v - front[1])
            front_req = max(front_req, ego_v * self.gap_front_time_s + closing * self.gap_rel_speed_time_s)
        if rear is not None:
            rear_gap = -rear[0]
            closing = max(0.0, rear[1] - ego_v)
            rear_req = max(rear_req, rear[1] * self.gap_rear_time_s + closing * self.gap_rel_speed_time_s)

        front_pred = front_gap
        rear_pred = rear_gap
        if front is not None:
            front_pred = front[0] + (front[1] - ego_v) * self.gap_predict_time_s
        if rear is not None:
            rear_pred = -rear[0] + (ego_v - rear[1]) * self.gap_predict_time_s

        epsilon = getattr(self, "gap_compare_epsilon_m", 0.10)
        if front_gap + epsilon < front_req:
            return False, "front-gap %.1f<%.1f" % (front_gap, front_req), front_gap - front_req
        if rear_gap + epsilon < rear_req:
            return False, "rear-gap %.1f<%.1f" % (rear_gap, rear_req), rear_gap - rear_req
        if front_pred + epsilon < self.gap_front_min_m:
            return False, "front-pred %.1f<%.1f" % (front_pred, self.gap_front_min_m), front_pred - self.gap_front_min_m
        if rear_pred + epsilon < self.gap_rear_min_m:
            return False, "rear-pred %.1f<%.1f" % (rear_pred, self.gap_rear_min_m), rear_pred - self.gap_rear_min_m
        score = min(
            front_gap - front_req,
            rear_gap - rear_req,
            front_pred - self.gap_front_min_m,
            rear_pred - self.gap_rear_min_m,
        )
        return True, (
            "gap front=%.1f/%.1f pred=%.1f rear=%.1f/%.1f pred=%.1f"
            % (front_gap, front_req, front_pred, rear_gap, rear_req, rear_pred)
        ), score

    def _link_front_speed_cap(self, link, ex, ey, safety_gap=None):
        safety_gap = self.front_safety_gap if safety_gap is None else safety_gap
        ref, s, th = self._link_reference(link)
        if len(ref) < 2:
            return None
        ego_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ex) ** 2 + (ref[i][1] - ey) ** 2)
        cap = None
        for raw in self.objects:
            self._spend(1)
            ox, oy, vx, vy, class_name, _size = self._object_fields(raw)
            obj_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ox) ** 2 + (ref[i][1] - oy) ** 2)
            s_rel = s[obj_i] - s[ego_i]
            if s_rel <= 0.0 or s_rel > self.detect_dist_m:
                continue
            lat = -math.sin(th[obj_i]) * (ox - ref[obj_i][0]) + math.cos(th[obj_i]) * (oy - ref[obj_i][1])
            lateral_limit = (
                max(self.front_safety_lat_m,
                    getattr(self, "vulnerable_lateral_m", 3.5))
                if self._is_vulnerable_class(class_name)
                else self.front_safety_lat_m)
            if abs(lat) > lateral_limit:
                continue
            if (not self._is_vulnerable_class(class_name) and
                    not self._object_matches_link_dir(th[obj_i], vx, vy)):
                continue
            obj_v = vx * math.cos(th[obj_i]) + vy * math.sin(th[obj_i])
            object_gap = (
                max(safety_gap, getattr(self, "vulnerable_safety_gap", 12.0))
                if self._is_vulnerable_class(class_name) else safety_gap)
            decel = (
                getattr(self, "vulnerable_brake_decel", 3.0)
                if self._is_vulnerable_class(class_name) else self.brake_decel)
            stop_space = max(0.0, s_rel - object_gap)
            safe_v = math.sqrt(max(0.0, 2.0 * decel * stop_space)) + max(0.0, obj_v)
            cap = safe_v if cap is None else min(cap, safe_v)
        return cap

    def _front_object_speed_cap(self, ex, ey, eh):
        cap = None
        ce = math.cos(eh)
        se = math.sin(eh)
        for raw in self.objects:
            self._spend(1)
            ox, oy, vx, vy, class_name, _size = self._object_fields(raw)
            dx = ox - ex
            dy = oy - ey
            s_rel = ce * dx + se * dy
            lat = -se * dx + ce * dy
            if s_rel <= 0.0 or s_rel > self.detect_dist_m:
                continue
            lateral_limit = (
                max(self.front_safety_lat_m,
                    getattr(self, "vulnerable_lateral_m", 3.5))
                if self._is_vulnerable_class(class_name)
                else self.front_safety_lat_m)
            if abs(lat) > lateral_limit:
                continue
            obj_v = vx * ce + vy * se
            gap = (getattr(self, "vulnerable_safety_gap", 12.0)
                   if self._is_vulnerable_class(class_name)
                   else self.front_safety_gap)
            decel = (getattr(self, "vulnerable_brake_decel", 3.0)
                     if self._is_vulnerable_class(class_name)
                     else self.brake_decel)
            stop_space = max(0.0, s_rel - gap)
            safe_v = math.sqrt(max(0.0, 2.0 * decel * stop_space)) + max(0.0, obj_v)
            cap = safe_v if cap is None else min(cap, safe_v)
        return cap

    def _has_rejoin_path(self, cur, cand, min_return_s):
        if not self.require_rejoin_path:
            return True, "rejoin-check-off"
        home_chain = self._forward(cur, self.rejoin_check_m + min_return_s)
        home_ids = {link.idx for link in home_chain}
        cand_chain = self._forward(cand, self.rejoin_check_m + min_return_s)
        travelled = 0.0
        for link in cand_chain:
            link_end_s = travelled + link.length
            if link_end_s >= min_return_s:
                for side, adj in self._adjacent_candidates(link):
                    if adj.idx in home_ids:
                        return True, "rejoin-%s:%s->%s" % (side, link.idx, adj.idx)
            travelled = link_end_s
        return False, "no-return-path"

    def _choose_avoid_link(self, cur, ex, ey, blocker_s=None):
        candidates = self._prioritize_avoid_candidates(
            self._adjacent_candidates(cur))
        if not candidates:
            return None, "no-adjacent-lane"
        clear = []
        gap_reasons = []
        for side, link in candidates:
            ok, gap_reason, gap_score = self._target_lane_gap_ok(link, ex, ey)
            if ok:
                clear.append((side, link, gap_reason, gap_score))
            else:
                gap_reasons.append("%s:%s" % (side, gap_reason))
        if not clear:
            return None, "target-blocked " + ",".join(gap_reasons)
        min_return_s = max(self.transition_m, (blocker_s or 0.0) + self.front_safety_gap)
        rejoinable = []
        blocked_reasons = []
        for side, link, gap_reason, gap_score in clear:
            ok, reason = self._has_rejoin_path(cur, link, min_return_s)
            if ok:
                rejoinable.append((side, link, reason, gap_reason, gap_score))
            else:
                blocked_reasons.append("%s:%s" % (side, reason))
        if not rejoinable:
            return None, "no-return-path"
        # Prefer the adjacent lane with the largest usable gap; this lets traffic merges happen promptly.
        if len(rejoinable) == 1:
            side, link, reason, gap_reason, _gap_score = rejoinable[0]
            return link, "%s/%s/%s" % (side, reason, gap_reason)
        side, link, reason, gap_reason, _gap_score = max(
            rejoinable,
            key=lambda item: (item[4], -abs(item[1].lane - cur.lane)),
        )
        return link, "%s/%s/%s" % (side, reason, gap_reason)

    def _choose_structural_avoid_link(self, cur, blocker_s=None):
        """Return a legal/rejoinable lane even while its live gap is closed.

        This is only a planning target.  OBB preflight remains authoritative
        and no mode/state is committed until a complete path is feasible.
        """
        candidates = list(self._adjacent_candidates(cur))
        seen = {link.idx for _side, link in candidates}
        # Intersection connector records can carry a lateral destination while
        # the raw can_move flag is false. Admit only the independently derived
        # same-direction pair, with tight geometry gates; the exact OBB and
        # steering preflight below still decides whether motion is allowed.
        for edge in getattr(
                self, "directional_inferred_same", {}).get(cur.idx, []):
            target_id = edge.get("target_link_id")
            target = self.links.get(target_id)
            if target is None or target_id in seen:
                continue
            if str(target.road) != str(cur.road):
                continue
            if float(edge.get("separation_m", float("inf"))) > 4.5:
                continue
            if abs(float(edge.get(
                    "heading_difference_deg", 180.0))) > 15.0:
                continue
            if float(edge.get("estimated_overlap_m", 0.0)) < 12.0:
                continue
            policy = getattr(self, "link_policies", {}).get(target_id)
            if policy in ("forbidden", "last_resort"):
                continue
            candidates.append((edge.get("side", "unknown"), target))
            seen.add(target_id)
        candidates = self._prioritize_avoid_candidates(candidates)
        minimum_return = max(
            self.transition_m,
            (blocker_s or 0.0) + self.front_safety_gap)
        for side, link in candidates:
            rejoinable, reason = self._has_rejoin_path(
                cur, link, minimum_return)
            if rejoinable:
                return link, "%s/%s/gap-pending" % (side, reason)
        return None, "no-structural-candidate"

    def _route_is_straight(self, route):
        """Return true when the forward reference has little heading change."""
        if (not getattr(self, "straight_lane_change_enable", True) or
                len(route) < 3):
            return False
        travelled = 0.0
        headings = []
        limit = getattr(self, "straight_lane_change_lookahead_m", 45.0)
        for first, second in zip(route, route[1:]):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            segment = math.hypot(dx, dy)
            if segment <= 1.0e-6:
                continue
            headings.append(math.atan2(dy, dx))
            travelled += segment
            if travelled >= limit:
                break
        if len(headings) < 2 or travelled < min(15.0, limit):
            return False
        reference = headings[0]
        maximum_change = max(abs(norm(value - reference)) for value in headings)
        return maximum_change <= getattr(
            self, "straight_lane_change_max_heading", math.radians(5.0))

    def _avoid_blend_m(self, blocker_s=None, reference_route=None):
        if blocker_s is None:
            blend = self.transition_m
        else:
            urgent = max(
                self.urgent_transition_min_m,
                blocker_s - self.urgent_transition_margin_m)
            blend = min(self.transition_m, urgent)
        if (reference_route is not None and
                self._route_is_straight(reference_route)):
            blend = min(
                blend, getattr(
                    self, "straight_lane_change_blend_m", 20.0))
        # When a late low-box detection has already stopped ego, the normal
        # 14 m transition still intersects the box for its first 5-7 m and
        # keeps the collision speed cap at zero forever.  Use a tighter
        # recovery transition only from standstill/creep; once selected,
        # change_blend_m retains this value while the manoeuvre accelerates.
        if (blocker_s is not None and
                blocker_s <= getattr(self, "stopped_avoid_trigger_m", 10.0) and
                abs(getattr(self, "ego_signed_v", 0.0)) <= 0.5):
            blend = min(
                blend, getattr(self, "stopped_avoid_blend_m", 10.0))
        return blend

    def _blocker_speed_cap(self, blocker):
        """Return the braking/following upper bound for one route blocker."""
        if blocker is None:
            return None
        vulnerable = self._blocker_is_vulnerable(blocker)
        safety_gap = (
            getattr(self, "vulnerable_safety_gap", 12.0)
            if vulnerable else self.front_safety_gap)
        decel = (
            getattr(self, "vulnerable_brake_decel", 3.0)
            if vulnerable else self.brake_decel)
        follow_gain = (
            getattr(self, "vulnerable_follow_gain", 0.40)
            if vulnerable else getattr(self, "obstacle_follow_gain", 0.35))
        stop_space = max(0.0, blocker[0] - safety_gap)
        object_speed = max(0.0, blocker[2])
        collision_cap = (
            math.sqrt(max(0.0, 2.0 * decel * stop_space)) +
            object_speed)
        follow_cap = max(
            0.0,
            object_speed + follow_gain * max(
                0.0, blocker[0] - safety_gap),
        )
        return min(collision_cap, follow_cap)

    def _limit_speed_cap_release(self, requested_mps):
        """Apply an asymmetric limiter: braking is immediate, release is not.

        Planner blockers can legitimately change as objects cross the route.
        A new, farther blocker must not turn that change into a full-throttle
        10->50 km/h step while the previous conflict is only one frame old.
        """
        requested = max(0.0, float(requested_mps))
        previous = getattr(self, "last_speed_command_mps", None)
        if previous is None or self.mode == "BACKUP":
            limited = requested
        elif requested <= previous:
            limited = requested
        else:
            rate = max(0.1, getattr(self, "speed_cap_rise_mps2", 1.0))
            dt = 1.0 / max(1.0, float(getattr(self, "rate_hz", 10.0)))
            limited = min(requested, previous + rate * dt)
        self.last_speed_command_mps = limited
        return limited

    def _safe_lane_change_route(self, blocker, target, ex, ey, blend_m):
        """Build a lane-change route and cap speed against that actual route.

        The normal cruise collision cap is measured on the source lane.  Once
        a clear adjacent lane has been selected, keeping that source-lane cap
        can stop the vehicle before it generates lateral motion.  Re-evaluate
        every object against the transition route before replacing the cap.
        """
        route = self._route_to_link(ex, ey, target, blend_m)
        if blocker is not None and blocker[0] <= self.avoid_hard_stop_m:
            self.auto_speed_cap = 0.0
            return route

        route_blocker = self._object_lead_on_route(
            route,
            ex,
            ey,
            lateral_limit=getattr(
                self, "lane_change_collision_lat_m", 2.4),
        )
        if route_blocker is not None:
            self.auto_speed_cap = self._blocker_speed_cap(route_blocker)
            return route

        # The transition corridor and target-lane gap are clear.  Replace the
        # stale source-lane cap with a manoeuvre cap that is fast enough to
        # complete the lateral move without allowing an abrupt high-speed cut.
        gap_ok, _gap_reason, gap_score = self._target_lane_gap_ok(
            target, ex, ey)
        target_reference = self._chain_pts_raw(self._forward(
            target,
            getattr(self, "straight_lane_change_lookahead_m", 45.0)))
        fast_straight = (
            gap_ok and
            gap_score >= getattr(self, "straight_fast_min_gap_margin_m", 2.0) and
            self._route_is_straight(target_reference))
        change_time = (
            getattr(self, "straight_lane_change_time_s", 1.3)
            if fast_straight else self.avoid_change_time_s)
        manoeuvre_cap = max(
            self.avoid_change_min_mps,
            float(blend_m) / change_time,
        )
        self.auto_speed_cap = min(
            self.cruise_mps, target.vmax, manoeuvre_cap)
        target_cap = self._link_front_speed_cap(target, ex, ey)
        if target_cap is not None:
            self.auto_speed_cap = min(self.auto_speed_cap, target_cap)
        return route

    def _delayed_lane_change_route(
            self, cur, target, ex, ey, delay_m, transition_m,
            initial_nudge=0.0):
        """Follow the source lane first, then shift through a free window."""
        source = self._route_to_link(ex, ey, cur)
        target_reference = self._resample(
            self._chain_pts_raw(self._forward(target, self.route_len)), 0.5)
        if len(source) < 3 or len(target_reference) < 3:
            return [], None
        route = []
        travelled = 0.0
        previous = source[0]
        target_index = 0
        goal = None
        for index, point in enumerate(source):
            if index:
                travelled += math.hypot(
                    point[0] - previous[0], point[1] - previous[1])
            previous = point
            search_end = min(len(target_reference), target_index + 100)
            target_index = min(
                range(target_index, search_end),
                key=lambda candidate: (
                    (target_reference[candidate][0] - point[0]) ** 2 +
                    (target_reference[candidate][1] - point[1]) ** 2))
            target_point = target_reference[target_index]
            if travelled <= delay_m:
                weight = 0.0
            else:
                weight = self._smoothstep(
                    (travelled - delay_m) / max(transition_m, 1.0))
            before = max(0, index - 1)
            after = min(len(source) - 1, index + 1)
            heading = math.atan2(
                source[after][1] - source[before][1],
                source[after][0] - source[before][0])
            nudge_weight = self._smoothstep(
                travelled / max(3.0, delay_m))
            nudge = initial_nudge * nudge_weight * (1.0 - weight)
            source_point = (
                point[0] - math.sin(heading) * nudge,
                point[1] + math.cos(heading) * nudge)
            route.append((
                source_point[0] + weight * (
                    target_point[0] - source_point[0]),
                source_point[1] + weight * (
                    target_point[1] - source_point[1])))
            if goal is None and travelled >= delay_m + transition_m + 5.0:
                goal = route[-1]
        return route, goal or route[-1]

    def _ego_body_extents(self):
        """(rear, front, width, model) of the body about the reference point.

        Every dimension is validated on its own (finite; length, width,
        wheelbase > 0; overhangs >= 0). rear_axle with all valid and
        |rear + wheelbase + front - length| <= 0.05: exact extents.
        body_center with valid length/width: half length each way. Otherwise
        (unknown reference, invalid or inconsistent fields) each extent is the
        maximum of every candidate derivable from the valid fields, never
        below half the valid length; valid length and width are always kept.
        Without a valid length or width the body is unbounded: model
        'invalid-config' and every footprint check fails closed."""
        def value(name, default):
            try:
                return float(getattr(self, name, default))
            except (TypeError, ValueError):
                return float("nan")
        length = value("bypass_vehicle_length", 4.635)
        width = value("bypass_vehicle_width", 1.892)
        wheelbase = value("bypass_wheelbase", 3.0)
        front_overhang = value("bypass_front_overhang", 0.845)
        rear_overhang = value("bypass_rear_overhang", 0.790)
        reference = str(getattr(self, "ego_reference_point", "rear_axle"))
        key = (length, width, wheelbase, front_overhang, rear_overhang,
               reference)
        cached = getattr(self, "_ego_extents_cache", None)
        if cached is not None and repr(cached[0]) == repr(key):
            return cached[1]
        ok = lambda v, positive: math.isfinite(v) and (v > 0.0 if positive else v >= 0.0)
        length_ok, width_ok = ok(length, True), ok(width, True)
        wheelbase_ok = ok(wheelbase, True)
        front_ok, rear_ok = ok(front_overhang, False), ok(rear_overhang, False)
        if not (length_ok and width_ok):
            extents = (float("inf"), float("inf"), float("inf"), "invalid-config")
            rospy.logerr(
                "[LaneChange] ego footprint: length=%r width=%r invalid; "
                "all footprint checks fail closed", length, width)
        elif reference == "body_center":
            extents = (0.5 * length, 0.5 * length, width, "body_center")
        elif (reference == "rear_axle" and wheelbase_ok and front_ok and
              rear_ok and abs(rear_overhang + wheelbase + front_overhang -
                              length) <= 0.05):
            extents = (rear_overhang, wheelbase + front_overhang, width,
                       "rear_axle")
        else:
            rears, fronts = [0.5 * length], [0.5 * length]
            if rear_ok:
                rears.append(rear_overhang)
                fronts.append(length - rear_overhang)
            if wheelbase_ok and front_ok:
                fronts.append(wheelbase + front_overhang)
                rears.append(length - wheelbase - front_overhang)
            extents = (max(rears), max(fronts), width, "conservative-union")
            rospy.logerr(
                "[LaneChange] ego footprint: reference=%s length=%r width=%r "
                "wheelbase=%r overhangs f=%r r=%r not a consistent rear-axle "
                "set; conservative union rear=%.3f front=%.3f width=%.3f",
                reference, length, width, wheelbase, front_overhang,
                rear_overhang, extents[0], extents[1], extents[2])
        self._ego_extents_cache = (key, extents)
        return extents

    def _ego_body_reach(self):
        """Largest distance from the reference point to any body point."""
        rear, front, width, _model = self._ego_body_extents()
        return math.hypot(max(rear, front), 0.5 * width)

    def _execution_envelope(self, speed_mps):
        """ArrivalEnvelope of this tick: measured speed and pose of the tick
        frame, v_top = max(requested cap, |v0|, previous emitted command),
        ego/object ages from the frame receipts. None when no finite ego."""
        ego = getattr(self, "ego", None)
        v0 = float(getattr(self, "ego_v", 0.0) or 0.0) if ego is not None else None
        last = getattr(self, "last_speed_command_mps", None)
        last = 0.0 if last is None or not math.isfinite(float(last)) else float(last)
        requested = float(speed_mps) if math.isfinite(float(speed_mps)) else float("nan")
        now = None
        try:
            now = rospy.Time.now()
        except Exception:
            now = None

        def age(stamp):
            if stamp is None or now is None:
                return 0.0
            # revision3: a future receipt (backward clock step) or an unstarted clock invalidates the
            # envelope; never clamped to age 0
            value = self._receipt_age(stamp, now)
            return float("inf") if value is None else value
        ego_age = age(getattr(self, "ego_receipt", None)) if ego is not None else 0.0
        object_age = age(getattr(self, "objects_receipt", None))
        if not math.isfinite(ego_age) or not math.isfinite(object_age):
            return None
        t_feed = float(getattr(self, "controller_feed_timeout_s", ENVELOPE_T_FEED))
        if v0 is None:
            # no measured ego state (offline/unit use): initial speed unknown
            # in [0, v_top]
            v_top = max(requested, last)
            return ArrivalEnvelope(
                v_top, v_top, ego_age=ego_age, object_age=object_age, v0_lo=0.0,
                a_max=getattr(self, "envelope_a_max_mps2", ENVELOPE_A_MAX),
                b_hi=getattr(self, "envelope_b_hi_mps2", ENVELOPE_B_HI),
                b_lo=getattr(self, "envelope_b_lo_mps2", ENVELOPE_B_LO),
                t_react=getattr(self, "envelope_reaction_s", ENVELOPE_T_REACT),
                t_tick=1.0 / max(1.0, float(getattr(self, "rate_hz", 10.0))),
                t_pred=getattr(self, "envelope_prediction_s", ENVELOPE_T_PRED), t_feed=t_feed)
        return ArrivalEnvelope(
            v0, max(requested, v0, last), ego_age=ego_age, object_age=object_age,
            a_max=getattr(self, "envelope_a_max_mps2", ENVELOPE_A_MAX),
            b_hi=getattr(self, "envelope_b_hi_mps2", ENVELOPE_B_HI),
            b_lo=getattr(self, "envelope_b_lo_mps2", ENVELOPE_B_LO),
            t_react=getattr(self, "envelope_reaction_s", ENVELOPE_T_REACT),
            t_tick=1.0 / max(1.0, float(getattr(self, "rate_hz", 10.0))),
            t_pred=getattr(self, "envelope_prediction_s", ENVELOPE_T_PRED), t_feed=t_feed)

    def _execution_ego_pose(self, path):
        ego = getattr(self, "ego", None)
        if ego is not None:
            try:
                x, y = float(ego.position.x), float(ego.position.y)
                h = math.radians(float(ego.heading))
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(h):
                    return x, y, h
            except (AttributeError, TypeError, ValueError):
                pass
            return None
        # No ego packet (offline unit use): the path start with its heading.
        a, b = path[0], path[1]
        return a[0], a[1], math.atan2(b[1] - a[1], b[0] - a[0])

    def _ego_swept_clearance(self, path, obstacles, speed_mps,
                             safety_margin=0.0, sample_step=0.2,
                             screening=False):
        """Certified swept check with the configured body; per-call and
        per-tick work budgets (exhaustion or invalid footprint config ->
        invalid, i.e. fail closed).

        Execution contract: unless screening=True this is the reachable
        arrival-time envelope check of this tick (any speed <= v_top, any
        stop/restart within the declared kinematic bounds). screening=True
        keeps the constant-speed model and is used ONLY to rank/filter
        candidates whose final published route is certified separately."""
        rear, front, width, model = self._ego_body_extents()
        failed = {"valid": False, "minimum_clearance": -float("inf"),
                  "closest": {"path_index": None, "time_s": None,
                              "object_id": None, "kind": model,
                              "clearance": -float("inf")},
                  "evaluations": 0, "budget_exhausted": False}
        if model == "invalid-config":
            return failed
        budget_token = getattr(self, "_tick_budget", None)
        remaining = getattr(self, "_tick_work_remaining", None)
        budget = SWEEP_CALL_BUDGET if remaining is None else max(
            0, min(SWEEP_CALL_BUDGET, remaining))
        if budget <= 0 and obstacles:
            if budget_token is not None:
                # The per-tick OBB budget is part of the tick budget: the
                # whole tick stops explicitly instead of trying other options.
                budget_token.exhausted = "obb-tick-budget"
                raise TickBudgetExceeded("obb-tick-budget")
            failed["closest"]["kind"] = "work-budget"
            failed["budget_exhausted"] = True
            return failed
        if screening:
            result = swept_ego_body_clearance(
                path, obstacles, rear, front, width, speed_mps,
                safety_margin=safety_margin, sample_step=sample_step,
                work_budget=budget, tick_budget=budget_token)
        else:
            envelope = self._execution_envelope(speed_mps)
            ego_pose = self._execution_ego_pose(path)
            if envelope is None or ego_pose is None:
                failed["closest"]["kind"] = "no-envelope"
                return failed
            result = envelope_ego_body_clearance(
                path, obstacles, rear, front, width, envelope, ego_pose,
                safety_margin=safety_margin, sample_step=sample_step,
                work_budget=budget, tick_budget=budget_token)
            self._last_envelope_result = {
                k: result.get(k) for k in ("valid", "minimum_clearance", "closest",
                                           "envelope", "s0", "d0",
                                           "excluded_following")}
        if remaining is not None:
            self._tick_work_remaining = remaining - result["evaluations"]
        self._tick_work_used = getattr(self, "_tick_work_used", 0) + result["evaluations"]
        return result

    def _envelope_required_length(self, speed_mps):
        """Checked-path length the envelope needs ahead of the reference."""
        envelope = self._execution_envelope(speed_mps)
        return None if envelope is None else envelope.required_length

    def _lane_change_preflight(
            self, route, cur, ex, ey, requested_speed_mps,
            horizon_m=60.0, require_direction_guard=True):
        """Validate the exact route before committing avoidance state."""
        result = {
            "feasible": False,
            "reason": "short-route",
            "route": route,
            "speed_cap": 0.0,
            "clearance": -float("inf"),
            "object_id": None,
        }
        if len(route) < 3:
            return result
        # Execution contract: the checked prefix must contain the reachable
        # stopping envelope (fastest distance at the stop-guarantee time).
        need = self._envelope_required_length(max(1.0, float(requested_speed_mps)))
        pose = self._execution_ego_pose(route)
        lead_in = 0.0 if pose is None else math.hypot(
            pose[0] - route[0][0], pose[1] - route[0][1])
        manoeuvre = self._route_prefix_by_length(
            route, max(12.0, min(70.0, horizon_m),
                       (need or 0.0) + lead_in + 2.0))
        if len(manoeuvre) < 3:
            return result
        if (require_direction_guard and
                not self._route_stays_same_direction(cur, manoeuvre)):
            result["reason"] = "direction-guard"
            return result
        speed = max(1.0, float(requested_speed_mps))
        # Kinematics first: they may lower the speed, and the prediction time
        # of moving objects depends on it. The swept check below runs at the
        # final (returned) speed.
        kinematics = validate_path_kinematics(
            manoeuvre, getattr(self, "bypass_wheelbase", 3.0),
            getattr(self, "bypass_max_steer_deg", 35.0),
            getattr(self, "bypass_max_steer_rate_deg_s", 50.0),
            speed, getattr(self, "bypass_max_lateral_accel", 2.26))
        if not kinematics["valid"]:
            hard_reasons = set(kinematics["reasons"]) & {
                "curvature", "steer"}
            if hard_reasons:
                result["reason"] = "kinematics:%s" % ",".join(
                    sorted(hard_reasons))
                return result
            safe_speed = min(speed, kinematics["speed_cap_mps"])
            steer_rate = kinematics["max_steer_rate_deg_s"]
            if steer_rate > 1.0e-6:
                safe_speed = min(
                    safe_speed,
                    speed * getattr(
                        self, "bypass_max_steer_rate_deg_s", 50.0) /
                    steer_rate)
            speed = max(0.8, 0.95 * safe_speed)
            kinematics = validate_path_kinematics(
                manoeuvre, getattr(self, "bypass_wheelbase", 3.0),
                getattr(self, "bypass_max_steer_deg", 35.0),
                getattr(self, "bypass_max_steer_rate_deg_s", 50.0),
                speed, getattr(self, "bypass_max_lateral_accel", 2.26))
            if not kinematics["valid"]:
                result["reason"] = "kinematics:%s" % ",".join(
                    kinematics["reasons"])
                return result
        obstacles = self._prefilter_obb_obstacles(
            manoeuvre, ex, ey, speed,
            getattr(self, "lane_change_preflight_margin", 0.50))
        clearance = self._ego_swept_clearance(
            manoeuvre, obstacles, speed, safety_margin=getattr(
                self, "lane_change_preflight_margin", 0.50),
            sample_step=0.35)
        result["clearance"] = clearance["minimum_clearance"]
        if clearance["closest"] is not None:
            result["object_id"] = clearance["closest"]["object_id"]
        if not clearance["valid"]:
            result["reason"] = (
                "work-budget-exhausted" if clearance.get("budget_exhausted")
                else "obb-blocked id=%s clearance=%.2f" % (
                    result["object_id"], result["clearance"]))
            return result

        result.update({
            "feasible": True,
            "reason": "clearance=%.2f curvature=%.3f" % (
                result["clearance"], kinematics["max_curvature"]),
            "speed_cap": speed,
            "kinematics": kinematics,
        })
        return result

    def _prefilter_obb_obstacles(self, route, ex, ey, speed_mps,
                                 extra_margin=0.0):
        """Conservative gate before the swept check.

        Uses the narrowphase's own prediction horizon (route length /
        max(0.1, |speed|)). An object is dropped only if the segment its
        centre sweeps over that whole horizon stays farther from the route
        polyline than body reach + object half diagonal + margin: the swept
        body then cannot come within the margin of it at any time. No motion
        clipping; unbounded footprint config keeps every object."""
        if len(route) < 2:
            return []
        route_length = sum(math.hypot(
            second[0] - first[0], second[1] - first[1])
            for first, second in zip(route, route[1:]))
        horizon_s = route_length / max(0.1, abs(float(speed_mps)))
        envelope = self._execution_envelope(speed_mps)
        if envelope is not None:
            # objects move from their own measurement time to the hold horizon
            # of an envelope covering this whole route (see ArrivalEnvelope)
            covering = envelope.covering(route_length)
            horizon_s = max(horizon_s, covering.tau_hold - covering.tau_obj0)
        budget_token = getattr(self, "_tick_budget", None)
        reach = self._ego_body_reach()
        margin = reach + max(0.0, float(extra_margin)) + 0.5
        segments = [(a, b) for a, b in zip(route, route[1:])
                    if math.hypot(b[0] - a[0], b[1] - a[1]) > 1.0e-9]
        if not segments:
            segments = [(route[0], route[-1])]
        min_x = min(point[0] for point in route)
        max_x = max(point[0] for point in route)
        min_y = min(point[1] for point in route)
        max_y = max(point[1] for point in route)
        output = []
        for raw in self.objects:
            if budget_token is not None:
                budget_token.spend(1)
            ox, oy, vx, vy, class_name, size = self._object_fields(raw)
            object_radius = 0.5 * math.hypot(max(0.4, size[0]), max(0.4, size[1]))
            qx, qy = ox + vx * horizon_s, oy + vy * horizon_s
            limit = margin + object_radius
            if math.isfinite(limit):
                if (max(ox, qx) + limit < min_x or min(ox, qx) - limit > max_x or
                        max(oy, qy) + limit < min_y or min(oy, qy) - limit > max_y):
                    continue
                if budget_token is not None:
                    budget_token.spend(len(segments))
                if min(_segment_distance((ox, oy), (qx, qy), a, b)
                       for a, b in segments) > limit:
                    continue
            metadata = self._object_metadata(raw)
            if metadata["heading"] is None:
                try:
                    yaw = project_to_frenet(ox, oy, route).heading
                except ValueError:
                    yaw = 0.0
            else:
                yaw = metadata["heading"]
            output.append(ObstacleOBB(
                ox, oy, yaw,
                max(0.4, size[0]), max(0.4, size[1]),
                vx, vy, class_name, metadata["id"]))
        return output

    def _choose_delayed_lane_change(
            self, cur, target, blocker, ex, ey, reference_route):
        """Search delayed transition windows before deciding to stop/reverse."""
        requested_speed = min(self.cruise_mps, cur.vmax, target.vmax)
        if str(getattr(cur, "link_type", "")) == "1":
            requested_speed = min(
                requested_speed, self.intersection_bypass_max_mps)
        maximum_delay = min(
            getattr(self, "delayed_lane_change_max_delay", 16.0),
            max(0.0, float(blocker[0]) - 3.0))
        delay_step = getattr(
            self, "delayed_lane_change_delay_step", 1.0)
        minimum_delay = 0.0
        target_side = 0.0
        if len(reference_route) >= 2:
            target_route = self._resample(
                self._chain_pts_raw(self._forward(
                    target, min(self.route_len, 50.0))), 0.5)
            if target_route:
                target_near = min(
                    target_route,
                    key=lambda point: (
                        (point[0] - ex) ** 2 + (point[1] - ey) ** 2))
                target_side = project_to_frenet(
                    target_near[0], target_near[1],
                    reference_route).d
                for raw in self.objects:
                    self._spend(1)
                    ox, oy, _vx, _vy, _class_name, size = (
                        self._object_fields(raw))
                    if math.hypot(
                            ox - blocker[3], oy - blocker[4]) < 1.0:
                        continue
                    projection = project_to_frenet(
                        ox, oy, reference_route)
                    if (projection.s <= 0.0 or
                            projection.s >= blocker[0] or
                            projection.d * target_side <= 0.0 or
                            abs(projection.d) < 1.0):
                        continue
                    body_buffer = (
                        0.5 * getattr(
                            self, "bypass_vehicle_length", 4.635) +
                        0.5 * max(0.4, size[0]) +
                        getattr(self, "lane_change_preflight_margin", 0.50))
                    minimum_delay = max(
                        minimum_delay, projection.s - body_buffer)
        if minimum_delay <= 1.0e-6:
            # With no target-side object to pass first, postponing the lane
            # change only moves the transition closer to the source blocker.
            delays = [0.0]
        else:
            delay = min(
                maximum_delay,
                math.ceil(minimum_delay / delay_step) * delay_step)
            delays = [delay]
            delay += delay_step
            while delay <= maximum_delay + 1.0e-6:
                delays.append(delay)
                delay += delay_step
        transitions = set((self._avoid_blend_m(
            blocker[0], reference_route),))
        transition = getattr(
            self, "delayed_lane_change_min_transition", 8.0)
        maximum_transition = getattr(
            self, "delayed_lane_change_max_transition", 20.0)
        while transition <= maximum_transition + 1.0e-6:
            transitions.add(transition)
            transition += 2.0
        ordered_transitions = []
        sorted_transitions = sorted(transitions)
        for value in (
                sorted_transitions[-1], sorted_transitions[0],
                sorted_transitions[len(sorted_transitions) // 2],
                sorted_transitions[len(sorted_transitions) // 4],
                sorted_transitions[(3 * len(sorted_transitions)) // 4]):
            if value not in ordered_transitions:
                ordered_transitions.append(value)
        nudges = [0.0]
        if minimum_delay > 1.0e-6 and abs(target_side) > 0.5:
            away = -1.0 if target_side > 0.0 else 1.0
            nudges.extend((0.25 * away, 0.50 * away))
        if len(nudges) > 1:
            primary = ordered_transitions[0]
            specs = [(delays[0], primary, nudge) for nudge in nudges]
            specs.extend((delays[0], candidate_transition, nudges[1])
                         for candidate_transition in ordered_transitions[1:])
        else:
            specs = [(delays[0], candidate_transition, 0.0)
                     for candidate_transition in ordered_transitions]
        if len(delays) > 1:
            specs.append((delays[1], ordered_transitions[0], nudges[-1]))
        specs = specs[:getattr(
            self, "max_delayed_transition_candidates", 5)]
        failure_reasons = []
        for delay, transition, nudge in specs:
            self._spend(200)   # candidate route build/check
            if delay <= 1.0e-6:
                route = self._route_to_link(ex, ey, target, transition)
            else:
                route, _goal = self._delayed_lane_change_route(
                    cur, target, ex, ey, delay, transition,
                    initial_nudge=nudge)
            route = self._resample(route, 1.0)
            goal_frame = self._route_frame_at_distance(
                route, delay + transition + 5.0)
            goal = goal_frame[0] if goal_frame else route[-1]
            horizon = max(
                delay + transition + 8.0,
                min(self.detect_dist_m, blocker[0] + 10.0))
            preflight = self._lane_change_preflight(
                route, cur, ex, ey, requested_speed, horizon,
                require_direction_guard=False)
            if not preflight["feasible"]:
                failure_reasons.append(preflight["reason"])
                continue
            # revision3: skip road-exit transitions so the next bounded spec can be selected
            legal_ok, legal_reason = self._route_road_legal(cur, route, goal)
            if not legal_ok:
                failure_reasons.append("road-boundary:%s" % legal_reason)
                continue
            return route, goal, preflight["speed_cap"], (
                "delay=%.1f transition=%.1f nudge=%.2f %s" % (
                    delay, transition, nudge, preflight["reason"]))
        reason = failure_reasons[0] if failure_reasons else "no-candidate"
        return None, None, None, "delayed-blocked:%s" % reason

    @staticmethod
    def _smoothstep(value):
        value = min(1.0, max(0.0, float(value)))
        return value * value * value * (
            value * (value * 6.0 - 15.0) + 10.0)

    def _intersection_bypass_candidate(self, base_route, blocker, side):
        """Offset the global route around one stationary intersection object."""
        reference = self._resample(base_route, 0.5)
        if len(reference) < 2:
            return [], None
        obstacle_s = float(blocker[0])
        entry_end = max(4.0, obstacle_s - 4.5)
        hold_end = obstacle_s + 4.5
        exit_end = obstacle_s + 15.0
        direction = 1.0 if side == "left" else -1.0
        try:
            route = build_clamped_bspline_offset_path(
                reference,
                direction * self.intersection_bypass_offset_m,
                entry_end, hold_end, exit_end, step=0.5)
        except ValueError:
            return [], None
        goal_frame = self._route_frame_at_distance(route, exit_end)
        goal = goal_frame[0] if goal_frame is not None else route[-1]
        return route, goal

    def _intersection_bypass_clearance(self, route, ex, ey):
        if len(route) < 2:
            return -float("inf")
        candidate_points = self._resample(route, 0.5)
        minimum = float("inf")
        ego_half_width = 1.05
        first_heading = math.atan2(
            candidate_points[1][1] - candidate_points[0][1],
            candidate_points[1][0] - candidate_points[0][0])
        for raw in self.objects:
            self._spend(1)
            ox, oy, _vx, _vy, _class_name, size = self._object_fields(raw)
            if math.hypot(ox - ex, oy - ey) > self.detect_dist_m + 5.0:
                continue
            forward = (
                math.cos(first_heading) * (ox - ex) +
                math.sin(first_heading) * (oy - ey))
            if forward < -3.0:
                continue
            distance = min(math.hypot(ox - px, oy - py)
                           for px, py in candidate_points)
            required = (
                ego_half_width + 0.5 * min(max(size[1], 0.4), 3.0) +
                self.intersection_bypass_clearance_m)
            minimum = min(minimum, distance - required)
        # The lateral tube above is centred on reference points only; it
        # misses the nose beyond the last point and corner swing on curves.
        # Also require the corrected static body sweep on the same points.
        if len(candidate_points) >= 2:
            static = [
                ObstacleOBB(obstacle.x, obstacle.y, obstacle.yaw,
                            obstacle.length, obstacle.width, 0.0, 0.0,
                            obstacle.kind, obstacle.object_id)
                for obstacle in self._prefilter_obb_obstacles(
                    candidate_points, ex, ey, 1.0,
                    self.intersection_bypass_clearance_m)]
            if static:
                body = self._ego_swept_clearance(
                    candidate_points, static, 1.0,
                    safety_margin=self.intersection_bypass_clearance_m,
                    sample_step=0.2, screening=True)   # static objects only
                minimum = min(minimum, body["minimum_clearance"])
        return minimum

    @staticmethod
    def _route_prefix_by_length(route, maximum_length):
        if len(route) < 2:
            return list(route)
        output = [route[0]]
        travelled = 0.0
        for first, second in zip(route, route[1:]):
            segment = math.hypot(
                second[0] - first[0], second[1] - first[1])
            if segment <= 1.0e-9:
                continue
            if travelled + segment >= maximum_length:
                fraction = max(0.0, min(
                    1.0, (maximum_length - travelled) / segment))
                output.append((
                    first[0] + fraction * (second[0] - first[0]),
                    first[1] + fraction * (second[1] - first[1])))
                break
            output.append(second)
            travelled += segment
        return output

    def _surveyed_intersection_bypass(
            self, cur, base_route, blocker, corridor):
        """Build and validate a bypass contained in one surveyed connector."""
        try:
            reference = self._resample(base_route, 0.25)
            if len(reference) < 3 or len(cur.pts) < 2:
                return None, None, "survey-reference-short"
            link_first = project_to_frenet(
                cur.pts[0][0], cur.pts[0][1], reference)
            link_last = project_to_frenet(
                cur.pts[-1][0], cur.pts[-1][1], reference)
            if max(link_first.distance, link_last.distance) > 4.0:
                return None, None, "survey-link-off-route"
            link_start_s = min(link_first.s, link_last.s)
            link_end_s = max(link_first.s, link_last.s)
            link_start_s += corridor["entry_margin_m"]
            link_end_s -= corridor["exit_margin_m"]
            object_projection = project_to_frenet(
                blocker[3], blocker[4], reference)
            if not (link_start_s < object_projection.s < link_end_s):
                return None, None, "survey-object-outside-link"

            arc = [0.0]
            for first, second in zip(reference, reference[1:]):
                arc.append(arc[-1] + math.hypot(
                    second[0] - first[0], second[1] - first[1]))
            start_index = min(
                range(len(arc)), key=lambda index: abs(arc[index] - link_start_s))
            start_s = arc[start_index]
            local_reference = reference[start_index:]
            if len(local_reference) < 3:
                return None, None, "survey-reference-after-link-short"
            object_local_s = object_projection.s - start_s
            link_available = link_end_s - start_s
            metadata_heading = blocker[9] if len(blocker) > 9 else None
            object_yaw = (
                object_projection.heading
                if metadata_heading is None else float(metadata_heading))
            object_length = max(0.4, float(blocker[6][0]))
            object_width = max(0.4, float(blocker[6][1]))
            object_along, object_lateral = obb_projected_half_extents(
                object_length, object_width, object_yaw,
                object_projection.heading)
            ego_half_width = 0.5 * self.bypass_vehicle_width
            # Reference point (rear axle by default) must reach the full
            # offset before the NOSE reaches the object and keep it until the
            # REAR has passed it.
            rear_extent, front_extent, _w, _m = self._ego_body_extents()
            entry_end = object_local_s - (
                front_extent + object_along +
                corridor["minimum_body_clearance_m"])
            hold_end = object_local_s + (
                rear_extent + object_along +
                corridor["minimum_body_clearance_m"])
            if entry_end <= 0.0 or hold_end >= link_available:
                return None, None, "survey-insufficient-longitudinal-room"

            targets = sample_lateral_targets(
                object_projection.d, object_lateral, ego_half_width,
                corridor["minimum_body_clearance_m"],
                self.local_bypass_tracking_margin,
                -corridor["maximum_center_offset_m"],
                corridor["maximum_center_offset_m"],
                step=self.local_bypass_nudge_step_m)
            allowed_sides = set(corridor["allowed_sides"])
            targets = [target for target in targets
                       if ((target > 0.0 and "left" in allowed_sides) or
                           (target < 0.0 and "right" in allowed_sides))]
            if not targets:
                return None, None, "survey-no-lateral-room"

            speed_mps = min(
                self.intersection_bypass_max_mps,
                corridor["maximum_speed_mps"])
            candidates = []
            for target in targets:
                self._spend(200)   # candidate route build/check
                minimum_transition = max(
                    8.0, minimum_quintic_transition_length(
                        target, corridor["maximum_curvature_inv_m"]))
                if entry_end + 1.0e-6 < minimum_transition:
                    continue
                # Use up to 10 m of the available approach.  The 8 m
                # geometric minimum alone exceeded the follower's 50 deg/s
                # steering-rate limit for the recorded connector.
                transition = min(entry_end, max(10.0, minimum_transition))
                exit_end = hold_end + transition
                if exit_end > link_available + 1.0e-6:
                    continue
                local_candidate = build_clamped_bspline_offset_path(
                    local_reference, target, entry_end, hold_end,
                    exit_end, step=0.25)
                manoeuvre = self._route_prefix_by_length(
                    local_candidate, exit_end + 3.0)
                # Enforce the surveyed centre-offset envelope at every sample.
                if any(abs(project_to_frenet(
                        point[0], point[1], reference).d) >
                        corridor["maximum_center_offset_m"] + 0.05
                       for point in manoeuvre):
                    continue

                obstacles = []
                arrival_s = max(0.0, start_s)
                arrival_time = arrival_s / max(speed_mps, 1.0)
                reach = self._ego_body_reach()
                for raw in self.objects:
                    self._spend(1)
                    ox, oy, vx, vy, class_name, size = self._object_fields(raw)
                    projection = project_to_frenet(ox, oy, reference)
                    # Body reach + object half diagonal: the nose extends
                    # front_extent beyond the last manoeuvre point.
                    pad = 3.0 + reach + 0.5 * math.hypot(
                        max(0.4, size[0]), max(0.4, size[1]))
                    if (projection.s < start_s - pad or
                            projection.s > start_s + exit_end + pad):
                        continue
                    metadata = self._object_metadata(raw)
                    yaw = (projection.heading if metadata["heading"] is None
                           else metadata["heading"])
                    obstacles.append(ObstacleOBB(
                        ox + vx * arrival_time,
                        oy + vy * arrival_time,
                        yaw, max(0.4, size[0]), max(0.4, size[1]),
                        vx, vy, class_name, metadata["id"]))
                # Candidate screening only (future-offset manoeuvre, objects
                # pre-shifted): the published full route is certified by
                # _finalize_local_bypass with the execution envelope.
                clearance = self._ego_swept_clearance(
                    manoeuvre, obstacles, speed_mps,
                    safety_margin=corridor["minimum_body_clearance_m"],
                    sample_step=0.2, screening=True)
                if not clearance["valid"]:
                    continue
                kinematics = validate_path_kinematics(
                    manoeuvre, self.bypass_wheelbase,
                    self.bypass_max_steer_deg,
                    self.bypass_max_steer_rate_deg_s,
                    speed_mps, self.bypass_max_lateral_accel,
                    max_operating_curvature=
                    corridor["maximum_curvature_inv_m"])
                if not kinematics["valid"]:
                    continue
                full_route = reference[:start_index] + local_candidate
                # The published form (0.5 m resampling) must also satisfy the
                # surveyed centre-offset envelope over the manoeuvre.
                finalized = self._resample(full_route, 0.5)
                envelope_ok = True
                for point in finalized:
                    frame = project_to_frenet(point[0], point[1], reference)
                    if (start_s <= frame.s <= start_s + exit_end + 3.0 and
                            abs(frame.d) > corridor["maximum_center_offset_m"] + 0.05):
                        envelope_ok = False
                        break
                if not envelope_ok:
                    continue
                # The global prefix can pass unrelated opposing links before
                # this connector.  Apply the directional guard only to the
                # bounded manoeuvre that this candidate modifies.
                if not self._route_stays_same_direction(cur, manoeuvre):
                    continue
                candidates.append((
                    abs(target), -clearance["minimum_clearance"],
                    target, full_route,
                    self._route_prefix_by_length(
                        local_candidate, exit_end)[-1],
                    clearance, kinematics, speed_mps))
            if not candidates:
                return None, None, "survey-all-candidates-invalid"
            (_excursion, _clearance_score, target, route, goal,
             clearance, kinematics, speed_mps) = min(
                 candidates, key=lambda item: item[:3])
            self.intersection_candidate_speed_mps = speed_mps
            side = "left" if target > 0.0 else "right"
            return route, goal, (
                "corridor=%s side=%s offset=%.2f clearance=%.2f "
                "curvature=%.3f steer=%.1f"
                % (corridor["id"], side, target,
                   clearance["minimum_clearance"],
                   kinematics["max_curvature"],
                   kinematics["max_steer_deg"]))
        except (TypeError, ValueError, IndexError, KeyError, AttributeError) as exc:
            return None, None, "survey-error:%s" % exc

    def _choose_intersection_bypass(self, cur, base_route, blocker, ex, ey):
        self.intersection_candidate_speed_mps = None
        if not getattr(self, "intersection_bypass_enable", False):
            return None, None, "disabled"
        # NGII link type 1 is an intersection connector. Normal road links are
        # deliberately excluded because an unannotated lateral offset there
        # could leave the carriageway.
        if str(getattr(cur, "link_type", "")) != "1":
            return None, None, "not-intersection"
        if self._blocker_is_vulnerable(blocker):
            return None, None, "vulnerable-stop-only"
        if (blocker[0] < self.intersection_bypass_min_object_m or
                abs(blocker[2]) > self.intersection_bypass_static_mps):
            return None, None, "late-or-moving"
        # An intersection connector has no physical road-edge polygon in the
        # bundled map.  Never invent a left/right offset there.  Only a side
        # backed by a legal same-direction lateral edge is treated as a white
        # lane corridor; yellow/opposite geometry is excluded.
        allowed_sides = {
            side for side, _link in self._avoidance_lateral_candidates(cur)
            if side in ("left", "right")
        }
        corridor = getattr(self, "local_bypass_corridors", {}).get(cur.idx)
        if not allowed_sides and corridor is not None:
            return self._surveyed_intersection_bypass(
                cur, base_route, blocker, corridor)
        if not allowed_sides:
            return None, None, "no-white-lane-corridor"
        candidates = []
        for side in sorted(allowed_sides):
            route, goal = self._intersection_bypass_candidate(
                base_route, blocker, side)
            if not self._route_stays_same_direction(cur, route):
                continue
            clearance = self._intersection_bypass_clearance(
                route, ex, ey)
            if clearance >= 0.0:
                candidates.append((clearance, side, route, goal))
        if not candidates:
            return None, None, "white-lane-sides-blocked"
        clearance, side, route, goal = max(candidates, key=lambda item: item[0])
        return route, goal, "%s clearance=%.2f" % (side, clearance)

    @staticmethod
    def _route_frame_at_distance(route, distance):
        if len(route) < 2:
            return None
        travelled = 0.0
        for first, second in zip(route, route[1:]):
            segment = math.hypot(
                second[0] - first[0], second[1] - first[1])
            if segment <= 1.0e-9:
                continue
            if travelled + segment >= distance:
                fraction = min(1.0, max(
                    0.0, (distance - travelled) / segment))
                point = (
                    first[0] + fraction * (second[0] - first[0]),
                    first[1] + fraction * (second[1] - first[1]),
                )
                return point, math.atan2(
                    second[1] - first[1], second[0] - first[0])
            travelled += segment
        return route[-1], math.atan2(
            route[-1][1] - route[-2][1],
            route[-1][0] - route[-2][0])

    def _free_bypass_candidate(
            self, base_route, blocker, target_offset, ex, ey,
            rapid_entry=False):
        """Create a non-HD-centerline path and merge back after the object."""
        reference = self._resample(base_route, 0.5)
        if len(reference) < 2:
            return [], None
        if (rapid_entry and math.hypot(
                reference[0][0] - ex,
                reference[0][1] - ey) > 0.75):
            # A rolling rejoin path can begin a few metres ahead of ego.  A
            # close bypass measured arc length from that future first point,
            # so its lateral shift completed after the object.  Anchor only
            # the near-stop rapid candidate to use all available road ahead.
            reference = self._resample([(ex, ey)] + reference, 0.5)
        first_heading = math.atan2(
            reference[1][1] - reference[0][1],
            reference[1][0] - reference[0][0])
        start_offset = (
            -math.sin(first_heading) * (ex - reference[0][0]) +
            math.cos(first_heading) * (ey - reference[0][1]))
        obstacle_s = max(
            float(blocker[0]),
            math.cos(first_heading) * (blocker[3] - ex) +
            math.sin(first_heading) * (blocker[4] - ey))
        if rapid_entry:
            # Used only from a near-stop. Complete the lateral motion before
            # the close box instead of the normal 8 m minimum, which placed
            # the path through a blocker detected at 4--7 m.
            entry_end = max(1.5, min(6.0, obstacle_s - 1.0))
            hold_end = obstacle_s + 3.0
            exit_end = obstacle_s + 12.0
        else:
            entry_end = max(8.0, obstacle_s - 6.0)
            hold_end = obstacle_s + 5.0
            exit_end = obstacle_s + 20.0
        try:
            output = build_clamped_bspline_offset_path(
                reference, target_offset, entry_end, hold_end, exit_end,
                start_offset=start_offset, step=0.5)
        except ValueError:
            return [], None
        goal_frame = self._route_frame_at_distance(output, exit_end)
        goal = goal_frame[0] if goal_frame is not None else output[-1]
        return output, goal

    def _marked_edge_bypass_sides(self, cur):
        """Return a single marked outer edge, never an opposing-lane side."""
        if not getattr(self, "marked_edge_bypass_enable", False):
            return []
        marked = []
        if getattr(cur, "lane_mark_left", ()):
            marked.append("left")
        if getattr(cur, "lane_mark_right", ()):
            marked.append("right")
        opposite_sides = {
            edge.get("side")
            for edge in getattr(self, "directional_opposite", {}).get(
                cur.idx, [])
            if edge.get("side") in ("left", "right")
        }
        safe = [side for side in marked if side not in opposite_sides]
        if opposite_sides:
            return safe
        # With no opposite-link geometry, only one-sided road-edge marking is
        # unambiguous. Two marked sides could include an unknown centreline.
        return safe if len(marked) == 1 else []

    def _choose_marked_edge_bypass(
            self, cur, base_route, blocker, ex, ey):
        """Slowly pass a small static box across one confirmed outer marking."""
        if self._blocker_is_vulnerable(blocker):
            return None, None, None, "vulnerable-stop-only"
        size = blocker[6] if len(blocker) > 6 else (1.0, 1.0, 1.0)
        if (size[1] > getattr(
                self, "marked_edge_bypass_max_object_width", 1.4) or
                size[0] > getattr(
                    self, "marked_edge_bypass_max_object_length", 2.0)):
            return None, None, None, "object-too-large"
        if (blocker[0] < getattr(
                self, "marked_edge_bypass_min_object", 6.5) or
                abs(blocker[2]) > self.intersection_bypass_static_mps):
            return None, None, None, "too-close-or-moving"
        allowed_sides = self._marked_edge_bypass_sides(cur)
        if not allowed_sides:
            return None, None, None, "no-unambiguous-marked-edge"

        ego_half_width = 1.05
        required_separation = (
            ego_half_width + 0.5 * min(max(size[1], 0.4), 3.0) +
            self.intersection_bypass_clearance_m + getattr(
                self, "marked_edge_bypass_tracking_margin", 0.5))
        candidates = []
        for side in allowed_sides:
            direction = 1.0 if side == "left" else -1.0
            target_offset = blocker[1] + direction * required_separation
            if abs(target_offset) > getattr(
                    self, "marked_edge_bypass_max_offset", 2.4):
                continue
            route, goal = self._free_bypass_candidate(
                base_route, blocker, target_offset, ex, ey,
                rapid_entry=True)
            if not self._route_stays_same_direction(cur, route):
                continue
            clearance = self._intersection_bypass_clearance(route, ex, ey)
            if clearance >= 0.0:
                candidates.append((abs(target_offset), -clearance, side,
                                   target_offset, clearance, route, goal))
        if not candidates:
            return None, None, None, "marked-edge-blocked"
        _rank, _clearance_rank, side, offset, clearance, route, goal = min(
            candidates)
        return route, goal, getattr(
            self, "marked_edge_bypass_max_mps", 15.0 / 3.6), (
                "%s offset=%.2f clearance=%.2f"
                % (side, offset, clearance))

    def _same_direction_lateral_links(self, cur):
        """Collect lanes reachable through explicit, crossable boundaries."""
        found = {cur.idx: cur}
        frontier = [cur]
        for _hop in range(max(1, min(self.max_avoid_hops, 4))):
            following = []
            for source in frontier:
                for _side, candidate in self._adjacent_candidates(source):
                    if candidate.idx in found:
                        continue
                    found[candidate.idx] = candidate
                    following.append(candidate)
            if not following:
                break
            frontier = following
        return list(found.values())

    def _avoidance_lateral_candidates(self, cur):
        """Return only HD-map lanes with an explicit legal lane-change edge.

        Geometric parallel-lane inference cannot distinguish a dashed white
        line from a solid white line in the bundled link-only map.  Treating
        inferred neighbours as crossable could therefore send a bypass over
        a solid line.  The HD-map can_move/destination edge is the conservative
        crossing permission used here.
        """
        return list(self._adjacent_candidates(cur))

    def _route_within_crossable_corridor(self, cur, route):
        """Reject local offsets that leave lanes connected by legal edges."""
        if cur is None or len(route) < 2:
            return False

        corridor_links = {}
        for seed in self._same_direction_lateral_links(cur):
            for link in self._forward(seed, self.route_len):
                corridor_links[link.idx] = link
        if not corridor_links:
            return False

        sample_step = max(1, int(round(2.0 / 0.5)))
        sampled = list(route[::sample_step])
        if sampled[-1] != route[-1]:
            sampled.append(route[-1])
        for index, (px, py) in enumerate(sampled):
            before = max(0, index - 1)
            after = min(len(sampled) - 1, index + 1)
            if before == after:
                return False
            route_heading = math.atan2(
                sampled[after][1] - sampled[before][1],
                sampled[after][0] - sampled[before][0])
            projections = []
            for link in corridor_links.values():
                lateral, distance, heading = self._proj(px, py, link)
                if (distance <= 8.0 and
                        abs(norm(heading - route_heading)) <= math.radians(45.0)):
                    projections.append((lateral, distance, link))
            if not projections:
                return False

            nearest_lateral, nearest_distance, nearest_link = min(
                projections, key=lambda item: item[1])
            half_width = 0.5 * min(
                nearest_link.width_start, nearest_link.width_end)
            # Keep the whole 2.1 m-wide ego body inside an uncrossable edge.
            inner_limit = max(0.35, half_width - 1.05 - 0.15)
            if nearest_distance <= inner_limit:
                continue

            # Between two different legally connected lane centres, crossing
            # is permitted. Outside the outermost centre, it is a solid/road
            # edge and the candidate must be rejected.
            left_lanes = {
                link.lane for lateral, _distance, link in projections
                if lateral > 0.0
            }
            right_lanes = {
                link.lane for lateral, _distance, link in projections
                if lateral < 0.0
            }
            if not any(left != right
                       for left in left_lanes for right in right_lanes):
                outside_side = "left" if nearest_lateral > 0.0 else "right"
                side_marks = (
                    nearest_link.lane_mark_left
                    if outside_side == "left" else
                    nearest_link.lane_mark_right)
                opposite_sides = {
                    edge.get("side")
                    for edge in getattr(
                        self, "directional_opposite", {}).get(
                            nearest_link.idx, [])
                    if edge.get("side") in ("left", "right")
                }
                # Intersection connectors often omit a boundary on the open
                # road-space side. Permit a measured-gap nudge only on that
                # unmarked side. A marked side or inferred opposing side stays
                # protected, so this cannot turn into a solid-line crossing.
                open_intersection_side = (
                    str(nearest_link.link_type) == "1" and
                    not side_marks and
                    outside_side not in opposite_sides and
                    nearest_distance <= getattr(
                        self, "intersection_bypass_offset_m", 3.2))
                if not open_intersection_side:
                    return False
        return True

    def _route_stays_same_direction(self, cur, route):
        """Reject a candidate whose centre approaches opposite-lane geometry."""
        if cur is None or len(route) < 2:
            return False
        directional_opposite = getattr(self, "directional_opposite", {})
        if not directional_opposite:
            return True

        same_links = {}
        seeds = self._same_direction_lateral_links(cur)
        for seed in seeds:
            for link in self._forward(seed, self.route_len):
                same_links[link.idx] = link

        opposite_links = {}
        for source in same_links.values():
            for edge in directional_opposite.get(source.idx, []):
                target_id = edge.get("target_link_id")
                target = self.links.get(target_id)
                if target is not None:
                    opposite_links[target.idx] = target
        if not opposite_links:
            return True

        sample_step = max(1, int(round(2.0 / 0.5)))
        sampled = list(route[::sample_step])
        if sampled[-1] != route[-1]:
            sampled.append(route[-1])
        guard = getattr(self, "centerline_guard_margin_m", 1.30)
        for px, py in sampled:
            same_projection = min(
                (self._proj(px, py, link)
                 for link in same_links.values()),
                key=lambda projection: projection[1])
            opposite_projection = min(
                (self._proj(px, py, link)
                 for link in opposite_links.values()),
                key=lambda projection: projection[1])
            # Inferred opposite links can curve across the route inside an
            # intersection.  That is crossing traffic geometry, not a
            # parallel centreline. Collision checks still guard actual users;
            # apply the centreline-distance rule only where local headings
            # are genuinely opposite.
            heading_difference = abs(norm(
                same_projection[2] - opposite_projection[2]))
            if heading_difference < getattr(
                    self, "centerline_guard_min_opposite_heading",
                    math.radians(145.0)):
                continue
            same_distance = same_projection[1]
            opposite_distance = opposite_projection[1]
            # The ego centre must remain closer to same-direction geometry by
            # at least the vehicle-half-width safety allowance.
            if same_distance + guard > opposite_distance:
                return False
        return True

    def _gap_interval_candidates(self, base_route, blocker, lane_centers):
        """Gap passage (bounded): free lateral intervals between ALL static objects of the blocker's longitudinal
        group inside the connected same-direction lane band. Returns (candidates, group, reason); a candidate is a
        proposal only -- the finalized route is validated by the authoritative chain."""
        reference = self._resample(base_route, 0.5)
        if len(reference) < 2:
            return [], None, "short-base-route"
        rear, front, width, model = self._ego_body_extents()
        if model == "invalid-config":
            return [], None, "invalid-body"
        half_w = 0.5 * width
        req = (half_w + float(getattr(self, "path_block_clearance_m", 0.35)) +
               float(getattr(self, "local_bypass_tracking_margin", 0.15)))
        road_margin = float(getattr(self, "gap_road_margin_m", 0.15))
        join = float(getattr(self, "gap_group_join_m", 12.0))
        max_objects = int(getattr(self, "gap_group_max_objects", 8))
        max_shift = float(getattr(self, "free_bypass_max_shift_m", 7.5))
        items = []
        for raw in self.objects[:int(getattr(self, "gap_max_objects_scanned", 64))]:
            self._spend(4)
            ox, oy, vx, vy, _class_name, size = self._object_fields(raw)
            if not all(math.isfinite(v) for v in (ox, oy, vx, vy, size[0], size[1])):
                return [], None, "invalid-object"
            if math.hypot(vx, vy) > self.intersection_bypass_static_mps:
                continue          # moving objects are validated by the envelope, never used to propose a gap
            try:
                proj = project_to_frenet(ox, oy, reference)
            except ValueError:
                continue
            if abs(proj.d) > max_shift + 6.0:
                continue
            meta = self._object_metadata(raw)
            yaw = proj.heading if meta["heading"] is None else meta["heading"]
            along, lateral = obb_projected_half_extents(max(0.4, size[0]), max(0.4, size[1]), yaw, proj.heading)
            items.append((proj.s - along, proj.s + along, proj.d, lateral, meta["id"]))
        try:
            bproj = project_to_frenet(blocker[3], blocker[4], reference)
        except ValueError:
            return [], None, "blocker-projection"
        seed = [it for it in items if it[0] - 0.5 <= bproj.s <= it[1] + 0.5]
        if not seed:
            return [], None, "blocker-not-static"
        group = list(seed); g0 = min(it[0] for it in group); g1 = max(it[1] for it in group)
        for it in sorted(items, key=lambda q: q[0]):
            if it in group:
                continue
            if it[0] <= g1 + join and it[1] >= g0 - join and len(group) < max_objects:
                group.append(it); g0 = min(g0, it[0]); g1 = max(g1, it[1])
        if len(items) > len(group) and any(it[0] <= g1 + join and it[1] >= g0 - join and it not in group for it in items):
            return [], None, "search-budget:group-objects"
        centers = [c for c, _link in lane_centers]
        widths = [min(link.width_start, link.width_end) for _c, link in lane_centers]
        band_lo = min(c - 0.5 * w for c, w in zip(centers, widths)) + half_w + road_margin
        band_hi = max(c + 0.5 * w for c, w in zip(centers, widths)) - half_w - road_margin
        blocked = sorted((d - lat - req, d + lat + req) for _a, _b, d, lat, _i in group)
        free, cursor = [], band_lo
        for lo, hi in blocked:
            if lo > cursor:
                free.append((cursor, min(lo, band_hi)))
            cursor = max(cursor, hi)
            if cursor >= band_hi:
                break
        if cursor < band_hi:
            free.append((cursor, band_hi))
        slack = 0.02
        free = [(a, b) for a, b in free if b - a > 2.0 * slack]
        if not free:
            return [], {"start": g0, "end": g1, "ids": [it[4] for it in group]}, "no-width"
        reps = []
        for a, b in sorted(free, key=lambda q: (min(abs(q[0]), abs(q[1])) if q[0] * q[1] > 0 else 0.0, -(q[1] - q[0]))):
            for off in (0.5 * (a + b), max(a + slack, min(b - slack, 0.0))):
                if all(abs(off - r["offset"]) > 0.05 for r in reps):
                    reps.append({"offset": off, "interval": (a, b), "width": b - a})
        reps = reps[:int(getattr(self, "gap_max_candidates", 4))]
        group_info = {"start": g0, "end": g1, "ids": [it[4] for it in group], "entry_end": max(8.0, g0 - front - 1.0),
                      "hold_end": g1 + rear + 1.0}
        group_info["exit_end"] = group_info["hold_end"] + float(getattr(self, "gap_exit_length_m", 15.0))
        return reps, group_info, "ok"

    def _gap_group_route(self, base_route, offset, group, ex, ey):
        """Existing clamped offset B-spline with GROUP stations (hold through the whole group)."""
        reference = self._resample(base_route, 0.5)
        if len(reference) < 2:
            return [], None
        first_heading = math.atan2(reference[1][1] - reference[0][1], reference[1][0] - reference[0][0])
        start_offset = (-math.sin(first_heading) * (ex - reference[0][0]) +
                        math.cos(first_heading) * (ey - reference[0][1]))
        try:
            output = build_clamped_bspline_offset_path(
                reference, offset, group["entry_end"], group["hold_end"], group["exit_end"],
                start_offset=start_offset, step=0.5)
        except ValueError:
            return [], None
        # rejoin handover only once the spline is back on the base line (the generic rejoin fires 2 m before the
        # goal; handing over during the exit curve made the heading-matched rejoin blend overshoot by ~0.2 m)
        goal_frame = self._route_frame_at_distance(
            output, group["exit_end"] + float(getattr(self, "gap_goal_after_exit_m", 6.0)))
        return output, (goal_frame[0] if goal_frame is not None else output[-1])

    def _choose_gap_passage(self, cur, base_route, blocker, ex, ey, lane_centers):
        """Gap passage candidates, each validated by the AUTHORITATIVE chain before selection (finalize: kinematics,
        timeout-aware execution envelope over every actor, direction guard; full-footprint legal lane union).
        Returns (route, goal, speed, reason) or (None, None, None, reason)."""
        reps, group, reason = self._gap_interval_candidates(base_route, blocker, lane_centers)
        self.last_gap_diagnostics = {"reason": reason, "group": group, "candidates": [
            {"offset": round(r["offset"], 3), "width": round(r["width"], 3)} for r in reps], "rejects": []}
        if not reps:
            return None, None, None, "gap:%s" % reason
        speed = min(self.free_bypass_max_mps, float(getattr(self, "gap_passage_max_mps", 2.0)))
        for rep_ in reps:
            self._spend(200)
            offset = rep_["offset"]
            if abs(offset) > self.free_bypass_max_shift_m:
                self.last_gap_diagnostics["rejects"].append((offset, "max-shift")); continue
            crossed = [(abs(c), link) for c, link in lane_centers
                       if c * offset > 0.0 and abs(c) <= abs(offset) + self.free_bypass_lateral_bias_m + 0.2]
            if not crossed:
                crossed = [min(((abs(c - offset), link) for c, link in lane_centers), key=lambda q: q[0])]
            moving_ok = True
            for _d, link in sorted(crossed, key=lambda q: q[0]):
                ok, gap_reason, _score = self._target_lane_gap_ok(link, ex, ey, moving_only=True)
                if not ok:
                    moving_ok = False
                    self.last_gap_diagnostics["rejects"].append((offset, "moving-conflict:%s" % gap_reason)); break
            if not moving_ok:
                continue
            route, goal = self._gap_group_route(base_route, offset, group, ex, ey)
            if len(route) < 2 or not self._route_stays_same_direction(cur, route):
                self.last_gap_diagnostics["rejects"].append((offset, "direction-or-build")); continue
            final, final_reason, final_cap = self._finalize_local_bypass(route, speed, goal)
            if final is None:
                self.last_gap_diagnostics["rejects"].append((offset, "finalize:%s" % final_reason)); continue
            # Legal full-footprint lane union over the modified section (through the rejoin goal plus one body
            # length); beyond it the finalized route is the unmodified base route.
            span = group["exit_end"] + sum(self._ego_body_extents()[:2]) + 2.0
            covered, travelled = [final[0]], 0.0
            for q0, q1 in zip(final, final[1:]):
                if travelled >= span:
                    break
                travelled += math.hypot(q1[0] - q0[0], q1[1] - q0[1])
                covered.append(q1)
            legal_ok, legal_reason = self._source_return_legal_corridor(cur, covered)
            if not legal_ok:
                self.last_gap_diagnostics["rejects"].append((offset, "road-boundary:%s" % legal_reason)); continue
            return route, goal, min(speed, final_cap), (
                "gap offset=%.2f width=%.2f group=%s stations=%.1f..%.1f cap=%.1fkm/h" % (
                    offset, rep_["width"], group["ids"], group["start"], group["end"], min(speed, final_cap) * 3.6))
        return None, None, None, "gap:all-rejected %s" % self.last_gap_diagnostics["rejects"][:4]

    def _choose_free_bypass(self, cur, base_route, blocker, ex, ey):
        if not getattr(self, "free_bypass_enable", False):
            return None, None, None, "disabled"
        if self._blocker_is_vulnerable(blocker):
            return None, None, None, "vulnerable-stop-only"
        if (blocker[0] < self.intersection_bypass_min_object_m or
                abs(blocker[2]) > self.intersection_bypass_static_mps):
            return None, None, None, "late-or-moving"
        frame = self._route_frame_at_distance(
            base_route, max(0.0, blocker[0]))
        if frame is None:
            return None, None, None, "short-base-route"
        reference_point, heading = frame
        lane_centers = []
        for link in self._same_direction_lateral_links(cur):
            nearest = min(link.pts, key=lambda point:
                          (point[0] - reference_point[0]) ** 2 +
                          (point[1] - reference_point[1]) ** 2)
            separation = math.hypot(
                nearest[0] - reference_point[0],
                nearest[1] - reference_point[1])
            if separation > self.free_bypass_max_shift_m + 2.0:
                continue
            lateral = (
                -math.sin(heading) * (nearest[0] - reference_point[0]) +
                math.cos(heading) * (nearest[1] - reference_point[1]))
            lane_centers.append((lateral, link))

        if len(lane_centers) < 2:
            return None, None, None, "no-white-lane-corridor"
        gap_reason = "gap:disabled"
        if getattr(self, "gap_passage_enable", True):
            route, goal, speed, gap_reason = self._choose_gap_passage(cur, base_route, blocker, ex, ey, lane_centers)
            if route is not None:
                return route, goal, speed, gap_reason
        self.last_gap_reason = gap_reason
        if True:
            centers = [item[0] for item in lane_centers]
            half_width = max(
                1.5,
                0.5 * max(
                    max(link.width_start, link.width_end)
                    for _center, link in lane_centers))
            edge_allowance = max(0.2, half_width - 1.4)
            minimum_offset = min(centers) - edge_allowance
            maximum_offset = max(centers) + edge_allowance
            bias = self.free_bypass_lateral_bias_m
            offsets = set()
            for center in centers:
                for offset in (center, center - bias, center + bias):
                    self._spend(200)   # candidate route build/check
                    offsets.add(min(maximum_offset, max(minimum_offset, offset)))
            required_lateral = (
                0.5 * getattr(self, "bypass_vehicle_width", 1.892) +
                0.5 * float(blocker[6][1]) +
                getattr(self, "path_block_clearance_m", 0.35) +
                getattr(self, "local_bypass_tracking_margin", 0.15))
            offsets = sorted(offsets, key=lambda value: (
                max(0.0, required_lateral - abs(value - blocker[1])),
                abs(value), value))[:getattr(
                    self, "max_avoid_candidates", 3)]

        candidates = []
        for offset in offsets:
            self._spend(200)   # candidate route build/check
            if abs(offset) > self.free_bypass_max_shift_m:
                continue
            gap_score = -float("inf")
            if lane_centers:
                # A multi-lane road-space path crosses every lane between the
                # global route and its selected offset.  Validate each crossed
                # lane, not only the final one; otherwise a fast rear vehicle
                # in the intermediate lane could be missed.
                crossed_links = [
                    (abs(center), link)
                    for center, link in lane_centers
                    if (center * offset > 0.0 and
                        abs(center) <= abs(offset) +
                        self.free_bypass_lateral_bias_m + 0.2)
                ]
                if not crossed_links:
                    _center, target_link = min(
                        lane_centers,
                        key=lambda item: abs(item[0] - offset))
                    crossed_links = [(abs(_center), target_link)]
                gap_scores = []
                corridor_gap_ok = True
                for _distance, crossed_link in sorted(crossed_links):
                    gap_ok, _gap_reason, checked_score = (
                        self._target_lane_gap_ok(crossed_link, ex, ey))
                    if not gap_ok:
                        corridor_gap_ok = False
                        break
                    gap_scores.append(checked_score)
                if not corridor_gap_ok:
                    continue
                gap_score = min(gap_scores) if gap_scores else -float("inf")
                _center, target_link = min(
                    lane_centers,
                    key=lambda item: abs(item[0] - offset))
                min_return_s = max(
                    self.transition_m,
                    float(blocker[0]) + self.front_safety_gap)
                return_ok, _return_reason = self._has_rejoin_path(
                    cur, target_link, min_return_s)
                if not gap_ok or not return_ok:
                    continue
            route, goal = self._free_bypass_candidate(
                base_route, blocker, offset, ex, ey)
            if not self._route_stays_same_direction(cur, route):
                continue
            # revision3: an illegal (road-exit) candidate must not monopolise selection -> try the next offset
            if not self._route_road_legal(cur, route, goal)[0]:
                continue
            clearance = self._intersection_bypass_clearance(route, ex, ey)
            if clearance >= 0.0:
                candidates.append((
                    abs(offset), clearance, gap_score, offset, route, goal))
        if not candidates:
            return None, None, None, "free-corridor-blocked"
        # Use the nearest collision-free road-space line. A maximum-clearance
        # score could cross two or three lanes even when one smooth offset was
        # sufficient.
        _excursion, clearance, gap_score, offset, route, goal = min(
            candidates, key=lambda item: (item[0], -item[1]))
        fast_straight = (
            gap_score >= getattr(self, "straight_fast_min_gap_margin_m", 2.0) and
            self._route_is_straight(base_route))
        speed_cap = (
            getattr(self, "straight_free_bypass_max_mps",
                    self.free_bypass_max_mps)
            if fast_straight else self.free_bypass_max_mps)
        return route, goal, speed_cap, (
            "offset=%.2f clearance=%.2f gap_margin=%.2f speed=%.1fkm/h"
            % (offset, clearance, gap_score, speed_cap * 3.6))

    def _local_bypass_route_check(self, route, speed_cap, goal=None):
        """Exact-point validation of a LOCAL_BYPASS route this tick (entry
        gate, gap replan and every positive-cap KEEP): objects fresh,
        kinematics at the commanded speed (lateral-accel/steer-rate lower the
        cap, curvature/steer reject), then the certified continuous swept
        body check at that final speed. Horizon: the route up to the bypass
        goal + 10 m (whole route without a goal). Returns (ok, reason, cap,
        checked_points)."""
        if len(route) < 2:
            return False, "short-route", 0.0, route
        fresh, fresh_reason = self._objects_fresh_for_commit()
        if not fresh:
            return False, fresh_reason, 0.0, route
        checked = list(route)
        if goal is not None:
            goal_index = min(range(len(checked)), key=lambda index: (
                (checked[index][0] - goal[0]) ** 2 +
                (checked[index][1] - goal[1]) ** 2))
            need = self._envelope_required_length(max(0.5, float(speed_cap or 0.0)))
            pose = self._execution_ego_pose(checked)
            lead_in = 0.0 if pose is None else math.hypot(
                pose[0] - checked[0][0], pose[1] - checked[0][1])
            prefix = self._route_prefix_by_length(
                checked, max(self._polyline_length(checked[:goal_index + 1]) + 10.0,
                              (need or 0.0) + lead_in + 2.0))
            if len(prefix) >= 2:
                checked = prefix
        speed = max(0.5, float(speed_cap or 0.0))
        if len(checked) >= 3:
            kinematics_args = (
                getattr(self, "bypass_wheelbase", 3.0),
                getattr(self, "bypass_max_steer_deg", 35.0),
                getattr(self, "bypass_max_steer_rate_deg_s", 50.0))
            lateral = getattr(self, "bypass_max_lateral_accel", 2.26)
            kinematics = validate_path_kinematics(
                checked, kinematics_args[0], kinematics_args[1],
                kinematics_args[2], speed, lateral)
            if not kinematics["valid"]:
                hard = set(kinematics["reasons"]) & {"curvature", "steer"}
                if hard:
                    return False, "kinematics:%s" % ",".join(sorted(hard)), 0.0, checked
                safe = min(speed, kinematics["speed_cap_mps"])
                rate = kinematics["max_steer_rate_deg_s"]
                if rate > 1.0e-6:
                    safe = min(safe, speed * kinematics_args[2] / rate)
                speed = 0.95 * safe
                if speed < 0.5:
                    return False, "kinematics-cap-too-low", 0.0, checked
                kinematics = validate_path_kinematics(
                    checked, kinematics_args[0], kinematics_args[1],
                    kinematics_args[2], speed, lateral)
                if not kinematics["valid"]:
                    return False, "kinematics:%s" % ",".join(
                        kinematics["reasons"]), 0.0, checked
        ex, ey = getattr(self, "_tick_ego", None) or checked[0]
        margin = float(getattr(
            self, "local_bypass_final_clearance_m",
            getattr(self, "intersection_bypass_clearance_m", 0.35)))
        obstacles = self._prefilter_obb_obstacles(checked, ex, ey, speed, margin)
        # revision3: the envelope runs even with no obstacle -- it also proves that the timeout-aware stop
        # (tick + reaction + controller feed timeout + braking) fits inside the checked points (the revision2
        # guard 'if obstacles' skipped it and certified e.g. 3 m/s on a 6 m route)
        clearance = self._ego_swept_clearance(
            checked, obstacles, speed, safety_margin=margin,
            sample_step=0.2)
        if not clearance["valid"] and (clearance.get("closest") or {}).get("kind") == "envelope-beyond-checked-route":
            # route too short for this speed: the largest speed whose stop fits (v_top also bounded by the
            # measured speed and the previous command); never extrapolate beyond the points
            pose = self._execution_ego_pose(checked)
            s0 = self._point_projection_on_route(pose[0], pose[1], checked) if pose is not None else None
            s0 = 0.0 if s0 is None else s0[0]
            total = self._polyline_length(checked)
            lo, hi = 0.0, speed
            for _ in range(12):
                mid = 0.5 * (lo + hi)
                need = self._envelope_required_length(mid)
                if need is not None and s0 + need <= total:
                    lo = mid
                else:
                    hi = mid
            if lo >= 0.5:
                speed = lo
                obstacles = self._prefilter_obb_obstacles(checked, ex, ey, speed, margin)
                clearance = self._ego_swept_clearance(
                    checked, obstacles, speed, safety_margin=margin, sample_step=0.2)
        if not clearance["valid"]:
            if clearance.get("budget_exhausted"):
                return False, "work-budget-exhausted", 0.0, checked
            closest = clearance["closest"] or {}
            if closest.get("kind") == "envelope-beyond-checked-route":
                return False, "route-too-short-for-stop %s" % (closest.get("detail") or ""), 0.0, checked
            return False, "final-obb id=%s clearance=%.2f" % (
                closest.get("object_id"), clearance["minimum_clearance"]), 0.0, checked
        return True, "final-ok", speed, checked

    def _finalize_local_bypass(self, route, speed_cap, goal=None):
        """Finalize a LOCAL_BYPASS route into its published form (0.5 m
        linear resampling), apply the direction guard to those points and
        validate them (_local_bypass_route_check). Returns (FinalRoute,
        reason, cap) or (None, reason, 0.0)."""
        final = FinalRoute(self._resample(route, 0.5))
        if len(final) < 2:
            return None, "short-final-route", 0.0
        cur = getattr(self, "_tick_cur", None)
        ok, reason, cap, checked = self._local_bypass_route_check(
            final, speed_cap, goal)
        if not ok:
            return None, reason, 0.0
        if cur is not None and not self._route_stays_same_direction(cur, checked):
            return None, "final-direction-guard", 0.0
        # revision3: mandatory full-body legal road containment on the EXACT final points (every commit path,
        # no true-stop exemption); unknown topology rejects; certificate only after it passes
        legal_ok, legal_reason = self._route_road_legal(cur, list(checked))
        if not legal_ok:
            return None, "road-boundary:%s" % legal_reason, 0.0
        self._certify(final, cap)
        return final, "final-ok", cap

    def _route_road_legal(self, cur, route, goal=None):
        """revision3: full-footprint legal lane union (unchanged _source_return_legal_corridor) over the changed
        section of a route: through the goal plus one body length plus 2 m when a goal is given, else the given
        points. Unknown topology (no current link) is illegal."""
        if cur is None:
            return False, "unknown-topology"
        section = list(route)
        if goal is not None and len(section) >= 2:
            gi = min(range(len(section)), key=lambda i: (section[i][0] - goal[0]) ** 2 + (section[i][1] - goal[1]) ** 2)
            rear, front, _w, _m = self._ego_body_extents()
            section = self._route_prefix_by_length(section, self._polyline_length(section[:gi + 1]) + rear + front + 2.0)
        if len(section) < 2:
            return False, "short-section"
        return self._source_return_legal_corridor(cur, section)

    # ── handover revision: reference continuity when an active reference is REPLACED while moving ──
    HANDOVER_ALIGNED_M = 0.05
    HANDOVER_ALIGNED_DEG = 1.0
    HANDOVER_BLENDS_M = (10.0, 15.0, 20.0)

    def _handover_retained(self):
        """The reference the controller is tracking (exact last emitted forward points) when a replacement must
        preserve continuity: ego moving (ego_v > 0.1) or a positive command outstanding. None otherwise (stopped:
        the existing direct replacement applies -> eventual progress). Now also returned at standstill; the
        callers decide with _handover_moving()."""
        last = getattr(self, "_last_emitted", None)
        if not last or len(last.get("points", ())) < 2 or getattr(self, "output_gear", "D") != "D":
            return None
        return [tuple(q) for q in last["points"]]

    def _handover_moving(self):
        """Moving (or a positive command outstanding): a replacement without a certified join is NOT allowed.
        At standstill a join is still attempted first; only if none certifies the direct replacement applies."""
        # revision2 (H4): creeping is moving; only a TRUE stop (<= 0.01 m/s and no outstanding command) is not
        return (float(getattr(self, "ego_v", 0.0) or 0.0) > 0.01 or
                float(getattr(self, "last_speed_command_mps", 0.0) or 0.0) > 0.0)

    @staticmethod
    def _handover_project(route, cum, px, py):
        """(arc, signed lateral, segment index, inside) of a point on a polyline (inside: foot not clamped at ends)."""
        best = None
        for i in range(len(route) - 1):
            ax, ay = route[i]; bx, by = route[i + 1]
            dx, dy = bx - ax, by - ay; l2 = dx * dx + dy * dy
            if l2 <= 1.0e-18:
                continue
            raw = ((px - ax) * dx + (py - ay) * dy) / l2; u = max(0.0, min(1.0, raw))
            qx, qy = ax + u * dx, ay + u * dy; d = math.hypot(px - qx, py - qy)
            if best is None or d < best[0]:
                seg = math.sqrt(l2)
                side = (dx * (py - ay) - dy * (px - ax)) / seg
                best = (d, cum[i] + u * seg, side, i, (i > 0 or raw >= 0.0) and (i < len(route) - 2 or raw <= 1.0))
        return None if best is None else best[1:]

    def _handover_join(self, retained, candidate, ex, ey):
        """Continuity join of a replacement reference to the retained emitted reference (design.json).
        Returns ("aligned", [candidate]) when no join is needed, ("join", [(route, blend_m), ...]) with bounded
        alternatives (shortest blend first), or ("impossible", reason)."""
        R = self._resample(list(retained), 0.5); C = self._resample(list(candidate), 0.5)
        if len(R) < 2 or len(C) < 2:
            return "impossible", "short-reference"
        def cum_of(route):
            out = [0.0]
            for a, b in zip(route, route[1:]):
                out.append(out[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
            return out
        Rs, Cs = cum_of(R), cum_of(C)
        v = max(0.0, float(getattr(self, "ego_v", 0.0) or 0.0))
        hold = 5.0 + max(1.0, v * 1.0) + 1.0
        pe = self._handover_project(R, Rs, ex, ey); pc = self._handover_project(C, Cs, ex, ey)
        if pe is None or pc is None:
            return "impossible", "projection"
        s_e, s_ce = pe[0], pc[0]
        # lateral offset of the retained reference in the candidate frame (candidate normals)
        def heading(route, i):
            j = min(i + 1, len(route) - 1); k = max(0, j - 1)
            return math.atan2(route[j][1] - route[k][1], route[j][0] - route[k][0])
        offsets = []
        for j in range(len(C)):
            self._spend(1)
            if Cs[j] < s_ce - 1.0 or Cs[j] > s_ce + hold + max(self.HANDOVER_BLENDS_M) + 1.0:
                offsets.append(None); continue
            pr = self._handover_project(R, Rs, C[j][0], C[j][1])
            if pr is None or not pr[3]:
                offsets.append(None); continue
            hc, hr = heading(C, j), heading(R, pr[2])
            # exact offset ALONG the candidate normal: intersect the normal line with the retained polyline near
            # the projected segment (a nearest-point offset is wrong by |d| sin(angle) and kinks the seam)
            nx, ny = -math.sin(hc), math.cos(hc); d_exact = None
            for i in range(max(0, pr[2] - 4), min(len(R) - 1, pr[2] + 5)):
                ax, ay = R[i]; bx, by = R[i + 1]; ex_, ey_ = bx - ax, by - ay
                den = nx * ey_ - ny * ex_
                if abs(den) < 1.0e-12:
                    continue
                rx, ry = ax - C[j][0], ay - C[j][1]
                d = (rx * ey_ - ry * ex_) / den; u = (rx * ny - ry * nx) / den
                if -1.0e-9 <= u <= 1.0 + 1.0e-9 and (d_exact is None or abs(d) < abs(d_exact)):
                    d_exact = d
            if d_exact is None:
                offsets.append(None); continue
            offsets.append((d_exact, math.degrees(abs(math.remainder(hr - hc, 2 * math.pi)))))   # retained - candidate
        # revision2 (H3): the promised continuity window must be COMPLETELY covered before any 'aligned' approval:
        # retained and candidate both extend over it, every candidate sample in it has a valid inside projection
        # and an exact normal intersection on the retained reference, and retained stations increase monotonically
        w_end = s_ce + hold + min(self.HANDOVER_BLENDS_M)
        if Rs[-1] < s_e + hold + min(self.HANDOVER_BLENDS_M) or Cs[-1] < w_end:
            return "impossible", "retained-shorter-than-hold"
        idx = [j for j in range(len(C)) if s_ce - 1.0e-9 <= Cs[j] <= w_end]
        stations = []
        for j in idx:
            if offsets[j] is None:
                return "impossible", "coverage-gap"
            pr = self._handover_project(R, Rs, C[j][0], C[j][1])
            stations.append(pr[0])
        if len(idx) < 2 or any(b < a - 1.0e-6 for a, b in zip(stations, stations[1:])):
            return "impossible", "coverage-order"
        window = [offsets[j] for j in idx]
        if max(abs(o[0]) for o in window) <= self.HANDOVER_ALIGNED_M and \
                max(o[1] for o in window) <= self.HANDOVER_ALIGNED_DEG:
            return "aligned", [candidate]
        start = max(0, pe[2])                                   # retained point at/behind the ego
        prefix = [R[i] for i in range(start, len(R)) if Rs[i] <= s_e + hold]
        ph = self._handover_project(C, Cs, prefix[-1][0], prefix[-1][1])
        if ph is None:
            return "impossible", "hold-projection"
        s_h = ph[0]
        joins = []
        for blend in self.HANDOVER_BLENDS_M:
            s_b = s_h + blend
            if Cs[-1] < s_b + 1.0:
                continue
            route = list(prefix); ok = True
            for j in range(len(C)):
                if Cs[j] <= s_h + 0.25:                       # no micro segment at the seam
                    continue
                if Cs[j] <= s_b:
                    if offsets[j] is None:
                        ok = False; break
                    u = (Cs[j] - s_h) / blend; w = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)
                    h = heading(C, j); d = (1.0 - w) * offsets[j][0]
                    route.append((C[j][0] - math.sin(h) * d, C[j][1] + math.cos(h) * d))
                else:
                    route.append(C[j])
            if ok and len(route) >= 3:
                joins.append((route, blend))
        return ("join", joins) if joins else ("impossible", "no-bounded-blend")

    def _handover_certify(self, retained, joins, speed_cap, goal, ex, ey, attempts=None):
        """revision2 (H1/H2): ONE exact-final-form contract for every handover caller. For each bounded join
        alternative: the published form is produced once by _finalize_local_bypass (FinalRoute; kinematics,
        timeout-aware all-actor envelope, direction guard, same-tick certificate), then the full-footprint legal
        lane union over the whole changed section (unknown topology = reject), then the retained-prefix continuity of
        the FINAL points (<= 0.10 m / 2 deg at every 0.25 m station of the hold window). Returns
        (FinalRoute, cap, blend, None) or (None, 0.0, None, reason). Commits nothing."""
        cur = getattr(self, "_tick_cur", None)
        why = "no-join"
        v = max(0.0, float(getattr(self, "ego_v", 0.0) or 0.0))
        hold = 5.0 + max(1.0, v * 1.0) + 1.0
        R = self._resample(list(retained), 0.5)
        Rc = [0.0]
        for a, b in zip(R, R[1:]):
            Rc.append(Rc[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        pe = self._handover_project(R, Rc, ex, ey)
        for joined, blend in (joins if attempts is None else joins[:attempts]):
            final, reason, cap = self._finalize_local_bypass(joined, speed_cap, goal)
            if final is None:
                why = "blend%.0f:%s" % (blend, reason); continue
            # revision3: the full-footprint lane union (unknown topology = reject) is now enforced inside
            # _finalize_local_bypass on the exact final points over the envelope-validated section (no duplicate)
            Fp = list(final); Fc = [0.0]
            for a, b in zip(Fp, Fp[1:]):
                Fc.append(Fc[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
            worst_d = worst_h = 0.0; cont_ok = pe is not None
            k = 0.0
            while cont_ok and k <= hold + 1.0e-9:
                sr = pe[0] + k
                if sr > Rc[-1]:
                    cont_ok = False; break
                i = max(0, min(len(R) - 2, next((q for q in range(len(Rc) - 1) if Rc[q + 1] >= sr), len(R) - 2)))
                t = 0.0 if Rc[i + 1] <= Rc[i] else (sr - Rc[i]) / (Rc[i + 1] - Rc[i])
                px, py = R[i][0] + t * (R[i + 1][0] - R[i][0]), R[i][1] + t * (R[i + 1][1] - R[i][1])
                hr = math.atan2(R[i + 1][1] - R[i][1], R[i + 1][0] - R[i][0])
                pf = self._handover_project(Fp, Fc, px, py)
                if pf is None:
                    cont_ok = False; break
                j = pf[2]
                hf = math.atan2(Fp[j + 1][1] - Fp[j][1], Fp[j + 1][0] - Fp[j][0])
                worst_d = max(worst_d, abs(pf[1])); worst_h = max(worst_h, abs(math.degrees(math.remainder(hf - hr, 2 * math.pi))))
                k += 0.25
            if not cont_ok or worst_d > 0.10 or worst_h > 2.0:
                why = "blend%.0f:continuity:%.3fm/%.2fdeg" % (blend, worst_d, worst_h); continue
            return final, cap, blend, None
        return None, 0.0, None, why

    def _handover_hold(self, retained, reason, status_prefix):
        """No certified continuity join: install nothing, keep the retained reference at cap 0 (retry next tick)."""
        self.auto_speed_cap = 0.0
        return FinalRoute(tuple(tuple(q) for q in retained)), (
            "AUTO_HANDOVER_HOLD %s | %s" % (reason, status_prefix[:120]))

    def _activate_local_bypass(
            self, route, goal, blocker, speed_cap, status_prefix):
        """Install one road-space bypass and cancel the HD-link lane change.

        The route is finalized and re-validated first; on failure nothing is
        installed and the tick stops (cap 0) on this tick's reference."""
        retained = self._handover_retained()
        handover = handover_note = ""
        if retained is not None:
            kind, joins = self._handover_join(retained, route, *(getattr(self, "_tick_ego", None) or route[0]))
            if kind == "impossible" and self._handover_moving():
                return self._handover_hold(retained, "activation:%s" % joins, status_prefix)
            if kind == "join":
                ex_, ey_ = getattr(self, "_tick_ego", None) or route[0]
                # bounded work: all blends while moving; one attempt at a true stop (then direct replacement)
                final_j, cap_j, blend, why = self._handover_certify(
                    retained, joins, speed_cap, goal, ex_, ey_, None if self._handover_moving() else 1)
                if final_j is None and self._handover_moving():
                    return self._handover_hold(retained, "activation:%s" % why, status_prefix)
                if final_j is not None:
                    final, final_reason, final_cap = final_j, "final-ok", cap_j
                    handover = " handover=blend%.0f" % blend
                    route = final
                else:
                    handover_note = " handover=direct-at-true-stop:%s" % why
        if not handover:
            final, final_reason, final_cap = self._finalize_local_bypass(
                route, speed_cap, goal)
        if final is None:
            self.auto_speed_cap = 0.0
            fallback = getattr(self, "_tick_reference_route", None) or route
            return fallback, ("AUTO_LOCAL_BYPASS_FINAL_REJECT %s | %s"
                              % (final_reason, status_prefix[:120]))
        route = final
        speed_cap = final_cap
        self.mode = "LOCAL_BYPASS"
        self.local_bypass_route = final
        self.local_bypass_progress = 0
        self.local_bypass_goal = goal
        self.local_bypass_source = (blocker[3], blocker[4])
        self.local_bypass_max_mps = speed_cap
        self.opposite_wait_cycles = 0
        self.changing = False
        self.active_target_lane = None
        self.change_dst_idx = None
        self.clear_cycles = 0
        self.local_bypass_replan_count = 0
        self.local_bypass_rejoin_handovers = 0
        self.path_revision = getattr(self, "path_revision", 0) + 1
        handover = handover or handover_note
        self.last_local_decision = status_prefix + handover
        self.auto_speed_cap = speed_cap
        return route, status_prefix + handover

    def _replan_local_bypass_gap(
            self, published_route, blocker, ex, ey, cur=None):
        """Nudge a blocked local path through a measured static gap."""
        if blocker is None or self._blocker_is_vulnerable(blocker):
            return None, None, "vulnerable-or-empty"
        if abs(blocker[2]) > self.intersection_bypass_static_mps:
            return None, None, "moving-blocker"
        if self.ego_v > 1.0:
            return None, None, "ego-moving"
        if getattr(self, "local_bypass_replan_count", 0) >= 2:
            return None, None, "replan-limit"

        step = self.local_bypass_nudge_step_m
        count = max(1, int(math.floor(
            self.local_bypass_nudge_max_m / step + 1e-6)))
        candidates = []
        rapid_entry = blocker[0] < self.intersection_bypass_min_object_m
        for index in range(1, count + 1):
            shift = index * step
            for offset in (shift, -shift):
                self._spend(200)   # candidate route build/check
                route, goal = self._free_bypass_candidate(
                    published_route, blocker, offset, ex, ey,
                    rapid_entry=rapid_entry)
                if (cur is not None and
                        (not self._route_stays_same_direction(cur, route) or
                         not self._route_within_crossable_corridor(cur, route))):
                    continue
                clearance = self._intersection_bypass_clearance(
                    route, ex, ey)
                if clearance >= getattr(
                        self, "local_bypass_tracking_margin", 0.15):
                    # revision3: a road-exit nudge must not monopolise the replan -> next candidate
                    if not self._route_road_legal(cur, route, goal)[0]:
                        continue
                    candidates.append((abs(offset), -clearance,
                                       offset, clearance, route, goal))
            if candidates:
                # revision3 (bounded work, identical selection): shifts are visited in increasing |offset| and the
                # selection key is (|offset|, -clearance, offset), so no later shift can be selected
                break
        if not candidates:
            return None, None, "no-measured-gap"
        _distance, _rank, offset, clearance, route, goal = min(candidates)
        return route, goal, (
            "offset=%.2f clearance=%.2f" % (offset, clearance))

    def _prefer_target_lane_speed(self, link, ex, ey):
        target_cap = self._link_front_speed_cap(link, ex, ey)
        if target_cap is not None:
            self.auto_speed_cap = (
                target_cap if self.auto_speed_cap is None
                else min(self.auto_speed_cap, target_cap))

    def _lane_change_route(self, cur, dst, ex, ey):
        cur_pts = self._route_to_link(ex, ey, cur)
        dst_pts = self._resample(self._chain_pts_raw(self._forward(dst, self.route_len)), 0.5)
        if len(cur_pts) < 2 or len(dst_pts) < 2:
            return cur_pts
        trans_n = max(4, int(self.transition_m / 0.5))
        route = []
        for i, point in enumerate(cur_pts[:trans_n]):
            w = i / max(1, trans_n - 1)
            j = min(range(len(dst_pts)), key=lambda k: (dst_pts[k][0] - point[0]) ** 2 + (dst_pts[k][1] - point[1]) ** 2)
            route.append(((1.0 - w) * point[0] + w * dst_pts[j][0], (1.0 - w) * point[1] + w * dst_pts[j][1]))
        if route:
            j = min(range(len(dst_pts)), key=lambda k: (dst_pts[k][0] - route[-1][0]) ** 2 + (dst_pts[k][1] - route[-1][1]) ** 2)
            route.extend(dst_pts[j + 1:])
        return route

    def _adjacent_candidates(self, cur):
        directional = getattr(self, "directional_same", {})
        if directional:
            out = []
            for edge in directional.get(cur.idx, []):
                target = edge.get("target_link_id")
                policy = getattr(self, "link_policies", {}).get(target)
                if (target in self.links and
                        policy not in ("forbidden", "last_resort")):
                    out.append((edge.get("declared_side", edge.get("side", "unknown")),
                                self.links[target]))
            return out
        out = []
        policies = getattr(self, "link_policies", {})
        if (cur.can_l and cur.dst_l in self.links and
                policies.get(cur.dst_l) not in ("forbidden", "last_resort")):
            out.append(("left", self.links[cur.dst_l]))
        if (cur.can_r and cur.dst_r in self.links and
                policies.get(cur.dst_r) not in ("forbidden", "last_resort")):
            out.append(("right", self.links[cur.dst_r]))
        return out

    def _prioritize_avoid_candidates(self, candidates):
        policies = getattr(self, "link_policies", {})
        avoidance = [item for item in candidates
                     if policies.get(item[1].idx) == "avoidance"]
        if avoidance:
            return avoidance
        normal = [item for item in candidates
                  if policies.get(item[1].idx) == "normal"]
        return normal if normal else candidates

    def _opposite_candidates(self, cur):
        directional = getattr(self, "directional_opposite", {})
        output = []
        for edge in directional.get(cur.idx, []):
            target = edge.get("target_link_id")
            policy = getattr(self, "link_policies", {}).get(target)
            # A manual normal/avoidance label must never reverse a link whose
            # geometry is classified as opposite direction. Direction safety
            # wins; only explicit last_resort or an unassigned legacy target
            # remains eligible for the guarded contraflow state.
            if (target in self.links and
                    policy not in ("forbidden", "normal", "avoidance")):
                output.append((edge.get("side", "unknown"), self.links[target], edge))
        return output

    def _opposite_corridor_clear(self, link, ex, ey):
        ref = self._contraflow_reference(link)
        if len(ref) < 2:
            return False, "short-contraflow-route"
        arc = [0.0]
        for first, second in zip(ref, ref[1:]):
            arc.append(arc[-1] + math.hypot(
                second[0] - first[0], second[1] - first[1]))
        ego_index = min(range(len(ref)), key=lambda index:
                        (ref[index][0] - ex) ** 2 + (ref[index][1] - ey) ** 2)
        for raw in self.objects:
            self._spend(1)
            ox, oy, _vx, _vy, _class_name, _size = self._object_fields(raw)
            object_index = min(range(len(ref)), key=lambda index:
                               (ref[index][0] - ox) ** 2 + (ref[index][1] - oy) ** 2)
            relative_s = arc[object_index] - arc[ego_index]
            if (relative_s < -self.opposite_clear_behind_m or
                    relative_s > self.opposite_clear_ahead_m):
                continue
            heading_index = min(object_index, len(ref) - 2)
            heading = math.atan2(
                ref[heading_index + 1][1] - ref[heading_index][1],
                ref[heading_index + 1][0] - ref[heading_index][0])
            lateral = abs(
                -math.sin(heading) * (ox - ref[object_index][0]) +
                math.cos(heading) * (oy - ref[object_index][1]))
            if lateral <= self.opposite_corridor_lat_m:
                return False, "occupied s=%.1f lat=%.1f" % (relative_s, lateral)
        return True, "clear"

    def _choose_opposite_last_resort(self, cur, ex, ey):
        if not getattr(self, "allow_opposite_last_resort", False):
            return None, "opposite-disabled"
        if not self.use_base_route or len(self.base_route) < 2:
            return None, "no-global-rejoin"
        if (self.base_match_distance is None or
                not math.isfinite(self.base_match_distance) or
                self.base_match_distance > self.global_rejoin_trigger_m):
            return None, "global-anchor-too-far"
        if self.ego_v > self.opposite_nearly_stopped_mps:
            return None, "not-stopped %.2f>%.2f" % (
                self.ego_v, self.opposite_nearly_stopped_mps)
        if self.opposite_wait_cycles < self.opposite_wait_cycles_needed:
            return None, "wait %d/%d" % (
                self.opposite_wait_cycles, self.opposite_wait_cycles_needed)
        candidates = []
        reasons = []
        for side, link, edge in self._opposite_candidates(cur):
            if edge.get("separation_m", float("inf")) > self.opposite_max_separation_m:
                reasons.append("%s:separation" % side)
                continue
            if edge.get("estimated_overlap_m", 0.0) < self.opposite_min_overlap_m:
                reasons.append("%s:short-overlap" % side)
                continue
            clear, reason = self._opposite_corridor_clear(link, ex, ey)
            if clear:
                candidates.append((side, link, edge))
            else:
                reasons.append("%s:%s" % (side, reason))
        if not candidates:
            return None, "opposite-blocked " + ",".join(reasons)
        side, link, edge = max(candidates, key=lambda item: (
            item[2].get("estimated_overlap_m", 0.0),
            -abs(item[2].get("separation_m", 99.0) - 3.5)))
        return link, "%s/opposite-last-resort/overlap=%.1f" % (
            side, edge.get("estimated_overlap_m", 0.0))

    def _select_next_lane_link(self, cur, target_lane):
        candidates = self._adjacent_candidates(cur)
        if not candidates:
            return None, "no-adjacent"
        exact = [(side, link) for side, link in candidates if link.lane == target_lane]
        if exact:
            return exact[0][1], exact[0][0]
        side, link = min(candidates, key=lambda item: abs(item[1].lane - target_lane))
        if abs(link.lane - target_lane) < abs(cur.lane - target_lane):
            return link, side
        return None, "no-progress"

    def _backup_rear_clear(self, ex, ey, eh):
        """Check the full swept corridor before commanding reverse gear."""
        clearance = getattr(self, "backup_recovery_rear_clear_m", 8.0)
        corridor = getattr(self, "backup_recovery_rear_lateral_m", 1.8)
        for raw in self.objects:
            self._spend(1)
            ox, oy, _vx, _vy, class_name, size = self._object_fields(raw)
            dx = ox - ex
            dy = oy - ey
            longitudinal = math.cos(eh) * dx + math.sin(eh) * dy
            if longitudinal >= -0.25:
                continue
            rear_distance = -longitudinal - 0.5 * max(0.4, size[0])
            lateral = -math.sin(eh) * dx + math.cos(eh) * dy
            lateral_limit = corridor + 0.5 * max(0.4, size[1])
            if rear_distance <= clearance and abs(lateral) <= lateral_limit:
                return False, "%s rear=%.1f lat=%.1f" % (
                    class_name, max(0.0, rear_distance), lateral)
        return True, "clear"

    def _build_backup_route(self, ex, ey, eh):
        """Freeze the recent driven trace, then follow it backwards."""
        target_length = getattr(self, "backup_recovery_distance_m", 5.0) + 3.0
        route = [(ex, ey)]
        travelled = 0.0
        for point in reversed(list(getattr(self, "ego_history", []))):
            segment = math.hypot(
                point[0] - route[-1][0], point[1] - route[-1][1])
            if segment < 0.15:
                continue
            # A stale/reset discontinuity must never become a reverse segment.
            if segment > 3.0:
                break
            route.append(point)
            travelled += segment
            if travelled >= target_length:
                break
        while travelled < target_length:
            step = min(0.5, target_length - travelled)
            route.append((
                route[-1][0] - step * math.cos(eh),
                route[-1][1] - step * math.sin(eh),
            ))
            travelled += step
        return self._resample(route, 0.5)

    def _start_backup_recovery(self, blocker, ex, ey, eh):
        rear_clear, rear_reason = self._backup_rear_clear(ex, ey, eh)
        if not rear_clear:
            self.backup_wait_cycles = 0
            return None, "AUTO_BACKUP_BLOCKED %s" % rear_reason
        self.mode = "BACKUP"
        self.backup_phase = "REVERSE"
        self.backup_route = self._build_backup_route(ex, ey, eh)
        self.backup_start = (ex, ey)
        self.backup_cycles = 0
        self.backup_drive_wait = 0
        self.backup_motion_started = False
        self.backup_motion_cycles = 0
        self.backup_stall_cycles = 0
        self.backup_travelled_m = 0.0
        self.backup_last_pose = (ex, ey)
        self.backup_fault = None
        self.backup_pending_bypass = None
        self.backup_source = (
            (float(blocker[3]), float(blocker[4]))
            if blocker is not None else None)
        self.backup_wait_cycles = 0
        self.output_gear = "R"
        self.auto_speed_cap = getattr(
            self, "backup_recovery_max_mps", 4.0 / 3.6)
        return list(self.backup_route), (
            "AUTO_BACKUP_REVERSE target=%.1f rear=%s" % (
                getattr(self, "backup_recovery_distance_m", 5.0),
                rear_reason))

    def _backup_forward_bypass_candidate(self, cur, ex, ey, eh):
        """Return the first validated forward bypass available after backing."""
        # Prefer the physically occupied lane after a failed lateral move. At
        # a branching connector the matcher can instead choose the adjacent
        # branch (A177) while the authoritative route and obstacle continue on
        # A215; fall back to the global slice only when the occupied branch has
        # no blocker.
        reference = self._route_to_link(ex, ey, cur)
        blocker = self._object_lead_on_route(
            reference, ex, ey, lateral_limit=self.block_lat_m)
        if blocker is None and self.use_base_route:
            global_reference = self._base_route_slice(ex, ey, eh)
            global_blocker = self._object_lead_on_route(
                global_reference, ex, ey, lateral_limit=self.block_lat_m)
            if global_blocker is not None:
                reference = global_reference
                blocker = global_blocker
        if blocker is None or self._blocker_is_vulnerable(blocker):
            return None

        blocker_link = self._blocker_route_link(reference, blocker) or cur
        surveyed = getattr(
            self, "local_bypass_corridors", {}).get(blocker_link.idx)
        if surveyed is not None:
            route, goal, reason = self._choose_intersection_bypass(
                blocker_link, reference, blocker, ex, ey)
            if route is not None:
                speed = getattr(
                    self, "intersection_candidate_speed_mps", None)
                if speed is None:
                    speed = self.intersection_bypass_max_mps
                return route, goal, blocker, speed, (
                    "surveyed:%s" % reason)
        elif getattr(self, "free_bypass_enable", True):
            route, goal, speed, reason = self._choose_free_bypass(
                cur, reference, blocker, ex, ey)
            if route is not None:
                return route, goal, blocker, speed, "free:%s" % reason

        target, selection_reason = self._choose_avoid_link(
            cur, ex, ey, blocker[0])
        if target is None:
            target, structural_reason = self._choose_structural_avoid_link(
                cur, blocker[0])
            selection_reason = "%s/%s" % (
                selection_reason, structural_reason)
        if target is None:
            return None
        route, goal, speed, reason = self._choose_delayed_lane_change(
            cur, target, blocker, ex, ey, reference)
        if route is None:
            return None
        return route, goal, blocker, speed, (
            "lane=%s %s %s" % (target.lane, selection_reason, reason))

    def _backup_recovery_step(self, cur, ex, ey, eh):
        """Reverse only until a validated forward bypass becomes available."""
        self.backup_cycles += 1
        last_pose = getattr(self, "backup_last_pose", None)
        if last_pose is not None:
            segment = math.hypot(ex - last_pose[0], ey - last_pose[1])
            if 0.0 < segment <= 1.0:
                self.backup_travelled_m = getattr(
                    self, "backup_travelled_m", 0.0) + segment
            elif self.backup_start is not None:
                # Deterministic replay/tests may advance one complete reverse
                # segment per sample. Accept only a bounded jump that lands at
                # the requested distance; larger localization resets remain
                # excluded.
                direct = math.hypot(
                    ex - self.backup_start[0], ey - self.backup_start[1])
                target = getattr(self, "backup_recovery_distance_m", 5.0)
                if target <= direct <= target + 1.0:
                    self.backup_travelled_m = direct
        self.backup_last_pose = (ex, ey)
        travelled = getattr(self, "backup_travelled_m", 0.0)
        if self.backup_phase == "REVERSE":
            rear_clear, rear_reason = self._backup_rear_clear(ex, ey, eh)
            moving = (
                travelled >= 0.05 or
                abs(getattr(self, "ego_signed_v", 0.0)) > 0.10)
            if moving:
                self.backup_motion_started = True
                self.backup_motion_cycles = getattr(
                    self, "backup_motion_cycles", 0) + 1
                self.backup_stall_cycles = 0
            else:
                self.backup_stall_cycles = getattr(
                    self, "backup_stall_cycles", 0) + 1
            finished = travelled >= getattr(
                self, "backup_recovery_distance_m", 5.0)
            timed_out = (
                getattr(self, "backup_motion_cycles", 0) >= getattr(
                    self, "backup_recovery_timeout_cycles", 120))
            start_failed = (
                not getattr(self, "backup_motion_started", False) and
                getattr(self, "backup_stall_cycles", 0) >= getattr(
                    self, "backup_recovery_start_timeout_cycles", 150))
            forward_candidate = None
            if rear_clear and not timed_out and not start_failed:
                forward_candidate = self._backup_forward_bypass_candidate(
                    cur, ex, ey, eh)
            if (forward_candidate is not None or not rear_clear or finished or
                    timed_out or start_failed):
                self.backup_phase = "DRIVE"
                self.backup_drive_wait = 0
                self.output_gear = "D"
                self.auto_speed_cap = 0.0
                self.backup_pending_bypass = forward_candidate
                route = (self._base_route_slice(ex, ey, eh)
                         if self.use_base_route else [])
                if len(route) < 2:
                    route = self._route_to_link(ex, ey, cur)
                reason = (
                    "forward-bypass" if forward_candidate is not None else
                    "rear-blocked " + rear_reason if not rear_clear else
                    "distance" if finished else
                    "motion-timeout" if timed_out else "start-timeout")
                if forward_candidate is None and not finished:
                    self.backup_fault = reason
                return route, (
                    "%s travelled=%.1f reason=%s%s" % (
                        "AUTO_BACKUP_EARLY_SHIFT_D"
                        if forward_candidate is not None else
                        "AUTO_BACKUP_SHIFT_D" if finished else
                        "AUTO_BACKUP_FAULT_SHIFT_D",
                        travelled, reason,
                        (" candidate=%s" % forward_candidate[4])
                        if forward_candidate is not None else ""))
            self.output_gear = "R"
            self.auto_speed_cap = getattr(
                self, "backup_recovery_max_mps", 4.0 / 3.6)
            return list(self.backup_route), (
                "AUTO_BACKUP_REVERSE travelled=%.1f/%.1f rear=%s" % (
                    travelled,
                    getattr(self, "backup_recovery_distance_m", 5.0),
                    rear_reason))

        self.output_gear = "D"
        self.auto_speed_cap = 0.0
        self.backup_drive_wait += 1
        route = (self._base_route_slice(ex, ey, eh)
                 if self.use_base_route else [])
        if len(route) < 2:
            route = self._route_to_link(ex, ey, cur)
        status_fresh = (
            self._receipt_age(self.vehicle_status_receipt) is not None and
            self._receipt_age(self.vehicle_status_receipt) <= 1.0)
        drive_confirmed = (
            status_fresh and self.vehicle_gear == 4 and
            abs(self.ego_signed_v) <= 0.15 and
            self.backup_drive_wait >= 5)
        drive_timeout = self.backup_drive_wait >= getattr(
            self, "backup_recovery_drive_wait_cycles", 60)
        if drive_confirmed or drive_timeout:
            confirmation = "confirmed" if drive_confirmed else "timeout"
            pending_bypass = getattr(
                self, "backup_pending_bypass", None)
            completed_source = self.backup_source
            # A fixed-distance recovery is exhausted once. An early shift is
            # exhausted only after its fresh candidate is actually installed;
            # a transiently lost window may be searched again safely.
            self.backup_completed_source = (
                completed_source if pending_bypass is None else None)
            self.mode = "CRUISE"
            self.backup_phase = None
            self.backup_route = []
            self.backup_start = None
            self.backup_source = None
            self.changing = False
            self.active_target_lane = None
            self.change_dst_idx = None
            self.avoid_hops = 0
            self.clear_cycles = 0
            self.avoid_source = None
            self.local_bypass_route = []
            self.local_bypass_progress = 0
            self.local_bypass_goal = None
            self.local_bypass_source = None
            self.backup_pending_bypass = None
            if drive_confirmed and pending_bypass is not None:
                # Rebuild from the current pose and newest object frame after
                # the gear change. Never accelerate on a stale stored route.
                fresh = self._backup_forward_bypass_candidate(
                    cur, ex, ey, eh)
                if fresh is not None:
                    route, goal, blocker, speed, reason = fresh
                    self.backup_completed_source = completed_source
                    return self._activate_local_bypass(
                        route, goal, blocker, speed,
                        "AUTO_BACKUP_EARLY_BYPASS gear_D=%s travelled=%.1f %s"
                        % (confirmation, travelled, reason))
            if getattr(self, "backup_fault", None):
                return route, (
                    "AUTO_BACKUP_FAULT_HOLD gear_D=%s reason=%s" % (
                        confirmation, self.backup_fault))
            return route, "AUTO_BACKUP_REPLAN gear_D=%s" % confirmation
        return route, "AUTO_BACKUP_SHIFT_D wait=%d" % self.backup_drive_wait

    def _maybe_start_backup_recovery(
            self, cur, route, status, ex, ey, eh):
        """Arm backup only after a static, non-vulnerable blocked standstill."""
        if not getattr(self, "backup_recovery_enable", True):
            self.backup_wait_cycles = 0
            return None
        eligible_status = (
            status.startswith("AUTO_WAIT") or
            status.startswith("AUTO_AVOID_KEEP") or
            status.startswith("AUTO_AVOID_HOLD") or
            status.startswith("AUTO_REJOIN_GAP"))
        stopped_by_planner = (
            self.auto_speed_cap is not None and self.auto_speed_cap <= 0.05)
        if (not eligible_status or not stopped_by_planner or
                abs(self.ego_signed_v) > 0.15):
            self.backup_wait_cycles = 0
            return None
        blocker = self._object_lead_on_route(
            route, ex, ey, min_s=1.0,
            max_s=getattr(self, "backup_recovery_trigger_m", 9.0))
        if (blocker is None or self._blocker_is_vulnerable(blocker) or
                abs(blocker[2]) > self.intersection_bypass_static_mps):
            self.backup_wait_cycles = 0
            return None
        completed = getattr(self, "backup_completed_source", None)
        if (completed is not None and math.hypot(
                blocker[3] - completed[0],
                blocker[4] - completed[1]) <= getattr(
                    self, "backup_recovery_rearm_distance_m", 3.0)):
            self.backup_wait_cycles = 0
            return route, (
                "AUTO_BACKUP_EXHAUSTED object_s=%.1f retry=blocked" %
                blocker[0])
        self.backup_wait_cycles += 1
        needed = getattr(self, "backup_recovery_wait_cycles", 15)
        if self.backup_wait_cycles < needed:
            return route, (
                "AUTO_BACKUP_ARMING %d/%d object_s=%.1f" % (
                    self.backup_wait_cycles, needed, blocker[0]))
        return self._start_backup_recovery(blocker, ex, ey, eh)

    def _policy(self):
        return getattr(self, "_envelope_policy", None) or envelope_policy(
            float(getattr(self, "controller_feed_timeout_s", ENVELOPE_T_FEED)))

    def _certify(self, route, cap):
        """Same-tick validation certificate for an exact route and cap,
        bound to this tick's frame and the execution-envelope policy. The
        envelope covers every speed <= cap (see ArrivalEnvelope), so a lower
        emitted command is covered by construction."""
        certificates = getattr(self, "_certificates", None)
        if certificates is None:
            certificates = self._certificates = []
        certificates.append((tuple(route), float(cap),
                             getattr(self, "_tick_frame", None), self._policy()))

    def _certified_cap(self, route):
        key = tuple(route)
        frame = getattr(self, "_tick_frame", None)
        caps = [entry[1] for entry in getattr(self, "_certificates", None) or []
                if len(entry) >= 4 and entry[0] == key and
                entry[2] == frame and entry[3] == self._policy()]   # revision2: no legacy 2-tuples
        return max(caps) if caps else None

    def _auto_route(self, cur, ex, ey, eh):
        self.auto_speed_cap = None
        self._tick_ego = (ex, ey)
        self._tick_cur = cur
        self._tick_reference_route = None
        self._tick_work_remaining = SWEEP_TICK_BUDGET
        self._tick_work_used = 0
        contraflow_link = None
        cruise_obstacle_override = False
        source_return_active = False
        source_return_status = None
        if self.mode != "CRUISE" or not self.use_base_route:
            # A source-reference return episode exists only between an
            # occupied-source WAIT and its validated merge back to global.
            self.source_return = None
        if (getattr(self, "contraflow_active", False) and
                self.contraflow_link_idx in self.links):
            contraflow_link = self.links[self.contraflow_link_idx]
            published_route = self._route_to_contraflow(
                ex, ey, contraflow_link, self.change_blend_m)
            blocker = self._object_lead_on_route(published_route, ex, ey)
        elif self.mode == "CRUISE" and self.use_base_route:
            published_route = self._base_route_slice(ex, ey, eh)
            blocker = self._object_lead_on_route(published_route, ex, ey)
            # In CRUISE the global path is authoritative. Do not slow or change
            # lanes for a vehicle that exists only on a nearby HD-map branch.
            # Exception: the ego can start one lane beside the global path.
            # A blind GLOBAL_REJOIN from there crossed the K5 before the Niro
            # was considered. Plan from the physically occupied lane whenever
            # it contains a confirmed blocker and remains adjacent to global.
            source_route = self._route_to_link(ex, ey, cur)
            source_blocker = self._object_lead_on_link(cur, ex, ey)
            global_distance = getattr(self, "base_match_distance", None)
            episode = getattr(self, "source_return", None)
            committed = episode is not None and episode.get("state") == "committed"
            if (not committed and source_blocker is not None and
                    global_distance is not None and
                    math.isfinite(global_distance) and
                    global_distance <= 4.5):
                published_route = source_route
                blocker = source_blocker
                cruise_obstacle_override = True
            elif episode is not None and (
                    committed or self._source_return_blocker(
                        episode, cur, ex, ey, source_blocker) is None):
                # An armed episode acts only once its own source blocker is
                # gone; while it persists (global beyond the override gate)
                # the original planner behaviour is unchanged.
                (return_route, source_return_status, return_cap,
                 return_veto) = self._source_return_step(
                     cur, ex, ey, eh, source_route, published_route)
                if return_route is not None and len(return_route) >= 2:
                    published_route = return_route
                    blocker = self._object_lead_on_route(
                        published_route, ex, ey)
                    source_return_active = True
                if return_cap is not None:
                    self.auto_speed_cap = (
                        return_cap if self.auto_speed_cap is None
                        else min(self.auto_speed_cap, return_cap))
                if return_veto or (return_cap is not None and
                                   not source_return_active):
                    # Hard veto (stale input, deviation, lost source, no
                    # reference): stop on the retained geometry. No later
                    # branch may raise this cap or commit avoidance state.
                    self.auto_speed_cap = 0.0
                    return published_route, source_return_status + " VETO"
        elif self.mode == "REJOIN" and self.use_base_route:
            published_route = self._base_rejoin_route(ex, ey, eh)
            blocker = self._object_lead_on_route(published_route, ex, ey)
        elif (self.mode == "LOCAL_BYPASS" and
              getattr(self, "local_bypass_route", None)):
            # This route has already passed dense OBB and kinematic checks.
            # Crop its travelled prefix without re-blending or smoothing it;
            # geometry changes after validation can restore a collision.
            previous = max(0, int(getattr(
                self, "local_bypass_progress", 0)))
            search_start = max(0, previous - 4)
            search_end = min(
                len(self.local_bypass_route), previous + 160)
            nearest = min(
                range(search_start, search_end),
                key=lambda index: (
                    (self.local_bypass_route[index][0] - ex) ** 2 +
                    (self.local_bypass_route[index][1] - ey) ** 2))
            if 0 < nearest < len(self.local_bypass_route) - 1:
                # gap passage: keep the segment that contains the ego. A crop starting AHEAD of the ego (nearest
                # point up to half a spacing ahead) made the envelope read that longitudinal lag as a lateral
                # offset d0 and inflate its margin, stopping narrow passages that have exact slack.
                ax_, ay_ = self.local_bypass_route[nearest]
                bx_, by_ = self.local_bypass_route[nearest + 1]
                if (ex - ax_) * (bx_ - ax_) + (ey - ay_) * (by_ - ay_) < 0.0:
                    nearest -= 1
            self.local_bypass_progress = max(previous, nearest)
            nearest = self.local_bypass_progress
            published_route = self.local_bypass_route[nearest:]
            if len(published_route) < 2:
                published_route = self.local_bypass_route[-2:]
            if isinstance(self.local_bypass_route, FinalRoute):
                published_route = FinalRoute(published_route)
            # The bypass was already checked against the actual vehicle and
            # object widths.  Reusing the normal 3 m lane blocker band here
            # classified the intentionally close low box as a fresh blocker,
            # set the speed cap back to zero, and stranded the vehicle before
            # it could develop lateral motion.  Zero selects the geometry
            # limit inside _object_lead_on_route; vulnerable users retain
            # their wider stop-only band there.
            blocker = self._object_lead_on_route(
                published_route, ex, ey,
                lateral_limit=0.0)
        else:
            published_route = self._route_to_link(ex, ey, cur)
            blocker = self._object_lead_on_link(cur, ex, ey)
        # This tick's reference (fallback for a rejected bypass finalization).
        self._tick_reference_route = published_route
        if blocker is not None:
            blocker_cap = self._blocker_speed_cap(blocker)
            self.auto_speed_cap = (
                blocker_cap if self.auto_speed_cap is None
                else min(self.auto_speed_cap, blocker_cap))

        if contraflow_link is not None:
            corridor_clear, corridor_reason = self._opposite_corridor_clear(
                contraflow_link, ex, ey)
            if not corridor_clear:
                self.auto_speed_cap = 0.0
                return published_route, (
                    "AUTO_OPPOSITE_HOLD link=%s reason=%s"
                    % (contraflow_link.idx, corridor_reason))
            source = self.links.get(self.contraflow_source_idx)
            source_blocker = (
                self._object_lead_on_link(
                    source, ex, ey, min_s=-6.0, max_s=self.detect_dist_m)
                if source is not None else blocker)
            if source_blocker is None:
                self.clear_cycles += 1
            else:
                self.clear_cycles = 0
            if self.clear_cycles >= self.clear_cycles_needed:
                self.contraflow_active = False
                self.contraflow_link_idx = None
                self.contraflow_source_idx = None
                self.opposite_wait_cycles = 0
                self._start_global_rejoin(ex, ey, eh)
                route = self._base_rejoin_route(ex, ey, eh)
                self.auto_speed_cap = min(
                    self.global_rejoin_max_mps, self.opposite_max_mps)
                return route, "AUTO_OPPOSITE_REJOIN_GLOBAL"
            self.auto_speed_cap = (
                self.opposite_max_mps if self.auto_speed_cap is None
                else min(self.auto_speed_cap, self.opposite_max_mps))
            return published_route, (
                "AUTO_OPPOSITE_LAST_RESORT link=%s clear=%d/%d"
                % (contraflow_link.idx, self.clear_cycles,
                   self.clear_cycles_needed))

        if self.mode == "LOCAL_BYPASS":
            gap_failure_reason = None
            # A newly appearing adjacent-lane object can block the selected
            # offset path even though the original global lane is still clear.
            # Do not remain stopped on that obsolete bypass indefinitely.  If
            # ego is still inside the global corridor and a freshly built
            # return path is collision-free, abandon the bypass and recover on
            # the original route.  The original obstacle will trigger a new
            # avoidance decision if perception confirms it again.
            if (blocker is not None and self.use_base_route and
                    self.base_match_distance is not None and
                    self.base_match_distance <= self.global_rejoin_lat_m):
                return_route = self._base_rejoin_route(ex, ey, eh)
                return_blocker = self._object_lead_on_route(
                    return_route, ex, ey,
                    lateral_limit=self.block_lat_m)
                if len(return_route) >= 2 and return_blocker is None:
                    blocked_lateral = blocker[1]
                    bypass_cap = getattr(
                        self, "local_bypass_max_mps",
                        getattr(self, "intersection_bypass_max_mps",
                                15.0 / 3.6))
                    self._start_global_rejoin(ex, ey, eh)
                    return_route = self._base_rejoin_route(ex, ey, eh)
                    self.auto_speed_cap = min(
                        bypass_cap,
                        getattr(self, "global_rejoin_recovery_mps",
                                5.0 / 3.6))
                    return return_route, (
                        "AUTO_LOCAL_BYPASS_ABORT_CLEAR_GLOBAL "
                        "blocked_lat=%.2f global_dist=%.2f"
                        % (blocked_lateral, self.base_match_distance))
            if blocker is not None:
                gap_route, gap_goal, gap_reason = (
                    self._replan_local_bypass_gap(
                        published_route, blocker, ex, ey, cur))
                if gap_route is not None:
                    # GAP_REPLAN (ego <= 1 m/s) joins only while moving; at standstill the replan installs directly
                    retained = self._handover_retained() if self._handover_moving() else None
                    kind, joins = ("aligned", [gap_route]) if retained is None else \
                        self._handover_join(retained, gap_route, ex, ey)
                    if kind == "impossible" and self._handover_moving():
                        return self._handover_hold(retained, "gap-replan:%s" % joins,
                                                   "AUTO_LOCAL_BYPASS_GAP_REPLAN %s" % gap_reason)
                    if kind == "impossible":
                        kind, joins = "aligned", [gap_route]           # standstill: direct replacement
                    if kind == "join":
                        gap_final, gap_cap, blend, gap_final_reason = self._handover_certify(
                            retained, joins, self.local_bypass_gap_max_mps, gap_goal, ex, ey)
                        if gap_final is not None:
                            gap_reason += " handover=blend%.0f" % blend
                    else:
                        gap_final, gap_final_reason, gap_cap = self._finalize_local_bypass(
                            joins[0], self.local_bypass_gap_max_mps, gap_goal)
                    if gap_final is None and kind == "join" and self._handover_moving():
                        return self._handover_hold(retained, "gap-replan:%s" % gap_final_reason,
                                                   "AUTO_LOCAL_BYPASS_GAP_REPLAN %s" % gap_reason)
                    if gap_final is None and kind == "join":           # standstill: direct replacement
                        gap_final, gap_final_reason, gap_cap = self._finalize_local_bypass(
                            gap_route, self.local_bypass_gap_max_mps, gap_goal)
                    if gap_final is None:
                        gap_route = None
                        gap_reason = "final-reject:%s" % gap_final_reason
                if gap_route is not None:
                    gap_route = gap_final
                    self.local_bypass_route = gap_final
                    self.local_bypass_progress = 0
                    self.local_bypass_goal = gap_goal
                    self.local_bypass_source = (blocker[3], blocker[4])
                    self.local_bypass_replan_count = getattr(
                        self, "local_bypass_replan_count", 0) + 1
                    self.path_revision = getattr(
                        self, "path_revision", 0) + 1
                    self.last_local_decision = gap_reason
                    self.local_bypass_max_mps = min(
                        getattr(self, "local_bypass_max_mps",
                                self.local_bypass_gap_max_mps),
                        self.local_bypass_gap_max_mps, gap_cap)
                    self.auto_speed_cap = self.local_bypass_max_mps
                    return gap_route, (
                        "AUTO_LOCAL_BYPASS_GAP_REPLAN %s count=%d"
                        % (gap_reason, self.local_bypass_replan_count))
                gap_failure_reason = gap_reason
            goal = getattr(self, "local_bypass_goal", None)
            if goal is None:
                self._start_global_rejoin(ex, ey, eh)
                return self._base_rejoin_route(ex, ey, eh), (
                    "AUTO_LOCAL_BYPASS_RECOVER missing-goal")
            goal_forward = (
                math.cos(eh) * (goal[0] - ex) +
                math.sin(eh) * (goal[1] - ey))
            if math.hypot(goal[0] - ex, goal[1] - ey) <= 2.0 or goal_forward < -1.0:
                rejoin_cap = min(
                    self.global_rejoin_max_mps,
                    getattr(self, "local_bypass_max_mps",
                            getattr(self, "intersection_bypass_max_mps",
                                    15.0 / 3.6)))
                retained = self._handover_retained() if self._handover_moving() else None
                if retained is not None:
                    # handover revision: build the replacement WITHOUT mutating state, join it to the retained
                    # reference, validate, then commit atomically (or keep LOCAL_BYPASS on the retained reference)
                    candidate = self._build_base_rejoin_route(ex, ey, eh)
                    kind, joins = self._handover_join(retained, candidate, ex, ey) if len(candidate) >= 2 else \
                        ("impossible", "no-rejoin-candidate")
                    if kind == "join" and getattr(self, "local_bypass_rejoin_handovers", 0) >= 2:
                        kind, joins = "impossible", "rejoin-handover-limit"
                    if kind == "join":
                        # revision2 (H1): the joined rejoin becomes the LOCAL_BYPASS FinalRoute (published verbatim,
                        # cropped as FinalRoute, re-validated + certified every KEEP tick); goal 8 m past the join
                        # and the rejoin merge, where the route lies on the global route
                        v_now = max(0.0, float(getattr(self, "ego_v", 0.0) or 0.0))
                        merge = max(6.0, float(getattr(self, "transition_m", 6.0)))
                        why = "no-join-certified"
                        for joined, blend in joins:
                            reach = 5.0 + max(1.0, v_now) + 1.0 + blend + merge + 8.0
                            gframe = self._route_frame_at_distance(joined, reach)
                            goal_j = gframe[0] if gframe is not None else tuple(joined[-1])
                            final_j, cap_j, blend_j, why = self._handover_certify(
                                retained, [(joined, blend)], rejoin_cap, goal_j, ex, ey)
                            if final_j is None:
                                continue
                            self.local_bypass_route = final_j
                            self.local_bypass_progress = 0
                            self.local_bypass_goal = goal_j
                            self.local_bypass_max_mps = min(rejoin_cap, cap_j)
                            self.local_bypass_rejoin_handovers = getattr(self, "local_bypass_rejoin_handovers", 0) + 1
                            self.path_revision = getattr(self, "path_revision", 0) + 1
                            self.auto_speed_cap = self.local_bypass_max_mps
                            return final_j, "AUTO_LOCAL_BYPASS_REJOIN_HANDOVER handover=blend%.0f count=%d" % (
                                blend_j, self.local_bypass_rejoin_handovers)
                        kind, joins = "impossible", why
                    if kind == "impossible" and self._handover_moving():
                        return self._handover_hold(retained, "rejoin:%s" % joins,
                                                   "AUTO_LOCAL_BYPASS_REJOIN_GLOBAL")
                self._start_global_rejoin(ex, ey, eh)
                route = self._base_rejoin_route(ex, ey, eh)
                self.auto_speed_cap = rejoin_cap
                return route, "AUTO_LOCAL_BYPASS_REJOIN_GLOBAL"
            bypass_cap = getattr(
                self, "local_bypass_max_mps",
                getattr(self, "intersection_bypass_max_mps", 15.0 / 3.6))
            self.auto_speed_cap = (
                bypass_cap
                if self.auto_speed_cap is None else
                min(self.auto_speed_cap, bypass_cap))
            if self.auto_speed_cap is not None and self.auto_speed_cap > 0.0:
                keep_ok, keep_reason, keep_cap, _checked = (
                    self._local_bypass_route_check(
                        published_route, self.auto_speed_cap, goal))
                if not keep_ok:
                    # Stop on the retained geometry; returning here means no
                    # later branch can raise this cap.
                    self.auto_speed_cap = 0.0
                    return published_route, (
                        "AUTO_LOCAL_BYPASS_HOLD reason=%s goal=%.1f" % (
                            keep_reason, math.hypot(goal[0] - ex, goal[1] - ey)))
                self.auto_speed_cap = min(self.auto_speed_cap, keep_cap)
                self._certify(published_route, self.auto_speed_cap)
            status = "AUTO_LOCAL_BYPASS_KEEP goal=%.1f" % math.hypot(
                goal[0] - ex, goal[1] - ey)
            if blocker is not None:
                status += " blocked_s=%.1f gap=%s" % (
                    blocker[0], gap_failure_reason or "not-attempted")
            return published_route, status

        if self.mode == "CRUISE":
            self.home_lane = cur.lane
            # The global route is authoritative. Never start another lateral
            # avoidance while already outside its recovery corridor; that can
            # select an HD-map branch with no controllable return. The log on
            # 2026-08-16 showed AUTO_AVOID at 9.19 m global error (limit 8 m),
            # followed by a return only after the vehicle happened to come
            # back within 3.60 m.
            if (self.use_base_route and
                    not cruise_obstacle_override and
                    not source_return_active and
                    self.base_match_distance is not None and
                    self.base_match_distance > self.global_rejoin_trigger_m):
                distance = self.base_match_distance
                if distance > getattr(
                        self, "global_rejoin_hard_stop_dist_m", 20.0):
                    self._start_global_rejoin(ex, ey, eh)
                    self.auto_speed_cap = 0.0
                    return self._route_to_link(ex, ey, cur), (
                        "AUTO_REJOIN_BLOCKED idx=%s dist=%.2f hard=%.2f road=%s lane=%s"
                        % (self.base_progress, distance,
                           getattr(self, "global_rejoin_hard_stop_dist_m", 20.0),
                           cur.road, cur.lane)
                    )
                if distance > self.global_rejoin_max_dist_m:
                    self._start_global_rejoin(ex, ey, eh)
                    route = self._base_rejoin_route(ex, ey, eh)
                    self.auto_speed_cap = min(
                        self.global_rejoin_max_mps,
                        getattr(self, "global_rejoin_recovery_mps", 5.0 / 3.6))
                    return (route if route else self._route_to_link(ex, ey, cur)), (
                        "AUTO_REJOIN_RECOVERY idx=%s dist=%.2f road=%s lane=%s"
                        % (self.base_progress, distance, cur.road, cur.lane))
                if self._global_rejoin_requested():
                    self._start_global_rejoin(ex, ey, eh)
                    route = self._base_rejoin_route(ex, ey, eh)
                    self.auto_speed_cap = self.global_rejoin_max_mps
                    return (route if route else self._route_to_link(ex, ey, cur)), (
                        "AUTO_REJOIN_GLOBAL idx=%s dist=%.2f heading=%.1f road=%s lane=%s"
                        % (self.base_progress, distance,
                           math.degrees(self.base_match_heading_error)
                           if self.base_match_heading_error is not None else float("inf"),
                           cur.road, cur.lane)
                    )
                self.auto_speed_cap = self.global_rejoin_max_mps
                base = self._base_route_slice(ex, ey, eh)
                return (base if base else self._route_to_link(ex, ey, cur)), (
                    "AUTO_REJOIN_PENDING idx=%s dist=%.2f road=%s lane=%s"
                    % (self.base_progress, distance, cur.road, cur.lane)
                )
            if blocker is not None:
                if self._blocker_is_vulnerable(blocker):
                    self.opposite_wait_cycles = 0
                    base = (published_route if len(published_route) >= 2 else
                            self._route_to_link(ex, ey, cur))
                    return base, (
                        "AUTO_VULNERABLE_BRAKE class=%s object_s=%.1f"
                        % (blocker[5], blocker[0]))
                blocker_link = self._blocker_route_link(
                    published_route, blocker) or cur
                surveyed = getattr(
                    self, "local_bypass_corridors", {}).get(blocker_link.idx)
                if surveyed is not None:
                    bypass, bypass_goal, bypass_reason = (
                        self._choose_intersection_bypass(
                            blocker_link, published_route, blocker, ex, ey))
                    if bypass is not None:
                        bypass_speed = getattr(
                            self, "intersection_candidate_speed_mps", None)
                        if bypass_speed is None:
                            bypass_speed = self.intersection_bypass_max_mps
                        rospy.logwarn(
                            "[LaneChange] surveyed intersection bypass: %s, "
                            "object %.1fm ahead", bypass_reason, blocker[0])
                        return self._activate_local_bypass(
                            bypass, bypass_goal, blocker, bypass_speed,
                            "AUTO_LOCAL_BYPASS_ENTER road=%s link=%s "
                            "object_s=%.1f %s"
                            % (blocker_link.road, blocker_link.idx,
                               blocker[0], bypass_reason))
                if surveyed is None:
                    free_route, free_goal, free_speed, free_reason = (
                        self._choose_free_bypass(
                            cur, published_route, blocker, ex, ey))
                else:
                    # Never replace a failed surveyed candidate with the
                    # generic road-space heuristic on the same connector.
                    free_route = free_goal = free_speed = None
                    free_reason = "surveyed-corridor-invalid:%s" % bypass_reason
                if free_route is not None:
                    rospy.logwarn(
                        "[LaneChange] free road-space bypass: %s, "
                        "object %.1fm ahead", free_reason, blocker[0])
                    return self._activate_local_bypass(
                        free_route, free_goal, blocker,
                        free_speed,
                        "AUTO_FREE_BYPASS_ENTER road=%s link=%s "
                        "object_s=%.1f %s"
                        % (cur.road, cur.idx, blocker[0], free_reason))
                dst, reason = self._choose_avoid_link(cur, ex, ey, blocker[0])
                if dst is None:
                    structural, structural_reason = (
                        self._choose_structural_avoid_link(
                            cur, blocker[0]))
                    if structural is not None:
                        dst = structural
                        reason = "%s/%s" % (reason, structural_reason)
                if dst is not None:
                    (route, goal, speed_cap,
                     preflight_reason) = self._choose_delayed_lane_change(
                         cur, dst, blocker, ex, ey, published_route)
                    if route is not None:
                        self.opposite_wait_cycles = 0
                        self._remember_avoid_source(blocker)
                        rospy.loginfo(
                            "[LaneChange] validated avoid: lane %d -> %d (%s), object %.1fm ahead, %s",
                            cur.lane, dst.lane, reason, blocker[0],
                            preflight_reason)
                        return self._activate_local_bypass(
                            route, goal, blocker, speed_cap,
                            "AUTO_DELAYED_BYPASS_ENTER road=%s lane=%s "
                            "target=%s object_s=%.1f %s"
                            % (cur.road, cur.lane, dst.lane, blocker[0],
                               preflight_reason))
                    reason = "%s/%s" % (reason, preflight_reason)
                bypass, bypass_goal, bypass_reason = (
                    self._choose_intersection_bypass(
                        blocker_link, published_route, blocker, ex, ey))
                if bypass is not None:
                    rospy.logwarn(
                        "[LaneChange] local intersection bypass: %s, "
                        "object %.1fm ahead", bypass_reason, blocker[0])
                    return self._activate_local_bypass(
                        bypass, bypass_goal, blocker,
                        getattr(self, "intersection_candidate_speed_mps", None)
                        or self.intersection_bypass_max_mps,
                        "AUTO_LOCAL_BYPASS_ENTER road=%s link=%s object_s=%.1f %s"
                        % (blocker_link.road, blocker_link.idx,
                           blocker[0], bypass_reason))
                edge_route, edge_goal, edge_speed, edge_reason = (
                    self._choose_marked_edge_bypass(
                        cur, published_route, blocker, ex, ey))
                if edge_route is not None:
                    rospy.logwarn(
                        "[LaneChange] marked-edge low-box bypass: %s, "
                        "object %.1fm ahead", edge_reason, blocker[0])
                    return self._activate_local_bypass(
                        edge_route, edge_goal, blocker, edge_speed,
                        "AUTO_MARKED_EDGE_BYPASS_ENTER road=%s link=%s "
                        "object_s=%.1f %s"
                        % (cur.road, cur.idx, blocker[0], edge_reason))
                self.opposite_wait_cycles = getattr(
                    self, "opposite_wait_cycles", 0) + 1
                opposite, opposite_reason = self._choose_opposite_last_resort(
                    cur, ex, ey)
                if opposite is not None:
                    self.mode = "AVOID"
                    self.changing = False
                    self.contraflow_active = True
                    self.contraflow_link_idx = opposite.idx
                    self.contraflow_source_idx = cur.idx
                    self.change_dst_idx = opposite.idx
                    self.change_blend_m = self._avoid_blend_m(blocker[0])
                    self.clear_cycles = 0
                    self.avoid_hops = 1
                    self._remember_avoid_source(blocker)
                    self.auto_speed_cap = (
                        self.opposite_max_mps if self.auto_speed_cap is None
                        else min(self.auto_speed_cap, self.opposite_max_mps))
                    rospy.logwarn(
                        "[LaneChange] opposite-direction LAST RESORT: %s -> %s (%s)",
                        cur.idx, opposite.idx, opposite_reason)
                    return self._route_to_contraflow(
                        ex, ey, opposite, self.change_blend_m), (
                        "AUTO_OPPOSITE_ENTER source=%s target=%s reason=%s"
                        % (cur.idx, opposite.idx, opposite_reason))
                if ((cruise_obstacle_override or source_return_active) and
                        len(published_route) >= 2):
                    # This tick deliberately planned from the occupied source
                    # lane for its confirmed blocker. Keep that same route
                    # while waiting; swapping in the global slice here moved
                    # the reference by a lane width at an unchanged ego pose
                    # (session60: 3.376 m). Speed caps are unaffected.
                    wait_route = published_route
                    if cruise_obstacle_override:
                        self._source_return_arm(cur, wait_route)
                else:
                    base = (self._base_route_slice(ex, ey, eh)
                            if self.use_base_route else [])
                    wait_route = (base if base else
                                  self._route_to_link(ex, ey, cur))
                return wait_route, (
                    "AUTO_WAIT road=%s lane=%s reason=%s local=%s edge=%s "
                    "opposite=%s object_s=%.1f"
                    % (cur.road, cur.lane, reason, bypass_reason,
                       edge_reason, opposite_reason, blocker[0])
                )
            self.opposite_wait_cycles = 0
            if source_return_active:
                self.global_rejoin_trigger_cycles = 0
                return published_route, source_return_status
            if self.use_base_route and self._global_rejoin_requested():
                self._start_global_rejoin(ex, ey, eh)
                route = self._base_rejoin_route(ex, ey, eh)
                self.auto_speed_cap = self.global_rejoin_max_mps
                return (route if route else self._route_to_link(ex, ey, cur)), (
                    "AUTO_REJOIN_GLOBAL idx=%s dist=%.2f heading=%.1f road=%s lane=%s"
                    % (self.base_progress, self.base_match_distance,
                       math.degrees(self.base_match_heading_error)
                       if self.base_match_heading_error is not None else float("inf"),
                       cur.road, cur.lane)
                )
            if (self.base_match_distance is None or
                    self.base_match_distance <= self.global_rejoin_trigger_m):
                self.global_rejoin_trigger_cycles = 0
            self.avoid_hops = 0
            base = self._base_route_slice(ex, ey, eh) if self.use_base_route else []
            return (base if base else self._route_to_link(ex, ey, cur)), (
                "AUTO_CRUISE global_idx=%s road=%s lane=%s link=%s"
                % (self.base_progress, cur.road, cur.lane, cur.idx)
            )

        if self.mode == "AVOID":
            # A link-centre lane change can still point through a blocker in
            # the adjacent lane. Switch to a continuous road-space path while
            # there is enough distance, then merge back to the global route.
            if blocker is not None and not self._blocker_is_vulnerable(blocker):
                base = self._base_route_slice(ex, ey, eh) if self.use_base_route else []
                free_route, free_goal, free_speed, free_reason = self._choose_free_bypass(
                    cur, base, blocker, ex, ey)
                if free_route is not None:
                    rospy.logwarn(
                        "[LaneChange] mapped avoid replaced by free bypass: "
                        "%s, object %.1fm ahead", free_reason, blocker[0])
                    return self._activate_local_bypass(
                        free_route, free_goal, blocker,
                        free_speed,
                        "AUTO_FREE_BYPASS_SWITCH road=%s link=%s "
                        "object_s=%.1f %s"
                        % (cur.road, cur.idx, blocker[0], free_reason))
            if self.changing and self.change_dst_idx in self.links:
                dst = self.links[self.change_dst_idx]
                detected = self._target_lane_reached(cur, ex, ey, eh)
                if detected is not None:
                    self.changing = False
                    cur = detected
                    rospy.loginfo("[LaneChange] AUTO avoid lane reached: %d", detected.lane)
                else:
                    if blocker is not None:
                        self.change_blend_m = min(
                            self.change_blend_m,
                            self._avoid_blend_m(blocker[0], published_route))
                    target_clear, target_reason, _target_score = self._target_lane_gap_ok(
                        dst, ex, ey)
                    if target_clear:
                        if blocker is None:
                            route = self._safe_lane_change_route(
                                blocker, dst, ex, ey,
                                self.change_blend_m)
                        else:
                            (route, goal, speed_cap,
                             preflight_reason) = (
                                self._choose_delayed_lane_change(
                                    cur, dst, blocker, ex, ey,
                                    published_route))
                            if route is not None:
                                return self._activate_local_bypass(
                                    route, goal, blocker, speed_cap,
                                    "AUTO_DELAYED_BYPASS_SWITCH road=%s "
                                    "lane=%s target=%s object_s=%.1f %s"
                                    % (cur.road, cur.lane, dst.lane,
                                       blocker[0], preflight_reason))
                            # The previous implementation kept AVOID_CHANGE
                            # committed to this invalid path forever.  Cancel
                            # the lateral commit and continue on the source
                            # lane; the CRUISE search retries every tick.
                            self.mode = "CRUISE"
                            self.changing = False
                            self.active_target_lane = None
                            self.change_dst_idx = None
                            self.change_blend_m = self.transition_m
                            self.avoid_hops = 0
                            source_route = self._route_to_link(ex, ey, cur)
                            source_blocker = self._object_lead_on_route(
                                source_route, ex, ey)
                            self.auto_speed_cap = (
                                self._blocker_speed_cap(source_blocker)
                                if source_blocker is not None else
                                getattr(self, "avoid_hold_max_mps",
                                        20.0 / 3.6))
                            self._source_return_arm(cur, source_route)
                            return source_route, (
                                "AUTO_WAIT_AVOID_PREPARE road=%s lane=%s "
                                "target=%s reason=%s"
                                % (cur.road, cur.lane, dst.lane,
                                   preflight_reason))
                    else:
                        # A short-lived target-lane gap rejection must not
                        # command a full stop while the source lane remains
                        # controllable. Recenter smoothly and continue at a
                        # bounded preparation speed. The source blocker cap
                        # still reaches zero at the hard safety distance.
                        hold_cap = getattr(
                            self, "avoid_hold_max_mps", 20.0 / 3.6)
                        if blocker is not None:
                            hold_cap = min(
                                hold_cap, self._blocker_speed_cap(blocker))
                            if blocker[0] <= self.avoid_hard_stop_m:
                                hold_cap = 0.0
                        self.auto_speed_cap = hold_cap
                        return self._route_to_link(ex, ey, cur), (
                            "AUTO_AVOID_HOLD road=%s lane=%s target=%s reason=%s"
                            % (cur.road, cur.lane, dst.lane, target_reason)
                        )
                    object_s = blocker[0] if blocker is not None else -1.0
                    return route, (
                        "AUTO_AVOID_CHANGE road=%s lane=%s target=%s object_s=%.1f blend=%.1f"
                        % (cur.road, cur.lane, dst.lane, object_s, self.change_blend_m)
                    )

            if blocker is not None and self.avoid_hops < self.max_avoid_hops:
                dst, reason = self._choose_avoid_link(cur, ex, ey, blocker[0])
                if dst is not None:
                    (route, goal, speed_cap,
                     preflight_reason) = self._choose_delayed_lane_change(
                         cur, dst, blocker, ex, ey, published_route)
                    if route is not None:
                        self.avoid_hops += 1
                        self._remember_avoid_source(blocker)
                        return self._activate_local_bypass(
                            route, goal, blocker, speed_cap,
                            "AUTO_DELAYED_BYPASS_CHAIN road=%s lane=%s "
                            "target=%s object_s=%.1f %s"
                            % (cur.road, cur.lane, dst.lane, blocker[0],
                               preflight_reason))

            home = self._current_link(ex, ey, eh, want_lane=self.home_lane) if self.home_lane is not None else None
            home_blocker = self._object_lead_on_link(home, ex, ey, min_s=-6.0, max_s=self.detect_dist_m) if home is not None else blocker
            source_passed = self._avoid_source_passed(ex, ey, eh)
            if home_blocker is None and source_passed:
                self.clear_cycles += 1
            else:
                self.clear_cycles = 0

            if self.clear_cycles >= self.clear_cycles_needed and self.use_base_route:
                # End every avoidance episode against the authoritative global
                # route. Lane numbers are local to each road, and equality with
                # home_lane does not prove that the vehicle has rejoined the
                # original route after an intersection or chained avoidance.
                self._start_global_rejoin(ex, ey, eh)
                route = self._base_rejoin_route(ex, ey, eh)
                return (route if route else self._route_to_link(ex, ey, cur)), (
                    "AUTO_REJOIN_GLOBAL idx=%s dist=%.2f heading=%.1f road=%s lane=%s"
                    % (self.base_progress,
                       self.base_match_distance if self.base_match_distance is not None else float("inf"),
                       math.degrees(self.base_match_heading_error)
                       if self.base_match_heading_error is not None else float("inf"),
                       cur.road, cur.lane)
                )

            if self.clear_cycles >= self.clear_cycles_needed and self.home_lane is not None and cur.lane != self.home_lane:
                dst, reason = self._select_next_lane_link(cur, self.home_lane)
                if dst is not None and self._target_lane_clear(dst, ex, ey):
                    self.mode = "RETURN"
                    self.changing = True
                    self.active_target_lane = dst.lane
                    self.change_dst_idx = dst.idx
                    self._prefer_target_lane_speed(dst, ex, ey)
                    rospy.loginfo("[LaneChange] AUTO return: lane %d -> %d", cur.lane, dst.lane)
                    return self._route_to_link(ex, ey, dst, self.transition_m), (
                        "AUTO_RETURN road=%s lane=%s target=%s" % (cur.road, cur.lane, dst.lane)
                    )
                self._prefer_target_lane_speed(cur, ex, ey)
                return self._route_to_link(ex, ey, cur), (
                    "AUTO_RETURN_WAIT road=%s lane=%s reason=%s" % (cur.road, cur.lane, reason)
                )
            self._prefer_target_lane_speed(cur, ex, ey)
            return self._route_to_link(ex, ey, cur), (
                "AUTO_AVOID_KEEP road=%s lane=%s clear=%d source_passed=%s" % (
                    cur.road, cur.lane, self.clear_cycles, source_passed)
            )

        if self.mode == "RETURN":
            # Recover old/in-flight RETURN state safely after entering a new
            # road. The global route is the only stable cross-road reference.
            if self.use_base_route and blocker is None:
                self._start_global_rejoin(ex, ey, eh)

            if self.changing and self.change_dst_idx in self.links:
                dst = self.links[self.change_dst_idx]
                detected = self._target_lane_reached(cur, ex, ey, eh)
                if detected is not None:
                    self.changing = False
                    cur = detected
                    if detected.lane == self.home_lane:
                        self._start_global_rejoin(ex, ey, eh)
                        rospy.loginfo("[LaneChange] AUTO home lane restored; global route rejoin")
                else:
                    target_clear, target_reason, _target_score = self._target_lane_gap_ok(dst, ex, ey)
                    object_s = blocker[0] if blocker is not None else -1.0
                    if target_clear:
                        route = self._safe_lane_change_route(
                            blocker, dst, ex, ey, self.transition_m)
                        return route, (
                            "AUTO_RETURN_CHANGE road=%s lane=%s target=%s object_s=%.1f"
                            % (cur.road, cur.lane, dst.lane, object_s)
                        )

                    if blocker is not None:
                        alt, avoid_reason = self._choose_avoid_link(cur, ex, ey, blocker[0])
                        if alt is not None:
                            self.mode = "AVOID"
                            self.changing = True
                            self.active_target_lane = alt.lane
                            self.change_dst_idx = alt.idx
                            self.change_blend_m = self._avoid_blend_m(
                                blocker[0], published_route)
                            self.avoid_hops = 1
                            self._remember_avoid_source(blocker)
                            self.clear_cycles = 0
                            route = self._safe_lane_change_route(
                                blocker, alt, ex, ey, self.change_blend_m)
                            rospy.loginfo(
                                "[LaneChange] AUTO return interrupted: lane %d -> %d (%s), object %.1fm ahead, return target blocked: %s",
                                cur.lane, alt.lane, avoid_reason, blocker[0], target_reason,
                            )
                            return route, (
                                "AUTO_RETURN_DETOUR_CHANGE road=%s lane=%s target=%s object_s=%.1f blend=%.1f reason=%s"
                                % (cur.road, cur.lane, alt.lane, blocker[0], self.change_blend_m, target_reason)
                            )

                    self.mode = "AVOID"
                    self.changing = False
                    self.active_target_lane = None
                    self.change_dst_idx = None
                    self.change_blend_m = self.transition_m
                    self.avoid_hops = 0
                    self.clear_cycles = 0
                    self._prefer_target_lane_speed(cur, ex, ey)
                    rospy.loginfo(
                        "[LaneChange] AUTO return paused: lane %d keeps current lane, return target %d blocked: %s",
                        cur.lane, dst.lane, target_reason,
                    )
                    return self._route_to_link(ex, ey, cur), (
                        "AUTO_RETURN_HOLD road=%s lane=%s target=%s reason=%s"
                        % (cur.road, cur.lane, dst.lane, target_reason)
                    )

            if self.home_lane is not None and cur.lane != self.home_lane:
                dst, reason = self._select_next_lane_link(cur, self.home_lane)
                if dst is not None and self._target_lane_clear(dst, ex, ey):
                    self.changing = True
                    self.active_target_lane = dst.lane
                    self.change_dst_idx = dst.idx
                    self._prefer_target_lane_speed(dst, ex, ey)
                    rospy.loginfo("[LaneChange] AUTO return step: lane %d -> %d", cur.lane, dst.lane)
                    return self._route_to_link(ex, ey, dst, self.transition_m), (
                        "AUTO_RETURN road=%s lane=%s target=%s" % (cur.road, cur.lane, dst.lane)
                    )
                if blocker is not None:
                    dst, avoid_reason = self._choose_avoid_link(cur, ex, ey, blocker[0])
                    if dst is not None:
                        self.mode = "AVOID"
                        self.changing = True
                        self.active_target_lane = dst.lane
                        self.change_dst_idx = dst.idx
                        self.change_blend_m = self._avoid_blend_m(
                            blocker[0], published_route)
                        self.avoid_hops = 1
                        self._remember_avoid_source(blocker)
                        self.clear_cycles = 0
                        route = self._safe_lane_change_route(
                            blocker, dst, ex, ey, self.change_blend_m)
                        rospy.loginfo(
                            "[LaneChange] AUTO return detour: lane %d -> %d (%s), object %.1fm ahead, blend %.1fm",
                            cur.lane, dst.lane, avoid_reason, blocker[0], self.change_blend_m,
                        )
                        return route, (
                            "AUTO_RETURN_DETOUR road=%s lane=%s target=%s object_s=%.1f blend=%.1f reason=%s"
                            % (cur.road, cur.lane, dst.lane, blocker[0], self.change_blend_m, reason)
                        )
                self._prefer_target_lane_speed(cur, ex, ey)
                return self._route_to_link(ex, ey, cur), (
                    "AUTO_RETURN_WAIT road=%s lane=%s reason=%s" % (cur.road, cur.lane, reason)
                )
            self._start_global_rejoin(ex, ey, eh)

        if self.mode == "REJOIN":
            route = self._base_rejoin_route(ex, ey, eh) if self.use_base_route else []
            if blocker is not None:
                if self._blocker_is_vulnerable(blocker):
                    self.rejoin_stable_cycles = 0
                    return (route if route else self._route_to_link(ex, ey, cur)), (
                        "AUTO_VULNERABLE_BRAKE_REJOIN class=%s object_s=%.1f"
                        % (blocker[5], blocker[0]))
                # A normal static obstacle can be detected while recovering
                # toward the global path (including after localization drift).
                # Build the same early, full-size bypass used in CRUISE before
                # falling back to a near-stop gap nudge.  Without this branch,
                # REJOIN followed the blocked spline until almost stationary.
                reference_route = (
                    route if len(route) >= 2 else published_route)
                free_route, free_goal, free_speed, free_reason = (
                    self._choose_free_bypass(
                        cur, reference_route, blocker, ex, ey))
                if free_route is not None:
                    return self._activate_local_bypass(
                        free_route, free_goal, blocker, free_speed,
                        "AUTO_REJOIN_FREE_BYPASS object_s=%.1f %s"
                        % (blocker[0], free_reason))
                dst, avoid_reason = self._choose_avoid_link(
                    cur, ex, ey, blocker[0])
                if dst is not None:
                    self.mode = "AVOID"
                    self.changing = True
                    self.home_lane = cur.lane
                    self.active_target_lane = dst.lane
                    self.change_dst_idx = dst.idx
                    self.change_blend_m = self._avoid_blend_m(
                        blocker[0], reference_route)
                    self.avoid_hops = 1
                    self._remember_avoid_source(blocker)
                    self.clear_cycles = 0
                    lane_route = self._safe_lane_change_route(
                        blocker, dst, ex, ey, self.change_blend_m)
                    return lane_route, (
                        "AUTO_REJOIN_AVOID road=%s lane=%s target=%s "
                        "object_s=%.1f blend=%.1f reason=%s"
                        % (cur.road, cur.lane, dst.lane, blocker[0],
                           self.change_blend_m, avoid_reason))
                # A bypass can finish beside the global route.  If a close
                # static box blocks the recovery spline, the former behavior
                # only applied a zero speed cap and remained in REJOIN
                # forever.  At low speed, reuse the measured-clearance nudge
                # to pass through an available gap before rejoining again.
                gap_route, gap_goal, gap_reason = self._replan_local_bypass_gap(
                    published_route, blocker, ex, ey, cur)
                if gap_route is not None:
                    return self._activate_local_bypass(
                        gap_route, gap_goal, blocker,
                        self.local_bypass_gap_max_mps,
                        "AUTO_REJOIN_GAP_BYPASS object_s=%.1f %s"
                        % (blocker[0], gap_reason))
            distance = self.base_match_distance if self.base_match_distance is not None else float("inf")
            heading_error = self.base_match_heading_error if self.base_match_heading_error is not None else float("inf")
            heading_speed = self.global_rejoin_max_mps * max(
                0.2, math.cos(min(heading_error, math.pi / 2.0)))
            self.auto_speed_cap = (
                heading_speed if self.auto_speed_cap is None
                else min(self.auto_speed_cap, heading_speed))
            if distance > getattr(
                    self, "global_rejoin_hard_stop_dist_m", 20.0):
                self.auto_speed_cap = 0.0
                self.rejoin_stable_cycles = 0
                return self._route_to_link(ex, ey, cur), (
                    "AUTO_REJOIN_BLOCKED idx=%s dist=%.2f hard=%.2f road=%s lane=%s"
                    % (self.base_progress, distance,
                       getattr(self, "global_rejoin_hard_stop_dist_m", 20.0),
                       cur.road, cur.lane)
                )
            if distance > self.global_rejoin_max_dist_m:
                self.auto_speed_cap = min(
                    self.auto_speed_cap,
                    getattr(self, "global_rejoin_recovery_mps", 5.0 / 3.6))
                self.rejoin_stable_cycles = 0
                return (route if route else self._route_to_link(ex, ey, cur)), (
                    "AUTO_REJOIN_RECOVERY idx=%s dist=%.2f heading=%.1f road=%s lane=%s"
                    % (self.base_progress, distance, math.degrees(heading_error),
                       cur.road, cur.lane))
            aligned = (distance <= self.global_rejoin_lat_m and
                       heading_error <= math.radians(20.0) and
                       self.ego_signed_v >= -0.1)
            if aligned and self.use_base_route:
                # handover revision: while moving, hand over to the base slice only where the retained rejoin
                # reference already coincides with it (<= 0.05 m / 1 deg over the handover window)
                retained = self._handover_retained() if self._handover_moving() else None
                if retained is not None:
                    base_now = self._base_route_slice(ex, ey, eh)
                    if len(base_now) >= 2 and self._handover_join(retained, base_now, ex, ey)[0] != "aligned":
                        aligned = False
            self.rejoin_stable_cycles = self.rejoin_stable_cycles + 1 if aligned else 0
            if self.rejoin_stable_cycles >= self.global_rejoin_cycles_needed:
                self.mode = "CRUISE"
                self.home_lane = None
                self.active_target_lane = None
                self.change_dst_idx = None
                self.change_blend_m = self.transition_m
                self.avoid_hops = 0
                self.rejoin_stable_cycles = 0
                self.rejoin_route = []
                route = self._base_route_slice(ex, ey, eh) if self.use_base_route else route
                return (route if route else self._route_to_link(ex, ey, cur)), (
                    "AUTO_REJOINED_GLOBAL idx=%s road=%s lane=%s"
                    % (self.base_progress, cur.road, cur.lane)
                )
            refresh = ""
            if getattr(self, "rejoin_refresh_last", None) is not None:
                refresh = " refresh=%d last_dev=%.2f last_v=%.1f" % (
                    self.rejoin_refresh_count, self.rejoin_refresh_last[0],
                    self.rejoin_refresh_last[1] * 3.6)
            return (route if route else self._route_to_link(ex, ey, cur)), (
                "AUTO_REJOIN_GLOBAL idx=%s dist=%.2f heading=%.1f road=%s lane=%s%s"
                % (self.base_progress, distance, math.degrees(heading_error), cur.road, cur.lane,
                   refresh)
            )

        self.mode = "CRUISE"
        return self._route_to_link(ex, ey, cur), "AUTO_RESET lane=%s" % cur.lane


    # ------------------------------------------------------------------
    # Source-reference return (Stage 2). After an occupied-source WAIT the
    # published reference is the source lane. When its blocker disappears,
    # merge back to the global route with a continuous path that starts on
    # the source reference (not on the ego pose). Every candidate is built in
    # locals and validated (fresh objects, swept full-body OBB at the executed
    # speed, kinematics, legal HD-map corridor and whole-footprint lane
    # containment) before the episode state changes. Invalid -> hold on the
    # source reference only with a validated cap, else stop; retried each tick.
    # ------------------------------------------------------------------
    def _source_return_stale_reference(self, ex, ey, cur):
        """Hard-stale hold: the reference the episode last published, without
        advancing or validating episode state (perception is not usable)."""
        episode = getattr(self, "source_return", None)
        if episode is None or self.mode != "CRUISE":
            return []
        if episode.get("state") == "committed":
            path = episode["path"]
            window = range(max(0, episode["progress"] - 2),
                           min(len(path), episode["progress"] + 200))
            nearest = min(window, key=lambda i: (path[i][0] - ex) ** 2 +
                          (path[i][1] - ey) ** 2)
            return FinalRoute(path[max(0, nearest - 1):])
        source_link = self.links.get(episode.get("link"))
        if source_link is None:
            return []
        return self._route_to_link(ex, ey, source_link)

    def _source_return_finalize(self, route):
        """Published form of a CRUISE-mode source-return route (the same
        transform _publish applies to CRUISE routes), marked FinalRoute so
        it is validated and published exactly."""
        if len(route) < 2:
            return FinalRoute(route)
        self._spend(1 + len(route) // 4)
        return FinalRoute(self._smooth(self._resample(route, 1.0)))

    @staticmethod
    def _source_return_final_index(raw, raw_index, final):
        """First finalized index whose arc length reaches the raw path's arc
        length at raw_index (merge end mapped onto the finalized path)."""
        target = 0.0
        for a, b in zip(raw[:raw_index], raw[1:raw_index + 1]):
            target += math.hypot(b[0] - a[0], b[1] - a[1])
        travelled = 0.0
        for index in range(1, len(final)):
            travelled += math.hypot(final[index][0] - final[index - 1][0],
                                    final[index][1] - final[index - 1][1])
            if travelled >= target - 1.0e-9:
                return index
        return len(final) - 1

    def _source_return_arm(self, cur, source_route):
        episode = getattr(self, "source_return", None)
        if episode is not None and episode.get("state") == "committed":
            return
        self.source_return = {"state": "armed", "link": cur.idx,
                              "lane": cur.lane, "attempts": 0,
                              "last_reason": "armed"}

    @staticmethod
    def _polyline_length(route):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(route, route[1:]))

    @staticmethod
    def _polyline_headings(route):
        """Central-difference headings that skip (near-)duplicate neighbours;
        resampling can end with a point ~0 m from its predecessor, whose
        atan2 would otherwise read 0 rad and rotate an end-pose footprint."""
        headings, count = [], len(route)
        for index in range(count):
            before, after = index, index
            while before > 0 and math.hypot(
                    route[index][0] - route[before][0],
                    route[index][1] - route[before][1]) < 1.0e-3:
                before -= 1
            while after < count - 1 and math.hypot(
                    route[after][0] - route[index][0],
                    route[after][1] - route[index][1]) < 1.0e-3:
                after += 1
            if before == index and index > 0:
                before -= 1
            if after == index and index < count - 1:
                after += 1
            headings.append(math.atan2(route[after][1] - route[before][1],
                                       route[after][0] - route[before][0]))
        return headings

    @staticmethod
    def _source_return_projector(reference):
        """Exact equivalent of project_to_frenet(px, py, reference) -> (s, d)
        with segment data computed once per reference. Same per-segment
        float operations and strict '<' in index order; a segment is skipped
        only when its midpoint lower bound (|p - mid| - length / 2) exceeds
        the best distance by > 1e-6, so it cannot be strictly closer."""
        if len(reference) < 2:
            raise ValueError("reference needs at least two points")
        segments = []
        travelled = 0.0
        for start, end in zip(reference, reference[1:]):
            dx, dy = end[0] - start[0], end[1] - start[1]
            length = math.hypot(dx, dy)
            if length <= 1.0e-9:
                continue
            segments.append((start[0], start[1], dx, dy, length, travelled,
                             start[0] + 0.5 * dx, start[1] + 0.5 * dy))
            travelled += length
        if not segments:
            raise ValueError("reference contains no non-zero segment")

        def project(px, py):
            best = None
            for segment in segments:
                sx, sy, dx, dy, length, before, mx, my = segment
                if best is not None and (
                        math.hypot(px - mx, py - my) - 0.5 * length >
                        best[0] + 1.0e-6):
                    continue
                fraction = max(0.0, min(
                    1.0, ((px - sx) * dx + (py - sy) * dy) /
                    (length * length)))
                x = sx + fraction * dx
                y = sy + fraction * dy
                distance = math.hypot(px - x, py - y)
                if best is None or distance < best[0]:
                    best = (distance, segment, fraction, x, y)
            _, (sx, sy, dx, dy, length, before, _, _), fraction, x, y = best
            heading = math.atan2(dy, dx)
            return (before + fraction * length,
                    -math.sin(heading) * (px - x) + math.cos(heading) * (py - y))
        return project

    def _source_return_merge_prepare(self, source_route, base_slice, max_length):
        """Geometry shared by merge candidates with merge_length <= max_length.

        Source points are projected onto the global slice in order; only
        strictly increasing stations are kept. Projection stops at the first
        kept station >= max_length + 1 (with >= 3 kept): source_offset() only
        reads stations up to the first one >= its query station (<= L), the
        insufficient-source test (last station < L + 1) is already decided,
        and a projection failure depends on the base alone, so it occurs at
        the first point. Output is identical to projecting every point.
        Pure geometry (no objects, pose or time)."""
        base = resample_polyline(base_slice, 0.5)
        source = resample_polyline(source_route, 0.5)
        if len(base) < 3 or len(source) < 3:
            return {"failure": "short-reference"}
        try:
            project = self._source_return_projector(base)
        except ValueError:
            return {"failure": "source-projection"}
        stations, offsets = [], []
        for point in source:
            station, offset = project(point[0], point[1])
            if stations and station <= stations[-1] + 1.0e-6:
                continue
            stations.append(station)
            offsets.append(offset)
            if len(stations) >= 3 and stations[-1] >= max_length + 1.0:
                break
        if len(stations) < 3 or stations[0] > 1.0:
            return {"failure": "source-not-alongside"}
        arc = [0.0]
        for a, b in zip(base, base[1:]):
            arc.append(arc[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        return {"failure": None, "max_length": max_length, "base": base,
                "stations": stations, "offsets": offsets, "arc": arc,
                "headings": None}

    def _source_return_merge(self, source_route, base_slice, merge_length,
                             prepared=None):
        """Offset blend from the source reference to the global slice.

        d(s) is the source reference's lateral offset from the global slice at
        station s; the merge follows d(s) * (1 - quintic(s / L)), so it starts
        exactly on the source reference with its tangent and reaches the
        global geometry with zero offset, slope and second derivative.
        `prepared` (from _source_return_merge_prepare of the same routes with
        max_length >= merge_length) avoids repeating the projections.
        Returns (path, merge_end_index, d0) or (None, None, reason)."""
        if prepared is None or prepared.get("failure") is None and (
                prepared["max_length"] < merge_length):
            prepared = self._source_return_merge_prepare(
                source_route, base_slice, merge_length)
        if prepared["failure"] is not None:
            return None, None, prepared["failure"]
        base = prepared["base"]
        stations, offsets = prepared["stations"], prepared["offsets"]
        if stations[-1] < merge_length + 1.0:
            return None, None, "insufficient-source-route %.1f<%.1f" % (
                stations[-1], merge_length + 1.0)
        arc = prepared["arc"]
        if arc[-1] < merge_length + 10.0:
            return None, None, "insufficient-global-route"
        if prepared["headings"] is None:
            prepared["headings"] = self._polyline_headings(base)
        headings = prepared["headings"]

        def source_offset(station):
            if station <= stations[0]:
                return offsets[0]
            for index in range(1, len(stations)):
                if station <= stations[index]:
                    t = ((station - stations[index - 1]) /
                         max(1.0e-9, stations[index] - stations[index - 1]))
                    return offsets[index - 1] + t * (
                        offsets[index] - offsets[index - 1])
            return offsets[-1]

        path, merge_end_index = [], None
        for index, point in enumerate(base):
            station = arc[index]
            if station <= merge_length:
                offset = source_offset(station) * (
                    1.0 - quintic_smoothstep(station / merge_length))
            else:
                offset = 0.0
                if merge_end_index is None:
                    merge_end_index = len(path)
            heading = headings[index]
            path.append((point[0] - math.sin(heading) * offset,
                         point[1] + math.cos(heading) * offset))
        return path, merge_end_index, source_offset(0.0)

    def _objects_fresh_for_commit(self):
        receipt = getattr(self, "objects_receipt", None)
        if receipt is None:
            return False, "objects-missing"
        age = self._receipt_age(receipt)
        if age is None:
            return False, "objects-invalid-receipt"        # future (backward clock) or unstarted clock
        if age > self.object_timeout_s:
            return False, "objects-stale age=%.2f" % age
        return True, "fresh"

    @staticmethod
    def _strip_projection(link, px, py, hint, window=40):
        """Signed lateral / distance / segment index of (px, py) on link.pts,
        searching segments within +-window of the hint (full scan if hint
        None). A best segment on a narrowed window's boundary is re-searched
        with the +-40 window."""
        pts = link.pts
        count = len(pts)
        if count < 2:
            return None
        if hint is None:
            candidates = range(count - 1)
        else:
            candidates = range(max(0, hint - window), min(count - 1, hint + window + 1))
        best = None
        for index in candidates:
            ax, ay = pts[index]
            bx, by = pts[index + 1]
            dx, dy = bx - ax, by - ay
            length_sq = dx * dx + dy * dy
            if length_sq <= 1.0e-12:
                continue
            raw = ((px - ax) * dx + (py - ay) * dy) / length_sq
            fraction = max(0.0, min(1.0, raw))
            qx, qy = ax + fraction * dx, ay + fraction * dy
            distance_sq = (px - qx) ** 2 + (py - qy) ** 2
            if best is None or distance_sq < best[0]:
                length = math.sqrt(length_sq)
                lateral = (-(dy / length) * (px - qx) + (dx / length) * (py - qy))
                interior = (0.0 <= raw <= 1.0 or
                            (0 < index and raw < 0.0) or
                            (index < count - 2 and raw > 1.0))
                best = (distance_sq, lateral, index, interior)
        if (window < 40 and hint is not None and best is not None and
                best[2] in (max(0, hint - window), min(count - 2, hint + window))
                and best[2] not in (0, count - 2)):
            return HdmapLaneChange._strip_projection(link, px, py, hint, 40)
        if best is None:
            return None
        return best[1], math.sqrt(best[0]), best[2], best[3]

    def _source_return_legal_corridor(self, cur, route):
        """Legal HD-map corridor + full-footprint coverage by the lane union.

        Crossing legality: the existing explicit can_move/destination-edge gate
        (_route_within_crossable_corridor). Containment: at poses every 1 m
        along the route (plus the end pose) the ego rectangle (rear/front
        extents about the reference point, _ego_body_extents), widened by a
        tracking allowance, is cut into cross-segments every <= 0.5 m of body
        length (front and rear edges included). For every legally connected
        same-direction lane strip (|lateral| <= min half width) the parameter
        interval of each cross-segment inside the strip is computed from the
        lateral at both segment ends (straight-strip approximation over one
        body width); the union of these intervals must cover the whole segment
        (union edges within 0.01 m; a gap between two adjacent legal strips
        up to source_return_strip_gap_m, default 0.30 m, is filled as map
        width-model mismatch; any larger gap fails). Unknown
        topology (no legal lanes / no strip near a pose) fails. This is
        containment of the sampled model (poses every 1 m, sections every
        <= 0.5 m, straight strip per cross-section, 0.2 m allowance), not a
        mathematical proof of continuous-sweep containment; the HD-map widths
        are map data, not surveyed road edges; the pose is the body centre."""
        if not self._route_within_crossable_corridor(cur, route):
            return False, "hdmap-crossable-corridor"
        legal = {}
        reverse = getattr(self, "reverse_adj", None) or {}
        for seed in self._same_direction_lateral_links(cur):
            for link in self._forward(seed, self.route_len):
                if len(link.pts) >= 2:
                    legal[link.idx] = link
            before = [link for link in reverse.get(seed.fr, []) if len(link.pts) >= 2]
            pool = ([link for link in before if link.lane == seed.lane] or
                    [link for link in before if link.road == seed.road])
            if len(seed.pts) >= 2:
                seed_th = math.atan2(seed.pts[1][1] - seed.pts[0][1],
                                     seed.pts[1][0] - seed.pts[0][0])
                for link in pool:
                    th = math.atan2(link.pts[-1][1] - link.pts[-2][1],
                                    link.pts[-1][0] - link.pts[-2][0])
                    if abs(norm(th - seed_th)) <= math.radians(55):
                        legal[link.idx] = link
        if not legal:
            return False, "no-legal-lanes"
        rear_extent, front_extent, width, model = self._ego_body_extents()
        if model == "invalid-config":
            return False, "invalid-ego-config"
        length = rear_extent + front_extent
        allowance = float(getattr(self, "source_return_tracking_allowance_m", 0.20))
        half_w = 0.5 * width + allowance
        reach_l = max(rear_extent, front_extent)
        sections = max(2, int(math.ceil(length / 0.5)))
        # Outer edges of the lane union: 0.01 m (map sampling noise only).
        # Interior gap between two legally connected strips: bounded map
        # width-model mismatch (installed map: up to 0.26 m between the
        # session60 lanes); any larger interior gap rejects.
        edge_tolerance_m = 0.01
        strip_gap_m = float(getattr(self, "source_return_strip_gap_m", 0.30))
        junction_m = 0.5
        continues_to = {}
        continues_from = {}
        for link in legal.values():
            continues_from[link.to] = True      # some legal link ends here
            continues_to[link.fr] = True        # some legal link starts here
        dense = resample_polyline(route, 1.0)
        headings = self._polyline_headings(dense)
        hints = {}
        for (px, py), heading in zip(dense, headings):
            c, s_ = math.cos(heading), math.sin(heading)
            near = []
            for idx, link in legal.items():
                projection = self._strip_projection(link, px, py, hints.get(idx))
                if projection is None:
                    continue
                hints[idx] = projection[2]
                if projection[1] <= 0.5 * min(link.width_start, link.width_end) + reach_l + half_w:
                    near.append(link)
            if not near:
                return False, "footprint-unknown-topology"
            for k in range(sections + 1):
                u = -rear_extent + length * k / sections
                cx, cy = px + u * c, py + u * s_
                ax, ay = cx + half_w * s_, cy - half_w * c      # right edge point
                bx, by = cx - half_w * s_, cy + half_w * c      # left edge point
                cross = 2.0 * half_w
                strips = []
                for link in near:
                    spacing = max(0.05, link.length / max(1, len(link.pts) - 1))
                    hint = min(len(link.pts) - 2, max(0, hints[link.idx] + int(round(u / spacing))))
                    window = max(3, min(40, int(math.ceil((half_w + 0.5) / spacing)) + 2))
                    pa = self._strip_projection(link, ax, ay, hint, window)
                    pb = self._strip_projection(link, bx, by, hint, window)
                    if pa is None or pb is None:
                        continue
                    t_from, t_to = 0.0, 1.0
                    if not (pa[3] and pb[3]):
                        # An end point lies beyond this link's first/last
                        # point. Continue the strip at most junction_m past a
                        # node where the legal set continues; keep only the
                        # part of the cross-section inside that extent.
                        outside = pb if pa[3] else pa
                        at_end = outside[2] >= len(link.pts) - 2
                        if at_end:
                            node_ok = continues_to.get(link.to, False)
                            ox, oy = link.pts[-1]
                            tx, ty = (link.pts[-1][0] - link.pts[-2][0],
                                      link.pts[-1][1] - link.pts[-2][1])
                        else:
                            node_ok = continues_from.get(link.fr, False)
                            ox, oy = link.pts[0]
                            tx, ty = (link.pts[0][0] - link.pts[1][0],
                                      link.pts[0][1] - link.pts[1][1])
                        norm_t = math.hypot(tx, ty)
                        if not node_ok or norm_t < 1.0e-9:
                            continue
                        tx, ty = tx / norm_t, ty / norm_t
                        along_a = (ax - ox) * tx + (ay - oy) * ty
                        along_b = (bx - ox) * tx + (by - oy) * ty
                        inside_a = along_a <= junction_m
                        inside_b = along_b <= junction_m
                        if not (inside_a or inside_b):
                            continue
                        if not (inside_a and inside_b):
                            split = min(1.0, max(0.0, (junction_m - along_a) /
                                                 (along_b - along_a)))
                            t_from, t_to = ((0.0, split) if inside_a else (split, 1.0))
                    strips.append((link, pa[0], pb[0], 0.5 * min(link.width_start, link.width_end),
                                   t_from, t_to))
                intervals = []
                for index, (link, la, lb, hw, t_from, t_to) in enumerate(strips):
                    # Fill a bounded gap to an adjacent legal strip: centre of
                    # the other strip in this strip's lateral frame is
                    # mid - mid_other (same-direction lanes).
                    grow_left = grow_right = 0.0
                    mid = 0.5 * (la + lb)
                    for other_index, (_, oa, ob, ohw, _, _) in enumerate(strips):
                        if other_index == index:
                            continue
                        offset = mid - 0.5 * (oa + ob)
                        gap = abs(offset) - hw - ohw
                        # 1e-6 m: float noise only (exact bound accepted).
                        if 0.0 < gap <= strip_gap_m + 1.0e-6:
                            if offset > 0.0:
                                grow_left = max(grow_left, 0.5 * gap)
                            else:
                                grow_right = max(grow_right, 0.5 * gap)
                    low, high = -hw - grow_right, hw + grow_left
                    if abs(lb - la) < 1.0e-9:
                        if low <= la <= high:
                            intervals.append((t_from, t_to))
                        continue
                    t1, t2 = (low - la) / (lb - la), (high - la) / (lb - la)
                    lo, hi = max(t_from, min(t1, t2)), min(t_to, max(t1, t2))
                    if hi >= lo:
                        intervals.append((lo, hi))
                intervals.sort()
                tolerance = edge_tolerance_m / cross
                reach = 0.0
                for lo, hi in intervals:
                    if lo > reach + tolerance:
                        break
                    reach = max(reach, hi)
                if reach < 1.0 - tolerance:
                    return False, "footprint-outside-legal-lanes"
        return True, "legal-corridor"

    def _source_return_validate(self, route, cur, ex, ey, horizon_m,
                                speed_request=None):
        """Fresh objects, preflight (direction, swept OBB, kinematics), swept
        clearance re-checked at the executed cap and at half of it, legal
        corridor. Returns (ok, reason, speed_cap)."""
        fresh, fresh_reason = self._objects_fresh_for_commit()
        if not fresh:
            return False, fresh_reason, 0.0
        speed_hint = (min(self.cruise_mps, getattr(cur, "vmax", self.cruise_mps),
                          self.global_rejoin_max_mps)
                      if speed_request is None else float(speed_request))
        need = self._envelope_required_length(max(1.0, speed_hint))
        pose = self._execution_ego_pose(route) if len(route) >= 2 else None
        lead_in = 0.0 if pose is None else math.hypot(
            pose[0] - route[0][0], pose[1] - route[0][1])
        prefix = self._route_prefix_by_length(
            route, max(15.0, min(70.0, horizon_m), (need or 0.0) + lead_in + 2.0))
        if len(prefix) < 3:
            return False, "short-route", 0.0
        speed = (min(self.cruise_mps, getattr(cur, "vmax", self.cruise_mps),
                     self.global_rejoin_max_mps)
                 if speed_request is None else float(speed_request))
        preflight = self._lane_change_preflight(
            prefix, cur, ex, ey, speed, horizon_m=self._polyline_length(prefix))
        if not preflight["feasible"]:
            return False, "preflight:%s" % preflight["reason"], 0.0
        cap = float(preflight["speed_cap"])
        margin = getattr(self, "lane_change_preflight_margin", 0.50)
        for check_speed in sorted({round(cap, 3), round(max(1.0, 0.5 * cap), 3)}):
            obstacles = self._prefilter_obb_obstacles(
                prefix, ex, ey, check_speed, margin)
            clearance = self._ego_swept_clearance(
                prefix, obstacles, check_speed,
                safety_margin=margin, sample_step=0.35)
            if not clearance["valid"]:
                if clearance.get("budget_exhausted"):
                    return False, "work-budget-exhausted", 0.0
                return False, "executed-speed-obb v=%.2f clearance=%.2f" % (
                    check_speed, clearance["minimum_clearance"]), 0.0
        legal, legal_reason = self._source_return_legal_corridor(cur, prefix)
        if not legal:
            return False, legal_reason, 0.0
        if isinstance(route, FinalRoute):
            self._certify(route, cap)
        return True, preflight["reason"], cap

    def _source_return_hold(self, cur, ex, ey, source_route, reason):
        """Hold only on the current source reference. A nonzero cap requires the
        same full validation of that reference at the hold speed; else stop."""
        episode = getattr(self, "source_return", None)
        if episode is not None:
            episode["last_reason"] = reason
        stale = reason.startswith("objects-")
        if len(source_route) < 2:
            return None, "AUTO_SOURCE_RETURN_HOLD reason=%s no-source-route" % reason, 0.0, True
        # Validate and publish exactly the finalized hold reference.
        source_route = self._source_return_finalize(source_route)
        if stale:
            return source_route, "AUTO_SOURCE_RETURN_HOLD reason=%s" % reason, 0.0, True
        remaining = max(0.0, self._polyline_length(source_route) -
                        self.front_safety_gap)
        hold_speed = min(getattr(self, "avoid_hold_max_mps", 20.0 / 3.6),
                         math.sqrt(2.0 * self.brake_decel * remaining))
        if hold_speed <= 0.1:
            return source_route, ("AUTO_SOURCE_RETURN_HOLD reason=%s "
                                  "source-end" % reason), 0.0, False
        ok, check_reason, cap = self._source_return_validate(
            source_route, cur, ex, ey, horizon_m=remaining, speed_request=hold_speed)
        if not ok:
            return source_route, ("AUTO_SOURCE_RETURN_HOLD reason=%s hold_check=%s"
                                  % (reason, check_reason)), 0.0, check_reason.startswith("objects-")
        return source_route, ("AUTO_SOURCE_RETURN_HOLD reason=%s"
                              % reason), min(hold_speed, cap), False

    def _source_return_link(self, episode, cur):
        """Episode source link: follows its own successors; an unrelated current
        link match never replaces it (map-matching ambiguity is not a validated
        transition)."""
        source_link = self.links.get(episode.get("link"))
        if source_link is None:
            return None
        if cur.idx != source_link.idx and cur.idx in {
                link.idx for link in self._forward(source_link, self.route_len)}:
            episode["link"] = cur.idx
            return cur
        return source_link

    def _source_return_blocker(self, episode, cur, ex, ey, cur_blocker):
        if episode.get("state") != "armed":
            return cur_blocker
        source_link = self._source_return_link(episode, cur)
        if source_link is None or source_link.idx == cur.idx:
            return cur_blocker
        return self._object_lead_on_link(source_link, ex, ey)

    def _source_return_step(self, cur, ex, ey, eh, source_route, base_slice):
        """Returns (route, status, speed_cap, veto); route None -> caller stops."""
        route, status, cap, veto = self._source_return_step_inner(
            cur, ex, ey, eh, source_route, base_slice)
        return route, status, cap, veto

    def _source_return_step_inner(self, cur, ex, ey, eh, source_route, base_slice):
        episode = self.source_return
        if episode.get("state") == "armed":
            source_link = self._source_return_link(episode, cur)
            if source_link is None:
                return None, "AUTO_SOURCE_RETURN_HOLD reason=source-link-missing", 0.0, True
            source_link_for_hold = source_link
            if source_link.idx != cur.idx:
                source_route = self._route_to_link(ex, ey, source_link)
            if len(source_route) < 2:
                return None, "AUTO_SOURCE_RETURN_HOLD reason=no-source-route", 0.0, True
            offset = math.hypot(source_route[0][0] - ex, source_route[0][1] - ey)
            if offset > self.rejoin_route_refresh_m:
                return self._source_return_finalize(source_route), (
                    "AUTO_SOURCE_RETURN_HOLD reason=source-lost "
                    "dev=%.2f" % offset), 0.0, True
            cur = source_link_for_hold
        if len(base_slice) < 2:
            return self._source_return_hold(cur, ex, ey, source_route, "no-global-slice")
        if episode.get("state") == "committed":
            path = episode["path"]
            window = range(max(0, episode["progress"] - 2),
                           min(len(path), episode["progress"] + 200))
            nearest = min(window, key=lambda i: (path[i][0] - ex) ** 2 +
                          (path[i][1] - ey) ** 2)
            deviation = math.hypot(path[nearest][0] - ex, path[nearest][1] - ey)
            retained = FinalRoute(path[max(0, nearest - 1):])
            if deviation > self.rejoin_route_refresh_m:
                # No ego-origin refresh: keep the committed geometry and stop.
                return retained, ("AUTO_SOURCE_RETURN_DEVIATION dev=%.2f stop"
                                  % deviation), 0.0, True
            episode["progress"] = max(episode["progress"], nearest)
            if (nearest >= episode["merge_end_index"] and
                    self.base_match_distance is not None and
                    self.base_match_distance <= self.global_rejoin_lat_m and
                    self.base_match_heading_error is not None and
                    self.base_match_heading_error <= math.radians(20.0)):
                final_base = self._source_return_finalize(base_slice)
                ok, reason, cap = self._source_return_validate(
                    final_base, cur, ex, ey, horizon_m=40.0)
                if ok:
                    self.source_return = None
                    return final_base, ("AUTO_SOURCE_RETURN_DONE dist=%.2f"
                                        % self.base_match_distance), cap, False
                return retained, ("AUTO_SOURCE_RETURN_DONE_BLOCKED reason=%s"
                                  % reason), 0.0, reason.startswith("objects-")
            remaining_merge = self._polyline_length(
                path[nearest:episode["merge_end_index"] + 1])
            ok, reason, cap = self._source_return_validate(
                retained, cur, ex, ey, horizon_m=remaining_merge + 10.0)
            if not ok:
                return retained, ("AUTO_SOURCE_RETURN_RECHECK_FAIL reason=%s"
                                  % reason), 0.0, reason.startswith("objects-")
            return retained, ("AUTO_SOURCE_RETURN progress=%d/%d d0=%.2f"
                              % (nearest, episode["merge_end_index"],
                                 episode["d0"])), cap, False
        # armed: build and validate in locals, then commit atomically
        episode["attempts"] = episode.get("attempts", 0) + 1
        fresh, fresh_reason = self._objects_fresh_for_commit()
        if not fresh:
            return self._source_return_hold(cur, ex, ey, source_route, fresh_reason)
        if len(source_route) < 2:
            return self._source_return_hold(cur, ex, ey, source_route, "no-source-route")
        try:
            d_probe = project_to_frenet(source_route[0][0], source_route[0][1],
                                        resample_polyline(base_slice, 0.5)).d
        except ValueError:
            return self._source_return_hold(cur, ex, ey, source_route, "source-projection")
        magnitude = abs(d_probe)
        if magnitude < 0.2:
            final_base = self._source_return_finalize(base_slice)
            ok, reason, cap = self._source_return_validate(
                final_base, cur, ex, ey, horizon_m=40.0)
            if ok:
                self.source_return = None
                return final_base, ("AUTO_SOURCE_RETURN_DONE direct d0=%.2f"
                                    % d_probe), cap, False
            return self._source_return_hold(cur, ex, ey, source_route,
                                            "direct:%s" % reason)
        reasons = []
        factors = (8.0, 6.0, 10.0, 5.0)
        # Geometry only (this tick's source/global routes); every candidate
        # is still validated against this tick's objects below.
        prepared = self._source_return_merge_prepare(
            source_route, base_slice,
            max(max(10.0, factor * magnitude) for factor in factors))
        for factor in factors:
            merge_length = max(10.0, factor * magnitude)
            candidate, merge_end_index, d0 = self._source_return_merge(
                source_route, base_slice, merge_length, prepared)
            if candidate is None:
                reasons.append("L%.0f:%s" % (merge_length, d0))
                continue
            # Finalize first; validate and commit exactly these points.
            final = self._source_return_finalize(candidate)
            final_end = self._source_return_final_index(
                candidate, merge_end_index, final)
            ok, reason, cap = self._source_return_validate(
                final, cur, ex, ey, horizon_m=merge_length + 10.0)
            if not ok:
                reasons.append("L%.0f:%s" % (merge_length, reason))
                continue
            self.source_return = {
                "state": "committed", "path": final, "progress": 0,
                "merge_end_index": final_end, "merge_length": merge_length,
                "d0": d0, "link": cur.idx, "lane": cur.lane,
                "attempts": episode["attempts"]}
            return FinalRoute(final), ("AUTO_SOURCE_RETURN_COMMIT L=%.1f d0=%.2f %s"
                                       % (merge_length, d0, reason)), cap, False
        return self._source_return_hold(cur, ex, ey, source_route,
                                        "|".join(reasons)[:240])

    def _start_global_rejoin(self, ex=None, ey=None, eh=None):
        self.mode = "REJOIN"
        self.home_lane = None
        self.changing = False
        self.active_target_lane = None
        self.change_dst_idx = None
        self.change_blend_m = self.transition_m
        self.clear_cycles = 0
        self.avoid_source = None
        self.avoid_hops = 0
        self.global_rejoin_trigger_cycles = 0
        self.rejoin_stable_cycles = 0
        self.rejoin_route = []
        self.rejoin_refresh_count = 0
        self.rejoin_refresh_last = None
        self.source_return = None
        self.local_bypass_route = []
        self.local_bypass_progress = 0
        self.local_bypass_goal = None
        self.local_bypass_source = None
        self.local_bypass_max_mps = getattr(
            self, "intersection_bypass_max_mps", 15.0 / 3.6)
        self.local_bypass_replan_count = 0
        self.local_bypass_rejoin_handovers = 0
        if ex is not None and ey is not None and eh is not None:
            self.rejoin_route = self._build_base_rejoin_route(ex, ey, eh)

    @staticmethod
    def _smooth(pts):
        pts = list(pts)
        for _ in range(2):
            if len(pts) < 5:
                break
            out = [pts[0], pts[1]]
            for i in range(2, len(pts) - 2):
                out.append((
                    (pts[i - 2][0] + pts[i - 1][0] + pts[i][0] + pts[i + 1][0] + pts[i + 2][0]) / 5.0,
                    (pts[i - 2][1] + pts[i - 1][1] + pts[i][1] + pts[i + 1][1] + pts[i + 2][1]) / 5.0,
                ))
            out.extend([pts[-2], pts[-1]])
            pts = out
        return pts

    def _publish_avoidance_debug(self, pts, status, stamp):
        publisher = getattr(self, "pub_debug_markers", None)
        if publisher is None or self.mode != "LOCAL_BYPASS" or not pts:
            return
        lifetime = rospy.Duration.from_sec(0.5)
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = stamp
        line.ns = "local_bypass_validated"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.12
        line.color.r = 0.0
        line.color.g = 0.95
        line.color.b = 1.0
        line.color.a = 0.95
        line.lifetime = lifetime
        line.points = [Point(x=x, y=y, z=0.12) for x, y in pts]

        label = Marker()
        label.header = line.header
        label.ns = line.ns
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = pts[0][0]
        label.pose.position.y = pts[0][1]
        label.pose.position.z = 1.8
        label.pose.orientation.w = 1.0
        label.scale.z = 0.55
        label.color.r = 0.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.lifetime = lifetime
        label.text = "BYPASS r%d %s" % (
            getattr(self, "path_revision", 0), status[:120])
        publisher.publish(MarkerArray(markers=[line, label]))

    def _publish_avoidance_diagnostics(self, pts, v_cmd, status, stamp):
        publisher = getattr(self, "pub_diagnostics", None)
        if publisher is None:
            return
        payload = {
            "stamp_s": stamp.to_sec(),
            "mode": self.mode,
            "status": status,
            "path_revision": getattr(self, "path_revision", 0),
            "route_points": len(pts),
            "target_speed_mps": float(v_cmd),
            "gear": getattr(self, "output_gear", "D"),
            "local_decision": getattr(self, "last_local_decision", ""),
            "base_match_distance_m": getattr(
                self, "base_match_distance", None),
            "rejoin_refresh_count": int(getattr(self, "rejoin_refresh_count", 0)),
        }
        refresh_last = getattr(self, "rejoin_refresh_last", None)
        if refresh_last is not None:
            payload["rejoin_refresh_last_dev_m"] = float(refresh_last[0])
            payload["rejoin_refresh_last_speed_kmh"] = float(refresh_last[1]) * 3.6
        source = getattr(self, "local_bypass_source", None)
        if source is not None:
            payload["source_object_xy"] = [source[0], source[1]]
        publisher.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"))))

    def _publish(self, pts, v_cmd, status):
        if isinstance(pts, FinalRoute):
            # Finalized form: no resampling or smoothing. A positive command
            # needs a same-tick certificate for exactly these points; the
            # marker alone is not one.
            certified = self._certified_cap(pts)
            if v_cmd > 0.0 and (certified is None or v_cmd > certified + 1.0e-9):
                if certified is None:
                    v_cmd = 0.0
                    status += " | UNCERTIFIED_STOP"
                    rospy.logerr("[LaneChange] uncertified FinalRoute command "
                                 "forced to 0: %s", status[:120])
                else:
                    v_cmd = certified
            pts = list(pts)
        elif self.mode == "LOCAL_BYPASS":
            # Preserve validated local-bypass geometry.  Half-metre linear
            # resampling only removes excess points; it does not move the
            # polyline as the generic five-point smoother does.
            self._spend(1 + len(pts) // 8)
            pts = self._resample(pts, 0.5)
        else:
            self._spend(1 + len(pts) // 8)
            pts = self._resample(pts, 1.0)
            self._spend(1 + len(pts) // 4)
            pts = self._smooth(pts)
        if len(pts) < 2 or not all(math.isfinite(q[0]) and math.isfinite(q[1]) for q in pts):
            # revision2: never leave the previous positive command active
            self._publish_stop_only(status + " | NO_VALID_ROUTE_STOP")
            return
        gear = getattr(self, "output_gear", "D")
        if not math.isfinite(v_cmd) or v_cmd < 0.0:
            v_cmd = 0.0
            status += " | NONFINITE_SPEED_STOP"
        final_points = tuple((float(q[0]), float(q[1])) for q in pts)
        if v_cmd > 0.0 and gear == "D":
            # revision2: ONE final authorization gate for every positive forward publication (plain list or
            # FinalRoute), on the exact points emitted below, the current frame and the envelope policy
            v_cmd, gate_reason = self._final_gate(final_points, v_cmd)
            if gate_reason != "certificate":
                status += " | GATE %s" % gate_reason
        self._spend(1 + len(pts) // 8)                     # serialization
        waypoints = []
        for i, (x, y) in enumerate(pts):
            j = min(i + 1, len(pts) - 1)
            heading = math.atan2(pts[j][1] - y, pts[j][0] - x)
            waypoints.append({
                "x": x, "y": y, "heading": heading, "gear": gear})
        if v_cmd > 0.0:
            # revision2: unconditional deadline check before any positive publication
            budget = getattr(self, "_tick_budget", None)
            if budget is not None:
                budget.check()
        # Single consistent command state: what is emitted is what the next
        # release limiter and the next envelope (previous command) start from.
        self.last_speed_command_mps = v_cmd
        self._last_emitted = {"points": tuple(tuple(q) for q in pts),
                              "mode": self.mode, "frame": getattr(self, "_tick_frame", None),
                              "speed": v_cmd, "status": status}
        self.pub_wps.publish(String(data=json.dumps({"waypoints": waypoints})))
        self.pub_vel.publish(Float32(data=v_cmd))
        self.pub_status.publish(String(data=status))
        self.pub_mode.publish(String(data=self.mode))

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for x, y in pts:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)
        self.pub_path.publish(path)
        self._publish_avoidance_diagnostics(
            pts, v_cmd, status, path.header.stamp)
        self._publish_avoidance_debug(
            pts, status, path.header.stamp)

    def _publish_stop_only(self, status):
        """Explicit zero speed without a usable route (no waypoints message)."""
        self.last_speed_command_mps = 0.0
        self.pub_vel.publish(Float32(data=0.0))
        self.pub_status.publish(String(data=status))
        self.pub_mode.publish(String(data=self.mode))

    def _stop_point_need(self, speed_mps):
        """Length ahead of the reference a stop-point prefix must contain. revision3: the controller feed timeout
        is part of every envelope (ArrivalEnvelope.t_feed), so this equals the envelope required length (no
        separate extra; the revision2 extra is folded in, not doubled)."""
        envelope = self._execution_envelope(speed_mps)
        if envelope is None or not envelope.valid():
            return float("inf")
        return envelope.required_length

    def _final_gate(self, points, v_cmd):
        """Authorize a positive forward command on the exact final points. Returns (speed, reason).

        1) a same-tick certificate for these exact points (frame + policy bound) with cap >= v -> reuse;
        2) FinalRoute publications are certified by their validators; missing -> handled by the caller;
        3) otherwise the execution envelope on the published route at v; if an object conflict (or the
           route is too short for the envelope) remains, a STOP-POINT prefix ending 0.5 m before the first
           conflicting station is certified with the largest speed whose stop-guarantee reach stays in the
           prefix. No such speed -> 0."""
        # revision3: an invalid actor in the frame or a non-finite object field never authorizes motion
        if getattr(self, "objects_invalid", ()):
            return 0.0, "gate-invalid-actor"
        if not all(all(math.isfinite(float(v)) for v in raw[:4]) and
                   all(math.isfinite(float(v)) for v in raw[5:7]) and
                   (len(raw) <= 10 or math.isfinite(float(raw[10]))) for raw in (self.objects or [])):
            return 0.0, "gate-nonfinite-object"
        if getattr(self, "_epoch_inputs_pending", False):
            return 0.0, "gate-clock-epoch-inputs-pending"
        certified = self._certified_cap(points)
        if certified is not None and certified >= v_cmd - 1.0e-9:
            return v_cmd, "certificate"
        if len(points) < 2:
            return 0.0, "short-route"
        margin = float(getattr(self, "local_bypass_final_clearance_m",
                               getattr(self, "intersection_bypass_clearance_m", 0.35)))
        route = list(points)
        obstacles = self._prefilter_obb_obstacles(route, route[0][0], route[0][1], v_cmd, margin)
        result = self._ego_swept_clearance(route, obstacles, v_cmd, safety_margin=margin, sample_step=0.35)
        if result.get("valid"):
            self._certify(points, v_cmd)
            return v_cmd, "certificate"
        pose = self._execution_ego_pose(route)
        if pose is None:
            return 0.0, "no-ego-pose"
        # stop-point prefix
        total = self._polyline_length(route)
        conflict = result.get("first_conflict_station", float("inf"))
        kind = (result.get("closest") or {}).get("kind")
        if not math.isfinite(conflict):
            if kind != "envelope-beyond-checked-route":
                return 0.0, "gate-%s" % kind
            conflict = total
        s0 = self._point_projection_on_route(pose[0], pose[1], route)
        s0 = 0.0 if s0 is None else s0[0]
        v_try = v_cmd
        for _attempt in range(4):
            prefix_len = conflict - 0.5
            if prefix_len <= s0 + 0.05:
                return 0.0, "gate-conflict-at-ego s=%.2f" % conflict
            lo, hi = 0.0, v_try
            if s0 + self._stop_point_need(0.0) > prefix_len:
                return 0.0, "gate-cannot-stop-before s=%.2f" % conflict
            for _ in range(12):
                mid = 0.5 * (lo + hi)
                need = self._stop_point_need(mid)
                if need is not None and s0 + need <= prefix_len:
                    lo = mid
                else:
                    hi = mid
            v_try = lo
            if v_try <= 1.0e-3:
                return 0.0, "gate-no-speed s=%.2f" % conflict
            prefix = self._route_prefix_by_length(route, prefix_len)
            if len(prefix) < 2:
                return 0.0, "gate-short-prefix"
            obstacles = self._prefilter_obb_obstacles(prefix, prefix[0][0], prefix[0][1], v_try, margin)
            result = self._ego_swept_clearance(prefix, obstacles, v_try, safety_margin=margin, sample_step=0.35)
            if result.get("valid"):
                self._certify(points, v_try)
                return v_try, "stop-point v=%.2f s=%.1f" % (v_try, prefix_len)
            nxt = result.get("first_conflict_station", float("inf"))
            if not math.isfinite(nxt) or nxt >= conflict - 1.0e-6:
                return 0.0, "gate-%s" % ((result.get("closest") or {}).get("kind"))
            conflict = nxt
        return 0.0, "gate-no-convergence"

    # ── Execution contract: tick budget, retained reference ─────────────────
    _FRAME_KEYS = frozenset((
        "ego_receipt", "ego", "ego_v", "ego_signed_v", "objects",
        "objects_receipt", "_tick_frame", "objects_invalid", "_frame_epoch", "_objects_ever",
        "_frame_ego_current", "_frame_obj_current"))
    _CALLBACK_KEYS = frozenset((
        "_ego_packet", "_ego_version", "ego_history", "vehicle_gear",
        "vehicle_status_receipt", "_obj_packet", "_obj_version",
        "_clock_lock", "_clock_epoch", "_clock_last", "_clock_discard", "_clock_resets",
        "traffic_filter", "_cmd_version"))
    # revision3: declared rollback-excluded progress cache (validated by exact rescoring on completion; an
    # aborted tick keeps completed reacquisition chunks, nothing else is claimed restored or kept)
    _PROGRESS_CACHE_KEYS = frozenset(("_reacq_state",))
    _CMD_KEYS = frozenset((
        "target_lane", "active_target_lane", "change_dst_idx", "changing",
        "mode", "home_lane"))

    def _spend(self, units=1):
        """Charge the shared per-tick budget (no-op outside a planning tick)."""
        budget = getattr(self, "_tick_budget", None)
        if budget is not None:
            budget.spend(units)

    def _begin_tick_budget(self):
        self._tick_budget = TickBudget(
            getattr(self, "planner_tick_work_units", TICK_WORK_UNITS),
            getattr(self, "planner_tick_deadline_s", TICK_DEADLINE_S))
        return self._tick_budget

    def _retained_reference(self, ex, ey):
        """Exact committed/last emitted reference suffix at the ego, computed
        read-only (no progress/state update). LOCAL_BYPASS: the committed
        FinalRoute suffix exactly as KEEP selects it. Otherwise: the last
        emitted points. Returns (FinalRoute, tag) or (None, None) when no
        usable cache exists (missing, < 2 points, ego > 3 m away)."""
        def near(points, lo=0, hi=None):
            hi = len(points) if hi is None else hi
            if hi - lo < 1:
                return None, None
            index = min(range(lo, hi), key=lambda k: (
                (points[k][0] - ex) ** 2 + (points[k][1] - ey) ** 2))
            return index, math.hypot(points[index][0] - ex, points[index][1] - ey)
        route = getattr(self, "local_bypass_route", None)
        if self.mode == "LOCAL_BYPASS" and route and len(route) >= 2:
            previous = max(0, int(getattr(self, "local_bypass_progress", 0)))
            index, distance = near(route, max(0, previous - 4),
                                   min(len(route), previous + 160))
            if index is not None and distance <= 3.0:
                start = max(previous, index)
                suffix = route[start:start + RETAINED_SCAN_MAX]      # bounded stop fallback
                if len(suffix) < 2:
                    suffix = route[-2:]
                return FinalRoute(tuple(tuple(q) for q in suffix)), "bypass-suffix"
        last = getattr(self, "_last_emitted", None)
        if last and len(last["points"]) >= 2:
            points = last["points"]
            index, distance = near(points, 0, min(len(points), RETAINED_SCAN_MAX))
            if index is not None and distance <= 3.0:
                suffix = (points[index:index + RETAINED_SCAN_MAX] if len(points) - index >= 2
                          else points[-2:])
                return FinalRoute(suffix), "emitted-suffix"
        return None, None

    def _plan(self):
        """One planning tick under a shared work/deadline budget. Budget
        exhaustion anywhere unwinds every chooser (TickBudgetExceeded), rolls
        back planner-owned state reassigned during the tick (frame and
        callback-owned fields excluded) and publishes an explicit stop on the
        retained reference; the next tick starts with a fresh budget."""
        budget = self._begin_tick_budget()
        snapshot = dict(self.__dict__)
        # revision2 P-STATE-TRANSACTION: the shallow snapshot restores reassigned attributes; the only
        # planner-owned container mutated IN PLACE during a tick is the source-return episode, so it is
        # snapshotted by value (small dict; ROS/callback state is not copied).
        import copy as _copy
        episode_snapshot = _copy.deepcopy(getattr(self, "source_return", None))
        cmd_version = getattr(self, "_cmd_version", 0)
        try:
            self._plan_body()
        except TickBudgetExceeded as exc:
            self._tick_budget = None
            skip = self._FRAME_KEYS | self._CALLBACK_KEYS | self._PROGRESS_CACHE_KEYS | {"_tick_budget"}
            if getattr(self, "_cmd_version", 0) != cmd_version:
                skip = skip | self._CMD_KEYS
            for key, value in snapshot.items():
                if key not in skip and self.__dict__.get(key) is not value:
                    self.__dict__[key] = value
            for key in [k for k in self.__dict__ if k not in snapshot and k not in skip]:
                del self.__dict__[key]          # revision2: attributes first created by the aborted tick
            self.source_return = episode_snapshot
            self._certificates = []
            self._budget_stop(budget, str(exc))
        finally:
            self._tick_budget = None
            self._last_tick_budget = budget.report()

    def _budget_stop(self, budget, reason):
        ego = getattr(self, "ego", None)
        report = budget.report()
        status = "TICK_BUDGET_EXHAUSTED reason=%s used=%d/%d elapsed=%.1fms" % (
            reason, report["used"], report["limit"], report["elapsed_ms"])
        pose = self._finite_ego_pose()
        if pose is None:
            self._publish_stop_only(status + " ref=none")
            return
        ex, ey, eh = pose
        route, tag = self._retained_reference(ex, ey)
        if route is None:
            # no validated cache: bounded explicit fallback (controller centres steering at rest)
            route = [(ex, ey), (ex + 5.0 * math.cos(eh), ey + 5.0 * math.sin(eh)),
                     (ex + 10.0 * math.cos(eh), ey + 10.0 * math.sin(eh))]
            tag = "STALE_NO_CACHE heading-hold"
        status += " ref=%s" % tag
        self.last_speed_command_mps = 0.0
        self._publish(route, 0.0, status)
        rospy.logwarn_throttle(1.0, "[LaneChange] %s", status)

    def _finite_ego_pose(self):
        ego = getattr(self, "ego", None)
        if ego is None:
            return None
        try:
            x, y = float(ego.position.x), float(ego.position.y)
            h = math.radians(float(ego.heading))
        except (AttributeError, TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(h)):
            return None
        return x, y, h

    def _hold_reference(self, ex, ey, eh, allow_map):
        """Braking/stopped-hold geometry, read-only: committed episode route, then the retained validated
        suffix (bypass or last emitted, any mode, also at rest), then the cached rejoin route; only without
        any cache the lane reference (allow_map) or a bounded heading line. Returns (route, tag)."""
        episode_route = self._source_return_stale_reference(ex, ey, None)
        if episode_route:
            return episode_route, "episode-route"
        route, tag = self._retained_reference(ex, ey)
        if route is not None:
            return route, tag
        if self.mode == "REJOIN" and len(getattr(self, "rejoin_route", []) or []) >= 2:
            return list(self.rejoin_route), "rejoin-route"
        if allow_map and self.mode not in ("AVOID", "RETURN", "LOCAL_BYPASS", "REJOIN", "BACKUP"):
            # lane-following modes only: an avoidance-mode ego may be offset from the lane, and braking onto an
            # unvalidated lane reference would steer it across (E2 no-cache rule kept from 08f7)
            cur = self._current_link(ex, ey, eh)
            if cur is not None:
                if self.use_base_route:
                    self._match_base_route(ex, ey, eh)
                    route = self._base_route_slice(ex, ey, eh)
                    if len(route) >= 2:
                        return route, "no-cache base-route"
                route = self._route_to_link(ex, ey, cur)
                if len(route) >= 2:
                    return route, "no-cache lane"
        return [(ex, ey), (ex + 5.0 * math.cos(eh), ey + 5.0 * math.sin(eh)),
                (ex + 10.0 * math.cos(eh), ey + 10.0 * math.sin(eh))], "STALE_NO_CACHE heading-hold"

    def _required_input_stop(self, status, allow_map=False, pose=None):
        """Explicit zero BEFORE any map-link/route dispatch (revision2 P-HARDSTALE-NOLANE)."""
        pose = pose or self._finite_ego_pose()
        if pose is None:
            self._publish_stop_only(status + " ref=none")
            rospy.logwarn_throttle(1.0, "[LaneChange] %s", status)
            return
        ex, ey, eh = pose
        route, tag = self._hold_reference(ex, ey, eh, allow_map)
        status += " ref=%s" % tag
        traffic_status = "TRAFFIC_SUPERSEDED %s" % status.split(" ", 1)[0].lower()
        self._publish_traffic_status(traffic_status)
        if getattr(self, "traffic_stop_enable", False):
            status += " | " + traffic_status
        self._write_diagnostics(None, route, status)
        self.last_speed_command_mps = 0.0
        self._publish(route, 0.0, status)
        rospy.logwarn_throttle(1.0, "[LaneChange] %s", status)

    def _plan_body(self):
        self._take_frame()
        self._certificates = []
        # revision3: first tick of a new clock epoch -> every time-based planner state is invalid; motion needs
        # fresh post-reset ego (and objects, if a perception feed was ever received)
        epoch = getattr(self, "_frame_epoch", 0)
        if epoch != getattr(self, "_planner_epoch", 0):
            self._planner_epoch = epoch
            self._epoch_inputs_pending = True
            self._epoch_objects_required = bool(getattr(self, "_objects_ever_before", False) or
                                                getattr(self, "_objects_ever", False))
            try:
                self.traffic_filter.reset()
            except Exception:
                pass
            self.vehicle_status_receipt = None
            # every planner-owned receipt not supplied by a current-epoch packet belongs to the old epoch
            if not getattr(self, "_frame_ego_current", False):
                self.ego = None; self.ego_receipt = None
            if not getattr(self, "_frame_obj_current", False):
                self.objects = []; self.objects_receipt = None; self.objects_invalid = ()
            rospy.logwarn("[LaneChange] ROS clock stepped backward: epoch %d, waiting for post-reset inputs", epoch)
        if getattr(self, "_epoch_inputs_pending", False):
            have_ego = self.ego is not None and self.ego_receipt is not None
            have_obj = self.objects_receipt is not None or not getattr(self, "_epoch_objects_required", False)
            if have_ego and have_obj:
                self._epoch_inputs_pending = False
            else:
                pose = self._finite_ego_pose() if have_ego else None
                self._required_input_stop("CLOCK_EPOCH_RESET waiting for post-reset %s" % (
                    "ego" if not have_ego else "objects"), pose=pose)
                return
        if self.ego is None:
            self._publish_stop_only("NO_EGO missing")
            return
        ego_age_v = self._receipt_age(self.ego_receipt)
        ego_age = float("inf") if ego_age_v is None else ego_age_v
        pose = self._finite_ego_pose()
        if pose is None:
            self._publish_stop_only("NO_EGO invalid_pose")
            return
        if ego_age_v is None or ego_age > self.ego_timeout_s:
            # stale, future (backward clock) or unstarted-clock receipt: hold on the retained reference
            self._required_input_stop("NO_EGO %s age=%.3f" % ("stale" if ego_age_v is not None else "invalid-receipt",
                                                             ego_age), pose=pose)
            return
        if self.objects_receipt is not None and self._receipt_age(self.objects_receipt) is None:
            # revision3: a future / unstarted-clock perception receipt is invalid, never fresh (all modes)
            self._required_input_stop("PERCEPTION_INVALID_RECEIPT (future or unstarted clock)", allow_map=True, pose=pose)
            return
        if getattr(self, "objects_invalid", ()):
            # revision3: an actor with a non-finite position/velocity/heading/size makes the frame an invalid
            # required input in every mode (never silently clear); the next valid packet recovers
            inv = self.objects_invalid
            self._required_input_stop("PERCEPTION_INVALID_ACTOR n=%d first=%s:%s:%s" % (
                len(inv), inv[0][0], inv[0][1], "/".join(inv[0][2])), allow_map=True, pose=pose)
            return
        object_age_v = self._receipt_age(self.objects_receipt)
        object_age = float("inf") if object_age_v is None else object_age_v
        perception_stale = (
            not math.isfinite(object_age) or object_age > self.object_timeout_s)
        hard_timeout = getattr(
            self, "object_hard_timeout_s", self.object_timeout_s)
        perception_hard_stale = (
            not math.isfinite(object_age) or object_age > hard_timeout)
        if perception_hard_stale:
            self.objects = []
        ex, ey, eh = pose
        if self.require_object_feed and perception_hard_stale:
            # Freeze the safety state machine while perception is unhealthy, before any link matching or
            # mode dispatch. Braking and the stopped hold keep the last validated geometry; the controller
            # centres steering at rest (no heading-line substitution).
            self._required_input_stop("PERCEPTION_STALE_HOLD mode=%s age=%.3f" % (self.mode, object_age),
                                      allow_map=True, pose=pose)
            return
        if self.use_base_route:
            self._match_base_route(ex, ey, eh)
            pending = getattr(self, "base_reacquire_pending", None)
            if pending is not None:
                self._required_input_stop("BASE_REACQUIRE_PENDING %d/%d" % pending, pose=pose)
                return
        cur = self._current_link(ex, ey, eh)
        if cur is None:
            recovery = self._rejoin_without_link(ex, ey, eh)
            if recovery is not None:
                route, v_cmd, status = recovery
                # Assess the route that will actually be published, exactly
                # as the linked REJOIN branch does, before any release limit.
                blocker = self._object_lead_on_route(route, ex, ey)
                if blocker is not None:
                    blocker_cap = self._blocker_speed_cap(blocker)
                    v_cmd = min(v_cmd, max(0.0, blocker_cap))
                    status += " | BLOCKER class=%s object_s=%.1f cap=%.1f" % (
                        blocker[5], blocker[0], blocker_cap * 3.6)
                if perception_stale and not perception_hard_stale:
                    v_cmd = min(
                        v_cmd,
                        getattr(self, "perception_degraded_mps", 10.0 / 3.6))
                    status += " | PERCEPTION_DEGRADED age=%.3f" % object_age
                traffic_cap, traffic_status = self._traffic_speed_limit(
                    route, ex, ey, rospy.Time.now().to_sec())
                if traffic_cap is not None:
                    v_cmd = min(v_cmd, max(0.0, traffic_cap))
                v_cmd = self._limit_speed_cap_release(v_cmd)
                self._publish_traffic_status(traffic_status)
                if getattr(self, "traffic_stop_enable", False):
                    status += " | " + traffic_status
                self._write_diagnostics(None, route, status)
                self._publish(route, v_cmd, status)
                rospy.logwarn_throttle(
                    1.0, "[LaneChange] %s v=%.1fkm/h", status, v_cmd * 3.6)
                return
            # no map match and no recovery route: explicit stop on the retained reference
            self._required_input_stop("NO_LANE cannot_match_current_pose", pose=pose)
            return
        self.auto_speed_cap = None
        self.output_gear = "D"

        if (self.target_lane <= 0 and self.auto_avoid and
                self.mode == "BACKUP"):
            route, status = self._backup_recovery_step(cur, ex, ey, eh)
        elif self.target_lane <= 0 and self.auto_avoid:
            route, status = self._auto_route(cur, ex, ey, eh)
        elif self.target_lane <= 0:
            self.changing = False
            self.active_target_lane = None
            self.change_dst_idx = None
            self.mode = "CRUISE"
            self.home_lane = None
            self.change_blend_m = self.transition_m
            self.avoid_hops = 0
            route = self._route_to_link(ex, ey, cur)
            status = "CRUISE road=%s lane=%s link=%s" % (cur.road, cur.lane, cur.idx)
        elif self.changing and self.change_dst_idx in self.links:
            self.mode = "MANUAL"
            dst = self.links[self.change_dst_idx]
            route = self._route_to_link(ex, ey, dst, self.transition_m)
            status = "LANE_CHANGE road=%s lane=%s target=%s step=%s link=%s" % (
                cur.road, cur.lane, self.target_lane, dst.lane, cur.idx
            )
            detected = self._target_lane_reached(cur, ex, ey, eh)
            if detected is not None:
                self.changing = False
                self.change_dst_idx = None
                self.ref_link = detected.idx
                rospy.loginfo("[LaneChange] %d차선 도착", detected.lane)
        elif self.target_lane == cur.lane:
            self.mode = "MANUAL"
            self.active_target_lane = None
            self.change_dst_idx = None
            self.change_blend_m = self.transition_m
            self.avoid_hops = 0
            route = self._route_to_link(ex, ey, cur)
            status = "CRUISE road=%s lane=%s link=%s" % (cur.road, cur.lane, cur.idx)
        else:
            self.mode = "MANUAL"
            dst, side = self._select_next_lane_link(cur, self.target_lane)
            if dst is None:
                route = self._route_to_link(ex, ey, cur)
                status = "WAIT road=%s lane=%s target=%s reason=%s link=%s" % (
                    cur.road, cur.lane, self.target_lane, side, cur.idx
                )
            else:
                self.changing = True
                self.active_target_lane = dst.lane
                self.change_dst_idx = dst.idx
                rospy.loginfo("[LaneChange] %d차선 -> %d차선 (%s)", cur.lane, dst.lane, side)
                route = self._route_to_link(ex, ey, dst, self.transition_m)
                status = "LANE_CHANGE road=%s lane=%s target=%s step=%s link=%s" % (
                    cur.road, cur.lane, self.target_lane, dst.lane, cur.idx
                )

        if (self.target_lane <= 0 and self.auto_avoid and
                self.mode != "BACKUP"):
            backup = self._maybe_start_backup_recovery(
                cur, route, status, ex, ey, eh)
            if backup is not None:
                route, status = backup

        v_cmd = min(self.cruise_mps, cur.vmax)
        if self.auto_speed_cap is not None:
            v_cmd = min(v_cmd, max(0.0, self.auto_speed_cap))
        if self.require_object_feed and perception_stale:
            v_cmd = min(
                v_cmd,
                getattr(self, "perception_degraded_mps", 10.0 / 3.6))
            status += " | PERCEPTION_DEGRADED age=%.3f" % object_age
        traffic_cap, traffic_status = self._traffic_speed_limit(
            route, ex, ey, rospy.Time.now().to_sec())
        if traffic_cap is not None:
            v_cmd = min(v_cmd, max(0.0, traffic_cap))
        v_cmd = self._limit_speed_cap_release(v_cmd)
        self._publish_traffic_status(traffic_status)
        if getattr(self, "traffic_stop_enable", False):
            status += " | " + traffic_status
        self._write_diagnostics(cur, route, status)
        self._publish(route, v_cmd, status)
        rospy.loginfo_throttle(
            1.0,
            "[LaneChange] %s v=%.1fkm/h",
            status,
            v_cmd * 3.6,
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._plan()
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                # revision3: a backward clock step must not kill the planner; the next tick sees the new epoch
                rate = rospy.Rate(self.rate_hz)


if __name__ == "__main__":
    try:
        HdmapLaneChange().spin()
    except rospy.ROSInterruptException:
        pass
