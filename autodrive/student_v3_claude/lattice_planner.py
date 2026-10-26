"""Candidate-path planner: only physical collision is a hard constraint.

Every frame a fan of lateral-offset paths is generated around the route
(Frenet offsets), together with the previous choice and the model path.
Each candidate is swept with the car body against LiDAR points and static
object boxes. Lane rules (the HD-map corridor), lateral deviation, changes of
mind and curvature only add cost. The car stops only when no candidate leaves
braking room before a measured obstacle.

The learned model is one candidate among many. A sideways model path without
an obstacle costs more than the lane centre and is not chosen (2026-10-03 logs:
86-96 % of the model's avoidance outputs had no obstacle).
"""
from __future__ import annotations

import math
import time
import os
from itertools import product

import numpy as np
from scipy.spatial import cKDTree

from tunnel_runtime import (LATTICE_NEAR_SIDE_MARGIN_M, partition_lattice_points,
                            swept_clearance, swept_clearances)
from ego_body import ROAD_LEGACY_SHIFT_M

HALF_L, HALF_W = 2.3175, .946
SIDE_MARGIN_M = float(os.environ.get("LATTICE_SIDE_MARGIN", ".35"))
END_MARGIN_M = .3
# How far the body may extend beyond the legal lanes (shoulder / opposite
# lane edge) when nothing else is collision-free. Penalised, never preferred.
# 0: the body never leaves the legal lanes (solid edge lines / centre line; dashed
# same-direction neighbours are legal). Rule 3-2 "차로 준수" and user 2026-10-05: no line
# crossing except a long stall (gui passes outside_max_m there).
OUTSIDE_LEGAL_MAX_M = float(os.environ.get("LATTICE_OUTSIDE_MAX", "1.0"))   # strict 0.0 squeezed past trucks (gate r7: studio_box_21/28/50 collisions)
# The body may never pass the physical road edge (union of all HD-map lanes).
ROAD_EDGE_TOLERANCE_M = float(os.environ.get("LATTICE_ROAD_EDGE_TOL", ".05"))
# Paved shoulder usable beyond the rightmost mapped lane edge (body corners).
RIGHT_SHOULDER_M = float(os.environ.get("LATTICE_RIGHT_SHOULDER_M", "1.2"))
W_WIDE = 4.            # only with wide_side_m (tunnel)
# Ioniq5: wheelbase 3.0 m, 35 deg road-wheel lock -> 0.23 1/m; keep a margin.
LOW_SPEED_MAX_CURVATURE = float(os.environ.get("LATTICE_LOW_SPEED_MAX_CURV", ".21"))
W_SIDE_SWITCH = 30.   # left<->right swap next to an obstacle
W_CENTRE_LINE = 200.   # crossing the centre (yellow) line: last resort only
W_SHOULDER_BEYOND = 120.   # beyond RIGHT_SHOULDER_M: last resort, before the centre line
SHOULDER_LAST_RESORT_M = float(os.environ.get("LATTICE_SHOULDER_LAST_RESORT_M", "1.5"))
BRAKE_MPS2 = float(os.environ.get("LATTICE_BRAKE_MPS2", "2.5"))
STANDOFF_M = 3.0
ALAT_MPS2 = float(os.environ.get("LATTICE_ALAT_MPS2", "3.0"))

W_LATERAL = 1.0        # per metre of mean offset from the route
W_OUTSIDE = 25.0       # per metre of body outside the legal lanes
W_SWITCH = 1.5         # per metre of change of the target offset
W_BLOCKED = 4.0        # per metre of horizon not collision-free
W_CURVATURE = 30.0     # per 1/m of peak curvature
W_MODEL = 0.0          # model agreement bonus (off: model is unreliable)
SWITCH_HYSTERESIS = 1.0
LAT_ACCEL_MAX_MPS2 = 3.0
W_HARSH = 50.          # effectively last resort  # cost units a new course must beat the current one by
W_PROXIMITY = 3.0      # passing closer than COMFORT_SIDE_M when room exists
COMFORT_SIDE_M = .8


def _arc(path):
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]


def _frame(route):
    tangent = np.gradient(route, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    return tangent, np.column_stack((-tangent[:, 1], tangent[:, 0]))


def _smooth(u):
    u = np.clip(u, 0., 1.)
    return u ** 3 * (10. - 15. * u + 6. * u * u)


RETURN_ALAT_MPS2 = float(os.environ.get("LATTICE_RETURN_ALAT", "2.0"))
# Line following (base_offset) re-plans from where the previous plan put the car, not
# from the car itself, while the car is within this distance of it (run 227: every
# frame restarted the join at the car, so a 2-5 deg heading bias of the localization
# showed up nowhere - follower CTE < 0.1 m - while the car slid from the guardrail line
# (-0.55 m) to +0.96 m, 0.05 m from the median guardrail). Beyond it: the car, as before
# (the follower enters RECOV at 1.5 m).
ANCHOR_MAX_DEV_M = float(os.environ.get("LATTICE_ANCHOR_MAX_DEV", ".8"))
RACING_PEAK_EXTRA_M = 6.
# Nothing in the lane and the car within this of the lane centre (or the racing
# line): the candidate is the line itself, not a join that starts at the car.
# Re-anchoring every plan at the car baked the follower's curve lag into the next
# path (CTE ~0 for the follower while 0.7-1.8 m off the lane in the start left turn,
# judge truth runs 224-234, centre-line touches nearly every run). The follower
# now sees the real offset and corrects it.
LANE_ANCHOR_M = float(os.environ.get("LATTICE_LANE_ANCHOR_M", "0"))   # off: strict lanes + anchoring hit a truck (studio_box_1); 0.8 to try


def comfortable_join_m(offset_m, v_mps, alat_mps2=None):
    """Shortest quintic join for a lateral offset at speed: peak curvature
    5.77*d/L^2 times v^2 within RETURN_ALAT_MPS2 (or alat_mps2)."""
    v = max(float(v_mps), 3.)
    alat = RETURN_ALAT_MPS2 if alat_mps2 is None else max(.3, float(alat_mps2))
    return math.sqrt(5.77 * abs(float(offset_m)) * v * v / alat)


def offset_profile(arc, d0, target, change_m, hold_until=None, return_m=None, slope0=0.):
    """d(s): quintic d0 -> target over change_m, hold, optional quintic back to 0.

    slope0 is the car's current lateral slope dd/ds. Starting every replan
    parallel to the lane reset a car that was mid-manoeuvre and made the
    heading error jump up to 10 deg between frames (wobble at 34 km/h).
    """
    u = np.clip(arc / max(change_m, 1e-3), 0., 1.)
    big_d, big_s = target - d0, slope0 * max(change_m, 1e-3)
    d = (d0 + big_s * u + (10. * big_d - 6. * big_s) * u ** 3
         + (-15. * big_d + 8. * big_s) * u ** 4 + (6. * big_d - 3. * big_s) * u ** 5)
    if hold_until is not None and return_m:
        back = _smooth((arc - hold_until) / return_m)
        d = d * (1. - back)
    return d


def slalom_profiles(arc, route, normal, points, d0, slope0, speed, side_margin, corridor,
                    max_row_gap_m=14., padding_steps=(.05, .25, .5, .75), min_first_join_m=8.):
    """Overlapping quintic joins for close rows, rather than one offset per cluster.

    Each row offers both passing sides. The full swept-body and map checks in
    plan() decide feasibility; a pointwise lateral gap alone is not sufficient.
    No reduced cruise speed or relaxed collision margin is used here.
    """
    if len(points) < 2:
        return []
    _, nearest = cKDTree(route).query(points)
    along = arc[nearest]
    lateral = np.sum((points - route[nearest]) * normal[nearest], axis=1)
    selected = (along > 2.) & (along < min(arc[-1] - 8., 55.)) & (np.abs(lateral) < 4.)
    order = np.argsort(along[selected])
    a, l = along[selected][order], lateral[selected][order]
    if len(a) < 2:
        return []
    cuts = np.flatnonzero(np.diff(a) > 1.5) + 1
    rows = [(float(np.median(aa)), float(ll.min()), float(ll.max()))
            for aa, ll in zip(np.split(a, cuts), np.split(l, cuts))]
    if not 2 <= len(rows) <= 4 or any(rows[i + 1][0] - rows[i][0] > max_row_gap_m
                                     for i in range(len(rows) - 1)):
        return []
    if max((lo + hi) / 2 for _, lo, hi in rows) - min((lo + hi) / 2 for _, lo, hi in rows) < .6:
        return []
    result = []
    for pad in padding_steps:
        choices = [(min(0., lo - HALF_W - side_margin - pad),
                    max(0., hi + HALF_W + side_margin + pad)) for _, lo, hi in rows]
        if corridor is not None:
            # Prune sides whose row targets already lie beyond the surveyed
            # envelope; full rotated-corner checks still run for every path.
            limited = []
            for (station, _, _), options in zip(rows, choices):
                j = int(np.argmin(abs(arc - station)))
                lower = float(corridor.right[j]) + HALF_W - .05
                upper = float(corridor.left[j]) - HALF_W + .05
                limited.append(tuple(dict.fromkeys(float(np.clip(t, lower, upper)) for t in options
                                                   if lower - .2 <= t <= upper + .2)))
            choices = limited
        for targets in product(*choices):
            if max(abs(t) for t in targets) > 3.5 or max(targets) - min(targets) < .3:
                continue
            for scale in (.35, .45, .65, .85, 1.05):
                first_join = max(min_first_join_m, min(rows[0][0], comfortable_join_m(targets[0] - d0, speed)))
                d = offset_profile(arc, d0, targets[0], first_join, slope0=slope0)
                for i in range(1, len(rows)):
                    gap = rows[i][0] - rows[i - 1][0]
                    length = max(gap, scale * comfortable_join_m(targets[i] - targets[i - 1], speed))
                    # Overlap the joins: insisting on a plateau until the rear
                    # clears every row loses feasible diagonal body sweeps.
                    start = max(0., (rows[i][0] + rows[i - 1][0] - length) / 2.)
                    d += (targets[i] - targets[i - 1]) * _smooth((arc - start) / length)
                hold = rows[-1][0] + HALF_L + 3.
                ret = max(10., comfortable_join_m(targets[-1], speed))
                d *= 1. - _smooth((arc - hold) / ret)
                result.append(("slalom", float(targets[-1]), first_join, hold, d))
    return result


def curvature(path):
    path = np.asarray(path, dtype=float)
    if len(path) < 3:
        return np.zeros(1)
    a, b = np.diff(path, axis=0)[:-1], np.diff(path, axis=0)[1:]
    chord = a + b
    return 2 * np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]) / np.maximum(
        1e-9, np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(chord, axis=1))


PEDESTRIAN_INFLATE_M = .5   # people get extra room on top of the side margin


def object_points(objects, ego, spacing=.2, pedestrian_inflate=PEDESTRIAN_INFLATE_M):
    """Box perimeters of static objects in the vehicle frame."""
    yaw = math.radians(float(ego["heading_deg"]))
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for obj in objects:
        length = max(.3, float(obj[6]) if len(obj) > 6 else float(obj[2]))
        width = max(.3, float(obj[7]) if len(obj) > 7 else float(obj[2]))
        if obj[4] == "pedestrian":
            length, width = length + 2 * pedestrian_inflate, width + 2 * pedestrian_inflate
        heading = math.radians(float(obj[8])) if len(obj) > 8 else yaw
        dx, dy = float(obj[0]) - ego["x"], float(obj[1]) - ego["y"]
        cx, cy = c * dx + s * dy, -s * dx + c * dy
        h = heading - yaw
        ax, ay = math.cos(h), math.sin(h)
        corners = []
        for p, q in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
            corners.append((cx + p * length / 2 * ax - q * width / 2 * ay,
                            cy + p * length / 2 * ay + q * width / 2 * ax))
        corners = np.asarray(corners)
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            n = max(2, int(np.linalg.norm(b - a) / spacing))
            out.append(a + (b - a) * np.linspace(0., 1., n, endpoint=False)[:, None])
    return np.vstack(out) if out else np.zeros((0, 2))


def corridor_side_violation(corridor, path, horizon, edge_margin=0.):
    """(left depth, right depth) beyond the corridor at each 0.5 m pose."""
    if corridor is None:
        return np.zeros(1), np.zeros(1)
    arc = _arc(path)
    sample = np.arange(0., min(horizon, arc[-1]) + .01, .5)
    centres = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
    tangent, normal = _frame(centres) if len(centres) > 2 else (np.array([[1., 0.]]), np.array([[0., 1.]]))
    shift = float(getattr(corridor, 'body_shift_m', ROAD_LEGACY_SHIFT_M))   # LaneCorridor / ego_body
    corners = np.stack([centres + a * tangent + b * normal
                        for a in (shift - HALF_L, shift, shift + HALF_L)
                        for b in (-HALF_W - edge_margin, HALF_W + edge_margin)], axis=1).reshape(-1, 2)
    _, index = corridor.tree.query(corners)
    lateral = np.sum((corners - corridor.route[index]) * corridor.normal[index], axis=1)
    left = np.maximum(0., lateral - corridor.left[index]).reshape(len(centres), -1).max(axis=1)
    right = np.maximum(0., corridor.right[index] - lateral).reshape(len(centres), -1).max(axis=1)
    return left, right


def corridor_violation(corridor, path, horizon, edge_margin=0.):
    """Depth (m) by which the body (+edge_margin) leaves the corridor at each 0.5 m pose."""
    if corridor is None:
        return np.zeros(1), np.zeros(1)
    arc = _arc(path)
    sample = np.arange(0., min(horizon, arc[-1]) + .01, .5)
    centres = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
    tangent, normal = _frame(centres) if len(centres) > 2 else (np.array([[1., 0.]]), np.array([[0., 1.]]))
    shift = float(getattr(corridor, 'body_shift_m', ROAD_LEGACY_SHIFT_M))   # LaneCorridor / ego_body
    corners = np.stack([centres + a * tangent + b * normal
                        for a in (shift - HALF_L, shift, shift + HALF_L)
                        for b in (-HALF_W - edge_margin, HALF_W + edge_margin)], axis=1).reshape(-1, 2)
    _, index = corridor.tree.query(corners)
    lateral = np.sum((corners - corridor.route[index]) * corridor.normal[index], axis=1)
    depth = np.maximum(lateral - corridor.left[index], corridor.right[index] - lateral)
    return sample, np.maximum(0., depth.reshape(len(centres), -1).max(axis=1))


def plan(route, ego, static_points, static_objects, corridor, previous=None,
         model_path=None, cruise_mps=10., max_curvature=.18, now=0., legal_horizon_m=None,
         outside_max_m=None, edge_margin_m=0., extra_clearance=None, road_corridor=None,
         extra_speed_cap=None,
         pedestrian_inflate_m=None, right_shoulder_m=None, wide_side_m=None, w_lateral=None,
         side_margin_m=None, blocked_standoff_m=None, max_offset_m=3.5, base_offset=None,
         base_label="racing line", join_alat_mps2=None, anchor_offsets=None, anchor_same_line=False,
         gentle_side=None, side_commit_s=None, wait_for_pedestrian=False, paved_road_only=False,
         path_transform=None, candidate_targets=None, candidate_lengths=None,
         reuse_first=False, candidate_quality=None, reuse_min_quality=0., body_shift_m=ROAD_LEGACY_SHIFT_M,
         deadline=None, slalom_first=False, geometry_speed_mps=None, required_horizon_m=None):
    """Return the chosen path (vehicle frame), its speed cap and diagnostics.

    body_shift_m: body-box centre ahead of each path sample in the swept checks
    (tunnel: ego_body.REAR_AXLE_SHIFT_M, the real body on the rear-axle pose; the
    road default keeps the legacy box centred on the pose). Corridor corners use
    the corridor's own body_shift_m.

    outside_max_m: how far the body may leave the corridor (default: the
    road allowance OUTSIDE_LEGAL_MAX_M). Tunnel walls are physical: 0 there,
    with edge_margin_m of clearance to the wall.

    base_offset: per-route-point lateral offset (m, left +) of a preferred
    line (the offline racing line of the guardrail S-curve). Used only while
    nothing is in the lane (no hold): the "lane centre" candidate then joins
    and follows this line instead of the route, and lateral cost is measured
    from it. The centre-line, road-edge, corridor and collision checks are
    unchanged. With an obstacle in the lane the route is the reference again.
    base_label names that line ("racing line", "guardrail line"); join_alat_mps2
    sets how gently the car joins it (switching lines mid-curve), only for a join
    toward gentle_side (+1 left / -1 right; None: either way).

    anchor_offsets: the previous plan's lateral offsets at these route points (NaN
    where it ended). Following a base_offset line with nothing to avoid, while the car
    is within ANCHOR_MAX_DEV_M of that plan, the plan is kept (anchor_same_line: same
    line, joined by the plan's end) or the join to a new line starts where the plan
    puts the car (lateral and slope at the first route point), so the follower sees its
    own tracking error instead of a path re-anchored at the car every frame.

    paved_road_only: obstacle-mission curb guard. Enables multi-row joins and
    checks every candidate against surveyed pavement, regardless of shoulder
    or stalled-escape allowances. Defaults off for tunnel/highway callers.

    candidate_targets/candidate_lengths: optional bounded proposal set. These
    replace the generated fan, never its curvature, body, wall or traffic checks.
    candidate_quality: measured passage clearance used after the safety checks;
    whole-horizon paths rank before shorter prefixes. reuse_first checks the
    current-pose previous path first and keeps it only with a complete horizon,
    comfort clearance, adequate passage quality and an unrestricted motion cap.
    geometry_speed_mps: optional intended speed for return and slalom shapes;
    measured ego speed still owns every physical admission and braking check.
    required_horizon_m: optional minimum sweep horizon, limited to the supplied
    route, for callers that require a complete pass beyond the speed lookahead.
    """
    # Crossing people are handled by the pedestrian stop guard. Check only a
    # lane-centre path, including real static hazards; never reuse a bypass or
    # accept a model swerve to drive around a person while waiting.
    if wait_for_pedestrian:
        previous, model_path, base_offset, anchor_offsets = None, None, None, None
        candidate_targets, candidate_lengths = None, None
    bounded = candidate_targets is not None or candidate_lengths is not None or candidate_quality is not None or reuse_first
    def proposals(values, name, positive=False):
        values = np.asarray(values, dtype=float).reshape(-1)
        if not len(values) or not np.isfinite(values).all() or (positive and np.any(values <= 0.)):
            raise ValueError(name + " must contain finite" + (" positive" if positive else "") + " values")
        return list(dict.fromkeys(float(value) for value in values))
    outside_max = OUTSIDE_LEGAL_MAX_M if outside_max_m is None else float(outside_max_m)
    side_margin = SIDE_MARGIN_M if side_margin_m is None else float(side_margin_m)
    route = np.asarray(route, dtype=float)[:, :2]
    v = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
    requested_shape_speed = v if geometry_speed_mps is None else float(geometry_speed_mps)
    if not math.isfinite(requested_shape_speed) or requested_shape_speed < 0.:
        raise ValueError('geometry_speed_mps must be finite and nonnegative')
    shape_speed = max(v, requested_shape_speed)
    arc = _arc(route)
    tangent, normal = _frame(route)
    d0 = -float(route[0] @ normal[0])          # ego lateral from the route
    # Car heading relative to the route tangent (vehicle frame: car = +x).
    slope0 = float(np.clip(math.tan(-math.atan2(tangent[0, 1], tangent[0, 0])), -.35, .35))
    horizon = float(min(arc[-1] - 1., max(40., v * 4. + 20.)))
    if required_horizon_m is not None:
        required = float(required_horizon_m)
        if not math.isfinite(required) or required <= 0.:
            raise ValueError('required_horizon_m must be finite and positive')
        horizon = min(float(arc[-1]), max(horizon, required))
    stop_need = v * .4 + v * v / (2. * BRAKE_MPS2) + STANDOFF_M

    pts = np.asarray(static_points, dtype=float).reshape(-1, 2) if static_points is not None else np.zeros((0, 2))
    obj_pts = object_points(static_objects, ego, pedestrian_inflate=PEDESTRIAN_INFLATE_M
                            if pedestrian_inflate_m is None else float(pedestrian_inflate_m))
    if len(obj_pts):
        pts = np.vstack((pts, obj_pts))
    pts = pts[(pts[:, 0] > -HALF_L - 1.) & (pts[:, 0] < horizon + 10.)] if len(pts) else pts
    # Returns already beside the body cannot be hit by driving forward. With
    # the full margin they made every candidate "blocked at 0 m" once the car
    # was alongside a standing pedestrian (live 2026-10-03: parked 80 s).
    near_pts, pts = partition_lattice_points(pts, body_shift_m)
    near_tree = cKDTree(near_pts) if len(near_pts) else None
    tree = cKDTree(pts) if len(pts) else None

    # Obstacle span along the route lane: where to hold an offset and return.
    hold_until = None
    obstacle_close = False
    side_groups = []
    if len(pts):
        _, nearest = cKDTree(route).query(pts)
        lateral = np.sum((pts - route[nearest]) * normal[nearest], axis=1)
        along = arc[nearest]
        # Only returns the car body would sweep: a 27 m structure 1.9-2.3 m
        # beside the tunnel exit lane triggered a needless bypass on recorded
        # scans with the old lane-wide (2.7 m) test.
        band = HALF_W + SIDE_MARGIN_M + .3
        # Also what lies ahead of the car where it actually is: off the route
        # after a bypass, a box 1.7 m right of the route but straight ahead of
        # the car got no avoidance candidates and the car "rejoined" into it
        # (live tunnel 2026-10-04).
        # Sweeping poses through the horizon also checks the front bumper
        # beyond it. Generate obstacle-mission joins for those same returns:
        # otherwise an obstacle 55 m away invalidates the previous path's
        # 53 m sweep, but creates no replacement candidates and forces a stop.
        obstacle_horizon = horizon + HALF_L + float(body_shift_m) + END_MARGIN_M if paved_road_only else horizon
        in_lane = (((np.abs(lateral) < band) | (np.abs(lateral - d0) < band))
                   & (along > 0.) & (along < obstacle_horizon))
        # Something beside or just ahead: changing sides now swings the body
        # across it while the follower still lags (random layout: -3.5 m ->
        # +0.5 m between alternating barrels hit a TrafficBarrel at 18 km/h).
        obstacle_close = bool(((along > -HALF_L) & (along < 15.) & (np.abs(lateral) < 3.5)).any())
        if side_commit_s is not None or paved_road_only:
            # Obstacle-relative sides (tunnel S rows): every obstacle within
            # side_commit_s of travel (>= 15 m), as (along, median lateral) groups.
            close_m = max(15., float(side_commit_s or 0.) * v)
            sel = (along > -HALF_L) & (along < close_m) & (np.abs(lateral) < 3.5)
            if sel.any():
                order = np.argsort(along[sel])
                a_s, l_s = along[sel][order], lateral[sel][order]
                cuts = np.flatnonzero(np.diff(a_s) > 1.5) + 1
                side_groups = [(float(np.median(ga)), float(np.median(gl)))
                               for ga, gl in zip(np.split(a_s, cuts), np.split(l_s, cuts))]
        if in_lane.any():
            first = float(along[in_lane].min())
            span = np.sort(along[in_lane])
            # Connected cluster: gaps under 2 car lengths stay one manoeuvre.
            end = first
            for value in span:
                if value - end > 2 * (2 * HALF_L) + 2.:
                    break
                end = value
            hold_until = end + HALF_L + 2.

    if hold_until is None and extra_speed_cap is not None and not wait_for_pedestrian:
        # A dynamic speed constraint can close the centre path before any
        # measured footprint enters its static band. It still needs the same
        # offset candidates; otherwise the recovery search only retries zero.
        hold_until = horizon
        if len(obj_pts):
            _, nearest = cKDTree(route).query(obj_pts)
            stations = arc[nearest]
            nearby = stations[(stations >= 0.) & (stations <= horizon)]
            if len(nearby):
                hold_until = max(10., float(nearby.max())+HALF_L+2.)

    lengths = sorted({round(max(8., v * 1.6), 1), round(max(12., v * 2.5), 1), round(max(18., v * 3.5), 1)})
    if hold_until is not None and v < 1.5:
        # Creeping or stopped next to the obstacle: full-lock short swerves
        # (user: "turn the wheel to the stop and it gets out"; the 8 m minimum
        # left no candidate after the 5 s wait, live 2026-10-04).
        lengths = sorted(set(lengths) | {4., 6.})
        if extra_speed_cap is not None:
            # A stopped dynamic bypass can fit the steering limit at 5 m,
            # while 4 m is rejected and 6 m reaches side clearance too late.
            lengths = sorted(set(lengths) | {5.})
        if not paved_road_only:
            max_curvature = max(max_curvature, LOW_SPEED_MAX_CURVATURE)
    ref = np.zeros(len(route))
    if base_offset is not None and hold_until is None:
        b = np.asarray(base_offset, dtype=float)[:len(route)]
        ref[:len(b)] = b
        if len(b) < len(route):
            ref[len(b):] = b[-1] if len(b) else 0.
    racing = bool(np.any(np.abs(ref) > 1e-3))
    ref_slope0 = float((ref[1] - ref[0]) / max(arc[1] - arc[0], 1e-6)) if len(ref) > 1 else 0.
    anchored = stitched = None
    if racing and hold_until is None and anchor_offsets is not None:
        a = np.asarray(anchor_offsets, dtype=float)[:len(route)]
        valid = int(np.argmin(np.isfinite(a))) if not np.isfinite(a).all() else len(a)
        if valid >= 3 and abs(float(a[0]) - d0) <= ANCHOR_MAX_DEV_M:
            anchored = (float(a[0]), float(np.clip((a[1] - a[0]) / max(arc[1] - arc[0], 1e-6), -.35, .35)))
            d0, slope0 = anchored
            if anchor_same_line and (valid == len(route) or abs(float(a[valid - 1] - ref[valid - 1])) <= .05):
                stitched = ref.copy()
                stitched[:valid] = a[:valid]
    d0_ref = d0 - float(ref[0])          # car (or plan) offset from the reference line
    if (join_alat_mps2 is not None and gentle_side is not None
            and float(np.sign(-d0_ref)) != float(np.sign(gentle_side))):
        # The gentle join is for moves toward the guardrail; away from it the usual
        # return applies (run 227: +3 m back to the racing line over 40 m at 1.2 m/s^2
        # left the car drifting at the guardrail).
        join_alat_mps2 = None
    # Lane-centre driving (no line): the candidate starts on the lane centre, not at the car.
    cand_d0, cand_slope0 = d0, slope0
    # Not beside or just behind/ahead of an obstacle: snapping to the lane centre while
    # passing a truck swung the car into it (sim studio_box_1, strict lanes).
    if hold_until is None and abs(d0) <= LANE_ANCHOR_M and not obstacle_close and not len(near_pts):
        cand_d0, cand_slope0 = 0., 0.
    if hold_until is None:
        # Nothing to avoid: return to the lane centre on the short join. The
        # gentlest join re-anchored at the car each frame hid a growing offset
        # in a tight turn (CTE 0 for the follower, 2.7 m off the lane, live
        # 2026-10-03 23:50, idx 1126-1180).
        # ...but never shorter than a RETURN_ALAT_MPS2 join for the actual offset:
        # 3.5 m back over 10.7 m at 24 km/h threw the follower into recovery with
        # full lock toward the fence side (auto run 179, user: "curves look odd").
        lengths = [max(lengths[0], comfortable_join_m(d0_ref, v, join_alat_mps2 if racing else None))]
    # Nothing in the lane: only the lane centre (and returning to it). Offsets
    # around roadside returns made the path weave 0 / +-0.5 / -1 m on an
    # empty road (live 2026-10-03), and cost 3x the planning time.
    # A tight-margin retry (bottleneck creep) needs finer offsets: a 2.45 m gap
    # only fits a car centred within +-0.15 m.
    step = .25 if side_margin < SIDE_MARGIN_M - 1e-6 else .5
    # max_offset_m: 3.5 on roads; the 12 m tunnel needs more (a skewed mixer truck
    # needed a 4.2 m bypass, auto run 118). Walls/corridor still veto candidates.
    reach = float(max_offset_m)
    targets = (np.round(np.arange(-reach, reach + .01, step), 2)
               if hold_until is not None and not wait_for_pedestrian else np.array([0.]))
    if candidate_targets is not None:
        targets = proposals(candidate_targets, "candidate_targets")
        if any(abs(value) > reach + 1e-9 for value in targets):
            raise ValueError("candidate_targets exceed max_offset_m")
    if candidate_lengths is not None:
        lengths = proposals(candidate_lengths, "candidate_lengths", positive=True)
    candidates = []
    for target in targets:
        for change_m in lengths:
            d = (stitched if stitched is not None else
                 ref + offset_profile(arc, d0_ref, target, change_m, slope0=slope0 - ref_slope0)
                 if racing else offset_profile(arc, cand_d0, target, change_m, slope0=cand_slope0))
            candidates.append(("lattice", float(target), change_m, None, d))
            if racing:
                # The plain lane-centre join stays available should the racing
                # candidate fail a check (it costs its distance from the line).
                lane_m = max(change_m if abs(d0) < 1e-6 else 0., lengths[0], comfortable_join_m(d0, v))
                candidates.append(("lattice_lane", float(target), lane_m, None,
                                   offset_profile(arc, cand_d0, target, lane_m, slope0=cand_slope0)))
            if hold_until is not None and abs(target) > .01:
                ret = max(change_m, 10., comfortable_join_m(target, shape_speed))
                d_ret = offset_profile(arc, d0, target, change_m, hold_until, ret, slope0=slope0)
                candidates.append(("lattice_return", float(target), change_m, hold_until, d_ret))

    if paved_road_only and hold_until is not None:
        # A junction may open before the obstacle, while the approach remains
        # one narrow lane. Starting every offset at the ego either crosses that
        # lane's line early or reaches the obstacle too late. Keep the approach
        # centred and start the checked turn when the front reaches open space.
        open_space = getattr(corridor, "intersection_mask", np.zeros(len(route), dtype=bool))
        entries = np.flatnonzero(open_space & ~np.r_[False, open_space[:-1]])
        for entry in entries:
            begin = max(0., float(arc[entry]) - HALF_L)
            if begin >= min(horizon, hold_until - HALF_L):
                continue
            approach = offset_profile(arc, d0, 0., max(8., begin), slope0=slope0)
            for target in targets:
                if abs(target) < .01:
                    continue
                for change_m in (12., 18.):
                    d = approach + target * _smooth((arc - begin) / change_m)
                    # Fit the return before the junction narrows again. Its
                    # checked curvature sets the pace, rather than requiring a
                    # long return at the measured approach speed and stopping.
                    ret = max(change_m, 10., comfortable_join_m(target, shape_speed)
                              if geometry_speed_mps is not None else 0.)
                    d *= 1. - _smooth((arc - hold_until) / ret)
                    candidates.append(("intersection", float(target), change_m, hold_until, d))
        # Use the intersection of physical pavement and legal left edge to
        # prune obviously unusable sides before expensive swept checks.
        from copy import copy
        envelope = copy(road_corridor if road_corridor is not None else corridor)
        if envelope is not None and corridor is not None:
            envelope.left = np.minimum(envelope.left, corridor.left)
        row_points = object_points([o for o in static_objects if o[4] == "obstacle"], ego)
        # Normal-road recovery must include every measured row inside this
        # checked horizon. A later, more distant row must not suppress the
        # S path needed to clear the close pair in front of it.
        row_gap = math.inf if slalom_first else 14.
        profiles = slalom_profiles(arc, route, normal, row_points, d0, slope0, shape_speed, side_margin, envelope,
                                  max_row_gap_m=row_gap,
                                  min_first_join_m=4. if paved_road_only and v < 1.5 and ego.get('measured') is True else 8.,
                                  padding_steps=(.10, .15, .05, .25, .5, .75)
                                      if geometry_speed_mps is not None else (.05, .25, .5, .75))
        if not profiles:
            profiles = slalom_profiles(arc, route, normal, pts, d0, slope0, shape_speed, side_margin, envelope,
                                      max_row_gap_m=row_gap,
                                      min_first_join_m=4. if paved_road_only and v < 1.5 and ego.get('measured') is True else 8.,
                                      padding_steps=(.10, .15, .05, .25, .5, .75)
                                          if geometry_speed_mps is not None else (.05, .25, .5, .75))
        candidates = profiles + candidates if slalom_first else candidates + profiles

    prev_target = float(previous.get("target", 0.)) if previous else 0.
    extra = []
    # Anchored on the previous plan, the lattice candidate already continues it.
    if previous is not None and previous.get("world_path") is not None and anchored is None:
        yaw = math.radians(float(ego["heading_deg"]))
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        local = (np.asarray(previous["world_path"]) - [ego["x"], ego["y"]]) @ rot
        k = int(np.argmin(np.linalg.norm(local, axis=1)))
        local = local[k:]
        if len(local) > 10 and _arc(local)[-1] > 15.:
            extra.append(("previous", prev_target, None, None, local))
    # Committed side of each near obstacle: where the previous path actually passes
    # it (its offset from the route at the obstacle), not its final target.
    side_refs = []
    if side_groups and previous is not None:
        prev_local = None
        if previous.get("world_path") is not None:
            yaw = math.radians(float(ego["heading_deg"]))
            rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            prev_local = (np.asarray(previous["world_path"], dtype=float)[:, :2] - [ego["x"], ego["y"]]) @ rot
        for ga, gl in side_groups:
            ref_d = prev_target
            if prev_local is not None and len(prev_local) >= 2:
                i = int(np.clip(np.searchsorted(arc, ga), 0, len(route) - 1))
                j = int(np.argmin(np.linalg.norm(prev_local - route[i], axis=1)))
                if float(np.linalg.norm(prev_local[j] - route[i])) < 6.:
                    ref_d = float((prev_local[j] - route[i]) @ normal[i])
            side_refs.append((ga, gl, ref_d))
    if model_path is not None and len(model_path) > 5:
        mp = np.asarray(model_path, dtype=float)[:, :2]
        if _arc(mp)[-1] > 8.:
            extra.append(("model", float(mp[-1, 1]), None, None, mp))

    best, best_blocked, previous_entry = None, None, None
    route_peak = float(curvature(route[arc <= horizon]).max()) if (arc <= horizon).sum() > 3 else 0.
    if racing and (arc <= horizon).sum() > 3:
        # The reference line's own bends are not "added" curvature (the racing line
        # curves back to the lane centre where the road is already straight). A line
        # 3.5 m inside a bend is up to ~6 m shorter than the route: its horizon reaches a
        # bend the route's does not yet (sim: one frame capped 58 -> 25 km/h at idx 1611).
        within = arc <= horizon + RACING_PEAK_EXTRA_M
        route_peak = max(route_peak, float(curvature(route[within]).max()),
                         float(curvature((route + normal * ref[:, None])[within]).max()))
    evaluated = 0
    rejected = dict(curvature=0, outside=0)
    paved = road_corridor if road_corridor is not None else corridor
    ego_edges = (corridor_side_violation(paved, np.array([[0., 0.], [1., 0.], [2., 0.]]), 0.)
                 if paved_road_only and paved is not None else None)
    base_road_right = (corridor_side_violation(road_corridor, route, horizon)[1]
                       if road_corridor is not None else np.zeros(0))
    centre_line = None
    if corridor is not None:
        # Keep the yellow-line limit through incomplete lane geometry. In the
        # obstacle mission, surveyed junction links explicitly allow open-space
        # paths; do not carry the approaching lane's line across that junction.
        # Physical pavement and swept-obstacle checks remain mandatory there.
        from copy import copy
        centre_line = copy(corridor)
        start = int(corridor.tree.query([0., 0.])[1])
        left = np.asarray(corridor.left, dtype=float).copy()
        open_space = (getattr(corridor, "intersection_mask", np.zeros(len(left), dtype=bool))
                      if paved_road_only else np.zeros(len(left), dtype=bool))
        closed = np.flatnonzero(~open_space[start:]) + start
        for segment in np.split(closed, np.flatnonzero(np.diff(closed) > 1) + 1):
            if len(segment):
                left[segment] = np.minimum.accumulate(left[segment])
        centre_line.left = left
    previous_first = paved_road_only or reuse_first
    committed = [item for item in extra if item[0] == "previous"] if previous_first else []
    choices = committed + candidates + [item for item in extra
                                        if not (previous_first and item[0] == "previous")]
    for kind, target, change_m, hold, data in choices:
        if deadline is not None and time.perf_counter() >= deadline:
            rejected['budget_exhausted'] = 1
            break
        centre_penalty = 0.
        if kind in ("previous", "model"):
            path = data
        else:
            path = route + normal * data[:, None]
        if path_transform is not None:
            # Validate the exact geometry that the consumer will publish.
            # Changing a checked path's origin afterward can rotate the rear
            # corner into a wall even when the original candidate was clear.
            path = np.asarray(path_transform(path), dtype=float)[:, :2]
            if len(path) < 3 or not np.isfinite(path).all():
                continue
        evaluated += 1
        curv = curvature(path[_arc(path) <= horizon])
        peak = float(curv.max()) if len(curv) else 0.
        if peak > max_curvature:
            rejected["curvature"] += 1
            continue
        # A change the follower can track: at 34 km/h the 15 m candidate
        # moved 3.5 m at ~8 m/s^2 lateral and the car wobbled. Kept only as
        # a fallback when nothing gentler is collision-free.
        harsh = max(v, 3.) ** 2 * max(0., peak - route_peak) > LAT_ACCEL_MAX_MPS2
        sample, depth = corridor_violation(corridor, path,
                                           horizon if legal_horizon_m is None else min(horizon, max(5., legal_horizon_m)),
                                           edge_margin=edge_margin_m)
        if paved_road_only:
            # The lane-map union is the surveyed paved boundary. An invented
            # shoulder must not widen it, including during a stalled retry or
            # reuse of a previous/model path. If the map is absent, use the legal
            # corridor conservatively; do not approve an unbounded bypass.
            if paved is None:
                rejected["road_edge"] = rejected.get("road_edge", 0) + 1
                continue
            for edge, ego_edge in zip(corridor_side_violation(paved, path, horizon), ego_edges):
                initial = min(float(edge[0]), float(ego_edge[0]))
                recovery = initial * np.clip(1. - np.arange(len(edge)) * .5 / 6., 0., 1.)
                if np.any(edge > recovery + min(.05, max(0., ROAD_EDGE_TOLERANCE_M))):
                    break
            else:
                edge = None
            if edge is not None:
                rejected["road_edge"] = rejected.get("road_edge", 0) + 1
                continue
        # With the road extent known, the centre line and road edge below
        # bound the path; leaving the legal lane to the right is only a cost.
        if (road_corridor is None or outside_max <= 0.) and float(depth.max()) > outside_max + max(0., float(depth[0])):
            rejected["outside"] += 1
            continue
        if road_corridor is not None and outside_max > 0.:
            # Left of the legal lanes is the oncoming lane (yellow centre
            # line): never cross it. To the right only the paved shoulder
            # beyond the mapped lanes may be used (user rule 2026-10-03: a box
            # in the lane, free space on the right, stuck 25 s at idx 575).
            # Last resort only: a huge cost, chosen only when no path stays on
            # this side (otherwise the car would stand forever).
            legal_left, _ = corridor_side_violation(centre_line, path, horizon)
            crossing = float(legal_left.max()) - float(legal_left[0])
            if crossing > ROAD_EDGE_TOLERANCE_M:
                rejected["centre_line"] = rejected.get("centre_line", 0) + 1
                if v > 8. / 3.6:
                    # Moving: brake instead; at low speed the shorter right-side
                    # candidates become feasible (live 23:23: crossed the
                    # yellow line at 39 km/h and wedged in front of a barrel).
                    continue
                centre_penalty = W_CENTRE_LINE * (1. + crossing)
            road_left, road_right = corridor_side_violation(road_corridor, path, horizon)
            n = min(len(road_right), len(base_road_right))
            shoulder = RIGHT_SHOULDER_M if right_shoulder_m is None else float(right_shoulder_m)
            allowed = np.maximum(base_road_right[:n], float(road_right[0])) + shoulder
            beyond = float(np.max(road_right[:n] - allowed)) if n else 0.
            if beyond > 0.:
                rejected["road_edge"] = rejected.get("road_edge", 0) + 1
                if beyond > (SHOULDER_LAST_RESORT_M if shoulder > 0. else 0.):
                    continue
                # Past the usual shoulder only when nothing else is clear
                # (user rule 2026-10-04); still cheaper than the centre line.
                centre_penalty += W_SHOULDER_BEYOND * (1. + beyond)
        margins = [side_margin, COMFORT_SIDE_M] + ([float(wide_side_m)] if wide_side_m is not None else [])
        multi = (swept_clearances(path, pts, horizon, END_MARGIN_M, margins, tree=tree, body_shift_m=body_shift_m)
                 if tree is not None else [horizon] * len(margins))
        clear = multi[0]
        if near_tree is not None:
            clear = min(clear, swept_clearance(path, near_pts, horizon, END_MARGIN_M, tree=near_tree,
                                               lateral_margin=LATTICE_NEAR_SIDE_MARGIN_M, body_shift_m=body_shift_m))
        if paved_road_only and kind == "previous" and clear < horizon - .1:
            # Reuse is allowed only after checking the whole current horizon.
            # A newly blocked suffix must trigger a fresh join, not inherit the
            # previous path's target or its hysteresis bonus.
            rejected["previous_collision"] = rejected.get("previous_collision", 0) + 1
            continue
        if (not paved_road_only and obstacle_close and v > 10. / 3.6 and abs(prev_target) > .3 and abs(target) > .3
                and np.sign(target) != np.sign(prev_target)):
            # Do not swing across an obstacle beside us at speed: brake on the
            # committed side instead and re-plan slower.
            clear = min(clear, 0.)
        p_arc = _arc(path)
        comfortable = multi[1]
        # Prefer the wide side of an obstacle where there is room (tunnel:
        # wall-to-wall space; it kept threading the narrow gap, live 2026-10-04).
        wide = multi[2] if wide_side_m is not None else horizon
        offsets = (data if kind not in ("previous", "model") else None)
        side_penalty = 0.
        if side_refs and offsets is not None:
            # Runs 222/225/271: the S row in the tunnel. +3.5 -> +0.5 kept the sign of
            # the target (no "side switch") but crossed to the right of the next drum
            # 14 m ahead at 39 km/h; with the follower trailing, the car hit it. Never
            # change which side of a near obstacle we pass while moving; brake on the
            # committed side instead (the wide re-plan then finds the room).
            for ga, gl, ref_d in side_refs:
                if abs(ref_d - gl) <= .3:
                    continue
                d_at = float(np.interp(ga, arc, offsets))
                if abs(d_at - gl) > .3 and np.sign(d_at - gl) != np.sign(ref_d - gl):
                    side_penalty = W_SIDE_SWITCH
                    if v > 10. / 3.6:
                        clear = min(clear, 0.)
                    break
        mean_offset = (float(np.mean(np.abs((offsets - ref)[arc <= horizon]))) if offsets is not None
                       else float(np.mean(np.abs(path[p_arc <= horizon][:, 1]))))
        if paved_road_only and offsets is None:
            future = path[p_arc <= horizon]
            _, at = cKDTree(route).query(future)
            mean_offset = float(np.mean(np.abs(np.sum((future - route[at]) * normal[at], axis=1))))
        cost = ((W_LATERAL if w_lateral is None else float(w_lateral)) * mean_offset
                + W_OUTSIDE * float(np.mean(depth))
                + W_SWITCH * abs(target - prev_target)
                + (W_SIDE_SWITCH if not paved_road_only and obstacle_close and abs(prev_target) > .3 and abs(target) > .3
                   and np.sign(target) != np.sign(prev_target) else 0.)
                + side_penalty
                + W_BLOCKED * max(0., horizon - clear)
                + W_CURVATURE * peak
                + W_PROXIMITY * float(comfortable < clear)
                + W_WIDE * float(wide < clear)
                + centre_penalty)
        if kind == "previous":
            cost -= .5           # mild preference to keep the committed path
        entry = dict(kind=kind, target=target, change_m=change_m, hold_until=hold, path=path,
                     offsets=offsets,
                     clear=float(clear), cost=float(cost), curvature=peak,
                     outside_m=float(depth.max()))
        if harsh and kind not in ("previous", "model"):
            entry["cost"] += W_HARSH
        if kind == "lattice_lane":
            entry["cost"] += W_HARSH      # only when the reference-line candidate is rejected
        if racing:
            # Compare with the penalties included. (Elsewhere the comparison below
            # still uses the unpenalised cost, as before: unchanged behaviour.)
            cost = entry["cost"]
        if paved_road_only:
            cost = entry["cost"]
        # Small encroachment ranks ahead of a large detour. Use half-metre
        # bands to avoid trading useful collision clearance for millimetres.
        entry["selection"] = (math.ceil(max(0., entry["outside_m"] - .05) / .5), cost)
        prefer = (best is None or (entry["selection"] < best["selection"]
                                  if paved_road_only else cost < best["cost"]))
        need = min(horizon, stop_need + 2. * HALF_L)
        if (extra_clearance is not None and
                (candidate_quality is not None or reuse_first or
                 (clear >= need and (prefer or kind == "previous")) or
                 (clear < need and (best_blocked is None or clear > best_blocked["clear"])))):
            # The tunnel's physical wall check costs ~10 ms per path: run it
            # only for a candidate that could be chosen (all 90 took 0.9 s per
            # decision in the live tunnel, 2026-10-04).
            before_wall = clear
            wall_clear = float(extra_clearance(path, horizon))
            if bounded and math.isnan(wall_clear):
                wall_clear = 0.
            clear = min(clear, wall_clear)
            entry["clear"] = float(clear)
            # Score the checked path, including any prefix lost to physical
            # tunnel walls. Otherwise an optimistic obstacle-only score can
            # replace a longer safe pass with one ending at a wall 10 m ahead.
            entry["cost"] += W_BLOCKED * max(0., before_wall - clear)
            if extra_speed_cap is not None:
                motion_cap = max(0., float(extra_speed_cap(path, horizon)))
                entry['motion_cap_mps'] = motion_cap
                # Compare time to clear the same stopping/body horizon at each
                # checked pace. A fixed two-second progress score underweighted
                # a crawling centre line, so its style preference defeated an
                # already collision/steering-checked low-speed micro-bypass.
                motion_distance = min(horizon, stop_need+2.*HALF_L)
                delay = (motion_distance/max(.05, motion_cap)-
                         motion_distance/max(.25, cruise_mps))
                entry['cost'] += W_BLOCKED*2.*min(60., max(0., delay))
                if harsh and kind not in ('previous', 'model') and v < 1. and motion_cap > 0.:
                    # At crawling speed a short, turnable checked recovery is
                    # useful. Its cruise-style detour penalty otherwise selects
                    # a slow path edging toward the person instead of the open
                    # side. Hard steering/body/road checks and the cap remain.
                    entry['cost'] -= W_HARSH
            cost = entry["cost"]
            entry["selection"] = (entry["selection"][0], cost)
            prefer = (best is None or (entry["selection"] < best["selection"]
                                      if paved_road_only else cost < best["cost"]))
        if (extra_speed_cap is not None and 'motion_cap_mps' not in entry
                and (candidate_quality is not None or reuse_first)):
            entry['motion_cap_mps'] = max(0., float(extra_speed_cap(path, horizon)))
        quality = 0.
        if candidate_quality is not None:
            quality = float(candidate_quality(path, horizon))
            quality = quality if math.isfinite(quality) else 0.
            entry['passage_quality_m'] = quality
            if quality >= 2.*(HALF_W+COMFORT_SIDE_M):
                # Once the measured passage supplies comfortable room on
                # both sides, avoid paying the old 1.6 m "wide" surcharge.
                # Otherwise a tiny obstacle still selects a 2.6 m detour even
                # when a gentler fully checked nudge supplies enough room.
                entry['cost'] -= W_WIDE * float(wide < clear)
            # Compare every small proposal after the same current-source
            # checks. A short safe prefix must not hide a complete wide pass.
            entry['selection'] = (entry['selection'][0], int(clear < horizon-.1),
                int(entry.get('motion_cap_mps', cruise_mps) <= .05),
                -math.floor(clear/.5),
                -math.floor(min(max(0., quality), 2.*(HALF_W+COMFORT_SIDE_M))/.25), entry['cost'])
            prefer = best is None or entry['selection'] < best['selection']
        if (reuse_first and kind == 'previous' and
                (clear < horizon-.1 or p_arc[-1] < horizon-.1)):
            rejected['previous_collision'] = rejected.get('previous_collision', 0)+1
            continue
        if clear >= need:
            if kind == "previous":
                previous_entry = entry
            if prefer:
                best = entry
            if (candidate_quality is not None and kind == 'lattice' and abs(target) < .01
                    and clear >= horizon-.1 and comfortable >= horizon-.1
                    and p_arc[-1] >= horizon-.1
                    and (extra_speed_cap is None or entry.get('motion_cap_mps', 0.) >= cruise_mps-.1)):
                # Current-source body, walls, steering and motion checks all
                # approve the route. More room beside it does not warrant a
                # new swerve on an empty tunnel or after an obstacle is passed.
                best = entry
                previous_entry = None
                break
            if (paved_road_only and kind == "previous" and clear >= horizon - .1
                    and entry["outside_m"] <= ROAD_EDGE_TOLERANCE_M
                    and (not reuse_first or (comfortable >= horizon-.1 and quality >= float(reuse_min_quality)))
                    and (extra_speed_cap is None or entry.get('motion_cap_mps', 0.) >= cruise_mps-.1)):
                # Continue a legal committed bypass only after rechecking every
                # current obstacle, body corner and map boundary. A new blocker
                # rejects it above and triggers the complete candidate search.
                best = entry
                break
            if (reuse_first and kind == 'previous' and clear >= horizon-.1
                    and p_arc[-1] >= horizon-.1 and comfortable >= horizon-.1
                    and entry['outside_m'] <= max(0., float(depth[0]))+ROAD_EDGE_TOLERANCE_M
                    and quality >= float(reuse_min_quality)
                    and (extra_speed_cap is None or entry.get('motion_cap_mps', 0.) >= cruise_mps-.1)):
                best = entry
                break
        elif best_blocked is None or (clear, -cost) > (best_blocked["clear"], -best_blocked["cost"]):
            best_blocked = entry

    if (best is not None and previous_entry is not None and best is not previous_entry
            and previous_entry["clear"] >= horizon - .1
            and (not paved_road_only or previous_entry["selection"][0] <= best["selection"][0])
            and (candidate_quality is None or previous_entry['selection'][:-1] <= best['selection'][:-1])
            and best["cost"] > previous_entry["cost"] - SWITCH_HYSTERESIS):
        # Hysteresis only while the current course is clear for the whole
        # horizon; a newly seen obstacle on it must not delay the change
        # (virtual chicane: kept +1.5 m until no avoidance was left).
        best = previous_entry
    blocked = best is None
    chosen = best if best is not None else best_blocked
    if chosen is None:
        # Nothing feasible at all (every path beyond steering limits): drive the
        # route and brake for whatever is on it.
        path = route
        clear = (swept_clearance(route, pts, horizon, END_MARGIN_M, tree=tree, lateral_margin=side_margin,
                                 body_shift_m=body_shift_m)
                 if tree is not None else horizon)
        if paved_road_only or bounded:
            # No tested path exists. Do not publish a positive speed on an
            # unchecked route that could cross the curb. Retry from a stop.
            clear = 0.
        _, route_depth = (corridor_violation(corridor, path, horizon, edge_margin=edge_margin_m)
                          if paved_road_only else (None, np.zeros(1)))
        chosen = dict(kind="route", target=0., change_m=None, hold_until=None, path=path, offsets=None,
                      clear=float(clear), cost=float("inf"), curvature=0., outside_m=float(route_depth.max()))
    clear = chosen["clear"]
    standoff = STANDOFF_M if not blocked or blocked_standoff_m is None else float(blocked_standoff_m)
    brake_cap = math.sqrt(max(0., 2. * BRAKE_MPS2 * max(0., clear - standoff)))
    # Road curvature is the follower's job (its curve-speed lookahead); cap
    # only the curvature an avoidance offset adds. Capping the route's own
    # curve 40 m early slowed an empty road to 20 km/h (virtual run).
    added = max(0., chosen["curvature"] - route_peak)
    curve_cap = math.sqrt(ALAT_MPS2 / max(added, 1e-3)) if added > .01 else cruise_mps
    speed_cap = min(cruise_mps, brake_cap if clear < horizon else cruise_mps, curve_cap,
                    chosen.get('motion_cap_mps', cruise_mps))
    emergency = blocked and clear <= max(STANDOFF_M, v * .3 + v * v / 6. + 1.5)
    on_racing = (racing and not blocked and chosen["kind"] in ("lattice", "previous")
                 and abs(chosen["target"]) < .01)
    if chosen["kind"] == "lattice_lane":
        chosen["kind"] = "lattice"
    reason = ("no collision-free path; braking for obstacle" if blocked else
              str(base_label) if on_racing else
              "lane centre" if chosen["kind"] in ("lattice", "previous") and abs(chosen["target"]) < .01 else
              f"{chosen['kind']} offset {chosen['target']:+.1f} m")
    if wait_for_pedestrian:
        chosen["kind"] = "pedestrian_wait"
        reason = "waiting for pedestrian" + ("; braking for static obstacle" if blocked else "")
    return dict(path=chosen["path"], target=chosen["target"], kind=chosen["kind"],
                clear_m=float(clear), blocked=bool(blocked), emergency=bool(emergency),
                speed_cap_mps=float(speed_cap), curvature=chosen["curvature"],
                outside_m=chosen["outside_m"], hold_until=chosen["hold_until"],
                evaluated=evaluated, rejected=rejected, horizon_m=horizon, reason=reason,
                anchor_m=None if anchored is None else round(float(anchored[0]), 2),
                stitched=stitched is not None, offsets=chosen.get("offsets"),
                obstacle_points=int(len(pts)), racing=bool(on_racing and base_label == "racing line"),
                near_obstacle_points=int(len(near_pts)), side_margin_m=side_margin, end_margin_m=END_MARGIN_M,
                base_line=str(base_label) if on_racing else None, reused=chosen['kind']=='previous')
