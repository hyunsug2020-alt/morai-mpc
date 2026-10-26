"""Model-based avoidance for the Claude working copy (STUDENT_PLANNER=model,
STUDENT_TUNNEL_POLICY=model). The live student_v3 never imports this module.

Glue between the host planner (gui.py, planning/tunnel/runtime.py) and two
external planner/model integrations:
  * tunnel_static_portable/  static-obstacle bypass and the tunnel route / model /
                             lane-hold policies (from 신호등_정적_터널.zip)
  * v6/                      the v6.3 judgement model, run in a child process
                             (v6_ext_child.py adds the path and pose it predicted from)

Rules (user 2026-10-10):
  * Moving things keep the host's existing yield / stop logic (stop, then go).
    Only what the host's LiDAR / fusion motion evidence calls stationary reaches
    the bypass planner, with its speed set to zero.
  * The v6 model chooses the bypass side once the package itself sees a static
    obstacle on the route; with no confirmed obstacle the model cannot move the car
    sideways. A wrong side cannot park the car: the planner retries both sides.
  * Every path still passes the host's footprint, lane, wall and LiDAR checks.
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import numpy as np

import tunnel_static_portable as portable
from ego_body import REAR_AXLE_SHIFT_M, ROAD_LEGACY_SHIFT_M
from path_safety import FUSED_STATIC_MPS
from perception.obstacle_geometry import (FUSION_OBSTACLE_MIN_DIMENSION_M,
                                          FUSION_OBSTACLE_DIMENSION_PADDING_M)
from perception.stationary_semantics import metadata as object_metadata, with_metadata
from tunnel_static_portable import geometry as _geometry
from tunnel_static_portable import static_obstacle as _static

ACTION_LONG = {"KEEP": "KEEP_LANE", "LEFT": "LEFT_CHANGE", "RIGHT": "RIGHT_CHANGE", "TL": "TURN_LEFT",
               "TR": "TURN_RIGHT", "STOP": "STOP", "YIELD": "YIELD", "UNKNOWN": "UNKNOWN"}
SIDE_OF_ACTION = {"LEFT_CHANGE": 1., "RIGHT_CHANGE": -1.}
V6_MAX_AGE_S = float(os.environ.get("STUDENT_V6_MAX_AGE_S", ".3"))
V6_MIN_P = float(os.environ.get("STUDENT_V6_MIN_P", ".5"))
# moving_conflict ignores non-pedestrians at or below 1.0 m/s, so a lower static
# threshold would leave slow obstacles handled by neither side. The published-path
# check uses the same value, so it cannot stop for what this bypasses as static.
OBSTACLE_STATIC_MPS = FUSED_STATIC_MPS
CAR_STATIC_MPS = .5
PLAN_BUDGET_S = float(os.environ.get("STUDENT_MODEL_PLAN_BUDGET_S", ".09"))
PLAN_EXTRA_S = .06              # once, while no candidate has passed the lane check yet
LATERAL_WITHOUT_HAZARD_M = .5   # a v6 path may not leave the route by more without an obstacle
TUNNEL_LANE_M = 3.5             # tunnel "offsets" are free widths up to the wall; one lane is enough
TUNNEL_MIN_LANE_M = 2.2
ROAD_STATIC_MARGIN_M = .25
PORTABLE_STATIC_MARGIN_M = .2
FAST_STATIC_MARGIN_M = .6
ROAD_STATIC_PASS_MPS = 25./3.6
ROAD_STATIC_FAST_MPS = 30./3.6


def _current_producer_geometry(obj, ego, geometry=None):
    """Revalidate the atomic measurement against this row and measured clock."""
    from perception.static_geometry_contract import validate_static_geometry
    if ego.get('measured') is not True:
        return None
    row = list(obj)
    if geometry is not None:
        try:
            row[6:8] = geometry['source_size_m']
            row[2] = max(row[6:8])
        except (KeyError, TypeError, ValueError, IndexError):
            return None
    return validate_static_geometry(object_metadata(obj).get('producer_static_geometry'),
                                    row, ego.get('stamp'))


def validate_road_geometry_sources(objects, ego):
    """Revoke changed/stale proof before static classification zeros motion."""
    return [with_metadata(obj, producer_static_geometry=None)
            if object_metadata(obj).get('producer_static_geometry') is not None
            and not _current_producer_geometry(obj, ego) else obj for obj in objects]


def road_lidar_objects(points, ego):
    """Road boxes and raw returns share one 25 cm clearance budget.

    The portable SAT adds 20 cm itself. Its measured-return boxes therefore
    add only the remaining 5 cm, rather than another full voxel allowance.
    Raw returns retain the independent 25 cm swept-body check.
    """
    from tunnel_runtime import lidar_objects
    return lidar_objects(points, ego,
                         bounds_padding_m=ROAD_STATIC_MARGIN_M-PORTABLE_STATIC_MARGIN_M)


def road_static_objects(objects, ego, points, *, allow_producer_geometry=False):
    """Share the road's static margin with verified stationary fusion boxes.

    Only semantic_fusion's generic obstacle geometry follows this contract.
    Keep its physical-size prior and every associated raw return, replacing
    the source padding with the road allowance left after portable SAT. This
    runs before normal-road static memory, so a near-field dropout cannot
    restore the source's duplicate padding. Pose-specific margin recovery is
    not retained in that memory. Raw swept-body and map checks are independent.

    The metadata records the source dimensions and applied padding. Repeated
    calls, other geometry corrections, unknown dimensions and missing raw
    evidence cannot trigger another reduction.
    """
    result = list(objects)
    try:
        cloud = np.asarray(points, dtype=float)
        pose = np.array([ego['x'], ego['y'], ego['heading_deg']], float)
        if (cloud.ndim != 2 or cloud.shape[1] != 2
                or not np.isfinite(cloud).all() or not np.isfinite(pose).all()):
            return road_static_margin_recovery(result, ego, points)
    except (KeyError, TypeError, ValueError, OverflowError):
        return road_static_margin_recovery(result, ego, points)
    angle = math.radians(pose[2])
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    world_points = cloud @ rotation.T + pose[:2]
    target_padding = 2.*max(0., ROAD_STATIC_MARGIN_M-PORTABLE_STATIC_MARGIN_M)
    for index, obj in enumerate(result):
        try:
            if (len(obj) < 9 or obj[3] != 'fusion_obstacle' or obj[4] != 'obstacle'
                    or abs(float(obj[5])) >= .2):
                continue
            shape = object_metadata(obj)
            if 'road_static_geometry' in shape:
                continue
            size = np.asarray(obj[6:8], float)
            centre = np.asarray(obj[:2], float)
            heading = float(obj[8])
            source_padding = float(shape.get('fusion_padding_m',
                                            FUSION_OBSTACLE_DIMENSION_PADDING_M))
            if (not np.isfinite(np.r_[size, centre, heading, obj[5], source_padding]).all()
                    or source_padding != FUSION_OBSTACLE_DIMENSION_PADDING_M
                    or source_padding <= target_padding
                    or np.any(size < FUSION_OBSTACLE_MIN_DIMENSION_M+source_padding-1e-6)):
                continue
            source_size = np.asarray(shape.get('raw_size_m', size), float)
            if source_size.shape != (2,) or not np.allclose(source_size, size, rtol=0., atol=1e-6):
                continue  # Another correction already changed the source geometry.
            yaw = math.radians(heading)
            axes = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
            relative = (world_points-centre) @ axes
            observed = relative[(np.abs(relative) <= size/2.+1e-6).all(axis=1)]
            producer_proof = bool(allow_producer_geometry and
                _current_producer_geometry(obj, ego))
            if not len(observed) and not producer_proof:
                continue
            physical = np.maximum(size-source_padding, FUSION_OBSTACLE_MIN_DIMENSION_M)
            # Downsampled raw points can lie slightly beyond source bounds.
            # Grow the physical estimate to enclose them; never crop them out.
            if len(observed):
                physical = np.maximum(physical, 2.*np.max(np.abs(observed), axis=0))
            checked_size = physical+target_padding
            if np.any(checked_size > size+1e-6):
                continue
            row = list(obj)
            row[6], row[7] = map(float, checked_size)
            row[2] = max(row[6:8])
            result[index] = with_metadata(row, road_static_geometry=dict(
                version=1, source='semantic_fusion', source_size_m=size.tolist(),
                source_padding_m=source_padding, applied_padding_m=target_padding,
                physical_size_m=physical.tolist(), observed_points=len(observed),
                producer_geometry=producer_proof))
        except (TypeError, ValueError, IndexError, OverflowError):
            continue
    return road_static_margin_recovery(result, ego, points)


def road_static_margin_recovery(objects, ego, points, *, recovery_ledger=None, read_only=False):
    """Recompute only current-pose margin recovery for verified road geometry.

    A retained physical box can remain beside the car after its low returns
    disappear. Its calibration survives that blind spot, but the body gap
    and restoration distance must be checked again at the current pose. No dimensions,
    observation times or raw/map/SAT checks are changed by this helper. The
    ordinary-road caller may retain one nonrenewable tracking allowance per
    UID; a fresh guard may consult those anchors without updating them.
    """
    from ego_body import body_gap, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M, EGO_REAR_M
    from perception.static_geometry_contract import STORED_PROOF_KEY, validate_stored_static_geometry
    result = list(objects)
    def ledger_uid(obj):
        try:
            value = obj[11]
            if (not isinstance(value, (bool, np.bool_)) and math.isfinite(float(value))
                    and int(value) == value and 0 <= int(value) < 2**31):
                return int(value)
        except (TypeError, ValueError, IndexError, OverflowError):
            pass
        return None
    uid_counts = {}
    if isinstance(recovery_ledger, dict):
        for obj in result:
            uid = ledger_uid(obj)
            if uid is not None:
                uid_counts[uid] = uid_counts.get(uid, 0)+1
    ready = False
    try:
        cloud = np.asarray(points, dtype=float)
        pose = np.array([ego['x'], ego['y'], ego['heading_deg']], float)
        ready = bool(cloud.ndim == 2 and cloud.shape[1] == 2 and len(cloud)
            and np.isfinite(cloud).all() and np.isfinite(pose).all()
            and body_gap(cloud) >= .35)
        angle = math.radians(pose[2])
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
    except (KeyError, TypeError, ValueError, OverflowError):
        pass
    padding = 2.*max(0., ROAD_STATIC_MARGIN_M-PORTABLE_STATIC_MARGIN_M)
    for index, obj in enumerate(result):
        geometry = object_metadata(obj).get('road_static_geometry')
        if not isinstance(geometry, dict):
            continue
        current = {k: v for k, v in geometry.items() if k not in
                   ('initial_margin_floor_m', 'initial_margin_restore_start_m')}
        if any(key in geometry for key in
               ('initial_margin_floor_m', 'initial_margin_restore_start_m')):
            result[index] = with_metadata(obj, road_static_geometry=current)
        try:
            observed = float(current.get('observed_points', 0))
            producer_verified = bool(current.get('producer_geometry') is True
                and (_current_producer_geometry(obj, ego, current)
                     or (ego.get('measured') is True
                         and validate_stored_static_geometry(
                             object_metadata(obj).get(STORED_PROOF_KEY), obj,
                             ego.get('stamp')) is not None)))
            if (not ready or obj[3] != 'fusion_obstacle' or obj[4] != 'obstacle'
                    or abs(float(obj[5])) >= .2 or current.get('version') != 1
                    or current.get('source') != 'semantic_fusion'
                    or not math.isfinite(observed)
                    or (observed < 1. and not producer_verified)):
                continue
            size, physical, source = (np.asarray(values, float) for values in
                (obj[6:8], current['physical_size_m'], current['source_size_m']))
            if any(values.shape != (2,) for values in (size, physical, source)):
                continue
            centre, heading = np.asarray(obj[:2], float), float(obj[8])
            if (not np.isfinite(np.r_[size, physical, source, centre, heading, obj[5]]).all()
                    or float(current['source_padding_m']) != FUSION_OBSTACLE_DIMENSION_PADDING_M
                    or not math.isclose(float(current['applied_padding_m']), padding, abs_tol=1e-6)
                    or np.any(physical < FUSION_OBSTACLE_MIN_DIMENSION_M-1e-6)
                    or np.any(source < FUSION_OBSTACLE_MIN_DIMENSION_M+
                              FUSION_OBSTACLE_DIMENSION_PADDING_M-1e-6)
                    or np.any(size > source+1e-6)
                    or not np.allclose(size, physical+padding, rtol=0., atol=1e-6)):
                continue
            local = (centre-pose[:2]) @ rotation - [REAR_AXLE_SHIFT_M, 0.]
            yaw = math.radians(heading-pose[2])
            axis = np.array([math.cos(yaw), math.sin(yaw)])
            cross = np.array([-axis[1], axis[0]])
            hl, hw = size/2.
            gap = max(abs(local[0])-EGO_HALF_LENGTH_M-hl*abs(axis[0])-hw*abs(cross[0]),
                abs(local[1])-EGO_HALF_WIDTH_M-hl*abs(axis[1])-hw*abs(cross[1]),
                abs(local @ axis)-EGO_HALF_LENGTH_M*abs(axis[0])-EGO_HALF_WIDTH_M*abs(axis[1])-hl,
                abs(local @ cross)-EGO_HALF_LENGTH_M*abs(cross[0])-EGO_HALF_WIDTH_M*abs(cross[1])-hw)
            uid = ledger_uid(obj) if isinstance(recovery_ledger, dict) else None
            if uid is not None and uid_counts[uid] != 1:
                continue
            if uid is not None and gap >= .2 and not read_only:
                recovery_ledger.pop(uid, None)
            if .12 <= gap < .2:
                floor = gap
                if uid is not None:
                    if (uid not in recovery_ledger and len(recovery_ledger) < 64 and not read_only
                            and observed == 0. and producer_verified
                            and not isinstance(current.get('observed_points'), (bool, np.bool_))):
                        # The SAT tie tolerance is 0.1 mm: together these
                        # permit at most 5 mm per episode, never per frame.
                        recovery_ledger[uid] = max(.12, gap-.0049)
                    if uid in recovery_ledger:
                        floor = float(recovery_ledger[uid])
                        if (not math.isfinite(floor) or not .12 <= floor < .2
                                or gap < floor-.0001):
                            # Do not fall back to a new exact floor: that
                            # would replenish the spent approach allowance.
                            continue
                result[index] = with_metadata(obj, road_static_geometry=dict(current,
                    initial_margin_floor_m=floor,
                    initial_margin_restore_start_m=max(0., local[0]+REAR_AXLE_SHIFT_M+
                        hl*abs(axis[0])+hw*abs(cross[0])+EGO_REAR_M+.2)))
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            continue
    return result


def road_comfort_objects(objects):
    """Preferred road clearance for compact raw clusters before fusion arrives.

    The hard boxes remain measured geometry. Only the optional comfortable
    search uses fusion's compact physical prior; its failure leaves the
    ordinary fully checked fallback available. Semantic boxes, people, motion
    and long/thin structures keep their existing geometry.
    """
    result = list(objects)
    minimum = FUSION_OBSTACLE_MIN_DIMENSION_M + 2.*(
        ROAD_STATIC_MARGIN_M-PORTABLE_STATIC_MARGIN_M)
    for index, obj in enumerate(result):
        try:
            name = str(obj[3])
            if (not name.startswith('tunnel_lidar_') or not name[len('tunnel_lidar_'):].isdigit()
                    or obj[4] != 'obstacle' or abs(float(obj[5])) >= .2
                    or len(obj) <= 12 or not isinstance(obj[12], dict)
                    or not math.isfinite(float(obj[12].get('point_count', 0)))
                    or float(obj[12].get('point_count', 0)) < 1. or wall_like(obj)):
                continue
            size = np.asarray(obj[6:8], float)
            if (not np.isfinite(size).all() or min(size) <= 0. or max(size) > 2.
                    or max(size) > 3.*min(size)):
                continue
            checked = np.maximum(size, minimum)
            if np.array_equal(checked, size):
                continue
            row = list(obj)
            row[6], row[7] = map(float, checked)
            row[2] = max(row[6:8])
            result[index] = tuple(row)
        except (TypeError, ValueError, IndexError, OverflowError):
            continue
    return result


def static_plan_complete(plan):
    if not plan or plan.get('blocked'):
        return False
    try:
        clear = float(plan['clear_distance_m'])
        required = max(float(plan['safe_horizon_m']), float(plan['hold_until_m']))
        return math.isfinite(clear) and math.isfinite(required) and clear >= required >= 0.
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def configure(max_curvature, clear_distance=None):
    """Use the host vehicle's limits inside the package.

    The package allows tan(35 deg)/3.0 = 0.233 1/m; the host follower is validated
    for tan(35 deg)/3.90 = 0.180 1/m, and the tunnel rejects sharper paths anyway.
    clear_distance: the host's vectorised footprint test (same four separating axes,
    margins and rear-object rule as the package's loop version, but faster)."""
    _static.MAX_CONTROL_CURVATURE = float(max_curvature)
    _geometry.MAX_CONTROL_CURVATURE = float(max_curvature)
    if clear_distance is not None:
        _static.static_clear_distance = clear_distance


# --------------------------------------------------------------- static / dynamic
def is_static(obj, standing_person=None, in_tunnel=False):
    """True only for things the host's motion evidence calls stationary."""
    kind, speed = obj[4], float(obj[5])
    if kind == "obstacle":
        return speed < OBSTACLE_STATIC_MPS
    if kind == "pedestrian":
        # Road: only a person confirmed standing in our lane past the wait (the
        # lattice's rule). Tunnel: TunnelTracks labels every moving cluster this way.
        return not in_tunnel and standing_person is not None and bool(standing_person(obj))
    if kind == "npc":
        # Fused cars are moving by construction (a parked one arrives as an
        # obstacle); a LiDAR car counts only with a measured, valid low speed.
        return (str(obj[3]) == "lidar_car" and len(obj) > 14 and bool(obj[14])
                and speed < CAR_STATIC_MPS)
    return False


def as_static(obj):
    values = list(obj)
    values[5] = 0.
    if len(values) > 10:
        values[9] = values[10] = 0.
    return tuple(values)


def split_static(objects, standing_person=None, in_tunnel=False):
    """(static objects with zero motion for the planner, everything else)."""
    static, dynamic = [], []
    for obj in objects:
        if is_static(obj, standing_person, in_tunnel):
            static.append(as_static(obj))
        else:
            dynamic.append(obj)
    return static, dynamic


def wall_like(obj):
    """Tunnel clusters TunnelTracks never moves: wall or kerb runs, not obstacles."""
    length, width = float(obj[6]), float(obj[7])
    longest, thinnest = max(length, width), min(length, width)
    return longest > 8. or (longest >= 6. and thinnest <= .35)


def merge_lidar_obstacles(objects, lidar_boxes, margin_m=1.0):
    """LiDAR-only boxes that no listed object explains (fusion missed them)."""
    kept = []
    for box in lidar_boxes:
        explained = False
        for obj in objects:
            reach = max(margin_m, .5 * float(obj[2]) + .5)
            if math.hypot(float(box[0]) - float(obj[0]), float(box[1]) - float(obj[1])) <= reach:
                explained = True
                break
        if not explained:
            kept.append(box)
    return kept


# ------------------------------------------------------------ package hazards
def confirmed_hazards(dense_route, ego, static_objects, footprint=None, body_shift_m=ROAD_LEGACY_SHIFT_M):
    """The package's own trigger, limited to its own activation distance.

    Same steps as the start of plan_static_bypass: route_hazard within 65 m, then
    the footprint test for objects the lateral test missed. Returns (hazards,
    engagement_m); hazards are dicts with "front" (m along the route).
    footprint: footprint_hazards() of the same inputs, if already computed.
    body_shift_m: body placement on the pose (ego_body; tunnel: REAR_AXLE_SHIFT_M)."""
    if not static_objects or dense_route is None or len(dense_route) < 3:
        return [], 0.
    hazards = (footprint_hazards(dense_route, ego, static_objects, body_shift_m)
               if footprint is None else footprint)
    speed = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
    activation = max(25., min(45., speed * 3.5 + 6.))
    required = max(4., speed * .7 + speed * speed / 5. + 2.)
    engagement = max(activation, required)
    return [h for h in hazards if h["front"] <= engagement], engagement


def footprint_hazards(dense_route, ego, static_objects, body_shift_m=ROAD_LEGACY_SHIFT_M):
    """Every static object within 65 m that the car's footprint (0.2 m margin) on the
    route would touch - the objects the planner has to go around - nearest first."""
    if not static_objects or dense_route is None or len(dense_route) < 3:
        return []
    debug = []
    _static.route_hazard(static_objects, ego, dense_route, ahead_m=65., half_width_m=1.146,
                         max_speed_mps=1.5, debug=debug)
    route = np.asarray(dense_route, dtype=float)[:, :2]
    route_length = min(65., float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum()))
    observed = {(obj[0], obj[1]): obj for obj in static_objects}
    hazards = []
    for record in debug:
        ok = record["ok"]
        if not ok:
            obj = observed.get(record.get("world_xy"))
            ok = obj is not None and _static.static_clear_distance(
                dense_route, ego, [obj], 65., body_shift_m=body_shift_m) < route_length - .1
        if ok:
            hazards.append(record)
    hazards.sort(key=lambda d: d["front"])
    return hazards


# ------------------------------------------------------------- normal-road room
# A neighbouring lane or junction space that begins only ahead of the car: the
# bypass keeps the route and moves over where the corridor on that side starts to
# widen. Starts are tried this far before that point; the corridor check decides.
ROAD_START_LEADS_M = (8., 4., 0.)
ROAD_WIDEN_M = .3


def road_offsets(near, ahead, arc, left, right, leads=ROAD_START_LEADS_M, widen_m=ROAD_WIDEN_M):
    """adjacent_offsets for the static planner on the normal road.

    near: the host's lookup 8 m ahead of the car. ahead: the same lookup beside the
    first confirmed obstacle. Looking only 8 m ahead, an obstacle inside a junction or
    past a lane that widens in had no room to go around until the car was ~16 m from
    it (follow-20261009T191227Z: blocked at 30 m, 39 -> 12 km/h; at 24 m, 16 -> 10
    km/h). A side found only ahead is offered too. Every side is searched lowest
    curvature first (starts as a tuple: static_obstacle's ordered search) with the
    planner's smaller open-road offsets (2.8, 2.2 m) besides the lane centre, so a car
    already holding 2.2 m is not sent to the lane centre once that lane comes within
    8 m. Starts: the route is kept until the corridor bound on that side (left/right
    along the dense route, at stations arc) moves out by widen_m, starting up to
    leads metres earlier, and at the car (the corridor check decides)."""
    offsets = Offsets({int(side): float(value) for side, value in (near or {}).items()})
    arc = np.asarray(arc, dtype=float)

    def delayed_starts(side):
        room = side * np.asarray(left if side > 0 else right, dtype=float)
        if len(room) != len(arc) or len(arc) < 3:
            return None
        widened = np.flatnonzero(room > float(room[0]) + widen_m)
        if not len(widened):
            return None
        opens_m = float(arc[widened[0]])
        return tuple(sorted({0.} | {round(max(0., opens_m - lead), 2) for lead in leads}))
    starts = {side: delayed_starts(side) or (0.,) for side in offsets}
    for side, value in sorted((ahead or {}).items()):
        side = int(side)
        if side in offsets or side not in (-1, 1):
            continue
        values = delayed_starts(side)
        if values is None:
            continue
        offsets[side] = float(value)
        starts[side] = values
    offsets.start_m_by_side = starts
    offsets.open_road_sides = tuple(sorted(offsets))
    return offsets


# --------------------------------------------------------------------- v6 model
def start_v6(runtime, environ=None):
    """Start the v6 child with the driving extension; None (GUI unaffected) if unset."""
    from v6 import shadow_runtime
    env = dict(os.environ if environ is None else environ)
    runtime.v6_latest = None
    runtime.v6_worker = None
    if not env.get("STUDENT_V6_CHECKPOINT"):
        return None
    if not env.get("STUDENT_LOG_DIR"):
        status = getattr(getattr(runtime, "args", None), "status", None)
        if status:
            env["STUDENT_LOG_DIR"] = str(Path(status).parent)   # the run's debug folder
    return shadow_runtime.start_v6_shadow(runtime, environ=env,
                                          child_cmd=[sys.executable, "-m", "v6_ext_child"])


def v6_decision(runtime, refresh=True, max_age_s=V6_MAX_AGE_S, now_wall=None):
    """(fresh v6 result as a dict, None) or (None, reason)."""
    worker = getattr(runtime, "v6_worker", None)
    if refresh and worker is not None and hasattr(worker, "pull_latest"):
        try:
            worker.pull_latest()
        except Exception:
            pass
    latest = getattr(runtime, "v6_latest", None)
    if not isinstance(latest, dict):
        return None, "v6 unavailable"
    if not latest.get("ok"):
        return None, "v6 " + str(latest.get("reason") or "not ready")
    try:
        age = (time.time() if now_wall is None else float(now_wall)) - float(latest["t"])
    except (KeyError, TypeError, ValueError):
        return None, "v6 time invalid"
    if not -.05 <= age <= max_age_s:
        return None, "v6 stale %.2f s" % age
    action = latest.get("action_full") or ACTION_LONG.get(latest.get("action"), latest.get("action"))
    probabilities = latest.get("probabilities") or {}
    try:
        probability = float(probabilities.get(action, latest.get("p") or 0.))
    except (TypeError, ValueError):
        probability = 0.
    return dict(action=action, p=probability, age_s=age, path=latest.get("path"),
                dt=float(latest.get("dt") or .1), pose=latest.get("pose"),
                probabilities=probabilities), None


def model_side(decision, min_p=V6_MIN_P):
    if not decision:
        return 0.
    side = SIDE_OF_ACTION.get(decision.get("action"), 0.)
    return side if side and float(decision.get("p", 0.)) >= min_p else 0.


MANOEUVRE_DONE_M = .35   # a saved plan whose remaining path is back on the route is no bypass any more


def manoeuvre_complete(previous_plan, dense_route):
    """True once a saved bypass has rejoined: its remaining path ahead stays on the route.

    Its side was otherwise kept for the next obstacle (follow-20261010T083538Z
    17:37:16-17:38:37 and lap 2: a left bypass, then a stopped car 30 m on with room
    only on the right; left 0.45 m plans and blocks repeated to a stop 5 m short)."""
    if previous_plan is None or previous_plan.get("path") is None:
        return False
    path = np.asarray(previous_plan["path"], dtype=float)[:, :2]
    route = np.asarray(dense_route, dtype=float)[:, :2]
    ahead = path[path[:, 0] > 0.]
    if len(ahead) < 2 or len(route) < 2:
        return False
    gap = np.min(np.linalg.norm(ahead[:, None, :]-route[None, :, :], axis=2), axis=1)
    return bool(gap.max() < MANOEUVRE_DONE_M)


def side_has_room(offsets, side, minimum_m=1.):
    """offsets (side -> lateral room beside the first obstacle) leaves room on this side."""
    return any(int(k) == int(side) and float(v) >= minimum_m for k, v in dict(offsets or {}).items())


def kept_side_closed(source, direction, offsets, dense_route, hazards, near_route_m=1.):
    """A kept side that has no room beside the next obstacle while the other side has.

    A rejoin's left side was kept for a stopped car 33 m ahead with room only on the
    right (follow-20261010T090630Z 18:13:31-37: left 0.45 m plans and holds, 37 -> 4
    km/h). Only while the car is still near the route, never beside an obstacle."""
    if source != "kept" or not hazards or direction not in (-1, 1, -1., 1.):
        return False
    route = np.asarray(dense_route, dtype=float)[:, :2]
    if len(route) == 0 or float(np.min(np.linalg.norm(route, axis=1))) > near_route_m:
        return False
    return not side_has_room(offsets, direction) and side_has_room(offsets, -direction)


def choose_direction(previous_plan, hazards, decision, held=None):
    """(committed_direction, source). A running manoeuvre keeps its side."""
    if previous_plan is not None and previous_plan.get("direction") in (-1, 1, -1., 1.):
        return float(previous_plan["direction"]), "kept"
    if held:
        return float(held["direction"]), "lane_hold"
    if hazards:
        side = model_side(decision)
        if side:
            return side, "v6"
    return 0., "planner"


def pending_bounded_search_release(previous, ego, policy, lane_corridor):
    """Release only an unentered bounded proposal while the body is still legal.

    The caller must first establish confirmed hazards and failure of the
    previous path's current complete clearance checks. This predicate never
    changes the original proposal, frozen corridor or excursion clock.
    """
    try:
        if (not previous or previous.get('bounded_static_pass') is not True
                or previous.get('static_borrow') or previous.get('standing_bypass_ids')
                or previous.get('direction') not in (-1., 1.) or policy is None
                or policy.entered_at is not None or ego.get('measured') is not True):
            return False
        pose = np.asarray([ego['x'], ego['y'], ego['heading_deg'], ego['stamp']], float)
        age = float(ego['age_s'])
        if (not np.isfinite(pose).all() or pose[-1] <= 0.
                or not math.isfinite(age) or not 0. <= age <= .3):
            return False
        legal = policy.corridor(ego, lane_corridor)
        if legal is None:
            return False
        route = np.asarray(legal.route, float)
        left, right = np.asarray(legal.left, float), np.asarray(legal.right, float)
        if (route.ndim != 2 or route.shape[1] < 2 or len(route) < 3
                or left.shape != (len(route),) or right.shape != (len(route),)
                or not np.isfinite(route).all() or not np.isfinite(left).all()
                or not np.isfinite(right).all() or np.any(left < right)):
            return False
        from static_lane_pass import body_stations
        _, over_left, over_right = body_stations(np.array([[0., 0.], [2., 0.], [4., 0.]]), legal)
        current = np.asarray([over_left[0], over_right[0]], float)
        return bool(np.isfinite(current).all() and np.all(current <= .03))
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return False


# ----------------------------------------------------------------- the planner
class Budget:
    """Callback time limit: PLAN_BUDGET_S, plus PLAN_EXTRA_S while nothing passed yet."""

    def __init__(self, seconds=PLAN_BUDGET_S, extra_s=PLAN_EXTRA_S, clock=time.perf_counter, deadline=None):
        self.clock = clock
        self.soft = clock() + seconds
        self.hard = self.soft + extra_s
        if deadline is not None:
            self.soft, self.hard = min(self.soft, deadline), min(self.hard, deadline)
        self.found = False

    def expired(self):
        now = self.clock()
        return now > self.hard or (self.found and now > self.soft)

    def lane(self, function):
        def lane_ok(path, tolerance=1.5):
            if self.expired():
                return False
            ok = bool(function(path, tolerance))
            self.found = self.found or ok
            return ok
        return lane_ok

    def clearance(self, function):
        def corridor_clearance(path, horizon=60.):
            return 0. if self.expired() else float(function(path, horizon))
        return corridor_clearance


def plan_bypass(dense_route, ego, static_objects, lane_ok, corridor_clearance, committed_direction=0.,
                previous_plan=None, adjacent_offsets=None, defer_rejoin=False, deferred_offset_m=0.,
                deferred_distance_m=0., budget_s=PLAN_BUDGET_S, body_shift_m=ROAD_LEGACY_SHIFT_M,
                deadline=None):
    """The package's plan_static_bypass with a time budget and a both-sides retry.

    Returns (plan, hazards, info). plan None = nothing to do, plan["blocked"] = stop.
    body_shift_m: body placement on the pose in its footprint tests (ego_body; the
    tunnel passes REAR_AXLE_SHIFT_M, the road keeps the legacy centred box)."""
    def run(direction):
        budget = Budget(budget_s, deadline=deadline)
        plan, hazards = portable.plan_static_bypass(
            dense_route, ego, static_objects, budget.lane(lane_ok), committed_direction=direction,
            previous_plan=previous_plan, adjacent_offsets=adjacent_offsets,
            corridor_clearance=budget.clearance(corridor_clearance), defer_rejoin=defer_rejoin,
            deferred_offset_m=deferred_offset_m, deferred_distance_m=deferred_distance_m,
            body_shift_m=body_shift_m, search_expired=budget.expired)
        return plan, hazards, budget.expired()

    plan, hazards, exhausted = run(committed_direction)
    info = dict(direction_tried=float(committed_direction), retried_both_sides=False,
                budget_exhausted=exhausted, blocked_without_hazard=False)
    failed = (plan is None and bool(hazards)) or (plan is not None and plan.get("blocked"))
    if committed_direction and previous_plan is None and not defer_rejoin and failed:
        # The model's side may be wrong or illegal (3-8 % opposite on real data).
        retry, retry_hazards, retry_exhausted = run(0.)
        info["retried_both_sides"] = True
        info["budget_exhausted"] = exhausted or retry_exhausted
        if retry is not None and not retry.get("blocked"):
            plan, hazards = retry, retry_hazards
    if plan is not None and plan.get("blocked") and not hazards and not defer_rejoin:
        # A rejoin with nothing on the route ahead is not a reason to stop: the
        # host keeps its route and checks it like any other path.
        info["blocked_without_hazard"] = True
        plan = None
    if plan is not None and not plan.get("blocked"):
        plan = dict(plan, generator="portable")   # 0.25 m swept-body margins in the tunnel checks
    return plan, hazards, info


def road_white_line_corridor(corridor, allowance_m=.5):
    """A bounded right white-line allowance for normal-road static passing.

    The host opts in only outside junctions, highways, tunnels and pedestrian
    manoeuvres. The left boundary, no-go polygons, body margin and raw-return
    sweep are unchanged. This is a lane-edge permission, not free space.
    """
    if corridor is None:
        return None
    import copy
    allowance = float(allowance_m)
    if not math.isfinite(allowance) or not 0. <= allowance <= .5:
        raise ValueError('normal static white-line allowance must be within 0.5 m')
    checked = copy.copy(corridor)
    amount = np.where(corridor.intersection_mask, 0., allowance)
    checked.right = np.asarray(corridor.right, float)-amount
    checked.body_shift_m = REAR_AXLE_SHIFT_M
    return checked


def road_static_checks(corridor, points, *, margin_m=ROAD_STATIC_MARGIN_M):
    """Use the rear-axle body for both mapped corners and measured static returns.

    Keep the caller's corridor intact: highway and junction policies share it.
    Missing scans or missing map geometry never establish free space.
    """
    import copy
    from tunnel_runtime import swept_clearance
    checked = copy.copy(corridor) if corridor is not None else None
    if checked is not None:
        checked.body_shift_m = REAR_AXLE_SHIFT_M

    def clearance(path, horizon=60.):
        if checked is None or points is None:
            return 0.
        return min(checked.clear_distance(path, horizon),
                   swept_clearance(path, points, horizon, margin_m,
                                   lateral_margin=margin_m,
                                   body_shift_m=REAR_AXLE_SHIFT_M))

    def lane_ok(path, tolerance=1.5):
        if checked is None:
            return False
        length = min(25., float(np.linalg.norm(np.diff(np.asarray(path)[:, :2], axis=0), axis=1).sum()))
        return checked.clear_distance(path, length) >= length-.3

    return lane_ok, clearance


def _road_sparse_front_objects(objects, ego, route):
    """Preference-only bounds for a few returns from a compact object ahead."""
    route = np.asarray(route, float)[:, :2]
    tangent = np.gradient(route, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    result = []
    for obj in objects:
        try:
            name = str(obj[3])
            count = float(obj[12].get('point_count', 0.)) if len(obj) > 12 and isinstance(obj[12], dict) else 0.
            size = np.asarray(obj[6:8], float)
            if (not name.startswith('tunnel_lidar_') or not name[13:].isdigit()
                    or obj[4] != 'obstacle' or not math.isfinite(float(obj[5]))
                    or abs(float(obj[5])) >= .2 or not 1. <= count <= 4.
                    or not np.isfinite(size).all() or min(size) <= 0.
                    or max(size) > 2. or max(size) > 3.*min(size) or wall_like(obj)):
                continue
            point = np.asarray(_static.local_xy(obj[0], obj[1], ego), float)
            at = int(np.argmin(np.sum((route-point)**2, axis=1)))
            normal = np.array([-tangent[at, 1], tangent[at, 0]])
            if not 8. <= point[0] <= 45. or abs(float((point-route[at])@normal)) > 3.:
                continue
            row = list(obj)
            row[6], row[7] = map(float, np.maximum(size, 1.1))
            row[2] = max(row[6:8])
            result.append(tuple(row))
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            continue
    return result


def _road_rejoin_preference_objects(objects, ego, route, *, include_near=False):
    """Use current verified compact fusion source bounds for ranking only.

    The ordinary normalized boxes keep their geometry and remain the hard
    clearance input. This preference follows the existing sparse-return
    bounds through fusion. Near fusion bounds are used only to verify an
    existing return; fresh proposals retain their original front limits.
    """
    try:
        route = np.asarray(route, float)
        if (route.ndim != 2 or route.shape[1] < 2 or len(route) < 2
                or not np.isfinite(route[:, :2]).all()):
            return []
        route = route[:, :2]
    except (TypeError, ValueError, OverflowError):
        return []
    result = _road_sparse_front_objects(objects, ego, route)
    tangent = np.gradient(route, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    padding = 2.*(ROAD_STATIC_MARGIN_M-PORTABLE_STATIC_MARGIN_M)
    for obj in objects:
        try:
            geometry = object_metadata(obj).get('road_static_geometry') or {}
            if (not isinstance(geometry, dict)
                    or obj[3] != 'fusion_obstacle' or obj[4] != 'obstacle'
                    or abs(float(obj[5])) >= .2 or geometry.get('version') != 1
                    or geometry.get('source') != 'semantic_fusion'
                    or geometry.get('source_padding_m') != FUSION_OBSTACLE_DIMENSION_PADDING_M
                    or not math.isfinite(float(geometry.get('observed_points', 0.)))
                    or (float(geometry.get('observed_points', 0.)) < 1.
                        and not (geometry.get('producer_geometry') is True
                                 and _current_producer_geometry(obj, ego, geometry)))
                    or wall_like(obj)):
                continue
            size, physical, source = (np.asarray(value, float) for value in
                (obj[6:8], geometry['physical_size_m'], geometry['source_size_m']))
            if (any(value.shape != (2,) for value in (size, physical, source))
                    or not np.isfinite(np.r_[size, physical, source, obj[:2], obj[8], obj[5]]).all()
                    or not math.isclose(float(geometry['applied_padding_m']), padding, abs_tol=1e-6)
                    or np.any(physical < FUSION_OBSTACLE_MIN_DIMENSION_M-1e-6)
                    or np.any(source < FUSION_OBSTACLE_MIN_DIMENSION_M+
                              FUSION_OBSTACLE_DIMENSION_PADDING_M-1e-6)
                    or np.any(size > source+1e-6)
                    or not np.allclose(size, physical+padding, rtol=0., atol=1e-6)
                    or max(source) > 2. or max(source) > 3.*min(source)):
                continue
            point = np.asarray(_static.local_xy(obj[0], obj[1], ego), float)
            at = int(np.argmin(np.sum((route-point)**2, axis=1)))
            normal = np.array([-tangent[at, 1], tangent[at, 0]])
            minimum_front = 0. if include_near else 8.
            if not minimum_front <= point[0] <= 45. or abs(float((point-route[at])@normal)) > 3.:
                continue
            row = list(obj)
            row[6], row[7] = map(float, source)
            row[2] = max(row[6:8])
            # Current-pose recovery belongs to the normalized hard box. The
            # larger source preference must prove the ordinary full SAT margin.
            preferred_geometry = {key: value for key, value in geometry.items()
                if key not in ('initial_margin_floor_m', 'initial_margin_restore_start_m')}
            result.append(with_metadata(row, road_static_geometry=preferred_geometry))
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            continue
    return result


def _road_sparse_gap(path, ego, objects, horizon, deadline):
    """Minimum separating-axis gap to current sparse front bounds; ranking only."""
    if time.perf_counter() >= deadline:
        return None
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    samples = np.arange(0., min(horizon, arc[-1])+.01, .25)
    centre = np.column_stack([np.interp(samples, arc, path[:, i]) for i in (0, 1)])
    tangent = np.gradient(centre, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    tangent[0] = (1., 0.)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    centre += REAR_AXLE_SHIFT_M*tangent
    result = math.inf
    for obj in objects:
        if time.perf_counter() >= deadline:
            return None
        delta = np.asarray(_static.local_xy(obj[0], obj[1], ego))-centre
        yaw = math.radians(float(obj[8])-float(ego['heading_deg']))
        axis = np.array([math.cos(yaw), math.sin(yaw)])
        cross = np.array([-axis[1], axis[0]])
        along, across = np.abs(tangent@axis), np.abs(tangent@cross)
        half_long, half_wide = float(obj[6])/2., float(obj[7])/2.
        gap = np.maximum.reduce((
            np.abs(np.sum(delta*tangent, axis=1))-_static.EGO_HALF_LENGTH_M-half_long*along-half_wide*across,
            np.abs(np.sum(delta*normal, axis=1))-_static.EGO_HALF_WIDTH_M-half_long*across-half_wide*along,
            np.abs(delta@axis)-half_long-_static.EGO_HALF_LENGTH_M*along-_static.EGO_HALF_WIDTH_M*across,
            np.abs(delta@cross)-half_wide-_static.EGO_HALF_LENGTH_M*across-_static.EGO_HALF_WIDTH_M*along))
        result = min(result, float(np.min(gap))-PORTABLE_STATIC_MARGIN_M)
    return result if math.isfinite(result) and time.perf_counter() < deadline else None



def _road_steering_limit(path, ego, speed):
    """The controller's signed steering-rate check at physical arc spacing."""
    if not np.isfinite(path).all():
        return None
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    rate_s = np.linspace(0., arc[-1], max(3, int(math.ceil(arc[-1]/.5))+1))
    xy = np.column_stack([np.interp(rate_s, arc, path[:, i]) for i in (0, 1)])
    edge, following, chord = xy[1:-1]-xy[:-2], xy[2:]-xy[1:-1], xy[2:]-xy[:-2]
    denominator = (np.linalg.norm(edge, axis=1)*np.linalg.norm(following, axis=1)
                   *np.linalg.norm(chord, axis=1))
    signed = np.divide(2.*(edge[:, 0]*chord[:, 1]-edge[:, 1]*chord[:, 0]), denominator,
                       out=np.zeros_like(denominator), where=denominator >= 1e-8)
    steering, steering_s = np.arctan(3.90*signed), rate_s[1:-1]
    initial = ego.get('wheel_angle')
    if initial is not None:
        if not math.isfinite(float(initial)):
            return None
        steering = np.r_[math.radians(float(initial)), steering]
        steering_s = np.r_[0., steering_s]
    interval = np.diff(steering_s)
    gradient = np.divide(np.diff(steering), interval, out=np.zeros_like(interval), where=interval >= 1e-8)
    physical_rate = math.radians(30.)
    reachable = np.sqrt(np.maximum(0., speed*speed-5.*np.maximum(0., steering_s[1:]-speed*.2)))
    if np.any((steering_s[1:] <= 5.) & (reachable*np.abs(gradient) > physical_rate+1e-6)):
        return None
    return physical_rate*.8/max(float(np.max(np.abs(gradient), initial=0.)), 1e-6)

def road_checked_continuation(previous, ego, canonical, clearance):
    """Retain a currently checked full pass when fresh proposal search fails.

    The saved path contributes geometry only. Its current measured-pose join,
    steering, lateral acceleration and full remaining clearance must pass
    again before it can survive a later search-budget expiry.
    """
    if (not static_plan_complete(previous)
            or not (previous.get('canonical_control') or
                    previous.get('bounded_static_pass') and previous.get('fixed_world_curve'))
            or previous.get('bounded_static_pass') and not previous.get('fixed_world_curve')
            or previous.get('static_borrow')
            or previous.get('standing_bypass_ids')):
        return None
    from gui import BYPASS_ALAT_MPS2
    path = np.asarray(canonical(previous['path']), dtype=float)[:, :2]
    if len(path) < 3 or not np.isfinite(path).all():
        return None
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    horizon = min(60., float(arc[-1]))
    speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
    required = max(4., speed*.7+speed*speed/5.+2.)
    hold_until = max(required, float(previous['hold_until_m']))
    curvature = np.asarray(_geometry.path_curvatures(path))
    peak = float(np.max(curvature, initial=0.))
    steering_cap = _road_steering_limit(path, ego, speed)
    if (horizon < max(35., required, hold_until) or steering_cap is None
            or peak > _geometry.MAX_CONTROL_CURVATURE
            or np.any(np.maximum(0., speed*speed-3.*np.maximum(0., arc[1:-1]-speed*.7))
                      *curvature > BYPASS_ALAT_MPS2+.05)):
        return None
    clear = clearance(path, horizon)
    if not math.isfinite(clear) or clear < horizon-1e-6:
        return None
    return dict(previous, path=path, canonical_control=True, reused=True,
                blocked=False, kept_checked_static=True, replan_needed=False,
                clear_distance_m=clear, safe_horizon_m=required,
                hold_until_m=hold_until, curvature=peak,
                ordinary_speed_cap_mps=ROAD_STATIC_PASS_MPS,
                speed_cap_mps=min(ROAD_STATIC_FAST_MPS,
                    math.sqrt(BYPASS_ALAT_MPS2/max(peak, 1e-3)), steering_cap))


def road_rejoin_plan(route, ego, objects, previous, canonical, clearance, deadline):
    """Search fresh full road geometry before reusing a shortened old return.

    Sparse front returns and verified compact fusion source bounds get an
    optional preference and measured SAT-gap ranking. Every selected path
    still passes the unchanged complete
    host clearance and dynamic checks; a failed preference never removes a
    fully checked hard fallback. No future observations enter this search.
    Recompute pace from this frame's curvature and measured steering: an old
    transient steering cap is not a permanent limit on a freshly checked path.
    """
    from gui import BYPASS_ALAT_MPS2
    if not previous or previous.get('bounded_static_pass') or previous.get('static_borrow'):
        return None
    route = np.asarray(route, dtype=float)[:, :2]
    hazards, _ = confirmed_hazards(route, ego, objects,
        footprint_hazards(route, ego, objects, body_shift_m=REAR_AXLE_SHIFT_M))
    v = max(0., float(ego.get('speed_kmh', 0.)))/3.6
    required = max(4., v*.7+v*v/5.+2.)
    hold_until = max([required]+[float(h.get('back', h['along']+2.))+3. for h in hazards])
    if (previous.get('kind') == 'rejoin' and abs(previous.get('offset_m', 0.)) < .05
            and not hazards
            and (not previous.get('road_return') or manoeuvre_complete(previous, route))):
        return None
    sparse = _road_rejoin_preference_objects(objects, ego, route)
    side = -1. if hazards and hazards[0]['lat'] > 0. else 1.
    if sparse:
        front = min(sparse, key=lambda obj: _static.local_xy(obj[0], obj[1], ego)[0])
        point = np.asarray(_static.local_xy(front[0], front[1], ego))
        at = int(np.argmin(np.sum((route-point)**2, axis=1)))
        tangent = route[min(len(route)-1, at+1)]-route[max(0, at-1)]
        normal = np.array([-tangent[1], tangent[0]])
        side = -1. if float((point-route[at])@normal) > 0. else 1.
    offsets = ((0.,)+tuple(side*value for value in (.6, .55, .5, .45, .4, .35, .3))
               if sparse else (0., side*.3, side*.45, side*.6) if hazards else (0.,))
    old_offset = abs(float(previous.get('offset_m', 0.)))
    if hazards and not sparse and previous.get('road_return') and 0. < old_offset <= .6:
        offsets = tuple(dict.fromkeys((0., side*old_offset)+offsets[1:]))
    cached_path = np.asarray(previous.get('path', np.empty((0, 2))), float)[:, :2]
    cached_length = float(np.linalg.norm(np.diff(cached_path, axis=0), axis=1).sum())
    # Leave one ordinary full-path check for an already selected world return;
    # the original overall deadline still bounds admission of any candidate.
    search_deadline = deadline-(.003 if previous.get('road_return') and cached_length >= 35. else 0.)
    hard_return = None
    if (previous.get('road_return') and cached_length >= 35.
            and (hazards or not manoeuvre_complete(previous, route))
            and time.perf_counter() < search_deadline):
        # A fresh, less curved hard fallback can have less clearance than the
        # existing world return. Keep that return only after proving its exact
        # current join against every full-path guard and the current source
        # bounds. Near fusion bounds protect continuation, not new proposals.
        path = np.asarray(canonical(previous['path']), float)[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        horizon = min(60., float(arc[-1]))
        curvature = np.asarray(_geometry.path_curvatures(path))
        peak = float(np.max(curvature, initial=0.))
        steering_cap = _road_steering_limit(path, ego, v)
        continuation_bounds = _road_rejoin_preference_objects(objects, ego, route, include_near=True)
        if (steering_cap is not None
                and horizon >= max(35., required, hold_until)
                and peak <= _geometry.MAX_CONTROL_CURVATURE
                and not np.any(np.maximum(0., v*v-3.*np.maximum(0., arc[1:-1]-v*.7))*curvature
                               > BYPASS_ALAT_MPS2+.05)
                and time.perf_counter() < search_deadline):
            source_clear = bool(continuation_bounds and _static.static_clear_distance(
                path, ego, continuation_bounds, horizon,
                body_shift_m=REAR_AXLE_SHIFT_M) >= horizon-.1)
            clear = clearance(path, horizon)
            if clear >= horizon-.1:
                hard_return = dict(previous, path=path, canonical_control=True, reused=True,
                    kept_comfortable_return=False, kept_hard_return=True,
                    cached_return_fallback=True, preferred_raw_gap_m=None,
                    fresh_return_candidates_n=0, clear_distance_m=clear,
                    safe_horizon_m=required, hold_until_m=hold_until, curvature=peak,
                    ordinary_speed_cap_mps=ROAD_STATIC_PASS_MPS,
                    speed_cap_mps=min(ROAD_STATIC_FAST_MPS,
                                     math.sqrt(BYPASS_ALAT_MPS2/max(peak, 1e-3)), steering_cap))
                if source_clear:
                    gap = _road_sparse_gap(path, ego, continuation_bounds, horizon, search_deadline)
                    return dict(hard_return, kept_comfortable_return=True,
                        kept_hard_return=False, cached_return_fallback=False,
                        preferred_raw_gap_m=gap)
                # Source bounds rank comfort; their failure cannot discard a
                # return that just passed every current hard guard. Search the
                # original preferred family, then keep this proven fallback.
                if not sparse:
                    return hard_return
    transitions = (26., 22., 18., 14., 10., 8., 6.) if sparse else (18., 14., 10., 8., 6.)
    # Interleave the two gentle lengths at each offset so neither family
    # consumes the budget before the other reaches a narrow free interval.
    trials = (iter([(offset, transition) for offset in offsets for transition in transitions[:2]]+
                   [(offset, transition) for transition in transitions[2:] for offset in offsets])
              if sparse else ((offset, transition) for offset in offsets for transition in transitions))
    preferred, fallback, deferred = [], [], []
    checked_n = 0
    for offset, transition in trials:
        if time.perf_counter() >= search_deadline:
            break
        hold = max(6., hold_until-transition)
        path, _, _ = _geometry.build_forced_bypass(route, np.sign(offset),
            offset_m=abs(offset), hold_m=hold, transitions=(transition,))
        if path is None:
            continue
        path = canonical(path)
        if time.perf_counter() >= search_deadline:
            break
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        horizon = min(60., float(arc[-1]))
        curvature = np.asarray(_geometry.path_curvatures(path))
        peak = float(np.max(curvature, initial=0.))
        if (horizon < max(35., required, hold_until) or peak > _geometry.MAX_CONTROL_CURVATURE
                or np.any(np.maximum(0., v*v-3.*np.maximum(0., arc[1:-1]-v*.7))*curvature
                          > BYPASS_ALAT_MPS2+.05)):
            continue
        steering_cap = _road_steering_limit(path, ego, v)
        if steering_cap is None:
            continue
        plan = dict(path=path, kind='rejoin' if offset == 0. else 'bypass',
            direction=float(np.sign(offset)) or previous.get('direction', 1.),
            offset_m=abs(offset), transition_m=transition, hold_m=hold,
            ordinary_speed_cap_mps=ROAD_STATIC_PASS_MPS,
            curvature=peak, speed_cap_mps=min(ROAD_STATIC_FAST_MPS,
                                            math.sqrt(BYPASS_ALAT_MPS2/max(peak, 1e-3)), steering_cap),
            safe_horizon_m=required, clear_distance_m=0., hold_until_m=hold_until,
            activation_m=35., replan_needed=False, reused=False, generator='portable',
            canonical_control=True, road_return=True)
        sparse_clear = (_static.static_clear_distance(path, ego, sparse, horizon,
            body_shift_m=REAR_AXLE_SHIFT_M) if sparse else horizon)
        if sparse and sparse_clear < horizon-.1 and fallback:
            # A complete hard fallback already survives budget expiry. Spend
            # the remaining preference search on the existing sparse-safe
            # candidates, then check better hard alternatives if time remains.
            deferred.append(((-peak,), plan, horizon))
            continue
        clear = clearance(path, horizon)
        if clear < horizon-.1:
            continue
        checked_n += 1
        plan.update(clear_distance_m=clear, fresh_return_candidates_n=checked_n)
        if not sparse:
            return plan
        # Preserve this complete hard candidate even if preference scoring
        # consumes the last part of the budget.
        fallback.append(((-peak,), plan))
        if time.perf_counter() >= search_deadline:
            break
        if sparse_clear < horizon-.1:
            continue
        gap = _road_sparse_gap(path, ego, sparse, horizon, search_deadline)
        if gap is not None:
            plan = dict(plan, preferred_raw_gap_m=gap)
            preferred.append(((gap, -peak), plan))
    if preferred:
        return dict(max(preferred, key=lambda row: row[0])[1], fresh_return_candidates_n=checked_n)
    if hard_return is not None:
        return dict(hard_return, fresh_return_candidates_n=checked_n)
    for score, plan, horizon in sorted(deferred, key=lambda row: row[0], reverse=True):
        if (time.perf_counter() >= search_deadline
                or score <= max(row[0] for row in fallback)):
            break
        clear = clearance(plan['path'], horizon)
        if clear >= horizon-.1:
            checked_n += 1
            fallback.append((score, dict(plan, clear_distance_m=clear)))
            break
    if fallback:
        return dict(max(fallback, key=lambda row: row[0])[1], fresh_return_candidates_n=checked_n)
    if previous.get('road_return') and time.perf_counter() < deadline:
        path = np.asarray(canonical(previous['path']), float)[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        horizon = min(60., float(arc[-1]))
        curvature = np.asarray(_geometry.path_curvatures(path))
        peak = float(np.max(curvature, initial=0.))
        steering_cap = _road_steering_limit(path, ego, v)
        if (steering_cap is not None and horizon >= max(35., required, hold_until)
                and peak <= _geometry.MAX_CONTROL_CURVATURE
                and not np.any(np.maximum(0., v*v-3.*np.maximum(0., arc[1:-1]-v*.7))*curvature
                               > BYPASS_ALAT_MPS2+.05)
                and time.perf_counter() < deadline):
            clear = clearance(path, horizon)
            if clear >= horizon-.1:
                return dict(previous, path=path, canonical_control=True,
                            reused=True, cached_return_fallback=True,
                            clear_distance_m=clear, safe_horizon_m=required,
                            hold_until_m=hold_until, curvature=peak,
                            ordinary_speed_cap_mps=ROAD_STATIC_PASS_MPS,
                            speed_cap_mps=min(ROAD_STATIC_FAST_MPS,
                                             math.sqrt(BYPASS_ALAT_MPS2/max(peak, 1e-3)), steering_cap))
    return None

def contact_static_pass(route, ego, objects, contact, canonical, clearance, deadline, preferred_side=0.):
    """Clear a measured side contact straight before starting a static bypass.

    The ordinary in-lane proposals all turn immediately. That can swing the
    body toward a remembered roadside return and repeatedly trip contact_guard.
    This only supplies geometry: the caller checks the same full body, map,
    static boxes, raw returns and contact sweep as every other proposal.
    """
    if (not contact or contact['block_forward'] or
            any(float(point[2]) > 0. for point in contact.get('points', ()))):
        return None
    hazards, _ = confirmed_hazards(route, ego, objects,
        footprint_hazards(route, ego, objects, body_shift_m=REAR_AXLE_SHIFT_M))
    if not hazards:
        return None
    start = max(0., float(contact['ahead_m'])+.5)
    ends = [float(h.get('back', h['along']+2.))+3. for h in hazards if h['front'] < 45.]
    if not ends:
        return None
    hold_until = max(ends)
    v = max(0., float(ego.get('speed_kmh', 0.)))/3.6
    required = max(4., v*.7+v*v/5.+2.)
    route = np.asarray(route, dtype=float)[:, :2]
    route_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    suffix = route[route_arc >= start]-[start, 0.]
    prefix = np.column_stack((np.linspace(0., start, max(2, int(start/.25)+1)),
                              np.zeros(max(2, int(start/.25)+1))))
    sides = (preferred_side, -preferred_side) if preferred_side in (-1., 1.) else (1., -1.)
    for transition in (22., 18., 14., 10.):
        for side in (0., *sides):
            for offset in ((0.,) if side == 0. else (1.2, .9, .6, .45)):
                if time.perf_counter() >= deadline:
                    return None
                hold = max(6., hold_until-start-transition)
                path, _, _ = _geometry.build_forced_bypass(suffix, side, offset_m=offset,
                    hold_m=hold, max_curvature_1pm=_geometry.MAX_CONTROL_CURVATURE,
                    transitions=(transition,))
                if path is None:
                    continue
                path = canonical(np.vstack((prefix[:-1], path[:, :2]+[start, 0.])))
                arc = float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum())
                horizon = min(60., arc)
                peak = float(max(_geometry.path_curvatures(path), default=0.))
                if horizon < max(required, hold_until) or peak > _geometry.MAX_CONTROL_CURVATURE:
                    continue
                clear = clearance(path, horizon)
                if clear < horizon-.1:
                    continue
                return dict(path=path, kind='rejoin' if side == 0. else 'bypass',
                    direction=side or preferred_side or 1., offset_m=offset,
                    transition_m=transition, hold_m=hold, start_m=start, curvature=peak,
                    speed_cap_mps=15./3.6, safe_horizon_m=required,
                    clear_distance_m=clear, hold_until_m=hold_until, activation_m=35.,
                    replan_needed=False, reused=False, generator='portable',
                    canonical_control=True, contact_delayed=True)
    return None


def road_side_watch(points):
    """Planner-only straight-departure proposals for close static side returns."""
    from ego_body import body_gap, EGO_REAR_M, EGO_FRONT_M, EGO_HALF_WIDTH_M
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(points) or not np.isfinite(points).all() or body_gap(points) < .35:
        return None
    side = points[(points[:, 0] > -EGO_REAR_M-.1) & (points[:, 0] < EGO_FRONT_M+.5) &
                  (abs(points[:, 1]) > EGO_HALF_WIDTH_M) & (abs(points[:, 1]) < EGO_HALF_WIDTH_M+.65)]
    if not len(side):
        return None
    return dict(block_forward=False, block_reverse=True, where=['side'],
        gap0=body_gap(side), points=np.column_stack((side, np.zeros(len(side)))),
        ahead_m=max(0., float(side[:, 0].max())+EGO_REAR_M+.4))


def fast_static_clearance(plan, ego, objects, points):
    """Extra clearance required before increasing a normal-road static pass pace.

    The ordinary 25 cm planner remains available at its existing speed. Faster
    passing needs 60 cm around the entire checked manoeuvre, in both raw returns
    and object boxes, to leave tracking room above the 30 cm contact guard.
    """
    if (not static_plan_complete(plan) or plan.get('standing_bypass_ids') or points is None
            or any(obj[4] == 'pedestrian' for obj in objects)):
        return 0.
    from gui import static_clear_distance
    from tunnel_runtime import swept_clearance
    try:
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if not np.isfinite(points).all():
            return 0.
        horizon = max(35., float(plan['safe_horizon_m']), float(plan['hold_until_m']))
        padding = FAST_STATIC_MARGIN_M-PORTABLE_STATIC_MARGIN_M
        expanded = []
        for obj in objects:
            row = list(obj)
            row[6], row[7] = float(obj[6])+2.*padding, float(obj[7])+2.*padding
            expanded.append(tuple(row))
        return min(static_clear_distance(plan['path'], ego, expanded, horizon,
                       body_shift_m=REAR_AXLE_SHIFT_M),
                   swept_clearance(plan['path'], points, horizon, FAST_STATIC_MARGIN_M,
                       lateral_margin=FAST_STATIC_MARGIN_M, body_shift_m=REAR_AXLE_SHIFT_M))
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return 0.


def standing_road_corridor(lanes, ego, route, legal):
    """Mapped neighbouring pavement for a waited-out person, preserving no-go zones.

    Lane-change flags alone may be relaxed. Require the same road or a parallel
    oncoming lane beside it, plus operator permission; never infer a shoulder
    from empty LiDAR. The ordinary legal corridor is tried first by the caller.
    """
    from lane_corridor import LaneCorridor
    if legal is None or not getattr(lanes, 'n_links', 0):
        return legal
    route = np.asarray(route, float)[:, :2]
    h = math.radians(float(ego['heading_deg']))
    rotation = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
    world = route @ rotation.T + [ego['x'], ego['y']]
    tangent = np.gradient(world, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    distance, index = lanes.tree.query(world)
    links = lanes.link_of[index]
    left, right = legal.left.copy(), legal.right.copy()
    for key in np.unique(links):
        base = lanes.links[int(key)]
        mask = (links == key) & (distance <= 1.5)
        if not base.get('road_id') or not mask.any():
            continue
        for link in lanes.links:
            if (link['idx'] == base['idx'] or lanes.blocked(link['idx']) or
                    link['idx'] not in lanes.geometry):
                continue
            p = lanes.geometry[link['idx']]
            same_road = link.get('road_id') == base['road_id']
            if not same_road:
                # Opposite directions commonly have different road ids. Only a
                # long, parallel mapped lane beside this road may be borrowed;
                # perpendicular junction links and distant roads are excluded.
                direction = p[-1]-p[0]
                length = float(np.linalg.norm(direction))
                if (length < 10. or str(link.get('link_type')) == '1' or
                        float(direction @ tangent[mask].mean(axis=0))/length > -.9 or
                        np.any(p.max(axis=0) < world[mask].min(axis=0)-5.) or
                        np.any(p.min(axis=0) > world[mask].max(axis=0)+5.)):
                    continue
            _, at = lanes.link_trees[link['idx']].query(world)
            nt = p[np.minimum(at + 1, len(p) - 1)] - p[np.maximum(at - 1, 0)]
            nt /= np.maximum(np.linalg.norm(nt, axis=1, keepdims=True), 1e-9)
            delta = p[at] - world
            lateral = np.sum(delta * normal, axis=1)
            valid = (mask & (np.linalg.norm(delta, axis=1) <= 5.) &
                     (np.abs(np.sum(nt * tangent, axis=1)) >= .9) & (np.abs(lateral) <= 4.5))
            # A short route connector may provide fewer than twenty samples,
            # although the adjacent opposite-direction lane itself is long.
            if not same_road and np.count_nonzero(valid) < min(20, np.count_nonzero(mask)):
                continue
            half = .5 * np.clip(min(float(link.get('width_start', 3.5)),
                                   float(link.get('width_end', 3.5))), 2.5, 4.5)
            left[valid] = np.maximum(left[valid], lateral[valid] + half)
            right[valid] = np.minimum(right[valid], lateral[valid] - half)
    return LaneCorridor(route, left, right, zones=legal.zones,
                        body_shift_m=REAR_AXLE_SHIFT_M)


def standing_static_clear_distance(path, ego, objects, horizon_m=60.):
    """Road standing-bypass SAT with 1.9 m width and the actual rear axle.

    Keep the ordinary 0.2 m static margin. Sample at most 0.05 m apart:
    the shared 0.25 m check missed a front corner near C's small box. This
    stricter road-only proof also applies when reusing a selected path.
    """
    from ego_body import EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M
    from ped_guard import local_xy
    path = np.asarray(path, float)[:, :2]
    if len(path) < 2 or not np.isfinite(path).all():
        return 0.
    if np.linalg.norm(path[0]) > .01:
        path = np.vstack(([0., 0.], path))
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    horizon = min(float(horizon_m), float(arc[-1]))
    if horizon <= 0.:
        return 0.
    objects = list(objects)
    if not objects:
        return horizon
    sample = np.linspace(0., horizon, max(3, int(math.ceil(horizon/.05))+1))
    xy = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
    tangent = np.gradient(xy, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    tangent[0] = (1., 0.)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    centres = xy+REAR_AXLE_SHIFT_M*tangent
    positions = np.asarray([local_xy(o[0], o[1], ego) for o in objects])
    yaw = np.radians([float(o[8])-float(ego['heading_deg']) for o in objects])
    axes = np.column_stack((np.cos(yaw), np.sin(yaw)))
    cross = np.column_stack((-axes[:, 1], axes[:, 0]))
    sizes = np.maximum(.3, np.asarray([o[6:8] for o in objects], float))/2.
    if not np.isfinite(np.r_[positions.ravel(), axes.ravel(), sizes.ravel()]).all():
        return 0.
    lengths, widths = sizes[:, 0, None], sizes[:, 1, None]
    dx = positions[:, 0, None]-centres[None, :, 0]
    dy = positions[:, 1, None]-centres[None, :, 1]
    tx, ty = tangent[:, 0], tangent[:, 1]
    nx, ny = normal[:, 0], normal[:, 1]
    ax, ay = axes[:, 0, None], axes[:, 1, None]
    cx, cy = cross[:, 0, None], cross[:, 1, None]
    at, an = abs(ax*tx+ay*ty), abs(ax*nx+ay*ny)
    ct, cn = abs(cx*tx+cy*ty), abs(cx*nx+cy*ny)
    hl, hw, margin = EGO_HALF_LENGTH_M, max(.95, EGO_HALF_WIDTH_M), .2
    overlap = abs(dx*tx+dy*ty) <= hl+lengths*at+widths*ct+margin
    overlap &= abs(dx*nx+dy*ny) <= hw+lengths*an+widths*cn+margin
    overlap &= abs(dx*ax+dy*ay) <= hl*at+hw*an+lengths+margin
    overlap &= abs(dx*cx+dy*cy) <= hl*ct+hw*cn+widths+margin
    hit = np.flatnonzero(overlap.any(axis=0))
    return min(horizon, float(sample[hit[0]])) if len(hit) else horizon


def standing_person_gap(path, ego, people):
    """Minimum person-body gap to the actual rear-axle vehicle sweep (metres)."""
    from ego_body import EGO_FRONT_M, EGO_REAR_M, EGO_HALF_WIDTH_M
    path = np.asarray(path, float)[:, :2]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    sample = np.linspace(0., arc[-1], max(3, int(math.ceil(arc[-1] / .1)) + 1))
    xy = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
    t = np.gradient(xy, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
    t[0] = (1., 0.)
    h = math.radians(float(ego['heading_deg']))
    c, s = math.cos(h), math.sin(h)
    gap = math.inf
    for obj in people:
        dx, dy = float(obj[0]) - ego['x'], float(obj[1]) - ego['y']
        delta = np.array([dx*c + dy*s, -dx*s + dy*c]) - xy
        along = np.sum(delta*t, axis=1)
        across = delta[:, 1]*t[:, 0] - delta[:, 0]*t[:, 1]
        radius = max(.3, .5*max(float(obj[6]), float(obj[7])))
        distances = np.hypot(np.maximum(np.maximum(along-EGO_FRONT_M, -EGO_REAR_M-along), 0.),
                             np.maximum(np.abs(across)-EGO_HALF_WIDTH_M, 0.)) - radius
        gap = min(gap, float(distances.min()))
    return gap


def _standing_join(route, side, offset, transition, hold_until, cubic=False):
    """A pose-anchored bypass; a cubic entry can turn earlier at standstill."""
    p = np.asarray(route, float)[:, :2]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    arc, keep = np.unique(arc, return_index=True)
    p = p[keep][arc <= 60.]
    arc = arc[arc <= 60.]
    if len(p) < 3 or arc[-1] < 25.:
        return None
    t = np.gradient(p, arc, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
    n = np.column_stack((-t[:, 1], t[:, 0]))
    def smooth(u):
        u = np.clip(u, 0., 1.)
        return u**3*(10.-15.*u+6.*u*u)
    return_start = max(transition + 6., hold_until)
    reference = p + n*(side*offset*(1.-smooth((arc-return_start)/transition)))[:, None]
    initial_tangent = (reference[1]-reference[0])/(arc[1]-arc[0])
    correction = reference[0]+arc[:, None]*(initial_tangent-[1., 0.])
    u = np.clip(arc/transition, 0., 1.)
    join = u*u*(3.-2.*u) if cubic else smooth(u)
    return reference-correction*(1.-join)[:, None]


def plan_standing_bypass(route, ego, static_objects, legal, raw_clearance, people,
                         previous=None, road=None, budget_s=.15, allow_search=True,
                         person_clearances=None, controlled_ids=None, static_person_ids=(),
                         speed_cap_mps=None):
    """Search one checked path around a stationary person and the whole box group.

    Road-only opt-in. Never reuse a braking prefix to authorize passing a person.
    Every candidate checks the true body, the caller's per-person clearance (the
    default is 1.05 m), ordinary margins for evidenced static objects, raw returns,
    road bounds and steering rate. Legal paths precede boundary excursions; an
    excursion is ranked by estimated 5 s penalties per 3 s of body contact.
    """
    from lane_corridor import LaneCorridor
    from ego_body import EGO_HALF_LENGTH_M
    started = time.perf_counter()
    hazards = footprint_hazards(route, ego, static_objects, REAR_AXLE_SHIFT_M)
    group = []
    for hazard in hazards:
        if group and hazard['front']-group[-1].get('back', group[-1]['along']) > 25.:
            break
        if hazard.get('back', hazard['along']) + 3. > 57.:
            break
        group.append(hazard)
    hold_until = max([10.] + [h.get('back', h['along']) + 3. for h in group])
    ids = sorted(controlled_ids or (int(o[11]) for o in people)) or list((previous or {}).get('standing_bypass_ids', ()))
    gaps = {int(o[11]): float((person_clearances or {}).get(int(o[11]), 1.05)) for o in people}
    cap_limit = float(speed_cap_mps if speed_cap_mps is not None else
                      (previous or {}).get('standing_bypass_cap_mps', 8./3.6))
    blocked = dict(blocked=True, kind='bypass', standing_bypass_ids=ids,
                   standing_bypass_cap_mps=cap_limit, safe_horizon_m=4., activation_m=25.,
                   person_clearances_m=gaps,
                   reason='no complete static bypass with required body clearance',
                   rejected=dict(curvature=0, person=0, clearance=0, steering=0))
    if legal is None or (not ids and previous is None):
        return blocked, hazards
    legal = LaneCorridor(legal.route, legal.left, legal.right, zones=legal.zones,
                         body_shift_m=REAR_AXLE_SHIFT_M)
    road = road or legal
    road = LaneCorridor(road.route, road.left, road.right, zones=road.zones,
                        body_shift_m=REAR_AXLE_SHIFT_M)
    static = [o for o in static_objects if o[4] != 'pedestrian']
    speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
    required = max(4., speed*.7 + speed*speed/5. + 2.)
    candidates = []

    def check(path, side, offset, transition, reused=False):
        if path is None or not np.isfinite(path).all():
            return
        from gui import RosPathPublisher
        path = (np.asarray(path, float)[:, :2] if reused else
                RosPathPublisher.control_geometry(np.asarray(path, np.float32), route,
                    dict(mode='STUDENT FORCED', validated_static=True)))
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        horizon = min(60., float(arc[-1]))
        if horizon < hold_until + transition or np.any(np.diff(path[:8, 0]) < -.01):
            return
        curvature = float(np.max(_geometry.path_curvatures(path)))
        if curvature > min(_static.MAX_CONTROL_CURVATURE, math.tan(math.radians(35.))/3.90):
            blocked['rejected']['curvature'] += 1
            return
        person_gap = standing_person_gap(path, ego, people)
        if math.isfinite(person_gap):
            blocked['best_person_gap_m'] = max(blocked.get('best_person_gap_m', -math.inf), person_gap)
        if any(standing_person_gap(path, ego, [o]) < gaps[int(o[11])] for o in people):
            blocked['rejected']['person'] += 1
            return
        clear = min(_static.static_clear_distance(path, ego, static, horizon,
                        body_shift_m=REAR_AXLE_SHIFT_M), raw_clearance(path, horizon),
                    road.clear_distance(path, horizon),
                    standing_static_clear_distance(path, ego, static, horizon))
        # Include the return: never call a path ending beside a person a bypass.
        if clear < horizon-.3 or clear < required:
            blocked['rejected']['clearance'] += 1
            return
        sample = np.arange(0., horizon, .25)
        xy = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
        t = np.gradient(xy, axis=0)
        heading = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
        wheel = np.arctan(3.90*np.gradient(heading, sample))
        if ego.get('wheel_angle') is not None:
            wheel[0] = math.radians(float(ego['wheel_angle']))
        gradients = np.abs(np.diff(wheel)/.25)
        rate_cap = math.radians(30.)*.8/max(float(gradients.max()), 1e-6)
        reachable = np.sqrt(np.maximum(0., speed*speed-5.*np.maximum(0., sample[1:]-speed*.2)))
        if np.any((sample[1:] <= 5.) & (reachable*gradients > math.radians(30.)+1e-6)):
            blocked['rejected']['steering'] += 1
            return
        cap = min(cap_limit, rate_cap, math.sqrt(1.7/max(curvature, 1e-3)))
        if cap < .2:
            return
        t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
        n = np.column_stack((-t[:, 1], t[:, 0]))
        corners = np.stack([xy+a*t+b*n for a in
            (REAR_AXLE_SHIFT_M-EGO_HALF_LENGTH_M, REAR_AXLE_SHIFT_M+EGO_HALF_LENGTH_M)
            for b in (-1.146, 1.146)], axis=1).reshape(-1, 2)
        _, at = legal.tree.query(corners)
        lateral = np.sum((corners-legal.route[at])*legal.normal[at], axis=1)
        outside = ((lateral > legal.left[at]+.03) | (lateral < legal.right[at]-.03)).reshape(-1, 4).any(axis=1)
        contact_s = float(np.count_nonzero(outside)*.25/cap)
        penalty_s = 5.*math.ceil(contact_s/3.) if contact_s else 0.
        plan = dict(path=path, direction=side, offset_m=offset, transition_m=transition,
                    hold_m=max(6., hold_until-transition), kind='bypass', curvature=curvature,
                    speed_cap_mps=cap, clear_distance_m=clear, safe_horizon_m=required,
                    activation_m=25., hold_until_m=hold_until, cluster_count=len(group),
                    replan_needed=False, reused=reused, standing_bypass_ids=ids,
                    standing_bypass_cap_mps=cap_limit,
                    static_person_ids=list(static_person_ids), person_clearances_m=gaps,
                    person_gap_m=person_gap if math.isfinite(person_gap) else None,
                    boundary_contact_s=contact_s, boundary_penalty_s=penalty_s,
                    generator='standing_bypass')
        candidates.append(((bool(contact_s), penalty_s, contact_s, horizon/cap,
                            0. if reused else 1., abs(offset)), plan))

    if previous is not None and previous.get('standing_bypass_ids'):
        check(previous['path'], previous['direction'], previous['offset_m'],
              previous['transition_m'], reused=True)
        if candidates and not candidates[0][0][0]:
            return candidates[0][1], hazards
    if not allow_search:
        return (min(candidates, key=lambda row: row[0])[1] if candidates else blocked), hazards
    from ped_guard import PathFrame, local_xy
    frame = PathFrame(route)
    laterals = frame.project([local_xy(o[0], o[1], ego) for o in people])[1]
    radii = [max(.3, .5*max(float(o[6]), float(o[7]))) for o in people]
    group_xy = {tuple(h.get('world_xy', (None, None))) for h in group}
    plateau_boxes = []
    for obj in static:
        if tuple(obj[:2]) not in group_xy:
            continue
        _, lateral, at = frame.project([local_xy(obj[0], obj[1], ego)])
        yaw = math.radians(float(obj[8])-float(ego['heading_deg']))
        axis = np.array([math.cos(yaw), math.sin(yaw)])
        cross = np.array([-axis[1], axis[0]])
        extent = .5*(float(obj[6])*abs(frame.n[at[0]]@axis)+float(obj[7])*abs(frame.n[at[0]]@cross))
        plateau_boxes.append((float(lateral[0]), extent))
    for offset in (.45, .6, .9, 1.2, 1.6, 2., 2.2, 2.5, 2.8, 3.2, 3.5, 4., 4.25, 4.5):
        for side in (1., -1.):
            # Skip plateau widths that cannot even fit the stationary body.
            # Offsets are relative to the route, including an already offset ego.
            if any(abs(lat-side*offset) < .946+radius+gaps[int(o[11])]-.05
                   for lat, radius, o in zip(laterals, radii, people)):
                continue
            if any(abs(lat-side*offset) < .946+extent+.2 for lat, extent in plateau_boxes):
                continue
            for transition in (6., 7., 8., 8.5, 9., 10., 12., 16., 22.):
                if time.perf_counter()-started > budget_s:
                    return (min(candidates, key=lambda row: row[0])[1] if candidates else blocked), hazards
                for cubic in (False, True):
                    path = _standing_join(route, side, offset, transition, hold_until, cubic)
                    check(path, side, offset, transition)
    return (min(candidates, key=lambda row: row[0])[1] if candidates else blocked), hazards


# ---------------------------------------------------------------------- tunnel
class Offsets(dict):
    """adjacent_offsets with the package's optional attributes."""
    start_m_by_side = {}
    open_road_sides = ()


def tunnel_offsets(available, lane_m=TUNNEL_LANE_M, min_lane_m=TUNNEL_MIN_LANE_M):
    """Free width to each wall -> at most one lane per side, the whole width open road."""
    offsets = Offsets({int(side): min(float(room), lane_m) for side, room in (available or {}).items()
                       if float(room) >= min_lane_m})
    offsets.open_road_sides = tuple(offsets)
    offsets.start_m_by_side = {}
    return offsets


def tunnel_route_bounds(route_xy, active):
    """First and last route index the host's own tunnel test calls inside."""
    inside = [i for i, point in enumerate(np.asarray(route_xy, dtype=float)[:, :2])
              if active(dict(x=float(point[0]), y=float(point[1]), heading_deg=0.))]
    return (inside[0], inside[-1]) if inside else None


# Return 70 m (package default 45 m) before the exit: the bypass lane must be left before
# the exit zone, where the portal ends the wall match (follow-20261009T185440Z).
TUNNEL_REJOIN_M = float(os.environ.get("STUDENT_TUNNEL_REJOIN_M", "70"))


def make_tunnel_policies(route_xy, bounds, rejoin_buffer_m=TUNNEL_REJOIN_M, max_kmh=50.):
    route = np.asarray(route_xy, dtype=float)[:, :2]
    lo, hi = bounds
    scope = portable.TunnelRoutePolicy(route, route[lo], route[hi], rejoin_buffer_m=rejoin_buffer_m, enabled=True)
    model = portable.TunnelModelPolicy(route, route[lo], route[hi], max_kmh=max_kmh, enabled=True)
    return scope, model


# ------------------------------------------------------- tunnel bypass shape and commitment
# follow-20261010T151125Z: the portable planner re-plans a fresh bypass nearly every
# frame (its reused 60 m path is always ~1 m short of the 60 m horizon), choosing among
# lane offsets 3.5/2.8/2.2 m and the first feasible 10-26 m transitions by a score that
# weighs curvature lightly. Offsets jumped 3.5 -> 2.8 -> 3.5 m beside the entry bike
# (00:16:35.0: a 14 m transition, 0.099 1/m, 20 km/h), and the 3.5 m lane offset past
# the parked car (lat -2.1 m) had to return within 9-15 m before the next parked car on
# the other side (lat +3.6 m): 0.055-0.081 1/m returns, 22-27 km/h for 2-4 s. Every
# hazard of that stretch (car, drum, car) is passed on one straight line 0.5-1.4 m left.
# The host keeps the planner's side and checks, and only re-shapes its manoeuvre:
#  * among the planner's offset, the committed one and smaller offsets down to 0.9 m,
#    with transitions up to 40 m, the path with the highest curvature speed (braking-
#    aware, tunnel_runtime.curve_speed_cap) that passes the planner's own complete-path
#    tests (static boxes, raw returns, walls, lane prefix, clear distance no shorter than
#    the planner's) and keeps TUNNEL_COMFORT_SIDE_M (0.8 m) from every static box;
#  * the committed offset is kept unless another is clearly faster (no per-frame jumps);
#  * a bypass is not given up for a fresh rejoin while a hazard on the route lies within
#    TUNNEL_COMMIT_AHEAD_M: the return would only be followed by a new swerve. After the
#    last hazard the planner's rejoin runs once.
TUNNEL_SMOOTH_TRANSITIONS_M = (40., 34., 28.)
# A join from the actual pose keeps its heading for ~0.27 x transition before it turns
# to the offset, so a heading error swings the path that far past it: 7.6 deg on a 40 m
# transition put the car 5.9 m left of the route instead of 3.5 m, 0.3 m from the kerb
# (follow-20261010T173230Z 02:41:43-46). Transitions whose swing exceeds
# TUNNEL_SWING_M are not offered; shorter ones are added instead.
TUNNEL_SHORT_TRANSITIONS_M = (22., 18., 14.)
TUNNEL_SWING_M = .8
# Body to the tunnel's inner faces at least this: the floor coverage confirms a footprint
# (0.25 m margin plus a 2 deg ray sector) only up to 0.5 m short of the kerb return, and
# the inner face lies 0.2 m beyond the kerb face. The wall check (0.25 m) let the swing
# reach 0.3 m from the kerb, where coverage read 1 m: a 25 s crawl at 3 km/h (same drive
# 02:41:45-02:42:11).
TUNNEL_SHAPE_WALL_MARGIN_M = 1.
TUNNEL_SMOOTH_OFFSETS_M = (2.2, 1.6, 1.2, .9)
# Body to every static box (the planner's own check keeps 0.2 m): the lattice's comfortable
# pass (lattice_planner.COMFORT_SIDE_M), so a faster line never passes closer than that.
TUNNEL_COMFORT_SIDE_M = .8
TUNNEL_COMMIT_AHEAD_M = 50.       # about the package's rejoin gap (2 x transition + 5 m)
TUNNEL_STICKY_MPS = 1.            # another offset must be this much faster to replace the kept one
TUNNEL_SMOOTH_BUDGET_S = .025
# Hazards closer than a return plus a new entry (2 x the longest transition + 5 m, the
# package's rejoin gap at these transitions) are passed as one group; the 57 m window
# below bounds it as in the package.
TUNNEL_HAZARD_GROUP_GAP_M = 2. * max(TUNNEL_SMOOTH_TRANSITIONS_M) + 5.


def tunnel_required_m(speed_mps):
    """Clear path length a tunnel manoeuvre needs at this speed: the runtime's own stop
    (TUNNEL_BRAKE_MPS2 after 0.5 s) plus 4 m. The package's v*0.7 + v^2/5 + 2 (2.5 m/s^2)
    asks 42 m at 45 km/h, which the tunnel's parked-car slalom (car, drum, then a car on
    the other side 35 m on) rarely leaves: every candidate failed and the planner
    returned "no safe bypass" for 1-4 frames at 45 km/h (follow-20261010T162527Z
    01:30:32.9, 01:35:21.9). The runtime still brakes for the measured clearance."""
    from tunnel_runtime import TUNNEL_BRAKE_MPS2
    v = max(0., float(speed_mps))
    return max(8., .5 * v + v * v / (2. * TUNNEL_BRAKE_MPS2) + 4.)


TUNNEL_BRIDGE_MAX_FRAMES = 15     # ~1.5 s of planner failures in a row


def bridge_tunnel_plan(plan, previous_plan, ego, static_objects, corridor_clearance, bridged_frames=0):
    """(plan, info): keep driving the committed manoeuvre when the planner finds none.

    The planner reports "no safe bypass" (or a blocked return) for single frames while
    the path being driven stays clear: a parked car beside the car became a rear hazard
    with its front at 0 m, a box flickered, or its 2.5 m/s^2 requirement exceeded the
    room (follow-20261010T162527Z 01:30:25.0: 39 -> 8.6 km/h; 01:30:32.9 and 01:35:21.9:
    45 -> 27 / 15 km/h; 01:35:28.0: 40 -> 19 km/h, then the exit stop). Blocked frames
    publish an approach cap of 8 km/h. The previous path is kept when it is still clear
    of the static boxes, raw returns, walls and lane for the tunnel stopping distance;
    the runtime's final checks and speed caps apply to it as to any plan."""
    info = dict(applied=False)
    blocked = plan is None or bool(plan.get("blocked"))
    if (not blocked or previous_plan is None or previous_plan.get("blocked")
            or previous_plan.get("path") is None or previous_plan.get("generator") != "portable"
            or bridged_frames >= TUNNEL_BRIDGE_MAX_FRAMES):
        return plan, info
    path = np.asarray(previous_plan["path"], dtype=float)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
    speed = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
    needed = tunnel_required_m(speed)
    horizon = min(60., float(arc[-1]))
    if horizon < needed:
        return plan, dict(info, reason="previous path too short")
    clear = min(_static.static_clear_distance(path, ego, static_objects, horizon, body_shift_m=REAR_AXLE_SHIFT_M),
                float(corridor_clearance(path, horizon)))
    if clear < needed:
        return plan, dict(info, reason="previous path obstructed", clear_m=round(clear, 1))
    return (dict(previous_plan, reused=True, bridged=True, blocked=False, clear_distance_m=clear),
            dict(applied=True, clear_m=round(clear, 1), needed_m=round(needed, 1), frames=bridged_frames + 1))


def _group_hold_until(hazards, max_front_m=57.):
    """Route distance (m) to 3 m past the last hazard of the first connected group."""
    group_back = None
    for hazard in sorted(hazards or (), key=lambda h: float(h["front"])):
        front = float(hazard["front"])
        back = float(hazard.get("back", 2. * float(hazard.get("along", front)) - front))
        if group_back is not None and front - group_back > TUNNEL_HAZARD_GROUP_GAP_M:
            break
        if back + 3. > max_front_m:
            break
        group_back = back if group_back is None else max(group_back, back)
    return None if group_back is None else group_back + 3.


def _comfort_boxes(static_objects, margin_m=TUNNEL_COMFORT_SIDE_M-.2):
    rows = []
    for obj in static_objects:
        row = list(obj)
        if len(row) > 7:
            row[6], row[7] = float(row[6]) + 2. * margin_m, float(row[7]) + 2. * margin_m
            row[2] = max(row[6], row[7])
        else:
            row[2] = float(row[2]) + 2. * margin_m
        rows.append(tuple(row))
    return rows


def heading_swing_limit_m(dense_route, swing_m=TUNNEL_SWING_M):
    """Longest join transition whose heading swing (0.27 T tan(error)) stays within swing_m."""
    route = np.asarray(dense_route, dtype=float)[:, :2]
    if len(route) < 3:
        return math.inf
    tangent = route[min(4, len(route) - 1)] - route[0]
    error = abs(math.atan2(float(tangent[1]), float(tangent[0])))
    return math.inf if error < 1e-3 else swing_m / (.27 * math.tan(min(error, math.radians(60.))))


def smooth_tunnel_plan(plan, hazards, dense_route, ego, static_objects, lane_ok, corridor_clearance,
                       previous_plan=None, budget_s=TUNNEL_SMOOTH_BUDGET_S, clock=time.perf_counter,
                       wall_room=None):
    """(plan, info): the planner's tunnel manoeuvre re-shaped for speed and kept stable.

    Only the shape of a validated portable plan changes (offset on the same side,
    transition length, hold); blocked, held (tunnel_hold) or reused-return plans are
    returned as they are. Every replacement passes the same tests as the planner's own
    candidates, plus TUNNEL_COMFORT_SIDE_M from the static boxes."""
    from tunnel_runtime import curve_speed_cap
    from gui import RosPathPublisher
    started = clock()
    info = dict(applied=False)
    if (plan is None or plan.get("blocked") or plan.get("path") is None
            or plan.get("generator") != "portable" or plan.get("kind") not in ("bypass", "rejoin")):
        return plan, info
    kind = plan["kind"]
    if kind == "rejoin" and (plan.get("reused") or float(plan.get("offset_m", 0.)) > .05):
        return plan, info   # a saved bypass path relabelled while no hazard is engaged
    committed = None
    if (previous_plan is not None and previous_plan.get("direction") in (-1, 1, -1., 1.)
            and float(previous_plan.get("offset_m", 0.)) >= .4 and not previous_plan.get("blocked")
            and previous_plan.get("kind") != "tunnel_hold"):
        committed = (float(previous_plan["direction"]), float(previous_plan["offset_m"]))
    route_hazards, commit = list(hazards or ()), False
    if kind == "rejoin" and committed is not None:
        # A fresh return while the route ahead is still obstructed: hold instead.
        ahead = [h for h in footprint_hazards(dense_route, ego, static_objects, REAR_AXLE_SHIFT_M)
                 if float(h["front"]) > -1.]
        if ahead and min(float(h["front"]) for h in ahead) <= TUNNEL_COMMIT_AHEAD_M:
            route_hazards, commit = ahead, True
    direction = committed[0] if commit else float(plan.get("direction", 0.))
    if kind == "bypass" and direction not in (-1., 1.):
        return plan, info
    speed = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
    required = tunnel_required_m(speed)

    def speed_of(path):
        return curve_speed_cap(RosPathPublisher.resample(np.asarray(path, dtype=float)[:, :2]), speed)[0]

    original_speed = speed_of(plan["path"])
    plan_path = np.asarray(plan["path"], dtype=float)[:, :2]
    plan_horizon = min(60., float(np.linalg.norm(np.diff(plan_path, axis=0), axis=1).sum()))
    # How far the planner's own path keeps TUNNEL_SHAPE_WALL_MARGIN_M from the walls.
    planner_room = float(wall_room(plan_path, plan_horizon)) if wall_room is not None else math.inf
    near_wall = planner_room < min(plan_horizon, max(required, 20.)) - .3
    if kind == "rejoin" and not commit:
        offsets, hold_until, sticky = [0.], 0., None
    else:
        hold_until = _group_hold_until(route_hazards)
        if hold_until is None:
            return plan, dict(info, reason="no hazard group")
        sticky = committed[1] if committed is not None and committed[0] == direction else None
        offsets = []
        for value in (sticky, float(plan.get("offset_m", 0.)) if kind == "bypass" else None) + TUNNEL_SMOOTH_OFFSETS_M:
            if value is not None and value >= .4 and not any(abs(value - o) < .05 for o in offsets):
                offsets.append(min(float(value), TUNNEL_LANE_M))
    swing_limit = heading_swing_limit_m(dense_route)
    transitions = {t for t in TUNNEL_SMOOTH_TRANSITIONS_M if t <= swing_limit}
    if near_wall or len(transitions) < len(TUNNEL_SMOOTH_TRANSITIONS_M):
        transitions |= set(TUNNEL_SHORT_TRANSITIONS_M)
    transitions = sorted(transitions | {float(plan.get("transition_m", TUNNEL_SMOOTH_TRANSITIONS_M[-1]))},
                         reverse=True)
    candidates = []
    for offset in offsets:
        for transition in transitions:
            if clock() - started > budget_s:
                break
            hold = max(6., hold_until - transition) if offset > 0. else 0.
            path, peak, _ = _geometry.build_forced_bypass(
                dense_route, direction if offset > 0. else 0., offset_m=offset, hold_m=hold,
                max_curvature_1pm=_static.MAX_CONTROL_CURVATURE, transitions=(transition,))
            if path is None or peak > _static.MAX_CONTROL_CURVATURE:
                continue
            kept = sticky is not None and abs(offset - sticky) < .05
            candidates.append((speed_of(path) + (TUNNEL_STICKY_MPS if kept else 0.), offset, transition, hold,
                               path, peak, kept))
    # Fastest first (the kept offset counts TUNNEL_STICKY_MPS faster), then the wider pass.
    candidates.sort(key=lambda c: (-c[0], -c[1]))
    comfort = _comfort_boxes(static_objects)
    plan_clear = float(plan.get("clear_distance_m", 0.))
    tried = 0
    for rank, offset, transition, hold, path, peak, kept in candidates:
        if clock() - started > budget_s:
            info["budget_exhausted"] = True
            break
        value = rank - (TUNNEL_STICKY_MPS if kept else 0.)
        if (not commit and not near_wall and
                not (value >= original_speed + .3 or (kept and value >= original_speed - .3))):
            continue
        tried += 1
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(np.asarray(path)[:, :2], axis=0), axis=1))]
        horizon = min(60., float(arc[-1]))
        needed = max(required, hold_until)
        if arc[-1] < needed:
            continue
        clear = min(_static.static_clear_distance(path, ego, static_objects, horizon, body_shift_m=REAR_AXLE_SHIFT_M),
                    float(corridor_clearance(path, horizon)))
        if clear < needed or clear < min(horizon, plan_clear) - .3:
            continue
        if offset > 0. and _static.static_clear_distance(path, ego, comfort, horizon,
                                                         body_shift_m=REAR_AXLE_SHIFT_M) < needed:
            continue
        room = float(wall_room(path, horizon)) if wall_room is not None else math.inf
        if room < min(horizon, max(needed, 20.)) - .3 and not (near_wall and room >= planner_room + 2.):
            continue   # hugs a wall where the floor coverage fails (or no better than the planner)
        end = min(clear, needed)
        if not lane_ok(path[:min(len(path), int(np.searchsorted(arc, end)) + 1)], 1.5):
            continue
        usable = max(0., clear - 2.)
        reshaped = dict(plan, path=path, kind="bypass" if offset > 0. else "rejoin", direction=direction,
                        offset_m=offset, transition_m=transition, hold_m=hold, start_m=0., curvature=peak,
                        clear_distance_m=clear, hold_until_m=hold_until, safe_horizon_m=required,
                        speed_cap_mps=math.sqrt((2.5 * .7) ** 2 + 5. * usable) - 2.5 * .7,
                        replan_needed=clear < horizon - .5, reused=False, generator="portable",
                        reshaped=True)
        info.update(applied=True, committed=commit, offset_m=offset, transition_m=transition,
                    wall_room_m=round(room, 1) if math.isfinite(room) else None,
                    planner_wall_room_m=round(planner_room, 1) if math.isfinite(planner_room) else None,
                    curve_mps=round(value, 2), planner_curve_mps=round(original_speed, 2),
                    planner_offset_m=plan.get("offset_m"), planner_kind=kind, tried=tried,
                    ms=round((clock() - started) * 1000., 1))
        return reshaped, info
    return plan, dict(info, tried=tried, planner_curve_mps=round(original_speed, 2), commit_rejected=commit,
                      planner_wall_room_m=round(planner_room, 1) if math.isfinite(planner_room) else None,
                      candidates=len(candidates), ms=round((clock() - started) * 1000., 1))


def stamp_for_monotonic(samples, t_mono, max_gap_s=.1):
    """ROS stamp of the ego sample received nearest t_mono, plus the residual."""
    best = None
    for sample in samples:
        if "received" not in sample or "stamp" not in sample:
            continue
        gap = abs(float(sample["received"]) - t_mono)
        if best is None or gap < best[0]:
            best = (gap, sample)
    if best is None or best[0] > max_gap_s or float(best[1]["stamp"]) <= 0.:
        return None
    return float(best[1]["stamp"]) + (t_mono - float(best[1]["received"]))


def v6_path_now(decision, ego_samples, odometry, current_odometry, points_at_pose):
    """The v6 path in the current body frame, moved by uncorrected odometry only.

    The path is relative to the ESKF pose at the model's reference time. Wall-aided
    ESKF / matched poses can jump; the wheel/IMU odometry between the two times
    cannot, so only that relative motion is applied."""
    path = np.asarray(decision.get("path") or [], dtype=float)
    pose = decision.get("pose")
    if path.ndim != 2 or len(path) < 3 or path.shape[1] < 2 or not pose:
        return None, "v6 path or pose missing"
    stamp = stamp_for_monotonic(ego_samples, float(pose["t_mono"]))
    if stamp is None:
        return None, "v6 reference time outside the ego history"
    reference = odometry.at_stamp(stamp) if odometry is not None else None
    if reference is None or current_odometry is None:
        return None, "v6 reference pose outside the odometry history"
    local = points_at_pose(path[:, :2], reference, current_odometry)
    local = local[local[:, 0] >= 0.]                  # already passed
    if len(local) < 3:
        return None, "v6 path behind the car"
    return local, None


def lateral_departure(path, reference, ahead_m=30.):
    """Largest distance of the path's first ahead_m from the reference polyline."""
    path = np.asarray(path, dtype=float)[:, :2]
    reference = np.asarray(reference, dtype=float)[:, :2]
    if len(path) < 2 or len(reference) < 2:
        return 0.
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    path = path[arc <= ahead_m]
    distance = np.sqrt(((path[:, None, :] - reference[None, :, :]) ** 2).sum(axis=2)).min(axis=1)
    return float(distance.max()) if len(distance) else 0.


def v6_tunnel_candidate(decision, why, model_policy, path_now, path_why, reference, ego, constraints,
                        clear, required_m, maintain_mps, hazards, exit_zone, checked_model_path):
    """Same (candidate, info) contract as tunnel_model_guidance.propose, from v6.

    No candidate (host route + static planner) when: v6 is stale or missing, the
    exit zone owns the path, v6 asks to STOP / YIELD (sensors own stops: a stopped
    car makes the model keep predicting STOP), the package's model policy rejects
    the path (reverse, discontinuous, stationary), the path leaves the route with no
    confirmed obstacle, or the host's tunnel checks fail. The model's own speed is
    not used as a cap (a slowing car would keep predicting slow)."""
    info = dict(enabled=True, accepted=False, source="lidar_fallback", generator="v6")
    if decision is None:
        return None, dict(info, reason=why or "v6 unavailable")
    info.update(action=decision["action"], probability=round(float(decision["p"]), 3),
                v6_age_s=round(float(decision["age_s"]), 3))
    if path_now is not None:
        info["_raw_path"] = np.asarray(path_now)[:, :2]
    if exit_zone:
        return None, dict(info, reason="exit zone: the route rejoin owns the path")
    if decision["action"] in ("STOP", "YIELD"):
        return None, dict(info, reason="v6 %s ignored: sensors own stops" % decision["action"])
    if path_now is None:
        return None, dict(info, reason=path_why or "v6 path unavailable")
    _, policy = model_policy.plan(path_now, float(decision["dt"]), decision["action"], float(decision["p"]))
    info["model_policy"] = dict(motion_state=policy.get("motion_state"), reason=policy.get("reason"),
                                model_speed_mps=round(float(policy.get("model_speed_mps", 0.)), 2))
    if policy.get("motion_state") != "DRIVE":
        return None, dict(info, reason="v6 path rejected: " + str(policy.get("reason")))
    departure = lateral_departure(path_now, reference)
    info["lateral_departure_m"] = round(departure, 2)
    if not hazards and departure > LATERAL_WITHOUT_HAZARD_M:
        return None, dict(info, reason="v6 lateral move without a confirmed obstacle")
    candidate, checked = checked_model_path(path_now, reference, ego, constraints, clear, required_m, maintain_mps)
    info.update(checked)
    if candidate is not None:
        info["source"] = "v6"
    return candidate, info
