"""Static-obstacle corridor planning helpers, extracted for reuse."""
import math
import os
import numpy as np

from .geometry import MAX_CONTROL_CURVATURE, build_forced_bypass, path_curvatures

# The ego pose is the rear axle: the 4.635 x 1.892 m body reaches 3.845 m ahead of it
# and 0.79 m behind, so its centre lies 1.5275 m ahead (host: ego_body.py). body_shift_m
# places the body box that far ahead of each path sample; the default 0 keeps the
# package's original box centred on the pose (the host's road/highway setting).
EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M = 2.3175, 0.946

BYPASS_ALAT_MPS2 = float(os.environ.get("STUDENT_BYPASS_ALAT", "1.7"))

def route_hazard(objects, ego, dense_route, ahead_m=25.0, half_width_m=1.8, max_speed_mps=None, debug=None):
    """Nearest object sitting on the route ahead: (along_m, lateral_m, name, kind) or None. lateral > 0 = object left of route.
    max_speed_mps: ignore objects moving faster than this (a bypass is only planned around (near-)static objects)."""
    route = np.asarray(dense_route, dtype=np.float32)[:, :2]
    if len(route) < 3 or not objects:
        return None
    tang = np.gradient(route, axis=0); tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-6)
    normal = np.column_stack((-tang[:, 1], tang[:, 0]))
    s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))))
    best = None
    yaw = math.radians(float(ego["heading_deg"]))
    for x, y, size, name, kind, speed, *extra in objects:
        if max_speed_mps is not None and speed > max_speed_mps:
            continue
        f, l = local_xy(x, y, ego)
        min_ahead = 1.0 if ahead_m <= 10.0 else 3.0   # stop check: anything from 1 m ahead; bypass trigger: from 3 m
        # Full-body callers inspect debug rows with their own rear-axle SAT.
        # A centre beside the axle can still meet a front/rear corner during
        # the return; dropping it here prematurely retires the whole bypass.
        if f < min_ahead - 2.0 and debug is None:
            continue
        p = np.array([f, l], dtype=np.float32)
        k = int(np.argmin(((route - p) ** 2).sum(axis=1)))
        along, lat = float(s[k]), float(((p - route[k]) * normal[k]).sum())
        if len(extra) >= 3:   # oriented footprint: extent across / along the route direction
            sx, sy, hd = float(extra[0]), float(extra[1]), math.radians(float(extra[2]))
            d = hd - (yaw + math.atan2(float(tang[k, 1]), float(tang[k, 0])))
            ext_lat = 0.5 * (abs(sx * math.sin(d)) + abs(sy * math.cos(d)))
            ext_along = 0.5 * (abs(sx * math.cos(d)) + abs(sy * math.sin(d)))
        else:
            ext_lat = ext_along = 0.5 * float(size)
        ext_lat = max(0.15, min(ext_lat, 0.5 * float(size)))
        front = along - min(ext_along, 3.0)   # nearest face, not the centre
        back = along + min(ext_along, 3.0)
        lat_ok = abs(lat) - ext_lat <= half_width_m
        # a long object whose near face is already beside the bumper still blocks the path while its far face is ahead
        along_ok = back >= min_ahead and front <= ahead_m and f >= min_ahead - 2.0
        if debug is not None and f < max(30.0, ahead_m + 5.0):
            debug.append({"n": name[:18], "k": kind, "f": round(f, 1), "l": round(l, 1), "along": round(along, 1), "lat": round(lat, 2),
                          "world_xy": (x, y),
                          "ext": round(ext_lat, 2), "front": round(front, 1), "v": round(float(speed), 2), "ok": bool(lat_ok and along_ok)})
        if along_ok and lat_ok:
            if best is None or front < best[0]:
                best = (max(front, min_ahead), lat, name, kind, float(speed))
    return best

def static_clear_distance(path, ego, objects, horizon_m=35.0, body_shift_m=0.0):
    """Return the first overlapping vehicle pose distance, or the checked path length.
    body_shift_m: body box centre ahead of each path sample along its heading."""
    route = np.asarray(path, dtype=float)[:, :2]
    if len(route) < 2:
        return 0.0
    if np.linalg.norm(route[0]) > 0.01:
        route = np.vstack(([0.0, 0.0], route))
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    samples = np.arange(0.0, min(horizon_m, arc[-1]) + 0.01, 0.25)
    centres = np.column_stack([np.interp(samples, arc, route[:, i]) for i in (0, 1)])
    if len(centres) < 2:
        return 0.0
    tang = np.gradient(centres, axis=0)
    tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-6)
    tang[0] = (1.0, 0.0)
    normal = np.column_stack((-tang[:, 1], tang[:, 0]))
    clearance = min(float(horizon_m), float(arc[-1]))
    forward_prefix = centres[:max(2, min(len(centres), int(5.0 / 0.25) + 1)), 0]
    centres = centres + float(body_shift_m) * tang
    for obj in objects:
        position = np.asarray(local_xy(obj[0], obj[1], ego))
        yaw = math.radians((obj[8] if len(obj) > 8 else ego["heading_deg"]) - ego["heading_deg"])
        axis = np.array([math.cos(yaw), math.sin(yaw)])
        cross = np.array([-axis[1], axis[0]])
        length = max(0.3, float(obj[6] if len(obj) > 6 else obj[2])) / 2
        width = max(0.3, float(obj[7] if len(obj) > 7 else obj[2])) / 2
        # An object already behind the ego origin may overlap the car's rear
        # footprint at s=0. It recedes on a forward path and must not invalidate
        # every new bypass (run 36: bollard behind ego blocked both free lanes).
        rear_face = position[0] + length * abs(axis[0]) + width * abs(cross[0])
        if rear_face < -0.25 and np.all(np.diff(forward_prefix) >= -0.05):
            future = centres[samples >= 5.0]
            radius = math.hypot(EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M) + math.hypot(length, width) + 0.2
            if not len(future) or np.linalg.norm(future - position, axis=1).min() > radius:
                continue
        delta = position - centres
        overlap = np.ones(len(centres), dtype=bool)
        for a in (tang, normal, axis, cross):
            projected = np.abs(np.sum(delta * a, axis=1))
            # Keep the same lateral pedestrian margin as moving_conflict.
            # Otherwise a rejoin path is accepted here and the yield rule
            # stops the car beside a standing person with no way to replan.
            margin = 0.65 if obj[4] == "pedestrian" else 0.2
            radius = (EGO_HALF_LENGTH_M * np.abs(np.sum(tang * a, axis=1)) +
                      EGO_HALF_WIDTH_M * np.abs(np.sum(normal * a, axis=1)) +
                      length * np.abs(np.sum(axis * a, axis=-1)) +
                      width * np.abs(np.sum(cross * a, axis=-1)) + margin)
            overlap &= projected <= radius
        if np.any(overlap):
            clearance = min(clearance, float(samples[np.flatnonzero(overlap)[0]]))
    return clearance

def static_approach_cap_mps(distance_m):
    """Slow before a static obstacle so a feasible bypass or stop can be followed."""
    return max(8.0, min(25.0, 8.0 + 0.5 * (float(distance_m) - 10.0))) / 3.6

def continue_static_plan(saved, ego, now):
    """Carry the remaining world-frame path into the new ego frame, if still trackable."""
    if not saved or not 0.0 <= now - saved["stamp"] <= 1.0:
        return None
    local = np.array([local_xy(x, y, ego) for x, y in saved["world_path"]])
    nearest = int(np.argmin(np.linalg.norm(local, axis=1)))
    path = local[nearest:]
    if len(path) < 3 or np.linalg.norm(path[0]) > 0.8:
        return None
    tangent = path[min(3, len(path) - 1)] - path[0]
    if abs(math.atan2(tangent[1], tangent[0])) > math.radians(35):
        return None
    # Join from the actual ego pose. A nearest waypoint can be behind the car;
    # prepending the origin to that point creates a reverse segment and a sharp
    # curvature spike in the follower even though the saved path itself is smooth.
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    sample = np.linspace(0., arc[-1], max(2, int(math.ceil(arc[-1]/.25))+1))
    if len(sample) < 3:
        return None
    reference = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
    u = np.clip(sample / 6.0, 0., 1.)
    weight = u ** 3 * (10 - 15 * u + 6 * u ** 2)
    initial_tangent = (reference[1] - reference[0]) / (sample[1] - sample[0])
    correction = reference[0] + sample[:, None] * (initial_tangent - np.array([1., 0.]))
    joined = reference - correction * (1 - weight[:, None])
    return dict(saved["plan"], path=joined, reused=True)

def static_speed_cap_mps(cruise_mps, plan, approach_distance=None):
    """Use the validated driving path for speed; approach limits apply until it exists."""
    if plan is not None and not plan.get("blocked"):
        return min(cruise_mps, math.sqrt(BYPASS_ALAT_MPS2 / max(plan["curvature"], 1e-3)),
                   plan["speed_cap_mps"])
    return min(cruise_mps, static_approach_cap_mps(approach_distance)) if approach_distance is not None else cruise_mps

def recover_static_prefix(previous, ego, objects, corridor_clearance, minimum_horizon_m=35., body_shift_m=0.0):
    """Continue a checked bypass prefix while a longer replan is unavailable."""
    if previous is None:
        return None
    path = previous["path"]
    speed = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
    required = max(4., speed * .7 + speed * speed / 5. + 2.)
    horizon = max(35., float(minimum_horizon_m))
    stationary = [o for o in objects if o[4] == "obstacle" or
                  (o[4] == "pedestrian" and o[5] < .3) or
                  (o[4] == "npc" and o[5] < .5)]
    clear = min(static_clear_distance(path, ego, stationary, horizon, body_shift_m=body_shift_m),
                corridor_clearance(path, horizon))
    if clear < required:
        return None
    curvature = float(np.max(path_curvatures(path)))
    usable = max(0., clear - 2.)
    cap = math.sqrt((2.5 * .7) ** 2 + 5. * usable) - 2.5 * .7
    return dict(previous, kind="hold", curvature=curvature, speed_cap_mps=cap,
                safe_horizon_m=required, clear_distance_m=clear,
                replan_needed=True, reused=True)

def plan_static_bypass(dense_route, ego, objects, lane_ok, committed_direction=0,
                       previous_plan=None, adjacent_offsets=None, corridor_clearance=None,
                       minimum_horizon_m=35., defer_rejoin=False,
                       deferred_offset_m=0., deferred_distance_m=0., body_shift_m=0.0,
                       search_expired=None):
    """Keep one lane through a connected obstacle group and validate a braking-safe prefix.
    body_shift_m: body placement on each path sample in every footprint test (above).
    search_expired: optional caller budget, checked between complete candidate checks.
    Expiration preserves already checked candidates; it never validates a prefix."""
    stationary = [o for o in objects if o[4] == "obstacle" or
                  (o[4] == "pedestrian" and o[5] < 0.3) or (o[4] == "npc" and o[5] < 0.5)]
    debug = []
    route_hazard(stationary, ego, dense_route, ahead_m=65.0, half_width_m=1.146,
                 max_speed_mps=1.5, debug=debug)
    route_length = min(65., float(np.linalg.norm(
        np.diff(np.asarray(dense_route)[:, :2], axis=0), axis=1).sum()))
    observed = {(obj[0], obj[1]): obj for obj in stationary}
    for record in debug:
        if not record["ok"]:
            obj = observed[record["world_xy"]]
            record["ok"] = static_clear_distance(dense_route, ego, [obj], 65., body_shift_m=body_shift_m) < route_length-.1
    hazards = sorted((d for d in debug if d["ok"]), key=lambda d: d["front"])
    speed = max(0.0, float(ego.get("speed_kmh", 0))) / 3.6
    # Do not shrink the planning window down to 14 m as approach braking starts.
    activation = max(25.0, min(45.0, speed * 3.5 + 6.0))
    required = max(4.0, speed * .7 + speed ** 2 / 5.0 + 2.0)
    side_locked = committed_direction in (-1, 1)
    committed = previous_plan is not None and side_locked
    # The route projection drops an object once its centre passes the ego,
    # although the rear bumper is still alongside it. Keep the manoeuvre
    # active until the whole object clears the vehicle's rear footprint.
    if side_locked and abs(float(np.asarray(dense_route)[0, 1])) > 1.0:
        route = np.asarray(dense_route, dtype=float)[:, :2]
        route_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        for obj in stationary:
            forward, lateral = local_xy(obj[0], obj[1], ego)
            extent = min(3., max(float(obj[2]), float(obj[6] if len(obj) > 6 else 0.)) / 2.)
            if not (-2.5 < forward + extent and forward < 4. and 1.8 < abs(lateral) < 8.):
                continue
            index = int(np.argmin(np.sum((route - [forward, lateral]) ** 2, axis=1)))
            if route_s[index] > 8. or np.linalg.norm(route[index] - [forward, lateral]) > 2.0:
                continue
            if any(h["n"] == obj[3] for h in hazards):
                continue
            hazards.append({"n": obj[3], "front": max(0., forward - extent),
                            "along": max(0., forward), "back": max(0., forward + extent),
                            "f": forward, "lat": lateral, "ok": True})
        hazards.sort(key=lambda d: d["front"])
    transition_hint = max(10., min(24., speed * 3.0),
                          float(previous_plan.get("transition_m", 0.)) if committed else 0.)
    # Returning and leaving again both need a complete lateral transition plus
    # room for the vehicle. This is a geometric decision, not a timer or route ID.
    rejoin_gap = 2.0 * transition_hint + 5.0
    engagement = max(activation, required, rejoin_gap if committed else 0.)
    active = bool(hazards and hazards[0]["front"] <= engagement)
    startup_rejoin = False
    if not active and not committed and adjacent_offsets:
        route = np.asarray(dense_route, dtype=float)[:, :2]
        tangent = route[min(4, len(route)-1)]-route[0]
        norm = float(np.linalg.norm(tangent))
        if norm > .1 and np.linalg.norm(route[0]) < 6.:
            normal = np.array([-tangent[1], tangent[0]])/norm
            lateral = float(-route[0] @ normal)
            starts = getattr(adjacent_offsets, "start_m_by_side", {}) or {}
            startup_rejoin = any(
                2.5 <= float(offset) <= 5. and min(start_values(starts.get(side, 0.))) <= 0.
                and abs(lateral-float(side)*float(offset)) <= .8
                for side, offset in adjacent_offsets.items())
            if startup_rejoin:
                committed_direction = 1 if lateral > 0. else -1
    if not active and not committed and not defer_rejoin and not startup_rejoin:
        return None, []
    group = []
    if active:
        for hazard in hazards:
            back = hazard.get("back", 2.0 * hazard["along"] - hazard["front"])
            if group and hazard["front"] - group[-1]["back"] > rejoin_gap:
                break
            if back + 3.0 > 57.0:
                break
            group.append(dict(hazard, back=back))
    hold_until = max([0.] + [d["back"] + 3.0 for d in group])
    horizon = min(60., max(35., float(minimum_horizon_m), hold_until + transition_hint,
                            float(deferred_distance_m) + 18. if defer_rejoin else 0.))
    candidates = []
    rejected = {"curvature": 0, "clearance": 0, "lane": 0}

    def consider(plan, lane_tol):
        path = plan["path"]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
        if arc[-1] < required or plan["curvature"] > MAX_CONTROL_CURVATURE:
            rejected["curvature"] += 1
            return False
        clear = static_clear_distance(path, ego, stationary, horizon, body_shift_m=body_shift_m)
        if corridor_clearance is not None:
            clear = min(clear, corridor_clearance(path, horizon))
        if clear < required:
            rejected["clearance"] += 1
            return False
        end = min(clear, max(required, hold_until))
        prefix = path[:min(len(path), int(np.searchsorted(arc, end)) + 1)]
        if not lane_ok(prefix, lane_tol):
            rejected["lane"] += 1
            return False
        usable = max(0., clear - 2.)
        cap = math.sqrt((2.5 * .7) ** 2 + 5.0 * usable) - 2.5 * .7
        complete = clear >= max(required, hold_until)
        plan = dict(plan, safe_horizon_m=required, clear_distance_m=clear,
                    speed_cap_mps=cap, activation_m=activation, manoeuvre_horizon_m=hold_until,
                    cluster_count=len(group), hold_until_m=hold_until,
                    replan_needed=clear < horizon - .5, reused=bool(plan.get("reused")))
        continuity = 0.
        if previous_plan is not None and not plan["reused"]:
            old = previous_plan["path"][:, :2]
            old_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(old, axis=0), axis=1))]
            sample = np.linspace(0., min(12., old_s[-1], arc[-1]), 24)
            current = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
            previous = np.column_stack([np.interp(sample, old_s, old[:, i]) for i in (0, 1)])
            continuity = float(np.mean(np.linalg.norm(current - previous, axis=1)))
        score = (4. * plan["curvature"] + 2. * continuity + .08 * (horizon - clear)
                 + (0. if plan["reused"] else .15) - (.5 if plan.get("adjacent_lane") else 0.))
        # A path that reaches only the braking-safe prefix still approaches a
        # known obstruction. Prefer a clear replan over reusing that path, even
        # when the new path differs from the previous steering trajectory.
        candidates.append(((not complete, plan["replan_needed"], score), plan))
        return complete

    def finish():
        # Before commitment, a braking-safe prefix cannot authorize a new pass.
        if not candidates or ((active or startup_rejoin) and previous_plan is None and
                              not any(not rank[0] for rank, _ in candidates)):
            return {"blocked": True, "kind": "bypass" if active else "rejoin",
                    "activation_m": activation, "safe_horizon_m": required,
                    "rejected": rejected}, hazards
        return min(candidates, key=lambda pair: pair[0])[1], hazards

    if (previous_plan is not None and previous_plan["direction"] == committed_direction
            and not (previous_plan.get("kind") == "tunnel_hold" and not active)):
        old = dict(previous_plan, curvature=float(np.max(path_curvatures(previous_plan["path"]))))
        # Reuse only if the saved path still stays out through the next group.
        # A safe short prefix alone must not endorse its old, premature return.
        if not active:
            old["kind"] = "rejoin"
        # A complete fresh footprint/corridor check is authoritative. Route
        # curvature can make lateral projection look like an early return even
        # while the committed world path clears the entire obstacle group.
        if consider(old, 1.9) and not candidates[-1][1]["replan_needed"]:
            return candidates[-1][1], hazards
    if not active and (committed or defer_rejoin) and defer_rejoin and deferred_offset_m >= 1.5:
        if search_expired is not None and search_expired():
            return finish()
        transition = 18.
        hold = max(0., float(deferred_distance_m) - transition)
        path_length = min(60., max(35., float(minimum_horizon_m),
                                   float(deferred_distance_m) + transition))
        path, curvature, _ = build_forced_bypass(
            dense_route, committed_direction, offset_m=deferred_offset_m, hold_m=hold,
            path_length_m=path_length, max_curvature_1pm=MAX_CONTROL_CURVATURE,
            transitions=(10., 12., 14., 16., 18.))
        if path is not None and curvature <= MAX_CONTROL_CURVATURE:
            hold_path = dict(path=path, direction=committed_direction, offset_m=deferred_offset_m,
                hold_m=hold, curvature=curvature, transition_m=transition, kind="tunnel_hold")
            if consider(hold_path, 1.9) and not candidates[-1][1]["replan_needed"]:
                return candidates[-1][1], hazards
        # A failed hold check must never silently publish the centreline. Caller
        # applies its existing stop and retains the last validated path.
        return {"blocked": True, "kind": "tunnel_hold", "direction": committed_direction}, hazards
    if not active and (committed or startup_rejoin):
        for transition in (10., 12., 14., 18., 20., 22., 26.):
            if search_expired is not None and search_expired():
                return finish()
            path, curvature, _ = build_forced_bypass(dense_route, 0, offset_m=0, hold_m=0,
                max_curvature_1pm=MAX_CONTROL_CURVATURE, transitions=(transition,))
            if path is not None:
                consider(dict(path=path, direction=committed_direction, offset_m=0., hold_m=0.,
                              curvature=curvature, transition_m=transition, kind="rejoin"), 1.9)
    else:
        sides = (float(committed_direction), -float(committed_direction)) if side_locked else (1., -1.)
        for side in sides:
            if search_expired is not None and search_expired():
                return finish()
            if side_locked and side != float(committed_direction) and speed > 1.5:
                continue
            if side_locked and side != float(committed_direction) and candidates:
                break
            lane_offset = (adjacent_offsets or {}).get(int(side))
            start_entry = (getattr(adjacent_offsets, "start_m_by_side", {}) or {}).get(int(side), 0.)
            starts = start_values(start_entry)
            start_m = starts[0]
            open_road = int(side) in getattr(adjacent_offsets, "open_road_sides", ())
            # With an HD map, a full bypass uses a permitted adjacent lane.
            # A small in-lane adjustment remains available for edge obstacles.
            choices = ([(lane_offset, True, start_m)] if lane_offset else [])
            if lane_offset and open_road:
                choices += [(offset, True, start_m) for offset in (2.8, 2.2)
                            if offset < lane_offset - .1]
            transitions = (6., 8., 10., 12., 14., 18., 22.) if speed <= 2.5 else (10., 12., 14., 18., 22., 26.)
            found = False
            if lane_offset and isinstance(start_entry, (list, tuple)):
                # Starts given as a sequence: room that may open only ahead of the car
                # (a junction, a lane that widens in) - keep the route, then move over
                # where it begins. Lowest curvature (fastest) first, earliest start
                # first, until a complete, fully clear path is found on this side; the
                # ranking below then compares it with the other side's. Every path
                # still gets the full footprint, corridor and lane checks.
                order = sorted((offset / transition ** 2, start, offset, transition)
                               for offset, _, _ in choices for transition in transitions for start in starts)
                for _, start, offset, transition in order:
                    if search_expired is not None and search_expired():
                        return finish()
                    hold = max(6., hold_until - start - transition)
                    path, curvature, _ = build_forced_bypass(dense_route, side, offset_m=offset,
                        hold_m=hold, max_curvature_1pm=MAX_CONTROL_CURVATURE,
                        transitions=(transition,), start_m=start)
                    if path is None:
                        continue
                    complete = consider(dict(path=path, direction=side, offset_m=offset, hold_m=hold,
                                             curvature=curvature, transition_m=transition, kind="bypass",
                                             start_m=start, open_road=open_road, adjacent_lane=True), 1.5)
                    if complete and not candidates[-1][1]["replan_needed"]:
                        found = True
                        break
                choices = []
            if not found:
                # In-lane adjustments (no adjacent-lane bonus) are tried only when the
                # ordered search found no complete, fully clear path (bounded time).
                choices += ([(1.2, False, 0.), (.9, False, 0.), (.6, False, 0.), (.45, False, 0.)]
                            if adjacent_offsets is not None else
                            [(2.2, False, 0.), (1.6, False, 0.), (1.0, False, 0.)])
            for offset, adjacent, start in choices:
                for transition in transitions:
                    if search_expired is not None and search_expired():
                        return finish()
                    hold = max(6., hold_until - start - transition)
                    path, curvature, _ = build_forced_bypass(dense_route, side, offset_m=offset,
                        hold_m=hold, max_curvature_1pm=MAX_CONTROL_CURVATURE,
                        transitions=(transition,), start_m=start)
                    if path is not None:
                        complete = consider(dict(path=path, direction=side, offset_m=offset, hold_m=hold,
                                      curvature=curvature, transition_m=transition, kind="bypass",
                                      start_m=start, open_road=open_road and adjacent,
                                      adjacent_lane=adjacent), 1.5 if adjacent else 1.9)
                        if start > 0. and complete and not candidates[-1][1]["replan_needed"]:
                            return candidates[-1][1], hazards
    # A braking-safe prefix is useful only after a bypass is already under way.
    # Before commitment it can end beside the obstacle, and latching its side
    # prevents a full adjacent-lane route found a few metres later from being used.
    return finish()

def start_values(value):
    """start_m_by_side entry -> ascending starts (m): one number (bypass from that
    start, first complete transition taken) or a sequence (ordered search above)."""
    values = sorted(float(v) for v in (value if isinstance(value, (list, tuple)) else (value,)))
    return values or [0.]

def local_xy(x, y, ego):
    yaw = math.radians(float(ego["heading_deg"]))
    dx, dy = x - float(ego["x"]), y - float(ego["y"])
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
