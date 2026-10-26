"""Complete static passes with a short, measured centre-line excursion.

This normal-road policy keeps the ordinary pedestrian and collision guards.
The 2.5 s admission budget reserves margin below the contest's 3 s penalty
interval. Body corners plus 5 cm conservatively cover wheel contact; the
physical road check retains its separate 20 cm body margin.
"""
import math
import threading
import time
from functools import wraps

import numpy as np
from scipy.interpolate import BPoly, CubicSpline

from ego_body import EGO_FRONT_M, EGO_REAR_M, REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor


RETURN_BUDGET_S = 2.5
MAX_ENCROACHMENT_M = .75
PASS_MPS = 25./3.6
FAST_PASS_MPS = 30./3.6
TIGHT_PASS_MPS = PASS_MPS
# Reserve a 20% tracking loss after applying the measured real-time factor.
# Rolling static passes track 23–25 km/h against a 25 km/h command; multiplying
# that measured speed by .75 again produced a 2.60 s forecast from 1.95 s at
# measured pace, repeatedly revoking a rolling pass before line contact.
# The independent 2.5 s budget still reserves .5 s below the contest interval.
TRACKING_SPEED_FACTOR = .8
PROFILE_RETURN_BUDGET_S = 2.75
PROFILE_TRACKING_FACTOR = .9
PROFILE_ADMISSION_RESERVE_S = .25


def _metric_profile_path(path):
    """A projected millimetre prefix is not a new physical steering step."""
    path = np.asarray(path, float)[:, :2]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    keep = np.r_[True, np.diff(arc) > 1e-6]
    arc, path = arc[keep], path[keep]
    if len(path) < 3 or arc[-1] <= 0.:
        raise ValueError('static motion profile path unavailable')
    stations = np.linspace(0., arc[-1], max(3, int(math.ceil(arc[-1]/.5))+1))
    return np.column_stack([np.interp(stations, arc, path[:, k]) for k in (0, 1)])


def static_timing_profile(path, ego, ceiling_mps=PASS_MPS):
    """Travel time under local wheel/curve limits and conservative acceleration.

    The 1.5 m/s² envelope stays below the measured 2.1–3 m/s² launch. A separate
    10% tracking reserve and .25 s admission margin remain in the timed proof.
    """
    from gui import path_curvatures, BYPASS_ALAT_MPS2
    path = _metric_profile_path(path)
    arc, gradient = _path_steering_profile(path, ego)
    limits = np.full(len(path), max(0., float(ceiling_mps)))
    limits[1:-1] = np.minimum(limits[1:-1], np.sqrt(
        BYPASS_ALAT_MPS2/np.maximum(path_curvatures(path), 1e-6)))
    wheel_limits = math.radians(30.)*.8/np.maximum(gradient, 1e-6)
    limits[:-1] = np.minimum(limits[:-1], wheel_limits)
    limits[1:] = np.minimum(limits[1:], wheel_limits)
    for i in range(len(limits)-2, -1, -1):
        limits[i] = min(limits[i], math.sqrt(limits[i+1]**2+3.*(arc[i+1]-arc[i])))
    speed = limits.copy()
    speed[0] = min(speed[0], max(0., float(ego.get('speed_kmh', 0.)))/3.6)
    for i in range(1, len(speed)):
        speed[i] = min(speed[i], math.sqrt(speed[i-1]**2+3.*(arc[i]-arc[i-1])))
    seconds = np.r_[0., np.cumsum(2.*np.diff(arc)/np.maximum(speed[1:]+speed[:-1], 1e-6))]
    return dict(station=arc, seconds=seconds, speed_mps=speed, limits_mps=limits,
                command_cap_mps=static_speed_profile(path, ego, ceiling_mps=ceiling_mps)['cap_mps'])


def _path_steering_profile(path, ego):
    """Signed road-wheel changes over the exact candidate's physical stations."""
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    tangent = np.gradient(path, axis=0)
    heading = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
    wheel = np.arctan(3.90*np.gradient(heading,
        np.maximum.accumulate(arc)+np.arange(len(arc))*1e-9))
    if ego.get('wheel_angle') is not None and math.isfinite(float(ego['wheel_angle'])):
        wheel[0] = math.radians(float(ego['wheel_angle']))
    return arc, np.abs(np.diff(wheel)/np.maximum(np.diff(arc), 1e-6))


def static_speed_profile(path, ego, *, ceiling_mps=FAST_PASS_MPS):
    """Current speed with gentle braking before each later turn and wheel-rate limit."""
    from gui import path_curvatures, BYPASS_ALAT_MPS2
    try:
        path = _metric_profile_path(path)
    except (TypeError, ValueError, IndexError, OverflowError):
        return dict(cap_mps=0., curve_cap_mps=0., steering_cap_mps=0.)
    arc, gradient = _path_steering_profile(path, ego)
    speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
    curve = np.asarray(path_curvatures(path), float)
    if not np.isfinite(np.r_[arc, gradient, curve]).all():
        return dict(cap_mps=0., curve_cap_mps=0., steering_cap_mps=0.)
    reaction_distance = max(speed, ceiling_mps)*.7
    curve_allowed = BYPASS_ALAT_MPS2/np.maximum(curve, 1e-4)+3.*np.maximum(
        0., arc[1:-1]-reaction_distance)
    steering_limit = math.radians(30.)*.8/np.maximum(gradient, 1e-6)
    steering_allowed = steering_limit**2+3.*np.maximum(0., arc[1:]-reaction_distance)
    curve_at, steering_at = int(np.argmin(curve_allowed)), int(np.argmin(steering_allowed))
    curve_cap, steering_cap = math.sqrt(float(curve_allowed[curve_at])), math.sqrt(float(steering_allowed[steering_at]))
    return dict(cap_mps=min(ceiling_mps, curve_cap, steering_cap),
        curve_cap_mps=curve_cap, steering_cap_mps=steering_cap,
        curve_station_m=float(arc[1:-1][curve_at]), steering_station_m=float(arc[1:][steering_at]),
        reaction_s=.7, decel_mps2=1.5)


def normal_static_area(lanes, ego, horizon_m=60.):
    """Keep the timed exception clear of tunnel and highway transitions."""
    try:
        station = float(lanes.route_progress(ego))
        sections = lanes.rules().get('sections') or []
        if not math.isfinite(station):
            return False
        for section in sections:
            if section.get('type') not in ('tunnel', 'overpass'):
                continue
            start, end = float(section['start_m']), float(section['end_m'])
            # Check the entire proposed return, with room before the portal.
            if station+horizon_m >= start-20. and station <= end+20.:
                return False
        return True
    except (AttributeError, TypeError, ValueError, KeyError):
        return False


def _known_wheel_angle(ego):
    try:
        return float(ego.get('wheel_angle', math.nan))
    except (TypeError, ValueError):
        return math.nan


def checked_static_backoff(lanes, record, *, distances=(1.5, 2.5)):
    """Find a bounded reverse with a feasible forward return at its endpoint.

    The endpoint is a planning hypothesis, never a measured motion permission.
    Reverse motion still needs current sensor checks at publication, and forward
    release still requires a new complete plan at the actual reached pose.
    """
    from ego_body import body_corners, reverse_poses, reverse_sweep_hits
    from lattice_planner import object_points
    from tunnel_runtime import rotation, corridor_at_pose, swept_clearance
    from perception.road_static_calibration import has_source_returns
    from gui import static_clear_distance
    try:
        ego = dict(record['ego'])
        if (ego.get('measured') is not True or
                not 0. <= float(ego.get('age_s', math.inf)) <= .2 or
                abs(float(ego['speed_kmh'])) > .5 or not record['ready'] or
                record.get('contact') and record['contact'].get('block_reverse') or
                (record.get('policy') or {}).get('entered_at') is not None):
            return None
        route = np.asarray(record['route'], float)[:, :2]
        points = np.asarray(record['points'], float).reshape(-1, 2)
        if (len(route) < 12 or not np.isfinite(route).all() or
                not np.isfinite(points).all()):
            return None
        objects = list(record['objects'])
        if not objects or any(o[4] in ('pedestrian', 'npc') or abs(float(o[5])) >= .5 for o in objects):
            return None
        ahead = np.vstack((np.zeros((1, 2)), route[route[:, 0] > .05]))
        if (swept_clearance(ahead, points, 7., .25, lateral_margin=.25,
                           body_shift_m=REAR_AXLE_SHIFT_M) > 6. or
                static_clear_distance(ahead, ego, record['static'], 7.,
                                      body_shift_m=REAR_AXLE_SHIFT_M) > 6. or
                not any(has_source_returns(obj, points, ego) for obj in record['static'])):
            return None
        value = record['legal']
        legal = LaneCorridor(np.asarray(value['route']), np.asarray(value['left']),
            np.asarray(value['right']), zones=value['zones'],
            intersection_mask=np.asarray(value['intersection_mask']),
            body_shift_m=REAR_AXLE_SHIFT_M)
        tangent = route[min(10, len(route)-1)]-route[0]
        error = math.atan2(float(tangent[1]), float(tangent[0]))
        if abs(error) > math.radians(15.):
            return None
        heading = float(ego['heading_deg'])+math.degrees(error)
        axes = rotation(ego)
        world_route = route@axes.T+[ego['x'], ego['y']]
        world_points = points@axes.T+[ego['x'], ego['y']]
        boxed = object_points(objects, ego) if objects else np.empty((0, 2))
        reverse_points = np.vstack((points, boxed))
        for distance in distances:
            distance = float(distance)
            if not 0. < distance <= 2.5:
                continue
            if reverse_sweep_hits(reverse_points, error, distance, margin=.25).any():
                continue
            poses = reverse_poses(error, distance)
            corners = np.vstack([body_corners(x, y, yaw, margin=.2)
                                 for x, y, yaw in poses])
            _, at = legal.tree.query(corners)
            lateral = np.sum((corners-legal.route[at])*legal.normal[at], axis=1)
            if np.any(lateral > legal.left[at]) or np.any(lateral < legal.right[at]):
                continue
            origin = np.asarray([ego['x'], ego['y']])-axes@np.asarray([
                distance*math.cos(error), distance*math.sin(error)])
            endpoint = dict(ego, x=float(origin[0]), y=float(origin[1]),
                            heading_deg=heading, wheel_angle=0.)
            endpoint_axes = rotation(endpoint)
            future_route = (world_route-origin)@endpoint_axes
            future_route = future_route[future_route[:, 0] >= 0.]
            future_points = (world_points-origin)@endpoint_axes
            future_legal = corridor_at_pose(legal, ego, endpoint)
            plan, proof = StaticLanePass().plan(lanes, endpoint, future_route,
                objects, record['static'], future_points, future_legal, None,
                record['now'], record['time_scale'], True, budget_s=.11,
                side_margin_m=record['side_margin_m'], contact=None,
                legal_checked=True, reserve_source_bounds=record['reserve_source_bounds'],
                slalom_first=record['slalom_first'])
            if plan is not None and not plan.get('blocked') and proof.get('ok'):
                return dict(distance_m=distance, heading_deg=heading,
                    source_stamp=ego['stamp'], endpoint_xy=origin.tolist(),
                    forward_clear_m=plan['clear_distance_m'],
                    forward_contact_s=proof.get('contact_s', 0.))
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return None
    return None


def measured_curvature_paths(route, ego, static):
    """Additional forward proposals whose measured steering changes early.

    An endpoint offset alone can postpone the turn until the front bumper
    already reaches a close body. Integrate a short continuous wheel change,
    then join the surveyed route with matching position, slope and curvature.
    These are proposals only: the caller checks every body, raw return, map
    boundary, actuator limit and complete timed return on the exact path.
    """
    wheel = _known_wheel_angle(ego)
    if not math.isfinite(wheel):
        return
    from gui import RosPathPublisher
    from highway_planner import local_points
    route = np.asarray(route, float)[:, :2]
    if len(route) < 4 or np.any(np.diff(route[:, 0]) <= 1e-6):
        return
    reference = CubicSpline(route[:, 0], route[:, 1])
    bodies = local_points(np.asarray([obj[:2] for obj in static], float).reshape(-1, 2), ego)
    nearby = bodies[(bodies[:, 0] > 0.) & (np.abs(bodies[:, 1]) < 4.)]
    if not len(nearby):
        return
    nearest = nearby[int(np.argmin(nearby[:, 0]))]
    side = -1. if nearest[1] > 0. else 1.
    rolling = float(ego['speed_kmh']) > 5.
    magnitudes = ((.02, .035, .05, .07, .09) if rolling else
                  (.17, .13, .09, .05) if nearest[0] < 6. else (.05, .09, .13, .17))
    ramps = (3., 4., 6., 8.) if rolling else (.75, .5, .25)
    initial = math.tan(math.radians(wheel))/3.90
    # Moving cars need both gentle sides before stronger turns. At rest, try
    # the short ramps away from the near body before its obstructed side.
    settings = (((sign, magnitude, hold, ramp) for magnitude in magnitudes
                 for hold in (0., .5, 1., 2.) for ramp in ramps for sign in (side, -side))
                if rolling else
                ((sign, magnitude, hold, ramp) for sign in (side, -side)
                 for magnitude in magnitudes for hold in (0., 2., .5, 1.) for ramp in ramps))
    for sign, magnitude, hold, ramp in settings:
        length = 2.*ramp+hold
        station = np.arange(0., length+.025, .05)
        target = sign*magnitude
        curve = np.where(station < ramp,
            initial+(target-initial)*station/ramp,
            np.where(station <= ramp+hold, target, target*(length-station)/ramp))
        heading = np.r_[0., np.cumsum(.5*(curve[:-1]+curve[1:])*np.diff(station))]
        mid = .5*(heading[:-1]+heading[1:])
        x = np.r_[0., np.cumsum(np.cos(mid)*np.diff(station))]
        y = np.r_[0., np.cumsum(np.sin(mid)*np.diff(station))]
        start = [y[-1], math.tan(heading[-1]), 0.]
        for goal in (18., 21., 24., 30.):
            if goal > route[-1, 0] or goal <= x[-1]+2.:
                continue
            for offset in (0., sign*.45, sign*.9, sign*1.35,
                           -sign*.45, -sign*.9, -sign*1.35):
                end = [reference(goal)+offset, reference(goal, 1), reference(goal, 2)]
                join = BPoly.from_derivatives([x[-1], goal], [start, end])
                tail = np.arange(x[-1]+.05, goal+.01, .05)
                yield RosPathPublisher.resample(np.column_stack((np.r_[x, tail], np.r_[y, join(tail)])))


def _rank_static_targets(route, ego, objects, corridor, targets, side_margin_m=.25):
    """Spend the bounded search on plausible constant offsets first.

    This is only proposal ordering. The exact transformed path still passes
    every swept-body, raw-return, road, steering and return-time check.
    """
    from lattice_planner import object_points, HALF_W
    targets = tuple(targets)
    groups = [object_points([obj], ego, spacing=10.) for obj in objects]
    if not groups:
        return [(target, (False, False, 0., abs(target))) for target in targets]
    points = np.vstack(groups)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    _, at = corridor.tree.query(points)
    along = arc[at]
    lateral = np.sum((points-corridor.route[at])*corridor.normal[at], axis=1)
    selected = (along > 0.) & (along < arc[-1]-1.)
    if not np.any(selected):
        return [(target, (False, False, 0., abs(target))) for target in targets]
    # Perimeter endpoints reserve the whole lateral span of every box.
    # Ranking against individual endpoints alone could prefer its interior.
    cuts = np.r_[0, np.cumsum([len(group) for group in groups])]
    rows = [(float(lateral[a:b].min()), float(lateral[a:b].max()))
            for a, b in zip(cuts[:-1], cuts[1:]) if np.any(selected[a:b])]
    lower = float(np.max(corridor.right+HALF_W+.2))
    upper = float(np.min(corridor.left-HALF_W-.2))
    def rank(target):
        obstacle_gap = min(max(lo-target, target-hi) for lo, hi in rows)-HALF_W-side_margin_m
        road_gap = min(target-lower, upper-target)
        return (road_gap < -.05, obstacle_gap < -.01,
                max(0., -obstacle_gap), abs(target))
    return sorted(((target, rank(target)) for target in targets), key=lambda item: item[1])


def ordered_static_targets(route, ego, objects, corridor, targets, side_margin_m=.25):
    return tuple(target for target, _ in _rank_static_targets(
        route, ego, objects, corridor, targets, side_margin_m))


def _reachable_join_dynamics(path, ego, max_curvature, lateral_accel, *, steering_rate_fraction=1.,
                             check_horizon_m=None):
    """Check the canonical path with physical stations and measured steering."""
    from gui import path_curvatures
    path = np.asarray(path, float)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    if len(path) < 4 or not np.isfinite(path).all() or np.any(np.diff(arc) < 1e-6):
        return False, False
    curve = np.asarray(path_curvatures(path))
    edge, following = path[1:-1]-path[:-2], path[2:]-path[1:-1]
    signed = curve*np.sign(edge[:, 0]*following[:, 1]-edge[:, 1]*following[:, 0])
    steering = np.r_[math.radians(float(ego['wheel_angle'])), np.arctan(3.90*signed)]
    steering_s = np.r_[0., arc[1:-1]]
    rate = np.abs(np.diff(steering)/np.diff(steering_s))
    speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
    reachable = np.sqrt(np.maximum(0., speed*speed-5.*np.maximum(
        0., steering_s[1:]-speed*.2)))
    lateral = np.maximum(0., speed*speed-3.*np.maximum(
        0., arc[1:-1]-speed*.7))*curve
    bad_curve = (curve > max_curvature+1e-9) | (lateral > lateral_accel+.05)
    rate_limit = math.radians(30.)*steering_rate_fraction
    bad_rate = reachable*rate > rate_limit+1e-6
    # Also retain the caller's gradient-heading discretization. A proposed
    # repair must pass both checks; changing derivative estimators is not a
    # reason to waive the existing physical steering-rate guard.
    tangent = np.gradient(path, axis=0)
    heading = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
    wheel = np.arctan(3.90*np.gradient(heading, arc+np.arange(len(arc))*1e-9))
    wheel[0] = math.radians(float(ego['wheel_angle']))
    outer_rate = np.abs(np.diff(wheel)/np.diff(arc))
    outer_speed = np.sqrt(np.maximum(0., speed*speed-5.*np.maximum(
        0., arc[1:]-speed*.2)))
    bad_outer_rate = outer_speed*outer_rate > rate_limit+1e-6
    if check_horizon_m is not None:
        bad_curve &= arc[1:-1] <= check_horizon_m
        bad_rate &= steering_s[1:] <= check_horizon_m
        bad_outer_rate &= arc[1:] <= check_horizon_m
    early = bool(np.any(bad_curve & (arc[1:-1] <= 2.))
                 or np.any(bad_rate & (steering_s[1:] <= 2.))
                 or np.any(bad_outer_rate & (arc[1:] <= 2.)))
    return not bool(np.any(bad_curve) or np.any(bad_rate) or np.any(bad_outer_rate)), early


def measured_initial_join(path, ego, *, max_curvature, lateral_accel, target_mps=None):
    """Repair an artificial origin kink without weakening any admission guard.

    The localization EgoVehicleStatus feed supplies signed road-wheel degrees;
    the competition follower explicitly uses degrees and a 3.90 m wheelbase.
    Unknown or invalid measurements retain the original conservative candidate.
    A quintic joins the measured axle, forward heading and initial curvature to
    the existing path's position, tangent and curvature. Only a fully reachable
    canonical result is returned; body, raw-scan, map and time checks remain the
    caller's responsibility and run on this exact replacement geometry.
    """
    from gui import RosPathPublisher
    try:
        wheel = float(ego['wheel_angle'])
        speed = float(ego['speed_kmh'])
        stamp = float(ego['stamp'])
        age = float(ego.get('age_s', math.inf))
        if (ego.get('measured') is not True or ego.get('src') == 'file'
                or not math.isfinite(stamp) or stamp <= 0.
                or not math.isfinite(age) or not 0. <= age <= .25
                or not math.isfinite(wheel) or abs(wheel) > 40.
                or not math.isfinite(speed) or speed < 0.):
            return path
        original = np.asarray(path, float)
        check_ego, rate_fraction = ego, 1.
        if target_mps is not None:
            target = float(target_mps)
            if not math.isfinite(target) or target < 0.:
                return path
            # A join reachable only at today's crawl speed otherwise renews
            # its own crawl cap forever. Generate additional geometry for the
            # intended pace, including the final command's steering reserve.
            # Final admission and publication retain actual measured speed.
            check_ego = dict(ego, speed_kmh=max(speed, target*3.6))
            rate_fraction = .8
        valid, early_failure = _reachable_join_dynamics(
            original, check_ego, max_curvature, lateral_accel,
            steering_rate_fraction=rate_fraction,
            check_horizon_m=12. if target_mps is not None else None)
        if valid or (not early_failure and target_mps is None) or np.any(np.diff(original[:, 0]) <= 1e-6):
            return path
        reference = CubicSpline(original[:, 0], original[:, 1])
        initial_curvature = math.tan(math.radians(wheel))/3.90
        for length in (4., 6., 8., 10., 12.):
            if original[-1, 0] < length+1.:
                break
            # y(x) has y(0)=y'(0)=0 and y''(0)=measured curvature.
            coeff = np.zeros(6)
            coeff[2] = .5*initial_curvature
            endpoint = np.array([reference(length), reference(length, 1), reference(length, 2)])
            matrix = np.array([[length**3, length**4, length**5],
                               [3*length**2, 4*length**3, 5*length**4],
                               [6*length, 12*length**2, 20*length**3]])
            coeff[3:] = np.linalg.solve(matrix, endpoint-np.array([
                coeff[2]*length**2, 2*coeff[2]*length, 2*coeff[2]]))
            x = np.r_[np.arange(0., length, .1), length]
            prefix = np.column_stack((x, np.polynomial.polynomial.polyval(x, coeff)))
            joined = np.vstack((prefix, original[original[:, 0] > length]))
            canonical = RosPathPublisher.resample(joined)
            if _reachable_join_dynamics(canonical, check_ego, max_curvature, lateral_accel,
                                       steering_rate_fraction=rate_fraction,
                                       check_horizon_m=12. if target_mps is not None else None)[0]:
                return canonical
        if target_mps is not None:
            # Prefer geometry reachable at the intended pace. If that pace
            # cannot be reached here, still try a join at the measured speed
            # before returning the original origin kink. Final admission
            # recomputes the speed cap, steering limits and complete body,
            # raw, lane and return-time checks on this replacement path.
            return measured_initial_join(path, ego, max_curvature=max_curvature,
                lateral_accel=lateral_accel)
    except (KeyError, TypeError, ValueError, IndexError, OverflowError, np.linalg.LinAlgError):
        pass
    return path


def _locked(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return call


def body_stations(path, legal, step=.1):
    path = np.asarray(path, float)[:, :2]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    if len(path) < 3 or not np.isfinite(path).all() or arc[-1] < 3.:
        raise ValueError('invalid static-pass path')
    stations = np.linspace(0., arc[-1], int(math.ceil(arc[-1]/step))+1)
    xy = np.column_stack([np.interp(stations, arc, path[:, i]) for i in (0, 1)])
    tangent = np.gradient(xy, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    tangent[0] = [1., 0.]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    corners = np.stack([xy+a*tangent+b*normal for a in
        (-EGO_REAR_M, REAR_AXLE_SHIFT_M, EGO_FRONT_M) for b in (-.996, .996)], axis=1)
    _, at = legal.tree.query(corners.reshape(-1, 2))
    lateral = np.sum((corners.reshape(-1, 2)-legal.route[at])*legal.normal[at], axis=1)
    left = (lateral-legal.left[at]).reshape(len(xy), -1).max(axis=1)
    right = (legal.right[at]-lateral).reshape(len(xy), -1).max(axis=1)
    return stations, left, right


def opposing_corridor(lanes, ego, route, legal):
    """Borrow at most 75 cm of a surveyed opposite lane on the left only.

    A whitelist governs ordinary lanes; this explicitly requested exception
    may touch an unlisted opposing lane. Explicit block overrides, no-go zones,
    junction links and the physical map boundary remain authoritative.
    """
    if legal is None or not getattr(lanes, 'n_links', 0):
        return None
    from highway_planner import world_points
    route = np.asarray(route, float)[:, :2]
    world = world_points(route, ego)
    tangent = np.gradient(world, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    left = legal.left.copy()
    links = lanes.links
    if hasattr(lanes, 'tree') and hasattr(lanes, 'link_of'):
        # The full K-city map has thousands of links. Only links whose
        # surveyed points are near this 60 m return can form its neighbour.
        nearby = lanes.tree.query_ball_point(world[::10], r=7.)
        indices = {int(lanes.link_of[i]) for group in nearby for i in group}
        links = [lanes.links[i] for i in sorted(indices)]
    for link in links:
        if (str(link.get('link_type')) == '1' or lanes.link_rule(link['idx']).get('block')
                or link['idx'] not in lanes.geometry):
            continue
        p = lanes.geometry[link['idx']]
        if np.linalg.norm(p[-1]-p[0]) < 10.:
            continue
        distance, at = lanes.link_trees[link['idx']].query(world)
        direction = p[np.minimum(at+1, len(p)-1)]-p[np.maximum(at-1, 0)]
        direction /= np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-9)
        lateral = np.sum((p[at]-world)*normal, axis=1)
        half = .5*np.clip(min(float(link.get('width_start', 3.5)),
                              float(link.get('width_end', 3.5))), 2.5, 4.5)
        valid = ((distance < 5.) & (np.sum(direction*tangent, axis=1) < -.95)
                 & (lateral > legal.left+.2)
                 & (lateral-half <= legal.left+MAX_ENCROACHMENT_M)
                 & ~legal.intersection_mask)
        left[valid] = np.maximum(left[valid], np.minimum(
            legal.left[valid]+MAX_ENCROACHMENT_M, lateral[valid]+half-.2))
    if np.max(left-legal.left) < .1:
        return None
    return LaneCorridor(route, left, legal.right.copy(), zones=legal.zones,
                        body_shift_m=REAR_AXLE_SHIFT_M)


class StaticLanePass:
    def __init__(self):
        self._lock = threading.RLock()
        self.legal = None
        self.origin = None
        self.entered_at = None
        self.inside_frames = 0
        self.last_stamp = None
        self.last_inside_at = None
        self.return_station_m = None
        self.time_scale = 1.
        self.last_check = {}
        self.finished = False
        self.physical = None
        self.physical_origin = None
        self._unused_inside_stamps = []
        self._completed_geometry = None

    @_locked
    def retire_completed_geometry(self, now):
        """Bridge an already completed path while its replacement is searched.

        A publisher may still hold the last world curve for at most two
        seconds. Its old surveyed corridor proves in-lane geometry only;
        this receipt never permits another boundary contact or time budget.
        """
        if self.finished and self.legal is not None and self.entered_at is None:
            self._completed_geometry = (self.legal, dict(self.origin), now+2.)

    @_locked
    def corridor(self, ego, fallback=None):
        if self.legal is None:
            return fallback
        from tunnel_runtime import corridor_at_pose
        return corridor_at_pose(self.legal, self.origin, ego)

    @_locked
    def observe(self, ego, now):
        if self.legal is None or self.finished:
            return
        try:
            stamp = float(ego['stamp'])
            if (not math.isfinite(stamp) or stamp <= 0.
                    or self.last_stamp is not None and stamp <= self.last_stamp):
                return
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        legal = self.corridor(ego)
        pose_path = np.array([[0., 0.], [2., 0.], [4., 0.]])
        _, left, right = body_stations(pose_path, legal)
        outside = max(float(left[0]), float(right[0])) > .03
        observed_at = now-max(0., float(ego.get('age_s', 0.)))
        if outside:
            self.finished = False
            if self.entered_at is None:
                # The crossing may have happened during a sensor blackout.
                # Do not grant a fresh budget when the stream resumes outside.
                self.entered_at = min(observed_at-.1,
                    self.last_inside_at if self.last_inside_at is not None else observed_at-.1)
            self.inside_frames = 0
        else:
            self.last_inside_at = observed_at
            if self.entered_at is not None:
                self.inside_frames += 1
                if self.inside_frames >= 2:
                    self.entered_at = None
                    self.finished = True
                    self.inside_frames = 0
        self.last_stamp = stamp

    @_locked
    def release_unused(self, ego, now, *, using_exception=False):
        """Retire an unentered proposal after two measured inside observations.

        A later ordinary plan can replace a proposed excursion before line
        contact starts. Its frozen, finite corridor must then stop observing
        unrelated bends. An actual excursion keeps its original return clock.
        """
        if self.legal is None or self.finished:
            return
        try:
            stamp, age = float(ego['stamp']), float(ego.get('age_s', 0.))
            if not math.isfinite(stamp) or not 0. <= age <= .3:
                return
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        self.observe(ego, now)
        if using_exception or self.entered_at is not None or self.finished:
            self._unused_inside_stamps.clear()
            return
        if self._unused_inside_stamps and stamp <= self._unused_inside_stamps[-1]:
            return
        self._unused_inside_stamps.append(stamp)
        if len(self._unused_inside_stamps) >= 2:
            self.finished = True
            self.last_check = dict(ok=True, retired=True,
                                  reason='unused static excursion completed inside its original lane')

    @_locked
    def assess(self, path, ego, cap, legal, now, objects=(), ready=True, *, motion_profile=False,
               profile_ceiling_mps=None):
        """Bound each continuous body-contact interval at the issued pace."""
        legal = self.corridor(ego, legal)
        if legal is None:
            completed = self._completed_geometry
            if motion_profile and completed is not None and now < completed[2]:
                from tunnel_runtime import corridor_at_pose
                previous = corridor_at_pose(completed[0], completed[1], ego)
                _, left, right = body_stations(path, previous)
                if np.all(left <= .03) and np.all(right <= .03):
                    return dict(ok=True, contact_s=0., encroachment_m=0.,
                                completed_return=True)
            return dict(ok=False, reason='static pass has no original lane geometry')
        stations, left, right = body_stations(path, legal)
        outside = left > .03
        if np.any(right > .03) or np.max(left) > MAX_ENCROACHMENT_M+.03:
            return dict(ok=False, reason='static pass exceeds bounded lane exception')
        if not outside.any():
            return dict(ok=True, contact_s=0., encroachment_m=0.)
        if outside[0] and self.legal is None:
            return dict(ok=False, reason='existing line contact has no observed start time')
        if not ready or left[-1] > .03 or np.any(outside[stations > stations[-1]-3.]):
            return dict(ok=False, reason='static pass lacks a complete observed return')
        # Pedestrians retain their existing policy. Opposing or unknown-speed
        # vehicles anywhere near the borrowed lane veto the exception; later
        # body/traffic checks still examine all directions on the final path.
        from highway_planner import local_points
        for obj in objects:
            f, l = local_points(np.asarray(obj[:2])[None], ego)[0]
            if obj[4] == 'pedestrian' and -5. < f < 60. and abs(l) < 6.:
                return dict(ok=False, reason='person near static lane exception')
            if obj[4] == 'npc' and -25. < f < 120.:
                at = int(legal.tree.query([f, l])[1])
                lateral = float((np.array([f, l])-legal.route[at])@legal.normal[at])
                if legal.left[at]-1.2 < lateral < legal.left[at]+5.:
                    return dict(ok=False, reason='opposing lane vehicle or unknown body')
        measured_speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
        pace = min(measured_speed, max(0., cap))
        pace *= min(1., max(0., self.time_scale))*TRACKING_SPEED_FACTOR
        returning = bool(outside[0] and self.entered_at is not None
                         and self.return_station_m is not None)
        profile = None
        if motion_profile:
            if (ego.get('measured') is not True or not 0. <= float(ego.get('age_s', math.inf)) <= .2
                    or not math.isfinite(float(ego.get('wheel_angle', math.nan)))
                    or float(ego.get('stamp', 0.)) <= 0. or cap <= 0. or self.time_scale <= 0.):
                return dict(ok=False, reason='static launch measured motion unavailable')
            ceiling = PASS_MPS
            profile = static_timing_profile(path, ego)
            # A carried command and its freshly projected curve can differ
            # slightly as measured steering changes between publications.
            # The 10% timing reserve covers a command lag up to 5%; treating
            # that small local difference as a ceiling on the entire future
            # profile repeatedly revoked an otherwise stationary launch.
            # Larger independent reductions still bound every future station.
            if profile_ceiling_mps is not None:
                ceiling = float(profile_ceiling_mps)
                if not math.isfinite(ceiling) or not 0. < ceiling <= PASS_MPS:
                    return dict(ok=False, reason='static motion ceiling unavailable')
            elif cap < .95*profile['command_cap_mps']:
                ceiling = cap
            profile = static_timing_profile(path, ego, ceiling_mps=ceiling)
            pace = max(.01, cap)*min(1., self.time_scale)*PROFILE_TRACKING_FACTOR
        if pace < .5:
            if (not returning and profile is None) or cap <= 0. or self.time_scale <= 0.:
                return dict(ok=False, reason='insufficient rolling speed for bounded return')
            # A necessary traffic stop must not prevent restarting the same
            # checked return. This never authorizes a new lane excursion.
            pace = cap*min(1., self.time_scale)*TRACKING_SPEED_FACTOR
        edges = np.diff(np.r_[False, outside, False].astype(int))
        starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)-1
        intervals = []
        for start, end in zip(starts, ends):
            # Two distinct clear observations are needed by observe(). A tiny
            # geometric gap cannot grant a new time budget between frames.
            if intervals and (stations[start]-stations[intervals[-1][1]])/pace < .25:
                intervals[-1] = (intervals[-1][0], end)
            else:
                intervals.append((start, end))
        if returning:
            # No new time credit until two actual inside observations. Finish
            # the original return even if tiny sampled gaps split its contact.
            intervals = [(intervals[0][0], intervals[-1][1])]
        longest = 0.
        for start, end in intervals:
            first, last = max(0, start-1), min(len(stations)-1, end+1)
            if profile is not None:
                entry, leave = max(0., stations[first]-.1), stations[last]+.1
                duration = float(np.interp(leave, profile['station'], profile['seconds'])-
                    np.interp(entry, profile['station'], profile['seconds']))/(
                        min(1., self.time_scale)*PROFILE_TRACKING_FACTOR)
            else:
                duration = float(stations[last]-stations[first]+.2)/pace
            if start == 0 and self.entered_at is not None:
                duration += max(0., now-self.entered_at)
            longest = max(longest, duration)
        # Freeze where the complete body must be back in the original lane.
        # The transformed corridor retains the original route station order.
        last = min(len(stations)-1, intervals[-1][1]+1)
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        return_xy = [np.interp(stations[last], arc, np.asarray(path)[:, k]) for k in (0, 1)]
        route_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(legal.route, axis=0), axis=1))]
        nearest = int(legal.tree.query(return_xy)[1])
        segments = np.clip([nearest-1, nearest], 0, len(legal.route)-2)
        delta = legal.route[segments+1]-legal.route[segments]
        length_sq = np.maximum(1e-12, np.sum(delta*delta, axis=1))
        fraction = np.clip(np.sum((return_xy-legal.route[segments])*delta, axis=1)/length_sq, 0., 1.)
        projected = legal.route[segments]+fraction[:, None]*delta
        at = int(np.argmin(np.linalg.norm(projected-return_xy, axis=1)))
        return_station = float(route_arc[segments[at]]+fraction[at]*math.sqrt(length_sq[at]))
        if (self.entered_at is not None and self.return_station_m is not None
                and return_station > self.return_station_m+.25):
            return dict(ok=False, reason='static pass extends its committed return point')
        # Once physical contact has started, braking solely for a missed time
        # budget prolongs the violation. Continue the checked, fixed return;
        # traffic, raw returns and physical road bounds still veto movement.
        budget = PROFILE_RETURN_BUDGET_S if profile is not None else RETURN_BUDGET_S
        late_return = returning and longest > budget
        allowed = longest <= budget or returning
        return dict(ok=allowed, contact_s=longest,
            encroachment_m=max(0., float(left.max())),
            launch_ready=bool(profile is not None and allowed),
            profile_ceiling_mps=ceiling if profile is not None else None,
            return_station_m=return_station, late_return=bool(late_return and allowed),
            reason=('continuing checked return after static time budget exceeded' if late_return and allowed else
                    None if longest <= budget else
                    'static return exceeds remaining 2.75 s profile budget' if profile is not None else
                    'static return exceeds remaining 2.5 s budget'))

    @_locked
    def command_check(self, path, ego, target, now, *, motion_profile=False, profile_ceiling_mps=None):
        self.observe(ego, now)
        from planning.highway.path_continuity import projected_suffix
        try:
            path = projected_suffix(path)
            check = self.assess(path, ego, target, self.corridor(ego), now,
                                motion_profile=motion_profile, profile_ceiling_mps=profile_ceiling_mps)
            if self.physical is not None:
                from tunnel_runtime import corridor_at_pose
                physical = corridor_at_pose(self.physical, self.physical_origin, ego)
                horizon = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                if physical.clear_distance(path, horizon) < horizon-.25:
                    check = dict(ok=False, reason='static pass left its surveyed road envelope')
        except (TypeError, ValueError, IndexError, OverflowError):
            check = dict(ok=False, reason='static pass command geometry unavailable')
        self.last_check = check
        return check

    def plan(self, lanes, ego, route, objects, static, points, legal, previous,
             now, time_scale=1., ready=True, *, budget_s=None, side_margin_m=.25, contact=None,
             legal_checked=False, reserve_source_bounds=True, slalom_first=True):
        """Try a complete legal slalom before the timed left-side exception."""
        from gui import (RosPathPublisher, MAX_CONTROL_CURVATURE, BYPASS_ALAT_MPS2,
                         path_curvatures, static_clear_distance)
        from lattice_planner import plan as lattice_plan
        from tunnel_runtime import swept_clearance
        deadline = time.perf_counter()+budget_s if budget_s is not None else None
        if not normal_static_area(lanes, ego):
            return None, dict(reason='timed static pass excluded near tunnel or highway')
        self.observe(ego, now)
        profile_mode = bool(previous and previous.get('static_motion_profile')
                            and self.legal is not None and not self.finished)
        with self._lock:
            if self.finished:
                self.retire_completed_geometry(now)
                self.legal, self.origin, self.finished = None, None, False
                self.physical, self.physical_origin = None, None
                self.return_station_m = None
                self._unused_inside_stamps.clear()
            self.time_scale = time_scale
        reference = self.corridor(ego, legal)
        if reference is None or points is None or not ready:
            return None, dict(reason='static pass geometry or sensor unavailable')
        points = np.asarray(points, float).reshape(-1, 2)
        if not np.isfinite(points).all():
            return None, dict(reason='static pass raw returns invalid')
        # Publish and check the complete return within a bounded forward path.
        route = np.asarray(route, float)[:, :2]
        route_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        search_horizon = 60.
        if (self.entered_at is not None and ego.get('measured') is True
                and 0. <= float(ego.get('age_s', math.inf)) <= .2):
            current_body = body_stations(np.array([[0., 0.], [2., 0.], [4., 0.]]), reference)
            if current_body[1][0] > .03 and current_body[2][0] <= .03:
                # Complete the current observed return before solving another
                # obstacle far beyond it. Keep at least the stopping envelope
                # plus a fully observed three-metre in-lane suffix. The entire
                # shorter path still receives all body/raw/map/motion checks.
                speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
                search_horizon = min(search_horizon, max(12., speed*.7+speed*speed/5.+4.))
        route = route[route_arc <= search_horizon]
        # A compact fusion box can lose its raw association beside the car.
        # Reserve its original published bounds before entering the pass, so
        # restoring source padding cannot invalidate the path mid-manoeuvre.
        # Multi-row slaloms use their complete measured sweep instead of
        # imposing a second, larger envelope on an already narrow S path.
        from model_avoidance import _road_rejoin_preference_objects
        physical_static = static
        source_bounds = [obj for obj in _road_rejoin_preference_objects(
            static, ego, route, include_near=True) if obj[3] == 'fusion_obstacle'] if reserve_source_bounds else []
        reserved_static = list(static) + source_bounds
        expanded = opposing_corridor(lanes, ego, reference.route, reference)
        # Fresh legal paths can also solve a stale committed-side dead end.
        bounds = [reference] + ([expanded] if expanded is not None else [])
        if legal_checked and expanded is not None:
            # The host already searched the ordinary lane in this frame.
            # Spend its bounded fallback budget on the additional option.
            bounds = [expanded]
        if self.entered_at is not None:
            bounds = [expanded] if expanded is not None else []
        previous_lattice = None
        if previous is not None and previous.get('bounded_static_pass'):
            from highway_planner import world_points
            previous_lattice = dict(path=previous['path'], target=previous['direction']*previous['offset_m'],
                world_path=world_points(previous['path'], ego), stamp=now, blocked=False)
            if previous.get('static_borrow') and expanded is not None:
                bounds = [expanded]
        canonical_cache, pace_cache = {}, {}
        def canonical(path):
            path = np.asarray(path, np.float32)[:, :2]
            if (previous is not None and previous.get('fixed_world_curve')
                    and np.array_equal(path, np.asarray(previous['path'], np.float32)[:, :2])):
                return np.asarray(previous['path'], float)[:, :2]
            if search_horizon < 60.:
                arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
                path = path[arc <= search_horizon]
            key = path.tobytes()
            if key in canonical_cache:
                return canonical_cache[key]
            ahead = np.flatnonzero(path[:, 0] > .05)
            if not len(ahead):
                return np.zeros((2, 2))
            # A surveyed route commonly starts one vertex behind the axle.
            # Joining origin -> that vertex -> forward creates a steering
            # reversal. Trim the rear prefix before adding the measured axle.
            path = np.vstack((np.zeros((1, 2)), path[ahead[0]:]))
            canonical = RosPathPublisher.control_geometry(path, route,
                dict(mode='LATTICE', validated_static=True))
            joined = measured_initial_join(canonical, ego,
                max_curvature=MAX_CONTROL_CURVATURE, lateral_accel=BYPASS_ALAT_MPS2,
                target_mps=FAST_PASS_MPS)
            canonical_cache[key] = joined
            approach_candidates.append(joined)
            return joined
        def pace(path):
            key = np.asarray(path, float).tobytes()
            if key in pace_cache:
                return pace_cache[key]
            if profile_mode:
                return static_speed_profile(path, ego, ceiling_mps=PASS_MPS)['cap_mps']
            curvature = max(path_curvatures(path), default=0.)
            _, gradient = _path_steering_profile(path, ego)
            value = min(FAST_PASS_MPS, math.sqrt(BYPASS_ALAT_MPS2/max(curvature, 1e-3)),
                math.radians(30.)*.8/max(float(np.max(gradient, initial=0.)), 1e-6))
            pace_cache[key] = value
            return value
        last_rejection = {}
        approach_candidates = []
        def check_candidate(path, cap, *, motion_profile=profile_mode):
            proof = self.assess(path, ego, cap, reference, now, objects, ready,
                                motion_profile=motion_profile)
            carried = bool(previous is not None and previous.get('static_motion_profile')
                and np.array_equal(np.asarray(path), np.asarray(previous['path'])))
            if (proof['ok'] and motion_profile and not carried and self.entered_at is None
                    and proof.get('contact_s', 0.) > PROFILE_RETURN_BUDGET_S-PROFILE_ADMISSION_RESERVE_S):
                # A new plan must survive one feedback/compute interval before
                # its next measured-pose check. The publication's hard limit
                # remains unchanged; an already issued curve keeps that limit.
                proof = dict(proof, ok=False, launch_ready=False,
                    reason='new static profile lacks feedback time reserve')
            return proof
        curvature_shapes = bool(ego.get('measured') is True
            and 0. <= float(ego.get('age_s', math.inf)) <= .2
            and math.isfinite(_known_wheel_angle(ego))
            and 0. <= float(ego.get('speed_kmh', math.inf)) <= 30.)
        stopped_shapes = bool(curvature_shapes and float(ego['speed_kmh']) <= 5.)
        for bound_index, bound in enumerate(bounds):
            if deadline is not None and time.perf_counter() >= deadline:
                break
            bound_deadline = (min(deadline, time.perf_counter()+max(0., deadline-time.perf_counter())/(len(bounds)-bound_index))
                              if deadline is not None else None)
            # Lattice intersection flags are indexed by its current route.
            # Preserve the frozen lane's world bounds while resampling them
            # onto that route; never pass old-frame profile indices through.
            _, at = bound.tree.query(route)
            delta = np.sum((route-bound.route[at])*bound.normal[at], axis=1)
            bound = LaneCorridor(route, bound.left[at]-delta, bound.right[at]-delta,
                zones=bound.zones, intersection_mask=bound.intersection_mask[at],
                body_shift_m=REAR_AXLE_SHIFT_M)
            static = reserved_static
            ranked_targets = _rank_static_targets(route, ego, static, bound,
                (0., .75, -.75, 1.05, -1.05, .45, -.45, 1.35, -1.35, -1.8), side_margin_m)
            targets = tuple(target for target, _ in ranked_targets)
            # A narrowing lane can require returning between two same-side
            # rows. If no held offset fits, try complete row profiles before
            # spending the frame on known-infeasible constant offsets.
            profiles_first = slalom_first or not any(
                not rank[0] and not rank[1] for _, rank in ranked_targets)
            used_source_bounds = source_bounds
            if (profiles_first and not slalom_first and source_bounds
                    and float(ego.get('speed_kmh', 0.)) <= .5
                    and self.entered_at is None):
                # A standstill retry may use the host's measured physical
                # boxes, as the alternating-row policy does. Rolling passes
                # reserve the full published source envelope against later
                # fusion jitter. Raw/SAT checks retain the 25 cm margin and
                # final fresh-source checks still own every publication.
                static = list(physical_static)
                used_source_bounds = []
                ranked_targets = _rank_static_targets(route, ego, static, bound,
                    targets, side_margin_m)
                targets = tuple(target for target, _ in ranked_targets)
            extra_cache = {}
            def calculate_extra(path, horizon):
                if contact:
                    from ped_guard import sweep_ok
                    if contact['block_forward'] or not sweep_ok(contact, path, max(1.5, contact['ahead_m']+.5)):
                        return 0.
                check = check_candidate(path, pace(path))
                if not check['ok']:
                    return 0.
                return min(static_clear_distance(path, ego, static, horizon,
                               body_shift_m=REAR_AXLE_SHIFT_M),
                           swept_clearance(path, points, horizon, side_margin_m,
                               lateral_margin=side_margin_m, body_shift_m=REAR_AXLE_SHIFT_M),
                           bound.clear_distance(path, horizon))
            def extra(path, horizon):
                # Sources, pose, lane and return clock are fixed in this call.
                # Rechecking an identical candidate must not consume the
                # remaining budget before the next complete path is searched.
                key = (np.asarray(path, float).tobytes(), float(horizon))
                if key not in extra_cache:
                    extra_cache[key] = calculate_extra(path, horizon)
                return extra_cache[key]
            def proposals():
                if previous_lattice is not None:
                    yield dict(path=np.asarray(previous['path']) if profile_mode or previous.get('fixed_world_curve') else canonical(previous['path']), blocked=False,
                        speed_cap_mps=FAST_PASS_MPS,
                        hold_until=previous['hold_until_m'], reused=True)
                yield lattice_plan(route, ego, points, static, bound, previous_lattice,
                    now=now, cruise_mps=FAST_PASS_MPS, max_curvature=MAX_CONTROL_CURVATURE,
                    outside_max_m=0., road_corridor=bound, paved_road_only=True,
                    extra_clearance=extra, path_transform=canonical,
                    extra_speed_cap=lambda path, horizon: static_speed_profile(path, ego)['cap_mps'],
                    candidate_quality=lambda path, horizon: 0.,
                    candidate_targets=targets,
                    candidate_lengths=((4., 5., 6., 8., 10., 14., 18., 22.) if stopped_shapes
                                       else (10., 14., 18., 22.)), side_margin_m=side_margin_m,
                    body_shift_m=REAR_AXLE_SHIFT_M, reuse_first=True, deadline=bound_deadline,
                    slalom_first=profiles_first, geometry_speed_mps=FAST_PASS_MPS,
                    required_horizon_m=60.)
            for result in proposals():
                path = np.asarray(result['path'], float)
                arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
                horizon = min(60., float(arc[-1]))
                if horizon < 8.:
                    continue  # the final publisher requires a full eight-metre path
                approach_candidates.append(path)
                clear = extra(path, horizon)
                cap = min(pace(path), result['speed_cap_mps'])
                # Retain the host's 30 deg/s actuator limit, including signed turns.
                arc, gradient = _path_steering_profile(path, ego)
                rate = np.max(gradient, initial=0.)
                if not profile_mode:
                    cap = min(cap, math.radians(30.)*.8/max(rate, 1e-6))
                proof = check_candidate(path, cap)
                speed_profile = None
                if proof['ok'] and not proof.get('contact_s') and self.entered_at is None:
                    # The entire in-lane return remains body/raw/map checked.
                    # A later bend owns its future pace rather than imposing
                    # that pace on the straight approach. Borrowed-lane clocks
                    # retain their original constant-speed proof.
                    speed_profile = static_speed_profile(path, ego)
                    cap = min(speed_profile['cap_mps'], result['speed_cap_mps'])
                    proof = check_candidate(path, cap)
                if proof.get('contact_s', 0.):
                    cap = min(cap, PASS_MPS)
                    proof = check_candidate(path, cap)
                v = max(0., float(ego.get('speed_kmh', 0.)))/3.6
                required = max(4., v*.7+v*v/5.+2.)
                from model_avoidance import fast_static_clearance
                wide_clear = fast_static_clearance(dict(path=path, clear_distance_m=clear,
                    safe_horizon_m=required, hold_until_m=horizon), ego, physical_static, points)
                if speed_profile is not None:
                    # The full measured SAT envelope already passed at the
                    # ordinary margin. Fresh raw returns reserve the wider
                    # tracking room for this dynamically certified in-lane
                    # path; coarse semantic size priors must not add that
                    # tracking allowance a second time.
                    speed_profile['raw_clear_m'] = fast_static_clearance(
                        dict(path=path, clear_distance_m=clear,
                             safe_horizon_m=required, hold_until_m=horizon), ego, [], points)
                if wide_clear < horizon-.25:
                    # Crossing permission does not remove the tracking margin
                    # learned from the 30 cm contact-guard stop. Tight paths
                    # keep a lower pace and must still satisfy the return time.
                    if speed_profile is None:
                        cap = min(cap, TIGHT_PASS_MPS)
                    else:
                        room = max(0., speed_profile['raw_clear_m']-max(v, FAST_PASS_MPS)*.7)
                        cap = min(cap, math.sqrt(TIGHT_PASS_MPS**2+3.*room))
                    proof = check_candidate(path, cap)
                reachable = np.sqrt(np.maximum(0., v*v-5.*np.maximum(0., arc[1:]-v*.2)))
                curve_reachable = np.maximum(0., v*v-3.*np.maximum(0., arc[1:-1]-v*.7))
                curve_safe = np.all(curve_reachable*np.asarray(path_curvatures(path)) <= BYPASS_ALAT_MPS2+.05)
                if (result['blocked'] or clear < max(horizon-.25, required) or cap < .5 or not proof['ok']
                        or not curve_safe or np.any(reachable*gradient > math.radians(30.)+1e-6)):
                    last_rejection = dict(clear_m=clear, horizon_m=horizon, cap_mps=cap,
                        blocked=bool(result['blocked']), timing=proof, candidates=result.get('rejected'))
                    continue
                _, at = reference.tree.query(path)
                offsets = np.sum((path-reference.route[at])*reference.normal[at], axis=1)
                peak = int(np.argmax(np.abs(offsets)))
                direction = 1. if offsets[peak] >= 0. else -1.
                with self._lock:
                    if proof.get('contact_s', 0.) and self.legal is None:
                        self.legal, self.origin = reference, dict(ego)
                        self.last_inside_at = now-max(0., float(ego.get('age_s', 0.)))
                        self.last_stamp = ego.get('stamp')
                    if proof.get('contact_s', 0.) and self.entered_at is None:
                        # A pending proposal can be replaced by a fully checked
                        # shorter/different excursion before physical contact.
                        # Freeze the latest admitted endpoint when observe()
                        # detects entry; earlier speculative geometry must not
                        # strand the car at an obsolete return point.
                        self.return_station_m = proof['return_station_m']
                    if proof.get('contact_s', 0.):
                        self.physical, self.physical_origin = bound, dict(ego)
                    self.last_check = proof
                return dict(path=path, kind='bypass', direction=direction,
                    offset_m=abs(float(offsets[peak])), hold_m=6., transition_m=14.,
                    curvature=float(max(path_curvatures(path))), speed_cap_mps=cap,
                    ordinary_speed_cap_mps=PASS_MPS,
                    safe_horizon_m=required, clear_distance_m=clear,
                    hold_until_m=min(horizon, max(required, float(result.get('hold_until') or 0.))),
                    activation_m=35., replan_needed=False, reused=bool(result.get('reused')),
                    bounded_static_pass=True, static_borrow=bool(proof.get('contact_s')),
                    boundary_contact_s=proof.get('contact_s', 0.),
                    static_speed_profile=speed_profile,
                    static_launch=bool(proof.get('launch_ready')),
                    static_motion_profile=profile_mode,
                    static_profile_ceiling_mps=proof.get('profile_ceiling_mps'),
                    generator='road_static_pass'), dict(proof, source_envelope_n=len(used_source_bounds))
        # A close body may need the wheel change before an endpoint-offset
        # join would start turning. These measured forward proposals keep the
        # same admission checks; ordinary in-lane paths need no time exception.
        if curvature_shapes and bounds and self.entered_at is None:
            from highway_planner import local_points
            from ego_body import inside_body
            people = [obj for obj in objects if obj[4] == 'pedestrian']
            people_local = local_points(np.asarray([obj[:2] for obj in people], float).reshape(-1, 2), ego)
            nearby_person = bool(len(people_local) and np.any(
                (people_local[:, 0] > -5.) & (people_local[:, 0] < 60.)
                & (np.abs(people_local[:, 1]) < 6.)))
            physical = expanded if expanded is not None else reference
            # Standstill recovery uses the same currently associated physical
            # boxes as the stopped lattice search above. Producer padding is
            # not a second physical obstacle; unvalidated boxes keep their
            # original dimensions, and the complete fresh raw sweep remains.
            recovery_static = physical_static if stopped_shapes else reserved_static
            recovery_deadline = time.perf_counter()+.25
            if not nearby_person and not inside_body(points, margin=.03).any():
                for path in measured_curvature_paths(route, ego, physical_static):
                    if time.perf_counter() >= recovery_deadline:
                        break
                    if max(path_curvatures(path), default=0.) > MAX_CONTROL_CURVATURE:
                        continue
                    if not _reachable_join_dynamics(
                            path, ego, MAX_CONTROL_CURVATURE, BYPASS_ALAT_MPS2)[0]:
                        continue
                    horizon = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                    # Reject a colliding proposal before the more expensive
                    # full lane-contact and future-speed calculation. The
                    # bounded search retains every identical admission check.
                    clear = min(static_clear_distance(path, ego, recovery_static, horizon,
                                    body_shift_m=REAR_AXLE_SHIFT_M),
                        static_clear_distance(path, ego,
                            [obj for obj in objects if obj[4] in ('pedestrian', 'npc')],
                            horizon, body_shift_m=REAR_AXLE_SHIFT_M),
                        swept_clearance(path, points, horizon, side_margin_m,
                            lateral_margin=side_margin_m, body_shift_m=REAR_AXLE_SHIFT_M))
                    if clear < horizon-.25:
                        continue
                    clear = min(clear, physical.clear_distance(path, horizon))
                    if clear < horizon-.25:
                        continue
                    speed_profile = static_speed_profile(path, ego, ceiling_mps=PASS_MPS)
                    cap = speed_profile['cap_mps']
                    proof = check_candidate(path, cap, motion_profile=True)
                    if not proof['ok'] or cap < .1:
                        continue
                    if contact:
                        from ped_guard import sweep_ok
                        if contact['block_forward'] or not sweep_ok(
                                contact, path, max(1.5, contact['ahead_m']+.5)):
                            continue
                    borrowed = bool(proof.get('contact_s'))
                    _, at = reference.tree.query(path)
                    offsets = np.sum((path-reference.route[at])*reference.normal[at], axis=1)
                    peak = int(np.argmax(np.abs(offsets)))
                    if borrowed:
                        with self._lock:
                            self.legal, self.origin = reference, dict(ego)
                            self.last_inside_at = now-max(0., float(ego.get('age_s', 0.)))
                            self.last_stamp = ego.get('stamp')
                            self.return_station_m = proof['return_station_m']
                            self.physical, self.physical_origin = physical, dict(ego)
                            self.last_check = proof
                    return dict(path=path, kind='bypass', direction=1. if offsets[peak] >= 0. else -1.,
                        offset_m=abs(float(offsets[peak])), hold_m=0., transition_m=0.,
                        curvature=float(max(path_curvatures(path))), speed_cap_mps=cap,
                        ordinary_speed_cap_mps=PASS_MPS, safe_horizon_m=4., clear_distance_m=clear,
                        hold_until_m=horizon, activation_m=35., replan_needed=True,
                        bounded_static_pass=True, static_borrow=borrowed,
                        boundary_contact_s=proof.get('contact_s', 0.),
                        static_speed_profile=None if borrowed else speed_profile,
                        static_launch=borrowed, static_motion_profile=borrowed,
                        static_profile_ceiling_mps=proof.get('profile_ceiling_mps'),
                        generator='road_static_launch' if borrowed else 'road_static_pass'), proof
        # A future timed excursion may be inadmissible at standstill even
        # though a complete, stopping-distance-bounded in-lane approach is
        # clear. Move only on that independently checked prefix; never use
        # acceleration assumptions to authorize contact with the centre line.
        if (bounds and self.entered_at is None and ego.get('measured') is True
                and 0. <= float(ego.get('age_s', math.inf)) <= .2):
            current_body = body_stations(np.array([[0., 0.], [2., 0.], [4., 0.]]), reference)
            if current_body[1][0] <= .03 and current_body[2][0] <= .03:
                if stopped_shapes:
                    physical = expanded if expanded is not None else reference
                    for candidate in approach_candidates:
                        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(candidate, axis=0), axis=1))]
                        for limit in (18., 24.):
                            path = candidate[arc <= limit]
                            if len(path) < 16:
                                continue
                            horizon = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                            cap = static_speed_profile(path, ego, ceiling_mps=PASS_MPS)['cap_mps']
                            proof = check_candidate(path, cap, motion_profile=True)
                            if not proof['ok'] or not proof.get('contact_s') or cap < .1:
                                continue
                            if contact:
                                from ped_guard import sweep_ok
                                if contact['block_forward'] or not sweep_ok(
                                        contact, path, max(1.5, contact['ahead_m']+.5)):
                                    continue
                            clear = min(physical.clear_distance(path, horizon),
                                static_clear_distance(path, ego, reserved_static, horizon,
                                                      body_shift_m=REAR_AXLE_SHIFT_M),
                                swept_clearance(path, points, horizon, side_margin_m,
                                    lateral_margin=side_margin_m, body_shift_m=REAR_AXLE_SHIFT_M))
                            if clear < horizon-.25 or not _reachable_join_dynamics(
                                    path, ego, MAX_CONTROL_CURVATURE, BYPASS_ALAT_MPS2)[0]:
                                continue
                            _, at = reference.tree.query(path)
                            offsets = np.sum((path-reference.route[at])*reference.normal[at], axis=1)
                            peak = int(np.argmax(np.abs(offsets)))
                            with self._lock:
                                if self.legal is None:
                                    self.legal, self.origin = reference, dict(ego)
                                    self.last_inside_at = now-max(0., float(ego.get('age_s', 0.)))
                                    self.last_stamp = ego.get('stamp')
                                self.return_station_m = proof['return_station_m']
                                self.physical, self.physical_origin = physical, dict(ego)
                                self.last_check = proof
                            return dict(path=path, kind='bypass', direction=1. if offsets[peak] >= 0. else -1.,
                                offset_m=abs(float(offsets[peak])), hold_m=0., transition_m=0.,
                                curvature=float(max(path_curvatures(path))), speed_cap_mps=cap,
                                ordinary_speed_cap_mps=PASS_MPS, safe_horizon_m=4., clear_distance_m=clear,
                                hold_until_m=horizon, activation_m=35., replan_needed=True,
                                bounded_static_pass=True, static_borrow=True,
                                boundary_contact_s=proof['contact_s'], static_launch=True,
                                static_motion_profile=True,
                                static_profile_ceiling_mps=proof.get('profile_ceiling_mps'),
                                generator='road_static_launch'), proof
                candidates = approach_candidates+(
                    [canonical(previous['path'])] if previous is not None else [])+[canonical(route)]
                for candidate in candidates:
                    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(candidate, axis=0), axis=1))]
                    for limit in (18., 12., 9., 8.5):
                        path = candidate[arc <= limit]
                        if len(path) < 16:
                            continue
                        horizon = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                        v = max(0., float(ego.get('speed_kmh', 0.)))/3.6
                        if horizon < max(8., v*.7+v*v/3.+2.):
                            continue
                        _, left, right = body_stations(path, reference)
                        if np.any(left > .03) or np.any(right > .03):
                            continue
                        if contact:
                            from ped_guard import sweep_ok
                            if contact['block_forward'] or not sweep_ok(
                                    contact, path, max(1.5, contact['ahead_m']+.5)):
                                continue
                        clear = min(reference.clear_distance(path, horizon),
                            static_clear_distance(path, ego, reserved_static, horizon,
                                                  body_shift_m=REAR_AXLE_SHIFT_M),
                            static_clear_distance(path, ego,
                                [obj for obj in objects if obj[4] in ('pedestrian', 'npc')],
                                horizon, body_shift_m=REAR_AXLE_SHIFT_M),
                            swept_clearance(path, points, horizon, side_margin_m,
                                lateral_margin=side_margin_m, body_shift_m=REAR_AXLE_SHIFT_M))
                        if clear < horizon-.25 or not _reachable_join_dynamics(
                                path, ego, MAX_CONTROL_CURVATURE, BYPASS_ALAT_MPS2)[0]:
                            continue
                        stopping_cap = max(0., math.sqrt(1.05**2+3.*max(0., horizon-2.))-1.05)
                        cap = min(PASS_MPS, stopping_cap,
                                  static_speed_profile(path, ego)['cap_mps'])
                        if cap < .5:
                            continue
                        return dict(path=path, kind='bypass', direction=0., offset_m=0.,
                            hold_m=0., transition_m=0., curvature=float(max(path_curvatures(path))),
                            speed_cap_mps=cap, ordinary_speed_cap_mps=PASS_MPS,
                            safe_horizon_m=max(4., v*.7+v*v/5.+2.), clear_distance_m=clear,
                            hold_until_m=horizon, activation_m=35., replan_needed=True,
                            bounded_static_pass=True, static_borrow=False, boundary_contact_s=0.,
                            approach_only=True, generator='road_static_approach'), dict(
                                ok=True, contact_s=0., encroachment_m=0., approach_only=True,
                                reason='checked in-lane approach before a timed static pass')
        def corridor_record(value):
            if value is None:
                return None
            return dict(route=value.route.tolist(), left=value.left.tolist(), right=value.right.tolist(),
                        zones=value.zones, intersection_mask=value.intersection_mask.tolist())
        # Preserve the actual failing search inputs. Raw bags alone cannot
        # reconstruct the frozen return clock, remembered boxes or prior plan.
        replay = dict(ego=dict(ego), route=route.tolist(), objects=list(objects),
            static=list(physical_static), points=points.tolist(), legal=corridor_record(legal),
            previous=previous, now=now, time_scale=time_scale, ready=ready,
            budget_s=budget_s, side_margin_m=side_margin_m, contact=contact,
            legal_checked=legal_checked, reserve_source_bounds=reserve_source_bounds,
            slalom_first=slalom_first, lateral_accel_mps2=BYPASS_ALAT_MPS2,
            policy=dict(legal=corridor_record(self.legal), origin=self.origin,
                entered_at=self.entered_at, inside_frames=self.inside_frames,
                last_stamp=self.last_stamp, last_inside_at=self.last_inside_at,
                return_station_m=self.return_station_m, finished=self.finished,
                physical=corridor_record(self.physical), physical_origin=self.physical_origin))
        return None, dict(reason='no complete static pass within lane/time/traffic bounds',
                          replay=replay, **last_rejection)
