"""Highway-only lane selection and common exit connection policy."""
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.spatial import cKDTree

from highway_planner import (TRAFFIC_CONTROL_HORIZON_S, geometry_speed_cap, geometry_speed_profile,
                             lane_change_path, local_points,
                             path_curvature, target_lane_rear_conflict, timed_gap,
                             world_points, lead_vehicle_speed_cap, STATIC_STANDOFF_M,
                             LANE_ADMISSION_MARGIN_M, CURRENT_PATH_FOOTPRINT_MARGIN_M)
from highway_planner import _GapForecastContext, _PREPARED_TIMED_GAP
from lane_corridor import LaneCorridor, points_in_polygon
from highway_speed_plan import clearance_forecast_bound
from .lane_map import HighwayLane, HighwayLaneMap, arc, smooth
from .object_policy import highway_static_objects, EGO_HALF_WIDTH_M
from ego_body import EGO_CENTRE_M, EGO_HALF_LENGTH_M
from .mobil import MobilEvaluator
from .path_continuity import join_reference, projected_suffix


def native_motion(obj, geometry):
    """Match measured motion to the same surveyed road used by forecasting."""
    if obj[4] != 'npc' or len(obj)<11 or len(obj)>14 and not obj[14]:
        return None
    velocity = np.asarray(obj[9:11], dtype=float)
    if np.linalg.norm(velocity) < .3:
        return None
    best = None
    for lane, path in geometry:
        squared = np.sum((path-np.asarray(obj[:2]))**2, axis=1)
        at = int(np.argmin(squared))
        if squared[at] > (lane.half_width_m+.3)**2:
            continue
        tangent = path[min(at+1, len(path)-1)]-path[max(0, at-1)]
        tangent /= max(1e-6, np.linalg.norm(tangent))
        forward = float(velocity @ tangent)
        if forward < np.linalg.norm(velocity)*math.cos(math.radians(30.)):
            continue
        normal = np.array([-tangent[1], tangent[0]])
        stations = arc(path)
        start = stations[at]+float((np.asarray(obj[:2])-path[at]) @ tangent)
        value = (path, stations, start, forward,
                 float((np.asarray(obj[:2])-path[at]) @ normal), float(velocity @ normal))
        if best is None or squared[at] < best[0]:
            best = squared[at], value
    return best[1] if best is not None else None


ALIGN_ESCAPE_M = 6.   # a standstill join must be back inside the lane margin within this distance
LANE_BODY_MARGIN_M = .3
# Corridor corner support follows the corridor's explicit rear-axle body shift.
CORRIDOR_MARGIN_M = .2
# An inward join from rest swings its outer rear corner out first (run
# follow-20261010T052911Z t=280: 0.080 m over the margin at rest, 0.145 m after
# 3 m, inside after 6 m). Creep through up to this much more, inside the margin.
ALIGN_SWING_M = .1
ALIGN_CREEP_MPS = 4./3.6   # below the 5 km/h standstill branch, so the join is kept
ALIGN_JOINS_M = (12., 16., 20.)   # standstill joins; 16-20 m reach 0.75-0.8 m-off starts
# Quicker heading corrections for the current-lane keep path. A 2.7-3.7 deg
# heading after a change, matched over 34 m at 48 km/h, bulged 0.62 m out of the
# leftmost lane (same run t=270.65): emergency stop, 5 min stall.
KEEP_HEADING_SPANS_M = (20., 12., 8.)
# A quicker correction must be drivable at the 3.5 m/s² it was validated with
# (lane-edge recovery runs); the faster highway speed caps use 4.0 m/s².
KEEP_PATH_CURVE_ALAT_MPS2 = 3.5
# A started change is rechecked every frame (3 s horizon, 0.6 m footprint margin)
# and cancelled on a conflict. Its start needs that clearance plus box-pose noise:
# a 9x3.2 m truck box jittered +-0.35 m and +-4.4 deg and erased a 0.45 m start
# slack (run follow-20261009T204445Z t=220.4->223.7). 0.3 m keeps a centred
# 3.2 m box in the next 3.5 m lane passable (0.95 m body clearance).
ADMISSION_HYSTERESIS_M = .3
# The static-relief cancel acts on a hit inside the comfort braking reserve when
# the current lane is 10 m clearer; a start must not already meet it.
ADMISSION_STATIC_HYSTERESIS_M = 5.
# Native branches can run less than a metre apart before joining. Keep the
# previous match while their measured distances differ by at most 25 cm.
CURRENT_LANE_HYSTERESIS_M = .25
# Lane selection plans static escapes with the gentle 1.2 m/s² / 2 m/s³ reserve
# it was tuned with, independent of the (firmer) longitudinal comfort braking:
# an obstacle ahead still triggers the escape search as early as before.
ESCAPE_PLANNING_DECEL_MPS2 = 1.2
ESCAPE_PLANNING_JERK_MPS3 = 2.
# Brisk adjacent change behind a slower leader: 3.6 s of travel (peak lateral
# acceleration 5.77*3.5 m/3.6² s = 1.56 m/s²), at least 30 m. Every length is
# checked with the same footprint, rear-gap, static and map tests.
BRISK_CHANGE_S = 3.6
BRISK_CHANGE_MIN_M = 30.
# Free lane choice: other lanes are searched during a committed change at
# about 6.7 Hz; the configured progress hold spans successive searches.
FREE_SEARCH_PERIOD_S = .15
# The recorded 486-query rear-gap frame spends most time trying many paces.
# Scalar checks keep early exits cheap; larger batches amortize NumPy's fixed
# step cost without dropping any goal from the original 0.5 m/s search grid.
YIELD_PREVIEW_SCALAR_PREFIX = 4
YIELD_PREVIEW_BATCH_SIZE = 64


def _paced_previews(model, path, ego, objects, paces, horizon, **kwargs):
    """Preserve pace order and stop requests while sharing each forecast step."""
    for pace in paces[:YIELD_PREVIEW_SCALAR_PREFIX]:
        yield pace, model.preview(path, ego, objects, float(pace), horizon, **kwargs)
    for start in range(YIELD_PREVIEW_SCALAR_PREFIX, len(paces), YIELD_PREVIEW_BATCH_SIZE):
        goals = paces[start:start+YIELD_PREVIEW_BATCH_SIZE]
        yield from zip(goals, model.preview_many(path, ego, objects, goals, horizon, **kwargs))


def _cached_gap_checker(path, ego, object_positions, object_yaws, body_shift_m=EGO_CENTRE_M,
                        gap_function=None):
    """Reuse identical forecasts within one immutable candidate evaluation.

    Path, ego pose, traffic contents, native motion and footprint headings stay
    fixed for this closure's lifetime. With a supplied trajectory, timed_gap
    uses its times/distances and ignores the requested goal and acceleration.
    Keep object identity and every changing check option in the key. Retain the
    objects to prevent id reuse; copy risks before caller annotations.
    """
    cache = {}
    gap_function = timed_gap if gap_function is None else gap_function
    prepared_options = (dict(_prepared=_GapForecastContext(object_positions, object_yaws))
                        if gap_function is _PREPARED_TIMED_GAP else {})
    stats = dict(queries=0, computations=0, hits=0)

    def check(objects, goal, trajectory, lane_change=True, ignore_rear_approach=False,
              footprint_margin_m=LANE_ADMISSION_MARGIN_M):
        stats['queries'] += 1
        key = (id(objects), lane_change, ignore_rear_approach, footprint_margin_m,
               trajectory.times.dtype.str, trajectory.times.shape, trajectory.times.tobytes(),
               trajectory.distances.dtype.str, trajectory.distances.shape,
               trajectory.distances.tobytes())
        previous = cache.get(key)
        if previous is not None and previous[0] is objects:
            stats['hits'] += 1
            risk = previous[1]
        else:
            stats['computations'] += 1
            risk = gap_function(path, ego, objects, goal, lane_change=lane_change,
                speed_trajectory=trajectory, object_positions=object_positions,
                object_yaws=object_yaws, ignore_rear_approach=ignore_rear_approach,
                footprint_margin_m=footprint_margin_m, body_shift_m=body_shift_m,
                **prepared_options)
            cache[key] = objects, dict(risk) if risk is not None else None
        return dict(risk) if risk is not None else None

    check.stats = stats
    return check


def lane_static_body(lane, ego, objects, reserve_m, bodies=None):
    """Nearest standing/unknown body in a lane's own corridor, ahead within reserve_m.

    A gradual (90-200 m) change reaches the destination lane only far ahead, so
    the path's static clearance missed a car of unknown speed 44 m ahead in it
    (run follow-20261009T185440Z: changed lanes at 82 km/h, then braked to
    29 km/h). Check the destination lane itself before admitting the change.
    ``bodies`` replaces the standing/unknown selection. Returns (distance_m, object) or None.
    """
    centre = np.asarray(lane.centre, dtype=float)[:, :2]
    if len(centre) < 2 or not math.isfinite(reserve_m) or reserve_m <= 0.:
        return None
    stations = arc(centre)
    tangents = np.gradient(centre, axis=0)
    tangents /= np.maximum(1e-9, np.linalg.norm(tangents, axis=1, keepdims=True))
    tree = cKDTree(centre)
    yaw0 = math.radians(float(ego['heading_deg']))
    nearest = None
    for obj in highway_static_objects(objects) if bodies is None else bodies:
        try:
            values = np.asarray([obj[0], obj[1], obj[6], obj[7], obj[8]], dtype=float)
        except (TypeError, ValueError, IndexError):
            continue
        if not np.isfinite(values).all():
            continue
        point = local_points(values[None, :2], ego)[0]
        _, k = tree.query(point)
        offset = point-centre[k]
        along = float(stations[k]+offset @ tangents[k])
        if not 0. < along <= reserve_m:
            continue
        lateral = abs(float(offset @ np.array([-tangents[k, 1], tangents[k, 0]])))
        relative = math.radians(values[4])-yaw0-math.atan2(tangents[k, 1], tangents[k, 0])
        half_across = (values[2]/2.*abs(math.sin(relative))+values[3]/2.*abs(math.cos(relative)))
        if lateral <= EGO_HALF_WIDTH_M+half_across+LANE_BODY_MARGIN_M:
            if nearest is None or along < nearest[0]:
                nearest = (along, obj)
    return nearest


def coasting_motion(obj):
    """A moving LiDAR car whose motion is retained from an earlier scan (age > 0).

    Its forecast body grows by 6*age^2 until it is observed again, so a change
    admitted around it is cancelled once it grows: a 0.18 s-old slow truck read
    as leaving the target lane started an escape that the next frame cancelled
    (run follow-20261009T204445Z t=494.27->494.45, uid 500057).
    """
    try:
        return (obj[3] == 'lidar_car' and len(obj) > 15 and bool(obj[14]) and
                float(obj[5]) >= .5 and float(obj[15]) > 0.)
    except (TypeError, ValueError, IndexError):
        return False


def lane_centred_copies(motion_paths, objects, path=None, ego=None, reach_m=5.5, horizon_s=14.):
    """Lane-matched cars also at their lane centre and heading (admission only).

    Vehicles keep their lanes, but box offset and yaw jitter within one: a start
    at a favourable sample (truck 0.39 m and 2.6 deg away from the target lane)
    was cancelled 3.3 s later at the opposite sample (+0.10 m, +4.4 deg). Copies
    are registered under their own key; the measured body stays in the check.
    Given the local ``path``, include copies whose lane reaches it during the
    checked horizon. A gradual change may reach a currently farther lane only
    later; one outward velocity sample must not make its slower car disappear
    from admission (follow-20261010T062150Z, 487.850 s).
    """
    if path is not None:
        path = np.asarray(path, dtype=float)[:, :2]
        tree = cKDTree(path)
    copies = []
    for obj in objects:
        key = obj[11] if len(obj) > 11 else id(obj)
        motion = motion_paths.get(key)
        if motion is None or isinstance(key, tuple):
            continue
        lane, stations, start, forward, lateral, _ = motion
        at = int(np.clip(np.searchsorted(stations, start), 1, len(lane)-1))
        tangent = lane[at]-lane[at-1]
        tangent = tangent/max(1e-6, float(np.linalg.norm(tangent)))
        heading = math.degrees(math.atan2(tangent[1], tangent[0]))
        relative = (float(obj[8])-heading+90.) % 180.-90.
        if abs(lateral) < .05 and abs(relative) < .5:
            continue
        row = list(obj)
        row[0:2] = [float(np.interp(start, stations, lane[:, i])) for i in (0, 1)]
        if path is not None:
            end = start+max(0., forward)*horizon_s
            sample = np.r_[start, stations[(stations > start) & (stations < end)], end]
            centres = np.column_stack([np.interp(sample, stations, lane[:, i]) for i in (0, 1)])
            points = local_points(centres, ego)
            distance, _ = tree.query(points)
            behind = points[:, 0] < path[0, 0]
            distance[behind] = np.abs(points[behind, 1]-path[0, 1])
            if distance.min() >= reach_m:
                continue
        row[8], row[9:11], row[11] = heading, (tangent*forward).tolist(), ('lane centre', key)
        motion_paths[row[11]] = (lane, stations, start, forward, 0., 0.)
        copies.append(tuple(row))
    return copies


def keep_path(centre, transition, speed, map_clearance, horizon=170.):
    """Current-lane join from the measured pose; quicker heading correction if needed.

    The measured heading is matched over min(transition, 35 m). When that leaves
    the mapped road, take the gentlest shorter span with the most clearance whose
    curvature is drivable at the current speed. A road that every span leaves is
    unchanged (and still stops the car)."""
    best = lane_change_path(centre, 0., transition, match_heading=True)
    clear = float(map_clearance(best, horizon))
    for span in KEEP_HEADING_SPANS_M:
        if clear >= horizon-.25:
            break
        if span >= min(transition, 35.):
            continue
        candidate = lane_change_path(centre, 0., transition, match_heading=True, heading_span_m=span)
        if geometry_speed_cap(candidate, max(speed, .1),
                              lateral_accel=KEEP_PATH_CURVE_ALAT_MPS2) < speed-.1:
            continue
        candidate_clear = float(map_clearance(candidate, horizon))
        if candidate_clear > clear+.25:
            best, clear = candidate, candidate_clear
    return best


def corridor_overrun(corridor, path, horizon_m):
    """LaneCorridor.clear_distance's worst corner overrun per 0.25 m station (+ = outside)."""
    if hasattr(corridor, "sample_overrun"):
        return corridor.sample_overrun(path, horizon_m)
    path = np.asarray(path, dtype=float)[:, :2]
    stations = arc(path)
    sample = np.arange(0., min(horizon_m, stations[-1])+.01, .25)
    centres = np.column_stack([np.interp(sample, stations, path[:, k]) for k in (0, 1)])
    tangent = np.gradient(centres, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    shift = getattr(corridor, 'body_shift_m', 0.)
    corners_spec = [(shift+a, b) for a in (-EGO_HALF_LENGTH_M, 0., EGO_HALF_LENGTH_M)
                    for b in (-EGO_HALF_WIDTH_M-CORRIDOR_MARGIN_M,
                              EGO_HALF_WIDTH_M+CORRIDOR_MARGIN_M)]
    corners = np.stack([centres+a*tangent+b*normal for a, b in corners_spec], axis=1).reshape(-1, 2)
    _, index = corridor.tree.query(corners)
    lateral = np.sum((corners-corridor.route[index])*corridor.normal[index], axis=1)
    overrun = np.maximum(lateral-corridor.left[index], corridor.right[index]-lateral)
    overrun = overrun.reshape(len(centres), -1).max(axis=1)
    for zone in corridor.zones:
        overrun[points_in_polygon(corners, zone).reshape(len(centres), -1).any(axis=1)] = 1e3
    return sample, overrun


def inward_recovery_clearance(path, centre, map_clearance):
    """Map clearance for leaving a standstill that ended over the lane-edge margin.

    With the body already inside the margin, every path fails the plain map check
    at its first metre and the low-speed alignment never starts (runs
    follow-20261009T183356Z and 20261010T052911Z stood 5 min). Accept the
    centre-alignment join at the first station within ALIGN_ESCAPE_M that has
    moved the body 0.1 m inward and from which the rest is mapped-clear, if the
    corner overrun before it (``map_clearance`` must be LaneCorridor's) grows by
    at most ALIGN_SWING_M and only inside the margin (3 cm, the corridor's own
    tolerance, when already beyond it). Obstacles and the rest of the path keep
    the full checks; any other path gets the plain map clearance."""
    corridor = getattr(map_clearance, '__self__', None)
    from .trajectory_geometry import normalize_path
    def canonical(candidate):
        if hasattr(corridor, 'sample_overrun'):
            return normalize_path(candidate)[0].astype(np.float32).astype(float)
        # Generic corridor callers retain their centre-rooted geometry.
        return np.asarray(candidate, dtype=float)[:, :2]
    try:
        path = canonical(path)
    except (TypeError, ValueError, IndexError, OverflowError):
        return None
    centre = np.asarray(centre, dtype=float)[:, :2]
    if len(path) < 5 or len(centre) < 2 or not hasattr(corridor, 'normal'):
        return None
    stations = arc(path)
    def off_centre(point):
        return float(np.min(np.linalg.norm(centre-point, axis=1)))
    def tail_clear(candidate, horizon):
        if hasattr(corridor, 'sample_overrun'):
            return corridor.clear_distance(candidate, horizon, include_current=False)
        return map_clearance(candidate, horizon)
    sample, overrun = corridor_overrun(corridor, path, ALIGN_ESCAPE_M)
    if np.any(overrun >= 1e3) or not np.isfinite(overrun).all():
        return None
    initial = max(0., float(overrun[0]))
    limit = max(initial+.03, min(initial+ALIGN_SWING_M, CORRIDOR_MARGIN_M))
    k = None
    for i in range(1, len(path)-3):
        if stations[i] > ALIGN_ESCAPE_M or np.any(overrun[sample <= stations[i]+1e-9] > limit):
            break
        if (off_centre(path[i]) < off_centre(path[0])-.1 and
                float(tail_clear(path[i:], 170.)) >= 12.):
            k = i
            break
    if k is None:
        return None
    def clearance(candidate, horizon):
        plain = float(map_clearance(candidate, horizon))
        try:
            candidate = canonical(candidate)
        except (TypeError, ValueError, IndexError, OverflowError):
            return plain
        if (plain >= ALIGN_ESCAPE_M or candidate.shape != path.shape
                or not np.allclose(candidate, path, atol=1e-6)):
            return plain
        tail = float(tail_clear(candidate[k:], max(0., float(horizon)-stations[k])))
        return min(float(horizon), float(stations[k])+tail)
    return clearance


class ThreeLanePolicy:
    def __init__(self, config):
        # One progress margin (14 s forecast average) both starts a discretionary
        # change and cancels a committed one; it must persist for stable_s. Only a
        # near-term (<= 3 s) conflict, a static escape or a configured safe entry
        # acts at once. All three
        # lanes have the same safety checks: the common-entrance join angle is not a preference
        # (right-lane entries passed at 64-71 km/h in live runs, a left-lane entry
        # slowed to 12 km/h; the 700 m preference zone covered the whole highway).
        self.gain_kmh = float(config.get("lane_gain_kmh", 3.))
        self.stable_s = float(config.get("lane_gain_hold_s", .5))
        self.brisk_changes = bool(config.get('brisk_lane_changes', False))
        # A committed change is cancelled for progress only by a larger margin
        # than the one that starts it (hysteresis against start/cancel flips).
        self.cancel_gain_kmh = max(self.gain_kmh,
                                   float(config.get("lane_cancel_gain_kmh", self.gain_kmh)))
        self.exit_speed_mps = float(config.get("exit_speed_kmh", 60.))/3.6
        rule = config.get("rule_speed_limit") or {}
        self.rule_until_idx = int(rule["until_idx"]) if rule.get("until_idx") is not None else None
        self.rule_speed_mps = float(rule.get("kmh", 60.))/3.6
        self.rule_cap_mps = float("inf")
        # Free lane choice: a committed change is rechecked against staying in
        # the measured current lane every frame. Without another replan reason
        # its progress advantage must persist for stable_s (since this stamp).
        self.progress_since = None
        # The committed-change candidate search runs at most every
        # FREE_SEARCH_PERIOD_S (it costs as much as a cruise frame's search).
        self.free_search_stamp = None
        self.admission_hysteresis_m = float(config.get("lane_admission_hysteresis_m", ADMISSION_HYSTERESIS_M))
        self.goal = int(config.get("lane_change_goal", 2))
        self.horizon_s = 14.
        self.target_chain = ()
        self.current_chain = ()
        self.pending = None
        self.pending_since = 0.
        self.change_started = 0.
        self.change_goal = None
        self.planned_sequence = ()
        self.path_transform = None
        self.path_speed_cap = None
        self.change_length = None
        self.change_chains = []
        self.entry_commit_identity = None
        self.entry_commit_stamp = None
        self.minimum_goal_mps = 70. / 3.6
        self.selection_model = config.get('lane_selection_model', 'forecast_progress')
        self.overtake_left_first = bool(config.get('overtake_left_first', False))
        self.overtake_lane_targets = {
            int(source): int(destination)
            for source, destination in (config.get('overtake_lane_targets') or {}).items()
        }
        self.continuation_reference = None
        self.continuity_active = False

    def reset_entry_commit(self):
        self.entry_commit_identity = None
        self.entry_commit_stamp = None

    def cancel_entry_commit(self, planner):
        """Release an entry whose fresh saved-path safety proof is unavailable."""
        self.reset_entry_commit()
        planner.saved_world = None
        planner.phase = 'CRUISE'
        self.target_chain = ()
        self.change_chains = []
        self.change_goal = None
        self.change_length = None
        self.planned_sequence = ()
        self.progress_since = None
        self.pending = None
        self.free_search_stamp = None

    @staticmethod
    def lateral(lane):
        tangent = lane.centre[min(3, len(lane.centre)-1)]-lane.centre[0]
        tangent /= max(1e-6, np.linalg.norm(tangent))
        return float(lane.centre[0] @ np.array([-tangent[1], tangent[0]]))

    def match_current_lane(self, lanes):
        if not lanes:
            self.current_chain = ()
            return None
        current = min(lanes, key=lambda lane: abs(self.lateral(lane)))
        previous = next((lane for lane in lanes if lane.link_id in self.current_chain), None)
        if (previous is not None and
                abs(self.lateral(previous)) <= abs(self.lateral(current))+CURRENT_LANE_HYSTERESIS_M):
            current = previous
        self.current_chain = current.chain
        return current

    def preferred_overtake_lane(self, current, lanes):
        """Prefer an adjacent native lane without changing admission or commitment.

        Native map numbers differ from the candidate ranks. Explicit targets
        use those map numbers; otherwise increasing rank is the left neighbour.
        A missing explicit target leaves the ordinary checked fallback intact.
        """
        if not self.overtake_left_first or current is None:
            return None
        number = getattr(current, 'native_lane_number', None)
        if number in self.overtake_lane_targets:
            target = self.overtake_lane_targets[number]
            return next((lane for lane in lanes
                if getattr(lane, 'native_lane_number', None) == target
                and abs(lane.rank-current.rank) == 1), None)
        return next((lane for lane in lanes if lane.rank == current.rank+1), None)

    def sequence_path(self, lanes, transition, speed):
        first = lanes[0].centre
        if first[0, 0] < 0.:
            ahead = np.flatnonzero(first[:, 0] >= 0.)
            if not len(ahead):
                raise ValueError('highway sequence has no forward suffix')
            first = first[ahead[0]:]
        path = lane_change_path(first, 0., transition, match_heading=True)
        previous = first
        for leg, lane in enumerate(lanes[1:], 1):
            _, at = cKDTree(lane.centre).query(first)
            destination = lane.centre[at]
            after = leg*(transition + max(3., speed*.5))
            weight = smooth((arc(first)-after)/transition)
            path += (destination-previous)*weight[:, None]
            previous = destination
        return path

    def update_motion_paths(self, planner, lanes, ego, objects):
        """Predict surveyed merges using measured along/lateral velocity.

        Boxes without a valid speed or a nearby aligned lane keep the original
        linear forecast. Rear boxes outside these forward paths also stay linear.
        """
        planner.longitudinal.motion_paths = {}
        geometry = [(l, world_points(l.centre, ego)) for l in lanes]
        for obj in objects:
            motion = native_motion(obj, geometry)
            if motion is not None:
                planner.longitudinal.motion_paths[obj[11] if len(obj)>11 else id(obj)] = motion

    def evaluate(self, planner, path, ego, objects, map_clearance, static_clearance,
                 changing=False, destinations=(), rear=None, transition=0., check_horizon_s=None,
                 goal_limit=None, continuing_change=True, admission=None):
        # A new change (admission) must pass what its every-frame recheck will
        # demand plus hysteresis; rechecking an admitted plan uses the plain test.
        admission = changing if admission is None else bool(admission and changing)
        speed = max(0., float(ego.get("speed_kmh", 0.)))/3.6
        continuity = dict(applied=False)
        if (self.continuation_reference is not None and continuing_change and
                (changing or planner.saved_world is not None)):
            try:
                path, continuity = join_reference(path, self.continuation_reference, speed,
                    float(getattr(planner.longitudinal, 'time_scale', 1.)),
                    brisk=self.brisk_changes and changing and planner.saved_world is None)
                if changing:
                    transition = max(transition, continuity['prefix_m']+continuity['blend_m'])
            except (TypeError, ValueError, IndexError, OverflowError) as error:
                continuity['unavailable_reason'] = str(error)
                # A new commitment must not silently discard its continuity
                # reservation. The current path still gets its ordinary checks
                # and braking if the previous reference has reached its end.
                if changing:
                    return None, dict(lanes=[l.link_id for l in destinations],
                        accepted=False, reason='lane-change continuity join unavailable',
                        path_continuity=continuity)
                path = lane_change_path(path, 0., max(12., speed*2.5), match_heading=True)
                continuity['legacy_rejoin'] = True
        # Clearance samplers use 25 cm stations. Align the query endpoint so
        # an empty scan's last sample cannot look like a hit just before it.
        check_distance = math.ceil(max(170., transition+20.)*4.)/4.
        map_m = float(map_clearance(path, check_distance))
        map_m = map_m if math.isfinite(map_m) else 0.
        original_map_m, transformed_map_m = map_m, None
        transform_rejected = False
        if self.path_transform is not None:
            transformed = self.path_transform(np.array(path, copy=True))
            transformed_map_m = float(map_clearance(transformed, check_distance))
            transformed_map_m = transformed_map_m if math.isfinite(transformed_map_m) else 0.
            # A roadside nudge is another geometric candidate. It may not
            # replace a legal native path with less mapped stopping room.
            # Static, traffic and curvature checks below still use exactly
            # the selected path, including when the original is obstructed.
            transform_rejected = transformed_map_m<original_map_m-1e-6
            if not transform_rejected:
                path, map_m = transformed, transformed_map_m
        static_m = float(static_clearance(path, check_distance))
        static_m = static_m if math.isfinite(static_m) else 0.
        ceiling = planner.target_mps
        if goal_limit is not None and (changing or continuing_change and planner.saved_world is not None):
            ceiling = min(planner.overtake_mps, max(ceiling, goal_limit))
        # Before the speed-limit exemption starts (rule zone), every candidate
        # and its forecast stay below the legal limit.
        ceiling = min(ceiling, self.rule_cap_mps)
        geometry_cap = geometry_speed_cap(path, ceiling)
        side_cap = self.path_speed_cap(path) if self.path_speed_cap is not None else float("inf")
        # free_goal: the path's persistent limits. Curvature is a position-
        # dependent envelope in the forecast; the command goal (goal) keeps the
        # current-position curvature cap exactly as before.
        free_goal = min(ceiling, side_cap)
        if goal_limit is not None:
            free_goal = min(free_goal, goal_limit)
        goal = min(free_goal, geometry_cap)
        speed_profile = geometry_speed_profile(path, ceiling)
        check = dict(lanes=[l.link_id for l in destinations], accepted=False,
                     path_continuity=continuity,
                     map_clear_m=map_m, static_clear_m=static_m,
                     original_map_clear_m=original_map_m,
                     transformed_map_clear_m=transformed_map_m,
                     path_transform_rejected=transform_rejected,
                     transition_m=transition, reason="checked current lane",
                     geometry_cap_kmh=geometry_cap*3.6, free_goal_kmh=free_goal*3.6,
                     speed_ceiling_kmh=ceiling*3.6,
                     side_gap_cap_kmh=side_cap*3.6 if math.isfinite(side_cap) else None,
                     goal_kmh=goal*3.6)
        if changing:
            if any(l.merge_remaining_m < transition+10. for l in destinations):
                check['reason'] = 'continuity join cannot finish before native merge'
                return None, check
            if map_m < transition+3.:
                check["reason"] = "lane change cannot finish inside mapped road"
                return None, check
            # A slower neighbour may be beside us now and already behind when
            # the gradual lateral transition reaches its lane. The fixed rear
            # gate cannot describe that manoeuvre; retain the full timed hull
            # and rear-headway checks below, including acceleration. Keep the
            # conservative gate for fast, unknown and unmapped rear traffic.
            fixed_rear_objects = []
            heading = math.radians(float(ego['heading_deg']))
            forward = np.array([math.cos(heading), math.sin(heading)])
            for obj in objects:
                key = obj[11] if len(obj)>11 else id(obj)
                measured = obj[4] == 'npc' and len(obj)>14 and bool(obj[14])
                velocity = np.asarray(obj[9:11], dtype=float) if measured else np.zeros(2)
                along_speed = float(velocity @ forward)
                separating = (measured and key in planner.longitudinal.motion_paths
                    and along_speed >= .3 and speed-along_speed > .5
                    and along_speed >= np.linalg.norm(velocity)*math.cos(math.radians(30.)))
                if not separating:
                    fixed_rear_objects.append(obj)
            for lane in destinations:
                offset = self.lateral(lane)
                conflict = target_lane_rear_conflict(ego, fixed_rear_objects, offset, speed,
                                                     body_shift_m=planner.body_shift_m)
                if conflict:
                    check.update(reason="target-lane rear/side car too close", rear_conflict=conflict)
                    return None, check
                if rear is not None and len(rear):
                    points = np.asarray(rear)
                    if np.any((points[:, 0] < -2.5) & (points[:, 0] > -70.) &
                              (np.abs(points[:, 1]-offset) < 1.2)):
                        check["reason"] = "unclassified target-lane rear returns"
                        return None, check
                for obj in objects:
                    p = local_points(np.asarray(obj[:2])[None], ego)[0]
                    if (obj[4] == "npc" and len(obj)>14 and not obj[14] and
                            -70. < p[0] < 5. and abs(p[1]-offset) < .946+obj[7]/2.+.3):
                        check["reason"] = "target-lane rear velocity unavailable"
                        return None, check
                reserve = speed*.4+speed*speed/8.+6.
                standing = lane_static_body(lane, ego, objects, reserve)
                if standing is not None:
                    check.update(reason="target-lane standing or unknown-speed body inside stopping reserve",
                                 target_lane_body_m=round(standing[0], 1),
                                 target_lane_body=standing[1][3])
                    return None, check
                coasting = admission and lane_static_body(lane, ego, objects, reserve,
                    bodies=[o for o in objects if coasting_motion(o)])
                if coasting:
                    check.update(reason="target-lane body motion not currently observed",
                                 target_lane_body_m=round(coasting[0], 1),
                                 target_lane_body=coasting[1][3],
                                 target_lane_body_age_s=round(float(coasting[1][15]), 2))
                    return None, check
        horizon = min(18., max(self.horizon_s, transition/max(speed, 3.)+1.5))
        forecast_clear = min(clearance_forecast_bound(map_m, check_distance),
                             clearance_forecast_bound(static_m, check_distance))
        trajectory = planner.longitudinal.preview(path, ego, objects, free_goal, horizon,
            static_clear_m=forecast_clear, standoff_m=STATIC_STANDOFF_M,
            ceiling_mps=ceiling, speed_profile=speed_profile)
        gap_objects, gap_margin = objects, LANE_ADMISSION_MARGIN_M
        if admission:
            gap_objects = list(objects)+lane_centred_copies(
                planner.longitudinal.motion_paths, objects, path, ego, horizon_s=horizon)
            gap_margin += self.admission_hysteresis_m
        check_gap = _cached_gap_checker(path, ego, planner.longitudinal.object_positions,
                                       planner.longitudinal.object_yaws, planner.body_shift_m)
        command_checks = 0

        def command_risk(candidate, pace):
            nonlocal command_checks
            command_checks += 1
            # The runtime publishes a constant current-position goal, not the
            # position-dependent envelope used to rank long lane forecasts.
            command_goal = pace
            if (planner._speed_hold_goal is not None and
                    float(ego.get('stamp', 0.)) < planner._speed_hold_until):
                command_goal = min(command_goal, planner._speed_hold_goal)
            published = planner.longitudinal.preview(path, ego, objects, command_goal,
                TRAFFIC_CONTROL_HORIZON_S, static_clear_m=forecast_clear,
                standoff_m=STATIC_STANDOFF_M, ceiling_mps=ceiling)
            executed = planner.longitudinal.command_preview(published)
            command_gap = check_gap(gap_objects, pace, executed, lane_change=True,
                                    footprint_margin_m=gap_margin)
            if command_gap is not None:
                return command_gap
            if np.any(executed.distances > forecast_clear-.25):
                return dict(when=0., reason="lane-change command static/map bound exceeded")
            return None

        risk = check_gap(gap_objects, goal,
                         trajectory if check_horizon_s is None else
                         trajectory.prefix(check_horizon_s),
                         lane_change=changing or continuing_change and planner.saved_world is not None,
                         ignore_rear_approach=not changing and not continuing_change, footprint_margin_m=gap_margin)
        if changing and risk is None:
            risk = command_risk(trajectory, goal)
        if changing and risk is not None and risk.get('rear_approach'):
            # Let the faster car pass while the checked adjacent road is still
            # separate. Do not brake after entering its closing rear gap.
            for pace, paced in _paced_previews(planner.longitudinal, path, ego, objects,
                    np.arange(goal-.5, max(0., speed-6.), -.5), horizon,
                    static_clear_m=forecast_clear, standoff_m=STATIC_STANDOFF_M,
                    ceiling_mps=ceiling, speed_profile=speed_profile):
                paced_trajectory = paced if check_horizon_s is None else paced.prefix(check_horizon_s)
                paced_risk = check_gap(objects, float(pace), paced_trajectory, lane_change=True)
                if paced_risk is None:
                    # The (unchanged) search finds a pace; a start must also pass
                    # the admission test there, without a longer search.
                    if admission and check_gap(gap_objects, float(pace), paced_trajectory,
                            lane_change=True,
                            footprint_margin_m=gap_margin) is not None:
                        break
                    if command_risk(paced, float(pace)) is not None:
                        continue
                    trajectory, goal, risk = paced, float(pace), None
                    # The yield pace stays the admitted change's speed limit.
                    free_goal = goal
                    check['rear_merge_yield'] = True
                    break
        check['traffic_gap_queries'] = dict(check_gap.stats)
        check['command_check_horizon_s'] = TRAFFIC_CONTROL_HORIZON_S if command_checks else 0.
        check['command_gap_checks'] = command_checks
        check['command_footprint_margin_m'] = gap_margin if changing or planner.saved_world is not None else CURRENT_PATH_FOOTPRINT_MARGIN_M
        if risk is not None and isinstance(risk.get('object_id'), tuple):
            risk = dict(risk, object_id=risk['object_id'][1], lane_centred_pose=True)
        if changing and risk is not None:
            check.update(reason="lane-change traffic gap occupied", gap_risk=risk)
            return None, check
        checked = trajectory if check_horizon_s is None else trajectory.prefix(check_horizon_s)
        if changing and (static_m < speed*.4+speed*speed/8.+6. or
                         static_m < check_distance-.25 and np.any(checked.distances > static_m-.25)):
            check["reason"] = "lane-change static stopping reserve unavailable"
            return None, check
        distance = float(np.interp(self.horizon_s, trajectory.times, trajectory.distances))
        check.update(reason="safe lane candidate", average_kmh=distance/self.horizon_s*3.6,
                     traffic_check_horizon_s=(horizon if check_horizon_s is None else check_horizon_s),
                     control_average_kmh=float(np.interp(TRAFFIC_CONTROL_HORIZON_S,
                         trajectory.times, trajectory.distances))/TRAFFIC_CONTROL_HORIZON_S*3.6,
                     peak_decel_mps2=max(0., -float(trajectory.accelerations.min())))
        return dict(path=path, goal=goal, free_goal=free_goal, risk=risk, trajectory=trajectory,
                    map_m=map_m, static_m=static_m, query_m=check_distance,
                    ceiling_mps=ceiling, check=check), check

    @staticmethod
    def static_relief_at_start(result, lane_static_m, speed, replan_reserve_m):
        """True when the next recheck's static-relief cancel would already undo this start.

        That cancel acts on a hit inside the comfort (or firm) braking reserve
        once the current lane is 10 m clearer. The admission test only required
        the firm reserve and a stoppable trajectory.
        """
        margin = ADMISSION_STATIC_HYSTERESIS_M
        hit = clearance_forecast_bound(result['static_m'], result['query_m'])
        near = (hit < replan_reserve_m+margin or
                result['static_m'] < speed*.4+speed*speed/8.+6.+margin)
        return bool(near and lane_static_m > result['static_m']+10.-margin)

    def decide(self, planner, index, route, ego, objects, lanes, map_clearance,
               static_clearance, perception_ready, rear, motion_options=None,
               path_transform=None, path_speed_cap=None):
        self.path_transform, self.path_speed_cap = path_transform, path_speed_cap
        planner.longitudinal.begin(ego)
        self.update_motion_paths(planner, lanes if motion_options is None else motion_options, ego, objects)
        stamp = float(ego.get("stamp", time.monotonic()))
        speed = max(0., float(ego.get("speed_kmh", 0.)))/3.6
        if speed >= 15.:
            self.continuity_active = True
        elif speed <= 12.:
            self.continuity_active = False
        self.continuation_reference = (getattr(planner, 'continuation_reference', None)
                                       if self.continuity_active else None)
        remaining = max(0., float(planner.station[planner.end]-planner.station[index]))
        planner.longitudinal.end_distance_m = remaining
        planner.longitudinal.end_speed_mps = self.exit_speed_mps
        self.rule_cap_mps = (self.rule_speed_mps if self.rule_until_idx is not None and
                             index < self.rule_until_idx else float("inf"))
        # Reserve controller response and the comfort jerk ramp before the
        # lower speed limit. This margin applies only at the highway boundary.
        planner.longitudinal.end_buffer_m = speed*(.6+
            planner.longitudinal.comfort_decel_mps2/planner.longitudinal.jerk_mps3)
        if planner.phase == "OUTSIDE":
            planner.phase = "CRUISE"
        current = self.match_current_lane(lanes)
        entry = planner.config.get('entry_lane_change') or {}
        owned_entry = bool(entry.get('commit_until_complete', False) and
                           entry.get('policy') == 'first_safe_adjacent')
        entry_identity = (entry.get('source_link'), entry.get('target_link'))
        entry_goal_refreshed = False
        entry_commit_cancel_reason = None
        entry_commit_cancelled = False
        entry_failure_check = None
        if self.entry_commit_identity is not None:
            if (not owned_entry or self.entry_commit_identity != entry_identity or
                    planner.saved_world is None):
                self.reset_entry_commit()
            elif (not perception_ready or not math.isfinite(stamp) or
                    not math.isfinite(speed) or self.entry_commit_stamp is not None and
                    stamp < self.entry_commit_stamp):
                entry_commit_cancel_reason = 'entry motion sources unavailable'
                entry_commit_cancelled = True
                self.cancel_entry_commit(planner)
        entry_ready = bool(entry and current is not None and planner.saved_world is None
            and perception_ready and not entry_commit_cancelled
            and entry['from_idx'] <= index <= entry['to_idx']
            and current.link_id == entry['source_link'])
        immediate_entry = False
        preference_applied = False
        target_lane = next((l for l in lanes if l.link_id in self.target_chain), None)
        continuation_recheck = False
        if planner.saved_world is not None and target_lane is not None:
            tangent = target_lane.centre[min(3, len(target_lane.centre)-1)]-target_lane.centre[0]
            if (abs(self.lateral(target_lane)) < .35 and
                    abs(math.atan2(tangent[1], tangent[0])) < math.radians(8.) and
                    stamp-self.change_started >= .5):
                planner.completed += 1
                current = target_lane
                if self.change_chains:
                    self.change_chains.pop(0)
                if self.change_chains:
                    self.target_chain = self.change_chains[0]
                    target_lane = next((l for l in lanes if l.link_id in self.target_chain), None)
                    self.change_started = stamp
                    continuation_recheck = True
                else:
                    planner.saved_world = None
                    self.change_goal = None
                    planner.phase = "CRUISE"
                    self.reset_entry_commit()
        if planner.saved_world is not None and target_lane is None:
            # A native successor must be observed before continuing a saved lane.
            planner.saved_world = None
            self.change_goal = None
            self.target_chain = ()
            self.change_chains = []
            if self.entry_commit_identity is not None:
                entry_commit_cancel_reason = 'entry target geometry unavailable'
                entry_commit_cancelled = True
                self.cancel_entry_commit(planner)
        preferred_lane = self.preferred_overtake_lane(current, lanes)
        # A six-second adjacent change at cruise keeps steering small. Short
        # changes also serve the configured first safe adjacent entry.
        transition = max(90., max(speed, planner.target_mps)*6.)
        if current is not None and planner.saved_world is None:
            path = keep_path(current.centre, max(12., speed*2.5), speed, map_clearance)
        elif current is not None:
            path = lane_change_path(current.centre, 0., max(12., speed*2.5), match_heading=True)
        else:
            path = np.asarray(route)[:, :2]
        if planner.saved_world is not None:
            local = local_points(planner.saved_world, ego)
            nearest = int(np.argmin(np.linalg.norm(local, axis=1)))
            entry_remaining_m = max(0., (self.change_length or transition)-arc(local)[nearest])
            if len(local)-nearest >= 4:
                path = local[nearest:]
                # Every continuation rejoins the measured pose and heading.
                # Even centimetres of tracking error can otherwise introduce
                # a near-lateral origin prefix at the nearest saved vertex.
                if self.continuation_reference is None:
                    path = lane_change_path(path, 0., max(12., speed*2.5), match_heading=True)
                else:
                    # Keep the saved destination and geometry fixed. The
                    # evaluated join uses the exact recent published prefix.
                    try:
                        path = projected_suffix(local)
                    except ValueError:
                        path = lane_change_path(path, 0., max(12., speed*2.5), match_heading=True)
            else:
                if self.entry_commit_identity is not None:
                    entry_commit_cancel_reason = 'entry saved path exhausted'
                    entry_commit_cancelled = True
                    self.cancel_entry_commit(planner)
                else:
                    planner.saved_world = None
        alignment_recovery = False
        base_map_clearance = map_clearance
        if current is not None and planner.saved_world is None and speed < 5./3.6:
            # At standstill, retaining a heading error can initially steer
            # farther toward a narrow ramp edge and trap the vehicle. Check
            # the existing centre-alignment path instead, without widening
            # the surveyed road or relaxing obstacle clearance.
            if map_clearance(path, 170.) < 12.:
                # A join inside the corridor first; otherwise the bounded inward
                # escape (a longer join swings its rear corner out less).
                joins = [lane_change_path(current.centre, 0., m, match_heading=False) for m in ALIGN_JOINS_M]
                clear = next((j for j in joins if map_clearance(j, 170.) >= 12.), None)
                if clear is not None:
                    path, alignment_recovery = clear, True
                else:
                    # The checked keep path already matches the measured heading.
                    # It can move inward without the initial corner swing of a
                    # centre-only join; apply the same bounded recovery proof to
                    # it before trying those alternative joins.
                    for aligned in (path, *joins):
                        escape = inward_recovery_clearance(aligned, current.centre, map_clearance)
                        if escape is not None:
                            path, alignment_recovery, base_map_clearance = aligned, True, escape
                            break
        base = None
        if self.entry_commit_identity is not None:
            # Keep the first accepted adjacent geometry, but renew its complete
            # traffic and executable-command proof at the current legal ceiling.
            # In particular, a 58 km/h admission before the rule boundary must
            # not permanently pin the saved entry to that old goal. This is a
            # continuation proof: all fixed rear/raw gates remain active, while
            # the existing admission-only box hysteresis is not imposed again.
            base, entry_check = self.evaluate(planner, path, ego, objects,
                base_map_clearance, static_clearance, changing=True,
                destinations=[target_lane], rear=rear,
                transition=entry_remaining_m, admission=False)
            if base is None:
                entry_failure_check = entry_check
                entry_commit_cancel_reason = entry_check['reason']
                entry_commit_cancelled = True
                self.cancel_entry_commit(planner)
                path = keep_path(current.centre, max(90., speed*6.), speed, map_clearance)
            else:
                entry_goal_refreshed = (self.change_goal is None or
                    abs(base['free_goal']-self.change_goal) > 1e-6)
                self.change_goal = base['free_goal']
                self.entry_commit_stamp = stamp
        if base is None:
            base, _ = self.evaluate(planner, path, ego, objects, base_map_clearance, static_clearance,
                                    changing=False,
                                    check_horizon_s=(TRAFFIC_CONTROL_HORIZON_S
                                        if planner.saved_world is not None else None),
                                    goal_limit=self.change_goal if planner.saved_world is not None else None)
        static_hit_m = clearance_forecast_bound(base['static_m'], base['query_m'])
        braking = min(planner.longitudinal.comfort_decel_mps2, ESCAPE_PLANNING_DECEL_MPS2)
        reaction = (.4+max(0., float(getattr(planner.longitudinal, 'static_sensor_age_s', 0.)))+
                    braking/min(planner.longitudinal.jerk_mps3, ESCAPE_PLANNING_JERK_MPS3))
        static_replan_reserve = speed*reaction+speed*speed/(2.*braking)+STATIC_STANDOFF_M
        static_escape_needed = math.isfinite(static_hit_m) and static_hit_m<static_replan_reserve
        # Extra speed is available for passing a measured slower lead. An
        # empty road must not reward otherwise unnecessary lane changes.
        passing_required = lead_vehicle_speed_cap(path, ego, objects, planner.target_mps, planner.body_shift_m) < planner.target_mps-.5
        pass_ceiling = planner.overtake_mps if passing_required else planner.target_mps
        mobil = None
        if (self.selection_model == 'mobil' and current is not None and
                len(lanes) > 1 and perception_ready and continuation_recheck):
            mobil = MobilEvaluator(ego, objects, lanes,
                lanes if motion_options is None else motion_options,
                planner.longitudinal, planner.config)
        continuation_blocked = False
        if continuation_recheck and target_lane is not None:
            continuation, _ = self.evaluate(planner, path, ego, objects,
                map_clearance, static_clearance, changing=True,
                destinations=[target_lane], rear=rear,
                transition=self.change_length or transition,
                goal_limit=self.change_goal, admission=False)
            continuation_blocked = continuation is None
        replan_requested = False
        replan_reasons = []
        if planner.saved_world is not None and perception_ready and current is not None:
            near_speed = float(np.interp(TRAFFIC_CONTROL_HORIZON_S,
                base['trajectory'].times, base['trajectory'].speeds))
            minimum = min(self.minimum_goal_mps, planner.target_mps)
            if continuation_blocked:
                replan_reasons.append('next leg admission blocked')
            if base['risk'] is not None:
                replan_reasons.append('near-term traffic gap occupied')
            if base['static_m'] < speed*.4+speed*speed/8.+6.:
                replan_reasons.append('static stopping reserve unavailable')
            if static_escape_needed:
                # A hit on a long saved transition can begin braking well
                # before the three-second reference drops below 70 km/h.
                # Try a checked geometric escape during that reserve, while a
                # completed no-hit query remains free rather than a stop line.
                replan_reasons.append('bounded static hit within comfort braking reserve')
            if remaining > transition+20. and self.entry_commit_identity is None:
                if base['free_goal'] < minimum:
                    replan_reasons.append('selected path speed limited')
                # An entry accelerating through 40–70 km/h is progressing.
                # Being below the preferred cruise speed alone must not
                # repeatedly cancel and re-root its checked gradual change.
                elif near_speed < minimum-.5 and near_speed < speed-.5:
                    replan_reasons.append('near-term progress decreasing')
            replan_requested = bool(replan_reasons)
        # Free lane choice (user 2026-10-11: every lane is usable; a blocked
        # far lane must not hold the car once another lane is faster). A
        # committed change is rechecked every frame against staying in the
        # measured lane, and each completed leg is a decision point: the next
        # leg must again be worth a start (lane_gain_kmh over staying).
        free_recheck = (planner.saved_world is not None and perception_ready and
                        current is not None and not replan_requested and
                        self.entry_commit_identity is None)
        leg_boundary = continuation_recheck and not continuation_blocked
        chosen = base
        hold, hold_check = None, None
        if entry_failure_check is not None:
            hold_check = dict(entry_failure_check, cancel_basis='entry safety')
        if planner.saved_world is None:
            self.progress_since = None
        if (replan_requested or free_recheck) and current is not None:
            # A new car can close the next lane after the first leg starts.
            # Staying in the current measured lane is another fully checked
            # candidate; it need not brake merely because the old continuation
            # is blocked. Preserve measured heading before changing the plan.
            hold_path = keep_path(current.centre, max(90., speed*6.), speed, map_clearance)
            # This candidate stays in the measured current lane. The old
            # saved change must not turn it into another lane-change check,
            # nor may a distant common-merge forecast veto a safe current
            # lane and leave the now-blocked saved change as the only plan.
            # Keep full raw/static/map clearance; match the final controller's
            # near-term moving-footprint horizon and recheck each observation.
            hold, hold_check = self.evaluate(planner, hold_path, ego, objects,
                map_clearance, static_clearance,
                check_horizon_s=TRAFFIC_CONTROL_HORIZON_S, continuing_change=False)
            hold_check['gap_risk'] = hold['risk'] if hold is not None else None
            hold_check['free_recheck'] = free_recheck
            if mobil is not None and continuation_recheck and hold is not None:
                next_lanes = [next((l for l in lanes if l.link_id in chain), None)
                              for chain in self.change_chains]
                if next_lanes and all(l is not None for l in next_lanes):
                    try:
                        assessment = mobil.assess(current, next_lanes, base, hold,
                                                  self.change_length or transition)
                    except (ValueError, TypeError, IndexError):
                        assessment = dict(available=False, safe=None,
                            incentive_mps2=None, reason='MOBIL invalid motion geometry')
                    hold_check['next_leg_mobil'] = assessment
                    # The full sequence already passed utility admission.
                    # Renew its follower bound over the actual saved suffix;
                    # unknown utility alone cannot cancel a checked commitment.
                    # The existing leg_stop still requires progress over staying.
                    if assessment['safe'] is not True:
                        continuation_blocked = True
                        leg_boundary = False
                        replan_requested = True
                        free_recheck = False
                        replan_reasons.append('next leg MOBIL admission blocked')
            if hold is not None and hold['risk'] is None:
                # Moving traffic blocks the committed change near-term: stay at once.
                unsafe_change = continuation_blocked or base['risk'] is not None
                # A static hit on the change path that the current lane clearly
                # avoids. A statically worse lane is never a fallback: it turned
                # an escape past a stopped car into a stop 27 m before it.
                static_relief = ((static_escape_needed or
                                  base['static_m'] < speed*.4+speed*speed/8.+6.) and
                                 hold['static_m'] > base['static_m']+10.)
                # Otherwise only the same progress margin that starts a change
                # may cancel it. A 0.1 km/h edge on the 3 s average flipped the
                # target 13 times in 2 s (run follow-20261009T191227Z, 434/408).
                progress = (self.entry_commit_identity is None and
                            hold['check']['average_kmh'] >=
                            base['check']['average_kmh']+self.cancel_gain_kmh)
                # At a completed leg the next one is a new commitment: it must
                # forecast the same margin over staying that starts a change.
                leg_stop = bool(leg_boundary and base['check']['average_kmh'] <
                                hold['check']['average_kmh']+self.gain_kmh)
                if (progress or leg_stop) and not (unsafe_change or static_relief):
                    # The recheck ignores closing rear traffic (a safety
                    # fallback); a progress return may not drive back in front
                    # of a car that the current lane would escape at once.
                    clear_behind = timed_gap(hold['path'], ego, objects, hold['goal'], lane_change=False,
                        speed_trajectory=hold['trajectory'].prefix(TRAFFIC_CONTROL_HORIZON_S),
                        object_positions=planner.longitudinal.object_positions,
                        object_yaws=planner.longitudinal.object_yaws) is None
                    progress, leg_stop = progress and clear_behind, leg_stop and clear_behind
                if progress and not (replan_requested or leg_stop or unsafe_change or static_relief):
                    # Periodic recheck only: one forecast frame cannot undo a
                    # checked change; the advantage must hold for stable_s.
                    if self.progress_since is None or stamp < self.progress_since:
                        self.progress_since = stamp
                    hold_check['progress_held_s'] = round(stamp-self.progress_since, 3)
                    progress = stamp-self.progress_since >= self.stable_s-1e-9
                elif not progress:
                    self.progress_since = None
                hold_check['cancel_basis'] = ('unsafe change' if unsafe_change else
                    'static relief' if static_relief else 'next leg not faster' if leg_stop
                    else 'progress' if progress else None)
                if unsafe_change or static_relief or progress or leg_stop:
                    chosen = hold
        checks, requested = [], False
        reason = ("highway checked low-speed lane alignment recovery" if alignment_recovery
                  else "highway keep fastest checked lane")
        if current is None:
            reason = "highway native lane geometry unavailable"
        elif planner.saved_world is not None and not (replan_requested or free_recheck and (
                self.free_search_stamp is None or stamp < self.free_search_stamp or
                stamp-self.free_search_stamp >= FREE_SEARCH_PERIOD_S-1e-9)):
            reason = "highway committed adjacent lane change"
        elif perception_ready and not entry_commit_cancelled:
            if planner.saved_world is not None:
                reason = "highway committed adjacent lane change"
                if free_recheck:
                    self.free_search_stamp = stamp
            candidates = []
            if (mobil is None and self.selection_model == 'mobil' and
                    current is not None and len(lanes) > 1):
                mobil = MobilEvaluator(ego, objects, lanes,
                    lanes if motion_options is None else motion_options,
                    planner.longitudinal, planner.config)
            # The static-relief cancel compares a change with this lane.
            lane_static_m = (hold if hold is not None else base)['static_m']
            for destination in lanes:
                if destination is current:
                    continue
                difference = destination.rank-current.rank
                if not difference:
                    continue
                direction = 1 if difference > 0 else -1
                sequence = [next((lane for lane in lanes
                                  if lane.rank == current.rank+direction*step), None)
                            for step in range(1, abs(difference)+1)]
                if any(lane is None for lane in sequence):
                    continue
                # A car entering at 60 km/h must be able to complete the first
                # leg before its lane converges with a slow lead. Always using
                # the 90 km/h/112 m leg delayed the next adjacent overtake.
                lengths = [transition, max(90., speed*6.)]
                if self.brisk_changes and len(sequence) == 1 and planner.saved_world is None:
                    lengths.append(max(BRISK_CHANGE_MIN_M, speed*BRISK_CHANGE_S))
                if (owned_entry and entry_ready and len(sequence) == 1 and
                        destination.link_id == entry['target_link']):
                    lengths.insert(0, max(float(entry.get('transition_min_m', 45.)),
                                          speed*float(entry.get('transition_s', 3.6))))
                flow_limited = (base['risk'] is not None or
                    static_escape_needed or
                    base['trajectory'].speeds[-1] < min(self.minimum_goal_mps,planner.target_mps)-.5)
                if flow_limited:
                    # Four seconds at cruise still needs less than one degree
                    # of steering on a standard adjacent lane. Consider it
                    # before slowing for a lead that the six-second path cannot
                    # pass; keep the longer path when progress is equivalent.
                    lengths.append(max(75.,max(speed,self.minimum_goal_mps)*4.))
                    # Behind a slower leader the 78 m path kept the car in the
                    # leader's lane for 6-7 s at 40-45 km/h (25.6 s of 59 s
                    # following time was a committed change still in progress,
                    # runs 2026-10-10). A brisk change leaves that lane sooner;
                    # it is chosen only when it forecasts more progress (the
                    # forecast applies its curvature only where it occurs).
                    lengths.append(max(BRISK_CHANGE_MIN_M, speed*BRISK_CHANGE_S))
                conflict = (base['risk'] or {}).get('collision_when')
                urgent = (base['static_m'] < speed*.4+speed*speed/8.+12. or
                          conflict is not None and conflict < 3.)
                if urgent or static_escape_needed:
                    lengths.append(max(30., speed*3.6))
                    lengths.append(max(30., speed*2.4))
                for length in dict.fromkeys(lengths):
                    required = len(sequence)*length + (len(sequence)-1)*max(3., speed*.5)
                    if min(l.merge_remaining_m for l in sequence) < required+10.:
                        checks.append(dict(lanes=[l.link_id for l in sequence], accepted=False,
                                           reason="insufficient distance before common merge"))
                        continue
                    # Re-rooting the admitted sequence continues it; anything
                    # else is a new start and must pass the admission test.
                    continuing = (planner.saved_world is not None and
                        len(sequence) == len(self.change_chains) and
                        all(l.link_id in chain for l, chain in zip(sequence, self.change_chains)))
                    candidate_path = self.sequence_path(sequence, length, speed)
                    result, check = self.evaluate(planner, candidate_path, ego, objects,
                        map_clearance, static_clearance, changing=True, destinations=sequence,
                        rear=rear, transition=required, goal_limit=pass_ceiling,
                        admission=not continuing)
                    checks.append(check)
                    check['urgent_current_lane'] = bool(urgent)
                    check['bounded_static_escape'] = static_escape_needed
                    if (result is not None and not continuing and self.static_relief_at_start(
                            result, lane_static_m, speed, static_replan_reserve)):
                        check['reason'] = "lane-change static hit nearer than current lane"
                        result = None
                    if result is not None:
                        if mobil is not None and not continuing:
                            try:
                                assessment = mobil.assess(current, sequence, result,
                                    hold if hold is not None else base, length)
                            except (ValueError, TypeError, IndexError):
                                assessment = dict(available=False, safe=None,
                                    incentive_mps2=None, reason='MOBIL invalid motion geometry')
                            check['mobil'] = assessment
                            mandatory = bool(urgent or static_escape_needed or entry_ready)
                            margin = self.cancel_gain_kmh if free_recheck else self.gain_kmh
                            progress_gain = check['average_kmh']-chosen['check']['average_kmh']
                            fallback = (assessment['safe'] is True and
                                assessment.get('forecast_progress_fallback') is True and
                                set(assessment.get('unavailable_reasons', ())) == {'estimated_leader'})
                            eligible = (progress_gain >= margin and (fallback or
                                assessment['available'] and
                                assessment['incentive_mps2'] > mobil.threshold))
                            if fallback:
                                check['mobil_admission'] = 'checked forecast progress for estimated leader'
                            if assessment['safe'] is False or not (mandatory or eligible):
                                check['reason'] = (assessment['reason'] if
                                    assessment['safe'] is False or not fallback and
                                    (not assessment['available'] or
                                     assessment['incentive_mps2'] <= mobil.threshold) else
                                    'MOBIL insufficient forecast progress')
                                continue
                        result.update(sequence=sequence, length=length)
                        candidates.append(result)
            if candidates:
                # At the surveyed dashed entry, a checked adjacent merge is
                # preferred even when the current lane has no slower leader.
                # Keep its first controller reference at measured speed or
                # above; occupied lanes retain the ordinary gap/brake policy.
                entry_candidates = [r for r in candidates if entry_ready
                    and len(r['sequence']) == 1
                    and r['sequence'][0].link_id == entry['target_link']
                    and np.min(r['trajectory'].speeds[
                        r['trajectory'].times <= .9]) >= speed-.05]
                immediate_entry = bool(entry_candidates)
                if immediate_entry and owned_entry:
                    # Finish the first safe adjacent merge before considering
                    # discretionary passes farther across the highway.
                    shortest = min(r['length'] for r in entry_candidates)
                    entry_candidates = [r for r in entry_candidates
                                        if r['length'] <= shortest+1e-6]
                # Apply the requested neighbour preference only to new,
                # discretionary starts that already passed every safety and
                # MOBIL admission check and improve on staying. Otherwise a
                # safe but unhelpful preferred lane could hide a useful pass.
                # Entry, physical escapes and committed sequences retain
                # their existing ranking and cancellation rules.
                if (preferred_lane is not None and planner.saved_world is None
                        and not entry_ready and not static_escape_needed
                        and not any(r['check'].get('urgent_current_lane') for r in candidates)):
                    preferred_candidates = [r for r in candidates
                        if len(r['sequence']) == 1
                        and r['sequence'][0] is preferred_lane
                        and r['check']['average_kmh']-chosen['check']['average_kmh'] >= self.gain_kmh]
                    if preferred_candidates:
                        candidates = preferred_candidates
                        preference_applied = True
                fast = [r for r in candidates if r['free_goal'] >= min(
                    self.minimum_goal_mps, planner.target_mps)]
                if immediate_entry:
                    candidates = entry_candidates
                elif fast:
                    candidates = fast
                # Discretionary starts rank by the measured MOBIL balance.
                # Physical escapes, entry and committed continuations retain
                # progress ranking; an unknown utility must not outrank a
                # checked mandatory escape with a negative incentive.
                utility_rank = mobil is not None and not (entry_ready or
                    static_escape_needed or any(r['check'].get('urgent_current_lane')
                                               for r in candidates) or
                    planner.saved_world is not None or
                    any(r['check'].get('mobil_admission') for r in candidates))
                best = max(candidates, key=lambda r: (
                           (r['check'].get('mobil', {}).get('incentive_mps2') or 0.)
                           if utility_rank else round(r["check"]["average_kmh"], 1),
                           round(r["check"]["average_kmh"], 1),
                           -len(r["sequence"]), -r["check"]["peak_decel_mps2"],
                           r['length']))
                if (self.brisk_changes and planner.saved_world is None and not static_escape_needed
                        and not any(r['check'].get('urgent_current_lane') for r in candidates)):
                    # Only compare already admitted paths through this same
                    # first lane. Additional legs can be reconsidered after
                    # completing it, rather than delaying the adjacent pass.
                    # A tiny forecast tie should not buy another 100 m beside
                    # the leader. Do not trade away a useful speed advantage
                    # or add braking merely to finish the lateral move sooner.
                    quick = [r for r in candidates if r['sequence'][0] is best['sequence'][0]
                        and r['check']['average_kmh'] >= best['check']['average_kmh']-.5
                        and r['goal'] >= best['goal']-.5/3.6
                        and r['check']['peak_decel_mps2'] <= best['check']['peak_decel_mps2']+.3]
                    best = min(quick, key=lambda r: (r['check']['transition_m'], r['length']))
                gain = best["check"]["average_kmh"]-chosen["check"]["average_kmh"]
                same_continuation = (planner.saved_world is not None and
                    len(best['sequence']) == len(self.change_chains) and
                    all(l.link_id in chain for l,chain in zip(best['sequence'],self.change_chains)))
                # Free lane choice while a change is committed (no replan
                # reason): another lane, back to the one we left included,
                # replaces the admitted manoeuvre only by the cancel margin.
                margin = (self.cancel_gain_kmh if free_recheck and not same_continuation
                          else self.gain_kmh)
                needed = immediate_entry or gain >= margin
                clear_improved = (static_escape_needed and best['static_m']>base['static_m']+10.)
                # A fast rear car can share our future merge while another
                # native lane keeps it separate. Choose the checked escape
                # even when its curvature makes it slightly slower, but only
                # for a conflict inside the controller horizon: the current-lane
                # recheck uses that horizon, and a 4-5 s converging-lane forecast
                # forced 45 km/h lanes over an 81 km/h one and was cancelled on
                # the next frame (13 flips, run follow-20261009T191227Z).
                near_conflict = (base['risk'] or {}).get('collision_when')
                near_conflict = near_conflict is not None and near_conflict <= TRAFFIC_CONTROL_HORIZON_S
                # A static hit inside the firm stopping distance needs the
                # escape now; farther ones wait for the same stable margin.
                urgent_escape = near_conflict or (clear_improved and gain > .1 and
                    base['static_m'] < speed*.4+speed*speed/8.+20.)
                if urgent_escape:
                    needed = True
                # Keep an already admitted adjacent change when its shorter
                # checked continuation avoids the measured hit without losing
                # progress to the safe-current-lane fallback. This preserves
                # the manoeuvre instead of cancelling it and starting again.
                continuation_escape = (same_continuation and clear_improved and
                    best['check']['average_kmh']>=chosen['check']['average_kmh']-.1)
                needed = needed or continuation_escape
                if same_continuation:
                    # Re-rooting the same transition for a small forecast gain
                    # continually moves its finish away from the controller.
                    # Keep its measured progress unless fresh near-term danger
                    # requires a checked geometric escape.
                    imminent = (base['risk'] or {}).get('collision_when')
                    needed = needed and (clear_improved or
                        imminent is not None and imminent < 3.)
                # The stable margin belongs to the first leg (the lane actually
                # entered). Alternating one- and two-lane sequences through the
                # same adjacent lane restarted the timer for 1-2 s while a
                # 10-20 km/h gain was available (follow-20261010T140051Z lap 2).
                # Keyed by its side, not its link id: a link boundary renamed
                # the same lane (0420 -> 0408) mid-margin and restarted it again
                # (follow-20261010T180455Z lap 2, 39.4 s).
                key = 1 if best["sequence"][0].rank > current.rank else -1
                if not needed:
                    self.pending = None
                elif self.pending != key or stamp < self.pending_since:
                    self.pending, self.pending_since = key, stamp
                requested = needed
                # Discretionary changes wait for a stable margin; entry, near-term
                # escapes and re-rooting an admitted change around a hit do not.
                if needed and (immediate_entry or urgent_escape or continuation_escape or
                               stamp-self.pending_since >= self.stable_s):
                    first = best["sequence"][0]
                    # Keep the complete, already checked adjacent sequence.
                    # Revalidate it on every new observation and measure each
                    # leg's completion without discarding the continuation.
                    if best is not None:
                        chosen = best
                        planner.saved_world = world_points(best['path'], ego)
                        planner.phase = "CHANGE"
                        self.target_chain = first.chain
                        self.change_chains = [l.chain for l in best['sequence']]
                        target_lane = first
                        self.planned_sequence = tuple(l.link_id for l in best["sequence"])
                        self.change_started = stamp
                        self.progress_since = None
                        self.change_goal = best['free_goal']
                        self.change_length = best['length']
                        if immediate_entry and owned_entry:
                            self.entry_commit_identity = entry_identity
                            self.entry_commit_stamp = stamp
                        else:
                            self.reset_entry_commit()
                        planner.lane_side = first.rank
                        reason = ("highway immediate checked entry lane change" if immediate_entry
                                  else "highway checked escape: near-term conflict or static hit" if urgent_escape
                                  else "highway adjacent overtake toward fastest lane")
                        best['check']["accepted"] = True
                        # The next discretionary change needs its own stable margin.
                        self.pending = None
                        planner._speed_hold_goal = None
                        planner._speed_hold_until = 0.
            else:
                self.pending = None
        else:
            reason = ("highway entry safety fallback: "+entry_commit_cancel_reason
                      if entry_commit_cancelled else
                      "highway perception late: retain checked lane")
        if chosen is not base and not chosen.get('sequence'):
            hold_check['accepted'] = True
            planner.saved_world = None
            self.change_chains = []
            self.target_chain = ()
            self.change_goal = None
            if self.entry_commit_identity is not None:
                entry_commit_cancel_reason = hold_check.get('cancel_basis')
            self.reset_entry_commit()
            planner.phase = 'CRUISE'
            planner._speed_hold_goal = None
            planner._speed_hold_until = 0.
            self.progress_since = None
            reason = 'highway retain safe current lane; recheck next merge'
        path = chosen["path"]
        goal = chosen["goal"]
        exit_cap = math.sqrt(self.exit_speed_mps**2 + 2.*planner.longitudinal.comfort_decel_mps2*
                             max(0., remaining-planner.longitudinal.end_buffer_m))
        goal = min(goal, exit_cap, self.rule_cap_mps)
        escape_join = chosen is base and base_map_clearance is not map_clearance
        if escape_join:
            # The inward join's corner swing is accepted at creep speed only.
            goal = min(goal, ALIGN_CREEP_MPS)
        emergency = current is None
        # HighwayRuntime checks the final nudged-path speed once. Feeding a
        # short-term reference back as its goal would suppress acceleration.
        target = 0. if emergency else goal
        current_id = current.link_id if current else None
        selected_id = ((target_lane.link_id if target_lane else self.target_chain[0]) if planner.saved_world is not None
                       else current_id)
        if planner.saved_world is None and current is not None and current.merge_remaining_m < speed*7.+15.:
            planner.phase = "MERGE"
            reason = "highway native lane connection to common entrance"
        self.current_chain = current.chain if current else ()
        planner.target_offset = self.lateral(target_lane or current) if current else 0.
        curvature = float(path_curvature(path).max()) if len(path)>3 else 0.
        status = dict(phase=planner.phase, policy="free_three_lane", lane_changes=planner.completed,
            lane_change_goal=self.goal, lane_change_goal_is_minimum=False,
            lane_selection_model=self.selection_model,
            overtake_left_first=self.overtake_left_first,
            overtake_preference_applied=preference_applied,
            preferred_overtake_lane=preferred_lane.link_id if preferred_lane else None,
            preferred_overtake_lane_number=getattr(preferred_lane, 'native_lane_number', None),
            current_lane_number=getattr(current, 'native_lane_number', None),
            target_lane_number=getattr(target_lane if planner.saved_world is not None else current,
                                       'native_lane_number', None),
            path_continuity=chosen['check'].get('path_continuity', dict(applied=False)),
            further_overtake_requested=requested, current_lane=current_id, target_lane=selected_id,
            available_lanes=[l.link_id for l in lanes], candidate_checks=checks,
            available_lane_count=len(lanes),
            low_speed_alignment_recovery=alignment_recovery, alignment_escape_join=escape_join,
            entry_kink_deg=round(float(getattr(current, 'entry_kink_deg', 0.)), 1) if current else None,
            entry_lane_change_eligible=entry_ready,
            immediate_entry_lane_change=immediate_entry,
            entry_commit_active=self.entry_commit_identity is not None,
            entry_goal_refreshed=entry_goal_refreshed,
            entry_commit_cancel_reason=entry_commit_cancel_reason,
            entry_lane_change_start_idx=entry.get('from_idx'),
            current_lane_recheck=hold_check,
            planned_lane_sequence=list(self.planned_sequence),
            merge_remaining_m=current.merge_remaining_m if current else None,
            common_entry=planner.config.get("end_link"),
            target_kmh=target*3.6, cruise_kmh=planner.target_mps*3.6, reason=reason,
            speed_ceiling_kmh=chosen['ceiling_mps']*3.6,
            overtake_limit_kmh=planner.overtake_mps*3.6,
            offset_m=planner.target_offset, transition_m=transition, curvature_1pm=curvature,
            selected_transition_m=self.change_length if planner.saved_world is not None else None,
            minimum_speed_goal_kmh=min(70., planner.target_mps*3.6),
            speed_below_minimum_reason=reason if target*3.6 < min(70., planner.target_mps*3.6)-.1 else None,
            clear_distance_m=min(chosen["map_m"], chosen["static_m"]), static_mover_kmh=0.,
            clearance_components_m=dict(map=chosen["map_m"], static=chosen["static_m"]),
            clearance_query_m=chosen['query_m'],
            command_footprint_margin_m=chosen['check']['command_footprint_margin_m'],
            path_transform_rejected=chosen['check']['path_transform_rejected'],
            original_map_clear_m=chosen['check']['original_map_clear_m'],
            transformed_map_clear_m=chosen['check']['transformed_map_clear_m'],
            continuation_traffic_check_horizon_s=TRAFFIC_CONTROL_HORIZON_S,
            continuation_replan_reasons=replan_reasons,
            static_replan_reserve_m=static_replan_reserve,
            bounded_static_hit_m=static_hit_m if math.isfinite(static_hit_m) else None,
            gap_risk=chosen["risk"], perception_ready=perception_ready, emergency_stop=emergency,
            current_average_kmh=base["check"]["average_kmh"],
            comfort_decel_mps2=planner.longitudinal.comfort_decel_mps2,
            comfort_jerk_mps3=planner.longitudinal.jerk_mps3,
            exit_cap_kmh=exit_cap*3.6,
            exit_speed_kmh=self.exit_speed_mps*3.6,
            rule_speed_cap_kmh=self.rule_cap_mps*3.6 if math.isfinite(self.rule_cap_mps) else None)
        planner.status = status
        # The runtime's final map check must judge the escape join like this one.
        return dict(path=path, target_mps=target, emergency=emergency, status=status,
                    map_clearance=base_map_clearance if escape_join else None)
