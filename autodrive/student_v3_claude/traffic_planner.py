"""Stateful gap acceptance using turning vehicle predictions and timed footprints."""
from dataclasses import dataclass
import math
import os

import numpy as np

import ring_entry
from ring_entry import following_speed_cap
from stop_line import (StopLineController, STOP_WINDOW_EXTRA_M, STOP_REST_EXTRA_M,
                       INTERSECTION_ENTRY_MPS, EMPTY_ENTRY_REASON,
                       yield_line_cap, YIELD_CHECK_DISTANCE_M)
from ego_body import EGO_FRONT_M

# A moving entry avoids repeated acceleration when the measured ring gap changes.
INTERSECTION_CONTACT_MPS = 24.0 / 3.6


@dataclass(frozen=True)
class _JunctionAssessment:
    """Immutable coverage from one actual current-source map evaluation."""
    epoch: object
    moving_ids: frozenset
    incoming: tuple


def fit_circle_motion(samples, heading_deg):
    """Use a circle only when recent measured positions support constant curvature."""
    if len(samples) < 12 or samples[-1][0] - samples[0][0] < 4.0:
        return None
    data = np.asarray(samples, dtype=float)
    xy = data[:, 1:3]
    if np.linalg.norm(xy[-1] - xy[0]) < 3.0:
        return None
    origin = xy.mean(axis=0)
    q = xy - origin
    try:
        solved, _, rank, _ = np.linalg.lstsq(np.column_stack((2*q, np.ones(len(q)))), np.sum(q*q, axis=1), rcond=None)
    except np.linalg.LinAlgError:
        return None
    if rank < 3:
        return None
    centre = solved[:2] + origin
    radial = xy - centre
    radii = np.linalg.norm(radial, axis=1)
    radius = float(radii.mean())
    angles = np.unwrap(np.arctan2(radial[:, 1], radial[:, 0]))
    sweep = angles[-1] - angles[0]
    if not 5.0 <= radius <= 100.0 or abs(sweep) < math.radians(60) or np.std(radii) > .25 or np.max(np.abs(radii-radius)) > .5:
        return None
    direction = 1. if sweep > 0 else -1.
    tangent = angles[-1] + direction * math.pi/2
    error = abs((tangent - math.radians(heading_deg) + math.pi) % (2*math.pi) - math.pi)
    if error > math.radians(12):
        return None
    return dict(centre=centre.tolist(), radius=float(radii[-1]), direction=direction)


def lidar_vehicle_velocity(obj):
    """Reliable world motion is independent of PointPillars box-axis direction."""
    if obj[3] != "lidar_car" or len(obj) <= 14 or not obj[14]:
        return None
    velocity = np.asarray(obj[9:11], dtype=float)
    if velocity.shape != (2,) or not np.isfinite(velocity).all() or np.linalg.norm(velocity) > 45.:
        return None
    return velocity


def vehicle_motion_heading(obj):
    """Direction for lane matching/classification; never rewrite the OBB yaw."""
    if obj[3] == "lidar_car":
        velocity = lidar_vehicle_velocity(obj)
        return (math.atan2(velocity[1], velocity[0])
                if velocity is not None and np.linalg.norm(velocity) > .3 else None)
    return math.radians(float(obj[8])) if len(obj) > 8 else None


def prediction_motion_headings(obj, xy, shape_headings):
    """Motion classification uses trajectory tangents; SAT keeps shape headings."""
    if obj[3] != "lidar_car" or vehicle_motion_heading(obj) is None:
        return shape_headings
    delta = np.gradient(np.asarray(xy), axis=0)
    return np.unwrap(np.arctan2(delta[:, 1], delta[:, 0]))


def predict_vehicle(obj, times):
    """Constant turn-rate prediction; untracked objects retain straight prediction."""
    speed = max(0.0, float(obj[5]))
    is_lidar = obj[3] == "lidar_car"
    velocity = lidar_vehicle_velocity(obj) if is_lidar else None
    if is_lidar:
        speed = float(np.linalg.norm(velocity)) if velocity is not None else 0.
    circle = obj[13] if len(obj) > 13 else None
    if circle is not None:
        centre = np.asarray(circle["centre"])
        radius, direction = circle["radius"], circle["direction"]
        initial = math.atan2(obj[1]-centre[1], obj[0]-centre[0])
        angle = initial + direction * speed / radius * times
        xy = centre + radius * np.column_stack((np.cos(angle), np.sin(angle)))
        headings = angle + direction * math.pi / 2
        if is_lidar:
            headings = math.radians(float(obj[8])) + headings - headings[0]
        return xy, headings
    yaw = math.radians(float(obj[8]))
    if is_lidar:
        velocity = velocity if velocity is not None else np.zeros(2)
        return np.asarray(obj[:2], dtype=float)+times[:, None]*velocity, np.full(len(times), yaw)
    rate = float(obj[12]) if len(obj) > 12 else 0.0
    if not math.isfinite(rate) or abs(rate) > 1.2:
        rate = 0.0
    headings = yaw + rate * times
    if abs(rate) < 1e-4:
        xy = np.column_stack((obj[0] + speed * np.cos(yaw) * times,
                              obj[1] + speed * np.sin(yaw) * times))
    else:
        xy = np.column_stack((obj[0] + speed / rate * (np.sin(headings) - math.sin(yaw)),
                              obj[1] - speed / rate * (np.cos(headings) - math.cos(yaw))))
    return xy, headings


def overlaps(a, ayaw, b, byaw, length, width, margin=0.4, pair_mask=None):
    """Oriented rectangle overlaps for every pair of vehicle poses."""
    if pair_mask is not None:
        # Only simultaneous/uncertain-time pairs can affect the decision.
        # Keep the identical SAT geometry, avoiding the other time pairs.
        rows, cols = np.nonzero(pair_mask)
        result = np.zeros((len(a), len(b)), dtype=bool)
        if not len(rows):
            return result
        at = np.column_stack((np.cos(ayaw[rows]), np.sin(ayaw[rows])))
        an = np.column_stack((-at[:, 1], at[:, 0]))
        bt = np.column_stack((np.cos(byaw[cols]), np.sin(byaw[cols])))
        bn = np.column_stack((-bt[:, 1], bt[:, 0]))
        delta = b[cols] - a[rows]
        hit = np.ones(len(rows), dtype=bool)
        for axis in (at, an, bt, bn):
            radius = (2.3175 * np.abs((at * axis).sum(1)) + .946 * np.abs((an * axis).sum(1)) +
                      length / 2 * np.abs((bt * axis).sum(1)) + width / 2 * np.abs((bn * axis).sum(1)) + margin)
            hit &= np.abs((delta * axis).sum(1)) <= radius
        result[rows, cols] = hit
        return result
    at = np.column_stack((np.cos(ayaw), np.sin(ayaw)))[:, None, :]
    an = np.concatenate((-at[..., 1:], at[..., :1]), axis=2)
    bt = np.column_stack((np.cos(byaw), np.sin(byaw)))[None, :, :]
    bn = np.concatenate((-bt[..., 1:], bt[..., :1]), axis=2)
    delta = b[None, :, :] - a[:, None, :]
    result = np.ones(delta.shape[:2], dtype=bool)
    for axis in (at, an, bt, bn):
        radius = (2.3175 * np.abs((at * axis).sum(2)) + .946 * np.abs((an * axis).sum(2)) +
                  length / 2 * np.abs((bt * axis).sum(2)) + width / 2 * np.abs((bn * axis).sum(2)) + margin)
        result &= np.abs((delta * axis).sum(2)) <= radius
    return result


CURVE_ALAT_MPS2 = float(os.environ.get("STUDENT_JUNCTION_CURVE_ALAT", "4.2"))   # 3.5 -> 4.2 for lap time (2026-10-04)


class MapVehiclePredictor:
    """Predict along matched directed map links, retaining plausible branches."""
    def __init__(self, links):
        import ast
        self.links, self.outgoing, self.origins = [], {}, []
        all_points, all_headings, owners, indices = [], [], [], []
        for item in links:
            raw = item.get("points", [])
            points = np.asarray(ast.literal_eval(raw) if isinstance(raw, str) else raw, dtype=float)[:, :2]
            if len(points) < 2:
                continue
            keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > .01]
            points = points[keep]
            if len(points) < 2:
                continue
            heading = np.arctan2(np.gradient(points[:, 1]), np.gradient(points[:, 0]))
            index = len(self.links)
            self.links.append((points, item.get("to_node_idx")))
            self.outgoing.setdefault(item.get("from_node_idx"), []).append(index)
            self.origins.append(item.get("from_node_idx"))
            all_points.append(points); all_headings.append(heading)
            owners.extend([index] * len(points)); indices.extend(range(len(points)))
        self.points = np.vstack(all_points) if all_points else np.empty((0, 2))
        self.headings = np.concatenate(all_headings) if all_headings else np.empty(0)
        self.owners, self.indices = np.asarray(owners), np.asarray(indices)
        self._lane_point_tree = None
        if len(self.points) and np.isfinite(self.points).all():
            try:
                from scipy.spatial import cKDTree
                # Retain an independent copy so an externally edited map can
                # be detected and use the original full-scan matching below.
                self._lane_point_tree = cKDTree(self.points, copy_data=True)
            except ImportError:
                pass

    # Entry decision only (GapPlanner sets it): also start from the sibling links of the best
    # match (same diverge node) that the car is still on. 085515Z 17:59:04.71-06.45: ring car
    # 500005 sat 0.15 m from the ring link but its noisy PointPillars heading (-52 deg) scored
    # the diverging exit link (0.68 m, 11 deg) best, so its ring continuation was never
    # predicted. Siblings start on their own centreline (no 1-2 m parallel offset).
    every_match = False
    SIBLING_M = 1.5

    def _branches(self, link_id, point_id, speed, times):
        """[(path, initial_s)] along the directed links from one matched point; None on a gap."""
        points, node = self.links[link_id]
        start = max(0, point_id - 1)
        points = points[start:]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        initial_s = float(arc[point_id - start])
        needed = initial_s + speed * float(times[-1]) + 1.
        queue, completed = [(points, node, 0)], []
        while queue:
            path, node, depth = queue.pop(0)
            length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
            children = self.outgoing.get(node, [])
            if length >= needed:
                completed.append((path, initial_s))
                continue
            if not children or depth >= 20:
                return None  # Do not invent a continuation across a map gap.
            if len(queue) + len(completed) + len(children) > 8:
                return None  # Ambiguous map topology: retain the motion-model fallback.
            for child in children:
                next_points, next_node = self.links[child]
                if np.linalg.norm(path[-1] - next_points[0]) > 2.0:
                    return None
                queue.append((np.vstack((path, next_points[1:])), next_node, depth + 1))
        return completed

    def lane_paths(self, position, heading=None, max_arc=70., max_branches=16):
        """[(xy, arc)] lane centrelines from a car's matched position (arc 0) downstream.

        ``heading`` (rad, motion direction) restricts the match to lanes within 40 deg;
        None (unmeasured motion) takes every lane within 2 m, each in its own driving
        direction. Branches at diverge nodes are all kept (up to ``max_branches``)."""
        if not len(self.points):
            return []
        position = np.asarray(position, dtype=float)[:2]
        tree = self._lane_point_tree
        if (tree is not None and position.shape == (2,) and np.isfinite(position).all()
                and np.array_equal(self.points, tree.data)):
            # Include a numerical fringe, then retain the original exact radius
            # and heading predicates. Sorting preserves global-point tie order.
            candidates = np.asarray(sorted(tree.query_ball_point(position, 2.+1e-9)), dtype=np.intp)
        else:
            candidates = np.arange(len(self.points))
        distance = np.linalg.norm(self.points[candidates] - position, axis=1)
        valid = distance <= 2.0
        if heading is not None:
            angle = np.abs((self.headings[candidates] - float(heading) + math.pi) % (2 * math.pi) - math.pi)
            valid &= angle < math.radians(40)
        if not valid.any():
            return []
        starts, start_distance = {}, {}
        for local_index in np.flatnonzero(valid):
            index = int(candidates[local_index])
            link = int(self.owners[index])
            if link not in starts or distance[local_index] < start_distance[link]:
                starts[link] = int(index)
                start_distance[link] = distance[local_index]
        result = []
        for link, index in starts.items():
            points, node = self.links[link]
            point_id = int(self.indices[index])
            start = points[point_id:]
            queue = [(start, node, 0)]
            while queue and len(result) < max_branches:
                path, node, depth = queue.pop(0)
                arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
                children = self.outgoing.get(node, []) if depth < 20 else []
                if arc[-1] >= max_arc or not children:
                    result.append((path, arc))
                    continue
                for child in children:
                    child_points, child_node = self.links[child]
                    if np.linalg.norm(path[-1] - child_points[0]) > 2.0:
                        result.append((path, arc))
                        continue
                    queue.append((np.vstack((path, child_points[1:])), child_node, depth + 1))
            for path, node, _ in queue:
                if len(result) >= max_branches:
                    break
                result.append((path, np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]))
        return result

    def predict(self, obj, times, every_match=None):
        """``every_match`` (sibling branches) is per call: the decision thread and the live
        checker share this predictor, so no mutable flag may carry it between them."""
        every_match = self.every_match if every_match is None else bool(every_match)
        motion_heading = vehicle_motion_heading(obj)
        velocity = lidar_vehicle_velocity(obj) if obj[3] == "lidar_car" else None
        speed = float(np.linalg.norm(velocity)) if velocity is not None else float(obj[5])
        if not len(self.points) or speed < .3 or motion_heading is None:
            return None
        position = np.asarray(obj[:2])
        distance = np.linalg.norm(self.points - position, axis=1)
        angle = np.abs((self.headings - motion_heading + math.pi) % (2 * math.pi) - math.pi)
        valid = (distance <= 2.0) & (angle < math.radians(40))
        if not valid.any():
            return None
        score = np.where(valid, distance ** 2 + 4 * angle ** 2, np.inf)
        nearest = int(np.argmin(score))
        completed = self._branches(int(self.owners[nearest]), int(self.indices[nearest]), speed, times)
        if completed is None:
            return None
        completed = [(path, initial_s, True) for path, initial_s in completed]
        origin = self.origins[int(self.owners[nearest])]
        if every_match and origin is not None:
            for link_id in self.outgoing.get(origin, []):
                near = np.where((self.owners == link_id) & (angle < math.radians(40)), distance, np.inf)
                best = int(np.argmin(near))
                if link_id == int(self.owners[nearest]) or near[best] > self.SIBLING_M:
                    continue
                extra = self._branches(link_id, int(self.indices[best]), speed, times) or []
                completed += [(path, initial_s, False) for path, initial_s in extra][:max(0, 8 - len(completed))]
        predictions = []
        for path, initial_s, keep_offset in completed:
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
            keep = np.r_[True, np.diff(arc) > .001]
            arc, path = arc[keep], path[keep]
            travel = initial_s + speed * times
            xy = np.column_stack([np.interp(travel, arc, path[:, i]) for i in (0, 1)])
            # Preserve the measured lane offset around the curve; pulling an
            # off-centre circulating car onto the centreline invents a crossing.
            hd = np.unwrap(np.arctan2(np.gradient(xy[:, 1]), np.gradient(xy[:, 0])))
            normal = np.column_stack((-np.sin(hd), np.cos(hd)))
            offset = float(np.dot(position - xy[0], normal[0])) if keep_offset else 0.
            xy += normal * offset
            xy += (position - xy[0]) * np.exp(-speed * times[:, None] / 8.)
            hd = np.unwrap(np.arctan2(np.gradient(xy[:, 1]), np.gradient(xy[:, 0])))
            if obj[3] == "lidar_car":
                hd = math.radians(float(obj[8])) + hd - hd[0]
            if every_match and any(np.max(np.linalg.norm(xy - other, axis=1)) < 1. for other, _ in predictions):
                continue    # the same lane reached from two matched links
            predictions.append((xy, hd))
        if predictions:
            expected, _ = predict_vehicle(obj, times)
            count = max(2, int(np.searchsorted(times, 3.0)) + 1)
            predictions.sort(key=lambda pair: float(np.mean(np.linalg.norm(pair[0][:count] - expected[:count], axis=1))))
        return predictions or None


class JunctionTrackMemory:
    """Keep junction cars through LiDAR tracker dropouts (live runs 200/210/211/222).

    At the roundabout a circulating car ~10 m from us vanished from the vehicle list
    for 0.9-2.6 s and came back under a new id; every new track then reports no valid
    velocity (v_ok False, speed 0) for 0.36-0.55 s. The gap planner read the empty
    list as a gap and the new track as a parked car, and we drove into its path (222).

    * a lost car is kept for HOLD_S as a "ghost" at its predicted position (map lane
      if the predictor matches it, else constant velocity);
    * a track without valid velocity inherits the velocity of the same id or of the
      lost car whose predicted position it matches (re-identification);
    * the rest (no velocity, no history) are reported as ``unknown``.
    """
    HOLD_S = 2.0
    MATCH_M = 4.0           # measured re-id error 0.1-3.7 m after 0.9-2.6 s
    MAX_RANGE_M = 45.

    def __init__(self):
        self.tracks = {}          # uid -> dict(obj, velocity, seen, measured)
        self.last_now = None
        self.ghost_ids, self.inherited_ids, self.unknown_ids = set(), set(), set()

    def reset(self):
        self.tracks.clear()
        self.last_now = None
        self.ghost_ids, self.inherited_ids, self.unknown_ids = set(), set(), set()

    @staticmethod
    def _with_velocity(obj, velocity, xy=None, heading_deg=None):
        vx, vy = float(velocity[0]), float(velocity[1])
        x, y = (float(obj[0]), float(obj[1])) if xy is None else (float(xy[0]), float(xy[1]))
        heading = float(obj[8]) if heading_deg is None else float(heading_deg)
        row = list(obj) + [None] * max(0, 15 - len(obj))
        row[0], row[1], row[5], row[8], row[9], row[10], row[14] = x, y, math.hypot(vx, vy), heading, vx, vy, True
        return tuple(row)

    def _predict(self, track, age, predictor):
        obj, velocity = track["obj"], np.asarray(track["velocity"], dtype=float)
        if float(np.linalg.norm(velocity)) < .3 or age <= 0.:
            return np.asarray(obj[:2], dtype=float), velocity, float(obj[8])
        if predictor is not None:
            try:
                paths = predictor.predict(self._with_velocity(obj, velocity), np.asarray([0., age]))
            except Exception:
                paths = None
            if paths:
                xy, hd = paths[0]
                speed = float(np.linalg.norm(velocity))
                turn = float(hd[-1]) - float(hd[0])
                tangent = math.atan2(velocity[1], velocity[0]) + turn
                return (np.asarray(xy[-1], dtype=float),
                        speed * np.array([math.cos(tangent), math.sin(tangent)]),
                        float(obj[8]) + math.degrees(turn))
        return np.asarray(obj[:2], dtype=float) + velocity * age, velocity, float(obj[8])

    def update(self, cars, now, ego_xy=None, predictor=None):
        """cars: VehicleReceiver rows (possibly empty while not ready). Returns rows to plan with."""
        now = float(now)
        if self.last_now is not None and not 0. <= now - self.last_now <= 2.:
            self.tracks.clear()
        self.last_now = now
        self.ghost_ids, self.inherited_ids, self.unknown_ids = set(), set(), set()
        out, matched_old = [], set()
        lost = {uid: track for uid, track in self.tracks.items()
                if track["measured"] and 0. < now - track["seen"] <= self.HOLD_S}
        current_ids = {int(car[11]) for car in cars if len(car) > 11}
        for car in cars:
            if len(car) < 15 or car[3] != "lidar_car":
                out.append(car)
                continue
            uid = int(car[11])
            measured = lidar_vehicle_velocity(car) is not None
            if measured:
                # A measured new id at a lost car's predicted position is that car.
                if uid not in self.tracks:
                    for old, track in lost.items():
                        if old in current_ids or old in matched_old:
                            continue
                        xy, _, _ = self._predict(track, now - track["seen"], predictor)
                        if np.linalg.norm(xy - np.asarray(car[:2], dtype=float)) <= self.MATCH_M:
                            matched_old.add(old)
                            break
                self.tracks[uid] = dict(obj=car, velocity=(float(car[9]), float(car[10])), seen=now, measured=True)
                out.append(car)
                continue
            source = self.tracks.get(uid)
            if source is None or not source["measured"]:
                best = None
                for old, track in lost.items():
                    if old in current_ids or old in matched_old:
                        continue
                    xy, velocity, _ = self._predict(track, now - track["seen"], predictor)
                    gap = float(np.linalg.norm(xy - np.asarray(car[:2], dtype=float)))
                    if gap <= self.MATCH_M and (best is None or gap < best[0]):
                        best = (gap, old, velocity)
                if best is not None:
                    matched_old.add(best[1])
                    source = dict(velocity=tuple(best[2]), measured=True)
            if source is not None and source["measured"]:
                row = self._with_velocity(car, source["velocity"])
                self.tracks[uid] = dict(obj=row, velocity=tuple(source["velocity"]), seen=now, measured=True)
                self.inherited_ids.add(uid)
                out.append(row)
            else:
                self.tracks[uid] = dict(obj=car, velocity=(0., 0.), seen=now, measured=False)
                self.unknown_ids.add(uid)
                out.append(car)
        for uid, track in lost.items():
            if uid in current_ids or uid in matched_old:
                continue
            age = now - track["seen"]
            xy, velocity, heading = self._predict(track, age, predictor)
            if ego_xy is not None and np.linalg.norm(xy - np.asarray(ego_xy, dtype=float)) > self.MAX_RANGE_M:
                continue
            self.ghost_ids.add(uid)
            row = list(self._with_velocity(track["obj"], velocity, xy, heading))
            row += [0.] * max(0, 16-len(row))
            row[15] = max(float(row[15] or 0.), age)
            out.append(tuple(row))
        self.tracks = {uid: track for uid, track in self.tracks.items()
                       if 0. <= now - track["seen"] <= self.HOLD_S and uid not in matched_old}
        return out


# Junction entry (roundabout at the mandatory stop line; GapPlanner.junction set by gui):
# ring_entry decides wait/go at the line (follow the passing ring car in right behind
# its rear, never in front of an approaching one); once committed only following.
# Front bumper this far past the line: inside the junction (committed whatever happened).
JUNCTION_INSIDE_M = 3.

# Timing slack between our footprint and a crossing car. 1.2 s plus the 0.4 m
# shell rejected every gap behind a passing car in steady cross traffic.
CROSSING_TIME_MARGIN_S = .8
FAST_TRAFFIC_GUARD = os.environ.get("STUDENT_FAST_TRAFFIC_GUARD", "0") == "1"


class GapPlanner:
    """WAIT -> ENTER -> CRUISE. Every ENTER update still checks new collision risks."""
    ACCEL_MPS2 = 3.0
    # Fast-traffic guard (competition): a vehicle at >= 8 m/s that will occupy our path ahead
    # within 4 s makes us wait before its lane instead of creeping in (890 m merge: a 74 km/h
    # K5 treated as a "merging lead" hit us three times while we inched in at 4-8 km/h).
    FAST_MPS = 8.0
    FAST_HORIZON_S = 4.0

    def __init__(self, map_links=()):
        self.map_predictor = MapVehiclePredictor(map_links)
        self.state = "CRUISE"
        self.clear_since = None
        self.merge_path = None
        self.merge_exit = 0.0
        self.last_now = None
        self.status = {}
        self.risk_details = []
        self.followable_ids = set()
        self.entry_cap_mps = None
        self.entry_cap_distance_m = 0.
        self.entry_committed = False
        # Junction entry mode (gui sets it inside the stop-line junction region):
        # dict(held=<stop line not released yet>, ready=<LiDAR detector ready>,
        #      unknown_ids=<tracks without velocity>, line_gap_m, entry_m, ...);
        # None elsewhere (unchanged behaviour).
        self.junction = None
        # Committed (latched until the junction region ends): we accepted the entry and
        # drive off. Only following then - decelerate behind what is on our path ahead,
        # never a stop that waits for a gap (user rule 2026-10-10).
        self.junction_committed = False
        self.junction_wait_since = None
        # Ring-car speed bound ratios by uid (gui: ring_entry.RingMotionTracker).
        self.speed_ratio = {}
        self._standing_since = {}
        self._junction_assessment = None
        self._junction_evaluation_epoch = None

    @property
    def committed_entry(self):
        """Use the accepted junction state for both geometry and headway checks."""
        if isinstance(self.junction, dict):
            return bool(self.junction_committed and not self.junction.get("held"))
        return self.entry_committed

    def latch_junction_commit(self):
        """The stop line released us into the junction: following only from now on."""
        if isinstance(self.junction, dict):
            self.junction_committed = True
            self.junction_wait_since = None

    def evaluate(self, objects, ego, local_path, target_mps, now, fresh=True):
        self._junction_assessment = None
        self._junction_evaluation_epoch = object()
        self._prediction_cache = {}
        if isinstance(self.junction, dict):
            # One pass: no acceleration-candidate retries at the junction (they quadrupled
            # the decision time there, 200 -> 820 ms median in follow-20261010T113545Z).
            result = self._evaluate_junction(objects, ego, local_path, target_mps, now, fresh)
            self._update_junction_commit(result, ego)
            return result
        # Try feasible crossing speeds before deciding to stop. Each candidate
        # is checked with the same footprint and acceleration uncertainty.
        fields = ("state", "clear_since", "merge_path", "merge_exit", "last_now", "status", "risk_details",
                  "followable_ids")
        saved = {key: getattr(self, key) for key in fields}
        first = None
        first_state = None
        result = None
        for factor in (1., .85, .7, .55):
            for key, value in saved.items():
                setattr(self, key, value)
            result = self._evaluate(objects, ego, local_path, target_mps * factor, now, fresh)
            if factor == 1.:
                first, first_state = result, {key: getattr(self, key) for key in fields}
            if result is None or result["kind"] != "stop":
                break
            if result["reason"] != "predicted footprint time overlap":
                break
        else:
            for key, value in first_state.items():
                setattr(self, key, value)
            result = first
        self._update_junction_commit(result, ego)
        return result

    def _update_junction_commit(self, result, ego):
        if not isinstance(self.junction, dict):
            self.junction_committed = False
            self.junction_wait_since = None
            self._standing_since = {}
            return
        if result is not None:
            result["junction_committed"] = self.junction_committed

    def _approaching_stop_line(self, speed=None):
        junction = self.junction if isinstance(self.junction, dict) else None
        if junction and junction.get('yield_only'):
            return False
        gap = junction.get("line_gap_m") if junction else None
        # At rest the stop line counts STOP_REST_EXTRA_M more as stopped at the line (092032Z
        # lap 2 rested at 1.12-1.16 m); "approach" there would never report a clear entry.
        window = StopLineController.BUFFER_M + STOP_WINDOW_EXTRA_M + (
            STOP_REST_EXTRA_M if speed is not None and speed <= .15 else 0.)
        return bool(junction and junction.get("held") and not self.junction_committed
                    and not junction.get("at_line")
                    and gap is not None and math.isfinite(float(gap)) and gap > window)

    def _path_geometry(self, ego, local_path):
        """World path, its stations/headings and the curve speed limit (None if unusable)."""
        path = np.asarray(local_path, dtype=float)[:, :2]
        if len(path) < 3 or not np.isfinite(path).all():
            return None
        yaw = math.radians(ego["heading_deg"])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        world = path @ rotation.T + [ego["x"], ego["y"]]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(world, axis=0), axis=1))]
        keep = np.r_[True, np.diff(arc) > .001]
        arc, world = arc[keep], world[keep]
        if len(arc) < 3 or arc[-1] < 8:
            return None
        headings = np.unwrap(np.arctan2(np.gradient(world[:, 1]), np.gradient(world[:, 0])))
        # Nonuniform link endpoints can be only centimetres apart. Estimate
        # physical curvature on a metre-spaced chord, not index-based gradients.
        # 2 m chords: 1 m three-point curvature spiked on map polyline joints.
        curve_s = np.arange(0., arc[-1] + .01, 2.)
        curve_xy = np.column_stack([np.interp(curve_s, arc, world[:, i]) for i in (0, 1)])
        ab, bc = np.diff(curve_xy, axis=0)[:-1], np.diff(curve_xy, axis=0)[1:]
        ac = ab + bc
        curvature = 2 * np.abs(ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]) / np.maximum(
            1e-6, np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ac, axis=1))

        # Speed allowed at each curvature sample (CURVE_ALAT_MPS2) and, before it, the
        # speed from which comfortable braking still reaches it there.
        sample_s = curve_s[1:-1]
        sample_v = np.sqrt(CURVE_ALAT_MPS2 / np.maximum(.001, curvature)) if len(curvature) else np.empty(0)

        def speed_limit(station):
            """Braking-aware curve speed at a path station: the exit curve 25 m ahead does
            not cap the turn-in (follow-20261010T130317Z: 21.5 km/h from the line on)."""
            if not len(sample_s):
                return math.inf
            ahead = sample_s >= float(station) - 1.
            if not ahead.any():
                return float(sample_v[-1])
            room = np.maximum(0., sample_s[ahead] - float(station))
            return float(np.min(np.sqrt(sample_v[ahead] ** 2 + 2. * ring_entry.CURVE_BRAKE_MPS2 * room)))
        return world, arc, headings, speed_limit

    def _lane_paths(self, obj, heading, velocity):
        """Lane centrelines a car can drive along (map), else its measured straight motion."""
        paths = self.map_predictor.lane_paths(obj[:2], heading, ring_entry.LANE_LOOKUP_M)
        junction = self.junction or {}
        uncertain = set(junction.get('unknown_ids') or ())
        uncertain.update(junction.get('ghost_ids') or ())
        uncertain.update(junction.get('inherited_ids') or ())
        if (paths and junction.get('following_only') and heading is not None
                and obj[3] == 'lidar_car' and len(obj) > 14 and obj[14]
                and obj[11] not in uncertain and (len(obj) <= 15 or not obj[15])):
            # A measured circulating car must not snap to a farther intersecting
            # lane simply because that lane's conflict has an earlier arc. Keep
            # every downstream branch from each plausible nearby lane seed.
            distances = np.array([np.linalg.norm(np.asarray(xy[0])[:2]-obj[:2])
                                  for xy, _ in paths])
            paths = [path for path, distance in zip(paths, distances)
                     if distance <= float(np.min(distances))+.75]
        if not paths and heading is not None:
            paths = self.map_predictor.lane_paths(obj[:2], None, ring_entry.LANE_LOOKUP_M)
        if not paths and velocity is not None:
            paths = ring_entry.straight_lane(obj[:2], velocity)
        return paths

    def _evaluate_junction(self, objects, ego, local_path, target_mps, now, fresh=True):
        """Roundabout entry at the stop line (ring_entry). Approach: drive up to the line.
        At the line: wait, or go once the passing ring car's rear will be just ahead of our
        nose and every later ring car reaches our conflict point >= 1 s after our rear.
        Committed: follow only (decelerate behind what is on our path ahead)."""
        junction = self.junction
        self.followable_ids = set()
        self.risk_details = []
        if self.last_now is not None and not 0 <= now - self.last_now <= 1.0:
            self.state = "CRUISE"
        self.last_now = now
        speed = max(0.0, float(ego.get("speed_kmh", 0))) / 3.6

        def answer(kind, name, along, when, cap, reason, **extra):
            self.status = dict(state=self.state, reason=reason, target_mps=float(cap), **extra)
            return dict(kind=kind, name=name, along=float(along), t=float(when),
                        v_mps=float(cap), source="traffic", state=self.state, reason=reason)

        geometry = self._path_geometry(ego, local_path)
        if geometry is None:
            self.state = "WAIT"
            return answer("stop", "path", 0, 0, 0, "invalid merge path")
        world, arc, _, speed_limit = geometry
        frame = ring_entry.PathFrame(world)
        centre = float(frame.project(np.asarray([ego["x"], ego["y"]], dtype=float))[0][0])
        if not math.isfinite(centre):
            centre = 0.
        ceiling = max(0., float(target_mps))
        following_only = bool(junction.get('following_only'))
        if following_only:
            ceiling = min(ceiling, INTERSECTION_CONTACT_MPS)
        if self.entry_cap_mps is not None and self.entry_cap_distance_m > 0.:
            ceiling = min(ceiling, float(self.entry_cap_mps))
        target = min(ceiling, speed_limit(centre))

        def entry_limit(station):
            return min(ceiling, speed_limit(station))
        yield_only = bool(junction.get('yield_only'))
        lead_cap, leader = ring_entry.follow_cap(objects, frame, centre, speed,
            ego_front_m=EGO_FRONT_M if yield_only or following_only else ring_entry.EGO_HALF_LENGTH_M,
            current_ego=ego if yield_only and junction.get('constant_ring_speed') else None)
        if leader is not None and leader.get("object_id") is not None:
            self.followable_ids = {leader["object_id"]}
        follow = min(target, lead_cap)

        if junction.get('recovery_only') and yield_only and not self.junction_committed:
            # Observed startup occupancy is not a newly accepted ring gap.
            # The publisher independently retains every NPC forecast here.
            self.state, self.junction_wait_since = 'RECOVERY_FOLLOW', None
            if not fresh or not junction.get('ready', True):
                return answer('stop', 'object_stream', 0., 0., 0.,
                              'junction recovery traffic stream unavailable')
            return answer('follow' if leader is not None else 'enter',
                leader['name'] if leader else 'path',
                leader['along'] if leader else 0., leader['t'] if leader else 0., follow,
                'junction recovery: follow with full live collision checks', leader=leader)

        if following_only:
            # Remove the surveyed-line wait only. Do not mark an unchecked
            # entry committed: publication-time body/crossing/person guards
            # must keep their independent emergency braking authority.
            self.junction_committed = False
            self.junction_wait_since = None
            if not fresh or not junction.get('ready', True):
                self.state = 'WAIT'
                return answer('stop', 'object_stream', 0., 0., 0.,
                              'junction traffic stream unavailable')
            cars, unknown = ring_entry.classify(objects, frame, centre, self._lane_paths,
                self.speed_ratio, junction.get('unknown_ids') or (),
                (float(ego['x']), float(ego['y']), math.radians(float(ego['heading_deg']))),
                observed_flow_ids=({o[11] for o in objects if len(o)>15
                    and o[3]=='lidar_car' and o[14] and not o[15]}-set().union(
                    *(set(junction.get(field) or ()) for field in ('unknown_ids', 'ghost_ids', 'inherited_ids'))))
                    if junction.get('constant_ring_speed') else ())
            # Constant-velocity body checks cannot anticipate a car turning
            # along the ring. Keep the existing conservative map timing proof,
            # but brake before its actual contact corridor, never the stop line.
            decision = ring_entry.entry_decision(cars, centre, speed, max(1., target),
                speed_limit=entry_limit, frame=frame, yield_only=True, body_geometry=True,
                launch_state=ring_entry.pose_launch_state(ego))
            contacts = []
            if decision['action'] == 'WAIT':
                checked_ids = {item['object_id'] for item in decision['checks']
                    if item['margin_s'] < ring_entry.NEXT_MARGIN_S + .01}
                contacts.extend((min(c.conflict['s_lane'],
                    c.conflict['s_cp']-ring_entry.LANE_CONTACT_M), c.name)
                    for c in cars if c.uid in checked_ids)
            relevant = ring_entry.unknown_relevant(unknown, centre,
                np.asarray([ego['x'], ego['y']], dtype=float))
            relevant_ids = {id(obj) for obj in relevant}
            if any(id(obj) in relevant_ids and conflict is None for obj, conflict, _ in unknown):
                self.state = 'WAIT'
                return answer('stop', 'unmeasured_vehicle', 0., 0., 0.,
                              'junction: nearby car without measured motion or mapped corridor')
            contacts.extend((min(conflict['s_lane'],
                conflict['s_cp']-ring_entry.LANE_CONTACT_M), obj[3])
                for obj, conflict, _ in unknown
                if id(obj) in relevant_ids and conflict is not None)
            if contacts:
                boundary, name = min(contacts)
                # The boundary already marks contact by the physical front.
                # yield_line_cap reserves its own buffer, observation travel,
                # reaction and braking distance; pass the actual gap once.
                gap = max(0., boundary-centre-EGO_FRONT_M)
                cap = min(follow, yield_line_cap(gap, speed, float(ego.get('age_s', 0.))))
                if cap <= .5 and speed <= .5:
                    # Hold the reached position at an occupied physical
                    # corridor instead of repeatedly requesting a tiny crawl.
                    # A fresh GO below has no such capture or release timer.
                    cap = 0.
                self.state = 'CROSSING_CAUTION'
                return answer('follow' if cap > 0. else 'stop', name, gap, 0., cap,
                    'junction: brake before occupied ring corridor', ring=decision,
                    contact_gap_m=gap)
            self.state = 'FOLLOW' if leader is not None else 'CRUISE'
            if decision['action'] == 'GO':
                incoming = tuple((item['object_id'], float(item['required_rear_travel_m']))
                                 for item in decision['checks'])
                included = set(decision['ahead']) | {uid for uid, _ in incoming}
                self._junction_assessment = _JunctionAssessment(
                    self._junction_evaluation_epoch,
                    frozenset(car.uid for car in cars if car.kind == 'moving' and car.uid in included),
                    incoming)
            return answer('follow' if leader is not None else 'enter',
                leader['name'] if leader else 'path',
                leader['along'] if leader else 0., leader['t'] if leader else 0., follow,
                'junction: follow the car ahead' if leader else 'junction: drive without stop-line wait',
                leader=leader, ring=decision)

        def wait(name, along, when, reason, **extra):
            if yield_only:
                self.state = 'YIELD_WAIT'
                cap = min(follow, yield_line_cap(junction.get('line_gap_m'), speed,
                                                 float(ego.get('age_s', 0.))))
                return answer('follow' if cap > 0. else 'stop', name, along, when, cap,
                              reason, **extra)
            self.state = 'WAIT'
            return answer('stop', name, along, when, 0., reason, **extra)

        entry_m = junction.get("entry_m")
        if (not yield_only and not self.junction_committed and not junction.get("held") and entry_m is not None
                and math.isfinite(float(entry_m)) and float(entry_m) >= JUNCTION_INSIDE_M):
            self.junction_committed = True      # past the line we only got by a release
        if self.junction_committed:
            # Following only: a car or obstacle ahead slows us down (to a standstill only
            # behind a stopped leader); nothing beside or behind us is waited for.
            self.state, self.junction_wait_since = "ENTER", None
            if leader is not None:
                return answer("follow", leader["name"], leader["along"], leader["t"], follow,
                              "junction committed: follow the car ahead", leader=leader)
            return answer("enter", "gap", 0., 0., follow, "junction committed: drive on")

        if yield_only and (junction.get('line_gap_m') is None
                or float(junction['line_gap_m']) > YIELD_CHECK_DISTANCE_M):
            # Until the ring lies inside the checked horizon, retain the normal
            # braking envelope. A far-away empty horizon is not entry permission.
            self.state = 'APPROACH_YIELD'
            cap = min(follow, yield_line_cap(junction.get('line_gap_m'), speed,
                                            float(ego.get('age_s', 0.))))
            return answer('follow', 'yield_line', 0., 0., cap,
                          'approach junction within braking envelope')

        if (junction.get("allow_empty_entry") is True and
                not yield_only and
                junction.get("empty_observation") is True and fresh and
                junction.get("ready") is True and not junction.get("at_line")
                and not objects and target > .3):
            # Empty-entry clearance is provisional. Keep checking every frame;
            # only the stop-line gate can commit after the front crosses the line.
            # A latched stop uses the normal timed gap even if cars disappear:
            # launch from rest on GO instead of restarting a 2.5 km/h crawl.
            self.state, self.junction_wait_since = "APPROACH_EMPTY", None
            return answer("enter", "empty_junction", 0., 0.,
                          min(target, INTERSECTION_ENTRY_MPS), EMPTY_ENTRY_REASON,
                          empty_observation=True)

        if self._approaching_stop_line(speed):
            # Up to the line the stop line controller owns the stop; ring traffic cannot
            # reach us before it. No readiness/unknown-motion stops here (follow-
            # 20261010T113545Z lap 2 stood 8.4 m before the line for a new track 45 m away).
            self.state, self.junction_wait_since = "APPROACH", None
            line_gap = float(junction["line_gap_m"])
            # A car beyond the line is no reason to slow before it: we stop at the line.
            cap = follow if leader is None or leader["gap_m"] <= line_gap + 1. else target
            return answer("follow", "stop_line", line_gap, 0., cap,
                          "junction: approach mandatory stop line", leader=leader)

        # At the line, not committed: decide wait / go.
        if speed < .3:
            if self.junction_wait_since is None:
                self.junction_wait_since = now
        else:
            self.junction_wait_since = None
        if not fresh:
            return wait("object_stream", 0, 0, "objects stale")
        if not junction.get("ready", True):
            return wait("lidar_vehicles", 0, 0, "junction: LiDAR vehicles not ready, wait")
        cars, unknown = ring_entry.classify(objects, frame, centre, self._lane_paths, self.speed_ratio,
                                            junction.get("unknown_ids") or (),
                                            (float(ego["x"]), float(ego["y"]), math.radians(float(ego["heading_deg"]))),
                                            observed_flow_ids=({o[11] for o in objects if len(o)>15
                                                and o[3]=='lidar_car' and o[14] and not o[15]}-set().union(
                                                *(set(junction.get(field) or ()) for field in ('unknown_ids', 'ghost_ids', 'inherited_ids'))))
                                                if junction.get('constant_ring_speed') else ())
        relevant = ring_entry.unknown_relevant(unknown, centre, np.asarray([ego["x"], ego["y"]], dtype=float))
        if relevant:
            obj = relevant[0]
            return wait(obj[3], 0, 0, "junction: car without measured velocity, wait",
                          unknown_ids=[o[11] for o in relevant if len(o) > 11])
        standoff = self._standoff_ids(cars, speed, now)
        decision = ring_entry.entry_decision(cars, centre, speed, max(1., target), standoff,
                                             speed_limit=entry_limit, frame=frame, yield_only=yield_only,
                                             body_geometry=bool(yield_only and junction.get('empty_actual_body')))
        ring = dict(decision, centre_s=round(centre, 2), standoff=sorted(standoff))
        if decision["action"] == "WAIT":
            self.risk_details = [dict(object_id=decision["blocker"], hypothesis=0, when=decision["rear_clear_s"],
                                      object_time=decision["earliest_s"], ego_xy=None, object_xy=None, ahead=False,
                                      name=decision["blocker_name"], along=float(decision["s_cp"] - centre))]
            return wait(decision["blocker_name"], decision["s_cp"] - centre, decision["rear_clear_s"],
                          "junction: wait at the line for a ring gap", ring=ring)
        self.followable_ids |= set(decision["ahead"])
        if follow <= .3:
            # The car we would follow is still right in front of us: wait for it to move on.
            return wait(leader["name"] if leader else "gap", leader["along"] if leader else 0., 0.,
                          "junction: wait for the car ahead to move on", ring=ring, leader=leader)
        if yield_only:
            incoming = tuple((item['object_id'], float(item['required_rear_travel_m']))
                             for item in decision['checks'])
            included = set(decision['ahead']) | {uid for uid, _ in incoming}
            self._junction_assessment = _JunctionAssessment(
                self._junction_evaluation_epoch,
                frozenset(car.uid for car in cars if car.kind == 'moving' and car.uid in included),
                incoming)
        self.state = "APPROACH_YIELD" if yield_only else "ENTER"
        if not junction.get("held") and not yield_only:
            self.junction_committed = True      # released: following only from here
        if decision["leader"] is not None or leader is not None:
            return answer("follow", leader["name"] if leader else "lidar_car",
                          leader["along"] if leader else 0., 0., follow,
                          "follow merging lead with checked headway", ring=ring, leader=leader)
        return answer("enter", "gap", 0., 0., follow, "gap accepted with acceleration and clearance", ring=ring)

    def _standoff_ids(self, cars, speed, now):
        """Standing ring cars beside our merge while we stand at the line yield to us."""
        standing = {car.uid for car in cars if car.kind == "standing"
                    and car.conflict["arc_cp"] <= ring_entry.STANDOFF_ARC_M}
        self._standing_since = {uid: self._standing_since.get(uid, now) for uid in standing}
        if speed >= .3 or self.junction_wait_since is None or now - self.junction_wait_since < ring_entry.STANDOFF_S:
            return set()
        return {uid for uid, since in self._standing_since.items() if now - since >= ring_entry.STANDOFF_S}

    def _fast_traffic_wait(self, predictions, leader_ids, route_s, route_xy, route_yaw, times, speed):
        """(zone begin m, time s, name) if a fast vehicle will occupy our path ahead soon and we can
        still stop before that zone; None otherwise (including when we are already inside it)."""
        front = 2.3175
        stop_reach = speed * .3 + speed * speed / (2 * 2.5)
        best = None
        for obj, xy, hd, length, width, hypothesis, _ in predictions:
            identity = int(obj[11]) if len(obj) > 11 else id(obj)
            if hypothesis or float(obj[5]) < self.FAST_MPS or identity in leader_ids:
                continue
            nearest = int(np.argmin(np.linalg.norm(route_xy - xy[0], axis=1)))
            tangent = np.array([math.cos(route_yaw[nearest]), math.sin(route_yaw[nearest])])
            along = route_s[nearest] + float(np.dot(xy[0] - route_xy[nearest], tangent))
            if along < 0.:
                # Behind us: it reaches our path only by following us. Slowing
                # for it invites a rear-end (virtual junction run 2026-10-03).
                continue
            motion = vehicle_motion_heading(obj)
            lateral = abs(float(np.dot(xy[0] - route_xy[nearest], [-tangent[1], tangent[0]])))
            if (motion is not None and lateral < 2.5 and
                    abs((motion - route_yaw[nearest] + math.pi) % (2 * math.pi) - math.pi) < math.radians(30)):
                # Already ahead in our lane and direction: headway, not a crossing.
                continue
            horizon = times <= self.FAST_HORIZON_S
            grid = overlaps(route_xy, route_yaw, xy[horizon], hd[horizon], length, width, margin=.8)
            ahead = route_s >= front - .5
            grid &= ahead[:, None]
            if not grid.any():
                continue
            rows, cols = np.nonzero(grid)
            begin = float(route_s[rows.min()])
            when = float(times[horizon][cols.min()])
            if begin - front <= .5:
                continue                     # already in its lane: the normal logic clears it
            if begin - front - .8 < stop_reach:
                continue                     # cannot stop short of it: do not park inside
            if best is None or when < best[1]:
                best = (begin, when, obj[3])
        return best

    def _evaluate(self, objects, ego, local_path, target_mps, now, fresh=True):
        self.followable_ids = set()
        if self.last_now is not None and not 0 <= now - self.last_now <= 1.0:
            self.state, self.clear_since, self.merge_path = "CRUISE", None, None
        self.last_now = now
        def answer(kind, name, along, when, speed, reason):
            self.status = dict(state=self.state, reason=reason, target_mps=float(speed))
            return dict(kind=kind, name=name, along=float(along), t=float(when),
                        v_mps=float(speed), source="traffic", state=self.state, reason=reason)
        if not fresh:
            self.state, self.clear_since = "WAIT", None
            return answer("stop", "object_stream", 0, 0, 0, "objects stale")
        path = np.asarray(local_path, dtype=float)[:, :2]
        if len(path) < 3 or not np.isfinite(path).all():
            self.state, self.clear_since = "WAIT", None
            return answer("stop", "path", 0, 0, 0, "invalid merge path")
        yaw = math.radians(ego["heading_deg"])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        world = path @ rotation.T + [ego["x"], ego["y"]]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(world, axis=0), axis=1))]
        keep = np.r_[True, np.diff(arc) > .001]
        arc, world = arc[keep], world[keep]
        if len(arc) < 3 or arc[-1] < 8:
            self.state, self.clear_since = "WAIT", None
            return answer("stop", "path", 0, 0, 0, "merge path too short")
        headings = np.unwrap(np.arctan2(np.gradient(world[:, 1]), np.gradient(world[:, 0])))
        # Nonuniform link endpoints can be only centimetres apart. Estimate
        # physical curvature on a metre-spaced chord, not index-based gradients.
        # 2 m chords: 1 m three-point curvature spiked on map polyline joints.
        curve_s = np.arange(0., arc[-1] + .01, 2.)
        curve_xy = np.column_stack([np.interp(curve_s, arc, world[:, i]) for i in (0, 1)])
        ab, bc = np.diff(curve_xy, axis=0)[:-1], np.diff(curve_xy, axis=0)[1:]
        ac = ab + bc
        curvature = 2 * np.abs(ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]) / np.maximum(
            1e-6, np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ac, axis=1))
        def curve_limit(distance):
            ahead = (curve_s[1:-1] >= distance) & (curve_s[1:-1] <= distance + 25.)
            maximum = float(np.max(curvature[ahead])) if ahead.any() else float(curvature[-1])
            # 3.5 m/s^2 as the follower's curve cap (was 2.26: 14 km/h exits).
            return math.sqrt(CURVE_ALAT_MPS2 / max(.001, maximum))
        requested = max(0.0, float(target_mps))
        target = min(requested, curve_limit(0.))
        speed = max(0.0, float(ego.get("speed_kmh", 0))) / 3.6
        times = np.arange(0., 8.01, .2)
        approach = self._approaching_stop_line(speed)
        stop_distance = (max(0., float(self.junction["line_gap_m"]) -
                             StopLineController.BUFFER_M) if approach else None)
        # Match the Student follower's acceleration ramp. Check +/-20% response
        # variation as well as the separate footprint timing margin.
        def rollout(accel, dead_s=0.):
            distances, distance, velocity = [0.0], 0.0, speed
            for step, t in enumerate(times[1:], 1):
                if t <= dead_s + 1e-9:
                    # Launch dead time from standstill (measured ~0.65 s at the junction).
                    distance += velocity * .2
                    distances.append(distance)
                    continue
                future_target = min(requested, curve_limit(distance))
                if (self.entry_cap_mps is not None and self.entry_cap_distance_m > 0.
                        and distance <= self.entry_cap_distance_m):
                    future_target = min(future_target, self.entry_cap_mps)
                if approach:
                    available = max(0., stop_distance - distance -
                                    velocity * StopLineController.REACTION_S)
                    future_target = min(future_target, math.sqrt(
                        2. * StopLineController.BRAKE_MPS2 * available))
                for lead_s, lead_speed, lead_length, lead_on_path in leader_profiles:
                    if (lead_on_path[step] and (not approach or
                            lead_s[step] - lead_length / 2. <= stop_distance + 2.3175)):
                        gap = lead_s[step] - distance - 2.3175 - lead_length / 2
                        cap = (following_speed_cap(gap, lead_speed, velocity) if self.committed_entry
                               else max(0., min(lead_speed, (gap - 2.) / 1.2)))
                        future_target = min(future_target, cap)
                next_v = velocity + np.clip(future_target - velocity, -2.5 * .2, accel * .2)
                distance += (velocity + next_v) * .1
                velocity = next_v
                distances.append(distance)
            return np.asarray(distances)
        merge_progress = 0.0
        if self.merge_path is not None:
            old_world, old_arc = self.merge_path
            nearest = np.argmin(np.linalg.norm(old_world - [ego["x"], ego["y"]], axis=1))
            if np.linalg.norm(old_world[nearest] - [ego["x"], ego["y"]]) > 5 or old_arc[nearest] >= self.merge_exit:
                self.state, self.merge_path, self.clear_since = "CRUISE", None, None
            else:
                merge_progress = float(old_arc[nearest] - old_arc[0])
        route_s = np.arange(0., min(40., arc[-1]) + .01, .5)
        route_xy = np.column_stack([np.interp(route_s, arc, world[:, i]) for i in (0, 1)])
        route_yaw = np.interp(route_s, arc, headings)
        risks, crossings, leaders, merge_leaders, predictions = [], [], [], [], []
        leader_ids = set()
        leader_profiles = []
        self.risk_details = []
        for obj in objects:
            if obj[4] != "npc":
                continue
            if np.linalg.norm(np.asarray(obj[:2]) - [ego["x"], ego["y"]]) > obj[5] * times[-1] + 50:
                continue
            cache = getattr(self, "_prediction_cache", {})
            alternatives = cache.get(id(obj))
            if alternatives is None:
                circle = obj[13] if len(obj) > 13 else None
                if circle is not None:
                    alternatives = [predict_vehicle(obj, times)]
                else:
                    alternatives = self.map_predictor.predict(obj, times, every_match=False)
                if alternatives is None:
                    alternatives = [predict_vehicle(obj, times)]
                cache[id(obj)] = alternatives
            for hypothesis, (xy, hd) in enumerate(alternatives):
                length, width = max(.5, float(obj[6])), max(.5, float(obj[7]))
                spatial = overlaps(route_xy, route_yaw, xy, hd, length, width)
                nearest = int(np.argmin(np.linalg.norm(route_xy - xy[0], axis=1)))
                delta = xy[0] - route_xy[nearest]
                tangent = np.array([math.cos(route_yaw[nearest]), math.sin(route_yaw[nearest])])
                lateral = float(np.dot(delta, [-tangent[1], tangent[0]]))
                along = float(route_s[nearest] + np.dot(delta, tangent))
                motion_heading = vehicle_motion_heading(obj)
                classification_heading = motion_heading if motion_heading is not None else hd[0]
                angle = abs((classification_heading - route_yaw[nearest] + math.pi) % (2 * math.pi) - math.pi)
                same_lane = angle < math.radians(30) and abs(lateral) <= .946 + width / 2 + .2
                # After an approved entry, a car following from behind cannot
                # revoke the merge merely by predicting that it will catch us.
                # Keep actual contact, crossing cars and unmeasured motion.
                measured = lidar_vehicle_velocity(obj) is not None
                touching = overlaps(np.asarray([[ego["x"], ego["y"]]]), np.asarray([yaw]),
                                    xy[:1], hd[:1], length, width, margin=0.).any()
                if (self.committed_entry and measured and same_lane and along < -2.3175
                        and not touching):
                    continue
                velocity = lidar_vehicle_velocity(obj)
                object_speed = float(np.linalg.norm(velocity)) if velocity is not None else float(obj[5])
                if along < -.5 and object_speed < 1.:
                    # A stopped car beside/behind our centre cannot reach us if
                    # we drive on. In the roundabout it had stopped to yield to
                    # us while we waited for it: 85 s mutual deadlock (2026-10-03).
                    # Its PointPillars box was rotated ~35 deg (heading flipping),
                    # so even a 0 m shell "touched" us; waiting never resolves it.
                    continue
                delta_grid = xy[:, None, :] - route_xy[None, :, :]
                nearest_future = np.argmin(np.linalg.norm(delta_grid, axis=2), axis=1)
                future_tangent = np.column_stack((np.cos(route_yaw[nearest_future]), np.sin(route_yaw[nearest_future])))
                relative = xy - route_xy[nearest_future]
                future_s = route_s[nearest_future] + np.sum(relative * future_tangent, axis=1)
                future_lat = np.abs(np.sum(relative * np.column_stack((-future_tangent[:, 1], future_tangent[:, 0])), axis=1))
                motion_hd = prediction_motion_headings(obj, xy, hd)
                future_angle = np.abs((motion_hd - route_yaw[nearest_future] + math.pi) % (2 * math.pi) - math.pi)
                on_route = (future_s > 3.) & (future_lat < .946 + width / 2 + .2) & (future_angle < math.radians(50))
                # A turning car becomes a leader as soon as its predicted path
                # joins ours for at least one second. A car merely crossing the
                # lane is still treated as cross traffic.
                first_on = int(np.flatnonzero(on_route)[0]) if on_route.any() else len(times)
                same_flow = (hypothesis == 0 and
                             any(on_route[j:j + 6].all() and future_s[j + 5] - future_s[j] >= 2.5
                                 for j in range(first_on, min(len(times) - 5, first_on + 25))))
                followable = same_flow and first_on <= 10
                if same_lane and 0 < along < 25 and hypothesis == 0:
                    # All fused vehicles share a class name; identity is needed
                    # to exempt only this leader from the fast-crossing guard.
                    leader_ids.add(int(obj[11]) if len(obj) > 11 else id(obj))
                    if len(obj) > 11:
                        self.followable_ids.add(int(obj[11]))
                    leader_profiles.append((future_s, float(obj[5]), length, on_route))
                    gap = along - 2.3175 - length / 2
                    leader_cap = (following_speed_cap(gap, obj[5], speed)
                                  if self.committed_entry and measured else
                                  max(0.0, min(float(obj[5]), (gap - 2.0) / 1.2)))
                    if not approach or along - length / 2. <= stop_distance + 2.3175:
                        leaders.append((leader_cap, obj[3], along))
                elif followable:
                    if len(obj) > 11:
                        self.followable_ids.add(int(obj[11]))
                    leader_profiles.append((future_s, float(obj[5]), length, on_route))
                    merge_leaders.append((float(obj[5]), obj[3], float(future_s[first_on])))
                angle_grid = np.abs((route_yaw[:, None] - motion_hd[None, :] + math.pi) % (2 * math.pi) - math.pi)
                crossing_indices = np.flatnonzero((spatial & (angle_grid >= math.radians(30))).any(axis=1))
                if len(crossing_indices) and hypothesis == 0:
                    for group in np.split(crossing_indices, np.flatnonzero(np.diff(crossing_indices) > 1) + 1):
                        crossings.append((float(route_s[group[0]]), float(route_s[group[-1]]), obj[3]))
                predictions.append((obj, xy, hd, length, width, hypothesis, same_flow))
        if leaders:
            target = min(target, min(c[0] for c in leaders))
        guard_approach = None
        if FAST_TRAFFIC_GUARD:
            guard = self._fast_traffic_wait(predictions, leader_ids, route_s, route_xy,
                                            route_yaw, times, speed)
            if approach and guard is not None and guard[0] > stop_distance:
                guard = None
            if guard is not None:
                begin, when, name = guard
                # A distant occupied lane needs a braking-safe approach, not a
                # full stop where we first see it. Recheck every observation
                # and stop before the zone if the vehicle has not cleared it.
                available = max(0., begin - 2.3175 - .8 - 2.)
                cap = max(0., math.sqrt((2.5 * .7) ** 2 + 5. * available) - 2.5 * .7)
                if cap > .3:
                    guard_approach = (begin, when, name, min(target, cap))
                    target = min(target, cap)
                    requested = min(requested, cap)
                else:
                    self.state, self.clear_since = "WAIT", None
                    return answer("stop", name, begin, when, 0, "fast traffic: wait before its lane")
        motion_envelope = [rollout(self.ACCEL_MPS2 * factor) for factor in (.8, 1., 1.2)]
        brake_distances, brake_s, brake_v = [0.], 0., speed
        for _ in times[1:]:
            next_v = max(0., brake_v - 2.5 * .2)
            brake_s += (brake_v + next_v) * .1
            brake_v = next_v
            brake_distances.append(brake_s)
        brake_distances = np.asarray(brake_distances)
        brake_times = []
        time_offset = np.abs(times[:, None] - times[None, :])
        ego_predictions = []
        for braking, distances in [(False, d) for d in motion_envelope] + [(True, brake_distances)]:
            ego_xy = np.column_stack([np.interp(distances, arc, world[:, i]) for i in (0, 1)])
            ego_hd = np.interp(distances, arc, headings)
            ego_predictions.append((braking, distances, ego_xy, ego_hd))
        brake_objects = set()
        for obj, xy, hd, length, width, hypothesis, same_flow in predictions:
            # Time margin prevents entering immediately before/after a circulating car.
            time_window = time_offset <= CROSSING_TIME_MARGIN_S
            # Branches sharing the current lane are indistinguishable far ahead.
            # Retain all near-term branches; use the motion-consistent branch
            # for the longer merge horizon, re-evaluating at every update.
            if hypothesis:
                time_window &= times[:, None] <= 3.0
            motion_hd = prediction_motion_headings(obj, xy, hd)
            identity = int(obj[11]) if len(obj) > 11 else id(obj)
            committed_lead = bool(self.committed_entry and lidar_vehicle_velocity(obj) is not None
                                  and identity in self.followable_ids)
            for braking, distances, ego_xy, ego_hd in ego_predictions:
                relative_heading = np.abs((ego_hd[:, None] - motion_hd[None, :] + math.pi) % (2 * math.pi) - math.pi)
                # Following vehicles occupy the same road at different times.
                # Comparing our current footprint with their later footprint
                # falsely vetoes a completed merge. Check simultaneous geometry
                # for aligned travel, retaining the time buffer for crossings.
                aligned = relative_heading < math.radians(20)
                # Circulating cars on the same route occupy the same points at
                # different times. Keep 0.2 s timing slack for them; the full
                # 1.2 s crossing margin invents collisions between successive
                # cars. rollout() still enforces headway behind a lead car.
                allowed = (time_window & (~aligned | (time_offset < .1)) &
                           (~same_flow | (time_offset <= .21)))
                if committed_lead:
                    # Headway controls a measured leader; comparing different
                    # times or a broad 40 cm shell causes repeated stop pulses.
                    allowed &= time_offset < .1
                hit = overlaps(ego_xy, ego_hd, xy, hd, length, width,
                               margin=.1 if committed_lead else .4, pair_mask=allowed)
                hit[distances > arc[-1], :] = False
                if np.any(hit):
                    row = int(np.flatnonzero(hit.any(axis=1))[0])
                    if braking:
                        brake_times.append(float(times[row]))
                        brake_objects.add(int(obj[11]) if len(obj) > 11 else id(obj))
                        continue
                    risks.append((float(times[row]), float(distances[row]), obj[3]))
                    col = int(np.flatnonzero(hit[row])[0])
                    # Where the car is in our frame at the conflict: ahead of the front
                    # bumper (it crosses in front: stopping avoids it) or beside/behind
                    # (it runs into our side: standing still does not).
                    rel = xy[col] - ego_xy[row]
                    ahead_x = float(rel[0] * math.cos(ego_hd[row]) + rel[1] * math.sin(ego_hd[row]))
                    self.risk_details.append(dict(object_id=obj[11] if len(obj) > 11 else None,
                                                  hypothesis=hypothesis, when=float(times[row]),
                                                  object_time=float(times[col]),
                                                  ego_xy=ego_xy[row].tolist(), object_xy=xy[col].tolist(),
                                                  ahead=bool(ahead_x >= 2.3175 - .3), name=obj[3],
                                                  along=float(distances[row])))
        # Only the first connected conflict zone defines this merge. Later
        # intersections remain prediction risks, not an ever-growing merge exit.
        crossings.sort()
        merge_end = crossings[0][1] if crossings else 5.0
        for begin, end, _ in crossings[1:]:
            if begin > merge_end + 1.0:
                break
            merge_end = max(merge_end, end)
        near_crossing = bool(crossings and crossings[0][0] <= max(8., speed * .7 + speed ** 2 / 5 + 3.))
        if risks and brake_times:
            first_risk = min(r[0] for r in risks)
            if first_risk > .5 and min(brake_times) <= first_risk:
                # Braking cannot avoid this car any more: we are already in its
                # path and stopping only parks us there (virtual junction run
                # 2026-10-03: stopped inside the merge, hit from behind-left).
                # Clearing the conflict leaves it more time. Recheck each update.
                return answer("enter", "gap", self.merge_exit, first_risk, target,
                              "clear conflict; braking trajectory conflicts sooner")
        if risks:
            when, along, name = min(risks)
            self.clear_since = None
            # A collision forecast several seconds and tens of metres away is
            # a reason to approach with braking room, not to remain parked at
            # the entry. Re-evaluate every frame as the lead car moves.
            braking_room = speed * .7 + speed * speed / (2 * 2.5) + 4.0
            if (self.merge_path is not None and merge_progress >= 2.0 and
                    when > 1.5 and along > speed * .35 + speed * speed / 5.0 + 3.0):
                # Once the car has entered, approach a forecast conflict with
                # braking room instead of sending a full stop in the junction.
                # Immediate overlaps still take the STOP branch below.
                cap = max(0., math.sqrt(5.0 * max(0., along - 4.0)) - 1.75)
                self.state = "ENTER"
                return answer("follow", name, along, when, min(target, cap),
                              "committed merge with braking reserve")
            if when <= 3.0 or along <= braking_room:
                self.state = "WAIT"
                return answer("stop", name, along, when, 0, "predicted footprint time overlap")
            cap = max(0., math.sqrt(2 * 2.5 * max(0., along - 4)) - 2.5 * .7)
            return answer("follow", name, along, when, min(target, cap), "approach conflict with braking reserve")
        if guard_approach is not None:
            begin, when, name, cap = guard_approach
            self.state, self.clear_since = "WAIT", None
            return answer("follow", name, begin, when, cap,
                          "approach fast traffic with braking reserve")
        if self.state == "ENTER":
            if leaders:
                cap, name, along = min(leaders)
                if cap <= 0.:
                    return answer("stop", name, along, 0, 0., "stationary lead vehicle blocks merge")
                return answer("follow", name, along, 0, min(target, cap), "merge lead vehicle headway")
            if merge_leaders:
                cap, name, along = min(merge_leaders)
                return answer("follow", name, along, 0, min(target, cap), "follow merging lead with checked headway")
            return answer("enter", "gap", self.merge_exit, 0, target, "continue checked merge")
        if near_crossing or self.state == "WAIT":
            # Accept the first safe observation; timing and acceleration margins
            # already cover uncertainty without an additional fixed dwell.
            exit_s = min(arc[-1], merge_end + 3)
            if min(d[-1] for d in motion_envelope) < exit_s and not merge_leaders:
                self.state = "WAIT"
                return answer("stop", "gap", exit_s, 8, 0, "cannot clear merge within prediction horizon")
            self.state = "ENTER"
            self.merge_path, self.merge_exit = (world.copy(), arc.copy()), exit_s
            if merge_leaders:
                cap, name, along = min(merge_leaders)
                return answer("follow", name, along, 0, min(target, cap), "follow merging lead with checked headway")
            return answer("enter", "gap", exit_s, 0, target, "gap accepted with acceleration and clearance")
        self.clear_since = None
        self.status = dict(state="CRUISE", reason="no crossing conflict", target_mps=target)
        if leaders:
            cap, name, along = min(leaders)
            return answer("follow", name, along, 0, min(target, cap), "lead vehicle headway")
        return None
