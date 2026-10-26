"""Surveyed highway lane geometry and shared path interpolation helpers."""
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.spatial import cKDTree

from highway_planner import local_points, world_points
from planning.highway.body_geometry import HighwayLaneCorridor
from planning.highway.merge_geometry import (gentle_merge_geometry, continue_merge_geometry,
                                            recover_merge_tracking)


# Surveyed successor endpoints coincide except for one 3.36 cm map seam.
# A larger gap or a turn outside the existing 30-degree motion match is not
# evidence of an actor's route. Stop before it rather than inventing a join.
OBSERVED_CHAIN_JOIN_M = .05
OBSERVED_CHAIN_TANGENT_COS = math.cos(math.radians(30.))


def arc(points):
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]


def smooth(value):
    u = np.clip(value, 0., 1.)
    return u**3 * (10. - 15.*u + 6.*u*u)


@dataclass
class HighwayLane:
    link_id: str
    rank: int
    centre: np.ndarray
    chain: tuple
    merge_remaining_m: float
    half_width_m: float = 1.75
    surveyed_centre: np.ndarray = None
    # Heading change where this lane's native chain joins the common entrance
    # (0 for the lane that runs straight into it). A large kink means the lane
    # must merge into the entrance's traffic at an angle.
    entry_kink_deg: float = 0.
    # Native HD-map numbering is separate from this frame's candidate rank.
    native_lane_number: int = None


def direction_over(points, length_m, from_end):
    """Mean direction of the first (or last) length_m of a polyline, as a unit vector."""
    points = np.asarray(points, dtype=float)[:, :2]
    if from_end:
        points = points[::-1]
    stations = arc(points)
    k = int(np.searchsorted(stations, min(length_m, stations[-1])))
    k = min(max(k, 1), len(points)-1)
    vector = points[k]-points[0]
    vector = vector if not from_end else -vector
    return vector/max(1e-9, float(np.linalg.norm(vector)))


class HighwayLaneMap:
    """Enumerate permitted adjacent lanes; follow native successors to the exit.

    Candidate indices are local ordering, never persistent lane identities.
    A lane can merge early and become the route lane without a lane change.
    """
    def __init__(self, lanes, config):
        self.lanes = lanes
        self.exit_link = config["end_link"]
        self.excluded_links = set(config.get("excluded_lane_links", ()))
        self.chains = {}
        self.observed_chains = {}
        self.entry_kinks = {}
        self.control_chains = {}
        self.recent_candidates = None
        self.merge_continuation = None
        self.geometry_reused = False
        route_xy = getattr(lanes, "route_xy", np.empty((0, 2)))
        _, route_points = lanes.tree.query(route_xy)
        self.route_links = {lanes.links[int(k)]["idx"] for k in lanes.link_of[route_points]}
        # Reverse breadth-first reachability avoids caching false negatives
        # when the road graph contains a loop through an alternate exit.
        self.exit_distance = {self.exit_link: 0}
        pending = [self.exit_link]
        for link_id in pending:
            node = lanes.link_by_id[link_id].get("from_node_idx")
            for parent in lanes.predecessors.get(node, ()):
                if parent not in self.exit_distance:
                    self.exit_distance[parent] = self.exit_distance[link_id]+1
                    pending.append(parent)
        self.exit_continuations = {self.exit_link}
        for _ in range(2):
            self.exit_continuations.update(k for link_id in tuple(self.exit_continuations)
                for k in lanes.successors.get(lanes.link_by_id[link_id].get("to_node_idx"), ()))

    def reaches_exit(self, link_id):
        return link_id in self.exit_distance

    def entry_kink_deg(self, chain):
        """Heading change from the link before the common entrance into it."""
        chain = tuple(chain)
        if chain in self.entry_kinks:
            return self.entry_kinks[chain]
        kink = 0.
        if self.exit_link in chain and chain.index(self.exit_link) > 0:
            before = self.lanes.geometry.get(chain[chain.index(self.exit_link)-1])
            after = self.lanes.geometry.get(self.exit_link)
            if before is not None and after is not None and len(before) > 1 and len(after) > 1:
                a, b = direction_over(before, 10., True), direction_over(after, 10., False)
                kink = abs(math.degrees(math.atan2(a[0]*b[1]-a[1]*b[0], float(a @ b))))
        self.entry_kinks[chain] = kink
        return kink

    def blocked(self, link_id):
        if link_id in self.excluded_links:
            return True
        # The dashboard whitelist governs additional lanes. The global route
        # remains available, as it does in HdMapLanes.corridor.
        if self.lanes.rules().get("mode") == "whitelist" and link_id in self.route_links:
            return False
        return self.lanes.blocked(link_id)

    def chain(self, seed):
        if seed in self.chains:
            return self.chains[seed]
        chain, points, merge_at = [], [], None
        if seed in self.exit_continuations and seed != self.exit_link:
            merge_at = self.lanes.geometry[seed][0].copy()
        current = seed
        while current not in chain and current in self.lanes.geometry:
            link = self.lanes.link_by_id[current]
            if link.get("opp_traffic") or self.blocked(current):
                break
            geometry = self.lanes.geometry[current].astype(float)
            if current == self.exit_link:
                merge_at = geometry[0].copy()
            chain.append(current)
            points.append(geometry if not points else geometry[1:])
            successors = [k for k in self.lanes.successors.get(link.get("to_node_idx"), ())
                          if k in self.lanes.geometry and
                          (k in self.exit_continuations if merge_at is not None else self.reaches_exit(k))
                          and not self.lanes.link_by_id[k].get("opp_traffic")]
            if not successors:
                break
            # Preserve tangent continuity at branches leading to the same exit.
            tangent = geometry[-1] - geometry[max(0, len(geometry)-4)]
            tangent /= max(1e-6, np.linalg.norm(tangent))
            def alignment(k):
                g = self.lanes.geometry[k]
                v = g[min(3, len(g)-1)]-g[0]
                return float(v @ tangent / max(1e-6, np.linalg.norm(v)))
            current = min(successors, key=lambda k: (self.exit_distance.get(k, 10000), -alignment(k)))
            if len(chain) >= 20:
                break
        if not points or merge_at is None:
            return None
        geometry = np.vstack(points)
        # Round sub-metre link sampling corners inside the existing 25 cm
        # displacement bound; footprint checks still use the surveyed road.
        from gui import smooth_control_route
        endpoints = geometry[[0, -1]].copy()
        geometry = smooth_control_route(geometry)
        geometry[[0, -1]] = endpoints
        self.chains[seed] = (tuple(chain), geometry, merge_at)
        return self.chains[seed]

    def candidates(self, ego, route):
        self.geometry_reused = False
        world = world_points(np.asarray(route)[:, :2], ego)
        _, at = self.lanes.tree.query(world[0])
        root = self.lanes.links[int(self.lanes.link_of[at])]
        seeds = [root]
        while True:
            other = self.lanes.adjacent_link(seeds[-1], 1)
            if other is None or other.get("opp_traffic") or other["idx"] in {l["idx"] for l in seeds}:
                break
            seeds.append(other)
        result = []
        for rank, seed in enumerate(seeds):
            native = self.chain(seed["idx"])
            if native is None:
                continue
            chain, geometry, merge = native
            # Block rules can change at runtime; do not retain a cached open lane.
            if any(self.blocked(k) for k in chain):
                continue
            nearest = int(np.argmin(np.linalg.norm(geometry-world[0], axis=1)))
            surveyed = local_points(geometry[nearest:], ego)
            if seed['idx'] not in self.control_chains:
                self.control_chains[seed['idx']] = gentle_merge_geometry(geometry, merge)
            centre = local_points(self.control_chains[seed['idx']][nearest:], ego)
            if len(centre) < 4:
                continue
            tangent = centre[min(3, len(centre)-1)]-centre[0]
            if tangent[0] <= 0. or abs(math.atan2(tangent[1], tangent[0])) > math.radians(35.):
                continue
            stations = arc(geometry)
            merge_index = int(np.argmin(np.linalg.norm(geometry-merge, axis=1)))
            number = seed.get('ego_lane')
            number = (int(number) if isinstance(number, (int, np.integer))
                      and not isinstance(number, (bool, np.bool_)) and number > 0 else None)
            result.append(HighwayLane(seed["idx"], rank, centre, chain,
                max(0., float(stations[merge_index]-stations[nearest])),
                min(float(seed.get("width_start", 3.5)), float(seed.get("width_end", 3.5)))/2.,
                surveyed, self.entry_kink_deg(chain), number))
        stamp = float(ego.get('stamp',time.monotonic()))
        if result:
            previous = self.recent_candidates
            if previous is None or not 0. <= stamp-previous[0] <= .25:
                self.merge_continuation = None
            elif self.merge_continuation is None:
                observed = previous[1]
                lane, controlled, surveyed = min(observed, key=lambda item:
                    float(np.min(np.linalg.norm(item[1]-[ego['x'],ego['y']],axis=1))))
                if (lane.link_id not in {l.link_id for l in result} and
                        lane.merge_remaining_m <= 5. and
                        np.min(np.linalg.norm(controlled-[ego['x'],ego['y']],axis=1)) < 2.75):
                    stations = arc(surveyed)
                    merge = np.array([np.interp(lane.merge_remaining_m,stations,surveyed[:,k])
                                      for k in (0,1)])
                    if any(np.linalg.norm(world_points(l.surveyed_centre[:1],ego)[0]-merge)<5.
                           for l in result):
                        self.merge_continuation = surveyed,controlled,merge
            if self.merge_continuation is not None:
                surveyed,controlled,merge = self.merge_continuation
                at = int(np.argmin(np.linalg.norm(surveyed-[ego['x'],ego['y']],axis=1)))
                merge_at = int(np.argmin(np.linalg.norm(surveyed-merge,axis=1)))
                if (np.linalg.norm(surveyed[at]-[ego['x'],ego['y']]) > 2.75 or
                        arc(surveyed)[at]-arc(surveyed)[merge_at] >= 45.):
                    self.merge_continuation = None
                else:
                    for lane in result:
                        proposed = world_points(lane.centre,ego)
                        fresh_survey = world_points(lane.surveyed_centre,ego)
                        lane.centre = local_points(continue_merge_geometry(fresh_survey,proposed,
                            surveyed,controlled,merge),ego)
            # Keep the surveyed envelope fixed. At the common-entry seam a
            # small heading/tracking error must be repaired on the proposed
            # control curve, rather than temporarily closing or widening it.
            speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
            near_merge = [lane for lane in result
                          if lane.merge_remaining_m <= 45.]
            if near_merge:
                corridor = self.corridor(ego, route, result)
                for lane in near_merge:
                    lane.centre = recover_merge_tracking(lane.surveyed_centre, lane.centre,
                        speed, corridor.clear_distance, merge_remaining_m=lane.merge_remaining_m)
            self.recent_candidates = (stamp,[(l,world_points(l.centre,ego),
                world_points(l.surveyed_centre,ego)) for l in result])
        elif self.recent_candidates is not None:
            previous,observed = self.recent_candidates
            if 0. <= stamp-previous <= .25:
                retained = []
                distances = []
                for lane,world_centre,world_surveyed in observed:
                    if any(self.blocked(k) for k in lane.chain):
                        continue
                    at = int(np.argmin(np.linalg.norm(world_centre-[ego['x'],ego['y']],axis=1)))
                    centre = local_points(world_centre[at:],ego)
                    if len(centre)<4:
                        continue
                    tangent = centre[min(3,len(centre)-1)]-centre[0]
                    if tangent[0]<=0. or abs(math.atan2(tangent[1],tangent[0]))>math.radians(35.):
                        continue
                    distances.append(float(np.linalg.norm(centre[0])))
                    retained.append(HighwayLane(lane.link_id,lane.rank,centre,lane.chain,
                        max(0.,lane.merge_remaining_m-float(arc(world_centre)[at])),lane.half_width_m,
                        local_points(world_surveyed[at:],ego),lane.entry_kink_deg,
                        lane.native_lane_number))
                if retained and min(distances)<2.75:
                    # Reuse only recently surveyed geometry at a link boundary.
                    # Fresh pose, raw LiDAR and traffic checks still apply, and
                    # block rules and the short age bound cannot be bypassed.
                    result = retained
                    self.geometry_reused = True
        return result

    def corridor(self, ego, route, candidates):
        """Union of the selected same-direction native lane envelopes."""
        route = np.asarray(route, dtype=float)[:, :2]
        tangent = np.gradient(route, axis=0)
        tangent /= np.maximum(1e-6, np.linalg.norm(tangent, axis=1, keepdims=True))
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        left, right = np.full(len(route), -np.inf), np.full(len(route), np.inf)
        for lane in candidates:
            surveyed = lane.surveyed_centre if lane.surveyed_centre is not None else lane.centre
            _, indices = cKDTree(surveyed).query(route)
            nearby = surveyed[indices]
            delta = nearby-route
            along = np.abs(np.sum(delta*tangent, axis=1))
            lateral = np.sum(delta*normal, axis=1)
            valid = (along <= 2.5) & (np.linalg.norm(delta, axis=1) <= 11.5)
            left[valid] = np.maximum(left[valid], lateral[valid]+lane.half_width_m)
            right[valid] = np.minimum(right[valid], lateral[valid]-lane.half_width_m)
        # Unknown road geometry is closed, rather than implicitly widened.
        left[~np.isfinite(left)], right[~np.isfinite(right)] = 0., 0.
        return HighwayLaneCorridor(route, left, right, zones=self.lanes.local_zones(ego))

    def motion_candidates(self, ego, objects):
        """Retain each observed vehicle's native road, including behind the ego."""
        result, seen = [], set()
        for obj in objects:
            if obj[4] != 'npc':
                continue
            distance, at = self.lanes.tree.query(obj[:2])
            seed = self.lanes.links[int(self.lanes.link_of[at])]
            if seed['idx'] in seen or distance > 2.1:
                continue
            seen.add(seed['idx'])
            native = self.chain(seed['idx'])
            if native is not None:
                chain, geometry, _ = native
                result.append(HighwayLane(seed['idx'], -1, local_points(geometry, ego),
                                          chain, 0.))
                continue
            # An actor can occupy a surveyed lane that does not reach our exit
            # or is closed to the ego. Those rules still govern candidates and
            # the corridor; they must not erase the actor's measured road.
            observed = self.observed_motion_chain(seed['idx'])
            if observed is not None:
                chain, geometry = observed
                try:
                    widths = (float(seed.get('width_start', 3.5)),
                              float(seed.get('width_end', 3.5)))
                except (TypeError, ValueError, OverflowError):
                    continue
                if not all(math.isfinite(width) and width > 0. for width in widths):
                    continue
                result.append(HighwayLane(seed['idx'], -1, local_points(geometry, ego),
                                          chain, 0., min(widths)/2.))
        return result

    def observed_motion_chain(self, seed):
        """Survey an actor's unambiguous road without granting ego admission.

        Successful ego chains retain their existing geometry. This fallback
        uses raw surveyed points and unique, continuous native successors;
        route reachability, exclusions and dashboard lane permissions do not
        describe where another vehicle is travelling. Opposite traffic keeps
        the existing linear-forecast handling. Unknown branches stop the chain
        and retain the forecaster's existing endpoint extrapolation.
        """
        if seed in self.observed_chains:
            return self.observed_chains[seed]
        chain, points, previous = [], [], None
        current = seed
        while current not in chain and len(chain) < 20:
            link = self.lanes.link_by_id.get(current)
            raw = self.lanes.geometry.get(current)
            if link is None or raw is None or link.get('opp_traffic'):
                break
            geometry = np.asarray(raw, dtype=float)
            if (geometry.ndim != 2 or geometry.shape[1] < 2 or len(geometry) < 2 or
                    not np.isfinite(geometry[:, :2]).all()):
                break
            geometry = geometry[:, :2]
            geometry = geometry[np.r_[True,
                np.linalg.norm(np.diff(geometry, axis=0), axis=1) > 1e-6]]
            if len(geometry) < 2:
                break
            if previous is not None:
                if np.linalg.norm(previous[-1]-geometry[0]) > OBSERVED_CHAIN_JOIN_M:
                    break
                before = direction_over(previous, 3., True)
                after = direction_over(geometry, 3., False)
                if float(before @ after) < OBSERVED_CHAIN_TANGENT_COS:
                    break
            chain.append(current)
            points.append(geometry if previous is None else geometry[1:])
            previous = geometry
            successors = tuple(dict.fromkeys(self.lanes.successors.get(
                link.get('to_node_idx'), ())))
            if len(successors) != 1:
                break
            current = successors[0]
        result = (tuple(chain), np.vstack(points)) if points else None
        self.observed_chains[seed] = result
        return result
