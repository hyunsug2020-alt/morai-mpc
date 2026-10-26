"""MOBIL utility and follower-braking admission for highway lane candidates.

The acceleration law uses the highway controller's following reference. This
is an additional decision model, not a replacement for timed footprint checks
or the measured longitudinal controller. All inputs use simulator units.
"""
import math

import numpy as np
from scipy.spatial import cKDTree

from ego_body import EGO_CENTRE_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M
from highway_planner import local_points
from highway_speed_plan import (STEP_S, HEADWAY_S, MIN_GAP_M, GAP_GAIN,
                               FOLLOW_RESPONSE_S, HIGHWAY_ACCEL_MPS2)
from planning.highway.object_policy import (motion_inferred, prediction_alignment_valid,
                                            PREDICTION_ALIGNMENT_INDEX, PREDICTION_ALIGNMENT_KEY)
from planning.highway.trajectory_geometry import normalize_path, projected_halves, sample_pose
from .lane_map import arc


def following_acceleration(speed, cruise, leader=None, gap=None, decel=2.5, age=0.):
    """Requested acceleration, before braking saturation, for MOBIL only.

    Saturating negative acceleration at b_safe would conceal an unsafe merge.
    Unknown speeds or overlapping bodies do not provide a usable utility.
    """
    values = [speed, cruise, decel, age]
    if leader is not None:
        values += [leader, gap]
    if any(v is None or not math.isfinite(v) for v in values) or decel <= 0.:
        return None
    target = max(0., cruise)
    if leader is not None:
        if gap <= 0.:
            return None
        error = gap-max(MIN_GAP_M, speed*HEADWAY_S)
        response = decel*(FOLLOW_RESPONSE_S+max(0., age))
        available = max(0., error)
        closing = (2.*decel*available /
                   (math.sqrt(response*response+2.*decel*available)+response))
        target = min(target, max(0., leader+min(GAP_GAIN*error, closing)))
    return min(HIGHWAY_ACCEL_MPS2, (target-max(0., speed))/.5)


def incentive(ego_before, ego_after, old_before, old_after, new_before, new_after,
              politeness):
    return ego_after-ego_before+politeness*(old_after-old_before+new_after-new_before)


class MobilEvaluator:
    def __init__(self, ego, objects, lanes, motion_lanes, model, config):
        self.ego, self.model = ego, model
        self.politeness = float(config.get('mobil_politeness', .1))
        self.threshold = float(config.get('mobil_threshold_mps2', .2))
        self.safe_decel = float(config.get('mobil_safe_decel_mps2', 2.5))
        if (not all(math.isfinite(v) for v in
                    (self.politeness, self.threshold, self.safe_decel)) or
                not 0. <= self.politeness <= .5 or not 0. <= self.threshold < HIGHWAY_ACCEL_MPS2 or
                not 0. < self.safe_decel <= model.comfort_decel_mps2):
            raise ValueError('Invalid highway MOBIL parameters')
        self.objects = []
        identities = set()
        # Duplicates are removed only from utility accounting. The original
        # bodies remain in every existing geometric and command safety check.
        vehicles = [o for o in objects if len(o) >= 15 and o[4] == 'npc']
        for obj in sorted(vehicles, key=lambda o: (not self.measured(o), o[3] != 'lidar_car')):
            if len(obj) < 15 or obj[4] != 'npc':
                continue
            key = repr(obj[11])
            if key in identities:
                continue
            duplicate = any(obj[3] != other[3] and
                np.linalg.norm(np.asarray(obj[:2])-other[:2]) <= 1. and
                np.linalg.norm(np.asarray(obj[9:11])-other[9:11]) <= 2.5
                for other in self.objects)
            if not duplicate:
                self.objects.append(obj)
                identities.add(key)
        self.geometry = {}
        for lane in lanes:
            motion = next((m for m in motion_lanes if m.link_id == lane.link_id), lane)
            path = np.asarray(motion.centre, dtype=float)
            tangent = np.gradient(path, axis=0)
            tangent /= np.maximum(1e-9, np.linalg.norm(tangent, axis=1, keepdims=True))
            self.geometry[lane.link_id] = path, arc(path), tangent, cKDTree(path)
        self.future, self.snapshots = {}, {}
        self.yaw = math.radians(float(ego['heading_deg']))
        self.rotation = np.array([[math.cos(self.yaw), -math.sin(self.yaw)],
                                  [math.sin(self.yaw), math.cos(self.yaw)]])

    @staticmethod
    def observation_issue(obj):
        """Separate an estimated leader from invalid or unobserved motion.

        Estimates may use the existing forecast-progress admission, but never
        provide measured follower acceleration or acquire scan-mask ownership.
        Explicit alignment lineage must validate even for a young estimate.
        """
        try:
            age = float(obj[15]) if len(obj) > 15 else 0.
            if not math.isfinite(age) or age < 0.:
                return 'invalid_age'
            aligned = (len(obj) > PREDICTION_ALIGNMENT_INDEX and
                       isinstance(obj[PREDICTION_ALIGNMENT_INDEX], dict) and
                       PREDICTION_ALIGNMENT_KEY in obj[PREDICTION_ALIGNMENT_INDEX])
            if aligned:
                if not prediction_alignment_valid(obj, age):
                    return 'invalid_alignment'
            elif (len(obj) > PREDICTION_ALIGNMENT_INDEX and
                    obj[PREDICTION_ALIGNMENT_INDEX] is not None and
                    not isinstance(obj[PREDICTION_ALIGNMENT_INDEX], dict)):
                return 'invalid_alignment'
            elif age > 1.:
                return 'invalid_age'
            if (not np.isfinite([obj[k] for k in (0, 1, 2, 5, 6, 7, 8, 9, 10)]).all()
                    or min(obj[6:8]) <= 0. or obj[5] < 0.):
                return 'invalid_geometry'
            if obj[14] != True:
                return 'unobserved_motion'
            return 'estimated_leader' if motion_inferred(obj) else None
        except (TypeError, ValueError, OverflowError, IndexError):
            return 'invalid_geometry'

    @staticmethod
    def measured(obj):
        return MobilEvaluator.observation_issue(obj) is None

    def project_many(self, lane, points):
        path, stations, tangent, tree = self.geometry[lane.link_id]
        _, at = tree.query(points)
        delta = points-path[at]
        along = np.sum(delta*tangent[at], axis=-1)
        lateral = delta[..., 1]*tangent[at, 0]-delta[..., 0]*tangent[at, 1]
        return stations[at]+along, lateral, tangent[at]

    def project(self, lane, point):
        s, lateral, tangent = self.project_many(lane, np.asarray(point))
        return float(s), float(lateral), tangent

    def prepare_future(self, times):
        missing = sorted({round(float(t), 8) for t in times}-self.future.keys())
        if not missing:
            return
        times = np.asarray(missing)
        lower, upper = np.maximum(0., times-.01), times+.01
        samples = np.r_[times, lower, upper]
        count = len(times)
        forecasts = []
        for obj in self.objects:
            points = local_points(self.model.object_positions(obj, samples), self.ego)
            velocity = (points[2*count:]-points[count:2*count])/(upper-lower)[:, None]
            forecasts.append((points[:count], self.model.object_yaws(obj, times), velocity))
        for i, t in enumerate(missing):
            self.future[t] = [(p[i], y[i], v[i]) for p, y, v in forecasts]

    def prepare_snapshots(self, lane, times):
        missing = sorted({round(float(t), 8) for t in times}
                         -{t for link, t in self.snapshots if link == lane.link_id})
        if not missing:
            return
        self.prepare_future(missing)
        if not self.objects:
            for t in missing:
                self.snapshots[lane.link_id, t] = []
            return
        # One KD query per lane/time batch, rather than one per object/sample.
        points = np.asarray([[p for p, _, _ in self.future[t]] for t in missing])
        shape = points.shape[:2]
        stations, lateral, tangent = self.project_many(lane, points.reshape(-1, 2))
        stations, lateral = stations.reshape(shape), lateral.reshape(shape)
        tangent = tangent.reshape(*shape, 2)
        yaws = np.asarray([[y for _, y, _ in self.future[t]] for t in missing])
        velocities = np.asarray([[v for _, _, v in self.future[t]] for t in missing])
        forward = np.sum(velocities*tangent, axis=-1)
        aligned = (np.isfinite(velocities).all(axis=-1) & (forward >= 0.) &
                   (forward >= np.linalg.norm(velocities, axis=-1)*math.cos(math.radians(30.))))
        angles = yaws-self.yaw-np.arctan2(tangent[..., 1], tangent[..., 0])
        halves, widths = projected_halves(np.asarray([o[6] for o in self.objects]),
                                          np.asarray([o[7] for o in self.objects]), angles)
        occupied = np.abs(lateral) <= EGO_HALF_WIDTH_M+widths+.3
        issues = [self.observation_issue(obj) for obj in self.objects]
        for i, t in enumerate(missing):
            states = []
            for j, obj in enumerate(self.objects):
                if occupied[i, j]:
                    states.append(dict(uid=repr(obj[11]), s=float(stations[i, j]),
                        half=float(halves[i, j]), speed=float(forward[i, j])
                        if issues[j] is None and aligned[i, j] else None,
                        motion_issue=(issues[j] if issues[j] not in (None, 'estimated_leader')
                                      or aligned[i, j] else 'unaligned_motion'),
                        age=float(obj[15]) if len(obj) > 15 else 0.))
            self.snapshots[lane.link_id, t] = states

    def snapshot(self, lane, t):
        key = lane.link_id, round(float(t), 8)
        if key not in self.snapshots:
            self.prepare_snapshots(lane, [t])
        return self.snapshots[key]

    @staticmethod
    def neighbors(states, station, omit=None):
        remaining = [s for s in states if s['uid'] != omit]
        front = min((s for s in remaining if s['s'] >= station),
                    key=lambda s: s['s'], default=None)
        rear = max((s for s in remaining if s['s'] < station),
                   key=lambda s: s['s'], default=None)
        return front, rear

    def acceleration(self, vehicle, leader, cruise):
        if vehicle['speed'] is None or leader is not None and leader['speed'] is None:
            return None
        gap = (leader['s']-vehicle['s']-vehicle['half']-leader['half']
               if leader is not None else None)
        return following_acceleration(vehicle['speed'], cruise,
            leader['speed'] if leader is not None else None, gap,
            self.model.comfort_decel_mps2, max(0., leader.get('age', 0.)) if leader is not None else 0.)

    @staticmethod
    def sample_ego(result, times):
        trajectory = result['trajectory']
        path, stations = normalize_path(result['path'])
        distance = np.interp(times, trajectory.times, trajectory.distances)
        points, yaws = sample_pose(path, stations, distance, 0.)
        directions = np.column_stack((np.cos(yaws), np.sin(yaws)))
        return (points+EGO_CENTRE_M*directions,
                np.interp(times, trajectory.times, trajectory.speeds), directions)

    def ego_states(self, lane, points, speeds, directions):
        stations, lateral, tangent = self.project_many(lane, points)
        angle = (np.arctan2(directions[:, 1], directions[:, 0])-
                 np.arctan2(tangent[:, 1], tangent[:, 0]))
        halves, widths = projected_halves(2.*EGO_HALF_LENGTH_M, 2.*EGO_HALF_WIDTH_M, angle)
        occupied = np.abs(lateral) <= lane.half_width_m+widths
        forward = speeds*np.sum(directions*tangent, axis=-1)
        states = [dict(s=float(s), speed=float(v), half=float(h), age=0.)
                  for s, v, h in zip(stations, forward, halves)]
        return states, occupied

    def inserted_acceleration(self, rear, leader, ego, cruise, occupied):
        if occupied and ego['s'] >= rear['s'] and (leader is None or ego['s'] <= leader['s']):
            leader = ego
        return self.acceleration(rear, leader, cruise)

    @staticmethod
    def equivalent_acceleration(trajectory, horizon):
        # 2/H² integral_0^H (H-t)*a(t) dt. Unlike terminal delta-v,
        # this retains an advantage from reaching the same cruise speed sooner.
        distance = float(np.interp(horizon, trajectory.times, trajectory.distances))
        return 2.*(distance-trajectory.distances[0]-trajectory.speeds[0]*horizon)/horizon**2

    def assess(self, current, sequence, result, baseline, length):
        """Horizon-adapted MOBIL with the production rear-axle pose contract.

        The ego and follower acceleration balances share a linearly decreasing
        time weight. Every entered lane has a separate, unweighted b_safe veto.
        Forecasts never replace the existing raw/map/OBB/command safety checks.
        An incomplete forecast still checks its known prefix before returning
        unavailable utility; a known unsafe prefix cannot become an escape.
        """
        out = dict(available=False, safe=None, incentive_mps2=None,
                   threshold_mps2=self.threshold, politeness=self.politeness,
                   safe_decel_mps2=self.safe_decel, follower_checks=[],
                   forecast_progress_fallback=False, unavailable_reasons=[],
                   unavailable_evidence=[])
        evidence = set()

        def unavailable(reason, uid=None, role=None, lane=None):
            if reason not in out['unavailable_reasons']:
                out['unavailable_reasons'].append(reason)
            key = reason, uid, role, lane
            if key not in evidence:
                evidence.add(key)
                out['unavailable_evidence'].append(dict(reason=reason, object_id=uid,
                                                       role=role, lane=lane))
        trajectory = result['trajectory']
        finish_distance = len(sequence)*length+(len(sequence)-1)*max(3., trajectory.speeds[0]*.5)
        _, path_stations = normalize_path(result['path'])
        complete = finish_distance <= min(trajectory.distances[-1], path_stations[-1])
        horizon = (float(np.interp(finish_distance, trajectory.distances, trajectory.times))
                   if complete else float(trajectory.times[-1]))
        horizon = max(STEP_S, horizon)
        # Safety covers the entire available nominal prefix, including a
        # saved suffix whose first leg finished before the original waypoint.
        # The utility horizon alone uses the candidate's construction length.
        prefix_end = float(np.interp(min(path_stations[-1], trajectory.distances[-1]),
                                     trajectory.distances, trajectory.times))
        times = np.unique(np.r_[trajectory.times[trajectory.times < prefix_end], prefix_end, min(horizon, prefix_end)])
        out['safety_prefix_s'] = prefix_end
        points, speeds, directions = self.sample_ego(result, times)
        changed = {}
        rear_unknown = False
        for lane in {l.link_id: l for l in [current]+list(sequence)}.values():
            changed[lane.link_id] = self.ego_states(lane, points, speeds, directions)
            self.prepare_snapshots(lane, times)
        for destination in sequence:
            ego_states, occupied = changed[destination.link_id]
            contact = np.flatnonzero(occupied)
            if not len(contact):
                rear_unknown = True
                unavailable('target_occupancy', role='target', lane=destination.link_id)
                continue
            first = float(times[contact[0]])
            if contact[0] > 0:
                contact = np.r_[contact[0]-1, contact]
            worst, rear_id = 0., None
            for i in contact:
                state = ego_states[i]
                _, rear = self.neighbors(self.snapshot(destination, times[i]), state['s'])
                if rear is None or state['s']-rear['s'] > 100.:
                    continue
                demand = self.acceleration(rear, state, result['ceiling_mps'])
                if demand is None:
                    rear_unknown = True
                    unavailable('target_follower', rear['uid'], 'target_rear', destination.link_id)
                elif demand < worst:
                    worst, rear_id = demand, rear['uid']
            out['follower_checks'].append(dict(lane=destination.link_id,
                entry_s=round(first, 3), rear_id=rear_id, acceleration_mps2=worst))
            if worst < -self.safe_decel:
                unavailable('unsafe_follower', rear_id, 'target_rear', destination.link_id)
                out.update(safe=False, reason='MOBIL target follower braking exceeds limit')
                return out
        out['safe'] = None if rear_unknown else True
        if not complete:
            unavailable('completion_horizon')
            out['reason'] = 'MOBIL lane completion outside prediction'
            return out
        _, stay_stations = normalize_path(baseline['path'])
        stay_trajectory = baseline['trajectory']
        if (horizon > stay_trajectory.times[-1] or
                np.interp(horizon, stay_trajectory.times, stay_trajectory.distances) > stay_stations[-1]):
            unavailable('stay_horizon')
            out['reason'] = 'MOBIL stay prediction unavailable'
            return out
        times = times[times <= horizon]
        points, speeds, directions = self.sample_ego(result, times)
        changed = {l.link_id: self.ego_states(l, points, speeds, directions)
                   for l in [current]+list(sequence)}
        old_points, old_speeds, old_directions = self.sample_ego(baseline, times)
        old_deltas, new_deltas = np.zeros(len(times)), np.zeros(len(times))
        cruise = max(baseline['ceiling_mps'], result['ceiling_mps'])
        # Evaluate the release of the old follower and the demand on each new
        # follower over the actual occupancy interval, including intermediate
        # lanes. A UID seen in overlapping native lanes is charged only once.
        lanes = list({l.link_id: l for l in [current]+list(sequence)}.values())
        stayed = {l.link_id: self.ego_states(l, old_points, old_speeds, old_directions) for l in lanes}
        for i, t in enumerate(times):
            demands = {}
            for lane in lanes:
                states = self.snapshot(lane, t)
                old_state, old_occupied = stayed[lane.link_id][0][i], stayed[lane.link_id][1][i]
                new_state, new_occupied = changed[lane.link_id][0][i], changed[lane.link_id][1][i]
                # A leader with unknown motion also makes a discretionary
                # utility comparison unavailable, independently of its SAT veto.
                for state, occupied, role in ((old_state, old_occupied, 'stay_leader'),
                                             (new_state, new_occupied, 'change_leader')):
                    if occupied:
                        leader, _ = self.neighbors(states, state['s'])
                        if leader is not None and leader['speed'] is None:
                            unavailable(leader.get('motion_issue') or 'unobserved_motion',
                                        leader['uid'], role, lane.link_id)
                rears = {}
                for state, occupied in ((old_state, old_occupied), (new_state, new_occupied)):
                    if occupied:
                        _, rear = self.neighbors(states, state['s'])
                        if rear is not None and state['s']-rear['s'] <= 100.:
                            rears[rear['uid']] = rear
                for uid, rear in rears.items():
                    lead, _ = self.neighbors(states, rear['s'], uid)
                    before = self.inserted_acceleration(rear, lead, old_state, cruise, old_occupied)
                    after = self.inserted_acceleration(rear, lead, new_state, cruise, new_occupied)
                    if before is None or after is None:
                        unavailable('follower_balance', uid, 'old_rear' if
                                    lane.link_id == current.link_id else 'new_rear', lane.link_id)
                        continue
                    if uid in demands:
                        prior = demands[uid]
                        before, after = min(before, prior[0]), min(after, prior[1])
                        old = prior[2] or lane.link_id == current.link_id
                    else:
                        old = lane.link_id == current.link_id
                    demands[uid] = before, after, old
            for before, after, old in demands.values():
                (old_deltas if old else new_deltas)[i] += after-before
        if out['unavailable_reasons']:
            out['forecast_progress_fallback'] = (out['safe'] is True and
                set(out['unavailable_reasons']) == {'estimated_leader'})
            out['reason'] = 'MOBIL motion or follower gap unavailable'
            return out
        weight = horizon-times
        old_delta = 2.*float(np.trapz(weight*old_deltas, times))/horizon**2
        new_delta = 2.*float(np.trapz(weight*new_deltas, times))/horizon**2
        before = self.equivalent_acceleration(stay_trajectory, horizon)
        after = self.equivalent_acceleration(trajectory, horizon)
        value = incentive(before, after, 0., old_delta, 0., new_delta, self.politeness)
        out.update(available=True, safe=True, incentive_mps2=value,
            ego_acceleration_before_mps2=before, ego_acceleration_after_mps2=after,
            old_follower_delta_mps2=old_delta, new_follower_delta_mps2=new_delta,
            utility_time_s=horizon,
            reason='MOBIL incentive satisfied' if value > self.threshold else 'MOBIL incentive below threshold')
        return out
