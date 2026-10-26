"""Prepare the next road policy without publishing a second control stream."""
import copy
import math
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import numpy as np

from lane_corridor import LaneCorridor
from ego_body import ROAD_LEGACY_SHIFT_M


PREPARE_MIN_M = 25.
PREPARE_MAX_M = 60.
PREPARE_TIME_S = 3.
PROPOSAL_MAX_AGE_S = 1.5


def rotation(ego):
    angle = math.radians(float(ego['heading_deg']))
    return np.array([[math.cos(angle), -math.sin(angle)],
                     [math.sin(angle), math.cos(angle)]])


def approach_corridor(tunnel, lanes, ego, route, body_shift_m=ROAD_LEGACY_SHIFT_M):
    """Keep ordinary lane limits outside the portal and wall limits inside it.
    body_shift_m: the lane check's body placement (LaneCorridor; the tunnel planner
    passes ego_body.REAR_AXLE_SHIFT_M). The wall check is always the real body."""
    route = np.asarray(route, dtype=float)[:, :2]
    normal = lanes.corridor(ego, route, allow_second_same_direction=True)
    if normal is None:
        return None
    world = route @ rotation(ego).T + [ego['x'], ego['y']]
    along = (world-tunnel.origin) @ tunnel.axis
    lateral = (world-tunnel.origin) @ tunnel.normal
    inward = normal.normal @ (rotation(ego).T @ tunnel.normal)
    inside = (along >= tunnel.start) & (along <= tunnel.end) & (inward > .82)
    left, right = normal.left.copy(), normal.right.copy()
    left[inside] = (tunnel.inner_left-lateral[inside])/inward[inside]
    right[inside] = (tunnel.inner_right-lateral[inside])/inward[inside]
    corridor = LaneCorridor(route, left, right, zones=normal.zones, body_shift_m=body_shift_m)
    lane_clear = corridor.clear_distance
    corridor.clear_distance = lambda path, horizon=60.: min(
        lane_clear(path, horizon), tunnel.path_clearance(path, ego, horizon))
    corridor.wall_only = False
    return corridor


def seed_from_command(command, now):
    if not command or command.get('gear', 'D') != 'D' or command.get('emergency'):
        return None
    if command.get('allowed_mps', command.get('target_mps', 0.)) <= 0.:
        return None
    if not 0. <= now-command['received'] <= PROPOSAL_MAX_AGE_S:
        return None
    path = np.asarray(command['world_path'], dtype=float)
    if len(path) < 3 or not np.isfinite(path).all():
        return None
    return dict(stamp=now, target=float(command.get('offset_m', 0.)), world_path=path.copy(),
                allowed_mps=float(command.get('allowed_mps', command.get('target_mps', 0.))),
                sensor_stamp=command.get('sensor_stamp'), plan_id=command.get('plan_id'),
                validation=copy.deepcopy(command.get('validation') or {}),
                decision=copy.deepcopy(command.get('decision')))


class PlannerTransition:
    """One active writer, one bounded background preparation job.

    A prepared path is only a proposal. The receiving planner checks it again
    against its latest scan before it can publish. Speed policy and emergency
    braking continue to belong to the receiving road section.
    """
    def __init__(self, executor=None):
        self.executor = executor
        self.future = None
        self.future_target = None
        self.prepared = None
        self.last_command = None
        self.owner = None
        self.last_stamp = None
        self.generation = 0
        self.last_submit = -math.inf
        self.applied_proposal = None
        self.tunnel_route = None
        self.tunnel_entries = None
        self.tunnel_stations = None
        self.info = dict(state='IDLE', target=None)

    def _entry_distance(self, runtime, tunnel, index, ego):
        """Use the upcoming route entry, not a distant crossing of its axis."""
        delta = np.array([ego['x'], ego['y']])-tunnel.origin
        along, lateral = float(delta @ tunnel.axis), float(delta @ tunnel.normal)
        portal_delta = np.array([tunnel.start-along,
            lateral-np.clip(lateral, tunnel.inner_right, tunnel.inner_left)])
        if np.linalg.norm(portal_delta) > PREPARE_MAX_M+3.:
            return None
        route = getattr(runtime, 'waypoint_xy', None)
        if route is not None and len(route) >= 3:
            if route is not self.tunnel_route:
                world = np.asarray(route, dtype=float)[:, :2]
                offset = world-tunnel.origin
                stations = offset @ tunnel.axis
                sides = offset @ tunnel.normal
                inside = ((stations >= tunnel.start) & (stations <= tunnel.end) &
                          (sides >= tunnel.right-2.) & (sides <= tunnel.left+2.))
                entries = np.flatnonzero(inside & ~np.r_[False, inside[:-1]])
                tangent = np.gradient(world, axis=0)
                forward = tangent[entries] @ tunnel.axis > .2*np.linalg.norm(tangent[entries], axis=1)
                self.tunnel_entries = entries[forward]
                self.tunnel_stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(world, axis=0), axis=1))]
                self.tunnel_route = route
            upcoming = self.tunnel_entries[self.tunnel_entries >= index]
            if not len(upcoming) or not 0 <= index < len(route):
                return None
            return float(self.tunnel_stations[upcoming[0]]-self.tunnel_stations[index])
        # Minimal runtimes without a route still require a nearby, forward entry.
        if (not tunnel.right-10. <= lateral <= tunnel.left+10. or
                rotation(ego)[:, 0] @ tunnel.axis <= .2):
            return None
        return tunnel.start-along

    def _poll(self, now):
        if self.future is None or not self.future.done():
            return
        try:
            candidate = self.future.result()
            if candidate is not None and candidate['generation'] == self.generation:
                self.prepared = candidate
                self.info = dict(state='READY' if candidate['ready'] else 'PREPARING',
                                 target=candidate['target'], reason=candidate['reason'])
        except Exception as exc:
            self.info = dict(state='PREPARING', target=self.future_target,
                             reason='preview failed: '+str(exc))
        self.future = None

    def before(self, runtime, status):
        runtime._transition_entry_obstacles = False
        runtime._transition_entry_corridor = False
        ego = status.get('ego')
        if not ego or float(ego.get('age_s', 999.)) > .25:
            return None
        now = time.monotonic()
        stamp = float(ego.get('stamp', 0.))
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.generation += 1
            self.prepared = self.last_command = self.owner = None
            for name in ('_transition_seed', '_transition_alternative', '_tunnel_last_good',
                         '_tunnel_lattice_prev', '_tunnel_decision_plan', '_tunnel_lead_tracker', '_tunnel_lead_world'):
                if hasattr(runtime, name):
                    delattr(runtime, name)
        self.last_stamp = stamp
        self._poll(now)
        index = int((status.get('path') or {}).get('nearest_idx', 0))
        highway = getattr(runtime, 'highway', None)
        tunnel = getattr(runtime, 'tunnel', None)
        in_highway = bool(highway and highway.active(index))
        in_tunnel = bool(tunnel and tunnel.active(ego))
        distance = None
        if tunnel and all(hasattr(tunnel, key) for key in ('origin', 'axis', 'start', 'end')):
            along = float((np.array([ego['x'], ego['y']])-tunnel.origin) @ tunnel.axis)
            distance = tunnel.start-along
            if self.owner == 'TUNNEL' and (distance > 6. or along > tunnel.end+.5):
                self.owner = None
        if in_highway:
            self.owner = None
            target = 'GENERAL'
            end = int(highway.config['to_idx'])
            stations = getattr(runtime, 'route_cumulative', None)
            distance = float(stations[end]-stations[index]) if stations is not None else (end-index)*.5
        else:
            target = 'TUNNEL'
            if distance is not None:
                distance = self._entry_distance(runtime, tunnel, index, ego)
        approaching = bool(not in_highway and not in_tunnel and distance is not None
                           and 0. <= distance <= PREPARE_MAX_M)
        # Latest measured obstacles belong to the active approach planner,
        # even while the next-policy proposal is pending, blocked or stale.
        runtime._transition_entry_obstacles = approaching
        runtime._transition_entry_corridor = approaching
        if distance is None or not 0. <= distance <= PREPARE_MAX_M:
            self.info = dict(state='IDLE', target=None)
            # Passing the route entry removes it from the upcoming-entry search.
            # Keep its proposal until the tunnel branch can revalidate the handoff.
            if (self.prepared and self.prepared['target'] == 'TUNNEL'
                    and not in_tunnel and self.owner != 'TUNNEL'):
                self.prepared = None
        prepared = self.prepared
        usable = bool(prepared and prepared['target'] == target and prepared['ready'] and
                      0. <= now-prepared['received'] <= PROPOSAL_MAX_AGE_S)
        if not in_highway and (in_tunnel or self.owner == 'TUNNEL'):
            if self.owner != 'TUNNEL':
                current_seed = seed_from_command(self.last_command, now)
                seed = current_seed or (prepared['seed'] if usable else None)
                if seed is not None:
                    runtime._transition_seed = copy.deepcopy(seed)
                runtime._transition_alternative = (copy.deepcopy(prepared['seed']) if usable else None)
                continuing = current_seed or (prepared['seed'] if usable else None)
                if continuing is not None:
                    runtime._tunnel_last_good = dict(stamp=now, plan={},
                                                     world_path=continuing['world_path'])
                self.owner = 'TUNNEL'
                runtime._lead_wait_since = None
                runtime._lead_bypass = None
            self.info = dict(state='ACTIVE', target='TUNNEL', prepared=usable,
                             approach=not in_tunnel)
            return 'TUNNEL'
        if (not in_highway and usable and distance is not None and
                0. <= distance <= PREPARE_MAX_M):
            proposal_id = prepared['generation'], prepared['received']
            if self.applied_proposal != proposal_id:
                current_seed = seed_from_command(self.last_command, now)
                runtime._lattice_prev = copy.deepcopy(current_seed or prepared['seed'])
                self.applied_proposal = proposal_id
            runtime._transition_entry_corridor = True
            self.info = dict(state='READY', target='TUNNEL', prepared=True,
                             reason='next path checked; ordinary policy owns approach')
        if not in_highway and prepared and prepared['target'] == 'GENERAL':
            if prepared['ready'] and 0. <= now-prepared['received'] <= PROPOSAL_MAX_AGE_S:
                runtime._lattice_prev = copy.deepcopy(prepared['seed'])
                self.info = dict(state='ACTIVE', target='GENERAL', prepared=True)
            self.prepared = None
        lag = max(0., float(getattr(runtime, 'decision_hz', 0.)))
        response = 1./lag if lag > 0. else .3
        window = min(PREPARE_MAX_M, max(PREPARE_MIN_M,
            float(ego.get('speed_kmh', 0.))/3.6*(PREPARE_TIME_S+response)))
        if (distance is not None and 0. <= distance <= window and
                not in_tunnel and self.future is None and now-self.last_submit >= .5):
            if self.executor is None:
                self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='planner-prepare')
            self.last_submit = now
            try:
                snapshot = self._snapshot(runtime, status)
            except Exception as exc:
                self.info = dict(state='PREPARING', target=target,
                                 reason='preview snapshot unavailable: '+str(exc))
                return None
            self.future_target = target
            self.info = dict(state='PREPARING', target=target, distance_m=round(distance, 2))
            self.future = self.executor.submit(self._prepare, snapshot, target, self.generation,
                                                seed_from_command(self.last_command, now))
        return None

    def after(self, runtime):
        publisher = getattr(runtime, 'path_publisher', None)
        command = getattr(publisher, '_published_plan', None)
        if command is not None:
            self.last_command = command
        decision = getattr(runtime, '_tunnel_decision_plan', None)
        if self.owner == 'TUNNEL' and decision is not None:
            self.info.update(plan_id=decision.plan_id, sensor_stamp=decision.sensor_stamp,
                             revalidated=bool(decision.validation.get('revalidated')),
                             handoff_reused=bool(decision.validation.get('handoff_reused')))
        record = getattr(runtime, '_web_record', None)
        if record is not None:
            record['transition'] = dict(self.info)

    @staticmethod
    def _snapshot(runtime, status):
        shadow = copy.copy(runtime)
        shadow.path_publisher = None
        shadow.tunnel_no_go_publisher = shadow.tunnel_no_go_receiver = None
        shadow._decision_log = SimpleNamespace(write=lambda line: None, dropped=0)
        shadow.latest_status = copy.deepcopy(status)
        for name in tuple(vars(shadow)):
            if name.startswith(('_tunnel_', '_lead_', '_static_', '_transition_')):
                delattr(shadow, name)
        shadow._transition_approach = True
        shadow.test_transition_status = copy.deepcopy(status)
        if getattr(runtime, 'lidar', None) is not None:
            scan = copy.deepcopy(runtime.lidar.scan_snapshot())
            points, age = runtime.lidar.snapshot()
            points = None if points is None else np.asarray(points).copy()
            shadow.lidar = copy.copy(runtime.lidar)
            shadow.lidar.scan_snapshot = lambda: scan
            shadow.lidar.snapshot = lambda: (points, age)
            stamps = (scan.get('stamp', 0.), status['ego'].get('stamp', 0.))
            for name in ('ego_receiver', 'tunnel_odometry'):
                receiver = getattr(runtime, name, None)
                if receiver is not None:
                    poses = {stamp: copy.deepcopy(receiver.at_stamp(stamp)) for stamp in stamps}
                    setattr(shadow, name, SimpleNamespace(at_stamp=lambda stamp, p=poses: p.get(stamp)))
        for name in ('vehicle_receiver', 'object_receiver'):
            receiver = getattr(runtime, name, None)
            if receiver is not None:
                data = copy.deepcopy(receiver.snapshot())
                clone = copy.copy(receiver)
                clone.snapshot = lambda d=data: d
                setattr(shadow, name, clone)
        return shadow

    @staticmethod
    def _prepare(shadow, target, generation, seed):
        status = shadow.test_transition_status
        ego = status['ego']
        received = time.monotonic()
        if target == 'TUNNEL':
            shadow._transition_seed = seed
            shadow.infer_tunnel(status, ego)
            record = shadow._web_record
            tunnel = record.get('tunnel') or {}
            ready = bool(tunnel.get('valid') and not tunnel.get('bypass_blocked') and
                         not record.get('emergency_stop') and record.get('planner_cap_kmh', 0.) > 0.)
            takeover_ready = ready
            local = np.asarray(shadow.last_result['path'], dtype=float)[:, :2]
            proposal = dict(stamp=received, target=shadow.last_result['postprocess'].get('applied_peak_m', 0.),
                            world_path=local @ rotation(ego).T+[ego['x'], ego['y']])
            decision = shadow._tunnel_decision_plan
            proposal.update(allowed_mps=decision.allowed_mps, sensor_stamp=decision.sensor_stamp,
                            plan_id=decision.plan_id, validation=dict(decision.validation),
                            decision=decision.summary())
            reason = record.get('reason')
            if not tunnel.get('valid') and ('both supported tunnel walls required' in reason or
                                            'outside mapped tunnel section' in reason):
                # Before both walls are visible, prepare against surveyed walls
                # and every raw return. This only seeds the ordinary planner;
                # it cannot authorize wall aiding or a tunnel takeover.
                from lattice_planner import plan
                from gui import MAX_CONTROL_CURVATURE, zone_speed_kmh
                scan = shadow.lidar.scan_snapshot()
                points = shadow.fallback_obstacle_points(scan)
                route = shadow.route_tensor(ego, status)[3]
                corridor = approach_corridor(shadow.tunnel, shadow.lanes, ego, route)
                if corridor is None:
                    return dict(target=target, generation=generation, received=received,
                        seed=None, ready=False, takeover_ready=False, reason='entry corridor unavailable')
                cars = shadow.tunnel_vehicle_objects([], ego)
                cap = zone_speed_kmh(shadow.speed_zones, status['path']['nearest_idx'])[0]/3.6
                result = plan(route, ego, points, cars, corridor, seed,
                    cruise_mps=cap, max_curvature=MAX_CONTROL_CURVATURE, max_offset_m=5.,
                    outside_max_m=0., blocked_standoff_m=6., edge_margin_m=.25,
                    extra_clearance=lambda p, h: shadow.tunnel.path_clearance(p, ego, h))
                ready = bool(scan.get('age_s', 999.) <= .25 and points is not None and
                             not result['blocked'] and not result['emergency'])
                proposal = dict(stamp=received, target=result['target'],
                    world_path=np.asarray(result['path'])[:, :2] @ rotation(ego).T+[ego['x'], ego['y']])
                reason = 'surveyed entry proposal: '+result['reason']
        else:
            takeover_ready = True
            from lattice_planner import plan
            from gui import MAX_CONTROL_CURVATURE, zone_speed_kmh
            route = shadow.route_tensor(ego, status)[3]
            points, age = shadow.lidar.snapshot()
            cars = shadow.vehicle_receiver.snapshot()[0] if shadow.vehicle_receiver else []
            end = int(shadow.highway.config['to_idx'])+1
            cap = zone_speed_kmh(shadow.speed_zones, end)[0]/3.6
            result = plan(route, ego, points, cars, shadow.lanes.corridor(ego, route), seed,
                          cruise_mps=cap, max_curvature=MAX_CONTROL_CURVATURE)
            ready = bool(age <= .25 and not result['blocked'] and not result['emergency'])
            proposal = dict(stamp=received, target=result['target'],
                            world_path=np.asarray(result['path'])[:, :2] @ rotation(ego).T+[ego['x'], ego['y']])
            reason = result['reason']
        return dict(target=target, generation=generation, received=received,
                    seed=proposal, ready=ready, takeover_ready=takeover_ready, reason=reason)
