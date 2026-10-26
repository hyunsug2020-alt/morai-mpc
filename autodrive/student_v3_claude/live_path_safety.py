"""Snapshot live sensors without waiting for camera inference or path search."""
from dataclasses import replace
import copy
import functools
import math
import threading
import time

import numpy as np

from driving_plan import DrivingPlan
from path_safety import PathSafety, stop_cap, FUSED_STATIC_MPS
import ring_entry
from tunnel_runtime import (points_at_pose, objects_at_pose, lidar_objects, rotation,
                            propagate_match, corridor_at_pose, swept_clearance, lattice_swept_clearance)
from tunnel_tracks import DetectorMotion, TunnelTracks, measured_vehicle, static_body
from traffic_planner import JunctionTrackMemory, INTERSECTION_CONTACT_MPS
from tunnel_memory import TunnelNearFieldMemory
from planning.tunnel.approach_memory import update_approach_points
from sim_time import SimClock, to_sim_objects
from ped_guard import PersonGhostFilter
from ego_body import EGO_HALF_LENGTH_M, EGO_FRONT_M, REAR_AXLE_SHIFT_M, ROAD_LEGACY_SHIFT_M, inside_body
from stop_line import intersection_front_station, INTERSECTION_ENTRY_MPS, intersection_entry_cap_mps
from roundabout_empty import (certified_geometry, certified_path_geometry, certified_pose_geometry,
                              approach_cap, braking_reserve, raw_entry_clearance,
                              semantic_entry_clearance, MAX_POSE_AGE_S, MAX_ENTRY_SOURCE_AGE_S, ROLL_MPS,
                              ROLL_TARGET_MPS, ROLL_CHECK_FRONT_GAP_M)


def owner_body_shift(owner):
    """Body placement on the pose (ego_body): the real rear-axle body in the tunnel,
    the road's legacy pose-centred box elsewhere."""
    return REAR_AXLE_SHIFT_M if owner == 'TUNNEL' else ROAD_LEGACY_SHIFT_M


def static_motion_source_freshness(points, ego, batches, scan_age, now, checked_at):
    """Keep current geometry separate from the age of its stationary label.

    A stationary semantic body retains the guard's existing 0.6-second label
    lifetime only when this fresh raw scan independently observes that body.
    People, vehicles, missing associations and empty semantic frames retain
    the 0.3-second entry limit. No source timestamp is advanced by association.
    Ages are expressed at ``now``; computation consumes the same lifetime.
    """
    cloud = np.asarray(points, float).reshape(-1, 2)
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    deadline = now+.3-float(scan_age)
    sources = []
    for source, rows, stamp, age in batches:
        limit, associated = .3, False
        if source == 'semantic' and rows and len(cloud):
            associated = True
            for obj in rows:
                if (len(obj) < 9 or obj[4] != 'obstacle'
                        or not np.isfinite(np.asarray([obj[i] for i in (0, 1, 5, 6, 7, 8)], float)).all()
                        or not 0. <= float(obj[5]) < FUSED_STATIC_MPS or min(obj[6:8]) <= 0.):
                    associated = False
                    break
                centre = (np.asarray(obj[:2])-[ego['x'], ego['y']])@rotation(ego)
                angle = math.radians(float(obj[8])-float(ego['heading_deg']))
                axes = np.array([[math.cos(angle), -math.sin(angle)],
                                 [math.sin(angle), math.cos(angle)]])
                local = (cloud-centre)@axes
                if not np.any(np.all(np.abs(local) <= np.asarray(obj[6:8])*.5+.25, axis=1)):
                    associated = False
                    break
            if associated:
                limit = .6
        valid_age = math.isfinite(float(age)) and 0. <= float(age) <= limit and float(stamp) > 0.
        deadline = min(deadline, now+limit-float(age)) if valid_age else -math.inf
        sources.append(dict(source=source, stamp=stamp, age_s=age,
                            max_age_s=limit, current_raw_association=associated))
    ready = bool(math.isfinite(deadline) and math.isfinite(scan_age)
                 and 0. <= scan_age <= .3 and now <= checked_at < deadline)
    return dict(ready=ready, expires_at=deadline if math.isfinite(deadline) else None,
                lidar_age_s=scan_age+max(0., checked_at-now), sources=sources)


def junction_moving_body_keys(batches, excluded_uids, owners_out=None):
    """Fresh measured NPCs whose full forecast stays in PathSafety.

    This transfers only the duplicate frozen future sweep. It grants no map
    timing permission and never supplies ``timed_vehicle_keys`` to the guard.
    """
    from junction_vehicle_motion import (_row, _alias, MAX_MEASURED_LENGTH_M,
                                        MAX_MEASURED_WIDTH_M)
    measured, measured_sources, sources = {}, {}, []
    if owners_out is not None:
        owners_out.clear()
    excluded = set(excluded_uids)
    for source, objects, _, age in batches:
        if not math.isfinite(age) or not 0. <= age <= .35:
            continue
        for obj in objects:
            if len(obj) <= 11 or obj[11] in excluded:
                continue
            try:
                measured_row = _row(obj, measured=True)
            except (IndexError, TypeError, ValueError, OverflowError):
                measured_row = False
            if (source == 'vehicles' and obj[3] == 'lidar_car' and measured_row
                    and obj[6] <= MAX_MEASURED_LENGTH_M and obj[7] <= MAX_MEASURED_WIDTH_M):
                current = list(obj)
                current[0] += current[9]*age
                current[1] += current[10]*age
                measured[(str(obj[3]), obj[11])] = tuple(current)
                measured_sources[(str(obj[3]), obj[11])] = (obj, age)
            elif obj[4] == 'npc' and obj[3] != 'lidar_car':
                sources.append((source, obj, age))
    keys, used, matches = set(measured), set(), []
    for source, obj, age in sources:
        for key, car in measured.items():
            try:
                matched = _alias(obj, age, car)
            except (IndexError, TypeError, ValueError, OverflowError):
                matched = False
            if matched:
                distance = np.linalg.norm(np.asarray(obj[:2])+np.asarray(obj[9:11])*age-car[:2])
                matches.append((float(distance), source, (str(obj[3]), obj[11]), key))
    for _, source, alias, key in sorted(matches, key=lambda m: (m[0], repr(m[2]))):
        if (source, key) not in used and alias not in keys:
            keys.add(alias)
            used.add((source, key))
            if owners_out is not None:
                owners_out[alias] = measured_sources[key]
    return frozenset(keys)


def remember_pedestrian_brakes(runtime, validation, batches, now):
    """Remember the exact current walkers for whom the live guard had to brake."""
    if not validation.get('people_emergency'):
        return
    checked = {(row.get('name'), row.get('uid')) for row in validation.get('objects', ())
               if row.get('state') == 'moving'}
    ids = {int(obj[11]) for _, objects, _, age in batches if 0. <= age <= .35
           for obj in objects if obj[4] == 'pedestrian' and len(obj) > 11
           and obj[11] is not None and (str(obj[3]), obj[11]) in checked}
    if ids:
        remembered = dict(getattr(runtime, '_ped_crossing_brakes', {}))
        remembered.update({uid: now for uid in ids})
        runtime._ped_crossing_brakes = remembered


def measured_people(points, ego, batches, stamp, age_s):
    """Associate compact fresh returns with a still-fresh semantic person.

    This refreshes geometry only. Class, departure status and motion evidence
    retain their original sensor timestamps. Unassociated raw points continue
    through the independent complete body sweep.
    """
    points = np.asarray(points, float).reshape(-1, 2)
    if not len(points):
        return []
    used, result = set(), []
    for _, objects, source_stamp, source_age in batches:
        if stamp < source_stamp:
            continue
        for obj in objects:
            if obj[4] != 'pedestrian' or len(obj) <= 11:
                continue
            predicted = np.asarray(obj[:2])+np.asarray(obj[9:11])*max(0., source_age-age_s)
            local = (predicted-[ego['x'], ego['y']])@rotation(ego)
            radius = min(.9, max(.45, .5*np.hypot(obj[6], obj[7])+.25))
            ids = np.flatnonzero(np.linalg.norm(points-local, axis=1) <= radius)
            ids = np.asarray([i for i in ids if i not in used], int)
            if len(ids) < 3:
                continue
            returns = points[ids]
            if np.max(np.ptp(returns, axis=0)) > 1.4:
                continue
            centre = np.median(returns, axis=0)
            if np.linalg.norm(centre-local) > .45:
                continue
            used.update(ids)
            result.append(dict(uid=obj[11], xy=(centre@rotation(ego).T+
                               [ego['x'], ego['y']]).tolist(), stamp=stamp, age_s=age_s))
    return result


def serialized_command(method):
    """Keep timer and planner path/speed publication one indivisible command."""
    @functools.wraps(method)
    def call(self, *args, **kwargs):
        if not hasattr(self, '_command_lock'):
            self._command_lock = threading.RLock()
        with self._command_lock:
            return method(self, *args, **kwargs)
    return call


class LivePathSafety:
    def update_yield_launch(self, ego=None, *, stopped=False):
        """Share launch cancellation across sensor checks and direct stop paths."""
        runtime = self.runtime
        controller = getattr(runtime, 'stop_line_controller', None)
        if controller is None or not getattr(controller, 'yield_launch_pending', False):
            return False
        # Only the provider's validated pose can prove physical crossing. A
        # direct stop/exception has no such proof and cancels a pending launch.
        front = intersection_front_station(ego, runtime.lanes.route_progress) if ego else math.nan
        if not controller.update_yield_launch(front, stopped=stopped):
            return False
        for planner in (getattr(runtime, 'gap_planner', None), self.traffic):
            if planner is not None:
                planner.junction_committed = False
                if isinstance(planner.junction, dict):
                    planner.junction = dict(planner.junction, held=True)
        return True

    def expire_yield_launch(self, now):
        last = getattr(self.runtime.path_publisher, '_last_target_time', now)
        if now-last > .6:
            self.update_yield_launch(stopped=True)

    def __init__(self, runtime):
        self.runtime = runtime
        self.guard = PathSafety()
        self.clusters = TunnelTracks()
        self.cluster_stamp = None
        self.cluster_objects = []
        self.detector_motion = DetectorMotion()
        self.owner = None
        self.traffic_baseline = None
        self.traffic = None
        self.junction_tracks = JunctionTrackMemory()
        self.vehicle_stamp = None
        self._junction_vehicle_frame = None
        self.junction_rows = []
        self.junction_measured_rows = []
        self.junction_confirmed_ids = frozenset()
        self.junction_unobserved_prior = set()
        self.junction_motion = ring_entry.RingMotionTracker()
        from junction_launch import JunctionLaunchMemory
        self.junction_launch = JunctionLaunchMemory()
        self.near_field = TunnelNearFieldMemory()
        # Fallback real-time factor for runtimes without their own clock.
        self.motion_clock = SimClock()

    def clear_published_launch(self, reason):
        self.junction_launch.clear(reason)

    def bound_static_lane_pass(self, plan, world_path, target_mps):
        """Recheck the actual post-ramp command against the original return clock."""
        policy = getattr(self.runtime, '_static_lane_pass', None)
        if (policy is None or policy.finished or plan is None or target_mps <= 0. or plan.emergency
                or not (plan.validation.get('static_lane_borrow')
                        or policy.entered_at is not None)):
            return plan, target_mps
        path = (np.asarray(world_path)-[plan.control_ego['x'], plan.control_ego['y']]) @ rotation(plan.control_ego)
        if plan.validation.get('static_motion_profile'):
            ceiling = plan.validation.get('static_profile_live_ceiling_mps')
            current_limit = plan.validation.get('static_profile_live_command_mps')
            deadline = plan.validation.get('static_profile_deadline')
            if deadline is not None and time.monotonic() > float(deadline):
                return plan.decided(allowed_mps=0., emergency=True,
                    stop_reason='static motion evidence expired'), 0.
            if ceiling is not None and current_limit is not None and target_mps+1e-6 < current_limit:
                ceiling = min(float(ceiling), float(target_mps))
            check = policy.command_check(path, plan.control_ego, target_mps, time.monotonic(),
                motion_profile=True, profile_ceiling_mps=ceiling)
        else:
            check = policy.command_check(path, plan.control_ego, target_mps, time.monotonic())
        plan = plan.decided(static_lane_pass=check)
        if not check['ok']:
            return plan.decided(allowed_mps=0., emergency=True,
                stop_reason=check.get('reason') or 'static pass return proof unavailable'), 0.
        return plan, target_mps

    def note_published(self, plan, world_path, target_mps, now):
        """Observe delivered, freshly checked approach and entry commands."""
        request = plan.validation.get('junction_source_launch') if plan is not None else None
        if request and request.get('consumed'):
            if (not math.isfinite(float(target_mps)) or target_mps <= 0. or plan.emergency
                    or not plan.junction_motion_fresh(now)):
                self.reject_source_publication(plan, 'accepted launch not published as a fresh positive command')
            else:
                request['published'] = True
                if request.get('transaction_id') == getattr(self, '_source_launch_transaction', None):
                    self._source_launch_transaction = None
        usable = bool(plan is not None and self.traffic is not None
            and (self.traffic.junction or {}).get('following_only')
            and not plan.emergency and target_mps > 0.
            and plan.validation.get('junction_live')
            and plan.junction_motion_fresh(now))
        if not usable:
            self.junction_launch.clear('no positive fresh junction publication')
            return
        deadlines = [plan.validation['junction_live']['expires_at']]
        if plan.validation.get('junction_motion'):
            deadlines.append(plan.validation['junction_motion']['expires_at'])
        self.junction_launch.published(world_path, plan.control_ego, target_mps, now,
            proof_expires_at=min(deadlines),
            proof_valid=True, emergency=False)

    def publication_source_guard(self, plan):
        from junction_source_guard import publication_guard
        return publication_guard(self, plan)

    def reject_source_publication(self, plan, reason='junction source publication rejected'):
        plan.validation.pop('junction_entry_profile_accepted', None)
        plan.validation['junction_source_rejected'] = reason
        request = plan.validation.get('junction_source_launch') or {}
        if (request.get('consumed') and not request.get('published')
                and request.get('transaction_id') == getattr(self, '_source_launch_transaction', None)):
            controller = getattr(self.runtime, 'stop_line_controller', None)
            if controller is not None:
                # This nonce belongs to the current not-yet-delivered command,
                # even if its checked front crossed the line. Never undo an
                # entry already delivered by an earlier publication.
                with controller._state_lock:
                    controller.released = controller.yield_launch_pending = controller.yield_approach = False
                    controller.yield_wait_latched = True
                    for planner in (getattr(self.runtime, 'gap_planner', None), self.traffic):
                        if planner is not None:
                            planner.junction_committed = False
                            planner.junction = dict(planner.junction, held=True)
            self._source_launch_transaction = None
            request['consumed'] = False

    def motion_rtf(self, ego, now):
        """The runtime's simulator real-time factor (gui updates it once per
        decision from the ESKF pose); measured here only without that clock."""
        clock = getattr(self.runtime, 'sim_clock', None)
        if clock is not None:
            return clock.rtf
        self.motion_clock.update(ego, now)
        return self.motion_clock.rtf

    def tunnel_near_points(self, ego):
        """Copy live blind-zone evidence for the existing tunnel bypass search.

        The timer sees more scans than the planner. Its retained measured
        barriers must also enter candidate generation, not only veto the
        selected path after the search. Never replay an old scan into memory.
        """
        publisher = self.runtime.path_publisher
        with publisher._command_lock:
            if self.owner != 'TUNNEL' or self.near_field.last_scan is None:
                return np.empty((0, 2))
            stamp = self.near_field.last_scan
            observations = list(self.near_field.cells.values())
        retained = []
        for observation in observations:
            when, count, world = observation
            if count < 2 or not 0. <= stamp-when:
                continue
            local = (world-[ego['x'], ego['y']])@rotation(ego)
            if -2.5 < local[0] < 6.5 and abs(local[1]) < 3.:
                retained.append(local)
        return np.asarray(retained, float).reshape(-1, 2)

    def recovery_inputs(self, ego, mode, requested_mps=None):
        """Give the existing bypass planner a non-mutating dynamic constraint.

        A speed-only stop does not tell the static planner which measured body
        blocked its path. Export those bodies for candidate generation, then
        check every selectable candidate with the original motion histories.
        Candidate rejection must never clear the actual stop latch.
        """
        publisher = self.runtime.path_publisher
        with publisher._command_lock:
            active = getattr(publisher, '_active_decision_plan', None)
            owner = 'TUNNEL' if mode.startswith('TUNNEL') else 'ROAD'
            retained = active.validation.get('dynamic_recovery_ids', []) if active is not None else []
            if (owner != self.owner or mode == 'HIGHWAY' or
                    active is None or not active.fresh() or
                    not active.validation.get('valid')):
                return None
            checks = active.validation.get('live_safety', {}).get('objects', [])
            baseline = getattr(publisher, '_safety_baseline', None) or {}
            planned = baseline.get('decision')
            requested = (planned.allowed_mps if planned is not None else
                         float(getattr(publisher, 'zone_mps', 0.)))
            if requested_mps is not None:
                # A previous dynamic cap belongs to its previous path. Search
                # the new candidates at this section's policy ceiling, then
                # apply their own motion, curvature and physical-clearance caps.
                requested = float(requested_mps)
                if not math.isfinite(requested) or requested <= 0.:
                    return None
            else:
                requested = max(.8, requested, abs(float(ego.get('speed_kmh', 0.)))/3.6)
            stopped = active.validation.get('live_safety', {}).get('stopped', False)
            near_m = max(8., abs(float(ego.get('speed_kmh', 0.)))/3.6*1.5+4.)
            checks = [check for check in checks if stopped or check['cap_mps'] < requested-.01
                      or check['gap_m'] < near_m]
            if not checks and not retained:
                return None
            # Circulating traffic keeps the existing gap/entry policy.
            if owner == 'ROAD' and (getattr(self.runtime, '_intersection_vehicle_status', None) is not None or
                                   (self.traffic is not None and self.traffic.junction is not None)):
                return None
            guard = copy.deepcopy(self.guard)
        now = time.monotonic()
        observed = (self.runtime.tunnel_odometry.at_stamp(float(ego.get('stamp', 0.)))
                    if owner == 'TUNNEL' else ego)
        if observed is None:
            return None
        ids = set(retained) | {check['uid'] for check in checks}
        objects = []
        expires_at = math.inf
        for track in guard.memory.observations(now):
            obj = track['object']
            if len(obj) <= 11 or not ids.intersection(track.get('identity_ids', (obj[11],))):
                continue
            row = list(obj)
            row[:2] = np.asarray(row[:2])+track['velocity']*(now-track['observed_at'])
            local = (np.asarray(row[:2])-[observed['x'], observed['y']])@rotation(observed)
            if local[0]+.5*np.hypot(row[6], row[7]) < -(EGO_HALF_LENGTH_M-owner_body_shift(owner)):
                continue  # release the manoeuvre only after the whole body passed
            expires_at = min(expires_at, track['observed_at']+.6)
            if owner == 'TUNNEL':
                row = list(objects_at_pose([tuple(row)], observed, ego)[0])
            # Freeze only the candidate-generation footprint. Actual motion
            # and possible departure remain in the independent check below.
            row[4], row[5], row[9], row[10] = 'obstacle', 0., 0., 0.
            objects.append(tuple(row))
        if not objects:
            return None

        cached = {}
        def evaluate(path, horizon):
            if time.monotonic() > expires_at:
                return None  # expired motion evidence cannot authorize a candidate
            key = (np.asarray(path, float).tobytes(), float(horizon))
            if cached.get('key') == key:
                return cached['result']
            guard.stopped, guard.clear_since = False, None
            guard.clear_samples.clear()
            result = guard.check(path, ego, [], time.monotonic(),
                requested_mps=requested,
                observation_ego=observed, dynamic_only=True, prediction_horizon_s=4.,
                motion_time_scale=self.motion_rtf(observed, time.monotonic()))
            cached.update(key=key, result=result)
            return result

        def clearance(path, horizon):
            result = evaluate(path, horizon)
            return (0. if result is None or result.emergency or result.cap_mps <= .05
                    else min(float(horizon), result.clear_m))

        def speed_cap(path, horizon):
            result = evaluate(path, horizon)
            return 0. if result is None or result.emergency else result.cap_mps

        return dict(objects=objects, clearance=clearance, speed_cap=speed_cap,
                    ids=sorted({o[11] for o in objects}))

    def measured_tunnel_cars(self, cars, stamp):
        """Detector cars in the tunnel move only as the measured clusters do.

        Parked-car boxes drift with their visible part, as in the planner. A
        standing box drifted over the bypass margin it was planned with
        (follow-20261010T062150Z 285.6 s: "collision: lidar_car" at 30 km/h with
        the raw returns clear for 54 m); the raw-point check owns its body.
        """
        movers = [o for o in self.cluster_objects if o[4] == 'pedestrian']
        cars = [measured_vehicle(car, movers, unknown_static=True,
                                 persisted=self.detector_motion.persisted(car, stamp)) for car in cars]
        return [car if float(car[5]) >= .3 else static_body(car) for car in cars]

    def person_measurements(self, points, ego, batches, stamp, age_s, now):
        """Refresh a briefly lost person's geometry with associated raw returns.

        A person leaving the camera's view can remain beside the rear body and
        visible to LiDAR. Preserve the original motion history and expiry; only
        an independently associated new scan supplies a newer position/age.
        Empty camera frames and rereading old LiDAR never renew that evidence.
        """
        associations = list(batches)
        current_ids = {o[11] for _, objects, _, _ in batches for o in objects if len(o)>11}
        for track in self.guard.memory.observations(now):
            obj = track['object']
            if obj[4] != 'pedestrian' or len(obj)<=11 or obj[11] in current_ids:
                continue
            row = list(obj)
            row[9:11] = track['velocity']
            associations.append(('retained_semantic', [tuple(row)], track['stamp'],
                                 now-track['observed_at']))
        return measured_people(points, ego, associations, stamp, age_s)

    def junction_snapshot(self, original):
        """Copy the entry planner's junction mode for this publication.

        Legacy line-gated entries retain their accepted commitment. No-line
        following mode instead owns current map timing here, using this
        publication's path, pose and measured-source motion provenance.
        """
        if original is self.traffic_baseline:
            return
        planner = getattr(self.runtime, 'gap_planner', None)
        junction = getattr(planner, 'junction', None)
        if isinstance(junction, dict) and junction.get('following_only'):
            prior = set().union(*(set(junction.get(field) or ())
                for field in ('unknown_ids', 'inherited_ids', 'ghost_ids')))
            if self.junction_tracks.last_now is None:
                # At bootstrap the GUI can remember a recently lost car this
                # live memory never saw. An empty new packet cannot clear it.
                self.junction_unobserved_prior.update(prior)
            else:
                self.junction_unobserved_prior.intersection_update(prior)
        else:
            self.junction_unobserved_prior.clear()
        previous_live = self.traffic
        self.traffic_baseline = original
        self.traffic = copy.copy(planner) if planner is not None else None
        if self.traffic is not None:
            self.traffic.junction = copy.copy(planner.junction)
            self.traffic.speed_ratio = dict(planner.speed_ratio)
            mode = self.traffic.junction or {}
            old_mode = getattr(previous_live, 'junction', None) or {}
            if (mode.get('yield_only') and mode.get('constant_ring_speed')
                    and mode.get('yield_advance_m', 0.) > 0.
                    and mode.get('line_id') == old_mode.get('line_id')
                    and old_mode.get('yield_only') and old_mode.get('constant_ring_speed')):
                # Current publications own this mode's waiting/motion history;
                # a new GUI geometry proposal does not restart the map model.
                self.traffic.state = previous_live.state
                self.traffic.last_now = previous_live.last_now
                self.traffic.junction_wait_since = previous_live.junction_wait_since
                self.traffic._standing_since = dict(previous_live._standing_since)

    def junction_vehicles(self, cars, stamp, age, now, ego):
        """Use the existing dropout/ID-change rules without refreshing a scan."""
        junction = self.traffic.junction or {}
        measured_timing = bool(junction.get('following_only') or
            (junction.get('yield_only') and junction.get('constant_ring_speed')))
        if stamp != self.vehicle_stamp:
            measured = [c for c in cars if len(c) <= 15 or not c[15]]
            forecasts = [c for c in cars if len(c) > 15 and c[15]]
            promoted = frozenset()
            if measured_timing:
                from junction_vehicle_motion import source_motion_rows
                stamps = {c[11]: stamp for c in measured if len(c) > 11}
                measured, promoted = source_motion_rows(measured, self.junction_motion, stamps)
            self.junction_measured_rows = list(measured)
            self.junction_confirmed_ids = promoted
            rows = self.junction_tracks.update(measured, now-age,
                (ego['x'], ego['y']), self.traffic.map_predictor)
            represented = {c[11] for c in rows}
            self.junction_rows = rows+[c for c in forecasts if c[11] not in represented]
            self.vehicle_stamp = stamp
        if measured_timing:
            self.traffic.speed_ratio.update(self.junction_motion.ratio)
            # Provenance belongs to this source and this live memory, rather
            # than the GUI's earlier detector frame. Re-reading a source keeps
            # its current unknown/inherited/ghost flags without renewing it.
            for field in ('unknown_ids', 'inherited_ids', 'ghost_ids'):
                self.traffic.junction[field] = set(getattr(self.junction_tracks, field))
            self.traffic.junction['ghost_ids'].update(c[11] for c in self.junction_rows
                if len(c)>15 and c[15])
            self.junction_unobserved_prior.difference_update(
                c[11] for c in self.junction_rows if len(c)>11)
            self.traffic.junction['unobserved_prior_ids'] = set(self.junction_unobserved_prior)
        rows = []
        for car in self.junction_rows:
            track = dict(obj=car, velocity=car[9:11])
            xy, velocity, heading = self.junction_tracks._predict(track, age, self.traffic.map_predictor)
            if len(car) > 14 and car[14]:
                car = self.junction_tracks._with_velocity(car, velocity, xy, heading)
            rows.append(car)
        return rows

    def junction_follow(self, result, runtime, traffic_cars, rtf, world_path, ego,
                        *, empty_vehicle_ready=False, empty_raw_clear_m=None, empty_sources_expire_at=None,
                        empty_path_geometry=None, empty_commit_allowed=False,
                        yield_vehicle_ready=False, yield_sources_expire_at=None,
                        yield_source_frame=None, yield_source_receiver=None,
                        yield_profile_override=False, yield_independent_clear_m=None,
                        yield_motion_evaluation=None):
        """Committed roundabout entry: following caps only (user rule 2026-10-10).

        Before the commit the published decision owns waiting at the line. After it,
        a car or obstacle ahead on our path only slows us (to a standstill behind a
        stopped leader); collision stops for anything but people become the braking
        cap behind the measured blocker, and nothing beside/behind us stops us."""
        planner = getattr(runtime, 'gap_planner', None)
        committed = bool(getattr(planner, 'junction_committed', False) or
                         getattr(self.traffic, 'junction_committed', False))
        junction = self.traffic.junction if self.traffic is not None else None
        yield_only = bool(junction and junction.get('yield_only'))
        if not committed and yield_only:
            # A moving permission is checked at every publication. The latest
            # vehicle frame may revoke a GUI GO before the physical front crosses.
            from stop_line import yield_line_cap, STOP_CAPTURE_M
            front = intersection_front_station(ego, runtime.lanes.route_progress)
            gap = float(junction['line_station_m'])-front
            speed = abs(float(ego.get('speed_kmh', 0.)))/3.6
            checked_now = time.monotonic()
            reported_pose_age = float(ego.get('age_s', 999.))
            pose_receipt_age = checked_now-float(ego.get('received', checked_now))
            pose_age = max(reported_pose_age, pose_receipt_age)
            fresh = bool(yield_vehicle_ready and yield_sources_expire_at is not None
                and math.isfinite(reported_pose_age) and 0. <= reported_pose_age <= .2
                and math.isfinite(pose_receipt_age) and 0. <= pose_receipt_age <= .2
                and math.isfinite(yield_sources_expire_at) and checked_now <= yield_sources_expire_at
                and math.isfinite(gap) and 0. <= pose_age <= .2 and 0. < rtf <= 1.)
            expiry = min(yield_sources_expire_at or -math.inf, checked_now+.2-pose_age)
            if fresh:
                self.traffic.junction = dict(junction, held=True, line_gap_m=gap)
                local = (np.asarray(world_path)-[ego['x'], ego['y']])@rotation(ego)
                entry_cap = intersection_entry_cap_mps(gap)
                checked_cap = min(result.cap_mps, entry_cap) if entry_cap is not None else result.cap_mps
                cached = yield_motion_evaluation or {}
                from junction_vehicle_motion import proof_matches
                reuse = bool(cached.get('proof') is not None
                    and abs(float(cached['profile_mps'])-checked_cap) <= 1e-9
                    and proof_matches(cached['proof'], ego, world_path, now=checked_now))
                if reuse:
                    # This publication already assessed these exact sources,
                    # pose, path and speed profile. Body checks cannot renew
                    # its deadline or reuse timing at a newly reduced ceiling.
                    traffic = copy.deepcopy(cached['decision'])
                    self.traffic.status = copy.deepcopy(cached['status'])
                    self.traffic.state = self.traffic.status['state']
                    expiry = min(expiry, cached['proof'].expires_at)
                    result.validation['junction_timing_reused'] = True
                else:
                    traffic = self.traffic.evaluate(to_sim_objects(traffic_cars, rtf), ego, local,
                        checked_cap, checked_now, fresh=True)
                go = bool(traffic and self.traffic.status.get('ring', {}).get('action') == 'GO'
                    and traffic.get('kind') in ('enter', 'follow') and traffic.get('v_mps', 0.) > .3)
                cap = min(result.cap_mps, max(0., float(traffic['v_mps'])))
                reason = traffic['reason']
            else:
                go = False
                cap = min(result.cap_mps, yield_line_cap(gap, speed, pose_age))
                reason = 'yield entry source proof unavailable: brake toward line'
            result.validation['junction_traffic'] = dict(state='APPROACH_YIELD' if go else 'YIELD_WAIT',
                reason=reason, gap_checked=go, front_gap_m=gap,
                ring=self.traffic.status.get('ring') if fresh else None)
            # Recheck deadlines after traffic prediction as well as under the
            # controller lock. No old GO can grant a crossing after a newer WAIT.
            if go and time.monotonic() > expiry:
                go = False
                current_age = max(pose_age, time.monotonic()-float(ego.get('received', checked_now)))
                cap = min(cap, yield_line_cap(gap, speed, current_age))
                result.validation['junction_traffic'].update(state='YIELD_WAIT', gap_checked=False,
                    reason='yield entry source proof expired during prediction')
            if go and yield_profile_override:
                from junction_vehicle_motion import _required_rear_travel
                try:
                    required = _required_rear_travel(self.traffic.status.get('ring'))
                    distance = max(required.values(), default=0.) if required is not None else math.inf
                    # A stopping cap cannot promise continued travel through
                    # an independent static stop. Moving current bodies keep
                    # their separate command cap and never shorten this proof.
                    clear = min(float(result.clear_m), max(0., float(yield_independent_clear_m)-1.))
                    enough = math.isfinite(clear) and math.isfinite(distance) and clear >= distance
                except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
                    clear, distance, enough = 0., math.inf, False
                result.validation['junction_rear_travel'] = dict(
                    required_m=distance if math.isfinite(distance) else None,
                    clear_m=clear, accepted=bool(enough))
                if not enough:
                    go, cap = False, 0.
                    reason = 'junction rear clearance blocked before incoming traffic'
                    result.validation['junction_traffic'].update(
                        state='YIELD_WAIT', gap_checked=False, reason=reason)
            if go and yield_source_frame is not None:
                from junction_source_guard import current_frame
                source_ok, source_state = current_frame(yield_source_receiver, yield_source_frame)
                if not source_ok:
                    result.validation['junction_source_retry_needed'] = source_state
                    return replace(result, cap_mps=0., emergency=True,
                                   reason='junction source changed during launch checks')
            if (yield_profile_override and fresh and not result.emergency
                    and math.isfinite(result.cap_mps) and result.cap_mps > 0.
                    and time.monotonic() <= expiry):
                # A WAIT does not grant movement, but its current independent
                # body/source check may refresh the prospective profile. Keep
                # traffic headway out of that future ceiling; the command is
                # still bounded by this publication's traffic/body result.
                result.validation['junction_entry_profile_check'] = dict(
                    ceiling_mps=min(result.cap_mps, INTERSECTION_ENTRY_MPS),
                    expires_at=expiry)
            controller = getattr(runtime, 'stop_line_controller', None)
            launch_distance = getattr(controller, 'yield_launch_capture_m', STOP_CAPTURE_M)
            launch = go and gap <= launch_distance
            if controller is not None and not launch:
                cap = controller.apply_yield_wait(junction.get('line_id'), junction['line_station_m'],
                    front, speed, cap, go=go and empty_commit_allowed
                        and not result.emergency and cap > 0., expires_at=expiry)
                if cap <= 0.:
                    go = False
                    result.validation['junction_traffic'].update(state='YIELD_WAIT', gap_checked=False)
            if go and yield_source_frame is not None:
                if empty_commit_allowed and not result.emergency and cap > 0.:
                    result.validation['junction_source_launch'] = dict(
                        kind='launch' if launch or gap <= 0. else 'provisional',
                        frame=yield_source_frame, expires_at=expiry,
                        line_id=junction.get('line_id'), line_station_m=junction['line_station_m'],
                        front_station_m=front, speed_mps=speed, pose_age_s=pose_age)
                    from junction_source_guard import receipt_evidence
                    result.validation['junction_source_launch']['receiver_capture'] = \
                        receipt_evidence(yield_source_receiver, yield_source_frame)
            if launch or gap <= 0.:
                if (not go or not empty_commit_allowed or result.emergency or cap <= 0.
                        or controller is None):
                    return replace(result, cap_mps=0., emergency=True,
                                   reason='moving entry launch not freshly authorized')
                if yield_source_frame is None:
                    # Direct classification callers retain the old API. The
                    # production provider always defers consumption to the
                    # publisher's atomic complete-source barrier below.
                    if not controller.consume_yield_launch(junction.get('line_id'),
                            junction['line_station_m'], front, speed, pose_age, expires_at=expiry):
                        return replace(result, cap_mps=0., emergency=True,
                                       reason='moving entry launch not freshly authorized')
                    planner.latch_junction_commit()
                    self.traffic.latch_junction_commit()
                    result.validation['junction_traffic']['state'] = 'ENTER'
            return replace(result, cap_mps=cap, emergency=result.emergency or cap <= 0.,
                reason=result.reason if result.emergency else reason)
        if not committed:
            junction = self.traffic.junction if self.traffic is not None else None
            provisional = bool(junction and junction.get('allow_empty_entry') is True
                               and self.traffic.status.get('state') == 'APPROACH_EMPTY')
            if provisional:
                front_station = intersection_front_station(ego, runtime.lanes.route_progress)
                try:
                    line_station = float(junction['line_station_m'])
                except (KeyError, TypeError, ValueError):
                    line_station = math.nan
                front_gap = line_station - front_station
                geometry_ok = (certified_geometry(runtime.lanes, ego, world_path)
                    if empty_path_geometry is None else bool(empty_path_geometry
                        and certified_pose_geometry(runtime.lanes, ego)))
                checked_now = time.monotonic()
                try:
                    pose_age = (max(float(ego.get('age_s', 0.)), time.monotonic()-float(ego['received']))
                                if 'received' in ego else float(ego.get('age_s', 0.)))
                    speed = abs(float(ego.get('speed_kmh', 0.)))/3.6
                except (TypeError, ValueError):
                    pose_age = speed = math.nan
                sources_fresh = bool(empty_vehicle_ready and (empty_sources_expire_at is None
                    or (math.isfinite(empty_sources_expire_at) and checked_now <= empty_sources_expire_at)))
                crossing = math.isfinite(front_gap) and front_gap <= 0.
                near_roll = front_gap <= ROLL_CHECK_FRONT_GAP_M and speed <= ROLL_MPS
                raw_reserve = (0. if empty_raw_clear_m is None else
                               min(result.clear_m, float(empty_raw_clear_m)))
                usable = (math.isfinite(front_gap) and math.isfinite(pose_age)
                    and 0. <= pose_age <= MAX_POSE_AGE_S and sources_fresh
                    and geometry_ok and math.isfinite(speed) and 0. < rtf <= 1.
                    and (not crossing or speed <= ROLL_MPS)
                    and (not near_roll or raw_reserve >= braking_reserve(speed, MAX_ENTRY_SOURCE_AGE_S)))
                result.validation['empty_entry_checks'] = dict(
                    geometry=geometry_ok, sources_fresh=sources_fresh,
                    pose_age_s=pose_age, front_gap_m=front_gap,
                    raw_reserve_m=raw_reserve, required_reserve_m=braking_reserve(speed, MAX_ENTRY_SOURCE_AGE_S))
                if not usable or traffic_cars:
                    # Loss of the tighter rolling proof restores the normal
                    # braking approach. It is not evidence of an obstacle at
                    # the current pose. The measured collision result remains
                    # authoritative and a missing proof still stops at the line.
                    fallback = (approach_cap(front_gap, speed, pose_age) if
                        math.isfinite(front_gap) and front_gap > 0. else 0.)
                    cap = min(result.cap_mps, fallback)
                    reason = ('empty entry proof unavailable: brake toward line' if cap > 0.
                              else 'empty entry clearance lost before commit')
                    result.validation['junction_traffic'] = dict(state='WAIT', reason=reason)
                    return replace(result, cap_mps=cap, emergency=result.emergency or cap <= 0.,
                                   reason=result.reason if result.emergency else reason)
                # Use the current pose, not the older publication pose. A timer
                # can observe a line crossing before the next decision commits it.
                if crossing:
                    if empty_commit_allowed and not result.emergency and result.cap_mps > 0.:
                        controller = getattr(runtime, 'stop_line_controller', None)
                        expiry = checked_now+MAX_POSE_AGE_S-pose_age
                        if empty_sources_expire_at is not None:
                            expiry = min(expiry, empty_sources_expire_at)
                        if (controller is None or not controller.consume_empty_crossing(
                                junction.get('line_id'), line_station, front_station, speed, pose_age,
                                expires_at=expiry)):
                            result.validation['junction_traffic'] = dict(state='WAIT', reason='empty entry active line changed')
                            return replace(result, cap_mps=0., emergency=True,
                                           reason='empty entry active line changed')
                        planner.latch_junction_commit()
                        self.traffic.latch_junction_commit()
                        self.traffic.junction['held'] = False
                    frame = ring_entry.PathFrame(np.asarray(world_path, float)[:, :2])
                    centre = float(frame.project(np.asarray([ego['x'], ego['y']], float))[0][0])
                    cap, _ = ring_entry.follow_cap(to_sim_objects(traffic_cars, rtf), frame,
                        centre if math.isfinite(centre) else 0., speed)
                    result = replace(result, cap_mps=min(result.cap_mps, cap))
                    result.validation['junction_traffic'] = dict(state='ENTER', reason='empty entry front crossed line')
                    return result
                result.validation['junction_traffic'] = dict(state='APPROACH_EMPTY',
                    reason='fresh empty entry rechecked', front_gap_m=front_gap)
                cap = approach_cap(front_gap, speed, pose_age, junction.get('empty_entry_clearance_m', 0.))
                return replace(result, cap_mps=min(result.cap_mps, cap),
                    emergency=result.emergency or cap <= 0.,
                    reason=(result.reason if result.emergency or cap > 0. else 'empty entry braking reserve exhausted'))
            result.validation['junction_traffic'] = dict(state='WAIT', reason='entry owned by the decision')
            return result
        speed = abs(float(ego.get('speed_kmh', 0.)))/3.6
        frame = ring_entry.PathFrame(np.asarray(world_path, float)[:, :2])
        centre = float(frame.project(np.asarray([ego['x'], ego['y']], float))[0][0])
        cap, leader = ring_entry.follow_cap(to_sim_objects(traffic_cars, rtf), frame,
                                            centre if math.isfinite(centre) else 0., speed,
                                            ego_front_m=EGO_FRONT_M if yield_only else ring_entry.EGO_HALF_LENGTH_M)
        reason = str(result.reason or '')
        if (result.emergency and not result.validation.get('people_emergency')
                and not any(word in reason.lower() for word in ('person', 'pedestrian', 'cycl'))):
            # Brake behind the measured blocker instead of an emergency stop.
            brake = stop_cap(max(0., float(result.clear_m)-1.), .3)
            result = replace(result, emergency=False, cap_mps=brake,
                             reason='live junction committed: brake behind '+reason)
        if math.isfinite(cap):
            result = replace(result, cap_mps=min(result.cap_mps, cap))
        result.validation['junction_traffic'] = dict(
            state='ENTER', reason='junction committed: follow the car ahead' if leader else 'junction committed: drive on',
            target_mps=round(float(min(cap, result.cap_mps)), 3) if math.isfinite(cap) else None, leader=leader)
        return result

    def __call__(self, world_path, current, mode, original=None, *, _source_retry=False):
        runtime = self.runtime
        now = time.monotonic()
        call_started_at = now
        tunnel = mode.startswith('TUNNEL')
        owner = 'TUNNEL' if tunnel else 'ROAD'
        if owner != self.owner:
            self.guard = PathSafety(body_shift_m=owner_body_shift(owner))
            self.clusters.reset()
            self.cluster_stamp = None
            self.cluster_objects = []
            self.detector_motion = DetectorMotion()
            self.owner = owner
            self.junction_tracks.reset()
            self.vehicle_stamp = None
            self._junction_vehicle_frame = None
            self.junction_rows = []
            self.junction_measured_rows = []
            self.junction_confirmed_ids = frozenset()
            self.junction_unobserved_prior.clear()
            self.junction_motion.reset()
            self.junction_launch.clear('control owner changed')
            self.near_field = TunnelNearFieldMemory()
            self.motion_clock = SimClock(self.motion_clock.estimate)
            if tunnel and hasattr(runtime, '_portal_approach_memory'):
                runtime._portal_approach_memory.reset()
        if not tunnel:
            self.junction_snapshot(original)
        timed_vehicles = bool(not tunnel and self.traffic is not None and self.traffic.junction is not None)
        following_only = bool(timed_vehicles and self.traffic.junction.get('following_only'))
        static_source_profile = bool(not tunnel and original is not None
            and original.validation.get('static_motion_profile')
            and original.validation.get('static_lane_borrow'))
        capture_age_origin = timed_vehicles or static_source_profile
        yield_moving_bodies = bool(timed_vehicles and self.traffic.junction.get('yield_only')
            and self.traffic.junction.get('constant_ring_speed') and not following_only
            and not self.traffic.junction.get('recovery_only'))
        yield_motion_entry = bool(yield_moving_bodies
            and not getattr(self.traffic, 'junction_committed', False))
        recovery = getattr(runtime, 'junction_recovery', None) if not tunnel else None
        recovery_requested = bool(timed_vehicles and self.traffic.junction.get('recovery_only'))
        selected_reference = bool(not tunnel and original is not None
                                  and original.validation.get('entry_reference'))
        committed = bool(getattr(getattr(runtime, 'gap_planner', None), 'junction_committed', False)
                         or getattr(self.traffic, 'junction_committed', False))
        # Both publication owners can observe commitment. An older copied
        # planner cannot demand another entry certificate after that release.
        yield_motion_entry = yield_motion_entry and not committed
        reference_entry_check = selected_reference and not committed
        reference_roll_speed = bool(reference_entry_check and timed_vehicles
            and self.traffic.status.get('state') == 'APPROACH_EMPTY')
        empty_path_geometry = None
        if (reference_entry_check or (timed_vehicles and self.traffic.junction.get('allow_empty_entry') is True
                and self.traffic.status.get('state') == 'APPROACH_EMPTY'
                and not getattr(runtime.gap_planner, 'junction_committed', False)
                and not self.traffic.junction_committed)):
            # Certify this publication's fixed world path before capturing any
            # entry sensors. Pose-dependent body checks use the following frame.
            world_path = np.array(world_path, dtype=float, copy=True)
            empty_path_geometry = certified_path_geometry(runtime.lanes, world_path)
            now = time.monotonic()
        ego, _, _ = runtime.ego_receiver.snapshot(1)
        ego = dict(ego or current)
        ego.pop('_junction_launch', None)
        if not following_only:
            self.junction_launch.clear('outside no-line junction')
        path = (np.asarray(world_path)-[ego['x'], ego['y']]) @ rotation(ego)
        validation = dict(original.validation) if original is not None else dict(
            valid=True, pose_valid=True, steering_valid=True, blocked=False)
        static_rear_axle = bool(not tunnel and validation.get('static_rear_axle'))
        # A copied plan cannot carry another publication's motion permission.
        validation.pop('junction_motion', None)
        validation.pop('junction_yield_motion', None)
        validation.pop('junction_curve_profile', None)
        validation.pop('junction_live', None)
        validation.pop('junction_recovery', None)
        validation.pop('junction_launch', None)
        validation.pop('junction_launch_status', None)
        validation.pop('junction_compute_ms', None)
        validation.pop('junction_motion_check', None)
        validation.pop('junction_source_retry', None)
        validation.pop('junction_source_launch', None)
        validation.pop('junction_source_publication', None)
        validation.pop('junction_source_same_stamp_revision', None)
        validation.pop('junction_entry_profile_accepted', None)
        validation.pop('junction_entry_profile_refreshed', None)
        entry_profile = None
        if (timed_vehicles and not committed and not recovery_requested
                and self.traffic.junction.get('yield_only') and original is not None):
            from junction_entry_profile import entry_profile_ceiling
            entry_profile = entry_profile_ceiling(original, now=time.monotonic(),
                                                   junction=self.traffic.junction)
            if entry_profile is not None:
                from roundabout_empty import entry_publication_can_commit
                candidate = original.decided(allowed_mps=entry_profile, emergency=False)
                if not entry_publication_can_commit(candidate, ego, time.monotonic()):
                    entry_profile = None
        validation.pop('junction_entry_profile', None)
        validation.pop('junction_raw_alias_keys', None)
        validation.pop('junction_current_semantic_clear_m', None)
        validation.pop('junction_current_contact', None)
        junction_current_contact = False
        standing_path = bool(not tunnel and validation.get('standing_bypass_ids'))
        empty_body = bool(not tunnel and self.traffic is not None and self.traffic.junction
                          and self.traffic.junction.get('empty_actual_body') is True)
        if not tunnel:
            shift = REAR_AXLE_SHIFT_M if standing_path or empty_body or static_rear_axle else ROAD_LEGACY_SHIFT_M
            # Preserve motion histories while checking the selected bypass
            # with the same rear-axle body as its candidate proof.
            self.guard.front_m = EGO_HALF_LENGTH_M+shift
            self.guard.rear_m = EGO_HALF_LENGTH_M-shift
            validation.pop('stationary_semantic_cap_mps', None)
        # The route planner's lead result belongs to its original sensor/pose
        # snapshot. Keep that evidence explicit; the live traffic proof below
        # is computed again for the path and sources of this publication.
        if 'lead' in validation:
            validation['baseline_lead'] = validation.pop('lead')
        valid = bool(ego.get('age_s', 999.) <= .25 and ego.get('stamp', 0.) > 0.)
        if reference_entry_check:
            # The selected world reference keeps its map/body contract when
            # traffic or detector readiness restores the ordinary stop policy.
            reference_valid = bool(empty_path_geometry and certified_pose_geometry(runtime.lanes, ego))
            validation['entry_reference_geometry_valid'] = reference_valid
            valid = valid and reference_valid
        elif selected_reference:
            # Entry permission has been consumed. Following and the full raw
            # actual-body sweep continue; an entry-only map bound must not
            # issue a new stop while already driving through the junction.
            validation.pop('entry_reference_geometry_valid', None)
            validation['entry_reference_committed'] = True
        scan_capture_at = time.monotonic()
        scan = runtime.lidar.scan_snapshot() if runtime.lidar is not None else {}
        stamp = float(scan.get('stamp') or 0.)
        source_lag = float(ego.get('stamp', 0.))-stamp
        scan_age = float(scan.get('age_s', 999.))
        if following_only and (not math.isfinite(scan_age) or scan_age < 0.):
            valid = False
        if capture_age_origin and math.isfinite(scan_age) and scan_age >= 0.:
            scan_age = max(0., scan_age-max(0., scan_capture_at-now))
        age = max(scan_age, max(0., source_lag)+float(ego.get('age_s', 999.)))
        valid = valid and stamp > 0. and -.12 <= source_lag <= .6 and age <= .6
        observation_ego = ego
        aligned = ego
        points, batches = None, []
        road_frame = None
        from static_lane_pass import normal_static_area
        road_scan_required = bool(isinstance(validation.get('road_scan_frame'), dict)
            and not tunnel and not following_only
            and not getattr(runtime, '_transition_entry_obstacles', False)
            and normal_static_area(runtime.lanes, ego))
        if not road_scan_required:
            # An old road plan can reach a portal or junction between model
            # ticks. Its marker cannot replace that area's dedicated source
            # frame or leave raw/scan_pose undefined for its later checks.
            validation.pop('road_scan_frame', None)
        sensor_times = dict(ego=ego.get('stamp'), lidar=stamp)
        if tunnel:
            observation_ego = runtime.tunnel_odometry.at_stamp(float(ego.get('stamp', 0.)))
            scan_pose = runtime.tunnel_odometry.at_stamp(stamp)
            base_ego = original.control_ego if original is not None else current
            base_pose = runtime.tunnel_odometry.at_stamp(float(base_ego.get('stamp', 0.)))
            valid = valid and observation_ego is not None and scan_pose is not None and base_pose is not None
            if valid:
                matched = original.validation_ego if original is not None else current
                aligned = propagate_match(matched, base_pose, observation_ego)
        elif road_scan_required:
            from perception.road_scan_frame import align_road_scan_wait
            road_frame = align_road_scan_wait(scan, ego,
                getattr(runtime.ego_receiver, 'measured_xy_at_stamp', None),
                captured_at=scan_capture_at)
            validation['road_scan_frame'] = road_frame['proof']
            valid = valid and road_frame['proof']['valid']
            frame_age = float(road_frame['proof'].get('age_s', age))
            if static_source_profile:
                frame_age = max(0., frame_age-max(0., time.monotonic()-now))
            age = max(age, frame_age)
            if valid:
                points = road_frame['points']
        else:
            scan_pose = runtime.ego_receiver.at_stamp(stamp)
            valid = valid and scan_pose is not None
        if valid and road_frame is None:
            raw = (scan.get('obstacle_points') if not tunnel and 'obstacle_points' in scan
                   else runtime.fallback_obstacle_points(scan))
            # Revalidate the same obstacle extraction as the selected tunnel
            # path. Elevated tunnel surfaces are checked as walls/width, rather
            # than reintroduced as a second set of apparent moving blockers.
            context = getattr(runtime, '_live_tunnel_corridor', None) if tunnel else None
            if (context is not None and context.get('wall_stats') is not None and
                    hasattr(runtime.tunnel, 'project_scan')):
                scan_aligned = propagate_match(matched, base_pose, scan_pose)
                projected_scan = runtime.tunnel.project_scan(scan['xyz'], scan_aligned,
                    scan['ground_plane'], context['wall_stats'])
                if projected_scan.get('valid'):
                    raw = projected_scan['obstacle_points']
            if raw is None:
                valid = False
            else:
                points = points_at_pose(raw, scan_pose, observation_ego)
        receiver = runtime.object_receiver
        if receiver is not None:
            object_capture_at = time.monotonic()
            if hasattr(receiver, 'snapshot_with_time'):
                objects, object_stamp, object_age = receiver.snapshot_with_time()
                object_stamp, object_age = float(object_stamp or 0.), float(object_age)
            else:
                objects = receiver.snapshot()
                object_age = float(getattr(receiver, 'age_s', 999.))
                object_stamp = float(getattr(receiver, 'observation_stamp', 0.) or 0.)
            if capture_age_origin and math.isfinite(object_age) and object_age >= 0.:
                # Snapshot ages refer to their individual capture times. The
                # later certificate adds elapsed time from this call's `now`;
                # first put each age on that same origin to avoid counting
                # work before the snapshot twice. Taking the time before the
                # receiver call conservatively includes work inside it.
                object_age = max(0., object_age-max(0., object_capture_at-now))
            sensor_times['objects'] = object_stamp
            if 0. <= object_age <= .6 and object_stamp > 0.:
                if tunnel:
                    # This competition tunnel contains static obstacles only.
                    # Both ego and object must be inside the surveyed tunnel;
                    # portal approaches and exits keep their existing guard.
                    # Raw returns and measured LiDAR movers still
                    # independently check the exact published path below.
                    inside_tunnel = bool(runtime.tunnel is not None and runtime.tunnel.active(ego))
                    static_scope = [bool(inside_tunnel and
                        runtime.tunnel.active(dict(x=float(o[0]), y=float(o[1])))) for o in objects]
                    source_ego = runtime.ego_receiver.at_stamp(object_stamp)
                    source_odometry = runtime.tunnel_odometry.at_stamp(object_stamp)
                    if source_ego is not None and source_odometry is not None:
                        objects = objects_at_pose(objects, source_ego, source_odometry)
                    else:
                        objects = []
                    # As in the tunnel planner, motion comes from measured LiDAR only;
                    # fused bodies stay obstacles (follow-20261010T062150Z 281.7 s:
                    # fusion_npc "moving" 9 m ahead beside a parked car, 6 km/h cap).
                    objects = [static_body(o, keep_person=not inside)
                               for o, inside in zip(objects, static_scope)]
                    for i, (obj, inside) in enumerate(zip(objects, static_scope)):
                        if inside and 'cycl' in str(obj[3]).lower():
                            row = list(obj)
                            row[3] = 'tunnel_semantic_static'
                            objects[i] = tuple(row)
                    validation['tunnel_static_semantic_ids'] = [o[11] for o, inside in
                        zip(objects, static_scope) if inside and len(o) > 11]
                else:
                    if hasattr(runtime, 'standing_obstacle_objects'):
                        # Read current positions/speeds against the road proof.
                        # Restart or stale evidence restores the human guard on
                        # this timer tick, without waiting for camera inference.
                        objects = runtime.standing_obstacle_objects(objects, now)
                        if any(o[4] == 'obstacle' and len(o)>17 and isinstance(o[17], dict)
                               and o[17].get('pedestrian_static_reason') and
                               -(EGO_HALF_LENGTH_M-REAR_AXLE_SHIFT_M)-.5 <
                                   ((np.asarray(o[:2])-[ego['x'], ego['y']])@rotation(ego))[0] < 30.
                               for o in objects):
                            baseline = original.allowed_mps if original is not None else runtime.path_publisher.zone_mps
                            validation['stationary_semantic_cap_mps'] = min(float(baseline), 5./3.6)
                    # A person's own returns grown into a car box move with them;
                    # the person row owns that body (ped_guard.person_vehicle_ghosts).
                    if not hasattr(self, 'person_ghosts'):
                        self.person_ghosts = PersonGhostFilter()
                    objects = self.person_ghosts(objects, ego, now)
                batches.append(('tunnel_semantic' if tunnel else 'semantic', objects, object_stamp, object_age))
            elif not tunnel and runtime.clearance != 'lidar':
                valid = False
        traffic_cars, vehicle_ready, tunnel_cars = [], False, None
        yield_source_frame = None
        empty_vehicle_age, empty_scan_age, empty_vehicle_lag = 999., 999., math.inf
        empty_vehicle_observation = False
        vehicle_receiver = getattr(runtime, 'vehicle_receiver', None)
        if vehicle_receiver is not None:
            vehicle_capture_at = time.monotonic()
            cars, ready, status = vehicle_receiver.snapshot()
            # An off-road filter is sufficient for normal gap following, but
            # cannot certify that no vehicle was observed in this fresh frame.
            if not tunnel:
                empty_vehicle_observation = bool(isinstance(status.get('vehicles'), list)
                    and not status['vehicles'] and not cars
                    and not any(status.get(key) for key in ('ghost_ids', 'unknown_ids', 'inherited_ids')))
            vehicle_stamp = float(status.get('stamp') or 0.)
            reported_vehicle_age = float(status.get('age_s', 999.))
            if following_only and (not math.isfinite(reported_vehicle_age) or reported_vehicle_age < 0.):
                ready = False
            if capture_age_origin and math.isfinite(reported_vehicle_age) and reported_vehicle_age >= 0.:
                reported_vehicle_age = max(0., reported_vehicle_age-max(0., vehicle_capture_at-now))
            vehicle_age = max(reported_vehicle_age,
                              max(0., float(ego.get('stamp', 0.))-vehicle_stamp))
            sensor_times['vehicles'] = vehicle_stamp
            if timed_vehicles and not recovery_requested and self.traffic.junction.get('yield_only'):
                from junction_source_guard import frame_identity
                yield_source_frame = frame_identity(status, ready)
                if yield_source_frame is None:
                    if not committed:
                        ready = False
                elif (self.vehicle_stamp == vehicle_stamp
                        and (getattr(self, '_junction_vehicle_frame', None) or {}).get('stamp') == vehicle_stamp
                        and self._junction_vehicle_frame != yield_source_frame):
                    # The tracker caches by source stamp. Do not authorize a
                    # revised body against cached geometry from that stamp;
                    # wait for a newer complete source instead of reinserting
                    # a same-time observation into its motion history.
                    ready = valid = False
                    validation['junction_source_same_stamp_revision'] = True
                else:
                    # Keep the identity paired with cached source geometry
                    # through commitment and an interrupted preline launch.
                    # An older preentry identity is not a same-time revision.
                    self._junction_vehicle_frame = yield_source_frame
            if ready and vehicle_stamp > 0. and vehicle_age <= .6:
                # Use the route planner's existing off-road false-car filter.
                # It rejects the roundabout island boxed as a parked car;
                # every actual LiDAR return remains in the body sweep below.
                if not tunnel:
                    cars = runtime.on_road_vehicles(cars)
                if tunnel:
                    source_ego = runtime.ego_receiver.at_stamp(vehicle_stamp)
                    source_odometry = runtime.tunnel_odometry.at_stamp(vehicle_stamp)
                    cars = (objects_at_pose(cars, source_ego, source_odometry)
                            if source_ego is not None and source_odometry is not None else [])
                    tunnel_cars = (len(batches), cars, vehicle_stamp, vehicle_age)
                    cars = self.measured_tunnel_cars(cars, vehicle_stamp)
                # Detector dropout boxes are already predicted to its source
                # stamp. Their original observation age remains a risk margin.
                batches.append(('vehicles', cars, vehicle_stamp, vehicle_age))
                if timed_vehicles:
                    traffic_cars = self.junction_vehicles(cars, vehicle_stamp, vehicle_age, now, ego)
                    if following_only or yield_moving_bodies:
                        # Preserve source positions/yaw/stamp while binding
                        # robust measured motion to the fresh vehicle batch.
                        batches[-1] = ('vehicles', self.junction_measured_rows,
                                       vehicle_stamp, vehicle_age)
                    vehicle_ready = True
                    entry_pose_age = max(float(ego.get('age_s', 999.)),
                                         now-float(ego.get('received', now)))
                    empty_vehicle_lag = float(ego.get('stamp', 0.))-vehicle_stamp
                    empty_vehicle_age = max(vehicle_age, max(0., empty_vehicle_lag)+entry_pose_age)
                    empty_scan_age = max(age, max(0., source_lag)+entry_pose_age)
        if valid and (following_only or yield_moving_bodies) and vehicle_receiver is not None and not _source_retry:
            # A complete detector result can arrive during source preparation.
            # Restart once before map/body assessment, rather than issuing a
            # stop for the superseded frame's short remaining lifetime. The
            # replacement receives all geometry and safety checks of its own.
            _, latest_ready, latest_status = vehicle_receiver.snapshot()
            latest_stamp = float(latest_status.get('stamp') or 0.)
            peek_now = time.monotonic()
            clock_valid = math.isfinite(peek_now) and peek_now >= call_started_at
            if clock_valid and latest_ready and math.isfinite(latest_stamp) and latest_stamp > vehicle_stamp:
                prep_ms = (peek_now-call_started_at)*1000.
                checked = self(world_path, current, mode, original, _source_retry=True)
                compute = {key: value+prep_ms for key, value in
                           checked.validation.get('junction_compute_ms', {}).items()}
                return replace(checked, validation=dict(checked.validation,
                    junction_source_retry=dict(from_stamp=vehicle_stamp, to_stamp=latest_stamp,
                                               preparation_ms=prep_ms),
                    junction_compute_ms=compute))
            if not clock_valid or not latest_ready or not math.isfinite(latest_stamp) or latest_stamp < vehicle_stamp:
                vehicle_ready = False
        if valid and (following_only or yield_moving_bodies) and vehicle_ready:
            # Source preparation can span a feedback frame. Capture the pose
            # used by timing after that work, rebuilding all pose-dependent
            # geometry and source alignment before any permission is issued.
            refreshed_at = time.monotonic()
            refreshed, _, _ = runtime.ego_receiver.snapshot(1)
            try:
                refreshed = dict(refreshed or {})
                fields = ('x', 'y', 'heading_deg', 'speed_kmh', 'stamp', 'age_s')
                values = np.asarray([refreshed[key] for key in fields], float)
                dt = float(refreshed['stamp'])-float(ego['stamp'])
                distance = float(np.linalg.norm(values[:2]-[ego['x'], ego['y']]))
                speed = max(abs(float(ego['speed_kmh'])), abs(float(refreshed['speed_kmh'])))/3.6
                speed_delta = abs(float(refreshed['speed_kmh'])-float(ego['speed_kmh']))/3.6
                yaw_delta = abs((math.radians(float(refreshed['heading_deg'])-
                    float(ego['heading_deg']))+math.pi) % (2*math.pi)-math.pi)
                same = dt > 1e-6 or (yaw_delta <= math.radians(.001) and
                    all(abs(float(refreshed.get(key, 0.))-float(ego.get(key, 0.))) <= 1e-6
                        for key in ('x', 'y', 'speed_kmh', 'accel', 'brake')))
                refreshed_valid = bool(np.isfinite(values).all() and refreshed_at >= now
                    and 0. <= dt <= .35 and 0. <= refreshed['age_s'] <= .2
                    and distance <= speed*dt+.25 and speed_delta <= 12.*dt+.3
                    and yaw_delta <= 1.5*dt+.1 and same)
            except (KeyError, TypeError, ValueError, OverflowError):
                refreshed_valid = False
            if not refreshed_valid:
                valid = False
            else:
                elapsed = max(0., refreshed_at-now)
                batches = [(source, rows, source_stamp, source_age+elapsed)
                           for source, rows, source_stamp, source_age in batches]
                now = refreshed_at
                ego = refreshed
                ego.pop('_junction_launch', None)
                observation_ego = aligned = ego
                path = (np.asarray(world_path)-[ego['x'], ego['y']])@rotation(ego)
                points = points_at_pose(raw, scan_pose, ego)
                sensor_times['ego'] = ego['stamp']
                pose_age = max(float(ego['age_s']), now-float(ego.get('received', now)))
                source_lag = float(ego['stamp'])-stamp
                age = max(age+elapsed, max(0., source_lag)+pose_age)
                empty_vehicle_lag = float(ego['stamp'])-vehicle_stamp
                vehicle_age = max(vehicle_age+elapsed, max(0., empty_vehicle_lag))
                batches = [(source, rows, source_stamp,
                            max(source_age, vehicle_age) if source == 'vehicles' else source_age)
                           for source, rows, source_stamp, source_age in batches]
                # The same source stamp only reprojects its stored rows. It
                # must not add another motion observation or refresh a ghost.
                traffic_cars = self.junction_vehicles(cars, vehicle_stamp, vehicle_age, now, ego)
                empty_vehicle_age = max(vehicle_age, max(0., empty_vehicle_lag)+pose_age)
                empty_scan_age = age
                valid = bool(0. <= pose_age <= .2 and -.12 <= source_lag <= .6 and age <= .6)
                if runtime.clearance != 'lidar':
                    valid = valid and all(0. <= source_age <= .6
                        for source, _, _, source_age in batches if source == 'semantic')
                if selected_reference:
                    valid = valid and bool(certified_pose_geometry(runtime.lanes, ego))
        recovery_proof = None
        if valid and timed_vehicles and recovery is not None:
            checked_now = time.monotonic()
            elapsed = max(0., checked_now-now)
            proof_ego = dict(ego, age_s=float(ego.get('age_s', math.inf))+elapsed)
            context = recovery.context(proof_ego, getattr(runtime, 'stop_line_controller', None),
                runtime.lanes, getattr(runtime, 'intersection_regions', ()), checked_now)
            if context is not None and context['line_id'] == self.traffic.junction.get('line_id'):
                recovery_requested = True
                self.traffic.junction = dict(self.traffic.junction, recovery_only=True)
                original_valid = bool(original is not None and original.fresh(checked_now)
                    and original.checks_match_path() and not original.emergency
                    and all(validation.get(key) for key in ('valid', 'pose_valid', 'steering_valid'))
                    and not validation.get('blocked'))
                recovery_proof = recovery.prepare(proof_ego, world_path,
                    getattr(runtime, 'stop_line_controller', None), runtime.lanes,
                    getattr(runtime, 'intersection_regions', ()), checked_now,
                    ready=bool(original_valid and vehicle_ready and -.12 <= empty_vehicle_lag <= .35),
                    source_ages=[empty_vehicle_age+elapsed, empty_scan_age+elapsed]
                        +[source_age+elapsed for _, _, _, source_age in batches])
                if recovery_proof is not None:
                    validation['junction_recovery'] = dict(recovery_proof)
        if timed_vehicles:
            # Match one camera vehicle to one measured box, as in the original
            # junction pipeline. People and other obstacles keep their checks.
            predicted_ids = set(self.junction_tracks.ghost_ids)|set(self.junction_tracks.inherited_ids)
            timed_body_ids = {(o[3], o[11]) for o in traffic_cars if len(o)>14 and o[14]
                              and o[4]=='npc' and float(o[5])>.5 and o[11] not in predicted_ids
                              and (len(o)<=15 or float(o[15])<=0.)}
            unused = set(range(len(traffic_cars)))
            for _, objects, _, object_age in batches:
                for obj in objects:
                    if obj[4] != 'npc' or obj[3] == 'lidar_car':
                        continue
                    match = min(unused, key=lambda i: np.linalg.norm(np.asarray(obj[:2])-traffic_cars[i][:2])) if unused else None
                    if match is not None and np.linalg.norm(np.asarray(obj[:2])-traffic_cars[match][:2]) < 2.:
                        unused.remove(match)
                        measured = traffic_cars[match]
                        if (measured[3], measured[11]) in timed_body_ids:
                            timed_body_ids.add((obj[3], obj[11]))
                    else:
                        row = list(obj)
                        row[0] += row[9]*object_age
                        row[1] += row[10]*object_age
                        traffic_cars.append(tuple(row))
        raw_clear = None
        forecast_raw_clear = None
        empty_raw_clear = None
        motion_proof = None
        yield_motion_proof = None
        yield_motion_evaluation = {}
        motion_keys = frozenset()
        motion_profile_mps = None
        motion_requested_mps = None
        motion_evaluation = {}
        if valid and yield_motion_entry and vehicle_ready:
            from junction_vehicle_motion import certify_moving_cars, proof_matches
            checked_now = time.monotonic()
            elapsed = max(0., checked_now-now)
            requested = entry_profile if entry_profile is not None else (
                original.allowed_mps if original is not None else float(runtime.path_publisher.zone_mps))
            # The GUI's earlier front station cannot own a current-source
            # launch assessment. Use the publication's measured pose once.
            self.traffic.junction = dict(self.traffic.junction, held=True,
                line_gap_m=float(self.traffic.junction['line_station_m'])
                    - intersection_front_station(ego, runtime.lanes.route_progress))
            yield_motion_proof = certify_moving_cars(self.traffic, ego, world_path,
                traffic_cars, [(source, rows, source_stamp, source_age+elapsed)
                              for source, rows, source_stamp, source_age in batches],
                rtf=self.motion_rtf(observation_ego, checked_now), now=checked_now,
                vehicle_age_s=empty_vehicle_age+elapsed, lidar_age_s=empty_scan_age+elapsed,
                requested_mps=requested, profile_ceiling_mps=validation.get('junction_body_cap_mps'),
                excluded_uids=predicted_ids, evaluation_out=yield_motion_evaluation)
            if not proof_matches(yield_motion_proof, ego, world_path, now=time.monotonic()):
                yield_motion_proof = None
            if yield_motion_proof is not None:
                profile = validation.get('junction_body_cap_mps')
                yield_motion_evaluation.update(proof=yield_motion_proof,
                    profile_mps=float(requested if profile is None else profile))
                validation['junction_yield_motion'] = dict(
                    covered_keys=sorted(yield_motion_proof.covered_keys),
                    current_clear_m=yield_motion_proof.current_clear_m,
                    expires_at=yield_motion_proof.expires_at)
        junction_compute = dict(source=(time.monotonic()-call_started_at)*1000.) if following_only else None
        if valid and following_only and vehicle_ready and not recovery_requested:
            from junction_vehicle_motion import certify_moving_cars, proof_matches
            checked_now = time.monotonic()
            capture_elapsed = max(0., checked_now-now)
            motion_rtf = self.motion_rtf(observation_ego, checked_now)
            launch = self.junction_launch.prediction(world_path, ego, checked_now, motion_rtf)
            validation['junction_launch_status'] = dict(active=self.junction_launch.active,
                                                        reason=self.junction_launch.reason)
            if launch is not None:
                ego['_junction_launch'] = launch
                validation['junction_launch'] = dict(launch)
            checked_batches = [(source, objects, source_stamp, source_age+capture_elapsed)
                for source, objects, source_stamp, source_age in batches]
            motion_proof = certify_moving_cars(self.traffic, ego, world_path,
                traffic_cars, checked_batches, rtf=motion_rtf,
                now=checked_now, vehicle_age_s=empty_vehicle_age+capture_elapsed,
                lidar_age_s=empty_scan_age+capture_elapsed,
                requested_mps=(original.allowed_mps if original is not None
                               else float(runtime.path_publisher.zone_mps)),
                profile_ceiling_mps=validation.get('junction_body_cap_mps'),
                excluded_uids=predicted_ids, evaluation_out=motion_evaluation)
            proof_checked_at = time.monotonic()
            proof_valid = bool(motion_proof is not None and
                proof_matches(motion_proof, ego, world_path, now=proof_checked_at))
            validation['junction_motion_check'] = dict(returned=motion_proof is not None,
                valid=proof_valid, remaining_ms=(motion_proof.expires_at-proof_checked_at)*1000.
                if motion_proof is not None else None)
            if proof_valid:
                motion_keys = motion_proof.covered_keys
                motion_requested_mps = (original.allowed_mps if original is not None
                                        else float(runtime.path_publisher.zone_mps))
                profile = validation.get('junction_body_cap_mps')
                motion_profile_mps = min(INTERSECTION_CONTACT_MPS,
                    float(profile) if profile is not None else motion_requested_mps)
                validation['junction_motion'] = dict(
                    covered_keys=sorted(motion_keys), current_clear_m=motion_proof.current_clear_m,
                    cap_mps=motion_proof.cap_mps, expires_at=motion_proof.expires_at)
            else:
                motion_proof = None
        if following_only:
            junction_compute['timing_done'] = (time.monotonic()-call_started_at)*1000.
        moving_body_keys = frozenset()
        moving_body_owners = {}
        if valid and (following_only or yield_moving_bodies):
            uncertain = set().union(*(set(self.traffic.junction.get(field) or ())
                for field in ('unknown_ids', 'inherited_ids', 'ghost_ids')))
            moving_body_keys = junction_moving_body_keys(batches, uncertain, moving_body_owners)
            validation['junction_moving_body_keys'] = sorted(moving_body_keys)
        if valid:
            body_motion_proof = motion_proof or yield_motion_proof
            body_motion_keys = body_motion_proof.covered_keys if body_motion_proof is not None else frozenset()
            side = float(validation.get('side_margin_m', .25))
            end = float(validation.get('end_margin_m', .25))
            projected = int(np.argmin(np.linalg.norm(path, axis=1)))
            forward = path[projected:]
            if len(forward) < 3:
                valid = False
            else:
                # Keep moving bodies in the forecast; only their associated
                # measured returns can leave the static raw-point check.
                from lidar_corridor import without_moving_objects
                moving, portal_moving = [], []
                raw_alias_keys = set()
                for _, objects, _, obj_age in batches:
                    for obj in objects:
                        covered_alias = bool(len(obj)>14 and obj[4] == 'npc' and obj[3] != 'lidar_car'
                            and ((body_motion_proof is not None
                                  and (str(obj[3]), obj[11]) in body_motion_keys)
                                 or (yield_moving_bodies
                                     and (str(obj[3]), obj[11]) in moving_body_keys)))
                        if ((obj[4] == 'pedestrian' and obj[5] >= .8) or
                                (obj[4] == 'npc' and len(obj)>14
                                 and (obj[14] or covered_alias) and obj[5] > .5)):
                            values = list(obj)
                            values[0] += values[9]*obj_age
                            values[1] += values[10]*obj_age
                            moving.append(tuple(values))
                            if covered_alias:
                                raw_alias_keys.add((str(obj[3]), obj[11]))
                            if obj_age <= .3 and (obj[3] != 'lidar_car' or
                                                  len(obj) <= 15 or obj[15] <= 0.):
                                portal_moving.append(tuple(values))
                if following_only:
                    # These camera rows use their original observed bodies,
                    # motion and age. Only this call's measured-car proof can
                    # associate their returns; current raw contact below still
                    # uses the complete unmasked scan.
                    validation['junction_raw_alias_keys'] = sorted(raw_alias_keys)
                if tunnel:
                    # Use the tunnel's measured-return memory in the live
                    # sweep as well. A low barrier entering the LiDAR blind
                    # zone cannot disappear from the currently issued plan.
                    if runtime.tunnel is not None and runtime.tunnel.active(ego):
                        self.near_field.observe_scan(scan, scan_pose,
                            getattr(runtime.lidar, 'mount', (1.05, 0., 1.35)))
                    else:
                        self.near_field.stop_observing()
                    points = self.near_field.update(points, observation_ego, stamp, moving)
                    validation['tunnel_memory_cleared_total'] = self.near_field.cleared_total
                elif getattr(runtime, '_transition_entry_obstacles', False):
                    measured = update_approach_points(runtime, scan, raw, observation_ego, portal_moving)
                    validation['portal_memory'] = getattr(runtime, '_portal_memory_status', None)
                    if measured is None:
                        valid, points = False, np.empty((0, 2))
                    else:
                        points = measured
                elif hasattr(runtime, '_portal_approach_memory'):
                    runtime._portal_approach_memory.reset()
                static_points = without_moving_objects(points, moving, observation_ego)
                if tunnel:
                    # Give the search the same measured blind-zone returns
                    # that can stop the issued path. Keep the odometry frame
                    # and source time; sharing a snapshot avoids timer races.
                    runtime._live_tunnel_obstacles = dict(points=static_points.copy(),
                        ego=dict(observation_ego), stamp=stamp, received=now)
                if not tunnel:
                    body_shift = REAR_AXLE_SHIFT_M if standing_path or empty_body or static_rear_axle else ROAD_LEGACY_SHIFT_M
                    raw_clear = (lattice_swept_clearance(forward, static_points, 60., side, end,
                                     body_shift_m=body_shift)
                                 if mode == 'LATTICE' or validation.get('generator') == 'lattice' else
                                 swept_clearance(forward, static_points, 60., end, lateral_margin=side,
                                     body_shift_m=body_shift))
                    validation['junction_raw_static_clear_m'] = raw_clear if following_only else None
                    if (timed_vehicles or motion_proof is not None) and inside_body(points, margin=.03).any():
                        # Moving returns may leave the future static sweep,
                        # but current measured physical contact never does.
                        raw_clear = 0.
                        junction_current_contact = True
                    if empty_body and inside_body(static_points, margin=.03).any():
                        # Current measured heading can differ from the path's
                        # tangent. Also check its actual t=0 rear-axle body.
                        raw_clear = 0.
                    forecast_raw_clear = raw_clear
                    if (empty_body and self.traffic.status.get('state') == 'APPROACH_EMPTY'
                            and abs(float(ego.get('speed_kmh', 0.)))/3.6 <= ROLL_MPS):
                        front = intersection_front_station(ego, runtime.lanes.route_progress)
                        line = self.traffic.junction.get('line_station_m')
                        if line is not None and float(line)-front <= ROLL_CHECK_FRONT_GAP_M:
                            empty_raw_clear = raw_entry_clearance(runtime.lanes, ego, static_points, world_path)
                            validation['empty_entry_raw_clear_m'] = empty_raw_clear
                    # Keep the same semantic footprint check before and after
                    # commitment; measured boxes do not vanish at ENTER.
                    empty_static = empty_body
                    if standing_path or empty_static:
                        from model_avoidance import standing_static_clear_distance, is_static, wall_like
                        static_boxes, forecast_boxes = [], []
                        for _, objects, _, obj_age in batches:
                            for obj in objects:
                                if empty_static and (len(obj) <= 10 or
                                        not np.isfinite(np.asarray([obj[i] for i in
                                            (0, 1, 5, 6, 7, 8, 9, 10)], float)).all()
                                        or float(obj[5]) < 0. or min(float(obj[6]), float(obj[7])) <= 0.):
                                    # A malformed body must not disappear from
                                    # both static and measured-motion checks.
                                    raw_clear = 0.
                                    forecast_raw_clear = 0.
                                    validation['empty_entry_invalid_semantic'] = True
                                    junction_current_contact = bool(timed_vehicles)
                                    continue
                                covered_body = bool(body_motion_proof is not None and len(obj)>11
                                    and (str(obj[3]), obj[11]) in body_motion_keys and 0. <= obj_age <= .35
                                    and (obj[4] == 'npc' or
                                         (obj[4] == 'obstacle' and obj[3] == 'fusion_parked_vehicle')))
                                if timed_vehicles and (obj[4] in ('npc', 'pedestrian') or covered_body):
                                    from junction_vehicle_motion import _separation
                                    contact_body, contact_age = obj, obj_age
                                    if yield_moving_bodies and len(obj)>11:
                                        owner = moving_body_owners.get((str(obj[3]), obj[11]))
                                        if owner is not None:
                                            # Fuse the duplicate camera size prior with its
                                            # fresh measured PP footprint. An inflated camera
                                            # width must not invent contact beside a real car.
                                            # Its measured body and every raw t=0 return stay.
                                            contact_body, contact_age = owner
                                            validation.setdefault('junction_current_body_aliases', []).append(
                                                [str(obj[3]), obj[11], contact_body[11]])
                                    current = (np.asarray(contact_body[:2])
                                               + np.asarray(contact_body[9:11])*contact_age)
                                    separation = min(_separation(ego, contact_body, contact_body[:2]),
                                                     _separation(ego, contact_body, current))
                                    validation['junction_current_semantic_clear_m'] = min(
                                        validation.get('junction_current_semantic_clear_m', math.inf), separation)
                                    if not math.isfinite(separation) or separation <= .1:
                                        # Every current NPC body remains real,
                                        # independently of its future owner.
                                        raw_clear = forecast_raw_clear = 0.
                                        junction_current_contact = True
                                if covered_body:
                                    # The same fresh map proof owns its future
                                    # movement; its actual t0 OBB was checked.
                                    continue
                                if ((following_only or yield_moving_bodies) and len(obj)>11 and
                                        (str(obj[3]), obj[11]) in moving_body_keys and 0. <= obj_age <= .35):
                                    # The full forecast remains with PathSafety
                                    # or the junction traffic model. Freezing
                                    # this moving box at every future station
                                    # would manufacture a second stop, including
                                    # after a ring car has passed the conflict.
                                    continue
                                if empty_static or (is_static(obj) and not wall_like(obj)):
                                    # The current footprint is real even while
                                    # motion confirmation holds its first report.
                                    if empty_static:
                                        static_boxes.append(obj)
                                    row = list(obj)
                                    row[0] += row[9]*obj_age
                                    row[1] += row[10]*obj_age
                                    static_boxes.append(tuple(row))
                                    # The measured ring trajectory owns this
                                    # moving car in the future. Its current body
                                    # still limits the emitted command below.
                                    timed_body = bool(timed_vehicles and not committed and not recovery_requested
                                        and self.traffic.junction.get('yield_only')
                                        and obj_age <= .35 and obj[4]=='npc'
                                        and (obj[3], obj[11]) in timed_body_ids)
                                    if not timed_body:
                                        forecast_boxes.extend((obj, tuple(row)))
                        body_clear = (semantic_entry_clearance(forward, ego, static_boxes) if empty_static
                                      else standing_static_clear_distance(forward, ego, static_boxes))
                        validation['empty_entry_semantic_clear_m' if empty_static else 'standing_static_clear_m'] = body_clear
                        raw_clear = min(raw_clear, body_clear)
                        if (timed_vehicles and not committed and not recovery_requested
                                and self.traffic.junction.get('yield_only')):
                            forecast_raw_clear = min(forecast_raw_clear,
                                semantic_entry_clearance(forward, ego, forecast_boxes))
                            validation['junction_nontraffic_clear_m'] = forecast_raw_clear
                else:
                    raw_clear = None
                if tunnel:
                    # Raw walls stay in the body sweep; mapped walls and the
                    # already selected road-width corridor keep their own checks.
                    context = getattr(runtime, '_live_tunnel_corridor', None)
                    corridor = None
                    if context is not None:
                        corridor = corridor_at_pose(context['corridor'], context['ego'], aligned)
                    # Reuse the planner's checks on this same final path.
                    if hasattr(runtime, 'tunnel_path_checks'):
                        components = runtime.tunnel_path_checks(forward, aligned,
                            static_points, corridor, validation=validation)
                        if validation.get('mapped_walls', True) and 'wall' not in components:
                            components['wall'] = runtime.tunnel.path_clearance(forward, aligned, 60.)
                        validation.update(components)
                        raw_clear = min(components.values())
                        validation['geometry_clearance_m'] = raw_clear
                    else:
                        raw_clear = swept_clearance(forward, static_points, 60., end, lateral_margin=side,
                                                    body_shift_m=REAR_AXLE_SHIFT_M)
                        if validation.get('mapped_walls', True):
                            raw_clear = min(raw_clear, runtime.tunnel.path_clearance(forward, aligned, 60.))
                        if corridor is not None:
                            raw_clear = min(raw_clear, corridor.clear_distance(forward, 60.))
                    # Track only compact corridor clusters, never wall motion.
                    from tunnel_runtime import corridor_points
                    clustered = corridor_points(points, corridor) if corridor is not None else points
                    if self.cluster_stamp != stamp:
                        observed = lidar_objects(clustered, observation_ego)
                        self.cluster_objects = self.clusters.update(observed, stamp)
                        self.cluster_stamp = stamp
                    if tunnel_cars is not None:
                        index, cars, vehicle_stamp, vehicle_age = tunnel_cars
                        batches[index] = ('vehicles', self.measured_tunnel_cars(cars, vehicle_stamp),
                                          vehicle_stamp, vehicle_age)
                    tracked = self.cluster_objects
                    # Semantic/full-size boxes own their associated clusters;
                    # unknown unassociated LiDAR objects keep motion warmup.
                    # A measured mover is not owned by a body without motion.
                    known = [o for _, objects, _, _ in batches for o in objects]
                    tracked = [o for o in tracked if max(o[6:8]) <= 8. and
                        not any(np.linalg.norm(np.asarray(o[:2])-k[:2]) < max(1., .5*float(k[2]))
                                for k in known if o[4] != 'pedestrian' or float(k[5]) >= .3)]
                    batches.append(('tunnel_lidar', tracked, stamp, age))
        result = None
        baseline_cap = original.allowed_mps if original is not None else float(runtime.path_publisher.zone_mps)
        sensor_recheck = validation.pop('object_stream_recheck', None)
        sensor_recovered = False
        if (not tunnel and not timed_vehicles and original is not None
                and original.mode == 'ROUTE BASE' and original.emergency
                and original.stop_reason in ('object stream stale', 'object stream stale; objects stale')
                and isinstance(sensor_recheck, dict) and valid
                and validation.get('blocked') is False):
            try:
                ceiling = float(sensor_recheck['ceiling_mps'])
                previous_stamp = float(sensor_recheck['source_stamp'])
                sensor_recovered = bool(math.isfinite(ceiling) and ceiling > 0.
                    and math.isfinite(previous_stamp) and previous_stamp > 0.
                    and object_stamp > previous_stamp and 0. <= object_age <= .5)
                if sensor_recovered:
                    baseline_cap = ceiling
                    validation['object_stream_recovered'] = dict(
                        previous_stamp=previous_stamp, source_stamp=object_stamp)
            except (TypeError, ValueError, KeyError, OverflowError):
                sensor_recovered = False
        baseline_cap = min(baseline_cap, validation.get('stationary_semantic_cap_mps', math.inf))
        static_motion = bool(original is not None and not tunnel and not timed_vehicles
            and not original.emergency and original.allowed_mps > 0.
            and validation.get('static_motion_profile') and validation.get('static_lane_borrow'))
        static_ceiling = None
        for key in ('static_profile_live_ceiling_mps', 'static_profile_live_command_mps', 'static_profile_deadline'):
            validation.pop(key, None)
        if static_motion:
            from static_lane_pass import PASS_MPS, static_speed_profile
            from planning.highway.path_continuity import projected_suffix
            try:
                static_ceiling = float(validation.get('static_profile_ceiling_mps') or PASS_MPS)
                source_limit = validation.get('static_profile_source_limit_mps')
                if source_limit is not None:
                    static_ceiling = min(static_ceiling, float(source_limit))
                if not math.isfinite(static_ceiling) or not 0. < static_ceiling <= PASS_MPS:
                    raise ValueError('static motion source ceiling invalid')
                baseline_cap = min(static_speed_profile(projected_suffix(path), ego,
                    ceiling_mps=static_ceiling)['cap_mps'],
                    validation.get('stationary_semantic_cap_mps', math.inf))
            except (TypeError, ValueError, IndexError, OverflowError):
                valid, baseline_cap, static_ceiling = False, 0., None
        forecast_body_cap = (baseline_cap if entry_profile is None else
            min(entry_profile, validation.get('stationary_semantic_cap_mps', math.inf)))
        if static_ceiling is not None:
            # Body forecasts constrain the complete future profile. Its
            # current wheel/curve command is a separate local speed limit.
            forecast_body_cap = min(static_ceiling,
                validation.get('stationary_semantic_cap_mps', math.inf))
        if entry_profile is not None:
            validation['junction_forecast_body_cap_mps'] = forecast_body_cap
        if motion_proof is not None:
            # A headway cap describes this command, not every future command.
            # The independent raw/person/body check must cover the same launch
            # profile whose rear-clearance time authorized map ownership.
            forecast_body_cap = min(motion_profile_mps,
                validation.get('stationary_semantic_cap_mps', math.inf))
            validation['junction_forecast_body_cap_mps'] = forecast_body_cap
        if (timed_vehicles and self.traffic.junction.get('yield_only') and original is not None
                and validation.get('junction_gap_checked') is True and not original.emergency
                and original.allowed_mps > 0.):
            from roundabout_empty import entry_publication_can_commit
            try:
                independent = float(validation.get('junction_body_cap_mps'))
                if (math.isfinite(independent) and independent >= 0.
                        and entry_publication_can_commit(original, ego, time.monotonic())):
                    forecast_body_cap = min(independent, INTERSECTION_ENTRY_MPS,
                        validation.get('stationary_semantic_cap_mps', math.inf))
                    validation['junction_forecast_body_cap_mps'] = forecast_body_cap
            except (TypeError, ValueError, OverflowError):
                pass
        validation.pop('entry_reference_cap_mps', None)
        if reference_roll_speed:
            # The low-speed certificate authorizes crossing the line only.
            # Retain the fixed reference through an interruption, but let an
            # ordinary stopping approach own its speed instead of crawling.
            baseline_cap = min(baseline_cap, ROLL_TARGET_MPS)
            validation['entry_reference_cap_mps'] = ROLL_TARGET_MPS
        baseline_emergency = original.emergency if original is not None else bool(runtime.path_publisher.emergency_stop)
        if sensor_recovered:
            baseline_emergency = False
        if valid:
            # Native ego speed uses simulation time; displacement velocities
            # use sensor/wall time. PathSafety forecasts in simulator seconds.
            rtf = self.motion_rtf(observation_ego, now)
            # People the decision already waited for and released (gui commit).
            released = getattr(runtime, '_ped_released', None)
            released = released if isinstance(released, dict) else {}
            split_timed_body = bool(timed_vehicles and not committed and not recovery_requested
                and self.traffic.junction.get('yield_only') and forecast_raw_clear is not None)
            result = self.guard.check(path, ego, batches, now, requested_mps=forecast_body_cap,
                raw_clear_m=forecast_raw_clear if split_timed_body else raw_clear,
                source_age_s=age, observation_ego=observation_ego,
                # Without the ring-gap gate this checker must also own NPC
                # forecasts; measured current bodies alone are insufficient.
                timed_vehicles=timed_vehicles and not following_only and not recovery_requested, dynamic_only=True,
                timed_vehicle_keys=motion_keys,
                motion_time_scale=rtf,
                measured_people=self.person_measurements(points, observation_ego, batches, stamp, age, now),
                released_people=() if tunnel else
                    {uid for uid, until in list(released.items()) if until > now})
            remember_pedestrian_brakes(runtime, result.validation, batches, now)
            current_body_cap = math.inf
            if split_timed_body and raw_clear is not None:
                gap = max(0., float(raw_clear)-1.)
                lag = age+self.guard.response_s
                current_body_cap = stop_cap(gap, lag, self.guard.brake_mps2)
                urgent = self.guard.immediate_stop(gap,
                    abs(float(ego.get('speed_kmh', 0.)))/3.6, lag, current_body_cap)
                result.validation['current_body_clear_m'] = raw_clear
                result.validation['current_body_cap_mps'] = current_body_cap
                if urgent:
                    self.guard.stopped, self.guard.clear_since = True, None
                    self.guard.clear_samples.clear()
                    result = replace(result, cap_mps=0., emergency=True,
                        reason='live lidar collision on published path')
            junction_live_deadline = None
            if following_only:
                # All no-line junction commands use this publication's source
                # snapshot. An earlier GUI WAIT/GO is diagnostic evidence only.
                checked_now = time.monotonic()
                junction_compute['body_done'] = (checked_now-call_started_at)*1000.
                pose_age = float(ego.get('age_s', math.inf))+max(0., checked_now-now)
                receipt_age = checked_now-float(ego.get('received', checked_now))
                pose_age = max(pose_age, receipt_age)
                source_age = max(empty_vehicle_age, empty_scan_age)
                junction_live_deadline = min(checked_now+.2-pose_age, now+.35-source_age)
                live_fresh = bool(vehicle_ready and not self.junction_unobserved_prior
                    and -.12 <= empty_vehicle_lag <= .35
                    and math.isfinite(pose_age) and 0. <= receipt_age <= .2
                    and 0. <= pose_age <= .2 and math.isfinite(source_age)
                    and 0. <= source_age <= .35 and now <= checked_now <= junction_live_deadline)
                if live_fresh:
                    validation['junction_live'] = dict(expires_at=junction_live_deadline)
                else:
                    result = replace(result, cap_mps=0., emergency=True,
                        reason='junction publication traffic or pose stale')
            if motion_proof is not None:
                body_cap = min(result.cap_mps, current_body_cap)
                accepted = not result.emergency and math.isfinite(body_cap) and body_cap > 0.
                rejected_reason = None
                if accepted and body_cap < motion_profile_mps-1e-9:
                    # A new nontraffic restriction can make the originally
                    # promised clearance impossible even while its command is
                    # positive. Re-evaluate timing at that restriction without
                    # running the stateful body/motion memories a second time.
                    rechecked_now = time.monotonic()
                    elapsed = max(0., rechecked_now-now)
                    rechecked_batches = [(source, objects, source_stamp, source_age+elapsed)
                        for source, objects, source_stamp, source_age in batches]
                    rechecked = (certify_moving_cars(self.traffic, ego, world_path,
                        traffic_cars, rechecked_batches, rtf=rtf, now=rechecked_now,
                        vehicle_age_s=empty_vehicle_age+elapsed,
                        lidar_age_s=empty_scan_age+elapsed,
                        requested_mps=motion_requested_mps, profile_ceiling_mps=body_cap,
                        excluded_uids=predicted_ids)
                        if proof_matches(motion_proof, ego, world_path, now=rechecked_now) else None)
                    accepted = (rechecked is not None and rechecked.covered_keys == motion_keys
                        and proof_matches(rechecked, ego, world_path, now=time.monotonic()))
                    if accepted:
                        # Rechecking the same observations cannot extend the
                        # first proof's deadline, including its pose deadline.
                        motion_proof = replace(rechecked,
                            expires_at=min(motion_proof.expires_at, rechecked.expires_at))
                        validation['junction_motion'].update(
                            cap_mps=motion_proof.cap_mps, expires_at=motion_proof.expires_at,
                            profile_ceiling_mps=body_cap)
                    result.validation['junction_profile_recheck'] = dict(
                        profile_mps=body_cap, accepted=bool(accepted))
                if accepted and motion_proof.required_rear_travel_m > 0.:
                    # A positive stopping cap is not a promise to pass a
                    # static obstruction. Incoming timing also needs enough
                    # axle travel to clear the ego's rear before that stop.
                    # PathSafety already subtracts its stand-off from semantic
                    # gaps, but reports raw footprint contact distance without
                    # the raw branch's 1 m stand-off. Covered moving cars are
                    # excluded from both independent clearances above.
                    travel_clear = min(result.clear_m,
                        max(0., float(raw_clear)-1.) if raw_clear is not None else 0.)
                    accepted = bool(math.isfinite(travel_clear) and
                        travel_clear >= motion_proof.required_rear_travel_m)
                    result.validation['junction_rear_travel'] = dict(
                        required_m=motion_proof.required_rear_travel_m,
                        clear_m=travel_clear, accepted=accepted)
                    if not accepted:
                        rejected_reason = 'junction rear clearance blocked before incoming traffic'
                if accepted:
                    result = replace(result, cap_mps=min(body_cap, motion_proof.cap_mps))
                    result.validation['junction_traffic'] = dict(state='MAP_TIMED_ENTRY',
                        reason='fresh same-path map timing and physical body checked',
                        target_mps=motion_proof.cap_mps,
                        current_clear_m=motion_proof.current_clear_m)
                else:
                    result = replace(result, cap_mps=0., emergency=True,
                        reason=result.reason if result.emergency else
                            rejected_reason or 'junction launch profile not freshly authorized after body check')
                    result.validation['junction_traffic'] = dict(state='WAIT', reason=result.reason)
            elif recovery_requested:
                # This is observed startup occupancy, not an accepted entry
                # gap. Never skip an NPC forecast or soften its emergency.
                checked_now = time.monotonic()
                usable = bool(recovery is not None and recovery_proof is not None
                    and recovery.matches(recovery_proof, ego, world_path, checked_now))
                if usable:
                    frame = ring_entry.PathFrame(np.asarray(world_path, float)[:, :2])
                    centre = float(frame.project([[ego['x'], ego['y']]])[0][0])
                    follow_cap, leader = ring_entry.follow_cap(to_sim_objects(traffic_cars, rtf),
                        frame, centre, abs(float(ego['speed_kmh']))/3.6, ego_front_m=EGO_FRONT_M)
                    result = replace(result, cap_mps=min(result.cap_mps, follow_cap))
                    if not result.emergency and result.cap_mps > 0.:
                        usable = recovery.accept(recovery_proof, ego, world_path,
                            getattr(runtime, 'stop_line_controller', None), time.monotonic())
                    result.validation['junction_traffic'] = dict(state='RECOVERY_FOLLOW',
                        reason='observed inside junction: full live collision checks', leader=leader)
                    result.validation['recovery_full_npc_forecast'] = True
                if not usable:
                    result = replace(result, cap_mps=0., emergency=True,
                                     reason='junction recovery proof unavailable')
            elif following_only:
                latest = None
                latest_status = None
                if 'junction_live' in validation:
                    try:
                        if motion_evaluation:
                            # The failed certificate may already have run this
                            # exact current-source map model. Reuse that result
                            # within this call, without renewing any deadline.
                            latest = copy.deepcopy(motion_evaluation['decision'])
                            latest_status = copy.deepcopy(motion_evaluation['status'])
                        else:
                            checked = copy.copy(self.traffic)
                            checked.junction = dict(self.traffic.junction)
                            checked.speed_ratio = dict(self.traffic.speed_ratio)
                            checked._standing_since = dict(self.traffic._standing_since)
                            profile = float(validation.get('junction_body_cap_mps', baseline_cap))
                            if not math.isfinite(profile) or profile < 0.:
                                raise ValueError('invalid junction body profile')
                            latest = checked.evaluate(to_sim_objects(traffic_cars, rtf),
                                dict(ego), path, profile, time.monotonic(), fresh=True)
                            latest_status = copy.deepcopy(checked.status)
                        latest_cap = float(latest['v_mps']) if latest else 0.
                        if not math.isfinite(latest_cap) or latest_cap < 0.:
                            latest = None
                    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
                        latest = None
                if latest is None:
                    result = replace(result, cap_mps=0., emergency=True,
                        reason=result.reason if result.emergency else
                            'junction current map timing unavailable')
                    result.validation['junction_traffic'] = dict(state='WAIT', reason=result.reason)
                else:
                    # No motion certificate means no NPC may leave the full
                    # generic body/forecast checks. A fresh map decision adds
                    # its crossing/approach cap; it cannot relax those checks.
                    cap = min(result.cap_mps, latest_cap)
                    result = replace(result, cap_mps=cap,
                        emergency=result.emergency or latest_cap <= 0.,
                        reason=result.reason if result.emergency else latest.get('reason'))
                    result.validation['junction_traffic'] = latest_status
            elif timed_vehicles:
                from roundabout_empty import entry_publication_can_commit
                result = self.junction_follow(result, runtime, traffic_cars, rtf, world_path, ego,
                    empty_path_geometry=empty_path_geometry,
                    empty_raw_clear_m=empty_raw_clear,
                    empty_vehicle_ready=bool(vehicle_ready and empty_vehicle_observation
                        and -.12 <= empty_vehicle_lag <= MAX_ENTRY_SOURCE_AGE_S
                        and 0. <= empty_vehicle_age <= MAX_ENTRY_SOURCE_AGE_S
                        and 0. <= empty_scan_age <= MAX_ENTRY_SOURCE_AGE_S and raw_clear is not None),
                    empty_sources_expire_at=now+MAX_ENTRY_SOURCE_AGE_S-max(empty_vehicle_age, empty_scan_age),
                    empty_commit_allowed=entry_profile is not None or
                        entry_publication_can_commit(original, ego, time.monotonic()),
                    yield_vehicle_ready=bool(vehicle_ready and -.12 <= empty_vehicle_lag <= .35
                        and 0. <= empty_vehicle_age <= .35 and 0. <= empty_scan_age <= .35
                        and raw_clear is not None),
                    yield_sources_expire_at=now+.35-max(empty_vehicle_age, empty_scan_age),
                    yield_source_frame=yield_source_frame, yield_source_receiver=vehicle_receiver,
                    yield_profile_override=entry_profile is not None,
                    yield_independent_clear_m=forecast_raw_clear,
                    yield_motion_evaluation=yield_motion_evaluation)
                if yield_moving_bodies:
                    # The dashboard and next geometry proposal report this
                    # current map result; they do not run a second predictor.
                    runtime.gap_planner.status = copy.deepcopy(self.traffic.status)
                    runtime.gap_planner.state = self.traffic.state
            result = replace(result, cap_mps=min(result.cap_mps, current_body_cap),
                clear_m=min(result.clear_m, raw_clear) if raw_clear is not None else result.clear_m)
            if junction_current_contact:
                # Commitment owns future following, never present contact.
                result = replace(result, cap_mps=0., clear_m=0., emergency=True,
                                 reason='live junction current body contact')
                validation['junction_current_contact'] = True
            final_now = time.monotonic()
            if (yield_motion_proof is not None and
                    not proof_matches(yield_motion_proof, ego, world_path, now=final_now)):
                result = replace(result, cap_mps=0., emergency=True,
                                 reason='junction yield motion proof expired before publication')
            if (recovery_requested and (recovery_proof is None or recovery is None
                    or not recovery.matches(recovery_proof, ego, world_path, final_now))):
                result = replace(result, cap_mps=0., emergency=True,
                                 reason='junction recovery proof expired before publication')
            if following_only:
                junction_compute['total'] = (final_now-call_started_at)*1000.
                validation['junction_compute_ms'] = junction_compute
            motion_expired = bool(motion_proof is not None and
                not proof_matches(motion_proof, ego, world_path, now=final_now))
            if motion_expired:
                result = replace(result, cap_mps=0., emergency=True,
                                 reason='junction motion proof expired before publication')
            if following_only and not motion_expired and ('junction_live' not in validation or
                    not now <= final_now <= junction_live_deadline):
                result = replace(result, cap_mps=0., emergency=True,
                                 reason='junction live check expired before publication')
        retry_state = result.validation.get('junction_source_retry_needed') if result else None
        if retry_state == 'changed' and not _source_retry:
            checked = self(world_path, current, mode, original, _source_retry=True)
            return replace(checked, validation=dict(checked.validation,
                junction_source_retry=dict(reason='new complete frame during full launch checks',
                                           previous_frame=yield_source_frame)))
        if result and result.validation.get('junction_source_launch'):
            validation['junction_source_launch'] = result.validation['junction_source_launch']
            validation['junction_live'] = dict(
                expires_at=validation['junction_source_launch']['expires_at'],
                source_frame=yield_source_frame)
        validation.update(valid=bool(validation.get('valid', True) and valid),
            pose_valid=bool(validation.get('pose_valid', True) and valid),
            sensor_times=sensor_times, live_safety=result.validation if result else dict(stopped=True, invalid_sensor=True))
        if (entry_profile is not None and result is not None
                and result.validation.get('junction_source_launch')
                and not result.emergency and result.cap_mps > 0. and valid):
            # Only the current full traffic/body result may replace a bound
            # pure traffic WAIT. The final publisher barrier still owns GO.
            validation['junction_entry_profile_accepted'] = True
            baseline_cap = min(entry_profile, validation.get('stationary_semantic_cap_mps', math.inf))
            baseline_emergency = False
        cap = min(baseline_cap, result.cap_mps) if result else 0.
        emergency = baseline_emergency or not valid or result.emergency
        # A committed constant-speed ring uses a distance-based curve preview.
        # This changes only when to brake for a bend; body and traffic caps above
        # and the follower's existing lateral-acceleration limit still apply.
        if (timed_vehicles and committed and not recovery_requested and valid
                and not emergency and cap > 0.
                and self.traffic.junction.get('constant_ring_speed')
                and vehicle_ready and 0. <= empty_vehicle_age <= .35
                and 0. <= empty_scan_age <= .35):
            checked_now = time.monotonic()
            pose_age = max(float(ego.get('age_s', math.inf)),
                           checked_now-float(ego.get('received', checked_now)))
            expiry = min(now+.35-max(empty_vehicle_age, empty_scan_age),
                         checked_now+.2-pose_age)
            if math.isfinite(pose_age) and 0. <= pose_age <= .2 and checked_now < expiry:
                validation['junction_curve_profile'] = dict(
                    kind='committed_ring', expires_at=expiry, target_mps=25./3.6)
        static_pass = getattr(runtime, '_static_lane_pass', None)
        static_pass_reason = None
        if (static_pass is not None and not tunnel and cap > 0. and not emergency
                and (validation.get('static_lane_borrow') or static_pass.entered_at is not None)):
            from planning.highway.path_continuity import projected_suffix
            try:
                legal = static_pass.corridor(ego)
                static_pass.observe(ego, time.monotonic())
                source_proof = (static_motion_source_freshness(points, ego, batches,
                    age, now, time.monotonic()) if static_ceiling is not None else None)
                if source_proof is not None:
                    validation['static_profile_sources'] = source_proof
                pass_check = static_pass.assess(projected_suffix(path), ego, cap,
                    legal, time.monotonic(),
                    [obj for _, objects, _, _ in batches for obj in objects],
                    ready=bool(valid and (source_proof['ready'] if source_proof is not None else
                        age <= .3 and all(source_age <= .3 for _, _, _, source_age in batches))),
                    motion_profile=bool(validation.get('static_motion_profile')),
                    profile_ceiling_mps=(min(static_ceiling, result.cap_mps, current_body_cap)
                        if static_ceiling is not None and result is not None else None))
                if pass_check['ok'] and static_ceiling is not None and result is not None:
                    checked_now = time.monotonic()
                    pose_age = max(float(ego.get('age_s', math.inf)),
                                   checked_now-float(ego.get('received', checked_now)))
                    validation.update(static_profile_live_ceiling_mps=min(
                            static_ceiling, result.cap_mps, current_body_cap),
                        static_profile_live_command_mps=cap,
                        static_profile_deadline=min(source_proof['expires_at'], checked_now+.2-pose_age))
            except (TypeError, ValueError, IndexError, OverflowError):
                pass_check = dict(ok=False, reason='static pass live geometry unavailable')
            validation['static_lane_pass'] = pass_check
            if not pass_check['ok']:
                cap, emergency = 0., True
                static_pass_reason = pass_check.get('reason')
        if self.update_yield_launch(ego if valid else None, stopped=emergency or cap <= 0.):
            validation['yield_launch_interrupted'] = True
        reason = (result.reason if result and result.reason else
                  original.stop_reason if original is not None and not sensor_recovered else None)
        reason = static_pass_reason or reason
        if not valid:
            reason = (road_frame['proof']['reason'] if road_frame is not None
                      and not road_frame['proof']['valid'] else 'live path sensor or pose stale')
        clear = min(original.forward_clear_m, result.clear_m) if original is not None and result else result.clear_m if result else 0.
        plan = DrivingPlan(getattr(runtime.path_publisher, '_plan_sequence', 0)+1,
            path, ego, dict(ego, **{k: aligned[k] for k in ('x', 'y', 'heading_deg')}),
            stamp, max(0., age), now, max(0., cap), max(0., clear), bool(emergency), validation, mode, reason)
        plan = plan.decided(stop_reason=reason, geometry_path_digest=plan.path_digest(),
            traffic_path_digest=plan.path_digest(), geometry_frame_digest=plan.frame_digest(),
            traffic_frame_digest=plan.frame_digest())
        profile_check = (result.validation.get('junction_entry_profile_check')
                         if result is not None else None)
        if (entry_profile is not None and isinstance(profile_check, dict) and valid
                and not junction_current_contact
                and time.monotonic() <= profile_check['expires_at']):
            from junction_entry_profile import bind_entry_profile
            metadata = original.validation['junction_entry_profile']
            ceiling = min(entry_profile, profile_check['ceiling_mps'],
                          validation.get('stationary_semantic_cap_mps', math.inf))
            plan = bind_entry_profile(plan, ceiling, metadata['line_id'], metadata['line_station_m'])
            if 'junction_entry_profile' in plan.validation:
                plan = plan.decided(stop_reason=reason, junction_entry_profile_refreshed=True)
        if (static_source_profile and not _source_retry and valid
                and not plan.emergency and plan.allowed_mps > 0.):
            deadline = plan.validation.get('static_profile_deadline')
            if deadline is not None and float(deadline)-time.monotonic() < 1./15.:
                latest = runtime.lidar.scan_snapshot() if runtime.lidar is not None else {}
                latest_stamp = float(latest.get('stamp') or 0.)
                if (math.isfinite(latest_stamp) and latest_stamp > stamp
                        and 0. <= float(latest.get('age_s', math.inf)) <= .3):
                    # Recheck the complete fixed world curve against a genuinely
                    # newer scan before serialization exhausts the old proof.
                    # The recursive call captures its own pose, objects and
                    # return clock. A new contact or missing source still stops.
                    checked = self(world_path, current, mode, original, _source_retry=True)
                    return checked.decided(stop_reason=checked.stop_reason,
                        static_source_retry=dict(from_stamp=stamp, to_stamp=latest_stamp))
        return plan


def bind_plan_id(plan, plan_id, target):
    """Publication ramp is part of the plan the controller actually receives."""
    return replace(plan, plan_id=plan_id, allowed_mps=min(plan.allowed_mps, target))
