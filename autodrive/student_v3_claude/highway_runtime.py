"""Highway-only perception, overtaking and speed control.

This branch is dispatched before the normal-road Student camera model. It shares
sensor inputs and the ROS output format, but owns its path and speed decisions.
"""
import math
import os
import time
import json
from pathlib import Path

import numpy as np
from json_safe import json_default

from highway_planner import local_points, world_points, geometry_speed_cap
from highway_observations import HighwayObservationFilter, HighwayVehicleMotion
from highway_speed_plan import clearance_forecast_bound
from lidar_corridor import without_moving_objects
from sim_time import runtime_rtf, to_sim_objects

# Scan/fusion jitter, not an outage: 99 % of the "sensor late: hold speed"
# frames in the 2026-10-10 runs had LiDAR 0.25-0.42 s or fusion 0.35-0.44 s old
# with the vehicle detector ready (1.0 s/lap lost to interrupted acceleration).
# Up to these ages the checked plan is used as is: its reaction reserves already
# include the measured sensor ages. Lane-change starts still wait for fresh data
# and older (<= 1 s) data still holds speed.
HIGHWAY_MILD_LIDAR_AGE_S = .45
HIGHWAY_MILD_OBJECT_AGE_S = .5


class HighwayRuntime:
    def infer_highway_only(self, status, ego):
        if self.input_error or float(ego.get("age_s", 999.)) > .25:
            self.last_result = None
            if self.path_publisher:
                self.path_publisher.emergency_stop = True
                self.path_publisher.publish_stop(self.input_error or 'highway localization stale')
            return False
        index = int(status["path"]["nearest_idx"])
        source = np.asarray(getattr(self, "control_waypoint_xy", self.highway.route), dtype=float)
        indices = (index + np.arange(640)) % len(source)
        xy = local_points(source[indices, :2], ego)
        arc = np.r_[np.linalg.norm(xy[0]), np.linalg.norm(xy[0]) + np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        route = np.column_stack((xy, arc, np.ones(len(xy))))
        return self.infer_highway(status, ego, route, route[:20], route[::10][:20],
                                  route[:20, :2], np.empty(0), np.empty(0), np.empty(0),
                                  0., time.perf_counter())

    def infer_highway(self, status, ego, dense_route, model_route, display_route,
                      predicted_path, probabilities, concept_values, scene, latency, started):
        """Use measured traffic gaps and a long lane-change path in the highway section."""
        from gui import (AsyncDecisionLog, highway_objects, highway_measured_objects,
                         highway_static_objects,
                         cosine_gallery_matches, trajectory_diagnostics,
                         speed_label, body_gap_m, HIGHWAY_SIDE_MARGIN_M)
        from ego_body import EGO_CENTRE_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M
        from planning.highway.body_geometry import (HighwayLaneCorridor,
            highway_boxed_clearance, highway_raw_clearance, highway_nudge, highway_side_gap_cap)
        from planning.highway.trajectory_geometry import normalize_path, sample_pose
        from scipy.spatial import cKDTree
        from highway_sensor_alignment import (align_measured_cars, align_moving_observations,
                                              align_scan_points, align_highway_predictions)
        from planning.highway.object_policy import (highway_scan_mask_objects,
                                                    without_far_moving_returns,
                                                    highway_tentative_objects)
        from planning.highway.static_returns import FarReturnPersistence
        from planning.highway.prediction_safety import aligned_prediction_front_contact
        nearest = int(status["path"]["nearest_idx"])
        cars, vehicle_ready, vehicle_status = self.vehicle_receiver.snapshot()
        cars = self.on_road_vehicles(cars)
        # Highway cars ahead without a detector velocity are moving traffic
        # (user rule): measured along their lane here, else the assumed flow.
        if not hasattr(self, '_highway_vehicle_motion'):
            self._highway_vehicle_motion = HighwayVehicleMotion()
        cars, vehicle_motion = self._highway_vehicle_motion.update(
            cars, vehicle_status.get('stamp'), ego, getattr(self, 'lanes', None), runtime_rtf(self))
        if self.object_receiver and hasattr(self.object_receiver, 'snapshot_with_time'):
            fused, fusion_stamp, fusion_age = self.object_receiver.snapshot_with_time()
        else:
            fused = self.object_receiver.snapshot() if self.object_receiver else []
            fusion_stamp = getattr(self.object_receiver, 'observation_stamp', None)
            fusion_age = float(getattr(self.object_receiver, 'age_s', 0.))
        fused_observations = list(fused)
        scan = (self.lidar.xy_snapshot() if self.lidar else
                dict(points=None, age_s=99., stamp=None, received=0.))
        lidar_points, lidar_age = scan['points'], float(scan['age_s'])
        scan_stamp = scan['stamp']
        pose_stamp = float(ego.get('stamp', 0.) or 0.)
        vehicle_stamp = float(vehicle_status.get('stamp', 0.) or 0.)
        scan_pose = None
        if scan_stamp and hasattr(getattr(self, 'ego_receiver', None), 'at_stamp'):
            scan_pose = self.ego_receiver.at_stamp(scan_stamp)
        frame_aligned = scan_pose is not None
        scan_pose_missing = bool(lidar_points is not None and scan_stamp and pose_stamp > 0.
                                 and not frame_aligned)
        if scan_pose_missing:
            # A timestamped scan without its acquisition pose cannot validate
            # an ego-frame path or create persistent near-field occupancy.
            lidar_points = None
        if lidar_points is not None and frame_aligned:
            lidar_points = align_scan_points(lidar_points, scan_pose, ego)
        propagation_age = fusion_age
        if (fusion_stamp and pose_stamp > 0. and
                -.12 <= pose_stamp-fusion_stamp <= .35):
            propagation_age = max(0., pose_stamp-fusion_stamp)
        # The observation filter propagates confirmed obstacle motion only.
        # Its NPC/person pass-through still needs source-to-planning alignment
        # before deriving any scan-time mask from the planning timestamp.
        fused, fusion_alignment = align_moving_observations(fused, fusion_stamp, pose_stamp)
        if not hasattr(self, "_highway_observations"):
            self._highway_observations = HighwayObservationFilter()
        fused, tentative, observation_info = self._highway_observations.update(
            fused, fusion_stamp, lidar_points, ego, age_s=propagation_age)
        observation_info['propagation_age_s'] = round(propagation_age, 3)
        observation_info['source_age_s'] = round(fusion_age, 3)
        # Parked camera boxes are not propagated: test a mover's ownership of
        # one where the mover was at the camera time (bounded like alignment).
        parked_delay = 0.
        if fusion_stamp and vehicle_stamp > 0. and abs(float(fusion_stamp)-vehicle_stamp) <= .35:
            parked_delay = float(fusion_stamp)-vehicle_stamp
        objects = highway_objects(fused, cars, ego=ego, parked_delay_s=parked_delay)
        # A camera parked-vehicle box ahead that no LiDAR car owns is the same
        # unmeasured highway vehicle: assumed flow, never a standing body.
        objects = self._highway_vehicle_motion.camera_vehicles(
            objects, ego, getattr(self, 'lanes', None), runtime_rtf(self), propagation_age)
        # Source age follows the actual moving leader. An unrelated camera
        # dropout must not enlarge a fresh LiDAR car's following envelope.
        vehicle_age = max(0., float(vehicle_status.get('age_s', 0.) or 0.))
        if pose_stamp > 0. and vehicle_stamp > 0.:
            vehicle_age = max(vehicle_age, pose_stamp-vehicle_stamp)
        self.highway.longitudinal.sensor_age_s = vehicle_age
        self.highway.longitudinal.static_sensor_age_s = max(0., float(lidar_age))
        self.highway.longitudinal.fusion_sensor_age_s = max(0., fusion_age)
        object_stale = bool(self.object_receiver and fusion_age > .35)
        stale = lidar_points is None or lidar_age > .25 or not vehicle_ready or object_stale
        # A late frame under CPU load (LiDAR 0.26 s) sent one stop at 82 km/h
        # (live 2026-10-03 20:38). Briefly late data keeps the plan but adds no
        # speed; only a real outage (>0.6 s) or warm-up stops.
        object_age = fusion_age
        # Never stop on the highway for a short sensor hiccup (user rule): up to 1 s late
        # LiDAR/objects, and a vehicle detector that dropped "ready" < 2 s ago, hold speed.
        # (Run 97 stopped at 0 km/h: detector at 2.5 Hz while a test ran on this PC.)
        now_mono = time.monotonic()
        if vehicle_ready:
            self._vehicle_ready_seen = now_mono
        recently_ready = vehicle_ready or now_mono - getattr(self, "_vehicle_ready_seen", -99.) < 2.
        briefly_late = bool(stale and lidar_points is not None and recently_ready and
                            lidar_age <= 1. and object_age <= 1. and getattr(self, "_highway_was_ready", False))
        self._highway_was_ready = not stale or briefly_late
        mildly_late = bool(briefly_late and vehicle_ready and lidar_age <= HIGHWAY_MILD_LIDAR_AGE_S and
                           (not self.object_receiver or object_age <= HIGHWAY_MILD_OBJECT_AGE_S))
        # hold_speed: briefly late beyond the mild bounds keeps the plan, adds no speed.
        hold_speed = briefly_late and not mildly_late
        lane_options = None
        motion_options = None
        motion_evidence = {}
        # Footprint headings of long lane followers (start test, recheck and
        # speed plan alike); object tuples, scan masks and logs keep the box yaw.
        self.highway.longitudinal.stable_yaws = {}
        if self.highway.free_three_lane:
            from highway_three_lane import HighwayLaneMap
            from planning.highway.motion_evidence import HighwayMotionEvidence
            if not hasattr(self, "_highway_lane_map"):
                self._highway_lane_map = HighwayLaneMap(self.lanes, self.highway.config)
            lane_options = self._highway_lane_map.candidates(ego, dense_route)
            motion_options = self._highway_lane_map.motion_candidates(ego, objects)
            if not hasattr(self, '_highway_motion_evidence'):
                self._highway_motion_evidence = HighwayMotionEvidence()
            objects, motion_evidence = self._highway_motion_evidence.update(
                objects, vehicle_status.get('stamp'), motion_options, ego)
            self.highway.longitudinal.stable_yaws = dict(self._highway_motion_evidence.lane_headings)
            corridor = self._highway_lane_map.corridor(ego, dense_route, lane_options)
        else:
            corridor = HighwayLaneCorridor.from_corridor(self.lanes.corridor(ego, dense_route))
        objects, car_alignment = align_measured_cars(objects, vehicle_stamp, pose_stamp)
        static_points = None
        mask_alignment = {}
        far_return_info = {}
        if lidar_points is not None:
            # Mask at the scan's source time, then retain only static returns.
            # A moving front face otherwise sits outside an older detector box
            # and is stored as a second, stationary obstacle in near memory.
            mask_objects = highway_measured_objects(objects)
            if scan_stamp and pose_stamp > 0.:
                if not car_alignment['timing_valid']:
                    mask_objects = [o for o in mask_objects if o[3] != 'lidar_car']
                if not fusion_alignment['timing_valid']:
                    mask_objects = [o for o in mask_objects if o[3] == 'lidar_car']
            scan_objects, mask_alignment = align_moving_observations(
                mask_objects, pose_stamp, scan_stamp)
            if ((frame_aligned and mask_alignment.get('timing_valid')) or
                    not scan_stamp or not pose_stamp):
                lidar_points = without_moving_objects(lidar_points, scan_objects, ego)
                # Beyond the near field, a moving car's rear face or a brief
                # (<= 0.5 s) prediction gap must not leave its own returns as a
                # 30-55 m stop line. The near field keeps the exact mask above.
                far_objects = ([] if scan_stamp and pose_stamp > 0. and
                               not car_alignment['timing_valid'] else
                               highway_scan_mask_objects(objects))
                if far_objects:
                    far_objects, _ = align_moving_observations(far_objects, pose_stamp, scan_stamp)
                    lidar_points = without_far_moving_returns(lidar_points, far_objects, ego)
            # Far returns ahead count once they stay at one map position.
            if not hasattr(self, '_highway_far_returns'):
                self._highway_far_returns = FarReturnPersistence()
            lidar_points = self._highway_far_returns.update(
                lidar_points, ego, scan_stamp if frame_aligned else None)
            far_return_info = dict(self._highway_far_returns.info)
            lidar_points = self.near_field_memory.update(
                lidar_points, ego, scan['received'], mask_objects)
            static_points = lidar_points
        # Scan masks retain their source-time contracts above. Retained traffic
        # predictions need one additional source-to-ego alignment for planning;
        # applying it before masking would move their far-return masks twice.
        objects, prediction_alignment = align_highway_predictions(
            objects, vehicle_stamp, pose_stamp)
        sensor_alignment = dict(scan_frame_aligned=frame_aligned,
            scan_stamp=scan_stamp, ego_stamp=pose_stamp,
            scan_pose_missing=scan_pose_missing,
            traffic=car_alignment, retained_traffic=prediction_alignment,
            fusion=fusion_alignment, static_mask=mask_alignment,
            far_returns=far_return_info)
        # The bound LaneCorridor method lets the policy bound a standstill
        # escape's corner swing with the same body model.
        map_clearance = corridor.clear_distance if corridor is not None else (lambda path, horizon: 0.)
        stationary = highway_static_objects(objects)
        static_tree = cKDTree(static_points) if static_points is not None and len(static_points) else None
        tight_margin_used = [False]
        static_components = {}
        def static_clearance(path, horizon, nudge_path=True):
            if static_points is None:
                return 0.
            if not self.highway.free_three_lane and nudge_path:
                path = highway_nudge(path, static_points, static_tree, float(ego.get("speed_kmh", 0.)) / 3.6)
            clear = highway_boxed_clearance(path, ego, stationary, horizon)
            # One physical rectangle: full 4.635 m length, 1.892 m width,
            # 25 cm margin. Do not pad a centre-radius hit again by half length.
            # 15 cm beside the body: roadside cones 25 cm away were reported
            # as blocking once 0.2 m grid rounding was added.
            raw_side_margin = HIGHWAY_SIDE_MARGIN_M
            raw_clear = highway_raw_clearance(path, static_points, horizon,
                extra_margin=.25, tree=static_tree, lateral_margin=raw_side_margin)
            if raw_clear < 15. and float(ego.get("speed_kmh", 0.)) < 3.:
                # Stopped beside lane-edge barrels 0.1 m from the body: the
                # 15 cm side margin blocked even the path steering away, so the
                # car stayed parked (live 2026-10-03 21:35). At creep speed only
                # the body itself (+3 cm) must stay clear.
                # Only to leave a standstill, then at walking pace: at 14 km/h
                # the 3 cm check let the car graze lane-edge barrels while its
                # tracking error was 0.5-0.7 m (live 2026-10-03 23:00).
                tight = highway_raw_clearance(path, static_points, horizon,
                    extra_margin=.25, tree=static_tree, lateral_margin=.03)
                if tight > raw_clear:
                    tight_margin_used[0] = True
                    raw_clear = tight
                    raw_side_margin = .03
            static_components.update(boxed_clear_m=float(clear), raw_clear_m=float(raw_clear),
                                     raw_lateral_margin_m=raw_side_margin)
            return min(clear, raw_clear)
        dense_xy = np.asarray(dense_route, dtype=float)[:, :2]
        dense_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(dense_xy, axis=0), axis=1))]
        lane_profile = []
        for start_m in (() if self.highway.free_three_lane else range(0, 160, 25)):
            k = int(np.searchsorted(dense_arc, start_m))
            if k < len(dense_xy) - 20:
                # adjacent_offsets measures 8 m along the route it is given.
                lane_profile.append((float(dense_arc[k]) + 8., self.lanes.adjacent_offsets(ego, dense_xy[k:])))
        # Everything above (alignment, scan masks, ownership, motion evidence)
        # pairs measured wall-clock velocities with wall-clock ages. The planner
        # times traffic against the ego in simulator units (sim_time).
        rtf = runtime_rtf(self)
        planner_objects = to_sim_objects(objects, rtf)
        self.highway.longitudinal.time_scale = rtf
        if self.highway.free_three_lane:
            from planning.highway.path_continuity import published_reference
            self.highway.continuation_reference = (published_reference(
                getattr(self.path_publisher, '_published_plan', None), ego,
                time.monotonic(), local_points) if self.highway.saved_world is not None else None)
        plan = self.highway.decide(nearest, dense_route, ego, planner_objects,
            lane_profile[0][1] if lane_profile else self.lanes.adjacent_offsets(ego, dense_route),
            map_clearance, static_clearance,
            perception_ready=not stale, rear_unclassified=static_points, lane_profile=lane_profile,
            lane_options=lane_options, motion_options=motion_options,
            path_transform=(lambda p: highway_nudge(p, static_points, static_tree,
                float(ego.get("speed_kmh", 0.)) / 3.6)) if self.highway.free_three_lane else None,
            path_speed_cap=(lambda p: highway_side_gap_cap(p, static_points, static_tree))
                if self.highway.free_three_lane and static_points is not None else None)
        path, info = plan["path"], plan["status"]
        if self.highway.free_three_lane:
            info = dict(info,native_geometry_reused=self._highway_lane_map.geometry_reused)
        tentative = highway_tentative_objects(tentative, cars, ego, parked_delay_s=parked_delay)
        if tentative and not plan["emergency"]:
            candidate_clear = highway_boxed_clearance(path, ego, tentative, 170.)
            speed = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
            if candidate_clear < max(12., speed*.4+speed*speed/8.):
                cautious = max(0., speed-.4)
                if cautious < plan["target_mps"]:
                    plan = dict(plan, target_mps=cautious)
                    info = dict(info, reason="highway unconfirmed obstacle: cautious slowdown")
            observation_info["candidate_clear_m"] = round(candidate_clear, 2)
        info = dict(info, observations=observation_info, motion_evidence=motion_evidence,
                    sensor_alignment=sensor_alignment, vehicle_motion=vehicle_motion)
        side_cap = float("inf")
        if static_points is not None and len(path) > 3:
            # Publish the same nudged path the clearance checks above evaluated.
            if not self.highway.free_three_lane:
                path = highway_nudge(path, static_points, static_tree, float(ego.get("speed_kmh", 0.)) / 3.6)
            side_cap = highway_side_gap_cap(path, static_points, static_tree)
            if side_cap < plan["target_mps"]:
                plan = dict(plan, target_mps=side_cap)
                info = dict(info, reason="highway narrow side gap: slow to pass")
        info["side_gap_cap_kmh"] = round(side_cap * 3.6, 1) if math.isfinite(side_cap) else None
        # Freeze the exact local waypoint representation consumed by the
        # publisher before the final curve, map, static and command checks.
        try:
            path = normalize_path(path)[0].astype(np.float32).astype(float)
            if not np.isfinite(path).all():
                raise ValueError('highway path cannot be represented by publisher')
        except (TypeError, ValueError, IndexError, OverflowError):
            self.last_result = None
            if self.path_publisher:
                self.path_publisher.emergency_stop = True
                self.path_publisher.publish_stop('invalid highway final path')
            return False
        # The follower consumes this highway plan without the normal-road
        # curvature heuristic. Check the path after the final LiDAR nudge too.
        selected_ceiling = float(info.get('speed_ceiling_kmh', self.highway.target_mps*3.6))/3.6
        final_curve_cap = geometry_speed_cap(path, selected_ceiling)
        if final_curve_cap < plan["target_mps"]:
            plan = dict(plan, target_mps=final_curve_cap)
            info = dict(info, reason="highway final-path curvature braking envelope")
        info["final_path_curve_cap_kmh"] = round(final_curve_cap * 3.6, 1)
        hard_stale = stale and not briefly_late
        emergency = bool(plan["emergency"] or hard_stale)
        reason = ("highway scan pose unavailable" if scan_pose_missing else
                  "highway sensor stream stale/warming" if hard_stale else
                  "highway sensor late: hold speed" if hold_speed else info["reason"])
        info["sensor_late_mild"] = mildly_late
        effective_target = (0. if hard_stale else
                            min(plan["target_mps"], max(0., float(ego.get("speed_kmh", 0.))) / 3.6)
                            if hold_speed else plan["target_mps"])
        if tight_margin_used[0]:
            effective_target = min(effective_target, 5. / 3.6)
        # Returning to cruise retains the checked braking ramp from measured
        # speed; a hard 120 -> 90 command would discard that same trajectory.
        measured_speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
        # The legal limit before the exemption zone is a hard ceiling, also
        # below measured momentum (rule 3-2: 15 s the moment it is exceeded).
        rule_cap = info.get('rule_speed_cap_kmh')
        command_ceiling = min(self.highway.overtake_mps, max(selected_ceiling, measured_speed),
            float(info.get('exit_cap_kmh', self.highway.overtake_mps*3.6))/3.6,
            max(0., float(ego.get("speed_kmh", 0.))) / 3.6 if hold_speed else float("inf"),
            5. / 3.6 if tight_margin_used[0] else float("inf"),
            float(rule_cap)/3.6 if rule_cap is not None else float("inf"))
        source_stamps = (float(scan_stamp or 0.),
                         float(vehicle_status.get('stamp', 0.) or 0.),
                         float(fusion_stamp or 0.))
        validated_stamp = (min(source_stamps) if not stale and all(
            math.isfinite(stamp) and stamp > 0. and
            -.12 <= float(ego.get('stamp', 0.))-stamp <= .35
            for stamp in source_stamps) else None)
        # Recheck the exact final path. Preserve real hits and mapped boundaries,
        # while carrying completed-query semantics into the command forecast.
        query_m = float(info.get('clearance_query_m', 170.))
        # A checked standstill escape join keeps its bounded corner swing here;
        # any other or changed path gets the plain corridor check.
        final_map_m = float((plan.get('map_clearance') or map_clearance)(path, query_m))
        final_static_m = float(static_clearance(path, query_m, nudge_path=False))
        info['static_clearance_sources'] = dict(static_components)
        raw_hit_m = static_components.get('raw_clear_m', query_m)
        if static_tree is not None and raw_hit_m < query_m-1e-6 and len(path) > 3:
            if raw_hit_m <= 1e-9:
                axle = np.zeros((1, 2))
                tangent = np.array([1., 0.])
            else:
                path_xy, path_arc = normalize_path(path)
                axle, yaw = sample_pose(path_xy, path_arc, np.array([raw_hit_m]), 0.)
                tangent = np.array([math.cos(yaw[0]), math.sin(yaw[0])])
            normal = np.array([-tangent[1], tangent[0]])
            centre = axle[0]+EGO_CENTRE_M*tangent
            raw_side_margin = static_components.get('raw_lateral_margin_m', HIGHWAY_SIDE_MARGIN_M)
            ids = static_tree.query_ball_point(centre,
                math.hypot(EGO_HALF_LENGTH_M+.25, EGO_HALF_WIDTH_M+raw_side_margin))
            if ids:
                points = static_points[ids]
                delta = points-centre
                hits = points[(np.abs(delta @ tangent) <= EGO_HALF_LENGTH_M+.25) &
                              (np.abs(delta @ normal) <= EGO_HALF_WIDTH_M+raw_side_margin)]
                if len(hits):
                    info['static_clearance_sources']['raw_hit_world_xy'] = np.round(
                        world_points(np.mean(hits, axis=0)[None, :], ego)[0], 3).tolist()
                    info['static_clearance_sources']['raw_hit_point_count'] = len(hits)
        final_bound = min(clearance_forecast_bound(final_map_m, query_m),
                          clearance_forecast_bound(final_static_m, query_m))
        info['clearance_components_m'] = dict(map=final_map_m, static=final_static_m)
        info['clear_distance_m'] = min(final_map_m, final_static_m)
        info['final_clearance_query'] = dict(horizon_m=query_m,
            map_query_complete=math.isinf(clearance_forecast_bound(final_map_m, query_m)),
            static_query_complete=math.isinf(clearance_forecast_bound(final_static_m, query_m)),
            lidar_max_range_m=100.)
        self.highway.longitudinal.time_scale = rtf
        # Keep the alignment-specific contact witness alongside the general
        # rear-axle forecast; time alignment must never release actual contact.
        prediction_contact = aligned_prediction_front_contact(
            objects, ego, prediction_alignment['aligned_indices'])
        info['retained_prediction_front_contact'] = prediction_contact
        if prediction_contact is not None:
            emergency = True
            reason = 'highway retained vehicle overlaps actual front body'
        speed_plan = self.highway.plan_speed(path, ego, planner_objects, effective_target,
            static_clear_m=final_bound,
            static_mover_mps=float(info.get("static_mover_kmh", 0.)) / 3.6,
            emergency=emergency, ceiling_mps=command_ceiling,
            validation_stamp=validated_stamp)
        effective_target = speed_plan["reference_mps"]
        emergency = emergency or speed_plan["emergency"]
        if (speed_plan["emergency"] and not plan["emergency"] and not hard_stale and
                prediction_contact is None):
            reason = "highway braking trajectory conflict"
        info["speed_plan"] = speed_plan["diagnostics"]
        info['speed_plan']['moving_sensor_age_s'] = vehicle_age
        info['speed_plan']['rtf'] = round(rtf, 3)
        stopped = emergency or effective_target <= .01
        info = dict(info, target_kmh=round(effective_target*3.6, 1),
                    reason=reason, emergency_stop=emergency)
        info["raw_lidar_body_margin_m"] = .25
        info["raw_lidar_side_margin_m"] = HIGHWAY_SIDE_MARGIN_M
        info["raw_lidar_body_width_m"] = 1.892
        info["sensor_failure"] = dict(lidar_missing=lidar_points is None,
            lidar_stale=bool(lidar_age > .25), vehicle_not_ready=not vehicle_ready,
            object_stale=object_stale)
        postprocess = dict(mode="HIGHWAY", reason=reason, validated_static=True,
            canonical_control=True,
            sensor_stamp=scan_stamp,
            validation=dict(valid=not hard_stale, pose_valid=True, steering_valid=True,
                blocked=emergency, reference_checked=bool(info['speed_plan'].get('reference_checked')),
                clearance_components_m=info.get('clearance_components_m'),
                sensor_times=dict(ego=ego.get('stamp'), lidar=scan_stamp,
                                  objects=fusion_stamp)),
            raw_peak_m=float(np.max(np.abs(predicted_path[:, 1]))), raw_end_m=float(predicted_path[-1, 1]),
            applied_peak_m=float(self.highway.target_offset), motion_state="STOP" if stopped else "DRIVE",
            stop_reason=reason if stopped else None, highway=info)
        self.zone_kmh, self.zone = self.highway.target_mps*3.6, self.highway.config
        self.last_hazard, self.last_yield = None, None
        self._n_obj, self._obj_debug = len(objects), []
        self._lidar_log = dict(n=len(lidar_points) if lidar_points is not None else None,
            age_s=round(lidar_age, 3), retained_near_field_n=self.near_field_memory.retained_n,
            path_block_m=info["clear_distance_m"])
        self.last_result = dict(path=path, raw_path=predicted_path, probabilities=probabilities,
            concept_values=concept_values, scene_matches=cosine_gallery_matches(scene, self.gallery_matrix, self.gallery) if scene.size else [],
            trajectory=trajectory_diagnostics(predicted_path, np.asarray(display_route)),
            hybrid_trajectory=trajectory_diagnostics(path, np.asarray(display_route)),
            postprocess=postprocess, local_route=np.asarray(display_route), model_route=np.asarray(model_route),
            latency_ms=latency, status=status, objects=objects, lane_corridor=corridor)
        if self.path_publisher:
            self.path_publisher.zone_mps = effective_target
            self.path_publisher.emergency_stop = emergency
            self.path_publisher.publish(ego, path, dense_route, postprocess)
            self.path_publisher.publish_camera_rates(self.receiver.rates())
            self.path_publisher.publish_corridor(ego, corridor)
        if not hasattr(self, "_decision_log"):
            sid = os.path.basename(os.environ.get("STUDENT_SESSION_DIR", "")) or "nosave"
            directory = os.environ.get("STUDENT_LOG_DIR", str(Path(self.args.status).parent))
            self._decision_log = AsyncDecisionLog(os.path.join(directory, f"student_gui_{sid}.jsonl"))
        record = dict(t=time.time(), ego=[round(ego["x"], 2), round(ego["y"], 2)],
            transition=dict(getattr(getattr(self, '_planner_transition', None), 'info', {})),
            speed_kmh=round(ego.get("speed_kmh", 0), 1), action=self.actions[int(np.argmax(probabilities))] if probabilities.size else "HIGHWAY",
            p=float(np.max(probabilities)) if probabilities.size else None, mode="HIGHWAY", reason=reason, stop_reason=postprocess["stop_reason"],
            motion_state=postprocess["motion_state"], target_kmh=self.zone_kmh, zone=self.zone,
            highway=info, lidar_vehicles=vehicle_status, n_obj=len(objects), object_stale=object_stale,
            emergency_stop=emergency, lidar=self._lidar_log, latency_ms=round(latency, 1),
            decision_ms=round((time.perf_counter()-started)*1000, 1),
            target_decision_hz=getattr(self, "decision_target_hz", 15.),
            decision_hz=round(getattr(self, "decision_hz", 0.), 1),
            planner_cap_kmh=round(effective_target*3.6, 1),
            published_cap_kmh=round(getattr(self.path_publisher, "_last_target_mps", 0.)*3.6, 1),
            ego_age_s=ego.get("age_s", 0.), ego_src=ego.get("src", "ros"), obj_src="semantic+pointpillars",
            route_dist_m=status["path"]["distance_m"], applied_peak_m=self.highway.target_offset,
            raw_peak_m=postprocess["raw_peak_m"], traffic_plan=info)
        source_stamp = float(ego.get('stamp', 0.))
        record['decision_source_age_s'] = (max(0., record['t']-source_stamp)
            if math.isfinite(source_stamp) and source_stamp > 0. else None)
        record["speed_label"] = speed_label(record)
        # Logged object/car velocities are measured (wall-clock) units; the
        # planner used them divided by this factor.
        record.update(self.sim_clock.record() if getattr(self, 'sim_clock', None) is not None
                      else dict(rtf=round(rtf, 3), rtf_measured=False))
        record["cars"] = self.surround_cars(ego)
        # Keep the exact collision inputs beside the emitted plan. Car-only
        # diagnostics cannot explain a stop caused by a fused obstacle track.
        record["object_tracks"] = list(objects)
        record["fused_observations"] = fused_observations
        if emergency and measured_speed > .3:
            record["checked_path_world"] = np.round(world_points(path, ego), 4).tolist()
        record["ego_h"] = round(float(ego.get("heading_deg", 0.)), 2)
        record["body_gap_m"] = body_gap_m(self.lidar)
        try:   # v6 summary (Claude working copy); a missing package records nothing
            from gui import v6_record_fields
            record.update(v6_record_fields(self))
        except Exception:
            pass
        self._web_record = record
        self._decision_log.write(json.dumps(record, default=json_default)+"\n")
        return True
