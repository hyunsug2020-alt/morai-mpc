"""Verify LiDAR-only tunnel decisions without ROS, cameras, or vehicle control."""
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

import test_avoidance_gate as gate_fixture
import tunnel_runtime


class TunnelMapFixture:
    def __init__(self, obstacle_points=(), *, valid=True, half_width=5., correction_y=0.):
        self.obstacle_points = np.asarray(obstacle_points, dtype=np.float32).reshape(-1, 2)
        self.valid = valid
        self.half_width = half_width
        self.correction_y = correction_y
        self.process_calls = 0

    def active(self, ego):
        return True

    def process(self, vehicle_xyz, ego, plane):
        self.process_calls += 1
        aligned = dict(ego, y=float(ego["y"]) + self.correction_y)
        return dict(valid=self.valid, reason="fixed tunnel wall match" if self.valid else "wall match rejected",
                    aligned_ego=aligned, obstacle_points=self.obstacle_points.copy(),
                    wall_stats=dict(left_n=30, right_n=30, lateral_correction_m=self.correction_y),
                    constraint=dict(valid=self.valid, active=True, map_heading_rad=0.,
                                    variance=dict(lateral=.03, yaw=.002)))

    def path_clearance(self, path, aligned_ego, horizon):
        path = np.asarray(path, dtype=float)[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        # The actual vehicle body must fit between the measured walls.
        usable = arc <= horizon
        bad = np.flatnonzero(usable & (np.abs(path[:, 1]) + .946 > self.half_width))
        return float(arc[bad[0]]) if len(bad) else float(horizon)

    def clear_distance(self, path, horizon=60.):
        return self.path_clearance(path, None, horizon)


class TunnelIntegrationTests(unittest.TestCase):
    def runtime(self, points=(), **map_options):
        runtime = gate_fixture.AvoidanceGateTests().runtime(action="YIELD", objects=[])
        runtime.tunnel = TunnelMapFixture(points, **map_options)
        runtime.args.cruise_kmh = 55.
        runtime.control_waypoint_xy = runtime.test_route.copy()
        runtime.waypoint_xy = runtime.test_route.copy()
        runtime.ego_receiver = SimpleNamespace(
            at_stamp=lambda stamp: dict(runtime.test_status["ego"], stamp=stamp))
        walls = np.array([(x, y, z) for x in np.arange(2., 33., .5)
                          for y in (-5., 5.) for z in (.5, 1., 1.5)], dtype=np.float32)
        azimuth = np.radians(np.arange(-180., 180., .25))
        ground_rays = np.column_stack((80. * np.cos(azimuth), 80. * np.sin(azimuth),
                                       np.zeros(len(azimuth)))).astype(np.float32)
        runtime.test_background_xyz = np.vstack((ground_rays, walls))
        objects_xyz = np.column_stack((runtime.tunnel.obstacle_points,
                                       np.full(len(runtime.tunnel.obstacle_points), .6))).astype(np.float32)
        xyz = np.vstack((runtime.test_background_xyz, objects_xyz))
        runtime.test_scan = dict(xyz=xyz, ground_plane=np.zeros(3), stamp=time.time(),
                                 sequence=1, age_s=.01)
        runtime.test_status["ego"]["stamp"] = runtime.test_scan["stamp"]
        runtime.test_odometry = dict(runtime.test_status["ego"])
        runtime.tunnel_odometry = SimpleNamespace(
            at_stamp=lambda stamp: dict(runtime.test_odometry, stamp=stamp))
        runtime.lidar.scan_snapshot = lambda: runtime.test_scan
        runtime.receiver.snapshot = Mock(side_effect=AssertionError("tunnel read camera input"))
        runtime.image_tensor = Mock(side_effect=AssertionError("tunnel prepared camera tensors"))
        runtime.ego_tensor = Mock(side_effect=AssertionError("tunnel prepared Student ego tensors"))
        runtime.model = Mock(side_effect=AssertionError("tunnel ran camera model"))
        runtime.path_publisher.publish = Mock(wraps=runtime.path_publisher.publish)
        runtime.path_publisher.publish_tunnel_constraint = Mock()
        return runtime

    @staticmethod
    def cluster(front=18.):
        return np.array([(x, y) for x in np.arange(front, front + .65, .2)
                         for y in (-.3, -.1, .1, .3)], dtype=np.float32)

    def assert_positive_target(self, runtime):
        self.assertTrue(runtime.infer())
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        runtime.receiver.snapshot.assert_not_called()
        runtime.image_tensor.assert_not_called()
        runtime.ego_tensor.assert_not_called()
        runtime.model.assert_not_called()

    def assert_fallback_drive(self, runtime):
        import tunnel_runtime
        runtime.infer()
        speed = runtime.path_publisher.velocity_pub.items[-1].data
        self.assertGreater(speed, 0.)
        self.assertLessEqual(speed, tunnel_runtime.TUNNEL_FALLBACK_KMH / 3.6 + 1e-6)
        self.assertIn('odometry fallback', runtime._web_record['reason'])
        runtime.model.assert_not_called()

    def assert_stopped(self, runtime):
        runtime.infer()
        self.assertTrue(runtime.path_publisher.velocity_pub.items)
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        runtime.model.assert_not_called()

    def test_empty_tunnel_uses_lidar_when_cameras_are_unavailable(self):
        self.assert_positive_target(self.runtime())

    def test_pointpillars_vehicle_tuple_is_planned_published_and_logged(self):
        runtime = self.runtime()
        runtime.highway = SimpleNamespace(config=dict(lidar_regions=[dict(from_idx=0, to_idx=10000)]),
                                         active=lambda index: False, phase='OUTSIDE')
        runtime.latest_status = runtime.test_status
        car = (12., 0., 4.8, 'lidar_car', 'npc', 0., 4.8, 1.9, 0.,
               0., 0., 500013, 0., None, False, 0.)
        runtime.vehicle_receiver = SimpleNamespace(snapshot=Mock(return_value=([car], True, {})))
        self.assertTrue(runtime.infer())
        self.assertTrue(runtime.path_publisher.velocity_pub.items)
        record = runtime._web_record
        logged = next(o for o in record['tunnel']['objects'] if o['name'] == 'lidar_car')
        self.assertEqual(logged['id'], 500013)
        self.assertEqual(logged['length_m'], 4.8)
        self.assertIsNone(logged['point_count'])
        json.dumps(record)

    @staticmethod
    def narrow_passage(centre=0., front=2., width=2.3):
        left, right = centre-width/2, centre+width/2
        sides = np.r_[np.arange(-5., left, .1), left, right,
                      np.arange(right+.1, 5.01, .1)]
        return np.array([(x, y) for x in np.arange(front, 23., .5) for y in sides])

    def stopped_runtime(self, points):
        runtime = self.runtime(points)
        runtime.test_status['ego']['speed_kmh'] = 0.
        runtime.test_odometry['speed_kmh'] = 0.
        return runtime

    def test_tight_passage_keeps_checked_margin_and_five_kmh_cap(self):
        runtime = self.stopped_runtime(self.narrow_passage())
        self.assert_positive_target(runtime)
        row = runtime._web_record
        self.assertFalse(row['emergency_stop'], row['stop_reason'])
        self.assertFalse(row['tunnel']['bypass_blocked'])
        self.assertGreaterEqual(row['tunnel']['clearance_components_m']['obstacle'], 40.)
        self.assertEqual(row['tunnel']['plan']['side_margin_m'], .12)
        self.assertTrue(row['tunnel']['plan']['tight_pass'])
        self.assertLessEqual(row['planner_cap_kmh'], 5.)
        self.assertLessEqual(runtime.path_publisher.velocity_pub.items[-1].data, 5./3.6)

    def test_small_offset_tight_passage_is_rechecked_with_its_margin(self):
        runtime = self.stopped_runtime(self.narrow_passage(centre=.25, front=8.))
        self.assert_positive_target(runtime)
        row = runtime._web_record
        self.assertFalse(row['emergency_stop'], row['stop_reason'])
        self.assertGreater(abs(row['applied_peak_m']), 0.)
        self.assertGreaterEqual(row['tunnel']['clearance_components_m']['obstacle'], 40.)
        self.assertEqual(row['tunnel']['plan']['side_margin_m'], .12)
        self.assertLessEqual(row['planner_cap_kmh'], 5.)

    def test_return_already_beside_body_does_not_invalidate_forward_path(self):
        runtime = self.stopped_runtime([[1., 1.1]])
        self.assert_positive_target(runtime)
        self.assertFalse(runtime._web_record['emergency_stop'])

    def test_tight_passage_does_not_allow_physical_contact(self):
        self.assert_stopped(self.stopped_runtime(self.narrow_passage(width=1.85)))
        self.assert_stopped(self.stopped_runtime([[1., .99]]))

    def test_tight_passage_remains_capped_while_moving_then_stops_for_new_contact(self):
        runtime = self.stopped_runtime(self.narrow_passage())
        self.assert_positive_target(runtime)
        first = runtime.test_scan['stamp']
        clock = runtime._tunnel_lattice_prev['stamp']
        for step, speed in ((1, 3.), (2, 5.)):
            runtime.test_status['ego'].update(speed_kmh=speed, stamp=first+.1*step)
            runtime.test_scan = dict(runtime.test_scan, stamp=first+.1*step, sequence=step+1)
            with patch('tunnel_runtime.time.monotonic', return_value=clock+.1*step):
                self.assert_positive_target(runtime)
            self.assertLessEqual(runtime._web_record['planner_cap_kmh'], 5.)
        runtime.tunnel.obstacle_points = np.vstack((runtime.tunnel.obstacle_points, [[1., 0.]]))
        runtime.test_scan = dict(runtime.test_scan, stamp=first+.3, sequence=4)
        runtime.test_status['ego']['stamp'] = first+.3
        with patch('tunnel_runtime.time.monotonic', return_value=clock+.3):
            self.assert_stopped(runtime)

    def test_stale_semantic_objects_do_not_stop_lidar_only_tunnel(self):
        runtime = self.runtime()
        runtime.object_receiver.age_s = 99.
        runtime.object_receiver.received = time.monotonic() - 99.
        runtime.object_receiver.objects = [(3., 0., 2., "obsolete_semantic_box", "obstacle", 0., 2., 2., 0.)]
        self.assert_positive_target(runtime)

    def test_real_lidar_obstacle_causes_checked_bypass(self):
        runtime = self.runtime(self.cluster())
        self.assert_positive_target(runtime)
        self.assertTrue(runtime.last_result["postprocess"]["validated_static"])
        path = np.asarray(runtime.last_result["path"])
        self.assertGreater(float(np.max(np.abs(path[:, 1]))), 1.)
        self.assertLess(float(np.max(np.abs(path[:, 1]))) + .946, runtime.tunnel.half_width + .05)

    def test_no_wall_safe_bypass_stops(self):
        self.assert_stopped(self.runtime(self.cluster(front=8.), half_width=1.2))

    def test_distant_blocked_bypass_brakes_with_available_room(self):
        runtime = self.runtime(self.cluster(), half_width=1.2)
        self.assertTrue(runtime.infer())
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertLess(runtime.path_publisher.velocity_pub.items[-1].data, 28.94 / 3.6)
        self.assertTrue(runtime._web_record["tunnel"]["bypass_blocked"])

    def test_immediate_obstacle_stops(self):
        self.assert_stopped(self.runtime(self.cluster(front=3.)))

    def test_confirmed_near_obstacle_survives_vlp16_blind_zone(self):
        runtime = self.runtime(self.cluster(front=6.), half_width=1.2)
        self.assert_stopped(runtime)
        runtime.test_scan = dict(runtime.test_scan, sequence=runtime.test_scan["sequence"] + 1,
                                 stamp=runtime.test_scan["stamp"] + .1)
        runtime.test_status["ego"]["stamp"] = runtime.test_scan["stamp"]
        self.assert_stopped(runtime)
        runtime.tunnel.obstacle_points = np.empty((0, 2), dtype=np.float32)
        runtime.test_scan = dict(runtime.test_scan, sequence=runtime.test_scan["sequence"] + 1,
                                 stamp=runtime.test_scan["stamp"] + .1,
                                 xyz=runtime.test_background_xyz.copy())
        runtime.test_status["ego"]["stamp"] = runtime.test_scan["stamp"]
        self.assert_stopped(runtime)
        self.assertGreater(runtime._tunnel_memory.retained_n, 0)
        self.assertGreater(runtime._web_record["n_obj"], 0)

    def test_missing_raw_lidar_stops(self):
        runtime = self.runtime()
        runtime.test_scan = {}
        self.assert_stopped(runtime)

    def test_stale_raw_lidar_stops(self):
        runtime = self.runtime()
        runtime.test_scan["age_s"] = 2.
        self.assert_stopped(runtime)

    def test_recent_receipt_with_old_source_stamp_stops(self):
        runtime = self.runtime()
        runtime.test_scan["stamp"] -= 1.
        self.assert_stopped(runtime)
        self.assertEqual(runtime.tunnel.process_calls, 0)

    def test_source_stamp_ahead_of_localization_stops(self):
        runtime = self.runtime()
        runtime.test_scan["stamp"] += .5
        self.assert_stopped(runtime)
        self.assertEqual(runtime.tunnel.process_calls, 0)

    # Live 2026-10-03: a failed wall match held the car for minutes. These
    # now continue on ESKF + raw LiDAR at TUNNEL_FALLBACK_KMH.
    def test_lidar_pose_alignment_missing_falls_back(self):
        runtime = self.runtime()
        runtime.ego_receiver.at_stamp = lambda stamp: None
        runtime.test_status['ego']['stamp'] += .13
        self.assert_fallback_drive(runtime)

    def test_fallback_still_stops_for_raw_lidar_obstacle(self):
        runtime = self.runtime(valid=False)
        runtime.test_scan = dict(runtime.test_scan,
            xyz=np.vstack((runtime.test_scan['xyz'], [[3.5, 0., .6], [3.5, .2, .8]])))
        self.assert_stopped(runtime)

    def test_recent_scan_pose_can_use_current_localization_and_pure_odometry(self):
        runtime = self.runtime()
        runtime.ego_receiver.at_stamp = lambda stamp: None
        runtime.test_status['ego']['stamp'] += .05
        self.assert_positive_target(runtime)

    def test_brief_wall_occlusion_uses_fresh_raw_points_with_bounded_old_pose(self):
        from tunnel_map import TunnelMap
        runtime = self.runtime()
        runtime.tunnel = TunnelMap(dict(origin_xy=[0., 0.], axis_heading_deg=0., left_offset_m=5.,
            right_offset_m=-5., longitudinal_min_m=-50., longitudinal_max_m=150.))
        self.assert_positive_target(runtime)
        constraint_calls = runtime.path_publisher.publish_tunnel_constraint.call_count
        original_stamp = runtime.test_scan['stamp']
        xyz = runtime.test_background_xyz
        # Remove one wall while preserving current ground rays and the other wall.
        one_wall = xyz[~((abs(xyz[:, 1]-5.) < .01) & (xyz[:, 2] > .1))]
        runtime.test_scan = dict(runtime.test_scan, xyz=one_wall, stamp=original_stamp+.1)
        runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
        self.assert_positive_target(runtime)
        self.assertEqual(runtime._web_record['tunnel']['reason'], 'recent tunnel wall pose propagated')
        self.assertEqual(runtime.path_publisher.publish_tunnel_constraint.call_count, constraint_calls)
        # A fresh obstacle must still force a stop during the wall occlusion.
        runtime.test_scan = dict(runtime.test_scan, stamp=original_stamp+.2,
            xyz=np.vstack((one_wall, [[3., 0., .6]])))
        runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
        self.assert_stopped(runtime)
        self.assertGreater(runtime._web_record['n_obj'], 0)
        # Successful propagation cannot extend the original measurement's age;
        # the car then continues on ESKF odometry + raw LiDAR (no tunnel stop).
        runtime.test_scan = dict(runtime.test_scan, xyz=one_wall, stamp=original_stamp+.3)
        runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
        self.assert_fallback_drive(runtime)
        self.assertIn('wall match unavailable', runtime._web_record['reason'])

    def test_pure_odometry_missing_falls_back(self):
        runtime = self.runtime()
        runtime.tunnel_odometry.at_stamp = lambda stamp: None
        self.assert_fallback_drive(runtime)

    def test_near_field_memory_remains_fixed_when_eskf_wall_correction_changes(self):
        runtime = self.runtime(self.cluster(front=6.), half_width=1.2)
        self.assert_stopped(runtime)
        for i in range(1, 3):
            runtime.test_scan = dict(runtime.test_scan, stamp=runtime.test_scan['stamp']+.1)
            runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
            self.assert_stopped(runtime)
        stored = {key: observation[2].copy() for key, observation in runtime._tunnel_memory.cells.items()}
        self.assertTrue(stored)
        runtime.test_status['ego']['y'] = .3
        runtime.test_scan = dict(runtime.test_scan, stamp=runtime.test_scan['stamp']+.1)
        runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
        self.assert_stopped(runtime)
        self.assertEqual(set(stored), set(runtime._tunnel_memory.cells))
        for key, point in stored.items():
            np.testing.assert_allclose(runtime._tunnel_memory.cells[key][2], point)

    def test_wall_match_rejected_falls_back(self):
        self.assert_fallback_drive(self.runtime(valid=False))

    def recorded_runtime(self, filename):
        from gui import Runtime, HdMapLanes, smooth_control_route
        from lidar_corridor import LidarCorridor
        from tunnel_map import TunnelMap
        from tunnel_runtime import pose_at_stamp
        root = Path(__file__).parent
        data = np.load(root/'test_data'/filename)
        meta = json.loads(str(data['meta']))
        ego = pose_at_stamp(meta['poses'], meta['stamp'])
        runtime = self.runtime()
        # Replays use the wall-map frame live when they were recorded (before the 2026-10-10 re-alignment).
        runtime.tunnel = TunnelMap.load(root/'tunnel_wall_map.json', survey_frame=True)
        runtime.waypoints = json.loads((root/'global_path.json').read_text())['waypoints']
        runtime.waypoint_xy = np.array([[p['x'], p['y']] for p in runtime.waypoints])
        runtime.control_waypoint_xy = smooth_control_route(runtime.waypoint_xy)
        runtime.lanes = HdMapLanes(root/'link_set.json')
        runtime.lanes.set_route(runtime.waypoint_xy)
        runtime.config['route_points'] = 20
        runtime.route_tensor = lambda current, status: Runtime.route_tensor(runtime, current, status)
        runtime.lidar = LidarCorridor(x_range=(-75., 100.))
        runtime.lidar.ingest(data['xyz'], stamp=meta['stamp'])
        runtime.ego_receiver = SimpleNamespace(at_stamp=lambda stamp: pose_at_stamp(meta['poses'], stamp))
        runtime.tunnel_odometry = SimpleNamespace(
            at_stamp=lambda stamp: pose_at_stamp(meta.get('odometry', meta['poses']), stamp))
        runtime.object_receiver.objects = meta['semantic']
        cars = [(o['position'][0], o['position'][1], max(o['size'][:2]), 'lidar_car', 'npc',
                 math.hypot(*o['velocity']), o['size'][0], o['size'][1], o['heading_deg'],
                 o['velocity'][0], o['velocity'][1], o['uid'], None,
                 o.get('motion_heading_deg'), o['velocity_valid']) for o in meta['cars']['vehicles']]
        runtime.tunnel_vehicle_objects = lambda *args: cars
        index = int(np.argmin(np.linalg.norm(runtime.waypoint_xy-[ego['x'], ego['y']], axis=1)))
        runtime.test_status = dict(ego=ego, path=dict(nearest_idx=index, distance_m=0.))
        return runtime

    def test_recorded_rejected_pose_bypass_uses_its_own_clearance_and_speed(self):
        runtime = self.recorded_runtime('tunnel_pose_rejected_20261008.npz')
        with patch.object(runtime.tunnel, 'relative_projection', return_value=None), \
                patch('tunnel_runtime.time.monotonic', return_value=runtime.lidar.received):
            self.assert_fallback_drive(runtime)
        row = runtime._web_record
        self.assertFalse(row['tunnel']['valid'])
        self.assertGreater(row['tunnel']['wall_stats']['lateral_correction_m'], .8)
        runtime.path_publisher.publish_tunnel_constraint.assert_not_called()
        self.assertIn('measured wall corridor', row['reason'])
        self.assertFalse(row['emergency_stop'])
        self.assertGreater(row['planner_cap_kmh'], 5.)

    def test_recorded_entry_compares_wide_left_before_accepting_short_right(self):
        filename = 'tunnel_wide_pass_entry_20261008.npz'
        narrow = self.recorded_runtime(filename)
        with patch('tunnel_runtime.TUNNEL_LATTICE_MAX_OFFSET_M', 3.5), \
                patch('tunnel_runtime.time.monotonic', return_value=narrow.lidar.received):
            self.assert_positive_target(narrow)
        before = narrow._web_record
        self.assertLess(before['applied_peak_m'], 0.)
        self.assertLess(before['tunnel']['geometry_clear_distance_m'], 15.)

        wide = self.recorded_runtime(filename)
        with patch('tunnel_runtime.time.monotonic', return_value=wide.lidar.received):
            self.assert_positive_target(wide)
        after = wide._web_record
        self.assertTrue(after['tunnel']['valid'])
        self.assertGreater(after['applied_peak_m'], 3.5)
        self.assertGreaterEqual(after['tunnel']['geometry_clear_distance_m'], 30.)
        self.assertGreaterEqual(after['tunnel']['observed_clear_distance_m'], 30.)
        self.assertFalse(after['tunnel']['plan']['tight_pass'])
        self.assertEqual(after['tunnel']['plan']['search_max_offset_m'], 5.)
        self.assertFalse(after['emergency_stop'])

    def test_recorded_tunnel_stop_uses_local_geometry_without_accepting_map_aiding(self):
        runtime = self.recorded_runtime('tunnel_semantic_stop_20261008.npz')
        runtime.object_receiver.snapshot = Mock(side_effect=AssertionError('camera fallback used'))
        with patch('tunnel_runtime.time.monotonic', return_value=runtime.lidar.received):
            self.assert_positive_target(runtime)
        row = runtime._web_record
        self.assertTrue(row['tunnel']['relative_geometry'])
        self.assertFalse(row['tunnel']['map_match_valid'])
        self.assertEqual(row['tunnel']['map_match_reason'], 'pose correction exceeds validated limit')
        self.assertGreater(row['tunnel']['wall_stats']['lateral_correction_m'], .8)
        runtime.path_publisher.publish_tunnel_constraint.assert_not_called()
        runtime.object_receiver.snapshot.assert_not_called()
        self.assertFalse(row['emergency_stop'])
        self.assertGreaterEqual(row['tunnel']['geometry_clear_distance_m'], 30.)
        self.assertGreater(row['planner_cap_kmh'], 5.)

    def test_local_wall_recovery_preserves_a_new_measured_blocker(self):
        runtime = self.recorded_runtime('tunnel_semantic_stop_20261008.npz')
        scan = runtime.lidar.scan_snapshot()
        barrier = np.array([[3., y, .6] for y in np.arange(-9., 11., .15)])
        scan = dict(scan, xyz=np.vstack((scan['xyz'], barrier)))
        runtime.lidar.scan_snapshot = lambda: scan
        with patch('tunnel_runtime.time.monotonic', return_value=runtime.lidar.received):
            self.assert_stopped(runtime)
        runtime.path_publisher.publish_tunnel_constraint.assert_not_called()

    def test_stale_localization_stops(self):
        runtime = self.runtime()
        runtime.test_status["ego"]["age_s"] = 2.
        runtime.input_error = "localization stale"
        self.assert_stopped(runtime)

    def test_valid_scan_recovers_after_wall_match_stop(self):
        runtime = self.runtime(valid=False)
        self.assert_fallback_drive(runtime)
        runtime.tunnel.valid = True
        runtime.test_scan = dict(runtime.test_scan, sequence=2, stamp=runtime.test_scan["stamp"] + .1)
        self.assert_positive_target(runtime)
        self.assertFalse(runtime.path_publisher.emergency_stop)

    def test_same_scan_reuses_wall_fit(self):
        runtime = self.runtime()
        self.assert_positive_target(runtime)
        self.assert_positive_target(runtime)
        self.assertEqual(runtime.tunnel.process_calls, 1)
        self.assertEqual(runtime.path_publisher.publish_tunnel_constraint.call_count, 1)

    @staticmethod
    def delayed_planners(runtime, clock, seconds, check_constraint=False):
        """Delay whichever tunnel planner is active (geometric or lattice)."""
        import gui
        import lattice_planner
        geometric, lattice = gui.plan_static_bypass, lattice_planner.plan
        def wrap(planner):
            def delayed(*args, **kwargs):
                if check_constraint:
                    runtime.path_publisher.publish_tunnel_constraint.assert_called_once()
                clock[0] += seconds
                return planner(*args, **kwargs)
            return delayed
        return (patch('gui.plan_static_bypass', side_effect=wrap(geometric)),
                patch('lattice_planner.plan', side_effect=wrap(lattice)))

    def test_wall_aiding_precedes_even_a_delayed_path_search(self):
        runtime = self.runtime()
        clock = [100.]
        geometric, lattice = self.delayed_planners(runtime, clock, .7, check_constraint=True)
        with patch('tunnel_runtime.time.perf_counter', side_effect=lambda: clock[0]), geometric, lattice:
            # A slow search no longer stops the car (live 39->0 km/h at the
            # tunnel entry); its travel is charged against the clearance.
            self.assert_positive_target(runtime)

    def test_late_but_checked_frame_keeps_driving(self):
        # 2026-10-03: frames of 0.25-0.35 s every ~2 s each sent a stop pulse
        # that restarted the speed ramp; the car crawled with a 35 km/h plan.
        runtime = self.runtime()
        clock = [100.]
        geometric, lattice = self.delayed_planners(runtime, clock, .3)
        with patch('tunnel_runtime.time.perf_counter', side_effect=lambda: clock[0]), geometric, lattice:
            self.assert_positive_target(runtime)
        self.assertIsNone(runtime._web_record['stop_reason'])

    def test_stop_diagnostics_separate_raw_obstacle_from_coverage(self):
        runtime = self.runtime(self.cluster(front=3.))
        self.assert_stopped(runtime)
        row = runtime._web_record
        self.assertEqual(row['stop_reason'], 'tunnel obstacle on path')
        self.assertEqual(row['tunnel']['limiting_source'], 'obstacle')
        self.assertGreater(row['tunnel']['clearance_components_m']['observed'], 2.)
        self.assertTrue(row['tunnel']['objects'])

    @patch('tunnel_runtime.TUNNEL_LATTICE', False)   # geometric planner's saved-plan release
    def test_completed_rejoin_releases_saved_plan_only_when_base_route_is_clear(self):
        for blocked in (False, True):
            with self.subTest(blocked=blocked):
                runtime = self.runtime(self.cluster(front=18.) if blocked else ())
                path = runtime.test_route.copy()
                # A small, previously validated return can still be saved after
                # the vehicle has reached the route. Base clearance gates release.
                path[:, 1] += 2.
                plan = dict(path=path, kind='rejoin', direction=1., offset_m=.45,
                            curvature=.02, speed_cap_mps=5.)
                with patch('gui.plan_static_bypass', return_value=(plan, [])):
                    runtime.infer()
                row = runtime._web_record
                self.assertEqual(row['tunnel']['rejoin_complete'], not blocked)
                self.assertEqual(row['mode'], 'TUNNEL BYPASS' if blocked else 'TUNNEL')
                if not blocked:
                    self.assertIsNone(runtime._tunnel_plan_saved)

    def test_dynamic_lidar_crossing_yields_without_camera_model(self):
        runtime = self.runtime()
        first_stamp = runtime.test_scan["stamp"]
        for i in range(4):
            points = self.cluster(front=10.) + (0., -3. + .2 * i)
            runtime.tunnel.obstacle_points = points
            objects_xyz = np.column_stack((points, np.full(len(points), .6))).astype(np.float32)
            stamp = first_stamp + .1 * i
            runtime.test_scan = dict(runtime.test_scan, xyz=np.vstack((runtime.test_background_xyz, objects_xyz)),
                                     stamp=stamp, sequence=i + 1)
            runtime.test_status["ego"]["stamp"] = stamp
            self.assertTrue(runtime.infer())
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertIsNotNone(runtime.last_yield)
        self.assertEqual(runtime.last_yield["kind"], "stop")
        self.assertTrue(runtime.last_yield["name"].startswith("tunnel_lidar_dynamic_"))
        runtime.receiver.snapshot.assert_not_called()
        runtime.model.assert_not_called()

    def test_same_scan_points_follow_pure_odometry_motion(self):
        runtime = self.runtime(self.cluster(front=40.))
        self.assert_positive_target(runtime)
        runtime.test_status["ego"].update(x=.5, stamp=runtime.test_scan["stamp"] + 1. / 15.)
        runtime.test_odometry["x"] = .5
        with patch("tunnel_runtime.lidar_objects", wraps=tunnel_runtime.lidar_objects) as measured:
            self.assert_positive_target(runtime)
        body_points = np.asarray(measured.call_args_list[0].args[0])
        self.assertAlmostEqual(float(body_points[:, 0].min()), 39.5, places=4)
        self.assertEqual(runtime.tunnel.process_calls, 1)

    def test_same_scan_does_not_double_apply_eskf_wall_correction(self):
        runtime = self.runtime(correction_y=.8)
        self.assert_positive_target(runtime)
        runtime.test_status["ego"]["y"] = .8
        self.assert_positive_target(runtime)
        self.assertEqual(runtime.tunnel.process_calls, 1)
        self.assertAlmostEqual(runtime._web_record["tunnel"]["aligned_ego"]["y"], .8)

    def test_frozen_source_stamp_stops_despite_recent_receipt(self):
        runtime = self.runtime()
        self.assert_positive_target(runtime)
        later = runtime._tunnel_cache["received"] + .3
        with patch("tunnel_runtime.time.monotonic", return_value=later):
            # A 0.3 s freeze is a CPU-load glitch: keep going on the raw scan.
            self.assert_fallback_drive(runtime)
        with patch("tunnel_runtime.time.monotonic", return_value=later + .5):
            self.assert_stopped(runtime)
        self.assertEqual(runtime.tunnel.process_calls, 1)

    def test_clock_reset_discards_old_scan_and_manoeuvre(self):
        runtime = self.runtime(self.cluster())
        self.assert_positive_target(runtime)
        self.assertIsNotNone(runtime._tunnel_plan_saved)
        runtime.test_scan["stamp"] -= 1.
        runtime.test_status["ego"]["stamp"] = runtime.test_scan["stamp"]
        self.assert_stopped(runtime)
        self.assertIsNone(runtime._tunnel_cache)
        self.assertIsNone(runtime._tunnel_plan_saved)
        self.assertFalse(hasattr(runtime, "_tunnel_memory"))
        self.assertEqual(runtime._tunnel_tracks.tracks, {})

    def test_new_scan_updates_wall_fit(self):
        runtime = self.runtime()
        self.assert_positive_target(runtime)
        runtime.test_scan = dict(runtime.test_scan, sequence=2, stamp=runtime.test_scan["stamp"] + .1)
        self.assert_positive_target(runtime)
        self.assertEqual(runtime.tunnel.process_calls, 2)

    def test_published_path_uses_follower_localization_frame(self):
        runtime = self.runtime(correction_y=.8)
        self.assert_positive_target(runtime)
        call = runtime.path_publisher.publish.call_args
        published_pose = call.args[0]
        self.assertEqual(published_pose["y"], runtime.test_status["ego"]["y"])
        payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        self.assertGreater(len(payload["waypoints"]), 15)

    def test_outside_tunnel_retains_camera_input_requirement(self):
        runtime = self.runtime()
        runtime.tunnel.active = lambda ego: False
        runtime.receiver.snapshot = Mock(return_value=None)
        self.assertFalse(runtime.infer())
        runtime.receiver.snapshot.assert_called_once()
        self.assertEqual(runtime.tunnel.process_calls, 0)
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)


if __name__ == "__main__":
    unittest.main()
