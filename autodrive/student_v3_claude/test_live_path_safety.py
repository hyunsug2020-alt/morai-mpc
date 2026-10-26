"""The real publisher applies live safety without another model/planner call."""
import json
import copy
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np

from driving_plan import DrivingPlan
from live_path_safety import LivePathSafety, measured_people
from test_avoidance_gate import AvoidanceGateTests
from test_ped_guard import person
from test_tunnel_integration import TunnelIntegrationTests
from test_junction_entry import car
from gui import HdMapLanes
from lane_corridor import LaneCorridor


class LivePathSafetyTests(unittest.TestCase):
    def test_person_geometry_requires_compact_associated_fresh_returns(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        obj = person(6., -2., 0., 0.)
        batch = [('semantic', [obj], 100., .3)]
        points = np.array([[5.95, -2.05], [6., -2.], [6.05, -1.95]])
        rows = measured_people(points, ego, batch, 100.2, .1)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['uid'], 16)
        np.testing.assert_allclose(rows[0]['xy'], [6., -2.])
        self.assertEqual(measured_people(points+3., ego, batch, 100.2, .1), [])
        self.assertEqual(measured_people(points[:2], ego, batch, 100.2, .1), [])
        self.assertEqual(measured_people(points, ego, batch, 99.9, .1), [])

    def recorded_postavoid_guard(self):
        data = json.loads((Path(__file__).parent/'test_data/postavoid_camera_dropout.json').read_text())
        provider = LivePathSafety(SimpleNamespace())
        for batch in data['batches']:
            provider.guard.memory.update(batch['source'], batch['objects'], batch['stamp'],
                                         batch['age_s'], batch['now'])
        return provider, data

    def test_recorded_postavoid_camera_dropout_keeps_fresh_measured_geometry(self):
        provider, data = self.recorded_postavoid_guard()
        guard, now = provider.guard, data['now']
        track = next(iter(guard.memory.tracks.values()))
        original_stamp, history = track['stamp'], list(track['history'])
        kwargs = dict(requested_mps=30./3.6, raw_clear_m=60.,
                      source_age_s=data['lidar_age_s'], dynamic_only=True)
        # The recorded old semantic age stopped this physically clear pass by a
        # departure-readiness stop for a person 0.4 m behind our rear bumper and
        # 1.4 m off our side. Braking cannot protect a person beside or behind the
        # body, so that check no longer applies there (ped_guard.BESIDE_*).
        old = copy.deepcopy(guard).check(data['path'], data['ego'], [], now,
                                         measured_people=[], **kwargs)
        self.assertFalse(old.emergency, old.reason)
        fresh = provider.person_measurements(data['lidar_points'], data['ego'], [],
            data['lidar_stamp'], data['lidar_age_s'], now)
        self.assertEqual([row['uid'] for row in fresh], [192])
        result = guard.check(data['path'], data['ego'], [], now, measured_people=fresh, **kwargs)
        self.assertFalse(result.emergency, result.reason)
        self.assertGreater(result.cap_mps*3.6, data['ego']['speed_kmh'])
        self.assertEqual(track['stamp'], original_stamp)
        self.assertEqual(len(track['history']), len(history))
        np.testing.assert_array_equal(track['velocity'], [-.033455684213935334, .42127753298028253])
        # A fresh associated person cannot release an actual raw body overlap.
        blocked = guard.check(data['path'], data['ego'], [], now,
            measured_people=fresh, **dict(kwargs, raw_clear_m=.5))
        self.assertTrue(blocked.emergency)
        self.assertEqual(blocked.cap_mps, 0.)

    def test_retained_person_association_cannot_renew_expired_motion_or_old_lidar(self):
        provider, data = self.recorded_postavoid_guard()
        fresh = provider.person_measurements(data['lidar_points'], data['ego'], [],
            data['lidar_stamp'], data['lidar_age_s'], data['now'])
        self.assertTrue(fresh)
        track = next(iter(provider.guard.memory.tracks.values()))
        self.assertEqual(provider.person_measurements(data['lidar_points'], data['ego'], [],
            track['stamp']-.1, .1, data['now']), [])
        self.assertEqual(provider.person_measurements(data['lidar_points'], data['ego'], [],
            data['lidar_stamp'], data['lidar_age_s']+.1, track['observed_at']+.61), [])

    def runtime(self):
        runtime = AvoidanceGateTests().runtime(action='KEEP')
        runtime.test_status['path']['nearest_idx'] = 900
        runtime.test_status['ego'].update(stamp=100., speed_kmh=30.)
        runtime.speed_zones = SimpleNamespace(kmh_at=lambda idx: (30., {}))
        runtime.test_scan = dict(xyz=np.empty((0, 3)), stamp=99.9, sequence=1,
                                 age_s=.1, ground_plane=np.zeros(3))
        return runtime

    def attach(self, runtime):
        if not hasattr(runtime, 'ego_receiver'):
            runtime.ego_receiver = SimpleNamespace()
        runtime.ego_receiver.snapshot = lambda count: (dict(runtime.test_status['ego']), [], 20.)
        runtime.ego_receiver.at_stamp = lambda stamp: dict(runtime.test_status['ego'], stamp=stamp)
        runtime.lidar.scan_snapshot = lambda: runtime.test_scan
        runtime.object_receiver.observation_stamp = runtime.test_scan['stamp']
        runtime.path_publisher._command_lock = threading.RLock()
        runtime.path_publisher._safety_provider = LivePathSafety(runtime)
        return runtime

    def payload(self, runtime):
        return json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)

    def advance(self, runtime, dt, objects):
        runtime.test_status['ego']['stamp'] += dt
        # Advance both LiDAR interfaces used by the production decision loop.
        # The source-stamped snapshot and the legacy receipt-age accessor must
        # describe the same arriving scan, including a simulated clock advance.
        runtime.lidar.received += dt
        runtime.test_scan = dict(runtime.test_scan, stamp=runtime.test_scan['stamp']+dt,
                                 sequence=runtime.test_scan['sequence']+1)
        runtime.object_receiver.observation_stamp = runtime.test_scan['stamp']
        runtime.object_receiver.objects = list(objects)

    def test_general_road_publishes_one_matching_geometry_cap_and_sensor_plan(self):
        runtime = self.attach(self.runtime())
        runtime.infer()
        payload = self.payload(runtime)
        plan = runtime.path_publisher._active_decision_plan
        np.testing.assert_array_equal(np.array([[p['x'], p['y']] for p in payload['waypoints']]), plan.path)
        self.assertEqual(payload['plan_id'], payload['decision']['plan_id'])
        self.assertEqual(payload['target_speed_mps'], payload['decision']['allowed_mps'])
        self.assertEqual(payload['decision']['validation']['sensor_times']['objects'], runtime.object_receiver.observation_stamp)
        self.assertTrue(plan.checks_match_path())

    def test_new_complete_object_frame_resolves_only_the_old_empty_road_stream_stop(self):
        for case in ('fresh', 'fresh_planning_area', 'same_stamp', 'still_stale', 'person', 'raw_contact'):
            with self.subTest(case=case):
                runtime = self.attach(self.runtime())
                runtime.test_status['path']['nearest_idx'] = 900 if case == 'fresh_planning_area' else 1400
                runtime.test_status['ego']['speed_kmh'] = 27.
                runtime.object_receiver.age_s = .55
                runtime.object_receiver.observation_stamp = 99.4
                provider = runtime.path_publisher._safety_provider
                def arriving_frame(world, ego, mode, original):
                    self.assertIsNotNone(original.validation.get('object_stream_recheck'))
                    runtime.object_receiver.age_s = .51 if case == 'still_stale' else .1
                    runtime.object_receiver.observation_stamp = 99.4 if case == 'same_stamp' else 99.9
                    if case == 'person':
                        runtime.object_receiver.objects = [person(3.9, 0., 0., 0.)]
                    if case == 'raw_contact':
                        runtime.test_scan['xyz'] = np.array([[3.8, 0., .6]])
                    return provider(world, ego, mode, original)
                runtime.path_publisher._safety_provider = arriving_frame
                self.assertTrue(runtime.infer())
                plan = runtime.path_publisher._active_decision_plan
                if case in ('fresh', 'fresh_planning_area'):
                    self.assertFalse(plan.emergency, plan.stop_reason)
                    self.assertGreater(plan.allowed_mps, 0.)
                    self.assertGreater(plan.allowed_mps, 27./3.6)
                    self.assertLessEqual(plan.allowed_mps, 30./3.6)
                    self.assertEqual(plan.validation['object_stream_recovered'],
                                     dict(previous_stamp=99.4, source_stamp=99.9))
                    self.assertGreater(runtime.path_publisher._safety_baseline['target'], 0.)
                else:
                    self.assertTrue(plan.emergency, plan.stop_reason)
                    self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_departure_stop_is_sent_without_waiting_for_model_or_path_search(self):
        runtime = self.attach(self.runtime())
        runtime.infer()
        previous = self.payload(runtime)
        self.advance(runtime, .1, [person(13.8, -1.7, 0., 0.)])
        runtime.model = lambda *args: self.fail('safety waited for camera/model inference')
        self.assertTrue(runtime.path_publisher.recheck_active_path())
        payload = self.payload(runtime)
        self.assertEqual(payload['waypoints'], previous['waypoints'])
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertTrue(payload['decision']['emergency'])
        self.assertEqual(payload['plan_id'], payload['decision']['plan_id'])

    def test_safety_thread_stops_while_camera_inference_is_blocked(self):
        runtime = self.attach(self.runtime())
        runtime.infer()
        entered, release = threading.Event(), threading.Event()
        original_model = runtime.model
        def slow_model(*args):
            entered.set()
            if not release.wait(2.):
                raise RuntimeError('test model release timed out')
            return original_model(*args)
        runtime.model = slow_model
        publisher = runtime.path_publisher
        worker = threading.Thread(target=runtime.infer)
        publisher.start_safety(publisher._safety_provider)
        try:
            worker.start()
            self.assertTrue(entered.wait(1.))
            self.advance(runtime, .1, [person(13.8, -1.7, 0., 0.)])
            deadline = time.monotonic()+.4
            while self.payload(runtime)['target_speed_mps'] > 0. and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)
            self.assertFalse(release.is_set())
        finally:
            publisher._safety_stop.set()
            publisher._safety_thread.join(timeout=1.)
            release.set()
            worker.join(timeout=2.)
        self.assertFalse(worker.is_alive())

    def test_stop_clear_restart_reuses_geometry_and_ramps_speed(self):
        runtime = self.attach(self.runtime())
        runtime.infer()
        now = time.monotonic()
        previous = self.payload(runtime)
        self.advance(runtime, .1, [person(13.8, -1.7, 0., 0.)])
        with patch('time.monotonic', return_value=now+.1):
            runtime.path_publisher.recheck_active_path()
        self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)
        runtime.test_status['ego']['speed_kmh'] = 0.
        self.advance(runtime, .1, [person(13.8, -8., 0., 0.)])
        with patch('time.monotonic', return_value=now+.2):
            runtime.path_publisher.recheck_active_path()
        self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)
        self.advance(runtime, .15, [person(13.8, -8., 0., 0.)])
        with patch('time.monotonic', return_value=now+.35):
            runtime.path_publisher.recheck_active_path()
        payload = self.payload(runtime)
        self.assertEqual(payload['waypoints'], previous['waypoints'])
        self.assertGreater(payload['target_speed_mps'], 0.)
        self.assertLessEqual(payload['target_speed_mps'], .4)

    def test_old_geometry_cannot_be_refreshed_forever_by_safety_thread(self):
        runtime = self.attach(self.runtime())
        runtime.infer()
        created = runtime.path_publisher._safety_baseline['created']
        with patch('time.monotonic', return_value=created+2.1):
            runtime.path_publisher.recheck_active_path()
        self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)

    def test_slow_search_keeps_checked_path_but_new_hazard_stops_immediately(self):
        runtime = self.attach(self.runtime())
        runtime.infer()
        publisher = runtime.path_publisher
        created = publisher._safety_baseline['created']
        previous = self.payload(runtime)
        self.advance(runtime, .1, [])
        with patch('time.monotonic', return_value=created+.8):
            publisher.recheck_active_path()
        self.assertGreater(self.payload(runtime)['target_speed_mps'], 0.)
        self.advance(runtime, .1, [person(13.8, -1.7, 0., 0.)])
        with patch('time.monotonic', return_value=created+.9):
            publisher.recheck_active_path()
        payload = self.payload(runtime)
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertEqual(payload['waypoints'], previous['waypoints'])

    def test_slow_search_does_not_authorize_stale_scan(self):
        runtime = self.attach(self.runtime())
        runtime.infer()
        publisher = runtime.path_publisher
        created = publisher._safety_baseline['created']
        runtime.test_scan['age_s'] = .7
        with patch('time.monotonic', return_value=created+.8):
            publisher.recheck_active_path()
        self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)

    def test_stale_semantics_stop_general_road_instead_of_authorizing_pass(self):
        runtime = self.attach(self.runtime())
        runtime.infer()
        runtime.object_receiver.age_s = .8
        runtime.path_publisher.recheck_active_path()
        self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)

    def test_general_road_guard_uses_the_planners_same_height_filtered_scan(self):
        runtime = self.attach(self.runtime())
        runtime.test_scan.update(xyz=np.array([[7., 0., 2.4]]), obstacle_points=np.empty((0, 2)))
        self.assertTrue(runtime.infer())
        self.assertFalse(self.payload(runtime)['decision']['emergency'])
        self.assertGreater(self.payload(runtime)['target_speed_mps'], 0.)
        # A current low obstacle is still an immediate collision constraint.
        self.advance(runtime, .1, [])
        runtime.test_scan.update(xyz=np.array([[7., 0., .6]]), obstacle_points=np.array([[7., 0.]]))
        runtime.path_publisher.recheck_active_path()
        self.assertTrue(self.payload(runtime)['decision']['emergency'])

    def test_dynamic_bypass_starts_while_decelerating_before_a_full_stop(self):
        runtime = self.attach(self.runtime())
        blocker = car(19., 0., 1., 0., 17)
        runtime.object_receiver.objects = [blocker]
        publisher = runtime.path_publisher
        publisher.zone_mps, publisher.emergency_stop = 30./3.6, False
        publisher.publish(runtime.test_status['ego'], runtime.test_route, runtime.test_route,
            dict(mode='LATTICE', reason='existing route', validated_static=True))
        self.assertFalse(self.payload(runtime)['decision']['emergency'])
        recovery = runtime.path_publisher._safety_provider.recovery_inputs(runtime.test_status['ego'], 'LATTICE')
        self.assertIsNotNone(recovery)
        self.assertEqual(recovery['ids'], [17])
        self.advance(runtime, .1, [blocker])
        with patch('gui.USE_LATTICE', True):
            self.assertTrue(runtime.infer())
        self.assertGreater(abs(runtime.last_result['postprocess']['applied_peak_m']), .5)
        self.assertGreater(self.payload(runtime)['target_speed_mps'], 0.)
        self.assertFalse(self.payload(runtime)['decision']['emergency'])

    def test_bypass_body_survives_fusion_class_and_id_change(self):
        runtime = self.attach(self.runtime())
        moving = list(car(19., 0., .6, 0., 17))
        moving[3] = 'fusion_npc'
        runtime.object_receiver.objects = [tuple(moving)]
        publisher = runtime.path_publisher
        publisher.zone_mps, publisher.emergency_stop = 30./3.6, False
        publisher.publish(runtime.test_status['ego'], runtime.test_route, runtime.test_route,
            dict(mode='LATTICE', reason='checked bypass', validated_static=True, dynamic_recovery_ids=[17]))
        parked = list(moving)
        parked[3], parked[4], parked[5], parked[9], parked[10], parked[11] = 'fusion_parked_vehicle', 'obstacle', 0., 0., 0., 32
        self.advance(runtime, .1, [tuple(parked)])
        with patch('time.monotonic', return_value=time.monotonic()+.1):
            publisher.recheck_active_path()
            self.assertEqual(set(publisher._safety_provider.guard.memory.tracks), {('semantic', 32)})
            recovery = publisher._safety_provider.recovery_inputs(runtime.test_status['ego'], 'LATTICE')
            self.assertIsNotNone(recovery)
            self.assertEqual(recovery['ids'], [32])

    def test_far_vehicle_not_limiting_speed_does_not_trigger_another_bypass_search(self):
        runtime = self.attach(self.runtime())
        runtime.object_receiver.objects = [car(50., 0., 1., 0., 17)]
        publisher = runtime.path_publisher
        publisher.zone_mps, publisher.emergency_stop = 30./3.6, False
        publisher.publish(runtime.test_status['ego'], runtime.test_route, runtime.test_route,
            dict(mode='LATTICE', reason='clear current path', validated_static=True))
        self.assertFalse(self.payload(runtime)['decision']['emergency'])
        self.assertIsNone(publisher._safety_provider.recovery_inputs(runtime.test_status['ego'], 'LATTICE'))
        self.assertIn(('semantic', 17), publisher._safety_provider.guard.memory.tracks)

    def test_tunnel_guard_keeps_planner_surface_filter_and_latest_low_obstacles(self):
        runtime = self.attach(TunnelIntegrationTests().runtime())
        runtime.test_scan['xyz'] = np.tile([[7., 0., 2.4]], (120, 1))
        from unittest.mock import Mock
        runtime.tunnel.project_scan = Mock(return_value=dict(valid=True, obstacle_points=np.empty((0, 2))))
        ego = runtime.test_status['ego']
        runtime._live_tunnel_corridor = dict(ego=dict(ego), wall_stats={},
            corridor=LaneCorridor(runtime.test_route, np.full(len(runtime.test_route), 5.),
                                 np.full(len(runtime.test_route), -5.)))
        base = DrivingPlan(1, runtime.test_route, ego, ego, runtime.test_scan['stamp'], .01,
            time.monotonic(), 30./3.6, 60., False,
            dict(valid=True, pose_valid=True, steering_valid=True), 'TUNNEL')
        runtime.path_publisher.publish(ego, base.path, base.path,
            dict(mode='TUNNEL', reason='validated tunnel route', validated_static=True), decision_plan=base)
        self.assertFalse(self.payload(runtime)['decision']['emergency'])
        self.assertIn('wall', self.payload(runtime)['decision']['validation'])
        self.assertIn('lane', self.payload(runtime)['decision']['validation'])
        runtime.tunnel.project_scan.assert_called()
        self.advance(runtime, .1, [])
        runtime.test_scan['xyz'] = np.tile([[7., 0., .6]], (120, 1))
        runtime.tunnel.project_scan.return_value = dict(valid=True, obstacle_points=np.array([[7., 0.]]))
        runtime.path_publisher.recheck_active_path()
        self.assertTrue(self.payload(runtime)['decision']['emergency'])

    def test_independent_planner_stop_cannot_be_released_by_clear_collision_check(self):
        runtime = self.attach(self.runtime())
        publisher = runtime.path_publisher
        publisher.emergency_stop, publisher.zone_mps = True, 30./3.6
        publisher.publish(runtime.test_status['ego'], runtime.test_route, runtime.test_route,
            dict(mode='ROUTE BASE', reason='stop line', stop_reason='stop line'))
        self.advance(runtime, .2, [])
        publisher.recheck_active_path()
        self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)

    def test_tunnel_live_recheck_keeps_repeated_measured_barrier_in_blind_zone(self):
        from unittest.mock import Mock
        runtime = self.attach(TunnelIntegrationTests().runtime())
        runtime.test_status['ego']['speed_kmh'] = 10.
        points = np.array([[5., -.2], [5., 0.], [5., .2], [5.2, 0.]])
        runtime.tunnel.project_scan = Mock(return_value=dict(valid=True, obstacle_points=points))
        ego = runtime.test_status['ego']
        runtime._live_tunnel_corridor = dict(ego=dict(ego), wall_stats={},
            corridor=LaneCorridor(runtime.test_route, np.full(len(runtime.test_route), 5.),
                                 np.full(len(runtime.test_route), -5.)))
        base = DrivingPlan(1, runtime.test_route, ego, ego, runtime.test_scan['stamp'], .01,
            time.monotonic(), 10./3.6, 60., False,
            dict(valid=True, pose_valid=True, steering_valid=True), 'TUNNEL')
        publisher = runtime.path_publisher
        publisher.publish(ego, base.path, base.path, dict(mode='TUNNEL', reason='route'), decision_plan=base)
        self.advance(runtime, .1, [])
        publisher.recheck_active_path()
        self.advance(runtime, .1, [])
        runtime.tunnel.project_scan.return_value = dict(valid=True, obstacle_points=np.empty((0, 2)))
        publisher.recheck_active_path()
        raw = self.payload(runtime)['decision']['validation']['live_safety']['raw_clear_m']
        self.assertLess(raw, 3.)
        self.assertGreater(publisher._safety_provider.near_field.retained_n, 0)

    def test_tunnel_planning_receives_live_measured_blind_zone_barrier(self):
        runtime = self.attach(TunnelIntegrationTests().runtime())
        provider = runtime.path_publisher._safety_provider
        provider.owner = 'TUNNEL'
        ego = dict(runtime.test_status['ego'])
        barrier = np.array([[5., -.2], [5., 0.], [5., .2], [5.2, 0.]])
        provider.near_field.update(barrier, ego, 100.)
        provider.near_field.update(barrier, ego, 100.1)
        provider.near_field.update(np.empty((0, 2)), ego, 100.2)
        points = provider.tunnel_near_points(ego)
        self.assertGreater(len(points), 0)
        from tunnel_runtime import swept_clearance
        self.assertLess(swept_clearance(runtime.test_route, points, 60.), 3.)
        captured = []
        original = runtime.tunnel_lattice_plan
        def lattice(route, aligned, points, objects, corridor, now):
            captured.append(points.copy())
            return original(route, aligned, points, objects, corridor, now)
        with patch.object(runtime, 'tunnel_lattice_plan', side_effect=lattice):
            runtime.infer()
        self.assertTrue(captured)
        self.assertLess(swept_clearance(runtime.test_route, captured[0], 60.), 3.)

    def test_tunnel_retained_points_follow_odometry_and_do_not_change_memory(self):
        runtime = self.attach(TunnelIntegrationTests().runtime())
        provider = runtime.path_publisher._safety_provider
        provider.owner = 'TUNNEL'
        ego = dict(runtime.test_status['ego'])
        barrier = np.array([[5., -.2], [5., 0.], [5., .2], [5.2, 0.]])
        provider.near_field.update(barrier, ego, 100.)
        provider.near_field.update(barrier, ego, 100.1)
        count = len(provider.near_field.cells)
        fresh = dict(ego, x=ego['x']+1.)
        points = provider.tunnel_near_points(fresh)
        self.assertGreater(len(points), 0)
        self.assertAlmostEqual(points[:, 0].min(), 4.)
        self.assertEqual(len(provider.near_field.cells), count)
        self.assertEqual(provider.near_field.last_scan, 100.1)
        provider.owner = 'ROAD'
        self.assertEqual(len(provider.tunnel_near_points(fresh)), 0)

    def test_highway_policy_is_not_replaced_by_general_road_guard(self):
        runtime = self.attach(self.runtime())
        publisher = runtime.path_publisher
        publisher._safety_provider = lambda *args: self.fail('general safety replaced highway policy')
        publisher.zone_mps, publisher.emergency_stop = 25., False
        publisher.publish(runtime.test_status['ego'], runtime.test_route, runtime.test_route,
            dict(mode='HIGHWAY', reason='highway own speed plan'))
        self.assertEqual(self.payload(runtime)['target_speed_mps'], 25.)
        self.assertFalse(publisher.recheck_active_path())

    def test_tunnel_uses_raw_lidar_and_keeps_wall_corrected_pose(self):
        runtime = self.attach(TunnelIntegrationTests().runtime(correction_y=1.))
        runtime.infer()
        plan = runtime.path_publisher._active_decision_plan
        self.assertTrue(plan.validation['pose_valid'])
        self.assertTrue(plan.validation['steering_valid'])
        self.assertTrue(plan.checks_match_path())
        self.assertIn('wall', plan.validation)
        self.assertIn('lane', plan.validation)
        self.assertAlmostEqual(plan.validation_ego['y']-plan.control_ego['y'], 1.)
        runtime.receiver.snapshot.assert_not_called()
        runtime.model.assert_not_called()

    def test_live_publication_keeps_baseline_lead_evidence_with_its_own_source(self):
        runtime = self.attach(self.runtime())
        ego = runtime.test_status['ego']
        lead = dict(plan_id=7, sensor_stamp=99.8, path_digest='previous-snapshot', cap_kmh=25.)
        base = DrivingPlan(7, runtime.test_route, ego, ego, 99.9, .1, time.monotonic(),
            25./3.6, 60., False, dict(valid=True, pose_valid=True, steering_valid=True, lead=lead))
        checked = runtime.path_publisher._safety_provider(base.path, ego, 'LATTICE', base)
        self.assertEqual(checked.validation['baseline_lead'], lead)
        self.assertNotIn('lead', checked.validation)
        self.assertEqual(checked.validation['traffic_path_digest'], checked.path_digest())
        self.assertEqual(checked.validation['sensor_times']['lidar'], runtime.test_scan['stamp'])
        self.assertTrue(checked.checks_match_path())

    def test_existing_concrete_bypass_is_preserved_and_live_departure_stops_that_path(self):
        box = (18., 0., 1.1, 'fusion_obstacle', 'obstacle', 0., 1.1, 1.1, 0., 0., 0., 77)
        points = [(18., 0.), (18., .2), (18., -.2)]
        original = self.runtime()
        original.object_receiver.objects = [box]
        original.lidar.pts = np.asarray(points, dtype=np.float32)
        merged = self.attach(self.runtime())
        merged.object_receiver.objects = [box]
        merged.lidar.pts = np.asarray(points, dtype=np.float32)
        merged.test_scan['xyz'] = np.column_stack((points, np.full(len(points), .6)))
        self.assertTrue(original.infer())
        self.assertTrue(merged.infer())
        before, checked = self.payload(original), self.payload(merged)
        np.testing.assert_allclose(
            [[p['x'], p['y'], p['heading']] for p in checked['waypoints']],
            [[p['x'], p['y'], p['heading']] for p in before['waypoints']], rtol=0., atol=1e-5)
        self.assertGreater(checked['target_speed_mps'], 0.)
        self.assertFalse(checked['decision']['emergency'])
        path = np.array([[p['x'], p['y']] for p in checked['waypoints']])
        station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        location = path[np.searchsorted(station, 10.)]
        self.advance(merged, .1, [box, person(*location, 0., 0.)])
        self.assertTrue(merged.path_publisher.recheck_active_path())
        stopped = self.payload(merged)
        self.assertEqual(stopped['waypoints'], checked['waypoints'])
        self.assertEqual(stopped['target_speed_mps'], 0.)
        self.assertTrue(stopped['decision']['emergency'])

    def stopped_car(self):
        runtime = self.attach(self.runtime())
        blocker = car(12., 0., 1., 0., 17)
        runtime.object_receiver.objects = [blocker]
        publisher = runtime.path_publisher
        publisher.zone_mps, publisher.emergency_stop = 30./3.6, False
        publisher.publish(runtime.test_status['ego'], runtime.test_route, runtime.test_route,
            dict(mode='LATTICE', reason='existing route', validated_static=True))
        self.assertTrue(self.payload(runtime)['decision']['emergency'])
        runtime.test_status['ego']['speed_kmh'] = 0.
        return runtime, blocker

    def test_live_dynamic_stop_replans_with_existing_lattice_and_restarts_on_same_checked_path(self):
        runtime, blocker = self.stopped_car()
        with patch('gui.USE_LATTICE', True):
            self.advance(runtime, .1, [blocker])
            self.assertTrue(runtime.infer())
            recovery = runtime.last_result['postprocess']
            self.assertEqual(recovery['dynamic_recovery_ids'], [17])
            self.assertGreater(abs(recovery['applied_peak_m']), .5)
            self.assertTrue(runtime.path_publisher._safety_provider.guard.stopped)
            now = time.monotonic()
            self.advance(runtime, .15, [blocker])
            with patch('time.monotonic', return_value=now+.15):
                self.assertTrue(runtime.infer())
            payload = self.payload(runtime)
            self.assertFalse(payload['decision']['emergency'], payload['decision']['stop_reason'])
            self.assertGreater(payload['target_speed_mps'], 0.)
            self.assertEqual(payload['target_speed_mps'], payload['decision']['allowed_mps'])
            self.assertTrue(runtime.path_publisher._active_decision_plan.checks_match_path())
            for k in range(3):
                self.advance(runtime, .1, [blocker])
                with patch('time.monotonic', return_value=now+.25+.1*k):
                    self.assertTrue(runtime.infer())
                payload = self.payload(runtime)
                self.assertFalse(payload['decision']['emergency'], payload['decision']['stop_reason'])
                self.assertGreater(payload['target_speed_mps'], 0.)
                self.assertEqual(payload['decision']['validation']['dynamic_recovery_ids'], [17])

    def test_candidate_checks_cannot_release_actual_stop_or_ignore_crossing_motion(self):
        runtime, _ = self.stopped_car()
        crossing = person(4., 6., 0., -4.)
        self.advance(runtime, .1, [crossing])
        runtime.path_publisher.recheck_active_path()
        provider = runtime.path_publisher._safety_provider
        recovery = provider.recovery_inputs(runtime.test_status['ego'], 'LATTICE')
        self.assertIsNotNone(recovery)
        self.assertLess(recovery['clearance'](runtime.test_route, 60.), 8.)
        side = runtime.test_route.copy()
        side[:, 1] = -3.
        self.assertLess(recovery['clearance'](side, 60.), 8.)
        self.assertTrue(provider.guard.stopped)
        self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)
        with patch('time.monotonic', return_value=time.monotonic()+1.):
            self.assertEqual(recovery['clearance'](runtime.test_route, 60.), 0.)
        self.assertTrue(provider.guard.stopped)

    def test_recovery_uses_new_section_policy_instead_of_inherited_dynamic_cap(self):
        runtime, _ = self.stopped_car()
        publisher = runtime.path_publisher
        previous = publisher._safety_baseline['decision']
        publisher._safety_baseline['decision'] = replace(previous, allowed_mps=.2)
        provider = publisher._safety_provider
        side = runtime.test_route.copy()
        side[:, 1] = 8.
        for policy in (30./3.6, 1.2):
            recovery = provider.recovery_inputs(runtime.test_status['ego'], 'LATTICE',
                                                 requested_mps=policy)
            self.assertIsNotNone(recovery)
            self.assertAlmostEqual(recovery['speed_cap'](side, 60.), policy)
            self.assertGreater(recovery['clearance'](side, 60.), 40.)
        self.assertTrue(provider.guard.stopped)
        self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)
        self.assertIsNone(provider.recovery_inputs(runtime.test_status['ego'], 'LATTICE', requested_mps=0.))

    def test_roadside_unknown_person_selects_micro_bypass_before_static_overlap(self):
        runtime = self.attach(self.runtime())
        runtime.test_status['ego']['speed_kmh'] = 0.
        runtime.object_receiver.age_s = .35
        person_beside = person(8., -2.5, 0., 0.)
        now = time.monotonic()
        with patch('gui.USE_LATTICE', True):
            with patch('time.monotonic', return_value=now):
                runtime.infer()
            self.advance(runtime, .1, [person_beside])
            with patch('time.monotonic', return_value=now+.1):
                runtime.path_publisher.recheck_active_path()
            self.advance(runtime, .15, [person_beside])
            with patch('time.monotonic', return_value=now+.25):
                runtime.infer()
            payload = self.payload(runtime)
            post = runtime.last_result['postprocess']
            self.assertGreater(post['lattice']['evaluated'], 3)
            self.assertEqual(payload['decision']['validation']['dynamic_recovery_ids'], [16])
            self.assertFalse(payload['decision']['emergency'])
            self.assertGreater(payload['target_speed_mps'], 0.)
            self.assertTrue(runtime.path_publisher._active_decision_plan.checks_match_path())
            # Reusing its identity cannot waive a newly measured crossing.
            self.advance(runtime, .1, [person(3., 6., 0., -4.)])
            with patch('time.monotonic', return_value=now+.35):
                runtime.path_publisher.recheck_active_path()
            stopped = self.payload(runtime)
            self.assertEqual(stopped['waypoints'], payload['waypoints'])
            self.assertEqual(stopped['target_speed_mps'], 0.)
            self.assertTrue(stopped['decision']['emergency'])

    def test_recovery_does_not_replace_roundabout_gap_or_highway_policy(self):
        runtime, _ = self.stopped_car()
        provider = runtime.path_publisher._safety_provider
        self.assertIsNone(provider.recovery_inputs(runtime.test_status['ego'], 'HIGHWAY'))
        runtime._intersection_vehicle_status = dict(ready=True)
        self.assertIsNone(provider.recovery_inputs(runtime.test_status['ego'], 'LATTICE'))

    def test_published_general_path_keeps_pedestrian_approach_cap(self):
        runtime = self.attach(self.runtime())
        runtime.object_receiver.objects = [person(16., -2.4, 0., 0.)]
        now = time.monotonic()
        with patch('gui.USE_LATTICE', True), patch('time.monotonic', return_value=now):
            runtime.infer()
        payload = self.payload(runtime)
        cap = runtime._ped_guard_log['approach_cap_kmh']
        self.assertGreater(cap, 10.)
        self.assertLess(cap, 30.)
        self.assertGreater(payload['target_speed_mps'], 0.)
        self.assertAlmostEqual(payload['target_speed_mps'], runtime.yield_cap_mps)
        self.assertLessEqual(payload['target_speed_mps'] * 3.6, cap + .05)
        self.assertEqual(payload['target_speed_mps'], payload['decision']['allowed_mps'])
        self.assertTrue(runtime.path_publisher._active_decision_plan.checks_match_path())

    def test_live_stop_remains_stricter_than_pedestrian_approach_cap(self):
        runtime = self.attach(self.runtime())
        runtime.object_receiver.objects = [person(8., -2.4, 0., 0.)]
        with patch('gui.USE_LATTICE', True):
            runtime.infer()
        payload = self.payload(runtime)
        self.assertEqual(runtime._ped_guard_log['approach_cap_kmh'], 10.)
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertTrue(payload['decision']['emergency'])

    def test_first_unknown_person_at_standstill_does_not_latch_zero_speed(self):
        runtime = self.attach(self.runtime())
        runtime.test_status['ego']['speed_kmh'] = 0.
        runtime.object_receiver.objects = [person(6., -2.5, 0., 0.)]
        now = time.monotonic()
        with patch('gui.USE_LATTICE', True), patch('time.monotonic', return_value=now):
            runtime.infer()
        payload = self.payload(runtime)
        self.assertGreater(payload['target_speed_mps'], 0.)
        self.assertFalse(payload['decision']['emergency'])
        self.assertTrue(runtime.path_publisher._active_decision_plan.checks_match_path())

    def test_recovery_rejects_zero_pace_even_when_geometric_clearance_is_positive(self):
        runtime = self.attach(self.runtime())
        runtime.test_status['ego']['speed_kmh'] = 0.
        now = time.monotonic()
        with patch('gui.USE_LATTICE', True), patch('time.monotonic', return_value=now):
            runtime.infer()
        self.advance(runtime, .1, [person(3.3, -1.6, 0., 0.)])
        with patch('time.monotonic', return_value=now+.1):
            runtime.path_publisher.recheck_active_path()
            recovery = runtime.path_publisher._safety_provider.recovery_inputs(
                runtime.test_status['ego'], 'LATTICE')
            self.assertIsNotNone(recovery)
            self.assertEqual(recovery['speed_cap'](runtime.test_route, 40.), 0.)
            self.assertEqual(recovery['clearance'](runtime.test_route, 40.), 0.)

    def test_standing_person_after_crossing_reuses_clear_path_without_old_hold(self):
        runtime = self.attach(self.runtime())
        runtime.test_status['ego']['speed_kmh'] = 0.
        runtime.object_receiver.age_s = .25
        blocker = person(3.8, -1.8, 0., 0.)
        returns = np.array([[3.75, -1.85], [3.8, -1.8], [3.85, -1.75]])
        runtime.lidar.pts = returns
        runtime.test_scan['obstacle_points'] = returns
        now = time.monotonic()
        with patch('gui.USE_LATTICE', True):
            with patch('time.monotonic', return_value=now):
                runtime.infer()
            self.advance(runtime, .1, [blocker])
            with patch('time.monotonic', return_value=now+.1):
                runtime.path_publisher.recheck_active_path()
            self.advance(runtime, .15, [blocker])
            with patch('time.monotonic', return_value=now+.25):
                runtime.path_publisher.recheck_active_path()
                runtime._pedestrian_hold_until = now+10.
                runtime._yield_stop_until = now+10.
                runtime.last_yield = dict(kind='stop', name='pedestrian')
                runtime.infer()
            # This straight fixture is already physically clear. Reusing it
            # must not inherit the obsolete positional pedestrian stop.
            for k in range(3):
                if k:
                    self.advance(runtime, .1, [blocker])
                    with patch('time.monotonic', return_value=now+.25+k*.1):
                        runtime.path_publisher.recheck_active_path()
                        runtime._pedestrian_hold_until = now+10.
                        runtime._yield_stop_until = now+10.
                        runtime.last_yield = dict(kind='stop', name='pedestrian')
                        runtime.infer()
                payload = self.payload(runtime)
                self.assertGreater(payload['target_speed_mps'], .5)
                self.assertFalse(payload['decision']['emergency'])
                self.assertTrue(runtime.path_publisher._active_decision_plan.checks_match_path())
                self.assertGreater(payload['decision']['validation']['live_safety']['raw_clear_m'], 8.)

    def test_retained_bypass_also_plans_around_a_new_dynamic_blocker(self):
        runtime, blocker = self.stopped_car()
        publisher = runtime.path_publisher
        self.advance(runtime, .1, [blocker, person(4., 6., 0., -4.)])
        publisher.recheck_active_path()
        active = publisher._active_decision_plan
        publisher._active_decision_plan = replace(active,
            validation=dict(active.validation, dynamic_recovery_ids=[17]))
        recovery = publisher._safety_provider.recovery_inputs(runtime.test_status['ego'], 'LATTICE')
        self.assertIsNotNone(recovery)
        self.assertEqual(recovery['ids'], [16, 17])
        self.assertTrue(publisher._safety_provider.guard.stopped)

    def measured_tunnel_blocker(self, runtime, blocker):
        """A detector car whose motion a measured tunnel cluster confirms.

        In the tunnel semantic fusion supplies no motion (its bodies are left
        to the raw-point check) and detector motion needs a measured mover.
        """
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
            [blocker], True, dict(stamp=runtime.test_scan['stamp'], age_s=.02)))
        return patch('live_path_safety.measured_vehicle', side_effect=lambda car, *args, **kwargs: car)

    def test_tunnel_recovery_uses_existing_fan_with_motion_and_wall_checks(self):
        for half_width in (5., 1.5):
            with self.subTest(half_width=half_width):
                runtime = self.attach(TunnelIntegrationTests().runtime(half_width=half_width))
                ego = runtime.test_status['ego']
                blocker = car(12., 0., 1., 0., 17)
                measured = self.measured_tunnel_blocker(runtime, blocker)
                measured.start()
                self.addCleanup(measured.stop)
                base = DrivingPlan(1, runtime.test_route, ego, ego, runtime.test_scan['stamp'], .01,
                    time.monotonic(), 30./3.6, 60., False,
                    dict(valid=True, pose_valid=True, steering_valid=True), 'TUNNEL')
                runtime.path_publisher.publish(ego, base.path, base.path,
                    dict(mode='TUNNEL', reason='existing route', validated_static=True), decision_plan=base)
                self.assertTrue(self.payload(runtime)['decision']['emergency'])
                ego['speed_kmh'] = 0.
                corridor = LaneCorridor(runtime.test_route, np.full(len(runtime.test_route), 5.),
                                        np.full(len(runtime.test_route), -5.))
                plan, _ = runtime.tunnel_lattice_plan(runtime.test_route, ego, np.empty((0, 2)),
                                                      [blocker], corridor, time.monotonic())
                self.assertIsNotNone(plan)
                self.assertEqual(plan['dynamic_recovery_ids'], [17])
                if half_width == 5.:
                    self.assertFalse(plan['blocked'])
                    self.assertGreater(abs(plan['offset_m']), .5)
                    self.assertGreater(runtime.tunnel.path_clearance(plan['path'], ego, 60.), 8.)
                else:
                    self.assertTrue(plan['blocked'])
                self.assertTrue(runtime.path_publisher._safety_provider.guard.stopped)

    def test_tunnel_fallback_recovery_keeps_measured_width_and_motion_checks(self):
        for half_width in (5., 1.5):
            with self.subTest(half_width=half_width):
                runtime = self.attach(TunnelIntegrationTests().runtime(half_width=half_width))
                ego = runtime.test_status['ego']
                measured = self.measured_tunnel_blocker(runtime, car(12., 0., 1., 0., 17))
                measured.start()
                self.addCleanup(measured.stop)
                base = DrivingPlan(1, runtime.test_route, ego, ego, runtime.test_scan['stamp'], .01,
                    time.monotonic(), 30./3.6, 60., False,
                    dict(valid=True, pose_valid=True, steering_valid=True), 'TUNNEL')
                runtime.path_publisher.publish(ego, base.path, base.path,
                    dict(mode='TUNNEL', reason='existing route', validated_static=True), decision_plan=base)
                ego['speed_kmh'] = 0.
                corridor = LaneCorridor(runtime.test_route, np.full(len(runtime.test_route), half_width),
                    np.full(len(runtime.test_route), -half_width))
                plan = runtime.fallback_lattice(runtime.test_route, ego, np.empty((0, 2)), corridor=corridor)
                if half_width > 4.:
                    self.assertIsNotNone(plan)
                    self.assertEqual(plan['dynamic_recovery_ids'], [17])
                    self.assertGreater(abs(plan['target']), .5)
                    self.assertGreater(runtime.tunnel.path_clearance(plan['path'], ego, 60.), 8.)
                else:
                    self.assertIsNone(plan)
                self.assertTrue(runtime.path_publisher._safety_provider.guard.stopped)

    def test_tunnel_fallback_replaces_previous_corridor_and_reports_final_stop_plan(self):
        runtime = self.attach(TunnelIntegrationTests().runtime(valid=False))
        old = LaneCorridor(runtime.test_route, np.full(len(runtime.test_route), .8),
                          np.full(len(runtime.test_route), -.8))
        runtime._live_tunnel_corridor = dict(ego=dict(runtime.test_status['ego']), corridor=old)
        runtime.infer()
        plan = runtime.path_publisher._active_decision_plan
        self.assertIsNot(runtime._live_tunnel_corridor['corridor'], old)
        self.assertFalse(plan.emergency, plan.stop_reason)
        self.assertGreater(plan.allowed_mps, 0.)
        self.assertGreater(plan.validation['lane'], 8.)
        self.assertEqual(runtime._web_record['decision']['plan_id'], plan.plan_id)
        self.assertEqual(runtime._web_record['decision']['emergency'], plan.emergency)

    def junction(self, cars, speed=12.):
        runtime = self.attach(self.runtime())
        ego = runtime.test_status['ego']
        ego['speed_kmh'] = speed
        runtime.gap_planner.junction = dict(held=False, ready=True, unknown_ids=[])
        runtime.gap_planner.junction_committed = True
        runtime.gap_planner.state = 'ENTER'
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
            cars, True, dict(stamp=runtime.test_scan['stamp'], age_s=.1)))
        path = runtime.test_route
        plan = DrivingPlan(1, path, ego, ego, 99.9, .1, time.monotonic(),
            8., 60., False, dict(valid=True, pose_valid=True, steering_valid=True, blocked=False), 'LATTICE')
        return runtime, plan

    def test_junction_keeps_accepted_side_clearance_instead_of_untimed_stop(self):
        runtime, plan = self.junction([car(1., 5., 0., -3., 12)])
        result = runtime.path_publisher._safety_provider(plan.path, plan.control_ego, plan.mode, plan)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertIn('junction_traffic', result.validation['live_safety'])
        self.assertTrue(result.checks_match_path())
        self.assertEqual(runtime.gap_planner.state, 'ENTER')
        self.assertEqual(runtime.gap_planner.risk_details, [])

    def test_junction_without_line_wait_keeps_live_npc_collision_braking(self):
        runtime, plan = self.junction([car(5.5, 0., 0., 0., 91)])
        runtime.stop_line_controller = None
        runtime.gap_planner.junction.update(following_only=True, empty_actual_body=True)
        runtime.gap_planner.junction_committed = False
        result = runtime.path_publisher._safety_provider(plan.path, plan.control_ego, plan.mode, plan)
        self.assertTrue(result.emergency, result.stop_reason)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertFalse(runtime.gap_planner.junction_committed)
        self.assertAlmostEqual(runtime.path_publisher._safety_provider.guard.front_m, 3.845)

    def test_junction_without_line_wait_drives_on_when_empty(self):
        runtime, plan = self.junction([])
        runtime.stop_line_controller = None
        runtime.gap_planner.junction.update(following_only=True, empty_actual_body=True)
        runtime.gap_planner.junction_committed = False
        result = runtime.path_publisher._safety_provider(plan.path, plan.control_ego, plan.mode, plan)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertFalse(runtime.gap_planner.junction_committed)

    def test_no_line_gui_defers_old_traffic_and_caution_to_live_timing(self):
        # The GUI's older WAIT/GO and position-only fallback cannot override
        # the fresh empty-traffic check on the final published path.
        for planned, caution in ((.42094, 8./3.6), (.41764, 8./3.6),
                                 (2., 0.), (0., 8./3.6)):
            with self.subTest(planned=planned, caution=caution):
                runtime = self.attach(self.runtime())
                runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
                    [], True, dict(stamp=99.9, age_s=.1, vehicles=[])))
                runtime.test_status['ego']['speed_kmh'] = 2.
                runtime._intersection_vehicle_status = dict(
                    active_region=dict(line_id='roundabout', stop_station_m=20.),
                    station_m=0., detector_ready=True)
                runtime.intersection_objects = lambda objects, ego: objects
                runtime.refresh_intersection_objects = lambda objects, ego: objects
                runtime.gap_planner.junction_committed = True
                runtime.gap_planner.junction_wait_since = 1.
                runtime.gap_planner.followable_ids = {999}
                runtime.gap_planner.risk_details = [dict(object_id=999)]
                traffic = dict(kind='stop' if planned == 0. else 'follow',
                    name='lidar_car', along=.8, t=0., v_mps=planned,
                    reason='junction: brake before occupied ring corridor')
                real_evaluate = type(runtime.gap_planner).evaluate
                gui_calls = []
                def evaluated(planner, *args, **kwargs):
                    if planner is runtime.gap_planner:
                        gui_calls.append(True)
                        return dict(traffic)
                    return real_evaluate(planner, *args, **kwargs)
                fallback = dict(kind='stop' if caution == 0. else 'follow',
                    name='crossing_caution:lidar_car', along=3., t=0., v_mps=caution)
                with patch('gui.crossing_caution', return_value=fallback), \
                        patch.object(type(runtime.gap_planner), 'evaluate', evaluated):
                    runtime.infer()
                self.assertTrue(runtime.gap_planner.junction['following_only'])
                self.assertEqual(gui_calls, [])
                self.assertFalse(runtime.gap_planner.junction_committed)
                self.assertIsNone(runtime.gap_planner.junction_wait_since)
                self.assertEqual(runtime.gap_planner.followable_ids, set())
                self.assertEqual(runtime.gap_planner.risk_details, [])
                self.assertIsNone(runtime.last_yield)
                self.assertGreater(self.payload(runtime)['target_speed_mps'], 0.)
                self.assertFalse(self.payload(runtime)['decision']['emergency'])

    def test_no_line_gui_keeps_traffic_check_without_a_live_publisher(self):
        runtime = self.attach(self.runtime())
        runtime.path_publisher._safety_provider = None
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
            [], True, dict(stamp=99.9, age_s=.1, vehicles=[])))
        runtime._intersection_vehicle_status = dict(
            active_region=dict(line_id='roundabout', stop_station_m=20.),
            station_m=0., detector_ready=True)
        runtime.intersection_objects = lambda objects, ego: objects
        runtime.refresh_intersection_objects = lambda objects, ego: objects
        original = type(runtime.gap_planner).evaluate
        calls = []
        def evaluated(planner, *args, **kwargs):
            calls.append(planner)
            return original(planner, *args, **kwargs)
        with patch.object(type(runtime.gap_planner), 'evaluate', evaluated):
            runtime.infer()
        self.assertIn(runtime.gap_planner, calls)

    def test_current_measured_standstill_survives_the_live_motion_history(self):
        provider = LivePathSafety(SimpleNamespace())
        provider.traffic = SimpleNamespace(junction=dict(following_only=True),
                                           speed_ratio={}, map_predictor=None)
        for index in range(6):
            stamp = 99.+.2*index
            row = tuple(car(15.+min(index, 4)*.6, 0.,
                            3. if index < 5 else 0., 0., 7))+(0.,)
            rows = provider.junction_vehicles([row], stamp, .1, stamp+.1,
                                             dict(x=0., y=0.))
        # A two-second position fit still suggests motion, but this current
        # valid zero measurement must reach both the map gate and proof batch.
        self.assertEqual(provider.junction_measured_rows[0][5], 0.)
        self.assertEqual(rows[0][5], 0.)
        self.assertEqual(rows[0][9:11], (0., 0.))
        np.testing.assert_allclose(rows[0][:2], row[:2])

    def test_no_line_entry_uses_one_follow_law_and_retains_acc_emergency(self):
        for state, traffic_cap, acc_emergency in (
                ('FOLLOW', 4., False), ('CROSSING_CAUTION', .42, False),
                ('FOLLOW', 4., True)):
            with self.subTest(state=state, emergency=acc_emergency):
                runtime = self.attach(self.runtime())
                runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
                    [], True, dict(stamp=99.9, age_s=.1, vehicles=[])))
                runtime._intersection_vehicle_status = dict(
                    active_region=dict(line_id='roundabout', stop_station_m=20.),
                    station_m=0., detector_ready=True)
                runtime.intersection_objects = lambda objects, ego: objects
                runtime.refresh_intersection_objects = lambda objects, ego: objects
                def traffic(*args, **kwargs):
                    runtime.gap_planner.status = dict(state=state)
                    return dict(kind='follow', name='lidar_car', along=8., t=0.,
                                v_mps=traffic_cap, reason='junction: follow the car ahead')
                runtime.gap_planner.evaluate = traffic
                runtime._lead_state = None
                runtime.lead_follow_step = lambda *args, **kwargs: dict(
                    cap_mps=1.28, emergency=acc_emergency, state=dict(gap_m=2.48))
                with patch('gui.crossing_caution', return_value=None):
                    runtime.infer()
                if acc_emergency:
                    self.assertAlmostEqual(runtime.yield_cap_mps, 1.28)
                else:
                    self.assertIsNone(runtime.yield_cap_mps)
                if acc_emergency:
                    self.assertEqual(self.payload(runtime)['target_speed_mps'], 0.)
                    self.assertTrue(self.payload(runtime)['decision']['emergency'])
                else:
                    self.assertGreaterEqual(self.payload(runtime)['decision']['validation']
                        ['junction_body_cap_mps'], traffic_cap)

    def test_live_timing_uses_the_independent_profile_without_raising_current_cap(self):
        from junction_vehicle_motion import certify_moving_cars
        for profile in (24./3.6, .5):
            with self.subTest(profile=profile):
                runtime, plan, world = self.native32_mapped_entry()
                plan = replace(plan, allowed_mps=1.28,
                    validation=dict(plan.validation, junction_body_cap_mps=profile))
                with patch('junction_vehicle_motion.certify_moving_cars',
                           wraps=certify_moving_cars) as checked:
                    result = runtime.path_publisher._safety_provider(
                        world, plan.control_ego, plan.mode, plan)
                self.assertEqual(checked.call_args.kwargs['profile_ceiling_mps'], profile)
                self.assertFalse(result.emergency, result.stop_reason)
                self.assertGreater(result.allowed_mps, 0.)
                self.assertLessEqual(result.allowed_mps, min(1.28, profile))

    @patch('time.monotonic', return_value=1000.)
    def test_checked_map_entry_uses_the_measured_launch_in_both_publish_paths(self, clock):
        runtime, plan, world = self.native32_mapped_entry()
        runtime.test_status['ego'].update(brake=1., accel=0.)
        publisher = runtime.path_publisher
        provider = publisher._safety_provider
        delivered = []
        note = provider.note_published
        def recorded_note(decision, path, target, now):
            self.assertAlmostEqual(self.payload(runtime)['target_speed_mps'], target)
            delivered.append(target)
            return note(decision, path, target, now)
        provider.note_published = recorded_note
        publisher.zone_mps = 24./3.6
        publisher._last_target_mps = 0.
        publisher._last_target_time = time.monotonic()-.01
        publisher.publish(plan.control_ego, plan.path, plan.path,
                          dict(mode='ROUTE BASE', reason='checked map entry'), decision_plan=plan)
        first = self.payload(runtime)
        self.assertIn('junction_motion', first['decision']['validation'])
        self.assertGreater(first['target_speed_mps'], 1.)
        self.assertIsNone(publisher._safety_baseline['launch_accel_mps2'])
        self.assertTrue(publisher._safety_baseline['junction_motion_launch'])
        publisher._last_target_mps = .1
        publisher._last_target_time = time.monotonic()-.01
        self.assertTrue(publisher.recheck_active_path())
        self.assertGreater(self.payload(runtime)['target_speed_mps'], 1.)
        self.assertTrue(delivered)
        self.assertTrue(provider.junction_launch.active)
        # The following sensor observation loses that permission. It must not
        # inherit the previous entry's target-ramp exemption.
        current = publisher._active_decision_plan
        no_permission = replace(current, allowed_mps=2.,
            validation={k: v for k, v in current.validation.items() if k != 'junction_motion'})
        publisher._safety_provider = lambda *args: no_permission
        publisher._last_target_mps = .1
        publisher._last_target_time = time.monotonic()-.01
        self.assertTrue(publisher.recheck_active_path())
        self.assertLess(self.payload(runtime)['target_speed_mps'], .5)

    def test_launch_model_requires_delivery_and_is_revoked_by_an_actual_stop(self):
        from test_junction_live_profile import JunctionLiveProfileTests
        runtime, plan, world = JunctionLiveProfileTests().fixture()
        runtime.test_status['ego'].update(brake=1., accel=0.)
        provider = runtime.path_publisher._safety_provider
        with patch('time.monotonic', return_value=100.):
            first = provider(world, plan.control_ego, plan.mode, plan)
            self.assertNotIn('junction_launch', first.validation)
            self.assertIn('junction_motion', first.validation)
            provider.note_published(first, world, first.allowed_mps, 100.)
        runtime.test_status['ego'].update(x=.03, speed_kmh=1.08,
                                         stamp=100.1, brake=0., accel=.3)
        runtime.test_scan.update(stamp=100., age_s=.1)
        with patch('time.monotonic', return_value=100.1):
            moving = provider(world, first.control_ego, first.mode, first)
            self.assertIn('junction_launch', moving.validation)
            self.assertLess(moving.validation['junction_launch']['dead_s'], .5)
            self.assertGreater(moving.allowed_mps, 0., moving.stop_reason)
            self.assertIn('junction_motion', moving.validation)
            provider.note_published(moving, world, 0., 100.1)
            stopped = provider(world, moving.control_ego, moving.mode, moving)
            self.assertNotIn('junction_launch', stopped.validation)
            self.assertNotIn('_junction_launch', stopped.control_ego)

    def test_old_motion_permission_cannot_transfer_to_a_new_empty_publication(self):
        runtime, plan = self.junction([])
        runtime.stop_line_controller = None
        runtime.gap_planner.junction.update(following_only=True, empty_actual_body=True)
        runtime.gap_planner.junction_committed = False
        plan = plan.decided(junction_motion=dict(expires_at=time.monotonic()-1.))
        result = runtime.path_publisher._safety_provider(plan.path, plan.control_ego, plan.mode, plan)
        self.assertNotIn('junction_motion', result.validation)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertTrue(result.fresh())

    def test_junction_without_line_wait_predicts_car_currently_beside_path(self):
        runtime, plan = self.junction([car(8., 8., 0., -4., 92)])
        runtime.stop_line_controller = None
        runtime.gap_planner.junction.update(following_only=True, empty_actual_body=True)
        runtime.gap_planner.junction_committed = False
        result = runtime.path_publisher._safety_provider(plan.path, plan.control_ego, plan.mode, plan)
        self.assertTrue(result.emergency, result.stop_reason)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertFalse(runtime.gap_planner.junction_committed)

    def native32_mapped_entry(self, admitted=True):
        data = json.loads((Path(__file__).parent/'test_data/roundabout_native32_map_go.json').read_text())
        # Isolate the measured body from the full-scene admission decision.
        # The earlier partial-on-path report is deliberately not certified.
        npc = data['admitted_moving_sample'] if admitted else data['npc']
        row = list(car(npc['x'], npc['y'], npc['vx'], npc['vy'], npc['uid'],
                       heading=npc['heading_deg']))
        row[6:8] = [npc['length_m'], npc['width_m']]
        runtime, _ = self.junction([tuple(row)], speed=0.)
        ego = dict(runtime.test_status['ego'], **data['ego'])
        runtime.test_status['ego'] = ego
        runtime.lanes = HdMapLanes(Path(__file__).parent/'link_set.json')
        from traffic_planner import GapPlanner
        runtime.gap_planner = GapPlanner(runtime.lanes.links)
        runtime.gap_planner.junction = dict(following_only=True, ready=True,
                                          empty_actual_body=True)
        runtime.stop_line_controller = None
        runtime.sim_clock = SimpleNamespace(rtf=data['rtf'])
        world = np.asarray(data['world_path'])
        yaw = np.radians(ego['heading_deg'])
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        local = (world-[ego['x'], ego['y']])@rotation
        plan = DrivingPlan(1, local, ego, ego, 99.9, .1, time.monotonic(),
            24./3.6, 60., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False), 'LATTICE')
        return runtime, plan, world

    def test_recorded_map_go_publishes_positive_speed_without_frozen_vehicle_tube(self):
        runtime, plan, world = self.native32_mapped_entry()
        result = runtime.path_publisher._safety_provider(world, plan.control_ego, plan.mode, plan)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertLessEqual(result.allowed_mps, 24./3.6)
        proof = result.validation['junction_motion']
        self.assertEqual(proof['covered_keys'], [('lidar_car', 500007)])
        self.assertGreater(proof['current_clear_m'], .8)
        self.assertEqual(result.validation['live_safety']['junction_traffic']['state'], 'MAP_TIMED_ENTRY')
        self.assertTrue(result.checks_match_path())
        self.assertFalse(runtime.gap_planner.junction_committed)

    def test_early_partial_on_path_car_is_not_certified_as_checked_ahead(self):
        runtime, plan, world = self.native32_mapped_entry(admitted=False)
        result = runtime.path_publisher._safety_provider(world, plan.control_ego, plan.mode, plan)
        self.assertNotIn('junction_motion', result.validation)
        self.assertEqual(result.allowed_mps, 0.)

    def test_certified_entry_retains_raw_contact_person_and_unmatched_vehicle(self):
        for hazard in ('raw', 'person', 'unmatched'):
            with self.subTest(hazard=hazard):
                runtime, plan, world = self.native32_mapped_entry()
                if hazard == 'raw':
                    runtime.test_scan['xyz'] = np.array([[2.5, 0., .6]])
                else:
                    blocker = list(person(0., 0., 0., 0.) if hazard == 'person'
                                   else car(0., 0., 0., 0., 900))
                    yaw = np.radians(plan.control_ego['heading_deg'])
                    xy = np.array([plan.control_ego['x'], plan.control_ego['y']])+2.5*np.array([np.cos(yaw), np.sin(yaw)])
                    blocker[:2] = xy
                    runtime.object_receiver.objects = [tuple(blocker)]
                result = runtime.path_publisher._safety_provider(world, plan.control_ego, plan.mode, plan)
                self.assertEqual(result.allowed_mps, 0., result.stop_reason)

    def test_stale_source_cannot_certify_recorded_map_go_vehicle(self):
        runtime, plan, world = self.native32_mapped_entry()
        vehicles, _, _ = runtime.vehicle_receiver.snapshot()
        runtime.vehicle_receiver.snapshot = lambda: (vehicles, True, dict(stamp=99.5, age_s=.5))
        result = runtime.path_publisher._safety_provider(world, plan.control_ego, plan.mode, plan)
        self.assertNotIn('junction_motion', result.validation)
        self.assertEqual(result.allowed_mps, 0., result.stop_reason)

    def test_snapshot_collection_cannot_extend_vehicle_or_lidar_freshness(self):
        runtime, plan, world = self.native32_mapped_entry()
        clock = [100.]
        runtime.test_scan['age_s'] = .34
        vehicles, _, _ = runtime.vehicle_receiver.snapshot()
        runtime.vehicle_receiver.snapshot = lambda: (vehicles, True, dict(stamp=99.9, age_s=.34))
        def delayed_semantics():
            clock[0] += .02
            return []
        runtime.object_receiver.snapshot = delayed_semantics
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            result = runtime.path_publisher._safety_provider(world, plan.control_ego, plan.mode, plan)
        self.assertNotIn('junction_motion', result.validation)
        self.assertEqual(result.allowed_mps, 0., result.stop_reason)

    def test_motion_proof_expiring_during_other_checks_stops_publication(self):
        runtime, plan, world = self.native32_mapped_entry()
        clock = [100.]
        from path_safety import PathSafety
        original_check = PathSafety.check
        def delayed_guard(guard, *args, **kwargs):
            result = original_check(guard, *args, **kwargs)
            clock[0] += .21
            return result
        with patch('time.monotonic', side_effect=lambda: clock[0]), patch.object(PathSafety, 'check', delayed_guard):
            result = runtime.path_publisher._safety_provider(world, plan.control_ego, plan.mode, plan)
        self.assertIn('junction_motion', result.validation)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)
        self.assertEqual(result.stop_reason, 'junction motion proof expired before publication')

    def test_vehicle_outside_map_entry_window_keeps_generic_collision_braking(self):
        runtime, _ = self.junction([car(37., 7.6, 0., -3., 901)], speed=60.)
        from traffic_planner import GapPlanner
        crossing = np.column_stack((np.full(161, 37.), np.arange(40., -40.01, -.5)))
        runtime.gap_planner = GapPlanner([dict(points=crossing.tolist(),
                                               from_node_idx='a', to_node_idx='b')])
        runtime.gap_planner.junction = dict(following_only=True, ready=True, empty_actual_body=True)
        runtime.stop_line_controller = None
        runtime.sim_clock = SimpleNamespace(rtf=.8)
        path = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))
        ego = runtime.test_status['ego']
        plan = DrivingPlan(1, path, ego, ego, 99.9, .1, time.monotonic(),
            30./3.6, 60., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False), 'LATTICE')
        result = runtime.path_publisher._safety_provider(path, ego, plan.mode, plan)
        self.assertNotIn('junction_motion', result.validation)
        self.assertTrue(result.emergency, result.stop_reason)
        self.assertEqual(result.allowed_mps, 0.)

    def test_junction_deadline_is_checked_after_command_serialization(self):
        for timer, proof_key in ((False, 'junction_motion'), (True, 'junction_motion'),
                                 (False, 'junction_live'), (True, 'junction_live')):
            with self.subTest(timer=timer, proof_key=proof_key):
                runtime = self.attach(self.runtime())
                runtime.infer()
                publisher = runtime.path_publisher
                provider = publisher._safety_provider
                clock = [time.monotonic()+.1]
                def expiring_provider(*args, **kwargs):
                    result = provider(*args, **kwargs)
                    return result.decided(**{proof_key: dict(expires_at=clock[0]+.01)})
                publisher._safety_provider = expiring_provider
                original_summary = DrivingPlan.summary
                summary_calls = [0]
                def delayed_summary(plan):
                    result = original_summary(plan)
                    if plan.allowed_mps > 0. and proof_key in plan.validation:
                        summary_calls[0] += 1
                        if timer or summary_calls[0] == 2:
                            clock[0] += .02
                    return result
                with patch('time.monotonic', side_effect=lambda: clock[0]), patch.object(DrivingPlan, 'summary', delayed_summary):
                    if timer:
                        publisher._last_target_mps = 0.
                        publisher.recheck_active_path()
                    else:
                        runtime.infer()
                payload = self.payload(runtime)
                self.assertEqual(payload['target_speed_mps'], 0.)
                self.assertEqual(payload['decision']['allowed_mps'], 0.)
                self.assertTrue(payload['decision']['emergency'])
                self.assertEqual(payload['decision']['stop_reason'],
                                 'junction motion proof expired before command publication')
                self.assertEqual(publisher._last_target_mps, 0.)

    def test_junction_latest_crossing_ahead_only_slows_the_committed_entry(self):
        runtime, plan = self.junction([car(8., 4., 0., -4., 13)])
        result = runtime.path_publisher._safety_provider(plan.path, plan.control_ego, plan.mode, plan)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertLess(result.allowed_mps, 8.)
        traffic = result.validation['live_safety']['junction_traffic']
        self.assertEqual(traffic['state'], 'ENTER')
        self.assertEqual(traffic['leader']['object_id'], 13)

    def test_junction_unassociated_lidar_inside_body_retains_emergency_after_commit(self):
        runtime, plan = self.junction([])
        runtime.test_scan['xyz'] = np.array([[2.5, 0., .6]])
        result = runtime.path_publisher._safety_provider(plan.path, plan.control_ego, plan.mode, plan)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency, result.stop_reason)
        self.assertTrue(result.validation['junction_current_contact'])
        self.assertEqual(result.stop_reason, 'live junction current body contact')

    def test_committed_future_pedestrian_collision_keeps_kind_based_emergency(self):
        from junction_vehicle_motion import _separation
        runtime, plan = self.junction([], speed=12.)
        runtime.gap_planner.junction.update(empty_actual_body=True, yield_only=True)
        pedestrian = person(6., 3., 0., -2.)
        runtime.object_receiver.objects = [pedestrian]
        result = runtime.path_publisher._safety_provider(
            plan.path, plan.control_ego, plan.mode, plan)
        # The measured and age-propagated bodies are both clear now. This stop
        # belongs to the pedestrian's future crossing, rather than the final
        # current-contact override or a person word in the detector's name.
        self.assertGreater(_separation(result.control_ego, pedestrian, pedestrian[:2]), .1)
        self.assertGreater(result.validation['junction_current_semantic_clear_m'], .1)
        self.assertFalse(result.validation.get('junction_current_contact', False))
        safety = result.validation['live_safety']
        self.assertTrue(safety['people_emergency'])
        crossing = next(row for row in safety['objects'] if row['uid'] == pedestrian[11])
        self.assertEqual(crossing['name'], 'p')
        self.assertGreater(crossing['cap_mps'], 0.)
        self.assertTrue(runtime.gap_planner.junction_committed)
        self.assertEqual(safety['junction_traffic']['state'], 'ENTER')
        self.assertEqual(result.stop_reason, 'live path collision: p')
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)

    def test_junction_unassociated_lidar_ahead_keeps_its_braking_cap(self):
        runtime, plan = self.junction([])
        runtime.test_scan['xyz'] = np.array([[5., 0., .6]])
        result = runtime.path_publisher._safety_provider(plan.path, plan.control_ego, plan.mode, plan)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertLess(result.allowed_mps, plan.allowed_mps)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertIn('brake behind', result.stop_reason or '')

    def test_recorded_roundabout_island_filter_applies_to_live_stop_checker(self):
        data = json.loads((Path(__file__).parent/'test_data/junction_live_safety.json').read_text())
        false_car = next(c for c in data['cars'] if c[11] == data['false_car_uid'])
        runtime, _ = self.junction([false_car], speed=0.)
        ego = dict(runtime.test_status['ego'], **data['ego'])
        runtime.test_status['ego'] = ego
        runtime.lanes = HdMapLanes(Path(__file__).parent/'link_set.json')
        self.assertEqual(runtime.on_road_vehicles([false_car]), [])
        path = np.asarray(data['path'])
        heading = np.radians(ego['heading_deg'])
        rotation = np.array([[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]])
        world = path@rotation.T+[ego['x'], ego['y']]
        plan = DrivingPlan(1, path, ego, ego, 99.9, .1, time.monotonic(),
            data['cap_mps'], 60., False, dict(valid=True, pose_valid=True, steering_valid=True, blocked=False), 'LATTICE')
        result = runtime.path_publisher._safety_provider(world, ego, plan.mode, plan)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertFalse(any(x['uid']==data['false_car_uid']
                             for x in result.validation['live_safety']['motion_states']))

    # Detector records of scenario car 40100008 (UNIQUEID 31) while the ego passed
    # it, follow-20261009T204445Z 294.0-295.6 s: (stamp, x, y, vx, vy,
    # velocity_valid, heading, length, width) in the ESKF world, and the ego (x, y,
    # heading, km/h) of the same log frames. From 294.84 s the detector velocity was
    # invalid; the live check fitted the drifting box centre (1.2 m/s towards the
    # path) and capped 35 -> 31 km/h: "live path stopping envelope: lidar_car".
    PARKED_CAR_DETECTIONS = (
        (1791578978.7904, -38.102, -522.023, 0., 0., False, 202.5, 4.97, 2.10, -6.780, -510.799, -148.08, 33.3),
        (1791578978.9148, -38.143, -522.084, 0., 0., False, 25.7, 4.97, 2.09, -8.069, -511.585, -147.92, 33.5),
        (1791578979.1789, -37.456, -521.645, .37, 2.89, True, 211.6, 4.97, 2.10, -9.013, -512.163, -147.86, 33.9),
        (1791578979.3176, -37.741, -521.704, .53, 1.58, True, 209.5, 4.97, 2.08, -10.538, -513.104, -147.88, 34.6),
        (1791578979.4512, -37.743, -521.803, .46, .96, True, 211.1, 5.02, 2.10, -10.949, -513.353, -148.03, 34.8),
        (1791578979.5763, -37.764, -521.860, 0., 0., True, 209.7, 5.00, 2.09, -11.854, -513.916, -149.02, 35.1),
        (1791578979.6908, -37.773, -522.114, 0., 0., True, 207.8, 4.99, 2.08, -13.576, -514.895, -152.18, 35.1),
        (1791578979.8042, -37.388, -522.827, 0., 0., False, 208.7, 5.02, 2.10, -13.976, -515.106, -152.67, 34.6),
        (1791578979.9284, -37.513, -522.845, 0., 0., False, 209.0, 4.97, 2.08, -15.047, -515.634, -154.54, 33.5),
        (1791578980.0767, -37.428, -521.984, 0., 0., False, 211.5, 4.95, 2.06, -16.063, -516.086, -155.54, 33.0),
        (1791578980.2271, -37.600, -522.763, -.50, .15, True, 210.7, 4.94, 2.07, -17.224, -516.569, -157.05, 31.9),
        (1791578980.4554, -37.763, -522.541, -.49, .42, True, 212.1, 4.97, 2.08, -18.511, -517.088, -156.67, 31.4),
    )

    def tunnel_vehicle_replay(self, steps):
        """Run the publish-time tunnel check over (stamp, ego, car, car outline) steps."""
        from unittest.mock import Mock
        from test_tunnel_tracks_recorded import surface_points
        # Recorded routes can be rotated relative to the initial ego frame.
        # Keep fixture walls outside those routes; the corridor follows them.
        runtime = self.attach(TunnelIntegrationTests().runtime(half_width=20.))
        provider = runtime.path_publisher._safety_provider
        runtime._live_tunnel_corridor = dict(ego=dict(runtime.test_status['ego']), wall_stats={},
            corridor=LaneCorridor(steps[0][4], np.full(len(steps[0][4]), 9.),
                                  np.full(len(steps[0][4]), -9.)))
        runtime.tunnel.project_scan = Mock()
        runtime.vehicle_receiver = Mock()
        base, results = None, []
        for stamp, ego, car, outline, route in steps:
            ego = dict(runtime.test_status['ego'], stamp=stamp, age_s=.01, **ego)
            runtime.test_status['ego'], runtime.test_odometry = ego, dict(ego)
            runtime.test_scan = dict(runtime.test_scan, stamp=stamp, sequence=runtime.test_scan['sequence']+1)
            runtime.object_receiver.observation_stamp = stamp
            yaw = np.radians(ego['heading_deg'])
            axes = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            sensor = np.array([ego['x'], ego['y']])+1.05*axes[:, 0]
            runtime.tunnel.project_scan.return_value = dict(valid=True,
                obstacle_points=(surface_points(outline, sensor)-[ego['x'], ego['y']]) @ axes)
            runtime.vehicle_receiver.snapshot.return_value = ([car], True, dict(stamp=stamp, age_s=.02))
            if base is None:
                local = (route-[ego['x'], ego['y']]) @ axes
                base = DrivingPlan(1, local, ego, ego, stamp, .01, time.monotonic(), 35./3.6, 60.,
                                   False, dict(valid=True, pose_valid=True, steering_valid=True), 'TUNNEL')
                clock = 1000.-stamp
            with patch('live_path_safety.time.monotonic', return_value=clock+stamp):
                results.append(provider(route, ego, 'TUNNEL', base))
        return results

    def parked_car_steps(self):
        from test_tunnel_tracks_recorded import box_outline
        rows = self.PARKED_CAR_DETECTIONS
        x0, y0, h0 = rows[0][9], rows[0][10], np.radians(rows[0][11])
        frame = np.array([[np.cos(h0), np.sin(h0)], [-np.sin(h0), np.cos(h0)]])
        local = lambda x, y: frame @ (np.array([x, y])-[x0, y0])
        heading = float(np.mean([r[6] for r in rows if 200. < r[6] < 220.]))-rows[0][11]
        centre = np.mean([local(r[1], r[2]) for r in rows], axis=0)
        outline = box_outline(centre[0], centre[1], 4.97, 2.09, heading)
        along = np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading))])
        normal = np.array([-along[1], along[0]])
        # The bypass passed the car's centre 3.3-3.8 m to its left.
        route = np.array([(float(centre @ normal)+3.6)*normal+s*along for s in np.arange(0., 80., .5)])
        steps = []
        for stamp, x, y, vx, vy, valid, hd, length, width, ex, ey, eh, kmh in rows:
            ego = dict(zip(('x', 'y'), local(ex, ey).tolist()), heading_deg=eh-rows[0][11], speed_kmh=kmh)
            position, velocity = local(x, y), frame @ np.array([vx, vy])
            car = (float(position[0]), float(position[1]), max(length, width), 'lidar_car', 'npc',
                   float(np.hypot(*velocity)), length, width, hd-rows[0][11], float(velocity[0]),
                   float(velocity[1]), 500032, 0., None, valid, 0.)
            steps.append((stamp, ego, car, outline, route))
        return steps

    def test_tunnel_parked_car_drifting_detector_box_no_longer_slows_the_pass(self):
        lidar_car = ('live path stopping envelope: lidar_car', 'live path collision: lidar_car')
        with patch('live_path_safety.measured_vehicle', side_effect=lambda car, *args, **kwargs: car):
            before = self.tunnel_vehicle_replay(self.parked_car_steps())
        self.assertTrue(any(r.stop_reason in lidar_car for r in before))
        self.assertLess(min(r.allowed_mps for r in before)*3.6, 32.)
        for result in self.tunnel_vehicle_replay(self.parked_car_steps()):
            self.assertFalse(result.emergency, result.stop_reason)
            self.assertNotIn(result.stop_reason, lidar_car)
            self.assertAlmostEqual(result.allowed_mps*3.6, 35., places=3)

    def test_tunnel_parked_car_with_unknown_detector_velocity_is_not_fitted_into_motion(self):
        # follow-20261010T060515Z 281.8 s / 560.0 s: the check fitted motion from the
        # drifting box of parked cars passed in the tunnel (no measured mover there).
        lidar_car = ('live path stopping envelope: lidar_car', 'live path collision: lidar_car')
        steps = [(stamp, ego, car[:5]+(0.,)+car[6:9]+(0., 0.)+car[11:14]+(False,)+car[15:], outline, route)
                 for stamp, ego, car, outline, route in self.parked_car_steps()]
        with patch('live_path_safety.measured_vehicle', side_effect=lambda car, *args, **kwargs: car):
            before = self.tunnel_vehicle_replay(steps)
        self.assertTrue(any(r.stop_reason in lidar_car for r in before))
        for result in self.tunnel_vehicle_replay(steps):
            self.assertFalse(result.emergency, result.stop_reason)
            self.assertNotIn(result.stop_reason, lidar_car)

    # follow-20261010T062150Z: detector records of the scenario's parked car 40100008
    # (UNIQUEID 32) as above, with the ego of the same log frames. 281.33-281.58 s:
    # the box flipped to the car's other end and read 5.9 m/s for one frame
    # ("collision: lidar_car" at 6 km/h). 285.46-285.93 s: a standing box skewed
    # 13 deg over the bypass margin ("collision: lidar_car" at 30 km/h, raw clear 54 m).
    FLIPPED_BOX_DETECTIONS = (
        (1791613591.6659, -61.604, -541.724, 0., 0., False, 213.5, 5.03, 2.10, -46.246, -532.330, -153.96, 1.5),
        (1791613591.7756, -62.836, -542.969, 0., 0., False, 31.4, 4.93, 2.06, -46.251, -532.333, -153.97, 2.9),
        (1791613591.9188, -61.618, -541.796, 4.41, 3.88, True, 213.1, 4.94, 2.06, -46.125, -532.264, -153.96, 5.6),
        (1791613592.0352, -63.090, -542.541, 0., 0., True, 32.3, 4.94, 2.07, -46.050, -532.223, -153.97, 5.3),
    )
    STANDING_BOX_DETECTIONS = (
        (1791613595.6307, -63.095, -542.647, -.30, -.09, True, 216.4, 4.94, 2.09, -55.551, -536.476, -157.35, 26.7),
        (1791613595.7641, -63.334, -542.936, -.29, -.18, True, 217.0, 4.91, 2.08, -56.393, -536.826, -157.55, 28.5),
        (1791613595.9017, -63.031, -542.675, 0., 0., True, 215.5, 4.95, 2.07, -56.837, -537.007, -157.53, 29.0),
        (1791613596.0047, -62.329, -541.943, 0., 0., True, 216.1, 4.95, 2.07, -57.835, -537.421, -157.58, 29.9),
        (1791613596.1275, -62.764, -542.398, .52, .33, True, 215.7, 4.97, 2.08, -58.782, -537.809, -157.76, 30.6),
        (1791613596.2528, -62.598, -542.185, .75, .70, True, 215.5, 4.95, 2.07, -59.267, -538.005, -157.89, 30.0),
    )
    # Measured body of that car (planner clusters 285.46-285.93 s) and the bypass
    # line that passed it 2.6 m to its right.
    PARKED_CAR_32 = (-63.0, -541.9, 4.9, 2.1, 33.)

    def recorded_steps(self, rows, lateral_m):
        """Recorded detector frames with a signed offset to the ego's left."""
        from test_tunnel_tracks_recorded import box_outline
        x0, y0, h0 = rows[0][9], rows[0][10], np.radians(rows[0][11])
        frame = np.array([[np.cos(h0), np.sin(h0)], [-np.sin(h0), np.cos(h0)]])
        local = lambda x, y: frame @ (np.array([x, y])-[x0, y0])
        cx, cy, length, width, heading = self.PARKED_CAR_32
        centre = local(cx, cy)
        outline = box_outline(centre[0], centre[1], length, width, heading-rows[0][11])
        # The recorded bypass follows the measured car's axis. A horizontal
        # line in the initial ego frame cuts through its skewed front corner.
        yaw = np.radians((heading-rows[0][11]+90.) % 180.-90.)
        along = np.array([np.cos(yaw), np.sin(yaw)])
        normal = np.array([-along[1], along[0]])
        route = np.array([(float(centre @ normal)+lateral_m)*normal+s*along
                          for s in np.arange(0., 80., .5)])
        steps = []
        for stamp, x, y, vx, vy, valid, hd, length, width, ex, ey, eh, kmh in rows:
            ego = dict(zip(('x', 'y'), local(ex, ey).tolist()), heading_deg=eh-rows[0][11], speed_kmh=kmh)
            position, velocity = local(x, y), frame @ np.array([vx, vy])
            car = (float(position[0]), float(position[1]), max(length, width), 'lidar_car', 'npc',
                   float(np.hypot(*velocity)), length, width, hd-rows[0][11], float(velocity[0]),
                   float(velocity[1]), 500030, 0., None, valid, 0.)
            steps.append((stamp, ego, car, outline, route))
        return steps

    def test_tunnel_flipped_detector_box_does_not_stop_the_crawl(self):
        from tunnel_tracks import DetectorMotion
        lidar_car = ('live path stopping envelope: lidar_car', 'live path collision: lidar_car')
        steps = self.recorded_steps(self.FLIPPED_BOX_DETECTIONS, -2.6)
        with patch.object(DetectorMotion, 'persisted', return_value=None):
            before = self.tunnel_vehicle_replay(steps)
        self.assertTrue(any(r.stop_reason in lidar_car and r.emergency for r in before))
        for result in self.tunnel_vehicle_replay(steps):
            self.assertFalse(result.emergency, result.stop_reason)

    def test_tunnel_standing_box_skewed_over_the_margin_does_not_stop_the_pass(self):
        lidar_car = ('live path stopping envelope: lidar_car', 'live path collision: lidar_car')
        steps = self.recorded_steps(self.STANDING_BOX_DETECTIONS, 2.6)
        with patch('live_path_safety.static_body', side_effect=lambda obj, *args, **kwargs: obj):
            before = self.tunnel_vehicle_replay(steps)
        self.assertTrue(any(r.stop_reason in lidar_car and r.emergency for r in before))
        for result in self.tunnel_vehicle_replay(steps):
            self.assertFalse(result.emergency, result.stop_reason)
            self.assertGreater(result.validation['live_safety']['raw_clear_m'], 40.)
            self.assertAlmostEqual(result.allowed_mps*3.6, 35., places=3)

    def test_tunnel_standing_detector_body_still_stops_an_overlapping_path(self):
        steps = self.recorded_steps(self.STANDING_BOX_DETECTIONS, 0.)
        for result in self.tunnel_vehicle_replay(steps):
            self.assertTrue(result.emergency, result.stop_reason)
            self.assertEqual(result.allowed_mps, 0.)
            self.assertLess(result.validation['live_safety']['raw_clear_m'], 10.)
            self.assertIn('lidar', result.stop_reason)

    def test_tunnel_fused_car_motion_does_not_slow_the_crawl(self):
        # 281.69-281.80 s: fusion_npc uid 129 "moving", gap 9 m, cap 6 km/h, beside the
        # parked car above. Its own box was not logged: placed on the car's measured
        # body, reported at 1.5 m/s towards the path (above the fused 1 m/s floor).
        from live_path_safety import LivePathSafety
        # Leave the measured body and the raw sweep's side margin clear so
        # the semantic velocity is the only source of the false slowdown.
        steps = self.recorded_steps(self.FLIPPED_BOX_DETECTIONS[-1:]*4, -2.6)
        fused = []
        for i, (stamp, ego, car, outline, route) in enumerate(steps):
            row = list(car)
            row[3], row[4], row[11], row[14] = 'fusion_npc', 'npc', 129, False
            row[5], row[9], row[10] = 1.5, 0., -1.5
            fused.append(tuple(row[:12]))
            steps[i] = (stamp+.1*i, ego, car[:5]+(0.,)+car[6:9]+(0., 0.)+car[11:], outline, route)
        original = LivePathSafety.__call__
        def call(provider, *args, **kwargs):
            provider.runtime.object_receiver.objects = list(fused)
            return original(provider, *args, **kwargs)
        with patch.object(LivePathSafety, '__call__', call):
            with patch('live_path_safety.static_body', side_effect=lambda obj, *args, **kwargs: obj):
                before = self.tunnel_vehicle_replay(steps)
            results = self.tunnel_vehicle_replay(steps)
        self.assertTrue(any(r.stop_reason == 'live path stopping envelope: fusion_npc' for r in before))
        for result in results:
            self.assertNotIn('fusion_npc', str(result.stop_reason))
            self.assertAlmostEqual(result.allowed_mps*3.6, 35., places=3)

    def moving_car_steps(self, speed, steps=7):
        from test_tunnel_tracks_recorded import box_outline
        route = np.column_stack((np.arange(0., 80., .5), np.zeros(160)))
        rows = []
        for i in range(steps):
            t, valid = .1*i, i >= 2   # the detector publishes zero until its velocity is valid
            y = -6.+speed*t   # crossing from the right, 16 m ahead, into our arrival
            car = (16., y, 4.6, 'lidar_car', 'npc', speed*valid, 4.6, 1.9, 90., 0., speed*valid,
                   500050, 0., None, valid, 0.)
            rows.append((100.+t, dict(x=0., y=0., heading_deg=0., speed_kmh=35.), car,
                         box_outline(16., y, 4.6, 1.9, 90.), route))
        return rows

    def test_tunnel_moving_car_still_stops_the_live_check(self):
        lidar_car = ('live path stopping envelope: lidar_car', 'live path collision: lidar_car')
        # slow: its measured face moves; several m/s: detector speed. A car-length cluster
        # starts moving only above tunnel_tracks.VEHICLE_BODY_MOVING_MPS (2.8 m/s, above
        # every parked-car slide recorded), so the slow case crosses at 3 m/s (was 2.5).
        for speed in (3., 6.):
            with self.subTest(speed=speed):
                with patch('live_path_safety.measured_vehicle', side_effect=lambda car, *args, **kwargs: car):
                    before = self.tunnel_vehicle_replay(self.moving_car_steps(speed, steps=12))
                results = self.tunnel_vehicle_replay(self.moving_car_steps(speed, steps=12))
                first = [i for i, r in enumerate(results) if r.stop_reason in lidar_car]
                self.assertTrue(first)
                self.assertTrue(results[first[0]].emergency)
                # Motion below the box drift needs the measured cluster (or a valid detector
                # speed): at most 0.3 s after the drift fit, and while the car's near end
                # (y+2.3 m) is still 1.15 m short of the path (body half width + margin).
                fitted = next(i for i, r in enumerate(before) if r.stop_reason in lidar_car)
                self.assertLessEqual(first[0]-fitted, 3)
                self.assertLess(-6.+speed*.1*first[0]+2.3, -1.15)

    def test_road_detector_velocity_is_not_reinterpreted(self):
        runtime = self.attach(self.runtime())
        slow = car(20., -4., 0., 1.5, 501)
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
            [slow], True, dict(stamp=runtime.test_scan['stamp'], age_s=.1)))
        runtime.on_road_vehicles = lambda cars: cars
        ego = runtime.test_status['ego']
        plan = DrivingPlan(1, runtime.test_route, ego, ego, 99.9, .1, time.monotonic(), 8., 60., False,
                           dict(valid=True, pose_valid=True, steering_valid=True, blocked=False), 'LATTICE')
        with patch('live_path_safety.measured_vehicle', side_effect=AssertionError('road car rewritten')):
            runtime.path_publisher._safety_provider(plan.path, ego, plan.mode, plan)
        track = runtime.path_publisher._safety_provider.guard.memory.tracks[('vehicles', 501)]
        np.testing.assert_allclose(track['velocity'], [0., 1.5])


if __name__ == '__main__':
    unittest.main()
