"""Regressions for one checked tunnel plan through lead, stop and gear decisions."""
from dataclasses import replace
import json
import math
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np

from driving_plan import DrivingPlan
from lead_follow import HeadingBias, STANDSTILL_GAP_M
import test_tunnel_integration as fixtures


class TunnelPlanContractTests(unittest.TestCase):
    def runtime(self, **options):
        return fixtures.TunnelIntegrationTests().runtime(**options)

    def plan(self, runtime, path=None, **changes):
        ego = dict(runtime.test_status['ego'])
        decision = DrivingPlan(1, runtime.test_route if path is None else path, ego, ego,
            runtime.test_scan['stamp'], .01, time.monotonic(), 4., 40., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False))
        return replace(decision, **changes)

    def publish(self, runtime, decision):
        publisher = runtime.path_publisher
        publisher.publish(decision.control_ego, decision.path, runtime.test_route,
            dict(mode='TUNNEL', reason='checked plan', validated_static=True, forward_clear_m=0.),
            decision_plan=decision)
        return json.loads(publisher.waypoints_pub.items[-1].data)

    def test_current_plan_replaces_previous_stop_and_zero_clearance(self):
        runtime = self.runtime()
        publisher = runtime.path_publisher
        publisher.zone_mps, publisher.emergency_stop = 0., True
        decision = self.plan(runtime)
        payload = self.publish(runtime, decision)
        self.assertGreater(publisher.velocity_pub.items[-1].data, 0.)
        self.assertFalse(publisher.emergency_stop)
        self.assertEqual(payload['decision']['forward_clear_m'], 40.)
        self.assertEqual(payload['decision']['sensor_stamp'], decision.sensor_stamp)
        actual = np.array([[point['x'], point['y']] for point in payload['waypoints']])
        np.testing.assert_array_equal(actual, decision.path)

    def test_external_path_mutation_cannot_change_a_checked_plan(self):
        runtime = self.runtime()
        path = runtime.test_route.copy()
        decision = self.plan(runtime, path)
        path[:, 1] = 3.
        self.assertTrue(np.all(decision.path[:, 1] == 0.))
        with self.assertRaises(ValueError):
            decision.path[:, 1] = 3.

    def test_checked_plan_is_published_without_legacy_lattice_trimming(self):
        runtime = self.runtime()
        path = runtime.test_route.copy()
        path[:, 0] -= 1.
        decision = self.plan(runtime, path)
        publisher = runtime.path_publisher
        publisher.publish(decision.control_ego, decision.path, path,
            dict(mode='LATTICE', reason='carried checked path', validated_static=True),
            decision_plan=decision)
        payload = json.loads(publisher.waypoints_pub.items[-1].data)
        actual = np.array([[point['x'], point['y']] for point in payload['waypoints']])
        np.testing.assert_array_equal(actual, decision.path)

    def test_changing_geometry_after_either_check_stops_the_last_valid_path(self):
        for check in ('geometry_path_digest', 'traffic_path_digest'):
            with self.subTest(check=check):
                runtime = self.runtime()
                decision = self.plan(runtime)
                decision = decision.decided(**{check: decision.path_digest()})
                previous = self.publish(runtime, decision)
                changed = decision.path.copy()
                changed[:, 1] += 1.
                stale_checks = replace(decision, path=changed)
                self.assertFalse(stale_checks.forward_ready(stale_checks.control_ego))
                self.publish(runtime, stale_checks)
                payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
                self.assertEqual(payload['waypoints'], previous['waypoints'])
                self.assertEqual(payload['target_speed_mps'], 0.)

    def test_changed_validation_pose_or_source_time_cannot_reuse_previous_checks(self):
        for fault in ('pose', 'sensor_stamp'):
            with self.subTest(fault=fault):
                runtime = self.runtime()
                decision = self.plan(runtime)
                decision = decision.decided(geometry_path_digest=decision.path_digest(),
                    geometry_frame_digest=decision.frame_digest(), traffic_frame_digest=decision.frame_digest())
                previous = self.publish(runtime, decision)
                if fault == 'pose':
                    changed = replace(decision, validation_ego=dict(decision.validation_ego, y=1.))
                else:
                    changed = replace(decision, sensor_stamp=decision.sensor_stamp+.05)
                self.publish(runtime, changed)
                payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
                self.assertEqual(payload['waypoints'], previous['waypoints'])
                self.assertEqual(payload['target_speed_mps'], 0.)
                self.assertFalse(changed.forward_ready(changed.control_ego))

    def test_invalid_sensor_or_wrong_geometry_cannot_authorize_forward(self):
        for fault in ('stale', 'future', 'wrong_path', 'wrong_pose'):
            with self.subTest(fault=fault):
                runtime = self.runtime()
                decision = self.plan(runtime)
                ego, path = dict(decision.control_ego), decision.path.copy()
                if fault == 'stale':
                    decision = replace(decision, checked_at=time.monotonic()-1.)
                elif fault == 'future':
                    decision = replace(decision, sensor_stamp=decision.sensor_stamp+1.)
                elif fault == 'wrong_path':
                    path[:, 1] += 1.
                else:
                    ego['heading_deg'] += 5.
                runtime.path_publisher.publish(ego, path, path,
                    dict(mode='TUNNEL', reason='must stop', validated_static=True), decision_plan=decision)
                self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_invalid_new_path_stops_last_bound_geometry_atomically(self):
        runtime = self.runtime()
        previous = self.publish(runtime, self.plan(runtime))
        publisher = runtime.path_publisher
        publisher.publish(runtime.test_status['ego'], np.zeros((1, 2)), runtime.test_route,
            dict(mode='TUNNEL', reason='invalid path', validated_static=True))
        stopped = json.loads(publisher.waypoints_pub.items[-1].data)
        self.assertEqual(stopped['waypoints'], previous['waypoints'])
        self.assertEqual(stopped['target_speed_mps'], 0.)
        self.assertTrue(stopped['decision']['emergency'])
        self.assertEqual(publisher.velocity_pub.items[-1].data, 0.)

    def test_missing_scan_stops_bound_bypass_and_fresh_scan_releases_it(self):
        fixture = fixtures.TunnelIntegrationTests()
        runtime = fixture.runtime(fixture.cluster())
        runtime.infer()
        previous = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        self.assertGreater(previous['target_speed_mps'], 0.)
        original = dict(runtime.test_scan)
        runtime.test_scan = dict(original, xyz=None, age_s=2.)
        runtime.infer()
        stopped = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        self.assertEqual(stopped['waypoints'], previous['waypoints'])
        self.assertEqual(stopped['target_speed_mps'], 0.)
        runtime.test_scan = dict(original, stamp=original['stamp']+.1, sequence=2)
        runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
        runtime.infer()
        resumed = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        self.assertGreater(resumed['target_speed_mps'], 0.)

    def test_reverse_ends_as_soon_as_forward_plan_is_ready_then_stops_in_r(self):
        for speed in (1.5, -1.5):
            with self.subTest(speed=speed):
                runtime = self.runtime()
                runtime.test_status['ego']['speed_kmh'] = speed
                publisher = runtime.path_publisher
                publisher.start_reverse(runtime.test_status['ego'])
                payload = self.publish(runtime, self.plan(runtime))
                self.assertIsNone(publisher.reverse_state)
                self.assertEqual(publisher.velocity_pub.items[-1].data, 0.)
                self.assertTrue(all(p['gear'] == 'R' for p in payload['waypoints']))
                runtime.test_status['ego']['speed_kmh'] = 0.
                with patch('gui.time.monotonic', return_value=time.monotonic()+.2):
                    payload = self.publish(runtime, self.plan(runtime))
                self.assertTrue(all(p['gear'] == 'D' for p in payload['waypoints']))
                self.assertGreater(publisher.velocity_pub.items[-1].data, 0.)

    def test_unsafe_forward_plan_keeps_reverse_and_stale_sensor_stops_reverse(self):
        for fault in ('blocked', 'steering', 'pose', 'clearance', 'stale'):
            with self.subTest(fault=fault):
                runtime = self.runtime()
                publisher = runtime.path_publisher
                publisher.start_reverse(runtime.test_status['ego'])
                decision = self.plan(runtime)
                if fault in ('blocked', 'steering', 'pose'):
                    key = {'blocked':'blocked', 'steering':'steering_valid', 'pose':'pose_valid'}[fault]
                    decision = replace(decision, validation=dict(decision.validation, **{key:fault == 'blocked'}))
                elif fault == 'clearance':
                    decision = replace(decision, forward_clear_m=2.)
                else:
                    decision = replace(decision, checked_at=time.monotonic()-1.)
                payload = self.publish(runtime, decision)
                self.assertIsNotNone(publisher.reverse_state)
                self.assertTrue(all(p['gear'] == 'R' for p in payload['waypoints']))
                if fault == 'stale':
                    self.assertEqual(publisher.velocity_pub.items[-1].data, 0.)

    def test_six_meter_limit_also_stops_before_selecting_d(self):
        runtime = self.runtime()
        publisher = runtime.path_publisher
        publisher.start_reverse(runtime.test_status['ego'])
        runtime.test_status['ego'].update(x=-6.1, speed_kmh=-1.5)
        payload = self.publish(runtime, self.plan(runtime, emergency=True))
        self.assertIsNone(publisher.reverse_state)
        self.assertTrue(all(p['gear'] == 'R' for p in payload['waypoints']))
        self.assertEqual(publisher.velocity_pub.items[-1].data, 0.)

    def test_tunnel_lead_checks_exact_curve_without_body_rotation(self):
        runtime = self.runtime()
        runtime.test_status['ego']['speed_kmh'] = 0.
        path = runtime.test_route.copy()
        path[:, 1] = path[:, 0]*math.tan(math.radians(7.))
        # A rear face 0.5 m inside the standstill gap (bumper at 2.3175 m).
        points = np.column_stack((np.full(8, 2.3175+STANDSTILL_GAP_M-.5), np.linspace(-1.25, -.65, 8)))
        runtime._lead_bias = HeadingBias()
        ordinary = runtime.lead_follow_step(runtime.test_status['ego'], time.monotonic(), [], points,
                                             path, True, False)
        self.assertEqual(ordinary['cap_mps'], 0.)
        decision = self.plan(runtime, path)
        tunnel = runtime.lead_follow_step(runtime.test_status['ego'], time.monotonic(), [], points,
            path, True, False, lidar_cars_in_objects=True, decision_plan=decision)
        self.assertIsNone(tunnel['cap_mps'])
        self.assertEqual(runtime._lead_state['path_digest'], decision.summary()['path_digest'])

    def test_held_lead_on_previous_path_is_rejected_after_checked_lane_change(self):
        runtime = self.runtime()
        runtime.test_status['ego']['speed_kmh'] = 0.
        points = np.column_stack((np.full(8, 20.), np.linspace(-.6, .6, 8)))
        now = time.monotonic()
        decision = self.plan(runtime)
        for delta in (0., .1):
            runtime.lead_follow_step(decision.validation_ego, now+delta, [], points,
                decision.path, True, False, decision_plan=replace(decision, sensor_stamp=decision.sensor_stamp+delta))
        self.assertIsNotNone(runtime._tunnel_lead_tracker.track)
        bypass = runtime.test_route.copy()
        u = np.clip(bypass[:, 0]/12., 0., 1.)
        bypass[:, 1] = 3.*u*u*(3.-2.*u)
        decision = self.plan(runtime, bypass)
        result = runtime.lead_follow_step(decision.validation_ego, now+.2, [], np.empty((0, 2)),
            decision.path, True, False, decision_plan=decision)
        self.assertIsNone(result['cap_mps'])
        self.assertIsNone(runtime._tunnel_lead_tracker.track)

    def test_new_scan_arriving_during_planning_stops_on_same_final_path(self):
        runtime = self.runtime()
        original = dict(runtime.test_scan)
        latest = dict(original, stamp=original['stamp']+.1, sequence=2,
            xyz=np.vstack((original['xyz'], [[3., 0., .8]])))
        runtime.lidar.scan_snapshot = lambda: original
        with patch.object(runtime.lidar, 'scan_snapshot', side_effect=[original, latest]):
            runtime.infer()
        payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertEqual(payload['decision']['sensor_stamp'], latest['stamp'])
        self.assertEqual(runtime._web_record['decision']['path_digest'], payload['decision']['path_digest'])
        self.assertIn('latest', runtime._web_record['stop_reason'])
        self.assertTrue(runtime._tunnel_decision_plan.validation['replan_deferred'])

    def late_scan(self, runtime, points):
        original = dict(runtime.test_scan)
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        latest = dict(original, stamp=original['stamp']+.1, sequence=2,
            xyz=np.vstack((original['xyz'], np.column_stack((points, np.full(len(points), .8))))))
        reads = [0]
        def snapshot():
            reads[0] += 1
            return original if reads[0] == 1 else latest
        return latest, snapshot

    def test_late_obstacle_replans_and_publishes_the_jointly_checked_bypass(self):
        for fallback in (False, True):
            with self.subTest(fallback=fallback):
                runtime = self.runtime(valid=not fallback)
                runtime.test_status['ego']['speed_kmh'] = 0.
                runtime.test_odometry['speed_kmh'] = 0.
                latest, snapshot = self.late_scan(runtime, fixtures.TunnelIntegrationTests.cluster(18.))
                method = 'fallback_lattice' if fallback else 'tunnel_lattice_plan'
                with patch.object(runtime.lidar, 'scan_snapshot', side_effect=snapshot), \
                        patch.object(runtime, method, wraps=getattr(runtime, method)) as search:
                    runtime.infer()
                self.assertLessEqual(search.call_count, 2)
                decision = runtime._tunnel_decision_plan
                self.assertTrue(decision.validation['replan_accepted'])
                self.assertEqual(decision.sensor_stamp, latest['stamp'])
                self.assertGreater(np.max(abs(decision.path[:, 1])), 1.)
                self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
                digest = decision.path_digest()
                self.assertEqual(decision.validation['geometry_path_digest'], digest)
                self.assertEqual(decision.validation['traffic_path_digest'], digest)
                self.assertEqual(decision.validation['geometry_frame_digest'], decision.frame_digest())
                self.assertEqual(decision.validation['traffic_frame_digest'], decision.frame_digest())
                self.assertEqual(decision.validation['lead']['path_digest'], digest)
                payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
                actual = np.array([[p['x'], p['y']] for p in payload['waypoints']])
                np.testing.assert_allclose(actual, decision.path, atol=1e-9)
                self.assertEqual(payload['decision']['path_digest'], digest)

    def test_replacement_touching_a_wall_is_rejected_without_publishing_it(self):
        runtime = self.runtime()
        runtime.test_status['ego']['speed_kmh'] = 0.
        runtime.test_odometry['speed_kmh'] = 0.
        latest, snapshot = self.late_scan(runtime, [[3., 0.]])
        original_search = runtime.tunnel_lattice_plan
        calls = [0]
        unsafe = runtime.test_route.copy()
        u = np.clip(unsafe[:, 0]/12., 0., 1.)
        unsafe[:, 1] = 5.*u*u*(3.-2.*u)
        def search(*args, **kwargs):
            calls[0] += 1
            if calls[0] == 1:
                return original_search(*args, **kwargs)
            runtime._tunnel_lattice_prev = dict(stamp=0., world_path=unsafe)
            return dict(path=unsafe, blocked=False, speed_cap_mps=4., safe_horizon_m=40.,
                        generator='lattice', side_margin_m=.35, end_margin_m=.3), []
        with patch.object(runtime.lidar, 'scan_snapshot', side_effect=snapshot), \
                patch.object(runtime, 'tunnel_lattice_plan', side_effect=search):
            runtime.infer()
        decision = runtime._tunnel_decision_plan
        self.assertEqual(calls[0], 2)
        self.assertTrue(decision.validation['replan_attempted'])
        self.assertFalse(decision.validation['replan_accepted'])
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertTrue(np.all(decision.path[:, 1] == 0.))
        self.assertIsNone(runtime._tunnel_lattice_prev)
        self.assertEqual(runtime._lead_state['path_digest'], decision.path_digest())

    def test_retry_failure_keeps_the_checked_stop_and_next_scan_recovers(self):
        runtime = self.runtime()
        runtime.test_status['ego']['speed_kmh'] = 0.
        runtime.test_odometry['speed_kmh'] = 0.
        latest, snapshot = self.late_scan(runtime, [[3., 0.]])
        original_search = runtime.tunnel_lattice_plan
        calls = [0]
        def search(*args, **kwargs):
            calls[0] += 1
            if calls[0] == 1:
                return original_search(*args, **kwargs)
            raise RuntimeError('replacement search failed')
        with patch.object(runtime.lidar, 'scan_snapshot', side_effect=snapshot), \
                patch.object(runtime, 'tunnel_lattice_plan', side_effect=search):
            self.assertTrue(runtime.infer())
        self.assertFalse(runtime._tunnel_decision_plan.validation['replan_accepted'])
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        runtime.test_scan = dict(runtime.test_scan, stamp=latest['stamp']+.1, sequence=3)
        runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
        self.assertTrue(runtime.infer())
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_fallback_crossing_vehicle_stop_uses_the_checked_path(self):
        runtime = self.runtime(valid=False)
        runtime.highway = SimpleNamespace(config=dict(lidar_regions=[dict(from_idx=0, to_idx=10000)]),
                                         active=lambda index: False, phase='OUTSIDE')
        runtime.latest_status = runtime.test_status
        car = (6., -2., 1., 'crossing_car', 'npc', 3., 1., .8, 90.,
               0., 3., 40001, 0., None, True, 0.)
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: ([car], True, {}))
        runtime.infer()
        decision = runtime._tunnel_decision_plan
        self.assertEqual(decision.validation['moving_conflict']['kind'], 'stop')
        self.assertEqual(decision.validation['moving_conflict']['name'], 'crossing_car')
        self.assertEqual(decision.validation['traffic_path_digest'], decision.path_digest())
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_search_outliving_pose_refreshes_sensor_and_pose_together(self):
        for obstacle in (False, True):
            with self.subTest(obstacle=obstacle):
                runtime = self.runtime()
                original = dict(runtime.test_scan)
                ego = dict(runtime.test_status['ego'])
                fresh_ego = dict(ego, x=2., stamp=ego['stamp']+.4)
                fresh_status = dict(runtime.test_status, ego=fresh_ego)
                latest_xyz = original['xyz'].copy()
                latest_xyz[:, 0] -= 2.
                if obstacle:
                    latest_xyz = np.vstack((latest_xyz, [[3., 0., .8]]))
                latest = dict(original, stamp=original['stamp']+.4, sequence=2, xyz=latest_xyz)
                runtime.tunnel_odometry.at_stamp = lambda stamp: dict(
                    fresh_ego if stamp >= latest['stamp'] else ego, stamp=stamp)
                with patch.object(runtime, 'update_ego', side_effect=[runtime.test_status, fresh_status]), \
                        patch.object(runtime.lidar, 'scan_snapshot', side_effect=[original, latest]):
                    runtime.infer()
                decision = runtime._tunnel_decision_plan
                self.assertTrue(decision.validation['pose_refreshed'])
                self.assertEqual(decision.control_ego['stamp'], latest['stamp'])
                self.assertEqual(decision.sensor_stamp, latest['stamp'])
                self.assertEqual(decision.emergency, obstacle)
                payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
                self.assertEqual(payload['source_stamp'], latest['stamp'])
                self.assertEqual(payload['decision']['path_digest'], decision.summary()['path_digest'])
                if obstacle:
                    self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
                else:
                    self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
                    self.assertAlmostEqual(payload['waypoints'][0]['x'], 2., places=4)

    def test_fallback_new_forward_plan_reaches_reverse_termination(self):
        runtime = self.runtime(valid=False)
        runtime.test_status['ego']['speed_kmh'] = 1.5
        publisher = runtime.path_publisher
        publisher.start_reverse(runtime.test_status['ego'])
        runtime.infer()
        self.assertGreater(runtime.last_result['postprocess']['forward_clear_m'], 8.)
        self.assertIsNone(publisher.reverse_state)
        payload = json.loads(publisher.waypoints_pub.items[-1].data)
        self.assertTrue(all(point['gear'] == 'R' for point in payload['waypoints']))
        self.assertEqual(publisher.velocity_pub.items[-1].data, 0.)
        runtime.test_status['ego']['speed_kmh'] = 0.
        runtime.infer()
        payload = json.loads(publisher.waypoints_pub.items[-1].data)
        self.assertTrue(all(point['gear'] == 'D' for point in payload['waypoints']))
        self.assertGreater(publisher.velocity_pub.items[-1].data, 0.)

    def test_pose_refresh_extends_sharp_join_without_relaxing_steering_or_wall_gates(self):
        from gui import continue_static_plan, path_curvatures, MAX_CONTROL_CURVATURE
        runtime = self.runtime()
        ego = dict(runtime.test_status['ego'])
        ego['speed_kmh'] = 14.
        fresh = dict(ego, x=2., y=-.3, stamp=ego['stamp']+.4)
        runtime.update_ego = lambda: dict(ego=fresh)
        runtime.tunnel_odometry.at_stamp = lambda stamp: dict(fresh if stamp >= fresh['stamp'] else ego, stamp=stamp)
        original = dict(runtime.test_scan)
        latest = dict(original, stamp=fresh['stamp'], sequence=2)
        runtime.lidar.scan_snapshot = lambda: latest
        x = np.arange(0., 60.5, .5)
        u = np.clip(x/12., 0., 1.)
        curve = np.column_stack((x, 3.5*u**3*(10.-15.*u+6.*u*u)))
        saved = dict(stamp=time.monotonic(), world_path=curve, plan={})
        short = continue_static_plan(saved, fresh, saved['stamp'])
        self.assertGreater(np.max(path_curvatures(short['path'])), MAX_CONTROL_CURVATURE*1.15)
        decision, _ = runtime.checked_tunnel_plan(curve, ego, ego, original, np.empty((0, 2)),
            4., 40., False, dict(valid=True, pose_valid=True, obstacle=60., wall=60., lane=60.))
        self.assertTrue(decision.validation['pose_refreshed'])
        self.assertGreater(decision.validation['pose_join_m'], 6.)
        self.assertTrue(decision.validation['steering_valid'])
        self.assertFalse(decision.emergency)
        self.assertLessEqual(decision.validation['curvature_1pm'], MAX_CONTROL_CURVATURE*1.15)
        self.assertTrue(decision.checks_match_path())

    def test_final_latest_scan_uses_selected_tunnel_background_extraction(self):
        from unittest.mock import Mock
        for obstacle in (False, True):
            with self.subTest(obstacle=obstacle):
                runtime = self.runtime()
                original = dict(runtime.test_scan)
                latest = dict(original, stamp=original['stamp']+.1, sequence=2,
                    xyz=np.vstack((original['xyz'], [[3., 0., 2.4]])))
                runtime.lidar.scan_snapshot = lambda: latest
                runtime._live_tunnel_corridor = dict(wall_stats={})
                filtered = np.array([[3., 0.]]) if obstacle else np.empty((0, 2))
                runtime.tunnel.project_scan = Mock(return_value=dict(valid=True, obstacle_points=filtered))
                ego = dict(runtime.test_status['ego'])
                decision, _ = runtime.checked_tunnel_plan(runtime.test_route, ego, ego,
                    original, np.empty((0, 2)), 4., 40., False,
                    dict(valid=True, pose_valid=True, obstacle=60., wall=60., lane=60.))
                runtime.tunnel.project_scan.assert_called_once()
                self.assertEqual(decision.emergency, obstacle)
                self.assertEqual(decision.sensor_stamp, latest['stamp'])


if __name__ == '__main__':
    unittest.main()
