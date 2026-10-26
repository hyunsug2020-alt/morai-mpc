"""Exercise preparation isolation, boundary ownership and fresh collision vetoes."""
from concurrent.futures import Future
import json
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from planner_transition import PlannerTransition, approach_corridor
import test_tunnel_integration as tunnel_fixture
from tunnel_map import TunnelMap


class InlineExecutor:
    def submit(self, function, *args):
        future = Future()
        try:
            future.set_result(function(*args))
        except Exception as exc:
            future.set_exception(exc)
        return future


class PlannerTransitionTests(unittest.TestCase):
    def runtime(self):
        runtime = tunnel_fixture.TunnelIntegrationTests().runtime()
        runtime.tunnel = TunnelMap(dict(origin_xy=[20., 0.], axis_heading_deg=0.,
            left_offset_m=5., right_offset_m=-5., longitudinal_min_m=0.,
            longitudinal_max_m=60., max_lateral_correction_m=.8,
            max_yaw_correction_deg=3., wall_filter_tolerance_m=.03))
        walls = np.array([(x, y, z) for x in np.arange(20., 75., .5)
                          for y in (-5., 5.) for z in (.5, 1., 1.5)], dtype=np.float32)
        ground = runtime.test_background_xyz[:1440]
        runtime.test_scan['xyz'] = np.vstack((ground, walls))
        runtime.latest_status = runtime.test_status
        runtime._planner_transition = PlannerTransition(InlineExecutor())
        runtime._infer_current = lambda status: runtime.path_publisher.publish(
            status['ego'], runtime.test_route, runtime.test_route,
            dict(mode='LATTICE', reason='checked ordinary route', validated_static=True,
                 applied_peak_m=0., forward_clear_m=60.))
        runtime.path_publisher.zone_mps = 28.94/3.6
        return runtime

    def test_preparation_has_no_control_or_localization_side_effects(self):
        runtime = self.runtime()
        shadow = PlannerTransition._snapshot(runtime, runtime.test_status)
        proposal = PlannerTransition._prepare(shadow, 'TUNNEL', 0, None)
        self.assertTrue(proposal['ready'], proposal['reason'])
        self.assertTrue(proposal['takeover_ready'])
        self.assertFalse(runtime.path_publisher.waypoints_pub.items)
        runtime.path_publisher.publish_tunnel_constraint.assert_not_called()
        self.assertIsNone(getattr(runtime, '_tunnel_cache', None))
        self.assertFalse(hasattr(runtime, '_tunnel_memory'))

    def test_prepare_before_portal_and_take_over_at_boundary_with_one_writer(self):
        runtime = self.runtime()
        self.assertFalse(runtime.tunnel.active(runtime.test_status['ego']))
        self.assertTrue(runtime.infer())
        self.assertEqual(len(runtime.path_publisher.waypoints_pub.items), 1)
        self.assertTrue(runtime.infer())
        self.assertEqual(len(runtime.path_publisher.waypoints_pub.items), 2)
        self.assertIsNone(runtime._planner_transition.owner)
        self.assertTrue(runtime._transition_entry_corridor)
        runtime.path_publisher.publish_tunnel_constraint.assert_not_called()
        runtime.test_status['ego']['x'] = 20.1
        self.assertTrue(runtime.infer())
        self.assertEqual(len(runtime.path_publisher.waypoints_pub.items), 3)
        self.assertEqual(runtime._planner_transition.owner, 'TUNNEL')
        self.assertFalse(runtime._transition_approach)
        self.assertGreater(runtime._web_record['published_cap_kmh'], 0.)
        self.assertLessEqual(runtime._web_record['published_cap_kmh'], 28.94+1e-6)
        payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        self.assertEqual(payload['plan_id'], runtime.path_publisher._published_plan['plan_id'])
        self.assertEqual(payload['target_speed_mps'], runtime.path_publisher.velocity_pub.items[-1].data)

    def test_prepared_clear_path_is_rechecked_against_a_new_raw_blocker(self):
        runtime = self.runtime()
        runtime.infer()
        wall = np.array([(3., y, .8) for y in np.arange(-5., 5.1, .1)], dtype=np.float32)
        runtime.test_scan['xyz'] = np.vstack((runtime.test_scan['xyz'], wall))
        runtime.test_scan['stamp'] += .1
        runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
        runtime.test_status['ego']['x'] = 20.1
        runtime.infer()
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertTrue(runtime._web_record['emergency_stop'])

    def test_fresh_handoff_reuses_checked_path_without_another_full_search(self):
        runtime = self.runtime()
        runtime.infer()
        runtime.infer()
        runtime._tunnel_crawl_until = time.monotonic()+3.
        runtime._tunnel_hold_mps = .1
        runtime._tunnel_clear_n = 0
        runtime.test_status['ego']['x'] = 20.1
        with patch.object(runtime, 'tunnel_lattice_plan', side_effect=AssertionError('handoff searched again')):
            self.assertTrue(runtime.infer())
        self.assertTrue(runtime._tunnel_decision_plan.validation['handoff_reused'])
        self.assertTrue(runtime._web_record['transition']['revalidated'])
        decision = runtime.path_publisher._published_plan['decision']
        self.assertEqual(decision['sensor_stamp'], runtime.test_scan['stamp'])
        self.assertEqual(decision['path_digest'], runtime._tunnel_decision_plan.summary()['path_digest'])
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, .1)

    def test_reversing_command_cannot_seed_a_forward_handoff(self):
        from planner_transition import seed_from_command
        runtime = self.runtime()
        runtime.path_publisher.start_reverse(runtime.test_status['ego'])
        runtime.path_publisher.publish_reverse(runtime.test_status['ego'])
        self.assertIsNone(seed_from_command(runtime.path_publisher._published_plan, time.monotonic()))

    def test_passing_route_entry_keeps_prepared_plan_when_current_command_is_stopped(self):
        runtime = self.runtime()
        runtime.infer()
        runtime.infer()
        transition = runtime._planner_transition
        self.assertTrue(transition.prepared['ready'])
        transition.last_command = dict(transition.last_command, emergency=True, target_mps=0.)
        index = int(np.flatnonzero(runtime.waypoint_xy[:, 0] >= 20.)[0])+1
        runtime.test_status['path']['nearest_idx'] = index
        runtime.test_status['ego']['x'] = float(runtime.waypoint_xy[index, 0])
        self.assertTrue(runtime.tunnel.active(runtime.test_status['ego']))
        self.assertIsNone(transition._entry_distance(runtime, runtime.tunnel, index,
                                                     runtime.test_status['ego']))
        with patch.object(runtime, 'tunnel_lattice_plan',
                          side_effect=AssertionError('lost ready boundary plan')):
            self.assertTrue(runtime.infer())
        self.assertTrue(runtime._web_record['transition']['prepared'])
        self.assertTrue(runtime._tunnel_decision_plan.validation['handoff_reused'])
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_pending_worker_does_not_stop_the_active_planner(self):
        runtime = self.runtime()
        transition = runtime._planner_transition
        transition.future = Future()
        transition.future_target = 'TUNNEL'
        for _ in range(2):
            runtime.infer()
            self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertIsNone(transition.owner)
        self.assertEqual(len(runtime.path_publisher.waypoints_pub.items), 2)

    def test_distant_crossing_of_tunnel_axis_does_not_prepare_or_change_corridor(self):
        runtime = self.runtime()
        runtime.test_status['ego']['y'] = 260.
        transition = runtime._planner_transition
        transition.info = dict(state='READY', target='TUNNEL', prepared=True)
        self.assertIsNone(transition.before(runtime, runtime.test_status))
        self.assertIsNone(transition.future)
        self.assertFalse(runtime._transition_entry_corridor)
        self.assertEqual(transition.info['state'], 'IDLE')

    def test_nearby_road_without_a_future_portal_entry_does_not_prepare(self):
        runtime = self.runtime()
        runtime.waypoint_xy = runtime.waypoint_xy.copy()
        runtime.waypoint_xy[:, 1] = 17.
        runtime.test_status['ego']['y'] = 17.
        self.assertIsNone(runtime._planner_transition.before(runtime, runtime.test_status))
        self.assertIsNone(runtime._planner_transition.future)
        self.assertFalse(runtime._transition_entry_corridor)

    def test_leaving_the_preparation_window_clears_the_ready_state(self):
        runtime = self.runtime()
        runtime.infer()
        runtime.infer()
        self.assertTrue(runtime._transition_entry_corridor)
        runtime.test_status['ego']['x'] = -100.
        runtime._planner_transition.before(runtime, runtime.test_status)
        self.assertFalse(runtime._transition_entry_corridor)
        self.assertIsNone(runtime._planner_transition.prepared)
        self.assertEqual(runtime._planner_transition.info['state'], 'IDLE')

    def test_unavailable_preview_snapshot_keeps_the_active_writer_running(self):
        runtime = self.runtime()
        with patch.object(PlannerTransition, '_snapshot', side_effect=ValueError('next sensor warming')):
            self.assertTrue(runtime.infer())
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertIn('snapshot unavailable', runtime._planner_transition.info['reason'])

    def test_surveyed_proposal_does_not_take_over_before_supported_walls(self):
        runtime = self.runtime()
        xyz = runtime.test_scan['xyz']
        runtime.test_scan['xyz'] = xyz[(xyz[:, 2] == 0.) | (xyz[:, 0] < 31.)]
        shadow = PlannerTransition._snapshot(runtime, runtime.test_status)
        proposal = PlannerTransition._prepare(shadow, 'TUNNEL', 0, None)
        self.assertTrue(proposal['ready'], proposal['reason'])
        self.assertFalse(proposal['takeover_ready'])
        transition = runtime._planner_transition
        transition.last_submit = time.monotonic()
        transition.prepared = proposal
        self.assertIsNone(transition.before(runtime, runtime.test_status))
        self.assertTrue(runtime._transition_entry_corridor)
        self.assertIsNone(transition.owner)

    def test_handoff_keeps_the_active_avoidance_direction_as_a_checked_candidate(self):
        runtime = self.runtime()
        transition = runtime._planner_transition
        now = time.monotonic()
        transition.prepared = dict(target='TUNNEL', ready=True, takeover_ready=True,
            received=now, generation=0, seed=dict(stamp=now, target=3., world_path=runtime.test_route))
        transition.last_command = dict(received=now, target_mps=5., offset_m=-2.,
            world_path=runtime.test_route, emergency=False)
        runtime.test_status['ego']['x'] = 20.1
        self.assertEqual(transition.before(runtime, runtime.test_status), 'TUNNEL')
        self.assertEqual(runtime._transition_seed['target'], -2.)

    def test_expired_preparation_cannot_authorize_takeover(self):
        runtime = self.runtime()
        transition = runtime._planner_transition
        transition.last_submit = time.monotonic()
        transition.prepared = dict(target='TUNNEL', ready=True, takeover_ready=True,
            received=time.monotonic()-2., seed={}, generation=0)
        self.assertIsNone(transition.before(runtime, runtime.test_status))
        self.assertIsNone(transition.owner)

    def test_clock_reset_discards_a_pending_result_from_the_old_timeline(self):
        runtime = self.runtime()
        transition = runtime._planner_transition
        transition.last_stamp = runtime.test_status['ego']['stamp']+1.
        transition.future = Future()
        transition.future.set_result(dict(target='TUNNEL', ready=True, reason='old',
            received=time.monotonic(), seed={}, generation=0))
        transition.last_submit = time.monotonic()
        self.assertIsNone(transition.before(runtime, runtime.test_status))
        self.assertIsNone(transition.prepared)
        self.assertEqual(transition.generation, 1)

    def test_preview_never_takes_control_inside_the_highway_region(self):
        runtime = self.runtime()
        runtime.highway = SimpleNamespace(active=lambda index: True,
            config=dict(to_idx=3500), phase='CRUISE')
        runtime.test_status['path']['nearest_idx'] = 3490
        transition = runtime._planner_transition
        transition.future = Future()
        transition.prepared = dict(target='TUNNEL', ready=True, takeover_ready=True,
            received=time.monotonic(), seed={}, generation=0)
        self.assertIsNone(transition.before(runtime, runtime.test_status))
        self.assertIsNone(transition.owner)

    def test_boundary_position_noise_does_not_toggle_back_to_the_general_planner(self):
        runtime = self.runtime()
        transition = runtime._planner_transition
        transition.owner = 'TUNNEL'
        runtime.test_status['ego']['x'] = 19.5
        self.assertEqual(transition.before(runtime, runtime.test_status), 'TUNNEL')
        runtime.test_status['ego']['x'] = 81.
        self.assertIsNone(transition.before(runtime, runtime.test_status))
        self.assertIsNone(transition.owner)

    def test_portal_corridor_preserves_ordinary_lane_and_physical_walls(self):
        runtime = self.runtime()
        runtime.lanes.corridor = lambda ego, route, **kwargs: __import__('lane_corridor').LaneCorridor(
            route, np.full(len(route), 2.), np.full(len(route), -2.))
        corridor = approach_corridor(runtime.tunnel, runtime.lanes,
                                     runtime.test_status['ego'], runtime.test_route)
        early = runtime.test_route.copy()
        early[:, 1] = 3.
        self.assertLess(corridor.clear_distance(early, 60.), 5.)
        late = runtime.test_route.copy()
        late[:, 1] = np.clip((late[:, 0]-23.)/10., 0., 1.)*2.5
        self.assertGreater(corridor.clear_distance(late, 60.), 55.)
        late[:, 1] *= 3.
        self.assertLess(corridor.clear_distance(late, 60.), 45.)

    def test_safe_forward_plan_shortens_reverse_and_stops_before_gear_change(self):
        runtime = self.runtime()
        publisher = runtime.path_publisher
        ego = dict(runtime.test_status['ego'], speed_kmh=1.5)
        publisher.start_reverse(ego)
        post = dict(mode='TUNNEL BYPASS', reason='fresh checked forward escape',
                    validated_static=True, forward_clear_m=20.)
        publisher.publish(ego, runtime.test_route, runtime.test_route, post)
        self.assertIsNone(publisher.reverse_state)
        payload = json.loads(publisher.waypoints_pub.items[-1].data)
        self.assertTrue(all(point['gear'] == 'D' for point in payload['waypoints']))
        self.assertEqual(publisher.velocity_pub.items[-1].data, 0.)
        ego['speed_kmh'] = 0.
        publisher.publish(ego, runtime.test_route, runtime.test_route, post)
        self.assertFalse(publisher._reverse_release_stop)

    def test_blocked_or_unvalidated_forward_path_does_not_cancel_reverse(self):
        for emergency, validated, clearance in [(True, True, 60.), (False, False, 60.), (False, True, 2.)]:
            with self.subTest(emergency=emergency, validated=validated, clearance=clearance):
                runtime = self.runtime()
                publisher = runtime.path_publisher
                publisher.emergency_stop = emergency
                publisher.start_reverse(runtime.test_status['ego'])
                publisher.publish(runtime.test_status['ego'], runtime.test_route, runtime.test_route,
                    dict(mode='TUNNEL', reason='blocked', validated_static=validated,
                         forward_clear_m=clearance))
                self.assertIsNotNone(publisher.reverse_state)
                payload = json.loads(publisher.waypoints_pub.items[-1].data)
                self.assertTrue(all(point['gear'] == 'R' for point in payload['waypoints']))


if __name__ == '__main__':
    unittest.main()
