"""The advanced physical yield line must not brake a clear 80 m approach."""
import unittest
from unittest.mock import patch

from ego_body import EGO_FRONT_M
from live_path_safety import LivePathSafety
from stop_line import competition_stop_controller
import test_junction_live_profile as profile_fixture
import test_roundabout_empty_entry as entry_fixture


class ApproachActivationTests(unittest.TestCase):
    def runtime(self):
        helper, runtime = entry_fixture.EmptyRuntimeTests().runtime()
        runtime.test_status['ego'].update(x=0., speed_kmh=54.7, stamp=100., age_s=.01)
        runtime.intersection_regions[0].update(line_id='entry', stop_station_m=80.,
            exit_station_m=110., end_station_m=135.)
        line = dict(entry_fixture.LINE, station_m=87., xy=[87., 0.],
                    surveyed_station_m=80., yield_advance_m=7., yield_launch_capture_m=3.)
        runtime.stop_line_controller = competition_stop_controller([line])
        runtime.speed_zones.kmh_at = lambda index: (55., {})
        runtime.path_publisher.cruise_mps = 55./3.6
        runtime.object_receiver.snapshot_with_time = lambda: ([], 100., .01)
        runtime.vehicle_receiver.snapshot = lambda: ([], True,
            dict(active=True, ready=True, age_s=.01, stamp=100., vehicles=[]))
        runtime.path_publisher._safety_provider = LivePathSafety(runtime)
        return helper, runtime

    def test_first_detector_frame_keeps_a_positive_braking_approach(self):
        helper, runtime = self.runtime()
        row = helper.step(runtime, 0)
        self.assertIsNotNone(runtime.stop_line_controller.active)
        self.assertEqual(runtime.gap_planner.junction['line_station_m'], 87.)
        self.assertAlmostEqual(runtime.gap_planner.junction['line_gap_m'], 87.-EGO_FRONT_M)
        self.assertFalse(row['emergency_stop'], row.get('stop_reason'))
        self.assertGreater(row['published_cap_kmh'], 40.)
        self.assertNotIn('junction_entry_profile', row['decision']['validation'])
        self.assertFalse(runtime.stop_line_controller.released)
        self.assertFalse(runtime.gap_planner.junction_committed)

    def test_activation_still_keeps_an_independent_front_obstacle_stopped(self):
        helper, runtime = self.runtime()
        import numpy as np
        runtime.lidar.pts = np.array([[3., 0.]])
        row = helper.step(runtime, 0)
        self.assertEqual(row['published_cap_kmh'], 0.)
        self.assertFalse(runtime.stop_line_controller.released)

    def test_gui_leaves_ring_timing_with_the_fresh_publication_owner(self):
        from traffic_planner import GapPlanner
        helper, runtime = self.runtime()
        evaluate = GapPlanner.evaluate
        gui_calls = []
        def checked(planner, *args, **kwargs):
            if planner is runtime.gap_planner:
                gui_calls.append(True)
            return evaluate(planner, *args, **kwargs)
        with patch.object(GapPlanner, 'evaluate', autospec=True, side_effect=checked):
            row = helper.step(runtime, 0)
        self.assertEqual(gui_calls, [])
        self.assertGreater(row['published_cap_kmh'], 40.)

    def test_next_gui_frame_preserves_an_accepted_launch_before_bumper_crossing(self):
        helper, runtime = self.runtime()
        gate = runtime.stop_line_controller
        ego = runtime.test_status['ego']
        ego.update(x=87.-EGO_FRONT_M-.9, speed_kmh=0.)
        gate.active = gate.lines[0]
        gate.released = gate.yield_launch_pending = True
        runtime.gap_planner.junction_committed = True
        row = helper.step(runtime, 0)
        self.assertTrue(runtime.gap_planner.junction_committed)
        self.assertTrue(gate.released)
        self.assertGreater(row['published_cap_kmh'], 0., row.get('stop_reason'))

    def test_timed_owner_also_owns_duplicate_crossing_caution_and_soft_acc_caps(self):
        helper, runtime = self.runtime()
        runtime._lead_state = {}
        runtime.lead_follow_step = lambda *a, **k: dict(cap_mps=0., emergency=False,
            state=dict(gap_m=1.7))
        with patch('gui.crossing_caution', side_effect=AssertionError('duplicate map traffic check')):
            row = helper.step(runtime, 0)
        self.assertGreater(row['published_cap_kmh'], 40., row.get('stop_reason'))

    def timing_scene(self):
        from virtual_sim import SimulatedVehicleReceiver
        helper = profile_fixture.JunctionLiveProfileTests()
        runtime, plan, world = helper.fixture(following=False)
        vehicles, _, _ = runtime.vehicle_receiver.snapshot()
        runtime.vehicle_receiver = SimulatedVehicleReceiver()
        with patch('time.monotonic', return_value=100.):
            runtime.vehicle_receiver.publish_rows(vehicles, stamp=99.9)
        line = dict(entry_fixture.LINE, station_m=10., xy=[10., 0.])
        runtime.stop_line_controller = competition_stop_controller([line])
        runtime.stop_line_controller.update(0., 0., 100., False,
            observation_age_s=.02, empty_front_station=EGO_FRONT_M)
        runtime.gap_planner.junction.update(yield_only=True, constant_ring_speed=True,
            held=True, line_id='entry', line_station_m=10., line_gap_m=10.-EGO_FRONT_M)
        runtime.lanes.route_progress = lambda ego: ego['x']
        plan = plan.decided(allowed_mps=24./3.6)
        return helper, runtime, plan, world

    def test_same_publication_does_not_run_map_classification_twice(self):
        import ring_entry
        helper, runtime, plan, world = self.timing_scene()
        with patch('time.monotonic', return_value=100.), \
                patch('ring_entry.classify', wraps=ring_entry.classify) as classify:
            result = helper.check(runtime, plan, world)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertTrue(result.validation['live_safety']['junction_timing_reused'])
        self.assertEqual(classify.call_count, 1)

    def test_front_person_still_revokes_a_cached_map_gap(self):
        from test_ped_guard import person
        helper, runtime, plan, world = self.timing_scene()
        runtime.object_receiver.objects = [person(2.5, 0., 0., 0.)]
        with patch('time.monotonic', return_value=100.):
            result = helper.check(runtime, plan, world)
        self.assertTrue(result.emergency)
        self.assertEqual(result.allowed_mps, 0.)

    def test_reduced_command_ceiling_requires_a_new_timing_assessment(self):
        import ring_entry
        helper, runtime, plan, world = self.timing_scene()
        plan = plan.decided(allowed_mps=1.28)
        with patch('time.monotonic', return_value=100.), \
                patch('ring_entry.classify', wraps=ring_entry.classify) as classify:
            result = helper.check(runtime, plan, world)
        self.assertLessEqual(result.allowed_mps, 1.28)
        self.assertFalse(result.validation['live_safety'].get('junction_timing_reused', False))
        self.assertEqual(classify.call_count, 2)

    def test_reusing_timing_after_body_checks_cannot_extend_its_deadline(self):
        from path_safety import PathSafety
        helper, runtime, plan, world = self.timing_scene()
        clock = [100.]
        original = PathSafety.check
        def delayed(guard, *args, **kwargs):
            result = original(guard, *args, **kwargs)
            clock[0] += .04
            return result
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(PathSafety, 'check', delayed):
            result = helper.check(runtime, plan, world)
        self.assertTrue(result.validation['live_safety']['junction_timing_reused'])
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertLessEqual(result.validation['junction_live']['expires_at'],
                             result.validation['junction_yield_motion']['expires_at'])


if __name__ == '__main__':
    unittest.main()
