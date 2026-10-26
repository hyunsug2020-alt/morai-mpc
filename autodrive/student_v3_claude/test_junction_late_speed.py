"""Keep approach braking while deferring the fixed entry ceiling to 20 m."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from ego_body import EGO_FRONT_M
from live_path_safety import LivePathSafety
from stop_line import (INTERSECTION_ENTRY_MPS, INTERSECTION_ENTRY_CAP_FRONT_GAP_M,
    YIELD_CHECK_DISTANCE_M, competition_stop_controller, intersection_entry_cap_mps,
    yield_line_cap)
from virtual_sim import SimulatedVehicleReceiver
import test_live_path_safety as live_fixture
import test_roundabout_empty_entry as entry_fixture


class LateEntrySpeedTests(unittest.TestCase):
    def runtime(self, gap, speed_kmh=55.):
        fixture, runtime = entry_fixture.EmptyRuntimeTests().runtime()
        runtime.stop_line_controller = competition_stop_controller([entry_fixture.LINE])
        runtime.intersection_regions[0]['start_station_m'] = -30.
        runtime.intersection_regions[0]['line_id'] = 'entry'
        runtime.test_status['ego'].update(x=50.-EGO_FRONT_M-gap, speed_kmh=speed_kmh)
        runtime.speed_zones = SimpleNamespace(kmh_at=lambda index: (55., {}))
        runtime.path_publisher.cruise_mps = 55./3.6
        runtime.path_publisher._last_target_mps = 55./3.6
        runtime.path_publisher._last_target_time = 100.
        runtime.vehicle_receiver = SimulatedVehicleReceiver()
        runtime.object_receiver.observation_stamp = 100.
        runtime.object_receiver.received = 100.
        runtime.path_publisher._safety_provider = LivePathSafety(runtime)
        return fixture, runtime

    def test_far_approach_keeps_its_braking_envelope_and_near_entry_keeps_30(self):
        for gap in (60., 40., 20., 18.):
            fixture, runtime = self.runtime(gap)
            with self.subTest(gap=gap), patch('time.monotonic', return_value=100.):
                runtime.vehicle_receiver.publish_rows([], stamp=100.)
                row = fixture.step(runtime, 0)
                plan = runtime.path_publisher._active_decision_plan
                payload = live_fixture.LivePathSafetyTests().payload(runtime)
                self.assertFalse(plan.emergency, plan.stop_reason)
                self.assertGreater(payload['target_speed_mps'], 0.)
                self.assertFalse(runtime.gap_planner.junction_committed)
                self.assertFalse(runtime.stop_line_controller.released)
                live = plan.validation['live_safety']['junction_traffic']
                self.assertAlmostEqual(live['front_gap_m'], gap)
                if gap > INTERSECTION_ENTRY_CAP_FRONT_GAP_M:
                    self.assertIsNone(runtime.gap_planner.entry_cap_mps)
                    self.assertGreater(payload['target_speed_mps'], INTERSECTION_ENTRY_MPS)
                    self.assertLessEqual(payload['target_speed_mps'], yield_line_cap(gap, 55./3.6, .01)+1e-6)
                    self.assertIsNone(live['ring'])
                else:
                    self.assertEqual(runtime.gap_planner.entry_cap_mps, INTERSECTION_ENTRY_MPS)
                    self.assertLessEqual(payload['target_speed_mps'], INTERSECTION_ENTRY_MPS)
                if gap == YIELD_CHECK_DISTANCE_M:
                    self.assertEqual(live['ring']['action'], 'GO')

    def test_invalid_progress_keeps_the_existing_conservative_entry_cap(self):
        for gap in (math.nan, math.inf, -math.inf, None, 'bad', 20., -1.):
            with self.subTest(gap=gap):
                self.assertEqual(intersection_entry_cap_mps(gap), INTERSECTION_ENTRY_MPS)
        self.assertIsNone(intersection_entry_cap_mps(20.001))
        self.assertGreater(INTERSECTION_ENTRY_CAP_FRONT_GAP_M, YIELD_CHECK_DISTANCE_M)

    def test_current_body_and_people_still_stop_a_far_approach(self):
        for hazard in ('raw', 'person'):
            fixture, runtime = self.runtime(60.)
            if hazard == 'raw':
                import numpy as np
                runtime.lidar.pts = np.array([[3., 0.]])
            else:
                runtime.object_receiver.objects = [live_fixture.person(
                    runtime.test_status['ego']['x']+3., 0., 0., 0.)]
            with self.subTest(hazard=hazard), patch('time.monotonic', return_value=100.):
                runtime.vehicle_receiver.publish_rows([], stamp=100.)
                fixture.step(runtime, 0)
                self.assertEqual(live_fixture.LivePathSafetyTests().payload(runtime)['target_speed_mps'], 0.)
                self.assertTrue(runtime.path_publisher._active_decision_plan.emergency)
                self.assertFalse(runtime.gap_planner.junction_committed)


if __name__ == '__main__':
    unittest.main()
