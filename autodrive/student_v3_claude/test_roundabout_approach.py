"""Physical stopping and independently refreshed roundabout entry checks."""
import copy
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from driving_plan import DrivingPlan
from live_path_safety import LivePathSafety
from stop_line import (StopLineController, INTERSECTION_ACCEL_MPS2, STOP_WINDOW_EXTRA_M,
                       STOP_REST_EXTRA_M)
from traffic_planner import GapPlanner
from ring_entry import following_speed_cap
import test_junction_entry as junction_fixture
import test_live_path_safety as live_fixture
import test_stop_line as stop_fixture


PATH = np.column_stack((np.arange(0., 40.01, .5), np.zeros(81)))
EGO = dict(x=0., y=0., heading_deg=0., speed_kmh=0., stamp=100.)


def planner(gap, speed=0.):
    result = GapPlanner()
    result.ACCEL_MPS2 = INTERSECTION_ACCEL_MPS2
    result.junction = dict(held=True, ready=True, unknown_ids=[],
                          line_id='roundabout-entry', line_gap_m=gap)
    return result, dict(EGO, speed_kmh=speed*3.6)


class RoundaboutApproachTests(unittest.TestCase):
    def test_slow_approach_does_not_require_clearing_whole_junction(self):
        p, ego = planner(8., .8)
        car = junction_fixture.car(20., 10., 0., -3., 1)
        for now in np.arange(0., 1., 1./15.):
            result = p.evaluate([car], ego, PATH, 1., float(now))
            self.assertEqual(result['kind'], 'follow')
            self.assertAlmostEqual(result['v_mps'], 1.)
            self.assertEqual(p.state, 'APPROACH')
            self.assertFalse(p.junction_committed)

    def test_actual_crossing_before_line_brakes_to_a_stop(self):
        # A car crossing 4 m ahead before the line: braked to a standstill behind it.
        p, ego = planner(8., 2.)
        result = p.evaluate([junction_fixture.car(4., 2., 0., -3., 1)], ego, PATH, 8., 0.)
        self.assertEqual(result['kind'], 'follow', result)
        self.assertLess(result['v_mps'], .3)
        self.assertFalse(p.junction_committed)

    def test_real_leader_before_line_retains_headway_cap(self):
        p, ego = planner(20., 4.)
        result = p.evaluate([junction_fixture.car(10., 0., 1., 0., 1)], ego, PATH, 8., 0.)
        self.assertLessEqual(result['v_mps'], following_speed_cap(10. - 2.3 - 2.3175, 1., 4.) + 1e-9)
        self.assertLess(result['v_mps'], 4.)

    def test_future_leader_beyond_stop_line_does_not_cap_approach(self):
        p, ego = planner(8., 2.)
        result = p.evaluate([junction_fixture.car(20., 0., 1., 0., 1)], ego, PATH, 8., 0.)
        self.assertEqual(result['kind'], 'follow')
        self.assertEqual(result['v_mps'], 8.)

    def test_fast_guard_does_not_replace_mandatory_stop_with_earlier_stop(self):
        p, ego = planner(8., 2.)
        with patch('traffic_planner.FAST_TRAFFIC_GUARD', True):
            result = p.evaluate([junction_fixture.car(20., 10., 0., -10., 1)], ego, PATH, 8., 0.)
        self.assertEqual(result['kind'], 'follow', result)
        self.assertGreater(result['v_mps'], 0.)

    def test_late_crossing_still_blocks_entry(self):
        # 17.4 m from our path at 3 m/s: it may speed up (ring_entry resume model) and
        # reach our lane before our rear clears it with the 1 s margin.
        p, ego = planner(.6)
        result = p.evaluate([junction_fixture.car(10., 17.4, 0., -3., 1)], ego, PATH, 8.333, 0.)
        self.assertEqual(result['kind'], 'stop', result)
        self.assertTrue(p.risk_details)

    def test_sensor_staleness_and_unknown_motion_still_block(self):
        p, ego = planner(.6)
        obj = junction_fixture.car(6., 12., 0., 0., 5, valid=False)
        p.junction['unknown_ids'] = [5]
        self.assertEqual(p.evaluate([obj], ego, PATH, 8., 0.)['kind'], 'stop')
        self.assertEqual(p.evaluate([], ego, PATH, 8., .1, fresh=False)['kind'], 'stop')


class RoundaboutCommittedPaceTests(unittest.TestCase):
    def recorded(self, committed=True, held=False):
        scene = json.loads((Path(__file__).parent / 'test_data' /
                            'roundabout_merging_lead.json').read_text())
        p = GapPlanner()
        p.ACCEL_MPS2 = INTERSECTION_ACCEL_MPS2
        p.junction = dict(held=held, ready=True, unknown_ids=[], line_gap_m=None)
        p.junction_committed, p.state = committed, 'ENTER'
        predictions = [(np.asarray(row['xy']), np.asarray(row['headings']))
                       for row in scene['predictions']]
        p.map_predictor.predict = lambda obj, times: predictions
        return p, scene

    def test_recorded_merging_lead_can_hold_measured_speed_without_stopping(self):
        p, scene = self.recorded()
        # Runtime sets junction_committed; the separate legacy flag is never set.
        self.assertFalse(p.entry_committed)
        result = p.evaluate([tuple(scene['object'])], scene['ego'], scene['local_path'],
                            scene['requested_mps'], scene['source_wall'])
        self.assertEqual(result['kind'], 'follow', result)
        self.assertGreater(result['v_mps'], 0.)
        self.assertLessEqual(result['v_mps'], scene['ego']['speed_kmh'] / 3.6)
        self.assertEqual(p.risk_details, [])

    def test_recorded_conflict_before_entry_still_requires_stop(self):
        p, scene = self.recorded(False, False)
        result = p.evaluate([tuple(scene['object'])], scene['ego'], scene['local_path'],
                            scene['requested_mps'], scene['source_wall'])
        self.assertEqual(result['kind'], 'stop', result)

    def test_released_junction_rear_vehicle_uses_actual_commit_state(self):
        p, ego = planner(None, 1.)
        p.junction['held'] = False
        p.junction_committed, p.state = True, 'ENTER'
        result = p.evaluate([junction_fixture.car(-8., 0., 6., 0., 1)], ego, PATH, 8., 100.)
        self.assertNotEqual((result or {}).get('kind'), 'stop', result)

    def test_body_contact_and_stopped_leader_brake_to_standstill(self):
        # Committed: following only - a car right ahead brakes us to a standstill
        # behind it; that is following, not a stop rule (user rule 2026-10-10).
        for x, velocity in ((4.5, 5.), (6., 0.)):
            with self.subTest(x=x, velocity=velocity):
                p, ego = planner(None, 2.)
                p.junction['held'] = False
                p.junction_committed, p.state = True, 'ENTER'
                result = p.evaluate([junction_fixture.car(x, 0., velocity, 0., 1)],
                                    ego, PATH, 8., 100.)
                self.assertEqual(result['kind'], 'follow', result)
                self.assertEqual(result['v_mps'], 0.)
                self.assertTrue(p.junction_committed)

    def test_crossing_vehicle_brakes_and_stale_stream_does_not_stop_committed_entry(self):
        for fresh in (True, False):
            with self.subTest(fresh=fresh):
                p, ego = planner(None, 2.)
                p.junction['held'] = False
                p.junction_committed, p.state = True, 'ENTER'
                result = p.evaluate([junction_fixture.car(7., -5., 0., 5., 1)],
                                    ego, PATH, 8., 100., fresh=fresh)
                self.assertEqual(result['kind'], 'follow', result)
                self.assertLess(result['v_mps'], 8.)
                self.assertTrue(p.junction_committed)

    def test_live_timer_copy_uses_same_committed_pace(self):
        p, scene = self.recorded()
        ego, path = scene['ego'], np.asarray(scene['local_path'])
        plan = DrivingPlan(1, path, ego, ego, 99.9, .1, 100., scene['requested_mps'],
            40., False, dict(valid=True, pose_valid=True, steering_valid=True,
                            blocked=False), 'LATTICE')
        guard = LivePathSafety(SimpleNamespace(gap_planner=p))
        guard.junction_snapshot(plan)
        result = guard.traffic.evaluate([tuple(scene['object'])], ego, path,
                                        scene['requested_mps'], scene['source_wall'])
        self.assertEqual(result['kind'], 'follow', result)
        self.assertGreater(result['v_mps'], 0.)
        self.assertFalse(p.entry_committed)


class RoundaboutLiveStateTests(unittest.TestCase):
    def test_live_check_keeps_no_gap_state_of_its_own(self):
        p, ego = planner(.6)
        guard = LivePathSafety(SimpleNamespace(gap_planner=p))
        original = DrivingPlan(1, PATH, ego, ego, 99.9, .1, 100., 8., 40., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False), 'LATTICE')
        guard.junction_snapshot(original)
        self.assertIsNot(guard.traffic, p)
        self.assertEqual(guard.traffic.junction, p.junction)
        self.assertIsNot(guard.traffic.junction, p.junction)
        self.assertFalse(hasattr(guard.traffic, 'junction_clear_since'))

    def test_released_entry_publishes_full_cap_without_ramp(self):
        fixture = live_fixture.LivePathSafetyTests()
        runtime = fixture.runtime()
        pub = runtime.path_publisher
        ego = dict(EGO)
        original = DrivingPlan(1, PATH, ego, ego, 99.9, .1, 100., 8., 40., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False), 'LATTICE')
        pub._safety_provider = lambda *args: original
        pub._last_target_mps, pub._last_target_time = 0., 100.
        pub._safety_baseline = dict(world_path=PATH, ego=ego, mode='LATTICE', decision=original,
            target=8., created=100., gear='D', payload=dict(waypoints=[]),
            launch_accel_mps2=None)
        pub._published_plan = {}
        with patch('gui.time.monotonic', return_value=100.1):
            self.assertTrue(pub.recheck_active_path())
        self.assertAlmostEqual(pub._last_target_mps, 8.)


class RoundaboutPhysicalStopTests(unittest.TestCase):
    def test_delayed_commands_stop_close_to_line_without_crossing(self):
        for kmh in (20., 28.94, 40.):
            with self.subTest(kmh=kmh):
                result = stop_fixture.StopLineTests.delayed_approach(kmh)
                self.assertTrue(result['held'] and result['released'])
                self.assertGreater(result['minimum_gap'], .2)
                # At rest up to STOP_REST_EXTRA_M beyond the window is the full stop now
                # (092032Z lap 2: held at 1.12-1.16 m instead of creeping to 1.09 m first).
                self.assertLess(result['minimum_gap'], StopLineController.BUFFER_M +
                                STOP_WINDOW_EXTRA_M + STOP_REST_EXTRA_M)


if __name__ == '__main__':
    unittest.main()
