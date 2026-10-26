"""Released junction entry follows measured traffic without a rear forecast stop.

Small isolated planner cases specify the policy contract. They do not replay
the simulator or invent responses of NPCs in a recorded drive.
"""
import json
from pathlib import Path
import unittest

import numpy as np

from traffic_planner import GapPlanner, overlaps


class IntersectionCommittedFollowTests(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=3.6)

    def planner(self, committed=True, path=None):
        planner = GapPlanner()
        planner.entry_committed = committed
        planner.state = 'ENTER'
        world = self.path if path is None else path
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(world, axis=0), axis=1))]
        planner.merge_path, planner.merge_exit = (world.copy(), arc), 35.
        return planner

    def car(self, x, y=0., vx=5., vy=0., uid=1, valid=True, heading=0.):
        return (x, y, 4.9, 'lidar_car', 'npc', float(np.hypot(vx, vy)),
                4.9, 2.1, heading, vx, vy, uid, 0., None, valid)

    def check_positive(self, result):
        if result is not None:
            self.assertNotEqual(result['kind'], 'stop', result)
            self.assertGreater(result['v_mps'], 0., result)

    def test_rear_same_flow_future_overlap_does_not_restop_released_entry(self):
        # Constant-speed forecasts catch the ego even though the simulator's
        # following NPC may brake. Exemption is scoped to committed entry and
        # an actually separate current body, independent of PP 180deg axes.
        for heading in (0., 180.):
            with self.subTest(heading=heading):
                planner = self.planner()
                rear = self.car(-8., vx=6., heading=heading)
                result = planner.evaluate([rear], self.ego, self.path, 5.55, 10.)
                self.check_positive(result)

    def test_rear_actual_body_overlap_is_not_exempt(self):
        planner = self.planner()
        rear = self.car(-4.5, vx=6.)
        self.assertTrue(overlaps(np.array([[0., 0.]]), np.array([0.]),
                                 np.array([rear[:2]]), np.array([0.]), rear[6], rear[7], margin=0.)[0, 0])
        result = planner.evaluate([rear], self.ego, self.path, 5.55, 10.)
        self.assertIsNotNone(result)
        self.assertEqual(result['kind'], 'stop', result)

    def test_unreleased_crossing_still_stops(self):
        planner = self.planner(committed=False)
        ego = dict(self.ego, speed_kmh=7.2)
        cross = self.car(10., -10., vx=0., vy=5., heading=90.)
        result = planner.evaluate([cross], ego, self.path, 5.55, 10.)
        self.assertIsNotNone(result)
        self.assertEqual(result['kind'], 'stop', result)

    def test_rear_crossing_is_not_a_following_vehicle(self):
        planner = self.planner()
        crossing = self.car(-6., -4., vx=4., vy=4., heading=45.)
        result = planner.evaluate([crossing], self.ego, self.path, 5.55, 10.)
        self.assertIsNotNone(result)
        self.assertEqual(result['kind'], 'stop', result)

    def test_stationary_front_vehicle_retains_braking_stop(self):
        planner = self.planner()
        ego = dict(self.ego, speed_kmh=10.8)
        stopped = self.car(8., vx=0., valid=False)
        result = planner.evaluate([stopped], ego, self.path, 5.55, 10.)
        self.assertIsNotNone(result)
        self.assertEqual(result['kind'], 'stop', result)

    def test_slow_lead_reduces_speed_with_checked_headway(self):
        planner = self.planner()
        ego = dict(self.ego, speed_kmh=10.)
        slow = self.car(12., vx=1.)
        result = planner.evaluate([slow], ego, self.path, 8., 10.)
        self.assertIsNotNone(result)
        self.assertNotEqual(result['kind'], 'stop', result)
        self.assertGreater(result['v_mps'], 0., result)
        self.assertLess(result['v_mps'], 8., result)

    def test_stale_stream_stops_even_after_release(self):
        result = self.planner().evaluate([], self.ego, self.path, 5.55, 10., fresh=False)
        self.assertEqual(result['kind'], 'stop')
        self.assertEqual(result['reason'], 'objects stale')

    def test_recorded_lead_margin_only_contact_allows_positive_follow(self):
        root = Path(__file__).resolve().parents[2]
        filename = root/'data/autodrive/validation/intersection_entry_20261003/intersection_committed_cases_063204.json'
        scene = json.loads(filename.read_text())[0]
        path, car = np.asarray(scene['local_path']), tuple(scene['object'])
        self.assertGreater(scene['physical_sat_separation_m'], .1)
        self.assertLess(scene['physical_sat_separation_m'], .4)
        yaw = np.radians([float(car[8])])
        self.assertFalse(overlaps(np.array([[0., 0.]]), np.array([0.]),
                                  np.array([car[:2]]), yaw, car[6], car[7], margin=0.)[0, 0])
        self.assertTrue(overlaps(np.array([[0., 0.]]), np.array([0.]),
                                 np.array([car[:2]]), yaw, car[6], car[7], margin=.4)[0, 0])
        result = self.planner(path=path).evaluate([car], scene['ego'], path, 5.55, 10.)
        self.check_positive(result)


    def test_stopped_car_yielding_beside_us_does_not_deadlock(self):
        # Live run 2026-10-03: an NPC stopped 2 m behind-left to yield to us,
        # and its 0.4 m shell kept us waiting for it for 40 s.
        for committed in (False, True):
            with self.subTest(committed=committed):
                planner = self.planner(committed=committed)
                ego = dict(self.ego, speed_kmh=0.)
                waiting = self.car(-2.3, 2.3, vx=0., vy=0., heading=150.)
                result = planner.evaluate([waiting], ego, self.path, 5.55, 10.)
                self.check_positive(result)

if __name__ == '__main__':
    unittest.main()
