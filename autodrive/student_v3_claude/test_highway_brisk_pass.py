"""Prompt adjacent passing keeps the ordinary admission and continuity checks."""
import unittest

import numpy as np

from highway_planner import HighwayPlanner
from planning.highway.path_continuity import join_reference
import test_highway_three_lane as lane_fixtures
from test_highway_path_continuity import curve, follower_curvature


class BriskHighwayTests(unittest.TestCase):
    def fixture(self, brisk=True):
        fixture = lane_fixtures.ThreeLaneTests()
        fixture.setUp()
        fixture.config.update(brisk_lane_changes=brisk, lane_gain_hold_s=.15,
            lane_selection_model='mobil', kmh=120., comfort_decel_mps2=2.5,
            comfort_jerk_mps3=4.)
        fixture.planner = HighwayPlanner(fixture.config, fixture.route)
        return fixture

    def test_clear_adjacent_pass_finishes_sooner_without_a_lower_speed_goal(self):
        results = []
        for brisk in (False, True):
            fixture = self.fixture(brisk)
            result = fixture.settle([fixture.car(65., 0., 12.)])
            self.assertEqual(result['status']['target_lane'], '1')
            self.assertFalse(result['emergency'])
            results.append(result)
        old, new = results
        self.assertLess(new['status']['selected_transition_m'], old['status']['selected_transition_m'])
        self.assertGreaterEqual(new['target_mps'], old['target_mps']-.5/3.6)
        self.assertLessEqual(new['target_mps']*3.6, 120.+1e-6)
        self.assertTrue(any(c.get('accepted') and c.get('command_gap_checks', 0) > 0
                            for c in new['status']['candidate_checks']))

    def test_two_fresh_safe_observations_start_the_pass_after_150_ms(self):
        fixture = self.fixture()
        cars = [fixture.car(65., 0., 12.)]
        fixture.decide(cars)
        self.assertIsNone(fixture.planner.saved_world)
        fixture.decide(cars)
        self.assertIsNone(fixture.planner.saved_world)
        result = fixture.decide(cars, ego=dict(fixture.ego, stamp=10.16))
        self.assertIsNotNone(fixture.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '1')

    def test_fast_rear_unknown_returns_and_stale_perception_still_block_the_pass(self):
        for reason in ('rear_car', 'raw_returns', 'stale'):
            fixture = self.fixture()
            cars = [fixture.car(65., 0., 12.)]
            rear = None
            if reason == 'rear_car':
                cars.append(fixture.car(-8., 3.5, 35., uid=2))
            elif reason == 'raw_returns':
                rear = np.array([[-8., 3.5]])
            result = fixture.settle(cars, rear=rear) if reason != 'stale' else fixture.decide(cars, ready=False)
            with self.subTest(reason=reason):
                self.assertIsNone(fixture.planner.saved_world)
                self.assertEqual(result['status']['target_lane'], '0')

    def test_empty_current_lane_does_not_create_a_discretionary_change(self):
        fixture = self.fixture()
        result = fixture.settle()
        self.assertIsNone(fixture.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '0')

    def test_straight_reference_keeps_response_prefix_and_reaches_the_lane_sooner(self):
        old = curve()
        candidate = curve()+[0., 3.5]
        standard, before = join_reference(candidate, old, 16.)
        brisk, after = join_reference(candidate, old, 16., brisk=True)
        self.assertEqual(before['prefix_m'], after['prefix_m'])
        self.assertLess(after['blend_m'], before['blend_m'])
        at = np.searchsorted(brisk[:, 0], 65.)
        self.assertGreater(brisk[at, 1], standard[at, 1]+.5)
        prefix = brisk[:, 0] < after['prefix_m']-.1
        self.assertLess(np.max(np.abs(brisk[prefix, 1])), 1e-6)
        self.assertLess(np.max(np.abs(follower_curvature(brisk)))*16.**2, 3.5)

    def test_curved_reference_keeps_the_existing_turn_reversal_reserve(self):
        for sign in (-1., 1.):
            reference = curve(sign*.004)
            candidate = curve(-sign*.004)
            standard, _ = join_reference(candidate, reference, 28.)
            brisk, _ = join_reference(candidate, reference, 28., brisk=True)
            np.testing.assert_array_equal(brisk, standard)


if __name__ == '__main__':
    unittest.main()
