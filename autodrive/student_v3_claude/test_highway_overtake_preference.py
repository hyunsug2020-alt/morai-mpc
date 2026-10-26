"""Native highway lane preferences retain admission and safe fallback behavior."""
import unittest
from unittest.mock import patch

import numpy as np

from highway_planner import HighwayPlanner, local_points
from highway_three_lane import HighwayLane, ThreeLanePolicy
import test_highway_three_lane as surveyed_fixture


class OvertakePreferenceTests(unittest.TestCase):
    def fixture(self, model='forecast_progress', current=3, enabled=True):
        route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        config = dict(from_idx=0, to_idx=1800, kmh=120., end_link='exit',
            lane_change_policy='free_three_lane', lane_change_goal=0,
            comfort_decel_mps2=2.5, comfort_jerk_mps3=4., lane_gain_kmh=2.,
            lane_gain_hold_s=.3, lane_cancel_gain_kmh=3., lane_selection_model=model)
        if enabled:
            config.update(overtake_left_first=True, overtake_lane_targets={'3':2, '2':3})
        planner = HighwayPlanner(config, route)
        y = (4-current)*3.5
        ego = dict(x=0., y=y, heading_deg=0., speed_kmh=60., stamp=10.)
        lanes = [HighwayLane('lane_'+str(number), rank, route+[0., rank*3.5-y],
            ('lane_'+str(number), 'exit'), 800., native_lane_number=number)
            for rank, number in enumerate((4, 3, 2))]
        behind = np.column_stack((np.arange(-200., 0., .5), np.zeros(400)))
        motion = [HighwayLane(l.link_id, -1, np.vstack((behind+l.centre[0], l.centre)),
            l.chain, 0., native_lane_number=l.native_lane_number) for l in lanes]
        def decide(stamp, cars):
            return planner.decide(0, local_points(route[:640], ego), dict(ego, stamp=stamp),
                cars, {}, lambda p,h:h, lambda p,h:h, perception_ready=True,
                lane_options=lanes, motion_options=motion)
        return planner, ego, lanes, decide

    @staticmethod
    def car(x, y, speed, uid=1):
        return (x, y, 4.9, 'lidar_car', 'npc', speed, 4.9, 2.1,
                0., speed, 0., uid, 0., None, True, 0.)

    def settled(self, decide, cars):
        decide(9.6, cars)
        return decide(10., cars)

    def assert_target(self, result, number, preferred=True):
        status = result['status']
        self.assertEqual(status['target_lane'], 'lane_'+str(number))
        self.assertEqual(status['target_lane_number'], number)
        self.assertEqual(status['overtake_preference_applied'], preferred)
        self.assertFalse(result['emergency'])

    def test_native_middle_three_prefers_left_two_in_both_selection_models(self):
        for model in ('forecast_progress', 'mobil'):
            _, ego, _, decide = self.fixture(model)
            result = self.settled(decide, [self.car(100., ego['y'], 8.)])
            with self.subTest(model=model):
                self.assert_target(result, 2)
                self.assertEqual(result['status']['current_lane_number'], 3)
                self.assertEqual(result['status']['preferred_overtake_lane_number'], 2)
                accepted = next(c for c in result['status']['candidate_checks'] if c.get('accepted'))
                if model == 'mobil':
                    self.assertTrue(accepted['mobil']['safe'])
                    self.assertGreater(accepted['mobil']['incentive_mps2'], .2)

    def test_fast_rear_in_preferred_lane_preserves_safe_right_fallback(self):
        for model in ('forecast_progress', 'mobil'):
            _, ego, _, decide = self.fixture(model)
            cars = [self.car(100., ego['y'], 8.), self.car(-10., 7., 30., 2)]
            result = self.settled(decide, cars)
            with self.subTest(model=model):
                self.assert_target(result, 4, preferred=False)
                self.assertTrue(any(c.get('lanes') == ['lane_2'] and not c.get('accepted')
                                    for c in result['status']['candidate_checks']))

    def test_no_gain_in_preferred_lane_does_not_hide_the_other_pass(self):
        original = ThreeLanePolicy.evaluate
        baseline = [None]
        def measured_no_gain(policy, *args, **kwargs):
            result, check = original(policy, *args, **kwargs)
            if not kwargs.get('changing') and result is not None:
                baseline[0] = check['average_kmh']
            destinations = kwargs.get('destinations', ())
            if result is not None and len(destinations) == 1 and destinations[0].native_lane_number == 2:
                check['average_kmh'] = baseline[0]
            return result, check
        for model in ('forecast_progress', 'mobil'):
            _, ego, _, decide = self.fixture(model)
            with self.subTest(model=model), patch.object(ThreeLanePolicy, 'evaluate', measured_no_gain):
                result = self.settled(decide, [self.car(100., ego['y'], 8.)])
                self.assert_target(result, 4, preferred=False)

    def test_empty_road_keeps_each_native_lane(self):
        for model in ('forecast_progress', 'mobil'):
            for current in (4, 3, 2):
                planner, _, _, decide = self.fixture(model, current)
                result = self.settled(decide, [])
                with self.subTest(model=model, current=current):
                    self.assert_target(result, current, preferred=False)
                    self.assertIsNone(planner.saved_world)

    def test_native_two_prefers_a_single_adjacent_return_to_three(self):
        for model in ('forecast_progress', 'mobil'):
            _, ego, _, decide = self.fixture(model, current=2)
            result = self.settled(decide, [self.car(100., ego['y'], 8.)])
            with self.subTest(model=model):
                self.assert_target(result, 3)
                self.assertEqual(result['status']['planned_lane_sequence'], ['lane_3'])

    def test_native_four_uses_the_adjacent_left_lane_without_an_explicit_pair(self):
        for model in ('forecast_progress', 'mobil'):
            _, ego, _, decide = self.fixture(model, current=4)
            result = self.settled(decide, [self.car(100., ego['y'], 8.)])
            with self.subTest(model=model):
                self.assert_target(result, 3)
                self.assertEqual(result['status']['planned_lane_sequence'], ['lane_3'])

    def test_configuration_without_preferences_keeps_legacy_selection(self):
        for model in ('forecast_progress', 'mobil'):
            _, ego, _, decide = self.fixture(model, enabled=False)
            result = self.settled(decide, [self.car(100., ego['y'], 8.)])
            with self.subTest(model=model):
                self.assert_target(result, 4, preferred=False)
                self.assertIsNone(result['status']['preferred_overtake_lane'])


class NativeLaneNumberTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        surveyed_fixture.SurveyedHighwayTests.setUpClass()

    def frame(self):
        return surveyed_fixture.SurveyedHighwayTests().frame(2800)

    def test_surveyed_map_preserves_native_numbers_and_physical_ranks(self):
        _, _, _, lanes = self.frame()
        self.assertEqual([(l.link_id, l.rank, l.native_lane_number) for l in lanes],
            [('A2256W000408', 0, 4), ('A2256W000434', 1, 3), ('A2256W000435', 2, 2)])

    def test_link_boundary_reuse_preserves_native_lane_identity(self):
        ego, local, native, observed = self.frame()
        with patch.object(native, 'chain', return_value=None):
            retained = native.candidates(dict(ego, stamp=ego['stamp']+.1), local)
        self.assertEqual([(l.link_id, l.rank, l.native_lane_number) for l in retained],
                         [(l.link_id, l.rank, l.native_lane_number) for l in observed])
        self.assertTrue(native.geometry_reused)

    def test_blocked_native_lane_is_removed_without_renumbering_its_neighbours(self):
        ego, local, native, _ = self.frame()
        with patch.object(native, 'blocked', side_effect=lambda link:link == 'A2256W000408'):
            lanes = native.candidates(dict(ego, stamp=ego['stamp']+.1), local)
        self.assertEqual([(l.rank, l.native_lane_number) for l in lanes], [(1, 3), (2, 2)])


if __name__ == '__main__':
    unittest.main()
