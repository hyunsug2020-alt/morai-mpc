"""Keep the first checked highway entry while rechecking its actual safety."""
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from gui import HdMapLanes, smooth_control_route
from highway_planner import HighwayPlanner, local_points
from highway_three_lane import HighwayLane, HighwayLaneMap, ThreeLanePolicy
from planning.highway.body_geometry import highway_raw_clearance


ROOT = Path(__file__).parent


class EntryCommitTests(unittest.TestCase):
    def fixture(self, enabled=True):
        self.world = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        self.entry = dict(from_idx=0, to_idx=80, source_link='0', target_link='1',
            policy='first_safe_adjacent', no_initial_deceleration=True,
            commit_until_complete=enabled, transition_s=3.6, transition_min_m=45.)
        self.config = dict(from_idx=0, to_idx=1800, kmh=120., end_link='exit',
            lane_change_policy='free_three_lane', lane_selection_model='mobil',
            lane_change_goal=0, lane_gain_kmh=2., lane_cancel_gain_kmh=3.,
            lane_gain_hold_s=.3, comfort_decel_mps2=2.5, comfort_jerk_mps3=4.,
            entry_lane_change=self.entry,
            rule_speed_limit=dict(until_idx=2, kmh=58.))
        self.planner = HighwayPlanner(self.config, self.world)
        self.pose = dict(x=0., y=0., heading_deg=0., speed_kmh=55.8, stamp=10.)
        return self.step()

    @staticmethod
    def car(x, y, speed, uid=500001):
        return (x, y, 4.9, 'lidar_car', 'npc', speed, 4.9, 2.1,
                0., speed, 0., uid, 0., None, True, 0.)

    def options(self, ego, count=4):
        lanes, motion = [], []
        full = np.column_stack((np.arange(-200., 1000., .5), np.zeros(2400)))
        for rank in range(count):
            number = 5-rank
            centre = local_points(self.world+[0., rank*3.5], ego)
            lane = HighwayLane(str(rank), rank, centre, (str(rank), 'exit'),
                800.-ego['x'], native_lane_number=number)
            lanes.append(lane)
            motion.append(HighwayLane(lane.link_id, -1,
                local_points(full+[0., rank*3.5], ego), lane.chain, 0.))
        return lanes, motion

    def step(self, *, index=0, ego=None, cars=(), ready=True, rear=None,
             map_clear=None, static_clear=None):
        ego = self.pose if ego is None else ego
        lanes, motion = self.options(ego)
        return self.planner.decide(index, local_points(self.world[:640], ego), ego,
            cars, {}, map_clear or (lambda p, h: h), static_clear or (lambda p, h: h),
            perception_ready=ready, rear_unclassified=rear,
            lane_options=lanes, motion_options=motion)

    def progressed_pose(self, distance=6.2, stamp=10.4):
        path = self.planner.saved_world
        y = float(np.interp(distance, path[:, 0], path[:, 1]))
        point = int(np.clip(np.searchsorted(path[:, 0], distance), 1, len(path)-1))
        tangent = path[point]-path[point-1]
        return dict(self.pose, x=distance, y=y,
            heading_deg=math.degrees(math.atan2(tangent[1], tangent[0])), stamp=stamp)

    def assert_owner(self, result):
        status = result['status']
        self.assertTrue(status['entry_commit_active'])
        self.assertEqual(status['target_lane'], '1')
        self.assertEqual(status['planned_lane_sequence'], ['1'])
        self.assertIsNotNone(self.planner.saved_world)
        self.assertFalse(result['emergency'])

    def assert_cancelled(self, result):
        self.assertFalse(result['status']['entry_commit_active'])
        self.assertIsNotNone(result['status']['entry_commit_cancel_reason'])
        self.assertIsNone(self.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '0')

    def test_empty_entry_immediately_owns_one_short_checked_leg(self):
        result = self.fixture()
        self.assert_owner(result)
        self.assertTrue(result['status']['immediate_entry_lane_change'])
        self.assertLessEqual(result['status']['selected_transition_m'], 55.8+1e-6)
        self.assertGreaterEqual(result['target_mps'], self.pose['speed_kmh']/3.6-.05)
        self.assertTrue(any(c.get('accepted') and c['lanes'] == ['1']
                            for c in result['status']['candidate_checks']))

    def test_expired_rule_cap_is_refreshed_on_the_same_checked_world_path(self):
        self.fixture()
        saved = self.planner.saved_world.copy()
        result = self.step(index=3, ego=self.progressed_pose())
        self.assert_owner(result)
        self.assertTrue(result['status']['entry_goal_refreshed'])
        self.assertGreater(result['target_mps']*3.6, 58.+1.)
        np.testing.assert_array_equal(self.planner.saved_world, saved)

    def test_utility_pressure_and_far_lanes_do_not_replace_the_first_leg(self):
        self.fixture()
        saved = self.planner.saved_world.copy()
        evaluate = ThreeLanePolicy.evaluate

        def favorable_utility(policy, *args, **kwargs):
            result, check = evaluate(policy, *args, **kwargs)
            # Alter only the progress estimate after the real physical checks,
            # reproducing the stay/far-lane utility pressure of the live run.
            if result is not None and (not kwargs.get('changing') or
                                      len(kwargs.get('destinations', ())) > 1):
                check['average_kmh'] += 40.
            return result, check

        with patch.object(ThreeLanePolicy, 'evaluate', favorable_utility):
            result = self.step(index=3, ego=self.progressed_pose())
        self.assert_owner(result)
        np.testing.assert_array_equal(self.planner.saved_world, saved)

    def test_a_new_fast_rear_car_cancels_the_saved_entry(self):
        self.fixture()
        ego = self.progressed_pose()
        result = self.step(index=3, ego=ego,
            cars=[self.car(ego['x']-8., 3.5, 30.)])
        self.assert_cancelled(result)
        self.assertIn('rear', result['status']['entry_commit_cancel_reason'])

    def test_unclassified_target_lane_rear_returns_still_cancel(self):
        self.fixture()
        ego = self.progressed_pose()
        rear = local_points(np.array([[ego['x']-10., 3.5]]), ego)
        result = self.step(index=3, ego=ego, rear=rear)
        self.assert_cancelled(result)
        self.assertIn('rear', result['status']['entry_commit_cancel_reason'])

    def test_new_map_and_raw_static_hits_cannot_be_hidden_by_the_owner(self):
        for kind in ('map', 'raw'):
            with self.subTest(kind=kind):
                self.fixture()
                ego = self.progressed_pose()
                if kind == 'map':
                    # A newly closed target corridor; the current lane remains open.
                    clearance = lambda p, h: 4. if np.max(p[:, 1]) > 1. else h
                    result = self.step(index=3, ego=ego, map_clear=clearance)
                else:
                    # An actual raw point on the target lane, checked with the
                    # same swept physical-body function as the runtime.
                    points = local_points(np.array([[40., 3.5]]), ego)
                    clearance = lambda p, h: highway_raw_clearance(p, points, h)
                    result = self.step(index=3, ego=ego, static_clear=clearance)
                self.assert_cancelled(result)

    def test_vehicle_source_loss_cancels_entry_ownership(self):
        self.fixture()
        result = self.step(index=3, ego=self.progressed_pose(), ready=False)
        self.assert_cancelled(result)

    def test_actual_first_leg_completion_releases_the_special_owner(self):
        first = self.fixture()
        distance = first['status']['selected_transition_m']+5.
        ego = dict(self.pose, x=distance, y=3.5, heading_deg=0., stamp=14.)
        result = self.step(index=80, ego=ego)
        self.assertFalse(result['status']['entry_commit_active'])
        self.assertIsNone(self.planner.saved_world)
        self.assertEqual(result['status']['lane_changes'], 1)
        self.assertEqual(result['status']['current_lane'], '1')

    def test_disabled_policy_preserves_the_legacy_progress_cancel(self):
        first = self.fixture(enabled=False)
        self.assertFalse(first['status']['entry_commit_active'])
        self.assertEqual(first['status']['selected_transition_m'], 200.)
        result = self.step(index=3, ego=self.progressed_pose())
        self.assertIsNone(self.planner.saved_world)
        self.assertEqual(result['status']['current_lane_recheck']['cancel_basis'], 'progress')

    def test_recorded_first_highway_marker_uses_a_short_safe_single_entry(self):
        world = np.array([[p['x'], p['y']] for p in
                          json.loads((ROOT/'global_path.json').read_text())['waypoints']])
        smooth = smooth_control_route(world)
        config = json.loads((ROOT/'highway_mission.json').read_text())
        lanes = HdMapLanes(ROOT/'link_set.json')
        lanes.set_route(world)
        # Native27's first safe 411->409 frame, before any utility cancellation.
        ego = dict(x=62.32, y=255.88, heading_deg=-89.4,
                   speed_kmh=55.8, stamp=100.)
        index = int(np.argmin(np.linalg.norm(world-[ego['x'], ego['y']], axis=1)))
        local = local_points(smooth[index:index+640], ego)
        native = HighwayLaneMap(lanes, config)
        options = native.candidates(ego, local)
        corridor = native.corridor(ego, local, options)
        planner = HighwayPlanner(config, world)
        result = planner.decide(index, local, ego, (), {}, corridor.clear_distance,
            lambda p, h: h, lane_options=options)
        status = result['status']
        self.assertTrue(status['entry_commit_active'])
        self.assertEqual(status['planned_lane_sequence'], ['A2256W000409'])
        self.assertLess(status['selected_transition_m'], 100.)
        self.assertFalse(result['emergency'])


if __name__ == '__main__':
    unittest.main()
