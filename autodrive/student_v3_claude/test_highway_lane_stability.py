"""Lane choice must be purposeful: no start/cancel loops between adjacent lanes.

Run follow-20261009T191227Z, t=62.0-64.4 s at 61 km/h before the common
entrance: the current lane (408) predicted a converging car's footprint at
4.3 s. Any such conflict forced a change into lane 434 (14 s forecast 45 km/h
against 81 km/h); the next frame the 3 s current-lane recheck saw no conflict
and a 0.1 km/h better 3 s average, and cancelled. 13 target flips in 2.4 s.
"""
import unittest
from unittest.mock import patch

import numpy as np

from highway_planner import HighwayPlanner, TRAFFIC_CONTROL_HORIZON_S
from highway_three_lane import HighwayLane


class LaneStabilityTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        self.config = dict(from_idx=0, to_idx=1800, kmh=120.,
            lane_change_policy="free_three_lane", lane_change_goal=0, end_link="exit")
        self.planner = HighwayPlanner(self.config, self.route)
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=61., stamp=10.)
        self.lanes = [HighwayLane(str(k), k, self.route+np.array([0., k*3.5]),
                                  (str(k), "exit"), 600.) for k in range(3)]

    @staticmethod
    def car(x, y, speed, uid=1, valid=True):
        return (x, y, 4.9, "lidar_car", "npc", speed, 4.9, 2.1,
                0., speed, 0., uid, 0., None, valid, 0.)

    def decide(self, objects=(), stamp=10., speed_kmh=None):
        ego = dict(self.ego, stamp=stamp, speed_kmh=speed_kmh or self.ego['speed_kmh'])
        # Native roads of observed cars also extend behind the ego.
        behind = np.column_stack((np.arange(-200., 0., .5), np.zeros(400)))
        motion = [HighwayLane(l.link_id, -1, np.vstack((behind+l.centre[0], l.centre)),
                              l.chain, 0.) for l in self.lanes]
        return self.planner.decide(0, self.route[:640], ego, objects, {},
            lambda p, h: h, lambda p, h: h, perception_ready=True, lane_options=self.lanes,
            motion_options=motion)

    def converging_conflict(self, collision_when):
        """Recorded forecasts: the current lane sees the converging car only on
        its full horizon unless the conflict is inside the 3 s control horizon;
        the escape lane is slow; a started change slows behind its leader."""
        policy = self.planner.three_lane_policy
        original = policy.evaluate
        def evaluate(*args, **kwargs):
            result, check = original(*args, **kwargs)
            if result is None:
                return result, check
            horizon = kwargs.get('check_horizon_s')
            changing = kwargs.get('changing', False)
            saved_change = (not changing and horizon == TRAFFIC_CONTROL_HORIZON_S and
                            kwargs.get('continuing_change', True))
            if changing or saved_change:
                check['average_kmh'], check['control_average_kmh'] = 45., 52.
                result['goal'] = min(result['goal'], 45./3.6)
                result['free_goal'] = min(result['free_goal'], 45./3.6)
            elif horizon is None or collision_when <= horizon:
                result['risk'] = dict(when=collision_when, collision_when=collision_when,
                                      name='lidar_car', object_id=500026, rear_approach=False,
                                      reason='predicted lane-change gap occupied')
            return result, check
        return patch.object(policy, 'evaluate', side_effect=evaluate)

    def test_recorded_far_converging_conflict_does_not_force_a_slower_lane(self):
        self.decide()
        targets = []
        with self.converging_conflict(4.3):
            for k in range(1, 16):
                targets.append(self.decide(stamp=10.+k/15.)['status']['target_lane'])
        self.assertEqual(targets, ['0']*15)
        self.assertIsNone(self.planner.saved_world)

    def test_near_term_conflict_escapes_at_once_and_is_not_cancelled(self):
        self.decide()
        with self.converging_conflict(2.):
            first = self.decide(stamp=10.1)
            self.assertEqual(first['status']['target_lane'], '1')
            self.assertEqual(first['status']['reason'],
                             'highway checked escape: near-term conflict or static hit')
            saved = self.planner.saved_world.copy()
            for k in range(2, 8):
                result = self.decide(stamp=10.+k/15.)
                self.assertEqual(result['status']['target_lane'], '1')
            np.testing.assert_array_equal(self.planner.saved_world, saved)

    def committed_change(self):
        lead = [self.car(70., 0., 15.)]
        self.decide(lead, stamp=9.5, speed_kmh=90.)
        result = self.decide(lead, stamp=10., speed_kmh=90.)
        self.assertEqual(result['status']['target_lane'], '1')
        return lead, self.planner.saved_world.copy()

    def margins(self, hold_gain_kmh, control_gain_kmh):
        """The committed change slows; the current lane is ahead by the given margins."""
        policy = self.planner.three_lane_policy
        original = policy.evaluate
        def evaluate(*args, **kwargs):
            result, check = original(*args, **kwargs)
            if result is None:
                return result, check
            horizon = kwargs.get('check_horizon_s')
            if horizon == TRAFFIC_CONTROL_HORIZON_S and kwargs.get('continuing_change', True):
                # 'selected path speed limited': a persistent (non-curvature) limit.
                result['goal'] = result['free_goal'] = 60./3.6
                check['average_kmh'], check['control_average_kmh'] = 80., 80.
            elif horizon == TRAFFIC_CONTROL_HORIZON_S:
                check['average_kmh'] = 80.+hold_gain_kmh
                check['control_average_kmh'] = 80.+control_gain_kmh
            return result, check
        return patch.object(policy, 'evaluate', side_effect=evaluate)

    def test_committed_change_survives_a_small_current_lane_edge(self):
        lead, saved = self.committed_change()
        with self.margins(hold_gain_kmh=1., control_gain_kmh=.5):
            result = self.decide(lead, stamp=10.1, speed_kmh=90.)
        self.assertIn('selected path speed limited', result['status']['continuation_replan_reasons'])
        self.assertEqual(result['status']['target_lane'], '1')
        np.testing.assert_array_equal(self.planner.saved_world, saved)
        self.assertIsNone(result['status']['current_lane_recheck']['cancel_basis'])

    def test_committed_change_cancels_for_the_start_margin(self):
        lead, _ = self.committed_change()
        with self.margins(hold_gain_kmh=3.5, control_gain_kmh=3.5):
            result = self.decide(lead, stamp=10.1, speed_kmh=90.)
        self.assertEqual(result['status']['target_lane'], '0')
        self.assertIsNone(self.planner.saved_world)
        self.assertEqual(result['status']['current_lane_recheck']['cancel_basis'], 'progress')

    def test_progress_cancel_never_returns_in_front_of_a_closing_rear_car(self):
        import planning.highway.policy as policy_module
        lead, saved = self.committed_change()
        original = policy_module.timed_gap
        def rear_conflict(*args, **kwargs):
            # The current-lane recheck skips closing rear traffic; the return
            # check sees a rear car reaching us in 1.8 s.
            if kwargs.get('lane_change') is False and 'ignore_rear_approach' not in kwargs:
                return dict(when=1.8, collision_when=1.8, name='lidar_car', object_id=9,
                            rear_approach=True, reason='predicted lane-change gap occupied')
            return original(*args, **kwargs)
        with self.margins(hold_gain_kmh=3.5, control_gain_kmh=3.5), \
                patch.object(policy_module, 'timed_gap', side_effect=rear_conflict):
            result = self.decide(lead, stamp=10.1, speed_kmh=90.)
        self.assertEqual(result['status']['target_lane'], '1')
        np.testing.assert_array_equal(self.planner.saved_world, saved)
        self.assertIsNone(result['status']['current_lane_recheck']['cancel_basis'])

    def test_statically_worse_current_lane_is_never_the_fallback(self):
        # virtual_sim highway_adjacent_traffic: the second leg of an escape past
        # a stopped car hit it 40 m ahead; falling back to the current lane
        # (stopped car 27 m ahead) ended in a 15 km/h collision.
        lead, saved = self.committed_change()
        policy = self.planner.three_lane_policy
        original = policy.evaluate
        def evaluate(*args, **kwargs):
            result, check = original(*args, **kwargs)
            if result is not None and kwargs.get('check_horizon_s') == TRAFFIC_CONTROL_HORIZON_S:
                saved_change = kwargs.get('continuing_change', True)
                result['static_m'] = 40. if saved_change else 27.
                check['average_kmh'] = 40. if saved_change else 41.
                check['control_average_kmh'] = 60. if saved_change else 70.
            return result, check
        with patch.object(policy, 'evaluate', side_effect=evaluate):
            result = self.decide(lead, stamp=10.1, speed_kmh=90.)
        self.assertIn('static stopping reserve unavailable', result['status']['continuation_replan_reasons'])
        self.assertEqual(result['status']['target_lane'], '1')
        np.testing.assert_array_equal(self.planner.saved_world, saved)

    def test_recorded_unknown_speed_car_blocks_the_target_lane(self):
        # Run follow-20261009T185440Z t=27.8 s: at 80 km/h the car changed into
        # lane 409 with a LiDAR car of unknown speed 44 m ahead in it, then
        # braked from 85 to 29 km/h once its 24 km/h speed was measured.
        unknown = self.car(44., 3.5, 0., uid=500018, valid=False)
        lead = self.car(70., 0., 15., uid=7)
        for stamp in (10., 10.6):
            result = self.decide([lead, unknown], stamp=stamp, speed_kmh=80.)
        self.assertEqual(result['status']['target_lane'], '0')
        reasons = {c['reason'] for c in result['status']['candidate_checks'] if c['lanes'] == ['1']}
        self.assertEqual(reasons, {'target-lane standing or unknown-speed body inside stopping reserve'})

    def test_target_lane_body_beyond_the_stopping_reserve_does_not_block(self):
        unknown = self.car(130., 3.5, 0., uid=500018, valid=False)
        lead = self.car(70., 0., 15., uid=7)
        for stamp in (10., 10.6):
            result = self.decide([lead, unknown], stamp=stamp, speed_kmh=80.)
        self.assertEqual(result['status']['target_lane'], '1')

    def test_single_frame_progress_flicker_does_not_start_a_change(self):
        self.decide(stamp=10.)
        policy = self.planner.three_lane_policy
        original = policy.evaluate
        flicker = {'on': True}
        def evaluate(*args, **kwargs):
            result, check = original(*args, **kwargs)
            if result is not None and not kwargs.get('changing') and flicker['on']:
                # A one-frame false far static hit: the current-lane forecast
                # brakes (old code: "limited", so the change started at once).
                check['average_kmh'] -= 30.
                result['trajectory'].speeds[-1] = 10.
            return result, check
        targets = []
        with patch.object(policy, 'evaluate', side_effect=evaluate):
            for k in range(1, 12):
                flicker['on'] = k % 2 == 1
                targets.append(self.decide(stamp=10.+k/15.)['status']['target_lane'])
        self.assertEqual(set(targets), {'0'})


if __name__ == '__main__':
    unittest.main()
