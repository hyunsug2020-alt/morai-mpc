"""Three-lane overtaking, native merge geometry and gentle following regressions."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

from gui import HdMapLanes, smooth_control_route
from highway_planner import HighwayPlanner, local_points, world_points, lane_change_path
from highway_three_lane import HighwayLane, HighwayLaneMap, ThreeLanePolicy, arc


HERE = Path(__file__).parent


class ThreeLaneTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        self.config = dict(from_idx=0, to_idx=1800, kmh=90.,
            lane_change_policy="free_three_lane", lane_change_goal=0, end_link="exit")
        self.planner = HighwayPlanner(self.config, self.route)
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=10.)
        self.lanes = [HighwayLane(str(k), k, self.route+np.array([0., k*3.5]),
                                 (str(k), "exit"), 800.) for k in range(3)]

    @staticmethod
    def car(x, y, speed, uid=1, valid=True):
        return (x, y, 4.9, "lidar_car", "npc", speed, 4.9, 2.1,
                0., speed, 0., uid, 0., None, valid)

    def decide(self, objects=(), lanes=None, static=None, ready=True, rear=None, ego=None):
        return self.planner.decide(0, self.route[:640], ego or self.ego, objects, {},
            lambda p, h: h, static or (lambda p, h: h), perception_ready=ready,
            rear_unclassified=rear, lane_options=self.lanes if lanes is None else lanes)

    def settle(self, objects=(), lanes=None, static=None, rear=None, ego=None):
        """A discretionary change starts once its margin held for lane_gain_hold_s."""
        ego = ego or self.ego
        self.decide(objects, lanes, static, rear=rear, ego=dict(ego, stamp=ego['stamp']-.5))
        return self.decide(objects, lanes, static, rear=rear, ego=ego)

    def test_empty_road_keeps_lane_when_mission_is_already_satisfied(self):
        result = self.decide()
        self.assertEqual(result["status"]["available_lanes"], ["0", "1", "2"])
        self.assertEqual(result["status"]["target_lane"], "0")
        self.assertIsNone(self.planner.saved_world)
        self.assertFalse(result["emergency"])

    def test_legacy_change_quota_does_not_force_changes_on_empty_road(self):
        self.planner.config['lane_change_goal'] = 2
        result = self.decide()
        self.assertEqual(result['status']['planned_lane_sequence'], [])
        self.assertEqual(result['status']['target_lane'], '0')
        self.assertIsNone(self.planner.saved_world)
        self.assertFalse(result['status']['lane_change_goal_is_minimum'])
        self.assertAlmostEqual(float(result['path'][:,1].max()), 0.)
        self.assertEqual(result['target_mps']*3.6,90.)

    def test_routine_change_with_slow_neighbour_uses_small_gradual_steering(self):
        self.planner.config['lane_change_goal'] = 2
        result = self.settle([self.car(110., 0., 8.)])
        self.assertEqual(result['status']['target_lane'], '1')
        self.assertGreaterEqual(result['status']['selected_transition_m'], 150.)
        self.assertGreaterEqual(result['target_mps']*3.6, 70.)
        tangents = np.diff(result['path'], axis=0)
        self.assertLess(np.max(np.abs(np.arctan2(tangents[:, 1], tangents[:, 0]))),
                        math.radians(3.))
        checks = result['status']['candidate_checks']
        self.assertFalse(any(c.get('urgent_current_lane') for c in checks))

    def test_optional_mission_does_not_choose_narrow_lane_below_70(self):
        self.planner.config['lane_change_goal'] = 2
        result = self.planner.decide(0, self.route[:640], self.ego, (), {},
            lambda p,h:h, lambda p,h:h, lane_options=self.lanes,
            path_speed_cap=lambda p: 40./3.6 if np.max(p[:, 1]) > 1. else float('inf'))
        self.assertEqual(result['status']['target_lane'], '0')
        self.assertGreaterEqual(result['target_mps']*3.6, 70.)
        self.assertTrue(any(c.get('side_gap_cap_kmh') == 40.
                            for c in result['status']['candidate_checks']))

    def test_final_candidate_geometry_is_checked_and_committed(self):
        self.planner.config['lane_change_goal'] = 2
        transformed, map_checked, static_checked = [], [], []
        def transform(p):
            result = np.array(p, copy=True)
            result[:, 1] += .1*np.clip(result[:, 0]/100., 0., 1.)**3
            transformed.append(result.copy())
            return result
        def map_clear(p, horizon):
            map_checked.append(np.array(p, copy=True))
            return horizon
        def static_clear(p, horizon):
            static_checked.append(np.array(p, copy=True))
            return horizon
        for stamp in (9.5, 10.):
            result = self.planner.decide(0, self.route[:640], dict(self.ego, stamp=stamp),
                [self.car(70., 0., 15.)], {},
                map_clear, static_clear, lane_options=self.lanes, path_transform=transform)
        self.assertIsNotNone(self.planner.saved_world)
        np.testing.assert_allclose(self.planner.saved_world, world_points(result['path'], self.ego))
        for checked in (map_checked, static_checked, transformed):
            self.assertTrue(any(np.array_equal(p, result['path']) for p in checked))

    def test_roadside_nudge_cannot_reduce_the_original_mapped_clearance(self):
        checked = []
        def transform(path):
            path[:, 1] += .7
            return path
        def map_clear(path, horizon):
            checked.append((float(np.max(np.abs(path[:, 1]))), horizon))
            return horizon if np.max(np.abs(path[:, 1]))<=.5 else 0.
        result = self.planner.decide(0, self.route[:640], self.ego, (), {},
            map_clear, lambda p,h:h, lane_options=self.lanes, path_transform=transform)
        self.assertFalse(result['emergency'])
        self.assertAlmostEqual(float(np.max(np.abs(result['path'][:, 1]))), 0.)
        self.assertTrue(result['status']['path_transform_rejected'])
        self.assertIn((0., 170.), checked)
        self.assertIn((.7, 170.), checked)

    def test_rejected_roadside_nudge_does_not_remove_a_real_obstacle_on_original_path(self):
        from tunnel_runtime import swept_clearance
        points = np.array([[30., 0.]])
        static = lambda p,h: swept_clearance(p, points, h, lateral_margin=.25)
        result = self.planner.decide(0, self.route[:640], self.ego, (), {},
            lambda p,h: h if np.max(np.abs(p[:,1]))<=.5 else 0.,
            static, lane_options=self.lanes,
            path_transform=lambda p: np.asarray(p)+[0., .7])
        self.assertAlmostEqual(float(np.max(np.abs(result['path'][:, 1]))), 0.)
        clearance = static(result['path'], 170.)
        self.assertLess(clearance, 30.)
        command = self.planner.plan_speed(result['path'], self.ego, (),
            result['target_mps'], static_clear_m=clearance)
        self.assertTrue(command['emergency'])
        self.assertEqual(command['reference_mps'], 0.)

    def test_needed_outer_overtake_stays_outer_without_forced_return(self):
        self.planner.three_lane_policy = None
        self.planner.config["lane_change_goal"] = 2
        first = self.settle([self.car(65., 0., 15.), self.car(110., 3.5, 15., 2)])
        self.assertEqual(first["status"]["target_lane"], "1")
        length = first['status']['selected_transition_m']
        # Complete the first change in its measured lane and recheck the next.
        ego = dict(self.ego, x=length, y=3.5, stamp=15.)
        lanes = [HighwayLane(l.link_id, l.rank,
            l.centre[l.centre[:, 0] >= length]-[length, 3.5], l.chain, 800.-length)
            for l in self.lanes]
        # The middle lane still has its slower car ahead, so the next leg is
        # worth a start (free lane choice: each completed leg is a decision).
        second = self.decide([self.car(length+40., 3.5, 15., 2)], lanes=lanes, ego=ego)
        self.assertEqual(self.planner.completed, 1)
        self.assertEqual(second["status"]["target_lane"], "2")
        self.assertTrue(np.all(second["path"][:, 1] <= 3.51))
        x = length*2.+max(3., 25.*.5)
        outer = [HighwayLane(l.link_id, l.rank,
            l.centre[l.centre[:, 0] >= x]-[x, 7.], l.chain, 800.-x)
            for l in self.lanes]
        third = self.decide(lanes=outer, ego=dict(self.ego, x=x, y=7., stamp=20.))
        self.assertEqual(third['status']['target_lane'], '2')
        self.assertIsNone(self.planner.saved_world)
        self.assertFalse(third['status']['further_overtake_requested'])

    def test_slow_front_car_selects_overtake_before_following(self):
        result = self.settle([self.car(70., 0., 15.)])
        self.assertEqual(result["status"]["target_lane"], "1")
        self.assertFalse(result["emergency"])
        self.assertGreaterEqual(result["target_mps"], 24.)

    def test_outer_lane_can_return_when_a_new_lead_requires_it(self):
        self.planner.completed = 3
        self.planner.config['lane_change_goal'] = 2
        outer = [HighwayLane(l.link_id, l.rank, l.centre-[0., 7.], l.chain, 800.)
                 for l in self.lanes]
        result = self.settle([self.car(70., 7., 15.)], lanes=outer,
                             ego=dict(self.ego, y=7.))
        self.assertEqual(result['status']['target_lane'], '1')
        self.assertTrue(result['status']['further_overtake_requested'])
        self.assertGreaterEqual(result['target_mps']*3.6, 70.)

    def test_outer_lane_plan_publishes_checked_continuous_adjacent_changes(self):
        result = self.settle([self.car(65., 0., 15.), self.car(110., 3.5, 15., 2)])
        self.assertTrue(any(len(c.get("lanes", [])) == 2 for c in result["status"]["candidate_checks"]))
        self.assertEqual(result["status"]["planned_lane_sequence"], ["1", "2"])
        path = result['path']
        self.assertAlmostEqual(float(path[:, 1].max()), 7.)
        length = result['status']['selected_transition_m']
        self.assertAlmostEqual(float(np.interp(length, path[:, 0], path[:, 1])), 3.5, places=2)
        # Reach the intermediate lane before starting the second change.
        self.assertLessEqual(float(path[path[:, 0] <= length, 1].max()), 3.51)
        np.testing.assert_allclose(self.planner.saved_world, world_points(path, self.ego))

    def test_fast_rear_car_blocks_middle_and_outer_sequences(self):
        result = self.decide([self.car(-20., 3.5, 35.), self.car(90., 0., 20., 2)])
        self.assertEqual(result["status"]["target_lane"], "0")
        self.assertFalse(any(c["accepted"] for c in result["status"]["candidate_checks"]))
        self.assertFalse(result["emergency"])

    def test_slower_parallel_rear_car_allows_accelerating_overtake(self):
        ego = dict(self.ego, speed_kmh=54.6)
        result = self.settle([self.car(0., 3.5, 7.8), self.car(47., 0., 5., 2)], ego=ego)
        self.assertEqual(result['status']['target_lane'], '1')
        self.assertTrue(any(c['accepted'] for c in result['status']['candidate_checks']))
        checked = self.planner.plan_speed(result['path'], ego,
            [self.car(0., 3.5, 7.8), self.car(47., 0., 5., 2)], result['target_mps'])
        self.assertFalse(checked['emergency'])
        self.assertGreaterEqual(checked['reference_mps']*3.6, ego['speed_kmh'])

    def test_slower_rear_car_does_not_reduce_clear_lane_speed(self):
        ego = dict(self.ego, speed_kmh=54.6)
        self.planner.longitudinal.begin(ego)
        result = self.planner.plan_speed(self.route, ego, [self.car(-3., 3.5, 7.8)], 25.)
        self.assertFalse(result['emergency'])
        self.assertGreater(result['reference_mps']*3.6, ego['speed_kmh'])

    def test_outer_rear_car_rejects_second_lane_in_sequence(self):
        result = self.decide([self.car(-20., 7., 35.), self.car(65., 0., 15., 2)])
        checks = [c for c in result["status"]["candidate_checks"] if len(c.get("lanes", [])) == 2]
        self.assertTrue(checks)
        self.assertTrue(all(not c["accepted"] for c in checks))

    def test_second_leg_rechecks_new_rear_traffic_after_first_completes(self):
        first = self.settle([self.car(65., 0., 15.), self.car(110., 3.5, 15., 2)])
        self.assertEqual(first['status']['planned_lane_sequence'],['1','2'])
        length = first['status']['selected_transition_m']
        lanes = [HighwayLane(l.link_id, l.rank,
            l.centre[l.centre[:, 0] >= length]-[length,3.5], l.chain,800.-length)
            for l in self.lanes]
        result = self.decide([self.car(length-35.,7.,35.)],lanes=lanes,
                             ego=dict(self.ego,x=length,y=3.5,stamp=16.))
        self.assertEqual(self.planner.completed,1)
        self.assertEqual(result['status']['target_lane'],'1')
        self.assertFalse(result['emergency'])
        self.assertIsNone(self.planner.saved_world)
        self.assertGreaterEqual(result['target_mps']*3.6, 70.)

    def test_second_leg_keeps_safe_sequence_when_rear_car_passes_before_merge(self):
        first = self.settle([self.car(65.,0.,15.),self.car(110.,3.5,15.,2)])
        length = first['status']['selected_transition_m']
        saved = self.planner.saved_world.copy()
        lanes = [HighwayLane(l.link_id,l.rank,
            l.centre[l.centre[:,0]>=length]-[length,3.5],l.chain,800.-length)
            for l in self.lanes]
        # The middle lane's slower car is still ahead: continuing is a gain.
        result = self.decide([# The fast car has cleared our real nose before the second leg.
            self.car(length+10.,7.,35.),self.car(length+40.,3.5,15.,3)],
            lanes=lanes, ego=dict(self.ego,x=length,y=3.5,stamp=16.))
        self.assertEqual(result['status']['target_lane'],'2')
        self.assertGreaterEqual(result['target_mps']*3.6,70.)
        np.testing.assert_array_equal(self.planner.saved_world,saved)

    def test_slow_leads_choose_continuous_overtake_with_small_steering(self):
        result = self.settle([self.car(65.,0.,15.),self.car(110.,3.5,15.,2)])
        self.assertEqual(result['status']['planned_lane_sequence'],['1','2'])
        self.assertGreaterEqual(result['target_mps']*3.6,70.)
        self.assertGreaterEqual(result['status']['selected_transition_m'],75.)
        self.assertLess(result['status']['selected_transition_m'],150.)
        # Wheelbase 2.97 m: the checked double change remains below 1 degree.
        from highway_planner import path_curvature
        steering = np.arctan(2.97*path_curvature(result['path']))
        self.assertLess(float(steering.max()),math.radians(1.))

    def test_replacement_change_matches_measured_heading(self):
        route = self.route[:640].copy()
        theta = math.radians(-2.)
        rotation = np.array([[math.cos(theta),-math.sin(theta)],
                             [math.sin(theta),math.cos(theta)]])
        route = route @ rotation.T + [0., 2.]
        path = lane_change_path(route, 0., 150., match_heading=True)
        initial = path[1]-path[0]
        self.assertLess(abs(math.atan2(initial[1],initial[0])), math.radians(.05))
        np.testing.assert_allclose(path[-1],route[-1])

    def test_heading_alignment_does_not_swing_into_another_lane(self):
        ego = dict(self.ego,heading_deg=5.)
        local = local_points(self.route[:640],ego)
        path = lane_change_path(local,0.,150.,match_heading=True)
        aligned = world_points(path,ego)
        self.assertLess(float(np.abs(aligned[:,1]).max()),.7)

    def test_active_change_is_not_restarted_to_satisfy_optional_mission(self):
        self.planner.config['lane_change_goal'] = 2
        first = self.settle([self.car(70., 0., 15.)])
        saved = self.planner.saved_world.copy()
        started = self.planner.three_lane_policy.change_started
        policy = self.planner.three_lane_policy
        original = policy.evaluate
        def limited(*args,**kwargs):
            result,check = original(*args,**kwargs)
            if result is not None:
                result['trajectory'].speeds[-1] = 15.
                if kwargs.get('changing'):
                    result['check']['average_kmh'] += .5
            return result,check
        with patch.object(policy,'evaluate',side_effect=limited):
            result = self.decide(ego=dict(self.ego,stamp=10.1))
        np.testing.assert_array_equal(self.planner.saved_world,saved)
        self.assertEqual(policy.change_started,started)

    def test_saved_gentle_change_replans_before_a_real_static_hit_forces_slowdown(self):
        from tunnel_runtime import swept_clearance
        self.planner.three_lane_policy = ThreeLanePolicy(self.config)
        policy = self.planner.three_lane_policy
        policy.target_chain = self.lanes[1].chain
        policy.change_chains = [self.lanes[1].chain]
        policy.change_length, policy.change_goal = 200., 25.
        policy.change_started = 9.
        self.planner.phase = 'CHANGE'
        saved = lane_change_path(self.lanes[1].centre, 0., 200., match_heading=True)
        self.planner.saved_world = world_points(saved, self.ego)
        points = np.array([[110., 2.]])
        static = lambda p,h: swept_clearance(p, points, h, lateral_margin=.25)
        original_clear = static(saved, 170.)
        self.assertGreater(original_clear, 100.)
        before = self.planner.longitudinal.preview(saved, self.ego, (), 25.,
            static_clear_m=original_clear, standoff_m=12.)
        self.assertGreater(np.interp(3., before.times, before.speeds)*3.6, 70.)
        result = self.decide(static=static)
        self.assertFalse(result['emergency'])
        self.assertEqual(result['status']['target_lane'], '1')
        self.assertLessEqual(result['status']['selected_transition_m'], 100.)
        self.assertGreaterEqual(result['target_mps']*3.6, 89.9)
        self.assertIn('bounded static hit within comfort braking reserve',
                      result['status']['continuation_replan_reasons'])
        self.assertGreater(static(result['path'], 170.), original_clear+10.)
        from highway_planner import path_curvature
        self.assertLess(np.max(np.arctan(2.97*path_curvature(result['path']))), math.radians(1.))

    def test_completed_static_query_does_not_restart_a_safe_saved_change(self):
        self.planner.three_lane_policy = ThreeLanePolicy(self.config)
        policy = self.planner.three_lane_policy
        policy.target_chain = self.lanes[1].chain
        policy.change_chains = [self.lanes[1].chain]
        policy.change_length, policy.change_goal = 200., 25.
        policy.change_started = 9.
        self.planner.phase = 'CHANGE'
        saved = lane_change_path(self.lanes[1].centre, 0., 200., match_heading=True)
        self.planner.saved_world = world_points(saved, self.ego)
        result = self.decide()
        self.assertEqual(result['status']['continuation_replan_reasons'], [])
        self.assertEqual(result['status']['selected_transition_m'], 200.)
        np.testing.assert_allclose(self.planner.saved_world, world_points(saved, self.ego))

    def test_bounded_static_hit_does_not_reroot_without_a_checked_clearance_gain(self):
        from tunnel_runtime import swept_clearance
        self.planner.three_lane_policy = ThreeLanePolicy(self.config)
        policy = self.planner.three_lane_policy
        policy.target_chain = self.lanes[1].chain
        policy.change_chains = [self.lanes[1].chain]
        policy.change_length, policy.change_goal = 200., 25.
        policy.change_started = 9.
        self.planner.phase = 'CHANGE'
        saved = lane_change_path(self.lanes[1].centre, 0., 200., match_heading=True)
        self.planner.saved_world = world_points(saved, self.ego)
        points = np.column_stack((np.full(88, 110.), np.linspace(-3., 10., 88)))
        static = lambda p,h: swept_clearance(p, points, h, lateral_margin=.25)
        result = self.decide(static=static)
        self.assertIn('bounded static hit within comfort braking reserve',
                      result['status']['continuation_replan_reasons'])
        self.assertEqual(result['status']['selected_transition_m'], 200.)
        np.testing.assert_allclose(self.planner.saved_world, world_points(saved, self.ego))

    def test_checked_new_path_clears_old_stop_hold_but_keeps_live_cut_in_stop(self):
        self.planner._speed_hold_goal = 0.
        self.planner._speed_hold_until = 12.
        result = self.settle([self.car(70.,0.,15.)])
        self.assertEqual(result['status']['target_lane'],'1')
        self.assertIsNone(self.planner._speed_hold_goal)
        cut_in = list(self.car(25.,3.6,10.))
        cut_in[10] = -1.
        live = self.planner.plan_speed(result['path'],self.ego,[tuple(cut_in)],25.)
        self.assertTrue(live['emergency'])
        self.assertEqual(live['reference_mps'],0.)

    def test_unclassified_rear_points_block_new_lane(self):
        result = self.decide([self.car(80., 0., 18.)], rear=np.array([[-15., 3.5]]))
        self.assertEqual(result["status"]["target_lane"], "0")

    def test_sensor_late_retains_lane_without_new_change(self):
        result = self.decide([self.car(80., 0., 18.)], ready=False)
        self.assertEqual(result["status"]["target_lane"], "0")

    def test_missing_native_geometry_does_not_enable_unknown_lane(self):
        result = self.decide(lanes=[])
        self.assertTrue(result["emergency"])
        self.assertEqual(result["target_mps"], 0.)

    def test_taper_distance_replaces_fixed_route_index_limit(self):
        self.planner.change_until = 0
        result = self.settle([self.car(70., 0., 15.)])
        self.assertEqual(result["status"]["target_lane"], "1")
        self.planner.reset()
        lanes = [HighwayLane(l.link_id, l.rank, l.centre, l.chain, 20.) for l in self.lanes]
        result = self.decide([self.car(70., 0., 15.)], lanes=lanes)
        self.assertEqual(result["status"]["target_lane"], "0")
        self.assertEqual(result["status"]["phase"], "MERGE")

    def test_free_lane_is_retained_after_overtake_goal(self):
        ego = dict(self.ego, y=7., stamp=20.)
        lanes = [HighwayLane(l.link_id, l.rank, l.centre-[0., 7.], l.chain, 800.) for l in self.lanes]
        self.planner.completed = 2
        result = self.decide(lanes=lanes, ego=ego)
        self.assertEqual(result["status"]["target_lane"], "2")
        self.assertIsNone(self.planner.saved_world)

    def test_optional_speed_gain_requires_half_second_of_stability(self):
        self.decide()
        policy = self.planner.three_lane_policy
        evaluate = policy.evaluate
        def scores(*args, **kwargs):
            result, check = evaluate(*args, **kwargs)
            if result is not None and not kwargs.get('changing'):
                # Controlled forecast input: a transient reduction has ended
                # by the horizon, so no immediate braking exception applies.
                result['check']['average_kmh'] = 80.
            return result, check
        with patch.object(policy, 'evaluate', side_effect=scores):
            first = self.decide(ego=dict(self.ego, stamp=11.))
            self.assertEqual(first['status']['target_lane'], '0')
            before = self.decide(ego=dict(self.ego, stamp=11.49))
            self.assertEqual(before['status']['target_lane'], '0')
            after = self.decide(ego=dict(self.ego, stamp=11.51))
            self.assertEqual(after['status']['target_lane'], '1')

    def test_static_blocked_target_lane_is_rejected(self):
        def static(path, horizon):
            return 10. if np.max(path[:, 1]) > 1. else horizon
        result = self.decide([self.car(70., 0., 15.)], static=static)
        self.assertEqual(result["status"]["target_lane"], "0")

    def test_gentle_following_limits_acceleration_and_jerk(self):
        model = self.planner.longitudinal
        trajectory = model.preview(self.route, self.ego, [self.car(60., 0., 18.)], 25., 10.)
        self.assertGreaterEqual(float(trajectory.accelerations.min()), -1.2-1e-9)
        self.assertLessEqual(float(np.abs(np.diff(trajectory.accelerations)).max()), .2+1e-9)
        self.assertLess(trajectory.speeds[-1], 25.)

    def test_close_cut_in_escalates_braking_instead_of_disabling_safety(self):
        self.planner.longitudinal.begin(self.ego)
        result = self.planner.plan_speed(self.route, self.ego, [self.car(35., 0., 10.)], 25.)
        self.assertTrue(result["emergency"] or result["diagnostics"]["braking_limit_mps2"] > 1.2)

    def test_measured_cut_in_retains_follower_response_reserve(self):
        obj = list(self.car(32., 3.6, 10.))
        obj[10] = -.7
        self.planner.longitudinal.begin(self.ego)
        result = self.planner.plan_speed(self.route, self.ego, [tuple(obj)], 25.)
        self.assertTrue(result['emergency'])
        self.assertEqual(result['reference_mps'], 0.)

    def test_parallel_neighbour_does_not_trigger_cut_in_braking(self):
        self.planner.longitudinal.begin(self.ego)
        result = self.planner.plan_speed(self.route, self.ego, [self.car(32., 3.6, 10.)], 25.)
        self.assertFalse(result['emergency'])
        self.assertGreaterEqual(result['reference_mps'], 24.9)

    def test_ego_heading_change_does_not_turn_parallel_motion_into_cut_in(self):
        ego = dict(self.ego,heading_deg=5.)
        road = np.column_stack((np.arange(-100.,1000.,.5),np.full(2200,4.25)))
        lane = HighwayLane('other',1,local_points(road,ego),('other','exit'),800.)
        car = self.car(4.,4.25,11.)
        path = lane_change_path(local_points(self.route[:640],ego),0.,150.,match_heading=True)
        self.planner.longitudinal.begin(ego)
        ThreeLanePolicy(self.config).update_motion_paths(self.planner,[lane],ego,[car])
        self.planner.saved_world = world_points(path,ego)
        result = self.planner.plan_speed(path,ego,[car],25.)
        self.assertFalse(result['emergency'])
        self.assertGreaterEqual(result['reference_mps'],24.9)
        # Use the same ID and lane geometry but a fresh measured lateral move.
        # The cached native motion estimate cannot waive the new cut-in.
        cut_in = list(self.car(32.,3.6,10.))
        cut_in[10] = -.7
        unsafe = self.planner.plan_speed(path,ego,[tuple(cut_in)],25.)
        self.assertTrue(unsafe['emergency'])
        self.assertEqual(unsafe['reference_mps'],0.)

    def test_native_rear_conflict_selects_a_checked_escape_lane(self):
        full = np.column_stack((np.arange(-100., 1000., .5), np.zeros(2200)))
        motion = [HighwayLane('rear-road', -1, full, ('rear-road', 'exit'), 800.)]
        result = self.planner.decide(0, self.route[:640], self.ego,
            [self.car(-30., 0., 35.)], {}, lambda p,h:h, lambda p,h:h,
            lane_options=self.lanes, motion_options=motion)
        self.assertTrue(result['status']['gap_risk'] is None)
        self.assertEqual(result['status']['target_lane'], '1')

    def test_highway_comfort_parameters_do_not_change_legacy_policy(self):
        legacy = HighwayPlanner(dict(from_idx=0, to_idx=1800, kmh=90.), self.route)
        self.assertEqual(legacy.longitudinal.comfort_decel_mps2, 2.5)
        self.assertEqual(legacy.longitudinal.jerk_mps3, 4.)
        self.planner.reset()
        self.assertEqual(self.planner.longitudinal.comfort_decel_mps2, 1.2)

    def test_invalid_comfort_parameters_fail_before_driving(self):
        for key in ('comfort_decel_mps2', 'comfort_jerk_mps3', 'emergency_jerk_mps3'):
            for value in (0., -1., float('nan'), float('inf')):
                with self.subTest(key=key, value=value):
                    with self.assertRaisesRegex(ValueError, 'finite and positive'):
                        HighwayPlanner(dict(self.config, **{key:value}), self.route)

    def test_exit_goal_arrives_at_sixty_kmh_ceiling(self):
        result = self.planner.decide(1800, self.route[:640], self.ego, [], {},
            lambda p, h: h, lambda p, h: h, lane_options=self.lanes)
        self.assertLessEqual(result["target_mps"]*3.6, 60.+1e-9)

    def test_exit_speed_envelope_acts_along_the_forecast_distance(self):
        model = self.planner.longitudinal
        model.end_distance_m = 160.
        model.end_buffer_m = 25.*(.6+model.comfort_decel_mps2/model.jerk_mps3)
        forecast = model.preview(self.route, self.ego, [], 25., 12.)
        self.assertLess(forecast.reference_mps, 25.)
        crossing = forecast.distances >= model.end_distance_m
        self.assertTrue(crossing.any())
        # The rollout uses 0.1 s integration steps; the published boundary
        # ceiling is tested separately against the exact 60 km/h limit.
        self.assertLessEqual(float(forecast.speeds[crossing].max()),
                             model.end_speed_mps+model.comfort_decel_mps2*.1)


class SurveyedHighwayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.route = np.array([[p["x"], p["y"]] for p in json.loads((HERE/"global_path.json").read_text())["waypoints"]])
        cls.smooth = smooth_control_route(cls.route)
        cls.lanes = HdMapLanes(HERE/"link_set.json")
        cls.lanes.set_route(cls.route)
        cls.config = json.loads((HERE/"highway_mission.json").read_text())
        cls.config.update(lane_change_policy="free_three_lane")

    def frame(self, index, rank=0):
        p = self.route[index]
        tangent = self.route[index+1]-self.route[index-1]
        ego = dict(x=p[0], y=p[1], heading_deg=math.degrees(math.atan2(tangent[1], tangent[0])),
                   speed_kmh=70., stamp=float(index))
        native = HighwayLaneMap(self.lanes, self.config)
        local = local_points(self.smooth[index:index+640], ego)
        candidates = native.candidates(ego, local)
        return ego, local, native, candidates

    def test_left_additional_lane_exists_in_surveyed_map(self):
        _, _, _, candidates = self.frame(2800)
        self.assertEqual([c.link_id for c in candidates],
                         ["A2256W000408", "A2256W000434", "A2256W000435"])

    def test_native_link_boundary_retains_recent_surveyed_geometry(self):
        ego,local,native,observed = self.frame(2800)
        with patch.object(native,'chain',return_value=None):
            retained = native.candidates(dict(ego,stamp=ego['stamp']+.1),local)
            self.assertEqual([l.link_id for l in retained],[l.link_id for l in observed])
            self.assertTrue(native.geometry_reused)
            self.assertEqual(native.candidates(dict(ego,stamp=ego['stamp']+.3),local),[])

    def test_recent_native_geometry_cannot_bypass_new_block_rules(self):
        ego,local,native,_ = self.frame(2800)
        with patch.object(native,'chain',return_value=None),patch.object(native,'blocked',return_value=True):
            self.assertEqual(native.candidates(dict(ego,stamp=ego['stamp']+.1),local),[])

    def test_recent_native_geometry_cannot_cover_ego_outside_surveyed_road(self):
        ego,local,native,_ = self.frame(2800)
        with patch.object(native,'chain',return_value=None):
            self.assertEqual(native.candidates(dict(ego,x=ego['x']+30.,stamp=ego['stamp']+.1),local),[])

    def test_production_policy_stays_inside_the_defined_highway_section(self):
        config = json.loads((HERE/'highway_mission.json').read_text())
        self.assertEqual(config['lane_change_policy'], 'free_three_lane')
        planner = HighwayPlanner(config, self.route)
        self.assertFalse(planner.active(config['from_idx']-1))
        self.assertTrue(planner.active(config['from_idx']))
        self.assertTrue(planner.active(config['to_idx']))
        self.assertFalse(planner.active(config['to_idx']+1))
        # Aggressive highway (2026-10-10): firmer planned braking and jerk.
        self.assertEqual(planner.longitudinal.comfort_decel_mps2, 2.5)
        self.assertEqual(planner.longitudinal.jerk_mps3, 4.)

    def test_all_three_native_paths_share_common_entrance(self):
        _, _, _, candidates = self.frame(3000)
        for lane in candidates:
            self.assertIn("A2256W000153", lane.chain)
            self.assertGreater(lane.merge_remaining_m, 0.)
        self.assertTrue(np.allclose(candidates[0].centre[-1], candidates[2].centre[-1]))

    def test_outer_lane_footprint_is_inside_highway_only_corridor(self):
        ego, local, native, candidates = self.frame(2800)
        corridor = native.corridor(ego, local, candidates)
        path = lane_change_path(candidates[2].centre, 0., 225.)
        self.assertGreaterEqual(corridor.clear_distance(path, 170.), 150.)
        general = self.lanes.corridor(ego, local)
        self.assertLess(general.clear_distance(path, 170.), 150.)

    def test_native_paths_are_available_through_mission_boundary(self):
        for index in (2251, 2430, 2600, 2800, 3060, 3200, 3400, 3500):
            with self.subTest(index=index):
                _, _, native, candidates = self.frame(index)
                self.assertTrue(candidates)
                entry = json.loads((HERE/'highway_mission.json').read_text()).get('entry_lane_change', {})
                ramp = [lane for lane in candidates if lane.link_id == entry.get('source_link')]
                self.assertLessEqual(len(candidates)-len(ramp), 3)
                for lane in candidates:
                    self.assertFalse(native.lanes.link_by_id[lane.link_id].get('opp_traffic'))
                    self.assertTrue(native.exit_link in lane.chain or
                                    lane.link_id in native.exit_continuations)

    def test_live_whitelist_keeps_route_and_all_three_selected_lanes(self):
        rules = dict(mode='whitelist', allowed_links=['A2256W000409','A2256W000410',
            'A2256W000430','A2256W000431','A2256W000434','A2256W000435',
            'A2256W000422','A2256W000423'])
        with patch.object(self.lanes, 'rules_cache', SimpleNamespace(value=rules)):
            _, _, _, candidates = self.frame(2800)
            self.assertEqual(len(candidates), 3)

    def test_whitelist_denied_additional_lane_remains_unavailable(self):
        rules = dict(mode='whitelist', allowed_links=['A2256W000434','A2256W000422'])
        with patch.object(self.lanes, 'rules_cache', SimpleNamespace(value=rules)):
            _, _, _, candidates = self.frame(2800)
            self.assertEqual([c.link_id for c in candidates],['A2256W000408','A2256W000434'])

    def test_moving_lead_forecast_follows_the_surveyed_lane_merge(self):
        ego,local,native,candidates = self.frame(2400)
        lane = candidates[0]
        world = world_points(lane.centre,ego)
        stations = arc(world)
        at = int(np.searchsorted(stations,35.))
        tangent = world[at+1]-world[at-1];tangent /= np.linalg.norm(tangent)
        velocity = tangent*11.
        obj = list(ThreeLaneTests.car(*world[at],11.))
        obj[9:11] = velocity.tolist()
        planner = HighwayPlanner(self.config,self.route)
        planner.decide(2400,local,ego,[tuple(obj)],{},lambda p,h:h,lambda p,h:h,lane_options=candidates)
        prediction = planner.longitudinal.object_positions(tuple(obj),np.array([0.,10.]))
        self.assertLess(float(np.linalg.norm(prediction[0]-world[at])),.1)
        self.assertLess(float(cKDTree(world).query(prediction[1])[0]),.4)
        linear = world[at]+velocity*10.
        self.assertGreater(float(np.linalg.norm(prediction[1]-linear)),1.)

    def test_rear_motion_uses_the_vehicle_own_native_link(self):
        ego, local, native, candidates = self.frame(2800)
        at = 2720
        tangent = self.route[at+1]-self.route[at-1]
        tangent /= np.linalg.norm(tangent)
        obj = list(ThreeLaneTests.car(*self.route[at], 30.))
        obj[9:11] = (tangent*30.).tolist()
        motion = native.motion_candidates(ego, [tuple(obj)])
        planner = HighwayPlanner(self.config, self.route)
        planner.decide(2800, local, ego, [tuple(obj)], {}, lambda p,h:h, lambda p,h:h,
                       lane_options=candidates, motion_options=motion)
        self.assertIn(obj[11], planner.longitudinal.motion_paths)
        predicted = planner.longitudinal.object_positions(tuple(obj), np.array([0., 2.]))
        self.assertLess(float(np.linalg.norm(predicted[0]-np.asarray(obj[:2]))), .1)

    def test_native_merge_is_independent_of_legacy_change_quota(self):
        planner = HighwayPlanner(self.config, self.route)
        legacy = HighwayPlanner(dict(self.config, lane_change_goal=2), self.route)
        native = HighwayLaneMap(self.lanes, self.config)
        index = planner.start
        position = self.smooth[index].copy()
        heading = math.atan2(*(self.smooth[index+3]-self.smooth[index])[::-1])
        speed = 25.
        for frame in range(700):
            lo, hi = max(planner.start, index-12), min(planner.end+1, index+32)
            index = lo+int(np.argmin(np.linalg.norm(self.smooth[lo:hi]-position, axis=1)))
            ego = dict(x=float(position[0]), y=float(position[1]),
                       heading_deg=math.degrees(heading), speed_kmh=speed*3.6,
                       stamp=frame*.2)
            local = local_points(self.smooth[index:index+640], ego)
            candidates = native.candidates(ego, local)
            corridor = native.corridor(ego, local, candidates)
            result = planner.decide(index, local, ego, [], {}, corridor.clear_distance,
                                    lambda path, horizon: horizon, lane_options=candidates)
            other = legacy.decide(index, local, ego, [], {}, corridor.clear_distance,
                                  lambda path, horizon: horizon, lane_options=candidates)
            self.assertEqual(result['status']['target_lane'], other['status']['target_lane'])
            self.assertEqual(result['status']['planned_lane_sequence'],
                             other['status']['planned_lane_sequence'])
            np.testing.assert_allclose(result['path'], other['path'])
            self.assertAlmostEqual(result['target_mps'], other['target_mps'])
            self.assertFalse(result['emergency'], (index, result['status']))
            self.assertGreater(result['target_mps'], 1., (index, result['status']))
            self.assertLessEqual(result['target_mps'], planner.target_mps)
            if index >= planner.end-1:
                break
            speed += float(np.clip(result['target_mps']-speed, -.6, .24))
            advance = min(5., speed*.2)
            path = result['path']
            stations = arc(path)
            point = np.array([np.interp(advance, stations, path[:, k]) for k in (0, 1)])
            tangent = np.array([np.interp(advance+.5, stations, path[:, k]) for k in (0, 1)])-point
            position = world_points(point[None], ego)[0]
            heading += math.atan2(tangent[1], tangent[0])
        self.assertGreaterEqual(index, planner.end-1)
        self.assertEqual(planner.completed, legacy.completed)
        self.assertEqual(planner.status['common_entry'], 'A2256W000153')
        self.assertEqual(planner.phase, 'MERGE')


if __name__ == "__main__":
    unittest.main()
