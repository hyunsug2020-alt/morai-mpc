"""Choose a measured safe small bypass while preserving necessary sharp manoeuvres."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from gui import (BYPASS_ALAT_MPS2, build_forced_bypass, continue_static_plan,
                 local_xy, path_curvatures, plan_static_bypass,
                 static_clear_distance, static_speed_cap_mps)
from lane_corridor import LaneCorridor


class StaticBypassSelectionTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 80.5, .5), np.zeros(161)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=18., wheel_angle=0.)
        self.corridor = LaneCorridor(self.route, np.full(161, 5.25), np.full(161, -5.25))

    def lane_ok(self, path, tolerance):
        horizon = min(60., float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()))
        return self.corridor.clear_distance(path, horizon) >= horizon-.3

    def plan(self, front=20., lateral=-1.2, speed=18., fast=True, **kwargs):
        ego = dict(self.ego, speed_kmh=speed)
        objects = [(front, lateral, .8, 'measured_box', 'obstacle', 0., .8, .4, 0.)]
        plan, hazards = plan_static_bypass(self.route, ego, objects, self.lane_ok,
            adjacent_offsets={1: 3.5, -1: 3.5}, corridor_clearance=self.corridor.clear_distance,
            first_complete=fast, **kwargs)
        return plan, hazards, ego, objects

    def assert_safe(self, plan, ego, objects, horizon=60.):
        self.assertFalse(plan.get('blocked'), plan)
        path = plan['path'][:, :2]
        end = min(horizon, float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()))
        self.assertGreaterEqual(static_clear_distance(path, ego, objects, end), end-.3)
        self.assertGreaterEqual(self.corridor.clear_distance(path, end), end-.3)
        self.assertGreaterEqual(plan['clear_distance_m'], plan['hold_until_m'])
        self.assertGreaterEqual(plan['clear_distance_m'], plan['safe_horizon_m'])
        curvature = float(np.max(path_curvatures(path)))
        steering = math.degrees(math.atan(3.90*curvature))
        self.assertLessEqual(steering, 35.+1e-5)
        cap = static_speed_cap_mps(28.94/3.6, plan)
        self.assertLessEqual(cap*cap*curvature, BYPASS_ALAT_MPS2+1e-6)
        self.assertLess(abs(float(path[-1, 1])), .01)
        return steering

    def test_both_planner_modes_choose_smallest_safe_edge_adjustment(self):
        for fast in (False, True):
            for lateral, side in ((-1.2, 1.), (1.2, -1.)):
                with self.subTest(fast=fast, lateral=lateral):
                    plan, hazards, ego, objects = self.plan(lateral=lateral, fast=fast)
                    self.assertTrue(hazards)
                    self.assertEqual(plan['offset_m'], .45)
                    self.assertEqual(plan['direction'], side)
                    self.assertFalse(plan['adjacent_lane'])
                    self.assert_safe(plan, ego, objects)

    def test_edge_adjustments_are_gentle_at_every_supported_local_speed(self):
        for speed in (0., 9., 18., 28.94):
            with self.subTest(speed=speed):
                plan, _, ego, objects = self.plan(speed=speed)
                self.assertEqual(plan['offset_m'], .45)
                self.assertGreaterEqual(plan['transition_m'], 22.)
                self.assertLess(self.assert_safe(plan, ego, objects), 2.)
                self.assertLessEqual((speed/3.6)**2*plan['curvature'], BYPASS_ALAT_MPS2)

    def test_fast_edge_search_checks_opposite_small_shift_before_wide_lane_change(self):
        with patch('gui.build_forced_bypass', wraps=build_forced_bypass) as builder:
            plan, _, ego, objects = self.plan(lateral=1.2)
        self.assertEqual(plan['offset_m'], .45)
        self.assertEqual(plan['direction'], -1.)
        self.assertLessEqual(builder.call_count, 7)
        self.assert_safe(plan, ego, objects)

    def test_narrow_centre_obstacle_uses_a_checked_intermediate_width(self):
        plan, _, ego, objects = self.plan(lateral=0., speed=28.94)
        self.assertEqual(plan['offset_m'], 1.6)
        self.assertFalse(plan['adjacent_lane'])
        self.assert_safe(plan, ego, objects)

    def test_wide_centre_obstacle_retains_a_required_full_lane_bypass(self):
        ego = dict(self.ego, speed_kmh=28.94)
        objects = [(28., 0., .8, 'wide_box', 'obstacle', 0., .8, 4., 0.)]
        plan, _ = plan_static_bypass(self.route, ego, objects, self.lane_ok,
            adjacent_offsets={1: 3.5, -1: 3.5}, corridor_clearance=self.corridor.clear_distance,
            first_complete=True)
        self.assertEqual(plan['offset_m'], 3.5)
        self.assertTrue(plan['adjacent_lane'])
        self.assert_safe(plan, ego, objects)

    def test_close_obstacle_retains_safe_sharp_fallback_and_speed_reduction(self):
        urgent, _, ego, objects = self.plan(front=7., lateral=0., speed=5.)
        gentle, _, _, _ = self.plan(front=22., lateral=0., speed=5.)
        urgent_steering = self.assert_safe(urgent, ego, objects)
        self.assertLess(urgent['transition_m'], gentle['transition_m'])
        self.assertGreater(urgent_steering, 20.)
        self.assertLess(static_speed_cap_mps(28.94/3.6, urgent)*3.6, 15.)

    def test_short_observed_prefix_cannot_start_an_uncommitted_bypass(self):
        objects = [(20., 0., .6, 'box', 'obstacle', 0., .6, .6, 0.)]
        plan, _ = plan_static_bypass(self.route, self.ego, objects, lambda *_: True,
            adjacent_offsets={1: 3.5}, corridor_clearance=lambda path, horizon: 15.,
            first_complete=True)
        self.assertTrue(plan['blocked'])

    def test_unknown_corridor_and_no_legal_lane_still_block(self):
        objects = [(20., 0., .6, 'box', 'obstacle', 0., .6, .6, 0.)]
        for lane_ok, clear in ((lambda *_: True, lambda *_: 0.),
                               (lambda *_: False, self.corridor.clear_distance)):
            with self.subTest():
                plan, _ = plan_static_bypass(self.route, self.ego, objects, lane_ok,
                    adjacent_offsets={1: 3.5}, corridor_clearance=clear, first_complete=True)
                self.assertTrue(plan['blocked'])

    def test_high_speed_without_a_full_braking_horizon_cannot_validate_a_bypass(self):
        for speed in (55., 90.):
            with self.subTest(speed=speed):
                plan, _, _, _ = self.plan(speed=speed)
                self.assertTrue(plan['blocked'])
                self.assertGreater(plan['safe_horizon_m'], 50.)

    def test_empty_return_uses_a_gentle_transition_instead_of_the_short_default(self):
        outgoing, _, _, _ = self.plan(front=22., lateral=0., speed=28.94)
        index = int(np.argmin(np.abs(outgoing['path'][:, 0]-35.)))
        position = outgoing['path'][index, :2]
        tangent = outgoing['path'][index+1, :2]-outgoing['path'][index-1, :2]
        heading = math.degrees(math.atan2(tangent[1], tangent[0]))
        for speed in (9., 18., 28.94):
            with self.subTest(speed=speed):
                ego = dict(x=float(position[0]), y=float(position[1]), heading_deg=heading, speed_kmh=speed)
                saved = dict(stamp=10., world_path=outgoing['path'][:, :2],
                             plan={k: v for k, v in outgoing.items() if k != 'path'})
                previous = continue_static_plan(saved, ego, 10.1)
                self.assertIsNotNone(previous)
                route = np.array([local_xy(*p, ego) for p in self.route[self.route[:, 0] >= 35.]])
                corridor = LaneCorridor(route, np.full(len(route), 5.25), np.full(len(route), -5.25))
                def lane_ok(path, tolerance):
                    end = min(60., float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()))
                    return corridor.clear_distance(path, end) >= end-.3
                rejoin, _ = plan_static_bypass(route, ego, [], lane_ok, outgoing['direction'], previous,
                    {1: 3.5, -1: 3.5}, corridor.clear_distance, first_complete=True)
                self.assertFalse(rejoin.get('blocked'))
                self.assertEqual(rejoin['kind'], 'rejoin')
                self.assertGreaterEqual(rejoin['transition_m'], 22.)
                self.assertLess(math.degrees(math.atan(3.90*rejoin['curvature'])), 7.)
                self.assertLess(np.linalg.norm(rejoin['path'][0, :2]), .01)
                tangent = rejoin['path'][1, :2]-rejoin['path'][0, :2]
                self.assertLess(abs(math.degrees(math.atan2(tangent[1], tangent[0]))), .1)
                cap = static_speed_cap_mps(28.94/3.6, rejoin)
                self.assertLessEqual(cap*cap*rejoin['curvature'], BYPASS_ALAT_MPS2+1e-6)
                target = route[-1]
                self.assertLess(np.linalg.norm(rejoin['path'][-1, :2]-target), .6)
                sample = np.arange(0., 3.01, .25)
                def prefix(path):
                    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
                    return np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
                self.assertLess(np.linalg.norm(prefix(rejoin['path'])-prefix(previous['path']), axis=1).max(), .2)

    def carry_rate_path(self, path, ego):
        previous = dict(path=path, direction=1., offset_m=3.5, hold_m=6.,
                        curvature=float(np.max(path_curvatures(path))), transition_m=12., kind='bypass')
        with patch('gui.build_forced_bypass', return_value=(None, 0., None)):
            return plan_static_bypass(self.route, ego, [], lambda *_: True,
                committed_direction=1., previous_plan=previous, first_complete=True)[0]

    def test_strong_signed_turn_rate_survives_dense_and_duplicate_sampling(self):
        def curve(x):
            def smooth(t):
                t = np.clip(t, 0., 1.)
                return 10*t**3-15*t**4+6*t**5
            return 3.5*(smooth(x/12.)-smooth((x-18.)/12.))
        coarse = np.arange(0., 60.001, .5)
        grids = (coarse, np.arange(0., 60.001, .01),
                 np.sort(np.r_[coarse, 24., 24.009]))
        caps = []
        for x in grids:
            with self.subTest(samples=len(x)):
                path = np.column_stack((x, curve(x)))
                plan = self.carry_rate_path(path, dict(self.ego, speed_kmh=0.))
                self.assertFalse(plan.get('blocked'))
                caps.append(plan['steering_rate_cap_mps'])
                # A genuine fast reversal must still require slow travel.
                self.assertGreater(plan['steering_gradient_radpm'], .15)
                self.assertLess(plan['steering_rate_cap_mps']*3.6, 6.)
        for cap in caps[1:]:
            self.assertAlmostEqual(cap, caps[0], delta=caps[0]*.03)

    def test_unknown_initial_angle_does_not_invent_zero_but_measured_zero_is_checked(self):
        arc = np.arange(0., 60.001, .5)
        path = np.column_stack((30.*np.sin(arc/30.), 30.*(1.-np.cos(arc/30.))))
        ego = dict(self.ego, speed_kmh=28.94)
        del ego['wheel_angle']
        unknown = self.carry_rate_path(path, ego)
        self.assertFalse(unknown.get('blocked'))
        self.assertGreater(unknown['speed_cap_mps']*3.6, 20.)
        measured = self.carry_rate_path(path, dict(ego, wheel_angle=0.))
        self.assertTrue(measured['blocked'])
        compatible = self.carry_rate_path(path,
            dict(ego, wheel_angle=math.degrees(math.atan(3.90/30.))))
        self.assertFalse(compatible.get('blocked'))

    def test_near_turn_cannot_be_approved_before_current_speed_can_slow(self):
        plan, _, _, _ = self.plan(front=7., lateral=0., speed=28.94)
        self.assertTrue(plan['blocked'])


if __name__ == '__main__':
    unittest.main()
