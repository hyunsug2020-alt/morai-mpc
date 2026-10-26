"""Static search yields its deadline to fallback without bypassing current checks."""
import json
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

import gui
import model_avoidance as ma
import test_avoidance_gate as fixture
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from tunnel_runtime import swept_clearance
from static_lane_pass import StaticLanePass


class StaticSearchBudgetTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        self.route = np.column_stack((np.arange(0., 65.5, .5), np.zeros(131)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=12., wheel_angle=0.)
        self.objects = [(20., 0., .8, 'box', 'obstacle', 0., .8, .8, 0.)]

    def portable(self, **kwargs):
        return ma.portable.plan_static_bypass(self.route, self.ego, self.objects,
            lambda path, tolerance: True, body_shift_m=REAR_AXLE_SHIFT_M, **kwargs)

    def test_expired_glue_deadline_builds_no_new_candidate(self):
        with patch.object(ma._static, 'build_forced_bypass',
                          side_effect=AssertionError('expired candidate built')):
            plan, _, info = ma.plan_bypass(self.route, self.ego, self.objects,
                lambda path, tolerance: True, lambda path, horizon: horizon,
                deadline=time.perf_counter()-1., body_shift_m=REAR_AXLE_SHIFT_M)
        self.assertTrue(plan['blocked'])
        self.assertTrue(info['budget_exhausted'])
        self.assertNotIn('path', plan)

    def test_first_complete_checked_candidate_survives_expiration(self):
        with patch.object(ma._static, 'build_forced_bypass',
                          wraps=ma._static.build_forced_bypass) as build:
            plan, _ = self.portable(corridor_clearance=lambda path, horizon: horizon,
                search_expired=lambda: build.call_count >= 1)
        self.assertEqual(build.call_count, 1)
        self.assertTrue(ma.static_plan_complete(plan), plan)
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], self.ego,
            self.objects, 35., body_shift_m=REAR_AXLE_SHIFT_M), 35.)

    def test_partial_candidate_cannot_start_a_pass_when_search_expires(self):
        with patch.object(ma._static, 'build_forced_bypass',
                          wraps=ma._static.build_forced_bypass) as build:
            plan, _ = self.portable(corridor_clearance=lambda path, horizon: 12.,
                search_expired=lambda: build.call_count >= 1)
        self.assertEqual(build.call_count, 1)
        self.assertTrue(plan['blocked'], plan)
        self.assertNotIn('path', plan)

    def test_new_front_return_rejects_a_previously_checked_path(self):
        previous, _ = self.portable(corridor_clearance=lambda path, horizon: horizon)
        self.assertTrue(ma.static_plan_complete(previous), previous)
        front = np.array([[3.9, 0.]])
        with patch.object(ma._static, 'build_forced_bypass',
                          wraps=ma._static.build_forced_bypass) as build:
            plan, _ = self.portable(committed_direction=previous['direction'],
                previous_plan=previous, search_expired=lambda: build.call_count >= 1,
                corridor_clearance=lambda path, horizon: swept_clearance(
                    path, front, horizon, .25, lateral_margin=.25,
                    body_shift_m=REAR_AXLE_SHIFT_M))
        self.assertTrue(plan['blocked'], plan)
        self.assertNotIn('path', plan)

    def test_no_callback_preserves_direct_caller_result(self):
        original, _ = self.portable(corridor_clearance=lambda path, horizon: horizon)
        explicit, _ = self.portable(corridor_clearance=lambda path, horizon: horizon,
                                    search_expired=None)
        np.testing.assert_array_equal(original['path'], explicit['path'])
        self.assertEqual(original['direction'], explicit['direction'])
        self.assertEqual(original['clear_distance_m'], explicit['clear_distance_m'])

    def recorded_search(self, *, force_fallback=False):
        with np.load(Path(__file__).parent/'test_data/static_search_frame15_20261016.npz') as data:
            ego = json.loads(str(data['ego']))
            objects = json.loads(str(data['objects']))
            points, route = data['points'].copy(), data['route'].copy()
            corridor = LaneCorridor(route, data['left'], data['right'],
                zones=[np.asarray(p) for p in json.loads(str(data['zones']))],
                intersection_mask=data['intersection_mask'])
            offsets = ma.Offsets({int(k): v for k, v in
                                  json.loads(str(data['adjacent_offsets'])).items()})
            offsets.open_road_sides = tuple(offsets)
            offsets.start_m_by_side = {int(k): tuple(v) for k, v in
                                      json.loads(str(data['offset_starts'])).items()}
            now, stamp = float(data['scan_stamp']), float(data['object_stamp'])
        runtime = fixture.AvoidanceGateTests().runtime(action='KEEP', objects=objects, points=points)
        runtime._contact, runtime._lidar_log = None, dict(age_s=.1)
        runtime.object_receiver.observation_stamp = stamp
        runtime.model_road_offsets = lambda *args: offsets
        corridor.body_shift_m = REAR_AXLE_SHIFT_M
        lane, clear = ma.road_static_checks(corridor, points)
        clock, budgets = [100.], []
        original_budget, original_build = ma.Budget, ma._static.build_forced_bypass
        original_search, searches = ma.plan_bypass, []

        def budget(*args, **kwargs):
            # Measured geometry still runs normally. A deterministic 1 ms
            # charge per candidate makes budget handoff independent of CPU load.
            kwargs['clock'] = lambda: clock[0]
            value = original_budget(*args, **kwargs)
            budgets.append(value)
            return value

        def build(*args, **kwargs):
            clock[0] += .001
            return original_build(*args, **kwargs)

        def search(*args, **kwargs):
            searches.append(kwargs.get('deadline'))
            if force_fallback and len(searches) == 1:
                # The comfortable corridor is unavailable. The real hard
                # search must still receive the remainder of the same budget.
                args = list(args)
                args[4] = lambda path, horizon: 0.
            return original_search(*args, **kwargs)

        with patch.object(ma, 'Budget', side_effect=budget), \
                patch.object(ma._static, 'build_forced_bypass', side_effect=build), \
                patch.object(ma, 'plan_bypass', side_effect=search), \
                patch.object(gui.time, 'perf_counter', side_effect=lambda: clock[0]), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch('static_lane_pass.normal_static_area', return_value=True):
            plan, _, context = runtime.model_static_plan(ego, route, objects, points,
                corridor, lane, clear, None, {}, now, True, True, 1.)
        return plan, context, runtime, ego, points, budgets, clock[0], searches

    def assert_recorded_path_safe(self, plan, context, runtime, ego, points):
        self.assertTrue(ma.static_plan_complete(plan), runtime._model_avoidance_log)
        required = max(35., plan['safe_horizon_m'], plan['hold_until_m'])
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], ego,
            context['static'], required, body_shift_m=REAR_AXLE_SHIFT_M), required)
        self.assertGreaterEqual(swept_clearance(plan['path'], points, required, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), required)
        self.assertGreaterEqual(context['corridor'].clear_distance(plan['path'], required), required)
        self.assertLessEqual(max(gui.path_curvatures(plan['path'])), gui.MAX_CONTROL_CURVATURE)

    def test_recorded_17m_obstacle_can_finish_in_the_comfort_budget(self):
        plan, context, runtime, ego, points, budgets, clock, searches = self.recorded_search()
        self.assertEqual(len(budgets), 1)
        self.assertEqual(len(searches), 1)
        self.assertAlmostEqual(budgets[0].hard, 100.025)
        self.assertLessEqual(clock, 100.026)
        self.assert_recorded_path_safe(plan, context, runtime, ego, points)

    def test_recorded_17m_obstacle_keeps_fallback_inside_the_shared_budget(self):
        plan, context, runtime, ego, points, budgets, clock, searches = self.recorded_search(force_fallback=True)
        self.assertEqual(len(budgets), 3)  # preferred side, same-deadline retry, hard fallback
        self.assertEqual(len(searches), 2)
        self.assertAlmostEqual(budgets[0].hard, 100.025)
        self.assertAlmostEqual(budgets[1].hard, 100.025)
        self.assertAlmostEqual(budgets[2].hard, 100.11)
        self.assertGreaterEqual(clock, 100.025)
        self.assertLessEqual(clock, 100.111)
        self.assert_recorded_path_safe(plan, context, runtime, ego, points)


class PendingBoundedSearchTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        self.route = np.column_stack((np.arange(0., 65.5, .5), np.zeros(131)))
        self.corridor = LaneCorridor(self.route, np.full(131, 5.), np.full(131, -5.))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=12., wheel_angle=0.,
                        measured=True, stamp=100., age_s=.01)
        self.previous = dict(path=self.route.copy(), direction=-1., offset_m=.6,
            transition_m=18., hold_m=20., hold_until_m=25., kind='bypass',
            curvature=0., safe_horizon_m=4., clear_distance_m=60.,
            bounded_static_pass=True, static_borrow=False, canonical_control=True)

    def test_pending_release_requires_a_current_measured_legal_body_without_changing_policy(self):
        policy = StaticLanePass()
        before = (policy.entered_at, policy.return_station_m, policy.legal, policy.last_stamp)
        self.assertTrue(ma.pending_bounded_search_release(self.previous, self.ego, policy, self.corridor))
        self.assertEqual((policy.entered_at, policy.return_station_m, policy.legal, policy.last_stamp), before)
        for ego in (dict(self.ego, age_s=.30001), dict(self.ego, age_s=-.01),
                    dict(self.ego, measured=False), dict(self.ego, stamp=float('nan')),
                    dict(self.ego, x=float('inf'))):
            self.assertFalse(ma.pending_bounded_search_release(self.previous, ego, policy, self.corridor))
        for changes in (dict(static_borrow=True), dict(bounded_static_pass=False),
                        dict(standing_bypass_ids=[7]), dict(direction=0.)):
            self.assertFalse(ma.pending_bounded_search_release(dict(self.previous, **changes),
                self.ego, policy, self.corridor))
        policy.entered_at = 0.
        self.assertFalse(ma.pending_bounded_search_release(self.previous, self.ego, policy, self.corridor))
        policy.entered_at = None
        for corridor in (None, LaneCorridor(self.route, np.full(131, .6), np.full(131, -.6))):
            self.assertFalse(ma.pending_bounded_search_release(self.previous, self.ego, policy, corridor))
        invalid = LaneCorridor(self.route, np.full(131, 5.), np.full(131, -5.))
        invalid.left[0] = np.nan
        self.assertFalse(ma.pending_bounded_search_release(self.previous, self.ego, policy, invalid))
        self.assertFalse(ma.pending_bounded_search_release(self.previous, self.ego, None, self.corridor))

    def host_search(self, *, direction=-1., safe=False, borrow=False, entered=None, inside=True, normal=True):
        objects = [(20., -1.2, .8, 'box', 'obstacle', 0., .8, .8, 0.)]
        points = np.array([[20., -1.2]])
        runtime = fixture.AvoidanceGateTests().runtime(action='KEEP', objects=objects, points=points)
        runtime._contact, runtime._lidar_log = None, dict(age_s=.01)
        runtime.model_road_offsets = lambda *args: {}
        policy = runtime._static_lane_pass = StaticLanePass()
        policy.entered_at, policy.return_station_m = entered, 31.
        policy.plan = Mock(return_value=(None, {}))
        policy.release_unused = Mock(wraps=policy.release_unused)
        corridor = self.corridor if inside else LaneCorridor(self.route, np.full(131, .6), np.full(131, -.6))
        lane, clearance = ma.road_static_checks(corridor, points)
        previous = dict(self.previous, direction=direction, static_borrow=borrow)
        if safe:
            path = self.route.copy()
            path[:, 1] = 2.*(1.-np.exp(-path[:, 0]/10.))
            previous['path'] = path
        calls = []
        def search(*args, **kwargs):
            calls.append(dict(direction=args[5], previous=args[6], deadline=kwargs.get('deadline')))
            # A search whose current full guards find no candidate must stop.
            return dict(blocked=True, kind='bypass'), [{'front': 18., 'lat': -1.2}], {}
        with patch.object(gui, 'model_avoidance', ma, create=True), \
                patch('static_lane_pass.normal_static_area', return_value=normal), \
                patch.object(ma, 'manoeuvre_complete', return_value=False), \
                patch.object(ma, 'road_rejoin_plan', return_value=None), \
                patch.object(ma, 'contact_static_pass', return_value=None), \
                patch.object(ma, 'plan_bypass', side_effect=search):
            plan, _, _ = runtime.model_static_plan(self.ego, self.route, objects, points,
                corridor, lane, clearance, previous, {}, 100., True, True, 1.)
        self.assertTrue(plan['blocked'])
        policy.release_unused.assert_not_called()
        self.assertEqual(policy.entered_at, entered)
        self.assertEqual(policy.return_station_m, 31.)
        return calls, previous, runtime._model_avoidance_log

    def test_failed_pending_bounded_searches_the_opposite_side_with_no_old_commitment(self):
        for direction in (-1., 1.):
            calls, previous, info = self.host_search(direction=direction)
            self.assertTrue(info['released_pending_bounded'])
            self.assertTrue(calls)
            self.assertTrue(all(c['previous'] is None and c['direction'] == -direction for c in calls))
            self.assertEqual(previous['direction'], direction)

    def test_current_full_safe_entered_borrow_outside_and_tunnel_scopes_keep_the_old_search(self):
        for changes in (dict(safe=True), dict(entered=0.), dict(borrow=True),
                        dict(inside=False), dict(normal=False)):
            with self.subTest(changes=changes):
                calls, previous, info = self.host_search(**changes)
                self.assertFalse(info.get('released_pending_bounded', False))
                self.assertTrue(calls)
                self.assertTrue(all(c['previous'] is previous and c['direction'] == -1. for c in calls))


if __name__ == '__main__':
    unittest.main()
