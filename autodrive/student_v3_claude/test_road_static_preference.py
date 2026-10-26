"""Current road observations retain a checked fallback while ranking full passes."""
import copy
import json
import math
from pathlib import Path
from unittest.mock import patch
import unittest

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from perception.stationary_semantics import metadata, with_metadata
from static_lane_pass import measured_initial_join
from tunnel_runtime import swept_clearance


class RecordedRoadPreferenceTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        with np.load(Path(__file__).parent/'test_data/road_static_preference_20261017.npz') as data:
            self.data = {key: data[key].copy() for key in data.files}
        with np.load(Path(__file__).parent/'test_data/road_static_return_hold_20261019.npz') as data:
            self.data.update({key: data[key].copy() for key in data.files})
        with np.load(Path(__file__).parent/'test_data/road_static_return_fallback_20261019.npz') as data:
            self.data.update({key: data[key].copy() for key in data.files})
        with np.load(Path(__file__).parent/'test_data/road_static_transient_hazard_20261019.npz') as data:
            self.data.update({key: data[key].copy() for key in data.files})

    def frame(self, seq):
        prefix = f'frame{seq}_'
        ego, objects, previous = (json.loads(str(self.data[prefix+key]))
                                  for key in ('ego', 'static', 'previous'))
        route, points = self.data[prefix+'route'][:, :2], self.data[prefix+'points']
        corridor = LaneCorridor(**{key: self.data[prefix+'corridor_'+key]
            for key in ('route', 'left', 'right', 'intersection_mask', 'body_shift_m')},
            zones=json.loads(str(self.data[prefix+'zones'])))
        soft = ma.road_comfort_objects(objects)
        margin = float(self.data.get(prefix+'clearance_margin_m', .45))
        checked_objects = soft if margin > ma.ROAD_STATIC_MARGIN_M else objects
        _, raw = ma.road_static_checks(corridor, points, margin_m=margin)
        cache = {}
        continued = np.asarray(previous['path'], float)[:, :2]
        checked = measured_initial_join(continued, ego,
            max_curvature=gui.MAX_CONTROL_CURVATURE, lateral_accel=gui.BYPASS_ALAT_MPS2)
        cache[continued.tobytes()] = checked
        cache[checked.tobytes()] = checked

        def canonical(path):
            path = np.asarray(path, float)[:, :2]
            key = path.tobytes()
            if key not in cache:
                ahead = np.flatnonzero(path[:, 0] > .05)
                joined = np.vstack((np.zeros((1, 2)), path[ahead[0]:]))
                checked = gui.RosPathPublisher.control_geometry(joined, route,
                    dict(mode='LATTICE', validated_static=True))
                checked = measured_initial_join(checked, ego,
                    max_curvature=gui.MAX_CONTROL_CURVATURE, lateral_accel=gui.BYPASS_ALAT_MPS2)
                cache[key] = checked
                cache[checked.tobytes()] = checked
            return cache[key]

        def clear(path, horizon):
            path = canonical(path)
            if max(gui.path_curvatures(path), default=math.inf) > gui.MAX_CONTROL_CURVATURE:
                return 0.
            return min(raw(path, horizon), gui.static_clear_distance(path, ego, checked_objects,
                horizon, body_shift_m=REAR_AXLE_SHIFT_M))
        return ego, objects, soft, previous, route, points, corridor, canonical, clear

    def assert_complete(self, plan, frame):
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        self.assertTrue(ma.static_plan_complete(plan), plan)
        path, horizon = plan['path'], plan['clear_distance_m']
        self.assertGreaterEqual(clear(path, horizon), horizon-.01)
        self.assertGreaterEqual(corridor.clear_distance(path, horizon), horizon-.01)
        self.assertGreaterEqual(swept_clearance(path, points, horizon, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(gui.static_clear_distance(path, ego, objects, horizon,
            body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertLessEqual(max(gui.path_curvatures(path)), gui.MAX_CONTROL_CURVATURE)
        self.assertIsNotNone(ma._road_steering_limit(path, ego, ego['speed_kmh']/3.6))
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        speed = ego['speed_kmh']/3.6
        lateral = np.maximum(0., speed*speed-3.*np.maximum(
            0., arc[1:-1]-speed*.7))*gui.path_curvatures(path)
        self.assertTrue(np.all(lateral <= gui.BYPASS_ALAT_MPS2+.05))
        if plan.get('preferred_raw_gap_m') is not None:
            preferred = ma._road_rejoin_preference_objects(soft, ego, route)
            self.assertGreaterEqual(gui.static_clear_distance(path, ego, preferred, horizon,
                body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)

    def test_existing_first_sparse_safe_candidate_survives_expiry_after_its_full_check(self):
        frame = self.frame(71)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        # Exercise fresh admission independently of the checked-return priority.
        previous = dict(previous, road_return=False)
        clock = [1.]
        score = ma._road_sparse_gap
        def expires_after_score(*args):
            gap = score(*args)
            clock[0] = 2.
            return gap
        with patch('model_avoidance.time.perf_counter', side_effect=lambda: clock[0]), \
                patch.object(ma, '_road_sparse_gap', side_effect=expires_after_score):
            plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, 1.025)
        self.assertEqual((plan['offset_m'], plan['transition_m']), (.55, 18.))
        self.assertGreater(plan['preferred_raw_gap_m'], .09)
        self.assert_complete(plan, frame)

    def test_expiry_after_the_first_complete_hard_fallback_keeps_its_full_proof(self):
        frame = self.frame(71)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        # Exercise fresh admission independently of the checked-return priority.
        previous = dict(previous, road_return=False)
        clock = [1.]
        def expires_after_complete(path, horizon):
            result = clear(path, horizon)
            if result >= horizon-.1:
                clock[0] = 2.
            return result
        with patch('model_avoidance.time.perf_counter', side_effect=lambda: clock[0]):
            plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical,
                expires_after_complete, 1.025)
        self.assertNotIn('preferred_raw_gap_m', plan)
        self.assert_complete(plan, frame)

    def test_deferred_lower_curvature_fallback_requires_a_new_complete_host_check(self):
        frame = self.frame(71)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        # Exercise fresh admission independently of the checked-return priority.
        previous = dict(previous, road_return=False)
        with patch('model_avoidance.time.perf_counter', return_value=1.), \
                patch.object(ma, '_road_sparse_gap', return_value=None):
            plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, 1.025)
        self.assertEqual((plan['offset_m'], plan['transition_m']), (.5, 22.))
        self.assert_complete(plan, frame)
        with patch('model_avoidance.time.perf_counter', return_value=1.):
            rejected = ma.road_rejoin_plan(route, ego, soft, previous, canonical,
                lambda path, horizon: 0., 1.025)
        self.assertIsNone(rejected)

    def test_new_current_hazard_after_a_zero_offset_return_gets_an_existing_full_pass(self):
        for seq, expected in ((265, (.6, 26.)), (267, (.6, 22.))):
            frame = self.frame(seq)
            ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
            original = copy.deepcopy(objects)
            self.assertEqual((previous['kind'], previous['offset_m']), ('rejoin', 0.))
            with patch('model_avoidance.time.perf_counter', return_value=1.):
                plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, 1.025)
            self.assertEqual((plan['offset_m'], plan['transition_m']), expected)
            self.assertGreater(plan['preferred_raw_gap_m'], .06)
            self.assertEqual(objects, original)
            self.assert_complete(plan, frame)

    def test_currently_comfortable_world_return_precedes_a_narrower_fresh_fallback(self):
        frame = self.frame(46)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        original = copy.deepcopy(previous)
        with patch.object(ma._geometry, 'build_forced_bypass',
                side_effect=AssertionError('checked comfortable return was discarded')):
            plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, math.inf)
        self.assertTrue(plan['kept_comfortable_return'])
        self.assertTrue(plan['reused'])
        self.assertEqual((plan['offset_m'], plan['transition_m']), (.6, 18.))
        self.assertGreater(plan['preferred_raw_gap_m'], .09)
        np.testing.assert_array_equal(previous['path'], original['path'])
        self.assert_complete(plan, frame)

    def test_new_far_raw_block_and_changed_near_box_reject_the_old_return(self):
        for seq in (52, 54):
            frame = self.frame(seq)
            ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
            with patch('model_avoidance.time.perf_counter', return_value=1.), \
                    patch.object(ma._geometry, 'build_forced_bypass',
                        wraps=ma._geometry.build_forced_bypass) as build:
                plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, 1.025)
            self.assertGreater(build.call_count, 0)
            self.assertIsNone(plan)

    def test_failed_source_preference_keeps_the_currently_proven_return_as_fallback(self):
        ego, objects, soft, previous, route, points, corridor, canonical, clear = self.frame(46)
        with patch('model_avoidance.time.perf_counter', return_value=1.), \
                patch.object(ma._static, 'static_clear_distance', return_value=0.), \
                patch.object(ma._geometry, 'build_forced_bypass',
                    wraps=ma._geometry.build_forced_bypass) as build:
            plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, 1.025)
        self.assertGreater(build.call_count, 0)
        self.assertTrue(ma.static_plan_complete(plan), plan)
        self.assertFalse(plan.get('kept_comfortable_return', False))
        self.assertTrue(plan['kept_hard_return'])
        self.assertTrue(plan['reused'])
        self.assertIsNone(plan['preferred_raw_gap_m'])
        self.assert_complete(plan, self.frame(46))

    def test_recorded_source_shift_cannot_replace_a_safe_return_with_a_thinner_fallback(self):
        frame = self.frame(126)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        bounds = ma._road_rejoin_preference_objects(soft, ego, route, include_near=True)
        path = canonical(previous['path'])
        horizon = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        self.assertLess(gui.static_clear_distance(path, ego, bounds, horizon,
            body_shift_m=REAR_AXLE_SHIFT_M), 8.)
        self.assertGreater(clear(path, horizon), 44.)
        original = copy.deepcopy(previous)
        with patch('model_avoidance.time.perf_counter', return_value=1.), \
                patch.object(ma, '_road_sparse_gap', return_value=None), \
                patch.object(ma._geometry, 'build_forced_bypass',
                    wraps=ma._geometry.build_forced_bypass) as build:
            plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, 1.025)
        self.assertGreater(build.call_count, 0)
        self.assertEqual((plan['offset_m'], plan['transition_m']), (.55, 26.))
        self.assertTrue(plan['kept_hard_return'])
        self.assertFalse(plan['kept_comfortable_return'])
        self.assertIsNone(plan['preferred_raw_gap_m'])
        np.testing.assert_array_equal(previous['path'], original['path'])
        self.assert_complete(plan, frame)

    def test_a_new_fully_preferred_candidate_still_precedes_the_safe_hard_return(self):
        frame = self.frame(126)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        with patch('model_avoidance.time.perf_counter', return_value=1.):
            plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, 1.025)
        self.assertEqual((plan['offset_m'], plan['transition_m']), (.35, 18.))
        self.assertGreater(plan['preferred_raw_gap_m'], 0.)
        self.assertFalse(plan.get('kept_hard_return', False))
        self.assert_complete(plan, frame)

    def test_expiry_after_the_previous_return_full_check_keeps_only_that_complete_proof(self):
        frame = self.frame(126)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        clock = [1.]
        def expires_after_current_check(path, horizon):
            result = clear(path, horizon)
            clock[0] = 2.
            return result
        with patch('model_avoidance.time.perf_counter', side_effect=lambda: clock[0]), \
                patch.object(ma._geometry, 'build_forced_bypass',
                    side_effect=AssertionError('search continued after budget expiry')):
            plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical,
                expires_after_current_check, 1.025)
        self.assertTrue(plan['kept_hard_return'])
        self.assert_complete(plan, frame)

    def test_near_source_failure_without_a_fresh_preference_keeps_the_safe_return(self):
        frame = self.frame(126)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        preference = ma._road_rejoin_preference_objects
        def near_only(*args, **kwargs):
            return preference(*args, **kwargs) if kwargs.get('include_near') else []
        with patch.object(ma, '_road_rejoin_preference_objects', side_effect=near_only), \
                patch.object(ma._geometry, 'build_forced_bypass',
                    side_effect=AssertionError('no preferred family can improve this hard return')):
            plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, math.inf)
        self.assertTrue(plan['kept_hard_return'])
        self.assert_complete(plan, frame)

    def test_changed_current_physical_box_and_failed_host_check_reject_a_hard_return(self):
        for seq in (126, 130):
            frame = self.frame(seq)
            ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
            previous = dict(previous, road_return=True)
            if seq == 126:
                clear = lambda path, horizon: 0.
            with patch('model_avoidance.time.perf_counter', return_value=1.):
                plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, clear, 1.025)
            self.assertIsNone(plan)

    def test_steering_and_lateral_acceleration_still_reject_the_existing_return(self):
        ego, objects, soft, previous, route, points, corridor, canonical, clear = self.frame(46)
        for target, value in (('model_avoidance._road_steering_limit', None),
                              ('gui.BYPASS_ALAT_MPS2', 0.)):
            context = patch(target, return_value=value) if value is None else patch(target, value)
            with context:
                self.assertIsNone(ma.road_rejoin_plan(route, ego, soft, previous,
                    canonical, clear, math.inf))

    def test_a_complete_bounded_in_lane_curve_survives_fresh_search_failure(self):
        frame = self.frame(46)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        previous = dict(previous, bounded_static_pass=True, fixed_world_curve=True,
                        static_borrow=False)
        previous.pop('canonical_control', None)
        plan = ma.road_checked_continuation(previous, ego, canonical, clear)
        self.assertIsNotNone(plan)
        self.assertTrue(plan['kept_checked_static'])
        np.testing.assert_array_equal(plan['path'], canonical(previous['path']))
        self.assert_complete(plan, frame)
        self.assertIsNone(ma.road_checked_continuation(previous, ego, canonical,
            lambda path, horizon: 0.))
        self.assertIsNone(ma.road_checked_continuation(dict(previous, static_borrow=True),
            ego, canonical, clear))

    def test_zero_offset_return_without_a_new_hazard_still_skips_search(self):
        ego, objects, soft, previous, route, points, corridor, canonical, clear = self.frame(265)
        previous = dict(previous, path=route)
        self.assertTrue(ma.manoeuvre_complete(previous, route))
        with patch.object(ma._geometry, 'build_forced_bypass',
                side_effect=AssertionError('unneeded return search')):
            self.assertIsNone(ma.road_rejoin_plan(route, ego, [], previous, canonical,
                clear, math.inf))

    def test_zero_offset_label_does_not_discard_an_unfinished_checked_world_return(self):
        frame = self.frame(44)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        self.assertEqual((previous['kind'], previous['offset_m']), ('rejoin', 0.))
        self.assertFalse(ma.manoeuvre_complete(previous, route))
        # The centre-only trigger has expired, but the measured front corner
        # still needs the unfinished return to pass this body safely.
        self.assertIsNone(ma._static.route_hazard(objects, ego, route, ahead_m=65.))
        self.assertTrue(ma.confirmed_hazards(route, ego, objects,
            ma.footprint_hazards(route, ego, objects,
                body_shift_m=REAR_AXLE_SHIFT_M))[0])
        with patch.object(ma._geometry, 'build_forced_bypass',
                side_effect=AssertionError('unfinished zero-offset return was discarded')):
            plan = ma.road_rejoin_plan(route, ego, objects, previous, canonical, clear, math.inf)
        self.assertTrue(plan['kept_hard_return'])
        np.testing.assert_array_equal(plan['path'], canonical(previous['path']))
        self.assert_complete(plan, frame)
        # With no new observation, a legacy non-road return still leaves this
        # special search alone. A current measured body remains actionable.
        with patch.object(ma._geometry, 'build_forced_bypass',
                side_effect=AssertionError('non-road return entered road search')):
            self.assertIsNone(ma.road_rejoin_plan(route, ego, [],
                dict(previous, road_return=False), canonical, clear, math.inf))

    def test_transient_hazard_absence_keeps_the_full_checked_unfinished_return(self):
        # Recorded current inputs from run 148869; no later observations enter
        # the check. The old planner selected a new zero-offset path in both.
        for seq in (43, 106):
            frame = self.frame(seq)
            ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
            # Simulate a transient missing semantic observation. The clearance
            # closure and assert_complete still check the recorded bodies and
            # raw scan, so absence cannot waive the complete path check.
            absent = []
            self.assertEqual(ma.confirmed_hazards(route, ego, absent,
                ma.footprint_hazards(route, ego, absent,
                    body_shift_m=REAR_AXLE_SHIFT_M))[0], [])
            self.assertFalse(ma.manoeuvre_complete(previous, route))
            recorded = json.loads(str(self.data[f'frame{seq}_recorded_plan']))
            self.assertEqual(recorded['offset_m'], 0.)
            original = copy.deepcopy(previous)
            with patch.object(ma._geometry, 'build_forced_bypass',
                    side_effect=AssertionError('unfinished checked return was discarded')):
                plan = ma.road_rejoin_plan(route, ego, absent, previous,
                    canonical, clear, math.inf)
            self.assertTrue(plan['kept_hard_return'])
            self.assertTrue(plan['reused'])
            self.assertEqual(plan['offset_m'], previous['offset_m'])
            np.testing.assert_array_equal(plan['path'], canonical(previous['path']))
            np.testing.assert_array_equal(previous['path'], original['path'])
            self.assert_complete(plan, frame)

    def test_hazard_absence_never_keeps_a_return_blocked_beyond_the_stopping_horizon(self):
        frame = self.frame(32)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        path = canonical(previous['path'])
        horizon = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        self.assertGreater(clear(path, horizon), previous['safe_horizon_m'])
        self.assertLess(clear(path, horizon), horizon-.1)
        with patch('model_avoidance.time.perf_counter', return_value=1.), \
                patch.object(ma._geometry, 'build_forced_bypass',
                    wraps=ma._geometry.build_forced_bypass) as build:
            plan = ma.road_rejoin_plan(route, ego, objects, previous, canonical, clear, 1.025)
        self.assertGreater(build.call_count, 0)
        self.assertFalse(plan and plan.get('kept_hard_return'))
        self.assertFalse(plan and plan.get('kept_comfortable_return'))

    def test_completed_or_uncommitted_returns_still_search_without_a_current_hazard(self):
        frame = self.frame(43)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        for prior in (dict(previous, path=route), dict(previous, road_return=False)):
            if prior.get('road_return'):
                self.assertTrue(ma.manoeuvre_complete(prior, route))
            with patch('model_avoidance.time.perf_counter', return_value=1.), \
                    patch.object(ma._geometry, 'build_forced_bypass',
                        wraps=ma._geometry.build_forced_bypass) as build:
                plan = ma.road_rejoin_plan(route, ego, objects, prior, canonical, clear, 1.025)
            self.assertGreater(build.call_count, 0)
            self.assertFalse(plan and plan.get('kept_hard_return'))
            self.assertFalse(plan and plan.get('kept_comfortable_return'))
        for key in ('bounded_static_pass', 'static_borrow'):
            with patch.object(ma._geometry, 'build_forced_bypass',
                    side_effect=AssertionError('excluded scope entered road search')):
                self.assertIsNone(ma.road_rejoin_plan(route, ego, objects,
                    dict(previous, **{key: True}), canonical, clear, math.inf))

    def test_transient_hazard_absence_preserves_every_current_admission_guard(self):
        frame = self.frame(43)
        ego, objects, soft, previous, route, points, corridor, canonical, clear = frame
        for target, value in (('model_avoidance._road_steering_limit', None),
                              ('gui.BYPASS_ALAT_MPS2', 0.)):
            context = patch(target, return_value=value) if value is None else patch(target, value)
            with context, patch.object(ma._geometry, 'build_forced_bypass',
                    return_value=(None, None, None)):
                self.assertIsNone(ma.road_rejoin_plan(route, ego, objects, previous,
                    canonical, clear, math.inf))
        with patch.object(ma._geometry, 'build_forced_bypass', return_value=(None, None, None)):
            self.assertIsNone(ma.road_rejoin_plan(route, ego, objects, previous,
                canonical, lambda path, horizon: 0., math.inf))
            self.assertIsNone(ma.road_rejoin_plan(route, ego, objects, previous,
                canonical, clear, 0.))


class VerifiedFusionPreferenceTests(unittest.TestCase):
    def setUp(self):
        self.ego = dict(x=0., y=0., heading_deg=0.)
        self.route = np.column_stack((np.arange(0., 70., .5), np.zeros(140)))
        self.obj = with_metadata((18., 0., .9, 'fusion_obstacle', 'obstacle', 0.,
            .9, .9, 0., 0., 0., 9, 0., None, None, 0., None), road_static_geometry=dict(
            version=1, source='semantic_fusion', source_size_m=[1.1, 1.1],
            source_padding_m=.3, applied_padding_m=.1, physical_size_m=[.8, .8],
            observed_points=2))

    def preference(self, obj=None, route=None):
        return ma._road_rejoin_preference_objects([self.obj if obj is None else obj], self.ego,
            self.route if route is None else route)

    def test_current_verified_fusion_source_is_only_a_preference_copy(self):
        original = copy.deepcopy(self.obj)
        preferred, = self.preference()
        self.assertEqual(preferred[6:8], (1.1, 1.1))
        self.assertEqual(self.obj, original)
        self.assertEqual(ma._road_sparse_front_objects([self.obj], self.ego, self.route), [])

    def test_near_source_bounds_protect_only_continuation_and_keep_raw_front_limits(self):
        row = list(self.obj)
        row[0] = 5.
        original = copy.deepcopy(tuple(row))
        self.assertEqual(self.preference(tuple(row)), [])
        preferred, = ma._road_rejoin_preference_objects([tuple(row)], self.ego,
            self.route, include_near=True)
        self.assertEqual(preferred[6:8], (1.1, 1.1))
        self.assertEqual(tuple(row), original)
        raw, = ma.road_lidar_objects(np.array([[5., 0.]]), self.ego)
        self.assertEqual(ma._road_rejoin_preference_objects([raw], self.ego,
            self.route, include_near=True), [])

    def test_unknown_changed_or_unverified_source_never_gets_the_preference(self):
        geometry = metadata(self.obj)['road_static_geometry']
        cases = [with_metadata(self.obj, road_static_geometry=dict(geometry, **change))
            for change in (dict(version=2), dict(source='unknown'), dict(source_padding_m=.2),
                dict(applied_padding_m=.2), dict(physical_size_m=[.7, .8]),
                dict(source_size_m=[.8, .8]), dict(observed_points=0),
                dict(observed_points=float('nan')), dict(source_size_m=[7., 1.1]))]
        cases.append(self.obj[:17])
        cases.append(with_metadata(self.obj, road_static_geometry=['unverified']))
        for obj in cases:
            with self.subTest(obj=obj):
                self.assertEqual(self.preference(obj), [])

    def test_source_preference_has_full_sat_margin_without_changing_hard_recovery(self):
        row = list(self.obj)
        row[0], row[1] = 1., 1.66
        geometry = dict(metadata(row)['road_static_geometry'],
            initial_margin_floor_m=.15, initial_margin_restore_start_m=4.)
        hard = with_metadata(row, road_static_geometry=geometry)
        original = copy.deepcopy(hard)
        preferred, = ma._road_rejoin_preference_objects([hard], self.ego,
            self.route, include_near=True)
        checked_geometry = metadata(preferred)['road_static_geometry']
        self.assertNotIn('initial_margin_floor_m', checked_geometry)
        self.assertNotIn('initial_margin_restore_start_m', checked_geometry)
        self.assertEqual(hard, original)
        path = np.column_stack((np.arange(0., 40.1, .5), np.zeros(81)))
        restored_floor = with_metadata(preferred, road_static_geometry=geometry)
        self.assertEqual(gui.static_clear_distance(path, self.ego, [restored_floor], 35.,
            body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        self.assertEqual(gui.static_clear_distance(path, self.ego, [preferred], 35.,
            body_shift_m=REAR_AXLE_SHIFT_M), 0.)

    def test_people_motion_long_walls_near_behind_and_off_route_are_excluded(self):
        for fields in ({4: 'pedestrian'}, {5: .2}, {5: float('nan')},
                       {0: 7.99}, {0: 45.01}, {0: -2.}, {1: 3.01},
                       {3: 'fusion_parked_vehicle'}, {6: 7.}, {7: float('nan')}):
            row = list(self.obj)
            for index, value in fields.items():
                row[index] = value
            with self.subTest(fields=fields):
                self.assertEqual(self.preference(tuple(row)), [])
        for route in ([], np.zeros((1, 2)), np.full((3, 2), np.nan), [0., 1.]):
            with self.subTest(route=route):
                self.assertEqual(self.preference(route=route), [])


if __name__ == '__main__':
    unittest.main()
