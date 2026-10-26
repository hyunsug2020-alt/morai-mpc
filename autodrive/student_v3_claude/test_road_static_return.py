"""Ordinary-road bypass completion must return instead of carrying its offset."""
import copy
import json
from pathlib import Path
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from tunnel_runtime import swept_clearance
import test_avoidance_gate as fixtures


class RoadStaticReturnTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        self.route = np.column_stack((np.arange(0., 80.1, .5), np.full(161, .9)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=18., stamp=100., wheel_angle=0.)
        self.objects = [(-6., 2., .4, 'passed barrel', 'obstacle', 0., .4, .4, 0.)]
        self.points = np.array([[-6., 2.]])
        self.corridor = LaneCorridor(self.route, np.full(161, 3.5), np.full(161, -3.5),
                                    body_shift_m=REAR_AXLE_SHIFT_M)
        self.previous = dict(path=np.column_stack((np.arange(0., 60.1, .5), np.zeros(121))),
            kind='bypass', direction=-1., offset_m=1.2, transition_m=14., hold_m=30.,
            safe_horizon_m=4., clear_distance_m=60., hold_until_m=25., speed_cap_mps=5.,
            canonical_control=True)

    def runtime(self):
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=self.objects, points=self.points)
        runtime._contact = None
        return runtime

    def plan(self, runtime=None, points=None, scope=True):
        runtime = runtime or self.runtime()
        points = self.points if points is None else points
        lane, clear = ma.road_static_checks(self.corridor, points)
        with patch('static_lane_pass.normal_static_area', return_value=scope), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            result, _, context = runtime.model_static_plan(self.ego, self.route, self.objects, points,
                self.corridor, lane, clear, self.previous, {}, 100., True, True, 1.)
        return result, context

    def test_rejoin_label_does_not_keep_the_old_offset_after_the_barrel_is_clear(self):
        with patch.object(ma, 'road_rejoin_plan', return_value=None):
            old, _ = self.plan()
        self.assertEqual(old['kind'], 'rejoin')
        self.assertLess(np.max(np.abs(old['path'][:, 1])), .01)
        new, context = self.plan()
        self.assertTrue(new['road_return'])
        self.assertEqual(new['kind'], 'rejoin')
        self.assertTrue(ma.static_plan_complete(new))
        self.assertGreater(new['speed_cap_mps'], self.previous['speed_cap_mps'])
        self.assertLessEqual(new['speed_cap_mps'], 30./3.6)
        self.assertEqual(new['ordinary_speed_cap_mps'], 25./3.6)
        self.assertGreater(new['path'][np.argmin(abs(new['path'][:, 0]-20.)), 1], .85)
        self.assertAlmostEqual(new['path'][-1, 1], .9, places=4)
        self.assertGreaterEqual(swept_clearance(new['path'], self.points, 35., .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        self.assertGreaterEqual(context['corridor'].clear_distance(new['path'], 35.), 35.)
        self.assertLessEqual(max(gui.path_curvatures(new['path'])), gui.MAX_CONTROL_CURVATURE)

    def test_a_front_return_still_prevents_a_positive_rejoin(self):
        points = np.vstack((self.points, [[3.9, 0.]]))
        _, clear = ma.road_static_checks(self.corridor, points)
        def checked(path, horizon):
            return min(clear(path, horizon), gui.static_clear_distance(path, self.ego,
                self.objects, horizon, body_shift_m=REAR_AXLE_SHIFT_M))
        result = ma.road_rejoin_plan(self.route, self.ego, self.objects, self.previous,
            lambda p: p[:, :2], checked, time.perf_counter()+1.)
        self.assertIsNone(result)

    def test_transient_low_cap_does_not_latch_a_clear_fresh_or_cached_return_at_crawl(self):
        for cached in (False, True):
            with self.subTest(cached=cached):
                self.previous.update(speed_cap_mps=1.2/3.6, road_return=cached)
                plan, context = self.plan()
                self.assertTrue(ma.static_plan_complete(plan), plan)
                self.assertGreater(plan['speed_cap_mps']*3.6, 20.)
                self.assertLessEqual(plan['speed_cap_mps'], 30./3.6)
                self.assertEqual(plan['ordinary_speed_cap_mps'], 25./3.6)
                self.assertIsNotNone(ma._road_steering_limit(
                    plan['path'], self.ego, self.ego['speed_kmh']/3.6))
                self.assertGreaterEqual(context['clearance'](plan['path'], 35.), 35.)

    def test_cached_fallback_rechecks_the_entire_current_join_and_its_curvature(self):
        previous = dict(self.previous, road_return=True)
        horizons = []
        def clear(path, horizon):
            horizons.append(horizon)
            return horizon
        with patch.object(ma._geometry, 'build_forced_bypass', return_value=(None, 0., 0.)):
            result = ma.road_rejoin_plan(self.route, self.ego, self.objects, previous,
                lambda p: p[:, :2], clear, time.perf_counter()+1.)
            self.assertTrue(result['cached_return_fallback'])
            self.assertTrue(result['reused'])
            self.assertEqual(horizons, [60.])
            bent = dict(previous, path=previous['path'].copy())
            bent['path'][1, 1] = 1.
            self.assertIsNone(ma.road_rejoin_plan(self.route, self.ego, self.objects, bent,
                lambda p: p[:, :2], clear, time.perf_counter()+1.))
        self.assertEqual(horizons, [60.])

    def test_a_clear_join_cannot_instantly_change_the_measured_steering(self):
        ego = dict(self.ego, wheel_angle=15.)
        previous = dict(self.previous, road_return=True)
        result = ma.road_rejoin_plan(self.route, ego, self.objects, previous,
            lambda p: p[:, :2], lambda path, horizon: horizon, time.perf_counter()+1.)
        self.assertIsNone(result)

    def test_a_blocked_right_commit_can_select_a_complete_left_pass_while_rolling(self):
        self.objects = [(14., 0., .8, 'next barrel', 'obstacle', 0., .8, .8, 0.)]
        self.points = np.array([[13.8, -.2], [14., 0.], [14.2, .2]])
        runtime = self.runtime()
        plan, context = self.plan(runtime)
        self.assertTrue(runtime._model_avoidance_log['released_static_side'])
        self.assertTrue(ma.static_plan_complete(plan), (plan, runtime._model_avoidance_log))
        self.assertEqual(plan['direction'], 1.)
        self.assertGreaterEqual(swept_clearance(plan['path'], self.points, 35., .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], self.ego, context['static'],
            35., body_shift_m=REAR_AXLE_SHIFT_M), 35.)

    def test_tunnel_portal_highway_and_people_do_not_use_the_ordinary_return_search(self):
        for change in ('tunnel', 'excluded_area', 'person'):
            self.setUp()
            runtime = self.runtime()
            if change == 'tunnel':
                runtime.tunnel = SimpleNamespace(active=lambda ego: True)
            elif change == 'person':
                self.objects.append((20., 2., .6, 'person', 'pedestrian', 1., .6, .6, 0.))
                runtime.object_receiver.objects = self.objects
            with self.subTest(change=change), patch.object(ma, 'road_rejoin_plan',
                    side_effect=AssertionError('ordinary return used outside its scope')):
                self.plan(runtime, scope=change != 'excluded_area')


class RecordedRoadReturnGapTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        with np.load(Path(__file__).parent/'test_data/road_static_rejoin_20261016.npz') as data:
            self.data = {key: data[key].copy() for key in data.files}

    def frame(self, index):
        key = f'frame{index}_'
        ego = json.loads(str(self.data[key+'ego']))
        objects = json.loads(str(self.data[key+'objects']))
        points, route = self.data[key+'points'], self.data[key+'route']
        corridor = LaneCorridor(route, self.data[key+'left'], self.data[key+'right'],
                                body_shift_m=REAR_AXLE_SHIFT_M)
        soft = ma.road_comfort_objects(objects)
        _, raw = ma.road_static_checks(corridor, points, margin_m=.45)
        def canonical(path):
            path = np.asarray(path, float)[:, :2]
            ahead = np.flatnonzero(path[:, 0] > .05)
            checked = gui.RosPathPublisher.control_geometry(
                np.vstack((np.zeros((1, 2)), path[ahead[0]:])), route,
                dict(mode='LATTICE', validated_static=True))
            from static_lane_pass import measured_initial_join
            return measured_initial_join(checked, ego, max_curvature=gui.MAX_CONTROL_CURVATURE,
                                         lateral_accel=gui.BYPASS_ALAT_MPS2)
        def checked(path, horizon=60.):
            return min(raw(path, horizon), gui.static_clear_distance(path, ego, soft,
                       horizon, body_shift_m=REAR_AXLE_SHIFT_M))
        return ego, objects, points, route, corridor, soft, canonical, checked

    def previous(self, route):
        return dict(path=route[:41].copy(), road_return=True, kind='bypass', direction=1.,
                    offset_m=.45, speed_cap_mps=40./3.6)

    def test_captured_current_observations_choose_a_full_gap_before_later_fusion_arrives(self):
        for index in (14, 15):
            with self.subTest(frame=index):
                ego, objects, points, route, corridor, soft, canonical, checked = self.frame(index)
                provenance = json.loads(str(self.data[f'frame{index}_wheel_provenance']))
                self.assertLess(provenance['pose_distance_m'], 1e-6)
                self.assertTrue(provenance['wheel_valid'])
                self.assertEqual(ego['wheel_angle'], provenance['wheel_raw'])
                # MPC logs matching measured ROS XY at six decimal places;
                # it does not expose that sample's exact header timestamp.
                original = copy.deepcopy(objects)
                previous = self.previous(route)
                plan = ma.road_rejoin_plan(route, ego, soft, previous, canonical, checked,
                                          time.perf_counter()+.2)
                self.assertTrue(ma.static_plan_complete(plan), plan)
                self.assertFalse(plan['reused'])
                self.assertGreater(plan['clear_distance_m'], 59.)
                self.assertGreater(plan['preferred_raw_gap_m'], .1)
                self.assertEqual(objects, original)
                np.testing.assert_array_equal(previous['path'], route[:41])
                path, horizon = plan['path'], plan['clear_distance_m']
                self.assertGreaterEqual(corridor.clear_distance(path, horizon), horizon-.01)
                self.assertGreaterEqual(swept_clearance(path, points, horizon, .25,
                    lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
                self.assertGreaterEqual(gui.static_clear_distance(path, ego, objects, horizon,
                    body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
                # The selection above receives only the original current scan.
                # Check its fixed world geometry against the later observation
                # separately; this is a hindsight regression, never planner input.
                later = ma.road_static_objects(ma.split_static(json.loads(str(
                    self.data['later_observed_objects'])))[0],
                    json.loads(str(self.data['later_observed_ego'])), self.data['later_observed_points'])
                future_box = next(obj for obj in later if obj[11] == 9)
                self.assertGreaterEqual(gui.static_clear_distance(path, ego, [future_box], horizon,
                    body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
                source = json.loads(str(self.data[f'frame{index}_source_ego']))
                def rotation(deg):
                    angle = np.radians(deg)
                    return np.array([[np.cos(angle), -np.sin(angle)],
                                     [np.sin(angle), np.cos(angle)]])
                aligned = (self.data[f'frame{index}_source_points']@rotation(source['heading_deg']).T+
                           [source['x'], source['y']]-[ego['x'], ego['y']])@rotation(ego['heading_deg'])
                np.testing.assert_allclose(aligned, points, atol=1e-10)

    def test_a_completed_hard_candidate_survives_expiry_before_preference_scoring(self):
        ego, objects, points, route, corridor, soft, canonical, checked = self.frame(14)
        clock = [1.]
        def expires_after_complete(path, horizon):
            clear = checked(path, horizon)
            if clear >= horizon-.1:
                clock[0] = 1.026
            return clear
        with patch('model_avoidance.time.perf_counter', side_effect=lambda: clock[0]):
            plan = ma.road_rejoin_plan(route, ego, soft, self.previous(route), canonical,
                                      expires_after_complete, 1.025)
        self.assertTrue(ma.static_plan_complete(plan), plan)
        self.assertGreaterEqual(checked(plan['path'], plan['clear_distance_m']),
                                plan['clear_distance_m']-.01)

    def test_expired_budget_and_a_short_cached_return_cannot_authorize_motion(self):
        ego, objects, points, route, corridor, soft, canonical, checked = self.frame(14)
        with patch('model_avoidance.time.perf_counter', return_value=2.):
            self.assertIsNone(ma.road_rejoin_plan(route, ego, soft, self.previous(route),
                                                canonical, checked, 1.))
        previous = self.previous(route)
        def only_cache_is_clear(path, horizon):
            return horizon if path is previous['path'] else 0.
        self.assertIsNone(ma.road_rejoin_plan(route, ego, soft, previous, canonical,
                                             only_cache_is_clear, time.perf_counter()+1.))

    def test_only_sparse_stationary_compact_returns_ahead_receive_a_preference(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        route = np.column_stack((np.arange(0., 70., .5), np.zeros(140)))
        raw = ma.road_lidar_objects(np.array([[18., 0.]]), ego)[0]
        original = copy.deepcopy(raw)
        preferred = ma._road_sparse_front_objects([raw], ego, route)
        self.assertEqual(len(preferred), 1)
        np.testing.assert_allclose(preferred[0][6:8], [1.1, 1.1])
        self.assertEqual(raw, original)
        for kind in ('behind', 'person', 'moving', 'wall', 'unknown', 'dense'):
            row = list(raw)
            if kind == 'behind': row[0] = -2.
            elif kind == 'person': row[4] = 'pedestrian'
            elif kind == 'moving': row[5] = 1.
            elif kind == 'wall': row[6:8] = [7., .3]
            elif kind == 'unknown': row[3] = 'fusion_obstacle'
            elif kind == 'dense': row[12] = dict(row[12], point_count=40)
            with self.subTest(kind=kind):
                self.assertEqual(ma._road_sparse_front_objects([row], ego, route), [])


if __name__ == '__main__':
    unittest.main()
