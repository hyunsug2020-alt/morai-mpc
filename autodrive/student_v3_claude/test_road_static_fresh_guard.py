"""Fresh road observations keep the same static body contract as candidate search."""
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from perception.stationary_semantics import metadata
import test_avoidance_gate as fixtures


def obstacle(x=10., y=1.65, *, size=(1.1, 1.1), kind='obstacle',
             name='fusion_obstacle', speed=0.):
    return (x, y, max(size), name, kind, speed, *size, 0., 0., 0., 9,
            0., None, None, 0., None,
            dict(raw_size_m=list(size), height_m=.7))


class FreshRoadStaticGuardTests(unittest.TestCase):
    def setUp(self):
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=20., stamp=100.)
        self.path = np.column_stack((np.arange(0., 40.01, .5), np.zeros(81)))
        self.obj = obstacle()
        self.points = np.array([[10., 1.65]])

    def check(self, objects=None, points=None, *, enabled=True, half_width=1.15, path=None):
        return gui.fresh_road_static_hazard([self.obj] if objects is None else objects,
            self.ego, self.path if path is None else path, 25., half_width,
            self.points if points is None else points, enabled=enabled)

    def test_coarse_side_overlap_does_not_stop_a_raw_verified_clear_body(self):
        self.assertIsNotNone(gui.route_hazard([self.obj], self.ego, self.path, 25., 1.15))
        hit, info = self.check()
        self.assertIsNone(hit)
        self.assertEqual(info['normalized_n'], 1)
        self.assertGreaterEqual(info['clear_m'], 25.)
        self.assertEqual(self.obj[6:8], (1.1, 1.1))

    def test_new_front_body_remains_a_stop_without_a_prior_planner_observation(self):
        hit, info = self.check([obstacle(y=0.)], np.array([[10., 0.]]))
        self.assertIsNotNone(hit)
        self.assertEqual(hit[2], 'fusion_obstacle')
        self.assertLess(info['clear_m'], 10.)
        self.assertEqual(info['normalized_n'], 1)

    def test_measured_static_braking_room_is_a_cap_and_current_contact_still_stops(self):
        for x, speed, age, expected in ((12., 20., .1, True), (4., 20., .1, False),
                                        (12., 55., .1, False), (12., 20., .31, False)):
            with self.subTest(x=x, speed=speed, age=age):
                hit, info = gui.fresh_road_static_hazard([obstacle(x=x, y=0.)],
                    dict(self.ego, speed_kmh=speed, age_s=.01), self.path, 25., 1.15,
                    np.array([[x, 0.]]), enabled=True, source_age_s=age)
                self.assertIsNotNone(hit)
                self.assertEqual(info.get('static_braking_cap_mps') is not None, expected)
                if expected:
                    self.assertGreater(info['static_braking_cap_mps'], 0.)

    def test_missing_or_unassociated_raw_keeps_the_larger_current_box(self):
        for points in (None, np.empty((0, 2)), np.array([[50., 50.]])):
            with self.subTest(points=points):
                hit, info = gui.fresh_road_static_hazard([self.obj], self.ego, self.path,
                    25., 1.15, points, enabled=True)
                self.assertIsNotNone(hit)
                self.assertEqual(info['normalized_n'], 0)

    def test_fresh_larger_box_cannot_inherit_the_previous_small_geometry(self):
        hit, info = self.check([obstacle(size=(2.1, 2.1))])
        self.assertIsNotNone(hit)
        self.assertLess(info['clear_m'], 25.)

    def test_people_motion_and_other_sources_keep_the_original_guard(self):
        cases = (obstacle(kind='pedestrian', name='fusion_pedestrian', speed=1.),
                 obstacle(speed=1.2), obstacle(name='unclassified_static'),
                 obstacle(name='fusion_parked_vehicle', size=(4.6, 1.9)))
        for obj in cases:
            with self.subTest(name=obj[3], speed=obj[5]):
                expected = gui.route_hazard([obj], self.ego, self.path, 25., 1.15)
                hit, info = self.check([obj])
                self.assertEqual(hit, expected)
                self.assertEqual(info['static_n'], 0)

    def test_outside_scope_keeps_the_original_stop(self):
        hit, info = self.check(enabled=False)
        self.assertEqual(hit, gui.route_hazard([self.obj], self.ego, self.path, 25., 1.15))
        self.assertFalse(info['active'])

    def test_a_verified_box_cannot_authorize_an_invalid_path(self):
        hit, _ = self.check(path=np.full((4, 2), np.nan))
        self.assertIsNotNone(hit)
        self.assertEqual(hit[0], 0.)

    def test_short_path_cannot_hide_a_new_front_body_beyond_its_endpoint(self):
        hit, info = self.check([obstacle(y=0.)], np.array([[10., 0.]]),
                              path=self.path[:9])
        self.assertIsNotNone(hit)
        self.assertEqual(hit[0], 0.)
        self.assertFalse(info['path_horizon_covered'])

    def test_a_braking_safe_checked_prefix_is_not_an_obstacle_at_the_bumper(self):
        ego = dict(self.ego, speed_kmh=29.4, age_s=.03)
        path = self.path[self.path[:, 0] <= 18.]
        hit, info = gui.fresh_road_static_hazard([self.obj], ego, path,
            21.1, 1.15, self.points, enabled=True, source_age_s=.1)
        self.assertIsNone(hit)
        self.assertFalse(info['path_horizon_covered'])
        self.assertGreater(info['clear_m'], 17.9)
        cap = info['checked_path_end_cap_mps']
        self.assertGreater(cap, 0.)
        # At the issued pace, the entire response and comfortable stop fit
        # before the checked prefix ends, with its one-metre reserve intact.
        self.assertLessEqual(cap*.25+cap*cap/(2.*2.5)+1., 18.+1e-6)

    def test_a_short_prefix_keeps_stale_current_contact_and_unstoppable_stops(self):
        path = self.path[self.path[:, 0] <= 18.]
        for speed, age, obj in ((55., .1, self.obj), (29.4, .31, self.obj),
                                (29.4, .1, obstacle(x=4., y=0.))):
            with self.subTest(speed=speed, age=age):
                hit, info = gui.fresh_road_static_hazard([obj], dict(self.ego, speed_kmh=speed),
                    path, 25., 1.15, self.points, enabled=True, source_age_s=age)
                self.assertIsNotNone(hit)
                self.assertNotIn('checked_path_end_cap_mps', info)

    def test_recorded_uid9_is_rechecked_even_when_the_planning_snapshot_was_empty(self):
        # follow-20261016T120632Z, capture 0020. The new fusion prior is
        # larger than its one-point LiDAR cluster and must remain authoritative.
        ego = dict(x=-60.078849971838615, y=-111.27472395443867,
                   heading_deg=92.50601201466475, speed_kmh=31.6)
        obj = (-58.6039592696359, -99.6052592766792, 1.100000023841858,
               'fusion_obstacle', 'obstacle', 0., 1.100000023841858,
               1.100000023841858, 92.91364542439973, 0., 0., 9, 0.,
               None, None, 0., None,
               dict(raw_size_m=[1.100000023841858]*2, height_m=.7055168616122891))
        points = np.array([[11.600000381469727, -2.]])
        waypoints = json.loads((Path(__file__).parent/'global_path.json').read_text())['waypoints']
        world = np.array([[p['x'], p['y']] for p in waypoints])
        at = int(np.argmin(np.linalg.norm(world-[ego['x'], ego['y']], axis=1)))
        yaw = math.radians(ego['heading_deg'])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        route = (gui.smooth_control_route(world)[np.arange(at, at+150)%len(world)]-
                 [ego['x'], ego['y']]) @ rotation
        path = gui.RosPathPublisher.control_geometry(route, route,
            dict(mode='LATTICE', validated_static=True))
        hit, info = gui.fresh_road_static_hazard([obj], ego, path, 25., .95, points, enabled=True)
        self.assertIsNotNone(hit)
        self.assertEqual(info['static_n'], 1)
        self.assertEqual(info['normalized_n'], 1)
        self.assertLess(info['clear_m'], 25.)


class CurrentStaticGuardFrameTests(unittest.TestCase):
    def setUp(self):
        stamp = time.time()
        self.old = dict(x=0., y=0., heading_deg=0., stamp=stamp-.4,
                        speed_kmh=7., measured=True)
        self.current = dict(self.old, x=.6, heading_deg=-5., stamp=stamp)
        self.path = np.column_stack((np.arange(0., 30.5, .5), np.zeros(61)))
        yaw = math.radians(self.current['heading_deg'])
        self.axes = np.array([[math.cos(yaw), -math.sin(yaw)],
                              [math.sin(yaw), math.cos(yaw)]])
        self.points = (np.array([[10., 1.65]])-[.6, 0.]) @ self.axes
        scan = dict(stamp=stamp, age_s=0., sequence=4, obstacle_points=self.points)
        self.runtime = SimpleNamespace(
            ego_receiver=SimpleNamespace(snapshot=lambda n: (self.current, [], 20.),
                measured_xy_at_stamp=lambda t: self.current if t == stamp else None),
            lidar=SimpleNamespace(scan_snapshot=lambda: scan))

    def test_selected_world_curve_and_atomic_raw_share_the_current_measured_pose(self):
        ego, path, points, proof = gui.current_static_guard_frame(
            self.runtime, self.old, self.path)
        self.assertTrue(proof['valid'], proof)
        self.assertIs(ego, self.current)
        np.testing.assert_allclose(points, self.points)
        world = path @ self.axes.T+[ego['x'], ego['y']]
        np.testing.assert_allclose(world[:, 1], 0., atol=1e-12)
        self.assertGreaterEqual(world[0, 0], .59)
        hit, info = gui.fresh_road_static_hazard([obstacle()], ego, path,
            20., .95, points, enabled=True)
        self.assertIsNone(hit)
        self.assertEqual(info['normalized_n'], 1)

    def test_a_new_physical_front_body_still_stops_in_the_refreshed_frame(self):
        ego, path, points, proof = gui.current_static_guard_frame(
            self.runtime, self.old, self.path)
        hit, info = gui.fresh_road_static_hazard([obstacle(y=0.)], ego, path,
            20., .95, points, enabled=True)
        self.assertIsNotNone(hit)
        self.assertLess(info['clear_m'], 10.)

    def test_new_atomic_scan_replaces_a_source_that_expired_during_search(self):
        fresh = self.runtime.lidar.scan_snapshot()
        stale = dict(fresh, stamp=fresh['stamp']-.35, age_s=.35, sequence=3)
        self.runtime.lidar.scan_snapshot = Mock(side_effect=[stale, fresh])
        ego, path, points, proof = gui.current_static_guard_frame(
            self.runtime, self.old, self.path)
        self.assertTrue(proof['valid'], proof)
        self.assertEqual(proof['sequence'], 4)
        self.assertEqual(proof['replaced_sequence'], 3)
        self.assertEqual(proof['stamp'], fresh['stamp'])
        np.testing.assert_allclose(points, self.points)

    def test_expired_source_without_a_replacement_remains_a_stop(self):
        stale = dict(self.runtime.lidar.scan_snapshot(), age_s=.35)
        self.runtime.lidar.scan_snapshot = lambda: stale
        started = time.monotonic()
        ego, path, points, proof = gui.current_static_guard_frame(
            self.runtime, self.old, self.path)
        self.assertFalse(proof['valid'])
        self.assertEqual(proof['reason'], 'road scan source stale')
        self.assertIsNone(points)
        self.assertLess(time.monotonic()-started, .2)

    def test_malformed_new_measurement_is_not_replaced_with_a_cached_clear_scan(self):
        fresh = self.runtime.lidar.scan_snapshot()
        self.runtime.lidar.scan_snapshot = Mock(side_effect=[
            dict(fresh, obstacle_points=np.array([[np.nan, 0.]])), fresh])
        _, _, points, proof = gui.current_static_guard_frame(
            self.runtime, self.old, self.path)
        self.assertFalse(proof['valid'])
        self.assertIsNone(points)
        self.assertEqual(self.runtime.lidar.scan_snapshot.call_count, 1)

    def test_missing_current_pose_measurement_cannot_reuse_the_old_scan(self):
        self.runtime.ego_receiver.measured_xy_at_stamp = lambda stamp: None
        ego, path, points, proof = gui.current_static_guard_frame(
            self.runtime, self.old, self.path)
        self.assertFalse(proof['valid'])
        self.assertIsNone(ego)
        self.assertIsNone(path)
        self.assertIsNone(points)

    def test_a_displaced_measured_pose_rejects_the_old_curve_without_ending_the_driver(self):
        self.current['y'] = 3.
        ego, path, points, proof = gui.current_static_guard_frame(
            self.runtime, self.old, self.path)
        self.assertFalse(proof['valid'])
        self.assertIsNone(ego)
        self.assertIsNone(path)
        self.assertIsNone(points)
        self.assertIn('does not match', proof['reason'])


class FreshRoadStaticIntegrationTests(unittest.TestCase):
    def infer_fresh(self, obj, *, normal=True):
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=[], points=[])
        runtime.test_status['path']['nearest_idx'] = 700
        # A second sensor snapshot arrives after planning. It must not be
        # replaced with the older empty model context.
        runtime.object_receiver.snapshot = Mock(side_effect=[[], [obj]])
        with patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch('static_lane_pass.normal_static_area', return_value=normal):
            self.assertTrue(runtime.infer())
        return runtime

    def test_object_arriving_after_planning_stops_the_real_publisher(self):
        runtime = self.infer_fresh(obstacle(y=0.))
        record = runtime._web_record
        self.assertTrue(record['fresh_static_guard']['active'])
        self.assertEqual(record['fresh_static_guard']['static_n'], 1)
        self.assertTrue(record['emergency_stop'])
        self.assertEqual(record['published_cap_kmh'], 0.)

    def test_portal_and_highway_exclusions_keep_the_old_coarse_guard(self):
        runtime = self.infer_fresh(obstacle(), normal=False)
        self.assertFalse(runtime._web_record['fresh_static_guard']['active'])
        self.assertTrue(runtime._web_record['emergency_stop'])

    def test_fresh_person_disables_the_generic_static_guard(self):
        runtime = self.infer_fresh(obstacle(y=0., kind='pedestrian', name='fusion_pedestrian', speed=1.))
        self.assertFalse(runtime._web_record['fresh_static_guard']['active'])
        self.assertTrue(runtime._web_record['emergency_stop'])

    def test_tunnel_dispatch_does_not_call_the_road_guard(self):
        runtime = fixtures.AvoidanceGateTests().runtime()
        runtime.tunnel = SimpleNamespace(active=lambda ego: True)
        runtime.infer_tunnel = Mock(return_value=True)
        with patch.object(gui, 'fresh_road_static_hazard', side_effect=AssertionError('road guard in tunnel')):
            self.assertTrue(runtime.infer())
        runtime.infer_tunnel.assert_called_once()

    def test_remembered_margin_is_refreshed_for_this_pose_without_persisting_it(self):
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=[], points=[])
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0., stamp=100.)
        source = obstacle(x=3., y=-1.55)
        measured = np.array([[3., -1.95]])
        checked = ma.road_static_objects([source], ego, measured)[0]
        runtime.remember_static_objects([checked], 99.8, True)
        self.assertNotIn('initial_margin_floor_m', metadata(runtime._static_object_memory[0][1])['road_static_geometry'])
        route = runtime.test_route
        corridor = LaneCorridor(route, np.full(len(route), 5.), np.full(len(route), -5.))
        points = np.array([[50., 8.]])
        lane, clear = ma.road_static_checks(corridor, points)
        with patch('static_lane_pass.normal_static_area', return_value=True), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            _, _, context = runtime.model_static_plan(ego, route, [], points,
                corridor, lane, clear, None, {}, 100., True, True, 1.)
        geometry = metadata(context['static'][0])['road_static_geometry']
        self.assertAlmostEqual(geometry['initial_margin_floor_m'], .154)
        self.assertNotIn('initial_margin_floor_m', metadata(runtime._static_object_memory[0][1])['road_static_geometry'])
        self.assertEqual(runtime._static_object_memory[0][0], 99.8)

    def test_comfort_prior_is_not_used_for_people_or_tunnel_transitions(self):
        for normal, person in ((False, False), (True, True)):
            people = [obstacle(x=30., y=3., kind='pedestrian', name='fusion_pedestrian')] if person else []
            runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=people,
                                                           points=[[20., 1.1]])
            runtime._contact = None
            route, ego = runtime.test_route, runtime.test_status['ego']
            corridor = runtime.lanes.corridor(ego, route)
            lane, clear = ma.road_static_checks(corridor, runtime.lidar.pts)
            with self.subTest(normal=normal, person=person), \
                    patch('static_lane_pass.normal_static_area', return_value=normal), \
                    patch.object(gui, 'model_avoidance', ma, create=True), \
                    patch.object(ma, 'road_comfort_objects', side_effect=AssertionError('comfort scope widened')):
                runtime.model_static_plan(ego, route, people, runtime.lidar.pts,
                    corridor, lane, clear, None, {}, 100., True, True, 1.)

    def test_small_raw_cluster_prepares_a_path_that_survives_later_fusion_growth(self):
        points = np.array([[20., 1.]])
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=[], points=points)
        runtime._contact = None
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=20., stamp=100.)
        route = runtime.test_route
        corridor = LaneCorridor(route, np.full(len(route), 3.5), np.full(len(route), -3.5))
        lane, clear = ma.road_static_checks(corridor, points)
        previous = None
        with patch('static_lane_pass.normal_static_area', return_value=True), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            for frame, objects in enumerate(([], [obstacle(x=20., y=1.)],
                                            [obstacle(x=20., y=1., size=(1.2, 1.2))])):
                plan, _, context = runtime.model_static_plan(ego, route, objects, points,
                    corridor, lane, clear, previous, {}, 100.+frame*.1, True, True, 1.)
                self.assertTrue(ma.static_plan_complete(plan), (frame, runtime._model_avoidance_log))
                self.assertGreater(plan['speed_cap_mps'], 0.)
                if frame == 0:
                    # Hard boxes remain the small measured cluster. Only the
                    # chosen comfortable pass reserves room for later fusion.
                    self.assertLess(context['static'][0][6], .9)
                    soft = ma.road_comfort_objects(context['static'])
                    self.assertGreaterEqual(gui.static_clear_distance(plan['path'], ego, soft,
                        35., body_shift_m=REAR_AXLE_SHIFT_M), 35.)
                    self.assertGreater(abs(plan['offset_m']), .6)
                hit, _ = gui.fresh_road_static_hazard(objects, ego, plan['path'], 20., .95,
                                                       points, enabled=True)
                self.assertIsNone(hit)
                previous = plan

    def test_unavailable_comfort_path_keeps_the_original_hard_fallback(self):
        points = np.array([[20., 1.]])
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=[], points=points)
        runtime._contact = None
        route = runtime.test_route
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=20., stamp=100.)
        corridor = LaneCorridor(route, np.full(len(route), 3.5), np.full(len(route), -3.5))
        lane, clear = ma.road_static_checks(corridor, points)
        def impossible_preference(objects):
            preferred = []
            for obj in objects:
                row = list(obj)
                row[2], row[6], row[7] = 20., 20., 20.
                preferred.append(tuple(row))
            return preferred
        with patch('static_lane_pass.normal_static_area', return_value=True), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch.object(ma, 'road_comfort_objects', side_effect=impossible_preference):
            plan, _, context = runtime.model_static_plan(ego, route, [], points,
                corridor, lane, clear, None, {}, 100., True, True, 1.)
        self.assertTrue(ma.static_plan_complete(plan), runtime._model_avoidance_log)
        self.assertLess(context['static'][0][6], .9)
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], ego, context['static'],
            35., body_shift_m=REAR_AXLE_SHIFT_M), 35.)


if __name__ == '__main__':
    unittest.main()
