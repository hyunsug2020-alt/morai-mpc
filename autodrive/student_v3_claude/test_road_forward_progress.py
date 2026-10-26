"""Recorded road-stall regressions using real inference and publication code.

The fixed world returns come from the 2026-10-16 stall at route index 748.
The offline follower is deliberately simple: these tests establish planner and
publisher progress, not MORAI vehicle dynamics or competition performance.
"""
import copy
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
import test_avoidance_gate as avoidance_fixtures
import test_live_path_safety as live_fixtures
from tunnel_runtime import swept_clearance


HERE = Path(__file__).parent


def rotation(heading_deg):
    yaw = math.radians(heading_deg)
    return np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])


class RecordedRoadForwardProgress(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(HERE/'test_data/road_static_fusion_748_20261016.npz') as data:
            cls.frame = {key: data[key].copy() for key in data.files}
        cls.initial = json.loads(str(cls.frame['ego']))
        cls.objects = json.loads(str(cls.frame['objects']))
        cls.point_world = cls.frame['points']@rotation(cls.initial['heading_deg']).T + [
            cls.initial['x'], cls.initial['y']]
        cls.route_world = cls.frame['route']@rotation(cls.initial['heading_deg']).T + [
            cls.initial['x'], cls.initial['y']]
        cls.waypoints = json.loads((HERE/'global_path.json').read_text())['waypoints']
        cls.world = np.array([[p['x'], p['y']] for p in cls.waypoints])
        cls.smooth = gui.smooth_control_route(cls.world)
        # Freeze the actual operator rules in the fixture. Tests must not depend
        # on a dashboard user changing the live drivable-area settings.
        with patch.dict(os.environ, {'STUDENT_DRIVABLE_RULES': ''}):
            cls.lanes = gui.HdMapLanes(HERE/'link_set.json')
        cls.lanes.rules_cache = SimpleNamespace(value=json.loads(str(cls.frame['rules'])))
        cls.lanes.set_route(cls.world)
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    def local_points(self, ego, point_world=None):
        return ((self.point_world if point_world is None else point_world) - [ego['x'], ego['y']]) @ rotation(
            ego['heading_deg'])

    def test_recorded_frame_has_a_complete_path_without_widening_the_legal_corridor(self):
        ego = dict(self.initial)
        corridor = LaneCorridor(self.frame['route'], self.frame['left'], self.frame['right'])
        points = self.frame['points']
        lane, clear = ma.road_static_checks(corridor, points)
        original, _, _ = ma.plan_bypass(self.frame['route'], ego, self.objects, lane, clear,
            committed_direction=1., adjacent_offsets={}, budget_s=2., body_shift_m=REAR_AXLE_SHIFT_M)
        self.assertTrue(original and original.get('blocked'), original)
        static = ma.road_static_objects(self.objects, ego, points)
        plan, _, info = ma.plan_bypass(self.frame['route'], ego, static, lane, clear,
            committed_direction=1., adjacent_offsets={}, body_shift_m=REAR_AXLE_SHIFT_M)
        self.assertTrue(ma.static_plan_complete(plan), (plan, info))
        self.assertGreaterEqual(plan['clear_distance_m'], 35.)
        self.assertFalse(info['budget_exhausted'])
        np.testing.assert_array_equal(corridor.left, self.frame['left'])
        np.testing.assert_array_equal(corridor.right, self.frame['right'])

    def test_324_pose_heading_speed_and_raw_return_variants_keep_full_checked_progress(self):
        # Perturb the measured world, rather than moving points with the ego.
        # Each candidate must independently pass object SAT, raw-return sweep,
        # body-in-corridor checks, and the vehicle's curvature limit.
        count = 0
        for lateral in (-.005, 0., .005):
            for forward in (-.005, 0., .005):
                for heading in (-.1, 0., .1):
                    for kmh in (0., 3., 6., 10.):
                        for noise in (-.003, 0., .003):
                            with self.subTest(lateral=lateral, forward=forward,
                                              heading=heading, kmh=kmh, noise=noise):
                                ego = dict(self.initial, speed_kmh=kmh,
                                           heading_deg=self.initial['heading_deg']+heading)
                                shift = np.array([forward, lateral]) @ rotation(self.initial['heading_deg']).T
                                ego['x'] += shift[0]
                                ego['y'] += shift[1]
                                route = (self.route_world-[ego['x'], ego['y']]) @ rotation(ego['heading_deg'])
                                points = self.local_points(ego)+noise
                                corridor = self.lanes.corridor(ego, route)
                                lane, clear = ma.road_static_checks(corridor, points)
                                objects = ma.road_static_objects(self.objects, ego, points)
                                plan, _, info = ma.plan_bypass(route, ego, objects, lane, clear,
                                    adjacent_offsets={}, body_shift_m=REAR_AXLE_SHIFT_M, budget_s=2.)
                                self.assertTrue(ma.static_plan_complete(plan), (plan, info))
                                self.assertGreaterEqual(clear(plan['path'], 35.), 34.9)
                                self.assertGreaterEqual(gui.static_clear_distance(plan['path'], ego,
                                    objects, 35., body_shift_m=REAR_AXLE_SHIFT_M), 34.9)
                                self.assertGreaterEqual(swept_clearance(plan['path'], points, 35., .25,
                                    lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), 34.9)
                                self.assertLessEqual(max(gui.path_curvatures(plan['path'])),
                                                     gui.MAX_CONTROL_CURVATURE)
                                self.assertTrue(np.isfinite(plan['path']).all())
                                self.assertTrue(np.all(np.diff(plan['path'][:, 0]) > -.05))
                                count += 1
        self.assertEqual(count, 324)

    def runtime(self):
        runtime = avoidance_fixtures.AvoidanceGateTests().runtime(action='KEEP')
        runtime.config.update(route_points=20, ego_steps=1)
        runtime.waypoints, runtime.waypoint_xy = self.waypoints, self.world
        runtime.control_waypoint_xy, runtime.lanes = self.smooth, self.lanes
        runtime.route_tensor = lambda ego, status: gui.Runtime.route_tensor(runtime, ego, status)
        runtime.speed_zones = SimpleNamespace(kmh_at=lambda idx: (20., {}))
        runtime.path_publisher.cruise_mps = 20./3.6
        runtime.test_scan = {'stamp': 1_000_000.}
        return live_fixtures.LivePathSafetyTests().attach(runtime)

    def observe(self, runtime, ego, now, sequence, *, point_world=None):
        nearest = int(np.argmin(np.linalg.norm(self.world-[ego['x'], ego['y']], axis=1)))
        runtime.test_status = dict(ego=dict(ego, stamp=now, age_s=.01, received=now, measured=True),
            path=dict(nearest_idx=nearest, distance_m=float(np.linalg.norm(
                self.world[nearest]-[ego['x'], ego['y']]))))
        points = self.local_points(ego, point_world)
        runtime.lidar.pts, runtime.lidar.received = points.astype(np.float32), now
        runtime.lidar._scan_stamp, runtime.lidar._scan_sequence = now, sequence
        runtime.object_receiver.objects = copy.deepcopy(self.objects[:1])
        runtime.object_receiver.received = runtime.object_receiver.observation_stamp = now
        runtime.test_scan = dict(xyz=np.empty((0, 3)), obstacle_points=points, stamp=now,
                                sequence=sequence, age_s=.01, ground_plane=np.zeros(3))
        runtime.ego_receiver.samples.append(dict(runtime.test_status['ego']))
        return points

    def test_real_inference_and_live_publisher_pass_the_obstacle_without_reverse_or_rejoin_stop(self):
        for heading_bias in (-.1, 0., .1):
            with self.subTest(heading_bias=heading_bias):
                runtime = self.runtime()
                ego = dict(self.initial, heading_deg=self.initial['heading_deg']+heading_bias)
                now, speed, travelled, rejoin_frames = 1_000_000., 0., 0., 0
                with patch('time.monotonic', side_effect=lambda: now), \
                        patch('time.time', side_effect=lambda: now), \
                        patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                        patch.object(gui, 'model_avoidance', ma, create=True):
                    for frame in range(220):
                        points = self.observe(runtime, ego, now, frame+1)
                        self.assertTrue(runtime.infer())
                        payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
                        target = float(payload['target_speed_mps'])
                        self.assertNotEqual(payload.get('gear'), 'R', payload)
                        self.assertIsNone(getattr(runtime.path_publisher, 'reverse_state', None))
                        self.assertGreater(target, .1, runtime._web_record)
                        self.assertFalse(payload['decision']['emergency'], payload['decision'])
                        rejoin_frames += runtime._web_record['mode'] == 'STUDENT REJOIN'
                        path_world = np.array([[p['x'], p['y']] for p in payload['waypoints']])
                        path = (path_world-[ego['x'], ego['y']]) @ rotation(ego['heading_deg'])
                        required = max(4., speed*.7+speed*speed/5.+2.)
                        self.assertGreaterEqual(swept_clearance(path, points, required, .25,
                            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), required-.01)
                        nearest = runtime.test_status['path']['nearest_idx']
                        route = (self.smooth[(nearest+np.arange(240)) % len(self.smooth)] - [
                            ego['x'], ego['y']]) @ rotation(ego['heading_deg'])
                        corridor = self.lanes.corridor(ego, route)
                        corridor.body_shift_m = REAR_AXLE_SHIFT_M
                        original_corridor = corridor
                        from static_lane_pass import normal_static_area
                        if gui.in_obstacle_mission(nearest) and normal_static_area(self.lanes, ego):
                            # Independent operator contract: ordinary static passes
                            # may use .5 m beyond the right white line. Retain the
                            # left line, intersection samples and no-go polygons;
                            # never reuse a planner's selected corridor as proof.
                            right = corridor.right-np.where(corridor.intersection_mask, 0., .5)
                            corridor = LaneCorridor(route, corridor.left, right, corridor.zones,
                                corridor.intersection_mask, body_shift_m=REAR_AXLE_SHIFT_M)
                        checked = corridor.clear_distance(path, required)
                        self.assertGreaterEqual(checked, required-.01, dict(
                            ego=ego, nearest=nearest, required_m=required,
                            original_clear_m=original_corridor.clear_distance(path, required),
                            original_full_clear_m=original_corridor.clear_distance(path, 60.),
                            permitted_clear_m=checked,
                            permitted_full_clear_m=corridor.clear_distance(path, 60.)))
                        objects = ma.road_static_objects(self.objects, ego, points)
                        self.assertGreaterEqual(gui.static_clear_distance(path, ego, objects,
                            required, body_shift_m=REAR_AXLE_SHIFT_M), required-.01)
                        self.assertLessEqual(max(gui.path_curvatures(path)), gui.MAX_CONTROL_CURVATURE+1e-5)
                        look = max(3., .45*speed+2.)
                        ahead = np.flatnonzero((path[:, 0] > 0.) &
                                              (np.linalg.norm(path, axis=1) >= look))
                        self.assertGreater(len(ahead), 0)
                        q = path[ahead[0]]
                        curvature = float(2.*q[1]/(q@q))
                        speed = max(0., speed+float(np.clip((target-speed)/.25, -3.5, 2.1))*.1)
                        yaw = math.radians(ego['heading_deg'])
                        ego['x'] += speed*math.cos(yaw)*.1
                        ego['y'] += speed*math.sin(yaw)*.1
                        ego['heading_deg'] += math.degrees(speed*curvature*.1)
                        ego['speed_kmh'] = speed*3.6
                        travelled += speed*.1
                        now += .1
                        if travelled > 70.:
                            break
                self.assertGreater(travelled, 70.)
                self.assertGreater(runtime.test_status['path']['nearest_idx'], 880)
                # Completion is a measured route return, not a minimum time
                # spent carrying the old offset under a REJOIN label.
                self.assertLess(runtime.test_status['path']['distance_m'], .35)
                self.assertEqual(runtime._web_record['mode'], 'ROUTE BASE')

    def test_an_actual_blocked_departure_cannot_force_straight_or_repeat_reverse(self):
        runtime = self.runtime()
        ego, now = dict(self.initial, speed_kmh=0.), 1_000_000.
        blocked_world = np.array([[3.9, 0.], [3.9, .1], [3.9, -.1]]) @ rotation(
            ego['heading_deg']).T + [ego['x'], ego['y']]
        with patch('time.monotonic', side_effect=lambda: now), \
                patch('time.time', side_effect=lambda: now), \
                patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            for frame in range(100):
                self.observe(runtime, ego, now, frame+1, point_world=np.vstack((self.point_world, blocked_world)))
                self.assertTrue(runtime.infer())
                payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
                self.assertEqual(payload['target_speed_mps'], 0.)
                self.assertNotEqual(payload.get('gear'), 'R')
                self.assertIsNone(getattr(runtime.path_publisher, 'reverse_state', None))
                now += .1

    def test_completed_rejoin_releases_its_old_side_for_the_next_obstacle(self):
        route = np.column_stack((np.arange(0., 60.01, .25), np.zeros(241)))
        for front in (8., 12.):
            with self.subTest(front=front):
                # The previous left bypass has fully rejoined. A new obstacle
                # permits only a right pass; it must not keep the old side just
                # because it lies inside the moving car's braking horizon.
                box = (front, .5, 1., 'box', 'obstacle', 0., 1., 1., 0.)
                points = np.array([[front, y] for y in np.arange(.1, 1.01, .1)])
                runtime = avoidance_fixtures.AvoidanceGateTests().runtime(
                    action='KEEP', objects=[box], points=points)
                runtime.lanes.adjacent_offsets = lambda *args: {}
                ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0. if front == 8. else 20.)
                corridor = LaneCorridor(route, np.full(len(route), 2.), np.full(len(route), -3.5))
                previous = dict(path=route, kind='rejoin', direction=1., offset_m=.1,
                    transition_m=10., hold_m=6., hold_until_m=8., curvature=0.)
                self.assertTrue(ma.manoeuvre_complete(previous, route))
                with patch.object(gui, 'model_avoidance', ma, create=True):
                    plan, _, context = runtime.model_static_plan(ego, route, [box], points,
                        corridor, lambda *args: True, corridor.clear_distance, previous, {},
                        1_000_000., True, True, 1.2)
                self.assertTrue(context['route_return_ready'])
                self.assertEqual(context['direction'], 0.)
                self.assertTrue(ma.static_plan_complete(plan), plan)
                self.assertEqual(plan['direction'], -1.)


if __name__ == '__main__':
    unittest.main()
