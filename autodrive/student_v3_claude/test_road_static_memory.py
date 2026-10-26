"""Static geometry remains conservative and stable through near-field dropouts."""
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from perception.stationary_semantics import metadata, with_metadata
from test_road_static_fusion import obstacle
import test_avoidance_gate as fixture
from tunnel_runtime import swept_clearance


class RoadStaticMemoryTests(unittest.TestCase):
    def setUp(self):
        self.runtime = gui.Runtime.__new__(gui.Runtime)
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=15., stamp=100.)
        self.obj = with_metadata(obstacle(), raw_size_m=[1.1, 1.1], height_m=.4)
        self.points = np.array([[4.9, -1.8], [4.9, -1.6], [5.1, -1.4]])

    def observe(self, objects, points, now):
        normalized = ma.road_static_objects(objects, self.ego, points)
        return self.runtime.remember_static_objects(normalized, now, True, objects)

    def test_dropout_keeps_verified_dimensions_without_renewing_expiry(self):
        checked, = self.observe([self.obj], self.points, 100.)
        np.testing.assert_allclose(checked[6:8], [.9, .9])
        for now in (100.2, 101., 102., 102.9):
            remembered, = self.observe([], np.array([[40., 8.]]), now)
            np.testing.assert_allclose(remembered[6:8], [.9, .9])
            self.assertEqual(self.runtime._static_object_memory[0][0], 100.)
        self.assertEqual(self.observe([], [], 100.+gui.STATIC_OBJECT_MEMORY_S+.001), [])

    def test_fresh_larger_surface_replaces_smaller_remembered_geometry(self):
        self.observe([self.obj], self.points, 100.)
        larger = with_metadata(obstacle(size=(2.3, 1.1)), raw_size_m=[2.3, 1.1])
        current, = self.observe([larger], np.array([[4., -1.6], [6., -1.6]]), 100.2)
        self.assertGreaterEqual(current[6], 2.1-1e-9)
        remembered, = self.observe([], [], 100.3)
        self.assertEqual(remembered[6:8], current[6:8])
        self.assertEqual(len(self.runtime._static_object_memory), 1)

    def test_unverified_fresh_observation_keeps_its_full_box(self):
        self.observe([self.obj], self.points, 100.)
        current, = self.observe([self.obj], [], 100.2)
        self.assertEqual(current[6:8], (1.1, 1.1))
        remembered, = self.observe([], [], 100.3)
        self.assertEqual(remembered[6:8], (1.1, 1.1))

    def test_pose_specific_recovery_is_not_transferred_to_another_pose(self):
        checked, = ma.road_static_objects([self.obj], self.ego, self.points)
        geometry = dict(metadata(checked)['road_static_geometry'],
                        initial_margin_floor_m=.15, initial_margin_restore_start_m=4.)
        checked = with_metadata(checked, road_static_geometry=geometry)
        current, = self.runtime.remember_static_objects([checked], 100., True)
        self.assertEqual(metadata(current)['road_static_geometry'], geometry)
        remembered, = self.runtime.remember_static_objects([], 100.2, True)
        stored = metadata(remembered)['road_static_geometry']
        self.assertNotIn('initial_margin_floor_m', stored)
        self.assertNotIn('initial_margin_restore_start_m', stored)
        self.assertEqual(stored['physical_size_m'], geometry['physical_size_m'])
        self.assertEqual(remembered[6:8], checked[6:8])
        self.assertIn('initial_margin_floor_m', metadata(checked)['road_static_geometry'])

    def test_remembered_actual_front_obstacle_still_blocks_the_body(self):
        obj = with_metadata(obstacle(y=0.), raw_size_m=[1.1, 1.1])
        self.observe([obj], np.array([[5., 0.]]), 100.)
        remembered = self.observe([], [], 100.2)
        path = np.column_stack((np.arange(0., 30.1, .25), np.zeros(121)))
        self.assertLess(gui.static_clear_distance(path, self.ego, remembered, 30.,
                            body_shift_m=REAR_AXLE_SHIFT_M), 2.)

    def test_model_host_normalizes_before_remembering_the_observation(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        route = np.column_stack((np.arange(0., 80.1, .5), np.zeros(161)))
        corridor = LaneCorridor(route, np.full(161, 3.5), np.full(161, -3.5))
        runtime = fixture.AvoidanceGateTests().runtime(action='KEEP', objects=[self.obj],
                                                     points=self.points)
        runtime._contact = None
        with patch('static_lane_pass.normal_static_area', return_value=True), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            for now, objects, points in ((100., [self.obj], self.points),
                                         (100.2, [], np.array([[40., 8.]]))):
                lane, clear = ma.road_static_checks(corridor, points)
                _, _, context = runtime.model_static_plan(self.ego, route, objects, points,
                    corridor, lane, clear, None, {}, now, True, True, 1.)
                obj = next(o for o in context['static'] if o[3] == 'fusion_obstacle')
                np.testing.assert_allclose(obj[6:8], [.9, .9])
        self.assertEqual(runtime.remember_static_objects([], 100.3, False), [])

    def test_recorded_bollard_dropout_keeps_a_complete_collision_checked_path(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        runtime = fixture.AvoidanceGateTests().runtime(action='KEEP')
        root = Path(__file__).parent
        world = np.array([[p['x'], p['y']] for p in json.loads(
            (root/'global_path.json').read_text())['waypoints']])
        runtime.lanes = gui.HdMapLanes(root/'link_set.json')
        runtime.lanes.set_route(world)
        runtime._contact = None
        saved = None
        with np.load(Path(__file__).parent/'test_data/static_memory_dropout_20261016.npz') as data, \
                patch('static_lane_pass.normal_static_area', return_value=True), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            for frame, speed in ((6, 38.1), (7, 32.6)):
                prefix = f'frame{frame}_'
                ego = dict(json.loads(str(data[prefix+'ego'])), speed_kmh=speed, wheel_angle=0.)
                objects = json.loads(str(data[prefix+'objects']))
                points = data[prefix+'points']
                route = data[prefix+'route']
                now = float(data[prefix+'scan_stamp'])
                runtime.object_receiver.observation_stamp = float(data[prefix+'object_stamp'])
                # The capture stores the already permitted right bound. Supply
                # its original bound; the host applies the permission once.
                corridor = LaneCorridor(route, data[prefix+'left'], data[prefix+'right']+.5,
                                        body_shift_m=REAR_AXLE_SHIFT_M)
                lane, clear = ma.road_static_checks(corridor, points)
                previous = gui.continue_static_plan(saved, ego, now) if saved else None
                plan, _, context = runtime.model_static_plan(ego, route, objects, points,
                    corridor, lane, clear, previous, {}, now, True, True, 1.)
                self.assertTrue(ma.static_plan_complete(plan), runtime._model_avoidance_log)
                path = plan['path']
                required = max(35., plan['hold_until_m'], plan['safe_horizon_m'])
                self.assertGreaterEqual(gui.static_clear_distance(path, ego, context['static'], required,
                    body_shift_m=REAR_AXLE_SHIFT_M), required-.01)
                self.assertGreaterEqual(swept_clearance(path, points, required, .25,
                    lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), required-.01)
                self.assertGreaterEqual(context['corridor'].clear_distance(path, required), required-.01)
                self.assertLessEqual(max(gui.path_curvatures(path)), gui.MAX_CONTROL_CURVATURE)
                self.assertGreater(plan['speed_cap_mps'], 0.)
                angle = math.radians(ego['heading_deg'])
                rotation = np.array([[math.cos(angle), -math.sin(angle)],
                                     [math.sin(angle), math.cos(angle)]])
                saved = dict(stamp=now, world_path=path@rotation.T+[ego['x'], ego['y']],
                             plan={k: v for k, v in plan.items() if k != 'path'})


if __name__ == '__main__':
    unittest.main()
