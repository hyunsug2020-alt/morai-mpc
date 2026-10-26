"""White-line permission never grants centre-line or collision permission."""
import unittest
import json
import math
from pathlib import Path
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from tunnel_runtime import swept_clearance
import test_avoidance_gate as fixture


class StaticWhiteLineTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        self.route = np.column_stack((np.arange(0., 80.1, .5), np.zeros(161)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=15., stamp=100., wheel_angle=0.)
        self.corridor = LaneCorridor(self.route, np.full(161, 1.75), np.full(161, -1.75))
        self.obj = (20., .83, 1.1, 'fusion_obstacle', 'obstacle', 0., 1.1, 1.1, 0.,
                    0., 0., 4, 0., None, None, 0., None,
                    dict(raw_size_m=[1.1, 1.1], height_m=.78))
        self.points = np.array([[19.9, .7], [20., .9], [20.1, .8]])

    def plan(self, allow=True, points=None, *, normal=True, objects=None):
        points = self.points if points is None else points
        objects = [self.obj] if objects is None else objects
        runtime = fixture.AvoidanceGateTests().runtime(action='KEEP', objects=objects, points=points)
        runtime._contact = None
        lane, clear = ma.road_static_checks(self.corridor, points)
        with patch('static_lane_pass.normal_static_area', return_value=normal), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch.object(ma, 'road_white_line_corridor', wraps=(ma.road_white_line_corridor
                                                                 if allow else lambda c, a: c)):
            plan, _, context = runtime.model_static_plan(self.ego, self.route, objects, points,
                self.corridor, lane, clear, None, {}, 100., True, True, 1.)
        return plan, context, runtime

    def test_lane_edge_deadlock_has_a_complete_checked_right_pass(self):
        strict, _, _ = self.plan(False)
        self.assertTrue(strict and strict.get('blocked'), strict)
        plan, context, _ = self.plan()
        self.assertTrue(ma.static_plan_complete(plan), plan)
        self.assertLess(self.corridor.clear_distance(plan['path'], 35.), 20.)
        self.assertGreaterEqual(context['corridor'].clear_distance(plan['path'], 35.), 35.)
        self.assertGreaterEqual(swept_clearance(plan['path'], self.points, 35., .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], self.ego, context['static'],
            35., body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        self.assertLessEqual(max(gui.path_curvatures(plan['path'])), gui.MAX_CONTROL_CURVATURE)
        self.assertLess(plan['direction'], 0.)
        # Low-box prior remains 0.8 m plus the existing shared padding.
        np.testing.assert_allclose(context['static'][0][6:8], [.9, .9])

    def test_no_centre_line_permission_or_mutation_is_added(self):
        checked = ma.road_white_line_corridor(self.corridor)
        np.testing.assert_array_equal(checked.left, self.corridor.left)
        np.testing.assert_array_equal(checked.right, self.corridor.right-.5)
        np.testing.assert_array_equal(self.corridor.right, np.full(161, -1.75))
        for amount in (-.1, .501, float('nan'), float('inf')):
            with self.subTest(amount=amount), self.assertRaises(ValueError):
                ma.road_white_line_corridor(self.corridor, amount)

    def test_junction_samples_and_no_go_polygons_still_block_the_path(self):
        mask = np.zeros(161, bool)
        mask[20:80] = True
        zone = np.array([[10., -2.5], [30., -2.5], [30., -.2], [10., -.2]])
        corridor = LaneCorridor(self.route, self.corridor.left, self.corridor.right,
                                zones=[zone], intersection_mask=mask)
        checked = ma.road_white_line_corridor(corridor)
        np.testing.assert_array_equal(checked.right[mask], corridor.right[mask])
        self.assertIs(checked.zones, corridor.zones)
        path = self.route.copy()
        path[:, 1] = -1.
        self.assertLess(checked.clear_distance(path, 35.), 35.)

    def test_actual_front_blocker_is_not_cleared_by_white_line_permission(self):
        points = np.vstack((self.points, [[3.9, 0.]]))
        plan, _, _ = self.plan(points=points)
        self.assertTrue(plan is None or plan.get('blocked'), plan)

    def test_tunnel_highway_and_pedestrian_scope_is_preserved(self):
        for normal, objects in ((False, [self.obj]),
                                (True, [self.obj, (10., 0., .6, 'person', 'pedestrian', 1., .6, .6, 0.)])):
            with self.subTest(normal=normal, people=len(objects)>1), \
                    patch.object(ma, 'road_white_line_corridor', side_effect=AssertionError('scope widened')):
                self.plan(normal=normal, objects=objects)

    def test_continued_checked_vertices_are_not_rejected_by_a_second_resampling(self):
        radius = 13.005608605302925
        station = np.arange(0., 20.01, .5)
        angle = station/radius
        curve = np.column_stack((radius*np.sin(angle), radius*(1.-np.cos(angle))))
        tail = curve[-1]+np.arange(.5, 40.01, .5)[:, None]*[math.cos(angle[-1]), math.sin(angle[-1])]
        world = np.vstack((curve, tail))
        tangent = world[18]-world[16]
        ego = dict(self.ego, x=world[17, 0]-.06751869760330192,
                   y=world[17, 1]-.06553506946233845,
                   heading_deg=math.degrees(math.atan2(tangent[1], tangent[0]))-7.480779226873505)
        saved = dict(stamp=100., world_path=world, plan=dict(kind='bypass', direction=1.,
            offset_m=.6, hold_m=15., transition_m=10., speed_cap_mps=4.,
            safe_horizon_m=5., clear_distance_m=50., hold_until_m=20., canonical_control=True))
        previous = gui.continue_static_plan(saved, ego, 100.1)
        path = previous['path']
        self.assertLess(max(gui.path_curvatures(path)), gui.MAX_CONTROL_CURVATURE)
        twice = gui.RosPathPublisher.control_geometry(path, self.route,
            dict(mode='LATTICE', validated_static=True))
        self.assertGreater(max(gui.path_curvatures(twice)), gui.MAX_CONTROL_CURVATURE)
        yaw = math.radians(ego['heading_deg'])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        xy = np.array([10., 0.])@rotation.T+[ego['x'], ego['y']]
        obj = (*xy, .5, 'box', 'obstacle', 0., .5, .5, 0.)
        points = np.array([[10., 0.]])
        corridor = LaneCorridor(self.route, np.full(len(self.route), 60.), np.full(len(self.route), -60.))
        runtime = fixture.AvoidanceGateTests().runtime(action='KEEP', objects=[obj], points=points)
        runtime._contact = None
        lane, clear = ma.road_static_checks(corridor, points)
        with patch('static_lane_pass.normal_static_area', return_value=True), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            plan, _, context = runtime.model_static_plan(ego, self.route, [obj], points,
                corridor, lane, clear, previous, {}, 100.1, True, True, 1.)
        self.assertTrue(ma.static_plan_complete(plan), (plan, runtime._model_avoidance_log))
        self.assertTrue(plan['reused'])
        np.testing.assert_array_equal(plan['path'], path)
        self.assertGreaterEqual(swept_clearance(plan['path'], points, 35., .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), 35.)

        blocked_points = np.vstack((points, [[3.9, 0.]]))
        lane, clear = ma.road_static_checks(corridor, blocked_points)
        with patch('static_lane_pass.normal_static_area', return_value=True), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            rejected, _, _ = runtime.model_static_plan(ego, self.route, [obj], blocked_points,
                corridor, lane, clear, previous, {}, 100.2, True, True, 1.)
        self.assertFalse(ma.static_plan_complete(rejected), rejected)


    def test_recorded_front_obstacles_pass_through_real_inference_and_publication(self):
        # Rewind along the actual route through the captured front obstacle
        # field. Historical rear returns cannot represent an earlier scan.
        # This checks planner/publication progress; MORAI validates dynamics.
        import test_road_forward_progress as recorded
        recorded.RecordedRoadForwardProgress.setUpClass()
        with np.load(Path(__file__).parent/'test_data/road_static_white_front_20261016.npz') as data:
            original = json.loads(str(data['ego']))
            objects = json.loads(str(data['objects']))
            points = data['points'].copy()
        f = recorded.RecordedRoadForwardProgress()
        f.initial, f.objects = original, objects
        f.point_world = points@recorded.rotation(original['heading_deg']).T+[original['x'], original['y']]
        at = int(np.argmin(np.linalg.norm(f.world-[original['x'], original['y']], axis=1)))-40
        xy, tangent = f.smooth[at], f.smooth[at+2]-f.smooth[at]
        ego = dict(original, x=float(xy[0]), y=float(xy[1]),
                   heading_deg=math.degrees(math.atan2(tangent[1], tangent[0])), speed_kmh=15.)
        runtime, now, speed, travel = f.runtime(), 1_000_000., 15./3.6, 0.
        with patch('time.monotonic', side_effect=lambda: now), patch('time.time', side_effect=lambda: now), \
                patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            for sequence in range(1, 160):
                current_points = f.observe(runtime, ego, now, sequence)
                runtime.object_receiver.objects = objects
                self.assertTrue(runtime.infer())
                payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
                self.assertGreater(payload['target_speed_mps'], .1, runtime._web_record)
                self.assertFalse(payload['decision']['emergency'], payload['decision'])
                self.assertNotEqual(payload.get('gear'), 'R')
                path = (np.array([[p['x'], p['y']] for p in payload['waypoints']])-[ego['x'], ego['y']]) @ recorded.rotation(ego['heading_deg'])
                required = max(4., speed*.7+speed*speed/5.+2.)
                self.assertGreaterEqual(swept_clearance(path, current_points, required, .25,
                    lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), required-.01)
                look = max(3., .45*speed+2.)
                ahead = np.flatnonzero((path[:, 0]>0.) & (np.linalg.norm(path, axis=1)>=look))
                q = path[ahead[0]]
                curvature = float(2.*q[1]/(q@q))
                speed = max(0., speed+float(np.clip((payload['target_speed_mps']-speed)/.25, -3.5, 2.1))*.1)
                h = math.radians(ego['heading_deg'])
                ego['x'] += speed*math.cos(h)*.1
                ego['y'] += speed*math.sin(h)*.1
                ego['heading_deg'] += math.degrees(speed*curvature*.1)
                ego['speed_kmh'] = speed*3.6
                travel += speed*.1
                now += .1
                if travel>40.:
                    break
        self.assertGreater(travel, 40.)


if __name__ == '__main__':
    unittest.main()
