"""Current-pose road margin recovery never reuses an old pose's allowance."""
import copy
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from perception.stationary_semantics import metadata, with_metadata
from tunnel_runtime import swept_clearance


class RoadStaticMemoryRecoveryTests(unittest.TestCase):
    def setUp(self):
        with np.load(Path(__file__).parent/'test_data/road_static_memory_recovery_20261016.npz') as data:
            self.ego = json.loads(str(data['frame18_ego']))
            self.objects = json.loads(str(data['frame18_static']))
            self.points = data['frame18_points'].copy()
            self.route = data['frame18_route'].copy()
            self.corridor = LaneCorridor(self.route, data['frame18_left'], data['frame18_right'])
        self.corridor.body_shift_m = REAR_AXLE_SHIFT_M
        self.box = next(o for o in self.objects if o[3] == 'fusion_obstacle' and o[11] == 7)
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    def floor(self, obj):
        return metadata(obj).get('road_static_geometry', {}).get('initial_margin_floor_m')

    def test_recorded_blind_spot_recovers_only_the_current_margin_deficit(self):
        original = copy.deepcopy(self.objects)
        checked = ma.road_static_margin_recovery(self.objects, self.ego, self.points)
        box = next(o for o in checked if o[3] == 'fusion_obstacle' and o[11] == 7)
        self.assertAlmostEqual(self.floor(box), .18085004563724416)
        self.assertEqual(self.objects, original)
        for before, after in zip(self.objects, checked):
            self.assertEqual(before[:17], list(after[:17]))
        path, _, _ = ma._static.build_forced_bypass(self.route, 1., offset_m=.3,
            hold_m=6., transitions=(14.,), max_curvature_1pm=gui.MAX_CONTROL_CURVATURE)
        path = path[:, :2]
        path = gui.RosPathPublisher.control_geometry(
            np.vstack((np.zeros((1, 2)), path[np.flatnonzero(path[:, 0] > .05)[0]:])),
            self.route, dict(mode='LATTICE', validated_static=True))
        self.assertEqual(gui.static_clear_distance(path, self.ego, self.objects, 35.,
            body_shift_m=REAR_AXLE_SHIFT_M), 0.)
        self.assertEqual(gui.static_clear_distance(path, self.ego, checked, 35.,
            body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        self.assertEqual(swept_clearance(path, self.points, 35., .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        self.assertEqual(self.corridor.clear_distance(path, 35.), 35.)
        self.assertLessEqual(max(gui.path_curvatures(path)), gui.MAX_CONTROL_CURVATURE)

    def test_an_old_floor_is_recomputed_at_the_new_pose(self):
        checked, = ma.road_static_margin_recovery([self.box], self.ego, self.points)
        heading = math.radians(self.ego['heading_deg'])
        ego = dict(self.ego, x=self.ego['x']+.01*math.sin(heading),
                   y=self.ego['y']-.01*math.cos(heading))
        updated, = ma.road_static_margin_recovery([checked], ego, self.points)
        self.assertAlmostEqual(self.floor(updated), self.floor(checked)+.01)
        # Moving 10 cm toward this box cannot reuse either earlier allowance.
        ego = dict(self.ego, x=self.ego['x']-.1*math.sin(heading),
                   y=self.ego['y']+.1*math.cos(heading))
        blocked, = ma.road_static_margin_recovery([checked], ego, self.points)
        self.assertIsNone(self.floor(blocked))

    def test_missing_invalid_or_close_raw_points_revoke_the_old_allowance(self):
        checked, = ma.road_static_margin_recovery([self.box], self.ego, self.points)
        for points in (None, np.empty((0, 2)), np.array([[np.nan, 0.]]),
                       np.zeros((1, 3)), np.array([[3.9, 0.]])):
            with self.subTest(points=points):
                revoked, = ma.road_static_margin_recovery([checked], self.ego, points)
                self.assertIsNone(self.floor(revoked))
                self.assertEqual(revoked[:17], checked[:17])
        revoked, = ma.road_static_margin_recovery([checked], {}, self.points)
        self.assertIsNone(self.floor(revoked))

    def test_physical_front_blocker_people_and_unverified_geometry_get_no_allowance(self):
        checked, = ma.road_static_margin_recovery([self.box], self.ego, self.points)
        h = math.radians(self.ego['heading_deg'])
        front = list(checked)
        front[:2] = (self.ego['x']+3.9*math.cos(h), self.ego['y']+3.9*math.sin(h))
        person, tunnel = list(checked), list(checked)
        person[3:5], tunnel[3] = ['fusion_pedestrian', 'pedestrian'], 'tunnel_lidar_0'
        geometry = metadata(checked)['road_static_geometry']
        unknown = with_metadata(checked, road_static_geometry=dict(geometry, source='unknown'))
        changed = list(checked)
        changed[6] += .1
        for obj in (front, person, tunnel, unknown, changed):
            with self.subTest(kind=obj[3]):
                revoked, = ma.road_static_margin_recovery([obj], self.ego, self.points)
                self.assertIsNone(self.floor(revoked))
                self.assertEqual(list(revoked[:17]), list(obj[:17]))

    def test_recovery_does_not_refresh_memory_age_or_change_its_physical_box(self):
        checked, = ma.road_static_margin_recovery([self.box], self.ego, self.points)
        runtime = SimpleNamespace()
        gui.Runtime.remember_static_objects(runtime, [checked], 100., True)
        self.assertIsNone(self.floor(runtime._static_object_memory[0][1]))
        remembered = gui.Runtime.remember_static_objects(runtime, [], 102.9, True)
        result = ma.road_static_margin_recovery(remembered, self.ego, self.points)
        self.assertAlmostEqual(self.floor(result[0]), .18085004563724416)
        self.assertEqual(runtime._static_object_memory[0][0], 100.)
        self.assertEqual(list(result[0][:17]), list(checked[:17]))
        self.assertEqual(gui.Runtime.remember_static_objects(runtime, [], 103.01, True), [])


class RoadComfortObjectTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    def test_recorded_compact_raw_cluster_uses_fusion_prior_only_in_the_preference(self):
        with np.load(Path(__file__).parent/'test_data/road_static_memory_recovery_20261016.npz') as data:
            ego = json.loads(str(data['frame19_ego']))
            points = data['frame19_points']
            later = next(o for o in json.loads(str(data['frame20_objects']))
                         if o[3] == 'fusion_obstacle' and o[11] == 9)
            later, = ma.road_static_objects([later], json.loads(str(data['frame20_ego'])),
                                           data['frame20_points'])
        hard = ma.road_lidar_objects(points, ego)
        before = copy.deepcopy(hard)
        soft = ma.road_comfort_objects(hard)
        index = min(range(len(hard)), key=lambda i: math.dist(hard[i][:2], later[:2]))
        np.testing.assert_allclose(hard[index][6:8], [.3, .3])
        np.testing.assert_allclose(soft[index][6:8], [.9, .9])
        np.testing.assert_allclose(soft[index][6:8], later[6:8], atol=1e-6)
        self.assertEqual(hard, before)
        self.assertEqual(hard[index][:2], soft[index][:2])
        self.assertEqual(hard[index][3:6], soft[index][3:6])
        self.assertEqual(hard[index][8:], soft[index][8:])

    def test_recorded_hard_safe_path_stays_available_when_the_preference_fails(self):
        from tunnel_runtime import corridor_points
        with np.load(Path(__file__).parent/'test_data/road_static_padding_20261015.npz') as data:
            ego = json.loads(str(data['ego']))
            objects = json.loads(str(data['objects']))
            path, points = data['path'], data['points']
            corridor = LaneCorridor(data['route'], data['left'], data['right'])
        hard = objects[:1]+ma.merge_lidar_obstacles(objects[:1],
            ma.road_lidar_objects(corridor_points(points, corridor), ego))
        before = copy.deepcopy(hard)
        soft = ma.road_comfort_objects(hard)
        self.assertGreater(gui.static_clear_distance(path, ego, hard, 60.,
            body_shift_m=REAR_AXLE_SHIFT_M), 58.)
        self.assertEqual(gui.static_clear_distance(path, ego, soft, 60.,
            body_shift_m=REAR_AXLE_SHIFT_M), .25)
        self.assertEqual(hard, before)

    def test_semantic_people_motion_guardrails_and_unknown_labels_keep_their_geometry(self):
        from tunnel_runtime import lidar_objects
        raw, = lidar_objects(np.array([[10., 0.]]), dict(x=0., y=0., heading_deg=0.),
                             bounds_padding_m=.05)
        rows = []
        for name, kind, speed, size in (
                ('fusion_obstacle', 'obstacle', 0., (.9, .9)),
                ('tunnel_lidar_0', 'pedestrian', 0., (.3, .3)),
                ('tunnel_lidar_0', 'obstacle', .3, (.3, .3)),
                ('tunnel_lidar_0', 'obstacle', 0., (7., .3)),
                ('tunnel_lidar_0', 'obstacle', 0., (1.8, .3)),
                ('tunnel_lidar_custom', 'obstacle', 0., (.3, .3)),
                ('tunnel_lidar_0', 'obstacle', 0., (1.4, 1.2))):
            row = list(raw)
            row[3:6], row[6:8], row[2] = [name, kind, speed], list(size), max(size)
            rows.append(tuple(row))
        checked = ma.road_comfort_objects(rows)
        for before, after in zip(rows, checked):
            self.assertIs(before, after)

    def test_recorded_preferred_candidate_still_passes_current_raw_map_and_sat(self):
        with np.load(Path(__file__).parent/'test_data/road_static_memory_recovery_20261016.npz') as data:
            ego = json.loads(str(data['frame19_ego']))
            objects = json.loads(str(data['frame19_static']))
            points, route = data['frame19_points'], data['frame19_route']
            corridor = LaneCorridor(route, data['frame19_left'], data['frame19_right'])
        corridor.body_shift_m = REAR_AXLE_SHIFT_M
        lane, _ = ma.road_static_checks(corridor, points)
        _, comfortable = ma.road_static_checks(corridor, points, margin_m=.45)
        soft = ma.road_comfort_objects(objects)

        def canonical(path):
            path = np.asarray(path)[:, :2]
            first = np.flatnonzero(path[:, 0] > .05)[0]
            return gui.RosPathPublisher.control_geometry(
                np.vstack((np.zeros((1, 2)), path[first:])), route,
                dict(mode='LATTICE', validated_static=True))

        def clear(path, horizon=60.):
            path = canonical(path)
            if max(gui.path_curvatures(path)) > gui.MAX_CONTROL_CURVATURE:
                return 0.
            return min(comfortable(path, horizon), gui.static_clear_distance(
                path, ego, soft, horizon, body_shift_m=REAR_AXLE_SHIFT_M))

        plan, _, _ = ma.plan_bypass(route, ego, objects,
            lambda path, tolerance: lane(canonical(path), tolerance), clear,
            adjacent_offsets={}, body_shift_m=REAR_AXLE_SHIFT_M, budget_s=2.)
        self.assertTrue(ma.static_plan_complete(plan), plan)
        path = canonical(plan['path'])
        self.assertEqual(gui.static_clear_distance(path, ego, objects, 35.,
            body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        self.assertEqual(swept_clearance(path, points, 35., .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        self.assertEqual(corridor.clear_distance(path, 35.), 35.)


if __name__ == '__main__':
    unittest.main()
