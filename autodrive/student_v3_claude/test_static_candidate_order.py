"""Bounded static searches reach the usable side before the frame expires."""
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
from static_lane_pass import StaticLanePass, ordered_static_targets, opposing_corridor, body_stations
from tunnel_runtime import swept_clearance
import test_avoidance_gate as fixtures


class StaticCandidateOrderTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        self.route = np.column_stack((np.arange(0., 100.1, .5), np.zeros(201)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=25., stamp=100.,
                        age_s=.01, measured=True, wheel_angle=0.)
        self.centres = [(5.4, -3.83), (16.1, .96), (39.4, 1.26),
                        (53.2, -4.23), (59.4, .76)]
        self.objects = [(x, y, 1.1, 'fusion_obstacle', 'obstacle', 0.,
                         1.1, 1.1, 0., 0., 0., i)
                        for i, (x, y) in enumerate(self.centres)]
        self.points = np.array([[x+dx, y+dy] for x, y in self.centres
                                for dx in (-.3, 0., .3) for dy in (-.3, 0., .3)])
        self.corridor = LaneCorridor(self.route, np.full(201, 1.75),
            np.full(201, -2.25), body_shift_m=REAR_AXLE_SHIFT_M)
        self.lanes = SimpleNamespace(n_links=0, route_progress=lambda ego: 0.,
                                      rules=lambda: {'sections': []})

    def test_same_side_obstacles_prefer_one_clear_held_offset(self):
        targets = (0., .75, -.75, 1.05, -1.05, .45, -.45, 1.35, -1.35, -1.8)
        ordered = ordered_static_targets(self.route, self.ego, self.objects,
                                         self.corridor, targets)
        self.assertEqual(ordered[0], -1.05)
        self.assertCountEqual(ordered, targets)
        # Input order and duplicate source envelopes cannot change the side.
        self.assertEqual(ordered_static_targets(self.route, self.ego,
            list(reversed(self.objects))*2, self.corridor, targets)[0], -1.05)

    def test_roadside_boxes_do_not_force_a_slalom_between_same_side_hazards(self):
        plan, info = StaticLanePass().plan(self.lanes, self.ego, self.route,
            self.objects, self.objects, self.points, self.corridor, None, 100.,
            budget_s=1., slalom_first=False)
        self.assertIsNotNone(plan, info)
        self.assertFalse(plan['static_borrow'])
        self.assertAlmostEqual(plan['speed_cap_mps']*3.6, 30.)
        self.assertLess(max(gui.path_curvatures(plan['path'])), .02)
        path, horizon = plan['path'], plan['clear_distance_m']
        for row in (16.1, 39.4, 59.4):
            self.assertLess(float(np.interp(row, path[:, 0], path[:, 1])), -.8)
        self.assertGreaterEqual(gui.static_clear_distance(path, self.ego,
            self.objects, horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(swept_clearance(path, self.points, horizon, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(self.corridor.clear_distance(path, horizon), horizon-.01)

    def test_better_order_does_not_admit_an_occupied_front_corridor(self):
        plan, _ = StaticLanePass().plan(self.lanes, self.ego, self.route,
            self.objects, self.objects, np.vstack((self.points, [[3.9, 0.]])),
            self.corridor, None, 100., budget_s=1., slalom_first=False)
        self.assertIsNone(plan)


class RecordedSignalApproachStaticTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        assets = Path(__file__).parent
        with np.load(assets/'test_data/road_static_signal02_20261021.npz') as data:
            self.ego = json.loads(str(data['ego']))
            self.objects = json.loads(str(data['objects']))
            for obj in self.objects:
                obj[3] = 'fusion_obstacle'
            self.route, self.points = data['route'].copy(), data['points'].copy()
        self.lanes = gui.HdMapLanes(assets/'link_set.json')
        world = json.loads((assets/'global_path.json').read_text())['waypoints']
        self.lanes.set_route(np.array([[p['x'], p['y']] for p in world]))
        self.corridor = self.lanes.corridor(self.ego, self.route)

    def replay(self, points):
        runtime = fixtures.AvoidanceGateTests().runtime(
            action='KEEP', objects=self.objects, points=points)
        runtime.lanes = self.lanes
        runtime._contact, runtime._lidar_log = None, dict(age_s=.01)
        lane, clear = ma.road_static_checks(self.corridor, points)
        original = StaticLanePass.plan
        def geometry_budget(policy, *args, **kwargs):
            # Geometry regression must not depend on test-host scheduling.
            kwargs['budget_s'] = 1.
            return original(policy, *args, **kwargs)
        with patch.object(gui, 'model_avoidance', ma, create=True), \
                patch.object(StaticLanePass, 'plan', geometry_budget):
            plan, _, context = runtime.model_static_plan(self.ego, self.route,
                self.objects, points, self.corridor, lane, clear, None,
                {1: 3.2, -1: 3.2}, time.monotonic(), True, True, 1.)
        return plan, context, runtime

    def test_stopped_second_signal_approach_has_a_complete_measured_return(self):
        plan, context, runtime = self.replay(self.points)
        self.assertTrue(ma.static_plan_complete(plan), runtime._model_avoidance_log)
        self.assertFalse(plan['static_borrow'])
        self.assertGreater(plan['speed_cap_mps'], .5)
        horizon = plan['clear_distance_m']
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], self.ego,
            context['static'], horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(swept_clearance(plan['path'], self.points, horizon, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        physical = opposing_corridor(self.lanes, self.ego, self.route, context['corridor'])
        self.assertIsNotNone(physical)
        self.assertGreaterEqual(physical.clear_distance(plan['path'], horizon), horizon-.01)
        _, left, right = body_stations(plan['path'], context['corridor'])
        self.assertLessEqual(float(left.max()), .03)
        self.assertLessEqual(float(right.max()), .03)

    def test_new_front_return_still_rejects_the_recorded_recovery(self):
        plan, _, _ = self.replay(np.vstack((self.points, [[3.9, 0.]])))
        self.assertFalse(ma.static_plan_complete(plan), plan)

    def test_rolling_path_survives_the_next_fusion_observation_and_publication_geometry(self):
        with np.load(Path(__file__).parent/'test_data/road_static_fresh_brake_20261021.npz') as data:
            self.ego = json.loads(str(data['ego']))
            self.objects = json.loads(str(data['objects']))
            fresh = json.loads(str(data['fresh_objects']))
            self.route, self.points = data['route'].copy(), data['points'].copy()
        self.corridor = self.lanes.corridor(self.ego, self.route)
        plan, _, runtime = self.replay(self.points)
        self.assertTrue(ma.static_plan_complete(plan), runtime._model_avoidance_log)
        self.assertGreater(runtime._model_avoidance_log['bounded_pass']['source_envelope_n'], 0)
        published = gui.RosPathPublisher.control_geometry(plan['path'], self.route,
            dict(mode='STUDENT FORCED', validated_static=True, canonical_control=True))
        hit, info = gui.fresh_road_static_hazard(fresh, self.ego, published,
            15., .95, self.points, enabled=True)
        self.assertIsNone(hit, info)
        self.assertGreaterEqual(info['clear_m'], 15.)


class StaticApproachSpeedTests(unittest.TestCase):
    def test_braking_preview_reaches_thirty_before_the_static_pass_without_capping_highway(self):
        zones = gui.SpeedZones.__new__(gui.SpeedZones)
        zones.reload = lambda: None
        zones.base, zones.route_len = 55., 4392
        zones.zones = [dict(from_idx=2251, to_idx=3500, kmh=90.)]
        zones.set_route(np.column_stack((np.arange(4392)*.5, np.zeros(4392))))
        approach = [gui.zone_speed_kmh(zones, idx)[0] for idx in (440, 460, 480)]
        self.assertGreater(approach[0], approach[1])
        self.assertGreater(approach[1], approach[2])
        self.assertEqual(approach[-1], 30.)
        self.assertEqual(gui.zone_speed_kmh(zones, 770)[0], 30.)
        self.assertEqual(gui.zone_speed_kmh(zones, 2500)[0], 90.)


class StaticSlalomPreviewTests(unittest.TestCase):
    def test_measured_opposite_row_outside_activation_prepares_the_complete_s_path(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        route = np.column_stack((np.arange(0., 100.1, .5), np.zeros(201)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=20., stamp=100.,
                   age_s=.01, measured=True, wheel_angle=0.)
        centres = [(18., 1.05), (36., -1.1)]
        objects = [(x, y, 1.1, 'fusion_obstacle', 'obstacle', 0.,
                    1.1, 1.1, 0., 0., 0., uid)
                   for uid, (x, y) in enumerate(centres)]
        points = np.array([[x+dx, y+dy] for x, y in centres
                           for dx in (-.3, 0., .3) for dy in (-.3, 0., .3)])
        corridor = LaneCorridor(route, np.full(201, 2.5), np.full(201, -2.5),
                                 body_shift_m=REAR_AXLE_SHIFT_M)
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=objects, points=points)
        runtime._contact, runtime._lidar_log = None, dict(age_s=.01)
        runtime.lanes.n_links = 0
        lane, clear = ma.road_static_checks(corridor, points)
        hazards, engagement = ma.confirmed_hazards(route, ego, objects,
            body_shift_m=REAR_AXLE_SHIFT_M)
        self.assertEqual(len(hazards), 1)
        self.assertLess(engagement, 35.)
        original = StaticLanePass.plan
        def geometry_budget(policy, *args, **kwargs):
            kwargs['budget_s'] = 1.
            return original(policy, *args, **kwargs)
        with patch.object(gui, 'model_avoidance', ma, create=True), \
                patch.object(StaticLanePass, 'plan', geometry_budget), \
                patch('static_lane_pass.normal_static_area', return_value=True):
            plan, _, context = runtime.model_static_plan(ego, route, objects, points,
                corridor, lane, clear, None, {}, time.monotonic(), True, True, 1.)
        self.assertTrue(runtime._model_avoidance_log['slalom_priority'])
        self.assertTrue(ma.static_plan_complete(plan), runtime._model_avoidance_log)
        horizon = plan['clear_distance_m']
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], ego,
            objects, horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(swept_clearance(plan['path'], points, horizon, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)


if __name__ == '__main__':
    unittest.main()
