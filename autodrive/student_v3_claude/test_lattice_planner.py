"""Candidate-path planner: collision is the only hard constraint."""
import unittest

import numpy as np

from lane_corridor import LaneCorridor
from lattice_planner import plan


class LatticePlannerTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 120.01, .5), np.zeros(241)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=30.)
        # Legal lanes: own lane plus one lane to the left.
        self.corridor = LaneCorridor(self.route, np.full(241, 5.25), np.full(241, -1.75))

    @staticmethod
    def box(x, y, size=1.):
        return (x, y, size, "box", "obstacle", 0., size, size, 0.)

    def run_plan(self, objects=(), points=None, **kwargs):
        return plan(self.route, self.ego, points, list(objects), self.corridor,
                    cruise_mps=35. / 3.6, **kwargs)

    def test_empty_road_keeps_lane_centre_at_cruise(self):
        result = self.run_plan()
        self.assertEqual(result["target"], 0.)
        self.assertFalse(result["blocked"])
        self.assertAlmostEqual(result["speed_cap_mps"], 35. / 3.6)

    def test_obstacle_in_lane_is_passed_without_stopping(self):
        result = self.run_plan([self.box(30., 0.)])
        self.assertFalse(result["blocked"])
        self.assertGreater(abs(result["target"]), 1.)
        self.assertAlmostEqual(result["speed_cap_mps"], 35. / 3.6, places=3)

    def test_roadside_object_does_not_move_or_slow_the_car(self):
        result = self.run_plan([self.box(30., -3.2, .5)])
        self.assertEqual(result["target"], 0.)
        self.assertAlmostEqual(result["speed_cap_mps"], 35. / 3.6, places=3)

    def test_model_swerve_without_obstacle_is_not_chosen(self):
        x = np.arange(0., 20.01, .5)
        swerve = np.column_stack((x, 2.5 * np.minimum(x / 8., 1.)))
        result = self.run_plan(model_path=swerve)
        self.assertEqual(result["kind"], "lattice")
        self.assertEqual(result["target"], 0.)

    def test_dynamic_pace_prefers_legal_micro_bypass_over_crawling_centre(self):
        def checked_speed(path, horizon):
            return .5 if abs(np.interp(18., path[:, 0], path[:, 1])) < .6 else 5.
        result = self.run_plan(extra_clearance=lambda path, horizon: horizon,
            extra_speed_cap=checked_speed, outside_max_m=0.)
        self.assertFalse(result['blocked'])
        self.assertGreater(result['target'], .5)
        self.assertGreater(result['speed_cap_mps'], .5)
        self.assertLessEqual(result['speed_cap_mps'], 5.)
        self.assertLessEqual(result['outside_m'], .01)

    def test_dynamic_pace_caps_motion_when_no_faster_legal_path_exists(self):
        result = self.run_plan(extra_clearance=lambda path, horizon: horizon,
            extra_speed_cap=lambda path, horizon: 2., outside_max_m=0.)
        self.assertFalse(result['blocked'])
        self.assertEqual(result['target'], 0.)
        self.assertAlmostEqual(result['speed_cap_mps'], 2.)

    def test_dynamic_pace_rechecks_slow_committed_path_before_keeping_it(self):
        import time
        previous = dict(stamp=time.monotonic(), target=0., world_path=self.route)
        def checked_speed(path, horizon):
            return .5 if abs(np.interp(18., path[:, 0], path[:, 1])) < .6 else 5.
        result = plan(self.route, self.ego, None, [], self.corridor, previous,
            cruise_mps=35./3.6, paved_road_only=True, outside_max_m=0.,
            extra_clearance=lambda path, horizon: horizon, extra_speed_cap=checked_speed)
        self.assertFalse(result['blocked'])
        self.assertGreater(result['target'], .5)
        self.assertGreater(result['speed_cap_mps'], .5)

    def test_wall_across_the_road_brakes_before_it(self):
        wall = np.array([(25., y) for y in np.arange(-8., 8.01, .2)])
        result = self.run_plan(points=wall)
        self.assertTrue(result["blocked"])
        self.assertLess(result["speed_cap_mps"], 35. / 3.6)
        self.assertLess(result["clear_m"], 25.)

    def test_physical_wall_shortens_clearance_before_candidate_comparison(self):
        self.ego['speed_kmh'] = 20.
        corridor = LaneCorridor(self.route, np.full(241, 6.), np.full(241, -6.))

        def physical_wall(path, horizon):
            # The right-hand passage narrows beyond 22 m. The left remains
            # clear through the obstacle and the entire planning horizon.
            return min(horizon, 22.) if np.interp(30., path[:, 0], path[:, 1]) < 0. else horizon

        result = plan(self.route, self.ego, None, [self.box(30., 0.)], corridor,
                      cruise_mps=35. / 3.6, extra_clearance=physical_wall)
        self.assertFalse(result['blocked'])
        self.assertGreater(result['target'], 0.)
        self.assertAlmostEqual(result['clear_m'], result['horizon_m'])

    def test_map_rule_is_a_cost_not_a_stop(self):
        # Only the own lane is legal; a box fills it. Slight encroachment is
        # allowed rather than stopping (single-lane CargoBox case).
        narrow = LaneCorridor(self.route, np.full(241, 1.75), np.full(241, -1.75))
        result = plan(self.route, self.ego, None, [self.box(30., -1.0, 1.)], narrow,
                      cruise_mps=35. / 3.6)
        self.assertFalse(result["blocked"])
        self.assertGreater(result["outside_m"], 0.)


    def test_bypass_uses_right_shoulder_never_the_centre_line(self):
        # Own lane only; the oncoming lane (yellow line) is on the left, a
        # paved shoulder on the right. A box on the left half of the lane is
        # passed on the right, never across the centre line (user rule).
        narrow = LaneCorridor(self.route, np.full(241, 1.75), np.full(241, -1.75))
        road = LaneCorridor(self.route, np.full(241, 5.25), np.full(241, -2.25))
        for box in (self.box(30., .9, 1.), self.box(30., .5, 1.6)):
            result = plan(self.route, self.ego, None, [box], narrow,
                          cruise_mps=35. / 3.6, road_corridor=road)
            path = np.asarray(result["path"])
            self.assertFalse(result["blocked"])
            self.assertLessEqual(float(path[:, 1].max()), 1.75 - .946 + .06)
            self.assertGreaterEqual(float(path[:, 1].min()), -2.25 - 1.2 + .946 - .06)
            self.assertLess(float(path[:, 1].min()), -.5)


class LatticeRuntimeSafetyTests(unittest.TestCase):
    """The legacy runtime safety contracts, checked with the lattice planner."""
    def runtime(self, **kwargs):
        import test_avoidance_gate as fixture
        return fixture.AvoidanceGateTests().runtime(**kwargs)

    def infer(self, runtime):
        from unittest.mock import patch
        import json
        with patch("gui.USE_LATTICE", True):
            self.assertTrue(runtime.infer())
        record = runtime._web_record
        waypoints = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)["waypoints"]
        return record, waypoints

    def test_empty_road_ignores_model_avoidance(self):
        for action in ("YIELD", "LEFT_CHANGE", "RIGHT_CHANGE"):
            with self.subTest(action=action):
                record, waypoints = self.infer(self.runtime(action=action))
                self.assertEqual(record["mode"], "LATTICE")
                self.assertTrue(all(abs(p["y"]) < 1e-3 for p in waypoints))
                self.assertFalse(record["emergency_stop"])

    def test_real_object_is_passed_on_a_checked_offset(self):
        box = (18., 0., 1., "box", "obstacle", 0., 1., 1., 0.)
        runtime = self.runtime(action="KEEP", objects=[box])
        runtime.test_status["path"]["nearest_idx"] = 700      # obstacle mission
        record, waypoints = self.infer(runtime)
        self.assertGreater(max(abs(p["y"]) for p in waypoints), 1.)
        self.assertFalse(record["emergency_stop"])

    def test_static_objects_are_ignored_outside_the_obstacle_mission(self):
        # Rule 3-2: obstacles only in the avoidance section; elsewhere curbs
        # and painted islands stopped the car. Vehicles are still handled.
        box = (18., 0., 1., "box", "obstacle", 0., 1., 1., 0.)
        runtime = self.runtime(action="KEEP", objects=[box])
        runtime.test_status["path"]["nearest_idx"] = 2000
        record, waypoints = self.infer(runtime)
        self.assertTrue(all(abs(p["y"]) < 1e-3 for p in waypoints))

    def test_stale_object_stream_still_stops(self):
        runtime = self.runtime()
        runtime.object_receiver.age_s = .6
        record, _ = self.infer(runtime)
        self.assertTrue(record["emergency_stop"])
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_moving_pedestrian_still_stops(self):
        pedestrian = (6., 0., .6, "person", "pedestrian", 1., .6, .6, 90., 0., 1.)
        runtime = self.runtime(objects=[pedestrian])
        runtime.test_status["path"]["nearest_idx"] = 900   # dynamic-obstacle section
        record, _ = self.infer(runtime)
        self.assertTrue(record["emergency_stop"])


class OffLaneVehicleFilterTests(unittest.TestCase):
    def test_stationary_box_off_every_lane_is_not_a_car(self):
        from types import SimpleNamespace
        from scipy.spatial import cKDTree
        from gui import Runtime
        runtime = Runtime.__new__(Runtime)
        lane = np.column_stack((np.arange(0., 50., .5), np.zeros(100)))
        runtime.lanes = SimpleNamespace(n_links=1, tree=cKDTree(lane))
        def car(x, y, vx):
            return (x, y, 5., "lidar_car", "npc", abs(vx), 4.9, 2.1, 0., vx, 0., 1, 0., None, True)
        island = car(10., 2.3, .8)        # roundabout island structure
        parked = car(20., .4, 0.)         # stopped on the lane: a real car
        changing = car(30., 2.0, 8.)      # moving between lanes
        kept = runtime.on_road_vehicles([island, parked, changing])
        self.assertNotIn(island, kept)
        self.assertIn(parked, kept)
        self.assertIn(changing, kept)


class DynamicCautionTests(unittest.TestCase):
    def test_person_near_the_road_slows_us_down(self):
        from gui import dynamic_caution_mps, DYNAMIC_CAUTION_MAX_KMH
        ego = dict(x=0., y=0., heading_deg=0.)
        person = (20., 1.5, .6, "person", "pedestrian", 0., .6, .6, 0.)   # within PASS_CLEAR_M of the path
        cap = dynamic_caution_mps([person], ego)
        self.assertIsNotNone(cap)
        self.assertLessEqual(cap, DYNAMIC_CAUTION_MAX_KMH / 3.6 + 1e-9)
        close = dynamic_caution_mps([(8., 1.5, .6, "person", "pedestrian", 1., .6, .6, 90.)], ego)
        self.assertLess(close, cap)                       # closer -> slower
        animal = (15., -3., .8, "obstacle", "obstacle", 1.2, .8, .5, 90.)
        self.assertIsNotNone(dynamic_caution_mps([animal], ego))

    def test_static_box_or_far_person_does_not(self):
        from gui import dynamic_caution_mps
        ego = dict(x=0., y=0., heading_deg=0.)
        self.assertIsNone(dynamic_caution_mps([(15., 0., 1., "box", "obstacle", 0., 1., 1., 0.)], ego))
        car_as_obstacle = (15., -3., 4.6, "fusion_obstacle", "obstacle", .8, 4.6, 1.9, 0.)
        self.assertIsNone(dynamic_caution_mps([car_as_obstacle], ego))
        moving_car = (15., -3., 4.6, "lidar_car", "npc", 8., 4.6, 1.9, 0.)
        self.assertIsNone(dynamic_caution_mps([moving_car], ego))
        passed = (1.0, 2.5, .6, "person", "pedestrian", .8, .6, .6, 90.)   # beside the car
        self.assertIsNone(dynamic_caution_mps([passed], ego))
        behind = (-5., 3., .6, "person", "pedestrian", .8, .6, .6, 90.)
        self.assertIsNone(dynamic_caution_mps([behind], ego))
        self.assertIsNone(dynamic_caution_mps([(60., 0., .6, "person", "pedestrian", 1., .6, .6, 0.)], ego))
        self.assertIsNone(dynamic_caution_mps([(20., 12., .6, "person", "pedestrian", 1., .6, .6, 0.)], ego))

class StaticMemoryTests(unittest.TestCase):
    def test_flickering_box_is_remembered_in_the_mission_only(self):
        from gui import Runtime
        runtime = Runtime.__new__(Runtime)
        box = (30., 0., 1., "box", "obstacle", 0., 1., 1., 0.)
        self.assertEqual(runtime.remember_static_objects([box], 0., True), [box])
        self.assertEqual(runtime.remember_static_objects([], .5, True), [box])     # dropped frame
        self.assertEqual(runtime.remember_static_objects([], 10., True), [])       # expired
        runtime.remember_static_objects([box], 11., True)
        self.assertEqual(runtime.remember_static_objects([], 11.5, False), [])     # outside mission

    def test_standing_person_beside_the_road_does_not_crawl(self):
        from gui import dynamic_caution_mps
        ego = dict(x=0., y=0., heading_deg=0.)
        beside = (3.2, 3.7, .6, "person", "pedestrian", 0., .6, .6, 0.)
        # Beside the road and not approaching: just passed, no caution cap (user 2026-10-05).
        self.assertIsNone(dynamic_caution_mps([beside], ego))
        crossing = (3.2, 1.0, .6, "person", "pedestrian", 1.2, .6, .6, 90.)
        self.assertLess(dynamic_caution_mps([crossing], ego), 5. / 3.6)


if __name__ == "__main__":
    unittest.main()
