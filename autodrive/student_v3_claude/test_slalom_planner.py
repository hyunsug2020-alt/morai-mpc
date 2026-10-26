"""Static path contracts for staggered obstacles and surveyed pavement limits."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from lane_corridor import LaneCorridor
from lattice_planner import (corridor_side_violation, object_points, plan,
                             offset_profile, swept_clearance)


class SlalomPlannerTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 100.01, .5), np.zeros(201)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=39.)
        self.legal = LaneCorridor(self.route, np.full(201, 1.75), np.full(201, -1.75))
        self.road = LaneCorridor(self.route, np.full(201, 5.25), np.full(201, -1.75))

    @staticmethod
    def box(x, y):
        return (x, y, 1.1, "box", "obstacle", 0., 1.1, 1.1, 0.)

    def choose(self, boxes=(), **kwargs):
        return plan(self.route, self.ego, None, boxes, self.legal,
                    cruise_mps=39. / 3.6, road_corridor=self.road,
                    paved_road_only=True, **kwargs)

    def assert_paved(self, result, road=None):
        for edge in corridor_side_violation(road or self.road, result["path"], result["horizon_m"]):
            self.assertLessEqual(float(edge.max()), .05 + 1e-9)

    def test_recorded_route_laterals_have_a_two_stage_path(self):
        for gap in (5., 7., 10.):
            for swapped in (False, True):
                with self.subTest(gap=gap, swapped=swapped):
                    lat = (1.42, -1.57) if swapped else (-1.57, 1.42)
                    boxes = [self.box(15., lat[0]), self.box(15. + gap, lat[1])]
                    result = self.choose(boxes)
                    self.assertFalse(result["blocked"])
                    self.assertEqual(result["kind"], "slalom")
                    self.assertLessEqual(result["outside_m"], .5)
                    self.assert_paved(result)
                    clear = swept_clearance(result["path"], object_points(boxes, self.ego),
                                             result["horizon_m"], .3, lateral_margin=.3)
                    self.assertGreaterEqual(clear, result["horizon_m"] - .1)

    def test_empty_road_retains_39_kmh_cruise(self):
        result = self.choose()
        self.assertFalse(result["blocked"])
        self.assertAlmostEqual(result["speed_cap_mps"], 39. / 3.6)
        self.assertEqual(result["target"], 0.)

    def test_stopped_escape_retains_the_callers_curvature_limit(self):
        self.ego["speed_kmh"] = 0.
        result = self.choose([self.box(5., -1.57), self.box(12., 1.42)], max_curvature=.1)
        self.assertLessEqual(result["curvature"], .1)
        self.assert_paved(result)

    def test_three_staggered_rows_fit_without_a_shoulder(self):
        boxes = [self.box(15., -1.57), self.box(25., 1.42), self.box(35., -1.57)]
        result = self.choose(boxes)
        self.assertFalse(result["blocked"])
        self.assertEqual(result["kind"], "slalom")
        self.assert_paved(result)
        self.assertGreaterEqual(result["clear_m"], result["horizon_m"] - .1)

    def test_truck_bypass_can_leave_the_legal_lane_within_mapped_pavement(self):
        road = LaneCorridor(self.route, np.full(201, 5.25), np.full(201, -3.25))
        truck = (35., .7, 9., "truck", "obstacle", 0., 9., 2.5, 0.)
        result = plan(self.route, self.ego, None, [truck], self.legal,
                      road_corridor=road, paved_road_only=True, cruise_mps=39. / 3.6)
        self.assertFalse(result["blocked"])
        self.assertGreater(result["outside_m"], .5)
        self.assert_paved(result, road)

    def test_previous_outside_pavement_is_rejected_even_with_escape_allowances(self):
        previous = dict(target=-3.5, world_path=np.column_stack((self.route[:, 0],
                               offset_profile(self.route[:, 0], 0., -3.5, 25.))))
        for outside in (0., 1., 3.):
            with self.subTest(outside=outside):
                result = self.choose(previous=previous, outside_max_m=outside, right_shoulder_m=2.5)
                self.assertNotEqual(result["kind"], "previous")
                self.assertGreater(result["rejected"].get("road_edge", 0), 0)
                self.assert_paved(result)

    def test_previous_and_model_cannot_invent_an_initial_boundary_overrun(self):
        unsafe = np.column_stack((self.route[:, 0], np.full(201, -1.)))
        result = self.choose(previous=dict(target=-1., world_path=unsafe), model_path=unsafe)
        self.assertNotIn(result["kind"], ("previous", "model"))
        self.assert_paved(result)

    def test_previous_is_rechecked_for_new_obstacle(self):
        result = self.choose([self.box(25., 0.)],
                             previous=dict(target=0., world_path=self.route.copy()))
        self.assertNotEqual(result["kind"], "previous")
        self.assertGreater(result["rejected"].get("previous_collision", 0), 0)
        if result["blocked"]:
            self.assertLess(result["clear_m"], 25.)
        else:
            self.assert_paved(result)

    def test_missing_all_map_bounds_stops_instead_of_approving_a_detour(self):
        result = plan(self.route, self.ego, None, [], None,
                      paved_road_only=True, cruise_mps=39. / 3.6)
        self.assertTrue(result["blocked"])
        self.assertEqual(result["speed_cap_mps"], 0.)

    def test_morai_four_object_layout_has_a_paved_zigzag(self):
        from gui import HdMapLanes, local_xy, smooth_control_route
        assets = Path(__file__).resolve().parent
        route_world = np.array([[p["x"], p["y"]] for p in
                                json.loads((assets / "global_path.json").read_text())["waypoints"]])
        route_world = smooth_control_route(route_world)
        lanes = HdMapLanes(assets / "link_set.json")
        lanes.set_route(route_world)
        boxes = [(x, y, 1.1, "box", "obstacle", 0., 1.1, 1.1, 90.) for x, y in
                 [(-58.21, -99.3), (-61.31, -92.4), (-58.53, -81.8), (-61.40, -71.2)]]
        for x, heading in ((-59.9, 87.25), (-59.9, 90.), (-59.9, 95.), (-60.08, 94.97)):
            with self.subTest(x=x, heading=heading):
                ego = dict(x=x, y=-116.18, heading_deg=heading, speed_kmh=39.)
                idx = int(np.argmin(np.linalg.norm(route_world - [ego["x"], ego["y"]], axis=1)))
                route = np.array([local_xy(*p, ego) for p in route_world[idx:idx + 161]])
                legal, road = lanes.corridor(ego, route), lanes.road_corridor(ego, route)
                result = plan(route, ego, None, boxes, legal, road_corridor=road,
                              paved_road_only=True, cruise_mps=39. / 3.6)
                self.assertFalse(result["blocked"])
                self.assertEqual(result["kind"], "slalom")
                self.assertLessEqual(result["outside_m"], .5)
                self.assert_paved(result, road)
                self.assertGreaterEqual(result["clear_m"], result["horizon_m"] - .1)

    def test_tunnel_default_does_not_generate_new_slalom_paths(self):
        with patch("lattice_planner.slalom_profiles") as profiles:
            result = plan(self.route, self.ego, None,
                          [self.box(15., -1.57), self.box(22., 1.42)], self.legal,
                          road_corridor=self.road, max_offset_m=5.)
        profiles.assert_not_called()
        self.assertIn(result["kind"], ("lattice", "lattice_return", "route", "previous"))


class SlalomRuntimeTests(unittest.TestCase):
    def test_mission_and_stalled_retry_enforce_pavement(self):
        from test_lattice_planner import LatticeRuntimeSafetyTests
        import gui
        fixture = LatticeRuntimeSafetyTests()
        for idx in (700, 900, 2000, 3780):
            runtime = fixture.runtime(action="KEEP")
            runtime.test_status["path"]["nearest_idx"] = idx
            runtime._stall_since = 1.
            calls = []
            real_plan = gui.lattice_plan

            def observe(*args, **kwargs):
                calls.append(kwargs)
                result = real_plan(*args, **kwargs)
                result["blocked"] = True
                return result

            with patch("gui.lattice_plan", side_effect=observe):
                fixture.infer(runtime)
            self.assertGreaterEqual(len(calls), 2)
            for call in calls:
                self.assertEqual(call.get("paved_road_only"), 480 <= idx <= 1100)


if __name__ == "__main__":
    unittest.main()
