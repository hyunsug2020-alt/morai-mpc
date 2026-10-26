"""Tunnel candidates use the full corridor without returning toward a hazard."""
import unittest

import numpy as np

from gui import plan_static_bypass, static_clear_distance, static_speed_cap_mps
from lane_corridor import LaneCorridor


class TunnelWidthPlanningTests(unittest.TestCase):
    def test_existing_side_separation_is_retained_until_obstacle_is_passed(self):
        ego = dict(x=0., y=1.2, heading_deg=0., speed_kmh=12., wheel_angle=0.)
        route = np.column_stack((np.arange(0., 65.5, .5), np.full(131, -1.2)))
        corridor = LaneCorridor(route, np.full(131, 8.5), np.full(131, -3.))
        objects = [(22., -.5, .3, "edge_box", "obstacle", 0., .3, .3, 0.)]
        plan, hazards = plan_static_bypass(route, ego, objects, lambda path, _: True,
            adjacent_offsets={1: 7., -1: 1.5}, corridor_clearance=corridor.clear_distance,
            first_complete=True, full_width=True)
        self.assertTrue(hazards)
        self.assertFalse(plan.get("blocked"), plan)
        self.assertGreaterEqual(plan["offset_m"], 1.15)
        self.assertGreaterEqual(static_clear_distance(plan["path"], ego, objects, 35.), 35.)
        before_pass = plan["path"][:, 0] <= 25.
        self.assertGreater(np.min(plan["path"][before_pass, 1]), -.1)

    def test_widths_between_one_lane_and_the_wall_are_checked(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=12., wheel_angle=0.)
        route = np.column_stack((np.arange(0., 65.5, .5), np.zeros(131)))
        corridor = LaneCorridor(route, np.full(131, 8.5), np.full(131, -2.))
        objects = [(30., 0., 6., "wide_barrier", "obstacle", 0., 1., 6., 0.)]
        plan, _ = plan_static_bypass(route, ego, objects, lambda path, _: True,
            adjacent_offsets={1: 7.}, corridor_clearance=corridor.clear_distance,
            first_complete=True, full_width=True)
        self.assertFalse(plan.get("blocked"), plan)
        self.assertGreater(plan["offset_m"], 3.5)
        self.assertLess(plan["offset_m"], 7.)
        self.assertGreaterEqual(static_clear_distance(plan["path"], ego, objects, 35.), 35.)

    def test_slow_or_stopped_car_still_plans_a_rolling_twenty_kmh_bypass(self):
        route = np.column_stack((np.arange(0., 65.5, .5), np.zeros(131)))
        corridor = LaneCorridor(route, np.full(131, 8.5), np.full(131, -2.))
        objects = [(30., 0., 6., "wide_barrier", "obstacle", 0., 1., 6., 0.)]
        for speed in (0., 12., 20.):
            with self.subTest(speed_kmh=speed):
                ego = dict(x=0., y=0., heading_deg=0., speed_kmh=speed, wheel_angle=0.)
                plan, _ = plan_static_bypass(route, ego, objects, lambda path, _: True,
                    adjacent_offsets={1: 7.}, corridor_clearance=corridor.clear_distance,
                    first_complete=True, full_width=True)
                self.assertFalse(plan.get("blocked"), plan)
                self.assertGreaterEqual(static_speed_cap_mps(28.94 / 3.6, plan) * 3.6, 20.)
                self.assertGreaterEqual(static_clear_distance(plan["path"], ego, objects, 35.), 35.)
                self.assertGreaterEqual(corridor.clear_distance(plan["path"], 35.), 35.)


if __name__ == "__main__":
    unittest.main()
