"""A bounded search retains only paths verified against the current LiDAR scan."""
import unittest
from unittest.mock import patch

import numpy as np

from gui import plan_static_bypass, static_clear_distance
from lane_corridor import LaneCorridor
from tunnel_runtime import swept_clearance


class TunnelPlanningBudgetTests(unittest.TestCase):
    def setUp(self):
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=12., wheel_angle=0.)
        self.route = np.column_stack((np.arange(0., 65.5, .5), np.zeros(131)))
        self.corridor = LaneCorridor(self.route, np.full(131, 8.5), np.full(131, -2.))
        self.objects = [(30., 0., 6., 'barrier', 'obstacle', 0., 1., 6., 0.)]
        self.previous, _ = plan_static_bypass(self.route, self.ego, self.objects,
            lambda path, _: True, adjacent_offsets={1: 7.},
            corridor_clearance=self.corridor.clear_distance, first_complete=True, full_width=True)
        self.assertFalse(self.previous.get('blocked'))

    def bounded_plan(self, points):
        clock = [0.]
        calls = []
        def measured_clearance(path, horizon):
            clear = min(self.corridor.clear_distance(path, horizon), swept_clearance(path, points, horizon))
            calls.append(clear)
            clock[0] = .2
            return clear
        with patch('gui.time.perf_counter', side_effect=lambda: clock[0]):
            plan, _ = plan_static_bypass(self.route, self.ego, self.objects,
                lambda path, _: True, self.previous['direction'], self.previous,
                adjacent_offsets={1: 7.}, corridor_clearance=measured_clearance,
                first_complete=True, full_width=True, deadline=.1)
        return plan, calls

    def test_budget_retains_a_freshly_checked_safe_prefix(self):
        point = self.previous['path'][40, :2][None, :]
        plan, calls = self.bounded_plan(point)
        self.assertFalse(plan.get('blocked'), plan)
        self.assertTrue(plan['planning_budget_exhausted'])
        self.assertEqual(plan['candidates_evaluated'], 1)
        self.assertEqual(len(calls), 1)
        self.assertGreater(plan['clear_distance_m'], plan['safe_horizon_m'])
        self.assertLess(plan['clear_distance_m'], 30.)
        self.assertGreater(plan['speed_cap_mps'], 0.)
        self.assertEqual(swept_clearance(plan['path'], point, 60.), plan['clear_distance_m'])
        self.assertGreaterEqual(static_clear_distance(plan['path'], self.ego, self.objects, 35.), 35.)

    def test_budget_never_reuses_a_path_newly_occupied_at_the_vehicle(self):
        plan, calls = self.bounded_plan(np.array([[0., 0.]]))
        self.assertTrue(plan['blocked'])
        self.assertTrue(plan['planning_budget_exhausted'])
        self.assertEqual(calls, [0.])
        self.assertNotIn('path', plan)


if __name__ == '__main__':
    unittest.main()
