"""Recorded contest regression for a static obstacle inside a mapped junction."""
import json
import math
from pathlib import Path
import unittest

import numpy as np

from lane_corridor import LaneCorridor
from lattice_planner import corridor_side_violation, plan


class RecordedIntersectionAvoidanceTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).parent / 'test_data/contest_static_intersection_20261008.json'
        self.fixture = json.loads(path.read_text())
        self.route = np.asarray(self.fixture['route'])
        self.road = LaneCorridor(self.route, self.fixture['road_left'], self.fixture['road_right'])

    def run_plan(self, mapped=True, points=None, road=None, previous=None):
        fixture = self.fixture
        corridor = LaneCorridor(self.route, fixture['corridor_left'], fixture['corridor_right'],
                                intersection_mask=fixture['intersection_mask'] if mapped else None)
        result = plan(self.route, fixture['ego'], fixture['points'] if points is None else points,
                      fixture['objects'], corridor, road_corridor=road or self.road,
                      paved_road_only=True, cruise_mps=40. / 3.6, previous=previous)
        return result, corridor

    def previous_path(self):
        result, _ = self.run_plan()
        ego = self.fixture['ego']
        yaw = math.radians(ego['heading_deg'])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        world = result['path'] @ rotation.T + [ego['x'], ego['y']]
        return dict(target=result['target'], world_path=world.tolist())

    def test_recorded_approach_generates_a_moving_bypass(self):
        self.assertEqual(self.fixture['original']['speed_kmh'], 39.4)
        self.assertTrue(self.fixture['original']['lattice']['blocked'])
        result, corridor = self.run_plan()
        self.assertFalse(result['blocked'])
        self.assertFalse(result['emergency'])
        self.assertEqual(result['kind'], 'intersection')
        self.assertGreater(result['target'], 1.)
        self.assertGreater(result['clear_m'], 60.)
        self.assertGreater(result['speed_cap_mps'] * 3.6, 20.)
        self.assertLess(result['speed_cap_mps'] * 3.6, 40.)
        for boundary in (corridor, self.road):
            depths = corridor_side_violation(boundary, result['path'], result['horizon_m'])
            self.assertLessEqual(max(float(edge.max()) for edge in depths), .05)

    def test_unmapped_widening_does_not_remove_the_centre_line(self):
        result, _ = self.run_plan(mapped=False)
        self.assertTrue(result['blocked'])
        self.assertGreater(result['rejected']['centre_line'], 0)

    def test_mapped_junction_still_cannot_cross_the_physical_road_edge(self):
        narrow = LaneCorridor(self.route, np.full(len(self.route), 1.75),
                              np.full(len(self.route), -1.75))
        result, _ = self.run_plan(road=narrow)
        self.assertTrue(result['blocked'])
        self.assertGreater(result['rejected']['road_edge'], 0)

    def test_wall_inside_the_junction_still_blocks_the_bypass(self):
        wall = np.array([(25., y) for y in np.arange(-12., 12.01, .15)])
        result, _ = self.run_plan(points=wall)
        self.assertTrue(result['blocked'])
        self.assertLess(result['speed_cap_mps'] * 3.6, 40.)

    def test_safe_committed_bypass_does_not_repeat_the_full_search(self):
        result, _ = self.run_plan(previous=self.previous_path())
        self.assertFalse(result['blocked'])
        self.assertEqual(result['kind'], 'previous')
        self.assertLessEqual(result['evaluated'], 3)

    def test_new_wall_invalidates_the_committed_bypass(self):
        wall = np.array([(25., y) for y in np.arange(-12., 12.01, .15)])
        result, _ = self.run_plan(points=wall, previous=self.previous_path())
        self.assertTrue(result['blocked'])
        self.assertGreater(result['evaluated'], 3)
        self.assertGreater(result['rejected']['previous_collision'], 0)


class RecordedReturnAvoidanceTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).parent / 'test_data/contest_static_return_20261008.json'
        self.fixture = json.loads(path.read_text())
        fixture = self.fixture
        self.route = np.asarray(fixture['route'])
        self.corridor = LaneCorridor(self.route, fixture['corridor_left'], fixture['corridor_right'],
                                    intersection_mask=fixture['intersection_mask'])
        self.road = LaneCorridor(self.route, fixture['road_left'], fixture['road_right'])

    def run_plan(self, points=None):
        fixture = self.fixture
        return plan(self.route, fixture['ego'], fixture['points'] if points is None else points,
                    fixture['objects'], self.corridor, previous=fixture['previous'],
                    road_corridor=self.road, paved_road_only=True, cruise_mps=40. / 3.6)

    def test_next_obstacle_at_the_swept_front_edge_generates_a_replacement(self):
        self.assertTrue(self.fixture['original']['emergency_stop'])
        result = self.run_plan()
        self.assertFalse(result['blocked'])
        self.assertFalse(result['emergency'])
        self.assertGreater(result['clear_m'], 50.)
        self.assertGreaterEqual(result['speed_cap_mps'] * 3.6, 30.)
        self.assertGreater(result['evaluated'], 3)
        for boundary in (self.corridor, self.road):
            depths = corridor_side_violation(boundary, result['path'], result['horizon_m'])
            self.assertLessEqual(max(float(edge.max()) for edge in depths), .05)

    def test_a_near_wall_still_requires_braking(self):
        wall = np.array([(10., y) for y in np.arange(-12., 12.01, .15)])
        result = self.run_plan(points=wall)
        self.assertTrue(result['blocked'])
        self.assertTrue(result['emergency'])
        self.assertLess(result['speed_cap_mps'] * 3.6, 30.)


if __name__ == '__main__':
    unittest.main()
