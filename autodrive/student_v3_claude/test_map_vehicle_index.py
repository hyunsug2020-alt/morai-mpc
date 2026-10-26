"""A spatial prefilter must preserve the full global map-matching result."""
import copy
import json
import math
from pathlib import Path
import unittest

import numpy as np

from traffic_planner import MapVehiclePredictor


class MapVehicleIndexTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.links = json.loads((Path(__file__).parent/'link_set.json').read_text())
        cls.real = MapVehiclePredictor(cls.links)

    def assert_global_equivalent(self, predictor, position, heading=None, **limits):
        # Force the complete global scan as the independent candidate oracle;
        # both sides retain the original exact radius/heading/branch predicates.
        global_scan = copy.copy(predictor)
        global_scan._lane_point_tree = None
        expected = global_scan.lane_paths(position, heading, **limits)
        actual = predictor.lane_paths(position, heading, **limits)
        self.assertEqual(len(actual), len(expected))
        for (xy, arc), (expected_xy, expected_arc) in zip(actual, expected):
            np.testing.assert_array_equal(xy, expected_xy)
            np.testing.assert_array_equal(arc, expected_arc)
        return actual

    def test_exact_two_metre_boundary_and_heading_wrap_keep_global_predicates(self):
        predictor = MapVehiclePredictor([dict(points=[[x, 0.] for x in range(6)],
                                            from_node_idx='a', to_node_idx='b')])
        for radius in (2.-1e-10, 2., 2.+1e-10):
            for heading in (None, 0., 2.*math.pi, math.radians(40.),
                            np.nextafter(math.radians(40.), math.inf),
                            np.nextafter(math.radians(40.), -math.inf), math.pi):
                self.assert_global_equivalent(predictor, (2., radius), heading)
        self.assertEqual(len(predictor.lane_paths((2., 2.), 0.)), 1)
        self.assertFalse(predictor.lane_paths((2., 2.+1e-10), 0.))

    def test_equal_distance_ties_and_branch_caps_keep_original_point_and_link_order(self):
        predictor = MapVehiclePredictor([
            dict(points=[[0., 0.], [1., 0.], [2., 0.]], from_node_idx='a', to_node_idx='b'),
            dict(points=[[2., 0.], [3., 1.], [4., 2.]], from_node_idx='b', to_node_idx='c'),
            dict(points=[[2., 0.], [3., -1.], [4., -2.]], from_node_idx='b', to_node_idx='d'),
            dict(points=[[0., 0.], [1., 0.], [2., 0.]], from_node_idx='x', to_node_idx='y')])
        for branches in (1, 2, 3, 16):
            actual = self.assert_global_equivalent(predictor, (1.5, 1.), None,
                                                   max_arc=70., max_branches=branches)
            np.testing.assert_array_equal(actual[0][0][0], [1., 0.])
            np.testing.assert_array_equal(actual[0][0][-1], [4., 2.])

    def test_nonadjacent_duplicate_map_points_choose_the_first_global_index(self):
        predictor = MapVehiclePredictor([dict(points=[[0., 0.], [1., 0.], [0., 0.]],
                                            from_node_idx='a', to_node_idx='b')])
        actual = self.assert_global_equivalent(predictor, (0., 0.))
        np.testing.assert_array_equal(actual[0][0], [[0., 0.], [1., 0.], [0., 0.]])

    def test_empty_nonfinite_or_legacy_broadcast_queries_preserve_full_scan_fallback(self):
        self.assertFalse(MapVehiclePredictor([]).lane_paths((0., 0.)))
        for position, heading in (((math.nan, 0.), None), ((math.inf, 0.), None),
                                  ((0., 0.), math.nan), ([1.], None)):
            self.assert_global_equivalent(self.real, position, heading)
        predictor = copy.copy(self.real)
        predictor._lane_point_tree = None
        self.assert_global_equivalent(predictor, self.real.points[300], None)

    def test_external_map_edits_fall_back_including_inplace_coordinate_changes(self):
        predictor = MapVehiclePredictor([dict(points=[[0., 0.], [1., 0.], [2., 0.], [3., 0.]],
                                            from_node_idx='a', to_node_idx='b')])
        predictor.points[0] = [100., 100.]
        self.assert_global_equivalent(predictor, (0., 0.))
        predictor.points = predictor.points.copy()
        predictor.points[1] = [-10., -10.]
        self.assert_global_equivalent(predictor, (1., 0.))

    def test_recorded_junction_sources_keep_all_unknown_and_directed_lane_branches(self):
        fixture = json.loads((Path(__file__).parent/'test_data/junction_native36_source_motion.json').read_text())
        for frame in fixture['frames'][::2]:
            for row in frame['cars']:
                velocity = row['velocity']
                heading = math.atan2(velocity[1], velocity[0]) if math.hypot(*velocity) > .3 else None
                for query_heading in (heading, None):
                    self.assert_global_equivalent(self.real, row['position'], query_heading)
        native39 = json.loads((Path(__file__).parent/'test_data/junction_native39_parked_alias.json').read_text())
        for row in (native39['pp'], native39['camera']):
            self.assert_global_equivalent(self.real, row['position'], None)

    def test_highway_entry_and_diverging_links_preserve_all_global_paths(self):
        ids = {'A2256W000411', 'A2256W000409', 'A2256W000410', 'A2256W000434', 'A2256W000435'}
        tested = 0
        for link in self.links:
            if str(link.get('idx', link.get('id'))) not in ids:
                continue
            points = np.asarray(link['points'], float)[:, :2]
            for index in (0, len(points)//2, len(points)-2):
                direction = points[min(index+1, len(points)-1)]-points[max(0, index-1)]
                heading = math.atan2(direction[1], direction[0])
                for query_heading in (heading, None):
                    self.assert_global_equivalent(self.real, points[index], query_heading)
                    tested += 1
        self.assertGreaterEqual(tested, 18)

    def test_seeded_near_boundary_queries_over_the_complete_global_map(self):
        generator = np.random.default_rng(73142)
        indices = generator.choice(len(self.real.points), size=30, replace=False)
        for count, index in enumerate(indices):
            angle = generator.uniform(-math.pi, math.pi)
            radius = (0., 1.99, 2., 2.+1e-10, 2.-1e-10)[count % 5]
            position = self.real.points[index]+radius*np.array([math.cos(angle), math.sin(angle)])
            for heading in (None, float(self.real.headings[index])+math.radians(40.),
                            float(self.real.headings[index])-math.radians(40.)):
                self.assert_global_equivalent(self.real, position, heading, max_branches=3)


if __name__ == '__main__':
    unittest.main()
