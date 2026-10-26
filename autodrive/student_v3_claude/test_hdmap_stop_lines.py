"""Surveyed junction stop nodes and continuous route progress regressions."""
import json
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from gui import HdMapLanes
from ego_body import EGO_FRONT_M
from stop_line import competition_stop_controller


class HdMapStopLineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets = Path(__file__).parent
        cls.route = np.array([[p['x'], p['y']] for p in
                              json.loads((cls.assets/'global_path.json').read_text())['waypoints']])
        cls.lanes = HdMapLanes(cls.assets/'link_set.json')
        cls.lanes.set_route(cls.route)

    def test_actual_junction_entrance_nodes_exclude_exits_and_tunnel_gate(self):
        nodes = {line['node_id'] for line in self.lanes.stop_lines}
        self.assertEqual(len(nodes), 10)
        self.assertIn('A1256W000709', nodes)
        for excluded in ('A1256W000728', 'A1256W000616', 'A1256W000831', 'A1256W000674'):
            self.assertNotIn(excluded, nodes)
        self.assertTrue(all(line['source'] == 'hdmap_node' for line in self.lanes.stop_lines))

    def test_recent_collision_entrance_uses_exact_surveyed_position(self):
        line = next(line for line in self.lanes.stop_lines if line['node_id'] == 'A1256W000709')
        np.testing.assert_allclose(line['xy'], [-99.31267240812303, 317.7989547071047], atol=1e-9)
        self.assertEqual(line['route_idx'], 1781)
        self.assertAlmostEqual(line['station_m'], self.lanes.route_station[1781], places=6)
        self.assertEqual(line['id'], 'A1256W000709:1781')

    def test_competition_accepts_first_checked_gap_only_at_requested_intersection(self):
        for line in self.lanes.stop_lines:
            controller = competition_stop_controller(self.lanes.stop_lines)
            station = line['station_m'] - EGO_FRONT_M - 1.
            decision = controller.update(station, 0., 0., clear=True,
                                         empty_front_station=station+EGO_FRONT_M)
            with self.subTest(node=line['node_id']):
                if line['node_id'] == 'A1256W000709':
                    self.assertEqual(decision['state'], 'APPROACH_YIELD')
                    self.assertFalse(decision['stop'])
                    self.assertEqual(decision['required_hold_s'], 0.)
                else:
                    self.assertIsNone(decision)

    def test_competition_moves_without_mandatory_stop_and_yields_only_for_traffic(self):
        controller = competition_stop_controller(self.lanes.stop_lines)
        line = controller.lines[0]
        station = line['station_m']-EGO_FRONT_M-.5
        front = station+EGO_FRONT_M
        self.assertFalse(controller.update(station, .3, 0., True,
                                          empty_front_station=front)['stop'])
        held = controller.update(station, 0., .1, False, empty_front_station=front)
        self.assertEqual(held['state'], 'YIELD_WAIT')
        self.assertTrue(held['stop'])
        self.assertEqual(controller.update(station, 0., .2, True,
                                           empty_front_station=front)['state'], 'APPROACH_YIELD')
        self.assertEqual(controller.update(station+.51, .3, .3, True,
                                           empty_front_station=front+.51)['state'], 'APPROACH_YIELD')
        self.assertFalse(controller.released)

    def test_missing_requested_stop_does_not_expand_to_other_stop_lines(self):
        other_lines = [line for line in self.lanes.stop_lines if line['node_id'] != 'A1256W000709']
        with self.assertRaisesRegex(ValueError, 'A1256W000709'):
            competition_stop_controller(other_lines)

    def test_progress_keeps_fractional_position_for_bumper_stop_distance(self):
        index, fraction = 1750, .37
        point = self.route[index]+fraction*(self.route[index+1]-self.route[index])
        ego = dict(x=point[0], y=point[1])
        expected = self.lanes.route_station[index]+fraction*np.linalg.norm(
            self.route[index+1]-self.route[index])
        self.assertAlmostEqual(self.lanes.route_progress(ego), expected, places=7)

    def test_crossing_route_projection_respects_measured_heading(self):
        lanes = HdMapLanes.__new__(HdMapLanes)
        lanes.route_xy = np.array([[-5., 0.], [0., 0.], [5., 0.], [5., -5.],
                                   [0., -5.], [0., 0.], [0., 5.]])
        lanes.route_tree = cKDTree(lanes.route_xy)
        lanes.route_station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(lanes.route_xy, axis=0), axis=1))]
        self.assertAlmostEqual(lanes.route_progress(dict(x=0., y=0., heading_deg=90.)), 25.)

    def test_repeated_route_entrances_have_unique_ids_and_local_station(self):
        lanes = HdMapLanes(self.assets/'link_set.json')
        lanes.set_route(np.vstack((self.route, self.route)))
        self.assertEqual(len(lanes.stop_lines), 20)
        self.assertEqual(len({line['id'] for line in lanes.stop_lines}), 20)
        same_node = [line for line in lanes.stop_lines if line['node_id'] == 'A1256W000709']
        self.assertEqual(len(same_node), 2)
        self.assertGreater(same_node[1]['station_m']-same_node[0]['station_m'], 2000.)


if __name__ == '__main__':
    unittest.main()
