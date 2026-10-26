"""Sensor false-stop regressions with retained collision and crossing checks."""
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'student_v3_claude'))
from lidar_corridor import LidarCorridor, fit_road_plane
from semantic_fusion import Tracker
from traffic_planner import GapPlanner


class PerceptionRegressionTests(unittest.TestCase):
    def road(self, slope):
        x, y = np.meshgrid(np.arange(2.1, 32., .22), np.arange(-3.8, 3.8, .18))
        return np.column_stack((x.ravel(), y.ravel(),
                                slope * x.ravel() + .015 * y.ravel() - .45))

    def corridor(self, body):
        corridor = LidarCorridor()
        corridor.ingest((body - corridor.mount).astype(np.float32))
        return corridor

    def test_uphill_and_downhill_road_do_not_block_driving_path(self):
        path = np.column_stack((np.arange(0., 30.5, .5), np.zeros(61)))
        for slope in (.04, -.04):
            with self.subTest(slope=slope):
                corridor = self.corridor(self.road(slope))
                self.assertAlmostEqual(corridor.ground_plane[0], slope, places=4)
                self.assertIsNone(corridor.blocked(path, 1.1))

    def test_low_box_and_person_on_sloped_road_still_block(self):
        path = np.column_stack((np.arange(0., 30.5, .5), np.zeros(61)))
        for height in (.4, 1.2):
            with self.subTest(height=height):
                road = self.road(.04)
                x, y = np.meshgrid(np.arange(8., 8.9, .12), np.arange(-.4, .5, .12))
                box = np.column_stack((x.ravel(), y.ravel(),
                                       .04*x.ravel() + .015*y.ravel() - .45 + height))
                corridor = self.corridor(np.vstack((road, box)))
                hit = corridor.blocked(path, 1.1)
                self.assertIsNotNone(hit)
                self.assertLess(hit[0], 9.)

    def test_roadside_object_remains_detected_without_blocking_clear_path(self):
        path = np.column_stack((np.arange(0., 30.5, .5), np.zeros(61)))
        x, y = np.meshgrid(np.arange(9., 10., .15), np.arange(2.8, 3.5, .15))
        obstacle = np.column_stack((x.ravel(), y.ravel(),
                                    .04*x.ravel() + .015*y.ravel() - .45 + .6))
        corridor = self.corridor(np.vstack((self.road(.04), obstacle)))
        self.assertGreater(len(corridor.snapshot()[0]), 0)
        self.assertIsNone(corridor.blocked(path, 1.1))

    def test_front_bumper_obstacle_survives_sensor_self_filter(self):
        road = self.road(0.)
        x, y = np.meshgrid(np.arange(2.40, 2.60, .05), np.arange(-.3, .31, .1))
        obstacle = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .5)))
        corridor = self.corridor(np.vstack((road, obstacle)))
        path = np.column_stack((np.arange(0., 30.5, .5), np.zeros(61)))
        hit = corridor.blocked(path, 1.1)
        self.assertIsNotNone(hit)
        self.assertLessEqual(hit[0], 2.5)

    def test_ego_housing_does_not_block_path(self):
        road = self.road(0.)
        housing = np.array([[2.1, 0., .7], [1.1, .5, 1.], [.7, -.4, 1.4]])
        corridor = self.corridor(np.vstack((road, housing)))
        self.assertEqual(len(corridor.snapshot()[0]), 0)

    def test_tracker_discards_future_tracks_after_simulator_time_reset(self):
        tracker = Tracker()
        first = tracker.update('npc', [self.detection(8., 3.5)], 100.)[0]
        second = tracker.update('npc', [self.detection(8., 3.5)], 10.)[0]
        self.assertNotEqual(first['uid'], second['uid'])
        self.assertNotIn(first['uid'], tracker.tracks)

    def test_sparse_ground_keeps_original_height_filter(self):
        points = np.array([[3., 0., .4], [4., 0., .5], [5., 0., .6]])
        np.testing.assert_array_equal(fit_road_plane(points, -.3), [0., 0., -.3])

    def detection(self, x, y):
        return dict(pos=np.array([x, y]), size=np.array([4.6, 1.9]), height=1.5, n=12)

    def test_parked_cluster_noise_and_one_face_jump_do_not_predict_crossing(self):
        tracker = Tracker()
        for i in range(18):
            noise = (0., .04, -.05, .06, -.04, .03)[i % 6]
            x = 8. + (1. if i == 7 else noise)
            track = tracker.update('npc', [self.detection(x, 3.5 + noise)], 100. + i*.1)[0]
            self.assertFalse(track['moving'])
            self.assertLess(np.linalg.norm(track['vel']), .1)

    def test_actual_moving_vehicle_is_confirmed_before_parked_warmup(self):
        tracker = Tracker()
        for i in range(5):
            track = tracker.update('npc', [self.detection(8. + i, 3.5)], 100. + i*.1)[0]
        self.assertTrue(track['moving'])
        np.testing.assert_allclose(track['vel'], [10., 0.], atol=1e-6)

    def test_padding_rotation_does_not_move_a_stationary_object(self):
        tracker = Tracker()
        for i in range(12):
            detection = self.detection(8. + i*.12, 3.5)
            detection['motion_bounds'] = np.array([[7., 3.], [9., 4.]])
            track = tracker.update('npc', [detection], 100. + i*.1)[0]
            self.assertFalse(track['moving'])
            np.testing.assert_array_equal(track['vel'], [0., 0.])

    def test_one_changing_visible_edge_is_not_crossing_motion(self):
        tracker = Tracker()
        for kind in ('npc', 'obstacle'):
            for i in range(9):
                bounds = np.array([[7., 3.], [9. + i*.15, 4.]])
                detection = self.detection(*bounds.mean(axis=0))
                detection['motion_bounds'] = bounds
                track = tracker.update(kind, [detection], 100. + i*.1)[0]
                self.assertFalse(track['moving'])
                np.testing.assert_array_equal(track['vel'], [0., 0.])

    def test_translating_observed_edges_confirm_actual_vehicle_motion(self):
        for speed in (1., 10.):
            tracker = Tracker()
            for i in range(5):
                detection = self.detection(8. + speed*i*.1, 3.5)
                detection['motion_bounds'] = np.array([[7., 3.], [9., 4.]]) + [speed*i*.1, 0.]
                track = tracker.update('npc', [detection], 100. + i*.1)[0]
            self.assertTrue(track['moving'])
            np.testing.assert_allclose(track['vel'], [speed, 0.], atol=1e-6)

    def test_distant_fast_crossing_approaches_with_braking_reserve(self):
        path = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        leader = (12., 0., 4.6, 'fusion_npc', 'npc', 3., 4.6, 1.9, 0., 3., 0., 1, 0.)
        crossing = (24., -20., 4.6, 'fusion_npc', 'npc', 10., 4.6, 1.9, 90., 0., 10., 2, 0.)
        with patch('traffic_planner.FAST_TRAFFIC_GUARD', True):
            result = GapPlanner().evaluate([leader, crossing], ego, path, 8., 100.)
        self.assertEqual(result['kind'], 'follow')
        self.assertEqual(result['reason'], 'approach fast traffic with braking reserve')
        self.assertGreater(result['v_mps'], 0.)
        self.assertLessEqual(result['v_mps'], 3.)

    def test_imminent_fast_crossing_still_stops(self):
        path = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        crossing = (9., -4., 4.6, 'fusion_npc', 'npc', 10., 4.6, 1.9, 90., 0., 10., 2, 0.)
        with patch('traffic_planner.FAST_TRAFFIC_GUARD', True):
            result = GapPlanner().evaluate([crossing], ego, path, 8., 100.)
        self.assertEqual(result['kind'], 'stop')

    def test_distant_fast_guard_does_not_skip_an_immediate_second_vehicle(self):
        path = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        far = (24., -20., 4.6, 'fusion_npc', 'npc', 10., 4.6, 1.9, 90., 0., 10., 1, 0.)
        near = (4., -3., 4.6, 'fusion_npc', 'npc', 2., 4.6, 1.9, 90., 0., 2., 2, 0.)
        with patch('traffic_planner.FAST_TRAFFIC_GUARD', True):
            result = GapPlanner().evaluate([far, near], ego, path, 8., 100.)
        self.assertEqual(result['kind'], 'stop')
        self.assertEqual(result['reason'], 'predicted footprint time overlap')

    def test_recorded_crossing_snapshots_retain_near_stops_and_allow_far_approach(self):
        # A snapshot uses the preceding published path, not a full perception replay.
        assets = HERE / 'student_v3_claude'
        data = json.loads((assets / 'test_data/run_20261002_traffic.json').read_text())
        links = json.loads((assets / 'link_set.json').read_text())
        for row in data['snapshots']:
            with self.subTest(time=row['time']), patch('traffic_planner.FAST_TRAFFIC_GUARD', True):
                result = GapPlanner(links).evaluate(row['objects'], row['ego'],
                    np.asarray(row['path']), 28.94 / 3.6, row['time'])
            self.assertEqual(result['kind'], row['expected_kind'])
            if row['expected_kind'] == 'follow':
                self.assertGreater(result['v_mps'], 0.)
                self.assertLessEqual(result['v_mps'], 28.94 / 3.6)

    def test_supported_ground_does_not_absorb_a_raised_platform(self):
        road = self.road(0.)
        raised = road[(road[:, 0] > 7.) & (road[:, 0] < 14.)].copy()
        road = road[(road[:, 0] <= 7.) | (road[:, 0] >= 14.)]
        raised[:, 2] += .5
        corridor = self.corridor(np.vstack((road, raised)))
        path = np.column_stack((np.arange(0., 30.5, .5), np.zeros(61)))
        self.assertIsNotNone(corridor.blocked(path, 1.1))


if __name__ == '__main__':
    unittest.main()
