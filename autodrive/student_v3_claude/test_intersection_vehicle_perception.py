"""Shared intersection activation and independent vehicle shape/motion checks."""
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

from stop_line import entry_is_clear
from intersection_perception import (active_intersection, build_intersection_regions,
    load_intersection_regions, write_intersection_regions)
from lidar_vehicle_detection import VehicleReceiver, VehicleTracks, detector_mission_active
from traffic_planner import GapPlanner, MapVehiclePredictor, overlaps, predict_vehicle, vehicle_motion_heading


def car(shape=180., velocity=(10., 0.), reliable=True):
    return (5., 0., 4.9, "lidar_car", "npc", math.hypot(*velocity),
            4.9, 2.1, shape, *velocity, 500001, 0., None, reliable)


class IntersectionVehiclePerceptionTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 200.5, .5), np.zeros(401)))
        self.station = self.route[:, 0]
        self.junction = np.zeros(401, dtype=bool)
        self.junction[200:260] = True
        self.lines = [dict(id="709:199", node_id="A1256W000709", route_idx=199,
                           station_m=99.7, entry_link_id="surveyed_entry")]
        self.regions = build_intersection_regions(self.lines, self.junction, self.station)

    def test_exact_surveyed_stop_and_real_exit_define_activation(self):
        region = self.regions[0]
        self.assertEqual(region['stop_station_m'], 99.7)
        self.assertAlmostEqual(region['start_station_m'], 19.7)
        self.assertEqual(region['exit_station_m'], 130.)
        self.assertEqual(region['end_station_m'], 155.)
        self.assertIsNone(active_intersection(19.69, self.regions))
        self.assertIsNotNone(active_intersection(19.7, self.regions))
        self.assertIsNotNone(active_intersection(155., self.regions))
        self.assertIsNone(active_intersection(155.01, self.regions))
        self.assertIsNone(active_intersection(float('nan'), self.regions))

    def test_detector_handoff_padding_retains_ready_after_gui_exit(self):
        mission = dict(from_idx=1000, to_idx=1200)
        for station in (10., 19.7, 150., 164.5):
            index = int(round(station/.5))
            self.assertTrue(detector_mission_active(index, mission, self.regions, self.station))
        self.assertFalse(detector_mission_active(19, mission, self.regions, self.station))
        self.assertFalse(detector_mission_active(331, mission, self.regions, self.station))
        receiver = VehicleReceiver.__new__(VehicleReceiver)
        receiver.lock, receiver.received = threading.Lock(), 100.
        receiver.record = dict(active=True, ready=True, vehicles=[])
        with patch('lidar_vehicle_detection.time.monotonic', return_value=100.4):
            self.assertFalse(receiver.snapshot()[1])

    def test_only_requested_node_and_complete_junction_are_allowed(self):
        self.assertRaises(ValueError, build_intersection_regions, [], self.junction, self.station)
        bad = [dict(self.lines[0], route_idx=30)]
        self.assertRaises(ValueError, build_intersection_regions, bad, self.junction, self.station)
        mask = self.junction.copy(); mask[200:] = True
        self.assertRaises(ValueError, build_intersection_regions, self.lines, mask, self.station)
        other = dict(self.lines[0], node_id='A1256W000073')
        self.assertEqual(len(build_intersection_regions(self.lines+[other], self.junction, self.station)), 1)

    def test_launch_json_roundtrip_rejects_wrong_route_nodes_or_bounds(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'regions.json'
            payload = write_intersection_regions(path, self.route, self.regions)
            self.assertEqual(load_intersection_regions(path, self.route), self.regions)
            changed = self.route.copy(); changed[0, 0] += .01
            self.assertRaises(ValueError, load_intersection_regions, path, changed)
            self.assertRaises(ValueError, load_intersection_regions, path, self.route, {'other_node'})
            payload['regions'][0]['end_station_m'] = float('nan')
            path.write_text(json.dumps(payload))
            self.assertRaises(ValueError, load_intersection_regions, path, self.route)

    def test_real_map_matches_existing_stop_line_geometry(self):
        from gui import HdMapLanes
        assets = Path(__file__).parent
        route = np.asarray([[p['x'], p['y']] for p in json.loads((assets/'global_path.json').read_text())['waypoints']])
        lanes = HdMapLanes(assets/'link_set.json'); lanes.set_route(route)
        regions = build_intersection_regions(lanes.stop_lines, lanes.route_junction, lanes.route_station)
        self.assertEqual([r['node_id'] for r in regions], ['A1256W000709'])
        region = regions[0]
        line = next(line for line in lanes.stop_lines if line['node_id']=='A1256W000709')
        self.assertEqual(region['stop_station_m'], line['station_m'])
        self.assertEqual((region['entry_route_idx'], region['exit_route_idx']), (1782, 1841))
        self.assertAlmostEqual(region['start_station_m'], 805.1006875597407)
        self.assertAlmostEqual(region['end_station_m'], 939.8373138299062)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'regions.json'
            write_intersection_regions(path, route, regions)
            self.assertEqual(load_intersection_regions(path, route), regions)

    def test_launch_export_and_detector_argument_share_same_config(self):
        source = (Path(__file__).parents[1]/'student_v3_follow_claude.sh').read_text()
        self.assertIn('export STUDENT_INTERSECTION_PERCEPTION="$RUN_DIR/intersection_perception.json"', source)
        self.assertIn('--output "$STUDENT_INTERSECTION_PERCEPTION"', source)
        self.assertIn('--intersection-regions "$STUDENT_INTERSECTION_PERCEPTION"', source)
        self.assertLess(source.index('--output "$STUDENT_INTERSECTION_PERCEPTION"'),
                        source.index('"$VEHICLE_PYTHON" "$ROOT/autodrive/lidar_vehicle_detection.py"'))

    def test_tracker_preserves_box_heading_despite_different_velocity_heading(self):
        tracks = VehicleTracks()
        for i in range(9):
            result = tracks.update([dict(position=[5.+10.*i*.1, 0.],
                size=[4.9, 2.1, 1.5], heading_deg=90., score=.9)], 10.+i*.1)
        self.assertTrue(result[0]['velocity_valid'])
        self.assertEqual(result[0]['heading_deg'], 90.)
        self.assertAlmostEqual(result[0]['motion_heading_deg'], 0.)
        self.assertAlmostEqual(result[0]['velocity'][0], 10.)

    def test_straight_prediction_uses_world_velocity_and_box_yaw_independently(self):
        times = np.arange(0., 2.01, .2)
        for heading in (90., 180.):
            with self.subTest(heading=heading):
                obj = car(heading)
                xy, yaw = predict_vehicle(obj, times)
                np.testing.assert_allclose(xy[:, 0], 5.+10.*times)
                np.testing.assert_allclose(xy[:, 1], 0.)
                np.testing.assert_allclose(yaw, math.radians(heading))
                self.assertAlmostEqual(vehicle_motion_heading(obj), 0.)
        invalid = car(90., reliable=False)
        xy, yaw = predict_vehicle(invalid, times)
        np.testing.assert_allclose(xy, np.tile(invalid[:2], (len(times), 1)))
        self.assertIsNone(vehicle_motion_heading(invalid))

    def test_directed_map_match_uses_motion_preserving_90_and_180_box_axes(self):
        times = np.arange(0., 2.01, .2)
        points = np.column_stack((np.arange(0., 80.5, .5), np.zeros(161)))
        links = [dict(points=points.tolist(), from_node_idx='a', to_node_idx='b'),
                 dict(points=points[::-1].tolist(), from_node_idx='b', to_node_idx='a')]
        predictor = MapVehiclePredictor(links)
        for shape in (90., 180.):
            with self.subTest(shape=shape):
                predictions = predictor.predict(car(shape), times)
                self.assertIsNotNone(predictions)
                xy, yaw = predictions[0]
                self.assertGreater(xy[-1, 0], xy[0, 0])
                np.testing.assert_allclose(yaw, math.radians(shape))
        self.assertIsNone(predictor.predict(car(reliable=False), times))

    def test_curved_map_rotates_box_by_turn_without_replacing_initial_shape(self):
        arc = np.arange(0., 60.01, .5)
        points = np.column_stack((30.*np.sin(arc/30.), 30.*(1.-np.cos(arc/30.))))
        predictor = MapVehiclePredictor([dict(points=points.tolist(), from_node_idx='a', to_node_idx='b')])
        obj = list(car(90.)); obj[:2] = points[0]
        times = np.arange(0., 2.01, .2)
        predictions = predictor.predict(tuple(obj), times)
        self.assertIsNotNone(predictions)
        xy, yaw = predictions[0]
        self.assertAlmostEqual(yaw[0], math.pi/2)
        self.assertGreater(yaw[-1], yaw[0]+.3)
        self.assertGreater(xy[-1, 0], 10.)

    def test_180_box_axis_flip_does_not_invent_crossing_or_change_follow_decision(self):
        route = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=14.4)
        answers = []
        for shape in (0., 180.):
            obj = list(car(shape, (5., 0.))); obj[0] = 10.
            planner = GapPlanner()
            answers.append(planner.evaluate([tuple(obj)], ego, route, 8., 100.))
        self.assertEqual(answers[0], answers[1])
        self.assertIsNotNone(answers[0])
        self.assertNotEqual(answers[0]['kind'], 'stop')

    def capture_ego_rollouts(self, planner):
        route = np.column_stack((np.arange(0., 80.5, .5), np.zeros(161)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        obj = list(car(0., (1., 0.))); obj[:2] = (0., 20.)
        captured = []
        def observe(a, ayaw, b, byaw, length, width, *args, **kwargs):
            if len(a) == 41:
                captured.append(a.copy())
            return overlaps(a, ayaw, b, byaw, length, width, *args, **kwargs)
        with patch('traffic_planner.overlaps', side_effect=observe):
            answer = planner.evaluate([tuple(obj)], ego, route, 8., 100.)
        self.assertTrue(answer is None or answer['kind'] != 'stop')
        self.assertGreaterEqual(len(captured), 3)
        return captured[1][:, 0]

    def test_intersection_rollout_matches_acceleration_and_entry_cap_then_releases(self):
        planner = GapPlanner(); planner.ACCEL_MPS2 = 1.2
        planner.entry_cap_mps, planner.entry_cap_distance_m = 12./3.6, 12.
        distance = self.capture_ego_rollouts(planner)
        self.assertAlmostEqual(distance[10], .5*1.2*2.**2, places=5)
        speeds = np.diff(distance)/.2
        inside = distance[:-1] <= 12.
        self.assertLessEqual(float(speeds[inside].max()), 12./3.6+1e-6)
        self.assertGreater(float(speeds[~inside].max()), 12./3.6+1.)
        # No entry profile on normal roads retains the existing rollout.
        normal = self.capture_ego_rollouts(GapPlanner())
        self.assertGreater(normal[10], distance[10]+2.)

    def test_zero_remaining_entry_distance_does_not_add_a_capped_step(self):
        normal = self.capture_ego_rollouts(GapPlanner())
        planner = GapPlanner(); planner.entry_cap_mps, planner.entry_cap_distance_m = 12./3.6, 0.
        np.testing.assert_allclose(self.capture_ego_rollouts(planner), normal)

    def test_slower_real_intersection_travel_rechecks_crossing_time(self):
        route = np.column_stack((np.arange(0., 80.5, .5), np.zeros(161)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        obj = list(car(90., (0., 5.))); obj[:2] = (12., -26.)
        normal = GapPlanner().evaluate([tuple(obj)], ego, route, 8., 100.)
        planner = GapPlanner(); planner.ACCEL_MPS2 = 1.2
        planner.entry_cap_mps, planner.entry_cap_distance_m = 12./3.6, 12.
        actual = planner.evaluate([tuple(obj)], ego, route, 8., 100.)
        # The default profile may clear the crossing outright; the slower
        # junction profile must still recheck it and lower the speed.
        self.assertIsNotNone(actual)
        if normal is not None:
            self.assertLess(actual['v_mps'], normal['v_mps'])
        self.assertFalse(entry_is_clear(actual, False, True))

    def test_intersection_waits_when_actual_profile_cannot_clear_in_eight_seconds(self):
        route = np.column_stack((np.arange(0., 80.5, .5), np.zeros(161)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        # A car has just crossed 25 m ahead. Its remaining footprint defines
        # a 31.5 m gap-exit window; the old acceleration cleared it too soon.
        obj = list(car(90., (0., 15.))); obj[:2] = (25., 0.)
        normal = GapPlanner(); normal.state = 'WAIT'
        accepted = normal.evaluate([tuple(obj)], ego, route, 8., 100.)
        self.assertEqual(accepted['kind'], 'enter')
        planner = GapPlanner(); planner.state = 'WAIT'; planner.ACCEL_MPS2 = 1.2
        planner.entry_cap_mps, planner.entry_cap_distance_m = 12./3.6, 12.
        actual = planner.evaluate([tuple(obj)], ego, route, 8., 100.)
        self.assertEqual(actual['kind'], 'stop')
        self.assertEqual(actual['reason'], 'cannot clear merge within prediction horizon')


if __name__ == '__main__':
    unittest.main()
