"""Measured regressions for missed cars and the October 7 tunnel standstill."""
import unittest
import threading
import time
from unittest.mock import Mock, patch

import numpy as np

from gui import Runtime, deduplicate_obstacles, highway_objects, highway_static_objects, highway_measured_objects, static_clear_distance
from highway_planner import HighwayPlanner, NearFieldMemory, timed_gap
from lidar_vehicle_detection import VehicleTracks, VehicleReceiver
from lattice_planner import plan
from lane_corridor import LaneCorridor
import test_tunnel_integration as tunnel_fixture
import test_intersection_runtime as intersection_fixture
from test_tunnel_visibility import returns, straight
from tunnel_visibility import ObservedLidarCoverage
from traffic_planner import JunctionTrackMemory


class DriveFixTests(unittest.TestCase):
    def test_missed_scan_retains_confirmed_motion_without_refreshing_its_stamp(self):
        tracks = VehicleTracks()
        for t in (0., .25, .5):
            output = tracks.update([dict(position=[50.+8.*t, 0.],
                size=[4.9, 2.1, 1.5], heading_deg=0.)], t)
        uid = output[0]['uid']
        for t in (.75, 1.25):
            output = tracks.update([], t)
            self.assertEqual(output[0]['uid'], uid)
            self.assertTrue(output[0]['velocity_valid'])
            self.assertEqual(output[0]['stamp'], .5)
            self.assertAlmostEqual(output[0]['position'][0], 50.+8.*t)
        self.assertEqual(tracks.update([], 1.51), [])

    def test_missed_unknown_vehicle_does_not_acquire_invented_motion(self):
        tracks = VehicleTracks()
        tracks.update([dict(position=[10., 0.], size=[4.9, 2.1, 1.5], heading_deg=0.)], 0.)
        car = tracks.update([], .5)[0]
        self.assertFalse(car['velocity_valid'])
        self.assertEqual(car['velocity'], [0., 0.])
        self.assertEqual(car['position'], [10., 0.])

    def test_weak_consistent_measurements_continue_only_confirmed_tracks(self):
        tracks = VehicleTracks()
        def car(t, score, y=0.):
            return dict(position=[40.+8.*t, y], size=[4.9, 2.1, 1.5], heading_deg=0., score=score)
        self.assertEqual(tracks.update([car(0., .3)], 0., min_score=.445), [])
        for t in (.1, .3, .5):
            strong = tracks.update([car(t, .7)], t, min_score=.445)
        uid = strong[0]['uid']
        for t in (.7, .9, 1.1, 1.3):
            output = tracks.update([car(t, .3), car(t, .3, 3.5)], t, min_score=.445)
            self.assertEqual(len(output), 1)
            self.assertEqual(output[0]['uid'], uid)
            self.assertEqual(output[0]['stamp'], t)
            self.assertTrue(output[0]['velocity_valid'])
        self.assertEqual(len(tracks.tracks), 1)

    def test_high_confidence_seed_can_measure_motion_from_later_weak_observations(self):
        tracks = VehicleTracks()
        for t, score in [(0., .7), (.2, .3), (.4, .3)]:
            output = tracks.update([dict(position=[40.+8.*t, 0.],
                size=[4.9, 2.1, 1.5], heading_deg=0., score=score)], t, min_score=.445)
            self.assertEqual(output[0]['uid'], 500000)
            self.assertEqual(output[0]['velocity_valid'], t >= .4)
        self.assertAlmostEqual(output[0]['velocity'][0], 8.)

    def test_matched_parked_semantic_box_does_not_duplicate_moving_car(self):
        car = (40., 0., 4.9, 'lidar_car', 'npc', 8., 4.9, 2.1, 0., 8., 0., 500000, 0., None, True)
        parked = (40.5, .5, 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 0.)
        box = (40.5, .5, 1., 'fusion_obstacle', 'obstacle', 0., 1., 1., 0.)
        self.assertEqual(highway_objects([parked, box], [car]), [box, car])
        self.assertEqual(highway_objects([parked], []), [parked])
        adjacent = tuple([40., 3.5]+list(parked[2:]))
        following = tuple([45., 0.]+list(parked[2:]))
        self.assertEqual(highway_objects([adjacent, following], [car]), [adjacent, following, car])
        another = tuple([40.8, .5]+list(parked[2:]))
        self.assertEqual(highway_objects([parked, another], [car]), [another, car])

    def test_unknown_or_stopped_lidar_car_still_brakes_from_40m(self):
        route = straight(300.)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90.)
        for valid in (False, True):
            with self.subTest(valid=valid):
                car = (40., 0., 4.9, 'lidar_car', 'npc', 0., 4.9, 2.1, 180., 0., 0., 500000, 0., None, valid)
                objects = highway_objects([], [car])
                static = highway_static_objects(objects)
                planner = HighwayPlanner(dict(from_idx=0, to_idx=500, kmh=90.), route)
                result = planner.decide(0, route, ego, objects, {}, lambda p,h:h,
                    lambda p,h:static_clear_distance(p, ego, static, h))
                self.assertLess(result['status']['clearance_components_m']['static'], 36.)
                self.assertLess(result['target_mps'], 15.)

    def test_predicted_track_cannot_remove_a_measured_parked_vehicle(self):
        predicted = (50., 0., 4.9, 'lidar_car', 'npc', 8., 4.9, 2.1, 0., 8., 0., 500000, 0., None, True, .75)
        parked = (50., 0., 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 0.)
        objects = highway_objects([parked], [predicted])
        self.assertIn(parked, objects)
        self.assertEqual(highway_static_objects(objects), [parked])
        self.assertEqual(highway_measured_objects(objects), [parked])

    def test_receiver_preserves_prediction_age_for_collision_checks(self):
        receiver = VehicleReceiver.__new__(VehicleReceiver)
        receiver.lock = threading.Lock()
        receiver.received = time.monotonic()
        receiver.record = dict(ready=True, active=True, vehicles=[dict(position=[50., 0.],
            velocity=[8., 0.], size=[4.9, 2.1, 1.5], heading_deg=0., uid=500000,
            velocity_valid=True, prediction_age_s=.75)])
        objects, ready, _ = receiver.snapshot()
        self.assertTrue(ready)
        self.assertEqual(objects[0][15], .75)
        self.assertEqual(highway_static_objects(objects), [])
        self.assertEqual(highway_measured_objects(objects), [])
        # A retained mover still checks the closing gap in the moving forecast;
        # it cannot become a frozen obstacle or own fresh raw measurements.
        risk = timed_gap(straight(170.), dict(x=0., y=0., heading_deg=0., speed_kmh=90.),
                         highway_objects([], objects), 25., horizon_s=3., lane_change=False)
        self.assertIsNotNone(risk)
        self.assertEqual(risk['object_id'], 500000)

    def test_predicted_car_does_not_erase_measured_blind_zone_obstacle(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        points = np.array([[5., -.2], [5., 0.], [5., .2], [5.4, 0.]])
        memory = NearFieldMemory(retain_measured_points=True)
        for t in (.3, .4):
            memory.update(points, ego, t)
        predicted = (6., 0., 4.9, 'lidar_car', 'npc', 8., 4.9, 2.1, 0., 8., 0., 500000, 0., None, True, .75)
        retained = memory.update(np.empty((0, 2)), ego, 1.15, highway_measured_objects([predicted]))
        self.assertGreater(len(retained), 0)
        self.assertGreater(len(memory.update(np.empty((0, 2)), ego, 1.41, [])), 0)

    def test_initial_footprint_uses_actual_heading_not_planned_tangent(self):
        # The pose is the rear axle (ego_body): a first path tangent turned 20 deg left
        # swings the real front-left corner (3.845 m ahead) out of the current body,
        # into the 40-50 deg sector this scan lacks. The straight path never leaves
        # the current body there.
        xyz = returns()
        angle = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]-1.05))
        xyz = xyz[(angle < 40.) | (angle > 50.)]
        path = straight(20.)
        yaw = np.radians(20.)
        turned = path @ np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
        coverage = ObservedLidarCoverage(xyz, [0., 0., 0.])
        self.assertLessEqual(coverage.clear_distance(turned, 20., exclude_initial_footprint=True), .25)
        self.assertEqual(coverage.clear_distance(path, 20., exclude_initial_footprint=True), 20.)

    def test_intersection_prediction_preserves_blind_zone_static_stop(self):
        fixture = intersection_fixture.IntersectionRuntimeTests()
        runtime = fixture.runtime()
        for step in range(16):
            fixture.step(runtime, step)
        ego = runtime.test_status['ego']
        points = np.array([[3., -.2], [3., 0.], [3., .2], [3.4, 0.]])
        for stamp in (100.6, 100.7):
            runtime.near_field_memory.update(points, ego, stamp)
        runtime.lidar.pts = np.empty((0, 2), dtype=np.float32)
        for step in range(16, 23):
            runtime.test_cars = [fixture.car(ego['x']+6.+8.*(step-16)/15., 0., vx=8.) +
                                 (.25+(step-16)/15.,)]
            row = fixture.step(runtime, step)
            self.assertGreater(row['lidar']['retained_near_field_n'], 0)
            # Committed entry: braked to a standstill behind it (no stop rule needed).
            self.assertEqual(row['published_cap_kmh'], 0.)

    def test_intersection_prediction_cannot_erase_parked_camera_vehicle(self):
        fixture = intersection_fixture.IntersectionRuntimeTests()
        runtime = fixture.runtime()
        for step in range(16):
            fixture.step(runtime, step)
        x = runtime.test_status['ego']['x']
        car = list(fixture.car(x+6., 0., vx=8.))
        car[2] = car[6] = 4.9
        car[7] = 2.1
        runtime.test_cars = [tuple(car)+(.5,)]
        runtime.object_receiver.objects = [(x+6., 0., 4.6, 'fusion_parked_vehicle',
                                            'obstacle', 0., 4.6, 1.9, 0.)]
        for step in range(16, 23):
            row = fixture.step(runtime, step)
            self.assertTrue(row['emergency_stop'])
            self.assertEqual(row['published_cap_kmh'], 0.)

    def test_junction_generated_ghost_cannot_delete_current_or_remembered_box(self):
        memory = JunctionTrackMemory()
        car = (2., 0., 4.9, 'lidar_car', 'npc', 8., 4.9, 2.1, 0., 8., 0., 500000, 0., None, True, 0.)
        memory.update([car], 100.)
        ghosts = memory.update([], 100.5)
        self.assertAlmostEqual(ghosts[0][0], 6.)
        self.assertAlmostEqual(ghosts[0][15], .5)
        box = (6., 0., 1., 'fusion_obstacle', 'obstacle', 0., 1., 1., 0.)
        self.assertIn(box, deduplicate_obstacles([box]+ghosts))
        runtime = Runtime.__new__(Runtime)
        runtime._static_object_memory = [(100., box)]
        self.assertEqual(runtime.remember_static_objects([], 100.5, True, ghosts), [box])

    def test_detector_prediction_does_not_refresh_junction_measured_timestamp(self):
        fixture = intersection_fixture.IntersectionRuntimeTests()
        runtime = fixture.runtime()
        ego = runtime.test_status['ego']
        runtime.test_cars = [fixture.car(ego['x']+10., 0., vx=8.)+(0.,)]
        with patch('gui.time.monotonic', return_value=100.):
            runtime.intersection_objects([], ego)
        runtime.test_cars = [fixture.car(ego['x']+14., 0., vx=8.)+(.5,)]
        with patch('gui.time.monotonic', return_value=100.5):
            cars = runtime.intersection_objects([], ego)
        self.assertEqual(runtime.junction_tracks.tracks[500001]['seen'], 100.)
        self.assertEqual(len(cars), 1)
        self.assertAlmostEqual(cars[0][15], .5)

    def test_transformed_path_is_checked_before_a_candidate_is_accepted(self):
        route = straight(60.)
        corridor = LaneCorridor(route, np.full(len(route), 5.), np.full(len(route), -5.))
        def corrupt_path(candidate):
            changed = np.asarray(candidate).copy()
            changed[:, 1] += 20. * (1.-np.exp(-np.maximum(changed[:, 0], 0.)/4.))
            return changed
        result = plan(route, dict(x=0., y=0., heading_deg=0., speed_kmh=0.),
            np.empty((0, 2)), [], corridor, outside_max_m=0., path_transform=corrupt_path)
        self.assertTrue(result['blocked'])

    def test_unknown_forward_sector_still_blocks_newly_swept_space(self):
        coverage = ObservedLidarCoverage(returns(gap_degrees=6.), [0., 0., 0.])
        self.assertEqual(coverage.clear_distance(straight(), 30., exclude_initial_footprint=True), 0.)
        missing = ObservedLidarCoverage(np.empty((0, 3)), [0., 0., 0.])
        self.assertEqual(missing.clear_distance(straight(), 30., exclude_initial_footprint=True), 0.)

    def test_missing_rear_support_does_not_block_forward_new_space(self):
        xyz = returns()
        xyz = xyz[xyz[:, 0] > 1.05]
        coverage = ObservedLidarCoverage(xyz, [0., 0., 0.])
        self.assertEqual(coverage.clear_distance(straight(), 20.), 0.)
        self.assertGreater(coverage.clear_distance(straight(), 20., exclude_initial_footprint=True), 1.)

    def test_wall_stop_is_not_hidden_by_short_observed_coverage(self):
        runtime = tunnel_fixture.TunnelIntegrationTests().runtime()
        runtime.tunnel.path_clearance = lambda *args, **kwargs: 1.
        runtime.reverse_escape_check = Mock(return_value=False)
        with patch('tunnel_visibility.ObservedLidarCoverage.clear_distance', return_value=0.):
            self.assertTrue(runtime.infer())
        row = runtime._web_record
        self.assertTrue(row['emergency_stop'])
        self.assertNotEqual(row['tunnel']['limiting_source'], 'observed')
        self.assertTrue(runtime.reverse_escape_check.call_args.args[1])


if __name__ == '__main__':
    unittest.main()
