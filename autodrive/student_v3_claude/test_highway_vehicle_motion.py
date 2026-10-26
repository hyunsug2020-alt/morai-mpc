"""Vehicle centroid jumps must not manufacture motion toward another lane."""
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar_vehicle_detection import VehicleTracks


def detection(x, y):
    return dict(position=[x, y], size=[4.9, 2.1, 1.5], heading_deg=0., score=.8)


class VehicleMotionTests(unittest.TestCase):
    def test_centroid_jump_preserves_measured_position_and_established_forward_speed(self):
        tracker = VehicleTracks()
        for i in range(5):
            car = tracker.update([detection(10.*i*.1, 0.)], 100.+i*.1)[0]
        self.assertTrue(car['velocity_valid'])
        shifted = detection(5., 1.3)
        car = tracker.update([shifted], 100.5)[0]
        self.assertEqual(car['position'], shifted['position'])
        self.assertTrue(car['velocity_valid'])
        self.assertAlmostEqual(car['velocity'][0], 10.)
        self.assertAlmostEqual(car['velocity'][1], 0.)
        for i in range(1, 5):
            car = tracker.update([detection(5.+i, 1.3)], 100.5+i*.1)[0]
        self.assertTrue(car['velocity_valid'])
        self.assertAlmostEqual(car['velocity'][1], 0.)

    def test_real_smooth_lane_change_keeps_lateral_motion(self):
        tracker = VehicleTracks()
        for i in range(12):
            car = tracker.update([detection(10.*i*.1, 2.*i*.1)], 100.+i*.1)[0]
        self.assertTrue(car['velocity_valid'])
        self.assertAlmostEqual(car['velocity'][1], 2.)

    def test_jump_during_initial_acquisition_does_not_create_a_merging_velocity(self):
        tracker = VehicleTracks()
        tracker.update([detection(0., 0.)], 100.)
        for i in range(1, 6):
            car = tracker.update([detection(float(i), 1.3)], 100.+i*.1)[0]
        self.assertTrue(car['velocity_valid'])
        self.assertAlmostEqual(car['velocity'][1], 0.)

    def test_recorded_parallel_vehicle_does_not_get_false_motion_toward_ego(self):
        samples = json.loads((Path(__file__).parent / 'test_data' /
                              'highway_vehicle_centroid_jump_20261007.json').read_text())
        tracker = VehicleTracks()
        for sample in samples:
            car = tracker.update([sample], sample['stamp'])[0]
            self.assertEqual(car['position'], sample['position'])
            if car['velocity_valid']:
                # Ego is east of this vehicle; positive world-x velocity was
                # the false merge trigger. Motion away retains its measurement.
                self.assertLess(car['velocity'][0], 1.)

    def test_recorded_parallel_box_jitter_does_not_create_a_cut_in(self):
        samples = json.loads((Path(__file__).parent / 'test_data' /
                              'highway_parallel_centroid_jitter_20261009.json').read_text())
        tracker = VehicleTracks()
        identifiers = set()
        checked = 0
        for sample in samples:
            car = tracker.update([sample], sample['stamp'])[0]
            identifiers.add(car['uid'])
            self.assertEqual(car['position'], sample['position'])
            if 1791549382.4 <= sample['stamp'] <= 1791549383.7:
                self.assertTrue(car['velocity_valid'])
                # Ground truth moves east by only 0.04 m/s. The old fit
                # invented 1.52 m/s east and triggered a highway emergency.
                self.assertLess(car['velocity'][0], .25)
                self.assertLess(car['velocity'][1], -5.)
                checked += 1
        self.assertEqual(len(identifiers), 1)
        self.assertGreaterEqual(checked, 6)

    def test_accelerating_real_turn_is_not_reanchored_as_box_noise(self):
        for interval in (.1, .2, .4):
            for acceleration in (2., 4., 6.):
                with self.subTest(interval=interval, acceleration=acceleration):
                    tracker = VehicleTracks()
                    for i in range(round(4./interval)+1):
                        t = i*interval
                        turning = max(0., t-2.)
                        lateral_speed = acceleration*turning
                        sample = detection(10.*t, .5*acceleration*turning**2)
                        sample['heading_deg'] = math.degrees(math.atan2(lateral_speed, 10.))
                        car = tracker.update([sample], 100.+t)[0]
                        self.assertEqual(car['position'], sample['position'])
                    self.assertTrue(car['velocity_valid'])
                    # The existing 1.2-second fit has finite lag, but a real
                    # smooth turn must retain its established lateral motion.
                    self.assertGreater(car['velocity'][1], acceleration*(turning-.8))

    def test_real_inward_motion_still_triggers_close_collision_control(self):
        import numpy as np
        from highway_planner import HighwayPlanner
        tracker = VehicleTracks()
        for i in range(15):
            t = i*.1
            sample = detection(5.+10.*t, 4.4-1.2*t)
            sample['heading_deg'] = math.degrees(math.atan2(-1.2, 10.))
            car = tracker.update([sample], 100.+t)[0]
        self.assertTrue(car['velocity_valid'])
        self.assertAlmostEqual(car['velocity'][1], -1.2)
        vx, vy = car['velocity']
        obj = (*car['position'], 4.9, 'lidar_car', 'npc', math.hypot(vx, vy),
               4.9, 2.1, car['heading_deg'], vx, vy, car['uid'], 0., None, True)
        route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        planner = HighwayPlanner(dict(from_idx=0, to_idx=1800, kmh=90.,
            lane_change_policy='free_three_lane'), route)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=101.4)
        planner.longitudinal.begin(ego)
        checked = planner.plan_speed(route, ego, [obj], 25.)
        self.assertTrue(checked['emergency'])
        self.assertEqual(checked['reference_mps'], 0.)

    def test_recorded_false_cut_in_no_longer_commands_an_emergency(self):
        import numpy as np
        from gui import HdMapLanes
        from highway_planner import HighwayPlanner, local_points
        from highway_three_lane import HighwayLaneMap, ThreeLanePolicy
        here = Path(__file__).parent
        samples = json.loads((here/'test_data/highway_parallel_centroid_jitter_20261009.json').read_text())
        cases = json.loads((here/'test_data/highway_false_cut_in_20261009.json').read_text())
        route = np.array([[p['x'], p['y']] for p in
                          json.loads((here/'global_path.json').read_text())['waypoints']])
        config = json.loads((here/'highway_mission.json').read_text())
        lanes = HdMapLanes(here/'link_set.json')
        lanes.set_route(route)
        native = HighwayLaneMap(lanes, config)
        for case in cases:
            with self.subTest(stamp=case['original']['stamp']):
                original = case['original']
                tracker = VehicleTracks()
                for sample in samples:
                    if sample['stamp'] > original['stamp']+1e-6:
                        break
                    corrected = tracker.update([sample], sample['stamp'])[0]
                self.assertEqual(corrected['position'], original['position'])
                corrected['uid'] = original['uid']
                ego = case['ego']
                path = local_points(np.asarray(case['world_path']), ego)
                for label, car in [('original', original), ('corrected', corrected)]:
                    vx, vy = car['velocity']
                    length, width = car['size'][:2]
                    obj = (*car['position'], max(length, width), 'lidar_car', 'npc',
                           math.hypot(vx, vy), length, width, car['heading_deg'],
                           vx, vy, car['uid'], 0., None, car['velocity_valid'])
                    planner = HighwayPlanner(config, route)
                    planner.longitudinal.begin(ego)
                    motion = native.motion_candidates(ego, [obj])
                    ThreeLanePolicy(config).update_motion_paths(planner, motion, ego, [obj])
                    result = planner.plan_speed(path, ego, [obj], 25., ceiling_mps=25.)
                    if label == 'original':
                        self.assertTrue(result['emergency'])
                    else:
                        self.assertFalse(result['emergency'])
                        self.assertGreater(result['reference_mps']*3.6, 70.)


if __name__ == '__main__':
    unittest.main()
