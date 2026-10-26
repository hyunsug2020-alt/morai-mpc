"""Roundabout entry before the highway: follow the passed ring car in, never park in the ring.

User rules 2026-10-10 (ring_entry): wait at the line until the passing ring car's rear will
be just ahead of our nose and the next one is far enough; once committed only decelerate.

follow-20261010T085515Z (logged numbers below, ego at about (-98, 321)): the entry was accepted
behind ring car 500004 at the line; at 17:59:04.37 (bumper 0.7 m before the line, 3.5 km/h)
ring car 500005 was predicted to reach our side 0.2-0.8 s after us and the committed planner
said "clear it"; the live check stopped, we crept in and stopped at 17:59:08 with the nose at
the ring lane edge. Stopped, we fell back to the at-the-line rules and a ring NPC that stopped
beside our left front (box heading 188 deg, raw LiDAR sweep clear 58.9 m) held us 208 s.
"""
import json
import math
from pathlib import Path
import unittest

import numpy as np

import ring_entry
from traffic_planner import GapPlanner, MapVehiclePredictor
from stop_line import INTERSECTION_ACCEL_MPS2, INTERSECTION_ENTRY_MPS
import test_junction_entry as junction_fixture

ROOT = Path(__file__).parent
STOP_STATION = 885.101          # surveyed stop line A1256W000709:1781
FRONT = 2.3175
_MAP = {}


def roundabout_map():
    if not _MAP:
        from gui import HdMapLanes
        _MAP['links'] = HdMapLanes(ROOT / 'link_set.json').links
        _MAP['route'] = np.asarray([[p['x'], p['y']] for p in
                                    json.loads((ROOT / 'global_path.json').read_text())['waypoints']])
    return _MAP


def car(x, y, vx, vy, uid, length=4.9, width=2.09, heading=None, valid=True):
    """VehicleReceiver row of a PointPillars car (lidar_vehicle_detection.VehicleReceiver)."""
    heading = math.degrees(math.atan2(vy, vx)) if heading is None else heading
    return (x, y, max(length, width), 'lidar_car', 'npc', math.hypot(vx, vy) if valid else 0.,
            length, width, heading, vx if valid else 0., vy if valid else 0., uid, 0., None, valid, 0.)


def scene(ego_xy, heading, kmh, station, committed=False, **junction):
    """Planner, ego and route-following local path as the runtime builds them in the junction."""
    data = roundabout_map()
    planner = GapPlanner(data['links'])
    planner.ACCEL_MPS2 = INTERSECTION_ACCEL_MPS2
    planner.entry_cap_mps = (None if station >= STOP_STATION - FRONT + 3. else INTERSECTION_ENTRY_MPS)
    planner.entry_cap_distance_m = max(0., STOP_STATION + 12. - station)
    planner.junction = dict(dict(held=False, ready=True, unknown_ids=set(), line_gap_m=None, line_id='entry',
                                 stop_station_m=STOP_STATION, entry_m=station + FRONT - STOP_STATION), **junction)
    if committed:
        planner.junction_committed, planner.state = True, 'ENTER'
    ego = dict(x=ego_xy[0], y=ego_xy[1], heading_deg=heading, speed_kmh=kmh)
    route = data['route']
    nearest = int(np.argmin(np.linalg.norm(route - ego_xy, axis=1)))
    yaw = math.radians(heading)
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    path = (route[max(0, nearest - 2):nearest + 120] - ego_xy) @ rotation
    return planner, ego, path


class RingCutInTests(unittest.TestCase):
    def test_ring_car_reaching_our_side_keeps_us_at_the_line(self):
        # 17:59:04.37 traffic with us standing at the line: 500005 at 6 m/s ~17 m up the
        # ring reaches our side before our rear could clear it - no entry decision.
        planner, ego, path = scene((-99.33, 314.47), 87.81, 0., 881.76, held=True, line_gap_m=1.)
        cars = [car(-91.47, 328.89, 4.38, 1.55, 500004), car(-115.28, 332.55, 2.31, -5.65, 500005)]
        result = planner.evaluate(cars, ego, path, 55. / 3.6, 100.)
        self.assertEqual(result['kind'], 'stop', result)
        self.assertEqual(planner.status['ring']['blocker'], 500005)
        self.assertFalse(planner.junction_committed)

    def test_committed_entry_never_stops_for_a_ring_car_reaching_our_side(self):
        # The same car after a release: following only (it is beside/behind our nose).
        planner, ego, path = scene((-99.32, 314.73), 87.85, 3.5, 882.034, committed=True)
        cars = [car(-91.47, 328.89, 4.38, 1.55, 500004), car(-115.28, 332.55, 2.31, -5.65, 500005)]
        result = planner.evaluate(cars, ego, path, 55. / 3.6, 100.)
        self.assertNotEqual(result['kind'], 'stop', result)
        planner.evaluate(cars, dict(ego, speed_kmh=0.), path, 55. / 3.6, 100.1)
        self.assertTrue(planner.junction_committed)

    def test_ring_continuation_is_predicted_when_exit_link_matches_best(self):
        # 17:59:04.71: 500005 0.15-0.35 m from the ring link, PointPillars heading -72 deg.
        predictor = MapVehiclePredictor(roundabout_map()['links'])
        obj = car(-114.05, 330.01, 2.31, -7.12, 500005)
        times = np.arange(0., 8.01, .2)
        def passes_entry(paths):
            return any(np.min(np.linalg.norm(xy - [-99., 326.], axis=1)) < 1.5 for xy, _ in paths)
        self.assertFalse(passes_entry(predictor.predict(obj, times)))     # exit branch only
        self.assertTrue(passes_entry(predictor.predict(obj, times, every_match=True)))
        self.assertFalse(predictor.every_match)                          # per call, no shared flag
        # Its lanes (ring_entry) include the ring continuation past our entry.
        lanes = predictor.lane_paths(obj[:2], math.atan2(-7.12, 2.31))
        self.assertTrue(any(np.min(np.linalg.norm(xy - [-99., 326.], axis=1)) < 1.5 for xy, _ in lanes))

    def test_follow_the_passed_ring_car_in_right_behind_its_rear(self):
        # follow-20261010T092032Z 18:24:11.29 (replay: the first go of that cycle): 500005
        # about 9 m before our conflict point at 5.4-6.4 m/s, the next ring car 500006
        # ~39 m up the ring at 4.8 m/s: enter behind 500005.
        planner, ego, path = scene((-99.33, 314.48), 87.86, 0., 881.787, held=True, line_gap_m=1.)
        planner.speed_ratio = {500005: ring_entry.FIT_RATIO, 500006: ring_entry.FIT_RATIO}
        cars = [car(-102.4, 325.2, 5.4, -1.2, 500005), car(-120.0, 347.8, -1.2, -4.7, 500006)]
        result = planner.evaluate(cars, ego, path, 55. / 3.6, 100.)
        self.assertEqual(result['kind'], 'follow', result)
        self.assertEqual(result['reason'], 'follow merging lead with checked headway')
        self.assertGreater(result['v_mps'], 1.)
        self.assertIn(500005, planner.followable_ids)
        self.assertFalse(planner.junction_committed)      # the stop line releases (gui latches)
        planner.junction['held'] = False
        planner.evaluate(cars, ego, path, 55. / 3.6, 100.1)
        self.assertTrue(planner.junction_committed)

    def test_approaching_moving_ring_car_keeps_us_waiting_at_the_line(self):
        planner, ego, path = scene((-99.33, 314.47), 87.81, 0., 881.76)
        cars = [car(-91.47, 328.89, 4.38, 1.55, 500004), car(-115.28, 332.55, 2.31, -5.65, 500005)]
        for step in range(int((ring_entry.STANDOFF_S + 1.) * 15)):
            result = planner.evaluate(cars, ego, path, 55. / 3.6, 100. + step / 15.)
            self.assertEqual(result['kind'], 'stop', result)
            self.assertFalse(planner.junction_committed)


class InsideRingTests(unittest.TestCase):
    def test_inside_the_ring_entry_rules_no_longer_hold_us(self):
        # 17:59:08.61: nose at the ring lane edge (bumper 6.2 m past the line), logged
        # "junction: car without measured velocity, wait" for a new static track behind-left.
        planner, ego, path = scene((-98.02, 321.45), 64.92, .8, 888.988, ready=False, unknown_ids={500010})
        cars = [car(-98.19, 325.81, 3.39, .3, 500005), car(-84.98, 347.98, .28, 4.23, 500004),
                car(-103.06, 318.45, 0., 0., 500010, valid=False), car(-119.53, 340.89, .55, -3.8, 500008)]
        result = planner.evaluate(cars, ego, path, 55. / 3.6, 100.)
        self.assertNotEqual(result['kind'], 'stop', result)
        self.assertTrue(planner.junction_committed)

    def test_ring_car_crossing_at_our_nose_inside_does_not_stop_us(self):
        # 17:59:07.79 (a commit the old planner made 0.7 m before the line): 500005 at
        # 5.7 m/s crosses at our nose. Committed, nothing beside our front is waited for
        # (the entry decision no longer commits into such a gap); we stay committed.
        planner, ego, path = scene((-98.14, 321.32), 66.88, 11.9, 888.819, committed=True)
        cars = [car(-101.93, 325.48, 5.63, -1.11, 500005, heading=174.3), car(-83.72, 342.86, 1.09, 4.86, 500004),
                car(-119.61, 346.2, -1.89, -5.55, 500008), car(-102.17, 319.26, 0., 0., 500007, heading=243.9)]
        result = planner.evaluate(cars, ego, path, 55. / 3.6, 100.)
        self.assertNotEqual(result['kind'], 'stop', result)
        result = planner.evaluate(cars, dict(ego, speed_kmh=0.), path, 55. / 3.6, 100.1)
        self.assertTrue(planner.junction_committed)
        self.assertNotEqual(result['kind'], 'stop', result)

    def test_standing_ring_npc_beside_us_inside_does_not_hold_us(self):
        # 17:59:28.24: the NPC stopped at our left front for us; logged "predicted footprint
        # time overlap" (box heading 188 deg) while the raw LiDAR sweep was clear for 58.9 m.
        planner, ego, path = scene((-97.92, 321.62), 64.94, 0., 889.192)
        cars = [car(-99.11, 325.68, 0., 0., 500011, heading=188.), car(-118.2, 336.55, .98, -2.19, 500012),
                car(-84.65, 347.65, .66, 3.23, 500009)]
        result = planner.evaluate(cars, ego, path, 55. / 3.6, 100.)
        self.assertNotEqual(result['kind'], 'stop', result)
        self.assertGreater(result['v_mps'], 0.)
        self.assertTrue(planner.junction_committed)


class StandoffAtLineTests(unittest.TestCase):
    """Synthetic straight entry (test_junction_entry): a car standing beside our path."""
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 40.01, .5), np.zeros(81)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)

    def planner(self):
        planner = GapPlanner()
        planner.junction = dict(held=False, ready=True, unknown_ids=set(), line_gap_m=None)
        return planner

    def test_standing_car_beside_path_does_not_hold_the_entry(self):
        # Box corner inside the old 0.4 m shell, centre 2.5 m off the path, standing:
        # not on our path and not driving towards it.
        standing = junction_fixture.car(4., 2.5, 0., 0., 21, heading=-20.)
        planner = self.planner()
        result = planner.evaluate([standing], self.ego, self.path, 8., 100.)
        self.assertNotEqual(result['kind'], 'stop', result)
        self.assertTrue(planner.junction_committed)

    def test_standing_car_on_our_path_is_waited_behind(self):
        blocking = junction_fixture.car(6., .5, 0., 0., 22)
        planner = self.planner()
        for step in range(int((ring_entry.STANDOFF_S + 1.) * 15)):
            result = planner.evaluate([blocking], self.ego, self.path, 8., 100. + step / 15.)
            self.assertEqual(result['kind'], 'stop', result)
        self.assertFalse(planner.junction_committed)

    def test_at_rest_just_beyond_stop_window_is_not_an_approach(self):
        # 092032Z lap 2 rested 1.12-1.16 m before the line: the stop line now holds there,
        # so the planner must judge the entry (not "approach") or the line never releases.
        planner = self.planner()
        planner.junction.update(held=True, line_gap_m=1.15)
        self.assertFalse(planner._approaching_stop_line(0.))
        self.assertTrue(planner._approaching_stop_line(1.))


if __name__ == '__main__':
    unittest.main()
