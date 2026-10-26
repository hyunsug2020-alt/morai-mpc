"""Highway vehicles without a measured velocity are moving traffic (user 2026-10-11).

"고속도로에서는 무조건 달리는 차밖에 없다": a car whose speed the detector has not
validated yet is never planned as a standing body. follow-20261010T184845Z lap 1,
03:53:00-05.5: cars 37-55 m ahead (PointPillars score 0.23-0.66, 5-7 Hz, uid
churn 500028 -> 500029, 500030, 500031) stayed velocity_valid=false for ~4 s
while driving ~40 km/h; as standing boxes they were a 48 m stop line at 82 km/h
during the 434 -> 435 change ("highway braking trajectory conflict", 84 -> 44
km/h). Measured slow or standing cars must still brake us.
"""
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

from gui import Runtime, HdMapLanes, RosPathPublisher, smooth_control_route
from highway_planner import HighwayPlanner, NearFieldMemory, local_points
from highway_observations import (HighwayVehicleMotion, UNMEASURED_CAR_KMH,
                                  UNMEASURED_CAR_MIN_KMH)
from lidar_corridor import LidarCorridor
from planning.highway.object_policy import (highway_measured_objects, highway_objects,
                                            highway_scan_mask_objects, highway_static_objects,
                                            motion_inferred)


HERE = Path(__file__).parent
RTF = .8


def car(x, y, uid, valid=False, velocity=(0., 0.), age=0., length=4.8, width=1.9, heading=0.):
    vx, vy = velocity
    return (x, y, max(length, width), 'lidar_car', 'npc', math.hypot(vx, vy), length, width,
            heading, vx, vy, uid, 0., None, valid, age)


class StraightLanes:
    """Three same-flow lanes along +x (y = 0, 3.5, 7) and one opposite lane (y = -10)."""
    def __init__(self):
        x = np.arange(-200., 400., 1.)
        lanes = [np.column_stack((x, np.full(len(x), y))) for y in (0., 3.5, 7.)]
        lanes.append(np.column_stack((x[::-1], np.full(len(x), -10.))))
        self.points = np.vstack(lanes)
        self.link_of = np.concatenate([np.full(len(x), k) for k in range(len(lanes))])
        self.tree = cKDTree(self.points)


EGO = dict(x=0., y=0., heading_deg=0., speed_kmh=80.)


class VehicleMotionStage(unittest.TestCase):
    def setUp(self):
        self.lanes = StraightLanes()
        self.stage = HighwayVehicleMotion()

    def update(self, cars, stamp=10., ego=EGO):
        return self.stage.update(cars, stamp, ego, self.lanes, RTF)

    def test_unmeasured_car_ahead_moves_at_the_assumed_lane_speed(self):
        (out,), info = self.update([car(40., .3, 1)])
        expected = UNMEASURED_CAR_KMH/3.6*RTF          # measured (wall) units
        self.assertTrue(out[14])
        self.assertEqual(out[16], 'assumed')
        self.assertTrue(motion_inferred(out))
        np.testing.assert_allclose(out[9:11], (expected, 0.), atol=1e-6)
        self.assertAlmostEqual(out[5], expected)
        self.assertEqual(tuple(out[:2]), (40., .3))
        self.assertEqual(info['inferred'][0]['kmh'], UNMEASURED_CAR_KMH)
        # A mover, never a standing body.
        self.assertEqual(highway_static_objects([out]), [])

    def test_measured_and_non_traffic_cars_are_unchanged(self):
        cars = [car(40., 0., 1, valid=True, velocity=(5., 0.)),   # detector-measured
                car(2., 3.5, 2),                                    # beside
                car(-20., 0., 3),                                   # behind
                car(40., 9.5, 4),                                   # 2.5 m off any lane
                car(40., -10., 5)]                                  # opposite flow
        out, info = self.update(cars)
        self.assertEqual(out, cars)
        self.assertEqual(info['inferred'], [])

    def test_slower_measured_flow_lowers_the_assumption_but_never_raises_it(self):
        default, minimum = UNMEASURED_CAR_KMH/3.6*RTF, UNMEASURED_CAR_MIN_KMH/3.6*RTF
        for neighbour_mps, expected in ((.5*(default+minimum), .5*(default+minimum)),
                                        (1., minimum), (20., default)):
            with self.subTest(neighbour_mps=neighbour_mps):
                stage = HighwayVehicleMotion()
                out, _ = stage.update([car(40., 0., 1), car(60., 3.5, 2, valid=True,
                                       velocity=(neighbour_mps, 0.))], 10., EGO, self.lanes, RTF)
                self.assertAlmostEqual(out[0][5], expected)
        # A measured car 80 m away is not nearby flow.
        out, _ = HighwayVehicleMotion().update([car(40., 0., 1), car(120., 0., 2, valid=True,
                                                velocity=(1., 0.))], 10., EGO, self.lanes, RTF)
        self.assertAlmostEqual(out[0][5], default)

    def test_retained_unmeasured_track_moves_with_its_assumed_speed(self):
        (out,), _ = self.update([car(40., 0., 1, age=.4)])
        speed = UNMEASURED_CAR_KMH/3.6*RTF
        self.assertAlmostEqual(out[0], 40.+speed*.4)
        self.assertEqual(out[15], .4)

    def test_lane_projected_measurement_ignores_lateral_centroid_jitter(self):
        # 10 m/s along the lane with +-0.9 m lateral jumps (the detector reset
        # its unvalidated history on such jumps; uid 500029 never validated).
        for k, (x, y) in enumerate(((40., -.9), (41.5, .8), (43., -.7), (44.5, .9), (46., -.8))):
            (out,), info = self.update([car(x, y, 7)], stamp=10.+.15*k)
        self.assertEqual(out[16], 'tracked')
        self.assertAlmostEqual(out[5], 10., delta=.3)
        self.assertAlmostEqual(out[10], 0.)

    def test_uid_churn_continues_the_dropped_track_history(self):
        stamp = 10.
        for x in (40., 41.5, 43.):
            self.update([car(x, .2, 1)], stamp=stamp)
            stamp += .15
        # uid 1 is only a retained prediction now; uid 2 appears where it went.
        out, info = self.update([car(43., .2, 1, age=.3), car(46., -.4, 2)], stamp=10.6)
        self.assertEqual(info['bridged'], [dict(from_uid=1, uid=2, gap_s=.3)])
        new = next(o for o in out if o[11] == 2)
        self.assertEqual(new[16], 'tracked')               # 4 samples over 0.6 s
        self.assertAlmostEqual(new[5], 10., delta=.3)

    def test_no_bridge_to_another_lane_or_backwards(self):
        stamp = 10.
        for x in (40., 41.5, 43.):
            self.update([car(x, 0., 1)], stamp=stamp)
            stamp += .15
        _, info = self.update([car(45., 3.5, 2), car(30., 0., 3)], stamp=10.6)
        self.assertEqual(info['bridged'], [])

    def test_measured_standing_body_stays_static(self):
        # Five fresh detections at one spot: a measured standing body brakes us.
        for k in range(5):
            (out,), _ = self.update([car(40.+.05*(-1)**k, 0., 9)], stamp=10.+.15*k)
        self.assertEqual(out[16], 'tracked')
        self.assertEqual(out[5], 0.)
        self.assertEqual(highway_static_objects([out]), [out])

    def test_inferred_motion_never_masks_lidar_returns(self):
        (assumed,), _ = self.update([car(40., 0., 1)])
        measured = car(60., 3.5, 2, valid=True, velocity=(8., 0.))
        self.assertEqual(highway_measured_objects([assumed, measured]), [measured])
        self.assertEqual([o[11] for o in highway_scan_mask_objects([assumed, measured])], [2])

    def test_camera_parked_vehicle_ahead_is_an_assumed_mover(self):
        self.update([])
        parked = (40., 0., 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 0., 0., 0., 123, 0., None)
        behind = (-20.,) + parked[1:11] + (124, 0., None)
        drum = (40., 3.5, 1.1, 'fusion_obstacle', 'obstacle', 0., 1.1, 1.1, 0., 0., 0., 125, 0., None)
        out = self.stage.camera_vehicles([parked, behind, drum], EGO, self.lanes, RTF, age_s=.2)
        speed = UNMEASURED_CAR_KMH/3.6*RTF
        self.assertEqual(out[0][3:5], ('fusion_parked_vehicle', 'npc'))
        self.assertTrue(out[0][14])
        self.assertEqual(out[0][16], 'assumed')
        self.assertAlmostEqual(out[0][0], 40.+speed*.2)
        self.assertEqual(out[1:], [behind, drum])
        self.assertEqual(highway_static_objects(out), [behind, drum])
        self.assertEqual(highway_measured_objects(out), [behind, drum])

    def test_fresh_inferred_car_owns_its_camera_duplicate(self):
        (assumed,), _ = self.update([car(40., 0., 1)])
        parked = (41., .2, 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 0., 0., 0., 123, 0., None)
        self.assertEqual([o[3] for o in highway_objects([parked], [assumed])], ['lidar_car'])


class RuntimeHarness:
    """HighwayRuntime.infer_highway with recorded or synthetic sensor snapshots."""
    def __init__(self, lanes, world, smooth):
        self.world, self.smooth = world, smooth
        runtime = Runtime.__new__(Runtime)
        runtime.highway = HighwayPlanner(json.loads((HERE/'highway_mission.json').read_text()), world)
        runtime.near_field_memory = NearFieldMemory()
        runtime.lidar = LidarCorridor()
        runtime.lanes = lanes
        runtime.receiver = SimpleNamespace(rates=lambda: dict(front=30., left=30., right=30.))
        runtime.actions = ['DRIVE', 'STOP']
        runtime.gallery_matrix, runtime.gallery = np.empty((0, 2)), []
        publisher = RosPathPublisher.__new__(RosPathPublisher)
        message = lambda data=None: SimpleNamespace(data=data)
        publisher.Float32 = publisher.String = message
        publisher.PoseStamped = lambda: SimpleNamespace(header=None, pose=SimpleNamespace(
            position=SimpleNamespace(x=0., y=0.)))
        publisher.PathMessage = lambda: SimpleNamespace(header=SimpleNamespace(stamp=None, frame_id=None),
                                                        poses=[])
        publisher.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: 1.))
        sink = lambda: SimpleNamespace(items=[], publish=lambda m: None)
        publisher.waypoints_pub = publisher.velocity_pub = publisher.mode_pub = sink()
        publisher.path_pub = publisher.status_pub = sink()
        publisher.cruise_mps, publisher.collision_stop = 25., False
        publisher.publish_camera_rates = lambda rates: None
        publisher.publish_corridor = lambda ego, corridor: None
        runtime.path_publisher = publisher
        self.runtime = runtime
        self.directory = tempfile.TemporaryDirectory()
        runtime.args = SimpleNamespace(status=Path(self.directory.name)/'status.json')

    def close(self):
        if hasattr(self.runtime, '_decision_log'):
            self.runtime._decision_log.close()
        self.directory.cleanup()

    def step(self, ego, cars, vehicle_stamp, rtf=RTF, fused=(), fusion_stamp=None, fusion_age=.1):
        runtime = self.runtime
        ego = dict(ego, age_s=.01)
        runtime.sim_clock = SimpleNamespace(rtf=rtf, record=lambda: dict(rtf=rtf, rtf_measured=True))
        status = dict(stamp=vehicle_stamp, age_s=.05, ready=True, active=True, vehicles=[])
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (list(cars), True, dict(status)))
        fusion_stamp = ego['stamp']-fusion_age if fusion_stamp is None else fusion_stamp
        runtime.object_receiver = SimpleNamespace(
            snapshot_with_time=lambda: ([tuple(o) for o in fused], fusion_stamp, fusion_age))
        runtime.lidar.pts = np.zeros((0, 2), np.float32)
        runtime.lidar.received = time.monotonic()
        index = 1800+int(np.argmin(np.linalg.norm(self.world[1800:3600]-[ego['x'], ego['y']], axis=1)))
        dense = local_points(self.smooth[index:index+640], ego)
        prediction = np.column_stack((np.arange(1., 21.), np.zeros(20)))
        with patch.dict('os.environ', {'STUDENT_LOG_DIR': self.directory.name}):
            runtime.infer_highway(dict(ego=ego, path=dict(nearest_idx=index, distance_m=0.)), ego,
                                  dense, dense[:20], dense[::10], prediction, np.array([.01, .99]),
                                  np.empty(0), np.empty(0), 3., time.perf_counter())
        return runtime._web_record


def recorded_cars(snapshot):
    """VehicleReceiver.snapshot() tuples of a recorded detector message."""
    cars = []
    for vehicle in snapshot['vehicles']:
        (x, y), (vx, vy) = vehicle['position'], vehicle['velocity']
        length, width = vehicle['size'][:2]
        cars.append((x, y, max(length, width), 'lidar_car', 'npc', math.hypot(vx, vy), length, width,
                     vehicle['heading_deg'], vx, vy, vehicle['uid'], 0., None,
                     bool(vehicle['velocity_valid']), float(vehicle.get('prediction_age_s', 0.))))
    return cars


class RecordedUnmeasuredCarsReplay(unittest.TestCase):
    """Recorded detector output, fused observations, pose, stamps and RTF replayed
    through the production highway runtime. LiDAR returns are not recorded; the
    recorded raw clearance was free except single 45 m hits at 03:53:03.65/04.25."""
    @classmethod
    def setUpClass(cls):
        world = np.array([[p['x'], p['y']] for p in
                          json.loads((HERE/'global_path.json').read_text())['waypoints']])
        cls.world, cls.smooth = world, smooth_control_route(world)
        cls.lanes = HdMapLanes(HERE/'link_set.json')
        cls.lanes.set_route(world)
        cls.frames = json.loads((HERE/'test_data/highway_unmeasured_cars_184845Z.json').read_text())['frames']
        cls.fixed = cls.replay()
        cls.unfixed = cls.replay(disabled=True)

    @classmethod
    def replay(cls, disabled=False):
        harness = RuntimeHarness(cls.lanes, cls.world, cls.smooth)
        if disabled:
            harness.runtime._highway_vehicle_motion = SimpleNamespace(
                update=lambda cars, *args: (list(cars), {}),
                camera_vehicles=lambda objects, *args: list(objects))
        try:
            if disabled:
                # This historical control predates both inferred highway flow
                # and measured semantic-motion preservation. Restore both
                # contracts to reproduce the recorded standing-car stop.
                # The recorded historical control also predates the highway
                # publication-continuity join. Keep it historical; the fixed
                # replay below exercises the current production contracts.
                with patch('planning.highway.object_policy._moving_semantic_npc', return_value=False), \
                        patch('planning.highway.path_continuity.published_reference', return_value=None):
                    return [(frame, harness.step(frame['ego'], recorded_cars(frame['vehicles']),
                                                 frame['vehicles']['stamp'], frame['rtf'], frame['fused'],
                                                 frame['fusion_stamp'], frame['fusion_age_s']))
                            for frame in cls.frames]
            return [(frame, harness.step(frame['ego'], recorded_cars(frame['vehicles']),
                                         frame['vehicles']['stamp'], frame['rtf'], frame['fused'],
                                         frame['fusion_stamp'], frame['fusion_age_s']))
                    for frame in cls.frames]
        finally:
            harness.close()

    def window(self, records, start='03:53:00.00', end='03:53:05.60'):
        return [(frame, record) for frame, record in records if start <= frame['local'] <= end]

    def test_fixture_reproduces_the_recorded_standing_car_emergency(self):
        logged = [f['local'] for f in self.frames if f['logged']['emergency']]
        self.assertEqual(logged[0], '03:53:02.95')
        stopped = [f['local'] for f, r in self.window(self.unfixed, '03:53:02.90', '03:53:04.10')
                   if r['emergency_stop']]
        self.assertEqual(stopped, logged)
        boxed = [r['highway']['static_clearance_sources']['boxed_clear_m']
                 for f, r in self.window(self.unfixed, '03:53:02.90', '03:53:04.10')]
        self.assertLess(max(boxed), 50.)                   # standing boxes 33-48 m ahead

    def test_lane_change_follows_the_moving_cars_without_emergency(self):
        for frame, record in self.window(self.fixed):
            with self.subTest(t=frame['local']):
                self.assertFalse(record['emergency_stop'])
                self.assertNotEqual(record['reason'], 'highway braking trajectory conflict')
                # A gradual reference from the measured speed, never a stop command.
                self.assertGreater(record['planner_cap_kmh'], .85*frame['ego']['speed_kmh'])
                plan = record['highway']['speed_plan']
                self.assertTrue(plan['reference_checked'])
                self.assertGreaterEqual(plan['acceleration_mps2'], -2.6)

    def test_former_stop_line_cars_are_movers_with_measured_or_assumed_speed(self):
        bridged = [b for _, r in self.fixed for b in r['highway']['vehicle_motion'].get('bridged', [])]
        self.assertIn((500028, 500029), [(b['from_uid'], b['uid']) for b in bridged])
        for frame, record in self.window(self.fixed, '03:53:02.90', '03:53:04.10'):
            with self.subTest(t=frame['local']):
                self.assertEqual(record['highway']['static_clearance_sources']['boxed_clear_m'], 170.)
                inferred = {d['uid']: d for d in record['highway']['vehicle_motion']['inferred']}
                self.assertEqual(inferred[500029]['source'], 'tracked')
                self.assertGreater(inferred[500029]['kmh'], 30.)
                self.assertLess(inferred[500029]['kmh'], 65.)
                self.assertIn(inferred[500030]['source'], ('tracked', 'assumed'))
                self.assertGreaterEqual(inferred[500030]['kmh'], UNMEASURED_CAR_MIN_KMH)
        # The binding car ahead is followed at the comfort rate.
        braking = [r['highway']['speed_plan']['acceleration_mps2']
                   for f, r in self.window(self.fixed, '03:53:03.40', '03:53:04.30')]
        self.assertTrue(all(-2.6 <= a < 0. for a in braking), braking)


class MeasuredSlowCarsStillBrake(unittest.TestCase):
    """Detector-measured slow or standing cars keep their braking and stop."""
    # Surveyed lane centres (x) beside route index 2800+80 (y = -57.4): A2256W000179,
    # 408 (route), 434, 435, 421. Blocking all of them leaves no escape lane.
    LANES_X = (64.04, 67.5, 71.13, 74.66, 78.16)

    @classmethod
    def setUpClass(cls):
        world = np.array([[p['x'], p['y']] for p in
                          json.loads((HERE/'global_path.json').read_text())['waypoints']])
        cls.world, cls.smooth = world, smooth_control_route(world)
        cls.lanes = HdMapLanes(HERE/'link_set.json')
        cls.lanes.set_route(world)

    def drive(self, make_cars, frames=4, speed_kmh=80., ahead_m=40., car_mps=0.):
        """Ego on the route (index 2800); cars ahead (make_cars(route y, scan)),
        moving car_mps (measured units) along the route between 0.15 s scans."""
        index = 2800
        heading = math.degrees(math.atan2(*(self.smooth[index+3]-self.smooth[index])[::-1]))
        x, y = self.world[index]
        harness = RuntimeHarness(self.lanes, self.world, self.smooth)
        try:
            records = []
            for k in range(frames):
                stamp = 100.+.15*k
                target = self.world[index+int(round((ahead_m+car_mps*.15*k)/.5))]
                ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed_kmh, stamp=stamp)
                records.append(harness.step(ego, make_cars(target, k), stamp-.05))
            return records
        finally:
            harness.close()

    def blocked(self, **kwargs):
        """One car in every lane at the route point (y) ahead."""
        return lambda xy, k: [car(x, xy[1], 900+i, heading=-90., **kwargs)
                              for i, x in enumerate(self.LANES_X)]

    def test_measured_standing_cars_ahead_still_stop(self):
        records = self.drive(self.blocked(valid=True))
        self.assertTrue(records[-1]['emergency_stop'])
        self.assertEqual(records[-1]['planner_cap_kmh'], 0.)
        self.assertEqual(records[-1]['highway']['vehicle_motion']['inferred'], [])
        self.assertLess(records[-1]['highway']['static_clearance_sources']['boxed_clear_m'], 40.)

    def test_measured_slow_cars_ahead_still_brake(self):
        slow = 8./3.6*RTF                                   # 8 km/h simulated, measured units
        outcome = {}
        for ahead_m in (35., 45., 60., 90.):
            last = self.drive(self.blocked(valid=True, velocity=(0., -slow)), ahead_m=ahead_m)[-1]
            self.assertEqual(last['highway']['vehicle_motion']['inferred'], [])
            self.assertLess(last['planner_cap_kmh'], 80.)
            plan = last['highway']['speed_plan']
            if ahead_m == 60.:
                self.assertTrue(plan['braking_reference_extended'])
                self.assertTrue(plan['execution_reference_checked'])
                self.assertGreater(last['planner_cap_kmh'], 70.)
            outcome[ahead_m] = ('stop' if last['emergency_stop'] else
                                (plan['braking_limit_mps2'], round(plan['acceleration_mps2'], 1)))
        self.assertEqual(outcome[35.], 'stop')              # too close: emergency stop
        # Actual bumper room is 38.84 m for 20 m/s closing. Even immediate
        # 5 m/s² needs 40 m; the delayed scalar command contacts sooner.
        self.assertEqual(outcome[45.], 'stop')
        self.assertEqual(outcome[60.][0], 5.)              # checked stronger scalar reference
        self.assertLessEqual(outcome[60.][1], -4.)
        self.assertEqual(outcome[90.][0], 2.5)             # distant lead retains comfort braking
        self.assertLess(outcome[90.][1], -1.5)

    def test_unmeasured_body_standing_at_one_spot_is_measured_and_stopped_for(self):
        records = self.drive(self.blocked(), frames=5)
        first = records[0]['highway']['vehicle_motion']['inferred']
        self.assertEqual({d['source'] for d in first}, {'assumed'})    # movers until measured
        self.assertEqual(records[0]['highway']['static_clearance_sources']['boxed_clear_m'], 170.)
        last = records[-1]['highway']['vehicle_motion']['inferred']
        self.assertEqual({(d['source'], d['kmh']) for d in last}, {('tracked', 0.)})
        self.assertTrue(records[-1]['emergency_stop'])                  # measured standing: stop

    def test_unmeasured_moving_car_ahead_is_followed_not_stopped_for(self):
        records = self.drive(lambda xy, k: [car(*xy, 902, heading=-90.)], frames=5, ahead_m=60.,
                             car_mps=10.)
        for record in records:
            self.assertFalse(record['emergency_stop'])
            self.assertGreater(record['planner_cap_kmh'], 0.)
            sources = record['highway']['static_clearance_sources']
            self.assertGreaterEqual(sources['boxed_clear_m'], 170.)
        inferred = [r['highway']['vehicle_motion']['inferred'][0] for r in records]
        self.assertEqual([d['source'] for d in inferred], ['assumed']*4+['tracked'])  # 5 scans, 0.6 s
        self.assertAlmostEqual(inferred[-1]['kmh'], 10./RTF*3.6, delta=2.)

if __name__ == '__main__':
    unittest.main()
