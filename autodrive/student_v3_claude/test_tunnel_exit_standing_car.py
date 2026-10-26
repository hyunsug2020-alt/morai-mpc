"""Tunnel exit deadlock beside a standing car (follow-20261010T200235Z, random layout
R_KR_PR_K-city_2025/global_hard_13_57cfa702a5.json, 05:09:44-05:12:30 local).

20 m before the exit a scenario NPC stood ~65 deg across the left lane and the sport bike
(40200017) stood in our lane 8 m on. Coming alongside the car, its PointPillars box was
refitted 1.0-1.6 m toward us (corner inside our own body) while the car's own returns stayed
0.9-1.6 m from our side. The box outline zeroed the obstacle clearance of every path from our
pose: "previous path obstructed" at 19.5 km/h, "no safe bypass", a reverse, a bypass found
3 m back, the same stop 1-2 m later, for 2 min 45 s.

test_data/tunnel_exit_standing_car_20261010T200235Z.json holds the logged frames: the
aligned (wall-frame) ego and the clusters and detector boxes the planner used. Raw returns
are not logged; they are replaced by the faces of each logged cluster the sensor looks at.
"""
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial import cKDTree

import gui
import model_avoidance as ma
import test_tunnel_integration as integration
import tunnel_runtime
from gui import continue_static_plan
from lattice_planner import object_points
from test_tunnel_speed_shape import HostCurvature
from tunnel_map import TunnelMap
from tunnel_runtime import (TunnelRuntime, anchor_detector_box, rotation, standing_cars_on_returns,
                            swept_clearance)
from tunnel_static_portable import TunnelLaneHold
from ego_body import REAR_AXLE_SHIFT_M

HERE = Path(__file__).resolve().parent
FIXTURE = HERE/'test_data'/'tunnel_exit_standing_car_20261010T200235Z.json'
EGO = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
STOPS = ('05:09:44.03', '05:09:44.23', '05:09:46.76', '05:10:28.98', '05:10:34.15', '05:10:46.82',
         '05:12:21.21')


def seen_faces(x, y, length, width, heading_deg, ego, step=.1, inset=.1):
    """Vehicle-frame returns on the faces of a world box that the LiDAR looks at (the
    logged cluster boxes carry a 0.1 m voxel margin, hence the inset)."""
    axis = np.array([math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg))])
    cross = np.array([-axis[1], axis[0]])
    sensor = np.array([ego['x'], ego['y']]) + np.asarray(tunnel_runtime.LIDAR_ORIGIN_XY) @ rotation(ego).T
    seen = (sensor - [x, y]) @ axis, (sensor - [x, y]) @ cross
    halves = (max(.05, .5*length-inset), max(.05, .5*width-inset))
    points = []
    for k, (normal, other) in enumerate(((axis, cross), (cross, axis))):
        for side in (-1., 1.):
            if side*seen[k] > halves[k]:
                span = halves[1-k]
                for t in np.linspace(-span, span, max(2, int(2.*span/step)+1)):
                    points.append(np.array([x, y]) + side*halves[k]*normal + t*other)
    if not points:
        points = [np.array([x, y])]
    return (np.asarray(points) - [ego['x'], ego['y']]) @ rotation(ego)


def car(x, y, heading_deg, length=4.9, width=2.1, speed=0., valid=True, uid=500059):
    return (x, y, max(length, width), 'lidar_car', 'npc', speed, length, width, heading_deg,
            0., 0., uid, 0., None, valid, 0.)


def body_gap(points):
    """Distance from our bare 4.635 x 1.892 m body to the nearest point (gui.body_gap_m)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return float(np.min(np.hypot(np.maximum(np.abs(points[:, 0])-2.3175, 0.),
                                 np.maximum(np.abs(points[:, 1])-.946, 0.))))


def straight(length=30.):
    return np.column_stack((np.linspace(0., length, int(length/.25)+1), np.zeros(int(length/.25)+1)))


class AnchorGeometryTests(unittest.TestCase):
    """Synthetic: a car standing at 65 deg to our left, its end facing our side."""
    REAL = (2.6, 4.7, 65.)

    def scene(self, toward_m=1.3, real=REAL):
        x, y, heading = real
        returns = seen_faces(x, y, 4.9, 2.1, heading, EGO)
        axis = np.array([math.cos(math.radians(heading)), math.sin(math.radians(heading))])
        return returns, car(x, y, heading), car(x-toward_m*axis[0], y-toward_m*axis[1], heading)

    def test_box_refitted_into_our_body_moves_back_onto_the_cars_faces(self):
        returns, real, refit = self.scene()
        self.assertGreater(body_gap(returns), .8)
        self.assertEqual(body_gap(object_points([refit], EGO)), 0.)
        self.assertEqual(swept_clearance(straight(), object_points([refit], EGO), 30.), 0.)
        anchored, info = anchor_detector_box(refit, returns, EGO)
        self.assertGreater(info['moved_m'], 1.)
        self.assertEqual(anchored[6:9], refit[6:9])           # never turned or shrunk
        # Back on the measured faces: no closer than 0.2 m in front of them, all returns kept inside.
        self.assertGreater(body_gap(object_points([anchored], EGO)), body_gap(returns)-.35)
        inside = tunnel_runtime._inside_box(returns, np.array(anchored[:2]), math.radians(anchored[8]),
                                            anchored[6], anchored[7], .25)
        self.assertTrue(inside.all())
        self.assertGreaterEqual(swept_clearance(straight(), np.vstack((returns, object_points([anchored], EGO))),
                                                30.), 30.)

    def test_box_on_its_faces_and_far_or_moving_cars_are_unchanged(self):
        returns, real, refit = self.scene()
        self.assertIsNone(anchor_detector_box(real, returns, EGO)[1])
        moving = car(*refit[:2], refit[8], speed=3.)
        far = car(refit[0]+40., refit[1], refit[8])
        cars, log = standing_cars_on_returns([moving, far], returns, EGO)
        self.assertEqual(cars, [moving, far])
        self.assertIsNone(log)

    def test_unseen_remainder_and_sparse_returns_keep_the_detector_body(self):
        # A far-side return alone cannot authorize the near side (tunnel vehicle perception).
        parked = car(12., 0., 0., 4.8, 1.9)
        self.assertIsNone(anchor_detector_box(parked, np.array([[12., -.95]]), EGO)[1])
        # A drum-sized cluster in front of a car is not that car's face.
        drum = np.array([(9.+.2*i, .2*j) for i in range(3) for j in range(-2, 3)])
        self.assertIsNone(anchor_detector_box(parked, drum, EGO)[1])

    def test_returns_or_a_nearer_occluder_in_the_strip_keep_the_detector_box(self):
        returns, real, refit = self.scene()
        # Something measured in the strip the box would give up (a person at the car's end),
        # apart from the car's own returns.
        anchored, _ = anchor_detector_box(refit, returns, EGO)
        grid = np.array([(x, y) for x in np.arange(-1., 4., .1) for y in np.arange(1.2, 5., .1)])
        strip = grid[tunnel_runtime._inside_box(grid, np.array(refit[:2]), math.radians(refit[8]), 4.9, 2.1, -.2)
                     & ~tunnel_runtime._inside_box(grid, np.array(anchored[:2]), math.radians(anchored[8]), 4.9, 2.1, .3)
                     & (cKDTree(returns).query(grid)[0] > .9)]
        self.assertTrue(len(strip))
        person = strip[0] + np.array([[.1*i, .1*j] for i in range(-1, 2) for j in range(-1, 2)])
        kept, info = anchor_detector_box(refit, np.vstack((returns, person)), EGO)
        self.assertEqual(kept, refit)
        self.assertEqual(info['kept'], 'returns in the strip given up')
        # A nearer return on a ray into the strip could hide part of the car.
        returns_far, _, refit_far = self.scene(real=(3.6, 5.6, 65.))
        trace = {}
        anchored, _ = anchor_detector_box(refit_far, returns_far, EGO, debug=trace)
        self.assertNotEqual(anchored, refit_far)
        sensor = np.asarray(tunnel_runtime.LIDAR_ORIGIN_XY)
        corner = trace['given_up'][np.argmax(np.linalg.norm(trace['given_up']-sensor, axis=1))]
        occluder = sensor + .5*(corner-sensor) + np.array([[0., 0.], [.05, .05]])
        kept, info = anchor_detector_box(refit_far, np.vstack((returns_far, occluder)), EGO)
        self.assertEqual(kept, refit_far)
        self.assertEqual(info['kept'], 'a nearer return could hide part of the car')

    def test_real_contact_is_not_moved_away(self):
        # The car's own returns 0.1 m from our side: the box on them stays, every path stops.
        returns, real, _ = self.scene(real=(1.0, 3.3, 65.))
        self.assertLess(body_gap(returns), .15)
        anchored, info = anchor_detector_box(real, returns, EGO)
        self.assertLess(body_gap(object_points([anchored], EGO)), .15)
        self.assertEqual(swept_clearance(straight(), np.vstack((returns, object_points([anchored], EGO))), 30.), 0.)


class StandingCarRuntimeTests(unittest.TestCase):
    """The tunnel decision with the detector box and the car's returns (integration fixture)."""
    def runtime(self, returns, box, speed_kmh=0.):
        runtime = integration.TunnelIntegrationTests().runtime(returns)
        runtime.test_status['ego']['speed_kmh'] = speed_kmh
        runtime.test_odometry['speed_kmh'] = speed_kmh
        runtime.highway = SimpleNamespace(config=dict(lidar_regions=[dict(from_idx=0, to_idx=10000)]),
                                         active=lambda index: False, phase='OUTSIDE')
        runtime.latest_status = runtime.test_status
        runtime.vehicle_receiver = SimpleNamespace(snapshot=Mock(return_value=([box], True, {})))
        return runtime

    def scene(self, real=AnchorGeometryTests.REAL):
        geometry = AnchorGeometryTests()
        returns, _, refit = geometry.scene(real=real)
        return returns, refit

    def test_refitted_box_beside_us_no_longer_stops_the_clear_road(self):
        returns, refit = self.scene()
        before = self.runtime(returns, refit)
        with patch.object(tunnel_runtime, 'standing_cars_on_returns', lambda cars, points, ego: (list(cars), None)):
            before.infer()
        self.assertEqual(before.path_publisher.velocity_pub.items[-1].data, 0.)   # the drive's stop
        self.assertEqual(before._web_record['tunnel']['clearance_components_m']['obstacle'], 0.)
        self.assertEqual(before._web_record['stop_reason'], 'tunnel obstacle on path')
        after = self.runtime(returns, refit)
        self.assertTrue(after.infer())
        row = after._web_record
        self.assertFalse(row['emergency_stop'], row['stop_reason'])
        self.assertGreater(after.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertGreater(row['tunnel']['detector_anchor'][0]['moved_m'], 1.)
        self.assertGreaterEqual(row['tunnel']['clearance_components_m']['obstacle'], 30.)
        json.dumps(row)

    def test_car_touching_our_side_still_stops(self):
        returns, refit = self.scene(real=(1.0, 3.3, 65.))
        runtime = self.runtime(returns, car(1.0, 3.3, 65.))
        runtime.infer()
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertTrue(runtime._web_record['emergency_stop'])


class LoggedExitFrames(HostCurvature):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.data = json.loads(FIXTURE.read_text())
        waypoints = json.loads((HERE/'global_path.json').read_text())['waypoints']
        cls.world = np.array([[p['x'], p['y']] for p in waypoints])
        cls.control = gui.smooth_control_route(cls.world)
        cls.tunnel = TunnelMap.load(HERE/'tunnel_wall_map.json')
        cls.scope, _ = ma.make_tunnel_policies(cls.world, ma.tunnel_route_bounds(cls.world, cls.tunnel.active))

    def frame(self, prefix):
        return next(f for f in self.data['frames'] if f['time'].startswith(prefix))

    def inputs(self, frame, anchor=True, extra_returns=None):
        ego = dict(frame['ego'])
        nearest = int(np.argmin(np.linalg.norm(self.world-[ego['x'], ego['y']], axis=1)))
        dense = (self.control[[(nearest+i) % len(self.control) for i in range(240)]]
                 - [ego['x'], ego['y']]) @ rotation(ego)
        clusters, cars, returns = [], [], []
        for x, y, name, kind, length, width, heading, speed, vx, vy, uid, count in frame['objects']:
            row = (x, y, max(length, width), name, kind, speed, length, width, heading, vx, vy, uid,
                   {'point_count': count or 0}, None, name == 'lidar_car', 0.)
            if kind == 'npc':
                cars.append(row)
            else:
                clusters.append(row)
                returns.append(seen_faces(x, y, length, width, heading, ego))
        returns = np.vstack(returns if extra_returns is None else returns+[extra_returns])
        log = None
        if anchor:
            cars, log = standing_cars_on_returns(cars, returns, ego)
        points = np.vstack([returns]+([object_points(cars, ego)] if cars else []))
        return ego, nearest, dense, clusters+cars, points, returns, log

    def plan(self, frame, previous=None, anchor=True, extra_returns=None):
        ego, nearest, dense, objects, points, returns, log = self.inputs(frame, anchor, extra_returns)
        host = SimpleNamespace(tunnel=self.tunnel, tunnel_route_policy=self.scope,
                               _tunnel_lane_hold=TunnelLaneHold(), v6_latest=None, v6_worker=None)
        corridor, tree = self.tunnel.corridor(ego), cKDTree(points)

        def checks(path, horizon=60.):
            return TunnelRuntime.tunnel_path_checks(host, np.asarray(path)[:, :2], ego, points, corridor,
                                                    horizon, tree=tree)

        def clear(path, horizon=60.):
            return min(checks(path, horizon).values())

        def lane_ok(path, tolerance=1.5):
            local = np.asarray(path)[:, :2]
            length = float(np.linalg.norm(np.diff(local, axis=0), axis=1).sum())
            return clear(local, length) >= length-.3
        plan, hazards, *_ = TunnelRuntime.tunnel_model_static(
            host, dense, ego, objects, lane_ok, clear, self.tunnel.adjacent_offsets(ego, dense),
            previous, nearest, False, True)   # exit zone, as logged
        return SimpleNamespace(plan=plan, hazards=hazards, ego=ego, checks=checks, returns=returns,
                               anchor=log, policy=host._tunnel_model_log)

    def drive(self, times, anchor=True):
        """Open-loop chain: each frame continues the previous frame's saved plan."""
        saved, results = None, []
        for time_prefix in times:
            frame = self.frame(time_prefix)
            previous = continue_static_plan(saved, frame['ego'], 0.) if saved else None
            result = self.plan(frame, previous, anchor)
            results.append(result)
            plan = result.plan
            saved = None
            if plan is not None and not plan.get('blocked'):
                world = np.asarray(plan['path'])[:, :2] @ rotation(frame['ego']).T + [frame['ego']['x'], frame['ego']['y']]
                saved = dict(stamp=0., world_path=world, plan=plan)
        return results


class LoggedExitTests(LoggedExitFrames, unittest.TestCase):
    def test_logged_stops_had_the_detector_box_on_our_body(self):
        for time_prefix in STOPS:
            frame = self.frame(time_prefix)
            with self.subTest(time=time_prefix):
                self.assertEqual(frame['logged']['clearance']['obstacle'], 0.)
                ego, _, _, objects, _, returns, _ = self.inputs(frame, anchor=False)
                box = next(o for o in objects if o[3] == 'lidar_car' and math.hypot(o[0]-ego['x'], o[1]-ego['y']) < 8.)
                self.assertLess(body_gap(object_points([box], ego)), .2)
                self.assertGreater(body_gap(returns), .7)      # the car's own returns

    def test_replay_reproduces_the_drive_without_the_anchor(self):
        for times in (('05:09:43.84', '05:09:44.03'), ('05:10:31.00', '05:10:31.07', '05:10:33.89', '05:10:34.15')):
            results = self.drive(times, anchor=False)
            self.assertFalse(results[0].plan.get('blocked'))
            self.assertTrue(results[-1].plan.get('blocked'), times)
            self.assertEqual(results[-1].policy['bridge'].get('reason'), 'previous path obstructed')

    def test_committed_bypass_continues_past_the_standing_car(self):
        for times in (('05:09:43.84', '05:09:44.03', '05:09:44.23'),
                      ('05:10:31.00', '05:10:31.07', '05:10:33.89', '05:10:34.15'),
                      ('05:12:16.24', '05:12:17.69', '05:12:21.21')):
            results = self.drive(times)
            with self.subTest(times=times):
                for result in results:
                    plan = result.plan
                    self.assertFalse(plan.get('blocked'), result.policy)
                    self.assertEqual(plan['kind'], 'bypass')
                    self.assertEqual(plan['direction'], 1.)          # one side, the free left lane
                for result in results[1:]:
                    # The same manoeuvre: reused, or re-joined from the pose with the same
                    # offset and transition. (05:10:33.89, 2.8 s after the open-loop previous
                    # frame: the old join's real front corner would pass the car's box inside
                    # the 0.25 m swept margin; the same 2.8 m / 18 m bypass from the pose is clear.)
                    self.assertTrue(result.plan.get('reused') or
                                    (abs(result.plan['offset_m']-results[0].plan['offset_m']) < .05 and
                                     result.plan['transition_m'] == results[0].plan['transition_m']),
                                    result.policy)

    def test_every_logged_stop_now_has_a_checked_bypass_clear_of_the_real_returns(self):
        for time_prefix in STOPS:
            result = self.plan(self.frame(time_prefix))
            with self.subTest(time=time_prefix):
                plan = result.plan
                self.assertFalse(plan.get('blocked'), result.policy)
                self.assertEqual(plan['direction'], 1.)
                self.assertLessEqual(plan['curvature'], gui.MAX_CONTROL_CURVATURE)
                self.assertGreaterEqual(min(result.checks(plan['path'], 35.).values()), 35.)
                # The car and the bike as measured: the planner's 0.25 m swept margins, and more,
                # around the real body (front 3.845 m ahead of the rear-axle pose).
                self.assertGreaterEqual(swept_clearance(plan['path'], result.returns, 35., .25,
                                                        lateral_margin=.3, body_shift_m=REAR_AXLE_SHIFT_M), 35.)
                self.assertTrue(result.anchor and 'moved_m' in result.anchor[0], result.anchor)

    def test_a_truly_blocked_exit_still_stops(self):
        # The same stop with a barrier across the whole tunnel 14 m ahead.
        frame = self.frame('05:10:46.82')
        barrier = np.column_stack((np.full(141, 14.), np.linspace(-7., 7., 141)))
        result = self.plan(frame, extra_returns=barrier)
        self.assertTrue(result.plan is None or result.plan.get('blocked'))
        self.assertLess(result.checks(straight(30.), 30.)['obstacle'], 14.)


class StopReasonTests(unittest.TestCase):
    def test_zero_geometry_is_not_reported_as_lidar_coverage(self):
        import test_tunnel_pose_refresh_race as race
        for obstacle, expected in ((0., 'tunnel obstacle on path'), (60., 'tunnel lidar coverage insufficient')):
            with self.subTest(obstacle=obstacle):
                runtime, ego, original, latest = race.TunnelPoseRefreshRaceTests().runtime()
                decision = runtime.checked_tunnel_plan(runtime.test_route, ego, ego, original,
                    np.empty((0, 2)), 4., 0., True,
                    dict(valid=True, pose_valid=True, wall=60., lane=60., obstacle=obstacle))[0]
                self.assertTrue(decision.emergency)
                self.assertEqual(decision.stop_reason, expected)


if __name__ == '__main__':
    unittest.main()
