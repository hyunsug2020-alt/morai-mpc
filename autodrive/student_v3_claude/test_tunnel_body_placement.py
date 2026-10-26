"""The real body on the rear-axle pose in the tunnel checks (ego_body).

MORAI's ego pose is the rear axle: the 4.635 x 1.892 m body reaches 3.845 m ahead of
it and 0.79 m behind. The tunnel checks placed a +-2.3175 m box on the pose, 1.53 m
behind the real body. Replays of logged frames (test_data/tunnel_body_placement_20261010.json):

* follow-20261010T210040Z (global_hard_13): 5.5 min of "tunnel no safe bypass": the
  checked rear corner overlapped a wreck the real body cleared by ~1.1 m.
* follow-20261010T211143Z (global_hard_12): a bollard read 2.0 m from the bumper while
  the real front was ~0.4 m from it.
* follow-20261010T200235Z (global_hard_13): a reverse escape turned the body 24 deg to the
  road heading; the front-left corner swung into a drum (~05:09:05).
"""
import json
import math
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
import ped_guard
import tunnel_runtime
from ego_body import (EGO_CENTRE_M, EGO_FRONT_M, EGO_HALF_LENGTH_M, EGO_REAR_M, REAR_AXLE_SHIFT_M,
                      ROAD_LEGACY_SHIFT_M, body_gap, inside_body, reverse_sweep_hits)
from lane_corridor import LaneCorridor
from live_path_safety import owner_body_shift
from path_safety import PathSafety
from test_tunnel_exit_standing_car import LoggedExitFrames, seen_faces, straight
from tunnel_runtime import corridor_at_pose, rotation, swept_clearance, swept_clearances

original = tunnel_runtime.TunnelRuntime.tunnel_model_static

HERE = Path(__file__).resolve().parent
FIXTURE = json.loads((HERE/'test_data'/'tunnel_body_placement_20261010.json').read_text())


def box_points(x, y, length, width, heading_deg, ego, step=.05):
    """Vehicle-frame outline of a world box (its exact edges, no inset)."""
    yaw = math.radians(heading_deg)
    axis, cross = np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])
    points = []
    for u in np.linspace(-.5, .5, max(3, int(max(length, width)/step)+1)):
        points += [u*length*axis+side*.5*width*cross for side in (-1., 1.)]
        points += [side*.5*length*axis+u*width*cross for side in (-1., 1.)]
    return (np.asarray(points)+[x-ego['x'], y-ego['y']]) @ rotation(ego)


def legacy_gap(points):
    """The gap the checks believed: the same box centred on the rear-axle pose."""
    return body_gap(np.asarray(points, dtype=float)[:, :2]+[EGO_CENTRE_M, 0.])


class EgoBodyTests(unittest.TestCase):
    def test_one_body_for_every_check(self):
        self.assertAlmostEqual(EGO_FRONT_M, 3.845)
        self.assertAlmostEqual(EGO_REAR_M, .79)
        self.assertAlmostEqual(EGO_HALF_LENGTH_M, 2.3175)
        self.assertAlmostEqual(EGO_CENTRE_M, 1.5275)
        self.assertEqual((ped_guard.EGO_FRONT_M, ped_guard.EGO_REAR_M), (EGO_FRONT_M, EGO_REAR_M))
        self.assertEqual(owner_body_shift('TUNNEL'), REAR_AXLE_SHIFT_M)
        self.assertEqual(owner_body_shift('ROAD'), ROAD_LEGACY_SHIFT_M)
        self.assertTrue(inside_body([[3.8, .9]])[0])
        self.assertFalse(inside_body([[-.85, 0.]])[0])
        self.assertAlmostEqual(body_gap([[4.845, 0.]]), 1.)
        self.assertAlmostEqual(body_gap([[-1.79, 0.]]), 1.)

    def test_swept_body_reaches_the_real_front_and_rear(self):
        path = straight(20.)
        # Front bumper + 0.25 m margin is 4.095 m ahead of each sample, not 2.5675 m.
        self.assertEqual(swept_clearance(path, [[10., 0.]], 20., body_shift_m=REAR_AXLE_SHIFT_M), 6.)
        self.assertEqual(swept_clearance(path, [[10., 0.]], 20.), 7.5)                  # road default
        self.assertEqual(swept_clearance(path, [[3.9, 0.]], 20., body_shift_m=REAR_AXLE_SHIFT_M), 0.)
        # Rear bumper 0.79 m behind the pose: 1 m behind it touches, 2 m behind it is free.
        self.assertEqual(swept_clearance(path, [[-1., 0.]], 20., body_shift_m=REAR_AXLE_SHIFT_M), 0.)
        self.assertEqual(swept_clearance(path, [[-2., 0.]], 20., body_shift_m=REAR_AXLE_SHIFT_M), 20.)
        self.assertEqual(swept_clearance(path, [[-2., 0.]], 20.), 0.)
        points = np.array([[10., .3], [6., -1.1], [-1., .5], [4., 1.3]])
        for margins in ((.25,), (.1, .35, .8)):
            multi = swept_clearances(path, points, 20., .25, margins, body_shift_m=REAR_AXLE_SHIFT_M)
            single = [swept_clearance(path, points, 20., .25, lateral_margin=m, body_shift_m=REAR_AXLE_SHIFT_M)
                      for m in margins]
            self.assertEqual(multi, single)

    def test_front_corner_swing_on_a_curve(self):
        # A 0.15 1/m left arc: the real front-right corner swings out on a radius of
        # 8.5 m (+ margin) about the turn centre, the box centred on the pose only 7.96 m.
        k = .15
        s = np.arange(0., 12.01, .25)
        arc = np.column_stack((np.sin(k*s)/k, (1.-np.cos(k*s))/k))
        point = [[6.51, 1.2]]
        self.assertEqual(swept_clearance(arc, point, 12., .25, body_shift_m=REAR_AXLE_SHIFT_M), 2.5)
        self.assertGreater(swept_clearance(arc, point, 12., .25), 11.9)

    def test_static_boxes_and_lane_corners_use_the_real_body(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        box = (5.5, 0., .4, 'drum', 'obstacle', 0., .4, .4, 0.)
        path = straight(30.)
        # Box face 5.3 m ahead; bumper 3.845 m + 0.2 m margin: contact after 1.255 m.
        self.assertEqual(gui.static_clear_distance(path, ego, [box], 30., body_shift_m=REAR_AXLE_SHIFT_M), 1.5)
        self.assertEqual(gui.static_clear_distance(path, ego, [box], 30.), 3.)
        from tunnel_static_portable import static_obstacle
        self.assertEqual(static_obstacle.static_clear_distance(path, ego, [box], 30., body_shift_m=REAR_AXLE_SHIFT_M), 1.5)
        # Lane corners: the front corner on the left edge 1.2 m out from the route.
        route = straight(40.)
        corridor = LaneCorridor(route, np.full(len(route), 1.2), np.full(len(route), -3.), body_shift_m=REAR_AXLE_SHIFT_M)
        station = np.arange(0., 20.01, .25)
        turning = np.column_stack((station, .004*station**2))
        legacy = LaneCorridor(route, np.full(len(route), 1.2), np.full(len(route), -3.))
        self.assertLess(corridor.clear_distance(turning, 20.), legacy.clear_distance(turning, 20.))
        moved = corridor_at_pose(corridor, dict(x=0., y=0., heading_deg=0.), dict(x=1., y=0., heading_deg=0.))
        self.assertEqual(moved.body_shift_m, REAR_AXLE_SHIFT_M)

    def test_tunnel_live_guard_measures_gaps_from_the_real_bumpers(self):
        tunnel, road = PathSafety(body_shift_m=REAR_AXLE_SHIFT_M), PathSafety()
        self.assertAlmostEqual(tunnel.front_m, EGO_FRONT_M)
        self.assertAlmostEqual(tunnel.rear_m, EGO_REAR_M)
        self.assertAlmostEqual(road.front_m, EGO_HALF_LENGTH_M)
        path = straight(40.)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=10.)
        car = (12., 0., 4.6, 'lidar_car', 'npc', 0., 4.6, 1.9, 0., 0., 0., 3, None, None, True, 0.)
        checks = {}
        for name, guard in (('tunnel', tunnel), ('road', road)):
            result = guard.check(path, ego, [('tunnel_lidar', [car], 100., .05)], 10., requested_mps=8.)
            checks[name] = result.validation['objects'][0]['gap_m']
        self.assertAlmostEqual(checks['road']-checks['tunnel'], EGO_FRONT_M-EGO_HALF_LENGTH_M, places=6)

    def test_tunnel_moving_conflict_places_people_against_the_real_bumpers(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        path = straight(30.)

        def person(f, l):
            return (f, l, .6, 'tunnel_lidar_dynamic_9', 'pedestrian', 0., .6, .6, 0., 0., 0., 9)
        # Beside the real front fender (3.0 m ahead of the rear axle, 1.5 m out): not ahead of us.
        self.assertIsNotNone(gui.moving_conflict([person(3., 1.5)], ego, path, 0.))
        self.assertIsNone(gui.moving_conflict([person(3., 1.5)], ego, path, 0., body_shift_m=REAR_AXLE_SHIFT_M))
        # 0.7 m behind the real rear bumper: behind us, not in our way.
        self.assertIsNotNone(gui.moving_conflict([person(-1.5, 0.)], ego, path, 0.))
        self.assertIsNone(gui.moving_conflict([person(-1.5, 0.)], ego, path, 0., body_shift_m=REAR_AXLE_SHIFT_M))
        # Ahead of the real front bumper the hold stays (no room for the 2.5 m standoff).
        ahead = gui.moving_conflict([person(4.5, .2)], ego, path, 0., body_shift_m=REAR_AXLE_SHIFT_M)
        self.assertEqual(ahead['v_mps'], 0.)


class LoggedWreckTests(LoggedExitFrames, unittest.TestCase):
    """global_hard_13 (follow-20261010T210040Z): parked 5.5 min beside a wreck."""

    def setUp(self):
        self.data = dict(frames=FIXTURE['runs']['hard13_wreck']['frames'])

    def plan(self, frame, previous=None, anchor=True, extra_returns=None):
        """LoggedExitFrames.plan outside the exit zone, as logged here."""
        self.assertFalse(frame['logged']['exit_zone'])
        with patch.object(tunnel_runtime.TunnelRuntime, 'tunnel_model_static',
                          lambda host, *args: original(host, *args[:-1], False)):
            return super().plan(frame, previous, anchor, extra_returns)

    def test_the_real_body_clears_the_wreck_the_checked_body_overlapped(self):
        for frame in self.data['frames']:
            with self.subTest(time=frame['time']):
                self.assertEqual(frame['logged']['clearance']['obstacle'], 0.)    # every frame
                self.assertEqual(frame['logged']['reason'], 'tunnel no safe bypass')
                ego, _, _, objects, points, returns, _ = self.inputs(frame)
                wreck = next(o for o in objects if o[3] == 'tunnel_lidar_0')
                outline = box_points(wreck[0], wreck[1], wreck[6], wreck[7], wreck[8], ego)
                self.assertGreater(body_gap(outline), 1.)                # the real rear-left corner
                self.assertLess(legacy_gap(outline), .6)                  # the checked one
                # Straight ahead from the pose: the old box (0.25 m margins) is touched at once,
                # the real body is clear for the whole 30 m.
                self.assertEqual(swept_clearance(straight(), points, 30., .25), 0.)
                self.assertEqual(swept_clearance(straight(), points, 30., .25, body_shift_m=REAR_AXLE_SHIFT_M), 30.)

    def test_the_held_frames_now_have_a_checked_bypass(self):
        real_budget = ma.Budget
        with patch.object(ma, 'Budget', lambda seconds=None, **kwargs: real_budget(5.)):   # CPU-independent
            for frame in self.data['frames'][1:]:                    # 06:08:36 / 06:11:20, the long hold
                with self.subTest(time=frame['time']):
                    result = self.plan(frame)
                    plan = result.plan
                    self.assertFalse(plan.get('blocked'), result.policy)
                    self.assertEqual(plan['kind'], 'bypass')
                    self.assertGreaterEqual(min(result.checks(plan['path'], 35.).values()), 35.)
                    self.assertEqual(swept_clearance(plan['path'], result.returns, 35., .25,
                                                     body_shift_m=REAR_AXLE_SHIFT_M), 35.)
                    # The old placement reproduces the logged stop.
                    with patch.object(ma, 'REAR_AXLE_SHIFT_M', ROAD_LEGACY_SHIFT_M), \
                            patch.object(tunnel_runtime, 'REAR_AXLE_SHIFT_M', ROAD_LEGACY_SHIFT_M):
                        self.assertTrue(self.plan(frame).plan.get('blocked'))


class LoggedBollardTests(unittest.TestCase):
    """global_hard_12 (follow-20261010T211143Z): a bollard 'read 2.0 m' from the bumper."""

    def bollard(self, frame):
        ego = frame['ego']
        rows = [row for row in frame['objects'] if max(row[4], row[5]) < .5]
        near = min(rows, key=lambda row: math.hypot(row[0]-ego['x'], row[1]-ego['y']))
        return ego, near, box_points(near[0], near[1], near[4], near[5], near[6], ego)

    def test_the_real_gap_to_the_bollard_is_detected(self):
        for frame in FIXTURE['runs']['hard12_bollard']['frames']:
            with self.subTest(time=frame['time']):
                self.assertEqual(frame['logged']['clearance']['obstacle'], 2.)     # what it believed
                self.assertGreater(frame['logged']['body_gap_m'], 2.)
                ego, row, outline = self.bollard(frame)
                # The cluster box's edge (0.1 m voxel margin included) and its measured face.
                faces = seen_faces(row[0], row[1], row[4], row[5], row[6], ego)
                for points in (outline, faces):
                    self.assertGreater(legacy_gap(points), 1.9)
                    self.assertGreater(body_gap(points), .3)
                    # Standing (06:16:49.94, 06:17:00.27): edge 0.41 m, face 0.51 m.
                    self.assertLess(body_gap(points), .55 if frame['ego']['speed_kmh'] < .5 else .65)
                    self.assertAlmostEqual(legacy_gap(points)-body_gap(points), EGO_CENTRE_M, delta=.1)
                path = straight(20.)
                self.assertEqual(swept_clearance(path, faces, 20., .25), 2.)      # the logged 2.0 m
                # With the real body the 0.25 m margin is reached within 0.5 m: the stop
                # at "2 m" comes 1.5 m earlier, at a real ~2 m gap.
                self.assertLessEqual(swept_clearance(path, faces, 20., .25, body_shift_m=REAR_AXLE_SHIFT_M), .5)
                lidar = SimpleNamespace(snapshot=lambda: (faces, .05))
                self.assertAlmostEqual(gui.body_gap_m(lidar), round(body_gap(faces), 2))


class Publisher:
    def __init__(self):
        self.reverse_state, self.reverse_abort, self.started = None, False, []

    def start_reverse(self, current, road_heading_deg=None, **_):
        self.started.append(road_heading_deg)
        self.reverse_state = dict(x=float(current['x']), y=float(current['y']),
                                  heading=float(current['heading_deg'] if road_heading_deg is None
                                                else road_heading_deg),
                                  until=time.monotonic()+14., distance=gui.REVERSE_ESCAPE_M)

    def reverse_active(self, current):
        return self.reverse_state is not None and not self.reverse_abort


class LoggedReverseSwingTests(unittest.TestCase):
    """follow-20261010T200235Z 05:09:01-05:09:06: reversing 24 deg to the road heading."""

    @classmethod
    def setUpClass(cls):
        cls.frames = FIXTURE['runs']['reverse_drum']['frames']
        waypoints = json.loads((HERE/'global_path.json').read_text())['waypoints']
        cls.route = np.array([[p['x'], p['y']] for p in waypoints])

    def scene(self, frame):
        ego = dict(frame['ego'])
        points = [box_points(r[0], r[1], r[4], r[5], r[6], ego) for r in frame['objects']
                  if max(r[4], r[5]) < 1. and math.hypot(r[0]-ego['x'], r[1]-ego['y']) < 6.]
        return ego, np.vstack(points)

    def host(self, publisher):
        return SimpleNamespace(path_publisher=publisher, waypoint_xy=self.route, object_receiver=None,
                               _reverse_wait_since=0., _contact=None)

    def start(self, frame, swing_check):
        ego, points = self.scene(frame)
        publisher = Publisher()
        host = self.host(publisher)
        host._reverse_wait_since = time.monotonic()-gui.REVERSE_AFTER_S-1.
        started = gui.Runtime.reverse_escape_check(host, dict(ego, speed_kmh=0.), True, 0., points,
                                                   time.monotonic(), swing_check=swing_check)
        return started, host, publisher

    def test_the_drum_beside_the_front_vetoes_the_turning_reverse(self):
        start = self.frames[0]
        ego, points = self.scene(start)
        self.assertAlmostEqual(start['road_heading_deg']-ego['heading_deg'], 23.8, delta=.3)
        self.assertGreater(body_gap(points), .7)                # 0.75 m beside the front-left corner
        # The rear-only check started the logged reverse; the swing check does not.
        started, _, publisher = self.start(start, swing_check=False)
        self.assertTrue(started)
        self.assertAlmostEqual(publisher.started[0], start['road_heading_deg'], delta=.3)
        started, host, publisher = self.start(start, swing_check=True)
        self.assertFalse(started)
        self.assertTrue(host._reverse_veto.startswith('reverse swing point'), host._reverse_veto)
        # Backing straight (no turn) never reaches the drum beside the front.
        self.assertFalse(reverse_sweep_hits(points, 0., gui.REVERSE_ESCAPE_M).any())

    def test_a_running_reverse_aborts_before_the_front_corner_reaches_the_drum(self):
        start, middle, contact = self.frames
        ego, points = self.scene(middle)
        self.assertGreater(body_gap(points), .3)                # 05:09:04.05: still 0.35 m
        self.assertEqual(body_gap(self.scene(contact)[1]), 0.)  # 05:09:05.71: the touch
        for swing_check in (False, True):
            publisher = Publisher()
            publisher.start_reverse(start['ego'], road_heading_deg=start['road_heading_deg'])
            host = self.host(publisher)
            self.assertTrue(gui.Runtime.reverse_escape_check(host, ego, True, 0., points, time.monotonic(),
                                                             swing_check=swing_check))
            self.assertEqual(publisher.reverse_abort, swing_check)
        self.assertTrue(host._reverse_veto.startswith('reverse swing abort'))


if __name__ == '__main__':
    unittest.main()
