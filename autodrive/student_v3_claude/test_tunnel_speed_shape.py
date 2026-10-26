"""Tunnel speed and bypass shape (follow-20261010T151125Z, 17.8 / 24.0 s tunnel laps).

Replays logged tunnel frames (test_data/tunnel_shape_20261010T151125Z.json: the
wall-matched ego and the LiDAR objects the planner used, frame by frame, open loop)
through the model-policy static slot, plus the speed rules around it. No ROS,
simulator or GPU; raw returns are replaced by the logged object outlines.
"""
import json
import math
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

import gui
import model_avoidance as ma
import tunnel_runtime
from driving_plan import DrivingPlan
from tunnel_map import TunnelMap
from tunnel_static_portable import TunnelLaneHold
from tunnel_runtime import (TunnelRuntime, curve_speed_cap, rotation, tunnel_conflict_objects,
                            yield_envelope_mps)

HERE = Path(__file__).resolve().parent
FIXTURE = HERE/'test_data'/'tunnel_shape_20261010T151125Z.json'


def outline(x, y, length, width, heading_deg, step=.2):
    yaw = math.radians(heading_deg)
    axes = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    corners = np.array([(-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1)]) * [.5*length-.1, .5*width-.1]
    points = []
    for a, b in zip(corners[:-1], corners[1:]):
        n = max(2, int(np.linalg.norm(b-a)/step)+1)
        points.extend(a+np.linspace(0., 1., n)[:, None]*(b-a))
    return np.asarray(points) @ axes.T + [x, y]


def resampled(path):
    path = gui.RosPathPublisher.resample(np.asarray(path, dtype=float)[:, :2])
    if np.linalg.norm(path[0]) > .01:
        path = gui.RosPathPublisher.resample(np.vstack(([0., 0.], path)))
    return path


class HostCurvature:
    """The E2E launch configures the package with the host's limits (gui MODEL_AVOID)."""
    @classmethod
    def setUpClass(cls):
        cls._saved = (ma._static.MAX_CONTROL_CURVATURE, ma._geometry.MAX_CONTROL_CURVATURE,
                      ma._static.static_clear_distance)
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    @classmethod
    def tearDownClass(cls):
        (ma._static.MAX_CONTROL_CURVATURE, ma._geometry.MAX_CONTROL_CURVATURE,
         ma._static.static_clear_distance) = cls._saved


class LoggedFrames(HostCurvature):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.data = json.loads(FIXTURE.read_text())
        waypoints = json.loads((HERE/'global_path.json').read_text())['waypoints']
        cls.world = np.array([[p['x'], p['y']] for p in waypoints])
        cls.control = gui.smooth_control_route(cls.world)
        cls.tunnel = TunnelMap.load(HERE/'tunnel_wall_map.json')
        cls.scope, _ = ma.make_tunnel_policies(cls.world, ma.tunnel_route_bounds(cls.world, cls.tunnel.active))

    def frame(self, window, time_prefix):
        return next(f for f in self.data['windows'][window] if f['time'].startswith(time_prefix))

    def inputs(self, frame):
        ego = dict(frame['ego'])
        nearest = int(np.argmin(np.linalg.norm(self.world-[ego['x'], ego['y']], axis=1)))
        dense = (self.control[[(nearest+i) % len(self.control) for i in range(240)]]
                 - [ego['x'], ego['y']]) @ rotation(ego)
        objects, points = [], []
        for x, y, name, kind, length, width, heading, speed, vx, vy, uid, count in frame['objects']:
            objects.append((x, y, max(length, width), name, kind, speed, length, width, heading, vx, vy, uid,
                            {'point_count': count or 0}, None, name == 'lidar_car', 0.))
            points.append((outline(x, y, length, width, heading)-[ego['x'], ego['y']]) @ rotation(ego))
        return ego, nearest, dense, objects, np.vstack(points)

    def plan(self, frame, shape=True, previous=None):
        ego, nearest, dense, objects, points = self.inputs(frame)
        host = SimpleNamespace(tunnel=self.tunnel, tunnel_route_policy=self.scope,
                               _tunnel_lane_hold=TunnelLaneHold(), v6_latest=None, v6_worker=None)
        corridor, tree = self.tunnel.corridor(ego), cKDTree(points)

        def clear(path, horizon=60.):
            return min(TunnelRuntime.tunnel_path_checks(host, np.asarray(path)[:, :2], ego, points, corridor,
                                                        horizon, tree=tree).values())

        def lane_ok(path, tolerance=1.5):
            local = np.asarray(path)[:, :2]
            length = float(np.linalg.norm(np.diff(local, axis=0), axis=1).sum())
            return clear(local, length) >= length-.3
        with patch.object(tunnel_runtime, 'TUNNEL_SHAPE', shape):
            plan, hazards, *_ = TunnelRuntime.tunnel_model_static(
                host, dense, ego, objects, lane_ok, clear, self.tunnel.adjacent_offsets(ego, dense),
                previous, nearest, False, False)
        return plan, hazards, ego, objects, clear, host._tunnel_model_log

    def curve_kmh(self, plan, ego):
        return curve_speed_cap(resampled(plan['path']), ego['speed_kmh']/3.6)[0]*3.6

    @staticmethod
    def former_curve_kmh(plan):
        # Former rule: the whole path's peak curvature at 3.0 m/s^2 (STUDENT_BYPASS_ALAT of
        # the E2E launch, through static_speed_cap_mps) and at TUNNEL_BYPASS_ALAT_MPS2.
        peak = float(np.max(gui.path_curvatures(resampled(plan['path']))))
        return math.sqrt(min(3., tunnel_runtime.TUNNEL_BYPASS_ALAT_MPS2)/max(peak, 1e-3))*3.6


class LoggedShapeTests(LoggedFrames, unittest.TestCase):
    def test_entry_bike_pass_keeps_a_gentle_bypass(self):
        # 00:16:35.0: a 14 m transition (0.099 1/m) held the planner at 20 km/h for 0.4 s.
        # With the real body on the rear-axle pose (ego_body) the bike stands 1.6 m, not
        # 3.2 m, ahead of the front bumper, and the planner's own pass is already an 18 m
        # transition (0.045 1/m, 33 km/h). Longer transitions exceed the heading-swing
        # limit (25.8 m here); the 22 m ones are 2.5 m less clear than that pass, so the
        # shape keeps it. (The 45 km/h reshape found before relied on the shorter body.)
        frame = self.frame('lap1_entry', '00:16:35.028')
        self.assertLess(frame['logged']['planner_cap_kmh'], 21.)
        self.assertGreater(frame['logged']['curvature'], .09)
        old, _, ego, _, _, _ = self.plan(frame, shape=False)
        new, hazards, ego, objects, clear, log = self.plan(frame)
        self.assertLess(self.former_curve_kmh(old), 40.)
        self.assertEqual(new['direction'], old['direction'])
        self.assertLess(float(new['curvature']), .05)
        self.assertGreater(self.curve_kmh(new, ego), 30.)
        self.assertGreaterEqual(clear(new['path'], 60.), new['hold_until_m'])

    def test_parked_car_stretch_is_passed_on_one_line(self):
        # Lap 2, car (UID 31) 37 m ahead, drum behind it, the second car on the other side:
        # the 2.8 m lane offset had to come back in 14 m beside the drum (22 km/h).
        frame = self.frame('lap2_car31_exit', '00:21:49.246')
        new, hazards, ego, objects, clear, log = self.plan(frame)
        self.assertTrue(new.get('reshaped'), log)
        self.assertLessEqual(new['offset_m'], 1.6)
        self.assertEqual(new['transition_m'], 40.)
        self.assertGreater(self.curve_kmh(new, ego), 55.)

    def test_every_reshaped_frame_is_faster_and_passes_the_planner_checks(self):
        counts = dict(frames=0, reshaped=0, slow_old=0, slow_new=0)
        for window in self.data['windows']:
            for frame in self.data['windows'][window]:
                old, *_ = self.plan(frame, shape=False)
                new, hazards, ego, objects, clear, log = self.plan(frame)
                if old is None or old.get('blocked'):
                    continue
                counts['frames'] += 1
                old_kmh, new_kmh = self.curve_kmh(old, ego), self.curve_kmh(new, ego)
                counts['slow_old'] += self.former_curve_kmh(old) < 40.
                counts['slow_new'] += new_kmh < 40.
                shape = (log or {}).get('shape') or {}
                if not shape.get('applied'):
                    continue
                counts['reshaped'] += 1
                with self.subTest(frame=frame['time']):
                    self.assertEqual(new['generator'], 'portable')
                    if not shape.get('committed'):
                        self.assertGreaterEqual(new_kmh, old_kmh-.3*3.6-1e-6)
                    if old['kind'] == 'bypass':
                        self.assertEqual(new['direction'], old['direction'])
                    if new['offset_m'] > 0.:
                        static = [o for o in objects if o[4] != 'pedestrian']
                        horizon = min(60., float(np.linalg.norm(np.diff(np.asarray(new['path'])[:, :2],
                                                                        axis=0), axis=1).sum()))
                        needed = max(new['safe_horizon_m'], new['hold_until_m'])
                        self.assertGreaterEqual(gui.static_clear_distance(
                            new['path'], ego, ma._comfort_boxes([ma.as_static(o) for o in static]), horizon), needed)
                        self.assertGreaterEqual(clear(new['path'], horizon), needed)
        self.assertGreater(counts['reshaped'], counts['frames']//2, counts)
        # Below the 40 km/h former ceiling: the planner's shape under the former peak rule
        # against the reshaped path under the braking-aware rule.
        self.assertGreater(counts['slow_old'], counts['frames']//2, counts)
        self.assertLess(counts['slow_new'], counts['slow_old']//5, counts)

    def test_false_parked_car_mover_is_no_moving_conflict(self):
        # 00:16:40.503: tunnel_lidar_dynamic_800013 (4.33 x 1.81 m, 0.80 m/s) on the parked
        # car 4.7 m right of the bypass read as a pedestrian disc 2.2 m wide: "stop".
        frame = self.frame('lap1_car31', '00:16:40.503')
        ego, _, _, objects, _ = self.inputs(frame)
        mover = [o for o in objects if o[3] == 'tunnel_lidar_dynamic_800013']
        route = np.column_stack((np.arange(0., 60., .5), np.full(120, 0.)))
        self.assertEqual(gui.moving_conflict(mover, ego, route, ego['speed_kmh']/3.6)['kind'], 'stop')
        self.assertIsNone(gui.moving_conflict(tunnel_conflict_objects(mover), ego, route, ego['speed_kmh']/3.6))


def straight(length=100., step=.5):
    xs = np.arange(0., length+step, step)
    return np.column_stack((xs, np.zeros_like(xs)))


def box(x, y, length=1., width=1., name='tunnel_lidar_1'):
    return (x, y, max(length, width), name, 'obstacle', 0., length, width, 0., 0., 0., 1)


class ShapeRuleTests(HostCurvature, unittest.TestCase):
    ego = dict(x=0., y=0., heading_deg=0., speed_kmh=40.)

    def checks(self, objects, wall_m=6.):
        points = np.vstack([outline(*o[:2], o[6], o[7], o[8]) for o in objects]) if objects else np.empty((0, 2))

        def clear(path, horizon=60.):
            path = np.asarray(path)[:, :2]
            distance = tunnel_runtime.swept_clearance(path, points, horizon) if len(points) else horizon
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
            outside = np.flatnonzero(np.abs(path[:, 1]) > wall_m-1.2)
            return min(distance, float(arc[outside[0]]) if len(outside) else min(horizon, float(arc[-1])))

        def lane_ok(path, tolerance=1.5):
            path = np.asarray(path)[:, :2]
            length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
            return clear(path, length) >= length-.3
        return clear, lane_ok

    def planner(self, objects, previous=None, direction=0.):
        clear, lane_ok = self.checks(objects)
        plan, hazards, _ = ma.plan_bypass(straight(), self.ego, objects, lane_ok, clear, direction, previous,
                                          ma.tunnel_offsets({1: 4.5, -1: 4.5}))
        return plan, hazards, clear, lane_ok

    def speed(self, plan):
        return curve_speed_cap(resampled(plan['path']), self.ego['speed_kmh']/3.6)[0]

    def test_bypass_is_reshaped_faster_on_the_same_side(self):
        objects = [box(30., -.3, .6, .6)]
        plan, hazards, clear, lane_ok = self.planner(objects, direction=1.)
        shaped, info = ma.smooth_tunnel_plan(plan, hazards, straight(), self.ego, objects, lane_ok, clear)
        self.assertTrue(info['applied'], info)
        self.assertEqual(shaped['direction'], plan['direction'])
        self.assertGreaterEqual(self.speed(shaped), self.speed(plan)+.3)
        self.assertGreaterEqual(clear(shaped['path'], 60.), shaped['hold_until_m'])

    def test_comfort_margin_from_static_boxes(self):
        # 0.9 m would pass 0.3 m from the box (the planner's own check keeps 0.2 m).
        objects = [box(30., -.8, 1., 1.)]
        plan, hazards, clear, lane_ok = self.planner(objects, direction=1.)
        shaped, info = ma.smooth_tunnel_plan(plan, hazards, straight(), self.ego, objects, lane_ok, clear)
        self.assertTrue(info['applied'], info)
        # Left edge of the box at -0.3 m: 0.8 m of room needs an offset of 1.45 m or more
        # (body half width 0.946 m); 1.2 m leaves 0.55 m.
        self.assertGreaterEqual(shaped['offset_m'], 1.45)
        self.assertGreaterEqual(gui.static_clear_distance(shaped['path'], self.ego, ma._comfort_boxes(objects), 60.),
                                shaped['hold_until_m'])
        # 0.9 m passes the planner's own 0.2 m box margin, not the comfort margin.
        thin = ma._geometry.build_forced_bypass(straight(), 1., offset_m=.9, hold_m=30., transitions=(14.,),
                                                max_curvature_1pm=gui.MAX_CONTROL_CURVATURE)[0]
        self.assertGreaterEqual(gui.static_clear_distance(thin, self.ego, objects, 60.), 40.)
        self.assertLess(gui.static_clear_distance(thin, self.ego, ma._comfort_boxes(objects), 60.), 40.)

    def test_kept_offset_is_not_traded_for_a_marginally_faster_one(self):
        objects = [box(30., 0., 1.2, 1.2)]
        previous = dict(kind='bypass', direction=1., offset_m=2.2, generator='portable', path=straight())
        plan, hazards, clear, lane_ok = self.planner(objects, direction=1.)
        with patch.object(ma, 'TUNNEL_STICKY_MPS', 50.):
            shaped, info = ma.smooth_tunnel_plan(plan, hazards, straight(), self.ego, objects, lane_ok, clear,
                                                 previous)
        self.assertAlmostEqual(shaped['offset_m'], 2.2)

    def rejoin(self, objects):
        clear, lane_ok = self.checks(objects)
        path, peak, _ = ma._geometry.build_forced_bypass(straight(), 0., offset_m=0., hold_m=0.,
                                                         max_curvature_1pm=gui.MAX_CONTROL_CURVATURE,
                                                         transitions=(26.,))
        plan = dict(path=path, kind='rejoin', direction=1., offset_m=0., transition_m=26., hold_m=0.,
                    curvature=peak, clear_distance_m=35., safe_horizon_m=10., generator='portable', reused=False)
        previous = dict(kind='bypass', direction=1., offset_m=1.6, generator='portable', path=straight())
        return plan, previous, clear, lane_ok

    def test_return_is_not_started_while_the_route_ahead_is_obstructed(self):
        objects = [box(40., -.5, 1.2, 1.2)]
        plan, previous, clear, lane_ok = self.rejoin(objects)
        shaped, info = ma.smooth_tunnel_plan(plan, ma.footprint_hazards(straight(), self.ego, objects),
                                             straight(), self.ego, objects, lane_ok, clear, previous)
        self.assertTrue(info.get('committed'), info)
        self.assertEqual(shaped['kind'], 'bypass')
        self.assertEqual(shaped['direction'], 1.)
        self.assertGreaterEqual(clear(shaped['path'], 60.), shaped['hold_until_m'])

    def test_return_runs_once_the_route_ahead_is_clear(self):
        for objects in ([], [box(70., 0., 1.2, 1.2)]):   # nothing, or beyond the commitment
            with self.subTest(objects=len(objects)):
                plan, previous, clear, lane_ok = self.rejoin(objects)
                shaped, info = ma.smooth_tunnel_plan(plan, [], straight(), self.ego, objects, lane_ok, clear,
                                                     previous)
                self.assertEqual(shaped['kind'], 'rejoin')
                self.assertEqual(shaped['offset_m'], 0.)
                self.assertFalse(info.get('committed'))

    def test_saved_bypass_relabelled_as_return_is_left_alone(self):
        objects = [box(40., -.5, 1.2, 1.2)]
        plan, previous, clear, lane_ok = self.rejoin(objects)
        reused = dict(plan, offset_m=1.6, reused=True)
        self.assertIs(ma.smooth_tunnel_plan(reused, [], straight(), self.ego, objects, lane_ok, clear,
                                            previous)[0], reused)

    def test_blocked_and_held_plans_pass_through(self):
        for plan in (None, dict(blocked=True, kind='bypass'),
                     dict(kind='tunnel_hold', path=straight(), generator='portable', direction=1., offset_m=2.2)):
            self.assertIs(ma.smooth_tunnel_plan(plan, [], straight(), self.ego, [], lambda *a: True,
                                                lambda p, h=60.: h)[0], plan)


class CurveSpeedTests(unittest.TestCase):
    def s_curve(self, start_m, offset=3.5, transition=14.):
        xs = np.arange(0., 80., .5)
        u = np.clip((xs-start_m)/transition, 0., 1.)
        return np.column_stack((xs, offset*u**3*(10.-15.*u+6.*u**2)))

    def test_far_return_curve_no_longer_caps_the_present_speed(self):
        path = self.s_curve(35.)
        peak = float(np.max(gui.path_curvatures(path)))
        before = math.sqrt(tunnel_runtime.TUNNEL_BYPASS_ALAT_MPS2/peak)
        cap, station = curve_speed_cap(path, 10.)
        self.assertLess(before, 6.5)                      # ~22 km/h for the whole approach
        self.assertGreater(cap, 11.)
        self.assertGreater(station, 35.)
        # Braking at TUNNEL_CURVE_DECEL_MPS2 from the cap reaches each curvature's own limit.
        brake = 2.*tunnel_runtime.TUNNEL_CURVE_DECEL_MPS2
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        s_peak = float(arc[1+int(np.argmax(gui.path_curvatures(path)))])
        self.assertGreaterEqual(cap**2, before**2+brake*(station-4.)-1e-6)
        self.assertLessEqual(cap**2, before**2+brake*(s_peak-4.)+1e-6)

    def test_curve_at_the_car_keeps_its_plain_limit(self):
        path = self.s_curve(0.)
        cap, _ = curve_speed_cap(path, 10.)
        peak = float(np.max(gui.path_curvatures(path)))
        self.assertAlmostEqual(cap, math.sqrt(tunnel_runtime.TUNNEL_BYPASS_ALAT_MPS2/peak), places=6)
        self.assertTrue(math.isinf(curve_speed_cap(np.zeros((2, 2)))[0]))
        self.assertGreater(curve_speed_cap(straight())[0], 100.)


class TunnelZoneTests(unittest.TestCase):
    def test_tunnel_span_gets_the_tunnel_cap_and_the_portal_keeps_its_own(self):
        launcher = [(480, 770, 40.), (770, 1100, 30.), (3700, 4030, 40.)]
        self.assertEqual(gui.with_tunnel_zone(launcher, (3777, 4030), 55.),
                         [(480, 770, 40.), (770, 1100, 30.), (3700, 3776, 40.), (3777, 4030, 55.)])
        self.assertEqual(gui.with_tunnel_zone([(3800, 4100, 30.)], (3777, 4030), 55.),
                         [(3800, 4030, 55.), (4031, 4100, 30.)])
        self.assertLessEqual(gui.TUNNEL_ZONE_KMH, 58.)
        self.assertEqual(gui.TUNNEL_ZONE_IDX, (3777, 4030))

    def test_planner_zone_inside_the_tunnel(self):
        zones = gui.SpeedZones.__new__(gui.SpeedZones)
        zones.base, zones.route_len = 55., 4200
        zones.zones, zones.loaded = [], time.monotonic()
        zones.file_cache = SimpleNamespace(value=[])
        zones.set_route(np.column_stack((np.arange(4200)*.5, np.zeros(4200))))
        dynamic = gui.with_tunnel_zone([(3700, 4030, 40.)], (3777, 4030), 55.)
        with patch.object(gui, 'DYNAMIC_ZONES', dynamic):
            self.assertEqual(gui.zone_speed_kmh(zones, 3800)[0], 55.)
            self.assertEqual(gui.zone_speed_kmh(zones, 4020)[0], 55.)
            self.assertEqual(gui.zone_speed_kmh(zones, 3740)[0], 40.)


class MovingConflictTests(unittest.TestCase):
    def decision(self, speed_kmh=40.):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=speed_kmh, stamp=10.)
        return DrivingPlan(1, straight(60.), ego, ego, 10., .05, time.monotonic(), 15., 60., False,
                           dict(valid=True, pose_valid=True), 'TUNNEL BYPASS')

    def traffic(self, conflict, speed_kmh=40.):
        host = SimpleNamespace(sim_clock=None)
        with patch('gui.moving_conflict', return_value=conflict):
            return TunnelRuntime.tunnel_traffic_checks(host, self.decision(speed_kmh), [], None, [])

    def test_conflict_beyond_the_stopping_distance_is_approached_braking(self):
        decision, _, yielding = self.traffic(dict(kind='stop', name='tunnel_lidar_dynamic_1', along=45., t=2.,
                                                  v_mps=1.2), speed_kmh=36.)
        self.assertFalse(decision.emergency)
        self.assertEqual(yielding['kind'], 'brake')
        self.assertAlmostEqual(decision.allowed_mps, yield_envelope_mps(45.))
        self.assertLess(decision.allowed_mps, 15.)
        self.assertIsNone(decision.stop_reason)

    def test_conflict_inside_the_stopping_distance_still_stops(self):
        decision, _, yielding = self.traffic(dict(kind='stop', name='tunnel_lidar_dynamic_1', along=12., t=1.,
                                                  v_mps=1.2), speed_kmh=36.)
        self.assertTrue(decision.emergency)
        self.assertEqual(yielding['kind'], 'stop')
        self.assertEqual(decision.stop_reason, 'tunnel moving object conflict')

    def test_envelope_stops_short_of_the_conflict(self):
        # The standoff is the real front bumper (3.845 m ahead of the rear-axle pose) + 2.5 m.
        self.assertAlmostEqual(tunnel_runtime.TUNNEL_YIELD_STANDOFF_M, 3.845+2.5)
        for along in (7., 15., 30., 60.):
            v = yield_envelope_mps(along)
            a, t = tunnel_runtime.TUNNEL_YIELD_BRAKE_MPS2, tunnel_runtime.TUNNEL_YIELD_REACTION_S
            self.assertAlmostEqual(v*t+v*v/(2.*a), along-tunnel_runtime.TUNNEL_YIELD_STANDOFF_M, places=6)
        self.assertEqual(yield_envelope_mps(3.), 0.)
        self.assertEqual(yield_envelope_mps(6.), 0.)

    def test_car_length_mover_is_a_vehicle_and_a_person_stays_a_person(self):
        car = ('tunnel_lidar_dynamic_7', 'pedestrian', 4.6, 1.9)
        rows = tunnel_conflict_objects([(10., 3.5, 4.6, car[0], car[1], 5., 4.6, 1.9, 0., 0., 5., 7),
                                        (10., 3.5, .7, 'tunnel_lidar_dynamic_8', 'pedestrian', 1.2, .7, .7, 0.,
                                         0., 1.2, 8)])
        self.assertEqual(rows[0][4], 'npc')
        self.assertAlmostEqual(rows[0][8], 90.)
        self.assertEqual(rows[1][4], 'pedestrian')
        route = straight(60.)
        ego = dict(x=0., y=0., heading_deg=0.)
        # Driving along the next lane at 5 m/s: no conflict for our lane.
        along_lane = [(15., 3.5, 4.6, car[0], car[1], 5., 4.6, 1.9, 0., 5., 0., 7)]
        self.assertIsNone(gui.moving_conflict(tunnel_conflict_objects(along_lane), ego, route, 10.))
        self.assertEqual(gui.moving_conflict(along_lane, ego, route, 10.)['kind'], 'stop')


class BridgeTests(HostCurvature, unittest.TestCase):
    """follow-20261010T162527Z: single "tunnel no safe bypass" frames at 39-45 km/h."""
    ego = dict(x=0., y=0., heading_deg=0., speed_kmh=45.)

    def previous(self, offset=1.2):
        path, peak, _ = ma._geometry.build_forced_bypass(straight(), 1., offset_m=offset, hold_m=20.,
                                                         max_curvature_1pm=gui.MAX_CONTROL_CURVATURE,
                                                         transitions=(40.,))
        return dict(path=path, kind='bypass', direction=1., offset_m=offset, transition_m=40., curvature=peak,
                    generator='portable', reused=True)

    def test_committed_path_still_clear_is_kept_instead_of_no_safe_bypass(self):
        objects = [box(20., -2.2, 4.9, 1.9, 'lidar_car')]          # the parked car being passed
        clear = lambda path, horizon=60.: horizon
        for blocked in (dict(blocked=True, kind='bypass'), None):
            plan, info = ma.bridge_tunnel_plan(blocked, self.previous(), self.ego, objects, clear)
            self.assertTrue(info['applied'], info)
            self.assertFalse(plan['blocked'])
            self.assertTrue(plan['bridged'])
            self.assertEqual(plan['offset_m'], 1.2)

    def test_obstructed_or_endless_bridges_keep_the_planner_result(self):
        blocked = dict(blocked=True, kind='bypass')
        objects = [box(15., 1.2, 1.2, 1.2)]                        # on the committed path
        plan, info = ma.bridge_tunnel_plan(blocked, self.previous(), self.ego, objects, lambda p, h=60.: h)
        self.assertIs(plan, blocked)
        self.assertFalse(info['applied'])
        plan, info = ma.bridge_tunnel_plan(blocked, self.previous(), self.ego, [], lambda p, h=60.: 12.)
        self.assertIs(plan, blocked)
        plan, _ = ma.bridge_tunnel_plan(blocked, self.previous(), self.ego, [], lambda p, h=60.: h,
                                        bridged_frames=ma.TUNNEL_BRIDGE_MAX_FRAMES)
        self.assertIs(plan, blocked)
        good = dict(self.previous(), reused=False)
        self.assertIs(ma.bridge_tunnel_plan(good, self.previous(), self.ego, [], lambda p, h=60.: h)[0], good)

    def test_tunnel_requirement_uses_the_tunnel_braking(self):
        v = 45./3.6
        self.assertGreater(v*.7+v*v/5.+2., 41.)                    # the package's 2.5 m/s^2
        self.assertLess(ma.tunnel_required_m(v), 31.)
        self.assertGreaterEqual(ma.tunnel_required_m(0.), 8.)


class TunnelPublishRampTests(unittest.TestCase):
    def setUp(self):
        import test_avoidance_gate as avoidance_fixture
        runtime = avoidance_fixture.AvoidanceGateTests().runtime()
        self.publisher, self.route = runtime.path_publisher, runtime.test_route
        self.ego = dict(runtime.test_status['ego'])

    def publish(self, clock, measured_kmh, cap_kmh, mode):
        self.ego['speed_kmh'] = measured_kmh
        self.publisher.zone_mps, self.publisher.emergency_stop = cap_kmh/3.6, False
        with patch('gui.time.monotonic', return_value=clock):
            self.publisher.publish(self.ego, self.route, self.route, dict(mode=mode, reason='test', validated_static=True))
        return self.publisher.velocity_pub.items[-1].data*3.6

    def test_one_frame_tunnel_dip_recovers_from_the_measured_speed(self):
        # 01:30:25.0: cap 8 km/h for one frame at 39 km/h, then 52 km/h allowed.
        self.publish(100., 38.2, 38.9, 'TUNNEL BYPASS')
        self.assertAlmostEqual(self.publish(100.1, 38.2, 8., 'TUNNEL'), 8.)
        recovered = self.publish(100.2, 38.9, 51.9, 'TUNNEL BYPASS')
        self.assertGreaterEqual(recovered, 38.9)
        self.assertLessEqual(recovered, 38.9+gui.LAUNCH_ACCEL_MPS2*.1*3.6+1e-9)

    def test_other_modes_keep_the_ramp_from_the_dip(self):
        self.publish(100., 38.2, 38.9, 'LATTICE')
        self.publish(100.1, 38.2, 8., 'LATTICE')
        self.assertAlmostEqual(self.publish(100.2, 38.9, 51.9, 'LATTICE'), 8.+gui.LAUNCH_ACCEL_MPS2*.1*3.6)


class ExitResidueTests(unittest.TestCase):
    """01:35:29.8-37.2: a 5 x 0.6 m band 2 m past the surveyed exit held the car 10 s."""

    def setUp(self):
        from test_tunnel_background import TunnelBackgroundTests
        fixture = TunnelBackgroundTests()
        fixture.setUp()
        self.fixture = fixture
        self.ego = dict(x=96., y=0., heading_deg=0.)            # 4 m before the surveyed end (100 m)

    def obstacles(self, extra):
        model = self.fixture.model()
        x, z = np.meshgrid(np.arange(-40., 4.01, .25), [.2, .8, 1.4, 2.0])   # walls up to the exit
        left = np.column_stack((x.ravel(), np.full(x.size, 4.), z.ravel()))
        right = left.copy(); right[:, 1] = -4.
        result = model.process(np.vstack((left, right, extra)), self.ego, [0., 0., -.4])
        self.assertTrue(result['valid'], result['reason'])
        return result['obstacle_points']

    def test_low_road_band_past_the_exit_is_road_and_a_real_object_stays(self):
        band = np.column_stack((np.full(26, 6.), np.linspace(-2.5, 2.5, 26), np.full(26, -.2)))  # 0.2 m high
        self.assertEqual(len(self.obstacles(band)), 0)
        drum = np.array([[8.1, 1., -.2], [8.3, 1., .2], [8.5, 1.2, .4]])   # 0.2 (its foot), 0.6, 0.8 m high
        kept = self.obstacles(np.vstack((band, drum)))
        self.assertEqual(len(kept), len(drum))
        tall_band = band.copy(); tall_band[:, 2] = .1                   # 0.5 m high: an object
        self.assertGreater(len(self.obstacles(tall_band)), 20)


class KerbSwingTests(LoggedFrames, unittest.TestCase):
    """follow-20261010T173230Z lap 2: a 40 m join kept a 7.6 deg heading, swung the car 5.9 m
    left of the route (plan 3.5 m), 0.3 m from the kerb; coverage read 1 m, 25 s crawl."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.kerb = json.loads((HERE/'test_data'/'tunnel_kerb_20261010T173230Z.json').read_text())

    def kerb_frame(self, prefix):
        return next(f for f in self.kerb['windows']['lap2_kerb'] if f['time'].startswith(prefix))

    def lateral(self, frame):
        return float((np.array([frame['ego']['x'], frame['ego']['y']])-self.tunnel.origin) @ self.tunnel.normal)

    def max_lateral(self, plan, ego, ahead_m=40.):
        path = np.asarray(plan['path'], dtype=float)[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        world = path[arc <= ahead_m] @ rotation(ego).T + [ego['x'], ego['y']]
        return float(np.max((world-self.tunnel.origin) @ self.tunnel.normal))

    def test_logged_drive_ended_against_the_kerb(self):
        frames = self.kerb['windows']['lap2_kerb']
        self.assertGreater(max(self.lateral(f) for f in frames), self.tunnel.inner_left-.946-.6)
        self.assertLess(min(f['logged']['observed_m'] for f in frames[-20:]), 2.)

    def test_heading_error_limits_the_join_transition(self):
        frame = self.kerb_frame('02:41:43.58')
        _, _, dense, _, _ = self.inputs(frame)
        limit = ma.heading_swing_limit_m(dense)
        self.assertLess(limit, 28.)
        self.assertGreater(limit, 14.)
        self.assertTrue(math.isinf(ma.heading_swing_limit_m(straight())))

    def test_reshaped_lines_keep_the_coverage_margin_from_the_walls(self):
        for frame in self.kerb['windows']['lap2_kerb'][:12]:       # before the car reached the kerb
            new, hazards, ego, objects, clear, log = self.plan(frame)
            shape = (log or {}).get('shape') or {}
            if not shape.get('applied'):
                continue
            with self.subTest(frame=frame['time']):
                self.assertLessEqual(new['transition_m'], max(28., ma.heading_swing_limit_m(self.inputs(frame)[2])))
                self.assertLessEqual(self.max_lateral(new, ego), self.tunnel.inner_left-.946-ma.TUNNEL_SHAPE_WALL_MARGIN_M+.05)


class CoverageEscapeTests(HostCurvature, unittest.TestCase):
    def test_line_beside_the_kerb_is_left_for_a_line_the_floor_coverage_confirms(self):
        dense = straight()
        path, peak, _ = ma._geometry.build_forced_bypass(dense, 1., offset_m=3.5, hold_m=30.,
                                                         max_curvature_1pm=gui.MAX_CONTROL_CURVATURE,
                                                         transitions=(28.,))
        plan = dict(path=path, kind='bypass', direction=1., offset_m=3.5, transition_m=28., generator='portable')
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=10.)

        def observed_of(candidate):   # the kerb cuts the rays while the line stays above 2.5 m
            candidate = np.asarray(candidate)
            return 1. if np.any(candidate[candidate[:, 0] < 10., 1] > 2.5) else 30.
        host = SimpleNamespace()
        escaped, info = TunnelRuntime.tunnel_coverage_escape(
            host, plan, [], dense, ego, [], lambda p, h=60.: h, observed_of, 1.)
        self.assertTrue(info['applied'], info)
        self.assertLess(escaped['offset_m'], 3.5)
        self.assertEqual(escaped['direction'], 1.)
        self.assertTrue(escaped['coverage_escape'])
        self.assertGreaterEqual(info['best_observed_m'], 6.)
        # No better line: keep the plan (the coverage cap still brakes for it).
        none, info = TunnelRuntime.tunnel_coverage_escape(
            host, plan, [], dense, ego, [], lambda p, h=60.: h, lambda c: 1., 1.)
        self.assertIsNone(none)
        self.assertFalse(info['applied'])


# The logged line ran 0.3 m from the kerb. (With the body still centred on the
# rear-axle pose, 0.6 m already read short: the hood's 1.5 m, then counted as
# unseen road ahead of the body, needed steep rays to the kerb; ego_body.)
KERB_Y = .946+.3


class KerbCoverageTests(unittest.TestCase):
    """02:41:45: rays to the kerb ended inside the footprint margin: observed 1 m."""

    def coverage(self):
        from tunnel_visibility import ObservedLidarCoverage
        # Vehicle at the origin, its left body side 0.3 m from the kerb face (y = 1.246 m):
        # every 0.2 deg ray returns the floor up to the kerb (or 30 m) and the kerb face.
        floor, kerb = [], []
        for azimuth in np.radians(np.arange(-180., 180., .2)):
            d = np.array([math.cos(azimuth), math.sin(azimuth)])
            reach = (KERB_Y/d[1]) if d[1] > 1e-6 else 30.
            for r in np.arange(1.5, min(reach, 30.), 1.):
                floor.append((1.05+r*d[0], r*d[1], 0.))
            if reach < 40.:
                kerb += [(1.05+reach*d[0], KERB_Y, z) for z in (.1, .3)]
        self.points = floor+kerb
        return ObservedLidarCoverage(np.asarray(floor+kerb), np.zeros(3))

    def test_line_beside_the_kerb_is_confirmed_up_to_the_kerb_face(self):
        coverage = self.coverage()
        line = np.column_stack((np.arange(0., 40., .5), np.zeros(80)))
        short = coverage.clear_distance(line, 60., exclude_initial_footprint=True)
        room = lambda centres, normals: (KERB_Y-centres[:, 1], np.full(len(centres), np.inf))
        bounded = coverage.clear_distance(line, 60., exclude_initial_footprint=True, lateral_room=room)
        self.assertLess(short, 5.)
        self.assertGreater(bounded, 20.)
        # Closer than the rays' range margin (0.5 m) the bound never reaches into the body:
        # a kerb at the body side stays unconfirmed.
        from tunnel_visibility import ObservedLidarCoverage
        close = np.column_stack((np.arange(0., 40., .5), np.zeros(80)))
        shifted = ObservedLidarCoverage(np.asarray(self.points)-[0., .3, 0.], np.zeros(3))
        self.assertLess(shifted.clear_distance(close, 60., exclude_initial_footprint=True,
                                               lateral_room=lambda c, n: (KERB_Y-.3-c[:, 1], np.full(len(c), np.inf))),
                        5.)

    def test_kerb_room_follows_the_tunnel_frame(self):
        tunnel = SimpleNamespace(origin=np.zeros(2), axis=np.array([1., 0.]), normal=np.array([0., 1.]),
                                 left=4., right=-4., start=0., end=100., ledge={1: .5, -1: .5})
        room = tunnel_runtime.kerb_room(tunnel, dict(x=10., y=1., heading_deg=0.))
        left, right = room(np.array([[0., 0.], [95., 0.]]), np.array([[0., 1.], [0., 1.]]))
        self.assertAlmostEqual(left[0], 2.5)
        self.assertAlmostEqual(right[0], 4.5)
        self.assertTrue(np.isinf(left[1]))                         # past the mapped end
        reverse = tunnel_runtime.kerb_room(tunnel, dict(x=10., y=1., heading_deg=180.))
        left, right = reverse(np.array([[0., 0.]]), np.array([[0., 1.]]))
        self.assertAlmostEqual(left[0], 4.5)                       # our left is the tunnel's right
        self.assertIsNone(tunnel_runtime.kerb_room(SimpleNamespace(), dict(x=0., y=0., heading_deg=0.)))


if __name__ == '__main__':
    unittest.main()
