"""Measured initial steering repairs a recorded axle-origin connection kink."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import gui
from lane_corridor import LaneCorridor
from lattice_planner import _arc, _frame, offset_profile
from static_lane_pass import (StaticLanePass, measured_initial_join,
                              _reachable_join_dynamics)


class StaticLaneInitialJoinTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Failed first-stop run follow-20261016T130249Z-83953, t=5856.592824.
        cls.ego = dict(stamp=1792155856.26121, x=-59.911187105435175,
            y=-112.37834992854026, heading_deg=95.1526018042622,
            speed_kmh=32.5, wheel_angle=-2.115, measured=True, age_s=.02)
        waypoints = json.loads((Path(__file__).parent/'global_path.json').read_text())['waypoints']
        world = np.array([[p['x'], p['y']] for p in waypoints])
        smooth = gui.smooth_control_route(world)
        at = int(np.argmin(np.linalg.norm(world-[cls.ego['x'], cls.ego['y']], axis=1)))
        yaw = math.radians(cls.ego['heading_deg'])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        route = (smooth[(at+np.arange(240)) % len(world)]-[cls.ego['x'], cls.ego['y']]) @ rotation
        route = route[_arc(route) <= 60.]
        tangent, normal = _frame(route)
        d0 = -float(route[0] @ normal[0])
        slope0 = math.tan(-math.atan2(tangent[0, 1], tangent[0, 0]))
        cls.source = route+normal*offset_profile(_arc(route), d0, 0., 22., slope0=slope0)[:, None]
        cls.route = route
        forward = np.asarray(cls.source, np.float32)
        forward = forward[np.flatnonzero(forward[:, 0] > .05)[0]:]
        cls.original = gui.RosPathPublisher.control_geometry(np.vstack((np.zeros((1, 2)), forward)),
            route, dict(mode='LATTICE', validated_static=True))

    def join(self, path=None, ego=None):
        return measured_initial_join(self.original if path is None else path,
            self.ego if ego is None else ego, max_curvature=gui.MAX_CONTROL_CURVATURE,
            lateral_accel=gui.BYPASS_ALAT_MPS2)

    def test_recorded_origin_kink_is_removed_with_all_dynamics_retained(self):
        self.assertLess(max(gui.path_curvatures(self.source)), .02)
        self.assertAlmostEqual(max(gui.path_curvatures(self.original)), .08046904135, places=7)
        self.assertEqual(_reachable_join_dynamics(self.original, self.ego,
            gui.MAX_CONTROL_CURVATURE, gui.BYPASS_ALAT_MPS2), (False, True))
        original_copy = self.original.copy()
        joined = self.join()
        np.testing.assert_array_equal(self.original, original_copy)
        self.assertLess(max(gui.path_curvatures(joined)), .022)
        self.assertTrue(_reachable_join_dynamics(joined, self.ego,
            gui.MAX_CONTROL_CURVATURE, gui.BYPASS_ALAT_MPS2)[0])
        arc = _arc(joined)
        lateral = np.maximum(0., (32.5/3.6)**2-3.*np.maximum(
            0., arc[1:-1]-(32.5/3.6)*.7))*gui.path_curvatures(joined)
        self.assertLessEqual(max(lateral), gui.BYPASS_ALAT_MPS2+.05)
        np.testing.assert_array_equal(joined[0], [0., 0.])
        self.assertGreater(arc[-1], 59.)
        # A second measured check retains the verified geometry exactly.
        self.assertIs(self.join(joined), joined)

    def test_a_crawling_measured_pose_can_prepare_a_thirty_kmh_join(self):
        from static_lane_pass import _path_steering_profile
        slow = dict(self.ego, speed_kmh=5.)
        old = measured_initial_join(self.original, slow,
            max_curvature=gui.MAX_CONTROL_CURVATURE, lateral_accel=gui.BYPASS_ALAT_MPS2)
        self.assertIs(old, self.original)
        _, old_rate = _path_steering_profile(old, slow)
        self.assertLess(math.radians(30.)*.8/max(old_rate)*3.6, 6.)
        original = self.original.copy()
        joined = measured_initial_join(self.original, slow,
            max_curvature=gui.MAX_CONTROL_CURVATURE, lateral_accel=gui.BYPASS_ALAT_MPS2,
            target_mps=30./3.6)
        np.testing.assert_array_equal(self.original, original)
        self.assertIsNot(joined, self.original)
        self.assertTrue(_reachable_join_dynamics(joined, slow,
            gui.MAX_CONTROL_CURVATURE, gui.BYPASS_ALAT_MPS2)[0])
        self.assertTrue(_reachable_join_dynamics(joined, dict(slow, speed_kmh=30.),
            gui.MAX_CONTROL_CURVATURE, gui.BYPASS_ALAT_MPS2,
            steering_rate_fraction=.8)[0])
        _, rate = _path_steering_profile(joined, slow)
        self.assertGreaterEqual(math.radians(30.)*.8/max(rate)*3.6, 30.)
        self.assertLessEqual(max(gui.path_curvatures(joined))*(30./3.6)**2,
                             gui.BYPASS_ALAT_MPS2+.05)

    def test_missing_stale_unmeasured_and_out_of_range_steering_never_invents_curvature(self):
        for changes in (dict(measured=False), dict(age_s=.251), dict(age_s=float('nan')),
                        dict(src='file'), dict(wheel_angle=float('nan')),
                        dict(wheel_angle=float('inf')), dict(wheel_angle=40.01),
                        dict(speed_kmh=float('nan'))):
            with self.subTest(changes=changes):
                self.assertIs(self.join(ego=dict(self.ego, **changes)), self.original)
        for field in ('wheel_angle', 'age_s', 'measured', 'stamp', 'speed_kmh'):
            ego = dict(self.ego)
            ego.pop(field)
            self.assertIs(self.join(ego=ego), self.original)

    def test_mirrored_measured_turn_retains_signed_steering_checks(self):
        mirrored = self.original*np.array([1., -1.])
        ego = dict(self.ego, wheel_angle=-self.ego['wheel_angle'])
        joined = self.join(mirrored, ego)
        self.assertTrue(_reachable_join_dynamics(joined, ego,
            gui.MAX_CONTROL_CURVATURE, gui.BYPASS_ALAT_MPS2)[0])
        np.testing.assert_allclose(joined, self.join()*[1., -1.], atol=1e-10)

    def test_valid_path_keeps_its_geometry_and_unreachable_turn_stays_rejected(self):
        straight = np.column_stack((np.arange(0., 60.1, .5), np.zeros(121)))
        self.assertIs(self.join(straight, dict(self.ego, wheel_angle=0.)), straight)
        later_turn = straight.copy()
        later_turn[40:50, 1] = 5.
        self.assertIs(self.join(later_turn, dict(self.ego, wheel_angle=0.)), later_turn)
        impossible = self.join(ego=dict(self.ego, wheel_angle=40.))
        self.assertFalse(_reachable_join_dynamics(impossible, dict(self.ego, wheel_angle=40.),
            gui.MAX_CONTROL_CURVATURE, gui.BYPASS_ALAT_MPS2)[0])

    def plan(self, *, objects=(), points=None, zones=None):
        ego = dict(self.ego)
        legal = LaneCorridor(self.route, np.full(len(self.route), 2.),
            np.full(len(self.route), -2.), zones=zones, body_shift_m=gui.REAR_AXLE_SHIFT_M)
        lanes = SimpleNamespace(n_links=0, route_progress=lambda ego: 0.,
            rules=lambda: {'sections': []})
        def proposal(route, ego, points, static, bound, previous, **kwargs):
            path = kwargs['path_transform'](self.source)
            return dict(path=path, blocked=False, speed_cap_mps=25./3.6, hold_until=50.)
        with patch('lattice_planner.plan', proposal):
            return StaticLanePass().plan(lanes, ego, self.route, list(objects), list(objects),
                np.empty((0, 2)) if points is None else points, legal, None, ego['stamp'])

    def test_exact_replacement_runs_full_raw_sat_map_and_return_guards(self):
        plan, reason = self.plan()
        self.assertIsNotNone(plan, reason)
        self.assertFalse(plan['static_borrow'])
        self.assertGreater(plan['speed_cap_mps'], 4.)
        self.assertGreater(plan['clear_distance_m'], 59.)
        blocker = (self.ego['x'], self.ego['y']+15., 1., 'new blocker', 'obstacle', 0., 1., 1., 0.)
        self.assertIsNone(self.plan(objects=[blocker])[0])
        self.assertIsNone(self.plan(points=np.array([[10., 0.], [10., .2]]))[0])
        oversized = (self.ego['x'], self.ego['y']+15., 8., 'wide obstacle', 'obstacle', 0., 8., 8., 0.)
        self.assertIsNone(self.plan(objects=[oversized])[0])
        pedestrian = (self.ego['x'], self.ego['y']+15., 1., 'person', 'pedestrian', 0., 1., 1., 0.)
        self.assertIsNone(self.plan(objects=[pedestrian])[0])
        self.assertIsNone(self.plan(zones=[np.array([[8., -3.], [12., -3.],
            [12., 3.], [8., 3.]])])[0])


if __name__ == '__main__':
    unittest.main()
