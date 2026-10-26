"""Carry checked paths without inventing sharper corners through linear resampling."""
import math
import unittest
import numpy as np

from gui import (MAX_CONTROL_CURVATURE, build_forced_bypass, continue_static_plan,
                 local_xy, path_curvatures, static_clear_distance)
from lane_corridor import LaneCorridor
from static_lane_pass import StaticLanePass


class StaticPlanContinuationTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        self.path, self.curvature, transition = build_forced_bypass(
            self.route, 1., offset_m=2.8, hold_m=14.2, transitions=(10.,))
        self.saved = dict(stamp=10., world_path=self.path[:, :2],
                          plan=dict(direction=1., offset_m=2.8, transition_m=transition,
                                    hold_m=14.2, kind='bypass', curvature=self.curvature))
        self.ego = dict(x=.02, y=.004, heading_deg=.1, speed_kmh=0.)

    def test_valid_bypass_does_not_double_curvature_when_carried(self):
        self.assertLess(self.curvature, MAX_CONTROL_CURVATURE)
        continued = continue_static_plan(self.saved, self.ego, 10.1)
        self.assertIsNotNone(continued)
        self.assertLessEqual(float(path_curvatures(continued['path']).max()), MAX_CONTROL_CURVATURE)
        # The return occurs beyond the short measured-pose join. Its original
        # 0.5 m vertices must retain the checked physical curvature.
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(continued['path'], axis=0), axis=1))]
        future = continued['path'][arc >= 6.]
        self.assertLessEqual(float(path_curvatures(future).max()), self.curvature*1.001)

    def test_vertices_beyond_pose_join_keep_exact_measured_shape(self):
        continued = continue_static_plan(self.saved, self.ego, 10.1)
        transformed = np.array([local_xy(*point, self.ego) for point in self.saved['world_path']])
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(transformed, axis=0), axis=1))]
        self.assertEqual(len(continued['path']), len(transformed))
        np.testing.assert_allclose(continued['path'][arc >= 6.], transformed[arc >= 6.], atol=1e-12)
        self.assertLess(np.linalg.norm(continued['path'][0]), 1e-12)

    def test_duplicate_source_vertices_are_removed_without_new_interpolated_corners(self):
        repeated = np.repeat(self.saved['world_path'], 2, axis=0)
        duplicate = continue_static_plan(dict(self.saved, world_path=repeated), self.ego, 10.1)
        ordinary = continue_static_plan(self.saved, self.ego, 10.1)
        np.testing.assert_allclose(duplicate['path'], ordinary['path'], atol=1e-12)
        self.assertTrue(np.isfinite(path_curvatures(duplicate['path'])).all())

    def test_stale_far_pose_and_wrong_heading_still_reject_carry(self):
        self.assertIsNone(continue_static_plan(self.saved, self.ego, 11.01))
        self.assertIsNone(continue_static_plan(self.saved, self.ego, 9.99))
        self.assertIsNone(continue_static_plan(self.saved, dict(self.ego, y=2.), 10.1))
        self.assertIsNone(continue_static_plan(self.saved, dict(self.ego, heading_deg=90.), 10.1))

    def test_carried_geometry_still_detects_a_real_new_obstacle(self):
        continued = continue_static_plan(self.saved, self.ego, 10.1)
        point = self.saved['world_path'][30]
        obstacle = (point[0], point[1], 1., 'new_obstacle', 'obstacle', 0., 1., 1., 0.)
        self.assertLess(static_clear_distance(continued['path'], self.ego, [obstacle], 35.), 16.)

    def test_completed_geometry_bridges_only_inside_paths_for_two_seconds(self):
        policy = StaticLanePass()
        policy.legal = LaneCorridor(self.route, np.full(len(self.route), 1.8),
                                   np.full(len(self.route), -1.8))
        policy.origin = dict(x=0., y=0., heading_deg=0.)
        policy.finished = True
        policy.retire_completed_geometry(10.)
        policy.legal = policy.origin = None
        ego = dict(x=0., y=0., heading_deg=0.)
        checked = policy.assess(self.route, ego, 2., None, 10.1, motion_profile=True)
        self.assertTrue(checked['ok'], checked)
        self.assertTrue(checked['completed_return'])
        self.assertFalse(policy.assess(self.route+[0., 1.], ego, 2., None,
                                       10.1, motion_profile=True)['ok'])
        self.assertFalse(policy.assess(self.route, ego, 2., None,
                                       12.1, motion_profile=True)['ok'])

    def test_ordinary_bounded_pass_preserves_its_checked_world_curve(self):
        saved = dict(self.saved, plan=dict(self.saved['plan'], bounded_static_pass=True,
            clear_distance_m=50., hold_until_m=35.))
        continued = continue_static_plan(saved, self.ego, 10.1)
        self.assertIsNotNone(continued)
        self.assertTrue(continued['fixed_world_curve'])
        self.assertLess(np.linalg.norm(continued['path'][0]), .02)
        # A world curve remains on its original side at the measured pose;
        # continuation must not replace its first vertices with a new join.
        world = np.array([gui_point for gui_point in saved['world_path']])
        expected = np.array([local_xy(*point, self.ego) for point in world])
        from planning.highway.path_continuity import projected_suffix
        from gui import RosPathPublisher
        np.testing.assert_allclose(continued['path'],
            RosPathPublisher.resample(projected_suffix(expected)), atol=1e-12)


if __name__ == '__main__':
    unittest.main()
