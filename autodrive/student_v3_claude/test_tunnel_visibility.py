"""Unknown LiDAR sectors must not be treated as checked tunnel free space."""
import math
import unittest

import numpy as np

from tunnel_visibility import ObservedLidarCoverage


def returns(radius=40., gap_degrees=None, height=0.):
    angle = np.linspace(-math.pi, math.pi, 3600, endpoint=False) + math.radians(.05)
    if gap_degrees is not None:
        angle = angle[np.abs(np.degrees(angle)) >= gap_degrees]
    return np.column_stack((1.05 + radius*np.cos(angle), radius*np.sin(angle),
                            np.full(len(angle), height)))


def straight(length=30.):
    x = np.arange(0., length + .01, .25)
    return np.column_stack((x, np.zeros(len(x))))


class TunnelVisibilityTests(unittest.TestCase):
    def test_complete_measured_scan_supports_the_full_body_corridor(self):
        coverage = ObservedLidarCoverage(returns(), [0., 0., 0.])
        self.assertEqual(coverage.status()["supported_bins"], 180)
        self.assertEqual(coverage.clear_distance(straight(), 30.), 30.)

    def test_missing_forward_sector_is_unknown_and_stops(self):
        coverage = ObservedLidarCoverage(returns(gap_degrees=6.), [0., 0., 0.])
        self.assertEqual(coverage.clear_distance(straight(), 30.), 0.)

    def test_short_range_scan_caps_before_its_front_footprint_runs_out(self):
        coverage = ObservedLidarCoverage(returns(radius=12.), [0., 0., 0.])
        distance = coverage.clear_distance(straight(), 30.)
        self.assertGreater(distance, 5.)
        # Source range is measured from the laser at body x=1.05 m.
        self.assertLessEqual(distance, 10.)

    def test_overhead_returns_cannot_supply_vehicle_height_visibility(self):
        coverage = ObservedLidarCoverage(returns(height=6.), [0., 0., 0.])
        self.assertEqual(coverage.clear_distance(straight(), 30.), 0.)

    def test_a_lone_distant_return_does_not_extend_a_sector(self):
        points = returns(radius=8.)
        far = np.array([[101.05, .1, 0.]])
        coverage = ObservedLidarCoverage(np.vstack((points, far)), [0., 0., 0.])
        self.assertLess(coverage.reach.max(), 8.)
        self.assertLessEqual(coverage.clear_distance(straight(), 30.), 6.)

    def test_same_azimuth_vertical_channels_do_not_cover_a_sector(self):
        points = np.array([[31.05, 0., z] for z in np.linspace(0., 2., 16)])
        coverage = ObservedLidarCoverage(points, [0., 0., 0.])
        self.assertEqual(coverage.status()["supported_bins"], 0)

    def test_recorded_exit_wall_inside_old_radial_cutoff_retains_support(self):
        # Actual vehicle-frame returns, source stamp 1790960584.8187146,
        # follow-20261002T165436Z-1549766. The old 2.5 m cutoff erased both
        # observed -82..-78 degree bins and made the whole path distance zero.
        wall = np.array([
            [1.362187, -2.221324, 1.074575], [1.375302, -2.216327, .914574],
            [1.396770, -2.237417, 1.956672], [1.408158, -2.218693, 1.153377],
            [1.423799, -2.224193, 1.547321], [1.435284, -2.207579, .749540],
            [1.443887, -2.231567, 1.957190], [1.457911, -2.224720, 1.708235],
            [1.471726, -2.219184, 1.468384], [1.492142, -2.224826, 1.873686],
            [1.505827, -2.218649, 1.628106], [1.519839, -2.212315, 1.389477],
        ])
        points = returns(height=-.37537024)
        angle = np.degrees(np.arctan2(points[:, 1], points[:, 0]-1.05))
        points = np.vstack((points[(angle < -82.) | (angle >= -78.)], wall))
        coverage = ObservedLidarCoverage(points, [.00221648, -.00385580, -.37537024])
        self.assertTrue(np.all(np.linalg.norm(wall[:, :2]-[1.05, 0.], axis=1) < 2.5))
        self.assertTrue(np.all(coverage.reach[[49, 50]] > 1.7))
        self.assertEqual(coverage.status()["supported_bins"], 180)
        self.assertEqual(coverage.clear_distance(straight(), 30.), 30.)
        # Actual short side reach still limits a turn into that wall sector.
        x = np.arange(0., 30.01, .25)
        turn = np.column_stack((x, -.2*x*x))
        self.assertLess(coverage.clear_distance(turn, 30.), 5.)

    def test_close_front_returns_remain_observed_and_stop_before_the_footprint(self):
        points = returns()
        angle = np.arctan2(points[:, 1], points[:, 0]-1.05)
        front = np.abs(angle) < math.radians(6.)
        points[front, 0] = 1.05 + 1.8*np.cos(angle[front])
        points[front, 1] = 1.8*np.sin(angle[front])
        coverage = ObservedLidarCoverage(points, [0., 0., 0.])
        self.assertGreater(coverage.reach[89], 0.)
        self.assertLess(coverage.reach[89], 1.4)
        self.assertEqual(coverage.clear_distance(straight(), 30.), 0.)

    def test_own_body_returns_cannot_fill_unobserved_sectors(self):
        body = returns(radius=.6, height=1.)
        coverage = ObservedLidarCoverage(body, [0., 0., 0.])
        self.assertEqual(coverage.status()["supported_bins"], 0)
        self.assertEqual(coverage.clear_distance(straight(), 30.), 0.)

    def test_curved_path_checks_vehicle_orientation_and_measured_range(self):
        theta = np.linspace(0., 1., 161)
        path = np.column_stack((20.*np.sin(theta), 20.*(1.-np.cos(theta))))
        coverage = ObservedLidarCoverage(returns(radius=40.), [0., 0., 0.])
        self.assertGreaterEqual(coverage.clear_distance(path, 20.), 19.)
        limited = ObservedLidarCoverage(returns(radius=12.), [0., 0., 0.])
        self.assertLess(limited.clear_distance(path, 20.), 10.)

    def test_missing_and_malformed_scans_fail_closed(self):
        for points, plane in ((None, [0., 0., 0.]), (np.zeros((0, 3)), [0., 0., 0.]),
                              (np.zeros((2, 2)), [0., 0., 0.]),
                              (returns(), [float("nan"), 0., 0.])):
            with self.subTest():
                self.assertEqual(ObservedLidarCoverage(points, plane).clear_distance(straight(), 30.), 0.)

    def test_plane_height_and_ray_origin_share_the_scan_frame(self):
        points = returns()
        translation = np.array([5., -3.])
        shifted = points.copy()
        shifted[:, :2] += translation
        path = straight() + translation
        original = ObservedLidarCoverage(points, [0., 0., 0.]).clear_distance(straight(), 30.)
        moved = ObservedLidarCoverage(shifted, [0., 0., 0.], origin_xy=np.array([1.05, 0.])+translation)
        self.assertEqual(moved.clear_distance(path, 30.), original)


if __name__ == "__main__":
    unittest.main()
