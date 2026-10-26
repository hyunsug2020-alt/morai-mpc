"""Geometry and timing regressions for the deterministic tunnel branch."""
import math
import unittest

import numpy as np

from tunnel_runtime import (lidar_objects, objects_at_pose, points_at_pose, pose_at_stamp,
                            propagate_match, swept_clearance, lattice_swept_clearance,
                            measured_wall_corridor, corridor_at_pose)


def pose(x=0., y=0., heading=0., stamp=100., speed=0.):
    return dict(x=x, y=y, heading_deg=heading, stamp=stamp, speed_kmh=speed)


class TunnelRuntimeHelperTests(unittest.TestCase):
    def test_pose_refresh_preserves_the_physical_no_go_area(self):
        from lane_corridor import LaneCorridor
        route = np.column_stack((np.arange(0., 65.5, .5), np.zeros(131)))
        zone = np.array([[20., -2.], [22., -2.], [22., 2.], [20., 2.]])
        corridor = LaneCorridor(route, np.full(131, 5.), np.full(131, -5.), zones=[zone])
        current = pose(10., 0., 15.)
        refreshed = corridor_at_pose(corridor, pose(), current)
        world_path = route[20:]
        path = points_at_pose(world_path, pose(), current)
        self.assertGreater(refreshed.clear_distance(path, 40.), 6.)
        self.assertLess(refreshed.clear_distance(path, 40.), 9.)
        bypass = points_at_pose(world_path+[0., 3.5], pose(), current)
        self.assertGreaterEqual(refreshed.clear_distance(bypass, 40.), 40.)

    def test_measured_walls_keep_asymmetric_space_when_exit_route_curves(self):
        walls = dict(validated=True, left=dict(slope=0., intercept_m=4.65),
                     right=dict(slope=0., intercept_m=-7.30))
        x = np.arange(0., 80., .5)
        route = np.column_stack((x, .02*np.maximum(x-35., 0.)**2))
        corridor = measured_wall_corridor(route, walls, pose(), pose())
        self.assertIsNotNone(corridor)
        wide = np.column_stack((np.arange(0., 40., .5), np.full(80, -5.)))
        narrow = np.column_stack((np.arange(0., 40., .5), np.full(80, 4.)))
        self.assertGreaterEqual(corridor.clear_distance(wide, 30.), 30.)
        self.assertLess(corridor.clear_distance(narrow, 30.), 1.)

    def test_measured_wall_bounds_follow_source_pose_and_current_rotation(self):
        walls = dict(validated=True, left=dict(slope=0., intercept_m=6.),
                     right=dict(slope=0., intercept_m=-3.))
        current = pose(4., 1., 90.)
        x = np.arange(0., 30., .5)
        world_path = np.column_stack((x, np.zeros(len(x))))
        from tunnel_runtime import rotation
        path = (world_path-[current['x'], current['y']]) @ rotation(current)
        corridor = measured_wall_corridor(path, walls, pose(), current)
        self.assertIsNotNone(corridor)
        self.assertGreaterEqual(corridor.clear_distance(path, 20.), 20.)
        wall_hit = (world_path+[0., 5.5]-[current['x'], current['y']]) @ rotation(current)
        self.assertLess(corridor.clear_distance(wall_hit, 20.), 1.)

    def test_unvalidated_walls_cannot_expand_fallback_road(self):
        route = np.column_stack((np.arange(20.), np.zeros(20)))
        for walls in (None, dict(validated=False),
                      dict(validated=True, left=dict(slope=0., intercept_m=2.),
                           right=dict(slope=0., intercept_m=-2.))):
            self.assertIsNone(measured_wall_corridor(route, walls, pose(), pose()))

    def test_beside_return_is_still_checked_when_path_turns_into_it(self):
        x = np.arange(0., 20.25, .25)
        straight = np.column_stack((x, np.zeros(len(x))))
        turning = np.column_stack((x, .25*x))
        points = np.array([[1., 1.1]])
        self.assertGreaterEqual(lattice_swept_clearance(straight, points, 15., .35, .3), 15.)
        self.assertEqual(lattice_swept_clearance(turning, points, 15., .35, .3), 0.)

    def test_vectorized_curvature_matches_scalar_geometry(self):
        from gui import path_curvatures
        random = np.random.default_rng(20261003)
        paths = [np.cumsum(random.normal(size=(400, 3)), axis=0),
                 np.zeros((10, 2)), np.array([[0., 0.], [0., 0.], [1., 2.]]),
                 np.array([[0., 0.], [1e-4, 0.], [2e-4, 1e-4]])]
        for points in paths:
            expected = []
            for a, b, c in zip(points[:-2, :2], points[1:-1, :2], points[2:, :2]):
                u, v, w = b-a, c-b, c-a
                denominator = np.linalg.norm(u)*np.linalg.norm(v)*np.linalg.norm(w)
                expected.append(0. if denominator < 1e-8 else
                                2.*abs(u[0]*w[1]-u[1]*w[0])/denominator)
            np.testing.assert_allclose(path_curvatures(points), expected, rtol=1e-12, atol=1e-12)
        angles = np.linspace(0., 1., 100)
        circle = 25.*np.column_stack((np.sin(angles), 1.-np.cos(angles)))
        np.testing.assert_allclose(path_curvatures(circle), 1./25., rtol=1e-10)
        self.assertTrue(np.isnan(path_curvatures([[0., 0.], [1., np.nan], [2., 0.]])[0]))

    def test_source_time_interpolation_uses_short_yaw_arc(self):
        samples = [pose(0., 0., 179., 100., 18.), pose(2., 4., -179., 100.1, 36.)]
        result = pose_at_stamp(samples, 100.05)
        self.assertAlmostEqual(result["x"], 1.)
        self.assertAlmostEqual(result["y"], 2.)
        self.assertAlmostEqual(result["speed_kmh"], 27.)
        self.assertAlmostEqual(abs(result["heading_deg"]), 180.)
        self.assertEqual(result["stamp"], 100.05)

    def test_extrapolation_is_forward_only_and_bounded(self):
        samples = [pose(4., 5., 90., 100., 36.)]
        result = pose_at_stamp(samples, 100.1)
        self.assertAlmostEqual(result["x"], 4.)
        self.assertAlmostEqual(result["y"], 6.)
        self.assertIsNone(pose_at_stamp(samples, 100.121))
        self.assertIsNone(pose_at_stamp(samples, 99.9))
        self.assertIsNone(pose_at_stamp([], 100.))
        self.assertIsNone(pose_at_stamp(samples, float("nan")))

    def test_exact_pose_is_valid_even_after_a_long_publication_gap(self):
        samples = [pose(0., 0., stamp=100.), pose(1., 0., stamp=101.)]
        self.assertEqual(pose_at_stamp(samples, 101.)["x"], 1.)
        self.assertIsNone(pose_at_stamp(samples, 100.5))

    def test_publication_gap_uses_the_same_bounded_forward_prediction(self):
        samples = [pose(0., 0., stamp=100., speed=36.), pose(3., 0., stamp=100.3, speed=36.)]
        self.assertAlmostEqual(pose_at_stamp(samples, 100.1)['x'], 1.)
        self.assertIsNone(pose_at_stamp(samples, 100.15))
        self.assertAlmostEqual(pose_at_stamp(samples, 100.25)['x'], 2.5)
        self.assertIsNone(pose_at_stamp(samples, 99.95))

    def test_vectorized_swept_body_matches_station_by_station_reference(self):
        from scipy.spatial import cKDTree
        rng = np.random.default_rng(20261003)
        for trial in range(20):
            x = np.linspace(0., 30., 121)
            path = np.column_stack((x, .5*np.sin(x/(5.+trial))))
            points = rng.uniform([-4., -8.], [40., 8.], (300, 2))
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
            sample = np.arange(0., min(30., arc[-1])+.001, .25)
            centres = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
            tangent = np.gradient(centres, axis=0)
            tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
            normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
            expected = 30.
            for station, centre, along, lateral in zip(sample, centres, tangent, normal):
                delta = points-centre
                if np.any((abs(delta @ along) <= 2.5675) & (abs(delta @ lateral) <= 1.196)):
                    expected = float(station)
                    break
            self.assertEqual(swept_clearance(path, points, 30., tree=cKDTree(points)), expected)

    def test_same_stamp_uses_latest_corrected_pose(self):
        samples = [pose(0., 0., stamp=100.), pose(1., 0., stamp=100.),
                   pose(2., 0., stamp=100.1)]
        self.assertEqual(pose_at_stamp(samples, 100.)["x"], 1.)
        self.assertAlmostEqual(pose_at_stamp(samples, 100.05)["x"], 1.5)

    def test_full_path_clearance_and_identical_prefix_have_the_same_lane_predicate(self):
        from tunnel_map import TunnelMap
        model = TunnelMap(dict(origin_xy=[0., 0.], axis_heading_deg=0.,
            left_offset_m=4., right_offset_m=-4., longitudinal_min_m=-10., longitudinal_max_m=100.))
        rng = np.random.default_rng(7410)
        for trial in range(30):
            x = np.arange(0., 40.01, .5)
            path = np.column_stack((x, (trial%6)*.3*np.sin(x/(3.+trial%7))))
            prefix = path[:int(rng.integers(12, 70))]
            length = float(np.linalg.norm(np.diff(prefix, axis=0), axis=1).sum())
            points = rng.uniform([-3., -6.], [45., 6.], (30, 2))
            # Also exercise a return at the prefix endpoint, where the one-sided
            # tangent differs from the interior tangent of the longer path.
            points = np.vstack((points, prefix[-1]+[0., 1.19]))
            full_clear = min(model.path_clearance(path, pose(), 40.), swept_clearance(path, points, 40.))
            prefix_clear = min(model.path_clearance(prefix, pose(), length), swept_clearance(prefix, points, length))
            self.assertEqual(min(full_clear, length) >= length-.3, prefix_clear >= length-.3)

    def test_match_propagation_rotates_odometry_translation_and_yaw(self):
        scan = pose(10., 20., 90., 100.)
        current = pose(10., 22., 100., 100.1)
        matched = pose(100., 200., 180., 100.)
        propagated = propagate_match(matched, scan, current)
        np.testing.assert_allclose([propagated["x"], propagated["y"]], [98., 200.], atol=1e-12)
        self.assertEqual(propagated["heading_deg"], 190.)
        self.assertEqual(propagated["stamp"], current["stamp"])

    def test_lidar_relative_motion_agrees_between_raw_and_matched_frames(self):
        scan = pose(10., 20., 90., 100.)
        current = pose(10., 22., 100., 100.1)
        matched = pose(100., 200., 180., 100.)
        aligned = propagate_match(matched, scan, current)
        points = np.array([[5., 0.], [8., 2.], [-3., -2.]])
        raw_relative = points_at_pose(points, scan, current)
        matched_relative = points_at_pose(points, matched, aligned)
        np.testing.assert_allclose(raw_relative, matched_relative, atol=1e-12)
        np.testing.assert_allclose(points_at_pose(raw_relative, current, scan), points, atol=1e-12)

    def test_pure_odometry_prevents_feedback_correction_from_becoming_motion(self):
        # The ESKF has already moved from y=0 to y=1 after the wall observation.
        # Independent odometry reports no vehicle motion, so do not add y=1 twice.
        scan_odometry = pose(100., 200., 0., 100.)
        current_odometry = pose(100., 200., 0., 100.1)
        wall_match = pose(0., 1., 0., 100.)
        current_eskf = pose(0., 1., 0., 100.1)
        aligned = propagate_match(wall_match, scan_odometry, current_odometry)
        np.testing.assert_allclose([aligned["x"], aligned["y"]],
                                   [current_eskf["x"], current_eskf["y"]])
        points = np.array([[8., 0.], [3., 2.]])
        np.testing.assert_array_equal(points_at_pose(points, scan_odometry, current_odometry), points)

    def test_memory_keeps_measured_coordinates_instead_of_quantized_cell_centres(self):
        from highway_planner import NearFieldMemory
        memory = NearFieldMemory(retain_measured_points=True)
        measured = np.array([[5.11, -1.31], [5.18, -1.28], [5.2, -1.35], [5.21, -1.32]])
        ego = pose(heading=17.)
        memory.update(measured, ego, 100.)
        memory.update(measured, ego, 100.1)
        retained = memory.update(np.empty((0, 2)), ego, 100.2)
        self.assertGreater(len(retained), 0)
        for point in retained:
            self.assertLess(np.linalg.norm(measured-point, axis=1).min(), 1e-12)
        self.assertTrue(np.all(abs(retained[:, 1]) > 1.196))

    def test_dynamic_box_and_velocity_transform_to_pure_odometry_world(self):
        old = pose(10., 20., 90.)
        odometry = pose(100., 200., 180.)
        obj = (10., 25., 2., 'moving', 'pedestrian', 1., 2., 1., 95., 0., 1., 42)
        transformed = objects_at_pose([obj], old, odometry)[0]
        np.testing.assert_allclose(transformed[:2], [95., 200.], atol=1e-12)
        np.testing.assert_allclose(transformed[9:11], [-1., 0.], atol=1e-12)
        self.assertEqual(transformed[8], 185.)
        self.assertEqual(transformed[11], 42)

    def test_singletons_and_clusters_make_vehicle_oriented_boxes(self):
        points = np.array([[8., 0.], [8.2, 0.], [8.2, .2], [14., -3.]])
        ego = pose(100., 200., 90.)
        objects = lidar_objects(points, ego)
        self.assertEqual(len(objects), 2)
        self.assertTrue(all(obj[4] == "obstacle" and obj[5] == 0. for obj in objects))
        self.assertTrue(all(obj[6] >= .3 and obj[7] >= .3 for obj in objects))
        self.assertTrue(all(obj[8] == 90. for obj in objects))
        singleton = next(obj for obj in objects if obj[1] > 212.)
        np.testing.assert_allclose(singleton[:2], [103., 214.], atol=1e-12)
        self.assertEqual(lidar_objects(np.zeros((0, 2)), ego), [])

    def test_fitted_cluster_box_contains_every_rotated_quantization_cell(self):
        theta = math.radians(13.)
        axes = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
        surface = np.array([(x, y) for x in np.arange(6., 10., .2) for y in (-2., -1.)])
        surface = np.vstack((surface, [[6., y] for y in np.arange(-2., -1., .2)])) @ axes.T
        ego = pose()
        objects = lidar_objects(surface, ego)
        self.assertEqual(len(objects), 1)
        obj = objects[0]
        self.assertGreater(abs(obj[8]), 5.)
        yaw = math.radians(obj[8])
        measured_axes = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        cells = np.unique(np.round(surface/.2), axis=0)*.2
        corners = np.vstack([cells+[x, y] for x in (-.1, .1) for y in (-.1, .1)])
        local = (corners-obj[:2]) @ measured_axes
        self.assertTrue(np.all(abs(local[:, 0]) <= obj[6]/2.+1e-8))
        self.assertTrue(np.all(abs(local[:, 1]) <= obj[7]/2.+1e-8))

    def test_swept_body_includes_front_sides_and_rear(self):
        path = np.column_stack((np.arange(0., 20.25, .25), np.zeros(81)))
        self.assertAlmostEqual(swept_clearance(path, [[10., 0.]], 20.), 7.5)
        self.assertLess(swept_clearance(path, [[5., 1.]], 20.), 5.)
        self.assertEqual(swept_clearance(path, [[-2., 0.]], 20.), 0.)
        self.assertEqual(swept_clearance(path, [[-3., 0.]], 20.), 20.)
        self.assertEqual(swept_clearance(path, np.zeros((0, 2)), 20.), 20.)
        self.assertEqual(swept_clearance(path, None, 20.), 0.)

    def test_swept_body_uses_curved_path_heading(self):
        radius = 20.
        theta = np.linspace(0., 1., 161)
        path = np.column_stack((radius * np.sin(theta), radius * (1. - np.cos(theta))))
        theta_obstacle = .5
        centre = np.array([radius * math.sin(theta_obstacle), radius * (1. - math.cos(theta_obstacle))])
        normal = np.array([-math.sin(theta_obstacle), math.cos(theta_obstacle)])
        points = (centre + .7 * normal)[None]
        clearance = swept_clearance(path, points, 20.)
        self.assertGreater(clearance, 0.)
        self.assertLess(clearance, 10.)
        straight = np.column_stack((np.arange(0., 20.25, .25), np.zeros(81)))
        self.assertEqual(swept_clearance(straight, points, 20.), 20.)


if __name__ == "__main__":
    unittest.main()

class FallbackExitRejoinTests(unittest.TestCase):
    def test_fallback_path_returns_to_route_before_the_exit(self):
        # Auto run 167: a 4 m bypass offset held through the exit left the car 4.7 m
        # off the route against the exit kerb.
        from tunnel_runtime import tunnel_parallel_path, FALLBACK_EXIT_REJOIN_M
        x = np.arange(-4., 80.01, .25)
        route = np.column_stack((x, np.full(len(x), -4.)))      # car 4 m left of the route
        path = tunnel_parallel_path(route, to_exit_m=50.)
        lateral = path[:, 1] - route[:, 1]
        self.assertAlmostEqual(float(lateral[np.argmin(np.abs(x - 15.))]), 4., places=1)
        self.assertLess(abs(float(lateral[np.argmin(np.abs(x - 50.))])), .05)
        hold = tunnel_parallel_path(route)
        self.assertAlmostEqual(float((hold[:, 1] - route[:, 1])[np.argmin(np.abs(x - 60.))]), 4., places=1)
