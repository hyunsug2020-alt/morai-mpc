"""Tunnel wall matching preserves collision returns and refuses unobserved structure."""
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tunnel_map import TunnelMap


class TunnelMapTests(unittest.TestCase):
    def setUp(self):
        self.payload = dict(format='morai_fixed_tunnel_walls_v1', origin_xy=[0., 0.],
            axis_heading_deg=0., left_offset_m=4., right_offset_m=-4.,
            longitudinal_min_m=0., longitudinal_max_m=100.)
        self.model = TunnelMap(self.payload)
        self.ego = dict(x=20., y=0., heading_deg=0.)
        x, z = np.meshgrid(np.arange(-10., 60.01, .25), [.2, .8, 1.4, 2.0])
        # Ground is -0.4 m relative to the vehicle body centre.
        self.left = np.column_stack((x.ravel(), np.full(x.size, 4.), z.ravel()))
        self.right = self.left.copy(); self.right[:, 1] = -4.
        self.walls = np.vstack((self.left, self.right))

    def test_empty_tunnel_surfaces_are_removed_with_two_supported_walls(self):
        result = self.model.process(self.walls, self.ego, [0., 0., -.4])
        self.assertTrue(result['valid'], result['reason'])
        self.assertEqual(len(result['obstacle_points']), 0)
        self.assertEqual(result['wall_stats']['longitudinal_correction_m'], 0.)

    def test_centre_and_wall_adjacent_obstacles_remain_in_original_body_frame(self):
        obstacles = np.array([[12., 0., .1], [12., 3.8, .1], [12., 3.96, .1]])
        result = self.model.process(np.vstack((self.walls, obstacles)), self.ego, [0., 0., -.4])
        self.assertTrue(result['valid'])
        np.testing.assert_allclose(result['obstacle_points'], obstacles[:, :2])

    def test_continuous_wall_kerb_folds_into_the_wall_but_objects_at_it_remain(self):
        # Auto runs 164/165: a 0.4-0.6 m kerb (1.2 m high) along both walls became
        # thousands of obstacle returns. A continuous kerb is wall; the wall face
        # used for planning moves inward by its width. A drum standing at the kerb
        # (short, protruding further) stays an obstacle.
        x, z = np.meshgrid(np.arange(-10., 60.01, .25), [-.2, .2, .6])
        kerb_l = np.column_stack((x.ravel(), np.full(x.size, 3.5), z.ravel()))
        kerb_r = kerb_l.copy(); kerb_r[:, 1] = -3.5
        drum = np.array([[12., 2.9, .2], [12., 2.9, .5], [12.2, 2.9, .2]])
        result = self.model.process(np.vstack((self.walls, kerb_l, kerb_r, drum)), self.ego, [0., 0., -.4])
        self.assertTrue(result['valid'], result['reason'])
        np.testing.assert_allclose(result['obstacle_points'], drum[:, :2])
        self.assertAlmostEqual(self.model.ledge[1], .5, places=2)
        self.assertAlmostEqual(self.model.inner_left, 3.7, places=2)     # 0.5 kerb - 0.2 credit
        self.assertAlmostEqual(self.model.inner_right, -3.7, places=2)

    def test_short_object_at_the_wall_is_not_taken_for_a_kerb(self):
        # A parked car hugging the wall (4.5 m) is shorter than the kerb span rule.
        x, z = np.meshgrid(np.arange(10., 14.51, .1), [-.2, .2, .6])
        car_side = np.column_stack((x.ravel(), np.full(x.size, 3.4), z.ravel()))
        result = self.model.process(np.vstack((self.walls, car_side)), self.ego, [0., 0., -.4])
        self.assertTrue(result['valid'])
        self.assertEqual(len(result['obstacle_points']), len(car_side))
        self.assertEqual(self.model.ledge[1], 0.)

    def test_ground_self_and_above_vehicle_returns_are_excluded(self):
        extra = np.array([[15., 0., -.4], [0., 0., .5], [15., 0., 2.5]])
        result = self.model.process(np.vstack((self.walls, extra)), self.ego, [0., 0., -.4])
        self.assertTrue(result['valid'])
        self.assertEqual(len(result['obstacle_points']), 0)

    def test_bounded_lateral_and_yaw_alignment_never_corrects_along_track(self):
        observed = dict(self.ego, y=.5, heading_deg=2.)
        result = self.model.process(self.walls, observed, [0., 0., -.4])
        self.assertTrue(result['valid'], result['reason'])
        self.assertAlmostEqual(result['aligned_ego']['x'], observed['x'])
        self.assertAlmostEqual(result['aligned_ego']['y'], 0., places=7)
        self.assertAlmostEqual(result['aligned_ego']['heading_deg'], 0., places=7)
        self.assertEqual(result['constraint']['source'], 'tunnel_lidar_wall')
        self.assertGreaterEqual(result['constraint']['lateral_variance_m2'], .09)
        self.assertGreaterEqual(result['constraint']['yaw_variance_rad2'], math.radians(2.)**2)

    def test_large_pose_error_one_wall_and_missing_scan_refuse_alignment(self):
        for points, ego in ((self.walls, dict(self.ego, y=2.)),
                            (self.walls, dict(self.ego, heading_deg=20.)),
                            (self.left, self.ego), (np.empty((0, 3)), self.ego)):
            self.assertFalse(self.model.process(points, ego, [0., 0., -.4])['valid'])

    def test_rejected_map_pose_keeps_separate_fresh_local_planning_projection(self):
        ego = dict(self.ego, y=.9)
        obstacle = np.array([[12., 0., .1], [12., 3.8, .1]])
        points = np.vstack((self.walls, obstacle))
        rejected = self.model.process(points, ego, [0., 0., -.4])
        self.assertFalse(rejected['valid'])
        projected = self.model.relative_projection(points, ego, [0., 0., -.4], rejected)
        self.assertTrue(projected['valid'])
        self.assertTrue(projected['relative_geometry'])
        self.assertIsNone(projected['constraint'])
        self.assertNotIn('propagated', projected['wall_stats'])
        for point in obstacle:
            self.assertTrue(np.any(np.linalg.norm(projected['obstacle_points']-point[:2], axis=1) < 1e-6))
        self.assertFalse(rejected['valid'])

    def test_local_projection_does_not_accept_unsupported_walls_or_excessive_error(self):
        ego = dict(self.ego, y=.9)
        rejected = self.model.process(self.walls, ego, [0., 0., -.4])
        for changed in (dict(rejected, body_walls={}),
                        dict(rejected, reason='wall width or parallelism inconsistent'),
                        dict(rejected, wall_stats=dict(rejected['wall_stats'], lateral_correction_m=1.21)),
                        dict(rejected, wall_stats=dict(rejected['wall_stats'], yaw_correction_deg=3.01))):
            self.assertIsNone(self.model.relative_projection(self.walls, ego, [0., 0., -.4], changed))

    def test_rejected_absolute_pose_retains_only_fresh_supported_wall_geometry(self):
        result = self.model.process(self.walls, dict(self.ego, y=.9), [0., 0., -.4])
        self.assertFalse(result['valid'])
        self.assertIsNone(result.get('constraint'))
        self.assertTrue(result['body_walls']['validated'])
        self.assertAlmostEqual(result['body_walls']['left']['intercept_m'], 4.)
        self.assertAlmostEqual(result['body_walls']['right']['intercept_m'], -4.)
        for scan in (self.left, np.vstack((self.left, self.right+[0., 1., 0.]))):
            rejected = self.model.process(scan, self.ego, [0., 0., -.4])
            self.assertFalse(rejected['valid'])
            self.assertNotIn('body_walls', rejected)

    def test_vehicle_corners_enforce_wall_at_map_entrance_before_centre_enters(self):
        path = np.column_stack((np.arange(0., 20.1, 5.), np.zeros(5)))
        # The pose is the rear axle: the front bumper (+0.25 m) is 4.095 m ahead of it,
        # so from x = -4.53 it reaches the map start (x = 0) after 0.43 m.
        free = self.model.path_clearance(path, dict(x=-4.53, y=4., heading_deg=0.), 20.)
        self.assertEqual(free, .5)
        # From x = -3 the real front is already inside the mapped section.
        self.assertEqual(self.model.path_clearance(path, dict(x=-3., y=4., heading_deg=0.), 20.), 0.)
        self.assertEqual(self.model.path_clearance(path, self.ego, 20.), 20.)
        self.assertEqual(self.model.path_clearance(path, dict(x=-50., y=4., heading_deg=0.), 20.), 20.)

    def test_scan_pose_just_before_entry_can_match_observed_wall_surfaces(self):
        result = self.model.process(self.walls, dict(self.ego, x=-.1), [0., 0., -.4])
        self.assertTrue(result['valid'], result['reason'])
        self.assertFalse(self.model.process(self.walls, dict(self.ego, x=-3.), [0., 0., -.4])['valid'])

    def test_wall_corridor_permits_both_sides_without_lane_metadata(self):
        route = np.column_stack((np.arange(0., 60.1, .5), np.zeros(121)))
        corridor = self.model.corridor(self.ego)
        self.assertEqual(corridor.clear_distance(route, 60.), 60.)
        offsets = self.model.adjacent_offsets(self.ego, route)
        self.assertGreater(offsets[1], 2.6)
        self.assertGreater(offsets[-1], 2.6)
        outside = route.copy(); outside[:, 1] = 3.
        self.assertEqual(corridor.clear_distance(outside, 60.), 0.)

    def test_wall_corridor_does_not_invent_walls_beyond_physical_exit(self):
        ego = dict(self.ego, x=98.)
        path = np.array([[0., 0.], [4., 0.], [12., 8.], [20., 12.]])
        corridor = self.model.corridor(ego)
        self.assertEqual(corridor.clear_distance(path, 20.), self.model.path_clearance(path, ego, 20.))

    def test_known_exit_requires_two_dense_vertical_walls_with_opposite_full_support(self):
        x, z = np.meshgrid(np.arange(-13., 2.001, .15), [.8, 1.4, 2., 2.6])
        short = np.column_stack((x.ravel(), np.full(x.size, -4.), z.ravel()))
        x, z = np.meshgrid(np.arange(-35., 2.001, .15), [.8, 1.4, 2., 2.6])
        full = np.column_stack((x.ravel(), np.full(x.size, 4.), z.ravel()))
        obstacle = np.array([[5., 0., .1]])
        points = np.vstack((full, short, obstacle))
        result = self.model.process(points, dict(self.ego, x=99.), [0., 0., -.4])
        self.assertTrue(result['valid'], result['reason'])
        self.assertTrue(result['wall_stats']['surveyed_exit_support'])
        self.assertTrue(np.any(np.linalg.norm(result['obstacle_points']-obstacle[0, :2], axis=1) < 1e-6))
        # Identical short returns away from the surveyed end, one missing wall,
        # or two short surfaces cannot relax the ordinary matching requirement.
        self.assertFalse(self.model.process(points, dict(self.ego, x=50.), [0., 0., -.4])['valid'])
        self.assertFalse(self.model.process(full, dict(self.ego, x=99.), [0., 0., -.4])['valid'])
        both_short = np.vstack((short, short*np.array([1., -1., 1.])))
        self.assertFalse(self.model.process(both_short, dict(self.ego, x=99.), [0., 0., -.4])['valid'])

    def test_recent_wall_projection_preserves_every_obstacle_and_wall_return(self):
        obstacles = np.array([[12., 0., .1], [12., 3.8, .1], [12., 3.96, .1]])
        result = self.model.project_scan(np.vstack((self.right, obstacles)), self.ego,
                                         [0., 0., -.4], {'right_n': 100})
        self.assertTrue(result['valid'])
        self.assertIsNone(result['constraint'])
        self.assertTrue(result['wall_stats']['propagated'])
        for point in obstacles:
            self.assertTrue(np.any(np.linalg.norm(result['obstacle_points']-point[:2], axis=1) < 1e-6))
        self.assertGreater(len(result['obstacle_points']), len(obstacles))

    def test_planner_paths_with_placeholder_third_coordinate_are_supported(self):
        path = np.column_stack((np.arange(0., 20.1, .5), np.zeros((41, 2))))
        self.assertEqual(self.model.path_clearance(path, self.ego, 20.), 20.)

    def test_load_rejects_nonfinite_reversed_or_unsafe_map_bounds(self):
        for patch in (dict(axis_heading_deg=float('nan')), dict(left_offset_m=-5.),
                      dict(longitudinal_max_m=-1.), dict(wall_filter_tolerance_m=.5)):
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'map.json'
                path.write_text(json.dumps(dict(self.payload, **patch)))
                with self.assertRaises(ValueError):
                    TunnelMap.load(path)

    def test_independent_stationary_capture_matches_and_preserves_injected_obstacle(self):
        from build_tunnel_map import capture_frames
        root = Path(__file__).resolve().parents[2]
        capture = root/'data/autodrive/tunnel_scans/entry_20261003_005222/raw_scans.npz'
        model = TunnelMap.load(Path(__file__).parent/'tunnel_wall_map.json', survey_frame=True)
        _, points, ego, plane = next(capture_frames(capture))
        obstacle = np.array([[12., 0., plane[2]+.45], [12., 3., plane[2]+.45]])
        result = model.process(np.vstack((points, obstacle)), ego, plane)
        self.assertTrue(result['valid'], result['reason'])
        for point in obstacle:
            self.assertTrue(np.any(np.linalg.norm(result['obstacle_points']-point[:2], axis=1) < 1e-6))

    def test_recorded_exit_uses_rear_wall_support_without_relaxing_match_gates(self):
        from build_tunnel_map import capture_frames
        root = Path(__file__).resolve().parents[2]
        capture = root/'data/autodrive/follow_runs/follow-20261002T152952Z-1309248/tunnel_analysis/tunnel_scans.npz'
        model = TunnelMap.load(Path(__file__).parent/'tunnel_wall_map.json', survey_frame=True)
        for scan, points, ego, plane in capture_frames(capture, scan_indices=range(211, 215)):
            result = model.process(points, ego, plane)
            self.assertTrue(result['valid'], (scan, result['reason']))
            self.assertGreaterEqual(result['wall_stats']['left_span_m'], 14.)
            self.assertGreaterEqual(result['wall_stats']['right_span_m'], 14.)


if __name__ == '__main__':
    unittest.main()
