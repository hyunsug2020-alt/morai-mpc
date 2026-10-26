"""Measured-clock alignment and the archived moving cushion false stop."""
import json
import math
from pathlib import Path
import unittest

import numpy as np

from perception.source_pose import align_cloud_to_pose, measured_pose_at
from perception.roadside_structures import RoadsideStructures
from semantic_fusion import filter_infrastructure_points
from roundabout_empty import semantic_entry_clearance


ROOT = Path(__file__).resolve().parent


class SensorPoseAlignmentTests(unittest.TestCase):
    def test_uses_observed_displacement_despite_native_velocity(self):
        poses = [[10., 0., 0., 0., 20., 0.], [10.1, 1., 0., 0., 20., 0.]]
        points = np.array([[5., 1., 2.], [9., -2., .4]], np.float32)
        saved = points.copy()
        result = align_cloud_to_pose(points, 10.1, poses[0], poses)
        np.testing.assert_allclose(result, [[6., 1., 2.], [10., -2., .4]])
        np.testing.assert_array_equal(points, saved)
        self.assertEqual(result.dtype, points.dtype)

    def test_world_position_survives_translation_and_rotation(self):
        source = [10., 2., 3., math.pi/2., 2., 0.]
        target = [10.1, 2.1, 3., math.pi/2.+.1, 2., 0.]
        point = np.array([[1., 2., 3.]])
        actual = align_cloud_to_pose(point, 10., target, [source, target])
        yaw = target[3]
        expected = np.array([[-2.1, 1.]]) @ np.array([[math.cos(yaw), -math.sin(yaw)],
                                                    [math.sin(yaw), math.cos(yaw)]])
        np.testing.assert_allclose(actual[:, :2], expected, atol=1e-12)
        self.assertEqual(actual[0, 2], 3.)

    def test_wraps_heading_when_interpolating(self):
        poses = [[10., 0., 0., math.radians(179), 2., 0.],
                 [10.1, .1, 0., math.radians(-179), 2., 0.]]
        pose = measured_pose_at(poses, 10.05)
        self.assertAlmostEqual(pose[1], .05)
        self.assertAlmostEqual(abs(pose[3]), math.pi)

    def test_causal_nearest_is_measured_and_has_a_strict_age_bound(self):
        pose = [10., 0., 0., 0., 2., 0.]
        np.testing.assert_array_equal(measured_pose_at([pose], 10.02), pose)
        self.assertIsNone(measured_pose_at([pose], 10.031))
        self.assertIsNone(measured_pose_at([pose], 9.969))

    def test_latest_small_eskf_revision_is_used_without_old_conflict_poisoning(self):
        old = [9., 0., 0., 0., 2., 0.]
        conflict = [9., 2., 0., 0., 2., 0.]
        first = [10., 1., 0., 0., 2., 0.]
        latest = [10., 1., 0., .00001, 1.99, -.1]
        np.testing.assert_array_equal(measured_pose_at([old, conflict, first, latest], 10.), latest)
        self.assertIsNone(measured_pose_at([old, conflict, first, latest], 9.))

    def test_target_revisions_are_checked_without_changing_the_callers_frame(self):
        source = [10., 0., 0., 0., 1., 0.]
        target = [10.1, .1, 0., 0., 1., 0.]
        points = np.array([[5., 1., .5]])
        self.assertIsNone(align_cloud_to_pose(points, 10., target,
            [source, target, [10.1, 1., 0., 0., 1., 0.]]))
        revised = [10.1, .1, 0., .001, 1.01, .01]
        result = align_cloud_to_pose(points, 10., target, [source, target, revised])
        np.testing.assert_allclose(result, [[4.9, 1., .5]])

    def test_missing_nonfinite_duplicate_or_discontinuous_history_fails_closed(self):
        p = [10., 0., 0., 0., 1., 0.]
        np.testing.assert_array_equal(measured_pose_at([p, p], 10.), p)
        for history in ([], [p, [10., .1, 0., 0., 1., 0.]], [[10., math.nan, 0., 0., 1., 0.]],
                        [p, [10.1, 8., 0., 0., 1., 0.]],
                        [p, [10.1, 0., 0., 2., 1., 0.]],
                        [p, [10.5, 0., 0., 0., 1., 0.]]):
            with self.subTest(history=history):
                self.assertIsNone(measured_pose_at(history, 10.01))
        self.assertIsNone(align_cloud_to_pose(np.ones((4, 3)), 10., p, []))
        self.assertIsNone(align_cloud_to_pose(np.full((4, 3), math.nan), 10., p, [p]))
        self.assertIsNone(align_cloud_to_pose(np.ones((4, 3), int), 10., p, [p]))
        for target in ([10.01, 8., 0., 0., 1., 0.], [10.01, 0., 0., 2., 1., 0.]):
            self.assertIsNone(align_cloud_to_pose(np.ones((4, 3)), 10., target, [p]))

    def test_unaligned_frame_cannot_exclude_returns(self):
        points = np.ones((10, 3))
        class ForbiddenMap:
            def local_point_mask(self, *args):
                raise AssertionError('uncertified alignment used for exclusion')
        kept, lines, mapped = filter_infrastructure_points(points, 'obstacle',
            (10., 0., 0., 0., 0., 0.), ForbiddenMap(),
            lambda p: np.ones(len(p), bool), aligned=False)
        np.testing.assert_array_equal(kept, points)
        self.assertEqual((lines, mapped), (0, 0))

    def test_archived_false_stop_retains_unknown_points_without_zero_clearance(self):
        # Native49 plans389/390: source support only, without archived segmentation.
        # The real camera/native integration is checked separately in MORAI.
        with np.load(ROOT/'test_data/roundabout_sensor_alignment_20261014.npz') as data:
            points, target = data['points'], data['target_pose']
            aligned = align_cloud_to_pose(points, float(data['cloud_stamp'][0]),
                                          target, data['causal_history'])
            self.assertIsNotNone(aligned)
            self.assertEqual(len(aligned), 319)
            route = np.array([[p['x'], p['y']] for p in
                              json.loads((ROOT/'global_path.json').read_text())['waypoints']])
            survey = RoadsideStructures(route)
            kept, _, mapped = filter_infrastructure_points(aligned, 'obstacle',
                target, survey, lambda p: np.zeros(len(p), bool))
            self.assertEqual((len(kept), mapped), (16, 303))
            # All 16 unregistered returns remain inside the same padded 1.1m box.
            low, high = kept[:, :2].min(0), kept[:, :2].max(0)
            local = (low+high)/2.
            c, s = math.cos(target[3]), math.sin(target[3])
            centre = local @ np.array([[c, s], [-s, c]])+target[1:3]
            size = np.maximum(high-low, .8)+.3
            row = (*centre, max(size), 'fusion_obstacle', 'obstacle', 0.,
                   *size, math.degrees(target[3]), 0., 0., 18)
            x, y, heading, speed = data['control_ego']
            ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed)
            angle = math.radians(heading)
            rotation = np.array([[math.cos(angle), -math.sin(angle)],
                                 [math.sin(angle), math.cos(angle)]])
            path = (data['world_path']-[x, y]) @ rotation
            path = path[int(np.argmin(np.linalg.norm(path, axis=1))):]
            bx, by, length, width, yaw = data['original_box']
            old = (bx, by, max(length, width), 'fusion_obstacle', 'obstacle',
                   0., length, width, yaw, 0., 0., 18)
            self.assertEqual(semantic_entry_clearance(path, ego, [old]), 0.)
            self.assertGreater(semantic_entry_clearance(path, ego, [row]), 50.)
            np.testing.assert_allclose(size, [1.1, 1.1])


if __name__ == '__main__':
    unittest.main()
