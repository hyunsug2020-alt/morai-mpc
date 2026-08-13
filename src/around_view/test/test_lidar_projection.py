#!/usr/bin/env python3

import unittest

import numpy as np

from around_view.lidar_projection import (
    camera_intrinsic_matrix,
    morai_rotation_matrix,
    project_lidar_points,
    transform_lidar_points_to_vehicle,
)
from around_view.projection import (
    auto_homography,
    vehicle_points_to_canvas,
    warp_image_points,
)


class LidarProjectionTest(unittest.TestCase):
    def setUp(self):
        self.lidar = {
            "position": [3.32, 0.0, 0.66],
            "rotation": [0.0, 0.0, 0.0],
        }
        self.front = {
            "position": [1.9, 0.0, 1.2],
            "rotation": [0.0, 2.0, 0.0],
            "width": 1280,
            "height": 720,
            "fov": 90.0,
        }

    def test_intrinsic_uses_horizontal_fov(self):
        intrinsic = camera_intrinsic_matrix(1280, 720, 90.0)
        self.assertAlmostEqual(intrinsic[0, 0], 640.0)
        self.assertAlmostEqual(intrinsic[1, 1], 640.0)
        self.assertAlmostEqual(intrinsic[0, 2], 640.0)
        self.assertAlmostEqual(intrinsic[1, 2], 360.0)

    def test_zero_rotation_is_identity(self):
        np.testing.assert_allclose(
            morai_rotation_matrix([0.0, 0.0, 0.0]), np.eye(3), atol=1e-12
        )

    def test_forward_lidar_point_projects_into_front_image(self):
        pixels, depth, indices = project_lidar_points(
            np.array([[10.0, 0.0, 0.0]]),
            self.lidar,
            self.front,
        )
        self.assertEqual(pixels.shape, (1, 2))
        self.assertEqual(indices.tolist(), [0])
        self.assertGreater(depth[0], 10.0)
        self.assertAlmostEqual(pixels[0, 0], 640.0, places=5)
        self.assertGreater(pixels[0, 1], 360.0)

    def test_point_behind_camera_is_rejected(self):
        pixels, depth, indices = project_lidar_points(
            np.array([[-10.0, 0.0, 0.0]]),
            self.lidar,
            self.front,
        )
        self.assertEqual(pixels.size, 0)
        self.assertEqual(depth.size, 0)
        self.assertEqual(indices.size, 0)

    def test_lidar_points_transform_into_vehicle_frame(self):
        transformed = transform_lidar_points_to_vehicle(
            np.array([[1.0, 2.0, 3.0]]),
            {
                "position": [10.0, -2.0, 0.5],
                "rotation": [0.0, 0.0, 0.0],
            },
            correction={
                "translation": [0.5, 0.25, -0.25],
                "rotation": [0.0, 0.0, 0.0],
            },
        )
        np.testing.assert_allclose(
            transformed,
            np.array([[11.5, 0.25, 3.25]]),
            atol=1e-12,
        )

    def test_warp_image_points_transforms_centres_only(self):
        homography = np.array(
            [
                [2.0, 0.0, 10.0],
                [0.0, 3.0, -5.0],
                [0.0, 0.0, 1.0],
            ]
        )
        warped = warp_image_points(
            np.array([[1.0, 2.0], [4.0, 5.0]]),
            homography,
        )
        np.testing.assert_allclose(
            warped,
            np.array([[12.0, 1.0], [18.0, 10.0]]),
            atol=1e-6,
        )

    def test_ground_lidar_point_matches_avm_ground_coordinates(self):
        destinations = [[280, 50], [520, 50], [520, 290], [280, 290]]
        ground_projection = {
            "pixels_per_meter": 40.0,
            "ego_center": [400, 450],
        }
        homography = auto_homography(
            "front",
            self.front,
            {
                "front_pitch_tenths": 0,
                "left_yaw_tenths": 0,
                "right_yaw_tenths": 0,
                "left_pitch_tenths": 0,
                "right_pitch_tenths": 0,
                "global_height_cm": 0,
            },
            self.front["width"],
            self.front["height"],
            destinations,
            ground_projection,
        )

        vehicle_ground_point = np.array([8.0, 1.0, 0.0])
        lidar_point = vehicle_ground_point - np.asarray(
            self.lidar["position"]
        )
        pixels, _depths, _indices = project_lidar_points(
            lidar_point.reshape((1, 3)),
            self.lidar,
            self.front,
        )
        canvas_point = warp_image_points(pixels, homography)[0]
        expected = np.array(
            [
                400.0 - vehicle_ground_point[1] * 40.0,
                450.0 - vehicle_ground_point[0] * 40.0,
            ]
        )
        np.testing.assert_allclose(canvas_point, expected, atol=1e-4)

    def test_vehicle_points_map_directly_to_avm_canvas(self):
        canvas_points = vehicle_points_to_canvas(
            np.array(
                [
                    [0.0, 0.0, 0.0],
                    [5.0, 2.0, 1.0],
                    [-1.0, -3.0, 2.0],
                ]
            ),
            {
                "pixels_per_meter": 40.0,
                "ego_center": [400, 450],
            },
        )
        np.testing.assert_allclose(
            canvas_points,
            np.array(
                [
                    [400.0, 450.0],
                    [320.0, 250.0],
                    [520.0, 490.0],
                ]
            ),
            atol=1e-12,
        )


if __name__ == "__main__":
    unittest.main()
