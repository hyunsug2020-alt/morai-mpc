#!/usr/bin/env python3

import math
import unittest

import numpy as np

from camera_vehicle_training.geometry import (
    make_vehicle_box_corners,
    projected_box,
    world_object_to_vehicle,
    xyxy_to_yolo,
)


CAMERA = {
    "position": [1.9, 0.0, 1.2],
    "rotation": [0.0, 2.0, 0.0],
    "width": 1280,
    "height": 720,
    "fov": 90.0,
    "minimum_depth": 0.5,
}


class GeometryTest(unittest.TestCase):
    def test_world_to_vehicle_at_zero_heading(self):
        center, size, yaw = world_object_to_vehicle(
            [12.0, 3.0, 0.0], 0.0, [4.0, 2.0, 1.5],
            [2.0, 1.0, 0.0], 0.0, True,
        )
        np.testing.assert_allclose(center, [10.0, 2.0, 0.75])
        np.testing.assert_allclose(size, [4.0, 2.0, 1.5])
        self.assertAlmostEqual(yaw, 0.0)

    def test_world_to_vehicle_rotates_with_ego(self):
        center, _size, yaw = world_object_to_vehicle(
            [0.0, 10.0, 0.0], 90.0, [4.0, 2.0, 2.0],
            [0.0, 0.0, 0.0], 90.0, True,
        )
        np.testing.assert_allclose(center, [10.0, 0.0, 1.0], atol=1.0e-9)
        self.assertAlmostEqual(yaw, 0.0)

    def test_front_vehicle_projects_inside_image(self):
        corners = make_vehicle_box_corners([15.0, 0.0, 0.75], [4.9, 2.1, 1.5], 0.0)
        result = projected_box(corners, CAMERA, 0.3)
        self.assertIsNotNone(result)
        box, ratio = result
        self.assertGreater(box[2] - box[0], 12.0)
        self.assertGreater(box[3] - box[1], 12.0)
        self.assertAlmostEqual(ratio, 1.0)
        yolo = xyxy_to_yolo(box, 1280, 720)
        self.assertTrue(all(0.0 <= value <= 1.0 for value in yolo))

    def test_vehicle_behind_camera_is_rejected(self):
        corners = make_vehicle_box_corners([-10.0, 0.0, 0.75], [4.9, 2.1, 1.5], math.pi)
        self.assertIsNone(projected_box(corners, CAMERA, 0.3))


if __name__ == "__main__":
    unittest.main()
