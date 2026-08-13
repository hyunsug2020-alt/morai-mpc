#!/usr/bin/env python3

import unittest

import numpy as np

from around_view.projection import (
    auto_homography,
    feather_mask,
    ground_to_camera_pixel,
    source_ground_mask,
)


CAMERAS = {
    "front": {
        "position": [1.9, 0.0, 1.2],
        "rotation": [0.0, 2.0, 0.0],
        "fov": 90.0,
    },
    "left": {
        "position": [1.15, 0.65, 1.2],
        "rotation": [0.0, 10.0, 70.0],
        "fov": 130.0,
    },
    "right": {
        "position": [1.15, -0.65, 1.2],
        "rotation": [0.0, 10.0, 290.0],
        "fov": 130.0,
    },
}

OFFSETS = {
    "front_pitch_tenths": 0,
    "left_yaw_tenths": 0,
    "right_yaw_tenths": 0,
    "left_pitch_tenths": 0,
    "right_pitch_tenths": 0,
    "global_height_cm": 0,
}

DESTINATIONS = {
    "front": [[280, 50], [520, 50], [520, 290], [280, 290]],
    "left": [[160, 50], [340, 50], [340, 490], [160, 490]],
    "right": [[460, 50], [640, 50], [640, 490], [460, 490]],
}

GROUND_PROJECTION = {
    "pixels_per_meter": 40.0,
    "ego_center": [400, 450],
}


class AutoProjectionTest(unittest.TestCase):
    def test_front_ground_point_projects_to_finite_pixel(self):
        pixel = ground_to_camera_pixel(
            "front",
            8.0,
            0.0,
            CAMERAS["front"],
            OFFSETS,
            1280,
            720,
        )
        self.assertIsNotNone(pixel)
        self.assertTrue(np.all(np.isfinite(pixel)))

    def test_original_camera_values_make_homographies(self):
        for name in ("front", "left", "right"):
            matrix = auto_homography(
                name,
                CAMERAS[name],
                OFFSETS,
                1280,
                720,
                DESTINATIONS[name],
                GROUND_PROJECTION,
            )
            self.assertIsNotNone(matrix, name)
            self.assertEqual(matrix.shape, (3, 3))
            self.assertTrue(np.all(np.isfinite(matrix)), name)

    def test_source_ground_mask_excludes_body_polygon(self):
        mask = source_ground_mask(
            100,
            80,
            {
                "include": [
                    [0.0, 0.4],
                    [1.0, 0.4],
                    [1.0, 1.0],
                    [0.0, 1.0],
                ],
                "exclude": [
                    [
                        [0.6, 0.7],
                        [1.0, 0.7],
                        [1.0, 1.0],
                        [0.6, 1.0],
                    ]
                ],
            },
        )
        self.assertEqual(mask.shape, (80, 100))
        self.assertEqual(mask[10, 10], 0)
        self.assertEqual(mask[50, 20], 255)
        self.assertEqual(mask[70, 80], 0)

    def test_feather_mask_softens_polygon_boundary(self):
        mask = np.zeros((80, 100), dtype=np.uint8)
        mask[:, :50] = 255
        weight = feather_mask(mask, 6)
        self.assertEqual(weight.shape, mask.shape)
        self.assertGreater(weight[40, 45], weight[40, 55])
        self.assertGreater(weight[40, 55], 0.0)
        self.assertLess(weight[40, 45], 1.0)


if __name__ == "__main__":
    unittest.main()
