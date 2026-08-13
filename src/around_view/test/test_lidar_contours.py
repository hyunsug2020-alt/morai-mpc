#!/usr/bin/env python3

import unittest

import numpy as np

from around_view.lidar_contours import build_object_contour_mask


class LidarContourTest(unittest.TestCase):
    def setUp(self):
        self.ground_projection = {
            "pixels_per_meter": 20.0,
            "ego_center": [100, 150],
        }

    def test_isolated_points_are_removed_after_dense_input(self):
        cluster = np.asarray(
            [
                [4.0, -0.20, 0.30],
                [4.0, -0.10, 0.60],
                [4.0, 0.00, 0.90],
                [4.0, 0.10, 1.20],
                [4.0, 0.20, 1.50],
                [4.1, 0.00, 0.70],
            ]
        )
        isolated = np.asarray([[1.0, 3.0, 1.0]])
        mask, contour_count, retained_count = build_object_contour_mask(
            np.vstack((cluster, isolated)),
            self.ground_projection,
            (200, 200),
            outlier_radius=0.25,
            outlier_minimum_neighbors=3,
            cluster_connection=0.30,
            cluster_minimum_points=4,
            fit_vehicle_boxes=False,
        )
        self.assertEqual(contour_count, 1)
        self.assertEqual(retained_count, len(cluster))
        self.assertGreater(np.count_nonzero(mask), 0)
        # The isolated point maps to x=40, y=130 and must not be outlined.
        self.assertEqual(mask[130, 40], 0)

    def test_vehicle_surface_is_completed_as_oriented_footprint(self):
        points = []
        for lateral in np.linspace(-0.8, 0.8, 9):
            for height in (0.25, 0.85, 1.45):
                points.append([5.0, lateral, height])

        mask, contour_count, retained_count = build_object_contour_mask(
            np.asarray(points),
            self.ground_projection,
            (200, 200),
            outlier_radius=0.25,
            outlier_minimum_neighbors=1,
            cluster_connection=0.30,
            cluster_minimum_points=6,
            fit_vehicle_boxes=True,
            vehicle_minimum_points=12,
            vehicle_minimum_height_span=0.35,
            vehicle_length=4.5,
            vehicle_width=1.8,
        )
        self.assertEqual(contour_count, 1)
        self.assertEqual(retained_count, len(points))
        rows = np.flatnonzero(np.any(mask > 0, axis=1))
        columns = np.flatnonzero(np.any(mask > 0, axis=0))
        self.assertGreater(rows[-1] - rows[0], 80)
        self.assertGreater(columns[-1] - columns[0], 30)

    def test_range_and_height_filters_reject_irrelevant_points(self):
        points = np.asarray(
            [
                [0.2, 0.0, 1.0],
                [4.0, 0.0, 0.05],
                [4.0, 0.0, 3.5],
                [30.0, 0.0, 1.0],
            ]
        )
        mask, contour_count, retained_count = build_object_contour_mask(
            points,
            self.ground_projection,
            (200, 200),
            minimum_range=0.8,
            maximum_range=25.0,
            minimum_height=0.15,
            maximum_height=2.8,
            outlier_minimum_neighbors=1,
        )
        self.assertEqual(contour_count, 0)
        self.assertEqual(retained_count, 0)
        self.assertEqual(np.count_nonzero(mask), 0)


if __name__ == "__main__":
    unittest.main()
