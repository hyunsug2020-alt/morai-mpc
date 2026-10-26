"""Optimized time-pair geometry must preserve the full SAT collision result."""
import unittest

import numpy as np

from traffic_planner import overlaps


class TrafficPairMaskTests(unittest.TestCase):
    def test_sparse_time_pairs_match_full_geometry_for_rotated_vehicles(self):
        rng = np.random.default_rng(732)
        for _ in range(30):
            a, b = rng.normal(0., 7., (2, 41, 2))
            ay, by = rng.uniform(-np.pi, np.pi, (2, 41))
            mask = rng.random((41, 41)) < .3
            length, width = rng.uniform(2., 10.), rng.uniform(1., 3.)
            expected = overlaps(a, ay, b, by, length, width) & mask
            actual = overlaps(a, ay, b, by, length, width, pair_mask=mask)
            np.testing.assert_array_equal(actual, expected)

    def test_empty_pair_mask_never_claims_a_collision(self):
        a = np.zeros((3, 2)); yaw = np.zeros(3)
        result = overlaps(a, yaw, a, yaw, 5., 2., pair_mask=np.zeros((3, 3), bool))
        self.assertFalse(result.any())


if __name__ == "__main__":
    unittest.main()
