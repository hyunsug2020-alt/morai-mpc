#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest


SCRIPT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "scripts", "lio_odometry_mux.py"))
SPEC = importlib.util.spec_from_file_location("lio_odometry_mux", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class LioOdometryMuxCovarianceTest(unittest.TestCase):
    def test_planar_covariance_rotates_and_inflates(self):
        mux = MODULE.LioOdometryMux.__new__(MODULE.LioOdometryMux)
        mux.alignment_yaw = math.pi / 2.0
        mux.primary_covariance_scale = 4.0
        mux.primary_position_variance = 0.25
        mux.primary_yaw_variance = 0.01
        mux.alignment_position_variance = 0.0
        mux.alignment_yaw_variance = 0.0
        covariance = [0.0] * 36
        covariance[0] = 1.0
        covariance[7] = 4.0
        covariance[35] = 0.09

        output = mux.transformed_covariance("eskf", covariance)

        self.assertEqual(len(output), 36)
        self.assertAlmostEqual(output[0], 16.25, places=9)
        self.assertAlmostEqual(output[7], 4.25, places=9)
        self.assertAlmostEqual(output[35], 0.37, places=9)


if __name__ == "__main__":
    unittest.main()
