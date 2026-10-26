"""Full-body activation remains invariant to the sensor's world frame."""
import math
import unittest

import numpy as np

import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M


class StaticBodyHazardTests(unittest.TestCase):
    def check(self, forward, lateral, *, heading=0., origin=(0., 0.)):
        angle = math.radians(heading)
        axes = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
        world = np.array([forward, lateral]) @ axes.T+origin
        obj = (*world, .2, 'measured_static', 'obstacle', 0., .2, .2, heading)
        ego = dict(x=origin[0], y=origin[1], heading_deg=heading, speed_kmh=20.)
        route = np.column_stack((np.arange(0., 50.5, .5), np.zeros(101)))
        return ma.footprint_hazards(route, ego, [obj], body_shift_m=REAR_AXLE_SHIFT_M)

    def test_a_centre_beside_the_axle_cannot_hide_a_measured_body_overlap(self):
        for heading, origin in ((0., (0., 0.)), (37., (300., -50.)), (-112., (-30., 70.))):
            with self.subTest(heading=heading):
                hazards = self.check(.5, 1., heading=heading, origin=origin)
                self.assertEqual(len(hazards), 1)

    def test_passed_rear_and_clear_side_bodies_do_not_create_false_hazards(self):
        self.assertEqual(self.check(-4., 0.), [])
        self.assertEqual(self.check(.5, 3.), [])
        self.assertEqual(len(self.check(20., 0.)), 1)


if __name__ == '__main__':
    unittest.main()
