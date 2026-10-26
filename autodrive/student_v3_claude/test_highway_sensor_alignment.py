"""Sensor-time alignment keeps moving echoes out of static highway hazards."""
import unittest

import numpy as np

from highway_sensor_alignment import align_measured_cars, align_moving_observations, align_scan_points
from lidar_corridor import without_moving_objects
from planning.highway.object_policy import highway_measured_objects


class HighwaySensorAlignmentTests(unittest.TestCase):
    def car(self, **changes):
        row = [10., 0., 4.793, 'lidar_car', 'npc', 17.13, 4.793, 2.071,
               0., 17.13, 0., 500037, 0., None, True, 0.]
        for key, value in changes.items():
            row[int(key)] = value
        return tuple(row)

    def test_recorded_delay_masks_vehicle_front_without_masking_nearby_static_point(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        car = self.car()
        # Run 984404: the car front appeared 3.355 m ahead of a stale centre,
        # outside length/2+.4. A 49 ms scan difference moves it 0.83937 m.
        points = np.array([[13.355, -.772], [13.355, 1.6], [5., 0.]])
        np.testing.assert_array_equal(without_moving_objects(points, [car], ego), points)
        scan_cars, info = align_measured_cars([car], 100., 100.049)
        self.assertTrue(info['timing_valid'])
        self.assertEqual(info['aligned_count'], 1)
        self.assertAlmostEqual(scan_cars[0][0], 10.+.049*17.13)
        self.assertEqual(scan_cars[0][2:], car[2:])
        np.testing.assert_array_equal(without_moving_objects(points, scan_cars, ego), points[1:])
        self.assertEqual(car[0], 10.)

    def test_scan_mask_and_planning_forecast_have_separate_target_times(self):
        car = self.car()
        scan, _ = align_measured_cars([car], 100., 100.049)
        plan, _ = align_measured_cars([car], 100., 100.12)
        self.assertAlmostEqual(scan[0][0], 10.+.049*17.13)
        self.assertAlmostEqual(plan[0][0], 10.+.12*17.13)
        self.assertEqual(scan[0][6:8], plan[0][6:8])

    def test_negative_delta_rewinds_observed_motion(self):
        car = self.car()
        aligned, info = align_measured_cars([car], 100.049, 100.)
        self.assertTrue(info['timing_valid'])
        self.assertAlmostEqual(aligned[0][0], 10.-.049*17.13)

    def test_moving_fusion_npc_and_person_rewind_to_scan_time(self):
        npc = (10., 2., 4.6, 'fusion_npc', 'npc', 8., 4.6, 1.9,
               0., 8., 0., 28, 0., None)
        person = (5., 1., .8, 'fusion_pedestrian', 'pedestrian', 1., .8, .6,
                  90., 0., 1., 29, 0., None)
        scan, info = align_moving_observations([npc, person], 100.1, 100.)
        self.assertEqual(info['aligned_count'], 2)
        self.assertAlmostEqual(scan[0][0], 9.2)
        self.assertAlmostEqual(scan[1][1], .9)
        self.assertEqual(scan[0][2:], npc[2:])
        self.assertEqual(scan[1][2:], person[2:])
        # The LiDAR-only helper never treats fusion motion as a LiDAR measurement.
        cars, cars_info = align_measured_cars([npc, person], 100.1, 100.)
        self.assertEqual(cars, [npc, person])
        self.assertEqual(cars_info['aligned_count'], 0)

    def test_fusion_unknown_obstacle_and_invalid_motion_never_gain_mask_ownership(self):
        npc = [10., 2., 4.6, 'fusion_npc', 'npc', 8., 4.6, 1.9,
               0., 8., 0., 28, 0., None]
        cases = []
        for field, value in [(3, 'fusion_parked_vehicle'), (4, 'obstacle'),
                             (5, 0.), (9, 0.), (9, float('nan')),
                             (6, float('inf'))]:
            row = list(npc)
            row[field] = value
            # A parked fusion vehicle is normally also kind obstacle.
            if field == 3:
                row[4], row[5], row[9] = 'obstacle', 0., 0.
            cases.append(tuple(row))
        aligned, info = align_moving_observations(cases, 100., 100.1)
        self.assertEqual(info['aligned_count'], 0)
        self.assertTrue(all(a is b for a, b in zip(aligned, cases)))

    def test_fusion_alignment_has_the_same_stale_time_bound(self):
        npc = (10., 2., 4.6, 'fusion_npc', 'npc', 8., 4.6, 1.9,
               0., 8., 0., 28, 0., None)
        aligned, info = align_moving_observations([npc], 100., 100.351)
        self.assertIs(aligned[0], npc)
        self.assertFalse(info['timing_valid'])

    def test_unknown_predicted_and_expired_cars_are_unchanged_and_cannot_mask(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        points = np.array([[10., 0.]])
        cases = [self.car(**{'14': False}), self.car(**{'15': .01}),
                 self.car(**{'15': 1.1}), self.car(**{'15': -.1}),
                 self.car(**{'15': float('nan')}), self.car()[:15]]
        for car in cases:
            with self.subTest(car=car):
                aligned, info = align_measured_cars([car], 100., 100.049)
                self.assertIs(aligned[0], car)
                self.assertEqual(info['aligned_count'], 0)
                # Older tuple formats are preserved but remain subject to the
                # existing measured-object ownership policy, not this helper.
                if len(car) > 15:
                    np.testing.assert_array_equal(without_moving_objects(
                        points, highway_measured_objects(aligned), ego), points)

    def test_nonfinite_or_invalid_body_is_preserved(self):
        for field, value in [('0', float('nan')), ('1', float('inf')),
                             ('2', float('nan')), ('5', float('nan')), ('6', 0.), ('7', float('nan')),
                             ('8', float('inf')), ('9', float('nan')), ('10', float('inf')),
                             ('14', float('nan'))]:
            car = self.car(**{field: value})
            aligned, info = align_measured_cars([car], 100., 100.049)
            self.assertIs(aligned[0], car)
            self.assertEqual(info['aligned_count'], 0)

    def test_invalid_or_old_timestamps_do_not_move_any_body(self):
        car = self.car()
        for source, target, bound in [(None, 100., .35), (0., 100., .35),
                                      (100., float('nan'), .35),
                                      (100., 100.351, .35), (100.351, 100., .35),
                                      (100., 100.049, .01), (100., 100.5, 1.),
                                      (100., 100., -1.)]:
            with self.subTest(source=source, target=target, bound=bound):
                aligned, info = align_measured_cars([car], source, target, bound)
                self.assertIs(aligned[0], car)
                self.assertFalse(info['timing_valid'])
                self.assertEqual(info['aligned_count'], 0)

    def test_scan_pose_translation_rotation_preserves_world_geometry(self):
        scan_pose = dict(x=20., y=30., heading_deg=90.)
        planning_pose = dict(x=22., y=28., heading_deg=0.)
        points = np.array([[4., 1., .8], [0., 0., .4]])
        aligned = align_scan_points(points, scan_pose, planning_pose)
        np.testing.assert_allclose(aligned, [[-3., 6., .8], [-2., 2., .4]], atol=1e-12)
        np.testing.assert_array_equal(points, [[4., 1., .8], [0., 0., .4]])
        np.testing.assert_allclose(align_scan_points(aligned, planning_pose, scan_pose),
                                   points, atol=1e-12)

    def test_empty_scan_and_missing_scan_do_not_invent_returns(self):
        pose = dict(x=1., y=2., heading_deg=45.)
        self.assertIsNone(align_scan_points(None, pose, pose))
        self.assertEqual(align_scan_points(np.empty((0, 2)), pose, pose).shape, (0, 2))

    def test_invalid_pose_or_point_shape_is_explicit(self):
        pose = dict(x=0., y=0., heading_deg=0.)
        for invalid in [None, {}, dict(pose, x=float('nan'))]:
            with self.assertRaises(ValueError):
                align_scan_points(np.array([[1., 2.]]), invalid, pose)
        with self.assertRaises(ValueError):
            align_scan_points(np.array([1., 2.]), pose, pose)


if __name__ == '__main__':
    unittest.main()
