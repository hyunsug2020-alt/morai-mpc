"""A recent predicted mover keeps motion without owning measured hazards."""
import unittest

import numpy as np

from gui import highway_objects, highway_static_objects, highway_measured_objects, static_clear_distance
from highway_planner import timed_gap
from lidar_corridor import without_moving_objects
from lidar_vehicle_detection import TRACK_KEEP_S, VehicleTracks
from planning.highway.object_policy import (MOTION_TRACK_KEEP_S, moving_car_owns_obstacle_fragment,
                                            ego_body_penetration_m, highway_scan_mask_objects,
                                            without_far_moving_returns, moving_car_owns_parked_duplicate,
                                            highway_tentative_objects)
from highway_planner import local_points, lane_change_path
from highway_sensor_alignment import align_moving_observations


class HighwayObjectPolicyTests(unittest.TestCase):
    def car(self, prediction_age=.280285):
        return (30., 0., 4.9, 'lidar_car', 'npc', 13.24, 4.9, 2.1,
                0., 13.24, 0., 500010, 0., None, True, prediction_age)

    def fragment(self, x=30., y=0., length=3., width=1.1, heading=0., speed=0.):
        return (x, y, max(length, width), 'fusion_obstacle', 'obstacle', speed,
                length, width, heading, speed, 0., 65, 0., None)

    def test_fully_contained_fragment_is_owned_by_current_measured_moving_body(self):
        car, fragment = self.car(0.), self.fragment()
        self.assertTrue(moving_car_owns_obstacle_fragment(fragment, car))
        objects = highway_objects([fragment], [car])
        self.assertEqual(objects, [car])
        self.assertEqual(highway_static_objects(objects), [])
        # Ownership does not enlarge the moving mask to erase a nearby real return.
        points = np.array([[30., 0.], [30., 1.5]])
        ego = dict(x=0., y=0., heading_deg=0.)
        np.testing.assert_array_equal(without_moving_objects(points, objects, ego), points[1:])

    def test_actual_run1029851_fragment65_on_the_car_side_is_owned(self):
        # Centre 0.88 m from the axis of a 2.03 m wide 39 km/h car, protruding
        # 0.42 m; it moved with the car and stopped the ego at 16 km/h.
        car = (71.96337632105815, -168.5547654928365, 4.851771354675293,
               'lidar_car', 'npc', 10.964449014261334, 4.851771354675293,
               2.0294671058654785, 89.74032151805297, .3474810446212826,
               -10.95894151412285, 500014, 0., None, True, 0.)
        fragment = self.fragment(71.08469, -168.19658, 3.008, 1.1, -85.196)
        self.assertTrue(moving_car_owns_obstacle_fragment(fragment, car))
        self.assertEqual(highway_objects([fragment], [car]), [car])

    def test_recorded_rear_face_fragment_of_slow_car_is_not_a_stop_line(self):
        # Run follow-20261009T191227Z t=30.85: camera fragment 93 on the rear of
        # a 33 km/h car 39 m ahead became a 36.75 m static hit (63 -> 25 km/h).
        car = (66.02613334555043, 137.63783613194434, 5.040555953979492, "lidar_car", "npc",
               9.033477102564747, 5.040555953979492, 1.9125640392303467, 269.3743526426863,
               0.04368109839831998, -9.033371492649037, 500018, 0.0, None, True, 0.0)
        fragment = (66.49601844281983, 139.55663312100015, 1.948319435119629, "fusion_obstacle",
                    "obstacle", 0.0, 1.100000023841858, 1.948319435119629, -90.185763356665,
                    -0.0, -0.0, 93, 0.0, None)
        ego = dict(x=63.47, y=178.81, heading_deg=-89.97)
        # The ego was changing into the slow car's lane (fragment 3.0 m left).
        path = lane_change_path(np.column_stack((np.arange(0., 170.1, .5), np.zeros(341))), 3., 60.)
        self.assertTrue(moving_car_owns_obstacle_fragment(fragment, car))
        objects = highway_objects([fragment], [car], ego=ego)
        self.assertEqual(objects, [car])
        self.assertEqual(static_clear_distance(path, ego, highway_static_objects(objects), 170.), 170.)
        # Before: the fragment alone stopped the path 36.75 m ahead.
        self.assertLess(static_clear_distance(path, ego, highway_static_objects([fragment]), 170.), 40.)

    def test_overlapping_fragments_within_camera_error_are_owned(self):
        car = self.car(0.)
        for fragment in [self.fragment(y=.75), self.fragment(x=32.),
                         self.fragment(length=5.), self.fragment(width=2.2),
                         self.fragment(x=27.2, length=1.1, width=2.)]:
            with self.subTest(fragment=fragment):
                self.assertTrue(moving_car_owns_obstacle_fragment(fragment, car))
                self.assertEqual(highway_objects([fragment], [car]), [car])

    def test_boxes_beside_beyond_or_larger_than_the_car_are_preserved(self):
        car = self.car(0.)
        for fragment in [self.fragment(y=1.4), self.fragment(y=2.), self.fragment(x=34.),
                         self.fragment(x=26.), self.fragment(length=5.6),
                         self.fragment(width=2.7), self.fragment(y=1., width=2.)]:
            with self.subTest(fragment=fragment):
                self.assertFalse(moving_car_owns_obstacle_fragment(fragment, car))
                self.assertEqual(highway_objects([fragment], [car]), [fragment, car])

    def test_rotated_containment_checks_every_corner(self):
        car = list(self.car(0.))
        car[8] = 90.
        car = tuple(car)
        inside = self.fragment(length=3., width=1.1, heading=-90.)
        across = self.fragment(length=4.5, width=1.1, heading=0.)
        self.assertTrue(moving_car_owns_obstacle_fragment(inside, car))
        self.assertFalse(moving_car_owns_obstacle_fragment(across, car))

    def test_recorded_parked_box_inside_the_ego_body_is_dropped(self):
        # Run follow-20261009T191227Z t=25.06 (also 185440Z/183356Z): a 4.6x1.9
        # "parked vehicle" 2.7 m ahead and 1 m left of our centre, raw LiDAR
        # clear, made static clearance 0 and an emergency stop at 72 km/h.
        ego = dict(x=62.55, y=240.86, heading_deg=-89.7)
        parked = (63.53425879886456, 238.20383140379178, 4.6, "fusion_parked_vehicle", "obstacle",
                  0.0, 4.6, 1.9, -89.79452178147075, 0.0, -0.0, 86, 0.0, None)
        path = np.column_stack((np.arange(0., 170.1, .5), np.zeros(341)))
        self.assertGreater(ego_body_penetration_m(parked, ego), .8)
        self.assertEqual(static_clear_distance(path, ego, highway_static_objects([parked]), 170.), 0.)
        self.assertEqual(highway_objects([parked], [], ego=ego), [])
        self.assertEqual(highway_objects([parked], []), [parked])   # no pose: unchanged

    def test_boxes_ahead_beside_or_moving_are_not_ego_projections(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        ahead = (6., 0., 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 0.)
        beside = (0., 3.5, 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 0.)
        touching = (0., 1.75, 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 0.)
        moving = (1., .5, 4.6, 'fusion_npc', 'npc', 5., 4.6, 1.9, 0., 5., 0., 7)
        for box in (ahead, beside, touching, moving):
            with self.subTest(box=box):
                self.assertEqual(highway_objects([box], [], ego=ego), [box])
        self.assertLess(ego_body_penetration_m(touching, ego), .3)

    @staticmethod
    def mover(x, y, length, width, yaw, vx, vy, uid, age=0.):
        return (x, y, max(length, width), 'lidar_car', 'npc', float(np.hypot(vx, vy)), length, width,
                yaw, vx, vy, uid, 0., None, True, age)

    @staticmethod
    def parked(x, y, width, yaw, uid, length=4.6):
        return (x, y, length, 'fusion_parked_vehicle', 'obstacle', 0., length, width, yaw, 0., -0., uid, 0., None)

    def test_recorded_parked_duplicates_of_measured_movers_are_owned(self):
        # Run follow-20261009T204445Z: beside the box the one-to-one match gives a
        # measured mover, the camera kept a second "parked vehicle" box 0.9-1.8 m
        # beyond its front. t=503.43: box 276 of 500056 (38 km/h) re-rooted the
        # change into its lane; t=507.06: box 279 cancelled it ("static relief",
        # 25 m hit); t=212.15: box 84 of the 9 m truck stood 12 m ahead of us.
        cases = [  # car, its matched camera box, the duplicate, camera minus car time
            (self.mover(70.2533, 54.9028, 6.7679, 2.5802, 267.109, .6176, -10.352, 500056),
             self.parked(70.5419, 54.3106, 2.1073, -89.7842, 275),
             self.parked(70.463, 49.7253, 1.9, -89.7842, 276), -.0095),
            (self.mover(70.7593, 13.6528, 7.4843, 2.7479, 272.558, .2327, -11.556, 500056),
             (70.832, 12.6117, 4.6, 'fusion_npc', 'npc', 10.6376, 4.6, 2.2153, -89.0918,
              .1686, -10.6363, 277, .0709, None),
             self.parked(71.2707, 10.2802, 1.9, -89.6556, 279), -.0119),
            (self.mover(66.6542, 128.5318, 8.8826, 3.1367, 269.19, .1957, -7.8772, 500013),
             (66.5155, 127.2399, 4.6, 'fusion_npc', 'npc', 8.5281, 4.6, 2.5577, -89.7665,
              .0347, -8.528, 78, .0122, None),
             self.parked(65.9759, 123.2221, 1.9, -86.6032, 84), .074),
        ]
        for car, matched, duplicate, delay in cases:
            with self.subTest(box=duplicate[11]):
                self.assertFalse(moving_car_owns_obstacle_fragment(duplicate, car))
                self.assertTrue(moving_car_owns_parked_duplicate(duplicate, car, delay))
                objects = highway_objects([matched, duplicate], [car], parked_delay_s=delay)
                self.assertEqual(objects, [car])
                self.assertEqual(highway_tentative_objects([duplicate], [car], parked_delay_s=delay), [])
        # t=507.06: our change path 2.07 m left toward the car's lane hit box 279.
        ego = dict(x=68.61, y=39.87, heading_deg=-85.71)
        path = lane_change_path(np.column_stack((np.arange(0., 170.1, .5), np.zeros(341))), 2.07, 30.)
        self.assertLess(static_clear_distance(path, ego, highway_static_objects([cases[1][2]]), 170.), 30.)

    def test_parked_box_beyond_beside_or_larger_than_a_mover_stays(self):
        car, matched = self.car(0.), self.parked(30., 0., 1.9, 0., 1)
        # 4.85 m ahead of the 4.9 m car's centre: a body just beyond touching distance.
        for box in (self.parked(34.85, 0., 1.9, 0., 2), self.parked(31., 1.4, 1.9, 0., 3),
                    self.parked(32., 0., 2.8, 0., 4, length=6.)):
            with self.subTest(box=box):
                self.assertFalse(moving_car_owns_parked_duplicate(box, car))
                self.assertEqual(highway_objects([matched, box], [car]), [box, car])
        duplicate = self.parked(32.5, 0., 1.9, 0., 5)
        self.assertTrue(moving_car_owns_parked_duplicate(duplicate, car))
        for field, value in ((15, .28), (14, False), (5, 0.), (9, 0.)):
            row = list(car)
            row[field] = value
            with self.subTest(field=field):
                self.assertFalse(moving_car_owns_parked_duplicate(duplicate, tuple(row)))
        self.assertFalse(moving_car_owns_parked_duplicate(self.fragment(32.5), car))

    def test_parked_duplicate_is_compared_with_the_car_at_the_camera_time(self):
        # 90 km/h, camera 0.1 s after the LiDAR scan: the car moved 2.5 m.
        car = self.mover(30., 0., 4.9, 2.1, 0., 25., 0., 500010)
        duplicate = self.parked(32.5+3., 0., 1.9, 0., 6)
        self.assertFalse(moving_car_owns_parked_duplicate(duplicate, car))
        self.assertTrue(moving_car_owns_parked_duplicate(duplicate, car, .1))
        self.assertFalse(moving_car_owns_parked_duplicate(duplicate, car, .5))

    def test_unknown_standing_or_predicted_car_never_owns_obstacle_fragment(self):
        fragment = self.fragment()
        for field, value in [(14, False), (5, 0.), (15, .001), (15, 1.1),
                             (15, float('nan')), (14, float('nan')), (9, float('nan'))]:
            car = list(self.car(0.))
            car[field] = value
            car = tuple(car)
            self.assertFalse(moving_car_owns_obstacle_fragment(fragment, car))
            self.assertIn(fragment, highway_objects([fragment], [car]))

    def test_invalid_or_moving_fragment_is_not_discarded(self):
        car = self.car(0.)
        for field, value in [(0, float('nan')), (6, 0.), (7, float('nan')),
                             (8, float('inf')), (5, 1.), (9, float('nan'))]:
            fragment = list(self.fragment())
            fragment[field] = value
            fragment = tuple(fragment)
            self.assertFalse(moving_car_owns_obstacle_fragment(fragment, car))
            self.assertIs(highway_objects([fragment], [car])[0], fragment)

    def test_measured_semantic_motion_survives_unknown_and_inferred_lidar_duplicate(self):
        semantic = (30., 0., 4.6, 'fusion_npc', 'npc', 6., 4.6, 1.9,
                    0., 6., 0., 67, 0., None)
        unknown = list(self.car(0.))
        unknown[14] = False
        variants = [tuple(unknown), tuple(unknown[:-2]), self.car(.2)]
        for source in ('assumed', 'tracked'):
            inferred = list(self.car(0.))
            inferred.append(source)
            variants.append(tuple(inferred))
        for car in variants:
            with self.subTest(car=car):
                objects = highway_objects([semantic], [car])
                self.assertIs(objects[0], semantic)
                self.assertEqual(objects[1][11], car[11])
                masks = highway_measured_objects(objects)
                self.assertIn(semantic, masks)
                self.assertNotIn(objects[1], masks)
                points = np.array([[30., 0.], [30., 1.5], [34., 0.]])
                np.testing.assert_array_equal(without_moving_objects(points, masks,
                    dict(x=0., y=0., heading_deg=0.)), points[1:])

    def test_measured_lidar_keeps_existing_semantic_jitter_ownership(self):
        car = self.car(0.)
        semantic = (33.9, 1.7, 4.6, 'fusion_npc', 'npc', 6., 4.6, 1.9,
                    0., 6., 0., 67, 0., None)
        self.assertEqual(highway_objects([semantic], [car]), [car])

    def test_recorded_front_returns_keep_measured_semantic_scan_time_mask(self):
        # follow-20261012T040248Z-3100834: measured NPC67 was deleted in favour
        # of assumed PF500022/500024. Its unmasked rear face became a static
        # 27.25/22.25 m stop line. These are recorded source tuples, source-time
        # deltas and rounded world hit witnesses; this is not a full-drive replay.
        cases = [
            (dict(x=67.38, y=90.88, heading_deg=-88.16), .23738670349121094,
             -.1506667137145996,
             (66.81627897157338, 58.735150877045626, 4.6, 'fusion_npc', 'npc',
              7.3646077529840746, 4.6, 1.9, -91.69877452040507,
              -.21832280142562488, -7.361370966721538, 67, -.056991178854517946, None),
             (66.17768096532369, 57.086608525629686, 8.696439743041992, 'lidar_car', 'npc',
              7.80766984764766, 8.696439743041992, 3.1167967319488525, 272.991646010436,
              .03073934670792366, -7.80760933592546, 500022, 0., None, True, 0., 'assumed'),
             (67.054, 59.659)),
            (dict(x=67.57, y=70.65, heading_deg=-90.35), .17435574531555176,
             -.0797429084777832,
             (67.19601302369585, 43.84514604096416, 4.6, 'fusion_npc', 'npc',
              8.66275035965621, 4.6, 1.9, -90.40399192789042,
              -.06108045391515941, -8.66253501995076, 67, -.02880380738450749, None),
             (67.30331344859125, 43.15535049393273, 4.906463623046875, 'lidar_car', 'npc',
              8.357264503156967, 4.906463623046875, 1.9005203247070312, 268.23654233541833,
              .05230172524558339, -8.357100843310665, 500024, 0., None, True, 0., 'assumed'),
             (67.231, 44.479)),
        ]
        from highway_planner import local_points
        from unittest.mock import patch
        for ego, fusion_delta, scan_delta, source_npc, car, hit in cases:
            with self.subTest(uid=car[11]):
                source_npc = (*source_npc, None, 0., None,
                              dict(height_m=1.2, raw_size_m=[4.6, 1.9]))
                semantic, timing = align_moving_observations([source_npc], 100.-fusion_delta, 100.)
                self.assertTrue(timing['timing_valid'])
                points = local_points(np.array([hit, [hit[0]+3., hit[1]]]), ego)
                with patch('planning.highway.object_policy._moving_semantic_npc', return_value=False):
                    before = highway_objects(semantic, [car])
                self.assertEqual(highway_measured_objects(before), [])
                objects = highway_objects(semantic, [car])
                scan_masks, timing = align_moving_observations(highway_measured_objects(objects),
                                                              100., 100.+scan_delta)
                self.assertTrue(timing['timing_valid'])
                np.testing.assert_array_equal(without_moving_objects(points, scan_masks, ego), points[1:])
                self.assertEqual([o[11] for o in objects], [67, car[11]])
                self.assertIsNone(source_npc[14])
                self.assertIs(objects[0][14], True)

    def test_semantic_padding_normalization_restores_leader_native_and_far_body_checks(self):
        from highway_speed_plan import HighwaySpeedPlanner
        from planning.highway.policy import native_motion
        from types import SimpleNamespace
        path = np.column_stack((np.arange(0., 300., .5), np.zeros(600)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90.)
        source = (40., 0., 4.6, 'fusion_npc', 'npc', 5., 4.6, 1.9, 0., 5., 0.,
                  67, 0., None, None, 0., None, dict(height_m=1.3, raw_size_m=[4.6, 1.9]))
        faster = list(self.car(0.))
        faster[0], faster[5], faster[9] = 40., 28., 28.
        faster = (*faster, 'assumed')
        objects = highway_objects([source], [faster])
        self.assertIsNone(source[14])
        self.assertIs(objects[0][14], True)
        self.assertEqual(objects[0][:14]+objects[0][15:], source[:14]+source[15:])
        model = HighwaySpeedPlanner()
        times = np.arange(0., 3.01, .1)
        self.assertEqual(len(model.leaders(path, ego, [source], times)), 0)
        leaders = model.leaders(path, ego, objects, times)
        self.assertEqual(len(leaders), 2)
        np.testing.assert_allclose(leaders[0][1], 5., rtol=0., atol=1e-12)
        lane = SimpleNamespace(half_width_m=1.75)
        self.assertIsNotNone(native_motion(objects[0], [(lane, path)]))
        self.assertIsNone(timed_gap(path, ego, [source, faster], 25.,
                                    horizon_s=3., lane_change=False))
        risk = timed_gap(path, ego, objects, 25., horizon_s=3., lane_change=False)
        self.assertEqual(risk['object_id'], 67)
        self.assertIsNotNone(risk['physical_collision_when'])
        measured = model.preview(path, ego, objects, 25., 7.)
        unknown_only = model.preview(path, ego, [faster], 25., 7.)
        self.assertLess(measured.speeds[-1], unknown_only.speeds[-1])

    def test_semantic_normalization_cannot_promote_invalid_or_unobserved_motion(self):
        source = (40., 0., 4.6, 'fusion_npc', 'npc', 5., 4.6, 1.9, 0., 5., 0.,
                  67, 0., None, None, 0., None, dict(height_m=1.3, raw_size_m=[4.6, 1.9]))
        for field, value in ((3, 'other_npc'), (4, 'obstacle'), (5, 0.),
                             (5, float('nan')), (9, 0.), (9, float('nan')), (6, 0.),
                             (14, False), (15, .1), (15, float('nan')), (16, 'assumed')):
            row = list(source)
            row[field] = value
            row = tuple(row)
            with self.subTest(field=field, value=value):
                self.assertIs(highway_objects([row], [])[0], row)

    def test_normalized_semantic_cut_in_keeps_relative_braking_reserve(self):
        from highway_planner import HighwayPlanner
        path = np.column_stack((np.arange(0., 500.), np.zeros(500)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=100.)
        source = (80., 3.5, 4.6, 'fusion_npc', 'npc', np.hypot(10., -1.),
                  4.6, 1.9, 0., 10., -1., 67, 0., None, None, 0., None)
        config = dict(from_idx=0, to_idx=450, kmh=90., lane_change_policy='free_three_lane')
        legacy = HighwayPlanner(config, path).plan_speed(path, ego, [source], 25.,
                                                        static_clear_m=float('inf'))
        result = HighwayPlanner(config, path).plan_speed(path, ego,
            highway_objects([source], []), 25., static_clear_m=float('inf'))
        self.assertFalse(result['emergency'])
        self.assertLess(result['reference_mps'], legacy['reference_mps'])
        self.assertGreater(result['diagnostics']['braking_limit_mps2'],
                           legacy['diagnostics']['braking_limit_mps2'])

    def test_unrecognized_or_unobserved_padding_cannot_acquire_only_mask_rights(self):
        source = (30., 0., 4.6, 'fusion_npc', 'npc', 5., 4.6, 1.9, 0., 5., 0.,
                  67, 0., None, None, 0., None)
        car = list(self.car(0.))
        car[14] = False
        car = tuple(car)
        for field, value in ((3, 'other_npc'), (15, .1), (15, float('nan'))):
            row = list(source)
            row[field] = value
            row = tuple(row)
            with self.subTest(field=field, value=value):
                objects = highway_objects([row], [car])
                self.assertEqual(objects, [car])
                self.assertEqual(highway_measured_objects(objects), [])

    def test_stationary_or_invalid_semantic_motion_does_not_replace_ownership(self):
        semantic = (30., 0., 4.6, 'fusion_npc', 'npc', 6., 4.6, 1.9,
                    0., 6., 0., 67, 0., None)
        unknown = list(self.car(0.))
        unknown[14] = False
        car = tuple(unknown)
        for field, value in ((5, 0.), (5, float('nan')), (9, 0.),
                             (9, float('nan')), (6, 0.)):
            row = list(semantic)
            row[field] = value
            with self.subTest(field=field, value=value):
                self.assertEqual(highway_objects([tuple(row)], [car]), [car])
        parked = self.parked(30., 0., 1.9, 0., 67)
        self.assertEqual(highway_objects([parked], [car]), [car])

    def test_preserved_semantic_mask_still_requires_valid_source_alignment(self):
        semantic = (30., 0., 4.6, 'fusion_npc', 'npc', 6., 4.6, 1.9,
                    0., 6., 0., 67, 0., None)
        inferred = (*self.car(0.), 'assumed')
        objects = highway_objects([semantic], [inferred])
        aligned, info = align_moving_observations(objects, 100., 100.1)
        self.assertTrue(info['timing_valid'])
        masks = highway_measured_objects(aligned)
        scan_masks, timing = align_moving_observations(masks, 100.1, 100.2)
        self.assertTrue(timing['timing_valid'])
        self.assertAlmostEqual(scan_masks[0][0], 31.2)
        # The runtime removes fusion masks on invalid fusion timing, even when
        # a LiDAR timestamp is valid. The preserved tuple cannot change this.
        for source in (None, 0., float('nan'), 99.6):
            _, timing = align_moving_observations([semantic], source, 100.)
            self.assertFalse(timing['timing_valid'])

    def test_slower_semantic_leader_stays_in_actual_future_body_check(self):
        path = np.column_stack((np.arange(0., 200., .5), np.zeros(400)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=80.)
        semantic = (30., 0., 4.6, 'fusion_npc', 'npc', 2., 4.6, 1.9,
                    0., 2., 0., 67, 0., None)
        car = list(self.car(0.))
        car[5], car[9] = 20., 20.
        car = (*car, 'assumed')
        self.assertIsNone(timed_gap(path, ego, [car], 80./3.6,
            horizon_s=3., lane_change=False))
        risk = timed_gap(path, ego, highway_objects([semantic], [car]), 80./3.6,
            horizon_s=3., lane_change=False)
        self.assertEqual(risk['object_id'], 67)
        self.assertIsNotNone(risk['physical_collision_when'])

    def test_preserved_semantic_current_contact_cannot_be_masked_away(self):
        from highway_planner import HighwayPlanner
        path = np.column_stack((np.arange(0., 500.), np.zeros(500)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=100.)
        semantic = (6., 0., 4.6, 'fusion_npc', 'npc', 6., 4.6, 1.9,
                    0., 6., 0., 67, 0., None)
        inferred = list(self.car(0.))
        inferred[0], inferred[1] = 8., .9
        objects = highway_objects([semantic], [(*inferred, 'assumed')])
        planner = HighwayPlanner(dict(from_idx=0, to_idx=450, kmh=120.,
            lane_change_policy='free_three_lane'), path)
        result = planner.plan_speed(path, ego, objects, 120./3.6,
                                    static_clear_m=float('inf'))
        self.assertTrue(result['emergency'])
        self.assertEqual(result['reference_mps'], 0.)

    def test_recorded_brief_prediction_does_not_invent_stationary_stop_line(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        path = np.column_stack((np.arange(0., 170.1, .5), np.zeros(341)))
        mover = self.car()
        # The mover remains in the dynamic list even while static returns are absent.
        objects = highway_objects([], [mover])
        self.assertEqual(objects[0][:2], mover[:2])
        self.assertEqual(objects[0][9:], mover[9:])
        self.assertEqual(highway_static_objects(objects), [])
        self.assertEqual(static_clear_distance(path, ego, highway_static_objects(objects), 170.), 170.)

    def test_prediction_cannot_mask_raw_points_or_own_fresh_parked_box(self):
        mover = self.car()
        parked = (30., 0., 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 0.)
        objects = highway_objects([parked], [mover])
        self.assertIn(parked, objects)
        self.assertNotIn(mover, highway_measured_objects(objects))
        self.assertEqual(highway_static_objects(objects), [parked])

    def test_expired_unknown_stopped_or_invalid_predictions_remain_static(self):
        self.assertEqual(MOTION_TRACK_KEEP_S, TRACK_KEEP_S)
        for age in (TRACK_KEEP_S+.001, float('inf'), float('nan'), -.1):
            car = self.car(age)
            self.assertEqual(highway_static_objects([car]), [car])
        for field, value in ((5, 0.), (5, float('nan')), (14, False), (9, float('nan')),
                             (0, float('nan')), (6, float('nan')), (8, float('nan'))):
            row = list(self.car())
            row[field] = value
            car = tuple(row)
            self.assertEqual(highway_static_objects([car]), [car])
            self.assertNotIn(car, highway_measured_objects([car]))

    def test_recorded_longer_missed_scans_keep_motion_and_grow_uncertainty(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90.)
        path = np.column_stack((np.arange(0., 170.1, .5), np.zeros(341)))
        sizes, risks = [], []
        for age in (0., .428863, .683992):
            original = self.car(age)
            mover = highway_objects([], [original])[0]
            self.assertEqual(mover[:2], original[:2])
            self.assertEqual(mover[9:], original[9:])
            self.assertEqual(highway_static_objects([mover]), [])
            risk = timed_gap(path, ego, [mover], 25., horizon_s=3., lane_change=False)
            self.assertEqual(risk['object_id'], original[11])
            risks.append(risk['collision_when'])
            sizes.append(mover[6:8])
        self.assertTrue(all(sizes[i][0] < sizes[i+1][0] and sizes[i][1] < sizes[i+1][1]
                            for i in range(2)))
        self.assertGreaterEqual(risks[0], risks[1])
        self.assertGreaterEqual(risks[1], risks[2])

    def test_predicted_footprint_never_erases_independent_raw_body_return(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        points = np.array([[30., 0.], [30., 2.], [10., 0.]])
        mover = highway_objects([], [self.car(.683992)])[0]
        measured = highway_measured_objects([mover])
        self.assertEqual(measured, [])
        np.testing.assert_array_equal(without_moving_objects(points, measured, ego), points)

    def test_detector_reconnects_same_uid_and_observed_shape_after_short_loss(self):
        tracks = VehicleTracks()
        def detection(stamp):
            return dict(position=[10.+10.*stamp, 0.], size=[4.9, 2.1, 1.5],
                        heading_deg=0., score=.9)
        for stamp in (0., .15, .3):
            observed = tracks.update([detection(stamp)], stamp)[0]
        self.assertTrue(observed['velocity_valid'])
        uid = observed['uid']
        for age in (.428863, .683992):
            predicted = tracks.update([], .3+age)[0]
            self.assertEqual(predicted['uid'], uid)
            self.assertAlmostEqual(predicted['position'][0], 13.+10.*age)
            self.assertAlmostEqual(predicted['prediction_age_s'], age)
        observed = tracks.update([detection(1.05)], 1.05)[0]
        self.assertEqual(observed['uid'], uid)
        self.assertNotIn('prediction_age_s', observed)
        original = self.car(0.)
        self.assertEqual(highway_objects([], [original]), [original])


class FarMovingReturnMaskTests(unittest.TestCase):
    """Returns of moving cars beyond the near field (runs 2026-10-09 E2E)."""

    @staticmethod
    def car(x, y, speed, age=0., length=4.9, width=2.1, uid=500010):
        return (x, y, max(length, width), 'lidar_car', 'npc', speed, length, width,
                0., speed, 0., uid, 0., None, True, age)

    def test_recorded_rear_face_return_is_removed_beyond_the_near_field(self):
        # Run follow-20261009T185440Z t=32.14: a 12-point hit 1.03 m beyond the
        # rear of a 27 km/h car became a 26.5 m stop line (braking 40 -> 29 km/h).
        ego = dict(x=63.54, y=142.79, heading_deg=-86.95)
        car = (66.54765018312818, 118.03263316007356, 6.694106578826904, "lidar_car", "npc",
               7.636268857487029, 6.694106578826904, 2.5820741653442383, 273.10838860683344,
               0.0466024681237506, -7.636126653859995, 500019, 0.0, None, True, 0.0)
        scan, pose = 1791572346.938081, 1791572347.032473
        hit = local_points(np.array([[67.01, 114.394]]), ego)
        measured, _ = align_moving_observations(highway_measured_objects([car]), pose, scan)
        np.testing.assert_array_equal(without_moving_objects(hit, measured, ego), hit)
        far, _ = align_moving_observations(highway_scan_mask_objects([car]), pose, scan)
        self.assertEqual(len(without_far_moving_returns(hit, far, ego)), 0)

    def test_near_field_keeps_the_exact_mask(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        car = self.car(8., 0., 8.)
        points = np.array([[8.+2.45+.9, 0.], [30.+2.45+.9, 0.]])
        mask = highway_scan_mask_objects([car, self.car(30., 0., 8.)])
        kept = without_far_moving_returns(points, mask, ego)
        np.testing.assert_array_equal(kept, points[:1])

    def test_only_recent_measured_motion_masks_far_returns(self):
        moving = self.car(40., 0., 8.)
        mask = highway_scan_mask_objects([moving])
        self.assertEqual(len(mask), 1)
        self.assertAlmostEqual(mask[0][6], 4.9+2.*(.6+.05*8.))
        self.assertAlmostEqual(mask[0][7], 2.1+2.*.3)
        recent = highway_objects([], [self.car(40., 0., 8., age=.3)])
        predicted = highway_scan_mask_objects(recent)
        self.assertEqual(len(predicted), 1)
        self.assertAlmostEqual(predicted[0][6], 4.9+2.*(.6+.15*8.+.3*(1.+.45)))
        for body in (self.car(40., 0., 0.), self.car(40., 0., 8., age=.6),
                     tuple(list(self.car(40., 0., 8.))[:14]+[False, 0.])):
            with self.subTest(body=body):
                self.assertEqual(highway_scan_mask_objects(highway_objects([], [body])), [])


if __name__ == '__main__':
    unittest.main()
