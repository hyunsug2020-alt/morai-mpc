"""Regress unlabeled moving-cluster prediction without ROS or a simulator."""
import unittest
from unittest.mock import patch

import numpy as np

from tunnel_tracks import VEHICLE_BODY_MOVING_MPS, TunnelTracks

# The view-filter tests below isolate that filter: car-length bodies also need
# VEHICLE_BODY_MOVING_MPS to start moving (tested separately at the end).
WITHOUT_BODY_GATE = patch('tunnel_tracks.VEHICLE_BODY_MOVING_MPS', 0.)


def box(x=10., y=4., length=.6, width=.6, source_id=0):
    return (x, y, max(length, width), "tunnel_lidar_%d" % source_id,
            "obstacle", 0., length, width, 0., 0., 0., source_id)


class TunnelTrackTests(unittest.TestCase):
    @WITHOUT_BODY_GATE
    def test_near_face_reveal_without_near_edge_motion_stays_static(self):
        for heading in (0., 73.):
            yaw = np.radians(heading)
            axes = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            for side in (-1., 1.):
                with self.subTest(heading=heading, side=side):
                    tracks, unfiltered = TunnelTracks(), TunnelTracks()
                    for i in range(6):
                        centre = axes @ [12., side*(3.+.1*i)]
                        observer = axes @ [.5*i, 0.]
                        row = list(box(*centre, length=4.8, width=2.1))
                        row[8] = heading
                        metadata = dict(point_count=65, motion_center=centre.tolist())
                        before = unfiltered.update([tuple(row)+(metadata,)], 100.+.1*i)[0]
                        # Only the far side is revealed: the near bound stays
                        # at +/-2.5 m despite 1 m/s of centroid displacement.
                        metadata = dict(metadata, motion_extent=[4.6, 1.+.2*i],
                                        observer=[*observer, heading])
                        out = tracks.update([tuple(row)+(metadata,)], 100.+.1*i)[0]
                        self.assertEqual(out[4], 'obstacle')
                        self.assertEqual(out[5], 0.)
                    self.assertEqual(before[4], 'pedestrian')

    def test_near_edge_motion_keeps_a_crossing_person_moving_on_either_side(self):
        for side in (-1., 1.):
            with self.subTest(side=side):
                tracks = TunnelTracks()
                for i in range(6):
                    y = side*(3.-.12*i)
                    metadata = dict(point_count=12, motion_center=[12., y],
                                    motion_extent=[.6, .6], observer=[.5*i, 0., 0.])
                    out = tracks.update([box(y=y)+(metadata,)], 100.+.1*i)[0]
                self.assertEqual(out[4], 'pedestrian')
                self.assertAlmostEqual(out[10], -side*1.2, places=5)

    def test_confirmed_crossing_motion_has_world_velocity_and_stable_id(self):
        tracks, ids = TunnelTracks(), []
        for i in range(5):
            obj = tracks.update([box(y=4. - i * .1, source_id=i)], 10. + i * .1)[0]
            ids.append(obj[11])
        self.assertEqual(len(set(ids)), 1)
        self.assertEqual(obj[4], "pedestrian")
        self.assertTrue(obj[3].startswith("tunnel_lidar_dynamic_"))
        self.assertAlmostEqual(obj[5], 1., places=5)
        self.assertAlmostEqual(obj[9], 0., places=5)
        self.assertAlmostEqual(obj[10], -1., places=5)

    def test_short_history_and_duplicate_timestamps_cannot_confirm_motion(self):
        tracks = TunnelTracks()
        for i in range(3):
            obj = tracks.update([box(y=4. - i * .1)], 10. + i * .1)[0]
        for _ in range(50):
            obj = tracks.update([box(y=3.8)], 10.2)[0]
        self.assertEqual(obj[4], "obstacle")
        self.assertEqual(obj[5], 0.)
        obj = tracks.update([box(y=3.7)], 10.3)[0]
        self.assertEqual(obj[4], "pedestrian")

    def test_stationary_centroid_noise_remains_static(self):
        tracks = TunnelTracks()
        for i in range(12):
            jitter = .04 if i % 2 else -.04
            obj = tracks.update([box(x=10. + jitter, y=4. - jitter)], 10. + i * .1)[0]
            self.assertEqual(obj[4], "obstacle")
            self.assertEqual(obj[5], 0.)

    def test_fitted_box_rotation_cannot_fabricate_surface_motion(self):
        tracks = TunnelTracks()
        for i in range(5):
            measured = box(x=10.+.1*i, y=4., length=4.6, width=2.) + ({
                'point_count': 65, 'motion_center': [10., 4.]},)
            out = tracks.update([measured], 10.+.1*i)[0]
            self.assertEqual(out[4], 'obstacle')
            self.assertEqual(out[:2], measured[:2])

    def test_independent_surface_motion_preserves_crossing_and_fitted_box_position(self):
        tracks = TunnelTracks()
        for i in range(5):
            measured = box(x=10.+.06*i, y=4.-.2*i) + ({
                'point_count': 8, 'motion_center': [10., 4.-.2*i]},)
            out = tracks.update([measured], 10.+.1*i)[0]
        self.assertEqual(out[4], 'pedestrian')
        self.assertAlmostEqual(out[9], 0.)
        self.assertAlmostEqual(out[10], -2.)
        self.assertEqual(out[:2], measured[:2])
        predicted = tracks.update([], 10.5)[0]
        self.assertAlmostEqual(predicted[0], measured[0])
        self.assertAlmostEqual(predicted[1], measured[1]-.2)

    def test_recorded_vehicle_face_centroid_jumps_do_not_confirm_motion(self):
        tracks = TunnelTracks()
        samples = ((1790960455.7034311, -63.164995676906145, -543.3179848482196, 2.4),
                   (1790960455.8129392, -63.12044129402341, -543.0746119498655, 2.2),
                   (1790960455.9238677, -63.16381869723372, -543.2006935630886, 2.2),
                   (1790960456.0340385, -63.24510249965588, -543.1424589735998, 2.2))
        for stamp, x, y, width in samples:
            out = tracks.update([box(x=x, y=y, length=4.4, width=width) +
                                 ({"point_count": 65},)], stamp)[0]
            self.assertEqual(out[4], "obstacle")
            self.assertEqual(out[5], 0.)

    @WITHOUT_BODY_GATE
    def test_slow_vehicle_translation_with_centroid_noise_remains_moving(self):
        tracks = TunnelTracks()
        for i in range(5):
            jitter = .02 if i % 2 else -.02
            out = tracks.update([box(x=10. + jitter, y=4. - .09 * i,
                                     length=4.4, width=2.2) + ({"point_count": 65},)],
                                10. + i * .1)[0]
        self.assertEqual(out[4], "pedestrian")
        self.assertGreater(out[5], .8)
        self.assertAlmostEqual(out[10], -.9, places=5)

    def test_slower_than_walking_drift_is_static_until_a_mover_slows(self):
        tracks = TunnelTracks()
        for i in range(8):
            out = tracks.update([box(y=4. - .06 * i)], 10. + i * .1)[0]
            self.assertEqual(out[4], "obstacle")
        tracks = TunnelTracks()
        for i in range(5):
            out = tracks.update([box(y=4. - .1 * i)], 10. + i * .1)[0]
        self.assertEqual(out[4], "pedestrian")
        for i in range(1, 9):
            out = tracks.update([box(y=3.6 - .06 * i)], 10.4 + i * .1)[0]
            self.assertEqual(out[4], "pedestrian")
        self.assertAlmostEqual(out[5], .6, places=5)

    def test_recorded_surface_that_settled_does_not_start_a_moving_track(self):
        tracks = TunnelTracks()
        samples = (
            (1790992240.9947596, -102.52336353593968, -124.45132917367724),
            (1790992241.111191, -102.65704030909353, -124.35945360491125),
            (1790992241.2275171, -102.69099242301576, -124.33609108036224),
            (1790992241.3408158, -102.69128979944722, -124.33592569491987),
        )
        for stamp, x, y in samples:
            measured = box(x=x, y=y, length=8., width=.8) + ({'point_count': 100},)
            out = tracks.update([measured], stamp)[0]
            self.assertEqual(out[4], 'obstacle')
            self.assertEqual(out[:2], measured[:2])
        self.assertEqual(tracks.update([], samples[-1][0] + .116), [])

    @WITHOUT_BODY_GATE
    def test_long_narrow_object_with_continuing_motion_is_still_tracked(self):
        tracks = TunnelTracks()
        for i in range(5):
            out = tracks.update([box(x=10.+.09*i, length=8., width=.8) +
                                 ({'point_count': 100},)], 10.+.1*i)[0]
        self.assertEqual(out[4], 'pedestrian')
        self.assertAlmostEqual(out[9], .9)

    def test_confirmed_mover_that_stops_is_not_extrapolated_from_old_motion(self):
        tracks = TunnelTracks()
        for i in range(5):
            out = tracks.update([box(y=4.-.1*i)], 10.+.1*i)[0]
        self.assertEqual(out[4], 'pedestrian')
        tracks.update([box(y=3.6)], 10.5)
        out = tracks.update([box(y=3.6)], 10.6)[0]
        self.assertEqual(out[4], 'obstacle')
        self.assertEqual(out[5], 0.)
        self.assertEqual(tracks.update([], 10.7), [])

    def test_gently_curving_measured_motion_remains_moving(self):
        tracks = TunnelTracks()
        for i in range(5):
            angle = .12 * i
            out = tracks.update([box(x=10. + .8 * np.sin(angle),
                                     y=4. + .8 * (1. - np.cos(angle)))], 10. + i * .1)[0]
        self.assertEqual(out[4], "pedestrian")
        self.assertGreater(out[5], .5)

    @WITHOUT_BODY_GATE
    def test_crossing_long_box_preserves_measured_axes_and_raw_point_mask(self):
        from lidar_corridor import without_moving_objects
        tracks = TunnelTracks()
        for i in range(5):
            measured = list(box(y=4. - .12 * i, length=4., width=.6))
            measured[8] = 30.
            out = tracks.update([tuple(measured)], 10. + i * .1)[0]
        self.assertEqual(out[4], "pedestrian")
        self.assertEqual(out[8], 30.)
        self.assertAlmostEqual(out[9], 0., places=5)
        self.assertAlmostEqual(out[10], -1.2, places=5)
        centre = np.asarray(out[:2])
        angle = np.deg2rad(out[8])
        along = np.array([np.cos(angle), np.sin(angle)])
        across = np.array([-np.sin(angle), np.cos(angle)])
        points = np.vstack((centre + 1.5 * along, centre + 1.5 * across))
        filtered = without_moving_objects(points, [out], dict(x=0., y=0., heading_deg=0.))
        np.testing.assert_allclose(filtered, points[1:])

    def test_large_shape_change_resets_confirmed_motion(self):
        tracks = TunnelTracks()
        for i in range(5):
            obj = tracks.update([box(y=4. - i * .1)], 10. + i * .1)[0]
        self.assertEqual(obj[4], "pedestrian")
        uid = obj[11]
        obj = tracks.update([box(y=3.5, length=2.5, width=1.8)], 10.5)[0]
        self.assertEqual(obj[11], uid)
        self.assertEqual(obj[4], "obstacle")
        self.assertEqual(obj[5], 0.)

    def test_long_wall_residual_cannot_become_moving_from_centroid_shift(self):
        for length, width in ((12., .3), (6., .3), (9., 2.)):
            with self.subTest(length=length, width=width):
                tracks = TunnelTracks()
                for i in range(8):
                    obj = tracks.update([box(x=10. + i * .2, length=length, width=width)],
                                        10. + i * .1)[0]
                    self.assertEqual(obj[4], "obstacle")
                    self.assertEqual(obj[5], 0.)

    def test_sparse_echoes_moving_with_vehicle_remain_static(self):
        for length, width in ((.3, .3), (.3, .4), (.4, .3)):
            with self.subTest(length=length, width=width):
                tracks = TunnelTracks()
                for i in range(10):
                    obj = tracks.update([box(x=20. - i * .7, y=4. - i * .25,
                                             length=length, width=width)], 10. + i * .1)[0]
                    self.assertEqual(obj[4], "obstacle")
                    self.assertEqual(obj[5], 0.)

    def test_visible_single_row_edge_centroid_shift_remains_static(self):
        tracks = TunnelTracks()
        positions = ((-77.4472477386, -543.6231857532, 1.4),
                     (-77.0567800724, -543.4140345889, 1.6),
                     (-76.9982457425, -543.3713328343, 1.8),
                     (-77.0171040658, -543.3609057948, 1.8))
        for i, (x, y, length) in enumerate(positions):
            obj = tracks.update([box(x=x, y=y, length=length, width=.4)],
                                1790955369.7 + i * .106)[0]
            self.assertEqual(obj[4], "obstacle")
            self.assertEqual(obj[5], 0.)

    def test_motion_requires_at_least_four_measured_voxels(self):
        for count in (1, 2, 3, float("nan"), "invalid"):
            with self.subTest(point_count=count):
                tracks = TunnelTracks()
                for i in range(5):
                    obj = box(y=4. - i * .1) + ({"point_count": count},)
                    out = tracks.update([obj], 10. + i * .1)[0]
                    self.assertEqual(out[4], "obstacle")
        tracks = TunnelTracks()
        for i in range(4):
            obj = box(y=4. - i * .1) + ({"point_count": 4},)
            out = tracks.update([obj], 10. + i * .1)[0]
        self.assertEqual(out[4], "pedestrian")
        self.assertEqual(out[12], {"point_count": 4})

    def test_loss_of_motion_support_requires_four_new_supported_scans(self):
        tracks = TunnelTracks()
        for i in range(4):
            out = tracks.update([box(y=4. - i * .1)], 10. + i * .1)[0]
        self.assertEqual(out[4], "pedestrian")
        out = tracks.update([box(y=3.6, width=.4)], 10.4)[0]
        self.assertEqual(out[4], "obstacle")
        for i in range(3):
            out = tracks.update([box(y=3.5 - i * .1)], 10.5 + i * .1)[0]
            self.assertEqual(out[4], "obstacle")
        out = tracks.update([box(y=3.2)], 10.8)[0]
        self.assertEqual(out[4], "pedestrian")

    def test_short_occlusion_preserves_id_but_requires_motion_reconfirmation(self):
        tracks = TunnelTracks()
        for i in range(5):
            obj = tracks.update([box(y=4. - i * .1)], 10. + i * .1)[0]
        uid = obj[11]
        tracks.update([], 10.5)
        obj = tracks.update([box(y=3.2)], 10.8)[0]
        self.assertEqual(obj[11], uid)
        self.assertEqual(obj[4], "obstacle")
        for i in range(1, 4):
            obj = tracks.update([box(y=3.2 - i * .1)], 10.8 + i * .1)[0]
        self.assertEqual(obj[4], "pedestrian")

    def test_long_missing_interval_creates_new_static_track(self):
        tracks = TunnelTracks()
        first = tracks.update([box()], 10.)[0]
        obj = tracks.update([box()], 10.6)[0]
        self.assertNotEqual(obj[11], first[11])
        self.assertEqual(obj[4], "obstacle")

    def test_confirmed_motion_survives_brief_missing_returns_then_expires(self):
        tracks = TunnelTracks()
        for i in range(5):
            obj = tracks.update([box(y=4. - i * .1)], 10. + i * .1)[0]
        predicted = tracks.update([], 10.6)
        self.assertEqual(len(predicted), 1)
        self.assertEqual(predicted[0][11], obj[11])
        self.assertEqual(predicted[0][4], "pedestrian")
        self.assertAlmostEqual(predicted[0][1], 3.4, places=5)
        self.assertEqual(tracks.update([], 10.7), [])

    def test_unconfirmed_and_static_objects_are_not_extrapolated(self):
        tracks = TunnelTracks()
        tracks.update([box()], 10.)
        self.assertEqual(tracks.update([], 10.1), [])

    def test_clock_reset_discards_motion_history(self):
        tracks = TunnelTracks()
        for i in range(5):
            obj = tracks.update([box(y=4. - i * .1)], 10. + i * .1)[0]
        uid = obj[11]
        obj = tracks.update([box(y=3.5)], 1.)[0]
        self.assertNotEqual(obj[11], uid)
        self.assertEqual(obj[4], "obstacle")

    def test_nearest_association_is_one_to_one_and_ignores_input_ids(self):
        tracks = TunnelTracks()
        first = tracks.update([box(x=8., source_id=1), box(x=20., source_id=2)], 10.)
        second = tracks.update([box(x=20., source_id=900), box(x=8., source_id=800)], 10.1)
        self.assertEqual(first[0][11], second[1][11])
        self.assertEqual(first[1][11], second[0][11])
        self.assertNotEqual(second[0][11], second[1][11])

    def test_crossing_cluster_triggers_existing_yield_rule(self):
        from gui import moving_conflict
        tracks = TunnelTracks()
        for i in range(5):
            objects = tracks.update([box(y=4. - i * .12)], 10. + i * .1)
        route = np.column_stack((np.arange(0., 60., .5), np.zeros(120)))
        risk = moving_conflict(objects, dict(x=0., y=0., heading_deg=0.), route, 7.5)
        self.assertIsNotNone(risk)
        self.assertEqual(risk["kind"], "stop")
        self.assertTrue(risk["name"].startswith("tunnel_lidar_dynamic_"))

    def test_moving_away_side_cluster_does_not_trigger_yield(self):
        from gui import moving_conflict
        tracks = TunnelTracks()
        for i in range(5):
            objects = tracks.update([box(y=4. + i * .12)], 10. + i * .1)
        route = np.column_stack((np.arange(0., 60., .5), np.zeros(120)))
        self.assertIsNone(moving_conflict(objects, dict(x=0., y=0., heading_deg=0.), route, 7.5))

    def test_confirmed_moving_cluster_is_excluded_from_static_bypass(self):
        from gui import plan_static_bypass
        tracks = TunnelTracks()
        for i in range(5):
            objects = tracks.update([box(x=18., y=-.4 + i * .1)], 10. + i * .1)
        route = np.column_stack((np.arange(0., 65.5, .5), np.zeros(131)))
        plan, hazards = plan_static_bypass(route, dict(x=0., y=0., heading_deg=0., speed_kmh=27.),
                                          objects, lambda path, tolerance: True)
        self.assertIsNone(plan)
        self.assertEqual(hazards, [])

    def test_invalid_stamp_and_coordinates_fail_explicitly(self):
        tracks = TunnelTracks()
        for stamp in (0., float("nan"), float("inf")):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                tracks.update([box()], stamp)
        with self.assertRaises(ValueError):
            tracks.update([box(x=float("nan"))], 10.)


class VehicleBodyGateTests(unittest.TestCase):
    """follow-20261010T151125Z: parked Grandeur clusters read as movers (0.8-2.43 m/s)."""

    def slide(self, speed, length=4.8, width=1.8, steps=9, start=0.):
        tracks, out = TunnelTracks(), []
        for i in range(steps):
            y = -4.7+speed*.1*i
            out.append(tracks.update([box(x=20., y=y, length=length, width=width) +
                                      ({"point_count": 34},)], 10.+start+.1*i)[0])
        return tracks, out

    def test_parked_car_body_sliding_below_vehicle_speed_stays_static(self):
        # 00:16:40.5 tunnel_lidar_dynamic_800013: 4.33 x 1.81 m, 30 points, 20 m ahead,
        # 4.7 m right of the bypass, 0.8 -> 2.43 m/s towards the path; also the side
        # facing the ego alone (4.9 x 0.5 m) while passing it.
        for length, width in ((4.8, 1.8), (4.9, .5), (2.6, 1.9)):
            for speed in (.8, 1.5, 2.43, 2.65):
                with self.subTest(size=(length, width), speed=speed):
                    _, out = self.slide(speed, length, width)
                    self.assertTrue(all(o[4] == "obstacle" and o[5] == 0. for o in out))
                    with WITHOUT_BODY_GATE:
                        _, before = self.slide(speed, length, width)
                    self.assertEqual(before[-1][4], "pedestrian")

    def test_car_body_at_vehicle_speed_is_confirmed_and_released_as_before(self):
        tracks, out = self.slide(3.5)
        self.assertEqual(out[-1][4], "pedestrian")
        self.assertAlmostEqual(out[-1][5], 3.5, delta=.2)
        self.assertGreater(VEHICLE_BODY_MOVING_MPS, 2.65)
        # Once confirmed, a slowing car keeps yielding down to the common release.
        y = -4.7+3.5*.8
        for i in range(1, 6):
            o = tracks.update([box(x=20., y=y+1.*.1*i, length=4.8, width=1.8) +
                               ({"point_count": 34},)], 10.8+.1*i)[0]
            self.assertEqual(o[4], "pedestrian")

    def test_walking_person_keeps_the_walking_threshold(self):
        tracks = TunnelTracks()
        for i in range(6):
            o = tracks.update([box(x=20., y=-3.+.9*.1*i) + ({"point_count": 12},)], 10.+.1*i)[0]
        self.assertEqual(o[4], "pedestrian")
        self.assertAlmostEqual(o[5], .9, delta=.05)

    def test_car_cut_shorter_than_a_car_keeps_the_body_gate(self):
        # The x > -5 m window cuts a parked car beside the ego: once car-length in
        # this track, the remaining piece still needs vehicle speed.
        tracks = TunnelTracks()
        for i, length in enumerate((3.2, 3.0, 2.8, 2.6, 2.4, 2.3, 2.2, 2.1)):
            o = tracks.update([box(x=1.-.15*i, y=3.3, length=length, width=1.8) +
                               ({"point_count": 60},)], 10.+.1*i)[0]
            self.assertEqual(o[4], "obstacle")


if __name__ == "__main__":
    unittest.main()
