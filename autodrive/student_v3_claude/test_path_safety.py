"""Departure, latency and restart checks on the follower's actual path."""
import unittest
import json
import math
from pathlib import Path

import numpy as np

from path_safety import MotionMemory, PathSafety, stop_cap
from test_ped_guard import person, EGO, PATH


class PathSafetyTests(unittest.TestCase):
    def check(self, guard, objects, *, speed=30., age=.1, stamp=100., now=10., path=PATH):
        return guard.check(path, dict(EGO, speed_kmh=speed),
            [('semantic', objects, stamp, age)], now, requested_mps=30./3.6)

    def test_unconfirmed_person_limits_speed_before_departure(self):
        r = self.check(PathSafety(), [person(13.8, -1.7, 0., 0.)])
        self.assertTrue(r.emergency)
        self.assertEqual(r.cap_mps, 0.)
        self.assertEqual(r.validation['objects'][0]['state'], 'unknown')
        self.assertTrue(r.validation['objects'][0]['possible_start'])

    def test_unknown_departure_does_not_close_a_clear_adjacent_path(self):
        obj = person(13.8, -2.5, 0., 0.)
        fresh = self.check(PathSafety(), [obj], age=.1, speed=0.)
        self.assertFalse(fresh.emergency)
        self.assertGreater(fresh.cap_mps, 0.)
        delayed = self.check(PathSafety(), [obj], age=.4)
        self.assertTrue(delayed.emergency)

    def test_unknown_pedestrian_movement_still_closes_the_same_path_immediately(self):
        guard = PathSafety()
        self.assertFalse(self.check(guard, [person(13.8, -2.5, 0., 0.)], speed=0.).emergency)
        moved = self.check(guard, [person(13.8, -2.3, 0., 0.)], stamp=100.1, now=10.1)
        self.assertTrue(moved.emergency)
        self.assertEqual(moved.validation['objects'][0]['state'], 'unknown')

    def test_possible_pedestrian_departure_limits_speed_before_ten_meter_activation(self):
        obj = person(25., 3.2, 0., 0.)
        far = self.check(PathSafety(), [obj], speed=36., age=.2)
        self.assertFalse(far.emergency)
        self.assertGreater(far.cap_mps, 0.)
        self.assertLess(far.cap_mps, 36./3.6)
        near = self.check(PathSafety(), [person(13.8, 3.2, 0., 0.)], speed=30.)
        self.assertTrue(near.emergency)
        stopped = self.check(PathSafety(), [person(3.3, 3.2, 0., 0.)], speed=0.)
        self.assertFalse(stopped.emergency)
        self.assertGreater(stopped.cap_mps, 0.)

    def test_weak_motion_cannot_release_the_stopping_envelope(self):
        for lateral_speed in (.25, .35):
            with self.subTest(lateral_speed=lateral_speed):
                r = self.check(PathSafety(), [person(16., -3., 0., lateral_speed)])
                self.assertLess(r.cap_mps, 30./3.6)
                self.assertTrue(r.validation['objects'][0]['possible_start'])

    def test_roadside_departure_cap_does_not_alternate_stop_when_launching(self):
        # Just ahead of the true front bumper (rear axle + 3.845 m): the
        # departure-readiness cap applies. (At 3.3 m the person stands beside
        # the front fender, see the next test.)
        obj = person(4.2, 3.2, 0., 0.)
        targets = []
        for speed in (0., 3., 5., 6.5):
            r = self.check(PathSafety(), [obj], speed=speed)
            self.assertFalse(r.emergency)
            self.assertGreater(r.cap_mps, 0.)
            targets.append(r.cap_mps)
        np.testing.assert_allclose(targets, targets[0])
        self.assertLess(targets[0], 30./3.6)
        delayed = self.check(PathSafety(), [obj], speed=6.5, age=.4)
        self.assertLess(delayed.cap_mps, targets[0])

    def test_standing_person_beside_the_body_has_no_departure_cap(self):
        # follow-20261010T162527Z 01:27:47-28:01: after crossing, the person stood
        # 2.5 m right of the path beside our body; the departure cap (gap 0) held
        # us at 3-5 km/h for 13 s. Braking cannot keep anyone off our side.
        for f in (3.3, 1., -1.):
            with self.subTest(f=f):
                for speed in (0., 5., 15.):
                    r = self.check(PathSafety(), [person(f, -2.5, 0., 0.)], speed=speed, age=.15)
                    self.assertFalse(r.emergency)
                    self.assertAlmostEqual(r.cap_mps, 30./3.6)
        # Within 0.6 m of our side, or walking into it, every check remains.
        close = self.check(PathSafety(), [person(3.3, -1.6, 0., 0.)], speed=0.)
        self.assertLess(close.cap_mps, .5)
        walking_in = self.check(PathSafety(), [person(3.3, -2.5, 0., 1.4)], speed=5.)
        self.assertLess(walking_in.cap_mps, 30./3.6)
        # Still ahead of the front bumper: the departure cap stays.
        ahead = self.check(PathSafety(), [person(5., -2.5, 0., 0.)], speed=5.)
        self.assertLess(ahead.cap_mps, 30./3.6)
        self.assertTrue(ahead.validation['objects'][0]['possible_start'])

    def test_micro_bypass_cannot_drop_departure_readiness_outside_lateral_gate(self):
        # Recorded bypass put the person 4.5 m from its final centreline.
        # A fixed 2 m departure gate had restored 27.7 km/h at a 10 m gap,
        # just before the standing person began to cross toward the ego.
        obj = person(10., 4.5, 0., 0.)
        r = self.check(PathSafety(), [obj], speed=20., age=.15)
        self.assertFalse(r.emergency)
        self.assertGreater(r.cap_mps, 0.)
        self.assertLess(r.cap_mps, 20./3.6)
        self.assertTrue(r.validation['objects'][0]['possible_start'])
        wide = self.check(PathSafety(), [person(10., 10., 0., 0.)], speed=30., age=.15)
        self.assertFalse(wide.emergency)
        self.assertAlmostEqual(wide.cap_mps, 30./3.6)
        self.assertEqual(wide.validation['objects'], [])

    def test_crossing_finished_before_arrival_does_not_block_every_candidate(self):
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=0.),
            [('semantic', [person(8., 6., 0., -4.)], 100., .1)], 10.,
            requested_mps=.8, prediction_horizon_s=4.)
        self.assertFalse(r.emergency)
        self.assertAlmostEqual(r.cap_mps, .8)
        self.assertGreater(r.clear_m, 40.)

    def test_same_time_crossing_still_blocks_stopped_vehicle_launch(self):
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=0.),
            [('semantic', [person(4., 6., 0., -4.)], 100., .1)], 10.,
            requested_mps=.8, prediction_horizon_s=4.)
        self.assertLess(r.clear_m, 1.)
        self.assertTrue(r.validation['objects'])

    def test_temporal_pass_cannot_release_current_raw_collision(self):
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=0.),
            [('semantic', [person(8., 6., 0., -4.)], 100., .1)], 10.,
            requested_mps=.8, prediction_horizon_s=4., raw_clear_m=.5)
        self.assertTrue(r.emergency)
        self.assertEqual(r.cap_mps, 0.)

    def test_sensor_and_ego_motion_use_the_same_simulator_clock(self):
        batch = [('semantic', [person(12., 5., 0., -4.)], 100., .1)]
        real_time = PathSafety().check(PATH, dict(EGO, speed_kmh=30.), batch, 10.,
            requested_mps=30./3.6, motion_time_scale=1.)
        slow_sim = PathSafety().check(PATH, dict(EGO, speed_kmh=30.), batch, 10.,
            requested_mps=30./3.6, motion_time_scale=.5)
        self.assertTrue(real_time.emergency)
        self.assertFalse(slow_sim.emergency)

    def test_new_displacement_is_used_before_motion_confirmation(self):
        guard = PathSafety()
        self.check(guard, [person(13., -3., 0., 0.)], stamp=100., now=10.)
        r = self.check(guard, [person(13., -2.75, 0., 0.)], stamp=100.1, now=10.1)
        track = next(iter(guard.memory.tracks.values()))
        self.assertEqual(track['state'], 'unknown')
        self.assertGreater(track['velocity'][1], 2.)
        self.assertTrue(r.emergency)

    def test_sensor_delay_reduces_allowed_speed(self):
        obj = person(18., -2.5, 0., .35)
        fresh = self.check(PathSafety(), [obj], speed=20., age=.02)
        late = self.check(PathSafety(), [obj], speed=20., age=.4)
        self.assertLess(late.cap_mps, fresh.cap_mps)

    def test_rereading_one_observation_cannot_confirm_motion_or_refresh_age(self):
        guard = PathSafety()
        for k in range(5):
            self.check(guard, [person(12., -2.5, 0., 0.)], stamp=100., now=10.+.1*k)
        track = next(iter(guard.memory.tracks.values()))
        self.assertEqual(len(track['history']), 1)
        self.assertAlmostEqual(track['observed_at'], 9.9)
        self.assertEqual(track['state'], 'unknown')

    def test_fresh_associated_lidar_updates_geometry_without_confirming_motion(self):
        guard = PathSafety()
        obj = person(3.8, -1.8, 0., 0.)
        measured = [dict(uid=16, xy=[3.8, -1.8], stamp=100.2, age_s=.05)]
        r = guard.check(PATH, dict(EGO, speed_kmh=0.),
            [('semantic', [obj], 100., .4)], 10., requested_mps=30./3.6,
            measured_people=measured)
        self.assertFalse(r.emergency)
        self.assertGreater(r.cap_mps, .5)
        track = guard.memory.tracks[('semantic', 16)]
        self.assertEqual(len(track['history']), 1)
        self.assertEqual(track['state'], 'unknown')
        self.assertEqual(track['stamp'], 100.)
        r = guard.check(PATH, dict(EGO, speed_kmh=0.), [], 10.2, requested_mps=30./3.6)
        self.assertAlmostEqual(r.validation['people_geometry'][0]['age_s'], .25)
        self.assertEqual(len(track['history']), 1)

    def test_unmeasured_close_person_keeps_full_departure_stand_off(self):
        obj = person(3.8, -1.8, 0., 0.)
        guard = PathSafety()
        r = guard.check(PATH, dict(EGO, speed_kmh=0.),
            [('semantic', [obj], 100., .4)], 10., requested_mps=30./3.6)
        self.assertLess(r.cap_mps, .5)
        unmeasured_cap = r.cap_mps
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=0.),
            [('semantic', [obj], 100., .4)], 10., requested_mps=30./3.6,
            measured_people=[dict(uid=16, xy=[3.8, -1.8], stamp=99.9, age_s=.05)])
        self.assertEqual(r.cap_mps, unmeasured_cap)

    def test_association_cannot_waive_a_current_measured_body_collision(self):
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=0.),
            [('semantic', [person(3.8, -1.8, 0., 0.)], 100., .4)], 10.,
            requested_mps=30./3.6, raw_clear_m=.5,
            measured_people=[dict(uid=16, xy=[3.8, -1.8], stamp=100.2, age_s=.05)])
        self.assertTrue(r.emergency)
        self.assertEqual(r.cap_mps, 0.)

    def test_short_dropout_does_not_release_a_stop(self):
        guard = PathSafety()
        self.check(guard, [person(10., -2., 0., 0.)])
        self.assertTrue(self.check(guard, [], stamp=100.1, now=10.1).emergency)

    def test_repeated_clear_read_cannot_release_stop(self):
        guard = PathSafety()
        self.check(guard, [person(10., -2., 0., 0.)])
        clear = person(10., -8., 0., 0.)
        self.check(guard, [clear], stamp=100.2, now=10.2)
        self.assertTrue(self.check(guard, [clear], stamp=100.2, now=10.4).emergency)
        r = self.check(guard, [clear], stamp=100.5, now=10.5)
        self.assertFalse(r.emergency)
        self.assertGreater(r.cap_mps, 0.)

    def test_fresh_clear_observations_allow_same_path_restart(self):
        guard = PathSafety()
        self.check(guard, [person(10., 0., 0., 1.)])
        clear = person(10., 8., 0., 1.)
        self.check(guard, [clear], stamp=100.2, now=10.2)
        r = self.check(guard, [clear], stamp=100.4, now=10.4)
        self.assertFalse(r.emergency)
        self.assertGreater(r.cap_mps, 0.)

    def test_existing_bypass_is_checked_instead_of_global_route(self):
        obj = (15., 0., 1., 'box', 'obstacle', 0., 1., 1., 0., 0., 0., 77)
        direct = self.check(PathSafety(), [obj])
        bypass = PATH.copy()
        bypass[:, 1] = 3.5*np.clip(bypass[:, 0]/8., 0., 1.)
        avoiding = self.check(PathSafety(), [obj], path=bypass)
        self.assertTrue(direct.emergency)
        self.assertFalse(avoiding.emergency)

    def test_rear_person_cannot_slow_open_road(self):
        r = self.check(PathSafety(), [person(-12., -2., 0., .8)])
        self.assertFalse(r.emergency)
        self.assertAlmostEqual(r.cap_mps, 30./3.6)

    def test_completed_pass_is_not_blocked_by_slower_car_wholly_behind(self):
        car = (-6., 0., 4.8, 'lidar_car', 'npc', 4., 4.8, 1.9, 0., 4., 0.,
               42, 0., None, True)
        r = self.check(PathSafety(), [car], speed=36.)
        self.assertFalse(r.emergency)
        self.assertAlmostEqual(r.cap_mps, 30./3.6)
        self.assertEqual(r.validation['objects'], [])

    def test_same_speed_vehicle_uses_relative_braking_distance(self):
        car = (14., 0., 4.8, 'lidar_car', 'npc', 8., 4.8, 1.9, 0., 8., 0., 42, 0., None, True, 0.)
        r = self.check(PathSafety(), [car], speed=28.8)
        self.assertFalse(r.emergency)
        self.assertGreaterEqual(r.cap_mps, 8.)

    def test_unknown_vehicle_is_not_treated_as_confirmed_parked(self):
        car = (12., -3., 4.8, 'fusion_parked_vehicle', 'obstacle', 0., 4.8, 1.9, 90., 0., 0., 42)
        r = self.check(PathSafety(), [car])
        self.assertTrue(r.emergency)
        self.assertTrue(r.validation['objects'][0]['possible_start'])

    def test_parallel_parked_vehicle_cannot_block_a_clear_adjacent_bypass(self):
        guard = PathSafety()
        car = (5.6, 2.9, 4.8, 'fusion_parked_vehicle', 'obstacle', 0., 4.8, 2.08, 0., 0., 0., 7)
        for k in range(10):
            r = self.check(guard, [car], speed=0., stamp=100.+.1*k, now=10.+.1*k)
            self.assertFalse(r.emergency)
            self.assertGreater(r.cap_mps, 0.)

    def test_recorded_parked_car_bypass_remains_drivable_without_reclassification(self):
        data = json.loads((Path(__file__).parent/'test_data/parked_bypass_safety.json').read_text())
        r = PathSafety().check(data['path'], data['ego'],
            [('semantic', data['objects'], 100., .15)], 10., requested_mps=30./3.6,
            raw_clear_m=40., source_age_s=.1)
        self.assertFalse(r.emergency)
        self.assertAlmostEqual(r.cap_mps, 30./3.6)

    def test_person_standing_outside_path_can_release_stop_but_remains_movable(self):
        guard = PathSafety()
        for k in range(12):
            r = self.check(guard, [person(3.3, 3.2, 0., 0.)], speed=0.,
                           stamp=100.+.1*k, now=10.+.1*k)
        self.assertFalse(r.emergency)
        self.assertGreater(r.cap_mps, 0.)
        track = next(iter(guard.memory.tracks.values()))
        self.assertEqual(track['state'], 'standing')
        r = self.check(guard, [person(3.3, 2.9, 0., 0.)], speed=0., stamp=101.3, now=11.3)
        self.assertNotEqual(next(iter(guard.memory.tracks.values()))['state'], 'standing')
        self.assertGreater(np.linalg.norm(next(iter(guard.memory.tracks.values()))['velocity']), 0.)
        r = self.check(guard, [person(3.3, 1.6, 0., 0.)], speed=0., stamp=101.5, now=11.5)
        self.assertTrue(r.emergency)

    def test_raw_unclassified_hit_triggers_stop_without_any_tracks(self):
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=30.), [], 10.,
            requested_mps=30./3.6, raw_clear_m=8., source_age_s=.2)
        self.assertTrue(r.emergency)
        self.assertEqual(r.cap_mps, 0.)

    def test_small_braking_envelope_reduction_is_deceleration_not_full_stop(self):
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=52.8), [], 10.,
            requested_mps=55./3.6, raw_clear_m=49.902776, source_age_s=.288420)
        self.assertFalse(r.emergency)
        self.assertGreater(r.cap_mps, 0.)
        self.assertLess(r.cap_mps, 52.8/3.6)

    def test_recorded_static_braking_boundary_requests_deceleration_without_zero_pulse(self):
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=46.), [], 10.,
            requested_mps=55./3.6, raw_clear_m=33.46335865691093, source_age_s=.2)
        self.assertFalse(r.emergency)
        self.assertGreater(r.cap_mps, 0.)
        self.assertLess(r.cap_mps, 46./3.6)

    def test_fusion_motion_heading_jump_does_not_rotate_a_parked_car_into_clear_path(self):
        guard = PathSafety()
        parked = (4., 3.2, 4.8, 'fusion_parked_vehicle', 'obstacle', 0., 4.8, 1.9,
                  0., 0., 0., 7)
        guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [parked], 100., .1)],
                    10., requested_mps=30./3.6, dynamic_only=True)
        jitter = (4., 3.2, 4.8, 'fusion_npc', 'npc', .6, 4.8, 1.9, 90., 0., .6, 7)
        r = guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [jitter], 100.1, .1)],
                        10.1, requested_mps=30./3.6, dynamic_only=True)
        track = next(iter(guard.memory.tracks.values()))
        self.assertLess(abs(track['object'][8]), 7.)
        # 0.6 m/s is below the planner's moving threshold for vehicles.
        np.testing.assert_array_equal(track['velocity'], [0., 0.])
        self.assertFalse(r.emergency)
        # Body smoothing cannot suppress departure toward the same path.
        incoming = list(jitter)
        incoming[9:11] = [0., -2.]
        incoming[5] = 2.
        r = guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [incoming], 100.2, .1)],
                        10.2, requested_mps=30./3.6, dynamic_only=True)
        self.assertTrue(r.emergency)

    def test_raw_cluster_box_jitter_cannot_override_the_measured_motion_tracker(self):
        guard = PathSafety()
        for k in range(10):
            obj = (12.+.2*k, 4., .8, 'tunnel_lidar_1', 'obstacle', 0., .8, .8,
                   0., 0., 0., 800001, dict(point_count=3, motion_center=[12., 4.]))
            r = guard.check(PATH, dict(EGO, speed_kmh=20.),
                [('tunnel_lidar', [obj], 100.+.1*k, .1)], 10.+.1*k, requested_mps=30./3.6)
        track = next(iter(guard.memory.tracks.values()))
        np.testing.assert_array_equal(track['velocity'], np.zeros(2))
        self.assertFalse(r.emergency)

    def test_supported_raw_departure_retains_measured_velocity(self):
        obj = (12., 3., .8, 'tunnel_lidar_dynamic_800001', 'pedestrian', 2.,
               .8, .8, 0., 0., -2., 800001, dict(point_count=12, motion_center=[12., 3.]))
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=30.),
            [('tunnel_lidar', [obj], 100., .1)], 10., requested_mps=30./3.6)
        self.assertTrue(r.emergency)

    def test_dynamic_only_keeps_static_geometry_with_existing_bypass_checker(self):
        obj = (8., 0., .8, 'fusion_obstacle', 'obstacle', 0., .8, .8, 0., 0., 0., 77)
        guard = PathSafety()
        r = guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [obj], 100., .1)],
                        10., requested_mps=30./3.6, dynamic_only=True)
        self.assertFalse(r.emergency)
        self.assertEqual(r.cap_mps, 30./3.6)
        self.assertEqual(len(guard.memory.tracks), 1)
        # Raw geometry still stops on an actual body collision.
        r = guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [obj], 100., .1)],
                        10., requested_mps=30./3.6, dynamic_only=True, raw_clear_m=5.)
        self.assertTrue(r.emergency)

    def test_concrete_size_does_not_create_a_second_uncertain_vehicle_margin(self):
        obj = (10., 1.5, 3., 'fusion_obstacle', 'obstacle', 0., 3., 1.5, 0., 0., 0., 77)
        r = PathSafety().check(PATH, EGO, [('semantic', [obj], 100., .1)],
            10., requested_mps=30./3.6, dynamic_only=True)
        self.assertFalse(r.emergency)
        self.assertEqual(r.validation['objects'], [])
        self.assertEqual(r.cap_mps, 30./3.6)

    def test_unclassified_object_departure_is_used_before_class_confirmation(self):
        guard = PathSafety()
        obj = (6., 4., .8, 'unclassified_cluster', 'obstacle', 0., .8, .8, 0., 0., 0., 77)
        guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [obj], 100., .1)],
                    10., requested_mps=30./3.6, dynamic_only=True)
        moved = list(obj)
        moved[1] = 3.
        r = guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [moved], 100.1, .1)],
                        10.1, requested_mps=30./3.6, dynamic_only=True)
        self.assertEqual(next(iter(guard.memory.tracks.values()))['state'], 'unknown')
        self.assertTrue(r.emergency)

    def test_static_fusion_box_jitter_cannot_revive_rejected_motion(self):
        # Concrete-like box from the 404_contest recording: fusion reports zero
        # after checking both observed bounds, while its padded centre jitters.
        samples = [(0., -59.894283, -140.860702), (.087, -60.116601, -140.804289),
                   (.239, -60.113968, -140.739009), (.327, -60.132454, -140.880865),
                   (.489, -60.191629, -140.886974), (.566, -60.119183, -140.675250),
                   (.625, -60.099728, -140.787617), (.705, -60.191857, -140.804639)]
        guard = PathSafety()
        for dt, x, y in samples:
            obj = (x, y, 1.1, 'fusion_obstacle', 'obstacle', 0., 1.1, 1.1,
                   103., 0., 0., 7)
            r = guard.check(PATH, EGO, [('semantic', [obj], 100.+dt, .1)],
                            10.+dt, requested_mps=30./3.6, dynamic_only=True)
            np.testing.assert_array_equal(next(iter(guard.memory.tracks.values()))['velocity'], [0., 0.])
            self.assertEqual(r.validation['objects'], [])

    def test_filtered_unclassified_departure_is_used_without_class_confirmation(self):
        obj = (12., 3., .8, 'fusion_obstacle', 'obstacle', 2., .8, .8, 0., 0., -2., 77)
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=30.),
            [('semantic', [obj], 100., .1)], 10., requested_mps=30./3.6, dynamic_only=True)
        self.assertTrue(r.emergency)
        self.assertEqual(r.validation['objects'][0]['state'], 'unknown')

    def test_fusion_class_id_change_replaces_old_ghost_but_keeps_strong_departure(self):
        guard = PathSafety()
        car = (10., 3.2, 4.8, 'fusion_npc', 'npc', .6, 4.8, 1.9, 0., 0., -.6, 8)
        guard.check(PATH, EGO, [('semantic', [car], 100., .1)], 10., requested_mps=8., dynamic_only=True)
        parked = (10., 3.2, 4.8, 'fusion_parked_vehicle', 'obstacle', 0., 4.8, 1.9, 0., 0., 0., 7)
        r = guard.check(PATH, EGO, [('semantic', [parked], 100.1, .1)], 10.1, requested_mps=8., dynamic_only=True)
        self.assertEqual(set(guard.memory.tracks), {('semantic', 7)})
        self.assertEqual(r.validation['objects'], [])
        moved = list(car)
        moved[5], moved[9], moved[10], moved[11] = 2., 0., -2., 9
        guard.check(PATH, EGO, [('semantic', [moved], 100.2, .1)], 10.2, requested_mps=8., dynamic_only=True)
        parked = list(parked)
        parked[1] -= .2
        r = guard.check(PATH, EGO, [('semantic', [parked], 100.3, .1)], 10.3, requested_mps=8., dynamic_only=True)
        self.assertGreater(np.linalg.norm(guard.memory.tracks[('semantic', 7)]['velocity']), 1.)
        self.assertTrue(r.validation['objects'])

    def test_class_dropout_does_not_refresh_old_velocity_evidence(self):
        memory = MotionMemory()
        moving = (10., 3.2, 4.8, 'fusion_npc', 'npc', 2., 4.8, 1.9, 0., 0., -2., 8)
        memory.update('semantic', [moving], 100., .1, 10.)
        for index in range(1, 6):
            row = list(moving)
            row[3], row[4] = ('fusion_parked_vehicle', 'obstacle') if index%2 else ('fusion_npc', 'npc')
            row[5], row[9], row[10], row[11] = 0., 0., 0., 8+index
            memory.update('semantic', [row], 100.+index*.1, .1, 10.+index*.1)
            self.assertEqual(len(memory.tracks), 1)
        track = next(iter(memory.tracks.values()))
        np.testing.assert_array_equal(track['velocity'], [0., 0.])
        self.assertIn(8, track['identity_ids'])

    def test_first_fused_motion_is_used_now_but_not_forecast_after_disappearance(self):
        guard = PathSafety()
        ego = dict(EGO, speed_kmh=30.)
        obj = (12., 3.2, 1.1, 'fusion_obstacle', 'obstacle', 2.,
               1.1, 1.1, -90., 0., -2., 155)
        current = guard.check(PATH, ego, [('semantic', [obj], 100., .1)], 10.,
                              requested_mps=8., dynamic_only=True)
        self.assertTrue(current.validation['objects'])
        self.assertLess(current.cap_mps, 8.)
        missing = guard.check(PATH, ego, [('semantic', [], 100.1, .1)], 10.1,
                              requested_mps=8., dynamic_only=True, raw_clear_m=60.)
        track = guard.memory.tracks[('semantic', 155)]
        np.testing.assert_array_equal(track['velocity'], [0., 0.])
        self.assertEqual(track['object'][:2], obj[:2])
        self.assertEqual(track['stamp'], 100.)
        self.assertEqual(track['observed_at'], 9.9)
        self.assertEqual(missing.validation['objects'], [])
        cleared = guard.check(PATH, ego, [('semantic', [], 100.21, .1)], 10.21,
                              requested_mps=8., dynamic_only=True, raw_clear_m=60.)
        self.assertFalse(cleared.emergency)
        self.assertEqual(cleared.cap_mps, 8.)
        blocked = guard.check(PATH, ego, [], 10.3, requested_mps=8.,
                              dynamic_only=True, raw_clear_m=.5)
        self.assertTrue(blocked.emergency)

    def test_repeated_fused_motion_keeps_forecasting_a_short_dropout(self):
        memory = MotionMemory()
        obj = (12., 3.2, 1.1, 'fusion_obstacle', 'obstacle', 2.,
               1.1, 1.1, -90., 0., -2., 155)
        for i in range(2):
            row = list(obj)
            row[1] -= .2*i
            memory.update('semantic', [row], 100.+.1*i, .1, 10.+.1*i)
        memory.update('semantic', [], 100.2, .1, 10.2)
        track = memory.tracks[('semantic', 155)]
        self.assertTrue(track['motion_confirmed'])
        np.testing.assert_array_equal(track['velocity'], [0., -2.])
        self.assertEqual(track['stamp'], 100.1)

    def test_disappeared_first_fused_mover_keeps_its_last_occupied_body(self):
        guard = PathSafety()
        ego = dict(EGO, speed_kmh=30.)
        obj = (8., 0., 1.1, 'fusion_obstacle', 'obstacle', 2.,
               1.1, 1.1, 90., 0., 2., 155)
        guard.check(PATH, ego, [('semantic', [obj], 100., .1)], 10., requested_mps=8.)
        missing = guard.check(PATH, ego, [('semantic', [], 100.1, .1)], 10.1,
                              requested_mps=8., raw_clear_m=60.)
        self.assertTrue(missing.emergency)
        self.assertEqual(missing.validation['objects'][0]['uid'], 155)
        self.assertLess(missing.clear_m, 8.)

    def test_duplicate_or_discontinuous_reports_cannot_confirm_fused_dropout_motion(self):
        obj = (12., 3.2, 1.1, 'fusion_obstacle', 'obstacle', 2.,
               1.1, 1.1, -90., 0., -2., 155)
        for second_stamp in (100., 100.4):
            with self.subTest(second_stamp=second_stamp):
                memory = MotionMemory()
                memory.update('semantic', [obj], 100., .1, 10.)
                memory.update('semantic', [obj], second_stamp, .1, 10.4)
                memory.update('semantic', [], second_stamp+.1, .1, 10.5)
                np.testing.assert_array_equal(memory.tracks[('semantic', 155)]['velocity'], [0., 0.])

    def test_first_fused_person_motion_keeps_forecasting_a_short_dropout(self):
        memory = MotionMemory()
        walker = (12., 3.2, .6, 'fusion_pedestrian', 'pedestrian', 1.2,
                  .6, .6, -90., 0., -1.2, 37)
        memory.update('semantic', [walker], 100., .1, 10.)
        memory.update('semantic', [], 100.1, .1, 10.1)
        np.testing.assert_array_equal(memory.tracks[('semantic', 37)]['velocity'], [0., -1.2])

    def test_verified_lidar_velocity_is_not_replaced_by_box_centre_jitter(self):
        memory = MotionMemory()
        for i, x in enumerate((10., 10.4, 10.6, 10.9)):
            obj = (x, 3.2, 4.8, 'lidar_car', 'npc', 0., 4.8, 1.9, 0., 0., 0., 9, 0., None, True)
            memory.update('vehicles', [obj], 100.+i*.1, .1, 10.+i*.1)
        np.testing.assert_array_equal(memory.tracks[('vehicles', 9)]['velocity'], [0., 0.])
        self.assertNotEqual(memory.tracks[('vehicles', 9)]['state'], 'moving')
        row = list(obj)
        row[5], row[10] = 2., -2.
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=30.), [('vehicles', [row], 100.4, .1)],
                               10.4, requested_mps=30./3.6, dynamic_only=True)
        self.assertTrue(r.emergency)

    def test_unconfirmed_lidar_car_still_uses_new_displacement(self):
        guard = PathSafety()
        obj = (12., 3.2, 4.8, 'lidar_car', 'npc', 0., 4.8, 1.9, 0., 0., 0., 9, 0., None, False)
        guard.check(PATH, EGO, [('vehicles', [obj], 100., .1)], 10., requested_mps=8., dynamic_only=True)
        row = list(obj)
        row[1] -= .4
        r = guard.check(PATH, dict(EGO, speed_kmh=30.), [('vehicles', [row], 100.1, .1)],
                        10.1, requested_mps=30./3.6, dynamic_only=True)
        self.assertTrue(r.emergency)

    def test_fusion_person_zero_without_velocity_valid_cannot_erase_displacement(self):
        memory = MotionMemory()
        for i, x in enumerate((10., 10.4, 10.6, 10.9)):
            obj = (x, 3.2, .6, 'fusion_pedestrian', 'pedestrian', 0., .6, .6, 0., 0., 0., 9)
            memory.update('semantic', [obj], 100.+i*.11, .1, 10.+i*.11)
        self.assertGreater(np.linalg.norm(memory.tracks[('semantic', 9)]['velocity']), 1.)
        row = list(obj)
        row[5], row[10] = 2., -2.
        memory.update('semantic', [row], 100.44, .1, 10.44)
        np.testing.assert_array_equal(memory.tracks[('semantic', 9)]['velocity'], [0., -2.])

    def test_parked_car_surface_fragment_cannot_block_clear_bypass_but_departure_can(self):
        body = (4., 3.2, 4.8, 'fusion_parked_vehicle', 'obstacle', 0., 4.8, 1.9, 0., 0., 0., 7)
        fragment = (4., 2.9, 1.1, 'fusion_obstacle', 'obstacle', .6, 1.1, 1.1, 0., 0., -.6, 11)
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [body, fragment], 100., .1)],
                               10., requested_mps=30./3.6, dynamic_only=True)
        self.assertFalse(r.emergency)
        for y, speed in ((2.9, 2.), (1.8, 1.2)):
            moving = list(fragment)
            moving[1], moving[5], moving[10] = y, speed, -speed
            r = PathSafety().check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [body, moving], 100., .1)],
                                   10., requested_mps=30./3.6, dynamic_only=True)
            self.assertTrue(r.emergency)
        # Below 1 m/s the planner bypasses the same report as a static box.
        slow = list(fragment)
        slow[1] = 1.8
        r = PathSafety().check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [body, slow], 100., .1)],
                               10., requested_mps=30./3.6, dynamic_only=True)
        self.assertFalse(r.emergency)

    def test_stopping_cap_includes_response_and_increases_with_clearance(self):
        self.assertEqual(stop_cap(0., .4), 0.)
        self.assertLess(stop_cap(10., .4), stop_cap(10., .1))
        self.assertLess(stop_cap(10., .4), stop_cap(20., .4))


def fused(x, y, uid, name='fusion_obstacle', kind='obstacle', size=(1.1, 1.1),
          heading=90., velocity=(0., 0.)):
    """A world-frame /Object_topic row as gui.RosObjectReceiver builds it."""
    vx, vy = velocity
    return (x, y, max(size), name, kind, float(np.hypot(vx, vy)), size[0], size[1],
            heading, vx, vy, uid, 0., None)


def replay(guard, frames, objects, world_path, requested_mps):
    """Recorded live checks: (now, ego x, y, heading, km/h, semantic stamp,
    raw LiDAR clearance, LiDAR age), as LivePathSafety calls the guard."""
    results = []
    for now, x, y, heading, kmh, stamp, raw_clear, lidar_age in frames:
        h = math.radians(heading)
        path = (np.asarray(world_path)-[x, y]) @ np.array([[math.cos(h), -math.sin(h)],
                                                            [math.sin(h), math.cos(h)]])
        ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=kmh)
        results.append(guard.check(path, ego, [('semantic', objects[stamp], stamp, now-stamp)], now,
            requested_mps=requested_mps, raw_clear_m=raw_clear, source_age_s=lidar_age,
            observation_ego=ego, dynamic_only=True))
    return results


def offset_path(hold_x, hold_until_y, route_x, rejoined_y):
    """Bypass held beside the obstacle, then a cosine rejoin to the route (north)."""
    y = np.arange(-170., -30., .5)
    blend = np.clip((y-hold_until_y)/(rejoined_y-hold_until_y), 0., 1.)
    return np.column_stack((hold_x+(route_x-hold_x)*(1.-np.cos(np.pi*blend))/2., y))


class RecordedFusedMotionTests(unittest.TestCase):
    """Static objects passed by a validated bypass, from the 2026-10-09 E2E runs.

    Ego poses, speeds, sensor stamps, raw LiDAR clearance and object positions
    are the logged values (student_gui_nosave.jsonl; times are seconds after
    run.json started_wall_ns). The published path is not logged; it is rebuilt
    from the logged ego track (and static_hold_until_m where logged).
    """
    # follow-20261009T204445Z 100.9-102.0 s, stamps -1791578785 s. A fused box
    # inside the parked car's footprint, reported at rest for 0.41 s, was
    # reported moving once at 786.412 s and then left the fusion output. The
    # logged check: 7 frames 'live path collision: fusion_obstacle' (gap 0,
    # ages .145-.567 s) and 2 frames 'waiting for fresh clear observations',
    # while the raw LiDAR clearance on the same path was 55-57 m.
    PASS_FRAMES = [
        (.988, -57.10, -77.71, 89.46, 29.4, .776, 34.48, .094),
        (1.083, -57.10, -77.23, 89.60, 29.4, .879, 34.00, .013),
        (1.178, -57.10, -76.76, 89.73, 29.4, .966, 59.50, .115),
        (1.305, -57.10, -75.97, 89.89, 29.5, 1.119, 58.78, .103),
        (1.368, -57.09, -75.47, 89.95, 29.5, 1.186, 58.28, .165),
        (1.503, -57.09, -74.64, 90.04, 27.6, 1.352, 57.46, .140),
        (1.559, -57.08, -74.10, 90.06, 26.6, 1.412, 57.41, .061),
        (1.633, -57.08, -73.70, 90.10, 25.6, 1.412, 57.00, .127),
        (1.694, -57.08, -73.70, 90.10, 25.6, 1.488, 56.50, .037),
        (1.753, -57.09, -73.17, 90.33, 24.5, 1.488, 55.99, .095),
        (1.846, -57.09, -72.71, 90.75, 22.2, 1.565, 55.50, .021),
        (1.909, -57.10, -72.43, 90.96, 20.7, 1.738, 55.26, .090),
        (1.985, -57.10, -72.15, 91.15, 18.7, 1.738, 55.48, .165),
        (2.071, -57.11, -71.82, 91.35, 16.5, 1.802, 55.00, .067),
        (2.120, -57.10, -71.60, 91.52, 14.4, 1.887, 54.93, .133)]

    def pass_objects(self, confirmed=False):
        car = lambda x, y: fused(x, y, 7, 'fusion_parked_vehicle', size=(4.6, 1.9))
        rest = lambda x, y: fused(x, y, 11)
        # Not logged: the moving report's box (beside the bumper, as gap 0
        # requires) and its 2 m/s toward the ego's side.
        onset = fused(-58.9, -74.6, 11, heading=0., velocity=(2., 0.))
        objects = {
            .776: [car(-60.07, -74.38), rest(-60.68, -75.38)],
            .879: [car(-60.08, -74.41), rest(-60.69, -75.40)],
            .966: [car(-60.09, -73.95), rest(-60.69, -74.84)],
            1.119: [car(-60.20, -73.76), rest(-60.69, -74.84)],
            1.186: [car(-60.19, -73.77), rest(-60.69, -74.84)],
            1.352: [car(-60.09, -73.24)],
            1.412: [car(-60.20, -72.90), onset],
            1.488: [car(-60.28, -72.51)]}
        for stamp in (1.565, 1.738, 1.802, 1.887):
            objects[stamp] = [car(-60.28, -72.51)]
        if confirmed:
            objects[1.488] = objects[1.488]+[fused(-58.75, -74.6, 11, heading=0., velocity=(2., 0.))]
        return objects

    def test_recorded_box_face_change_while_passing_does_not_stop_or_latch(self):
        guard = PathSafety()
        results = replay(guard, self.PASS_FRAMES, self.pass_objects(),
                         offset_path(-57.10, -68., -59.93, -48.), 21.5/3.6)
        for frame, r in zip(self.PASS_FRAMES, results):
            self.assertFalse(r.emergency, (frame[0], r.reason))
            self.assertAlmostEqual(r.cap_mps, 21.5/3.6, msg=frame[0])
        onset = [m for m in results[6].validation['motion_states'] if m['uid'] == 11]
        self.assertEqual(onset, [dict(uid=11, state='unknown', stamp=1.412, fusion_mps=2., onset=True)])
        self.assertFalse(guard.stopped)

    def test_repeated_report_of_the_same_start_still_stops(self):
        results = replay(PathSafety(), self.PASS_FRAMES, self.pass_objects(confirmed=True),
                         offset_path(-57.10, -68., -59.93, -48.), 21.5/3.6)
        self.assertFalse(any(r.emergency for r in results[:8]))
        self.assertTrue(results[8].emergency)
        self.assertEqual(results[8].reason, 'live path collision: fusion_obstacle')
        self.assertEqual(results[8].cap_mps, 0.)

    def test_recorded_parked_car_relabelled_moving_for_two_reports_does_not_stop(self):
        # follow-20261009T183356Z 117.3-118.9 s, stamps -1791570953 s. Car 8
        # stood for 1.3 s, then was listed as fusion_npc at 0.94 m/s for two
        # reports within its face jitter (heading ~11 deg off the road, from
        # the logged 1.31/1.40 m half-width): 3 collision frames + 1 waiting.
        frames = [
            (.411, -58.92, -85.22, 82.94, 24.3, .144, 52.0, .030),
            (.490, -58.87, -84.93, 82.67, 24.5, .270, 51.5, .114),
            (.554, -58.81, -84.42, 82.40, 24.8, .356, 51.5, .176),
            (.633, -58.81, -84.42, 82.40, 24.8, .464, 51.0, .026),
            (.777, -58.68, -83.49, 81.93, 25.3, .574, 50.78, .172),
            (.836, -58.68, -83.49, 81.93, 25.3, .697, 50.28, .030),
            (.973, -58.60, -82.87, 81.73, 25.8, .788, 49.5, .168),
            (1.044, -58.54, -82.49, 81.70, 26.0, .867, 49.77, .073),
            (1.165, -58.48, -82.11, 81.71, 26.2, .955, 49.39, .185),
            (1.265, -58.38, -81.47, 81.75, 26.4, 1.049, 48.5, .085),
            (1.414, -58.31, -80.93, 81.82, 26.7, 1.157, 47.5, .227),
            (1.465, -58.24, -80.49, 81.94, 26.9, 1.282, 47.5, .071),
            (1.541, -58.19, -80.07, 82.10, 27.0, 1.372, 47.33, .144),
            (1.673, -58.12, -79.64, 82.33, 27.2, 1.473, 46.89, .071),
            (1.750, -58.12, -79.64, 82.33, 27.2, 1.578, 46.39, .156),
            (1.844, -58.04, -79.18, 82.58, 27.3, 1.682, 45.92, .025),
            (1.926, -57.97, -78.64, 82.91, 27.5, 1.682, 45.38, .107),
            (1.977, -57.92, -78.24, 83.18, 26.6, 1.793, 45.47, .161),
            (2.072, -57.92, -78.24, 83.18, 26.6, 1.899, 44.97, .042)]
        parked = lambda x, y: [fused(x, y, 8, 'fusion_parked_vehicle', size=(4.6, 1.9), heading=90.3)]
        listed = lambda x, y: [fused(x, y, 8, 'fusion_npc', 'npc', (4.6, 1.9), 79.,
                                     (.94*math.cos(math.radians(79.)), .94*math.sin(math.radians(79.))))]
        positions = {.144: (-60.12, -74.59), .270: (-60.04, -74.40), .356: (-60.03, -74.67),
                     .464: (-60.03, -74.67), .574: (-60.02, -74.41), .697: (-60.08, -74.81),
                     .788: (-60.01, -74.48), .867: (-60.04, -74.79), .955: (-60.13, -74.69),
                     1.049: (-60.18, -75.05), 1.157: (-60.03, -74.62), 1.282: (-59.96, -74.09),
                     1.372: (-59.98, -74.07), 1.473: (-60.07, -74.64), 1.793: (-59.92, -73.87),
                     1.899: (-60.04, -73.99)}
        objects = {stamp: parked(*xy) for stamp, xy in positions.items()}
        objects[1.578], objects[1.682] = listed(-60.01, -74.18), listed(-60.06, -74.35)
        y = np.arange(-170., -30., .5)
        world = offset_path(-57.45, -70., -59.93, -56.)
        world[:, 0] = np.where(y < -75., -57.45-.143*(-75.-y), world[:, 0])
        guard = PathSafety()
        results = replay(guard, frames, objects, world, 30./3.6)
        for frame, r in zip(frames, results):
            self.assertFalse(r.emergency, (frame[0], r.reason))
        car = guard.memory.tracks[('semantic', 8)]
        self.assertAlmostEqual(car['object'][8], 90.3)
        self.assertNotEqual(car['state'], 'moving')

    def test_slow_reports_on_measured_static_objects_follow_the_planner_threshold(self):
        # Recorded fused speeds on objects that stayed put: a box 0.87 m/s
        # (20261009T191227Z 97.7 s) and a parked car 0.56 / 0.94 m/s
        # (20261009T204445Z 394.7 s, 20261009T183356Z 118.6 s).
        cases = [(fused(12., 2.5, 7, heading=0.), .87, 'fusion_obstacle', 'obstacle', (1.1, 1.1)),
                 (fused(12., 2.9, 8, 'fusion_parked_vehicle', size=(4.6, 1.9), heading=0.), .56,
                  'fusion_npc', 'npc', (4.6, 1.9)),
                 (fused(12., 2.9, 8, 'fusion_parked_vehicle', size=(4.6, 1.9), heading=0.), .94,
                  'fusion_npc', 'npc', (4.6, 1.9))]
        for rest, speed, name, kind, size in cases:
            with self.subTest(name=name, speed=speed):
                guard = PathSafety()
                for k in range(6):
                    r = guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [rest], 100.+.1*k, .1)],
                                    10.+.1*k, requested_mps=30./3.6, dynamic_only=True)
                    self.assertFalse(r.emergency)
                report = fused(rest[0], rest[1], rest[11], name, kind, size, -90., (0., -speed))
                for k in range(6, 9):
                    r = guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [report], 100.+.1*k, .1)],
                                    10.+.1*k, requested_mps=30./3.6, dynamic_only=True)
                    self.assertFalse(r.emergency, r.reason)
                    self.assertAlmostEqual(r.cap_mps, 30./3.6)
                # The same object really moving toward the path stops the car
                # once its start from rest has been reported twice.
                moving = fused(rest[0], rest[1], rest[11], name, kind, size, -90., (0., -1.2))
                first = guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [moving], 101., .1)],
                                    11., requested_mps=30./3.6, dynamic_only=True)
                self.assertFalse(first.emergency)
                moved = list(moving)
                moved[1] -= .12
                second = guard.check(PATH, dict(EGO, speed_kmh=30.), [('semantic', [tuple(moved)], 101.1, .1)],
                                     11.1, requested_mps=30./3.6, dynamic_only=True)
                self.assertTrue(second.emergency)
                self.assertEqual(second.reason, 'live path collision: '+name)

    def test_real_start_reported_around_the_threshold_still_stops(self):
        guard = PathSafety()
        ego = dict(EGO, speed_kmh=30.)
        for k in range(6):
            r = guard.check(PATH, ego, [('semantic', [fused(12., 2.5, 7)], 100.+.1*k, .1)],
                            10.+.1*k, requested_mps=30./3.6, dynamic_only=True)
        results = []
        for k, speed in enumerate((1.1, .95, 1.05)):
            report = fused(12., 2.4-.1*k, 7, heading=-90., velocity=(0., -speed))
            results.append(guard.check(PATH, ego, [('semantic', [report], 100.6+.1*k, .1)],
                                       10.6+.1*k, requested_mps=30./3.6, dynamic_only=True))
        self.assertEqual([r.emergency for r in results], [False, False, True])
        self.assertEqual(results[-1].reason, 'live path collision: fusion_obstacle')

    def test_recorded_last_report_jump_is_not_forecast_after_the_box_disappears(self):
        # follow-20261009T183356Z 95.6-96.6 s, stamps -1791570931 s: a box at
        # rest for 0.46 s jumped 0.7 m toward the ego with 1.3 m/s in its last
        # report, then left the fusion output; 3 collision frames + 1 waiting.
        # The 1.09 s report reached a timer recheck between logged frames, so
        # that pose is interpolated; the other box in the scene is omitted.
        rest = lambda x, y: [fused(x, y, 4)]
        objects = {.498: rest(-60.06, -140.77), .723: rest(-59.96, -140.87),
                   .956: rest(-60.05, -140.85), 1.09: [fused(-59.97, -141.56, 4, heading=-83.6,
                       velocity=(1.3*math.cos(math.radians(-83.6)), 1.3*math.sin(math.radians(-83.6))))],
                   1.231: [], 1.34: [], 1.456: [], 1.643: []}
        frames = [(.717, -60.87, -170.65, 84.04, 39.4, .498, 58.32, .197),
                  (.858, -60.81, -170.04, 84.24, 39.4, .723, 58.32, .115),
                  (1.199, -60.63, -168.11, 84.79, 38.2, .956, 24.0, .222),
                  (1.24, -60.61, -167.80, 84.83, 37.6, 1.09, 24.0, .15),
                  (1.354, -60.57, -167.41, 84.89, 37.0, 1.231, 58.32, .108),
                  (1.499, -60.46, -166.13, 85.00, 34.5, 1.34, 58.32, .027),
                  (1.689, -60.41, -165.57, 85.02, 33.4, 1.456, 58.82, .027),
                  (1.855, -60.34, -164.78, 85.08, 30.8, 1.643, 58.82, .197)]
        route = np.column_stack((np.linspace(-60.65, -59.83, 200), np.linspace(-175., -120., 200)))
        results = replay(PathSafety(), frames, objects, route, 16.0/3.6)
        for frame, r in zip(frames, results):
            self.assertFalse(r.emergency, (frame[0], r.reason))

    def test_walking_person_still_stops_with_a_slow_obstacle_duplicate(self):
        # follow-20261009T185440Z 143.3-143.6 s: a person stepping toward the
        # path is also fused as a slow fusion_obstacle; the person stops us.
        guard = PathSafety()
        for k in range(6):
            walker = fused(6., 2.4-.09*k, 37, 'fusion_pedestrian', 'pedestrian', (.6, .6), -90., (0., -.9))
            twin = fused(6., 2.4-.09*k, 36, heading=-90., velocity=(0., -.6))
            r = guard.check(PATH, dict(EGO, speed_kmh=13.7), [('semantic', [walker, twin], 100.+.1*k, .1)],
                            10.+.1*k, requested_mps=30./3.6, dynamic_only=True)
        self.assertTrue(r.emergency)
        self.assertEqual(r.reason, 'live path collision: fusion_pedestrian')
        self.assertEqual({o['uid'] for o in r.validation['objects']}, {37})


class CoveredVehicleRestartTests(unittest.TestCase):
    @staticmethod
    def car(uid=42, *, x=4., y=4., valid=True, prediction_age=0.):
        return (x, y, 4.9, 'lidar_car', 'npc', 4., 4.9, 2.1, -90.,
                0., -4., uid, 0., None, valid, prediction_age)

    def check(self, guard, cars=(), *, semantic=(), keys=(), raw=60.,
              now=10., stamp=100., age=.05, raw_age=0.):
        return guard.check(PATH, dict(EGO, speed_kmh=0.),
            [('vehicles', cars, stamp, age), ('semantic', semantic, stamp, age)],
            now, requested_mps=6., raw_clear_m=raw, source_age_s=raw_age,
            timed_vehicle_keys=keys)

    def stop(self, guard, cars=None, **kwargs):
        result = self.check(guard, [self.car()] if cars is None else cars, **kwargs)
        self.assertTrue(result.emergency)
        self.assertEqual(result.cap_mps, 0.)
        return result

    def test_fresh_complete_vehicle_proof_releases_on_the_first_clear_scan(self):
        guard = PathSafety()
        self.stop(guard)
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.05, stamp=100.05)
        self.assertFalse(result.emergency)
        self.assertEqual(result.cap_mps, 6.)
        self.assertTrue(result.validation['covered_vehicle_restart'])

    def test_confirmed_moving_semantic_alias_can_use_its_own_proof_key(self):
        def alias(y):
            row = list(self.car(y=y))
            row[3], row[14] = 'fusion_npc', None
            return tuple(row)

        guard = PathSafety()
        # Establish actual repeated semantic motion while the short executed
        # command keeps the future crossing outside our travel envelope.
        for index in range(4):
            result = guard.check(PATH, dict(EGO, speed_kmh=0.),
                [('semantic', [alias(10.-.4*index)], 100.+.1*index, .05)],
                10.+.1*index, requested_mps=.1, raw_clear_m=60.)
            self.assertFalse(result.emergency)
        self.stop(guard, [], semantic=[alias(8.4)], now=10.4, stamp=100.4)
        result = self.check(guard, [], semantic=[alias(8.2)],
            keys={('fusion_npc', 42)}, now=10.45, stamp=100.45)
        self.assertFalse(result.emergency)
        self.assertTrue(result.validation['covered_vehicle_restart'])

    def test_only_exact_fresh_static_alias_keys_can_replace_the_future_body_check(self):
        cases = [('fusion_parked_vehicle', 'semantic', .05, True),
                 ('fusion_obstacle', 'semantic', .05, False),
                 ('box', 'semantic', .05, False),
                 ('fusion_parked_vehicle', 'other', .05, False),
                 ('fusion_parked_vehicle', 'semantic', .4, False)]
        for name, source, age, allowed in cases:
            with self.subTest(name=name, source=source, age=age):
                obj = (12., 0., 4.6, name, 'obstacle', 0., 4.6, 2.1, 0.,
                       0., 0., 24, 0., None, None, 0.)
                result = PathSafety().check(PATH, dict(EGO, speed_kmh=30.),
                    [(source, [obj], 100., age)], 10., requested_mps=6., raw_clear_m=60.,
                    timed_vehicle_keys={(name, 24)})
                self.assertEqual(result.emergency, not allowed)
        # A fresh alias key replaces only its duplicate future box. Independent
        # raw physical contact still stops, and its prior static stop keeps the latch.
        guard = PathSafety()
        obj = (3., 0., 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 2.1,
               0., 0., 0., 24, 0., None, None, 0.)
        first = self.check(guard, [], semantic=[obj])
        self.assertTrue(first.emergency)
        held = self.check(guard, [], semantic=[obj], keys={('fusion_parked_vehicle', 24)},
                          now=10.05, stamp=100.05)
        self.assertTrue(held.emergency)
        contact = self.check(PathSafety(), [], semantic=[obj],
                             keys={('fusion_parked_vehicle', 24)}, raw=.5)
        self.assertTrue(contact.emergency)

    def test_coverage_must_include_every_prior_stopping_car(self):
        for keys, released in (({('lidar_car', 42)}, False),
                               ({('lidar_car', 42), ('lidar_car', 43)}, True)):
            with self.subTest(keys=keys):
                guard = PathSafety()
                self.stop(guard, [self.car(), self.car(43)])
                # Both cars are now outside the generic forecast; it is the
                # prior two-car cause set that requires complete coverage.
                result = self.check(guard, [self.car(y=30.), self.car(43, y=30.)],
                    keys=keys, now=10.05, stamp=100.05)
                self.assertEqual(result.emergency, not released)

    def test_unrelated_or_expired_caller_proof_keeps_two_scan_restart(self):
        for keys in ((), {('lidar_car', 43)}, {('fusion_npc', 42)}):
            with self.subTest(keys=keys):
                guard = PathSafety()
                self.stop(guard)
                result = self.check(guard, [self.car(y=30.)], keys=keys,
                                    now=10.05, stamp=100.05)
                self.assertTrue(result.emergency)
                self.assertEqual(result.reason, 'live path waiting for fresh clear observations')

    def test_current_raw_contact_cannot_be_released_by_a_vehicle_proof(self):
        guard = PathSafety()
        self.stop(guard)
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)}, raw=.5,
                            now=10.05, stamp=100.05)
        self.assertTrue(result.emergency)
        self.assertEqual(result.cap_mps, 0.)
        # The independent stop must also replace the old vehicle provenance.
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.1, stamp=100.1)
        self.assertTrue(result.emergency)

    def test_prior_raw_or_mixed_stop_retains_clear_observation_requirement(self):
        for cars in ([], [self.car()]):
            with self.subTest(mixed=bool(cars)):
                guard = PathSafety()
                self.stop(guard, cars, raw=.5)
                result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                                    now=10.05, stamp=100.05)
                self.assertTrue(result.emergency)

    def test_later_vehicle_stop_cannot_erase_an_unreleased_independent_stop(self):
        guard = PathSafety()
        self.stop(guard, [], raw=.5)
        self.stop(guard, now=10.05, stamp=100.05)
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.1, stamp=100.1)
        self.assertTrue(result.emergency)
        self.assertFalse(result.validation['covered_vehicle_restart'])

    def test_independent_clear_accumulates_while_vehicle_forecast_still_stops(self):
        guard = PathSafety()
        self.stop(guard, [], raw=.5)
        first = self.stop(guard, now=10.05, stamp=100.05)
        self.assertTrue(first.validation['independent_stop_pending'])
        self.assertEqual(first.validation['independent_clear_samples'], 1)
        second = self.stop(guard, now=10.2, stamp=100.2)
        self.assertFalse(second.validation['independent_stop_pending'])
        self.assertEqual(second.validation['independent_clear_samples'], 2)
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.25, stamp=100.25)
        self.assertFalse(result.emergency)
        self.assertGreater(result.cap_mps, 0.)
        self.assertTrue(result.validation['covered_vehicle_restart'])

    def test_repeated_source_cannot_complete_independent_clear_evidence(self):
        guard = PathSafety()
        self.stop(guard, [], raw=.5)
        self.stop(guard, now=10.05, stamp=100.05)
        self.stop(guard, now=10.2, stamp=100.05)
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.25, stamp=100.05)
        self.assertTrue(result.emergency)
        self.assertFalse(result.validation['covered_vehicle_restart'])

    def test_new_raw_urgent_resets_independent_clear_progress(self):
        guard = PathSafety()
        self.stop(guard, [], raw=.5)
        self.stop(guard, now=10.05, stamp=100.05)
        self.stop(guard, raw=.5, now=10.2, stamp=100.2)
        self.stop(guard, now=10.25, stamp=100.25)
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.3, stamp=100.3)
        self.assertTrue(result.emergency)
        self.assertFalse(result.validation['covered_vehicle_restart'])

    def test_stale_raw_cannot_supply_independent_clear_evidence(self):
        guard = PathSafety()
        self.stop(guard, [], raw=.5)
        self.stop(guard, now=10.05, stamp=100.05)
        self.stop(guard, raw_age=.4, now=10.2, stamp=100.2)
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.25, stamp=100.25)
        self.assertTrue(result.emergency)
        self.assertFalse(result.validation['covered_vehicle_restart'])

    def test_retained_person_prevents_independent_clear_during_vehicle_stops(self):
        guard = PathSafety()
        self.stop(guard, semantic=[person(2., 0., 0., 0.)])
        self.stop(guard, now=10.05, stamp=100.05)
        self.stop(guard, now=10.2, stamp=100.2)
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.25, stamp=100.25)
        self.assertTrue(result.emergency)

    def test_cleared_independent_barrier_does_not_replace_a_missing_vehicle_proof(self):
        guard = PathSafety()
        self.stop(guard, [], raw=.5)
        self.stop(guard, now=10.05, stamp=100.05)
        self.stop(guard, now=10.2, stamp=100.2)
        self.stop(guard, now=10.25, stamp=100.25)
        first = self.check(guard, [self.car(y=30.)], now=10.3, stamp=100.3)
        self.assertTrue(first.emergency)
        second = self.check(guard, [self.car(y=30.)], now=10.45, stamp=100.45)
        self.assertFalse(second.emergency)
        self.assertFalse(second.validation['covered_vehicle_restart'])

    def test_external_stop_resets_already_accumulated_independent_clear(self):
        guard = PathSafety()
        self.stop(guard, [], raw=.5)
        self.stop(guard, now=10.05, stamp=100.05)
        self.stop(guard, now=10.2, stamp=100.2)
        guard.stopped = True
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.25, stamp=100.25)
        self.assertTrue(result.emergency)

    def test_person_or_static_mixed_stop_cannot_gain_vehicle_restart(self):
        blockers = [person(2., 0., 0., 0.),
                    (2., 0., 1., 'box', 'obstacle', 0., 1., 1., 0., 0., 0., 77)]
        for blocker in blockers:
            with self.subTest(name=blocker[3]):
                guard = PathSafety()
                self.stop(guard, semantic=[blocker])
                moved = list(blocker)
                moved[1] = 30.
                result = self.check(guard, [self.car()], semantic=[tuple(moved)],
                    keys={('lidar_car', 42)}, now=10.05, stamp=100.05)
                self.assertTrue(result.emergency)
                self.assertFalse(result.validation['covered_vehicle_restart'])

    def test_unknown_prediction_or_stale_motion_has_no_restart_provenance(self):
        variants = [(self.car(valid=False), .05),
                    (self.car(prediction_age=.1), .05), (self.car(), .4)]
        for car, age in variants:
            with self.subTest(valid=car[14], prediction=car[15], age=age):
                guard = PathSafety()
                self.stop(guard, [car], age=age)
                result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                                    now=10.05, stamp=100.05)
                self.assertTrue(result.emergency)

    def test_current_uncovered_car_keeps_its_urgent_forecast(self):
        guard = PathSafety()
        self.stop(guard)
        result = self.check(guard, [self.car(), self.car(43)],
            keys={('lidar_car', 42)}, now=10.05, stamp=100.05)
        self.assertTrue(result.emergency)
        self.assertEqual(result.cap_mps, 0.)

    def test_external_stop_discards_old_vehicle_only_provenance(self):
        guard = PathSafety()
        self.stop(guard)
        guard.stopped = True  # A caller's physical-contact/invalid-source stop.
        result = self.check(guard, [self.car()], keys={('lidar_car', 42)},
                            now=10.05, stamp=100.05)
        self.assertTrue(result.emergency)
        self.assertFalse(result.validation['covered_vehicle_restart'])

    def test_missing_provenance_fails_closed_and_ordinary_restart_still_works(self):
        guard = PathSafety()
        guard.stopped = True
        first = self.check(guard, [self.car(y=30.)], keys={('lidar_car', 42)})
        self.assertTrue(first.emergency)
        second = self.check(guard, [self.car(y=30.)], keys={('lidar_car', 42)},
                            now=10.15, stamp=100.15)
        self.assertFalse(second.emergency)
        self.assertFalse(second.validation['covered_vehicle_restart'])


if __name__ == '__main__':
    unittest.main()
