"""Static passing pace retains local curvature and stopping constraints."""
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from tunnel_runtime import curve_speed_cap


class StaticPassSpeedTests(unittest.TestCase):
    @staticmethod
    def path(turn_at):
        x = np.arange(0., 100.01, .25)
        u = np.clip((x-turn_at)/12., 0., 1.)
        return np.column_stack((x, 1.6*u**3*(10.-15.*u+6.*u*u)))

    def plan(self, path, **changes):
        return dict(dict(path=path, curvature=float(max(gui.path_curvatures(path))),
            speed_cap_mps=20., safe_horizon_m=15., hold_until_m=50.,
            clear_distance_m=100., fast_static_clear_m=100., kind='bypass'), **changes)

    def cap(self, plan, speed=5.):
        return gui.model_static_speed_cap_mps(55./3.6, plan, speed_mps=speed)

    def test_distant_return_bend_does_not_set_the_whole_straight_passing_speed(self):
        plan = self.plan(self.path(35.))
        old = gui.static_speed_cap_mps(55./3.6, plan)
        new = self.cap(plan)
        self.assertGreater(new, old+2.)
        self.assertLessEqual(new, plan['speed_cap_mps'])
        self.assertLess(self.cap(self.plan(self.path(1.))), new-2.)

    def test_near_curve_keeps_the_same_lateral_acceleration_limit(self):
        plan = self.plan(self.path(0.))
        expected, station = curve_speed_cap(gui.RosPathPublisher.resample(plan['path']),
            10., alat_mps2=gui.BYPASS_ALAT_MPS2, decel_mps2=1.5, reaction_s=.7)
        self.assertAlmostEqual(self.cap(plan, 10.), expected)
        self.assertLess(station, 10.)
        self.assertLess(self.cap(plan, 10.), 8.)

    def test_clearance_and_steering_rate_caps_remain_hard_limits(self):
        path = self.path(35.)
        for cap in (.7, 2., 4.):
            with self.subTest(cap=cap):
                plan = self.plan(path, speed_cap_mps=cap)
                self.assertLessEqual(self.cap(plan), cap)

    def test_checked_straight_return_reaches_thirty_only_with_full_wide_clearance(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        path = np.column_stack((np.arange(0., 80.1, .5), np.zeros(161)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=18., wheel_angle=0.)
        previous = self.plan(path, canonical_control=True)
        plan = ma.road_checked_continuation(previous, ego, lambda p: p,
            lambda p, horizon: horizon)
        self.assertTrue(ma.static_plan_complete(plan))
        np.testing.assert_array_equal(plan['path'], path)
        plan['fast_static_clear_m'] = ma.fast_static_clearance(plan, ego, [], np.empty((0, 2)))
        self.assertAlmostEqual(self.cap(plan)*3.6, 30.)

        # The same ordinary 25 cm pass cannot use the higher ceiling when
        # either independent 60 cm clearance check finds insufficient room.
        for objects, points in (([], np.array([[18., 1.3]])),
                                ([(18., 1.8, .8, 'box', 'obstacle', 0., .8, .8, 0.)],
                                 np.empty((0, 2)))):
            plan['fast_static_clear_m'] = ma.fast_static_clearance(plan, ego, objects, points)
            self.assertLess(plan['fast_static_clear_m'], 35.)
            self.assertAlmostEqual(self.cap(plan)*3.6, 25.)

    def test_higher_road_ceiling_preserves_tighter_explicit_caps_and_curve_limits(self):
        for path in (self.path(0.), self.path(35.)):
            plan = self.plan(path, ordinary_speed_cap_mps=25./3.6,
                             speed_cap_mps=30./3.6)
            for cap in (.7, 2., 15./3.6):
                limited = dict(plan, speed_cap_mps=cap)
                self.assertLessEqual(self.cap(limited), cap)
            curve_cap, _ = curve_speed_cap(gui.RosPathPublisher.resample(path),
                5., alat_mps2=gui.BYPASS_ALAT_MPS2, decel_mps2=1.5, reaction_s=.7)
            self.assertLessEqual(self.cap(plan), curve_cap)
            self.assertLessEqual(self.cap(plan), 30./3.6)

    def test_incomplete_and_standing_person_paths_keep_the_existing_speed(self):
        path = self.path(35.)
        for changes in (dict(clear_distance_m=30.), dict(standing_bypass_ids=[13]),
                        dict(fast_static_clear_m=10.), dict(fast_static_clear_m=0.)):
            with self.subTest(changes=changes):
                plan = self.plan(path, **changes)
                self.assertEqual(self.cap(plan), gui.static_speed_cap_mps(55./3.6, plan))

    def test_no_validated_path_keeps_the_original_stopping_envelope(self):
        for plan in (None, dict(blocked=True)):
            for distance in (0., 4., 10., 40.):
                self.assertEqual(gui.model_static_speed_cap_mps(55./3.6, plan, distance,
                    speed_mps=10.), gui.blocked_static_stop_cap_mps(distance))

    def test_recorded_static_paths_keep_the_checked_geometry_and_speed_ceiling(self):
        for name in ('road_static_padding_20261015.npz', 'road_static_fusion_748_20261016.npz'):
            with self.subTest(name=name), np.load(Path(__file__).parent/'test_data'/name) as data:
                path = data['path'].copy()
                plan = self.plan(path, clear_distance_m=60., hold_until_m=20.)
                original = path.copy()
                new = self.cap(plan)
                self.assertGreaterEqual(new, gui.static_speed_cap_mps(55./3.6, plan))
                self.assertLessEqual(new, plan['speed_cap_mps'])
                np.testing.assert_array_equal(path, original)

    def test_replanned_approach_reaches_the_curve_without_exceeding_its_local_limit(self):
        path = self.path(35.)
        stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        speed, distance, dt = 10., 0., .02
        while distance < 60.:
            at = min(len(path)-4, int(np.searchsorted(stations, distance)))
            suffix = path[at:]-path[at]
            target = self.cap(self.plan(suffix), speed)
            acceleration = max(-1.5, min(2.5, (target-speed)/dt))
            speed = max(0., speed+acceleration*dt)
            distance += speed*dt
            local = gui.path_curvatures(gui.RosPathPublisher.resample(suffix))
            self.assertLessEqual(speed*speed*float(local[0]), gui.BYPASS_ALAT_MPS2+.1)

    def test_actual_road_inference_supplies_measured_speed_to_the_static_envelope(self):
        from test_road_forward_progress import RecordedRoadForwardProgress
        fixture = RecordedRoadForwardProgress()
        fixture.setUpClass()
        runtime = fixture.runtime()
        ego = dict(fixture.initial, speed_kmh=10.)
        fixture.observe(runtime, ego, 1_000_000., 1)
        with patch('time.monotonic', return_value=1_000_000.), \
                patch('time.time', return_value=1_000_000.), \
                patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch('gui.model_static_speed_cap_mps', wraps=gui.model_static_speed_cap_mps) as cap:
            runtime.infer()
        self.assertTrue(cap.called)
        self.assertAlmostEqual(cap.call_args.kwargs['speed_mps'], 10./3.6)

    def test_narrow_raw_pass_keeps_the_old_speed_even_when_the_ordinary_path_is_clear(self):
        path = self.path(35.)
        plan = self.plan(path)
        ego = dict(x=0., y=0., heading_deg=0.)
        points = np.array([[18., 1.3]])
        from tunnel_runtime import swept_clearance
        self.assertGreaterEqual(swept_clearance(path, points, 35., .25,
            lateral_margin=.25, body_shift_m=ma.REAR_AXLE_SHIFT_M), 35.)
        plan['fast_static_clear_m'] = ma.fast_static_clearance(plan, ego, [], points)
        self.assertLess(plan['fast_static_clear_m'], 35.)
        self.assertEqual(self.cap(plan), gui.static_speed_cap_mps(55./3.6, plan))

    def test_wide_pass_requires_independent_raw_and_semantic_clearance(self):
        path = self.path(35.)
        plan = self.plan(path)
        ego = dict(x=0., y=0., heading_deg=0.)
        empty = np.empty((0, 2))
        self.assertGreaterEqual(ma.fast_static_clearance(plan, ego, [], empty), 50.)
        close_box = (18., 1.8, .8, 'measured_box', 'obstacle', 0., .8, .8, 0.)
        self.assertLess(ma.fast_static_clearance(plan, ego, [close_box], empty), 35.)
        for points in (None, np.array([[float('nan'), 0.]])):
            self.assertEqual(ma.fast_static_clearance(plan, ego, [], points), 0.)

    def test_recorded_tight_static_pass_cannot_use_the_faster_curve_envelope(self):
        with np.load(Path(__file__).parent/'test_data/road_static_padding_20261015.npz') as data:
            import json
            ego = json.loads(str(data['ego']))
            objects = ma.road_static_objects(json.loads(str(data['objects'])), ego, data['points'])
            plan = self.plan(data['path'], clear_distance_m=60., hold_until_m=20.)
            plan['fast_static_clear_m'] = ma.fast_static_clearance(plan, ego, objects, data['points'])
            self.assertLess(plan['fast_static_clear_m'], 35.)
            self.assertEqual(self.cap(plan), gui.static_speed_cap_mps(55./3.6, plan))


if __name__ == '__main__':
    unittest.main()
