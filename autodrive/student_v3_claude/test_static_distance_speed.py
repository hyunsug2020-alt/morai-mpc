"""Replay the measured static return that imposed a distant bend's crawl pace."""
import json
import math
from pathlib import Path
import unittest

import numpy as np

import gui
from static_lane_pass import (StaticLanePass, _path_steering_profile,
                              measured_initial_join, static_speed_profile)
import test_static_lane_pass as lane_fixture


class StaticDistanceSpeedTests(unittest.TestCase):
    def setUp(self):
        data = json.loads((Path(__file__).parent/'test_data/static_slow_return_20261021.json').read_text())
        self.frames = data['frames']

    def path(self, frame):
        ego = frame['ego']
        angle = math.radians(ego['heading_deg'])
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        path = (np.asarray(frame['world_path'])-[ego['x'], ego['y']]) @ rotation
        path = np.vstack((np.zeros((1, 2)), path[np.flatnonzero(path[:, 0] > .05)[0]:]))
        return gui.RosPathPublisher.control_geometry(path, path,
            dict(mode='LATTICE', validated_static=True))

    def test_current_recorded_frames_keep_the_later_return_out_of_the_current_crawl_cap(self):
        for frame in self.frames:
            with self.subTest(wall=frame['wall']):
                self.assertLess(frame['stamp_error_s'], .03)
                ego, path = frame['ego'], self.path(frame)
                _, old_rate = _path_steering_profile(path, ego)
                old_cap = math.radians(30.)*.8/max(old_rate)
                self.assertLess(old_cap*3.6, 11.)
                checked = measured_initial_join(path, ego,
                    max_curvature=gui.MAX_CONTROL_CURVATURE,
                    lateral_accel=gui.BYPASS_ALAT_MPS2, target_mps=30./3.6)
                profile = static_speed_profile(checked, ego)
                self.assertGreaterEqual(profile['cap_mps']*3.6, 25.)
                self.assertLessEqual(profile['cap_mps']*3.6, 30.+1e-6)
                self.assertGreater(profile['steering_station_m'], 15.)

                plan = dict(path=checked, kind='bypass', generator='road_static_pass',
                    curvature=max(gui.path_curvatures(checked)),
                    speed_cap_mps=profile['cap_mps'], clear_distance_m=55.,
                    safe_horizon_m=4., hold_until_m=40.,
                    ordinary_speed_cap_mps=25./3.6, fast_static_clear_m=55.,
                    static_speed_profile=profile, static_borrow=False)
                cap = gui.model_static_speed_cap_mps(55./3.6, plan,
                    speed_mps=ego['speed_kmh']/3.6)
                self.assertGreaterEqual(cap*3.6, 25.)
                # A new nearby width restriction retains the ordinary ceiling.
                plan['fast_static_clear_m'] = 0.
                self.assertLessEqual(gui.model_static_speed_cap_mps(55./3.6, plan,
                    speed_mps=ego['speed_kmh']/3.6), 25./3.6)

    def test_a_near_turn_retains_the_same_lateral_acceleration_and_wheel_rate_limits(self):
        x = np.arange(0., 70.01, .5)
        def path(start):
            u = np.clip((x-start)/12., 0., 1.)
            return np.column_stack((x, 1.6*u**3*(10.-15.*u+6.*u*u)))
        ego = dict(speed_kmh=30., wheel_angle=0.)
        distant = static_speed_profile(path(35.), ego)
        near = static_speed_profile(path(0.), ego)
        self.assertAlmostEqual(distant['cap_mps']*3.6, 30.)
        self.assertLess(near['cap_mps']*3.6, 22.)
        self.assertLess(near['cap_mps'], distant['cap_mps'])
        self.assertLessEqual(near['cap_mps'], near['curve_cap_mps'])
        self.assertLessEqual(near['cap_mps'], near['steering_cap_mps'])

    def test_certified_raw_tracking_room_does_not_duplicate_the_semantic_size_reserve(self):
        frame = self.frames[0]
        path = measured_initial_join(self.path(frame), frame['ego'],
            max_curvature=gui.MAX_CONTROL_CURVATURE, lateral_accel=gui.BYPASS_ALAT_MPS2,
            target_mps=30./3.6)
        profile = dict(static_speed_profile(path, frame['ego']), raw_clear_m=55.)
        plan = dict(path=path, kind='bypass', generator='road_static_pass',
            curvature=max(gui.path_curvatures(path)), speed_cap_mps=profile['cap_mps'],
            clear_distance_m=55., safe_horizon_m=4., hold_until_m=40.,
            ordinary_speed_cap_mps=25./3.6, fast_static_clear_m=0.,
            static_speed_profile=profile, static_borrow=False)
        self.assertAlmostEqual(gui.model_static_speed_cap_mps(55./3.6, plan,
            speed_mps=frame['ego']['speed_kmh']/3.6)*3.6, 30.)
        profile['raw_clear_m'] = 0.
        self.assertAlmostEqual(gui.model_static_speed_cap_mps(55./3.6, plan,
            speed_mps=frame['ego']['speed_kmh']/3.6)*3.6, 25.)

    def test_borrowed_and_person_passes_cannot_use_the_in_lane_speed_profile(self):
        frame = self.frames[0]
        path = self.path(frame)
        plan = dict(path=path, kind='bypass', generator='road_static_pass',
            curvature=max(gui.path_curvatures(path)), speed_cap_mps=30./3.6,
            clear_distance_m=55., safe_horizon_m=4., hold_until_m=40.,
            fast_static_clear_m=0., ordinary_speed_cap_mps=25./3.6,
            static_speed_profile=dict(cap_mps=30./3.6))
        for changes in (dict(static_borrow=True), dict(standing_bypass_ids=[7]),
                        dict(generator='other')):
            guarded = dict(plan, **changes)
            self.assertEqual(gui.model_static_speed_cap_mps(55./3.6, guarded,
                speed_mps=5.), gui.static_speed_cap_mps(55./3.6, guarded))

    def test_new_front_occupancy_still_rejects_the_complete_fast_pass(self):
        fixture = lane_fixture.StaticLanePassTests()
        fixture.setUp()
        plan, _ = StaticLanePass().plan(fixture.lanes, fixture.ego, fixture.route,
            [], [], np.array([[3.9, 0.]]), fixture.legal, None, 100., budget_s=1.)
        self.assertIsNone(plan)


if __name__ == '__main__':
    unittest.main()
