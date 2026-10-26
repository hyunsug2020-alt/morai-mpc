"""A local curve command and an independent future cap have different meanings."""
import json
import math
from dataclasses import replace
import unittest
from unittest.mock import patch

import numpy as np

from driving_plan import DrivingPlan
from highway_planner import world_points
from live_path_safety import static_motion_source_freshness
from static_lane_pass import StaticLanePass
from test_static_initial_join_regression import ROOT, corridor
import test_static_initial_join_regression as initial
import test_live_path_safety as live_fixtures


class StaticMotionContractTests(unittest.TestCase):
    setUp = initial.RecordedInitialJoinTests.setUp

    def runtime(self):
        with np.load(ROOT/'test_data/static_close_stopped_20261023.npz',
                     allow_pickle=False) as fixture:
            r = json.loads(str(fixture['record']))
        policy = StaticLanePass()
        for key, value in r['policy'].items():
            setattr(policy, key, corridor(value) if key in ('legal', 'physical') else value)
        previous = (dict(r['previous'], path=np.asarray(r['previous']['path']))
                    if r['previous'] is not None else None)
        candidate, proof = policy.plan(self.lanes, r['ego'], np.asarray(r['route']),
            r['objects'], r['static'], np.asarray(r['points']), corridor(r['legal']),
            previous, r['now'], r['time_scale'], True, budget_s=.11,
            **{key: r[key] for key in ('side_margin_m', 'contact', 'legal_checked',
                                       'reserve_source_bounds', 'slalom_first')})
        self.assertIsNotNone(candidate, {k: v for k, v in proof.items() if k != 'replay'})
        self.assertTrue(candidate['static_borrow'])
        fixture = live_fixtures.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        ego = dict(r['ego'], stamp=100., age_s=.01, received=1000.)
        runtime.test_status['ego'] = ego
        runtime.test_scan.update(stamp=99.95, age_s=.05,
                                 obstacle_points=np.asarray(r['points']))
        runtime.object_receiver.objects = []
        runtime.object_receiver.observation_stamp = 99.95
        runtime.lanes = self.lanes
        runtime._static_lane_pass = policy
        provider = runtime.path_publisher._safety_provider
        old_local_cap = candidate['speed_cap_mps']*.7
        original = DrivingPlan(1, candidate['path'], ego, ego, 99.95, .05, 1000.,
            old_local_cap, candidate['clear_distance_m'], False,
            dict(valid=True, pose_valid=True, steering_valid=True, static_rear_axle=True,
                 static_complete=True, static_lane_borrow=True, static_motion_profile=True,
                 static_profile_ceiling_mps=candidate['static_profile_ceiling_mps'],
                 static_profile_source_limit_mps=None), 'STUDENT FORCED')
        return runtime, provider, original, world_points(candidate['path'], ego)

    def test_fresh_motion_profile_replaces_only_the_old_local_curve_command(self):
        runtime, provider, original, world = self.runtime()
        with patch('time.monotonic', return_value=1000.):
            checked = provider(world, original.control_ego, original.mode, original)
            self.assertFalse(checked.emergency, checked.stop_reason)
            self.assertGreater(checked.allowed_mps, original.allowed_mps)
            self.assertTrue(checked.validation['static_lane_pass']['launch_ready'])
            bound, target = provider.bound_static_lane_pass(checked, world, checked.allowed_mps)
        self.assertFalse(bound.emergency, bound.stop_reason)
        self.assertGreater(target, 0.)

    def test_an_independent_body_cap_cannot_borrow_the_profile_future_speed(self):
        runtime, provider, original, world = self.runtime()
        with patch('time.monotonic', return_value=1000.):
            provider(world, original.control_ego, original.mode, original)
        real_check = provider.guard.check
        def limited(*args, **kwargs):
            return replace(real_check(*args, **kwargs), cap_mps=.1, emergency=False)
        with patch('time.monotonic', return_value=1000.), \
                patch.object(provider.guard, 'check', side_effect=limited):
            checked = provider(world, original.control_ego, original.mode, original)
        self.assertTrue(checked.emergency)
        self.assertEqual(checked.allowed_mps, 0.)

    def test_a_late_cap_or_expired_source_still_stops_before_publication(self):
        runtime, provider, original, world = self.runtime()
        with patch('time.monotonic', return_value=1000.):
            checked = provider(world, original.control_ego, original.mode, original)
            self.assertFalse(checked.emergency, checked.stop_reason)
            bound, target = provider.bound_static_lane_pass(checked, world, .1)
        self.assertTrue(bound.emergency)
        self.assertEqual(target, 0.)
        with patch('time.monotonic', return_value=1000.4):
            bound, target = provider.bound_static_lane_pass(checked, world, checked.allowed_mps)
        self.assertTrue(bound.emergency)
        self.assertEqual(target, 0.)

    def test_nearly_expired_motion_proof_rechecks_one_new_complete_scan(self):
        for case in ('fresh', 'same_source', 'new_contact'):
            with self.subTest(case=case):
                runtime, provider, original, world = self.runtime()
                fresh = dict(runtime.test_scan)
                if case == 'new_contact':
                    fresh['obstacle_points'] = np.array([[3.8, 0.]])
                old = dict(runtime.test_scan, stamp=99.72, age_s=.28, sequence=1)
                snapshots = [0]
                def scan():
                    snapshots[0] += 1
                    return old if snapshots[0] == 1 or case == 'same_source' else fresh
                runtime.lidar.scan_snapshot = scan
                with patch('time.monotonic', return_value=1000.):
                    checked = provider(world, original.control_ego, original.mode, original)
                if case == 'same_source':
                    self.assertNotIn('static_source_retry', checked.validation)
                    self.assertEqual(checked.sensor_stamp, 99.72)
                    self.assertLess(checked.validation['static_profile_deadline'], 1000.02)
                else:
                    self.assertEqual(checked.sensor_stamp, fresh['stamp'])
                    self.assertEqual(checked.validation['static_source_retry'],
                        dict(from_stamp=99.72, to_stamp=99.95))
                    if case == 'new_contact':
                        self.assertTrue(checked.emergency, checked.stop_reason)
                        self.assertEqual(checked.allowed_mps, 0.)
                    else:
                        self.assertFalse(checked.emergency, checked.stop_reason)
                        self.assertGreater(checked.allowed_mps, 0.)
                        self.assertGreater(checked.validation['static_profile_deadline'], 1000.1)
    def test_live_publication_checks_old_stationary_labels_against_current_raw_geometry(self):
        runtime, provider, original, world = self.runtime()
        with np.load(ROOT/'test_data/static_close_stopped_20261023.npz',
                     allow_pickle=False) as fixture:
            rows = json.loads(str(fixture['record']))['objects']
        runtime.object_receiver.objects = rows
        runtime.object_receiver.age_s = .4
        runtime.object_receiver.observation_stamp = 99.6
        with patch('time.monotonic', return_value=1000.):
            checked = provider(world, original.control_ego, original.mode, original)
            self.assertFalse(checked.emergency, checked.stop_reason)
            sources = checked.validation['static_profile_sources']
            self.assertTrue(sources['ready'])
            self.assertTrue(sources['sources'][0]['current_raw_association'])
            bound, target = provider.bound_static_lane_pass(checked, world, checked.allowed_mps)
        self.assertFalse(bound.emergency, bound.stop_reason)
        self.assertGreater(target, 0.)


class StaticSourceFreshnessTests(unittest.TestCase):
    def check(self, *, age=.4, scan_age=.1, points=((8., 0.),), kind='obstacle',
              speed=0., elapsed=0., heading=0., origin=(0., 0.)):
        angle = math.radians(heading)
        axes = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
        world = np.array([8., 0.])@axes.T+origin
        obj = (*world, 1., 'measured_body', kind, speed, 1., 1., heading)
        ego = dict(x=origin[0], y=origin[1], heading_deg=heading)
        return static_motion_source_freshness(points, ego,
            [('semantic', [obj], 100.-age, age)], scan_age, 1000., 1000.+elapsed)

    def test_current_raw_geometry_can_check_a_still_valid_stationary_label(self):
        for heading, origin in ((0., (0., 0.)), (57., (-80., 200.))):
            with self.subTest(heading=heading):
                proof = self.check(heading=heading, origin=origin)
                self.assertTrue(proof['ready'])
                self.assertAlmostEqual(proof['expires_at'], 1000.2)
                self.assertEqual(proof['sources'][0]['stamp'], 99.6)
                self.assertTrue(proof['sources'][0]['current_raw_association'])

    def test_no_association_movers_and_expired_sources_keep_the_stop(self):
        for change in (dict(points=()), dict(points=((8., 3.),)), dict(kind='npc'),
                       dict(kind='pedestrian'), dict(speed=1.), dict(age=.61),
                       dict(scan_age=.31), dict(elapsed=.21)):
            with self.subTest(change=change):
                self.assertFalse(self.check(**change)['ready'])

    def test_small_fusion_face_velocity_uses_the_existing_static_guard_threshold(self):
        self.assertTrue(self.check(speed=1.4/3.6)['ready'])

    def test_empty_semantic_absence_does_not_acquire_a_longer_lifetime(self):
        proof = static_motion_source_freshness([[8., 0.]],
            dict(x=0., y=0., heading_deg=0.), [('semantic', [], 99.6, .4)],
            .1, 1000., 1000.)
        self.assertFalse(proof['ready'])


if __name__ == '__main__':
    unittest.main()
