"""Road safety shares one measured scan frame and never renews its source age."""
import copy
import json
import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from highway_planner import NearFieldMemory
from perception.road_scan_frame import align_road_scan, align_road_scan_wait
from perception.stationary_semantics import metadata
import test_avoidance_gate as fixtures
import test_live_path_safety as live_fixtures


def pose(stamp=100., x=0., y=0., heading_deg=0., **extra):
    return dict(stamp=stamp, x=x, y=y, heading_deg=heading_deg,
                speed_kmh=36., measured=True, **extra)


def rotation(deg):
    yaw = math.radians(deg)
    return np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])


class RoadScanFrameTests(unittest.TestCase):
    def frame(self, points=None, source=None, target=None, **changes):
        source = source or pose()
        target = target or dict(source)
        scan = dict(stamp=source['stamp'], age_s=.01, sequence=7,
                    obstacle_points=np.array([[5., 1.], [-2., 0.]]) if points is None else points)
        scan.update(changes)
        def lookup(stamp):
            return source if stamp == source['stamp'] else target if stamp == target['stamp'] else None
        return align_road_scan(scan, target, lookup, now_wall=100.1,
                               now_mono=20.02, captured_at=20.)

    def test_translation_rotation_preserves_all_points_and_source_identity(self):
        source, target = pose(100., 10., 20., 90.), pose(99.9, 10., 19.5, 85.)
        points = np.array([[5., 1.], [-2., 0.]])
        result = self.frame(points, source, target)
        self.assertTrue(result['proof']['valid'], result)
        expected = (points@rotation(90.).T+[10., 20.]-[10., 19.5])@rotation(85.)
        np.testing.assert_allclose(result['points'], expected, atol=1e-12)
        self.assertEqual(result['proof']['stamp'], 100.)
        self.assertEqual(result['proof']['sequence'], 7)
        self.assertAlmostEqual(result['proof']['age_s'], .03)
        self.assertFalse(result['points'].flags.writeable)
        np.testing.assert_array_equal(points, [[5., 1.], [-2., 0.]])
        self.assertTrue(self.frame(np.empty((0, 2)))['proof']['valid'])

    def test_missing_invalid_stale_and_discontinuous_proofs_fail_closed(self):
        cases = [dict(age_s=.31), dict(stamp=99.),
                 dict(obstacle_points=np.array([[np.nan, 0.]])),
                 dict(obstacle_points=np.zeros((1, 3)))]
        for changes in cases:
            with self.subTest(changes=changes):
                self.assertFalse(self.frame(**changes)['proof']['valid'])
        for target in (dict(pose(), measured=False), dict(pose(), src='file'),
                       pose(99.9, 10., 0.), pose(99.9, 0., 0., 90.)):
            with self.subTest(target=target):
                self.assertFalse(self.frame(target=target)['proof']['valid'])
        scan = dict(stamp=100., age_s=.01, sequence=1, obstacle_points=np.empty((0, 2)))
        for lookup in (None, lambda stamp: None, lambda stamp: pose(99.9)):
            checked = align_road_scan(scan, pose(), lookup, now_wall=100.1,
                                     now_mono=1., captured_at=1.)
            self.assertFalse(checked['proof']['valid'])

    def test_target_label_needs_a_matching_measured_history_witness(self):
        scan = dict(stamp=100., age_s=.01, sequence=1, obstacle_points=np.empty((0, 2)))
        target = pose(99.9, .2)
        for witness in (None, pose(99.9, .3), pose(99.8, .2), pose(99.9, .2, 0., 1.)):
            lookup = lambda stamp: pose() if stamp == 100. else witness
            result = align_road_scan(scan, target, lookup, now_wall=100.1,
                                    now_mono=1., captured_at=1.)
            self.assertFalse(result['proof']['valid'])

    def test_finite_input_overflow_is_rejected(self):
        extreme = pose(100., 1.7e308, 1.7e308)
        result = self.frame(np.array([[1.7e308, 1.7e308]]), extreme, extreme)
        self.assertFalse(result['proof']['valid'])
        self.assertEqual(result['proof']['reason'], 'road scan transform invalid')

    def test_bracket_retry_reuses_original_scan_and_accumulates_age(self):
        clock = [100.01]
        scan = dict(stamp=100., age_s=.01, sequence=7, obstacle_points=np.array([[5., 1.]]))
        before = copy.deepcopy(scan)
        calls = []
        def lookup(stamp):
            calls.append(stamp)
            return pose() if clock[0] >= 100.016 else None
        def sleep(seconds):
            clock[0] += seconds
        with patch('perception.road_scan_frame.time.time', side_effect=lambda: clock[0]), \
                patch('perception.road_scan_frame.time.monotonic', side_effect=lambda: clock[0]), \
                patch('perception.road_scan_frame.time.sleep', side_effect=sleep):
            result = align_road_scan_wait(scan, pose(), lookup, captured_at=100.01)
        self.assertTrue(result['proof']['valid'], result)
        self.assertGreaterEqual(result['proof']['age_s'], .016-1e-9)
        self.assertEqual(result['proof']['stamp'], before['stamp'])
        self.assertEqual(result['proof']['sequence'], before['sequence'])
        self.assertEqual(scan['age_s'], before['age_s'])
        np.testing.assert_array_equal(scan['obstacle_points'], before['obstacle_points'])
        clock[:] = [100.29]
        scan['age_s'] = .295
        with patch('perception.road_scan_frame.time.time', side_effect=lambda: clock[0]), \
                patch('perception.road_scan_frame.time.monotonic', side_effect=lambda: clock[0]), \
                patch('perception.road_scan_frame.time.sleep', side_effect=sleep):
            result = align_road_scan_wait(scan, pose(), lambda stamp: None, captured_at=100.29)
        self.assertFalse(result['proof']['valid'])
        self.assertEqual(result['proof']['reason'], 'road scan source stale')
        self.assertLessEqual(clock[0], 100.311)

    def test_measured_callback_delays_fit_the_bounded_wait_without_time_renewal(self):
        for delay in (.05, .073, .091):
            with self.subTest(delay=delay):
                clock = [100.01]
                scan = dict(stamp=100., age_s=.01, sequence=71,
                            obstacle_points=np.array([[8., 2.]]))
                receiver = SimpleNamespace(lock=threading.RLock(), samples=deque())
                def sleep(seconds):
                    clock[0] += seconds
                    if clock[0] >= 100.01+delay and not receiver.samples:
                        receiver.samples.append(pose())
                lookup = lambda stamp: gui.RosEgoReceiver.measured_xy_at_stamp(receiver, stamp)
                with patch('perception.road_scan_frame.time.time', side_effect=lambda: clock[0]), \
                        patch('perception.road_scan_frame.time.monotonic', side_effect=lambda: clock[0]), \
                        patch('perception.road_scan_frame.time.sleep', side_effect=sleep):
                    result = align_road_scan_wait(scan, pose(), lookup, captured_at=100.01)
                self.assertTrue(result['proof']['valid'], result)
                self.assertGreaterEqual(result['proof']['age_s'], .01+delay-1e-9)
                self.assertEqual(result['proof']['stamp'], 100.)
                self.assertEqual(result['proof']['sequence'], 71)
                self.assertEqual(scan['age_s'], .01)
                self.assertLessEqual(clock[0], 100.11+1e-9)

    def test_over_budget_bracket_and_expiring_source_stop_without_replacement(self):
        clock = [100.01]
        scan = dict(stamp=100., age_s=.01, sequence=71,
                    obstacle_points=np.array([[8., 2.]]))
        with patch('perception.road_scan_frame.time.time', side_effect=lambda: clock[0]), \
                patch('perception.road_scan_frame.time.monotonic', side_effect=lambda: clock[0]), \
                patch('perception.road_scan_frame.time.sleep', side_effect=lambda value: clock.__setitem__(0, clock[0]+value)):
            result = align_road_scan_wait(scan, pose(), lambda stamp: None,
                                          captured_at=100.01, wait_s=1.)
        self.assertFalse(result['proof']['valid'])
        self.assertTrue(result['proof']['awaiting_source_pose'])
        self.assertAlmostEqual(clock[0], 100.11)
        self.assertEqual(result['proof']['stamp'], 100.)
        self.assertEqual(scan['age_s'], .01)
        # The existing expiry case above stops during the wait, before either
        # this timeout or a later measurement can renew the source's lifetime.

    def test_invalid_raw_or_known_pose_jump_never_waits_for_a_bracket(self):
        for points, source, target in ((np.array([[np.nan, 0.]]), None, pose()),
                                      (np.array([[8., 0.]]), pose(), pose(99.9, 10.))):
            scan = dict(stamp=100., age_s=.01, sequence=1, obstacle_points=points)
            lookup = lambda stamp: source if stamp == 100. else target
            with patch('perception.road_scan_frame.time.time', return_value=100.1), \
                    patch('perception.road_scan_frame.time.monotonic', return_value=1.), \
                    patch('perception.road_scan_frame.time.sleep',
                          side_effect=AssertionError('invalid source must stop immediately')):
                result = align_road_scan_wait(scan, target, lookup, captured_at=1.)
            self.assertFalse(result['proof']['valid'])
            self.assertFalse(result['proof'].get('awaiting_source_pose', False))

    def test_actual_scan_geometry_normalization_is_frame_invariant(self):
        # Captured source stamps/points and GUI geometry are original. GUI did
        # not log target acquisition stamps; declare a synthetic -0.2 s target
        # solely for the geometry replay instead of inventing native timing.
        with np.load(Path(__file__).parent/'test_data/road_scan_frame_20261016.npz') as data:
            for index, uid in ((13, 6), (21, 10)):
                with self.subTest(frame=index):
                    prefix = f'frame{index}_'
                    source = json.loads(str(data[prefix+'ego']))
                    target = json.loads(str(data[prefix+'target_geometry']))
                    stamp = float(data[prefix+'scan_stamp'])
                    source.update(stamp=stamp, measured=True, speed_kmh=target['speed_kmh'])
                    target.update(stamp=stamp-.2, measured=True)
                    points = data[prefix+'points']
                    objects = json.loads(str(data[prefix+'objects']))
                    scan = dict(stamp=stamp, age_s=.01, sequence=index, obstacle_points=points)
                    lookup = lambda value: source if value == stamp else target if value == stamp-.2 else None
                    result = align_road_scan(scan, target, lookup, now_wall=stamp+.05,
                                            now_mono=1., captured_at=1.)
                    self.assertTrue(result['proof']['valid'], result)
                    original = next(o for o in objects if o[3] == 'fusion_obstacle' and o[11] == uid)
                    original = ma.split_static([original])[0][0]
                    calibrated, = ma.road_static_objects([original], source, points)
                    coherent, = ma.road_static_objects([original], target, result['points'])
                    mixed, = ma.road_static_objects([original], target, points)
                    np.testing.assert_allclose(coherent[:2], calibrated[:2], atol=1e-10)
                    np.testing.assert_allclose(coherent[6:8], [.9, .9], atol=1e-6)
                    np.testing.assert_allclose(coherent[6:8], calibrated[6:8], atol=1e-10)
                    self.assertGreater(mixed[6], coherent[6]+.1)
                    self.assertEqual(metadata(coherent)['road_static_geometry']['observed_points'],
                                     metadata(calibrated)['road_static_geometry']['observed_points'])

    def test_planar_lookup_requires_exact_measured_brackets_without_imu_or_extrapolation(self):
        receiver = SimpleNamespace(lock=threading.RLock(), samples=deque([
            pose(100., 0.), pose(100.1, .5), dict(pose(100.2, 5.), src='file')]))
        result = gui.RosEgoReceiver.measured_xy_at_stamp(receiver, 100.05)
        self.assertTrue(result['measured'])
        self.assertAlmostEqual(result['x'], .25)
        self.assertAlmostEqual(result['stamp'], 100.05)
        self.assertIsNone(gui.RosEgoReceiver.measured_xy_at_stamp(receiver, 100.15))
        self.assertIsNone(gui.RosEgoReceiver.measured_xy_at_stamp(receiver, 99.99))

    def test_same_source_does_not_reconfirm_near_field_memory_after_reprojection(self):
        points = np.array([[5., .2], [5., .3], [5.1, .2], [5.1, .3]])
        memory = NearFieldMemory(retain_measured_points=True)
        first, second = pose(), pose(100.1, .5)
        memory.update(points, first, 100.)
        before = copy.deepcopy(memory.cells)
        scan = dict(stamp=100., age_s=.02, sequence=7, obstacle_points=points)
        frame = align_road_scan(scan, second, lambda stamp: first if stamp == 100. else second,
                               now_wall=100.15, now_mono=1., captured_at=1.)
        self.assertTrue(frame['proof']['valid'], frame)
        memory.update(frame['points'], second, frame['proof']['stamp'])
        self.assertEqual(set(memory.cells), set(before))
        for key in before:
            self.assertEqual(memory.cells[key][:2], before[key][:2])
            np.testing.assert_allclose(memory.cells[key][2], before[key][2])


class RoadScanIntegrationTests(unittest.TestCase):
    def runtime(self):
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP')
        runtime.test_status['path']['nearest_idx'] = 500
        return runtime

    def test_gui_uses_atomic_points_and_certifies_the_published_frame(self):
        runtime = self.runtime()
        source = dict(runtime.test_status['ego'])
        target = dict(source, stamp=source['stamp']-.1, x=-.5)
        runtime.test_status['ego'] = target
        runtime.ego_receiver.samples.append(target)
        runtime.lidar.pts = np.array([[20., 4.], [25., -4.]], np.float32)
        runtime.lidar.snapshot = lambda: self.fail('mixed snapshot read')
        captured = []
        runtime.contact_check = lambda points, *args: captured.append(points.copy()) or None
        with patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch('static_lane_pass.normal_static_area', return_value=True):
            self.assertTrue(runtime.infer())
        np.testing.assert_allclose(captured[0], [[20.5, 4.], [25.5, -4.]])
        proof = runtime.last_result['postprocess']['road_scan_frame']
        self.assertTrue(proof['valid'])
        self.assertEqual(runtime.near_field_memory.last_scan, source['stamp'])
        self.assertEqual(runtime._lidar_log['mapped_road_floor']['scan_stamp'], source['stamp'])
        self.assertEqual(runtime._lidar_log['road_scan_frame'], proof)

    def test_gui_missing_measured_source_stops_without_legacy_fallback(self):
        runtime = self.runtime()
        runtime.ego_receiver.samples.clear()
        runtime.lidar.snapshot = lambda: self.fail('missing proof must not use legacy scan')
        with patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch('static_lane_pass.normal_static_area', return_value=True):
            self.assertFalse(runtime.infer())
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertIn('source pose unavailable', runtime.input_error)

    def test_live_revalidation_aligns_once_and_loses_permission_without_source_proof(self):
        runtime = self.runtime()
        source = dict(runtime.test_status['ego'])
        target = dict(source, stamp=source['stamp']-.1, x=-.5)
        runtime.test_status['ego'] = target
        runtime.ego_receiver.samples.append(target)
        runtime.test_scan['obstacle_points'] = np.array([[20., 4.], [25., -4.]])
        runtime.lidar.pts = runtime.test_scan['obstacle_points'].copy()
        live_fixtures.LivePathSafetyTests().attach(runtime)
        with patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch('static_lane_pass.normal_static_area', return_value=True), \
                patch('live_path_safety.points_at_pose', side_effect=AssertionError('double transform')):
            self.assertTrue(runtime.infer())
            result = runtime.path_publisher._active_decision_plan
            self.assertTrue(result.validation['road_scan_frame']['valid'])
            self.assertFalse(result.emergency, result.validation)
            runtime.ego_receiver.samples.clear()
            self.assertTrue(runtime.path_publisher.recheck_active_path())
        result = runtime.path_publisher._active_decision_plan
        self.assertTrue(result.emergency)
        self.assertFalse(result.validation['road_scan_frame']['valid'])
        self.assertEqual(result.allowed_mps, 0.)

    def test_old_road_marker_cannot_replace_portal_or_junction_source_contract(self):
        for following, portal, normal_area in ((True, False, True),
                                               (False, True, True),
                                               (False, False, False)):
            with self.subTest(following=following, portal=portal, normal_area=normal_area):
                runtime, plan = live_fixtures.LivePathSafetyTests().junction([])
                runtime._transition_entry_obstacles = portal
                runtime.gap_planner.junction.update(following_only=following,
                                                     empty_actual_body=following)
                if following:
                    runtime.stop_line_controller = None
                    runtime.gap_planner.junction_committed = False
                plan = plan.decided(road_scan_frame=dict(valid=True, stamp=99.9, sequence=1))
                with patch('static_lane_pass.normal_static_area', return_value=normal_area), \
                        patch('perception.road_scan_frame.align_road_scan_wait',
                              side_effect=AssertionError('old marker crossed frame scope')), \
                        patch('live_path_safety.update_approach_points',
                              return_value=np.empty((0, 2))) as entry:
                    result = runtime.path_publisher._safety_provider(
                        plan.path, plan.control_ego, plan.mode, plan)
                self.assertNotIn('road_scan_frame', result.validation)
                self.assertFalse(result.emergency, result.stop_reason)
                if portal:
                    entry.assert_called_once()
                    np.testing.assert_array_equal(entry.call_args.args[2], np.empty((0, 2)))
                else:
                    entry.assert_not_called()

    def test_gui_and_live_wait_for_real_pose_brackets_of_their_unchanged_scans(self):
        runtime = self.runtime()
        first = pose(100.)
        runtime.test_status['ego'] = dict(first, age_s=.01)
        runtime.ego_receiver.samples = deque([dict(first)])
        clock, delivered = [100.01], set()
        runtime.test_scan = dict(stamp=100.001, sequence=1,
            xyz=np.empty((0, 3)), obstacle_points=np.array([[20., 4.], [25., -4.]]))
        runtime.lidar.pts = runtime.test_scan['obstacle_points'].copy()
        runtime.lidar.received = 100.001
        live_fixtures.LivePathSafetyTests().attach(runtime)
        runtime.lidar.scan_snapshot = lambda: dict(runtime.test_scan,
            age_s=clock[0]-runtime.lidar.received)
        def sleep(seconds):
            clock[0] += seconds
            for arrival, sample in ((100.055, pose(100.054, .1)),
                                    (100.145, pose(100.144, .25))):
                if clock[0] >= arrival and arrival not in delivered:
                    delivered.add(arrival)
                    runtime.ego_receiver.samples.append(sample)
                    runtime.test_status['ego'] = dict(sample, age_s=.01)
        with patch('time.time', side_effect=lambda: clock[0]), \
                patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch('time.sleep', side_effect=sleep), \
                patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch('static_lane_pass.normal_static_area', return_value=True):
            self.assertTrue(runtime.infer())
            planned = runtime.last_result['postprocess']['road_scan_frame']
            first_live = runtime.path_publisher._active_decision_plan.validation['road_scan_frame']
            self.assertEqual(planned['stamp'], 100.001)
            self.assertEqual(first_live['stamp'], 100.001)
            self.assertTrue(first_live['valid'])
            clock[0] = 100.07
            runtime.test_scan = dict(runtime.test_scan, stamp=100.06, sequence=2)
            runtime.lidar.received = 100.061
            runtime.object_receiver.observation_stamp = 100.06
            baseline = runtime.path_publisher._safety_baseline
            result = runtime.path_publisher._safety_provider(baseline['world_path'],
                baseline['ego'], baseline['mode'], baseline['decision'])
        proof = result.validation['road_scan_frame']
        self.assertTrue(proof['valid'], proof)
        self.assertEqual(proof['stamp'], 100.06)
        self.assertEqual(proof['sequence'], 2)
        self.assertGreaterEqual(proof['age_s'], .084-1e-9)
        self.assertFalse(result.emergency, result.stop_reason)


if __name__ == '__main__':
    unittest.main()
