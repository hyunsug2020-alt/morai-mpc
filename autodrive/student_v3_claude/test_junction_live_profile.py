"""Map launch timing needs the live body speed ceiling and rear-clear travel."""
from dataclasses import replace
import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from driving_plan import DrivingPlan
from junction_vehicle_motion import certify_moving_cars
from path_safety import PathSafety
import test_live_path_safety as fixtures
from test_junction_entry import car
from test_ped_guard import person
from traffic_planner import GapPlanner


class JunctionLiveProfileTests(unittest.TestCase):
    def fixture(self, *, crossing=False, following=True):
        helper = fixtures.LivePathSafetyTests()
        runtime = helper.attach(helper.runtime())
        ego = runtime.test_status['ego']
        ego.update(x=0., y=0., heading_deg=0., speed_kmh=0., stamp=100., age_s=.02)
        world = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))
        runtime.test_route = world
        lane = (np.column_stack((np.full(161, 10.), np.arange(40., -40.01, -.5)))
                if crossing else world)
        runtime.gap_planner = GapPlanner([
            dict(points=lane.tolist(), from_node_idx='a', to_node_idx='b')])
        runtime.gap_planner.junction = dict(following_only=following, ready=True,
                                          empty_actual_body=True, held=True)
        runtime.stop_line_controller = None
        runtime.sim_clock = SimpleNamespace(rtf=.8)
        vehicle = car(10., 35., 0., -3., 7) if crossing else car(15., 0., 3., 0., 7)
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
            [vehicle], True, dict(stamp=99.9, age_s=.1)))
        plan = DrivingPlan(1, world, ego, ego, 99.9, .1, 100., 1.28, 60., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False,
                 junction_body_cap_mps=24./3.6), 'LATTICE')
        return runtime, plan, world

    def check(self, runtime, plan, world):
        return runtime.path_publisher._safety_provider(world, plan.control_ego, plan.mode, plan)

    def test_production_yield_controller_exists_with_line_wait_disabled_and_has_no_hold(self):
        import gui
        lines = [dict(id='entry', node_id='A1256W000709', station_m=100., xy=[100., 0.])]
        with patch.dict(os.environ, STUDENT_INTERSECTION_LINE_WAIT='0'), \
                patch.object(gui, 'YIELD_AT_JUNCTION', True):
            controller = gui.junction_controller(lines)
        self.assertIsNotNone(controller)
        self.assertTrue(controller.yield_only)
        self.assertTrue(controller.allow_empty_entry)
        self.assertEqual(controller.hold_s, 0.)
        # A moving, fresh clear approach needs no prior standing sample. The
        # GUI permission remains provisional until the live publisher accepts it.
        answer = controller.update(95.755, 1., 100., True,
            observation_age_s=.02, empty_front_station=99.6)
        self.assertEqual(answer['state'], 'APPROACH_YIELD')
        self.assertTrue(answer['gap_checked'])
        self.assertFalse(answer['stop'])
        self.assertGreater(answer['cap_mps'], 0.)
        self.assertEqual(answer['required_hold_s'], 0.)
        self.assertEqual(answer['held_sim_s'], 0.)
        self.assertFalse(controller.released)

    def test_production_controller_still_obeys_the_junction_yield_switch(self):
        import gui
        with patch.dict(os.environ, STUDENT_INTERSECTION_LINE_WAIT='1'), \
                patch.object(gui, 'YIELD_AT_JUNCTION', False):
            self.assertIsNone(gui.junction_controller([]))

    def committed_fixture(self, following):
        runtime, plan, world = self.fixture(following=following)
        if not following:
            runtime.gap_planner.junction['yield_only'] = True
            runtime.gap_planner.latch_junction_commit()
        return runtime, plan, world

    def test_current_raw_contact_survives_moving_mask_and_legacy_commit(self):
        for following in (True, False):
            with self.subTest(following_only=following):
                runtime, plan, world = self.committed_fixture(following)
                # This return is inside the measured moving-car mask as well
                # as the actual ego front. Its car OBB is still 0.255 m clear.
                vehicle = car(6.4, 0., 0., 3., 7, heading=0.)
                runtime.vehicle_receiver.snapshot = lambda: (
                    [vehicle], True, dict(stamp=99.9, age_s=.1))
                runtime.test_scan['xyz'] = np.array([[3.8, 0., .6]])
                with patch('time.monotonic', return_value=100.):
                    result = self.check(runtime, plan, world)
                self.assertGreater(result.validation['junction_current_semantic_clear_m'], .1)
                if following:
                    self.assertGreater(result.validation['junction_raw_static_clear_m'], 50.)
                self.assertTrue(result.validation['junction_current_contact'])
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)
                self.assertEqual(result.stop_reason, 'live junction current body contact')

    def test_current_known_and_unknown_npc_contact_cannot_be_softened_after_commit(self):
        for following in (True, False):
            for known in (True, False):
                with self.subTest(following_only=following, velocity_known=known):
                    runtime, plan, world = self.committed_fixture(following)
                    vehicle = car(5.5, 0., 3. if known else 0., 0., 7, valid=known)
                    runtime.vehicle_receiver.snapshot = lambda: (
                        [vehicle], True, dict(stamp=99.9, age_s=.1))
                    with patch('time.monotonic', return_value=100.):
                        result = self.check(runtime, plan, world)
                    self.assertLess(result.validation['junction_current_semantic_clear_m'], 0.)
                    self.assertTrue(result.validation['junction_current_contact'])
                    self.assertEqual(result.allowed_mps, 0.)
                    self.assertTrue(result.emergency)
                    self.assertEqual(result.stop_reason, 'live junction current body contact')

    def test_person_contact_remains_an_emergency_in_both_junction_modes(self):
        for following in (True, False):
            with self.subTest(following_only=following):
                runtime, plan, world = self.committed_fixture(following)
                runtime.object_receiver.objects = [person(3.9, 0., 0., 0.)]
                with patch('time.monotonic', return_value=100.):
                    result = self.check(runtime, plan, world)
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)

    def test_clear_legacy_commit_continues_following_without_a_new_entry_wait(self):
        runtime, plan, world = self.committed_fixture(False)
        with patch('time.monotonic', return_value=100.):
            result = self.check(runtime, plan, world)
        self.assertTrue(runtime.gap_planner.junction_committed)
        self.assertFalse(result.validation.get('junction_current_contact', False))
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)

    def test_preentry_frame_identity_does_not_invalidate_a_newer_committed_observation(self):
        from junction_source_guard import frame_identity
        runtime, plan, world = self.fixture(following=False)
        runtime.lanes.route_progress = lambda ego: ego['x']
        runtime.gap_planner.junction.update(yield_only=True, constant_ring_speed=True,
            line_station_m=50., line_id='entry', line_gap_m=50.)
        provider = runtime.path_publisher._safety_provider
        provider.owner = 'ROAD'
        record = dict(stamp=99.9, age_s=.1, ready=True, active=True, vehicles=[])
        runtime.vehicle_receiver.snapshot = lambda: ([], True, record)
        # The committed publication has already processed 99.9; the last
        # preentry identity still belongs to 99.7. It is not a 99.9 revision.
        provider.vehicle_stamp = 99.9
        provider._junction_vehicle_frame = frame_identity(dict(record, stamp=99.7))
        with patch('time.monotonic', return_value=100.):
            checked = self.check(runtime, plan, world)
        self.assertTrue(checked.validation['valid'])
        self.assertFalse(checked.validation.get('junction_source_same_stamp_revision'))
        # A genuinely revised body at 99.9 must still invalidate cached work.
        record['vehicles'] = [dict(uid=7, velocity=[3., 0.])]
        with patch('time.monotonic', return_value=100.):
            revised = self.check(runtime, plan, world)
        self.assertFalse(revised.validation['valid'])
        self.assertTrue(revised.validation['junction_source_same_stamp_revision'])
        self.assertTrue(revised.emergency)

    def test_committed_observations_keep_their_identity_with_the_cached_body(self):
        from junction_source_guard import frame_identity
        runtime, plan, world = self.committed_fixture(False)
        runtime.lanes.route_progress = lambda ego: ego['x']
        runtime.gap_planner.junction.update(constant_ring_speed=True,
            line_station_m=50., line_id='entry', line_gap_m=50.)
        provider = runtime.path_publisher._safety_provider
        record = dict(stamp=99.9, age_s=.1, ready=True, active=True, vehicles=[])
        runtime.vehicle_receiver.snapshot = lambda: ([], True, record)
        with patch('time.monotonic', return_value=100.):
            checked = self.check(runtime, plan, world)
        self.assertFalse(checked.emergency, checked.stop_reason)
        self.assertEqual(provider._junction_vehicle_frame, frame_identity(record))
        record['vehicles'] = [dict(uid=7, velocity=[3., 0.])]
        with patch('time.monotonic', return_value=100.):
            revised = self.check(runtime, plan, world)
        self.assertTrue(revised.emergency)
        self.assertTrue(revised.validation['junction_source_same_stamp_revision'])

    def test_each_snapshot_age_and_compute_elapsed_are_counted_once(self):
        for processing_s in (.04, .24):
            with self.subTest(processing_s=processing_s):
                runtime, plan, world = self.fixture()
                runtime.test_status['ego'].update(stamp=99.98, received=99.98)
                clock = [100.]
                def scan():
                    value = dict(runtime.test_scan)
                    clock[0] += .04
                    return value
                runtime.lidar.scan_snapshot = scan
                runtime.object_receiver.snapshot_with_time = lambda: ([], 99.9, clock[0]-99.9)
                vehicle = car(15., 0., 3., 0., 7)
                runtime.vehicle_receiver.snapshot = lambda: (
                    [vehicle], True, dict(stamp=99.9, age_s=clock[0]-99.9))
                def on_road(cars):
                    clock[0] += processing_s
                    return cars
                runtime.on_road_vehicles = on_road
                with patch('time.monotonic', side_effect=lambda: clock[0]), \
                        patch('junction_vehicle_motion.certify_moving_cars',
                              wraps=certify_moving_cars) as certify:
                    result = self.check(runtime, plan, world)
                if processing_s > .2:
                    # Preparation outlived the retained pose before the new
                    # timing assessment could start. It must stop immediately.
                    self.assertEqual(certify.call_count, 0)
                    self.assertEqual(result.allowed_mps, 0.)
                    self.assertTrue(result.emergency)
                    continue
                call = certify.call_args_list[0]
                expected_age = .14+processing_s
                self.assertAlmostEqual(call.kwargs['vehicle_age_s'], expected_age)
                self.assertAlmostEqual(call.kwargs['lidar_age_s'], expected_age)
                for _, _, _, age in call.args[4]:
                    self.assertAlmostEqual(age, expected_age)
                self.assertTrue(result.validation['junction_motion_check']['valid'])

    def delayed_feedback(self, runtime, *, changes=None, preparation_s=.1):
        old = dict(runtime.test_status['ego'], stamp=99.98, accel=0., brake=1., received=99.98)
        fresh = dict(old, x=.02, heading_deg=2., stamp=100.08,
                     speed_kmh=1.8, accel=.3, brake=0., received=100.08)
        fresh.update(changes or {})
        clock, calls = [100.], []
        def snapshot(count):
            calls.append(clock[0])
            return dict(old if len(calls) == 1 else fresh), [], 20.
        runtime.ego_receiver.snapshot = snapshot
        runtime.ego_receiver.at_stamp = lambda stamp: dict(old, stamp=stamp)
        def prepare(cars):
            clock[0] += preparation_s
            return cars
        runtime.on_road_vehicles = prepare
        return clock, old, fresh, calls

    def test_yield_timing_recaptures_current_pose_after_source_preparation(self):
        runtime, plan, world = self.fixture(following=False)
        runtime.lanes.route_progress = lambda ego: ego['x']
        runtime.gap_planner.junction.update(yield_only=True, constant_ring_speed=True,
            line_id='entry', line_station_m=50., line_gap_m=50.)
        runtime.vehicle_receiver.snapshot = lambda: ([car(15., 0., 3., 0., 7)], True,
            dict(stamp=99.9, age_s=.1, active=True, ready=True, vehicles=[]))
        clock, old, fresh, calls = self.delayed_feedback(runtime)
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch('junction_vehicle_motion.certify_moving_cars',
                      wraps=certify_moving_cars) as certify:
            self.check(runtime, plan, world)
        self.assertGreaterEqual(len(calls), 2)
        self.assertTrue(certify.call_args_list)
        checked_ego = certify.call_args_list[0].args[1]
        self.assertEqual(checked_ego['stamp'], fresh['stamp'])
        self.assertEqual(checked_ego['accel'], fresh['accel'])
        self.assertEqual(checked_ego['brake'], 0.)
        self.assertGreater(certify.call_args_list[0].kwargs['vehicle_age_s'], .1)

    def test_parent_commit_cannot_be_vetoed_by_a_stale_copied_entry_mode(self):
        runtime, plan, world = self.fixture(following=False)
        runtime.gap_planner.junction.update(yield_only=True, constant_ring_speed=True)
        runtime.gap_planner.latch_junction_commit()
        provider = runtime.path_publisher._safety_provider
        provider.owner = 'ROAD'
        provider.junction_snapshot(plan)
        provider.traffic.junction_committed = False
        with patch('time.monotonic', return_value=100.), \
                patch('junction_vehicle_motion.certify_moving_cars') as certify:
            checked = self.check(runtime, plan, world)
        certify.assert_not_called()
        self.assertFalse(checked.emergency, checked.stop_reason)

    def test_preparation_recaptures_native_throttle_ack_before_the_real_launch_assessment(self):
        runtime, plan, world = self.fixture()
        provider = runtime.path_publisher._safety_provider
        runtime.test_status['ego'].update(stamp=99.94, received=99.92, accel=0., brake=1.)
        with patch('time.monotonic', return_value=99.94):
            previous = self.check(runtime, plan, world)
        self.assertIn('junction_motion', previous.validation)
        provider.note_published(previous, world, previous.allowed_mps, 99.94)
        self.assertTrue(provider.junction_launch.active)
        clock, old, fresh, snapshots = self.delayed_feedback(runtime)
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch('junction_vehicle_motion.certify_moving_cars',
                      wraps=certify_moving_cars) as certify:
            result = self.check(runtime, plan, world)
        self.assertEqual(len(snapshots), 2)
        self.assertEqual(old['brake'], 1.)
        self.assertEqual(certify.call_args_list[0].args[1]['stamp'], fresh['stamp'])
        self.assertEqual(certify.call_args_list[0].args[1]['brake'], 0.)
        launch = result.validation['junction_launch']
        self.assertEqual(launch['stamp'], fresh['stamp'])
        self.assertEqual(launch['v_mps'], 0.)
        self.assertAlmostEqual(launch['dead_s'], .5-(100.1-99.94)*.8)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)

    def test_refreshed_pose_rebuilds_raw_coordinates_path_and_actual_body_frame(self):
        from live_path_safety import points_at_pose, rotation
        runtime, plan, world = self.fixture()
        runtime.test_scan['xyz'] = np.array([[8., 2., .6]])
        clock, _, fresh, _ = self.delayed_feedback(runtime)
        transformed, guarded = [], []
        def transform(points, source, destination):
            answer = points_at_pose(points, source, destination)
            transformed.append((dict(destination), answer.copy()))
            return answer
        original_check = PathSafety.check
        def check(guard, path, ego, *args, **kwargs):
            guarded.append((np.asarray(path).copy(), dict(ego)))
            return original_check(guard, path, ego, *args, **kwargs)
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch('live_path_safety.points_at_pose', side_effect=transform), \
                patch.object(PathSafety, 'check', check):
            result = self.check(runtime, plan, world)
        self.assertEqual(len(transformed), 2)
        self.assertNotEqual(transformed[0][0]['stamp'], transformed[1][0]['stamp'])
        np.testing.assert_allclose(transformed[1][1],
            (np.array([[8., 2.]])-[fresh['x'], fresh['y']])@rotation(fresh))
        expected_path = (world-[fresh['x'], fresh['y']])@rotation(fresh)
        np.testing.assert_allclose(result.path, expected_path)
        np.testing.assert_allclose(guarded[0][0], expected_path)
        self.assertEqual(guarded[0][1]['stamp'], fresh['stamp'])
        self.assertEqual(result.control_ego['stamp'], fresh['stamp'])
        self.assertFalse(result.emergency, result.stop_reason)

    def test_pose_refresh_preserves_source_observation_times_and_ingests_pp_only_once(self):
        from junction_vehicle_motion import source_motion_rows
        runtime, plan, world = self.fixture()
        runtime.object_receiver.snapshot_with_time = lambda: ([], 99.9, .1)
        clock, _, fresh, _ = self.delayed_feedback(runtime)
        provider = runtime.path_publisher._safety_provider
        original_tracks_update = provider.junction_tracks.update
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(provider.junction_tracks, 'update',
                             wraps=original_tracks_update) as tracks, \
                patch('junction_vehicle_motion.source_motion_rows',
                      wraps=source_motion_rows) as motion, \
                patch('junction_vehicle_motion.certify_moving_cars',
                      wraps=certify_moving_cars) as certify:
            result = self.check(runtime, plan, world)
        self.assertEqual(tracks.call_count, 1)
        self.assertEqual(motion.call_count, 1)
        self.assertEqual(len(provider.junction_motion.history[7]), 1)
        call = certify.call_args_list[0]
        self.assertAlmostEqual(call.kwargs['now'], 100.1)
        self.assertAlmostEqual(call.kwargs['vehicle_age_s'], .2)
        self.assertAlmostEqual(call.kwargs['lidar_age_s'], .2)
        for _, _, stamp, age in call.args[4]:
            self.assertAlmostEqual(stamp, 99.9)
            self.assertAlmostEqual(call.kwargs['now']-age, 99.9)
        self.assertAlmostEqual(result.validation['junction_motion']['expires_at'], 100.25)
        self.assertAlmostEqual(result.validation['junction_live']['expires_at'], 100.25)
        for track in provider.guard.memory.tracks.values():
            self.assertEqual(len(track['history']), 1)
            self.assertAlmostEqual(track['observed_at'], 99.9)
        self.assertEqual(result.validation['sensor_times']['ego'], fresh['stamp'])

    def test_new_pose_cannot_renew_an_expired_original_vehicle_source(self):
        runtime, plan, world = self.fixture()
        clock, _, _, _ = self.delayed_feedback(runtime)
        runtime.test_scan.update(stamp=99.71, age_s=.29)
        runtime.vehicle_receiver.snapshot = lambda: (
            [car(15., 0., 3., 0., 7)], True, dict(stamp=99.71, age_s=.29))
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            result = self.check(runtime, plan, world)
        self.assertTrue(result.emergency)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertNotIn('junction_motion', result.validation)
        self.assertEqual(result.sensor_stamp, 99.71)

    def test_bad_refreshed_pose_cannot_reuse_the_preparation_pose_permission(self):
        variants = (dict(x=float('nan')), dict(stamp=99.97),
                    dict(stamp=99.98, x=.01), dict(x=3.),
                    dict(heading_deg=90.), dict(speed_kmh=80.),
                    dict(stamp=99.98, x=0., heading_deg=1., speed_kmh=0.,
                         accel=0., brake=1., received=99.98, age_s=.12))
        for changes in variants:
            with self.subTest(changes=changes):
                runtime, plan, world = self.fixture()
                clock, _, _, _ = self.delayed_feedback(runtime, changes=changes)
                with patch('time.monotonic', side_effect=lambda: clock[0]), \
                        patch('junction_vehicle_motion.certify_moving_cars',
                              wraps=certify_moving_cars) as certify:
                    result = self.check(runtime, plan, world)
                self.assertEqual(certify.call_count, 0)
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)
                self.assertFalse(result.validation['pose_valid'])

    def test_same_stamp_eskf_heading_jitter_keeps_the_original_pose_deadline(self):
        runtime, plan, world = self.fixture()
        clock, _, _, _ = self.delayed_feedback(runtime, changes=dict(
            stamp=99.98, x=0., heading_deg=.0000307935, speed_kmh=0.,
            accel=0., brake=1., received=99.98, age_s=.12))
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            result = self.check(runtime, plan, world)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertAlmostEqual(result.validation['junction_motion']['expires_at'], 100.18)
        self.assertAlmostEqual(result.validation['junction_live']['expires_at'], 100.18)
        self.assertEqual(result.control_ego['stamp'], 99.98)

    def late_vehicle_source(self, runtime, *, empty=False, third=False, latest_ready=True,
                            latest_stamp=99.96, preparation_s=.04, old_stationary=False):
        clock, state, reads = [100.], [0], []
        runtime.ego_receiver.snapshot = lambda count: (dict(runtime.test_status['ego'],
            stamp=clock[0]-.02, received=clock[0]-.02, age_s=.02), [], 20.)
        runtime.lidar.scan_snapshot = lambda: dict(runtime.test_scan, age_s=clock[0]-99.9)
        runtime.object_receiver.snapshot_with_time = lambda: ([], 99.9, clock[0]-99.9)
        packets = [([car(5.8, 0., 0., 0., 7)] if old_stationary else
                    [car(15., 0., 3., 0., 7)], True, 99.8),
                   ([] if empty else [car(15.48, 0., 3., 0., 7)], latest_ready, latest_stamp),
                   ([car(15.66, 0., 3., 0., 7)], True, 100.02)]
        def snapshot():
            rows, ready, stamp = packets[state[0]]
            reads.append((clock[0], stamp))
            return list(rows), ready, dict(stamp=stamp, age_s=clock[0]-stamp)
        def prepare(rows):
            clock[0] += preparation_s
            state[0] = min(2 if third else 1, state[0]+1)
            return rows
        runtime.vehicle_receiver.snapshot = snapshot
        runtime.on_road_vehicles = prepare
        return clock, reads

    def test_new_complete_pp_during_preparation_restarts_before_timing_and_body_checks(self):
        from junction_vehicle_motion import source_motion_rows
        runtime, plan, world = self.fixture()
        clock, reads = self.late_vehicle_source(runtime)
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch('junction_vehicle_motion.source_motion_rows', wraps=source_motion_rows) as motion, \
                patch('junction_vehicle_motion.certify_moving_cars', wraps=certify_moving_cars) as certify, \
                patch.object(PathSafety, 'check', autospec=True, side_effect=PathSafety.check) as body:
            result = self.check(runtime, plan, world)
        self.assertEqual([stamp for _, stamp in reads], [99.8, 99.96, 99.96])
        self.assertEqual(certify.call_count, 1)
        self.assertEqual(body.call_count, 1)
        self.assertEqual(motion.call_count, 2)
        provider = runtime.path_publisher._safety_provider
        self.assertEqual([sample[0] for sample in provider.junction_motion.history[7]], [99.8, 99.96])
        self.assertEqual(result.validation['sensor_times']['vehicles'], 99.96)
        retry = result.validation['junction_source_retry']
        self.assertEqual((retry['from_stamp'], retry['to_stamp']), (99.8, 99.96))
        self.assertAlmostEqual(result.validation['junction_motion']['expires_at'], 100.25)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)

    def test_retry_is_bounded_when_a_third_complete_source_arrives_during_second_preparation(self):
        runtime, plan, world = self.fixture()
        clock, reads = self.late_vehicle_source(runtime, third=True)
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch('junction_vehicle_motion.certify_moving_cars', wraps=certify_moving_cars) as certify:
            result = self.check(runtime, plan, world)
        self.assertEqual(len(reads), 3)
        self.assertEqual(certify.call_count, 1)
        self.assertEqual(result.validation['sensor_times']['vehicles'], 99.96)
        self.assertEqual(runtime.path_publisher._safety_provider.vehicle_stamp, 99.96)
        self.assertEqual(runtime.vehicle_receiver.snapshot()[2]['stamp'], 100.02)
        self.assertAlmostEqual(result.validation['junction_motion']['expires_at'], 100.25)

    def test_retry_empty_packet_retains_the_just_observed_close_stationary_car_as_a_ghost(self):
        runtime, plan, world = self.fixture()
        clock, _ = self.late_vehicle_source(runtime, empty=True, old_stationary=True)
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            result = self.check(runtime, plan, world)
        provider = runtime.path_publisher._safety_provider
        self.assertEqual(provider.junction_tracks.ghost_ids, {7})
        self.assertEqual(len(provider.junction_motion.history[7]), 1)
        self.assertNotIn('junction_motion', result.validation)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)

    def test_changed_unready_backward_or_nonfinite_peek_cannot_reuse_the_first_source_go(self):
        for ready, stamp in ((False, 99.96), (True, 99.7), (True, float('nan'))):
            with self.subTest(ready=ready, stamp=stamp):
                runtime, plan, world = self.fixture()
                clock, reads = self.late_vehicle_source(runtime, latest_ready=ready, latest_stamp=stamp)
                with patch('time.monotonic', side_effect=lambda: clock[0]), \
                        patch('junction_vehicle_motion.certify_moving_cars', wraps=certify_moving_cars) as certify:
                    result = self.check(runtime, plan, world)
                self.assertEqual(len(reads), 2)
                self.assertEqual(certify.call_count, 0)
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)

    def test_retry_preserves_an_unchanged_camera_deadline_through_later_body_work(self):
        runtime, plan, world = self.fixture()
        clock, _ = self.late_vehicle_source(runtime, preparation_s=.1)
        original_check = PathSafety.check
        def delayed(guard, *args, **kwargs):
            result = original_check(guard, *args, **kwargs)
            clock[0] += .051
            return result
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(PathSafety, 'check', delayed):
            result = self.check(runtime, plan, world)
        self.assertAlmostEqual(result.validation['junction_motion']['expires_at'], 100.25)
        self.assertNotIn('junction_live', result.validation)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)
        self.assertIn('expired', result.stop_reason)

    def test_clock_rewind_during_preparation_cannot_be_hidden_by_source_retry(self):
        runtime, plan, world = self.fixture()
        clock, _ = self.late_vehicle_source(runtime)
        prepare = runtime.on_road_vehicles
        calls = []
        def rewind(rows):
            result = prepare(rows)
            calls.append(clock[0])
            if len(calls) == 1:
                clock[0] -= .08
            return result
        runtime.on_road_vehicles = rewind
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            result = self.check(runtime, plan, world)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)

    def camera_prefix(self, *, following=True):
        runtime, plan, world = self.fixture(following=following)
        crossing = np.column_stack((np.full(161, 10.), np.arange(40., -40.01, -.5)))
        runtime.gap_planner = GapPlanner([
            dict(points=world.tolist(), from_node_idx='a', to_node_idx='b'),
            dict(points=crossing.tolist(), from_node_idx='c', to_node_idx='d')])
        runtime.gap_planner.junction = dict(following_only=following, ready=True,
                                          empty_actual_body=True, held=True)
        vehicles = [car(15., 0., 3., 0., 7), car(10., 35., 0., -3., 8)]
        runtime.vehicle_receiver.snapshot = lambda: (list(vehicles), True,
                                                     dict(stamp=99.9, age_s=.1))
        alias = list(car(15., 0., 3., 0., 24))
        alias[2], alias[3], alias[6], alias[14] = 5.8, 'fusion_npc', 5.8, None
        runtime.object_receiver.objects = [tuple(alias)]
        runtime.object_receiver.snapshot_with_time = lambda: (
            list(runtime.object_receiver.objects), 99.9, .1)
        # The return lies inside the original camera body, outside the PP
        # mask, and before the incoming-car rear-clearance travel requirement.
        runtime.test_scan['xyz'] = np.array([[12.45, 0., .6]])
        return runtime, plan, world, tuple(alias), vehicles

    def test_exact_certified_camera_body_can_own_its_raw_prefix_without_changing_its_shape(self):
        from lidar_corridor import without_moving_objects
        runtime, plan, world, alias, _ = self.camera_prefix()
        masked_rows = []
        def masked(points, objects, ego):
            masked_rows.extend(row for row in objects if row[3] == 'fusion_npc')
            return without_moving_objects(points, objects, ego)
        with patch('time.monotonic', return_value=100.), \
                patch('lidar_corridor.without_moving_objects', side_effect=masked):
            result = self.check(runtime, plan, world)
        self.assertEqual(result.validation['junction_raw_alias_keys'], [('fusion_npc', 24)])
        self.assertGreater(result.validation['junction_raw_static_clear_m'], 50.)
        self.assertTrue(result.validation['live_safety']['junction_rear_travel']['accepted'])
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertEqual(runtime.object_receiver.objects, [alias])
        self.assertEqual(len(masked_rows), 1)
        self.assertEqual(masked_rows[0][2:], alias[2:])
        np.testing.assert_allclose(masked_rows[0][:2], np.asarray(alias[:2])+np.asarray(alias[9:11])*.1)

    def test_camera_without_current_coverage_keeps_the_raw_prefix(self):
        for treatment in ('no_proof', 'unmatched_motion', 'legacy'):
            with self.subTest(treatment=treatment):
                runtime, plan, world, alias, _ = self.camera_prefix(following=treatment != 'legacy')
                if treatment == 'unmatched_motion':
                    opposite = list(alias)
                    opposite[8], opposite[9] = 180., -3.
                    runtime.object_receiver.objects = [tuple(opposite)]
                with patch('time.monotonic', return_value=100.):
                    if treatment == 'no_proof':
                        with patch('junction_vehicle_motion.certify_moving_cars', return_value=None):
                            result = self.check(runtime, plan, world)
                    else:
                        result = self.check(runtime, plan, world)
                self.assertEqual(result.validation.get('junction_raw_alias_keys', []), [])
                self.assertLess(result.validation['live_safety']['raw_clear_m'], 20.)

    def test_certificate_expired_before_masking_cannot_remove_the_camera_prefix(self):
        runtime, plan, world, _, _ = self.camera_prefix()
        clock = [100.]
        def expires(*args, **kwargs):
            answer = certify_moving_cars(*args, **kwargs)
            self.assertIsNotNone(answer)
            clock[0] = answer.expires_at+.001
            return answer
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch('junction_vehicle_motion.certify_moving_cars', side_effect=expires):
            result = self.check(runtime, plan, world)
        self.assertTrue(result.validation['junction_motion_check']['returned'])
        self.assertFalse(result.validation['junction_motion_check']['valid'])
        self.assertEqual(result.validation['junction_raw_alias_keys'], [])
        self.assertLess(result.validation['live_safety']['raw_clear_m'], 20.)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)

    def test_certified_camera_mask_keeps_current_raw_person_and_unknown_car_stops(self):
        for hazard in ('raw', 'person', 'unknown'):
            with self.subTest(hazard=hazard):
                runtime, plan, world, alias, vehicles = self.camera_prefix()
                if hazard == 'raw':
                    runtime.test_scan['xyz'] = np.array([[12.45, 0., .6], [2.5, 0., .6]])
                elif hazard == 'person':
                    runtime.object_receiver.objects = [alias, person(3.9, 0., 0., 0.)]
                else:
                    vehicles.append(car(5.8, 0., 0., 0., 9, valid=False))
                with patch('time.monotonic', return_value=100.):
                    result = self.check(runtime, plan, world)
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)
                if hazard == 'raw':
                    self.assertEqual(result.validation['live_safety']['raw_clear_m'], 0.)

    def test_snapshot_age_normalization_cannot_make_invalid_ages_fresh(self):
        for source in ('objects', 'lidar', 'vehicles'):
            for invalid_age in (float('nan'), float('inf'), -.01):
                with self.subTest(source=source, age=invalid_age):
                    runtime, plan, world = self.fixture()
                    if source == 'objects':
                        runtime.object_receiver.snapshot_with_time = lambda: ([], 99.9, invalid_age)
                    elif source == 'lidar':
                        runtime.test_scan['age_s'] = invalid_age
                    else:
                        runtime.vehicle_receiver.snapshot = lambda: (
                            [car(15., 0., 3., 0., 7)], True, dict(stamp=99.9, age_s=invalid_age))
                    with patch('time.monotonic', return_value=100.):
                        result = self.check(runtime, plan, world)
                    self.assertTrue(result.emergency)
                    self.assertEqual(result.allowed_mps, 0.)

    def test_positive_static_restriction_is_reapproved_and_keeps_current_headway_cap(self):
        runtime, plan, world = self.fixture()
        runtime.test_scan['xyz'] = np.array([[5.7, 0., .6]])
        checks = []
        original = PathSafety.check
        def checked(guard, *args, **kwargs):
            checks.append(kwargs['requested_mps'])
            return original(guard, *args, **kwargs)
        with patch('time.monotonic', return_value=100.), \
                patch.object(PathSafety, 'check', checked), \
                patch('junction_vehicle_motion.certify_moving_cars', wraps=certify_moving_cars) as certify:
            result = self.check(runtime, plan, world)
        self.assertEqual(checks, [24./3.6])
        self.assertEqual(certify.call_count, 2)
        recheck = result.validation['live_safety']['junction_profile_recheck']
        self.assertGreater(recheck['profile_mps'], 1.2)
        self.assertLess(recheck['profile_mps'], 1.5)
        self.assertTrue(recheck['accepted'])
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertLessEqual(result.allowed_mps, 1.28)
        self.assertEqual(result.validation['junction_motion']['covered_keys'], [('lidar_car', 7)])
        tracks = runtime.path_publisher._safety_provider.guard.memory.tracks.values()
        self.assertTrue(all(len(track['history']) == 1 for track in tracks))

    def test_positive_person_restriction_is_reapproved_without_ignoring_the_person(self):
        runtime, plan, world = self.fixture()
        runtime.object_receiver.objects = [person(7., 0., 0., 0.)]
        with patch('time.monotonic', return_value=100.), \
                patch('junction_vehicle_motion.certify_moving_cars', wraps=certify_moving_cars) as certify:
            result = self.check(runtime, plan, world)
        self.assertEqual(certify.call_count, 2)
        recheck = result.validation['live_safety']['junction_profile_recheck']
        self.assertGreater(recheck['profile_mps'], 0.)
        self.assertLess(recheck['profile_mps'], 24./3.6)
        self.assertTrue(recheck['accepted'])
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertLessEqual(result.allowed_mps, min(1.28, recheck['profile_mps']))
        self.assertTrue(any(row['name'] == 'p'
            for row in result.validation['live_safety']['objects']))

    def test_body_slowdown_rejects_a_gap_that_only_the_faster_profile_could_clear(self):
        runtime, plan, world = self.fixture(crossing=True)
        runtime.test_scan['xyz'] = np.array([[5.85, 0., .6]])
        with patch('time.monotonic', return_value=100.), \
                patch('junction_vehicle_motion.certify_moving_cars', wraps=certify_moving_cars) as certify:
            result = self.check(runtime, plan, world)
        self.assertEqual(certify.call_count, 2)
        self.assertFalse(result.validation['live_safety']['junction_profile_recheck']['accepted'])
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)
        self.assertIn('launch profile', result.stop_reason)

    def test_changed_coverage_cannot_reuse_the_independent_guard_skip_set(self):
        runtime, plan, world = self.fixture()
        runtime.test_scan['xyz'] = np.array([[5.85, 0., .6]])
        calls = []
        def changed(*args, **kwargs):
            proof = certify_moving_cars(*args, **kwargs)
            calls.append(proof)
            return replace(proof, covered_keys=frozenset()) if len(calls) == 2 else proof
        with patch('time.monotonic', return_value=100.), \
                patch('junction_vehicle_motion.certify_moving_cars', side_effect=changed):
            result = self.check(runtime, plan, world)
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(proof is not None for proof in calls))
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)

    def test_positive_stop_before_rear_clearance_cannot_authorize_incoming_traffic(self):
        for obstacle_x in (10., 12.):
            with self.subTest(obstacle_x=obstacle_x):
                runtime, plan, world = self.fixture(crossing=True)
                runtime.test_scan['xyz'] = np.array([[obstacle_x, 0., .6]])
                proofs = []
                def certify(*args, **kwargs):
                    proof = certify_moving_cars(*args, **kwargs)
                    proofs.append(proof)
                    return proof
                with patch('time.monotonic', return_value=100.), \
                        patch('junction_vehicle_motion.certify_moving_cars', side_effect=certify):
                    result = self.check(runtime, plan, world)
                self.assertEqual(len(proofs), 2)
                self.assertTrue(all(proof is not None and proof.cap_mps > 0. for proof in proofs))
                self.assertTrue(result.validation['live_safety']['junction_profile_recheck']['accepted'])
                travel = result.validation['live_safety']['junction_rear_travel']
                self.assertGreater(travel['required_m'], travel['clear_m'])
                self.assertFalse(travel['accepted'])
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)
                self.assertIn('rear clearance blocked', result.stop_reason)

    def test_stop_after_rear_clearance_keeps_a_safe_incoming_gap(self):
        runtime, plan, world = self.fixture(crossing=True)
        runtime.test_scan['xyz'] = np.array([[20., 0., .6]])
        with patch('time.monotonic', return_value=100.):
            result = self.check(runtime, plan, world)
        travel = result.validation['live_safety']['junction_rear_travel']
        self.assertGreater(travel['clear_m'], travel['required_m'])
        self.assertTrue(travel['accepted'])
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertLessEqual(result.allowed_mps, 1.28)

    def test_old_gui_wait_cannot_veto_a_fresh_live_go(self):
        runtime, plan, world = self.fixture(crossing=True)
        runtime.gap_planner.status = dict(state='CROSSING_CAUTION', target_mps=0.,
            ring=dict(action='WAIT'), reason='previous camera-frame decision')
        with patch('time.monotonic', return_value=100.):
            result = self.check(runtime, plan, world)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertIn('junction_motion', result.validation)
        self.assertIn('junction_live', result.validation)
        self.assertEqual(result.validation['live_safety']['junction_traffic']['state'], 'MAP_TIMED_ENTRY')
        self.assertEqual(runtime.gap_planner.status['ring']['action'], 'WAIT')

    def approach_publication(self, runtime, plan, world, now, x=0., speed=0., accel=0., brake=1.):
        runtime.test_status['ego'].update(x=x, speed_kmh=speed*3.6, stamp=now,
            age_s=.02, received=now-.02, accel=accel, brake=brake)
        source = now-.1
        runtime.test_scan.update(stamp=source, age_s=.1)
        runtime.object_receiver.observation_stamp = source
        runtime.vehicle_receiver.snapshot = lambda: (
            [car(20., 10.-3.*(now-100.), 0., -3., 7)], True,
            dict(stamp=source, age_s=.1))
        with patch('time.monotonic', return_value=now):
            result = self.check(runtime, replace(plan, plan_id=int((now-99.)*1000)), world)
            runtime.path_publisher._safety_provider.note_published(
                result, world, result.allowed_mps, now)
        return result

    def approach_fixture(self):
        runtime, plan, world = self.fixture(crossing=True)
        crossing = np.column_stack((np.full(161, 20.), np.arange(40., -40.01, -.5)))
        runtime.gap_planner = GapPlanner([
            dict(points=crossing.tolist(), from_node_idx='a', to_node_idx='b')])
        runtime.gap_planner.junction = dict(following_only=True, ready=True,
                                          empty_actual_body=True, held=True)
        return runtime, plan, world

    def test_delivered_approach_without_go_uses_only_acknowledged_progress_as_the_next_initial_speed(self):
        runtime, plan, world = self.approach_fixture()
        first = self.approach_publication(runtime, plan, world, 100.)
        provider = runtime.path_publisher._safety_provider
        self.assertGreater(first.allowed_mps, 0.)
        self.assertNotIn('junction_motion', first.validation)
        self.assertIn('junction_live', first.validation)
        self.assertTrue(provider.junction_launch.active)
        second = self.approach_publication(runtime, plan, world, 100.15,
                                           x=.03, speed=.4, accel=.3, brake=0.)
        third = self.approach_publication(runtime, plan, world, 100.3,
                                          x=.10, speed=.7, accel=.3, brake=0.)
        for result in (second, third):
            self.assertNotIn('junction_motion', result.validation)
            self.assertEqual(result.validation['live_safety']['junction_traffic']['ring']['action'], 'WAIT')
            self.assertGreater(result.allowed_mps, 0.)
        self.assertEqual(second.validation['junction_launch']['v_mps'], 0.)
        initial = third.validation['junction_launch']
        self.assertGreater(initial['v_mps'], 0.)
        self.assertLessEqual(initial['v_mps'], .4-.15+1e-9)
        self.assertEqual(initial['stamp'], 100.3)
        self.assertFalse(runtime.gap_planner.junction_committed)
        self.assertFalse(provider.traffic.junction_committed)
        stopped = third.decided(allowed_mps=0., emergency=True, stop_reason='new physical hazard')
        provider.note_published(stopped, world, 0., 100.3)
        self.assertFalse(provider.junction_launch.active)
        self.assertIsNone(provider.junction_launch.prediction(world, third.control_ego, 100.3, .8))

    def test_live_only_approach_expiry_and_positive_deceleration_cannot_seed_the_observation(self):
        for treatment in ('expired', 'decelerating'):
            with self.subTest(treatment=treatment):
                runtime, plan, world = self.approach_fixture()
                result = self.approach_publication(runtime, plan, world, 100.,
                    speed=1.2 if treatment == 'decelerating' else 0., accel=.3, brake=0.)
                provider = runtime.path_publisher._safety_provider
                self.assertGreater(result.allowed_mps, 0.)
                self.assertNotIn('junction_motion', result.validation)
                if treatment == 'expired':
                    provider.note_published(result, world, result.allowed_mps,
                        result.validation['junction_live']['expires_at']+.001)
                self.assertFalse(provider.junction_launch.active)

    def test_whole_go_proof_expiry_still_limits_an_otherwise_fresh_live_publication(self):
        runtime, plan, world = self.fixture()
        runtime.test_status['ego'].update(accel=0., brake=1.)
        with patch('time.monotonic', return_value=100.):
            result = self.check(runtime, plan, world)
        provider = runtime.path_publisher._safety_provider
        result = result.decided(junction_live=dict(expires_at=100.12),
                                junction_motion=dict(result.validation['junction_motion'], expires_at=100.08))
        with patch.object(provider.junction_launch, 'published',
                          wraps=provider.junction_launch.published) as published:
            provider.note_published(result, world, result.allowed_mps, 100.)
        self.assertTrue(provider.junction_launch.active)
        self.assertEqual(published.call_args.kwargs['proof_expires_at'], 100.08)
        provider.note_published(result, world, result.allowed_mps, 100.09)
        self.assertFalse(provider.junction_launch.active)

    def test_old_gui_motion_flags_cannot_mark_the_current_measured_source_uncertain(self):
        runtime, plan, world = self.fixture(crossing=True)
        for field in ('unknown_ids', 'inherited_ids', 'ghost_ids'):
            runtime.gap_planner.junction[field] = {7}
        with patch('time.monotonic', return_value=100.):
            result = self.check(runtime, plan, world)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertIn('junction_motion', result.validation)
        provider = runtime.path_publisher._safety_provider
        for field in ('unknown_ids', 'inherited_ids', 'ghost_ids'):
            self.assertEqual(provider.traffic.junction[field], set())
            self.assertEqual(runtime.gap_planner.junction[field], {7})

    def test_current_inherited_velocity_still_cannot_acquire_motion_ownership(self):
        runtime, plan, world = self.fixture()
        with patch('time.monotonic', return_value=100.):
            first = self.check(runtime, plan, world)
        self.assertIn('junction_motion', first.validation)
        row = list(car(15.3, 0., 0., 0., 7))
        row[14] = False
        runtime.test_status['ego']['stamp'] = 100.1
        runtime.test_scan.update(stamp=100.05, age_s=.05)
        runtime.vehicle_receiver.snapshot = lambda: ([tuple(row)], True, dict(stamp=100.05, age_s=.05))
        with patch('time.monotonic', return_value=100.1):
            result = self.check(runtime, plan, world)
        provider = runtime.path_publisher._safety_provider
        self.assertEqual(provider.traffic.junction['inherited_ids'], {7})
        self.assertNotIn('junction_motion', result.validation)

    def test_bootstrap_unseen_gui_ghost_cannot_be_cleared_by_an_empty_packet(self):
        runtime, plan, world = self.fixture()
        runtime.gap_planner.junction['ghost_ids'] = {999}
        runtime.vehicle_receiver.snapshot = lambda: ([], True, dict(stamp=99.9, age_s=.1))
        with patch('time.monotonic', return_value=100.):
            first = self.check(runtime, plan, world)
            still_missing = self.check(runtime, replace(plan, plan_id=2), world)
            runtime.gap_planner.junction['ghost_ids'].clear()
            cleared = self.check(runtime, replace(plan, plan_id=3), world)
        self.assertEqual(first.allowed_mps, 0.)
        self.assertEqual(still_missing.allowed_mps, 0.)
        self.assertNotIn('junction_live', first.validation)
        self.assertGreater(cleared.allowed_mps, 0.)

    def test_no_proof_measured_npc_and_its_alias_keep_full_guard_without_frozen_future(self):
        runtime, plan, world = self.fixture()
        alias = list(car(15., 0., 3., 0., 24))
        alias[3], alias[14] = 'fusion_npc', None
        runtime.object_receiver.objects = [tuple(alias)]
        calls = []
        original = PathSafety.check
        def checked(guard, *args, **kwargs):
            calls.append((kwargs['timed_vehicle_keys'],
                [(row[3], row[11]) for _, objects, _, _ in args[2] for row in objects]))
            return original(guard, *args, **kwargs)
        with patch('time.monotonic', return_value=100.), \
                patch('junction_vehicle_motion.certify_moving_cars', return_value=None), \
                patch.object(PathSafety, 'check', checked):
            result = self.check(runtime, plan, world)
        self.assertEqual(calls[0][0], frozenset())
        self.assertIn(('lidar_car', 7), calls[0][1])
        self.assertIn(('fusion_npc', 24), calls[0][1])
        self.assertEqual(set(map(tuple, result.validation['junction_moving_body_keys'])),
                         {('lidar_car', 7), ('fusion_npc', 24)})
        self.assertGreater(result.validation['empty_entry_semantic_clear_m'], 50.)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertNotIn('junction_motion', result.validation)

    def test_no_proof_full_guard_still_stops_for_a_close_measured_moving_npc(self):
        runtime, plan, world = self.fixture()
        runtime.vehicle_receiver.snapshot = lambda: ([car(6.8, 0., 3., 0., 7)], True,
                                                     dict(stamp=99.9, age_s=.1))
        with patch('time.monotonic', return_value=100.), \
                patch('junction_vehicle_motion.certify_moving_cars', return_value=None):
            result = self.check(runtime, plan, world)
        self.assertGreater(result.validation['junction_current_semantic_clear_m'], .1)
        self.assertGreater(result.validation['empty_entry_semantic_clear_m'], 50.)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)
        self.assertTrue(any(row['uid'] == 7 and row['gap_m'] <= .1
                            for row in result.validation['live_safety']['objects']))

    def test_no_proof_current_semantic_contact_is_not_lost_with_the_frozen_sweep(self):
        runtime, plan, world = self.fixture()
        runtime.vehicle_receiver.snapshot = lambda: ([car(6., 0., 3., 0., 7)], True,
                                                     dict(stamp=99.9, age_s=.1))
        with patch('time.monotonic', return_value=100.), \
                patch('junction_vehicle_motion.certify_moving_cars', return_value=None):
            result = self.check(runtime, plan, world)
        self.assertLessEqual(result.validation['junction_current_semantic_clear_m'], .1)
        self.assertGreater(result.validation['empty_entry_semantic_clear_m'], 50.)
        self.assertEqual(result.validation['live_safety']['raw_clear_m'], 0.)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)

    def test_failed_certificate_map_evaluation_is_reused_only_inside_the_same_call(self):
        runtime, plan, world = self.fixture(crossing=True)
        runtime.test_status['ego']['x'] = 4.
        runtime.vehicle_receiver.snapshot = lambda: ([car(10., 15., 0., -3., 7)], True,
                                                     dict(stamp=99.9, age_s=.1))
        calls = []
        original = GapPlanner.evaluate
        def evaluated(planner, *args, **kwargs):
            calls.append(1)
            return original(planner, *args, **kwargs)
        with patch('time.monotonic', return_value=100.), \
                patch.object(GapPlanner, 'evaluate', evaluated):
            first = self.check(runtime, plan, world)
            second = self.check(runtime, plan, world)
        self.assertEqual(len(calls), 2)
        for result in (first, second):
            self.assertEqual(result.allowed_mps, 0.)
            self.assertEqual(result.validation['live_safety']['junction_traffic']['ring']['action'], 'WAIT')

    def test_no_proof_unmasked_raw_contact_remains_after_moving_return_mask(self):
        runtime, plan, world = self.fixture()
        runtime.vehicle_receiver.snapshot = lambda: ([car(6.4, 0., 0., 3., 7, heading=0.)], True,
                                                     dict(stamp=99.9, age_s=.1))
        runtime.test_scan['xyz'] = np.array([[3.8, 0., .6]])
        with patch('time.monotonic', return_value=100.), \
                patch('junction_vehicle_motion.certify_moving_cars', return_value=None):
            result = self.check(runtime, plan, world)
        self.assertGreater(result.validation['junction_current_semantic_clear_m'], .1)
        self.assertGreater(result.validation['junction_raw_static_clear_m'], 50.)
        self.assertEqual(result.validation['live_safety']['raw_clear_m'], 0.)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)

    def test_parked_or_unknown_semantics_cannot_transfer_their_frozen_future(self):
        runtime, plan, world = self.fixture()
        parked = list(car(8., 0., 0., 0., 24))
        parked[3:5], parked[14] = ['fusion_parked_vehicle', 'obstacle'], None
        runtime.object_receiver.objects = [tuple(parked)]
        with patch('time.monotonic', return_value=100.), \
                patch('junction_vehicle_motion.certify_moving_cars', return_value=None):
            result = self.check(runtime, plan, world)
        self.assertNotIn(('fusion_parked_vehicle', 24),
                         set(map(tuple, result.validation['junction_moving_body_keys'])))
        self.assertLess(result.validation['empty_entry_semantic_clear_m'], 10.)

    def test_exact_parked_alias_needs_a_fresh_body_checked_motion_certificate(self):
        for certified in (True, False):
            with self.subTest(certified=certified):
                runtime, plan, world = self.fixture()
                parked = list(car(15.25, 0., 0., 0., 24, heading=30.))
                parked[3:5], parked[7], parked[14] = ['fusion_parked_vehicle', 'obstacle'], 3.2, None
                parked.append(0.)
                runtime.object_receiver.objects = [tuple(parked)]
                runtime.vehicle_receiver.snapshot = lambda: (
                    [(*car(15., 0., 3., 0., 7), 0.)], True, dict(stamp=99.9, age_s=.1))
                with patch('time.monotonic', return_value=100.), \
                        patch('junction_vehicle_motion.certify_moving_cars',
                              wraps=certify_moving_cars if certified else None,
                              **({} if certified else dict(return_value=None))):
                    result = self.check(runtime, plan, world)
                self.assertNotIn(('fusion_parked_vehicle', 24),
                                 set(map(tuple, result.validation['junction_moving_body_keys'])))
                if certified:
                    self.assertIn(('fusion_parked_vehicle', 24),
                                  set(map(tuple, result.validation['junction_motion']['covered_keys'])))
                    self.assertGreater(result.validation['empty_entry_semantic_clear_m'], 50.)
                    self.assertGreater(result.validation['junction_current_semantic_clear_m'], .1)
                    self.assertGreater(result.allowed_mps, 0.)
                else:
                    self.assertNotIn('junction_motion', result.validation)
                    self.assertLess(result.validation['empty_entry_semantic_clear_m'], 20.)

    def test_old_gui_go_cannot_override_a_new_live_wait(self):
        runtime, plan, world = self.fixture(crossing=True)
        runtime.test_status['ego']['x'] = 4.
        runtime.gap_planner.status = dict(state='CRUISE', target_mps=24./3.6,
                                        ring=dict(action='GO'))
        runtime.vehicle_receiver.snapshot = lambda: (
            [car(10., 15., 0., -3., 7)], True, dict(stamp=99.9, age_s=.1))
        with patch('time.monotonic', return_value=100.):
            result = self.check(runtime, replace(plan, allowed_mps=24./3.6), world)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)
        self.assertNotIn('junction_motion', result.validation)
        live = result.validation['live_safety']['junction_traffic']
        self.assertEqual(live['state'], 'CROSSING_CAUTION')
        self.assertEqual(live['ring']['action'], 'WAIT')
        self.assertEqual(live['target_mps'], 0.)

    def test_empty_road_needs_fresh_live_timing_but_no_motion_skip_certificate(self):
        runtime, plan, world = self.fixture()
        runtime.vehicle_receiver.snapshot = lambda: ([], True, dict(stamp=99.9, age_s=.1))
        with patch('time.monotonic', return_value=100.):
            result = self.check(runtime, plan, world)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertNotIn('junction_motion', result.validation)
        self.assertEqual(result.validation['live_safety']['junction_traffic']['state'], 'CRUISE')
        self.assertTrue(result.junction_motion_fresh(100.17))
        self.assertFalse(result.junction_motion_fresh(100.19))

    def test_stale_live_sources_or_pose_cannot_use_an_old_gui_go(self):
        for stale in ('vehicle', 'lidar', 'pose', 'not_ready'):
            with self.subTest(stale=stale):
                runtime, plan, world = self.fixture()
                runtime.gap_planner.status = dict(state='CRUISE', ring=dict(action='GO'))
                runtime.vehicle_receiver.snapshot = lambda: (
                    [], stale != 'not_ready', dict(stamp=99.9, age_s=.36 if stale == 'vehicle' else .1))
                if stale == 'lidar':
                    runtime.test_scan['age_s'] = .36
                elif stale == 'pose':
                    runtime.test_status['ego']['age_s'] = .201
                with patch('time.monotonic', return_value=100.):
                    result = self.check(runtime, plan, world)
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)
                self.assertNotIn('junction_live', result.validation)

    def test_unknown_unmapped_nearby_car_keeps_the_live_stop(self):
        runtime, plan, world = self.fixture(crossing=True)
        row = list(car(7., 3., 0., 0., 7))
        row[14] = False
        runtime.vehicle_receiver.snapshot = lambda: ([tuple(row)], True, dict(stamp=99.9, age_s=.1))
        with patch('time.monotonic', return_value=100.):
            result = self.check(runtime, plan, world)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)
        self.assertNotIn('junction_motion', result.validation)
        self.assertEqual(result.validation['live_safety']['junction_traffic']['state'], 'WAIT')

    def test_time_spent_in_fallback_map_evaluation_cannot_renew_the_deadline(self):
        runtime, plan, world = self.fixture()
        runtime.vehicle_receiver.snapshot = lambda: ([], True, dict(stamp=99.9, age_s=.1))
        clock = [100.]
        original = GapPlanner.evaluate
        def delayed(planner, *args, **kwargs):
            decision = original(planner, *args, **kwargs)
            clock[0] += .21
            return decision
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(GapPlanner, 'evaluate', delayed):
            result = self.check(runtime, plan, world)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)
        self.assertIn('live check expired', result.stop_reason)

    def test_fallback_without_pose_receipt_still_keeps_the_capture_deadline(self):
        runtime, plan, world = self.fixture()
        runtime.vehicle_receiver.snapshot = lambda: ([], True, dict(stamp=99.9, age_s=.1))
        clock = [100.]
        original = PathSafety.check
        def delayed(guard, *args, **kwargs):
            result = original(guard, *args, **kwargs)
            clock[0] += .1
            return result
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(PathSafety, 'check', delayed):
            result = self.check(runtime, plan, world)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertLessEqual(result.validation['junction_live']['expires_at'], 100.18)

    def test_profile_recheck_keeps_the_original_expiry_and_does_not_reingest_memory(self):
        runtime, plan, world = self.fixture()
        runtime.test_scan['xyz'] = np.array([[5.85, 0., .6]])
        clock, proofs, checks = [100.], [], []
        original = PathSafety.check
        def delayed(guard, *args, **kwargs):
            checks.append(1)
            result = original(guard, *args, **kwargs)
            clock[0] += .1
            return result
        def certify(*args, **kwargs):
            proof = certify_moving_cars(*args, **kwargs)
            proofs.append((proof, kwargs))
            return proof
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(PathSafety, 'check', delayed), \
                patch('junction_vehicle_motion.certify_moving_cars', side_effect=certify):
            result = self.check(runtime, plan, world)
        self.assertEqual(len(checks), 1)
        self.assertEqual(len(proofs), 2)
        self.assertAlmostEqual(proofs[1][1]['vehicle_age_s']-proofs[0][1]['vehicle_age_s'], .1)
        self.assertLessEqual(result.validation['junction_motion']['expires_at'], proofs[0][0].expires_at)
        self.assertFalse(result.emergency, result.stop_reason)

    def test_time_spent_in_body_check_cannot_renew_a_pose_deadline(self):
        runtime, plan, world = self.fixture()
        runtime.test_scan['xyz'] = np.array([[5.85, 0., .6]])
        clock = [100.]
        original = PathSafety.check
        def delayed(guard, *args, **kwargs):
            result = original(guard, *args, **kwargs)
            clock[0] += .21
            return result
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(PathSafety, 'check', delayed):
            result = self.check(runtime, plan, world)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)

    def test_clock_rewind_during_body_check_cannot_rebind_the_old_sources(self):
        runtime, plan, world = self.fixture()
        runtime.test_scan['xyz'] = np.array([[5.85, 0., .6]])
        clock = [100.]
        original = PathSafety.check
        def rewound(guard, *args, **kwargs):
            result = original(guard, *args, **kwargs)
            clock[0] -= .01
            return result
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(PathSafety, 'check', rewound):
            result = self.check(runtime, plan, world)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)

    def test_current_raw_and_person_contact_keep_emergency_authority(self):
        for hazard in ('raw', 'person'):
            with self.subTest(hazard=hazard):
                runtime, plan, world = self.fixture()
                if hazard == 'raw':
                    runtime.test_scan['xyz'] = np.array([[2.5, 0., .6]])
                else:
                    runtime.object_receiver.objects = [person(2.5, 0., 0., 0.)]
                with patch('time.monotonic', return_value=100.):
                    result = self.check(runtime, plan, world)
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)

    def test_curve_profile_is_fresh_and_scoped_to_a_committed_constant_ring(self):
        for committed, constant in ((True, True), (True, False), (False, True)):
            runtime, plan, world = self.fixture(following=False)
            runtime.gap_planner.junction.update(yield_only=True, constant_ring_speed=constant,
                                                line_station_m=50., line_id='entry')
            runtime.lanes.route_progress = lambda pose: float(pose['x'])
            if committed:
                runtime.gap_planner.latch_junction_commit()
            with patch('time.monotonic', return_value=100.):
                result = self.check(runtime, plan, world)
            if committed and constant:
                self.assertFalse(result.emergency, result.stop_reason)
                profile = result.validation['junction_curve_profile']
                self.assertEqual(profile['target_mps'], 25./3.6)
                self.assertGreater(profile['expires_at'], 100.)
                self.assertLessEqual(profile['expires_at'], 100.2)
            else:
                self.assertNotIn('junction_curve_profile', result.validation)

    def test_legacy_junction_keeps_its_current_command_forecast(self):
        runtime, plan, world = self.fixture(following=False)
        checks = []
        original = PathSafety.check
        def checked(guard, *args, **kwargs):
            checks.append(kwargs['requested_mps'])
            return original(guard, *args, **kwargs)
        with patch('time.monotonic', return_value=100.), \
                patch.object(PathSafety, 'check', checked), \
                patch('junction_vehicle_motion.certify_moving_cars', wraps=certify_moving_cars) as certify:
            self.check(runtime, plan, world)
        self.assertEqual(checks, [1.28])
        self.assertEqual(certify.call_count, 0)


if __name__ == '__main__':
    unittest.main()
