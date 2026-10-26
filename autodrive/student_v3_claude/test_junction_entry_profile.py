"""Prospective entry evidence never grants movement without live approval."""
from contextlib import contextmanager
from dataclasses import replace
import copy
import math
import unittest
from unittest.mock import patch

import numpy as np

from driving_plan import DrivingPlan
from ego_body import EGO_FRONT_M
from junction_entry_profile import bind_entry_profile, entry_profile_ceiling
from live_path_safety import LivePathSafety
from stop_line import competition_stop_controller
import test_live_path_safety as live_fixture
import test_roundabout_empty_entry as entry_fixture


class EntryProfileTests(unittest.TestCase):
    def plan(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0., stamp=100., age_s=.01)
        return DrivingPlan(1, entry_fixture.PATH, ego, ego, 99.99, .01, 100., 0., 60., True,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False),
            'ROUTE BASE', 'junction traffic wait')

    def test_binding_keeps_wait_command_reason_and_source_deadlines(self):
        original = self.plan()
        plan = bind_entry_profile(original, 8., 'entry', 50.)
        self.assertEqual((plan.allowed_mps, plan.emergency, plan.stop_reason),
                         (0., True, original.stop_reason))
        self.assertEqual((plan.sensor_stamp, plan.sensor_age_s, plan.checked_at),
                         (original.sensor_stamp, original.sensor_age_s, original.checked_at))
        self.assertEqual(entry_profile_ceiling(plan, 100.), 8.)
        self.assertNotIn('junction_entry_profile', original.validation)

    def test_changed_path_pose_line_and_non_yield_scope_refuse_the_hint(self):
        plan = bind_entry_profile(self.plan(), 8., 'entry', 50.)
        for altered in (replace(plan, path=plan.path+[0., .1]),
                        replace(plan, control_ego=dict(plan.control_ego, heading_deg=.1)),
                        plan.decided(blocked=True), plan.decided(valid=False)):
            self.assertIsNone(entry_profile_ceiling(altered, 100.))
        junction = dict(yield_only=True, ready=True, line_id='entry', line_station_m=50.)
        self.assertEqual(entry_profile_ceiling(plan, 100., junction), 8.)
        for fields in (dict(line_id='other'), dict(line_station_m=50.01), dict(ready=False),
                       dict(following_only=True), dict(recovery_only=True), dict(yield_only=False)):
            self.assertIsNone(entry_profile_ceiling(plan, 100., dict(junction, **fields)))

    def test_stale_or_malformed_evidence_cannot_supply_a_profile(self):
        plan = bind_entry_profile(self.plan(), 8., 'entry', 50.)
        self.assertIsNone(entry_profile_ceiling(plan, 100.351))
        self.assertIsNone(entry_profile_ceiling(plan, 99.999))
        for cap in (0., -1., math.nan, math.inf, 'bad'):
            self.assertNotIn('junction_entry_profile', bind_entry_profile(self.plan(), cap,
                'entry', 50.).validation)
        for key, value in (('traffic_wait_only', False), ('path_digest', 'other'),
                           ('frame_digest', 'other'), ('line_id', ''), ('line_station_m', math.nan)):
            altered = copy.deepcopy(plan.validation['junction_entry_profile'])
            altered[key] = value
            self.assertIsNone(entry_profile_ceiling(plan.decided(junction_entry_profile=altered), 100.))

    def test_live_result_must_be_bound_again_after_a_pose_refresh(self):
        plan = bind_entry_profile(self.plan(), 8., 'entry', 50.)
        refreshed = replace(plan, control_ego=dict(plan.control_ego, stamp=100.1),
            checked_at=100.1, allowed_mps=3., emergency=False)
        self.assertIsNone(entry_profile_ceiling(refreshed, 100.1))
        refreshed = bind_entry_profile(refreshed, 8., 'entry', 50.)
        self.assertEqual(entry_profile_ceiling(refreshed, 100.1), 8.)

class EntryProfileGuiTests(unittest.TestCase):
    def wait_runtime(self, gap=.95):
        fixture, runtime = entry_fixture.EmptyRuntimeTests().runtime()
        runtime.stop_line_controller = competition_stop_controller([entry_fixture.LINE])
        ego = runtime.test_status['ego']
        ego.update(x=50.-EGO_FRONT_M-gap, speed_kmh=0.)
        runtime.stop_line_controller.update(ego['x'], 1.5, 99.9, False,
            observation_age_s=.1, empty_front_station=50.-gap)
        self.assertTrue(runtime.stop_line_controller.yield_wait_latched)
        runtime.test_cars = [fixture.car(ego['x']+10., 8., vy=-3., heading=-90.)]
        captured = []
        class Capture:
            def __call__(self, world, current, mode, plan):
                captured.append(plan)
                return plan
        runtime.path_publisher._safety_provider = Capture()
        return fixture, runtime, captured

    def live_timer_runtime(self):
        from virtual_sim import SimulatedVehicleReceiver
        fixture, runtime, _ = self.wait_runtime()
        with patch('gui.crossing_caution', return_value=None):
            fixture.step(runtime, 0)
        clock = [100.]
        runtime.vehicle_receiver = SimulatedVehicleReceiver()
        runtime.lidar.scan_snapshot = lambda: dict(stamp=clock[0], age_s=.01,
            obstacle_points=runtime.lidar.pts.copy())
        runtime.path_publisher._safety_provider = LivePathSafety(runtime)
        return runtime, clock

    @staticmethod
    def refresh_timer_sources(runtime, now, cars=()):
        runtime.test_status['ego'].update(stamp=now, age_s=.01)
        runtime.object_receiver.observation_stamp = now
        runtime.object_receiver.received = now
        runtime.lidar.received = now
        runtime.vehicle_receiver.publish_rows(list(cars), stamp=now)

    def test_successful_launch_continues_for_five_timer_frames_without_extending_geometry(self):
        runtime, clock = self.live_timer_runtime()
        publisher = runtime.path_publisher
        baseline = publisher._safety_baseline
        created, world = baseline['created'], baseline['world_path']
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            for index in range(6):
                clock[0] = 100.+index*.05
                self.refresh_timer_sources(runtime, clock[0])
                publisher.recheck_active_path()
                self.assertGreater(publisher._last_target_mps, 0.)
                self.assertTrue(runtime.gap_planner.junction_committed)
                self.assertGreater(baseline['target'], 0.)
                self.assertFalse(baseline['decision'].emergency)
                self.assertEqual(baseline['created'], created)
                self.assertIs(baseline['world_path'], world)

    def test_wait_evidence_refreshes_beyond_its_first_source_deadline_without_geometry_renewal(self):
        runtime, clock = self.live_timer_runtime()
        publisher = runtime.path_publisher
        baseline = publisher._safety_baseline
        created, world = baseline['created'], baseline['world_path']
        ego = runtime.test_status['ego']
        incoming = live_fixture.car(ego['x']+10., 8., 0., -3., 81)
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            for index in range(8):
                clock[0] = 100.+index*.1
                self.refresh_timer_sources(runtime, clock[0], [incoming])
                publisher.recheck_active_path()
                current = baseline['decision']
                self.assertTrue(current.validation['junction_entry_profile_refreshed'])
                self.assertEqual(current.control_ego['stamp'], clock[0])
                self.assertIsNotNone(entry_profile_ceiling(current, clock[0]))
                self.assertEqual(publisher._last_target_mps, 0.)
                self.assertFalse(runtime.gap_planner.junction_committed)
                self.assertEqual(baseline['created'], created)
                self.assertIs(baseline['world_path'], world)

    def test_new_raw_person_or_stale_pose_stops_a_promoted_launch_without_refreshing_its_baseline(self):
        for hazard in ('raw', 'person', 'stale'):
            runtime, clock = self.live_timer_runtime()
            publisher = runtime.path_publisher
            with self.subTest(hazard=hazard), patch('time.monotonic', side_effect=lambda: clock[0]):
                self.refresh_timer_sources(runtime, clock[0])
                publisher.recheck_active_path()
                self.assertGreater(publisher._last_target_mps, 0.)
                baseline_plan = publisher._safety_baseline['decision']
                clock[0] += .05
                self.refresh_timer_sources(runtime, clock[0])
                if hazard == 'raw':
                    runtime.lidar.pts = np.array([[3., 0.]])
                elif hazard == 'person':
                    runtime.object_receiver.objects = [live_fixture.person(
                        runtime.test_status['ego']['x']+3., 0., 0., 0.)]
                else:
                    runtime.test_status['ego']['age_s'] = .3
                publisher.recheck_active_path()
                self.assertEqual(publisher._last_target_mps, 0.)
                self.assertTrue(publisher._active_decision_plan.emergency)
                self.assertIs(publisher._safety_baseline['decision'], baseline_plan)

    def test_source_rejection_or_publication_failure_cannot_promote_a_positive_candidate(self):
        for failure in ('source_changed', 'publish_exception'):
            runtime, clock = self.live_timer_runtime()
            publisher = runtime.path_publisher
            provider = publisher._safety_provider
            baseline = publisher._safety_baseline
            original, created, world = baseline['decision'], baseline['created'], baseline['world_path']
            real_guard = provider.publication_source_guard
            @contextmanager
            def changed_source(plan):
                runtime.vehicle_receiver.publish_rows([], stamp=100.01)
                with real_guard(plan) as accepted:
                    yield accepted
            with self.subTest(failure=failure), patch('time.monotonic', side_effect=lambda: clock[0]):
                self.refresh_timer_sources(runtime, clock[0])
                if failure == 'source_changed':
                    with patch.object(provider, 'publication_source_guard', changed_source):
                        publisher.recheck_active_path()
                    self.assertEqual(publisher._last_target_mps, 0.)
                else:
                    with patch.object(publisher.waypoints_pub, 'publish',
                            side_effect=RuntimeError('ROS publish failure')):
                        with self.assertRaisesRegex(RuntimeError, 'ROS publish failure'):
                            publisher.recheck_active_path()
                self.assertIs(baseline['decision'], original)
                self.assertEqual(baseline['created'], created)
                self.assertIs(baseline['world_path'], world)
                self.assertFalse(runtime.gap_planner.junction_committed)

    def test_post_delivery_expiry_rejection_cannot_promote_the_launch_baseline(self):
        runtime, clock = self.live_timer_runtime()
        publisher = runtime.path_publisher
        provider = publisher._safety_provider
        baseline = publisher._safety_baseline
        original, created, world = baseline['decision'], baseline['created'], baseline['world_path']
        note = provider.note_published
        def delayed_notification(plan, path, target, now):
            self.assertGreater(target, 0.)
            self.assertTrue(plan.validation['junction_source_launch']['consumed'])
            clock[0] += .3
            note(plan, path, target, clock[0])
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            self.refresh_timer_sources(runtime, clock[0])
            with patch.object(provider, 'note_published', delayed_notification):
                publisher.recheck_active_path()
            self.assertIn('junction_source_rejected', publisher._active_decision_plan.validation)
            self.assertIs(baseline['decision'], original)
            self.assertEqual(baseline['created'], created)
            self.assertIs(baseline['world_path'], world)
            self.assertFalse(runtime.gap_planner.junction_committed)

    def test_positive_traffic_wait_follow_captured_to_zero_keeps_a_separate_profile(self):
        fixture, runtime, captured = self.wait_runtime()
        # The mapped traffic owner is the only stop in this case; the
        # independent unclassified-crossing fallback is covered separately.
        with patch('gui.crossing_caution', return_value=None):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['traffic_plan']['ring']['action'], 'WAIT')
        self.assertGreater(row['traffic_plan']['target_mps'], 0.)
        self.assertEqual(row['stop_line']['state'], 'YIELD_WAIT')
        self.assertEqual(row['published_cap_kmh'], 0.)
        self.assertTrue(captured[-1].emergency)
        self.assertGreater(entry_profile_ceiling(captured[-1], 100.), 0.)
        self.assertFalse(runtime.gap_planner.junction_committed or runtime.stop_line_controller.released)

    def test_retained_independent_stop_and_late_acc_emergency_cannot_be_laundered(self):
        for condition in ('retained', 'late_acc'):
            fixture, runtime, captured = self.wait_runtime()
            if condition == 'retained':
                runtime._yield_stop_until = 101.
            else:
                runtime.lead_follow_step = lambda *a, **k: dict(cap_mps=0., emergency=True,
                    state=dict(gap_m=.1))
            with self.subTest(condition=condition):
                row = fixture.step(runtime, 0)
                self.assertEqual(row['published_cap_kmh'], 0.)
                self.assertNotIn('junction_entry_profile', captured[-1].validation)

    def test_physical_constant_ring_wait_gets_a_proposal_without_releasing_the_stop(self):
        fixture, runtime, captured = self.wait_runtime()
        runtime.stop_line_controller.active = dict(runtime.stop_line_controller.active, yield_advance_m=7.)
        runtime._yield_stop_until = 101.
        with patch('gui.crossing_caution', return_value=None):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['published_cap_kmh'], 0.)
        self.assertTrue(captured[-1].emergency)
        self.assertGreater(entry_profile_ceiling(captured[-1], 100.), 5.)
        self.assertFalse(runtime.gap_planner.junction_committed or runtime.stop_line_controller.released)

    def test_physical_wait_requires_a_new_live_source_check_and_raw_obstacles_still_stop_it(self):
        from virtual_sim import SimulatedVehicleReceiver
        fixture, runtime, _ = self.wait_runtime()
        runtime.stop_line_controller.active = dict(runtime.stop_line_controller.active, yield_advance_m=7.)
        runtime._yield_stop_until = 101.
        with patch('gui.crossing_caution', return_value=None):
            fixture.step(runtime, 0)
        clock = [100.]
        runtime.vehicle_receiver = SimulatedVehicleReceiver()
        runtime.lidar.scan_snapshot = lambda: dict(stamp=clock[0], age_s=.01,
            obstacle_points=runtime.lidar.pts.copy())
        runtime.path_publisher._safety_provider = LivePathSafety(runtime)
        publisher = runtime.path_publisher
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            self.refresh_timer_sources(runtime, clock[0])
            publisher.recheck_active_path()
            self.assertGreater(publisher._last_target_mps, 0.)
            self.assertTrue(publisher._active_decision_plan.validation['junction_source_launch']['consumed'])
            clock[0] += .05
            self.refresh_timer_sources(runtime, clock[0])
            runtime.lidar.pts = np.array([[3., 0.]])
            publisher.recheck_active_path()
            self.assertEqual(publisher._last_target_mps, 0.)

    def test_unclassified_crossing_caution_is_not_a_pure_mapped_traffic_wait(self):
        fixture, runtime, captured = self.wait_runtime()
        row = fixture.step(runtime, 0)
        self.assertTrue(row['yield']['name'].startswith('crossing_caution'))
        self.assertEqual(row['published_cap_kmh'], 0.)
        self.assertNotIn('junction_entry_profile', captured[-1].validation)

    def test_actual_live_timer_rechecks_wait_zero_and_preserves_new_independent_hazards(self):
        from virtual_sim import SimulatedVehicleReceiver
        for hazard in (None, 'raw', 'person', 'stale'):
            fixture, runtime, captured = self.wait_runtime()
            with patch('gui.crossing_caution', return_value=None):
                fixture.step(runtime, 0)
            original = captured[-1]
            self.assertEqual(original.allowed_mps, 0.)
            self.assertTrue(original.emergency)
            publisher = runtime.path_publisher
            runtime.vehicle_receiver = SimulatedVehicleReceiver()
            runtime.object_receiver.observation_stamp = 100.
            publisher._safety_provider = LivePathSafety(runtime)
            if hazard == 'raw':
                runtime.lidar.pts = np.array([[3., 0.]])
            elif hazard == 'person':
                runtime.object_receiver.objects = [live_fixture.person(
                    runtime.test_status['ego']['x']+3., 0., 0., 0.)]
            with self.subTest(hazard=hazard), patch('time.monotonic', return_value=100.):
                runtime.vehicle_receiver.publish_rows([], stamp=99.64 if hazard == 'stale' else 100.)
                publisher.recheck_active_path()
                payload = live_fixture.LivePathSafetyTests().payload(runtime)
                if hazard is None:
                    self.assertGreater(payload['target_speed_mps'], 0.)
                    active = publisher._active_decision_plan
                    self.assertTrue(active.validation['junction_entry_profile_accepted'])
                    self.assertIsNotNone(entry_profile_ceiling(active, 100.))
                    self.assertEqual(payload['decision']['validation']['live_safety']
                        ['junction_traffic']['state'], 'ENTER')
                    self.assertTrue(runtime.gap_planner.junction_committed)
                else:
                    self.assertEqual(payload['target_speed_mps'], 0.)
                    self.assertTrue(payload['decision']['emergency'])
                    self.assertFalse(runtime.gap_planner.junction_committed)


class PublicationGuardGuiTests(unittest.TestCase):
    def test_encoding_expiry_rolls_back_the_actual_new_launch_before_and_after_the_line(self):
        from virtual_sim import SimulatedVehicleReceiver
        import gui
        for timer in (False, True):
            for gap in (.95, -.01):
                fixture, runtime, captured = EntryProfileGuiTests().wait_runtime(gap)
                with patch('gui.crossing_caution', return_value=None):
                    fixture.step(runtime, 0)
                original = captured[-1]
                self.assertEqual(original.allowed_mps, 0.)
                self.assertFalse(runtime.gap_planner.junction_committed)
                self.assertFalse(runtime.stop_line_controller.released)
                receiver = SimulatedVehicleReceiver()
                runtime.vehicle_receiver = receiver
                runtime.object_receiver.observation_stamp = 100.
                publisher = runtime.path_publisher
                publisher._safety_provider = LivePathSafety(runtime)
                original_baseline = publisher._safety_baseline['decision']
                clock = [100.]
                dumps = gui.json.dumps
                def delayed_encoding(payload, *args, **kwargs):
                    encoded = dumps(payload, *args, **kwargs)
                    if isinstance(payload, dict) and payload.get('target_speed_mps', 0.) > 0.:
                        request = payload['decision']['validation']['junction_source_launch']
                        self.assertTrue(request['consumed'])
                        self.assertFalse(request['published'])
                        self.assertTrue(runtime.gap_planner.junction_committed)
                        clock[0] += .3
                    return encoded
                with self.subTest(timer=timer, gap=gap), \
                        patch('time.monotonic', side_effect=lambda: clock[0]):
                    receiver.publish_rows([], stamp=100.)
                    with patch('gui.json.dumps', side_effect=delayed_encoding):
                        if timer:
                            publisher.recheck_active_path()
                        else:
                            publisher.publish(runtime.test_status['ego'], original.path,
                                original.path, dict(mode='ROUTE BASE', reason='route'), original)
                payload = live_fixture.LivePathSafetyTests().payload(runtime)
                self.assertEqual(payload['target_speed_mps'], 0.)
                self.assertTrue(payload['decision']['emergency'])
                self.assertEqual(payload['decision']['stop_reason'],
                    'junction motion proof expired before command publication')
                self.assertFalse(runtime.gap_planner.junction_committed)
                self.assertFalse(publisher._safety_provider.traffic.junction_committed)
                self.assertFalse(runtime.stop_line_controller.released)
                self.assertIs(publisher._safety_baseline['decision'], original_baseline)

    def test_source_guard_owns_serialization_both_commands_and_notification(self):
        for timer in (False, True):
            for accepted in (False, True):
                fixture = live_fixture.LivePathSafetyTests()
                runtime = fixture.attach(fixture.runtime())
                runtime.infer()
                publisher = runtime.path_publisher
                original = publisher._safety_provider
                active = [False]
                notes = []
                class Guarded:
                    def __call__(self, *args, **kwargs):
                        return original(*args, **kwargs)
                    @contextmanager
                    def publication_source_guard(self, plan):
                        active[0] = True
                        plan.validation['source_guard_entered'] = True
                        try:
                            yield accepted
                        finally:
                            active[0] = False
                    def note_published(self, plan, world, target, now):
                        if not active[0]:
                            raise AssertionError('notification escaped source lock')
                        notes.append(target)
                publisher._safety_provider = Guarded()
                publish_path = publisher.waypoints_pub.publish
                publish_speed = publisher.velocity_pub.publish
                def path_command(message):
                    self.assertTrue(active[0])
                    publish_path(message)
                def speed_command(message):
                    self.assertTrue(active[0])
                    publish_speed(message)
                with self.subTest(timer=timer, accepted=accepted), \
                        patch.object(publisher.waypoints_pub, 'publish', path_command), \
                        patch.object(publisher.velocity_pub, 'publish', speed_command):
                    if timer:
                        publisher._last_target_mps = 0.
                        publisher.recheck_active_path()
                    else:
                        runtime.infer()
                    payload = fixture.payload(runtime)
                    self.assertTrue(payload['decision']['validation']['source_guard_entered'])
                    self.assertTrue(notes)
                    if accepted:
                        self.assertGreater(payload['target_speed_mps'], 0.)
                    else:
                        self.assertEqual(payload['target_speed_mps'], 0.)
                        self.assertTrue(payload['decision']['emergency'])
                        self.assertEqual(publisher._last_target_mps, 0.)


if __name__ == '__main__':
    unittest.main()
