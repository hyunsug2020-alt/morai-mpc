"""Moving roundabout yield: no mandatory stop, freshly checked near-line launch."""
import copy
import math
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from ego_body import EGO_FRONT_M
from live_path_safety import LivePathSafety
from path_safety import SafetyResult
from stop_line import competition_stop_controller, yield_line_cap
from traffic_planner import GapPlanner
import ring_entry
from test_junction_entry import car
from test_roundabout_empty_entry import LINE, PATH
import test_roundabout_empty_entry as legacy_fixture
import test_live_path_safety as live_fixture


def planner(gap, speed=12.):
    p = GapPlanner()
    p.junction = dict(held=True, ready=True, yield_only=True, allow_empty_entry=True,
        empty_actual_body=True, line_id=LINE['id'], line_station_m=50., line_gap_m=gap,
        unknown_ids=[])
    ego = dict(x=50.-EGO_FRONT_M-gap, y=0., heading_deg=0., speed_kmh=speed,
               stamp=100., age_s=.01, received=99.99)
    return p, ego


class YieldGateTests(unittest.TestCase):
    def test_interrupted_launch_requires_new_proof_until_physical_crossing(self):
        for crossed in (False, True):
            gate = competition_stop_controller([LINE])
            gate.update(50.-EGO_FRONT_M-.95, 0., 100., True,
                observation_age_s=.01, empty_front_station=49.05)
            with patch('stop_line.time.monotonic', return_value=100.):
                self.assertTrue(gate.consume_yield_launch(LINE['id'], 50., 49.05,
                    0., .01, expires_at=100.1))
            cancelled = gate.update_yield_launch(50.01 if crossed else 49.05, stopped=True)
            self.assertEqual(cancelled, not crossed)
            self.assertEqual(gate.released, crossed)
            self.assertFalse(gate.yield_launch_pending)

    def test_unconfirmed_pose_stop_cancels_the_pending_launch(self):
        gate = competition_stop_controller([LINE])
        gate.active, gate.released, gate.yield_launch_pending = LINE, True, True
        self.assertTrue(gate.update_yield_launch(math.nan, stopped=True))
        self.assertFalse(gate.released or gate.yield_launch_pending)

    def test_gui_enter_pose_cannot_confirm_the_physical_crossing(self):
        gate = competition_stop_controller([LINE])
        gate.active, gate.released, gate.yield_launch_pending = LINE, True, True
        result = gate.update(50.-EGO_FRONT_M+.01, 0., 100., False,
            observation_age_s=.201, empty_front_station=50.01)
        self.assertEqual(result['state'], 'ENTER')
        self.assertTrue(gate.yield_launch_pending)
        self.assertTrue(gate.update_yield_launch(math.nan, stopped=True))
        self.assertFalse(gate.released)

    def test_wait_stays_at_stopped_position_until_live_go(self):
        gate = competition_stop_controller([LINE])
        station, front = 50.-EGO_FRONT_M-.55, 49.45
        for now in (100., 100.1, 100.2):
            waiting = gate.update(station, 0., now, False, empty_front_station=front)
            self.assertEqual(waiting['cap_mps'], 0.)
            self.assertTrue(gate.yield_wait_latched)
        proposed = gate.update(station, 0., 100.3, True,
            observation_age_s=.01, empty_front_station=front)
        self.assertGreater(proposed['cap_mps'], 0.)
        self.assertTrue(gate.yield_wait_latched)
        with patch('stop_line.time.monotonic', return_value=100.3):
            self.assertEqual(gate.apply_yield_wait(LINE['id'], 50., front, 0., 1.), 0.)
            self.assertEqual(gate.apply_yield_wait(LINE['id'], 50., front, 0., 3.,
                go=True, expires_at=100.4), 3.)
        self.assertFalse(gate.yield_wait_latched)

    def test_clear_entry_has_no_stop_window_dwell_or_rolling_speed_limit(self):
        for speed in (0., 12./3.6, 20./3.6, 40./3.6):
            gate = competition_stop_controller([LINE])
            for index, gap in enumerate((18., 5., 1., .01, -.01)):
                result = gate.update(50.-EGO_FRONT_M-gap, speed, index*.1, True,
                    observation_age_s=.01, empty_front_station=50.-gap)
                self.assertEqual(result['state'], 'APPROACH_YIELD')
                self.assertTrue(result['gap_checked'])
                self.assertFalse(result['stop'] or gate.released or gate.hold_latched)
                self.assertAlmostEqual(result['cap_mps']*3.6, 30.)
            self.assertTrue(gate.consume_yield_crossing(LINE['id'], 50., 50.01,
                speed, .01, expires_at=time.monotonic()+.1))
            self.assertTrue(gate.released)

    def test_unsafe_gap_can_stop_and_the_first_go_restarts_without_dwell(self):
        gate = competition_stop_controller([LINE])
        station = 50.-EGO_FRONT_M-.5
        waiting = gate.update(station, 0., 0., False, empty_front_station=49.5)
        self.assertEqual(waiting['state'], 'YIELD_WAIT')
        self.assertEqual(waiting['cap_mps'], 0.)
        go = gate.update(station, 0., .01, True, empty_front_station=49.5)
        self.assertTrue(go['gap_checked'])
        self.assertAlmostEqual(go['cap_mps']*3.6, 30.)
        self.assertFalse(go['stop'] or gate.stop_latched or gate.hold_latched)

    def test_uncovered_far_gap_stays_in_braking_envelope_without_commit(self):
        gate = competition_stop_controller([LINE])
        r = gate.update(50.-EGO_FRONT_M-40., 40./3.6, 0., True,
                        observation_age_s=.1, empty_front_station=10.)
        self.assertFalse(r['gap_checked'] or gate.released)
        self.assertEqual(r['cap_mps'], yield_line_cap(40., 40./3.6, .1))

    def test_old_pose_and_expired_or_wrong_line_proof_cannot_commit(self):
        gate = competition_stop_controller([LINE])
        gate.update(50.-EGO_FRONT_M+.01, 3., 100., True,
                    observation_age_s=.201, empty_front_station=50.01)
        self.assertFalse(gate.yield_approach or gate.released)
        gate.update(50.-EGO_FRONT_M+.01, 3., 100.01, True,
                    observation_age_s=.01, empty_front_station=50.01)
        for line_id, expiry in (('other', 101.), (LINE['id'], 99.)):
            with patch('stop_line.time.monotonic', return_value=100.):
                self.assertFalse(gate.consume_yield_crossing(line_id, 50., 50.01,
                    3., .01, expires_at=expiry))
        self.assertFalse(gate.released)

    def test_delayed_empty_approach_crosses_without_a_physical_stop(self):
        for kmh in (12., 20., 40.):
            gate = competition_stop_controller([LINE])
            dt, x, speed = 1./150., -35., kmh/3.6
            history, commands = [], []
            target, next_tick, minimum = speed, 0., speed
            for step in range(15000):
                now = step*dt
                history.append((x, speed))
                if now >= next_tick:
                    ox, ov = history[max(0, step-23)]
                    result = gate.update(ox, ov, now, True, observation_age_s=.1534,
                        empty_front_station=ox+EGO_FRONT_M)
                    commands.append((now+.5, min(kmh/3.6, result['cap_mps']) if result else kmh/3.6))
                    next_tick = now+1./15.
                while commands and commands[0][0] <= now:
                    target = commands.pop(0)[1]
                previous = speed
                speed = min(target, speed+1.2*dt) if target > speed else max(target, speed-2.5*dt)
                x += .5*(previous+speed)*dt
                minimum = min(minimum, speed)
                if x+EGO_FRONT_M >= 50.:
                    break
            with self.subTest(kmh=kmh):
                self.assertLess(step, 14999)
                self.assertGreater(minimum, .15)
                self.assertFalse(gate.released)


class YieldTrafficTests(unittest.TestCase):
    def test_far_empty_horizon_is_not_a_gap_or_a_commit(self):
        p, ego = planner(80., 40.)
        p.evaluate([], ego, PATH+[ego['x'], 0.], 30./3.6, 100.)
        self.assertFalse(p.junction_committed)
        self.assertNotIn('ring', p.status)
        self.assertEqual(p.status['state'], 'APPROACH_YIELD')

    def test_moving_go_is_permission_until_physical_crossing(self):
        p, ego = planner(5.)
        r = p.evaluate([], ego, PATH, 30./3.6, 100.)
        self.assertEqual(p.status['ring']['action'], 'GO')
        self.assertEqual(p.status['state'], 'APPROACH_YIELD')
        self.assertGreater(r['v_mps']*3.6, 2.5)
        self.assertFalse(p.junction_committed)

    def test_inside_metadata_cannot_bypass_canonical_crossing_permission(self):
        p, ego = planner(-3.)
        p.junction.update(held=False, entry_m=3., ready=False)
        r = p.evaluate([], ego, PATH, 8., 100.)
        self.assertFalse(p.junction_committed)
        self.assertEqual(p.status['state'], 'YIELD_WAIT')
        self.assertEqual(r['v_mps'], 0.)

    def test_new_crossing_or_unknown_car_revokes_moving_permission(self):
        for valid in (True, False):
            p, ego = planner(5.)
            ego.update(x=0., speed_kmh=12.)
            p.evaluate([], ego, PATH, 8., 100.)
            r = p.evaluate([car(10., 8., 0., -3., 1, valid=valid)], ego, PATH, 8., 100.1)
            self.assertEqual(p.status['state'], 'YIELD_WAIT')
            self.assertNotEqual(p.status.get('ring', {}).get('action'), 'GO')
            self.assertLess(r['v_mps'], 8.)
            self.assertFalse(p.junction_committed)

    def test_earliest_physical_nose_rejects_an_old_standstill_model_gap(self):
        obj = car(16., 4., 5., 0., 1)
        c = dict(s_lane=8., s_cp=10., arc_lane=1., arc_cp=4., arc_exit=None)
        vehicle = ring_entry.RingCar(obj, 5., 1.1, 'moving', c, None)
        legacy = ring_entry.entry_decision([vehicle], 0., 12./3.6, 8.)
        moving = ring_entry.entry_decision([vehicle], 0., 12./3.6, 8., yield_only=True)
        self.assertEqual(legacy['action'], 'GO')
        self.assertEqual(moving['action'], 'WAIT')

    def test_stopped_ego_can_follow_a_passing_car_without_nan_timing(self):
        obj = car(16., 4., 5., 0., 1)
        c = dict(s_lane=8., s_cp=10., arc_lane=1., arc_cp=4., arc_exit=None)
        vehicle = ring_entry.RingCar(obj, 5., 1.1, 'moving', c, None)
        result = ring_entry.entry_decision([vehicle], 0., 0., 8., yield_only=True)
        self.assertEqual(result['action'], 'GO')
        self.assertEqual(result['ahead'], [1])

    def test_rear_bound_does_not_promise_earlier_clearance_for_a_braking_car(self):
        obj = car(16., 4., 0., 0., 1)
        c = dict(s_lane=8., s_cp=10., arc_lane=1., arc_cp=4., arc_exit=None)
        vehicle = ring_entry.RingCar(obj, 0., 1.1, 'standing', c, None)
        stationary = ring_entry.entry_decision([vehicle], 0., 0., 8., yield_only=True)
        moving = ring_entry.entry_decision([vehicle], 0., 20./3.6, 8., yield_only=True)
        self.assertEqual(stationary['checks'][0]['rear_clear_s'], moving['checks'][0]['rear_clear_s'])

    def test_new_low_speed_cap_cannot_delay_the_predicted_earliest_nose(self):
        obj = car(16., 4., 5., 0., 1)
        c = dict(s_lane=8., s_cp=10., arc_lane=1., arc_cp=4., arc_exit=None)
        vehicle = ring_entry.RingCar(obj, 5., 1.1, 'moving', c, None)
        result = ring_entry.entry_decision([vehicle], 0., 20./3.6, 1.,
            speed_limit=lambda station:1., yield_only=True)
        self.assertEqual(result['action'], 'WAIT')
        self.assertNotIn(1, result['ahead'])


class YieldPublicationTests(unittest.TestCase):
    def test_live_wait_or_invalid_go_cannot_restart_a_traffic_stop(self):
        for condition in ('blocker', 'unknown', 'expired', 'ineligible', 'person', 'raw', 'clear'):
            guard, runtime, ego = self.fixture(.55)
            ego['speed_kmh'] = 0.
            gate = runtime.stop_line_controller
            gate.update(ego['x'], 0., 99.9, False, empty_front_station=49.45)
            gate.update(ego['x'], 0., 100., True, observation_age_s=.01,
                empty_front_station=49.45)
            self.assertTrue(gate.yield_wait_latched)
            cars = [car(ego['x']+10., 8., 0., -3., 1, valid=condition!='unknown')] \
                if condition in ('blocker', 'unknown') else []
            body = SafetyResult(0., True, 0., condition, {}) if condition in ('person', 'raw') else None
            with self.subTest(condition=condition):
                result = self.check(guard, runtime, ego, cars,
                    expiry=99. if condition=='expired' else 100.3,
                    eligible=condition!='ineligible', result=body)
                if condition == 'clear':
                    self.assertGreater(result.cap_mps, 0.)
                    self.assertFalse(gate.yield_wait_latched)
                    self.assertTrue(gate.released and runtime.gap_planner.junction_committed)
                else:
                    self.assertEqual(result.cap_mps, 0.)
                    self.assertTrue(gate.yield_wait_latched)
                    self.assertFalse(gate.released or runtime.gap_planner.junction_committed)

    def test_failed_locked_crossing_does_not_release_wait_first(self):
        guard, runtime, ego = self.fixture(-.01)
        runtime.stop_line_controller.yield_wait_latched = True
        with patch.object(runtime.stop_line_controller, 'consume_yield_launch', return_value=False), \
                patch.object(runtime.stop_line_controller, 'apply_yield_wait',
                    side_effect=AssertionError('crossing must consume atomically')):
            result = self.check(guard, runtime, ego)
        self.assertEqual(result.cap_mps, 0.)
        self.assertTrue(runtime.stop_line_controller.yield_wait_latched)
        self.assertFalse(runtime.stop_line_controller.released or runtime.gap_planner.junction_committed)

    def test_checked_near_line_launch_is_committed_without_a_mandatory_stop(self):
        for gap in (2., .95, .01):
            for speed in (0., 12., 20.):
                guard, runtime, ego = self.fixture(gap)
                ego['speed_kmh'] = speed
                with self.subTest(gap=gap, speed=speed):
                    result = self.check(guard, runtime, ego)
                    self.assertGreater(result.cap_mps, 0.)
                    self.assertEqual(result.validation['junction_traffic']['state'], 'ENTER')
                    self.assertTrue(runtime.stop_line_controller.released)
                    self.assertTrue(runtime.gap_planner.junction_committed)

    def test_a_later_arrival_does_not_restart_an_already_checked_launch(self):
        guard, runtime, ego = self.fixture(.95)
        self.assertGreater(self.check(guard, runtime, ego).cap_mps, 0.)
        # A ring arrival during the accepted entry is now handled by following;
        # restarting a standstill rear-clearance prediction caused timer WAITs.
        arriving = car(ego['x']+14., 30., 0., -5.5, 2)
        with patch.object(guard.traffic, 'evaluate',
                side_effect=AssertionError('accepted launch must not be re-accepted')):
            result = self.check(guard, runtime, ego, [arriving])
        self.assertGreater(result.cap_mps, 0.)
        self.assertEqual(result.validation['junction_traffic']['state'], 'ENTER')

    def test_near_line_commit_keeps_raw_and_person_stops_after_acceptance(self):
        for reason in ('person', 'actual raw blocker'):
            guard, runtime, ego = self.fixture(.95)
            self.check(guard, runtime, ego)
            result = self.check(guard, runtime, ego,
                result=SafetyResult(0., True, 0., reason, {}))
            self.assertEqual(result.cap_mps, 0.)
            if reason == 'person':
                self.assertTrue(result.emergency)
            self.assertTrue(runtime.gap_planner.junction_committed)

    def test_outside_launch_area_keeps_rechecking_and_never_commits(self):
        guard, runtime, ego = self.fixture(2.01)
        result = self.check(guard, runtime, ego)
        self.assertGreater(result.cap_mps, 0.)
        self.assertEqual(result.validation['junction_traffic']['state'], 'APPROACH_YIELD')
        self.assertFalse(runtime.stop_line_controller.released or runtime.gap_planner.junction_committed)

    def fixture(self, gap=.01):
        p, ego = planner(gap, 12.)
        gate = competition_stop_controller([LINE])
        gate.update(ego['x'], 12./3.6, 100., True,
                    observation_age_s=.01, empty_front_station=50.-gap)
        runtime = SimpleNamespace(gap_planner=p, stop_line_controller=gate,
            lanes=SimpleNamespace(route_progress=lambda e:e['x']))
        guard = LivePathSafety(runtime)
        guard.traffic = copy.copy(p)
        return guard, runtime, ego

    def check(self, guard, runtime, ego, cars=(), *, ready=True, expiry=100.3, eligible=True, result=None):
        with patch('live_path_safety.time.monotonic', return_value=100.), \
                patch('stop_line.time.monotonic', return_value=100.):
            return guard.junction_follow(result or SafetyResult(8., False, 40., None, {}),
                runtime, list(cars), 1., PATH+[ego['x'], 0.], ego,
                yield_vehicle_ready=ready, yield_sources_expire_at=expiry,
                empty_commit_allowed=eligible)

    def test_crossing_consumes_only_a_current_checked_gap(self):
        guard, runtime, ego = self.fixture(-.01)
        r = self.check(guard, runtime, ego)
        self.assertFalse(r.emergency)
        self.assertTrue(runtime.stop_line_controller.released and runtime.gap_planner.junction_committed)
        self.assertEqual(r.validation['junction_traffic']['state'], 'ENTER')

    def test_new_blocker_expiry_or_old_invalid_publication_cannot_consume_go(self):
        for condition in ('blocker', 'unknown', 'not_ready', 'expired', 'ineligible', 'person', 'raw'):
            guard, runtime, ego = self.fixture(-.01)
            cars = [car(ego['x']+10., 8., 0., -3., 1, valid=condition!='unknown')] \
                if condition in ('blocker', 'unknown') else []
            result = SafetyResult(0., True, 0., condition, {}) if condition in ('person','raw') else None
            with self.subTest(condition=condition):
                r = self.check(guard, runtime, ego, cars, ready=condition!='not_ready',
                    expiry=99. if condition=='expired' else 100.3,
                    eligible=condition!='ineligible', result=result)
                self.assertEqual(r.cap_mps, 0.)
                self.assertTrue(r.emergency)
                self.assertFalse(runtime.stop_line_controller.released or runtime.gap_planner.junction_committed)

    def test_vehicle_source_loss_before_line_brakes_without_an_unconditional_stop(self):
        guard, runtime, ego = self.fixture(10.)
        r = self.check(guard, runtime, ego, ready=False)
        self.assertFalse(r.emergency)
        self.assertGreater(r.cap_mps, 0.)
        self.assertLess(r.cap_mps, 8.)
        self.assertFalse(runtime.gap_planner.junction_committed)

    def test_current_body_cap_bounds_the_rear_clearance_prediction(self):
        guard, runtime, ego = self.fixture(5.)
        evaluate = guard.traffic.evaluate
        targets = []
        def capture(objects, pose, path, target, now, **kwargs):
            targets.append(target)
            return evaluate(objects, pose, path, target, now, **kwargs)
        guard.traffic.evaluate = capture
        r = self.check(guard, runtime, ego, result=SafetyResult(3., False, 40., None, {}))
        self.assertEqual(targets, [3.])
        self.assertLessEqual(r.cap_mps, 3.)
        self.assertFalse(runtime.gap_planner.junction_committed)

    def test_body_speed_limit_rejects_a_slot_that_would_fit_a_faster_launch(self):
        states, rear_times, arrival = [], [], None
        for cap in (8., 3.):
            guard, runtime, ego = self.fixture(5.)
            r = self.check(guard, runtime, ego, [car(ego['x']+10., 34., 0., -5.5, 1)],
                           result=SafetyResult(cap, False, 40., None, {}))
            ring = r.validation['junction_traffic']['ring']
            states.append(ring['action'])
            rear_times.append(ring['checks'][0]['rear_clear_s'])
            arrival = ring['checks'][0]['earliest_s']
            self.assertFalse(runtime.gap_planner.junction_committed)
        self.assertEqual(states, ['GO', 'WAIT'])
        self.assertLess(rear_times[0]+ring_entry.NEXT_MARGIN_S, arrival)
        self.assertGreater(rear_times[1]+ring_entry.NEXT_MARGIN_S, arrival)

    def test_expiry_during_traffic_prediction_cannot_grant_a_crossing(self):
        guard, runtime, ego = self.fixture(-.01)
        clock = [100.]
        evaluate = guard.traffic.evaluate
        def slow(*args, **kwargs):
            result = evaluate(*args, **kwargs)
            clock[0] += .05
            return result
        guard.traffic.evaluate = slow
        with patch('live_path_safety.time.monotonic', side_effect=lambda:clock[0]), \
                patch('stop_line.time.monotonic', side_effect=lambda:clock[0]):
            result = guard.junction_follow(SafetyResult(8., False, 40., None, {}), runtime,
                [], 1., PATH+[ego['x'],0.], ego, yield_vehicle_ready=True,
                yield_sources_expire_at=100.01, empty_commit_allowed=True)
        self.assertTrue(result.emergency)
        self.assertFalse(result.validation['junction_traffic']['gap_checked'])
        self.assertFalse(runtime.gap_planner.junction_committed or runtime.stop_line_controller.released)

    def test_pose_deadline_survives_the_whole_preline_prediction(self):
        guard, runtime, ego = self.fixture(5.)
        ego.update(age_s=.199, received=99.801)
        clock = [100.]
        evaluate = guard.traffic.evaluate
        def slow(*args, **kwargs):
            result = evaluate(*args, **kwargs)
            clock[0] += .002
            return result
        guard.traffic.evaluate = slow
        with patch('live_path_safety.time.monotonic', side_effect=lambda:clock[0]):
            r = guard.junction_follow(SafetyResult(8., False, 40., None, {}), runtime,
                [], 1., PATH+[ego['x'],0.], ego, yield_vehicle_ready=True,
                yield_sources_expire_at=100.3, empty_commit_allowed=True)
        self.assertFalse(r.validation['junction_traffic']['gap_checked'])
        self.assertLess(r.cap_mps, 8.)
        self.assertFalse(runtime.gap_planner.junction_committed)


class YieldRuntimeTests(unittest.TestCase):
    def test_inflight_traffic_wait_does_not_cancel_a_timer_accepted_launch(self):
        fixture, runtime = legacy_fixture.EmptyRuntimeTests().runtime()
        gate = runtime.stop_line_controller = competition_stop_controller([LINE])
        runtime.intersection_regions[0]['line_id'] = LINE['id']
        ego = runtime.test_status['ego']
        ego.update(x=50.-EGO_FRONT_M-.95, speed_kmh=0.)
        planner = runtime.gap_planner
        evaluate, apply = planner.evaluate, runtime.apply_stop_line
        def traffic(*args, **kwargs):
            if not planner.junction_committed:
                return dict(kind='stop', source='traffic', name='gap', along=0., t=0.,
                    v_mps=0., reason='junction: wait at the line for a ring gap')
            return evaluate(*args, **kwargs)
        def timer_accepts_before_application(*args, **kwargs):
            self.assertTrue(kwargs['traffic_stop_only'])
            gate.active, gate.yield_approach = LINE, True
            self.assertTrue(gate.consume_yield_launch(LINE['id'], 50., 49.05,
                0., .01, expires_at=100.1))
            planner.latch_junction_commit()
            return apply(*args, **kwargs)
        with patch.object(planner, 'evaluate', side_effect=traffic), \
                patch.object(runtime, 'apply_stop_line', side_effect=timer_accepts_before_application):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'ENTER')
        self.assertFalse(row['emergency_stop'])
        self.assertGreater(row['published_cap_kmh'], 0.)
        self.assertTrue(planner.junction_committed)

    def test_late_gui_enter_cannot_restore_a_cancelled_launch(self):
        _, runtime = legacy_fixture.EmptyRuntimeTests().runtime()
        gate = runtime.stop_line_controller = competition_stop_controller([LINE])
        gate.active = LINE
        runtime.zone_kmh = 30.
        runtime.yield_cap_mps = None
        runtime._intersection_vehicle_status = dict(detector_ready=True)
        def cancel_then_return_old_enter(*args, **kwargs):
            runtime.gap_planner.junction_committed = False
            gate.released = False
            return dict(state='ENTER', cap_mps=8., stop=False)
        with patch.object(gate, 'update', side_effect=cancel_then_return_old_enter):
            runtime.apply_stop_line(runtime.test_status['ego'], 100., None, False, True)
        self.assertFalse(runtime.gap_planner.junction_committed)

    def test_first_region_frame_keeps_line_identity_and_positive_approach_cap(self):
        fixture, runtime = legacy_fixture.EmptyRuntimeTests().runtime()
        runtime.stop_line_controller = competition_stop_controller([LINE])
        runtime.test_status['ego'].update(x=50.-EGO_FRONT_M-25., speed_kmh=20.)
        runtime.intersection_regions[0]['line_id'] = LINE['id']
        runtime.test_scan = dict(stamp=100., age_s=.01, xyz=np.empty((0, 3)),
            obstacle_points=np.empty((0, 2)))
        live_fixture.LivePathSafetyTests().attach(runtime)
        self.assertIsNone(runtime.stop_line_controller.active)
        row = fixture.step(runtime, 0)
        self.assertEqual(runtime.gap_planner.junction['line_id'], LINE['id'])
        self.assertTrue(runtime.gap_planner.junction['held'])
        self.assertGreater(row['published_cap_kmh'], 0.)
        self.assertFalse(row['emergency_stop'])
        self.assertGreater(live_fixture.LivePathSafetyTests().payload(runtime)['target_speed_mps'], 0.)

    def test_full_runtime_clear_gap_does_not_stop_or_commit_before_crossing(self):
        fixture, runtime = legacy_fixture.EmptyRuntimeTests().runtime()
        runtime.stop_line_controller = competition_stop_controller([LINE])
        runtime.test_status['ego'].update(x=50.-EGO_FRONT_M-5., speed_kmh=12.)
        row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'APPROACH_YIELD')
        self.assertTrue(row['stop_line']['gap_checked'])
        self.assertFalse(row['emergency_stop'] or runtime.gap_planner.junction_committed)
        self.assertGreater(row['published_cap_kmh'], 2.5)


class YieldForecastTests(unittest.TestCase):
    def publish(self, independent=8., *, gap_checked=True, emergency=False, raw=False,
                traffic=False, gap=5., return_runtime=False, lead_kind='moving', semantic=False):
        fixture = live_fixture.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        runtime.gap_planner, ego = planner(gap, 0.)
        runtime.test_status['ego'].update(ego)
        runtime.lanes = SimpleNamespace(route_progress=lambda e:e['x'])
        runtime.stop_line_controller = competition_stop_controller([LINE])
        runtime.stop_line_controller.update(ego['x'], 0., 100., True,
            observation_age_s=.01, empty_front_station=50.-gap)
        cars = ([car(ego['x']+10., 0., 5.5, 0., 1),
                 car(ego['x']+10., 44., 0., -5.5, 2)] if traffic else [])
        if cars and lead_kind == 'overlap':
            cars[0] = car(ego['x']+2., 0., 5.5, 0., 1)
        elif cars and lead_kind in ('standing', 'unknown', 'predicted'):
            cars[0] = car(ego['x']+10., 0., 0. if lead_kind=='standing' else 5.5,
                0., 1, valid=lead_kind!='unknown')
            if lead_kind == 'predicted':
                cars[0] = tuple(cars[0])+(.1,)
        from virtual_sim import SimulatedVehicleReceiver
        runtime.vehicle_receiver = SimulatedVehicleReceiver()
        runtime.test_scan.update(stamp=100., age_s=.01,
            obstacle_points=np.array([[3., 0.]]) if raw else np.empty((0, 2)))
        runtime.object_receiver.observation_stamp = 100.
        if semantic:
            runtime.object_receiver.objects = [car(ego['x']+10., 0., 5.5, 0., 12),
                (ego['x']+3., 0., 2., 'fusion_obstacle', 'obstacle', 0., 2., 1., 0., 0., 0., 13)]
        runtime.path_publisher.zone_mps = 2.78
        runtime.path_publisher.emergency_stop = emergency
        guard = runtime.path_publisher._safety_provider
        guard.owner = 'ROAD'
        requested = []
        check = guard.guard.check
        def capture(*args, **kwargs):
            requested.append(kwargs['requested_mps'])
            return check(*args, **kwargs)
        guard.guard.check = capture
        with patch('time.monotonic', return_value=100.):
            runtime.vehicle_receiver.publish_rows(cars, stamp=100.)
            runtime.path_publisher.publish(ego, PATH, PATH, dict(mode='ROUTE BASE', reason='route',
                junction_body_cap_mps=independent, stop_line=dict(gap_checked=gap_checked)))
        result = (requested, fixture.payload(runtime))
        return (*result, runtime) if return_runtime else result

    def test_moving_body_limits_current_command_without_becoming_a_static_future_ceiling(self):
        _, payload = self.publish(traffic=True)
        validation = payload['decision']['validation']
        live = validation['live_safety']
        self.assertGreater(validation['junction_nontraffic_clear_m'], 30.)
        self.assertLess(live['current_body_clear_m'], 4.)
        self.assertLessEqual(payload['target_speed_mps'], live['current_body_cap_mps'])
        self.assertEqual(live['junction_traffic']['ring']['action'], 'GO')

    def test_current_npc_overlap_cannot_consume_an_entry_despite_the_future_split(self):
        _, payload, runtime = self.publish(traffic=True, gap=.95,
            lead_kind='overlap', return_runtime=True)
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertTrue(payload['decision']['emergency'])
        self.assertFalse(runtime.stop_line_controller.released or runtime.gap_planner.junction_committed)

    def test_unmeasured_standing_or_predicted_body_remains_in_the_future_guard(self):
        for kind in ('standing', 'unknown', 'predicted'):
            _, payload = self.publish(traffic=True, lead_kind=kind)
            with self.subTest(kind=kind):
                self.assertLess(payload['decision']['validation']['junction_nontraffic_clear_m'], 4.)

    def test_a_paired_semantic_npc_does_not_remove_an_unmatched_obstacle(self):
        _, payload = self.publish(traffic=True, semantic=True)
        self.assertLess(payload['decision']['validation']['junction_nontraffic_clear_m'], 1.)
        self.assertEqual(payload['target_speed_mps'], 0.)

    def test_timer_can_continue_a_launch_but_cannot_reuse_an_interrupted_proof(self):
        requested, payload, runtime = self.publish(gap=.95, return_runtime=True)
        self.assertGreater(payload['target_speed_mps'], 0.)
        self.assertTrue(runtime.stop_line_controller.yield_launch_pending)
        with patch('time.monotonic', return_value=100.):
            runtime.path_publisher.recheck_active_path()
            self.assertGreater(live_fixture.LivePathSafetyTests().payload(runtime)['target_speed_mps'], 0.)
            runtime.test_scan['obstacle_points'] = np.array([[3., 0.]])
            runtime.path_publisher.recheck_active_path()
            self.assertEqual(live_fixture.LivePathSafetyTests().payload(runtime)['target_speed_mps'], 0.)
            self.assertFalse(runtime.stop_line_controller.released or runtime.gap_planner.junction_committed)
            runtime.test_scan['obstacle_points'] = np.empty((0, 2))
            runtime.path_publisher.recheck_active_path()
            self.assertEqual(live_fixture.LivePathSafetyTests().payload(runtime)['target_speed_mps'], 0.)
            self.assertFalse(runtime.gap_planner.junction_committed)

    def test_direct_stop_and_command_gap_discard_uncrossed_launch_authority(self):
        for kind in ('direct', 'gap', 'reverse'):
            _, _, runtime = self.publish(gap=.95, return_runtime=True)
            with self.subTest(kind=kind), patch('time.monotonic', return_value=100.):
                if kind == 'direct':
                    runtime.path_publisher.publish_stop('geometry stale')
                elif kind == 'reverse':
                    runtime.path_publisher.reverse_state = None
                    runtime.path_publisher.publish_reverse(runtime.test_status['ego'], stop=True)
                else:
                    runtime.path_publisher._safety_provider.expire_yield_launch(100.601)
                self.assertFalse(runtime.stop_line_controller.released or runtime.gap_planner.junction_committed)
                # The command-gap expiry and recheck use the same clock.
                # Returning from 100.601 to 100 would make the old detector
                # packet fresh again and legitimately permit a new full check.
                with patch('time.monotonic', return_value=100.601 if kind == 'gap' else 100.):
                    runtime.path_publisher.recheck_active_path()
                self.assertEqual(live_fixture.LivePathSafetyTests().payload(runtime)['target_speed_mps'], 0.)

    def test_forecast_uses_independent_body_cap_but_emits_current_follow_cap(self):
        requested, payload = self.publish()
        self.assertEqual(requested, [8.])
        self.assertGreater(payload['target_speed_mps'], 0.)
        self.assertLessEqual(payload['target_speed_mps'], 2.78)
        self.assertEqual(payload['decision']['validation']['junction_forecast_body_cap_mps'], 8.)

    def test_real_body_cap_bounds_forecast(self):
        requested, payload = self.publish(3.)
        self.assertEqual(requested, [3.])
        self.assertLessEqual(payload['target_speed_mps'], 2.78)

    def test_passing_leader_and_incoming_car_use_future_headway_once(self):
        actions = []
        for body_cap in (8., 2.78, 3.):
            requested, payload = self.publish(body_cap, traffic=True)
            ring = payload['decision']['validation']['live_safety']['junction_traffic']['ring']
            actions.append(ring['action'])
            self.assertIn(1, ring['ahead'])
            self.assertEqual(requested, [body_cap])
            self.assertLessEqual(payload['target_speed_mps'], 2.78)
        self.assertEqual(actions, ['GO', 'WAIT', 'WAIT'])

    def test_unchecked_invalid_or_emergency_metadata_cannot_raise_forecast_cap(self):
        for independent, checked, emergency in ((None, True, False), (math.nan, True, False),
                (-1., True, False), (8., False, False), (8., True, True)):
            with self.subTest(independent=independent, checked=checked, emergency=emergency):
                requested, payload = self.publish(independent,
                    gap_checked=checked, emergency=emergency)
                self.assertEqual(requested, [2.78])
                self.assertLessEqual(payload['target_speed_mps'], 2.78)
                if emergency:
                    self.assertEqual(payload['target_speed_mps'], 0.)

    def test_raw_body_overlap_still_vetoes_independent_forecast(self):
        requested, payload = self.publish(raw=True)
        self.assertEqual(requested, [8.])
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertTrue(payload['decision']['emergency'])


if __name__ == '__main__':
    unittest.main()
