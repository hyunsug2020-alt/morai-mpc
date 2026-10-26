"""Measured empty-roundabout rolling entry and its independent publication guard."""
import copy
import json
import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from ego_body import EGO_FRONT_M
from driving_plan import DrivingPlan
from live_path_safety import LivePathSafety
from path_safety import SafetyResult
from stop_line import (EMPTY_ENTRY_REASON, EMPTY_ENTRY_ROLL_MPS, StopLineController,
                       competition_stop_controller as make_competition_controller, empty_intersection_observation,
                       intersection_front_station)
from traffic_planner import GapPlanner
import test_intersection_runtime as runtime_fixture
import test_live_path_safety as live_fixture
from test_junction_entry import car
from roundabout_empty import (CLEARANCE_M, ROLL_TARGET_MPS, approach_cap, braking_reserve,
                             certified_geometry, raw_entry_clearance, entry_scan_points, semantic_entry_clearance,
                             fresh_entry_pose, entry_proof_fresh, MAX_ENTRY_SOURCE_AGE_S, ROLL_MPS)
from roundabout_empty import entry_publication_can_commit


LINE = dict(id='entry', node_id='A1256W000709', station_m=50., xy=[50., 0.])
PATH = np.column_stack((np.arange(0., 40.01, .5), np.zeros(81)))


def competition_stop_controller(lines):
    """Exercise the optional legacy stop policy independently of competition yield."""
    return make_competition_controller(lines, require_full_stop=True)


def observation(**changes):
    return dict(detector_ready=True, age_s=.01, stamp=100., car_count=0,
                vehicles=[], ghost_ids=[], inherited_ids=[], unknown_ids=[],
                active_region=dict(node_id='A1256W000709'), **changes)


class EmptyObservationTests(unittest.TestCase):
    def test_only_fresh_complete_empty_observation_permits_the_exception(self):
        self.assertTrue(empty_intersection_observation(observation()))
        mutations = [('detector_ready', False), ('detector_ready', 'true'),
                     ('age_s', .351), ('age_s', -.1), ('age_s', math.nan),
                     ('stamp', 0.), ('stamp', math.inf), ('car_count', 1),
                     ('car_count', False), ('active_region', dict(node_id='other'))]
        mutations += [(key, [7]) for key in ('vehicles', 'ghost_ids', 'inherited_ids', 'unknown_ids')]
        for key, value in mutations:
            with self.subTest(key=key, value=value):
                status = observation()
                status[key] = value
                self.assertFalse(empty_intersection_observation(status))
        for key in observation():
            status = observation()
            del status[key]
            self.assertFalse(empty_intersection_observation(status))

    def test_front_projection_respects_heading_and_does_not_mutate_pose(self):
        ego = dict(x=10., y=20., heading_deg=90.)
        self.assertAlmostEqual(intersection_front_station(ego, lambda p: p['y']), 20.+EGO_FRONT_M)
        self.assertEqual(ego, dict(x=10., y=20., heading_deg=90.))
        self.assertTrue(math.isnan(intersection_front_station(dict(ego, x=math.nan), lambda p: p['y'])))


class EntryScanTests(unittest.TestCase):
    def fixture(self):
        ego = dict(x=2., y=0., heading_deg=0., stamp=100., age_s=.01)
        vehicle = dict(stamp=100., age_s=.01, vehicles=[])
        scan = dict(stamp=99.99, age_s=.02, obstacle_points=np.array([[3., 0.]]))
        runtime = SimpleNamespace(vehicle_receiver=SimpleNamespace(snapshot=lambda: ([], True, vehicle)),
            lidar=SimpleNamespace(scan_snapshot=lambda: scan),
            ego_receiver=SimpleNamespace(at_stamp=lambda stamp: dict(ego, x=1.)))
        return runtime, ego, vehicle, scan

    def test_atomic_scan_is_aligned_and_both_source_ages_are_bounded(self):
        runtime, ego, vehicle, scan = self.fixture()
        np.testing.assert_allclose(entry_scan_points(runtime, ego, observation(), 100.), [[2., 0.]])
        for source, changes in ((vehicle, dict(age_s=.251)), (vehicle, dict(stamp=99.7)),
                                (scan, dict(age_s=.251)), (scan, dict(stamp=99.7))):
            old = dict(source)
            source.update(changes)
            self.assertIsNone(entry_scan_points(runtime, ego, observation(), 100.))
            source.clear()
            source.update(old)
        runtime.ego_receiver.at_stamp = lambda stamp: None
        self.assertIsNone(entry_scan_points(runtime, ego, observation(), 100.))

    def test_inspected_sources_must_still_be_fresh_after_checking(self):
        runtime, ego, vehicle, scan = self.fixture()
        vehicle['age_s'] = .24
        proof = {}
        self.assertIsNotNone(entry_scan_points(runtime, ego, observation(), 100., proof=proof))
        self.assertTrue(entry_proof_fresh(proof, 100.009))
        self.assertFalse(entry_proof_fresh(proof, 100.011))
        self.assertFalse(entry_proof_fresh({}, 100.))

    def test_source_budget_does_not_extend_the_pose_deadline(self):
        runtime, ego, vehicle, scan = self.fixture()
        for age, accepted in ((.199, True), (.201, False)):
            ego['age_s'] = age
            scan['stamp'] = 100.
            proof = {}
            raw = entry_scan_points(runtime, ego, observation(), 100., proof=proof)
            self.assertEqual(raw is not None, accepted)
            if accepted:
                self.assertTrue(entry_proof_fresh(proof, 100.0005))
                self.assertFalse(entry_proof_fresh(proof, 100.0015))

    def test_detector_pipeline_delay_is_bounded_separately_from_current_pose(self):
        runtime, ego, vehicle, scan = self.fixture()
        for source in (vehicle, scan):
            for age, accepted in ((.249, True), (.251, False)):
                source['age_s'] = age
                self.assertEqual(entry_scan_points(runtime, ego, observation(), 100.) is not None,
                                 accepted)
            source['age_s'] = .01


class EntryPoseTests(unittest.TestCase):
    def fixture(self):
        planned = dict(x=2., y=0., heading_deg=0., speed_kmh=3., stamp=99.8)
        pose = dict(planned, x=2.1, stamp=100., received=99.99, age_s=.01)
        runtime = SimpleNamespace(ego_receiver=SimpleNamespace(snapshot=lambda count: (pose, [], 30.)))
        return runtime, planned, pose

    def test_current_pose_is_copied_and_plan_is_unchanged(self):
        runtime, planned, pose = self.fixture()
        before = dict(planned)
        with patch('roundabout_empty.time.monotonic', return_value=100.):
            current = fresh_entry_pose(runtime, planned)
        self.assertEqual(current['stamp'], 100.)
        self.assertEqual(planned, before)
        self.assertIsNot(current, pose)

    def test_missing_stale_invalid_rewound_or_discontinuous_pose_is_rejected(self):
        mutations = ({'age_s': .201}, {'received': 99.799}, {'received': 100.001},
                     {'stamp': 99.7}, {'stamp': 100.151}, {'x': 20.},
                     {'heading_deg': math.nan}, {'speed_kmh': math.inf}, {'age_s': -.01})
        for changes in mutations:
            runtime, planned, pose = self.fixture()
            pose.update(changes)
            with self.subTest(changes=changes), patch('roundabout_empty.time.monotonic', return_value=100.):
                self.assertIsNone(fresh_entry_pose(runtime, planned))
        for key in ('x', 'y', 'heading_deg', 'speed_kmh', 'stamp', 'received', 'age_s'):
            runtime, planned, pose = self.fixture()
            del pose[key]
            with self.subTest(missing=key), patch('roundabout_empty.time.monotonic', return_value=100.):
                self.assertIsNone(fresh_entry_pose(runtime, planned))

    def test_source_window_boundary_and_signed_speed(self):
        runtime, planned, pose = self.fixture()
        planned['stamp'], planned['speed_kmh'] = 99.65, -3.
        pose['speed_kmh'] = -2.
        with patch('roundabout_empty.time.monotonic', return_value=100.):
            self.assertIsNotNone(fresh_entry_pose(runtime, planned))
            planned['stamp'] -= .000001
            self.assertIsNone(fresh_entry_pose(runtime, planned))

    def test_fresh_pose_resolves_latest_scan_after_old_plan_without_widening_lag(self):
        runtime, planned, pose = self.fixture()
        planned.update(stamp=99.851867, age_s=.148133)
        pose.update(stamp=100.044767, received=99.99)
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: ([], True,
            dict(stamp=100., age_s=.01, vehicles=[])))
        runtime.lidar = SimpleNamespace(scan_snapshot=lambda: dict(stamp=100., age_s=.01,
            obstacle_points=np.empty((0, 2))))
        runtime.ego_receiver.at_stamp = lambda stamp: pose
        self.assertIsNone(entry_scan_points(runtime, planned, observation(), 100.))
        with patch('roundabout_empty.time.monotonic', return_value=100.):
            current = fresh_entry_pose(runtime, planned)
        self.assertIsNotNone(current)
        self.assertIsNotNone(entry_scan_points(runtime, current, observation(), 100.))


class EmptyStopGateTests(unittest.TestCase):
    def step(self, gate, front_gap, speed=.7, clear=True, now=0., age=.1, empty=True):
        station = LINE['station_m']-front_gap-EGO_FRONT_M
        return gate.update(station, speed, now, clear, observation_age_s=age,
                           empty_entry_clear=empty, empty_front_station=station+EGO_FRONT_M,
                           empty_entry_prepare=True,
                           empty_entry_clearance_m=CLEARANCE_M)

    def test_circulating_traffic_approach_does_not_crawl_at_empty_entry_speed(self):
        gate = competition_stop_controller([LINE])
        station = LINE['station_m']-EGO_FRONT_M-4.5
        result = gate.update(station, 2., 0., False, observation_age_s=.1,
                             empty_front_station=station+EGO_FRONT_M)
        room = 4.5-2.*.1-.5*gate.ACCEL_MPS2*.1**2-gate.BUFFER_M-2.*gate.REACTION_S
        self.assertEqual(result['state'], 'APPROACH')
        self.assertAlmostEqual(result['cap_mps'], math.sqrt(2.*gate.BRAKE_MPS2*room))
        self.assertGreater(result['cap_mps'], 3.*ROLL_TARGET_MPS)

    def test_delayed_traffic_approach_stops_before_physical_line_without_long_crawl(self):
        # The changed normal approach still stops its actual front with a
        # 150 ms pose delay and 500 ms actuator delay at every tested speed.
        for kmh in (12., 20., 40.):
            gate = competition_stop_controller([LINE])
            dt, x, speed = 1./150., -35., kmh/3.6
            history, commands = [], []
            target, next_tick, minimum_gap, five_at = speed, 0., math.inf, None
            for step in range(15000):
                now = step*dt
                history.append((x, speed))
                gap = LINE['station_m']-x-EGO_FRONT_M
                if gap <= 5. and five_at is None:
                    five_at = now
                if now >= next_tick:
                    ox, ov = history[max(0, step-23)]
                    result = gate.update(ox, ov, now, False, observation_age_s=.1534,
                                         empty_front_station=ox+EGO_FRONT_M)
                    cap = min(kmh/3.6, result['cap_mps']) if result else kmh/3.6
                    commands.append((now+.5, cap))
                    next_tick = now+1./15.
                while commands and commands[0][0] <= now:
                    target = commands.pop(0)[1]
                previous = speed
                speed = min(target, speed+1.2*dt) if target > speed else max(target, speed-2.5*dt)
                x += .5*(previous+speed)*dt
                minimum_gap = min(minimum_gap, gap)
                if gate.at_line and speed <= .05:
                    break
            with self.subTest(kmh=kmh):
                self.assertLess(step, 14999)
                self.assertGreater(minimum_gap, 0.)
                self.assertLess(now-five_at, 3.)
                self.assertFalse(gate.released)

    def test_reappearing_empty_proof_does_not_restart_a_latched_stop(self):
        gate = competition_stop_controller([LINE])
        self.step(gate, 1.8)
        self.assertTrue(self.step(gate, 1.7, now=.1, empty=False, clear=False)['stop'])
        for index in range(8):
            result = self.step(gate, 1.6, speed=.3, now=.2+index*.1, empty=index%2 == 0)
            self.assertEqual(result['cap_mps'], 0.)
            self.assertEqual(result['state'], 'HOLD')
            self.assertTrue(gate.stop_latched)
            self.assertFalse(gate.empty_approach)
        waiting = self.step(gate, 1.6, speed=0., now=1.1, empty=False, clear=False)
        self.assertEqual(waiting['state'], 'WAIT_CLEAR')
        released = self.step(gate, 1.6, speed=0., now=1.2)
        self.assertEqual(released['state'], 'ENTER')
        self.assertGreater(released['cap_mps'], 10.*ROLL_TARGET_MPS)
        self.assertFalse(gate.empty_committed)

    def test_late_empty_observation_keeps_normal_braking_until_one_stop(self):
        gate = competition_stop_controller([LINE])
        station = LINE['station_m']-EGO_FRONT_M-6.
        gate.update(station, 12./3.6, 0., False, observation_age_s=.1,
                    empty_front_station=station+EGO_FRONT_M)
        self.assertFalse(gate.rolling_entry_available(3.8, 12./3.6))
        result = self.step(gate, 3.8, speed=12./3.6, now=.1)
        room = 3.8-(12./3.6)*.1-.5*gate.ACCEL_MPS2*.1**2-gate.BUFFER_M-(12./3.6)*gate.REACTION_S
        self.assertEqual(result['state'], 'APPROACH')
        self.assertAlmostEqual(result['cap_mps'], math.sqrt(2.*gate.BRAKE_MPS2*room))
        self.assertTrue(gate.empty_rolling_deferred)
        self.assertFalse(gate.rolling_entry_available(2.5, .6))
        self.assertNotEqual(self.step(gate, 2.5, speed=.6, now=.2)['state'], 'APPROACH_EMPTY')
        released = self.step(gate, 1., speed=0., now=.3)
        self.assertEqual(released['state'], 'ENTER')
        self.assertFalse(gate.empty_committed)
        # Reset/lap re-arms the continuous empty approach independently.
        self.step(gate, 40., speed=.6, now=.4)
        self.assertFalse(gate.empty_rolling_deferred)
        self.assertEqual(self.step(gate, 4., speed=.6, now=.5)['state'], 'APPROACH_EMPTY')

    def test_first_empty_observation_just_outside_window_cannot_force_a_speed_jump(self):
        for gap, kmh in ((5.01, 12.), (5.01, 20.), (6., 20.)):
            with self.subTest(gap=gap, kmh=kmh):
                gate = competition_stop_controller([LINE])
                self.assertFalse(gate.rolling_entry_available(gap, kmh/3.6))
                result = self.step(gate, gap, speed=kmh/3.6)
                self.assertNotEqual(result['state'], 'APPROACH_EMPTY')
                self.assertTrue(gate.empty_rolling_deferred)
                room = max(0., gap-kmh/3.6*.1-.5*gate.ACCEL_MPS2*.1**2
                           -gate.BUFFER_M-kmh/3.6*gate.REACTION_S)
                self.assertAlmostEqual(result['cap_mps'], math.sqrt(2.*gate.BRAKE_MPS2*room))

    def test_moving_empty_approach_remains_uncommitted_until_front_crosses(self):
        gate = competition_stop_controller([LINE])
        result = self.step(gate, .3)
        self.assertEqual(result['state'], 'APPROACH_EMPTY')
        self.assertGreater(result['cap_mps'], 0.)
        self.assertFalse(result['stop'] or gate.released or gate.hold_latched)
        crossed = self.step(gate, -.01, now=.1)
        self.assertEqual(crossed['state'], 'ENTER')
        self.assertTrue(gate.released)

    def test_checked_crossing_cannot_be_undone_by_a_waiting_older_update(self):
        gate = competition_stop_controller([LINE])
        station = LINE['station_m']-EGO_FRONT_M-.004
        self.step(gate, .004, speed=2.3/3.6)
        started, results = threading.Event(), []
        def older_update():
            started.set()
            results.append(gate.update(station, 2.3/3.6, .1, False,
                                      empty_front_station=station+EGO_FRONT_M))
        with gate._state_lock:
            worker = threading.Thread(target=older_update)
            worker.start()
            self.assertTrue(started.wait(1.))
            self.assertTrue(gate.consume_empty_crossing(LINE['id'], 50., 50.023, 2.3/3.6, .04,
                                                       expires_at=time.monotonic()+.1))
        worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(results[0]['state'], 'ENTER')
        self.assertTrue(gate.released and gate.empty_committed)
        # A teleport/new lap must still clear the consumed permission.
        reset = gate.update(0., 0., 1., False, empty_front_station=EGO_FRONT_M)
        self.assertNotEqual(reset['state'], 'ENTER')
        self.assertFalse(gate.empty_committed)

    def test_consumption_rejects_other_lines_stops_and_unusable_crossings(self):
        for fields in (('other', 50., 50.023, .6, .04), ('entry', 51., 51.023, .6, .04),
                       ('entry', 50., 49.99, .6, .04), ('entry', 50., 50.023, .73, .04),
                       ('entry', 50., 50.023, .6, .201), ('entry', 50., math.nan, .6, .04)):
            gate = competition_stop_controller([LINE])
            self.step(gate, .004, speed=.6)
            with self.subTest(fields=fields):
                self.assertFalse(gate.consume_empty_crossing(*fields, expires_at=time.monotonic()+.1))
                self.assertFalse(gate.released)
        gate = competition_stop_controller([LINE])
        self.step(gate, .004, speed=.6, clear=False)
        self.assertFalse(gate.consume_empty_crossing('entry', 50., 50.023, .6, .04,
                                                    expires_at=time.monotonic()+.1))

    def test_waiting_for_state_lock_cannot_renew_an_expired_crossing_proof(self):
        gate = competition_stop_controller([LINE])
        self.step(gate, .004, speed=.6)
        clock, started, results = [100.], threading.Event(), []
        def consume():
            started.set()
            results.append(gate.consume_empty_crossing('entry', 50., 50.023, .6, .04, expires_at=100.16))
        with patch('stop_line.time.monotonic', side_effect=lambda: clock[0]):
            with gate._state_lock:
                worker = threading.Thread(target=consume)
                worker.start()
                self.assertTrue(started.wait(1.))
                clock[0] = 100.161
            worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(results, [False])
        self.assertFalse(gate.released or gate.empty_committed)

    def test_faster_measured_vehicle_retains_physical_braking_envelope(self):
        gate = competition_stop_controller([LINE])
        result = self.step(gate, 10., speed=5.)
        room = 10.-(5.*.1+.5*gate.ACCEL_MPS2*.1**2)-gate.BUFFER_M-5.*gate.REACTION_S
        self.assertLessEqual(result['cap_mps'], math.sqrt(2.*gate.BRAKE_MPS2*room))
        blocked = self.step(gate, .5, speed=5., now=.1)
        self.assertTrue(blocked['stop'])
        self.assertFalse(gate.released)

    def test_preline_conflict_or_stale_pose_restores_stop(self):
        for change in ({'clear': False}, {'empty': False}, {'age': .51}):
            with self.subTest(change=change):
                gate = competition_stop_controller([LINE])
                self.step(gate, .2)
                result = self.step(gate, .1, now=.1, **change)
                self.assertTrue(result['stop'])
                self.assertFalse(gate.released)

    def test_default_controller_and_missing_front_proof_do_not_roll(self):
        self.assertTrue(self.step(StopLineController([LINE], hold_s=0.), -.5)['stop'])
        gate = competition_stop_controller([LINE])
        result = gate.update(50.-gate.FRONT_M-1., 1., 0., True, empty_entry_clear=True)
        self.assertTrue(result['stop'])

    def test_fallback_brakes_using_physical_front_after_empty_proof_is_lost(self):
        gate = competition_stop_controller([LINE])
        self.step(gate, 6., speed=1., now=-.1)
        self.step(gate, 4., speed=1.)
        fallback = self.step(gate, 3.8, speed=1., now=.1, empty=False, clear=False)
        room = 3.8-(1.*.1+.5*gate.ACCEL_MPS2*.1**2)-gate.BUFFER_M-1.*gate.REACTION_S
        self.assertEqual(fallback['state'], 'APPROACH')
        self.assertAlmostEqual(fallback['distance_m'], 3.8)
        self.assertAlmostEqual(fallback['cap_mps'], min(ROLL_TARGET_MPS,
                              math.sqrt(2.*gate.BRAKE_MPS2*room)))
        # The old bumper convention added 1.5275 m and allowed a new launch
        # after provisional clearance had already required a physical stop.
        self.step(gate, 1.8, speed=.7, now=.2)
        self.step(gate, 1.7, speed=.7, now=.3, clear=False, empty=False)
        self.assertTrue(gate.stop_latched)
        for gap in (1.8, 2.8):
            result = self.step(gate, gap, speed=.4, now=.3, clear=False, empty=False)
            self.assertEqual(result['cap_mps'], 0.)
        reset = self.step(gate, 3.1, speed=.4, now=.4, clear=False, empty=False)
        self.assertEqual(reset['state'], 'APPROACH')
        self.assertFalse(gate.stop_latched)

    def test_physical_capture_requires_standstill_and_a_current_gap(self):
        gate = competition_stop_controller([LINE])
        self.step(gate, 1.8)
        lost = self.step(gate, 1.7, now=.1, empty=False, clear=False)
        self.assertEqual(lost['state'], 'HOLD')
        still_moving = self.step(gate, -.1, speed=.3, now=.2, empty=False)
        self.assertTrue(still_moving['stop'])
        self.assertFalse(gate.released)
        stale = self.step(gate, -.1, speed=0., now=.3, empty=False, age=.51)
        self.assertTrue(stale['stop'])
        waiting = self.step(gate, -.1, speed=0., now=.4, empty=False, clear=False)
        self.assertEqual(waiting['state'], 'WAIT_CLEAR')
        entered = self.step(gate, -.1, speed=0., now=.5, empty=False)
        self.assertEqual(entered['state'], 'ENTER')

    def test_missing_or_nonfinite_physical_front_is_fail_closed(self):
        for front in (None, math.nan, math.inf, -math.inf, 'invalid'):
            gate = competition_stop_controller([LINE])
            with self.subTest(front=front):
                result = gate.update(40., 0., 0., True, empty_front_station=front)
                self.assertEqual(result['reason'], 'stop line physical front invalid')
                self.assertEqual(result['cap_mps'], 0.)
                self.assertFalse(gate.released)

    def test_candidate_and_released_cleanup_use_consistent_body_boundaries(self):
        gate = competition_stop_controller([LINE])
        # Physical front has passed the line: the legacy rear+2.3175 candidate
        # must not arm a new line that is already more than .5 m behind us.
        station = 50.-EGO_FRONT_M+.6
        self.assertIsNone(gate.update(station, 0., 0., True,
                                    empty_front_station=station+EGO_FRONT_M))
        gate = competition_stop_controller([LINE])
        self.step(gate, -.1)
        self.assertTrue(gate.empty_committed)
        for station, state in ((59., 'ENTER'), (62.1, None)):
            result = gate.update(station, .7, .1, False,
                                 empty_front_station=station+EGO_FRONT_M)
            self.assertEqual(result['state'] if result else None, state)

    def test_default_controller_keeps_legacy_front_and_full_stop_contract(self):
        station = 50.-StopLineController.FRONT_M-4.
        for front in (None, math.nan, station+EGO_FRONT_M):
            gate = StopLineController([LINE])
            result = gate.update(station, 1., 0., False, empty_front_station=front)
            self.assertAlmostEqual(result['distance_m'], 4.)
            self.assertEqual(result['state'], 'APPROACH')

    def test_invalid_or_faster_time_scale_disables_rolling(self):
        for scale in (math.nan, 0., -1., 1.01):
            gate = competition_stop_controller([LINE])
            station = 50.-EGO_FRONT_M-.1
            result = gate.update(station, .7, 0., True, time_scale=scale,
                empty_entry_clear=True, empty_front_station=station+EGO_FRONT_M,
                empty_entry_clearance_m=CLEARANCE_M)
            self.assertNotIn(result['state'], ('APPROACH_EMPTY', 'ENTER'))
        self.assertFalse(gate.empty_committed)

    def test_near_line_speed_ceiling_prepares_roll_without_overriding_braking(self):
        speed, age = 1.1, .15
        self.assertAlmostEqual(approach_cap(4.5, speed, age, CLEARANCE_M), ROLL_TARGET_MPS)
        self.assertEqual(approach_cap(.5, speed, age, CLEARANCE_M), 0.)
        self.assertEqual(approach_cap(.5, .7, age, 0.), 0.)
        self.assertGreater(approach_cap(5.01, speed, age, CLEARANCE_M), ROLL_TARGET_MPS)

    def test_rolling_preparation_slows_before_the_check_window_within_braking_envelope(self):
        speed, age, gap = 12./3.6, .03, 10.
        cap = approach_cap(gap, speed, age, CLEARANCE_M)
        self.assertGreater(cap, ROLL_TARGET_MPS)
        self.assertLess(cap, speed)
        room = gap-speed*age-.5*StopLineController.ACCEL_MPS2*age*age \
            -StopLineController.BUFFER_M-speed*StopLineController.REACTION_S
        self.assertLessEqual(cap, math.sqrt(2.*StopLineController.BRAKE_MPS2*room))
        self.assertEqual(approach_cap(.5, speed, age, CLEARANCE_M), 0.)

    def test_delayed_empty_approach_does_not_need_standstill(self):
        # 150 ms pose delay, 500 ms actuator delay; the collision-free entry is
        # continuously observed. This tests the closed-loop speed transition,
        # rather than assigning an old stopped trace to the new controller.
        for kmh in (12., 20., 40.):
            gate = competition_stop_controller([LINE])
            dt, x, speed = 1./150., -35., kmh/3.6
            history, commands = [], []
            target, next_tick, minimum, released = speed, 0., speed, False
            previous_cap, largest_near_line_drop = speed, 0.
            for step in range(10000):
                now = step*dt
                history.append((x, speed))
                if now >= next_tick:
                    observed_x, observed_speed = history[max(0, step-23)]
                    front_gap = LINE['station_m']-observed_x-EGO_FRONT_M
                    # The actual GUI first offers an empty-entry certificate
                    # inside five metres. Earlier preparation must also work
                    # through the ordinary, unapproved approach branch.
                    certified = front_gap <= 5.
                    result = gate.update(observed_x, observed_speed, now, True,
                        observation_age_s=.1534, empty_entry_clear=certified,
                        empty_entry_prepare=True,
                        empty_front_station=observed_x+EGO_FRONT_M,
                        empty_entry_clearance_m=CLEARANCE_M if certified else 0.)
                    if result and result['state'] == 'ENTER':
                        released = True
                        break
                    cap = result['cap_mps'] if result else kmh/3.6
                    if observed_x+EGO_FRONT_M > LINE['station_m']-15.:
                        largest_near_line_drop = max(largest_near_line_drop, previous_cap-cap)
                    previous_cap = cap
                    commands.append((now+.5, cap))
                    next_tick = now+1./15.
                while commands and commands[0][0] <= now:
                    target = commands.pop(0)[1]
                previous = speed
                speed = min(target, speed+1.2*dt) if target > speed else max(target, speed-2.5*dt)
                x += .5*(speed+previous)*dt
                minimum = min(minimum, speed)
            with self.subTest(kmh=kmh):
                self.assertTrue(released)
                self.assertGreater(minimum, .15)
                self.assertLessEqual(observed_speed, EMPTY_ENTRY_ROLL_MPS)
                # Native type-1 control saturated its brake on a 9.1 km/h
                # target drop even though every planner command was positive.
                self.assertLess(largest_near_line_drop*3.6, 1.5)

    def test_preparation_does_not_release_without_empty_entry_permission(self):
        gate = competition_stop_controller([LINE])
        station = LINE['station_m']-EGO_FRONT_M-10.
        result = gate.update(station, 12./3.6, 0., False,
                             empty_entry_prepare=True,
                             empty_front_station=station+EGO_FRONT_M)
        self.assertEqual(result['state'], 'APPROACH')
        self.assertGreater(result['cap_mps'], ROLL_TARGET_MPS)
        self.assertLess(result['cap_mps'], 12./3.6)
        station = LINE['station_m']-EGO_FRONT_M-.4
        for now in (1., 2., 3.):
            result = gate.update(station, 0., now, False,
                                 empty_front_station=station+EGO_FRONT_M)
            self.assertEqual(result['cap_mps'], 0.)
            self.assertNotEqual(result['state'], 'ENTER')
        self.assertFalse(gate.empty_committed)


class EmptyPlannerTests(unittest.TestCase):
    def planner(self):
        p = GapPlanner()
        p.junction = dict(held=True, ready=True, line_gap_m=2., line_station_m=50.,
                          allow_empty_entry=True, empty_observation=True)
        return p

    def test_empty_proposal_is_checked_without_committing(self):
        p = self.planner()
        result = p.evaluate([], dict(x=0., y=0., heading_deg=0., speed_kmh=5.), PATH, 8., 0.)
        self.assertEqual(result['reason'], EMPTY_ENTRY_REASON)
        self.assertEqual(p.status['state'], 'APPROACH_EMPTY')
        self.assertFalse(p.junction_committed)

    def test_latched_line_uses_the_first_normal_gap_instead_of_a_new_roll(self):
        p = self.planner()
        p.junction['at_line'] = True
        result = p.evaluate([], dict(x=0., y=0., heading_deg=0., speed_kmh=0.), PATH, 8., 0.)
        self.assertEqual(p.status['state'], 'ENTER')
        self.assertEqual(p.status['ring']['action'], 'GO')
        self.assertNotEqual(result['reason'], EMPTY_ENTRY_REASON)
        self.assertGreater(result['v_mps'], 10.*ROLL_TARGET_MPS)
        self.assertFalse(p.junction_committed)

    def test_any_object_or_readiness_loss_removes_empty_exception(self):
        for objects, field, value, fresh in (([car(10., 10., 0., -3., 1)], None, None, True),
                ([(8., 3., .6, 'person', 'pedestrian', 0.)], None, None, True),
                ([], 'ready', False, True), ([], 'empty_observation', False, True),
                ([], None, None, False)):
            with self.subTest(objects=objects, field=field, fresh=fresh):
                p = self.planner()
                if field:
                    p.junction[field] = value
                result = p.evaluate(objects, dict(x=0., y=0., heading_deg=0., speed_kmh=5.),
                                    PATH, 8., 0., fresh=fresh)
                self.assertNotEqual(result['reason'], EMPTY_ENTRY_REASON)
                self.assertFalse(p.junction_committed)


class EmptyRuntimeTests(unittest.TestCase):
    def setUp(self):
        # Synthetic straight route fixtures isolate the controller/sensor gates.
        pose = patch('gui.certified_pose_geometry', return_value=True)
        pose.start()
        self.addCleanup(pose.stop)

    def runtime(self):
        fixture = runtime_fixture.IntersectionRuntimeTests()
        runtime = fixture.runtime()
        runtime.stop_line_controller = competition_stop_controller([LINE])
        runtime.intersection_regions[0]['node_id'] = 'A1256W000709'
        runtime.test_status['ego'].update(x=50.-EGO_FRONT_M-.2,
                                         speed_kmh=EMPTY_ENTRY_ROLL_MPS*3.6)
        runtime.test_status['ego'].update(stamp=100., age_s=.01)
        runtime.ego_receiver = SimpleNamespace(at_stamp=lambda stamp: dict(runtime.test_status['ego']),
            snapshot=lambda count: (dict(runtime.test_status['ego'],
                received=time.monotonic()-.01), [], 30.))
        runtime.lidar.scan_snapshot = lambda: dict(stamp=100., age_s=.01,
            obstacle_points=runtime.lidar.pts.copy())
        runtime.vehicle_receiver.snapshot = lambda: (runtime.test_cars, runtime.test_lidar_ready,
            dict(ready=runtime.test_lidar_ready, age_s=.01, stamp=100., vehicles=[]))
        return fixture, runtime

    def test_full_infer_publishes_moving_clear_entry_without_commit(self):
        fixture, runtime = self.runtime()
        with patch('gui.certified_path_geometry', return_value=True), patch('gui.raw_entry_clearance', return_value=60.):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'APPROACH_EMPTY')
        self.assertFalse(row['emergency_stop'])
        self.assertGreater(row['published_cap_kmh'], 0.)
        self.assertFalse(runtime.gap_planner.junction_committed)
        runtime.test_status['ego']['x'] += .3
        with patch('gui.certified_path_geometry', return_value=True), patch('gui.raw_entry_clearance', return_value=60.):
            row = fixture.step(runtime, 1)
        self.assertEqual(row['stop_line']['state'], 'ENTER')
        self.assertTrue(runtime.gap_planner.junction_committed)

    def test_stopped_entry_releases_without_restarting_the_rolling_certificate(self):
        fixture, runtime = self.runtime()
        ego = runtime.test_status['ego']
        ego['speed_kmh'] = 0.
        gate = runtime.stop_line_controller
        gate.update(ego['x'], 0., 99.9, False, observation_age_s=.1,
                    empty_front_station=ego['x']+EGO_FRONT_M)
        self.assertTrue(gate.at_line)
        # The rolling geometry is unavailable, while ordinary sensors and the
        # measured gap are fresh. A completed stop needs no new crawl proof.
        with patch('gui.certified_path_geometry', return_value=False):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'ENTER')
        self.assertFalse(row['emergency_stop'])
        self.assertGreater(row['published_cap_kmh'], 2.5)
        self.assertTrue(runtime.gap_planner.junction_committed)
        self.assertFalse(gate.empty_committed)

    def test_late_empty_runtime_observation_retains_normal_braking_target(self):
        fixture, runtime = self.runtime()
        runtime.test_status['ego'].update(x=50.-EGO_FRONT_M-3.8, speed_kmh=12.)
        with patch('gui.certified_path_geometry', return_value=True):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'APPROACH')
        self.assertEqual(row['traffic_plan']['state'], 'APPROACH')
        self.assertGreater(row['published_cap_kmh'], 2.5)
        self.assertTrue(runtime.stop_line_controller.empty_rolling_deferred)
        self.assertFalse(runtime.gap_planner.junction_committed)

    def test_inflight_gap_wait_is_rechecked_after_a_publication_commits(self):
        fixture, runtime = self.runtime()
        gate, planner = runtime.stop_line_controller, runtime.gap_planner
        gate.active, gate.empty_approach = LINE, True
        evaluate, apply = planner.evaluate, runtime.apply_stop_line
        def traffic(*args, **kwargs):
            if not planner.junction_committed:
                return dict(kind='stop', name='gap', source='traffic', along=0., t=0., v_mps=0.,
                            reason='junction: wait at the line for a ring gap')
            return evaluate(*args, **kwargs)
        def publish_crossing_before_application(*args, **kwargs):
            self.assertTrue(kwargs['traffic_stop_only'])
            self.assertTrue(gate.consume_empty_crossing('entry', 50., 50.023, .6, .04,
                                                       expires_at=time.monotonic()+.1))
            planner.latch_junction_commit()
            return apply(*args, **kwargs)
        with patch.object(planner, 'evaluate', side_effect=traffic), \
                patch.object(runtime, 'apply_stop_line', side_effect=publish_crossing_before_application), \
                patch('gui.certified_path_geometry', return_value=True), \
                patch('gui.raw_entry_clearance', return_value=60.):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'ENTER')
        self.assertFalse(row['emergency_stop'])
        self.assertGreater(row['published_cap_kmh'], 0.)

    def test_committed_entry_keeps_nontraffic_emergency_and_actual_following_stop(self):
        for other_emergency, following_cap in ((True, 8.), (False, 0.)):
            fixture, runtime = self.runtime()
            gate = runtime.stop_line_controller
            gate.active, gate.released, gate.empty_committed = LINE, True, True
            runtime._intersection_vehicle_status = observation()
            runtime.gap_planner.junction = dict(held=False)
            runtime.gap_planner.latch_junction_commit()
            runtime.yield_cap_mps = None
            runtime.zone_kmh = 30.
            yielding = dict(kind='stop', source='traffic', v_mps=0., reason='old gap wait')
            with self.subTest(other_emergency=other_emergency, following_cap=following_cap):
                emergency, decision = runtime.apply_stop_line(runtime.test_status['ego'], 100.,
                    yielding, True, True, traffic_stop_only=not other_emergency,
                    committed_traffic_check=lambda: dict(kind='follow', source='traffic', v_mps=following_cap,
                        reason='junction committed: follow the car ahead'))
                self.assertEqual(decision['state'], 'ENTER')
                if other_emergency:
                    self.assertTrue(emergency)
                else:
                    self.assertEqual(runtime.yield_cap_mps, 0.)

    def test_inflight_entry_does_not_erase_a_retained_nontraffic_hold(self):
        fixture, runtime = self.runtime()
        gate, planner = runtime.stop_line_controller, runtime.gap_planner
        gate.active, gate.empty_approach = LINE, True
        runtime._yield_stop_until = math.inf
        apply = runtime.apply_stop_line
        def committed_during_application(*args, **kwargs):
            self.assertFalse(kwargs['traffic_stop_only'])
            self.assertTrue(gate.consume_empty_crossing('entry', 50., 50.023, .6, .04,
                                                       expires_at=time.monotonic()+.1))
            planner.latch_junction_commit()
            return apply(*args, **kwargs)
        waiting = dict(kind='stop', name='gap', source='traffic', along=0., t=0., v_mps=0.,
                       reason='junction: wait at the line for a ring gap')
        with patch.object(planner, 'evaluate', return_value=waiting), \
                patch.object(runtime, 'apply_stop_line', side_effect=committed_during_application), \
                patch('gui.certified_path_geometry', return_value=True), \
                patch('gui.raw_entry_clearance', return_value=60.):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'ENTER')
        self.assertTrue(row['emergency_stop'])
        self.assertEqual(row['published_cap_kmh'], 0.)

    def test_far_approach_keeps_the_existing_controller_without_empty_proof(self):
        fixture, runtime = self.runtime()
        runtime.test_status['ego']['x'] -= 15.
        with patch('gui.certified_path_geometry', return_value=True) as geometry, \
                patch('gui.raw_entry_clearance', return_value=60.) as raw:
            row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'APPROACH')
        self.assertGreater(row['published_cap_kmh'], 0.)
        self.assertFalse(row['emergency_stop'])
        self.assertFalse(runtime.gap_planner.junction_committed)
        geometry.assert_not_called()
        raw.assert_not_called()

    def test_unready_or_unknown_vehicle_cannot_use_empty_entry(self):
        for ready, cars in ((False, []), (True, [car(52., 12., 0., 0., 1, valid=False)])):
            fixture, runtime = self.runtime()
            runtime.test_lidar_ready, runtime.test_cars = ready, cars
            row = fixture.step(runtime, 0)
            self.assertTrue(row['emergency_stop'])
            self.assertFalse(runtime.gap_planner.junction_committed)

    def test_failed_raw_roll_proof_uses_the_existing_stop_and_gap_release(self):
        fixture, runtime = self.runtime()
        with patch('gui.certified_path_geometry', return_value=True), patch('gui.raw_entry_clearance', return_value=60.):
            fixture.step(runtime, 0)
        with patch('gui.certified_path_geometry', return_value=True), patch('gui.raw_entry_clearance', return_value=0.):
            moving = fixture.step(runtime, 1)
            self.assertNotEqual(moving['stop_line']['state'], 'APPROACH_EMPTY')
            runtime.test_status['ego']['speed_kmh'] = 0.
            stopped = fixture.step(runtime, 2)
        self.assertEqual(stopped['stop_line']['state'], 'ENTER')
        self.assertTrue(runtime.gap_planner.junction_committed)

    def test_stale_raw_or_vehicle_sample_cannot_commit_a_rolling_entry(self):
        for stream in ('raw', 'vehicle'):
            fixture, runtime = self.runtime()
            runtime.test_status['ego']['x'] += .3
            if stream == 'raw':
                runtime.lidar.scan_snapshot = lambda: dict(stamp=100., age_s=.251,
                    obstacle_points=np.empty((0, 2)))
            else:
                runtime.vehicle_receiver.snapshot = lambda: ([], True,
                    dict(ready=True, age_s=.251, stamp=100., vehicles=[]))
            with patch('gui.certified_path_geometry', return_value=True), patch('gui.raw_entry_clearance', return_value=60.):
                row = fixture.step(runtime, 0)
            self.assertNotEqual(row['stop_line']['state'], 'ENTER')
            self.assertFalse(runtime.gap_planner.junction_committed)

    def test_commit_rechecks_proposal_sources_instead_of_latching_early(self):
        fixture, runtime = self.runtime()
        runtime.test_status['ego']['x'] += .3
        def first_scan(*args, proof=None):
            proof.update(checked_at=100., expires_at=100.2)
            return np.empty((0, 2))
        calls = iter((first_scan, lambda *args, **kwargs: None))
        with patch('gui.certified_path_geometry', return_value=True), \
                patch('gui.raw_entry_clearance', return_value=60.), \
                patch('gui.entry_scan_points', side_effect=lambda *args, **kwargs: next(calls)(*args, **kwargs)) as scans:
            row = fixture.step(runtime, 0)
        self.assertEqual(scans.call_count, 2)
        self.assertNotEqual(row['stop_line']['state'], 'ENTER')
        self.assertFalse(runtime.gap_planner.junction_committed)

    def test_old_plan_uses_current_pose_for_front_speed_age_without_moving_world_path(self):
        fixture, runtime = self.runtime()
        planned = runtime.test_status['ego']
        planned.update(stamp=99.76, received=99.76, age_s=.24, speed_kmh=7.)
        latest = dict(planned, x=planned['x']+.05, stamp=100., received=99.99,
                      age_s=.01, speed_kmh=2.)
        runtime.ego_receiver.snapshot = lambda count: (latest, [], 30.)
        before = dict(planned)
        with patch('gui.certified_path_geometry', return_value=True), patch('gui.raw_entry_clearance', return_value=60.):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'APPROACH_EMPTY')
        self.assertAlmostEqual(row['stop_line']['empty_front_distance_m'], .15)
        self.assertAlmostEqual(row['stop_line']['observation_age_s'], .01)
        self.assertFalse(row['emergency_stop'])
        self.assertAlmostEqual(row['published_cap_kmh'], 2.5)
        self.assertEqual(planned, before)
        self.assertAlmostEqual(runtime._empty_entry_world_path[0, 0], before['x'])

    def test_normal_stop_line_application_does_not_brake_for_an_old_planning_pose(self):
        _, runtime = self.runtime()
        planned = dict(runtime.test_status['ego'], stamp=99.688, received=99.688, age_s=.312)
        latest = dict(planned, x=planned['x']+.05, stamp=100., received=99.99, age_s=.01,
                      speed_kmh=2.)
        runtime.ego_receiver.snapshot = lambda count: (latest, [], 30.)
        runtime._intersection_vehicle_status = observation()
        with patch('gui.time.monotonic', return_value=100.), \
                patch.object(runtime.stop_line_controller, 'update', return_value=None) as update:
            runtime.apply_stop_line(planned, 100., None, False, True)
        self.assertAlmostEqual(update.call_args.args[0], latest['x'])
        self.assertAlmostEqual(update.call_args.args[1], 2./3.6)
        self.assertAlmostEqual(update.call_args.kwargs['observation_age_s'], .01)
        self.assertEqual(planned['stamp'], 99.688)

    def test_application_continues_from_same_decisions_measured_entry_pose(self):
        _, runtime = self.runtime()
        planned = dict(runtime.test_status['ego'], stamp=99.5, received=99.5, age_s=.5)
        proposal = dict(planned, stamp=99.85, received=99.85, age_s=.15)
        latest = dict(proposal, x=proposal['x']+.05, stamp=100., received=99.99,
                      age_s=.01, speed_kmh=2.)
        runtime.ego_receiver.snapshot = lambda count: (latest, [], 30.)
        runtime._intersection_vehicle_status = observation()
        runtime.gap_planner.junction = dict(line_station_m=50., empty_entry_clearance_m=CLEARANCE_M)
        runtime._empty_entry_world_path = PATH.copy()
        runtime.yield_cap_mps = None
        with patch('gui.time.monotonic', return_value=100.), \
                patch('gui.certified_path_geometry', return_value=True), \
                patch('gui.raw_entry_clearance', return_value=60.):
            self.assertIsNone(fresh_entry_pose(runtime, planned))
            emergency, result = runtime.apply_stop_line(planned, 100.,
                dict(kind='enter', source='traffic', reason=EMPTY_ENTRY_REASON, v_mps=8.),
                False, True, entry_pose=proposal)
        self.assertEqual(result['state'], 'APPROACH_EMPTY')
        self.assertAlmostEqual(result['observation_age_s'], .01)
        self.assertFalse(emergency)
        self.assertEqual(planned['stamp'], 99.5)
        np.testing.assert_array_equal(runtime._empty_entry_world_path, PATH)

    def test_late_intersection_refresh_uses_new_cars_and_keeps_people(self):
        _, runtime = self.runtime()
        planned = dict(runtime.test_status['ego'], stamp=99.688, received=99.688, age_s=.312)
        latest = dict(planned, x=planned['x']+.05, stamp=100., received=99.99, age_s=.01)
        runtime.ego_receiver.snapshot = lambda count: (latest, [], 30.)
        runtime._intersection_vehicle_status = dict(observation(), stamp=99.68)
        current = car(55., 2., 0., 0., 52)
        runtime.test_cars = [current]
        stale = tuple(list(car(50., 2., 0., 0., 51))[:3]+['lidar_car']+list(car(50., 2., 0., 0., 51))[4:])
        person = (47., 1., .5, 'person', 'pedestrian', 0., .5, .5, 0., 0., 0., 99)
        with patch('gui.time.monotonic', return_value=100.):
            refreshed = runtime.refresh_intersection_objects([stale, person], planned)
        self.assertIn(person, refreshed)
        self.assertTrue(any(o[3] == 'lidar_car' and o[11] == 52 for o in refreshed))
        self.assertFalse(any(o[3] == 'lidar_car' and o[11] == 51 for o in refreshed))
        self.assertEqual(runtime._intersection_vehicle_status['stamp'], 100.)

    def test_application_reads_current_pose_again_before_crossing_commit(self):
        fixture, runtime = self.runtime()
        planned = runtime.test_status['ego']
        planned.update(stamp=99.76, received=99.76, age_s=.24)
        proposal = dict(planned, x=planned['x']+.13, stamp=100., received=99.99, age_s=.01, speed_kmh=2.)
        application = dict(proposal, x=planned['x']+.25, stamp=100.05)
        with patch.object(runtime.ego_receiver, 'snapshot', side_effect=[(proposal, [], 30.),
                (proposal, [], 30.), (application, [], 30.)]), patch('gui.certified_path_geometry', return_value=True), \
                patch('gui.raw_entry_clearance', return_value=60.):
            row = fixture.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'ENTER')
        self.assertEqual(row['stop_line']['reason'], 'empty junction entry checked')
        self.assertAlmostEqual(row['stop_line']['empty_front_distance_m'], -.05)
        self.assertTrue(runtime.gap_planner.junction_committed)

    def test_invalid_application_pose_or_changed_path_cannot_commit(self):
        for invalid in ('stale', 'rewind', 'missing', 'geometry'):
            fixture, runtime = self.runtime()
            planned = runtime.test_status['ego']
            planned['x'] += .3
            proposal = dict(planned, received=99.99)
            application = dict(proposal)
            if invalid == 'stale':
                application['age_s'] = .201
            elif invalid == 'rewind':
                application['stamp'] = 99.9
            elif invalid == 'missing':
                application = None
            geometry = [True, False] if invalid == 'geometry' else [True, True]
            with self.subTest(invalid=invalid), \
                    patch.object(runtime.ego_receiver, 'snapshot', side_effect=[(proposal, [], 30.),
                        (proposal, [], 30.), (application, [], 30.)]), \
                    patch('gui.certified_path_geometry', side_effect=geometry), \
                    patch('gui.raw_entry_clearance', return_value=60.):
                row = fixture.step(runtime, 0)
            self.assertNotEqual(row['stop_line']['state'], 'ENTER')
            self.assertFalse(runtime.gap_planner.junction_committed)

    def test_failed_application_proof_cannot_release_a_latched_standstill(self):
        for failure in ('raw', 'geometry', 'new_car', 'ghost', 'pose'):
            fixture, runtime = self.runtime()
            runtime.test_status['ego']['speed_kmh'] = 0.
            gate = runtime.stop_line_controller
            gate.active = LINE
            gate.hold_latched = gate.stop_latched = gate.empty_stop_latched = True
            runtime._intersection_vehicle_status = observation()
            runtime.gap_planner.junction = dict(line_station_m=50., empty_entry_clearance_m=CLEARANCE_M)
            runtime._empty_entry_world_path = PATH
            runtime.yield_cap_mps = None
            if failure == 'new_car':
                runtime.vehicle_receiver.snapshot = lambda: ([car(50., 0., 0., 0., 1)], True,
                    dict(stamp=100., age_s=.01, vehicles=[{}]))
            elif failure == 'ghost':
                runtime._intersection_vehicle_status['ghost_ids'] = [8]
            elif failure == 'pose':
                runtime.ego_receiver.snapshot = lambda count: (None, [], 0.)
            with self.subTest(failure=failure), patch('gui.time.monotonic', return_value=100.), \
                    patch('gui.certified_path_geometry', return_value=failure != 'geometry'), \
                    patch('gui.raw_entry_clearance', return_value=0. if failure == 'raw' else 60.):
                emergency, result = runtime.apply_stop_line(runtime.test_status['ego'], 100.,
                    dict(kind='enter', source='traffic', reason=EMPTY_ENTRY_REASON, v_mps=8.), False, True)
            self.assertNotEqual(result['state'], 'ENTER')
            self.assertFalse(gate.released or runtime.gap_planner.junction_committed)
            self.assertTrue(emergency)

    def test_application_captures_sources_after_slow_path_proof(self):
        _, runtime = self.runtime()
        runtime.zone_kmh = 30.
        planned = dict(runtime.test_status['ego'], speed_kmh=0.)
        planned['x'] += .3
        runtime._intersection_vehicle_status = observation()
        runtime.gap_planner.junction = dict(line_station_m=50., empty_entry_clearance_m=CLEARANCE_M)
        runtime._empty_entry_world_path = PATH.copy()
        runtime.yield_cap_mps = None
        clock, calls = [100.], []
        def slow_path(lanes, world_path):
            calls.append('path')
            clock[0] += .15
            return True
        def current_pose(count):
            calls.append('pose')
            return dict(planned, stamp=100.15, received=100.14, age_s=.01), [], 30.
        runtime.ego_receiver.snapshot = current_pose
        runtime.lidar.scan_snapshot = lambda: dict(stamp=100.15, age_s=.01,
            obstacle_points=np.empty((0, 2)))
        runtime.vehicle_receiver.snapshot = lambda: ([], True,
            dict(stamp=100.15, age_s=.01, vehicles=[]))
        with patch('gui.time.monotonic', side_effect=lambda: clock[0]), \
                patch('gui.certified_path_geometry', side_effect=slow_path), \
                patch('gui.raw_entry_clearance', return_value=60.):
            emergency, result = runtime.apply_stop_line(planned, 100.,
                dict(kind='enter', source='traffic', reason=EMPTY_ENTRY_REASON, v_mps=8.), False, True)
        self.assertEqual(calls, ['path', 'pose'])
        self.assertEqual(result['state'], 'ENTER')
        self.assertEqual(result['reason'], 'empty junction entry checked')
        self.assertFalse(emergency)
        np.testing.assert_array_equal(runtime._empty_entry_world_path, PATH)

    def test_application_rejects_sources_that_expire_during_raw_proof(self):
        fixture, runtime = self.runtime()
        runtime.test_status['ego']['speed_kmh'] = 0.
        gate = runtime.stop_line_controller
        gate.active = LINE
        gate.hold_latched = gate.stop_latched = gate.empty_stop_latched = True
        runtime._intersection_vehicle_status = observation()
        runtime.gap_planner.junction = dict(line_station_m=50., empty_entry_clearance_m=CLEARANCE_M)
        runtime._empty_entry_world_path = PATH
        runtime.yield_cap_mps = None
        runtime.lidar.scan_snapshot = lambda: dict(stamp=100., age_s=.24,
            obstacle_points=np.empty((0, 2)))
        runtime.vehicle_receiver.snapshot = lambda: ([], True,
            dict(stamp=100., age_s=.24, vehicles=[]))
        clock = [100.]
        def delayed_raw_proof(*args):
            clock[0] += .05
            return 60.
        with patch('gui.time.monotonic', side_effect=lambda: clock[0]), \
                patch('gui.certified_path_geometry', return_value=True), \
                patch('gui.raw_entry_clearance', side_effect=delayed_raw_proof):
            emergency, result = runtime.apply_stop_line(runtime.test_status['ego'], 100.,
                dict(kind='enter', source='traffic', reason=EMPTY_ENTRY_REASON, v_mps=8.), False, True)
        self.assertNotEqual(result['state'], 'ENTER')
        self.assertFalse(gate.released or runtime.gap_planner.junction_committed)
        self.assertTrue(emergency)

    def test_application_rechecks_expiry_at_the_final_controller_clock(self):
        fixture, runtime = self.runtime()
        runtime.test_status['ego']['speed_kmh'] = 0.
        gate = runtime.stop_line_controller
        gate.active = LINE
        gate.hold_latched = gate.stop_latched = gate.empty_stop_latched = True
        runtime._intersection_vehicle_status = observation()
        runtime.gap_planner.junction = dict(line_station_m=50., empty_entry_clearance_m=CLEARANCE_M)
        runtime._empty_entry_world_path = PATH
        runtime.yield_cap_mps = None
        runtime.lidar.scan_snapshot = lambda: dict(stamp=100., age_s=.24,
            obstacle_points=np.empty((0, 2)))
        runtime.vehicle_receiver.snapshot = lambda: ([], True,
            dict(stamp=100., age_s=.24, vehicles=[]))
        clock = [100.]
        def checked_then_delayed(proof, now):
            fresh = entry_proof_fresh(proof, now)
            clock[0] += .02
            return fresh
        with patch('gui.time.monotonic', side_effect=lambda: clock[0]), \
                patch('gui.certified_path_geometry', return_value=True), \
                patch('gui.raw_entry_clearance', return_value=60.), \
                patch('gui.entry_proof_fresh', side_effect=checked_then_delayed) as checks:
            emergency, result = runtime.apply_stop_line(runtime.test_status['ego'], 100.,
                dict(kind='enter', source='traffic', reason=EMPTY_ENTRY_REASON, v_mps=8.), False, True)
        self.assertEqual(checks.call_count, 2)
        self.assertNotEqual(result['state'], 'ENTER')
        self.assertFalse(gate.released or runtime.gap_planner.junction_committed)
        self.assertTrue(emergency)


class EmptyLiveGuardTests(unittest.TestCase):
    def setUp(self):
        # Direct junction_follow tests still exercise its full-proof branch.
        for name in ('certified_path_geometry', 'certified_pose_geometry'):
            proof = patch('live_path_safety.'+name, return_value=True)
            proof.start()
            self.addCleanup(proof.stop)

    def guard(self):
        p = GapPlanner()
        p.junction = dict(held=True, allow_empty_entry=True, line_station_m=50.,
                          empty_entry_clearance_m=CLEARANCE_M)
        p.status = dict(state='APPROACH_EMPTY')
        runtime = SimpleNamespace(gap_planner=p, lanes=SimpleNamespace(route_progress=lambda e: e['x']))
        guard = LivePathSafety(runtime)
        guard.traffic = copy.copy(p)
        return guard, runtime

    def check(self, cars=(), ready=True, front_gap=.2, result=None,
              speed=EMPTY_ENTRY_ROLL_MPS*3.6, age=.1, geometry=True, raw=8.):
        guard, runtime = self.guard()
        ego = dict(x=50.-EGO_FRONT_M-front_gap, y=0., heading_deg=0., speed_kmh=speed, age_s=age)
        with patch('live_path_safety.certified_geometry', return_value=geometry):
            return guard.junction_follow(result or SafetyResult(5., False, 40., None, {}),
                runtime, list(cars), 1., PATH+np.array([ego['x'], 0.]), ego,
                empty_vehicle_ready=ready, empty_raw_clear_m=raw)

    def test_fresh_empty_publication_remains_positive(self):
        result = self.check()
        self.assertFalse(result.emergency)
        self.assertEqual(result.validation['junction_traffic']['state'], 'APPROACH_EMPTY')

    def test_expired_rolling_proof_brakes_toward_line_without_an_immediate_stop(self):
        result = self.check(ready=False, front_gap=2.49, speed=1.1)
        self.assertFalse(result.emergency)
        self.assertGreater(result.cap_mps, 0.)
        self.assertLessEqual(result.cap_mps, 2.5/3.6)
        self.assertEqual(result.reason, 'empty entry proof unavailable: brake toward line')
        stopped = self.check(ready=False, front_gap=.2, speed=1.1)
        self.assertEqual(stopped.cap_mps, 0.)
        self.assertTrue(stopped.emergency)
        collision = self.check(ready=False, front_gap=2.49, speed=1.1,
            result=SafetyResult(0., True, 0., 'actual raw blocker', {}))
        self.assertEqual(collision.cap_mps, 0.)
        self.assertTrue(collision.emergency)

    def test_observed_box_does_not_disappear_when_entry_state_changes(self):
        for state in ('APPROACH_EMPTY', 'APPROACH', 'HOLD', 'WAIT_CLEAR', 'ENTER'):
            fixture = live_fixture.LivePathSafetyTests()
            runtime = fixture.attach(fixture.runtime())
            runtime.test_status['ego'].update(x=0., y=0., heading_deg=0., speed_kmh=0.,
                stamp=100., age_s=.01, received=99.99)
            runtime.test_scan.update(stamp=100., age_s=.01, obstacle_points=np.empty((0, 2)))
            runtime.object_receiver.objects = [(3., 4., 2., 'fusion_obstacle', 'obstacle',
                0., 2., 1., 0., 0., 0., 40)]
            runtime.object_receiver.observation_stamp = 100.
            runtime.gap_planner.junction = dict(held=state != 'ENTER', allow_empty_entry=True,
                empty_actual_body=True, line_station_m=4.1, empty_entry_clearance_m=CLEARANCE_M)
            runtime.gap_planner.status = dict(state=state)
            runtime.gap_planner.junction_committed = state == 'ENTER'
            runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
            runtime.path_publisher.zone_mps, runtime.path_publisher.emergency_stop = 8., False
            guard = runtime.path_publisher._safety_provider
            guard.traffic_baseline = object()
            with self.subTest(state=state), \
                    patch('live_path_safety.time.monotonic', return_value=100.), \
                    patch('live_path_safety.semantic_entry_clearance', return_value=0.) as semantic, \
                    patch.object(guard, 'junction_follow', side_effect=lambda result, *a, **kw: result):
                plan = guard(PATH, runtime.test_status['ego'], 'ROUTE BASE')
            semantic.assert_called_once()
            self.assertEqual(plan.validation['empty_entry_semantic_clear_m'], 0.)
            self.assertEqual(plan.allowed_mps, 0.)

    def test_reference_map_and_pose_proofs_remain_mandatory_in_fallback(self):
        for state in ('APPROACH_EMPTY', 'APPROACH', 'HOLD', 'WAIT_CLEAR', 'ENTER'):
            for failed in ('path', 'pose', None):
                fixture = live_fixture.LivePathSafetyTests()
                runtime = fixture.attach(fixture.runtime())
                runtime.lanes.route_progress = lambda ego: ego['x']
                runtime.test_status['ego'].update(speed_kmh=0., age_s=.01,
                    stamp=100., received=99.99)
                runtime.gap_planner.junction = dict(held=state != 'ENTER',
                    allow_empty_entry=True, empty_actual_body=True, line_station_m=4.1)
                runtime.gap_planner.status = dict(state=state)
                runtime.gap_planner.junction_committed = state == 'ENTER'
                publisher = runtime.path_publisher
                publisher.zone_mps, publisher.emergency_stop = 8., False
                guard = publisher._safety_provider
                with self.subTest(state=state, failed=failed), \
                        patch('live_path_safety.certified_path_geometry', return_value=failed != 'path') as path, \
                        patch('live_path_safety.certified_pose_geometry', return_value=failed != 'pose') as pose, \
                        patch.object(guard, 'junction_follow', side_effect=lambda result, *a, **kw: result):
                    publisher.publish(runtime.test_status['ego'], runtime.test_route,
                        runtime.test_route, dict(mode='ROUTE BASE', entry_reference=True,
                                                 reason='checked entry reference'))
                    decision = publisher._active_decision_plan
                    committed = state == 'ENTER'
                    invalid = failed is not None and not committed
                    self.assertEqual(decision.validation.get('entry_reference_geometry_valid'),
                                     None if committed else failed is None)
                    self.assertEqual(decision.validation['valid'], not invalid)
                    if invalid:
                        self.assertEqual(decision.allowed_mps, 0.)
                        self.assertTrue(decision.emergency)
                    else:
                        self.assertGreater(decision.allowed_mps, 0.)
                        if state != 'APPROACH_EMPTY':
                            self.assertEqual(decision.allowed_mps, 8.)
                            self.assertNotIn('entry_reference_cap_mps', decision.validation)
                        else:
                            self.assertLessEqual(decision.allowed_mps, 2.5/3.6)
                            self.assertEqual(decision.validation['entry_reference_cap_mps'], 2.5/3.6)
                    self.assertEqual(path.call_count, 0 if committed else 1)
                    if failed != 'path' and not committed:
                        pose.assert_called_once()
                    # Timer revalidation must retain this publication's contract.
                    if failed is None or committed:
                        publisher.recheck_active_path()
                        self.assertEqual(path.call_count, 0 if committed else 2)
                        self.assertLessEqual(publisher._active_decision_plan.allowed_mps,
                                             2.5/3.6 if state == 'APPROACH_EMPTY' else 8.)
                        path.return_value = False
                        publisher.recheck_active_path()
                        self.assertEqual(path.call_count, 0 if committed else 3)
                        if committed:
                            self.assertGreater(fixture.payload(runtime)['target_speed_mps'], 0.)
                            runtime.test_scan['obstacle_points'] = np.array([[3.4, 0.]])
                            publisher.recheck_active_path()
                        self.assertEqual(fixture.payload(runtime)['target_speed_mps'], 0.)

    def test_publication_rejects_sources_that_expire_during_geometry_check(self):
        guard, runtime = self.guard()
        ego = dict(x=50.-EGO_FRONT_M-.2, y=0., heading_deg=0., speed_kmh=3.,
                   age_s=.01, received=99.99)
        clock = [100.]
        def delayed_geometry(*args):
            clock[0] += .05
            return True
        with patch('live_path_safety.time.monotonic', side_effect=lambda: clock[0]), \
                patch('live_path_safety.certified_geometry', side_effect=delayed_geometry):
            result = guard.junction_follow(SafetyResult(5., False, 40., None, {}),
                runtime, [], 1., PATH+np.array([ego['x'], 0.]), ego,
                empty_vehicle_ready=True, empty_raw_clear_m=8., empty_sources_expire_at=100.01)
        self.assertTrue(result.emergency)
        self.assertEqual(result.cap_mps, 0.)

    def test_native_publication_bounds_the_same_vehicle_source_age_and_future_lag(self):
        for vehicle_stamp, receiver_present in ((99.9, True), (100.13, True), (100., False)):
            fixture = live_fixture.LivePathSafetyTests()
            runtime = fixture.attach(fixture.runtime())
            runtime.test_status['ego'].update(x=0., y=0., heading_deg=0., speed_kmh=3.,
                stamp=100., age_s=.1, received=99.9)
            runtime.test_scan.update(stamp=100., age_s=.01, obstacle_points=np.empty((0, 2)))
            runtime.gap_planner.junction = dict(held=True, allow_empty_entry=True, empty_actual_body=True,
                line_station_m=4.1, empty_entry_clearance_m=CLEARANCE_M)
            runtime.gap_planner.status = dict(state='APPROACH_EMPTY')
            runtime.vehicle_receiver = (SimpleNamespace(snapshot=lambda: ([], True,
                dict(stamp=vehicle_stamp, age_s=.01, vehicles=[]))) if receiver_present else None)
            runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
            runtime.path_publisher.zone_mps = 8.
            runtime.path_publisher.emergency_stop = False
            guard = runtime.path_publisher._safety_provider
            guard.traffic_baseline = object()
            with self.subTest(vehicle_stamp=vehicle_stamp, receiver_present=receiver_present), \
                    patch('live_path_safety.time.monotonic', return_value=100.), \
                    patch('live_path_safety.raw_entry_clearance', return_value=60.), \
                    patch.object(guard, 'junction_follow', side_effect=lambda result, *args, **kwargs: result) as follow:
                guard(PATH, runtime.test_status['ego'], 'ROUTE BASE')
            kwargs = follow.call_args.kwargs
            self.assertEqual(kwargs['empty_vehicle_ready'], receiver_present and vehicle_stamp == 99.9)
            if receiver_present and vehicle_stamp == 99.9:
                self.assertAlmostEqual(kwargs['empty_sources_expire_at'], 100.05)

    def test_native_path_proof_precedes_current_sensor_capture_and_uses_fixed_path(self):
        fixture = live_fixture.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        planned = dict(runtime.test_status['ego'], x=0., y=0., heading_deg=0.,
            speed_kmh=EMPTY_ENTRY_ROLL_MPS*3.6, stamp=100., received=99.81, age_s=.19)
        runtime.gap_planner.junction = dict(held=True, allow_empty_entry=True, empty_actual_body=True,
            line_station_m=4.1, empty_entry_clearance_m=CLEARANCE_M)
        runtime.gap_planner.status = dict(state='APPROACH_EMPTY')
        runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
        runtime.path_publisher.zone_mps, runtime.path_publisher.emergency_stop = 8., False
        guard = runtime.path_publisher._safety_provider
        guard.traffic_baseline = object()
        clock, calls, world = [100.], [], PATH.copy()
        def slow_path(lanes, inspected):
            calls.append('path')
            np.testing.assert_array_equal(inspected, PATH)
            clock[0] += .15
            return True
        def fresh_pose(count):
            calls.append('pose')
            return dict(planned, x=.05, stamp=100.15, received=100.14, age_s=.01), [], 30.
        runtime.ego_receiver.snapshot = fresh_pose
        runtime.test_scan.update(stamp=100.15, age_s=.01, obstacle_points=np.empty((0, 2)))
        runtime.object_receiver.observation_stamp = 100.15
        runtime.ego_receiver.at_stamp = lambda stamp: dict(planned, x=.05, stamp=stamp)
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: ([], True,
            dict(stamp=100.15, age_s=.01, vehicles=[])))
        with patch('live_path_safety.time.monotonic', side_effect=lambda: clock[0]), \
                patch('live_path_safety.certified_path_geometry', side_effect=slow_path), \
                patch('live_path_safety.certified_geometry') as full_proof, \
                patch('live_path_safety.raw_entry_clearance', return_value=60.):
            plan = guard(world, planned, 'ROUTE BASE')
        self.assertEqual(calls, ['path', 'pose'])
        full_proof.assert_not_called()
        self.assertFalse(plan.emergency)
        self.assertGreater(plan.allowed_mps, 0.)
        self.assertAlmostEqual(plan.validation_ego['x'], .05)
        np.testing.assert_allclose(plan.path[:, :2]+[.05, 0.], PATH)
        np.testing.assert_array_equal(world, PATH)

    def test_prechecked_path_cannot_bypass_the_current_pose_or_reuse_a_false_proof(self):
        for path_ok, pose_ok in ((False, True), (True, False)):
            guard, runtime = self.guard()
            ego = dict(x=50.-EGO_FRONT_M-.2, y=0., heading_deg=0., speed_kmh=3., age_s=.01)
            with self.subTest(path_ok=path_ok, pose_ok=pose_ok), \
                    patch('live_path_safety.certified_pose_geometry', return_value=pose_ok):
                result = guard.junction_follow(SafetyResult(5., False, 40., None, {}), runtime, [],
                    1., PATH, ego, empty_vehicle_ready=True, empty_raw_clear_m=8.,
                    empty_path_geometry=path_ok)
            self.assertTrue(result.emergency)
            self.assertEqual(result.cap_mps, 0.)

    def test_native_empty_entry_preserves_original_candidates_before_offroad_filter(self):
        for field in ('vehicles', 'cars', 'ghost_ids', 'unknown_ids', 'inherited_ids', 'missing_vehicles'):
            fixture = live_fixture.LivePathSafetyTests()
            runtime = fixture.attach(fixture.runtime())
            runtime.test_status['ego'].update(x=0., y=0., heading_deg=0., speed_kmh=3.,
                stamp=100., age_s=.01)
            runtime.test_scan.update(stamp=100., age_s=.01, obstacle_points=np.empty((0, 2)))
            runtime.gap_planner.junction = dict(held=True, allow_empty_entry=True, empty_actual_body=True,
                line_station_m=4.1, empty_entry_clearance_m=CLEARANCE_M)
            runtime.gap_planner.status = dict(state='APPROACH_EMPTY')
            runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
            runtime.path_publisher.zone_mps, runtime.path_publisher.emergency_stop = 8., False
            status = dict(stamp=100., age_s=.01, vehicles=[])
            cars = []
            if field == 'cars':
                cars = [car(5., 4., 0., 0., 500000, valid=False)]
            elif field == 'missing_vehicles':
                status.pop('vehicles')
            else:
                status[field] = [dict(uid=500000)] if field == 'vehicles' else [500000]
            runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (cars, True, status))
            runtime.on_road_vehicles = lambda rows: []
            guard = runtime.path_publisher._safety_provider
            guard.traffic_baseline = object()
            with self.subTest(field=field), patch('live_path_safety.raw_entry_clearance', return_value=60.):
                plan = guard(PATH, runtime.test_status['ego'], 'ROUTE BASE')
            self.assertTrue(plan.emergency)
            self.assertEqual(plan.allowed_mps, 0.)
            self.assertEqual(plan.validation['live_safety']['junction_traffic']['state'], 'WAIT')

    def test_new_car_ghost_or_stale_detector_stops_before_commit(self):
        for cars, ready in (([car(52., 12., 0., -3., 1)], True),
                            ([car(52., 12., 0., 0., 2, valid=False)], True), ([], False)):
            result = self.check(cars, ready)
            self.assertTrue(result.emergency)
            self.assertEqual(result.cap_mps, 0.)

    def test_pedestrian_or_raw_emergency_is_never_erased(self):
        for reason in ('person in path', 'raw body contact'):
            original = SafetyResult(0., True, 0., reason, {})
            result = self.check(result=original)
            self.assertTrue(result.emergency)
            self.assertEqual(result.reason, reason)

    def test_current_crossed_pose_does_not_reopen_a_preline_clearance_stop(self):
        result = self.check(front_gap=-.01)
        self.assertFalse(result.emergency)
        self.assertEqual(result.validation['junction_traffic']['state'], 'ENTER')

    def test_publication_crossing_commits_both_planners_before_the_next_gui_update(self):
        fixture = live_fixture.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        old = dict(runtime.test_status['ego'], x=50.-EGO_FRONT_M-.004, y=0., heading_deg=0.,
                   speed_kmh=2.3, stamp=100., age_s=.01, received=99.99)
        current = dict(old, x=50.-EGO_FRONT_M+.023, stamp=100.04, received=100.03)
        runtime.test_status['ego'].update(current)
        runtime.test_scan.update(stamp=100.03, age_s=.01, obstacle_points=np.empty((0, 2)))
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: ([], True,
            dict(stamp=100.03, age_s=.01, vehicles=[])))
        runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
        gate = runtime.stop_line_controller = competition_stop_controller([LINE])
        gate.update(old['x'], 2.3/3.6, 100., True, empty_entry_clear=True,
                    empty_front_station=old['x']+EGO_FRONT_M, empty_entry_clearance_m=CLEARANCE_M)
        runtime.gap_planner.junction = dict(held=True, allow_empty_entry=True, empty_actual_body=True,
            line_id=LINE['id'], line_station_m=50., empty_entry_clearance_m=CLEARANCE_M)
        runtime.gap_planner.status = dict(state='APPROACH_EMPTY')
        base = DrivingPlan(1, PATH, old, old, 100., .01, 100., ROLL_TARGET_MPS, 60., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False, entry_reference=True), 'ROUTE BASE')
        guard = runtime.path_publisher._safety_provider
        with patch('live_path_safety.time.monotonic', return_value=100.04), \
                patch('live_path_safety.raw_entry_clearance', return_value=60.):
            result = guard(PATH+[old['x'], 0.], old, 'ROUTE BASE', base)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertTrue(gate.released and gate.empty_committed)
        self.assertTrue(runtime.gap_planner.junction_committed)
        decision = gate.update(current['x'], 2.3/3.6, 100.05, False,
                               empty_front_station=current['x']+EGO_FRONT_M)
        self.assertEqual(decision['state'], 'ENTER')
        self.assertGreater(decision['cap_mps'], ROLL_TARGET_MPS)

    def test_emergency_crossing_never_consumes_permission(self):
        guard, runtime = self.guard()
        gate = runtime.stop_line_controller = competition_stop_controller([LINE])
        gate.active, gate.empty_approach = LINE, True
        runtime.gap_planner.junction['line_id'] = LINE['id']
        ego = dict(x=50.-EGO_FRONT_M+.023, y=0., heading_deg=0., speed_kmh=2.3, age_s=.04)
        result = guard.junction_follow(SafetyResult(0., True, 40., 'person in path', {}),
            runtime, [], 1., PATH+[ego['x'], 0.], ego, empty_vehicle_ready=True,
            empty_raw_clear_m=8., empty_commit_allowed=True)
        self.assertTrue(result.emergency)
        self.assertFalse(gate.released or runtime.gap_planner.junction_committed)

    def test_invalid_stopped_old_or_discontinuous_publication_cannot_commit(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=2.3, stamp=100.)
        plan = DrivingPlan(1, PATH, ego, ego, 100., .01, 10., ROLL_TARGET_MPS, 60., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False), 'ROUTE BASE')
        current = dict(ego, x=.023, stamp=100.04)
        self.assertTrue(entry_publication_can_commit(plan, current, 10.04))
        for flags in ({'valid': False}, {'pose_valid': False}, {'steering_valid': False},
                      {'blocked': True}, {'geometry_path_digest': 'changed'}):
            with self.subTest(flags=flags):
                self.assertFalse(entry_publication_can_commit(plan.decided(**flags), current, 10.04))
        self.assertFalse(entry_publication_can_commit(plan.decided(emergency=True), current, 10.04))
        self.assertFalse(entry_publication_can_commit(plan.decided(allowed_mps=0.), current, 10.04))
        self.assertFalse(entry_publication_can_commit(plan, current, 10.61))
        for change in ({'stamp': 100.351}, {'stamp': 99.99}, {'x': 2.}, {'speed_kmh': math.nan}):
            self.assertFalse(entry_publication_can_commit(plan, dict(current, **change), 10.04))

    def test_front_crossing_cannot_skip_speed_sensor_geometry_or_raw_checks(self):
        for change in ({'ready': False}, {'speed': 3.01}, {'age': .201}, {'geometry': False}, {'raw': None},
                       {'cars': [car(52., 12., 0., 0., 2, valid=False)]},
                       {'result': SafetyResult(1., False, 2., None, {})}):
            with self.subTest(change=change):
                result = self.check(front_gap=-.01, **change)
                self.assertTrue(result.emergency)
                self.assertEqual(result.cap_mps, 0.)

    def test_raw_blocker_inside_delayed_stopping_corridor_removes_roll_floor(self):
        result = self.check(result=SafetyResult(.7, False, 4.4, None, {}))
        self.assertTrue(result.emergency)
        self.assertEqual(result.cap_mps, 0.)

    def test_actual_front_raw_contact_stops_empty_entry_and_preserves_body_after_commit(self):
        fixture = live_fixture.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        runtime.test_status['ego'].update(x=0., y=0., heading_deg=0., speed_kmh=3., age_s=.01)
        runtime.test_scan['obstacle_points'] = np.array([[3.4, 0.]])
        runtime.test_scan['age_s'] = .01
        runtime.gap_planner.junction = dict(held=True, allow_empty_entry=True, empty_actual_body=True,
            line_station_m=4.1, empty_entry_clearance_m=CLEARANCE_M)
        runtime.gap_planner.status = dict(state='APPROACH_EMPTY')
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: ([], True,
            dict(stamp=100., age_s=.01, vehicles=[])))
        runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
        runtime.path_publisher.zone_mps = 8.
        runtime.path_publisher.emergency_stop = False
        guard = runtime.path_publisher._safety_provider
        guard.traffic_baseline = object()
        with patch('live_path_safety.certified_geometry', return_value=True):
            plan = guard(PATH, runtime.test_status['ego'], 'ROUTE BASE')
        self.assertAlmostEqual(guard.guard.front_m, EGO_FRONT_M)
        self.assertTrue(plan.emergency)
        self.assertEqual(plan.allowed_mps, 0.)
        runtime.gap_planner.junction_committed = True
        with patch('live_path_safety.certified_geometry', return_value=True):
            committed = guard(PATH, runtime.test_status['ego'], 'ROUTE BASE')
        self.assertAlmostEqual(guard.guard.front_m, EGO_FRONT_M)
        self.assertEqual(committed.allowed_mps, 0.)

    def test_current_body_contact_is_seen_when_path_tangent_differs(self):
        fixture = live_fixture.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        runtime.test_status['ego'].update(x=0., y=0., heading_deg=0., speed_kmh=3., age_s=.01)
        runtime.test_scan['obstacle_points'] = np.array([[3.7, .9]])
        runtime.test_scan['age_s'] = .01
        runtime.gap_planner.junction = dict(held=True, allow_empty_entry=True, empty_actual_body=True,
            line_station_m=4.1, empty_entry_clearance_m=CLEARANCE_M)
        runtime.gap_planner.status = dict(state='APPROACH_EMPTY')
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: ([], True, dict(stamp=100., age_s=.01, vehicles=[])))
        runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
        runtime.path_publisher.zone_mps = 8.
        runtime.path_publisher.emergency_stop = False
        guard = runtime.path_publisher._safety_provider
        guard.traffic_baseline = object()
        angle = math.radians(-10.)
        path = PATH@np.array([[math.cos(angle), math.sin(angle)], [-math.sin(angle), math.cos(angle)]])
        with patch('live_path_safety.certified_geometry', return_value=True):
            plan = guard(path, runtime.test_status['ego'], 'ROUTE BASE')
        self.assertTrue(plan.emergency)
        self.assertEqual(plan.allowed_mps, 0.)

    def test_final_roll_envelope_cannot_stop_a_distant_empty_approach(self):
        fixture = live_fixture.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        runtime.test_status['ego'].update(x=0., y=0., heading_deg=0., speed_kmh=3., age_s=.01)
        runtime.test_scan.update(obstacle_points=np.empty((0, 2)), age_s=.01)
        runtime.gap_planner.junction = dict(held=True, allow_empty_entry=True, empty_actual_body=True,
            line_station_m=50., empty_entry_clearance_m=CLEARANCE_M)
        runtime.gap_planner.status = dict(state='APPROACH_EMPTY')
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: ([], True, dict(stamp=100., age_s=.01, vehicles=[])))
        runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
        runtime.path_publisher.zone_mps, runtime.path_publisher.emergency_stop = 8., False
        guard = runtime.path_publisher._safety_provider
        guard.traffic_baseline = object()
        with patch('live_path_safety.certified_geometry', return_value=True), \
                patch('live_path_safety.raw_entry_clearance', return_value=0.) as roll_check:
            plan = guard(PATH, runtime.test_status['ego'], 'ROUTE BASE')
        roll_check.assert_not_called()
        self.assertFalse(plan.emergency)
        self.assertGreater(plan.allowed_mps, 0.)

    def test_semantic_presence_uses_body_path_and_stops_an_unseen_on_path_box(self):
        for lateral, length, report_speed, allowed in ((3., 1.1, 0., True), (0., 1.1, 0., False),
                (0., 9., 0., False), (0., math.nan, 0., False), (0., 0., 0., False),
                (0., 1.1, 5., False), (1.35, .3, 0., False)):
            fixture = live_fixture.LivePathSafetyTests()
            runtime = fixture.attach(fixture.runtime())
            runtime.test_status['ego'].update(x=0., y=0., heading_deg=0., speed_kmh=2.5, age_s=.01)
            runtime.test_scan.update(obstacle_points=np.empty((0, 2)), age_s=.01)
            runtime.object_receiver.objects = [(5., lateral, .55, 'fusion_obstacle', 'obstacle',
                report_speed, length, .3 if length == .3 else 1.1, 0., report_speed, 0., 35)]
            runtime.gap_planner.junction = dict(held=True, allow_empty_entry=True, empty_actual_body=True,
                line_station_m=5., empty_entry_clearance_m=CLEARANCE_M)
            runtime.gap_planner.status = dict(state='APPROACH_EMPTY')
            runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: ([], True,
                dict(stamp=100., age_s=.01, vehicles=[])))
            runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
            runtime.path_publisher.zone_mps = ROLL_TARGET_MPS
            runtime.path_publisher.emergency_stop = False
            guard = runtime.path_publisher._safety_provider
            guard.traffic_baseline = object()
            with patch('live_path_safety.certified_geometry', return_value=True), \
                    patch('live_path_safety.raw_entry_clearance', return_value=60.):
                plan = guard(PATH, runtime.test_status['ego'], 'ROUTE BASE')
            if allowed:
                self.assertGreater(plan.allowed_mps, 0.)
                self.assertFalse(plan.emergency)
            else:
                self.assertEqual(plan.allowed_mps, 0.)
                self.assertTrue(plan.emergency)

    def test_invalid_semantic_speed_or_velocity_cannot_disappear_from_both_guards(self):
        for field in (5, 9, 10):
            fixture = live_fixture.LivePathSafetyTests()
            runtime = fixture.attach(fixture.runtime())
            runtime.test_status['ego'].update(x=0., y=0., heading_deg=0., speed_kmh=2.5, age_s=.01)
            runtime.test_scan.update(obstacle_points=np.empty((0, 2)), age_s=.01)
            box = [5., 0., .55, 'fusion_obstacle', 'obstacle', 0., 1.1, 1.1, 0., 0., 0., 35]
            box[field] = math.nan
            runtime.object_receiver.objects = [tuple(box)]
            runtime.gap_planner.junction = dict(held=True, allow_empty_entry=True, empty_actual_body=True,
                line_station_m=5., empty_entry_clearance_m=CLEARANCE_M)
            runtime.gap_planner.status = dict(state='APPROACH_EMPTY')
            runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: ([], True, dict(stamp=100., age_s=.01, vehicles=[])))
            runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
            runtime.path_publisher.zone_mps = ROLL_TARGET_MPS
            runtime.path_publisher.emergency_stop = False
            guard = runtime.path_publisher._safety_provider
            guard.traffic_baseline = object()
            with patch('live_path_safety.certified_geometry', return_value=True), \
                    patch('live_path_safety.raw_entry_clearance', return_value=60.):
                plan = guard(PATH, runtime.test_status['ego'], 'ROUTE BASE')
            self.assertEqual(plan.allowed_mps, 0.)
            self.assertTrue(plan.emergency)
            self.assertTrue(plan.validation['empty_entry_invalid_semantic'])


class SurveyedCertificateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from gui import HdMapLanes
        assets = Path(__file__).parent
        cls.route = np.array([[p['x'], p['y']] for p in
                              json.loads((assets/'global_path.json').read_text())['waypoints']])
        cls.lanes = HdMapLanes(assets/'link_set.json')
        cls.lanes.set_route(cls.route)
        direction = cls.route[1766]-cls.route[1765]
        cls.ego = dict(x=cls.route[1765, 0], y=cls.route[1765, 1],
                       heading_deg=math.degrees(math.atan2(direction[1], direction[0])))
        cls.path = cls.route[1765:1835]

    def test_actual_map_route_and_plain_entry_match_the_certificate(self):
        self.assertTrue(certified_geometry(self.lanes, self.ego, self.path))
        self.assertLess(braking_reserve(EMPTY_ENTRY_ROLL_MPS, MAX_ENTRY_SOURCE_AGE_S), CLEARANCE_M)
        self.assertGreater(braking_reserve(EMPTY_ENTRY_ROLL_MPS, .26), CLEARANCE_M)
        self.assertGreater(braking_reserve(2.7/3.6, MAX_ENTRY_SOURCE_AGE_S), CLEARANCE_M)
        self.assertEqual(EMPTY_ENTRY_ROLL_MPS, ROLL_MPS)

    def test_off_route_path_heading_error_and_short_horizon_are_rejected(self):
        self.assertFalse(certified_geometry(self.lanes, self.ego, self.path+[.4, 0.]))
        self.assertFalse(certified_geometry(self.lanes,
            dict(self.ego, heading_deg=self.ego['heading_deg']+15.), self.path))
        self.assertFalse(certified_geometry(self.lanes, self.ego, self.path[:5]))

    def test_changed_map_or_route_cannot_reuse_the_certificate(self):
        changed = copy.copy(self.lanes)
        changed.links = copy.deepcopy(self.lanes.links)
        changed.links[0]['points'][0][0] += .01
        self.assertFalse(certified_geometry(changed, self.ego, self.path))
        changed = copy.copy(self.lanes)
        changed.route_xy = self.lanes.route_xy.copy()
        changed.route_xy[0, 0] += .01
        self.assertFalse(certified_geometry(changed, self.ego, self.path))

    def test_in_place_map_or_route_mutation_and_body_override_invalidate_certificate(self):
        with patch.dict(self.lanes.links[0], width_start=999.):
            self.assertFalse(certified_geometry(self.lanes, self.ego, self.path))
        old = self.lanes.route_xy[0, 0]
        try:
            self.lanes.route_xy[0, 0] += .01
            self.assertFalse(certified_geometry(self.lanes, self.ego, self.path))
        finally:
            self.lanes.route_xy[0, 0] = old
        with patch('ego_body.EGO_FRONT_M', 4.):
            self.assertFalse(certified_geometry(self.lanes, self.ego, self.path))

    def test_short_original_segment_reversal_or_spike_cannot_hide_between_samples(self):
        point = self.path[10]
        direction = self.path[11]-point
        unit = direction/np.linalg.norm(direction)
        for inserted in ([point+.1*unit, point+.09*unit],
                         [point+.01*unit+.25*np.array([-unit[1], unit[0]])]):
            path = np.concatenate((self.path[:11], inserted, self.path[11:]))
            self.assertFalse(certified_geometry(self.lanes, self.ego, path))

    def test_low_speed_floor_needs_delay_and_clearance_proof(self):
        self.assertEqual(approach_cap(.1, EMPTY_ENTRY_ROLL_MPS, .2, CLEARANCE_M), ROLL_TARGET_MPS)
        self.assertEqual(approach_cap(.1, EMPTY_ENTRY_ROLL_MPS, .201, CLEARANCE_M), 0.)
        self.assertEqual(approach_cap(.1, EMPTY_ENTRY_ROLL_MPS, .1, 0.), 0.)
        self.assertEqual(approach_cap(.1, 2., .1, CLEARANCE_M), 0.)

    def test_raw_sweep_covers_future_front_corner_at_heading_error(self):
        from scipy.spatial import cKDTree
        route = np.column_stack((np.arange(0., 20.01, .5), np.zeros(41)))
        lanes = copy.copy(self.lanes)
        lanes.route_xy, lanes.route_station, lanes.route_tree = route, route[:, 0], cKDTree(route)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=EMPTY_ENTRY_ROLL_MPS*3.6)
        self.assertEqual(raw_entry_clearance(lanes, ego, np.array([[4.4, 1.49]]), route), 0.)
        self.assertEqual(raw_entry_clearance(lanes, ego, np.empty((0, 2)), route),
                         braking_reserve(EMPTY_ENTRY_ROLL_MPS, MAX_ENTRY_SOURCE_AGE_S))
        self.assertEqual(raw_entry_clearance(lanes, dict(ego, speed_kmh=2.7),
                                             np.empty((0, 2)), route), 0.)
        self.assertEqual(raw_entry_clearance(lanes, dict(ego, heading_deg=3.01), np.empty((0, 2)), route), 0.)
        self.assertEqual(raw_entry_clearance(lanes, dict(ego, y=.101), np.empty((0, 2)), route), 0.)
        self.assertEqual(raw_entry_clearance(lanes, ego, np.empty((0, 2))), 0.)
        bad_path = route@np.array([[math.cos(math.radians(8.)), math.sin(math.radians(8.))],
                                  [-math.sin(math.radians(8.)), math.cos(math.radians(8.))]])
        self.assertEqual(raw_entry_clearance(lanes, ego, np.empty((0, 2)), bad_path), 0.)
        boundary_yaw = math.radians(6.01)
        boundary_path = route@np.array([[math.cos(boundary_yaw), math.sin(boundary_yaw)],
                                       [-math.sin(boundary_yaw), math.cos(boundary_yaw)]])
        self.assertEqual(raw_entry_clearance(lanes, dict(ego, heading_deg=6.01),
                                             np.empty((0, 2)), boundary_path), 0.)

    def test_larger_source_budget_checks_the_added_stopping_prefix(self):
        from scipy.spatial import cKDTree
        route = np.column_stack((np.arange(0., 20.01, .5), np.zeros(41)))
        lanes = copy.copy(self.lanes)
        lanes.route_xy, lanes.route_station, lanes.route_tree = route, route[:, 0], cKDTree(route)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=EMPTY_ENTRY_ROLL_MPS*3.6)
        blocker = np.array([[8.95, 0.]])
        with patch('roundabout_empty.MAX_ENTRY_SOURCE_AGE_S', .2):
            self.assertGreater(raw_entry_clearance(lanes, ego, blocker, route), 0.)
        self.assertEqual(raw_entry_clearance(lanes, ego, blocker, route), 0.)

    def test_semantic_start_projection_does_not_create_a_reverse_prefix(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        path = np.column_stack((np.arange(-.2, 20.01, .5), np.zeros(41)))
        box = (-2.5, 0., .15, 'fusion_obstacle', 'obstacle', 0., .3, .3, 0.)
        self.assertGreater(semantic_entry_clearance(path, ego, [box]), 19.)
        contact = (3.7, .9, .15, 'fusion_obstacle', 'obstacle', 0., .3, .3, 0.)
        angled = PATH@np.array([[math.cos(math.radians(-10.)), math.sin(math.radians(-10.))],
                                [-math.sin(math.radians(-10.)), math.cos(math.radians(-10.))]])
        self.assertEqual(semantic_entry_clearance(angled, ego, [contact]), 0.)


if __name__ == '__main__':
    unittest.main()
