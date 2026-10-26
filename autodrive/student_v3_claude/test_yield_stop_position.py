"""Traffic-only stopping position with delayed poses and type-1 pedals."""
import math
from collections import deque
import unittest
from unittest.mock import patch

from ego_body import EGO_FRONT_M
from stop_line import (INTERSECTION_ENTRY_MPS, StopLineController,
                       STOP_CAPTURE_M, YIELD_BUFFER_M, YIELD_WAIT_CAPTURE_M,
                       competition_stop_controller, yield_line_cap)


LINE = dict(id='entry', node_id='A1256W000709', station_m=50., xy=[50., 0.])


def approach(initial_gap, initial_kmh, *, target_delay=.5, pose_delay=.1534):
    """Apply native pedal gains after a delayed target, with fresh speed feedback.

    The target rises at 3 m/s², throttle gain is .35, brake gain .45, and
    deadband .15 m/s, as in path_follower's type-1 command. A zero target
    retains the native .35 holding brake. The plant bounds acceleration at
    3.2 m/s²; brake 1 gives 6 m/s², so the holding brake gives 2.1 m/s².
    """
    gate = competition_stop_controller([LINE])
    dt = 1./150.
    x = LINE['station_m']-EGO_FRONT_M-initial_gap
    speed = initial_kmh/3.6
    maximum = speed if speed > 0. else INTERSECTION_ENTRY_MPS
    history, pending, samples = [], deque(), []
    active_target = previous_target = speed
    next_decision = next_control = 0.
    throttle = brake = 0.
    first_five = stopped_at = None
    for index in range(9000):
        now = index*dt
        history.append((x, speed))
        gap = LINE['station_m']-x-EGO_FRONT_M
        if gap <= 5. and first_five is None:
            first_five = now
        if now+1e-9 >= next_decision:
            source_index = max(0, index-math.floor(pose_delay/dt))
            ox, ov = history[source_index]
            decision = gate.update(ox, ov, now, False,
                observation_age_s=pose_delay, empty_front_station=ox+EGO_FRONT_M)
            target = min(maximum, decision['cap_mps']) if decision else maximum
            pending.append((now+target_delay, target))
            samples.append((now, gap, speed, target))
            next_decision = now+1./15.
        while pending and pending[0][0] <= now+1e-9:
            _, active_target = pending.popleft()
        if now+1e-9 >= next_control:
            target = min(active_target, previous_target+3.*.05)
            previous_target = target
            error = target-speed
            if target <= .15:
                throttle, brake = 0., min(1., max(.35, .45*speed))
            elif error > .15:
                throttle, brake = min(1., .35*error), 0.
            elif error < -.15:
                throttle, brake = 0., min(1., -.45*error)
            else:
                throttle = brake = 0.
            next_control = now+.05
        after = max(0., speed+dt*(min(3.2, 8.*throttle)-6.*brake))
        x += .5*(speed+after)*dt
        speed = after
        if speed <= .001 and gate.yield_wait_latched:
            stopped_at = now
            break
    return dict(gate=gate, samples=samples, gap=LINE['station_m']-x-EGO_FRONT_M,
                speed=speed, stopped_at=stopped_at, first_five=first_five,
                brake=brake, steps=index)


class YieldStopPositionTests(unittest.TestCase):
    def test_recorded_positive_wait_cap_cannot_restart_a_near_stop(self):
        # Native run 56 dipped to .2304 km/h and accelerated to 1.2427
        # while WAIT targets rebounded from .899 to 2.047 km/h.
        gate = competition_stop_controller([LINE])
        front = LINE['station_m']-1.1
        gate.update(front-EGO_FRONT_M, 1., 100., False,
                    observation_age_s=.01, empty_front_station=front)
        for speed, target in ((.230401/3.6, .899/3.6),
                              (1.24269/3.6, 2.047/3.6)):
            cap = gate.apply_yield_wait(LINE['id'], LINE['station_m'],
                front, speed, target, go=False)
            self.assertEqual(cap, 0.)
            self.assertTrue(gate.yield_wait_latched)

    def test_recorded_wait_stop_does_not_restart_on_a_rebounding_cap(self):
        # Native run 54: the same blocking car produced plans 181, 182,
        # 185 and 196. Deceleration lifted the approach envelope again.
        trace = ((1.076588, 3.397008/3.6, .338816772/3.6),
                 (.988582, 2.805328/3.6, 0.),
                 (.910145, 0., 1.855596922/3.6),
                 (.715, .2, .458497034/3.6))
        gate = competition_stop_controller([LINE])
        for i, (gap, speed, requested) in enumerate(trace):
            front = LINE['station_m']-gap
            gate.update(front-EGO_FRONT_M, speed, 100.+i*.1, False,
                        observation_age_s=.1, empty_front_station=front)
            cap = gate.apply_yield_wait(LINE['id'], LINE['station_m'], front,
                                       speed, requested, go=False)
            self.assertEqual(cap, 0.)
            self.assertTrue(gate.yield_wait_latched)
            self.assertFalse(gate.released or gate.hold_latched)

    def test_gui_wait_captures_a_requested_stop_before_actual_standstill(self):
        gate = competition_stop_controller([LINE])
        front = LINE['station_m']-1.
        first = gate.update(front-EGO_FRONT_M, 1., 100., False,
                            observation_age_s=.2, empty_front_station=front)
        self.assertEqual(first['cap_mps'], 0.)
        self.assertTrue(gate.yield_wait_latched)
        # Pose age and measured speed improve, but the ring is still occupied.
        rebound = gate.update(front-EGO_FRONT_M, 0., 100.1, False,
                              observation_age_s=.01, empty_front_station=front)
        self.assertGreater(yield_line_cap(1., 0., .01), .15)
        self.assertEqual(rebound['cap_mps'], 0.)

    def test_wait_stop_releases_on_first_fresh_go_without_dwell(self):
        gate = competition_stop_controller([LINE])
        front = LINE['station_m']-1.
        gate.update(front-EGO_FRONT_M, 1., 100., False,
                    observation_age_s=.2, empty_front_station=front)
        go = gate.update(front-EGO_FRONT_M, 0., 100.01, True,
                         observation_age_s=.01, empty_front_station=front)
        self.assertEqual(go['cap_mps'], INTERSECTION_ENTRY_MPS)
        self.assertEqual(go['required_hold_s'], 0.)
        with patch('stop_line.time.monotonic', return_value=100.01):
            self.assertEqual(gate.apply_yield_wait(LINE['id'], LINE['station_m'],
                front, 0., 3., go=True, expires_at=100.), 0.)
            self.assertTrue(gate.yield_wait_latched)
            self.assertTrue(gate.consume_yield_launch(LINE['id'], LINE['station_m'],
                front, 0., .01, expires_at=100.11))
        self.assertFalse(gate.yield_wait_latched)
        self.assertTrue(gate.released)

    def test_far_traffic_brake_does_not_latch_a_line_stop(self):
        gate = competition_stop_controller([LINE])
        front = LINE['station_m']-8.
        gate.update(front-EGO_FRONT_M, 3., 100., False,
                    observation_age_s=.1, empty_front_station=front)
        self.assertEqual(gate.apply_yield_wait(LINE['id'], LINE['station_m'],
            front, 3., 0., go=False), 0.)
        self.assertFalse(gate.yield_wait_latched)

    def test_restart_inside_consumes_a_new_go_before_releasing(self):
        gate = competition_stop_controller([LINE])
        station = LINE['station_m']+3.84568
        front = LINE['station_m']+7.66044
        result = gate.update(station, 32./3.6, 100., True,
            observation_age_s=.01, empty_front_station=front)
        self.assertTrue(result['gap_checked'])
        self.assertEqual(result['required_hold_s'], 0.)
        self.assertFalse(gate.released)
        with patch('stop_line.time.monotonic', return_value=100.):
            self.assertTrue(gate.consume_yield_launch(LINE['id'], LINE['station_m'],
                front, 32./3.6, .01, expires_at=100.1))
        self.assertTrue(gate.released)
        result = gate.update(station+.1, 32./3.6, 100.05, False,
            observation_age_s=.01, empty_front_station=front+.1)
        self.assertEqual(result['state'], 'ENTER')

    def test_restart_inside_keeps_wait_or_stale_go_closed(self):
        for clear, age in ((False, .01), (True, .201)):
            with self.subTest(clear=clear, age=age):
                gate = competition_stop_controller([LINE])
                station = LINE['station_m']+3.84568
                front = LINE['station_m']+7.66044
                result = gate.update(station, 0., 100., clear,
                    observation_age_s=age, empty_front_station=front)
                self.assertEqual(result['cap_mps'], 0.)
                self.assertFalse(result['gap_checked'])
                with patch('stop_line.time.monotonic', return_value=100.):
                    self.assertFalse(gate.consume_yield_launch(LINE['id'], LINE['station_m'],
                        front, 0., age, expires_at=100.1))
                self.assertFalse(gate.released)

    def test_restart_candidate_ends_at_the_existing_rear_pose_exit(self):
        for offset, active in ((12., True), (12.000001, False)):
            with self.subTest(offset=offset):
                gate = competition_stop_controller([LINE])
                station = LINE['station_m']+offset
                result = gate.update(station, 0., 100., True,
                    observation_age_s=.01, empty_front_station=station+EGO_FRONT_M)
                self.assertEqual(result is not None, active)
                self.assertEqual(gate.active is not None, active)
                self.assertFalse(gate.released)

    def test_restart_does_not_rearm_completed_or_mandatory_lines(self):
        completed = competition_stop_controller([LINE])
        completed.completed.add(LINE['id'])
        mandatory = StopLineController([LINE], hold_s=0., allow_empty_entry=True)
        for gate in (completed, mandatory):
            station = LINE['station_m']+3.84568
            result = gate.update(station, 0., 100., True,
                observation_age_s=.01, empty_front_station=station+EGO_FRONT_M)
            self.assertIsNone(result)
            self.assertFalse(gate.released)

    def test_restart_go_does_not_survive_a_teleport_back_before_the_line(self):
        gate = competition_stop_controller([LINE])
        station = LINE['station_m']+3.84568
        front = LINE['station_m']+7.66044
        gate.update(station, 8., 100., True, observation_age_s=.01,
            empty_front_station=front)
        with patch('stop_line.time.monotonic', return_value=100.):
            self.assertTrue(gate.consume_yield_launch(LINE['id'], LINE['station_m'],
                front, 8., .01, expires_at=100.1))
        result = gate.update(0., 0., 101., False, observation_age_s=.01,
            empty_front_station=EGO_FRONT_M)
        self.assertFalse(gate.released)
        self.assertFalse(result['gap_checked'])

    def test_delayed_normal_approach_finishes_before_the_physical_line(self):
        for kmh in (12., 20., 40.):
            run = approach(60., kmh)
            with self.subTest(kmh=kmh):
                self.assertIsNotNone(run['stopped_at'])
                self.assertGreater(run['gap'], .15)
                self.assertLessEqual(run['gap'], STOP_CAPTURE_M)
                self.assertLess(run['stopped_at']-run['first_five'], 3.)
                self.assertGreaterEqual(run['brake'], .35)
                self.assertFalse(run['gate'].released or run['gate'].hold_latched)
                # The stopping approach does not restart after commanding zero.
                targets = [sample[3] for sample in run['samples']]
                first_zero = next(i for i, target in enumerate(targets) if target <= 0.)
                self.assertTrue(all(target <= 0. for target in targets[first_zero:]))

    def test_a_car_initially_stopped_too_far_completes_one_approach(self):
        run = approach(1.135, 0.)
        self.assertIsNotNone(run['stopped_at'])
        self.assertGreater(run['samples'][0][3], .15)
        self.assertGreater(run['gap'], .15)
        self.assertLessEqual(run['gap'], STOP_CAPTURE_M)
        self.assertLess(run['stopped_at'], 4.)
        targets = [sample[3] for sample in run['samples']]
        first_zero = next(i for i, target in enumerate(targets) if target <= 0.)
        self.assertTrue(all(target <= 0. for target in targets[first_zero:]))
        gate = run['gate']
        for i, age in enumerate((.01, .199, .04, .12)*5):
            front = LINE['station_m']-run['gap']+(i%2-.5)*.04
            result = gate.update(front-EGO_FRONT_M, .14 if i%2 else 0., 20.+i*.1,
                bool(i%2), observation_age_s=age, empty_front_station=front)
            with patch('stop_line.time.monotonic', return_value=20.+i*.1):
                cap = gate.apply_yield_wait(LINE['id'], LINE['station_m'], front,
                    0., result['cap_mps'], go=False)
            self.assertEqual(cap, 0.)
            self.assertTrue(gate.yield_wait_latched)

    def test_an_interrupted_launch_stays_held_even_outside_wait_capture(self):
        gate = competition_stop_controller([LINE])
        front = LINE['station_m']-1.135
        gate.update(front-EGO_FRONT_M, 0., 100., True,
                    observation_age_s=.01, empty_front_station=front)
        with patch('stop_line.time.monotonic', return_value=100.):
            self.assertTrue(gate.consume_yield_launch(LINE['id'], LINE['station_m'],
                front, 0., .01, expires_at=100.1))
        self.assertTrue(gate.update_yield_launch(front, stopped=True))
        result = gate.update(front-EGO_FRONT_M, 0., 100.1, False,
                    observation_age_s=.01, empty_front_station=front)
        self.assertEqual(result['cap_mps'], 0.)
        self.assertTrue(gate.yield_wait_latched)
        self.assertFalse(gate.released or gate.yield_approach)

    def test_a_raw_or_person_zero_cap_is_preserved_before_capture(self):
        gate = competition_stop_controller([LINE])
        front = LINE['station_m']-1.135
        result = gate.update(front-EGO_FRONT_M, 0., 100., False,
                    observation_age_s=.01, empty_front_station=front)
        self.assertGreater(result['cap_mps'], 0.)
        self.assertEqual(gate.apply_yield_wait(LINE['id'], LINE['station_m'],
            front, 0., 0., go=False), 0.)
        self.assertFalse(gate.released)

    def test_cap_reserves_measured_and_requested_response_travel(self):
        for gap, speed, age in ((1.135, 0., .2), (5., 2., .2), (20., 6., .01)):
            cap = yield_line_cap(gap, speed, age)
            room = max(0., gap-speed*age-.5*3.*age*age-YIELD_BUFFER_M)
            self.assertLessEqual(cap*.5+cap*cap/(2.*2.), room+1e-12)
            self.assertLessEqual(cap*cap/(2.*2.), max(0., room-speed*.5)+1e-12)
        for gap, speed, age in ((math.nan, 0., 0.), (2., math.inf, 0.),
                                (2., 0., -.01)):
            self.assertEqual(yield_line_cap(gap, speed, age), 0.)

    def test_the_generic_mandatory_stop_keeps_its_original_buffer(self):
        gate = StopLineController([LINE])
        gap, speed, age = 5., 2., .1
        station = LINE['station_m']-gate.FRONT_M-gap
        result = gate.update(station, speed, 100., False, observation_age_s=age)
        room = gap-speed*age-.5*gate.ACCEL_MPS2*age*age-.8-speed*.5
        self.assertAlmostEqual(result['cap_mps'], math.sqrt(2.*2.*room))
        self.assertEqual(gate.BUFFER_M, .8)
        self.assertFalse(gate.yield_only)


if __name__ == '__main__':
    unittest.main()
