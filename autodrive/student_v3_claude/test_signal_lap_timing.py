"""Lap timing excludes confirmed signal holds while retaining driving faults."""
import time
import unittest

import numpy as np

from driving_plan import DrivingPlan
from lap_finish import LapTracker, signal_waiting
from test_signal_control import controller_at


ROUTE = np.array([[0., 0.], [100., 0.], [100., 100.], [0., 100.]])


class SignalLapTimingTests(unittest.TestCase):
    def sample(self):
        ego = dict(measured=True, age_s=.01, speed_kmh=0.)
        signal = dict(enabled=True, active=True, fresh=True, stop_required=True,
            raw_state='RED', phase='HOLD', line_id='light', frame_wall=999.95,
            pre_signal_cap_mps=6., pre_signal_emergency=False, cap_mps=0.)
        return ego, signal

    def test_only_fresh_prohibiting_signal_holds_are_excluded(self):
        ego, signal = self.sample()
        self.assertTrue(signal_waiting(ego, signal, 1000.))
        # At a red hold the acceleration ramp is re-evaluated from zero on
        # every command; a small positive independent target is still usable.
        self.assertTrue(signal_waiting(ego, dict(signal, pre_signal_cap_mps=.08), 1000.))
        for change in (dict(raw_state='UNKNOWN'), dict(raw_state='GREEN'), dict(fresh=False),
                       dict(phase='BRAKE', cap_mps=2.), dict(pre_signal_cap_mps=0.),
                       dict(pre_signal_emergency=True), dict(enabled=False), dict(active=False),
                       dict(frame_wall=998.), dict(stop_required=False)):
            with self.subTest(change=change):
                self.assertFalse(signal_waiting(ego, dict(signal, **change), 1000.))
        self.assertFalse(signal_waiting(dict(ego, speed_kmh=2.), signal, 1000.))
        self.assertFalse(signal_waiting(dict(ego, age_s=.3), signal, 1000.))

    def test_adjusted_clock_and_original_total_remain_auditable(self):
        tracker = LapTracker(ROUTE)
        tracker.update(0., 0., 0., 0., rtf=.8)
        tracker.update(2., 0., 2., 1., rtf=.8)
        tracker.update(3., 0., 1., 2., rtf=.8)
        tracker.update(3., 0., 0., 3., rtf=.8, signal_wait=True)
        tracker.update(3., 0., 0., 3.25, rtf=.8, signal_wait=True)
        tracker.update(3., 0., 0., 3.5, rtf=.8, signal_wait=True)
        tracker.update(3., 0., 0., 3.75, rtf=.8)
        tracker.update(4., 0., 1., 4., rtf=.8)
        tracker.finished_at = 4.
        record = tracker.record()
        self.assertEqual(record['lap_time_s'], 2.5)
        self.assertEqual(record['lap_time_sim_s'], 2.)
        self.assertEqual(record['lap_time_total_s'], 3.)
        self.assertEqual(record['lap_time_total_sim_s'], 2.4)
        self.assertEqual(record['signal_wait_s'], .5)
        self.assertEqual(record['signal_wait_sim_s'], .4)

    def test_roundabout_wait_motion_and_unobserved_gaps_stay_in_lap_time(self):
        tracker = LapTracker(ROUTE)
        tracker.update(0., 0., 0., 0.)
        tracker.update(2., 0., 2., 1.)
        tracker.update(2., 0., 0., 1.25)
        tracker.update(2., 0., 0., 1.5)
        self.assertEqual(tracker.signal_wait_s, 0.)
        tracker.update(3., 0., 2., 1.75, signal_wait=True)
        tracker.update(4., 0., 2., 2., signal_wait=True)
        self.assertEqual(tracker.signal_wait_s, 0.)
        tracker.update(4., 0., 0., 2.25, signal_wait=True)
        tracker.update(4., 0., 0., 10., signal_wait=True)
        self.assertEqual(tracker.signal_wait_s, 0.)

    def test_scenario_reset_discards_signal_wait_from_the_previous_lap(self):
        tracker = LapTracker(ROUTE)
        tracker.update(0., 0., 0., 0.)
        tracker.update(2., 0., 2., 1.)
        tracker.update(2., 0., 0., 1.25, signal_wait=True)
        tracker.update(2., 0., 0., 1.5, signal_wait=True)
        self.assertGreater(tracker.signal_wait_s, 0.)
        tracker.update(100., 100., 0., 1.6)
        self.assertEqual(tracker.signal_wait_s, 0.)
        self.assertEqual(tracker.signal_wait_sim_s, 0.)

    def test_signal_application_records_independent_control_before_its_own_stop(self):
        for body_cap, body_emergency in ((6., False), (0., False), (0., True)):
            controller, worker, ego = controller_at(.1)
            controller.evaluate(ego)
            worker.publish('RED')
            ego.update(measured=True, age_s=.01, speed_kmh=0.)
            plan = DrivingPlan(1, np.array([[0., 0.], [10., 0.], [20., 0.]]),
                ego, ego, 99.9, .1, 100., body_cap, 60., body_emergency, {})
            cap, result = controller.apply(ego, body_cap, plan)
            signal = result.validation['traffic_signal']
            with self.subTest(body_cap=body_cap, body_emergency=body_emergency):
                self.assertEqual(cap, 0.)
                self.assertEqual(signal['pre_signal_cap_mps'], body_cap)
                self.assertEqual(signal['pre_signal_emergency'], body_emergency)
                self.assertEqual(signal_waiting(ego, signal, time.time()), body_cap > .3)
            # The final publication recheck receives a signal-clamped emergency
            # decision. Its timing attribution must retain the independent cap.
            cap, repeated = controller.apply(ego, cap, result,
                control_before_signal=(body_cap, plan))
            self.assertEqual(cap, 0.)
            self.assertTrue(repeated.emergency)
            self.assertEqual(signal_waiting(ego, repeated.validation['traffic_signal'], time.time()),
                             body_cap > .3)
            rejected = result.decided(allowed_mps=0., emergency=True,
                                      stop_reason='source publication rejected')
            _, rejected = controller.apply(ego, 0., rejected)
            self.assertFalse(signal_waiting(ego, rejected.validation['traffic_signal'], time.time()))

    def test_actual_red_stop_in_the_follower_deadband_counts_without_ideal_hold(self):
        controller, worker, ego = controller_at(1.07)
        controller.evaluate(ego)
        worker.publish('RED')
        ego.update(measured=True, age_s=.01, speed_kmh=0.)
        plan = DrivingPlan(1, np.array([[0., 0.], [10., 0.], [20., 0.]]),
            ego, ego, 99.9, .1, 100., 6., 60., False, {})
        _, stopped = controller.apply(ego, .4, plan)
        signal = stopped.validation['traffic_signal']
        self.assertEqual(signal['phase'], 'BRAKE')
        self.assertGreater(signal['cap_mps'], .05)
        self.assertTrue(signal_waiting(ego, signal, time.time()))
        self.assertFalse(signal_waiting(dict(ego, speed_kmh=1.), signal, time.time()))


if __name__ == '__main__':
    unittest.main()
