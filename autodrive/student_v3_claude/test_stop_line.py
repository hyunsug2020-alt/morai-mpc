"""Intersection stop, release, and controller latency regressions."""
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from stop_line import StopLineController, entry_is_clear


LINE = dict(id="entry", station_m=50., xy=[50., 0.])
STATION = 50. - StopLineController.FRONT_M - 1.


class StopLineTests(unittest.TestCase):
    def test_zero_dwell_accepts_a_fresh_stop_in_the_same_frame_at_any_rtf(self):
        for scale in (.5, 1.):
            gate = StopLineController([LINE], hold_s=0.)
            result = gate.update(STATION, 0., 0., True, time_scale=scale)
            self.assertEqual(result['state'], 'ENTER')
            self.assertEqual(result['held_sim_s'], 0.)
            self.assertFalse(result['stop'])

    def test_zero_dwell_does_not_accept_moving_or_stale_standstill(self):
        gate = StopLineController([LINE], hold_s=0.)
        stale = gate.update(STATION, 0., 0., True, observation_age_s=.51)
        self.assertTrue(stale['stop'])
        self.assertFalse(stale['hold_latched'])
        moving = gate.update(STATION, .3, .1, True, observation_age_s=.1)
        self.assertTrue(moving['stop'])
        self.assertFalse(moving['hold_latched'])
        self.assertEqual(gate.update(STATION, .15, .2, True)['state'], 'ENTER')

    def test_zero_dwell_retains_conflict_wait_and_rearms_after_reset(self):
        gate = StopLineController([LINE], hold_s=0.)
        blocked = gate.update(STATION, 0., 0., False)
        self.assertEqual(blocked['state'], 'WAIT_CLEAR')
        self.assertTrue(blocked['stop'])
        self.assertEqual(gate.update(STATION, 0., 2., True)['state'], 'ENTER')
        self.assertTrue(gate.update(STATION, .3, 0., True)['stop'])
        self.assertFalse(gate.hold_latched)

    def test_invalid_hold_duration_fails(self):
        for duration in (-.1, float('nan'), float('inf')):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                StopLineController([LINE], hold_s=duration)

    def test_empty_detection_still_requires_complete_stop(self):
        gate = StopLineController([LINE])
        result = gate.update(20., 8., 0., clear=True)
        self.assertEqual(result["state"], "APPROACH")
        for i in range(40):
            result = gate.update(STATION, .3, (i + 1) / 10., clear=True)
            self.assertTrue(result["stop"])
        # Time spent creeping does not count as a full stop.
        for i in range(10):
            result = gate.update(STATION, 0., 4.1 + i / 10., clear=True)
            self.assertTrue(result["stop"])
        self.assertEqual(gate.update(STATION, 0., 5.1, clear=True)["state"], "ENTER")

    def test_completed_stop_accepts_first_fresh_conflict_free_gap(self):
        gate = StopLineController([LINE])
        for i in range(21):
            result = gate.update(STATION, 0., i / 10., clear=False)
            self.assertTrue(result["stop"])
        result = gate.update(STATION, 0., 2.1, clear=True)
        self.assertEqual(result["state"], "ENTER")

    def test_frame_gap_does_not_count_as_stopped_observation(self):
        gate = StopLineController([LINE])
        gate.update(STATION, 0., 0., clear=True)
        self.assertTrue(gate.update(STATION, 0., 10., clear=True)["stop"])

    def test_overshoot_does_not_drop_active_stop(self):
        gate = StopLineController([LINE])
        gate.update(STATION, 2., 0., clear=True)
        self.assertTrue(gate.update(49., 2., .1, clear=True)["stop"])

    def test_release_does_not_repeat_stop_on_position_jitter(self):
        gate = StopLineController([LINE])
        for i in range(24):
            gate.update(STATION, 0., i / 10., clear=True)
        for i in range(10):
            self.assertEqual(gate.update(STATION + (-1) ** i * .2, 0., 2.4 + i / 10., True)["state"], "ENTER")
        gate.update(63., 6., 3.5, True)
        self.assertIsNone(gate.update(62.5, 6., 3.6, True))
        # A scenario reset back to the approach rearms this stop.
        self.assertEqual(gate.update(20., 0., 3.7, True)["state"], "APPROACH")

    def test_rest_just_beyond_stop_window_holds_and_releases_without_creep(self):
        # follow-20261010T092032Z lap 2, 18:29:09.43-12.16: at rest 1.117-1.158 m from the line,
        # pose ages 0.16-0.26 s. The hold never started (window 1.10 m) and the age reserve
        # flipped the stop to APPROACH, so we crept to 1.09 m first (~4 s lost).
        gaps = (1.117, 1.122, 1.122, 1.122, 1.127, 1.133, 1.141, 1.144, 1.148, 1.152, 1.155, 1.157, 1.158)
        ages = (.168, .194, .219, .16, .163, .203, .226, .208, .256, .193, .243, .177, .23)
        for clear_from in (0., 2.):
            with self.subTest(clear_from=clear_from):
                gate, released_at = StopLineController([LINE]), None
                for i in range(40):
                    now = i * .25
                    gap, age = gaps[min(i, len(gaps) - 1)], ages[i % len(ages)]
                    result = gate.update(50. - gate.FRONT_M - gap, 0., now, now >= clear_from,
                                         observation_age_s=age)
                    if result["state"] == "ENTER":
                        released_at = now
                        break
                    self.assertTrue(result["stop"])
                    self.assertEqual(result["cap_mps"], 0.)       # no creep toward the line
                    self.assertIn(result["state"], ("HOLD", "WAIT_CLEAR"))
                self.assertEqual(released_at, max(StopLineController.HOLD_S, clear_from))
        # Farther out at rest is still an approach to the line, not the mandatory stop.
        far = StopLineController([LINE]).update(50. - StopLineController.FRONT_M - 1.6, 0., 0., True,
                                                observation_age_s=.2)
        self.assertEqual(far["state"], "APPROACH")
        self.assertGreater(far["cap_mps"], 0.)

    def test_commanded_stop_is_held_where_the_car_comes_to_rest(self):
        # follow-20261010T130317Z lap 2: stop commanded at 1.71 m at 5 km/h, rest at 1.54 m
        # (rest window 1.40 m) -> APPROACH released a creep and the hold restarted.
        gate = StopLineController([LINE])
        trace = ((1.87, 1.44), (1.78, 1.44), (1.71, 1.42), (1.64, 1.22), (1.57, .6), (1.54, .1), (1.54, 0.))
        now, states = 0., []
        for gap, speed in trace:
            states.append(gate.update(50. - gate.FRONT_M - gap, speed, now, False, observation_age_s=.25)['state'])
            now += .1
        first = states.index('HOLD')
        self.assertTrue(all(state == 'HOLD' for state in states[first:]), states)
        self.assertTrue(gate.at_line)
        result = None
        for _ in range(12):
            result = gate.update(50. - gate.FRONT_M - 1.54, 0., now, False, observation_age_s=.1)
            now += .1
            self.assertNotEqual(result['state'], 'APPROACH')
        self.assertTrue(result['hold_latched'])
        self.assertEqual(gate.update(50. - gate.FRONT_M - 1.54, 0., now, True, observation_age_s=.1)['state'], 'ENTER')

    def test_rest_short_of_the_line_without_a_commanded_stop_still_approaches(self):
        # Stopped 1.9 m before the line by something else: drive up to the line.
        gate = StopLineController([LINE])
        result = gate.update(50. - gate.FRONT_M - 1.9, 0., 0., True, observation_age_s=.1)
        self.assertEqual(result['state'], 'APPROACH')
        self.assertGreater(result['cap_mps'], 0.)
        self.assertFalse(gate.at_line)

    def test_current_risks_cannot_be_replaced_by_drive_to_line(self):
        self.assertFalse(entry_is_clear(None, True, True))
        self.assertFalse(entry_is_clear(None, False, False))
        for kind in ("stop", "slow"):
            self.assertFalse(entry_is_clear(dict(kind=kind), False, True))
        self.assertFalse(entry_is_clear(dict(kind="follow", source="traffic", v_mps=2.,
                                            reason="approach conflict with braking reserve"), False, True))
        self.assertTrue(entry_is_clear(None, False, True))

    def test_braking_with_half_second_delay_stops_before_line(self):
        # Delayed speed command, limited deceleration and acceleration; no
        # object detections. Check physical bumper position before release.
        for initial_kmh in (20., 28.94, 40.):
            gate = StopLineController([LINE])
            x, speed, dt = 0., initial_kmh / 3.6, 1. / 15.
            commands = [speed] * 8
            minimum_gap, held, released = 999., False, False
            for i in range(1800):
                result = gate.update(x, speed, i * dt, True)
                if result["state"] == "ENTER":
                    released = True
                    break
                minimum_gap = min(minimum_gap, 50. - x - gate.FRONT_M)
                held |= speed <= .15 and result["stop"]
                commands.append(min(initial_kmh / 3.6, result["cap_mps"]))
                target = commands.pop(0)
                previous = speed
                speed = min(target, speed + 1.2 * dt) if target > speed else max(target, speed - 2.5 * dt)
                x += .5 * (speed + previous) * dt
            with self.subTest(initial_kmh=initial_kmh):
                self.assertTrue(held and released)
                self.assertGreater(minimum_gap, .2)

    @staticmethod
    def delayed_approach(initial_kmh, gate=None):
        """Replay a 0.15 s pose delay, 0.5 s actuator delay and one 0.755 s plan."""
        gate = gate or StopLineController([LINE])
        dt, x, speed = 1. / 150., 0., initial_kmh / 3.6
        target, due, pending = speed, 0., None
        history, commands = [], []
        minimum_gap, delayed, held, released = 999., False, False, False
        for step in range(12000):
            now = step * dt
            history.append((now, x, speed))
            if pending is None and now >= due:
                observed_index = max(0, step - 23)
                observed = history[observed_index]
                gap = LINE['station_m']-observed[1]-gate.FRONT_M
                planning = .755 if not delayed and gap <= 2. and observed[2] > .15 else 0.
                delayed |= planning > 0.
                pending = (now+planning, observed)
            if pending is not None and now >= pending[0]:
                observed = pending[1]
                result = gate.update(observed[1], observed[2], now, True,
                                     observation_age_s=now-observed[0])
                if result['state'] == 'ENTER':
                    released = True
                    break
                held |= speed <= .15 and result['stop']
                commands.append((now+.5, min(initial_kmh/3.6, result['cap_mps'])))
                pending, due = None, now+1./15.
            while commands and now >= commands[0][0]:
                target = commands.pop(0)[1]
            previous = speed
            speed = min(target, speed+1.2*dt) if target > speed else max(target, speed-2.5*dt)
            x += .5*(speed+previous)*dt
            minimum_gap = min(minimum_gap, LINE['station_m']-x-gate.FRONT_M)
        return dict(minimum_gap=minimum_gap, delayed=delayed, held=held, released=released)

    def test_recorded_planning_pause_and_pose_delay_keep_bumper_before_line(self):
        for initial_kmh in (20., 28.94, 40.):
            with self.subTest(initial_kmh=initial_kmh):
                result = self.delayed_approach(initial_kmh)
                self.assertTrue(result['delayed'])
                self.assertTrue(result['held'] and result['released'])
                self.assertGreater(result['minimum_gap'], .2)

    def test_observation_delay_reduces_cap_without_changing_reported_station(self):
        station = 50.-StopLineController.FRONT_M-1.9513
        result = StopLineController([LINE]).update(station, 6.7/3.6, 100., True,
                                                  observation_age_s=.755)
        self.assertAlmostEqual(result['distance_m'], 1.9513)
        self.assertLess(result['effective_distance_m'], .3)
        self.assertTrue(result['stop'])
        self.assertEqual(result['cap_mps'], 0.)

    def test_stale_or_invalid_pose_age_cannot_finish_mandatory_hold(self):
        gate = StopLineController([LINE])
        for step in range(40):
            result = gate.update(STATION, 0., step/10., True, observation_age_s=.6)
            self.assertTrue(result['stop'])
            self.assertNotEqual(result['state'], 'ENTER')
        for age in (-.1, float('nan'), float('inf'), None):
            self.assertEqual(gate.update(STATION, 0., 4., True, observation_age_s=age)['cap_mps'], 0.)

    def test_runtime_uses_application_clock_and_preserved_receive_time(self):
        from gui import Runtime
        runtime = Runtime.__new__(Runtime)
        runtime.stop_line_controller = StopLineController([LINE])
        runtime.lanes = SimpleNamespace(route_progress=lambda ego: ego['x'])
        runtime.yield_cap_mps = None
        ego = dict(x=50.-StopLineController.FRONT_M-1.9513, y=0.,
                   speed_kmh=6.7, received=99.95, age_s=.05)
        with patch('gui.time.monotonic', return_value=100.8):
            emergency, status = runtime.apply_stop_line(ego, 100., None, False, True)
        self.assertTrue(emergency)
        self.assertAlmostEqual(status['observation_age_s'], .85)
        self.assertEqual(runtime.stop_line_controller.last_time, 100.8)
        self.assertEqual(ego['age_s'], .05)
        self.assertAlmostEqual(status['distance_m'], 1.9513)

    def test_runtime_invalid_receive_time_retains_safe_stop(self):
        from gui import Runtime
        runtime = Runtime.__new__(Runtime)
        runtime.stop_line_controller = StopLineController([LINE])
        runtime.lanes = SimpleNamespace(route_progress=lambda ego: ego['x'])
        runtime.yield_cap_mps = None
        for received in (101., float('nan'), None):
            with self.subTest(received=received), patch('gui.time.monotonic', return_value=100.8):
                ego = dict(x=STATION, y=0., speed_kmh=0., received=received)
                emergency, status = runtime.apply_stop_line(ego, 100., None, False, True)
                self.assertTrue(emergency)
                self.assertEqual(status['cap_mps'], 0.)

    def test_runtime_integration_limits_speed_without_erasing_other_stop(self):
        from gui import Runtime
        runtime = Runtime.__new__(Runtime)
        runtime.stop_line_controller = StopLineController([LINE])
        runtime.lanes = SimpleNamespace(route_progress=lambda ego: ego["x"])
        runtime.yield_cap_mps = None
        ego = dict(x=STATION, y=0., speed_kmh=0.)
        emergency, status = runtime.apply_stop_line(ego, 0., None, False, True)
        self.assertTrue(emergency)
        self.assertEqual(runtime.yield_cap_mps, 0.)
        self.assertEqual(status["state"], "HOLD")
        for i in range(1, 40):
            emergency, status = runtime.apply_stop_line(ego, i / 10., None, True, True)
            self.assertTrue(emergency)
            self.assertNotEqual(status["state"], "ENTER")


if __name__ == "__main__":
    unittest.main()
