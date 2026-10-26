"""A stopped crossing yield resumes on a checked gap, without an approach restart."""
import unittest
from types import SimpleNamespace

import numpy as np

from live_path_safety import remember_pedestrian_brakes
from test_pedestrian_approach import runtime
from test_ped_guard import EGO, PATH, person


class CrossingBrakeTests(unittest.TestCase):
    def step(self, host, obj, speed, now):
        ego = dict(EGO, speed_kmh=speed)
        result = host.pedestrian_guard([obj], ego, PATH, now, points=np.empty((0, 2)))
        command = host.apply_pedestrian_guard(result, None, ego, None, [obj], now, PATH)
        return result, command

    def test_emergency_stop_cannot_restart_a_stopping_approach(self):
        host = runtime()
        obj = person(12., 0., 0., .4)
        result, _ = self.step(host, obj, 18., 1.)
        self.assertEqual(result['kind'], 'stop')
        self.assertIn(16, host._ped_crossing_brakes)
        # The same geometry at standstill normally permits approaching the
        # stop point. After this emergency stop, that is the observed restart.
        result, command = self.step(host, obj, 0., 1.1)
        self.assertEqual((result['kind'], command['v_mps']), ('stop', 0.))
        result, command = self.step(host, person(12., 1.7, 0., .4), 0., 1.2)
        self.assertEqual(result['kind'], 'clear')
        self.assertIsNone(command)
        self.assertFalse(host._ped_crossing_brakes)
        self.assertIn(16, host._ped_released)

    def test_live_person_brake_is_carried_into_the_next_gui_plan(self):
        host = runtime()
        obj = person(12., 0., 0., .4)
        validation = dict(people_emergency=True,
                          objects=[dict(name='p', uid=16, state='moving')])
        remember_pedestrian_brakes(host, validation, [('semantic', [obj], .9, .1)], 1.)
        result, command = self.step(host, obj, 0., 1.1)
        self.assertEqual((result['kind'], command['v_mps']), ('stop', 0.))

    def test_braking_for_the_same_crossing_cannot_accelerate_before_standstill(self):
        host = runtime()
        obj = person(12., 0., 0., .4)
        result, _ = self.step(host, obj, 18., 1.)
        self.assertEqual(result['kind'], 'stop')
        for now, speed in ((1.1, 8.), (1.2, .9), (1.3, 0.)):
            with self.subTest(speed_kmh=speed):
                result, command = self.step(host, obj, speed, now)
                self.assertEqual((result['kind'], command['v_mps']), ('stop', 0.))
                self.assertIn(16, host._ped_crossing_brakes)

    def test_a_checked_crossing_gap_can_release_while_decelerating(self):
        host = runtime()
        self.step(host, person(12., 0., 0., .4), 18., 1.)
        result, command = self.step(host, person(12., 1.7, 0., .4), .9, 1.1)
        self.assertEqual(result['kind'], 'clear')
        self.assertIsNone(command)
        self.assertFalse(host._ped_crossing_brakes)

    def test_temporal_arrival_after_gap_releases_before_corridor_is_empty(self):
        host = runtime()
        host._ped_crossing_brakes = {16: 1.}
        result, command = self.step(host, person(12., 1., 0., 1.), 0., 1.1)
        self.assertIn(result['kind'], ('clear', 'yield'))
        self.assertFalse(result.get('stopping', False))
        if command is not None:
            self.assertGreater(command['v_mps'], 0.)
        self.assertFalse(host._ped_crossing_brakes)

    def test_unrelated_person_and_old_evidence_cannot_inherit_hold(self):
        for brakes, uid in (({16: 1.}, 17), ({16: -20.}, 16)):
            host = runtime()
            host._ped_crossing_brakes = brakes
            result, command = self.step(host, person(12., 0., 0., .4, uid), 0., 1.1)
            self.assertEqual(result['kind'], 'yield')
            self.assertGreater(command['v_mps'], 0.)
            self.assertNotIn(uid, host._ped_crossing_brakes)

    def test_nonperson_unknown_or_stale_live_stop_does_not_seed_hold(self):
        obj = person(12., 0., 0., .4)
        for emergency, state, age, name in ((False, 'moving', .1, 'p'),
                                           (True, 'unknown', .1, 'p'),
                                           (True, 'moving', .36, 'p'),
                                           (True, 'moving', .1, 'different')):
            host = SimpleNamespace()
            remember_pedestrian_brakes(host, dict(people_emergency=emergency,
                objects=[dict(name=name, uid=16, state=state)]),
                [('semantic', [obj], .9, age)], 1.)
            self.assertFalse(getattr(host, '_ped_crossing_brakes', {}))


if __name__ == '__main__':
    unittest.main()
