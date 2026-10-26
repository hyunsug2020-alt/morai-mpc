"""Per-person near-stop hysteresis; crossing gaps and lost-person checks remain owned."""
import unittest

import numpy as np

import ped_guard as pg
from test_ped_guard import EGO, Known, person
from test_pedestrian_approach import runtime, Standing


PATH = np.column_stack((np.arange(0., 40.001, .01), np.zeros(4001)))


def at_stop(distance, uid=16, lateral=0., velocity=1.):
    return person(pg.EGO_FRONT_M + pg.STOP_CLEAR_M + distance,
                  lateral, 0., velocity, uid)


class PedestrianStopHoldTests(unittest.TestCase):
    def host(self, tracker=None):
        host = runtime(tracker=tracker or Known())
        host._ped_waiting = {16: 0.}
        return host

    def step(self, host, distance, now, kmh=0., **kwargs):
        obj = at_stop(distance, **kwargs)
        ego = dict(EGO, speed_kmh=kmh)
        result = host.pedestrian_guard([obj], ego, PATH, now, points=np.empty((0, 2)))
        command = host.apply_pedestrian_guard(result, None, ego, None, [obj], now, PATH)
        return result, command

    def test_logged_stop_point_variations_cannot_restart_the_same_approach(self):
        # Observed 404 stop points and creep speeds, with a synthetic straight
        # crossing to isolate the boundary. This is not a closed-loop bag replay.
        for distances in ((.98, 1.03, 1.02, 1.06, .99),
                          (.99, 1.00, 1.11, 1.10, .96)):
            host = self.host()
            for i, distance in enumerate(distances):
                result, command = self.step(host, distance, (i + 1) * .1,
                                            kmh=(.1, 1.5, 0., 2.91, 2.86)[i])
                self.assertEqual((result['kind'], result['cap_mps']), ('stop', 0.))
                self.assertEqual(host._ped_stop_hold_ids, {16})
                self.assertEqual(set(host._ped_waiting), {16})
                self.assertFalse(host._ped_released)
                self.assertEqual((command['kind'], command['v_mps']), ('stop', 0.))

    def test_waiting_without_a_near_stop_does_not_latch_an_approach(self):
        result, command = self.step(self.host(), 1.11, .1, kmh=1.5)
        self.assertEqual(result['kind'], 'yield')
        self.assertTrue(result['stopping'])
        self.assertGreater(command['v_mps'], 0.)
        self.assertFalse(result['stop_hold_ids'])

    def test_a_stop_point_moving_beyond_the_band_clears_its_old_hold(self):
        host = self.host()
        self.step(host, .98, .1)
        result, _ = self.step(host, 1.32, .2)
        self.assertEqual(result['kind'], 'yield')
        self.assertFalse(host._ped_stop_hold_ids)
        # Returning inside the band cannot revive an expired near-stop command.
        result, _ = self.step(host, 1.11, .3)
        self.assertEqual(result['kind'], 'yield')
        self.assertFalse(host._ped_stop_hold_ids)

    def test_checked_gap_releases_immediately_while_the_person_is_still_in_corridor(self):
        host = self.host()
        self.step(host, .98, .1)
        self.assertLess(1., pg.CORRIDOR_HALF_M)
        result, command = self.step(host, 1.03, .2, lateral=1.)
        self.assertEqual(result['kind'], 'clear')
        self.assertIsNone(command)
        self.assertFalse(host._ped_stop_hold_ids)
        self.assertFalse(host._ped_waiting)
        self.assertIn(16, host._ped_released)
        # Turning back into the path revokes the release and stops again.
        result, command = self.step(host, -.1, .3, lateral=.3, velocity=-1.)
        self.assertEqual((result['kind'], command['v_mps']), ('stop', 0.))
        self.assertNotIn(16, host._ped_released)

    def test_nonstopping_temporal_yield_is_not_held_by_a_previous_near_stop(self):
        host = self.host()
        self.step(host, .98, .1)
        result, command = self.step(host, 4., .2, kmh=1.5)
        self.assertEqual(result['kind'], 'yield')
        self.assertFalse(result.get('stopping', False))
        self.assertGreater(command['v_mps'], 0.)
        self.assertFalse(host._ped_stop_hold_ids)
        self.assertIn(16, host._ped_released)

    def test_mixed_decision_seeds_only_each_person_near_its_own_stop_point(self):
        host = self.host()
        host._ped_waiting[17] = 0.
        objects = [at_stop(.98), person(20., -4., 0., 1., uid=17)]
        result = host.pedestrian_guard(objects, dict(EGO, speed_kmh=1.5), PATH, .1)
        self.assertEqual(result['kind'], 'stop')
        self.assertEqual(result['blocking_ids'], {16, 17})
        self.assertEqual(result['stop_hold_ids'], {16})
        self.assertEqual(host._ped_stop_hold_ids, {16})

    def test_another_uid_cannot_inherit_a_near_stop(self):
        host = self.host()
        self.step(host, .98, .1)
        result, _ = self.step(host, 1.11, .2, uid=17)
        self.assertEqual(result['kind'], 'yield')
        self.assertFalse(host._ped_stop_hold_ids)
        # A new person immediately in front retains the original full stop.
        result = host.pedestrian_guard([person(3., -.8, 0., 1., uid=18)],
                                      dict(EGO, speed_kmh=5.), PATH, .3)
        self.assertEqual((result['kind'], result['cap_mps']), ('stop', 0.))

    def test_missing_person_keeps_hold_until_existing_lidar_clear_release(self):
        host = self.host()
        self.step(host, .98, .1)
        spot = np.array([at_stop(.98)[:2]])
        result = host.pedestrian_guard([], dict(EGO, speed_kmh=0.), PATH, .2, points=spot)
        self.assertEqual(result['kind'], 'stop')
        self.assertEqual(host._ped_stop_hold_ids, {16})
        # A fresh observation inside the resume band keeps that person's hold.
        result, _ = self.step(host, 1.11, .3)
        self.assertEqual(result['kind'], 'stop')
        for now in (.4, .9, 1.39):
            result = host.pedestrian_guard([], dict(EGO, speed_kmh=0.), PATH,
                                          now, points=np.empty((0, 2)))
            self.assertEqual(result['kind'], 'stop')
        result = host.pedestrian_guard([], dict(EGO, speed_kmh=0.), PATH,
                                      1.41, points=np.empty((0, 2)))
        self.assertEqual(result['kind'], 'clear')
        self.assertFalse(host._ped_stop_hold_ids)

    def test_standing_person_uses_the_same_band_and_exact_exit_boundary(self):
        for distance, kind in ((pg.STOP_RESUME_M, 'stop'),
                               (pg.STOP_RESUME_M + 1e-6, 'yield')):
            result = pg._stand_limit([(16, 'p', distance)], 0., {16}, {16})
            self.assertEqual(result['kind'], kind)
        host = self.host(Standing())
        for i, distance in enumerate((.98, 1.03, 1.11)):
            result, _ = self.step(host, distance, .1 * (i + 1), velocity=0.)
            self.assertEqual((result['kind'], result['cap_mps']), ('stop', 0.))
        result, _ = self.step(host, 1.32, .4, velocity=0.)
        self.assertEqual(result['kind'], 'yield')
        self.assertFalse(host._ped_stop_hold_ids)

    def test_high_speed_contact_checks_remain_an_emergency_stop(self):
        host = self.host()
        self.step(host, .98, .1)
        result, command = self.step(host, 1.11, .2, kmh=30.)
        self.assertEqual(command['v_mps'], 0.)
        self.assertTrue(result['stopping'])
        self.assertGreater(result['required_decel_mps2'], pg.HARD_DECEL)


if __name__ == '__main__':
    unittest.main()
