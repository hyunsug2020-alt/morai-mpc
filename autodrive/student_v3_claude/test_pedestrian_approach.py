"""Stopping-readiness caps survive live checking and mixed pedestrian decisions."""
import json
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import ped_guard as pg
from gui import Runtime
from test_ped_guard import EGO, PATH, Known, person


class Unknown(Known):
    def state(self, uid):
        return dict(state='unknown', age=.1)


class Standing(Known):
    rtf = .84

    def state(self, uid):
        return dict(state='standing', age=2.)


def runtime(provider=True, tracker=None, zone_kmh=30.):
    host = Runtime.__new__(Runtime)
    host.zone_kmh = zone_kmh
    host.path_publisher = SimpleNamespace(zone_mps=zone_kmh / 3.6,
        _safety_provider=object() if provider else None)
    host._ped_tracker = tracker or Known()
    return host


class PedestrianApproachTests(unittest.TestCase):
    def test_recorded_cyclist_approach_preserves_cap_on_exact_published_path(self):
        data = json.loads((Path(__file__).parent /
                           'test_data/pedestrian_approach_20261012.json').read_text())
        ego = data['ego']
        world = np.asarray(data['world_path'])
        yaw = np.radians(ego['heading_deg'])
        rotation = np.array([[np.cos(yaw), np.sin(yaw)],
                             [-np.sin(yaw), np.cos(yaw)]])
        path = (world - [ego['x'], ego['y']]) @ rotation.T
        np.testing.assert_allclose(path[0], [0., 0.], atol=1e-10)
        objects = [tuple(row) for row in data['objects']]
        raw = pg.plan(objects, ego, path, ego['speed_kmh'] / 3.6,
                      data['zone_kmh'] / 3.6, Standing(), now=data['now'],
                      anticipate_crossing=True, released={14})
        self.assertEqual(raw['kind'], 'hold')
        self.assertAlmostEqual(raw['cap_mps'], 10. / 3.6)
        self.assertEqual(data['source']['original_target_mps'], 30. / 3.6)
        host = runtime(tracker=Standing())
        host._ped_released = {14: data['now'] + 2.}
        checked = host.pedestrian_guard(objects, ego, path, data['now'],
                                        anticipate_crossing=True)
        self.assertEqual(checked['cap_mps'], raw['cap_mps'])
        self.assertEqual(host._ped_guard_log['approach_cap_kmh'], 10.)
        command = host.apply_pedestrian_guard(checked, None, ego, None, objects,
                                              data['now'], path_local=path)
        self.assertEqual(command['kind'], 'follow')
        self.assertEqual(command['v_mps'], 10. / 3.6)

    def test_live_provider_does_not_raise_approach_cap_or_a_lower_zone(self):
        objects = [person(8., -2.4, 0., 0.)]
        for provider in (False, True):
            for zone_kmh, expected in ((30., 10.), (8., 8.)):
                with self.subTest(provider=provider, zone_kmh=zone_kmh):
                    host = runtime(provider, zone_kmh=zone_kmh)
                    r = host.pedestrian_guard(objects, dict(EGO, speed_kmh=24.), PATH,
                                              1., anticipate_crossing=True)
                    self.assertEqual(r['kind'], 'hold')
                    self.assertAlmostEqual(r['cap_mps'] * 3.6, expected)

    def test_unknown_at_standstill_still_delegates_to_live_checker(self):
        objects = [person(8., -2.4, 0., 0.)]
        for provider, expected in ((False, 0.), (True, 30.)):
            host = runtime(provider, Unknown())
            r = host.pedestrian_guard(objects, dict(EGO, speed_kmh=0.), PATH,
                                      1., anticipate_crossing=True)
            self.assertEqual(r['kind'], 'hold')
            self.assertIsNone(r.get('approach_cap_mps'))
            self.assertAlmostEqual(r['cap_mps'] * 3.6, expected)

    def test_approach_and_unknown_keep_the_more_restrictive_speed(self):
        host = runtime(tracker=Unknown())
        r = host.pedestrian_guard([person(8., -2.4, 0., 0.)],
                                  dict(EGO, speed_kmh=6.), PATH, 1.,
                                  anticipate_crossing=True)
        self.assertAlmostEqual(r['approach_cap_mps'] * 3.6, 10.)
        self.assertAlmostEqual(r['cap_mps'] * 3.6, 6.)

    def test_approach_cap_ends_after_the_actual_front_passes_the_person(self):
        limit = pg.EGO_FRONT_M - pg.PED_R - pg.BESIDE_MARGIN_M
        for forward, expected in ((limit - .001, 30.), (limit + .001, 10.)):
            host = runtime()
            r = host.pedestrian_guard([person(forward, -2.4, 0., 0.)],
                                      dict(EGO, speed_kmh=24.), PATH, 1.,
                                      anticipate_crossing=True)
            self.assertEqual(r['kind'], 'hold')
            self.assertAlmostEqual(r['cap_mps'] * 3.6, expected)
            self.assertEqual(r.get('approach_cap_mps') is None, expected == 30.)

    def test_temporal_pass_for_another_walker_keeps_approach_cap(self):
        objects = [person(8., -2.4, 0., 0., uid=1),
                   person(10., -5., 0., .5, uid=2)]
        ego = dict(EGO, speed_kmh=30.)
        host = runtime()
        r = host.pedestrian_guard(objects, ego, PATH, 1., anticipate_crossing=True)
        self.assertEqual(r['kind'], 'pass')
        self.assertEqual(r['pass_ids'], {2})
        self.assertAlmostEqual(r['cap_mps'] * 3.6, 10.)
        command = host.apply_pedestrian_guard(r, None, ego, None, objects, 1., PATH)
        self.assertEqual(command['kind'], 'follow')
        self.assertAlmostEqual(command['v_mps'] * 3.6, 10.)
        self.assertGreater(host._ped_pass_until[2], 1.)

    def test_unowned_person_does_not_erase_pass_approach_or_an_independent_stop(self):
        objects = [person(8., -2.4, 0., 0., uid=1),
                   person(10., -5., 0., .5, uid=2),
                   person(60., -3., 1.2, 0., uid=3)]
        ego = dict(EGO, speed_kmh=30.)
        host = runtime()
        r = host.pedestrian_guard(objects, ego, PATH, 1., anticipate_crossing=True)
        self.assertEqual(r['kind'], 'pass')
        host._pedestrian_hold_until = host._yield_stop_until = 100.
        command = host.apply_pedestrian_guard(r, None, ego, None, objects, 1., PATH)
        self.assertAlmostEqual(command['v_mps'] * 3.6, 10.)
        self.assertEqual(host._yield_stop_until, 100.)
        self.assertEqual(host._pedestrian_hold_until, 100.)
        for name in ('pedestrian', 'vehicle', 'static obstacle'):
            stop = dict(kind='stop', name=name, v_mps=0.)
            self.assertIs(host.apply_pedestrian_guard(r, stop, ego, None, objects,
                                                     1., PATH), stop)

    def test_lower_independent_vehicle_cap_wins_over_approach(self):
        host = runtime()
        objects = [person(8., -2.4, 0., 0.)]
        ego = dict(EGO, speed_kmh=24.)
        r = host.pedestrian_guard(objects, ego, PATH, 1., anticipate_crossing=True)
        lower = dict(kind='follow', name='vehicle', v_mps=5. / 3.6)
        self.assertIs(host.apply_pedestrian_guard(r, lower, ego, None, objects,
                                                 1., PATH), lower)

    def test_crossing_stop_wins_and_yield_keeps_lower_approach(self):
        for distance, expected in ((3., 'stop'), (24., 'yield')):
            objects = [person(8., -2.4, 0., 0., uid=1),
                       person(distance, -2.5, 0., .7, uid=2)]
            r = runtime().pedestrian_guard(objects, dict(EGO, speed_kmh=30.),
                                             PATH, 1., anticipate_crossing=True)
            self.assertEqual(r['kind'], expected)
            self.assertLessEqual(r['cap_mps'], 10. / 3.6)
            if expected == 'stop':
                self.assertEqual(r['cap_mps'], 0.)

    def test_standing_stop_semantics_survive_a_lower_approach_and_crossing_yield(self):
        objects = [person(8., -2.4, 0., 0., uid=1),
                   person(10., 0., 0., 0., uid=2),
                   person(14., -2.5, 0., .7, uid=3)]
        r = runtime().pedestrian_guard(objects, dict(EGO, speed_kmh=12.), PATH,
                                       1., anticipate_crossing=True)
        self.assertEqual(r['kind'], 'yield')
        self.assertTrue(r['stopping'])
        self.assertIn('person standing in our path:', r['reason'])
        self.assertAlmostEqual(r['along'], 10. - pg.EGO_FRONT_M - pg.STOP_CLEAR_M)
        self.assertIn(2, r['blocking_ids'])
        self.assertAlmostEqual(r['cap_mps'] * 3.6, 10.)


if __name__ == '__main__':
    unittest.main()
