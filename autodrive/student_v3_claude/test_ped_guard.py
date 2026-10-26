"""Pedestrian guard (auto runs 243 / 247: crossing people hit while we stopped on their line)."""
import unittest

import numpy as np

import ped_guard as pg


class Known:
    rtf = 1.

    def state(self, uid):
        return {"state": "moving", "age": 5.}


def person(f, l, vf, vl, uid=16):
    speed = float(np.hypot(vf, vl))
    return (f, l, .6, "p", "pedestrian", speed, .6, .6, 0., vf, vl, uid)


EGO = dict(x=0., y=0., heading_deg=0.)
PATH = np.column_stack((np.arange(-5., 80., .5), np.zeros(170)))


def decide(o, kmh, committed=None, zone_kmh=30.):
    return pg.plan([o], dict(EGO, speed_kmh=kmh), PATH, kmh / 3.6, zone_kmh / 3.6, Known(), committed or {}, 0.)


class TrackerTests(unittest.TestCase):
    def test_early_fused_speed_dropout_keeps_recent_crossing_velocity(self):
        tracker = pg.PedTracker()
        tracker.update([person(12., 0., 0., .89)], 0., 0.)
        out = tracker.update([person(12., .1, 0., 0.)], .1, .1)
        self.assertAlmostEqual(out[0][10], .89)
        self.assertEqual(tracker.state(16)["state"], "fused")
        # Carrying the velocity must not refresh its age indefinitely.
        tracker.update([person(12., .1, 0., 0.)], 2.01, 2.01)
        self.assertEqual(tracker.state(16)["state"], "unknown")

    def test_flickering_fused_speed_is_replaced_by_the_walk(self):
        tracker = pg.PedTracker()
        out = None
        for k in range(10):     # 0.45 m/s across, fused velocity 0 (run 247)
            t = .1 * k
            out = tracker.update([(10., -2.8 + .45 * t, .6, "p", "pedestrian", 0., .6, .6, 0., 0., 0., 120)], t, t)
        self.assertAlmostEqual(out[0][10], .45, delta=.05)
        self.assertEqual(tracker.state(120)["state"], "moving")

    def test_standing_person_stays_standing(self):
        tracker = pg.PedTracker()
        rng = np.random.default_rng(1)
        for k in range(20):
            t = .1 * k
            out = tracker.update([(10. + rng.normal(0., .08), -2.8 + rng.normal(0., .08), .6, "p", "pedestrian",
                                   0., .6, .6, 0., 0., 0., 7)], t, t)
        self.assertEqual(out[0][5], 0.)
        self.assertEqual(tracker.state(7)["state"], "standing")

    def test_real_time_factor_scales_walking_speed(self):
        tracker = pg.PedTracker()
        for k in range(25):   # reported 10 m/s, moved 5.5 m per second: MORAI at 0.55 real time
            tracker.update_rtf(dict(x=5.5 * .1 * k, y=0., speed_kmh=36.), .1 * k)
        self.assertAlmostEqual(tracker.rtf, .55, delta=.03)
        for k in range(10):
            t = .1 * k
            out = tracker.update([(20., -2.8 + .45 * t, .6, "p", "pedestrian", 0., .6, .6, 0., 0., 0., 3)], t, t)
        self.assertAlmostEqual(out[0][10], .45 / .55, delta=.08)


class PlanTests(unittest.TestCase):
    def test_far_crossing_person_uses_deceleration_gap(self):
        r = decide(person(35., -3., 0., .8), 30.)
        self.assertEqual(r["kind"], "yield")
        self.assertFalse(r.get("stopping", False))
        self.assertGreaterEqual(r["cap_mps"], 2.)
        self.assertLess(r["cap_mps"], 30. / 3.6)

    def test_run247_no_longer_uses_unsafe_front_gap(self):
        r = decide(person(7.1, -2.3, 0., .71), 28.6)
        self.assertEqual(r["kind"], "stop")
        self.assertEqual(r["cap_mps"], 0.)
        self.assertTrue(r["stopping"])
        self.assertGreater(r["required_decel_mps2"], pg.HARD_DECEL)
        self.assertEqual(r["pass_ids"], set())

    def test_committed_pass_survives_time_margin_noise(self):
        initial = decide(person(10., -3.5, 0., .7), 37.)
        self.assertEqual(initial["kind"], "pass")
        noisy = person(10., -2.9, 0., .7)
        self.assertNotEqual(decide(noisy, 37.)["kind"], "pass")
        r = decide(noisy, 37., committed={16: 5.})
        self.assertEqual(r["kind"], "pass")
        self.assertNotEqual(decide(person(10., -2.65, 0., .7), 37., committed={16: 5.})["kind"], "pass")

    def test_walker_along_the_shoulder_is_clear(self):
        r = decide(person(15., 2.2, 1.2, 0.), 40.)
        self.assertEqual(r["kind"], "clear")

    def test_stopped_car_does_not_reverse_for_crossing_person(self):
        r = decide(person(3.2, -3.5, 0., .5), 0.)
        self.assertEqual(r["kind"], "stop")
        self.assertIsNone(r["reverse_m"])

    def test_person_crossing_well_ahead_of_a_stopped_car_is_waited_for(self):
        r = decide(person(9.5, -2.5, 0., .7), 0.)
        self.assertIn(r["kind"], ("stop", "yield"))
        # Creep at most to 2.5 m short of their walk (f = 9.5).
        self.assertLessEqual(r["along"], 9.5 - pg.EGO_FRONT_M - pg.STOP_CLEAR_M + .3)

    def test_unknown_person_beside_the_path_holds_speed_only(self):
        class Unknown(Known):
            def state(self, uid):
                return {"state": "unknown", "age": .2}
        r = pg.plan([person(14., -2.5, 0., 0.)], dict(EGO, speed_kmh=39.), PATH, 39. / 3.6, 40. / 3.6, Unknown(), {}, 0.)
        self.assertEqual(r["kind"], "hold")
        self.assertAlmostEqual(r["cap_mps"], 39. / 3.6, delta=.01)

    def test_contest_approach_slows_before_a_close_walker_starts(self):
        # Recorded pre-crossing geometry: 13.8 m ahead, 2.55 m beside the
        # path, with weak apparent motion along the road before activation.
        r = pg.plan([person(13.8, -2.55, .63, 0.)], dict(EGO, speed_kmh=29.2),
                    PATH, 29.2 / 3.6, 30. / 3.6, Known(), {}, 0., anticipate_crossing=True)
        self.assertEqual(r["kind"], "hold")
        self.assertGreater(r["cap_mps"] * 3.6, 10.)
        self.assertLess(r["cap_mps"] * 3.6, 20.)

    def test_stationary_contest_walker_caps_shrink_without_an_off_path_stop(self):
        caps = []
        for distance in (22., 16., 10., 5.):
            r = pg.plan([person(distance, -2.6, 0., 0.)], dict(EGO, speed_kmh=30.),
                        PATH, 30. / 3.6, 30. / 3.6, Known(), {}, 0., anticipate_crossing=True)
            self.assertEqual(r["kind"], "hold")
            caps.append(r["cap_mps"])
        self.assertEqual(caps, sorted(caps, reverse=True))
        self.assertGreaterEqual(min(caps), 10. / 3.6)

    def test_far_and_parallel_walkers_do_not_trigger_contest_approach_caps(self):
        for walker in (person(32., -2.6, 0., 0.), person(15., -4., 0., 0.),
                       person(15., -2.6, 1.4, 0.), person(15., 1.6, 0., .7)):
            r = pg.plan([walker], dict(EGO, speed_kmh=30.), PATH, 30. / 3.6,
                        30. / 3.6, Known(), {}, 0., anticipate_crossing=True)
            self.assertEqual(r["kind"], "clear")
            self.assertIsNone(r["cap_mps"])

    def test_stopped_ego_keeps_the_existing_standing_person_wait_rules(self):
        r = pg.plan([person(10., -2.6, 0., 0.)], dict(EGO, speed_kmh=0.), PATH,
                    0., 30. / 3.6, Known(), {}, 0., anticipate_crossing=True)
        self.assertEqual(r["kind"], "clear")
        self.assertIsNone(r["cap_mps"])


if __name__ == "__main__":
    unittest.main()
