"""Crossing-time contracts and logged run-417 inputs; no simulator required."""
import unittest
from unittest.mock import patch

import numpy as np

import ped_guard as pg
from gui import Runtime
from test_ped_guard import EGO, PATH, Known, decide, person


class CrossingTimeTests(unittest.TestCase):
    def test_review5_standstill_crossing_uses_executable_acceleration(self):
        from gui import LAUNCH_ACCEL_MPS2
        o = person(20., -4., 0., 1.5)
        t = np.arange(0., 12., .001)

        def executed_launch(target):
            # Integrate the publisher's permitted ramp, independently of the
            # guard profile. Keep the guard's existing launch response delay.
            elapsed = np.maximum(0., t - pg.LAUNCH_DEAD_S)
            ramp = np.minimum(elapsed, target / LAUNCH_ACCEL_MPS2)
            return .5 * LAUNCH_ACCEL_MPS2 * ramp ** 2 + target * np.maximum(0., elapsed - target / LAUNCH_ACCEL_MPS2)

        unrestricted = executed_launch(30. / 3.6)
        self.assertAlmostEqual(pg._clearance(np.full_like(t, 20.), -4. + 1.5 * .8 * t,
                                            unrestricted).min(), -.20, delta=.01)
        r = decide(o, 0.)
        self.assertEqual(r["kind"], "yield")
        self.assertFalse(r.get("stopping", False))
        self.assertLess(r["cap_mps"], 30. / 3.6)
        limited = executed_launch(r["cap_mps"])
        np.testing.assert_allclose(pg._profile("speed", 0., t, v_target=r["cap_mps"]), limited, atol=1e-9)
        arrival = pg._first_reach(t, limited + pg.EGO_FRONT_M - 20.)
        self.assertGreaterEqual(arrival, r["timing"][0]["t_exit"] + pg.AFTER_GAP_S - .002)
        for factor in pg.FACTORS:
            self.assertGreater(pg._clearance(np.full_like(t, 20.), -4. + 1.5 * factor * t, limited).min(), 0.)

    def test_review4_comfortable_stop_uses_shrinking_envelope(self):
        o = person(20., 0., 0., .3)
        r = decide(o, 18.)
        self.assertEqual(r["kind"], "yield")
        self.assertTrue(r["stopping"])
        self.assertGreater(r["cap_mps"], 0.)
        self.assertAlmostEqual(r["along"], 13.655)
        self.assertAlmostEqual(r["required_decel_mps2"], 1.12, delta=.01)
        self.assertGreater(r["clearance"], 0.)
        t = np.arange(0., 20., .01)
        # A frozen cap would drive through the pedestrian. The executed cap
        # shrinks with remaining distance, including acceleration and brake lag.
        frozen = pg._profile("speed", 5., t, v_target=r["cap_mps"])
        self.assertLess(pg._clearance(np.full_like(t, 20.), .3 * t, frozen).min(), 0.)
        travel = pg._profile("stop_envelope", 5., t, stop_m=r["along"], v_target=30. / 3.6)
        self.assertAlmostEqual(travel[-1], r["along"], delta=.01)
        self.assertTrue(np.all(np.diff(travel)[travel[:-1] < r["along"] - .01] > 0.))
        self.assertGreater(travel[100], 5.)  # The current cap allows acceleration.
        for factor in pg.FACTORS:
            self.assertGreater(pg._clearance(np.full_like(t, 20.), .3 * factor * t, travel).min(), 0.)

    def test_review4_stop_caps_shrink_and_release_when_person_exits(self):
        caps = []
        # Independent approach states, not a vehicle/environment simulation.
        for remaining, speed in ((13.655, 5.), (10., 4.), (6., 3.), (3., 2.),
                                 (1., 1.), (.2, .3), (.06, .1)):
            with self.subTest(remaining=remaining):
                o = person(remaining + pg.EGO_FRONT_M + pg.STOP_CLEAR_M, 0., 0., .3)
                r = decide(o, speed * 3.6)
                self.assertEqual(r["kind"], "yield")
                self.assertTrue(r["stopping"])
                self.assertGreater(r["cap_mps"], 0.)
                caps.append(r["cap_mps"])
        self.assertTrue(np.all(np.diff(caps) < 0.))
        at_target = decide(person(pg.EGO_FRONT_M + pg.STOP_CLEAR_M, 0., 0., .3), 0.)
        self.assertEqual(at_target["kind"], "stop")
        self.assertEqual(at_target["cap_mps"], 0.)
        gone = decide(person(pg.EGO_FRONT_M + pg.STOP_CLEAR_M, 1.6, 0., .3), 0.)
        self.assertEqual(gone["kind"], "clear")
        self.assertIsNone(gone["cap_mps"])

    def test_review4_braking_profile_remains_feasible_on_replanning(self):
        t = np.arange(0., 20., .01)
        travel = pg._profile("stop_envelope", 5., t, stop_m=13.655, v_target=30. / 3.6)
        speed = np.gradient(travel, .01)
        accel = np.diff(speed) / .01
        self.assertLessEqual(accel.max(), pg.LAUNCH_ACCEL + 1e-7)
        self.assertGreaterEqual(accel.min(), -pg.COMFORT_DECEL - 1e-7)
        self.assertAlmostEqual(travel[50], 2.5)  # Full initial response delay.
        for index in (100, 200, 300, 400):
            with self.subTest(t=t[index]):
                remaining = 13.655 - travel[index]
                o = person(remaining + pg.EGO_FRONT_M + pg.STOP_CLEAR_M, 0., 0., .3)
                r = decide(o, speed[index] * 3.6)
                self.assertEqual(r["kind"], "yield")
                self.assertTrue(r["stopping"])
                self.assertGreater(r["clearance"], 0.)

    def test_review4_ideal_stop_cannot_hide_infeasible_executed_transition(self):
        o = person(12., 0., 0., .4)
        stop = 12. - pg.EGO_FRONT_M - pg.STOP_CLEAR_M
        t = np.arange(0., 20., .01)
        ideal = pg._profile("stopat", 5., t, stop_m=stop)
        self.assertAlmostEqual(ideal[-1], stop)
        actual = pg._profile("stop_envelope", 5., t, stop_m=stop, v_target=30. / 3.6)
        self.assertGreater(actual[-1], stop + 1.)
        r = decide(o, 18.)
        self.assertEqual(r["kind"], "stop")
        self.assertEqual(r["cap_mps"], 0.)
        self.assertGreater(r["clearance"], 0.)
        self.assertIn("exceeds stop point", r["reason"])

    def test_review4_stop_preserves_unknown_hold_and_lower_zone(self):
        class Mixed(Known):
            def state(self, uid):
                return {"state": "unknown", "age": .2} if uid == 1 else super().state(uid)
        unknown = person(6., 2., 0., 0., uid=1)
        crossing = person(20., 0., 0., .3, uid=2)
        t = np.arange(0., 30., .01)
        for zone in (30. / 3.6, 1.):
            with self.subTest(zone=zone):
                r = pg.plan([unknown, crossing], EGO, PATH, 5., zone, Mixed())
                self.assertEqual(r["kind"], "yield")
                self.assertTrue(r["stopping"])
                self.assertLessEqual(r["cap_mps"], 5.)
                actual = pg._profile("stop_envelope", 5., t, stop_m=r["along"], v_target=min(5., zone))
                self.assertLessEqual(np.gradient(actual, .01)[200:].max(), min(5., zone) + 1e-7)
                for factor in pg.FACTORS:
                    self.assertGreater(pg._clearance(np.full_like(t, 20.), .3 * factor * t, actual).min(), 0.)

    def test_review3_pass_checks_deceleration_to_lower_zone(self):
        o = person(6., -3.5, 0., .35)
        t = np.arange(0., 20., .001)
        actual = pg._profile("speed", 2., t, v_target=.5)
        self.assertAlmostEqual(pg._clearance(np.full_like(t, 6.), -3.5 + .35 * t, actual).min(), -.3)
        r = pg.plan([o], EGO, PATH, 2., .5, Known())
        self.assertEqual(r["kind"], "stop")
        self.assertEqual(r["cap_mps"], 0.)

    def test_review3_safe_lower_zone_keeps_pass_or_clear(self):
        t = np.arange(0., 30., .001)
        actual = pg._profile("speed", 2., t, v_target=.5)
        for o, expected in ((person(6., -5., 0., .2), "pass"),
                            (person(pg.EGO_FRONT_M + 10., 0., 0., .7), "clear")):
            with self.subTest(position=o[:2]):
                r = pg.plan([o], EGO, PATH, 2., .5, Known())
                self.assertEqual(r["kind"], expected)
                self.assertIsNone(r["cap_mps"])
                for factor in (*pg.FACTORS, 1.4):
                    self.assertGreater(pg._clearance(np.full_like(t, o[0]), o[1] + o[10] * factor * t, actual).min(), 0.)

    def test_review2_parallel_walker_enters_curved_path_corridor(self):
        x = np.arange(-5., 80., .1)
        u = np.clip((x - 16.) / 9., 0., 1.)
        path = np.column_stack((x, 2.2 * u * u * (3. - 2. * u)))
        o = person(15., 2.2, 1.2, 0.)
        t = np.arange(0., 6., .01)
        frame = pg.PathFrame(path)
        origin = frame.project([[0., 0.]])[0][0]
        al, la, _ = frame.project(np.column_stack((o[0] + o[9] * t, o[1] + o[10] * t)))
        self.assertAlmostEqual(pg._clearance(al - origin, la, 4. * t).min(), -.3)
        r = pg.plan([o], EGO, path, 4., 4., Known())
        self.assertNotEqual(r["kind"], "clear")
        self.assertIn(16, r["ids"])
        self.assertTrue(r["timing"])
        # The 4 m/s zone cannot authorize cruising through the walker. It can
        # authorize a controlled approach whose envelope shrinks to the stop.
        self.assertEqual(r["kind"], "yield")
        self.assertTrue(r["stopping"])
        self.assertGreater(r["clearance"], 0.)
        for ceiling in (np.inf, 4.):
            travel = pg._profile("stop_envelope", 4., t, stop_m=r["along"], v_target=ceiling)
            self.assertGreater(pg._clearance(al - origin, la, travel).min(), 0.)

    def test_review2_straight_shoulder_and_rear_only_predictions_remain_clear(self):
        for o in (person(15., 2.2, 1.2, 0.), person(-1.5, -3., 0., 1.)):
            with self.subTest(position=o[:2]):
                r = pg.plan([o], EGO, PATH, 4., 30. / 3.6, Known())
                self.assertEqual(r["kind"], "clear")
                self.assertIsNone(r["cap_mps"])
                self.assertEqual(r["timing"], [])

    def test_review2_rear_diagonal_walker_is_predicted_when_launching(self):
        o = person(-1.5, -3., 1., 1.)
        t = np.arange(0., 6., .001)
        launch = pg._profile("speed", 0., t, v_target=30. / 3.6)
        # At the actual 2.5 m/s^2 bound the nominal walker clears the body,
        # but faster hypotheses still enter it and must remain in prediction.
        self.assertGreater(pg._clearance(o[0] + t, o[1] + t, launch).min(), 0.)
        for factor in (1.2, 1.4):
            self.assertLess(pg._clearance(o[0] + factor * t, o[1] + factor * t, launch).min(), 0.)
        r = pg.plan([o], EGO, PATH, 0., 30. / 3.6, Known())
        self.assertNotEqual(r["kind"], "clear")
        self.assertIn(16, r["ids"])
        self.assertTrue(r["timing"])
        self.assertEqual(r["kind"], "stop")
        self.assertEqual(r["cap_mps"], 0.)

    def test_review_clear_requires_safe_acceleration_to_zone_speed(self):
        # The front keeps STOP_CLEAR_M from a walk until the walker has left our
        # corridor (user 2026-10-11): their stand-off line, 2.5 m short of the walk,
        # is what the arrival is checked against.
        o = person(15.5 + pg.STOP_CLEAR_M, -1., 0., .7)
        stand_off = o[0] - pg.STOP_CLEAR_M
        t = np.arange(0., 6., .001)
        unrestricted = pg._profile("speed", 2., t, v_target=30. / 3.6)
        self.assertLess(pg._clearance(np.full_like(t, stand_off), o[1] + o[10] * t, unrestricted).min(), 0.)
        r = decide(o, 7.2, zone_kmh=30.)
        self.assertEqual(r["kind"], "yield")
        self.assertIsNotNone(r["cap_mps"])
        self.assertLess(r["cap_mps"], 30. / 3.6)
        for factor in pg.FACTORS:
            selected = pg._profile("speed", 2., t, v_target=r["cap_mps"])
            self.assertGreater(pg._clearance(np.full_like(t, stand_off), o[1] + o[10] * factor * t, selected).min(), 0.)

    def test_review_stop_cap_must_validate_selected_deceleration(self):
        o = person(15., -.4, 0., .7)
        t = np.arange(0., 6., .001)
        hard = pg._profile("hard", 8., t)
        self.assertAlmostEqual(pg._clearance(np.full_like(t, 15.), -.4 + .7 * t, hard).min(), .56, delta=.01)
        unsafe = pg._profile("speed", 8., t, v_target=6.58629919389665)
        self.assertLess(pg._clearance(np.full_like(t, 15.), -.4 + .7 * t, unsafe).min(), 0.)
        r = decide(o, 28.8)
        self.assertGreater(r["required_decel_mps2"], pg.HARD_DECEL)
        self.assertEqual(r["kind"], "stop")
        self.assertEqual(r["cap_mps"], 0.)
        runtime = Runtime.__new__(Runtime)
        applied = runtime.apply_pedestrian_guard(r, None, EGO, None, [o], 0.)
        self.assertEqual(applied["kind"], "stop")

    def test_review_unknown_hold_survives_a_second_crossing_person(self):
        class Mixed(Known):
            def state(self, uid):
                return {"state": "unknown", "age": .2} if uid == 1 else super().state(uid)
        unknown = person(6., 2., 0., 0., uid=1)
        crossing = person(44., 0., 0., .7, uid=2)
        alone = pg.plan([unknown], EGO, PATH, 5., 30. / 3.6, Mixed())
        together = pg.plan([unknown, crossing], EGO, PATH, 5., 30. / 3.6, Mixed())
        self.assertEqual(alone["kind"], "hold")
        self.assertEqual(together["kind"], "hold")
        self.assertEqual(together["cap_mps"], 5.)
        self.assertEqual(together["ids"], {1, 2})
        closer = pg.plan([unknown, person(28., -1., 0., .7, uid=2)], EGO, PATH, 5., 30. / 3.6, Mixed())
        self.assertEqual(closer["kind"], "yield")
        self.assertLess(closer["cap_mps"], together["cap_mps"])
        self.assertFalse(closer.get("stopping", False))

    def test_arrive_after_crossing_without_deceleration(self):
        r = decide(person(pg.EGO_FRONT_M + 10., 0., 0., .7), 7.2, zone_kmh=7.2)
        self.assertEqual(r["kind"], "clear")
        self.assertIsNone(r["cap_mps"])
        timing = r["timing"][0]
        self.assertGreaterEqual(timing["t_arr"], timing["t_exit"] + .6)

    def test_safe_zone_acceleration_still_releases_cap(self):
        o = person(35., 0., 0., .7)
        r = decide(o, 7.2, zone_kmh=30.)
        self.assertEqual(r["kind"], "clear")
        self.assertIsNone(r["cap_mps"])
        t = np.arange(0., 10., .001)
        selected = pg._profile("speed", 2., t, v_target=30. / 3.6)
        for factor in pg.FACTORS:
            self.assertGreater(pg._clearance(np.full_like(t, o[0]), o[1] + o[10] * factor * t, selected).min(), 0.)

    def test_stop_cap_is_checked_even_below_hard_braking_limit(self):
        for forward, speed in ((14., 7.), (18., 9.)):
            with self.subTest(forward=forward, speed=speed):
                r = decide(person(forward, -.4, 0., .7), speed * 3.6)
                self.assertLess(r["required_decel_mps2"], pg.HARD_DECEL)
                self.assertEqual(r["kind"], "stop")
                self.assertEqual(r["cap_mps"], 0.)
                self.assertLess(r["clearance"], 0.)

    def test_largest_feasible_cap_is_about_2_4_when_braking_is_feasible(self):
        # 10 m from the front to the stand-off line STOP_CLEAR_M short of the walk.
        o = person(pg.EGO_FRONT_M + pg.STOP_CLEAR_M + 10., -.4, 0., .7)
        r = decide(o, 10.)
        self.assertEqual(r["kind"], "yield")
        self.assertFalse(r.get("stopping", False))
        self.assertAlmostEqual(r["cap_mps"], 2.4, delta=.03)
        t = np.arange(0., 10., .001)
        target_time = r["timing"][0]["t_exit"] + .6
        arrival = pg._first_reach(t, pg._profile("speed", 10. / 3.6, t, v_target=r["cap_mps"]) - 10.)
        self.assertGreaterEqual(arrival, target_time - .002)
        faster = pg._first_reach(t, pg._profile("speed", 10. / 3.6, t, v_target=r["cap_mps"] + .01) - 10.)
        self.assertLess(faster, target_time)

    def test_37_kmh_at_10_metres_cannot_instantly_become_2_4_mps(self):
        # Run-417 brief's illustrative 10 m / 4.1 s ignores braking travel.
        o = person(pg.EGO_FRONT_M + 10., -.4, 0., .7)
        r = decide(o, 37.)
        self.assertTrue(r["stopping"])
        self.assertEqual(r["kind"], "stop")
        self.assertEqual(r["cap_mps"], 0.)
        self.assertGreater(r["required_decel_mps2"], pg.HARD_DECEL)
        t = np.arange(0., 8., .001)
        arrival = pg._first_reach(t, pg._profile("speed", 37. / 3.6, t, v_target=2.) - 10.)
        self.assertLess(arrival, r["timing"][0]["t_exit"] + .6)

    def test_pass_ahead_uses_rear_bumper_and_0_8_second_gap(self):
        r = decide(person(10., -3.5, 0., .7), 37.)
        self.assertEqual(r["kind"], "pass")
        self.assertIsNone(r["cap_mps"])
        self.assertIn(16, r["pass_ids"])
        self.assertLessEqual(r["timing"][0]["t_rear"], r["timing"][0]["t_enter"] - .8)

    def test_cap_recovers_each_frame_and_clears_as_person_exits(self):
        walk = pg.EGO_FRONT_M + pg.STOP_CLEAR_M + 10.
        a = decide(person(walk, -.4, 0., .7), 10.)
        b = decide(person(walk, -.2, 0., .7), 10.)
        c = decide(person(walk, 1.6, 0., .7), 10.)
        self.assertGreater(b["cap_mps"], a["cap_mps"])
        self.assertEqual(c["kind"], "clear")
        self.assertIsNone(c["cap_mps"])

    def test_stop_profile_reaches_exact_target_without_early_zero(self):
        r = decide(person(12., 0., 0., .4), 18.)
        self.assertTrue(r["stopping"])
        self.assertAlmostEqual(r["along"], 12. - pg.EGO_FRONT_M - 2.5)
        t = np.arange(0., 20., .001)
        travel = pg._profile("stopat", 5., t, stop_m=r["along"])
        self.assertAlmostEqual(travel[-1], r["along"], places=6)
        self.assertTrue(np.all(np.diff(travel)[travel[:-1] < r["along"] - .001] > 0.))
        short = pg._profile("stopat", 10., t, stop_m=.01)
        self.assertAlmostEqual(short[-1], .01, places=6)
        near = decide(person(pg.EGO_FRONT_M + 2.5, 0., 0., .4), .1)
        self.assertEqual(near["kind"], "stop")
        self.assertEqual(near["cap_mps"], 0.)

    def test_uncertainty_interval_interpolates_entry_and_exit(self):
        r = decide(person(35., -3., 0., .8), 30.)
        timing = r["timing"][0]
        self.assertAlmostEqual(timing["t_enter"], (3. - pg.CORRIDOR_HALF_M) / (.8 * 1.2))
        self.assertAlmostEqual(timing["t_exit"], (3. + pg.CORRIDOR_HALF_M) / (.8 * .8))

    def test_1_4_factor_is_used_only_within_8_metres(self):
        original = pg._clearance
        with patch("ped_guard._clearance", wraps=original) as check:
            decide(person(10., -3.5, 0., .7), 37.)
            factors = {round((call.args[1][1] - call.args[1][0]) / (.7 * pg.STEP_S), 1)
                       for call in check.call_args_list}
            self.assertEqual(factors, set(pg.FACTORS))
            check.reset_mock()
            decide(person(7.5, -3., 0., .7), 37.)
            factors = {round((call.args[1][1] - call.args[1][0]) / (.7 * pg.STEP_S), 1)
                       for call in check.call_args_list}
            self.assertEqual(factors, set(pg.FACTORS) | {1.4})

    def test_stationary_tracks_remain_owned_by_10_second_bypass(self):
        class Standing(Known):
            def state(self, uid):
                return {"state": "standing", "age": 20.}
        # Beside the path: the older rules (and their bypass timer) own them.
        r = pg.plan([person(10., 2.5, 0., 0.)], EGO, PATH, 5., 10., Standing())
        self.assertEqual(r["ids"], set())
        self.assertEqual(r["kind"], "clear")
        # In our path the older rules still see them (not in ids), but the guard
        # stops us STOP_CLEAR_M short of them and waits (user 2026-10-11).
        r = pg.plan([person(10., 0., 0., 0.)], EGO, PATH, 5., 10., Standing())
        self.assertEqual(r["ids"], set())
        self.assertEqual(r["blocking_ids"], {16})
        self.assertEqual(r["kind"], "stop")
        self.assertAlmostEqual(r["along"], 10. - pg.EGO_FRONT_M - pg.STOP_CLEAR_M)

    def test_multiple_crossings_share_a_safe_cap(self):
        objects = [person(20., 0., 0., .7, uid=1), person(35., -3., 0., .8, uid=2)]
        r = pg.plan(objects, EGO, PATH, 5., 10., Known())
        self.assertEqual(r["kind"], "yield")
        self.assertFalse(r.get("stopping", False))
        self.assertGreaterEqual(r["cap_mps"], 2.)
        t = np.arange(0., 20., .001)
        for o, timing in zip(objects, r["timing"]):
            arr = pg._first_reach(t, pg._profile("speed", 5., t, v_target=r["cap_mps"]) + pg.EGO_FRONT_M - o[0])
            self.assertGreaterEqual(arr, timing["t_exit"] + .6 - .002)

    def test_run417_t10_to18_logged_snapshots_respect_physical_limits(self):
        # Logged path-relative along/lat and reported velocity. The original
        # fitted direction/full path were not logged, so this is a transverse
        # projection contract, not a reconstruction of the actual drive.
        rows = [
            (11.699, 13.5, 1.67, 1.29, 31.3),
            (12.205, 12.5, 1.57, .83, 22.4),
            (12.620, 12., 1.38, .73, 12.2),
            (13.071, 11., 1.24, .69, 3.8),
            (13.669, 11., .99, .73, 0.),
            (14.638, 11., .55, .91, 2.4),
            (15.538, 10., .26, .7, 9.4),
            (16.002, 9., 0., .84, 11.6),
            (16.585, 8.5, -.19, .87, 3.6),
            (17.706, 8.5, -.51, .74, 0.),
        ]
        results = [decide(person(along, lat, 0., -speed), kmh)
                   for _, along, lat, speed, kmh in rows]
        self.assertTrue(all(r["kind"] != "reverse" for r in results))
        self.assertTrue(results[0]["stopping"])
        # The walker is in our corridor in every row: no "arrive after" release
        # while they cross (2.6 m/s at t=14.6 before the 2026-10-11 stand-off);
        # each frame approaches a stop point STOP_CLEAR_M short of their walk.
        for (_, along, _, _, _), r in zip(rows, results):
            self.assertTrue(r["kind"] == "stop" or r.get("stopping"), r)
            self.assertLessEqual(r["along"], along - pg.EGO_FRONT_M - pg.STOP_CLEAR_M + 1e-6)
        # Walking on out of the corridor ends it.
        out = decide(person(8.5, -1.3, 0., -.74), 0.)
        self.assertEqual(out["kind"], "clear")
        self.assertIsNone(out["cap_mps"])


class RuntimeTimeTests(unittest.TestCase):
    def test_review5_runtime_keeps_cap_on_unsafe_standstill_launch(self):
        from types import SimpleNamespace
        from test_avoidance_gate import AvoidanceGateTests
        runtime = AvoidanceGateTests().runtime(objects=[person(20., -4., 0., 1.5)])
        runtime.test_status["path"]["nearest_idx"] = 900
        runtime.test_status["ego"]["speed_kmh"] = 0.
        runtime.speed_zones = SimpleNamespace(kmh_at=lambda index: (30., {}))
        with patch("gui.USE_LATTICE", True):
            self.assertTrue(runtime.infer())
        record = runtime._web_record
        self.assertEqual(record["ped_guard"]["kind"], "yield")
        self.assertFalse(record["ped_guard"]["stopping"])
        self.assertEqual(runtime.last_yield["kind"], "follow")
        self.assertFalse(record["emergency_stop"])
        self.assertGreater(record["published_cap_kmh"], 0.)
        self.assertLess(record["published_cap_kmh"], 30.)
        self.assertLessEqual(record["published_cap_kmh"], record["ped_guard"]["cap_kmh"] + .1)

    def test_review4_runtime_approaches_feasible_stop_without_emergency(self):
        from test_avoidance_gate import AvoidanceGateTests
        runtime = AvoidanceGateTests().runtime(objects=[person(20., 0., 0., .3)])
        runtime.test_status["path"]["nearest_idx"] = 900
        runtime.test_status["ego"]["speed_kmh"] = 18.
        with patch("gui.USE_LATTICE", True):
            self.assertTrue(runtime.infer())
        record = runtime._web_record
        self.assertEqual(record["ped_guard"]["kind"], "yield")
        self.assertTrue(record["ped_guard"]["stopping"])
        self.assertEqual(runtime.last_yield["kind"], "follow")
        self.assertFalse(record["emergency_stop"])
        self.assertGreater(record["published_cap_kmh"], 0.)

    def test_review3_runtime_brakes_for_unsafe_lower_zone_pass(self):
        from types import SimpleNamespace
        from test_avoidance_gate import AvoidanceGateTests
        runtime = AvoidanceGateTests().runtime(objects=[person(6., -3.5, 0., .35)])
        runtime.test_status["path"]["nearest_idx"] = 900
        runtime.test_status["ego"]["speed_kmh"] = 7.2
        runtime.speed_zones = SimpleNamespace(kmh_at=lambda index: (1.8, {}))
        with patch("gui.USE_LATTICE", True):
            self.assertTrue(runtime.infer())
        record = runtime._web_record
        self.assertEqual(record["ped_guard"]["kind"], "stop")
        self.assertEqual(runtime.last_yield["kind"], "stop")
        self.assertTrue(record["emergency_stop"])
        self.assertEqual(record["published_cap_kmh"], 0.)

    def test_review3_unrelated_stander_does_not_block_pass_latch_release(self):
        from test_avoidance_gate import AvoidanceGateTests
        moving, standing = person(10., -3.5, 0., .7), person(35., 5., 0., 0., uid=17)
        runtime = AvoidanceGateTests().runtime(objects=[moving, standing])
        runtime.test_status["path"]["nearest_idx"] = 900
        runtime.test_status["ego"]["speed_kmh"] = 37.
        runtime.last_yield = dict(kind="stop", name="pedestrian_guard:p")
        runtime._yield_stop_until = 10.5
        runtime._pedestrian_hold_until = 10.5
        with patch("gui.time.monotonic", return_value=10.), patch("gui.USE_LATTICE", True):
            self.assertTrue(runtime.infer())
        record = runtime._web_record
        self.assertEqual(record["ped_guard"]["kind"], "pass")
        self.assertIsNone(runtime.last_yield)
        self.assertEqual(runtime._yield_stop_until, 0.)
        self.assertEqual(runtime._pedestrian_hold_until, 0.)
        self.assertFalse(record["emergency_stop"])
        self.assertGreater(record["published_cap_kmh"], 0.)

    def test_review2_clear_and_active_pedestrian_hold_preserve_vehicle_stop(self):
        from test_avoidance_gate import AvoidanceGateTests
        vehicle_stop = dict(kind="stop", name="crossing_car", source="traffic",
                            along=8., t=.5, v_mps=0., reason="vehicle collision")
        for hold in (False, True):
            with self.subTest(hold=hold):
                runtime = AvoidanceGateTests().runtime(objects=[person(13.845, 1.6, 0., .7)])
                runtime.test_status["path"]["nearest_idx"] = 900
                runtime.test_status["ego"]["speed_kmh"] = 10.
                runtime.last_yield = dict(kind="stop", name="pedestrian_guard:p")
                runtime._pedestrian_hold_until = 11. if hold else 0.
                with patch("gui.time.monotonic", return_value=10.), patch("gui.USE_LATTICE", True), \
                        patch.object(runtime.gap_planner, "evaluate", return_value=vehicle_stop):
                    self.assertTrue(runtime.infer())
                record = runtime._web_record
                self.assertEqual(record["ped_guard"]["kind"], "clear")
                self.assertEqual(runtime.last_yield, vehicle_stop)
                self.assertTrue(record["emergency_stop"])
                self.assertEqual(record["published_cap_kmh"], 0.)
                self.assertEqual(runtime._pedestrian_hold_until, 0.)

    def test_review_runtime_blocks_unsafe_zone_acceleration(self):
        from test_avoidance_gate import AvoidanceGateTests
        runtime = AvoidanceGateTests().runtime(objects=[person(15.5 + pg.STOP_CLEAR_M, -1., 0., .7)])
        runtime.test_status["path"]["nearest_idx"] = 900
        runtime.test_status["ego"]["speed_kmh"] = 7.2
        with patch("gui.USE_LATTICE", True):
            self.assertTrue(runtime.infer())
        record = runtime._web_record
        self.assertEqual(record["yield"]["kind"], "follow")
        self.assertAlmostEqual(record["published_cap_kmh"], 2.269 * 3.6, delta=.1)
        self.assertFalse(record["emergency_stop"])

    def test_review_runtime_uses_emergency_brake_for_unsafe_stop_cap(self):
        from test_avoidance_gate import AvoidanceGateTests
        runtime = AvoidanceGateTests().runtime(objects=[person(15., -.4, 0., .7)])
        runtime.test_status["path"]["nearest_idx"] = 900
        runtime.test_status["ego"]["speed_kmh"] = 28.8
        with patch("gui.USE_LATTICE", True):
            self.assertTrue(runtime.infer())
        record = runtime._web_record
        self.assertEqual(record["yield"]["kind"], "stop")
        self.assertTrue(record["emergency_stop"])
        self.assertEqual(record["published_cap_kmh"], 0.)

    def test_runtime_publishes_ttc_cap_and_recovers_on_next_frame(self):
        from test_avoidance_gate import AvoidanceGateTests
        walk = 13.845 + pg.STOP_CLEAR_M      # 10 m from the front to the stand-off line
        runtime = AvoidanceGateTests().runtime(objects=[person(walk, -.4, 0., .7)])
        runtime.test_status["path"]["nearest_idx"] = 900
        runtime.test_status["ego"]["speed_kmh"] = 10.
        with patch("gui.USE_LATTICE", True):
            self.assertTrue(runtime.infer())
        before = runtime._web_record
        self.assertEqual(before["yield"]["kind"], "follow")
        self.assertAlmostEqual(before["published_cap_kmh"], 2.4 * 3.6, delta=.1)
        self.assertFalse(before["emergency_stop"])
        self.assertEqual(before["lattice"]["target"], 0.)
        runtime.object_receiver.objects = [person(walk, 1.6, 0., .7)]
        with patch("gui.USE_LATTICE", True):
            self.assertTrue(runtime.infer())
        after = runtime._web_record
        self.assertIsNone(after["yield"])
        self.assertFalse(after["emergency_stop"])
        self.assertGreater(after["published_cap_kmh"], before["published_cap_kmh"])

    def test_lattice_wait_only_in_corridor_and_forward_10_metres(self):
        runtime = Runtime.__new__(Runtime)
        for f, lat, expected in ((10., 0., True), (10.01, 0., False),
                                 (5., pg.CORRIDOR_HALF_M, True),
                                 (5., pg.CORRIDOR_HALF_M + .001, False), (-1., 0., False)):
            with self.subTest(f=f, lat=lat):
                self.assertEqual(runtime.pedestrian_lattice_wait([person(f, lat, 0., .7)], EGO, PATH), expected)

    def test_guard_recovery_releases_pedestrian_latch_but_preserves_box_stop(self):
        runtime = Runtime.__new__(Runtime)
        runtime._yield_stop_until = 10.
        runtime._pedestrian_hold_until = 10.
        runtime.last_yield = dict(kind="stop", name="pedestrian_guard:p")
        o = person(13.845, 1.6, 0., .7)
        r = decide(o, 10.)
        self.assertIsNone(runtime.apply_pedestrian_guard(r, None, EGO, None, [o], 1.))
        self.assertEqual(runtime._yield_stop_until, 0.)
        box = dict(kind="stop", name="box")
        runtime._yield_stop_until = 10.
        runtime.last_yield = box
        self.assertEqual(runtime.apply_pedestrian_guard(r, box, EGO, None, [o], 1.), box)
        self.assertEqual(runtime._yield_stop_until, 10.)

    def test_guard_preserves_an_independent_stationary_pedestrian_stop(self):
        runtime = Runtime.__new__(Runtime)
        runtime._yield_stop_until = 10.
        moving, standing = person(10., -3.5, 0., .7), person(15., 0., 0., 0., uid=17)
        r = decide(moving, 37.)
        stop = dict(kind="stop", name="pedestrian", v_mps=0.)
        self.assertEqual(runtime.apply_pedestrian_guard(r, stop, EGO, None, [moving, standing], 1.), stop)
        self.assertEqual(runtime._yield_stop_until, 10.)

    def test_review3_stander_on_curved_path_still_blocks_latch_release(self):
        runtime = Runtime.__new__(Runtime)
        runtime._yield_stop_until = 10.
        runtime.last_yield = dict(kind="stop", name="pedestrian_guard:p")
        moving, standing = person(10., -3.5, 0., .7), person(35., 5., 0., 0., uid=17)
        r = decide(moving, 37.)
        curved = PATH.copy()
        curved[:, 1] = 5. * np.clip((curved[:, 0] - 20.) / 15., 0., 1.)
        stop = dict(kind="stop", name="pedestrian", v_mps=0.)
        self.assertEqual(runtime.apply_pedestrian_guard(r, stop, EGO, None, [moving, standing], 1.,
                                                      path_local=curved), stop)
        self.assertEqual(runtime._yield_stop_until, 10.)

    def test_review3_unowned_moving_person_still_blocks_latch_release(self):
        runtime = Runtime.__new__(Runtime)
        runtime._yield_stop_until = 10.
        runtime.last_yield = dict(kind="stop", name="pedestrian_guard:p")
        moving, unowned = person(10., -3.5, 0., .7), person(35., 5., 0., -.7, uid=17)
        r = decide(moving, 37.)
        stop = dict(kind="stop", name="pedestrian", v_mps=0.)
        self.assertEqual(runtime.apply_pedestrian_guard(r, stop, EGO, None, [moving, unowned], 1.,
                                                      path_local=PATH), stop)
        self.assertEqual(runtime._yield_stop_until, 10.)


if __name__ == "__main__":
    unittest.main()
