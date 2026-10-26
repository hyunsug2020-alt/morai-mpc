"""Guardrail S-curve racing line: lattice base_offset and the offline line data."""
import json
import unittest
from pathlib import Path

import numpy as np

from lane_corridor import LaneCorridor
from lattice_planner import plan

HERE = Path(__file__).resolve().parent


class RacingBaseOffsetTests(unittest.TestCase):
    def setUp(self):
        n = 241
        self.route = np.column_stack((np.arange(0., 120.01, .5), np.zeros(n)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=45.)
        # Two same-direction lanes: own (centre 0) and the right one (centre -3.5).
        self.corridor = LaneCorridor(self.route, np.full(n, 1.75), np.full(n, -5.25))
        self.road = LaneCorridor(self.route, np.full(n, 1.75), np.full(n, -5.25))
        s = self.route[:, 0]
        # Smooth move into the right lane between 20 m and 70 m.
        u = np.clip((s - 20.) / 50., 0., 1.)
        self.base = -3.0 * u ** 3 * (10. - 15. * u + 6. * u * u)

    def run_plan(self, objects=(), points=None, **kwargs):
        return plan(self.route, self.ego, points, list(objects), self.corridor,
                    road_corridor=self.road, cruise_mps=55. / 3.6, **kwargs)

    def test_empty_road_follows_the_racing_line(self):
        result = self.run_plan(base_offset=self.base)
        self.assertFalse(result["blocked"])
        self.assertTrue(result["racing"])
        self.assertEqual(result["reason"], "racing line")
        path = result["path"]
        far = np.flatnonzero(path[:, 0] > 80.)[0]
        self.assertAlmostEqual(float(path[far, 1]), -3.0, delta=.05)
        self.assertAlmostEqual(result["speed_cap_mps"], 55. / 3.6, places=3)

    def test_without_base_offset_lane_centre_is_unchanged(self):
        result = self.run_plan()
        self.assertFalse(result["racing"])
        self.assertEqual(result["reason"], "lane centre")
        self.assertLess(float(np.max(np.abs(result["path"][:, 1]))), 1e-6)

    def test_obstacle_in_lane_uses_the_route_reference(self):
        box = (30., 0., 1., "box", "obstacle", 0., 1., 1., 0.)
        with_base = self.run_plan([box], base_offset=self.base)
        without = self.run_plan([box])
        self.assertFalse(with_base["racing"])
        self.assertEqual(with_base["target"], without["target"])
        np.testing.assert_allclose(with_base["path"], without["path"])

    def test_racing_line_over_the_centre_line_falls_back_to_lane_centre(self):
        # A (wrong) line 2 m left would cross the yellow centre line: rejected,
        # the plain lane-centre candidate is driven instead of stopping.
        result = self.run_plan(base_offset=-self.base * 2. / 3.)
        self.assertFalse(result["blocked"])
        self.assertFalse(result["racing"])
        self.assertLess(float(np.max(result["path"][:, 1])), .3)

    def test_join_from_lane_centre_onto_an_offset_line_is_gentle(self):
        # Line already 3 m right at the car (e.g. re-enabled mid-curve).
        result = self.run_plan(base_offset=np.full(len(self.route), -3.))
        self.assertTrue(result["racing"])
        path = result["path"]
        self.assertLess(abs(float(path[np.flatnonzero(path[:, 0] >= 10.)[0], 1])), 1.)
        self.assertLess(result["curvature"], .03)

    def test_same_line_keeps_the_previous_plan_while_the_car_drifts(self):
        # Run 227: the car 0.5 m left of its plan; the plan must not move with it.
        first = self.run_plan(base_offset=self.base)
        self.ego = dict(self.ego, y=.5)
        route = self.route - [0., .5]          # route in the drifted car's frame
        kept = plan(route, self.ego, None, [], self.corridor, road_corridor=self.road,
                    cruise_mps=55. / 3.6, base_offset=self.base, anchor_offsets=first["offsets"],
                    anchor_same_line=True)
        self.assertTrue(kept["stitched"])
        np.testing.assert_allclose(kept["offsets"], first["offsets"])
        fresh = plan(route, self.ego, None, [], self.corridor, road_corridor=self.road,
                     cruise_mps=55. / 3.6, base_offset=self.base)
        self.assertGreater(abs(float(fresh["offsets"][10] - first["offsets"][10])), .3)

    def test_plan_far_from_the_car_is_not_kept(self):
        first = self.run_plan(base_offset=self.base)
        route = self.route - [0., 1.2]
        result = plan(route, dict(self.ego, y=1.2), None, [], self.corridor, road_corridor=self.road,
                      cruise_mps=55. / 3.6, base_offset=self.base, anchor_offsets=first["offsets"],
                      anchor_same_line=True)
        self.assertIsNone(result["anchor_m"])
        self.assertFalse(result["stitched"])

    def test_gentle_join_only_toward_the_guardrail(self):
        line = np.full(len(self.route), -3.)
        gentle = self.run_plan(base_offset=line, join_alat_mps2=1.2)
        away = self.run_plan(base_offset=line, join_alat_mps2=1.2, gentle_side=1)
        left = np.full(len(self.route), .5)    # toward the (left) guardrail: still gentle
        toward = self.run_plan(base_offset=left, join_alat_mps2=1.2, gentle_side=1)
        toward_plain = self.run_plan(base_offset=left, join_alat_mps2=1.2)
        at = lambda r, x: abs(float(r["path"][np.flatnonzero(r["path"][:, 0] >= x)[0], 1]))
        self.assertGreater(at(away, 20.), at(gentle, 20.) + .2)     # right, away: the usual join
        np.testing.assert_allclose(toward["path"], toward_plain["path"])


class RacingLineDataTests(unittest.TestCase):
    def test_offline_line_stays_in_the_two_lanes_and_is_smooth(self):
        payload = json.loads((HERE / "racing_line.json").read_text())
        offsets = np.asarray(payload["offsets_m"])
        self.assertEqual(payload["first_idx"] + len(offsets) - 1, payload["last_idx"])
        self.assertTrue(1270 <= payload["first_idx"] < payload["last_idx"] <= 1720)
        # Never left of the own-lane centre (guardrail side), never past the 2nd lane.
        self.assertLessEqual(float(offsets.max()), 1e-6)
        self.assertGreaterEqual(float(offsets.min()), -3.75)
        # Joins the route at both ends and has no kinks (0.5 m spacing).
        self.assertLess(abs(offsets[0]) + abs(offsets[-1]), .02)
        self.assertLess(float(np.max(np.abs(np.diff(offsets, 2)))), .01)


class GuardrailLineTests(unittest.TestCase):
    """Run 203 touched the guardrail (idx ~1510, +0.65 m drift at 48 km/h)."""

    def setUp(self):
        self.payload = json.loads((HERE / "racing_line.json").read_text())

    def arrays(self, *names):
        return [np.asarray(self.payload[name], dtype=float) for name in names]

    def test_lines_keep_the_guardrail_clearance_with_the_measured_drift(self):
        guard, drift, racing, wide = self.arrays("guard_m", "drift_design_m", "offsets_m", "safe_wide_m")
        inner = slice(70, len(guard) - 80)       # past the 30 m entry/exit ramps (junction speeds)
        for line in (racing, wide):
            worst = guard - (line + .946 + .1) - drift
            self.assertGreaterEqual(float(worst[inner].min()), .8)

    def test_lane_bounded_line_stays_in_lane_one_and_is_speed_capped_where_short(self):
        guard, drift, lane, cap = self.arrays("guard_m", "drift_design_m", "safe_lane_m", "lane_vcap_kmh")
        self.assertGreaterEqual(float(lane.min()), -1.75 + .946 + .2)   # 0.25 m inside the lane line
        self.assertLessEqual(float(lane.max()), 1e-6)
        inner = slice(70, len(guard) - 80)
        short = (guard - (lane + .946 + .1) - drift)[inner] < float(self.payload["lane_worst_m"])
        self.assertTrue(short.any())
        self.assertTrue(np.all(cap[inner][short] < 999.))

    def test_guardrail_line_label_in_the_planner(self):
        n = 241
        route = np.column_stack((np.arange(0., 120.01, .5), np.zeros(n)))
        corridor = LaneCorridor(route, np.full(n, 1.75), np.full(n, -5.25))
        result = plan(route, dict(x=0., y=0., heading_deg=0., speed_kmh=45.), None, [], corridor,
                      road_corridor=corridor, cruise_mps=58. / 3.6, base_offset=np.full(n, -.8),
                      base_label="guardrail line", join_alat_mps2=1.2)
        self.assertEqual(result["reason"], "guardrail line")
        self.assertFalse(result["racing"])
        self.assertEqual(result["base_line"], "guardrail line")
        self.assertAlmostEqual(float(result["path"][-1, 1]), -.8, delta=.05)


if __name__ == "__main__":
    unittest.main()
