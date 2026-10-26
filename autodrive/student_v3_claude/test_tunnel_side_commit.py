"""Tunnel S row (runs 222/225/271): never swap the side of a near obstacle mid-pass.

Run 271, 14 m before drum 30004 at 39 km/h: committed +3.5 m left of the route,
the drum route-relative +2.5..3.1 (matched pose 0.7 m left of truth). The normal
(+-3.5 m) lattice picked a smaller positive target that passed the drum on its
RIGHT -- no "side switch" by target sign -- the car trailed the path and hit it.
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.environ.get("STUDENT_V3_DIR", os.path.dirname(os.path.abspath(__file__))))

import lattice_planner as lp  # noqa: E402
from lane_corridor import LaneCorridor  # noqa: E402


def drum(x, y, r=.3):
    a = np.linspace(0., 2. * np.pi, 24, endpoint=False)
    return np.column_stack((x + r * np.cos(a), y + r * np.sin(a)))


class TunnelSideCommitTests(unittest.TestCase):
    def setUp(self):
        xs = np.arange(0., 90., .5)                # route starts at the car, as the tunnel dense route
        d0 = 3.5                                   # car 3.5 m left of the route
        self.route = np.column_stack((xs, np.full(len(xs), -d0)))
        self.corridor = LaneCorridor(self.route, np.full(len(xs), 8.6), np.full(len(xs), -2.6))
        # beside us (30003), 17 m ahead (inside 2 s at 39 km/h) on the committed side's edge (30004), then the next one
        self.points = np.vstack((drum(4., -d0 + .45), drum(17., -d0 + 3.1), drum(24.4, -d0 - .4)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=39.)
        self.previous = dict(stamp=0., target=3.5, world_path=np.column_stack((xs, np.zeros(len(xs)))))

    def plan(self, max_offset, **extra):
        return lp.plan(self.route, self.ego, self.points, [], self.corridor, self.previous, cruise_mps=11.,
                       max_curvature=.18, outside_max_m=0., edge_margin_m=.25, wide_side_m=1.6, w_lateral=.3,
                       blocked_standoff_m=6., max_offset_m=max_offset, **extra)

    def test_without_commitment_the_planner_crosses_to_the_other_side(self):
        # The pre-fix behaviour (still the ordinary-road default): same-sign target,
        # opposite side of the drum ahead.
        result = self.plan(3.5)
        self.assertFalse(result["blocked"])
        self.assertLess(result["target"], 3.1)

    def test_committed_side_is_kept_brake_then_wide_offset(self):
        result = self.plan(3.5, side_commit_s=2.)
        # Nothing on the committed (left) side within +-3.5 m: brake there, do not cross.
        self.assertTrue(result["blocked"])
        self.assertGreaterEqual(result["target"], 3.1)
        # The tunnel's wide re-plan (with the same previous) then passes on the left.
        wide = self.plan(5., side_commit_s=2.)
        self.assertFalse(wide["blocked"])
        self.assertGreater(wide["target"], 3.1 + .3)

    def test_no_previous_plan_no_commitment(self):
        self.previous = None
        result = self.plan(3.5, side_commit_s=2.)
        self.assertFalse(result["blocked"])


if __name__ == "__main__":
    unittest.main()
