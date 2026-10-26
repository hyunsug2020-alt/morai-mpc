"""Common-entrance lane preference and the lane-edge standstill escape
(E2E run follow-20261009T183356Z: overtook into the 11 deg right merge lane 22 s
before the entrance, yielded to a stop there, re-entered at an angle, ended 0.8 m
over the lane-edge margin and stood until the time limit)."""
import math
import unittest
from types import SimpleNamespace

import numpy as np

from highway_planner import HighwayPlanner
from highway_three_lane import HighwayLane
from lane_corridor import LaneCorridor
from planning.highway.lane_map import HighwayLaneMap, direction_over
from planning.highway.policy import corridor_overrun, inward_recovery_clearance


class JoinAngleIsNotALanePreference(unittest.TestCase):
    """All three lanes stay free near the common entrance.

    The 700 m kink preference covered the whole 680 m highway and started 7 of
    24 changes in run follow-20261009T185440Z (one into a lane with an
    unknown-speed car 44 m ahead: 85 -> 29 km/h). Join angle did not predict the
    entrance speed: right-lane (11.3 deg) entries passed at 64-71 km/h in live
    runs 171938Z/174200Z, a left-lane (5.5 deg) entry slowed to 12 km/h."""
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        self.config = dict(from_idx=0, to_idx=1800, kmh=90.,
            lane_change_policy="free_three_lane", lane_change_goal=0, end_link="exit")
        self.planner = HighwayPlanner(self.config, self.route)
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=10.)

    def lanes(self, kinks, merge_m):
        return [HighwayLane(str(k), k, self.route+np.array([0., k*3.5]), (str(k), "exit"), merge_m,
                            entry_kink_deg=kink) for k, kink in enumerate(kinks)]

    @staticmethod
    def car(x, y, speed, uid=1):
        return (x, y, 4.9, "lidar_car", "npc", speed, 4.9, 2.1, 0., speed, 0., uid, 0., None, True)

    def decide(self, lanes, objects=(), stamp=10.):
        return self.planner.decide(0, self.route[:640], dict(self.ego, stamp=stamp), objects, {},
            lambda p, h: h, lambda p, h: h, perception_ready=True, lane_options=lanes)

    def test_overtake_into_a_sharper_join_is_allowed_near_the_entrance(self):
        lanes = self.lanes((5.5, 9.5, 11.3), 600.)
        self.decide(lanes, [self.car(110., 0., 8.)], stamp=10.)
        result = self.decide(lanes, [self.car(110., 0., 8.)], stamp=10.6)
        self.assertEqual(result["status"]["target_lane"], "1")
        self.assertEqual(result["status"]["entry_kink_deg"], 5.5)

    def test_no_change_toward_an_aligned_lane_without_a_speed_gain(self):
        lanes = self.lanes((11.3, 9.5, 5.5), 600.)
        for stamp in (10., 10.6, 11.2):
            result = self.decide(lanes, stamp=stamp)
            self.assertEqual(result["status"]["target_lane"], "0")
        self.assertIsNone(self.planner.saved_world)
        self.assertNotIn("merge_lane_preferred", result["status"])


class EntryKink(unittest.TestCase):
    def test_kink_is_measured_where_the_lane_joins_the_entrance(self):
        straight = np.column_stack((np.zeros(31), -np.arange(0., 31.)))
        angled = np.column_stack((np.linspace(-6., 0., 31), -np.arange(0., 31.)))
        entrance = np.column_stack((np.zeros(31), -np.arange(30., 61.)))
        lane_map = HighwayLaneMap.__new__(HighwayLaneMap)
        lane_map.exit_link, lane_map.entry_kinks = "exit", {}
        lane_map.lanes = SimpleNamespace(geometry={"a": straight, "b": angled, "exit": entrance})
        self.assertAlmostEqual(lane_map.entry_kink_deg(("a", "exit")), 0., places=3)
        self.assertAlmostEqual(lane_map.entry_kink_deg(("b", "exit")), math.degrees(math.atan2(6., 30.)), places=1)
        self.assertEqual(lane_map.entry_kink_deg(("exit",)), 0.)
        self.assertAlmostEqual(float(direction_over(straight, 10., False) @ np.array([0., -1.])), 1.)


class LaneEdgeStandstill(unittest.TestCase):
    """One 3.5 m lane on y=0 with LaneCorridor's body model (corners, 0.2 m margin):
    a body centred 0.604 m off the centre touches the margin, 0.804 m the painted edge."""

    def setUp(self):
        route = np.column_stack((np.arange(-20., 200., .5), np.zeros(440)))
        self.corridor = LaneCorridor(route, np.full(440, 1.75), np.full(440, -1.75))
        self.centre = np.column_stack((np.arange(0., 200., .5), np.zeros(400)))

    @staticmethod
    def join(start_y, end_y, length=12.):
        x = np.arange(0., 60.5, .5)
        u = np.clip(x/length, 0., 1.)
        return np.column_stack((x, start_y+(end_y-start_y)*u**3*(10.-15.*u+6.*u*u)))

    def test_inward_join_from_over_the_margin_is_accepted(self):
        path = self.join(.70, 0.)
        # Turning inward swings the rear corner out first: the plain check stops it.
        self.assertEqual(self.corridor.clear_distance(path, 170.), 1.)
        clearance = inward_recovery_clearance(path, self.centre, self.corridor.clear_distance)
        self.assertIsNotNone(clearance)
        self.assertGreaterEqual(clearance(path, 170.), 12.)
        stations, overrun = corridor_overrun(self.corridor, path, 6.)
        self.assertLessEqual(overrun.max(), min(overrun[0]+.1, .2))   # body inside the painted edge
        other = self.join(.70, .70)
        self.assertEqual(clearance(other, 170.), self.corridor.clear_distance(other, 170.))

    def test_outward_unprogressing_or_edge_crossing_joins_are_refused(self):
        refused = [self.join(.70, 1.), self.join(.70, .65),
                   self.join(.75, 0.),            # its swing would cross the painted edge
                   self.join(.85, 0., 16.)]       # already beyond it: grows by more than 3 cm
        for path in refused:
            with self.subTest(start=path[0, 1], end=path[-1, 1]):
                self.assertIsNone(inward_recovery_clearance(path, self.centre, self.corridor.clear_distance))
        # Without LaneCorridor's body model the corner swing cannot be bounded.
        self.assertIsNone(inward_recovery_clearance(self.join(.70, 0.), self.centre,
                                                    lambda path, horizon: self.corridor.clear_distance(path, horizon)))

    def test_longer_join_escapes_closer_to_the_edge_but_a_closed_road_still_stops(self):
        self.assertIsNone(inward_recovery_clearance(self.join(.80, 0.), self.centre,
                                                    self.corridor.clear_distance))
        config = dict(from_idx=0, to_idx=1800, kmh=90., lane_change_policy="free_three_lane",
                      lane_change_goal=0, end_link="exit")
        route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        closed = np.where(route[:, 0] < 10., 1.75, 0.)    # the lane ends 10 m ahead
        for offset, left, escapes in ((.80, np.full(2000, 1.75), True), (.70, closed, False)):
            with self.subTest(offset=offset, closed=not escapes):
                planner = HighwayPlanner(config, route)
                ego = dict(x=0., y=offset, heading_deg=0., speed_kmh=0., stamp=10.)
                lane = HighwayLane("0", 0, route-[0., offset], ("0", "exit"), 800.)
                corridor = LaneCorridor(route-[0., offset], left, -left)
                result = planner.decide(0, route[:640]-[0., offset], ego, [], {}, corridor.clear_distance,
                                        lambda p, h: h, perception_ready=True, lane_options=[lane])
                self.assertEqual(result['status']['alignment_escape_join'], escapes)
                final = (result.get('map_clearance') or corridor.clear_distance)(result['path'], 170.)
                if escapes:
                    self.assertEqual(final, 170.)
                    self.assertAlmostEqual(result['target_mps']*3.6, 4.)
                else:
                    self.assertLess(final, 12.)
                    self.assertLess(planner.plan_speed(result['path'], ego, [], result['target_mps'],
                                                       static_clear_m=final)['reference_mps'], .1)


if __name__ == "__main__":
    unittest.main()
