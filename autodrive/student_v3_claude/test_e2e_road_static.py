"""E2E mode (STUDENT_PLANNER=model) normal-road static obstacles: room that opens only
ahead of the car and the approach speed before a bypass exists.

Geometry from follow-20261009T191227Z / 185440Z / 183356Z (route idx 600-800):
  A  a 1.1 m box 30 m ahead, centred (lat 0.15), inside a junction that starts 16 m
     ahead and ends 46 m ahead; single lane (+-1.75 m) before and after.
  B  a box and a parked car 35-38 m ahead (lat 0.2-0.3) beside a right lane that widens
     in from 17.5 m (bound -1.75 m -> -5.5 m at 50 m).
The host's adjacent-lane lookup 8 m ahead of the car found no room for either until the
car was ~16 m away: blocked plans, 39 -> 12 km/h (A) and 34 -> 6..16 km/h (B).
No ROS, simulator or HD map needed: the corridor is the real LaneCorridor footprint check."""
import functools
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import model_avoidance as ma  # noqa: E402
from lane_corridor import LaneCorridor  # noqa: E402
from tunnel_static_portable import static_obstacle as so  # noqa: E402

HOST_CURVATURE = math.tan(math.radians(35.)) / 3.90     # gui.MAX_CONTROL_CURVATURE
ALAT = 3.0                                               # competition STUDENT_BYPASS_ALAT


def box(x, y, length=1.1, width=1.1, name="fusion_obstacle"):
    return (x, y, max(length, width), name, "obstacle", 0., length, width, 0., 0., 0., 7, 0., None)


def route(length=120., step=.5):
    xs = np.arange(0., length + step, step)
    return np.column_stack((xs, np.zeros_like(xs)))


def ego(kmh):
    return dict(x=0., y=0., heading_deg=0., speed_kmh=float(kmh))


def corridor(dense, left_of_s, right_of_s):
    s = np.asarray(dense)[:, 0]
    return LaneCorridor(dense, np.array([left_of_s(v) for v in s]), np.array([right_of_s(v) for v in s]))


def junction_corridor(dense):
    """Encounter A: junction space +-4.5 m from 16 m to 46 m, one lane elsewhere."""
    return corridor(dense, lambda s: 4.5 if 16. <= s <= 46. else 1.75,
                    lambda s: -4.5 if 16. <= s <= 46. else -1.75)


def widening_corridor(dense):
    """Encounter B: a right lane widening in from 17.5 m to its full width at 50 m."""
    return corridor(dense, lambda s: 1.75, lambda s: -min(5.5, 1.75 + max(0., s - 17.5) * 3.75 / 32.5))


def callbacks(lanes):
    def clearance(path, horizon=60.):
        return lanes.clear_distance(path, horizon)

    def lane_ok(path, tolerance=1.5):
        path = np.asarray(path)[:, :2]
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        return lanes.clear_distance(path, length) >= length - .3
    return lane_ok, clearance


def arc(dense):
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(np.asarray(dense)[:, :2], axis=0), axis=1))]


def speed_kmh(plan):
    return 3.6 * min(math.sqrt(ALAT / max(plan["curvature"], 1e-3)), plan["speed_cap_mps"])


class HostLimits(unittest.TestCase):
    """The package runs with the host's curvature limit and footprint test (gui configure)."""

    @classmethod
    def setUpClass(cls):
        cls._saved = (so.MAX_CONTROL_CURVATURE, ma._geometry.MAX_CONTROL_CURVATURE)
        ma.configure(HOST_CURVATURE)

    @classmethod
    def tearDownClass(cls):
        so.MAX_CONTROL_CURVATURE, ma._geometry.MAX_CONTROL_CURVATURE = cls._saved


class RoomAhead(HostLimits):
    def plan(self, kmh, objects, lanes, near, ahead, direction=0.):
        dense = route()
        lane_ok, clearance = callbacks(lanes)
        offsets = ma.road_offsets(near, ahead, arc(dense), lanes.left, lanes.right)
        plan, hazards, _ = ma.plan_bypass(dense, ego(kmh), objects, lane_ok, clearance, direction,
                                          None, offsets, budget_s=5.)
        return plan, hazards, offsets

    def test_junction_box_gets_a_bypass_at_first_sight(self):
        dense, lanes = route(), junction_corridor(route())
        objects = [box(30.5, .15)]
        lane_ok, clearance = callbacks(lanes)
        # The 8 m lookup alone (before the junction): no room, blocked at 40 km/h.
        old, _, _ = ma.plan_bypass(dense, ego(40.), objects, lane_ok, clearance, 0., None, {}, budget_s=5.)
        self.assertTrue(old and old.get("blocked"))
        plan, hazards, offsets = self.plan(40., objects, lanes, {}, {1: 3.2, -1: 3.2})
        self.assertTrue(hazards)
        self.assertFalse(plan.get("blocked"))
        self.assertEqual(plan["kind"], "bypass")
        # Complete through the box and validated to the end of the horizon.
        self.assertGreaterEqual(plan["clear_distance_m"], plan["hold_until_m"])
        self.assertFalse(plan["replan_needed"])
        # The whole checked path keeps the footprint inside the lanes / junction.
        path_length = float(arc(plan["path"])[-1])
        self.assertGreaterEqual(lanes.clear_distance(plan["path"], 60.), min(60., path_length) - .3)
        # Moves over only where the junction begins; faster than the old 14-17 km/h cap.
        self.assertGreater(plan["start_m"], 0.)
        self.assertGreater(speed_kmh(plan), 20.)
        self.assertEqual(set(offsets.open_road_sides), {-1, 1})

    def test_widening_lane_is_used_before_it_is_eight_metres_ahead(self):
        dense, lanes = route(), widening_corridor(route())
        objects = [box(35.5, .17), box(37.5, .3, 4.6, 1.9, "fusion_parked_vehicle")]
        lane_ok, clearance = callbacks(lanes)
        old, _, _ = ma.plan_bypass(dense, ego(34.), objects, lane_ok, clearance, -1., None, {}, budget_s=5.)
        self.assertTrue(old is not None and old.get("blocked"))
        plan, _, _ = self.plan(34., objects, lanes, {}, {-1: 3.48}, direction=-1.)
        self.assertFalse(plan.get("blocked"))
        self.assertEqual(plan["direction"], -1.)
        self.assertFalse(plan["replan_needed"])
        self.assertGreaterEqual(plan["clear_distance_m"], plan["hold_until_m"])
        # A gentle move into the widening lane keeps the zone speed (was 6-16 km/h).
        self.assertGreaterEqual(speed_kmh(plan), 30.)

    def test_lowest_curvature_side_wins(self):
        # Left: junction-like room only 16-30 m (sharp in and out); right: a full lane.
        dense = route()
        lanes = corridor(dense, lambda s: 4.5 if 16. <= s <= 40. else 1.75, lambda s: -5.25)
        plan, _, _ = self.plan(30., [box(30.5, 0.)], lanes, {-1: 3.5}, {1: 3.2})
        self.assertFalse(plan.get("blocked"))
        self.assertEqual(plan["direction"], -1.)

    def test_already_wide_side_keeps_smaller_offset(self):
        # A normal adjacent lane 3.5 m to the left: the 2.2 m open-road offset clears a
        # centred box with less curvature than the lane centre, and a car holding 2.2 m
        # is not sent across to the lane centre.
        dense = route()
        lanes = corridor(dense, lambda s: 5.25, lambda s: -1.75)
        plan, _, offsets = self.plan(40., [box(35., 0.)], lanes, {1: 3.5}, {})
        self.assertEqual(offsets.start_m_by_side, {1: (0.,)})
        self.assertFalse(plan.get("blocked"))
        self.assertAlmostEqual(plan["offset_m"], 2.2)
        self.assertFalse(plan["replan_needed"])

    def test_no_room_beside_the_obstacle_is_not_offered(self):
        dense = route()
        lanes = corridor(dense, lambda s: 1.75, lambda s: -1.75)
        offsets = ma.road_offsets({}, {1: 3.2}, arc(dense), lanes.left, lanes.right)
        self.assertEqual(dict(offsets), {})


class RoadOffsets(unittest.TestCase):
    def test_starts_from_where_the_corridor_widens(self):
        dense = route()
        lanes = junction_corridor(dense)
        offsets = ma.road_offsets({}, {1: 3.2}, arc(dense), lanes.left, lanes.right)
        self.assertEqual(dict(offsets), {1: 3.2})
        starts = offsets.start_m_by_side[1]
        self.assertIsInstance(starts, tuple)
        self.assertEqual(starts[0], 0.)
        self.assertTrue(any(abs(s - 16.) <= .5 for s in starts))      # the junction entry
        self.assertTrue(all(s <= 16.5 for s in starts))
        self.assertEqual(offsets.open_road_sides, (1,))

    def test_near_side_keeps_its_lookup_and_a_start_at_the_car(self):
        dense = route()
        lanes = corridor(dense, lambda s: 5.25, lambda s: -1.75)
        offsets = ma.road_offsets({1: 3.5}, {1: 3.4, -1: 3.0}, arc(dense), lanes.left, lanes.right)
        self.assertEqual(dict(offsets), {1: 3.5})                     # right never widens
        self.assertEqual(offsets.start_m_by_side, {1: (0.,)})


class Package(HostLimits):
    """Scalar or missing starts (the tunnel, plain dict offsets) keep the old search."""

    def test_plain_offsets_unchanged(self):
        dense, lanes = route(), corridor(route(), lambda s: 5.25, lambda s: -1.75)
        lane_ok, clearance = callbacks(lanes)
        plain, _ = so.plan_static_bypass(dense, ego(30.), [box(30., 0.)], lane_ok, 0, None, {1: 3.5}, clearance)
        offsets = ma.Offsets({1: 3.5})
        offsets.start_m_by_side, offsets.open_road_sides = {}, ()
        wrapped, _ = so.plan_static_bypass(dense, ego(30.), [box(30., 0.)], lane_ok, 0, None, offsets, clearance)
        for key in ("direction", "offset_m", "transition_m", "hold_m", "curvature", "clear_distance_m"):
            self.assertEqual(plain[key], wrapped[key])
        self.assertAlmostEqual(plain["offset_m"], 3.5)

    def test_start_values(self):
        self.assertEqual(so.start_values(0.), [0.])
        self.assertEqual(so.start_values((12., 0., 4.)), [0., 4., 12.])


class Hazards(unittest.TestCase):
    def test_footprint_hazards_reach_past_the_engagement(self):
        dense = route()
        far = box(55., 0.)
        self.assertEqual(ma.confirmed_hazards(dense, ego(20.), [far])[0], [])
        self.assertEqual(len(ma.footprint_hazards(dense, ego(20.), [far])), 1)

    def test_object_beside_the_footprint_is_no_hazard(self):
        # Edge 1.35 m from the route centre: clear of the 0.946 m half body + 0.2 m
        # margin (the old 1.4 m approach band braked for it).
        beside = box(20., 1.85, 1., 1.)
        self.assertEqual(ma.footprint_hazards(route(), ego(30.), [beside]), [])
        touching = box(20., 1.55, 1., 1.)
        self.assertEqual(len(ma.footprint_hazards(route(), ego(30.), [touching])), 1)

    def test_confirmed_hazards_accepts_precomputed_footprint(self):
        dense, objects = route(), [box(20., 0.), box(60., 0.)]
        footprint = ma.footprint_hazards(dense, ego(20.), objects)
        self.assertEqual(ma.confirmed_hazards(dense, ego(20.), objects, footprint),
                         ma.confirmed_hazards(dense, ego(20.), objects))


class ApproachSpeed(unittest.TestCase):
    """gui.model_static_speed_cap_mps: the braking envelope, not the 8-25 km/h cap."""

    @classmethod
    def setUpClass(cls):
        import gui
        cls.gui = gui

    def test_no_bypass_yet_only_the_checked_stop_envelope(self):
        gui, cruise = self.gui, 40. / 3.6
        far = gui.model_static_speed_cap_mps(cruise, None, 44.)
        self.assertAlmostEqual(far, math.sqrt(2. * 1.5 * 40.))         # 39.4 km/h
        self.assertLess(gui.static_approach_cap_mps(44.) * 3.6, 25.1)  # the old cap
        self.assertAlmostEqual(gui.model_static_speed_cap_mps(cruise, None, 20.), math.sqrt(2. * 1.5 * 16.))
        self.assertEqual(gui.model_static_speed_cap_mps(cruise, None, 4.), 0.)
        self.assertEqual(gui.model_static_speed_cap_mps(cruise, None, None), cruise)
        self.assertAlmostEqual(gui.model_static_speed_cap_mps(cruise, dict(blocked=True), 20.),
                               math.sqrt(2. * 1.5 * 16.))

    def test_validated_bypass_uses_its_own_caps(self):
        gui, cruise = self.gui, 40. / 3.6
        plan = dict(curvature=.02, speed_cap_mps=9.)
        self.assertEqual(gui.model_static_speed_cap_mps(cruise, plan, 10.),
                         gui.static_speed_cap_mps(cruise, plan))
        self.assertLessEqual(gui.model_static_speed_cap_mps(cruise, plan, 10.), 9.)


class HostWiring(HostLimits):
    """gui.Runtime.model_road_offsets + model_static_plan on a stub host."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        import gui
        cls.gui = gui

    def host(self, lanes_lookup):
        from types import MethodType, SimpleNamespace
        Runtime = self.gui.Runtime
        host = SimpleNamespace(lanes=lanes_lookup, clearance="objects", object_receiver=True,
                               _intersection_vehicle_status=None, v6_latest=None, v6_worker=None,
                               lidar=None)
        host.lead_bypass_active = lambda ego, now: False
        host.pedestrian_wait_expired = lambda now: False
        host.person_blocks = lambda *args: False
        host.remember_static_objects = lambda static, now, active, tracked=(), **kwargs: static
        host.model_road_offsets = MethodType(Runtime.model_road_offsets, host)
        return host

    def test_junction_room_found_beside_the_obstacle(self):
        from types import SimpleNamespace
        dense = route()
        lanes = junction_corridor(dense)

        def adjacent_offsets(ego_pose, route_local):
            # The HD-map lookup samples 8 m into the route it is given.
            local = np.asarray(route_local)[:, :2]
            station = float(local[min(len(local) - 1, 16), 0])
            return {1: 3.2, -1: 3.2} if 16. <= station <= 46. else {}
        host = self.host(SimpleNamespace(adjacent_offsets=adjacent_offsets))
        lane_ok, clearance = callbacks(lanes)
        # A generous planning budget: the test runs at nice 19 beside a drive.
        with patch.object(self.gui, "model_avoidance", ma, create=True), \
                patch.object(ma, "plan_bypass", functools.partial(ma.plan_bypass, budget_s=5.)):
            plan, hazards, context = self.gui.Runtime.model_static_plan(
                host, ego(40.), dense, [box(30.5, .15)], None, lanes, lane_ok, clearance, None,
                adjacent_offsets(ego(40.), dense), 0., True, True, .95)
        self.assertTrue(hazards)
        self.assertFalse(plan.get("blocked"))
        self.assertAlmostEqual(context["hazard_m"], hazards[0]["front"])
        log = host._model_avoidance_log
        self.assertEqual(log["offsets"], {"-1": 3.2, "1": 3.2})
        self.assertFalse(log["blocked"])
        self.assertIsNotNone(log["plan_offset_m"])


if __name__ == "__main__":
    unittest.main()
