"""Tunnel static semantics remove false departure caps, never physical checks."""
import time
import unittest
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from driving_plan import DrivingPlan
from lane_corridor import LaneCorridor
import test_live_path_safety as live_fixture
from test_ped_guard import person
import test_tunnel_integration as tunnel_fixture
from tunnel_tracks import static_body
import gui
from tunnel_map import TunnelMap
from model_avoidance import tunnel_route_bounds


class TunnelStaticSemanticTests(unittest.TestCase):
    def check(self, *, scope=lambda pose: True, raw=(), name="fusion_pedestrian"):
        runtime = live_fixture.LivePathSafetyTests().attach(
            tunnel_fixture.TunnelIntegrationTests().runtime(half_width=20.))
        ego = dict(runtime.test_status["ego"], speed_kmh=35.)
        runtime.test_status["ego"], runtime.test_odometry = ego, dict(ego)
        route = np.column_stack((np.arange(0., 80., .5), np.zeros(160)))
        runtime.tunnel.active = scope
        runtime.tunnel.project_scan = Mock(return_value=dict(valid=True,
            obstacle_points=np.asarray(raw, float).reshape(-1, 2)))
        runtime._live_tunnel_corridor = dict(ego=ego, wall_stats={},
            corridor=LaneCorridor(route, np.full(len(route), 9.), np.full(len(route), -9.)))
        obj = list(person(12., 2.2, 0., 0., uid=46))
        obj[3] = name
        runtime.object_receiver.objects = [tuple(obj)]
        runtime.vehicle_receiver = Mock()
        runtime.vehicle_receiver.snapshot.return_value = ([], True,
            dict(stamp=runtime.test_scan["stamp"], age_s=.02))
        plan = DrivingPlan(1, route, ego, ego, runtime.test_scan["stamp"], .01,
            time.monotonic(), 35./3.6, 60., False,
            dict(valid=True, pose_valid=True, steering_valid=True), "TUNNEL")
        result = runtime.path_publisher._safety_provider(route, ego, "TUNNEL", plan)
        self.assertEqual(runtime.object_receiver.objects, [tuple(obj)])
        return result

    def test_false_semantic_person_beside_clear_bypass_keeps_speed(self):
        with patch("live_path_safety.static_body", side_effect=lambda obj, **kw: static_body(obj, keep_person=True)):
            before = self.check()
        self.assertLess(before.allowed_mps*3.6, 35.)
        self.assertIn("fusion_pedestrian", before.stop_reason)
        result = self.check()
        self.assertAlmostEqual(result.allowed_mps*3.6, 35.)
        self.assertFalse(result.emergency)
        self.assertEqual(result.validation["tunnel_static_semantic_ids"], [46])

    def test_portal_and_outside_obstacle_keep_existing_person_guard(self):
        # Ego outside/object inside, then ego inside/object outside.
        for scope in (lambda pose: pose["x"] > 10., lambda pose: pose["x"] < 2.):
            outside = self.check(scope=scope)
            self.assertLess(outside.allowed_mps*3.6, 35.)
            self.assertIn("fusion_pedestrian", outside.stop_reason)
            self.assertEqual(outside.validation["tunnel_static_semantic_ids"], [])

    def test_static_semantics_never_remove_actual_lidar_contact(self):
        raw = [(x, y) for x in (3., 3.2, 3.4, 3.6) for y in (-.3, 0., .3)]
        result = self.check(raw=raw)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)
        self.assertIn("lidar", result.stop_reason)

    def test_cyclist_name_cannot_restore_human_departure_for_tunnel_static_body(self):
        result = self.check(name="fusion_cyclist")
        self.assertAlmostEqual(result.allowed_mps*3.6, 35.)
        self.assertFalse(result.emergency)

    def test_speed_changes_only_inside_surveyed_tunnel(self):
        root = Path(__file__).parent
        route = np.array([[p["x"], p["y"]] for p in json.loads((root/"global_path.json").read_bytes())["waypoints"]])
        tunnel = TunnelMap(json.loads((root/"tunnel_wall_map.json").read_bytes()))
        self.assertEqual(tunnel_route_bounds(route, tunnel.active), gui.TUNNEL_STATIC_SPEED_IDX)
        new_zones = json.loads((root/"web_speed_zones.json").read_bytes())
        old_zones = [z for z in new_zones if z.get("note") != "tunnel static corridor"]

        def zones(rows):
            result = gui.SpeedZones.__new__(gui.SpeedZones)
            result.base, result.route_len = 55., len(route)
            result.zones, result.loaded = rows, time.monotonic()
            result.file_cache = SimpleNamespace(value=rows)
            result.set_route(route)
            return result

        old, new = zones(old_zones), zones(new_zones)
        previous_dynamic = gui.with_tunnel_zone([(480, 800, 40.), (800, 1100, 30.), (3700, 4030, 40.)])
        lo, hi = gui.TUNNEL_STATIC_SPEED_IDX
        for index in range(len(route)):
            if not lo <= index <= hi:
                self.assertAlmostEqual(old.braking_kmh(index, previous_dynamic)[0],
                                       new.braking_kmh(index, gui.DYNAMIC_ZONES)[0], places=6)
        self.assertAlmostEqual(new.braking_kmh(3900, gui.DYNAMIC_ZONES)[0], 58.)


if __name__ == "__main__":
    unittest.main()
