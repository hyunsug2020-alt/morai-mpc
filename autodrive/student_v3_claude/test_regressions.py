"""Offline regressions for gap entry and static adjacent-lane bypass."""
import json
import unittest
import gui
import tempfile
import threading
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from gui import (HdMapLanes, RosObjectReceiver, RosPathPublisher, build_forced_bypass, continue_static_plan, crossing_caution,
                 blocked_static_stop_cap_mps, local_xy, plan_static_bypass, smooth_control_route, static_approach_cap_mps,
                 recover_static_prefix, static_clear_distance, static_speed_cap_mps, moving_conflict,
                 pedestrian_velocity)
from lane_corridor import LaneCorridor
from traffic_planner import GapPlanner


class DrivingRegressions(unittest.TestCase):
    def test_side_pedestrian_jitter_and_missing_frame_do_not_restart_yield(self):
        receiver = RosObjectReceiver.__new__(RosObjectReceiver)
        receiver.lock = threading.Lock()
        receiver.objects, receiver.received = [], 0.
        receiver._tracks, receiver._motion, receiver._turn, receiver._position_history = {}, {}, {}, {}
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=3.6)
        route = np.column_stack((np.arange(0., 30.5, .5), np.zeros(61)))
        noise = (.00, .04, -.05, .05, -.04, .06, -.03, .03, .00)
        for i, offset in enumerate(noise):
            person = SimpleNamespace(position=SimpleNamespace(x=5., y=2.55 + offset),
                                     size=SimpleNamespace(x=.6, y=.6),
                                     velocity=SimpleNamespace(x=0., y=0.), heading=0.,
                                     unique_id=120, name="fusion_pedestrian")
            message = SimpleNamespace(npc_list=[], pedestrian_list=[person] if i != 5 else [],
                                      obstacle_list=[])
            with patch("gui.time.monotonic", return_value=100. + i * .1):
                receiver.on_objects(message)
            self.assertIsNone(moving_conflict(receiver.objects, ego, route, 1.))
        self.assertGreater(len(receiver._position_history[("pedestrian", 120)]), 4)
        self.assertLess(receiver.objects[0][5], .1)

    def test_measured_slow_pedestrian_approaching_path_still_yields(self):
        samples = [(100. + i * .1, 2., 2.3 - .4 * i * .1) for i in range(9)]
        velocity = pedestrian_velocity(samples)
        self.assertIsNotNone(velocity)
        self.assertAlmostEqual(velocity[1], -.4)
        person = (2., 2.3, .6, "fusion_pedestrian", "pedestrian", .4,
                  .6, .6, -90., *velocity)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        route = np.column_stack((np.arange(0., 30.5, .5), np.zeros(61)))
        self.assertEqual(moving_conflict([person], ego, route, 0.)["kind"], "stop")

    def test_predicted_pedestrian_crossing_behind_rear_does_not_stop_car(self):
        route = np.column_stack((np.arange(0., 30.5, .5), np.zeros(61)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=4.32)
        passed = (-1.6, 3.5, .6, "fusion_pedestrian", "pedestrian", 1.,
                  .6, .6, -90., 0., -1.)
        self.assertIsNone(moving_conflict([passed], ego, route, 1.2))
        # The same motion ahead of the bumper remains a crossing risk.
        approaching = (6.,) + passed[1:]
        self.assertEqual(moving_conflict([approaching], ego, route, 3.)["kind"], "stop")

    def test_run39_merging_lane_keeps_barrel_bypass_available(self):
        assets = Path(__file__).resolve().parent
        world = np.array([[p["x"], p["y"]] for p in
                          json.loads((assets / "global_path.json").read_text())["waypoints"]])
        smoothed = smooth_control_route(world)
        lanes = HdMapLanes(assets / "link_set.json")
        obstacles = [
            (-41.47647, 344.64551, .81, "TrafficBarrel", "obstacle", 0., .81, .81, 7.20764),
            (-21.07001, 346.35645, .81, "TrafficBarrel", "obstacle", 0., .81, .81, 4.74878),
        ]
        snapshots = (
            (-66.69763, 342.00470, 2.35589, 17.4),
            (-56.79234, 342.76920, 7.01950, 13.1),
            (-52.28455, 343.17615, 6.17993, 9.8),
        )
        for x, y, heading, speed in snapshots:
            with self.subTest(x=x):
                ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed)
                nearest = int(np.argmin(np.linalg.norm(world - [x, y], axis=1)))
                route = np.array([local_xy(*point, ego) for point in
                                  smoothed[nearest:nearest + 240]])
                corridor = lanes.corridor(ego, route)
                plan, hazards = plan_static_bypass(
                    route, ego, obstacles,
                    lambda path, tolerance: corridor.clear_distance(path, 25.) >= min(
                        25., np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()) - .3,
                    -1., None, lanes.adjacent_offsets(ego, route), corridor.clear_distance)
                self.assertEqual(len(hazards), 2)
                self.assertFalse(plan.get("blocked"))
                self.assertGreaterEqual(plan["clear_distance_m"], plan["hold_until_m"])
                self.assertGreaterEqual(static_clear_distance(plan["path"], ego, obstacles, 35.), 35.)

    def test_three_column_static_path_publishes_as_ros_xy(self):
        class Sink:
            def __init__(self): self.items = []
            def publish(self, item): self.items.append(item)
        class Message:
            def __init__(self, data=None): self.data = data
        class Pose:
            def __init__(self):
                self.header = None
                self.pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0.))
        class Path:
            def __init__(self): self.header = SimpleNamespace(stamp=None, frame_id=None); self.poses = []
        pub = RosPathPublisher.__new__(RosPathPublisher)
        pub.Float32 = pub.String = Message
        pub.PoseStamped, pub.PathMessage = Pose, Path
        pub.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: 1.))
        pub.waypoints_pub, pub.velocity_pub = Sink(), Sink()
        pub.mode_pub, pub.path_pub, pub.status_pub = Sink(), Sink(), Sink()
        pub.cruise_mps = 25. / 3.6
        route = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        candidate = np.column_stack((route, np.zeros(len(route))))
        ego = dict(x=3., y=4., heading_deg=0.)
        details = dict(mode="STUDENT FORCED", reason="static path", validated_static=True)
        for stamp, emergency in ((100., False), (101., True), (102., False)):
            with patch("gui.time.monotonic", return_value=stamp):
                pub.emergency_stop = emergency
                self.assertTrue(pub.publish(ego, candidate, route, details))
        self.assertGreater(len(pub.path_pub.items[0].poses), 16)
        self.assertEqual(pub.mode_pub.items[0].data, "AVOID")
        self.assertAlmostEqual(pub.path_pub.items[0].poses[0].pose.position.x, 3.)
        self.assertEqual(pub.velocity_pub.items[-2].data, 0.)
        self.assertAlmostEqual(pub.velocity_pub.items[-1].data, gui.LAUNCH_ACCEL_MPS2)
        pub.collision_stop = True
        with patch("gui.time.monotonic", return_value=103.):
            self.assertTrue(pub.publish(ego, candidate, route, details))
        self.assertAlmostEqual(pub.velocity_pub.items[-1].data, 2 * gui.LAUNCH_ACCEL_MPS2)
        self.assertAlmostEqual(pub._last_target_mps, 2 * gui.LAUNCH_ACCEL_MPS2)

    def test_run21_static_stops_can_use_shared_same_direction_lane(self):
        assets = Path(__file__).resolve().parent
        world = np.array([[p["x"], p["y"]] for p in
                          json.loads((assets / "global_path.json").read_text())["waypoints"]])
        smoothed = smooth_control_route(world)
        lanes = HdMapLanes(assets / "link_set.json")
        lane = lanes.link_by_id["A2256W000451"]
        self.assertIsNotNone(lanes.adjacent_link(lane, 1))
        self.assertIsNone(lanes.adjacent_link(dict(lane, road_id="other"), 1))
        self.assertIsNone(lanes.adjacent_link(dict(lane, lane_mark_left=[]), 1))
        self.assertIsNone(lanes.adjacent_link(dict(lane, opp_traffic=True), 1))
        snapshots = (
            ((-46.41, 98.42), 16.6, [(-31.674, 98.906, 1.2, "WoodBox", "obstacle", 0., 1.2, 1.2, -18.58)]),
            ((-60.01, 342.14), 21.6, [(-43.1, 342.34, 1.65, "Bicycle_city", "obstacle", 0., 1.65, .66, 0.)]),
            ((70.52, -372.7), 0., [(70.177, -379.847, .75, "CargoBox", "obstacle", 0., .75, .75, -73.37),
                                    (72.080, -404.506, .366, "Bollard01", "obstacle", 0., .366, .366, -63.80)]),
        )
        for xy, speed, obstacles in snapshots:
            with self.subTest(obstacle=obstacles[0][3]):
                nearest = int(np.argmin(np.linalg.norm(world - xy, axis=1)))
                heading = world[nearest + 6] - world[nearest]
                ego = dict(x=xy[0], y=xy[1], heading_deg=np.degrees(np.arctan2(heading[1], heading[0])),
                           speed_kmh=speed)
                route = np.array([local_xy(*point, ego) for point in smoothed[nearest:nearest + 240]])
                corridor = lanes.corridor(ego, route)
                adjacent = lanes.adjacent_offsets(ego, route)
                self.assertGreater(max(adjacent.values()), 3.)
                plan, hazards = plan_static_bypass(
                    route, ego, obstacles,
                    lambda path, tolerance: corridor.clear_distance(path, 25.) >= min(
                        25., np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()) - .3,
                    adjacent_offsets=adjacent, corridor_clearance=corridor.clear_distance)
                self.assertTrue(hazards)
                self.assertFalse(plan.get("blocked"))
                if plan["adjacent_lane"]:
                    self.assertIn(int(plan["direction"]), adjacent)
                self.assertGreaterEqual(plan["clear_distance_m"], plan["safe_horizon_m"])
                self.assertEqual(plan["cluster_count"], len(obstacles))
                complete_group = max(plan["safe_horizon_m"], plan["hold_until_m"])
                self.assertGreaterEqual(static_clear_distance(plan["path"], ego, obstacles, complete_group),
                                        complete_group-.3)
                self.assertGreaterEqual(corridor.clear_distance(plan["path"], complete_group), complete_group-.3)
                self.assertLessEqual(np.degrees(np.arctan(3.90*plan["curvature"])), 35.+1e-5)
                cap = static_speed_cap_mps(28.94/3.6, plan)
                self.assertLessEqual(cap, plan["steering_rate_cap_mps"]+1e-6)
                self.assertLess(np.linalg.norm(route-plan["path"][-1, :2], axis=1).min(), .01)

    def test_blocked_static_stop_envelope_keeps_clearance(self):
        self.assertEqual(blocked_static_stop_cap_mps(4.), 0.)
        self.assertLess(blocked_static_stop_cap_mps(8.), blocked_static_stop_cap_mps(12.))
        self.assertAlmostEqual(blocked_static_stop_cap_mps(12.) ** 2 / (2 * 1.5) + 4., 12.)

    def test_run22_keeps_adjacent_lane_through_bicycle_rejoin(self):
        assets = Path(__file__).resolve().parent
        world = np.array([[p["x"], p["y"]] for p in
                          json.loads((assets / "global_path.json").read_text())["waypoints"]])
        smoothed = smooth_control_route(world)
        lanes = HdMapLanes(assets / "link_set.json")
        bicycle = (-60.634586, -46.773731, 1.65, "Bicycle_city", "obstacle", 0., 1.65, .66, 95.294)
        for x, y, heading, speed in ((-56.65, -50.95, 92.759, 17.8),
                                     (-56.54, -49.26, 99.847, 16.7),
                                     (-56.69, -48.13, 99.974, 9.5)):
            with self.subTest(y=y):
                ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed)
                nearest = int(np.argmin(np.linalg.norm(world - [x, y], axis=1)))
                route = np.array([local_xy(*point, ego) for point in smoothed[nearest:nearest + 240]])
                corridor = lanes.corridor(ego, route)
                plan, hazards = plan_static_bypass(
                    route, ego, [bicycle],
                    lambda path, tolerance: corridor.clear_distance(path, 25.) >= min(
                        25., np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()) - .3,
                    committed_direction=-1, adjacent_offsets=lanes.adjacent_offsets(ego, route),
                    corridor_clearance=corridor.clear_distance)
                self.assertTrue(hazards)
                self.assertFalse(plan.get("blocked"))
                self.assertGreaterEqual(plan["clear_distance_m"], plan["safe_horizon_m"])
        prior = dict(plan, direction=-1)
        self.assertIsNotNone(recover_static_prefix(prior, ego, [bicycle], corridor.clear_distance))
        self.assertIsNone(recover_static_prefix(prior, ego, [bicycle], lambda path, horizon: 2.))

    def test_valid_path_uses_curvature_speed_instead_of_global_obstacle_distance(self):
        plan = dict(curvature=.03, speed_cap_mps=10.)
        self.assertAlmostEqual(static_speed_cap_mps(25./3.6, plan, 8.) * 3.6, 25.)
        self.assertLess(static_speed_cap_mps(25./3.6, None, 8.) * 3.6, 10.)
        self.assertLess(static_speed_cap_mps(25./3.6, dict(plan, curvature=.2)) * 3.6, 11.)

    def test_already_in_adjacent_lane_does_not_create_another_s_bend(self):
        route = np.column_stack((np.arange(0., 80.5, .5), np.full(161, 3.5)))
        path, curvature, _ = build_forced_bypass(route, -1., offset_m=3.5, hold_m=25., transitions=(14.,))
        np.testing.assert_allclose(path[:60, 1], 0., atol=1e-5)
        self.assertLess(curvature, .12)

    def test_connected_obstacles_keep_same_lane_then_rejoin_after_last(self):
        route = np.column_stack((np.arange(0., 80.5, .5), np.zeros(161)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=20.)
        objects = [(20., 0., .6, "first", "obstacle", 0., .6, .6, 0.),
                   (35., 0., .6, "second", "obstacle", 0., .6, .6, 0.)]
        corridor = LaneCorridor(route, np.full(161, 1.75), np.full(161, -5.25))
        old, k, t = build_forced_bypass(route, -1., offset_m=3.5, hold_m=6., transitions=(10.,))
        saved = dict(path=old, direction=-1., offset_m=3.5, hold_m=6., curvature=k,
                     transition_m=t, kind="bypass", adjacent_lane=True, reused=True)
        plan, _ = plan_static_bypass(route, ego, objects, lambda *a: True, -1., saved,
                                     {-1: 3.5}, corridor.clear_distance)
        self.assertFalse(plan.get("blocked"))
        self.assertEqual(plan["cluster_count"], 2)
        self.assertGreaterEqual(plan["hold_until_m"], 38.)
        # Retain the selected side through both boxes; a checked intermediate
        # width can pass the group without an unnecessary full lane change.
        for front in (20., 28., 35.):
            self.assertLess(np.interp(front, plan["path"][:, 0], plan["path"][:, 1]), -1.6)
        self.assertGreaterEqual(static_clear_distance(plan["path"], ego, objects, 50.), 50.)
        self.assertGreaterEqual(corridor.clear_distance(plan["path"], 50.), 50.)
        next_y = float(np.interp(26., plan["path"][:, 0], plan["path"][:, 1]))
        next_ego = dict(ego, x=26., y=next_y)
        saved_world = dict(stamp=10., world_path=plan["path"][:, :2], plan={k:v for k,v in plan.items() if k != "path"})
        continued = continue_static_plan(saved_world, next_ego, 10.1)
        self.assertIsNotNone(continued)
        local_route = route[route[:, 0] >= 26.] - [26., next_y]
        next_corridor = LaneCorridor(local_route, np.full(161, 1.75), np.full(161, -5.25))
        ongoing, _ = plan_static_bypass(local_route, next_ego, objects, lambda *a: True,
                                        -1., continued, {-1: 3.5}, next_corridor.clear_distance)
        self.assertFalse(ongoing.get("blocked"))
        self.assertEqual(ongoing["direction"], -1.)
        self.assertEqual(ongoing["kind"], "bypass")
        self.assertLess(abs(np.interp(5., ongoing["path"][:, 0], ongoing["path"][:, 1])), .3)
        finished, _ = plan_static_bypass(local_route, dict(next_ego, x=48.), [], lambda *a: True,
                                         -1., dict(ongoing, path=local_route), {-1: 3.5}, next_corridor.clear_distance)
        self.assertEqual(finished["kind"], "rejoin")

    def test_distant_conflict_keeps_braking_safe_prefix_but_close_conflict_blocks(self):
        route = np.column_stack((np.arange(0., 80.5, .5), np.zeros(161)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=20.)
        objects = [(18., 0., .6, "barrel", "obstacle", 0., .6, .6, 0.)]
        plan, _ = plan_static_bypass(route, ego, objects, lambda *a: True,
                                     adjacent_offsets={-1: 3.5}, corridor_clearance=lambda p,h: 28.)
        self.assertFalse(plan.get("blocked"))
        self.assertTrue(plan["replan_needed"])
        self.assertGreater(plan["clear_distance_m"], plan["safe_horizon_m"])
        blocked, _ = plan_static_bypass(route, ego, objects, lambda *a: True,
                                        adjacent_offsets={-1: 3.5}, corridor_clearance=lambda p,h: 5.)
        self.assertTrue(blocked["blocked"])

    def test_run23_does_not_commit_partial_bicycle_bypass_before_adjacent_lane(self):
        assets = Path(__file__).resolve().parent
        world = np.array([[p["x"], p["y"]] for p in
                          json.loads((assets / "global_path.json").read_text())["waypoints"]])
        smoothed = smooth_control_route(world)
        lanes = HdMapLanes(assets / "link_set.json")
        bicycle = (-43.16, 343.39, 1.65, "Bicycle_city", "obstacle", 0., 1.65, .66, 23.43)
        snapshots = ((-68.9993, 341.9999, .8472, 17.34),
                     (-66.1088, 342.0311, 3.2054, 16.34))
        plans = []
        for x, y, heading, speed in snapshots:
            ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed)
            nearest = int(np.argmin(np.linalg.norm(world - [x, y], axis=1)))
            route = np.array([local_xy(*point, ego) for point in smoothed[nearest:nearest + 240]])
            corridor = lanes.corridor(ego, route)
            plan, hazards = plan_static_bypass(
                route, ego, [bicycle],
                lambda path, tolerance: corridor.clear_distance(path, 25.) >= min(
                    25., np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()) - .3,
                adjacent_offsets=lanes.adjacent_offsets(ego, route),
                corridor_clearance=corridor.clear_distance)
            self.assertTrue(hazards)
            plans.append(plan)
        self.assertTrue(plans[0]["blocked"])
        self.assertFalse(plans[1].get("blocked"))
        self.assertEqual(plans[1]["direction"], -1.)
        self.assertGreaterEqual(plans[1]["clear_distance_m"], plans[1]["hold_until_m"])
        complete = max(plans[1]["hold_until_m"], plans[1]["safe_horizon_m"])
        self.assertGreaterEqual(static_clear_distance(plans[1]["path"], ego, [bicycle], complete), complete-.3)
        self.assertGreaterEqual(corridor.clear_distance(plans[1]["path"], complete), complete-.3)
        self.assertLessEqual(np.degrees(np.arctan(3.90*plans[1]["curvature"])), 35.+1e-5)
        self.assertLess(np.linalg.norm(route-plans[1]["path"][-1, :2], axis=1).min(), .01)

    def test_run24_adjacent_lane_continues_across_route_link_change(self):
        assets = Path(__file__).resolve().parent
        world = np.array([[p["x"], p["y"]] for p in
                          json.loads((assets / "global_path.json").read_text())["waypoints"]])
        smoothed = smooth_control_route(world)
        lanes = HdMapLanes(assets / "link_set.json")
        for x, y, heading in ((-58.78, -43.12, 94.), (-58.95, -40.50, 96.)):
            with self.subTest(y=y):
                ego = dict(x=x, y=y, heading_deg=heading)
                nearest = int(np.argmin(np.linalg.norm(world - [x, y], axis=1)))
                route = np.array([local_xy(*point, ego) for point in smoothed[nearest:nearest + 240]])
                corridor = lanes.corridor(ego, route)
                self.assertLess(corridor.right[0], -5.)
                rejoin, curvature, _ = build_forced_bypass(
                    route, 0., offset_m=0., hold_m=0., transitions=(10.,))
                self.assertLess(curvature, .16)
                self.assertGreaterEqual(corridor.clear_distance(rejoin, 35.), 35.)

    def test_run25_replans_rejoin_before_standing_pedestrian(self):
        assets = Path(__file__).resolve().parent
        world = np.array([[p["x"], p["y"]] for p in
                          json.loads((assets / "global_path.json").read_text())["waypoints"]])
        ego = dict(x=-51.7821, y=91.8802, heading_deg=43.1838, speed_kmh=13.8361)
        nearest = int(np.argmin(np.linalg.norm(world - [ego["x"], ego["y"]], axis=1)))
        route = np.array([local_xy(*point, ego) for point in
                          smooth_control_route(world)[nearest:nearest + 240]])
        lanes = HdMapLanes(assets / "link_set.json")
        corridor = lanes.corridor(ego, route)
        person = (-37.63, 95.87, .6, "Bicycle_Female1", "pedestrian", 0., .6, .6, -87.9)
        old_path, curvature, transition = build_forced_bypass(
            route, -1., 3.2, 6., transitions=(14.,))
        prior = dict(path=old_path, direction=-1., offset_m=3.2, hold_m=6.,
                     curvature=curvature, transition_m=transition, kind="rejoin", reused=True)
        self.assertLess(static_clear_distance(old_path, ego, [person], 35.), 20.)
        plan, hazards = plan_static_bypass(
            route, ego, [person],
            lambda path, tolerance: corridor.clear_distance(path, 25.) >= min(
                25., np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()) - .3,
            -1., prior, lanes.adjacent_offsets(ego, route), corridor.clear_distance)
        self.assertFalse(hazards)  # Beside the global route, but on the old rejoin path.
        self.assertEqual(plan["kind"], "rejoin")
        self.assertFalse(plan["reused"])
        self.assertGreaterEqual(plan["clear_distance_m"], 35.)
        self.assertIsNone(moving_conflict([person], ego, plan["path"], ego["speed_kmh"] / 3.6))

    def test_run25_committed_bypass_can_recover_in_same_direction_lane(self):
        assets = Path(__file__).resolve().parent
        world = np.array([[p["x"], p["y"]] for p in
                          json.loads((assets / "global_path.json").read_text())["waypoints"]])
        ego = dict(x=74.652, y=-378.162, heading_deg=-64.0364, speed_kmh=6.92)
        nearest = int(np.argmin(np.linalg.norm(world - [ego["x"], ego["y"]], axis=1)))
        route = np.array([local_xy(*point, ego) for point in
                          smooth_control_route(world)[nearest:nearest + 240]])
        lanes = HdMapLanes(assets / "link_set.json")
        strict = lanes.corridor(ego, route)
        recovery = lanes.corridor(ego, route, allow_second_same_direction=True)
        objects = [(70.35, -383.21, .81, "TrafficBarrel", "obstacle", 0., .81, .81, -87.9),
                   (71.29, -399.36, .56, "YellowBarrel", "obstacle", 0., .56, .56, -87.3)]
        adjacent = lanes.adjacent_offsets(ego, route)
        def plan(corridor):
            return plan_static_bypass(
                route, ego, objects,
                lambda path, tolerance: corridor.clear_distance(path, 25.) >= min(
                    25., np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()) - .3,
                1., None, adjacent, corridor.clear_distance)[0]
        self.assertTrue(plan(strict)["blocked"])
        self.assertGreater(recovery.left[0], strict.left[0] + 3.)
        recovered = plan(recovery)
        self.assertFalse(recovered.get("blocked"))
        self.assertGreaterEqual(recovered["clear_distance_m"], recovered["hold_until_m"])
        self.assertGreaterEqual(static_clear_distance(recovered["path"], ego, objects, 35.), 35.)

    def test_lane_corridor_rejects_opposing_lane_and_checks_vehicle_width(self):
        x = np.arange(0., 81., .5)
        def link(name, y, reverse=False, **kw):
            pts = np.column_stack((x, np.full(len(x), y)))
            return dict(idx=name, points=(pts[::-1] if reverse else pts).tolist(), width_start=3.5, width_end=3.5, **kw)
        links = [link("route", 0., can_move_left_lane=True, can_move_right_lane=True,
                      left_lane_change_dst_link_idx="opposite", right_lane_change_dst_link_idx="right"),
                 link("right", -3.5), link("opposite", 3.5, True)]
        with tempfile.TemporaryDirectory() as td:
            p = Path(td)/"links.json"; p.write_text(json.dumps(links))
            route = np.column_stack((x, np.zeros(len(x))))
            corridor = HdMapLanes(p).corridor(dict(x=0., y=0., heading_deg=0.), route)
        np.testing.assert_allclose(corridor.left, 1.75)
        np.testing.assert_allclose(corridor.right, -5.25)
        right, _, _ = build_forced_bypass(route, -1., offset_m=3.5, hold_m=25., transitions=(18.,))
        left, _, _ = build_forced_bypass(route, 1., offset_m=3.5, hold_m=25., transitions=(18.,))
        edge, _, _ = build_forced_bypass(route, 1., offset_m=1., hold_m=25., transitions=(18.,))
        self.assertGreaterEqual(corridor.clear_distance(right, 50.), 50.)
        self.assertLess(corridor.clear_distance(left, 50.), 10.)
        self.assertLess(corridor.clear_distance(edge, 50.), 18.)

    def test_adjacent_lane_successor_is_checked_and_oncoming_successor_rejected(self):
        x = np.arange(0., 80.5, .5)
        route = np.column_stack((x, np.zeros(len(x))))
        path, _, _ = build_forced_bypass(route, -1., 3.5, 60., transitions=(14.,))
        def link(name, start, end, y, source, target, **extra):
            xx = np.arange(start, end + .25, .5)
            return dict(idx=name, points=np.column_stack((xx, np.full(len(xx), y))).tolist(),
                        from_node_idx=source, to_node_idx=target,
                        width_start=3.5, width_end=3.5, **extra)
        links = [link("route_start", 0., 20., 0., "a", "b", can_move_right_lane=True,
                      right_lane_change_dst_link_idx="adjacent"),
                 link("route_end", 20., 80., 0., "b", "z"),
                 link("adjacent", 0., 20., -3.5, "c", "d"),
                 link("next", 20., 80., -3.5, "d", "e")]
        with tempfile.TemporaryDirectory() as td:
            file = Path(td) / "links.json"
            file.write_text(json.dumps(links))
            ego = dict(x=0., y=0., heading_deg=0.)
            self.assertGreaterEqual(HdMapLanes(file).corridor(ego, route).clear_distance(path, 50.), 50.)
            links[-1]["opp_traffic"] = True
            file.write_text(json.dumps(links))
            self.assertLess(HdMapLanes(file).corridor(ego, route).clear_distance(path, 50.), 25.)

    def test_run18_curved_adjacent_lane_and_curb_failure(self):
        assets = Path(__file__).resolve().parent
        world = np.array([[p["x"], p["y"]] for p in json.loads((assets/"global_path.json").read_text())["waypoints"]])
        smoothed = smooth_control_route(world)
        lanes = HdMapLanes(assets/"link_set.json")
        for snapshot in json.loads((assets/"test_data/run18_avoidance.json").read_text()):
            ego = snapshot["ego"]
            at = np.argmin(np.linalg.norm(world - [ego["x"], ego["y"]], axis=1))
            route = np.array([local_xy(*p, ego) for p in smoothed[at:at+240]])
            corridor = lanes.corridor(ego, route)
            plan, _ = plan_static_bypass(route, ego, snapshot["objects"], lambda *a: True,
                adjacent_offsets=lanes.adjacent_offsets(ego, route), corridor_clearance=corridor.clear_distance)
            if snapshot["t"] == 1790699499.:
                # Already pointing out across the curb: no forward shortcut is safe.
                self.assertTrue(plan["blocked"])
            else:
                self.assertFalse(plan.get("blocked"))
                self.assertGreaterEqual(plan["clear_distance_m"], plan["safe_horizon_m"])
                if snapshot["t"] in (1790699410., 1790699574.):
                    complete = max(plan["safe_horizon_m"], plan["hold_until_m"])
                    self.assertGreaterEqual(static_clear_distance(plan["path"], ego, snapshot["objects"], complete),
                                            complete-.3)
                    self.assertGreaterEqual(corridor.clear_distance(plan["path"], complete), complete-.3)
                    self.assertLessEqual(np.degrees(np.arctan(3.90*plan["curvature"])), 35.+1e-5)
                    self.assertGreater(plan["clear_distance_m"], 45.)
                    # A group near the planning horizon can still be alongside
                    # at its last sample. Its later return is checked on replan.
                    self.assertLessEqual(np.linalg.norm(route-plan["path"][-1, :2], axis=1).min(),
                                         abs(plan["offset_m"])+.1)

    def test_static_bypass_brakes_instead_of_reversing_side_at_speed(self):
        route = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        obstacle = (20., 0., .6, "barrel", "obstacle", 0., .6, .6, 0.)
        previous = dict(path=route.copy(), direction=1., offset_m=1.6,
                        hold_m=6., curvature=.1, transition_m=12., kind="bypass")
        def only_right_lane(path, tolerance=1.5):
            return np.min(np.asarray(path)[:, 1]) < -.5
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=25.)
        plan, hazards = plan_static_bypass(route, ego, [obstacle], only_right_lane, 1., previous)
        self.assertTrue(hazards)
        self.assertTrue(plan["blocked"])
        plan_without_saved_path, _ = plan_static_bypass(route, ego, [obstacle], only_right_lane, 1.)
        self.assertTrue(plan_without_saved_path["blocked"])
        self.assertGreater(static_approach_cap_mps(30.), static_approach_cap_mps(15.))
        self.assertLessEqual(static_approach_cap_mps(29.) * 3.6, 18.)
        self.assertLessEqual(static_approach_cap_mps(15.) * 3.6, 13.)

    def test_turning_merge_leader_does_not_trigger_duplicate_crossing_stop(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=10.)
        crossing = (8., -6., 0., "side car", "npc", 3., 4.5, 2., 90., 0., 3., 7, 0.)
        self.assertEqual(crossing_caution([crossing], ego)["kind"], "stop")
        self.assertIsNone(crossing_caution([crossing], ego, {7}))
        self.assertEqual(crossing_caution([crossing, (9., -6., 0., "other car", "npc", 3., 4.5, 2., 90., 0., 3., 8, 0.)], ego, {7})["name"], "crossing_caution:other car")

        route = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        leader = (12., 0., 0., "lead car", "npc", 3., 4.5, 2., 0., 3., 0., 7, 0.)
        planner = GapPlanner()
        planner.evaluate([leader], ego, route, 23. / 3.6, 100.)
        self.assertIn(7, planner.followable_ids)

    def test_distant_crossing_allows_approach_but_near_crossing_stops(self):
        route = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        car = (24., -21., 0., "crossing", "npc", 3., 4.5, 2., 90., 0., 3., 1, 0.)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        planner = GapPlanner()
        planner.state = "WAIT"
        result = planner.evaluate([car], ego, route, 23. / 3.6, 100.)
        self.assertEqual(result["kind"], "follow")
        self.assertGreater(result["v_mps"], 8. / 3.6)

        imminent = (8., -6., 0., "crossing", "npc", 3., 4.5, 2., 90., 0., 3., 1, 0.)
        result = GapPlanner().evaluate([imminent], dict(ego, speed_kmh=10.), route, 23. / 3.6, 100.)
        self.assertEqual(result["kind"], "stop")

    def test_committed_merge_keeps_braking_room_but_stops_for_imminent_crossing(self):
        route = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        old = np.column_stack((np.arange(-4., 60.5, .5), np.zeros(129)))
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(old, axis=0), axis=1))]
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=10.)
        def evaluate(x):
            planner = GapPlanner()
            planner.state = "ENTER"
            planner.merge_path, planner.merge_exit = (old, arc), 25.
            crossing = (x, -7., 0., "crossing", "npc", 4., 4.5, 2., 90., 0., 4., 7, 0.)
            return planner.evaluate([crossing], ego, route, 23. / 3.6, 100.)
        distant = evaluate(15.)
        self.assertEqual(distant["kind"], "follow")
        self.assertEqual(distant["state"], "ENTER")
        self.assertGreater(distant["v_mps"], 0.)
        imminent = evaluate(8.)
        self.assertEqual(imminent["kind"], "stop")

    def test_rear_bollard_does_not_block_empty_adjacent_lane(self):
        # Run 36 stopped at this pose although the mapped right lane was free.
        ego = dict(x=-62.02195, y=22.43919, heading_deg=89.28207, speed_kmh=0.)
        assets = Path(__file__).resolve().parent
        route_world = np.array([[p["x"], p["y"]] for p in
                                json.loads((assets / "global_path.json").read_text())["waypoints"]])
        nearest = int(np.argmin(np.linalg.norm(route_world - [ego["x"], ego["y"]], axis=1)))
        route = np.array([local_xy(x, y, ego) for x, y in route_world[nearest:nearest + 130]])
        lanes = HdMapLanes(assets / "link_set.json")
        map_points = np.vstack(lanes.local_links(ego, 90.))

        def lane_ok(path, tolerance=1.5):
            points = np.asarray(path)[:, :2]
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
            hold = points[(arc >= 8.) & (arc <= 40.)]
            if len(hold) < 3:
                return True
            distances = np.sqrt(((hold[:, None, :] - map_points[None, :, :]) ** 2).sum(axis=2)).min(axis=1)
            return float(np.percentile(distances, 80)) <= tolerance

        objects = [
            (-60.97, 21.1, .366, "rear bollard", "obstacle", 0., .366, .366, 0.),
            (-59.28, 32.22, .56, "first barrel", "obstacle", 0., .56, .56, 0.),
            (-61.5, 40., .56, "second barrel", "obstacle", 0., .56, .56, 0.),
            (-63.37, 53.45, 1.7, "bicycle", "obstacle", 0., 1.7, .65, 0.),
        ]
        adjacent = lanes.adjacent_offsets(ego, route)
        self.assertGreater(adjacent[-1], 3.5)
        plan, _ = plan_static_bypass(route, ego, objects, lane_ok, adjacent_offsets=adjacent)
        self.assertIsNotNone(plan)
        self.assertEqual(plan["direction"], -1)
        self.assertGreaterEqual(static_clear_distance(plan["path"], ego, objects, 25.), 25.)
        self.assertGreaterEqual(plan["clear_distance_m"], plan["hold_until_m"])
        self.assertTrue(lane_ok(plan["path"], 1.5 if plan["adjacent_lane"] else 1.9))
        self.assertLessEqual(np.degrees(np.arctan(3.90*plan["curvature"])), 35.+1e-5)
        cap = static_speed_cap_mps(28.94/3.6, plan)
        self.assertLessEqual(cap, plan["steering_rate_cap_mps"]+1e-6)
        self.assertLess(np.linalg.norm(route-plan["path"][-1, :2], axis=1).min(), .01)


if __name__ == "__main__":
    unittest.main()
