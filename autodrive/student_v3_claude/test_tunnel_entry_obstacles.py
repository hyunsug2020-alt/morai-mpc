"""Keep measured portal obstacles in the active planner before tunnel takeover."""
from concurrent.futures import Future
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

import gui
import test_avoidance_gate as gate_fixture
import test_intersection_runtime as intersection_fixture
from planner_transition import PlannerTransition, PROPOSAL_MAX_AGE_S
from tunnel_map import TunnelMap


class TunnelEntryObstacleTests(unittest.TestCase):
    INDEX = 3748
    RAW = np.array([(23.1, -.2), (23.1, 0.), (23.1, .2)], np.float32)

    def setUp(self):
        for name, value in (("USE_LATTICE", True),
                            ("STATIC_MISSION_IDX", [(480, 1100), (3760, 4030)])):
            override = patch.object(gui, name, value)
            override.start()
            self.addCleanup(override.stop)

    @staticmethod
    def box(ego):
        return (ego["x"] + 23.1, ego["y"], 1., "box", "obstacle", 0., 1., 1., 0.)

    def configure_entry(self, runtime, distance=20., lateral=0., index=INDEX):
        ego = runtime.test_status["ego"]
        ego["y"] = lateral
        runtime.test_status["path"]["nearest_idx"] = index
        runtime.tunnel = TunnelMap(dict(
            origin_xy=[ego["x"] + distance, 0.], axis_heading_deg=0.,
            left_offset_m=5., right_offset_m=-5.,
            longitudinal_min_m=0., longitudinal_max_m=60.))
        runtime.waypoint_xy = np.column_stack((
            ego["x"] + (np.arange(index + 201) - index) * .5,
            np.full(index + 201, lateral)))
        executor = SimpleNamespace(submit=Mock(
            side_effect=AssertionError("test unexpectedly started a preview worker")))
        transition = runtime._planner_transition = PlannerTransition(executor)
        transition.future = Future()
        transition.future_target = "TUNNEL"
        transition.info = dict(state="PREPARING", target="TUNNEL")
        runtime.infer_tunnel = Mock(side_effect=AssertionError("premature tunnel takeover"))
        runtime.path_publisher.publish_tunnel_constraint = Mock()
        return runtime

    def runtime(self, *, raw=True, semantic=False, **geometry):
        runtime = gate_fixture.AvoidanceGateTests().runtime(
            action="KEEP", points=self.RAW if raw else ())
        self.configure_entry(runtime, **geometry)
        if semantic:
            runtime.object_receiver.objects = [self.box(runtime.test_status["ego"])]
        runtime.model = Mock(wraps=runtime.model)
        return runtime

    @staticmethod
    def proposal(runtime, now, *, ready=True, expired=False):
        ego = runtime.test_status["ego"]
        world = runtime.test_route + [ego["x"], ego["y"]]
        return dict(target="TUNNEL", ready=ready, takeover_ready=False,
                    reason="checked preview" if ready else "preview blocked",
                    received=now - PROPOSAL_MAX_AGE_S - .1 if expired else now,
                    generation=runtime._planner_transition.generation,
                    seed=dict(stamp=now, target=0., world_path=world))

    def step(self, runtime, now=100., *, allow_preview=False):
        transition = runtime._planner_transition
        if not allow_preview:
            transition.last_submit = now
        runtime.lidar.received = runtime.object_receiver.received = now
        count = len(runtime.path_publisher.waypoints_pub.items)
        with patch("gui.time.monotonic", return_value=now), \
                patch("gui.lattice_plan", wraps=gui.lattice_plan) as planner:
            self.assertTrue(runtime.infer())
        self.assertEqual(len(runtime.path_publisher.waypoints_pub.items), count + 1)
        runtime.infer_tunnel.assert_not_called()
        runtime.path_publisher.publish_tunnel_constraint.assert_not_called()
        transition.executor.submit.assert_not_called()
        self.assertIsNone(transition.owner)
        return planner

    def assert_moving_bypass(self, runtime, planner, *, raw=True, semantic=False):
        self.assertTrue(planner.called)
        points, objects = planner.call_args_list[0].args[2:4]
        self.assertEqual(0 if points is None else len(points), len(self.RAW) if raw else 0)
        self.assertEqual(sum(obj[3] == "box" for obj in objects), int(semantic))
        if raw:
            np.testing.assert_allclose(points, self.RAW)
        record = runtime._web_record
        self.assertEqual(record["mode"], "LATTICE")
        self.assertGreater(abs(record["applied_peak_m"]), 1.)
        self.assertFalse(record["lattice"]["blocked"])
        self.assertFalse(record["emergency_stop"])
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertLessEqual(runtime.path_publisher.velocity_pub.items[-1].data, 28.94 / 3.6 + 1e-6)
        payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        path = np.array([(point["x"], point["y"]) for point in payload["waypoints"]])
        self.assertGreater(np.abs(path[:, 1]).max(), 1.)
        self.assertGreaterEqual(runtime.tunnel.path_clearance(
            path, runtime.test_status["ego"], 60.), 55.)
        self.assertTrue(runtime._transition_entry_obstacles)
        self.assertTrue(runtime._transition_entry_corridor)
        runtime.model.assert_called()

    def test_pending_preparation_keeps_raw_obstacle_in_the_actual_lattice(self):
        runtime = self.runtime()
        self.assertFalse(gui.in_obstacle_mission(self.INDEX))
        self.assert_moving_bypass(runtime, self.step(runtime))
        self.assertFalse(runtime._planner_transition.future.done())

    def test_pending_preparation_keeps_semantic_box_without_raw_points(self):
        runtime = self.runtime(raw=False, semantic=True)
        self.assert_moving_bypass(runtime, self.step(runtime), raw=False, semantic=True)
        self.assertGreater(runtime._web_record["lattice"]["obstacle_points"], 0)

    def test_ready_unready_failed_and_expired_previews_keep_measured_obstacles(self):
        for raw, semantic in ((True, False), (False, True)):
            runtime = self.runtime(raw=raw, semantic=semantic)
            transition = runtime._planner_transition
            for number, state in enumerate(("pending", "ready", "unready", "failed", "expired")):
                with self.subTest(raw=raw, semantic=semantic, state=state):
                    now = 100. + number * .1
                    transition.future = Future()
                    if state == "failed":
                        transition.future.set_exception(ValueError("preview worker failed"))
                    elif state != "pending":
                        transition.future.set_result(self.proposal(
                            runtime, now, ready=state != "unready", expired=state == "expired"))
                    self.assert_moving_bypass(runtime, self.step(runtime, now),
                                              raw=raw, semantic=semantic)
                    if state == "ready":
                        self.assertEqual(transition.info["state"], "READY")
                    elif state == "failed":
                        self.assertIn("preview failed", transition.info["reason"])

    def test_snapshot_failure_cannot_drop_active_entry_obstacles(self):
        runtime = self.runtime()
        runtime._planner_transition.future = None
        with patch.object(PlannerTransition, "_snapshot",
                          side_effect=ValueError("next sensor warming")) as snapshot:
            self.assert_moving_bypass(runtime, self.step(runtime, allow_preview=True))
        snapshot.assert_called_once()
        self.assertIn("snapshot unavailable", runtime._planner_transition.info["reason"])

    def test_entry_window_includes_sixty_metres(self):
        runtime = self.runtime(distance=60.)
        self.assert_moving_bypass(runtime, self.step(runtime))

    def test_outside_window_or_another_road_keeps_static_geometry_excluded(self):
        for geometry in (dict(distance=60.5), dict(lateral=260.), dict(lateral=17.)):
            with self.subTest(geometry=geometry):
                runtime = self.runtime(semantic=True, **geometry)
                planner = self.step(runtime)
                points, objects = planner.call_args_list[0].args[2:4]
                self.assertIsNone(points)
                self.assertEqual(objects, [])
                self.assertFalse(runtime._transition_entry_obstacles)
                self.assertFalse(runtime._transition_entry_corridor)
                self.assertEqual(runtime._web_record["applied_peak_m"], 0.)
                self.assertFalse(runtime._web_record["emergency_stop"])
                self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_leaving_the_window_clears_previously_enabled_entry_flags(self):
        runtime = self.runtime(semantic=True)
        self.assert_moving_bypass(runtime, self.step(runtime), semantic=True)
        runtime.test_status["ego"]["x"] = -40.5
        runtime.test_status["path"]["nearest_idx"] = self.INDEX - 81
        runtime.object_receiver.objects = [self.box(runtime.test_status["ego"])]
        planner = self.step(runtime, 100.1)
        self.assertIsNone(planner.call_args_list[0].args[2])
        self.assertEqual(planner.call_args_list[0].args[3], [])
        self.assertFalse(runtime._transition_entry_obstacles)
        self.assertFalse(runtime._transition_entry_corridor)

    def test_blocked_entry_escape_and_tight_retry_keep_physical_wall_checks(self):
        runtime = self.runtime()
        runtime.test_status["ego"]["speed_kmh"] = 0.
        runtime.lidar.pts = np.array([(8., y) for y in np.arange(-6., 6.1, .1)], np.float32)
        with patch("gui.LANE_ESCAPE_ON_BLOCK", True):
            planner = self.step(runtime)
        self.assertEqual(len(planner.call_args_list), 3)
        self.assertEqual(planner.call_args_list[1].kwargs["outside_max_m"], 1.)
        self.assertEqual(planner.call_args_list[2].kwargs["side_margin_m"], .12)
        unsafe = runtime.test_route.copy()
        unsafe[:, 1] = 4.8
        for call in planner.call_args_list:
            guard = call.kwargs.get("extra_clearance")
            self.assertTrue(callable(guard))
            self.assertLess(guard(unsafe, 40.), 30.)
            self.assertEqual(guard(runtime.test_route, 40.), 40.)
        self.assertTrue(runtime._web_record["lattice"]["blocked"])
        self.assertLess(runtime.path_publisher.velocity_pub.items[-1].data, 28.94 / 3.6)

    def test_nearby_portal_does_not_replace_highway_dispatch(self):
        runtime = self.runtime(index=3490)
        runtime.highway = SimpleNamespace(active=lambda index: True,
            config=dict(to_idx=3500), phase="CRUISE")
        runtime._planner_transition.prepared = self.proposal(runtime, 100.)
        runtime.receiver.snapshot = Mock(side_effect=AssertionError("highway read normal cameras"))
        runtime.model = Mock(side_effect=AssertionError("highway ran normal model"))

        def publish_highway(status, ego):
            runtime.path_publisher.zone_mps = 28.94 / 3.6
            return runtime.path_publisher.publish(ego, runtime.test_route, runtime.test_route,
                dict(mode="HIGHWAY", reason="highway owns control", applied_peak_m=0.))

        with patch.object(runtime, "infer_highway_only", side_effect=publish_highway) as highway:
            planner = self.step(runtime)
        highway.assert_called_once()
        planner.assert_not_called()
        runtime.receiver.snapshot.assert_not_called()
        runtime.model.assert_not_called()
        self.assertFalse(runtime._transition_entry_obstacles)
        self.assertFalse(runtime._transition_entry_corridor)
        self.assertEqual(runtime.path_publisher.mode_pub.items[-1].data, "HIGHWAY")

    def test_junction_keeps_full_stop_gap_policy_and_static_exclusion(self):
        runtime = self.configure_entry(intersection_fixture.IntersectionRuntimeTests().runtime())
        runtime.lidar.pts = self.RAW.copy()
        runtime.object_receiver.objects = [self.box(runtime.test_status["ego"])]
        for number in range(16):
            planner = self.step(runtime, 100. + number / 15.)
            self.assertIsNone(planner.call_args_list[0].args[2])
            self.assertEqual(planner.call_args_list[0].args[3], [])
            self.assertEqual(runtime._web_record["applied_peak_m"], 0.)
            if number < 15:
                self.assertTrue(runtime._web_record["emergency_stop"])
                self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        self.assertEqual(runtime._web_record["stop_line"]["state"], "ENTER")
        self.assertFalse(runtime._web_record["emergency_stop"])
        runtime.test_cars = [intersection_fixture.IntersectionRuntimeTests.car(
            runtime.test_status["ego"]["x"] + 8., 8., vy=-8., heading=90.)]
        self.step(runtime, 101.1)
        # Released = committed (user rule 2026-10-10): a car crossing ahead only slows
        # us behind it; no stop rule, no new gap decision.
        traffic = runtime._web_record['traffic_plan']
        self.assertEqual(traffic['reason'], 'junction committed: follow the car ahead')
        self.assertFalse(runtime._web_record["emergency_stop"])
        self.assertLess(runtime.path_publisher.velocity_pub.items[-1].data, traffic['target_mps'] + 1e-6)
        self.assertLess(traffic['target_mps'], 8.)


if __name__ == "__main__":
    unittest.main()
