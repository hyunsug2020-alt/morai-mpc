"""E2E mode (STUDENT_PLANNER=model, STUDENT_TUNNEL_POLICY=model): model_avoidance.py,
v6_ext_child.py and their host hooks. No ROS, simulator or GPU needed."""
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import model_avoidance as ma  # noqa: E402
from tunnel_static_portable import TunnelLaneHold  # noqa: E402

PYTHON = sys.executable


def obj(x, y, kind="obstacle", speed=0., name=None, size=1., heading=0., valid=None):
    values = [x, y, size, name or "fusion_" + kind, kind, speed, size, size, heading,
              speed, 0., 7, 0., None]
    if valid is not None:
        values += [valid, 0.]
    return tuple(values)


def straight(length=100., step=.5):
    xs = np.arange(0., length + step, step)
    return np.column_stack((xs, np.zeros_like(xs)))


EGO = dict(x=0., y=0., heading_deg=0., speed_kmh=20.)


class Classification(unittest.TestCase):
    def test_obstacles_split_at_one_metre_per_second(self):
        static, dynamic = ma.split_static([obj(10, 0, speed=.5), obj(10, 0, speed=1.2)])
        self.assertEqual(len(static), 1)
        self.assertEqual(static[0][5], 0.)
        self.assertEqual(len(dynamic), 1)

    def test_people_need_the_standing_gate_and_never_in_the_tunnel(self):
        walker = obj(10, 0, "pedestrian", .1)
        self.assertEqual(ma.split_static([walker], lambda o: False)[0], [])
        static, _ = ma.split_static([walker], lambda o: True)
        self.assertEqual(len(static), 1)
        self.assertEqual(ma.split_static([walker], lambda o: True, in_tunnel=True)[0], [])

    def test_cars_static_only_with_measured_low_speed(self):
        fused = obj(10, 0, "npc", .2, name="fusion_npc")
        valid_slow = obj(10, 0, "npc", .3, name="lidar_car", valid=True)
        unconfirmed = obj(10, 0, "npc", .0, name="lidar_car", valid=False)
        valid_fast = obj(10, 0, "npc", .8, name="lidar_car", valid=True)
        static, dynamic = ma.split_static([fused, valid_slow, unconfirmed, valid_fast])
        self.assertEqual([o[3] for o in static], ["lidar_car"])
        self.assertEqual(len(dynamic), 3)

    def test_wall_like_clusters(self):
        def box(length, width):
            return (0., 0., max(length, width), "c", "obstacle", 0., length, width, 0., 0., 0., 1)
        self.assertTrue(ma.wall_like(box(9., 1.)))
        self.assertTrue(ma.wall_like(box(6.5, .3)))
        self.assertFalse(ma.wall_like(box(2., 1.)))

    def test_lidar_boxes_explained_by_listed_objects_are_dropped(self):
        listed = [obj(20., 0., size=1.)]
        kept = ma.merge_lidar_obstacles(listed, [obj(20.4, 0.2), obj(30., 0.)])
        self.assertEqual([round(b[0]) for b in kept], [30])


class Hazards(unittest.TestCase):
    def test_static_obstacle_on_route_inside_activation(self):
        hazards, engagement = ma.confirmed_hazards(straight(), EGO, [obj(20., 0.)])
        self.assertEqual(len(hazards), 1)
        self.assertGreaterEqual(engagement, 25.)

    def test_far_or_off_route_objects_do_not_confirm(self):
        self.assertEqual(ma.confirmed_hazards(straight(), EGO, [obj(60., 0.)])[0], [])
        self.assertEqual(ma.confirmed_hazards(straight(), EGO, [obj(20., 4.)])[0], [])
        self.assertEqual(ma.confirmed_hazards(straight(), EGO, [])[0], [])


class Direction(unittest.TestCase):
    left = dict(action="LEFT_CHANGE", p=.8)

    def test_running_manoeuvre_keeps_its_side(self):
        self.assertEqual(ma.choose_direction(dict(direction=-1.), [dict(front=10.)], self.left), (-1., "kept"))

    def test_model_side_only_with_a_confirmed_obstacle(self):
        self.assertEqual(ma.choose_direction(None, [dict(front=10.)], self.left), (1., "v6"))
        self.assertEqual(ma.choose_direction(None, [], self.left), (0., "planner"))
        self.assertEqual(ma.choose_direction(None, [dict(front=10.)], dict(action="LEFT_CHANGE", p=.3)),
                         (0., "planner"))
        self.assertEqual(ma.choose_direction(None, [dict(front=10.)], None), (0., "planner"))

    def test_lane_hold_before_the_model(self):
        held = dict(direction=-1., offset_m=2.2)
        self.assertEqual(ma.choose_direction(None, [dict(front=10.)], self.left, held), (-1., "lane_hold"))


class V6Results(unittest.TestCase):
    def runtime(self, **latest):
        record = dict(t=time.time(), ok=True, action="LEFT", p=.9, action_full="LEFT_CHANGE",
                      probabilities={"LEFT_CHANGE": .9, "KEEP_LANE": .1}, path=[[0, 0], [1, 0], [2, 0]],
                      dt=.1, pose=dict(x=0., y=0., heading_deg=0., t_mono=1.))
        record.update(latest)
        return SimpleNamespace(v6_latest=record, v6_worker=None)

    def test_fresh_result(self):
        decision, why = ma.v6_decision(self.runtime())
        self.assertIsNone(why)
        self.assertEqual((decision["action"], decision["p"]), ("LEFT_CHANGE", .9))
        self.assertEqual(ma.model_side(decision), 1.)

    def test_stale_failed_and_short_names(self):
        self.assertIsNone(ma.v6_decision(self.runtime(t=time.time() - 1.))[0])
        self.assertIsNone(ma.v6_decision(self.runtime(ok=False, reason="masks_stale"))[0])
        self.assertIn("masks_stale", ma.v6_decision(self.runtime(ok=False, reason="masks_stale"))[1])
        decision, _ = ma.v6_decision(self.runtime(action_full=None, action="RIGHT", probabilities={}, p=.7))
        self.assertEqual((decision["action"], decision["p"]), ("RIGHT_CHANGE", .7))
        self.assertIsNone(ma.v6_decision(SimpleNamespace(v6_latest=None, v6_worker=None))[0])


class Planner(unittest.TestCase):
    def setUp(self):
        self.route = straight()
        self.clear = lambda path, horizon=60.: horizon

    def test_bypass_on_the_model_side(self):
        plan, hazards, info = ma.plan_bypass(self.route, EGO, [obj(20., 0.)], lambda p, t=1.5: True,
                                             self.clear, committed_direction=1.)
        self.assertTrue(hazards)
        self.assertFalse(plan.get("blocked"))
        self.assertEqual(plan["direction"], 1.)
        self.assertEqual(plan["generator"], "portable")
        self.assertFalse(info["retried_both_sides"])

    def test_wrong_model_side_retries_both_sides(self):
        right_only = lambda path, t=1.5: float(np.max(np.asarray(path)[:, 1])) < .3
        plan, _, info = ma.plan_bypass(self.route, EGO, [obj(20., 0.)], right_only, self.clear,
                                       committed_direction=1.)
        self.assertTrue(info["retried_both_sides"])
        self.assertFalse(plan.get("blocked"))
        self.assertEqual(plan["direction"], -1.)

    def test_blocked_rejoin_with_nothing_ahead_is_no_plan(self):
        previous, _, _ = ma.plan_bypass(self.route, EGO, [obj(20., 0.)], lambda p, t=1.5: True, self.clear,
                                        committed_direction=1.)
        plan, hazards, info = ma.plan_bypass(self.route, EGO, [], lambda p, t=1.5: False, self.clear,
                                             committed_direction=1., previous_plan=previous)
        self.assertIsNone(plan)
        self.assertEqual(hazards, [])
        self.assertTrue(info["blocked_without_hazard"])

    def test_failed_tunnel_hold_still_stops(self):
        plan, _, info = ma.plan_bypass(self.route, EGO, [], lambda p, t=1.5: False, self.clear,
                                       committed_direction=1., defer_rejoin=True, deferred_offset_m=2.2,
                                       deferred_distance_m=50.)
        self.assertTrue(plan and plan.get("blocked"))
        self.assertFalse(info["blocked_without_hazard"])

    def test_budget_bounds_planning_time(self):
        def slow(path, t=1.5):
            time.sleep(.01)
            return False
        started = time.perf_counter()
        ma.plan_bypass(self.route, EGO, [obj(20., 0.)], slow, self.clear, budget_s=.05)
        self.assertLess(time.perf_counter() - started, .5)

    def test_host_curvature_limit_applies(self):
        ma.configure(math.tan(math.radians(35.)) / 3.90)
        try:
            plan, _, _ = ma.plan_bypass(self.route, EGO, [obj(20., 0.)], lambda p, t=1.5: True, self.clear)
            self.assertLessEqual(plan["curvature"], math.tan(math.radians(35.)) / 3.90 + 1e-9)
        finally:
            ma.configure(math.tan(math.radians(35.)) / 3.0)


class Tunnel(unittest.TestCase):
    def test_free_width_becomes_one_open_lane(self):
        offsets = ma.tunnel_offsets({1: 6.0, -1: 1.2})
        self.assertEqual(dict(offsets), {1: 3.5})
        self.assertEqual(offsets.open_road_sides, (1,))

    def test_route_scope_from_the_host_tunnel_test(self):
        route = straight(200., 1.)
        bounds = ma.tunnel_route_bounds(route, lambda ego: 30. <= ego["x"] <= 150.)
        self.assertEqual(bounds, (30, 150))
        scope, model = ma.make_tunnel_policies(route, bounds)
        self.assertFalse(scope.state(29)["tunnel_active"])
        self.assertTrue(scope.state(40)["defer_rejoin"])
        self.assertTrue(scope.state(120)["rejoin_phase"])
        self.assertTrue(model.active(100))

    def test_reference_time_to_ros_stamp(self):
        samples = [dict(received=10. + .02 * i, stamp=500. + .02 * i) for i in range(50)]
        self.assertAlmostEqual(ma.stamp_for_monotonic(samples, 10.205), 500.205, places=6)
        self.assertIsNone(ma.stamp_for_monotonic(samples, 20.))

    def test_v6_path_moves_by_odometry_only(self):
        poses = {500.: dict(x=0., y=0., heading_deg=0.)}
        odometry = SimpleNamespace(at_stamp=lambda stamp: poses.get(round(stamp, 3)))
        samples = [dict(received=10., stamp=500.)]
        decision = dict(path=[[i, 0.] for i in range(10)], pose=dict(t_mono=10.))
        from tunnel_runtime import points_at_pose
        path, why = ma.v6_path_now(decision, samples, odometry, dict(x=2., y=0., heading_deg=0.), points_at_pose)
        self.assertIsNone(why)
        self.assertAlmostEqual(float(path[0, 0]), 0.)
        self.assertEqual(len(path), 8)                       # 0 and 1 m are behind the car now

    def test_lateral_departure(self):
        path = np.column_stack((np.arange(0., 30.), np.full(30, 1.)))
        self.assertAlmostEqual(ma.lateral_departure(path, straight()), 1.)

    def candidate(self, decision, path=None, hazards=(), exit_zone=False, checked=None):
        scope, model = ma.make_tunnel_policies(straight(200., 1.), (0, 199))
        path = np.column_stack((np.arange(0., 40., 1.), np.zeros(40))) if path is None else path
        checked = checked or (lambda raw, ref, ego, c, clear, req, keep: (np.asarray(raw), dict(
            accepted=True, clear_distance_m=40., curvature_1pm=0., speed_cap_mps=10., steering_gradient_radpm=0.)))
        return ma.v6_tunnel_candidate(decision, "v6 unavailable", model, path, None, straight(), EGO, None,
                                      None, 12., 5.6, list(hazards), exit_zone, checked)

    def test_v6_candidate_rules(self):
        drive = dict(action="KEEP_LANE", p=.9, age_s=.05, dt=.1)
        candidate, info = self.candidate(drive)
        self.assertIsNotNone(candidate)
        self.assertEqual(info["source"], "v6")
        self.assertIsNone(self.candidate(None)[0])
        self.assertIn("sensors own stops", self.candidate(dict(drive, action="STOP"))[1]["reason"])
        self.assertIn("exit zone", self.candidate(drive, exit_zone=True)[1]["reason"])
        sideways = np.column_stack((np.arange(0., 40., 1.), np.linspace(0., 2., 40)))
        self.assertIn("without a confirmed obstacle", self.candidate(drive, sideways)[1]["reason"])
        self.assertIsNotNone(self.candidate(drive, sideways, hazards=[dict(front=12.)])[0])
        stationary = np.zeros((40, 2))
        self.assertIn("stationary", self.candidate(drive, stationary)[1]["reason"])

    def test_lane_hold_follows_the_driven_side(self):
        from tunnel_runtime import TunnelRuntime
        host = SimpleNamespace(_tunnel_lane_hold=TunnelLaneHold())
        host._tunnel_lane_hold.value = dict(direction=1., offset_m=2.2)
        state = dict(tunnel_active=True, defer_rejoin=True, rejoin_phase=False)
        driven = dict(kind="bypass", direction=-1., offset_m=2.8, generator="portable")
        TunnelRuntime.observe_tunnel_lane_hold(host, driven, state, True)
        self.assertEqual(host._tunnel_lane_hold.value, dict(direction=-1., offset_m=2.8))
        rejoin = dict(kind="rejoin", direction=-1., offset_m=0., generator="portable")
        TunnelRuntime.observe_tunnel_lane_hold(host, rejoin, dict(state, rejoin_phase=True, defer_rejoin=False), True)
        self.assertIsNone(host._tunnel_lane_hold.value)
        TunnelRuntime.observe_tunnel_lane_hold(host, driven, state, False)   # invalid decision: no change
        self.assertIsNone(host._tunnel_lane_hold.value)


class TunnelExit(unittest.TestCase):
    def host(self):
        from tunnel_runtime import TunnelRuntime
        tunnel = SimpleNamespace(end=100., origin=np.zeros(2), axis=np.array([1., 0.]), max_lateral=.8, max_yaw=3.,
                                 project_scan=lambda xyz, ego, plane, stats: dict(valid=True, aligned_ego=dict(ego),
                                     obstacle_points=np.empty((0, 2)), wall_stats=dict(stats, propagated=True), constraint=None))
        host = SimpleNamespace(tunnel=tunnel)
        host.anchor = lambda *a: TunnelRuntime.tunnel_exit_anchor(host, *a)
        return host

    def anchor(self, x):
        pose = dict(x=x, y=0., heading_deg=0.)
        return dict(stamp=10., match=dict(aligned_ego=dict(pose), wall_stats={}), odometry=dict(pose))

    def test_failed_match_near_the_exit_keeps_the_last_verified_one(self):
        host = self.host(); failed = dict(valid=False, reason="both supported tunnel walls required")
        ego = dict(x=80., y=.1, heading_deg=0.)
        match = host.anchor(failed, self.anchor(70.), dict(xyz=None, ground_plane=None), 15., ego, dict(ego))
        self.assertTrue(match["valid"])
        self.assertIsNone(match["constraint"])
        self.assertEqual(match["exit_anchor"]["failed"], "both supported tunnel walls required")

    def test_far_from_the_exit_old_or_inconsistent_stays_failed(self):
        host = self.host(); failed = dict(valid=False, reason="pose correction exceeds validated limit")
        far = dict(x=40., y=0., heading_deg=0.)
        self.assertFalse(host.anchor(failed, self.anchor(35.), {}, 11., far, dict(far))["valid"])
        near = dict(x=80., y=0., heading_deg=0.)
        self.assertFalse(host.anchor(failed, self.anchor(70.), {}, 40., near, dict(near))["valid"])      # 30 s old
        shifted = dict(x=80., y=1.5, heading_deg=0.)
        self.assertFalse(host.anchor(failed, self.anchor(70.), {}, 15., shifted, dict(x=80., y=0., heading_deg=0.))["valid"])

    def test_exit_zone_and_centre_return_release_the_lane_hold(self):
        from tunnel_runtime import TunnelRuntime
        route = straight(200., 1.)
        scope, model = ma.make_tunnel_policies(route, (30, 150))
        hold = TunnelLaneHold(); hold.value = dict(direction=1., offset_m=2.2)
        host = SimpleNamespace(tunnel_route_policy=scope, _tunnel_lane_hold=hold, v6_latest=None, v6_worker=None)
        TunnelRuntime.tunnel_model_static(host, straight(), EGO, [], lambda p, t=1.5: True,
                                          lambda p, h=60.: h, {}, None, 140, False, exit_zone=True)
        self.assertIsNone(hold.value)
        hold.value = dict(direction=1., offset_m=2.2)
        TunnelRuntime.observe_tunnel_lane_hold(host, None, dict(tunnel_active=True, rejoin_phase=True), True)
        self.assertIsNone(hold.value)
        hold.value = dict(direction=1., offset_m=2.2)
        TunnelRuntime.observe_tunnel_lane_hold(host, None, dict(tunnel_active=True, rejoin_phase=False), True)
        self.assertIsNotNone(hold.value)

    def test_rejoin_starts_seventy_metres_before_the_exit(self):
        scope, _ = ma.make_tunnel_policies(straight(200., 1.), (30, 150))
        self.assertTrue(scope.state(75)["defer_rejoin"])     # exit at 150 m: return from 80 m on
        self.assertTrue(scope.state(81)["rejoin_phase"])


class ChildRecord(unittest.TestCase):
    def test_pose_recorder_and_extended_latest(self):
        import v6_ext_child
        recorder = v6_ext_child.PoseRecorder(lambda t, x, y, yaw, t_end: (np.array([[1., 2.], [3., 4.]]),
                                                                         np.array([0., math.pi / 2])))
        recorder([1., 2.], [0, 0], [0, 0], [0, 0], 5.)
        self.assertEqual(recorder.pose, dict(x=3., y=4., heading_deg=90., t_mono=2.))
        with tempfile.TemporaryDirectory() as d:
            child = object.__new__(v6_ext_child.DrivingChild)
            child.a = SimpleNamespace(latest=os.path.join(d, "latest.json"))
            child.log = open(os.path.join(d, "log.jsonl"), "w")
            child.write(dict(t=1., ok=True, action="LEFT_CHANGE", action_probabilities={"LEFT_CHANGE": .8},
                             path_full=[[0., 0.], [1., 0.]], path_xy=[[1., 0.]], dt=.1,
                             pose=dict(x=0., y=0., heading_deg=0., t_mono=3.)))
            child.log.close()
            with open(child.a.latest) as handle:
                latest = json.load(handle)
            with open(os.path.join(d, "log.jsonl")) as handle:
                line = json.loads(handle.readline())
        self.assertEqual((latest["action"], latest["action_full"], latest["p"]), ("LEFT", "LEFT_CHANGE", .8))
        self.assertEqual(latest["path"], [[0., 0.], [1., 0.]])
        self.assertEqual(latest["pose"]["t_mono"], 3.)
        self.assertNotIn("path_full", line)
        self.assertEqual(line["path_xy"], [[1., 0.]])


class ModeImports(unittest.TestCase):
    def run_python(self, code, **env):
        result = subprocess.run([PYTHON, "-c", code], cwd=str(HERE), capture_output=True, text=True,
                                env=dict(os.environ, **env), timeout=240)
        self.assertEqual(result.returncode, 0, result.stderr[-2000:])
        return result.stdout.strip().splitlines()[-1]

    def test_tunnel_policy_flag(self):
        self.assertEqual(self.run_python("import tunnel_runtime as t; print(t.TUNNEL_MODEL, t.TUNNEL_LATTICE)",
                                         STUDENT_TUNNEL_POLICY="model"), "True False")
        self.assertEqual(self.run_python("import tunnel_runtime as t; print(t.TUNNEL_MODEL, t.TUNNEL_LATTICE)"),
                         "False True")

    def test_gui_model_mode_configures_the_package(self):
        out = self.run_python("import gui, model_avoidance as m; print(gui.MODEL_AVOID, gui.USE_LATTICE, "
                              "m._static.MAX_CONTROL_CURVATURE == gui.MAX_CONTROL_CURVATURE, "
                              "m._static.static_clear_distance is gui.static_clear_distance)",
                              STUDENT_PLANNER="model")
        self.assertEqual(out, "True False True True")


if __name__ == "__main__":
    unittest.main()


class ManoeuvreComplete(unittest.TestCase):
    """A finished bypass must not hand its side to the next obstacle (follow-20261010T083538Z)."""
    route = np.column_stack((np.arange(-5., 80., .5), np.zeros(170)))

    def plan(self, offset):
        x = np.arange(0., 40., .5)
        return dict(direction=1., offset_m=offset, path=np.column_stack((x, offset*np.exp(-((x-15.)/6.)**2))))

    def test_rejoined_plan_is_complete_and_a_running_bypass_is_not(self):
        self.assertTrue(ma.manoeuvre_complete(self.plan(0.2), self.route))
        self.assertFalse(ma.manoeuvre_complete(self.plan(2.2), self.route))
        self.assertFalse(ma.manoeuvre_complete(None, self.route))

    def test_complete_plan_no_longer_fixes_the_side(self):
        decision = dict(action="RIGHT_CHANGE", p=.8)
        hazards = [dict(distance_m=30.)]
        previous = self.plan(0.)
        self.assertEqual(ma.choose_direction(previous, hazards, decision), (1., "kept"))
        if ma.manoeuvre_complete(previous, self.route):
            previous = None
        self.assertEqual(ma.choose_direction(previous, hazards, decision), (-1., "v6"))


class KeptSideClosed(unittest.TestCase):
    """follow-20261010T090630Z 18:13:31: room only on the right, the rejoin's left side kept."""
    route = np.column_stack((np.arange(-5., 80., .5), np.full(170, .45)))

    def test_closed_kept_side_is_released_near_the_route(self):
        hazards = [dict(distance_m=33.5)]
        self.assertTrue(ma.kept_side_closed("kept", 1., {-1: 3.48}, self.route, hazards))
        self.assertFalse(ma.kept_side_closed("kept", 1., {-1: 3.48, 1: 3.2}, self.route, hazards))
        self.assertFalse(ma.kept_side_closed("v6", 1., {-1: 3.48}, self.route, hazards))
        self.assertFalse(ma.kept_side_closed("kept", 1., {}, self.route, hazards))
        self.assertFalse(ma.kept_side_closed("kept", 1., {-1: 3.48}, self.route, []))

    def test_not_released_while_beside_an_obstacle(self):
        beside = np.column_stack((np.arange(-5., 80., .5), np.full(170, -2.5)))   # car 2.5 m off the route
        self.assertFalse(ma.kept_side_closed("kept", 1., {-1: 3.48}, beside, [dict(distance_m=10.)]))
