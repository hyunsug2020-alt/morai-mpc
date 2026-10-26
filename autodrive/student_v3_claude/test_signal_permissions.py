"""Route-position policies and immediate green release through perception/control."""
import json
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from ego_body import EGO_FRONT_M
from gui import HdMapLanes
from signal_control.permissions import FOURTH_SIGNAL_NODE
from signal_control.runtime import SignalController, SignalFrameWorker, mapped_signal_lines, route_signal_maneuver
from signal_control.signal_config import DEFAULT_CONFIG
from signal_control.signal_path import SignalPathGate
from test_signal_control import ReportWorker, lamps, controller_at


class SignalPermissionTests(unittest.TestCase):
    def route_lanes(self):
        root = Path(__file__).resolve().parent
        lanes = HdMapLanes(root/"link_set.json")
        route = json.loads((root/"global_path.json").read_bytes())["waypoints"]
        lanes.set_route(np.array([[p["x"], p["y"]] for p in route]))
        return lanes

    def test_numbering_counts_only_signal_control_positions_and_fourth_is_last_left(self):
        lanes = self.route_lanes()
        rows = mapped_signal_lines(lanes)
        self.assertEqual([(r["stop_line_node_id"], r["signal_index"], r["permission_policy"]) for r in rows],
            [("A1256W000682", 1, "green"), ("A1256W000807", 2, "green"),
             ("A1256W000208", 3, "green"), ("A1256W000757", None, "ignore"),
             ("A1256W000742", 4, "red_left_only")])
        self.assertEqual(FOURTH_SIGNAL_NODE, rows[-1]["stop_line_node_id"])

    def test_actual_path_overrides_incorrect_map_turn_annotations(self):
        rows = mapped_signal_lines(self.route_lanes())
        self.assertEqual(rows[0]["mapped_related_signal"], "right_unprotected")
        self.assertEqual(rows[0]["maneuver"], "straight")
        self.assertLess(abs(rows[0]["route_turn_deg"]), 1.)
        self.assertEqual(rows[3]["mapped_related_signal"], "left")
        self.assertEqual(rows[3]["maneuver"], "right")
        self.assertLess(rows[3]["route_turn_deg"], -70.)
        self.assertTrue(all(row["maneuver_source"] == "route" for row in rows))

    def test_missing_or_invalid_route_never_grants_map_right_turn_exemption(self):
        for geometry in ({}, dict(route_xy=[[0., 0.], [1., 0.]], route_station=[0., 1.]),
                         dict(route_xy=[[0., 0.], [float("nan"), 30.]], route_station=[0., 30.])):
            lanes = SimpleNamespace(**geometry, links=[dict(idx="entry", related_signal="right_unprotected")],
                stop_lines=[dict(id="signal", station_m=10., entry_link_id="entry", source="hdmap_node",
                                 node_id="node", xy=[10., 0.], traffic_light_id="C1")])
            self.assertEqual(route_signal_maneuver(lanes, 10.), ("straight", None))
            self.assertEqual(mapped_signal_lines(lanes)[0]["permission_policy"], "green")

    def test_first_actual_straight_brakes_by_distance_and_only_green_releases(self):
        lanes = self.route_lanes()
        worker = ReportWorker()
        controller = SignalController(lanes, enabled=True, run_id="run", worker=worker)
        station = controller.lines[0]["station"]-EGO_FRONT_M-80.
        xy = [np.interp(station, lanes.route_station, lanes.route_xy[:, axis]) for axis in (0, 1)]
        ahead = [np.interp(station+.1, lanes.route_station, lanes.route_xy[:, axis]) for axis in (0, 1)]
        heading = np.degrees(np.arctan2(ahead[1]-xy[1], ahead[0]-xy[0]))
        ego = dict(x=xy[0], y=xy[1], heading_deg=heading, speed_kmh=55.)
        controller.evaluate(ego)
        for state in ("UNKNOWN", "RED", "YELLOW", "LEFT", "RED_LEFT", "GREEN", "GREEN_LEFT"):
            worker.publish(state)
            plan = controller.evaluate(ego)
            self.assertEqual(plan["signal_index"], 1)
            self.assertEqual(plan["maneuver"], "straight")
            self.assertAlmostEqual(plan["distance_m"], 80., delta=.05)
            self.assertEqual(plan["stop_required"], state not in ("GREEN", "GREEN_LEFT"))
            if plan["stop_required"]:
                self.assertGreater(plan["cap_mps"], 0.)
                self.assertLess(plan["cap_mps"]*3.6, ego["speed_kmh"])
            else:
                self.assertIsNone(plan["cap_mps"])

    def test_actual_worker_and_gate_apply_exact_policy_on_first_valid_frame(self):
        states = ("UNKNOWN", "RED", "YELLOW", "GREEN", "LEFT", "RED_LEFT", "GREEN_LEFT")
        for policy, allowed in (("green", {"GREEN", "GREEN_LEFT"}), ("red_left_only", {"RED_LEFT"})):
            receiver = SimpleNamespace(lock=threading.Lock(), frames={"front_low": []}, counts={"front_low": []})
            worker = SignalFrameWorker(receiver, "run", DEFAULT_CONFIG, SimpleNamespace(mask=lambda *_: lamps()[1]))
            worker.select("signal", context=dict(traffic_light_id="test", permission_policy=policy, layout="four",
                window=[.4, .05, .6, .3], expected_box=[600/1280, 100/720, 700/1280, 124/720]))
            gate = SignalPathGate("run")
            line = dict(station=EGO_FRONT_M+.1, link_id="entry", maneuver="left", permission_policy=policy)
            # Include green-family alternation after RED/UNKNOWN: no three-frame wait.
            for i, state in enumerate(states+("RED", "GREEN", "GREEN_LEFT", "GREEN", "UNKNOWN", "RED_LEFT")):
                with self.subTest(policy=policy, state=state, frame=i):
                    _, jpeg = cv2.imencode(".jpg", lamps(state)[0])
                    receiver.frames["front_low"].append((i, jpeg.tobytes()))
                    receiver.counts["front_low"].append(time.monotonic())
                    self.assertTrue(worker.process_once())
                    report = worker.snapshot()
                    self.assertEqual(report["raw_state"], state)
                    self.assertEqual(report["stop_required"], state not in allowed)
                    self.assertTrue(report["permission_stable"])
                    self.assertEqual(worker.pipeline.config["stable_frames"], 1)
                    plan = gate.evaluate(report, line, 0., time.time())
                    self.assertEqual(plan["stop_required"], state not in allowed)
                    self.assertEqual(plan["cap_mps"], None if state in allowed else 0.)

    def test_camera_prepares_first_signal_before_any_speed_cap_owns_the_approach(self):
        lanes = self.route_lanes()
        worker = ReportWorker()
        controller = SignalController(lanes, enabled=True, run_id='run', worker=worker)
        station = controller.lines[0]['station']-EGO_FRONT_M-110.
        xy = [np.interp(station, lanes.route_station, lanes.route_xy[:, axis]) for axis in (0, 1)]
        ego = dict(x=xy[0], y=xy[1], heading_deg=90., speed_kmh=30.)
        plan = controller.evaluate(ego)
        self.assertEqual(worker.target, controller.lines[0]['id'])
        self.assertFalse(plan['active'])
        self.assertIsNone(plan['cap_mps'])
        # Re-reading a prepared target must not make the distant line active.
        worker.publish('RED')
        plan = controller.evaluate(ego)
        self.assertFalse(plan['active'])
        self.assertIsNone(plan['cap_mps'])

    def test_stale_or_missing_observation_stops_even_after_permitted_green_or_red_arrow(self):
        for policy, permitted in (("green", "GREEN"), ("red_left_only", "RED_LEFT")):
            controller, worker, ego = controller_at(.1, policy=policy)
            worker.publish(permitted)
            self.assertIsNone(controller.evaluate(ego)["cap_mps"])
            worker.report["received_mono"] -= 2.
            self.assertEqual(controller.evaluate(ego)["cap_mps"], 0.)
            worker.report = {}
            self.assertEqual(controller.evaluate(ego)["cap_mps"], 0.)

    def test_right_turn_never_waits_for_signal_or_selects_next_straight_signal(self):
        controller, worker, ego = controller_at(.1, maneuver="right")
        next_line = dict(controller.lines[0], id="next", station=30., maneuver="straight", permission_policy="green")
        controller.lines.append(next_line)
        for state in ("UNKNOWN", "RED", "YELLOW", "GREEN", "RED_LEFT"):
            worker.publish(state)
            plan = controller.evaluate(ego)
            self.assertFalse(plan["active"])
            self.assertFalse(plan["stop_required"])
            self.assertIsNone(plan["cap_mps"])
            self.assertEqual(plan["permission_policy"], "ignore")
            self.assertIsNone(worker.target)
        passed = controller.evaluate(dict(ego, x=2.))
        self.assertTrue(passed["active"])
        self.assertEqual(passed["permission_policy"], "green")

    def test_permitted_crossing_cannot_reacquire_old_signal_from_older_inference_pose(self):
        for policy, allowed in (("green", "GREEN"), ("red_left_only", "RED_LEFT")):
            controller, worker, ego = controller_at(.2, policy=policy)
            worker.publish(allowed)
            self.assertIsNone(controller.evaluate(ego)["cap_mps"])
            # Recorded failure: publisher passes -0.3 m, clears the camera target;
            # the next inference still carries a pose at -0.1 m and reselects it.
            for x in (.7, .3, .15, .8, 4.):
                plan = controller.evaluate(dict(ego, x=x))
                self.assertFalse(plan["stop_required"])
                self.assertIsNone(plan["cap_mps"])
                self.assertEqual(plan["phase"], "CLEARED")
                self.assertIsNone(controller.gate.hold_line)
            self.assertEqual(SignalPathGate.apply(plan, 0., True), (0., True))
            # Clearance persists beyond camera freshness, but not into another lap.
            self.assertIsNone(controller.evaluate(dict(ego, x=4.), now=time.time()+10.)["cap_mps"])
            self.assertTrue(controller.evaluate(dict(ego, x=-20.))["stop_required"])

    def test_clearance_never_releases_next_signal_or_red_seen_before_crossing(self):
        controller, worker, ego = controller_at(.2)
        worker.publish("GREEN")
        controller.evaluate(ego)
        controller.evaluate(dict(ego, x=.7))
        controller.lines.append(dict(controller.lines[0], id="next", link_id="next", station=30.))
        next_plan = controller.evaluate(dict(ego, x=4.))
        self.assertTrue(next_plan["stop_required"])
        self.assertTrue(next_plan["active"])

        for policy, allowed, blockings in (("green", "GREEN", ("RED", "YELLOW", "LEFT", "RED_LEFT")),
                ("red_left_only", "RED_LEFT", ("GREEN", "GREEN_LEFT", "LEFT", "RED", "YELLOW"))):
            for blocking in blockings:
                controller, worker, ego = controller_at(2., policy=policy)
                worker.publish(allowed)
                controller.evaluate(ego)
                worker.publish(blocking)
                self.assertTrue(controller.evaluate(dict(ego, x=1.))["stop_required"])
                worker.report = {}
                self.assertEqual(controller.evaluate(dict(ego, x=2.1))["cap_mps"], 0.)

    def test_recorded_fourth_signal_crossing_never_latches_unknown_hold(self):
        fixture = json.loads((Path(__file__).parent/"test_data/signal_crossing_reselection_20261020.json").read_text())
        rows = fixture["rows"]
        controller, worker, ego = controller_at(rows[0]["gap"], policy=fixture["permission_policy"])
        start = rows[0]["wall"]
        base = time.time()
        for row in rows:
            if row["fresh"] and row["raw"] == "RED_LEFT":
                worker.publish("RED_LEFT", time=base+row["wall"]-start,
                               frame_wall=base+row["frame_wall"]-start)
            else:
                worker.report = {}
            plan = controller.evaluate(dict(ego, x=rows[0]["gap"]-row["gap"], speed_kmh=row["speed_kmh"]),
                                       now=base+row["wall"]-start)
            self.assertFalse(plan["stop_required"])
            self.assertIsNone(plan["cap_mps"])
            self.assertIsNone(controller.gate.hold_line)

    def test_unknown_without_recent_live_permission_keeps_overshoot_hold(self):
        for changes in ({"raw_state": "RED", "state": "RED"}, {"source": "recording"},
                        {"received_mono": time.monotonic()-2.}, {"frame_wall": time.time()-2.}):
            controller, worker, ego = controller_at(.2)
            worker.publish(**dict(dict(state="GREEN"), **changes))
            self.assertEqual(controller.evaluate(ego)["cap_mps"], 0.)
            worker.report = {}
            self.assertEqual(controller.evaluate(dict(ego, x=.3))["cap_mps"], 0.)
        controller, worker, ego = controller_at(.2)
        worker.publish("GREEN")
        controller.evaluate(ego)
        worker.report = {}
        self.assertEqual(controller.evaluate(dict(ego, x=.3), now=time.time()+2.)["cap_mps"], 0.)


if __name__ == "__main__":
    unittest.main()
