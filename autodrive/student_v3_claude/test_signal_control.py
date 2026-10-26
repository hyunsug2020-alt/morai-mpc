"""Camera freshness, mapped signal scope and final command regressions."""
import json
import threading
import time
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from gui import HdMapLanes
from ego_body import EGO_FRONT_M
from signal_control import DEFAULT_CONFIG, SignalFramePipeline, SignalPathGate, requires_stop, validate_config
from signal_control.runtime import SignalController, SignalFrameWorker, mapped_signal_lines, signal_enabled_from_env
from signal_control.signal_inference import ContainerSemanticSignal
import test_avoidance_gate as avoidance_tests
import test_live_path_safety as live_tests


def lamps(state="GREEN"):
    image = np.zeros((720, 1280, 3), np.uint8)
    mask = np.zeros((720, 1280), np.uint8)
    x, y, w, h = 600, 100, 100, 24
    mask[y:y+h, x:x+w] = 255
    image[y:y+h, x:x+w] = 30
    on = {"RED": (0,), "YELLOW": (1,), "LEFT": (2,), "GREEN": (3,),
          "RED_LEFT": (0, 2), "GREEN_LEFT": (2, 3), "UNKNOWN": ()}[state]
    colors = ((0, 0, 255), (0, 255, 255), (0, 255, 0), (0, 255, 0))
    for slot in on:
        cv2.circle(image, (x+12+slot*25, y+12), 5, colors[slot], -1)
    return image, mask


class ReportWorker:
    def __init__(self):
        self.target, self.report = None, {}

    def select(self, target, context=None):
        if target != self.target:
            self.target, self.report = target, {}

    def snapshot(self):
        return dict(self.report)

    def publish(self, state="GREEN", **kwargs):
        now = time.time()
        self.report = dict(fresh=True, source="live", run_id="run", time=now, frame_wall=now,
                           state=state, raw_state=state, permission_stable=True,
                           stable_count=3, received_mono=time.monotonic(), line_id=self.target,
                           camera_id=4, camera_view="front_low")
        self.report.update(kwargs)


def controller_at(gap=10., *, enabled=True, lanes=None, maneuver="straight", policy=None):
    if lanes is None:
        station = EGO_FRONT_M+gap
        exit_xy = ([station, -40.] if maneuver == "right" else
                   [station, 40.] if maneuver == "left" else [station+40., 0.])
        route = np.array([[-30., 0.], [station, 0.], exit_xy])
        lanes = SimpleNamespace(route_progress=lambda ego: ego["x"],
            route_xy=route,
            route_station=np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]-30.,
            links=[dict(idx="entry", related_signal=maneuver)],
            stop_lines=[dict(id="signal", station_m=EGO_FRONT_M+gap,
                entry_link_id="entry", source="hdmap_node", node_id="node",
                xy=[EGO_FRONT_M+gap, 0.], traffic_light_id="C1")])
    worker = ReportWorker()
    controller = SignalController(lanes, enabled=enabled, run_id="run", worker=worker)
    if policy is not None:
        for line in controller.lines:
            line["permission_policy"] = policy
    ego = dict(x=0., y=0., heading_deg=0., speed_kmh=30.)
    controller.evaluate(ego)
    return controller, worker, ego


class SignalPerceptionTests(unittest.TestCase):
    def test_gpu_pipe_timeout_poisoning_prevents_late_response_reuse(self):
        model = ContainerSemanticSignal.__new__(ContainerSemanticSignal)
        model.failed = model.closed = False
        model.process = SimpleNamespace(poll=lambda: None)
        writes = []
        model._write_exact = lambda data: writes.append(bytes(data))
        model._read_exact = lambda _: (_ for _ in ()).throw(TimeoutError("late GPU result"))
        with self.assertRaises(TimeoutError):
            model.mask(np.zeros((4, 8, 3), np.uint8), .5)
        self.assertTrue(model.failed)
        self.assertEqual(len(writes), 2)
        with self.assertRaises(RuntimeError):
            model.mask(np.zeros((4, 8, 3), np.uint8), .5)
        self.assertEqual(len(writes), 2)

    def test_lamp_slots_stability_and_immediate_stop(self):
        config = validate_config(DEFAULT_CONFIG)
        for state in ("GREEN", "LEFT", "RED_LEFT", "GREEN_LEFT", "RED", "YELLOW", "UNKNOWN"):
            with self.subTest(state=state):
                pipeline = SignalFramePipeline(config)
                for i in range(3):
                    result, _, _ = pipeline.process(*lamps(state), "run", 100.+i/5, 100.+i/5)
                    self.assertEqual(result["raw_state"], state)
                    if i < 2:
                        self.assertFalse(result["permission_stable"])
                self.assertEqual(result["stop_required"], requires_stop(state, "straight"))
                red, _, _ = pipeline.process(*lamps("RED"), "run", 101., 101.)
                self.assertTrue(red["stop_required"])

    def test_missing_mask_stale_frame_and_changed_housing_reject_permission(self):
        config = validate_config(DEFAULT_CONFIG)
        pipeline = SignalFramePipeline(config)
        image, mask = lamps()
        for i in range(3):
            result, _, _ = pipeline.process(image, mask, "run", 100.+i/5, 100.+i/5)
        self.assertFalse(result["stop_required"])
        for mask, frame, now in ((mask, 100., 103.), (np.zeros_like(mask), 103., 103.)):
            result, _, _ = pipeline.process(image, mask, "run", frame, now)
            self.assertTrue(result["stop_required"])

    def test_worker_uses_distinct_receipts_and_accepts_remote_clock_offset(self):
        image, mask = lamps()
        _, jpeg = cv2.imencode(".jpg", image)
        receiver = SimpleNamespace(lock=threading.Lock(), frames={"front_low": deque()}, counts={"front_low": deque()})
        model = SimpleNamespace(mask=lambda *_: mask)
        worker = SignalFrameWorker(receiver, "run", validate_config(DEFAULT_CONFIG), model)
        worker.select("signal")
        for i in range(3):
            # Source wall clock is deliberately unrelated to this PC.
            receiver.frames["front_low"].append((10+i, jpeg.tobytes()))
            receiver.counts["front_low"].append(time.monotonic())
            self.assertTrue(worker.process_once())
            count = worker.snapshot()["stable_count"]
            self.assertFalse(worker.process_once())
            self.assertEqual(worker.snapshot()["stable_count"], count)
        self.assertTrue(worker.snapshot()["permission_stable"])
        self.assertTrue(worker.snapshot()["fresh"])
        worker.last_received -= 2.
        receiver.frames["front_low"].append((13, jpeg.tobytes()))
        receiver.counts["front_low"].append(time.monotonic())
        worker.process_once()
        self.assertFalse(worker.snapshot()["permission_stable"])
        worker.select("next-signal")
        self.assertEqual(worker.snapshot(), {})

    def test_worker_discards_inference_when_selected_line_changes(self):
        image, mask = lamps()
        _, jpeg = cv2.imencode(".jpg", image)
        receiver = SimpleNamespace(lock=threading.Lock(),
            frames={"front_low": [(1, jpeg.tobytes())]}, counts={"front_low": [time.monotonic()]})
        entered, release = threading.Event(), threading.Event()
        def delayed(*_):
            entered.set()
            self.assertTrue(release.wait(2.))
            return mask
        worker = SignalFrameWorker(receiver, "run", validate_config(DEFAULT_CONFIG), SimpleNamespace(mask=delayed))
        worker.select("old")
        thread = threading.Thread(target=worker.process_once)
        thread.start()
        self.assertTrue(entered.wait(2.))
        worker.select("new")
        release.set()
        thread.join(2.)
        self.assertFalse(thread.is_alive())
        self.assertEqual(worker.snapshot(), {})

    def test_old_camera_receipt_is_not_refreshed_by_polling(self):
        image, _ = lamps()
        _, jpeg = cv2.imencode(".jpg", image)
        model = SimpleNamespace(mask=lambda *_: self.fail("stale image reached GPU"))
        receiver = SimpleNamespace(lock=threading.Lock(),
            frames={"front_low": [(1, jpeg.tobytes())]}, counts={"front_low": [time.monotonic()-2.]})
        worker = SignalFrameWorker(receiver, "run", validate_config(DEFAULT_CONFIG), model)
        worker.select("signal")
        self.assertFalse(worker.process_once())
        self.assertEqual(worker.snapshot(), {})

    def test_recorded_images_cannot_supply_live_green_permission(self):
        controller, worker, ego = controller_at(.1)
        worker.publish(source="recording")
        self.assertEqual(controller.evaluate(ego)["cap_mps"], 0.)

    def test_front_camera_cannot_replace_missing_or_stale_signal_camera(self):
        image, mask = lamps()
        _, jpeg = cv2.imencode(".jpg", image)
        receiver = SimpleNamespace(lock=threading.Lock(), frames={"front": [(10, jpeg.tobytes())]},
                                   counts={"front": [time.monotonic()]})
        model = SimpleNamespace(mask=lambda *_: self.fail("front camera reached signal inference"))
        worker = SignalFrameWorker(receiver, "run", validate_config(DEFAULT_CONFIG), model)
        worker.select("signal")
        self.assertFalse(worker.process_once())
        receiver.frames["front_low"] = [(1, jpeg.tobytes())]
        receiver.counts["front_low"] = [time.monotonic()-2.]
        self.assertFalse(worker.process_once())
        self.assertEqual(worker.snapshot(), {})

    def test_other_camera_green_cannot_release_signal_stop(self):
        controller, worker, ego = controller_at(.1)
        for camera_id, view in ((1, "front"), (None, None), (4, "front")):
            with self.subTest(camera_id=camera_id, view=view):
                worker.publish(camera_id=camera_id, camera_view=view)
                self.assertEqual(controller.evaluate(ego)["cap_mps"], 0.)
        worker.publish(camera_id=4, camera_view="front_low")
        self.assertIsNone(controller.evaluate(ego)["cap_mps"])


class SignalGateTests(unittest.TestCase):
    def test_unknown_yellow_red_and_bad_reports_stop(self):
        for state in ("UNKNOWN", "YELLOW", "RED", "LEFT", "RED_LEFT"):
            controller, worker, ego = controller_at(.1)
            worker.publish(state)
            plan = controller.evaluate(ego)
            self.assertEqual(plan["phase"], "HOLD")
            self.assertEqual(plan["cap_mps"], 0.)
        bad = [dict(fresh=False), dict(source="recording"), dict(run_id="other"),
               dict(frame_wall=time.time()-5), dict(time=time.time()-5),
               dict(frame_wall=time.time()+5), dict(time=float("nan")),
               dict(permission_stable=False), dict(permission_stable=None),
               dict(permission_stable="true"), dict(line_id="other"),
               dict(received_mono=time.monotonic()-5), dict(raw_state="RED")]
        for changes in bad:
            with self.subTest(changes=changes):
                controller, worker, ego = controller_at(.1)
                worker.publish(**changes)
                self.assertEqual(controller.evaluate(ego)["cap_mps"], 0.)

    def test_missing_stability_field_cannot_grant_green(self):
        controller, worker, ego = controller_at(.1)
        worker.publish()
        worker.report.pop("permission_stable")
        self.assertEqual(controller.evaluate(ego)["cap_mps"], 0.)

    def test_red_envelope_does_not_creep_under_localization_jitter(self):
        controller, worker, ego = controller_at(20.)
        worker.publish("RED")
        previous = 100.
        for x in (0., 3., 2.8, 8., 7.7, 15., 19.9, 19.7, 20.2):
            plan = controller.evaluate(dict(ego, x=x))
            self.assertLessEqual(plan["cap_mps"], previous)
            previous = plan["cap_mps"]
        self.assertEqual(previous, 0.)
        worker.publish("GREEN")
        self.assertFalse(controller.evaluate(dict(ego, x=20.))['stop_required'])

    def test_green_has_no_stop_line_cap_and_cannot_release_other_emergency(self):
        controller, worker, ego = controller_at(.1)
        worker.publish()
        self.assertIsNone(controller.evaluate(ego)["cap_mps"])
        self.assertEqual(controller.apply(ego, 0.)[0], 0.)
        self.assertEqual(controller.apply(ego, 12.)[0], 12.)
        plan = controller.evaluate(ego)
        self.assertEqual(SignalPathGate.apply(plan, 0., True), (0., True))

    def test_selected_braking_line_survives_dynamic_horizon_shrink(self):
        controller, worker, ego = controller_at(95.)
        controller.evaluate(dict(ego, speed_kmh=55.))
        worker.publish("RED")
        self.assertTrue(controller.evaluate(dict(ego, speed_kmh=55.))["active"])
        self.assertTrue(controller.evaluate(dict(ego, speed_kmh=5.))["active"])

    def test_approach_gains_about_ten_kmh_without_spending_the_stop_reserve(self):
        for distance in (5., 10., 20., 40., 50., 60., 100.):
            line = dict(link_id='approach', station=EGO_FRONT_M+distance,
                        maneuver='straight', permission_policy='green')
            gate = SignalPathGate('run')
            plan = gate.evaluate({}, line, 0., 1000., speed_mps=55./3.6)
            speed = plan['cap_mps']
            self.assertTrue(plan['stop_required'])
            self.assertLessEqual(speed*gate.reaction_seconds+
                speed*speed/(2*gate.deceleration)+gate.stop_margin, distance+1e-9)
            if distance in (50., 60.):
                old = SignalPathGate('run', deceleration=.8, stop_margin=.25)
                old_cap = old.evaluate({}, line, 0., 1000.)['cap_mps']
                self.assertGreater((speed-old_cap)*3.6, 8.)
                self.assertLess((speed-old_cap)*3.6, 12.)
        controller, worker, ego = controller_at(1.)
        worker.publish('RED')
        self.assertEqual(controller.evaluate(ego)['cap_mps'], 0.)

    def test_delayed_approach_stops_before_the_line_at_normal_road_speeds(self):
        # Exercise motion under delayed commands and a physical brake limit,
        # rather than checking only the envelope formula at a static pose.
        for kmh in (25., 40., 55.):
            for delay_s in (.4, .8, 1.):
                with self.subTest(kmh=kmh, delay_s=delay_s):
                    gate, dt, speed = SignalPathGate('run'), .02, kmh/3.6
                    distance = (speed*gate.reaction_seconds+speed*speed/(2*gate.deceleration)
                                +gate.stop_margin+5.)
                    line = dict(link_id='approach', station=EGO_FRONT_M+distance,
                                maneuver='straight', permission_policy='green')
                    commands = deque([speed]*round(delay_s/dt))
                    for step in range(5000):
                        now = 1000.+step*dt
                        report = dict(fresh=True, source='live', run_id='run', frame_wall=now,
                                      time=now, state='RED', raw_state='RED', permission_stable=True)
                        plan = gate.evaluate(report, line,
                            line['station']-EGO_FRONT_M-distance, now, speed_mps=speed)
                        target = min(kmh/3.6, plan['cap_mps'])
                        commands.append(target)
                        next_speed = max(0., speed+max(-2.5*dt,
                            min(2.5*dt, commands.popleft()-speed)))
                        distance -= (speed+next_speed)*dt/2.
                        speed = next_speed
                        self.assertGreaterEqual(distance, .15)
                        if speed < .01 and target < .01:
                            break
                    else:
                        self.fail('signal approach did not finish stopping')
                    self.assertLess(distance, 2.)

    def test_passed_line_does_not_stop_junction_and_next_line_restarts_stability(self):
        controller, worker, ego = controller_at(2.)
        worker.publish()
        self.assertFalse(controller.evaluate(ego)["stop_required"])
        self.assertFalse(controller.evaluate(dict(ego, x=10.))["active"])
        controller.lines.append(dict(controller.lines[0], id="next", station=50.))
        plan = controller.evaluate(dict(ego, x=10.))
        self.assertTrue(plan["stop_required"])
        self.assertFalse(plan["fresh"])

    def test_disabled_does_not_load_model_start_worker_or_cap_other_planners(self):
        with patch("signal_control.runtime.make_signal_model") as load:
            controller = SignalController(SimpleNamespace(), enabled=False)
            controller.start()
            self.assertEqual(controller.apply({}, 7.), (7., None))
            self.assertEqual(controller.apply({}, 0.), (0., None))
            controller.close()
            load.assert_not_called()
        with patch.dict("os.environ", {"STUDENT_SIGNAL_GUARD": "false"}):
            with self.assertRaises(ValueError):
                signal_enabled_from_env()

    def test_mapped_signal_scope_and_maneuvers_use_actual_route(self):
        root = Path(__file__).parent
        route = np.array([[p["x"], p["y"]] for p in json.loads((root/"global_path.json").read_text())["waypoints"]])
        lanes = HdMapLanes(root/"link_set.json")
        lanes.set_route(route)
        lines = mapped_signal_lines(lanes)
        self.assertEqual([line["stop_line_node_id"] for line in lines],
                         ["A1256W000682", "A1256W000807", "A1256W000208", "A1256W000757", "A1256W000742"])
        self.assertEqual([line["maneuver"] for line in lines], ["straight", "straight", "straight", "right", "left"])
        self.assertNotIn("A1256W000709", [line["stop_line_node_id"] for line in lines])
        self.assertTrue(all(line["traffic_light_id"].startswith("C") for line in lines))
        for line in lines:
            original = next(s for s in lanes.stop_lines if s["id"] == line["id"])
            self.assertEqual(line["station"], original["station_m"])

    def test_maneuver_permissions_do_not_confuse_arrow_with_straight(self):
        for maneuver in ("straight", "left", "right"):
            for state in ("RED", "YELLOW", "UNKNOWN", "GREEN", "LEFT", "RED_LEFT", "GREEN_LEFT"):
                controller, worker, ego = controller_at(.1, maneuver=maneuver)
                worker.publish(state)
                plan = controller.evaluate(ego)
                policy = "ignore" if maneuver == "right" else "green"
                self.assertEqual(plan["stop_required"], requires_stop(state, maneuver, policy))


class SignalPublisherTests(unittest.TestCase):
    def attach_signal(self, runtime, gap=.1, policy=None):
        controller, worker, _ = controller_at(gap, policy=policy)
        runtime.signal_controller = controller
        runtime.path_publisher._signal_controller = controller
        return controller, worker

    def payload(self, runtime):
        return json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)

    def test_final_publisher_cannot_overwrite_signal_hold_with_planner_speed(self):
        runtime = avoidance_tests.AvoidanceGateTests().runtime(action="KEEP")
        controller, worker = self.attach_signal(runtime)
        worker.publish("RED")
        runtime.infer()
        self.assertEqual(self.payload(runtime)["target_speed_mps"], 0.)
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        worker.publish("GREEN")
        runtime.infer()
        self.assertGreater(self.payload(runtime)["target_speed_mps"], 0.)

    def test_live_timer_stops_on_red_or_expired_permission_without_model(self):
        helper = live_tests.LivePathSafetyTests()
        for stale in (False, True):
            with self.subTest(stale=stale):
                runtime = helper.attach(helper.runtime())
                controller, worker = self.attach_signal(runtime)
                worker.publish("GREEN")
                runtime.infer()
                self.assertGreater(self.payload(runtime)["target_speed_mps"], 0.)
                runtime.model = lambda *_: self.fail("signal stop waited for model inference")
                if stale:
                    worker.report['received_mono'] -= 2.
                else:
                    worker.publish("RED")
                runtime.path_publisher.recheck_active_path()
                payload = self.payload(runtime)
                self.assertEqual(payload["target_speed_mps"], 0.)
                self.assertEqual(payload["decision"]["allowed_mps"], 0.)
                self.assertTrue(payload["decision"]["emergency"])
                self.assertEqual(payload["decision"]["validation"]["traffic_signal"]["phase"], "HOLD")

    def test_signal_off_retains_person_stop(self):
        from test_ped_guard import person
        runtime = live_tests.LivePathSafetyTests().attach(live_tests.LivePathSafetyTests().runtime())
        controller, worker = self.attach_signal(runtime)
        controller.enabled = False
        runtime.object_receiver.objects = [person(5., 0., 0., 0.)]
        runtime.infer()
        self.assertEqual(self.payload(runtime)["target_speed_mps"], 0.)

    def test_green_releases_red_hold_without_waiting_for_model_or_retaining_zero_baseline(self):
        helper = live_tests.LivePathSafetyTests()
        runtime = helper.attach(helper.runtime())
        controller, worker = self.attach_signal(runtime, policy="green")
        worker.publish("RED")
        runtime.infer()
        publisher = runtime.path_publisher
        self.assertEqual(self.payload(runtime)["target_speed_mps"], 0.)
        self.assertGreater(publisher._safety_baseline["target"], .05)
        runtime.test_status['ego']['speed_kmh'] = 0.
        worker.publish("GREEN")
        helper.advance(runtime, .1, [])
        runtime.model = lambda *_: self.fail("green release waited for model inference")
        now = time.monotonic()
        with patch('time.monotonic', return_value=now+.2):
            self.assertTrue(publisher.recheck_active_path())
        payload = self.payload(runtime)
        self.assertGreater(payload["target_speed_mps"], .05)
        self.assertFalse(payload["decision"]["emergency"])
        self.assertIsNone(payload["decision"]["validation"]["traffic_signal"]["cap_mps"])

    def test_fourth_signal_only_red_plus_arrow_survives_final_publisher_checks(self):
        for state in ("UNKNOWN", "RED", "YELLOW", "GREEN", "LEFT", "GREEN_LEFT", "RED_LEFT"):
            with self.subTest(state=state):
                runtime = avoidance_tests.AvoidanceGateTests().runtime(action="KEEP")
                controller, worker = self.attach_signal(runtime, policy="red_left_only")
                worker.publish(state)
                runtime.infer()
                target = self.payload(runtime)["target_speed_mps"]
                self.assertEqual(target > .05, state == "RED_LEFT")


if __name__ == "__main__":
    unittest.main()
