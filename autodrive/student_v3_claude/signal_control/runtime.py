"""Adapt the portable signal code to the competition camera, map and publisher.

Only mapped traffic lights own a signal speed cap. Unsigned intersections,
roundabout yield lines and tunnel lane-control signs never enter this gate.
The worker uses camera 4 from the shared receiver, without another UDP bind
or changes to the obstacle semantic sidecar's class masks.
"""
import json
import math
import os
import threading
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

from .pipeline import SignalFramePipeline
from .signal_config import DEFAULT_CONFIG, load_config
from .signal_geometry import FRONT_BUMPER_M
from .signal_inference import make_signal_model
from .signal_path import SignalPathGate
from .debug_store import SignalDebugStore
from .visualization import signal_debug_images
from .camera import CAMERA_ID, CAMERA_VIEW
from .mapped_housing import MappedSignalHousing
from .permissions import GREEN, IGNORE, policy_for_line


def signal_enabled_from_env():
    value = os.environ.get("STUDENT_SIGNAL_GUARD", "1")
    if value not in ("0", "1"):
        raise ValueError("STUDENT_SIGNAL_GUARD must be 0 or 1")
    return value == "1"


def route_signal_maneuver(lanes, station):
    """Classify the driven path, not the junction link's signal annotation.

    K-city annotations describe other movements at some shared entrances:
    the first straight has right_unprotected, and the later right has left.
    Missing or incomplete geometry must never grant a right-turn exemption.
    """
    route = np.asarray(getattr(lanes, "route_xy", []), dtype=float)
    stations = np.asarray(getattr(lanes, "route_station", []), dtype=float)
    if (route.ndim != 2 or route.shape[1] < 2 or stations.ndim != 1 or
            len(route) != len(stations) or len(route) < 2 or
            not np.isfinite(route[:, :2]).all() or not np.isfinite(stations).all() or
            not np.all(np.diff(stations) > 0.) or
            station-12. < stations[0] or station+30. > stations[-1]):
        return "straight", None
    headings = []
    for start, end in ((-12., -2.), (20., 30.)):
        points = np.array([[np.interp(station+offset, stations, route[:, axis])
                            for axis in (0, 1)] for offset in (start, end)])
        delta = points[1]-points[0]
        if np.linalg.norm(delta) < 1.:
            return "straight", None
        headings.append(math.atan2(delta[1], delta[0]))
    turn = math.degrees((headings[1]-headings[0]+math.pi) % (2*math.pi)-math.pi)
    return ("right" if turn < -45. else "left" if turn > 45. else "straight"), turn


def mapped_signal_lines(lanes):
    links = {link["idx"]: link for link in lanes.links}
    lines = []
    for line in lanes.stop_lines:
        light = line.get("traffic_light_id")
        if not light or str(light).startswith("LCS"):
            continue
        link = links[line["entry_link_id"]]
        maneuver, turn = route_signal_maneuver(lanes, float(line["station_m"]))
        lines.append(dict(
            id=line["id"], link_id=line["entry_link_id"], station=float(line["station_m"]),
            maneuver=maneuver, traffic_light_id=light,
            maneuver_source="route" if turn is not None else "unknown",
            route_turn_deg=turn, mapped_related_signal=link.get("related_signal"),
            stop_line_source=line["source"], stop_line_node_id=line["node_id"],
            stop_line_xy=line["xy"], label=line["node_id"],
        ))
    lines.sort(key=lambda line: line["station"])
    index = 0
    for line in lines:
        policy = policy_for_line(line["stop_line_node_id"], line["maneuver"])
        if policy != IGNORE:
            index += 1
        line.update(signal_index=index if policy != IGNORE else None, permission_policy=policy)
    return lines


class SignalFrameWorker(threading.Thread):
    def __init__(self, receiver, run_id, config, model, *, source="live", debug_store=None):
        super().__init__(name="traffic-signal", daemon=True)
        self.receiver, self.run_id, self.config, self.model = receiver, run_id, config, model
        self.source = source
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.pipeline = SignalFramePipeline(config)
        self.target = None
        self.target_context = None
        self.report = {}
        self.last_stamp = None
        self.last_received = None
        self.debug_store = debug_store

    def select(self, key, context=None):
        with self.lock:
            if key != self.target:
                self.target = key
                self.pipeline.stable.reset()
                self.last_stamp, self.last_received, self.report = None, None, {}
            self.target_context = context

    def snapshot(self):
        with self.lock:
            return dict(self.report)

    def process_once(self):
        with self.lock:
            target = self.target
            target_context = self.target_context
        if target is None:
            return False
        # Camera source timestamps may come from a different PC's wall clock.
        # Freshness uses the local receipt of that exact frame, never poll time.
        with self.receiver.lock:
            frames = self.receiver.frames.get(CAMERA_VIEW, ())
            counts = self.receiver.counts.get(CAMERA_VIEW, ())
            if not frames or not counts:
                return False
            stamp, jpeg = frames[-1]
            received = float(counts[-1])
        now_mono, now_wall = time.monotonic(), time.time()
        with self.lock:
            if target != self.target or stamp == self.last_stamp:
                return False
            if (self.last_received is None or received - self.last_received > self.config["stale_seconds"]
                    or (self.last_stamp is not None and stamp < self.last_stamp)):
                self.pipeline.stable.reset()
            self.last_stamp, self.last_received = stamp, received
        age = now_mono - received
        if not 0. <= age <= self.config["stale_seconds"]:
            with self.lock:
                self.pipeline.stable.reset()
                self.report = {}
            return False
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("signal camera JPEG is invalid")
        mask = self.model.mask(image, self.config["semantic_threshold"])
        frame_config = dict(self.config)
        if target_context is not None:
            # A valid mapped-camera observation takes effect on this frame.
            # Previous RED/UNKNOWN or GREEN<->GREEN_LEFT must not delay release.
            frame_config.update(layout=target_context["layout"], roi=[0., 0., 1., 1.], stable_frames=1,
                                permission_policy=target_context.get("permission_policy", GREEN))
        with self.lock:
            if target != self.target:
                return False  # A finished inference cannot authorize a different junction.
            self.pipeline.set_config(frame_config)
            report, crop, masks = self.pipeline.process(
                image, mask, self.run_id, frame_wall=now_wall-age, source=self.source, target=target_context)
            report.update(line_id=target, camera_id=CAMERA_ID, camera_view=CAMERA_VIEW,
                          camera_stamp_ns=int(stamp), received_mono=received)
            self.report = report
        if self.debug_store is not None:
            try:
                images = signal_debug_images(image, mask, report, crop, masks, frame_config)
                self.debug_store.publish(report, frame_config, images)
            except Exception:
                pass  # A preview failure must never reset stability or change control.
        return True

    def run(self):
        while not self.stop_event.is_set():
            began = time.monotonic()
            try:
                self.process_once()
            except Exception as error:
                with self.lock:
                    self.pipeline.stable.reset()
                    self.report = {"error": str(error)[:240], "fresh": False}
            self.stop_event.wait(max(0., 1. / self.config["max_fps"] - (time.monotonic()-began)))

    def close(self):
        self.stop_event.set()
        if self.is_alive():
            self.join(timeout=4.)
        if hasattr(self.model, "close"):
            self.model.close()


class SignalController:
    def __init__(self, lanes, receiver=None, *, enabled=True, run_id=None, worker=None,
                 config=None, model=None, source="live", status_path=None, projector=None):
        if not isinstance(enabled, bool):
            raise ValueError("signal enabled must be a boolean")
        self.enabled = enabled
        self.run_id = run_id or os.environ.get("STUDENT_SIGNAL_RUN_ID") or uuid.uuid4().hex
        self.lanes = lanes
        self.lines = mapped_signal_lines(lanes) if self.enabled else []
        self.config = config if config is not None else load_config() if enabled else dict(DEFAULT_CONFIG)
        self.gate = SignalPathGate(self.run_id, max_age=self.config["stale_seconds"])
        self.lock = threading.RLock()
        self.selected_id = None
        self.last_plan = {"active": False, "enabled": self.enabled, "run_id": self.run_id,
                          "camera_id": CAMERA_ID, "camera_view": CAMERA_VIEW}
        self.status_path = Path(status_path) if status_path else None
        self.last_status_write = 0.
        self.worker = worker
        self.projector = projector
        if self.enabled and self.worker is None:
            if receiver is None:
                raise ValueError("signal camera receiver is required")
            if self.projector is None:
                from competition_profiles import profile
                sensor = profile(os.environ.get("STUDENT_NETWORK_PROFILE", "single_pc"))["sensor_file"]
                self.projector = MappedSignalHousing(self.lanes, sensor)
            self.projector.validate(self.lines)
            default_model = (Path(__file__).resolve().parents[3] /
                             "semantic/models/semantic_b1_noyellow_v2_final_20261002/best_fp16.onnx")
            model = model if model is not None else make_signal_model(
                os.environ.get("SIGNAL_MODEL_PATH", str(default_model)))
            self.worker = SignalFrameWorker(receiver, self.run_id, self.config, model,
                                            source=source, debug_store=SignalDebugStore())

    def start(self):
        if self.enabled and self.worker is not None:
            self.worker.start()

    def close(self):
        if self.worker is not None:
            self.worker.close()

    def _line(self, station, speed_mps):
        remembered = self.gate.hold_line or self.gate.braking_target
        if remembered is not None and abs(remembered["station"]-station-FRONT_BUMPER_M) <= 5.:
            return remembered
        # At normal-road speed the approach envelope needs more than 60 m.
        # Start within the actual braking reserve and
        # keep the selected approach until passing; slowing must not deselect it.
        horizon = max(60., speed_mps*self.gate.reaction_seconds +
                      speed_mps**2/(2*self.gate.deceleration) + self.gate.stop_margin + 5.)
        for line in self.lines:
            gap = line["station"]-station-FRONT_BUMPER_M
            if gap >= -.3:
                if line.get("permission_policy") == IGNORE:
                    return line  # Do not select a later junction during this right turn.
                return line if gap <= horizon or line["id"] == self.selected_id else None
        return None

    def evaluate(self, ego, now=None):
        with self.lock:
            now = time.time() if now is None else float(now)
            if not self.enabled:
                return dict(self.last_plan)
            station = float(self.lanes.route_progress(ego))
            speed = abs(float(ego.get("speed_kmh", 0.)))/3.6
            if not math.isfinite(station) or not math.isfinite(speed):
                raise ValueError("signal route pose is invalid")
            line = self._line(station, speed)
            ignored = bool(line and line.get("permission_policy") == IGNORE)
            self.selected_id = line["id"] if line and not ignored else None
            perception_line = line
            if perception_line is None:
                # Start camera inference before the braking envelope activates.
                # Preparing a later observation never imposes a distant cap.
                perception_line = next((candidate for candidate in self.lines
                    if -.3 <= candidate["station"]-station-FRONT_BUMPER_M <= 120.), None)
            perception_ignored = bool(perception_line and perception_line.get("permission_policy") == IGNORE)
            target_id = perception_line["id"] if perception_line and not perception_ignored else None
            context = (self.projector.project(ego, perception_line)
                if self.projector is not None and target_id is not None else None)
            self.worker.select(target_id, context=context)
            report = self.worker.snapshot()
            perception_error = report.get("error")
            if (report.get("line_id") != target_id or
                    report.get("camera_id") != CAMERA_ID or report.get("camera_view") != CAMERA_VIEW or
                    (self.projector is not None and perception_line is not None and
                     report.get("traffic_light_id") != perception_line["traffic_light_id"]) or
                    not 0. <= time.monotonic()-report.get("received_mono", -math.inf) <= self.gate.max_age):
                report = {}
            plan = self.gate.evaluate(report, line, station, now, speed_mps=speed)
            plan.update(enabled=True, route_station_m=station,
                        camera_id=CAMERA_ID, camera_view=CAMERA_VIEW,
                        selected_line_id=self.selected_id,
                        perception_error=perception_error,
                        perception_reason=report.get("reason"),
                        detection_stage=report.get("detection_stage"),
                        target_box=report.get("target_box"),
                        stable_count=report.get("stable_count", 0),
                        frame_wall=report.get("frame_wall"), camera_stamp_ns=report.get("camera_stamp_ns"))
            self.last_plan = plan
            self._write_status()
            return dict(plan)

    def _write_status(self):
        now = time.monotonic()
        if self.status_path is None or now-self.last_status_write < .2:
            return
        self.last_status_write = now
        try:
            temporary = self.status_path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(dict(self.last_plan, time=time.time())))
            os.replace(temporary, self.status_path)
        except OSError:
            pass  # Diagnostic disk failures cannot release or prevent a signal stop.

    def apply(self, ego, target_mps, decision=None, *, control_before_signal=None):
        plan = self.evaluate(ego)
        # The publication boundary checks the signal twice. Keep the independently
        # checked control for timing; the first signal stop must not become an
        # apparent obstacle stop at the second check. Control still uses the
        # current target and decision below.
        timing_target, timing_decision = control_before_signal or (target_mps, decision)
        plan['pre_signal_cap_mps'] = min(float(timing_target), timing_decision.allowed_mps) if timing_decision else float(timing_target)
        plan['pre_signal_emergency'] = bool(timing_decision.emergency) if timing_decision else False
        cap, emergency = self.gate.apply(plan, target_mps, decision.emergency if decision else False)
        if decision is not None and self.enabled:
            reason = (plan.get("reason") if plan.get("stop_required") and
                      plan.get("cap_mps") is not None and plan["cap_mps"] <= target_mps
                      else decision.stop_reason)
            decision = decision.decided(allowed_mps=min(decision.allowed_mps, cap),
                emergency=emergency, stop_reason=reason, traffic_signal=plan)
        return cap, decision
