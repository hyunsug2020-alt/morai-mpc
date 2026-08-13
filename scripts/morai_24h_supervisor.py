#!/usr/bin/env python3
"""Supervise a long MORAI odometry and pedestrian-data experiment.

The supervisor never reads Ego pose for odometry.  Position is used only to
decide whether the simulator is stuck and to score validation summaries.
"""

import argparse
import fcntl
import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import rospy
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from std_msgs.msg import String


WORKSPACE = Path("/home/david/morai-mpc-agent-morai-lio-gps-integration")
LOG_ROOT = WORKSPACE / "src/morai_control/logs/longrun_20260801"
SUMMARY_GLOB = "longrun_accuracy_*_summary.json"


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(str(temporary), str(path))


class LongRunSupervisor:
    def __init__(self, args):
        self.args = args
        self.deadline = datetime.fromisoformat(args.deadline)
        self.lock = threading.RLock()
        self.samples = deque(maxlen=20000)
        self.last_ego_arrival = None
        self.pedestrians = 0
        self.obstacles = 0
        self.odom_diagnostics = None
        self.dataset_status = None
        self.recovery_count = 0
        self.recovery_failures = 0
        self.last_recovery = None
        self.last_recovery_reason = None
        self.dataset_inactive_since = None
        self.pedestrian_spawn_count = 0
        self.last_pedestrian_spawn = None
        self.dataset_revisit_count = 0
        self.last_dataset_revisit = None
        self.odom_process = None
        self.odom_log_stream = None
        self.seen_summaries = set()
        self.run_results = []
        self.stop_requested = False
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        (LOG_ROOT / "bags").mkdir(parents=True, exist_ok=True)
        self.pedestrian_state_path = LOG_ROOT / "pedestrian_spawn_state.json"
        if self.pedestrian_state_path.exists():
            try:
                with self.pedestrian_state_path.open(encoding="utf-8") as stream:
                    pedestrian_state = json.load(stream)
                self.pedestrian_spawn_count = int(
                    pedestrian_state.get("spawn_count") or 0)
                self.last_pedestrian_spawn = pedestrian_state.get(
                    "last_spawn")
            except (OSError, TypeError, ValueError):
                pass
        else:
            heartbeat_path = LOG_ROOT / "heartbeat.json"
            if heartbeat_path.exists():
                try:
                    with heartbeat_path.open(encoding="utf-8") as stream:
                        previous_heartbeat = json.load(stream)
                    self.pedestrian_spawn_count = int(
                        previous_heartbeat.get("pedestrian_spawn_count") or 0)
                    self.last_pedestrian_spawn = previous_heartbeat.get(
                        "last_pedestrian_spawn")
                except (OSError, TypeError, ValueError):
                    pass
        session_path = LOG_ROOT / "session.json"
        if session_path.exists():
            with session_path.open(encoding="utf-8") as stream:
                session = json.load(stream)
            self.started_at = datetime.fromisoformat(session["started_at"])
        else:
            self.started_at = datetime.now().astimezone()
            atomic_json(session_path, {
                "started_at": self.started_at.isoformat(),
                "deadline": self.deadline.isoformat(),
            })
        results_path = LOG_ROOT / "run_results.json"
        if results_path.exists():
            with results_path.open(encoding="utf-8") as stream:
                self.run_results = json.load(stream)
            self.run_results = [
                result for result in self.run_results
                if Path(result.get("summary", "")).name.startswith(
                    "longrun_accuracy_")
            ]
            self.seen_summaries = {
                result["summary"] for result in self.run_results
                if result.get("summary")
            }

        rospy.init_node("morai_24h_supervisor", disable_signals=True)
        rospy.Subscriber(
            "/Ego_topic", EgoVehicleStatus, self.ego_callback,
            queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(
            "/Object_topic", ObjectStatusList, self.object_callback,
            queue_size=20, tcp_nodelay=True)
        rospy.Subscriber(
            "/pure_odometry/diagnostics", String,
            self.odom_diagnostics_callback, queue_size=5)
        rospy.Subscriber(
            "/lidar_detection/dataset_status", String,
            self.dataset_status_callback, queue_size=5)

    def ego_callback(self, message):
        now = time.monotonic()
        sample = {
            "t": now,
            "x": float(message.position.x),
            "y": float(message.position.y),
            "speed": math.hypot(message.velocity.x, message.velocity.y),
            "accel": float(message.accel),
            "brake": float(message.brake),
        }
        with self.lock:
            self.samples.append(sample)
            self.last_ego_arrival = now

    def object_callback(self, message):
        with self.lock:
            self.pedestrians = int(message.num_of_pedestrian)
            self.obstacles = int(message.num_of_obstacle)

    def odom_diagnostics_callback(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            self.odom_diagnostics = value

    def dataset_status_callback(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            self.dataset_status = value
            if bool(value.get("enabled")):
                self.dataset_inactive_since = None
            elif self.dataset_inactive_since is None:
                self.dataset_inactive_since = time.monotonic()

    @staticmethod
    def percentile95(values):
        if not values:
            return 0.0
        ordered = sorted(values)
        return ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]

    def motion_window(self, duration):
        now = time.monotonic()
        with self.lock:
            values = [sample for sample in self.samples
                      if sample["t"] >= now - duration]
        if len(values) < 2:
            return None
        elapsed = values[-1]["t"] - values[0]["t"]
        displacement = math.hypot(
            values[-1]["x"] - values[0]["x"],
            values[-1]["y"] - values[0]["y"])
        return {
            "elapsed_s": elapsed,
            "displacement_m": displacement,
            "speed_p95_mps": self.percentile95(
                [sample["speed"] for sample in values]),
            "control_p95": self.percentile95([
                max(sample["accel"], sample["brake"])
                for sample in values
            ]),
            "last": values[-1],
        }

    def is_stuck(self):
        window = self.motion_window(self.args.stuck_seconds)
        if window is None:
            return False, window
        with self.lock:
            ego_age = (None if self.last_ego_arrival is None else
                       time.monotonic() - self.last_ego_arrival)
        stationary_stuck = bool(
            window["displacement_m"] < self.args.stuck_displacement
            and window["speed_p95_mps"] < self.args.stuck_speed)
        uncontrolled_roll = bool(
            window["control_p95"] < 0.02
            and window["speed_p95_mps"] < 0.5)
        stuck = bool(
            ego_age is not None
            and ego_age < 2.0
            and window["elapsed_s"] >= 0.9 * self.args.stuck_seconds
            and (stationary_stuck or uncontrolled_roll))
        return stuck, window

    @staticmethod
    def command(*arguments, timeout=10.0):
        return subprocess.run(
            list(arguments), check=False, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=timeout)

    def simulator_window(self):
        result = self.command(
            "xdotool", "search", "--onlyvisible", "--name", "^Simulator$")
        identifiers = [line.strip() for line in result.stdout.splitlines()
                       if line.strip().isdigit()]
        if not identifiers:
            raise RuntimeError("visible Simulator window not found")
        identifier = identifiers[-1]
        name = self.command("xdotool", "getwindowname", identifier).stdout.strip()
        if name != "Simulator":
            raise RuntimeError("unexpected simulator window: {}".format(name))
        return identifier

    def send_key(self, key):
        window = self.simulator_window()
        self.command("xdotool", "windowraise", window)
        focused = self.command(
            "xdotool", "windowfocus", "--sync", window, timeout=5.0)
        if focused.returncode != 0:
            raise RuntimeError("cannot focus Simulator: {}".format(
                focused.stdout.strip()))
        # MORAI/Unity ignores synthetic keyboard events while a side-panel
        # widget owns focus.  Move focus back to the 3-D viewport first.
        geometry = self.command(
            "xdotool", "getwindowgeometry", "--shell", window).stdout
        values = {}
        for line in geometry.splitlines():
            if "=" in line:
                name, value = line.split("=", 1)
                if value.isdigit():
                    values[name] = int(value)
        width = values.get("WIDTH", 1024)
        height = values.get("HEIGHT", 720)
        clicked = self.command(
            "xdotool", "mousemove", "--window", window,
            str(max(50, min(width - 50, int(width * 0.40)))),
            str(max(50, min(height - 50, int(height * 0.50)))),
            "click", "1")
        if clicked.returncode != 0:
            raise RuntimeError("cannot focus MORAI viewport: {}".format(
                clicked.stdout.strip()))
        time.sleep(0.25)
        sent = self.command(
            "xdotool", "key", "--clearmodifiers", key)
        if sent.returncode != 0:
            raise RuntimeError("cannot send {}: {}".format(
                key, sent.stdout.strip()))

    def click_simulator(self, x, y, modifier=None):
        window = self.simulator_window()
        self.command("xdotool", "windowraise", window)
        focused = self.command(
            "xdotool", "windowfocus", "--sync", window, timeout=5.0)
        if focused.returncode != 0:
            raise RuntimeError("cannot focus Simulator")
        moved = self.command(
            "xdotool", "mousemove", "--window", window,
            str(int(x)), str(int(y)))
        if moved.returncode != 0:
            raise RuntimeError("cannot move MORAI pointer")
        if modifier is None:
            clicked = self.command("xdotool", "click", "1")
        else:
            clicked = self.command(
                "xdotool", "keydown", modifier, "click", "1",
                "keyup", modifier)
        if clicked.returncode != 0:
            raise RuntimeError("cannot click MORAI viewport")

    def should_respawn_pedestrians(self):
        with self.lock:
            inactive_since = self.dataset_inactive_since
            pedestrians = self.pedestrians
            dataset = self.dataset_status
        if inactive_since is None or dataset is None:
            return False
        if (self.pedestrian_spawn_count
                >= self.args.maximum_pedestrian_spawns):
            return False
        if float(dataset.get("free_disk_gb") or 0.0) < 20.0:
            return False
        if int(dataset.get("writer_errors") or 0) != 0:
            return False
        return bool(
            time.monotonic() - inactive_since
            >= self.args.pedestrian_respawn_seconds)

    def should_revisit_dataset_scene(self):
        with self.lock:
            inactive_since = self.dataset_inactive_since
            dataset = self.dataset_status
        if inactive_since is None or dataset is None:
            return False
        if float(dataset.get("free_disk_gb") or 0.0) < 20.0:
            return False
        if int(dataset.get("writer_errors") or 0) != 0:
            return False
        if int(dataset.get("next_index") or 0) >= 30000:
            return False
        return bool(
            time.monotonic() - inactive_since
            >= self.args.dataset_revisit_seconds)

    def respawn_roadside_pedestrians(self):
        """Pause MORAI and place two Man1 actors beside the current route."""
        lock_path = "/tmp/morai-ui.lock"
        with open(lock_path, "a+", encoding="utf-8") as ui_lock:
            fcntl.flock(ui_lock.fileno(), fcntl.LOCK_EX)
            self.stop_odometry_run()
            entered_editor = False
            try:
                window = self.simulator_window()
                geometry = self.command(
                    "xdotool", "getwindowgeometry", "--shell",
                    window).stdout
                values = {}
                for line in geometry.splitlines():
                    if "=" in line:
                        name, value = line.split("=", 1)
                        if value.isdigit():
                            values[name] = int(value)
                width = values.get("WIDTH", 1024)
                height = values.get("HEIGHT", 731)
                before = self.pedestrians
                self.send_key("F2")
                entered_editor = True
                time.sleep(2.0)
                # Scenario Edit: pedestrian category, then Man1 prefab.
                self.click_simulator(12, 111)
                time.sleep(0.5)
                self.click_simulator(55, 126)
                time.sleep(0.5)
                for x_fraction, y_fraction in ((0.65, 0.73), (0.71, 0.67)):
                    self.click_simulator(
                        width * x_fraction, height * y_fraction,
                        modifier="Shift_L")
                    time.sleep(1.0)
                self.send_key("F2")
                entered_editor = False
                time.sleep(5.0)
                with self.lock:
                    after = self.pedestrians
                    self.dataset_inactive_since = time.monotonic()
                observed_added = max(0, after - before)
                self.pedestrian_spawn_count += 2
                self.last_pedestrian_spawn = (
                    datetime.now().astimezone().isoformat())
                atomic_json(self.pedestrian_state_path, {
                    "spawn_count": self.pedestrian_spawn_count,
                    "last_spawn": self.last_pedestrian_spawn,
                })
                print(
                    "pedestrian respawn: before={} after={} observed_added={} "
                    "cumulative_spawned={}".format(
                        before, after, observed_added,
                        self.pedestrian_spawn_count), flush=True)
            finally:
                if entered_editor:
                    try:
                        self.send_key("F2")
                        time.sleep(3.0)
                    except Exception as error:
                        print("failed to leave Scenario Edit: {}".format(
                            error), flush=True)
                with self.lock:
                    self.samples.clear()
                self.ensure_odometry_run()
                fcntl.flock(ui_lock.fileno(), fcntl.LOCK_UN)

    def wait_controller_activity(self, seconds):
        with self.lock:
            start_count = len(self.samples)
            start = None if not self.samples else dict(self.samples[-1])
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not self.stop_requested:
            time.sleep(1.0)
            with self.lock:
                current = None if not self.samples else dict(self.samples[-1])
                new_count = len(self.samples) - start_count
            if current is None or start is None or new_count <= 0:
                continue
            if (current["speed"] > 0.5 or current["accel"] > 0.05
                    or current["brake"] > 0.05):
                return True
        return False

    def stop_odometry_run(self):
        self.command(
            "rosnode", "kill", "/pure_odometry_analysis", timeout=8.0)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            listed = self.command("rosnode", "list", timeout=5.0).stdout
            if "/pure_odometry_analysis" not in listed:
                break
            time.sleep(0.5)
        if self.odom_process is not None:
            try:
                self.odom_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.odom_process.pid, signal.SIGINT)
            self.odom_process = None
        if self.odom_log_stream is not None:
            self.odom_log_stream.close()
            self.odom_log_stream = None
        self.collect_summaries()

    def ensure_odometry_run(self):
        listed = self.command("rosnode", "list", timeout=5.0).stdout
        if "/pure_odometry" in listed and "/pure_odometry_analysis" in listed:
            return
        log_path = LOG_ROOT / "odometry_roslaunch.log"
        self.odom_log_stream = log_path.open("a", encoding="utf-8")
        self.odom_process = subprocess.Popen(
            ["roslaunch", "morai_control", "odometry.launch",
             "start_analysis:=true", "analysis_gui:=false",
             "record_debug_bag:=true",
             "analysis_output_csv:={}".format(
                 LOG_ROOT / "longrun_accuracy.csv"),
             "bag_output_prefix:={}".format(
                 LOG_ROOT / "bags/longrun_debug")],
            cwd=str(WORKSPACE), stdout=self.odom_log_stream,
            stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(5.0)

    def recover_vehicle(self, reason):
        lock_path = "/tmp/morai-ui.lock"
        with open(lock_path, "a+", encoding="utf-8") as ui_lock:
            fcntl.flock(ui_lock.fileno(), fcntl.LOCK_EX)
            self.stop_odometry_run()
            recovered = False
            try:
                # In this competition build Built-In -> q -> Keyboard.
                self.send_key("q")
                time.sleep(2.0)
                self.send_key("i")
                time.sleep(10.0)
                for _attempt in range(4):
                    self.send_key("q")
                    if self.wait_controller_activity(15.0):
                        recovered = True
                        break
                if not recovered:
                    self.recovery_failures += 1
                self.recovery_count += 1
                self.last_recovery = datetime.now().astimezone().isoformat()
                self.last_recovery_reason = reason
                if reason == "dataset_scene_revisit":
                    self.dataset_revisit_count += 1
                    self.last_dataset_revisit = self.last_recovery
            except Exception as error:
                self.recovery_failures += 1
                with self.lock:
                    # Back off instead of crashing/restarting once per second
                    # when the Simulator window is temporarily unavailable.
                    self.dataset_inactive_since = time.monotonic()
                print("vehicle recovery failed ({}): {}".format(
                    reason, error), flush=True)
            finally:
                with self.lock:
                    self.samples.clear()
                self.ensure_odometry_run()
                fcntl.flock(ui_lock.fileno(), fcntl.LOCK_UN)
        return recovered

    def collect_summaries(self):
        summary_dir = LOG_ROOT
        for path in sorted(summary_dir.glob(SUMMARY_GLOB)):
            key = str(path.resolve())
            if key in self.seen_summaries:
                continue
            if path.stat().st_mtime < self.started_at.timestamp():
                continue
            self.seen_summaries.add(key)
            try:
                with path.open(encoding="utf-8") as stream:
                    summary = json.load(stream)
            except (OSError, ValueError):
                continue
            distance = float(summary.get("truth_distance_m") or 0.0)
            rmse = summary.get("position_rmse_m")
            odom_distance = float(summary.get("lio_distance_m") or 0.0)
            position_accuracy = None
            distance_accuracy = None
            overall_accuracy = None
            if distance > 0.0 and rmse is not None:
                position_accuracy = max(
                    0.0, 100.0 * (1.0 - float(rmse) / distance))
                distance_accuracy = max(
                    0.0, 100.0 * (
                        1.0 - abs(odom_distance / distance - 1.0)))
                overall_accuracy = min(position_accuracy, distance_accuracy)
            valid = bool(distance >= self.args.minimum_scored_distance)
            passed = bool(
                valid and overall_accuracy is not None
                and overall_accuracy >= self.args.accuracy_gate)
            self.run_results.append({
                "summary": key,
                "truth_distance_m": distance,
                "position_rmse_m": rmse,
                "position_accuracy_percent": position_accuracy,
                "distance_accuracy_percent": distance_accuracy,
                "overall_accuracy_percent": overall_accuracy,
                "valid_distance": valid,
                "passed_90_percent_gate": passed,
            })
        atomic_json(LOG_ROOT / "run_results.json", self.run_results)

    def heartbeat(self, window):
        with self.lock:
            now = time.monotonic()
            ego_age = (None if self.last_ego_arrival is None else
                       now - self.last_ego_arrival)
            odom = self.odom_diagnostics
            dataset = self.dataset_status
            pedestrians = self.pedestrians
            obstacles = self.obstacles
        valid_runs = [result for result in self.run_results
                      if result["valid_distance"]]
        passed_runs = [result for result in valid_runs
                       if result["passed_90_percent_gate"]]
        payload = {
            "started_at": self.started_at.isoformat(),
            "updated_at": datetime.now().astimezone().isoformat(),
            "deadline": self.deadline.isoformat(),
            "ego_age_s": ego_age,
            "motion_window": window,
            "pedestrians": pedestrians,
            "obstacles": obstacles,
            "recovery_count": self.recovery_count,
            "recovery_failures": self.recovery_failures,
            "last_recovery": self.last_recovery,
            "last_recovery_reason": self.last_recovery_reason,
            "pedestrian_spawn_count": self.pedestrian_spawn_count,
            "last_pedestrian_spawn": self.last_pedestrian_spawn,
            "dataset_revisit_count": self.dataset_revisit_count,
            "last_dataset_revisit": self.last_dataset_revisit,
            "odometry_diagnostics": odom,
            "dataset_status": dataset,
            "valid_accuracy_runs": len(valid_runs),
            "passed_accuracy_runs": len(passed_runs),
            "notion_ready": len(passed_runs) >= self.args.required_passes,
        }
        atomic_json(LOG_ROOT / "heartbeat.json", payload)

    def finish(self):
        self.stop_odometry_run()
        self.command(
            "rosservice", "call", "/lidar_dataset_recorder/set_enabled",
            "false", timeout=8.0)
        self.collect_summaries()
        valid_runs = [result for result in self.run_results
                      if result["valid_distance"]]
        passed_runs = [result for result in valid_runs
                       if result["passed_90_percent_gate"]]
        atomic_json(LOG_ROOT / "final_report.json", {
            "started_at": self.started_at.isoformat(),
            "finished_at": datetime.now().astimezone().isoformat(),
            "deadline": self.deadline.isoformat(),
            "valid_runs": len(valid_runs),
            "passed_runs": len(passed_runs),
            "required_passes": self.args.required_passes,
            "accuracy_gate_percent": self.args.accuracy_gate,
            "notion_ready": len(passed_runs) >= self.args.required_passes,
            "runs": self.run_results,
        })

    def run(self):
        self.collect_summaries()
        self.ensure_odometry_run()
        next_heartbeat = 0.0
        while not self.stop_requested and not rospy.is_shutdown():
            if datetime.now().astimezone() >= self.deadline:
                break
            stuck, window = self.is_stuck()
            now = time.monotonic()
            if now >= next_heartbeat:
                self.collect_summaries()
                self.heartbeat(window)
                next_heartbeat = now + 5.0
            if stuck:
                self.recover_vehicle("stationary_window")
                next_heartbeat = 0.0
            elif self.should_revisit_dataset_scene():
                self.recover_vehicle("dataset_scene_revisit")
                next_heartbeat = 0.0
            time.sleep(1.0)
        self.finish()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--deadline", default="2026-08-02T03:50:00+09:00")
    parser.add_argument("--stuck-seconds", type=float, default=180.0)
    parser.add_argument("--stuck-displacement", type=float, default=0.5)
    parser.add_argument("--stuck-speed", type=float, default=0.15)
    parser.add_argument("--minimum-scored-distance", type=float, default=30.0)
    parser.add_argument("--accuracy-gate", type=float, default=90.0)
    parser.add_argument("--required-passes", type=int, default=3)
    parser.add_argument(
        "--pedestrian-respawn-seconds", type=float, default=120.0)
    parser.add_argument(
        "--maximum-pedestrian-spawns", type=int, default=30)
    parser.add_argument(
        "--dataset-revisit-seconds", type=float, default=20.0)
    return parser.parse_args()


def main():
    args = parse_args()
    supervisor = LongRunSupervisor(args)

    def request_stop(_signum, _frame):
        supervisor.stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    supervisor.run()


if __name__ == "__main__":
    main()
