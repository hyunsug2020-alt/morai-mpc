#!/usr/bin/env python3
"""Read-only live ESKF validator GUI for MORAI.

The GUI compares /Ego_topic with /eskf/odom for validation and visualization
only. Ground-truth pose and heading are never published back to the estimator.
It deliberately exposes no tuning controls so opening the window cannot change
the running ESKF, MORAI, sensor, or network configuration.
"""

import collections
import json
import math
import os
import sys
import threading
import time

import matplotlib

matplotlib.use("Qt5Agg")
import numpy as np
import rospy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.collections import LineCollection
from matplotlib.figure import Figure
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from PyQt5 import QtCore, QtWidgets
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


PLOT_SAMPLES = 900
TRAJECTORY_SAMPLES = 20000
RATE_WINDOW_SEC = 5.0


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def odometry_yaw(message):
    orientation = message.pose.pose.orientation
    return euler_from_quaternion([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ])[2]


def finite(value):
    value = float(value)
    return value if math.isfinite(value) else 0.0


class EskfMonitorGui(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESKF Live Validator (read-only)")
        self.resize(1280, 820)
        self.move(40, 40)

        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.eskf_topic = rospy.get_param("~eskf_topic", "/eskf/odom")
        self.diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "/eskf/diagnostics")
        self.max_pair_age = float(rospy.get_param("~max_pair_age_sec", 0.25))
        self.ui_refresh_hz = max(
            1.0, float(rospy.get_param("~ui_refresh_hz", 20.0)))
        self.plot_refresh_hz = max(
            0.2, float(rospy.get_param("~plot_refresh_hz", 2.0)))
        self.plot_interval = 1.0 / self.plot_refresh_hz
        self.report_dir = os.path.abspath(os.path.expanduser(rospy.get_param(
            "~report_dir", os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), "logs"))))

        self.lock = threading.RLock()
        self.latest_truth = None
        self.latest_diagnostics = None
        self.last_sample = None
        self.sequence = 0
        self.auto_resets = 0
        self.ego_arrivals = collections.deque(maxlen=1000)
        self.eskf_arrivals = collections.deque(maxlen=1000)
        self.samples = collections.deque(maxlen=TRAJECTORY_SAMPLES)
        self._clear_plot_buffers_locked()
        self.cached_metrics = None
        self.last_plot_draw = 0.0

        self._build_ui()
        rospy.Subscriber(
            self.ego_topic, EgoVehicleStatus, self._ego_callback,
            queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(
            self.eskf_topic, Odometry, self._eskf_callback,
            queue_size=200, tcp_nodelay=True)
        rospy.Subscriber(
            self.diagnostics_topic, String, self._diagnostics_callback,
            queue_size=20)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        # Numeric labels update independently from the relatively expensive
        # plots, so the table remains responsive without stealing estimator
        # CPU time for unnecessary Matplotlib redraws.
        self.timer.start(max(20, int(round(1000.0 / self.ui_refresh_hz))))
        rospy.logwarn(
            "[ESKF GUI] validation only: %s is displayed but never fed to ESKF",
            self.ego_topic)

    def _clear_plot_buffers_locked(self):
        self.plot_index = collections.deque(maxlen=PLOT_SAMPLES)
        self.position_errors = collections.deque(maxlen=PLOT_SAMPLES)
        self.longitudinal_errors = collections.deque(maxlen=PLOT_SAMPLES)
        self.lateral_errors = collections.deque(maxlen=PLOT_SAMPLES)
        self.yaw_errors = collections.deque(maxlen=PLOT_SAMPLES)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        self.trajectory_figure = Figure(figsize=(5.2, 6.0))
        self.trajectory_axis = self.trajectory_figure.add_subplot(111)
        self.trajectory_axis.set_title(
            "Trajectory: GT (blue) vs ESKF (red) — gray=error")
        self.trajectory_axis.grid(True, alpha=0.3)
        self.trajectory_axis.set_aspect("equal", "datalim")
        self.trajectory_connectors = LineCollection(
            [], colors="0.65", linewidths=0.6, zorder=1)
        self.trajectory_axis.add_collection(self.trajectory_connectors)
        self.gt_line, = self.trajectory_axis.plot(
            [], [], color="tab:blue", linewidth=1.8,
            label="GT (Ego)", zorder=2)
        self.eskf_line, = self.trajectory_axis.plot(
            [], [], color="tab:red", linewidth=1.2,
            label="ESKF", zorder=3)
        self.gt_current, = self.trajectory_axis.plot(
            [], [], "o", color="tab:blue", markersize=6, zorder=4)
        self.eskf_current, = self.trajectory_axis.plot(
            [], [], "o", color="tab:red", markersize=6, zorder=4)
        self.trajectory_axis.legend(loc="best", fontsize=8)
        self.trajectory_canvas = FigureCanvas(self.trajectory_figure)
        root.addWidget(self.trajectory_canvas, 3)

        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 4)

        self.error_figure = Figure(figsize=(6.4, 4.5))
        self.position_axis = self.error_figure.add_subplot(211)
        self.yaw_axis = self.error_figure.add_subplot(
            212, sharex=self.position_axis)
        self.position_axis.grid(True, alpha=0.3)
        self.position_axis.set_ylabel("position error [m]")
        self.position_line, = self.position_axis.plot(
            [], [], color="tab:blue", label="|pos|")
        self.longitudinal_line, = self.position_axis.plot(
            [], [], color="tab:purple", linewidth=0.8,
            label="longitudinal")
        self.lateral_line, = self.position_axis.plot(
            [], [], color="tab:green", linewidth=0.8, label="lateral")
        self.position_axis.legend(loc="upper left", fontsize=8)
        self.yaw_axis.grid(True, alpha=0.3)
        self.yaw_axis.set_ylabel("yaw error [deg]")
        self.yaw_axis.set_xlabel("synchronized sample")
        self.yaw_line, = self.yaw_axis.plot([], [], color="tab:red")
        self.error_canvas = FigureCanvas(self.error_figure)
        right.addWidget(self.error_canvas, 4)

        self.summary_label = QtWidgets.QLabel(
            "waiting for synchronized /Ego_topic and /eskf/odom ...")
        self.summary_label.setStyleSheet(
            "font-family:monospace; font-size:13px; padding:3px;")
        right.addWidget(self.summary_label)

        self.comparison_label = QtWidgets.QLabel("")
        self.comparison_label.setStyleSheet(
            "font-family:monospace; font-size:12px; background:#111; "
            "color:#ddd; padding:5px;")
        right.addWidget(self.comparison_label)

        self.status_label = QtWidgets.QLabel(
            "waiting for /eskf/diagnostics ...")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet(
            "font-family:monospace; font-size:12px; background:#222; "
            "color:#9fe; padding:5px;")
        right.addWidget(self.status_label)

        self.params_label = QtWidgets.QLabel(self._parameter_text())
        self.params_label.setStyleSheet(
            "font-family:monospace; font-size:11px; background:#f0f0f0; "
            "color:#222; padding:4px;")
        right.addWidget(self.params_label)

        buttons = QtWidgets.QHBoxLayout()
        read_only = QtWidgets.QLabel("READ ONLY · 설정 변경 없음")
        read_only.setStyleSheet("font-weight:bold; color:#187a32;")
        buttons.addWidget(read_only)
        buttons.addStretch(1)
        reset_button = QtWidgets.QPushButton("화면 통계 초기화")
        reset_button.clicked.connect(self.reset_statistics)
        buttons.addWidget(reset_button)
        report_button = QtWidgets.QPushButton("리포트 저장")
        report_button.clicked.connect(self.save_report)
        buttons.addWidget(report_button)
        right.addLayout(buttons)

    @staticmethod
    def _parameter_value(name):
        value = rospy.get_param("/eskf_node/" + name, "-")
        if isinstance(value, float):
            return "%.4g" % value
        return str(value)

    def _parameter_text(self):
        names = [
            ("accel noise", "accel_noise_std"),
            ("gyro noise", "gyro_noise_std"),
            ("GPS variance", "gps_position_variance"),
            ("odom speed var", "odometry_speed_variance"),
            ("odom pos var", "odometry_position_variance"),
        ]
        values = ["%s=%s" % (label, self._parameter_value(name))
                  for label, name in names]
        return "현재 설정(읽기 전용) | " + " | ".join(values)

    def _ego_callback(self, message):
        now = time.monotonic()
        truth = {
            "arrival": now,
            "x": finite(message.position.x),
            "y": finite(message.position.y),
            "yaw": math.radians(finite(message.heading)),
            "speed": math.hypot(
                finite(message.velocity.x), finite(message.velocity.y)),
        }
        with self.lock:
            previous = self.latest_truth
            if previous is not None:
                jump = math.hypot(
                    truth["x"] - previous["x"],
                    truth["y"] - previous["y"])
                if jump > 25.0 and now - previous["arrival"] < 2.0:
                    self._reset_statistics_locked()
                    self.auto_resets += 1
            self.latest_truth = truth
            self.ego_arrivals.append(now)

    def _eskf_callback(self, message):
        now = time.monotonic()
        with self.lock:
            self.eskf_arrivals.append(now)
            truth = self.latest_truth
            if truth is None or now - truth["arrival"] > self.max_pair_age:
                return

            estimate_x = finite(message.pose.pose.position.x)
            estimate_y = finite(message.pose.pose.position.y)
            estimate_yaw = odometry_yaw(message)
            estimate_speed = math.hypot(
                finite(message.twist.twist.linear.x),
                finite(message.twist.twist.linear.y))
            delta_x = estimate_x - truth["x"]
            delta_y = estimate_y - truth["y"]
            cosine = math.cos(truth["yaw"])
            sine = math.sin(truth["yaw"])
            longitudinal = cosine * delta_x + sine * delta_y
            lateral = -sine * delta_x + cosine * delta_y
            position_error = math.hypot(delta_x, delta_y)
            yaw_error_deg = math.degrees(wrap_angle(
                estimate_yaw - truth["yaw"]))
            speed_error = estimate_speed - truth["speed"]

            self.sequence += 1
            sample = {
                "index": self.sequence,
                "time": time.time(),
                "gt_x": truth["x"],
                "gt_y": truth["y"],
                "gt_yaw_deg": math.degrees(truth["yaw"]),
                "gt_speed": truth["speed"],
                "eskf_x": estimate_x,
                "eskf_y": estimate_y,
                "eskf_yaw_deg": math.degrees(estimate_yaw),
                "eskf_speed": estimate_speed,
                "position_error": position_error,
                "longitudinal_error": longitudinal,
                "lateral_error": lateral,
                "yaw_error_deg": yaw_error_deg,
                "speed_error": speed_error,
                "pair_age_ms": 1000.0 * (now - truth["arrival"]),
            }
            self.samples.append(sample)
            self.last_sample = sample
            self.plot_index.append(self.sequence)
            self.position_errors.append(position_error)
            self.longitudinal_errors.append(longitudinal)
            self.lateral_errors.append(lateral)
            self.yaw_errors.append(yaw_error_deg)

    def _diagnostics_callback(self, message):
        try:
            diagnostics = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        with self.lock:
            self.latest_diagnostics = diagnostics

    @staticmethod
    def _rate(arrivals, now):
        recent = [stamp for stamp in arrivals if now - stamp <= RATE_WINDOW_SEC]
        if len(recent) < 2:
            return 0.0
        duration = recent[-1] - recent[0]
        return 0.0 if duration <= 0.0 else (len(recent) - 1) / duration

    @staticmethod
    def _grade(position_rmse, yaw_rmse):
        if position_rmse < 0.5 and yaw_rmse < 1.0:
            return "우수"
        if position_rmse < 1.0 and yaw_rmse < 2.0:
            return "양호"
        if position_rmse < 2.0 and yaw_rmse < 3.0:
            return "사용 가능"
        return "점검 필요"

    @staticmethod
    def _metrics(samples):
        if not samples:
            return None
        position = np.asarray(
            [sample["position_error"] for sample in samples])
        yaw = np.asarray([sample["yaw_error_deg"] for sample in samples])
        speed = np.asarray([sample["speed_error"] for sample in samples])
        return {
            "samples": len(samples),
            "duration_s": samples[-1]["time"] - samples[0]["time"],
            "position_mean_m": float(np.mean(position)),
            "position_rmse_m": float(np.sqrt(np.mean(position ** 2))),
            "position_p95_m": float(np.percentile(position, 95)),
            "position_max_m": float(np.max(position)),
            "yaw_rmse_deg": float(np.sqrt(np.mean(yaw ** 2))),
            "speed_rmse_mps": float(np.sqrt(np.mean(speed ** 2))),
        }

    def _diagnostics_text(self, diagnostics):
        if not diagnostics:
            return "waiting for /eskf/diagnostics ..."
        counters = diagnostics.get("counters", {})
        position_std = diagnostics.get("position_std_m", [None, None])
        return (
            "filter={mode} | GPS={gps} age={gps_age} | pos σ={pos_std}\n"
            "odom align={align} speed={speed}m/s age={odom_age} | "
            "time scale={scale} updates={scale_updates}\n"
            "GPS accept/reject={ga}/{gr} | odom pose={oa}/{orr} | "
            "speed={sa}/{sr} | IMU spike={spike} | invalid={invalid} | "
            "local jump={jump} | ZUPT block={zupt}"
        ).format(
            mode=diagnostics.get("mode", "-"),
            gps=diagnostics.get("gps_mode", "-"),
            gps_age=self._format_optional(diagnostics.get("gps_age_sec")),
            pos_std=self._format_pair(position_std),
            align=diagnostics.get("odometry_alignment_ready", False),
            speed=self._format_optional(
                diagnostics.get("odometry_speed_mps"), 3),
            odom_age=self._format_optional(
                diagnostics.get("odometry_speed_age_sec")),
            scale=self._format_optional(
                diagnostics.get("odometry_time_scale"), 4),
            scale_updates=diagnostics.get("odometry_time_scale_updates", 0),
            ga=counters.get("gps_accepted", 0),
            gr=counters.get("gps_rejected", 0),
            oa=counters.get("odometry_accepted", 0),
            orr=counters.get("odometry_rejected", 0),
            sa=counters.get("wheel_speed_accepted", 0),
            sr=counters.get("wheel_speed_rejected", 0),
            spike=counters.get("imu_spikes", 0),
            invalid=counters.get("invalid_measurements", 0),
            jump=diagnostics.get("odometry_local_jumps", 0),
            zupt=diagnostics.get("zupt_blocked_by_odometry", 0),
        )

    @staticmethod
    def _format_optional(value, digits=3):
        if value is None:
            return "-"
        try:
            return ("%%.%df" % digits) % float(value)
        except (TypeError, ValueError):
            return "-"

    @classmethod
    def _format_pair(cls, values):
        if not isinstance(values, (list, tuple)) or len(values) < 2:
            return "-"
        return "(%s,%s)m" % (
            cls._format_optional(values[0]), cls._format_optional(values[1]))

    def reset_statistics(self):
        with self.lock:
            self._reset_statistics_locked()

    def _reset_statistics_locked(self):
        self.samples.clear()
        self.last_sample = None
        self._clear_plot_buffers_locked()
        self.cached_metrics = None
        self.last_plot_draw = 0.0

    def save_report(self):
        with self.lock:
            samples = list(self.samples)
            diagnostics = dict(self.latest_diagnostics or {})
        metrics = self._metrics(samples)
        if metrics is None or len(samples) < 5:
            self.summary_label.setText("리포트 저장 실패: 동기화 샘플이 부족함")
            return
        os.makedirs(self.report_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(
            self.report_dir, "eskf_live_report_%s.png" % stamp)
        json_path = os.path.join(
            self.report_dir, "eskf_live_report_%s.json" % stamp)

        figure = Figure(figsize=(12, 5))
        trajectory = figure.add_subplot(121)
        gt_x = [sample["gt_x"] for sample in samples]
        gt_y = [sample["gt_y"] for sample in samples]
        es_x = [sample["eskf_x"] for sample in samples]
        es_y = [sample["eskf_y"] for sample in samples]
        trajectory.plot(gt_x, gt_y, color="tab:blue", label="GT (Ego)")
        trajectory.plot(es_x, es_y, color="tab:red", label="ESKF")
        trajectory.set_aspect("equal", "datalim")
        trajectory.grid(True, alpha=0.3)
        trajectory.legend()
        trajectory.set_title("Trajectory: GT vs ESKF")
        error_axis = figure.add_subplot(122)
        error_axis.plot(
            [sample["position_error"] for sample in samples],
            color="tab:purple", label="position error")
        error_axis.axhline(
            metrics["position_rmse_m"], color="tab:red", linestyle="--",
            label="RMSE %.3fm" % metrics["position_rmse_m"])
        error_axis.grid(True, alpha=0.3)
        error_axis.legend()
        error_axis.set_title("Position error [m]")
        figure.tight_layout()
        figure.savefig(image_path, dpi=130)

        payload = {
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "validation_only": True,
            "topics": {
                "ground_truth": self.ego_topic,
                "estimate": self.eskf_topic,
                "diagnostics": self.diagnostics_topic,
            },
            "metrics": metrics,
            "grade": self._grade(
                metrics["position_rmse_m"], metrics["yaw_rmse_deg"]),
            "diagnostics": diagnostics,
            "image": image_path,
        }
        with open(json_path, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, ensure_ascii=False,
                      sort_keys=True)
            stream.write("\n")
        self.summary_label.setText(
            "리포트 저장 완료: %s | RMSE %.3fm" % (
                json_path, metrics["position_rmse_m"]))
        rospy.loginfo(
            "[ESKF GUI] report saved: %s, %s", image_path, json_path)

    def refresh(self):
        now = time.monotonic()
        plot_due = now - self.last_plot_draw >= self.plot_interval
        with self.lock:
            last_sample = dict(self.last_sample) if self.last_sample else None
            diagnostics = dict(self.latest_diagnostics or {})
            ego_rate = self._rate(self.ego_arrivals, now)
            eskf_rate = self._rate(self.eskf_arrivals, now)
            auto_resets = self.auto_resets
            metrics = self.cached_metrics
            if plot_due:
                samples = list(self.samples)
                plot_index = list(self.plot_index)
                position_errors = list(self.position_errors)
                longitudinal_errors = list(self.longitudinal_errors)
                lateral_errors = list(self.lateral_errors)
                yaw_errors = list(self.yaw_errors)

        if plot_due:
            metrics = self._metrics(samples)
            with self.lock:
                self.cached_metrics = metrics
                self.last_plot_draw = now
        self.status_label.setText(self._diagnostics_text(diagnostics))
        if metrics is not None and last_sample is not None:
            grade = self._grade(
                metrics["position_rmse_m"], metrics["yaw_rmse_deg"])
            self.summary_label.setText(
                "n=%-6d | pos %.3fm (lon %+.3f lat %+.3f) | "
                "yaw %+.2f° | speed Δ%+.2fm/s\n"
                "RMSE pos %.3fm | p95 %.3fm | max %.3fm | "
                "RMSE yaw %.2f° | 등급: %s"
                % (
                    metrics["samples"], last_sample["position_error"],
                    last_sample["longitudinal_error"],
                    last_sample["lateral_error"],
                    last_sample["yaw_error_deg"],
                    last_sample["speed_error"],
                    metrics["position_rmse_m"],
                    metrics["position_p95_m"],
                    metrics["position_max_m"],
                    metrics["yaw_rmse_deg"], grade))
            self.comparison_label.setText(
                "             %12s %12s %11s\n"
                "x [m]       %12.3f %12.3f Δ%+9.3f\n"
                "y [m]       %12.3f %12.3f Δ%+9.3f\n"
                "yaw [deg]   %12.2f %12.2f Δ%+9.2f\n"
                "speed [m/s] %12.3f %12.3f Δ%+9.3f\n"
                "rate        Ego %6.1f Hz | ESKF %6.1f Hz | "
                "pair %5.1f ms | UI age %5.1f ms | reset %d"
                % (
                    "Ego (GT)", "ESKF", "차이",
                    last_sample["gt_x"], last_sample["eskf_x"],
                    last_sample["eskf_x"] - last_sample["gt_x"],
                    last_sample["gt_y"], last_sample["eskf_y"],
                    last_sample["eskf_y"] - last_sample["gt_y"],
                    last_sample["gt_yaw_deg"],
                    last_sample["eskf_yaw_deg"],
                    last_sample["yaw_error_deg"],
                    last_sample["gt_speed"], last_sample["eskf_speed"],
                    last_sample["speed_error"],
                    ego_rate, eskf_rate, last_sample["pair_age_ms"],
                    max(0.0, 1000.0 * (time.time() - last_sample["time"])),
                    auto_resets))
            if plot_due:
                self._draw_trajectory(samples)
                self._draw_errors(
                    plot_index, position_errors, longitudinal_errors,
                    lateral_errors, yaw_errors)

    def _draw_trajectory(self, samples):
        if not samples:
            self.gt_line.set_data([], [])
            self.eskf_line.set_data([], [])
            self.gt_current.set_data([], [])
            self.eskf_current.set_data([], [])
            self.trajectory_connectors.set_segments([])
            self.trajectory_canvas.draw_idle()
            return
        display_step = max(1, len(samples) // 3000)
        shown = samples[::display_step]
        gt_x = [sample["gt_x"] for sample in shown]
        gt_y = [sample["gt_y"] for sample in shown]
        es_x = [sample["eskf_x"] for sample in shown]
        es_y = [sample["eskf_y"] for sample in shown]
        connector_step = max(1, len(shown) // 50)
        segments = [
            [(gt_x[index], gt_y[index]), (es_x[index], es_y[index])]
            for index in range(0, len(shown), connector_step)
        ]
        self.trajectory_connectors.set_segments(segments)
        self.gt_line.set_data(gt_x, gt_y)
        self.eskf_line.set_data(es_x, es_y)
        self.gt_current.set_data([gt_x[-1]], [gt_y[-1]])
        self.eskf_current.set_data([es_x[-1]], [es_y[-1]])
        self.trajectory_axis.relim()
        self.trajectory_axis.autoscale_view()
        self.trajectory_canvas.draw_idle()

    def _draw_errors(self, indices, position, longitudinal, lateral, yaw):
        self.position_line.set_data(indices, position)
        self.longitudinal_line.set_data(indices, longitudinal)
        self.lateral_line.set_data(indices, lateral)
        self.yaw_line.set_data(indices, yaw)
        self.position_axis.relim()
        self.position_axis.autoscale_view()
        self.yaw_axis.relim()
        self.yaw_axis.autoscale_view()
        self.error_canvas.draw_idle()


def main():
    rospy.init_node("eskf_monitor_gui", anonymous=False, disable_signals=True)
    application = QtWidgets.QApplication(sys.argv)
    gui = EskfMonitorGui()
    gui.show()
    shutdown_timer = QtCore.QTimer()
    shutdown_timer.timeout.connect(
        lambda: application.quit() if rospy.is_shutdown() else None)
    shutdown_timer.start(200)
    return application.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
