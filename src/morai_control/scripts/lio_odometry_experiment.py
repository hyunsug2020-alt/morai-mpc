#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Live LIO-SAM versus MORAI ground-truth experiment.

The MORAI ground-truth topic is used only by this validation node.  It is never
republished and never enters localization or control.
"""

import csv
import json
import math
import os
import threading
from collections import deque
from datetime import datetime

import numpy as np
import rospy
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def message_stamp(header):
    if header.stamp != rospy.Time():
        return header.stamp.to_sec()
    return rospy.Time.now().to_sec()


def odometry_yaw(msg):
    quaternion = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    ]
    return euler_from_quaternion(quaternion)[2]


class LioOdometryExperiment:
    CSV_FIELDS = [
        "ros_time", "elapsed_s", "truth_stamp", "odom_stamp", "sync_dt_s",
        "truth_x_m", "truth_y_m", "truth_yaw_deg", "truth_speed_mps",
        "lio_raw_x_m", "lio_raw_y_m", "lio_raw_yaw_deg",
        "lio_base_x_m", "lio_base_y_m",
        "lio_aligned_x_m", "lio_aligned_y_m", "lio_aligned_yaw_deg",
        "lio_speed_mps", "error_x_m", "error_y_m", "position_error_m",
        "along_track_error_m", "cross_track_error_m", "yaw_error_deg",
        "speed_error_mps", "truth_distance_m", "lio_distance_m",
        "position_rmse_m", "cross_track_rmse_m", "yaw_rmse_deg",
        "position_max_m", "lio_degenerate", "gps_status", "gps_age_s",
        "gps_shadow", "status",
    ]

    def __init__(self):
        rospy.init_node("lio_odometry_experiment")
        self.lock = threading.RLock()

        self.truth_topic = rospy.get_param("~ground_truth_topic", "/Ego_topic")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/lio_sam/mapping/odometry")
        self.incremental_topic = rospy.get_param(
            "~incremental_topic",
            "/lio_sam/mapping/odometry_incremental")
        self.gps_topic = rospy.get_param("~gps_topic", "/gps")
        self.heading_mode = rospy.get_param(
            "~ground_truth_heading_mode", "ros_yaw_deg")
        self.max_sync_delta = float(rospy.get_param(
            "~max_sync_delta_s", 0.25))
        self.alignment_samples_needed = max(
            1, int(rospy.get_param("~alignment_samples", 20)))
        self.gps_shadow_timeout = float(rospy.get_param(
            "~gps_shadow_timeout_s", 1.0))
        self.degeneracy_timeout = float(rospy.get_param(
            "~degeneracy_timeout_s", 0.5))
        self.drift_warning_m = float(rospy.get_param(
            "~drift_warning_m", 1.0))
        self.yaw_warning_deg = float(rospy.get_param(
            "~yaw_warning_deg", 2.0))
        self.lidar_x = float(rospy.get_param("~lidar_x", 1.676))
        self.lidar_y = float(rospy.get_param("~lidar_y", 0.005))
        self.enable_gui = bool(rospy.get_param("~enable_gui", True))
        self.gui_rate_hz = max(
            1.0, float(rospy.get_param("~gui_rate_hz", 10.0)))
        self.plot_history = max(
            100, int(rospy.get_param("~plot_history_samples", 3000)))

        output_csv = os.path.expanduser(rospy.get_param(
            "~output_csv",
            "/tmp/lio_odometry_experiment.csv"))
        unique_output = bool(rospy.get_param("~unique_output", True))
        self.output_csv = self._unique_path(output_csv, unique_output)
        self.summary_json = os.path.splitext(self.output_csv)[0] + "_summary.json"
        output_dir = os.path.dirname(self.output_csv)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        self.csv_stream = open(
            self.output_csv, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(
            self.csv_stream, fieldnames=self.CSV_FIELDS)
        self.csv_writer.writeheader()
        self.rows_since_flush = 0

        self.start_time = rospy.Time.now().to_sec()
        self.truth_queue = deque(maxlen=400)
        self.alignment_pairs = []
        self.alignment = None
        self.last_truth = None
        self.last_odom_receipt = None
        self.last_incremental_receipt = None
        self.last_gps_receipt = None
        self.gps_status = None
        self.degenerate = False

        self.sample_count = 0
        self.skipped_sync = 0
        self.position_sq_sum = 0.0
        self.cross_sq_sum = 0.0
        self.yaw_sq_sum = 0.0
        self.position_max = 0.0
        self.truth_distance = 0.0
        self.lio_distance = 0.0
        self.previous_truth_xy = None
        self.previous_lio_xy = None
        self.status_counts = {}
        self.latest_metrics = None

        self.history = {
            "time": deque(maxlen=self.plot_history),
            "truth_x": deque(maxlen=self.plot_history),
            "truth_y": deque(maxlen=self.plot_history),
            "lio_x": deque(maxlen=self.plot_history),
            "lio_y": deque(maxlen=self.plot_history),
            "position": deque(maxlen=self.plot_history),
            "along": deque(maxlen=self.plot_history),
            "cross": deque(maxlen=self.plot_history),
            "yaw": deque(maxlen=self.plot_history),
            "gps_shadow": deque(maxlen=self.plot_history),
            "degenerate": deque(maxlen=self.plot_history),
        }

        rospy.Subscriber(
            self.truth_topic, EgoVehicleStatus,
            self.truth_callback, queue_size=100)
        rospy.Subscriber(
            self.odom_topic, Odometry,
            self.odom_callback, queue_size=100)
        rospy.Subscriber(
            self.incremental_topic, Odometry,
            self.incremental_callback, queue_size=100)
        rospy.Subscriber(
            self.gps_topic, GPSMessage,
            self.gps_callback, queue_size=50)
        rospy.on_shutdown(self.shutdown)

        self.figure = None
        if self.enable_gui:
            self._initialize_gui()

        rospy.logwarn(
            "[LIO experiment] %s is validation-only ground truth",
            self.truth_topic)
        rospy.loginfo(
            "[LIO experiment] odom=%s incremental=%s csv=%s",
            self.odom_topic, self.incremental_topic, self.output_csv)

    @staticmethod
    def _unique_path(path, enabled):
        root, extension = os.path.splitext(path)
        extension = extension or ".csv"
        if not enabled:
            return root + extension
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return "{}_{}{}".format(root, stamp, extension)

    def truth_callback(self, msg):
        yaw_deg = float(msg.heading)
        if self.heading_mode == "morai_north_cw_deg":
            yaw = math.radians(90.0 - yaw_deg)
        else:
            yaw = math.radians(yaw_deg)
        sample = {
            "stamp": message_stamp(msg.header),
            "x": float(msg.position.x),
            "y": float(msg.position.y),
            "yaw": wrap_angle(yaw),
            "speed": math.hypot(msg.velocity.x, msg.velocity.y),
        }
        with self.lock:
            self.truth_queue.append(sample)
            self.last_truth = sample

    def gps_callback(self, msg):
        with self.lock:
            self.last_gps_receipt = rospy.Time.now().to_sec()
            self.gps_status = int(msg.status)

    def incremental_callback(self, msg):
        with self.lock:
            self.last_incremental_receipt = rospy.Time.now().to_sec()
            self.degenerate = bool(msg.pose.covariance[0] >= 0.5)

    def _nearest_truth(self, stamp):
        if not self.truth_queue:
            return None, math.inf
        truth = min(
            self.truth_queue,
            key=lambda sample: abs(sample["stamp"] - stamp))
        return truth, abs(truth["stamp"] - stamp)

    def _base_pose(self, msg):
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)
        yaw = wrap_angle(odometry_yaw(msg))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        base_x = raw_x - (
            cos_yaw * self.lidar_x - sin_yaw * self.lidar_y)
        base_y = raw_y - (
            sin_yaw * self.lidar_x + cos_yaw * self.lidar_y)
        return raw_x, raw_y, base_x, base_y, yaw

    def _update_alignment(self, truth, base_x, base_y, yaw):
        self.alignment_pairs.append((
            truth["x"], truth["y"], truth["yaw"], base_x, base_y, yaw))
        if len(self.alignment_pairs) < self.alignment_samples_needed:
            return False

        yaw_offsets = [
            wrap_angle(pair[2] - pair[5])
            for pair in self.alignment_pairs]
        yaw_offset = math.atan2(
            sum(math.sin(value) for value in yaw_offsets),
            sum(math.cos(value) for value in yaw_offsets))
        cos_yaw = math.cos(yaw_offset)
        sin_yaw = math.sin(yaw_offset)
        translations_x = []
        translations_y = []
        for truth_x, truth_y, _, local_x, local_y, _ in self.alignment_pairs:
            translations_x.append(
                truth_x - (cos_yaw * local_x - sin_yaw * local_y))
            translations_y.append(
                truth_y - (sin_yaw * local_x + cos_yaw * local_y))
        self.alignment = {
            "x": float(np.median(translations_x)),
            "y": float(np.median(translations_y)),
            "yaw": yaw_offset,
            "samples": len(self.alignment_pairs),
        }
        rospy.loginfo(
            "[LIO experiment] alignment ready: x=%.3f y=%.3f yaw=%.3fdeg",
            self.alignment["x"], self.alignment["y"],
            math.degrees(self.alignment["yaw"]))
        return True

    def _align_pose(self, x, y, yaw):
        cos_yaw = math.cos(self.alignment["yaw"])
        sin_yaw = math.sin(self.alignment["yaw"])
        aligned_x = (
            cos_yaw * x - sin_yaw * y + self.alignment["x"])
        aligned_y = (
            sin_yaw * x + cos_yaw * y + self.alignment["y"])
        aligned_yaw = wrap_angle(yaw + self.alignment["yaw"])
        return aligned_x, aligned_y, aligned_yaw

    def _gps_state(self, now):
        if self.last_gps_receipt is None:
            return math.inf, True
        age = max(0.0, now - self.last_gps_receipt)
        invalid_status = self.gps_status is not None and self.gps_status < 0
        return age, bool(age > self.gps_shadow_timeout or invalid_status)

    def odom_callback(self, msg):
        now = rospy.Time.now().to_sec()
        odom_stamp = message_stamp(msg.header)
        raw_x, raw_y, base_x, base_y, local_yaw = self._base_pose(msg)
        lio_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        with self.lock:
            self.last_odom_receipt = now
            truth, sync_delta = self._nearest_truth(odom_stamp)
            if truth is None or sync_delta > self.max_sync_delta:
                self.skipped_sync += 1
                return
            if self.alignment is None:
                self._update_alignment(
                    truth, base_x, base_y, local_yaw)
                return

            aligned_x, aligned_y, aligned_yaw = self._align_pose(
                base_x, base_y, local_yaw)
            error_x = aligned_x - truth["x"]
            error_y = aligned_y - truth["y"]
            position_error = math.hypot(error_x, error_y)
            cos_truth = math.cos(truth["yaw"])
            sin_truth = math.sin(truth["yaw"])
            along_error = cos_truth * error_x + sin_truth * error_y
            cross_error = -sin_truth * error_x + cos_truth * error_y
            yaw_error = wrap_angle(aligned_yaw - truth["yaw"])
            speed_error = lio_speed - truth["speed"]

            truth_xy = np.array([truth["x"], truth["y"]])
            lio_xy = np.array([aligned_x, aligned_y])
            if self.previous_truth_xy is not None:
                self.truth_distance += float(np.linalg.norm(
                    truth_xy - self.previous_truth_xy))
            if self.previous_lio_xy is not None:
                self.lio_distance += float(np.linalg.norm(
                    lio_xy - self.previous_lio_xy))
            self.previous_truth_xy = truth_xy
            self.previous_lio_xy = lio_xy

            self.sample_count += 1
            self.position_sq_sum += position_error**2
            self.cross_sq_sum += cross_error**2
            self.yaw_sq_sum += yaw_error**2
            self.position_max = max(self.position_max, position_error)
            position_rmse = math.sqrt(
                self.position_sq_sum / self.sample_count)
            cross_rmse = math.sqrt(
                self.cross_sq_sum / self.sample_count)
            yaw_rmse_deg = math.degrees(math.sqrt(
                self.yaw_sq_sum / self.sample_count))

            gps_age, gps_shadow = self._gps_state(now)
            degeneracy_fresh = (
                self.last_incremental_receipt is not None
                and now - self.last_incremental_receipt
                <= self.degeneracy_timeout)
            lio_degenerate = bool(degeneracy_fresh and self.degenerate)
            status_parts = []
            if gps_shadow:
                status_parts.append("GPS_SHADOW")
            if lio_degenerate:
                status_parts.append("LIO_DEGENERATE")
            if position_error >= self.drift_warning_m:
                status_parts.append("POSITION_DRIFT")
            if abs(math.degrees(yaw_error)) >= self.yaw_warning_deg:
                status_parts.append("YAW_DRIFT")
            status = "|".join(status_parts) if status_parts else "OK"
            self.status_counts[status] = self.status_counts.get(status, 0) + 1

            elapsed = now - self.start_time
            row = {
                "ros_time": "{:.9f}".format(now),
                "elapsed_s": "{:.3f}".format(elapsed),
                "truth_stamp": "{:.9f}".format(truth["stamp"]),
                "odom_stamp": "{:.9f}".format(odom_stamp),
                "sync_dt_s": "{:.6f}".format(sync_delta),
                "truth_x_m": "{:.6f}".format(truth["x"]),
                "truth_y_m": "{:.6f}".format(truth["y"]),
                "truth_yaw_deg": "{:.6f}".format(
                    math.degrees(truth["yaw"])),
                "truth_speed_mps": "{:.6f}".format(truth["speed"]),
                "lio_raw_x_m": "{:.6f}".format(raw_x),
                "lio_raw_y_m": "{:.6f}".format(raw_y),
                "lio_raw_yaw_deg": "{:.6f}".format(
                    math.degrees(local_yaw)),
                "lio_base_x_m": "{:.6f}".format(base_x),
                "lio_base_y_m": "{:.6f}".format(base_y),
                "lio_aligned_x_m": "{:.6f}".format(aligned_x),
                "lio_aligned_y_m": "{:.6f}".format(aligned_y),
                "lio_aligned_yaw_deg": "{:.6f}".format(
                    math.degrees(aligned_yaw)),
                "lio_speed_mps": "{:.6f}".format(lio_speed),
                "error_x_m": "{:.6f}".format(error_x),
                "error_y_m": "{:.6f}".format(error_y),
                "position_error_m": "{:.6f}".format(position_error),
                "along_track_error_m": "{:.6f}".format(along_error),
                "cross_track_error_m": "{:.6f}".format(cross_error),
                "yaw_error_deg": "{:.6f}".format(
                    math.degrees(yaw_error)),
                "speed_error_mps": "{:.6f}".format(speed_error),
                "truth_distance_m": "{:.6f}".format(self.truth_distance),
                "lio_distance_m": "{:.6f}".format(self.lio_distance),
                "position_rmse_m": "{:.6f}".format(position_rmse),
                "cross_track_rmse_m": "{:.6f}".format(cross_rmse),
                "yaw_rmse_deg": "{:.6f}".format(yaw_rmse_deg),
                "position_max_m": "{:.6f}".format(self.position_max),
                "lio_degenerate": int(lio_degenerate),
                "gps_status": (
                    "" if self.gps_status is None else self.gps_status),
                "gps_age_s": (
                    "inf" if math.isinf(gps_age)
                    else "{:.6f}".format(gps_age)),
                "gps_shadow": int(gps_shadow),
                "status": status,
            }
            self.csv_writer.writerow(row)
            self.rows_since_flush += 1
            if self.rows_since_flush >= 10:
                self.csv_stream.flush()
                self.rows_since_flush = 0

            self.latest_metrics = {
                "elapsed": elapsed,
                "position_error": position_error,
                "along_error": along_error,
                "cross_error": cross_error,
                "yaw_error_deg": math.degrees(yaw_error),
                "position_rmse": position_rmse,
                "cross_rmse": cross_rmse,
                "yaw_rmse_deg": yaw_rmse_deg,
                "gps_age": gps_age,
                "gps_shadow": gps_shadow,
                "degenerate": lio_degenerate,
                "status": status,
                "sync_delta": sync_delta,
            }
            values = {
                "time": elapsed,
                "truth_x": truth["x"],
                "truth_y": truth["y"],
                "lio_x": aligned_x,
                "lio_y": aligned_y,
                "position": position_error,
                "along": along_error,
                "cross": cross_error,
                "yaw": math.degrees(yaw_error),
                "gps_shadow": int(gps_shadow),
                "degenerate": int(lio_degenerate),
            }
            for key, value in values.items():
                self.history[key].append(value)

    def _initialize_gui(self):
        if not os.environ.get("DISPLAY"):
            rospy.logwarn(
                "[LIO experiment] DISPLAY is unset; CSV-only mode")
            self.enable_gui = False
            return
        try:
            import matplotlib.pyplot as plt
            import matplotlib.gridspec as gridspec
            self.plt = plt
            plt.ion()
            self.figure = plt.figure(figsize=(16, 9))
            self.figure.suptitle(
                "LIO-SAM Odometry vs MORAI Ground Truth",
                fontsize=14, fontweight="bold")
            grid = gridspec.GridSpec(
                2, 3, figure=self.figure, hspace=0.35, wspace=0.30)

            self.ax_trajectory = self.figure.add_subplot(grid[:, 0])
            self.truth_line, = self.ax_trajectory.plot(
                [], [], "k-", linewidth=1.5, label="MORAI ground truth")
            self.lio_line, = self.ax_trajectory.plot(
                [], [], "b--", linewidth=1.2, label="LIO-SAM aligned")
            self.ax_trajectory.set_title("Trajectory")
            self.ax_trajectory.set_xlabel("x [m]")
            self.ax_trajectory.set_ylabel("y [m]")
            self.ax_trajectory.set_aspect("equal", adjustable="datalim")
            self.ax_trajectory.grid(True, alpha=0.3)
            self.ax_trajectory.legend(fontsize=8)

            self.ax_position = self.figure.add_subplot(grid[0, 1])
            self.position_line, = self.ax_position.plot(
                [], [], "r-", label="2D position")
            self.ax_position.axhline(
                self.drift_warning_m, color="r", linestyle=":", alpha=0.7)
            self.ax_position.set_title("Position error [m]")
            self.ax_position.grid(True, alpha=0.3)

            self.ax_track = self.figure.add_subplot(grid[1, 1])
            self.along_line, = self.ax_track.plot(
                [], [], color="tab:orange", label="along-track")
            self.cross_line, = self.ax_track.plot(
                [], [], color="tab:green", label="cross-track")
            self.ax_track.axhline(0.0, color="k", linewidth=0.5)
            self.ax_track.set_title("Directional error [m]")
            self.ax_track.grid(True, alpha=0.3)
            self.ax_track.legend(fontsize=8)

            self.ax_yaw = self.figure.add_subplot(grid[0, 2])
            self.yaw_line, = self.ax_yaw.plot(
                [], [], color="tab:purple")
            self.ax_yaw.axhline(0.0, color="k", linewidth=0.5)
            self.ax_yaw.axhline(
                self.yaw_warning_deg, color="r", linestyle=":", alpha=0.7)
            self.ax_yaw.axhline(
                -self.yaw_warning_deg, color="r", linestyle=":", alpha=0.7)
            self.ax_yaw.set_title("Yaw error [deg]")
            self.ax_yaw.grid(True, alpha=0.3)

            self.ax_info = self.figure.add_subplot(grid[1, 2])
            self.ax_info.axis("off")
            self.info_text = self.ax_info.text(
                0.02, 0.98, "Waiting for topics...",
                transform=self.ax_info.transAxes, va="top",
                family="monospace", fontsize=10)
            self.figure.canvas.mpl_connect(
                "close_event",
                lambda _event: rospy.signal_shutdown("GUI closed"))
            plt.show(block=False)
        except Exception as exc:
            rospy.logerr(
                "[LIO experiment] GUI initialization failed: %s", exc)
            self.enable_gui = False
            self.figure = None

    @staticmethod
    def _set_line(line, x_values, y_values):
        line.set_data(list(x_values), list(y_values))

    def update_gui(self):
        if self.figure is None:
            return
        with self.lock:
            history = {
                key: list(values) for key, values in self.history.items()}
            metrics = dict(self.latest_metrics) if self.latest_metrics else None
            alignment_count = len(self.alignment_pairs)
            alignment_ready = self.alignment is not None
            skipped_sync = self.skipped_sync
            sample_count = self.sample_count
            last_odom_receipt = self.last_odom_receipt

        if history["time"]:
            time_values = history["time"]
            self._set_line(
                self.truth_line, history["truth_x"], history["truth_y"])
            self._set_line(
                self.lio_line, history["lio_x"], history["lio_y"])
            self._set_line(
                self.position_line, time_values, history["position"])
            self._set_line(
                self.along_line, time_values, history["along"])
            self._set_line(
                self.cross_line, time_values, history["cross"])
            self._set_line(
                self.yaw_line, time_values, history["yaw"])
            for axis in (
                    self.ax_trajectory, self.ax_position,
                    self.ax_track, self.ax_yaw):
                axis.relim()
                axis.autoscale_view()

        if metrics is None:
            odom_age = (
                math.inf if last_odom_receipt is None
                else max(0.0, rospy.Time.now().to_sec() - last_odom_receipt))
            info = (
                "ALIGNMENT: {}\n"
                "samples: {}/{}\n"
                "odom age: {}\n"
                "sync skips: {}\n\n"
                "CSV:\n{}"
            ).format(
                "READY" if alignment_ready else "WAITING",
                alignment_count, self.alignment_samples_needed,
                "inf" if math.isinf(odom_age) else "{:.2f}s".format(odom_age),
                skipped_sync, self.output_csv)
        else:
            gps_age_text = (
                "inf" if math.isinf(metrics["gps_age"])
                else "{:.2f}s".format(metrics["gps_age"]))
            info = (
                "STATUS: {status}\n"
                "samples: {samples}\n"
                "sync dt: {sync:.3f}s\n\n"
                "current position: {position:.3f}m\n"
                "current along:    {along:+.3f}m\n"
                "current cross:    {cross:+.3f}m\n"
                "current yaw:      {yaw:+.3f}deg\n\n"
                "position RMSE:    {position_rmse:.3f}m\n"
                "cross RMSE:       {cross_rmse:.3f}m\n"
                "yaw RMSE:         {yaw_rmse:.3f}deg\n"
                "max position:     {position_max:.3f}m\n\n"
                "GPS age: {gps_age}\n"
                "GPS shadow: {gps_shadow}\n"
                "LIO degenerate: {degenerate}\n"
                "sync skips: {skips}\n\n"
                "CSV:\n{csv_path}"
            ).format(
                status=metrics["status"], samples=sample_count,
                sync=metrics["sync_delta"],
                position=metrics["position_error"],
                along=metrics["along_error"],
                cross=metrics["cross_error"],
                yaw=metrics["yaw_error_deg"],
                position_rmse=metrics["position_rmse"],
                cross_rmse=metrics["cross_rmse"],
                yaw_rmse=metrics["yaw_rmse_deg"],
                position_max=self.position_max,
                gps_age=gps_age_text,
                gps_shadow=metrics["gps_shadow"],
                degenerate=metrics["degenerate"],
                skips=skipped_sync,
                csv_path=self.output_csv)
        self.info_text.set_text(info)
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def run(self):
        if not self.enable_gui:
            rospy.spin()
            return
        rate = rospy.Rate(self.gui_rate_hz)
        while not rospy.is_shutdown():
            self.update_gui()
            self.plt.pause(0.001)
            rate.sleep()

    def shutdown(self):
        with self.lock:
            if self.csv_stream and not self.csv_stream.closed:
                self.csv_stream.flush()
                self.csv_stream.close()
            result = {
                "purpose": "validation_only_not_used_for_control",
                "csv_file": self.output_csv,
                "ground_truth_topic": self.truth_topic,
                "odometry_topic": self.odom_topic,
                "incremental_topic": self.incremental_topic,
                "sample_count": self.sample_count,
                "skipped_sync_samples": self.skipped_sync,
                "alignment": self.alignment,
                "position_rmse_m": (
                    None if self.sample_count == 0 else
                    math.sqrt(self.position_sq_sum / self.sample_count)),
                "cross_track_rmse_m": (
                    None if self.sample_count == 0 else
                    math.sqrt(self.cross_sq_sum / self.sample_count)),
                "yaw_rmse_deg": (
                    None if self.sample_count == 0 else
                    math.degrees(math.sqrt(
                        self.yaw_sq_sum / self.sample_count))),
                "position_max_m": (
                    None if self.sample_count == 0 else self.position_max),
                "truth_distance_m": self.truth_distance,
                "lio_distance_m": self.lio_distance,
                "status_counts": self.status_counts,
            }
            temporary = self.summary_json + ".tmp"
            try:
                with open(temporary, "w", encoding="utf-8") as stream:
                    json.dump(result, stream, indent=2, sort_keys=True)
                    stream.write("\n")
                os.replace(temporary, self.summary_json)
                rospy.loginfo(
                    "[LIO experiment] summary=%s", self.summary_json)
            except OSError as exc:
                rospy.logerr(
                    "[LIO experiment] cannot write summary: %s", exc)


if __name__ == "__main__":
    try:
        experiment = LioOdometryExperiment()
        experiment.run()
    except rospy.ROSInterruptException:
        pass
