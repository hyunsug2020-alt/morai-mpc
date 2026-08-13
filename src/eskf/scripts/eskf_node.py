#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import sys
import threading
from collections import deque
from functools import wraps

import numpy as np
import rospy
import tf2_ros

try:
    from eskf import (
        FixedLagHistory,
        GpsMode,
        GpsModeMachine,
        ImuReplayEvent,
        RobustPlanarESKF,
        SE2Alignment,
        SpeedReplayEvent,
        latlon_to_utm,
        robust_gps_velocity,
        wrap_angle,
    )
except ImportError:
    # Keep direct source-tree execution compatible before catkin_make.
    sys.path.insert(0, os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..")))
    from eskf import (  # noqa: E402
        FixedLagHistory,
        GpsMode,
        GpsModeMachine,
        ImuReplayEvent,
        RobustPlanarESKF,
        SE2Alignment,
        SpeedReplayEvent,
        latlon_to_utm,
        robust_gps_velocity,
        wrap_angle,
    )
from geometry_msgs.msg import TransformStamped
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def synchronized(method):
    """Serialize callbacks that read or mutate the shared filter state."""
    @wraps(method)
    def wrapper(self, *args, **kwargs):
        with self.lock:
            return method(self, *args, **kwargs)
    return wrapper


class ESKFNode:
    """ROS wrapper for robust planar MORAI GPS/IMU localization."""

    def __init__(self):
        rospy.init_node("eskf_node")
        self.lock = threading.RLock()
        self.imu_topic = rospy.get_param("~imu_topic", "/imu")
        self.gps_topic = rospy.get_param("~gps_topic", "/gps")
        self.vehicle_state_topic = rospy.get_param(
            "~vehicle_state_topic", "/Ego_topic")
        self.odom_topic = rospy.get_param("~odom_topic", "/eskf/odom")
        self.slam_topic = rospy.get_param(
            "~slam_topic", "/lio_sam/mapping/odometry")
        self.slam_degeneracy_topic = rospy.get_param(
            "~slam_degeneracy_topic",
            "/lio_sam/mapping/odometry_incremental")
        self.diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "/eskf/diagnostics")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.utm_zone = int(rospy.get_param("~utm_zone", 52))
        self.publish_tf = bool(rospy.get_param("~publish_tf", False))
        self.sensor_config_file = rospy.get_param("~sensor_config_file", "")
        self.vehicle_model = rospy.get_param(
            "~vehicle_model", "2023_Hyundai_ioniq5")
        self.wheelbase = float(rospy.get_param("~wheelbase", 3.0))
        self.vehicle_length = float(
            rospy.get_param("~vehicle_length", 4.635))
        self.vehicle_width = float(rospy.get_param("~vehicle_width", 1.892))
        self.front_overhang = float(rospy.get_param("~front_overhang", 0.845))
        self.rear_overhang = float(rospy.get_param("~rear_overhang", 0.790))
        self.max_predict_dt = float(rospy.get_param("~max_predict_dt", 0.2))
        self.max_gps_lag = float(
            rospy.get_param("~max_gps_lag_sec", 0.5))
        self.gps_replay_enabled = bool(
            rospy.get_param("~gps_replay_enabled", True))
        self.gps_replay_max_age = float(
            rospy.get_param("~gps_replay_max_age_sec", 2.0))
        self.gps_replay_min_delay = float(
            rospy.get_param("~gps_replay_min_delay_sec", 0.02))
        self.gps_rejection_limit = int(
            rospy.get_param("~gps_rejection_limit", 5))
        self.gps_recovery_accepts = int(
            rospy.get_param("~gps_recovery_accepts", 3))
        self.speed_deadband = float(
            rospy.get_param("~speed_deadband_mps", 0.15))
        self.wheel_aiding_enabled = bool(rospy.get_param(
            "~wheel_aiding_enabled", True))
        self.wheel_speed_scale = float(rospy.get_param(
            "~wheel_speed_scale", 1.0))
        self.wheel_speed_variance = float(rospy.get_param(
            "~wheel_speed_variance", 0.04))
        self.wheel_max_message_lag = float(rospy.get_param(
            "~wheel_max_message_lag_sec", 0.25))
        self.odometry_aiding_enabled = bool(rospy.get_param(
            "~odometry_aiding_enabled", True))
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/odometry/pure")
        self.odometry_diagnostics_topic = rospy.get_param(
            "~odometry_diagnostics_topic",
            "/pure_odometry/diagnostics")
        self.use_sim_time = bool(rospy.get_param("/use_sim_time", False))
        self.odometry_speed_aiding_enabled = bool(rospy.get_param(
            "~odometry_speed_aiding_enabled", True))
        self.odometry_speed_variance = float(rospy.get_param(
            "~odometry_speed_variance", 0.09))
        self.odometry_pose_aiding_enabled = bool(rospy.get_param(
            "~odometry_pose_aiding_enabled", True))
        self.odometry_position_variance = float(rospy.get_param(
            "~odometry_position_variance", 0.25))
        self.odometry_position_drift_per_meter = float(rospy.get_param(
            "~odometry_position_drift_per_meter", 0.03))
        self.odometry_correlation_inflation = max(
            1.0, float(rospy.get_param(
                "~odometry_correlation_inflation", 9.0)))
        self.odometry_activation_delay = float(rospy.get_param(
            "~odometry_activation_delay_sec", 1.0))
        self.odometry_max_message_lag = float(rospy.get_param(
            "~odometry_max_message_lag_sec", 0.25))
        self.odometry_max_alignment_age = float(rospy.get_param(
            "~odometry_max_alignment_age_sec", 180.0))
        self.odometry_alignment_gps_max_age = float(rospy.get_param(
            "~odometry_alignment_gps_max_age_sec", 0.75))
        self.odometry_update_rate_hz = float(rospy.get_param(
            "~odometry_update_rate_hz", 10.0))
        self.odometry_update_rate_hz = max(
            self.odometry_update_rate_hz, 1.0e-3)
        self.odometry_slam_fallback_age = float(rospy.get_param(
            "~odometry_slam_fallback_age_sec", 0.5))
        self.invalid_gps_status = int(
            rospy.get_param("~invalid_gps_status", -1))
        self.gps_velocity_window = float(
            rospy.get_param("~gps_velocity_window_sec", 0.6))
        self.initialization_window = int(
            rospy.get_param("~initialization_window_samples", 7))
        self.initialization_min_inliers = int(
            rospy.get_param("~initialization_min_inliers", 4))
        self.initialization_radius = float(
            rospy.get_param("~initialization_radius_m", 6.0))
        self.gps_reacquisition_gap = float(
            rospy.get_param("~gps_reacquisition_gap_sec", 1.5))
        self.gps_reacquisition_samples = int(
            rospy.get_param("~gps_reacquisition_samples", 7))
        self.gps_reacquisition_radius = float(
            rospy.get_param("~gps_reacquisition_radius_m", 6.0))
        self.nhc_rate_hz = float(rospy.get_param("~nhc_rate_hz", 10.0))
        self.nhc_min_speed = float(
            rospy.get_param("~nhc_min_speed_mps", 0.5))
        self.zupt_speed = float(
            rospy.get_param("~zupt_speed_threshold_mps", 0.25))
        self.zupt_accel = float(
            rospy.get_param("~zupt_accel_threshold_mps2", 0.3))
        self.zupt_gyro = float(
            rospy.get_param("~zupt_gyro_threshold_rps", 0.03))
        self.zupt_hold = float(rospy.get_param("~zupt_hold_sec", 0.8))
        self.max_abs_accel = float(
            rospy.get_param("~max_abs_accel_mps2", 20.0))
        self.max_abs_gyro = float(
            rospy.get_param("~max_abs_gyro_rps", 2.0))
        self.accel_spike_floor = float(
            rospy.get_param("~accel_spike_floor_mps2", 4.0))
        self.gyro_spike_floor = float(
            rospy.get_param("~gyro_spike_floor_rps", 0.35))
        self.spike_sigma = float(rospy.get_param("~spike_sigma", 6.0))
        self.slam_aiding_enabled = bool(
            rospy.get_param("~slam_aiding_enabled", True))
        self.slam_activation_delay = float(
            rospy.get_param("~slam_activation_delay_sec", 1.0))
        self.slam_max_alignment_age = float(
            rospy.get_param("~slam_max_alignment_age_sec", 120.0))
        self.slam_max_message_lag = float(
            rospy.get_param("~slam_max_message_lag_sec", 0.5))
        self.slam_update_rate_hz = float(
            rospy.get_param("~slam_update_rate_hz", 10.0))
        self.slam_alignment_alpha = float(
            rospy.get_param("~slam_alignment_alpha", 0.05))
        self.slam_alignment_gps_max_age = float(
            rospy.get_param("~slam_alignment_gps_max_age_sec", 0.75))
        self.slam_alignment_reset_distance = float(
            rospy.get_param("~slam_alignment_reset_distance_m", 5.0))
        self.slam_alignment_reset_yaw = math.radians(float(
            rospy.get_param("~slam_alignment_reset_yaw_deg", 20.0)))
        self.slam_alignment_reset_confirm_samples = int(rospy.get_param(
            "~slam_alignment_reset_confirm_samples", 3))
        self.slam_alignment_min_baseline = float(
            rospy.get_param("~slam_alignment_min_baseline_m", 12.0))
        self.slam_continuity_confirm_samples = int(
            rospy.get_param("~slam_continuity_confirm_samples", 10))
        self.slam_continuity_fast_recovery_gap = float(
            rospy.get_param(
                "~slam_continuity_fast_recovery_gap_sec", 1.0))
        self.slam_position_drift_per_meter = float(
            rospy.get_param("~slam_position_drift_per_meter", 0.02))
        self.slam_yaw_drift_per_meter = float(
            rospy.get_param("~slam_yaw_drift_per_meter_rad", 0.001))
        self.slam_correlation_inflation = float(
            rospy.get_param("~slam_correlation_inflation", 4.0))
        self.slam_degenerate_aiding_enabled = bool(
            rospy.get_param("~slam_degenerate_aiding_enabled", True))
        self.slam_degenerate_variance_scale = max(
            1.0, float(rospy.get_param(
                "~slam_degenerate_variance_scale", 36.0)))
        self.slam_motion_window = float(
            rospy.get_param("~slam_motion_window_sec", 1.2))
        self.slam_stationary_speed = float(
            rospy.get_param(
                "~slam_stationary_speed_threshold_mps", 0.3))
        self.slam_stationary_filter_speed_max = float(
            rospy.get_param(
                "~slam_stationary_filter_speed_max_mps", 1.5))
        self.slam_stationary_release_accel = float(
            rospy.get_param(
                "~slam_stationary_release_accel_mps2", 0.45))
        self.slam_stationary_reactivation_delay = float(
            rospy.get_param(
                "~slam_stationary_reactivation_delay_sec", 2.0))
        self.slam_stationary_hint_max_age = float(
            rospy.get_param(
                "~slam_stationary_hint_max_age_sec", 6.0))
        self.slam_stationary_max_residual_variance = float(
            rospy.get_param(
                "~slam_stationary_max_residual_variance", 0.25))
        self.slam_local_jump_distance = float(
            rospy.get_param("~slam_local_jump_distance_min_m", 3.0))
        self.slam_local_jump_yaw = math.radians(float(
            rospy.get_param("~slam_local_jump_yaw_min_deg", 20.0)))
        self.slam_local_jump_speed_margin = float(
            rospy.get_param("~slam_local_jump_speed_margin_mps", 5.0))

        self.gps_lever = np.array([
            float(rospy.get_param("~gps_x", 3.232)),
            float(rospy.get_param("~gps_y", 0.037)),
        ])
        self.imu_lever = np.array([
            float(rospy.get_param("~imu_x", 3.423)),
            float(rospy.get_param("~imu_y", 0.012)),
        ])
        self.lidar_lever = np.array([
            float(rospy.get_param("~lidar_x", 1.045)),
            float(rospy.get_param("~lidar_y", 0.000)),
        ])
        self._load_sensor_extrinsics()
        self._validate_vehicle_geometry()
        filter_config = {
            name: rospy.get_param("~" + name)
            for name in (
                "accel_noise_std",
                "gyro_noise_std",
                "accel_bias_random_walk_std",
                "gyro_bias_random_walk_std",
                "gps_position_variance",
                "gps_velocity_variance_floor",
                "wheel_speed_variance",
                "imu_yaw_variance",
                "nhc_variance",
                "zupt_velocity_variance",
                "slam_position_variance",
                "slam_yaw_variance",
                "gps_nis_soft",
                "gps_nis_hard",
                "velocity_nis_soft",
                "velocity_nis_hard",
                "wheel_speed_nis_soft",
                "wheel_speed_nis_hard",
                "yaw_nis_soft",
                "yaw_nis_hard",
                "constraint_nis_hard",
                "slam_nis_soft",
                "slam_nis_hard",
                "odometry_nis_soft",
                "odometry_nis_hard",
                "adaptive_alpha",
                "adaptive_scale_min",
                "adaptive_scale_max",
                "max_accel_bias",
                "max_gyro_bias",
            )
            if rospy.has_param("~" + name)
        }
        self.filter = RobustPlanarESKF(
            filter_config,
            self.gps_lever - self.imu_lever,
            self.imu_lever)
        self.gps_mode = GpsModeMachine(
            outage_timeout=self.gps_reacquisition_gap,
            rejection_limit=self.gps_rejection_limit,
            recovery_accepts=self.gps_recovery_accepts)
        self.gps_replay = FixedLagHistory(
            max_age=self.gps_replay_max_age,
            min_delay=self.gps_replay_min_delay)

        self.last_predict_stamp = None
        self.last_imu_yaw = None
        self.last_imu_input = np.zeros(3, dtype=float)
        self.last_nhc_stamp = None
        self.last_gps_message_time = None
        self.last_gps_accept_time = None
        self.last_wheel_message_time = None
        self.last_wheel_accept_time = None
        self.last_wheel_source_stamp = None
        self.wheel_duplicate_stamps = 0
        self.last_wheel_speed = 0.0
        self.last_wheel_angle_deg = 0.0
        self.last_odometry_message_time = None
        self.last_odometry_speed_accept_time = None
        self.last_odometry_pose_accept_time = None
        self.last_odometry_update_time = None
        self.last_odometry_source_stamp = None
        self.odometry_duplicate_stamps = 0
        self.zupt_blocked_by_odometry = 0
        self.last_odometry_speed = 0.0
        self.odometry_recovery_speed_history = deque(maxlen=5)
        self.last_odometry_local_position = None
        self.last_odometry_local_stamp = None
        self.odometry_local_jumps = 0
        self.odometry_total_distance = 0.0
        self.odometry_anchor_distance = 0.0
        self.last_odometry_incremental_variance = 0.0
        self.odometry_time_scale = 1.0
        self.odometry_time_scale_updates = 0
        self.odometry_shadow_anchor_local_position = None
        self.odometry_shadow_anchor_global_position = None
        self.odometry_shadow_anchor_yaw = None
        self.odometry_shadow_anchor_samples = 0
        self.last_slam_accept_time = None
        self.last_slam_message_time = None
        self.last_slam_update_time = None
        self.slam_degenerate = False
        self.slam_degeneracy_time = None
        self.last_local_slam_position = None
        self.last_local_slam_yaw = None
        self.last_local_slam_stamp = None
        self.trusted_local_slam_position = None
        self.trusted_local_slam_yaw = None
        self.trusted_local_slam_stamp = None
        self.trusted_filter_position = None
        self.trusted_filter_yaw = None
        self.slam_motion_history = deque(maxlen=30)
        self.slam_stationary_hint_time = None
        self.slam_stationary_accel_reference = None
        self.slam_stationary_filtered_accel_delta = 0.0
        self.slam_stationary_release_time = None
        self.slam_alignment = SE2Alignment(
            self.slam_alignment_alpha,
            self.slam_alignment_reset_distance,
            self.slam_alignment_reset_yaw,
            self.slam_alignment_reset_confirm_samples,
            min_baseline=self.slam_alignment_min_baseline,
            continuity_confirm_samples=(
                self.slam_continuity_confirm_samples))
        self.odometry_alignment = SE2Alignment(
            alpha=0.05,
            reset_distance=5.0,
            reset_yaw=math.radians(20.0),
            reset_confirm_samples=3,
            window_size=600,
            min_samples=10,
            min_baseline=5.0,
            continuity_confirm_samples=5)
        self.gps_history = deque(maxlen=40)
        self.initial_gps_buffer = deque(maxlen=self.initialization_window)
        self.gps_reacquisition_buffer = deque(
            maxlen=max(self.gps_reacquisition_samples * 2, 10))
        self.gps_reacquisition_active = False
        self.imu_windows = [deque(maxlen=11) for _ in range(3)]
        self.zupt_candidate_since = None

        self.odom_pub = rospy.Publisher(
            self.odom_topic, Odometry, queue_size=20)
        self.diagnostics_pub = rospy.Publisher(
            self.diagnostics_topic, String, queue_size=2, latch=True)
        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster() if self.publish_tf else None)
        self.imu_sub = rospy.Subscriber(
            self.imu_topic, Imu, self.imu_callback, queue_size=100)
        self.gps_sub = rospy.Subscriber(
            self.gps_topic, GPSMessage, self.gps_callback, queue_size=20)
        self.vehicle_state_sub = rospy.Subscriber(
            self.vehicle_state_topic, EgoVehicleStatus,
            self.vehicle_state_callback, queue_size=50)
        self.slam_sub = rospy.Subscriber(
            self.slam_topic, Odometry, self.slam_callback, queue_size=20)
        self.slam_degeneracy_sub = rospy.Subscriber(
            self.slam_degeneracy_topic,
            Odometry,
            self.slam_degeneracy_callback,
            queue_size=20)
        self.odometry_sub = rospy.Subscriber(
            self.odometry_topic,
            Odometry,
            self.odometry_callback,
            queue_size=50,
            tcp_nodelay=True)
        self.odometry_diagnostics_sub = rospy.Subscriber(
            self.odometry_diagnostics_topic,
            String,
            self.odometry_diagnostics_callback,
            queue_size=5)
        self.diagnostics_timer = rospy.Timer(
            rospy.Duration(1.0), self.publish_diagnostics)

        rospy.loginfo(
            "[ESKF] robust 8-state GPS+IMU filter: %s + %s -> %s",
            self.gps_topic, self.imu_topic, self.odom_topic)
        rospy.loginfo(
            "[ESKF] conditional SLAM aiding: %s (%s)",
            "enabled" if self.slam_aiding_enabled else "disabled",
            self.slam_topic)
        rospy.loginfo(
            "[ESKF] delayed GPS replay: %s (window %.2fs)",
            "enabled" if self.gps_replay_enabled else "disabled",
            self.gps_replay_max_age)
        rospy.loginfo(
            "[ESKF] wheel-speed aiding: %s (%s, pose/heading unused)",
            "enabled" if self.wheel_aiding_enabled else "disabled",
            self.vehicle_state_topic)
        rospy.loginfo(
            "[ESKF] validated odometry aiding: %s (%s, speed=%s pose=%s)",
            "enabled" if self.odometry_aiding_enabled else "disabled",
            self.odometry_topic,
            "on" if self.odometry_speed_aiding_enabled else "off",
            "GPS-shadow only" if self.odometry_pose_aiding_enabled else "off")
        rospy.loginfo(
            "[ESKF] lever arms GPS=(%.3f, %.3f), IMU=(%.3f, %.3f), "
            "LiDAR=(%.3f, %.3f), %s",
            self.gps_lever[0], self.gps_lever[1],
            self.imu_lever[0], self.imu_lever[1],
            self.lidar_lever[0], self.lidar_lever[1],
            self.sensor_config_file or "launch defaults")

    @staticmethod
    def _sensor_xy(sensor_list):
        if not sensor_list:
            raise ValueError("sensor list is empty")
        position = sensor_list[0]["pos"]
        return np.array([float(position["x"]), float(position["y"])])

    def _load_sensor_extrinsics(self):
        if not self.sensor_config_file:
            return
        path = os.path.expanduser(self.sensor_config_file)
        try:
            with open(path, "r", encoding="utf-8-sig") as stream:
                config = json.load(stream)
            self.gps_lever = self._sensor_xy(config.get("GPSList", []))
            self.imu_lever = self._sensor_xy(config.get("IMUList", []))
            self.lidar_lever = self._sensor_xy(
                config.get("Lidar3DList", []))
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn(
                "[ESKF] cannot load sensor config %s: %s; using defaults",
                path, exc)

    def _validate_vehicle_geometry(self):
        expected_length = (
            self.front_overhang + self.wheelbase + self.rear_overhang)
        if abs(expected_length - self.vehicle_length) > 0.05:
            rospy.logwarn(
                "[ESKF] vehicle geometry mismatch %.3fm vs %.3fm",
                expected_length, self.vehicle_length)
        lateral_limit = self.vehicle_width * 0.5 + 0.4
        for name, lever in (
                ("GPS", self.gps_lever),
                ("IMU", self.imu_lever),
                ("LiDAR", self.lidar_lever)):
            if abs(lever[1]) > lateral_limit:
                rospy.logwarn(
                    "[ESKF] %s lateral lever %.3fm exceeds vehicle envelope",
                    name, lever[1])

    @staticmethod
    def _stamp_or_now(header):
        return header.stamp if header.stamp != rospy.Time() else rospy.Time.now()

    @staticmethod
    def _valid_quaternion(msg):
        norm_sq = (
            msg.orientation.x**2 + msg.orientation.y**2
            + msg.orientation.z**2 + msg.orientation.w**2)
        return math.isfinite(norm_sq) and norm_sq > 1e-8

    def _reject_spike(self, value, index):
        window = self.imu_windows[index]
        if len(window) >= 5:
            values = np.asarray(window, dtype=float)
            median = float(np.median(values))
            mad = float(np.median(np.abs(values - median)))
            floor = (
                self.accel_spike_floor if index < 2 else self.gyro_spike_floor)
            threshold = max(floor, self.spike_sigma * 1.4826 * mad)
            if abs(value - median) > threshold:
                value = median + math.copysign(threshold, value - median)
                self.filter.counters["imu_spikes"] += 1
        hard_limit = self.max_abs_accel if index < 2 else self.max_abs_gyro
        clipped = float(np.clip(value, -hard_limit, hard_limit))
        if clipped != value:
            self.filter.counters["imu_spikes"] += 1
        window.append(clipped)
        return clipped

    def _physics_duration(self, source_duration):
        scale = 1.0 if self.use_sim_time else self.odometry_time_scale
        return float(source_duration) * scale

    def _trusted_odometry_base_velocity(self, stamp_sec):
        """Return a robust map-frame velocity from recent pure odometry.

        This path is used only for recovery from a confirmed GPS outage or
        repeated GPS rejection.  Requiring several continuous odometry
        samples prevents one malformed speed value from resetting the ESKF.
        """
        recent = [
            (sample_stamp, speed)
            for sample_stamp, speed in self.odometry_recovery_speed_history
            if -0.1 <= stamp_sec - sample_stamp <= 0.75
        ]
        if len(recent) < 3:
            return None
        speeds = np.asarray([sample[1] for sample in recent], dtype=float)
        median_speed = float(np.median(speeds))
        speed_tolerance = max(0.5, 0.15 * abs(median_speed) + 0.2)
        if (
                not math.isfinite(median_speed)
                or abs(median_speed) > 100.0
                or float(np.max(np.abs(speeds - median_speed)))
                > speed_tolerance):
            return None
        yaw = float(self.filter.x[self.filter.YAW])
        return np.array([
            math.cos(yaw) * median_speed,
            math.sin(yaw) * median_speed,
        ])

    def _gps_sensor_velocity_from_base(self, base_velocity):
        """Convert base-link map velocity to the GPS antenna velocity."""
        yaw = float(self.filter.x[self.filter.YAW])
        base_to_gps_world = (
            self.filter._rotation(yaw)
            @ (self.filter.imu_lever + self.filter.gps_from_imu))
        omega = self.filter.last_corrected_yaw_rate
        return np.asarray(base_velocity, dtype=float) + np.array([
            -omega * base_to_gps_world[1],
            omega * base_to_gps_world[0],
        ])

    def _clear_odometry_shadow_anchor(self):
        """Discard a shadow anchor tied to a pre-reacquisition map frame."""
        self.odometry_shadow_anchor_samples = 0
        self.odometry_shadow_anchor_local_position = None
        self.odometry_shadow_anchor_global_position = None
        self.odometry_shadow_anchor_yaw = None
        self.odometry_anchor_distance = self.odometry_total_distance
        self.last_odometry_incremental_variance = 0.0

    @synchronized
    def odometry_diagnostics_callback(self, msg):
        if not self.odometry_aiding_enabled:
            return
        try:
            diagnostics = json.loads(msg.data)
            reported_scale = float(diagnostics["motion_rate_scale"])
            reported_sim_time = bool(diagnostics.get(
                "uses_sim_time", False))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.filter.counters["invalid_measurements"] += 1
            return
        if not math.isfinite(reported_scale):
            self.filter.counters["invalid_measurements"] += 1
            return
        if self.use_sim_time or reported_sim_time:
            reported_scale = 1.0
        if not 0.05 <= reported_scale <= 2.0:
            self.filter.counters["invalid_measurements"] += 1
            return
        self.odometry_time_scale = reported_scale
        self.odometry_time_scale_updates += 1

    @synchronized
    def gps_callback(self, msg):
        values = (
            msg.latitude, msg.longitude, msg.eastOffset, msg.northOffset)
        if not all(math.isfinite(value) for value in values):
            self.filter.counters["invalid_measurements"] += 1
            rospy.logwarn_throttle(2.0, "[ESKF] non-finite GPS measurement")
            return
        if msg.status == self.invalid_gps_status:
            self.filter.counters["gps_rejected"] += 1
            self.gps_mode.rejected()
            rospy.logwarn_throttle(
                2.0, "[ESKF] GPS status reports no valid fix")
            return

        utm_x, utm_y = latlon_to_utm(
            msg.latitude, msg.longitude, self.utm_zone)
        gps_position = np.array([
            utm_x - msg.eastOffset,
            utm_y - msg.northOffset,
        ])
        stamp = self._stamp_or_now(msg.header)
        stamp_sec = stamp.to_sec()
        self.gps_mode.received(stamp_sec)
        previous_gps_message_time = self.last_gps_message_time
        self.last_gps_message_time = (
            stamp_sec if previous_gps_message_time is None
            else max(stamp_sec, previous_gps_message_time))
        gps_message_gap = (
            math.inf if previous_gps_message_time is None
            else stamp_sec - previous_gps_message_time)
        gps_accept_gap = (
            math.inf if self.last_gps_accept_time is None
            else stamp_sec - self.last_gps_accept_time)
        if (
                self.filter.initialized
                and not self.gps_reacquisition_active
                and (
                    gps_message_gap >= self.gps_reacquisition_gap
                    or gps_accept_gap >= self.gps_reacquisition_gap)):
            self.gps_reacquisition_active = True
            self.gps_reacquisition_buffer.clear()

        if not self.filter.initialized:
            self.initial_gps_buffer.append(gps_position.copy())
            samples = np.asarray(self.initial_gps_buffer)
            center = np.median(samples, axis=0)
            residuals = np.linalg.norm(samples - center, axis=1)
            inliers = samples[residuals <= self.initialization_radius]
            if len(inliers) < self.initialization_min_inliers:
                rospy.loginfo_throttle(
                    1.0,
                    "[ESKF] waiting for GPS consensus: %d/%d inliers",
                    len(inliers), self.initialization_min_inliers)
                return
            yaw = self.last_imu_yaw if self.last_imu_yaw is not None else 0.0
            initial_position = np.mean(inliers, axis=0)
            self.filter.initialize(initial_position, yaw)
            self.last_predict_stamp = stamp
            self.last_gps_accept_time = stamp_sec
            self.gps_mode.initialized(stamp_sec)
            self.gps_replay.seed(stamp_sec, self.filter.snapshot())
            self.gps_history.append((stamp_sec, initial_position.copy()))
            base_position, _ = self.filter.base_state()
            rospy.loginfo(
                "[ESKF] initialized from %d GPS inliers: "
                "base_link=(%.3f, %.3f), yaw=%.2f deg",
                len(inliers),
                base_position[0], base_position[1],
                math.degrees(self.filter.x[self.filter.YAW]))
            self.publish(stamp)
            return

        if self.gps_reacquisition_active:
            self.gps_reacquisition_buffer.append(
                (stamp_sec, gps_position.copy()))
            if (
                    len(self.gps_reacquisition_buffer)
                    >= self.gps_reacquisition_samples):
                trusted_base_velocity = (
                    self._trusted_odometry_base_velocity(stamp_sec))
                if trusted_base_velocity is not None:
                    projection_velocity = (
                        self._gps_sensor_velocity_from_base(
                            trusted_base_velocity))
                else:
                    _, fallback_base_velocity = self.filter.base_state()
                    projection_velocity = (
                        self._gps_sensor_velocity_from_base(
                            fallback_base_velocity))
                projected_positions = np.asarray([
                    position + projection_velocity * self._physics_duration(
                        stamp_sec - sample_stamp)
                    for sample_stamp, position
                    in self.gps_reacquisition_buffer
                ])
                center = np.median(projected_positions, axis=0)
                residuals = np.linalg.norm(
                    projected_positions - center, axis=1)
                inliers = projected_positions[
                    residuals <= self.gps_reacquisition_radius]
                required_inliers = max(
                    4, self.gps_reacquisition_samples // 2 + 1)
                if len(inliers) >= required_inliers:
                    consensus_position = np.mean(inliers, axis=0)
                    self.filter.reanchor_gps_position(
                        consensus_position,
                        base_velocity=trusted_base_velocity,
                        velocity_variance=max(
                            self.odometry_speed_variance, 0.25))
                    self._clear_odometry_shadow_anchor()
                    self.gps_history.clear()
                    self.gps_history.append(
                        (stamp_sec, consensus_position.copy()))
                    self.last_gps_accept_time = stamp_sec
                    self.gps_mode.reanchored(stamp_sec)
                    barrier_stamp = (
                        self.last_predict_stamp.to_sec()
                        if self.last_predict_stamp is not None
                        else stamp_sec)
                    self.gps_replay.mark_barrier(
                        barrier_stamp, self.filter.snapshot())
                    self.gps_reacquisition_active = False
                    self.gps_reacquisition_buffer.clear()
                    rospy.logwarn(
                        "[ESKF] GPS reacquired from %d/%d consensus fixes "
                        "(velocity=%s)",
                        len(inliers), len(projected_positions),
                        "pure_odometry"
                        if trusted_base_velocity is not None else "preserved")
                    self.publish(stamp)
                    return

        gps_lag = 0.0
        if self.last_predict_stamp is not None:
            gps_lag = (self.last_predict_stamp - stamp).to_sec()
            if (
                    gps_lag > self.max_gps_lag
                    and (
                        not self.gps_replay_enabled
                        or gps_lag > self.gps_replay_max_age)):
                self.filter.counters["stale_gps"] += 1
                self.gps_mode.rejected(stamp_sec)
                rospy.logwarn_throttle(
                    1.0,
                    "[ESKF] rejected stale GPS: lag=%.3fs",
                    gps_lag)
                return
        velocity_samples = [
            (history_stamp, history_position)
            for history_stamp, history_position in self.gps_history
            if 0.0 <= stamp_sec - history_stamp <= 2.0
        ]
        velocity_samples.append((stamp_sec, gps_position.copy()))
        velocity_estimate = robust_gps_velocity(
            velocity_samples,
            min_span=self.gps_velocity_window)

        def apply_gps(position_variance):
            accepted_position, position_nis = (
                self.filter.update_gps_position(
                    gps_position, position_variance))
            if not accepted_position:
                return False, position_nis
            if velocity_estimate is not None:
                gps_velocity, denominator, residual_variance = (
                    velocity_estimate)
                time_scale = (
                    1.0 if self.use_sim_time else self.odometry_time_scale)
                gps_velocity = gps_velocity.copy() / time_scale
                yaw = self.filter.x[self.filter.YAW]
                lever_world = (
                    self.filter._rotation(yaw)
                    @ self.filter.gps_from_imu)
                omega = self.filter.last_corrected_yaw_rate
                gps_velocity += np.array([
                    omega * lever_world[1],
                    -omega * lever_world[0],
                ])
                effective_position_variance = max(
                    self.filter.gps_position_variance
                    * self.filter.measurement_scales["gps_position"],
                    residual_variance)
                velocity_variance = (
                    effective_position_variance / denominator
                    / (time_scale**2)
                    + self.filter.gps_velocity_variance_floor)
                accepted_velocity, velocity_nis = (
                    self.filter.update_gps_velocity(
                        gps_velocity, velocity_variance))
                if not accepted_velocity:
                    rospy.logwarn_throttle(
                        1.0,
                        "[ESKF] rejected GPS velocity: NIS=%.2f",
                        velocity_nis)
            return True, position_nis

        replay_result = None
        if (
                self.gps_replay_enabled
                and self.last_predict_stamp is not None
                and gps_lag >= self.gps_replay_min_delay):
            replay_result = self.gps_replay.replay(
                stamp_sec,
                self.last_predict_stamp.to_sec(),
                self.filter.snapshot,
                self.filter.restore,
                lambda: apply_gps(
                    self.filter.gps_position_variance),
                self._apply_replay_event)

        if replay_result is not None and replay_result.attempted:
            accepted = replay_result.accepted
            nis = (
                replay_result.update_result[1]
                if isinstance(replay_result.update_result, tuple)
                else math.inf)
            if not accepted:
                # A rejected rewind restores all old diagnostics too, so count
                # this newly rejected receiver fix once after restoration.
                self.filter.counters["gps_rejected"] += 1
        else:
            if gps_lag > self.max_gps_lag:
                self.filter.counters["stale_gps"] += 1
                self.gps_mode.rejected(stamp_sec)
                reason = (
                    replay_result.reason
                    if replay_result is not None else "replay_disabled")
                rospy.logwarn_throttle(
                    1.0,
                    "[ESKF] rejected stale GPS: lag=%.3fs (%s)",
                    gps_lag, reason)
                return
            _, base_velocity = self.filter.base_state()
            delay_variance = (
                np.linalg.norm(base_velocity)
                * self._physics_duration(max(gps_lag, 0.0)))**2
            accepted, nis = apply_gps(
                self.filter.gps_position_variance + delay_variance)
            barrier_stamp = (
                self.last_predict_stamp.to_sec()
                if self.last_predict_stamp is not None else stamp_sec)
            if accepted:
                self.gps_replay.mark_barrier(
                    barrier_stamp, self.filter.snapshot())
        if not accepted:
            self.gps_mode.rejected(stamp_sec)
            rospy.logwarn_throttle(
                1.0, "[ESKF] rejected GPS outlier: NIS=%.2f", nis)
            return
        self.gps_history.append((stamp_sec, gps_position.copy()))
        while (
                self.gps_history
                and stamp_sec - self.gps_history[0][0] > 2.0):
            self.gps_history.popleft()
        self.last_gps_accept_time = (
            stamp_sec if self.last_gps_accept_time is None
            else max(stamp_sec, self.last_gps_accept_time))
        self.gps_mode.accepted(stamp_sec)
        self.gps_reacquisition_active = False
        self.gps_reacquisition_buffer.clear()
        publish_stamp = (
            self.last_predict_stamp
            if self.last_predict_stamp is not None
            and self.last_predict_stamp > stamp
            else stamp)
        self.publish(publish_stamp)

    @synchronized
    def vehicle_state_callback(self, msg):
        """Use only wheel-equivalent channels; never read pose or heading."""
        if not self.wheel_aiding_enabled:
            return
        # Deliberate whitelist: position, heading, and acceleration from the
        # simulator ground-truth message are not accessed here.
        velocity_x = float(msg.velocity.x)
        velocity_y = float(msg.velocity.y)
        wheel_angle_deg = float(msg.wheel_angle)
        if not all(math.isfinite(value) for value in (
                velocity_x, velocity_y, wheel_angle_deg)):
            self.filter.counters["invalid_measurements"] += 1
            return
        stamp = self._stamp_or_now(msg.header)
        stamp_sec = stamp.to_sec()
        self.last_wheel_message_time = stamp_sec
        if (
                self.last_wheel_source_stamp is not None
                and stamp_sec <= self.last_wheel_source_stamp):
            self.wheel_duplicate_stamps += 1
            return
        self.last_wheel_source_stamp = stamp_sec
        if not self.filter.initialized:
            return
        if self.last_predict_stamp is not None:
            lag = abs((self.last_predict_stamp - stamp).to_sec())
            if lag > self.wheel_max_message_lag:
                self.filter.counters["wheel_speed_rejected"] += 1
                return

        speed = self.wheel_speed_scale * math.hypot(
            velocity_x, velocity_y)
        # Large steering increases the risk of sideslip.  Wheel angle changes
        # covariance only and is never integrated as an absolute heading.
        steering_scale = 1.0 + (abs(wheel_angle_deg) / 20.0)**2
        variance = self.wheel_speed_variance * steering_scale
        accepted, _ = self.filter.update_wheel_speed(speed, variance)
        self.last_wheel_speed = speed
        self.last_wheel_angle_deg = wheel_angle_deg
        if accepted:
            self.last_wheel_accept_time = stamp_sec
            filter_stamp = (
                self.last_predict_stamp.to_sec()
                if self.last_predict_stamp is not None else stamp_sec)
            self.gps_replay.record(
                filter_stamp,
                SpeedReplayEvent(speed, variance),
                self.filter.snapshot())
            self.publish(stamp)

    @synchronized
    def odometry_callback(self, msg):
        """Fuse validated wheel/IMU odometry without using Ego pose/heading."""
        if not self.odometry_aiding_enabled or not self.filter.initialized:
            return
        values = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
        )
        if not all(math.isfinite(value) for value in values):
            self.filter.counters["invalid_measurements"] += 1
            return
        quaternion = values[2:6]
        quaternion_norm = math.sqrt(sum(value**2 for value in quaternion))
        if quaternion_norm < 1.0e-6:
            self.filter.counters["invalid_measurements"] += 1
            return
        _, _, local_yaw = euler_from_quaternion(quaternion)
        local_yaw = wrap_angle(local_yaw)
        local_position = np.asarray(values[0:2], dtype=float)
        forward_speed = float(values[6])
        stamp = self._stamp_or_now(msg.header)
        stamp_sec = stamp.to_sec()
        self.last_odometry_message_time = stamp_sec
        if (
                self.last_odometry_source_stamp is not None
                and stamp_sec <= self.last_odometry_source_stamp):
            self.odometry_duplicate_stamps += 1
            return
        self.last_odometry_source_stamp = stamp_sec
        local_pose_continuous = True
        if (
                self.last_odometry_local_position is not None
                and self.last_odometry_local_stamp is not None):
            local_step = float(np.linalg.norm(
                local_position - self.last_odometry_local_position))
            local_dt = stamp_sec - self.last_odometry_local_stamp
            physics_local_dt = self._physics_duration(local_dt)
            maximum_step = max(
                3.0,
                (abs(forward_speed) + 5.0)
                * max(physics_local_dt, 0.0))
            if local_dt <= 0.0 or local_step > maximum_step:
                local_pose_continuous = False
                self.odometry_local_jumps += 1
                self.odometry_shadow_anchor_samples = 0
                self.odometry_shadow_anchor_local_position = None
                self.odometry_shadow_anchor_global_position = None
                self.odometry_shadow_anchor_yaw = None
            else:
                self.odometry_total_distance += local_step
        self.last_odometry_local_position = local_position.copy()
        self.last_odometry_local_stamp = stamp_sec

        lag = 0.0
        filter_stamp = stamp_sec
        if self.last_predict_stamp is not None:
            filter_stamp = self.last_predict_stamp.to_sec()
            lag = filter_stamp - stamp_sec
            if abs(lag) > self.odometry_max_message_lag:
                self.filter.counters["odometry_rejected"] += 1
                return

        if local_pose_continuous:
            self.odometry_recovery_speed_history.append(
                (stamp_sec, forward_speed))
        else:
            self.odometry_recovery_speed_history.clear()

        if self.odometry_speed_aiding_enabled:
            reported_variance = float(msg.twist.covariance[0])
            speed_variance = max(
                self.odometry_speed_variance,
                reported_variance
                if math.isfinite(reported_variance) else 0.0)
            accepted_speed, _ = self.filter.update_wheel_speed(
                forward_speed, speed_variance)
            self.last_odometry_speed = forward_speed
            speed_reanchored = False
            recovery_mode = (
                self.gps_reacquisition_active
                or self.gps_mode.mode in (
                    GpsMode.GPS_LOST,
                    GpsMode.REACQUIRING,
                    GpsMode.DEGRADED))
            if not accepted_speed and recovery_mode:
                trusted_base_velocity = (
                    self._trusted_odometry_base_velocity(stamp_sec))
                if trusted_base_velocity is not None:
                    self.filter.reanchor_base_velocity(
                        trusted_base_velocity,
                        variance=max(speed_variance, 0.25))
                    accepted_speed = True
                    speed_reanchored = True
                    self.gps_replay.mark_barrier(
                        filter_stamp, self.filter.snapshot())
                    rospy.logwarn(
                        "[ESKF] velocity recovered from pure odometry: "
                        "%.3f m/s",
                        forward_speed)
            if accepted_speed:
                self.last_odometry_speed_accept_time = stamp_sec
                if not speed_reanchored:
                    self.gps_replay.record(
                        filter_stamp,
                        SpeedReplayEvent(forward_speed, speed_variance),
                        self.filter.snapshot())

        base_position, base_velocity = self.filter.base_state()
        physics_lag = self._physics_duration(max(lag, 0.0))
        synchronized_position = base_position - base_velocity * physics_lag
        synchronized_yaw = wrap_angle(
            self.filter.x[self.filter.YAW]
            - self.filter.last_corrected_yaw_rate * physics_lag)
        gps_age = (
            math.inf if self.last_gps_accept_time is None
            else max(0.0, stamp_sec - self.last_gps_accept_time))
        if gps_age <= self.odometry_alignment_gps_max_age:
            alignment_updated = self.odometry_alignment.update(
                synchronized_position,
                synchronized_yaw,
                local_position,
                local_yaw,
                stamp_sec)
            if alignment_updated and local_pose_continuous:
                # Relative odometry has slowly changing scale and yaw error.
                # Anchor at the latest GPS-backed pose instead of applying a
                # long-window rigid fit during the next shadow interval.
                self.odometry_shadow_anchor_local_position = (
                    local_position.copy())
                self.odometry_shadow_anchor_global_position = (
                    synchronized_position.copy())
                self.odometry_shadow_anchor_yaw = wrap_angle(
                    synchronized_yaw - local_yaw)
                self.odometry_shadow_anchor_samples += 1
            # Only drift accumulated after the last GPS-backed anchor is
            # relevant to a future shadow interval.
            self.odometry_anchor_distance = self.odometry_total_distance
            self.last_odometry_incremental_variance = 0.0
            return
        if not self.odometry_pose_aiding_enabled:
            return
        if not local_pose_continuous:
            self.filter.counters["odometry_rejected"] += 1
            return
        if gps_age < self.odometry_activation_delay:
            return
        if (
                self.odometry_shadow_anchor_samples < 10
                or self.odometry_shadow_anchor_local_position is None
                or self.odometry_shadow_anchor_global_position is None
                or self.odometry_shadow_anchor_yaw is None
                or self.odometry_alignment.anchor_stamp is None
                or stamp_sec - self.odometry_alignment.anchor_stamp
                > self.odometry_max_alignment_age):
            self.filter.counters["odometry_rejected"] += 1
            return
        slam_age = (
            math.inf if self.last_slam_accept_time is None
            else max(0.0, stamp_sec - self.last_slam_accept_time))
        if slam_age <= self.odometry_slam_fallback_age:
            return
        if (
                self.last_odometry_update_time is not None
                and stamp_sec - self.last_odometry_update_time
                < 1.0 / self.odometry_update_rate_hz):
            return
        self.last_odometry_update_time = stamp_sec

        local_delta = (
            local_position
            - self.odometry_shadow_anchor_local_position)
        global_position = (
            self.odometry_shadow_anchor_global_position
            + self.filter._rotation(self.odometry_shadow_anchor_yaw)
            @ local_delta)
        global_position += base_velocity * physics_lag
        distance = max(
            0.0,
            self.odometry_total_distance - self.odometry_anchor_distance)
        # pose.covariance is the absolute covariance of the pure-odometry
        # state since process start. Its map-axis diagonals rotate and can
        # decrease, so subtracting two snapshots does not produce a valid
        # relative covariance. Shadow uncertainty is instead grown from the
        # configured validated drift-per-distance model below.
        incremental_position_variance = 0.0
        self.last_odometry_incremental_variance = 0.0
        position_variance = self.odometry_correlation_inflation * (
            self.odometry_position_variance
            + incremental_position_variance
            + (self.odometry_position_drift_per_meter * distance)**2
            + (np.linalg.norm(base_velocity) * physics_lag)**2)
        accepted_pose, nis = self.filter.update_odometry_position(
            global_position, position_variance)
        if accepted_pose:
            self.last_odometry_pose_accept_time = stamp_sec
            self.gps_replay.mark_barrier(
                filter_stamp, self.filter.snapshot())
        else:
            rospy.logwarn_throttle(
                1.0, "[ESKF] rejected relative odometry: NIS=%.2f", nis)

    @synchronized
    def slam_degeneracy_callback(self, msg):
        stamp = self._stamp_or_now(msg.header).to_sec()
        self.slam_degenerate = msg.pose.covariance[0] >= 0.5
        self.slam_degeneracy_time = stamp

    def _remember_trusted_slam(
            self, local_position, local_yaw, stamp_sec,
            filter_position, filter_yaw):
        self.trusted_local_slam_position = local_position.copy()
        self.trusted_local_slam_yaw = local_yaw
        self.trusted_local_slam_stamp = stamp_sec
        self.trusted_filter_position = np.asarray(
            filter_position, dtype=float).copy()
        self.trusted_filter_yaw = float(filter_yaw)

    def _start_slam_continuity_recovery(
            self, local_position, local_yaw, stamp_sec,
            synchronized_position, synchronized_yaw,
            confirm_samples=None):
        if (
                self.trusted_local_slam_position is None
                or self.trusted_filter_position is None
                or self.trusted_filter_yaw is None):
            return False
        trusted_global_position, trusted_global_yaw = (
            self.slam_alignment.transform(
                self.trusted_local_slam_position,
                self.trusted_local_slam_yaw))
        continuity_position = (
            trusted_global_position
            + synchronized_position - self.trusted_filter_position)
        continuity_yaw = wrap_angle(
            trusted_global_yaw
            + wrap_angle(synchronized_yaw - self.trusted_filter_yaw))
        self.slam_alignment.recover_continuity(
            continuity_position,
            continuity_yaw,
            local_position,
            local_yaw,
            stamp_sec,
            start=True,
            confirm_samples=confirm_samples)
        return True

    def _update_slam_stationary_hint(
            self, local_position, stamp_sec, local_frame_jump):
        if local_frame_jump:
            self.slam_motion_history.clear()
            self.slam_stationary_hint_time = None
            self.slam_stationary_accel_reference = None
            return
        self.slam_motion_history.append(
            (stamp_sec, local_position.copy()))
        while (
                self.slam_motion_history
                and stamp_sec - self.slam_motion_history[0][0]
                > self.slam_motion_window):
            self.slam_motion_history.popleft()
        estimate = robust_gps_velocity(
            self.slam_motion_history,
            min_span=min(0.6, self.slam_motion_window * 0.5))
        if estimate is None:
            return
        velocity, _, residual_variance = estimate
        _, filter_velocity = self.filter.base_state()
        if (
                np.linalg.norm(velocity) <= self.slam_stationary_speed
                and np.linalg.norm(filter_velocity)
                <= self.slam_stationary_filter_speed_max
                and residual_variance
                <= self.slam_stationary_max_residual_variance):
            if (
                    self.slam_stationary_release_time is not None
                    and stamp_sec - self.slam_stationary_release_time
                    < self.slam_stationary_reactivation_delay):
                return
            hint_is_new = (
                self.slam_stationary_hint_time is None
                or stamp_sec - self.slam_stationary_hint_time
                > self.slam_stationary_hint_max_age)
            if hint_is_new:
                self.filter.counters["slam_stationary_hints"] += 1
                accel_samples = list(self.imu_windows[0])
                self.slam_stationary_accel_reference = float(
                    np.median(accel_samples)
                    if accel_samples else self.last_imu_input[0])
                self.slam_stationary_filtered_accel_delta = 0.0
            self.slam_stationary_hint_time = stamp_sec
        elif np.linalg.norm(velocity) > self.slam_stationary_speed * 2.0:
            self.slam_stationary_hint_time = None
            self.slam_stationary_accel_reference = None
            self.slam_stationary_filtered_accel_delta = 0.0

    @synchronized
    def slam_callback(self, msg):
        if not self.slam_aiding_enabled or not self.filter.initialized:
            return
        values = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.filter.counters["invalid_measurements"] += 1
            return
        quaternion = values[2:6]
        quaternion_norm = math.sqrt(sum(value**2 for value in quaternion))
        if quaternion_norm < 1e-6:
            self.filter.counters["invalid_measurements"] += 1
            return
        _, _, local_yaw = euler_from_quaternion(quaternion)
        local_yaw = wrap_angle(local_yaw)
        local_position = np.array(values[0:2], dtype=float)
        # LIO-SAM publishes the LiDAR origin; ESKF state/output use base_link.
        local_position -= (
            self.filter._rotation(local_yaw) @ self.lidar_lever)
        stamp = self._stamp_or_now(msg.header)
        stamp_sec = stamp.to_sec()
        self.last_slam_message_time = stamp_sec

        slam_lag = 0.0
        if self.last_predict_stamp is not None:
            slam_lag = (self.last_predict_stamp - stamp).to_sec()
            if slam_lag > self.slam_max_message_lag:
                self.filter.counters["slam_stale"] += 1
                return
        base_position, base_velocity = self.filter.base_state()
        lag = max(slam_lag, 0.0)
        physics_lag = self._physics_duration(lag)
        synchronized_position = base_position - base_velocity * physics_lag
        synchronized_yaw = wrap_angle(
            self.filter.x[self.filter.YAW]
            - self.filter.last_corrected_yaw_rate * physics_lag)
        gps_age = (
            math.inf if self.last_gps_accept_time is None
            else max(0.0, stamp_sec - self.last_gps_accept_time))
        gps_message_age = (
            math.inf if self.last_gps_message_time is None
            else max(0.0, stamp_sec - self.last_gps_message_time))
        degeneracy_is_fresh = (
            self.slam_degeneracy_time is not None
            and abs(stamp_sec - self.slam_degeneracy_time) <= 0.5)
        measurement_degenerate = (
            degeneracy_is_fresh and self.slam_degenerate)
        if measurement_degenerate:
            self.filter.counters["slam_degenerate"] += 1
        if (
                measurement_degenerate
                and not self.slam_degenerate_aiding_enabled):
            return
        previous_local_position = self.last_local_slam_position
        previous_local_yaw = self.last_local_slam_yaw
        previous_local_stamp = self.last_local_slam_stamp
        local_dt = None
        local_frame_jump = False
        if (
                previous_local_position is not None
                and previous_local_stamp is not None):
            local_dt = stamp_sec - previous_local_stamp
            if local_dt > 0.0:
                position_step = float(np.linalg.norm(
                    local_position - previous_local_position))
                yaw_step = abs(wrap_angle(
                    local_yaw - previous_local_yaw))
                position_limit = max(
                    self.slam_local_jump_distance,
                    (
                        np.linalg.norm(base_velocity)
                        + self.slam_local_jump_speed_margin)
                    * local_dt)
                yaw_limit = max(
                    self.slam_local_jump_yaw,
                    (
                        abs(self.filter.last_corrected_yaw_rate) + 0.5)
                    * local_dt)
                local_frame_jump = (
                    position_step > position_limit
                    or yaw_step > yaw_limit)
                if local_frame_jump:
                    self.filter.counters["slam_frame_jumps"] += 1
        if (
                self.last_local_slam_stamp is None
                or stamp_sec > self.last_local_slam_stamp):
            self.last_local_slam_position = local_position.copy()
            self.last_local_slam_yaw = local_yaw
            self.last_local_slam_stamp = stamp_sec
        if not measurement_degenerate:
            self._update_slam_stationary_hint(
                local_position, stamp_sec, local_frame_jump)

        if gps_age <= self.slam_alignment_gps_max_age:
            if measurement_degenerate:
                return
            alignment_updated = self.slam_alignment.update(
                synchronized_position,
                synchronized_yaw,
                local_position,
                local_yaw,
                stamp_sec)
            if alignment_updated and self.slam_alignment.ready:
                self._remember_trusted_slam(
                    local_position, local_yaw, stamp_sec,
                    synchronized_position, synchronized_yaw)
            return
        if (
                measurement_degenerate
                and (
                    local_frame_jump
                    or self.slam_alignment.continuity_pending
                    or self.slam_alignment.pending_count > 0
                    or self.slam_alignment.recovering)):
            return
        continuity_confirm_samples = (
            self.slam_alignment_reset_confirm_samples
            if (
                local_dt is not None
                and local_dt >= self.slam_continuity_fast_recovery_gap)
            else self.slam_continuity_confirm_samples)
        if self.slam_alignment.continuity_pending:
            old_global_position, old_global_yaw = (
                self.slam_alignment.transform(
                    local_position, local_yaw))
            old_transform_matches = (
                np.linalg.norm(
                    old_global_position - synchronized_position)
                <= self.slam_alignment_reset_distance * 3.0
                and abs(wrap_angle(
                    old_global_yaw - synchronized_yaw))
                <= self.slam_alignment_reset_yaw)
            if old_transform_matches:
                self.slam_alignment.cancel_continuity_recovery()
            elif local_frame_jump:
                if self._start_slam_continuity_recovery(
                        local_position,
                        local_yaw,
                        stamp_sec,
                        synchronized_position,
                        synchronized_yaw,
                        continuity_confirm_samples):
                    return
            else:
                recovered = self.slam_alignment.recover_continuity(
                    synchronized_position,
                    synchronized_yaw,
                    local_position,
                    local_yaw,
                    stamp_sec)
                if recovered:
                    self._remember_trusted_slam(
                        local_position, local_yaw, stamp_sec,
                        synchronized_position, synchronized_yaw)
                return
        if (
                local_frame_jump
                and self._start_slam_continuity_recovery(
                    local_position,
                    local_yaw,
                    stamp_sec,
                    synchronized_position,
                    synchronized_yaw,
                    continuity_confirm_samples)):
            return
        if self.slam_alignment.pending_count > 0:
            self.slam_alignment.update(
                synchronized_position,
                synchronized_yaw,
                local_position,
                local_yaw,
                stamp_sec)
            return
        if self.slam_alignment.recovering:
            self.slam_alignment.update(
                synchronized_position,
                synchronized_yaw,
                local_position,
                local_yaw,
                stamp_sec)
            return
        if gps_message_age < self.slam_activation_delay:
            return
        if (
                not self.slam_alignment.initialized
                or not self.slam_alignment.ready
                or self.slam_alignment.anchor_stamp is None
                or stamp_sec - self.slam_alignment.anchor_stamp
                > self.slam_max_alignment_age):
            self.filter.counters["slam_stale"] += 1
            return
        if gps_age < self.slam_activation_delay:
            return
        if (
                self.last_slam_update_time is not None
                and stamp_sec - self.last_slam_update_time
                < 1.0 / self.slam_update_rate_hz):
            return
        self.last_slam_update_time = stamp_sec

        global_position, _ = self.slam_alignment.transform(
            local_position, local_yaw)
        # The SLAM pose belongs to its header timestamp. Bring it to the
        # current filter epoch so transport delay cannot pull the state back.
        global_position += base_velocity * physics_lag
        distance = self.slam_alignment.distance_from_anchor(local_position)
        position_variance = self.slam_correlation_inflation * (
            self.filter.slam_position_variance
            + (self.slam_position_drift_per_meter * distance)**2
            + (np.linalg.norm(base_velocity) * physics_lag)**2)
        if measurement_degenerate:
            position_variance *= self.slam_degenerate_variance_scale
        accepted, nis = self.filter.update_slam_position(
            global_position,
            position_variance)
        if accepted:
            self.last_slam_accept_time = stamp_sec
            barrier_stamp = (
                self.last_predict_stamp.to_sec()
                if self.last_predict_stamp is not None else stamp_sec)
            self.gps_replay.mark_barrier(
                barrier_stamp, self.filter.snapshot())
            if measurement_degenerate:
                self.filter.counters["slam_degenerate_accepted"] += 1
            else:
                self._remember_trusted_slam(
                    local_position, local_yaw, stamp_sec,
                    synchronized_position, synchronized_yaw)
        else:
            rospy.logwarn_throttle(
                1.0, "[ESKF] rejected SLAM pose: NIS=%.2f", nis)

    def _apply_replay_event(self, event):
        if isinstance(event, SpeedReplayEvent):
            self.filter.update_wheel_speed(
                event.forward_speed, event.variance)
            return
        self.filter.predict(
            event.acceleration_body, event.yaw_rate, event.dt)
        if event.yaw_measurement is not None:
            self.filter.update_yaw(
                event.yaw_measurement, event.yaw_variance)
        if event.apply_nhc:
            self.filter.update_nonholonomic_constraint(event.yaw_rate)
        if event.apply_zupt:
            self.filter.update_zero_velocity()

    @synchronized
    def imu_callback(self, msg):
        stamp = self._stamp_or_now(msg.header)
        yaw_measurement = None
        if self._valid_quaternion(msg):
            quaternion = [
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w,
            ]
            _, _, yaw_measurement = euler_from_quaternion(quaternion)
            yaw_measurement = wrap_angle(yaw_measurement)
            self.last_imu_yaw = yaw_measurement

        raw_values = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.angular_velocity.z,
        )
        if not all(math.isfinite(value) for value in raw_values):
            self.filter.counters["invalid_measurements"] += 1
            rospy.logwarn_throttle(2.0, "[ESKF] non-finite IMU measurement")
            return
        imu_input = np.array([
            self._reject_spike(raw_values[index], index)
            for index in range(3)
        ])
        self.last_imu_input = imu_input

        if not self.filter.initialized:
            return
        if self.last_predict_stamp is None:
            self.last_predict_stamp = stamp
            return
        dt = (stamp - self.last_predict_stamp).to_sec()
        if dt <= 0.0:
            self.filter.counters["out_of_order_imu"] += 1
            rospy.logwarn_throttle(
                2.0, "[ESKF] ignored out-of-order IMU: dt=%.6fs", dt)
            return
        self.last_predict_stamp = stamp
        dt = min(self._physics_duration(dt), self.max_predict_dt)
        self.filter.predict(imu_input[0:2], imu_input[2], dt)

        yaw_variance = None
        if yaw_measurement is not None:
            yaw_variance = self.filter.imu_yaw_variance
            covariance = msg.orientation_covariance[8]
            if math.isfinite(covariance) and covariance > 0.0:
                yaw_variance = float(np.clip(covariance, 1e-5, 1.0))
            accepted, nis = self.filter.update_yaw(
                yaw_measurement, yaw_variance)
            if not accepted:
                rospy.logwarn_throttle(
                    1.0, "[ESKF] rejected IMU yaw outlier: NIS=%.2f", nis)

        now_sec = stamp.to_sec()
        apply_nhc = False
        _, base_velocity = self.filter.base_state()
        speed = float(np.linalg.norm(base_velocity))
        if (
                speed >= self.nhc_min_speed
                and (
                    self.last_nhc_stamp is None
                    or now_sec - self.last_nhc_stamp >= 1.0 / self.nhc_rate_hz)):
            apply_nhc = True
            self.filter.update_nonholonomic_constraint(imu_input[2])
            self.last_nhc_stamp = now_sec

        corrected_accel = (
            imu_input[0:2]
            - self.filter.x[self.filter.BAX:self.filter.BAY + 1])
        corrected_gyro = abs(
            imu_input[2] - self.filter.x[self.filter.BGZ])
        slam_stationary_hint = (
            self.slam_stationary_hint_time is not None
            and now_sec - self.slam_stationary_hint_time
            <= self.slam_stationary_hint_max_age)
        if slam_stationary_hint:
            release_accel = (
                corrected_accel[0]
                if self.slam_stationary_accel_reference is None
                else imu_input[0]
                - self.slam_stationary_accel_reference)
            self.slam_stationary_filtered_accel_delta = (
                0.8 * self.slam_stationary_filtered_accel_delta
                + 0.2 * release_accel)
            if (
                    abs(self.slam_stationary_filtered_accel_delta)
                    >= self.slam_stationary_release_accel):
                self.slam_stationary_hint_time = None
                self.slam_stationary_accel_reference = None
                self.slam_stationary_filtered_accel_delta = 0.0
                self.slam_stationary_release_time = now_sec
                slam_stationary_hint = False
        stationary = (
            (speed < self.zupt_speed or slam_stationary_hint)
            and np.linalg.norm(corrected_accel) < self.zupt_accel
            and corrected_gyro < self.zupt_gyro)
        odometry_speed_fresh = bool(
            self.odometry_aiding_enabled
            and self.last_odometry_message_time is not None
            and 0.0 <= now_sec - self.last_odometry_message_time
            <= max(0.5, 2.0 * self.odometry_max_message_lag))
        # The validated odometry's signed forward speed is an independent
        # motion indication even when its Kalman update is temporarily gated.
        # Never let a low filter speed trigger a false ZUPT while that source
        # says the vehicle is moving.
        if (
                stationary
                and odometry_speed_fresh
                and abs(self.last_odometry_speed) >= self.zupt_speed):
            stationary = False
            self.zupt_blocked_by_odometry += 1
        if slam_stationary_hint:
            stationary = not (
                odometry_speed_fresh
                and abs(self.last_odometry_speed) >= self.zupt_speed)
        apply_zupt = False
        if stationary:
            if self.zupt_candidate_since is None:
                self.zupt_candidate_since = now_sec
            elif now_sec - self.zupt_candidate_since >= self.zupt_hold:
                apply_zupt = True
                self.filter.update_zero_velocity()
        else:
            self.zupt_candidate_since = None
        replay_event = ImuReplayEvent(
            dt=dt,
            acceleration_body=imu_input[0:2],
            yaw_rate=imu_input[2],
            yaw_measurement=yaw_measurement,
            yaw_variance=yaw_variance,
            apply_nhc=apply_nhc,
            apply_zupt=apply_zupt)
        self.gps_replay.record(
            now_sec, replay_event, self.filter.snapshot())
        self.publish(stamp)

    def publish(self, stamp):
        base_position, base_velocity_map = self.filter.base_state()
        yaw = float(self.filter.x[self.filter.YAW])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        if np.linalg.norm(base_velocity_map) < self.speed_deadband:
            base_velocity_map[:] = 0.0

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(base_position[0])
        odom.pose.pose.position.y = float(base_position[1])
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        odom.twist.twist.linear.x = float(
            cos_yaw * base_velocity_map[0]
            + sin_yaw * base_velocity_map[1])
        odom.twist.twist.linear.y = float(
            -sin_yaw * base_velocity_map[0]
            + cos_yaw * base_velocity_map[1])
        odom.twist.twist.angular.z = self.filter.last_corrected_yaw_rate

        p = self.filter.p
        odom.pose.covariance[0] = float(p[self.filter.PX, self.filter.PX])
        odom.pose.covariance[1] = float(p[self.filter.PX, self.filter.PY])
        odom.pose.covariance[6] = float(p[self.filter.PY, self.filter.PX])
        odom.pose.covariance[7] = float(p[self.filter.PY, self.filter.PY])
        odom.pose.covariance[35] = float(
            p[self.filter.YAW, self.filter.YAW])
        odom.twist.covariance[0] = float(
            p[self.filter.VX, self.filter.VX])
        odom.twist.covariance[7] = float(
            p[self.filter.VY, self.filter.VY])
        odom.twist.covariance[35] = float(
            p[self.filter.BGZ, self.filter.BGZ]
            + self.filter.gyro_noise_std**2)
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    @synchronized
    def publish_diagnostics(self, event):
        # rospy.Time.now() follows /clock when use_sim_time is enabled.
        now_sec = rospy.Time.now().to_sec()
        gps_age = (
            None if self.last_gps_accept_time is None
            else max(0.0, now_sec - self.last_gps_accept_time))
        gps_message_age = (
            None if self.last_gps_message_time is None
            else max(0.0, now_sec - self.last_gps_message_time))
        slam_age = (
            None if self.last_slam_accept_time is None
            else max(0.0, now_sec - self.last_slam_accept_time))
        odometry_pose_age = (
            None if self.last_odometry_pose_accept_time is None
            else max(0.0, now_sec - self.last_odometry_pose_accept_time))
        odometry_speed_age = (
            None if self.last_odometry_speed_accept_time is None
            else max(0.0, now_sec - self.last_odometry_speed_accept_time))
        wheel_age = (
            None if self.last_wheel_accept_time is None
            else max(0.0, now_sec - self.last_wheel_accept_time))
        if not self.filter.initialized:
            mode = "initializing"
        elif gps_age is not None and gps_age <= 1.0:
            mode = "tracking"
        elif slam_age is not None and slam_age <= 1.0:
            mode = "slam_aided"
        elif odometry_pose_age is not None and odometry_pose_age <= 1.0:
            mode = "odometry_aided"
        elif gps_age is None or gps_age > 10.0:
            mode = "dead_reckoning"
        else:
            mode = "dead_reckoning"
        payload = {
            "mode": mode,
            "gps_age_sec": gps_age,
            "gps_message_age_sec": gps_message_age,
            "slam_age_sec": slam_age,
            "odometry_pose_age_sec": odometry_pose_age,
            "odometry_speed_age_sec": odometry_speed_age,
            "odometry_speed_mps": self.last_odometry_speed,
            "odometry_alignment_ready": self.odometry_alignment.ready,
            "odometry_alignment_resets": self.odometry_alignment.reset_count,
            "odometry_duplicate_stamps": self.odometry_duplicate_stamps,
            "odometry_local_jumps": self.odometry_local_jumps,
            "zupt_blocked_by_odometry": self.zupt_blocked_by_odometry,
            "odometry_shadow_distance_m": max(
                0.0,
                self.odometry_total_distance
                - self.odometry_anchor_distance),
            "odometry_incremental_position_variance": (
                self.last_odometry_incremental_variance),
            "odometry_time_scale": self.odometry_time_scale,
            "odometry_time_scale_updates": self.odometry_time_scale_updates,
            "wheel_age_sec": wheel_age,
            "wheel_speed_mps": self.last_wheel_speed,
            "wheel_angle_deg": self.last_wheel_angle_deg,
            "wheel_duplicate_stamps": self.wheel_duplicate_stamps,
            "wheel_uses_ego_position": False,
            "wheel_uses_ego_heading": False,
            "slam_alignment_ready": self.slam_alignment.ready,
            "slam_alignment_resets": self.slam_alignment.reset_count,
            "slam_alignment_orientation_yaw_deg": (
                None
                if self.slam_alignment.orientation_yaw_estimate is None
                else math.degrees(
                    self.slam_alignment.orientation_yaw_estimate)),
            "slam_degenerate": self.slam_degenerate,
            "position_std_m": [
                math.sqrt(max(self.filter.p[0, 0], 0.0)),
                math.sqrt(max(self.filter.p[1, 1], 0.0)),
            ],
            "yaw_std_deg": math.degrees(math.sqrt(max(
                self.filter.p[self.filter.YAW, self.filter.YAW], 0.0))),
            "accel_bias_mps2": self.filter.x[
                self.filter.BAX:self.filter.BAY + 1].tolist(),
            "gyro_bias_rps": float(self.filter.x[self.filter.BGZ]),
            "measurement_scales": dict(self.filter.measurement_scales),
            "last_nis": dict(self.filter.last_nis),
            "counters": dict(self.filter.counters),
        }
        payload.update(self.gps_mode.diagnostics(now_sec))
        payload.update(self.gps_replay.diagnostics())
        self.diagnostics_pub.publish(
            String(data=json.dumps(payload, sort_keys=True)))


if __name__ == "__main__":
    try:
        ESKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
