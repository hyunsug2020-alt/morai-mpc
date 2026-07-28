#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
from collections import deque

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def latlon_to_utm(latitude, longitude, zone):
    """Convert WGS84 latitude/longitude to UTM without an external dependency."""
    a = 6378137.0
    ecc_sq = 0.00669438
    k0 = 0.9996
    ecc_prime_sq = ecc_sq / (1.0 - ecc_sq)

    lat = math.radians(latitude)
    lon = math.radians(longitude)
    lon_origin = math.radians((zone - 1) * 6 - 180 + 3)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    tan_lat = math.tan(lat)
    n = a / math.sqrt(1.0 - ecc_sq * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ecc_prime_sq * cos_lat * cos_lat
    aa = cos_lat * (lon - lon_origin)

    m = a * (
        (1.0 - ecc_sq / 4.0 - 3.0 * ecc_sq**2 / 64.0
         - 5.0 * ecc_sq**3 / 256.0) * lat
        - (3.0 * ecc_sq / 8.0 + 3.0 * ecc_sq**2 / 32.0
           + 45.0 * ecc_sq**3 / 1024.0) * math.sin(2.0 * lat)
        + (15.0 * ecc_sq**2 / 256.0
           + 45.0 * ecc_sq**3 / 1024.0) * math.sin(4.0 * lat)
        - (35.0 * ecc_sq**3 / 3072.0) * math.sin(6.0 * lat)
    )

    easting = k0 * n * (
        aa + (1.0 - t + c) * aa**3 / 6.0
        + (5.0 - 18.0 * t + t**2 + 72.0 * c
           - 58.0 * ecc_prime_sq) * aa**5 / 120.0
    ) + 500000.0

    northing = k0 * (
        m + n * tan_lat * (
            aa**2 / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c**2) * aa**4 / 24.0
            + (61.0 - 58.0 * t + t**2 + 600.0 * c
               - 330.0 * ecc_prime_sq) * aa**6 / 720.0
        )
    )
    if latitude < 0.0:
        northing += 10000000.0
    return easting, northing


def robust_gps_velocity(samples, min_span=0.8, min_samples=5):
    """Estimate map-frame velocity from a robust fit over recent GPS fixes."""
    if len(samples) < min_samples:
        return None
    ordered = sorted(samples, key=lambda sample: sample[0])
    times = np.asarray([sample[0] for sample in ordered], dtype=float)
    positions = np.asarray([sample[1] for sample in ordered], dtype=float)
    if times[-1] - times[0] < min_span:
        return None

    centered_times = times - np.mean(times)
    denominator = float(centered_times @ centered_times)
    if denominator < 1e-6:
        return None
    position_mean = np.mean(positions, axis=0)
    velocity = centered_times @ (positions - position_mean) / denominator
    predicted = position_mean + np.outer(centered_times, velocity)
    residuals = np.linalg.norm(positions - predicted, axis=1)
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    threshold = max(1.0, median + 3.0 * 1.4826 * mad)
    inliers = residuals <= threshold
    if np.count_nonzero(inliers) >= min_samples:
        times = times[inliers]
        positions = positions[inliers]
        centered_times = times - np.mean(times)
        denominator = float(centered_times @ centered_times)
        position_mean = np.mean(positions, axis=0)
        velocity = (
            centered_times @ (positions - position_mean) / denominator)
        predicted = position_mean + np.outer(centered_times, velocity)
        residuals = np.linalg.norm(positions - predicted, axis=1)

    robust_residual_std = (
        float(np.median(residuals)) / math.sqrt(2.0 * math.log(2.0))
        if len(residuals) else 0.0)
    return velocity, denominator, robust_residual_std**2


class RobustPlanarESKF:
    """Planar error-state filter with IMU bias estimation and robust updates."""

    PX = 0
    PY = 1
    VX = 2
    VY = 3
    YAW = 4
    BAX = 5
    BAY = 6
    BGZ = 7
    SIZE = 8

    def __init__(self, config=None, gps_from_imu=None, imu_lever=None):
        config = config or {}
        self.accel_noise_std = float(config.get("accel_noise_std", 0.35))
        self.gyro_noise_std = float(config.get("gyro_noise_std", 0.015))
        self.accel_bias_rw_std = float(
            config.get("accel_bias_random_walk_std", 0.02))
        self.gyro_bias_rw_std = float(
            config.get("gyro_bias_random_walk_std", 0.002))
        self.gps_position_variance = float(
            config.get("gps_position_variance", 1.0))
        self.gps_velocity_variance_floor = float(
            config.get("gps_velocity_variance_floor", 0.5))
        self.imu_yaw_variance = float(
            config.get("imu_yaw_variance", 0.02))
        self.nhc_variance = float(config.get("nhc_variance", 0.09))
        self.zupt_velocity_variance = float(
            config.get("zupt_velocity_variance", 0.01))
        self.slam_position_variance = float(
            config.get("slam_position_variance", 0.25))
        self.slam_yaw_variance = float(
            config.get("slam_yaw_variance", 0.0025))
        self.gps_nis_soft = float(config.get("gps_nis_soft", 5.991))
        self.gps_nis_hard = float(config.get("gps_nis_hard", 13.816))
        self.velocity_nis_soft = float(
            config.get("velocity_nis_soft", 5.991))
        self.velocity_nis_hard = float(
            config.get("velocity_nis_hard", 13.816))
        self.yaw_nis_soft = float(config.get("yaw_nis_soft", 3.841))
        self.yaw_nis_hard = float(config.get("yaw_nis_hard", 10.828))
        self.constraint_nis_hard = float(
            config.get("constraint_nis_hard", 10.828))
        self.slam_nis_soft = float(config.get("slam_nis_soft", 7.815))
        self.slam_nis_hard = float(config.get("slam_nis_hard", 16.266))
        self.adaptive_alpha = float(config.get("adaptive_alpha", 0.04))
        self.adaptive_scale_min = float(
            config.get("adaptive_scale_min", 0.5))
        self.adaptive_scale_max = float(
            config.get("adaptive_scale_max", 16.0))
        self.max_accel_bias = float(config.get("max_accel_bias", 3.0))
        self.max_gyro_bias = float(config.get("max_gyro_bias", 0.3))

        self.gps_from_imu = np.asarray(
            gps_from_imu if gps_from_imu is not None else [0.0, 0.0],
            dtype=float)
        self.imu_lever = np.asarray(
            imu_lever if imu_lever is not None else [0.0, 0.0],
            dtype=float)
        self.x = np.zeros(self.SIZE, dtype=float)
        self.p = np.diag([
            4.0, 4.0, 9.0, 9.0, 0.5,
            0.25, 0.25, 0.0025,
        ])
        self.initialized = False
        self.last_corrected_yaw_rate = 0.0
        self.measurement_scales = {
            "gps_position": 1.0,
            "gps_velocity": 1.0,
            "imu_yaw": 1.0,
            "slam_pose": 1.0,
        }
        self.last_nis = {
            "gps_position": None,
            "gps_velocity": None,
            "imu_yaw": None,
            "nhc": None,
            "zupt": None,
            "slam_pose": None,
        }
        self.counters = {
            "gps_accepted": 0,
            "gps_rejected": 0,
            "gps_reacquired": 0,
            "velocity_accepted": 0,
            "velocity_rejected": 0,
            "yaw_accepted": 0,
            "yaw_rejected": 0,
            "nhc_accepted": 0,
            "nhc_rejected": 0,
            "zupt_accepted": 0,
            "imu_spikes": 0,
            "invalid_measurements": 0,
            "stale_gps": 0,
            "out_of_order_imu": 0,
            "slam_accepted": 0,
            "slam_rejected": 0,
            "slam_degenerate": 0,
            "slam_degenerate_accepted": 0,
            "slam_stationary_hints": 0,
            "slam_stale": 0,
            "slam_frame_jumps": 0,
        }

    @staticmethod
    def _rotation(yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return np.array([
            [cos_yaw, -sin_yaw],
            [sin_yaw, cos_yaw],
        ])

    def initialize(self, gps_position, yaw):
        yaw = wrap_angle(float(yaw))
        gps_position = np.asarray(gps_position, dtype=float)
        self.x.fill(0.0)
        self.x[self.YAW] = yaw
        self.x[0:2] = (
            gps_position - self._rotation(yaw) @ self.gps_from_imu)
        self.p = np.diag([
            self.gps_position_variance,
            self.gps_position_variance,
            4.0,
            4.0,
            max(self.imu_yaw_variance, 1e-4),
            0.25,
            0.25,
            0.0025,
        ])
        self.initialized = True

    def predict(self, acceleration_body, yaw_rate_measurement, dt):
        acceleration_body = np.asarray(acceleration_body, dtype=float)
        yaw = float(self.x[self.YAW])
        accel_unbiased = acceleration_body - self.x[self.BAX:self.BAY + 1]
        yaw_rate = float(yaw_rate_measurement) - self.x[self.BGZ]
        rotation = self._rotation(yaw)
        acceleration_map = rotation @ accel_unbiased
        d_accel_d_yaw = np.array([
            -math.sin(yaw) * accel_unbiased[0]
            - math.cos(yaw) * accel_unbiased[1],
            math.cos(yaw) * accel_unbiased[0]
            - math.sin(yaw) * accel_unbiased[1],
        ])

        self.x[0:2] += self.x[2:4] * dt + 0.5 * acceleration_map * dt**2
        self.x[2:4] += acceleration_map * dt
        self.x[self.YAW] = wrap_angle(yaw + yaw_rate * dt)
        self.last_corrected_yaw_rate = yaw_rate

        f = np.eye(self.SIZE)
        f[0:2, 2:4] = np.eye(2) * dt
        f[0:2, self.YAW] = 0.5 * d_accel_d_yaw * dt**2
        f[0:2, self.BAX:self.BAY + 1] = -0.5 * rotation * dt**2
        f[2:4, self.YAW] = d_accel_d_yaw * dt
        f[2:4, self.BAX:self.BAY + 1] = -rotation * dt
        f[self.YAW, self.BGZ] = -dt

        accel_var = self.accel_noise_std**2
        gyro_var = self.gyro_noise_std**2
        q = np.zeros((self.SIZE, self.SIZE), dtype=float)
        q[0:2, 0:2] = np.eye(2) * 0.25 * accel_var * dt**4
        q[0:2, 2:4] = np.eye(2) * 0.5 * accel_var * dt**3
        q[2:4, 0:2] = q[0:2, 2:4]
        q[2:4, 2:4] = np.eye(2) * accel_var * dt**2
        q[self.YAW, self.YAW] = gyro_var * dt**2
        q[self.BAX:self.BAY + 1, self.BAX:self.BAY + 1] = (
            np.eye(2) * self.accel_bias_rw_std**2 * dt)
        q[self.BGZ, self.BGZ] = self.gyro_bias_rw_std**2 * dt
        self.p = f @ self.p @ f.T + q
        self._stabilize_covariance()

    def _stabilize_covariance(self):
        self.p = 0.5 * (self.p + self.p.T)
        diagonal = np.maximum(np.diag(self.p), 1e-12)
        np.fill_diagonal(self.p, diagonal)

    def _inject(self, delta):
        self.x += delta
        self.x[self.YAW] = wrap_angle(self.x[self.YAW])
        self.x[self.BAX:self.BAY + 1] = np.clip(
            self.x[self.BAX:self.BAY + 1],
            -self.max_accel_bias, self.max_accel_bias)
        self.x[self.BGZ] = float(np.clip(
            self.x[self.BGZ], -self.max_gyro_bias, self.max_gyro_bias))

    def _update(self, innovation, h, r, name, soft_gate=None,
                hard_gate=None, angle=False, adaptive=False):
        innovation = np.atleast_1d(np.asarray(innovation, dtype=float))
        if angle:
            innovation[0] = wrap_angle(innovation[0])
        h = np.atleast_2d(np.asarray(h, dtype=float))
        r = np.atleast_2d(np.asarray(r, dtype=float))
        scale = self.measurement_scales.get(name, 1.0) if adaptive else 1.0
        r_scaled = r * scale
        s = h @ self.p @ h.T + r_scaled
        try:
            solved = np.linalg.solve(s, innovation)
        except np.linalg.LinAlgError:
            self.counters["invalid_measurements"] += 1
            return False, math.inf
        nis = float(innovation.T @ solved)
        self.last_nis[name] = nis

        if hard_gate is not None and nis > hard_gate:
            rejected_key = {
                "gps_position": "gps_rejected",
                "gps_velocity": "velocity_rejected",
                "imu_yaw": "yaw_rejected",
                "nhc": "nhc_rejected",
                "slam_pose": "slam_rejected",
            }.get(name)
            if rejected_key in self.counters:
                self.counters[rejected_key] += 1
            return False, nis

        if soft_gate is not None and nis > soft_gate:
            r_scaled *= max(1.0, nis / soft_gate)
            s = h @ self.p @ h.T + r_scaled
        try:
            kalman_gain = np.linalg.solve(
                s.T, (self.p @ h.T).T).T
        except np.linalg.LinAlgError:
            self.counters["invalid_measurements"] += 1
            return False, nis

        delta = kalman_gain @ innovation
        self._inject(delta)
        identity = np.eye(self.SIZE)
        ikh = identity - kalman_gain @ h
        self.p = (
            ikh @ self.p @ ikh.T
            + kalman_gain @ r_scaled @ kalman_gain.T)
        self._stabilize_covariance()

        accepted_key = {
            "gps_position": "gps_accepted",
            "gps_velocity": "velocity_accepted",
            "imu_yaw": "yaw_accepted",
            "nhc": "nhc_accepted",
            "zupt": "zupt_accepted",
            "slam_pose": "slam_accepted",
        }.get(name)
        if accepted_key in self.counters:
            self.counters[accepted_key] += 1

        if adaptive and name in self.measurement_scales:
            normalized_nis = nis / max(innovation.size, 1)
            target = float(np.clip(
                normalized_nis,
                self.adaptive_scale_min,
                self.adaptive_scale_max))
            old_scale = self.measurement_scales[name]
            self.measurement_scales[name] = float(np.clip(
                (1.0 - self.adaptive_alpha) * old_scale
                + self.adaptive_alpha * target,
                self.adaptive_scale_min,
                self.adaptive_scale_max))
        return True, nis

    def update_gps_position(self, gps_position, variance=None):
        gps_position = np.asarray(gps_position, dtype=float)
        yaw = float(self.x[self.YAW])
        lever_world = self._rotation(yaw) @ self.gps_from_imu
        expected = self.x[0:2] + lever_world
        h = np.zeros((2, self.SIZE), dtype=float)
        h[0:2, 0:2] = np.eye(2)
        h[:, self.YAW] = np.array([-lever_world[1], lever_world[0]])
        position_variance = (
            self.gps_position_variance if variance is None else variance)
        return self._update(
            gps_position - expected,
            h,
            np.eye(2) * position_variance,
            "gps_position",
            self.gps_nis_soft,
            self.gps_nis_hard,
            adaptive=True)

    def reanchor_gps_position(self, gps_position, variance=None):
        """Reacquire a consensus GPS fix after a confirmed receiver outage."""
        gps_position = np.asarray(gps_position, dtype=float)
        yaw = float(self.x[self.YAW])
        self.x[0:2] = (
            gps_position - self._rotation(yaw) @ self.gps_from_imu)
        self.p[0:2, :] = 0.0
        self.p[:, 0:2] = 0.0
        position_variance = (
            self.gps_position_variance if variance is None else variance)
        self.p[self.PX, self.PX] = max(float(position_variance), 1e-4)
        self.p[self.PY, self.PY] = max(float(position_variance), 1e-4)
        self.p[self.VX, self.VX] = max(self.p[self.VX, self.VX], 4.0)
        self.p[self.VY, self.VY] = max(self.p[self.VY, self.VY], 4.0)
        self.measurement_scales["gps_position"] = 1.0
        self.measurement_scales["gps_velocity"] = 1.0
        self.counters["gps_reacquired"] += 1
        self._stabilize_covariance()

    def update_gps_velocity(self, velocity_map, variance):
        h = np.zeros((2, self.SIZE), dtype=float)
        h[0:2, 2:4] = np.eye(2)
        return self._update(
            np.asarray(velocity_map, dtype=float) - self.x[2:4],
            h,
            np.eye(2) * max(variance, self.gps_velocity_variance_floor),
            "gps_velocity",
            self.velocity_nis_soft,
            self.velocity_nis_hard,
            adaptive=True)

    def update_yaw(self, yaw_measurement, variance=None):
        h = np.zeros((1, self.SIZE), dtype=float)
        h[0, self.YAW] = 1.0
        yaw_variance = self.imu_yaw_variance if variance is None else variance
        return self._update(
            [wrap_angle(yaw_measurement - self.x[self.YAW])],
            h,
            [[yaw_variance]],
            "imu_yaw",
            self.yaw_nis_soft,
            self.yaw_nis_hard,
            angle=True,
            adaptive=True)

    def update_nonholonomic_constraint(self, yaw_rate_measurement):
        yaw = float(self.x[self.YAW])
        vx = float(self.x[self.VX])
        vy = float(self.x[self.VY])
        measured_rate = float(yaw_rate_measurement)
        body_lateral_velocity = (
            -math.sin(yaw) * vx + math.cos(yaw) * vy
            - (measured_rate - self.x[self.BGZ]) * self.imu_lever[0])
        h = np.zeros((1, self.SIZE), dtype=float)
        h[0, self.VX] = -math.sin(yaw)
        h[0, self.VY] = math.cos(yaw)
        h[0, self.YAW] = -math.cos(yaw) * vx - math.sin(yaw) * vy
        h[0, self.BGZ] = self.imu_lever[0]
        return self._update(
            [-body_lateral_velocity],
            h,
            [[self.nhc_variance]],
            "nhc",
            hard_gate=self.constraint_nis_hard)

    def update_zero_velocity(self):
        h = np.zeros((2, self.SIZE), dtype=float)
        h[0:2, 2:4] = np.eye(2)
        return self._update(
            -self.x[2:4],
            h,
            np.eye(2) * self.zupt_velocity_variance,
            "zupt",
            hard_gate=self.gps_nis_hard)

    def update_slam_pose(self, base_position, yaw, position_variance,
                         yaw_variance):
        base_position = np.asarray(base_position, dtype=float)
        state_yaw = float(self.x[self.YAW])
        imu_world = self._rotation(state_yaw) @ self.imu_lever
        expected_position = self.x[0:2] - imu_world
        innovation = np.array([
            base_position[0] - expected_position[0],
            base_position[1] - expected_position[1],
            wrap_angle(yaw - state_yaw),
        ])
        h = np.zeros((3, self.SIZE), dtype=float)
        h[0:2, 0:2] = np.eye(2)
        h[0:2, self.YAW] = np.array([
            imu_world[1], -imu_world[0]])
        h[2, self.YAW] = 1.0
        r = np.diag([
            max(position_variance, self.slam_position_variance),
            max(position_variance, self.slam_position_variance),
            max(yaw_variance, self.slam_yaw_variance),
        ])
        return self._update(
            innovation,
            h,
            r,
            "slam_pose",
            self.slam_nis_soft,
            self.slam_nis_hard,
            adaptive=True)

    def update_slam_position(self, base_position, position_variance):
        base_position = np.asarray(base_position, dtype=float)
        state_yaw = float(self.x[self.YAW])
        imu_world = self._rotation(state_yaw) @ self.imu_lever
        expected_position = self.x[0:2] - imu_world
        h = np.zeros((2, self.SIZE), dtype=float)
        h[0:2, 0:2] = np.eye(2)
        h[0:2, self.YAW] = np.array([
            imu_world[1], -imu_world[0]])
        variance = max(position_variance, self.slam_position_variance)
        return self._update(
            base_position - expected_position,
            h,
            np.eye(2) * variance,
            "slam_pose",
            self.slam_nis_soft,
            self.slam_nis_hard,
            adaptive=True)

    def base_state(self):
        yaw = float(self.x[self.YAW])
        rotation = self._rotation(yaw)
        imu_world = rotation @ self.imu_lever
        base_position = self.x[0:2] - imu_world
        omega = self.last_corrected_yaw_rate
        base_velocity_map = self.x[2:4] + np.array([
            omega * imu_world[1],
            -omega * imu_world[0],
        ])
        return base_position, base_velocity_map


class SE2Alignment:
    """Robustly align a local SLAM frame with the global MORAI map frame."""

    def __init__(self, alpha=0.05, reset_distance=5.0,
                 reset_yaw=math.radians(20.0), reset_confirm_samples=3,
                 window_size=600, min_samples=10, min_baseline=12.0,
                 continuity_max_failures=5,
                 continuity_confirm_samples=10):
        self.alpha = float(alpha)
        self.reset_distance = float(reset_distance)
        self.reset_yaw = float(reset_yaw)
        self.reset_confirm_samples = int(reset_confirm_samples)
        self.min_samples = int(min_samples)
        self.min_baseline = float(min_baseline)
        self.continuity_max_failures = int(continuity_max_failures)
        self.continuity_confirm_samples = max(
            int(continuity_confirm_samples), self.reset_confirm_samples)
        self.continuity_target_count = self.continuity_confirm_samples
        self.translation = np.zeros(2, dtype=float)
        self.yaw = 0.0
        self.initialized = False
        self.ready = False
        self.recovering = False
        self.anchor_local_position = None
        self.anchor_stamp = None
        self.reset_count = 0
        self.correspondences = deque(maxlen=int(window_size))
        self.pending_position_correction = None
        self.pending_yaw = None
        self.pending_count = 0
        self.continuity_translation = None
        self.continuity_yaw = None
        self.continuity_count = 0
        self.continuity_failures = 0
        self.continuity_correspondences = []

    @staticmethod
    def _rotation(yaw):
        return RobustPlanarESKF._rotation(yaw)

    @staticmethod
    def _circular_mean(angles):
        angles = np.asarray(angles, dtype=float)
        return math.atan2(
            float(np.mean(np.sin(angles))),
            float(np.mean(np.cos(angles))))

    @staticmethod
    def _fit_rigid_transform(local_points, global_points):
        local_mean = np.mean(local_points, axis=0)
        global_mean = np.mean(global_points, axis=0)
        local_centered = local_points - local_mean
        global_centered = global_points - global_mean
        covariance = local_centered.T @ global_centered
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        translation = global_mean - rotation @ local_mean
        return translation, wrap_angle(yaw)

    def _fit_correspondences(self):
        if len(self.correspondences) < self.min_samples:
            return None
        local_points = np.asarray([
            sample[0] for sample in self.correspondences], dtype=float)
        global_points = np.asarray([
            sample[1] for sample in self.correspondences], dtype=float)
        orientation_differences = np.asarray([
            sample[2] for sample in self.correspondences], dtype=float)
        local_centered = local_points - np.mean(local_points, axis=0)
        baseline = float(np.max(np.linalg.norm(local_centered, axis=1)))

        if baseline >= self.min_baseline:
            translation, yaw = self._fit_rigid_transform(
                local_points, global_points)
            predicted = (
                (self._rotation(yaw) @ local_points.T).T
                + translation)
            residuals = np.linalg.norm(predicted - global_points, axis=1)
            median = float(np.median(residuals))
            mad = float(np.median(np.abs(residuals - median)))
            threshold = max(1.0, median + 3.0 * 1.4826 * mad)
            inliers = residuals <= threshold
            if np.count_nonzero(inliers) >= self.min_samples:
                translation, yaw = self._fit_rigid_transform(
                    local_points[inliers], global_points[inliers])
        else:
            yaw = self._circular_mean(orientation_differences)
            rotation = self._rotation(yaw)
            translations = (
                global_points - (rotation @ local_points.T).T)
            translation = np.median(translations, axis=0)
        return np.asarray(translation, dtype=float), wrap_angle(yaw)

    def _apply_fit(self):
        fitted = self._fit_correspondences()
        if fitted is None:
            return
        fitted_translation, fitted_yaw = fitted
        if self.ready:
            self.translation = (
                (1.0 - self.alpha) * self.translation
                + self.alpha * fitted_translation)
            self.yaw = wrap_angle(
                self.yaw
                + self.alpha * wrap_angle(fitted_yaw - self.yaw))
        else:
            self.translation = fitted_translation
            self.yaw = fitted_yaw
        self.ready = True
        self.recovering = False

    @property
    def continuity_pending(self):
        return self.continuity_translation is not None

    @property
    def orientation_yaw_estimate(self):
        if not self.correspondences:
            return None
        return self._circular_mean([
            sample[2] for sample in self.correspondences
        ])

    def cancel_continuity_recovery(self):
        self.continuity_translation = None
        self.continuity_yaw = None
        self.continuity_count = 0
        self.continuity_failures = 0
        self.continuity_correspondences = []
        self.continuity_target_count = self.continuity_confirm_samples

    def recover_continuity(self, global_position, global_yaw,
                           local_position, local_yaw, stamp, start=False,
                           confirm_samples=None):
        """Transfer a trusted transform across a persistent local-frame jump."""
        global_position = np.asarray(global_position, dtype=float)
        local_position = np.asarray(local_position, dtype=float)
        if start or not self.continuity_pending:
            candidate_yaw = wrap_angle(global_yaw - local_yaw)
            self.continuity_yaw = candidate_yaw
            self.continuity_translation = (
                global_position
                - self._rotation(candidate_yaw) @ local_position)
            self.continuity_count = 1
            self.continuity_failures = 0
            self.continuity_target_count = max(
                self.reset_confirm_samples,
                int(
                    self.continuity_confirm_samples
                    if confirm_samples is None else confirm_samples))
            self.continuity_correspondences = [(
                local_position.copy(),
                global_position.copy(),
                candidate_yaw)]
            return False

        predicted_position = (
            self.continuity_translation
            + self._rotation(self.continuity_yaw) @ local_position)
        predicted_yaw = wrap_angle(self.continuity_yaw + local_yaw)
        consistent = (
            np.linalg.norm(predicted_position - global_position)
            <= self.reset_distance * 3.0
            and abs(wrap_angle(predicted_yaw - global_yaw))
            <= self.reset_yaw)
        if not consistent:
            self.continuity_failures += 1
            if self.continuity_failures > self.continuity_max_failures:
                self.cancel_continuity_recovery()
            return False

        self.continuity_correspondences.append((
            local_position.copy(),
            global_position.copy(),
            wrap_angle(global_yaw - local_yaw)))
        self.continuity_yaw = self._circular_mean([
            sample[2] for sample in self.continuity_correspondences
        ])
        rotation = self._rotation(self.continuity_yaw)
        translations = np.asarray([
            sample_global - rotation @ sample_local
            for sample_local, sample_global, _
            in self.continuity_correspondences
        ])
        self.continuity_translation = np.median(
            translations, axis=0)
        self.continuity_count += 1
        if self.continuity_count < self.continuity_target_count:
            return False
        self.translation = self.continuity_translation
        self.yaw = self.continuity_yaw
        self.ready = True
        self.recovering = False
        self.reset_count += 1
        self.correspondences.clear()
        self.correspondences.append((
            local_position.copy(),
            global_position.copy(),
            wrap_angle(global_yaw - local_yaw)))
        self.anchor_local_position = local_position.copy()
        self.anchor_stamp = float(stamp)
        self.cancel_continuity_recovery()
        return True

    def update(self, global_position, global_yaw,
               local_position, local_yaw, stamp):
        global_position = np.asarray(global_position, dtype=float)
        local_position = np.asarray(local_position, dtype=float)
        candidate_yaw = wrap_angle(global_yaw - local_yaw)
        candidate_translation = (
            global_position
            - self._rotation(candidate_yaw) @ local_position)
        if not self.initialized:
            self.translation = candidate_translation
            self.yaw = candidate_yaw
            self.initialized = True
            self.correspondences.append((
                local_position.copy(),
                global_position.copy(),
                candidate_yaw))
        else:
            predicted_position, _ = self.transform(
                local_position, local_yaw)
            position_correction = global_position - predicted_position
            position_error = np.linalg.norm(position_correction)
            yaw_error = abs(wrap_angle(candidate_yaw - self.yaw))
            is_large_change = (
                len(self.correspondences) >= 2 * self.min_samples
                and (
                    position_error > self.reset_distance
                    or yaw_error > self.reset_yaw))
            if is_large_change:
                pending_is_consistent = (
                    self.pending_position_correction is not None
                    and np.linalg.norm(
                        position_correction
                        - self.pending_position_correction)
                    <= self.reset_distance * 0.5
                    and abs(wrap_angle(
                        candidate_yaw - self.pending_yaw))
                    <= self.reset_yaw * 0.5)
                if pending_is_consistent:
                    self.pending_count += 1
                    self.pending_position_correction = (
                        0.5 * self.pending_position_correction
                        + 0.5 * position_correction)
                    self.pending_yaw = wrap_angle(
                        self.pending_yaw
                        + 0.5 * wrap_angle(
                            candidate_yaw - self.pending_yaw))
                else:
                    self.pending_position_correction = position_correction
                    self.pending_yaw = candidate_yaw
                    self.pending_count = 1
                if self.pending_count < self.reset_confirm_samples:
                    return False
                reset_yaw = self.pending_yaw
                self.translation = (
                    global_position
                    - self._rotation(reset_yaw) @ local_position)
                self.yaw = reset_yaw
                self.reset_count += 1
                self.ready = False
                self.recovering = True
                self.correspondences.clear()
                self.pending_position_correction = None
                self.pending_yaw = None
                self.pending_count = 0
            else:
                self.pending_position_correction = None
                self.pending_yaw = None
                self.pending_count = 0
            self.correspondences.append((
                local_position.copy(),
                global_position.copy(),
                candidate_yaw))
            self._apply_fit()

        self.anchor_local_position = local_position.copy()
        self.anchor_stamp = float(stamp)
        return True

    def transform(self, local_position, local_yaw):
        local_position = np.asarray(local_position, dtype=float)
        global_position = (
            self.translation
            + self._rotation(self.yaw) @ local_position)
        global_yaw = wrap_angle(self.yaw + local_yaw)
        return global_position, global_yaw

    def distance_from_anchor(self, local_position):
        if self.anchor_local_position is None:
            return math.inf
        return float(np.linalg.norm(
            np.asarray(local_position, dtype=float)
            - self.anchor_local_position))


class ESKFNode:
    """ROS wrapper for robust planar MORAI GPS/IMU localization."""

    def __init__(self):
        rospy.init_node("eskf_node")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu")
        self.gps_topic = rospy.get_param("~gps_topic", "/gps")
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
        self.speed_deadband = float(
            rospy.get_param("~speed_deadband_mps", 0.15))
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
            float(rospy.get_param("~lidar_x", 1.676)),
            float(rospy.get_param("~lidar_y", 0.005)),
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
                "imu_yaw_variance",
                "nhc_variance",
                "zupt_velocity_variance",
                "slam_position_variance",
                "slam_yaw_variance",
                "gps_nis_soft",
                "gps_nis_hard",
                "velocity_nis_soft",
                "velocity_nis_hard",
                "yaw_nis_soft",
                "yaw_nis_hard",
                "constraint_nis_hard",
                "slam_nis_soft",
                "slam_nis_hard",
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

        self.last_predict_stamp = None
        self.last_imu_yaw = None
        self.last_imu_input = np.zeros(3, dtype=float)
        self.last_nhc_stamp = None
        self.last_gps_message_time = None
        self.last_gps_accept_time = None
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
        self.slam_sub = rospy.Subscriber(
            self.slam_topic, Odometry, self.slam_callback, queue_size=20)
        self.slam_degeneracy_sub = rospy.Subscriber(
            self.slam_degeneracy_topic,
            Odometry,
            self.slam_degeneracy_callback,
            queue_size=20)
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

    def gps_callback(self, msg):
        values = (
            msg.latitude, msg.longitude, msg.eastOffset, msg.northOffset)
        if not all(math.isfinite(value) for value in values):
            self.filter.counters["invalid_measurements"] += 1
            rospy.logwarn_throttle(2.0, "[ESKF] non-finite GPS measurement")
            return
        if msg.status == self.invalid_gps_status:
            self.filter.counters["gps_rejected"] += 1
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
        previous_gps_message_time = self.last_gps_message_time
        self.last_gps_message_time = stamp_sec
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
                _, base_velocity = self.filter.base_state()
                projected_positions = np.asarray([
                    position + base_velocity * (stamp_sec - sample_stamp)
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
                    self.filter.reanchor_gps_position(consensus_position)
                    self.gps_history.clear()
                    self.gps_history.append(
                        (stamp_sec, consensus_position.copy()))
                    self.last_gps_accept_time = stamp_sec
                    self.gps_reacquisition_active = False
                    self.gps_reacquisition_buffer.clear()
                    rospy.logwarn(
                        "[ESKF] GPS reacquired from %d/%d consensus fixes",
                        len(inliers), len(projected_positions))
                    self.publish(stamp)
                    return

        gps_lag = 0.0
        if self.last_predict_stamp is not None:
            gps_lag = (self.last_predict_stamp - stamp).to_sec()
            if gps_lag > self.max_gps_lag:
                self.filter.counters["stale_gps"] += 1
                rospy.logwarn_throttle(
                    1.0,
                    "[ESKF] rejected stale GPS: lag=%.3fs",
                    gps_lag)
                return
        _, base_velocity = self.filter.base_state()
        delay_variance = (
            np.linalg.norm(base_velocity) * max(gps_lag, 0.0))**2
        accepted, nis = self.filter.update_gps_position(
            gps_position,
            self.filter.gps_position_variance + delay_variance)
        if not accepted:
            rospy.logwarn_throttle(
                1.0, "[ESKF] rejected GPS outlier: NIS=%.2f", nis)
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
        if velocity_estimate is not None:
            gps_velocity, denominator, residual_variance = velocity_estimate
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
                + self.filter.gps_velocity_variance_floor)
            accepted_velocity, velocity_nis = (
                self.filter.update_gps_velocity(
                    gps_velocity, velocity_variance))
            if not accepted_velocity:
                rospy.logwarn_throttle(
                    1.0,
                    "[ESKF] rejected GPS velocity: NIS=%.2f",
                    velocity_nis)
        self.gps_history.append((stamp_sec, gps_position.copy()))
        while (
                self.gps_history
                and stamp_sec - self.gps_history[0][0] > 2.0):
            self.gps_history.popleft()
        self.last_gps_accept_time = stamp_sec
        self.gps_reacquisition_active = False
        self.gps_reacquisition_buffer.clear()
        self.publish(stamp)

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
        synchronized_position = base_position - base_velocity * lag
        synchronized_yaw = wrap_angle(
            self.filter.x[self.filter.YAW]
            - self.filter.last_corrected_yaw_rate * lag)
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
        global_position += base_velocity * lag
        distance = self.slam_alignment.distance_from_anchor(local_position)
        position_variance = self.slam_correlation_inflation * (
            self.filter.slam_position_variance
            + (self.slam_position_drift_per_meter * distance)**2
            + (np.linalg.norm(base_velocity) * lag)**2)
        if measurement_degenerate:
            position_variance *= self.slam_degenerate_variance_scale
        accepted, nis = self.filter.update_slam_position(
            global_position,
            position_variance)
        if accepted:
            self.last_slam_accept_time = stamp_sec
            if measurement_degenerate:
                self.filter.counters["slam_degenerate_accepted"] += 1
            else:
                self._remember_trusted_slam(
                    local_position, local_yaw, stamp_sec,
                    synchronized_position, synchronized_yaw)
        else:
            rospy.logwarn_throttle(
                1.0, "[ESKF] rejected SLAM pose: NIS=%.2f", nis)

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
        dt = min(dt, self.max_predict_dt)
        self.filter.predict(imu_input[0:2], imu_input[2], dt)

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
        _, base_velocity = self.filter.base_state()
        speed = float(np.linalg.norm(base_velocity))
        if (
                speed >= self.nhc_min_speed
                and (
                    self.last_nhc_stamp is None
                    or now_sec - self.last_nhc_stamp >= 1.0 / self.nhc_rate_hz)):
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
        if slam_stationary_hint:
            stationary = True
        if stationary:
            if self.zupt_candidate_since is None:
                self.zupt_candidate_since = now_sec
            elif now_sec - self.zupt_candidate_since >= self.zupt_hold:
                self.filter.update_zero_velocity()
        else:
            self.zupt_candidate_since = None
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

    def publish_diagnostics(self, event):
        now_sec = event.current_real.to_sec()
        gps_age = (
            None if self.last_gps_accept_time is None
            else max(0.0, now_sec - self.last_gps_accept_time))
        gps_message_age = (
            None if self.last_gps_message_time is None
            else max(0.0, now_sec - self.last_gps_message_time))
        slam_age = (
            None if self.last_slam_accept_time is None
            else max(0.0, now_sec - self.last_slam_accept_time))
        if not self.filter.initialized:
            mode = "initializing"
        elif gps_age is not None and gps_age <= 1.0:
            mode = "tracking"
        elif slam_age is not None and slam_age <= 1.0:
            mode = "slam_aided"
        elif gps_age is None or gps_age > 10.0:
            mode = "dead_reckoning"
        else:
            mode = "dead_reckoning"
        payload = {
            "mode": mode,
            "gps_age_sec": gps_age,
            "gps_message_age_sec": gps_message_age,
            "slam_age_sec": slam_age,
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
        self.diagnostics_pub.publish(
            String(data=json.dumps(payload, sort_keys=True)))


if __name__ == "__main__":
    try:
        ESKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
