#!/usr/bin/env python3

import argparse
import heapq
import json
import math
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..")))
from eskf import (  # noqa: E402
    RobustPlanarESKF,
    SE2Alignment,
    robust_gps_velocity,
    wrap_angle,
)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_eskf_noise import (  # noqa: E402
    DEFAULT_CONFIG,
    HampelInputFilter,
    motion_command,
)


CONFIG = dict(DEFAULT_CONFIG)
CONFIG.update({
    "slam_position_variance": 0.25,
    "slam_yaw_variance": 0.0025,
    "slam_nis_soft": 7.815,
    "slam_nis_hard": 16.266,
    "slam_position_drift_per_meter": 0.02,
    "slam_correlation_inflation": 4.0,
    "slam_degenerate_aiding_enabled": True,
    "slam_degenerate_variance_scale": 36.0,
    "slam_alignment_min_baseline_m": 12.0,
    "slam_continuity_confirm_samples": 10,
    "slam_continuity_fast_recovery_gap_sec": 1.0,
    "slam_stationary_hint_max_age_sec": 6.0,
    "slam_stationary_filter_speed_max_mps": 1.5,
    "slam_stationary_release_accel_mps2": 0.45,
    "slam_stationary_reactivation_delay_sec": 2.0,
})

GPS_DROPOUTS = ((30.0, 75.0), (95.0, 140.0))
SLAM_DROPOUTS = ((52.0, 57.0), (117.0, 122.0))
SLAM_DEGENERATE = ((64.0, 69.0), (129.0, 134.0))

DEFAULT_SCENARIO = {
    "duration": 150.0,
    "dt": 0.02,
    "gps_dropouts": GPS_DROPOUTS,
    "slam_dropouts": SLAM_DROPOUTS,
    "slam_degenerate": SLAM_DEGENERATE,
    "gps_std": 1.5,
    "gps_outlier_probability": 0.03,
    "gps_outlier_min_m": 18.0,
    "gps_outlier_max_m": 35.0,
    "accel_std": 0.25,
    "gyro_std": 0.01,
    "yaw_std": 0.05,
    "imu_spike_probability": 0.005,
    "imu_spike_accel_std": 10.0,
    "imu_spike_gyro_std": 1.0,
    "yaw_outlier_std": 1.0,
    "accel_bias": (0.22, -0.15),
    "gyro_bias": 0.008,
    "local_to_global_yaw": 0.35,
    "local_to_global_translation": (40.0, -25.0),
    "slam_position_noise_std": 0.03,
    "slam_yaw_noise_std": 0.003,
    "slam_drift_std_per_update": 0.004,
    "slam_drift_scale": 0.0008,
    "slam_yaw_drift_std_per_update": 0.00015,
    "slam_degenerate_position_drift_std": 0.025,
    "slam_degenerate_yaw_drift_std": 0.001,
    "slam_outlier_probability": 0.01,
    "slam_outlier_position_std": 15.0,
    "slam_outlier_yaw_std": 0.7,
    "slam_timestamp_drop_probability": 0.0,
    "slam_message_delay_sec": 0.0,
    "slam_message_delay_jitter_std": 0.0,
    "slam_max_message_lag_sec": 0.5,
    "slam_frame_resets": (),
}


def in_intervals(time_sec, intervals):
    return any(start <= time_sec <= end for start, end in intervals)


class FilterRuntime:
    def __init__(self):
        self.filter = RobustPlanarESKF(CONFIG)
        self.gps_history = deque(maxlen=30)
        self.zupt_since = None
        self.last_gps_message = None
        self.last_gps_accept = None
        self.gps_reacquisition_buffer = deque(maxlen=14)
        self.gps_reacquisition_active = False
        self.slam_stationary_hint_time = None
        self.slam_stationary_accel_reference = None
        self.slam_stationary_filtered_accel_delta = 0.0
        self.slam_stationary_release_time = None

    def initialize(self, position, yaw, time_sec):
        self.filter.initialize(position, yaw)
        self.gps_history.append((time_sec, np.asarray(position).copy()))
        self.last_gps_message = time_sec
        self.last_gps_accept = time_sec

    def propagate(self, acceleration, yaw_rate, yaw, time_sec, dt,
                  nhc_period_steps, yaw_variance):
        self.filter.predict(acceleration, yaw_rate, dt)
        self.filter.update_yaw(yaw, yaw_variance)
        _, velocity = self.filter.base_state()
        if (
                int(round(time_sec / dt)) % nhc_period_steps == 0
                and np.linalg.norm(velocity) > 0.5):
            self.filter.update_nonholonomic_constraint(yaw_rate)
        corrected_accel = (
            acceleration
            - self.filter.x[self.filter.BAX:self.filter.BAY + 1])
        slam_stationary_hint = (
            self.slam_stationary_hint_time is not None
            and time_sec - self.slam_stationary_hint_time
            <= CONFIG["slam_stationary_hint_max_age_sec"])
        if slam_stationary_hint:
            release_accel = (
                corrected_accel[0]
                if self.slam_stationary_accel_reference is None
                else acceleration[0]
                - self.slam_stationary_accel_reference)
            self.slam_stationary_filtered_accel_delta = (
                0.8 * self.slam_stationary_filtered_accel_delta
                + 0.2 * release_accel)
            if (
                    abs(self.slam_stationary_filtered_accel_delta)
                    >= CONFIG["slam_stationary_release_accel_mps2"]):
                self.slam_stationary_hint_time = None
                self.slam_stationary_accel_reference = None
                self.slam_stationary_filtered_accel_delta = 0.0
                self.slam_stationary_release_time = time_sec
                slam_stationary_hint = False
        stationary = (
            (np.linalg.norm(velocity) < 0.25 or slam_stationary_hint)
            and np.linalg.norm(corrected_accel) < 0.3
            and abs(yaw_rate - self.filter.x[self.filter.BGZ]) < 0.03)
        if slam_stationary_hint:
            stationary = True
        if stationary:
            if self.zupt_since is None:
                self.zupt_since = time_sec
            elif time_sec - self.zupt_since >= 0.8:
                self.filter.update_zero_velocity()
        else:
            self.zupt_since = None

    def update_gps(self, position, time_sec):
        previous_gps_message = self.last_gps_message
        self.last_gps_message = time_sec
        gps_message_gap = (
            math.inf if previous_gps_message is None
            else time_sec - previous_gps_message)
        gps_accept_gap = (
            math.inf if self.last_gps_accept is None
            else time_sec - self.last_gps_accept)
        if (
                not self.gps_reacquisition_active
                and (gps_message_gap >= 1.5 or gps_accept_gap >= 1.5)):
            self.gps_reacquisition_active = True
            self.gps_reacquisition_buffer.clear()
        if self.gps_reacquisition_active:
            self.gps_reacquisition_buffer.append(
                (time_sec, np.asarray(position).copy()))
            if len(self.gps_reacquisition_buffer) >= 7:
                _, base_velocity = self.filter.base_state()
                projected_positions = np.asarray([
                    sample_position
                    + base_velocity * (time_sec - sample_time)
                    for sample_time, sample_position
                    in self.gps_reacquisition_buffer
                ])
                center = np.median(projected_positions, axis=0)
                residuals = np.linalg.norm(
                    projected_positions - center, axis=1)
                inliers = projected_positions[residuals <= 6.0]
                if len(inliers) >= 4:
                    consensus_position = np.mean(inliers, axis=0)
                    self.filter.reanchor_gps_position(consensus_position)
                    self.gps_history.clear()
                    self.gps_history.append(
                        (time_sec, consensus_position.copy()))
                    self.last_gps_accept = time_sec
                    self.gps_reacquisition_active = False
                    self.gps_reacquisition_buffer.clear()
                    return True
        accepted, _ = self.filter.update_gps_position(position)
        if not accepted:
            return False
        self.gps_reacquisition_active = False
        self.gps_reacquisition_buffer.clear()
        velocity_samples = [
            (history_time, history_position)
            for history_time, history_position in self.gps_history
            if 0.0 <= time_sec - history_time <= 2.0
        ]
        velocity_samples.append(
            (time_sec, np.asarray(position).copy()))
        velocity_estimate = robust_gps_velocity(velocity_samples)
        if velocity_estimate is not None:
            velocity, denominator, residual_variance = velocity_estimate
            effective_position_variance = max(
                CONFIG["gps_position_variance"]
                * self.filter.measurement_scales["gps_position"],
                residual_variance)
            variance = (
                effective_position_variance / denominator
                + CONFIG["gps_velocity_variance_floor"])
            self.filter.update_gps_velocity(velocity, variance)
        self.gps_history.append((time_sec, np.asarray(position).copy()))
        while (
                self.gps_history
                and time_sec - self.gps_history[0][0] > 2.0):
            self.gps_history.popleft()
        self.last_gps_accept = time_sec
        return True


def run_trial(seed, scenario=None, record_trace=False):
    scenario_values = dict(DEFAULT_SCENARIO)
    if scenario:
        scenario_values.update(scenario)
    rng = np.random.default_rng(seed)
    dt = float(scenario_values["dt"])
    duration = float(scenario_values["duration"])
    gps_period = int(round(0.1 / dt))
    slam_period = int(round(0.1 / dt))
    nhc_period = max(1, int(round(0.1 / dt)))
    no_slam = FilterRuntime()
    slam_aided = FilterRuntime()
    input_filter = HampelInputFilter()
    initialization_buffer = deque(maxlen=7)
    alignment = SE2Alignment(
        alpha=0.05,
        reset_distance=5.0,
        reset_yaw=math.radians(20.0),
        reset_confirm_samples=3,
        min_baseline=CONFIG["slam_alignment_min_baseline_m"],
        continuity_confirm_samples=CONFIG[
            "slam_continuity_confirm_samples"])

    true_position = np.zeros(2, dtype=float)
    true_yaw = 0.0
    accel_bias = np.asarray(scenario_values["accel_bias"], dtype=float)
    gyro_bias = float(scenario_values["gyro_bias"])
    active_local_to_global_yaw = float(
        scenario_values["local_to_global_yaw"])
    active_local_to_global_translation = np.asarray(
        scenario_values["local_to_global_translation"], dtype=float)
    lidar_lever = np.array([1.045, 0.000])
    slam_position_drift = np.zeros(2, dtype=float)
    slam_yaw_drift = 0.0
    previous_slam_true_position = true_position.copy()
    slam_queue = []
    slam_sequence = 0
    frame_resets = sorted(
        scenario_values["slam_frame_resets"],
        key=lambda event: event["time_sec"])
    next_frame_reset = 0
    no_slam_errors = []
    slam_errors = []
    outage_no_slam_errors = []
    outage_slam_errors = []
    slam_outlier_rejections = 0
    last_slam_accept = None
    state_finite = True
    covariance_min_eigenvalue = math.inf
    latest_slam_measurement_error = None
    last_local_slam_position = None
    last_local_slam_yaw = None
    last_local_slam_stamp = None
    trusted_local_slam_position = None
    trusted_local_slam_yaw = None
    trusted_local_slam_stamp = None
    trusted_filter_position = None
    trusted_filter_yaw = None
    local_frame_jump_count = 0
    slam_motion_history = deque(maxlen=30)
    trace = []

    for step in range(int(duration / dt)):
        time_sec = step * dt
        speed, longitudinal_accel, yaw_rate = motion_command(time_sec)
        true_yaw = wrap_angle(true_yaw + yaw_rate * dt)
        true_position += np.array([
            math.cos(true_yaw) * speed,
            math.sin(true_yaw) * speed,
        ]) * dt
        true_acceleration = np.array([
            longitudinal_accel,
            speed * yaw_rate,
        ])
        measured_acceleration = (
            true_acceleration + accel_bias
            + rng.normal(0.0, scenario_values["accel_std"], 2))
        measured_yaw_rate = (
            yaw_rate + gyro_bias
            + rng.normal(0.0, scenario_values["gyro_std"]))
        if rng.random() < scenario_values["imu_spike_probability"]:
            measured_acceleration += rng.normal(
                0.0, scenario_values["imu_spike_accel_std"], 2)
            measured_yaw_rate += rng.normal(
                0.0, scenario_values["imu_spike_gyro_std"])
        filtered_input = input_filter.apply([
            measured_acceleration[0],
            measured_acceleration[1],
            measured_yaw_rate,
        ])
        measured_yaw = wrap_angle(
            true_yaw + rng.normal(0.0, scenario_values["yaw_std"]))
        if rng.random() < scenario_values["imu_spike_probability"]:
            measured_yaw = wrap_angle(
                measured_yaw
                + rng.normal(0.0, scenario_values["yaw_outlier_std"]))

        gps_position = None
        if (
                step % gps_period == 0
                and not in_intervals(
                    time_sec, scenario_values["gps_dropouts"])):
            gps_position = true_position + rng.normal(
                0.0, scenario_values["gps_std"], 2)
            if rng.random() < scenario_values["gps_outlier_probability"]:
                direction = rng.uniform(-math.pi, math.pi)
                gps_position += rng.uniform(
                    scenario_values["gps_outlier_min_m"],
                    scenario_values["gps_outlier_max_m"]) * np.array([
                    math.cos(direction), math.sin(direction)])

        if not slam_aided.filter.initialized:
            if gps_position is None:
                continue
            initialization_buffer.append(gps_position.copy())
            samples = np.asarray(initialization_buffer)
            center = np.median(samples, axis=0)
            inliers = samples[
                np.linalg.norm(samples - center, axis=1) <= 6.0]
            if len(inliers) < 4:
                continue
            initial_position = np.mean(inliers, axis=0)
            no_slam.initialize(initial_position, measured_yaw, time_sec)
            slam_aided.initialize(initial_position, measured_yaw, time_sec)
        else:
            for runtime in (no_slam, slam_aided):
                runtime.propagate(
                    filtered_input[0:2],
                    filtered_input[2],
                    measured_yaw,
                    time_sec,
                    dt,
                    nhc_period,
                    scenario_values["yaw_std"]**2)
                if gps_position is not None:
                    runtime.update_gps(gps_position, time_sec)

        if step % slam_period == 0 and slam_aided.filter.initialized:
            while (
                    next_frame_reset < len(frame_resets)
                    and frame_resets[next_frame_reset]["time_sec"]
                    <= time_sec):
                event = frame_resets[next_frame_reset]
                active_local_to_global_yaw = wrap_angle(
                    active_local_to_global_yaw
                    + float(event["yaw_delta"]))
                active_local_to_global_translation = (
                    active_local_to_global_translation
                    + np.asarray(event["translation_delta"], dtype=float))
                next_frame_reset += 1

            global_to_local_rotation = RobustPlanarESKF._rotation(
                -active_local_to_global_yaw)
            local_delta = global_to_local_rotation @ (
                true_position - previous_slam_true_position)
            slam_position_drift += (
                scenario_values["slam_drift_scale"] * local_delta
                + rng.normal(
                    0.0,
                    scenario_values["slam_drift_std_per_update"],
                    2))
            slam_yaw_drift += rng.normal(
                0.0,
                scenario_values["slam_yaw_drift_std_per_update"])
            if in_intervals(
                    time_sec, scenario_values["slam_degenerate"]):
                slam_position_drift += rng.normal(
                    0.0,
                    scenario_values[
                        "slam_degenerate_position_drift_std"],
                    2)
                slam_yaw_drift += rng.normal(
                    0.0,
                    scenario_values["slam_degenerate_yaw_drift_std"])

            local_base_position = (
                global_to_local_rotation
                @ (true_position - active_local_to_global_translation)
                + slam_position_drift
                + rng.normal(
                    0.0,
                    scenario_values["slam_position_noise_std"],
                    2))
            local_yaw = wrap_angle(
                true_yaw - active_local_to_global_yaw
                + slam_yaw_drift
                + rng.normal(
                    0.0, scenario_values["slam_yaw_noise_std"]))
            local_lidar_position = (
                local_base_position
                + RobustPlanarESKF._rotation(local_yaw) @ lidar_lever)
            is_outlier = (
                rng.random()
                < scenario_values["slam_outlier_probability"])
            if is_outlier:
                local_lidar_position += rng.normal(
                    0.0,
                    scenario_values["slam_outlier_position_std"],
                    2)
                local_yaw = wrap_angle(
                    local_yaw
                    + rng.normal(
                        0.0,
                        scenario_values["slam_outlier_yaw_std"]))
            local_position = (
                local_lidar_position
                - RobustPlanarESKF._rotation(local_yaw) @ lidar_lever)
            previous_slam_true_position = true_position.copy()

            message_available = (
                not in_intervals(
                    time_sec, scenario_values["slam_dropouts"])
                and rng.random() >= scenario_values[
                    "slam_timestamp_drop_probability"])
            if message_available:
                delay = max(
                    0.0,
                    float(scenario_values["slam_message_delay_sec"])
                    + rng.normal(
                        0.0,
                        scenario_values["slam_message_delay_jitter_std"]))
                measurement = {
                    "stamp": time_sec,
                    "local_position": local_position.copy(),
                    "local_yaw": local_yaw,
                    "degenerate": in_intervals(
                        time_sec, scenario_values["slam_degenerate"]),
                    "is_outlier": is_outlier,
                }
                heapq.heappush(
                    slam_queue,
                    (time_sec + delay, slam_sequence, measurement))
                slam_sequence += 1

        while slam_queue and slam_queue[0][0] <= time_sec:
            _, _, measurement = heapq.heappop(slam_queue)
            measurement_stamp = measurement["stamp"]
            lag = max(0.0, time_sec - measurement_stamp)
            if lag > scenario_values["slam_max_message_lag_sec"]:
                continue
            if (
                    measurement["degenerate"]
                    and not CONFIG["slam_degenerate_aiding_enabled"]):
                continue

            gps_age = (
                math.inf if slam_aided.last_gps_accept is None
                else max(
                    0.0,
                    measurement_stamp - slam_aided.last_gps_accept))
            gps_message_age = (
                math.inf if slam_aided.last_gps_message is None
                else max(
                    0.0,
                    measurement_stamp - slam_aided.last_gps_message))
            base_position, base_velocity = slam_aided.filter.base_state()
            synchronized_position = base_position - base_velocity * lag
            synchronized_yaw = wrap_angle(
                slam_aided.filter.x[slam_aided.filter.YAW]
                - slam_aided.filter.last_corrected_yaw_rate * lag)
            local_position = measurement["local_position"]
            local_yaw = measurement["local_yaw"]
            previous_local_position = last_local_slam_position
            previous_local_yaw = last_local_slam_yaw
            previous_local_stamp = last_local_slam_stamp
            local_dt = None
            local_frame_jump = False
            if (
                    previous_local_position is not None
                    and previous_local_stamp is not None):
                local_dt = measurement_stamp - previous_local_stamp
                if local_dt > 0.0:
                    position_step = float(np.linalg.norm(
                        local_position - previous_local_position))
                    yaw_step = abs(wrap_angle(
                        local_yaw - previous_local_yaw))
                    position_limit = max(
                        3.0,
                        (np.linalg.norm(base_velocity) + 5.0) * local_dt)
                    yaw_limit = max(
                        math.radians(20.0),
                        (
                            abs(slam_aided.filter.last_corrected_yaw_rate)
                            + 0.5)
                        * local_dt)
                    local_frame_jump = (
                        position_step > position_limit
                        or yaw_step > yaw_limit)
                    if local_frame_jump:
                        local_frame_jump_count += 1
            if (
                    last_local_slam_stamp is None
                    or measurement_stamp > last_local_slam_stamp):
                last_local_slam_position = local_position.copy()
                last_local_slam_yaw = local_yaw
                last_local_slam_stamp = measurement_stamp
            if not measurement["degenerate"]:
                if local_frame_jump:
                    slam_motion_history.clear()
                    slam_aided.slam_stationary_hint_time = None
                    slam_aided.slam_stationary_accel_reference = None
                slam_motion_history.append(
                    (measurement_stamp, local_position.copy()))
                while (
                        slam_motion_history
                        and measurement_stamp
                        - slam_motion_history[0][0] > 1.2):
                    slam_motion_history.popleft()
                motion_estimate = robust_gps_velocity(
                    slam_motion_history, min_span=0.6)
                if motion_estimate is not None:
                    slam_velocity, _, residual_variance = motion_estimate
                    slam_speed = float(np.linalg.norm(slam_velocity))
                    _, filter_velocity = slam_aided.filter.base_state()
                    if (
                            slam_speed <= 0.3
                            and np.linalg.norm(filter_velocity)
                            <= CONFIG[
                                "slam_stationary_filter_speed_max_mps"]
                            and residual_variance <= 0.25):
                        reactivation_blocked = (
                            slam_aided.slam_stationary_release_time
                            is not None
                            and measurement_stamp
                            - slam_aided.slam_stationary_release_time
                            < CONFIG[
                                "slam_stationary_reactivation_delay_sec"])
                        if not reactivation_blocked:
                            hint_is_new = (
                                slam_aided.slam_stationary_hint_time is None
                                or measurement_stamp
                                - slam_aided.slam_stationary_hint_time
                                > CONFIG[
                                    "slam_stationary_hint_max_age_sec"])
                            if hint_is_new:
                                accel_samples = list(
                                    input_filter.windows[0])
                                slam_aided.slam_stationary_accel_reference = (
                                    float(np.median(accel_samples))
                                    if accel_samples
                                    else float(filtered_input[0]))
                                slam_aided.slam_stationary_filtered_accel_delta = 0.0
                            slam_aided.slam_stationary_hint_time = (
                                measurement_stamp)
                    elif slam_speed > 0.6:
                        slam_aided.slam_stationary_hint_time = None
                        slam_aided.slam_stationary_accel_reference = None
                        slam_aided.slam_stationary_filtered_accel_delta = 0.0
            if gps_age <= 0.75:
                if not measurement["degenerate"]:
                    alignment_updated = alignment.update(
                        synchronized_position,
                        synchronized_yaw,
                        local_position,
                        local_yaw,
                        measurement_stamp)
                    if alignment_updated and alignment.ready:
                        trusted_local_slam_position = local_position.copy()
                        trusted_local_slam_yaw = local_yaw
                        trusted_local_slam_stamp = measurement_stamp
                        trusted_filter_position = synchronized_position.copy()
                        trusted_filter_yaw = synchronized_yaw
            elif (
                    measurement["degenerate"]
                    and (
                        local_frame_jump
                        or alignment.continuity_pending
                        or alignment.pending_count > 0
                        or alignment.recovering)):
                pass
            elif alignment.continuity_pending:
                old_global_position, old_global_yaw = alignment.transform(
                    local_position, local_yaw)
                old_transform_matches = (
                    np.linalg.norm(
                        old_global_position - synchronized_position) <= 15.0
                    and abs(wrap_angle(
                        old_global_yaw - synchronized_yaw))
                    <= math.radians(20.0))
                if old_transform_matches:
                    alignment.cancel_continuity_recovery()
                elif (
                        local_frame_jump
                        and trusted_local_slam_position is not None
                        and trusted_filter_position is not None
                        and trusted_filter_yaw is not None):
                    trusted_global_position, trusted_global_yaw = (
                        alignment.transform(
                            trusted_local_slam_position,
                            trusted_local_slam_yaw))
                    continuity_position = (
                        trusted_global_position
                        + synchronized_position - trusted_filter_position)
                    continuity_yaw = wrap_angle(
                        trusted_global_yaw
                        + wrap_angle(
                            synchronized_yaw - trusted_filter_yaw))
                    alignment.recover_continuity(
                        continuity_position,
                        continuity_yaw,
                        local_position,
                        local_yaw,
                        measurement_stamp,
                        start=True,
                        confirm_samples=(
                            3
                            if (
                                local_dt is not None
                                and local_dt >= CONFIG[
                                    "slam_continuity_fast_recovery_gap_sec"])
                            else CONFIG[
                                "slam_continuity_confirm_samples"]))
                else:
                    recovered = alignment.recover_continuity(
                        synchronized_position,
                        synchronized_yaw,
                        local_position,
                        local_yaw,
                        measurement_stamp)
                    if recovered:
                        trusted_local_slam_position = local_position.copy()
                        trusted_local_slam_yaw = local_yaw
                        trusted_local_slam_stamp = measurement_stamp
                        trusted_filter_position = synchronized_position.copy()
                        trusted_filter_yaw = synchronized_yaw
            elif (
                    local_frame_jump
                    and trusted_local_slam_position is not None
                    and trusted_filter_position is not None
                    and trusted_filter_yaw is not None):
                trusted_global_position, trusted_global_yaw = (
                    alignment.transform(
                        trusted_local_slam_position,
                        trusted_local_slam_yaw))
                continuity_position = (
                    trusted_global_position
                    + synchronized_position - trusted_filter_position)
                continuity_yaw = wrap_angle(
                    trusted_global_yaw
                    + wrap_angle(synchronized_yaw - trusted_filter_yaw))
                alignment.recover_continuity(
                    continuity_position,
                    continuity_yaw,
                    local_position,
                    local_yaw,
                    measurement_stamp,
                    start=True,
                    confirm_samples=(
                        3
                        if (
                            local_dt is not None
                            and local_dt >= CONFIG[
                                "slam_continuity_fast_recovery_gap_sec"])
                        else CONFIG[
                            "slam_continuity_confirm_samples"]))
            elif alignment.pending_count > 0:
                alignment.update(
                    synchronized_position,
                    synchronized_yaw,
                    local_position,
                    local_yaw,
                    measurement_stamp)
            elif alignment.recovering:
                alignment.update(
                    synchronized_position,
                    synchronized_yaw,
                    local_position,
                    local_yaw,
                    measurement_stamp)
            elif (
                    gps_age >= 1.0
                    and gps_message_age >= 1.0
                    and alignment.initialized
                    and alignment.ready
                    and measurement_stamp - alignment.anchor_stamp <= 120.0):
                global_position, _ = alignment.transform(
                    local_position, local_yaw)
                global_position += base_velocity * lag
                distance = alignment.distance_from_anchor(local_position)
                position_variance = CONFIG[
                    "slam_correlation_inflation"] * (
                    CONFIG["slam_position_variance"]
                    + (
                        CONFIG["slam_position_drift_per_meter"]
                        * distance)**2
                    + (np.linalg.norm(base_velocity) * lag)**2)
                if measurement["degenerate"]:
                    position_variance *= CONFIG[
                        "slam_degenerate_variance_scale"]
                accepted, _ = slam_aided.filter.update_slam_position(
                    global_position,
                    position_variance)
                latest_slam_measurement_error = float(np.linalg.norm(
                    global_position - true_position))
                if accepted:
                    last_slam_accept = measurement_stamp
                    if not measurement["degenerate"]:
                        trusted_local_slam_position = local_position.copy()
                        trusted_local_slam_yaw = local_yaw
                        trusted_local_slam_stamp = measurement_stamp
                        trusted_filter_position = synchronized_position.copy()
                        trusted_filter_yaw = synchronized_yaw
                elif measurement["is_outlier"]:
                    slam_outlier_rejections += 1

        if slam_aided.filter.initialized and time_sec >= 2.0:
            covariance = slam_aided.filter.p
            covariance_min_eigenvalue = min(
                covariance_min_eigenvalue,
                float(np.min(np.linalg.eigvalsh(covariance))))
            if not (
                    np.all(np.isfinite(slam_aided.filter.x))
                    and np.all(np.isfinite(covariance))):
                state_finite = False
                break
            no_slam_position, _ = no_slam.filter.base_state()
            slam_position, _ = slam_aided.filter.base_state()
            no_slam_error = float(np.linalg.norm(
                no_slam_position - true_position))
            slam_error = float(np.linalg.norm(
                slam_position - true_position))
            no_slam_errors.append(no_slam_error)
            slam_errors.append(slam_error)
            if in_intervals(
                    time_sec, scenario_values["gps_dropouts"]):
                outage_no_slam_errors.append(no_slam_error)
                outage_slam_errors.append(slam_error)
            if (
                    record_trace
                    and step % max(1, int(round(1.0 / dt))) == 0):
                gps_age = (
                    None if slam_aided.last_gps_accept is None
                    else time_sec - slam_aided.last_gps_accept)
                trace.append({
                    "time_sec": time_sec,
                    "gps_age_sec": gps_age,
                    "no_slam_error_m": no_slam_error,
                    "slam_error_m": slam_error,
                    "slam_measurement_error_m":
                        latest_slam_measurement_error,
                    "slam_position_drift_m": float(np.linalg.norm(
                        slam_position_drift)),
                    "slam_yaw_drift_deg": math.degrees(slam_yaw_drift),
                    "alignment_ready": alignment.ready,
                    "alignment_yaw_error_deg": math.degrees(wrap_angle(
                        alignment.yaw - active_local_to_global_yaw)),
                    "alignment_orientation_yaw_error_deg": (
                        None
                        if alignment.orientation_yaw_estimate is None
                        else math.degrees(wrap_angle(
                            alignment.orientation_yaw_estimate
                            - active_local_to_global_yaw))),
                    "alignment_resets": alignment.reset_count,
                    "slam_accepted":
                        slam_aided.filter.counters["slam_accepted"],
                    "slam_rejected":
                        slam_aided.filter.counters["slam_rejected"],
                    "slam_stationary_hint": (
                        slam_aided.slam_stationary_hint_time is not None
                        and time_sec
                        - slam_aided.slam_stationary_hint_time
                        <= CONFIG["slam_stationary_hint_max_age_sec"]),
                    "slam_stationary_accel_reference":
                        slam_aided.slam_stationary_accel_reference,
                    "slam_stationary_filtered_accel_delta":
                        slam_aided.slam_stationary_filtered_accel_delta,
                    "slam_stationary_release_time":
                        slam_aided.slam_stationary_release_time,
                    "estimated_speed_mps": float(np.linalg.norm(
                        slam_aided.filter.base_state()[1])),
                    "true_speed_mps": speed,
                    "position_std_m": math.sqrt(max(
                        slam_aided.filter.p[
                            slam_aided.filter.PX,
                            slam_aided.filter.PX],
                        0.0)),
                })

    if not slam_errors:
        return {
            "initialized": False,
            "state_finite": False,
            "covariance_min_eigenvalue": -math.inf,
        }
    result = {
        "initialized": True,
        "state_finite": state_finite,
        "covariance_min_eigenvalue": covariance_min_eigenvalue,
        "no_slam_rmse_m": float(np.sqrt(np.mean(np.square(no_slam_errors)))),
        "slam_rmse_m": float(np.sqrt(np.mean(np.square(slam_errors)))),
        "no_slam_outage_p95_m": float(np.percentile(
            outage_no_slam_errors, 95)),
        "slam_outage_p95_m": float(np.percentile(outage_slam_errors, 95)),
        "slam_max_m": float(np.max(slam_errors)),
        "slam_updates": slam_aided.filter.counters["slam_accepted"],
        "slam_rejected": slam_aided.filter.counters["slam_rejected"],
        "slam_outlier_rejections": slam_outlier_rejections,
        "alignment_resets": alignment.reset_count,
        "alignment_yaw_error_deg": math.degrees(wrap_angle(
            alignment.yaw - active_local_to_global_yaw)),
        "slam_frame_jumps": local_frame_jump_count,
        "last_slam_accept_sec": last_slam_accept,
        "gps_accepted": slam_aided.filter.counters["gps_accepted"],
        "gps_rejected": slam_aided.filter.counters["gps_rejected"],
        "gps_reacquired": slam_aided.filter.counters["gps_reacquired"],
        "yaw_rejected": slam_aided.filter.counters["yaw_rejected"],
    }
    if record_trace:
        result["trace"] = trace
    return result


def aggregate(trials):
    return {
        key: float(np.mean([trial[key] for trial in trials]))
        for key in trials[0]
        if isinstance(trials[0][key], (int, float))
    }


def main():
    parser = argparse.ArgumentParser(
        description="GPS-denied LIO-SAM-aided ESKF Monte Carlo test")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20260728)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    trials = [
        run_trial(args.seed + trial_index)
        for trial_index in range(args.trials)
    ]
    result = aggregate(trials)
    passed = (
        result["slam_rmse_m"] < result["no_slam_rmse_m"] * 0.6
        and result["slam_outage_p95_m"] < 4.0
        and result["slam_max_m"] < 8.0)
    output = {"passed": passed, "result": result}
    if args.json:
        print(json.dumps(output, indent=2, sort_keys=True))
    else:
        print(
            "no SLAM RMSE={:.3f}m, SLAM-aided RMSE={:.3f}m, "
            "outage p95={:.3f}m, max={:.3f}m".format(
                result["no_slam_rmse_m"],
                result["slam_rmse_m"],
                result["slam_outage_p95_m"],
                result["slam_max_m"]))
        print("PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
