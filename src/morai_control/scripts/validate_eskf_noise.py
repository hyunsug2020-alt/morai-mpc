#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eskf_node import RobustPlanarESKF, wrap_angle  # noqa: E402


DEFAULT_CONFIG = {
    "accel_noise_std": 0.35,
    "gyro_noise_std": 0.015,
    "accel_bias_random_walk_std": 0.02,
    "gyro_bias_random_walk_std": 0.002,
    "gps_position_variance": 1.0,
    "gps_velocity_variance_floor": 0.5,
    "imu_yaw_variance": 0.02,
    "nhc_variance": 0.09,
    "zupt_velocity_variance": 0.01,
    "gps_nis_soft": 5.991,
    "gps_nis_hard": 13.816,
    "velocity_nis_soft": 5.991,
    "velocity_nis_hard": 13.816,
    "yaw_nis_soft": 3.841,
    "yaw_nis_hard": 10.828,
    "constraint_nis_hard": 10.828,
    "adaptive_alpha": 0.04,
    "adaptive_scale_min": 0.5,
    "adaptive_scale_max": 16.0,
}

SCENARIOS = {
    "gaussian": {
        "gps_std": 1.2,
        "accel_std": 0.20,
        "gyro_std": 0.008,
        "yaw_std": 0.04,
        "gps_outlier_probability": 0.0,
        "imu_spike_probability": 0.0,
        "dropouts": [],
    },
    "bias": {
        "gps_std": 1.2,
        "accel_std": 0.20,
        "gyro_std": 0.008,
        "yaw_std": 0.04,
        "gps_outlier_probability": 0.0,
        "imu_spike_probability": 0.0,
        "dropouts": [],
        "accel_bias": [0.18, -0.12],
        "gyro_bias": 0.006,
    },
    "outlier": {
        "gps_std": 1.2,
        "accel_std": 0.20,
        "gyro_std": 0.008,
        "yaw_std": 0.04,
        "gps_outlier_probability": 0.025,
        "imu_spike_probability": 0.003,
        "dropouts": [],
        "accel_bias": [0.18, -0.12],
        "gyro_bias": 0.006,
    },
    "combined": {
        "gps_std": 1.5,
        "accel_std": 0.25,
        "gyro_std": 0.01,
        "yaw_std": 0.05,
        "gps_outlier_probability": 0.03,
        "imu_spike_probability": 0.005,
        "dropouts": [(35.0, 50.0), (94.0, 104.0)],
        "accel_bias": [0.22, -0.15],
        "gyro_bias": 0.008,
    },
}


class LegacyFiveStateFilter:
    """Small reproduction of the former ungated, bias-free implementation."""

    def __init__(self):
        self.x = np.zeros(5, dtype=float)
        self.p = np.diag([4.0, 4.0, 9.0, 9.0, 0.5])
        self.initialized = False

    def initialize(self, position, yaw):
        self.x[0:2] = position
        self.x[4] = yaw
        self.initialized = True

    def predict(self, acceleration, yaw_rate, dt):
        yaw = self.x[4]
        rotation = np.array([
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)],
        ])
        acceleration_map = rotation @ acceleration
        self.x[0:2] += self.x[2:4] * dt + 0.5 * acceleration_map * dt**2
        self.x[2:4] += acceleration_map * dt
        self.x[4] = wrap_angle(self.x[4] + yaw_rate * dt)
        f = np.eye(5)
        f[0, 2] = dt
        f[1, 3] = dt
        self.p = f @ self.p @ f.T + np.diag([
            0.25 * 0.8 * dt**4,
            0.25 * 0.8 * dt**4,
            0.8 * dt**2,
            0.8 * dt**2,
            0.03 * dt**2,
        ])

    def _update(self, innovation, h, r):
        s = h @ self.p @ h.T + r
        k = np.linalg.solve(s.T, (self.p @ h.T).T).T
        self.x += k @ innovation
        self.x[4] = wrap_angle(self.x[4])
        identity = np.eye(5)
        ikh = identity - k @ h
        self.p = ikh @ self.p @ ikh.T + k @ r @ k.T

    def update_gps(self, position):
        h = np.zeros((2, 5), dtype=float)
        h[:, 0:2] = np.eye(2)
        self._update(position - self.x[0:2], h, np.eye(2) * 2.0)

    def update_yaw(self, yaw):
        h = np.zeros((1, 5), dtype=float)
        h[0, 4] = 1.0
        self._update(
            np.array([wrap_angle(yaw - self.x[4])]),
            h,
            np.array([[0.02]]))


class HampelInputFilter:
    def __init__(self):
        self.windows = [deque(maxlen=11) for _ in range(3)]
        self.spike_count = 0

    def apply(self, values):
        result = []
        floors = (4.0, 4.0, 0.35)
        limits = (20.0, 20.0, 2.0)
        for index, raw_value in enumerate(values):
            value = float(raw_value)
            window = self.windows[index]
            if len(window) >= 5:
                samples = np.asarray(window, dtype=float)
                median = float(np.median(samples))
                mad = float(np.median(np.abs(samples - median)))
                threshold = max(floors[index], 6.0 * 1.4826 * mad)
                if abs(value - median) > threshold:
                    value = median + math.copysign(
                        threshold, value - median)
                    self.spike_count += 1
            clipped = float(np.clip(value, -limits[index], limits[index]))
            if clipped != value:
                self.spike_count += 1
            window.append(clipped)
            result.append(clipped)
        return np.asarray(result)


def motion_command(time_sec):
    if time_sec < 5.0:
        return 0.0, 0.0, 0.0
    if time_sec < 10.0:
        return 1.2 * (time_sec - 5.0), 1.2, 0.0
    if time_sec < 35.0:
        return 6.0, 0.0, 0.0
    if time_sec < 55.0:
        return 6.0, 0.0, 0.035
    if time_sec < 70.0:
        return 6.0, 0.0, 0.0
    if time_sec < 75.0:
        return 6.0 - 1.2 * (time_sec - 70.0), -1.2, 0.0
    if time_sec < 82.0:
        return 0.0, 0.0, 0.0
    if time_sec < 87.0:
        return 1.2 * (time_sec - 82.0), 1.2, 0.0
    if time_sec < 95.0:
        return 6.0, 0.0, 0.0
    if time_sec < 115.0:
        return 6.0, 0.0, -0.025
    return 6.0, 0.0, 0.0


def in_dropout(time_sec, intervals):
    return any(start <= time_sec <= end for start, end in intervals)


def run_trial(seed, scenario):
    rng = np.random.default_rng(seed)
    dt = 0.02
    duration = 120.0
    gps_period_steps = int(round(0.1 / dt))
    robust = RobustPlanarESKF(DEFAULT_CONFIG)
    legacy = LegacyFiveStateFilter()
    input_filter = HampelInputFilter()
    accel_bias = np.asarray(
        scenario.get("accel_bias", [0.0, 0.0]), dtype=float)
    gyro_bias = float(scenario.get("gyro_bias", 0.0))
    true_position = np.zeros(2, dtype=float)
    true_yaw = 0.0
    gps_history = deque(maxlen=30)
    initialization_buffer = deque(maxlen=7)
    robust_errors = []
    legacy_errors = []
    robust_yaw_errors = []
    legacy_yaw_errors = []
    zupt_since = None

    steps = int(duration / dt)
    for step in range(steps):
        time_sec = step * dt
        speed, longitudinal_accel, yaw_rate = motion_command(time_sec)
        true_yaw = wrap_angle(true_yaw + yaw_rate * dt)
        true_position += np.array([
            math.cos(true_yaw) * speed,
            math.sin(true_yaw) * speed,
        ]) * dt
        true_accel_body = np.array([
            longitudinal_accel,
            speed * yaw_rate,
        ])

        measured_accel = (
            true_accel_body
            + accel_bias
            + rng.normal(0.0, scenario["accel_std"], 2))
        measured_gyro = (
            yaw_rate + gyro_bias
            + rng.normal(0.0, scenario["gyro_std"]))
        if rng.random() < scenario["imu_spike_probability"]:
            measured_accel += rng.normal(0.0, 10.0, 2)
            measured_gyro += rng.normal(0.0, 1.0)
        filtered_input = input_filter.apply(
            [measured_accel[0], measured_accel[1], measured_gyro])

        measured_yaw = wrap_angle(
            true_yaw + rng.normal(0.0, scenario["yaw_std"]))
        if rng.random() < scenario["imu_spike_probability"]:
            measured_yaw = wrap_angle(
                measured_yaw + rng.normal(0.0, 1.0))

        gps_available = (
            step % gps_period_steps == 0
            and not in_dropout(time_sec, scenario["dropouts"]))
        gps_position = None
        if gps_available:
            gps_position = (
                true_position
                + rng.normal(0.0, scenario["gps_std"], 2))
            if rng.random() < scenario["gps_outlier_probability"]:
                angle = rng.uniform(-math.pi, math.pi)
                magnitude = rng.uniform(18.0, 35.0)
                gps_position += magnitude * np.array([
                    math.cos(angle), math.sin(angle)])

        if not robust.initialized:
            if gps_position is None:
                continue
            if not legacy.initialized:
                legacy.initialize(gps_position, measured_yaw)
            initialization_buffer.append(gps_position.copy())
            initialization_samples = np.asarray(initialization_buffer)
            initialization_center = np.median(
                initialization_samples, axis=0)
            initialization_residuals = np.linalg.norm(
                initialization_samples - initialization_center, axis=1)
            initialization_inliers = initialization_samples[
                initialization_residuals <= 6.0]
            if len(initialization_inliers) < 4:
                continue
            initial_position = np.mean(initialization_inliers, axis=0)
            robust.initialize(initial_position, measured_yaw)
            gps_history.append((time_sec, initial_position.copy()))
        else:
            robust.predict(filtered_input[0:2], filtered_input[2], dt)
            robust.update_yaw(measured_yaw, scenario["yaw_std"]**2)
            legacy.predict(measured_accel, measured_gyro, dt)
            legacy.update_yaw(measured_yaw)

            if step % 5 == 0 and speed >= 0.5:
                robust.update_nonholonomic_constraint(filtered_input[2])

            corrected_accel = (
                filtered_input[0:2]
                - robust.x[robust.BAX:robust.BAY + 1])
            if (
                    np.linalg.norm(robust.x[robust.VX:robust.VY + 1]) < 0.25
                    and np.linalg.norm(corrected_accel) < 0.3
                    and abs(filtered_input[2] - robust.x[robust.BGZ]) < 0.03):
                if zupt_since is None:
                    zupt_since = time_sec
                elif time_sec - zupt_since >= 0.8:
                    robust.update_zero_velocity()
            else:
                zupt_since = None

            if gps_position is not None:
                accepted, _ = robust.update_gps_position(gps_position)
                legacy.update_gps(gps_position)
                if accepted:
                    reference = None
                    for history_time, history_position in gps_history:
                        if time_sec - history_time >= 0.6:
                            reference = history_time, history_position
                            break
                    if reference is not None:
                        history_time, history_position = reference
                        gps_dt = time_sec - history_time
                        if gps_dt <= 2.0:
                            gps_velocity = (
                                gps_position - history_position) / gps_dt
                            velocity_variance = (
                                2.0 * DEFAULT_CONFIG[
                                    "gps_position_variance"] / gps_dt**2
                                + DEFAULT_CONFIG[
                                    "gps_velocity_variance_floor"])
                            robust.update_gps_velocity(
                                gps_velocity, velocity_variance)
                    gps_history.append(
                        (time_sec, gps_position.copy()))
                    while (
                            gps_history
                            and time_sec - gps_history[0][0] > 2.0):
                        gps_history.popleft()

        if time_sec >= 2.0 and robust.initialized:
            robust_position, _ = robust.base_state()
            robust_errors.append(
                float(np.linalg.norm(robust_position - true_position)))
            legacy_errors.append(
                float(np.linalg.norm(legacy.x[0:2] - true_position)))
            robust_yaw_errors.append(abs(wrap_angle(
                robust.x[robust.YAW] - true_yaw)))
            legacy_yaw_errors.append(abs(wrap_angle(
                legacy.x[4] - true_yaw)))

    return {
        "robust_rmse_m": float(np.sqrt(np.mean(np.square(robust_errors)))),
        "robust_p95_m": float(np.percentile(robust_errors, 95)),
        "robust_max_m": float(np.max(robust_errors)),
        "legacy_rmse_m": float(np.sqrt(np.mean(np.square(legacy_errors)))),
        "legacy_p95_m": float(np.percentile(legacy_errors, 95)),
        "robust_yaw_rmse_deg": math.degrees(float(np.sqrt(
            np.mean(np.square(robust_yaw_errors))))),
        "legacy_yaw_rmse_deg": math.degrees(float(np.sqrt(
            np.mean(np.square(legacy_yaw_errors))))),
        "gps_rejected": robust.counters["gps_rejected"],
        "yaw_rejected": robust.counters["yaw_rejected"],
        "imu_spikes_filtered": input_filter.spike_count,
        "estimated_accel_bias": robust.x[
            robust.BAX:robust.BAY + 1].tolist(),
        "estimated_gyro_bias": float(robust.x[robust.BGZ]),
    }


def aggregate(trials):
    scalar_keys = [
        key for key, value in trials[0].items()
        if isinstance(value, (int, float))
    ]
    return {
        key: float(np.mean([trial[key] for trial in trials]))
        for key in scalar_keys
    }


def main():
    parser = argparse.ArgumentParser(
        description="Monte Carlo noise/outlier/dropout test for MORAI ESKF")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20260728)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    results = {}
    for scenario_index, (name, scenario) in enumerate(SCENARIOS.items()):
        trials = [
            run_trial(
                args.seed + scenario_index * 1000 + trial_index,
                scenario)
            for trial_index in range(args.trials)
        ]
        results[name] = aggregate(trials)

    combined = results["combined"]
    passed = (
        combined["robust_rmse_m"] < 3.0
        and combined["robust_p95_m"] < 6.0
        and combined["robust_rmse_m"] < combined["legacy_rmse_m"] * 0.5)
    if args.json:
        print(json.dumps(
            {"passed": passed, "results": results},
            indent=2,
            sort_keys=True))
    else:
        for name, metrics in results.items():
            print(
                "{:<9} robust RMSE={:6.3f}m p95={:6.3f}m "
                "legacy RMSE={:7.3f}m yaw={:5.2f}deg rejected={:.1f}".format(
                    name,
                    metrics["robust_rmse_m"],
                    metrics["robust_p95_m"],
                    metrics["legacy_rmse_m"],
                    metrics["robust_yaw_rmse_deg"],
                    metrics["gps_rejected"]))
        print("PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
