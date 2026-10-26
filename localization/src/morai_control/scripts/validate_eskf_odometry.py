#!/usr/bin/env python3
"""Monte Carlo gate for ESKF + validated relative odometry aiding.

The test continuously perturbs GPS and IMU, injects GPS outliers, creates
random GPS-shadow intervals, and compares the same ESKF with and without the
pure-odometry speed/relative-position updates used by the ROS wrapper.
"""

import argparse
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
from validate_eskf_noise import HampelInputFilter, motion_command  # noqa: E402


CONFIG = {
    "accel_noise_std": 0.35,
    "gyro_noise_std": 0.015,
    "accel_bias_random_walk_std": 0.02,
    "gyro_bias_random_walk_std": 0.002,
    "gps_position_variance": 1.0,
    "gps_velocity_variance_floor": 0.5,
    "wheel_speed_variance": 0.09,
    "imu_yaw_variance": 0.02,
    "nhc_variance": 0.09,
    "zupt_velocity_variance": 0.01,
    "gps_nis_soft": 5.991,
    "gps_nis_hard": 13.816,
    "velocity_nis_soft": 5.991,
    "velocity_nis_hard": 13.816,
    "wheel_speed_nis_soft": 3.841,
    "wheel_speed_nis_hard": 10.828,
    "yaw_nis_soft": 3.841,
    "yaw_nis_hard": 10.828,
    "constraint_nis_hard": 10.828,
    "odometry_nis_soft": 5.991,
    "odometry_nis_hard": 13.816,
    "adaptive_alpha": 0.04,
    "adaptive_scale_min": 0.5,
    "adaptive_scale_max": 16.0,
}


def random_dropouts(rng, duration):
    intervals = []
    for _ in range(int(rng.integers(2, 5))):
        length = float(rng.uniform(8.0, 35.0))
        start = float(rng.uniform(25.0, duration - length - 8.0))
        intervals.append((start, start + length))
    return tuple(sorted(intervals))


def in_intervals(time_sec, intervals):
    return any(start <= time_sec <= end for start, end in intervals)


class Runtime:
    def __init__(self, odometry_aided):
        self.filter = RobustPlanarESKF(CONFIG)
        self.odometry_aided = bool(odometry_aided)
        self.gps_history = deque(maxlen=30)
        self.last_gps_accept = None
        self.last_gps_message = None
        self.reacquisition = deque(maxlen=14)
        self.alignment = SE2Alignment(
            alpha=0.05,
            reset_distance=5.0,
            reset_yaw=math.radians(20.0),
            reset_confirm_samples=3,
            min_samples=10,
            min_baseline=5.0,
            continuity_confirm_samples=5)
        self.zupt_since = None
        self.odometry_accepts = 0

    def initialize(self, position, yaw, time_sec):
        self.filter.initialize(position, yaw)
        self.last_gps_accept = time_sec
        self.last_gps_message = time_sec
        self.gps_history.append((time_sec, np.asarray(position).copy()))

    def propagate(self, acceleration, yaw_rate, yaw, wheel_speed,
                  time_sec, dt):
        self.filter.predict(acceleration, yaw_rate, dt)
        self.filter.update_yaw(yaw, 0.05**2)
        if self.odometry_aided and int(round(time_sec / dt)) % 3 == 0:
            self.filter.update_wheel_speed(wheel_speed, 0.09)
        _, velocity = self.filter.base_state()
        if int(round(time_sec / dt)) % 5 == 0 and np.linalg.norm(velocity) > 0.5:
            self.filter.update_nonholonomic_constraint(yaw_rate)
        corrected_accel = (
            acceleration
            - self.filter.x[self.filter.BAX:self.filter.BAY + 1])
        externally_moving = (
            self.odometry_aided and abs(wheel_speed) >= 0.25)
        stationary = (
            not externally_moving
            and np.linalg.norm(velocity) < 0.25
            and np.linalg.norm(corrected_accel) < 0.3
            and abs(yaw_rate - self.filter.x[self.filter.BGZ]) < 0.03)
        if stationary:
            if self.zupt_since is None:
                self.zupt_since = time_sec
            elif time_sec - self.zupt_since >= 0.8:
                self.filter.update_zero_velocity()
        else:
            self.zupt_since = None

    def update_gps(self, position, time_sec):
        gap = (
            math.inf if self.last_gps_message is None
            else time_sec - self.last_gps_message)
        self.last_gps_message = time_sec
        if gap >= 1.5:
            self.reacquisition.clear()
        if gap >= 1.5 or self.reacquisition:
            self.reacquisition.append(np.asarray(position).copy())
            if len(self.reacquisition) < 7:
                return False
            samples = np.asarray(self.reacquisition)
            center = np.median(samples, axis=0)
            inliers = samples[np.linalg.norm(samples - center, axis=1) <= 6.0]
            if len(inliers) < 4:
                return False
            position = np.mean(inliers, axis=0)
            self.filter.reanchor_gps_position(position)
            self.gps_history.clear()
            self.gps_history.append((time_sec, position.copy()))
            self.last_gps_accept = time_sec
            self.reacquisition.clear()
            return True

        accepted, _ = self.filter.update_gps_position(position)
        if not accepted:
            return False
        velocity_samples = [
            sample for sample in self.gps_history
            if 0.0 <= time_sec - sample[0] <= 2.0]
        velocity_samples.append((time_sec, np.asarray(position).copy()))
        estimate = robust_gps_velocity(velocity_samples, min_span=0.6)
        if estimate is not None:
            velocity, denominator, residual_variance = estimate
            variance = (
                max(CONFIG["gps_position_variance"], residual_variance)
                / denominator
                + CONFIG["gps_velocity_variance_floor"])
            self.filter.update_gps_velocity(velocity, variance)
        self.gps_history.append((time_sec, np.asarray(position).copy()))
        self.last_gps_accept = time_sec
        return True

    def update_odometry_pose(self, local_position, local_yaw, time_sec,
                             shadow_distance, incremental_variance):
        if not self.odometry_aided:
            return
        base_position, _ = self.filter.base_state()
        gps_age = (
            math.inf if self.last_gps_accept is None
            else time_sec - self.last_gps_accept)
        if gps_age <= 0.75:
            self.alignment.update(
                base_position,
                self.filter.x[self.filter.YAW],
                local_position,
                local_yaw,
                time_sec)
            return
        if gps_age < 1.0 or not self.alignment.ready:
            return
        global_position, _ = self.alignment.transform(
            local_position, local_yaw)
        variance = 9.0 * (
            0.25
            + incremental_variance
            + (0.03 * shadow_distance)**2)
        accepted, _ = self.filter.update_odometry_position(
            global_position, variance)
        if accepted:
            self.odometry_accepts += 1


def run_trial(seed, duration=150.0):
    rng = np.random.default_rng(seed)
    dt = 0.02
    gps_period = 5
    odometry_period = 3
    dropouts = random_dropouts(rng, duration)
    baseline = Runtime(False)
    aided = Runtime(True)
    input_filter = HampelInputFilter()
    initialization = deque(maxlen=7)
    true_position = np.zeros(2, dtype=float)
    true_yaw = 0.0
    local_position = rng.uniform(-30.0, 30.0, 2)
    local_yaw = float(rng.uniform(-math.pi, math.pi))
    odometry_speed_scale = float(rng.normal(1.0, 0.004))
    odometry_yaw_bias = float(rng.normal(0.0, 0.0015))
    shadow_distance = 0.0
    incremental_variance = 0.0
    errors = {"baseline": [], "aided": []}
    outage_errors = {"baseline": [], "aided": []}

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
        acceleration = (
            true_acceleration
            + np.array([0.22, -0.15])
            + rng.normal(0.0, 0.25, 2))
        gyro = yaw_rate + 0.008 + rng.normal(0.0, 0.01)
        if rng.random() < 0.005:
            acceleration += rng.normal(0.0, 10.0, 2)
            gyro += rng.normal(0.0, 1.0)
        filtered = input_filter.apply([
            acceleration[0], acceleration[1], gyro])
        measured_yaw = wrap_angle(true_yaw + rng.normal(0.0, 0.05))
        if rng.random() < 0.005:
            measured_yaw = wrap_angle(measured_yaw + rng.normal(0.0, 1.0))

        wheel_speed = (
            speed * odometry_speed_scale + rng.normal(0.0, 0.05))
        local_yaw = wrap_angle(
            local_yaw
            + (yaw_rate + odometry_yaw_bias
               + rng.normal(0.0, 0.003)) * dt)
        local_position += np.array([
            math.cos(local_yaw) * wheel_speed,
            math.sin(local_yaw) * wheel_speed,
        ]) * dt

        gps_available = not in_intervals(time_sec, dropouts)
        if gps_available:
            shadow_distance = 0.0
            incremental_variance = 0.0
        else:
            shadow_distance += abs(wheel_speed) * dt
            incremental_variance += (0.05 * dt)**2
        gps_position = None
        if step % gps_period == 0 and gps_available:
            gps_position = true_position + rng.normal(0.0, 1.5, 2)
            if rng.random() < 0.03:
                direction = rng.uniform(-math.pi, math.pi)
                gps_position += rng.uniform(18.0, 35.0) * np.array([
                    math.cos(direction), math.sin(direction)])

        if not aided.filter.initialized:
            if gps_position is None:
                continue
            initialization.append(gps_position.copy())
            samples = np.asarray(initialization)
            center = np.median(samples, axis=0)
            inliers = samples[np.linalg.norm(samples - center, axis=1) <= 6.0]
            if len(inliers) < 4:
                continue
            initial_position = np.mean(inliers, axis=0)
            baseline.initialize(initial_position, measured_yaw, time_sec)
            aided.initialize(initial_position, measured_yaw, time_sec)
        else:
            for runtime in (baseline, aided):
                runtime.propagate(
                    filtered[0:2], filtered[2], measured_yaw,
                    wheel_speed, time_sec, dt)
                if gps_position is not None:
                    runtime.update_gps(gps_position.copy(), time_sec)
            if step % odometry_period == 0:
                aided.update_odometry_pose(
                    local_position,
                    local_yaw,
                    time_sec,
                    shadow_distance,
                    incremental_variance)

        if aided.filter.initialized and time_sec >= 2.0:
            for name, runtime in (("baseline", baseline), ("aided", aided)):
                position, _ = runtime.filter.base_state()
                error = float(np.linalg.norm(position - true_position))
                errors[name].append(error)
                if not gps_available:
                    outage_errors[name].append(error)

    def metrics(values):
        array = np.asarray(values, dtype=float)
        return {
            "rmse_m": float(np.sqrt(np.mean(array**2))),
            "p95_m": float(np.percentile(array, 95)),
            "max_m": float(np.max(array)),
        }

    return {
        "seed": seed,
        "dropouts": dropouts,
        "baseline": metrics(errors["baseline"]),
        "aided": metrics(errors["aided"]),
        "outage_baseline": metrics(outage_errors["baseline"]),
        "outage_aided": metrics(outage_errors["aided"]),
        "odometry_accepts": aided.odometry_accepts,
        "state_finite": bool(
            baseline.filter.is_numerically_valid()
            and aided.filter.is_numerically_valid()),
    }


def aggregate(trials):
    def mean(path):
        return float(np.mean([
            trial[path[0]][path[1]] for trial in trials]))

    result = {
        "trials": len(trials),
        "baseline_rmse_m": mean(("baseline", "rmse_m")),
        "aided_rmse_m": mean(("aided", "rmse_m")),
        "outage_baseline_p95_m": mean(("outage_baseline", "p95_m")),
        "outage_aided_p95_m": mean(("outage_aided", "p95_m")),
        "outage_aided_max_m": float(max(
            trial["outage_aided"]["max_m"] for trial in trials)),
        "odometry_accepts": int(sum(
            trial["odometry_accepts"] for trial in trials)),
        "all_states_finite": all(trial["state_finite"] for trial in trials),
    }
    result["passed"] = bool(
        result["all_states_finite"]
        and result["odometry_accepts"] > 0
        and result["outage_aided_p95_m"] < 6.0
        and result["outage_aided_max_m"] < 15.0
        and result["outage_aided_p95_m"]
        < 0.65 * result["outage_baseline_p95_m"])
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20260803)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    trials = [run_trial(args.seed + index) for index in range(args.trials)]
    result = aggregate(trials)
    payload = {"summary": result, "trials": trials}
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(
            "baseline RMSE={:.3f}m aided RMSE={:.3f}m | "
            "outage p95 {:.3f}m -> {:.3f}m max={:.3f}m accepts={}".format(
                result["baseline_rmse_m"],
                result["aided_rmse_m"],
                result["outage_baseline_p95_m"],
                result["outage_aided_p95_m"],
                result["outage_aided_max_m"],
                result["odometry_accepts"]))
        print("PASS" if result["passed"] else "FAIL")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
