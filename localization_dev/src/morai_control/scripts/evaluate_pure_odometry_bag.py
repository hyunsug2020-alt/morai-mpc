#!/usr/bin/env python3
"""Deterministic offline regression test for pure wheel/IMU odometry.

Ground-truth position and heading are read only to score the estimate.  They
are never passed to the dead-reckoning update.  ``--legacy-auto-time-scale``
exists solely for bags recorded before MORAI Simulation Time was enabled.
"""

import argparse
import glob
import json
import math
import os

import numpy as np
import rosbag


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def alpha(delta_time, time_constant):
    if time_constant <= 0.0:
        return 1.0
    return 1.0 - math.exp(-max(0.0, delta_time) / time_constant)


def unique_by_stamp(samples):
    return [dict((row[0], row) for row in samples)[key]
            for key in sorted(set(row[0] for row in samples))]


def load_bag(path):
    vehicle = []
    imu = []
    with rosbag.Bag(path) as bag:
        for topic, message, _ in bag.read_messages(
                topics=["/Ego_topic", "/imu/data"]):
            stamp = message.header.stamp.to_sec()
            if topic == "/Ego_topic":
                vehicle.append((
                    stamp,
                    float(message.velocity.x),
                    math.radians(float(message.wheel_angle)),
                    float(message.position.x),
                    float(message.position.y),
                    math.radians(float(message.heading)),
                ))
            else:
                imu.append((stamp, float(message.angular_velocity.z)))
    return (
        np.asarray(unique_by_stamp(vehicle), dtype=float),
        np.asarray(unique_by_stamp(imu), dtype=float),
    )


def evaluate(path, args):
    vehicle, imu = load_bag(path)
    if len(vehicle) < 2 or len(imu) < 2:
        raise RuntimeError("required /Ego_topic or /imu/data samples missing")

    source_dt = np.diff(vehicle[:, 0])
    truth_distance = float(np.sum(np.hypot(
        np.diff(vehicle[:, 3]), np.diff(vehicle[:, 4]))))
    speed_integral = float(np.sum(
        0.5 * (np.abs(vehicle[:-1, 1]) + np.abs(vehicle[1:, 1]))
        * np.maximum(source_dt, 0.0)))
    time_scale = 1.0
    if args.legacy_auto_time_scale and speed_integral > 0.1:
        time_scale = truth_distance / speed_integral

    x = float(vehicle[0, 3])
    y = float(vehicle[0, 4])
    yaw = float(vehicle[0, 5])
    filtered_speed = float(vehicle[0, 1])
    filtered_rate = 0.0
    gyro_bias = 0.0
    stationary_since = None
    estimated = []
    stale_imu = 0
    rejected_gap = 0
    gyro_outliers = 0
    model_disagreements = 0
    applied_weights = []

    imu_stamps = imu[:, 0]
    for index in range(1, len(vehicle)):
        previous = vehicle[index - 1]
        current = vehicle[index]
        delta_time = (current[0] - previous[0]) * time_scale
        if delta_time <= 0.0:
            continue
        if delta_time > args.max_recoverable_gap:
            rejected_gap += 1
            continue

        previous_speed = filtered_speed
        filtered_speed += alpha(delta_time, args.speed_tau) * (
            current[1] - filtered_speed)
        steering = float(current[2])
        denominator = args.wheelbase + args.understeer * filtered_speed**2
        model_rate = filtered_speed / denominator * math.tan(steering)

        source_stamp = float(current[0])
        gyro = float(np.interp(source_stamp, imu_stamps, imu[:, 1]))
        imu_age = float(np.min(np.abs(imu_stamps - source_stamp)))
        imu_fresh = imu_age <= args.imu_timeout
        corrected_gyro = gyro - gyro_bias
        previous_rate = filtered_rate

        if abs(filtered_speed) < args.stationary_speed:
            if stationary_since is None:
                stationary_since = float(current[0])
            stationary_duration = max(
                0.0, (float(current[0]) - stationary_since) * time_scale)
            if (imu_fresh
                    and stationary_duration >= args.stationary_bias_hold
                    and abs(gyro) < args.stationary_gyro):
                gyro_bias += alpha(delta_time, 2.0) * (gyro - gyro_bias)
            fused_rate = 0.0
            filtered_speed = 0.0
            weight = 0.0
        elif not imu_fresh:
            stationary_since = None
            fused_rate = model_rate
            weight = 0.0
            stale_imu += 1
        else:
            stationary_since = None
            innovation = corrected_gyro - model_rate
            model_noise = (
                args.model_yaw_rate_noise
                + 0.15 * abs(steering)
                + 0.0015 * filtered_speed**2)
            innovation_variance = (
                model_noise**2 + args.gyro_yaw_rate_noise**2)
            nis = innovation**2 / max(innovation_variance, 1.0e-9)
            if abs(corrected_gyro) > args.max_abs_gyro:
                fused_rate = model_rate
                weight = 0.0
                gyro_outliers += 1
            else:
                turn_fraction = min(1.0, abs(steering) / 0.20)
                weight = args.imu_weight + (
                    args.imu_weight_max - args.imu_weight
                ) * turn_fraction
                if nis > args.nis_gate:
                    weight = args.imu_weight_max
                    model_disagreements += 1
                weight = min(
                    args.imu_weight_max,
                    max(args.imu_weight_min, weight))
                fused_rate = model_rate + weight * innovation
                if (abs(filtered_speed) > 3.0
                        and abs(model_rate) < 0.02
                        and abs(steering) < math.radians(0.5)):
                    correction = args.moving_bias_gain * (
                        gyro - model_rate - gyro_bias) * delta_time
                    gyro_bias += min(1.0e-4, max(-1.0e-4, correction))

        filtered_rate += alpha(delta_time, args.yaw_rate_tau) * (
            fused_rate - filtered_rate)
        if filtered_speed == 0.0:
            filtered_rate = 0.0
        mean_rate = 0.5 * (previous_rate + filtered_rate)
        mean_speed = 0.5 * (previous_speed + filtered_speed)
        yaw_middle = yaw + 0.5 * mean_rate * delta_time
        distance = mean_speed * delta_time
        x += distance * math.cos(yaw_middle)
        y += distance * math.sin(yaw_middle)
        yaw = math.atan2(
            math.sin(yaw + mean_rate * delta_time),
            math.cos(yaw + mean_rate * delta_time))
        estimated.append((index, x, y, yaw))
        applied_weights.append(weight)

    if not estimated:
        raise RuntimeError("no usable odometry intervals")
    indices = np.asarray([row[0] for row in estimated], dtype=int)
    estimate = np.asarray([row[1:] for row in estimated], dtype=float)
    truth = vehicle[indices, 3:6]
    position_error = np.hypot(
        estimate[:, 0] - truth[:, 0],
        estimate[:, 1] - truth[:, 1])
    yaw_error = wrap(estimate[:, 2] - truth[:, 2])
    position_rmse = float(np.sqrt(np.mean(position_error**2)))
    yaw_rmse_deg = float(np.degrees(np.sqrt(np.mean(yaw_error**2))))
    endpoint_error = float(position_error[-1])
    distance_denominator = max(truth_distance, 1.0)
    result = {
        "bag": os.path.abspath(path),
        "validation_only_ground_truth": True,
        "estimator_inputs": ["velocity.x", "wheel_angle", "imu.angular_velocity.z"],
        "uses_gps": False,
        "uses_lidar": False,
        "vehicle_unique_samples": int(len(vehicle)),
        "imu_unique_samples": int(len(imu)),
        "truth_distance_m": truth_distance,
        "source_duration_s": float(vehicle[-1, 0] - vehicle[0, 0]),
        "time_scale": time_scale,
        "legacy_truth_time_scale_enabled": bool(args.legacy_auto_time_scale),
        "position_rmse_m": position_rmse,
        "position_rmse_percent_distance": 100.0 * position_rmse / distance_denominator,
        "position_endpoint_error_m": endpoint_error,
        "endpoint_drift_percent_distance": 100.0 * endpoint_error / distance_denominator,
        "yaw_rmse_deg": yaw_rmse_deg,
        "gyro_bias_radps": gyro_bias,
        "mean_imu_weight": float(np.mean(applied_weights)),
        "imu_stale_fallbacks": stale_imu,
        "gyro_outliers": gyro_outliers,
        "model_disagreements": model_disagreements,
        "rejected_time_gaps": rejected_gap,
    }
    result["passes_regression_gate"] = bool(
        truth_distance < 1.0
        or (
            result["position_rmse_percent_distance"] <= 3.5
            and result["endpoint_drift_percent_distance"] <= 5.0
            and yaw_rmse_deg <= 2.0
            and stale_imu <= max(2, int(0.01 * len(vehicle)))
        ))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", help="bag paths or glob patterns")
    parser.add_argument("--output-json", default="")
    parser.add_argument("--legacy-auto-time-scale", action="store_true")
    parser.add_argument("--wheelbase", type=float, default=3.0)
    parser.add_argument("--understeer", type=float, default=0.022)
    parser.add_argument("--imu-weight", type=float, default=0.95)
    parser.add_argument("--imu-weight-min", type=float, default=0.85)
    parser.add_argument("--imu-weight-max", type=float, default=0.98)
    parser.add_argument("--imu-timeout", type=float, default=0.08)
    parser.add_argument("--nis-gate", type=float, default=6.63)
    parser.add_argument("--model-yaw-rate-noise", type=float, default=0.03)
    parser.add_argument("--gyro-yaw-rate-noise", type=float, default=0.03)
    parser.add_argument("--max-abs-gyro", type=float, default=2.0)
    parser.add_argument("--moving-bias-gain", type=float, default=0.002)
    parser.add_argument("--stationary-speed", type=float, default=0.05)
    parser.add_argument("--stationary-gyro", type=float, default=0.03)
    parser.add_argument("--stationary-bias-hold", type=float, default=0.8)
    parser.add_argument("--speed-tau", type=float, default=0.04)
    parser.add_argument("--yaw-rate-tau", type=float, default=0.02)
    parser.add_argument("--max-dt", type=float, default=0.25)
    parser.add_argument("--max-recoverable-gap", type=float, default=1.0)
    args = parser.parse_args()

    paths = []
    for pattern in args.bags:
        matches = sorted(glob.glob(pattern))
        paths.extend(matches if matches else [pattern])
    results = [evaluate(path, args) for path in paths]
    moving = [row for row in results if row["truth_distance_m"] >= 1.0]
    summary = {
        "results": results,
        "aggregate": {
            "bag_count": len(results),
            "moving_bag_count": len(moving),
            "passed": sum(row["passes_regression_gate"] for row in results),
            "pass_rate_percent": 100.0 * sum(
                row["passes_regression_gate"] for row in results
            ) / max(len(results), 1),
            "mean_position_rmse_m": float(np.mean([
                row["position_rmse_m"] for row in moving])) if moving else 0.0,
            "mean_yaw_rmse_deg": float(np.mean([
                row["yaw_rmse_deg"] for row in moving])) if moving else 0.0,
            "all_passed": all(
                row["passes_regression_gate"] for row in results),
        },
    }
    rendered = json.dumps(summary, indent=2, sort_keys=True)
    print(rendered)
    if args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    if not summary["aggregate"]["all_passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
