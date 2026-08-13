#!/usr/bin/env python3
"""Offline comparison of IMU-derived MORAI motion-rate scale strategies."""

import argparse
import glob
import json
import math
import os
from collections import deque

import numpy as np
import rosbag
from tf.transformations import euler_from_quaternion


STRATEGIES = {
    "bootstrap_alpha_0.02": (10.0, "bootstrap", 0.02),
    "alpha_0.02": (10.0, "fixed_alpha", 0.02),
    "alpha_0.10": (10.0, "bootstrap", 0.10),
    "alpha_0.25": (10.0, "bootstrap", 0.25),
    "direct_candidate": (10.0, "direct", 1.0),
    "window_5s_alpha_0.10": (5.0, "bootstrap", 0.10),
    "window_5s_direct": (5.0, "direct", 1.0),
    "window_3s_alpha_0.10": (3.0, "bootstrap", 0.10),
    "instant_median_1s": (1.0, "median_direct", 1.0),
    "instant_median_2s": (2.0, "median_direct", 1.0),
    "instant_median_3s": (3.0, "median_direct", 1.0),
    "instant_median_2s_alpha_0.25": (2.0, "median_alpha", 0.25),
    "trend_10s_beta_0.5": (10.0, "trend_0.5", 1.0),
    "trend_10s_beta_1.0": (10.0, "trend_1.0", 1.0),
}


def wrap_angle(value):
    return np.arctan2(np.sin(value), np.cos(value))


def unique_sorted(rows):
    return np.asarray(sorted(dict((row[0], row) for row in rows).values()),
                      dtype=float)


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
                    float(message.position.x),
                    float(message.position.y),
                    math.radians(float(message.heading)),
                ))
            else:
                quaternion = message.orientation
                values = np.asarray([
                    quaternion.x, quaternion.y,
                    quaternion.z, quaternion.w], dtype=float)
                norm = float(np.linalg.norm(values))
                if (
                        not np.all(np.isfinite(values))
                        or norm <= 1.0e-6
                        or message.orientation_covariance[0] == -1.0):
                    continue
                yaw = euler_from_quaternion((values / norm).tolist())[2]
                imu.append((
                    stamp, yaw, float(message.angular_velocity.z)))
    return unique_sorted(vehicle), unique_sorted(imu)


def scale_series(imu, window_sec, mode, alpha):
    history = deque()
    ratio_history = deque()
    candidate_history = deque()
    scales = np.empty(len(imu), dtype=float)
    candidates = np.full(len(imu), np.nan, dtype=float)
    scale = 0.49848
    updates = 0
    for index in range(len(imu)):
        if index:
            dt = imu[index, 0] - imu[index - 1, 0]
            if 0.0 < dt <= 0.5:
                signed_yaw_increment = float(wrap_angle(
                    imu[index, 1] - imu[index - 1, 1]))
                mean_gyro = float(
                    0.5 * (imu[index - 1, 2] + imu[index, 2]))
                yaw_increment = abs(signed_yaw_increment)
                gyro_increment = abs(mean_gyro * dt)
                history.append((
                    imu[index, 0], yaw_increment, gyro_increment))
                if abs(mean_gyro) >= 0.03:
                    ratio = signed_yaw_increment / (mean_gyro * dt)
                    if 0.2 <= ratio <= 1.2:
                        ratio_history.append((imu[index, 0], ratio))
        cutoff = imu[index, 0] - window_sec
        while history and history[0][0] < cutoff:
            history.popleft()
        while ratio_history and ratio_history[0][0] < cutoff:
            ratio_history.popleft()
        yaw_sum = sum(row[1] for row in history)
        gyro_sum = sum(row[2] for row in history)
        if mode.startswith("median") and len(ratio_history) >= 5:
            candidate = float(np.median([
                row[1] for row in ratio_history]))
            candidates[index] = candidate
            gain = 1.0 if mode == "median_direct" else alpha
            scale += gain * (candidate - scale)
            updates += 1
        elif not mode.startswith("median") and gyro_sum >= 0.05:
            candidate = yaw_sum / gyro_sum
            if 0.2 <= candidate <= 1.2:
                candidates[index] = candidate
                candidate_history.append((imu[index, 0], candidate))
                while (
                        candidate_history
                        and candidate_history[0][0]
                        < imu[index, 0] - 20.0):
                    candidate_history.popleft()
                if mode.startswith("trend_"):
                    beta = float(mode.split("_", 1)[1])
                    target_stamp = imu[index, 0] - 0.5 * window_sec
                    old_candidate = candidate_history[0][1]
                    for history_stamp, history_candidate in candidate_history:
                        if history_stamp > target_stamp:
                            break
                        old_candidate = history_candidate
                    predicted = candidate + beta * (
                        candidate - old_candidate)
                    scale = float(np.clip(predicted, 0.2, 1.2))
                    updates += 1
                    scales[index] = scale
                    continue
                if mode == "direct":
                    gain = 1.0
                elif mode == "bootstrap":
                    gain = max(alpha, 1.0 / (updates + 1.0))
                else:
                    gain = alpha
                scale += gain * (candidate - scale)
                updates += 1
        scales[index] = scale
    return scales, candidates


def integrate(vehicle, imu, scales):
    imu_yaw = np.unwrap(imu[:, 1])
    yaw_at_vehicle = np.interp(vehicle[:, 0], imu[:, 0], imu_yaw)
    scale_at_vehicle = np.interp(vehicle[:, 0], imu[:, 0], scales)
    estimated = np.zeros((len(vehicle), 5), dtype=float)
    estimated[:, 0] = vehicle[:, 0]
    estimated[0, 3] = 0.0
    filtered_speed = float(vehicle[0, 1])
    estimated[0, 4] = filtered_speed
    yaw_reference = float(yaw_at_vehicle[0])
    for index in range(1, len(vehicle)):
        source_dt = vehicle[index, 0] - vehicle[index - 1, 0]
        if source_dt <= 0.0 or source_dt > 1.0:
            estimated[index] = estimated[index - 1]
            estimated[index, 0] = vehicle[index, 0]
            continue
        dt = source_dt * scale_at_vehicle[index]
        speed_alpha = 1.0 - math.exp(-dt / 0.04)
        previous_speed = filtered_speed
        filtered_speed += speed_alpha * (
            vehicle[index, 1] - filtered_speed)
        previous_yaw = estimated[index - 1, 3]
        yaw = float(wrap_angle(yaw_at_vehicle[index] - yaw_reference))
        yaw_increment = float(wrap_angle(yaw - previous_yaw))
        yaw_mid = previous_yaw + 0.5 * yaw_increment
        distance = 0.5 * (previous_speed + filtered_speed) * dt
        estimated[index, 1] = (
            estimated[index - 1, 1] + distance * math.cos(yaw_mid))
        estimated[index, 2] = (
            estimated[index - 1, 2] + distance * math.sin(yaw_mid))
        estimated[index, 3] = yaw
        estimated[index, 4] = filtered_speed
    return estimated


def align_and_score(estimated, vehicle, settle_sec=0.5):
    steps = np.hypot(np.diff(vehicle[:, 2]), np.diff(vehicle[:, 3]))
    jumps = np.flatnonzero(steps > 20.0)
    boundaries = [0] + (jumps + 1).tolist() + [len(vehicle)]
    position_errors = []
    yaw_errors = []
    endpoint_errors = []
    truth_distance = 0.0
    rolling = {30.0: [], 60.0: [], 120.0: []}
    for first, last in zip(boundaries[:-1], boundaries[1:]):
        start_stamp = vehicle[first, 0] + settle_sec
        first = int(np.searchsorted(vehicle[:, 0], start_stamp, side="left"))
        if last - first < 20:
            continue
        local = estimated[first:last]
        truth = vehicle[first:last]
        yaw_offset = float(wrap_angle(truth[0, 4] - local[0, 3]))
        cosine = math.cos(yaw_offset)
        sine = math.sin(yaw_offset)
        rotated = np.column_stack((
            cosine * local[:, 1] - sine * local[:, 2],
            sine * local[:, 1] + cosine * local[:, 2]))
        translation = truth[0, 2:4] - rotated[0]
        aligned = rotated + translation
        errors = np.hypot(
            aligned[:, 0] - truth[:, 2],
            aligned[:, 1] - truth[:, 3])
        yaw = wrap_angle(local[:, 3] + yaw_offset - truth[:, 4])
        position_errors.extend(errors.tolist())
        yaw_errors.extend(np.degrees(yaw).tolist())
        endpoint_errors.append(float(errors[-1]))
        truth_distance += float(np.sum(np.hypot(
            np.diff(truth[:, 2]), np.diff(truth[:, 3]))))
        for duration, destination in rolling.items():
            next_start = local[0, 0]
            for index in range(len(local) - 1):
                if local[index, 0] + 1.0e-9 < next_start:
                    continue
                end = int(np.searchsorted(
                    local[:, 0], local[index, 0] + duration))
                if end >= len(local):
                    break
                window_yaw_offset = float(wrap_angle(
                    truth[index, 4] - local[index, 3]))
                c = math.cos(window_yaw_offset)
                s = math.sin(window_yaw_offset)
                delta_local = local[end, 1:3] - local[index, 1:3]
                predicted = truth[index, 2:4] + np.array([
                    c * delta_local[0] - s * delta_local[1],
                    s * delta_local[0] + c * delta_local[1],
                ])
                destination.append(float(np.linalg.norm(
                    predicted - truth[end, 2:4])))
                next_start = local[index, 0] + 5.0
    if not position_errors:
        raise RuntimeError("no continuous scoring segment")
    position_errors = np.asarray(position_errors)
    yaw_errors = np.asarray(yaw_errors)
    return {
        "position_rmse_m": float(np.sqrt(np.mean(position_errors ** 2))),
        "position_p95_m": float(np.percentile(position_errors, 95)),
        "position_max_m": float(np.max(position_errors)),
        "yaw_rmse_deg": float(np.sqrt(np.mean(yaw_errors ** 2))),
        "endpoint_max_m": max(endpoint_errors),
        "truth_distance_m": truth_distance,
        "rolling_position_p95_m": {
            "{}s".format(int(duration)): (
                float(np.percentile(values, 95)) if values else None)
            for duration, values in rolling.items()
        },
    }


def evaluate_bag(path):
    vehicle, imu = load_bag(path)
    if len(vehicle) < 20 or len(imu) < 20:
        raise RuntimeError("insufficient Ego/IMU orientation samples")
    results = {}
    for name, (window, mode, alpha) in STRATEGIES.items():
        scales, candidates = scale_series(imu, window, mode, alpha)
        estimated = integrate(vehicle, imu, scales)
        score = align_and_score(estimated, vehicle)
        score.update({
            "scale_mean": float(np.mean(scales)),
            "scale_final": float(scales[-1]),
            "candidate_count": int(np.count_nonzero(np.isfinite(candidates))),
        })
        results[name] = score
    return {
        "bag": os.path.abspath(path),
        "vehicle_samples": len(vehicle),
        "imu_samples": len(imu),
        "strategies": results,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    paths = []
    for pattern in args.bags:
        matches = sorted(glob.glob(pattern))
        paths.extend(matches if matches else [pattern])
    reports = []
    skipped = []
    for path in paths:
        try:
            reports.append(evaluate_bag(path))
        except Exception as error:  # report malformed legacy bags explicitly
            skipped.append({"bag": os.path.abspath(path), "reason": str(error)})
    aggregate = {}
    for name in STRATEGIES:
        rows = [report["strategies"][name] for report in reports]
        aggregate[name] = {
            "bags": len(rows),
            "mean_position_rmse_m": float(np.mean([
                row["position_rmse_m"] for row in rows])) if rows else None,
            "mean_yaw_rmse_deg": float(np.mean([
                row["yaw_rmse_deg"] for row in rows])) if rows else None,
            "mean_60s_p95_m": float(np.mean([
                row["rolling_position_p95_m"]["60s"] for row in rows
                if row["rolling_position_p95_m"]["60s"] is not None
            ])) if any(
                row["rolling_position_p95_m"]["60s"] is not None
                for row in rows) else None,
        }
    result = {
        "validation_only_ground_truth": True,
        "reports": reports,
        "skipped": skipped,
        "aggregate": aggregate,
    }
    payload = json.dumps(result, indent=2, sort_keys=True)
    print(payload)
    if args.output:
        output = os.path.abspath(os.path.expanduser(args.output))
        os.makedirs(os.path.dirname(output), exist_ok=True)
        with open(output, "w", encoding="utf-8") as stream:
            stream.write(payload + "\n")


if __name__ == "__main__":
    main()
