#!/usr/bin/env python3
"""Evaluate recorded /eskf/odom against MORAI ground truth by source stamp."""

import argparse
import json
import math
import os

import numpy as np
import rosbag
from tf.transformations import euler_from_quaternion


def wrap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def summarize(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return {
        "samples": int(len(values)),
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values ** 2))),
        "p95_abs": float(np.percentile(np.abs(values), 95)),
        "max_abs": float(np.max(np.abs(values))),
    }


def load_bag(path):
    truth = []
    estimates = []
    diagnostic_records = []
    estimate_clock_samples = []
    with rosbag.Bag(path) as bag:
        for topic, message, record_stamp in bag.read_messages(
                topics=["/Ego_topic", "/eskf/odom", "/eskf/diagnostics"]):
            if topic == "/eskf/diagnostics":
                try:
                    diagnostic_records.append((
                        record_stamp.to_sec(), json.loads(message.data)))
                except (TypeError, ValueError, json.JSONDecodeError):
                    pass
                continue
            stamp = message.header.stamp.to_sec()
            if topic == "/Ego_topic":
                truth.append((
                    stamp,
                    float(message.position.x),
                    float(message.position.y),
                    math.radians(float(message.heading)),
                    math.hypot(
                        float(message.velocity.x),
                        float(message.velocity.y)),
                ))
            else:
                estimate_clock_samples.append((
                    record_stamp.to_sec(), stamp))
                orientation = message.pose.pose.orientation
                yaw = euler_from_quaternion([
                    orientation.x, orientation.y,
                    orientation.z, orientation.w])[2]
                estimates.append((
                    stamp,
                    float(message.pose.pose.position.x),
                    float(message.pose.pose.position.y),
                    yaw,
                    math.hypot(
                        float(message.twist.twist.linear.x),
                        float(message.twist.twist.linear.y)),
                ))
    if len(truth) < 2 or not estimates:
        raise RuntimeError("bag needs /Ego_topic and /eskf/odom samples")
    truth = np.asarray(sorted(truth), dtype=float)
    estimates = np.asarray(sorted(estimates), dtype=float)
    truth = truth[np.r_[True, np.diff(truth[:, 0]) > 0.0]]
    estimates = estimates[np.r_[True, np.diff(estimates[:, 0]) > 0.0]]
    # Diagnostics do not carry a Header. A replay recorded at e.g. 4x rate has
    # bag-record time on a different scale from the source stamps, so convert
    # record time to source time using the adjacent ESKF odometry clock. This
    # also remains an identity-like mapping for ordinary live recordings.
    diagnostics = []
    if diagnostic_records and estimate_clock_samples:
        clock = np.asarray(estimate_clock_samples, dtype=float)
        order = np.argsort(clock[:, 0])
        clock = clock[order]
        unique = np.r_[True, np.diff(clock[:, 0]) > 0.0]
        clock = clock[unique]
        for record_stamp, payload in diagnostic_records:
            source_stamp = float(np.interp(
                record_stamp, clock[:, 0], clock[:, 1]))
            diagnostics.append((source_stamp, payload))
    return truth, estimates, diagnostics


def interpolate_truth(truth, stamps, max_gap):
    indices = np.searchsorted(truth[:, 0], stamps, side="right")
    valid = (indices > 0) & (indices < len(truth))
    lower = np.clip(indices - 1, 0, len(truth) - 1)
    upper = np.clip(indices, 0, len(truth) - 1)
    gaps = truth[upper, 0] - truth[lower, 0]
    valid &= (gaps > 0.0) & (gaps <= max_gap)
    fractions = np.zeros_like(stamps)
    fractions[valid] = (
        (stamps[valid] - truth[lower[valid], 0]) / gaps[valid])
    values = np.empty((len(stamps), 4), dtype=float)
    for column in (1, 2, 4):
        values[:, column - 1] = (
            truth[lower, column]
            + fractions * (truth[upper, column] - truth[lower, column]))
    yaw_unwrapped = np.unwrap(truth[:, 3])
    values[:, 2] = (
        yaw_unwrapped[lower]
        + fractions * (yaw_unwrapped[upper] - yaw_unwrapped[lower]))
    return valid, values


def segment_metrics(name, mask, position, yaw, speed, longitudinal, lateral):
    return {
        "name": name,
        "position_m": summarize(position[mask]),
        "yaw_deg": summarize(np.degrees(yaw[mask])),
        "speed_mps": summarize(speed[mask]),
        "longitudinal_m": summarize(longitudinal[mask]),
        "lateral_m": summarize(lateral[mask]),
    }


def evaluate(path, max_truth_gap, scan_lag):
    truth, estimates, diagnostics = load_bag(path)
    valid, interpolated = interpolate_truth(
        truth, estimates[:, 0], max_truth_gap)
    estimates = estimates[valid]
    interpolated = interpolated[valid]
    if len(estimates) < 100:
        raise RuntimeError("insufficient synchronized ESKF samples")

    delta_x = estimates[:, 1] - interpolated[:, 0]
    delta_y = estimates[:, 2] - interpolated[:, 1]
    position = np.hypot(delta_x, delta_y)
    yaw = wrap_angle(estimates[:, 3] - interpolated[:, 2])
    speed = estimates[:, 4] - interpolated[:, 3]
    cosine = np.cos(interpolated[:, 2])
    sine = np.sin(interpolated[:, 2])
    longitudinal = cosine * delta_x + sine * delta_y
    lateral = -sine * delta_x + cosine * delta_y
    truth_speed = interpolated[:, 3]

    top_indices = np.argsort(position)[-20:][::-1]
    diagnostic_stamps = np.asarray(
        [item[0] for item in diagnostics], dtype=float)
    top_errors = []
    for index in top_indices:
        diagnostic = None
        if len(diagnostic_stamps):
            diagnostic_index = int(np.argmin(np.abs(
                diagnostic_stamps - estimates[index, 0])))
            diagnostic = diagnostics[diagnostic_index][1]
        counters = (diagnostic or {}).get("counters", {})
        top_errors.append({
            "elapsed_sec": float(estimates[index, 0] - estimates[0, 0]),
            "position_error_m": float(position[index]),
            "longitudinal_error_m": float(longitudinal[index]),
            "lateral_error_m": float(lateral[index]),
            "yaw_error_deg": float(math.degrees(yaw[index])),
            "truth_position": interpolated[index, :2].tolist(),
            "eskf_position": estimates[index, 1:3].tolist(),
            "truth_speed_mps": float(truth_speed[index]),
            "eskf_speed_mps": float(estimates[index, 4]),
            "gps_mode": (diagnostic or {}).get("gps_mode"),
            "gps_rejected": counters.get("gps_rejected"),
            "odometry_alignment_ready": (
                (diagnostic or {}).get("odometry_alignment_ready")),
        })

    truth_steps = np.hypot(
        np.diff(truth[:, 1]), np.diff(truth[:, 2]))
    jump_indices = np.flatnonzero(truth_steps > 20.0)
    truth_jumps = [{
        "elapsed_sec": float(truth[index + 1, 0] - truth[0, 0]),
        "distance_m": float(truth_steps[index]),
        "dt_sec": float(truth[index + 1, 0] - truth[index, 0]),
        "before": truth[index, 1:3].tolist(),
        "after": truth[index + 1, 1:3].tolist(),
        "reported_speed_before_mps": float(truth[index, 4]),
        "reported_speed_after_mps": float(truth[index + 1, 4]),
    } for index in jump_indices]

    segments = [
        segment_metrics(
            "all", np.ones(len(estimates), dtype=bool),
            position, yaw, speed, longitudinal, lateral),
        segment_metrics(
            "stationary_lt_0.25mps", truth_speed < 0.25,
            position, yaw, speed, longitudinal, lateral),
        segment_metrics(
            "low_0.25_to_5mps",
            (truth_speed >= 0.25) & (truth_speed < 5.0),
            position, yaw, speed, longitudinal, lateral),
        segment_metrics(
            "medium_5_to_15mps",
            (truth_speed >= 5.0) & (truth_speed < 15.0),
            position, yaw, speed, longitudinal, lateral),
        segment_metrics(
            "high_ge_15mps", truth_speed >= 15.0,
            position, yaw, speed, longitudinal, lateral),
    ]

    lag_result = None
    moving = truth_speed >= 2.0
    if scan_lag > 0.0 and np.count_nonzero(moving) >= 100:
        candidates = np.linspace(-scan_lag, scan_lag, 81)
        scores = []
        moving_estimates = estimates[moving]
        for offset in candidates:
            lag_valid, lag_truth = interpolate_truth(
                truth, moving_estimates[:, 0] + offset, max_truth_gap)
            if np.count_nonzero(lag_valid) < 100:
                scores.append(math.inf)
                continue
            error = np.hypot(
                moving_estimates[lag_valid, 1] - lag_truth[lag_valid, 0],
                moving_estimates[lag_valid, 2] - lag_truth[lag_valid, 1])
            scores.append(float(np.sqrt(np.mean(error ** 2))))
        best = int(np.argmin(scores))
        zero = int(np.argmin(np.abs(candidates)))
        lag_result = {
            "best_truth_time_offset_sec": float(candidates[best]),
            "best_position_rmse_m": float(scores[best]),
            "zero_offset_position_rmse_m": float(scores[zero]),
            "interpretation": (
                "positive means ESKF position matches a later truth pose"),
        }

    return {
        "bag": os.path.abspath(path),
        "duration_sec": float(estimates[-1, 0] - estimates[0, 0]),
        "truth_samples": int(len(truth)),
        "eskf_samples": int(len(estimates)),
        "synchronization": "linear interpolation by ROS source stamp",
        "max_truth_gap_sec": max_truth_gap,
        "segments": segments,
        "lag_scan": lag_result,
        "top_errors": top_errors,
        "truth_position_jumps_over_20m": truth_jumps,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--max-truth-gap", type=float, default=0.15)
    parser.add_argument("--scan-lag", type=float, default=0.20)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    result = evaluate(args.bag, args.max_truth_gap, args.scan_lag)
    payload = json.dumps(result, indent=2, sort_keys=True)
    print(payload)
    if args.output:
        output = os.path.abspath(os.path.expanduser(args.output))
        os.makedirs(os.path.dirname(output), exist_ok=True)
        with open(output, "w", encoding="utf-8") as stream:
            stream.write(payload + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
