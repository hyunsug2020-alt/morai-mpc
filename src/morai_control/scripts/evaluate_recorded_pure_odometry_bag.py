#!/usr/bin/env python3
"""Score recorded pure odometry without feeding truth to the estimator.

The pure odometry frame is local, so each physically continuous MORAI truth
segment receives one rigid SE(2) alignment at its first synchronized sample.
Large, speed-inconsistent simulator teleports split segments and are reported
explicitly instead of being counted as dead-reckoning error.
"""

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
        "mean_abs": float(np.mean(np.abs(values))),
        "rmse": float(np.sqrt(np.mean(values ** 2))),
        "p95_abs": float(np.percentile(np.abs(values), 95)),
        "max_abs": float(np.max(np.abs(values))),
    }


def unique_sorted(rows):
    rows = sorted(rows, key=lambda row: row[0])
    return np.asarray([
        row for index, row in enumerate(rows)
        if index == 0 or row[0] > rows[index - 1][0]
    ], dtype=float)


def load_bag(path, odometry_topic):
    truth = []
    odometry = []
    diagnostics = []
    with rosbag.Bag(path) as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=[
                "/Ego_topic", odometry_topic,
                "/pure_odometry/diagnostics"]):
            if topic == "/pure_odometry/diagnostics":
                try:
                    diagnostics.append((
                        bag_stamp.to_sec(), json.loads(message.data)))
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
                    float(message.velocity.x),
                ))
            else:
                orientation = message.pose.pose.orientation
                yaw = euler_from_quaternion([
                    orientation.x, orientation.y,
                    orientation.z, orientation.w])[2]
                odometry.append((
                    stamp,
                    float(message.pose.pose.position.x),
                    float(message.pose.pose.position.y),
                    yaw,
                    float(message.twist.twist.linear.x),
                ))
    if len(truth) < 2 or len(odometry) < 2:
        raise RuntimeError(
            "bag needs /Ego_topic and /odometry/pure samples")
    return unique_sorted(truth), unique_sorted(odometry), diagnostics


def interpolate_truth(truth, stamps, max_gap):
    upper = np.searchsorted(truth[:, 0], stamps, side="right")
    valid = (upper > 0) & (upper < len(truth))
    lower = np.clip(upper - 1, 0, len(truth) - 1)
    upper = np.clip(upper, 0, len(truth) - 1)
    gaps = truth[upper, 0] - truth[lower, 0]
    valid &= (gaps > 0.0) & (gaps <= max_gap)
    fraction = np.zeros(len(stamps), dtype=float)
    fraction[valid] = (
        (stamps[valid] - truth[lower[valid], 0]) / gaps[valid])
    values = np.empty((len(stamps), 4), dtype=float)
    values[:, 0] = (
        truth[lower, 1]
        + fraction * (truth[upper, 1] - truth[lower, 1]))
    values[:, 1] = (
        truth[lower, 2]
        + fraction * (truth[upper, 2] - truth[lower, 2]))
    unwrapped_yaw = np.unwrap(truth[:, 3])
    values[:, 2] = (
        unwrapped_yaw[lower]
        + fraction * (unwrapped_yaw[upper] - unwrapped_yaw[lower]))
    values[:, 3] = (
        truth[lower, 4]
        + fraction * (truth[upper, 4] - truth[lower, 4]))
    return valid, values


def align_segment(odometry, truth):
    yaw_offset = float(wrap_angle(truth[0, 2] - odometry[0, 3]))
    cosine = math.cos(yaw_offset)
    sine = math.sin(yaw_offset)
    rotated_x = cosine * odometry[:, 1] - sine * odometry[:, 2]
    rotated_y = sine * odometry[:, 1] + cosine * odometry[:, 2]
    translation = np.array([
        truth[0, 0] - rotated_x[0],
        truth[0, 1] - rotated_y[0],
    ])
    aligned = np.empty((len(odometry), 4), dtype=float)
    aligned[:, 0] = rotated_x + translation[0]
    aligned[:, 1] = rotated_y + translation[1]
    aligned[:, 2] = wrap_angle(odometry[:, 3] + yaw_offset)
    aligned[:, 3] = odometry[:, 4]
    return aligned, {
        "translation_xy_m": translation.tolist(),
        "yaw_deg": math.degrees(yaw_offset),
    }


def metrics(aligned, truth, truth_distance):
    delta_x = aligned[:, 0] - truth[:, 0]
    delta_y = aligned[:, 1] - truth[:, 1]
    position = np.hypot(delta_x, delta_y)
    yaw = wrap_angle(aligned[:, 2] - truth[:, 2])
    speed = aligned[:, 3] - truth[:, 3]
    cosine = np.cos(truth[:, 2])
    sine = np.sin(truth[:, 2])
    longitudinal = cosine * delta_x + sine * delta_y
    lateral = -sine * delta_x + cosine * delta_y
    endpoint = float(position[-1])
    return {
        "position_m": summarize(position),
        "yaw_deg": summarize(np.degrees(yaw)),
        "speed_mps": summarize(speed),
        "longitudinal_m": summarize(longitudinal),
        "lateral_m": summarize(lateral),
        "truth_distance_m": float(truth_distance),
        "endpoint_error_m": endpoint,
        "endpoint_drift_percent_distance": (
            100.0 * endpoint / max(float(truth_distance), 1.0)),
    }, position, yaw, speed, longitudinal, lateral


def rolling_drift(odometry, truth, duration_sec, step_sec=5.0):
    """Score local drift after a fresh alignment at each shadow start."""
    position_errors = []
    yaw_errors = []
    drift_percent = []
    truth_distances = []
    next_start_stamp = float(odometry[0, 0])
    for start_index in range(len(odometry) - 1):
        start_stamp = float(odometry[start_index, 0])
        if start_stamp + 1.0e-9 < next_start_stamp:
            continue
        end_index = int(np.searchsorted(
            odometry[:, 0], start_stamp + duration_sec, side="left"))
        if end_index >= len(odometry):
            break
        actual_duration = odometry[end_index, 0] - start_stamp
        if actual_duration > duration_sec + 0.25:
            next_start_stamp = start_stamp + step_sec
            continue
        yaw_offset = float(wrap_angle(
            truth[start_index, 2] - odometry[start_index, 3]))
        cosine = math.cos(yaw_offset)
        sine = math.sin(yaw_offset)
        start_rotated = np.array([
            cosine * odometry[start_index, 1]
            - sine * odometry[start_index, 2],
            sine * odometry[start_index, 1]
            + cosine * odometry[start_index, 2],
        ])
        translation = truth[start_index, 0:2] - start_rotated
        end_position = np.array([
            cosine * odometry[end_index, 1]
            - sine * odometry[end_index, 2],
            sine * odometry[end_index, 1]
            + cosine * odometry[end_index, 2],
        ]) + translation
        position_error = float(np.linalg.norm(
            end_position - truth[end_index, 0:2]))
        yaw_error = float(wrap_angle(
            odometry[end_index, 3] + yaw_offset
            - truth[end_index, 2]))
        truth_distance = float(np.sum(np.hypot(
            np.diff(truth[start_index:end_index + 1, 0]),
            np.diff(truth[start_index:end_index + 1, 1]))))
        position_errors.append(position_error)
        yaw_errors.append(math.degrees(yaw_error))
        truth_distances.append(truth_distance)
        drift_percent.append(
            100.0 * position_error / max(truth_distance, 1.0))
        next_start_stamp = start_stamp + step_sec
    return position_errors, yaw_errors, drift_percent, truth_distances


def evaluate(path, max_truth_gap, teleport_distance, odometry_topic):
    truth_raw, odometry, diagnostics = load_bag(path, odometry_topic)
    truth_steps = np.hypot(
        np.diff(truth_raw[:, 1]), np.diff(truth_raw[:, 2]))
    truth_dt = np.diff(truth_raw[:, 0])
    reported_motion = (
        0.5 * (
            np.abs(truth_raw[:-1, 4])
            + np.abs(truth_raw[1:, 4]))
        * np.maximum(truth_dt, 0.0))
    teleport_indices = np.flatnonzero(
        (truth_steps > teleport_distance)
        & (truth_steps > reported_motion + teleport_distance * 0.5))
    teleports = [{
        "elapsed_sec": float(
            truth_raw[index + 1, 0] - truth_raw[0, 0]),
        "distance_m": float(truth_steps[index]),
        "dt_sec": float(truth_dt[index]),
        "reported_motion_m": float(reported_motion[index]),
        "before_xy": truth_raw[index, 1:3].tolist(),
        "after_xy": truth_raw[index + 1, 1:3].tolist(),
    } for index in teleport_indices]
    boundaries = [truth_raw[0, 0]] + [
        truth_raw[index + 1, 0] for index in teleport_indices
    ] + [truth_raw[-1, 0] + 1.0e-9]

    segment_reports = []
    combined_errors = [[], [], [], [], []]
    rolling_values = {
        duration: [[], [], [], []]
        for duration in (10.0, 30.0, 60.0, 120.0)
    }
    total_distance = 0.0
    for segment_index in range(len(boundaries) - 1):
        start = boundaries[segment_index]
        end = boundaries[segment_index + 1]
        mask = (odometry[:, 0] >= start) & (odometry[:, 0] < end)
        segment_odometry = odometry[mask]
        if len(segment_odometry) < 20:
            continue
        valid, segment_truth = interpolate_truth(
            truth_raw, segment_odometry[:, 0], max_truth_gap)
        segment_odometry = segment_odometry[valid]
        segment_truth = segment_truth[valid]
        if len(segment_odometry) < 20:
            continue
        truth_distance = float(np.sum(np.hypot(
            np.diff(segment_truth[:, 0]),
            np.diff(segment_truth[:, 1]))))
        aligned, alignment = align_segment(
            segment_odometry, segment_truth)
        segment_metrics, *errors = metrics(
            aligned, segment_truth, truth_distance)
        segment_reports.append({
            "segment": segment_index,
            "start_elapsed_sec": float(
                segment_odometry[0, 0] - odometry[0, 0]),
            "duration_sec": float(
                segment_odometry[-1, 0] - segment_odometry[0, 0]),
            "samples": int(len(segment_odometry)),
            "alignment_validation_only": alignment,
            **segment_metrics,
        })
        total_distance += truth_distance
        for destination, values in zip(combined_errors, errors):
            destination.extend(np.asarray(values, dtype=float).tolist())
        for duration, destinations in rolling_values.items():
            values = rolling_drift(
                segment_odometry, segment_truth, duration)
            for destination, samples in zip(destinations, values):
                destination.extend(samples)

    if not segment_reports:
        raise RuntimeError("no synchronized continuous odometry segments")
    position, yaw, speed, longitudinal, lateral = [
        np.asarray(values, dtype=float) for values in combined_errors]
    motion_scales = [
        float(payload["motion_rate_scale"])
        for _, payload in diagnostics
        if isinstance(payload, dict)
        and isinstance(payload.get("motion_rate_scale"), (int, float))
        and math.isfinite(float(payload["motion_rate_scale"]))
    ]
    longest = max(segment_reports, key=lambda row: row["duration_sec"])
    aggregate = {
        "continuous_segment_count": len(segment_reports),
        "samples": int(len(position)),
        "truth_distance_m": total_distance,
        "position_m": summarize(position),
        "yaw_deg": summarize(np.degrees(yaw)),
        "speed_mps": summarize(speed),
        "longitudinal_m": summarize(longitudinal),
        "lateral_m": summarize(lateral),
        "longest_segment_duration_sec": longest["duration_sec"],
        "longest_segment_endpoint_error_m": longest["endpoint_error_m"],
        "longest_segment_endpoint_drift_percent_distance": (
            longest["endpoint_drift_percent_distance"]),
    }
    rolling_report = {}
    for duration, values in rolling_values.items():
        position_values, yaw_values, drift_values, distances = values
        rolling_report["{}s".format(int(duration))] = {
            "windows": len(position_values),
            "endpoint_position_error_m": summarize(position_values),
            "endpoint_yaw_error_deg": summarize(yaw_values),
            "endpoint_drift_percent_distance": summarize(drift_values),
            "truth_distance_m": summarize(distances),
            "window_step_sec": 5.0,
        }
    return {
        "bag": os.path.abspath(path),
        "validation_only_ground_truth": True,
        "estimator_topic": odometry_topic,
        "estimator_inputs": [
            "EgoVehicleStatus.velocity.x",
            "EgoVehicleStatus.wheel_angle",
            "Imu.orientation/angular_velocity.z/linear_acceleration.x",
        ],
        "uses_gps": False,
        "uses_lidar": False,
        "synchronization": "linear interpolation by ROS source stamp",
        "continuous_segment_alignment": (
            "one initial rigid transform per simulator-continuous segment"),
        "simulator_teleports": teleports,
        "motion_rate_scale": summarize(motion_scales),
        "segments": segment_reports,
        "aggregate": aggregate,
        "rolling_gps_shadow_drift": rolling_report,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--max-truth-gap", type=float, default=0.15)
    parser.add_argument("--teleport-distance", type=float, default=20.0)
    parser.add_argument(
        "--odometry-topic", default="/odometry/pure")
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    result = evaluate(
        args.bag, args.max_truth_gap, args.teleport_distance,
        args.odometry_topic)
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
