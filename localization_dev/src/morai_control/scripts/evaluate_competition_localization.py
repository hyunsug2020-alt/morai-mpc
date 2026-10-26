#!/usr/bin/env python3
"""Compare Competition-topic-aided ESKF with its absolute GPS input."""

import argparse
import csv
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from tf.transformations import euler_from_quaternion

from eskf import latlon_to_utm


def _stamp(message, record_stamp):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    value = stamp.to_sec() if stamp is not None else 0.0
    return value if value > 0.0 else record_stamp.to_sec()


def _unique(samples):
    values = np.asarray(sorted(samples), dtype=float)
    if not len(values):
        return values
    # ESKF may publish prediction and one or more corrections at the same
    # source timestamp.  The last sample is the finalized posterior state.
    return values[np.r_[np.diff(values[:, 0]) > 1.0e-6, True]]


def _summary(values):
    values = np.asarray(values, dtype=float)
    return {
        "samples": int(len(values)),
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values * values))),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def evaluate(arguments):
    gps = []
    eskf = []
    status = []
    diagnostics = []
    with rosbag.Bag(arguments.bag) as bag:
        for topic, message, record_stamp in bag.read_messages(topics=[
                arguments.gps_topic, arguments.eskf_topic,
                arguments.status_topic, arguments.diagnostics_topic]):
            stamp = _stamp(message, record_stamp)
            if topic == arguments.gps_topic:
                x, y = latlon_to_utm(
                    float(message.latitude), float(message.longitude),
                    arguments.utm_zone)
                gps.append((
                    stamp, x - float(message.eastOffset),
                    y - float(message.northOffset)))
            elif topic == arguments.eskf_topic:
                pose = message.pose.pose
                q = pose.orientation
                yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                covariance = message.pose.covariance
                position_std = math.sqrt(max(
                    0.0, float(covariance[0]) + float(covariance[7])))
                speed = math.hypot(
                    float(message.twist.twist.linear.x),
                    float(message.twist.twist.linear.y))
                eskf.append((stamp, pose.position.x, pose.position.y,
                             yaw, speed, position_std))
            elif topic == arguments.status_topic:
                status.append((stamp, abs(float(message.velocity.x)),
                               float(message.position.x),
                               float(message.position.y)))
            else:
                try:
                    diagnostics.append((
                        record_stamp.to_sec(), json.loads(message.data)))
                except (TypeError, ValueError, json.JSONDecodeError):
                    pass

    gps, eskf, status = map(_unique, (gps, eskf, status))
    if len(gps) < 2 or len(eskf) < 2 or len(status) < 2:
        raise RuntimeError("insufficient GPS, ESKF, or Competition samples")
    indices = np.searchsorted(gps[:, 0], eskf[:, 0], side="right")
    lower = np.clip(indices - 1, 0, len(gps) - 1)
    upper = np.clip(indices, 0, len(gps) - 1)
    gaps = gps[upper, 0] - gps[lower, 0]
    valid = ((indices > 0) & (indices < len(gps)) &
             (gaps > 0.0) & (gaps <= arguments.max_gps_gap))
    fraction = np.zeros(len(eskf))
    fraction[valid] = (
        (eskf[valid, 0] - gps[lower[valid], 0]) / gaps[valid])
    gps_xy = gps[lower, 1:3] + fraction[:, None] * (
        gps[upper, 1:3] - gps[lower, 1:3])

    yaw = eskf[:, 3]
    estimated_antenna = eskf[:, 1:3] + np.column_stack((
        np.cos(yaw) * arguments.gps_lever_x -
        np.sin(yaw) * arguments.gps_lever_y,
        np.sin(yaw) * arguments.gps_lever_x +
        np.cos(yaw) * arguments.gps_lever_y))
    errors_xy = estimated_antenna - gps_xy
    errors = np.linalg.norm(errors_xy, axis=1)

    status_speed = np.interp(eskf[:, 0], status[:, 0], status[:, 1])
    status_position_valid = np.hypot(status[:, 2], status[:, 3]) > 1.0e-6
    status_nearest = np.minimum(
        np.abs(eskf[:, 0] - status[
            np.clip(np.searchsorted(status[:, 0], eskf[:, 0]),
                    0, len(status) - 1), 0]),
        np.abs(eskf[:, 0] - status[
            np.clip(np.searchsorted(status[:, 0], eskf[:, 0]) - 1,
                    0, len(status) - 1), 0]))
    valid &= status_nearest <= arguments.max_status_gap
    selected = np.flatnonzero(valid)
    if len(selected) < 100:
        raise RuntimeError("insufficient timestamp-synchronized samples")
    elapsed = eskf[selected, 0] - eskf[selected[0], 0]
    position_error = errors[selected]
    speed_error = np.abs(eskf[selected, 4] - status_speed[selected])
    moving = status_speed[selected] >= 0.25
    stationary = ~moving

    steps = np.linalg.norm(np.diff(eskf[:, 1:3], axis=0), axis=1)
    step_dt = np.diff(eskf[:, 0])
    expected_steps = eskf[:-1, 4] * np.maximum(step_dt, 0.0)
    excess_steps = np.maximum(0.0, steps - expected_steps)
    gps_steps = np.linalg.norm(np.diff(gps[:, 1:3], axis=0), axis=1)

    gps_modes = {}
    rejected_max = 0
    for _stamp_s, diagnostic in diagnostics:
        mode = str(diagnostic.get("gps_mode", "unknown"))
        gps_modes[mode] = gps_modes.get(mode, 0) + 1
        rejected_max = max(
            rejected_max,
            int((diagnostic.get("counters") or {}).get("gps_rejected", 0)))

    summary = {
        "duration_s": float(eskf[-1, 0] - eskf[0, 0]),
        "source_note": (
            "GPS is an ESKF input, not independent ground truth; metrics are "
            "GPS consistency and continuity, not absolute truth error."),
        "competition_position": {
            "nonzero_samples": int(np.count_nonzero(status_position_valid)),
            "usable_as_ground_truth": bool(np.any(status_position_valid)),
            "note": ("Competition_topic.position is zero in this UDP stream; "
                     "the final launch uses its velocity, not its position.")},
        "raw_samples": {
            "gps": int(len(gps)), "eskf": int(len(eskf)),
            "competition_status": int(len(status)),
            "diagnostics": int(len(diagnostics))},
        "synchronized_samples": int(len(selected)),
        "gps_to_eskf_antenna_error_m": _summary(position_error),
        "moving_error_m": (_summary(position_error[moving])
                           if np.any(moving) else None),
        "stationary_error_m": (_summary(position_error[stationary])
                               if np.any(stationary) else None),
        "competition_to_eskf_speed_abs_error_mps": _summary(speed_error),
        "eskf_position_std_m": _summary(eskf[selected, 5]),
        "eskf_excess_step_m": _summary(excess_steps),
        "raw_gps_step_m": _summary(gps_steps),
        "gps_modes": gps_modes,
        "gps_rejected_counter_max": rejected_max,
        "maximum_competition_speed_mps": float(np.max(status[:, 1])),
        "eskf_start_end_displacement_m": float(np.linalg.norm(
            eskf[-1, 1:3] - eskf[0, 1:3])),
    }

    os.makedirs(arguments.output_dir, exist_ok=True)
    csv_path = os.path.join(arguments.output_dir, "gps_eskf_comparison.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "elapsed_s", "gps_x_m", "gps_y_m", "eskf_gps_x_m",
            "eskf_gps_y_m", "position_error_m", "status_speed_mps",
            "eskf_speed_mps", "speed_abs_error_mps", "position_std_m"])
        for output_index, index in enumerate(selected):
            writer.writerow([
                elapsed[output_index], gps_xy[index, 0], gps_xy[index, 1],
                estimated_antenna[index, 0], estimated_antenna[index, 1],
                errors[index], status_speed[index], eskf[index, 4],
                abs(eskf[index, 4] - status_speed[index]), eskf[index, 5]])

    summary_path = os.path.join(arguments.output_dir, "summary.json")
    with open(summary_path, "w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)

    figure, axes = plt.subplots(2, 2, figsize=(15, 10))
    axes[0, 0].plot(gps_xy[selected, 0], gps_xy[selected, 1],
                    label="GPS antenna", linewidth=1.0)
    axes[0, 0].plot(estimated_antenna[selected, 0],
                    estimated_antenna[selected, 1],
                    label="ESKF + GPS lever arm", linewidth=1.0)
    axes[0, 0].set_title("Trajectory comparison")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(elapsed, position_error, linewidth=0.8)
    axes[0, 1].axhline(summary["gps_to_eskf_antenna_error_m"]["p95"],
                       color="tab:orange", linestyle="--", label="p95")
    axes[0, 1].set_title("GPS - ESKF antenna position difference")
    axes[0, 1].set_ylabel("m")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(elapsed, status_speed[selected],
                    label="Competition status", linewidth=0.8)
    axes[1, 0].plot(elapsed, eskf[selected, 4],
                    label="ESKF", linewidth=0.8, alpha=0.8)
    axes[1, 0].set_title("Speed aiding consistency")
    axes[1, 0].set_xlabel("elapsed s")
    axes[1, 0].set_ylabel("m/s")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(elapsed, eskf[selected, 5],
                    label="ESKF position std", linewidth=0.8)
    axes[1, 1].plot(elapsed, position_error,
                    label="GPS difference", linewidth=0.6, alpha=0.65)
    axes[1, 1].set_title("Filter uncertainty and GPS consistency")
    axes[1, 1].set_xlabel("elapsed s")
    axes[1, 1].set_ylabel("m")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    figure.suptitle("Competition-topic-aided localization: 10-minute run")
    figure.tight_layout()
    plot_path = os.path.join(arguments.output_dir, "localization_10min.png")
    figure.savefig(plot_path, dpi=160)
    plt.close(figure)
    return summary, csv_path, summary_path, plot_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("output_dir")
    parser.add_argument("--utm-zone", type=int, default=52)
    parser.add_argument("--gps-lever-x", type=float, default=3.280)
    parser.add_argument("--gps-lever-y", type=float, default=-0.520)
    parser.add_argument("--max-gps-gap", type=float, default=0.6)
    parser.add_argument("--max-status-gap", type=float, default=0.25)
    parser.add_argument("--gps-topic", default="/gps")
    parser.add_argument("--eskf-topic", default="/eskf/odom")
    parser.add_argument("--status-topic", default="/Competition_topic")
    parser.add_argument("--diagnostics-topic", default="/eskf/diagnostics")
    arguments = parser.parse_args()
    summary, csv_path, summary_path, plot_path = evaluate(arguments)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(csv_path)
    print(summary_path)
    print(plot_path)


if __name__ == "__main__":
    main()
