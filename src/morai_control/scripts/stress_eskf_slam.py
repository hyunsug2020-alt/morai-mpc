#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_eskf_slam import run_trial  # noqa: E402


def random_intervals(rng, duration, count, min_length, max_length,
                     earliest=10.0):
    intervals = []
    for _ in range(count):
        length = float(rng.uniform(min_length, max_length))
        latest_start = max(earliest, duration - length - 2.0)
        start = float(rng.uniform(earliest, latest_start))
        intervals.append((start, min(duration, start + length)))
    return tuple(sorted(intervals))


def random_scenario(seed):
    rng = np.random.default_rng(seed)
    duration = float(rng.uniform(100.0, 190.0))
    gps_dropout_count = int(rng.integers(1, 4))
    slam_dropout_count = int(rng.integers(0, 3))
    slam_degenerate_count = int(rng.integers(0, 3))
    gps_dropouts = random_intervals(
        rng, duration, gps_dropout_count, 8.0, 65.0, earliest=12.0)
    if rng.random() < 0.15:
        initial_end = float(rng.uniform(2.0, 12.0))
        gps_dropouts = tuple(sorted(((0.0, initial_end),) + gps_dropouts))

    gps_std = float(rng.uniform(0.3, 3.0))
    accel_bias = rng.uniform(-0.45, 0.45, 2)
    frame_resets = []
    if rng.random() < 0.35:
        frame_resets.append({
            "time_sec": float(rng.uniform(20.0, duration - 10.0)),
            "yaw_delta": float(rng.uniform(-0.6, 0.6)),
            "translation_delta": rng.uniform(-30.0, 30.0, 2).tolist(),
        })
    scenario = {
        "duration": duration,
        "dt": 0.05,
        "gps_dropouts": gps_dropouts,
        "slam_dropouts": random_intervals(
            rng, duration, slam_dropout_count, 2.0, 12.0, earliest=15.0),
        "slam_degenerate": random_intervals(
            rng, duration, slam_degenerate_count, 2.0, 12.0, earliest=15.0),
        "gps_std": gps_std,
        "gps_outlier_probability": float(rng.uniform(0.0, 0.12)),
        "gps_outlier_min_m": float(rng.uniform(8.0, 25.0)),
        "gps_outlier_max_m": float(rng.uniform(35.0, 100.0)),
        "accel_std": float(rng.uniform(0.05, 0.7)),
        "gyro_std": float(rng.uniform(0.002, 0.035)),
        "yaw_std": float(rng.uniform(0.01, 0.15)),
        "imu_spike_probability": float(rng.uniform(0.0, 0.02)),
        "imu_spike_accel_std": float(rng.uniform(5.0, 30.0)),
        "imu_spike_gyro_std": float(rng.uniform(0.3, 2.5)),
        "yaw_outlier_std": float(rng.uniform(0.3, 2.0)),
        "accel_bias": accel_bias.tolist(),
        "gyro_bias": float(rng.uniform(-0.025, 0.025)),
        "local_to_global_yaw": float(rng.uniform(-math.pi, math.pi)),
        "local_to_global_translation": rng.uniform(-150.0, 150.0, 2).tolist(),
        "slam_position_noise_std": float(rng.uniform(0.01, 0.20)),
        "slam_yaw_noise_std": float(rng.uniform(0.001, 0.02)),
        "slam_drift_std_per_update": float(rng.uniform(0.001, 0.02)),
        "slam_drift_scale": float(rng.uniform(-0.003, 0.008)),
        "slam_yaw_drift_std_per_update": float(
            rng.uniform(0.00005, 0.001)),
        "slam_degenerate_position_drift_std": float(
            rng.uniform(0.01, 0.15)),
        "slam_degenerate_yaw_drift_std": float(
            rng.uniform(0.0005, 0.01)),
        "slam_outlier_probability": float(rng.uniform(0.0, 0.05)),
        "slam_outlier_position_std": float(rng.uniform(3.0, 40.0)),
        "slam_outlier_yaw_std": float(rng.uniform(0.2, 2.0)),
        "slam_timestamp_drop_probability": float(rng.uniform(0.0, 0.15)),
        "slam_message_delay_sec": float(rng.uniform(0.0, 0.42)),
        "slam_message_delay_jitter_std": float(rng.uniform(0.0, 0.04)),
        "slam_max_message_lag_sec": 0.5,
        "slam_frame_resets": frame_resets,
    }
    return scenario


def evaluate(result, scenario):
    reasons = []
    if not result.get("initialized", False):
        reasons.append("not_initialized")
        return reasons, math.inf
    if not result.get("state_finite", False):
        reasons.append("non_finite_state")
    if result.get("covariance_min_eigenvalue", -1.0) < -1e-8:
        reasons.append("non_psd_covariance")

    gps_std = scenario["gps_std"]
    rmse_limit = max(5.0, 2.5 * gps_std)
    outage_p95_limit = max(8.0, 4.0 * gps_std)
    max_error_limit = max(20.0, 8.0 * gps_std)
    slam_rmse = result.get("slam_rmse_m", math.inf)
    slam_p95 = result.get("slam_outage_p95_m", math.inf)
    slam_max = result.get("slam_max_m", math.inf)
    if slam_rmse > rmse_limit:
        reasons.append("rmse_limit")
    if slam_p95 > outage_p95_limit:
        reasons.append("outage_p95_limit")
    if slam_max > max_error_limit:
        reasons.append("max_error_limit")

    no_slam_rmse = result.get("no_slam_rmse_m", math.inf)
    if no_slam_rmse > 2.0 and slam_rmse > no_slam_rmse * 0.9:
        reasons.append("slam_no_improvement")
    score = max(
        slam_rmse / rmse_limit,
        slam_p95 / outage_p95_limit,
        slam_max / max_error_limit)
    return reasons, score


def write_results(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=".eskf_stress_", suffix=".json", dir=directory)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary_path, path)
    finally:
        if os.path.exists(temporary_path):
            os.unlink(temporary_path)


def main():
    parser = argparse.ArgumentParser(
        description="Randomized long-run stress test for LIO-aided ESKF")
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--seed", type=int, default=20260728)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument(
        "--results", default="/tmp/eskf_slam_stress_latest.json")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument("--progress-every", type=int, default=10)
    args = parser.parse_args()

    started = time.monotonic()
    failures = []
    worst_cases = []
    completed = 0
    for iteration in range(args.iterations):
        if args.seconds > 0.0 and time.monotonic() - started >= args.seconds:
            break
        seed = args.seed + iteration
        scenario = random_scenario(seed)
        try:
            result = run_trial(seed, scenario)
            reasons, score = evaluate(result, scenario)
        except Exception as exc:
            result = {"exception": repr(exc)}
            reasons = ["exception"]
            score = math.inf
        completed += 1
        case = {
            "seed": seed,
            "score": score,
            "reasons": reasons,
            "result": result,
            "scenario": scenario,
        }
        if reasons:
            try:
                traced_result = run_trial(
                    seed, scenario, record_trace=True)
                result["trace"] = traced_result.get("trace", [])
            except Exception as trace_exc:
                result["trace_exception"] = repr(trace_exc)
            failures.append(case)
        worst_cases.append(case)
        worst_cases = sorted(
            worst_cases,
            key=lambda item: item["score"],
            reverse=True)[:10]
        payload = {
            "completed": completed,
            "elapsed_sec": time.monotonic() - started,
            "failure_count": len(failures),
            "failures": failures[-20:],
            "seed_start": args.seed,
            "worst_cases": worst_cases,
        }
        write_results(args.results, payload)
        if (
                args.progress_every > 0
                and completed % args.progress_every == 0):
            print(
                "completed={} failures={} worst_score={:.3f}".format(
                    completed,
                    len(failures),
                    worst_cases[0]["score"]),
                flush=True)
        if reasons and args.stop_on_failure:
            print(
                "FAIL seed={} reasons={}".format(seed, ",".join(reasons)),
                flush=True)
            return 1

    print(
        "STRESS {}: completed={} failures={} elapsed={:.1f}s result={}".format(
            "PASS" if not failures else "FAIL",
            completed,
            len(failures),
            time.monotonic() - started,
            args.results))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
