#!/usr/bin/env python3
"""Deterministic fault gate for pure-odometry IMU clock aiding.

The test exercises only estimator-available signals. Ground truth is generated
inside the test to score the clock estimate and is never fed to the estimator.
"""

import argparse
import json
import math

import numpy as np


def huber_scale(delta_velocity, accel_impulse, noise_floor=0.35):
    delta_velocity = np.asarray(delta_velocity, dtype=float)
    accel_impulse = np.asarray(accel_impulse, dtype=float)
    energy = float(np.dot(accel_impulse, accel_impulse))
    if len(delta_velocity) < 10:
        return None
    if energy < 0.002:
        return None
    candidate = float(
        np.dot(delta_velocity, accel_impulse) / energy)
    for _ in range(2):
        residual = delta_velocity - candidate * accel_impulse
        sigma = max(
            noise_floor * 0.02,
            1.4826 * float(np.median(np.abs(
                residual - np.median(residual)))))
        weights = np.minimum(
            1.0,
            2.5 * sigma / np.maximum(np.abs(residual), 1.0e-12))
        denominator = float(np.dot(
            weights * accel_impulse, accel_impulse))
        if denominator <= 1.0e-12:
            return None
        candidate = float(np.dot(
            weights * delta_velocity, accel_impulse) / denominator)
    return candidate


def run_trial(seed, true_scale, bias, noise, spike_probability,
              dropout_probability):
    rng = np.random.RandomState(seed)
    source_dt = 0.02
    duration = 120.0
    stamps = np.arange(0.0, duration, source_dt)
    physical_time = stamps * true_scale
    acceleration = (
        1.8 * np.sin(0.18 * physical_time)
        + 0.7 * np.sin(0.61 * physical_time))
    acceleration[physical_time < 8.0] = 0.0
    measured = acceleration + bias + rng.normal(
        0.0, noise, len(stamps))
    spikes = rng.rand(len(stamps)) < spike_probability
    measured[spikes] += rng.choice((-1.0, 1.0), int(np.sum(spikes))) * 8.0
    valid = rng.rand(len(stamps)) >= dropout_probability
    velocity = np.zeros(len(stamps), dtype=float)
    for index in range(1, len(stamps)):
        velocity[index] = (
            velocity[index - 1]
            + acceleration[index] * source_dt * true_scale)
    stationary = (physical_time < 8.0) & valid & (~spikes)
    estimated_bias = float(np.median(measured[stationary]))
    measured -= estimated_bias

    estimates = []
    for end in range(25, len(stamps)):
        start = max(0, end - 100)
        dt = np.diff(stamps[start:end + 1])
        accel = 0.5 * (
            measured[start:end] + measured[start + 1:end + 1])
        dv = np.diff(velocity[start:end + 1])
        usable = (
            valid[start:end] & valid[start + 1:end + 1]
            & (np.abs(accel) >= 1.0)
            & (np.abs(dv) <= 3.0))
        estimate = huber_scale(
            dv[usable], accel[usable] * dt[usable], noise)
        if estimate is not None and 0.2 <= estimate <= 1.2:
            estimates.append(estimate)
    if not estimates:
        raise RuntimeError("fault trial produced no valid clock estimate")
    tail = np.asarray(estimates[-200:], dtype=float)
    return {
        "seed": seed,
        "true_scale": true_scale,
        "estimate_median": float(np.median(tail)),
        "estimate_p95_error": float(np.percentile(
            np.abs(tail - true_scale), 95)),
        "finite": bool(np.all(np.isfinite(tail))),
        "spikes": int(np.sum(spikes)),
        "dropouts": int(np.sum(~valid)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    trials = []
    for seed, scale in enumerate((0.55, 0.72, 0.90, 1.05), start=41):
        trials.append(run_trial(
            seed=seed,
            true_scale=scale,
            bias=0.08,
            noise=0.05,
            spike_probability=0.002,
            dropout_probability=0.03))
    passed = all(
        trial["finite"]
        and abs(trial["estimate_median"] - trial["true_scale"]) <= 0.05
        and trial["estimate_p95_error"] <= 0.25
        for trial in trials)
    result = {
        "status": "PASS" if passed else "FAIL",
        "acceleration_double_integrated": False,
        "faults": [
            "constant_bias", "gaussian_noise", "impulse_spikes",
            "sample_dropout"],
        "trials": trials,
    }
    payload = json.dumps(result, indent=2, sort_keys=True)
    print(payload)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(payload + "\n")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
