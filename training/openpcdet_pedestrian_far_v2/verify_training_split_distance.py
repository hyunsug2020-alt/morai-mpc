#!/usr/bin/env python3
"""Verify that the actual OpenPCDet train split spans near, mid, and far ranges."""

import argparse
import json
import math
import os
import pickle
from pathlib import Path


BANDS = (
    ("near_0_15m", 0.0, 15.0),
    ("mid_15_30m", 15.0, 30.0),
    ("far_30_45m", 30.0, 45.0),
)
FINE_BINS = tuple(range(0, 46, 5))


def atomic_write(path, payload):
    temporary = Path(str(path) + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(str(temporary), str(path))


def summarize(info_path):
    with info_path.open("rb") as stream:
        infos = pickle.load(stream)

    object_bins = {
        "{:02d}-{:02d}m".format(lo, hi): 0
        for lo, hi in zip(FINE_BINS[:-1], FINE_BINS[1:])
    }
    band_objects = {name: 0 for name, _lo, _hi in BANDS}
    frames_with_band = {name: 0 for name, _lo, _hi in BANDS}
    total_objects = 0

    for info in infos:
        boxes = info.get("annos", {}).get("gt_boxes_lidar", ())
        distances = [math.hypot(float(box[0]), float(box[1])) for box in boxes]
        total_objects += len(distances)
        for distance in distances:
            for lo, hi in zip(FINE_BINS[:-1], FINE_BINS[1:]):
                if lo <= distance < hi:
                    object_bins["{:02d}-{:02d}m".format(lo, hi)] += 1
                    break
        for name, lo, hi in BANDS:
            count = sum(lo <= distance < hi for distance in distances)
            band_objects[name] += count
            frames_with_band[name] += int(count > 0)

    return {
        "frames": len(infos),
        "objects": total_objects,
        "object_bins": object_bins,
        "band_objects": band_objects,
        "frames_with_band": frames_with_band,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--min-train-frames", type=int, default=30000)
    parser.add_argument("--min-near-objects", type=int, default=10000)
    parser.add_argument("--min-mid-objects", type=int, default=10000)
    parser.add_argument("--min-far-objects", type=int, default=1000)
    parser.add_argument("--min-far-frames", type=int, default=1000)
    args = parser.parse_args()

    train = summarize(args.dataset / "custom_infos_train.pkl")
    validation = summarize(args.dataset / "custom_infos_val.pkl")
    minimums = {
        "train_frames": args.min_train_frames,
        "near_objects": args.min_near_objects,
        "mid_objects": args.min_mid_objects,
        "far_objects": args.min_far_objects,
        "far_frames": args.min_far_frames,
    }
    failures = []
    if train["frames"] < args.min_train_frames:
        failures.append("train_frames")
    if train["band_objects"]["near_0_15m"] < args.min_near_objects:
        failures.append("near_objects")
    if train["band_objects"]["mid_15_30m"] < args.min_mid_objects:
        failures.append("mid_objects")
    if train["band_objects"]["far_30_45m"] < args.min_far_objects:
        failures.append("far_objects")
    if train["frames_with_band"]["far_30_45m"] < args.min_far_frames:
        failures.append("far_frames")

    report = {
        "status": "passed" if not failures else "failed",
        "failures": failures,
        "minimums": minimums,
        "train": train,
        "validation": validation,
    }
    output = args.dataset / "training_split_distance_report.json"
    atomic_write(output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True), flush=True)
    if failures:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
