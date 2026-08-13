#!/usr/bin/env python3
"""Fail before training when the 30k dataset lacks site or range diversity."""

import argparse
import json
import os
from pathlib import Path


def atomic_write(path, document):
    temporary = Path(str(path) + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("summary", type=Path)
    parser.add_argument("--min-total-frames", type=int, default=30000)
    parser.add_argument("--min-train-frames", type=int, default=30000)
    parser.add_argument("--min-site-frames", type=int, default=250)
    parser.add_argument("--min-band-objects", type=int, default=100)
    arguments = parser.parse_args()

    summary = json.loads(arguments.summary.read_text(encoding="utf-8"))
    bins = summary.get("distance_bins_m", {})
    band_objects = {"near_0_15m": 0, "mid_15_30m": 0, "far_30_45m": 0}
    for label, count in bins.items():
        lower = int(label.split("-", 1)[0])
        if lower < 15:
            band_objects["near_0_15m"] += int(count)
        elif lower < 30:
            band_objects["mid_15_30m"] += int(count)
        elif lower < 45:
            band_objects["far_30_45m"] += int(count)

    expected_sites = ["site_{}".format(number) for number in range(1, 6)]
    site_frames = summary.get("site_frames", {})
    failures = []
    if int(summary.get("frames", 0)) < arguments.min_total_frames:
        failures.append(
            "only {} prepared frames; requires {}".format(
                summary.get("frames", 0), arguments.min_total_frames
            )
        )
    if int(summary.get("train_frames", 0)) < arguments.min_train_frames:
        failures.append(
            "only {} training frames; requires {}".format(
                summary.get("train_frames", 0), arguments.min_train_frames
            )
        )
    for site in expected_sites:
        if int(site_frames.get(site, 0)) < arguments.min_site_frames:
            failures.append(
                "{} has {} frames; requires {}".format(
                    site, site_frames.get(site, 0), arguments.min_site_frames
                )
            )
    for band, count in band_objects.items():
        if count < arguments.min_band_objects:
            failures.append(
                "{} has {} objects; requires {}".format(
                    band, count, arguments.min_band_objects
                )
            )
    if int(summary.get("scene_count", 0)) < 10:
        failures.append("fewer than 10 moving scenes")
    if int(summary.get("train_frames", 0)) <= 0 or int(summary.get("validation_frames", 0)) <= 0:
        failures.append("train or validation split is empty")

    report = {
        "status": "failed" if failures else "passed",
        "site_frames": site_frames,
        "site_objects": summary.get("site_objects", {}),
        "distance_bins_m": bins,
        "distance_bands": band_objects,
        "requirements": {
            "min_total_frames": arguments.min_total_frames,
            "min_train_frames": arguments.min_train_frames,
            "sites": expected_sites,
            "min_site_frames": arguments.min_site_frames,
            "min_band_objects": arguments.min_band_objects,
            "min_scenes": 10,
        },
        "failures": failures,
    }
    output = arguments.summary.parent / "coverage_report.json"
    atomic_write(output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if failures:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
