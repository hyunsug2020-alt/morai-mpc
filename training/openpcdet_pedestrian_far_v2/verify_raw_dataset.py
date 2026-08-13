#!/usr/bin/env python3
"""Verify every raw frame before preparing or training the 30k dataset."""

import argparse
import json
import math
import os
from collections import Counter
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
    parser.add_argument("raw", type=Path)
    parser.add_argument("--target", type=int, default=30000)
    parser.add_argument("--min-ego-ratio", type=float, default=0.85)
    arguments = parser.parse_args()

    labels = {path.stem: path for path in (arguments.raw / "labels").glob("*.json")}
    clouds = {path.stem: path for path in (arguments.raw / "velodyne").glob("*.bin")}
    failures = []
    if len(labels) < arguments.target:
        failures.append("only {} label frames; target is {}".format(len(labels), arguments.target))
    label_only = sorted(set(labels) - set(clouds))
    cloud_only = sorted(set(clouds) - set(labels))
    if label_only:
        failures.append("{} labels have no point cloud".format(len(label_only)))
    if cloud_only:
        failures.append("{} point clouds have no label".format(len(cloud_only)))

    malformed_labels = []
    malformed_clouds = []
    timestamps = []
    ego_frames = 0
    object_frames = 0
    object_count = 0
    distance_bins = Counter()
    point_counts = Counter()
    for frame_id in sorted(set(labels) & set(clouds)):
        point_size = clouds[frame_id].stat().st_size
        if point_size <= 0 or point_size % 16:
            malformed_clouds.append(frame_id)
        try:
            document = json.loads(labels[frame_id].read_text(encoding="utf-8"))
            timestamps.append(int(document["stamp_ns"]))
            ego_frames += int(bool(document.get("ego")))
            pedestrians = [
                obj for obj in document.get("objects", [])
                if obj.get("class_name") == "Pedestrian"
            ]
            object_frames += int(bool(pedestrians))
            for obj in pedestrians:
                center = obj["center"]
                distance = math.hypot(float(center[0]), float(center[1]))
                if distance <= 45.0:
                    lower = int(distance // 5) * 5
                    distance_bins["{:02d}-{:02d}".format(lower, lower + 5)] += 1
                    object_count += 1
                    points = int(obj.get("points_in_box", 0))
                    if points <= 0:
                        point_bucket = "0"
                    elif points == 1:
                        point_bucket = "1"
                    elif points == 2:
                        point_bucket = "2"
                    elif points <= 5:
                        point_bucket = "3-5"
                    elif points <= 10:
                        point_bucket = "6-10"
                    elif points <= 20:
                        point_bucket = "11-20"
                    elif points <= 50:
                        point_bucket = "21-50"
                    else:
                        point_bucket = "51+"
                    point_counts[point_bucket] += 1
        except Exception:
            malformed_labels.append(frame_id)

    if malformed_labels:
        failures.append("{} malformed label files".format(len(malformed_labels)))
    if malformed_clouds:
        failures.append("{} malformed point cloud files".format(len(malformed_clouds)))
    timestamp_regressions = sum(b <= a for a, b in zip(timestamps, timestamps[1:]))
    if timestamp_regressions:
        failures.append("{} non-increasing timestamps".format(timestamp_regressions))
    paired = len(set(labels) & set(clouds))
    ego_ratio = ego_frames / paired if paired else 0.0
    if ego_ratio < arguments.min_ego_ratio:
        failures.append(
            "ego pose ratio {:.3f} is below {:.3f}".format(ego_ratio, arguments.min_ego_ratio)
        )
    populated_bins = sum(count > 0 for count in distance_bins.values())
    if populated_bins < 6:
        failures.append("fewer than six populated 5 m distance bins")
    if object_count <= 0:
        failures.append("no pedestrian objects within 45 m")

    report = {
        "status": "failed" if failures else "passed",
        "target_frames": arguments.target,
        "label_frames": len(labels),
        "cloud_frames": len(clouds),
        "paired_frames": paired,
        "ego_frames": ego_frames,
        "ego_ratio": ego_ratio,
        "object_frames": object_frames,
        "pedestrian_objects_within_45m": object_count,
        "distance_bins_m": dict(sorted(distance_bins.items())),
        "points_in_box_histogram": dict(point_counts),
        "timestamp_regressions": timestamp_regressions,
        "malformed_labels": malformed_labels[:100],
        "malformed_clouds": malformed_clouds[:100],
        "label_only": label_only[:100],
        "cloud_only": cloud_only[:100],
        "failures": failures,
    }
    output = arguments.raw / "raw_verification.json"
    atomic_write(output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if failures:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
