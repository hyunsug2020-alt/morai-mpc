#!/usr/bin/env python3
"""Wait for and exhaustively verify a paired MORAI LiDAR dataset target."""

import argparse
import json
import os
import time
from collections import Counter
from datetime import datetime
from pathlib import Path


PAIRED_DIRECTORIES = {
    "velodyne": ".bin",
    "labels": ".json",
    "label_lidar": ".txt",
    "timestamps": ".txt",
}


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def frame_stems(path, extension):
    return {
        item.stem for item in path.glob("*" + extension)
        if item.stem.isdigit()
    }


def count_frames(root):
    return len(frame_stems(root / "velodyne", ".bin"))


def verify(root, target):
    expected = {"{:06d}".format(index) for index in range(target)}
    counts = {}
    missing = {}
    extra = {}
    for directory, extension in PAIRED_DIRECTORIES.items():
        stems = frame_stems(root / directory, extension)
        counts[directory] = len(stems)
        missing[directory] = sorted(expected - stems)
        extra[directory] = sorted(stems - expected)

    corrupt_bins = []
    invalid_labels = []
    category_objects = Counter()
    class_objects = Counter()
    frames_by_category = Counter()
    total_objects = 0
    total_points = 0

    for stem in sorted(expected):
        point_path = root / "velodyne" / (stem + ".bin")
        if point_path.exists():
            size = point_path.stat().st_size
            if size == 0 or size % 16 != 0:
                corrupt_bins.append(stem)
            else:
                total_points += size // 16

        label_path = root / "labels" / (stem + ".json")
        if not label_path.exists():
            continue
        try:
            document = json.loads(label_path.read_text(encoding="utf-8"))
            if document.get("frame_id") != stem:
                raise ValueError("frame_id mismatch")
            objects = document.get("objects", [])
            seen_categories = set()
            for item in objects:
                category = str(item.get("category", ""))
                class_name = str(item.get("class_name", ""))
                points_in_box = int(item.get("points_in_box", 0))
                if not category or not class_name or points_in_box < 5:
                    raise ValueError("invalid object label")
                category_objects[category] += 1
                class_objects[class_name] += 1
                seen_categories.add(category)
                total_objects += 1
            for category in seen_categories:
                frames_by_category[category] += 1
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
            invalid_labels.append({"frame_id": stem, "error": str(error)})

    temporary_files = sorted(
        str(path.relative_to(root)) for path in root.rglob("*.tmp")
    )
    required_categories = {"pedestrian", "obstacle"}
    present_categories = set(category_objects)
    complete = bool(
        all(not values for values in missing.values())
        and not corrupt_bins
        and not invalid_labels
        and required_categories.issubset(present_categories)
    )
    return {
        "status": "complete" if complete else "failed_verification",
        "verified_at": datetime.now().astimezone().isoformat(),
        "target_frames": target,
        "paired_file_counts": counts,
        "missing_counts": {
            key: len(value) for key, value in missing.items()
        },
        "missing_examples": {
            key: value[:20] for key, value in missing.items() if value
        },
        "extra_counts": {key: len(value) for key, value in extra.items()},
        "corrupt_bin_count": len(corrupt_bins),
        "corrupt_bin_examples": corrupt_bins[:20],
        "invalid_label_count": len(invalid_labels),
        "invalid_label_examples": invalid_labels[:20],
        "category_object_counts": dict(category_objects),
        "class_object_counts": dict(class_objects),
        "frames_by_category": dict(frames_by_category),
        "total_labeled_objects": total_objects,
        "total_lidar_points": total_points,
        "temporary_files_seen": temporary_files,
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--target", type=int, default=30000)
    parser.add_argument("--poll-seconds", type=float, default=10.0)
    parser.add_argument("--deadline", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    root = args.dataset.resolve()
    deadline = datetime.fromisoformat(args.deadline)
    report_path = root / "collection_30000_report.json"
    while datetime.now().astimezone() < deadline:
        current = count_frames(root)
        if current >= args.target:
            # Let the writer finish the last atomic rename before auditing.
            time.sleep(5.0)
            report = verify(root, args.target)
            atomic_json(report_path, report)
            if report["status"] == "complete":
                return
            raise RuntimeError("30,000-frame verification failed")
        print("dataset progress: {}/{}".format(
            current, args.target), flush=True)
        time.sleep(args.poll_seconds)
    raise RuntimeError("dataset deadline reached before target")


if __name__ == "__main__":
    main()
