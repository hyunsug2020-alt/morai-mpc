#!/usr/bin/env python3
"""Convert the verified MORAI pedestrian dataset to OpenPCDet CustomDataset."""

import argparse
import json
import math
import os
from collections import Counter
from pathlib import Path

import numpy as np


CLASS_NAMES = ("Pedestrian", "Obstacle")


def atomic_write(path, content):
    temporary = Path(str(path) + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def load_frame_ids(source, target_frames):
    frames = []
    with (source / "manifest.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                frames.append(str(json.loads(line)["frame_id"]))
            if len(frames) == target_frames:
                break
    if len(frames) != target_frames:
        raise RuntimeError(
            "verified target frames are unavailable: {} != {}".format(
                len(frames), target_frames
            )
        )
    expected = ["{:06d}".format(index) for index in range(target_frames)]
    if frames != expected:
        raise RuntimeError("manifest is not contiguous from frame 000000")
    return frames


def create_point_link(source, output):
    target = (source / "velodyne").resolve()
    link = output / "points"
    if link.is_symlink():
        if link.resolve() != target:
            raise RuntimeError("existing points symlink has a different target")
        return
    if link.exists():
        raise RuntimeError("output points path exists and is not a symlink")
    link.symlink_to(target, target_is_directory=True)


def strict_point_count(points, center, size, yaw):
    delta_x = points[:, 0] - center[0]
    delta_y = points[:, 1] - center[1]
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    box_x = cosine * delta_x + sine * delta_y
    box_y = -sine * delta_x + cosine * delta_y
    inside = (
        (np.abs(box_x) <= 0.5 * size[0])
        & (np.abs(box_y) <= 0.5 * size[1])
        & (np.abs(points[:, 2] - center[2]) <= 0.5 * size[2])
    )
    return int(np.count_nonzero(inside))


def convert_label(source_path, point_path, output_path, min_strict_points):
    document = json.loads(source_path.read_text(encoding="utf-8"))
    flat_points = np.fromfile(point_path, dtype=np.float32)
    if flat_points.size % 4:
        raise RuntimeError("invalid x/y/z/intensity file: {}".format(point_path))
    points = flat_points.reshape((-1, 4))
    kept = Counter()
    source = Counter()
    filtered = Counter()
    lines = []
    for obj in document.get("objects", []):
        class_name = str(obj.get("class_name", ""))
        if class_name not in CLASS_NAMES:
            continue
        source[class_name] += 1
        center = np.asarray(obj["center"], dtype=np.float64)
        size = np.asarray(obj["size"], dtype=np.float64)
        yaw = float(obj["yaw_rad"])
        if strict_point_count(points, center, size, yaw) < min_strict_points:
            filtered[class_name] += 1
            continue
        values = list(center) + list(size) + [yaw]
        lines.append(
            "{} {}".format(
                " ".join("{:.9f}".format(value) for value in values),
                class_name,
            )
        )
        kept[class_name] += 1
    atomic_write(output_path, "\n".join(lines) + ("\n" if lines else ""))
    return kept, source, filtered


def prepare(source, output, target_frames, validation_frames, min_strict_points):
    source = source.resolve()
    output.mkdir(parents=True, exist_ok=True)
    labels_dir = output / "labels"
    image_sets_dir = output / "ImageSets"
    labels_dir.mkdir(exist_ok=True)
    image_sets_dir.mkdir(exist_ok=True)
    create_point_link(source, output)

    frames = load_frame_ids(source, target_frames)
    if not 1 <= validation_frames < target_frames:
        raise ValueError("validation_frames must be within the target range")
    split_at = target_frames - validation_frames
    train = frames[:split_at]
    validation = frames[split_at:]
    kept_total = Counter()
    source_total = Counter()
    filtered_total = Counter()
    empty_frames = 0
    for position, frame_id in enumerate(frames, 1):
        kept, source_count, filtered = convert_label(
            source / "labels" / (frame_id + ".json"),
            source / "velodyne" / (frame_id + ".bin"),
            labels_dir / (frame_id + ".txt"),
            min_strict_points,
        )
        kept_total.update(kept)
        source_total.update(source_count)
        filtered_total.update(filtered)
        empty_frames += int(sum(kept.values()) == 0)
        if position % 2000 == 0 or position == len(frames):
            print("converted {}/{}".format(position, len(frames)), flush=True)

    atomic_write(image_sets_dir / "train.txt", "\n".join(train) + "\n")
    atomic_write(image_sets_dir / "val.txt", "\n".join(validation) + "\n")
    atomic_write(image_sets_dir / "all.txt", "\n".join(frames) + "\n")
    summary = {
        "status": "complete",
        "source": str(source),
        "output": str(output.resolve()),
        "target_frames": target_frames,
        "train_frames": len(train),
        "validation_frames": len(validation),
        "train_range": [train[0], train[-1]],
        "validation_range": [validation[0], validation[-1]],
        "split_method": "final contiguous sequence holdout",
        "classes": list(CLASS_NAMES),
        "kept_objects": dict(kept_total),
        "source_objects": dict(source_total),
        "filtered_objects": dict(filtered_total),
        "empty_frames": empty_frames,
        "min_strict_points_per_box": min_strict_points,
    }
    atomic_write(
        output / "preparation_summary.json",
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )
    print(json.dumps(summary, ensure_ascii=False), flush=True)


def main():
    repository = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=repository / "datasets" / "morai_lidar_pedestrian_20260801",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=repository / "datasets" / "morai_pedestrian_openpcdet",
    )
    parser.add_argument("--target-frames", type=int, default=30000)
    parser.add_argument("--validation-frames", type=int, default=6000)
    parser.add_argument("--min-strict-points", type=int, default=5)
    arguments = parser.parse_args()
    prepare(
        arguments.source,
        arguments.output,
        arguments.target_frames,
        arguments.validation_frames,
        max(0, arguments.min_strict_points),
    )


if __name__ == "__main__":
    main()
