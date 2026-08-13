#!/usr/bin/env python3
"""Convert the recorded MORAI dataset into OpenPCDet CustomDataset layout."""

import argparse
import json
import math
import os
from pathlib import Path

import numpy as np


def atomic_write(path, content):
    temporary = Path(str(path) + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def load_frame_ids(source):
    manifest_path = source / "manifest.jsonl"
    frames = []
    with manifest_path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                frames.append(str(json.loads(line)["frame_id"]))
    if not frames:
        raise RuntimeError("source manifest contains no frames")
    return frames


def split_frames(frames, block_size, validation_block_period):
    train = []
    validation = []
    for position, frame_id in enumerate(frames):
        block_index = position // block_size
        if block_index % validation_block_period == validation_block_period - 1:
            validation.append(frame_id)
        else:
            train.append(frame_id)
    return train, validation


def create_point_link(source, output):
    target = source / "velodyne"
    link = output / "points"
    if link.is_symlink():
        if link.resolve() != target.resolve():
            raise RuntimeError("existing points symlink has a different target")
        return
    if link.exists():
        raise RuntimeError("output points path already exists and is not a symlink")
    link.symlink_to(target.resolve(), target_is_directory=True)


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
    with source_path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    points = np.fromfile(point_path, dtype=np.float32)
    if points.size % 4:
        raise RuntimeError(
            "point file does not contain x/y/z/intensity rows: {}".format(
                point_path
            )
        )
    points = points.reshape((-1, 4))
    lines = []
    source_count = 0
    filtered_count = 0
    for obj in document.get("objects", []):
        if obj.get("class_name") != "Car":
            continue
        source_count += 1
        center = [float(value) for value in obj["center"]]
        size = [float(value) for value in obj["size"]]
        yaw = float(obj["yaw_rad"])
        point_count = strict_point_count(points, center, size, yaw)
        if point_count < min_strict_points:
            filtered_count += 1
            continue
        values = center + size + [yaw]
        lines.append(
            "{} Car".format(" ".join("{:.9f}".format(value) for value in values))
        )
    atomic_write(output_path, "\n".join(lines) + ("\n" if lines else ""))
    return len(lines), source_count, filtered_count


def prepare(
    source,
    output,
    block_size,
    validation_block_period,
    min_strict_points,
):
    source = source.resolve()
    output.mkdir(parents=True, exist_ok=True)
    labels_dir = output / "labels"
    image_sets_dir = output / "ImageSets"
    labels_dir.mkdir(exist_ok=True)
    image_sets_dir.mkdir(exist_ok=True)
    create_point_link(source, output)

    frames = load_frame_ids(source)
    train, validation = split_frames(
        frames, block_size, validation_block_period
    )
    object_count = 0
    source_object_count = 0
    filtered_object_count = 0
    empty_count = 0
    for frame_id in frames:
        count, source_count, filtered_count = convert_label(
            source / "labels" / (frame_id + ".json"),
            source / "velodyne" / (frame_id + ".bin"),
            labels_dir / (frame_id + ".txt"),
            min_strict_points,
        )
        object_count += count
        source_object_count += source_count
        filtered_object_count += filtered_count
        empty_count += int(count == 0)

    atomic_write(image_sets_dir / "train.txt", "\n".join(train) + "\n")
    atomic_write(
        image_sets_dir / "val.txt", "\n".join(validation) + "\n"
    )
    summary = {
        "source": str(source),
        "output": str(output.resolve()),
        "frames": len(frames),
        "train_frames": len(train),
        "validation_frames": len(validation),
        "vehicle_boxes": object_count,
        "source_vehicle_boxes": source_object_count,
        "filtered_vehicle_boxes": filtered_object_count,
        "min_strict_points_per_box": min_strict_points,
        "empty_frames": empty_count,
        "split_method": "contiguous temporal blocks",
        "block_size": block_size,
        "validation_block_period": validation_block_period,
    }
    atomic_write(
        output / "preparation_summary.json",
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )
    print(json.dumps(summary, ensure_ascii=False))


def main():
    repository = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=repository / "datasets" / "morai_lidar",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=repository / "datasets" / "morai_openpcdet",
    )
    parser.add_argument("--block-size", type=int, default=250)
    parser.add_argument("--validation-block-period", type=int, default=5)
    parser.add_argument("--min-strict-points", type=int, default=5)
    arguments = parser.parse_args()
    prepare(
        arguments.source,
        arguments.output,
        max(10, arguments.block_size),
        max(2, arguments.validation_block_period),
        max(0, arguments.min_strict_points),
    )


if __name__ == "__main__":
    main()
