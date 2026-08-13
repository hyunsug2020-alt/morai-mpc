#!/usr/bin/env python3
"""Build a motion-compensated, scene-split pedestrian dataset."""

import argparse
import json
import math
import os
from collections import Counter, deque
from pathlib import Path

import numpy as np


LIDAR_OFFSET = np.asarray([1.045, 0.0, 1.234], dtype=np.float64)


def atomic_write(path, content):
    path = Path(path)
    temporary = Path(str(path) + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def rotation(yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.asarray([[cosine, -sine], [sine, cosine]], dtype=np.float64)


def pose(document):
    ego = document.get("ego")
    if not ego:
        return None
    return (
        np.asarray(ego["position"], dtype=np.float64),
        math.radians(float(ego["heading_deg"])),
    )


def transform_points(points, source_pose, target_pose):
    source_position, source_yaw = source_pose
    target_position, target_yaw = target_pose
    output = points.copy()
    source_vehicle_xy = points[:, :2].astype(np.float64) + LIDAR_OFFSET[:2]
    global_xy = source_vehicle_xy @ rotation(source_yaw).T + source_position[:2]
    target_vehicle_xy = (global_xy - target_position[:2]) @ rotation(-target_yaw).T
    output[:, :2] = (target_vehicle_xy - LIDAR_OFFSET[:2]).astype(np.float32)
    global_z = points[:, 2].astype(np.float64) + source_position[2] + LIDAR_OFFSET[2]
    output[:, 2] = (global_z - target_position[2] - LIDAR_OFFSET[2]).astype(np.float32)
    return output


def strict_count(points, center, size, yaw):
    delta = points[:, :2] - center[:2]
    cosine, sine = math.cos(yaw), math.sin(yaw)
    box_x = cosine * delta[:, 0] + sine * delta[:, 1]
    box_y = -sine * delta[:, 0] + cosine * delta[:, 1]
    inside = (
        (np.abs(box_x) <= 0.5 * size[0])
        & (np.abs(box_y) <= 0.5 * size[1])
        & (np.abs(points[:, 2] - center[2]) <= 0.5 * size[2])
    )
    return int(np.count_nonzero(inside))


def read_source(source):
    frames = []
    for label_path in sorted((source / "labels").glob("*.json")):
        document = json.loads(label_path.read_text(encoding="utf-8"))
        frame_pose = pose(document)
        if frame_pose is None:
            continue
        point_path = source / "velodyne" / (label_path.stem + ".bin")
        if not point_path.exists():
            continue
        frames.append(
            {
                "source_id": label_path.stem,
                "stamp_ns": int(document["stamp_ns"]),
                "document": document,
                "pose": frame_pose,
                "point_path": point_path,
            }
        )
    return frames


def assign_scenes(frames, reset_distance, reset_heading_deg, max_gap_seconds):
    scene = -1
    previous = None
    for frame in frames:
        new_scene = previous is None
        if previous is not None:
            distance = np.linalg.norm(frame["pose"][0][:2] - previous["pose"][0][:2])
            heading = abs(math.degrees(math.atan2(
                math.sin(frame["pose"][1] - previous["pose"][1]),
                math.cos(frame["pose"][1] - previous["pose"][1]),
            )))
            gap = (frame["stamp_ns"] - previous["stamp_ns"]) / 1.0e9
            new_scene = (
                distance > reset_distance
                or heading > reset_heading_deg
                or gap <= 0.0
                or gap > max_gap_seconds
            )
        if new_scene:
            scene += 1
        frame["scene"] = scene
        previous = frame
    return scene + 1


def discard_static_scenes(frames, minimum_scene_motion):
    motion = Counter()
    previous = {}
    for frame in frames:
        scene = frame["scene"]
        if scene in previous:
            motion[scene] += float(
                np.linalg.norm(frame["pose"][0][:2] - previous[scene]["pose"][0][:2])
            )
        previous[scene] = frame
    active = {scene for scene, distance in motion.items() if distance >= minimum_scene_motion}
    filtered = [frame for frame in frames if frame["scene"] in active]
    remap = {scene: index for index, scene in enumerate(sorted(active))}
    for frame in filtered:
        frame["scene"] = remap[frame["scene"]]
    return filtered, len(frames) - len(filtered), len(active)


def load_points(path):
    flat = np.fromfile(str(path), dtype=np.float32)
    if flat.size % 4:
        raise RuntimeError("invalid point file: {}".format(path))
    return flat.reshape((-1, 4))


def load_sites(path):
    if path is None or not path.exists():
        return []
    document = json.loads(path.read_text(encoding="utf-8"))
    return [
        {
            "site": int(item["site"]),
            "position": np.asarray([item["x"], item["y"]], dtype=np.float64),
            "scenario": item["scenario"],
        }
        for item in document
    ]


def assign_scene_sites(frames, sites):
    if not sites:
        return {}
    first_pose = {}
    for frame in frames:
        first_pose.setdefault(frame["scene"], frame["pose"][0][:2])
    return {
        scene: min(
            sites,
            key=lambda site: float(np.linalg.norm(position - site["position"])),
        )["site"]
        for scene, position in first_pose.items()
    }


def prepare(arguments):
    source = arguments.source.resolve()
    output = arguments.output.resolve()
    points_dir = output / "points"
    labels_dir = output / "labels"
    image_sets = output / "ImageSets"
    for directory in (points_dir, labels_dir, image_sets):
        directory.mkdir(parents=True, exist_ok=True)

    frames = read_source(source)
    if arguments.max_frames > 0:
        frames = frames[: arguments.max_frames]
    if not frames:
        raise RuntimeError("no frames with ego pose are available")
    assign_scenes(
        frames,
        arguments.reset_distance,
        arguments.reset_heading_deg,
        arguments.max_gap_seconds,
    )
    frames, discarded_static_frames, scene_count = discard_static_scenes(
        frames, arguments.minimum_scene_motion
    )
    if not frames:
        raise RuntimeError("all scenes were static")

    sites = load_sites(arguments.site_manifest)
    scene_sites = assign_scene_sites(frames, sites)
    history = deque(maxlen=arguments.sweeps)
    train, validation, mapping = [], [], []
    kept, filtered = Counter(), Counter()
    distance_bins = Counter()
    site_frames, site_objects = Counter(), Counter()
    empty_frames = 0
    for output_index, frame in enumerate(frames):
        site_name = "site_{}".format(scene_sites.get(frame["scene"], "unknown"))
        site_frames[site_name] += 1
        while history and history[-1]["scene"] != frame["scene"]:
            history.clear()
        history.append(frame)
        selected_history = list(history)[-arguments.sweeps :]
        accumulated = []
        for old in selected_history:
            age = (frame["stamp_ns"] - old["stamp_ns"]) / 1.0e9
            if age < -1.0e-6 or age > arguments.max_sweep_age:
                continue
            accumulated.append(
                transform_points(load_points(old["point_path"]), old["pose"], frame["pose"])
            )
        points = np.concatenate(accumulated, axis=0) if accumulated else load_points(frame["point_path"])
        frame_id = "{:06d}".format(output_index)
        points.astype(np.float32, copy=False).tofile(str(points_dir / (frame_id + ".bin")))

        lines = []
        for obj in frame["document"].get("objects", []):
            if obj.get("class_name") != "Pedestrian":
                continue
            center = np.asarray(obj["center"], dtype=np.float64)
            size = np.asarray(obj["size"], dtype=np.float64)
            yaw = float(obj["yaw_rad"])
            distance = float(np.linalg.norm(center[:2]))
            if np.any(size <= 0.05) or distance > arguments.max_range:
                filtered["invalid_size_or_range"] += 1
                continue
            count = strict_count(points, center, size, yaw)
            if count < arguments.min_strict_points:
                filtered["insufficient_points"] += 1
                continue
            values = list(center) + list(size) + [yaw]
            lines.append("{} Pedestrian".format(" ".join("{:.9f}".format(x) for x in values)))
            kept["Pedestrian"] += 1
            site_objects[site_name] += 1
            distance_bins["{:02d}-{:02d}".format(int(distance // 5) * 5, int(distance // 5) * 5 + 5)] += 1
        atomic_write(labels_dir / (frame_id + ".txt"), "\n".join(lines) + ("\n" if lines else ""))
        empty_frames += int(not lines)

        is_validation = (
            (frame["scene"] + 1) % arguments.validation_scene_period == 0
        )
        (validation if is_validation else train).append(frame_id)
        mapping.append(
            {
                "frame_id": frame_id,
                "source_id": frame["source_id"],
                "scene": frame["scene"],
                "site": site_name,
                "split": "val" if is_validation else "train",
                "sweeps": len(accumulated),
                "num_points": int(len(points)),
            }
        )
        if (output_index + 1) % 500 == 0:
            print("prepared {}/{}".format(output_index + 1, len(frames)), flush=True)

    if not train or not validation:
        raise RuntimeError("scene split produced an empty train or validation set")
    atomic_write(image_sets / "train.txt", "\n".join(train) + "\n")
    atomic_write(image_sets / "val.txt", "\n".join(validation) + "\n")
    atomic_write(image_sets / "all.txt", "\n".join(train + validation) + "\n")
    atomic_write(output / "source_mapping.jsonl", "".join(json.dumps(x, separators=(",", ":")) + "\n" for x in mapping))
    summary = {
        "status": "complete",
        "source": str(source),
        "output": str(output),
        "frames": len(frames),
        "train_frames": len(train),
        "validation_frames": len(validation),
        "scene_count": scene_count,
        "split_method": "whole-scene holdout every {} scenes".format(arguments.validation_scene_period),
        "sweeps": arguments.sweeps,
        "max_sweep_age_seconds": arguments.max_sweep_age,
        "kept_objects": dict(kept),
        "filtered_objects": dict(filtered),
        "distance_bins_m": dict(sorted(distance_bins.items())),
        "site_frames": dict(sorted(site_frames.items())),
        "site_objects": dict(sorted(site_objects.items())),
        "site_manifest": str(arguments.site_manifest) if arguments.site_manifest else None,
        "empty_frames": empty_frames,
        "discarded_static_frames": discarded_static_frames,
        "minimum_scene_motion_m": arguments.minimum_scene_motion,
    }
    atomic_write(output / "preparation_summary.json", json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, ensure_ascii=False), flush=True)


def main():
    repository = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=repository / "datasets/morai_lidar_pedestrian_far_v2_20260802")
    parser.add_argument("--output", type=Path, default=repository / "datasets/morai_pedestrian_far_v2_openpcdet")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--sweeps", type=int, default=3)
    parser.add_argument("--max-sweep-age", type=float, default=1.2)
    parser.add_argument("--max-range", type=float, default=45.0)
    parser.add_argument("--min-strict-points", type=int, default=1)
    parser.add_argument("--reset-distance", type=float, default=12.0)
    parser.add_argument("--reset-heading-deg", type=float, default=25.0)
    parser.add_argument("--max-gap-seconds", type=float, default=20.0)
    parser.add_argument("--minimum-scene-motion", type=float, default=3.0)
    # Hold out whole scenes while keeping at least 30k of the 31k+ prepared
    # frames in the actual training split.  Every fifth scene left only 26.5k
    # train frames and did not satisfy the requested 30k-frame training set.
    parser.add_argument("--validation-scene-period", type=int, default=20)
    parser.add_argument(
        "--site-manifest",
        type=Path,
        default=repository / "logs/far_v2/multisite_manifest.json",
    )
    prepare(parser.parse_args())


if __name__ == "__main__":
    main()
