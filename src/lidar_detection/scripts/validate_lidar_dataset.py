#!/usr/bin/env python3
"""Validate a recorded MORAI LiDAR dataset and write a quality report."""

import argparse
import json
import math
import os
import statistics
from collections import Counter, defaultdict
from datetime import datetime

import numpy as np


def numbered_files(directory, extension):
    result = {}
    if not os.path.isdir(directory):
        return result
    for name in os.listdir(directory):
        stem, suffix = os.path.splitext(name)
        if suffix == extension and stem.isdigit():
            result[stem] = os.path.join(directory, name)
    return result


def percentile(values, percentage):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentage))


class DatasetValidator:
    def __init__(self, root, report_path=None):
        self.root = os.path.abspath(root)
        self.report_path = report_path or os.path.join(
            self.root, "validation_report.json"
        )
        self.errors = []
        self.warnings = []
        self.max_reported_issues = 100

    def error(self, message):
        if len(self.errors) < self.max_reported_issues:
            self.errors.append(message)

    def warning(self, message):
        if len(self.warnings) < self.max_reported_issues:
            self.warnings.append(message)

    def validate(self):
        point_files = numbered_files(
            os.path.join(self.root, "velodyne"), ".bin"
        )
        json_files = numbered_files(
            os.path.join(self.root, "labels"), ".json"
        )
        text_files = numbered_files(
            os.path.join(self.root, "label_lidar"), ".txt"
        )
        timestamp_files = numbered_files(
            os.path.join(self.root, "timestamps"), ".txt"
        )
        frame_sets = {
            "velodyne": set(point_files),
            "labels": set(json_files),
            "label_lidar": set(text_files),
            "timestamps": set(timestamp_files),
        }
        reference_frames = frame_sets["velodyne"]
        for name, frames in frame_sets.items():
            missing = sorted(reference_frames - frames)
            extra = sorted(frames - reference_frames)
            if missing:
                self.error(
                    "{} missing {} frames, first={}".format(
                        name, len(missing), missing[:5]
                    )
                )
            if extra:
                self.error(
                    "{} has {} extra frames, first={}".format(
                        name, len(extra), extra[:5]
                    )
                )

        ordered_frames = sorted(reference_frames, key=int)
        if ordered_frames:
            indices = [int(frame) for frame in ordered_frames]
            expected = list(range(indices[0], indices[-1] + 1))
            if indices != expected:
                self.error("frame indices are not contiguous")
        else:
            self.error("dataset contains no point-cloud frames")

        manifest_frames = self.read_manifest()
        image_set_frames = self.read_image_set()
        if manifest_frames != ordered_frames:
            self.error(
                "manifest frame list differs from velodyne frame list"
            )
        if image_set_frames != ordered_frames:
            self.error(
                "ImageSets/all.txt differs from velodyne frame list"
            )

        point_counts = []
        object_counts = []
        timestamps_ns = []
        classes = Counter()
        unique_ids = set()
        points_per_box = []
        ranges = []
        centers_by_id = defaultdict(list)
        sync_modes = Counter()
        empty_frames = 0
        non_finite_point_frames = 0

        for frame in ordered_frames:
            point_path = point_files[frame]
            size_bytes = os.path.getsize(point_path)
            if size_bytes <= 0 or size_bytes % 16 != 0:
                self.error(
                    "{} invalid point byte size {}".format(frame, size_bytes)
                )
                continue
            point_count = size_bytes // 16
            point_counts.append(point_count)
            points = np.memmap(
                point_path, dtype=np.float32, mode="r", shape=(point_count, 4)
            )
            if not np.isfinite(points).all():
                non_finite_point_frames += 1
                self.error("{} contains non-finite points".format(frame))
            del points

            try:
                with open(json_files[frame], encoding="utf-8") as stream:
                    label = json.load(stream)
            except (OSError, ValueError) as error:
                self.error("{} invalid JSON label: {}".format(frame, error))
                continue

            if str(label.get("frame_id")) != frame:
                self.error("{} label frame_id mismatch".format(frame))
            if int(label.get("num_points", -1)) != point_count:
                self.error("{} num_points mismatch".format(frame))
            sync_mode = str(label.get("sync", {}).get("mode", "missing"))
            sync_modes[sync_mode] += 1
            if sync_mode != "interpolated":
                self.error(
                    "{} sync mode is {}".format(frame, sync_mode)
                )

            stamp_ns = int(label.get("stamp_ns", -1))
            timestamps_ns.append(stamp_ns)
            try:
                with open(
                    timestamp_files[frame], encoding="utf-8"
                ) as stream:
                    timestamp_text = stream.read().strip()
                seconds_text, nanoseconds_text = timestamp_text.split(".", 1)
                timestamp_file_ns = (
                    int(seconds_text) * 1000000000
                    + int(nanoseconds_text.ljust(9, "0")[:9])
                )
                if timestamp_file_ns != stamp_ns:
                    self.error("{} timestamp mismatch".format(frame))
            except (OSError, ValueError) as error:
                self.error(
                    "{} invalid timestamp: {}".format(frame, error)
                )

            objects = label.get("objects", [])
            object_counts.append(len(objects))
            if not objects:
                empty_frames += 1
            for obj in objects:
                class_name = str(obj.get("class_name", "missing"))
                classes[class_name] += 1
                unique_id = int(obj.get("id", -1))
                unique_ids.add(unique_id)
                points_in_box = int(obj.get("points_in_box", -1))
                points_per_box.append(points_in_box)
                center = [float(value) for value in obj.get("center", [])]
                size = [float(value) for value in obj.get("size", [])]
                yaw = float(obj.get("yaw_rad", float("nan")))
                values = center + size + [yaw]
                if (
                    len(center) != 3
                    or len(size) != 3
                    or not all(math.isfinite(value) for value in values)
                ):
                    self.error("{} invalid object geometry".format(frame))
                    continue
                if any(value <= 0.0 for value in size):
                    self.error("{} non-positive object size".format(frame))
                object_range = math.hypot(center[0], center[1])
                ranges.append(object_range)
                centers_by_id[unique_id].append(center)

            try:
                with open(text_files[frame], encoding="utf-8") as stream:
                    text_lines = [
                        line for line in stream.read().splitlines() if line
                    ]
                if len(text_lines) != len(objects):
                    self.error(
                        "{} text/JSON object count mismatch".format(frame)
                    )
            except OSError as error:
                self.error(
                    "{} invalid text label: {}".format(frame, error)
                )

        time_deltas = [
            (second - first) / 1.0e9
            for first, second in zip(timestamps_ns, timestamps_ns[1:])
            if second > first
        ]
        non_monotonic = sum(
            second <= first
            for first, second in zip(timestamps_ns, timestamps_ns[1:])
        )
        if non_monotonic:
            self.error(
                "{} non-monotonic timestamps".format(non_monotonic)
            )

        motion_spans = {}
        for unique_id, centers in centers_by_id.items():
            if len(centers) < 2:
                motion_spans[str(unique_id)] = 0.0
                continue
            array = np.asarray(centers, dtype=np.float64)
            span = np.ptp(array, axis=0)
            motion_spans[str(unique_id)] = float(np.linalg.norm(span[:2]))

        report = {
            "valid": not self.errors,
            "validated_at": datetime.now().astimezone().isoformat(),
            "root": self.root,
            "frame_count": len(ordered_frames),
            "first_frame": ordered_frames[0] if ordered_frames else None,
            "last_frame": ordered_frames[-1] if ordered_frames else None,
            "errors": self.errors,
            "warnings": self.warnings,
            "integrity": {
                "manifest_frames": len(manifest_frames),
                "image_set_frames": len(image_set_frames),
                "non_finite_point_frames": non_finite_point_frames,
                "non_monotonic_timestamps": non_monotonic,
                "sync_modes": dict(sync_modes),
            },
            "points": self.summarize(point_counts),
            "objects_per_frame": self.summarize(object_counts),
            "labels": {
                "total_objects": sum(object_counts),
                "empty_frames": empty_frames,
                "empty_frame_ratio": (
                    float(empty_frames) / len(ordered_frames)
                    if ordered_frames else None
                ),
                "classes": dict(classes),
                "unique_ids": len(unique_ids),
                "points_per_box": self.summarize(points_per_box),
                "range_m": self.summarize(ranges),
                "object_center_span_xy_m_by_id": motion_spans,
            },
            "frame_interval_seconds": self.summarize(time_deltas),
        }
        self.atomic_write_report(report)
        return report

    def read_manifest(self):
        path = os.path.join(self.root, "manifest.jsonl")
        frames = []
        try:
            with open(path, encoding="utf-8") as stream:
                for line_number, line in enumerate(stream, 1):
                    if not line.strip():
                        continue
                    try:
                        frames.append(str(json.loads(line)["frame_id"]))
                    except (KeyError, TypeError, ValueError) as error:
                        self.error(
                            "manifest line {} invalid: {}".format(
                                line_number, error
                            )
                        )
        except OSError as error:
            self.error("cannot read manifest: {}".format(error))
        return frames

    def read_image_set(self):
        path = os.path.join(self.root, "ImageSets", "all.txt")
        try:
            with open(path, encoding="utf-8") as stream:
                return [line.strip() for line in stream if line.strip()]
        except OSError as error:
            self.error("cannot read ImageSets/all.txt: {}".format(error))
            return []

    @staticmethod
    def summarize(values):
        if not values:
            return {
                "count": 0,
                "min": None,
                "mean": None,
                "median": None,
                "p95": None,
                "max": None,
            }
        return {
            "count": len(values),
            "min": min(values),
            "mean": statistics.mean(values),
            "median": statistics.median(values),
            "p95": percentile(values, 95),
            "max": max(values),
        }

    def atomic_write_report(self, report):
        temporary = self.report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(
                report,
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_root")
    parser.add_argument("--report")
    arguments = parser.parse_args()
    report = DatasetValidator(
        arguments.dataset_root, arguments.report
    ).validate()
    print(
        json.dumps(
            {
                "valid": report["valid"],
                "frame_count": report["frame_count"],
                "errors": report["errors"],
            },
            ensure_ascii=False,
        )
    )
    raise SystemExit(0 if report["valid"] else 1)


if __name__ == "__main__":
    main()
