#!/usr/bin/env python3
"""Validate paired camera images and normalized YOLO vehicle labels."""

import argparse
import json
from pathlib import Path

import cv2


def validate(root):
    root = root.expanduser().resolve()
    images = {path.stem: path for path in (root / "images").glob("*.jpg")}
    labels = {path.stem: path for path in (root / "labels").glob("*.txt")}
    metadata = {path.stem: path for path in (root / "metadata").glob("*.json")}
    problems = []
    object_count = 0
    empty_count = 0
    for frame_id in sorted(set(images) | set(labels) | set(metadata)):
        missing = [
            name
            for name, collection in (
                ("image", images), ("label", labels), ("metadata", metadata)
            )
            if frame_id not in collection
        ]
        if missing:
            problems.append("{} missing {}".format(frame_id, ",".join(missing)))
            continue
        image = cv2.imread(str(images[frame_id]), cv2.IMREAD_COLOR)
        if image is None:
            problems.append("{} unreadable image".format(frame_id))
            continue
        frame_objects = 0
        with labels[frame_id].open(encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                fields = line.split()
                if len(fields) != 5:
                    problems.append(
                        "{}:{} expected 5 YOLO columns".format(frame_id, line_number)
                    )
                    continue
                try:
                    class_id = int(fields[0])
                    values = [float(value) for value in fields[1:]]
                except ValueError:
                    problems.append(
                        "{}:{} invalid numeric value".format(frame_id, line_number)
                    )
                    continue
                if class_id != 0:
                    problems.append(
                        "{}:{} unexpected class {}".format(
                            frame_id, line_number, class_id
                        )
                    )
                if not all(0.0 <= value <= 1.0 for value in values):
                    problems.append(
                        "{}:{} normalized value outside [0,1]".format(
                            frame_id, line_number
                        )
                    )
                if values[2] <= 0.0 or values[3] <= 0.0:
                    problems.append(
                        "{}:{} non-positive box size".format(frame_id, line_number)
                    )
                frame_objects += 1
        try:
            with metadata[frame_id].open(encoding="utf-8") as stream:
                document = json.load(stream)
            if int(document.get("num_labels", -1)) != frame_objects:
                problems.append("{} label/metadata count mismatch".format(frame_id))
            expected_width = int(document["image"]["width"])
            expected_height = int(document["image"]["height"])
            if image.shape[1] != expected_width or image.shape[0] != expected_height:
                problems.append("{} image/metadata size mismatch".format(frame_id))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            problems.append("{} invalid metadata: {}".format(frame_id, error))
        object_count += frame_objects
        empty_count += int(frame_objects == 0)

    report = {
        "root": str(root),
        "frames": len(images),
        "vehicle_boxes": object_count,
        "empty_frames": empty_count,
        "problems": problems,
        "valid": not problems and bool(images),
    }
    report_path = root / "validation_report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["valid"] else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    arguments = parser.parse_args()
    raise SystemExit(validate(arguments.dataset))


if __name__ == "__main__":
    main()
