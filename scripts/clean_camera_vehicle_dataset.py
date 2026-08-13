#!/usr/bin/env python3
"""Apply a reviewed camera-dataset cleanup while preserving every original file."""

import argparse
import json
import os
import shutil
from datetime import datetime
from pathlib import Path

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET = PROJECT_ROOT / "datasets" / "morai_camera_vehicle" / "train"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("review", type=Path, help="Reviewed cleanup decisions (JSON)")
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--quarantine", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def atomic_text(path, content):
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def atomic_jpeg(path, image):
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise RuntimeError("OpenCV JPEG encoding failed: {}".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(encoded.tobytes())
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def read_manifest(path):
    records = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as error:
                raise RuntimeError(
                    "Invalid manifest line {}: {}".format(line_number, error)
                )
    return records


def draw_overlay(image, labels):
    overlay = image.copy()
    for label in labels:
        x1, y1, x2, y2 = [int(round(value)) for value in label["bbox_xyxy"]]
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            overlay,
            "Car id={} {:.1f}m".format(label["object_id"], label["distance_m"]),
            (x1, max(20, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    return overlay


def require_sample_files(dataset, frame_id):
    paths = {
        "images": dataset / "images" / (frame_id + ".jpg"),
        "labels": dataset / "labels" / (frame_id + ".txt"),
        "metadata": dataset / "metadata" / (frame_id + ".json"),
        "overlays": dataset / "overlays" / (frame_id + ".jpg"),
    }
    missing = [str(path) for path in paths.values() if not path.is_file()]
    if missing:
        raise RuntimeError("Missing sample files: {}".format(", ".join(missing)))
    return paths


def main():
    args = parse_args()
    dataset = args.dataset.resolve()
    review_path = args.review.resolve()
    review = json.loads(review_path.read_text(encoding="utf-8"))
    manifest_path = dataset / "manifest.jsonl"
    records = read_manifest(manifest_path)
    records_by_id = {str(record["frame_id"]): record for record in records}
    if len(records_by_id) != len(records):
        raise RuntimeError("Manifest contains duplicate frame IDs")

    quarantine_frames = {
        str(frame_id): reason
        for frame_id, reason in review.get("quarantine_frames", {}).items()
    }
    remove_boxes = {
        str(frame_id): {int(object_id) for object_id in object_ids}
        for frame_id, object_ids in review.get("remove_boxes", {}).items()
    }
    overlap = set(quarantine_frames) & set(remove_boxes)
    if overlap:
        raise RuntimeError("Frames have conflicting actions: {}".format(sorted(overlap)))
    unknown = (set(quarantine_frames) | set(remove_boxes)) - set(records_by_id)
    if unknown:
        raise RuntimeError("Review references unknown frames: {}".format(sorted(unknown)))

    quarantine = args.quarantine
    if quarantine is None:
        quarantine = dataset.parent / "quarantine" / (
            "reviewed_cleanup_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        )
    quarantine = quarantine.resolve()
    if quarantine.exists():
        raise RuntimeError("Refusing to reuse quarantine directory: {}".format(quarantine))

    plan = {
        "dataset": str(dataset),
        "quarantine": str(quarantine),
        "frames_before": len(records),
        "quarantined_frames": len(quarantine_frames),
        "modified_frames": len(remove_boxes),
        "requested_box_removals": sum(len(ids) for ids in remove_boxes.values()),
    }
    if args.dry_run:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return

    quarantine.mkdir(parents=True)
    shutil.copy2(review_path, quarantine / "review.json")
    shutil.copy2(manifest_path, quarantine / "manifest.before_cleanup.jsonl")

    removed_records = []
    modified_details = []
    for frame_id in sorted(quarantine_frames):
        paths = require_sample_files(dataset, frame_id)
        for kind, source in paths.items():
            destination = quarantine / "removed_frames" / kind / source.name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(source), str(destination))
        record = dict(records_by_id[frame_id])
        record["cleanup_reason"] = quarantine_frames[frame_id]
        removed_records.append(record)

    for frame_id in sorted(remove_boxes):
        paths = require_sample_files(dataset, frame_id)
        metadata = json.loads(paths["metadata"].read_text(encoding="utf-8"))
        old_labels = list(metadata.get("labels", []))
        requested_ids = remove_boxes[frame_id]
        present_ids = {int(label["object_id"]) for label in old_labels}
        missing_ids = requested_ids - present_ids
        if missing_ids:
            raise RuntimeError(
                "Frame {} does not contain object IDs {}".format(
                    frame_id, sorted(missing_ids)
                )
            )
        new_labels = [
            label for label in old_labels
            if int(label["object_id"]) not in requested_ids
        ]

        for kind in ("labels", "metadata", "overlays"):
            source = paths[kind]
            destination = quarantine / "originals_before_box_edit" / kind / source.name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)

        metadata["labels"] = new_labels
        metadata["num_labels"] = len(new_labels)
        metadata["cleanup"] = {
            "removed_object_ids": sorted(requested_ids),
            "reason": "vehicle fully or severely occluded by a wall/building",
        }
        yolo_lines = [
            "{} {:.9f} {:.9f} {:.9f} {:.9f}".format(
                label["class_id"], *label["bbox_yolo"]
            )
            for label in new_labels
        ]
        atomic_text(
            paths["labels"],
            "\n".join(yolo_lines) + ("\n" if yolo_lines else ""),
        )
        atomic_text(
            paths["metadata"],
            json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        )
        image = cv2.imread(str(paths["images"]), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("Could not read image: {}".format(paths["images"]))
        atomic_jpeg(paths["overlays"], draw_overlay(image, new_labels))
        modified_details.append(
            {
                "frame_id": frame_id,
                "removed_object_ids": sorted(requested_ids),
                "labels_before": len(old_labels),
                "labels_after": len(new_labels),
            }
        )

    active_records = []
    for record in records:
        frame_id = str(record["frame_id"])
        if frame_id in quarantine_frames:
            continue
        updated = dict(record)
        if frame_id in remove_boxes:
            metadata = json.loads(
                (dataset / "metadata" / (frame_id + ".json")).read_text(
                    encoding="utf-8"
                )
            )
            updated["num_labels"] = int(metadata["num_labels"])
        active_records.append(updated)

    atomic_text(
        manifest_path,
        "".join(
            json.dumps(record, ensure_ascii=False) + "\n" for record in active_records
        ),
    )
    atomic_text(
        quarantine / "removed_manifest.jsonl",
        "".join(
            json.dumps(record, ensure_ascii=False) + "\n" for record in removed_records
        ),
    )
    report = dict(plan)
    report.update(
        {
            "frames_after": len(active_records),
            "box_removals_applied": sum(
                item["labels_before"] - item["labels_after"]
                for item in modified_details
            ),
            "modified_details": modified_details,
        }
    )
    atomic_text(
        quarantine / "cleanup_report.json",
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
