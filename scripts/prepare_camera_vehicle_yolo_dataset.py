#!/usr/bin/env python3
"""Build a session-isolated YOLO train/validation view using symlinks."""

import argparse
import json
import os
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PROJECT_ROOT / "datasets" / "morai_camera_vehicle" / "train"
DEFAULT_OUTPUT = PROJECT_ROOT / "datasets" / "morai_camera_vehicle_yolo"
DEFAULT_VAL_SESSIONS = (
    "train_20260811_210435",
    "train_20260811_210729",
    "train_20260811_212113",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create leak-free YOLO train/val directories by session."
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--val-session",
        action="append",
        dest="val_sessions",
        help="Session ID assigned to validation; may be repeated.",
    )
    return parser.parse_args()


def replace_symlink(destination, source):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink():
        if destination.resolve() == source.resolve():
            return
        destination.unlink()
    elif destination.exists():
        raise RuntimeError("Refusing to replace non-symlink: {}".format(destination))
    destination.symlink_to(os.path.relpath(str(source), str(destination.parent)))


def remove_stale_symlinks(directory, expected_names):
    if not directory.exists():
        return
    for path in directory.iterdir():
        if path.name in expected_names:
            continue
        if path.is_symlink():
            path.unlink()
        else:
            raise RuntimeError("Unexpected non-symlink in generated data: {}".format(path))


def main():
    args = parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    val_sessions = set(args.val_sessions or DEFAULT_VAL_SESSIONS)
    manifest_path = source / "manifest.jsonl"
    if not manifest_path.is_file():
        raise RuntimeError("Manifest not found: {}".format(manifest_path))

    records = []
    seen_ids = set()
    with manifest_path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            record = json.loads(line)
            frame_id = str(record["frame_id"])
            if frame_id in seen_ids:
                raise RuntimeError("Duplicate frame ID: {}".format(frame_id))
            seen_ids.add(frame_id)
            records.append(record)

    available_sessions = {str(record["session_id"]) for record in records}
    missing_sessions = val_sessions - available_sessions
    if missing_sessions:
        raise RuntimeError(
            "Validation sessions not present: {}".format(", ".join(sorted(missing_sessions)))
        )

    expected = {
        (split, kind): set()
        for split in ("train", "val")
        for kind in ("images", "labels")
    }
    counts = {
        "train": {"frames": 0, "labelled_frames": 0, "boxes": 0},
        "val": {"frames": 0, "labelled_frames": 0, "boxes": 0},
    }
    session_counts = {}

    for record in records:
        frame_id = str(record["frame_id"])
        session_id = str(record["session_id"])
        split = "val" if session_id in val_sessions else "train"
        image_source = source / "images" / (frame_id + ".jpg")
        label_source = source / "labels" / (frame_id + ".txt")
        if not image_source.is_file() or not label_source.is_file():
            raise RuntimeError("Missing image or label for frame {}".format(frame_id))

        image_name = frame_id + ".jpg"
        label_name = frame_id + ".txt"
        replace_symlink(output / "images" / split / image_name, image_source)
        replace_symlink(output / "labels" / split / label_name, label_source)
        expected[(split, "images")].add(image_name)
        expected[(split, "labels")].add(label_name)

        box_count = sum(1 for line in label_source.read_text().splitlines() if line.strip())
        counts[split]["frames"] += 1
        counts[split]["labelled_frames"] += int(box_count > 0)
        counts[split]["boxes"] += box_count
        session_counts.setdefault(session_id, {"split": split, "frames": 0})
        session_counts[session_id]["frames"] += 1

    for (split, kind), names in expected.items():
        remove_stale_symlinks(output / kind / split, names)

    summary = {
        "source": str(source),
        "output": str(output),
        "validation_sessions": sorted(val_sessions),
        "counts": counts,
        "sessions": dict(sorted(session_counts.items())),
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "split_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
