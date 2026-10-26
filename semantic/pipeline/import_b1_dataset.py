#!/usr/bin/env python3
"""Create six-class B0 masks from existing B1 processed data without changing it."""
from __future__ import annotations

import argparse
import json
from collections import Counter
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from camera_config import supervised_class_ids

from semantic_common import PROCESSED_ROOT, atomic_write_json, link_or_copy, load_class_config, write_latest

NAMES = ("vehicle", "pedestrian", "obstacle", "traffic_light", "yellow_lane", "white_lane")


def convert(sources: list[Path], output: Path) -> dict:
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(output)
    output.mkdir(parents=True, exist_ok=True)
    counts = np.zeros(6, np.int64)
    split_counts = Counter()
    total_pixels = 0
    supervised_pixels = np.zeros(6, np.int64)
    copied = 0
    with (output / "manifest.jsonl").open("w", encoding="utf-8") as target:
        for source in sources:
            source = source.resolve()
            classes_path = source / "classes.yaml"
            _, old_classes, _, _ = load_class_config(classes_path)
            names = {row["name"]: row["id"] for row in old_classes}
            for required in NAMES:
                if required not in names:
                    raise ValueError(f"{source}: missing old class {required}")
            lut = np.full(256, 255, np.uint8)
            for new_id, name in enumerate(NAMES):
                lut[names[name]] = new_id
            # The old palette separates standing objects from obstacle.
            if "standing_object" in names:
                lut[names["standing_object"]] = 2
            with (source / "manifest.jsonl").open(encoding="utf-8") as stream:
                for index, line in enumerate(stream):
                    row = json.loads(line)
                    split = row["split"]
                    image = source / row["image"]
                    mask = cv2.imread(str(source / row["mask"]), cv2.IMREAD_UNCHANGED)
                    if mask is None or mask.ndim != 2 or not image.is_file():
                        raise ValueError(f"bad image or mask in {source}: {index}")
                    mapped = lut[mask]
                    active_ids = supervised_class_ids(row)
                    row["supervised_class_ids"] = active_ids
                    mapped[~np.isin(mapped, active_ids)] = 255
                    supervised_pixels[active_ids] += mapped.size
                    valid = mapped < 6
                    counts += np.bincount(mapped[valid], minlength=6)
                    total_pixels += mapped.size
                    name = f"{source.name}_{index:08d}"
                    image_out = output / "images" / split / (name + ".jpg")
                    mask_out = output / "masks" / split / (name + ".png")
                    link_or_copy(image, image_out)
                    mask_out.parent.mkdir(parents=True, exist_ok=True)
                    if not cv2.imwrite(str(mask_out), mapped):
                        raise OSError(mask_out)
                    row.update(image=str(image_out.relative_to(output)), mask=str(mask_out.relative_to(output)))
                    target.write(json.dumps(row, ensure_ascii=False) + "\n")
                    split_counts[split] += 1
                    copied += 1
                    if copied % 500 == 0:
                        print(f"converted {copied}", flush=True)
    local_classes = Path(__file__).with_name("classes.yaml")
    (output / "classes.yaml").write_bytes(local_classes.read_bytes())
    stats = {
        "samples": copied,
        "splits": dict(split_counts),
        "classes": [{"id": i, "name": name, "pixels": int(counts[i])} for i, name in enumerate(NAMES)],
        "ignored_pixels": int(total_pixels - counts.sum()),
        "total_pixels": int(total_pixels),
        "supervised_pixels_per_class": supervised_pixels.tolist(),
        "sources": [str(path.resolve()) for path in sources],
    }
    atomic_write_json(output / "statistics.json", stats)
    write_latest(PROCESSED_ROOT, output)
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, default=PROCESSED_ROOT / datetime.now().strftime("b0_import_%Y%m%d_%H%M%S"))
    args = parser.parse_args()
    print(json.dumps(convert(args.source, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
