#!/usr/bin/env python3
"""Convert raw MORAI Semantic JPEG pairs into a train/val/test dataset."""

from __future__ import annotations

import argparse
import json
import random
import shutil
from collections import Counter
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

from camera_config import supervised_class_ids

from semantic_common import (
    DEFAULT_CLASSES,
    PROCESSED_ROOT,
    RAW_ROOT,
    atomic_write_json,
    build_palette_lut,
    link_or_copy,
    load_class_config,
    read_latest,
    safe_name,
    semantic_bgr_to_mask,
    write_latest,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, action="append", help="raw session directory; repeatable; default is latest")
    parser.add_argument("--output", type=Path, help="default: timestamped processed dataset")
    parser.add_argument("--classes", type=Path, default=DEFAULT_CLASSES)
    parser.add_argument("--split", default="0.8,0.1,0.1", help="train,val,test ratios")
    parser.add_argument("--block-size", type=int, default=50)
    parser.add_argument("--temporal-split", action="store_true", help="smoke-only temporal split with guard intervals")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--quantization-bits", type=int, default=6)
    parser.add_argument("--max-color-distance", type=float, default=90.0)
    parser.add_argument("--front-exclusion-mask", type=Path, help="binary PNG: nonzero front-camera pixels excluded from input, loss and metrics")
    return parser.parse_args()


def read_records(session):
    metadata=session/'session.json'
    if metadata.is_file() and json.loads(metadata.read_text()).get('exclude_from_training'):
        raise ValueError(f'{session.name}: 재수집 대상으로 표시된 세션입니다. 새 차량 수집 세션을 선택하세요.')
    manifest = session / "semantic_pairs.jsonl"
    if not manifest.is_file():
        manifest = session / "manifest.jsonl"
    if not manifest.is_file():
        raise FileNotFoundError(manifest)
    records = []
    with manifest.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"{manifest}:{line_number}: {error}") from error
            record["_root"] = session
            records.append(record)
    return records


def assign_splits(records, ratios, block_size, seed, temporal=False):
    """Keep every camera in a recording together; temporal mode is smoke-only."""
    units = {}
    for record in records:
        if temporal:
            # MORAI timestamps share the same clock across all four cameras.
            stamp = record.get("rgb_timestamp_ns")
            key = ("time", int(stamp // 10_000_000_000)) if stamp else (record["session_id"], (int(record["sequence"]) - 1) // block_size)
        else:
            key = record.get("recording_id", record["session_id"])
        units.setdefault(key, []).append(record)
    if len(units) < 3:
        raise ValueError("학습/검증/테스트를 위해 독립된 주행 세션 3개 이상이 필요합니다. 짧은 점검에는 시간 분리 옵션을 사용하세요.")
    keys = sorted(units, key=str)
    random.Random(seed).shuffle(keys)
    # Search deterministic allocations that keep every observed camera in every split.
    expected = {row["view"] for row in records}
    coverage = len({tuple(sorted({row["view"] for row in rows})) for rows in units.values()})
    nv, nt = max(coverage, round(len(keys)*ratios[1])), max(coverage, round(len(keys)*ratios[2]))
    if nv + nt >= len(keys):
        raise ValueError("insufficient split groups")
    rng = random.Random(seed)
    for _ in range(1000):
        assigned = {"train": [], "val": [], "test": []}
        for i, key in enumerate(keys):
            split = "val" if i < nv else "test" if i < nv+nt else "train"
            rows = units[key]
            if temporal and block_size > 2:
                # Avoid adjacent frames at boundaries; keep cross-view assignments identical.
                if "rgb_timestamp_ns" in rows[0]:
                    rows = [row for row in rows if 1_000_000_000 <= row["rgb_timestamp_ns"] % 10_000_000_000 < 9_000_000_000]
                else:
                    rows = [row for row in rows if 0 < (int(row["sequence"])-1) % block_size < block_size-1]
            assigned[split].extend(rows)
        if all({row["view"] for row in rows} == expected for rows in assigned.values()):
            return assigned
        rng.shuffle(keys)
    raise ValueError("각 카메라가 train/val/test에 포함되도록 일반·신호등 세션을 추가 수집하세요.")


def main() -> int:
    args = parse_args()
    if not args.input:
        args.input = [read_latest(RAW_ROOT)]
    if args.output is None:
        args.output = PROCESSED_ROOT / datetime.now().strftime("semantic_%Y%m%d_%H%M%S")
    ratios = tuple(float(value) for value in args.split.split(","))
    if len(ratios) != 3 or any(value <= 0 for value in ratios) or abs(sum(ratios) - 1.0) > 1e-6:
        raise ValueError("--split must contain three positive ratios summing to 1")
    if args.block_size <= 0:
        raise ValueError("--block-size must be positive")
    output = args.output.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"output already exists and is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    exclusion = None
    if args.front_exclusion_mask:
        exclusion = cv2.imread(str(args.front_exclusion_mask), cv2.IMREAD_GRAYSCALE)
        if exclusion is None or not np.any(exclusion):
            raise ValueError("invalid or empty front exclusion mask")
        exclusion = exclusion > 0
        cv2.imwrite(str(output / 'front_exclusion_mask.png'), exclusion.astype(np.uint8)*255)

    config, classes, palette_rgb, palette_ids = load_class_config(args.classes)
    ignore_index = int(config.get("ignore_index", 255))
    lut, _distance = build_palette_lut(
        palette_rgb,
        palette_ids,
        quantization_bits=args.quantization_bits,
        max_distance=args.max_color_distance,
        ignore_index=ignore_index,
    )
    records = []
    for path in args.input:
        records.extend(read_records(path.expanduser().resolve()))
    if not records:
        raise ValueError("no raw records found")
    split_records = assign_splits(records, ratios, args.block_size, args.seed, args.temporal_split)

    pixel_counts = np.zeros(len(classes), dtype=np.int64)
    ignored_pixels = 0
    total_pixels = 0
    supervised_pixels = np.zeros(len(classes), dtype=np.int64)
    view_counts = Counter()
    output_manifest = output / "manifest.jsonl"
    with output_manifest.open("w", encoding="utf-8") as manifest:
        processed = 0
        for split, items in split_records.items():
            for record in sorted(items, key=lambda item: (item["session_id"], item["view"], item["sequence"])):
                root = record.pop("_root")
                rgb_path = root / record["rgb"]
                semantic_path = root / record["semantic"]
                rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
                semantic = cv2.imread(str(semantic_path), cv2.IMREAD_COLOR)
                if rgb is None or semantic is None:
                    raise ValueError(f"unreadable pair: {rgb_path}, {semantic_path}")
                if rgb.shape[:2] != semantic.shape[:2]:
                    raise ValueError(f"shape mismatch: {rgb_path}, {semantic_path}")
                mask = semantic_bgr_to_mask(semantic, lut, args.quantization_bits)
                valid_lut = (_distance <= args.max_color_distance).astype(np.uint8)
                valid_mask = semantic_bgr_to_mask(semantic, valid_lut, args.quantization_bits)
                active_ids = supervised_class_ids(record)
                record["supervised_class_ids"] = active_ids
                mask[~np.isin(mask, active_ids)] = ignore_index
                if exclusion is not None and record['view'] == 'front':
                    if exclusion.shape != mask.shape:
                        raise ValueError('front camera resolution differs from calibrated exclusion mask')
                    mask[exclusion] = ignore_index
                    valid_mask[exclusion] = 0
                supervised_pixels[active_ids] += int(valid_mask.sum())
                total_pixels += mask.size
                valid = mask != ignore_index
                if valid.any():
                    pixel_counts += np.bincount(mask[valid], minlength=len(classes))
                ignored_pixels += int((~valid).sum())

                filename = (
                    f"{safe_name(record['session_id'])}__{record['view']}__"
                    f"{int(record['sequence']):08d}.jpg"
                )
                image_output = output / "images" / split / filename
                mask_output = output / "masks" / split / Path(filename).with_suffix(".png")
                link_or_copy(rgb_path, image_output)
                mask_output.parent.mkdir(parents=True, exist_ok=True)
                if not cv2.imwrite(str(mask_output), mask, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
                    raise OSError(f"failed to write {mask_output}")
                valid_output = output / "valid" / split / Path(filename).with_suffix(".png")
                valid_output.parent.mkdir(parents=True, exist_ok=True)
                if not cv2.imwrite(str(valid_output), valid_mask):
                    raise OSError("valid mask write failed")
                converted = {
                    "valid_mask": str(valid_output.relative_to(output)),
                    **record,
                    "split": split,
                    "image": str(image_output.relative_to(output)),
                    "mask": str(mask_output.relative_to(output)),
                }
                manifest.write(json.dumps(converted, ensure_ascii=False) + "\n")
                view_counts[record["view"]] += 1
                processed += 1
                if processed % 100 == 0:
                    print(f"processed={processed}/{len(records)}", flush=True)

    shutil.copy2(args.classes, output / "classes.yaml")
    split_summary = {name: len(items) for name, items in split_records.items()}
    stats = {
        "label_version": 2,
        "animal_verified": bool(config.get("animal_verified", False)),
        "split_method": "temporal_smoke" if args.temporal_split else "session",
        "samples": processed,
        "splits": split_summary,
        "views": dict(sorted(view_counts.items())),
        "classes": [
            {**item, "pixels": int(pixel_counts[int(item["id"])])}
            for item in classes
        ],
        "ignored_pixels": ignored_pixels,
        "total_pixels": total_pixels,
        "supervised_pixels_per_class": supervised_pixels.tolist(),
        "quantization_bits": args.quantization_bits,
        "max_color_distance": args.max_color_distance,
        "block_size": args.block_size,
        "seed": args.seed,
        "sources": [str(path.expanduser().resolve()) for path in args.input],
        "front_exclusion_mask": 'front_exclusion_mask.png' if exclusion is not None else None,
    }
    atomic_write_json(output / "statistics.json", stats)
    atomic_write_json(output / "split.json", split_summary)
    write_latest(PROCESSED_ROOT, output)
    print(json.dumps(stats, ensure_ascii=False, indent=2))
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
