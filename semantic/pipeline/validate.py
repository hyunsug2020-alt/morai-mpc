#!/usr/bin/env python3
"""Validate a prepared MORAI semantic-segmentation dataset."""

from __future__ import annotations

import argparse
import json
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from camera_config import supervised_class_ids

from semantic_common import PROCESSED_ROOT, load_class_config, read_latest, atomic_write_json


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, help="default: latest processed dataset")
    args = parser.parse_args()
    root = args.data.expanduser().resolve() if args.data else read_latest(PROCESSED_ROOT)
    config, classes, _rgb, _ids = load_class_config(root / "classes.yaml")
    valid_ids = set(range(len(classes))) | {int(config.get("ignore_index", 255))}
    errors = []
    counts = Counter()
    manifest = root / "manifest.jsonl"
    if not manifest.is_file():
        raise FileNotFoundError(manifest)
    with manifest.open(encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream]
    assignments = {}
    stats_path = root / 'statistics.json'
    stats = json.loads(stats_path.read_text()) if stats_path.is_file() else {}
    for record in records:
        if record.get('view') not in ('front','left','right','signal'):
            errors.append('unknown camera view')
            continue
        if stats.get('split_method') == 'session':
            group = record.get('recording_id', record['session_id'])
            assignments.setdefault(group, set()).add(record['split'])
        if stats.get('label_version') == 2:
            valid_path = root / record.get('valid_mask', '__missing__')
            valid_mask = cv2.imread(str(valid_path), cv2.IMREAD_UNCHANGED)
            if valid_mask is None or valid_mask.ndim != 2 or not set(np.unique(valid_mask)).issubset({0,1}):
                errors.append('invalid or missing validity mask')

        image_path = root / record["image"]
        mask_path = root / record["mask"]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
        key = f"{record['session_id']}/{record['view']}/{record['sequence']}"
        if image is None or mask is None:
            errors.append(f"{key}: unreadable image or mask")
            continue
        if mask.ndim != 2 or mask.dtype != np.uint8:
            errors.append(f"{key}: mask must be uint8 single-channel")
            continue
        if image.shape[:2] != mask.shape:
            errors.append(f"{key}: image/mask shape mismatch")
        unknown = set(int(value) for value in np.unique(mask)) - valid_ids
        if unknown:
            errors.append(f"{key}: invalid mask IDs {sorted(unknown)}")
        active_ids = set(supervised_class_ids(record))
        unexpected = set(int(value) for value in np.unique(mask)) - active_ids - {int(config.get("ignore_index", 255))}
        if unexpected:
            errors.append(f"{key}: mask contains unsupervised classes {sorted(unexpected)}")
        counts[(record["split"], record["view"])] += 1
    if any(len(splits) > 1 for splits in assignments.values()):
        errors.append("recording appears in multiple splits")
    print(f"samples={len(records)} errors={len(errors)}")
    for key in sorted(counts):
        print(f"{key[0]}/{key[1]}={counts[key]}")
    for error in errors[:100]:
        print("ERROR " + error)
    report = {'time': time.time(), 'ok': not errors, 'samples': len(records), 'error_count': len(errors),
              'errors': errors[:30], 'counts': {f'{split}/{view}': count for (split,view),count in counts.items()},
              'label_version': stats.get('label_version', 1), 'split_method': stats.get('split_method', 'legacy'),
              'model_quality_verified': False}
    atomic_write_json(root / 'validation.json', report)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
