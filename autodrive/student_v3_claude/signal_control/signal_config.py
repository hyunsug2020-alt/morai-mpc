"""Portable signal-classifier settings with target-project path overrides."""
import copy
import json
import math
import os
from pathlib import Path
from .permissions import POLICIES, LEGACY

PROJECT_ROOT = Path(os.environ.get("AUTODRIVE_ROOT", Path.cwd())).resolve()
CONFIG_PATH = Path(os.environ.get(
    "SIGNAL_CONFIG_PATH", PROJECT_ROOT / "config/traffic_signal.json"
)).resolve()
MODEL_PATH = Path(os.environ.get(
    "SIGNAL_MODEL_PATH", PROJECT_ROOT / "models/traffic_light.onnx"
)).resolve()

DEFAULT_CONFIG = {
    "roi": [240 / 1280, 0, 1100 / 1280, 1],
    "reference_size": [1280, 720],
    "layout": "four",
    "maneuver": "straight",
    "semantic_threshold": 0.5,
    "s_min": 90,
    "v_min": 90,
    "red_h": [0, 10, 170, 179],
    "yellow_h": [15, 38],
    "green_h": [40, 95],
    "min_pixels": 3,
    "min_fraction": 0.012,
    "min_semantic_area": 20,
    "stable_frames": 3,
    "stale_seconds": 1.0,
    "max_fps": 5.0,
    "stop_on_yellow": True,
    "permission_policy": LEGACY,
}


def validate_config(value):
    if not isinstance(value, dict) or set(value) - set(DEFAULT_CONFIG):
        raise ValueError("unknown traffic-signal setting")
    result = copy.deepcopy(DEFAULT_CONFIG)
    result.update(value)
    roi = result["roi"]
    if not isinstance(roi, list) or len(roi) != 4 or any(
        isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in roi
    ) or not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1):
        raise ValueError("ROI must be [left, top, right, bottom] in 0..1")
    if roi[2] - roi[0] < 0.01 or roi[3] - roi[1] < 0.01:
        raise ValueError("ROI is too small")
    if result["layout"] not in ("three", "four") or result["maneuver"] not in ("straight", "left"):
        raise ValueError("invalid lamp layout or maneuver")
    if result["permission_policy"] not in POLICIES:
        raise ValueError("invalid signal permission policy")
    for key, lo, hi, integer in (
        ("semantic_threshold", 0.05, 0.95, False), ("s_min", 0, 255, True),
        ("v_min", 0, 255, True), ("min_pixels", 1, 1000, True),
        ("min_fraction", 0, 0.5, False), ("min_semantic_area", 1, 100000, True),
        ("stable_frames", 1, 30, True), ("stale_seconds", 0.2, 5, False),
        ("max_fps", 0.5, 20, False),
    ):
        number = result[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or not lo <= number <= hi:
            raise ValueError(key + " out of range")
        if integer and int(number) != number:
            raise ValueError(key + " must be an integer")
        result[key] = int(number) if integer else float(number)
    for key, length in (("red_h", 4), ("yellow_h", 2), ("green_h", 2)):
        hue = result[key]
        if not isinstance(hue, list) or len(hue) != length or any(
            isinstance(x, bool) or not isinstance(x, int) or not 0 <= x <= 179 for x in hue
        ) or any(hue[i] > hue[i + 1] for i in range(0, length, 2)):
            raise ValueError(key + " must use OpenCV hue values 0..179")
    if result["reference_size"] != [1280, 720]:
        raise ValueError("reference_size must be 1280x720")
    if result["stop_on_yellow"] is not True:
        raise ValueError("yellow-stop behavior cannot be disabled")
    return result


def load_config(path=None):
    target = Path(path) if path is not None else CONFIG_PATH
    try:
        return validate_config(json.loads(target.read_text(encoding="utf-8")))
    except FileNotFoundError:
        return copy.deepcopy(DEFAULT_CONFIG)


def save_config(value, path=None):
    config = validate_config(value)
    target = Path(path) if path is not None else CONFIG_PATH
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(str(temporary), str(target))
    return config
