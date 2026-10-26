#!/usr/bin/env python3
"""Shared MORAI Semantic UDP, palette, and image utilities."""

from __future__ import annotations

import hashlib
import json
import os
import uuid
import re
import shutil
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import cv2
import numpy as np
import yaml

from camera_config import CAMERA_PAIRS


ROOT = Path(os.environ.get("MORAI_SEMANTIC_ROOT", Path(__file__).resolve().parents[1]))
DEFAULT_CLASSES = Path(__file__).with_name("classes.yaml")
RAW_ROOT = Path(os.environ.get("MORAI_SEMANTIC_RAW_ROOT", ROOT / "data/raw"))
PROCESSED_ROOT = Path(
    os.environ.get("MORAI_SEMANTIC_PROCESSED_ROOT", ROOT / "data/processed")
)
MODEL_ROOT = Path(os.environ.get("MORAI_SEMANTIC_MODEL_ROOT", ROOT / "data/models"))
SENSOR_PRESET = ROOT / "sensor/object_alpha_semantic.json"
NETWORK_PRESET = ROOT / "network/alphamale_hyunsugboyman.json"


@dataclass(frozen=True)
class EncodedFrame:
    timestamp_ns: int
    jpeg: bytes
    image: np.ndarray


class MoraiTimestampedJpegDecoder:
    """Decode the MORAI 19-byte timestamped UDP JPEG protocol."""

    def __init__(self) -> None:
        self._timestamp_ns = None
        self._expected_index = None
        self._buffer = bytearray()

    def reset(self) -> None:
        self._timestamp_ns = None
        self._expected_index = None
        self._buffer.clear()

    def feed(self, datagram: bytes):
        if len(datagram) < 21 or not datagram.startswith(b"MOR"):
            return None
        seconds, nanoseconds, index, size = struct.unpack("<IIII", datagram[3:19])
        payload_end = 19 + size
        if nanoseconds >= 1_000_000_000 or size <= 0 or payload_end > len(datagram) - 2:
            self.reset()
            return None
        timestamp_ns = seconds * 1_000_000_000 + nanoseconds
        payload = datagram[19:payload_end]
        # MORAI pads UDP camera datagrams to a fixed length. The two-byte
        # AI/EI marker is at the datagram end, not immediately after payload.
        tail = datagram[-2:]

        if index == 0:
            self._timestamp_ns = timestamp_ns
            self._expected_index = 1
            self._buffer = bytearray(payload)
        elif (
            self._expected_index is None
            or index != self._expected_index
            or timestamp_ns != self._timestamp_ns
        ):
            self.reset()
            return None
        else:
            self._buffer.extend(payload)
            self._expected_index += 1

        if tail != b"EI":
            return None

        jpeg = bytes(self._buffer)
        frame_timestamp = int(self._timestamp_ns)
        self.reset()
        eoi = jpeg.find(b"\xff\xd9")
        if eoi >= 0:
            jpeg = jpeg[: eoi + 2]
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return None
        return EncodedFrame(frame_timestamp, jpeg, image)


def load_class_config(path: Path = DEFAULT_CLASSES):
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    classes = sorted(config["classes"], key=lambda item: int(item["id"]))
    expected = list(range(len(classes)))
    actual = [int(item["id"]) for item in classes]
    if actual != expected:
        raise ValueError(f"class IDs must be contiguous from zero: {actual}")
    names = {str(item["name"]): int(item["id"]) for item in classes}
    palette_rgb = []
    palette_ids = []
    for item in config["palette"]:
        class_name = str(item["class"])
        if class_name != "none" and class_name not in names:
            raise ValueError(f"unknown palette class: {class_name}")
        color = [int(value) for value in item["rgb"]]
        if len(color) != 3 or any(value < 0 or value > 255 for value in color):
            raise ValueError(f"invalid RGB palette entry: {color}")
        palette_rgb.append(color)
        palette_ids.append(int(config.get("ignore_index", 255)) if class_name == "none" else names[class_name])
    return config, classes, np.asarray(palette_rgb, np.int16), np.asarray(palette_ids, np.uint8)


def build_palette_lut(palette_rgb, palette_ids, quantization_bits=6, max_distance=50.0, ignore_index=255):
    if quantization_bits < 4 or quantization_bits > 8:
        raise ValueError("quantization_bits must be in [4, 8]")
    bins = 1 << quantization_bits
    shift = 8 - quantization_bits
    centers = np.minimum(255, (np.arange(bins, dtype=np.int16) << shift) + (1 << max(0, shift - 1)))
    rr, gg, bb = np.meshgrid(centers, centers, centers, indexing="ij")
    colors = np.stack((rr.ravel(), gg.ravel(), bb.ravel()), axis=1)
    lut = np.empty(len(colors), np.uint8)
    distance = np.empty(len(colors), np.float32)
    chunk = 32768
    palette = palette_rgb.astype(np.int32)
    for start in range(0, len(colors), chunk):
        values = colors[start : start + chunk].astype(np.int32)
        squared = ((values[:, None, :] - palette[None, :, :]) ** 2).sum(axis=2)
        nearest = squared.argmin(axis=1)
        best = np.sqrt(squared[np.arange(len(values)), nearest])
        mapped = palette_ids[nearest].copy()
        mapped[best > float(max_distance)] = int(ignore_index)
        lut[start : start + len(values)] = mapped
        distance[start : start + len(values)] = best
    return lut, distance


def semantic_bgr_to_mask(image, lut, quantization_bits=6):
    shift = 8 - int(quantization_bits)
    rgb = image[:, :, ::-1].astype(np.uint32)
    bins = 1 << int(quantization_bits)
    index = ((rgb[:, :, 0] >> shift) * bins + (rgb[:, :, 1] >> shift)) * bins + (rgb[:, :, 2] >> shift)
    return lut[index]


def letterbox(image, width, height, interpolation=cv2.INTER_LINEAR, value=0):
    source_height, source_width = image.shape[:2]
    scale = min(float(width) / source_width, float(height) / source_height)
    resized_width = max(1, round(source_width * scale))
    resized_height = max(1, round(source_height * scale))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=interpolation)
    left = (width - resized_width) // 2
    top = (height - resized_height) // 2
    if image.ndim == 2:
        canvas = np.full((height, width), value, dtype=image.dtype)
        canvas[top : top + resized_height, left : left + resized_width] = resized
    else:
        canvas = np.full((height, width, image.shape[2]), value, dtype=image.dtype)
        canvas[top : top + resized_height, left : left + resized_width] = resized
    return canvas, {"left": left, "top": top, "width": resized_width, "height": resized_height}


def atomic_write_bytes(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    temporary.write_bytes(content)
    os.replace(temporary, path)


def atomic_write_json(path: Path, value) -> None:
    atomic_write_bytes(path, (json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))


def write_latest(root: Path, target: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    atomic_write_bytes(root / "LATEST", (str(target.resolve()) + "\n").encode("utf-8"))


def read_latest(root: Path) -> Path:
    marker = root / "LATEST"
    if not marker.is_file():
        raise FileNotFoundError(f"latest marker not found: {marker}")
    target = Path(marker.read_text(encoding="utf-8").strip())
    if not target.exists():
        raise FileNotFoundError(f"latest target not found: {target}")
    return target


def safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("._") or "session"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def link_or_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, destination)
    except OSError:
        shutil.copy2(source, destination)
