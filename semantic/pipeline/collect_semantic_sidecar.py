#!/usr/bin/env python3
"""Record four semantic cameras while Student owns all four RGB ports."""
from __future__ import annotations

import argparse
import json
import select
import signal
import socket
import time
from pathlib import Path

from camera_config import validate_sensor_preset
from semantic_common import CAMERA_PAIRS, SENSOR_PRESET, MoraiTimestampedJpegDecoder, atomic_write_bytes, atomic_write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--save-fps", type=float, default=5)
    args = parser.parse_args()
    if not 0 < args.save_fps <= 30:
        raise ValueError("save-fps must be in (0,30]")
    session = args.session.resolve()
    session.mkdir(parents=True, exist_ok=True)
    validate_sensor_preset(SENSOR_PRESET)
    atomic_write_json(session / "session.json", {"camera_pairs": CAMERA_PAIRS, "save_fps": args.save_fps,
                                               "rgb_source": "Student for front/left/right/signal"})
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    sockets = {}
    decoders = {}
    streams = {(view, "semantic"): pair["semantic_port"] for view, pair in CAMERA_PAIRS.items()}
    streams.update({(view, "rgb"): pair["rgb_port"] for view, pair in CAMERA_PAIRS.items() if not pair["student_rgb"]})
    counts = {key: 0 for key in streams}
    last_saved = {key: None for key in streams}
    try:
        for (view, kind), port in streams.items():
            (session / "raw" / view / kind).mkdir(parents=True, exist_ok=True)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
            sock.bind(("0.0.0.0", port))
            sock.setblocking(False)
            sockets[sock] = (view, kind)
            decoders[(view, kind)] = MoraiTimestampedJpegDecoder()
        print("sidecar UDP ready: " + ", ".join(str(port) for port in streams.values()), flush=True)
        interval = round(1e9 / args.save_fps)
        with (session / "semantic_manifest.jsonl").open("x", buffering=1) as semantic_manifest, \
                (session / "rgb_manifest.jsonl").open("x", buffering=1) as rgb_manifest:
            while not stop:
                ready, _, _ = select.select(list(sockets), [], [], 0.2)
                for sock in ready:
                    key = sockets[sock]
                    view, kind = key
                    frame = decoders[key].feed(sock.recv(65535))
                    previous = last_saved[key]
                    if frame is None or (previous is not None and 0 <= frame.timestamp_ns - previous < interval):
                        continue
                    counts[key] += 1
                    last_saved[key] = frame.timestamp_ns
                    relative = Path("raw") / view / kind / f"{counts[key]:08d}_{frame.timestamp_ns}.jpg"
                    atomic_write_bytes(session / relative, frame.jpeg)
                    manifest = semantic_manifest if kind == "semantic" else rgb_manifest
                    manifest.write(json.dumps({"view": view, f"{kind}_timestamp_ns": frame.timestamp_ns,
                                               kind: str(relative), "width": frame.image.shape[1],
                                               "height": frame.image.shape[0],
                                               "supervised_class_ids": CAMERA_PAIRS[view]["supervised_class_ids"]}) + "\n")
    finally:
        for sock in sockets:
            sock.close()
        print("sidecar saved " + json.dumps({f"{view}/{kind}": count for (view, kind), count in counts.items()}), flush=True)


if __name__ == "__main__":
    main()
