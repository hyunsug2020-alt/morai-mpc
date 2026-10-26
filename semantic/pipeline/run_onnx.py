#!/usr/bin/env python3
"""Run B0 on three general RGB views and the traffic-light RGB view."""

from __future__ import annotations

import argparse
import json
import os
import select
import signal
import socket
import time
import threading
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

from camera_config import CAMERA_PAIRS

from semantic_common import (
    MODEL_ROOT,
    MoraiTimestampedJpegDecoder,
    letterbox,
    load_class_config,
    read_latest,
)


def preprocess(image, width, height, exclusion=None):
    if exclusion is not None:
        if exclusion.shape != image.shape[:2]:
            raise ValueError('exclusion mask resolution mismatch')
        image = image.copy()
        image[exclusion > 0] = 0
    canvas, placement = letterbox(image, width, height, cv2.INTER_LINEAR, 0)
    if exclusion is not None:
        resized, _ = letterbox(exclusion, width, height, cv2.INTER_NEAREST, 0)
        canvas[resized > 0] = 0
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    rgb = (rgb - np.asarray((0.485, 0.456, 0.406), np.float32)) / np.asarray(
        (0.229, 0.224, 0.225), np.float32
    )
    return rgb.transpose(2, 0, 1), placement


def traffic_light_hsv(image, mask):
    """Inspect raw RGB camera pixels inside predicted traffic-light boxes."""
    binary = (mask == 3).astype(np.uint8)
    n, _, boxes, _ = cv2.connectedComponentsWithStats(binary, 8)
    states = []
    for component in range(1, n):
        x, y, width, height, area = boxes[component]
        if area < 8 or width < 2 or height < 2:
            continue
        hsv = cv2.cvtColor(image[y:y + height, x:x + width], cv2.COLOR_BGR2HSV)
        color = (hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2])
        bright = (color[1] >= 90) & (color[2] >= 90)
        scores = {
            "red": int((bright & ((color[0] <= 10) | (color[0] >= 170))).sum()),
            "yellow": int((bright & (color[0] >= 15) & (color[0] <= 38)).sum()),
            "green": int((bright & (color[0] >= 40) & (color[0] <= 90)).sum()),
        }
        state = max(scores, key=scores.get)
        if scores[state] < 3:
            state = "unknown"
        states.append({"box": [int(x), int(y), int(width), int(height)], "state": state,
                       "pixels": int(scores.get(state, 0))})
    return states


def build_visualization(name, image, mask, colors, class_names, inference_ms, tile_width=360):
    """Build RGB, class mask, overlay, and a compact visible-class legend."""
    tile_height = max(1, round(image.shape[0] * tile_width / image.shape[1]))
    resized_image = cv2.resize(image, (tile_width, tile_height), interpolation=cv2.INTER_AREA)
    resized_mask = cv2.resize(mask, (tile_width, tile_height), interpolation=cv2.INTER_NEAREST)
    colored = np.zeros_like(resized_image)
    found = resized_mask < len(colors)
    colored[found] = colors[resized_mask[found]]
    overlay = resized_image.copy()
    overlay[found] = cv2.addWeighted(resized_image, 0.56, colored, 0.44, 0)[found]
    tiles = [resized_image, colored, overlay]
    header = np.full((46, tile_width * 3, 3), (23, 33, 28), np.uint8)
    title = f"{name.upper()}  |  ONNX batch {inference_ms:.1f} ms"
    cv2.putText(
        header,
        title,
        (12, 29),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (240, 245, 241),
        2,
        cv2.LINE_AA,
    )
    for index, label in enumerate(("RGB INPUT", "CLASS MASK", "OVERLAY")):
        cv2.rectangle(tiles[index], (0, 0), (tile_width, 28), (14, 22, 18), -1)
        cv2.putText(
            tiles[index],
            label,
            (9, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    counts = np.bincount(resized_mask.reshape(-1), minlength=len(class_names))
    total = max(1, int(counts.sum()))
    present = [
        (class_id, 100.0 * int(count) / total)
        for class_id, count in enumerate(counts[: len(class_names)])
        if count / total >= 0.001
    ]
    present.sort(key=lambda item: item[1], reverse=True)
    present = present[:8]
    columns = 4
    legend_rows = max(1, (len(present) + columns - 1) // columns)
    legend = np.full((14 + legend_rows * 28, tile_width * 3, 3), (247, 246, 240), np.uint8)
    cell_width = legend.shape[1] // columns
    for index, (class_id, percent) in enumerate(present):
        row, column = divmod(index, columns)
        x, y = column * cell_width + 12, 12 + row * 28
        color = tuple(int(value) for value in colors[class_id])
        cv2.rectangle(legend, (x, y), (x + 16, y + 16), color, -1)
        label = f"{class_names[class_id]} {percent:.1f}%"
        cv2.putText(
            legend,
            label,
            (x + 23, y + 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (30, 38, 33),
            1,
            cv2.LINE_AA,
        )
    return np.vstack((header, np.hstack(tiles), legend))


def prediction_masks(prediction, views, output_name="logits", threshold=0.5, excluded_ids=()):
    """Restrict classes before argmax so other channels cannot suppress a signal."""
    if not 0 < threshold < 1:
        raise ValueError("threshold must be between zero and one")
    masks = []
    for values, view in zip(prediction, views):
        active_ids = np.asarray([i for i in CAMERA_PAIRS[view]["supervised_class_ids"] if i not in excluded_ids], dtype=np.uint8)
        if output_name == "logits":
            scores = values[active_ids]
            best = scores.argmax(axis=0)
            mask = active_ids[best]
            mask[scores.max(axis=0) < np.log(threshold / (1.0 - threshold))] = 255
        else:
            mask = values.astype(np.uint8, copy=True)
            mask[~np.isin(mask, active_ids)] = 255
        masks.append(mask)
    return masks


class ManifestFeed:
    def __init__(self, root, filename, views):
        self.root, self.path = root, root / filename
        self.offset = 0
        self.views = set(views)

    def poll(self, paths):
        if not self.path.is_file():
            return
        if self.path.stat().st_size < self.offset:
            self.offset = 0
        with self.path.open("rb") as stream:
            stream.seek(self.offset)
            while line := stream.readline():
                if not line.endswith(b"\n"):
                    break
                row = json.loads(line)
                self.offset = stream.tell()
                if row.get("view") in self.views and "rgb" in row:
                    paths[row["view"]] = self.root / row["rgb"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, help="default: latest run/best.onnx")
    parser.add_argument("--classes", type=Path, default=Path(__file__).with_name("classes.yaml"))
    parser.add_argument("--source-ip", default="")
    parser.add_argument("--session-dir", type=Path, help="read Student-recorded RGB files instead of binding UDP ports")
    parser.add_argument("--aux-session-dir", type=Path, help="sidecar session containing signal RGB in rgb_manifest.jsonl")
    parser.add_argument("--max-batches", type=int, default=0, help="exit after this many inference batches; 0 runs continuously")
    parser.add_argument("--max-fps", type=float, default=20.0)
    parser.add_argument("--duration-seconds", type=float, default=0)
    parser.add_argument("--allow-cpu", action="store_true")
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--output-dir", type=Path, help="write latest overlay JPEGs for a web viewer")
    args = parser.parse_args()
    if args.max_fps <= 0:
        raise ValueError("--max-fps must be positive")
    output_dir = args.output_dir.expanduser().resolve() if args.output_dir else None
    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
    model = (
        args.model.expanduser().resolve()
        if args.model
        else read_latest(MODEL_ROOT) / "best.onnx"
    )
    metadata = json.loads(model.with_suffix(model.suffix + ".json").read_text(encoding="utf-8"))
    if metadata.get("mode") != "foreground_sigmoid" or metadata.get("backbone") not in ("efficientvit_b0.r224_in1k", "efficientvit_b1.r256_in1k") or metadata.get("num_classes") != 6:
        raise ValueError("Only trained six-class EfficientViT-B0/B1 ONNX models are accepted")
    width, height = int(metadata["input_width"]), int(metadata["input_height"])
    exclusion_names = dict(metadata.get('view_exclusion_masks') or {})
    if metadata.get('front_exclusion_mask'):
        exclusion_names.setdefault('front', metadata['front_exclusion_mask'])
    exclusions = {view: cv2.imread(str(model.parent / name), cv2.IMREAD_GRAYSCALE) for view, name in exclusion_names.items()}
    if any(mask is None for mask in exclusions.values()):
        raise ValueError('missing calibrated exclusion mask')
    camera_pairs = {view: pair for view, pair in CAMERA_PAIRS.items()
                    if not metadata.get('general_only') or view != 'signal'}
    class_path = model.parent / "classes.yaml"
    _config, classes, _palette, _ids = load_class_config(class_path if class_path.is_file() else args.classes)
    colors = np.asarray([item["color"] for item in classes], np.uint8)[:, ::-1]
    class_names = [str(item["name"]) for item in classes]
    # classes.yaml excluded_classes: channels the model never learned (e.g. yellow_lane) must not be decoded.
    excluded_ids = tuple(class_names.index(n) for n in (_config.get("excluded_classes") or []) if n in class_names)
    providers = [
        name
        for name in ("CUDAExecutionProvider", "CPUExecutionProvider")
        if name in ort.get_available_providers()
    ]
    if "CUDAExecutionProvider" in providers:
        ort.preload_dlls()
    session = ort.InferenceSession(str(model), providers=providers)
    if session.get_providers()[0] != "CUDAExecutionProvider" and not args.allow_cpu:
        raise RuntimeError("CUDA 실행이 불가능합니다. CPU 전환 대신 GPU 환경을 확인하세요.")
    outputs = {output.name for output in session.get_outputs()}
    output_name = "logits" if "logits" in outputs else str(metadata.get("output", "masks"))
    print(f"ONNX provider: {session.get_providers()[0]}", flush=True)

    rgb_ports = {pair["rgb_port"]: view for view, pair in camera_pairs.items()}
    sockets, decoders, latest = {}, {}, {}
    versions = {view: 0 for view in camera_pairs}
    processed_versions = versions.copy()
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    if not args.session_dir:
        for port in rgb_ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            sock.bind(("0.0.0.0", port))
            sock.setblocking(False)
            sockets[sock] = port
            decoders[port] = MoraiTimestampedJpegDecoder()
    feeds = []
    if args.session_dir:
        views = [view for view, pair in camera_pairs.items() if pair["student_rgb"]] if args.aux_session_dir else camera_pairs
        feeds.append(ManifestFeed(args.session_dir, "manifest.jsonl", views))
    if args.aux_session_dir:
        if not args.session_dir:
            raise ValueError("--aux-session-dir requires --session-dir")
        feeds.append(ManifestFeed(args.aux_session_dir, "rgb_manifest.jsonl",
                                  [view for view, pair in camera_pairs.items() if not pair["student_rgb"]]))
    lock = threading.Lock()
    arrivals = {}
    received_counts = {view: 0 for view in camera_pairs}
    def receive_udp():
        while not stop:
            ready, _, _ = select.select(list(sockets), [], [], .05)
            for sock in ready:
                for _ in range(256):
                    try:
                        data, address = sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    if args.source_ip and address[0] != args.source_ip:
                        continue
                    port = sockets[sock]
                    frame = decoders[port].feed(data)
                    if frame is not None:
                        view = rgb_ports[port]
                        with lock:
                            latest[view] = frame.image
                            versions[view] += 1
                            received_counts[view] += 1
                            arrivals[view] = time.monotonic()
    receiver = threading.Thread(target=receive_udp, daemon=True) if sockets else None
    if receiver:
        receiver.start()
    latest_paths = {}
    last_paths = {}
    last_run = 0.0
    started_waiting = time.monotonic()
    warned = False
    batches = 0
    counts = {view: 0 for view in camera_pairs}
    latencies = {view: deque(maxlen=100000) for view in camera_pairs}
    preview_times = {view: 0 for view in camera_pairs}
    last_status_write = 0
    status = {"model": str(model), "backbone": metadata['backbone'], "views": {}, "target_fps": args.max_fps,
              "provider": session.get_providers()[0], "latency_basis": "host decoded frame to mask", "started_at": time.time()}
    try:
        while not stop:
            if args.duration_seconds and time.monotonic() - started_waiting >= args.duration_seconds:
                break
            if feeds:
                for feed in feeds:
                    feed.poll(latest_paths)
                for view, path in latest_paths.items():
                    if last_paths.get(view) == path:
                        continue
                    frame_image = cv2.imread(str(path))
                    if frame_image is not None:
                        latest[view] = frame_image
                        versions[view] += 1
                        arrivals[view] = time.monotonic()
                        received_counts[view] += 1
                        last_paths[view] = path
                time.sleep(0.02)
            else:
                time.sleep(.001)
            now = time.monotonic()
            if not warned and len(latest) < len(camera_pairs) and now - started_waiting > 5.0:
                missing = sorted(set(camera_pairs) - set(latest))
                print("RGB 수신 대기: " + ", ".join(missing), flush=True)
                warned = True
            with lock:
                views = [view for view in camera_pairs if versions[view] > processed_versions[view] and now - arrivals.get(view, 0) < 1]
                ordered = [latest[name] for name in views]
                selected_versions = {name: versions[name] for name in views}
                selected_arrivals = {name: arrivals[name] for name in views}
            if output_dir and now - last_status_write > 1:
                status.update(time=time.time(), missing_views=[v for v in camera_pairs if now-arrivals.get(v,0)>1])
                from semantic_common import atomic_write_json
                atomic_write_json(output_dir / 'status.json', status)
                last_status_write = now
            if not views or now - last_run < 1.0 / args.max_fps:
                if not args.no_gui and cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue
            prepared = [preprocess(image, width, height, exclusions.get(name))
                        for name, image in zip(views, ordered)]
            batch = np.stack([item[0] for item in prepared])
            inference_started = time.perf_counter()
            prediction = session.run([output_name], {"input": batch})[0]
            inference_ms = (time.perf_counter() - inference_started) * 1000.0
            masks = prediction_masks(prediction, views, output_name, float(metadata.get("threshold", 0.5)), excluded_ids)
            rows = []
            status.update(time=time.time(), inference_ms=round(inference_ms, 2),
                          missing_views=[v for v in camera_pairs if now-arrivals.get(v,0)>1])
            for name, image, mask, (_, placement) in zip(views, ordered, masks, prepared):
                crop = mask[
                    placement["top"] : placement["top"] + placement["height"],
                    placement["left"] : placement["left"] + placement["width"],
                ]
                crop = cv2.resize(crop, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
                if exclusions.get(name) is not None:
                    crop[exclusions[name] > 0] = 255
                components, _, boxes, centers = cv2.connectedComponentsWithStats((crop == 3).astype(np.uint8), 8)
                states = [{'box': [int(v) for v in boxes[i][:4]], 'center': [float(v) for v in centers[i]],
                           'pixels': int(boxes[i][4])} for i in range(1, components) if boxes[i][4] >= 2] if name == 'signal' else []
                elapsed = max(.001, time.monotonic() - started_waiting)
                counts[name] += 1
                latencies[name].append((time.monotonic() - selected_arrivals[name]) * 1000)
                status["views"][name] = {"traffic_lights": states, "time": time.time(),
                    "fps": counts[name] / elapsed, "frames": counts[name],
                    "p95_latency_ms": float(np.percentile(latencies[name], 95)),
                    "rgb_port": CAMERA_PAIRS[name]["rgb_port"],
                    "supervised_class_ids": CAMERA_PAIRS[name]["supervised_class_ids"],
                    "pixel_counts": {class_names[i]: int((crop == i).sum()) for i in CAMERA_PAIRS[name]["supervised_class_ids"]}}
                processed_versions[name] = selected_versions[name]
                if output_dir:
                    from semantic_common import atomic_write_bytes
                    atomic_write_bytes(output_dir / f'{name}_mask.png', cv2.imencode('.png', crop)[1].tobytes())
                if args.no_gui and time.monotonic() - preview_times[name] < .2:
                    continue
                preview_times[name] = time.monotonic()
                visualization = build_visualization(
                    name,
                    image,
                    crop,
                    colors,
                    class_names,
                    inference_ms,
                )
                rows.append(visualization)
                if output_dir:
                    ok, encoded = cv2.imencode(".jpg", visualization, [cv2.IMWRITE_JPEG_QUALITY, 88])
                    if not ok:
                        raise OSError(f"failed to encode inference preview: {name}")
                    temporary = output_dir / f".{name}.jpg.tmp"
                    temporary.write_bytes(encoded.tobytes())
                    os.replace(temporary, output_dir / f"{name}.jpg")
            if output_dir:
                temporary = output_dir / ".status.json.tmp"
                temporary.write_text(json.dumps(status, ensure_ascii=False))
                os.replace(temporary, output_dir / "status.json")
            if not args.no_gui and rows:
                canvas = np.vstack(rows)
                scale = min(1.0, 1500 / canvas.shape[1], 1000 / canvas.shape[0])
                if scale < 1.0:
                    canvas = cv2.resize(
                        canvas,
                        (round(canvas.shape[1] * scale), round(canvas.shape[0] * scale)),
                        interpolation=cv2.INTER_AREA,
                    )
                cv2.imshow("MORAI RGB semantic ONNX", canvas)
            last_run = now
            batches += 1
            if args.max_batches > 0 and batches >= args.max_batches:
                break
            if not args.no_gui and cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        stop = True
        if receiver:
            receiver.join(timeout=2)
        if output_dir:
            from semantic_common import atomic_write_json
            elapsed = max(.001, time.monotonic() - started_waiting)
            report = {'elapsed_seconds': elapsed, 'backbone': metadata['backbone'], 'target_fps': args.max_fps,
                      'provider': session.get_providers()[0], 'views': status['views'],
                      'fps_pass': all(counts[v] / elapsed >= args.max_fps * .99 for v in camera_pairs),
                      'latency_pass': all(latencies[v] and np.percentile(latencies[v], 95) <= 100 for v in camera_pairs),
                      'hardware_stability_verified': False,
                      'note': 'GPU memory, MORAI and Student coexistence require external monitoring.'}
            atomic_write_json(output_dir / 'benchmark.json', report)
        for sock in sockets:
            sock.close()
        if not args.no_gui:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
