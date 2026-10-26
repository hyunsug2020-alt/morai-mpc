#!/usr/bin/env python3
"""Collect MORAI raw RGB (left/front/right) over UDP, every frame, no Semantic dependency.

Standard library only so it runs on the host without cv2. JPEG bytes are saved as-is;
decoding/resizing happens later in student/build_clips.py.
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import select
import shutil
import signal
import socket
import struct
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

VIEWS = {"front": 12002, "left": 12004, "right": 12006}
CAMERA_IDS = {"front": 1, "left": 2, "right": 3}


class FrameAssembler:
    """MORAI 19-byte timestamped UDP JPEG protocol: b'MOR' + <IIII sec, nsec, index, size> ... 'EI'."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.ts = None
        self.expected = None
        self.buf = bytearray()

    def feed(self, datagram):
        if len(datagram) < 21 or not datagram.startswith(b"MOR"):
            return None
        sec, nsec, index, size = struct.unpack("<IIII", datagram[3:19])
        end = 19 + size
        if nsec >= 1_000_000_000 or size <= 0 or end > len(datagram) - 2:
            self.reset()
            return None
        ts = sec * 1_000_000_000 + nsec
        payload = datagram[19:end]
        if index == 0:
            self.ts, self.expected, self.buf = ts, 1, bytearray(payload)
        elif self.expected is None or index != self.expected or ts != self.ts:
            self.reset()
            return None
        else:
            self.buf.extend(payload)
            self.expected += 1
        if datagram[-2:] != b"EI":
            return None
        jpeg = bytes(self.buf)
        frame_ts = self.ts
        self.reset()
        eoi = jpeg.find(b"\xff\xd9")
        if eoi >= 0:
            jpeg = jpeg[: eoi + 2]
        if not jpeg.startswith(b"\xff\xd8"):
            return None
        return frame_ts, jpeg


def atomic_write(path: Path, content: bytes):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(content)
    os.replace(tmp, path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--session-id", required=True)
    p.add_argument("--output-root", type=Path, default=Path(__file__).resolve().parents[1] / "data" / "raw")
    p.add_argument("--duration-seconds", type=float, required=True)
    p.add_argument("--stop-file", type=Path, help="stop and finalize when this session marker appears")
    p.add_argument("--min-free-gb", type=float, default=10.0)
    p.add_argument("--bind-host", default="0.0.0.0")
    args = p.parse_args()

    session = args.output_root.resolve() / args.session_id
    if (session / "manifest.jsonl").exists() or any((session / "raw").rglob("*.jpg")) if (session / "raw").exists() else False:
        raise FileExistsError(f"session already has RGB data: {session}")
    for view in VIEWS:
        (session / "raw" / view / "rgb").mkdir(parents=True, exist_ok=True)

    socks = {}
    for view, port in VIEWS.items():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
        s.bind((args.bind_host, port))
        s.setblocking(False)
        socks[s] = view
    socket_receive_bytes = {view: s.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
                            for s, view in socks.items()}
    asm = {view: FrameAssembler() for view in VIEWS}
    saved = {view: 0 for view in VIEWS}
    received = {view: 0 for view in VIEWS}
    queue_drops = {view: 0 for view in VIEWS}
    accepted_ts = {view: None for view in VIEWS}
    first_ts = {view: None for view in VIEWS}
    last_ts = {view: None for view in VIEWS}
    dup = {view: 0 for view in VIEWS}
    preview_dir = Path(os.environ["STUDENT_CAMERA_CACHE"]) if os.environ.get("STUDENT_CAMERA_CACHE") else None
    if preview_dir is not None:
        preview_dir.mkdir(parents=True, exist_ok=True)
    frames_to_write = queue.Queue(maxsize=1024)
    writer_errors = []
    report_errors = []
    report_stalls = []
    receiver_stalls = []
    queue_peak = 0

    stop = {"flag": False}

    def on_signal(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    started_wall_ns = time.time_ns()
    meta = {
        "format_version": 2,
        "kind": "rgb_only",
        "session_id": args.session_id,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "started_wall_ns": started_wall_ns,
        "views": {v: {"camera_id": CAMERA_IDS[v], "udp_port": VIEWS[v]} for v in VIEWS},
        "duration_seconds": args.duration_seconds,
        "every_frame": True,
        "timestamp_source": "MORAI UDP packet header (host wallclock, verified ~1ms offset)",
        "socket_receive_buffer_bytes": socket_receive_bytes,
    }
    (session / "session.json").write_text(json.dumps(meta, indent=2))
    # A buffered manifest avoids one NTFS/FUSE write per JPEG. The writer
    # flushes once per second so the dashboard can still show live counts.
    manifest = (session / "manifest.jsonl").open("a", buffering=256 * 1024)
    stalls = (session / "capture_stalls.jsonl").open("a", buffering=1)
    report_stop = threading.Event()

    def write_frame():
        last_manifest_flush = time.monotonic()
        while True:
            item = frames_to_write.get()
            if item is None:
                frames_to_write.task_done()
                return
            view, ts, jpeg, received_wall_ns = item
            try:
                if not writer_errors:
                    n = saved[view]
                    rel = Path("raw") / view / "rgb" / f"{n:08d}_{ts}.jpg"
                    began = time.monotonic_ns()
                    # Filenames are unique in a new session. The manifest is
                    # appended only after close, so an interrupted partial
                    # JPEG is never advertised as a saved frame. Avoiding a
                    # temporary file and rename halves metadata operations on
                    # the NTFS recording volume.
                    with (session / rel).open("xb") as output:
                        output.write(jpeg)
                    wrote = time.monotonic_ns()
                    manifest.write(json.dumps({
                        "view": view, "camera_id": CAMERA_IDS[view], "frame_index": n,
                        "rgb_timestamp_ns": ts, "received_wall_ns": received_wall_ns,
                        "rgb": str(rel), "bytes": len(jpeg),
                    }) + "\n")
                    logged = time.monotonic_ns()
                    saved[view] += 1
                    if time.monotonic() - last_manifest_flush >= 1.0:
                        manifest.flush()
                        last_manifest_flush = time.monotonic()
                    last_ts[view] = ts
                    if first_ts[view] is None:
                        first_ts[view] = ts
                    if wrote - began > 100_000_000 or logged - wrote > 100_000_000:
                        stalls.write(json.dumps({"wall_ns": time.time_ns(), "view": view,
                                                 "write_ms": round((wrote - began) / 1e6, 1),
                                                 "manifest_ms": round((logged - wrote) / 1e6, 1),
                                                 "queue_depth": frames_to_write.qsize()}) + "\n")
            except Exception as error:
                writer_errors.append(repr(error))
                stop["flag"] = True
            finally:
                frames_to_write.task_done()

    writer = threading.Thread(target=write_frame, name="rgb-disk-writer", daemon=True)
    writer.start()
    deadline = time.monotonic() + args.duration_seconds
    last_loop = time.monotonic_ns()

    def report_progress():
        warned = False
        while not report_stop.wait(2.0):
            el = [(last_ts[v] - first_ts[v]) / 1e9 for v in VIEWS if first_ts[v] is not None]
            line = " ".join(f"{v}={saved[v]}" for v in VIEWS)
            if el:
                line += f" elapsed={min(el):.1f}s hz≈{min(saved.values()) / max(min(el), 1e-6):.1f}"
            missing = [v for v in VIEWS if received[v] == 0]
            if missing:
                line += f" waiting={','.join(missing)}"
            began = time.monotonic_ns()
            print(line, flush=True)
            logged = time.monotonic_ns()
            try:
                free_gb = shutil.disk_usage(session).free / 1e9
                checked = time.monotonic_ns()
                if logged - began > 100_000_000 or checked - logged > 100_000_000:
                    report_stalls.append({"wall_ns": time.time_ns(), "stage": "progress_report",
                                          "log_ms": round((logged - began) / 1e6, 1),
                                          "disk_check_ms": round((checked - logged) / 1e6, 1)})
                if free_gb < args.min_free_gb:
                    report_errors.append("disk space below minimum")
                    stop["flag"] = True
                    print("디스크 여유 공간 부족, 수집 중단", file=sys.stderr, flush=True)
                    return
            except OSError as error:
                report_errors.append(repr(error))
                stop["flag"] = True
                return
            if not warned and not any(received.values()) and time.monotonic() - (deadline - args.duration_seconds) > 5:
                print("수신 데이터 없음: MORAI에서 rgb_cam.json 적용, Sensor Connect, Play 확인 필요.",
                      file=sys.stderr, flush=True)
                warned = True

    reporter = threading.Thread(target=report_progress, name="rgb-progress", daemon=True)
    reporter.start()
    def watch_stop_file():
        # Metadata reads on the recording volume can block behind a JPEG write.
        # Never stat that volume from the UDP receive loop.
        while not report_stop.wait(.1):
            if args.stop_file and args.stop_file.exists():
                stop["flag"] = True
                return
    stop_watcher = threading.Thread(target=watch_stop_file, name="rgb-stop-file", daemon=True)
    stop_watcher.start()
    try:
        while not stop["flag"] and time.monotonic() < deadline:
            ready, _, _ = select.select(list(socks), [], [], 0.2)
            loop_now = time.monotonic_ns()
            if loop_now - last_loop > 500_000_000:
                receiver_stalls.append({"wall_ns": time.time_ns(), "stage": "receiver_loop",
                                        "gap_ms": round((loop_now - last_loop) / 1e6, 1),
                                        "queue_depth": frames_to_write.qsize()})
            last_loop = loop_now
            for s in ready:
                view = socks[s]
                while True:
                    try:
                        d = s.recv(65535)
                    except BlockingIOError:
                        break
                    received[view] += 1
                    out = asm[view].feed(d)
                    if out is None:
                        continue
                    ts, jpeg = out
                    received_wall_ns = time.time_ns()
                    if accepted_ts[view] is not None and ts <= accepted_ts[view]:
                        dup[view] += 1
                        continue
                    # The live consumer must not wait for the archival disk writer.
                    # This directory is owned by the launch and resides on tmpfs.
                    if preview_dir is not None:
                        atomic_write(preview_dir / (view + ".frame"),
                                     struct.pack("!QQ", ts, received_wall_ns) + jpeg)
                    try:
                        frames_to_write.put_nowait((view, ts, jpeg, received_wall_ns))
                        accepted_ts[view] = ts
                        queue_peak = max(queue_peak, frames_to_write.qsize())
                    except queue.Full:
                        queue_drops[view] += 1
    finally:
        report_stop.set()
        stop_watcher.join(timeout=.2)
        reporter.join()
        frames_to_write.put(None)
        frames_to_write.join()
        writer.join()
        for item in receiver_stalls:
            stalls.write(json.dumps(item) + "\n")
        for item in report_stalls:
            stalls.write(json.dumps(item) + "\n")
        manifest.close()
        stalls.close()
        for s in socks:
            s.close()
        meta["finished_at"] = datetime.now(timezone.utc).isoformat()
        meta["finished_wall_ns"] = time.time_ns()
        meta["saved_per_view"] = saved
        meta["datagrams_per_view"] = received
        meta["duplicate_or_reordered_frames"] = dup
        meta["writer_queue_peak"] = queue_peak
        meta["writer_queue_drops"] = queue_drops
        meta["writer_errors"] = writer_errors
        meta["report_errors"] = report_errors
        meta["stream_duration_seconds_per_view"] = {
            v: ((last_ts[v] - first_ts[v]) / 1e9 if first_ts[v] is not None else 0.0) for v in VIEWS}
        meta["measured_hz_per_view"] = {
            v: (saved[v] / meta["stream_duration_seconds_per_view"][v]
                if meta["stream_duration_seconds_per_view"][v] > 0 else 0.0) for v in VIEWS}
        (session / "session.json").write_text(json.dumps(meta, indent=2))
        print("saved: " + " ".join(f"{k}={v}" for k, v in saved.items()), flush=True)
        print(session, flush=True)
    return 1 if writer_errors or report_errors or any(queue_drops.values()) else 0


if __name__ == "__main__":
    raise SystemExit(main())
