#!/usr/bin/env python3
"""Collect synchronized MORAI RGB and Semantic camera JPEG pairs over UDP."""

from __future__ import annotations

import argparse
import json
import select
import shutil
import signal
import socket
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from camera_config import validate_sensor_preset, CAMERA_GROUPS
from capture_timing import SampleClock, TopicIndex, PendingRecords
from capture_resume import resume_state

from semantic_common import (
    CAMERA_PAIRS,
    RAW_ROOT,
    SENSOR_PRESET,
    NETWORK_PRESET,
    MoraiTimestampedJpegDecoder,
    atomic_write_bytes,
    atomic_write_json,
    safe_name,
    sha256_file,
    write_latest,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=RAW_ROOT,
    )
    parser.add_argument("--session-id", default=datetime.now().strftime("semantic_%Y%m%d_%H%M%S"))
    parser.add_argument('--resume-session', action='store_true')
    parser.add_argument("--save-fps", type=float, default=5.0, help="saved pairs per second per view")
    parser.add_argument("--every-frame", action="store_true", help="save every synchronized camera frame")
    stop_group = parser.add_mutually_exclusive_group()
    stop_group.add_argument('--until-stopped', action='store_true', help='batch controller stops collection after route completion')
    stop_group.add_argument(
        "--duration-minutes",
        type=float,
        help="MORAI synchronized stream duration; default: 10 minutes",
    )
    stop_group.add_argument(
        "--target-per-view",
        type=int,
        help="stop after this many saved pairs per view",
    )
    parser.add_argument("--sync-tolerance-ms", type=float, default=60.0)
    parser.add_argument("--minimum-free-gb", type=float, default=10.0)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--source-ip", default="", help="accept packets only from this simulator IP")
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--recording-id")
    parser.add_argument("--group", choices=CAMERA_GROUPS, default="all")
    parser.add_argument("--student-session", type=Path)
    parser.add_argument("--telemetry-root", type=Path)
    return parser.parse_args()


def encode_preview(rgb, semantic, label, count, delta_ms, width=560):
    height = max(1, round(rgb.shape[0] * width / rgb.shape[1]))
    rgb = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)
    semantic = cv2.resize(semantic, (width, height), interpolation=cv2.INTER_NEAREST)
    row = np.hstack((rgb, semantic))
    cv2.rectangle(row, (0, 0), (row.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(
        row,
        f"{label}  saved={count}  sync={delta_ms:.1f} ms",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return row



def telemetry_refs(root, timestamp_ns):
    if not root:
        return {"status": "unavailable"}
    try:
        snapshots = json.loads((root / "latest.json").read_text())
    except (OSError, ValueError):
        return {"status": "missing"}
    result = {}
    for topic in ("/Object_topic", "/Ego_topic"):
        row = snapshots.get(topic)
        fresh = row and time.time() - row.get("received_at", 0) < 1
        delta = abs(row.get("timestamp_ns", 0) - timestamp_ns) / 1e6 if row else None
        result[topic] = {"status": "timestamp_candidate" if fresh and row.get("timestamp_ns") and delta <= 60 else "unmatched",
                         "delta_ms": delta, "archive": row.get("archive") if row else None,
                         "offset": row.get("offset") if row else None,
                         "time_basis_verified": False}
    return result


class StudentFeed:
    def __init__(self, root, views):
        self.root, self.views, self.offset = root, views, 0

    def poll(self):
        from semantic_common import EncodedFrame
        path = self.root / "manifest.jsonl"
        if not path.is_file():
            return []
        output = []
        with path.open('rb') as stream:
            stream.seek(self.offset)
            while True:
                line = stream.readline()
                if not line.endswith(b'\n'):
                    break
                self.offset = stream.tell()
                row = json.loads(line)
                if row.get('view') not in self.views or 'rgb_timestamp_ns' not in row:
                    continue
                source = (self.root / row['rgb']).resolve()
                if not source.is_relative_to(self.root.resolve()):
                    raise ValueError('Student image escapes session')
                data = source.read_bytes()
                picture = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if picture is not None:
                    output.append((row['view'], EncodedFrame(row['rgb_timestamp_ns'], data, picture)))
        return output[-120:]


def main() -> int:
    args = parse_args()
    camera_pairs = {v: CAMERA_PAIRS[v] for v in CAMERA_GROUPS[args.group]}
    if args.duration_minutes is None and args.target_per_view is None and not args.until_stopped:
        args.duration_minutes = 10.0
    if args.save_fps <= 0 or args.save_fps > 60:
        raise ValueError("--save-fps must be in (0, 60]")
    if args.duration_minutes is not None and args.duration_minutes <= 0:
        raise ValueError("--duration-minutes must be positive")
    if args.target_per_view is not None and args.target_per_view <= 0:
        raise ValueError("--target-per-view must be positive")
    if args.sync_tolerance_ms <= 0 or args.minimum_free_gb < 0:
        raise ValueError("tolerance must be positive and free-space limit non-negative")
    if not SENSOR_PRESET.is_file():
        raise FileNotFoundError(SENSOR_PRESET)
    validate_sensor_preset(SENSOR_PRESET)

    session = args.output_root.expanduser().resolve() / safe_name(args.session_id)
    if session.exists() and any(session.iterdir()) and not args.resume_session:
        raise FileExistsError(f"session already exists and is not empty: {session}")
    session.mkdir(parents=True, exist_ok=True)
    write_latest(args.output_root.expanduser().resolve(), session)
    for view in camera_pairs:
        (session / "raw" / view / "rgb").mkdir(parents=True, exist_ok=args.resume_session)
        (session / "raw" / view / "semantic").mkdir(parents=True, exist_ok=args.resume_session)

    ports = sorted(
        {pair[key] for pair in camera_pairs.values() for key in (("semantic_port",) if args.student_session else ("rgb_port", "semantic_port"))}
    )
    pair_by_port = {}
    for view, pair in camera_pairs.items():
        pair_by_port[pair["rgb_port"]] = (view, "rgb")
        pair_by_port[pair["semantic_port"]] = (view, "semantic")

    sockets = {}
    decoders = {}
    queues = {view: {"rgb": deque(maxlen=120), "semantic": deque(maxlen=120)} for view in camera_pairs}
    received = {port: 0 for port in ports}
    dropped = {view: {"rgb": 0, "semantic": 0} for view in camera_pairs}
    saved = {view: 0 for view in camera_pairs}
    latest = {view: None for view in camera_pairs}
    last_received_at = {view: None for view in camera_pairs}
    pair_counts = {view: 0 for view in camera_pairs}
    saved_bytes = {view: 0 for view in camera_pairs}
    sample_history = {view: deque(maxlen=30) for view in camera_pairs}
    first_pair_ns = {view: None for view in camera_pairs}
    latest_pair_ns = {view: None for view in camera_pairs}
    interval_ns = 0 if args.every_frame else round(1_000_000_000 / args.save_fps)
    clocks = {view: SampleClock(interval_ns) for view in camera_pairs}
    topic_index = TopicIndex(args.telemetry_root, session.name)
    pending_records = PendingRecords(topic_index)
    telemetry_matches = {view: {topic: 0 for topic in ('/Object_topic','/Ego_topic')} for view in camera_pairs}
    tolerance_ns = round(args.sync_tolerance_ms * 1_000_000)
    duration_ns = (
        round(args.duration_minutes * 60.0 * 1_000_000_000)
        if args.duration_minutes is not None
        else None
    )
    manifest_path = session / "manifest.jsonl"

    student_feed = StudentFeed(args.student_session, camera_pairs) if args.student_session else None
    metadata = {
        "group": args.group,
        "recording_id": args.recording_id or args.session_id,
        "student_session": str(args.student_session) if args.student_session else None,
        "format_version": 1,
        "session_id": safe_name(args.session_id),
        "started_at": datetime.now().astimezone().isoformat(),
        "sensor_preset": str(SENSOR_PRESET),
        "sensor_preset_sha256": sha256_file(SENSOR_PRESET),
        "network_preset": str(NETWORK_PRESET),
        "network_preset_sha256": sha256_file(NETWORK_PRESET),
        "camera_pairs": camera_pairs,
        "save_fps": args.save_fps,
        "every_frame": args.every_frame,
        "duration_minutes": args.duration_minutes,
        "target_per_view": args.target_per_view,
        "sync_tolerance_ms": args.sync_tolerance_ms,
        "source_ip": args.source_ip,
    }
    if args.resume_session:
        previous, counts, first, last = resume_state(session, metadata)
        atomic_write_json(session / ('session_before_resume_' + str(time.time_ns()) + '.json'), previous)
        saved.update(counts); first_pair_ns.update(first); latest_pair_ns.update(last)
        saved_bytes.update(previous['saved_bytes_per_view'])
        received.update({int(k): v for k, v in previous.get('received_frames_per_port', {}).items()})
        for view in camera_pairs:
            clocks[view].due(last[view])
            telemetry_matches[view].update(previous.get('telemetry_matches', {}).get(view, {}))
            dropped[view].update(previous.get('sync_drops', {}).get(view, {}))
        previous.setdefault('resume_segments', []).append({'closed_at': previous['finished_at'],
            'resumed_at': metadata['started_at'], 'saved_before_resume': dict(counts)})
        previous.pop('finished_at'); previous['complete'] = False
        metadata = previous
    shutil.copy2(SENSOR_PRESET, session / "sensor.json")
    shutil.copy2(NETWORK_PRESET, session / "network.json")
    atomic_write_json(session / "session.json", metadata)
    segment_saved = dict(saved)

    stop = False

    def request_stop(_signum=None, _frame=None):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        for port in ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            sock.bind((args.bind_host, port))
            sock.setblocking(False)
            sockets[sock] = port
            decoders[port] = MoraiTimestampedJpegDecoder()
        print("UDP listening: " + ", ".join(str(port) for port in ports), flush=True)
        print(f"save: {session}", flush=True)
        print("exit: q or Ctrl+C", flush=True)

        last_report = time.monotonic()
        started_waiting = last_report
        warned_no_data = False
        with manifest_path.open("a", encoding="utf-8", buffering=1) as manifest:
            while not stop:
                if student_feed:
                    for view, frame in student_feed.poll():
                        queues[view]["rgb"].append(frame)
                ready, _, _ = select.select(list(sockets), [], [], 0.03)
                for sock in ready:
                    for _ in range(64):
                        try:
                            datagram, address = sock.recvfrom(65535)
                        except BlockingIOError:
                            break
                        if args.source_ip and address[0] != args.source_ip:
                            continue
                        port = sockets[sock]
                        frame = decoders[port].feed(datagram)
                        if frame is None:
                            continue
                        received[port] += 1
                        view, kind = pair_by_port[port]
                        queues[view][kind].append(frame)

                for view, pair in camera_pairs.items():
                    rgb_queue = queues[view]["rgb"]
                    semantic_queue = queues[view]["semantic"]
                    while rgb_queue and semantic_queue:
                        rgb = rgb_queue[0]
                        semantic = semantic_queue[0]
                        delta = semantic.timestamp_ns - rgb.timestamp_ns
                        if abs(delta) <= tolerance_ns:
                            rgb_queue.popleft()
                            semantic_queue.popleft()
                            latest[view] = (rgb.image, semantic.image, abs(delta) / 1e6)
                            last_received_at[view] = time.time()
                            pair_counts[view] += 1
                            if first_pair_ns[view] is None:
                                first_pair_ns[view] = rgb.timestamp_ns
                            latest_pair_ns[view] = rgb.timestamp_ns
                            if rgb.image.shape[:2] != semantic.image.shape[:2]:
                                dropped[view]["rgb"] += 1
                                dropped[view]["semantic"] += 1
                                continue
                            if args.target_per_view is not None and saved[view] >= args.target_per_view:
                                continue
                            if not clocks[view].due(rgb.timestamp_ns):
                                continue
                            number = saved[view] + 1
                            stem = f"{number:08d}_{rgb.timestamp_ns}"
                            rgb_relative = Path("raw") / view / "rgb" / f"{stem}.jpg"
                            semantic_relative = Path("raw") / view / "semantic" / f"{stem}.jpg"
                            atomic_write_bytes(session / rgb_relative, rgb.jpeg)
                            atomic_write_bytes(session / semantic_relative, semantic.jpeg)
                            record = {
                                "session_id": metadata["session_id"],
                                "recording_id": metadata["recording_id"],
                                "view": view,
                                "sequence": number,
                                "rgb_sensor_id": pair["rgb_id"],
                                "semantic_sensor_id": pair["semantic_id"],
                                "supervised_class_ids": pair["supervised_class_ids"],
                                "rgb_timestamp_ns": rgb.timestamp_ns,
                                "semantic_timestamp_ns": semantic.timestamp_ns,
                                "sync_delta_ms": abs(delta) / 1e6,
                                "width": int(rgb.image.shape[1]),
                                "height": int(rgb.image.shape[0]),
                                "rgb": str(rgb_relative),
                                "semantic": str(semantic_relative),
                            }
                            try:
                                context = json.loads((session / 'collection_context.json').read_text())
                                record['collection_context'] = context
                            except (OSError, ValueError):
                                pass
                            try:
                                drive = json.loads((session / 'drive_status.json').read_text())
                                if time.time() - drive.get('time', 0) < 2:
                                    record['drive'] = drive
                            except (OSError, ValueError):
                                pass
                            pending_records.append(record)
                            saved[view] = number
                            saved_bytes[view] += len(rgb.jpeg) + len(semantic.jpeg)
                            if sum(saved.values()) % 100 == 0:
                                free = shutil.disk_usage(session).free
                                if free < args.minimum_free_gb * 1024**3:
                                    print(f"minimum free disk limit reached: {free / 1024**3:.1f} GiB", file=sys.stderr)
                                    stop = True
                                    break
                        elif delta < 0:
                            semantic_queue.popleft()
                            dropped[view]["semantic"] += 1
                        else:
                            rgb_queue.popleft()
                            dropped[view]["rgb"] += 1

                pending_records.flush(manifest, telemetry_matches)
                if args.target_per_view is not None and all(
                    value >= args.target_per_view for value in saved.values()
                ):
                    print("target reached", flush=True)
                    break
                if duration_ns is not None and all(
                    first_pair_ns[view] is not None
                    and latest_pair_ns[view] - first_pair_ns[view] >= duration_ns
                    for view in camera_pairs
                ):
                    print(f"duration reached: {args.duration_minutes:g} minutes", flush=True)
                    break

                if not args.no_gui:
                    rows = []
                    for view in camera_pairs:
                        if latest[view] is not None:
                            rgb_image, semantic_image, delta_ms = latest[view]
                            rows.append(
                                encode_preview(
                                    rgb_image,
                                    semantic_image,
                                    view,
                                    saved[view],
                                    delta_ms,
                                )
                            )
                    if rows:
                        canvas = np.vstack(rows)
                        cv2.imshow("MORAI RGB + Semantic collection", canvas)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break

                now = time.monotonic()
                if now - last_report >= 0.5:
                    preview_dir = session / 'preview'
                    preview_dir.mkdir(exist_ok=True)
                    view_status = {}
                    for name in camera_pairs:
                        item = latest[name]
                        if item:
                            rgb_image, semantic_image, delta_ms = item
                            for mode, picture in [('rgb', rgb_image), ('semantic', semantic_image),
                                                  ('overlay', cv2.addWeighted(rgb_image, .5, semantic_image, .5, 0))]:
                                small = cv2.resize(picture, (640, max(1, round(picture.shape[0]*640/picture.shape[1]))))
                                atomic_write_bytes(preview_dir / f'{name}_{mode}.jpg', cv2.imencode('.jpg', small)[1].tobytes())
                        elapsed = max(.001, now - started_waiting)
                        sample_history[name].append((now, saved[name], saved_bytes[name]))
                        history = sample_history[name]
                        window = max(.001, now-history[0][0])
                        recent_fps = (saved[name]-history[0][1])/window
                        recent_bytes = (saved_bytes[name]-history[0][2])/window
                        view_status[name] = {'last_received_at': last_received_at[name], 'receive_fps': pair_counts[name]/elapsed, 'saved': saved[name], 'save_fps': (saved[name]-segment_saved[name])/elapsed, 'recent_save_fps': recent_fps,
                                             'saved_bytes': saved_bytes[name], 'bytes_per_second': recent_bytes,
                                             'stream_seconds': (latest_pair_ns[name]-first_pair_ns[name])/1e9 if first_pair_ns[name] is not None else 0,
                                             'sync_delta_ms': item[2] if item else None,
                                             'last_timestamp_ns': latest_pair_ns[name], 'telemetry_matched': telemetry_matches[name]}
                    atomic_write_json(session / 'capture_status.json', {'time': time.time(), 'views': view_status,
                                      'received': received, 'dropped': dropped, 'group': args.group,
                                      'free_gb': shutil.disk_usage(session).free / 1024**3})
                    if shutil.disk_usage(session).free < args.minimum_free_gb * 1024**3:
                        raise RuntimeError('저장 공간 부족: 수집을 중지합니다.')
                    status = " ".join(f"{view}={saved[view]}" for view in camera_pairs)
                    elapsed_values = [
                        (latest_pair_ns[view] - first_pair_ns[view]) / 1e9
                        for view in camera_pairs
                        if first_pair_ns[view] is not None
                    ]
                    if elapsed_values:
                        status += f" elapsed={min(elapsed_values):.1f}s"
                    missing = [str(port) for port, count in received.items() if count == 0]
                    suffix = f" waiting={','.join(missing)}" if missing else ""
                    print(status + suffix, flush=True)
                    last_report = now
                if not warned_no_data and now - started_waiting >= 5.0 and not any(received.values()):
                    print(
                        "수신 데이터 없음: MORAI에서 object_alpha_semantic.json 적용, Sensor Connect, Play 상태 확인 필요.",
                        file=sys.stderr,
                        flush=True,
                    )
                    warned_no_data = True
    finally:
        # Flush the short metadata queue even on SIGTERM or an exception. Images already exist.
        with manifest_path.open("a", encoding="utf-8", buffering=1) as final_manifest:
            pending_records.flush(final_manifest, telemetry_matches, force=True)
        for sock in sockets:
            sock.close()
        if not args.no_gui:
            cv2.destroyAllWindows()
        metadata["finished_at"] = datetime.now().astimezone().isoformat()
        metadata["saved_per_view"] = saved
        metadata["saved_bytes_per_view"] = saved_bytes
        metadata["sampling_version"] = 2
        metadata["telemetry_alignment_version"] = 2
        metadata["telemetry_matches"] = telemetry_matches
        metadata["received_frames_per_port"] = received
        metadata["sync_drops"] = dropped
        if args.target_per_view is not None:
            metadata["complete"] = all(
                value >= args.target_per_view for value in saved.values()
            )
        elif duration_ns is not None:
            metadata["complete"] = all(
                first_pair_ns[view] is not None
                and latest_pair_ns[view] - first_pair_ns[view] >= duration_ns
                for view in camera_pairs
            )
        else:
            metadata['complete'] = False  # The batch controller verifies route completion separately.
        metadata["stream_duration_seconds_per_view"] = {
            view: (
                (latest_pair_ns[view] - first_pair_ns[view]) / 1e9
                if first_pair_ns[view] is not None
                else 0.0
            )
            for view in camera_pairs
        }
        atomic_write_json(session / "session.json", metadata)
        print("saved: " + " ".join(f"{key}={value}" for key, value in saved.items()), flush=True)
        print(session, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
