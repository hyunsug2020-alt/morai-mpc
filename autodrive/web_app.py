#!/usr/bin/env python3
"""Autodrive dashboard: one page to run the E2E competition launch (student_v3_follow_claude.sh, code and
data in autodrive/student_v3_claude), collect RGB+Ego+Route sessions with it, and (full mode only) the old
Teacher/Student training steps. Host process, standard library only.

  python3 autodrive/web_app.py --port 8767
"""
import argparse
import fcntl
import json
import math
import os
import select
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "autodrive"))
from competition_profiles import DEFAULT_PROFILE, profile as competition_profile
from web_sensor_preview import SensorPreview
from student_v3_claude.signal_control.launch import command_with_signal_setting
from student_v3_claude.signal_control.debug_store import SignalDebugStore, VIEWS as SIGNAL_VIEWS
WEB = Path(__file__).resolve().parent / "web"
DATA = ROOT / "data"
RAW = DATA / "raw"
STATUS_JSON = DATA / "autodrive" / "status.json"
SOURCE = DATA / "student" / "source.jsonl"
CONTAINER = os.environ.get("AUTODRIVE_CONTAINER", "morai_autodrive")
MODE = os.environ.get("AUTODRIVE_MODE", "full")  # "collect" = data-collection PC: no Teacher/Student steps
CAMERA_PORTS = {view: spec["udp_port"] for view, spec in competition_profile("single_pc")["cameras"].items()}
# The one driving stack (E2E: v6.3 model + portable static/tunnel avoidance). The rule-based
# student_v3 launch was removed 2026-10-11; route/HD-map/tunnel data are read from this copy too.
ASSETS = ROOT / "autodrive/student_v3_claude"
LAUNCH = ROOT / "autodrive/student_v3_follow_claude.sh"
VIEWS = ("front", "left", "right")
SENSOR_PREVIEW = SensorPreview()
SIGNAL_DEBUG = SignalDebugStore()


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


MPC_FILES = {"mpc.csv", "mpc.json", "mpc_emissions.jsonl", "mpc_lifecycle.jsonl", "ltv_speed_profile.csv", "run.json"}


def mpc_runs():
    runs = []
    for path in (DATA / "autodrive/mpc_runs").glob("*/*/run.json"):
        meta = read_json(path, {}) or {}
        files = [{"name": name, "size": (path.parent / name).stat().st_size}
                 for name in sorted(MPC_FILES) if (path.parent / name).is_file()]
        runs.append({**meta, "session_id": path.parent.parent.name, "run": path.parent.name, "files": files})
    return sorted(runs, key=lambda r: r.get("started_wall_ns", 0), reverse=True)[:100]


def mpc_file(session, run, name):
    if name not in MPC_FILES or not session or not run:
        raise ValueError("MPC 로그 파일 지정 오류")
    for component in (session, run):
        if Path(component).name != component or component in (".", ".."):
            raise ValueError("MPC 로그 경로 오류")
    root = (DATA / "autodrive/mpc_runs").resolve()
    target = (root / session / run / name).resolve()
    if root not in target.parents or not target.is_file():
        raise ValueError("MPC 로그 없음")
    return target


def delete_local_session(session, data=DATA, raw=RAW):
    """Delete one finished capture and all its MPC runs, serialized with cloud backup."""
    if not session.isdecimal() or len(session) > 12:
        raise ValueError("숫자 세션 번호 필요")
    raw_root = raw.resolve()
    mpc_root = (data / "autodrive/mpc_runs").resolve()
    capture = raw_root / session
    controller = mpc_root / session
    legacy_diagnostics = data / "autodrive" / ("student_gui_" + session + ".jsonl")
    lock_path = data / "autodrive/cloud_backup/session_locks" / (session + ".lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("이 세션은 Drive 백업 중이라 삭제할 수 없음")
        backup = read_json(data / "autodrive/cloud_backup/status.json", {}) or {}
        if backup.get("session") == session and backup.get("phase") in ("packing", "uploading", "paused", "verifying", "pruning"):
            raise RuntimeError("이 세션은 Drive 백업 중이라 삭제할 수 없음")
        if capture.is_symlink() or controller.is_symlink():
            raise ValueError("심볼릭 링크 세션은 삭제할 수 없음")
        targets = [p for p in (capture, controller) if p.is_dir()]
        if not targets and not legacy_diagnostics.is_file():
            raise ValueError("삭제할 로컬 세션 없음")
        if capture.is_dir() and not (read_json(capture / "session.json", {}) or {}).get("finished_at"):
            raise RuntimeError("수집 중인 세션은 삭제할 수 없음")
        for run in controller.glob("*/run.json") if controller.is_dir() else ():
            if not (read_json(run, {}) or {}).get("ended_wall_ns"):
                raise RuntimeError("MPC가 기록 중인 세션은 삭제할 수 없음")
        active = read_json(data / "autodrive/drive_run.json", {}) or {}
        if active.get("session_id") == session:
            try:
                os.kill(int(active["owner_pid"]), 0)
            except (OSError, ValueError, KeyError):
                pass
            else:
                raise RuntimeError("현재 주행 중인 세션은 삭제할 수 없음")
        deleted = data / "autodrive/cloud_backup/deleted_sessions"
        deleted.mkdir(parents=True, exist_ok=True)
        marker = deleted / (session + ".json")
        had_capture, had_mpc = capture.is_dir(), controller.is_dir()
        # Keep a tombstone so automatic numbering never reuses this session ID.
        temp = marker.with_suffix(".json.tmp")
        temp.write_text(json.dumps({"session": session, "deleted_at": time.time(),
                                    "local_capture": had_capture, "local_mpc": had_mpc}))
        os.replace(temp, marker)
        for target in targets:
            shutil.rmtree(target)
        legacy_diagnostics.unlink(missing_ok=True)
        # No archive should later upload a deleted session's local staging ZIP.
        staging = raw_root.parent / "cloud_staging"
        for suffix in (".zip", ".part"):
            (staging / ("session_" + session + suffix)).unlink(missing_ok=True)
        return {"session": session, "capture": had_capture, "mpc": had_mpc}


def safe_session_id(value):
    value = str(value or "").strip()
    if not value:
        value = time.strftime("drive_%Y%m%d_%H%M%S")
    if len(value) > 80 or not all(c.isalnum() or c in "-_" for c in value):
        raise ValueError("세션 이름은 영문/숫자/-/_ 80자 이하")
    return value


def next_session_id():
    existing = [int(p.name) for p in RAW.iterdir() if p.is_dir() and p.name.isdecimal()] if RAW.is_dir() else []
    deleted = DATA / "autodrive/cloud_backup/deleted_sessions"
    existing.extend(int(p.stem) for p in deleted.glob("*.json") if p.stem.isdecimal())
    mpc_root = DATA / "autodrive/mpc_runs"
    if mpc_root.is_dir():
        existing.extend(int(p.name) for p in mpc_root.iterdir() if p.is_dir() and p.name.isdecimal())
    return str(max(existing + [0]) + 1)


def number(value, name, lo, hi, integer=False):
    try:
        v = int(value) if integer else float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: 숫자 필요")
    if not lo <= v <= hi:
        raise ValueError(f"{name}: {lo}~{hi} 범위")
    return v


def run(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=str(ROOT))
        return r.returncode, (r.stdout + r.stderr).strip()
    except (subprocess.TimeoutExpired, OSError) as e:
        return 1, str(e)


JOBS_DIR = DATA / "autodrive" / "jobs"


class JobManager:
    """Runs one job at a time. The child's output goes to a log FILE (not a pipe), so a dashboard restart neither
    kills the job (own session) nor breaks its stdout; on startup a still-running job is re-adopted from current.json."""

    def __init__(self):
        self.lock = threading.Lock()
        self.process = None      # Popen when started by this dashboard instance
        self.adopted_pid = None  # pid of a job started by a previous instance
        self.action = None
        self.command = []
        self.started_at = self.finished_at = self.exit_code = None
        self.log_path = None
        JOBS_DIR.mkdir(parents=True, exist_ok=True)
        self._adopt()

    def _adopt(self):
        cur = read_json(JOBS_DIR / "current.json")
        if not cur:
            return
        self.action, self.command, self.started_at, self.log_path = cur.get("action"), cur.get("command", []), cur.get("started_at"), Path(cur["log"])
        try:
            os.kill(int(cur["pid"]), 0)
            self.adopted_pid = int(cur["pid"])
        except (OSError, ValueError):
            self.adopted_pid = None
            self.finished_at = cur.get("finished_at") or self.log_path.stat().st_mtime if self.log_path.exists() else None
            self.exit_code = cur.get("exit_code")

    def running(self):
        if self.process is not None:
            return self.process.poll() is None
        if self.adopted_pid:
            try:
                os.kill(self.adopted_pid, 0)
                return True
            except OSError:
                self.adopted_pid = None
                self.finished_at = time.time()
                self.exit_code = self.exit_code if self.exit_code is not None else -1
        return False

    def logs(self, limit=400):
        if not self.log_path or not Path(self.log_path).exists():
            return []
        try:
            with open(self.log_path, "rb") as f:
                f.seek(0, 2)
                size = f.tell()
                f.seek(max(0, size - 200_000))
                lines = f.read().decode("utf-8", "replace").splitlines()
            return lines[-limit:]
        except OSError:
            return []

    def snapshot(self):
        with self.lock:
            running = self.running()
            return {"running": running, "action": self.action, "command": self.command, "started_at": self.started_at,
                    "finished_at": self.finished_at, "exit_code": None if running else self.exit_code,
                    "elapsed_seconds": (time.time() - self.started_at) if running and self.started_at else 0.0,
                    "log_file": str(self.log_path) if self.log_path else None, "logs": self.logs()}

    def start(self, action, command):
        with self.lock:
            if self.running():
                raise RuntimeError(f"이미 {self.action} 작업 실행 중")
            self.log_path = JOBS_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_{action}.log"
            log = open(self.log_path, "ab", buffering=0)
            log.write(("$ " + " ".join(command) + "\n").encode())
            self.action, self.command = action, command
            self.started_at, self.finished_at, self.exit_code, self.adopted_pid = time.time(), None, None, None
            env = dict(os.environ, PYTHONUNBUFFERED="1", ROS_CONTAINER=CONTAINER)
            # Wait for an in-flight preview to close its UDP sockets before launching receivers.
            with SENSOR_PREVIEW.lock:
                SENSOR_PREVIEW.clear()
                self.process = subprocess.Popen(command, cwd=str(ROOT), env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
            log.close()
            (JOBS_DIR / "current.json").write_text(json.dumps({"pid": self.process.pid, "action": action, "command": command, "started_at": self.started_at, "log": str(self.log_path)}))
            threading.Thread(target=self._wait, args=(self.process,), daemon=True).start()

    def _wait(self, process):
        code = process.wait()
        with self.lock:
            if self.process is process:
                self.exit_code, self.finished_at = code, time.time()
                with open(self.log_path, "a") as f:
                    f.write(f"[종료 코드 {code}]\n")
                cur = read_json(JOBS_DIR / "current.json") or {}
                cur.update({"exit_code": code, "finished_at": self.finished_at})
                (JOBS_DIR / "current.json").write_text(json.dumps(cur))

    def stop(self):
        with self.lock:
            if not self.running():
                return False
            pid = self.process.pid if self.process is not None else self.adopted_pid
            with open(self.log_path, "a") as f:
                f.write("[사용자 중지 요청]\n")
        try:
            os.killpg(pid, signal.SIGINT)
            if self.action == "autodrive_collect":
                grace = 60.0  # sensor collection: the launch's cleanup also finalizes the session
            elif any("student_v3_follow" in part for part in self.command):
                grace = 20.0  # competition launch: stops the car first, then shuts down in parallel
            else:
                grace = 15.0
            threading.Timer(grace, lambda: self._force(pid)).start()
        except ProcessLookupError:
            pass
        return True

    def _force(self, pid):
        with self.lock:
            if self.process is not None:
                if self.process.pid != pid or self.process.poll() is not None:
                    return
            elif self.adopted_pid != pid:
                return
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


JOBS = JobManager()


DRIVABLE_RULES = DATA / "autodrive/drivable_rules.json"


def _flag(value):
    """link_set.json stores booleans as strings ("False" is truthy in Python)."""
    return str(value).strip().lower() == "true"


def _link_ref(value):
    return None if value in (None, "", "None") else value


MISSION_SECTIONS = [
    # Route distances (m) from morai-scenario-studio public/missions.js + engine.js, and the
    # competition GPS blackout (tunnel) of student_v3_follow_claude.sh. Starting point for the editor.
    {"type": "signal", "label": "신호등 1", "start_m": 204, "end_m": 253},
    {"type": "static", "label": "신호등 + 정적", "start_m": 283, "end_m": 385},
    {"type": "dynamic", "label": "신호등 + 횡단 보행자", "start_m": 408, "end_m": 499},
    {"type": "signal", "label": "신호등 2", "start_m": 571, "end_m": 642},
    {"type": "overpass", "label": "고가도로 진입·합류", "start_m": 1040, "end_m": 1745},
    {"type": "tunnel", "label": "터널 (GPS 차단, 양쪽 차선 사용)", "start_m": 1879, "end_m": 1999},
    {"type": "static", "label": "터널 정적 장애물", "start_m": 1890, "end_m": 1990},
]
SECTION_TYPES = ("static", "dynamic", "tunnel", "overpass", "signal", "intersection", "normal")


def route_lanes():
    """HD-map lanes around the global route only, classified against the route direction."""
    import numpy as np
    route = read_json(ASSETS / "global_path.json") or {}
    pts = np.asarray([(p["x"], p["y"]) for p in route.get("waypoints") or []], dtype=float)
    if len(pts) < 2:
        return {"route": [], "lanes": [], "length_m": 0}
    station = np.r_[0.0, np.cumsum(np.hypot(*np.diff(pts, axis=0).T))]
    tangent = np.gradient(pts, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    links = read_json(ASSETS / "link_set.json") or []
    lanes, route_ids = [], set()
    for link in links:
        lp = np.asarray([p[:2] for p in link.get("points") or []], dtype=float)
        if len(lp) < 2:
            continue
        d2 = ((lp[:, None, :] - pts[None, ::4, :]) ** 2).sum(-1)
        nearest = d2.argmin(1) * 4
        dist = np.sqrt(d2.min(1))
        near = dist <= 12.0
        if near.sum() < max(3, 0.3 * len(lp)):
            continue
        lt = np.gradient(lp, axis=0)
        lt /= np.maximum(np.linalg.norm(lt, axis=1, keepdims=True), 1e-9)
        align = float(np.median((lt[near] * tangent[nearest[near]]).sum(1)))
        if abs(align) < 0.7:
            continue                                   # crossing / turning roads
        on_route = align > 0 and np.mean(dist <= 1.0) >= 0.5
        kind = "route" if on_route else ("same" if align > 0 else "opposite")
        if on_route:
            route_ids.add(link.get("idx"))
        step = max(1, len(lp) // 40)
        sampled = np.vstack([lp[::step], lp[-1:]])
        lanes.append({"id": link.get("idx"), "kind": kind, "road": link.get("road_id"), "lane": link.get("ego_lane"),
                      "s0": round(float(station[nearest[near]].min()), 1), "s1": round(float(station[nearest[near]].max()), 1),
                      "left_dst": _link_ref(link.get("left_lane_change_dst_link_idx")),
                      "right_dst": _link_ref(link.get("right_lane_change_dst_link_idx")),
                      "pts": [[round(float(x), 2), round(float(y), 2)] for x, y in sampled]})
    # What the planner used before any selection: same-direction lanes next to a route lane.
    by_id = {lane["id"]: lane for lane in lanes}
    for lane in lanes:
        if lane["kind"] == "route":
            for key in ("left_dst", "right_dst"):
                other = by_id.get(lane[key])
                if other is not None and other["kind"] == "same":
                    other["hdmap_default"] = True
    # Intersections the planner derives on its own: route lanes on HD-map junction links (link_type 1).
    junction_ids = {link.get("idx") for link in links if str(link.get("link_type")) == "1"}
    spans = sorted((lane["s0"], lane["s1"]) for lane in lanes if lane["kind"] == "route" and lane["id"] in junction_ids)
    auto_intersections = []
    for s0, s1 in spans:
        if auto_intersections and s0 <= auto_intersections[-1]["end_m"] + 1.0:
            auto_intersections[-1]["end_m"] = max(auto_intersections[-1]["end_m"], s1)
        else:
            auto_intersections.append({"type": "intersection", "label": "교차로 (HD맵 자동)", "start_m": s0, "end_m": s1, "auto": True})
    return {"auto_intersections": auto_intersections,
            "route": [[round(float(x), 2), round(float(y), 2)] for x, y in pts[::2]],
            "station": [round(float(v), 1) for v in station[::2]], "length_m": round(float(station[-1]), 1),
            "lanes": lanes, "section_types": SECTION_TYPES, "mission_sections": MISSION_SECTIONS}


def map3d():
    """3-D HD map for the live LiDAR view: lane edges (with height) around the route, the route and tunnel walls."""
    import numpy as np
    ids = {lane["id"]: lane["kind"] for lane in CACHE.get("route_lanes", 3600.0, route_lanes)["lanes"]}
    links = read_json(ASSETS / "link_set.json") or []
    lanes, all_pts = [], []
    for link in links:
        if link.get("idx") not in ids:
            continue
        lp = np.asarray([p[:3] if len(p) >= 3 else (p[0], p[1], 0.) for p in link.get("points") or []], dtype=float)
        if len(lp) < 2:
            continue
        seg = np.hypot(*np.diff(lp[:, :2], axis=0).T)
        s = np.r_[0., np.cumsum(seg)]
        keep = np.unique(np.r_[np.searchsorted(s, np.arange(0., s[-1], 2.)), len(lp) - 1])
        c = lp[keep]
        t = np.gradient(c[:, :2], axis=0) if len(c) > 1 else np.array([[1., 0.]])
        t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
        n = np.c_[-t[:, 1], t[:, 0]]
        half = .5 * float(link.get("width_start") or 3.5)
        edge = lambda k: [[round(float(x), 2), round(float(y), 2), round(float(z), 2)]
                          for (x, y), z in zip(c[:, :2] + k * half * n, c[:, 2])]
        lanes.append({"kind": ids[link["idx"]], "center": edge(0.), "left": edge(1.), "right": edge(-1.)})
        all_pts.append(lp)
    route = read_json(ASSETS / "global_path.json") or {}
    rp = np.asarray([(p["x"], p["y"]) for p in route.get("waypoints") or []], dtype=float)[::3]
    ref = np.vstack(all_pts) if all_pts else np.zeros((1, 3))
    def height(xy):
        out = []
        for chunk in np.array_split(xy, max(1, len(xy) // 200)):
            d = ((chunk[:, None, :] - ref[None, :, :2]) ** 2).sum(-1)
            out.append(ref[d.argmin(1), 2])
        return np.concatenate(out) if out else np.zeros(0)
    route3 = [[round(float(x), 2), round(float(y), 2), round(float(z), 2)] for (x, y), z in zip(rp, height(rp))] if len(rp) else []
    walls = []
    tunnel = read_json(ASSETS / "tunnel_wall_map.json") or {}
    for side in ("left", "right"):
        w = np.asarray((tunnel.get("walls") or {}).get(side) or [], dtype=float)
        if len(w) >= 2:
            w = np.vstack([np.linspace(a, b, max(2, int(np.hypot(*(b - a)) // 3))) for a, b in zip(w[:-1], w[1:])])
            walls.append([[round(float(x), 2), round(float(y), 2), round(float(z), 2)] for (x, y), z in zip(w, height(w))])
    return {"lanes": lanes, "route": route3, "walls": walls}


def live_ego():
    """Latest competition-launch position (localization only) for the map pages; None when stale."""
    runs = DATA / "autodrive/follow_runs"
    try:
        latest = max((d for d in runs.iterdir() if d.is_dir()), key=lambda d: d.stat().st_mtime)
        status = read_json(latest / "loc_status.json") or {}
    except (OSError, ValueError):
        return {"ego": None}
    if time.time() - float(status.get("time", 0)) > 2.0 or not status.get("ego"):
        return {"ego": None}
    return {"ego": status["ego"], "run": latest.name}


def read_drivable_rules():
    rules = read_json(DRIVABLE_RULES)
    if not isinstance(rules, dict):
        rules = {}
    return {"mode": rules.get("mode"), "allowed_links": rules.get("allowed_links") or [],
            "sections": rules.get("sections") or [], "zones": rules.get("zones") or [],
            "path": str(DRIVABLE_RULES), "updated_at": rules.get("updated_at")}


def write_drivable_rules(body):
    """Validate and atomically save operator rules; the planner reloads them within 2 s.

    mode "whitelist": besides the route lane, bypasses may use only allowed_links
    (same-direction lanes chosen on the 주행 가능 영역 page)."""
    lanes = {lane["id"]: lane for lane in CACHE.get("route_lanes", 3600.0, route_lanes)["lanes"]}
    allowed = []
    for link_id in body.get("allowed_links") or []:
        lane = lanes.get(link_id)
        if lane is None:
            raise ValueError(f"경로 주변 차선이 아님: {link_id}")
        if lane["kind"] == "opposite":
            raise ValueError(f"반대 방향 차선은 사용할 수 없음: {link_id}")
        if lane["kind"] == "same" and link_id not in allowed:
            allowed.append(link_id)
    sections = []
    for index, section in enumerate(body.get("sections") or []):
        kind = section.get("type")
        start, end = float(section.get("start_m", 0)), float(section.get("end_m", 0))
        if kind not in SECTION_TYPES or not 0 <= start < end:
            raise ValueError(f"구간 {index + 1}: 종류 또는 시작/끝 거리가 잘못됨")
        sections.append({"type": kind, "label": str(section.get("label") or "")[:40],
                         "start_m": round(start, 1), "end_m": round(end, 1)})
    zones = []
    for index, zone in enumerate(body.get("zones") or []):
        polygon = zone.get("polygon") if isinstance(zone, dict) else None
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise ValueError(f"금지구역 {index + 1}: 꼭짓점이 3개 이상 필요")
        zones.append({"name": str(zone.get("name") or f"금지구역 {index + 1}")[:40],
                      "polygon": [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in polygon]})
    rules = {"mode": "whitelist", "allowed_links": allowed, "sections": sorted(sections, key=lambda s: s["start_m"]),
             "zones": zones, "updated_at": time.strftime("%Y-%m-%d %H:%M:%S")}
    if DRIVABLE_RULES.exists():
        DRIVABLE_RULES.with_name("drivable_rules.backup.json").write_bytes(DRIVABLE_RULES.read_bytes())
    tmp = DRIVABLE_RULES.with_suffix(".tmp")
    tmp.write_text(json.dumps(rules, ensure_ascii=False, indent=2))
    os.replace(tmp, DRIVABLE_RULES)
    return rules


# Only one driving mode is left: E2E. A drive_mode in a request (stale browser preference
# "rule", older clients) is ignored; every competition launch runs student_v3_follow_claude.sh.
def job_drive_mode(job):
    if job.get("action") not in ("autodrive_follow", "autodrive_collect"):
        return None
    return "e2e"


def check_launch_files():
    if not (LAUNCH.is_file() and (ASSETS / "gui.py").is_file()
            and (ASSETS / "best_v5.pt").is_file() and (ASSETS / "v6_model.pt").is_file()):
        raise ValueError("E2E 실행 파일 또는 모델 없음 (autodrive/student_v3_follow_claude.sh, autodrive/student_v3_claude)")


def build_command(action, o):
    if not isinstance(o, dict):
        raise ValueError("options must be a JSON object")
    signal_enabled = o.get("signal_enabled", True)
    if action in ("autodrive_follow", "autodrive_collect") and not isinstance(signal_enabled, bool):
        raise ValueError("signal_enabled must be a JSON boolean")
    if action == "autodrive_follow":
        # Competition launch (E2E): nothing is collected; the run keeps a debug bag and logs.
        # Competition rules: /Ego_topic and /Object_topic are never used, whatever the UI selector says.
        network_profile = o.get("network_profile", DEFAULT_PROFILE)
        if network_profile not in ("single_pc", "two_pc"):
            raise ValueError("대회 런치는 '이 PC 한 대' 또는 'PC 두 대' 버튼으로 구성 선택 필요")
        check_launch_files()
        # One lap from the start point, then a stop there; no time limit (the minutes field
        # does not apply; "0" only fills the launch's duration slot).
        return command_with_signal_setting(
            ["bash", str(LAUNCH), "0", "90.0", "localization", network_profile], signal_enabled)
    if action == "autodrive_collect":
        sid = safe_session_id(next_session_id() if MODE == "collect" else o.get("session_id") or next_session_id())
        # Collection is always the local single-PC sensor-only stack: the same E2E launch with the
        # session id as its 5th argument (one lap, then stop). The session id stays the last element
        # (auto_collect.py reads it from there).
        check_launch_files()
        return command_with_signal_setting(
            ["bash", str(LAUNCH), "0", "90.0", "localization", "single_pc", sid], signal_enabled)
    if action == "collect":
        sid = safe_session_id(o.get("session_id") or next_session_id())
        sec = int(number(o.get("minutes", 10), "수집 시간(분)", 0.1, 240) * 60)
        return ["bash", str(ROOT / "collect/collect_session.sh"), sid, str(sec)]
    if action == "label":
        raise ValueError("Teacher 라벨링 스크립트(autodrive/label_sessions.sh)는 2026-10-11 정리로 삭제됨")
    if action == "check":
        sessions = o.get("sessions") or []
        if not isinstance(sessions, list) or len(sessions) != 1:
            raise ValueError("Teacher 검사는 세션 1개만 선택")
        safe_session_id(sessions[0])
        if not (DATA / "student/sessions" / sessions[0] / "responses.jsonl").is_file():
            raise ValueError(f"{sessions[0]}: Teacher 응답 없음 (먼저 라벨링)")
        return [str(ROOT / "student/run.sh"), "check_teacher.py", "--session-dir", f"/data/student/sessions/{sessions[0]}"]
    if action == "check_student":
        sessions = o.get("sessions") or []
        if not isinstance(sessions, list) or len(sessions) != 1:
            raise ValueError("Student 검사는 세션 1개만 선택")
        safe_session_id(sessions[0])
        model_name = str(o.get("model_name") or "trial").strip()
        safe_session_id(model_name)
        weights = "last.pt" if str(o.get("weights", "best")) == "last" else "best.pt"
        ckpt = DATA / "models/student" / model_name / weights
        if not ckpt.is_file():
            raise ValueError(f"{model_name}/{weights} 없음 (먼저 시험/전체 학습)")
        if not (DATA / "student/sessions" / sessions[0] / "responses.jsonl").is_file():
            raise ValueError(f"{sessions[0]}: Teacher 응답 없음 (먼저 라벨링)")
        return [str(ROOT / "student/run.sh"), "check_student.py", "--session-dir", f"/data/student/sessions/{sessions[0]}",
                "--checkpoint", f"/data/models/student/{model_name}/{weights}", "--fp16"]
    if action == "video":
        sessions = o.get("sessions") or []
        if not isinstance(sessions, list) or len(sessions) != 1:
            raise ValueError("판단 영상은 세션 1개만 선택")
        safe_session_id(sessions[0])
        if not (DATA / "student/sessions" / sessions[0] / "responses.jsonl").is_file():
            raise ValueError(f"{sessions[0]}: Teacher 응답 없음 (먼저 라벨링)")
        cmd = [str(ROOT / "student/run.sh"), "render_video.py", "--session-dir", f"/data/student/sessions/{sessions[0]}"]
        if o.get("whole_video"):
            cmd.append("--whole")
        model_name = str(o.get("model_name") or "").strip()
        weights = "last.pt" if str(o.get("weights", "best")) == "last" else "best.pt"
        if model_name:
            safe_session_id(model_name)
            if (DATA / "models/student" / model_name / weights).is_file():
                cmd += ["--checkpoint", f"/data/models/student/{model_name}/{weights}"]
        return cmd
    if action == "prepare":
        if not SOURCE.is_file():
            raise ValueError("source.jsonl 없음: 먼저 라벨링 실행")
        vf = number(o.get("val_fraction", 0.2), "검증 비율", 0.05, 0.5)
        return [str(ROOT / "student/run.sh"), "prepare.py", "--manifest", "/data/student/source.jsonl", "--output", "/data/student/prepared",
                "--profile", "realtime_max", "--val-fraction", str(vf)]
    if action == "trial":
        steps = number(o.get("max_steps", 30), "step 수", 1, 5000, integer=True)
        return [str(ROOT / "student/run.sh"), "train.py", "--manifest", "/data/student/prepared/manifest.jsonl", "--output", "/data/models/student/trial",
                "--profile", "realtime_max", "--max-steps", str(steps), "--batch-size", "2", "--workers", "2", "--epochs", "1"]
    if action == "train":
        epochs = number(o.get("epochs", 60), "epoch", 1, 1000, integer=True)
        batch = number(o.get("batch_size", 8), "batch", 1, 64, integer=True)
        name = str(o.get("model_name") or "rgb_realtime_max_v1").strip()
        safe_session_id(name)
        gpus = [0, 1, 2, 3]
        busy = gpus_used_by_simulator()
        if o.get("exclude_simulator_gpu", True) and busy:
            gpus = [g for g in gpus if g not in busy]
        gpu_list = ",".join(map(str, gpus))
        return ["bash", "-c", f"./stop-teacher.sh; ./stop-teacher-pool.sh; export TRAIN_GPUS={gpu_list}; exec ./student/train-4gpu.sh --manifest /data/student/prepared/manifest.jsonl "
                              f"--output /data/models/student/{name} --profile realtime_max --epochs {epochs} --batch-size {batch} --grad-accum 2 --workers 4"]
    raise ValueError(f"알 수 없는 작업: {action}")


class Cache:
    def __init__(self):
        self.lock = threading.Lock()
        self.items = {}

    def get(self, key, ttl, fn):
        with self.lock:
            v, t = self.items.get(key, (None, 0))
            if time.time() - t < ttl:
                return v
        v = fn()
        with self.lock:
            self.items[key] = (v, time.time())
        return v


CACHE = Cache()


def gpus_used_by_simulator():
    """GPU indices where the MORAI simulator process is running (nvidia-smi), so training can leave them alone."""
    code, out = run(["nvidia-smi", "--query-compute-apps=gpu_uuid,process_name", "--format=csv,noheader"], timeout=5)
    if code != 0:
        return []
    code2, uuids = run(["nvidia-smi", "--query-gpu=index,gpu_uuid", "--format=csv,noheader"], timeout=5)
    index = {}
    for line in uuids.splitlines():
        parts = [x.strip() for x in line.split(",")]
        if len(parts) == 2:
            index[parts[1]] = int(parts[0])
    busy = set()
    for line in out.splitlines():
        parts = [x.strip() for x in line.split(",")]
        if len(parts) == 2 and "Simulator" in parts[1] and parts[0] in index:
            busy.add(index[parts[0]])
    return sorted(busy)


def stack_running():
    code, out = run(["docker", "inspect", "-f", "{{.State.Running}}", CONTAINER], timeout=5)
    return code == 0 and out.strip() == "true"


def teacher_health():
    """Primary teacher (8766) health plus the number of healthy pool replicas (8768/8769/8770)."""
    primary, alive = None, []
    for port in (8766, 8768, 8769, 8770):
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=1.0) as r:
                h = json.load(r)
                if h.get("status") == "ready":
                    alive.append(port)
                    primary = primary or h
        except Exception:
            pass
    if not primary:
        return None
    return {**primary, "pool_ports": alive, "pool_size": len(alive)}


def drive_lock_held():
    """True while an owned drive script (collect or follow-only, web or terminal) holds data/autodrive/drive.lock."""
    try:
        with open(DATA / "autodrive/drive.lock", "a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                return True
            fcntl.flock(lock, fcntl.LOCK_UN)
            return False
    except OSError:
        return False


def sniff_cameras(seconds=1.0):
    """The legacy lab shares the scoped preview's receivers and launch handoff."""
    sample = SENSOR_PREVIEW.sample(competition_profile(DEFAULT_PROFILE),
                                   can_read=lambda: not drive_lock_held(), seconds=seconds)
    return {view: sample["streams"][view]["hz"] for view in VIEWS} if sample else None


_LIVE_MANIFEST_LOCK = threading.Lock()
_LIVE_MANIFEST = {}


def live_manifest_stats(session):
    """Read only newly committed manifest lines for an unfinished session."""
    key = str(session)
    with _LIVE_MANIFEST_LOCK:
        state = _LIVE_MANIFEST.setdefault(key, {
            "inode": None, "offset": 0, "frames": {view: 0 for view in VIEWS},
            "first": {}, "last": {}})
        try:
            with (session / "manifest.jsonl").open("rb") as source:
                stat = os.fstat(source.fileno())
                if stat.st_ino != state["inode"] or stat.st_size < state["offset"]:
                    state.update(inode=stat.st_ino, offset=0,
                                 frames={view: 0 for view in VIEWS}, first={}, last={})
                source.seek(state["offset"])
                while True:
                    start = source.tell()
                    line = source.readline()
                    if not line or not line.endswith(b"\n"):
                        state["offset"] = start
                        break
                    try:
                        row = json.loads(line)
                        view = row.get("view")
                        stamp = int(row.get("rgb_timestamp_ns", 0))
                        if view in state["frames"] and stamp > 0:
                            state["frames"][view] += 1
                            state["first"].setdefault(view, stamp)
                            state["last"][view] = stamp
                    except (ValueError, TypeError):
                        pass
        except OSError:
            pass
        counts, first, last = state["frames"].copy(), state["first"], state["last"]
    hz = {view: counts[view] * 1e9 / (last[view] - first[view])
          for view in VIEWS if view in last and last[view] > first[view]}
    return {"frames": counts, "hz": hz}


def list_sessions():
    rows = []
    if RAW.is_dir():
        for path in sorted((p for p in RAW.iterdir() if p.is_dir()),
                           key=lambda p: str((read_json(p / "session.json", {}) or {}).get("started_at", "")), reverse=True)[:60]:
            meta = read_json(path / "session.json", {}) or {}
            if meta.get("format_version") != 2:
                continue
            audit = meta.get("audit") or {}
            cams = audit.get("camera") or {}
            backup = read_json(path / "cloud_backup.json", {}) or {}
            recording = not bool(meta.get("finished_at"))
            live = CACHE.get("manifest:" + path.name, 3.0, lambda: live_manifest_stats(path)) if recording else None
            frames = meta.get("saved_per_view") or (live or {}).get("frames") or {}
            measured_hz = meta.get("measured_hz_per_view") or (live or {}).get("hz") or {}
            rows.append({"name": path.name, "started_at": meta.get("started_at", ""), "duration_s": meta.get("duration_seconds"),
                         "frames": frames, "hz": {v: round(x, 1) for v, x in measured_hz.items()}, "recording": recording,
                         "has_ego": (path / "ego.csv").is_file(), "backup": {"verified": bool(backup), "pruned": bool(backup.get("pruned")), "url": backup.get("url")}, "ego_hz": audit.get("ego_hz"), "heading": audit.get("heading_convention"),
                         "grid_pct": {v: round(c.get("grid10hz_within_20ms_pct", 0), 1) for v, c in cams.items()},
                         "route_dist_m": audit.get("ego_to_route_distance_m_median"), "collisions": audit.get("collision_events"), "collision_s": audit.get("collision_seconds"), "finished": bool(meta.get("finished_at")),
                         "objects_kb": round((path / "objects.jsonl").stat().st_size / 1024) if (path / "objects.jsonl").is_file() else None,
                         "labeled": read_json(DATA / "student/sessions" / path.name / "build_summary.json") is not None})
    return rows


def cloud_backup_status():
    state = read_json(DATA / "autodrive/cloud_backup/status.json", {}) or {}
    receipts = DATA / "autodrive/cloud_backup/receipts"
    numbers = read_json(DATA / "autodrive/cloud_backup/numbers.json", {}) or {}
    items = []
    if RAW.is_dir():
        for session in RAW.iterdir():
            if not session.is_dir() or not (read_json(session / "session.json", {}) or {}).get("finished_at"):
                continue
            receipt = read_json(receipts / (session.name + ".json"), {}) or {}
            number = int(session.name) if session.name.isdecimal() else numbers.get(session.name)
            phase = "verified" if receipt.get("pruned") else "cleaning" if receipt else state.get("phase") if state.get("session") == session.name else "queued"
            items.append({"session": session.name, "number": number, "phase": phase,
                          "url": receipt.get("url"), "size": receipt.get("size"), "pruned": bool(receipt.get("pruned"))})
    items.sort(key=lambda row: (row["number"] is None, row["number"] or 0, row["session"]))
    state["items"] = items
    state["verified"] = sum(item["phase"] == "verified" for item in items)
    state["total"] = len(items)
    state["pending"] = state["total"] - state["verified"]
    return state


def datasets():
    out = []
    root = DATA / "student/sessions"
    if root.is_dir():
        for d in sorted(root.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True)[:60]:
            summary = read_json(d / "build_summary.json")
            if not summary:
                continue
            responses = sum(1 for _ in open(d / "responses.jsonl")) if (d / "responses.jsonl").is_file() else 0
            failures = sum(1 for _ in open(d / "failures.jsonl")) if (d / "failures.jsonl").is_file() else 0
            src = sum(1 for _ in open(d / "source.jsonl")) if (d / "source.jsonl").is_file() else 0
            check = read_json(d / "teacher_check.json") or {}
            scheck = read_json(d / "student_check.json") or {}
            video = read_json(d / "judgement.json") or {}
            out.append({"name": d.name, "anchors": summary.get("anchors"), "clips": summary.get("written"), "skipped": summary.get("skipped", {}),
                        "teacher_responses": responses, "teacher_failures": failures, "source_rows": src,
                        "check": {k: check.get(k) for k in ("samples", "ade_m_mean", "fde_m_mean", "actions")} if check else None,
                        "student_check": {k: scheck.get(k) for k in ("checkpoint", "training_steps", "samples", "action_agreement", "ade_vs_teacher_m_mean", "fde_vs_teacher_m_mean", "ade_vs_actual_m_mean", "latency_ms_p95")} if scheck else None,
                        "video": {**{k: video.get(k) for k in ("checkpoint", "seconds", "size_mb", "clips_judged")}, "mp4": (d / "judgement.mp4").is_file(), "rendered_at": (d / "judgement.json").stat().st_mtime} if video else None})
    return out


def source_summary():
    if not SOURCE.is_file():
        return {"rows": 0, "sessions": 0, "val_sessions": 0}
    sessions, val = set(), set()
    n = 0
    for line in SOURCE.read_text().splitlines():
        if not line.strip():
            continue
        n += 1
        r = json.loads(line)
        sessions.add(r["session_id"])
        if r.get("split") == "val":
            val.add(r["session_id"])
    return {"rows": n, "sessions": len(sessions), "val_sessions": len(val)}


def models():
    out = []
    root = DATA / "models/student"
    if root.is_dir():
        for d in sorted((p for p in root.iterdir() if p.is_dir()), key=lambda p: p.stat().st_mtime, reverse=True)[:10]:
            m = d / "metrics.jsonl"
            last = None
            if m.is_file():
                lines = [l for l in m.read_text().splitlines() if l.strip()]
                last = json.loads(lines[-1]) if lines else None
            out.append({"name": d.name, "best": (d / "best.pt").is_file(), "last_metrics": last})
    return out


def status_payload():
    job = JOBS.snapshot()
    ros = read_json(STATUS_JSON) or {}
    ros_age = time.time() - ros.get("time", 0) if ros else None
    collecting = job["running"] and job["action"] in ("collect", "autodrive_collect")
    cams = None
    # A running drive's gui.py owns the camera UDP ports; sniffing would steal its frames.
    if not collecting and not drive_lock_held():
        cams = CACHE.get("cams", 3.0, lambda: sniff_cameras(1.0))
    else:
        for line in reversed(job["logs"]):
            if "hz≈" in line:
                try:
                    hz = float(line.split("hz≈")[1].split()[0])
                    cams = {v: hz for v in VIEWS}
                except ValueError:
                    pass
                break
    return {"time": time.time(), "mode": MODE, "stack_running": CACHE.get("stack", 3.0, stack_running), "simulator_gpus": CACHE.get("simgpu", 10.0, gpus_used_by_simulator), "ros": ros, "ros_age_s": ros_age,
            "teacher": (CACHE.get("teacher", 5.0, teacher_health) if MODE != "collect" else None), "cameras": cams,
            "previews": {view: preview_info(view) for view in VIEWS}, "collecting": collecting, "job": job,
            "sessions": list_sessions(), "datasets": datasets(), "source": source_summary(),
            "prepared": read_json(DATA / "student/prepared/summary.json"), "models": models(),
            "cloud_backup": cloud_backup_status(), "mpc_runs": mpc_runs(),
            "network_profile": read_json(DATA / "autodrive/network_profile.json"),
            "default_network_profile": DEFAULT_PROFILE,
            "drive_mode": job_drive_mode(job),
            "competition_profiles": {name: competition_profile(name) for name in ("single_pc", "two_pc")},
            "paths": {"대회 주행 디버그": str(DATA / "autodrive/follow_runs"), "mpc (PC 보관)": str(DATA / "autodrive/mpc_runs"), "raw (Drive 전송 대기)": str(RAW), "source": str(SOURCE), "prepared": str(DATA / "student/prepared"), "models": str(DATA / "models/student")}}


def latest_preview(view):
    if view not in VIEWS or not RAW.is_dir():
        return None
    sessions = [p for p in RAW.iterdir() if p.is_dir() and (read_json(p / "session.json", {}) or {}).get("format_version") == 2]
    if not sessions:
        return None
    for session in sorted(sessions,
                          key=lambda p: str((read_json(p / "session.json", {}) or {}).get("started_at", "")), reverse=True):
        files = sorted((session / "raw" / view / "rgb").glob("*.jpg"))
        if files:
            return files[-1]
    return None


def preview_info(view):
    path = CACHE.get("preview_" + view, 2.0, lambda: latest_preview(view))
    if path is None or not path.is_file():
        return None
    return {"session": path.parents[3].name, "age_s": max(0, round(time.time() - path.stat().st_mtime, 1))}


def follow(action, kmh, avoid=False):
    if action not in ("start", "stop"):
        raise ValueError("action은 start/stop")
    job = JOBS.snapshot()
    if job["running"] and any("student_v3_follow" in part for part in job["command"]):
        raise ValueError("Student 주행은 위의 '중지' 버튼으로 종료")
    cmd = ["docker", "exec", CONTAINER, "/opt/autodrive/ros/ctl.sh", f"follow-{action}"] + ([str(kmh), "avoid" if avoid else "follow"] if action == "start" else [])
    # Local C++ edits are built before start; stop may wait for ROS shutdown.
    code, out = run(cmd, timeout=540 if action == "start" else 40)
    if code != 0:
        raise RuntimeError(out or "ctl.sh 실패")
    return out


def requested_drive_profile(query):
    name = query.get("profile", [""])[0]
    if name not in ("single_pc", "two_pc"):
        raise ValueError("1 PC용 또는 2 PC 대회용 페이지에서 구성 선택 필요")
    return competition_profile(name)


def job_network_profile(job):
    command = job.get("command") or []
    if command[:1] == ["env"] and len(command) > 2 and command[1].startswith("STUDENT_SIGNAL_GUARD="):
        command = command[2:]
    if len(command) >= 6 and Path(command[1]).name == "student_v3_follow_claude.sh":
        return command[5] if command[5] in ("single_pc", "two_pc") else None
    return None


def active_drive_directory(job, name):
    if not job.get("running") or job_network_profile(job) != name:
        return None
    owner = read_json(DATA / "autodrive/drive_run.json") or {}
    if owner.get("started_wall_ns", 0) / 1e9 < (job.get("started_at") or 0):
        return None
    nonce = owner.get("nonce", "")
    if not nonce or nonce in (".", "..") or Path(nonce).name != nonce:
        return None
    sid = owner.get("session_id")
    directory = DATA / "autodrive" / ("mpc_runs" if sid else "follow_runs")
    if sid:
        try:
            sid = safe_session_id(sid)
        except ValueError:
            return None
        directory /= sid
    directory /= nonce
    configured = read_json(directory / "network_profile.json") or {}
    return directory if configured.get("id") == name else None


def drive_camera(profile, view, job=None):
    if view not in VIEWS:
        return None
    job = JOBS.snapshot() if job is None else job
    directory = active_drive_directory(job, profile["id"])
    if directory:
        path = DATA / "autodrive/semantic_live" / (view + ".jpg")
        try:
            modified = path.stat().st_mtime
            if modified >= job["started_at"] and time.time() - modified < 3:
                return path.read_bytes()
        except OSError:
            pass
        return None
    if job.get("running") or drive_lock_held():
        return None
    return SENSOR_PREVIEW.image(profile["id"], view)


def drive_status(profile):
    job = JOBS.snapshot()
    name = profile["id"]
    owner = job_network_profile(job)
    busy = bool(job["running"] or drive_lock_held())
    sensor_input = None if busy else SENSOR_PREVIEW.sample(profile, can_read=lambda: not drive_lock_held())
    directory = active_drive_directory(job, name)
    ros = (read_json(directory / "loc_status.json") or {}) if directory else {}
    age = time.time() - ros.get("time", 0) if ros else None
    if age is None or age > 3:
        ros = {}
    visible_job = job if owner == name else dict(job, running=False, action=None, command=[], logs=[],
                                               started_at=None, finished_at=None, exit_code=None, elapsed_seconds=0)
    run_meta = read_json(directory / "run.json", {}) if directory else {}
    signal_state = read_json(directory / "signal_status.json", {}) if directory else {}
    if not isinstance(signal_state, dict) or time.time()-signal_state.get("time", 0.) > 2.:
        signal_state = {}
    return {"profile": profile, "sensor_input": sensor_input, "ros": ros, "ros_age_s": age,
            "signal_enabled": run_meta.get("signal_enabled") if run_meta else None,
            "traffic_signal": signal_state,
            "job": visible_job, "busy": busy,
            "other_job": {"profile": owner, "action": job["action"]} if busy and owner != name else None,
            "monitor_mode": "driving" if directory else "starting" if busy else "idle",
            "camera_available": {view: drive_camera(profile, view, job) is not None for view in VIEWS}}


def signal_view_context(profile):
    """Read only this profile's current run; never open camera UDP sockets."""
    job = JOBS.snapshot()
    owner = job_network_profile(job)
    directory = active_drive_directory(job, profile["id"])
    meta = read_json(directory / "run.json", {}) if directory else {}
    control = read_json(directory / "signal_status.json", {}) if directory else {}
    try:
        fresh = (isinstance(control, dict) and control.get("run_id") == (directory.name if directory else None)
                 and 0. <= time.time()-float(control.get("time", 0.)) <= 2.)
    except (TypeError, ValueError, OverflowError):
        fresh = False
    if not fresh:
        control = {}
    return dict(profile=profile["id"], directory=directory, job=job,
        busy=bool(job.get("running") or drive_lock_held()),
        running=bool(job.get("running") and owner == profile["id"]),
        owner=owner, signal_enabled=meta.get("signal_enabled") if isinstance(meta, dict) else None,
        control=control)


def signal_view_status(profile):
    context = signal_view_context(profile)
    directory, control = context["directory"], context["control"]
    perception = None
    if directory and context["signal_enabled"] is True and control.get("active"):
        perception = SIGNAL_DEBUG.latest(directory.name)
        if perception and (not control.get("selected_line_id") or
                perception.get("line_id") != control["selected_line_id"]):
            perception = None
    if perception:
        reason = "실시간 HSV 판정"
    elif context["signal_enabled"] is False:
        reason = "신호등 미사용 주행 중"
    elif directory and control and not control.get("active"):
        reason = "다음 신호등 구간 대기"
    elif context["running"]:
        reason = "실시간 신호등 영상 수신 대기"
    elif context["busy"]:
        reason = "다른 구성에서 주행 중"
    else:
        reason = "신호등 사용을 선택하고 주행 시작 시 영상 표시"
    result = {key: context[key] for key in ("profile", "busy", "running", "owner", "signal_enabled", "control")}
    result.update(available=perception is not None, reason=reason, perception=perception,
                  frame_age_s=time.time()-perception["frame_wall"] if perception else None)
    return result


def signal_view_image(profile, frame_id, slot, view):
    context = signal_view_context(profile)
    control, directory = context["control"], context["directory"]
    if (directory is None or context["signal_enabled"] is not True or not control.get("active")
            or not control.get("selected_line_id")):
        return None
    return SIGNAL_DEBUG.image(directory.name, frame_id, slot, view,
                              line_id=control["selected_line_id"])


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send_bytes(self, content, content_type, status=HTTPStatus.OK):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def send_json(self, value, status=HTTPStatus.OK):
        self.send_bytes(json.dumps(value, ensure_ascii=False).encode(), "application/json; charset=utf-8", status)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path in ("/", "/index.html"):
            return self.send_bytes((WEB / "profiles.html").read_bytes(), "text/html; charset=utf-8")
        if url.path in ("/single-pc", "/competition"):
            name = "single_pc" if url.path == "/single-pc" else "two_pc"
            html = (WEB / "drive.html").read_text().replace("__PROFILE__", name)
            return self.send_bytes(html.encode(), "text/html; charset=utf-8")
        if url.path == "/drive.js":
            return self.send_bytes((WEB / "drive.js").read_bytes(), "text/javascript; charset=utf-8")
        if url.path == "/signals":
            try:
                profile = requested_drive_profile(parse_qs(url.query))
            except ValueError as error:
                return self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
            html = (WEB / "signal.html").read_text().replace("__PROFILE__", profile["id"])
            return self.send_bytes(html.encode(), "text/html; charset=utf-8")
        if url.path == "/signal.js":
            return self.send_bytes((WEB / "signal.js").read_bytes(), "text/javascript; charset=utf-8")
        if url.path in ("/api/signal/status", "/api/signal/image"):
            query = parse_qs(url.query)
            try:
                profile = requested_drive_profile(query)
                if url.path.endswith("/status"):
                    return self.send_json(signal_view_status(profile))
                view = query.get("view", [""])[0]
                slot = int(query.get("slot", ["-1"])[0])
                if view not in SIGNAL_VIEWS or not 0 <= slot < 8:
                    raise ValueError("신호등 영상 종류 또는 프레임 지정 오류")
            except ValueError as error:
                return self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
            jpeg = signal_view_image(profile, query.get("frame", [""])[0], slot, view)
            if jpeg is None:
                return self.send_json({"error": "현재 주행의 실시간 신호등 영상 없음"}, HTTPStatus.NOT_FOUND)
            return self.send_bytes(jpeg, "image/jpeg")
        if url.path == "/lab":
            return self.send_bytes((WEB / "index.html").read_bytes(), "text/html; charset=utf-8")
        if url.path in ("/api/drive-status", "/api/drive-camera"):
            query = parse_qs(url.query)
            try:
                profile = requested_drive_profile(query)
            except ValueError as error:
                return self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
            if url.path == "/api/drive-status":
                return self.send_json(drive_status(profile))
            jpeg = drive_camera(profile, query.get("view", [""])[0])
            if jpeg is None:
                return self.send_json({"error": "선택한 구성의 실시간 카메라 수신 대기"}, HTTPStatus.NOT_FOUND)
            return self.send_bytes(jpeg, "image/jpeg")
        if url.path == "/api/preset":
            query = parse_qs(url.query)
            name = query.get("profile", [DEFAULT_PROFILE])[0]
            kind = query.get("kind", [""])[0]
            if name not in ("single_pc", "two_pc") or kind not in ("sensor", "network"):
                return self.send_json({"error": "프리셋 구성 또는 종류가 잘못됨"}, HTTPStatus.BAD_REQUEST)
            path = Path(competition_profile(name)[kind + "_file"])
            content = path.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)
            return
        if url.path == "/lidar":
            return self.send_bytes((WEB / "lidar.html").read_bytes(), "text/html; charset=utf-8")
        if url.path == "/route-picker":
            return self.send_bytes((WEB / "route_picker.html").read_bytes(), "text/html; charset=utf-8")
        if url.path in ("/spider-theme.css", "/spider-intro.js", "/ironman-theme.css", "/ironman-intro.js", "/theme-switcher.js", "/hud-fx.js"):
            name = url.path[1:]
            content_type = "text/css; charset=utf-8" if name.endswith(".css") else "text/javascript; charset=utf-8"
            return self.send_bytes((WEB / name).read_bytes(), content_type)
        if url.path.startswith("/spider-assets/") or url.path.startswith("/ironman-assets/"):
            asset_dir, _, name = url.path[1:].partition("/")
            content_types = {"spiderman.webm": "video/webm", "spiderman-browser.mp4": "video/mp4",
                             "spider-cinema.jpg": "image/jpeg", "spider-action.jpg": "image/jpeg"} if asset_dir == "spider-assets" else \
                            {"ironman-intro.mp4": "video/mp4", "ironman-cinema.jpg": "image/jpeg"}
            if name not in content_types:
                return self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            path = WEB / asset_dir / name
            if not path.is_file():
                return self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            if name.endswith((".webm", ".mp4")):
                size = path.stat().st_size
                byte_range = self.headers.get("Range", "")
                start, end = 0, size - 1
                if byte_range.startswith("bytes="):
                    first, _, last = byte_range[6:].partition("-")
                    if first.isdigit():
                        start = int(first)
                        end = min(int(last), size - 1) if last.isdigit() else min(start + 1024 * 1024 - 1, size - 1)
                    if start >= size or end < start:
                        self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                        self.send_header("Content-Range", f"bytes */{size}")
                        self.end_headers()
                        return
                with path.open("rb") as video_file:
                    video_file.seek(start)
                    chunk = video_file.read(end - start + 1)
                self.send_response(HTTPStatus.PARTIAL_CONTENT if byte_range.startswith("bytes=") else HTTPStatus.OK)
                self.send_header("Content-Type", content_types[name])
                self.send_header("Accept-Ranges", "bytes")
                self.send_header("Content-Length", str(len(chunk)))
                if byte_range.startswith("bytes="):
                    self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
                self.end_headers()
                self.wfile.write(chunk)
                return
            return self.send_bytes(path.read_bytes(), content_types[name])
        if url.path == "/vendor/roslib.min.js":
            return self.send_bytes((WEB / "vendor/roslib.min.js").read_bytes(), "application/javascript")
        if url.path in ("/drivable", "/drivable.html"):
            return self.send_bytes((WEB / "drivable.html").read_bytes(), "text/html; charset=utf-8")
        if url.path == "/api/map3d":
            return self.send_json(CACHE.get("map3d", 3600.0, map3d))
        if url.path == "/api/route-lanes":
            return self.send_json(CACHE.get("route_lanes", 3600.0, route_lanes))
        if url.path == "/api/drivable-rules":
            return self.send_json(read_drivable_rules())
        if url.path == "/api/live-ego":
            return self.send_json(live_ego())
        if url.path == "/api/route":
            route = read_json(ASSETS / "global_path.json")
            if not isinstance(route, dict) or not isinstance(route.get("waypoints"), list):
                return self.send_json({"error": "E2E global path unavailable (autodrive/student_v3_claude/global_path.json)"}, HTTPStatus.NOT_FOUND)
            return self.send_json({"waypoints": route["waypoints"]})
        if url.path == "/api/status":
            return self.send_json(status_payload())
        if url.path == "/api/mpc-runs":
            return self.send_json({"runs": mpc_runs(), "path": str(DATA / "autodrive/mpc_runs")})
        if url.path == "/api/mpc-log":
            query = parse_qs(url.query)
            try:
                path = mpc_file(*(query.get(key, [""])[0] for key in ("session", "run", "file")))
            except ValueError as error:
                return self.send_json({"error": str(error)}, HTTPStatus.NOT_FOUND)
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/csv; charset=utf-8" if path.suffix == ".csv" else "application/octet-stream")
            self.send_header("Content-Disposition", f'attachment; filename="{path.parent.parent.name}_{path.name}"')
            # The active CSV can grow; stream exactly the advertised snapshot size.
            size = path.stat().st_size
            self.send_header("Content-Length", str(size))
            self.end_headers()
            with path.open("rb") as source:
                while size:
                    block = source.read(min(size, 1024 * 1024))
                    if not block:
                        break
                    self.wfile.write(block)
                    size -= len(block)
            return
        if url.path in ("/api/teacher_check", "/api/student_check"):
            sid = parse_qs(url.query).get("session", [""])[0]
            try:
                safe_session_id(sid)
            except ValueError:
                return self.send_json({"error": "bad session"}, HTTPStatus.BAD_REQUEST)
            png = DATA / "student/sessions" / sid / ("teacher_check.png" if url.path.endswith("teacher_check") else "student_check.png")
            if not png.is_file():
                return self.send_json({"error": "no report"}, HTTPStatus.NOT_FOUND)
            return self.send_bytes(png.read_bytes(), "image/png")
        if url.path == "/api/video":
            sid = parse_qs(url.query).get("session", [""])[0]
            try:
                safe_session_id(sid)
            except ValueError:
                return self.send_json({"error": "bad session"}, HTTPStatus.BAD_REQUEST)
            fmt = parse_qs(url.query).get("format", ["webm"])[0]
            path = DATA / "student/sessions" / sid / ("judgement.mp4" if fmt == "mp4" else "judgement.webm")
            if not path.is_file():
                return self.send_json({"error": "no video"}, HTTPStatus.NOT_FOUND)
            size = path.stat().st_size
            rng = self.headers.get("Range")
            start, end = 0, size - 1
            if rng and rng.startswith("bytes="):
                a_, _, b_ = rng[6:].partition("-")
                start = int(a_ or 0)
                end = int(b_) if b_ else min(start + 4 * 1024 * 1024, size - 1)
                end = min(end, size - 1)
            with path.open("rb") as f:
                f.seek(start)
                chunk = f.read(end - start + 1)
            self.send_response(HTTPStatus.PARTIAL_CONTENT if rng else HTTPStatus.OK)
            self.send_header("Content-Type", "video/mp4" if fmt == "mp4" else "video/webm")
            if fmt == "mp4":
                self.send_header("Content-Disposition", f'attachment; filename="{sid}_judgement.mp4"')
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(len(chunk)))
            if rng:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(chunk)
            return
        if url.path in ("/api/gui", "/api/decision"):
            # Alpamayo web view written by gui.py (headless): camera row with semantic tint, plan view, decision JSON
            shm = Path("/dev/shm/alpamayo_semantic")
            if url.path == "/api/decision":
                path = shm / "decision.json"
                return self.send_bytes(path.read_bytes() if path.is_file() else b"{}", "application/json; charset=utf-8")
            part = parse_qs(url.query).get("part", ["cams"])[0]
            path = shm / f"gui_{part}.jpg"
            if part not in ("cams", "plan") or not path.is_file():
                return self.send_json({"error": "no image"}, HTTPStatus.NOT_FOUND)
            return self.send_bytes(path.read_bytes(), "image/jpeg")
        if url.path == "/api/semantic":
            # Alpamayo semantic sidecar overlays (semantic_sidecar.py)
            view = parse_qs(url.query).get("view", ["front"])[0]
            live = DATA / "autodrive/semantic_live"
            if view == "status":
                path = live / "status.json"
                return self.send_bytes(path.read_bytes() if path.is_file() else b"{}", "application/json; charset=utf-8")
            if view not in ("front", "left", "right") or not (live / f"{view}.jpg").is_file():
                return self.send_json({"error": "no image"}, HTTPStatus.NOT_FOUND)
            return self.send_bytes((live / f"{view}.jpg").read_bytes(), "image/jpeg")
        if url.path == "/api/preview":
            view = parse_qs(url.query).get("view", ["front"])[0]
            path = CACHE.get("preview_" + view, 2.0, lambda: latest_preview(view))
            if not path or not path.is_file():
                return self.send_json({"error": "no image"}, HTTPStatus.NOT_FOUND)
            return self.send_bytes(path.read_bytes(), "image/jpeg")
        return self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self):
        url = urlparse(self.path)
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}") if length else {}
            if url.path in ("/api/drive/start", "/api/drive/stop"):
                name = body.get("profile")
                profile = requested_drive_profile({"profile": [name]})
                if url.path.endswith("/stop"):
                    job = JOBS.snapshot()
                    if job["running"] and job_network_profile(job) != profile["id"]:
                        raise ValueError("다른 구성의 주행이 실행 중임. 해당 구성 페이지에서 중지 필요")
                    return self.send_json({"ok": JOBS.stop()})
                collect = body.get("collect", False)
                if not isinstance(collect, bool) or (collect and name != "single_pc"):
                    raise ValueError("센서 수집은 1 PC용 페이지에서만 실행 가능")
                action = "autodrive_collect" if collect else "autodrive_follow"
                options = body.get("options", {})
                if not isinstance(options, dict):
                    raise ValueError("options must be a JSON object")
                JOBS.start(action, build_command(action, {
                    "network_profile": name, "signal_enabled": options.get("signal_enabled", True)}))
                return self.send_json({"ok": True, "profile": name})
            if url.path == "/api/start":
                action = str(body.get("action", ""))
                if MODE == "collect" and action not in ("autodrive_collect", "autodrive_follow"):
                    raise ValueError("이 웹은 알파마요 자율추종·수집만 실행합니다.")
                JOBS.start(action, build_command(action, body.get("options", {})))
                return self.send_json({"ok": True})
            if url.path == "/api/stop":
                return self.send_json({"ok": JOBS.stop()})
            if url.path == "/api/drivable-rules":
                return self.send_json({"ok": True, "rules": write_drivable_rules(body)})
            if url.path == "/api/delete-session":
                if not self.headers.get("Content-Type", "").lower().startswith("application/json"):
                    raise ValueError("JSON 요청 필요")
                origin = self.headers.get("Origin")
                if origin and urlparse(origin).netloc != self.headers.get("Host"):
                    raise ValueError("이 웹에서 보낸 삭제 요청만 허용")
                session = str(body.get("session", ""))
                if body.get("confirm") != session:
                    raise ValueError("삭제 확인 번호가 다름")
                result = delete_local_session(session)
                return self.send_json({"ok": True, **result})
            if url.path == "/api/follow":
                if MODE == "collect":
                    raise ValueError("이 웹은 알파마요 자율추종·수집만 실행합니다.")
                out = follow(str(body.get("action", "")), number(body.get("cruise_kmh", 30), "순항 속도", 5, 60), bool(body.get("avoid")))
                CACHE.items.pop("stack", None)
                return self.send_json({"ok": True, "message": out})
            return self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        except (ValueError, RuntimeError) as e:
            return self.send_json({"error": str(e)}, HTTPStatus.BAD_REQUEST)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8767)
    a = p.parse_args()
    (DATA / "autodrive").mkdir(parents=True, exist_ok=True)
    server = ThreadingHTTPServer((a.host, a.port), Handler)
    print(f"autodrive dashboard: http://127.0.0.1:{a.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
