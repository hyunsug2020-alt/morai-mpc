"""student_v3 GUI용 v6 그림자 실행 — GUI 쪽 연결부. 판단·경로를 '별도 프로세스'에서 계산해 기록만 한다.
주행 제어(경로·속도 발행)에는 절대 관여하지 않는다.

왜 별도 프로세스인가: 같은 프로세스에서 같은 GPU를 쓰면 실시간 추론의 torch.cuda.synchronize()가 그림자 연산까지
기다린다. 별도 프로세스는 CUDA 컨텍스트가 달라 실시간 쪽이 그림자를 기다리지 않고, 마스크 전처리(CPU)도 GUI의 GIL을 쓰지 않는다.

GUI 쪽이 하는 일 (가벼움):
  - ego 샘플을 /dev/shm 에 주기적으로 기록 (스레드, 판단 주기)
  - 실시간 추론이 끝날 때마다 시각을 공유 메모리(8바이트)에 기록 → 자식은 그 직후 15 ms 안에만 GPU 사용
  - 자식이 쓴 최신 요약을 읽어 runtime.v6_latest 에 둠 (결정 기록에 첨부)

켜는 법: 환경변수 STUDENT_V6_CHECKPOINT=<v6 best.pt>  (없으면 아무 일도 하지 않음)
  STUDENT_V6_HZ           판단 주기 (기본 10 Hz)
  STUDENT_V6_DEVICE       기본 cuda:0
  SEMANTIC_FEED_DIR       sidecar masks.npz 위치 (기본 /dev/shm/alpamayo_semantic)
  STUDENT_V6_MAX_MASK_AGE 어느 카메라든 이 초 넘게 새 프레임이 없으면 판단 보류 (기본 0.5)
  STUDENT_V6_MAX_EGO_AGE  ego 샘플이 이 초 넘게 오래되면 판단 보류 (기본 0.3)
  STUDENT_V6_LANE_MAP     HD맵 정제 파일 (기본 autodrive/ws/src/moraimpc/data/competition_directional_drivable_map.json.gz,
                          차선 정보 입력 모델(v6.2~)만 사용) / STUDENT_V6_LANE_CORRIDORS 우회 구간 설정
기록: <log_dir>/v6_shadow_<sid>.jsonl (자식이 씀), 자식 출력: 같은 이름 + .stderr
GUI 연결: Runtime.__init__ 끝 start_v6_shadow(self) / infer 루프 매 추론 뒤 notify_v6_idle(self) / 종료 시 stop_v6_shadow(self)
"""
from __future__ import annotations

import json
import mmap
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import uuid

import numpy as np

# 공유 메모리 기록 (seqlock): [seq_begin u64][idle_at f64][stop f64][seq_end u64]
# 쓰는 쪽: seq_begin=n → 값 → seq_end=n.  읽는 쪽: seq_end, 값, seq_begin 이 같을 때만 사용 (반쯤 쓴 값 방지)
IDLE_FMT = "<QddQ"
IDLE_SIZE = struct.calcsize(IDLE_FMT)
# HD맵 정제 파일 기본 위치: morai_competition/autodrive/ws/src/moraimpc/data (v6 패키지 = autodrive/student_v3/v6)
MAP_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "ws", "src", "moraimpc", "data")


class IdleRecord:
    def __init__(self, mm):
        self.mm = mm
        self.seq = 0
        self.lock = threading.Lock()
        self.idle_at = 0.0
        self.stop = 0.0

    def _write(self):
        with self.lock:
            self.seq += 1
            self.mm[0:8] = struct.pack("<Q", self.seq)
            self.mm[8:24] = struct.pack("<dd", self.idle_at, self.stop)
            self.mm[24:32] = struct.pack("<Q", self.seq)

    def set_idle(self, t):
        self.idle_at = t
        self._write()

    def request_stop(self):
        self.stop = 1.0
        self._write()


def read_idle_record(mm, tries=5):
    for _ in range(tries):
        end = struct.unpack("<Q", mm[24:32])[0]
        idle_at, stop = struct.unpack("<dd", mm[8:24])
        begin = struct.unpack("<Q", mm[0:8])[0]
        if begin == end:
            return idle_at, stop
    return None


def write_atomic_npz(path, **arrays):
    tmp = f"{path}.tmp.npz"
    np.savez(tmp, **arrays)
    os.replace(tmp, path)


class ShadowBridge(threading.Thread):
    """GUI 프로세스 쪽: ego 공급 + 결과 수거 + 자식 감시."""

    def __init__(self, runtime, env, checkpoint, log_path, child_cmd=None):
        super().__init__(name="v6-shadow-bridge", daemon=True)
        self.runtime = runtime
        self.hz = max(1.0, min(30.0, float(env.get("STUDENT_V6_HZ", "10"))))
        self.stop_event = threading.Event()
        self.child = None
        self.idle_map = self.idle_file = self.child_log = None
        self.dir = tempfile.mkdtemp(prefix="v6_shadow_", dir="/dev/shm" if os.path.isdir("/dev/shm") else None)
        try:
            self._setup(env, os.path.abspath(checkpoint), os.path.abspath(log_path), child_cmd)
        except Exception:
            self._release()                                  # 중간 실패 → 만든 파일·핸들 정리
            raise

    def _setup(self, env, checkpoint, log_path, child_cmd):
        self.ego_path = os.path.join(self.dir, "ego.npz")
        self.latest_path = os.path.join(self.dir, "latest.json")
        self.idle_path = os.path.join(self.dir, "idle.bin")
        with open(self.idle_path, "wb") as f:
            f.write(struct.pack(IDLE_FMT, 0, 0.0, 0.0, 0))
        self.idle_file = open(self.idle_path, "r+b")
        self.idle_map = mmap.mmap(self.idle_file.fileno(), IDLE_SIZE)
        self.idle = IdleRecord(self.idle_map)
        route_path = os.path.join(self.dir, "route.npy")
        np.save(route_path, np.asarray(self.runtime.waypoint_xy, np.float64))
        self.log_path = log_path
        self.child_log = open(log_path + ".stderr", "ab")
        cmd = child_cmd or [sys.executable, "-m", "v6.shadow_child"]
        args = ["--checkpoint", checkpoint, "--route", route_path, "--ego", self.ego_path, "--idle", self.idle_path,
                "--latest", self.latest_path, "--log", log_path, "--hz", str(self.hz),
                "--device", env.get("STUDENT_V6_DEVICE", "cuda:0"),
                "--shm", os.path.abspath(env.get("SEMANTIC_FEED_DIR", "/dev/shm/alpamayo_semantic")),
                "--max-mask-age", env.get("STUDENT_V6_MAX_MASK_AGE", "0.5"),
                "--max-ego-age", env.get("STUDENT_V6_MAX_EGO_AGE", "0.3"), "--parent-pid", str(os.getpid()),
                "--lane-map", os.path.abspath(env.get("STUDENT_V6_LANE_MAP", os.path.join(MAP_DIR, "competition_directional_drivable_map.json.gz"))),
                "--lane-corridors", os.path.abspath(env.get("STUDENT_V6_LANE_CORRIDORS", os.path.join(MAP_DIR, "local_bypass_corridor_overrides.json")))]
        pkg_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # student_v3 (v6 패키지의 부모)
        child_env = dict(os.environ)
        child_env["PYTHONPATH"] = pkg_parent + os.pathsep + child_env.get("PYTHONPATH", "")
        self.child = subprocess.Popen(cmd + args, cwd=pkg_parent, env=child_env, stdin=subprocess.DEVNULL,
                                      stdout=self.child_log, stderr=subprocess.STDOUT, start_new_session=True)
        self.latest_mtime = None

    # ---- GUI infer 루프에서 호출 (가벼움: 공유 메모리 8바이트 쓰기)
    def notify_idle(self):
        try:
            self.idle.set_idle(time.monotonic())
        except (ValueError, OSError, TypeError, AttributeError):
            pass

    def ego_samples(self):
        receiver = getattr(self.runtime, "ego_receiver", None)
        if receiver is None:
            return None
        lock = getattr(receiver, "lock", None)
        if lock is not None:
            with lock:
                return list(receiver.samples)
        return list(receiver.samples)

    def push_ego(self):
        samples = self.ego_samples()
        if not samples:
            return
        write_atomic_npz(self.ego_path,
                         t=np.array([s["received"] for s in samples], np.float64),
                         x=np.array([s["x"] for s in samples], np.float64),
                         y=np.array([s["y"] for s in samples], np.float64),
                         heading_deg=np.array([s["heading_deg"] for s in samples], np.float64))

    def pull_latest(self):
        try:
            mtime = os.stat(self.latest_path).st_mtime_ns
        except OSError:
            return
        if mtime == self.latest_mtime:
            return
        try:
            with open(self.latest_path) as f:
                latest = json.load(f)
        except (OSError, ValueError):
            return
        self.latest_mtime = mtime
        self.runtime.v6_latest = latest

    def run(self):
        period = 1.0 / self.hz
        while not self.stop_event.is_set():
            began = time.monotonic()
            try:
                self.push_ego()
                self.pull_latest()
                if self.child.poll() is not None:
                    self.pull_latest()
                    self.runtime.v6_latest = {"t": time.time(), "ok": False, "action": None, "p": None,
                                              "reason": f"child_exit:{self.child.returncode}"}
                    print(f"[v6-shadow] child exited rc={self.child.returncode} (see {self.log_path}.stderr)", flush=True)
                    break
            except Exception:
                traceback.print_exc()
            self.stop_event.wait(max(0.0, period - (time.monotonic() - began)))

    def stop(self, timeout=3.0):
        self.stop_event.set()
        try:
            self.idle.request_stop()                         # 자식에게 정지 요청
        except (ValueError, OSError, TypeError, AttributeError):
            pass
        if self.child is not None:
            try:
                self.child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.child.terminate()
                try:
                    self.child.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.child.kill()
                    self.child.wait(timeout=2.0)
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout=timeout)
        self._release()

    def _release(self):
        for obj in (self.idle_map, self.idle_file, self.child_log):
            try:
                if obj is not None:
                    obj.close()
            except (OSError, ValueError):
                pass
        try:
            for name in os.listdir(self.dir):
                os.remove(os.path.join(self.dir, name))
            os.rmdir(self.dir)
        except OSError:
            pass


def start_v6_shadow(runtime, environ=None, child_cmd=None):
    """체크포인트 환경변수가 있으면 그림자 프로세스 시작. 실패해도 GUI는 그대로 동작 (None 반환)."""
    env = os.environ if environ is None else environ
    runtime.v6_latest = None
    runtime.v6_worker = None
    checkpoint = env.get("STUDENT_V6_CHECKPOINT")
    if not checkpoint:
        return None
    bridge = None
    try:
        sid = os.path.basename(env.get("STUDENT_SESSION_DIR", "")) or f"nosave_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
        log_dir = env.get("STUDENT_LOG_DIR") or ("/data/autodrive" if os.path.isdir("/data/autodrive") else ".")
        os.makedirs(log_dir, exist_ok=True)
        bridge = ShadowBridge(runtime, env, checkpoint, os.path.join(log_dir, f"v6_shadow_{sid}.jsonl"), child_cmd=child_cmd)
        bridge.start()
        runtime.v6_worker = bridge
        print(f"[v6-shadow] child pid {bridge.child.pid}: {checkpoint} -> {bridge.log_path}", flush=True)
        return bridge
    except Exception:
        print("[v6-shadow] not started (GUI continues without it):", flush=True)
        traceback.print_exc()
        if bridge is not None:
            try:
                bridge.stop(timeout=1.0)
            except Exception:
                pass
        return None


def notify_v6_idle(runtime):
    """GUI infer 루프: 실시간 추론 한 번 끝날 때마다 호출 (자식은 이 직후 빈 시간에만 GPU 사용)."""
    worker = getattr(runtime, "v6_worker", None)
    if worker is not None:
        worker.notify_idle()


def stop_v6_shadow(runtime):
    worker = getattr(runtime, "v6_worker", None)
    if worker is not None:
        runtime.v6_worker = None
        worker.stop()


def v6_record_fields(runtime):
    """결정 기록(student_gui_*.jsonl)에 붙일 요약. 없으면 빈 dict."""
    latest = getattr(runtime, "v6_latest", None)
    if not isinstance(latest, dict) or "t" not in latest:
        return {}
    try:
        return {"v6": dict(latest, age_s=round(time.time() - float(latest["t"]), 2))}
    except (TypeError, ValueError):
        return {}
