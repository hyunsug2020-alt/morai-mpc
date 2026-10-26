"""v6 그림자 실행 — 자식 프로세스 (GUI가 python -m v6.shadow_child 로 띄움). 주행 제어에 관여하지 않는다.

- 실시간 추론 끝 신호(공유 메모리) 뒤 15 ms 안에만 GPU를 쓴다. 신호가 없으면 아무것도 하지 않는다.
- 모델 적재도 첫 신호 뒤에 한다.
- 부모(GUI)가 죽거나 정지 요청이 오면 스스로 끝난다.
"""
from __future__ import annotations

import argparse
import json
import mmap
import os
import struct
import sys
import time
import traceback

import numpy as np

IDLE_FMT = "<QddQ"     # seqlock: [seq_begin][idle_at][stop][seq_end] (shadow_runtime.IdleRecord)


def read_idle_record(mm, tries=5):
    for _ in range(tries):
        end = struct.unpack("<Q", mm[24:32])[0]
        idle_at, stop = struct.unpack("<dd", mm[8:24])
        begin = struct.unpack("<Q", mm[0:8])[0]
        if begin == end:
            return idle_at, stop
    return None
ACTION_SHORT = {"KEEP_LANE": "KEEP", "LEFT_CHANGE": "LEFT", "RIGHT_CHANGE": "RIGHT", "TURN_LEFT": "TL",
                "TURN_RIGHT": "TR", "STOP": "STOP", "YIELD": "YIELD", "UNKNOWN": "UNKNOWN"}


class Child:
    def __init__(self, a, shadow_factory=None):
        self.a = a
        self.idle_file = open(a.idle, "r+b")
        self.idle_map = mmap.mmap(self.idle_file.fileno(), struct.calcsize(IDLE_FMT))
        self.log = open(a.log, "a", buffering=1)
        self.route = np.load(a.route)
        self.shadow_factory = shadow_factory
        self.shadow = None
        self.last_idle = 0.0
        self.ego_mtime = None
        self.ego = None
        self.checks = {"max_mask_age_s": a.max_mask_age, "max_ego_age_s": a.max_ego_age, "frame_lag_correction": True}

    def idle_state(self):
        rec = read_idle_record(self.idle_map)
        return rec if rec is not None else (self.last_idle, 0.0)   # 쓰는 중 → 이번엔 새 신호 없음으로

    def parent_alive(self):
        return os.getppid() == self.a.parent_pid

    def wait_slot(self, poll_s=0.002):
        """새 '실시간 추론 끝' 신호를 받고 15 ms 안이면 True. 정지·부모 종료면 None."""
        while True:
            idle_at, stop = self.idle_state()
            if stop or not self.parent_alive():
                return None
            now = time.monotonic()
            if idle_at > self.last_idle:
                self.last_idle = idle_at
                if now - idle_at <= self.a.idle_fresh:
                    return True
            time.sleep(poll_s)

    def ego_samples(self):
        try:
            mtime = os.stat(self.a.ego).st_mtime_ns
        except OSError:
            return self.ego
        if mtime != self.ego_mtime:
            try:
                with np.load(self.a.ego) as d:
                    self.ego = [{"received": float(t), "x": float(x), "y": float(y), "heading_deg": float(h)}
                                for t, x, y, h in zip(d["t"], d["x"], d["y"], d["heading_deg"])]
                self.ego_mtime = mtime
            except (OSError, ValueError, KeyError, EOFError):
                pass
        return self.ego

    def write(self, result):
        try:
            self.log.write(json.dumps(result, ensure_ascii=False) + "\n")
        except (OSError, ValueError, TypeError):
            pass
        ok = bool(result.get("ok"))
        probs = result.get("action_probabilities") or {}
        latest = {"t": result["t"], "ok": ok, "action": ACTION_SHORT.get(result.get("action"), result.get("action")) if ok else None,
                  "p": round(max(probs.values()), 3) if ok and probs else None, "reason": None if ok else result.get("reason")}
        tmp = self.a.latest + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(latest, f)
            os.replace(tmp, self.a.latest)
        except OSError:
            pass

    def in_window(self):
        return time.monotonic() - self.last_idle <= self.a.idle_fresh

    def load(self):
        """CPU로 먼저 읽고(언제든), GPU로 옮기기·예열은 빈 시간에 (start-up 1회)."""
        factory = self.shadow_factory
        if factory is None:
            from .shadow import V6Shadow
            factory = V6Shadow
        self.shadow = factory(self.a.checkpoint, self.route, shm_dir=self.a.shm, device="cpu",
                              lane_map=self.a.lane_map or None, lane_corridors=self.a.lane_corridors or None)
        runner = getattr(self.shadow, "runner", None)
        if runner is not None and str(self.a.device).startswith("cuda"):
            torch = runner.torch
            # GUI 실시간 추론이 아직 시작 전이면(신호 없음) 지금 바로 GPU 적재·예열 — 주행 전에 끝냄.
            # 이미 돌고 있으면 단계마다 빈 시간을 기다림 (별도 CUDA 컨텍스트라 실시간 쪽이 기다리지는 않음).
            before_gui = self.idle_state()[0] == 0.0
            if not before_gui and self.wait_slot() is None:
                return False
            runner.device = torch.device(self.a.device)
            runner.model.to(runner.device)
            c = runner.config
            for _ in range(2):                               # 예열 (cudnn 알고리즘 선택)
                if not before_gui and self.wait_slot() is None:
                    return False
                runner.infer(np.full((c.frames, c.cameras, c.height, c.width), 8, np.uint8),
                             np.zeros((c.ego_steps, 6), np.float32), np.zeros((c.route_points, getattr(c, "route_features", 4)), np.float32))
        self.shadow.start()
        return True

    def step(self):
        samples = self.ego_samples()
        if not samples:
            return {"ok": False, "reason": "no_ego"}
        packed, why = self.shadow.inputs(samples, time_key="received", now=time.monotonic(), **self.checks)   # CPU
        if packed is None:
            return {"ok": False, "reason": why}
        if not self.in_window():                         # GPU에 넣기 직전 다시 확인 — 창이 지났으면 건너뜀
            return None
        started = time.monotonic()
        result = self.shadow.infer_packed(packed)        # GPU
        result["slot_delay_ms"] = round((started - self.last_idle) * 1e3, 2)
        path = result.pop("path")
        result["path_xy"] = [[round(float(x), 2), round(float(y), 2)] for x, y, _ in path[4::5]]   # 0.5 s 간격
        result["action_probabilities"] = {k: round(float(v), 4) for k, v in result["action_probabilities"].items()}
        result["concepts"] = {k: round(float(v), 3) for k, v in result["concepts"].items()}
        result["infer_ms"] = round(float(result["infer_ms"]), 2)
        return result

    def run(self):
        try:
            if self.idle_state()[1] or not self.parent_alive():      # 이미 정지 요청·부모 종료
                return 0
            try:
                if not self.load():
                    return 0
                print(f"[v6-shadow child] loaded {self.a.checkpoint}", flush=True)
            except Exception as error:
                traceback.print_exc()
                self.write({"t": time.time(), "ok": False, "reason": "load_failed", "error": repr(error)[:300], "disabled": True})
                return 2
            period, last, failures = 1.0 / self.a.hz, 0.0, 0
            while True:
                slot = self.wait_slot()
                if slot is None:
                    return 0
                if time.monotonic() - last < period:
                    continue
                last = time.monotonic()
                try:
                    result = self.step()
                    failures = 0
                    if result is None:                       # 창을 놓침 → 다음 빈 시간에
                        last = 0.0
                        continue
                except Exception as error:
                    failures += 1
                    if failures == 1:
                        traceback.print_exc()
                    result = {"ok": False, "reason": "error", "error": repr(error)[:300]}
                    if failures >= 20:
                        result["disabled"] = True
                result["t"] = time.time()
                self.write(result)
                if result.get("disabled"):
                    return 3
        finally:
            self.close()

    def close(self):
        shadow, self.shadow = self.shadow, None
        if shadow is not None:
            try:
                shadow.close()
            except Exception:
                pass
        for closer in (self.log.close, self.idle_map.close, self.idle_file.close):
            try:
                closer()
            except (OSError, ValueError):
                pass


def parse(argv=None):
    ap = argparse.ArgumentParser()
    for k in ("checkpoint", "route", "ego", "idle", "latest", "log", "shm"):
        ap.add_argument("--" + k, required=True)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--max-mask-age", type=float, default=0.5)
    ap.add_argument("--max-ego-age", type=float, default=0.3)
    ap.add_argument("--idle-fresh", type=float, default=0.015)
    ap.add_argument("--parent-pid", type=int, required=True)
    ap.add_argument("--lane-map", default="")
    ap.add_argument("--lane-corridors", default="")
    return ap.parse_args(argv)


def main(argv=None, shadow_factory=None):
    return Child(parse(argv), shadow_factory).run()


if __name__ == "__main__":
    sys.exit(main())
