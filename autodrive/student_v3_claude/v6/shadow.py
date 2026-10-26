"""v6 판단 모델 그림자 실행기 (student_v3 GUI에 붙일 모듈). 판단만 계산·기록, 주행 제어에는 쓰지 않는다.

입력은 학습과 같은 함수(runtime_inputs.py)로만 만든다:
  - 시맨틱: sidecar가 쓰는 <shm>/masks.npz 를 계속 읽어 카메라별 (시각, 320×180 클래스 ID) 1 s 버퍼 → select_frames
  - ego: GUI RosEgoReceiver.samples (/localization/ego_status) → resample_history → ego_features
  - 경로: 전역 경로 (x, y) 배열 → route_features (2 m × 20점)
GUI에서 쓰는 법:
  shadow = V6Shadow(ckpt, route_xy); shadow.start()
  result = shadow.infer(list(ego_receiver.samples))   # 매 판단 주기
"""
from __future__ import annotations

import math, threading, time
from collections import deque
from pathlib import Path

import numpy as np

from .lane_map import LaneMap
from .runtime_inputs import (SEM_INVALID, VIEW_ORDER, StudentRunner, ego_features, resample_history, route_cumulative,
                            route_features, select_frames, semantic_from_mask)


TRAIN_FRAME_LAG_S = 0.041     # 학습 클립(517 실측): 판단 시각 - 프레임 기준시각(카메라별 최신 중 가장 늦은 것) 평균 41 ms
MAX_CAMERA_SKEW_S = 0.15      # 학습 실측: 카메라 간 최신 프레임 차이 중앙값 1 ms, 최대 115 ms


class MaskBuffer(threading.Thread):
    """sidecar masks.npz 를 읽어 카메라별 (MORAI 시각 ns, 320×180 클래스 ID) 버퍼를 유지.
    파일 수정 시각이 바뀔 때만 읽는다. 카메라마다 '새 프레임이 마지막으로 들어온 시각'을 따로 기록 (한 카메라만 멈춰도 감지)."""
    def __init__(self, shm_dir="/dev/shm/alpamayo_semantic", keep_s=1.2, poll_s=0.01):
        super().__init__(name="v6-masks", daemon=True)
        self.path = Path(shm_dir) / "masks.npz"
        self.keep_ns = int(keep_s * 1e9); self.poll_s = poll_s
        self.lock = threading.Lock(); self.stop_event = threading.Event()
        self.frames = {v: deque() for v in VIEW_ORDER}      # (stamp_ns, ids[180,320])
        self.last_seq = None
        self.last_mtime = None
        self.last_file_version = None
        self.last_update_mono = None                         # 새 마스크 묶음을 받은 시각 (sidecar 멈춤 감지)
        self.view_advance_mono = {v: None for v in VIEW_ORDER}   # 카메라별 새 프레임 받은 시각

    @property
    def stop_flag(self):
        return self.stop_event.is_set()

    def poll_once(self):
        try:
            info = self.path.stat()
            mtime = info.st_mtime_ns
            version = (info.st_ino, mtime)
        except OSError:
            return
        if version == self.last_file_version:
            return
        try:
            with np.load(self.path) as d:
                seq = int(d["seq"])
                if seq == self.last_seq:
                    self.last_mtime = mtime
                    self.last_file_version = version
                    return
                items = [(v, int(d[f"{v}_stamp_ns"]), semantic_from_mask(d[f"{v}_mask"], d[f"{v}_place"])) for v in VIEW_ORDER
                         if f"{v}_mask" in d]
        except (FileNotFoundError, OSError, ValueError, KeyError, EOFError):
            return                                           # 교체 중 → 다음 폴링에서 다시
        self.last_mtime, self.last_seq = mtime, seq
        self.last_file_version = version
        now = time.monotonic()
        with self.lock:
            self.last_update_mono = now
            for v, st, ids in items:
                q = self.frames[v]
                if q and q[-1][0] >= st:
                    continue                                 # 이 카메라는 새 프레임 아님
                q.append((st, ids))
                self.view_advance_mono[v] = now
                while q and st - q[0][0] > self.keep_ns:
                    q.popleft()

    def run(self):
        while not self.stop_event.is_set():
            self.poll_once()
            self.stop_event.wait(self.poll_s)

    def snapshot(self):
        with self.lock:
            return {v: list(self.frames[v]) for v in VIEW_ORDER}, dict(self.view_advance_mono), self.last_update_mono

    def close(self, timeout=1.0):
        self.stop_event.set()
        if self.is_alive():
            self.join(timeout=timeout)


class V6Shadow:
    def __init__(self, checkpoint, route_xy, shm_dir="/dev/shm/alpamayo_semantic", device="cuda", model_module=None,
                 lane_map=None, lane_corridors=None):
        if model_module is None:
            from . import driving_student as model_module   # 학습 model.py 복사본 (재구현 금지)
        self.runner = StudentRunner(checkpoint, device, model_module)
        self.route_xy = np.asarray(route_xy, np.float64)[:, :2]
        self.route_cum = route_cumulative(self.route_xy)
        self.lane_map = None
        if getattr(self.runner.config, "route_features", 4) > 4:   # HD맵 차선 정보 입력 모델 → 정제 파일 필수
            if not lane_map:
                raise ValueError("this checkpoint needs the HD map lane file (competition_directional_drivable_map.json.gz)")
            self.lane_map = LaneMap(lane_map, lane_corridors)
            self.lane_cum = route_cumulative(self.lane_map.points)
        self.masks = MaskBuffer(shm_dir)
        self.nearest = None

    def start(self):
        self.masks.start()

    def inputs(self, ego_samples, time_key="received", now=None, max_mask_age_s=None, max_ego_age_s=None,
               frame_lag_correction=False):
        """now: 판단 시각(ego 시간축, 기본=마지막 ego 샘플 시각). 학습 클립은 판단 시각 기준으로 만든다.
        max_mask_age_s: 어느 카메라든 이 시간 넘게 새 프레임이 없으면 판단하지 않음 (오래된 화면으로 판단 금지).
        max_ego_age_s: 마지막 ego 샘플이 now 보다 이만큼 넘게 오래되면 판단하지 않음.
        frame_lag_correction: 주행 중엔 sidecar 처리 지연만큼 카메라가 ego보다 늦다. 학습(판단 시각 t 이전 프레임,
            평균 ~35 ms 묵음)과 같은 관계가 되도록 ego 기준 시각을 그 초과 지연만큼 당긴다 (MORAI 시각 = 호스트 벽시계)."""
        snap, advanced, last = self.masks.snapshot()
        mono = time.monotonic()
        if max_mask_age_s is not None:
            if last is None or mono - last > max_mask_age_s:
                return None, "masks_stale"
            stale = [v for v in VIEW_ORDER if advanced.get(v) is None or mono - advanced[v] > max_mask_age_s]
            if stale:
                return None, "camera_stale:" + ",".join(stale)
        if any(len(snap[v]) < 1 for v in VIEW_ORDER):
            return None, "no_masks"
        stamps = {v: np.asarray([s for s, _ in snap[v]], np.int64) for v in VIEW_ORDER}
        latest = [int(stamps[v][-1]) for v in VIEW_ORDER]
        if max_mask_age_s is not None and (max(latest) - min(latest)) / 1e9 > MAX_CAMERA_SKEW_S:
            return None, "camera_skew"
        frame_anchor_s = min(latest) / 1e9
        idx, valid = select_frames(stamps)
        sem = np.full((4, 3, 180, 320), SEM_INVALID, np.uint8)
        for k in range(4):
            for j, v in enumerate(VIEW_ORDER):
                if valid[k, j]:
                    sem[k, j] = snap[v][idx[k][j]][1]
        if len(ego_samples) < 5:
            return None, "no_ego"
        t = np.asarray([s[time_key] for s in ego_samples], np.float64)
        order = np.argsort(t, kind="stable"); t = t[order]
        x = np.asarray([ego_samples[i]["x"] for i in order]); y = np.asarray([ego_samples[i]["y"] for i in order])
        yaw = np.unwrap(np.radians([ego_samples[i]["heading_deg"] for i in order]))
        keep = np.concatenate([[True], np.diff(t) > 1e-6])
        t, x, y, yaw = t[keep], x[keep], y[keep], yaw[keep]
        t_end = t[-1] if now is None else float(now)
        if max_ego_age_s is not None and t_end - t[-1] > max_ego_age_s:
            return None, "ego_stale"
        if frame_lag_correction:
            # select_frames 와 같은 기준시각(카메라별 최신 중 가장 늦은 것)으로 지연을 잰다
            excess = (time.time() - frame_anchor_s) - TRAIN_FRAME_LAG_S
            t_end -= min(max(excess, 0.0), 0.3)
        keep_now = t <= t_end + 1e-9
        t, x, y, yaw = t[keep_now], x[keep_now], y[keep_now], yaw[keep_now]
        if len(t) < 5:
            return None, "no_ego"
        lo = int(np.searchsorted(t, t_end - 3.0))
        hist_xy, hist_yaw = resample_history(t[lo:], x[lo:], y[lo:], yaw[lo:], t_end)
        ego = ego_features(hist_xy, hist_yaw)
        route = route_features(self.route_xy, self.route_cum, hist_xy[-1], hist_yaw[-1])
        lane_map = getattr(self, "lane_map", None)
        if lane_map is not None:          # backfill_route_lane.py 와 같은 계산 (맵 경로 최근접 진행거리 → 앞 20점)
            s0 = lane_map.nearest_station(lane_map.points, self.lane_cum, hist_xy[-1])
            route = np.concatenate([route, lane_map.features(s0)], 1)
        return (sem, ego, route, valid), None

    def infer_packed(self, packed):
        """GPU 부분만 (입력 준비는 inputs 에서 CPU로 끝낸 뒤)."""
        sem, ego, route, valid = packed
        t0 = time.perf_counter()
        out = self.runner.infer(sem, ego, route, valid)
        out.update(ok=True, infer_ms=(time.perf_counter() - t0) * 1e3, camera_valid=valid.tolist())
        return out

    def infer(self, ego_samples, time_key="received", now=None, **checks):
        packed, why = self.inputs(ego_samples, time_key, now, **checks)
        if packed is None:
            return {"ok": False, "reason": why}
        sem, ego, route, valid = packed
        t0 = time.perf_counter()
        out = self.runner.infer(sem, ego, route, valid)
        out.update(ok=True, infer_ms=(time.perf_counter() - t0) * 1e3, camera_valid=valid.tolist())
        return out

    def close(self):
        """마스크 스레드 정지 + 모델 해제 (GPU 메모리 반환)."""
        self.masks.close()
        runner = getattr(self, "runner", None)
        self.runner = None
        if runner is not None:
            torch = runner.torch
            dev = getattr(runner, "device", None)
            del runner
            if dev is not None and getattr(dev, "type", "") == "cuda":
                torch.cuda.empty_cache()   # 해제된 블록만 반환 — 실시간 모델 메모리에는 영향 없음
