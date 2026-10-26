"""학습(build_clips/data.py)과 주행 GUI가 '똑같은' 모델 입력을 만들도록 하는 공용 모듈.

2026-10-07: 주행 GUI(student_v3)가 학습과 다른 입력을 넣고 있었다.
  - 모델 구조를 따로 다시 짜서 영상 특징 격자(5x4↔4x5)와 결합 순서가 뒤바뀜
  - ego 6칸을 [앞, 옆, 속도, 가속, 브레이크, 조향]으로 넣음 (학습: [x, y, z, sin, cos, 속도])
  - 경로 20점을 0.5 m 간격·3번째 칸 누적거리로 넣음 (학습: 2 m 간격, 3번째 칸 0)
  - 이미지 축소를 BILINEAR로 함 (학습: INTER_AREA)
  - 카메라 순서를 센서 설정 순서(전방·왼쪽·오른쪽)로 넣음 (학습: 왼쪽·전방·오른쪽)
이 모듈의 함수만 쓰면 학습과 주행 입력이 일치한다. test_runtime_inputs.py가 녹화 세션으로 일치를 검사한다.
numpy·opencv만 사용 (torch는 StudentRunner에서만).
"""
from __future__ import annotations

import math

import cv2
import numpy as np

HISTORY_STEPS = 16          # ego_steps
HISTORY_DT_S = 0.1          # 10 Hz, 1.5 s
ROUTE_POINTS = 20
ROUTE_SPACING_M = 2.0       # build_clips --route-spacing-m 기본값
VIEW_ORDER = ("left", "front", "right")   # 모델 카메라 순서 = 학습 기준(data.py camera_indices, serve_student.py)


# ---------------- 프레임 선택 (주행 GUI CameraReceiver.snapshot 과 같은 규칙) ----------------
FRAME_INTERVAL_NS = 100_000_000
FRAME_MAX_ERROR_NS = 80_000_000     # 목표 시각과 이보다 멀면 그 카메라 프레임은 무효(camera_valid=0)


def select_frames(stamps_by_view: dict, count: int = 4, interval_ns: int = FRAME_INTERVAL_NS,
                  max_error_ns: int = FRAME_MAX_ERROR_NS, views=None):
    """각 카메라의 수신 프레임 시각(오름차순, ns)에서 모델용 프레임을 고른다.

    기준 = 각 카메라 최신 프레임 시각 중 가장 이른 것. 그 기준에서 0.1 s 간격 4개 목표 시각마다
    카메라별로 가장 가까운 프레임을 고른다(주행 GUI와 동일). 반환: (indices [count][view], valid [count, views]).
    """
    views = list(views or VIEW_ORDER)
    arrays = {v: np.asarray(stamps_by_view[v], np.int64) for v in views}
    if any(len(a) == 0 for a in arrays.values()):
        raise ValueError("camera without frames")
    latest = min(int(a[-1]) for a in arrays.values())
    targets = [latest - interval_ns * i for i in reversed(range(count))]
    indices, valid = [], np.zeros((count, len(views)), np.float32)
    for k, t in enumerate(targets):
        row = []
        for j, v in enumerate(views):
            a = arrays[v]
            i = int(np.abs(a - t).argmin())
            row.append(i)
            valid[k, j] = float(abs(int(a[i]) - t) <= max_error_ns)
        indices.append(row)
    return indices, valid


# ---------------- 시맨틱 입력 (주행 sidecar masks.npz 형식 → 모델 클래스 ID) ----------------
# sidecar 마스크 값: 0 차량, 1 사람·동물, 2 장애물, 3 신호등(일반 카메라엔 없음), 5 흰 차선, 254 내 차 영역, 255 배경
SEMANTIC_CLASSES = 8                 # 모델 Config.classes. ID 8 = 무효(프레임 없음)
SEM_BACKGROUND, SEM_EGO, SEM_INVALID = 6, 7, 8


def semantic_from_mask(mask: np.ndarray, place, width: int = 320, height: int = 180) -> np.ndarray:
    """레터박스된 모델 해상도 마스크 + place(left, top, w, h, src_w, src_h) → 학습 입력 [H,W] uint8 클래스 ID."""
    left, top, w, h = (int(v) for v in list(place)[:4])
    crop = np.asarray(mask)[top:top + h, left:left + w]
    small = cv2.resize(crop, (width, height), interpolation=cv2.INTER_NEAREST)
    out = small.copy()
    out[small == 255] = SEM_BACKGROUND
    out[small == 254] = SEM_EGO
    out[(small > 5) & (small < 254)] = SEM_BACKGROUND
    return out.astype(np.uint8)


def preprocess_bgr(bgr: np.ndarray, width: int = 320, height: int = 180) -> np.ndarray:
    """카메라 원본 BGR → 모델 입력 RGB uint8 [3,H,W]. build_clips와 동일(cv2 INTER_AREA)."""
    small = cv2.resize(bgr, (width, height), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(small, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)


def preprocess_jpeg(jpeg: bytes, width: int = 320, height: int = 180) -> np.ndarray:
    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("undecodable jpeg")
    return preprocess_bgr(bgr, width, height)


def ego_features(xy: np.ndarray, yaw_rad: np.ndarray, z: np.ndarray | None = None, dt_s: float = HISTORY_DT_S) -> np.ndarray:
    """오래된→최신 순 16개 자세(0.1 s 간격) → 모델 ego 입력 [16,6] = [x, y, z, sin(yaw), cos(yaw), speed].

    좌표는 최신 자세 기준 (x 앞, y 왼쪽), yaw도 최신 기준 상대값. build_clips.ego_history + data.py 와 동일.
    """
    xy = np.asarray(xy, np.float64).reshape(-1, 2)
    yaw = np.unwrap(np.asarray(yaw_rad, np.float64).reshape(-1))
    zz = np.zeros(len(xy)) if z is None else np.asarray(z, np.float64).reshape(-1)
    if len(xy) != HISTORY_STEPS or len(yaw) != HISTORY_STEPS:
        raise ValueError(f"need {HISTORY_STEPS} poses")
    c, s = math.cos(yaw[-1]), math.sin(yaw[-1])
    d = xy - xy[-1]
    local = np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], zz - zz[-1]], -1)
    rel = yaw - yaw[-1]
    speed = np.linalg.norm(np.diff(local[:, :2], axis=0), axis=1) / dt_s
    speed = np.concatenate([speed[:1], speed])
    return np.concatenate([local, np.sin(rel)[:, None], np.cos(rel)[:, None], speed[:, None]], 1).astype(np.float32)


def resample_history(times_s, x, y, yaw_rad, t_end_s, steps: int = HISTORY_STEPS, dt_s: float = HISTORY_DT_S):
    """수신 샘플(시각 오름차순)을 t_end 기준 0.1 s 격자로 '선형 보간'한다 (학습 build_clips.ego_history와 같은 방식).

    가장 가까운 샘플을 고르면 수신 간격이 들쭉날쭉해 위치 차이로 계산하는 속도 칸이 최대 수 m/s 튄다.
    반환: (xy [steps,2], yaw [steps]) — 오래된→최신.
    """
    t = np.asarray(times_s, np.float64)
    q = t_end_s - dt_s * np.arange(steps - 1, -1, -1)
    q = np.clip(q, t[0], t[-1])           # 이력이 1.5 s보다 짧으면 처음 값으로 채움(주행 시작 직후)
    yaw = np.unwrap(np.asarray(yaw_rad, np.float64))
    return np.stack([np.interp(q, t, x), np.interp(q, t, y)], -1), np.interp(q, t, yaw)


def route_cumulative(route_xy: np.ndarray) -> np.ndarray:
    seg = np.diff(np.asarray(route_xy, np.float64), axis=0)
    return np.concatenate([[0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])


def route_features(route_xy: np.ndarray, cum: np.ndarray, pos_xy, yaw_rad: float,
                   n: int = ROUTE_POINTS, spacing: float = ROUTE_SPACING_M, nearest: int | None = None) -> np.ndarray:
    """전역 경로 → 모델 route 입력 [20,4] = [앞, 옆, 0, 유효]. build_clips.route_ahead 와 동일(2 m 간격, 40 m).

    nearest: 호출자가 이미 아는 최근접 경로점 인덱스(주행 GUI). 없으면 전체 탐색(학습과 같음).
    """
    route_xy = np.asarray(route_xy, np.float64)
    pos = np.asarray(pos_xy, np.float64)
    if nearest is None:
        nearest = int(np.linalg.norm(route_xy - pos[None], axis=1).argmin())
    s0 = cum[nearest]
    out = np.zeros((n, 4), np.float32)
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    for k in range(n):
        st = s0 + (k + 1) * spacing
        if st > cum[-1]:
            break
        j = int(np.searchsorted(cum, st))
        j = min(max(j, 1), len(cum) - 1)
        a = (st - cum[j - 1]) / max(cum[j] - cum[j - 1], 1e-9)
        p = route_xy[j - 1] * (1 - a) + route_xy[j] * a
        dx, dy = p[0] - pos[0], p[1] - pos[1]
        out[k] = [c * dx + s * dy, -s * dx + c * dy, 0.0, 1.0]
    return out


class StudentRunner:
    """학습 코드의 DrivingStudent를 그대로 불러 추론한다 (모델 구조를 따로 다시 짜지 않는다)."""

    def __init__(self, checkpoint_path: str, device: str = "cuda", model_module=None):
        import torch
        if model_module is None:
            from . import driving_student as model_module   # 학습 model.py 복사본
        self.torch = torch
        ck = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        if ck.get("architecture") != "morai-semantic-student-v1":
            raise ValueError("morai-semantic-student-v1 체크포인트가 아님")
        self.config = model_module.Config(**ck["config"])
        self.model = model_module.DrivingStudent(self.config)
        self.model.load_state_dict(ck["model"], strict=True)
        self.device = torch.device(device if torch.cuda.is_available() or device == "cpu" else "cpu")
        self.model.eval().to(self.device)
        self.actions = list(ck["actions"])
        self.concepts = list(ck.get("concepts", []))
        self.metadata = ck
        # 정답으로 학습하지 않은 행동(UNKNOWN 등)은 판단 후보에서 뺀다.
        self.allowed = np.array([a != "UNKNOWN" for a in self.actions])

    def infer(self, rgb: np.ndarray, ego: np.ndarray, route: np.ndarray, camera_valid: np.ndarray | None = None) -> dict:
        """rgb uint8 [T,C,3,H,W] (T: 오래된→최신, C: VIEW_ORDER), ego [16,6], route [20,4]."""
        torch = self.torch
        c = self.config
        if camera_valid is None:
            camera_valid = np.ones((c.frames, c.cameras), np.float32)
        with torch.inference_mode():
            out = self.model(torch.from_numpy(np.ascontiguousarray(rgb))[None].to(self.device),
                             torch.from_numpy(np.asarray(ego, np.float32))[None].to(self.device),
                             torch.from_numpy(np.asarray(route, np.float32))[None].to(self.device),
                             torch.from_numpy(np.asarray(camera_valid, np.float32))[None].to(self.device))
        logits = out[0][0].float().cpu().numpy()
        logits = np.where(self.allowed, logits, -np.inf)
        probs = np.exp(logits - logits.max()); probs /= probs.sum()
        idx = int(probs.argmax())
        concepts = torch.sigmoid(out[3][0].float()).cpu().numpy()
        return {"action": self.actions[idx], "action_index": idx, "action_probabilities": dict(zip(self.actions, probs.tolist())),
                "path": out[1][0].float().cpu().numpy(), "concepts": dict(zip(self.concepts, concepts.tolist()))}
