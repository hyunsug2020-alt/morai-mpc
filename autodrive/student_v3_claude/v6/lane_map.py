"""HD맵 정제 파일(competition_directional_drivable_map.json.gz) → 경로 차선 정보. 학습(클립 보완)과 주행(GUI)이 같은 함수를 쓴다.

글로벌 패스 점마다 (그 점이 매칭된 link의 옆 차선 정보, 맵 기준이라 교차로에서도 값이 있음):
  0 left_legal   왼쪽에 합법 차선변경 가능한 같은 방향 차선 (legal_same_direction_lateral, side=left)
  1 right_legal  오른쪽 동일
  2 opp_left     왼쪽에 반대 방향 차선 (inferred_opposite_direction_parallel) = 왼쪽에 노란선
  3 left_free_m  왼쪽으로 갈 수 있는 폭: 반대 차선이 있으면 (가장 왼쪽 같은 방향 차선 중심 + 반대 차선 중심)/2 = 노란선,
                 없으면 가장 왼쪽 같은 방향 차선 중심 + 반폭 (도로 끝)
  4 right_free_m 오른쪽: 가장 오른쪽 같은 방향 차선 중심 + 반폭
  5 bypass_side  우회 구간 설정(local_bypass_corridor_overrides): +1 왼쪽만, -1 오른쪽만, 0 설정 없음
모델 경로 입력 20점(2 m 간격)마다 이 6값을 붙인다 → route [20, 4+6].
"""
from __future__ import annotations

import gzip
import json
from pathlib import Path

import numpy as np

LANE_COLUMNS = ("left_legal", "right_legal", "opp_left", "left_free_m", "right_free_m", "bypass_side")
LANE_FEATURES = len(LANE_COLUMNS)
HALF_WIDTH_M = 1.75


def _open_json(path):
    path = Path(path)
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8") as f:
            return json.load(f)
    return json.loads(path.read_text(encoding="utf-8"))


class LaneMap:
    """글로벌 패스 점별 차선 정보 표 (points, cum, table[N,6])."""

    def __init__(self, drivable_map_path, corridors_path=None):
        d = _open_json(drivable_map_path)
        if d.get("format") != "morai_directional_drivable_map":
            raise ValueError("not a morai_directional_drivable_map")
        self.points = np.asarray(d["global_route"]["points"], np.float64)[:, :2]
        seg = np.diff(self.points, axis=0)
        self.cum = np.concatenate([[0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
        edges = d["edges"]
        links = d["links"]
        side_corr = {}
        if corridors_path and Path(corridors_path).is_file():
            for c in _open_json(corridors_path).get("corridors", []):
                if not c.get("enabled", True):
                    continue
                sides = set(c.get("allowed_sides") or [])
                side_corr[c["link_id"]] = 1.0 if sides == {"left"} else (-1.0 if sides == {"right"} else 0.0)
        per_link = {}
        for lid in links:
            def sides(kind):
                return [(e["side"], float(e["separation_m"])) for e in (edges.get(kind, {}).get(lid) or [])]
            legal = sides("legal_same_direction_lateral")
            same = legal + sides("inferred_same_direction_parallel")
            opp = sides("inferred_opposite_direction_parallel")
            half = 0.5 * float(links[lid].get("width_start") or 2 * HALF_WIDTH_M)
            left_same = max([s for side, s in same if side == "left"], default=0.0)
            right_same = max([s for side, s in same if side == "right"], default=0.0)
            opp_left = [s for side, s in opp if side == "left"]
            if opp_left and min(opp_left) > left_same:
                left_free = 0.5 * (left_same + min(opp_left))
            else:
                left_free = left_same + half
            per_link[lid] = np.array([float(any(side == "left" for side, _ in legal)), float(any(side == "right" for side, _ in legal)),
                                      float(bool(opp_left)), left_free, right_same + half, side_corr.get(lid, 0.0)], np.float32)
        self.table = np.stack([per_link.get(m["link_id"], np.zeros(LANE_FEATURES, np.float32))
                               for m in d["global_route"]["matches"]]).astype(np.float32)
        if len(self.table) != len(self.points):
            raise ValueError("route matches and points differ in length")

    def features(self, s0: float, n: int = 20, spacing: float = 2.0) -> np.ndarray:
        """경로 진행거리 s0(최근접 점)에서 앞쪽 n점(spacing 간격)의 차선 정보 [n,6]. 경로 끝 너머는 0.
        runtime_inputs.route_features 와 같은 점(st = s0 + (k+1)*spacing)을 쓴다."""
        out = np.zeros((n, LANE_FEATURES), np.float32)
        for k in range(n):
            st = s0 + (k + 1) * spacing
            if st > self.cum[-1]:
                break
            j = int(np.searchsorted(self.cum, st))
            out[k] = self.table[min(max(j, 0), len(self.table) - 1)]
        return out

    def nearest_station(self, route_xy: np.ndarray, route_cum: np.ndarray, pos_xy) -> float:
        """route_features 와 같은 최근접 점(전체 탐색) → 진행거리."""
        nearest = int(np.linalg.norm(np.asarray(route_xy) - np.asarray(pos_xy)[None], axis=1).argmin())
        return float(route_cum[nearest])
