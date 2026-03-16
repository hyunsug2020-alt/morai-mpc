#!/usr/bin/env python3
"""
Hybrid A* 경로 계획기
======================
논문 기반:
  1. "하이브리드의 근본 논문" - Dolgov et al., Hybrid A* fundamentals
  2. "하이브리드의 구조적 설계" - Guided Hybrid A* for Valet Parking
  3. "하이브리드와 mpc의 구조적 연결" - Hybrid A* + MPC 계층 구조

특징:
  - 3D 상태 (x, y, theta) 격자 탐색
  - 운동학적 자전거 모델 확장
  - Reed-Shepp 단축 경로 (해석적 확장)
  - 후진 지원
  - Voronoi 필드 비용 (장애물 회피)
  - Guided Hybrid A* (가시성 다이어그램 기반 휴리스틱)
"""

import heapq
import math
import numpy as np
from typing import Optional, List, Tuple, Dict
from dataclasses import dataclass, field


# ============================================================
# 데이터 구조
# ============================================================

@dataclass
class HybridAStarConfig:
    """Hybrid A* 설정"""
    # 격자 해상도
    xy_resolution: float  = 0.10    # [m] x,y 해상도
    theta_resolution: float = math.radians(5.0)  # [rad] 방향 해상도

    # 차량 파라미터
    wheelbase: float = 0.30        # [m] 축간거리
    max_steer: float = math.radians(35.0)  # [rad] 최대 조향각
    vehicle_length: float = 0.45   # [m] 차량 길이
    vehicle_width: float  = 0.25   # [m] 차량 너비

    # 확장 파라미터
    n_steer: int = 5               # 조향각 이산화 수
    step_size: float = 0.05        # [m] 확장 스텝 크기
    n_steps: int = 3               # 스텝당 시뮬레이션 수

    # Reed-Shepp 확장
    use_reed_shepp: bool = True    # Reed-Shepp 단축 경로 사용
    rs_every_n: int = 5            # N 노드마다 RS 시도

    # 비용 가중치
    w_forward:  float = 1.0        # 전진 비용
    w_reverse:  float = 2.5        # 후진 비용 (더 비쌈)
    w_steer:    float = 0.3        # 조향 비용
    w_change:   float = 0.5        # 기어 전환 비용
    w_obstacle: float = 50.0       # 장애물 패널티
    w_voronoi:  float = 5.0        # Voronoi 필드 가중치
    w_heuristic: float = 1.0       # 휴리스틱 가중치

    # 탐색 제한
    max_nodes: int = 100000        # 최대 탐색 노드 수
    goal_tolerance_xy: float = 0.15    # [m] 목표 위치 허용 오차
    goal_tolerance_theta: float = math.radians(10.0)  # [rad] 목표 방향 허용 오차


@dataclass
class HybridNode:
    """Hybrid A* 노드"""
    x: float          # 연속 x 위치
    y: float          # 연속 y 위치
    theta: float      # 방향 [rad]
    g: float = 0.0    # 시작점부터 비용
    f: float = 0.0    # 전체 비용 f = g + h
    steer: float = 0.0   # 이 노드로 오는 조향각
    direction: int = 1   # 1=전진, -1=후진
    parent: Optional['HybridNode'] = field(default=None, repr=False)

    def __lt__(self, other):
        return self.f < other.f

    def grid_key(self, cfg: HybridAStarConfig) -> Tuple[int, int, int]:
        """격자 셀 인덱스"""
        xi = int(round(self.x / cfg.xy_resolution))
        yi = int(round(self.y / cfg.xy_resolution))
        ti = int(round(self.theta / cfg.theta_resolution)) % int(
            round(2 * math.pi / cfg.theta_resolution))
        return (xi, yi, ti)


@dataclass
class Obstacle:
    """장애물 (사각형)"""
    cx: float   # 중심 x
    cy: float   # 중심 y
    w: float    # 너비
    h: float    # 높이
    angle: float = 0.0  # 회전각 [rad]


# ============================================================
# Reed-Shepp 경로 (최단 연결 경로)
# Reeds-Shepp curves: 전진/후진을 포함한 최적 연결
# ============================================================

class ReedShepp:
    """Reed-Shepp 경로 생성기 (간소화 버전)"""

    @staticmethod
    def path_length(x: float, y: float, phi: float,
                    max_curvature: float) -> float:
        """Reed-Shepp 경로 길이 추정 (휴리스틱용)"""
        # 간소화: 회전 반경 고려한 거리 + 방향 비용
        min_radius = 1.0 / max_curvature if max_curvature > 1e-6 else 1e6
        dist = math.sqrt(x*x + y*y)
        # 방향 오차를 경로 비용으로 환산
        theta_cost = abs(phi) * min_radius
        return max(dist, theta_cost * 0.5)

    @staticmethod
    def generate_path(start_x: float, start_y: float, start_theta: float,
                      goal_x: float, goal_y: float, goal_theta: float,
                      max_curvature: float,
                      step_size: float = 0.05) -> Optional[List[Tuple[float, float, float]]]:
        """
        Reed-Shepp 경로 생성
        반환: [(x, y, theta), ...] 또는 None (경로 없을 경우)

        간소화: LSL 타입 RS 경로 시도
        """
        if max_curvature < 1e-6:
            return None

        R = 1.0 / max_curvature

        # 목표까지 상대 좌표 변환
        dx = goal_x - start_x
        dy = goal_y - start_y
        c  = math.cos(start_theta)
        s  = math.sin(start_theta)
        lx = dx * c + dy * s
        ly = -dx * s + dy * c
        phi = goal_theta - start_theta
        phi = math.atan2(math.sin(phi), math.cos(phi))  # 정규화

        # 간단한 직선 + 회전 경로 시도 (단순 CSC)
        # 더 완전한 RS 구현은 reeds_shepp_paths 라이브러리 참고
        path = []
        dist = math.sqrt(dx*dx + dy*dy)

        # 단거리 직선 접근 가능 시
        if dist < 0.5 and abs(phi) < math.radians(30):
            n_pts = max(int(dist / step_size), 2)
            for i in range(n_pts + 1):
                t = i / n_pts
                x = start_x + t * dx
                y = start_y + t * dy
                th = start_theta + t * phi
                path.append((x, y, th))
            return path

        # 원호 + 직선 + 원호 시도 (LSL 타입)
        # 정밀도를 위해 단순화된 2-호 경로 사용
        # 중간점을 통한 경유
        mid_x = (start_x + goal_x) / 2
        mid_y = (start_y + goal_y) / 2
        mid_theta = math.atan2(goal_y - start_y, goal_x - start_x)

        # 1단계: 시작 → 중간 (원호)
        path1 = ReedShepp._arc_path(start_x, start_y, start_theta,
                                     mid_x, mid_y, max_curvature, step_size)
        # 2단계: 중간 → 목표 (원호)
        path2 = ReedShepp._arc_path(mid_x, mid_y, mid_theta,
                                     goal_x, goal_y, max_curvature, step_size,
                                     target_theta=goal_theta)
        if path1 and path2:
            return path1 + path2[1:]

        return None

    @staticmethod
    def _arc_path(x0: float, y0: float, theta0: float,
                  xg: float, yg: float, max_curvature: float,
                  step_size: float,
                  target_theta: Optional[float] = None) -> List[Tuple[float, float, float]]:
        """시작점 → 목표점 원호 경로"""
        dist = math.sqrt((xg-x0)**2 + (yg-y0)**2)
        if dist < 1e-6:
            return [(x0, y0, theta0)]

        n_pts = max(int(dist / step_size), 2)
        path = []
        for i in range(n_pts + 1):
            t = i / n_pts
            x = x0 + t * (xg - x0)
            y = y0 + t * (yg - y0)
            if target_theta is not None:
                theta = theta0 + t * (target_theta - theta0)
            else:
                theta = math.atan2(yg - y0, xg - x0)
            path.append((x, y, theta))
        return path


# ============================================================
# Voronoi 필드 (장애물 회피 비용)
# ============================================================

class VoronoiField:
    """
    Voronoi 필드 비용 맵
    논문: ρV(x,y) = α/(α+dO) · (dV/(dO+dV)) · ((dO-dO_max)²/dO_max²)
    """

    def __init__(self, grid_size_x: int, grid_size_y: int,
                 resolution: float, alpha: float = 1.0,
                 d_max: float = 2.0):
        self.gx = grid_size_x
        self.gy = grid_size_y
        self.res = resolution
        self.alpha = alpha
        self.d_max = d_max
        # 거리 맵: -1 = 미계산
        self.obstacle_dist = np.full((grid_size_x, grid_size_y), d_max)
        self.voronoi_dist  = np.full((grid_size_x, grid_size_y), d_max)

    def update(self, obstacles: List[Tuple[int, int]]) -> None:
        """장애물 셀 목록으로 거리 맵 업데이트 (간소화: BFS)"""
        if not obstacles:
            return

        # BFS로 장애물 거리 계산
        from collections import deque
        dist_arr = np.full((self.gx, self.gy), float('inf'))
        q = deque()
        for (ox, oy) in obstacles:
            if 0 <= ox < self.gx and 0 <= oy < self.gy:
                dist_arr[ox, oy] = 0.0
                q.append((ox, oy))

        while q:
            x, y = q.popleft()
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                if 0 <= nx < self.gx and 0 <= ny < self.gy:
                    nd = dist_arr[x,y] + 1.0
                    if nd < dist_arr[nx, ny]:
                        dist_arr[nx, ny] = nd
                        q.append((nx, ny))

        self.obstacle_dist = dist_arr * self.res

    def cost(self, xi: int, yi: int) -> float:
        """Voronoi 필드 비용"""
        if not (0 <= xi < self.gx and 0 <= yi < self.gy):
            return self.d_max
        dO = self.obstacle_dist[xi, yi]
        if dO >= self.d_max:
            return 0.0
        # ρV = α/(α+dO) · (dO-d_max)²/d_max²
        rho = (self.alpha / (self.alpha + dO)) * ((dO - self.d_max)**2 / self.d_max**2)
        return max(0.0, rho)


# ============================================================
# Hybrid A* 메인 클래스
# ============================================================

class HybridAStarPlanner:
    """
    Hybrid A* 경로 계획기

    사용법:
        planner = HybridAStarPlanner(config)
        planner.set_obstacles(obstacles)
        path = planner.plan(start_x, start_y, start_theta,
                            goal_x, goal_y, goal_theta)
        # path: [(x, y, theta, direction), ...] 또는 None
    """

    def __init__(self, cfg: Optional[HybridAStarConfig] = None):
        self.cfg = cfg or HybridAStarConfig()
        self.obstacles: List[Obstacle] = []
        self.obstacle_cells: set = set()  # 격자 셀 집합
        self.voronoi: Optional[VoronoiField] = None
        self._rs = ReedShepp()

    def set_obstacles(self, obstacles: List[Obstacle],
                      map_origin_x: float = -10.0,
                      map_origin_y: float = -10.0,
                      map_size_x: float = 20.0,
                      map_size_y: float = 20.0) -> None:
        """장애물 설정 및 격자 생성"""
        self.obstacles = obstacles
        res = self.cfg.xy_resolution

        gx = int(map_size_x / res) + 1
        gy = int(map_size_y / res) + 1
        self._origin_x = map_origin_x
        self._origin_y = map_origin_y

        # 장애물 셀 마킹
        self.obstacle_cells = set()
        obs_list = []
        for obs in obstacles:
            # 차량 크기 포함 팽창 (마진 추가)
            margin = max(self.cfg.vehicle_width, self.cfg.vehicle_length) / 2 + 0.05
            half_w = obs.w / 2 + margin
            half_h = obs.h / 2 + margin

            # 로컬 코너 → 전역 좌표
            corners_local = [(-half_w, -half_h), (half_w, -half_h),
                             (half_w,  half_h), (-half_w,  half_h)]
            c, s = math.cos(obs.angle), math.sin(obs.angle)
            for lx, ly in corners_local:
                gx_c = lx * c - ly * s + obs.cx
                gy_c = lx * s + ly * c + obs.cy

            # 격자 셀 채우기 (바운딩 박스 기반)
            x_min = obs.cx - half_w - 0.1
            x_max = obs.cx + half_w + 0.1
            y_min = obs.cy - half_h - 0.1
            y_max = obs.cy + half_h + 0.1

            xi_min = int((x_min - map_origin_x) / res)
            xi_max = int((x_max - map_origin_x) / res) + 1
            yi_min = int((y_min - map_origin_y) / res)
            yi_max = int((y_max - map_origin_y) / res) + 1

            for xi in range(xi_min, xi_max + 1):
                for yi in range(yi_min, yi_max + 1):
                    if 0 <= xi < gx and 0 <= yi < gy:
                        self.obstacle_cells.add((xi, yi))
                        obs_list.append((xi, yi))

        # Voronoi 필드 계산
        self.voronoi = VoronoiField(gx, gy, res)
        self.voronoi.update(obs_list)

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        res = self.cfg.xy_resolution
        xi = int(round((x - self._origin_x) / res))
        yi = int(round((y - self._origin_y) / res))
        return (xi, yi)

    def _is_collision(self, x: float, y: float, theta: float) -> bool:
        """차량 충돌 검사 (간소화: 3점 검사)"""
        half_l = self.cfg.vehicle_length / 2
        half_w = self.cfg.vehicle_width / 2
        c, s = math.cos(theta), math.sin(theta)

        check_pts = [
            (x + half_l * c, y + half_l * s),   # 전방
            (x - half_l * c, y - half_l * s),   # 후방
            (x, y),                               # 중심
            (x + half_l * c - half_w * s, y + half_l * s + half_w * c),  # 전방 좌
            (x + half_l * c + half_w * s, y + half_l * s - half_w * c),  # 전방 우
        ]
        for px, py in check_pts:
            gi = self._world_to_grid(px, py)
            if gi in self.obstacle_cells:
                return True
        return False

    def _heuristic(self, node: HybridNode, gx: float, gy: float,
                   g_theta: float, max_curvature: float) -> float:
        """
        휴리스틱: max(유클리드 거리, Reed-Shepp 경로 길이)
        Guided Hybrid A*: 가시성 다이어그램 기반 waypoint 활용 가능
        """
        dx = gx - node.x
        dy = gy - node.y
        d_theta = abs(math.atan2(math.sin(g_theta - node.theta),
                                  math.cos(g_theta - node.theta)))

        # 비홀로노믹 휴리스틱 (Reed-Shepp)
        rs_len = self._rs.path_length(dx, dy, g_theta - node.theta,
                                       max_curvature)
        # 홀로노믹 휴리스틱 (유클리드)
        eucl = math.sqrt(dx*dx + dy*dy)

        return self.cfg.w_heuristic * max(eucl, rs_len)

    def _expand(self, node: HybridNode) -> List[HybridNode]:
        """노드 확장: 전진/후진 × 여러 조향각"""
        cfg = self.cfg
        successors = []
        max_curvature = math.tan(cfg.max_steer) / cfg.wheelbase

        # 조향각 후보
        steers = np.linspace(-cfg.max_steer, cfg.max_steer, cfg.n_steer)

        for direction in [1, -1]:  # 전진, 후진
            for steer in steers:
                # 자전거 모델 시뮬레이션
                x, y, theta = node.x, node.y, node.theta
                total_dist = 0.0

                for _ in range(cfg.n_steps):
                    # 운동학: ψ̇ = v/L · tan(δ)
                    beta = cfg.step_size * direction * math.tan(steer) / cfg.wheelbase
                    x += cfg.step_size * direction * math.cos(theta)
                    y += cfg.step_size * direction * math.sin(theta)
                    theta = math.atan2(math.sin(theta + beta),
                                       math.cos(theta + beta))
                    total_dist += cfg.step_size

                # 충돌 검사
                if self._is_collision(x, y, theta):
                    continue

                # 비용 계산
                move_cost = total_dist
                if direction == -1:
                    move_cost *= cfg.w_reverse
                else:
                    move_cost *= cfg.w_forward

                steer_cost = cfg.w_steer * abs(steer)
                change_cost = cfg.w_change if direction != node.direction else 0.0

                # Voronoi 비용
                voronoi_cost = 0.0
                if self.voronoi is not None:
                    gi = self._world_to_grid(x, y)
                    voronoi_cost = cfg.w_voronoi * self.voronoi.cost(gi[0], gi[1])

                g_new = node.g + move_cost + steer_cost + change_cost + voronoi_cost

                child = HybridNode(
                    x=x, y=y, theta=theta,
                    g=g_new, f=g_new,  # f는 나중에 h 더함
                    steer=steer,
                    direction=direction,
                    parent=node
                )
                successors.append(child)

        return successors

    def plan(self,
             start_x: float, start_y: float, start_theta: float,
             goal_x: float,  goal_y: float,  goal_theta: float,
             map_origin_x: float = None,
             map_origin_y: float = None) -> Optional[List[Tuple[float, float, float, int]]]:
        """
        Hybrid A* 경로 계획

        반환: [(x, y, theta, direction), ...] 또는 None
               direction: 1=전진, -1=후진
        """
        cfg = self.cfg
        max_curvature = math.tan(cfg.max_steer) / cfg.wheelbase

        # 맵 기준 설정 (미설정시 시작점 주변)
        if not hasattr(self, '_origin_x'):
            self._origin_x = start_x - 10.0
            self._origin_y = start_y - 10.0

        # 시작 노드
        start = HybridNode(x=start_x, y=start_y, theta=start_theta,
                           g=0.0, direction=1)
        h = self._heuristic(start, goal_x, goal_y, goal_theta, max_curvature)
        start.f = h

        # 우선순위 큐
        open_list = []
        heapq.heappush(open_list, start)
        closed_dict: Dict[Tuple[int,int,int], float] = {}  # key → g 값

        n_expanded = 0

        while open_list and n_expanded < cfg.max_nodes:
            current = heapq.heappop(open_list)
            key = current.grid_key(cfg)

            # 이미 방문한 더 좋은 노드 있으면 스킵
            if key in closed_dict and closed_dict[key] <= current.g:
                continue
            closed_dict[key] = current.g
            n_expanded += 1

            # 목표 도달 확인
            dx = abs(current.x - goal_x)
            dy = abs(current.y - goal_y)
            d_theta = abs(math.atan2(math.sin(goal_theta - current.theta),
                                      math.cos(goal_theta - current.theta)))

            if (dx < cfg.goal_tolerance_xy and
                dy < cfg.goal_tolerance_xy and
                d_theta < cfg.goal_tolerance_theta):
                return self._reconstruct_path(current, goal_x, goal_y, goal_theta)

            # Reed-Shepp 단축 경로 시도
            if cfg.use_reed_shepp and n_expanded % cfg.rs_every_n == 0:
                rs_path = self._rs.generate_path(
                    current.x, current.y, current.theta,
                    goal_x, goal_y, goal_theta,
                    max_curvature, cfg.step_size)
                if rs_path is not None:
                    # RS 경로 충돌 검사
                    valid = all(
                        not self._is_collision(x, y, th)
                        for x, y, th in rs_path
                    )
                    if valid:
                        # RS 경로를 최종 경로로 추가
                        return self._reconstruct_path_with_rs(
                            current, rs_path)

            # 확장
            children = self._expand(current)
            for child in children:
                ck = child.grid_key(cfg)
                if ck in closed_dict and closed_dict[ck] <= child.g:
                    continue
                h = self._heuristic(child, goal_x, goal_y, goal_theta, max_curvature)
                child.f = child.g + h
                heapq.heappush(open_list, child)

        # 경로 없음
        return None

    def _reconstruct_path(self,
                          node: HybridNode,
                          goal_x: float, goal_y: float,
                          goal_theta: float
                          ) -> List[Tuple[float, float, float, int]]:
        """노드 체인에서 경로 재구성"""
        path = []
        cur = node
        while cur is not None:
            path.append((cur.x, cur.y, cur.theta, cur.direction))
            cur = cur.parent
        path.reverse()

        # 목표점 추가
        if path:
            path.append((goal_x, goal_y, goal_theta, path[-1][3]))

        return path

    def _reconstruct_path_with_rs(self,
                                   node: HybridNode,
                                   rs_path: List[Tuple[float, float, float]]
                                   ) -> List[Tuple[float, float, float, int]]:
        """A* 경로 + Reed-Shepp 연결 경로"""
        base = self._reconstruct_path(node, node.x, node.y, node.theta)
        for x, y, th in rs_path[1:]:
            base.append((x, y, th, 1))
        return base

    @staticmethod
    def smooth_path(raw_path: List[Tuple[float, float, float, int]],
                    n_iter: int = 50,
                    w_smooth: float = 0.5,
                    w_data: float = 0.5
                    ) -> List[Tuple[float, float, float, int]]:
        """
        켤레 기울기 강하를 이용한 경로 평활화
        논문: "하이브리드의 근본 논문" 제안 방법
        """
        if len(raw_path) < 3:
            return raw_path

        xs = [p[0] for p in raw_path]
        ys = [p[1] for p in raw_path]
        dirs = [p[3] for p in raw_path]

        xs_smooth = xs[:]
        ys_smooth = ys[:]

        for _ in range(n_iter):
            for i in range(1, len(xs) - 1):
                # 데이터 항
                xs_smooth[i] += w_data * (xs[i] - xs_smooth[i])
                ys_smooth[i] += w_data * (ys[i] - ys_smooth[i])
                # 평활화 항
                xs_smooth[i] += w_smooth * (
                    xs_smooth[i-1] + xs_smooth[i+1] - 2*xs_smooth[i])
                ys_smooth[i] += w_smooth * (
                    ys_smooth[i-1] + ys_smooth[i+1] - 2*ys_smooth[i])

        # 방향 재계산
        result = []
        for i in range(len(xs_smooth)):
            if i < len(xs_smooth) - 1:
                dx = xs_smooth[i+1] - xs_smooth[i]
                dy = ys_smooth[i+1] - ys_smooth[i]
                theta = math.atan2(dy, dx)
            else:
                theta = raw_path[-1][2]
            result.append((xs_smooth[i], ys_smooth[i], theta, dirs[i]))

        return result

    @staticmethod
    def path_to_ros_waypoints(path: List[Tuple[float, float, float, int]],
                               ds: float = 0.05
                               ) -> List[Tuple[float, float, float]]:
        """
        Hybrid A* 경로 → ROS PoseStamped 리스트 변환용 (x, y, theta)
        등간격 보간 수행
        """
        if not path:
            return []

        # 누적 거리 계산
        dists = [0.0]
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            dists.append(dists[-1] + math.sqrt(dx*dx + dy*dy))

        total = dists[-1]
        if total < 1e-6:
            return [(path[0][0], path[0][1], path[0][2])]

        # 등간격 보간
        result = []
        s = 0.0
        while s <= total:
            # 인덱스 찾기
            idx = 0
            for j in range(len(dists)-1):
                if dists[j] <= s <= dists[j+1]:
                    idx = j
                    break
            t = (s - dists[idx]) / max(dists[idx+1] - dists[idx], 1e-9)
            x = path[idx][0] + t * (path[idx+1][0] - path[idx][0])
            y = path[idx][1] + t * (path[idx+1][1] - path[idx][1])
            # 방향 보간
            th_a, th_b = path[idx][2], path[idx+1][2]
            dth = math.atan2(math.sin(th_b-th_a), math.cos(th_b-th_a))
            theta = th_a + t * dth
            result.append((x, y, theta))
            s += ds

        # 마지막 점
        if result and math.sqrt((result[-1][0]-path[-1][0])**2 +
                                 (result[-1][1]-path[-1][1])**2) > ds/2:
            result.append((path[-1][0], path[-1][1], path[-1][2]))

        return result


# ============================================================
# 단독 테스트
# ============================================================
if __name__ == '__main__':
    cfg = HybridAStarConfig(
        xy_resolution=0.1,
        theta_resolution=math.radians(5),
        wheelbase=0.30,
        max_steer=math.radians(35),
    )

    planner = HybridAStarPlanner(cfg)
    # 장애물 설정 (주차 공간 주변 벽)
    obstacles = [
        Obstacle(cx=1.0, cy=0.5,  w=0.1, h=2.0),
        Obstacle(cx=1.0, cy=-0.5, w=0.1, h=2.0),
    ]
    planner.set_obstacles(obstacles,
                          map_origin_x=-1.0, map_origin_y=-2.0,
                          map_size_x=4.0,  map_size_y=4.0)

    print("Hybrid A* 경로 계획 중...")
    path = planner.plan(
        start_x=0.0, start_y=0.0, start_theta=0.0,
        goal_x=2.0,  goal_y=0.0,  goal_theta=0.0
    )

    if path:
        print(f"경로 찾음: {len(path)}개 점")
        smooth = HybridAStarPlanner.smooth_path(path)
        waypoints = HybridAStarPlanner.path_to_ros_waypoints(smooth)
        print(f"보간 후: {len(waypoints)}개 웨이포인트")
    else:
        print("경로 없음!")
