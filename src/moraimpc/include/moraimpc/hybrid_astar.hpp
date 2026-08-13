// Hybrid A* for non-holonomic vehicle path planning
// 기반: Dolgov et al. 2008, "Practical Search Techniques in Path Planning for Autonomous Driving"
//
// 특징:
// - 5-state 자전거 모델 motion primitives
// - Forward + Reverse (D/R) 자동 전환
// - 그리드 (xy_res, th_res) 이산화 + 연속 상태 유지
// - Reverse penalty + Direction switch penalty
// - Euclidean heuristic (Phase 2 MVP — RS expansion은 추후)
// - 장애물: ObstacleGrid (콘 inflation)
//
// 사용:
//   HybridAStar planner(cfg);
//   std::vector<PathPoint> path;
//   bool ok = planner.plan(start, goal, grid, path);

#pragma once

#include <vector>
#include <cstdint>

namespace moraimpc {

struct HState {
    double x;     // m
    double y;     // m
    double yaw;   // rad, [-π, π]
};

struct HAStarConfig {
    // 차량
    double L          = 3.0;                    // wheelbase
    double max_steer  = 35.0 * 3.141592653589793 / 180.0;
    int    n_steer    = 5;                      // steering 샘플 수 (홀수, 0 포함)
    double step_len   = 0.7;                    // primitive 진행 거리 [m]

    // 비용
    double reverse_penalty = 1.5;               // 후진 비용 multiplier
    double switch_penalty  = 2.0;               // D↔R 전환 페널티

    // 그리드
    double xy_res = 0.5;                        // 위치 격자 [m]
    double th_res = 5.0 * 3.141592653589793 / 180.0;

    // Goal
    double goal_xy_thresh = 0.5;                // [m]
    double goal_th_thresh = 10.0 * 3.141592653589793 / 180.0;

    // 검색 한계
    int    max_iter = 20000;

    // 차량 footprint (간단 충돌체크용 — 3 circle 근사)
    double veh_length = 4.6;
    double veh_width  = 1.9;
    double safety     = 0.30;                   // 추가 마진
};

struct PathPoint {
    double x, y, yaw;
    int    gear;          // +1 forward, -1 reverse
    double steer;         // 이 점까지 도달한 steering [rad]
};

// 2D Occupancy Grid (콘 inflation 적용)
class ObstacleGrid {
public:
    ObstacleGrid() = default;
    ObstacleGrid(double origin_x, double origin_y, double res, int W, int H);
    void reset();
    void inflateCircle(double cx, double cy, double r);

    bool isFree(double x, double y) const;
    // 차량 footprint (3 원 근사: 앞·중간·뒤) 충돌 체크
    bool isFreeFootprint(double x, double y, double yaw,
                         double veh_L, double veh_W, double safety) const;

    int  width()  const { return W_; }
    int  height() const { return H_; }
    double res()  const { return res_; }
    double originX() const { return ox_; }
    double originY() const { return oy_; }
    const std::vector<uint8_t>& data() const { return grid_; }

    bool worldToGrid(double x, double y, int& i, int& j) const;

private:
    double ox_ = 0, oy_ = 0;
    double res_ = 0.25;
    int    W_ = 0, H_ = 0;
    std::vector<uint8_t> grid_;  // 0=free, 1=occupied
};

class HybridAStar {
public:
    explicit HybridAStar(const HAStarConfig& cfg);

    // 검색 실행. 성공 시 path 채움 (start → goal 순서).
    bool plan(const HState& start, const HState& goal,
              const ObstacleGrid& grid,
              std::vector<PathPoint>& path) const;

    // 진단용
    int last_iter() const { return last_iter_; }

private:
    HAStarConfig cfg_;
    mutable int last_iter_ = 0;
};

}  // namespace moraimpc
