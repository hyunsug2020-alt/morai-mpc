#include "moraimpc/hybrid_astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace moraimpc {

namespace {
inline double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}

// ─────────────────────────────────────────────────────────────────────────────
// ObstacleGrid
// ─────────────────────────────────────────────────────────────────────────────
ObstacleGrid::ObstacleGrid(double ox, double oy, double res, int W, int H)
    : ox_(ox), oy_(oy), res_(res), W_(W), H_(H), grid_(W * H, 0) {}

void ObstacleGrid::reset() {
    std::fill(grid_.begin(), grid_.end(), 0);
}

bool ObstacleGrid::worldToGrid(double x, double y, int& i, int& j) const {
    i = static_cast<int>(std::floor((x - ox_) / res_));
    j = static_cast<int>(std::floor((y - oy_) / res_));
    return (i >= 0 && i < W_ && j >= 0 && j < H_);
}

void ObstacleGrid::inflateCircle(double cx, double cy, double r) {
    int ci, cj;
    if (!worldToGrid(cx, cy, ci, cj)) {
        // 중심 grid 밖이어도 영향권은 안에 들어올 수 있음 — 박스로 처리
    }
    int rcells = static_cast<int>(std::ceil(r / res_));
    int i0 = std::max(0, ci - rcells);
    int i1 = std::min(W_ - 1, ci + rcells);
    int j0 = std::max(0, cj - rcells);
    int j1 = std::min(H_ - 1, cj + rcells);
    for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
            double cellx = ox_ + (i + 0.5) * res_;
            double celly = oy_ + (j + 0.5) * res_;
            double dx = cellx - cx, dy = celly - cy;
            if (dx*dx + dy*dy <= r * r) grid_[j * W_ + i] = 1;
        }
    }
}

bool ObstacleGrid::isFree(double x, double y) const {
    int i, j;
    if (!worldToGrid(x, y, i, j)) return false;  // 그리드 밖 = 미지 → 안전상 occupied 취급
    return grid_[j * W_ + i] == 0;
}

bool ObstacleGrid::isFreeFootprint(double x, double y, double yaw,
                                   double veh_L, double veh_W, double safety) const {
    // 3-circle 근사: 차량 길이 따라 3개 원
    double r = 0.5 * veh_W + safety;
    double cs = std::cos(yaw), sn = std::sin(yaw);
    double offsets[3] = { -veh_L * 0.35, 0.0, veh_L * 0.35 };
    for (double off : offsets) {
        double cx = x + cs * off;
        double cy = y + sn * off;
        // 원 내부 격자 sample (5개 점만 빠르게)
        for (double a = 0.0; a < 2 * M_PI; a += M_PI / 4.0) {
            double sx = cx + r * std::cos(a);
            double sy = cy + r * std::sin(a);
            if (!isFree(sx, sy)) return false;
        }
        if (!isFree(cx, cy)) return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// HybridAStar
// ─────────────────────────────────────────────────────────────────────────────
HybridAStar::HybridAStar(const HAStarConfig& cfg) : cfg_(cfg) {}

namespace {

// 셀 인덱스 → 64비트 해시
inline uint64_t cellKey(int xi, int yi, int ti) {
    auto u = [](int v) { return static_cast<uint32_t>(v + (1 << 19)); };  // -524288~+524287
    return (static_cast<uint64_t>(u(xi)) << 40) |
           (static_cast<uint64_t>(u(yi) & 0xFFFFF) << 20) |
            static_cast<uint64_t>(u(ti) & 0xFFFFF);
}

struct Node {
    HState s;
    double g, h;
    int    parent;     // -1 = root
    int    gear;       // +1 / -1
    double steer;
    int    xi, yi, ti;
};

}  // namespace

bool HybridAStar::plan(const HState& start, const HState& goal,
                       const ObstacleGrid& grid,
                       std::vector<PathPoint>& path) const {
    last_iter_ = 0;
    path.clear();

    auto idx = [&](double v, double res) { return static_cast<int>(std::floor(v / res)); };
    auto t_idx = [&](double yaw) {
        double a = wrapPi(yaw) + M_PI;       // [0, 2π)
        return static_cast<int>(std::floor(a / cfg_.th_res));
    };

    auto heur = [&](const HState& s) {
        // Euclidean + 약한 yaw 차이
        double dx = goal.x - s.x, dy = goal.y - s.y;
        double dth = std::abs(wrapPi(goal.yaw - s.yaw));
        return std::hypot(dx, dy) + 0.2 * dth;
    };

    auto goalReached = [&](const HState& s) {
        double dx = goal.x - s.x, dy = goal.y - s.y;
        double dth = std::abs(wrapPi(goal.yaw - s.yaw));
        return std::hypot(dx, dy) < cfg_.goal_xy_thresh && dth < cfg_.goal_th_thresh;
    };

    // Motion primitive 생성: steering N개 × {forward, reverse}
    std::vector<double> steers;
    if (cfg_.n_steer <= 1) {
        steers = {0.0};
    } else {
        int n = cfg_.n_steer;
        for (int k = 0; k < n; ++k) {
            double t = (n == 1) ? 0.5 : (double)k / (n - 1);
            steers.push_back(-cfg_.max_steer + 2.0 * cfg_.max_steer * t);
        }
    }

    std::vector<Node> nodes;
    nodes.reserve(cfg_.max_iter * 2);

    // start 노드
    Node n0;
    n0.s = start; n0.g = 0; n0.h = heur(start);
    n0.parent = -1; n0.gear = +1; n0.steer = 0.0;
    n0.xi = idx(start.x, cfg_.xy_res);
    n0.yi = idx(start.y, cfg_.xy_res);
    n0.ti = t_idx(start.yaw);
    nodes.push_back(n0);

    using PQItem = std::pair<double, int>;  // f, node_idx
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open;
    open.push({n0.g + n0.h, 0});

    std::unordered_map<uint64_t, double> closed;  // cell → best g

    while (!open.empty() && last_iter_ < cfg_.max_iter) {
        ++last_iter_;
        auto [f, ni] = open.top(); open.pop();
        const Node cur = nodes[ni];

        uint64_t key = cellKey(cur.xi, cur.yi, cur.ti);
        auto it = closed.find(key);
        if (it != closed.end() && it->second <= cur.g) continue;
        closed[key] = cur.g;

        if (goalReached(cur.s)) {
            // backtrack
            int p = ni;
            std::vector<PathPoint> rev;
            while (p >= 0) {
                const Node& nd = nodes[p];
                rev.push_back({nd.s.x, nd.s.y, nd.s.yaw, nd.gear, nd.steer});
                p = nd.parent;
            }
            std::reverse(rev.begin(), rev.end());
            path = std::move(rev);
            return true;
        }

        // expand
        for (double st : steers) {
            for (int dir : {+1, -1}) {
                double Ds = (dir > 0) ? cfg_.step_len : -cfg_.step_len;
                double dyaw = (Ds / cfg_.L) * std::tan(st);
                double yaw_mid = cur.s.yaw + 0.5 * dyaw;
                double nx = cur.s.x + Ds * std::cos(yaw_mid);
                double ny = cur.s.y + Ds * std::sin(yaw_mid);
                double nyaw = wrapPi(cur.s.yaw + dyaw);

                // 충돌 체크
                if (!grid.isFreeFootprint(nx, ny, nyaw,
                                          cfg_.veh_length, cfg_.veh_width, cfg_.safety)) {
                    continue;
                }

                double cost = std::abs(Ds);
                if (dir < 0) cost *= cfg_.reverse_penalty;
                if (dir != cur.gear && cur.parent >= 0) cost += cfg_.switch_penalty;
                double g_new = cur.g + cost;

                Node nn;
                nn.s = {nx, ny, nyaw};
                nn.g = g_new; nn.h = heur(nn.s);
                nn.parent = ni; nn.gear = dir; nn.steer = st;
                nn.xi = idx(nx, cfg_.xy_res);
                nn.yi = idx(ny, cfg_.xy_res);
                nn.ti = t_idx(nyaw);

                uint64_t nkey = cellKey(nn.xi, nn.yi, nn.ti);
                auto cit = closed.find(nkey);
                if (cit != closed.end() && cit->second <= g_new) continue;

                int new_idx = static_cast<int>(nodes.size());
                nodes.push_back(nn);
                open.push({g_new + nn.h, new_idx});
            }
        }
    }
    return false;
}

}  // namespace moraimpc
