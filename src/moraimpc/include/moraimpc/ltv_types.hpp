#pragma once
#include <cmath>
#include <vector>
#include <Eigen/Dense>

namespace moraimpc {

// 5-State LTV Model: [dr, theta, kappa, theta_r, kappa_r]
static constexpr int kNx = 5;
static constexpr int kNu = 1; // input: kappa_dot

struct LTVMPCConfig {
    int N = 30;            // Horizon (15→30: 1.5초 예측, 커브 미리 대응)
    double Ts = 0.05;
    double L = 2.7;

    // Weights
    double w_dr    =  20.0;  // CTE (강화: 5→20, RECOV 후 큰 CTE 수정력 확보)
    double w_theta =  50.0;  // 헤딩 (방향 정렬 우선)
    double w_kappa =  10.0;  // (κ-κ_r)² 패널티 (ltv_cost.cpp 수정 적용)
    double w_u     = 2000.0; // kappa_dot 억제 (연속 포화 완전 차단)

    // Constraints
    double max_steer_deg = 35.0;
    double kappa_max =  std::tan(35.0 * M_PI / 180.0) / L;
    double kappa_min = -std::tan(35.0 * M_PI / 180.0) / L;
    double u_max =  0.10;
    double u_min = -0.10;

    double target_vel = 5.55; // 20 km/h (m/s)
};

struct MPCState {
    Eigen::VectorXd x; // [dr, theta, kappa, theta_r, kappa_r]
    MPCState() : x(kNx) { x.setZero(); }
};

struct MPCControl {
    double kappa_dot;
};

} // namespace moraimpc