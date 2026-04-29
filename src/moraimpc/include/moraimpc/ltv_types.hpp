#pragma once
#include <cmath>
#include <vector>
#include <Eigen/Dense>

namespace moraimpc {

// 5-State LTV Model: [dr, theta, kappa, theta_r, kappa_r]
static constexpr int kNx = 5;
static constexpr int kNu = 1; // input: kappa_dot

struct LTVMPCConfig {
    int N = 20;            // Horizon
    double Ts = 0.05;      // Sampling time (20Hz)
    double L = 2.7;        // Wheelbase

    // Weights
    // ── 수정 이유 ────────────────────────────────────────
    // w_dr=1, w_theta=1은 경로 추종력이 너무 약해 직선조차 못 따라감.
    // bisa ROS2 버전 기준(w_d=130, w_theta=165) 참고하여 적정값으로 상향.
    double w_dr    = 120.0;  // 횡방향 오차 가중치 (1→120)
    double w_theta =  80.0;  // 헤딩 오차 가중치  (1→80)
    double w_kappa =   5.0;  // 곡률 가중치       (1→5)
    double w_u     =  20.0;  // 입력 변화율 패널티 (50→20, 너무 높으면 반응성 저하)

    // Constraints
    // max_steer_deg=35° 기준으로 kappa_max 통일: tan(35°)/L
    double max_steer_deg = 35.0;
    double kappa_max =  std::tan(35.0 * M_PI / 180.0) / L;  // ≈ 0.259
    double kappa_min = -std::tan(35.0 * M_PI / 180.0) / L;
    double u_max =  0.8;   // kappa 변화율 제한 (2.0→0.8): 3스텝 폭주 방지
    double u_min = -0.8;

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