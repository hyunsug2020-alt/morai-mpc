#pragma once
#include <cmath>
#include <vector>
#include <Eigen/Dense>

namespace moraimpc {

// 5-State LTV Model: [dr, theta, kappa, theta_r, kappa_r]
static constexpr int kNx = 5;
static constexpr int kNu = 1; // input: kappa_dot

struct LTVMPCConfig {
    int N = 30;            // Horizon (1.5초 예측)
    double Ts = 0.05;
    double L = 2.7;
    double kappa_gain = 6.0; // 실제 차량 곡률이 Kinematic 모델보다 6배 큰 점 반영

    // Weights
    double w_dr    =   5.0;  // CTE 가중치 (0.1m 오차 시 0.05 패널티)
    double w_theta =  40.0;  // 헤딩 가중치
    double w_kappa =  15.0;  // 곡률 추종 가중치
    double w_u     = 80000.0; // kappa_dot 억제 (진동 방지의 핵심)

    // Adaptive weights (속도에 따른 가중치 증가분)
    double w_u_v_gain = 10000.0; // v=10m/s 시 w_u에 100000 추가

    // Constraints
    // max_steer_rate(18 deg/s) = 0.314 rad/s
    // kappa_dot_max = 0.314 / L / kappa_gain = 0.314 / 2.7 / 6.0 = 0.019
    double max_steer_deg = 35.0;
    double kappa_max =  std::tan(35.0 * M_PI / 180.0) / L;
    double kappa_min = -std::tan(35.0 * M_PI / 180.0) / L;
    double u_max =  0.018; // 0.02 -> 0.018 (더욱 정교한 제한)
    double u_min = -0.018; 

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
