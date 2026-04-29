#pragma once
#include <cmath>
#include <vector>
#include <Eigen/Dense>

namespace moraimpc {

// 5-State LTV Model: [dr, theta, kappa, theta_r, kappa_r]
static constexpr int kNx = 5;
static constexpr int kNu = 1; // input: kappa_dot

struct LTVMPCConfig {
    int N = 20;            // Horizon (30→20: Stanley가 방향을 올바르게 잡아줘 긴 horizon 불필요)
    double Ts = 0.05;      // Sampling time (20Hz)
    double L = 2.7;        // Wheelbase

    // Weights — Stanley 제거 후 CTE 직접 제어
    // w_dr >> w_theta: CTE 교정을 최우선, 헤딩은 보조만
    double w_dr    = 100.0;  // CTE 가중치 (경로 추종 주 목표)
    double w_theta =   5.0;  // 헤딩 가중치 최소화 (MPC가 헤딩 집착 → CTE 발산 방지)
    double w_kappa =   5.0;  // kappa 억제 완화
    double w_u     =  20.0;  // 빠른 응답 허용

    // Constraints
    double max_steer_deg = 35.0;
    double kappa_max =  std::tan(35.0 * M_PI / 180.0) / L;  // ≈ 0.259
    double kappa_min = -std::tan(35.0 * M_PI / 180.0) / L;
    double u_max =  0.5;   // kappa 변화율 제한 유지
    double u_min = -0.5;

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