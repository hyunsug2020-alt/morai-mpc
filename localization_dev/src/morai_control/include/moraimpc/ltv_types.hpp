#pragma once
#include <vector>
#include <Eigen/Dense>

namespace moraimpc {

// 5-State LTV Model: [dr, theta, kappa, theta_r, kappa_r]
static constexpr int kNx = 5;
static constexpr int kNu = 1; // input: kappa_dot

struct LTVMPCConfig {
    int N = 20;            // Horizon
    double Ts = 0.05;      // Sampling time (20Hz)
    double L = 3.0;        // 2023 Hyundai IONIQ 5 wheelbase

    // Weights
    double w_dr = 1.0;
    double w_theta = 1.0;
    double w_kappa = 1.0;
    double w_u = 50.0;      // Change rate penalty

    // Constraints
    double kappa_max = 0.3; // Max curvature (~25 deg steer)
    double kappa_min = -0.3;
    double u_max = 1.0;     // Max change rate
    double u_min = -1.0;

    double target_vel = 5.55; // 20 km/h
};

struct MPCState {
    Eigen::VectorXd x; // [dr, theta, kappa, theta_r, kappa_r]
    MPCState() : x(kNx) { x.setZero(); }
};

struct MPCControl {
    double kappa_dot;
};

} // namespace moraimpc
