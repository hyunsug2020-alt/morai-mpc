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
    double L = 2.7;        // Wheelbase

    // Weights
    double w_dr = 50.0;
    double w_theta = 30.0;
    double w_kappa = 10.0;
    double w_u = 1.0;      // Change rate penalty

    // Constraints
    double kappa_max = 0.4; // Max curvature (~25 deg steer)
    double kappa_min = -0.4;
    double u_max = 2.0;     // Max change rate
    double u_min = -2.0;

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
