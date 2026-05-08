#include "moraimpc/ltv_cost.hpp"

namespace moraimpc {

LTVCost::LTVCost(const LTVMPCConfig& cfg) : cfg_(cfg) {}

void LTVCost::buildQPObjective(const Eigen::VectorXd& x0,
                                const Eigen::VectorXd& z_bar,
                                const Eigen::MatrixXd& A_bar,
                                const Eigen::MatrixXd& B_bar,
                                const Eigen::MatrixXd& E_bar,
                                const std::vector<double>& v_profile,
                                double max_kappa_ahead,
                                Eigen::SparseMatrix<double>& P,
                                Eigen::VectorXd& q) {
    int N = cfg_.N;

    // Adaptive w_theta: speed-dependent linear interpolation
    double v_avg = 0.0;
    for (int i = 0; i < N; ++i) v_avg += v_profile[i];
    v_avg /= N;
    double t_frac = std::clamp((v_avg - cfg_.w_theta_v_low) / (cfg_.w_theta_v_high - cfg_.w_theta_v_low + 1e-6), 0.0, 1.0);
    double w_theta_adaptive = cfg_.w_theta_low_speed + t_frac * (cfg_.w_theta_high_speed - cfg_.w_theta_low_speed);

    // Adaptive w_dr: boost in curves
    double w_dr_adaptive = cfg_.w_dr;
    if (max_kappa_ahead > cfg_.curve_kappa_thresh) {
        w_dr_adaptive *= cfg_.w_dr_curve_boost;
    }

    // Build Q_bar (State Weight Matrix)
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(kNx * N, kNx * N);
    for (int i = 0; i < N; ++i) {
        Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(kNx, kNx);
        Qi(0, 0) = w_dr_adaptive;
        Qi(1, 1) = w_theta_adaptive;
        Qi(1, 3) = -w_theta_adaptive;
        Qi(3, 1) = -w_theta_adaptive;
        Qi(3, 3) = w_theta_adaptive;
        Qi(2, 2) = cfg_.w_kappa;
        Qi(2, 4) = -cfg_.w_kappa;
        Qi(4, 2) = -cfg_.w_kappa;
        Qi(4, 4) = cfg_.w_kappa;
        Q_bar.block(i * kNx, i * kNx, kNx, kNx) = Qi;
    }

    // Build R_bar (Input Weight Matrix) - 속도에 따른 가중치 가변
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(N, N);
    for (int i = 0; i < N; ++i) {
        double v_i = v_profile[i];
        // 고속에서 w_u를 더 키워 진동 억제 (v=20km/h(5.5m/s) 기준 약 1.5배 증가)
        R_bar(i, i) = cfg_.w_u + cfg_.w_u_v_gain * v_i;
    }

    // x_free = A_bar*x0 + E_bar*z_bar
    Eigen::VectorXd x_free = A_bar * x0 + E_bar * z_bar;

    // QP objective: J = 0.5 * u^T * P * u + q^T * u
    // H = 2 * (B_bar^T * Q_bar * B_bar + R_bar)
    // f = 2 * B_bar^T * Q_bar * x_free
    Eigen::MatrixXd H = 2.0 * (B_bar.transpose() * Q_bar * B_bar + R_bar);
    Eigen::VectorXd f = 2.0 * B_bar.transpose() * Q_bar * x_free;

    // P must be upper triangular for OSQP
    Eigen::MatrixXd P_dense = H.triangularView<Eigen::Upper>();
    P = P_dense.sparseView();
    q = f;
}

} // namespace moraimpc
