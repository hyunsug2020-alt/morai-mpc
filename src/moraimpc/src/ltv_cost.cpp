#include "moraimpc/ltv_cost.hpp"

namespace moraimpc {

LTVCost::LTVCost(const LTVMPCConfig& cfg) : cfg_(cfg) {}

void LTVCost::buildQPObjective(const Eigen::VectorXd& x0, 
                                const Eigen::VectorXd& z_bar, 
                                const Eigen::MatrixXd& A_bar, 
                                const Eigen::MatrixXd& B_bar, 
                                const Eigen::MatrixXd& E_bar,
                                Eigen::SparseMatrix<double>& P, 
                                Eigen::VectorXd& q) {
    int N = cfg_.N;

    // Build Q_bar (State Weight Matrix)
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(kNx * N, kNx * N);
    for (int i = 0; i < N; ++i) {
        Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(kNx, kNx);
        Qi(0, 0) = cfg_.w_dr;
        Qi(1, 1) = cfg_.w_theta;
        Qi(1, 3) = -cfg_.w_theta;
        Qi(3, 1) = -cfg_.w_theta;
        Qi(3, 3) = cfg_.w_theta;
        // (κ - κ_r)² 패널티: κ²만 쓰면 κ=0 선호 → 커브에서 직선 주행 → 급격한 오버슈트
        Qi(2, 2) = cfg_.w_kappa;
        Qi(2, 4) = -cfg_.w_kappa;
        Qi(4, 2) = -cfg_.w_kappa;
        Qi(4, 4) = cfg_.w_kappa;
        Q_bar.block(i * kNx, i * kNx, kNx, kNx) = Qi;
    }

    // Build R_bar (Input Weight Matrix)
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Identity(N, N) * cfg_.w_u;

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
