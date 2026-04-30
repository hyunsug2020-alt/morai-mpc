#include "moraimpc/ltv_model.hpp"
#include <unsupported/Eigen/MatrixFunctions>

namespace moraimpc {

LTVModel::LTVModel(const LTVMPCConfig& cfg) : cfg_(cfg) {}

void LTVModel::discretize(double v, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& E) {
    // ẋ = Ac*x + Bc*u + Ec*z
    // x = [dr, theta, kappa, theta_r, kappa_r]^T
    // u = kappa_dot, z = kappa_r_dot

    Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(kNx, kNx);
    Ac(0, 1) = v;      // ḋr = v*theta
    Ac(0, 3) = -v;     // -v*theta_r
    Ac(1, 2) = v;      // θ̇ = v*kappa
    Ac(3, 4) = v;      // θ̇r = v*kappa_r
    
    Eigen::VectorXd Bc = Eigen::VectorXd::Zero(kNx);
    Bc(2) = 1.0;  // kappa_dot = u (입력 자체가 곡률 변화율)
    
    Eigen::VectorXd Ec = Eigen::VectorXd::Zero(kNx);
    Ec(4) = 1.0;       // κ̇r = z

    // Exact Discretization using Augmented Matrix Exponential
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(kNx + 2, kNx + 2);
    M.block(0, 0, kNx, kNx) = Ac;
    M.block(0, kNx, kNx, 1) = Bc;
    M.block(0, kNx + 1, kNx, 1) = Ec;

    Eigen::MatrixXd expM = (M * cfg_.Ts).exp();
    A = expM.block(0, 0, kNx, kNx);
    B = expM.block(0, kNx, kNx, 1);
    E = expM.block(0, kNx + 1, kNx, 1);
}

void LTVModel::buildBatchMatrices(const std::vector<double>& v_profile, 
                                  Eigen::MatrixXd& A_bar, 
                                  Eigen::MatrixXd& B_bar, 
                                  Eigen::MatrixXd& E_bar) {
    int N = cfg_.N;
    A_bar.resize(kNx * N, kNx);
    B_bar.resize(kNx * N, N);
    E_bar.resize(kNx * N, N);
    A_bar.setZero();
    B_bar.setZero();
    E_bar.setZero();

    std::vector<Eigen::MatrixXd> A_list(N), B_list(N), E_list(N);
    for (int i = 0; i < N; ++i) {
        discretize(v_profile[i], A_list[i], B_list[i], E_list[i]);
    }

    // x(k) = A_bar[k]*x0 + B_bar[k,:]*U + E_bar[k,:]*Z
    for (int i = 0; i < N; ++i) {
        Eigen::MatrixXd prod = Eigen::MatrixXd::Identity(kNx, kNx);
        for (int j = 0; j <= i; ++j) {
            prod = A_list[j] * prod;
        }
        A_bar.block(i * kNx, 0, kNx, kNx) = prod;

        for (int j = 0; j <= i; ++j) {
            Eigen::MatrixXd b_prod = Eigen::MatrixXd::Identity(kNx, kNx);
            for (int k = j + 1; k <= i; ++k) {
                b_prod = A_list[k] * b_prod;
            }
            B_bar.block(i * kNx, j, kNx, 1) = b_prod * B_list[j];
            E_bar.block(i * kNx, j, kNx, 1) = b_prod * E_list[j];
        }
    }
}

} // namespace moraimpc
