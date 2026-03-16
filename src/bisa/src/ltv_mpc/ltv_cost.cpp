#include "ltv_cost.hpp"

#include <algorithm>

namespace bisa {

LTVCost::LTVCost(const LTVMPCConfig& cfg) : cfg_(cfg) {}

void LTVCost::setConfig(const LTVMPCConfig& cfg) { cfg_ = cfg; }

void LTVCost::buildStateInputWeights(Eigen::MatrixXd& Q_bar,
                                     Eigen::MatrixXd& R_bar) const {
  const int N = cfg_.N;
  Q_bar = Eigen::MatrixXd::Zero(kLTVNx * N, kLTVNx * N);
  R_bar = Eigen::MatrixXd::Zero(N, N);

  Eigen::Matrix<double, kLTVNx, kLTVNx> Qk;
  Qk.setZero();
  Qk(0, 0) = cfg_.w_d;
  Qk(1, 1) = cfg_.w_theta;
  Qk(1, 3) = -cfg_.w_theta;
  Qk(2, 2) = cfg_.w_kappa;
  Qk(3, 1) = -cfg_.w_theta;
  Qk(3, 3) = cfg_.w_theta;

  for (int k = 0; k < N; ++k) {
    double scale = 1.0;
    if (k == N - 1) scale = std::max(cfg_.w_terminal_scale, 1e-6);
    Q_bar.block(k * kLTVNx, k * kLTVNx, kLTVNx, kLTVNx) = scale * Qk;
  }

  const double wu = std::max(cfg_.w_u, 1e-9);
  R_bar.diagonal().setConstant(wu);
}

void LTVCost::buildQPObjective(const Eigen::VectorXd& x0, const Eigen::VectorXd& z_bar,
                               const Eigen::MatrixXd& A_bar,
                               const Eigen::MatrixXd& B_bar,
                               const Eigen::MatrixXd& E_bar,
                               const Eigen::MatrixXd& Q_bar,
                               const Eigen::MatrixXd& R_bar,
                               Eigen::SparseMatrix<double>& P,
                               Eigen::VectorXd& q) const {
  const Eigen::VectorXd x_free = A_bar * x0 + E_bar * z_bar;
  Eigen::MatrixXd H = B_bar.transpose() * Q_bar * B_bar + R_bar;
  H = 0.5 * (H + H.transpose());
  H += 1e-8 * Eigen::MatrixXd::Identity(H.rows(), H.cols());
  const Eigen::VectorXd f = B_bar.transpose() * Q_bar * x_free;

  const Eigen::MatrixXd P_dense = 2.0 * H;
  const Eigen::MatrixXd P_upper = P_dense.triangularView<Eigen::Upper>();
  P = P_upper.sparseView();
  P.makeCompressed();
  q = 2.0 * f;
}

}  // namespace bisa
