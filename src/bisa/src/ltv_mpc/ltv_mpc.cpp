#include "ltv_mpc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace bisa {

LTVMPC::LTVMPC(const LTVMPCConfig& cfg)
    : cfg_(cfg), model_(cfg), cost_(cfg), solver_(cfg), kappa_state_(0.0) {}

void LTVMPC::setConfig(const LTVMPCConfig& cfg) {
  cfg_ = cfg;
  model_.setConfig(cfg);
  cost_.setConfig(cfg);
  solver_.setConfig(cfg);
}

void LTVMPC::reset() {
  kappa_state_ = 0.0;
  kappa_initialized_ = false;
}

void LTVMPC::buildInputConstraints(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l,
                                   Eigen::VectorXd& u, int nvar) const {
  const int N = cfg_.N;
  A.resize(N, nvar);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(N);
  for (int i = 0; i < N; ++i) {
    triplets.emplace_back(i, i, 1.0);
  }
  A.setFromTriplets(triplets.begin(), triplets.end());
  l = Eigen::VectorXd::Constant(N, cfg_.u_min);
  u = Eigen::VectorXd::Constant(N, cfg_.u_max);
}

// Gutjahr 2017 Section III: hard constraint on predicted curvature state κ(k).
// κ(k) = A_bar[k*Nx+2, :]*x0 + B_bar[k*Nx+2, :]*u + E_bar[k*Nx+2, :]*z
// => (kappa_min - kappa_free(k)) <= B_kappa(k,:)*u <= (kappa_max - kappa_free(k))
void LTVMPC::buildKappaConstraints(const Eigen::VectorXd& x0,
                                   const Eigen::VectorXd& z_bar,
                                   const Eigen::MatrixXd& A_bar,
                                   const Eigen::MatrixXd& B_bar,
                                   const Eigen::MatrixXd& E_bar,
                                   Eigen::SparseMatrix<double>& A,
                                   Eigen::VectorXd& l,
                                   Eigen::VectorXd& u,
                                   int nvar) const {
  const int N = cfg_.N;
  A.resize(N, nvar);
  l.resize(N);
  u.resize(N);

  const Eigen::VectorXd x_free = A_bar * x0 + E_bar * z_bar;

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(N * N / 2);

  for (int k = 0; k < N; ++k) {
    const int kappa_row = k * kLTVNx + 2;  // κ is state index 2 in x=[dr,θ,κ,θr,κr]
    const double kappa_free = x_free(kappa_row);

    // B_bar is lower-triangular in block structure: only columns j <= k are non-zero.
    for (int j = 0; j <= k; ++j) {
      const double v = B_bar(kappa_row, j);
      if (std::abs(v) > 1e-12) triplets.emplace_back(k, j, v);
    }

    l(k) = cfg_.kappa_min - kappa_free;
    u(k) = cfg_.kappa_max - kappa_free;
  }

  A.setFromTriplets(triplets.begin(), triplets.end());
}

void LTVMPC::buildLateralSlackConstraints(
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& z_bar,
    const Eigen::MatrixXd& A_bar,
    const Eigen::MatrixXd& B_bar,
    const Eigen::MatrixXd& E_bar,
    const Eigen::MatrixXd& C_bar,
    Eigen::SparseMatrix<double>& A,
    Eigen::VectorXd& l,
    Eigen::VectorXd& u,
    int nvar) const {
  const int N = cfg_.N;
  const int n_lat_per_step = 3;
  const int n_lat = n_lat_per_step * N;
  const int rows = 2 * n_lat + 2;  // upper/lower + epsilon_u/l non-negative
  constexpr double kInf = 1e20;

  A.resize(rows, nvar);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(2 * n_lat * (N + 1) + 2));

  Eigen::MatrixXd C_d = Eigen::MatrixXd::Zero(n_lat, kLTVNx * N);
  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < n_lat_per_step; ++i) {
      C_d.row(k * n_lat_per_step + i) = C_bar.row(k * kLTVNy + i);
    }
  }

  const Eigen::VectorXd x_free = A_bar * x0 + E_bar * z_bar;
  const Eigen::VectorXd y_free = C_d * x_free;
  const Eigen::MatrixXd Cy = C_d * B_bar;
  const double d_bound = std::max(0.01, cfg_.lateral_bound);

  l = Eigen::VectorXd::Constant(rows, -kInf);
  u = Eigen::VectorXd::Constant(rows, kInf);

  // Upper soft: Cy*u - eps_u <= d_bound - y_free
  for (int r = 0; r < n_lat; ++r) {
    for (int c = 0; c < N; ++c) {
      const double v = Cy(r, c);
      if (std::abs(v) > 1e-12) triplets.emplace_back(r, c, v);
    }
    triplets.emplace_back(r, N, -1.0);  // -eps_u
    u(r) = d_bound - y_free(r);
  }

  // Lower soft: -Cy*u - eps_l <= d_bound + y_free
  for (int r = 0; r < n_lat; ++r) {
    const int rr = n_lat + r;
    for (int c = 0; c < N; ++c) {
      const double v = -Cy(r, c);
      if (std::abs(v) > 1e-12) triplets.emplace_back(rr, c, v);
    }
    triplets.emplace_back(rr, N + 1, -1.0);  // -eps_l
    u(rr) = d_bound + y_free(r);
  }

  // eps_u >= 0, eps_l >= 0
  const int reu = 2 * n_lat;
  const int rel = reu + 1;
  triplets.emplace_back(reu, N, 1.0);
  triplets.emplace_back(rel, N + 1, 1.0);
  l(reu) = 0.0;
  l(rel) = 0.0;

  A.setFromTriplets(triplets.begin(), triplets.end());
}

MPCCommand LTVMPC::computeControl(
    const geometry_msgs::msg::Pose& ego_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& local_path) {
  MPCCommand out;
  const auto t0 = std::chrono::high_resolution_clock::now();

  if (local_path.size() < 5) return out;

  Eigen::VectorXd x0, z_bar, v_bar;
  std::vector<double> theta_r_seq;
  std::vector<double> kappa_r_seq;
  model_.buildReferenceProfiles(ego_pose, local_path, kappa_state_, x0, z_bar, v_bar,
                                theta_r_seq, kappa_r_seq);

  if (x0.size() != kLTVNx || z_bar.size() != cfg_.N || v_bar.size() != cfg_.N) {
    return out;
  }

  // Initialize curvature state from path reference once at startup.
  if (!kappa_initialized_ && !kappa_r_seq.empty()) {
    kappa_state_ = std::clamp(kappa_r_seq[0], cfg_.kappa_min, cfg_.kappa_max);
    kappa_initialized_ = true;
  }

  Eigen::MatrixXd A_bar, B_bar, E_bar, C_bar;
  model_.buildBatchDynamics(v_bar, A_bar, B_bar, E_bar, C_bar);

  Eigen::MatrixXd Q_bar, R_bar;
  cost_.buildStateInputWeights(Q_bar, R_bar);

  Eigen::SparseMatrix<double> P_base;
  Eigen::VectorXd q_base;
  cost_.buildQPObjective(x0, z_bar, A_bar, B_bar, E_bar, Q_bar, R_bar, P_base, q_base);

  const bool use_soft_lateral = (cfg_.lateral_bound > 0.0);
  const int nvar = cfg_.N + (use_soft_lateral ? 2 : 0);

  Eigen::SparseMatrix<double> P;
  Eigen::VectorXd q;
  if (!use_soft_lateral) {
    P = P_base;
    q = q_base;
  } else {
    P.resize(nvar, nvar);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(P_base.nonZeros() + 2));
    for (int k = 0; k < P_base.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(P_base, k); it; ++it) {
        triplets.emplace_back(it.row(), it.col(), it.value());
      }
    }
    triplets.emplace_back(cfg_.N, cfg_.N, 2.0 * cfg_.w_lateral_slack_quad);
    triplets.emplace_back(cfg_.N + 1, cfg_.N + 1, 2.0 * cfg_.w_lateral_slack_quad);
    P.setFromTriplets(triplets.begin(), triplets.end());

    q = Eigen::VectorXd::Zero(nvar);
    q.head(cfg_.N) = q_base;
    q(cfg_.N) = cfg_.w_lateral_slack_lin;
    q(cfg_.N + 1) = cfg_.w_lateral_slack_lin;
  }

  Eigen::SparseMatrix<double> A_in, A_kap, A_lat;
  Eigen::VectorXd l_in, u_in, l_kap, u_kap, l_lat, u_lat;
  buildInputConstraints(A_in, l_in, u_in, nvar);
  // Gutjahr 2017 Section III: κ hard constraint inside QP.
  // Without this, MPC plans beyond κ limits and post-hoc clipping causes overshoot.
  buildKappaConstraints(x0, z_bar, A_bar, B_bar, E_bar, A_kap, l_kap, u_kap, nvar);

  // Helper: vertically stack two sparse matrices with given row offsets.
  auto append_sparse = [&](std::vector<Eigen::Triplet<double>>& triples,
                           const Eigen::SparseMatrix<double>& mat, int row_offset) {
    for (int k = 0; k < mat.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(mat, k); it; ++it)
        triples.emplace_back(it.row() + row_offset, it.col(), it.value());
  };

  Eigen::SparseMatrix<double> A_cons;
  Eigen::VectorXd l_cons, u_cons;
  if (!use_soft_lateral) {
    const int rows_in  = A_in.rows();
    const int rows_kap = A_kap.rows();
    A_cons.resize(rows_in + rows_kap, nvar);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(A_in.nonZeros() + A_kap.nonZeros()));
    append_sparse(triplets, A_in,  0);
    append_sparse(triplets, A_kap, rows_in);
    A_cons.setFromTriplets(triplets.begin(), triplets.end());
    l_cons.resize(rows_in + rows_kap);
    u_cons.resize(rows_in + rows_kap);
    l_cons << l_in, l_kap;
    u_cons << u_in, u_kap;
  } else {
    buildLateralSlackConstraints(x0, z_bar, A_bar, B_bar, E_bar, C_bar, A_lat, l_lat, u_lat, nvar);
    const int rows_in  = A_in.rows();
    const int rows_kap = A_kap.rows();
    const int rows_lat = A_lat.rows();
    A_cons.resize(rows_in + rows_kap + rows_lat, nvar);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(A_in.nonZeros() + A_kap.nonZeros() + A_lat.nonZeros()));
    append_sparse(triplets, A_in,  0);
    append_sparse(triplets, A_kap, rows_in);
    append_sparse(triplets, A_lat, rows_in + rows_kap);
    A_cons.setFromTriplets(triplets.begin(), triplets.end());
    l_cons.resize(rows_in + rows_kap + rows_lat);
    u_cons.resize(rows_in + rows_kap + rows_lat);
    l_cons << l_in, l_kap, l_lat;
    u_cons << u_in, u_kap, u_lat;
  }

  const auto t_model_end = std::chrono::high_resolution_clock::now();
  out.model_time_us =
      std::chrono::duration_cast<std::chrono::microseconds>(t_model_end - t0).count();

  Eigen::VectorXd u_opt;
  const auto t_solver_start = std::chrono::high_resolution_clock::now();
  const bool solved = solver_.solve(P, q, A_cons, l_cons, u_cons, u_opt);
  const auto t_solver_end = std::chrono::high_resolution_clock::now();
  out.solver_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
      t_solver_end - t_solver_start).count();
  if (!solved || u_opt.size() < cfg_.N) return out;

  const double u0 = u_opt(0);
  kappa_state_ += cfg_.Ts * u0;

  // Always-on exponential blend to reduce corner-exit heading lag.
  if (!kappa_r_seq.empty()) {
    const double kappa_ref =
        std::clamp(kappa_r_seq[0], cfg_.kappa_min, cfg_.kappa_max);
    const double alpha = std::clamp(cfg_.kappa_blend_alpha, 0.0, 1.0);
    kappa_state_ = (1.0 - alpha) * kappa_state_ + alpha * kappa_ref;
  }

  kappa_state_ = std::max(cfg_.kappa_min, std::min(cfg_.kappa_max, kappa_state_));

  const double v_cmd =
      std::max(cfg_.min_velocity, std::min(cfg_.max_velocity, v_bar(0)));
  const double omega_cmd = v_cmd * kappa_state_;

  out.solved = true;
  out.kappa_cmd = kappa_state_;
  out.v_cmd = v_cmd;
  out.omega_cmd = omega_cmd;
  out.u_seq.resize(cfg_.N);
  for (int i = 0; i < cfg_.N; ++i) out.u_seq[i] = u_opt(i);

  out.predicted_xy.reserve(cfg_.N);
  double px = ego_pose.position.x;
  double py = ego_pose.position.y;
  const double qnorm =
      std::sqrt(ego_pose.orientation.x * ego_pose.orientation.x +
                ego_pose.orientation.y * ego_pose.orientation.y +
                ego_pose.orientation.z * ego_pose.orientation.z +
                ego_pose.orientation.w * ego_pose.orientation.w);
  double yaw = 0.0;
  if (std::isfinite(qnorm) && qnorm > 1e-6 &&
      std::abs(qnorm - 1.0) <= 0.15) {
    const double x = ego_pose.orientation.x / qnorm;
    const double y = ego_pose.orientation.y / qnorm;
    const double z = ego_pose.orientation.z / qnorm;
    const double w = ego_pose.orientation.w / qnorm;
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  } else {
    yaw = ego_pose.orientation.z;
  }

  double kappa_pred = kappa_state_;
  for (int k = 0; k < cfg_.N; ++k) {
    const double vk = (k < v_bar.size()) ? v_bar(k) : v_bar(v_bar.size() - 1);
    const double uk = (k < u_opt.size()) ? u_opt(k) : 0.0;
    kappa_pred += cfg_.Ts * uk;
    kappa_pred = std::max(cfg_.kappa_min, std::min(cfg_.kappa_max, kappa_pred));
    const double omega = vk * kappa_pred;
    px += vk * std::cos(yaw) * cfg_.Ts;
    py += vk * std::sin(yaw) * cfg_.Ts;
    yaw += omega * cfg_.Ts;
    out.predicted_xy.push_back({px, py, 0.0});
  }

  return out;
}

}  // namespace bisa
