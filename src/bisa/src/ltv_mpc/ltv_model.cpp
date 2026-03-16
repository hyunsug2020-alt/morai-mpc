#include "ltv_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unsupported/Eigen/MatrixFunctions>

namespace bisa {

LTVModel::LTVModel(const LTVMPCConfig& cfg) : cfg_(cfg) {}

void LTVModel::setConfig(const LTVMPCConfig& cfg) { cfg_ = cfg; }

double LTVModel::wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double LTVModel::quatToYaw(const geometry_msgs::msg::Quaternion& q) const {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.15) {
    return wrapAngle(q.z);
  }

  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return wrapAngle(std::atan2(siny_cosp, cosy_cosp));
}

double LTVModel::curvature3pt(double x1, double y1, double x2, double y2, double x3,
                              double y3) const {
  const double dx1 = x2 - x1;
  const double dy1 = y2 - y1;
  const double dx2 = x3 - x2;
  const double dy2 = y3 - y2;
  const double cross = dx1 * dy2 - dy1 * dx2;
  const double d1 = std::hypot(dx1, dy1);
  const double d2 = std::hypot(dx2, dy2);

  if (d1 < 1e-6 || d2 < 1e-6) return 0.0;
  // Signed curvature with consistent 1/m unit.
  // κ = 2 * cross / (d1 * d2 * (d1 + d2))
  return (2.0 * cross) / (d1 * d2 * (d1 + d2) + 1e-6);
}

int LTVModel::findClosestIndex(
    const geometry_msgs::msg::Pose& ego_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& path) const {
  if (path.empty()) return -1;

  const double x = ego_pose.position.x;
  const double y = ego_pose.position.y;
  double best = std::numeric_limits<double>::max();
  int best_idx = 0;

  // local_path is already a forward window; widen moderately to reduce
  // corner-limit saturation without over-searching far segments.
  const int search_limit = std::min(static_cast<int>(path.size()), 50);
  for (int i = 0; i < search_limit; ++i) {
    const double dx = path[i].pose.position.x - x;
    const double dy = path[i].pose.position.y - y;
    const double d2 = dx * dx + dy * dy;
    // Keep moderate forward bias (too low: backward jump risk, too high:
    // misses true closest near sharp corners).
    const double score = d2 + 0.012 * static_cast<double>(i);
    if (score < best) {
      best = score;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

double LTVModel::referenceYawAt(
    const std::vector<geometry_msgs::msg::PoseStamped>& path, int idx) const {
  const int n = static_cast<int>(path.size());
  if (n == 0) return 0.0;
  idx = std::max(0, std::min(idx, n - 1));
  int j = std::min(idx + 1, n - 1);
  if (j == idx && idx > 0) j = idx - 1;

  const double dx = path[j].pose.position.x - path[idx].pose.position.x;
  const double dy = path[j].pose.position.y - path[idx].pose.position.y;
  if (std::abs(dx) + std::abs(dy) < 1e-9) return 0.0;
  return wrapAngle(std::atan2(dy, dx));
}

void LTVModel::buildReferenceProfiles(
    const geometry_msgs::msg::Pose& ego_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& path, double current_kappa,
    Eigen::VectorXd& x0, Eigen::VectorXd& z_bar, Eigen::VectorXd& v_bar,
    std::vector<double>& theta_r_seq, std::vector<double>& kappa_r_seq) const {
  const int N = cfg_.N;
  x0 = Eigen::VectorXd::Zero(kLTVNx);
  z_bar = Eigen::VectorXd::Zero(N);
  v_bar = Eigen::VectorXd::Constant(N, cfg_.min_velocity);
  theta_r_seq.assign(N + 1, 0.0);
  kappa_r_seq.assign(N + 1, 0.0);

  if (path.size() < 3) return;

  const int closest = findClosestIndex(ego_pose, path);
  const int last_idx = static_cast<int>(path.size()) - 1;
  const int start_idx =
      std::min(last_idx, std::max(0, closest + std::max(0, cfg_.ref_preview_steps)));

  for (int k = 0; k <= N; ++k) {
    const int idx = std::min(start_idx + k, last_idx);
    const int i0 = std::max(0, idx - 1);
    const int i1 = idx;
    const int i2 = std::min(last_idx, idx + 1);

    theta_r_seq[k] = referenceYawAt(path, idx);
    kappa_r_seq[k] = curvature3pt(path[i0].pose.position.x, path[i0].pose.position.y,
                                  path[i1].pose.position.x, path[i1].pose.position.y,
                                  path[i2].pose.position.x, path[i2].pose.position.y);
  }

  // Smooth reference curvature to reduce discontinuity-driven feedforward spikes.
  const int kappa_smooth_window = 3;
  std::vector<double> kappa_smoothed = kappa_r_seq;
  for (int k = 1; k < N; ++k) {
    double sum = 0.0;
    int cnt = 0;
    for (int j = std::max(0, k - kappa_smooth_window);
         j <= std::min(N, k + kappa_smooth_window); ++j) {
      sum += kappa_r_seq[j];
      ++cnt;
    }
    kappa_smoothed[k] = (cnt > 0) ? (sum / static_cast<double>(cnt)) : kappa_r_seq[k];
  }

  for (int k = 0; k < N; ++k) {
    const double z_raw =
        (kappa_smoothed[k + 1] - kappa_smoothed[k]) / std::max(cfg_.Ts, 1e-6);
    const double z_limit = 8.0;
    z_bar(k) = std::clamp(z_raw, -z_limit, z_limit);

    const double curvature = std::abs(kappa_smoothed[k]);
    const double v = cfg_.max_velocity /
                     (1.0 + std::max(0.0, cfg_.curvature_speed_gain) * curvature);
    v_bar(k) = std::max(cfg_.min_velocity, std::min(v, cfg_.max_velocity));
  }

  const int idx0 = start_idx;
  const double x_ego = ego_pose.position.x;
  const double y_ego = ego_pose.position.y;
  const double x_ref = path[idx0].pose.position.x;
  const double y_ref = path[idx0].pose.position.y;
  const double theta = quatToYaw(ego_pose.orientation);
  const double theta_r = theta_r_seq[0];

  const double dx = x_ego - x_ref;
  const double dy = y_ego - y_ref;
  const double dr = -std::sin(theta_r) * dx + std::cos(theta_r) * dy;

  x0(0) = dr;
  x0(1) = theta;
  x0(2) = current_kappa;
  x0(3) = theta_r;
  x0(4) = kappa_r_seq[0];
}

void LTVModel::buildContinuousMatrices(double v_k, MatA& Ac, VecB& Bc,
                                       VecE& Ec) const {
  Ac.setZero();
  Ac(0, 1) = v_k;
  Ac(0, 3) = -v_k;
  Ac(1, 2) = v_k;
  Ac(3, 4) = v_k;

  Bc.setZero();
  Bc(2, 0) = 1.0;

  Ec.setZero();
  Ec(4, 0) = 1.0;
}

void LTVModel::discretizeExact(const MatA& Ac, const VecB& Bc, const VecE& Ec,
                               double Ts, MatA& Ad, VecB& Bd, VecE& Ed) const {
  const double ts = (std::isfinite(Ts) && Ts > 1e-6) ? Ts : 0.05;
  Eigen::Matrix<double, kLTVNx + 2, kLTVNx + 2> M;
  M.setZero();
  M.block<kLTVNx, kLTVNx>(0, 0) = Ac;
  M.block<kLTVNx, 1>(0, kLTVNx) = Bc;
  M.block<kLTVNx, 1>(0, kLTVNx + 1) = Ec;

  const auto expM = (M * ts).exp();
  Ad = expM.block<kLTVNx, kLTVNx>(0, 0);
  Bd = expM.block<kLTVNx, 1>(0, kLTVNx);
  Ed = expM.block<kLTVNx, 1>(0, kLTVNx + 1);
}

void LTVModel::buildOutputMatrix(MatC& C) const {
  C.setZero();
  C(0, 0) = 1.0;
  C(1, 0) = 1.0;
  C(1, 1) = cfg_.wheelbase * 0.5;
  C(1, 3) = -cfg_.wheelbase * 0.5;
  C(2, 0) = 1.0;
  C(2, 1) = cfg_.wheelbase;
  C(2, 3) = -cfg_.wheelbase;
  C(3, 2) = 1.0;
}

void LTVModel::buildBatchDynamics(const Eigen::VectorXd& v_bar, Eigen::MatrixXd& A_bar,
                                  Eigen::MatrixXd& B_bar, Eigen::MatrixXd& E_bar,
                                  Eigen::MatrixXd& C_bar) const {
  const int N = cfg_.N;
  A_bar = Eigen::MatrixXd::Zero(kLTVNx * N, kLTVNx);
  B_bar = Eigen::MatrixXd::Zero(kLTVNx * N, N);
  E_bar = Eigen::MatrixXd::Zero(kLTVNx * N, N);
  C_bar = Eigen::MatrixXd::Zero(kLTVNy * N, kLTVNx * N);

  std::vector<MatA> A_list(N);
  std::vector<VecB> B_list(N);
  std::vector<VecE> E_list(N);
  MatC Ck;
  buildOutputMatrix(Ck);

  for (int k = 0; k < N; ++k) {
    MatA Ac, Ad;
    VecB Bc, Bd;
    VecE Ec, Ed;
    const double v_k = (k < v_bar.size()) ? v_bar(k) : v_bar(v_bar.size() - 1);
    buildContinuousMatrices(v_k, Ac, Bc, Ec);
    discretizeExact(Ac, Bc, Ec, cfg_.Ts, Ad, Bd, Ed);
    A_list[k] = Ad;
    B_list[k] = Bd;
    E_list[k] = Ed;
  }

  for (int i = 0; i < N; ++i) {
    MatA prod = MatA::Identity();
    for (int j = 0; j <= i; ++j) {
      prod = A_list[j] * prod;
    }
    A_bar.block(i * kLTVNx, 0, kLTVNx, kLTVNx) = prod;
    C_bar.block(i * kLTVNy, i * kLTVNx, kLTVNy, kLTVNx) = Ck;
  }

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j <= i; ++j) {
      MatA prod = MatA::Identity();
      for (int k = j + 1; k <= i; ++k) {
        prod = A_list[k] * prod;
      }
      B_bar.block(i * kLTVNx, j, kLTVNx, 1) = prod * B_list[j];
      E_bar.block(i * kLTVNx, j, kLTVNx, 1) = prod * E_list[j];
    }
  }
}

}  // namespace bisa
