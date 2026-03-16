#include "rti_nmpc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include <osqp/osqp.h>

namespace bisa {

namespace {

// 각도 정규화 [-π, π]
double wrapAngle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// Pose에서 yaw 추출 (쿼터니언 또는 z 필드 fallback)
double poseYaw(const geometry_msgs::msg::Pose& p) {
  const auto& q = p.orientation;
  const double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (std::isfinite(n) && n > 1e-6 && std::abs(n - 1.0) <= 0.05) {
    const double xn = q.x/n, yn = q.y/n, zn = q.z/n, wn = q.w/n;
    return wrapAngle(std::atan2(2.0*(wn*zn + xn*yn), 1.0 - 2.0*(yn*yn + zn*zn)));
  }
  return wrapAngle(q.z);
}

double poseYawStamped(const geometry_msgs::msg::PoseStamped& ps) {
  return poseYaw(ps.pose);
}

}  // namespace

// ============================================================
// 생성자 / 설정
// ============================================================
RTINMPCController::RTINMPCController(const RTINMPCConfig& cfg)
    : cfg_(cfg) {
  reset();
}

void RTINMPCController::setConfig(const RTINMPCConfig& cfg) {
  cfg_ = cfg;
}

void RTINMPCController::reset() {
  kappa_state_     = 0.0;
  v_state_         = 0.0;
  initialized_     = false;
  last_closest_idx_ = 0;
  u_warm_.assign(cfg_.N, Eigen::VectorXd::Zero(kRTINu));
}

// ============================================================
// 운동학 모델: f(x, u)
// x = [px, py, psi, v, kappa]
// u = [av, a_kappa]
// ============================================================
Eigen::VectorXd RTINMPCController::dynamics(const Eigen::VectorXd& x,
                                             const Eigen::VectorXd& u) const {
  Eigen::VectorXd xdot(kRTINx);
  const double psi   = x(2);
  const double v     = x(3);
  const double kappa = x(4);
  const double av    = u(0);
  const double akap  = u(1);

  xdot(0) = v * std::cos(psi);   // ẋ  = v·cos(ψ)
  xdot(1) = v * std::sin(psi);   // ẏ  = v·sin(ψ)
  xdot(2) = v * kappa;            // ψ̇  = v·κ
  xdot(3) = av;                   // v̇  = av
  xdot(4) = akap;                 // κ̇  = a_κ
  return xdot;
}

// ============================================================
// ∂f/∂x 야코비안
// ============================================================
Eigen::MatrixXd RTINMPCController::jacobianFx(const Eigen::VectorXd& x,
                                               const Eigen::VectorXd& /*u*/) const {
  Eigen::MatrixXd Jx = Eigen::MatrixXd::Zero(kRTINx, kRTINx);
  const double psi   = x(2);
  const double v     = x(3);
  const double kappa = x(4);

  //       px  py  psi            v            kappa
  Jx(0, 2) = -v * std::sin(psi);  // ∂(v·cosψ)/∂ψ
  Jx(0, 3) =  std::cos(psi);      // ∂(v·cosψ)/∂v
  Jx(1, 2) =  v * std::cos(psi);  // ∂(v·sinψ)/∂ψ
  Jx(1, 3) =  std::sin(psi);      // ∂(v·sinψ)/∂v
  Jx(2, 3) =  kappa;              // ∂(v·κ)/∂v
  Jx(2, 4) =  v;                  // ∂(v·κ)/∂κ
  // v̇=av, κ̇=a_κ → 상태에 대한 편미분 없음
  return Jx;
}

// ============================================================
// ∂f/∂u 야코비안
// ============================================================
Eigen::MatrixXd RTINMPCController::jacobianFu(const Eigen::VectorXd& /*x*/,
                                               const Eigen::VectorXd& /*u*/) const {
  Eigen::MatrixXd Ju = Eigen::MatrixXd::Zero(kRTINx, kRTINu);
  Ju(3, 0) = 1.0;  // ∂(v̇)/∂av
  Ju(4, 1) = 1.0;  // ∂(κ̇)/∂a_κ
  return Ju;
}

// ============================================================
// 암시적 오일러: x(k+1) = x(k) + Ts·f(x(k+1), u(k))
// Newton 반복으로 풀이
// ============================================================
Eigen::VectorXd RTINMPCController::implicitEulerStep(const Eigen::VectorXd& x_k,
                                                      const Eigen::VectorXd& u_k) const {
  const double Ts = cfg_.Ts;
  Eigen::VectorXd x_next = x_k;  // 초기 추정값 = 현재 상태

  for (int iter = 0; iter < cfg_.newton_max_iter; ++iter) {
    // 잔차: r = x_next - x_k - Ts·f(x_next, u_k)
    Eigen::VectorXd r = x_next - x_k - Ts * dynamics(x_next, u_k);

    // 야코비안: ∂r/∂x_next = I - Ts·∂f/∂x
    Eigen::MatrixXd Jr = Eigen::MatrixXd::Identity(kRTINx, kRTINx)
                         - Ts * jacobianFx(x_next, u_k);

    // Newton 업데이트: x_next ← x_next - Jr^{-1} · r
    x_next -= Jr.ldlt().solve(r);

    // 수렴 확인
    if (r.norm() < 1e-8) break;
  }

  // 각도 정규화
  x_next(2) = wrapAngle(x_next(2));
  // 곡률 클리핑
  x_next(4) = std::clamp(x_next(4), cfg_.kappa_min, cfg_.kappa_max);
  // 속도 클리핑
  x_next(3) = std::clamp(x_next(3), cfg_.min_velocity, cfg_.max_velocity);

  return x_next;
}

// ============================================================
// 이산화 + 선형화
// A_d ≈ (I - Ts·Fx)^{-1},  B_d ≈ A_d·Ts·Fu
// d_k = x_next - A_d·x_k - B_d·u_k  (선형화 오차 보정)
// ============================================================
void RTINMPCController::discretizeLinearize(const Eigen::VectorXd& x_k,
                                             const Eigen::VectorXd& u_k,
                                             Eigen::MatrixXd& A_d,
                                             Eigen::MatrixXd& B_d,
                                             Eigen::VectorXd& d_k) const {
  const double Ts = cfg_.Ts;
  const Eigen::MatrixXd Fx = jacobianFx(x_k, u_k);
  const Eigen::MatrixXd Fu = jacobianFu(x_k, u_k);

  // 암시적 오일러 선형화: (I - Ts·Fx)·x_next = x_k + Ts·Fu·u_k
  const Eigen::MatrixXd M = Eigen::MatrixXd::Identity(kRTINx, kRTINx) - Ts * Fx;
  A_d = M.inverse();
  B_d = A_d * Ts * Fu;

  // 비선형 해
  const Eigen::VectorXd x_next_nl = implicitEulerStep(x_k, u_k);

  // 선형화 오차 보정 항
  d_k = x_next_nl - A_d * x_k - B_d * u_k;
}

// ============================================================
// 상태 벡터 구성
// ============================================================
Eigen::VectorXd RTINMPCController::buildStateVector(
    const geometry_msgs::msg::Pose& pose, double v) const {
  Eigen::VectorXd x(kRTINx);
  x(0) = pose.position.x;
  x(1) = pose.position.y;
  x(2) = poseYaw(pose);
  x(3) = v;
  x(4) = kappa_state_;
  return x;
}

// ============================================================
// 가장 가까운 경로점 탐색
// ============================================================
int RTINMPCController::findClosestWaypoint(
    const Eigen::VectorXd& x0,
    const std::vector<geometry_msgs::msg::PoseStamped>& path,
    int search_start) const {
  const int n = static_cast<int>(path.size());
  if (n == 0) return 0;

  const int start = std::max(0, search_start);
  const int end   = std::min(n, start + 80);  // 80점 탐색 윈도우

  int best_idx = start;
  double best_dist = std::numeric_limits<double>::max();
  for (int i = start; i < end; ++i) {
    const double dx = path[i].pose.position.x - x0(0);
    const double dy = path[i].pose.position.y - x0(1);
    const double d  = dx*dx + dy*dy;
    if (d < best_dist) {
      best_dist = d;
      best_idx  = i;
    }
  }
  return best_idx;
}

// ============================================================
// 참조 상태 시퀀스 구성 (경로 → x_ref 시퀀스)
// ============================================================
bool RTINMPCController::buildReferenceSequence(
    const Eigen::VectorXd& x0,
    const std::vector<geometry_msgs::msg::PoseStamped>& path,
    std::vector<Eigen::VectorXd>& x_ref,
    std::vector<double>& v_ref) const {
  const int n = static_cast<int>(path.size());
  if (n < 2) return false;

  // 가장 가까운 점 탐색
  const int closest = findClosestWaypoint(x0, path, last_closest_idx_);

  x_ref.resize(cfg_.N + 1);
  v_ref.resize(cfg_.N + 1);

  for (int k = 0; k <= cfg_.N; ++k) {
    const int idx = std::min(closest + k, n - 1);
    Eigen::VectorXd xr(kRTINx);
    xr(0) = path[idx].pose.position.x;
    xr(1) = path[idx].pose.position.y;
    xr(2) = poseYawStamped(path[idx]);
    xr(3) = cfg_.parking_velocity;
    xr(4) = 0.0;  // 참조 곡률 (필요시 경로에서 계산)

    // 경로 방향에서 곡률 계산
    if (idx > 0 && idx < n - 1) {
      const double dx1 = path[idx].pose.position.x - path[idx-1].pose.position.x;
      const double dy1 = path[idx].pose.position.y - path[idx-1].pose.position.y;
      const double dx2 = path[idx+1].pose.position.x - path[idx].pose.position.x;
      const double dy2 = path[idx+1].pose.position.y - path[idx].pose.position.y;
      const double dtheta = wrapAngle(std::atan2(dy2, dx2) - std::atan2(dy1, dx1));
      const double ds = std::sqrt(dx2*dx2 + dy2*dy2) + 1e-9;
      xr(4) = std::clamp(dtheta / ds, cfg_.kappa_min, cfg_.kappa_max);
    }

    x_ref[k] = xr;
    v_ref[k]  = cfg_.parking_velocity;
  }

  return true;
}

// ============================================================
// QP 구성 및 풀이 (배치 정식화)
//
// min   Σ_k ||x(k)-x_ref(k)||_Q + ||u(k)||_R
// s.t.  x(k+1) = A_k·x(k) + B_k·u(k) + d_k
//       u_min ≤ u(k) ≤ u_max
//       x_min ≤ x(k) ≤ x_max
// ============================================================
bool RTINMPCController::buildAndSolveQP(
    const Eigen::VectorXd& x0,
    const std::vector<Eigen::MatrixXd>& A_seq,
    const std::vector<Eigen::MatrixXd>& B_seq,
    const std::vector<Eigen::VectorXd>& d_seq,
    const std::vector<Eigen::VectorXd>& x_ref,
    const std::vector<Eigen::VectorXd>& /*u_warm*/,
    Eigen::VectorXd& u_opt) const {
  const int N  = cfg_.N;
  const int Nx = kRTINx;
  const int Nu = kRTINu;

  // 결정 변수: U = [u(0), u(1), ..., u(N-1)]  크기 N*Nu
  const int n_dec = N * Nu;

  // ---- 비용 행렬 Q, R ----
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(Nx, Nx);
  Q(0, 0) = cfg_.w_px;
  Q(1, 1) = cfg_.w_py;
  Q(2, 2) = cfg_.w_psi;
  Q(3, 3) = cfg_.w_v;
  Q(4, 4) = cfg_.w_kappa;

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(Nu, Nu);
  R(0, 0) = cfg_.w_av;
  R(1, 1) = cfg_.w_akappa;

  // ---- 배치 예측 행렬 구성 ----
  // x(k) = Phi_k·x0 + Gamma_k·U + sigma_k
  // 여기서 Phi_k = A_{k-1}·...·A_0
  //         Gamma_k = [A_{k-1}·...·A_1·B_0, ..., B_{k-1}, 0, ...]

  // Phi: (N+1)*Nx × Nx
  // Gamma: (N+1)*Nx × N*Nu
  Eigen::MatrixXd Phi   = Eigen::MatrixXd::Zero((N+1)*Nx, Nx);
  Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero((N+1)*Nx, n_dec);
  Eigen::VectorXd sigma = Eigen::VectorXd::Zero((N+1)*Nx);

  // k=0: x(0) = x0
  Phi.block(0, 0, Nx, Nx) = Eigen::MatrixXd::Identity(Nx, Nx);

  // 누적 전이 행렬
  Eigen::MatrixXd Phi_k = Eigen::MatrixXd::Identity(Nx, Nx);
  Eigen::VectorXd sigma_k = Eigen::VectorXd::Zero(Nx);

  for (int k = 0; k < N; ++k) {
    const Eigen::MatrixXd& Ak = A_seq[k];
    const Eigen::MatrixXd& Bk = B_seq[k];
    const Eigen::VectorXd& dk = d_seq[k];

    // Phi_{k+1} = A_k · Phi_k
    Phi_k  = Ak * Phi_k;
    sigma_k = Ak * sigma_k + dk;

    Phi.block((k+1)*Nx, 0, Nx, Nx) = Phi_k;
    sigma.segment((k+1)*Nx, Nx) = sigma_k;

    // Gamma 업데이트: 열 j <= k
    for (int j = 0; j <= k; ++j) {
      if (j == k) {
        Gamma.block((k+1)*Nx, j*Nu, Nx, Nu) = Bk;
      } else {
        Gamma.block((k+1)*Nx, j*Nu, Nx, Nu) =
            Ak * Gamma.block(k*Nx, j*Nu, Nx, Nu);
      }
    }
  }

  // ---- QP 목적함수 구성 ----
  // min 0.5 * U^T H U + f^T U
  // H = Gamma^T · Q_bar · Gamma + R_bar
  // f = Gamma^T · Q_bar · (Phi·x0 + sigma - X_ref)

  // Q_bar: 블록 대각 (N+1)*Nx
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero((N+1)*Nx, (N+1)*Nx);
  for (int k = 0; k <= N; ++k) {
    Q_bar.block(k*Nx, k*Nx, Nx, Nx) = Q;
  }
  // 터미널 비용 스케일 (마지막 스텝 강조)
  Q_bar.block(N*Nx, N*Nx, Nx, Nx) *= 3.0;

  // R_bar: 블록 대각 N*Nu
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_dec, n_dec);
  for (int k = 0; k < N; ++k) {
    R_bar.block(k*Nu, k*Nu, Nu, Nu) = R;
  }

  // X_ref 벡터
  Eigen::VectorXd X_ref((N+1)*Nx);
  for (int k = 0; k <= N; ++k) {
    X_ref.segment(k*Nx, Nx) = x_ref[k];
  }

  // 자유 응답 편차
  const Eigen::VectorXd e_free = Phi * x0 + sigma - X_ref;

  // H = Gamma^T Q_bar Gamma + R_bar
  const Eigen::MatrixXd GtQ  = Gamma.transpose() * Q_bar;
  Eigen::MatrixXd H_dense    = GtQ * Gamma + R_bar;

  // 대칭성 보정 + 정규화
  H_dense = 0.5 * (H_dense + H_dense.transpose());
  H_dense += 1e-6 * Eigen::MatrixXd::Identity(n_dec, n_dec);

  const Eigen::VectorXd f = GtQ * e_free;

  // ---- 제약 조건 구성 ----
  // 입력 제약: u_min ≤ u(k) ≤ u_max  (N*Nu개)
  // 상태 제약: kappa 범위만 적용  (N*2개, κ = x(4))

  const int n_u_cons = n_dec;
  const int n_x_cons = N * 2;  // κ 상한/하한
  const int n_cons   = n_u_cons + n_x_cons;

  Eigen::MatrixXd A_cons_dense = Eigen::MatrixXd::Zero(n_cons, n_dec);
  Eigen::VectorXd lb(n_cons), ub(n_cons);

  // 입력 제약
  for (int i = 0; i < n_u_cons; ++i) {
    A_cons_dense(i, i) = 1.0;
  }
  for (int k = 0; k < N; ++k) {
    lb(k*Nu + 0) = cfg_.av_min;
    ub(k*Nu + 0) = cfg_.av_max;
    lb(k*Nu + 1) = cfg_.akappa_min;
    ub(k*Nu + 1) = cfg_.akappa_max;
  }

  // κ 상태 제약: Gamma_kappa·U + (Phi_kappa·x0 + sigma_kappa) ∈ [κ_min, κ_max]
  const int kappa_row = 4;  // 상태 벡터에서 κ 인덱스
  for (int k = 0; k < N; ++k) {
    const int row_k   = (k+1)*Nx + kappa_row;
    const int cons_row = n_u_cons + k*2;

    // 상한/하한 Gamma 행
    A_cons_dense.row(cons_row)   = Gamma.row(row_k);
    A_cons_dense.row(cons_row+1) = Gamma.row(row_k);

    const double kappa_free = (Phi * x0 + sigma)(row_k);
    lb(cons_row)   = cfg_.kappa_min - kappa_free;
    ub(cons_row)   = cfg_.kappa_max - kappa_free;
    lb(cons_row+1) = cfg_.kappa_min - kappa_free;
    ub(cons_row+1) = cfg_.kappa_max - kappa_free;
  }

  // Dense → Sparse 변환
  const auto toSparse = [](const Eigen::MatrixXd& M) {
    Eigen::SparseMatrix<double> S(M.rows(), M.cols());
    std::vector<Eigen::Triplet<double>> triplets;
    for (int i = 0; i < M.rows(); ++i)
      for (int j = 0; j < M.cols(); ++j)
        if (std::abs(M(i,j)) > 1e-12)
          triplets.emplace_back(i, j, M(i,j));
    S.setFromTriplets(triplets.begin(), triplets.end());
    return S;
  };

  const Eigen::SparseMatrix<double> H_sparse    = toSparse(H_dense);
  const Eigen::SparseMatrix<double> A_cons_sparse = toSparse(A_cons_dense);

  return solveQPOSQP(H_sparse, f, A_cons_sparse, lb, ub, u_opt);
}

// ============================================================
// OSQP 인터페이스
// ============================================================
bool RTINMPCController::solveQPOSQP(const Eigen::SparseMatrix<double>& H,
                                     const Eigen::VectorXd& f,
                                     const Eigen::SparseMatrix<double>& A_cons,
                                     const Eigen::VectorXd& lb,
                                     const Eigen::VectorXd& ub,
                                     Eigen::VectorXd& solution) const {
  const int n = H.cols();
  const int m = A_cons.rows();

  // OSQP CSC 형식으로 변환
  Eigen::SparseMatrix<double> H_upper = H.triangularView<Eigen::Upper>();
  H_upper.makeCompressed();
  Eigen::SparseMatrix<double> A_csc = A_cons;
  A_csc.makeCompressed();

  OSQPCscMatrix* P_mat = static_cast<OSQPCscMatrix*>(
      malloc(sizeof(OSQPCscMatrix)));
  OSQPCscMatrix* A_mat = static_cast<OSQPCscMatrix*>(
      malloc(sizeof(OSQPCscMatrix)));

  csc_set_data(P_mat, n, n, H_upper.nonZeros(),
               H_upper.valuePtr(),
               H_upper.innerIndexPtr(),
               H_upper.outerIndexPtr());
  csc_set_data(A_mat, m, n, A_csc.nonZeros(),
               A_csc.valuePtr(),
               A_csc.innerIndexPtr(),
               A_csc.outerIndexPtr());

  OSQPSettings* settings = static_cast<OSQPSettings*>(malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->max_iter  = cfg_.osqp_max_iter;
  settings->eps_abs   = cfg_.osqp_eps_abs;
  settings->eps_rel   = cfg_.osqp_eps_rel;
  settings->warm_starting = cfg_.osqp_warm_start;
  settings->polishing = cfg_.osqp_polish ? 1 : 0;
  settings->verbose   = 0;

  OSQPSolver* solver = nullptr;
  OSQPInt exit_code = osqp_setup(&solver, P_mat, f.data(),
                                  A_mat, lb.data(), ub.data(),
                                  m, n, settings);

  bool solved = false;
  if (exit_code == 0 && solver != nullptr) {
    osqp_solve(solver);
    if (solver->info->status_val == OSQP_SOLVED ||
        solver->info->status_val == OSQP_SOLVED_INACCURATE) {
      solution.resize(n);
      for (int i = 0; i < n; ++i) solution(i) = solver->solution->x[i];
      solved = true;
    }
    osqp_cleanup(solver);
  }

  free(P_mat);
  free(A_mat);
  free(settings);

  return solved;
}

// ============================================================
// 메인 제어 계산
// ============================================================
RTINMPCCommand RTINMPCController::computeControl(
    const geometry_msgs::msg::Pose& ego_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& reference_path,
    double current_v) {
  RTINMPCCommand out;
  const auto t0 = std::chrono::high_resolution_clock::now();

  if (reference_path.size() < 3) return out;

  // --- 초기화 ---
  if (!initialized_) {
    kappa_state_ = 0.0;
    v_state_     = current_v;
    u_warm_.assign(cfg_.N, Eigen::VectorXd::Zero(kRTINu));
    initialized_ = true;
  }

  // --- 현재 상태 벡터 구성 ---
  v_state_ = current_v;
  Eigen::VectorXd x0 = buildStateVector(ego_pose, current_v);
  x0(4) = kappa_state_;

  // --- 참조 시퀀스 구성 ---
  std::vector<Eigen::VectorXd> x_ref;
  std::vector<double> v_ref;
  if (!buildReferenceSequence(x0, reference_path, x_ref, v_ref)) return out;

  // closest 인덱스 업데이트
  last_closest_idx_ = std::max(0,
      findClosestWaypoint(x0, reference_path, last_closest_idx_) - 2);

  const auto t_model_start = std::chrono::high_resolution_clock::now();

  // --- RTI: 예측 궤적 선형화 ---
  // warm-start 궤적 시뮬레이션
  std::vector<Eigen::VectorXd> x_traj(cfg_.N + 1);
  x_traj[0] = x0;
  for (int k = 0; k < cfg_.N; ++k) {
    x_traj[k+1] = implicitEulerStep(x_traj[k], u_warm_[k]);
  }

  // 각 스텝에서 이산화 + 선형화
  std::vector<Eigen::MatrixXd> A_seq(cfg_.N), B_seq(cfg_.N);
  std::vector<Eigen::VectorXd> d_seq(cfg_.N);
  for (int k = 0; k < cfg_.N; ++k) {
    discretizeLinearize(x_traj[k], u_warm_[k], A_seq[k], B_seq[k], d_seq[k]);
  }

  const auto t_model_end = std::chrono::high_resolution_clock::now();
  out.model_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
      t_model_end - t_model_start).count();

  // --- QP 풀이 ---
  const auto t_solver_start = std::chrono::high_resolution_clock::now();
  Eigen::VectorXd u_flat;
  const bool ok = buildAndSolveQP(x0, A_seq, B_seq, d_seq, x_ref, u_warm_, u_flat);
  const auto t_solver_end = std::chrono::high_resolution_clock::now();
  out.solver_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
      t_solver_end - t_solver_start).count();

  if (!ok || u_flat.size() < cfg_.N * kRTINu) return out;

  // --- 입력 적용 ---
  // warm-start 시프트: u_warm[k] ← u_opt[k+1]
  for (int k = 0; k < cfg_.N - 1; ++k) {
    u_warm_[k] = u_flat.segment((k+1) * kRTINu, kRTINu);
  }
  u_warm_[cfg_.N - 1] = u_flat.segment((cfg_.N - 1) * kRTINu, kRTINu);

  // 첫 번째 입력
  const double av0    = u_flat(0);
  const double akap0  = u_flat(1);

  // 상태 적분
  v_state_     = std::clamp(v_state_     + cfg_.Ts * av0,
                             cfg_.min_velocity, cfg_.max_velocity);
  kappa_state_ = std::clamp(kappa_state_ + cfg_.Ts * akap0,
                             cfg_.kappa_min, cfg_.kappa_max);

  // --- 출력 ---
  out.solved    = true;
  out.kappa_cmd = kappa_state_;
  out.v_cmd     = v_state_;
  out.omega_cmd = v_state_ * kappa_state_;

  // 예측 궤적 (시각화용)
  out.predicted_xy.reserve(cfg_.N);
  double px = ego_pose.position.x;
  double py = ego_pose.position.y;
  double psi = poseYaw(ego_pose);
  double v_p = v_state_;
  double k_p = kappa_state_;
  for (int k = 0; k < cfg_.N; ++k) {
    px  += v_p * std::cos(psi) * cfg_.Ts;
    py  += v_p * std::sin(psi) * cfg_.Ts;
    psi  = wrapAngle(psi + v_p * k_p * cfg_.Ts);
    out.predicted_xy.push_back({px, py, psi});
  }

  return out;
}

}  // namespace bisa
