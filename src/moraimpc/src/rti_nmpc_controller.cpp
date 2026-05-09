#include "moraimpc/rti_nmpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace moraimpc {

namespace {

double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// Pose에서 yaw 추출 (쿼터니언 → atan2, z 필드 fallback)
double poseYaw(const geometry_msgs::Pose& p) {
    const auto& q = p.orientation;
    const double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (std::isfinite(n) && n > 1e-6 && std::abs(n - 1.0) <= 0.05) {
        const double xn = q.x/n, yn = q.y/n, zn = q.z/n, wn = q.w/n;
        return wrapAngle(std::atan2(2.0*(wn*zn + xn*yn), 1.0 - 2.0*(yn*yn + zn*zn)));
    }
    return wrapAngle(q.z);
}

double poseYawStamped(const geometry_msgs::PoseStamped& ps) {
    return poseYaw(ps.pose);
}

}  // namespace

// ============================================================
// 생성자 / 소멸자 / 설정
// ============================================================
RTINMPCController::RTINMPCController(const RTINMPCConfig& cfg) : cfg_(cfg) {
    settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose    = 0;
    settings_->max_iter   = cfg_.osqp_max_iter;
    settings_->eps_abs    = cfg_.osqp_eps_abs;
    settings_->eps_rel    = cfg_.osqp_eps_rel;
    settings_->warm_start = cfg_.osqp_warm_start ? 1 : 0;
    reset();
}

RTINMPCController::~RTINMPCController() {
    if (solver_)   osqp_cleanup(solver_);
    if (settings_) c_free(settings_);
}

void RTINMPCController::setConfig(const RTINMPCConfig& cfg) {
    cfg_ = cfg;
    settings_->max_iter   = cfg_.osqp_max_iter;
    settings_->eps_abs    = cfg_.osqp_eps_abs;
    settings_->eps_rel    = cfg_.osqp_eps_rel;
    settings_->warm_start = cfg_.osqp_warm_start ? 1 : 0;
}

void RTINMPCController::reset() {
    kappa_state_     = 0.0;
    v_state_         = 0.0;
    initialized_     = false;
    last_closest_idx_ = 0;
    u_warm_.assign(cfg_.N, Eigen::VectorXd::Zero(kRTINu));
    if (solver_) { osqp_cleanup(solver_); solver_ = nullptr; }
}

void RTINMPCController::setInitialKappa(double kappa) {
    kappa_state_ = std::clamp(kappa, cfg_.kappa_min, cfg_.kappa_max);
    initialized_ = true;   // skip 자체 초기화 (kappa_state_ 보존)
    // warm-start sequence를 현재 kappa 유지하도록 초기화 (av=0, akappa=0 → 같은 kappa 유지)
    u_warm_.assign(cfg_.N, Eigen::VectorXd::Zero(kRTINu));
}

// ============================================================
// 운동학 모델 f(x, u)
// x = [px, py, psi, v, kappa], u = [av, a_kappa]
// ============================================================
Eigen::VectorXd RTINMPCController::dynamics(const Eigen::VectorXd& x,
                                             const Eigen::VectorXd& u) const {
    Eigen::VectorXd xdot(kRTINx);
    const double psi   = x(2);
    const double v     = x(3);
    const double kappa = x(4);

    xdot(0) = v * std::cos(psi);
    xdot(1) = v * std::sin(psi);
    xdot(2) = v * kappa;
    xdot(3) = u(0);    // v̇ = av
    xdot(4) = u(1);    // κ̇ = a_κ
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

    Jx(0, 2) = -v * std::sin(psi);   // ∂(v·cosψ)/∂ψ
    Jx(0, 3) =  std::cos(psi);       // ∂(v·cosψ)/∂v
    Jx(1, 2) =  v * std::cos(psi);   // ∂(v·sinψ)/∂ψ
    Jx(1, 3) =  std::sin(psi);       // ∂(v·sinψ)/∂v
    Jx(2, 3) =  kappa;               // ∂(v·κ)/∂v
    Jx(2, 4) =  v;                   // ∂(v·κ)/∂κ
    return Jx;
}

// ============================================================
// ∂f/∂u 야코비안
// ============================================================
Eigen::MatrixXd RTINMPCController::jacobianFu(const Eigen::VectorXd& /*x*/,
                                               const Eigen::VectorXd& /*u*/) const {
    Eigen::MatrixXd Ju = Eigen::MatrixXd::Zero(kRTINx, kRTINu);
    Ju(3, 0) = 1.0;   // ∂v̇/∂av
    Ju(4, 1) = 1.0;   // ∂κ̇/∂a_κ
    return Ju;
}

// ============================================================
// 암시적 오일러: x(k+1) = x(k) + Ts·f(x(k+1), u(k))
// Newton 반복으로 풀이
// ============================================================
Eigen::VectorXd RTINMPCController::implicitEulerStep(const Eigen::VectorXd& x_k,
                                                      const Eigen::VectorXd& u_k) const {
    const double Ts = cfg_.Ts;
    Eigen::VectorXd x_next = x_k;

    for (int iter = 0; iter < cfg_.newton_max_iter; ++iter) {
        const Eigen::VectorXd r = x_next - x_k - Ts * dynamics(x_next, u_k);
        const Eigen::MatrixXd Jr = Eigen::MatrixXd::Identity(kRTINx, kRTINx)
                                   - Ts * jacobianFx(x_next, u_k);
        // Jr는 비대칭 (Fx 비대칭) → ldlt 부적합. partialPivLu 사용.
        x_next -= Jr.partialPivLu().solve(r);
        if (r.norm() < 1e-8) break;
    }

    x_next(2) = wrapAngle(x_next(2));
    x_next(3) = std::clamp(x_next(3), cfg_.min_velocity, cfg_.max_velocity);
    x_next(4) = std::clamp(x_next(4), cfg_.kappa_min,    cfg_.kappa_max);
    return x_next;
}

// ============================================================
// 이산화 + 선형화
// A_d ≈ (I - Ts·Fx)^{-1},  B_d ≈ A_d·Ts·Fu
// d_k = x_next_nl - A_d·x_k - B_d·u_k
// ============================================================
void RTINMPCController::discretizeLinearize(const Eigen::VectorXd& x_k,
                                            const Eigen::VectorXd& u_k,
                                            Eigen::MatrixXd& A_d,
                                            Eigen::MatrixXd& B_d,
                                            Eigen::VectorXd& d_k) const {
    const double Ts = cfg_.Ts;
    const Eigen::MatrixXd Fx = jacobianFx(x_k, u_k);
    const Eigen::MatrixXd Fu = jacobianFu(x_k, u_k);
    const Eigen::MatrixXd M  = Eigen::MatrixXd::Identity(kRTINx, kRTINx) - Ts * Fx;

    A_d = M.inverse();
    B_d = A_d * Ts * Fu;

    const Eigen::VectorXd x_next_nl = implicitEulerStep(x_k, u_k);
    d_k = x_next_nl - A_d * x_k - B_d * u_k;
}

// ============================================================
// 상태 벡터 구성
// ============================================================
Eigen::VectorXd RTINMPCController::buildStateVector(
    const geometry_msgs::Pose& pose, double v) const {
    Eigen::VectorXd x(kRTINx);
    x(0) = pose.position.x;
    x(1) = pose.position.y;
    x(2) = poseYaw(pose);
    x(3) = v;
    x(4) = kappa_state_;
    return x;
}

// ============================================================
// 가장 가까운 경로점 탐색 (80점 윈도우)
// ============================================================
int RTINMPCController::findClosestWaypoint(
    const Eigen::VectorXd& x0,
    const std::vector<geometry_msgs::PoseStamped>& path,
    int search_start) const {
    const int n     = static_cast<int>(path.size());
    const int start = std::max(0, search_start);
    const int end   = std::min(n, start + 80);

    int    best_idx  = start;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = start; i < end; ++i) {
        const double dx = path[i].pose.position.x - x0(0);
        const double dy = path[i].pose.position.y - x0(1);
        const double d  = dx*dx + dy*dy;
        if (d < best_dist) { best_dist = d; best_idx = i; }
    }
    return best_idx;
}

// ============================================================
// 참조 상태 시퀀스 구성
// ============================================================
bool RTINMPCController::buildReferenceSequence(
    const Eigen::VectorXd& x0,
    const std::vector<geometry_msgs::PoseStamped>& path,
    std::vector<Eigen::VectorXd>& x_ref) const {
    const int n = static_cast<int>(path.size());
    if (n < 2) return false;

    const int closest = findClosestWaypoint(x0, path, last_closest_idx_);
    x_ref.resize(cfg_.N + 1);

    for (int k = 0; k <= cfg_.N; ++k) {
        const int idx = std::min(closest + k, n - 1);
        Eigen::VectorXd xr(kRTINx);
        xr(0) = path[idx].pose.position.x;
        xr(1) = path[idx].pose.position.y;
        xr(2) = poseYawStamped(path[idx]);
        xr(3) = cfg_.target_velocity;
        xr(4) = 0.0;

        // 중간 점에서 곡률 추정
        if (idx > 0 && idx < n - 1) {
            const double dx2 = path[idx+1].pose.position.x - path[idx].pose.position.x;
            const double dy2 = path[idx+1].pose.position.y - path[idx].pose.position.y;
            const double dx1 = path[idx].pose.position.x - path[idx-1].pose.position.x;
            const double dy1 = path[idx].pose.position.y - path[idx-1].pose.position.y;
            const double dtheta = wrapAngle(std::atan2(dy2, dx2) - std::atan2(dy1, dx1));
            const double ds = std::sqrt(dx2*dx2 + dy2*dy2) + 1e-9;
            // R 모드 (target_velocity<0): κ_steering = -κ_path (수학 시뮬 검증됨)
            double k_path = dtheta / ds;
            if (cfg_.target_velocity < 0) k_path = -k_path;
            xr(4) = std::clamp(k_path, cfg_.kappa_min, cfg_.kappa_max);
        }
        x_ref[k] = xr;
    }
    return true;
}

// ============================================================
// 배치 QP 구성 및 풀이
// min  Σ_k ||x(k)-x_ref(k)||_Q + ||u(k)||_R
// s.t. x(k+1) = A_k·x(k) + B_k·u(k) + d_k
//      u_min ≤ u(k) ≤ u_max,  kappa_min ≤ κ(k) ≤ kappa_max
// ============================================================
bool RTINMPCController::buildAndSolveQP(
    const Eigen::VectorXd& x0,
    const std::vector<Eigen::MatrixXd>& A_seq,
    const std::vector<Eigen::MatrixXd>& B_seq,
    const std::vector<Eigen::VectorXd>& d_seq,
    const std::vector<Eigen::VectorXd>& x_ref,
    Eigen::VectorXd& u_opt) {
    const int N     = cfg_.N;
    const int Nx    = kRTINx;
    const int Nu    = kRTINu;
    const int n_dec = N * Nu;

    // 비용 행렬 Q, R
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(Nx, Nx);
    Q(0,0) = cfg_.w_px;    Q(1,1) = cfg_.w_py;
    Q(2,2) = cfg_.w_psi;   Q(3,3) = cfg_.w_v;
    Q(4,4) = cfg_.w_kappa;

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(Nu, Nu);
    R(0,0) = cfg_.w_av;    R(1,1) = cfg_.w_akappa;

    // 배치 예측 행렬: x(k) = Phi_k·x0 + Gamma_k·U + sigma_k
    Eigen::MatrixXd Phi   = Eigen::MatrixXd::Zero((N+1)*Nx, Nx);
    Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero((N+1)*Nx, n_dec);
    Eigen::VectorXd sigma = Eigen::VectorXd::Zero((N+1)*Nx);

    Phi.block(0, 0, Nx, Nx) = Eigen::MatrixXd::Identity(Nx, Nx);
    Eigen::MatrixXd Phi_k   = Eigen::MatrixXd::Identity(Nx, Nx);
    Eigen::VectorXd sigma_k = Eigen::VectorXd::Zero(Nx);

    for (int k = 0; k < N; ++k) {
        const auto& Ak = A_seq[k];
        const auto& Bk = B_seq[k];
        const auto& dk = d_seq[k];

        Phi_k   = Ak * Phi_k;
        sigma_k = Ak * sigma_k + dk;
        Phi.block((k+1)*Nx, 0, Nx, Nx)     = Phi_k;
        sigma.segment((k+1)*Nx, Nx)          = sigma_k;

        for (int j = 0; j <= k; ++j) {
            Gamma.block((k+1)*Nx, j*Nu, Nx, Nu) = (j == k)
                ? Bk
                : Ak * Gamma.block(k*Nx, j*Nu, Nx, Nu);
        }
    }

    // Q_bar (블록 대각, 터미널 3배 강조)
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero((N+1)*Nx, (N+1)*Nx);
    for (int k = 0; k <= N; ++k)
        Q_bar.block(k*Nx, k*Nx, Nx, Nx) = (k == N) ? 3.0 * Q : Q;

    // R_bar
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_dec, n_dec);
    for (int k = 0; k < N; ++k)
        R_bar.block(k*Nu, k*Nu, Nu, Nu) = R;

    // X_ref 벡터
    Eigen::VectorXd X_ref((N+1)*Nx);
    for (int k = 0; k <= N; ++k)
        X_ref.segment(k*Nx, Nx) = x_ref[k];

    Eigen::VectorXd e_free = Phi * x0 + sigma - X_ref;
    // ψ 성분 wrap [-π,π] — 안 하면 vehicle yaw 근처 ±π에서 raw diff 358°로 잘못된 gradient
    const int Nx_local = kRTINx;
    for (int k = 0; k <= N; ++k) {
        int psi_i = k * Nx_local + 2;   // ψ 인덱스
        double w = e_free(psi_i);
        while (w >  M_PI) w -= 2.0 * M_PI;
        while (w < -M_PI) w += 2.0 * M_PI;
        e_free(psi_i) = w;
    }
    const Eigen::MatrixXd GtQ   = Gamma.transpose() * Q_bar;

    Eigen::MatrixXd H_dense = GtQ * Gamma + R_bar;
    H_dense = 0.5 * (H_dense + H_dense.transpose());
    H_dense += 1e-6 * Eigen::MatrixXd::Identity(n_dec, n_dec);

    const Eigen::VectorXd f_vec = GtQ * e_free;

    // 제약: 입력 제약 + κ 상태 제약
    const int n_u_cons = n_dec;
    const int n_x_cons = N * 2;
    const int n_cons   = n_u_cons + n_x_cons;

    Eigen::MatrixXd A_cons = Eigen::MatrixXd::Zero(n_cons, n_dec);
    Eigen::VectorXd lb(n_cons), ub(n_cons);

    for (int i = 0; i < n_u_cons; ++i) A_cons(i, i) = 1.0;
    for (int k = 0; k < N; ++k) {
        lb(k*Nu + 0) = cfg_.av_min;    ub(k*Nu + 0) = cfg_.av_max;
        lb(k*Nu + 1) = cfg_.akappa_min; ub(k*Nu + 1) = cfg_.akappa_max;
    }

    const Eigen::VectorXd x_free = Phi * x0 + sigma;
    for (int k = 0; k < N; ++k) {
        const int row_k    = (k+1)*Nx + 4;  // κ 행
        const int cons_row = n_u_cons + k*2;
        A_cons.row(cons_row)   = Gamma.row(row_k);
        A_cons.row(cons_row+1) = Gamma.row(row_k);
        lb(cons_row)   = cfg_.kappa_min - x_free(row_k);
        ub(cons_row)   = cfg_.kappa_max - x_free(row_k);
        lb(cons_row+1) = cfg_.kappa_min - x_free(row_k);
        ub(cons_row+1) = cfg_.kappa_max - x_free(row_k);
    }

    Eigen::SparseMatrix<double> P_sp = (2.0 * H_dense).sparseView();
    P_sp = P_sp.triangularView<Eigen::Upper>();
    P_sp.makeCompressed();
    Eigen::SparseMatrix<double> A_sp = A_cons.sparseView();
    A_sp.makeCompressed();

    return solveOSQP(P_sp, 2.0 * f_vec, A_sp, lb, ub, u_opt);
}

// ============================================================
// OSQP 풀이 (구버전 API: OSQPWorkspace, csc_matrix)
// ============================================================
// 안전한 Eigen → OSQP CSC 변환 (element-wise; c_float typedef 호환)
static csc* eigenSparseToCsc(const Eigen::SparseMatrix<double>& mat) {
    const int rows = mat.rows();
    const int cols = mat.cols();
    const int nnz  = mat.nonZeros();
    c_float* x  = (c_float*)c_malloc(nnz * sizeof(c_float));
    c_int*   ri = (c_int*)  c_malloc(nnz * sizeof(c_int));
    c_int*   p  = (c_int*)  c_malloc((cols + 1) * sizeof(c_int));
    int cnt = 0;
    for (int col = 0; col < cols; ++col) {
        p[col] = cnt;
        for (Eigen::SparseMatrix<double>::InnerIterator it(mat, col); it; ++it) {
            x[cnt]  = (c_float)it.value();
            ri[cnt] = (c_int) it.row();
            cnt++;
        }
    }
    p[cols] = cnt;
    return csc_matrix(rows, cols, nnz, x, ri, p);
}

bool RTINMPCController::solveOSQP(const Eigen::SparseMatrix<double>& P,
                                   const Eigen::VectorXd& q,
                                   const Eigen::SparseMatrix<double>& Ac,
                                   const Eigen::VectorXd& lb,
                                   const Eigen::VectorXd& ub,
                                   Eigen::VectorXd& sol) {
    const int n = P.cols();
    const int m = Ac.rows();

    std::vector<c_float> q_v(n), l_v(m), u_v(m);
    for (int i = 0; i < n; ++i) q_v[i] = (c_float)q(i);
    for (int i = 0; i < m; ++i) { l_v[i] = (c_float)lb(i); u_v[i] = (c_float)ub(i); }

    OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));
    data->n = n; data->m = m;
    data->P = eigenSparseToCsc(P);
    data->q = q_v.data();
    data->A = eigenSparseToCsc(Ac);
    data->l = l_v.data();
    data->u = u_v.data();

    if (solver_) { osqp_cleanup(solver_); solver_ = nullptr; }
    osqp_setup(&solver_, data, settings_);
    osqp_solve(solver_);

    bool ok = (solver_->info->status_val == OSQP_SOLVED ||
               solver_->info->status_val == OSQP_SOLVED_INACCURATE);
    if (ok) {
        sol.resize(n);
        for (int i = 0; i < n; ++i) sol(i) = solver_->solution->x[i];
    }

    // csc_matrix 내부 버퍼 해제
    c_free(data->P->x); c_free(data->P->i); c_free(data->P->p); c_free(data->P);
    c_free(data->A->x); c_free(data->A->i); c_free(data->A->p); c_free(data->A);
    c_free(data);
    return ok;
}

// ============================================================
// 메인 제어 계산
// ============================================================
RTINMPCCommand RTINMPCController::computeControl(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<geometry_msgs::PoseStamped>& reference_path,
    double current_v) {
    RTINMPCCommand out;

    if (reference_path.size() < 3) return out;

    if (!initialized_) {
        kappa_state_ = 0.0;
        v_state_     = current_v;
        u_warm_.assign(cfg_.N, Eigen::VectorXd::Zero(kRTINu));
        initialized_ = true;
    }

    v_state_ = current_v;
    Eigen::VectorXd x0 = buildStateVector(ego_pose, current_v);
    x0(4) = kappa_state_;

    std::vector<Eigen::VectorXd> x_ref;
    if (!buildReferenceSequence(x0, reference_path, x_ref)) return out;

    last_closest_idx_ = std::max(0,
        findClosestWaypoint(x0, reference_path, last_closest_idx_) - 2);

    const auto t_model_start = std::chrono::high_resolution_clock::now();

    // warm-start 궤적 시뮬레이션 + 선형화
    std::vector<Eigen::VectorXd> x_traj(cfg_.N + 1);
    x_traj[0] = x0;
    for (int k = 0; k < cfg_.N; ++k)
        x_traj[k+1] = implicitEulerStep(x_traj[k], u_warm_[k]);

    std::vector<Eigen::MatrixXd> A_seq(cfg_.N), B_seq(cfg_.N);
    std::vector<Eigen::VectorXd> d_seq(cfg_.N);
    for (int k = 0; k < cfg_.N; ++k)
        discretizeLinearize(x_traj[k], u_warm_[k], A_seq[k], B_seq[k], d_seq[k]);

    out.model_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - t_model_start).count();

    const auto t_solver_start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd u_flat;
    const bool ok = buildAndSolveQP(x0, A_seq, B_seq, d_seq, x_ref, u_flat);
    out.solver_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - t_solver_start).count();

    if (!ok || u_flat.size() < cfg_.N * kRTINu) return out;

    // warm-start 시프트
    for (int k = 0; k < cfg_.N - 1; ++k)
        u_warm_[k] = u_flat.segment((k+1) * kRTINu, kRTINu);
    u_warm_[cfg_.N - 1] = u_flat.segment((cfg_.N - 1) * kRTINu, kRTINu);

    const double av0   = u_flat(0);
    const double akap0 = u_flat(1);

    v_state_     = std::clamp(v_state_     + cfg_.Ts * av0,
                               cfg_.min_velocity, cfg_.max_velocity);
    kappa_state_ = std::clamp(kappa_state_ + cfg_.Ts * akap0,
                               cfg_.kappa_min, cfg_.kappa_max);

    out.solved    = true;
    out.kappa_cmd = kappa_state_;
    out.v_cmd     = v_state_;
    out.omega_cmd = v_state_ * kappa_state_;
    // κ = tan(δ)/L → δ = atan(κ·L)
    out.steer_deg = std::atan(kappa_state_ * cfg_.wheelbase) * 180.0 / M_PI;

    // 예측 궤적 (시각화용)
    out.predicted_xy.reserve(cfg_.N);
    double px  = ego_pose.position.x;
    double py  = ego_pose.position.y;
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

}  // namespace moraimpc
