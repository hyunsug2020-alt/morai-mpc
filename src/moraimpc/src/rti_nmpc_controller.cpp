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

void RTINMPCController::setObstacles(const std::vector<RTINMPCObstacle>& obs) {
    obstacles_ = obs;
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

    // 속도 기반 fractional idx_step + 선형 보간 (저속 ref far-ahead 버그 fix)
    // - 이전: idx = closest + k → wp 1개씩 (저속 시 ref 11x 멀리 봄)
    // - 현재: idx_step_f = v·Ts / wp_spacing, 보간으로 정확한 v·Ts·k 거리 ref 생성
    const double v_abs = std::max(0.3, std::abs(cfg_.target_velocity));
    const double dx_first = path[std::min(1, n-1)].pose.position.x - path[0].pose.position.x;
    const double dy_first = path[std::min(1, n-1)].pose.position.y - path[0].pose.position.y;
    const double wp_spacing = std::max(0.01, std::hypot(dx_first, dy_first));
    const double idx_step_f = v_abs * cfg_.Ts / wp_spacing;

    for (int k = 0; k <= cfg_.N; ++k) {
        const double idx_real = closest + k * idx_step_f;
        const int idx  = std::min((int)idx_real, n - 1);
        const int idx2 = std::min(idx + 1, n - 1);
        const double t = std::clamp(idx_real - idx, 0.0, 1.0);

        Eigen::VectorXd xr(kRTINx);
        // 위치 선형 보간
        xr(0) = path[idx].pose.position.x + t * (path[idx2].pose.position.x - path[idx].pose.position.x);
        xr(1) = path[idx].pose.position.y + t * (path[idx2].pose.position.y - path[idx].pose.position.y);
        // yaw 보간 (wrap 처리)
        const double y0 = poseYawStamped(path[idx]);
        const double y1 = poseYawStamped(path[idx2]);
        xr(2) = y0 + t * wrapAngle(y1 - y0);
        xr(3) = cfg_.target_velocity;
        xr(4) = 0.0;

        // 곡률 추정 (보간된 idx 양쪽)
        if (idx > 0 && idx < n - 1) {
            const double dx2 = path[idx+1].pose.position.x - path[idx].pose.position.x;
            const double dy2 = path[idx+1].pose.position.y - path[idx].pose.position.y;
            const double dx1 = path[idx].pose.position.x - path[idx-1].pose.position.x;
            const double dy1 = path[idx].pose.position.y - path[idx-1].pose.position.y;
            const double dtheta = wrapAngle(std::atan2(dy2, dx2) - std::atan2(dy1, dx1));
            const double ds = std::sqrt(dx2*dx2 + dy2*dy2) + 1e-9;
            double k_path = dtheta / ds;
            if (cfg_.target_velocity < 0) k_path = -k_path;  // R: κ_steering=-κ_path
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

    // LTV 기법 (C29): 곡선 boost + w_psi 속도 적응 계산
    double max_kappa_ref = 0.0;
    double v_avg = 0.0;
    for (int k = 0; k <= N; ++k) {
        max_kappa_ref = std::max(max_kappa_ref, std::abs(x_ref[k](4)));
        v_avg += std::abs(x_ref[k](3));
    }
    v_avg /= (N + 1);
    const double pos_mult = (max_kappa_ref > cfg_.curve_kappa_thresh) ? cfg_.w_pos_curve_boost : 1.0;
    // w_psi 적응: 단 path_follower가 override(default 10에서 ±0.5 벗어남)했으면 그대로 사용
    double w_psi_eff;
    if (std::abs(cfg_.w_psi - 10.0) > 0.5) {
        w_psi_eff = cfg_.w_psi;
    } else {
        const double t = std::clamp((v_avg - cfg_.w_psi_v_low)
                                    / (cfg_.w_psi_v_high - cfg_.w_psi_v_low + 1e-6), 0.0, 1.0);
        w_psi_eff = cfg_.w_psi_low_speed + t * (cfg_.w_psi_high_speed - cfg_.w_psi_low_speed);
    }

    // 비용 행렬 Q (스테이지 균일, 곡선 boost 적용)
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(Nx, Nx);
    Q(0,0) = cfg_.w_px * pos_mult;    Q(1,1) = cfg_.w_py * pos_mult;
    Q(2,2) = w_psi_eff;               Q(3,3) = cfg_.w_v;
    Q(4,4) = cfg_.w_kappa;

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

    // R_bar — LTV 기법: stage별 v 비례 input cost (직선 진동 제거 핵심)
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_dec, n_dec);
    for (int k = 0; k < N; ++k) {
        const double v_k = std::abs(x_ref[k](3));
        Eigen::Matrix2d Rk = Eigen::Matrix2d::Zero();
        Rk(0,0) = cfg_.w_av     + cfg_.w_av_v_gain     * v_k;
        Rk(1,1) = cfg_.w_akappa + cfg_.w_akappa_v_gain * v_k;
        R_bar.block(k*Nu, k*Nu, Nu, Nu) = Rk;
    }

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

    // ────────────────────────────────────────────────────────────
    // 장애물 stage 제약 (방안 B: NMPC stage 제약)
    // 비선형 √((px-ox)² + (py-oy)²) ≥ r_safe → x_ref 둘레로 1차 Taylor
    // soft slack penalty로 infeasibility 흡수
    // ────────────────────────────────────────────────────────────
    std::vector<RTINMPCObstacle> active_obs;
    if (cfg_.obs_enable && !obstacles_.empty()) {
        // 차량 현재 위치(x0) 기준 거리만 사용 — 예측 경로 전체 검사 시 미래 path NPC도 활성되어 조기 회피 발생
        const double ego_x = x_ref[0](0), ego_y = x_ref[0](1);
        std::vector<std::pair<double, int>> dist_idx;
        for (size_t i = 0; i < obstacles_.size(); ++i) {
            const auto& o = obstacles_[i];
            const double dx = ego_x - o.cx;
            const double dy = ego_y - o.cy;
            const double d_now = std::sqrt(dx*dx + dy*dy);
            if (d_now < cfg_.obs_active_dist + o.r_safe) {
                dist_idx.emplace_back(d_now, (int)i);
            }
        }
        std::sort(dist_idx.begin(), dist_idx.end());
        const int n_cap = std::min((int)dist_idx.size(), cfg_.obs_max_count);
        for (int i = 0; i < n_cap; ++i) {
            active_obs.push_back(obstacles_[dist_idx[i].second]);
        }
    }
    const int n_obs   = (int)active_obs.size();
    const int n_slack = n_obs * N;          // stage k=0..N-1 (sk=1..N), obs별 slack
    const int n_dec_new = n_dec + n_slack;

    if (n_obs == 0) {
        // 기존 경로 유지 (zero overhead)
        Eigen::SparseMatrix<double> P_sp = (2.0 * H_dense).sparseView();
        P_sp = P_sp.triangularView<Eigen::Upper>();
        P_sp.makeCompressed();
        Eigen::SparseMatrix<double> A_sp = A_cons.sparseView();
        A_sp.makeCompressed();
        return solveOSQP(P_sp, 2.0 * f_vec, A_sp, lb, ub, u_opt);
    }

    // H_full = blockdiag(H_dense, w_slack_quad·I)
    Eigen::MatrixXd H_full = Eigen::MatrixXd::Zero(n_dec_new, n_dec_new);
    H_full.block(0, 0, n_dec, n_dec) = H_dense;
    H_full.block(n_dec, n_dec, n_slack, n_slack) =
        cfg_.w_obs_slack_quad * Eigen::MatrixXd::Identity(n_slack, n_slack);

    Eigen::VectorXd f_full(n_dec_new);
    f_full.head(n_dec) = f_vec;
    f_full.tail(n_slack) = 0.5 * cfg_.w_obs_slack_lin * Eigen::VectorXd::Ones(n_slack);

    // A_cons_new = [A_cons | 0; obs rows | I_slack; 0 | I_slack≥0]
    const int n_obs_rows  = n_obs * N;       // obstacle 제약
    const int n_slack_pos = n_slack;         // slack ≥ 0
    const int n_cons_new  = n_cons + n_obs_rows + n_slack_pos;

    Eigen::MatrixXd A_cons_new = Eigen::MatrixXd::Zero(n_cons_new, n_dec_new);
    Eigen::VectorXd lb_new(n_cons_new), ub_new(n_cons_new);

    A_cons_new.block(0, 0, n_cons, n_dec) = A_cons;
    lb_new.head(n_cons) = lb;
    ub_new.head(n_cons) = ub;

    const double kInf = 1e30;
    int row_off = n_cons;
    for (int i = 0; i < n_obs; ++i) {
        const auto& o = active_obs[i];
        for (int k = 0; k < N; ++k) {
            const int sk = k + 1;  // stage k+1 (k=0..N-1 → sk=1..N)
            if (sk <= cfg_.obs_skip_first) {
                // 너무 가까운 stage skip — slack 변수만 ≥0 강제
                lb_new(row_off) = -kInf;
                ub_new(row_off) =  kInf;
                ++row_off;
                continue;
            }
            const double px_nom = x_ref[sk](0);
            const double py_nom = x_ref[sk](1);
            const double dx = px_nom - o.cx;
            const double dy = py_nom - o.cy;
            const double d_nom = std::sqrt(dx*dx + dy*dy + 1e-9);
            const double nx_g = dx / d_nom;
            const double ny_g = dy / d_nom;

            // row = nx · Gamma_px + ny · Gamma_py + slack
            A_cons_new.row(row_off).head(n_dec) =
                nx_g * Gamma.row(sk*Nx + 0) + ny_g * Gamma.row(sk*Nx + 1);
            A_cons_new(row_off, n_dec + i*N + k) = 1.0;  // +slack

            // bound: nx·px + ny·py + s ≥ r_safe + nx·ox + ny·oy
            //        nx·(Phi*x0+Gamma*u+sigma)_px + ny·(...)_py + s ≥ ...
            // → Gamma*u 항만 LHS, 나머지 RHS
            const double bound = o.r_safe + nx_g*o.cx + ny_g*o.cy
                               - nx_g * x_free(sk*Nx + 0)
                               - ny_g * x_free(sk*Nx + 1);
            lb_new(row_off) = bound;
            ub_new(row_off) = kInf;
            ++row_off;
        }
    }
    // slack ≥ 0
    for (int j = 0; j < n_slack; ++j) {
        A_cons_new(row_off, n_dec + j) = 1.0;
        lb_new(row_off) = 0.0;
        ub_new(row_off) = kInf;
        ++row_off;
    }

    Eigen::SparseMatrix<double> P_sp = (2.0 * H_full).sparseView();
    P_sp = P_sp.triangularView<Eigen::Upper>();
    P_sp.makeCompressed();
    Eigen::SparseMatrix<double> A_sp = A_cons_new.sparseView();
    A_sp.makeCompressed();

    Eigen::VectorXd z_opt;
    const bool ok = solveOSQP(P_sp, 2.0 * f_full, A_sp, lb_new, ub_new, z_opt);
    if (ok && z_opt.size() >= n_dec) {
        u_opt = z_opt.head(n_dec);
    } else {
        u_opt = Eigen::VectorXd::Zero(n_dec);
    }
    return ok;
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
