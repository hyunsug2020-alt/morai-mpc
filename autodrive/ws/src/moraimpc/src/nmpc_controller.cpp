#include "moraimpc/nmpc_controller.hpp"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace moraimpc {

static inline double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

NMPCController::NMPCController(const NMPCConfig& cfg) : cfg_(cfg) {
    settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose  = 0;
    settings_->max_iter = 2000;
    settings_->eps_abs  = 1e-4;
    settings_->eps_rel  = 1e-4;
    settings_->warm_start = 1;
    u_warm_ = Eigen::VectorXd::Zero(cfg_.N);
}

NMPCController::~NMPCController() {
    if (work_)     osqp_cleanup(work_);
    if (settings_) c_free(settings_);
}

void NMPCController::reset() {
    has_warm_ = false;
    u_warm_.setZero();
}

csc* NMPCController::eigenToCsc(const Eigen::SparseMatrix<double>& mat) {
    int m = mat.rows();
    int n = mat.cols();
    int nnz = mat.nonZeros();

    c_float* x  = (c_float*)c_malloc(nnz * sizeof(c_float));
    c_int*   ri = (c_int*)c_malloc(nnz * sizeof(c_int));
    c_int*   p  = (c_int*)c_malloc((n + 1) * sizeof(c_int));

    int count = 0;
    for (int col = 0; col < n; ++col) {
        p[col] = count;
        for (Eigen::SparseMatrix<double>::InnerIterator it(mat, col); it; ++it) {
            x[count]  = (c_float)it.value();
            ri[count] = (c_int)it.row();
            count++;
        }
    }
    p[n] = count;
    return csc_matrix(m, n, nnz, x, ri, p);
}

NMPCResult NMPCController::solve(const NMPCRef& ref,
                                 double cur_x, double cur_y, double cur_yaw,
                                 double prev_steer_rad) {
    auto t0 = ros::WallTime::now();
    NMPCResult out;
    const int    N  = cfg_.N;
    const double dt = cfg_.Ts;
    const double L  = cfg_.L;
    const double dmax = cfg_.max_steer_deg * M_PI / 180.0;

    if ((int)ref.px.size() != N || (int)ref.py.size() != N ||
        (int)ref.yaw.size() != N || (int)ref.v.size() != N) {
        ROS_WARN("[NMPC] ref size mismatch");
        return out;
    }

    // ── 1. 초기 추정: warm-start (shifted) ─────────────────
    Eigen::VectorXd u_star(N);
    if (has_warm_ && u_warm_.size() == N) {
        for (int k = 0; k < N - 1; ++k) u_star(k) = u_warm_(k + 1);
        u_star(N - 1) = u_warm_(N - 1);
    } else {
        u_star.setZero();
    }

    // SQP 메인 루프 ─────────────────────────────────────────
    Eigen::VectorXd du_sol = Eigen::VectorXd::Zero(N);

    for (int iter = 0; iter < cfg_.sqp_iters; ++iter) {
        // ── 2. nominal trajectory rollout (Euler) ─────────
        std::vector<Eigen::Vector3d> x_star(N + 1);
        x_star[0] << cur_x, cur_y, cur_yaw;
        for (int k = 0; k < N; ++k) {
            double v_k    = ref.v[k];
            double yaw_k  = x_star[k](2);
            double delta  = u_star(k);
            x_star[k + 1](0) = x_star[k](0) + dt * v_k * std::cos(yaw_k);
            x_star[k + 1](1) = x_star[k](1) + dt * v_k * std::sin(yaw_k);
            x_star[k + 1](2) = x_star[k](2) + dt * v_k / L * std::tan(delta);
        }

        // ── 3. Jacobian A_k(3x3), B_k(3x1) at (x*_k, u*_k) ──
        std::vector<Eigen::Matrix3d> A(N);
        std::vector<Eigen::Vector3d> B(N);
        for (int k = 0; k < N; ++k) {
            double v_k    = ref.v[k];
            double yaw_k  = x_star[k](2);
            double delta  = u_star(k);
            double cd     = std::cos(delta);
            double sec2   = 1.0 / std::max(1e-6, cd * cd);

            A[k] = Eigen::Matrix3d::Identity();
            A[k](0, 2) += dt * (-v_k * std::sin(yaw_k));
            A[k](1, 2) += dt * ( v_k * std::cos(yaw_k));
            B[k] << 0.0, 0.0, dt * v_k / L * sec2;
        }

        // ── 4. T (3N x N): du -> dx_seq (k=1..N) 민감도 ─────
        Eigen::MatrixXd T = Eigen::MatrixXd::Zero(3 * N, N);
        T.block(0, 0, 3, 1) = B[0];
        for (int k = 2; k <= N; ++k) {
            for (int j = 0; j < k - 1; ++j) {
                T.block(3 * (k - 1), j, 3, 1) = A[k - 1] * T.block(3 * (k - 2), j, 3, 1);
            }
            T.block(3 * (k - 1), k - 1, 3, 1) = B[k - 1];
        }

        // ── 5. Stack residuals: r = G du + e0, weight W ────
        // Rows: e_lat[1..N], e_yaw[1..N], delta[0..N-1], ddelta[0..N-1] = 4N
        const int rows = 4 * N;
        Eigen::MatrixXd G  = Eigen::MatrixXd::Zero(rows, N);
        Eigen::VectorXd e0 = Eigen::VectorXd::Zero(rows);
        Eigen::VectorXd w  = Eigen::VectorXd::Zero(rows);

        // e_lat_k (k=1..N) at rows [0..N-1]
        for (int k = 1; k <= N; ++k) {
            double yaw_ref = ref.yaw[k - 1];
            double n_x = -std::sin(yaw_ref);
            double n_y =  std::cos(yaw_ref);
            int row = k - 1;
            for (int j = 0; j < N; ++j) {
                double tx = T(3 * (k - 1) + 0, j);
                double ty = T(3 * (k - 1) + 1, j);
                G(row, j) = n_x * tx + n_y * ty;
            }
            e0(row) = n_x * (x_star[k](0) - ref.px[k - 1])
                    + n_y * (x_star[k](1) - ref.py[k - 1]);
            w(row) = cfg_.w_lat;
        }

        // e_yaw_k (k=1..N) at rows [N..2N-1]
        for (int k = 1; k <= N; ++k) {
            int row = N + (k - 1);
            for (int j = 0; j < N; ++j)
                G(row, j) = T(3 * (k - 1) + 2, j);
            e0(row) = wrapPi(x_star[k](2) - ref.yaw[k - 1]);
            w(row) = cfg_.w_yaw;
        }

        // delta_k (k=0..N-1) at rows [2N..3N-1]
        for (int k = 0; k < N; ++k) {
            int row = 2 * N + k;
            G(row, k) = 1.0;
            e0(row) = u_star(k);
            w(row) = cfg_.w_delta;
        }

        // ddelta_k (k=0..N-1) at rows [3N..4N-1]
        for (int k = 0; k < N; ++k) {
            int row = 3 * N + k;
            G(row, k) = 1.0;
            if (k == 0) {
                e0(row) = u_star(0) - prev_steer_rad;
            } else {
                G(row, k - 1) = -1.0;
                e0(row) = u_star(k) - u_star(k - 1);
            }
            w(row) = cfg_.w_ddelta;
        }

        // ── 6. P = 2 G^T W G,  q = 2 G^T W e0 ───────────────
        Eigen::MatrixXd WG = w.asDiagonal() * G;
        Eigen::MatrixXd P_dense = 2.0 * (G.transpose() * WG);
        Eigen::VectorXd q_vec   = 2.0 * (G.transpose() * (w.cwiseProduct(e0)));

        // 대각 정규화 (수치 안정)
        for (int i = 0; i < N; ++i) P_dense(i, i) += 1e-8;

        // OSQP는 P 상삼각만 받음
        Eigen::SparseMatrix<double> P_sp(N, N);
        P_sp.reserve(Eigen::VectorXi::Constant(N, N));
        for (int j = 0; j < N; ++j)
            for (int i = 0; i <= j; ++i)
                if (std::abs(P_dense(i, j)) > 1e-12)
                    P_sp.insert(i, j) = P_dense(i, j);
        P_sp.makeCompressed();

        // ── 7. 박스 제약: -dmax - u*_k <= du_k <= dmax - u*_k ─
        Eigen::SparseMatrix<double> A_cons(N, N);
        A_cons.reserve(Eigen::VectorXi::Constant(N, 1));
        for (int i = 0; i < N; ++i) A_cons.insert(i, i) = 1.0;
        A_cons.makeCompressed();

        Eigen::VectorXd l_cons(N), u_cons(N);
        for (int k = 0; k < N; ++k) {
            l_cons(k) = -dmax - u_star(k);
            u_cons(k) =  dmax - u_star(k);
        }

        // ── 8. OSQP 풀이 ─────────────────────────────────────
        std::vector<c_float> q_c(N), l_c(N), u_c(N);
        for (int i = 0; i < N; ++i) {
            q_c[i] = (c_float)q_vec(i);
            l_c[i] = (c_float)l_cons(i);
            u_c[i] = (c_float)u_cons(i);
        }

        if (work_ && prev_n_ != N) {
            osqp_cleanup(work_); work_ = nullptr;
        }

        if (!work_) {
            OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));
            data->n = N; data->m = N;
            data->P = eigenToCsc(P_sp);
            data->q = q_c.data();
            data->A = eigenToCsc(A_cons);
            data->l = l_c.data(); data->u = u_c.data();
            osqp_setup(&work_, data, settings_);
            prev_n_ = N;
            c_free(data->P->x); c_free(data->P->i); c_free(data->P->p); c_free(data->P);
            c_free(data->A->x); c_free(data->A->i); c_free(data->A->p); c_free(data->A);
            c_free(data);
        } else {
            // 값만 update
            int P_nnz = P_sp.nonZeros();
            std::vector<c_float> Pv(P_nnz);
            int cnt = 0;
            for (int col = 0; col < P_sp.cols(); ++col)
                for (Eigen::SparseMatrix<double>::InnerIterator it(P_sp, col); it; ++it)
                    Pv[cnt++] = (c_float)it.value();
            osqp_update_P(work_, Pv.data(), OSQP_NULL, (c_int)P_nnz);
            osqp_update_lin_cost(work_, q_c.data());
            osqp_update_bounds(work_, l_c.data(), u_c.data());
        }

        osqp_solve(work_);
        bool ok = (work_->info->status_val == OSQP_SOLVED ||
                   work_->info->status_val == OSQP_SOLVED_INACCURATE);
        if (!ok) {
            out.success = false;
            return out;
        }

        for (int i = 0; i < N; ++i) du_sol(i) = work_->solution->x[i];

        // SQP step (필요시 line search 추가 가능, 지금은 step=1)
        u_star += du_sol;
        // box clip (수치 오차 방지)
        for (int k = 0; k < N; ++k)
            u_star(k) = std::clamp(u_star(k), -dmax, dmax);
    }

    // ── 9. 결과 패키징 ─────────────────────────────────────
    out.success = true;
    out.steer_rad = u_star(0);

    // 1-스텝 예측 오차 (디버깅용)
    {
        double yaw0 = cur_yaw;
        double v0   = ref.v[0];
        double px1  = cur_x + dt * v0 * std::cos(yaw0);
        double py1  = cur_y + dt * v0 * std::sin(yaw0);
        double yaw1 = yaw0 + dt * v0 / L * std::tan(u_star(0));
        double n_x = -std::sin(ref.yaw[0]);
        double n_y =  std::cos(ref.yaw[0]);
        out.e_lat_pred = n_x * (px1 - ref.px[0]) + n_y * (py1 - ref.py[0]);
        out.e_yaw_pred = wrapPi(yaw1 - ref.yaw[0]);
    }

    // 최종 rollout (대시보드/로그용)
    out.traj_x.resize(N + 1);
    out.traj_y.resize(N + 1);
    out.traj_x[0] = cur_x; out.traj_y[0] = cur_y;
    {
        double xk = cur_x, yk = cur_y, yk_yaw = cur_yaw;
        for (int k = 0; k < N; ++k) {
            double v_k = ref.v[k];
            xk += dt * v_k * std::cos(yk_yaw);
            yk += dt * v_k * std::sin(yk_yaw);
            yk_yaw += dt * v_k / L * std::tan(u_star(k));
            out.traj_x[k + 1] = xk;
            out.traj_y[k + 1] = yk;
        }
    }

    // warm-start 저장
    u_warm_   = u_star;
    has_warm_ = true;

    auto t1 = ros::WallTime::now();
    out.solve_ms = (t1 - t0).toSec() * 1000.0;
    return out;
}

}  // namespace moraimpc
