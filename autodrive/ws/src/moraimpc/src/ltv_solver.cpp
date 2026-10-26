#include "moraimpc/ltv_solver.hpp"
#include <cmath>
#include <iostream>

namespace moraimpc {

LTVSolver::LTVSolver(const LTVMPCConfig& cfg) : cfg_(cfg) {
    settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;
    settings_->max_iter = 4000;
    settings_->eps_abs = 1e-4;
    settings_->eps_rel = 1e-4;
    // Determinism: this OSQP build defines PROFILING, where adaptive_rho_interval = 0 means
    // "derive the rho-update interval from the measured setup TIME". Iterates (and results at
    // the 1e-9 level) then depend on wall-clock timing, which made same-binary equivalence
    // reruns disagree. Use OSQP's own non-profiling rule instead: every
    // ADAPTIVE_RHO_MULTIPLE_TERMINATION * check_termination iterations (4 * 25 = 100).
    settings_->adaptive_rho_interval = ADAPTIVE_RHO_MULTIPLE_TERMINATION * settings_->check_termination;
}

LTVSolver::~LTVSolver() {
    if (work_) osqp_cleanup(work_);
    if (settings_) c_free(settings_);
}

void LTVSolver::invalidate() {
    if (work_) osqp_cleanup(work_);
    work_ = nullptr;
    prev_A_rows_ = -1;
    P_outer_.clear(); P_inner_.clear(); A_outer_.clear(); A_inner_.clear();
}

csc* LTVSolver::eigenToCsc(const Eigen::SparseMatrix<double>& mat) {
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

    return csc_matrix(m, n, nnz, x, ri, p);  // m=rows, n=cols (수정)
}

bool LTVSolver::solve(const Eigen::SparseMatrix<double>& P,
                      const Eigen::VectorXd& q,
                      const Eigen::SparseMatrix<double>& A,
                      const Eigen::VectorXd& l,
                      const Eigen::VectorXd& u,
                      Eigen::VectorXd& solution) {
    last_error_.clear();
    last_status_ = 0;
    auto fail = [&](const std::string& why) {
        last_error_ = why;
        invalidate();          // never keep a workspace whose data may be stale/corrupt
        return false;
    };

    // ── input validation (dimensions, finiteness, bound ordering) ─────────
    const int n = static_cast<int>(P.cols());
    const int m = static_cast<int>(A.rows());
    // structural checks are always on (CSC conversion relies on them)
    if (n <= 0 || P.rows() != n) return fail("invalid-input:P-dims");
    if (q.size() != n) return fail("invalid-input:q-dims");
    if (A.cols() != n) return fail("invalid-input:A-dims");
    if (l.size() != m || u.size() != m) return fail("invalid-input:bound-dims");
    if (validate_inputs_) {
        for (int col = 0; col < P.outerSize(); ++col)
            for (Eigen::SparseMatrix<double>::InnerIterator it(P, col); it; ++it) {
                if (!std::isfinite(it.value())) return fail("invalid-input:P-nonfinite");
                if (it.row() > it.col()) return fail("invalid-input:P-not-upper");
                // necessary PSD condition (the MPC Hessian is 2(B'QB+R) by construction)
                if (it.row() == it.col() && it.value() < 0.0) return fail("invalid-input:P-diag-negative");
            }
        for (int col = 0; col < A.outerSize(); ++col)
            for (Eigen::SparseMatrix<double>::InnerIterator it(A, col); it; ++it)
                if (!std::isfinite(it.value())) return fail("invalid-input:A-nonfinite");
        if (!q.allFinite()) return fail("invalid-input:q-nonfinite");
        for (int i = 0; i < m; ++i) {
            if (std::isnan(l[i]) || std::isnan(u[i])) return fail("invalid-input:bound-nan");
            if (l[i] > u[i]) return fail("invalid-input:bound-order");
        }
    }

    // q 벡터 변환
    std::vector<c_float> q_vec(q.size());
    for (int i = 0; i < (int)q.size(); ++i) q_vec[i] = (c_float)q[i];

    // P 값 추출 (CSC 열우선 순서, 상삼각)
    int P_nnz = P.nonZeros();
    std::vector<c_float> P_vals(P_nnz);
    int cnt = 0;
    for (int col = 0; col < P.cols(); ++col)
        for (Eigen::SparseMatrix<double>::InnerIterator it(P, col); it; ++it)
            P_vals[cnt++] = (c_float)it.value();

    // A 크기가 변하면 work_ 재생성 (kappa constraint 추가 시 N→2N rows)
    if (work_ && A.rows() != prev_A_rows_) invalidate();
    // Sparsity pattern change -> new setup (value-only updates would corrupt
    // the workspace and produce NaN on every later solve).
    Eigen::SparseMatrix<double> P_pat = P;  P_pat.makeCompressed();
    Eigen::SparseMatrix<double> A_pat = A;  A_pat.makeCompressed();
    auto pattern = [](const Eigen::SparseMatrix<double>& M, std::vector<int>& outer, std::vector<int>& inner) {
        outer.assign(M.outerIndexPtr(), M.outerIndexPtr() + M.outerSize() + 1);
        inner.assign(M.innerIndexPtr(), M.innerIndexPtr() + M.nonZeros());
    };
    std::vector<int> po, pi, ao, ai;
    pattern(P_pat, po, pi);
    pattern(A_pat, ao, ai);
    if (work_ && (po != P_outer_ || pi != P_inner_ || ao != A_outer_ || ai != A_inner_)) invalidate();

    std::vector<c_float> l_vec(l.size()), u_vec(u.size());
    for (int i = 0; i < (int)l.size(); ++i) {
        l_vec[i] = (c_float)l[i];
        u_vec[i] = (c_float)u[i];
    }

    if (!work_) {
        // 첫 호출 (또는 재생성): 전체 setup
        OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));
        data->n = P.cols();
        data->m = A.rows();
        data->P = eigenToCsc(P);
        data->q = q_vec.data();
        data->A = eigenToCsc(A);
        data->l = l_vec.data();
        data->u = u_vec.data();

        OSQPWorkspace* w = nullptr;
        const c_int rc = osqp_setup(&w, data, settings_);
        c_free(data->P->x); c_free(data->P->i); c_free(data->P->p); c_free(data->P);
        c_free(data->A->x); c_free(data->A->i); c_free(data->A->p); c_free(data->A);
        c_free(data);
        if (rc != 0 || !w) {
            if (w) osqp_cleanup(w);
            return fail("setup:" + std::to_string((long)rc));
        }
        work_ = w;
        ++setup_count_;
        prev_A_rows_ = A.rows();
        P_outer_ = po; P_inner_ = pi; A_outer_ = ao; A_inner_ = ai;
    } else {
        // 이후 호출: P, q, A 값, bounds 업데이트 (warm-start). Every return code is checked.
        c_int rc = osqp_update_P(work_, P_vals.data(), OSQP_NULL, (c_int)P_nnz);
        if (rc != 0) return fail("update_P:" + std::to_string((long)rc));
        rc = osqp_update_lin_cost(work_, q_vec.data());
        if (rc != 0) return fail("update_lin_cost:" + std::to_string((long)rc));

        // A 값 업데이트 (kappa_free가 매 스텝 변함)
        int A_nnz = A_pat.nonZeros();
        std::vector<c_float> A_vals(A_nnz);
        int cnt = 0;
        for (int col = 0; col < A_pat.cols(); ++col)
            for (Eigen::SparseMatrix<double>::InnerIterator it(A_pat, col); it; ++it)
                A_vals[cnt++] = (c_float)it.value();
        rc = osqp_update_A(work_, A_vals.data(), OSQP_NULL, (c_int)A_nnz);
        if (rc != 0) return fail("update_A:" + std::to_string((long)rc));
        rc = osqp_update_bounds(work_, l_vec.data(), u_vec.data());
        if (rc != 0) return fail("update_bounds:" + std::to_string((long)rc));
    }

    const c_int src = osqp_solve(work_);
    if (src != 0) return fail("solve:" + std::to_string((long)src));
    last_status_ = (int)work_->info->status_val;

    // OSQP_SOLVED_INACCURATE도 허용 (근사해 사용)
    bool success = (work_->info->status_val == OSQP_SOLVED ||
                    work_->info->status_val == OSQP_SOLVED_INACCURATE);
    if (!success) return fail("status:" + std::to_string(last_status_));
    solution.resize(P.cols());
    for (int i = 0; i < P.cols(); ++i)
        solution[i] = work_->solution->x[i];
    if (!solution.allFinite()) {
        // Never report a non-finite solution as solved; drop the workspace
        // so the next tick starts from a clean setup (no latched NaN).
        return fail("nonfinite-solution");
    }
    return true;
}

}  // namespace moraimpc
