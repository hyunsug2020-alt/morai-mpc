#include "moraimpc/ltv_solver.hpp"
#include <iostream>

namespace moraimpc {

LTVSolver::LTVSolver(const LTVMPCConfig& cfg) : cfg_(cfg) {
    settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;
    settings_->max_iter = 4000;
    settings_->eps_abs = 1e-4;
    settings_->eps_rel = 1e-4;
}

LTVSolver::~LTVSolver() {
    if (work_) osqp_cleanup(work_);
    if (settings_) c_free(settings_);
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
    if (work_ && A.rows() != prev_A_rows_) {
        osqp_cleanup(work_);
        work_ = nullptr;
    }

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

        osqp_setup(&work_, data, settings_);
        prev_A_rows_ = A.rows();

        c_free(data->P->x); c_free(data->P->i); c_free(data->P->p); c_free(data->P);
        c_free(data->A->x); c_free(data->A->i); c_free(data->A->p); c_free(data->A);
        c_free(data);
    } else {
        // 이후 호출: P, q, A 값, bounds 업데이트 (warm-start)
        osqp_update_P(work_, P_vals.data(), OSQP_NULL, (c_int)P_nnz);
        osqp_update_lin_cost(work_, q_vec.data());

        // A 값 업데이트 (kappa_free가 매 스텝 변함)
        Eigen::SparseMatrix<double> A_csc = A;
        A_csc.makeCompressed();
        int A_nnz = A_csc.nonZeros();
        std::vector<c_float> A_vals(A_nnz);
        int cnt = 0;
        for (int col = 0; col < A_csc.cols(); ++col)
            for (Eigen::SparseMatrix<double>::InnerIterator it(A_csc, col); it; ++it)
                A_vals[cnt++] = (c_float)it.value();
        osqp_update_A(work_, A_vals.data(), OSQP_NULL, (c_int)A_nnz);
        osqp_update_bounds(work_, l_vec.data(), u_vec.data());
    }

    osqp_solve(work_);

    // OSQP_SOLVED_INACCURATE도 허용 (근사해 사용)
    bool success = (work_->info->status_val == OSQP_SOLVED ||
                    work_->info->status_val == OSQP_SOLVED_INACCURATE);
    if (success) {
        solution.resize(P.cols());
        for (int i = 0; i < P.cols(); ++i)
            solution[i] = work_->solution->x[i];
    }

    return success;
}

} // namespace moraimpc
