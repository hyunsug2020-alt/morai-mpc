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
    int n = mat.cols();
    int nnz = mat.nonZeros();
    
    // Allocate space for CSC matrix
    c_float* x = (c_float*)c_malloc(nnz * sizeof(c_float));
    c_int* i = (c_int*)c_malloc(nnz * sizeof(c_int));
    c_int* p = (c_int*)c_malloc((n + 1) * sizeof(c_int));

    int count = 0;
    for (int col = 0; col < n; ++col) {
        p[col] = count;
        for (Eigen::SparseMatrix<double>::InnerIterator it(mat, col); it; ++it) {
            x[count] = (c_float)it.value();
            i[count] = (c_int)it.row();
            count++;
        }
    }
    p[n] = count;
    
    return csc_matrix(n, mat.rows(), nnz, x, i, p);
}

bool LTVSolver::solve(const Eigen::SparseMatrix<double>& P, 
                      const Eigen::VectorXd& q, 
                      const Eigen::SparseMatrix<double>& A, 
                      const Eigen::VectorXd& l, 
                      const Eigen::VectorXd& u,
                      Eigen::VectorXd& solution) {
    
    // Cleanup previous workspace
    if (work_) {
        osqp_cleanup(work_);
        work_ = nullptr;
    }

    OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));
    data->n = P.cols();
    data->m = A.rows();
    data->P = eigenToCsc(P);
    
    std::vector<c_float> q_vec(q.size());
    for(int i=0; i<q.size(); ++i) q_vec[i] = (c_float)q[i];
    data->q = q_vec.data();
    
    data->A = eigenToCsc(A);
    
    std::vector<c_float> l_vec(l.size()), u_vec(u.size());
    for(int i=0; i<l.size(); ++i) {
        l_vec[i] = (c_float)l[i];
        u_vec[i] = (c_float)u[i];
    }
    data->l = l_vec.data();
    data->u = u_vec.data();

    // Setup solver
    osqp_setup(&work_, data, settings_);
    osqp_solve(work_);

    bool success = (work_->info->status_val == OSQP_SOLVED);
    if (success) {
        solution.resize(data->n);
        for (int i = 0; i < data->n; ++i) {
            solution[i] = work_->solution->x[i];
        }
    }

    // Clean up OSQP data (must free csc matrices x, i, p manually)
    c_free(data->P->x); c_free(data->P->i); c_free(data->P->p); c_free(data->P);
    c_free(data->A->x); c_free(data->A->i); c_free(data->A->p); c_free(data->A);
    c_free(data);

    return success;
}

} // namespace moraimpc
