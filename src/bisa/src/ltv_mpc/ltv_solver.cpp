#include "ltv_solver.hpp"

namespace bisa {

LTVSolver::LTVSolver(const LTVMPCConfig& cfg) : cfg_(cfg) {}

void LTVSolver::setConfig(const LTVMPCConfig& cfg) { cfg_ = cfg; }

LTVSolver::CSCData LTVSolver::eigenToCSC(const Eigen::SparseMatrix<double>& M) const {
  CSCData csc;
  Eigen::SparseMatrix<double> C = M;
  C.makeCompressed();

  csc.data.resize(C.nonZeros());
  csc.indices.resize(C.nonZeros());
  csc.indptr.resize(C.cols() + 1);

  int idx = 0;
  for (int col = 0; col < C.outerSize(); ++col) {
    csc.indptr[col] = idx;
    for (Eigen::SparseMatrix<double>::InnerIterator it(C, col); it; ++it) {
      csc.data[idx] = static_cast<c_float>(it.value());
      csc.indices[idx] = static_cast<c_int>(it.row());
      ++idx;
    }
  }
  csc.indptr[C.cols()] = idx;
  return csc;
}

bool LTVSolver::solve(const Eigen::SparseMatrix<double>& P, const Eigen::VectorXd& q,
                      const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& l,
                      const Eigen::VectorXd& u, Eigen::VectorXd& solution) const {
  if (P.rows() == 0 || P.cols() == 0 || A.rows() == 0 || q.size() == 0) return false;

  const c_int n = static_cast<c_int>(P.rows());
  const c_int m = static_cast<c_int>(A.rows());
  if (q.size() != n || l.size() != m || u.size() != m) return false;

  const CSCData P_csc = eigenToCSC(P);
  const CSCData A_csc = eigenToCSC(A);

  std::vector<c_float> q_data(n);
  std::vector<c_float> l_data(m);
  std::vector<c_float> u_data(m);
  for (c_int i = 0; i < n; ++i) q_data[i] = static_cast<c_float>(q(i));
  for (c_int i = 0; i < m; ++i) {
    l_data[i] = static_cast<c_float>(l(i));
    u_data[i] = static_cast<c_float>(u(i));
  }

  OSQPData data;
  data.n = n;
  data.m = m;
  data.P = csc_matrix(n, n, static_cast<c_int>(P_csc.data.size()),
                      const_cast<c_float*>(P_csc.data.data()),
                      const_cast<c_int*>(P_csc.indices.data()),
                      const_cast<c_int*>(P_csc.indptr.data()));
  data.q = q_data.data();
  data.A = csc_matrix(m, n, static_cast<c_int>(A_csc.data.size()),
                      const_cast<c_float*>(A_csc.data.data()),
                      const_cast<c_int*>(A_csc.indices.data()),
                      const_cast<c_int*>(A_csc.indptr.data()));
  data.l = l_data.data();
  data.u = u_data.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = cfg_.osqp_polish ? 1 : 0;
  settings.max_iter = cfg_.osqp_max_iter;
  settings.eps_abs = cfg_.osqp_eps_abs;
  settings.eps_rel = cfg_.osqp_eps_rel;
  settings.warm_start = cfg_.osqp_warm_start ? 1 : 0;

  OSQPWorkspace* work = nullptr;
  const c_int exitflag = osqp_setup(&work, &data, &settings);
  if (exitflag != 0 || work == nullptr) return false;

  osqp_solve(work);
  const c_int status = work->info->status_val;

  // OSQP_MAX_ITER_REACHED(-2): 반복 한계 도달했지만 부분 해 사용 가능
  // → SOLVED_INACCURATE와 동일하게 허용해서 FAILED 방지
  const bool ok = (status == OSQP_SOLVED ||
                   status == OSQP_SOLVED_INACCURATE ||
                   status == OSQP_MAX_ITER_REACHED);

  if (ok && work->solution != nullptr) {
    solution = Eigen::VectorXd::Zero(n);
    for (c_int i = 0; i < n; ++i) solution(i) = work->solution->x[i];
  }

  osqp_cleanup(work);
  return ok && !solution.hasNaN();
}

}  // namespace bisa

