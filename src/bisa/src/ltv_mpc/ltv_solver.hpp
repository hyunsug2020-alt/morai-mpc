#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp/osqp.h>

#include <vector>

#include "ltv_types.hpp"

namespace bisa {

class LTVSolver {
 public:
  explicit LTVSolver(const LTVMPCConfig& cfg);
  void setConfig(const LTVMPCConfig& cfg);

  bool solve(const Eigen::SparseMatrix<double>& P, const Eigen::VectorXd& q,
             const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& l,
             const Eigen::VectorXd& u, Eigen::VectorXd& solution) const;

 private:
  struct CSCData {
    std::vector<c_float> data;
    std::vector<c_int> indices;
    std::vector<c_int> indptr;
  };

  CSCData eigenToCSC(const Eigen::SparseMatrix<double>& M) const;

  LTVMPCConfig cfg_;
};

}  // namespace bisa

