#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp.h>
#include <vector>
#include "moraimpc/ltv_types.hpp"

namespace moraimpc {

class LTVSolver {
public:
    LTVSolver(const LTVMPCConfig& cfg);
    ~LTVSolver();
    bool solve(const Eigen::SparseMatrix<double>& P,
               const Eigen::VectorXd& q,
               const Eigen::SparseMatrix<double>& A,
               const Eigen::VectorXd& l,
               const Eigen::VectorXd& u,
               Eigen::VectorXd& solution);

private:
    LTVMPCConfig cfg_;
    OSQPWorkspace* work_ = nullptr;
    OSQPSettings* settings_ = nullptr;
    int prev_A_rows_ = -1;   // A 크기 변경 감지용

    csc* eigenToCsc(const Eigen::SparseMatrix<double>& mat);
};

} // namespace moraimpc
