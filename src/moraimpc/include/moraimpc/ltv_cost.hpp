#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "moraimpc/ltv_types.hpp"

namespace moraimpc {

class LTVCost {
public:
    LTVCost(const LTVMPCConfig& cfg);
    void buildQPObjective(const Eigen::VectorXd& x0, 
                          const Eigen::VectorXd& z_bar,
                          const Eigen::MatrixXd& A_bar, 
                          const Eigen::MatrixXd& B_bar, 
                          const Eigen::MatrixXd& E_bar,
                          const std::vector<double>& v_profile,
                          Eigen::SparseMatrix<double>& P, 
                          Eigen::VectorXd& q);private:
    LTVMPCConfig cfg_;
};

} // namespace moraimpc
