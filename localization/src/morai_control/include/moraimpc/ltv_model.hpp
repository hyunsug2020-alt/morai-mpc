#pragma once
#include <Eigen/Dense>
#include <vector>
#include "moraimpc/ltv_types.hpp"

namespace moraimpc {

class LTVModel {
public:
    LTVModel(const LTVMPCConfig& cfg);
    void discretize(double v, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& E);
    void buildBatchMatrices(const std::vector<double>& v_profile, 
                            Eigen::MatrixXd& A_bar, 
                            Eigen::MatrixXd& B_bar, 
                            Eigen::MatrixXd& E_bar);
private:
    LTVMPCConfig cfg_;
};

} // namespace moraimpc
