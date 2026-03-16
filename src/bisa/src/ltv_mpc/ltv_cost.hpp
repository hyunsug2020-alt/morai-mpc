#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "ltv_types.hpp"

namespace bisa {

class LTVCost {
 public:
  explicit LTVCost(const LTVMPCConfig& cfg);
  void setConfig(const LTVMPCConfig& cfg);

  void buildStateInputWeights(Eigen::MatrixXd& Q_bar, Eigen::MatrixXd& R_bar) const;

  void buildQPObjective(const Eigen::VectorXd& x0, const Eigen::VectorXd& z_bar,
                        const Eigen::MatrixXd& A_bar, const Eigen::MatrixXd& B_bar,
                        const Eigen::MatrixXd& E_bar, const Eigen::MatrixXd& Q_bar,
                        const Eigen::MatrixXd& R_bar,
                        Eigen::SparseMatrix<double>& P,
                        Eigen::VectorXd& q) const;

 private:
  LTVMPCConfig cfg_;
};

}  // namespace bisa

