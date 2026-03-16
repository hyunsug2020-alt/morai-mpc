#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

#include "ltv_cost.hpp"
#include "ltv_model.hpp"
#include "ltv_solver.hpp"
#include "ltv_types.hpp"

namespace bisa {

class LTVMPC {
 public:
  explicit LTVMPC(const LTVMPCConfig& cfg);

  void setConfig(const LTVMPCConfig& cfg);
  void reset();

  MPCCommand computeControl(
      const geometry_msgs::msg::Pose& ego_pose,
      const std::vector<geometry_msgs::msg::PoseStamped>& local_path);

 private:
  void buildInputConstraints(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l,
                             Eigen::VectorXd& u, int nvar) const;
  void buildKappaConstraints(const Eigen::VectorXd& x0,
                             const Eigen::VectorXd& z_bar,
                             const Eigen::MatrixXd& A_bar,
                             const Eigen::MatrixXd& B_bar,
                             const Eigen::MatrixXd& E_bar,
                             Eigen::SparseMatrix<double>& A,
                             Eigen::VectorXd& l,
                             Eigen::VectorXd& u,
                             int nvar) const;
  void buildLateralSlackConstraints(const Eigen::VectorXd& x0,
                                    const Eigen::VectorXd& z_bar,
                                    const Eigen::MatrixXd& A_bar,
                                    const Eigen::MatrixXd& B_bar,
                                    const Eigen::MatrixXd& E_bar,
                                    const Eigen::MatrixXd& C_bar,
                                    Eigen::SparseMatrix<double>& A,
                                    Eigen::VectorXd& l,
                                    Eigen::VectorXd& u,
                                    int nvar) const;

  LTVMPCConfig cfg_;
  LTVModel model_;
  LTVCost cost_;
  LTVSolver solver_;
  double kappa_state_ = 0.0;
  bool kappa_initialized_ = false;
};

}  // namespace bisa
