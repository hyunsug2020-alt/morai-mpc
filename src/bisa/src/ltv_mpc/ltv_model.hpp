#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

#include "ltv_types.hpp"

namespace bisa {

class LTVModel {
 public:
  using MatA = Eigen::Matrix<double, kLTVNx, kLTVNx>;
  using VecB = Eigen::Matrix<double, kLTVNx, kLTVNu>;
  using VecE = Eigen::Matrix<double, kLTVNx, 1>;
  using MatC = Eigen::Matrix<double, kLTVNy, kLTVNx>;

  explicit LTVModel(const LTVMPCConfig& cfg);
  void setConfig(const LTVMPCConfig& cfg);

  void buildReferenceProfiles(
      const geometry_msgs::msg::Pose& ego_pose,
      const std::vector<geometry_msgs::msg::PoseStamped>& path,
      double current_kappa,
      Eigen::VectorXd& x0,
      Eigen::VectorXd& z_bar,
      Eigen::VectorXd& v_bar,
      std::vector<double>& theta_r_seq,
      std::vector<double>& kappa_r_seq) const;

  void buildContinuousMatrices(double v_k, MatA& Ac, VecB& Bc, VecE& Ec) const;

  void discretizeExact(const MatA& Ac, const VecB& Bc, const VecE& Ec, double Ts,
                       MatA& Ad, VecB& Bd, VecE& Ed) const;

  void buildOutputMatrix(MatC& C) const;

  void buildBatchDynamics(const Eigen::VectorXd& v_bar, Eigen::MatrixXd& A_bar,
                          Eigen::MatrixXd& B_bar, Eigen::MatrixXd& E_bar,
                          Eigen::MatrixXd& C_bar) const;

 private:
  LTVMPCConfig cfg_;

  int findClosestIndex(const geometry_msgs::msg::Pose& ego_pose,
                       const std::vector<geometry_msgs::msg::PoseStamped>& path) const;

  double quatToYaw(const geometry_msgs::msg::Quaternion& q) const;
  double curvature3pt(double x1, double y1, double x2, double y2, double x3,
                      double y3) const;

  double referenceYawAt(const std::vector<geometry_msgs::msg::PoseStamped>& path,
                        int idx) const;

  static double wrapAngle(double a);
};

}  // namespace bisa

