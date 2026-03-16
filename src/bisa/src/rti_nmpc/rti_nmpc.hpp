#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

#include "rti_nmpc_types.hpp"

namespace bisa {

// ============================================================
// RTI-NMPC 컨트롤러
//
// 논문: 2410.12170v1.pdf  "Implicit Discretization RTI-NMPC"
//
// 차량 모델 (운동학적 자전거 모델):
//   ẋ  = v·cos(ψ)
//   ẏ  = v·sin(ψ)
//   ψ̇  = v·κ
//   v̇  = av
//   κ̇  = a_κ
//
// 암시적 오일러 이산화:
//   x(k+1) = x(k) + Ts·f(x(k+1), u(k))
//
// RTI: 매 스텝 1회 SQP 반복
//   1) 현재 궤적 주변 선형화
//   2) QP 풀이 (OSQP)
//   3) 첫 번째 입력 적용
// ============================================================
class RTINMPCController {
 public:
  explicit RTINMPCController(const RTINMPCConfig& cfg);

  void setConfig(const RTINMPCConfig& cfg);
  void reset();

  // 메인 제어 계산
  // ego_pose: 현재 차량 자세
  // reference_path: 참조 경로 (PoseStamped 목록)
  // current_v: 현재 속도 [m/s] (음수=후진)
  RTINMPCCommand computeControl(
      const geometry_msgs::msg::Pose& ego_pose,
      const std::vector<geometry_msgs::msg::PoseStamped>& reference_path,
      double current_v = 0.0);

 private:
  // --------------------------------------------------------
  // 모델 함수
  // --------------------------------------------------------

  // 운동학 f(x, u) 계산
  Eigen::VectorXd dynamics(const Eigen::VectorXd& x,
                           const Eigen::VectorXd& u) const;

  // 암시적 오일러: x_next = x + Ts*f(x_next, u)
  // Newton 반복으로 x_next 풀이
  Eigen::VectorXd implicitEulerStep(const Eigen::VectorXd& x_k,
                                    const Eigen::VectorXd& u_k) const;

  // ∂f/∂x 야코비안 (Jacobian)
  Eigen::MatrixXd jacobianFx(const Eigen::VectorXd& x,
                              const Eigen::VectorXd& u) const;

  // ∂f/∂u 야코비안
  Eigen::MatrixXd jacobianFu(const Eigen::VectorXd& x,
                              const Eigen::VectorXd& u) const;

  // 이산화 후 선형화: A_d, B_d, d (이산화 Jacobian + linearization 오차)
  void discretizeLinearize(const Eigen::VectorXd& x_k,
                           const Eigen::VectorXd& u_k,
                           Eigen::MatrixXd& A_d,
                           Eigen::MatrixXd& B_d,
                           Eigen::VectorXd& d_k) const;

  // --------------------------------------------------------
  // 참조 경로 처리
  // --------------------------------------------------------

  // 현재 자세로부터 상태 벡터 구성
  Eigen::VectorXd buildStateVector(const geometry_msgs::msg::Pose& pose,
                                   double v) const;

  // 경로에서 참조 상태 시퀀스 추출
  bool buildReferenceSequence(
      const Eigen::VectorXd& x0,
      const std::vector<geometry_msgs::msg::PoseStamped>& path,
      std::vector<Eigen::VectorXd>& x_ref,
      std::vector<double>& v_ref) const;

  // 가장 가까운 경로점 인덱스 탐색
  int findClosestWaypoint(
      const Eigen::VectorXd& x0,
      const std::vector<geometry_msgs::msg::PoseStamped>& path,
      int search_start = 0) const;

  // --------------------------------------------------------
  // QP 구성 및 풀이
  // --------------------------------------------------------

  // 배치 QP 구성
  // min 0.5 * U^T * H * U + f^T * U
  // s.t. lb <= C * U <= ub
  bool buildAndSolveQP(
      const Eigen::VectorXd& x0,
      const std::vector<Eigen::MatrixXd>& A_seq,
      const std::vector<Eigen::MatrixXd>& B_seq,
      const std::vector<Eigen::VectorXd>& d_seq,
      const std::vector<Eigen::VectorXd>& x_ref,
      const std::vector<Eigen::VectorXd>& u_warm,
      Eigen::VectorXd& u_opt) const;

  // OSQP 인터페이스
  bool solveQPOSQP(const Eigen::SparseMatrix<double>& H,
                   const Eigen::VectorXd& f,
                   const Eigen::SparseMatrix<double>& A_cons,
                   const Eigen::VectorXd& lb,
                   const Eigen::VectorXd& ub,
                   Eigen::VectorXd& solution) const;

  // --------------------------------------------------------
  // 내부 상태
  // --------------------------------------------------------
  RTINMPCConfig cfg_;

  // warm-start 용 이전 입력 시퀀스
  std::vector<Eigen::VectorXd> u_warm_;

  // 내부 곡률 상태 (적분)
  double kappa_state_ = 0.0;
  double v_state_     = 0.0;
  bool   initialized_ = false;

  // 마지막으로 탐색한 경로 인덱스 (중복 탐색 방지)
  int last_closest_idx_ = 0;
};

}  // namespace bisa
