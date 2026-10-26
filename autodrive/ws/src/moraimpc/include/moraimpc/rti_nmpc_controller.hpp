#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <osqp.h>
#include <vector>

#include "moraimpc/rti_nmpc_types.hpp"

namespace moraimpc {

// RTI-NMPC 컨트롤러 (운동학적 자전거 모델)
//
// 논문: 2410.12170v1.pdf "Implicit Discretization RTI-NMPC"
//
// 차량 모델:
//   ẋ  = v·cos(ψ)
//   ẏ  = v·sin(ψ)
//   ψ̇  = v·κ
//   v̇  = av
//   κ̇  = a_κ
//
// 암시적 오일러: x(k+1) = x(k) + Ts·f(x(k+1), u(k))
// RTI: 매 스텝 1회 SQP → 현재 궤적 선형화 → QP(OSQP) → 첫 입력 적용
class RTINMPCController {
public:
    explicit RTINMPCController(const RTINMPCConfig& cfg);
    ~RTINMPCController();

    void setConfig(const RTINMPCConfig& cfg);
    void reset();

    // LTV→NMPC 인계: 외부에서 kappa_state_ 강제 설정 (warm-start)
    void setInitialKappa(double kappa);

    // 장애물 설정 (매 tick 호출). 빈 vector 시 회피 비활성
    void setObstacles(const std::vector<RTINMPCObstacle>& obs);

    // 메인 제어 계산
    // ego_pose      : 현재 차량 자세 (geometry_msgs/Pose)
    // reference_path: 참조 경로 (PoseStamped 목록)
    // current_v     : 현재 속도 [m/s] (음수=후진)
    RTINMPCCommand computeControl(
        const geometry_msgs::Pose& ego_pose,
        const std::vector<geometry_msgs::PoseStamped>& reference_path,
        double current_v = 0.0);

private:
    // 운동학 f(x, u)
    Eigen::VectorXd dynamics(const Eigen::VectorXd& x,
                              const Eigen::VectorXd& u) const;

    // 암시적 오일러: Newton 반복으로 x_next 풀이
    Eigen::VectorXd implicitEulerStep(const Eigen::VectorXd& x_k,
                                      const Eigen::VectorXd& u_k) const;

    // ∂f/∂x 야코비안
    Eigen::MatrixXd jacobianFx(const Eigen::VectorXd& x,
                                const Eigen::VectorXd& u) const;

    // ∂f/∂u 야코비안
    Eigen::MatrixXd jacobianFu(const Eigen::VectorXd& x,
                                const Eigen::VectorXd& u) const;

    // 이산화 + 선형화: A_d, B_d, d_k 반환
    void discretizeLinearize(const Eigen::VectorXd& x_k,
                             const Eigen::VectorXd& u_k,
                             Eigen::MatrixXd& A_d,
                             Eigen::MatrixXd& B_d,
                             Eigen::VectorXd& d_k) const;

    // 현재 자세에서 상태 벡터 구성
    Eigen::VectorXd buildStateVector(const geometry_msgs::Pose& pose,
                                     double v) const;

    // 경로에서 참조 상태 시퀀스 추출
    bool buildReferenceSequence(
        const Eigen::VectorXd& x0,
        const std::vector<geometry_msgs::PoseStamped>& path,
        std::vector<Eigen::VectorXd>& x_ref) const;

    // 가장 가까운 경로점 인덱스 탐색
    int findClosestWaypoint(
        const Eigen::VectorXd& x0,
        const std::vector<geometry_msgs::PoseStamped>& path,
        int search_start = 0) const;

    // 배치 QP 구성 및 OSQP 풀이
    bool buildAndSolveQP(
        const Eigen::VectorXd& x0,
        const std::vector<Eigen::MatrixXd>& A_seq,
        const std::vector<Eigen::MatrixXd>& B_seq,
        const std::vector<Eigen::VectorXd>& d_seq,
        const std::vector<Eigen::VectorXd>& x_ref,
        Eigen::VectorXd& u_opt);

    bool solveOSQP(const Eigen::SparseMatrix<double>& P,
                   const Eigen::VectorXd& q,
                   const Eigen::SparseMatrix<double>& A_cons,
                   const Eigen::VectorXd& lb,
                   const Eigen::VectorXd& ub,
                   Eigen::VectorXd& solution);

    RTINMPCConfig cfg_;
    std::vector<RTINMPCObstacle> obstacles_;   // setObstacles로 갱신

    std::vector<Eigen::VectorXd> u_warm_;   // warm-start 이전 입력 시퀀스
    double kappa_state_    = 0.0;
    double v_state_        = 0.0;
    bool   initialized_    = false;
    int    last_closest_idx_ = 0;

    OSQPWorkspace* solver_   = nullptr;
    OSQPSettings*  settings_ = nullptr;
};

}  // namespace moraimpc
