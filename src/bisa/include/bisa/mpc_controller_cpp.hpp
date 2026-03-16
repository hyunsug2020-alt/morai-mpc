#ifndef BISA_MPC_CONTROLLER_CPP_HPP_
#define BISA_MPC_CONTROLLER_CPP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp/osqp.h>
#include <vector>
#include <array>
#include <cmath>
#include <map>

namespace bisa {

/**
 * @brief LTV-MPC 파라미터 (논문 정확히 따름)
 * 
 * 논문: "Lateral Vehicle Trajectory Optimization Using Constrained Linear Time-Varying MPC"
 * Gutjahr et al., IEEE Trans. ITS, 2017
 */
struct LTVMPCParams {
    // === 예측 파라미터 ===
    int N = 20;                    // 예측 구간 (Prediction horizon)
    double Ts = 0.2;               // 이산화 시간 간격 [s]
    int NS = 4;                    // Soft constraint 구간
    
    // === 차량 파라미터 ===
    double l = 0.33;               // 축간 거리 [m] (config.ini 기준: 0.17+0.16)
    double vehicle_width = 0.15;   // 차량 폭 [m]
    
    // === Equation (9): 입력 제약 ===
    double u_min = -2.0;          // 최소 곡률 변화율 [1/ms]
    double u_max = 2.0;           // 최대 곡률 변화율 [1/ms]
    
    // === 곡률 제약 ===
    double kappa_min_delta = -0.25;  // 최소 곡률 (조향) [1/m]
    double kappa_max_delta = 0.25;   // 최대 곡률 (조향) [1/m]
    
    // === Equation (10): 비용 함수 가중치 ===
    double wd = 10.0;              // 경로 이탈 패널티
    double wtheta = 5.0;           // 방향 오차 패널티
    double wkappa = 2.0;           // 곡률 패널티
    double wu = 0.5;               // 조향 가속도 패널티
    
    // === Equation (21): Slack variable 가중치 ===
    double k1_upper = 1e5;         // 선형 패널티 (상한)
    double k1_lower = 1e5;         // 선형 패널티 (하한)
    double k2_upper = 1e6;         // 이차 패널티 (상한)
    double k2_lower = 1e6;         // 이차 패널티 (하한)
    
    // === 마찰 파라미터 (Kamm's circle) ===
    double mu = 0.8;               // 타이어-노면 마찰계수
    double g = 9.81;               // 중력가속도 [m/s^2]
    
    // === 속도 제한 ===
    double max_velocity = 3.0;     // 최대 속도 [m/s]
    double min_velocity = 0.5;     // 최소 속도 [m/s]
};

struct ControlOutput {
    double velocity;
    double angular_velocity;
    std::vector<std::array<double, 3>> predicted_trajectory;
};

struct CollisionConstraints {
    std::map<std::string, double> bounds;  // "d1_min_0", "d1_max_0", ...
};

/**
 * @brief Linear Time-Varying MPC Controller (논문 정확 구현)
 */
class MPCControllerCpp {
public:
    MPCControllerCpp();
    ~MPCControllerCpp();
    
    void update_parameters(const LTVMPCParams& params);
    void reset_state();
    
    ControlOutput compute_control(
        const geometry_msgs::msg::Pose& current_pose,
        const std::vector<geometry_msgs::msg::PoseStamped>& local_path,
        const CollisionConstraints& constraints = CollisionConstraints()
    );

private:
    LTVMPCParams params_;
    double current_kappa_ = 0.0;  // 현재 곡률 (상태 유지)
    
    // ===================================================================
    // Equation (2): 연속시간 시스템 행렬
    // ===================================================================
    void compute_system_matrices_continuous(
        double v,
        Eigen::MatrixXd& Ac,
        Eigen::VectorXd& Bc,
        Eigen::VectorXd& Ec
    );
    
    // ===================================================================
    // Equation (4): 출력 행렬 (3원 차량 근사)
    // ===================================================================
    void compute_output_matrix(Eigen::MatrixXd& C);
    
    // ===================================================================
    // Equation (5): 이산화 (Exact discretization)
    // ===================================================================
    void discretize_system(
        const Eigen::MatrixXd& Ac,
        const Eigen::VectorXd& Bc,
        const Eigen::VectorXd& Ec,
        double Ts,
        Eigen::MatrixXd& A,
        Eigen::MatrixXd& B,
        Eigen::MatrixXd& E
    );
    
    // ===================================================================
    // Equation (16-17): 배치 정식화 (Batch formulation)
    // ===================================================================
    void build_prediction_matrices(
        const std::vector<double>& v_profile,
        Eigen::MatrixXd& A_bar,
        Eigen::MatrixXd& B_bar,
        Eigen::MatrixXd& E_bar,
        Eigen::MatrixXd& C_bar
    );
    
    // ===================================================================
    // Equation (11), (18): 비용 함수 행렬
    // ===================================================================
    void compute_cost_matrices(
        const std::vector<double>& v_profile,
        Eigen::MatrixXd& Q_bar,
        Eigen::MatrixXd& R_bar
    );
    
    // ===================================================================
    // Equation (23): QP 문제 구성
    // ===================================================================
    bool formulate_qp(
        const Eigen::VectorXd& x0,
        const Eigen::VectorXd& z,
        const std::vector<double>& v_profile,
        const CollisionConstraints& constraints,
        Eigen::SparseMatrix<double>& P,
        Eigen::VectorXd& q,
        Eigen::SparseMatrix<double>& A_qp,
        Eigen::VectorXd& l,
        Eigen::VectorXd& u
    );
    
    // ===================================================================
    // OSQP로 QP 문제 해결
    // ===================================================================
    bool solve_qp_osqp(
        const Eigen::SparseMatrix<double>& P,
        const Eigen::VectorXd& q,
        const Eigen::SparseMatrix<double>& A,
        const Eigen::VectorXd& l,
        const Eigen::VectorXd& u,
        Eigen::VectorXd& solution
    );
    
    // ===================================================================
    // 보조 함수들
    // ===================================================================
    
    // Equation (7): 상태 벡터 계산 [dr, theta, kappa, theta_r, kappa_r]
    Eigen::VectorXd compute_state_vector(
        const geometry_msgs::msg::Pose& current_pose,
        const std::vector<geometry_msgs::msg::PoseStamped>& reference_path
    );
    
    // Equation (7): 기준 곡률 미분 z = dκr/dt
    Eigen::VectorXd compute_disturbance_signal(
        const std::vector<geometry_msgs::msg::PoseStamped>& reference_path
    );
    
    // 속도 프로파일 생성 (곡률 기반)
    std::vector<double> generate_velocity_profile(
        const std::vector<geometry_msgs::msg::PoseStamped>& reference_path
    );
    
    // Kamm's circle 기반 곡률 제한
    void compute_curvature_limits(
        double v,
        double& kappa_min,
        double& kappa_max
    );
    
    // 3점 곡률 계산
    double compute_curvature_3points(
        double x1, double y1,
        double x2, double y2,
        double x3, double y3
    );
    
    // Quaternion to Yaw (올바른 변환)
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q);
    
    // 가장 가까운 경로점 찾기
    int find_closest_waypoint(
        const geometry_msgs::msg::Pose& current_pose,
        const std::vector<geometry_msgs::msg::PoseStamped>& path
    );
    
    // Dense to Sparse 변환 (OSQP 입력용)
    Eigen::SparseMatrix<double> dense_to_sparse(const Eigen::MatrixXd& dense);
};

}  // namespace bisa

#endif  // BISA_MPC_CONTROLLER_CPP_HPP_
