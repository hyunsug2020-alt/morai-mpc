#ifndef BISA_MPC_PATH_TRACKER_CPP_HPP_
#define BISA_MPC_PATH_TRACKER_CPP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include "bisa/msg/mpc_performance.hpp"
#include "bisa/mpc_controller_cpp.hpp"
#include "ltv_mpc/ltv_mpc.hpp"
#include <memory>
#include <optional>

namespace bisa {

/**
 * @brief LTV-MPC 기반 경로 추종 노드
 * 
 * 논문: "Lateral Vehicle Trajectory Optimization Using Constrained Linear Time-Varying MPC"
 * (Gutjahr et al., 2017)
 */
class MPCPathTrackerCpp : public rclcpp::Node {
public:
    MPCPathTrackerCpp();

private:
    int get_effective_horizon() const;
    void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void control_loop();
    void publish_control(double v, double w);
    void publish_predicted_path(const std::vector<std::array<double, 3>>& traj);
    void publish_performance(double model_time_us, double solver_time_us,
                             double total_time_us, int solver_iterations,
                             const std::string& solver_status);
    void update_controller_params();
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter>& params);
    
    std::unique_ptr<LTVMPC> ltv_controller_;
    std::unique_ptr<MPCControllerCpp> legacy_controller_;
    std::vector<geometry_msgs::msg::PoseStamped> local_path_;
    std::optional<geometry_msgs::msg::Pose> current_pose_;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_raw_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pred_pub_;
    rclcpp::Publisher<bisa::msg::MPCPerformance>::SharedPtr perf_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Time last_log_time_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    double max_omega_abs_ = 1.2;
    double max_omega_rate_ = 6.0;
    double max_v_rate_ = 3.0;
    double prev_v_cmd_ = 0.0;
    double prev_w_cmd_ = 0.0;
    rclcpp::Time prev_cmd_time_;
    bool cmd_initialized_ = false;
    // Sigmoid velocity smoother
    double v_sig_state_  = 0.0;   // smoothed velocity state
    double sig_tau_up_   = 0.40;  // acceleration time constant (s)
    double sig_tau_down_ = 0.20;  // deceleration time constant (s)
    int effective_horizon_ = 20;
    bool has_local_path_anchor_ = false;
    double local_path_anchor_x_ = 0.0;
    double local_path_anchor_y_ = 0.0;
    double path_reset_distance_threshold_ = 1.0;
    double path_hold_distance_gain_ = 0.8;
    double path_hold_heading_gain_ = 0.7;
    double path_hold_max_omega_ = 1.1;
    int path_hold_lookahead_index_ = 10;
    double path_hold_recovery_distance_ = 0.45;
    double path_hold_heading_error_gain_ = 1.4;
    double path_hold_cross_track_gain_ = 1.1;
    double path_hold_recovery_blend_ = 0.75;
    bool curve_speed_enable_ = true;
    double curve_speed_heading_threshold_ = 0.22;
    double curve_speed_reduction_gain_ = 0.58;
    double curve_speed_min_ratio_ = 0.45;
    double overshoot_guard_distance_ = 0.35;
    double overshoot_reverse_damping_ = 0.75;
    bool oscillation_guard_enable_ = true;
    double oscillation_guard_cte_deadband_ = 0.12;
    double oscillation_guard_heading_deadband_ = 0.24;
    double oscillation_guard_reverse_damping_ = 0.82;
    bool adaptive_corner_mode_enable_ = true;
    double adaptive_near_cte_thresh_ = 0.10;
    double adaptive_near_heading_thresh_ = 0.12;
    double adaptive_near_omega_damping_ = 0.55;
    double adaptive_near_omega_rate_scale_ = 0.45;
    double adaptive_near_v_scale_ = 0.95;
    bool off_path_recovery_enable_ = true;
    double off_path_recovery_distance_ = 0.90;
    double off_path_recovery_exit_distance_ = 0.45;
    double off_path_recovery_speed_ = 0.18;
    double off_path_recovery_heading_gain_ = 2.4;
    double off_path_recovery_cte_gain_ = 1.6;
    double off_path_recovery_max_omega_ = 1.6;
    bool off_path_recovery_latched_ = false;
    bool has_prev_errors_ = false;
    double prev_signed_cte_ = 0.0;
    double prev_heading_error_ = 0.0;
    
    // CAV ID
    int target_cav_id_;
    bool use_ltv_mpc_{true};
    bool publish_accel_cmd_{true};
};

}

#endif
