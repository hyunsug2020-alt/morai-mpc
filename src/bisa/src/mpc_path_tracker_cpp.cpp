#include "bisa/mpc_path_tracker_cpp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace bisa {

namespace {
double wrap_angle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double pose_yaw_from_quat_or_packed(const geometry_msgs::msg::Pose& pose) {
    const auto& q = pose.orientation;
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    const bool quat_component_range_ok =
        std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
        std::abs(q.x) <= 1.0 + 1e-3 && std::abs(q.y) <= 1.0 + 1e-3 &&
        std::abs(q.z) <= 1.0 + 1e-3 && std::abs(q.w) <= 1.0 + 1e-3;
    const bool use_quaternion = quat_component_range_ok && std::isfinite(n) && std::abs(n - 1.0) <= 0.05;
    if (!use_quaternion) {
        // Fallback for packed-yaw pose conventions:
        // - radians in z
        // - degrees in z
        // - [0, 2pi] wrapped radians in z
        double yaw = q.z;
        if (!std::isfinite(yaw)) return 0.0;
        if (std::abs(yaw) > 2.0 * M_PI + 0.5) {
            yaw = yaw * M_PI / 180.0;
        } else if (yaw > M_PI && yaw <= 2.0 * M_PI + 0.5) {
            yaw -= 2.0 * M_PI;
        } else if (yaw < -M_PI && yaw >= -2.0 * M_PI - 0.5) {
            yaw += 2.0 * M_PI;
        }
        return wrap_angle(yaw);
    }
    const double x = q.x / n;
    const double y = q.y / n;
    const double z = q.z / n;
    const double w = q.w / n;
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return wrap_angle(std::atan2(siny_cosp, cosy_cosp));
}
}  // namespace

MPCPathTrackerCpp::MPCPathTrackerCpp()
    : Node("mpc_path_tracker_cpp"), last_log_time_(this->now()), prev_cmd_time_(this->now()) {
    
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "LTV-MPC Path Tracker C++ - Gutjahr 2017");
    RCLCPP_INFO(this->get_logger(), "===========================================");
    
    // CAV ID 파라미터
    this->declare_parameter("target_cav_id", 1);
    this->declare_parameter("use_ltv_mpc", true);
    this->declare_parameter("publish_accel_cmd", true);
    target_cav_id_ = this->get_parameter("target_cav_id").as_int();
    use_ltv_mpc_ = this->get_parameter("use_ltv_mpc").as_bool();
    publish_accel_cmd_ = this->get_parameter("publish_accel_cmd").as_bool();
    std::string id_str = (target_cav_id_ < 10) ? 
                         "0" + std::to_string(target_cav_id_) : 
                         std::to_string(target_cav_id_);
    
    RCLCPP_INFO(this->get_logger(), "LTV-MPC Tracker for CAV_%s", id_str.c_str());
    
    // Existing parameter names (backward compatibility)
    this->declare_parameter("prediction_horizon", 20);      // N
    this->declare_parameter("time_step", 0.05);             // Ts
    this->declare_parameter("wheelbase", 0.3);              // l
    this->declare_parameter("weight_position", 10.0);       // wd
    this->declare_parameter("weight_heading", 5.0);         // wtheta
    this->declare_parameter("weight_curvature", 2.0);       // wkappa
    this->declare_parameter("weight_input", 0.5);           // wu
    this->declare_parameter("max_velocity", 3.0);
    this->declare_parameter("min_velocity", 0.5);
    this->declare_parameter("u_min", -2.0);
    this->declare_parameter("u_max", 2.0);
    this->declare_parameter("kappa_min_delta", -2.0);
    this->declare_parameter("kappa_max_delta", 2.0);

    // Legacy names from previous configs/docs (keep optional via sentinel)
    this->declare_parameter("Q_pos", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("Q_heading", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("R_v", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("R_w", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("max_accel", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("max_angular_vel", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("horizon", -1);

    // New explicit LTV-MPC names (optional override via sentinel)
    this->declare_parameter("N", -1);
    this->declare_parameter("Ts", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("w_d", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("w_theta", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("w_kappa", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("w_u", std::numeric_limits<double>::quiet_NaN());
    this->declare_parameter("ref_preview_steps", 0);
    this->declare_parameter("curvature_speed_gain", 2.5);
    this->declare_parameter("kappa_blend_alpha", 0.15);
    this->declare_parameter("max_omega_abs", 2.0);
    this->declare_parameter("max_omega_rate", 20.0);
    this->declare_parameter("max_v_rate", 10.0);
    this->declare_parameter("sigmoid_tau_accel", 0.40);
    this->declare_parameter("sigmoid_tau_decel", 0.20);
    // OSQP solver tuning (exposed so yaml can override)
    this->declare_parameter("osqp_max_iter",  4000);
    this->declare_parameter("osqp_eps_abs",   1e-4);
    this->declare_parameter("osqp_eps_rel",   1e-4);
    this->declare_parameter("osqp_polish",    false);
    this->declare_parameter("lateral_bound", -1.0);
    this->declare_parameter("w_lateral_slack_lin", 500.0);
    this->declare_parameter("w_lateral_slack_quad", 5000.0);
    this->declare_parameter("enable_path_fallback", false);
    this->declare_parameter("fallback_min_abs_omega", 0.03);
    this->declare_parameter("fallback_lookahead_index", 20);
    this->declare_parameter("fallback_max_abs_omega", 0.9);
    this->declare_parameter("fallback_blend", 0.85);
    this->declare_parameter("kappa_limit_ref_velocity", -1.0);
    this->declare_parameter("path_reset_distance_threshold", 1.0);
    this->declare_parameter("path_hold_distance_gain", 0.8);
    this->declare_parameter("path_hold_heading_gain", 0.7);
    this->declare_parameter("path_hold_max_omega", 1.1);
    this->declare_parameter("path_hold_lookahead_index", 10);
    this->declare_parameter("path_hold_recovery_distance", 0.45);
    this->declare_parameter("path_hold_heading_error_gain", 1.4);
    this->declare_parameter("path_hold_cross_track_gain", 1.1);
    this->declare_parameter("path_hold_recovery_blend", 0.75);
    this->declare_parameter("curve_speed_enable", true);
    this->declare_parameter("curve_speed_heading_threshold", 0.22);
    this->declare_parameter("curve_speed_reduction_gain", 0.58);
    this->declare_parameter("curve_speed_min_ratio", 0.45);
    this->declare_parameter("overshoot_guard_distance", 0.35);
    this->declare_parameter("overshoot_reverse_damping", 0.75);
    this->declare_parameter("oscillation_guard_enable", true);
    this->declare_parameter("oscillation_guard_cte_deadband", 0.12);
    this->declare_parameter("oscillation_guard_heading_deadband", 0.24);
    this->declare_parameter("oscillation_guard_reverse_damping", 0.82);
    this->declare_parameter("adaptive_corner_mode_enable", true);
    this->declare_parameter("adaptive_near_cte_thresh", 0.10);
    this->declare_parameter("adaptive_near_heading_thresh", 0.12);
    this->declare_parameter("adaptive_near_omega_damping", 0.55);
    this->declare_parameter("adaptive_near_omega_rate_scale", 0.45);
    this->declare_parameter("adaptive_near_v_scale", 0.95);
    this->declare_parameter("off_path_recovery_enable", true);
    this->declare_parameter("off_path_recovery_distance", 0.90);
    this->declare_parameter("off_path_recovery_exit_distance", 0.45);
    this->declare_parameter("off_path_recovery_speed", 0.18);
    this->declare_parameter("off_path_recovery_heading_gain", 2.4);
    this->declare_parameter("off_path_recovery_cte_gain", 1.6);
    this->declare_parameter("off_path_recovery_max_omega", 1.6);
    path_reset_distance_threshold_ =
        std::max(0.1, this->get_parameter("path_reset_distance_threshold").as_double());
    
    // 파라미터 콜백 등록
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MPCPathTrackerCpp::parameter_callback, this, std::placeholders::_1));
    
    // 컨트롤러 생성 및 파라미터 업데이트
    ltv_controller_ = std::make_unique<LTVMPC>(LTVMPCConfig{});
    legacy_controller_ = std::make_unique<MPCControllerCpp>();
    update_controller_params();
    
    // Topic 이름 설정 (use generic names so launch remappings work)
    std::string local_path_topic = "/local_path";
    std::string pose_topic = "/Ego_pose";
    std::string accel_topic = "/Accel";
    std::string pred_traj_topic = "/mpc_predicted_path";// Subscriber 생성
    local_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        local_path_topic, 10,
        std::bind(&MPCPathTrackerCpp::local_path_callback, this, std::placeholders::_1));
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&MPCPathTrackerCpp::pose_callback, this, std::placeholders::_1));
    
    // Publisher 생성
    auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    if (publish_accel_cmd_) {
        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>(accel_topic, qos_cmd);
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "publish_accel_cmd=false: skip /Accel publisher (gate-only output)");
    }
    std::string accel_raw_topic = "/Accel_raw";
    accel_pub_raw_ = this->create_publisher<geometry_msgs::msg::Accel>(accel_raw_topic, qos_cmd);
    pred_pub_ = this->create_publisher<nav_msgs::msg::Path>(pred_traj_topic, 10);
    perf_pub_ = this->create_publisher<bisa::msg::MPCPerformance>("/mpc_performance", 10);
    
    // 제어 루프 타이머 (20Hz - Ts=0.05s)
    // 논문에서는 Ts=0.2s이지만, 실시간 제어를 위해 더 빠른 주기 사용
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20Hz
        std::bind(&MPCPathTrackerCpp::control_loop, this));
    
    RCLCPP_INFO(this->get_logger(), "LTV-MPC Ready (20Hz control loop)");
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  - Prediction Horizon (N): %d", 
                this->get_parameter("prediction_horizon").as_int());
    RCLCPP_INFO(this->get_logger(), "  - Time Step (Ts): %.2f s", 
                this->get_parameter("time_step").as_double());
    RCLCPP_INFO(this->get_logger(), "  - Wheelbase (l): %.2f m", 
                this->get_parameter("wheelbase").as_double());
}

void MPCPathTrackerCpp::update_controller_params() {
    auto read_nan_double = [this](const char *name) -> double {
        return this->get_parameter(name).as_double();
    };

    LTVMPCConfig cfg;
    cfg.N = this->get_parameter("prediction_horizon").as_int();
    cfg.Ts = this->get_parameter("time_step").as_double();
    cfg.wheelbase = this->get_parameter("wheelbase").as_double();
    cfg.w_d = this->get_parameter("weight_position").as_double();
    cfg.w_theta = this->get_parameter("weight_heading").as_double();
    cfg.w_kappa = this->get_parameter("weight_curvature").as_double();
    cfg.w_u = this->get_parameter("weight_input").as_double();
    cfg.max_velocity = this->get_parameter("max_velocity").as_double();
    cfg.min_velocity = this->get_parameter("min_velocity").as_double();
    cfg.u_min = this->get_parameter("u_min").as_double();
    cfg.u_max = this->get_parameter("u_max").as_double();
    cfg.kappa_min = this->get_parameter("kappa_min_delta").as_double();
    cfg.kappa_max = this->get_parameter("kappa_max_delta").as_double();

    // Legacy mapping
    const int horizon_legacy = this->get_parameter("horizon").as_int();
    if (horizon_legacy > 0) cfg.N = horizon_legacy;

    const double q_pos = read_nan_double("Q_pos");
    if (std::isfinite(q_pos)) cfg.w_d = q_pos;
    const double q_heading = read_nan_double("Q_heading");
    if (std::isfinite(q_heading)) cfg.w_theta = q_heading;
    const double r_v = read_nan_double("R_v");
    if (std::isfinite(r_v)) {
        cfg.w_kappa = r_v;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Parameter 'R_v' is deprecated. Use 'weight_curvature' or 'w_kappa'.");
    }
    const double r_w = read_nan_double("R_w");
    if (std::isfinite(r_w)) {
        cfg.w_u = r_w;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Parameter 'R_w' is deprecated. Use 'weight_input' or 'w_u'.");
    }
    const double max_accel = read_nan_double("max_accel");
    if (std::isfinite(max_accel) && max_accel > 0.0) {
        cfg.u_min = -max_accel;
        cfg.u_max = max_accel;
    }
    const double max_w = read_nan_double("max_angular_vel");
    if (std::isfinite(max_w) && max_w > 0.0) {
        const double v_ref_param = read_nan_double("kappa_limit_ref_velocity");
        const double v_ref_for_limit =
            (std::isfinite(v_ref_param) && v_ref_param > 0.0)
            ? v_ref_param
            : std::max(cfg.min_velocity, 0.1);
        const double kappa_limit = max_w / v_ref_for_limit;
        cfg.kappa_min = -kappa_limit;
        cfg.kappa_max = kappa_limit;
    }

    // New explicit overrides
    const int n_new = this->get_parameter("N").as_int();
    if (n_new > 0) cfg.N = n_new;
    const double ts_new = read_nan_double("Ts");
    if (std::isfinite(ts_new) && ts_new > 1e-4) cfg.Ts = ts_new;
    const double wd_new = read_nan_double("w_d");
    if (std::isfinite(wd_new)) cfg.w_d = wd_new;
    const double wth_new = read_nan_double("w_theta");
    if (std::isfinite(wth_new)) cfg.w_theta = wth_new;
    const double wk_new = read_nan_double("w_kappa");
    if (std::isfinite(wk_new)) cfg.w_kappa = wk_new;
    const double wu_new = read_nan_double("w_u");
    if (std::isfinite(wu_new)) cfg.w_u = wu_new;
    const int ref_preview_steps = this->get_parameter("ref_preview_steps").as_int();
    cfg.ref_preview_steps = std::max(0, ref_preview_steps);
    cfg.curvature_speed_gain =
        std::max(0.0, this->get_parameter("curvature_speed_gain").as_double());
    cfg.kappa_blend_alpha =
        std::clamp(this->get_parameter("kappa_blend_alpha").as_double(), 0.0, 1.0);
    cfg.lateral_bound =
        this->get_parameter("lateral_bound").as_double();
    cfg.w_lateral_slack_lin =
        std::max(0.0, this->get_parameter("w_lateral_slack_lin").as_double());
    cfg.w_lateral_slack_quad =
        std::max(0.0, this->get_parameter("w_lateral_slack_quad").as_double());
    // OSQP settings
    cfg.osqp_max_iter  = static_cast<int>(std::max(100L, this->get_parameter("osqp_max_iter").as_int()));
    cfg.osqp_eps_abs   = std::max(1e-7, this->get_parameter("osqp_eps_abs").as_double());
    cfg.osqp_eps_rel   = std::max(1e-7, this->get_parameter("osqp_eps_rel").as_double());
    cfg.osqp_polish    = this->get_parameter("osqp_polish").as_bool();

    max_omega_abs_ = std::max(0.1, this->get_parameter("max_omega_abs").as_double());
    max_omega_rate_ = std::max(0.1, this->get_parameter("max_omega_rate").as_double());
    max_v_rate_ = std::max(0.1, this->get_parameter("max_v_rate").as_double());
    sig_tau_up_   = std::max(0.01, this->get_parameter("sigmoid_tau_accel").as_double());
    sig_tau_down_ = std::max(0.01, this->get_parameter("sigmoid_tau_decel").as_double());
    path_reset_distance_threshold_ =
        std::max(0.1, this->get_parameter("path_reset_distance_threshold").as_double());
    path_hold_distance_gain_ =
        std::max(0.0, this->get_parameter("path_hold_distance_gain").as_double());
    path_hold_heading_gain_ =
        std::max(0.0, this->get_parameter("path_hold_heading_gain").as_double());
    path_hold_max_omega_ =
        std::max(0.1, this->get_parameter("path_hold_max_omega").as_double());
    path_hold_lookahead_index_ =
        std::max(1, static_cast<int>(this->get_parameter("path_hold_lookahead_index").as_int()));
    path_hold_recovery_distance_ =
        std::max(0.1, this->get_parameter("path_hold_recovery_distance").as_double());
    path_hold_heading_error_gain_ =
        std::max(0.0, this->get_parameter("path_hold_heading_error_gain").as_double());
    path_hold_cross_track_gain_ =
        std::max(0.0, this->get_parameter("path_hold_cross_track_gain").as_double());
    path_hold_recovery_blend_ =
        std::clamp(this->get_parameter("path_hold_recovery_blend").as_double(), 0.0, 1.0);
    curve_speed_enable_ = this->get_parameter("curve_speed_enable").as_bool();
    curve_speed_heading_threshold_ =
        std::max(0.01, this->get_parameter("curve_speed_heading_threshold").as_double());
    curve_speed_reduction_gain_ =
        std::max(0.0, this->get_parameter("curve_speed_reduction_gain").as_double());
    curve_speed_min_ratio_ =
        std::clamp(this->get_parameter("curve_speed_min_ratio").as_double(), 0.1, 1.0);
    overshoot_guard_distance_ =
        std::max(0.05, this->get_parameter("overshoot_guard_distance").as_double());
    overshoot_reverse_damping_ =
        std::clamp(this->get_parameter("overshoot_reverse_damping").as_double(), 0.1, 1.0);
    oscillation_guard_enable_ = this->get_parameter("oscillation_guard_enable").as_bool();
    oscillation_guard_cte_deadband_ =
        std::max(0.01, this->get_parameter("oscillation_guard_cte_deadband").as_double());
    oscillation_guard_heading_deadband_ =
        std::max(0.01, this->get_parameter("oscillation_guard_heading_deadband").as_double());
    oscillation_guard_reverse_damping_ =
        std::clamp(this->get_parameter("oscillation_guard_reverse_damping").as_double(), 0.1, 1.0);
    adaptive_corner_mode_enable_ =
        this->get_parameter("adaptive_corner_mode_enable").as_bool();
    adaptive_near_cte_thresh_ =
        std::max(0.01, this->get_parameter("adaptive_near_cte_thresh").as_double());
    adaptive_near_heading_thresh_ =
        std::max(0.01, this->get_parameter("adaptive_near_heading_thresh").as_double());
    adaptive_near_omega_damping_ =
        std::clamp(this->get_parameter("adaptive_near_omega_damping").as_double(), 0.1, 1.0);
    adaptive_near_omega_rate_scale_ =
        std::clamp(this->get_parameter("adaptive_near_omega_rate_scale").as_double(), 0.1, 1.0);
    adaptive_near_v_scale_ =
        std::clamp(this->get_parameter("adaptive_near_v_scale").as_double(), 0.5, 1.0);
    off_path_recovery_enable_ = this->get_parameter("off_path_recovery_enable").as_bool();
    off_path_recovery_distance_ =
        std::max(0.2, this->get_parameter("off_path_recovery_distance").as_double());
    off_path_recovery_exit_distance_ =
        std::clamp(this->get_parameter("off_path_recovery_exit_distance").as_double(), 0.1,
                   off_path_recovery_distance_ - 0.05);
    off_path_recovery_speed_ =
        std::max(0.05, this->get_parameter("off_path_recovery_speed").as_double());
    off_path_recovery_heading_gain_ =
        std::max(0.0, this->get_parameter("off_path_recovery_heading_gain").as_double());
    off_path_recovery_cte_gain_ =
        std::max(0.0, this->get_parameter("off_path_recovery_cte_gain").as_double());
    off_path_recovery_max_omega_ =
        std::max(0.2, this->get_parameter("off_path_recovery_max_omega").as_double());
    effective_horizon_ = std::max(1, cfg.N);

    ltv_controller_->setConfig(cfg);

    if (legacy_controller_) {
        LTVMPCParams legacy_params;
        legacy_params.N = cfg.N;
        legacy_params.Ts = cfg.Ts;
        legacy_params.l = cfg.wheelbase;
        legacy_params.wd = cfg.w_d;
        legacy_params.wtheta = cfg.w_theta;
        legacy_params.wkappa = cfg.w_kappa;
        legacy_params.wu = cfg.w_u;
        legacy_params.max_velocity = cfg.max_velocity;
        legacy_params.min_velocity = cfg.min_velocity;
        legacy_params.u_min = cfg.u_min;
        legacy_params.u_max = cfg.u_max;
        legacy_params.kappa_min_delta = cfg.kappa_min;
        legacy_params.kappa_max_delta = cfg.kappa_max;
        legacy_controller_->update_parameters(legacy_params);
    }
}

rcl_interfaces::msg::SetParametersResult MPCPathTrackerCpp::parameter_callback(
    const std::vector<rclcpp::Parameter>& params) {
    
    for (const auto& param : params) {
        RCLCPP_INFO(this->get_logger(), "Parameter updated: %s", param.get_name().c_str());
    }
    
    update_controller_params();
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "LTV-MPC parameters updated successfully";
    return result;
}

void MPCPathTrackerCpp::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    local_path_ = msg->poses;
    if (local_path_.empty()) return;

    const double ax = local_path_.front().pose.position.x;
    const double ay = local_path_.front().pose.position.y;
    if (!has_local_path_anchor_) {
        local_path_anchor_x_ = ax;
        local_path_anchor_y_ = ay;
        has_local_path_anchor_ = true;
        return;
    }

    const double dist = std::hypot(ax - local_path_anchor_x_, ay - local_path_anchor_y_);
    if (dist > path_reset_distance_threshold_) {
        if (ltv_controller_) ltv_controller_->reset();
        if (legacy_controller_) legacy_controller_->reset_state();
        cmd_initialized_ = false;
        v_sig_state_ = 0.0;
        has_prev_errors_ = false;
        off_path_recovery_latched_ = false;
    }
    local_path_anchor_x_ = ax;
    local_path_anchor_y_ = ay;
}

void MPCPathTrackerCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = msg->pose;
}

int MPCPathTrackerCpp::get_effective_horizon() const {
    return std::max(1, effective_horizon_);
}

void MPCPathTrackerCpp::control_loop() {
    const auto start_total = std::chrono::high_resolution_clock::now();

    // 경로나 현재 위치가 없으면 정지
    if (local_path_.empty() || !current_pose_) {
        if (ltv_controller_) ltv_controller_->reset();
        if (legacy_controller_) legacy_controller_->reset_state();
        cmd_initialized_ = false;
        v_sig_state_ = 0.0;
        has_prev_errors_ = false;
        off_path_recovery_latched_ = false;
        publish_control(0.0, 0.0);
        publish_performance(0.0, 0.0, 0.0, -1, "NO_DATA");
        return;
    }
    
    const size_t min_path_points = static_cast<size_t>(get_effective_horizon() + 10);
    if (local_path_.size() < min_path_points) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Insufficient local path points: got=%zu required>=%zu",
            local_path_.size(), min_path_points);
        if (ltv_controller_) ltv_controller_->reset();
        if (legacy_controller_) legacy_controller_->reset_state();
        cmd_initialized_ = false;
        v_sig_state_ = 0.0;
        has_prev_errors_ = false;
        off_path_recovery_latched_ = false;
        publish_control(0.0, 0.0);
        publish_performance(0.0, 0.0, 0.0, -1, "LOCAL_PATH_TOO_SHORT");
        return;
    }
    
    double v_cmd = 0.0;
    double w_cmd = 0.0;
    std::vector<std::array<double, 3>> pred;

    double model_time_us = 0.0;
    double solver_time_us = 0.0;
    int solver_iterations = -1;
    std::string solver_status = "UNKNOWN";

    if (use_ltv_mpc_) {
        auto output = ltv_controller_->computeControl(*current_pose_, local_path_);
        model_time_us = output.model_time_us;
        solver_time_us = output.solver_time_us;
        v_cmd = output.v_cmd;
        w_cmd = output.omega_cmd;
        pred = std::move(output.predicted_xy);
        solver_status = output.solved ? "SOLVED" : "FAILED";
    } else {
        const auto start_solver = std::chrono::high_resolution_clock::now();
        auto output = legacy_controller_->compute_control(*current_pose_, local_path_);
        const auto end_solver = std::chrono::high_resolution_clock::now();
        solver_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end_solver - start_solver).count();
        model_time_us = solver_time_us;
        v_cmd = output.velocity;
        w_cmd = output.angular_velocity;
        pred = std::move(output.predicted_trajectory);
        solver_status = "LEGACY";
    }
    const auto end_total = std::chrono::high_resolution_clock::now();
    const double total_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        end_total - start_total).count();

    // Compute nearest distance from ego to local path for path-hold correction.
    double min_path_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    double signed_cte = 0.0;
    double heading_error_near = 0.0;
    bool near_metrics_valid = false;
    double adaptive_rate_scale = 1.0;
    double adaptive_v_scale = 1.0;
    if (current_pose_ && !local_path_.empty()) {
        const double ex = current_pose_->position.x;
        const double ey = current_pose_->position.y;
        for (size_t i = 0; i < local_path_.size(); ++i) {
            const double dx = local_path_[i].pose.position.x - ex;
            const double dy = local_path_[i].pose.position.y - ey;
            const double d = std::hypot(dx, dy);
            if (d < min_path_dist) {
                min_path_dist = d;
                closest_idx = i;
            }
        }

        if (closest_idx + 1 < local_path_.size()) {
            const auto& p0 = local_path_[closest_idx].pose.position;
            const auto& p1 = local_path_[closest_idx + 1].pose.position;
            const double path_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
            const double yaw = pose_yaw_from_quat_or_packed(*current_pose_);
            const double rx = ex - p0.x;
            const double ry = ey - p0.y;
            signed_cte = -std::sin(path_yaw) * rx + std::cos(path_yaw) * ry;
            heading_error_near = wrap_angle(path_yaw - yaw);
            near_metrics_valid = true;
        }
    }

    if (off_path_recovery_enable_ && near_metrics_valid) {
        if (!off_path_recovery_latched_ && min_path_dist > off_path_recovery_distance_) {
            off_path_recovery_latched_ = true;
        } else if (off_path_recovery_latched_ &&
                   min_path_dist < off_path_recovery_exit_distance_) {
            off_path_recovery_latched_ = false;
        }
    } else {
        off_path_recovery_latched_ = false;
    }

    // Path-guided steering assist: derive yaw-rate from lookahead geometry and blend with MPC.
    const bool enable_path_fallback = this->get_parameter("enable_path_fallback").as_bool();
    if (!off_path_recovery_latched_ &&
        enable_path_fallback && current_pose_ && !local_path_.empty()) {
        const double min_abs_omega = std::max(0.0, this->get_parameter("fallback_min_abs_omega").as_double());
        const int lookahead_idx_param = this->get_parameter("fallback_lookahead_index").as_int();
        const size_t lookahead_idx = static_cast<size_t>(std::max(1, std::min<int>(lookahead_idx_param, static_cast<int>(local_path_.size()) - 1)));
        const double fallback_max_abs = std::max(0.1, this->get_parameter("fallback_max_abs_omega").as_double());
        const double fallback_blend = std::clamp(this->get_parameter("fallback_blend").as_double(), 0.0, 1.0);
        const auto& p = local_path_[lookahead_idx].pose.position;
        const double dx = p.x - current_pose_->position.x;
        const double dy = p.y - current_pose_->position.y;
        const double ld = std::hypot(dx, dy);
        if (ld > 1e-3) {
            const double yaw = pose_yaw_from_quat_or_packed(*current_pose_);
            const double target_heading = std::atan2(dy, dx);
            const double alpha = wrap_angle(target_heading - yaw);
            const double v_for_fb = std::max(0.5, std::abs(v_cmd));
            double w_fb = 2.0 * v_for_fb * std::sin(alpha) / ld;
            w_fb = std::clamp(w_fb, -fallback_max_abs, fallback_max_abs);

            if (std::isfinite(w_fb)) {
                if (std::abs(w_cmd) < min_abs_omega) {
                    w_cmd = w_fb;
                } else {
                    w_cmd = (1.0 - fallback_blend) * w_cmd + fallback_blend * w_fb;
                }
            }
        }
    }

    // Always-on path-hold assist:
    // 1) recover cross-track error quickly when outside path corridor
    // 2) align heading to path tangent immediately after re-entry
    if (!off_path_recovery_latched_ &&
        current_pose_ && !local_path_.empty() && std::isfinite(min_path_dist)) {
        const size_t recovery_lookahead = 3;
        const size_t lookahead =
            (min_path_dist > path_hold_recovery_distance_) ? recovery_lookahead
                                                           : static_cast<size_t>(path_hold_lookahead_index_);
        const size_t tgt_idx = std::min(closest_idx + lookahead, local_path_.size() - 1);
        const auto &p = local_path_[tgt_idx].pose.position;
        const auto &p0 = local_path_[closest_idx].pose.position;
        const auto &p1 = local_path_[std::min(closest_idx + static_cast<size_t>(1), local_path_.size() - 1)].pose.position;
        const double dx = p.x - current_pose_->position.x;
        const double dy = p.y - current_pose_->position.y;
        const double ld = std::hypot(dx, dy);
        const double yaw = pose_yaw_from_quat_or_packed(*current_pose_);
        const double path_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
        const double target_heading = std::atan2(dy, dx);
        const double alpha = wrap_angle(target_heading - yaw);
        const double heading_error = wrap_angle(path_yaw - yaw);
        const double rx = current_pose_->position.x - p0.x;
        const double ry = current_pose_->position.y - p0.y;
        const double cross_track_error = -std::sin(path_yaw) * rx + std::cos(path_yaw) * ry;
        const double dist_scale = 1.0 / (1.0 + path_hold_distance_gain_ * min_path_dist);
        v_cmd *= std::clamp(dist_scale, 0.2, 1.0);

        double w_geo = 0.0;
        if (ld > 1e-3) {
            w_geo = 2.0 * std::max(0.2, std::abs(v_cmd)) * std::sin(alpha) / ld;
        }
        double w_heading = path_hold_heading_error_gain_ * heading_error;
        double w_cte = -path_hold_cross_track_gain_ * cross_track_error *
                       std::max(0.3, std::abs(v_cmd));
        // Prevent heading/CTE correction from overwhelming geometry guidance.
        w_heading = std::clamp(w_heading, -path_hold_max_omega_, path_hold_max_omega_);
        w_cte = std::clamp(w_cte, -path_hold_max_omega_, path_hold_max_omega_);
        double w_aux = w_heading + w_cte;
        if (std::abs(w_geo) > 1e-3 && (w_aux * w_geo) < 0.0) {
            w_aux *= 0.3;
        }
        const double w_hold = w_geo + w_aux;

        // Blend MPC yaw-rate with path-hold corrective yaw-rate.
        const double base_blend = std::clamp(
            path_hold_heading_gain_ * (0.3 + std::min(min_path_dist, 2.0) / 2.0), 0.0, 0.9);
        const double recovery_scale = std::clamp(
            (min_path_dist - path_hold_recovery_distance_) /
                std::max(path_hold_recovery_distance_, 1e-3),
            0.0, 1.0);
        const double blend =
            std::clamp(base_blend + path_hold_recovery_blend_ * recovery_scale, 0.0, 0.95);
        const double w_blended = (1.0 - blend) * w_cmd + blend * w_hold;
        if (blend > 1e-6) {
            w_cmd = std::clamp(w_blended, -path_hold_max_omega_, path_hold_max_omega_);
        } else {
            w_cmd = w_blended;
        }
    }

    if (near_metrics_valid && off_path_recovery_latched_) {
        const double w_recover =
            off_path_recovery_heading_gain_ * heading_error_near -
            off_path_recovery_cte_gain_ * signed_cte;
        w_cmd = std::clamp(w_recover, -off_path_recovery_max_omega_, off_path_recovery_max_omega_);
        v_cmd = std::min(v_cmd, off_path_recovery_speed_);
    }

    if (near_metrics_valid && !off_path_recovery_latched_) {
        if (curve_speed_enable_) {
            const double h_abs = std::abs(heading_error_near);
            if (h_abs > curve_speed_heading_threshold_) {
                const double excess = h_abs - curve_speed_heading_threshold_;
                const double ratio = std::max(
                    curve_speed_min_ratio_, 1.0 - curve_speed_reduction_gain_ * excess);
                v_cmd *= ratio;
            }
        }

        if (std::abs(signed_cte) > overshoot_guard_distance_ &&
            (w_cmd * signed_cte) > 0.0) {
            w_cmd *= overshoot_reverse_damping_;
        }

        if (oscillation_guard_enable_ && has_prev_errors_) {
            const bool cte_flip =
                (std::abs(signed_cte) > oscillation_guard_cte_deadband_) &&
                (std::abs(prev_signed_cte_) > oscillation_guard_cte_deadband_) &&
                ((signed_cte * prev_signed_cte_) < 0.0);
            const bool heading_flip =
                (std::abs(heading_error_near) > oscillation_guard_heading_deadband_) &&
                (std::abs(prev_heading_error_) > oscillation_guard_heading_deadband_) &&
                ((heading_error_near * prev_heading_error_) < 0.0);
            if (cte_flip || heading_flip) {
                w_cmd *= oscillation_guard_reverse_damping_;
            }
        }

        if (adaptive_corner_mode_enable_) {
            const bool near_path =
                (std::abs(signed_cte) < adaptive_near_cte_thresh_) &&
                (std::abs(heading_error_near) < adaptive_near_heading_thresh_);
            if (near_path) {
                w_cmd *= adaptive_near_omega_damping_;
                adaptive_rate_scale = adaptive_near_omega_rate_scale_;
                adaptive_v_scale = adaptive_near_v_scale_;
            }
        }

        prev_signed_cte_ = signed_cte;
        prev_heading_error_ = heading_error_near;
        has_prev_errors_ = true;
    }

    // 제어 출력 제한: 크기 포화 + 레이트 제한(지연/급변 완화)
    const auto now = this->now();
    double dt = (now - prev_cmd_time_).seconds();
    if (!std::isfinite(dt) || dt <= 1e-4) dt = 0.05;
    if (!cmd_initialized_) {
        prev_v_cmd_ = v_cmd;
        prev_w_cmd_ = w_cmd;
        v_sig_state_ = std::max(0.0, v_cmd);
        cmd_initialized_ = true;
    }

    w_cmd = std::clamp(w_cmd, -max_omega_abs_, max_omega_abs_);
    const double recovery_boost =
        (std::isfinite(min_path_dist) && min_path_dist > path_hold_recovery_distance_) ? 1.6 : 1.0;
    const double max_dw = max_omega_rate_ * recovery_boost * adaptive_rate_scale * dt;
    w_cmd = std::clamp(w_cmd, prev_w_cmd_ - max_dw, prev_w_cmd_ + max_dw);

    // Sigmoid velocity smoother: 급출발/급정지 완화
    // 가속 시 tau_up, 감속 시 tau_down의 지수 필터 → S자 속도 곡선
    {
        const double v_target  = v_cmd * adaptive_v_scale;
        const double tau       = (v_target > v_sig_state_) ? sig_tau_up_ : sig_tau_down_;
        const double alpha     = 1.0 - std::exp(-dt / std::max(tau, 1e-3));
        v_sig_state_          += alpha * (v_target - v_sig_state_);
        v_sig_state_           = std::max(0.0, v_sig_state_);
        v_cmd                  = v_sig_state_;
    }

    prev_v_cmd_ = v_cmd;
    prev_w_cmd_ = w_cmd;
    prev_cmd_time_ = now;

    // 제어 명령 발행
    publish_control(v_cmd, w_cmd);

    // 예측 궤적 발행 (visualization용)
    publish_predicted_path(pred);
    publish_performance(model_time_us, solver_time_us, total_time_us, solver_iterations, solver_status);
    
    // 로깅 (2초마다)
    if ((now - last_log_time_).seconds() > 1.0) {
        RCLCPP_INFO(this->get_logger(), 
                   "ControlPath=%s | v: %.2f m/s, ω: %.3f rad/s",
                   use_ltv_mpc_ ? "LTV_MPC" : "LEGACY_MPC",
                   v_cmd, w_cmd);
        last_log_time_ = now;
    }
}

void MPCPathTrackerCpp::publish_performance(double model_time_us, double solver_time_us,
                                            double total_time_us, int solver_iterations,
                                            const std::string& solver_status) {
    if (!perf_pub_) return;

    bisa::msg::MPCPerformance msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    msg.cav_id = target_cav_id_;
    msg.model_time_us = model_time_us;
    msg.solver_time_us = solver_time_us;
    msg.total_time_us = total_time_us;
    msg.horizon = get_effective_horizon();
    msg.solver_iterations = solver_iterations;
    msg.solver_status = solver_status;
    perf_pub_->publish(msg);
}

void MPCPathTrackerCpp::publish_control(double v, double w) {
    // Keep original semantics used in this project:
    //   linear.x  = velocity [m/s]
    //   angular.z = yaw-rate  [rad/s]
    geometry_msgs::msg::Accel msg;
    msg.linear.x = v;
    msg.angular.z = w;

    if (publish_accel_cmd_ && accel_pub_) accel_pub_->publish(msg);  // /Accel -> remapped to /cavXX/accel_cmd
    if (accel_pub_raw_) accel_pub_raw_->publish(msg); // /Accel_raw -> remapped to /cavXX/accel_raw
}

void MPCPathTrackerCpp::publish_predicted_path(
    const std::vector<std::array<double, 3>>& traj) {
    
    auto msg = nav_msgs::msg::Path();
    msg.header.frame_id = "world";
    msg.header.stamp = this->now();
    
    for (const auto& pt : traj) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = pt[0];
        pose.pose.position.y = pt[1];
        pose.pose.position.z = pt[2];
        pose.pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
    }
    
    pred_pub_->publish(msg);
}

}  // namespace bisa

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bisa::MPCPathTrackerCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
