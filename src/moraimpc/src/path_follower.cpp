#include "moraimpc/path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>

#include <jsoncpp/json/json.h>

namespace moraimpc {

static double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ── 생성자 ─────────────────────────────────────────────────────────
PathFollower::PathFollower(ros::NodeHandle& nh) {
    std::string path_file;
    double target_vel = 20.0;
    nh.param<std::string>("path_file",   path_file,   "/tmp/waypoints.json");
    nh.param<double>     ("target_vel",  target_vel,  20.0);

    cfg_.target_vel = target_vel / 3.6;   // km/h → m/s
    nh.param<std::string>("log_file", log_file_, "/tmp/mpc_log.json");
    log_t0_ = ros::Time::now();
    log_recs_.reserve(20000);
    loadPath(path_file);

    model_  = std::make_unique<LTVModel>(cfg_);
    cost_   = std::make_unique<LTVCost>(cfg_);
    solver_ = std::make_unique<LTVSolver>(cfg_);

    ego_sub_    = nh.subscribe("/Ego_topic",       1, &PathFollower::egoCallback,  this);
    ctrl_pub_   = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0",       1);
    perf_pub_   = nh.advertise<std_msgs::Float32MultiArray>("/mpc_performance", 1);
    status_pub_ = nh.advertise<std_msgs::String>   ("/mpc_status",       1);

    timer_ = nh.createTimer(ros::Duration(cfg_.Ts), &PathFollower::controlLoop, this);
    prev_cmd_time_ = ros::Time::now();

    ROS_INFO("[PathFollower] Mobility-Structure MPC 시작 — 목표속도: %.1f km/h", target_vel);
}

PathFollower::~PathFollower() {
    flushLog();
}

void PathFollower::flushLog() {
    if (log_recs_.empty()) return;
    Json::Value root(Json::objectValue);
    root["total_records"] = static_cast<int>(log_recs_.size());
    Json::Value arr(Json::arrayValue);
    for (auto& r : log_recs_) arr.append(r);
    root["records"] = arr;
    std::ofstream ofs(log_file_);
    Json::StreamWriterBuilder wb; wb["indentation"] = " ";
    ofs << Json::writeString(wb, root);
}

void PathFollower::loadPath(const std::string& file) {
    std::ifstream ifs(file);
    Json::Value root; Json::Reader reader;
    if (!reader.parse(ifs, root)) return;
    const Json::Value wps = root["waypoints"];
    int n = static_cast<int>(wps.size());
    wp_x_.resize(n); wp_y_.resize(n); wp_h_.resize(n); wp_k_.resize(n, 0.0);
    for (int i = 0; i < n; ++i) {
        wp_x_[i] = wps[i]["x"].asDouble();
        wp_y_[i] = wps[i]["y"].asDouble();
    }
    if (n >= 2) {
        wp_h_[0] = std::atan2(wp_y_[1] - wp_y_[0], wp_x_[1] - wp_x_[0]);
        for (int i = 1; i < n - 1; ++i)
            wp_h_[i] = std::atan2(wp_y_[i+1] - wp_y_[i-1], wp_x_[i+1] - wp_x_[i-1]);
        wp_h_[n-1] = std::atan2(wp_y_[n-1] - wp_y_[n-2], wp_x_[n-1] - wp_x_[n-2]);
    }
    for (int i = 1; i < n - 1; ++i) {
        double dh = wrapAngle(wp_h_[i+1] - wp_h_[i-1]);
        double ds = std::hypot(wp_x_[i+1] - wp_x_[i-1], wp_y_[i+1] - wp_y_[i-1]);
        wp_k_[i] = (ds > 1e-6) ? (dh / ds) : 0.0;
    }
    wp_k_[0] = wp_k_[1]; wp_k_[n-1] = wp_k_[n-2];
    if (n >= 2) {
        double total = 0.0;
        for (int i = 1; i < n; ++i) total += std::hypot(wp_x_[i] - wp_x_[i-1], wp_y_[i] - wp_y_[i-1]);
        wp_spacing_ = total / (n - 1);
    }
}

void PathFollower::egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    cur_x_ = msg->position.x;
    cur_y_ = msg->position.y;
    cur_yaw_ = wrapAngle(msg->heading * M_PI / 180.0);
    cur_v_ = std::hypot(msg->velocity.x, msg->velocity.y);
    ego_rcvd_ = true;
}

PathFollower::NearResult PathFollower::findNearest() {
    const int n = static_cast<int>(wp_x_.size());
    const double hx = std::cos(cur_yaw_), hy = std::sin(cur_yaw_);
    double min_d = std::numeric_limits<double>::max();
    int closest = nearest_idx_;
    if (search_init_) {
        int w_s = std::max(0, nearest_idx_ - 10), w_e = std::min(n - 1, nearest_idx_ + kSearchWindow);
        for (int i = w_s; i <= w_e; ++i) {
            double dx = wp_x_[i] - cur_x_, dy = wp_y_[i] - cur_y_;
            if (dx * hx + dy * hy < kDotThreshold) continue;
            double dist = std::hypot(dx, dy);
            if (dist < min_d) { min_d = dist; closest = i; }
        }
    }
    if (!search_init_ || min_d > kRecovDist) {
        double best = std::numeric_limits<double>::max(); int best_idx = nearest_idx_; bool found_fwd = false;
        for (int i = 0; i < n; ++i) {
            double dx = wp_x_[i] - cur_x_, dy = wp_y_[i] - cur_y_, dist = std::hypot(dx, dy);
            if (dx * hx + dy * hy > 0.0) { if (!found_fwd || dist < best) { best = dist; best_idx = i; found_fwd = true; } }
            else if (!found_fwd && dist < best) { best = dist; best_idx = i; }
        }
        closest = best_idx; min_d = best; search_init_ = true;
    }
    int delta = closest - nearest_idx_;
    if (delta < 0) closest = nearest_idx_;
    else if (delta > kMaxIndexStep && min_d <= kRecovDist) closest = nearest_idx_ + kMaxIndexStep;
    nearest_idx_ = std::min(closest, n - 1);
    int ni = nearest_idx_, ni1 = std::min(ni + 1, n - 1);
    double path_yaw = std::atan2(wp_y_[ni1] - wp_y_[ni], wp_x_[ni1] - wp_x_[ni]);
    double rx = cur_x_ - wp_x_[ni], ry = cur_y_ - wp_y_[ni];
    double signed_cte = -std::sin(path_yaw) * rx + std::cos(path_yaw) * ry;
    double heading_err = wrapAngle(cur_yaw_ - path_yaw);
    return { nearest_idx_, min_d, signed_cte, heading_err, path_yaw };
}

double PathFollower::steerRateLimit(double steer_deg, double dt) {
    if (!cmd_init_) { prev_steer_ = steer_deg; cmd_init_ = true; return steer_deg; }
    double max_d = max_steer_rate_ * dt;
    steer_deg = std::clamp(steer_deg, prev_steer_ - max_d, prev_steer_ + max_d);
    return std::clamp(steer_deg, -max_steer_deg_, max_steer_deg_);
}

double PathFollower::velocitySigmoid(double v_tgt, double dt) {
    if (!v_sig_init_) { v_sig_ = v_tgt; v_sig_init_ = true; return v_sig_; }
    double tau = (v_tgt > v_sig_) ? sig_tau_up_ : sig_tau_down_;
    double alpha = 1.0 - std::exp(-dt / std::max(tau, 1e-3));
    v_sig_ += alpha * (v_tgt - v_sig_);
    return std::max(0.0, v_sig_);
}

void PathFollower::controlLoop(const ros::TimerEvent&) {
    if (!ego_rcvd_ || wp_x_.empty()) return;
    ros::Time now = ros::Time::now();
    double dt = (now - prev_cmd_time_).toSec();
    if (!std::isfinite(dt) || dt <= 1e-4) dt = cfg_.Ts;
    prev_cmd_time_ = now;
    auto t_start = ros::WallTime::now();

    NearResult near = findNearest();
    const int n = static_cast<int>(wp_x_.size());

    Json::Value rec(Json::objectValue);
    rec["t"] = (now - log_t0_).toSec(); rec["v_kmh"] = cur_v_ * 3.6;
    rec["cte"] = near.signed_cte; rec["hdg_err_deg"] = near.heading_err * 180.0 / M_PI;

    if (nearest_idx_ >= n - 2) {
        if (std::hypot(wp_x_.back() - cur_x_, wp_y_.back() - cur_y_) < 2.0) {
            publishCmd(0.0, 0.0); return;
        }
    }

    bool enter_recov = (near.dist > kRecovDist) || (std::abs(near.heading_err) > kRecovHdgThresh);
    bool exit_recov = (near.dist < kRecovDistExit) && (std::abs(near.heading_err) < kRecovHdgExit);
    if (exit_recov) in_recov_ = false; else if (enter_recov) in_recov_ = true;

    if (in_recov_) {
        double la_dist = std::max(4.0, 10.0 - near.dist * 2.0);
        int target_idx = std::min(nearest_idx_ + (int)std::round(la_dist / wp_spacing_), n - 1);
        double alpha = wrapAngle(std::atan2(wp_y_[target_idx] - cur_y_, wp_x_[target_idx] - cur_x_) - cur_yaw_);
        double steer_deg = std::clamp(alpha * 180.0 / M_PI, -max_steer_deg_, max_steer_deg_);
        double v_cmd = velocitySigmoid(kRecovMaxVel, dt);
        prev_steer_ = steer_deg; prev_was_recov_ = true;
        publishCmd(v_cmd, steer_deg); return;
    }

    // ── NORMAL MPC (Mobility Structure) ──────────────────────────
    double theta_ref = wp_h_[nearest_idx_], kappa_ref = wp_k_[nearest_idx_];
    double dx = cur_x_ - wp_x_[nearest_idx_], dy = cur_y_ - wp_y_[nearest_idx_];
    double dr = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;
    double delta_theta = wrapAngle(cur_yaw_ - theta_ref);

    Eigen::VectorXd x0(kNx); x0 << dr, theta_ref + delta_theta, current_kappa_, theta_ref, kappa_ref;

    std::vector<double> v_profile(cfg_.N, cfg_.target_vel);
    Eigen::MatrixXd A_bar, B_bar, E_bar; model_->buildBatchMatrices(v_profile, A_bar, B_bar, E_bar);

    // ── Build z_bar (Future Curvature Changes) ──────────────────
    double v_ref = std::max(2.0 / 3.6, cur_v_); // Min 2km/h for index calculation
    int idx_per_step = std::max(1, (int)std::round(v_ref * cfg_.Ts / wp_spacing_));
    Eigen::VectorXd z_bar = Eigen::VectorXd::Zero(cfg_.N);
    for (int i = 0; i < cfg_.N; ++i) {
        int ki  = std::min(nearest_idx_ + (i + 1) * idx_per_step, (int)wp_k_.size() - 1);
        int ki0 = std::min(nearest_idx_ +  i      * idx_per_step, (int)wp_k_.size() - 1);
        z_bar(i) = (wp_k_[ki] - wp_k_[ki0]) / cfg_.Ts;
    }

    Eigen::VectorXd x_free = A_bar * x0 + E_bar * z_bar;

    Eigen::SparseMatrix<double> P; Eigen::VectorXd q_vec;
    cost_->buildQPObjective(x0, z_bar, A_bar, B_bar, E_bar, v_profile, P, q_vec);

    // [Structure] Physical Constraint Synchronization (Paper & Mobility Style)
    // 1. Dynamic Input Limit based on max_steer_rate
    const double cur_steer_rad = std::atan(current_kappa_ * cfg_.L);
    const double cos_s = std::cos(cur_steer_rad);
    // kappa_dot = (sec^2(delta) * delta_dot) / L
    double k_dot_limit = (max_steer_rate_ * M_PI / 180.0) / (cfg_.L * cos_s * cos_s + 1e-6);
    double u_lim = k_dot_limit; // Now u is pure kappa_dot

    // 2. Build Constraint Matrix A_cons: [Input Constraints; State Constraints]
    Eigen::MatrixXd A_cons_dense = Eigen::MatrixXd::Zero(2 * cfg_.N, cfg_.N);
    A_cons_dense.block(0, 0, cfg_.N, cfg_.N).setIdentity();
    for (int k = 0; k < cfg_.N; ++k) {
        A_cons_dense.block(cfg_.N + k, 0, 1, cfg_.N) = B_bar.block(k * kNx + 2, 0, 1, cfg_.N);
    }
    Eigen::SparseMatrix<double> A_cons = A_cons_dense.sparseView();

    Eigen::VectorXd l_cons(2 * cfg_.N), u_cons(2 * cfg_.N);
    l_cons.head(cfg_.N) = Eigen::VectorXd::Constant(cfg_.N, -u_lim);
    u_cons.head(cfg_.N) = Eigen::VectorXd::Constant(cfg_.N, u_lim);
    for (int k = 0; k < cfg_.N; ++k) {
        double k_free = x_free(k * kNx + 2);
        l_cons(cfg_.N + k) = cfg_.kappa_min - k_free; 
        u_cons(cfg_.N + k) = cfg_.kappa_max - k_free;
    }

    Eigen::VectorXd sol;
    if (solver_->solve(P, q_vec, A_cons, l_cons, u_cons, sol) && sol.size() > 0) {
        current_kappa_ += sol[0] * cfg_.Ts;
        current_kappa_ = std::clamp(current_kappa_, cfg_.kappa_min, cfg_.kappa_max);

        double steer_rad = std::atan(current_kappa_ * cfg_.L);
        // Apply kappa_gain for MORAI responsiveness
        double final_steer = steer_rad * cfg_.kappa_gain; 
        
        prev_steer_ = final_steer * 180.0 / M_PI;
        // MORAI steering is usually Radian and Sign-inverted
        publishCmd(cfg_.target_vel * 3.6, -final_steer); 
        rec["steer_cmd"] = -final_steer; rec["current_kappa"] = current_kappa_;
    }
    log_recs_.push_back(rec);
    if (++log_tick_ % 200 == 0) flushLog();
}

void PathFollower::publishCmd(double vel_kmh, double steer_deg) {
    morai_msgs::CtrlCmd cmd; cmd.longlCmdType = 2;
    cmd.velocity = std::max(0.0, vel_kmh); cmd.steering = steer_deg;
    ctrl_pub_.publish(cmd);
}

} // namespace moraimpc

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    ros::NodeHandle nh("~");
    moraimpc::PathFollower node(nh);
    ros::spin();
    return 0;
}
