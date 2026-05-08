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
    gear_srv_   = nh.serviceClient<morai_msgs::MoraiEventCmdSrv>("/Service_MoraiEventCmd");
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

    // Summary statistics
    double cte_sq = 0.0, hdg_sq = 0.0, vel_sq = 0.0;
    int cnt = 0;
    for (auto& r : log_recs_) {
        if (!r.isMember("cte")) continue;
        double c = r["cte"].asDouble();
        double h = r["hdg_err_deg"].asDouble();
        cte_sq += c * c;
        hdg_sq += h * h;
        if (r.isMember("vel_error")) {
            double ve = r["vel_error"].asDouble();
            vel_sq += ve * ve;
        }
        ++cnt;
    }
    if (cnt > 0) {
        Json::Value stats(Json::objectValue);
        stats["cte_rmse"]     = std::sqrt(cte_sq / cnt);
        stats["hdg_rmse_deg"] = std::sqrt(hdg_sq / cnt);
        stats["vel_err_rmse"] = std::sqrt(vel_sq / cnt);
        stats["num_records"]  = cnt;
        root["summary"] = stats;
    }

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
    wp_gear_.resize(n, 1);  // 기본값: 전진(D)
    for (int i = 0; i < n; ++i) {
        wp_x_[i] = wps[i]["x"].asDouble();
        wp_y_[i] = wps[i]["y"].asDouble();
        if (wps[i].isMember("gear")) {
            std::string g = wps[i]["gear"].asString();
            wp_gear_[i] = (g == "R" || g == "r") ? -1 : 1;
        }
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
    // 후진 구간이면 heading 벡터를 180° 반전하여 dot product 계산
    double gear_sign = (cur_gear_ < 0) ? -1.0 : 1.0;
    const double hx = gear_sign * std::cos(cur_yaw_), hy = gear_sign * std::sin(cur_yaw_);
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
    // 후진 시: 차량은 경로 진행 방향의 반대를 향하므로 π 보정
    double heading_err = (cur_gear_ < 0)
        ? wrapAngle(cur_yaw_ - path_yaw + M_PI)
        : wrapAngle(cur_yaw_ - path_yaw);
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

    // ── 기어 전환 관리 ─────────────────────────────────────────
    int wp_gear = wp_gear_[std::min(nearest_idx_, (int)wp_gear_.size() - 1)];
    if (wp_gear != cur_gear_ && !gear_switching_) {
        // 기어 전환 시작: 정지 명령 + 기어 발행
        publishCmd(0.0, 0.0);
        gear_switching_ = true;
        gear_switch_time_ = now;
        cur_gear_ = wp_gear;
        morai_msgs::MoraiEventCmdSrv srv;
        srv.request.request.option = 2;    // gear 변경 적용
        srv.request.request.ctrl_mode = 3;
        srv.request.request.gear = (cur_gear_ < 0) ? 2 : 4;  // R=2, D=4
        if (gear_srv_.call(srv)) {
            ROS_INFO("[PathFollower] 기어 전환 성공: %s (gear=%d)", (cur_gear_ < 0) ? "R" : "D", srv.request.request.gear);
        } else {
            ROS_WARN("[PathFollower] 기어 전환 서비스 호출 실패");
        }
        return;
    }
    if (gear_switching_) {
        double elapsed = (now - gear_switch_time_).toSec();
        if (elapsed < kGearSwitchWait) {
            publishCmd(0.0, 0.0);  // 대기 중 정지 유지
            return;
        }
        gear_switching_ = false;
        ROS_INFO("[PathFollower] 기어 전환 완료, 주행 재개");
    }

    NearResult near = findNearest();
    const int n = static_cast<int>(wp_x_.size());

    // ── Compute curvature lookahead ────────────────────────────
    double max_kappa_ahead = 0.0;
    {
        double la_m = cfg_.curve_lookahead_m;
        int la_steps = std::max(1, (int)std::round(la_m / wp_spacing_));
        for (int i = 0; i <= la_steps; ++i) {
            int ki = std::min(nearest_idx_ + i, (int)wp_k_.size() - 1);
            max_kappa_ahead = std::max(max_kappa_ahead, std::abs(wp_k_[ki]));
        }
    }

    Json::Value rec(Json::objectValue);
    rec["t"] = (now - log_t0_).toSec();
    rec["x"] = cur_x_; rec["y"] = cur_y_;
    rec["v_kmh"] = cur_v_ * 3.6;
    rec["cte"] = near.signed_cte;
    rec["hdg_err_deg"] = near.heading_err * 180.0 / M_PI;
    rec["target_vel"] = cfg_.target_vel * 3.6;
    rec["actual_vel"] = cur_v_ * 3.6;
    rec["vel_error"] = (cfg_.target_vel - cur_v_) * 3.6;
    rec["path_curvature"] = wp_k_[nearest_idx_];
    rec["max_kappa_ahead"] = max_kappa_ahead;

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
        // 후진 시 look-ahead를 경로 진행 방향으로 (인덱스 증가 방향)
        int target_idx = std::min(nearest_idx_ + (int)std::round(la_dist / wp_spacing_), n - 1);
        double bearing = std::atan2(wp_y_[target_idx] - cur_y_, wp_x_[target_idx] - cur_x_);
        // 후진 시: 차량 뒤쪽으로 가야 하므로 bearing에서 π 보정
        double alpha = (cur_gear_ < 0)
            ? wrapAngle(bearing - cur_yaw_ + M_PI)
            : wrapAngle(bearing - cur_yaw_);
        
        // RECOV mode output also in RADIANS
        double steer_rad = std::clamp(alpha, -max_steer_deg_ * M_PI / 180.0, max_steer_deg_ * M_PI / 180.0);
        double recov_vel = (cur_gear_ < 0) ? std::min(kRecovMaxVel, kReverseMaxVel) : kRecovMaxVel;
        double v_cmd = velocitySigmoid(recov_vel, dt);

        // Apply rate limit and unify sign (- for MORAI)
        double steer_deg_limited = steerRateLimit(steer_rad * 180.0 / M_PI, dt);
        double final_steer_rad = steer_deg_limited * M_PI / 180.0;

        prev_steer_ = steer_deg_limited;
        prev_was_recov_ = true;

        rec["mode"] = (cur_gear_ < 0) ? "RECOV_R" : "RECOV";
        rec["steer_cmd"] = final_steer_rad;
        log_recs_.push_back(rec);

        {
            std_msgs::Float32MultiArray perf;
            perf.data.resize(7);
            perf.data[0] = static_cast<float>(near.dist);
            perf.data[1] = 0.0f;
            perf.data[2] = static_cast<float>(near.signed_cte);
            perf.data[3] = static_cast<float>(near.heading_err * 180.0 / M_PI);
            perf.data[4] = static_cast<float>(cur_v_ * 3.6);
            perf.data[5] = 0.0f;
            perf.data[6] = static_cast<float>(recov_vel);
            perf_pub_.publish(perf);

            std_msgs::String status;
            status.data = (cur_gear_ < 0) ? "RECOV_R" : "RECOV";
            status_pub_.publish(status);
        }

        publishCmd(v_cmd, final_steer_rad); return;
    }

    rec["mode"] = (cur_gear_ < 0) ? "NORMAL_R" : "NORMAL";
    rec["gear"] = (cur_gear_ < 0) ? "R" : "D";
    // ── NORMAL MPC (Mobility Structure) ──────────────────────────
    double theta_ref = wp_h_[nearest_idx_], kappa_ref = wp_k_[nearest_idx_];
    double dx = cur_x_ - wp_x_[nearest_idx_], dy = cur_y_ - wp_y_[nearest_idx_];
    double dr = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;
    // 후진 시: 차량 heading이 경로 방향과 π 차이 → 보정
    double delta_theta = (cur_gear_ < 0)
        ? wrapAngle(cur_yaw_ - theta_ref + M_PI)
        : wrapAngle(cur_yaw_ - theta_ref);

    Eigen::VectorXd x0(kNx); x0 << dr, theta_ref + delta_theta, current_kappa_, theta_ref, kappa_ref;

    // ── Curvature-based velocity profile ────────────────────────
    double v_ref_calc = std::max(2.0 / 3.6, cur_v_);
    int idx_per_step_v = std::max(1, (int)std::round(v_ref_calc * cfg_.Ts / wp_spacing_));
    std::vector<double> v_profile(cfg_.N);
    for (int i = 0; i < cfg_.N; ++i) {
        // 예측 구간 내 최대 곡률 (lookahead 윈도우)
        int la_steps = std::max(1, (int)std::round(cfg_.curve_lookahead_m / wp_spacing_));
        int base_idx = nearest_idx_ + (i + 1) * idx_per_step_v;
        double max_k = 0.0;
        for (int j = 0; j <= la_steps; ++j) {
            int ki = std::min(base_idx + j, (int)wp_k_.size() - 1);
            max_k = std::max(max_k, std::abs(wp_k_[ki]));
        }
        // 곡률 비례 연속 감속: v = v_max / (1 + alpha * kappa)
        // kappa=0 → v_max, kappa=0.1 → ~v_max/3, kappa=0.2 → ~v_max/5
        double alpha = 20.0;
        double v_target = cfg_.target_vel / (1.0 + alpha * max_k);
        v_target = std::max(cfg_.curve_min_vel, v_target);
        // 후진 시: 속도 제한 + 음수로 LTV 모델에 전달 (A행렬 자동 반전)
        if (cur_gear_ < 0) {
            v_target = std::min(v_target, kReverseMaxVel / 3.6);
            v_target = -v_target;
        }
        v_profile[i] = v_target;
    }
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
    cost_->buildQPObjective(x0, z_bar, A_bar, B_bar, E_bar, v_profile, max_kappa_ahead, P, q_vec);

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
        double raw_steer = steer_rad * cfg_.kappa_gain; 
        
        // Apply rate limit on degrees
        double steer_deg_limited = steerRateLimit(raw_steer * 180.0 / M_PI, dt);
        double final_steer_rad = steer_deg_limited * M_PI / 180.0;
        
        prev_steer_ = steer_deg_limited;
        // MORAI steering: Unified to Positive = Left
        // 후진 시 v_profile[0]이 음수 → abs로 sigmoid에 전달, 결과를 양수 km/h로 전달
        double cmd_vel = velocitySigmoid(std::abs(v_profile[0]), dt);
        publishCmd(cmd_vel * 3.6, final_steer_rad);
        rec["steer_cmd"] = final_steer_rad; rec["current_kappa"] = current_kappa_;
        rec["target_vel"] = v_profile[0] * 3.6;
        rec["predicted_cte"] = dr + sol[0] * cfg_.Ts;
    } else {
        rec["solve_failed"] = true;
    }
    
    auto t_end = ros::WallTime::now();
    rec["solve_ms"] = (t_end - t_start).toSec() * 1000.0;

    // Additional diagnostics
    rec["lateral_accel"] = cur_v_ * cur_v_ * std::abs(wp_k_[nearest_idx_]);
    if (cmd_init_ && dt > 1e-4) {
        double steer_now = rec.isMember("steer_cmd") ? rec["steer_cmd"].asDouble() * 180.0 / M_PI : prev_steer_;
        rec["steering_rate"] = (steer_now - prev_steer_) / dt;
    }
    rec["yaw_rate"] = cur_v_ * current_kappa_;

    // ── Publish real-time performance topic ─────────────────────
    {
        std_msgs::Float32MultiArray perf;
        perf.data.resize(7);
        perf.data[0] = static_cast<float>(near.dist);
        perf.data[1] = static_cast<float>(rec["solve_ms"].asDouble());
        perf.data[2] = static_cast<float>(near.signed_cte);
        perf.data[3] = static_cast<float>(near.heading_err * 180.0 / M_PI);
        perf.data[4] = static_cast<float>(cur_v_ * 3.6);
        perf.data[5] = static_cast<float>(max_kappa_ahead);
        perf.data[6] = static_cast<float>(v_profile[0] * 3.6);
        perf_pub_.publish(perf);

        std_msgs::String status;
        status.data = (cur_gear_ < 0) ? "NORMAL_R" : "NORMAL";
        status_pub_.publish(status);
    }

    // ── Log every tick (no filter) ───────────────────────────────
    log_recs_.push_back(rec);

    if (++log_tick_ % 200 == 0) flushLog();
}

void PathFollower::publishCmd(double vel_kmh, double steer_deg) {
    morai_msgs::CtrlCmd cmd; cmd.longlCmdType = 2;
    // 후진 시에도 MORAI에는 양수 속도 전달 (기어가 R이면 자동 후진)
    cmd.velocity = std::abs(vel_kmh); cmd.steering = steer_deg;
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
