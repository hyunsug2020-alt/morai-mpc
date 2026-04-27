#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <cmath>
#include <algorithm>

#include "moraimpc/ltv_types.hpp"
#include "moraimpc/ltv_model.hpp"
#include "moraimpc/ltv_cost.hpp"
#include "moraimpc/ltv_solver.hpp"

namespace moraimpc {

static double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class LTVMPCNode {
public:
    LTVMPCNode() {
        ros::NodeHandle nh;

        std::string path_file;
        nh.param<std::string>("path_file", path_file, "/home/david/recorded_path.json");
        loadPath(path_file);

        ego_sub_    = nh.subscribe("/Ego_topic", 1, &LTVMPCNode::egoCallback, this);
        ctrl_pub_   = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);
        perf_pub_   = nh.advertise<std_msgs::Float32MultiArray>("/mpc_performance", 1);
        status_pub_ = nh.advertise<std_msgs::String>("/mpc_status", 1);

        model_  = std::make_unique<LTVModel>(cfg_);
        cost_   = std::make_unique<LTVCost>(cfg_);
        solver_ = std::make_unique<LTVSolver>(cfg_);

        timer_ = nh.createTimer(ros::Duration(cfg_.Ts), &LTVMPCNode::controlLoop, this);
        prev_cmd_time_ = ros::Time::now();
        ROS_INFO("LTV-MPC: Ego_topic ground truth, global search on first step");
    }

private:
    // ── 경로 로드 ─────────────────────────────────────────────
    void loadPath(const std::string& file) {
        std::ifstream ifs(file);
        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(ifs, root)) {
            ROS_ERROR("Failed to parse: %s", file.c_str());
            return;
        }
        const Json::Value wps = root["waypoints"];
        int n = wps.size();
        waypoints_x_.resize(n); waypoints_y_.resize(n);
        waypoints_h_.resize(n); waypoints_k_.resize(n, 0.0);

        for (int i = 0; i < n; ++i) {
            waypoints_x_[i] = wps[i]["x"].asDouble();
            waypoints_y_[i] = wps[i]["y"].asDouble();
            waypoints_h_[i] = wps[i]["heading"].asDouble();
        }
        // 참조 곡률 중앙 차분
        for (int i = 1; i < n - 1; ++i) {
            double dh = wrapAngle(waypoints_h_[i+1] - waypoints_h_[i-1]);
            double ds = std::hypot(waypoints_x_[i+1] - waypoints_x_[i-1],
                                   waypoints_y_[i+1] - waypoints_y_[i-1]);
            waypoints_k_[i] = (ds > 1e-6) ? dh / ds : 0.0;
        }
        waypoints_k_[0]   = waypoints_k_[1];
        waypoints_k_[n-1] = waypoints_k_[n-2];

        // 웨이포인트 평균 간격 계산 (z_bar 인덱스 속도 보정에 사용)
        if (n >= 2) {
            double total_len = 0.0;
            for (int i = 1; i < n; ++i)
                total_len += std::hypot(waypoints_x_[i] - waypoints_x_[i-1],
                                        waypoints_y_[i] - waypoints_y_[i-1]);
            waypoint_spacing_ = total_len / (n - 1);
        }
        ROS_INFO("Loaded %d waypoints (avg spacing %.3f m)", n, waypoint_spacing_);
    }

    // ── Ego_topic 콜백 ───────────────────────────────────────
    void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
        cur_x_ = msg->position.x;
        cur_y_ = msg->position.y;
        // MORAI heading: 0=North CW [deg] → ROS yaw: 0=East CCW [rad]
        cur_yaw_ = wrapAngle((90.0 - msg->heading) * M_PI / 180.0);
        cur_v_   = std::hypot(msg->velocity.x, msg->velocity.y);
        is_odom_received_ = true;
    }

    // ── 조향 변화율 제한 ─────────────────────────────────────
    double applySteerRateLimit(double steer_deg, double dt) {
        if (!cmd_initialized_) {
            prev_steer_      = steer_deg;
            cmd_initialized_ = true;
            return steer_deg;
        }
        double max_d = max_steer_rate_ * dt;
        steer_deg = std::clamp(steer_deg,
                               prev_steer_ - max_d,
                               prev_steer_ + max_d);
        return std::clamp(steer_deg, -max_steer_deg_, max_steer_deg_);
    }

    // ── 속도 시그모이드 ──────────────────────────────────────
    double applyVelocitySigmoid(double v_target, double dt) {
        if (!v_sig_init_) { v_sig_ = v_target; v_sig_init_ = true; return v_sig_; }
        double tau   = (v_target > v_sig_) ? sig_tau_up_ : sig_tau_down_;
        double alpha = 1.0 - std::exp(-dt / std::max(tau, 1e-3));
        v_sig_ += alpha * (v_target - v_sig_);
        return std::max(0.0, v_sig_);
    }

    // ── 경로 지표 ────────────────────────────────────────────
    bool computePathMetrics(int idx, double& signed_cte, double& heading_err) {
        int n  = (int)waypoints_x_.size();
        int ni = std::min(idx + 1, n - 1);
        double th = std::atan2(waypoints_y_[ni] - waypoints_y_[idx],
                               waypoints_x_[ni] - waypoints_x_[idx]);
        double dx = cur_x_ - waypoints_x_[idx];
        double dy = cur_y_ - waypoints_y_[idx];
        signed_cte  = -std::sin(th) * dx + std::cos(th) * dy;
        heading_err = wrapAngle(cur_yaw_ - th);
        return std::isfinite(signed_cte) && std::isfinite(heading_err);
    }

    // ── 제어 루프 ────────────────────────────────────────────
    void controlLoop(const ros::TimerEvent&) {
        if (!is_odom_received_ || waypoints_x_.empty()) return;

        ros::Time now = ros::Time::now();
        double dt = (now - prev_cmd_time_).toSec();
        if (!std::isfinite(dt) || dt <= 1e-4) dt = cfg_.Ts;
        prev_cmd_time_ = now;

        auto t_start = ros::WallTime::now();

        // 1. 웨이포인트 탐색
        // ── 핵심 수정: 첫 스텝은 전체 탐색, 이후는 로컬 탐색 ──
        int search_start, search_end;
        if (!search_initialized_) {
            // 처음에는 전체 경로에서 가장 가까운 점 탐색
            search_start = 0;
            search_end   = (int)waypoints_x_.size() - 1;
        } else {
            // 이후에는 이전 인덱스 주변만 탐색 (점프 방지)
            search_start = std::max(0, prev_nearest_idx_ - 10);
            search_end   = std::min((int)waypoints_x_.size() - 1,
                                    prev_nearest_idx_ + 50);
        }

        int nearest_idx = prev_nearest_idx_;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = search_start; i <= search_end; ++i) {
            double d = std::hypot(waypoints_x_[i] - cur_x_,
                                  waypoints_y_[i] - cur_y_);
            if (d < min_dist) { min_dist = d; nearest_idx = i; }
        }
        prev_nearest_idx_  = nearest_idx;
        search_initialized_ = true;

        double theta_ref = waypoints_h_[nearest_idx];
        double kappa_ref = waypoints_k_[nearest_idx];

        double dx_ref = cur_x_ - waypoints_x_[nearest_idx];
        double dy_ref = cur_y_ - waypoints_y_[nearest_idx];
        double dr = -std::sin(theta_ref) * dx_ref + std::cos(theta_ref) * dy_ref;

        double theta_aligned = theta_ref + wrapAngle(cur_yaw_ - theta_ref);

        Eigen::VectorXd x0(kNx);
        x0 << dr, theta_aligned, current_kappa_, theta_ref, kappa_ref;

        // 2. 배치 행렬
        // Bug2 수정: 정지 시 target_vel 사용 (0.5m/s 선형화 방지)
        double v_ref = std::max(cfg_.target_vel, cur_v_);
        std::vector<double> v_profile(cfg_.N, v_ref);
        Eigen::MatrixXd A_bar, B_bar, E_bar;
        model_->buildBatchMatrices(v_profile, A_bar, B_bar, E_bar);

        // Bug3 수정: 실제 속도 기반으로 웨이포인트 인덱스 보정
        // v_ref * Ts = 한 스텝당 이동거리, waypoint_spacing_ = 웨이포인트 간격
        int idx_per_step = std::max(1,
            (int)std::round(v_ref * cfg_.Ts / waypoint_spacing_));

        Eigen::VectorXd z_bar = Eigen::VectorXd::Zero(cfg_.N);
        for (int i = 0; i < cfg_.N; ++i) {
            int ki  = std::min(nearest_idx + (i + 1) * idx_per_step,
                               (int)waypoints_k_.size() - 1);
            int ki0 = std::min(nearest_idx +  i      * idx_per_step,
                               (int)waypoints_k_.size() - 1);
            z_bar(i) = (waypoints_k_[ki] - waypoints_k_[ki0]) / cfg_.Ts;
        }

        // 3. QP 풀기
        Eigen::SparseMatrix<double> P;
        Eigen::VectorXd q_vec;
        cost_->buildQPObjective(x0, z_bar, A_bar, B_bar, E_bar, P, q_vec);

        Eigen::SparseMatrix<double> A_cons(cfg_.N, cfg_.N);
        A_cons.setIdentity();
        Eigen::VectorXd l_cons = Eigen::VectorXd::Constant(cfg_.N, cfg_.u_min);
        Eigen::VectorXd u_cons = Eigen::VectorXd::Constant(cfg_.N, cfg_.u_max);

        Eigen::VectorXd solution;
        bool success = solver_->solve(P, q_vec, A_cons, l_cons, u_cons, solution);

        double solve_ms = (ros::WallTime::now() - t_start).toSec() * 1000.0;

        if (!success || solution.size() == 0) {
            ROS_WARN_THROTTLE(1.0, "MPC Failed  dist=%.2f", min_dist);
            publishCmd(0.0, 0.0);
            return;
        }

        // 4. kappa 적분 (핵심 수정: kappa_ref + u*Ts 가 아니라 누적 적분)
        // ── 버그 원인 ────────────────────────────────────────────────
        // 기존: current_kappa_ = kappa_ref + solution[0] * cfg_.Ts
        //   → 매 스텝마다 kappa_ref 기준으로 덮어써서 MPC 출력이 무의미해짐
        // 수정: 적분 후 kappa_ref 방향으로 10% 블렌드(안정성)
        // ─────────────────────────────────────────────────────────────
        current_kappa_ += solution[0] * cfg_.Ts;
        // kappa_ref 방향 소프트 블렌드 (드리프트 방지, 0.12 = 약 12% 보정)
        const double kappa_blend = 0.12;
        current_kappa_ = (1.0 - kappa_blend) * current_kappa_ + kappa_blend * kappa_ref;
        current_kappa_ = std::clamp(current_kappa_, cfg_.kappa_min, cfg_.kappa_max);

        double steer_deg = std::atan(current_kappa_ * cfg_.L) * (180.0 / M_PI);

        // 5. 경로 지표
        double signed_cte = 0.0, heading_err = 0.0;
        bool metrics_ok = computePathMetrics(nearest_idx, signed_cte, heading_err);

        // ── 후처리 A: 커브 속도 감소 ────────────────────────
        double v_cmd = cfg_.target_vel * 3.6;
        if (metrics_ok) {
            double h_abs = std::abs(heading_err);
            if (h_abs > curve_spd_hdg_thresh_) {
                double ratio = std::max(curve_spd_min_ratio_,
                    1.0 - curve_spd_gain_ * (h_abs - curve_spd_hdg_thresh_));
                v_cmd *= ratio;
            }
        }

        // ── 후처리 B: 오버슈트 방지 ─────────────────────────
        if (metrics_ok) {
            double steer_rad = steer_deg * M_PI / 180.0;
            if (std::abs(signed_cte) > overshoot_dist_ &&
                (steer_rad * signed_cte) > 0.0) {
                steer_deg *= overshoot_damp_;
            }
        }

        // ── 후처리 C: 진동 감지 → 감쇠 ─────────────────────
        if (metrics_ok && has_prev_errors_) {
            bool cte_flip =
                (std::abs(signed_cte)  > osc_cte_db_) &&
                (std::abs(prev_cte_)   > osc_cte_db_) &&
                (signed_cte * prev_cte_ < 0.0);
            bool hdg_flip =
                (std::abs(heading_err) > osc_hdg_db_) &&
                (std::abs(prev_hdg_)   > osc_hdg_db_) &&
                (heading_err * prev_hdg_ < 0.0);
            if (cte_flip || hdg_flip) {
                steer_deg *= osc_damp_;
            }
        }
        if (metrics_ok) {
            prev_cte_        = signed_cte;
            prev_hdg_        = heading_err;
            has_prev_errors_ = true;
        }

        // ── 후처리 D: 경로 근접 시 추가 감쇠 ───────────────
        if (metrics_ok &&
            std::abs(signed_cte)  < near_cte_thresh_ &&
            std::abs(heading_err) < near_hdg_thresh_) {
            steer_deg *= near_steer_damp_;
            v_cmd     *= near_v_scale_;
        }

        // ── 후처리 E: 조향 변화율 제한 ─────────────────────
        steer_deg   = applySteerRateLimit(steer_deg, dt);
        prev_steer_ = steer_deg;

        // ── 후처리 F: 속도 시그모이드 ────────────────────────
        v_cmd = applyVelocitySigmoid(v_cmd, dt);

        publishCmd(v_cmd, steer_deg);

        std_msgs::Float32MultiArray perf;
        perf.data = {(float)min_dist, (float)solve_ms, (float)std::abs(signed_cte)};
        perf_pub_.publish(perf);

        std_msgs::String status;
        status.data = "OPTIMAL";
        status_pub_.publish(status);
    }

    void publishCmd(double vel_kmh, double steer_deg) {
        morai_msgs::CtrlCmd cmd;
        cmd.longlCmdType = 2;
        cmd.velocity = std::max(0.0, vel_kmh);
        // Bug1 수정: MORAI 양수=우회전, MPC 양수=좌회전 → 부호 반전 필요
        cmd.steering = -steer_deg;
        ctrl_pub_.publish(cmd);
    }

    // ── 멤버 변수 ────────────────────────────────────────────
    LTVMPCConfig cfg_;
    std::unique_ptr<LTVModel>  model_;
    std::unique_ptr<LTVCost>   cost_;
    std::unique_ptr<LTVSolver> solver_;

    std::vector<double> waypoints_x_, waypoints_y_, waypoints_h_, waypoints_k_;
    double waypoint_spacing_  = 0.5;   // 웨이포인트 평균 간격 [m] (loadPath에서 계산)
    double cur_x_ = 0, cur_y_ = 0, cur_v_ = 0, cur_yaw_ = 0;
    double current_kappa_     = 0.0;
    int    prev_nearest_idx_  = 0;
    bool   search_initialized_ = false;
    bool   is_odom_received_  = false;

    // 후처리 파라미터 (수정: 너무 공격적인 감쇠값 완화)
    double curve_spd_hdg_thresh_ = 0.20;
    double curve_spd_gain_       = 0.06;  // 0.08→0.06
    double curve_spd_min_ratio_  = 0.70;  // 0.60→0.70 (속도 너무 많이 깎지 않게)
    double overshoot_dist_       = 0.15;  // 0.10→0.15 (더 넓은 허용 대역)
    double overshoot_damp_       = 0.55;  // 0.30→0.55 (30% 감쇠는 너무 강함)
    double osc_cte_db_           = 0.08;  // 0.05→0.08 (노이즈 불감대 확장)
    double osc_hdg_db_           = 0.10;  // 0.08→0.10
    double osc_damp_             = 0.65;  // 0.40→0.65 (40% 감쇠는 너무 강함)
    double near_cte_thresh_      = 0.06;  // 0.08→0.06
    double near_hdg_thresh_      = 0.06;  // 0.08→0.06
    double near_steer_damp_      = 0.90;  // 0.85→0.90 (근접 시 감쇠 약화)
    double near_v_scale_         = 0.98;  // 0.97→0.98
    double max_steer_rate_       = 50.0;  // 40.0→50.0 (조향 변화율 허용 확대)
    double max_steer_deg_        = 35.0;
    double sig_tau_up_           = 0.30;  // 0.40→0.30 (가속 응답 빠르게)
    double sig_tau_down_         = 0.15;  // 0.20→0.15

    bool   has_prev_errors_ = false;
    double prev_cte_        = 0.0;
    double prev_hdg_        = 0.0;
    bool   cmd_initialized_ = false;
    double prev_steer_      = 0.0;
    bool   v_sig_init_      = false;
    double v_sig_           = 0.0;
    ros::Time prev_cmd_time_;

    ros::Subscriber ego_sub_;
    ros::Publisher  ctrl_pub_, perf_pub_, status_pub_;
    ros::Timer      timer_;
};

} // namespace moraimpc

int main(int argc, char** argv) {
    ros::init(argc, argv, "ltv_mpc_node");
    moraimpc::LTVMPCNode node;
    ros::spin();
    return 0;
}