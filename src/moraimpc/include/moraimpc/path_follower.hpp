#pragma once
#include <ros/ros.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <vector>
#include <memory>

#include "moraimpc/ltv_types.hpp"
#include "moraimpc/ltv_model.hpp"
#include "moraimpc/ltv_cost.hpp"
#include "moraimpc/ltv_solver.hpp"

namespace moraimpc {

class PathFollower {
public:
    explicit PathFollower(ros::NodeHandle& nh);
    ~PathFollower();

private:
    // ── 경로 로드 ──────────────────────────────────────────────────
    void loadPath(const std::string& file);

    // ── ROS 콜백 ───────────────────────────────────────────────────
    void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg);
    void controlLoop(const ros::TimerEvent&);

    // ── 최근접 탐색 결과 ───────────────────────────────────────────
    struct NearResult {
        int    idx;
        double dist;
        double signed_cte;
        double heading_err;
        double path_yaw;
    };

    // ── 최근접 탐색: forward-filter + jump suppression ─────────────
    // local_path_pub_cpp.cpp 패턴 기반:
    //   1) 초기화 전 / dist > kRecovDist: 전방향 전역 탐색 (DOT > 0)
    //   2) 초기화 후: 윈도우 탐색 (DOT > kDotThreshold)
    //   3) dist <= kRecovDist: 역방향 점프 차단, 전방향 점프 kMaxIndexStep 제한
    NearResult findNearest();

    // ── 후처리 ─────────────────────────────────────────────────────
    double steerRateLimit(double steer_deg, double dt);
    double velocitySigmoid(double v_tgt, double dt);

    // ── 명령 발행 ──────────────────────────────────────────────────
    void publishCmd(double vel_kmh, double steer_deg);

    // ── 주행 기록 ──────────────────────────────────────────────────
    void flushLog();

    // ═══════════════════════════════════════════════════════════════
    // 상수
    // ═══════════════════════════════════════════════════════════════
    static constexpr int    kSearchWindow  = 300;
    static constexpr int    kMaxIndexStep  = 30;
    static constexpr double kRecovDist     = 1.5;            // [m]  거리 이탈 판단 (4.0→1.5: MPC는 1.5m 이내에서만 사용)
    static constexpr double kRecovHdgThresh= 60.0*M_PI/180.0;// [rad] 헤딩 이탈 판단
    static constexpr double kDotThreshold  = 0.1;            // 전방 필터 임계값
    static constexpr double kRecovMaxVel   = 10.0;           // [km/h] RECOV 최대 속도

    // ═══════════════════════════════════════════════════════════════
    // MPC
    // ═══════════════════════════════════════════════════════════════
    LTVMPCConfig             cfg_;
    std::unique_ptr<LTVModel>  model_;
    std::unique_ptr<LTVCost>   cost_;
    std::unique_ptr<LTVSolver> solver_;
    double current_kappa_ = 0.0;

    // ═══════════════════════════════════════════════════════════════
    // 경로
    // ═══════════════════════════════════════════════════════════════
    std::vector<double> wp_x_, wp_y_, wp_h_, wp_k_;
    double wp_spacing_ = 0.5;

    // ═══════════════════════════════════════════════════════════════
    // 차량 상태
    // ═══════════════════════════════════════════════════════════════
    double cur_x_    = 0.0;
    double cur_y_    = 0.0;
    double cur_yaw_  = 0.0;
    double cur_v_    = 0.0;
    bool   ego_rcvd_ = false;

    // ═══════════════════════════════════════════════════════════════
    // 탐색 상태
    // ═══════════════════════════════════════════════════════════════
    int  nearest_idx_ = 0;
    bool search_init_ = false;

    // ═══════════════════════════════════════════════════════════════
    // 후처리 파라미터 (ltv_mpc_node.cpp 기준값)
    // ═══════════════════════════════════════════════════════════════
    double curve_spd_hdg_thresh_ = 0.20;
    double curve_spd_gain_       = 0.06;
    double curve_spd_min_ratio_  = 0.70;
    double overshoot_dist_       = 0.10;
    double overshoot_damp_       = 0.40;
    double osc_cte_db_           = 0.08;
    double osc_hdg_db_           = 0.10;
    double osc_damp_             = 0.65;
    double near_cte_thresh_      = 0.06;
    double near_hdg_thresh_      = 0.06;
    double near_steer_damp_      = 0.90;
    double near_v_scale_         = 0.98;
    double k_stanley_            = 0.0;   // Stanley 제거 (k=5: MPC 역방향 폭주 유발)
    double max_steer_rate_       = 50.0;
    double max_steer_deg_        = 35.0;
    double sig_tau_up_           = 0.30;
    double sig_tau_down_         = 0.15;

    // ── 후처리 상태 ────────────────────────────────────────────────
    bool      has_prev_errors_ = false;
    double    prev_cte_        = 0.0;
    double    prev_hdg_        = 0.0;
    bool      cmd_init_        = false;
    double    prev_steer_      = 0.0;
    bool      v_sig_init_      = false;
    double    v_sig_           = 0.0;
    ros::Time prev_cmd_time_;

    // ═══════════════════════════════════════════════════════════════
    // ROS
    // ═══════════════════════════════════════════════════════════════
    ros::Subscriber ego_sub_;
    ros::Publisher  ctrl_pub_;
    ros::Publisher  perf_pub_;
    ros::Publisher  status_pub_;
    ros::Timer      timer_;

    // ═══════════════════════════════════════════════════════════════
    // 주행 기록 (JSON)
    // ═══════════════════════════════════════════════════════════════
    std::vector<Json::Value> log_recs_;
    std::string              log_file_;
    ros::Time                log_t0_;
    int                      log_tick_ = 0;
};

}  // namespace moraimpc
