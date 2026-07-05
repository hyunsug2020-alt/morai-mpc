#pragma once
#include <ros/ros.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/EventInfo.h>
#include <morai_msgs/MoraiEventCmdSrv.h>
#include <morai_msgs/ObjectStatusList.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <vector>
#include <memory>

#include "moraimpc/ltv_types.hpp"
#include "moraimpc/ltv_model.hpp"
#include "moraimpc/ltv_cost.hpp"
#include "moraimpc/ltv_solver.hpp"
#include "moraimpc/nmpc_controller.hpp"
#include "moraimpc/rti_nmpc_controller.hpp"

namespace moraimpc {

class PathFollower {
public:
    explicit PathFollower(ros::NodeHandle& nh);
    ~PathFollower();

private:
    // ── 경로 로드 ──────────────────────────────────────────────────
    void loadPath(const std::string& file);
    void loadWaypoints(const Json::Value& wps, const std::string& src_file = "");  // 파일/토픽 공용

    // ── ROS 콜백 ───────────────────────────────────────────────────
    void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg);
    void avoidanceCallback(const std_msgs::Float64::ConstPtr& msg);
    void objectCallback(const morai_msgs::ObjectStatusList::ConstPtr& msg);
    void avoidWpsCallback(const std_msgs::String::ConstPtr& msg);   // hdmap_lane_avoid 동적 경로
    void avoidVelCallback(const std_msgs::Float32::ConstPtr& msg);  // IDM 목표속도 상한
    void controlLoop(const ros::TimerEvent&);

    // ── Frenet 회피 (Phase 1+2) ───────────────────────────────────
    struct Obstacle {
        double x, y;       // 월드 위치
        double vx, vy;     // 월드 속도 (m/s)
        double sx, sy;     // 사이즈 (m)
    };
    // 매 tick, k=0..N-1 의 corridor d_min/d_max 계산
    // d_max[k] = +INF면 좌측 제약 없음, d_min[k] = -INF면 우측 제약 없음
    void buildObstacleCorridor(const std::vector<double>& v_profile,
                               std::vector<double>& d_min,
                               std::vector<double>& d_max,
                               double& v_scale);

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
    static constexpr int    kSearchWindow  = 300;           // 원본 복원 (일반 추종 영향 차단)
    static constexpr int    kMaxIndexStep  =  5;            // 원본 복원
    static constexpr double kRecovDist      = 1.5;            // [m]  RECOV 진입 (1.2→1.5: MPC가 더 처리)
    static constexpr double kRecovDistExit = 0.50;           // [m]  RECOV 탈출 (경로에 더 붙고 탈출)
    static constexpr double kRecovHdgThresh= 35.0*M_PI/180.0;// [rad] RECOV 진입: 전진 헤딩 기준
    static constexpr double kRecovHdgThreshR=60.0*M_PI/180.0;// [rad] RECOV 진입: 후진 헤딩 기준 (완화)
    static constexpr double kRecovHdgExit  = 12.0*M_PI/180.0;// [rad] RECOV 탈출 기준
    static constexpr double kDotThreshold  = 0.1;            // 전방 필터 임계값
    static constexpr double kRecovMaxVel   =  3.0;           // [km/h] RECOV 최대 속도 (6→3: 슬라롬/오버슈팅 방지)
    static constexpr double kGearSwitchWait= 1.5;            // [s]  기어 전환 대기 (MORAI 실제 gear 변경 시간 보장)

    // 런치 파라미터: 후진 최대 속도 (정확도 우선, 기본 2 km/h)
    double reverse_max_vel_kmh_ = 5.0;

    // 고속 D 사전감속용: 이전 tick 의 목표 속도 (rate-limit / jerk 제한)
    // 0.0 = 초기화 sentinel — 첫 tick 은 rate limit 미적용
    double prev_v_target_kmh_ = 0.0;

    // ═══════════════════════════════════════════════════════════════
    // MPC
    // ═══════════════════════════════════════════════════════════════
    LTVMPCConfig             cfg_;
    std::unique_ptr<LTVModel>  model_;
    std::unique_ptr<LTVCost>   cost_;
    std::unique_ptr<LTVSolver> solver_;
    double current_kappa_ = 0.0;

    // NMPC (저속·후진 전용 운동학 자전거)
    NMPCConfig                       nmpc_cfg_;
    std::unique_ptr<NMPCController>  nmpc_;     // (legacy, 보존)
    double low_speed_thresh_kmh_ = 4.0;         // 이 속도 이하 D 모드도 NMPC

    // RTI-NMPC (운동학 자전거, 5-state, 암시적 오일러)
    RTINMPCConfig                       rti_cfg_;
    std::unique_ptr<RTINMPCController>  rti_nmpc_;

    // 저속 모드 hysteresis (4 km/h ping-pong 방지)
    bool   in_low_speed_ = true;             // 직전 tick 저속 모드 여부 (시작은 저속 가정)
    double low_speed_hyst_kmh_ = 1.0;        // ±0.5 km/h hysteresis

    // 직전 tick NMPC 사용 여부 (LTV→NMPC 전환 감지 → kappa 인계)
    bool   prev_use_nmpc_ = false;

    // 기어 전환 사전 감속 — D→R / R→D 경계 N m 앞부터 NMPC_LO 모드로 진입
    double pre_gear_change_dist_m_ = 5.0;

    // R 진입 yaw 동기화 (gear 전환 직후 ψ_ref blend)
    bool   r_align_active_ = false;
    double r_entry_yaw_ = 0.0;       // 진입 시점 vehicle yaw
    double r_align_x0_ = 0.0;        // 진입 시점 위치
    double r_align_y0_ = 0.0;
    static constexpr double kAlignDist = 2.0;   // [m] blend 거리 (0→2m)

    // R 모드 컨트롤러 선택 — RTI(default) vs Stanley/PD fallback
    bool   r_use_stanley_ = false;          // false=RTI, true=Stanley FF+PD
    double r_stanley_k_hdg_ = 0.5;          // hdg P gain
    double r_stanley_k_cte_ = 0.4;          // cte gain

    // 주차 모드 — 첫 R 진입 후 모든 D 세그먼트 저속 유지
    bool   parking_mode_ = false;
    double parking_max_kmh_ = 2.0;          // 3→2 km/h (D2 끝 sharp curve 추종)

    // R idx stuck 감지 (가짜 진행 방지 — cte 작을 때만 advance)
    int       r_idx_stuck_prev_ = -1;
    ros::Time r_idx_stuck_t_;
    bool      r_idx_stuck_init_ = false;
    static constexpr double kStuckTimeoutSec = 10.0;   // 5→10s (덜 자주)
    static constexpr int    kStuckAdvanceStep = 2;     // +5→+2 (작게 점프)
    static constexpr double kStuckMaxCte = 2.0;        // cte<2m 일 때만 advance

    // ═══════════════════════════════════════════════════════════════
    // 경로
    // ═══════════════════════════════════════════════════════════════
    std::vector<double> wp_x_, wp_y_, wp_h_, wp_k_;
    std::vector<double> wp_s_;        // 누적 path 거리 [m]
    std::vector<double> wp_avoid_off_;// 원본 path 대비 lateral offset [m] — 회피 영역 detect용
    std::vector<int>    wp_gear_;   // +1=전진(D), -1=후진(R)
    std::vector<std::pair<int,int>> gear_segments_;  // [start, end] inclusive — 같은 기어 연속 구간
    int  cur_segment_ = 0;          // gear_segments_ 내 현재 위치
    double wp_spacing_ = 0.5;

    // ═══════════════════════════════════════════════════════════════
    // 차량 상태
    // ═══════════════════════════════════════════════════════════════
    double cur_x_    = 0.0;
    double cur_y_    = 0.0;
    double cur_yaw_  = 0.0;
    double cur_v_    = 0.0;       // 절대값 (hypot)
    double cur_v_signed_ = 0.0;   // signed (velocity.x, 후진=음수)
    bool   ego_rcvd_ = false;

    // ═══════════════════════════════════════════════════════════════
    // 탐색 상태
    // ═══════════════════════════════════════════════════════════════
    int  nearest_idx_ = 0;
    bool search_init_ = false;
    double vehicle_s_ = 0.0;        // 차량의 path 진행 거리 [m] — self-overlap path 강건 매칭용
    int  cur_gear_    = 1;          // +1=D, -1=R (현재 기어 상태)
    bool gear_switching_ = false;   // 기어 전환 중 플래그
    bool gear_initialized_ = false; // 첫 틱에 MORAI에 기어 명령 보냈는지
    bool gear_switch_sent_ = false; // 정지 후 service call 보냈는지
    int  gear_switch_target_ = 1;   // 전환 대상 기어 (+1=D, -1=R)
    ros::Time gear_switch_time_;    // service call 후 dwell 시작 시각

    // ═══════════════════════════════════════════════════════════════
    // 후처리 파라미터 (ltv_mpc_node.cpp 기준값)
    // ═══════════════════════════════════════════════════════════════
    double curve_spd_hdg_thresh_ = 0.20;
    double curve_spd_gain_       = 0.06;
    double curve_spd_min_ratio_  = 0.70;
    double overshoot_dist_       = 0.10;
    double overshoot_damp_       = 0.40;
    double osc_cte_db_           = 0.10;  // 0.08 -> 0.10 (데드밴드 상향)
    double osc_hdg_db_           = 0.12;  // 원복: 데드밴드 (정상 추종 응답 보존)
    double osc_damp_             = 0.60;  // 원복: 댐핑 (응답 둔감 방지)
    double near_cte_thresh_      = 0.08;
    double near_hdg_thresh_      = 0.08;
    double near_steer_damp_      = 0.85;
    double near_v_scale_         = 0.96;
    double k_stanley_            = 0.5;
    double max_steer_rate_       = 18.0; // 원본 복원 (회피 외 영역 영향 차단)
    double max_steer_deg_        = 35.0;
    double sig_tau_up_           = 0.30;
    double sig_tau_down_         = 0.15;

    // ── 후처리 상태 ────────────────────────────────────────────────
    bool      has_prev_errors_ = false;
    double    prev_cte_        = 0.0;
    double    prev_hdg_        = 0.0;
    bool      in_recov_        = false;  // 히스테리시스 RECOV 상태 플래그
    bool      prev_was_recov_  = true;   // 직전 스텝 RECOV 여부 (NORMAL 첫 스텝 kappa 초기화용)
    bool      cmd_init_        = false;
    double    prev_steer_      = 0.0;
    bool      v_sig_init_      = false;
    double    v_sig_           = 0.0;
    ros::Time prev_cmd_time_;

    // ═══════════════════════════════════════════════════════════════
    // ROS
    // ═══════════════════════════════════════════════════════════════
    // 회피 lateral offset (legacy) — avoidance_planner_node 호환용. Phase 1 이후 미사용.
    bool   avoidance_enabled_ = false;   // launch param. false면 일반 추종 그대로
    bool   force_nmpc_ = false;          // launch param. true면 항상 RTI-NMPC (고속 튜닝용)
    double avoidance_offset_ = 0.0;
    ros::Subscriber avoid_sub_;
    // ── hdmap_lane_avoid (IDM+MOBIL) 동적 경로/속도 ──
    ros::Subscriber avoid_wps_sub_;
    ros::Subscriber avoid_vel_sub_;
    bool      use_avoid_path_       = false;
    double    avoid_target_vel_mps_ = 0.0;
    bool      avoid_vel_rcvd_       = false;
    ros::Time avoid_vel_time_;
    double    prev_avoid_cmd_kmh_   = 0.0;   // avoid 경로 속도 직접출력 rate-limit 상태

    // 동적 장애물 (Frenet 회피) — Object_topic 직접 sub
    std::vector<Obstacle> obstacles_;
    ros::Subscriber obj_sub_;
    // 디버그용 마지막 corridor (k=1)
    double last_d_min_ = -1e6;
    double last_d_max_ =  1e6;
    double last_obs_dist_s_ = -1.0;
    // 회피 활성 후 ego 정렬(cte<0.3, |yaw_err|<0.1rad) 만족까지 RECOV 차단
    bool obs_block_until_align_ = false;
    int  align_stable_count_ = 0;   // align 연속 만족 tick (cte 가로지를 때 false-positive 차단)
    int  obs_clear_count_ = 0;      // 장애물 사라진 후 경과 tick (정렬 못해도 타임아웃 해제 — deadlock 방지)

    ros::Subscriber ego_sub_;
    ros::Publisher  ctrl_pub_;
    ros::ServiceClient gear_srv_;   // /Service_MoraiEventCmd 기어 전환용 (primary)
    ros::Publisher  event_pub_;     // /InsnControl EventInfo (fallback, service 미advertise 시)
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
