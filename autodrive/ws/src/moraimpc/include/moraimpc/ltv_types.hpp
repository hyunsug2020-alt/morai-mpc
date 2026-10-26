#pragma once
#include <cmath>
#include <vector>
#include <Eigen/Dense>

namespace moraimpc {

// 5-State LTV Model: [dr, theta, kappa, theta_r, kappa_r]
static constexpr int kNx = 5;
static constexpr int kNu = 1; // input: kappa_dot
static constexpr int kNy = 4; // output: [d1, d2, d3, kappa] (Mobility 구조)

struct LTVMPCConfig {
    // --- 구조적 설정 ---
    int N = 30;
    double Ts = 0.05;
    double L = 3.0;
    double kappa_gain = 1.28; // 조향 응답 소폭 상향: 곡선/복귀 구간 추종 강화

    // --- MORAI 최적 파라미터 (사용자 지정값 기반) ---
    double w_dr    = 220.0;   // lateral 오차 우선순위 상향
    double w_theta = 280.0;   // heading 추종 강화
    double w_kappa = 120.0;
    double w_u     = 5200.0;  // 입력 cost 완화: 조향이 더 빨리 따라붙도록 조정
    double w_u_v_gain = 350.0;  // 고속에서도 과도하게 둔해지지 않게 완화
    // Wang 2019 fuzzy adaptive: cte 클 때 w_u 동적 증가
    double w_u_cte_boost_thresh = 0.5;  // 이 이상 cte이면 boost 시작
    double w_u_cte_boost_max    = 4.0;  // 최대 4배 증가

    // --- 제약 조건 (BISA 구조적 제약 방식) ---
    double max_steer_deg = 35.0;
    double kappa_max =  std::tan(35.0 * M_PI / 180.0) / L;
    double kappa_min = -std::tan(35.0 * M_PI / 180.0) / L;
    
    // u_max는 물리적 한계(18deg/s)를 따르되 구조적으로 QP에서 엄격히 관리
    double u_max = 0.019; 
    double u_min = -0.019;

    double target_vel = 16.67; // 60 km/h (m/s)

    // --- 커브 속도 프로파일 ---
    double curve_kappa_thresh  = 0.01;   // 곡률 임계값 [rad/m] (연속 감속 시작점)
    double curve_speed_factor  = 0.5;    // 미사용 (연속 감속으로 대체)
    double curve_lookahead_m   = 18.0;   // 로컬 곡률 반응 강화
    double curve_min_vel       =  5.0 / 3.6; // 곡선 최소 5 km/h (10→5: D 끝 곡선 yaw 정렬)

    // --- 적응형 가중치 ---
    double w_theta_low_speed   = 250.0;  // 저속 sharp curve hdg overshoot 완화
    double w_theta_high_speed  = 220.0;  // 150→220 (커브 후 헤딩 진동 댐핑 강화)
    double w_theta_v_low       = 3.0;    // 저속 기준 [m/s]
    double w_theta_v_high      = 15.0;   // 고속 기준 [m/s] (~54km/h)
    double w_dr_curve_boost    = 2.8;    // 커브 진입 시 lateral 추종 우선

    // --- 동적 장애물 회피 (Frenet d 제약 + slack) ---
    double obs_lat_safety   = 0.9;   // [m] NPC lateral half-width + safety margin
    double obs_long_safety  = 2.5;   // [m] NPC longitudinal half + safety
    double obs_s_window     = 12.0;  // b1f96bb 원본값 복원 (LTV corridor 활성 거리)
    double w_slack_quad     = 1.0e5; // slack quadratic penalty
    double w_slack_lin      = 1.0e3; // slack linear penalty
    double obs_v_decel_kmh  = 5.0;   // (미사용)
    double obs_v_scale_min  = 0.15;  // 회피 활성 중 최저 속도
    double obs_cooldown_v_scale = 0.30; // 회피 종료 후 정렬 cooldown 시 v (yaw 복귀 가능 최소)
};

struct MPCState {
    Eigen::VectorXd x;
    MPCState() : x(kNx) { x.setZero(); }
};

} // namespace moraimpc
