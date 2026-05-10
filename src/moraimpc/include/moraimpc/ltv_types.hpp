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
    double L = 2.7;
    double kappa_gain = 1.2; // 1.5 -> 1.2 (저주파 진동 억제)

    // --- MORAI 최적 파라미터 (사용자 지정값 기반) ---
    double w_dr    = 150.0;   // 80→150 (cte 누적 방지 — 위치 우선순위 ↑, hdg overshoot 완화)
    double w_theta = 250.0;   // 40 -> 250 (헤딩 댐핑 대폭 강화 - 웨이브 방지 핵심)
    double w_kappa = 100.0;   
    double w_u     = 7500.0;  // 기본 input cost
    double w_u_v_gain = 600.0;  // 1200→600 (고속 sluggishness 절반 — cte 누적 방지)
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
    double curve_lookahead_m   = 25.0;   // 곡률 프리뷰 거리 [m] (고속 대응)
    double curve_min_vel       =  5.0 / 3.6; // 곡선 최소 5 km/h (10→5: D 끝 곡선 yaw 정렬)

    // --- 적응형 가중치 ---
    double w_theta_low_speed   = 250.0;  // 350→250 (저속 sharp curve hdg overshoot ±11° 완화)
    double w_theta_high_speed  = 150.0;  // 200→150 (고속은 이미 거의 완벽, 추가 진동 방지)
    double w_theta_v_low       = 3.0;    // 저속 기준 [m/s]
    double w_theta_v_high      = 15.0;   // 고속 기준 [m/s] (~54km/h)
    double w_dr_curve_boost    = 2.0;    // 커브 진입 시 w_dr 배수
};

struct MPCState {
    Eigen::VectorXd x;
    MPCState() : x(kNx) { x.setZero(); }
};

} // namespace moraimpc
