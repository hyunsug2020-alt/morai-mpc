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
    double w_dr    = 80.0;    // 40 -> 80 (후진 누적 cte 발산 방지: 측방 대시각 고집 강화)
    double w_theta = 250.0;   // 40 -> 250 (헤딩 댐핑 대폭 강화 - 웨이브 방지 핵심)
    double w_kappa = 100.0;   
    double w_u     = 7500.0;  // 6500→7500 (커브 후 오버슈팅 댐핑)
    double w_u_v_gain = 1200.0; // 1000→1200

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
    double curve_min_vel       = 10.0 / 3.6; // 최소 속도 10 km/h → m/s

    // --- 적응형 가중치 ---
    double w_theta_low_speed   = 350.0;  // 저속 w_theta
    double w_theta_high_speed  = 200.0;  // 150→200 (고속 헤딩 잡아 오버슈팅 억제)
    double w_theta_v_low       = 3.0;    // 저속 기준 [m/s]
    double w_theta_v_high      = 15.0;   // 고속 기준 [m/s] (~54km/h)
    double w_dr_curve_boost    = 2.0;    // 커브 진입 시 w_dr 배수
};

struct MPCState {
    Eigen::VectorXd x;
    MPCState() : x(kNx) { x.setZero(); }
};

} // namespace moraimpc
