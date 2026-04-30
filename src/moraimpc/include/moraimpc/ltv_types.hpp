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
    double kappa_gain = 1.5; // 3.5 -> 1.5 (Snapping 방지를 위해 대폭 하향)

    // --- MORAI 최적 파라미터 (사용자 지정값 기반) ---
    double w_dr    = 40.0;    // 80 -> 40 (라인에 붙으려는 고집을 줄임)
    double w_theta = 250.0;   // 40 -> 250 (헤딩 댐핑 대폭 강화 - 웨이브 방지 핵심)
    double w_kappa = 100.0;   
    double w_u     = 6000.0;  // 10000 -> 6000 (지연 시간 단축하여 오버슈트 방지)
    double w_u_v_gain = 1000.0; // 1500 -> 1000

    // --- 제약 조건 (BISA 구조적 제약 방식) ---
    double max_steer_deg = 35.0;
    double kappa_max =  std::tan(35.0 * M_PI / 180.0) / L;
    double kappa_min = -std::tan(35.0 * M_PI / 180.0) / L;
    
    // u_max는 물리적 한계(18deg/s)를 따르되 구조적으로 QP에서 엄격히 관리
    double u_max = 0.019; 
    double u_min = -0.019;

    double target_vel = 5.55; // 20 km/h (m/s)
};

struct MPCState {
    Eigen::VectorXd x;
    MPCState() : x(kNx) { x.setZero(); }
};

} // namespace moraimpc
