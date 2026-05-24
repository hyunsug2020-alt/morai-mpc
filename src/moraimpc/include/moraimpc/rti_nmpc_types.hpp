#pragma once

#include <array>
#include <vector>

namespace moraimpc {

// RTI-NMPC 설정 구조체
// 논문: 2410.12170v1.pdf (Implicit Discretization RTI-NMPC)
struct RTINMPCConfig {
    // 예측 파라미터 (N 60→80: 4s 미래 예측, 곡선 사전 대응 ↑)
    int    N  = 80;
    double Ts = 0.05;

    // IONIQ 5 차량 파라미터
    double wheelbase = 3.000;  // 축간거리 [m]

    // 속도 제한
    double max_velocity     =  30.0;   // 최대 전진 속도 [m/s]
    double min_velocity     =  -3.0;   // 최대 후진 속도 [m/s]
    double target_velocity  =  20.83;  // 기본 목표속도 75 km/h [m/s]

    // 입력 제약 u = [av, a_kappa]
    double av_min     = -5.0;    // 최소 가속도 [m/s²]
    double av_max     =  3.0;    // 최대 가속도 [m/s²]
    double akappa_min = -0.5;    // 최소 곡률 변화율 [1/m·s]
    double akappa_max =  0.5;    // 최대 곡률 변화율 [1/m·s]

    // 상태 제약 (δ_max=40deg → κ_max = tan(40deg)/3.0)
    double kappa_min = -0.280;   // 최소 곡률 [1/m]
    double kappa_max =  0.280;   // 최대 곡률 [1/m]

    // 비용 함수 가중치 (저속/R 기준 best — 고속은 path_follower eff_cfg에서 override)
    double w_px     = 20.0;
    double w_py     = 20.0;
    double w_psi    = 10.0;
    double w_v      =  2.0;
    double w_kappa  =  2.0;
    double w_av     =  0.5;
    double w_akappa =  4.0;

    // LTV 기법 이식 (C29) — path_follower eff_cfg와 중복 시 0으로 비활성
    double w_av_v_gain      = 0.0;    // R(0,0) v 비례 (path_follower에서 처리)
    double w_akappa_v_gain  = 0.0;    // R(1,1) v 비례 (path_follower에서 처리)
    double w_psi_low_speed  = 10.0;   // 저속 hdg (default)
    double w_psi_high_speed = 10.0;   // 고속 hdg (default)
    double w_psi_v_low      = 3.0;
    double w_psi_v_high     = 15.0;
    double w_pos_curve_boost = 1.0;   // off (path_follower cte_boost 사용)
    double curve_kappa_thresh = 0.05;

    // RTI/SQP 파라미터 (R1 곡선 정밀 강화)
    int sqp_max_iter    = 5;
    int newton_max_iter = 15;  // 10→15

    // OSQP 파라미터
    int    osqp_max_iter   = 2000;
    double osqp_eps_abs    = 1e-5;
    double osqp_eps_rel    = 1e-5;
    bool   osqp_warm_start = true;

    // ── 장애물 회피 (방안 B: NMPC stage 제약) ──────────────────────
    // 비선형 거리 제약을 nominal trajectory 둘레로 1차 Taylor → 선형 half-space
    // soft slack penalty로 infeasibility 방지
    bool   obs_enable        = false;   // launch에서 true 설정 시만 활성
    double obs_safe_margin   = 1.0;     // 1.5→1.0 회피 거리 축소
    double obs_active_dist   = 8.0;     // 30→8 활성 거리 축소 (회피 조기 트리거 방지)
    int    obs_max_count     = 5;       // 동시 처리 최대 obstacle 수
    int    obs_skip_first    = 1;       // k=0..skip-1 stage는 제약 skip (현재 ego 위치)
    double w_obs_slack_quad  = 1e5;     // slack 2차 penalty
    double w_obs_slack_lin   = 1e3;     // slack 1차 penalty
};

// RTI-NMPC 상태 벡터 x = [px, py, psi, v, kappa]^T
struct RTINMPCState {
    double px    = 0.0;   // 위치 x [m]
    double py    = 0.0;   // 위치 y [m]
    double psi   = 0.0;   // 방향각 [rad]
    double v     = 0.0;   // 속도 [m/s] (음수=후진)
    double kappa = 0.0;   // 곡률 [1/m]
};

// RTI-NMPC 제어 입력 u = [av, a_kappa]^T
struct RTINMPCInput {
    double av      = 0.0;  // 가속도 [m/s²]
    double a_kappa = 0.0;  // 곡률 변화율 [1/m·s]
};

// 장애물 (NMPC stage 제약용 — circle approximation)
struct RTINMPCObstacle {
    double cx     = 0.0;    // 중심 x [m]
    double cy     = 0.0;    // 중심 y [m]
    double r_safe = 1.5;    // 안전 반경 (r_obstacle + margin) [m]
    double vx     = 0.0;    // 속도 x [m/s] (동적, 옵션)
    double vy     = 0.0;    // 속도 y [m/s] (동적, 옵션)
};

// RTI-NMPC 명령 출력
struct RTINMPCCommand {
    double v_cmd      = 0.0;    // 속도 명령 [m/s]
    double omega_cmd  = 0.0;    // 각속도 명령 [rad/s]
    double kappa_cmd  = 0.0;    // 곡률 명령 [1/m]
    double steer_deg  = 0.0;    // 조향각 [deg] (MORAI 전송용)
    bool   solved     = false;
    double model_time_us  = 0.0;
    double solver_time_us = 0.0;
    std::vector<std::array<double, 3>> predicted_xy;  // [px, py, psi] 시각화용
};

static constexpr int kRTINx = 5;  // [px, py, psi, v, kappa]
static constexpr int kRTINu = 2;  // [av, a_kappa]

}  // namespace moraimpc
