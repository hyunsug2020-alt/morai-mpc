#pragma once

#include <array>
#include <vector>

namespace moraimpc {

// RTI-NMPC 설정 구조체
// 논문: 2410.12170v1.pdf (Implicit Discretization RTI-NMPC)
struct RTINMPCConfig {
    // 예측 파라미터
    int    N  = 25;     // 15→25 (R U-turn 곡선 preview 강화 1.25s)
    double Ts = 0.05;   // 이산화 시간 간격 [s]

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

    // 비용 함수 가중치 (R U-turn 추종 — κ 강제 추종 + hdg 적당)
    double w_px     = 10.0;
    double w_py     = 10.0;
    double w_psi    = 18.0;   // 순간 hdg 보정 약화
    double w_kappa  = 40.0;   // path κ 강제 추종 (부호 일관 보장)
    double w_v      =  2.0;
    double w_av     =  0.5;
    double w_akappa =  3.0;

    // RTI/SQP 파라미터
    int sqp_max_iter   = 1;    // RTI는 1회 SQP 반복
    int newton_max_iter = 3;   // 암시적 오일러 Newton 반복

    // OSQP 파라미터
    int    osqp_max_iter   = 1000;
    double osqp_eps_abs    = 1e-4;
    double osqp_eps_rel    = 1e-4;
    bool   osqp_warm_start = true;
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
