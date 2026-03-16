#pragma once

#include <array>
#include <vector>

namespace bisa {

// ============================================================
// RTI-NMPC 설정 구조체
// 논문: "Real-Time Iteration NMPC for Autonomous Vehicle Control"
// 2410.12170v1.pdf 기반
// ============================================================
struct RTINMPCConfig {
  // --- 예측 파라미터 ---
  int N   = 15;     // 예측 구간 (Ts=0.04s → 0.6s)
  double Ts = 0.04; // 이산화 시간 간격 [s]

  // --- 차량 파라미터 ---
  double wheelbase = 0.30;  // 축간거리 [m]

  // --- 속도 제한 ---
  double max_velocity     =  1.0;   // 최대 전진 속도 [m/s]
  double min_velocity     = -1.0;   // 최대 후진 속도 [m/s] (음수)
  double parking_velocity =  0.3;   // 주차 추종 속도 [m/s]

  // --- 입력 제약 (u = [av, a_kappa]) ---
  double av_min    = -3.0;   // 최소 가속도 [m/s^2]
  double av_max    =  3.0;   // 최대 가속도 [m/s^2]
  double akappa_min = -2.0;  // 최소 곡률 변화율 [1/ms]
  double akappa_max =  2.0;  // 최대 곡률 변화율 [1/ms]

  // --- 상태 제약 ---
  double kappa_min = -0.9;   // 최소 곡률 [1/m] (최대 조향각)
  double kappa_max =  0.9;   // 최대 곡률 [1/m]

  // --- 비용 함수 가중치 ---
  double w_px    = 15.0;   // 위치 x 추종 가중치
  double w_py    = 15.0;   // 위치 y 추종 가중치
  double w_psi   = 8.0;    // 방향각 추종 가중치
  double w_v     = 2.0;    // 속도 추종 가중치
  double w_kappa = 1.0;    // 곡률 가중치
  double w_av    = 0.5;    // 가속도 입력 가중치
  double w_akappa = 1.0;   // 곡률 변화율 입력 가중치

  // --- SQP/RTI 파라미터 ---
  int sqp_max_iter = 1;    // RTI는 1회 SQP 반복
  int newton_max_iter = 3; // 암시적 오일러 Newton 반복

  // --- OSQP 파라미터 ---
  int osqp_max_iter   = 2000;
  double osqp_eps_abs = 1e-5;
  double osqp_eps_rel = 1e-5;
  bool osqp_warm_start = true;
  bool osqp_polish    = false;
};

// ============================================================
// RTI-NMPC 상태 벡터: x = [px, py, psi, v, kappa]^T
// ============================================================
struct RTINMPCState {
  double px    = 0.0;   // 위치 x [m]
  double py    = 0.0;   // 위치 y [m]
  double psi   = 0.0;   // 방향각 [rad]
  double v     = 0.0;   // 속도 [m/s] (음수=후진)
  double kappa = 0.0;   // 곡률 [1/m]
};

// ============================================================
// RTI-NMPC 제어 입력: u = [av, a_kappa]^T
// ============================================================
struct RTINMPCInput {
  double av     = 0.0;  // 가속도 [m/s^2]
  double a_kappa = 0.0; // 곡률 변화율 [1/ms]
};

// ============================================================
// RTI-NMPC 명령 출력 구조체
// ============================================================
struct RTINMPCCommand {
  double v_cmd      = 0.0;     // 속도 명령 [m/s]
  double omega_cmd  = 0.0;     // 각속도 명령 [rad/s]
  double kappa_cmd  = 0.0;     // 곡률 명령 [1/m]
  bool   solved     = false;   // 최적화 성공 여부
  double model_time_us  = 0.0; // 모델 구성 시간 [µs]
  double solver_time_us = 0.0; // QP 풀이 시간 [µs]

  // 예측 궤적 (시각화용)
  std::vector<std::array<double, 3>> predicted_xy;
};

// 상태벡터 차원
static constexpr int kRTINx = 5;  // [px, py, psi, v, kappa]
static constexpr int kRTINu = 2;  // [av, a_kappa]

}  // namespace bisa
