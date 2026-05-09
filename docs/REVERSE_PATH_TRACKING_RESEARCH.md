# 자율주행차 후진 경로 추종(Reverse Path Tracking) 종합 자료

> 정리일: 2026-05-09
> 대상: morai-mpc 후진 경로 추종 구현 참고용
> 범위: 운동학 모델 · 경로 계획 · 추종 제어 · 후진 특수 난제 · 학습 기반 · 한국어 자료

---

## 목차

1. [후진 시 차량 기구학(Kinematic) 모델](#1-후진-시-차량-기구학-모델)
2. [후진 경로 계획 (Path Planning)](#2-후진-경로-계획-path-planning)
3. [경로 추종 제어 알고리즘](#3-경로-추종-제어-알고리즘)
4. [후진 시 핵심 난제](#4-후진-시-핵심-난제)
5. [트레일러/연결차량 후진 (참고)](#5-트레일러연결차량-후진-참고)
6. [자동주차 시스템에서의 후진 추종 파이프라인](#6-자동주차-시스템에서의-후진-추종-파이프라인)
7. [구현 권장 조합 (실용 가이드)](#7-구현-권장-조합-실용-가이드)
8. [한국어 자료](#8-한국어-자료-kcidbpiascienceon)
9. [참고 문헌 (Sources)](#9-참고-문헌-sources)

---

## 1. 후진 시 차량 기구학 모델

### 1.1 자전거 모델 (Bicycle Model) — 후축 기준

표준 식 (후축 중심점 `(x, y)`, 차체 헤딩 `θ`, 휠베이스 `L`, 조향각 `δ`, 종방향 속도 `v`):

```
dx/dt   = v · cos(θ)
dy/dt   = v · sin(θ)
dθ/dt   = v · tan(δ) / L
dv/dt   = a
```

### 1.2 후진 시 거동 (`v < 0`)

- 위 식에 `v < 0`을 그대로 대입하면 **헤딩 변화율 `dθ/dt`의 부호가 반전**됨.
  - 즉, 같은 조향각이라도 회전 방향이 전진 시와 반대.
- 위치 업데이트(`dx/dt`, `dy/dt`)도 반대 방향으로 흐름.
- 모델 자체는 그대로 유효하나, **제어기 입장에서는 부호 처리(sign convention)에 매우 민감**.

### 1.3 기준점 선택의 영향

| 기준점 | 전진 적합성 | 후진 적합성 | 비고 |
|---|---|---|---|
| 후축 중심 | 좋음 (Pure Pursuit 표준) | 후진 시에도 안정적, 부호만 보정 | 운동학 가장 단순 |
| 전축 중심 | Stanley 표준 | 후진 시 cross-track 정의 까다로움 | virtual axle 필요 |
| 차량 무게중심 | 일부 동역학 모델에 사용 | 운동학에는 비표준 | 조향-운동 디커플링 깨짐 |

> **저속 가정의 한계**: 슬립각 0 가정. 주차/저속 후진에는 충분하지만, 고속 또는 빙판/젖은 노면에서는 **동역학(dynamic) 모델**(타이어 횡력, Pacejka 등) 필요.

---

## 2. 후진 경로 계획 (Path Planning)

### 2.1 알고리즘 비교

| 알고리즘 | 특징 | 후진 처리 |
|---|---|---|
| **Dubins curve** | 최소 회전반경 제약, 전진만 | 후진 불가 |
| **Reeds-Shepp curve** | Dubins 확장, 전·후진 모두 허용 | **48개 패턴**, 비홀로노믹 차량의 최단 경로 |
| **Hybrid A*** | 3D state(x,y,θ) 격자 + RS 해석 확장 | 좁은 공간/주차에서 표준, **후진 비용 가산** 가능 |
| **RRT / RRT* / Bi-RRT** | 샘플링 기반, 고차원 적합 | RS 곡선과 결합해 후진 허용 |
| **B-spline 후처리** | 곡률 연속성 보장 | RS/Dubins의 곡률 불연속 보정 |
| **NMPC 최적화** | 운동학·충돌 제약 동시 처리 | 좁은 주차공간(narrow space)에서 정확 |

### 2.2 표준 파이프라인 (자동주차/후진)

```
[Hybrid A*] 격자 탐색 (전·후진 모션 프리미티브)
      ↓
[Reeds-Shepp] 분석적 확장으로 골 인접 시 closed-form 연결
      ↓
[B-spline / NMPC] 곡률 연속화 + 평활화
      ↓
실시간 추종 가능한 reference trajectory
```

### 2.3 핵심 설계 포인트

- **Reeds-Shepp 패턴 비용 가중**: 단순 거리 최소화가 아니라, 후진 구간(reverse segment) 또는 시프트 포인트(cusp, 전·후진 전환)에 가산 비용을 부여하면 사람-친화적 경로 생성.
- **곡률 연속성**: RS/Dubins의 출력은 원호+직선이라 곡률이 불연속 → 조향 입력 점프. B-spline 또는 clothoid로 보정 필수.
- **충돌 제약**: 좁은 공간은 NMPC의 polygon collision constraints가 안전.

---

## 3. 경로 추종 제어 알고리즘

### 3.1 Pure Pursuit (기하학적, 후축 기준)

조향각 식:

```
δ = atan2(2 · L · sin(α), l_d)
```

- `α`: 후축에서 lookahead 점까지의 헤딩 오차
- `l_d`: lookahead 거리 (속도 비례 권장)

#### 후진 적용 변형

1. **Reversed Pure Pursuit**:
   - lookahead 점을 **차량 뒤쪽** 경로 방향으로 잡음.
   - `α` 부호를 반전시키거나, 가상 기준점(virtual rear axle 거울 대칭)을 사용.
   - 일부 ROS Nav2 구현에서 `reversing` 모드 지원.
2. **속도 부호에 따른 자동 전환**:
   ```
   if v < 0:
       lookahead_target = backward_lookahead(path, pos, l_d)
       delta = -atan2(2*L*sin(alpha), l_d)
   else:
       (전진 표준식)
   ```

**한계**: 동역학 무시, 고속에서 추종 오차 큼, 곡률 급변 구간에서 진동.

### 3.2 Stanley Controller (전축 기준)

```
δ = ψ_e + atan(k · e / v)
```

- `ψ_e`: 헤딩 오차
- `e`: 전축에서의 cross-track 오차

#### 후진 시 문제

- `v` 부호 반전으로 `atan(k·e/v)` 항이 **발산/뒤집힘**.
- 해결: 후진 시 cross-track 오차의 부호를 반전, 또는 전축 기준을 후축 기준으로 swap (virtual axle).
- 그래도 후진에서는 Pure Pursuit 변형이나 MPC가 더 안정적.

### 3.3 Model Predictive Control (MPC / NMPC) — **자동주차/후진의 사실상 표준**

#### 일반 정식화

```
min  Σ_{k=0}^{N-1} [ ||x_k - x_ref,k||²_Q + ||u_k - u_ref,k||²_R + ||Δu_k||²_S ]
s.t. x_{k+1} = f(x_k, u_k)              (kinematic bicycle, 이산)
     u_min ≤ u_k ≤ u_max                (조향각·가감속 한계)
     |Δu_k| ≤ Δu_max                    (조향속도 한계)
     g(x_k) ≤ 0                         (충돌 제약, 도로 경계)
     v_k의 부호 = 후진 구간 부호        (gear-aware)
```

#### 후진 처리 핵심

- **Gear-aware MPC**: 예측 horizon 내에서 전·후진 전환(cusp)이 발생하는 reference면, 해당 시점에서 `v=0`을 강제하고 다음 스텝부터 부호 전환.
- **Tube-based MPC**: 외란 강건성 + 경로/속도 동시 추종 (관절차량에 효과적).
- **Linear Time-Varying MPC (LTV-MPC)**: 매 스텝 reference 주변 선형화 → QP로 풀어 실시간성 확보 (morai-mpc 저장소의 `Linear_Time-Varying_MPC.pdf` 참고).
- **NMPC + soft constraints**: 좁은 공간에서 충돌 회피 + 조향 부드러움.

#### 장점

- 조향각 변화 부드러움(`||Δu||²` 항).
- 충돌·운동학 제약 직접 반영.
- 후진 불안정성을 예측 horizon으로 안정화.

### 3.4 Sliding Mode Control (SMC)

- 트레일러 후진처럼 **언더액추에이티드·내부 불안정** 시스템에 강건성 우수.
- Sliding surface `s = e_lat + λ · e_heading` 형태로 정의.
- chattering 완화 위해 boundary layer/super-twisting 사용.
- 단일 차량 후진에는 MPC 대비 이득 적음.

### 3.5 LQR + Feedforward (곡률 기반)

- 한국정밀공학회지(2024) 김형규 외 — **곡률 기반 피드포워드 + LQR 피드백** 조합.
- 피드포워드: `δ_ff = atan(L · κ_ref)` (참조 곡률 보상).
- 피드백: 횡오차/헤딩오차 상태로 LQR.
- **후진 적용 시**: A/B 행렬에서 `v` 부호 반영, Q/R 재튜닝 필요.

### 3.6 학습 기반 (RL / DRL)

| 방법 | 적용 사례 | 성능 |
|---|---|---|
| **PPO** | 후진주차 | 100% 성공률, 최단 10초 |
| **DDPG** | car-like robot 후진 | reward = 목표 자세 오차 |
| **DRL + Imitation Learning** | 자동주차 시뮬레이션 | 98% 성공률 |

**실제 차량 적용 시 한계**: sim-to-real 갭, 안전 보장(formal safety) 부재 → 일반적으로 MPC와 결합한 hybrid가 현실적.

---

## 4. 후진 시 핵심 난제

### 4.1 내부 동역학 불안정성

- 전진은 자기 안정(self-stabilizing): 작은 조향 오차도 차체가 경로로 복귀.
- **후진은 발산**: 작은 자세 오차가 시간에 따라 증폭.
- → **예측 기반 제어(MPC)** 또는 **고이득 피드백** 필요.

### 4.2 비홀로노미 제약

- 옆방향 직접 이동 불가.
- 좁은 공간에서는 **다중 시프트(multiple cusps)** 경로 필요.

### 4.3 곡률 불연속

- Dubins/Reeds-Shepp 경로 이음새에서 조향 점프 → 액추에이터 saturation.
- B-spline, clothoid, 또는 NMPC로 평활화.

### 4.4 액추에이터 한계

- 조향각 한계 `δ_max`
- 조향속도 한계 `δ̇_max` (정지 상태에서 조향하는 "static steering"은 EPS 부하 큼)
- → MPC 제약에 직접 반영.

### 4.5 인지/센서 제약

- 후진 시 전방 라이다·카메라 약함.
- **후방 perception** 필수: 후방 카메라, USS, AVM, 후방 라이다.
- 저속에서는 **AVM(Around View Monitor) 기반 SLAM** + 휠 오도메트리가 신뢰도 높음.

### 4.6 저속 동역학

- 슬립 거의 없음 → 운동학 모델로 충분.
- 단, **정밀 위치 추정**(휠 인코더 + IMU + RTK/SLAM)이 추종 성능을 좌우.

### 4.7 사람-친화성

- 같은 경로라도 시프트 횟수가 적고 곡률이 부드러운 편이 승객/관찰자에게 자연스러움.
- 비용함수에 cusp count 패널티 권장.

---

## 5. 트레일러/연결차량 후진 (참고)

### 5.1 잭나이프(Jackknife)

- 트랙터-트레일러 hitch 각이 임계각(critical hitch angle) 초과 시 비가역 → 사고.
- **임계각**: Absolute / Directional 두 종류 (조향 목표·운영 제약에 따라).

### 5.2 제어 접근

| 방법 | 핵심 |
|---|---|
| Tube-based MPC | 외란 강건성 |
| Anti-jackknife MPC | hitch 각 제약을 OCP에 직접 포함 |
| LQR + 차동제동 | 트랙터 yaw rate, 사이드슬립, articulation angle 동시 안정화 |
| 곡률 기반 후방 추종 | 꼬리 트레일러를 기준으로 정렬 |
| SMC | 언더액추에이션 강건성 |

---

## 6. 자동주차 시스템에서의 후진 추종 파이프라인

```
[Perception]      주차공간 검출 (USS, 카메라, AVM, 라이다)
      │
[Localization]    AVM/SLAM/RTK + 휠 오도메트리
      │
[Path Planning]   Hybrid A* + Reeds-Shepp → B-spline 평활
      │
[Path Tracking]   NMPC (또는 reversed Pure Pursuit + 속도 PID)
      │
[Vehicle Actuation]  EPS 조향 + 종방향 토크/브레이크
```

### 단계별 책임

1. **환경맵 구축**: 주차공간/장애물 검출, 2D occupancy 또는 polygon map.
2. **궤적 계획(온라인)**: 시작 자세 → 목표 자세까지 전·후진 혼합 경로.
3. **실시간 추종**: 추정 자세 vs 참조 → 조향·가감속 명령.

---

## 7. 구현 권장 조합 (실용 가이드)

| 시나리오 | 추천 |
|---|---|
| 직선 짧은 후진 | reversed Pure Pursuit + 속도 PID |
| 평행/직각 주차 | Hybrid A* + Reeds-Shepp + NMPC 추종 |
| 좁은 공간/장애물 밀집 | NMPC (collision constraints, soft) |
| 트레일러 견인 | NMPC + jackknife 제약, 또는 SMC |
| 학습 기반 시도 | PPO/DDPG + 시뮬레이터(CARLA, Highway-env, MORAI) |
| 고속 후진 (드물지만) | LQR/MPC + 동역학 모델 |

### morai-mpc 프로젝트 적용 권장

- 이미 보유: LTV-MPC 자료 + reverse parking MPC 논문(`Vehicle_steering_control_with_MPC_for_target_trajectory_tracking_of_autonomous_reverse_parking.pdf`).
- 구현 우선순위:
  1. **Gear-aware LTV-MPC**: 기존 전진 LTV-MPC에 `v` 부호 처리, cusp 시점 `v=0` 강제 추가.
  2. **참조 경로 입력**: Hybrid A* + Reeds-Shepp으로 사전 생성된 경로(시뮬레이터에서 받음) 가정.
  3. **B-spline 평활**: 곡률 연속성 보장 후 MPC reference로 사용.
  4. **후진 시 lookahead/horizon 짧게**: 후진은 불안정 → horizon 길수록 발산 위험, 적절히 단축.

---

## 8. 한국어 자료 (KCI/DBpia/ScienceON)

| 제목 | 저자/출판 | 핵심 |
|---|---|---|
| 곡률 기반 경로 추종 제어 알고리즘 (LQR+예측거리) | 김형규 외, 한국정밀공학회지 2024, 41(6) | 피드포워드(예측 거리) + LQR 피드백 |
| 차선 인식/맵 기반 전환 추종 | 조민석·박기서, 대한기계학회 논문집 A, 2024 | 차동 구동 로봇, 차선/맵 기반 모드 전환 |
| CarSim 기반 추종 알고리즘 비교 | 대한전자공학회 학술대회 | 시뮬레이션 기반 비교 분석 |
| 무인운전차량 경로점 기반 경로계획 | Korea Science | 경로점(waypoint) 기반 |
| 지능형 자율주행 제어 알고리즘 개발 및 시험차량 평가 | ScienceON | 종방향 슬라이딩 + 횡방향 최적 예견 |

---

## 9. 참고 문헌 (Sources)

### 후진 추종 전반

- [The Reverse Path Tracking Control of Articulated Vehicles Based on NMPC (ResearchGate)](https://www.researchgate.net/publication/397045796_The_Reverse_Path_Tracking_Control_of_Articulated_Vehicles_Based_on_Nonlinear_Model_Predictive_Control)
- [A Simple Curvature-Based Backward Path-Tracking Control for a Mobile Robot with N Trailers (MDPI Actuators)](https://www.mdpi.com/2076-0825/13/7/237)
- [Sliding Mode Controller for Autonomous Tractor-Trailer Vehicle Reverse Path Tracking (MDPI Applied Sciences)](https://www.mdpi.com/2076-3417/13/21/11998)
- [Vehicle steering control with MPC for target trajectory tracking of autonomous reverse parking (IEEE)](https://ieeexplore.ieee.org/document/6662766/)
- [Anti-jackknife reverse perpendicular parking control of tractor-trailer vehicle via MPC (IEEE)](https://ieeexplore.ieee.org/document/9066754/)

### 기구학·기하 추종

- [Kinematic Bicycle Model — Algorithms for Automated Driving](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html)
- [The kinematic bicycle model: A consistent model for planning feasible trajectories (ResearchGate)](https://www.researchgate.net/publication/318810853_The_kinematic_bicycle_model_A_consistent_model_for_planning_feasible_trajectories_for_autonomous_vehicles)
- [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC (Medium, Yan Ding)](https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081)
- [Implementation of the Pure Pursuit Path Tracking Algorithm (CMU, Coulter 1992)](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
- [Automatic Steering Methods for Autonomous Automobile Path Tracking (CMU)](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
- [Model-Based Hybrid Control of Pure Pursuit and Stanley (MDPI Sensors)](https://www.mdpi.com/1424-8220/25/20/6491)

### 경로 계획 (전·후진)

- [Practical Search Techniques in Path Planning for Autonomous Driving (Hybrid A*, Stanford)](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
- [Reeds-Shepp Path Planner](https://linusweigand.github.io/reeds-shepp/)
- [plannerHybridAStar (MATLAB)](https://www.mathworks.com/help/nav/ref/plannerhybridastar.html)
- [Improved Hybrid A* with Lemming Optimization (MDPI)](https://www.mdpi.com/2076-3417/15/14/7734)
- [Auto parking path planning using modified Reeds-Shepp curve (ResearchGate)](https://www.researchgate.net/publication/283778223_Auto_parking_path_planning_system_using_modified_Reeds-Shepp_curve_algorithm)
- [Path-Planning for Autonomous Parking with Dubins Curves (ResearchGate)](https://www.researchgate.net/publication/320893250_Path-Planning_for_Autonomous_Parking_with_Dubins_Curves)
- [Autonomous Parking Path Planning Based on Improved RRT (MDPI)](https://www.mdpi.com/2032-6653/16/7/374)
- [Automatic parking trajectory planning in narrow spaces based on Hybrid A* and NMPC (Sci. Reports)](https://www.nature.com/articles/s41598-025-85541-x)

### MPC 기반 추종

- [A New Trajectory Tracking Algorithm Based on MPC (MDPI Sensors)](https://www.mdpi.com/1424-8220/21/21/7165)
- [Linear MPC of automatic parking path tracking with soft constraints (SAGE)](https://journals.sagepub.com/doi/10.1177/1729881419852201)
- [Model predictive path tracking control for automated road vehicles: A review (ScienceDirect)](https://www.sciencedirect.com/science/article/pii/S1367578822001377)
- [Real-Time MPC with Convex-Polygon-Aware Collision Avoidance (arXiv 2025)](https://arxiv.org/html/2505.04935v1)
- [Automated Driving Using MPC (MathWorks)](https://www.mathworks.com/help/mpc/ug/automated-driving-using-model-predictive-control.html)

### 잭나이프 / 트레일러

- [Critical hitch angle for jackknife avoidance (ResearchGate)](https://www.researchgate.net/publication/263702584_The_critical_hitch_angle_for_jackknife_avoidance_during_slow_backing_up_of_vehicle-trailer_systems)
- [Jackknifing Prevention of Tractor-Semitrailer with Active Braking (ResearchGate)](https://www.researchgate.net/publication/298972410_Jackknifing_Prevention_of_Tractor-Semitrailer_Combination_Using_Active_Braking_Control)
- [Systems and methods for preventing a jackknife condition (US Patent US20200001920A1)](https://patents.google.com/patent/US20200001920A1/en)

### 학습 기반

- [A reinforcement learning-based reverse-parking system (IET)](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/itr2.12614)
- [Reverse Parking a Car-Like Mobile Robot with Deep RL and Preview Control (IEEE)](https://ieeexplore.ieee.org/document/8666613/)
- [Reinforcement Learning-Based End-to-End Parking (MDPI Sensors)](https://www.mdpi.com/1424-8220/19/18/3996)
- [Deep RL and Imitation Learning for Autonomous Parking (MDPI Electronics)](https://www.mdpi.com/2079-9292/14/10/1992)

### 한국어 논문

- [곡률 기반 경로 추종 제어 알고리즘 (KCI, 2024)](https://www.kci.go.kr/kciportal/ci/sereArticleSearch/ciSereArtiView.kci?sereArticleSearchBean.artiId=ART003086004)
- [차선 인식/맵 기반 추종 — 대한기계학회 (DBpia)](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11859093)
- [CarSim 기반 추종 알고리즘 비교 (DBpia)](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE06385293)
- [지능형 자율주행 제어 알고리즘 개발 (ScienceON)](https://scienceon.kisti.re.kr/srch/selectPORSrchArticle.do?cn=NPAP08306936&dbt=NPAP)
- [무인운전차량 경로점 기반 경로계획 (Korea Science)](https://koreascience.kr/article/JAKO201411560020913.pdf)

---

## 10. 프로젝트 내 보유 자료 (참고)

저장소 루트에 이미 PDF 형태로 존재:

- `Linear_Time-Varying_MPC.pdf` — LTV-MPC 정식화 (gear-aware로 확장 가능).
- `Path_Tracking_Control_for_Autonomous_Vehicles_Based_on_an_Improved_MPC.pdf` — 개선된 MPC 추종.
- `Vehicle_steering_control_with_MPC_for_target_trajectory_tracking_of_autonomous_reverse_parking.pdf` — **후진 주차 MPC 핵심 자료**.
- `wevj-16-00596.pdf` — 추가 참조.

`docs/` 내 관련 문서:

- `REVERSE_GEAR_IMPLEMENTATION.md` — 후진 기어 구현 세부.
- `REVERSE_TRACKING_PROGRESS.md` — 후진 추종 진행 상황.
