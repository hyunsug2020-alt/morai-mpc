# morai-mpc — MORAI Sim NMPC 경로 추종 + 회피

MORAI Linux Simulator에서 NMPC(Nonlinear Model Predictive Control) 기반 경로 추종 + 동적 NPC 회피.

## 브랜치 구성

| 브랜치 | 용도 |
|---|---|
| **`노트북용_nmpc_최종`** (현재) | NMPC 추종 본체 — 회피 path 생성·추종 코드, msg, launch |
| `auto-cycle-tooling` | 자동 cycle 도구 — uinput 기반 sim 자동 재시도·튜닝 |

## 최종 런치 파일

```bash
roslaunch moraimpc mpc_track_avoid.launch
```

흐름:
1. `morai_mode_init.py` — MORAI 자율주행 모드 활성
2. `path_replanner.py` — `/Object_topic` NPC 감지 → `mixed.json`에 lateral bulge 적용 → `mixed_avoid.json` 저장
3. `path_follower_node` — RTI-NMPC로 `mixed_avoid.json` 추종, gear 자동 전환
4. `simple_dashboard_node` — OpenCV path/차량/NPC/cte 실시간 시각화

## 핵심 알고리즘

### 1. 회피 path 생성 (`nodes/path_replanner.py`)
- NPC 위치 → path nearest wp 기준 lateral offset (signed)
- 같은 방향 NPC 거리 < 50m이면 **plateau bulge merge** (사이 lateral 일정)
- 방향 통일 (d 부호 합 기반, path 중앙 NPC noise 영향 제거)
- weight: **quintic S-curve** `1 - (10t³ - 15t⁴ + 6t⁵)` (양끝 곡률 0 → 매끄러운 합류)
- 비대칭 taper: `half_front=15m`, `half_back=3m`, `shift_max=1.0`

### 2. NMPC 추종 (`src/path_follower.cpp`)
- **state**: `[x, y, yaw, v, κ]`, **control**: `[a, δκ]`
- **model**: kinematic bicycle, wheelbase L=2.7m
- **arc-length s 기반 nearest 검색** (`vehicle_s_`, `wp_s_`) — self-overlapping path 강건
- **3단계 분리 NMPC weight**:

| 영역 | w_px | w_psi | w_kappa | w_akappa | target_vel |
|---|---|---|---|---|---|
| 일반 직선 추종 | 15 | 16 | 12 | 90 | 30 km/h |
| **회피 active** | **60** | **3** | **80** | 70 | **18 km/h** |
| **회피 cooldown** (30wp) | 30 | **40** | 20 | 100 | 22 km/h |

- **actuator lag 보상**: 회피 영역에서 NMPC initial state 0.12s lookahead shift
  ([Preview Path Tracking With Delay Compensation, IEEE TITS 2021](https://ieeexplore.ieee.org/document/9076791/))

### 3. 자동 NPC 감지 + lateral offset 결정 흐름
```
/Object_topic (morai_msgs/ObjectStatusList)
  ↓ path_replanner: NPC d signed = path tangent ⊥ NPC 위치
  ↓ shift = ±shift_max (d 합 부호 통일)
  ↓ quintic taper 적용 → mixed_avoid.json
  ↓ path_follower: arc-length s nearest 검색
  ↓ wp_avoid_off_[i] > 0.1 → in_avoid_active 분기
  ↓ NMPC computeControl(future ego_pose, ref_path) → ctrl_cmd_0
```

## 빌드

```bash
cd ~/morai-mpc
catkin_make
source devel/setup.bash
```

의존성:
- ROS Noetic
- OpenCV, jsoncpp (apt)
- Eigen3, osqp (apt)
- python3-evdev, xdotool (auto-cycle 사용 시)
- **morai_msgs** ROS 패키지 (`~/catkin_ws/src/morai_msgs/` 또는 `docker/ros2_morai_msgs/`)

## 주요 파일

| 파일 | 역할 |
|---|---|
| `src/moraimpc/launch/mpc_track_avoid.launch` | **최종 런치** |
| `src/moraimpc/src/path_follower.cpp` | NMPC 추종 본체 (controlLoop, findNearest, gear 전환) |
| `src/moraimpc/include/moraimpc/path_follower.hpp` | 상수 (kSearchWindow=100, kMaxIndexStep=2, max_steer_rate=25°/s) |
| `src/moraimpc/include/moraimpc/rti_nmpc_types.hpp` | RTI-NMPC config (wheelbase 2.7m, weight default) |
| `src/moraimpc/nodes/path_replanner.py` | 회피 path 생성 (quintic + plateau merge + 방향 통일) |
| `src/moraimpc/scripts/run_replanner_then_follower.sh` | replanner → follower wrapper |
| `src/moraimpc/src/simple_dashboard_node.cpp` | OpenCV 대시보드 (path 곡률 색상 + 차량 + NPC) |
| `src/moraimpc/data/mixed.json` | 원본 path waypoints |

## 알려진 한계

- 회피 정점 부근 cte ~1.2m, 통과 후 path 복귀 변동 — NMPC kinematic bicycle model이 sim 실제 차량 dynamic(slip + steer servo lag)과 mismatch
- 근본 해결: dynamic bicycle model 교체 또는 NMPC + PID feedback (별도 작업)
- path 자체의 sharp curves(R 18~50m wp[150~650])는 회피와 무관

## 참고 논문

- [Preview Path Tracking Control With Delay Compensation for Autonomous Vehicles (IEEE TITS 2021)](https://ieeexplore.ieee.org/document/9076791/)
- [Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design (UCB)](https://nuhuo08.github.io/control/IV_KinematicMPC_jason.pdf)
- [MPC-based Path Tracking Control with Forward Compensation (ScienceDirect 2021)](https://www.sciencedirect.com/science/article/pii/S2405896321016049)
