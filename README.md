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
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
catkin_make
source devel/setup.bash
```

의존성:
- ROS Noetic
- OpenCV, jsoncpp (apt)
- Eigen3, osqp (apt)
- python3-evdev, xdotool (auto-cycle 사용 시)
- **morai_msgs** ROS 패키지 (`~/catkin_ws/src/morai_msgs/` 또는 `docker/ros2_morai_msgs/`)

## 대회 통신 환경 모사 기준

현재 개발 환경에서는 센서를 UDP로 유지하고 차량 통신만 ROS WebSocket으로
연결함. 대회에서 허용되는 데이터만 사용하기 위해 다음 입출력으로 제한함.

- 3D LiDAR: UDP `2368`
- GPS: UDP `9090`
- IMU: UDP `9091`
- 전·좌·우·후 카메라: UDP `9092`~`9095`
- `/Competition_topic`: 차량 속도·조향 등 제한된 차량 상태만 사용
- `/CollisionData`: 충돌 여부 확인에만 사용
- `/ctrl_cmd`: 차량 제어 출력. 종방향 제어는 `longlCmdType=1`의
  가속·브레이크 방식 사용

대회 런타임에서 사용하지 않는 입력:

- `/Ego_topic`
- `/Object_topic`
- Ego 실제 위치·heading 등 Ground Truth
- 대회 허용 목록에 없는 MORAI 토픽

따라서 WebSocket 연결은 유지하되 차량 통신에서는 `/Competition_topic`,
`/CollisionData`, `/ctrl_cmd`만 사용하면 대회 입력 조건과 가장 유사함.
단, WebSocket은 UDP의 패킷 손실·순서 변경·지연 특성까지 재현하지 않으므로
UDP 전송 특성 자체를 검증하는 환경은 아님.

## 3카메라 어라운드뷰

`src/around_view`에 전방·좌측·우측 카메라 기반 어라운드뷰가 포함되어 있음.
카메라 위치·회전·FOV·입력 해상도와 기존 보정값은 `heo_ws` 설정을 그대로
사용함. 후방 카메라는 합성에 사용하지 않음.

어라운드뷰만 실행:

```bash
source devel/setup.bash
roslaunch around_view around_view.launch
```

MORAI 연결, LIO-SAM, ESKF와 함께 실행:

```bash
source devel/setup.bash
roslaunch morai_launch morai.launch start_around_view:=true
```

통합 launch에서 어라운드뷰는 기본적으로 꺼져 있으므로 필요할 때만
`start_around_view:=true`를 지정함.

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
| `src/eskf/` | ROS 노드·설정·8-state 코어·LIO/오도메트리 정렬·GPS 상태기계·지연 GPS replay를 모두 포함한 독립 catkin 패키지 |
| `src/eskf/scripts/eskf_node.py` | 독립 ESKF 패키지의 ROS 토픽 래퍼 |

ESKF는 기존 `/gps`, `/imu/data`, `/lio_sam/mapping/odometry` 입력과
`/eskf/odom`, `/eskf/diagnostics` 출력을 유지함. 지연 GPS는 최대 2초의 IMU
이력을 되감아 측정 시각에 반영한 후 다시 전파하며, wheel 또는 SLAM 보정이
들어온 구간 앞까지는 되감지 않아 다른 센서 보정을 지우지 않음.
`/odometry/pure`의 속도·GPS 음영 상대 위치와
`/pure_odometry/diagnostics`의 MORAI 물리시간 비율도 함께 사용함.

실시간 ESKF 궤적·오차·RMSE를 MORAI Ego 참값과 비교하는 읽기 전용 GUI:

```bash
roslaunch eskf monitor_gui.launch
```

GUI는 `/Ego_topic`을 화면 검증에만 사용하며 ESKF 또는 제어기로 다시 보내지
않음. 설정 변경 기능이 없으므로 실행 중인 ESKF 수치도 바꾸지 않음.

## 순수 오도메트리 IMU 오차 대응

`/odometry/pure`는 차량 전진 속도·조향각과 IMU만 사용하는 GPS 음영 백업
오도메트리다. Ego 위치·heading, GPS, LiDAR를 추정 입력으로 사용하지 않는다.

- IMU quaternion으로 gyro 적분 yaw drift를 제한한다.
- 정지 구간에서 gyro bias와 종가속도 bias를 갱신한다.
- IMU 종가속도는 중력 투영을 제거하고 Huber gate를 적용한다.
- 종가속도는 위치로 이중 적분하지 않는다. 직선 구간의 MORAI 물리시간 비율
  추정에만 사용해 가속도 noise·bias의 위치 발산을 방지한다.
- 회전 가능한 구간은 quaternion/gyro 시간비율을 우선하며, 회전 관측이 없는
  직선 구간만 종가속도/속도 변화량으로 보완한다.

10분 실제 기록(`577.2 s`, `2444.6 m` 연속 구간) 재생 결과는 위치 RMSE
`8.07 m`, yaw RMSE `0.281°`, 종점 drift `0.255%`다. 기존 `10 s` 시간창의
`8.81 m`, `0.354%`보다 개선됐다. 실시간 UDP 입력에서도 최종 확인이 필요하며,
Ego 참값은 검증에만 사용한다.

## 알려진 한계

- 회피 정점 부근 cte ~1.2m, 통과 후 path 복귀 변동 — NMPC kinematic bicycle model이 sim 실제 차량 dynamic(slip + steer servo lag)과 mismatch
- 근본 해결: dynamic bicycle model 교체 또는 NMPC + PID feedback (별도 작업)
- path 자체의 sharp curves(R 18~50m wp[150~650])는 회피와 무관

## 참고 논문

- [Preview Path Tracking Control With Delay Compensation for Autonomous Vehicles (IEEE TITS 2021)](https://ieeexplore.ieee.org/document/9076791/)
- [Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design (UCB)](https://nuhuo08.github.io/control/IV_KinematicMPC_jason.pdf)
- [MPC-based Path Tracking Control with Forward Compensation (ScienceDirect 2021)](https://www.sciencedirect.com/science/article/pii/S2405896321016049)
