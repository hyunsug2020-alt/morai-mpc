# MORAI 전진+후진 혼합 경로 추종 (MPC)

## 개요
MORAI 시뮬레이터에서 LTV-MPC 기반 경로 추종기에 후진(R) 기어 지원을 추가하여,
전진(D) → 정지 → 후진(R) → 정지 → 전진(D) 혼합 경로를 자동 추종할 수 있도록 구현.
주차 시나리오 등에 활용 가능.

## MORAI 기어 제어 핵심 발견사항

### 기어 값
| 값 | 기어 |
|----|------|
| 1  | P (주차) |
| 2  | R (후진) |
| 3  | N (중립) |
| 4  | D (전진) |

### 기어 전환 방법
- **서비스 호출** 방식 사용: `/Service_MoraiEventCmd` (MoraiEventCmdSrv)
- **`option` 필드가 핵심**: `EventInfo.option`을 반드시 설정해야 기어 전환이 적용됨
  - `option=1`: ctrl_mode만 적용
  - `option=2`: gear만 적용
  - `option=3`: ctrl_mode + gear 모두 적용
- `option`을 설정하지 않으면 (기본값 0) 기어 전환이 무시됨
- 토픽 발행(`/InsnControl`)만으로는 기어 전환 안 됨 → 서비스 필수

### 후진 시 속도 명령
- MORAI에 **양수 속도**를 전달 (기어가 R이면 자동으로 후진)
- `CtrlCmd.velocity`에 음수를 넣으면 안 됨

## 구현 내용

### 1. 경로 포맷 확장 (`waypoints.json`)
웨이포인트에 `"gear"` 필드 추가 (선택사항, 없으면 "D"):
```json
{
  "waypoints": [
    {"x": 10.0, "y": 100.0},
    {"x": 11.0, "y": 101.0, "gear": "R"},
    {"x": 10.5, "y": 100.5, "gear": "R"},
    {"x": 11.0, "y": 100.0}
  ]
}
```

### 2. 기어 전환 관리 (`path_follower.cpp`)
- `controlLoop()` 시작부에서 현재 웨이포인트의 gear와 현재 기어 비교
- 기어 변경 필요 시: **정지 → 서비스로 기어 전환 → 0.5초 대기 → 주행 재개**
- `MoraiEventCmdSrv` 서비스 사용, `option=2` 설정

### 3. findNearest() 후진 대응
- 전진 시: heading 방향 기준 dot product 필터
- **후진 시**: heading 벡터를 180° 반전하여 탐색
  ```cpp
  double gear_sign = (cur_gear_ < 0) ? -1.0 : 1.0;
  const double hx = gear_sign * cos(cur_yaw_);
  ```
- heading error도 π 보정: `wrapAngle(cur_yaw_ - path_yaw + M_PI)`

### 4. MPC 후진 dynamics
- **LTV 모델 자동 반전**: `v_profile`에 음수 속도를 넣으면 A행렬의 `Ac(0,1)=v`, `Ac(1,2)=v`가 자연스럽게 후진 dynamics를 반영
- 후진 최대 속도 제한: 15 km/h (`kReverseMaxVel`)
- `publishCmd()`에서 `std::abs(vel)`로 양수 변환 후 MORAI에 전달

### 5. RECOV 모드 후진 대응
- 후진 구간에서 RECOV 진입 시 look-ahead bearing에 π 보정
- 후진 시에도 경로 인덱스 증가 방향으로 look-ahead 유지

## 변경 파일 목록

| 파일 | 변경 내용 |
|------|----------|
| `include/moraimpc/path_follower.hpp` | `wp_gear_`, `cur_gear_`, `gear_srv_` 등 추가 |
| `src/path_follower.cpp` | 기어 전환, findNearest 반전, MPC 후진, publishCmd 수정 |
| `nodes/morai_mode_init.py` | `option=3` 추가 |
| `nodes/test_reverse_gear.py` | R기어 테스트 스크립트 (서비스 방식) |
| `nodes/record_waypoints.py` | 수동 주행 웨이포인트+기어 기록 스크립트 |

## 사용법

### 1. R기어 동작 확인 (최초 1회)
```bash
rosrun moraimpc test_reverse_gear.py
```

### 2. 주차 경로 기록
```bash
rosrun moraimpc record_waypoints.py
# MORAI에서 키보드로 전진/후진 주행 → Ctrl+C로 저장
# → data/waypoints_recorded.json
```

### 3. 경로 적용 및 실행
```bash
cp src/moraimpc/data/waypoints_recorded.json src/moraimpc/data/waypoints.json
roslaunch moraimpc mpc_tracking.launch
```

### 4. 상태 모니터링
```bash
rostopic echo /mpc_status
# NORMAL   : 전진 MPC
# NORMAL_R : 후진 MPC
# RECOV    : 전진 복구
# RECOV_R  : 후진 복구
```

## 주의사항
- 후진 구간의 웨이포인트는 **진행 방향(후방)** 순서로 나열해야 함 (인덱스가 증가하는 방향 = 차량이 이동하는 방향)
- 전진↔후진 전환점에서 차량이 완전히 정지한 후 기어가 전환됨 (0.5초 대기)
- 후진 시 steering convention은 전진과 동일 (MORAI 기준)
