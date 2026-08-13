# 후진 MPC 추종 — 진행 상황 & 내일 시작 가이드

## TL;DR (오늘 결과)

- **MORAI 후진 기어 동작 확인 완료** (gear=2=R, `option=2` 발행 필수)
- **path_follower.cpp 후진 지원 구현** (기어 전환, findNearest 반전, MPC 후진, RECOV 후진)
- **S자 후진 추종 검증 진행 중**:
  - 길이 15m, 진폭 1m, 파장 12m → 13초 NORMAL_R 100% 추종 완료 (CTE RMSE 22cm)
  - 길이 45m (3배) → 30초까지 NORMAL_R 추종, 끝 4초에서 누적 발산
- **마지막 튜닝**: `w_dr 40→80` (cte 누적 발산 방지). 내일 검증 필요.

---

## 핵심 발견사항

### MORAI 기어 제어 (가장 중요)
| 항목 | 값 |
|------|---|
| D (전진) | `gear=4` |
| R (후진) | `gear=2` |
| **option 필드** | **반드시 2 (gear 적용 플래그)** |
| 전송 방식 | 서비스 `/Service_MoraiEventCmd` (토픽만 보내면 안 적용됨) |
| 후진 시 속도 | 양수로 보냄 (`abs(vel_kmh)`), 기어가 R이면 자동 후진 |

### 후진 시 운동학
- LTV 모델은 `v` 부호로 후진 자동 처리: `v_profile`에 음수 넣으면 A행렬 자동 반전
- 단 MORAI 명령은 양수 (gear 부호로 방향 결정)
- heading 보정: 후진 시 `cur_yaw - path_yaw + π` (차량은 진행 반대 향함)
- findNearest의 dot product: gear_sign으로 heading 벡터 180° 반전

---

## 파일 구조 (오늘 추가/변경)

### 새 파일
| 파일 | 용도 |
|------|------|
| `nodes/test_reverse_gear.py` | R 기어 동작 검증 (gear=2 확인용) |
| `nodes/record_waypoints.py` | 수동 주행 웨이포인트 기록 (5cm 간격) |
| `nodes/make_simple_reverse_path.py` | 차량 현재 위치 기준 직선 R 경로 자동 생성 |
| `nodes/make_s_reverse_path.py` | 차량 현재 위치 기준 **S자 R 경로** 자동 생성 (양방향 ramp) |
| `scripts/smooth_waypoints.py` | 기록된 경로 후처리 (스무딩, 등간격 리샘플링) |
| `launch/mpc_s_reverse.launch` | **S자 후진 통합 실행** (경로 생성 + path_follower 동시) |
| `docs/REVERSE_GEAR_IMPLEMENTATION.md` | MORAI 후진 구현 상세 문서 |

### 수정된 파일 (git HEAD 이후)
| 파일 | 변경 |
|------|------|
| `ltv_types.hpp` | `w_dr 40→80` (cte 누적 발산 방지) |
| `path_follower.hpp` | `kMaxIndexStep 30→5`, `kRecovHdgThreshR 60°` 추가 (후진 전용) |
| `path_follower.cpp` | 후진 시 hdg 임계 분기, idx cap 항상 적용, near_dist/idx/gear 디버그 로그 추가 |
| `make_s_reverse_path.py` | 양방향 ramp (시작·끝 부드러움) |
| `mpc_s_reverse.launch` | 진폭 0.5m, 파장 18m default |

---

## 튜닝 내역 (시도-결과 시계열)

| 시도 | 결과 | 평가 |
|------|------|------|
| 직선 R 10m, target_vel=5 | 1km/h로 정확히 후진 | 직선은 OK |
| S자 (진폭 2m, 파장 7.5m) | 시작 hdg 59° 어긋남 → RECOV 발산 | 시작 부분 문제 |
| **시작 직선 추가** | 시작 OK, 곡구에서 발산 | hdg 35° 임계 초과로 RECOV |
| **launch 통합** | 차량 위치 mismatch 41m 해결 | 시작점 일치 |
| **kReverseMaxVel 15→5km/h** + S자 1m/12m | 13초 100% NORMAL_R, CTE 22cm RMSE | 짧은 경로 성공 |
| **양방향 ramp (시작·끝)** | 끝부분 cte 0.4m 발산 → 0.1m | 끝 마무리 OK |
| **길이 45m (3배)** | 14초 RECOV 진입, 발산 | sin 3주기 한계 |
| **idx cap 30 (항상)** | 14→16초 RECOV, 발산 지속 | cap 30 너무 큼 |
| **idx cap 5 + hdg 60° 후진** | 16.4→16.4초 (큰 차이 없음) | 부분적 효과 |
| **곡선 완화 (0.5m, 18m)** | **30초 NORMAL_R**, 끝 4초 발산 | 큰 진전, 끝 잡으면 됨 |
| **w_dr 40→80** ← 마지막 | 미검증 | **내일 검증** |

---

## 현재 추적 한계

- **길이 45m, 진폭 0.5m, 파장 18m, 5km/h 후진**:
  - 0~30s: NORMAL_R 정상 추종 (CTE 0.3m 내외)
  - 30~32s (끝 9m): cte 0.9→1.4m 누적 발산 → RECOV → 발산

- **20cm 이내 비율**: 18.1% (전체 45m 기준, 발산 구간 포함)
- **30s 시점까지**: 50%+ 추정 (발산 전까지)

---

## 내일 시작하는 법

### 1. 환경 준비
```bash
# 터미널 1
roscore

# 터미널 2: MORAI 시뮬레이터 띄우고 rosbridge 연결
# (Windows MORAI에서 rosbridge ws://172.30.1.22:9090)
source ~/catkin_ws/devel/setup.bash
roslaunch morai_launch morai.launch   # 또는 사용자가 쓰는 MORAI launch
```

### 2. 차량 위치 세팅
- MORAI에서 차량을 **빈 공간**(뒤가 막히지 않은 곳)에 배치
- **자율주행 모드 + D단** 상태로
- **완전 정지** 확인

### 3. 후진 추종 실행
```bash
# 터미널 3
source ~/catkin_ws/devel/setup.bash

# A. S자 후진 (45m, 진폭 0.5m, 파장 18m default)
roslaunch moraimpc mpc_s_reverse.launch length:=45.0

# B. 더 짧게 (안전 검증)
roslaunch moraimpc mpc_s_reverse.launch length:=20.0

# C. 직선 후진 (단순 검증)
rosrun moraimpc make_simple_reverse_path.py
roslaunch moraimpc mpc_tracking.launch
```

### 4. 결과 분석
```bash
# Ctrl+C로 launch 끊고 (로그 flush 보장)
python3 -c "
import json
with open('/home/coss/morai-mpc-master/src/moraimpc/logs/mpc_log.json') as f:
    data = json.load(f)
print(data['summary'])
"
```

---

## 내일 첫 검증 항목

**w_dr 40→80 변경 효과**:
- 가설: 끝 4초에서 cte 누적 발산을 막아 NORMAL_R 끝까지 유지
- 측정: 45m 풀 코스에서 RECOV 진입 시점 / 20cm 이내 비율

**예상 결과**:
- ✅ 좋아짐: RECOV 진입 안 함, 20cm 이내 70%+ → 그대로 진행
- ⚠ 부작용: 직선 부분 좌우 흔들림(오버슈팅) → w_dr 60으로 후퇴 또는 다른 파라미터

---

## 결정 보류 (내일 또는 그 후)

1. **Hybrid A* 플래너 도입** (별도 패키지 `morai_parking`)
2. **acados NMPC RTI** 도입 (현재 LTV-MPC 한계 시)
3. **Apollo open_space_planner 코드 분석**

논문 baseline은 후순위. 일단 현재 구현 완성 우선.

---

## 핵심 파라미터 빠른 참조

```cpp
// ltv_types.hpp
target_vel        = 60 km/h (D 기준, R은 cap 5km/h)
w_dr              = 80         ← 오늘 변경
w_theta_low       = 350
w_theta_high      = 150
kappa_gain        = 1.2

// path_follower.hpp
kMaxIndexStep     = 5          ← 오늘 변경
kRecovDist        = 1.5
kRecovHdgThresh   = 35°  (전진)
kRecovHdgThreshR  = 60°  (후진) ← 오늘 추가
kRecovMaxVel      = 3 km/h
kReverseMaxVel    = 5 km/h
```

```python
# launch/mpc_s_reverse.launch defaults
length     = 15 m   # roslaunch에서 인자로 변경
amplitude  = 0.5 m
wavelength = 18 m
spacing    = 0.1 m
```
