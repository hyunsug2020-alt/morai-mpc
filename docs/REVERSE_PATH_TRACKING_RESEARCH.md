# 자율주행 후진 경로 추종 — 보유 논문 4편 정독 정리

> 정리일: 2026-05-10
> 출처: 저장소 루트 PDF 4편 본문 직접 정독
> 목적: morai-mpc 후진 추종 구현 시 직접 인용·구현 가능한 수준의 정확한 정리

---

## 0. 4편 한눈에 비교

| # | 논문 | 연/저자 | 핵심 | 후진 직접 다룸 |
|---|---|---|---|---|
| 1 | Vehicle Steering Control with MPC for Target Trajectory Tracking of Autonomous Reverse Parking | 2013 / Tashiro (Osaka Sangyo Univ.) | x-y 기반(시간 무관) MPC, 후진 주차 전용 | ✅ 핵심 |
| 2 | Lateral Vehicle Trajectory Optimization Using Constrained Linear Time-Varying MPC | 2017 / Gutjahr·Gröll·Werling (BMW + KIT) | LTV-MPC, 5-상태, QP, 차원충돌제약 | ❌ 횡방향만 (속도 v(t) 입력) |
| 3 | Path Tracking Control for Autonomous Vehicles Based on an Improved MPC | 2019 / Wang·Liu (Beijing Jiaotong) | 동역학 모델 + 퍼지 적응 가중 LMPC | ❌ 전진 |
| 4 | The Reverse Path Tracking Control of Articulated Vehicles Based on NMPC (wevj-16-00596) | 2025 / Liu·Bai (USTB) | 후축 기준점 재정의 + NMPC, 광산 관절차량 | ✅ 핵심 |

→ **morai-mpc 후진 구현 직접 참고 1순위: [1] Tashiro 2013, [4] Liu·Bai 2025.**

---

## 1. Tashiro 2013 — Reverse Parking MPC (가장 직접적인 참조)

### 1.1 문제 설정 (논문이 다루는 범위)

- **단방향 조향 후진 주차**만 다룸 (Fig. 2). 즉 **switch-back(전·후진 전환) 없음**.
- 평행주차의 switch-back은 hybrid dynamical system으로 별도 정의 필요 — **본 논문 범위 외**.
- 좌표계: 최종 목표 위치 (0, 0), 최종 헤딩 θ = π/2.

### 1.2 모델 (식 1~5)

차량 회전 모델 (Fig. 3, Ackermann 기반):

```
r = L / tan(δ)                                  ... (1)  회전반경
xc = x + r·sin(θ),  yc = y - r·cos(θ)           ... (2)  회전중심
φ = v·T / r                                              회전각

x_{t+T} = (x_t - xc_t)·cos(φ_t) - (y_t - yc_t)·sin(φ_t) + xc_t
y_{t+T} = (x_t - xc_t)·sin(φ_t) + (y_t - yc_t)·cos(φ_t) + yc_t
θ_{t+T} = φ_t + θ_t                              ... (3)
```

조향 모델 (1차 지연, EPS):
```
r_{t+1} = (1 - τ)·r_t + τ·u_t                    ... (4)
```
- `u`: 요청 회전반경 (= 요청 조향각과 등가)
- `τ`: 시상수 관련 상수.

종방향: 별도 longitudinal controller가 목표속도 추종 — **추종 제어기는 속도를 외부 입력으로만 받음**.

### 1.3 핵심 아이디어 — "시간이 아니라 방향이 일치할 때 비교"

기존 MPC 비용함수 (식 6):
```
V = Σ {(x̂ - x)² + (ŷ - y)² + α(θ̂ - θ)² + β(r_t - r_{t-1})²}
```
→ 같은 timing에서 비교. 차량 속도 오차 → 위치 오차로 직결.

**Tashiro의 변형 (식 7~9)**: target과 predicted를 **방향(heading)이 일치하는 시점**에 비교.

```
조건: θ̃_{t+T} = θ̂_{t+T}  (예측 헤딩이 목표 헤딩과 같아질 때)

→ x̃_{t+T} = r_t·sin(θ̂_{t+T}) - r_t·sin(θ̂_t) + x_t
   ỹ_{t+T} = r_t·cos(θ̂_t)   - r_t·cos(θ̂_{t+T}) + y_t

V₁ = Σ_{j=1..n} { (x̂ - x̃)² + (ŷ - ỹ)² + β(r_{t+jT} - r_{t+(j-1)T})² }
```

θ̃ = θ̂이므로 θ 항은 0이 되어 사라짐.

**효과**: 차량 속도 오차로 인한 timing 오차가 비용함수에 반영되지 않음 → 속도 변동에 robust.

### 1.4 시뮬레이션 결과 (검증된 수치)

| 항목 | 값 |
|---|---|
| 휠베이스 L | 2.5 m |
| 목표 차속 | 9 km/h |
| 제어주기 | 40 ms |
| 예측주기 | 40 / 120 / 240 ms (가변) |
| Horizon n | 4 |
| 가중치 β | 1e-5 |
| τ (조향 1차지연 시상수) | 0.25 s |

**관찰된 trade-off**:
- 예측주기 짧음 → 추적 정확도 ↑, 조향 급변 ↑
- 예측주기 김 → 조향 부드러움 ↑, 목표 추적 다소 deteriorate

**강건성** (검증):
- 차속 오차 ±2 km/h (7 / 9 / 11 km/h) → 모두 추적 성공.
- 조향 응답 시상수 ±30% 오차 → 추적 성공.
- 조향각 상한 도달하는 시나리오 (case ii) → 긴 예측주기가 max 조향각을 작게 만듦.

### 1.5 한계 / 명시된 가정

- 위치·방향은 **지연 없이 정확히 측정** 가정.
- 직진은 별도 처리 (조향 0 명령).
- **switch-back 없음** — 평행주차에는 hybrid system 확장 필요.

---

## 2. Liu·Bai 2025 — Articulated Reverse NMPC (구조적 통찰)

### 2.1 문제 정의 — 광산 관절차량(MAV) 한정이지만 통찰은 일반적

- LHD, mining truck 등 **active articulated steering** 차량.
- 세미트레일러와 다름: 능동 조향 관절.
- 후진은 **전체 운반 작업의 50% 이상** 차지(LHD 기준).

### 2.2 핵심 발견 — "센서 위치 vs. 진행 방향" 모순

전진 시:
- 측위장치는 **원래 전축**에 설치 (전축 기준 운동학이 잘 정립됨).
- 후축은 자유.

후진 시:
- 차량의 진행 방향이 뒤집히면, **원래 전축이 사실상 새로운 후축**이 되고, 측위장치는 그 후축 위에 있게 됨.
- 새로운 "전축"(원래 후축)은 측위 없음.
- 전축 기준 운동학에서는 **측위 데이터로부터 새로운 전축 자세를 추론**해야 하는데, 전·후축 자세 간 강한 비선형 관계 → 큰 오차.

→ **결론: 후진 시에는 후축 기준 운동학으로 재정식화해야 함.**

### 2.3 부호·기준계 처리 (식 1~4)

측위장치 출력 헤딩 `θ_p`가 reference path 진행 방향과 어긋남(±180°에 가까움) → 반복적으로 ±180° 보정:

```
θ_0 = θ_p
loop:
  if  θ_i - θ_ref < -90°: θ_{i+1} = θ_i + 180°
  elif θ_i - θ_ref > +90°: θ_{i+1} = θ_i - 180°
  else: 종료
θ_ar = θ_i  (실제 후축 헤딩)
```

종방향 속도와 관절각도 부호 반전:
```
v_ar = -v_p
γ    = -γ_p
```
이렇게 해서 알고리즘 내부에서는 `v_ar > 0` 기준으로 처리.

### 2.4 후진용 운동학 재정식화 (식 5~8)

기존(전축 기준) 관절차량 운동학:
```
v_f = v_r·cos(γ) + θ̇_r·l_r·sin(γ)
θ̇_f·l_f = v_r·sin(γ) - θ̇_r·l_r·cos(γ)
γ = θ_f - θ_r
```

후축 기준으로 정리하면 (식 6):
```
θ̇_r = [v_r·sin(γ)] / [l_f + l_r·cos(γ)]  -  [ω_γ·l_f] / [l_f + l_r·cos(γ)]
```

후진용 최종 모델 (식 8, "원래" 전축이 사실상 후축이 됨):
```
ẋ_ar = v_ar·cos(θ_ar)
ẏ_ar = v_ar·sin(θ_ar)
θ̇_ar = v_r·sin(γ)/(l_or + l_of·cos(γ))  -  ω_γ·l_or/(l_or + l_of·cos(γ))
γ̇    = ω_γ
```

### 2.5 NMPC 정식화

상태/입력:
```
x = [x_ar, y_ar, θ_ar, γ]^T
u = [ω_γ]
```

Euler 이산화 후 (식 11):
```
x(t+k|t) = x(t+k-1|t) + T·f(x(t+k-1|t), u(t+k|t))
```

비용함수 (식 13):
```
min J = Σ_{k=1..p} ‖x(t+k|t) - x_ref(t+k|t)‖²_Q
s.t. -ω_γ_max ≤ ω_γ ≤ ω_γ_max
     -γ_max   ≤ γ   ≤ γ_max
```

가중치 선택(저자 권장):
- `q_x = q_y = q_θ = 1`, `q_γp = 0`
- 헤딩/평활 가중치 >> 변위 가중치 → 안정성·평활성 우수, 직선 잔여오차 일부 수용.

### 2.6 시뮬레이션 결과 (검증된 수치)

| 항목 | 값 |
|---|---|
| 제어주기 T | 50 ms |
| 예측 horizon p | 100 |
| 제어 horizon c | 2 |
| 속도 | 2 / 3 m/s |
| 경로 | U-shape, 직선 + 반경 R=20/25/30 m 원호 |
| 실패 기준 | 횡오차 > 1 m |

**제안 NMPC 결과**:
- 변위오차 진폭 ≤ **0.101 m** (R=30, v=2 m/s)
- 헤딩오차 진폭 ≤ **0.0372 rad** (R=30, v=3 m/s)
- 곡률 증가(R 30→25→20) 시 변위오차 0.074→0.089→0.112 m (소폭 증가)
- 1주기당 최대 솔버 시간 **0.0065 s** (제어주기 50 ms 충분히 만족)
- 솔버 평균 반복 4.94회, 최악 7회

**비교군 4종 모두 후진에서 발산**:
- NMPC (전진용, 출력 부호만 반전): heading error -3 ~ -4 rad 범위로 발산.
- 전축모델 기반 reverse NMPC: 점진적 발산.
- 후진 LMPC: 비볼록성으로 인해 NMPC보다 오히려 계산시간 더 길고, 추적도 발산.
- Stanley: 관절각속도 제약 위반.

### 2.7 한계

- 실시간성은 충분하나 고성능 컴퓨팅 필요 → 산업현장 비용 부담.
- 차속 과도 + 관절각/관절각속도 한계 초과 시 어떤 제어기도 추적 불가.
- 실차 실험 미진행 (Gazebo 시뮬까지만).

---

## 3. Gutjahr 2017 — Constrained LTV-MPC (참고: 횡방향 추적 일반론)

### 3.1 위치 및 한계

- BMW Group + KIT 공동 연구. **횡방향 안내**(lateral guidance)에 집중. 후진 자체를 다루지 않음 — 속도 v(t)를 **외부 시변 파라미터**로 받아 처리.
- 5상태 LTV-MPC를 단일 QP로 풀어 저성능 ECU에서도 ms 단위로 동작.

### 3.2 모델 (식 1, 2)

상태 벡터:
```
x = [d_r, θ, κ, θ_r, κ_r]^T
```
- `d_r`: 후축 중심에서 reference curve로의 법선거리
- `θ, κ`: 차량 헤딩·곡률
- `θ_r, κ_r`: reference 헤딩·곡률 (base point 기준)

비선형 식:
```
ḋ_r = v(t)·sin(θ - θ_r)
θ̇   = v(t)·κ
κ̇   = u                        (입력 = 곡률 변화율)
θ̇_r = v(t)·cos(θ - θ_r)/(1 - d_r·κ_r) · κ_r
κ̇_r = z                        (외부 신호)
```

선형화: `sin(α) ≈ α`, `cos(α) ≈ 1`, `d_r·κ_r ≪ 1` 가정 → LTV 시스템.

### 3.3 시스템 출력 (식 3, 4) — 충돌검사용 3원 근사

차체를 3개 원으로 근사 (후축, 휠베이스 중간, 전축):
```
d_i = d_r + l_i·sin(θ - θ_r) ≈ d_r + l_i·(θ - θ_r),  i = 1, 2, 3
```
출력에 곡률 κ도 포함 → 시스템 한계(Kamm circle) 제약 구성.

### 3.4 비용함수 (식 10) — 인간 운전 모사

```
l(x, u) = w_d·d_r² + w_θ·(θ - θ_r)² + w_κ·κ² + w_u·u²
```
- 가중치는 속도 의존: 고속에선 `w_κ` ↑, `w_d` ↓ (코너 cutting).
- 동적 장애물 회피용 soft 제약 + slack variables로 infeasibility 회피.

### 3.5 실험 (실차)

- BMW i3, BMW 5-series + 라이다.
- **dSpace Autobox DS1005 (PowerPC 750GX, 1 GHz)** — 1주기 turnaround **6 ms**.
- Prediction horizon: N=20, Δt=200 ms → 4 s.
- 시나리오 3종: 주차(1 m/s), 라운드어바웃(높은 횡g, 7 m/s²), 동적 장애물 회피(20 m/s).

### 3.6 morai-mpc 적용 시 의의

- LTV-MPC + 단일 QP라는 **계산 효율 패턴**이 후진 LTV-MPC 확장의 출발점.
- 단, 위 수식은 v(t) 부호 가정이 명시적이지 않음 → **후진 적용 시 부호와 d_r 정의를 재검토**해야 함 (논문의 가정 `θ - θ_r < 20°`는 후진 시 깨질 수 있음).

---

## 4. Wang 2019 — Improved MPC with Fuzzy Adaptive Weights (참고: 가중치 적응)

### 4.1 위치

- 전진 추종 전용. **후진 미언급**.
- 핵심 기여: **고정 가중치 LMPC의 ride comfort 문제를 fuzzy adaptive weight으로 해결**.

### 4.2 모델

- 자전거 동역학 모델 (Ackermann), Pacejka tire 선형화.
- 6-상태: `χ = [ẋ, ẏ, ϕ, ϕ̇, X, Y]^T`, 입력 `u = δ_f`.
- 운동학이 아니라 **동역학** 채택 — 횡방향 안정성 동시 추구.

### 4.3 LMPC

- 증분형 입력 `Δu`로 reformulation, 상태 augment `χ̃ = [χ; u_{k-1}]`.
- 비용함수:
  ```
  J = ‖Q(Ỹ_a - Ỹ_a,ref)‖² + ‖R·ΔU_a‖²
  ```
- QP로 풀이.

### 4.4 Fuzzy Adaptive Weight

입력: 횡오차 `e_Y`, 헤딩오차 `e_ϕ` (각 5단계: NB/NS/ZO/PS/PB)
출력: `r_Q_ϕ`, `r_Q_Y`, `r_R_Δδ` (4단계: ZO/PS/PM/PB)

규칙(요지): **목표경로에서 멀리 떨어졌을 때 `R_Δδ` 가중치를 키움** → 무리한 조향 방지, 부드럽게 접근.

### 4.5 결과 (CarSim co-simulation)

| 시나리오 | Pure Pursuit | Classical MPC | Improved MPC |
|---|---|---|---|
| 시나리오 1 (초기위치 일치) max 횡오차 | 0.211 m | — | 0.243 m (조향 부드러움 우위) |
| 시나리오 2 (초기 (-2, -4)에서 출발) max 횡오차 | — | 더 작음 | ~0.18 m 더 큼 (대신 부드러움) |

→ **정확도 vs 부드러움 trade-off를 동적으로 조절**하는 패턴.

### 4.6 morai-mpc 적용 시 의의

- 후진은 본질적으로 불안정 → **horizon 길이 + 가중치 동적 조정**이 효과적일 수 있음.
- 후진 추종 시 큰 경로 이탈 발생 시 `R_Δδ`(또는 `R_Δω_γ`) 가중을 일시 강화하는 fuzzy/스케줄링 기법은 직접 이식 가능.

---

## 5. 4편 종합 — morai-mpc 후진 구현 권고

### 5.1 어떤 패턴을 채택해야 하나

| 상황 | 권장 패턴 | 근거 |
|---|---|---|
| 일반 승용차 단방향 후진 주차 | Tashiro 2013 식 9 (x-y 기반 V₁) | 속도 변동·조향 시상수 오차에 강건, 단순 |
| 좁은 공간 + 충돌제약 | Gutjahr 2017 LTV-MPC + Tashiro의 후진용 부호 처리 | 단일 QP로 ms 단위 응답 |
| 동역학 효과 무시 못함 (저마찰) | Wang 2019 동역학 LMPC + fuzzy 가중 | 슬립 고려 |
| switch-back 포함 평행주차 | hybrid system 확장 (모든 논문이 명시적 미해결) | Tashiro도 명시: "future work" |
| 트레일러/관절차량 후진 | Liu·Bai 2025 NMPC (rear axle 기준 재정식화) | 후진 발산 문제 직격 |

### 5.2 일반 차량(비관절) 후진에 Liu·Bai의 통찰을 적용

Liu·Bai는 관절차량용이지만, **두 가지 통찰은 일반 차량에도 그대로 적용**:

1. **부호 처리 일관성 (식 1~4 패턴)**:
   ```
   v_ctrl = -v_sensor      (제어기 내부 v > 0 가정)
   θ_ctrl = θ_sensor를 reference 방향에 ±180° 정렬
   δ는 후진 시 효과 부호 반전 (운동학에서 자동)
   ```

2. **기준점 일관성**: 측위 데이터의 출처점(센서 장착 위치)과 운동학 모델의 기준점을 **반드시 일치**시킬 것. 후진 시 "전축에 측위, 운동학은 후축 기준" 같은 mismatch는 큰 오차의 직접 원인.

### 5.3 Tashiro의 V₁을 LTV-MPC에 결합한 후진 코스트 (제안)

Gutjahr 식 (10)에 Tashiro의 "방향 일치 시점 비교" 아이디어를 결합:

```
l_reverse(x, u) = w_d·d_r²
                 + w_θ·(θ - θ_r)²
                 + w_κ·κ²
                 + w_Δκ·(κ_k - κ_{k-1})²
                 + w_u·u²

with:
  v(t) < 0 가정 명시 (예측 모델 부호 보정)
  d_r 정의: 후축 → reference curve 법선거리 (전·후진 동일, 부호 보존)
  horizon은 전진보다 짧게 (후진 발산 방지)
```

### 5.4 검증 시 필수 체크리스트

| # | 체크 항목 | 기준 | 출처 근거 |
|---|---|---|---|
| 1 | 횡오차 1 m 초과 시 fail-safe | 정지 + 알람 | Liu·Bai 5장 정의 |
| 2 | 차속 변동 ±20% 추적 유지 | RMSE 변화 < 30% | Tashiro Fig. 8 |
| 3 | 조향 시상수 ±30% 오차 | 추적 성공 | Tashiro 4장 |
| 4 | 곡률 변화 시 솔버 시간 안정 | < 제어주기 70% | Liu·Bai Fig. 19 |
| 5 | 조향속도 한계 위반 횟수 | 0 | Liu·Bai vs Stanley 비교 |
| 6 | 평활도 (조향 ΔΔδ RMS) | classical MPC 대비 ↓ | Wang 2019 시나리오 2 |

---

## 6. 본 정리에서 검증되지 않은 사항 (정직 표시)

- 실차 적용 결과는 어느 논문도 후진에 대해 보고하지 않음 (Gutjahr만 전진 실차).
- Liu·Bai의 Gazebo 결과는 본문에 미수록(저자가 redundant라며 생략) → 인용 시 한계 인지 필요.
- 본 문서에 인용한 식·수치는 4편 PDF에서 **직접 발췌·검증**한 것임.

---

## 7. morai-mpc 저장소 다른 자료와의 관계

| 파일 | 상태 |
|---|---|
| `Linear_Time-Varying_MPC.pdf` | 위 §3 (Gutjahr 2017) — 본 문서에 전체 반영 |
| `Path_Tracking_Control_for_Autonomous_Vehicles_Based_on_an_Improved_MPC.pdf` | 위 §4 (Wang 2019) — 본 문서에 전체 반영 |
| `Vehicle_steering_control_with_MPC...reverse_parking.pdf` | 위 §1 (Tashiro 2013) — 본 문서에 전체 반영 |
| `wevj-16-00596.pdf` | 위 §2 (Liu·Bai 2025) — 본 문서에 전체 반영 |
| `docs/REVERSE_GEAR_IMPLEMENTATION.md` | 본 문서와 별개 — 코드 구현 진행 |
| `docs/REVERSE_TRACKING_PROGRESS.md` | 본 문서와 별개 — 진행 상황 추적 |
