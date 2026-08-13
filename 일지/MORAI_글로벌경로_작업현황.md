# MORAI 글로벌 경로 작업 현황

작성일: 2026-08-03

## 1. 작업 목적

사용자가 지정한 글로벌 노드 순서를 MORAI Simulator에서 Ego 차량이 추종할 수 있는 waypoint 경로로 변환하는 작업임.

지정된 글로벌 노드는 다음과 같음.

```text
218, 422, 395, 447, 453, 402, 352, 25, 328, 35, 339, 158, 133,
333, 384, 439, 354, 458, 446, 359, 426, 423, 361, 113, 60, 247,
257, 466, 271, 362, 371, 253, 266, 5, 54, 58, 367, 358, 134
```

이 경로 파일은 MORAI 화면에 직접 불러오는 형식이 아님. ROS 경로 추종 노드가 파일을 읽고 `/ctrl_cmd`를 발행하여 MORAI Ego 차량을 움직이는 방식임.

## 2. 완료된 작업

### 글로벌 노드 및 링크 검증

- 지정된 글로벌 노드 39개가 HD Map에 모두 존재함을 확인했음.
- 연속된 노드 사이의 38개 구간이 모두 실제 방향성 링크로 연결되어 있음을 확인했음.
- 시작 노드 `#218`은 실제 MGeo 노드 `A1256W000332`에 해당함.
- 종료 노드 `#134`는 실제 MGeo 노드 `A1256W000144`에 해당함.

### 경로 시각화

다음 HTML 파일에 선택 노드와 연결 경로가 표시되도록 반영했음.

```text
/home/bisa/Downloads/hdmap_viz.html
```

### MORAI 추종용 waypoint 생성

글로벌 노드 사이의 실제 HD Map 링크 형상을 따라 경로를 생성했음.

- waypoint 개수: 4,354개
- 전체 경로 길이: 약 2,165.8m
- waypoint 최대 간격: 0.5m
- 최대 국부 회전각: 약 11.17도
- waypoint 필드: `x`, `y`, `heading`, `gear`
- 모든 waypoint의 기어: `D`

원본 생성 결과는 다음 위치에 있음.

```text
/home/bisa/Downloads/morai_global_path.json
```

경로 재생성 스크립트는 다음 위치에 있음.

```text
/home/bisa/Downloads/build_morai_global_path.py
```

### ROS 프로젝트 적용

생성한 경로를 ROS 추종기의 기본 HD Map 경로 위치에 적용했음.

```text
/home/bisa/morai-mpc-agent-morai-lio-gps-integration/src/moraimpc/data/hdmap_path.json
```

기존 경로는 다음 파일로 백업했음.

```text
/home/bisa/morai-mpc-agent-morai-lio-gps-integration/src/moraimpc/data/hdmap_path.before_global_218_134.json
```

## 3. 경로 시작 정보

- 시작 노드: 글로벌 노드 `#218`
- 실제 MGeo ID: `A1256W000332`
- 시작 위치: `X=-131.6898`, `Y=-428.3310`
- 시작 방향: 약 `61.3도`
- 시작 진행 방향: `#218 → #422`
- 종료 위치: `X=-140.8606`, `Y=-444.8091`

시작점과 종료점 사이의 직선거리가 약 18m로 가까움. 차량을 임의 위치에서 시작하면 추종기가 종료 지점 부근을 현재 위치로 잘못 선택할 가능성이 있음. 따라서 Ego 차량을 반드시 시작 노드 `#218`에 배치하고 실행해야 함.

## 4. 실제 시뮬레이터 테스트 결과

실행 중인 MORAI Simulator와 ROS Bridge를 이용해 실제 추종 테스트를 수행했음.

확인된 사항은 다음과 같음.

- 경로 파일을 정상적으로 읽었음.
- 총 4,354개 waypoint와 약 2,165.8m 경로로 인식했음.
- `/ctrl_cmd`가 약 20Hz로 발행됐음.
- MORAI의 rosbridge가 `/ctrl_cmd`를 구독하는 것을 확인했음.
- 차량이 실제 제어 명령에 반응하여 움직이는 것을 확인했음.

다만 전체 경로 완주 테스트는 성공하지 못했음.

실패 원인은 다음과 같음.

1. 테스트 시작 시 Ego 차량이 정확한 시작 노드에 배치되지 않았음.
2. 차량 상태가 `gear=0`, `ctrl_mode=0`으로 외부 자율주행 모드와 D단이 아니었음.
3. 시작점과 종료점이 가까워 추종기가 종료 waypoint를 현재 위치로 잘못 선택했음.
4. 결과적으로 추종기가 정상 주행 모드가 아닌 `RECOV` 상태로만 동작했음.

테스트 중 차량은 제동 명령으로 정지시켰고 추종 노드도 종료했음.

테스트 기록은 다음 위치에 있음.

```text
/home/bisa/Downloads/global_route_test_log.json
```

## 5. 현재 상태

| 항목 | 상태 |
|---|---|
| 글로벌 노드 존재 확인 | 완료 |
| 노드 간 링크 연결 확인 | 완료 |
| HTML 경로 시각화 | 완료 |
| MORAI 추종용 JSON 생성 | 완료 |
| ROS 프로젝트 기본 경로로 적용 | 완료 |
| ROS 경로 로딩 확인 | 완료 |
| `/ctrl_cmd` 통신 확인 | 완료 |
| 실제 차량 반응 확인 | 완료 |
| 전체 경로 정상 완주 | 미완료 |
| 시작점 오매칭 방지 개선 | 미완료 |

## 6. 현재 경로 실행 방법

### MORAI 설정

1. MORAI에서 해당 HD Map과 시나리오를 실행함.
2. Ego 차량을 시작 노드 `#218` 부근에 배치함.
3. 차량 방향을 `#422` 방향인 약 `61.3도`로 설정함.
4. `Edit → Network Settings → Ego Network → Cmd Control`에서 ROS와 `/ctrl_cmd`를 사용하도록 설정함.
5. Manual-keyboard 상태에서 숫자 `1`을 눌러 D단으로 변경함.

### ROS 실행

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source devel/setup.bash

roslaunch moraimpc hdmap_path_follower.launch \
  path_file:=/home/bisa/morai-mpc-agent-morai-lio-gps-integration/src/moraimpc/data/hdmap_path.json \
  cruise_kmh:=10 \
  use_avoid_path:=false \
  stop_at_path_end:=true
```

추종기 로그에서 경로 인덱스가 `idx=0` 부근으로 잡혔는지 확인한 뒤 MORAI의 차량 제어 모드를 `AV-ExternalCtrl`로 전환해야 함.

현재 프로젝트의 `mpc_tracking.launch`는 사용하지 않는 것이 안전함. 이 launch에 포함된 smoothing 스크립트가 기존 `waypoints_recorded.json`을 이용해 현재 `hdmap_path.json`을 덮어쓸 수 있음.

## 7. 남은 작업

전체 경로를 안정적으로 완주하려면 다음 작업이 필요함.

1. MORAI Ego 차량을 시작 노드 `#218`에 정확하게 배치함.
2. D단과 `AV-ExternalCtrl` 상태를 MORAI UI에서 확인함.
3. 초기 경로 탐색 범위를 waypoint 0 부근으로 제한하여 종점 오매칭을 방지함.
4. 우선 10km/h로 전체 경로 완주 테스트를 수행함.
5. 완주 후 횡방향 오차, 헤딩 오차, 이탈 구간을 확인하고 제어 파라미터를 조정함.

## 8. 결론

글로벌 노드 목록을 실제 HD Map 링크에 연결하고 MORAI ROS 추종기가 읽을 수 있는 경로 JSON으로 만드는 작업까지 완료했음. ROS와 MORAI 사이의 제어 명령 전달 및 차량 반응도 확인했음.

현재 남은 핵심 작업은 차량 시작 위치와 외부 제어 모드를 정확히 맞춘 상태에서 전체 경로를 다시 주행하고, 시작점과 종료점이 가까워 발생하는 초기 waypoint 오매칭을 방지하는 것임.
