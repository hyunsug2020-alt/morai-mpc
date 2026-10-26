# autodrive 폴더 안내 (E2E 전용, 2026-10-11 정리)

주행 방식은 **E2E 하나**만 남았습니다 (v6.3 판단 모델 + 이식 정적/터널 회피).
룰베이스 `student_v3/`, 스냅샷 폴더, 옛 런치·백업·판정 스크립트는 2026-10-11에
`~/morai_trash_20261011/autodrive/` 로 옮겼습니다 (같은 상대 경로 유지).

## 1. 주행 (런치 1개)

| 경로 | 역할 |
|---|---|
| `student_v3_follow_claude.sh` | **유일한 대회 런치**. `<초(무시)> 90.0 localization <single_pc\|two_pc> [세션번호]` — 시작 지점에서 한 바퀴 돌고 그 자리에 정지 (시간 제한 없음). 5번째 인자가 있으면 센서 수집 |
| `student_v3_claude/` | 주행 코드 + 데이터 전부 (`gui.py`, `model_avoidance.py`, `v6/`, `tunnel_static_portable/`, `best_v5.pt`, `v6_model.pt`, `global_path.json`, `link_set.json`, `tunnel_wall_map.json`, `web_speed_zones.json`, 테스트) |
| `semantic_fusion.py`, `semantic_sidecar.py`, `semantic_gpu_masks.py` | 카메라 시맨틱 + LiDAR 물체 인식 |
| `lidar_vehicle_detection.py`, `pointpillar_runtime.py` | LiDAR 차량 인식 (PointPillars) |
| `localization_monitor.py`, `gps_blackout_gate.py`, `sensor_health.py` | 위치 상태 기록, 터널 GPS 차단, 센서 상태 |
| `competition_profiles.py`, `competition_network.py` | 1PC/2PC 센서·네트워크 설정 |
| `competition_speed.py`, `speed_guard.py` | 속도 단계 (설정 `data/autodrive/competition_speed.json`) |
| `prepare_cpu.py`, `cpu_placement.py`, `performance_cpu.json` | CPU 배치 |
| `ros/`, `ws/`, `rviz/`, `compose.yaml`, `Dockerfile*`, `start.sh`, `stop.sh` | ROS 컨테이너·MPC 제어기(C++) |
| `competition_viz.py` | RViz 표시 |

공유 모듈(`semantic_fusion.py`, `lidar_vehicle_detection.py`)은 런치가 내보내는
`STUDENT_V3_DIR`(= `student_v3_claude`)에서 플래너 모듈을 읽습니다.

## 2. 웹·자동 반복 도구

| 경로 | 역할 |
|---|---|
| `web_app.py`, `web/` | 대시보드 http://127.0.0.1:8767 (`morai-collector-dashboard.service`). "대회 런치" = E2E 주행, "주행 + 센서 수집" = 같은 런치 + 세션 기록 |
| `auto_collect.py` | 자동 수집 루프 (시나리오 생성 → MORAI 로드 → 한 바퀴 → 다음). 다른 도구가 함수만 가져다 씀 |
| `tools/morai_test/competition_*.py` | 시나리오 반복 주행·NPC 활성화·충돌 증거 서비스 (`morai-competition-*.service`) |
| `video_every10.sh`, `make_run_video.sh` | 회차 영상 |
| `portable/` | 다른 PC용 1PC 번들 (`launch.sh` → E2E 런치) |

## 3. 테스트

```
cd autodrive/student_v3_claude && STUDENT_V3_DIR=$PWD nice -n 19 \
  /home/david/miniconda3/envs/lane_tool/bin/python -m unittest discover -s . -p 'test_*.py'
```
`autodrive/test_*.py`, `autodrive/tests/`, `tools/morai_test/test_competition_*.py` 는 공유 모듈·웹 검사.
(`test_competition_profiles.py` 의 CaptureTests 는 `data/autodrive/drive.lock` 을 잡으므로 주행 중에는 실행 금지.)

## 4. 데이터 위치

| 경로 | 내용 |
|---|---|
| `../data/autodrive/follow_runs/<run>/` | 회차별 디버그 기록 |
| `../data/autodrive/mpc_runs/<번호>/` | 수집 회차 주행 기록 |
| `../data/raw/<번호>/` | 수집 세션 |
