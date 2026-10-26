# Claude → Codex 인계 (2026-10-10 15:50 KST)

## 작업 범위 / 규칙
- 작업 폴더: `autodrive/student_v3_claude` (웹 "E2E 모드", 런치 `autodrive/student_v3_follow_claude.sh`). 라이브 `student_v3`(룰베이스)는 이번 작업에서 안 건드림. 원본 보관본 `autodrive/student_v3_claude_base`(10-10 03:08 라이브 스냅샷).
- 검증은 MORAI 실주행으로(가상 시나리오/평가 스크립트 X). 테스트: `cd autodrive/student_v3_claude && env -u STUDENT_PLANNER -u STUDENT_TUNNEL_POLICY STUDENT_V3_DIR=$PWD OMP_NUM_THREADS=2 nice -n 19 /home/david/miniconda3/envs/lane_tool/bin/python -m unittest discover -s . -p 'test_*.py' -q` (주행 중엔 금지, 한 프로세스). `test_v6_shadow ... mask_buffer` 는 부하 시 타이밍 플레이크(단독 실행 시 통과).
- 주행 시작(사용자가 원할 때만, drive.lock 확인): 스크래치 스크립트와 같은 절차 = `auto_collect.load_scenario("404_contest.json")` 후 `POST /api/start {"action":"autodrive_follow","options":{"minutes":10,"network_profile":"single_pc","drive_mode":"e2e"}}` (웹 8767).

## 오늘 완료·실주행 확인된 수정 (마지막 검증 주행 follow-20261010T062150Z: 2바퀴 274/247 s, 충돌 0, 고속도로 무정지 56/47 s 최고 113/118 km/h, 터널 19/17 s)
1. 터널 벽 지도 재정렬: `tunnel_wall_map.json` 이 0.544°/0.177 m 틀어져 있었음(10-02 제작 당시 ESKF 방향 고정 전 데이터). ESKF는 정상(블랙아웃 게이트가 막은 GPS 원신호와 터널 내 ~0.1 m 일치) — 공용 ESKF 설정은 바꾸지 말 것. `realigned_from` 기록 + `tunnel_map.py` 배경 서베이 허용 + `TunnelMap.load(..., survey_frame=True)`(10-02/03 캡처 재생용). 실주행 4회 통과 모두 벽 매칭 100%, 방향 보정 ~0°. ※ 라이브 student_v3 는 아직 옛 지도(같은 잠재 버그).
2. 장애물 구간 상자 급제동: `path_safety.py` FUSED_STATIC_MPS=1.0(플래너와 공유), 정지→출발 1회 보고는 재확인 필요(`fused_motion`), 정차 npc 헤딩 유지.
3. 고속도로: 차로변경 시작/취소 기준 일치(admission, `planning/highway/policy.py`), 카메라 '주차차량' 중복상자 소유(`object_policy.py`), 긴 차량 상자 방향 차로 정렬(`planning/highway/motion_evidence.py`, `highway_speed_plan.stable_yaws`), 차로 변경 직후 차선 경계 비상정지→짧은 헤딩 경로(`keep_path`), 정지 후 복귀 경로(`inward_recovery_clearance`, `corridor_overrun`, 4 km/h, 런타임 최종 지도검사도 같은 함수).
4. 터널 플래너 오인 움직임: `tunnel_tracks.py` 진행축 양끝 동시이동 게이트, 20 m 밖 시선방향 2 m, 보행 0.8 m/s, `measured_vehicle`(감지기 5 m/s 미만은 측정 무버 필요). LivePathSafety 터널도 같은 규칙(`unknown_static=True`).
5. GUI 크래시: `highway_planner.py` recovery_brake_retained numpy bool → `bool()`; 상태 토픽/판단로그 json 에 `json_safe.json_default`. ※ 라이브 student_v3 `highway_planner.py:555` 에 같은 잠재 크래시 있음.

## 미완료 (이어서 할 것)
A. **진행 중이던 터널 수정(미완성, 테스트 2개 실패 상태)**: 15:44~15:49 편집된 `tunnel_tracks.py, path_safety.py, live_path_safety.py, test_live_path_safety.py, planning/tunnel/runtime.py`. 실패: `test_tunnel_fused_car_motion_does_not_slow_the_crawl`, `test_tunnel_standing_box_skewed_over_the_margin_does_not_stop_the_pass`. 목표(run 062150Z 터널 1바퀴 279.5–286 s, 24→4 km/h):
   - LivePathSafety 자체 TunnelTracks 군집 `tunnel_lidar_dynamic_800051` 이 주차차 옆 통과 시 옆방향 중심 흔들림으로 '움직임' → 가까운 군집의 횡방향 이동은 관측자 쪽 '가까운 모서리'도 같이 움직일 때만 인정(차 뒤에서 나오는 사람은 그대로 감지).
   - 그 가짜 무버가 감지기 상자 500030 안에 있어 차가 다시 '움직임'(281.49 s, gap 0).
   - 터널에서 카메라 fusion_npc(uid 129) 움직임 사용 → 터널 플래너처럼 fusion 움직임 미사용.
   - 정지 감지기 상자가 경로로 밀려 gap 0 'collision: lidar_car'(285.6 s, raw_clear 53 m) → 터널에서 정지 차량은 실측 점 기준.
B. **장애물 구간 상자(2바퀴 371.9 s)**: fusion_obstacle uid 155 가 처음부터 '움직임'으로 1회 보고 후 사라짐 → MotionMemory 가 0.56 s 예측(raw_clear 34–58 m). '휴지 이력 없이 처음 본 움직임'도 사라지면 외삽하지 않게(path_safety.py `fused_motion`/MotionMemory). A 와 같은 파일이라 A 후에.
C. **고속도로(분석만, 수정 전)**: run 062150Z 488.1 s 'unsafe change' 취소(116 km/h, 차로 420) — 시작 시 알 수 있었는지; 220–228 s 차로변경 중 차로표시 409/411 깜빡임 + 112 km/h 비상('braking trajectory conflict') 원인; 같은 런의 비상 프레임 29개 분류.
D. 참고(수정 범위 밖으로 판단): 고속도로에서 50 m 앞 느린 차가 처음엔 속도 0/카메라 '주차차량'으로 잡혀 강한 감속(필요한 감속이었음, 감지거리 한계).

## 분석 도구 (스크래치, 읽기 전용 참고)
`/tmp/claude-1000/-home-david/7629a205-6f53-4a28-b329-1b6c6895e27a/scratchpad/` : `watch_run2.py`(실주행 이벤트 감시), `collision_watch.py`(/CollisionData), `r5/summary.py`, `r5/section_speed.py`, `r5/stall_rebuild.py`(고속도로 지도·경로 재구성), `r5/wallfit.py`·`r5/tunnel_gps.py`(터널 지도 vs GPS 원신호), `r5/start_e2e.sh`(주행 시작).
