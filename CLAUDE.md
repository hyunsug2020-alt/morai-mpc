# moraimpc_ros1 프로젝트

## 목표
MORAI 시뮬레이터 환경에서 MPC(Model Predictive Control)를 이용한 자율주행 구현

## 분석 방법
- 주행 시마다 MPC 결과값을 JSON 파일로 저장 중
- 로그 파일 위치: `src/moraimpc/logs/mpc_log.json`
- 해당 JSON 데이터를 분석하여 MPC의 문제점을 파악하고 순차적으로 개선 중

## 작업 방식
- 주행 결과 JSON 분석 → 문제점 식별 → 코드 수정 → 재주행 검증 반복
