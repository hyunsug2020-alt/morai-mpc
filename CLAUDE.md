# Claude 행동 규칙
1. 절대로 임의판단하고 임의행동 하지 않기. 반드시 사용자에게 확인 후 진행할 것.
2. 간결하게 ~음 형태로 말하기. 불필요한 설명 생략.
3. 모든 명령어에 -y 플래그 등을 사용하여 확인 프롬프트 없이 자동 진행되게 하기.

---

# moraimpc_ros1 프로젝트

## 목표
MORAI 시뮬레이터 환경에서 MPC(Model Predictive Control)를 이용한 자율주행 구현

## 분석 방법
- 주행 시마다 MPC 결과값을 JSON 파일로 저장 중
- 로그 파일 위치: `src/moraimpc/logs/mpc_log.json`
- 해당 JSON 데이터를 분석하여 MPC의 문제점을 파악하고 순차적으로 개선 중

## 작업 방식
- 주행 결과 JSON 분석 → 문제점 식별 → 코드 수정 → 재주행 검증 반복

## 실행 명령어

### 1. 의존성 빌드 (catkin_ws에서)
```bash
cd /home/coss/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 2. 실행 (각각 별도 터미널)
```bash
# 터미널1: roscore
roscore

# 터미널2: MORAI rosbridge 연결
source ~/catkin_ws/devel/setup.bash
roslaunch morai_launch morai.launch

# 터미널3: MPC 경로 추종
source ~/catkin_ws/devel/setup.bash
roslaunch moraimpc mpc_tracking.launch
```

### 3. 디버깅
```bash
source ~/catkin_ws/devel/setup.bash
rostopic list                        # 토픽 목록
rostopic hz /Ego_topic               # MORAI 데이터 수신 확인
rostopic echo /Ego_topic --noarr -n1 # 데이터 내용 확인
rostopic echo /ctrl_cmd_0 -n1        # MPC 제어 명령 확인
```

## 환경 정보
- Ubuntu IP: `172.30.1.22`
- MORAI(Windows) → rosbridge websocket: `ws://172.30.1.22:9090`
- ROS Noetic / catkin workspace: `/home/coss/catkin_ws`
- morai_msgs: `MORAI-ROS_morai_msgs` 확장 버전 사용 (타이어/슬립 필드 포함)

## 워크스페이스 구조
- `/home/coss/catkin_ws/src/morai_msgs` → symlink → `/home/coss/morai-mpc-master/src/morai_msgs`
- `/home/coss/catkin_ws/src/moraimpc` → symlink → `/home/coss/morai-mpc-master/src/moraimpc`
- `/home/coss/catkin_ws/src/morai_launch` → symlink → `/home/coss/morai_launch`
