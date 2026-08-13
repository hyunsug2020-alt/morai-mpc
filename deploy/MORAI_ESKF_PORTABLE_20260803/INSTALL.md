# MORAI ESKF Portable Bundle

이 압축파일은 다른 Ubuntu 20.04 + ROS Noetic 컴퓨터에 현재 검증된
MORAI 순수 오도메트리 v5와 robust planar ESKF를 옮기기 위한 최소 배포본임.

## 포함 패키지

- `src/eskf`: 8-state planar ESKF, GPS 상태기계, fixed-lag replay,
  SE(2) 정렬, GUI, 설정 및 단위시험임.
- `src/morai_msgs`: `GPSMessage`, `EgoVehicleStatus` 등 MORAI ROS1 메시지임.
- `src/morai_eskf_runtime`: 순수 오도메트리 v5, 통합 launch,
  기존 `/localization/ego_status` 호환 출력 및 sensor TF 노드임.

## 지원 환경

- Ubuntu 20.04 amd64임.
- ROS Noetic임.
- Python 3임.
- MORAI에서 `/gps`, `/imu/data`, `/Ego_topic`을 발행하는 구성이 필요함.
- LIO-SAM 토픽은 선택 사항임. 발행자가 없더라도 GPS+IMU+순수 오도메트리로
  실행됨.

## 1. 필요한 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  python3-numpy \
  python3-matplotlib \
  python3-pyqt5 \
  python3-nose
```

ROS Noetic 설치가 이미 끝났다면 누락된 Python 패키지만 설치해도 됨.

## 2. 자동 설치

압축을 푼 디렉터리에서 실행함.

```bash
./install_into_catkin.sh ~/catkin_ws
```

스크립트는 다음 동작을 수행함.

1. 대상 catkin workspace의 `src`를 확인함.
2. 기존 `eskf` 또는 `morai_eskf_runtime` 패키지가 있으면 삭제하지 않고
   workspace 루트의 `portable_backup_YYYYMMDD_HHMMSS` 폴더로 이동함.
3. 기존 `morai_msgs`가 필요한 두 메시지를 이미 제공하면 그대로 유지함.
4. 세 패키지를 복사하고 `catkin_make`를 실행함.

## 3. 수동 설치

```bash
mkdir -p ~/catkin_ws/src
cp -a src/eskf ~/catkin_ws/src/
cp -a src/morai_msgs ~/catkin_ws/src/
cp -a src/morai_eskf_runtime ~/catkin_ws/src/
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 4. 실행

MORAI/rosbridge/UDP 센서 수신기가 이미 실행 중인 ROS master에서 실행함.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch morai_eskf_runtime morai_eskf.launch
```

기본 입력과 출력은 다음과 같음.

| 구분 | 토픽 |
|---|---|
| GPS 입력 | `/gps` |
| IMU 입력 | `/imu/data` |
| 차량 속도·조향 입력 | `/Ego_topic` |
| 선택 LIO 입력 | `/lio_sam/mapping/odometry` |
| 순수 오도메트리 | `/odometry/pure` |
| ESKF 위치 | `/eskf/odom` |
| ESKF 진단 | `/eskf/diagnostics` |
| 기존 형식 호환 출력 | `/localization/ego_status` |

토픽 이름이 다른 컴퓨터에서는 launch 인자로만 바꾸면 됨.

```bash
roslaunch morai_eskf_runtime morai_eskf.launch \
  imu_topic:=/imu \
  gps_topic:=/gps \
  vehicle_state_topic:=/Ego_topic
```

## 5. GUI 실행

```bash
roslaunch eskf monitor_gui.launch
```

GUI는 ESKF 추정값과 `/Ego_topic`을 비교하는 검증용임. `/Ego_topic`의 position과
heading은 GUI 평가에만 사용되며 ESKF 또는 순수 오도메트리 추정 입력으로 사용되지
않음.

## 6. 확인

```bash
rostopic hz /odometry/pure
rostopic hz /eskf/odom
rostopic echo -n1 /eskf/diagnostics
```

단위시험은 다음과 같이 실행함.

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest discover -s src/eskf/test -p 'test_*.py'
```

## 7. 중요한 주의사항

- 이 배포본의 production ESKF는 예전 Notion 문서의 15-state 6-DOF 버전이 아니라
  현재 차량 평면 운동용 **8-state planar ESKF**임.
- 상태는 `[px, py, vx, vy, yaw, bax, bay, bgz]`임.
- `config/eskf_robust.yaml`, `config/ioniq5_vehicle.yaml`,
  `config/morai_sensor.json`은 검증된 현재 값임. 다른 차량이나 센서 배치가 아니라면
  임의 변경하지 않는 것이 안전함.
- `/use_sim_time=false`가 기본이며 MORAI Real Time Mode의 시간비율은 순수
  오도메트리가 IMU quaternion과 gyro 적분의 비로 적응 추정함.
- 기존 메인 launch와 함께 실행할 때 `/eskf_node` 또는 `/pure_odometry`가 이미
  있으면 중복 실행하지 않아야 함.

## 8. 검증된 결과 요약

- 577.218초·2.445km 순수 오도메트리: 위치 RMSE 8.87m,
  yaw RMSE 0.261도, 종점 drift 0.354%였음.
- ESKF 강제 결함 4개 seed: 전체 위치 RMSE 0.902~1.174m였음.
- 8~30초 무작위 GPS 음영 6회/seed: 음영 p95 1.794~3.496m,
  최대 2.078~4.156m였음.
- ESKF 단위시험 10/10과 catkin 빌드를 통과했음.
