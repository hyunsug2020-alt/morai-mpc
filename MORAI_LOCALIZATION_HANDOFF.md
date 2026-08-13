# MORAI 위치추정 작업 인수인계

## 현재까지 완료

- UDP 포트 고정: LiDAR `2368`, IMU `9091`, GPS `9090`
- 115-byte IMU 패킷의 timestamp/데이터 offset 수정 및 중복 timestamp 제거
- MORAI VLP-16을 공식 Velodyne 드라이버로 처리해 실제 `ring`, point `time` 사용
- 평지 퇴화 억제 후 정지 20초 LIO 드리프트 약 `409 m` → `1.77 m`
- `/gps`를 `/lio_sam/gps/odometry`로 변환하는 노드 추가
- 첫 키프레임과 `(0, 0)` 원점부터 GPS factor 허용, 1 m마다 GPS 보정
- 연결 런치와 LIO 재시작 런치 분리
- `/Ego_topic` 연결 감시 추가
- LIO-SAM `mapOptimization` Release 빌드 성공

## 확인된 원인

- 기존 IMU 파서는 MORAI timestamp 8 byte를 quaternion으로 읽어 가속도와
  각속도를 한 필드씩 잘못 전달했음.
- 평지에서는 LiDAR 기하구조만으로 자세와 평면 이동 일부를 구속하지 못해
  `LIO_DEGENERATE`가 지속됐음.
- `/Ego_topic`은 rosbridge 재시작 후 MORAI WebSocket 클라이언트가 자동
  재접속하지 않아 publisher가 0개가 됐음.

## 다음 실행 순서

터미널 1 — 연결은 계속 유지:

```bash
cd /home/coss/catkin_ws
source devel/setup.bash
roslaunch morai_launch morai.launch start_lio_sam:=false start_localization:=false
```

MORAI Network에서 ROS Bridge를 한 번 Connect한 뒤 확인:

```bash
rostopic info /Ego_topic
rostopic hz /Ego_topic
```

터미널 2 — 자유롭게 재시작 가능한 LIO:

```bash
cd /home/coss/catkin_ws
source devel/setup.bash
roslaunch morai_launch lio_restart.launch
```

터미널 3 — 비교 측정:

```bash
cd /home/coss/catkin_ws
source devel/setup.bash
roslaunch morai_control lio_odometry_experiment.launch
```

## 다음 작업

1. `/Ego_topic`, `/imu/data`, `/gps`, `/velodyne_points` rate 확인
2. 정지 30초 드리프트 재측정
3. 직선·회전 주행 후 Ego 대비 LIO 위치/yaw RMSE를 CSV로 기록
4. `Large velocity, reset IMU-preintegration!` 재발 여부 확인
5. 정상 GPS 구간과 GPS 음영 구간 성능 비교
6. 최종 항법 출력은 GPS/IMU ESKF에 LIO를 조건부 보정값으로 결합 검토

## Git 상태

- `/home/coss/catkin_ws` 자체는 Git 저장소가 아님
- `src/LIO-SAM`만 공식 LIO-SAM 원격 저장소에 연결돼 있음
- `morai_launch`와 `morai_control`까지 보관하려면 전체 workspace용 사용자
  GitHub 저장소와 원격 주소가 필요함
