# lidar_detection

MORAI `/Object_topic`의 객체 크기와 pose를 `/Ego_topic` 및 LiDAR 외부
파라미터로 `/velodyne_points` 좌표계에 투영함.

각 LiDAR 프레임을 잠시 버퍼링하고, LiDAR 타임스탬프 앞뒤의
`/Ego_topic`과 `/Object_topic` 샘플을 찾음. 위치는 선형 보간하고
heading은 ±180도 경계를 고려한 최단 각도로 보간하여 LiDAR 시각에 맞춤.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lidar_detection ground_truth_boxes.launch
```

출력:

- `/lidar_detection/ground_truth_markers`: RViz 3D 박스와 라벨
- `/lidar_detection/ground_truth_diagnostics`: 동기화 방식과 앞뒤 샘플
  간격, 박스 중심·크기·yaw 및 내부 점 수

학습 데이터 생성 전, 각 객체의 `points_in_box`와 RViz 정렬 상태를 먼저
확인해야 함.

## 데이터셋 저장

보간 시각화 노드가 실행 중일 때 별도 터미널에서 실행함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lidar_detection record_dataset.launch
```

기본 출력은 저장소의 `datasets/morai_lidar`임. 차량 박스가 있는 프레임은
모두 저장하고, 차량이 없는 프레임은 오검출 방지용으로 10개 중 1개만
저장함.

출력:

- `velodyne/*.bin`: `float32` x, y, z, intensity
- `labels/*.json`: 정밀 보간 박스와 동기화 진단
- `label_lidar/*.txt`: LiDAR 좌표계 PointPillars 변환용 박스
- `timestamps/*.txt`: 원본 ROS 타임스탬프
- `ImageSets/all.txt`, `manifest.jsonl`: 프레임 인덱스

실행 중 일시 정지 및 재개:

```bash
rosservice call /lidar_dataset_recorder/set_enabled "data: false"
rosservice call /lidar_dataset_recorder/set_enabled "data: true"
```

30,000프레임 또는 2시간에서 자동 정지:

```bash
roslaunch lidar_detection guard_collection.launch \
  target_total_frames:=30000 max_duration_seconds:=7200
```

완료 시 `collection_report.json`을 데이터셋 루트에 저장함.

녹화 완료 후 전체 데이터 무결성과 라벨 통계를 검사함.

```bash
rosrun lidar_detection validate_lidar_dataset.py \
  /home/bisa/morai-mpc-agent-morai-lio-gps-integration/datasets/morai_lidar
```

검증 결과는 데이터셋 루트의 `validation_report.json`에 저장함.

## epoch 39 모델 실시간 차량 검출

학습 완료된 `trained_models/morai_pointpillar_2026-07-31/best_model.pth`를
사용하여 `/velodyne_points`의 차량을 실시간으로 검출함. 학습 데이터와 동일하게
LiDAR 좌표는 x=전방, y=좌측, z=위쪽이어야 하며 intensity가 없는 입력은 0으로
채움.

먼저 MORAI LiDAR가 `/velodyne_points`로 발행되는지 확인한 뒤 실행함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lidar_detection pointpillar_detection.launch
```

출력:

- `/lidar_detection/detection_markers`: RViz 차량 3D 박스와 신뢰도 라벨
- `/lidar_detection/detections`: 중심, 크기, yaw, 점수, 추론 지연이 담긴 JSON

기본 검출 점수 기준은 실제 차량과 지면 오검출을 분리하도록 0.445로 설정했으며
실행할 때 조절할 수 있음.

```bash
roslaunch lidar_detection pointpillar_detection.launch \
  score_threshold:=0.25 start_rviz:=true
```

차량 검출 launch는 기본적으로 수평도·센서 높이 제한을 둔 RANSAC으로 도로
평면을 제거한 뒤 PointPillars에 전달함. 필터 결과는
`/lidar_detection/points_no_ground`에 게시되며 RViz도 이 토픽을 표시함.
필요하면 `remove_ground:=false`로 원본 포인트를 그대로 사용할 수 있고,
기본 평면 거리 임계값은 `ground_distance_threshold:=0.15`m임.

LiDAR를 별도로 실행해야 한다면 먼저 다음 launch를 실행함.

```bash
roslaunch lio_sam morai_udp.launch start_lio_sam:=false start_rviz:=false
```

정상 동작 확인:

```bash
rostopic hz /velodyne_points
rostopic echo -n 1 /lidar_detection/detections
```

## epoch 40 사람 위치 검출 및 RViz 박스

센서 launch는 포함하지 않으며, 이미 연결된 `/velodyne_points`를 입력으로
사용함. 사람·장애물 모델에서 `Pedestrian` 클래스만 통과시켜 RViz에 사람
3D 박스와 신뢰도를 표시함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lidar_detection pedestrian_detection.launch
```

출력:

- `/lidar_detection/pedestrian_markers`: RViz 사람 3D 박스와 점수
- `/lidar_detection/pedestrian_detections`: 사람 중심 좌표·크기·거리 JSON

RViz 없이 검출 노드만 실행하거나 입력 토픽을 변경할 수도 있음.

```bash
roslaunch lidar_detection pedestrian_detection.launch \
  start_rviz:=false point_cloud_topic:=/velodyne_points
```

처리가 입력 속도보다 느릴 때는 오래된 프레임을 큐에 쌓지 않고 최신 프레임만
남겨 실시간성을 유지함. `dropped_frames`는 이 동작으로 건너뛴 프레임 수임.

## 여러 장소·0~45m 사람 v2 30k 모델

30에포크 학습과 마지막 5에포크 평가가 끝난 뒤 선택된 v2 모델만 별도로
실행함. 센서 launch는 포함하지 않으며 기존 차량·사람 v1 모델은 변경하지 않음.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lidar_detection pedestrian_far_v2_detection.launch
```

RViz 없이 실행하려면 `start_rviz:=false`를 추가함. v2 출력 토픽은
`/lidar_detection/pedestrian_far_v2_markers`와
`/lidar_detection/pedestrian_far_v2_detections`임.

## 차량 + 보행자 통합 검출

차량 epoch 39 모델과 보행자 far-v2 epoch 27 모델을 함께 실행하고, 같은
LiDAR 프레임의 결과를 합쳐 하나의 RViz에 표시함. 차량 박스 내부에 생성된
보행자 후보는 단일 클래스 모델 간 오검출로 보고 통합 단계에서 제거함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lidar_detection car_pedestrian_detection.launch
```

출력:

- `/lidar_detection/combined_markers`: 차량(파란색)과 보행자(초록색) 박스
- `/lidar_detection/combined_detections`: 통합 결과와 제거된 중복 수 JSON

센서 launch는 포함하지 않으므로 `/velodyne_points`가 먼저 발행되어야 함.
