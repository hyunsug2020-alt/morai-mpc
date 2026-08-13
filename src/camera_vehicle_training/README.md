# camera_vehicle_training

MORAI 전방 카메라 영상과 `/Ego_topic`, `/Object_topic`을 영상 timestamp에
맞춰 보간하고 NPC 차량의 3D Ground Truth 박스를 영상에 투영하여 YOLO 차량
학습 데이터로 저장하는 독립 ROS 패키지임. 기존 LiDAR 학습 모델이나 카메라
추론 모델은 데이터 수집 중 실행하지 않음.

## 출력 구조

기본 출력은 저장소의 `datasets/morai_camera_vehicle`임.

```text
images/000000.jpg       원본 전방 영상
labels/000000.txt       class x_center y_center width height
metadata/000000.json    timestamp, 동기화 방식, 객체 및 투영 정보
overlays/000000.jpg     자동 라벨 검수 영상
manifest.jsonl          저장 프레임 목록과 session ID
```

## 실행

같은 PC에서 MORAI를 실행하는 현재 구성의 센서 네트워크 값은 다음과 같음.

```text
ROS Bridge: ws://127.0.0.1:9090
전방 카메라 UDP 목적지: 127.0.0.1:9092
Time Manager: Real Time Mode
```

ROS Bridge, 전방 UDP 수신기, 자동 라벨 저장 노드와 검수 창을 한 번에
실행하려면 다음 스크립트를 사용함.

```bash
cd /home/david/morai-mpc-agent-morai-lio-gps-integration
./scripts/collect_camera_vehicle_data.sh train
```

두 번째 인자로 명시적인 주행 ID를 지정할 수 있음.

```bash
./scripts/collect_camera_vehicle_data.sh train train_drive_01
./scripts/collect_camera_vehicle_data.sh val validation_drive_01
./scripts/collect_camera_vehicle_data.sh test test_drive_01
```

아래는 구성 요소를 따로 실행해야 할 때 사용하는 상세 방법임.

MORAI에서 전방 카메라 목적지 포트를 `9092`로 설정하고 ROS Bridge를 연결한
후 실행함. launch는 기본적으로 기존 `around_view` UDP 카메라 수신기도 같이
시작함.

```bash
cd /home/david/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

roslaunch camera_vehicle_training record_camera_vehicle_dataset.launch \
  session_id:=drive_01
```

이미 `/around_view/camera/front/image_raw`이 발행 중이면 UDP 포트 중복 사용을
피하기 위해 다음과 같이 실행함.

```bash
roslaunch camera_vehicle_training record_camera_vehicle_dataset.launch \
  start_camera_receiver:=false session_id:=drive_01
```

확인 토픽:

```bash
rostopic hz /around_view/camera/front/image_raw
rostopic hz /Ego_topic
rostopic hz /Object_topic
rostopic echo /camera_vehicle_training/status
rqt_image_view /camera_vehicle_training/label_overlay
```

일시 정지와 재개:

```bash
rosservice call /camera_vehicle_dataset_recorder/set_enabled "data: false"
rosservice call /camera_vehicle_dataset_recorder/set_enabled "data: true"
```

## 데이터 검증

```bash
rosrun camera_vehicle_training validate_camera_vehicle_dataset.py \
  /home/david/morai-mpc-agent-morai-lio-gps-integration/datasets/morai_camera_vehicle
```

자동 투영은 기하학적으로 보이는 후보를 생성하지만 건물이나 다른 차량에 의한
가림을 완전히 판단하지는 못함. 학습 전에 `overlays`를 검수하고 잘못된 박스는
수정하거나 해당 프레임을 제외해야 함.

## YOLO 학습

검수가 끝난 데이터는 주행 session 단위로 학습/검증 세트를 분리함. 연속된
프레임이 양쪽에 섞여 검증 점수가 부풀려지는 것을 막기 위해 프레임 단위 무작위
분할은 사용하지 않음.

```bash
cd /home/david/morai-mpc-agent-morai-lio-gps-integration
./scripts/prepare_camera_vehicle_yolo_dataset.py
./scripts/train_camera_vehicle_yolo.sh test
```

짧은 테스트가 정상적으로 끝나면 정식 학습을 실행함.

전방 카메라 원본은 1280x720임. Ultralytics 학습은 한 변 기준 `imgsz`를
사용하므로 `imgsz=1280 rect=True`로 설정하여 원본을 축소하지 않고 16:9
비율을 유지함. 모델 stride 배수 조건에 필요한 최소 세로 패딩만 추가됨.

```bash
./scripts/train_camera_vehicle_yolo.sh train
```

추가 Ultralytics 인자는 뒤에 붙일 수 있음. 예를 들어 기존 실행을 이어서 새
이름으로 저장하려면 다음과 같이 실행함.

```bash
./scripts/train_camera_vehicle_yolo.sh train name=morai_camera_vehicle_v2
```

## MORAI 실시간 차량 검출

MORAI 전방 카메라 UDP 목적지를 `127.0.0.1:9092`로 설정한 뒤 다음 명령으로
전방 카메라 수신기, YOLO 추론 노드, 결과 창을 함께 실행함.

```bash
cd /home/david/morai-mpc-agent-morai-lio-gps-integration
./scripts/run_camera_vehicle_detection.sh
```

이미 다른 launch에서 `/around_view/camera/front/image_raw`을 발행하고 있다면
UDP 9092 포트 중복을 피하도록 수신기를 끄고 실행함.

```bash
./scripts/run_camera_vehicle_detection.sh false
```

결과 토픽은 다음과 같음.

```text
/camera_vehicle_detection/overlay       박스가 그려진 bgr8 영상
/camera_vehicle_detection/detections    박스, 신뢰도, 지연시간 JSON
```

카메라 위치·해상도·FOV, 저장 간격과 필터 설정은
`config/camera_vehicle_dataset.yaml`에서 변경함. 카메라 위치나 FOV를 MORAI에서
변경했다면 이 설정도 같은 값으로 수정해야 함.
