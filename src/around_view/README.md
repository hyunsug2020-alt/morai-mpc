# around_view

MORAI UDP 카메라를 ROS `sensor_msgs/Image`로 발행하고 전방·좌측·우측
카메라를 하나의 어라운드 뷰 영상으로 합성하는 ROS Noetic 패키지임.

## 빌드

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
catkin_make
source devel/setup.bash
```

## 카메라 확인

MORAI 카메라의 목적지 포트를 `9092`~`9095`로 설정한 뒤 실행함.

```bash
roslaunch around_view camera_view.launch
```

화면 표시 없이 ROS 이미지 토픽만 발행하려면 다음과 같이 실행함.

```bash
roslaunch around_view camera_view.launch show_windows:=false
```

발행되는 기본 토픽은 다음과 같음.

```text
/around_view/camera/front/image_raw
/around_view/camera/left/image_raw
/around_view/camera/right/image_raw
/around_view/camera/rear/image_raw
```

## 어라운드 뷰

일반 어라운드뷰를 실행함. 저장된
`~/.ros/around_view/calibration.yaml`이 있으면 자동으로 적용됨.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch around_view around_view.launch
```

합성 영상은 `/around_view/image` 토픽으로도 발행됨. 영상 창에서 `q`를
누르면 종료됨.

LiDAR 포인트와 어라운드뷰를 한 화면에서 확인하고 보정하려면 실행함.
카메라 영상을 먼저 탑뷰로 합성하고, LiDAR에서 지면 점을 제거한 뒤 차량
좌표계의 물체 점을 탑뷰에 직접 옮겨 주황색 외곽선만 표시함. 카메라와
LiDAR가 서로 독립적인 좌표로 표시되므로 실제 정렬 상태를 비교할 수 있음.

`morai.launch`가 이미 실행 중이면 그 launch가 발행하는
`/velodyne_points`를 그대로 사용함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch around_view lidar_around_view_calibration.launch
```

`morai.launch` 없이 캘리브레이션만 단독으로 실행할 때는 Velodyne UDP
드라이버를 함께 시작함.

```bash
roslaunch around_view lidar_around_view_calibration.launch \
  start_lidar_driver:=true
```

통합 영상은 `/around_view/lidar_overlay`로 발행됨. AVM 창에서 `s`를 누르면
보정값을 저장하고, `q`를 누르면 종료함. 저장된 LiDAR 외부 보정값
`~/.ros/around_view/lidar_camera_calibration.yaml`은 자동으로 적용됨.

통합 윤곽선은 LiDAR 점을 임의로 건너뛰지 않고 전부 받은 뒤 다음 순서로
처리함.

1. 거리와 높이 범위 밖의 점을 제거함.
2. 주변 이웃이 부족한 고립점을 제거함.
3. 가까운 점을 물체별 군집으로 분리함.
4. 일반 물체는 측정 외곽선을, 차량으로 판단되는 군집은 회전 차량
   바운딩박스를 표시함.
5. 기본값은 최신 LiDAR 스캔만 즉시 표시해 이전 윤곽의 잔상을 남기지 않음.

관련 필터와 차량 크기 설정은
`config/lidar_camera_calibration.yaml`의 `projection` 항목에 있음. 차량
바운딩박스는 단일 LiDAR 스캔에서 보이지 않는 반대편 면을 추정한 결과이며,
실제 측정점만 확인하려면 `fit_vehicle_boxes: false`로 설정하면 됨.
벽·난간이 차량으로 오판되지 않도록 차량 박스는 기본적으로 차량 중심에서
좌우 `2.5m` 안쪽의 군집에만 적용함.
`contour_smoothing_alpha`를 `1.0`보다 낮추면 움직임은 부드러워지지만 이전
윤곽이 짧게 남을 수 있음.

MORAI 연결, LIO-SAM, ESKF와 함께 실행하려면 다음 명령을 사용함.

```bash
roslaunch morai_launch morai.launch start_around_view:=true
```

통합 launch에서는 어라운드뷰가 기본적으로 비활성화되어 있음.

## 고정 보정값

실행 중 조정 UI는 사용하지 않음. 카메라 투영값은
`config/calibration.yaml`, LiDAR 외부 보정값은
`config/lidar_camera_calibration.yaml`에서 관리함. 설정 변경 후 노드를
재시작해야 적용됨.

## 3카메라 합성 방식

전방·좌측·우측 카메라만 합성하며 후방 영상은 사용하지 않음. MORAI에 설정된
카메라 위치, 회전, FOV와 입력 해상도는 고정값으로 유지함.

근접 영역을 최대한 유지하기 위해 원본 영상 전체를 지면 투영에 사용함.
카메라 경계에는 블러나 페더링을 적용하지 않고 원본 픽셀을 사용함. 따라서
카메라에 보이는 후드나 사이드미러가 근접 영역에 함께 투영될 수 있음.

직진 차선에서는 고정된 전방 90도 FOV에 맞춘 부채꼴 영역을 전방 카메라가
우선 담당함. 전방 원본 영상 범위를 벗어나 값이 없는 좌우 영역만 좌측·우측
카메라 순서로 채움. 이 우선순위 방식으로 직진 차선은 전방 영상의 수직선을
유지하고, 측면 카메라가 중앙 차선을 방사형으로 늘려 덮는 현상을 방지함.

## 설정

- UDP 포트와 ROS 토픽: `config/cameras.yaml`
- 카메라 물리값, 캔버스, 보정값, 합성 영역: `config/calibration.yaml`
- 지면 입력 마스크: `source_ground_masks`
- 카메라 경계 혼합 폭: `seam_feather_pixels`
- 전방 사각 영역과 차량 마스크: `vehicle`
- `packet_format:=auto`: MORAI 11바이트 및 19바이트 UDP 헤더 자동 판별

## LiDAR–카메라 외부 보정

MORAI `25.S4.MolitComp03`의 고정 센서 위치와 FOV를 초기값으로 사용해
`/velodyne_points`를 전방·좌측·우측 영상에 투영함.

MORAI 시뮬레이터에서 센서 설정을 불러오고 Play 상태로 둔 다음 실행함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source devel/setup.bash
roslaunch around_view lidar_camera_calibration.launch
```

보정 도구를 실행해둔 상태에서 새 터미널을 열어 원본 360도 LiDAR RViz를
별도로 실행함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source devel/setup.bash
rviz -d "$(rospack find lio_sam)/launch/include/config/morai_lidar_raw.rviz"
```

LIO-SAM 결과와 RViz만 확인할 때는 다음 통합 명령을 사용함.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source devel/setup.bash
roslaunch morai_launch morai.launch start_lio_rviz:=true
```

각 영상에서 건물 모서리, 연석, 차량 외곽선에 LiDAR 색상 점이 겹치는지
확인함. `q`를 누르면 영상 창이 종료됨. MORAI의 고정 센서 위치·회전·FOV·
해상도와 잔여 보정값은 `config/lidar_camera_calibration.yaml`에서 관리함.

## LiDAR와 어라운드뷰 통합 확인

깨끗한 탑뷰 어라운드뷰를 먼저 합성하고 다음 순서로 독립적인 LiDAR 물체
외곽선을 겹쳐 카메라 지면 투영을 보정함.

1. LiDAR 점을 차량 좌표계로 변환함
2. 높이 `0.30~3.00 m`의 비지면 물체 점만 남김
3. 실제 차량 좌표 `x/y`를 탑뷰 캔버스에 직접 배치함
4. 인접 점을 군집화하고 외곽선만 주황색으로 그림

카메라 영상과 LiDAR를 같은 호모그래피로 함께 워핑하지 않으므로 둘이 항상
붙어서 움직이는 문제가 없으며, 실제 좌표 정렬 상태를 확인할 수 있음.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch around_view lidar_around_view_calibration.launch
```

- 통합 영상 토픽: `/around_view/lidar_overlay`
- 좌측 상단 상태: 사용한 물체 점 수, 검출 외곽선 수, 센서 시간차
- 통합 창에서 `q`: 통합 오버레이 노드 종료
- LiDAR 외부 보정값은
  `~/.ros/around_view/lidar_camera_calibration.yaml`을 자동으로 불러옴
