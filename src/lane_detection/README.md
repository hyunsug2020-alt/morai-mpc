# Lane detection

학습된 SegFormer로 기존 전방 카메라 토픽의 차선을 분할한다. 이 패키지는
WebSocket, LiDAR/GPS/IMU, 카메라 UDP 수신기를 시작하지 않는다.

```bash
cd /home/bisa/morai-mpc-agent-morai-lio-gps-integration
source devel/setup.bash
roslaunch lane_detection lane_detection.launch
```

입력은 `/around_view/camera/front/image_raw`, 출력은 다음과 같다.

- `/lane_detection/mask`: 0~3 클래스 ID를 담은 `mono8` 마스크
- `/lane_detection/overlay`: 원본 영상 위에 라벨 색을 합성한 영상
- `/lane_detection/diagnostics`: 처리 속도, 지연시간, 클래스별 픽셀 수 JSON

이미 영상 뷰어가 실행 중이면 `show_viewer:=false`를 붙여 중복 창을 막을 수 있다.
