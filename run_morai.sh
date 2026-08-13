#!/usr/bin/env bash

set -eo pipefail

workspace_dir="/home/david/morai-mpc-agent-morai-lio-gps-integration"

cd "$workspace_dir"
source /opt/ros/noetic/setup.bash
source "$workspace_dir/devel/setup.bash"

morai_connection_count() {
  ss -Htn state established 2>/dev/null \
    | awk '$4 ~ /:9090$/ || $5 ~ /:9090$/ { count++ } END { print count+0 }'
}

wait_for_rosbridge() {
  local attempt
  for attempt in $(seq 1 30); do
    if ss -Hltn 2>/dev/null | awk '$4 ~ /:9090$/ { found=1 } END { exit !found }'; then
      return 0
    fi
    sleep 0.5
  done
  echo "오류: rosbridge 9090 포트가 열리지 않았음" >&2
  return 1
}

configure_morai() {
  if ! command -v xdotool >/dev/null 2>&1; then
    echo "오류: MORAI 자동 연결에 xdotool이 필요함" >&2
    return 1
  fi

  local simulator_window
  local geometry action_x action_y close_x close_y
  simulator_window="$(xdotool search --onlyvisible --name '^Simulator$' \
    2>/dev/null | head -n 1)"
  if [[ -z "$simulator_window" ]]; then
    echo "오류: 실행 중인 MORAI Simulator 창을 찾지 못했음" >&2
    return 1
  fi

  wait_for_rosbridge

  geometry="$(xdotool getwindowgeometry --shell "$simulator_window")"
  eval "$geometry"
  # MORAI 25.S4 Network Settings positions scale with the Simulator window.
  # Ratios are based on the current 1920x1043 Linux layout and remain stable
  # when the secondary display resolution or window size changes.
  action_x=$((WIDTH * 643 / 1000))
  action_y=$((HEIGHT * 753 / 1000))
  close_x=$((WIDTH * 693 / 1000))
  close_y=$((HEIGHT * 238 / 1000))

  # MORAI 25.S4 Linux uses a latched UI state for Ego ROS networking.  When
  # rosbridge is restarted the UI can still say Connected while its sockets
  # are closed. Open Network Settings and press the action button once. If it
  # was the stale Disconnect state, press the resulting Connect button again.
  xdotool windowraise "$simulator_window"
  xdotool windowfocus --sync "$simulator_window"
  xdotool key F4
  sleep 0.8
  if [[ "$(morai_connection_count)" -eq 0 ]]; then
    xdotool mousemove --window "$simulator_window" \
      "$action_x" "$action_y" click 1
    sleep 4
    if [[ "$(morai_connection_count)" -eq 0 ]]; then
      xdotool mousemove --window "$simulator_window" \
        "$action_x" "$action_y" click 1
      sleep 4
    fi
  fi
  if [[ "$(morai_connection_count)" -eq 0 ]]; then
    echo "오류: MORAI Ego WebSocket 재연결에 실패했음" >&2
    return 1
  fi

  # Close Network Settings. MORAI 25.S4 Linux window-relative coordinates
  # are stable across window placement changes.
  xdotool mousemove --window "$simulator_window" \
    "$close_x" "$close_y" click 1
  sleep 0.3

  # Competition mode leaves Time Manager untouched and uses MORAI's default
  # Real Time Mode.
  echo "MORAI WebSocket 연결 완료 (Time Manager 변경 없음)"
}

selection="${1:-}"
if [[ -z "$selection" ]]; then
  echo "1) MORAI 연결만 (WebSocket + 모든 UDP 센서)"
  echo "2) 전체 통합 + Around View"
  echo "3) 전체 통합 (Around View 제외)"
  read -r -p "선택 [1/2/3]: " selection
fi

case "$selection" in
  1)
    launch_file="morai_connections.launch"
    launch_args=()
    ;;
  2)
    launch_file="morai.launch"
    launch_args=("start_around_view:=true" "use_sim_time:=false")
    ;;
  3)
    launch_file="morai.launch"
    launch_args=("start_around_view:=false" "use_sim_time:=false")
    ;;
  *)
    echo "오류: 1, 2 또는 3을 입력해야 함" >&2
    exit 2
    ;;
esac

if [[ "$launch_file" == "morai_connections.launch" ]]; then
  echo "MORAI 연결 실행 중: WebSocket + LiDAR/GPS/IMU/카메라 UDP"
else
  if [[ "$selection" == "2" ]]; then
    around_view="true"
  else
    around_view="false"
  fi
  echo "MORAI 전체 통합 실행 중: start_around_view:=$around_view"
fi
# roslaunch must open port 9090 before MORAI is told to reconnect. Keep the
# setup helper in the background and leave roslaunch in the foreground so
# Ctrl-C and its exit status behave normally.
configure_morai &
exec roslaunch morai_launch "$launch_file" "${launch_args[@]}"
