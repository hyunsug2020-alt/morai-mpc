#!/usr/bin/env bash
set -euo pipefail

WS_DIR="/home/david/morai-mpc-agent-morai-lio-gps-integration"
LOG_DIR="/tmp/morai_lane_rviz"
mkdir -p "${LOG_DIR}"

source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

if ! rostopic list >/dev/null 2>&1; then
  echo "[run_hdmap_lane_rviz] ROS master starting via morai.launch"
fi

if ! pgrep -f "roslaunch morai_launch morai.launch" >/dev/null 2>&1; then
  nohup roslaunch morai_launch morai.launch >"${LOG_DIR}/morai_websocket.log" 2>&1 &
  echo "[run_hdmap_lane_rviz] morai.launch started in background (log: ${LOG_DIR}/morai_websocket.log)"
  sleep 5
else
  echo "[run_hdmap_lane_rviz] morai.launch already running"
fi

exec roslaunch moraimpc hdmap_lane_rviz.launch
