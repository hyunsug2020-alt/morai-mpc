#!/usr/bin/env bash
# stack: roscore(11312) + rosbridge(9091) + global path publisher + status writer. Follower is started
# separately by ctl.sh so the dashboard can toggle it. Any other argument is exec'd in the ROS environment.
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /opt/autodrive/ws/devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11312}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
DATA="${AUTODRIVE_DATA:-/data/autodrive}"
mkdir -p "$DATA"

if [[ "${1:-stack}" != "stack" ]]; then
  exec "$@"
fi

PATH_TXT="${AUTODRIVE_PATH_TXT:-$(rospack find moraimpc)/data/competition_directional_global_path.txt}"
python3 /opt/autodrive/ros/path_to_json.py "$PATH_TXT" "$DATA/global_path.json"
cp -f "$PATH_TXT" "$DATA/global_path.txt"

trap 'kill 0' EXIT INT TERM
roscore -p "${ROS_MASTER_URI##*:}" &
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
roslaunch rosbridge_server rosbridge_websocket.launch port:="${ROSBRIDGE_PORT:-9091}" &
python3 /opt/autodrive/ros/global_path_publisher.py _path_file:="$PATH_TXT" _output_topic:=/planning/global_path _frame_id:=map &
python3 /opt/autodrive/ros/status_node.py _output:="$DATA/status.json" _path_file:="$PATH_TXT" &
echo "[autodrive] stack up: master $ROS_MASTER_URI, rosbridge ws://127.0.0.1:${ROSBRIDGE_PORT:-9091}, status $DATA/status.json"
wait
