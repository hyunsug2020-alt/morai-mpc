#!/usr/bin/env bash
set -eo pipefail

repo="/home/david/morai-mpc-agent-morai-lio-gps-integration"
data_split="${1:-train}"
session_id="${2:-${data_split}_$(date +%Y%m%d_%H%M%S)}"

case "$data_split" in
  train|val|test) ;;
  *)
    printf 'usage: %s [train|val|test] [session_id]\n' "$0" >&2
    exit 2
    ;;
esac

source /opt/ros/noetic/setup.bash
source "$repo/devel/setup.bash"
set -u

start_rosbridge=true
start_topic_init=true
start_camera_receiver=true
ros_nodes=""

# A normal MORAI bringup may already own rosbridge and UDP camera ports.
# Reuse those nodes instead of replacing same-named nodes or rebinding ports.
if ros_nodes="$(rosnode list 2>/dev/null)"; then
  if printf '%s\n' "$ros_nodes" | grep -Eq '(^|/)camera_vehicle_dataset_recorder$'; then
    printf 'ERROR: camera dataset collection is already running. Stop it with Ctrl+C before starting another session.\n' >&2
    exit 3
  fi
  if printf '%s\n' "$ros_nodes" | grep -Eq '(^|/)rosbridge_websocket$'; then
    start_rosbridge=false
  fi
  if printf '%s\n' "$ros_nodes" | grep -Eq '(^|/)morai_topic_init$'; then
    start_topic_init=false
  fi
  if printf '%s\n' "$ros_nodes" | grep -Eq '(^|/)udp_camera$'; then
    start_camera_receiver=false
  fi
fi

printf 'MORAI local settings: ROS Bridge ws://127.0.0.1:9090, front camera UDP 127.0.0.1:9092\n'
printf 'dataset split=%s session=%s\n' "$data_split" "$session_id"
printf 'launch new nodes: rosbridge=%s topic_init=%s camera_receiver=%s\n' \
  "$start_rosbridge" "$start_topic_init" "$start_camera_receiver"

exec roslaunch camera_vehicle_training collect_camera_vehicle_dataset.launch \
  data_split:="$data_split" session_id:="$session_id" \
  start_rosbridge:="$start_rosbridge" \
  start_topic_init:="$start_topic_init" \
  start_camera_receiver:="$start_camera_receiver"
