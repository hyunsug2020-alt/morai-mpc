#!/usr/bin/env bash
set -eo pipefail

project_root="/home/david/morai-mpc-agent-morai-lio-gps-integration"
start_camera_receiver="${1:-true}"

case "$start_camera_receiver" in
  true|false) ;;
  *)
    printf 'usage: %s [true|false]\n' "$0" >&2
    printf '  true: start the MORAI front UDP receiver on port 9092\n' >&2
    printf '  false: reuse an already-running front camera topic\n' >&2
    exit 2
    ;;
esac

if command -v nvidia-modprobe >/dev/null 2>&1; then
  nvidia-modprobe -u -c=0 >/dev/null 2>&1 || true
fi

source /opt/ros/noetic/setup.bash
source "$project_root/devel/setup.bash"

exec roslaunch camera_vehicle_training camera_vehicle_detection.launch \
  start_camera_receiver:="$start_camera_receiver"
