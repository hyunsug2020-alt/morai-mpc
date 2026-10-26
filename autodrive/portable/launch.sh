#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/common.sh"
"${COMPOSE[@]}" up -d ros semantic rpc
export ROS_MASTER_URI=http://127.0.0.1:11312 ROS_IP=127.0.0.1
for _ in $(seq 1 60); do
  if timeout 2 docker exec morai_autodrive /opt/autodrive/ros/entrypoint.sh rostopic list >/dev/null 2>&1; then
    # E2E launch: one lap from the start point, then stop there (the duration slot is ignored).
    exec bash "$BUNDLE_ROOT/autodrive/student_v3_follow_claude.sh" "${1:-0}" 90.0 localization single_pc
  fi
  sleep .5
done
echo 'ROS stack did not become ready; check docker logs morai_autodrive.' >&2
exit 1
