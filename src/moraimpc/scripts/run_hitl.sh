#!/bin/bash
# HITL 단일 시나리오 실행: run_hitl.sh <ego_lane> <cruise_kmh> <sim_time> '<npc_specs_json>'
source /opt/ros/noetic/setup.bash
source /home/coss/catkin_ws/devel/setup.bash
EGO=${1:-1}; KMH=${2:-45}; TS=${3:-22}; SPECS=${4:-[]}
rm -f /tmp/hitl_result.json
timeout 90 roslaunch moraimpc hitl.launch \
    ego_lane:=$EGO cruise_kmh:=$KMH sim_time:=$TS npc_specs:="$SPECS" \
    result_file:=/tmp/hitl_result.json >/tmp/hitl_launch.log 2>&1
echo "=== RESULT ==="
cat /tmp/hitl_result.json 2>/dev/null || { echo "결과없음 — launch 로그 tail:"; tail -20 /tmp/hitl_launch.log; }
echo
