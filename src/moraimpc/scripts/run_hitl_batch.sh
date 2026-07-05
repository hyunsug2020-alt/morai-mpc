#!/bin/bash
# 대표 시나리오 실제 follower HITL 배치
source /opt/ros/noetic/setup.bash
source /home/coss/catkin_ws/devel/setup.bash
run(){
  local name="$1" ego="$2" kmh="$3" ts="$4" specs="$5"
  rm -f /tmp/hitl_result.json
  timeout 80 roslaunch moraimpc hitl.launch ego_lane:=$ego cruise_kmh:=$kmh sim_time:=$ts npc_specs:="$specs" result_file:=/tmp/hitl_result.json >/tmp/hitl_launch.log 2>&1
  printf "%-22s " "$name"; cat /tmp/hitl_result.json 2>/dev/null || echo "결과없음"; echo
}
run "저속앞차추월@50"   2 50 20 "[[2,0.45,4.0]]"
run "박스인양옆막힘@45"  3 45 22 "[[3,0.42,3.0],[2,0.5,0.0],[4,0.5,0.0]]"
run "전차선정지@45"     2 45 22 "[[1,0.5,0.0],[2,0.5,0.0],[3,0.5,0.0],[4,0.5,0.0]]"
run "밀집혼잡@50"       2 50 20 "[[2,0.4,3.5],[3,0.6,6.0],[1,0.75,5.0],[4,0.55,7.0]]"
run "연속2대@45"        2 45 22 "[[2,0.35,3.0],[2,0.62,4.0]]"
echo "ALLDONE"
