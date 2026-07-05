#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/coss/catkin_ws/devel/setup.bash
rm -f /tmp/hitl_result.json
roslaunch moraimpc hitl.launch ego_lane:=1 cruise_kmh:=40 sim_time:=30 npc_specs:="[[1,0.45,5.2]]" >/tmp/hitl_launch.log 2>&1 &
LP=$!
sleep 7
echo "=== 프로세스 생존 ==="; ps aux | grep -E "hdmap_lane_avoid|path_follower_node|vehicle_sim" | grep -v grep | awk '{print $11,$12,$13}'
echo "=== launch.log (planner 관련) ==="; grep -iE "hdmap_lane_avoid|lane_avoid|process has died|Traceback|Error|ImportError|No module|zip|현재|못찾" /tmp/hitl_launch.log | head -25
echo "=== launch.log 마지막 30줄 ==="; tail -30 /tmp/hitl_launch.log
kill $LP 2>/dev/null; pkill -f hdmap_lane_avoid; pkill -f path_follower_node; pkill -f vehicle_sim; pkill -f rosmaster; pkill -f roscore
sleep 1; echo DONE
