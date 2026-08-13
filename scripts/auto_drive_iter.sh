#!/bin/bash
# 1 cycle: launch (auto via service) → 90s drive → i(reset) → analyze
# q 키는 launch가 이미 service로 ON 시키므로 토글 OFF되어 생략
set -e
ITER="${1:-iter0}"
WS=/home/david/morai-mpc-agent-morai-lio-gps-integration
LOG_OUT="$WS/src/moraimpc/logs/run_${ITER}.json"
KEY=/tmp/morai_key.py
PNG_OUT="/tmp/${ITER}.png"

source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

echo "[$(date +%T)] iter=$ITER START log=$LOG_OUT"

# 이전 잔존 정리
pkill -9 -f "path_follower_node|mpc_dashboard_node|morai_mode_init" 2>/dev/null || true
sleep 1

# launch
roslaunch moraimpc mpc_tracking.launch log_file:="$LOG_OUT" \
    > /tmp/launch_${ITER}.log 2>&1 &
LAUNCH_PID=$!
sleep 10  # smoothing + init dwell

# ctrl_cmd_0 rate 확인
RATE=$(timeout 3 rostopic hz /ctrl_cmd_0 2>&1 | grep "average rate" | head -1 | awk '{print $3}')
echo "  ctrl_cmd_0 rate: $RATE Hz"

# 5초 후 vel 확인
V5=$(timeout 2 rostopic echo -n 1 /Ego_topic 2>&1 | grep -A1 "^velocity:" | tail -1 | awk '{print $2}')
echo "  vel @5s: $V5 m/s"

# 120s 주행
echo "[$(date +%T)] driving 120s..."
sleep 120

# 직후 ego
POST=$(timeout 2 rostopic echo -n 1 /Ego_topic 2>&1 | python3 -c "
import sys; lines=sys.stdin.read().split('\n'); out={}
for i,l in enumerate(lines):
    s=l.strip()
    if s.startswith('position:'):
        try:
            out['x']=float(lines[i+1].split(':')[1])
            out['y']=float(lines[i+2].split(':')[1])
        except:pass
    if s.startswith('velocity:'):
        try:out['vx']=float(lines[i+1].split(':')[1])
        except:pass
import json;print(json.dumps(out))
")
echo "  post-drive: $POST"

# 종료 (clean shutdown으로 flushLog 보장)
rosnode kill /path_follower_node 2>/dev/null || true
sleep 4
kill -SIGINT $LAUNCH_PID 2>/dev/null || true
sleep 5
# 그래도 살아있으면 강제 종료
if kill -0 $LAUNCH_PID 2>/dev/null; then
    pkill -9 -f "path_follower_node|mpc_dashboard_node|morai_mode_init" 2>/dev/null || true
    kill -9 $LAUNCH_PID 2>/dev/null || true
fi
sleep 1

# i 키 reset
DISPLAY=:1 python3 $KEY i 1 || true
sleep 1

# 분석
if [ -f "$LOG_OUT" ] && [ -s "$LOG_OUT" ]; then
    python3 $WS/scripts/analyze_run.py "$LOG_OUT" "$WS/src/moraimpc/data/mixed.json" "$PNG_OUT" 2>&1 | tail -18
else
    if [ -f "$WS/src/moraimpc/logs/mpc_log.json" ]; then
        cp "$WS/src/moraimpc/logs/mpc_log.json" "$LOG_OUT"
        python3 $WS/scripts/analyze_run.py "$LOG_OUT" "$WS/src/moraimpc/data/mixed.json" "$PNG_OUT" 2>&1 | tail -18
    else
        echo "  ⚠ no log"
    fi
fi
echo "[$(date +%T)] iter=$ITER DONE"
