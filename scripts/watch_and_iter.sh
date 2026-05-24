#!/bin/bash
# 차량 unstuck 감지 → 자동 iter 진행 (user의 시뮬 reset 대기)
# /Ego_topic velocity > 0.3 m/s 또는 position 변동 > 5m 시 unstuck 판정
set -e
WS=/home/coss/morai-mpc
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

ITER_BASE=${1:-iter}
ITER_N=8

echo "[$(date +%T)] watch start — 차량 unstuck 대기 중..."
INIT_POS=$(timeout 3 rostopic echo -n 1 /Ego_topic 2>&1 | python3 -c "
import sys
lines = sys.stdin.read().split('\n')
for i,l in enumerate(lines):
    if l.strip().startswith('position:'):
        x = float(lines[i+1].split(':')[1])
        y = float(lines[i+2].split(':')[1])
        print(f'{x:.2f} {y:.2f}')
        break
")
echo "  init pos: $INIT_POS"
IX=$(echo $INIT_POS | awk '{print $1}')
IY=$(echo $INIT_POS | awk '{print $2}')

while true; do
    EGO=$(timeout 3 rostopic echo -n 1 /Ego_topic 2>&1 | python3 -c "
import sys
lines = sys.stdin.read().split('\n')
out = {}
for i,l in enumerate(lines):
    s=l.strip()
    if s.startswith('position:'):
        try:
            out['x']=float(lines[i+1].split(':')[1])
            out['y']=float(lines[i+2].split(':')[1])
        except: pass
    if s.startswith('velocity:'):
        try: out['vx']=float(lines[i+1].split(':')[1])
        except: pass
import json; print(json.dumps(out))
")
    X=$(echo $EGO | python3 -c "import json,sys; d=json.loads(sys.stdin.read()); print(d.get('x',0))")
    Y=$(echo $EGO | python3 -c "import json,sys; d=json.loads(sys.stdin.read()); print(d.get('y',0))")
    VX=$(echo $EGO | python3 -c "import json,sys; d=json.loads(sys.stdin.read()); print(abs(d.get('vx',0)))")

    # 거리 또는 속도 변화 감지
    DIST=$(python3 -c "import math; print(f'{math.hypot($X-$IX,$Y-$IY):.2f}')")
    UNSTUCK=$(python3 -c "print(1 if (abs($VX) > 0.3 or $DIST > 5.0) else 0)")

    echo "[$(date +%T)] pos=($X,$Y) |v|=$VX dist_from_init=$DIST unstuck=$UNSTUCK"

    if [ "$UNSTUCK" = "1" ]; then
        echo "[$(date +%T)] UNSTUCK 감지! iter $ITER_N 시작"
        bash $WS/scripts/auto_drive_iter.sh ${ITER_BASE}${ITER_N}
        ITER_N=$((ITER_N + 1))
        # 다음 init pos 갱신
        sleep 5
        INIT_POS=$(timeout 3 rostopic echo -n 1 /Ego_topic 2>&1 | python3 -c "
import sys
lines = sys.stdin.read().split('\n')
for i,l in enumerate(lines):
    if l.strip().startswith('position:'):
        x = float(lines[i+1].split(':')[1])
        y = float(lines[i+2].split(':')[1])
        print(f'{x:.2f} {y:.2f}')
        break
")
        IX=$(echo $INIT_POS | awk '{print $1}')
        IY=$(echo $INIT_POS | awk '{print $2}')
    fi
    sleep 5
done
