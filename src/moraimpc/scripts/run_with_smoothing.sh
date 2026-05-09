#!/bin/bash
# launch-prefix wrapper: 추종 시작 전 waypoints 후처리 자동 실행
# 사용 (mpc_tracking.launch에서 path_follower_node에 launch-prefix로 부착):
#   $0  recorded_file  path_file  /node/binary  __args...
# 동작:
#   - recorded_file 존재하면 smooth_waypoints.py로 후처리 → path_file 갱신
#   - 실패하거나 recorded_file 없으면 기존 path_file 사용
#   - 마지막에 exec "$@"로 path_follower_node 실행 (동기)
RECORDED="$1"; PATHF="$2"; shift 2
# 스크립트 자체 위치 기반 (rospack 실패해도 안정)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SMOOTH="$SCRIPT_DIR/smooth_waypoints.py"

if [ -n "$RECORDED" ] && [ -f "$RECORDED" ]; then
    echo "[run_with_smoothing] post-process: $RECORDED -> $PATHF"
    if /usr/bin/python3 "$SMOOTH" "$RECORDED" "$PATHF" \
            --spacing 0.3 --window 21 --max-kappa 0.22 --max-kappa-passes 20; then
        echo "[run_with_smoothing] smoothing OK"
    else
        echo "[run_with_smoothing] smoothing 실패 — 기존 $PATHF 사용"
    fi
else
    echo "[run_with_smoothing] recorded 없음 ($RECORDED) — 기존 $PATHF 사용"
fi
exec "$@"
