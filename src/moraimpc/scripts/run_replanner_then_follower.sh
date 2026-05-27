#!/bin/bash
# launch-prefix wrapper: smoothing → path_replanner (NPC 회피 path 생성) → path_follower
# 사용: $0 <recorded_file> <smoothed_path> <replanned_path> <node_binary> [args...]
set -e
RECORDED="$1"; PATH_SMOOTH="$2"; PATH_FINAL="$3"; shift 3
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1) smoothing
if [ -n "$RECORDED" ] && [ -f "$RECORDED" ]; then
    /usr/bin/python3 "$SCRIPT_DIR/smooth_waypoints.py" "$RECORDED" "$PATH_SMOOTH" \
        --spacing 0.3 --window 21 --max-kappa 0.22 --max-kappa-passes 20 \
        && echo "[wrap] smoothing OK"
fi

# 2) replanning (NPC 위치 받아 lateral shift)
NODES_DIR="$(dirname "$SCRIPT_DIR")/nodes"
/usr/bin/python3 "$NODES_DIR/path_replanner.py" \
    _in_path:="$PATH_SMOOTH" _out_path:="$PATH_FINAL" \
    _shift_max:=0.6 _bulge_half_front_m:=25.0 _bulge_half_back_m:=15.0 \
    _lat_thresh:=3.0 _wait_npcs_sec:=3.0 \
    && echo "[wrap] replanning OK"

# 3) path_follower 실행
exec "$@"
