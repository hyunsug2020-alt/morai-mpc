#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
SUMMARY="$REPO_ROOT/trained_models/morai_pedestrian_far_v2_pointpillar_30k_2026-08-02/training_summary.json"
MODEL_DIR="$REPO_ROOT/trained_models/morai_pedestrian_far_v2_pointpillar_30k_2026-08-02"

is_ready() {
  [[ -s "$SUMMARY" ]] || return 1
  python3 - "$SUMMARY" <<'PY' >/dev/null 2>&1
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    summary = json.load(stream)
raise SystemExit(0 if summary.get("status") == "complete" else 1)
PY
}

printf '%s waiting for verified training archive\n' "$(date --iso-8601=seconds)"
until is_ready; do
  sleep 15
done

for artifact in \
  "$MODEL_DIR/best_model.pth" \
  "$MODEL_DIR/pointpillar_pedestrian_far_v2.yaml" \
  "$MODEL_DIR/morai_pedestrian_far_v2_dataset.yaml"; do
  if [[ ! -e "$artifact" ]]; then
    printf '%s missing required artifact: %s\n' "$(date --iso-8601=seconds)" "$artifact" >&2
    exit 1
  fi
done

cd "$REPO_ROOT"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export DISPLAY=${DISPLAY:-:20}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

if rosnode list 2>/dev/null | grep -Fxq /pedestrian_far_v2_pointpillar_detector; then
  printf '%s detector is already running\n' "$(date --iso-8601=seconds)"
  exit 0
fi

printf '%s starting pedestrian detection and RViz\n' "$(date --iso-8601=seconds)"
exec roslaunch lidar_detection pedestrian_far_v2_detection.launch
