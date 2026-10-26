#!/usr/bin/env bash
# One driving session: RGB (host UDP) + Ego/Route + all /Object_topic obstacles (ROS container) recorded together, then audited.
# usage: collect/collect_session.sh <session_id> <duration_seconds>
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd -P)"
SESSION_ID="${1:?session_id}"
DURATION="${2:?duration_seconds}"
ROS_CONTAINER="${ROS_CONTAINER:-morai_autodrive}"   # project-owned ROS stack (autodrive/); ros1_bev_dh also works via ROS_EXEC
SESSION="$ROOT/data/raw/$SESSION_ID"
RGB_STOP_ARGS=()
ROS_STOP_ARGS=()
if [[ -n "${AUTODRIVE_CAPTURE_STOP_FILE:-}" ]]; then
  [[ "$AUTODRIVE_CAPTURE_STOP_FILE" == "$SESSION/.student_stop" ]] || { echo 'invalid capture stop file' >&2; exit 2; }
  RGB_STOP_ARGS=(--stop-file "$AUTODRIVE_CAPTURE_STOP_FILE")
  ROS_STOP_ARGS=(--stop-file "/data/raw/$SESSION_ID/.student_stop")
fi

if [[ -e "$SESSION" ]] && [[ -n "$(ls -A "$SESSION")" ]]; then
  echo "session exists and is not empty: $SESSION" >&2; exit 1
fi
mkdir -p "$SESSION"

if [[ "$ROS_CONTAINER" == "morai_autodrive" ]]; then
  ROS_EXEC=(docker exec -i "$ROS_CONTAINER" /opt/autodrive/ros/entrypoint.sh python3 -u -)
else
  ROS_EXEC=(docker exec -i "$ROS_CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; source /root/ros1_ws/devel/setup.bash 2>/dev/null; exec python3 -u - \"\$@\"" --)
fi
"${ROS_EXEC[@]}" --duration-seconds "$((${DURATION%.*} + 3))" "${ROS_STOP_ARGS[@]}" \
  < "$ROOT/collect/record_ego.py" > "$SESSION/ego_raw.jsonl" 2> "$SESSION/ego_recorder.log" &
EGO_PID=$!
"${ROS_EXEC[@]}" --duration-seconds "$((${DURATION%.*} + 3))" "${ROS_STOP_ARGS[@]}" \
  < "$ROOT/collect/record_objects.py" > "$SESSION/objects.jsonl" 2> "$SESSION/objects_recorder.log" &
OBJ_PID=$!

cleanup() { kill "$EGO_PID" "$OBJ_PID" 2>/dev/null || true; }
trap cleanup INT TERM

sleep 1.5   # let the ROS subscriber attach before cameras start being saved
RGB_RC=0
python3 "$ROOT/collect/collect_rgb.py" --session-id "$SESSION_ID" --output-root "$ROOT/data/raw" \
  --duration-seconds "$DURATION" "${RGB_STOP_ARGS[@]}" || RGB_RC=$?

wait "$EGO_PID" || true
wait "$OBJ_PID" || true
trap - INT TERM
echo "--- ego recorder ---"; tail -n 2 "$SESSION/ego_recorder.log" || true
echo "--- object recorder ---"; tail -n 1 "$SESSION/objects_recorder.log" || true
echo "--- audit ---"
if python3 -c "import numpy" 2>/dev/null; then
  python3 "$ROOT/collect/finalize_session.py" "$SESSION"
else  # collector-only PCs have no numpy on the host: run the audit inside the ROS container (python3-numpy there)
  docker exec -i "$ROS_CONTAINER" /opt/autodrive/ros/entrypoint.sh python3 - "/data/raw/$SESSION_ID" < "$ROOT/collect/finalize_session.py"
fi
if ((RGB_RC != 0)); then
  echo "RGB capture failed with exit $RGB_RC" >&2
  exit "$RGB_RC"
fi
