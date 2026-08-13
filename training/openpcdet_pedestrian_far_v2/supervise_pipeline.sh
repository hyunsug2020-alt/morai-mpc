#!/usr/bin/env bash
set -u

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
LOG_DIR="$REPO_ROOT/logs/far_v2"
LOG_FILE="$LOG_DIR/supervisor.log"
STOP_FILE="$LOG_DIR/STOP_SUPERVISOR"
SUMMARY="$REPO_ROOT/trained_models/morai_pedestrian_far_v2_pointpillar_30k_2026-08-02/training_summary.json"
PIPELINE_PID=
mkdir -p "$LOG_DIR"

log() {
  printf '%s %s\n' "$(date --iso-8601=seconds)" "$*" | tee -a "$LOG_FILE"
}

is_complete() {
  [[ -f "$SUMMARY" ]] && python3 - "$SUMMARY" <<'PY' >/dev/null 2>&1
import json, sys
document = json.load(open(sys.argv[1]))
raise SystemExit(0 if document.get("status") == "complete" else 1)
PY
}

cleanup() {
  if [[ -n "${PIPELINE_PID:-}" ]]; then
    kill -TERM "$PIPELINE_PID" 2>/dev/null || true
    wait "$PIPELINE_PID" 2>/dev/null || true
  fi
  mapfile -t drivers < <(pgrep -f '^[[:space:]]*bash training/openpcdet_pedestrian/drive_far_collection.sh$' || true)
  if ((${#drivers[@]})); then kill -TERM "${drivers[@]}" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

attempt=0
log "supervisor started"
while [[ ! -e "$STOP_FILE" ]]; do
  if is_complete; then
    log "verified training summary complete; supervisor exiting"
    exit 0
  fi
  attempt=$((attempt + 1))
  log "pipeline attempt=$attempt starting"
  cd "$REPO_ROOT"
  "$SCRIPT_DIR/run_pipeline.sh" >> "$LOG_DIR/pipeline.stdout.log" 2>&1 &
  PIPELINE_PID=$!
  wait "$PIPELINE_PID"
  result=$?
  PIPELINE_PID=
  if (( result == 0 )) && is_complete; then
    log "pipeline completed and final summary verified"
    exit 0
  fi
  log "pipeline exited rc=$result; retrying in 30 seconds"
  sleep 30
done
log "stop file detected; supervisor exiting"
