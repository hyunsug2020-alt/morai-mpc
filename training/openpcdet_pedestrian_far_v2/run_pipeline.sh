#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
RAW_DIR="$REPO_ROOT/datasets/morai_lidar_pedestrian_far_v2_20260802"
PREPARED_DIR="$REPO_ROOT/datasets/morai_pedestrian_far_v2_openpcdet"
# Collect extra raw frames because legacy frames without an ego pose and fully
# static scenes are intentionally removed before multisweep preparation.  The
# user-facing requirement is at least 30k *prepared* training/evaluation
# frames, not merely 30k files on disk.
TARGET=35000
MIN_PREPARED_FRAMES=30000
LOG_DIR="$REPO_ROOT/logs/far_v2"
PIPELINE_LOG="$LOG_DIR/pipeline.log"
PRETRAINED="$REPO_ROOT/trained_models/morai_pedestrian_pointpillar_2026-08-01/best_model_epoch_40.pth"
SITE_STATE="$LOG_DIR/current_site"
DRIVER_HOLD="$LOG_DIR/HOLD_DRIVING"
mkdir -p "$LOG_DIR"

log() {
  printf '%s %s\n' "$(date --iso-8601=seconds)" "$*" | tee -a "$PIPELINE_LOG"
}

frame_count() {
  find "$RAW_DIR/labels" -maxdepth 1 -type f -name '*.json' -printf '.' | wc -c
}

start_driver_if_needed() {
  # Allows scenario/network recovery without the autonomous key driver racing
  # the MORAI menus.  Removing the file resumes normal supervision.
  [[ -e "$DRIVER_HOLD" ]] && return 0
  if ! pgrep -f '^[[:space:]]*bash training/openpcdet_pedestrian/drive_far_collection.sh$' >/dev/null; then
    log "driver missing; restarting"
    cd "$REPO_ROOT"
    setsid training/openpcdet_pedestrian/drive_far_collection.sh >> "$LOG_DIR/driver.stdout.log" 2>&1 < /dev/null &
    echo $! > "$LOG_DIR/driver.pid"
  fi
}

stop_driver() {
  mapfile -t driver_pids < <(pgrep -f '^[[:space:]]*bash training/openpcdet_pedestrian/drive_far_collection.sh$' || true)
  if ((${#driver_pids[@]})); then
    kill -TERM "${driver_pids[@]}" || true
    sleep 3
    mapfile -t driver_pids < <(pgrep -f '^[[:space:]]*bash training/openpcdet_pedestrian/drive_far_collection.sh$' || true)
    if ((${#driver_pids[@]})); then kill -KILL "${driver_pids[@]}" || true; fi
  fi
}

desired_site() {
  local count=$1
  if (( count < 6500 )); then printf '1\n'
  elif (( count < 12500 )); then printf '2\n'
  elif (( count < 18500 )); then printf '3\n'
  else
    # The first 18.5k frames already cover sites 1-3.  Rotate the remaining
    # collection every 1k frames instead of dwelling at one location for 6k
    # frames, so the final dataset keeps genuine map/location diversity.
    local rotation=(4 5 2 3 1)
    local slot=$(( ((count - 18500) / 1000) % ${#rotation[@]} ))
    printf '%s\n' "${rotation[$slot]}"
  fi
}

switch_site_if_needed() {
  local count=$1 wanted current attempt loaded=0
  wanted="$(desired_site "$count")"
  current="$(cat "$SITE_STATE" 2>/dev/null || printf '1')"
  [[ "$wanted" == "$current" ]] && return 0
  stop_driver
  log "switching collection site current=$current next=$wanted frames=$count"
  for attempt in 1 2 3; do
    if "$SCRIPT_DIR/load_collection_site.sh" "$wanted" 2>&1 | tee -a "$PIPELINE_LOG"; then
      loaded=1
      break
    fi
    log "site switch retry site=$wanted attempt=$attempt"
    sleep 3
  done
  if (( ! loaded )); then
    log "site switch failed site=$wanted after=3_attempts"
    return 72
  fi
  log "site switch complete site=$wanted frames=$count"
}

log "pipeline monitoring started target=$TARGET"
while true; do
  count=$(frame_count)
  if (( count >= TARGET )); then
    log "collection complete frames=$count"
    break
  fi
  if ! pgrep -f 'roslaunch lidar_detection record_pedestrian_far_v2.launch' >/dev/null; then
    log "collector stopped before target at frames=$count"
    exit 70
  fi
  if ! pgrep -f '^/home/david/.*/Simulator\.x86_64([[:space:]]|$)' >/dev/null; then
    log "MORAI simulator stopped before target at frames=$count"
    exit 71
  fi
  switch_site_if_needed "$count"
  start_driver_if_needed
  log "collection progress frames=$count/$TARGET"
  sleep 60
done

stop_driver

source /opt/ros/noetic/setup.bash
source "$REPO_ROOT/devel/setup.bash"
rosservice call /lidar_dataset_recorder/set_enabled false >/dev/null 2>&1 || true
sleep 3

log "verifying all raw label/point-cloud pairs"
"$REPO_ROOT/.venv-openpcdet/bin/python" "$SCRIPT_DIR/verify_raw_dataset.py" "$RAW_DIR" --target "$TARGET" 2>&1 | tee -a "$PIPELINE_LOG"
log "preparing motion-compensated 3-sweep dataset"
"$REPO_ROOT/.venv-openpcdet/bin/python" "$SCRIPT_DIR/prepare_multisweep_dataset.py" --source "$RAW_DIR" --output "$PREPARED_DIR" 2>&1 | tee -a "$PIPELINE_LOG"
log "verifying at least ${MIN_PREPARED_FRAMES} prepared frames plus five-site and near/mid/far coverage"
"$REPO_ROOT/.venv-openpcdet/bin/python" "$SCRIPT_DIR/verify_prepared_coverage.py" \
  "$PREPARED_DIR/preparation_summary.json" \
  --min-total-frames "$MIN_PREPARED_FRAMES" 2>&1 | tee -a "$PIPELINE_LOG"
log "creating OpenPCDet info files"
"$REPO_ROOT/.venv-openpcdet/bin/python" "$SCRIPT_DIR/create_infos.py" --data-path "$PREPARED_DIR" --workers 4 2>&1 | tee -a "$PIPELINE_LOG"
log "verifying near/mid/far coverage inside the actual 30k train split"
"$REPO_ROOT/.venv-openpcdet/bin/python" "$SCRIPT_DIR/verify_training_split_distance.py" \
  "$PREPARED_DIR" --min-train-frames "$MIN_PREPARED_FRAMES" 2>&1 | tee -a "$PIPELINE_LOG"

validation_args=(--rounds 10 --batch-size 2 --config "$SCRIPT_DIR/pointpillar_pedestrian_far_v2.yaml" --pretrained "$PRETRAINED")
bios=$(tr -d '[:space:]' < /sys/class/dmi/id/bios_version)
if [[ $bios =~ ^[0-9]+$ ]] && (( 10#$bios < 2603 )); then
  validation_args+=(--allow-unsupported-bios)
fi
log "running ten-step protected CUDA validation"
"$REPO_ROOT/.venv-openpcdet/bin/python" "$REPO_ROOT/training/openpcdet_pedestrian/validate_training_stack.py" "${validation_args[@]}" 2>&1 | tee -a "$PIPELINE_LOG"

log "starting protected 30-epoch far-range training"
MORAI_TRAIN_CONFIG="$SCRIPT_DIR/pointpillar_pedestrian_far_v2.yaml" \
MORAI_TRAIN_TAG="morai_pedestrian_far_v2_30k" \
MORAI_TRAIN_PRETRAINED="$PRETRAINED" \
  "$REPO_ROOT/training/openpcdet_pedestrian/run_training.sh" 2>&1 | tee -a "$PIPELINE_LOG"
log "selecting and archiving best checkpoint"
"$REPO_ROOT/.venv-openpcdet/bin/python" "$SCRIPT_DIR/finalize_training.py" 2>&1 | tee -a "$PIPELINE_LOG"
log "pipeline complete"
