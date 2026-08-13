#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
OUTPUT_DIR="$REPO_ROOT/third_party/OpenPCDet/output/custom_models/pointpillar_pedestrian/morai_pedestrian_30k"
REPORT="$REPO_ROOT/trained_models/morai_pedestrian_pointpillar_2026-08-01/training_summary.json"

while true; do
    if [[ -s $REPORT ]] && grep -q '"status": "complete"' "$REPORT"; then
        printf 'completed archive already exists: %s\n' "$REPORT"
        exit 0
    fi
    TRAIN_LOG=$(find "$OUTPUT_DIR" -maxdepth 1 -type f -name 'train_*.log' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-)
    TRAINING_RUNNING=false
    if pgrep -u bisa -f 'train.py .*pointpillar_pedestrian.yaml' >/dev/null; then
        TRAINING_RUNNING=true
    fi
    if [[ -n $TRAIN_LOG ]] && grep -q 'End evaluation' "$TRAIN_LOG" && [[ $TRAINING_RUNNING == false ]]; then
        break
    fi
    if [[ $TRAINING_RUNNING == false ]]; then
        printf 'training failed before final evaluation; finalization stopped\n' >&2
        exit 70
    fi
    sleep 60
done

exec "$REPO_ROOT/.venv-openpcdet/bin/python" "$SCRIPT_DIR/finalize_training.py"
