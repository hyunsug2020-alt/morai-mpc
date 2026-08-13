#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
VALIDATION_ARGS=(--rounds 10 --batch-size 2)
BIOS_VERSION=$(tr -d '[:space:]' < /sys/class/dmi/id/bios_version)
if [[ $BIOS_VERSION =~ ^[0-9]+$ ]] && (( 10#$BIOS_VERSION < 2603 )); then
    VALIDATION_ARGS+=(--allow-unsupported-bios)
fi
"$REPO_ROOT/.venv-openpcdet/bin/python" \
    "$SCRIPT_DIR/validate_training_stack.py" "${VALIDATION_ARGS[@]}"
exec "$SCRIPT_DIR/run_training.sh"
