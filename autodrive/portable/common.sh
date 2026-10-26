#!/usr/bin/env bash
# Shared target-PC paths for setup, validation and the single-PC launcher.
set -euo pipefail
BUNDLE_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../.." && pwd -P)"
if [[ ! -f "$BUNDLE_ROOT/runtime.env" ]]; then
  cp "$BUNDLE_ROOT/autodrive/portable/env.example" "$BUNDLE_ROOT/runtime.env"
  echo "Edit Python/model paths in $BUNDLE_ROOT/runtime.env, then run again." >&2
  exit 2
fi
set -a
source "$BUNDLE_ROOT/runtime.env"
set +a
export STUDENT_MPC_WS="$BUNDLE_ROOT/localization"
export STUDENT_ASSETS="$BUNDLE_ROOT/autodrive/student_v3_claude"   # E2E launch code + data
export SEMANTIC_CODE="$BUNDLE_ROOT/semantic"
export LOCAL_UID="$(id -u)" LOCAL_GID="$(id -g)"
# Source-PC core numbers are not transferable to another CPU.
export STUDENT_IO_CPUS='' STUDENT_COMPUTE_CPUS='' STUDENT_SIM_WORKER_CPUS='' AUTODRIVE_LIDAR_CPUS=''
export SEMANTIC_MAIN_CPUS='' SEMANTIC_BACKGROUND_CPUS='' SEMANTIC_DISPLAY_CPUS=''
COMPOSE=(docker compose -f "$BUNDLE_ROOT/autodrive/portable/compose.yaml")
