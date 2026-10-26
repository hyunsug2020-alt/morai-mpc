#!/usr/bin/env bash
set -euo pipefail
COPY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ASSETS="${STUDENT_ASSETS:-$COPY_ROOT}"
export STUDENT_ASSETS="$ASSETS"
export STUDENT_PYTHON="$COPY_ROOT/competition_python.py"
export STUDENT_VEHICLE_PYTHON="$COPY_ROOT/competition_vehicle_python.sh"
exec bash "$COPY_ROOT/../student_v3_follow_claude.sh" "$@"
