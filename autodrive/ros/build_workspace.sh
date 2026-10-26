#!/usr/bin/env bash
# Validate local Python sources and incrementally rebuild C++ before following.
set -euo pipefail
source /opt/ros/noetic/setup.bash
DATA="${AUTODRIVE_DATA:-/data/autodrive}"
mkdir -p "$DATA"
exec 8>/tmp/autodrive-build.lock
flock -w 250 8
LOG="$DATA/build.log"
echo "[build] checking local sources; log: $LOG"
if ! (
  cd /opt/autodrive/ws
  python3 - <<'PY' || exit 1
from pathlib import Path
for root in (Path('src'), Path('/opt/autodrive/ros')):
    for path in sorted(root.rglob('*.py')):
        compile(path.read_bytes(), str(path), 'exec')
PY
  timeout --kill-after=10s 240s catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release
) >"$LOG" 2>&1; then
  tail -n 60 "$LOG" >&2
  echo "[build] failed; follower will not start" >&2
  exit 1
fi
echo "[build] local sources ready"
