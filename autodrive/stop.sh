#!/usr/bin/env bash
set -uo pipefail
ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd -P)"
cd "$ROOT"
if systemctl --user cat morai-collector-dashboard.service >/dev/null 2>&1; then
  systemctl --user stop morai-collector-dashboard.service || true
fi
docker exec morai_autodrive /opt/autodrive/ros/ctl.sh follow-stop 2>/dev/null || true
PID_FILE="data/autodrive/web.pid"
[[ -f "$PID_FILE" ]] && kill "$(cat "$PID_FILE")" 2>/dev/null && rm -f "$PID_FILE" && echo "dashboard stopped"
docker compose -f autodrive/compose.yaml down
