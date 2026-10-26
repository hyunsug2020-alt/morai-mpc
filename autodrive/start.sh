#!/usr/bin/env bash
# Bring up the autodrive ROS stack container and the dashboard (http://127.0.0.1:8767).
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd -P)"
cd "$ROOT"
mkdir -p data/autodrive
if [[ -n "${AUTODRIVE_NO_BUILD:-}" ]] || docker image inspect morai-autodrive:1 >/dev/null 2>&1; then
  docker compose -f autodrive/compose.yaml up -d
else
  docker compose -f autodrive/compose.yaml up -d --build
fi
PID_FILE="data/autodrive/web.pid"
if systemctl --user cat morai-collector-dashboard.service >/dev/null 2>&1; then
  systemctl --user restart morai-collector-dashboard.service
  echo "dashboard managed by systemd user service"
elif [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "dashboard already running (pid $(cat "$PID_FILE"))"
else
  AUTODRIVE_MODE="${AUTODRIVE_MODE:-$(cat autodrive/.mode 2>/dev/null || echo full)}" nohup python3 autodrive/web_app.py --port "${AUTODRIVE_PORT:-8767}" > data/autodrive/web.log 2>&1 &
  echo $! > "$PID_FILE"
  echo "dashboard started (pid $!)"
fi
echo "자율주행 관제실: http://127.0.0.1:${AUTODRIVE_PORT:-8767}"
echo "ROS 스택 로그:   docker logs -f morai_autodrive"
