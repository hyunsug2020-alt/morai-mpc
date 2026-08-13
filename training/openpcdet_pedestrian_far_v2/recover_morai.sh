#!/usr/bin/env bash
set -Eeuo pipefail

repo="/home/david/morai-mpc-agent-morai-lio-gps-integration"
runtime_dir="${repo}/logs/far_v2"
raw_labels="${repo}/datasets/morai_lidar_pedestrian_far_v2_20260802/labels"
hold_file="${runtime_dir}/HOLD_DRIVING"
launcher="/home/david/MoraiLauncher_Lin/MORAISim.sh"
recovery_log="${runtime_dir}/morai_auto_recovery.log"
export DISPLAY="${MORAI_DISPLAY:-:0}"
mkdir -p "${runtime_dir}"

log() {
  printf '%s %s\n' "$(date --iso-8601=seconds)" "$*" | tee -a "${recovery_log}"
}

frame_count() {
  find "${raw_labels}" -maxdepth 1 -type f -name '*.json' -printf '.' | wc -c
}

desired_site() {
  local count=$1
  if (( count < 6500 )); then printf '1\n'
  elif (( count < 12500 )); then printf '2\n'
  elif (( count < 18500 )); then printf '3\n'
  else
    # Keep crash recovery on the same 1k-frame multisite rotation as the
    # pipeline monitor.  A recovered simulator must not fall back to one site.
    local rotation=(4 5 2 3 1)
    local slot=$(( ((count - 18500) / 1000) % ${#rotation[@]} ))
    printf '%s\n' "${rotation[$slot]}"
  fi
}

stop_driver() {
  mapfile -t pids < <(pgrep -f '^[[:space:]]*bash training/openpcdet_pedestrian/drive_far_collection.sh$' || true)
  for pid in "${pids[@]}"; do
    pkill -TERM -P "${pid}" 2>/dev/null || true
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 2
  mapfile -t pids < <(pgrep -f '^[[:space:]]*bash training/openpcdet_pedestrian/drive_far_collection.sh$' || true)
  ((${#pids[@]} == 0)) || kill -KILL "${pids[@]}" 2>/dev/null || true
}

if pgrep -f '^/home/david/.*/Simulator\.x86_64([[:space:]]|$)' >/dev/null; then
  log "simulator already running"
  exit 0
fi

touch "${hold_file}"
recovery_complete=0
launched_pid=
cleanup() {
  if (( recovery_complete )); then
    unlink "${hold_file}" 2>/dev/null || true
    return
  fi
  log "recovery incomplete; keeping driver on hold for retry"
  if [[ "${MORAI_RECOVERY_KEEP_ON_FAILURE:-0}" == "1" ]]; then
    log "debug keep requested; leaving incomplete simulator running"
    return
  fi
  if [[ -n "${launched_pid}" ]] && kill -0 "${launched_pid}" 2>/dev/null; then
    kill -TERM "${launched_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
stop_driver
log "starting MORAI on NVIDIA Vulkan display=${DISPLAY}"
setsid "${launcher}" >> "${runtime_dir}/morai_auto_recovery.stdout.log" 2>&1 < /dev/null &
launched_pid=$!

window=
for _ in $(seq 1 60); do
  window="$(xdotool search --onlyvisible --name '^Simulator$' 2>/dev/null | tail -n 1 || true)"
  [[ -n "${window}" ]] && break
  sleep 1
done
[[ -n "${window}" ]] || { log "recovery failed: simulator window missing"; exit 69; }

timeout 3 xdotool windowsize "${window}" 1920 820 || true
# Vulkan shader/UI initialization regularly needs longer than the window
# creation itself.  Clicking while the map selector is still initializing is
# silently ignored and leaves the simulator at the launcher screen.
sleep 30
xdotool windowactivate --sync "${window}"
xdotool windowfocus --sync "${window}"

slow_click() {
  xdotool windowfocus --sync "${window}"
  xdotool mousemove --window "${window}" "$1" "$2"
  xdotool mousedown 1
  sleep 0.3
  xdotool mouseup 1
}

source /opt/ros/noetic/setup.bash
source "${repo}/devel/setup.bash"

# Direct simulator startup opens Map and Vehicle.  The Unity list row only
# accepts pointer input over part of its width, so sweep three safe x positions
# and verify map startup from /Ego_topic instead of assuming a click landed.
map_started=0
for startup_attempt in 1 2 3; do
  xdotool windowactivate --sync "${window}"
  xdotool windowfocus --sync "${window}"
  for x in 370 430 500; do slow_click "${x}" 216; sleep 0.4; done
  sleep 2
  xdotool mousemove --window "${window}" 1460 450
  for _ in $(seq 1 18); do
    xdotool click 5
    sleep 0.12
  done
  sleep 2
  for x in 1400 1460 1520; do slow_click "${x}" 553; sleep 0.4; done
  sleep 2
  slow_click 1525 655

  for _ in $(seq 1 12); do
    if timeout 3 rostopic echo -n 1 /Ego_topic >/dev/null 2>&1; then
      map_started=1
      break
    fi
    sleep 5
  done
  (( map_started )) && break
  log "recovery map-start retry attempt=${startup_attempt}"
done
if (( ! map_started )); then
  log "recovery failed: K-City map did not start after=3_attempts"
  exit 68
fi

site="$(desired_site "$(frame_count)")"
site_loaded=0
for attempt in 1 2 3; do
  if "${repo}/training/openpcdet_pedestrian_far_v2/load_collection_site.sh" "${site}" >> "${recovery_log}" 2>&1; then
    site_loaded=1
    break
  fi
  log "recovery site-load retry site=${site} attempt=${attempt}"
  sleep 8
done
if (( ! site_loaded )); then
  log "recovery failed: site load failed site=${site} after=3_attempts"
  exit 70
fi

lidar_ready=0
for _ in $(seq 1 2); do
  if timeout 6 rostopic hz -w 3 /velodyne_points 2>/dev/null | grep -q 'average rate'; then
    lidar_ready=1
    break
  fi
  sleep 4
done

# A fresh MORAI process can restore the 3D LiDAR as visually connected while
# its previous network session is stale (SESSION_NOT_EXIST).  Cycling the
# LiDAR connection in Sensor Edit Mode creates a new UDP session.
if (( ! lidar_ready )); then
  log "lidar session stale; reconnecting 3D LiDAR"
  xdotool windowactivate --sync "${window}"
  xdotool windowfocus --sync "${window}"
  xdotool key --clearmodifiers F3
  sleep 3
  slow_click 208 505
  sleep 2
  slow_click 204 751
  sleep 2
  slow_click 204 751
  sleep 3
  xdotool key --clearmodifiers F3
  sleep 2
  xdotool key --clearmodifiers Escape
  sleep 6
fi

for _ in $(seq 1 10); do
  if timeout 6 rostopic hz -w 3 /velodyne_points 2>/dev/null | grep -q 'average rate'; then
    lidar_ready=1
    break
  fi
  sleep 4
done
if (( ! lidar_ready )); then
  log "recovery failed: lidar did not resume"
  exit 71
fi
log "recovery complete site=${site} frames=$(frame_count)"
recovery_complete=1
