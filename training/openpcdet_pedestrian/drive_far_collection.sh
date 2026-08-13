#!/usr/bin/env bash
set -u

# MORAI must run on the physical NVIDIA X display.  The old :20 display uses
# llvmpipe and cannot receive input for the live simulator window.
export DISPLAY="${MORAI_DISPLAY:-:0}"
source /opt/ros/noetic/setup.bash
source /home/david/morai-mpc-agent-morai-lio-gps-integration/devel/setup.bash
runtime_dir="/home/david/morai-mpc-agent-morai-lio-gps-integration/logs/far_v2"
stop_file="${runtime_dir}/STOP_DRIVING"
log_file="${runtime_dir}/driving_cycles.log"
site_state="${runtime_dir}/current_site"
site_loader="/home/david/morai-mpc-agent-morai-lio-gps-integration/training/openpcdet_pedestrian_far_v2/load_collection_site.sh"
mkdir -p "${runtime_dir}"
rm -f "${stop_file}"

cleanup() {
  local window
  window="$(xdotool search --onlyvisible --name '^Simulator$' 2>/dev/null | tail -n 1 || true)"
  if [[ -n "${window}" ]]; then
    xdotool windowactivate --sync "${window}" 2>/dev/null || true
    xdotool keyup w a d s 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ensure_keyboard_control() {
  local window start_x start_y end_x end_y
  window="$(xdotool search --onlyvisible --name '^Simulator$' 2>/dev/null | tail -n 1 || true)"
  [[ -n "${window}" ]] || return 1
  xdotool windowactivate --sync "${window}"
  xdotool windowfocus --sync "${window}"
  # A freshly loaded scenario can take more than a second to engage Drive and
  # settle the vehicle on the road.  A short 0.8 s probe falsely classified
  # valid keyboard mode as broken at the later collection sites.
  for _ in 1 2 3; do
    read -r start_x start_y < <(
      timeout 3 rostopic echo -n 1 -p /Ego_topic/position 2>/dev/null \
        | tail -n 1 | awk -F, '{print $2, $3}'
    )
    xdotool mousemove --window "${window}" 960 410
    xdotool mousedown 1
    sleep 0.2
    xdotool mouseup 1
    xdotool keydown w
    sleep 2.5
    xdotool keyup w
    read -r end_x end_y < <(
      timeout 3 rostopic echo -n 1 -p /Ego_topic/position 2>/dev/null \
        | tail -n 1 | awk -F, '{print $2, $3}'
    )
    if [[ -n "${start_x:-}" && -n "${end_x:-}" ]] && \
      awk -v sx="${start_x}" -v sy="${start_y}" -v ex="${end_x}" -v ey="${end_y}" \
        'BEGIN { dx=ex-sx; dy=ey-sy; exit !((dx*dx+dy*dy) >= 0.04) }'; then
      return 0
    fi
    # I returns to the map's built-in spawn on sites 2-5, so only use it on
    # site 1.  Later sites must retain their distinct scenario coordinates.
    if [[ "${current_site:-1}" == "1" ]]; then
      xdotool key --clearmodifiers i
      sleep 2
    fi
    xdotool key --clearmodifiers q
    sleep 2
  done
  printf '%s keyboard_mode_probe_failed\n' "$(date --iso-8601=seconds)" >> "${log_file}"
  return 1
}

cycle=0
while [[ ! -e "${stop_file}" ]]; do
  window="$(xdotool search --onlyvisible --name '^Simulator$' 2>/dev/null | tail -n 1 || true)"
  if [[ -z "${window}" ]]; then
    printf '%s simulator_window_missing\n' "$(date --iso-8601=seconds)" >> "${log_file}"
    sleep 2
    continue
  fi

  current_site="$(cat "${site_state}" 2>/dev/null || printf '1')"
  # Reload every site before each approach.  Site 1 formerly relied on I, but
  # after repeated Q mode transitions this build can ignore I and let the ego
  # drift far beyond the pedestrian field.  A scenario reload resets ego and
  # pedestrians together and is verified by load_collection_site.sh.
  if ! "${site_loader}" "${current_site}" >> "${log_file}" 2>&1; then
    printf '%s site_reload_failed site=%s\n' \
      "$(date --iso-8601=seconds)" "${current_site}" >> "${log_file}"
    sleep 3
    continue
  fi

  if [[ "${current_site}" != "1" ]] || (( cycle % 20 == 0 )); then
    ensure_keyboard_control || { sleep 2; continue; }
  fi

  route=$((cycle % 7))
  printf '%s cycle=%d route=%d start\n' "$(date --iso-8601=seconds)" "${cycle}" "${route}" >> "${log_file}"
  xdotool windowactivate --sync "${window}"

  # At site 1 only, MORAI's built-in reset returns to the correct origin. Other
  # sites were reloaded together with their pedestrian field above.
  if [[ "${current_site}" == "1" ]]; then
    xdotool key --clearmodifiers i
    sleep 2
  fi

  # Two routes begin farther behind the field and dwell there. This raises the
  # proportion of sparse but important 35-45 m pedestrian returns before the
  # normal near/mid-range approach starts.
  if [[ "${route}" -eq 0 ]] || [[ "${route}" -eq 6 ]]; then
    printf '%s cycle=%d far_dwell_start\n' \
      "$(date --iso-8601=seconds)" "${cycle}" >> "${log_file}"
    sleep 4
  fi

  xdotool keydown w
  sleep 3.0
  xdotool keyup w
  sleep 2

  if [[ "${route}" -eq 0 ]]; then
    xdotool keydown w a
    sleep 1.0
    xdotool keyup w a
  elif [[ "${route}" -eq 1 ]]; then
    xdotool keydown w a
    sleep 0.65
    xdotool keyup w a
  elif [[ "${route}" -eq 2 ]]; then
    xdotool keydown w a
    sleep 0.3
    xdotool keyup w a
  elif [[ "${route}" -eq 3 ]]; then
    xdotool keydown w
    sleep 1.2
    xdotool keyup w
  elif [[ "${route}" -eq 4 ]]; then
    xdotool keydown w d
    sleep 0.3
    xdotool keyup w d
  elif [[ "${route}" -eq 5 ]]; then
    xdotool keydown w d
    sleep 0.65
    xdotool keyup w d
  else
    xdotool keydown w d
    sleep 1.0
    xdotool keyup w d
  fi

  sleep 5
  xdotool keydown s
  sleep 1.5
  xdotool keyup s
  sleep 2
  cycle=$((cycle + 1))
done
