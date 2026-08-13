#!/usr/bin/env bash
set -Eeuo pipefail

site="${1:?usage: load_collection_site.sh SITE_NUMBER}"
case "${site}" in
  1) scenario="far_pedestrian_v2_50k_20260802" ;;
  2|3|4|5) scenario="far_pedestrian_v2_site${site}_20260802" ;;
  *) printf 'unsupported site: %s\n' "${site}" >&2; exit 64 ;;
esac

export DISPLAY="${MORAI_DISPLAY:-:0}"
repo="/home/david/morai-mpc-agent-morai-lio-gps-integration"
runtime_dir="${repo}/logs/far_v2"
scenario_file="/home/david/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/${scenario}.json"
mkdir -p "${runtime_dir}"
[[ -f "${scenario_file}" ]] || { printf 'scenario missing: %s\n' "${scenario_file}" >&2; exit 66; }

window="$(xdotool search --onlyvisible --name '^Simulator$' 2>/dev/null | tail -n 1 || true)"
[[ -n "${window}" ]] || { printf 'MORAI Simulator window not found\n' >&2; exit 69; }

slow_click() {
  xdotool windowfocus --sync "${window}"
  xdotool mousemove --window "${window}" "$1" "$2"
  xdotool mousedown 1
  sleep 0.3
  xdotool mouseup 1
}

# Keep Unity's menu/dialog coordinates deterministic on the large physical
# desktop.  Earlier collection silently missed site changes after the window
# was resized, so every load now normalizes the client geometry first.
timeout 3 xdotool windowsize "${window}" 1920 820 || true
# The desktop window manager clamps this large window away from (0,0) and
# returns a non-zero status even though the geometry is usable.
timeout 3 xdotool windowmove "${window}" 0 0 || true
# Unity needs a few frames after a Vulkan swap-chain resize.  Inputs sent
# during that interval are silently dropped, leaving the old site active.
sleep 5

# Open Edit -> Scenario -> Load Scenario.
xdotool windowactivate --sync "${window}"
xdotool windowfocus --sync "${window}"
xdotool key --clearmodifiers Escape
sleep 1
slow_click 34 10
sleep 0.5
xdotool mousemove --window "${window}" 90 98
sleep 0.5
slow_click 290 141
sleep 2

# Scenario rows are sorted as the original followed by site2..site5. Direct row
# selection avoids Unity occasionally dropping synthetic text in the search box.
row_y=$((310 + (site - 1) * 25))
slow_click 920 "${row_y}"
sleep 0.8
slow_click 920 "${row_y}"
sleep 1
slow_click 1008 637
sleep 8

# Load Scenario keeps Pause Mode checked. Escape resumes simulation. Saved
# scenarios start in Built-In mode in this MORAI build; switch once with Q to
# Keyboard immediately so the ego cannot drive tens of metres away while the
# loaded pose and pedestrian count are being verified.
xdotool key --clearmodifiers Escape
sleep 0.5
xdotool key --clearmodifiers q
sleep 2

source /opt/ros/noetic/setup.bash
source "${repo}/devel/setup.bash"
expected_pedestrians="$(python3 - "${scenario_file}" <<'PY'
import json, sys
print(len(json.load(open(sys.argv[1]))["pedestrianList"]))
PY
)"
pedestrians="$(timeout 5 rostopic echo -n 1 -p /Object_topic/num_of_pedestrian 2>/dev/null | tail -n 1 | awk -F, '{print $2}' || true)"
minimum_pedestrians=$((expected_pedestrians - 2))
if [[ -z "${pedestrians}" ]] || (( pedestrians < minimum_pedestrians )); then
  printf 'site load verification failed: pedestrian_count=%s minimum=%s configured=%s\n' \
    "${pedestrians:-missing}" "${minimum_pedestrians}" "${expected_pedestrians}" >&2
  exit 70
fi

read -r actual_x actual_y < <(
  timeout 5 rostopic echo -n 1 -p /Ego_topic/position 2>/dev/null \
    | tail -n 1 | awk -F, '{print $2, $3}'
)
read -r expected_x expected_y < <(
  python3 - "${scenario_file}" <<'PY'
import json, sys
p = json.load(open(sys.argv[1]))["egoVehicle"]["initPosition"]["pos"]
print(p["x"], p["y"])
PY
)
if ! awk -v ax="${actual_x}" -v ay="${actual_y}" -v ex="${expected_x}" -v ey="${expected_y}" \
  'BEGIN { dx=ax-ex; dy=ay-ey; exit !((dx*dx+dy*dy) < 100.0) }'; then
  printf 'site load verification failed: expected=(%s,%s) actual=(%s,%s)\n' \
    "${expected_x}" "${expected_y}" "${actual_x}" "${actual_y}" >&2
  exit 71
fi

printf '%s\n' "${site}" > "${runtime_dir}/current_site"
printf '%s site=%s scenario=%s pedestrians=%s\n' \
  "$(date --iso-8601=seconds)" "${site}" "${scenario}" "${pedestrians}" >> "${runtime_dir}/site_switches.log"
printf 'loaded site=%s scenario=%s pose=(%s,%s) pedestrians=%s\n' \
  "${site}" "${scenario}" "${actual_x}" "${actual_y}" "${pedestrians}"
