#!/usr/bin/env bash
set -euo pipefail

WS_DIR="/home/david/morai-mpc-agent-morai-lio-gps-integration"
MASTER_URI="${ROS_MASTER_URI:-http://localhost:11319}"
LOG_DIR="/tmp/lane_change_stress"
mkdir -p "${LOG_DIR}"

source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

tests=(
  "lane12|1|28|2,1|70"
  "lane12321|1|32|2,3,2,1|90"
  "lane1234321|1|28|2,3,4,3,2,1|120"
  "lane32123|3|30|2,1,2,3|100"
  "lane12321_fast|1|38|2,3,2,1|100"
)

fail=0
for item in "${tests[@]}"; do
  IFS='|' read -r name ego_lane cruise_kmh sequence timeout_sec <<<"${item}"
  result_file="${LOG_DIR}/${name}.json"
  log_file="${LOG_DIR}/${name}.log"
  rm -f "${result_file}" "${log_file}"

  printf '[LaneStress] RUN %s ego=%s cruise=%skmh sequence=%s\n' "${name}" "${ego_lane}" "${cruise_kmh}" "${sequence}"
  set +e
  ROS_MASTER_URI="${MASTER_URI}" timeout "${timeout_sec}" roslaunch moraimpc hdmap_lane_change_hitl.launch \
    ego_lane:="${ego_lane}" \
    cruise_kmh:="${cruise_kmh}" \
    sequence:="${sequence}" \
    result_file:="${result_file}" \
    >"${log_file}" 2>&1
  rc=$?
  set -e

  if [[ ! -f "${result_file}" ]]; then
    printf '[LaneStress] FAIL %s no result rc=%s log=%s\n' "${name}" "${rc}" "${log_file}"
    tail -40 "${log_file}" || true
    fail=1
    continue
  fi

  python3 - "$name" "$result_file" "$log_file" <<'PY'
import json
import sys
name, result_file, log_file = sys.argv[1:4]
with open(result_file) as f:
    data = json.load(f)
lanes = [str(x["lane"]) for x in data.get("history", [])]
ok = bool(data.get("success"))
print("[LaneStress] %s %s elapsed=%s max_abs_lat=%s lanes=%s result=%s" % (
    "PASS" if ok else "FAIL", name, data.get("elapsed"), data.get("max_abs_lat"),
    "->".join(lanes), result_file
))
if not ok:
    print("[LaneStress] reason=%s log=%s" % (data.get("reason"), log_file))
    sys.exit(2)
PY
  if [[ $? -ne 0 ]]; then
    fail=1
  fi
done

exit "${fail}"
