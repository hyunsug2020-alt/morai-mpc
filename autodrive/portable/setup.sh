#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/common.sh"
[[ -f /opt/ros/noetic/setup.bash ]] || { echo 'Host ROS Noetic is required; see 실행방법.txt.' >&2; exit 2; }
docker info >/dev/null
docker compose version >/dev/null
[[ -d "$MORAI_SAVEFILE_DIR" ]] || { echo "MORAI SaveFile directory missing: $MORAI_SAVEFILE_DIR" >&2; exit 2; }
[[ -x "$STUDENT_PYTHON" && -x "$STUDENT_VEHICLE_PYTHON" ]] || { echo 'Set executable Python paths in runtime.env.' >&2; exit 2; }
mkdir -p "$BUNDLE_ROOT/data/autodrive/semantic_live" "$BUNDLE_ROOT/data/raw" /dev/shm/alpamayo_semantic
set +u
source /opt/ros/noetic/setup.bash
set -u
catkin_make -C "$BUNDLE_ROOT/localization" -j2 -l2 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF \
  '-DCATKIN_WHITELIST_PACKAGES=morai_msgs;eskf;morai_control;morai_launch'
python3 "$BUNDLE_ROOT/morai/install_final_sensor.py" --savefile "$MORAI_SAVEFILE_DIR"
python3 - "$BUNDLE_ROOT" "$MORAI_SAVEFILE_DIR" <<'PY'
import hashlib, shutil, sys, time
from pathlib import Path
root, savefile = map(Path, sys.argv[1:])
source = root/'morai/scenarios/404_contest.json'
target = savefile/'Scenario/R_KR_PR_K-city_2025/404_contest.json'
target.parent.mkdir(parents=True, exist_ok=True)
if target.exists() and target.read_bytes() != source.read_bytes():
    shutil.copy2(target, target.with_name(target.name+'.bak.'+str(time.time_ns())))
shutil.copy2(source, target)
for folder in ('EgoNetwork',):
    source = root/'morai/scenarios'/folder/'404_contest.json'
    if source.exists():
        destination = target.parent/folder/source.name
        destination.parent.mkdir(exist_ok=True)
        if destination.exists() and destination.read_bytes() != source.read_bytes():
            shutil.copy2(destination, destination.with_name(destination.name+'.bak.'+str(time.time_ns())))
        shutil.copy2(source, destination)
print('Installed scenario:',target)
PY
"${COMPOSE[@]}" config --quiet
"${COMPOSE[@]}" build
echo 'Setup complete. Load 404_contest and the single_pc sensor/network presets in MORAI, then run 01_check.sh.'
