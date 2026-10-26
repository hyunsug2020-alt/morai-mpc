#!/usr/bin/env bash
# The competition launch (E2E): drives autodrive/student_v3_claude. Started as a copy of the rule-based
# student_v3_follow.sh (2026-10-10); that launch and student_v3 were removed on 2026-10-11.
# Shared competition drive for explicit single_pc/two_pc profiles and sensor collection.
#  - Position comes ONLY from GPS+IMU ESKF (/localization/ego_status). /Ego_topic is never
#    used for driving or for the start/liveness checks.
#  - Obstacles: semantic camera x LiDAR fusion (/perception/objects) + LiDAR clearance; /Object_topic is not used.
#  - GPS blackout: inside the tunnel stretch of the route (competition has no GPS there)
#    gps_blackout_gate.py drops every fix; fixed tunnel walls aid lateral/yaw odometry.
#  - gui.py reads MORAI camera UDP directly; frames are saved only for collection.
#  - Every run keeps a debug record in data/autodrive/follow_runs/<run>/.
#  - One lap, no time limit: the car drives one full lap of the closed route from wherever it
#    stands at launch and brakes smoothly to rest back at that start point (gui.py +
#    student_v3_claude/lap_finish.py). gui.py then writes <run>/lap_complete.json and the launch
#    shuts down through the normal cleanup. The first argument (duration seconds, still sent by
#    the dashboard) is accepted for compatibility and ignored. STUDENT_ONE_LAP=0 drives on until
#    the dashboard's stop (/api/stop) instead.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd -P)"
export AUTODRIVE_ROOT="$ROOT"
DURATION_ARG="${1:-0}"; KMH="${2:-90}"   # duration: ignored (one lap, no time limit)
ONE_LAP="${STUDENT_ONE_LAP:-1}"
export STUDENT_SIGNAL_GUARD="${STUDENT_SIGNAL_GUARD:-1}"
[[ "$STUDENT_SIGNAL_GUARD" == 0 || "$STUDENT_SIGNAL_GUARD" == 1 ]] || { echo 'STUDENT_SIGNAL_GUARD must be 0 or 1' >&2; exit 2; }
[[ "$ONE_LAP" == 0 || "$ONE_LAP" == 1 ]] || { echo 'STUDENT_ONE_LAP must be 0 or 1' >&2; exit 2; }
if [[ -n "${3:-}" && "$3" != localization ]]; then
  echo "competition launch uses GPS/IMU localization only (/Ego_topic is not allowed)" >&2; exit 2
fi
DATA="$ROOT/data/autodrive"
SEMANTIC_CODE="${SEMANTIC_CODE:-$ROOT/semantic}"
ASSETS="${STUDENT_ASSETS:-$ROOT/autodrive/student_v3_claude}"   # test a staging copy in MORAI without touching live
[[ -s "$ASSETS/semantic_fusion.py" ]] || { echo "semantic fusion entrypoint missing: $ASSETS/semantic_fusion.py" >&2; exit 2; }
# Shared modules (lidar_vehicle_detection.py, semantic_fusion.py) put STUDENT_V3_DIR first on
# sys.path so the helper processes this script starts import this code copy (STUDENT_ASSETS).
export STUDENT_V3_DIR="$ASSETS"
CHECKPOINT="${STUDENT_CHECKPOINT:-$ASSETS/best_v5.pt}"
CTL=(docker exec morai_autodrive /opt/autodrive/ros/ctl.sh)
PYTHON="${STUDENT_PYTHON:-/home/david/miniconda3/envs/lane_tool/bin/python}"
MPC_WS="${STUDENT_MPC_WS:-$ROOT/localization}"
NETWORK_PROFILE="${4:-two_pc}"
if [[ -z "${4:-}" ]]; then
  echo 'network profile omitted: using two_pc (대회용 · PC 2대)'
fi
SESSION_ID="${5:-}"
PROFILE_ENV="$(python3 "$ROOT/autodrive/competition_profiles.py" shell "$NETWORK_PROFILE")"
eval "$PROFILE_ENV"
python3 "$ROOT/autodrive/competition_profiles.py" check "$NETWORK_PROFILE"
if [[ -n "$SESSION_ID" ]]; then
  [[ "$NETWORK_PROFILE" == single_pc && "$SESSION_ID" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$ ]] || { echo 'sensor collection requires single_pc and a valid session id' >&2; exit 2; }
  SESSION="$ROOT/data/raw/$SESSION_ID"
  [[ ! -e "$SESSION" ]] || { echo "session exists: $SESSION" >&2; exit 2; }
fi
if [[ "$STUDENT_SIGNAL_GUARD" == 1 || -n "$SESSION_ID" ]]; then
  CAMERA_PORT_PATTERN="$CAMERA_PORT_PATTERN|$SIGNAL_CAMERA_PORT"
fi
EGO_TOPIC=/localization/ego_status
mkdir -p "$DATA"
[[ -x "$PYTHON" && -s "$CHECKPOINT" && -s "$ASSETS/global_path.json" ]] || { echo 'Student v5 runtime or model missing' >&2; exit 2; }
# E2E mode (model_avoidance.py): the v6.3 judgement model, its child process and the
# portable static/tunnel planner must all load before the car moves.
export STUDENT_V6_CHECKPOINT="${STUDENT_V6_CHECKPOINT:-$ASSETS/v6_model.pt}" STUDENT_V6_HZ="${STUDENT_V6_HZ:-10}"
[[ -s "$STUDENT_V6_CHECKPOINT" ]] || { echo "E2E: v6 model missing: $STUDENT_V6_CHECKPOINT" >&2; exit 2; }
(cd "$ASSETS" && "$PYTHON" -c "import v6, v6.shadow, v6.shadow_child, v6_ext_child, model_avoidance, tunnel_static_portable") \
  || { echo 'E2E: v6 / model_avoidance packages do not import' >&2; exit 2; }
echo "[E2E] v6 model: $STUDENT_V6_CHECKPOINT (${STUDENT_V6_HZ} Hz) · planner=model · tunnel=model"
[[ -f "$MPC_WS/devel/setup.bash" ]] || { echo "competition localization workspace missing: $MPC_WS" >&2; exit 2; }
python3 - "$KMH" "$DATA" <<'PY'
import math,shutil,sys
speed=float(sys.argv[1])
if not math.isfinite(speed) or not 5 <= speed <= 90:
    raise SystemExit('Student speed 5–90 km/h required')
free=shutil.disk_usage(sys.argv[2]).free
if free < 1024**3:
    raise SystemExit(f'녹화 디스크 공간 부족: 여유 {free/1024**2:.0f} MiB, 최소 1024 MiB 필요; 주행 시작 안 함')
PY

exec 8>"$DATA/drive.lock"
flock -n 8 || { echo 'another autonomous drive owns the vehicle' >&2; exit 3; }
# Docker can create the bind source as root on a fresh boot. The unprivileged
# camera feed must be able to write frames before the semantic pipeline starts.
SEMANTIC_FEED_DIR="${SEMANTIC_FEED_DIR:-/dev/shm/alpamayo_semantic}"
if [[ "$SEMANTIC_FEED_DIR" == /dev/shm/alpamayo_semantic && ! -w "$SEMANTIC_FEED_DIR" ]]; then
  docker exec -u 0 alpamayo_semantic chown "$(id -u):$(id -g)" "$SEMANTIC_FEED_DIR"
fi
mkdir -p "$SEMANTIC_FEED_DIR"
[[ -w "$SEMANTIC_FEED_DIR" ]] || { echo "semantic feed directory is not writable: $SEMANTIC_FEED_DIR" >&2; exit 1; }
python3 "$ROOT/autodrive/prepare_cpu.py"
# gui.py binds the camera UDP ports itself. Holding drive.lock stops the dashboard's
# 1 s camera-rate sniff from starting again; wait out one already in flight.
PORTS_FREE=0
for _ in $(seq 1 10); do
  if ! ss -Hlun 2>/dev/null | awk '{print $4}' | grep -qE ":($CAMERA_PORT_PATTERN)$"; then PORTS_FREE=1; break; fi
  sleep 0.5
done
if [[ "$PORTS_FREE" != 1 ]]; then
  echo "camera UDP ports $CAMERA_PORT_PATTERN still in use after 5 s:" >&2
  ss -Hlunp 2>/dev/null | grep -E ":($CAMERA_PORT_PATTERN)\\b" >&2 || true
  exit 1
fi
RUN="follow-$(date -u +%Y%m%dT%H%M%SZ)-$$"
export STUDENT_SIGNAL_RUN_ID="$RUN"
RUN_DIR="$DATA/follow_runs/$RUN"
MPC_LOG_FILE="/data/autodrive/follow_runs/$RUN/mpc.json"
[[ -n "$SESSION_ID" ]] && RUN_DIR="$DATA/mpc_runs/$SESSION_ID/$RUN"
[[ -n "$SESSION_ID" ]] && MPC_LOG_FILE="/data/autodrive/mpc_runs/$SESSION_ID/$RUN/mpc.json"
STATUS="$RUN_DIR/loc_status.json"
LAP_MARKER="$RUN_DIR/lap_complete.json"
# ctl.sh follow-start only accepts a run whose nonce is recorded here while drive.lock is held.
python3 - "$DATA/drive_run.json" "$RUN" "$DATA" "$SESSION_ID" <<'PY'
import json,os,sys,time
path,nonce,data,sid=sys.argv[1:]
record=dict(nonce=nonce,session_id=sid or None,capture_dir=data,owner_pid=os.getppid(),started_wall_ns=time.time_ns())
tmp=path+'.tmp.'+str(os.getpid())
with open(tmp,'w') as out: json.dump(record,out)
os.replace(tmp,path)
PY
mkdir -p "$RUN_DIR"
cp "$ROOT/autodrive/ros/ltv_speed_profile_competition.csv" "$RUN_DIR/ltv_speed_profile.csv"
cp "$ROOT/autodrive/ros/ioniq5_vehicle_competition.yaml" "$RUN_DIR/" 2>/dev/null || true
python3 "$ROOT/autodrive/competition_profiles.py" json "$NETWORK_PROFILE" > "$RUN_DIR/network_profile.json"
cp "$RUN_DIR/network_profile.json" "$DATA/network_profile.json"
python3 - "$RUN_DIR/run.json" "$RUN" "$CHECKPOINT" "$SESSION_ID" "$RUN_DIR/network_profile.json" "$ONE_LAP" <<'PY'
import json,os,sys,time
path,run,checkpoint,sid,profile_file=sys.argv[1:6]
with open(path,'w') as out:
    json.dump(dict(run=run,checkpoint=checkpoint,ego_source="localization",ego_topic="/localization/ego_status",
                   obstacles="semantic_fusion+lidar",gps_blackout=True,collect=bool(sid),session_id=sid or None,
                   network_profile=json.load(open(profile_file)),
                   rates_hz=dict(student=15,semantic_feed=15,semantic_inference=15,semantic_fusion=15),
                   signal_enabled=os.environ['STUDENT_SIGNAL_GUARD']=='1',
                   run_length="one_lap" if sys.argv[6]=="1" else "until_stopped",
                   started_wall_ns=time.time_ns()),out,indent=2)
PY
# Speed ladder (competition_speed.py): +5 % per clean run on normal roads and in curves.
read -r LADDER_KMH LADDER_ALAT LADDER_LEVEL < <(python3 "$ROOT/autodrive/competition_speed.py" plan --run-dir "$RUN_DIR")
[[ -n "${LADDER_ALAT:-}" ]] || { echo 'speed ladder plan failed' >&2; exit 1; }
echo "speed ladder level $LADDER_LEVEL: normal roads $LADDER_KMH km/h, curve lateral accel $LADDER_ALAT m/s^2"
# Controller heading: map-grid heading (1) matches the positions; true-north (0) is /Ego_topic's
# convention and left a speed-growing ~0.26 m right-side offset at 50 km/h.
GRID_HEADING="${STUDENT_GRID_HEADING:-1}"
python3 - "$RUN_DIR/run.json" "$LADDER_LEVEL" "$LADDER_KMH" "$LADDER_ALAT" "$GRID_HEADING" <<'PY'
import json,sys
path,level,kmh,alat,grid=sys.argv[1:]
meta=json.load(open(path)); meta.update(speed_level=int(level),normal_kmh=float(kmh),curve_alat_max=float(alat),grid_heading=grid=="1")
json.dump(meta,open(path,'w'),indent=2)
PY

# Reads the localization-only status written by localization_monitor.py (never /Ego_topic).
status_field() { python3 - "$STATUS" "$1" <<'PY'
import json,sys,time
try:
    d=json.load(open(sys.argv[1]))
    if time.time()-d.get('time',0)>3: raise ValueError('stale status')
    for part in sys.argv[2].split('.'): d=d[part]
    print(d)
except (OSError,ValueError,KeyError,TypeError): print('')
PY
}

GUI_PID=''; FOLLOWER_REQUESTED=''; LIDAR_STARTED=0; CLEANED=''; ESKF_PID=''; BAG_PID=''; MON_PID=''
BRIDGE_PID=''; EGO_RECORD_PID=''; OBJECT_RECORD_PID=''; VEHICLE_PID=''; START_BRAKE_PID=''
cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  [[ -n "$CLEANED" ]] && return; CLEANED=1
  if [[ -n "$FOLLOWER_REQUESTED" ]]; then
    "${CTL[@]}" follow-stop --run "$RUN" || rc=1
  fi
  # Car is stopped; shut the rest down in parallel so "작업중단" returns within a few seconds.
  local pids=()
  if [[ -n "$GUI_PID" ]] && kill -0 "$GUI_PID" 2>/dev/null; then
    kill -TERM "$GUI_PID" 2>/dev/null || true
    ( for _ in $(seq 1 10); do kill -0 "$GUI_PID" 2>/dev/null || exit 0; sleep 0.3; done; kill -KILL "$GUI_PID" 2>/dev/null ) &
    pids+=($!)
  fi
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    # SIGINT lets rosbag close and reindex the bag; the group includes its record child.
    kill -INT -- "-$BAG_PID" 2>/dev/null || true
    ( for _ in $(seq 1 20); do kill -0 "$BAG_PID" 2>/dev/null || exit 0; sleep 0.5; done ) &
    pids+=($!)
  fi
  if [[ -n "$MON_PID" ]] && kill -0 "$MON_PID" 2>/dev/null; then kill -TERM "$MON_PID" 2>/dev/null || true; fi
  if [[ -n "${FUSION_PID:-}" ]] && kill -0 "$FUSION_PID" 2>/dev/null; then kill -TERM "$FUSION_PID" 2>/dev/null || true; fi
  if [[ -n "$VEHICLE_PID" ]] && kill -0 "$VEHICLE_PID" 2>/dev/null; then kill -TERM "$VEHICLE_PID" 2>/dev/null || true; fi
  # Outside the competition launch GPS passes through unchanged.
  rosparam set /gps_blackout_gate/enabled false 2>/dev/null &
  pids+=($!)
  if [[ "$LIDAR_STARTED" == 1 ]]; then "${CTL[@]}" lidar-stop & pids+=($!); fi
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
  if [[ -n "$BRIDGE_PID" ]]; then
    kill -INT -- "-$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
  if [[ -n "$START_BRAKE_PID" ]]; then
    kill -TERM -- "-$START_BRAKE_PID" 2>/dev/null || true
    wait "$START_BRAKE_PID" 2>/dev/null || true
  fi
  if [[ -n "$SESSION_ID" && ( -n "$EGO_RECORD_PID" || -n "$OBJECT_RECORD_PID" ) ]]; then
    # Stop markers allow the container's JSONL writers to flush before the audit.
    touch "$SESSION/.student_stop"
    for pid in "$EGO_RECORD_PID" "$OBJECT_RECORD_PID"; do
      [[ -n "$pid" ]] && { wait "$pid" || rc=1; }
    done
    if [[ -f "$SESSION/session.json" ]]; then
      "$PYTHON" "$ROOT/collect/finalize_session.py" "$SESSION" --force-convention yaw=heading || rc=1
      # Planner decisions + surrounding (incl. rear) LiDAR cars per frame, for v6 training.
      cp "$RUN_DIR/student_gui_$SESSION_ID.jsonl" "$SESSION/decisions.jsonl" 2>/dev/null || true
    else
      echo 'sensor collection stopped before camera metadata was created' >&2
      rc=1
    fi
  fi
  if [[ -n "$ESKF_PID" ]]; then
    echo "note: this run started localization (ESKF, pid $ESKF_PID); left running. To stop: kill $ESKF_PID" >&2
  fi
  for log in follower.log eskf_localization.log gps_udp.log imu_udp.log gps_blackout_gate.log lidar_stream.log follower_run.json; do
    [[ -f "$DATA/$log" ]] && cp "$DATA/$log" "$RUN_DIR/" 2>/dev/null || true
  done
  if ls "$RUN_DIR"/drive*.bag >/dev/null 2>&1; then
    # Summary only; detached so stopping does not wait on a multi-GB bag scan.
    setsid bash -c 'rosbag info "$1"/drive*.bag > "$1/bag_info.txt" 2>&1' _ "$RUN_DIR" </dev/null >/dev/null 2>&1 &
  fi
  if [[ -n "$FOLLOWER_REQUESTED" && -f "$RUN_DIR/mpc.csv" ]]; then
    python3 "$ROOT/autodrive/competition_speed.py" evaluate --run-dir "$RUN_DIR" || true
  fi
  python3 - "$RUN_DIR/run.json" "$rc" "$LAP_MARKER" <<'PY'
import json,os,sys,time
path,rc,lap=sys.argv[1:]
with open(path) as source: meta=json.load(source)
meta.update(ended_wall_ns=time.time_ns(),exit_code=int(rc))
try: meta['lap']=json.load(open(lap))
except (OSError,ValueError): meta['lap']=None
with open(path+'.tmp','w') as target: json.dump(meta,target,indent=2)
os.replace(path+'.tmp',path)
PY
  echo "Student v5 competition launch stopped; logs $RUN_DIR; exit $rc"
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

export ROS_MASTER_URI='http://127.0.0.1:11312' ROS_IP='127.0.0.1'
unset ROS_NAMESPACE  # a namespaced node would bypass the /gps gate checks
set +u
source /opt/ros/noetic/setup.bash
source "$MPC_WS/devel/setup.bash"
set -u
CTL=(docker exec -e MORAI_SIM_IP="$MORAI_SIM_IP" -e MORAI_LIDAR_RPM="$MORAI_LIDAR_RPM" morai_autodrive /opt/autodrive/ros/ctl.sh)
rosnode kill /competition_udp_bridge >/dev/null 2>&1 || true
setsid roslaunch "$ROOT/autodrive/ros/final_sensor_udp.launch" > "$RUN_DIR/competition_udp.log" 2>&1 8>&- &
BRIDGE_PID=$!
sleep 1
kill -0 "$BRIDGE_PID" 2>/dev/null || { cat "$RUN_DIR/competition_udp.log" >&2; exit 1; }
if [[ "$NETWORK_PROFILE" == single_pc ]]; then
  # AUTO can resume before perception/following is ready. Stream the same brake
  # used in the verified 404_contest run, releasing only on the follower's command.
  setsid env AUTODRIVE_RUN_NONCE="$RUN" AUTODRIVE_ROLE=startup_brake \
    python3 "$ROOT/autodrive/ros/startup_brake.py" --run-id "$RUN" \
    --follower-manifest "$DATA/follower_run.json" \
    --ready-file "$RUN_DIR/startup_brake.json" > "$RUN_DIR/startup_brake.log" 2>&1 8>&- &
  START_BRAKE_PID=$!
  for _ in $(seq 1 50); do
    [[ -s "$RUN_DIR/startup_brake.json" ]] && break
    kill -0 "$START_BRAKE_PID" 2>/dev/null || { cat "$RUN_DIR/startup_brake.log" >&2; exit 1; }
    sleep .1
  done
  [[ -s "$RUN_DIR/startup_brake.json" ]] || { echo 'startup brake did not register' >&2; exit 1; }
  python3 "$ROOT/morai/install_final_sensor.py" > "$RUN_DIR/preset_install.log"
  python3 "$ROOT/autodrive/competition_network.py"
  python3 "$ROOT/autodrive/competition_profiles.py" request single_pc > "$RUN_DIR/network_request.json"
  docker exec alpha_semantic_b0_web mkdir -p /tmp/morai_competition_code/runtime
  docker cp "$SEMANTIC_CODE/runtime/." alpha_semantic_b0_web:/tmp/morai_competition_code/runtime
  docker cp "$RUN_DIR/network_profile.json" alpha_semantic_b0_web:/tmp/morai_competition_profile.json
  docker cp "$RUN_DIR/network_request.json" alpha_semantic_b0_web:/tmp/morai_competition_request.json
  # The simulator must already hold every requested UDP host port (preset loaded in
  # MORAI's Network Settings); its gRPC network getter/setter cannot be trusted.
  NETWORK_FLAG=()
  if python3 - "$RUN_DIR/network_request.json" <<'PY'
import json, subprocess, sys
request = json.load(open(sys.argv[1]))
wanted = {f"{v['NETWORK_ITEM_TYPE_HOST_IP']}:{v['NETWORK_ITEM_TYPE_HOST_PORT']}"
          for v in ({x["type"]: x["value"] for x in item["items"]} for item in request["protocol_settings"]["items"])}
owned = {line.split()[3] for line in subprocess.run(["ss", "-Hlunp"], capture_output=True, text=True).stdout.splitlines()
         if "Simulator.x86" in line}
missing = sorted(wanted - owned)
if missing:
    print("MORAI UDP ports not open: " + " ".join(missing), file=sys.stderr)
sys.exit(1 if missing else 0)
PY
  then NETWORK_FLAG=(--network-verified-on-host); fi
  docker exec -w /tmp/morai_competition_code -e PYTHONPATH=/tmp/morai_competition_code alpha_semantic_b0_web \
    python -m runtime.configure_competition --profile-file /tmp/morai_competition_profile.json \
    --request-file /tmp/morai_competition_request.json "${NETWORK_FLAG[@]}" > "$RUN_DIR/morai_profile_apply.log" 2>&1 \
    || { cat "$RUN_DIR/morai_profile_apply.log" >&2; exit 1; }
fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export STUDENT_HDMAP_LINKS="$ASSETS/link_set.json" STUDENT_SPEED_ZONES="$RUN_DIR/speed_zones.json"
# Competition: camera/LiDAR fusion outside the tunnel; wall-filtered LiDAR inside.
# MORAI's /Object_topic is neither subscribed to nor recorded.
OBJECT_TOPIC=/perception/objects
export STUDENT_OBJECT_GUARD=1 STUDENT_OBJECT_TOPIC=$OBJECT_TOPIC STUDENT_CLEARANCE=both
# Camera decisions and semantic stages share a 15 Hz target; sensor presets stay fixed.
export STUDENT_DECISION_HZ=15 SEMANTIC_FEED_HZ=15 SEMANTIC_FUSION_HZ=15
export STUDENT_HIGHWAY_MISSION="$ASSETS/highway_mission.json"
export STUDENT_TUNNEL_MAP="$ASSETS/tunnel_wall_map.json"
[[ -s "$STUDENT_TUNNEL_MAP" ]] || { echo 'fixed tunnel wall map missing' >&2; exit 1; }
# Bypass paths may use 2.0 m/s^2 lateral accel (gui default 1.7): ~9 % faster avoidance.
export STUDENT_BYPASS_ALAT="${STUDENT_BYPASS_ALAT:-3.0}"
# Until a bypass is validated, approach a static obstacle only as fast as a stop 4 m short allows.
export STUDENT_STRICT_STATIC_STOP="${STUDENT_STRICT_STATIC_STOP:-1}"
# Normal-road planner: lattice (candidate paths, collision-only stops) or legacy.
export STUDENT_PLANNER="${STUDENT_PLANNER:-model}"            # E2E: portable static planner + v6 side
export STUDENT_TUNNEL_POLICY="${STUDENT_TUNNEL_POLICY:-model}"  # E2E: v6 path + portable tunnel policy
# Pedestrian section 30 km/h from idx 770, not 800: crossings sit right after 800 and the car
# arrived at 39 km/h from the 40 km/h section, stopped 0.7 m short and was walked into (run 217).
export STUDENT_DYNAMIC_ZONE="${STUDENT_DYNAMIC_ZONE:-480-770:40,770-1100:30,3700-4030:40}"
# Wait before the lane of a >= 29 km/h vehicle that reaches our path within 4 s (merges/crossings).
export STUDENT_FAST_TRAFFIC_GUARD="${STUDENT_FAST_TRAFFIC_GUARD:-1}"
# Trial full stop only at intersection entry A1256W000709 (shared single/two-PC policy).
export STUDENT_STOP_AT_STOP_LINE="${STUDENT_STOP_AT_STOP_LINE:-1}"
# Drivable-area overrides edited on the dashboard (주행 가능 영역); missing file = HD-map defaults.
export STUDENT_DRIVABLE_RULES="${STUDENT_DRIVABLE_RULES:-$DATA/drivable_rules.json}"
export STUDENT_LIDAR_TOPIC=/lidar/points
# The same receiver performs inference and collection without sharing UDP sockets.
unset STUDENT_SESSION_DIR STUDENT_CAMERA_CACHE
if [[ -n "$SESSION_ID" ]]; then
  mkdir -p "$SESSION"
  export STUDENT_SESSION_DIR="$SESSION"
fi
export STUDENT_LOG_DIR="$RUN_DIR"
# One lap from the start pose, then a smooth stop there (gui.py writes $LAP_MARKER).
export STUDENT_ONE_LAP="$ONE_LAP" STUDENT_LAP_MARKER="$LAP_MARKER"
rm -f "$LAP_MARKER"

LIDAR_STATE="$("${CTL[@]}" lidar-status)"
if [[ "$LIDAR_STATE" == stopped ]]; then
  "${CTL[@]}" lidar-start
  LIDAR_STARTED=1
fi
LIDAR_READY=0
for _ in $(seq 1 15); do
  if timeout 2 docker exec morai_autodrive /opt/autodrive/ros/entrypoint.sh \
       rostopic echo -n 1 /lidar/points/header >/dev/null 2>&1; then LIDAR_READY=1; break; fi
  sleep 0.5
done
if [[ "$LIDAR_READY" != 1 ]]; then
  echo 'LiDAR scan missing on /lidar/points; check MORAI Lidar3D, UDP 2368 and lidar_stream.log' >&2
  tail -n 30 "$DATA/lidar_stream.log" >&2 2>/dev/null || true
  exit 1
fi

# Shared ROS infra (GPS/IMU receivers, blackout gate, ESKF) outlives this run. Start it in its own
# systemd scope: otherwise it sits in the dashboard service's cgroup and a dashboard restart kills it.
shared_infra() {
  if systemd-run --user --scope --quiet true >/dev/null 2>&1; then
    exec systemd-run --user --scope --quiet --collect setsid -w "$@"
  fi
  exec setsid -w "$@"
}
# GPS and IMU ports come from the selected final_sensor profile.
# GPS chain: receiver -> /gps_raw -> gps_blackout_gate.py -> /gps. Shared infra, left running.
for node_port in "morai_gps_udp:gps_port:$GPS_PORT" "morai_imu_udp:imu_port:$IMU_PORT"; do
  IFS=: read -r node param port <<<"$node_port"
  if [[ "$(rosparam get /$node/$param 2>/dev/null || true)" != "$port" \
     || "$(rosparam get /$node/source_ip 2>/dev/null || true)" != "$MORAI_SIM_IP" \
     || "$(rosparam get /$node/bind_ip 2>/dev/null || true)" != "$MORAI_LOCAL_IP" ]]; then
    rosnode kill "/$node" >/dev/null 2>&1 || true
  fi
done
# Use the same local receive-time clock as GPS and the competition gateway.
if [[ "$(rosparam get /morai_imu_udp/timestamp_source 2>/dev/null || true)" != arrival ]]; then
  rosnode kill /morai_imu_udp >/dev/null 2>&1 || true
fi
# A receiver left under another namespace can feed old sensor data into the
# selected profile. Remove competing publishers before starting/reusing ours.
for topic_owner in /gps_raw:/morai_gps_udp /imu/data:/morai_imu_udp; do
  IFS=: read -r topic expected <<<"$topic_owner"
  for node in $(rostopic info "$topic" 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/s/^ \* \(\/[^ ]*\) .*/\1/p'); do
    if [[ "$node" != "$expected" ]]; then
      echo "removing competing sensor publisher $node on $topic; selected $expected" >&2
      rosnode kill "$node" >/dev/null 2>&1 || true
    fi
  done
done
# Any other /gps publisher (e.g. an older receiver, possibly under another namespace) would
# bypass the blackout gate and feed ESKF directly, so remove it.
for node in $(rostopic info /gps 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/s/^ \* \(\/[^ ]*\) .*/\1/p'); do
  [[ "$node" == /gps_blackout_gate ]] || rosnode kill "$node" >/dev/null 2>&1 || true
done
for pid in $(pgrep -f 'lio_sam/morai_gps_ud[p].py'); do
  grep -qa '/gps:=/gps_raw' "/proc/$pid/cmdline" 2>/dev/null || kill "$pid" 2>/dev/null || true
done
sleep 1
NODES="$(rosnode list 2>/dev/null || true)"
if ! grep -qx /morai_gps_udp <<<"$NODES"; then
  shared_infra rosrun lio_sam morai_gps_udp.py __name:=morai_gps_udp _gps_port:="$GPS_PORT" \
    _bind_ip:="$MORAI_LOCAL_IP" _source_ip:="$MORAI_SIM_IP" /gps:=/gps_raw \
    >>"$DATA/gps_udp.log" 2>&1 8>&- &
fi
if ! grep -qx /gps_blackout_gate <<<"$NODES"; then
  shared_infra python3 "$ROOT/autodrive/gps_blackout_gate.py" --route "$ASSETS/global_path.json" \
    --start="${STUDENT_GPS_BLACKOUT_START:-26.58,-485.45}" --end="${STUDENT_GPS_BLACKOUT_END:--73.75,-552.79}" \
    >>"$DATA/gps_blackout_gate.log" 2>&1 8>&- &
fi
if ! grep -qx /morai_imu_udp <<<"$NODES"; then
  shared_infra rosrun lio_sam morai_imu_udp.py __name:=morai_imu_udp _imu_port:="$IMU_PORT" \
    _bind_ip:="$MORAI_LOCAL_IP" _source_ip:="$MORAI_SIM_IP" _timestamp_source:=arrival \
    >>"$DATA/imu_udp.log" 2>&1 8>&- &
fi
rosparam set /gps_blackout_gate/enabled "$([[ "${STUDENT_GPS_BLACKOUT:-1}" == 1 ]] && echo true || echo false)"
for topic in /gps_raw /imu/data; do
  ok=0
  for _ in $(seq 1 10); do
    if timeout 8 rostopic echo -n 1 "$topic" >/dev/null 2>&1; then ok=1; break; fi
    sleep 1
  done
  [[ "$ok" == 1 ]] || { echo "$topic missing: MORAI GPS/IMU UDP ports $GPS_PORT/$IMU_PORT (check sensor preset)" >&2; exit 1; }
done
for topic_owner in /gps_raw:/morai_gps_udp /imu/data:/morai_imu_udp; do
  IFS=: read -r topic expected <<<"$topic_owner"
  publishers="$(rostopic info "$topic" 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/s/^ \* \(\/[^ ]*\) .*/\1/p')"
  [[ "$publishers" == "$expected" ]] || {
    echo "$topic publishers must be exactly $expected, got: ${publishers:-none}; sensor profiles must not mix" >&2
    exit 1
  }
done
# ESKF must get GPS only through the gate; otherwise it silently dead-reckons from the start.
gps_pubs="$(rostopic info /gps 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/s/^ \* \(\/[^ ]*\) .*/\1/p')"
[[ "$gps_pubs" == /gps_blackout_gate ]] || {
  echo "/gps publishers must be exactly /gps_blackout_gate, got: ${gps_pubs:-none} (see $DATA/gps_blackout_gate.log)" >&2; exit 1; }

# IMU yaw is true-north, map positions are UTM grid: rotate by the grid convergence (measured
# -1.36 deg: /Ego_topic course-over-ground minus heading on straights) or GPS-free stretches drift.
IMU_YAW_OFFSET_DEG="${STUDENT_IMU_YAW_OFFSET_DEG:--1.35}"
# Competition ESKF tuning: trusts MORAI's ~0.05 m GPS (gps_position_variance 0.01, not 1.0).
ESKF_CONFIG="${STUDENT_ESKF_CONFIG:-$MPC_WS/src/eskf/config/eskf_competition.yaml}"
ESKF_GPS_VAR="$(python3 -c "import yaml,sys; print(float(yaml.safe_load(open(sys.argv[1]))['gps_position_variance']))" "$ESKF_CONFIG")"
# Any edit to the tuning file restarts localization: the running ESKF is tagged with the file's md5.
ESKF_CONFIG_MD5="$(md5sum "$ESKF_CONFIG" | cut -c1-32)"
if [[ "$GRID_HEADING" == 1 ]]; then EGO_HEADING_OFFSET_DEG=0.0
else EGO_HEADING_OFFSET_DEG="$(python3 -c "print(-float('$IMU_YAW_OFFSET_DEG'))")"; fi
ESKF_OK=0
if timeout 8 rostopic echo -n 1 /localization/ego_status >/dev/null 2>&1; then
  # Reuse a running ESKF only if it has these lever arms, wheel-angle sign and yaw offset.
  running_preset="$(rosparam get /eskf_node/sensor_config_file 2>/dev/null || true)"
  running_sign="$(rosparam get /ego_status_from_eskf/wheel_angle_sign 2>/dev/null || true)"
  running_yaw="$(rosparam get /eskf_node/imu_yaw_offset_deg 2>/dev/null || true)"
  running_rate="$(rosparam get /pure_odometry/motion_rate_scale 2>/dev/null || true)"
  running_heading="$(rosparam get /ego_status_from_eskf/heading_offset_deg 2>/dev/null || true)"
  running_gpsvar="$(rosparam get /eskf_node/gps_position_variance 2>/dev/null || true)"
  running_md5="$(rosparam get /competition/eskf_config_md5 2>/dev/null || true)"
  if [[ "$running_preset" == "$SENSOR_PRESET" && "$running_sign" == "-1.0" && "$running_yaw" == "$IMU_YAW_OFFSET_DEG" \
        && "$running_rate" == "1.0" && "$running_heading" == "$EGO_HEADING_OFFSET_DEG" \
        && "$running_gpsvar" == "$ESKF_GPS_VAR" && "$running_md5" == "$ESKF_CONFIG_MD5" ]]; then
    ESKF_OK=1
  else
    echo "restarting ESKF (previous preset='$running_preset' -> selected preset='$SENSOR_PRESET'; sign='$running_sign' yaw_offset='$running_yaw' rate='$running_rate' heading='$running_heading' gps_var='$running_gpsvar' config_md5='$running_md5')" >&2
    pkill -INT -f 'gps_odometry_eskf.launc[h]' || true
    for _ in $(seq 1 20); do
      rosnode list 2>/dev/null | grep -qx /eskf_node || break
      sleep 0.5
    done
    rosnode cleanup <<<y >/dev/null 2>&1 || true
  fi
fi
if [[ "$ESKF_OK" == 1 ]]; then
  :
else
  echo 'localization (/localization/ego_status) not found; starting gps_odometry_eskf.launch' >&2
  : > "$DATA/eskf_localization.log" 2>/dev/null || true
  shared_infra roslaunch morai_launch gps_odometry_eskf.launch sensor_config_file:="$SENSOR_PRESET" ego_wheel_angle_sign:=-1.0 \
    imu_yaw_offset_deg:="$IMU_YAW_OFFSET_DEG" odometry_motion_rate_scale:=1.0 ego_heading_offset_deg:="$EGO_HEADING_OFFSET_DEG" eskf_config:="$ESKF_CONFIG" \
    >>"$DATA/eskf_localization.log" 2>&1 8>&- &
  ESKF_PID=$!
  rosparam set /competition/eskf_config_md5 "$ESKF_CONFIG_MD5"
  ESKF_READY=0
  for _ in $(seq 1 20); do
    kill -0 "$ESKF_PID" 2>/dev/null || { echo 'localization exited before becoming ready' >&2; tail -n 40 "$DATA/eskf_localization.log" >&2; exit 1; }
    if timeout 8 rostopic echo -n 1 /localization/ego_status >/dev/null 2>&1; then ESKF_READY=1; break; fi
    sleep 1
  done
  [[ "$ESKF_READY" == 1 ]] || { echo '/localization/ego_status not available after 20 s; check /gps and /imu/data on this roscore' >&2; tail -n 40 "$DATA/eskf_localization.log" >&2; exit 1; }
fi
# Do not drive on a filter that is still settling: GPS-tracking mode and a sane MORAI time scale
# (a 0.5 scale made the estimate run at half speed and fall 15 m behind right after a restart).
ESKF_SETTLED=0
for attempt in $(seq 1 40); do
  if timeout 3 rostopic echo -n 1 --noarr /eskf/diagnostics 2>/dev/null | python3 -c '
import sys, json, yaml
d = json.loads(yaml.safe_load(sys.stdin.read().split("---")[0])["data"])
scale = float(d.get("odometry_time_scale", 0))
sys.exit(0 if d.get("mode") == "tracking" and 0.3 <= scale <= 1.5 else 1)'; then ESKF_SETTLED=1; break; fi
  if [[ "$attempt" == 8 ]]; then
    # Typically the car was reset/teleported in MORAI: the filter rejects every GPS fix as an
    # outlier and cannot recover while standing still. A fresh start re-initializes from GPS.
    echo 'ESKF not tracking GPS after 8 s (vehicle reset?); restarting localization' >&2
    pkill -INT -f 'gps_odometry_eskf.launc[h]' || true
    for _ in $(seq 1 20); do rosnode list 2>/dev/null | grep -qx /eskf_node || break; sleep 0.5; done
    rosnode cleanup <<<y >/dev/null 2>&1 || true
    shared_infra roslaunch morai_launch gps_odometry_eskf.launch sensor_config_file:="$SENSOR_PRESET" ego_wheel_angle_sign:=-1.0 \
      imu_yaw_offset_deg:="$IMU_YAW_OFFSET_DEG" odometry_motion_rate_scale:=1.0 ego_heading_offset_deg:="$EGO_HEADING_OFFSET_DEG" eskf_config:="$ESKF_CONFIG" \
      >>"$DATA/eskf_localization.log" 2>&1 8>&- &
    ESKF_PID=$!
    rosparam set /competition/eskf_config_md5 "$ESKF_CONFIG_MD5"
  fi
  sleep 1
done
[[ "$ESKF_SETTLED" == 1 ]] || { echo 'ESKF not settled (mode!=tracking or time scale off) after 40 s' >&2; exit 1; }

# Sensor/pipeline health for the dashboard (주행 가능 영역 page). Shared infra, left running.
if ! rosnode list 2>/dev/null | grep -qx /sensor_health; then
  shared_infra python3 "$ROOT/autodrive/sensor_health.py" >>"$DATA/sensor_health.log" 2>&1 8>&- &
fi
# RViz (competition view): HD map + drivable rules, Alpamayo path, corridor, objects, ego.
# Off by default (the dashboard 주행 가능 영역 page shows the same live view); STUDENT_RVIZ=1 opens it.
if [[ "${STUDENT_RVIZ:-0}" == 1 ]]; then
  NODES="$(rosnode list 2>/dev/null || true)"
  if ! grep -qx /competition_viz <<<"$NODES"; then
    shared_infra python3 "$ROOT/autodrive/competition_viz.py" --rules "$STUDENT_DRIVABLE_RULES" \
      >>"$DATA/competition_viz.log" 2>&1 8>&- &
  fi
  if ! pgrep -f "rviz/competitio[n].rviz" >/dev/null; then
    export DISPLAY="${DISPLAY:-:0}"
    if [[ -z "${XAUTHORITY:-}" && -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
      export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
    fi
    shared_infra rviz -d "$ROOT/autodrive/rviz/competition.rviz" __name:=competition_rviz \
      >>"$DATA/competition_rviz.log" 2>&1 8>&- &
  fi
fi
python3 "$ROOT/autodrive/localization_monitor.py" --route "$ASSETS/global_path.json" \
  --status "$STATUS" --trace "$RUN_DIR/localization_trace.csv" >"$RUN_DIR/localization_monitor.log" 2>&1 8>&- &
MON_PID=$!
# Semantic camera x LiDAR fusion -> /perception/objects (needs the alpamayo_semantic sidecar container).
start_semantic_fusion() {
  local fusion_script="$ASSETS/semantic_fusion.py"
  [[ -s "$fusion_script" ]] || { echo "semantic fusion entrypoint missing: $fusion_script" >&2; return 2; }
  printf '[perception] semantic fusion entrypoint: %s\n' "$fusion_script"
  exec "$PYTHON" "$fusion_script" --sensor "$SENSOR_PRESET" --route "$ASSETS/global_path.json"
}
start_semantic_fusion >"$RUN_DIR/semantic_fusion.log" 2>&1 8>&- &
FUSION_PID=$!
for _ in $(seq 1 20); do
  # The monitor's first record can have zero samples before its ROS callback.
  # Wait for an actual publication rather than accepting that startup record.
  hz="$(status_field ego_hz)"
  if awk -v hz="$hz" 'BEGIN { exit !(hz+0 > 0) }'; then break; fi
  kill -0 "$MON_PID" 2>/dev/null || { tail -n 20 "$RUN_DIR/localization_monitor.log" >&2; exit 1; }
  sleep 0.5
done
python3 - "$(status_field ego_hz)" "$(status_field path.distance_m)" <<'PY'
import sys
hz,dist=sys.argv[1:]
if float(hz or 0)<=0: raise SystemExit('/localization/ego_status not publishing')
if dist=='' : raise SystemExit('no localization fix near the global route')
if float(dist)>10: raise SystemExit(f'localization is {float(dist):.1f} m from the global route; place the vehicle on the route')
PY

# GUI and detector share surveyed stop/junction bounds before control starts.
export STUDENT_INTERSECTION_PERCEPTION="$RUN_DIR/intersection_perception.json"
"$PYTHON" "$ASSETS/intersection_perception.py" --links "$STUDENT_HDMAP_LINKS" \
  --route "$ASSETS/global_path.json" --output "$STUDENT_INTERSECTION_PERCEPTION" \
  >"$RUN_DIR/intersection_perception.log" 2>&1

# Strict Car-model startup must succeed before control can start.
VEHICLE_PYTHON="${STUDENT_VEHICLE_PYTHON:-$ROOT/.venv-openpcdet/bin/python}"
[[ -x "$VEHICLE_PYTHON" ]] || { echo 'PointPillars Python runtime missing' >&2; exit 1; }
"$VEHICLE_PYTHON" "$ROOT/autodrive/lidar_vehicle_detection.py" --mission "$STUDENT_HIGHWAY_MISSION" \
  --model-dir "${STUDENT_VEHICLE_MODEL_DIR:-$ROOT/trained_models/morai_pointpillar_20k_2026-08-14}" \
  --route "$ASSETS/global_path.json" --sensor "$SENSOR_PRESET" --hz 15 \
  --intersection-regions "$STUDENT_INTERSECTION_PERCEPTION" \
  --ready-file "$RUN_DIR/lidar_vehicle_ready.json" >"$RUN_DIR/lidar_vehicles.log" 2>&1 8>&- &
VEHICLE_PID=$!
for _ in $(seq 1 60); do
  kill -0 "$VEHICLE_PID" 2>/dev/null || { tail -n 30 "$RUN_DIR/lidar_vehicles.log" >&2; exit 1; }
  [[ -s "$RUN_DIR/lidar_vehicle_ready.json" ]] && break
  sleep 0.5
done
[[ -s "$RUN_DIR/lidar_vehicle_ready.json" ]] || { echo 'PointPillars model did not become ready' >&2; exit 1; }
cp "$STUDENT_HIGHWAY_MISSION" "$RUN_DIR/highway_mission.json"
cp "$STUDENT_TUNNEL_MAP" "$RUN_DIR/tunnel_wall_map.json"
python3 - "$RUN_DIR/run.json" "$STUDENT_HIGHWAY_MISSION" "$STUDENT_INTERSECTION_PERCEPTION" <<'META'
import json,sys
p=sys.argv[1];meta=json.load(open(p));meta['highway_mission']=json.load(open(sys.argv[2]))
meta['rates_hz'].update(lidar=15,lidar_vehicle_detection=15)
meta['obstacles']='semantic_fusion+lidar+highway_intersection_pointpillars_cars'
meta['intersection_perception']=json.load(open(sys.argv[3]))
meta['tunnel']='fixed_wall_map+lidar_geometry+wheel_imu_relative_motion+checked_student_path'
meta['tunnel_no_go']={'topic':'/perception/no_go_areas','scope':'tunnel_only',
                      'model':'existing_student_checkpoint','purpose':'candidate_path_clearance'}
json.dump(meta,open(p,'w'),indent=2)
META

# Raw rosbag capture is opt-in; ordinary driving keeps lightweight diagnostics.
if [[ "${STUDENT_RECORD_ROSBAG:-0}" == 1 ]]; then
BAG_TOPICS=(/Competition_topic /localization/ego_status /eskf/odom /eskf/diagnostics /gps_raw /gps_blackout/active
  /odometry/pure /odometry/filtered /gps /imu/data /ctrl_cmd /competition/gear_cmd
  /avoid_waypoints /avoid_target_vel /lane_change_mode /student/path_status /student/processed_path /localization/lane_constraint
  /CollisionData /mpc_status /mpc_performance /planning/global_path /tf /tf_static /rosout)
BAG_TOPICS+=(/perception/objects /perception/lidar_vehicles /perception/no_go_areas /lidar/points /competition_udp/status /competition_udp/collision_stop)
setsid rosbag record --lz4 --split --size=2048 -O "$RUN_DIR/drive.bag" "${BAG_TOPICS[@]}" \
  >"$RUN_DIR/rosbag.log" 2>&1 8>&- &
BAG_PID=$!
sleep 1
kill -0 "$BAG_PID" 2>/dev/null || { echo 'rosbag record failed to start' >&2; tail -n 20 "$RUN_DIR/rosbag.log" >&2; exit 1; }
fi
cp "$ROOT/autodrive/ros/follower_student_competition.launch" "$RUN_DIR/" 2>/dev/null || true
cp "$SENSOR_PRESET" "$RUN_DIR/sensor_preset.json" 2>/dev/null || true
md5sum "$CHECKPOINT" "$ASSETS/gui.py" "$ASSETS/semantic_fusion.py" "$ASSETS/lidar_corridor.py" "$ASSETS"/tunnel_*.py "$STUDENT_TUNNEL_MAP" "$ASSETS"/signal_control/*.py > "$RUN_DIR/versions.md5" 2>/dev/null || true

if [[ -n "$SESSION_ID" ]]; then
  # Recorders end on the stop file; this is only their safety bound (no drive time limit).
  RECORD_DURATION=$((4 * 3600))
  docker exec -i morai_autodrive /opt/autodrive/ros/entrypoint.sh python3 -u - \
    --duration-seconds "$RECORD_DURATION" --stop-file "/data/raw/$SESSION_ID/.student_stop" \
    --ego-topic /localization/ego_status < "$ROOT/collect/record_ego.py" \
    > "$SESSION/ego_raw.jsonl" 2> "$SESSION/ego_recorder.log" 8>&- &
  EGO_RECORD_PID=$!
  docker exec -i morai_autodrive /opt/autodrive/ros/entrypoint.sh python3 -u - \
    --duration-seconds "$RECORD_DURATION" --stop-file "/data/raw/$SESSION_ID/.student_stop" \
    --object-topic /perception/objects < "$ROOT/collect/record_objects.py" \
    > "$SESSION/objects.jsonl" 2> "$SESSION/objects_recorder.log" 8>&- &
  OBJECT_RECORD_PID=$!
  sleep 1
fi

"$PYTHON" "$ASSETS/gui.py" --checkpoint "$CHECKPOINT" \
  --route "$ASSETS/global_path.json" --status "$STATUS" --ego-topic "$EGO_TOPIC" \
  --camera-source udp --publish-control-path \
  --force-student-avoidance --cruise-kmh "$KMH" --headless \
  >"$RUN_DIR/gui.log" 2>&1 8>&- &
GUI_PID=$!
SEMANTIC_READY=0
for _ in $(seq 1 60); do
  kill -0 "$GUI_PID" 2>/dev/null || { tail -n 20 "$RUN_DIR/gui.log" >&2; exit 1; }
  # A path_status can report STOP before camera masks reach the fusion node.
  # Require a fresh mask and object message before handing control to the follower.
  if [[ -f "$SEMANTIC_FEED_DIR/masks.npz" ]] \
     && (( $(date +%s) - $(stat -c %Y "$SEMANTIC_FEED_DIR/masks.npz") <= 2 )) \
     && timeout 2 docker exec morai_autodrive /opt/autodrive/ros/entrypoint.sh \
          rostopic echo -n 1 /perception/objects >/dev/null 2>&1 \
     && timeout 2 docker exec morai_autodrive /opt/autodrive/ros/entrypoint.sh \
          rostopic echo -n 1 /student/path_status >/dev/null 2>&1; then
    SEMANTIC_READY=1
    break
  fi
  sleep 0.3
done
kill -0 "$GUI_PID" 2>/dev/null || { echo 'Student inference stopped' >&2; exit 1; }
[[ "$SEMANTIC_READY" == 1 ]] || {
  echo "semantic masks, objects, or student path not ready; check $SEMANTIC_FEED_DIR permissions and $RUN_DIR/gui.log" >&2
  exit 1
}
if ! timeout 3 docker exec morai_autodrive /opt/autodrive/ros/entrypoint.sh \
     rostopic echo -n 1 /student/path_status >/dev/null 2>&1; then
  echo 'Student did not publish a valid path; check student GUI log, localization and LiDAR (outside tunnel: cameras UDP 9202/9203/9204)' >&2; exit 1
fi
# Require the selected UDP gateway before starting the pedal-command follower.
[[ "$(rostopic info /Competition_topic 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/s/^ \* \(\/[^ ]*\) .*/\1/p')" == /competition_udp_bridge ]] \
  || { echo '/Competition_topic must be published only by the selected UDP gateway' >&2; exit 1; }
FOLLOWER_REQUESTED=1
if [[ -n "$START_BRAKE_PID" ]]; then
  kill -0 "$START_BRAKE_PID" 2>/dev/null || { echo 'startup brake ended before follower start' >&2; cat "$RUN_DIR/startup_brake.log" >&2; exit 1; }
fi
docker exec -e STUDENT_MPC_LOG_FILE="$MPC_LOG_FILE" -e STUDENT_CURVE_ALAT="$LADDER_ALAT" morai_autodrive \
  /opt/autodrive/ros/ctl.sh follow-start "$KMH" student-competition --run "$RUN"
if [[ -n "$START_BRAKE_PID" ]]; then
  HANDOFF=0
  for _ in $(seq 1 50); do
    if python3 - "$RUN_DIR/startup_brake.json" <<'PY'
import json,sys
raise SystemExit(0 if json.load(open(sys.argv[1]))['state']=='handoff' else 1)
PY
    then HANDOFF=1; break; fi
    kill -0 "$START_BRAKE_PID" 2>/dev/null || break
    sleep .1
  done
  [[ "$HANDOFF" == 1 ]] || { echo 'follower did not take over startup brake' >&2; cat "$RUN_DIR/startup_brake.log" >&2; exit 1; }
fi

MOVED=0
for _ in $(seq 1 60); do
  kill -0 "$GUI_PID" 2>/dev/null || { echo 'Student inference exited' >&2; exit 1; }
  [[ "$(status_field ego.speed_kmh)" != '' ]] || { echo 'localization lost' >&2; exit 1; }
  if python3 - "$(status_field ego.speed_kmh)" <<'PY'
import sys
raise SystemExit(0 if float(sys.argv[1])>3 else 1)
PY
  then MOVED=1; break; fi
  sleep 1
done
# Judge by whether movement was ever seen: a car still coasting at launch passed the loop at
# >3 km/h and was then failed here once it slowed below 3 (follow-20260930T164757Z).
python3 - "$MOVED" <<'PY'
import sys
if sys.argv[1] != "1": raise SystemExit('vehicle did not move within 60 s')
PY

HIGHWAY_SUMMARY="$(python3 - "$RUN_DIR/highway_mission.json" "$KMH" <<'PY'
import json,sys
mission=json.load(open(sys.argv[1]))
limit=float(mission['kmh'])
if not mission.get('use_mission_speed',False):
    limit=min(limit,float(sys.argv[2]))
print(f"highway<={limit:g}km/h policy={mission.get('lane_change_policy','legacy')}")
PY
)"
if [[ "$ONE_LAP" == 1 ]]; then RUN_LENGTH='one lap from the start point, then stop (no time limit)'
else RUN_LENGTH='until stopped from the dashboard (no time limit)'; fi
echo "Student v5 $NETWORK_PROFILE decisions=15Hz semantic=15Hz driving $RUN_LENGTH (ego $EGO_TOPIC, objects $OBJECT_TOPIC+LiDAR+PointPillars, $HIGHWAY_SUMMARY, session=${SESSION_ID:-none}, speed level $LADDER_LEVEL = $LADDER_KMH km/h)"
while true; do
  if [[ "$ONE_LAP" == 1 && -s "$LAP_MARKER" ]]; then
    python3 - "$LAP_MARKER" <<'PY' || echo '[LAP] one lap complete; stopping'
import json,sys
d=json.load(open(sys.argv[1]))
def fmt(v): return '?' if v is None else f'{v:.1f}'
stop=d.get('stop') or {}
print(f"[LAP] one lap complete: {fmt(d.get('progress_m'))}/{fmt(d.get('route_length_m'))} m, "
      f"lap time {fmt(d.get('lap_time_s'))} s wall (~{fmt(d.get('lap_time_sim_s'))} s sim), "
      f"stopped {fmt(stop.get('distance_from_start_m'))} m from the start point"
      + (' (overshoot stop)' if d.get('overshoot') else '') + "; ending the run")
PY
    break
  fi
  kill -0 "$GUI_PID" 2>/dev/null || { echo 'Student inference stopped early' >&2; exit 1; }
  kill -0 "$VEHICLE_PID" 2>/dev/null || { echo 'LiDAR vehicle detection stopped' >&2; exit 1; }
  kill -0 "$FUSION_PID" 2>/dev/null || { echo 'semantic fusion stopped' >&2; exit 1; }
  kill -0 "$MON_PID" 2>/dev/null || { echo 'localization monitor stopped' >&2; exit 1; }
  kill -0 "$BRIDGE_PID" 2>/dev/null || { echo 'competition UDP gateway stopped' >&2; exit 1; }
  if [[ -n "$SESSION_ID" ]]; then
    kill -0 "$EGO_RECORD_PID" 2>/dev/null && kill -0 "$OBJECT_RECORD_PID" 2>/dev/null \
      || { echo 'sensor session recorder stopped' >&2; exit 1; }
  fi
  timeout 3 docker exec morai_autodrive /opt/autodrive/ros/entrypoint.sh \
    python3 /opt/autodrive/ros/owned_run.py alive --manifest /data/autodrive/follower_run.json \
    || { echo 'Student MPC follower stopped' >&2; exit 1; }
  python3 - "$(status_field ego_hz)" <<'PY'
import sys
if float(sys.argv[1] or 0)<=0: raise SystemExit('localization lost')
PY
  sleep 1
done
