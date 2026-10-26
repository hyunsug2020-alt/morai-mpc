#!/usr/bin/env bash
# Follower and diagnostics-recorder control, run inside the container.
#   ctl.sh follow-start [cruise_kmh] [follow|avoid|avoid-bspline-test] [--run NONCE]
#   ctl.sh follow-stop [--run NONCE]     --run: automatic cleanup, acts only on that run's follower;
#                                        no --run: explicit manual stop of the current owned follower
#   ctl.sh follow-stop-legacy            explicit migration: stop a follower started by the pre-ownership ctl.sh
#   ctl.sh follow-status
#   ctl.sh diag-start --run NONCE --session ID --capture-dir DIR [--meta FILE] [--ready-timeout S]
#   ctl.sh diag-stop --run NONCE [--timeout S]
# Ownership: every owned process carries AUTODRIVE_RUN_NONCE/ROLE (see owned_run.py); nothing is
# stopped by process or node name. A running follower is a start conflict, never adopted.
set -euo pipefail
SELF_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source /opt/autodrive/ws/devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11312}"
DATA="${AUTODRIVE_DATA:-/data/autodrive}"
LOCKS="${AUTODRIVE_LOCK_DIR:-/tmp}"
PID="$DATA/follower.pid"
LOG="$DATA/follower.log"
FMAN="$DATA/follower_run.json"
RMAN="$DATA/recorder_run.json"
DRIVE_LOCK="$DATA/drive.lock"
DRIVE_MAN="$DATA/drive_run.json"
OWN=(python3 "${AUTODRIVE_OWNED_RUN:-$SELF_DIR/owned_run.py}")
RECORDER_CMD="${AUTODRIVE_RECORDER_CMD:-python3 $SELF_DIR/record_drive_diagnostics.py}"
LATEST_CSV="${AUTODRIVE_LATEST_CSV:-$DATA/drive_diagnostics.csv}"
LIDAR_MAN="$DATA/lidar_stream_run.json"
LIDAR_LOG="$DATA/lidar_stream.log"
LIDAR_EXIT="$DATA/lidar_stream_exit.json"

json_get() { python3 -c 'import json,sys
try: d=json.load(open(sys.argv[1]))
except Exception: sys.exit(1)
v=d
for k in sys.argv[2].split("."): v=v.get(k) if isinstance(v,dict) else None
print("" if v is None else v)' "$@"; }

legacy_running() { [[ ! -f "$FMAN" && -f "$PID" ]] && kill -0 "$(cat "$PID")" 2>/dev/null; }

# The drive owner (auto_collect.sh) holds $DRIVE_LOCK for the whole drive and names its nonce in $DRIVE_MAN.
drive_owner_ok() {
  [[ "$(json_get "$DRIVE_MAN" nonce 2>/dev/null || true)" == "$1" ]] || return 1
  ! flock -n "$DRIVE_LOCK" true 2>/dev/null
}

# MORAI keeps the last longlCmdType 2 command: stream a stop for 3 s on the given master.
# Prints one JSON line. Success (rc 0) means only: master reachable, this publisher registered for /ctrl_cmd at
# the master, and 30 publish() calls at 10 Hz completed locally on the WALL clock (a paused sim clock cannot
# stall it). It does not prove that the simulator received or applied the command.
stream_stop() {
  local out rc
  out="$(ROS_MASTER_URI="$1" timeout --kill-after=2 "${AUTODRIVE_STOPSTREAM_TIMEOUT:-15}" python3 - 2>/dev/null <<'PY'
import http.client, json, os, sys, time, xmlrpc.client
res = {"master": os.environ.get("ROS_MASTER_URI"), "registered": False, "attempted": 0, "published": 0,
       "subscriber_connections_max": 0, "ok": False, "error": None, "start_wall_ns": time.time_ns()}
class TT(xmlrpc.client.Transport):
    def __init__(self, t): super().__init__(); self.t = t
    def make_connection(self, host):
        c = super().make_connection(host); c.timeout = self.t; return c
rpc = lambda t: xmlrpc.client.ServerProxy(res["master"], transport=TT(t))
fault = os.environ.get("AUTODRIVE_STOPSTREAM_TEST_FAULT", "")   # test seam, inert unless set
try:
    rpc(2.0).getPid("/autodrive_stop_check")
    import rospy
    from morai_msgs.msg import CtrlCmd
    rospy.init_node("autodrive_stop", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)
    me, t_end = rospy.get_name(), time.monotonic() + 3.0
    while time.monotonic() < t_end and not res["registered"]:
        pubs = rpc(1.0).getSystemState(me)[2][0]
        res["registered"] = any(t == "/ctrl_cmd" and me in nodes for t, nodes in pubs)
        time.sleep(0.05)
    if not res["registered"]:
        raise RuntimeError("stop publisher not registered at master")
    msg = CtrlCmd(longlCmdType=2, accel=0.0, brake=1.0, steering=0.0, velocity=0.0, acceleration=0.0)
    t0 = time.monotonic()
    for i in range(30):
        time.sleep(max(0.0, t0 + 0.1 * i - time.monotonic()))
        res["attempted"] += 1
        if fault == "raise" and i == 5:
            raise RuntimeError("injected publish failure")
        if fault == "hang" and i == 5:
            time.sleep(3600)
        pub.publish(msg)
        res["published"] += 1
        res["subscriber_connections_max"] = max(res["subscriber_connections_max"], pub.get_num_connections())
    res["ok"] = res["published"] == 30
except Exception as e:
    res["error"] = repr(e)[:300]
res["end_wall_ns"] = time.time_ns()
print(json.dumps(res, sort_keys=True))
sys.exit(0 if res["ok"] else 1)
PY
)"; rc=$?
  [[ -n "$out" ]] || out="{\"ok\": false, \"master\": \"$1\", \"error\": \"stop stream process ended without a result (rc $rc; timeout/kill)\"}"
  echo "$out"
  return $(( rc == 0 ? 0 : 1 ))
}

STATE_DIR="${AUTODRIVE_STOPSTREAM_STATE_DIR:-$DATA/stopstream_state}"
# stopstate <nonce> <master> <op> [result json]: durable pending/done stop-stream state per (run, master)
stopstate() {
  if [[ -n "${4:-}" ]]; then "${OWN[@]}" stopstate --dir "$STATE_DIR" --nonce "$1" --master "$2" --op "$3" --result "$4"
  else "${OWN[@]}" stopstate --dir "$STATE_DIR" --nonce "$1" --master "$2" --op "$3"; fi
}
state_of() { python3 -c 'import json,sys; d=json.loads(sys.argv[1]); print(d.get("state") or ("error" if d.get("error") else ""))' "$1" 2>/dev/null || echo error; }

# other_active_run <nonce> <master>: JSON of a DIFFERENT run's live follower on the same master (else empty).
# A stale run's stop stream on the master-wide /ctrl_cmd topic must not brake a newer active run.
other_active_run() {
  local procs; procs="$("${OWN[@]}" procs --role follower --master "$2")"
  python3 - "$1" "$2" "$procs" "$FMAN" <<'PY'
import json, sys
nonce, master, procs, fman = sys.argv[1], sys.argv[2], json.loads(sys.argv[3] or "[]"), sys.argv[4]
other = sorted({p["nonce"] for p in procs if p.get("nonce") != nonce})
if other:
    try:
        m = json.load(open(fman))
    except Exception:
        m = {}
    print(json.dumps({"other_runs": other, "master": master, "current_manifest_nonce": m.get("nonce")}))
PY
}

# stream_for_run <nonce> <master>: publish only while the run's state is pending and no newer run is active.
# Sets STREAM (json|null), STRC (0 ok/not needed, 5 failed, 6 blocked, 7 state I/O error), SSTATE (summary word).
stream_for_run() {
  local cur st conflict rec
  cur="$(stopstate "$1" "$2" get)" || true; st="$(state_of "$cur")"
  STREAM='null'; STRC=0
  case "$st" in
    done) SSTATE=done_before; return 0 ;;
    absent) SSTATE=not_needed; return 0 ;;
    pending) ;;
    *) SSTATE=state_error; STREAM="{\"ok\": false, \"state_error\": $cur}"; STRC=7; return 0 ;;
  esac
  conflict="$(other_active_run "$1" "$2")"
  if [[ -n "$conflict" ]]; then
    STREAM="{\"ok\": false, \"blocked_by\": $conflict, \"end_wall_ns\": $(date +%s%N)}"; STRC=6; SSTATE=blocked
    stopstate "$1" "$2" record "$STREAM" >/dev/null || { STRC=7; SSTATE=state_error; }
    return 0
  fi
  set +e; STREAM="$(stream_stop "$2")"; STRC=$?; set -e
  [[ $STRC -eq 0 ]] && SSTATE=ok || { STRC=5; SSTATE=failed; }
  if ! rec="$(stopstate "$1" "$2" record "$STREAM")"; then STRC=7; SSTATE=state_error; STREAM="{\"ok\": false, \"published\": $STREAM, \"state_error\": $rec}"; fi
  return 0
}

record_stream() {   # record_stream <manifest> <stream json>: append the attempt; success marker only on success
  python3 - "$1" "$2" <<'PY'
import json, os, sys, time
p, r = sys.argv[1], json.loads(sys.argv[2])
m = json.load(open(p))
m.setdefault("stopstream_attempts", []).append(r)
if r.get("ok") is True:
    m["stopstream_wall_ns"] = r.get("end_wall_ns") or time.time_ns()
t = p + ".tmp"; json.dump(m, open(t, "w"), indent=1, sort_keys=True); os.replace(t, p)
PY
}

parse_run() { RUN=""; ARGS=(); while (($#)); do case "$1" in --run) RUN="$2"; shift 2 ;; *) ARGS+=("$1"); shift ;; esac; done; }

case "${1:-follow-status}" in
  follow-start)
    shift; parse_run "$@"; set -- "${ARGS[@]+"${ARGS[@]}"}"
    # Serialize starts; stop can cancel a pending build through the request file.
    exec 9>"$LOCKS/autodrive-follow-start.lock"
    if ! flock -n 9; then echo "follower start already in progress" >&2; exit 1; fi
    if [[ -f "$FMAN" ]] && "${OWN[@]}" alive --manifest "$FMAN"; then
      echo "follower already running (run $(json_get "$FMAN" nonce)); not adopting" >&2; exit 3
    fi
    if legacy_running; then echo "legacy follower running (pid $(cat "$PID")); use follow-stop-legacy first" >&2; exit 3; fi
    if [[ "$("${OWN[@]}" procs --role follower --master "$ROS_MASTER_URI")" != "[]" ]]; then
      echo "owned follower processes still alive on $ROS_MASTER_URI; not starting" >&2; exit 3
    fi
    CRUISE="${1:-30}"
    MODE="${2:-follow}"   # follow | avoid | avoid-bspline-test
    if [[ -n "$RUN" ]]; then
      drive_owner_ok "$RUN" || { echo "run $RUN does not own the active drive" >&2; exit 3; }
    else
      exec 6>"$DRIVE_LOCK"
      flock -n 6 || { echo "an owned drive is in progress; manual start refused" >&2; exit 3; }
      RUN="manual-$(date -u +%Y%m%dT%H%M%SZ)-$$"
    fi
    REQUEST="$LOCKS/autodrive-follow-request"
    echo "$$ $RUN" > "$REQUEST"
    trap 'rm -f "$REQUEST"' EXIT
    if [[ -n "${AUTODRIVE_BUILD_CMD:-}" ]]; then bash -c "$AUTODRIVE_BUILD_CMD"; else bash "$SELF_DIR/build_workspace.sh"; fi
    # A short state lock makes the final launch atomic with respect to stop.
    exec 7>"$LOCKS/autodrive-follow-state.lock"
    flock 7
    if [[ "$(cat "$REQUEST" 2>/dev/null || true)" != "$$ $RUN" ]]; then
      echo "follower start cancelled" >&2; exit 1
    fi
    : > "$LOG"
    if [[ -n "${AUTODRIVE_FOLLOWER_CMD:-}" ]]; then
      LAUNCH=(bash -c "$AUTODRIVE_FOLLOWER_CMD")
    elif [[ "$MODE" == "avoid-bspline-test" ]]; then
      LAUNCH=(roslaunch "$SELF_DIR/follower_avoid_bspline_test.launch" cruise_kmh:="$CRUISE")
    elif [[ "$MODE" == "avoid" ]]; then
      LAUNCH=(roslaunch "$SELF_DIR/follower_avoid.launch" cruise_kmh:="$CRUISE")
    elif [[ "$MODE" == "student" ]]; then
      LAUNCH=(roslaunch "$SELF_DIR/follower_student.launch" cruise_kmh:="$CRUISE")
    elif [[ "$MODE" == "student-competition" ]]; then
      LAUNCH=(roslaunch "$SELF_DIR/follower_student_competition.launch" cruise_kmh:="$CRUISE")
    else
      # competition route is a closed loop (first point == last point): keep driving past the end by default
      LAUNCH=(roslaunch "$SELF_DIR/follower.launch" cruise_kmh:="$CRUISE" stop_at_path_end:="${AUTODRIVE_STOP_AT_END:-false}")
    fi
    rm -f "$FMAN"
    if ! SPAWN="$("${OWN[@]}" spawn --role follower --nonce "$RUN" --manifest "$FMAN" --log "$LOG" \
        --exit-file "$DATA/follower_exit.json" -- "${LAUNCH[@]}" 6>&- 7>&- 9>&-)"; then
      echo "follower spawn failed and was rolled back: $SPAWN" >&2
      exit 1
    fi
    json_get "$FMAN" supervisor_pid > "$PID"
    echo "$MODE" > "$DATA/follower.mode"
    echo "follower started pid $(cat "$PID") mode=$MODE cruise_kmh=$CRUISE run=$RUN"
    ;;
  follow-stop)
    shift; parse_run "$@"
    exec 7>"$LOCKS/autodrive-follow-state.lock"
    flock 7
    REQUEST="$LOCKS/autodrive-follow-request"
    if [[ -z "$RUN" ]] || [[ "$(cut -d' ' -f2 "$REQUEST" 2>/dev/null || true)" == "$RUN" ]]; then rm -f "$REQUEST"; fi
    MASTER_NOW="$ROS_MASTER_URI"
    if [[ -f "$FMAN" ]]; then MNONCE="$(json_get "$FMAN" nonce)"; else MNONCE=""; fi
    if [[ -n "$RUN" && "$MNONCE" != "$RUN" ]]; then
      # No manifest for this run (never written, lost, or replaced by a later run whose manifest is never touched):
      # record "pending" BEFORE stopping (while process evidence exists), then stop exactly this run on this master.
      STATE_ERR=""
      if [[ "$("${OWN[@]}" procs --nonce "$RUN" --role follower --master "$MASTER_NOW")" != "[]" ]]; then
        INIT="$(stopstate "$RUN" "$MASTER_NOW" init)" || STATE_ERR="$INIT"
      fi
      set +e
      RES="$("${OWN[@]}" stop-run --nonce "$RUN" --role follower --master "$MASTER_NOW" --grace "${AUTODRIVE_FOLLOW_STOP_GRACE:-20}")"; SRC=$?
      set -e
      if [[ -n "$STATE_ERR" ]]; then
        # pending state could not be persisted: the processes were still stopped; stream once, never report success
        set +e; STREAM="$(stream_stop "$MASTER_NOW")"; set -e
        STREAM="{\"ok\": false, \"published\": $STREAM, \"state_error\": $STATE_ERR}"; STRC=7; SSTATE=state_error
      else
        stream_for_run "$RUN" "$MASTER_NOW"
      fi
      echo "{\"mode\": \"no_manifest_for_run\", \"stop\": $RES, \"stopstream\": $STREAM, \"stopstream_state\": \"$SSTATE\", \"ok\": $([[ $SRC -eq 0 && $STRC -eq 0 ]] && echo true || echo false)}"
      if [[ $SRC -ne 0 ]]; then echo "run $RUN: owned follower processes survived" >&2; exit 1; fi
      case $STRC in
        5) echo "run $RUN: stop stream FAILED; state stays pending (retry follow-stop --run $RUN)" >&2; exit 5 ;;
        6) echo "run $RUN: stop stream NOT published: a newer run is active on $MASTER_NOW; state stays pending" >&2; exit 6 ;;
        7) echo "run $RUN: stop-stream state I/O error; completion cannot be recorded" >&2; exit 7 ;;
      esac
      exit 0
    fi
    if [[ -z "$MNONCE" ]]; then   # manual stop, nothing owned
      if legacy_running; then
        echo "legacy follower (pid $(cat "$PID")) has no owned manifest; stop it explicitly with follow-stop-legacy" >&2
        exit 4
      fi
      set +e; STREAM="$(stream_stop "$MASTER_NOW")"; STRC=$?; set -e
      echo "{\"mode\": \"manual_no_follower\", \"stopstream\": $STREAM, \"ok\": $([[ $STRC -eq 0 ]] && echo true || echo false)}"
      [[ $STRC -eq 0 ]] || { echo "stop stream FAILED" >&2; exit 5; }
      exit 0
    fi
    FMASTER="$(json_get "$FMAN" ros_master_uri)"
    STATE_ERR=""
    if [[ -n "$RUN" && -z "$(json_get "$FMAN" stopstream_wall_ns)" ]]; then
      INIT="$(stopstate "$RUN" "$FMASTER" init)" || STATE_ERR="$INIT"
    fi
    set +e
    RES="$("${OWN[@]}" stop --manifest "$FMAN" --grace "${AUTODRIVE_FOLLOW_STOP_GRACE:-20}")"; SRC=$?
    set -e
    # stream once per owned run until it succeeds (automatic cleanup); an explicit manual stop always streams
    STREAM='null'; STRC=0
    if [[ -z "$RUN" || -z "$(json_get "$FMAN" stopstream_wall_ns)" ]]; then
      set +e; STREAM="$(stream_stop "$FMASTER")"; STRC=$?; set -e
      record_stream "$FMAN" "$STREAM"
      if [[ -n "$RUN" ]]; then
        if [[ -n "$STATE_ERR" ]] || ! REC="$(stopstate "$RUN" "$FMASTER" record "$STREAM")"; then
          STREAM="{\"ok\": false, \"published\": $STREAM, \"state_error\": ${STATE_ERR:-$REC}}"; STRC=7
        fi
      fi
    fi
    [[ $SRC -eq 0 ]] && rm -f "$PID"
    echo "{\"mode\": \"manifest\", \"run\": \"$MNONCE\", \"stop\": $RES, \"stopstream\": $STREAM, \"stopstream_done_before\": $([[ "$STREAM" == null ]] && echo true || echo false), \"ok\": $([[ $SRC -eq 0 && $STRC -eq 0 ]] && echo true || echo false)}"
    if [[ $SRC -ne 0 ]]; then echo "run $MNONCE: owned follower processes SURVIVED the stop" >&2; exit 1; fi
    if [[ $STRC -eq 7 ]]; then echo "run $MNONCE: stop-stream state I/O error; completion cannot be recorded" >&2; exit 7; fi
    if [[ $STRC -ne 0 ]]; then echo "run $MNONCE: follower stopped but stop stream FAILED (retry follow-stop)" >&2; exit 5; fi
    exit 0
    ;;
  follow-stop-legacy)
    # explicit migration only: the old ctl.sh recorded a roslaunch session leader in follower.pid
    exec 7>"$LOCKS/autodrive-follow-state.lock"
    flock 7
    if ! legacy_running; then echo "no legacy follower"; exit 0; fi
    P="$(cat "$PID")"
    CMD="$(tr '\0' ' ' < "/proc/$P/cmdline" 2>/dev/null || true)"
    if [[ "$CMD" != *roslaunch*follower*.launch* ]] || [[ "$(ps -o sid= -p "$P" | tr -d ' ')" != "$P" ]]; then
      echo "pid $P is not a legacy follower session leader ($CMD); refusing" >&2; exit 4
    fi
    kill -INT -- -"$P" 2>/dev/null || true
    for _ in $(seq 1 80); do kill -0 "$P" 2>/dev/null || break; sleep 0.25; done
    kill -0 "$P" 2>/dev/null && kill -KILL -- -"$P" 2>/dev/null || true
    rm -f "$PID"
    set +e; STREAM="$(stream_stop "$ROS_MASTER_URI")"; STRC=$?; set -e
    echo "{\"mode\": \"legacy\", \"session\": $P, \"stopstream\": $STREAM}"
    [[ $STRC -eq 0 ]] || { echo "legacy follower stopped but stop stream FAILED" >&2; exit 5; }
    ;;
  follow-status)
    if [[ -f "$FMAN" ]] && "${OWN[@]}" alive --manifest "$FMAN"; then echo "running pid $(cat "$PID" 2>/dev/null) run $(json_get "$FMAN" nonce)"
    elif legacy_running; then echo "running pid $(cat "$PID") (legacy, unowned)"
    else echo "stopped"; fi
    ;;
  lidar-status)
    if [[ -f "$LIDAR_MAN" ]] && "${OWN[@]}" alive --manifest "$LIDAR_MAN"; then
      echo "running-owned"
    elif rosnode list 2>/dev/null | grep -qx '/lidar_stream_nodelet_manager'; then
      echo "running-external"
    else
      echo "stopped"
    fi
    ;;
  lidar-start)
    if [[ -f "$LIDAR_MAN" ]] && "${OWN[@]}" alive --manifest "$LIDAR_MAN"; then
      # device_ip filters the VLP16 sender. A stream left from the other network profile
      # (two_pc 192.168.0.1 vs single_pc 127.0.0.1) published a frozen scan; restart it.
      if [[ "$(cat "$LIDAR_MAN.source_ip" 2>/dev/null)" == "${MORAI_SIM_IP:-}" ]]; then
        echo "LiDAR stream already running (owned)"; exit 0
      fi
      echo "LiDAR stream source changed to ${MORAI_SIM_IP:-any}; restarting"
      "${OWN[@]}" stop --manifest "$LIDAR_MAN" --nonce "$(json_get "$LIDAR_MAN" nonce)" --grace 5
    fi
    if rosnode list 2>/dev/null | grep -qx '/lidar_stream_nodelet_manager'; then
      echo "LiDAR stream already running (external)"; exit 0
    fi
    rospack find velodyne_pointcloud >/dev/null || { echo "velodyne_pointcloud package missing; rebuild autodrive image" >&2; exit 1; }
    mkdir -p "$DATA"
    rm -f "$LIDAR_MAN" "$LIDAR_EXIT"
    LNONCE="lidar-$(date -u +%Y%m%dT%H%M%SZ)-$$"
    LIDAR_CMD=(roslaunch "$SELF_DIR/lidar_stream.launch")
    if [[ -n "${AUTODRIVE_LIDAR_CPUS-}" ]]; then
      taskset -c "$AUTODRIVE_LIDAR_CPUS" true || exit 2
      LIDAR_CMD=(taskset -c "$AUTODRIVE_LIDAR_CPUS" "${LIDAR_CMD[@]}")
    fi
    "${OWN[@]}" spawn --role lidar --nonce "$LNONCE" --manifest "$LIDAR_MAN" --log "$LIDAR_LOG" \
      --exit-file "$LIDAR_EXIT" -- "${LIDAR_CMD[@]}"
    printf '%s' "${MORAI_SIM_IP:-}" > "$LIDAR_MAN.source_ip"
    ;;
  lidar-stop)
    if [[ ! -f "$LIDAR_MAN" ]] || ! "${OWN[@]}" alive --manifest "$LIDAR_MAN"; then
      echo "LiDAR stream not owned or already stopped"; exit 0
    fi
    LNONCE="$(json_get "$LIDAR_MAN" nonce)"
    "${OWN[@]}" stop --manifest "$LIDAR_MAN" --nonce "$LNONCE" --grace 5
    ;;
  diag-start)
    shift; parse_run "$@"; set -- "${ARGS[@]+"${ARGS[@]}"}"
    SESSION=""; CAPDIR=""; META=""; READY_TIMEOUT=30
    while (($#)); do case "$1" in
      --session) SESSION="$2"; shift 2 ;; --capture-dir) CAPDIR="$2"; shift 2 ;;
      --meta) META="$2"; shift 2 ;; --ready-timeout) READY_TIMEOUT="$2"; shift 2 ;;
      *) echo "unknown diag-start arg $1" >&2; exit 2 ;; esac; done
    [[ -n "$RUN" && -n "$SESSION" && -n "$CAPDIR" ]] || { echo "diag-start needs --run --session --capture-dir" >&2; exit 2; }
    drive_owner_ok "$RUN" || { echo "run $RUN does not own the active drive" >&2; exit 3; }
    if [[ -f "$RMAN" ]] && "${OWN[@]}" alive --manifest "$RMAN"; then echo "a recorder is still running (run $(json_get "$RMAN" nonce))" >&2; exit 3; fi
    mkdir -p "$(dirname "$CAPDIR")"
    rm -f "$RMAN"
    read -r -a RC <<< "$RECORDER_CMD"
    EXTRA=(); [[ -n "$META" ]] && EXTRA=(--source-meta "$META")
    "${OWN[@]}" spawn --role recorder --nonce "$RUN" --manifest "$RMAN" --log "$CAPDIR.recorder.log" \
      --exit-file "$CAPDIR.recorder_exit.json" -- "${RC[@]}" --session-id "$SESSION" --run-id "$RUN" \
      --capture-root "$(dirname "$CAPDIR")" --capture-dir "$CAPDIR" --latest-csv "$LATEST_CSV" \
      --startup-timeout "$READY_TIMEOUT" "${EXTRA[@]+"${EXTRA[@]}"}" ${AUTODRIVE_RECORDER_ARGS:-} >/dev/null
    # committed readiness of exactly this run: marker identity == spawned child, capture.json recording
    python3 - "$RMAN" "$CAPDIR" "$RUN" "$READY_TIMEOUT" <<'PY'
import json, os, sys, time
man_p, cap, run, tmo = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
m = json.load(open(man_p))
def st(pid):
    try: return int(open("/proc/%d/stat" % pid).read().rsplit(")", 1)[1].split()[19])
    except Exception: return None
t_end = time.time() + tmo
while time.time() < t_end:
    try:
        r = json.load(open(os.path.join(cap, "ready")))
        c = json.load(open(os.path.join(cap, "capture.json")))
    except Exception:
        r = c = None
    if r:
        ok = (r.get("run_id") == run and r.get("committed") is True and r.get("pid") == m["child_pid"]
              and r.get("pid_starttime") == m["child_starttime"] == st(m["child_pid"]) and r.get("boot_id") == m["boot_id"]
              and c and c.get("state") == "recording" and c.get("run_id") == run)
        print(json.dumps({"ready": ok, "pid": m["child_pid"], "capture_dir": cap, "marker": r, "capture_state": c and c.get("state")}))
        sys.exit(0 if ok else 4)
    ex = m.get("exit_file")
    if ex and os.path.exists(ex):
        print(json.dumps({"ready": False, "exited": json.load(open(ex))})); sys.exit(4)
    time.sleep(0.05)
print(json.dumps({"ready": False, "timeout_s": tmo})); sys.exit(5)
PY
    ;;
  diag-stop)
    shift; parse_run "$@"; set -- "${ARGS[@]+"${ARGS[@]}"}"
    TIMEOUT=25; [[ "${1:-}" == "--timeout" ]] && TIMEOUT="$2"
    [[ -n "$RUN" ]] || { echo "diag-stop needs --run" >&2; exit 2; }
    # a cancelled diag-start may still be spawning: give its manifest a moment, never leave a nonce-owned recorder behind
    for _ in $(seq 1 50); do
      [[ -f "$RMAN" && "$(json_get "$RMAN" nonce)" == "$RUN" ]] && break
      [[ "$("${OWN[@]}" procs --nonce "$RUN" --role recorder)" == "[]" ]] && [[ $_ -gt 10 ]] && break
      sleep 0.1
    done
    if [[ ! -f "$RMAN" || "$(json_get "$RMAN" nonce)" != "$RUN" ]]; then
      if [[ "$("${OWN[@]}" procs --nonce "$RUN" --role recorder)" != "[]" ]]; then
        echo "{\"result\": \"orphan_recorder_without_manifest\", \"killed\": $(python3 "${OWN[1]}" _killnonce "$RUN" recorder)}"
      else
        echo '{"result": "no_recorder_for_run"}'
      fi
      exit 0
    fi
    SIG="$("${OWN[@]}" signal-once --manifest "$RMAN" --nonce "$RUN" --signal INT)"
    set +e
    EXIT="$("${OWN[@]}" wait-exit --manifest "$RMAN" --timeout "$TIMEOUT")"; WRC=$?
    FORCED=null
    if [[ $WRC -eq 1 ]]; then   # still running after the deadline: forced termination, never a second SIGINT
      FORCED="$("${OWN[@]}" kill --manifest "$RMAN" --nonce "$RUN")"
      EXIT="$("${OWN[@]}" wait-exit --manifest "$RMAN" --timeout 5)"
    fi
    set -e
    echo "{\"signal\": ${SIG:-null}, \"exit\": ${EXIT:-null}, \"forced\": ${FORCED:-null}}"
    ;;
  *) echo "usage: ctl.sh follow-start [kmh] [follow|avoid|avoid-bspline-test] [--run N] | follow-stop [--run N] | follow-stop-legacy | follow-status | diag-start ... | diag-stop ..." >&2; exit 2 ;;
esac
