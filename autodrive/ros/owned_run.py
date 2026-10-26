#!/usr/bin/env python3
"""Owned-run process control for ctl.sh (inside the ROS container, stdlib only).

A run is identified by a nonce. Every process of a run carries AUTODRIVE_RUN_NONCE=<nonce> and
AUTODRIVE_RUN_ROLE=<role> in its environment (inherited by roslaunch nodes and their children),
so ownership is proven per process from /proc/<pid>/environ plus /proc/<pid>/stat start time and
the kernel boot id - never from a process name, a PID file alone or a ROS node name. Processes of
another master, another run or a reused PID are therefore never signalled.

  spawn     --role R --nonce N --manifest M --log L --exit-file E -- cmd...
            start a detached supervisor (own session). The supervisor persists a "starting" manifest
            BEFORE it starts cmd, then the "running" manifest with the child identity, and when cmd
            ends its exit status to E. It ignores SIGINT/SIGTERM. Spawn is a transaction: if the
            running manifest is not published (write error, supervisor death, timeout), every verified
            process of this spawn (nonce + role + master, started after the spawn began) is SIGKILLed
            and the result, including survivors, is reported with a nonzero exit.
  procs     --nonce N [--role R] [--master URI] | --role R   list live owned processes (JSON)
  stop-run  --nonce N --role R --master URI [--grace S]   stop a run's processes without a manifest
  stopstate --dir D --nonce N --master URI --op init|get|record [--result JSON]
            durable per (nonce, master) stop-stream state: "pending" until a successful local publication
            is recorded, then "done". Atomic writes (tmp+fsync+rename+dir fsync); I/O errors exit 2.
  alive     --manifest M                              exit 0 if the manifest's run has live processes
  stop      --manifest M [--nonce N] [--grace S]      SIGINT the verified group, wait, SIGKILL verified leftovers
  signal-once --manifest M --nonce N --signal INT    signal the verified child once (recorded in manifest)
  wait-exit --manifest M --timeout S                  wait for the supervisor's exit record
  kill      --manifest M --nonce N                    SIGKILL every verified process of the run
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

NONCE_ENV, ROLE_ENV = "AUTODRIVE_RUN_NONCE", "AUTODRIVE_RUN_ROLE"


def boot_id():
    try:
        return open("/proc/sys/kernel/random/boot_id").read().strip()
    except OSError:
        return None


def stat(pid):
    """(starttime, pgid, sid, state) or None."""
    try:
        with open("/proc/%d/stat" % pid) as f:
            rest = f.read().rsplit(")", 1)[1].split()
        return int(rest[19]), int(rest[2]), int(rest[3]), rest[0]
    except (OSError, IndexError, ValueError):
        return None


def environ(pid):
    try:
        with open("/proc/%d/environ" % pid, "rb") as f:
            items = f.read().split(b"\0")
    except OSError:
        return {}
    out = {}
    for it in items:
        if b"=" in it:
            k, v = it.split(b"=", 1)
            if k in (NONCE_ENV.encode(), ROLE_ENV.encode(), b"ROS_MASTER_URI"):
                out[k.decode()] = v.decode(errors="replace")
    return out


def cmdline(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            return f.read().replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        return ""


def owned_procs(nonce=None, role=None, master=None, since=None):
    out = []
    me = os.getpid()
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == me:
            continue
        pid = int(d)
        env = environ(pid)
        if nonce is not None and env.get(NONCE_ENV) != nonce:
            continue
        if nonce is None and NONCE_ENV not in env:
            continue
        if role is not None and env.get(ROLE_ENV) != role:
            continue
        if master is not None and env.get("ROS_MASTER_URI") != master:
            continue
        st = stat(pid)
        if st is None or st[3] == "Z":
            continue
        if since is not None and st[0] < since:
            continue
        out.append({"pid": pid, "starttime": st[0], "pgid": st[1], "sid": st[2], "nonce": env.get(NONCE_ENV),
                    "role": env.get(ROLE_ENV), "master": env.get("ROS_MASTER_URI"), "cmd": cmdline(pid)[:200]})
    return out


def verified(p):
    """Re-check that pid still is the same owned process (nonce + start time) right before signalling."""
    st = stat(p["pid"])
    return st is not None and st[3] != "Z" and st[0] == p["starttime"] and environ(p["pid"]).get(NONCE_ENV) == p["nonce"]


def atomic_json(path, obj):
    tmp = "%s.tmp.%d" % (path, os.getpid())
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def load(path):
    try:
        return json.load(open(path))
    except (OSError, ValueError):
        return None


def ident_ok(man, key):
    """Leader/child identity from the manifest still valid (same boot, same pid start time)."""
    pid, st0 = man.get(key + "_pid"), man.get(key + "_starttime")
    if not pid or man.get("boot_id") != boot_id():
        return False
    st = stat(pid)
    return st is not None and st[3] != "Z" and st[0] == st0 and environ(pid).get(NONCE_ENV) == man.get("nonce")


def reap_supervisor(nonce, role, sup, wait=3.0):
    """Give the supervisor time to write the exit record after its child ended; SIGKILL it only if it lingers."""
    t_end = time.time() + wait
    while time.time() < t_end:
        if not [p for p in owned_procs(nonce, role) if p["pid"] == sup]:
            return []
        time.sleep(0.05)
    return kill_verified([p for p in owned_procs(nonce, role) if p["pid"] == sup], signal.SIGKILL)


def rollback(nonce, role, master, since, wait=3.0):
    """SIGKILL every verified process of one spawn (nonce + role + master, started at/after `since`)."""
    killed = kill_verified(owned_procs(nonce, role, master, since), signal.SIGKILL)
    t_end = time.time() + wait
    while time.time() < t_end and owned_procs(nonce, role, master, since):
        killed += kill_verified(owned_procs(nonce, role, master, since), signal.SIGKILL)
        time.sleep(0.05)
    return {"killed": sorted(set(killed)), "survivors": [p["pid"] for p in owned_procs(nonce, role, master, since)]}


def cmd_spawn(a):
    if os.path.exists(a.exit_file):
        os.unlink(a.exit_file)
    master = os.environ.get("ROS_MASTER_URI")
    env = dict(os.environ, **{NONCE_ENV: a.nonce, ROLE_ENV: a.role})
    since = stat(os.getpid())[0]           # every process of this spawn starts after this process
    sup, err = None, None
    try:
        log = open(a.log, "ab", buffering=0)
        sup = subprocess.Popen([sys.executable, os.path.abspath(__file__), "_supervise", "--role", a.role, "--nonce", a.nonce,
                                "--manifest", a.manifest, "--exit-file", a.exit_file, "--"] + a.cmd,
                               env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True, close_fds=True)
        t_end = time.time() + a.timeout
        while time.time() < t_end:
            m = load(a.manifest)
            if m and m.get("nonce") == a.nonce and m.get("supervisor_pid") == sup.pid and m.get("state") == "running":
                print(json.dumps(m, sort_keys=True))
                return 0
            if sup.poll() is not None:
                err = "supervisor exited (rc %s) before publishing the running manifest" % sup.poll()
                break
            time.sleep(0.02)
        else:
            err = "running manifest not published within %.1f s" % a.timeout
    except BaseException as e:
        err = "spawn exception: %r" % (e,)
    rb = rollback(a.nonce, a.role, master, since)
    m = load(a.manifest)
    if m and sup is not None and m.get("supervisor_pid") == sup.pid:
        try:
            m.update(state="spawn_failed", spawn_error=err, rollback=rb)
            atomic_json(a.manifest, m)
        except OSError:
            pass
    print(json.dumps({"error": err, "supervisor_pid": sup.pid if sup else None, "rollback": rb}, sort_keys=True))
    return 1 if not rb["survivors"] else 3


def _test_fault(name):
    """Test seam (inert unless OWNED_RUN_TEST_FAULT names the fault, optionally as fault@role): exercises spawn rollback paths."""
    faults = os.environ.get("OWNED_RUN_TEST_FAULT", "").split(",")
    return name in faults or "%s@%s" % (name, os.environ.get(ROLE_ENV, "")) in faults


def cmd_supervise(a):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    me = stat(os.getpid())
    man = {"nonce": a.nonce, "role": a.role, "boot_id": boot_id(), "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
           "supervisor_pid": os.getpid(), "supervisor_starttime": me[0], "pgid": me[1], "sid": me[2],
           "child_pid": None, "child_starttime": None, "cmd": a.cmd, "state": "starting",
           "started_wall_ns": time.time_ns(), "exit_file": a.exit_file}
    if _test_fault("fail_premanifest"):
        raise OSError("injected pre-manifest write failure")
    atomic_json(a.manifest, man)          # failure here: no child was ever started
    child = subprocess.Popen(a.cmd, stdin=subprocess.DEVNULL, restore_signals=True,
                             preexec_fn=lambda: (signal.signal(signal.SIGINT, signal.SIG_DFL), signal.signal(signal.SIGTERM, signal.SIG_DFL)))
    try:
        if _test_fault("crash_after_child"):
            os._exit(9)
        if _test_fault("hang_before_running_manifest"):
            time.sleep(3600)
        if _test_fault("fail_running_manifest"):
            raise OSError("injected running-manifest write failure")
        cs = stat(child.pid)
        man.update(child_pid=child.pid, child_starttime=cs[0] if cs else None, state="running")
        atomic_json(a.manifest, man)
    except BaseException as e:
        rb = rollback(a.nonce, a.role, os.environ.get("ROS_MASTER_URI"), me[0])
        sys.stderr.write("supervisor: running manifest failed (%r); rollback %s\n" % (e, json.dumps(rb)))
        return 2
    rc = child.wait()
    atomic_json(a.exit_file, {"nonce": a.nonce, "role": a.role, "child_pid": child.pid,
                              "exit_code": rc if rc >= 0 else None, "signal": -rc if rc < 0 else None,
                              "end_wall_ns": time.time_ns()})
    return 0


def cmd_procs(a):
    print(json.dumps(owned_procs(a.nonce, a.role, a.master)))
    return 0


def cmd_alive(a):
    m = load(a.manifest)
    if not m:
        return 1
    return 0 if owned_procs(m["nonce"], m.get("role"), m.get("ros_master_uri")) else 1


def kill_verified(procs, sig):
    done = []
    for p in procs:
        if verified(p):
            try:
                os.kill(p["pid"], sig)
                done.append(p["pid"])
            except ProcessLookupError:
                pass
    return done


def cmd_stop(a):
    m = load(a.manifest)
    if not m:
        print(json.dumps({"result": "no_manifest"}))
        return 0
    if a.nonce and m.get("nonce") != a.nonce:
        print(json.dumps({"result": "not_owner", "manifest_nonce": m.get("nonce")}))
        return 0
    nonce, role, master = m["nonce"], m.get("role"), m.get("ros_master_uri")
    res = {"result": "stopped", "nonce": nonce, "role": role, "master": master, "signalled_group": None, "sigint_pids": [],
           "forced_pids": [], "survivors": [], "start_wall_ns": time.time_ns()}
    procs = owned_procs(nonce, role, master)
    if not procs:
        res["result"] = "not_running"
    else:
        if ident_ok(m, "supervisor"):
            # Leader verified in this boot: the whole session group belongs to this run.
            try:
                os.killpg(m["pgid"], signal.SIGINT)
                res["signalled_group"] = m["pgid"]
            except ProcessLookupError:
                pass
        else:
            res["sigint_pids"] = kill_verified(procs, signal.SIGINT)
        t_end = time.time() + a.grace
        while time.time() < t_end:
            left = [p for p in owned_procs(nonce, role, master) if p["pid"] != m.get("supervisor_pid")]
            if not left:
                break
            time.sleep(0.1)
        sup = m.get("supervisor_pid")
        res["forced_pids"] = kill_verified([p for p in owned_procs(nonce, role, master) if p["pid"] != sup], signal.SIGKILL)
        res["forced_pids"] += reap_supervisor(nonce, role, sup)
        res["survivors"] = [p["pid"] for p in owned_procs(nonce, role, master)]
    res["end_wall_ns"] = time.time_ns()
    m.setdefault("stops", []).append(res)
    m["stopped"] = not res["survivors"]
    atomic_json(a.manifest, m)
    print(json.dumps(res, sort_keys=True))
    return 0 if not res["survivors"] else 1


def cmd_stop_run(a):
    """No manifest (lost/failed spawn): stop exactly the processes of nonce + role + master."""
    res = {"result": "stopped", "nonce": a.nonce, "role": a.role, "master": a.master, "found": [], "sigint_pids": [],
           "forced_pids": [], "survivors": [], "start_wall_ns": time.time_ns()}
    procs = owned_procs(a.nonce, a.role, a.master)
    res["found"] = [p["pid"] for p in procs]
    if not procs:
        res["result"] = "not_running"
    else:
        res["sigint_pids"] = kill_verified(procs, signal.SIGINT)
        t_end = time.time() + a.grace
        while time.time() < t_end and owned_procs(a.nonce, a.role, a.master):
            time.sleep(0.1)
        res["forced_pids"] = kill_verified(owned_procs(a.nonce, a.role, a.master), signal.SIGKILL)
        time.sleep(0.3)
        res["survivors"] = [p["pid"] for p in owned_procs(a.nonce, a.role, a.master)]
    res["end_wall_ns"] = time.time_ns()
    print(json.dumps(res, sort_keys=True))
    return 0 if not res["survivors"] else 1


def stopstate_path(d, nonce, master):
    import hashlib
    if not nonce or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-" for c in nonce):
        raise ValueError("invalid nonce %r" % nonce)
    return os.path.join(d, "%s__%s.json" % (nonce, hashlib.sha256(master.encode()).hexdigest()[:16]))


def durable_json(path, obj):
    atomic_json(path, obj)
    fd = os.open(os.path.dirname(path), os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def cmd_stopstate(a):
    try:
        path = stopstate_path(a.dir, a.nonce, a.master)
        cur = None
        if os.path.exists(path):
            cur = json.load(open(path))
            if cur.get("nonce") != a.nonce or cur.get("master") != a.master:
                raise ValueError("state file identity mismatch: %s" % path)
        if a.op == "get":
            print(json.dumps(cur or {"state": "absent", "nonce": a.nonce, "master": a.master, "path": path}, sort_keys=True))
            return 0
        if a.op == "init":
            if cur is None:
                os.makedirs(a.dir, exist_ok=True)
                cur = {"nonce": a.nonce, "master": a.master, "state": "pending", "created_wall_ns": time.time_ns(),
                       "attempts": [], "path": path}
                durable_json(path, cur)
            print(json.dumps(cur, sort_keys=True))
            return 0
        if a.op == "record":
            if cur is None:
                raise ValueError("record without init: %s" % path)
            r = json.loads(a.result)
            cur.setdefault("attempts", []).append(r)
            if r.get("ok") is True and cur.get("state") != "done":
                cur["state"], cur["done_wall_ns"] = "done", r.get("end_wall_ns") or time.time_ns()
            durable_json(path, cur)
            print(json.dumps(cur, sort_keys=True))
            return 0
    except Exception as e:
        print(json.dumps({"error": repr(e)[:300], "op": a.op, "nonce": a.nonce, "master": a.master}))
        return 2


def cmd_signal_once(a):
    m = load(a.manifest)
    if not m or m.get("nonce") != a.nonce:
        print(json.dumps({"result": "not_owner"}))
        return 0
    if m.get("signal_sent"):
        print(json.dumps({"result": "already_signalled", "signal_sent": m["signal_sent"]}))
        return 0
    if not ident_ok(m, "child"):
        print(json.dumps({"result": "child_not_running_or_identity_changed"}))
        return 0
    sig = getattr(signal, "SIG" + a.signal)
    os.kill(m["child_pid"], sig)
    m["signal_sent"] = {"signal": a.signal, "wall_ns": time.time_ns(), "pid": m["child_pid"]}
    atomic_json(a.manifest, m)
    print(json.dumps({"result": "signalled", "signal_sent": m["signal_sent"]}))
    return 0


def cmd_wait_exit(a):
    m = load(a.manifest)
    if not m:
        print(json.dumps({"result": "no_manifest"}))
        return 2
    t_end = time.time() + a.timeout
    while True:
        e = load(m["exit_file"])
        if e and e.get("nonce") == m["nonce"]:
            print(json.dumps(dict(e, result="exited"), sort_keys=True))
            return 0
        if not ident_ok(m, "supervisor") and not ident_ok(m, "child"):
            print(json.dumps({"result": "vanished_without_exit_record"}))
            return 3
        if time.time() >= t_end:
            print(json.dumps({"result": "timeout"}))
            return 1
        time.sleep(0.1)


def cmd_kill(a):
    m = load(a.manifest)
    if not m or m.get("nonce") != a.nonce:
        print(json.dumps({"result": "not_owner"}))
        return 0
    sup = m.get("supervisor_pid")
    # spare the supervisor first so it can still record the child's (forced) exit status
    killed = kill_verified([p for p in owned_procs(a.nonce, m.get("role")) if p["pid"] != sup], signal.SIGKILL)
    killed += reap_supervisor(a.nonce, m.get("role"), sup)
    left = [p["pid"] for p in owned_procs(a.nonce, m.get("role"))]
    print(json.dumps({"result": "killed", "pids": killed, "survivors": left}))
    return 0 if not left else 1


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "_killnonce":   # used by ctl.sh when a spawn never published its manifest
        print(json.dumps(kill_verified(owned_procs(sys.argv[2], sys.argv[3]), signal.SIGKILL)))
        return 0
    p = argparse.ArgumentParser()
    sp = p.add_subparsers(dest="cmd_name", required=True)
    s = sp.add_parser("spawn")
    s.add_argument("--role", required=True); s.add_argument("--nonce", required=True)
    s.add_argument("--manifest", required=True); s.add_argument("--log", required=True)
    s.add_argument("--exit-file", required=True); s.add_argument("--timeout", type=float, default=10.0)
    s.add_argument("cmd", nargs=argparse.REMAINDER)
    s = sp.add_parser("_supervise")
    s.add_argument("--role", required=True); s.add_argument("--nonce", required=True)
    s.add_argument("--manifest", required=True); s.add_argument("--exit-file", required=True)
    s.add_argument("cmd", nargs=argparse.REMAINDER)
    s = sp.add_parser("procs"); s.add_argument("--nonce"); s.add_argument("--role"); s.add_argument("--master")
    s = sp.add_parser("stopstate"); s.add_argument("--dir", required=True); s.add_argument("--nonce", required=True)
    s.add_argument("--master", required=True); s.add_argument("--op", required=True, choices=["init", "get", "record"])
    s.add_argument("--result", default="{}")
    s = sp.add_parser("stop-run"); s.add_argument("--nonce", required=True); s.add_argument("--role", required=True)
    s.add_argument("--master", required=True); s.add_argument("--grace", type=float, default=20.0)
    s = sp.add_parser("alive"); s.add_argument("--manifest", required=True)
    s = sp.add_parser("stop"); s.add_argument("--manifest", required=True); s.add_argument("--nonce"); s.add_argument("--grace", type=float, default=20.0)
    s = sp.add_parser("signal-once"); s.add_argument("--manifest", required=True); s.add_argument("--nonce", required=True)
    s.add_argument("--signal", default="INT", choices=["INT", "TERM"])
    s = sp.add_parser("wait-exit"); s.add_argument("--manifest", required=True); s.add_argument("--timeout", type=float, default=25.0)
    s = sp.add_parser("kill"); s.add_argument("--manifest", required=True); s.add_argument("--nonce", required=True)
    a = p.parse_args()
    if hasattr(a, "cmd") and a.cmd and a.cmd[0] == "--":
        a.cmd = a.cmd[1:]
    if a.cmd_name == "procs" and not a.nonce and not a.role:
        p.error("procs needs --nonce or --role")
    return {"spawn": cmd_spawn, "_supervise": cmd_supervise, "procs": cmd_procs, "alive": cmd_alive, "stop": cmd_stop, "stop-run": cmd_stop_run, "stopstate": cmd_stopstate,
            "signal-once": cmd_signal_once, "wait-exit": cmd_wait_exit, "kill": cmd_kill}[a.cmd_name](a)


if __name__ == "__main__":
    sys.exit(main())
