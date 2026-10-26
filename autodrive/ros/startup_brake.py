#!/usr/bin/env python3
"""Hold an owned launch at rest until its follower sends the first control."""
import argparse
import json
import os
from pathlib import Path
import signal
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--follower-manifest", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=300.)
    args = parser.parse_args()
    if not 1. <= args.timeout <= 900.:
        parser.error("timeout must be between 1 and 900 seconds")

    import rospy
    from morai_msgs.msg import CtrlCmd

    done = threading.Event()
    lock = threading.Lock()
    state = {"state": "holding", "run": args.run_id, "caller": None}

    def record():
        payload = dict(state, updated_at=time.time())
        temporary = args.ready_file.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload) + "\n")
        os.replace(str(temporary), str(args.ready_file))

    def observe(message):
        caller = (getattr(message, "_connection_header", None) or {}).get("callerid")
        if caller == "/path_follower_node":
            try:
                owner = json.loads(args.follower_manifest.read_text())
            except (OSError, ValueError):
                return
            if owner.get("nonce") != args.run_id or owner.get("stopped"):
                return
            with lock:
                if state["state"] == "holding":
                    state.update(state="handoff", caller=caller)
                    record()
                    done.set()

    def stop(signum, _frame):
        done.set()

    args.ready_file.parent.mkdir(parents=True, exist_ok=True)
    rospy.init_node("autodrive_start_hold", anonymous=True, disable_signals=True)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    subscriber = rospy.Subscriber("/ctrl_cmd", CtrlCmd, observe, queue_size=1)
    publisher = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
    message = CtrlCmd(longlCmdType=1, accel=0., brake=1., steering=0.,
                      velocity=0., acceleration=0.)
    with lock:
        record()
    deadline = time.monotonic() + args.timeout
    print("Startup brake active; waiting for /path_follower_node", flush=True)
    while not done.is_set() and not rospy.is_shutdown() and time.monotonic() < deadline:
        # Serialise publication and handoff so no brake follows the accepted command.
        with lock:
            if state["state"] == "holding":
                publisher.publish(message)
        done.wait(.05)
    with lock:
        if state["state"] == "holding":
            state["state"] = "timeout" if time.monotonic() >= deadline else "stopped"
            record()
    publisher.unregister()
    subscriber.unregister()
    rospy.signal_shutdown("startup brake " + state["state"])
    print("Startup brake " + state["state"], flush=True)
    return 0 if state["state"] == "handoff" else 4


if __name__ == "__main__":
    raise SystemExit(main())
