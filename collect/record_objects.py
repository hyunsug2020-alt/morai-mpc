#!/usr/bin/env python3
"""Record sensor-fused /perception/objects messages as JSONL on stdout.

Runs INSIDE the ROS container, same as record_ego.py. The host pipes this file in:
    docker exec -i morai_autodrive /opt/autodrive/ros/entrypoint.sh python3 -u - --duration-seconds 120 \
        < collect/record_objects.py > data/raw/<session>/objects.jsonl
One line per message: {"type":"objects","t_us","recv_us","seq","npc":[..],"pedestrian":[..],"obstacle":[..]}
Each object keeps the full morai_msgs/ObjectStatus (id, type, name, heading, position, velocity, acceleration, size).
t_us is header.stamp (0 if MORAI leaves it unset); recv_us is host wallclock on receipt, same clock as the cameras.
"""
import argparse
import os
import json
import signal
import sys
import time

import rospy
from morai_msgs.msg import ObjectStatusList


def obj_to_dict(o):
    return {"id": o.unique_id, "type": o.type, "name": o.name, "heading_deg": o.heading,
            "x": o.position.x, "y": o.position.y, "z": o.position.z,
            "vx": o.velocity.x, "vy": o.velocity.y, "vz": o.velocity.z,
            "ax": o.acceleration.x, "ay": o.acceleration.y, "az": o.acceleration.z,
            "size_x": o.size.x, "size_y": o.size.y, "size_z": o.size.z}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--duration-seconds", type=float, required=True)
    p.add_argument("--stop-file", help="stop when this session marker appears")
    p.add_argument("--object-topic", default="/perception/objects")
    args = p.parse_args()

    out = sys.stdout
    state = {"msgs": 0, "objects": 0, "stop": False}

    def on_objects(m):
        rec = {"type": "objects", "t_us": m.header.stamp.secs * 1_000_000 + m.header.stamp.nsecs // 1000,
               "recv_us": int(time.time() * 1e6), "seq": m.header.seq,
               "npc": [obj_to_dict(o) for o in m.npc_list],
               "pedestrian": [obj_to_dict(o) for o in m.pedestrian_list],
               "obstacle": [obj_to_dict(o) for o in m.obstacle_list]}
        out.write(json.dumps(rec, separators=(",", ":")) + "\n")
        out.flush()
        state["msgs"] += 1
        state["objects"] += len(rec["npc"]) + len(rec["pedestrian"]) + len(rec["obstacle"])

    rospy.init_node("rgb_teacher_student_object_recorder", anonymous=True, disable_signals=True)
    rospy.Subscriber(args.object_topic, ObjectStatusList, on_objects, queue_size=200)

    def on_signal(signum, frame):
        state["stop"] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    deadline = time.monotonic() + args.duration_seconds
    last = time.monotonic()
    while not state["stop"] and time.monotonic() < deadline and not rospy.is_shutdown() and not (args.stop_file and os.path.exists(args.stop_file)):
        time.sleep(0.05)
        if time.monotonic() - last >= 2.0:
            last = time.monotonic()
            print(f"object_msgs={state['msgs']} objects={state['objects']}", file=sys.stderr, flush=True)
    out.flush()
    print(f"done object_msgs={state['msgs']} objects={state['objects']}", file=sys.stderr, flush=True)
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
