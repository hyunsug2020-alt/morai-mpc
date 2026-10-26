#!/usr/bin/env python3
"""Record GPS/IMU localization and /planning/global_path as JSONL on stdout.

Runs INSIDE the ros1_bev_dh container (it has rospy + morai_msgs). The host pipes this file in:
    docker exec -i ros1_bev_dh bash -lc 'source ...; python3 -u - --duration-seconds 120' \
        < collect/record_ego.py > data/raw/<session>/ego_raw.jsonl
Ego timestamps use header.stamp (wallclock, same clock as the camera UDP header).
"""
import argparse
import os
import hashlib
import json
import signal
import sys
import threading
import time

import rospy
from morai_msgs.msg import CollisionData, EgoVehicleStatus
from nav_msgs.msg import Path as PathMsg


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--duration-seconds", type=float, required=True)
    p.add_argument("--stop-file", help="stop when this session marker appears")
    p.add_argument("--ego-topic", default="/localization/ego_status")
    p.add_argument("--path-topic", default="/planning/global_path")
    p.add_argument("--collision-topic", default="/CollisionData")
    args = p.parse_args()

    out = sys.stdout
    state = {"ego": 0, "route": 0, "collision": 0, "route_hash": None, "stop": False}
    lock = threading.Lock()  # ego and path callbacks run on different rospy threads; keep each line atomic

    def emit(obj):
        line = json.dumps(obj, separators=(",", ":")) + "\n"
        with lock:
            out.write(line)
            out.flush()

    def on_ego(m):
        stamp_us = m.header.stamp.secs * 1_000_000 + m.header.stamp.nsecs // 1000
        emit({"type": "ego", "t_us": stamp_us, "recv_us": int(time.time() * 1e6), "seq": m.header.seq,
              "x": m.position.x, "y": m.position.y, "z": m.position.z, "heading_deg": m.heading,
              "vx": m.velocity.x, "vy": m.velocity.y, "vz": m.velocity.z,
              "ax": m.acceleration.x, "ay": m.acceleration.y,
              "accel": float(m.accel), "brake": float(m.brake), "wheel_angle": float(m.wheel_angle)})
        state["ego"] += 1

    def on_path(m):
        poses = []
        for ps in m.poses:
            q = ps.pose.orientation
            poses.append([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, q.x, q.y, q.z, q.w])
        h = hashlib.sha1(json.dumps(poses).encode()).hexdigest()
        if h == state["route_hash"]:
            return
        state["route_hash"] = h
        emit({"type": "route", "t_us": m.header.stamp.secs * 1_000_000 + m.header.stamp.nsecs // 1000,
              "recv_us": int(time.time() * 1e6), "frame_id": m.header.frame_id, "sha1": h,
              "poses_xyz_qxyzw": poses})
        state["route"] += 1

    def on_collision(m):
        if not m.collision_object:
            return
        emit({"type": "collision", "t_us": m.header.stamp.secs * 1_000_000 + m.header.stamp.nsecs // 1000,
              "recv_us": int(time.time() * 1e6),
              "objects": [{"id": o.unique_id, "type": o.type, "name": o.name, "x": o.position.x, "y": o.position.y} for o in m.collision_object]})
        state["collision"] += 1

    rospy.init_node("rgb_teacher_student_ego_recorder", anonymous=True, disable_signals=True)
    rospy.Subscriber(args.collision_topic, CollisionData, on_collision, queue_size=50)
    rospy.Subscriber(args.ego_topic, EgoVehicleStatus, on_ego, queue_size=200)
    rospy.Subscriber(args.path_topic, PathMsg, on_path, queue_size=2)

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
            print(f"ego={state['ego']} route_msgs={state['route']} collisions={state['collision']}", file=sys.stderr, flush=True)
    out.flush()
    print(f"done ego={state['ego']} route_msgs={state['route']} collisions={state['collision']}", file=sys.stderr, flush=True)
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
