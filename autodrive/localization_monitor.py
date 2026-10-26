#!/usr/bin/env python3
"""Competition-safe drive status: /localization/ego_status only (never /Ego_topic).

Writes a status JSON (same shape the drive scripts read: time, ego_hz, ego.speed_kmh,
path.distance_m) every 0.2 s, and appends a 5 Hz CSV trace for debugging.
"""
import argparse
import csv
import json
import math
import os
import threading
import time
from collections import deque

import rospy
from morai_msgs.msg import EgoVehicleStatus


class EgoStatusHistory:
    """Capture a coherent pose and rate history across ROS callback threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self._stamps = deque(maxlen=200)
        self._message = None
        self._received = None

    def record(self, message):
        with self._lock:
            received = time.monotonic()
            self._stamps.append(received)
            self._message, self._received = message, received

    def snapshot(self):
        with self._lock:
            return tuple(self._stamps), self._message, self._received


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", required=True)
    parser.add_argument("--status", required=True)
    parser.add_argument("--trace", required=True)
    parser.add_argument("--topic", default="/localization/ego_status")
    args = parser.parse_args()

    with open(args.route) as source:
        route = [(float(p["x"]), float(p["y"])) for p in json.load(source)["waypoints"]]
    rospy.init_node("localization_monitor", anonymous=True, disable_signals=True)
    history = EgoStatusHistory()
    rospy.Subscriber(args.topic, EgoVehicleStatus, history.record, queue_size=20)
    trace_file = open(args.trace, "a", newline="")
    trace = csv.writer(trace_file)
    if trace_file.tell() == 0:
        trace.writerow(["wall_time", "x", "y", "heading_deg", "speed_kmh", "wheel_angle", "route_idx", "route_dist_m", "hz"])
    last_trace = 0.0
    while not rospy.is_shutdown():
        stamps, msg, received = history.snapshot()
        now_mono = time.monotonic()
        recent = [t for t in stamps if now_mono - t <= 2.0]
        hz = len(recent) / 2.0
        status = {"time": time.time(), "ego_hz": hz, "source": args.topic}
        if msg is not None and now_mono - received < 1.0:
            x, y = msg.position.x, msg.position.y
            nearest = min(range(len(route)), key=lambda i: (route[i][0] - x) ** 2 + (route[i][1] - y) ** 2)
            dist = math.hypot(route[nearest][0] - x, route[nearest][1] - y)
            speed_kmh = msg.velocity.x * 3.6
            status["ego"] = {"x": x, "y": y, "heading_deg": msg.heading, "speed_kmh": speed_kmh,
                             "wheel_angle": msg.wheel_angle, "age_s": round(now_mono - received, 3)}
            status["path"] = {"nearest_idx": nearest, "distance_m": dist}
            if now_mono - last_trace >= 0.2:
                trace.writerow([f"{time.time():.3f}", f"{x:.3f}", f"{y:.3f}", f"{msg.heading:.2f}", f"{speed_kmh:.2f}",
                                f"{msg.wheel_angle:.3f}", nearest, f"{dist:.3f}", f"{hz:.1f}"])
                trace_file.flush()
                last_trace = now_mono
        tmp = args.status + ".tmp"
        with open(tmp, "w") as out:
            json.dump(status, out)
        os.replace(tmp, args.status)
        time.sleep(0.2)
    trace_file.close()


if __name__ == "__main__":
    main()
