#!/usr/bin/env python3
"""Simulated GPS blackout (competition tunnel): relay /gps_raw -> /gps, except inside
the blackout stretch of the global route, where fixes are dropped so localization
runs on IMU + wheel odometry only. The zone test uses the GPS fix itself (never /Ego_topic).

Also publishes /gps_blackout/active (std_msgs/Bool, 5 Hz) for the debug bag.
"""
import argparse
import json
import math
import sys
import threading
from pathlib import Path

import rospy
from morai_msgs.msg import GPSMessage
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "localization/src/eskf"))
from util import latlon_to_utm  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", required=True)
    parser.add_argument("--start", required=True, help="x,y of the blackout entry (local ENU)")
    parser.add_argument("--end", required=True, help="x,y of the blackout exit (local ENU)")
    parser.add_argument("--corridor-m", type=float, default=15.0, help="max lateral distance from the route")
    parser.add_argument("--utm-zone", type=int, default=52)
    args = parser.parse_args(rospy.myargv()[1:])

    with open(args.route) as source:
        route = [(float(p["x"]), float(p["y"])) for p in json.load(source)["waypoints"]]

    def nearest(x, y):
        i = min(range(len(route)), key=lambda k: (route[k][0] - x) ** 2 + (route[k][1] - y) ** 2)
        return i, math.hypot(route[i][0] - x, route[i][1] - y)

    i_start, _ = nearest(*map(float, args.start.split(",")))
    i_end, _ = nearest(*map(float, args.end.split(",")))
    lo, hi = min(i_start, i_end), max(i_start, i_end)

    rospy.init_node("gps_blackout_gate")
    pub = rospy.Publisher("/gps", GPSMessage, queue_size=10)
    active_pub = rospy.Publisher("/gps_blackout/active", Bool, queue_size=1, latch=True)
    state = {"active": False, "enabled": False}
    lock = threading.Lock()
    # Blackout only while ~enabled is true (set by the competition launch); otherwise pass-through.
    state["enabled"] = bool(rospy.get_param("~enabled", False))
    rospy.loginfo("[gps_blackout] route idx %d..%d (%d points), enabled=%s", lo, hi, hi - lo + 1, state["enabled"])

    def on_gps(msg):
        inside = False
        if state["enabled"]:
            # MORAI itself emits lat=lon=0 fixes inside the tunnel, interleaved with a few valid
            # ones. An invalid fix carries no position: keep the current state (and drop it while
            # blacked out) instead of treating it as "left the zone".
            if abs(msg.latitude) < 1e-6 and abs(msg.longitude) < 1e-6:
                inside = state["active"]
            else:
                ux, uy = latlon_to_utm(msg.latitude, msg.longitude, args.utm_zone)
                idx, lateral = nearest(ux - msg.eastOffset, uy - msg.northOffset)
                inside = lo <= idx <= hi and lateral <= args.corridor_m
        with lock:
            if inside != state["active"]:
                state["active"] = inside
                rospy.logwarn("[gps_blackout] %s", "GPS BLACKOUT START (tunnel)" if inside else "GPS restored")
                active_pub.publish(Bool(data=inside))
        if not inside:
            pub.publish(msg)

    rospy.Subscriber("/gps_raw", GPSMessage, on_gps, queue_size=20)
    active_pub.publish(Bool(data=False))
    rate = rospy.Rate(5)
    tick = 0
    while not rospy.is_shutdown():
        tick += 1
        if tick % 5 == 0:
            enabled = bool(rospy.get_param("~enabled", False))
            if enabled != state["enabled"]:
                state["enabled"] = enabled
                rospy.logwarn("[gps_blackout] blackout zone %s", "ENABLED" if enabled else "disabled (pass-through)")
        with lock:
            active_pub.publish(Bool(data=state["active"]))
        rate.sleep()


if __name__ == "__main__":
    main()
