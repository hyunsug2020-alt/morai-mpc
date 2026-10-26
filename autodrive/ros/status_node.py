#!/usr/bin/env python3
"""Dashboard status from GPS/IMU localization, control and sensor-fused objects."""
import json
import math
import os
import time
from collections import deque

import rospy
import rosnode
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, ObjectStatusList
from std_msgs.msg import Int32


class Status:
    def __init__(self, path_file, output_dir="/data/autodrive"):
        self.output_dir = output_dir
        self.ego_t, self.cmd_t, self.obj_t = deque(maxlen=200), deque(maxlen=200), deque(maxlen=200)
        self.obj_counts = None
        self.ego, self.cmd = None, None
        self.clients = 0
        self.pts = []
        with open(path_file) as f:
            for line in f:
                p = line.strip().replace(",", " ").split()
                if len(p) >= 2 and not line.startswith("#"):
                    self.pts.append((float(p[0]), float(p[1])))
        self.cum = [0.0]
        for (x0, y0), (x1, y1) in zip(self.pts, self.pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))
        rospy.Subscriber("/localization/ego_status", EgoVehicleStatus, self.on_ego, queue_size=50)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self.on_cmd, queue_size=50)
        rospy.Subscriber("/client_count", Int32, self.on_clients, queue_size=1)
        rospy.Subscriber("/perception/objects", ObjectStatusList, self.on_objects, queue_size=50)

    def on_ego(self, m):
        self.ego_t.append(time.time())
        self.ego = m
        self.write_sample("ego.json", {
            "t": time.time(), "stamp": m.header.stamp.to_sec(), "x": m.position.x, "y": m.position.y,
            "heading_deg": m.heading,
            "speed_kmh": math.hypot(m.velocity.x, m.velocity.y) * 3.6,
            "accel": m.accel, "brake": m.brake, "wheel_angle": m.wheel_angle,
        })

    def on_cmd(self, m):
        self.cmd_t.append(time.time())
        self.cmd = m

    def on_objects(self, m):
        self.obj_t.append(time.time())
        self.obj_counts = {"npc": len(m.npc_list), "pedestrian": len(m.pedestrian_list), "obstacle": len(m.obstacle_list)}
        rows = []
        for kind, items in (("npc", m.npc_list), ("pedestrian", m.pedestrian_list), ("obstacle", m.obstacle_list)):
            for obj in items:
                speed = math.hypot(obj.velocity.x, obj.velocity.y) / 3.6
                yaw = math.radians(obj.heading)
                rows.append([obj.position.x, obj.position.y, max(obj.size.x, obj.size.y),
                             obj.name, kind, speed, obj.size.x, obj.size.y, obj.heading,
                             speed * math.cos(yaw), speed * math.sin(yaw), int(obj.unique_id)])
        stamp = m.header.stamp.to_sec()
        lag = rospy.get_time()-stamp if stamp > 0. else 0.
        if not math.isfinite(stamp) or stamp < 0. or not math.isfinite(lag) or lag < -.05:
            lag = 999.
        self.write_sample("objects.json", {"t": time.time(), "objects": rows,
                                           "source_stamp": stamp,
                                           "source_lag_s": max(0., lag)})

    def write_sample(self, name, data):
        target = os.path.join(self.output_dir, name)
        temporary = target + ".tmp"
        with open(temporary, "w") as stream:
            json.dump(data, stream)
        os.replace(temporary, target)

    def on_clients(self, m):
        self.clients = m.data

    @staticmethod
    def hz(ts):
        now = time.time()
        recent = [t for t in ts if now - t <= 2.0]
        return round(len(recent) / 2.0, 1)

    def snapshot(self, pid_file):
        out_dir = pid_file
        s = {"time": time.time(), "ego_hz": self.hz(self.ego_t), "ctrl_hz": self.hz(self.cmd_t),
             "object_hz": self.hz(self.obj_t), "objects": self.obj_counts,
             "rosbridge_clients": self.clients, "path_points": len(self.pts), "path_length_m": round(self.cum[-1], 1)}
        s["follower_running"] = False
        try:
            pid = int(open(pid_file).read().strip())
            os.kill(pid, 0)
            s["follower_running"] = True
        except (OSError, ValueError):
            pass
        try:  # a surviving node (orphan after a hard kill) still counts as running
            names = rosnode.get_node_names()
            s["follower_nodes"] = [n for n in names if n in ("/path_follower_node", "/hdmap_lane_change")]
            if s["follower_nodes"]:
                s["follower_running"] = True
        except Exception:
            s["follower_nodes"] = []
        try:
            s["follower_mode"] = open(os.path.join(os.path.dirname(out_dir), "follower.mode")).read().strip()
        except OSError:
            s["follower_mode"] = None
        if self.ego is not None:
            e = self.ego
            s["ego"] = {"x": round(e.position.x, 2), "y": round(e.position.y, 2), "heading_deg": round(e.heading, 1),
                        "speed_kmh": round(math.hypot(e.velocity.x, e.velocity.y) * 3.6, 1), "age_s": round(time.time() - self.ego_t[-1], 1)}
            d = [math.hypot(px - e.position.x, py - e.position.y) for px, py in self.pts]
            i = min(range(len(d)), key=d.__getitem__)
            s["path"] = {"nearest_idx": i, "distance_m": round(d[i], 1), "progress_pct": round(100 * self.cum[i] / max(self.cum[-1], 1e-6), 1)}
        if self.cmd is not None:
            c = self.cmd
            s["ctrl"] = {"accel": round(c.accel, 2), "brake": round(c.brake, 2), "steering_rad": round(c.steering, 3), "velocity_kmh": round(c.velocity, 1)}
        return s


def main():
    rospy.init_node("autodrive_status")
    out = rospy.get_param("~output", "/data/autodrive/status.json")
    st = Status(rospy.get_param("~path_file"), os.path.dirname(out))
    pid_file = os.path.join(os.path.dirname(out), "follower.pid")
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        tmp = out + ".tmp"
        with open(tmp, "w") as f:
            json.dump(st.snapshot(pid_file), f)
        os.replace(tmp, out)
        rate.sleep()


if __name__ == "__main__":
    main()
