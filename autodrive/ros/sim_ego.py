#!/usr/bin/env python3
"""Stand-in for MORAI: kinematic bicycle that consumes /ctrl_cmd and publishes /Ego_topic at 30 Hz.

Only for verifying the follower + recorder loop without the simulator. Conventions mirror what the
follower emits (CtrlCmd.steering in radians — MORAI convention, saturates at 35deg=0.611; longlCmdType 2 = velocity km/h) and what MORAI publishes
(heading in degrees, velocity in m/s body-frame). Also serves /Service_MoraiEventCmd so gear init succeeds.
    docker exec -i morai_autodrive /opt/autodrive/ros/entrypoint.sh python3 -u - --duration 90 < autodrive/ros/sim_ego.py
"""
import argparse
import math
import sys
import time

import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvResponse


class Sim:
    def __init__(self, x, y, yaw_deg, wheelbase):
        self.x, self.y, self.yaw, self.v = x, y, math.radians(yaw_deg), 0.0
        self.L = wheelbase
        self.cmd = None
        self.cmd_time = 0.0
        self.gear_calls = 0

    def on_cmd(self, m):
        self.cmd, self.cmd_time = m, time.time()

    def on_event(self, req):
        self.gear_calls += 1
        rospy.loginfo("sim: event cmd option=%d ctrl_mode=%d gear=%d", req.request.option, req.request.ctrl_mode, req.request.gear)
        return MoraiEventCmdSrvResponse(response=req.request)

    def step(self, dt):
        target = 0.0
        steer = 0.0
        if self.cmd is not None and time.time() - self.cmd_time < 0.5:
            steer = max(-0.7, min(0.7, self.cmd.steering))  # radians
            if self.cmd.longlCmdType == 2:
                target = max(0.0, self.cmd.velocity) / 3.6
            else:
                target = self.v + (self.cmd.accel * 3.0 - self.cmd.brake * 6.0) * dt
        dv = max(-6.0 * dt, min(2.5 * dt, target - self.v))
        self.v = max(0.0, self.v + dv)
        self.yaw += self.v / self.L * math.tan(steer) * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

    def msg(self):
        m = EgoVehicleStatus()
        m.header.stamp = rospy.Time.now()
        m.position.x, m.position.y, m.position.z = self.x, self.y, 28.5
        m.heading = math.degrees(math.atan2(math.sin(self.yaw), math.cos(self.yaw)))
        m.velocity.x = self.v
        m.gear = 4
        m.ctrl_mode = 3
        return m


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=120)
    p.add_argument("--path-file", default="/data/autodrive/global_path.txt")
    p.add_argument("--wheelbase", type=float, default=3.0)
    a = p.parse_args()
    pts = []
    with open(a.path_file) as f:
        for line in f:
            q = line.strip().replace(",", " ").split()
            if len(q) >= 2 and not line.startswith("#"):
                pts.append((float(q[0]), float(q[1])))
    yaw0 = math.degrees(math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]))
    rospy.init_node("sim_ego", disable_signals=True)
    sim = Sim(pts[0][0], pts[0][1], yaw0, a.wheelbase)
    rospy.Subscriber("/ctrl_cmd", CtrlCmd, sim.on_cmd, queue_size=5)
    rospy.Service("/Service_MoraiEventCmd", MoraiEventCmdSrv, sim.on_event)
    pub = rospy.Publisher("/Ego_topic", EgoVehicleStatus, queue_size=5)
    rate = rospy.Rate(30)
    t0 = time.time()
    last = t0
    n = 0
    print(f"sim_ego: start ({sim.x:.1f},{sim.y:.1f}) yaw {yaw0:.1f} deg, {len(pts)} path points", file=sys.stderr, flush=True)
    while not rospy.is_shutdown() and time.time() - t0 < a.duration:
        now = time.time()
        sim.step(min(now - last, 0.1))
        last = now
        pub.publish(sim.msg())
        n += 1
        if n % 150 == 0:
            d = min(math.hypot(px - sim.x, py - sim.y) for px, py in pts[::4])
            print(f"sim_ego: t={now - t0:5.1f}s v={sim.v * 3.6:5.1f} km/h pos=({sim.x:.1f},{sim.y:.1f}) off-path≈{d:.2f} m gear_calls={sim.gear_calls}", file=sys.stderr, flush=True)
        rate.sleep()
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
