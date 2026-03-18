#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
경로 추종 노드 - Pure Pursuit (ROS1 Noetic)
=============================================
- JSON 웨이포인트 파일 로드
- /Ego_topic 현재 위치 수신
- Pure Pursuit으로 조향각 계산
- /ctrl_cmd_0 발행

실행:
  python3 path_follower.py [경로파일] [목표속도(km/h)]
  python3 path_follower.py /tmp/my_path.json 20
"""
import sys
import json
import math
import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus


class PathFollower:
    def __init__(self, path_file, target_vel=20.0):
        self.target_vel  = target_vel
        self.wheelbase   = 2.7        # 차량 축거 [m]
        self.lookahead   = 5.0        # Pure Pursuit 전방주시거리 [m]
        self.max_steer   = 0.6        # 최대 조향각 [rad]

        # 웨이포인트 로드
        with open(path_file) as f:
            data = json.load(f)
        self.waypoints = [(wp['x'], wp['y']) for wp in data['waypoints']]
        rospy.loginfo(f"웨이포인트 {len(self.waypoints)}개 로드: {path_file}")

        self.cur_x   = None
        self.cur_y   = None
        self.cur_yaw = None

        rospy.init_node('path_follower_node')
        self.cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=10)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self._ego_cb)

        self.timer = rospy.Timer(rospy.Duration(0.1), self._control_cb)
        rospy.loginfo(f"경로 추종 시작 - 목표속도: {target_vel} km/h")
        rospy.spin()

    def _ego_cb(self, msg: EgoVehicleStatus):
        self.cur_x = msg.position.x
        self.cur_y = msg.position.y
        hdg_deg    = msg.heading
        # MORAI heading: 0=North, CW → ROS yaw: 0=East, CCW
        self.cur_yaw = math.radians(90.0 - hdg_deg)

    def _control_cb(self, event):
        if self.cur_x is None:
            return

        # 전방주시점 탐색 (Pure Pursuit)
        target = self._find_lookahead()
        if target is None:
            rospy.loginfo("경로 종료 - 정지")
            self._publish(0.0, 0.0)
            return

        # 조향각 계산
        dx = target[0] - self.cur_x
        dy = target[1] - self.cur_y
        alpha = math.atan2(dy, dx) - self.cur_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        dist = math.hypot(dx, dy)
        if dist < 0.01:
            steer = 0.0
        else:
            steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), dist)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        self._publish(self.target_vel, math.degrees(steer))

    def _find_lookahead(self):
        """전방주시거리 내 가장 먼 웨이포인트 반환"""
        best = None
        for (wx, wy) in self.waypoints:
            dist = math.hypot(wx - self.cur_x, wy - self.cur_y)
            if dist >= self.lookahead:
                best = (wx, wy)
                break

        # 모든 포인트가 전방주시거리 안에 있으면 마지막 포인트
        if best is None and self.waypoints:
            last = self.waypoints[-1]
            if math.hypot(last[0] - self.cur_x, last[1] - self.cur_y) < 2.0:
                return None   # 도착
            best = last
        return best

    def _publish(self, vel, steer_deg):
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity  = float(vel)
        cmd.steering  = float(steer_deg)
        cmd.accel     = 0.0
        cmd.brake     = 0.0 if vel > 0 else 1.0
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    path_file  = sys.argv[1] if len(sys.argv) > 1 else '/tmp/waypoints.json'
    target_vel = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    PathFollower(path_file=path_file, target_vel=target_vel)
