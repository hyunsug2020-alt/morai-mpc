#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
경로 생성 노드 (ROS1 Noetic)
==============================
- 수동 주행 중 /Ego_topic에서 위치 기록
- 일정 거리 간격으로 웨이포인트 저장
- Ctrl+C 시 JSON 파일로 저장

실행:
  python3 path_maker.py [저장파일경로]
  python3 path_maker.py /tmp/my_path.json
"""
import sys
import json
import math
import signal
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class PathMaker:
    def __init__(self, save_path='/tmp/waypoints.json', min_dist=0.5):
        self.save_path = save_path
        self.min_dist  = min_dist   # 최소 기록 간격 [m]
        self.waypoints = []
        self.last_x    = None
        self.last_y    = None

        rospy.init_node('path_maker_node')
        # /Ego_topic 대신 ESKF의 결과인 /eskf/odom을 구독
        rospy.Subscriber('/eskf/odom', Odometry, self._odom_cb)

        # Ctrl+C 시 저장
        signal.signal(signal.SIGINT, self._save_and_exit)

        rospy.loginfo(f"ESKF 기반 경로 기록 시작 - 간격: {min_dist}m, 저장: {save_path}")
        rospy.loginfo("차량을 수동으로 주행하세요. Ctrl+C로 저장 종료.")
        rospy.spin()

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, heading = euler_from_quaternion(q)

        if self.last_x is None:
            self._record(x, y, heading)
            return

        dist = math.hypot(x - self.last_x, y - self.last_y)
        if dist >= self.min_dist:
            self._record(x, y, heading)

    def _record(self, x, y, heading):
        self.last_x = x
        self.last_y = y
        self.waypoints.append({'x': x, 'y': y, 'heading': heading})
        rospy.loginfo(f"[{len(self.waypoints):4d}] x={x:.2f}, y={y:.2f}, hdg={heading:.1f}")

    def _save_and_exit(self, sig, frame):
        with open(self.save_path, 'w') as f:
            json.dump({'waypoints': self.waypoints}, f, indent=2)
        rospy.loginfo(f"\n저장 완료: {self.save_path} ({len(self.waypoints)} 포인트)")
        rospy.signal_shutdown('저장 완료')
        sys.exit(0)


if __name__ == '__main__':
    save_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/waypoints.json'
    PathMaker(save_path=save_path, min_dist=0.5)
