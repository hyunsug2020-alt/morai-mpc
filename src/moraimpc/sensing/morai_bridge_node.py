#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI 토픽 브릿지 - Ioniq 5
==============================
MORAI → 시스템 변환:
  /Ego_topic  → /Ego_pose, /odometry/filtered, /ublox_gps/fix_velocity
  /imu        → /handsfree/imu  (패스스루)
  /gps        → /gnss_rover/fix (패스스루)
  /image_raw  → /camera/image_raw (패스스루)

시스템 → MORAI:
  /Accel OR /cmd_vel → /ctrl_cmd

MORAI EgoVehicleStatus 좌표계:
  heading: 0=North, 시계방향 [deg]
  position_x/y: UTM [m]
  velocity_x: 전진 방향 속도 [m/s]
"""
from __future__ import annotations
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist, Accel, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, Image
from std_msgs.msg import Float64

try:
    from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
    _HAS_MORAI = True
except ImportError:
    _HAS_MORAI = False


class MoraiBridgeNode(Node):

    def __init__(self):
        super().__init__('morai_bridge_node')

        self.declare_parameter('wheelbase',      3.0)
        self.declare_parameter('max_steer_deg', 35.0)
        self.declare_parameter('use_accel_cmd',  True)

        self.wb       = float(self.get_parameter('wheelbase').value)
        self.max_st   = math.radians(float(self.get_parameter('max_steer_deg').value))
        self.kap_max  = math.tan(self.max_st) / self.wb
        self.use_accel= bool(self.get_parameter('use_accel_cmd').value)
        self._cur_v   = 0.0
        self._cur_yaw = 0.0

        s_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=10)
        r_qos = QoSProfile(depth=10)

        # 구독
        if _HAS_MORAI:
            self.create_subscription(EgoVehicleStatus, '/Ego_topic', self._ego_cb, r_qos)
        self.create_subscription(Imu,      '/imu',       self._imu_cb, s_qos)
        self.create_subscription(NavSatFix,'/gps',       self._gps_cb, s_qos)
        self.create_subscription(Image,    '/image_raw', self._cam_cb, s_qos)
        self.create_subscription(Accel,    '/Accel',     self._accel_cb, r_qos)
        self.create_subscription(Twist,    '/cmd_vel',   self._twist_cb, r_qos)

        # 발행
        self.pose_pub = self.create_publisher(PoseStamped, '/Ego_pose', 10)
        self.odom_pub = self.create_publisher(Odometry,    '/odometry/filtered', 10)
        self.vel_pub  = self.create_publisher(TwistWithCovarianceStamped, '/ublox_gps/fix_velocity', 10)
        self.hdg_pub  = self.create_publisher(Float64,     '/morai/heading_deg', 10)
        self.imu_pub  = self.create_publisher(Imu,         '/handsfree/imu', 10)
        self.gps_pub  = self.create_publisher(NavSatFix,   '/gnss_rover/fix', 10)
        self.cam_pub  = self.create_publisher(Image,       '/camera/image_raw', 10)
        if _HAS_MORAI:
            self.ctrl_pub = self.create_publisher(CtrlCmd, '/ctrl_cmd', 10)

        self.get_logger().info(f'MORAI 브릿지 시작 (morai_msgs={"OK" if _HAS_MORAI else "없음"})')

    def _ego_cb(self, msg) -> None:
        now = self.get_clock().now().to_msg()

        # 헤딩 변환: MORAI(0=North,CW) → ROS(0=East,CCW)
        hdg_deg = float(getattr(msg, 'heading', 0.0))
        ros_yaw = math.radians(90.0 - hdg_deg)
        ros_yaw = math.atan2(math.sin(ros_yaw), math.cos(ros_yaw))
        self._cur_yaw = ros_yaw

        px = float(getattr(msg, 'position_x', 0.0))
        py = float(getattr(msg, 'position_y', 0.0))
        pz = float(getattr(msg, 'position_z', 0.0))
        vx = float(getattr(msg, 'velocity_x', 0.0))
        vy = float(getattr(msg, 'velocity_y', 0.0))
        wz = float(getattr(msg, 'angular_velocity_z', 0.0))
        self._cur_v = vx

        qz = math.sin(ros_yaw / 2)
        qw = math.cos(ros_yaw / 2)

        # PoseStamped
        ps = PoseStamped()
        ps.header.stamp = now; ps.header.frame_id = 'map'
        ps.pose.position.x = px; ps.pose.position.y = py; ps.pose.position.z = pz
        ps.pose.orientation.z = qz; ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        # Odometry
        od = Odometry()
        od.header = ps.header; od.child_frame_id = 'base_link'
        od.pose.pose = ps.pose
        od.twist.twist.linear.x = vx; od.twist.twist.linear.y = vy
        od.twist.twist.angular.z = wz
        self.odom_pub.publish(od)

        # GPS 속도 (ESKF 입력)
        vm = TwistWithCovarianceStamped()
        vm.header = ps.header
        vm.twist.twist.linear.x = vx; vm.twist.twist.linear.y = vy
        vm.twist.covariance[0] = 0.04; vm.twist.covariance[7] = 0.04
        self.vel_pub.publish(vm)

        # 헤딩
        h = Float64(); h.data = math.degrees(ros_yaw)
        self.hdg_pub.publish(h)

    def _imu_cb(self, msg: Imu)      -> None: self.imu_pub.publish(msg)
    def _gps_cb(self, msg: NavSatFix) -> None: self.gps_pub.publish(msg)
    def _cam_cb(self, msg: Image)    -> None: self.cam_pub.publish(msg)

    def _accel_cb(self, msg: Accel) -> None:
        if not _HAS_MORAI: return
        steer = self._omega_to_steer(msg.angular.z)
        self._send_ctrl(float(msg.linear.x), steer)

    def _twist_cb(self, msg: Twist) -> None:
        if not _HAS_MORAI or self.use_accel: return
        accel = (float(msg.linear.x) - self._cur_v) * 2.0
        steer = self._omega_to_steer(float(msg.angular.z))
        self._send_ctrl(accel, steer)

    def _omega_to_steer(self, omega: float) -> float:
        v = self._cur_v if abs(self._cur_v) > 0.1 else 0.1
        kappa = max(-self.kap_max, min(self.kap_max, omega / v))
        return max(-self.max_st, min(self.max_st, math.atan(kappa * self.wb)))

    def _send_ctrl(self, lon_acc: float, steer: float) -> None:
        c = CtrlCmd()
        c.longi_accel = float(lon_acc)
        c.steering    = float(steer)
        c.brake       = max(0.0, -lon_acc * 0.3) if lon_acc < 0 else 0.0
        self.ctrl_pub.publish(c)


def main(args=None):
    rclpy.init(args=args)
    node = MoraiBridgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
