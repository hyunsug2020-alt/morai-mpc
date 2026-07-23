#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import socket
import struct
import math
from sensor_msgs.msg import Imu

class MoraiImuUDP:
    def __init__(self):
        rospy.init_node('morai_imu_udp')
        self.UDP_PORT = rospy.get_param('~imu_port', 9091)
        self.pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.UDP_PORT))
        self.sock.settimeout(1.0)

        rospy.loginfo("MORAI IMU UDP 수신: port %d" % self.UDP_PORT)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(4096)
                self.parse_and_publish(data)
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logwarn_throttle(1.0, "IMU 에러: %s" % str(e))

    def parse_and_publish(self, data):
        if not data.startswith(b'#IMUData$'):
            return

        payload = data[9:]
        if len(payload) < 96:
            return

        try:
            vals = struct.unpack_from('<12d', payload)
            # [0],[1]: 패딩
            # [2~5]: 쿼터니언 w,x,y,z
            # [6~8]: 각속도 x,y,z
            # [9~11]: 가속도 x,y,z
            wx = vals[6]
            wy = vals[7]
            wz = vals[8]
            ax = vals[9]
            ay = vals[10]
            az = vals[11]

            msg = Imu()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "imu_link"

            # orientation 무시 (단위 쿼터니언)
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0

            msg.angular_velocity.x = wx
            msg.angular_velocity.y = wy
            msg.angular_velocity.z = wz

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            msg.orientation_covariance[0] = 0.01
            msg.orientation_covariance[4] = 0.01
            msg.orientation_covariance[8] = 0.01
            msg.angular_velocity_covariance[0] = 0.01
            msg.angular_velocity_covariance[4] = 0.01
            msg.angular_velocity_covariance[8] = 0.01
            msg.linear_acceleration_covariance[0] = 0.01
            msg.linear_acceleration_covariance[4] = 0.01
            msg.linear_acceleration_covariance[8] = 0.01

            self.pub.publish(msg)
            rospy.loginfo_throttle(1.0,
                "IMU: az=%.2f wz=%.4f" % (az, wz))

        except Exception as e:
            rospy.logwarn_throttle(1.0, "파싱 에러: %s" % str(e))

if __name__ == '__main__':
    MoraiImuUDP()
