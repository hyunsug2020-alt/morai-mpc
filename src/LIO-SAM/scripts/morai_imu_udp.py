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
        self.use_sim_time = bool(rospy.get_param('/use_sim_time', False))
        self.pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.last_sensor_stamp_ns = None

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
            rospy.logwarn_throttle(
                1.0, "IMU UDP 헤더 불일치: len=%d header=%r"
                % (len(data), data[:16]))
            return

        if len(data) < 95:
            rospy.logwarn_throttle(
                1.0, "IMU UDP 패킷이 너무 짧음: len=%d" % len(data))
            return

        try:
            data_length = struct.unpack_from('<I', data, 9)[0]
            if data_length < 80:
                rospy.logwarn_throttle(
                    1.0, "IMU UDP data_length 오류: %d" % data_length)
                return

            # MORAI 24.R2.2 packet:
            #   header(9) + size(4) + aux(12) + imu(80) + CRLF(2) = 107
            # Current MORAI versions prepend a ROS timestamp to the IMU block:
            #   header(9) + size(4) + aux(12) + stamp(8) + imu(80)
            #   + CRLF(2) = 115, and report size=88.
            # In both variants the actual IMU block is the final 80 bytes
            # immediately before the packet tail.
            tail_size = 2 if data.endswith(b'\r\n') else 0
            data_start = len(data) - tail_size - 80
            if data_start < 13 or data_start + 80 > len(data) - tail_size:
                rospy.logwarn_throttle(
                    1.0, "IMU UDP 구조 오류: len=%d data_length=%d"
                    % (len(data), data_length))
                return

            vals = struct.unpack_from('<10d', data, data_start)
            # MORAI order: quaternion w,x,y,z; angular velocity x,y,z;
            # linear acceleration x,y,z.
            qw, qx, qy, qz = vals[0:4]
            wx, wy, wz = vals[4:7]
            ax, ay, az = vals[7:10]

            msg = Imu()
            if self.use_sim_time:
                # MORAI keeps the IMU packet's embedded stamp on wall time
                # even in Simulation Time mode.  Ego/GPS use simulation time,
                # so stamp IMU packets from /clock to keep every sensor in the
                # same time domain.  Do not leak pre-clock packets into ESKF.
                msg.header.stamp = rospy.Time.now()
                if msg.header.stamp == rospy.Time():
                    rospy.logwarn_throttle(
                        2.0, "IMU waiting for MORAI /clock")
                    return
            else:
                # The 115-byte packet stores uint32 seconds and nanoseconds
                # directly before the IMU block. Preserve it in wall-time mode.
                stamp_start = data_start - 8
                if data_length >= 88 and stamp_start >= 25:
                    stamp_sec, stamp_nsec = struct.unpack_from(
                        '<II', data, stamp_start)
                    if stamp_sec > 0 and stamp_nsec < 1000000000:
                        msg.header.stamp = rospy.Time(stamp_sec, stamp_nsec)
                    else:
                        msg.header.stamp = rospy.Time.now()
                else:
                    msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "imu_link"

            stamp_ns = msg.header.stamp.to_nsec()
            if (self.last_sensor_stamp_ns is not None
                    and stamp_ns <= self.last_sensor_stamp_ns):
                # MORAI resets its timestamp to zero when Time Manager is
                # changed from Real Time to Simulation Time.  Treat a large
                # backwards jump as a new clock epoch; otherwise every IMU
                # packet after the mode change would be rejected forever.
                if stamp_ns < self.last_sensor_stamp_ns - 5_000_000_000:
                    rospy.logwarn(
                        "IMU simulation-time reset: current=%d last=%d"
                        % (stamp_ns, self.last_sensor_stamp_ns))
                    self.last_sensor_stamp_ns = None
                else:
                    rospy.logwarn_throttle(
                        2.0, "IMU 중복/역행 timestamp 제거: current=%d last=%d"
                        % (stamp_ns, self.last_sensor_stamp_ns))
                    return
            self.last_sensor_stamp_ns = stamp_ns

            quaternion_norm = math.sqrt(
                qw * qw + qx * qx + qy * qy + qz * qz)
            if quaternion_norm > 1e-8:
                msg.orientation.w = qw / quaternion_norm
                msg.orientation.x = qx / quaternion_norm
                msg.orientation.y = qy / quaternion_norm
                msg.orientation.z = qz / quaternion_norm
            else:
                msg.orientation.w = 1.0
                msg.orientation_covariance[0] = -1.0

            msg.angular_velocity.x = wx
            msg.angular_velocity.y = wy
            msg.angular_velocity.z = wz

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            if msg.orientation_covariance[0] >= 0.0:
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
                "IMU: |a|=%.3f az=%.3f wz=%.5f"
                % (math.sqrt(ax * ax + ay * ay + az * az), az, wz))

        except Exception as e:
            rospy.logwarn_throttle(1.0, "파싱 에러: %s" % str(e))

if __name__ == '__main__':
    MoraiImuUDP()
