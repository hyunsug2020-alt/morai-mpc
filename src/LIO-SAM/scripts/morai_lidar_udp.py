#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import socket
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

# VLP-16 레이저 수직 각도
LASER_ANGLES = [
    -15, 1, -13, 3, -11, 5, -9, 7,
    -7, 9, -5, 11, -3, 13, -1, 15
]
# 수직각도 순서대로 ring 정렬 (-15도=0, +15도=15)
RING_MAP = {
    -15: 0, -13: 1, -11: 2, -9: 3, -7: 4, -5: 5, -3: 6, -1: 7,
    1: 8, 3: 9, 5: 10, 7: 11, 9: 12, 11: 13, 13: 14, 15: 15
}
DISTANCE_RESOLUTION = 0.002  # 2mm

class MoraiLidarUDP:
    def __init__(self):
        rospy.init_node('morai_lidar_udp')
        self.UDP_PORT = rospy.get_param('~lidar_port', 2368)
        self.pub = rospy.Publisher(
            '/velodyne_points', PointCloud2, queue_size=1)

        self.sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.UDP_PORT))
        self.sock.settimeout(1.0)

        self.points_buf = []
        self.packet_count = 0
        rospy.loginfo("Velodyne UDP 수신: port %d" % self.UDP_PORT)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(65535)
                if len(data) == 1206:
                    self.parse_velodyne(data)
                    self.packet_count += 1
                    # VLP-16: 한 바퀴 = 약 75 패킷
                    if self.packet_count >= 75:
                        if self.points_buf:
                            self.publish(self.points_buf)
                            self.points_buf = []
                        self.packet_count = 0
            except socket.timeout:
                if self.points_buf:
                    self.publish(self.points_buf)
                    self.points_buf = []
                self.packet_count = 0

    def parse_velodyne(self, data):
        points = []
        for block in range(12):
            offset = block * 100
            if offset + 4 > len(data):
                break
            flag = struct.unpack_from('<H', data, offset)[0]
            if flag != 0xEEFF:
                continue
            azimuth = struct.unpack_from('<H', data, offset+2)[0] / 100.0

            for laser_id in range(16):
                ch_offset = offset + 4 + laser_id * 3
                if ch_offset + 3 > len(data):
                    break
                distance = struct.unpack_from('<H', data, ch_offset)[0] \
                           * DISTANCE_RESOLUTION
                intensity = data[ch_offset + 2]

                if distance < 0.1:
                    continue

                vert_deg = LASER_ANGLES[laser_id]
                vert_angle = math.radians(vert_deg)
                horiz_angle = math.radians(azimuth)

                x = distance * math.cos(vert_angle) * math.cos(horiz_angle)
                y = distance * math.cos(vert_angle) * math.sin(horiz_angle)
                z = distance * math.sin(vert_angle)

                # 수직각도 순서대로 ring 정렬
                ring = RING_MAP[vert_deg]
                # deskew 비활성화 (time=0)
                time_offset = 0.0

                points.append([x, y, z, float(intensity),
                               ring, time_offset])

        self.points_buf.extend(points)

    def publish(self, points):
        if not points:
            return
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        fields = [
            PointField('x',         0,  PointField.FLOAT32, 1),
            PointField('y',         4,  PointField.FLOAT32, 1),
            PointField('z',         8,  PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('ring',      16, PointField.UINT16,  1),
            PointField('time',      20, PointField.FLOAT32, 1),
        ]
        cloud = pc2.create_cloud(header, fields, points)
        cloud.is_dense = True
        self.pub.publish(cloud)
        rospy.loginfo_throttle(1.0, "퍼블리시: %d포인트" % len(points))

if __name__ == '__main__':
    MoraiLidarUDP()
