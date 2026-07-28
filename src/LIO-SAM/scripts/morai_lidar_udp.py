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
VELODYNE_PACKET_SIZE = 1206


class MoraiLidarUDP:
    def __init__(self):
        rospy.init_node('morai_lidar_udp')
        self.UDP_PORT = rospy.get_param('~lidar_port', 2368)
        self.scan_period = float(rospy.get_param('~scan_period', 0.1))
        self.max_packets_per_scan = int(
            rospy.get_param('~max_packets_per_scan', 90))
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
        self.last_azimuth = None
        rospy.loginfo("Velodyne UDP 수신: port %d" % self.UDP_PORT)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(65535)
                packet = self.extract_velodyne_packet(data)
                if packet is None:
                    rospy.logwarn_throttle(
                        2.0, "지원하지 않는 LiDAR UDP 크기: %d" % len(data))
                    continue
                self.parse_velodyne(packet)
                self.packet_count += 1
                # Azimuth wrap가 누락된 비표준 스트림에서도 버퍼가 무한히
                # 커지지 않도록 약 한 바퀴 분량에서 강제로 발행한다.
                if self.packet_count >= self.max_packets_per_scan:
                    self.publish_scan()
            except socket.timeout:
                self.publish_scan()
            except OSError as exc:
                if not rospy.is_shutdown():
                    rospy.logwarn_throttle(2.0, "LiDAR UDP 오류: %s" % exc)

    @staticmethod
    def extract_velodyne_packet(data):
        """Accept a raw packet or MORAI's optional auxiliary UDP header."""
        if len(data) == VELODYNE_PACKET_SIZE:
            return data
        search_end = min(len(data), 64)
        packet_start = data.find(b'\xff\xee', 0, search_end)
        packet_end = packet_start + VELODYNE_PACKET_SIZE
        if packet_start >= 0 and packet_end <= len(data):
            return data[packet_start:packet_end]
        return None

    def parse_velodyne(self, data):
        azimuths = []
        for block in range(12):
            offset = block * 100
            flag = struct.unpack_from('<H', data, offset)[0]
            if flag != 0xEEFF:
                azimuths.append(None)
            else:
                azimuths.append(
                    struct.unpack_from('<H', data, offset + 2)[0] / 100.0)

        previous_delta = 0.2
        for block, azimuth in enumerate(azimuths):
            if azimuth is None:
                continue
            next_azimuth = (
                azimuths[block + 1] if block + 1 < len(azimuths) else None)
            if next_azimuth is not None:
                delta = (next_azimuth - azimuth) % 360.0
                if delta <= 5.0:
                    previous_delta = delta
            delta = previous_delta

            # A VLP-16 block contains two firings of all 16 lasers.
            for firing in range(2):
                firing_azimuth = (
                    azimuth + 0.5 * firing * delta) % 360.0
                if (self.last_azimuth is not None
                        and self.last_azimuth > 300.0
                        and firing_azimuth < 60.0):
                    self.publish_scan()
                self.last_azimuth = firing_azimuth

                for laser_id in range(16):
                    channel = firing * 16 + laser_id
                    ch_offset = block * 100 + 4 + channel * 3
                    distance = (
                        struct.unpack_from('<H', data, ch_offset)[0]
                        * DISTANCE_RESOLUTION)
                    intensity = data[ch_offset + 2]

                    if distance < 0.1:
                        continue

                    vert_deg = LASER_ANGLES[laser_id]
                    vert_angle = math.radians(vert_deg)
                    horiz_angle = math.radians(firing_azimuth)

                    x = (
                        distance * math.cos(vert_angle)
                        * math.cos(horiz_angle))
                    y = (
                        distance * math.cos(vert_angle)
                        * math.sin(horiz_angle))
                    z = distance * math.sin(vert_angle)
                    ring = RING_MAP[vert_deg]
                    self.points_buf.append(
                        [x, y, z, float(intensity), ring])

    def publish_scan(self):
        if self.points_buf:
            self.publish(self.points_buf)
        self.points_buf = []
        self.packet_count = 0

    def publish(self, points):
        if not points:
            return
        last_index = max(len(points) - 1, 1)
        timed_points = [
            point + [index / last_index * self.scan_period]
            for index, point in enumerate(points)
        ]
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
        cloud = pc2.create_cloud(header, fields, timed_points)
        cloud.is_dense = True
        self.pub.publish(cloud)
        rospy.loginfo_throttle(
            1.0, "LiDAR scan: %d points" % len(timed_points))


if __name__ == '__main__':
    MoraiLidarUDP()
