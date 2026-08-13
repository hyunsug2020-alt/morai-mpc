#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import socket

import rospy
from morai_msgs.msg import GPSMessage


def nmea_coordinate(value, hemisphere):
    if not value:
        raise ValueError("empty NMEA coordinate")
    raw = float(value)
    degrees = int(raw / 100.0)
    minutes = raw - degrees * 100.0
    coordinate = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        coordinate = -coordinate
    return coordinate


class MoraiGpsUDP:
    def __init__(self):
        rospy.init_node("morai_gps_udp")
        self.udp_port = int(rospy.get_param("~gps_port", 9090))
        self.use_sim_time = bool(rospy.get_param("/use_sim_time", False))
        self.east_offset = float(
            rospy.get_param("~east_offset", 302595.0))
        self.north_offset = float(
            rospy.get_param("~north_offset", 4124145.0))
        self.frame_id = rospy.get_param("~frame_id", "gps")
        self.publisher = rospy.Publisher(
            "/gps", GPSMessage, queue_size=10)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("0.0.0.0", self.udp_port))
        self.socket.settimeout(1.0)
        rospy.loginfo(
            "MORAI GPS UDP 수신: port %d, offsets=(%.3f, %.3f)",
            self.udp_port, self.east_offset, self.north_offset)
        self.run()

    @staticmethod
    def parse_gga(sentence):
        fields = sentence.split("*", 1)[0].split(",")
        if len(fields) < 10 or fields[0] not in ("$GPGGA", "$GNGGA"):
            return None

        quality = int(fields[6] or 0)
        latitude = nmea_coordinate(fields[2], fields[3])
        longitude = nmea_coordinate(fields[4], fields[5])
        altitude = float(fields[9] or 0.0)
        if not all(math.isfinite(value) for value in (
                latitude, longitude, altitude)):
            raise ValueError("non-finite GPS value")
        return latitude, longitude, altitude, quality

    def parse_and_publish(self, packet):
        text = packet.decode("ascii", errors="ignore")
        for sentence in text.replace("\x00", "").splitlines():
            sentence = sentence.strip()
            if not sentence.startswith(("$GPGGA", "$GNGGA")):
                continue
            try:
                parsed = self.parse_gga(sentence)
                if parsed is None:
                    continue
                latitude, longitude, altitude, quality = parsed
                message = GPSMessage()
                message.header.stamp = rospy.Time.now()
                if (self.use_sim_time
                        and message.header.stamp == rospy.Time()):
                    rospy.logwarn_throttle(
                        2.0, "GPS waiting for MORAI /clock")
                    return
                message.header.frame_id = self.frame_id
                message.latitude = latitude
                message.longitude = longitude
                message.altitude = altitude
                message.eastOffset = self.east_offset
                message.northOffset = self.north_offset
                message.status = 0 if quality > 0 else -1
                self.publisher.publish(message)
                rospy.loginfo_throttle(
                    1.0, "GPS: lat=%.8f lon=%.8f status=%d",
                    latitude, longitude, message.status)
                return
            except (ValueError, IndexError) as error:
                rospy.logwarn_throttle(
                    1.0, "GPS NMEA 파싱 오류: %s", str(error))

        rospy.logwarn_throttle(
            1.0, "GPS UDP에 GGA 문장이 없음: len=%d", len(packet))

    def run(self):
        while not rospy.is_shutdown():
            try:
                packet, _ = self.socket.recvfrom(8192)
                self.parse_and_publish(packet)
            except socket.timeout:
                continue
            except Exception as error:
                rospy.logwarn_throttle(
                    1.0, "GPS UDP 오류: %s", str(error))


if __name__ == "__main__":
    MoraiGpsUDP()
