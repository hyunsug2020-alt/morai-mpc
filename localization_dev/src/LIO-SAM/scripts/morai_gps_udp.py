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
        self.bind_ip = str(rospy.get_param("~bind_ip", "0.0.0.0")).strip()
        self.source_ip = str(rospy.get_param("~source_ip", "")).strip()
        self.use_sim_time = bool(rospy.get_param("/use_sim_time", False))
        self.east_offset = float(
            rospy.get_param("~east_offset", 302595.0))
        self.north_offset = float(
            rospy.get_param("~north_offset", 4124145.0))
        # RMC contains no altitude. XY localization is unaffected; keeping a
        # configurable fixed value also gives downstream messages a finite Z.
        self.rmc_altitude = float(rospy.get_param("~rmc_altitude", 0.0))
        self.frame_id = rospy.get_param("~frame_id", "gps")
        self.publisher = rospy.Publisher(
            "/gps", GPSMessage, queue_size=10)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.socket.bind((self.bind_ip, self.udp_port))
        except OSError as error:
            self.socket.close()
            rospy.logfatal(
                "GPS UDP bind %s:%d failed: %s. MORAI GPS Destination "
                "IP/Port와 로컬 NIC 설정 확인 필요.",
                self.bind_ip, self.udp_port, str(error))
            raise
        self.socket.settimeout(1.0)
        rospy.loginfo(
            "MORAI GPS UDP 수신: %s:%d source=%s, offsets=(%.3f, %.3f)",
            self.bind_ip, self.udp_port, self.source_ip or "any",
            self.east_offset, self.north_offset)
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

    @staticmethod
    def parse_rmc(sentence, altitude=0.0):
        fields = sentence.split("*", 1)[0].split(",")
        if len(fields) < 10 or fields[0] not in ("$GPRMC", "$GNRMC"):
            return None

        valid = fields[2] == "A"
        latitude = nmea_coordinate(fields[3], fields[4])
        longitude = nmea_coordinate(fields[5], fields[6])
        if not all(math.isfinite(value) for value in (
                latitude, longitude, altitude)):
            raise ValueError("non-finite GPS value")
        return latitude, longitude, altitude, 1 if valid else 0

    def parse_sentence(self, sentence):
        if sentence.startswith(("$GPGGA", "$GNGGA")):
            return self.parse_gga(sentence)
        if sentence.startswith(("$GPRMC", "$GNRMC")):
            return self.parse_rmc(sentence, self.rmc_altitude)
        return None

    def parse_and_publish(self, packet):
        text = packet.decode("ascii", errors="ignore")
        for sentence in text.replace("\x00", "").splitlines():
            sentence = sentence.strip()
            try:
                parsed = self.parse_sentence(sentence)
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
            1.0, "GPS UDP에 지원되는 GGA/RMC 문장이 없음: len=%d",
            len(packet))

    def run(self):
        while not rospy.is_shutdown():
            try:
                packet, address = self.socket.recvfrom(8192)
                if self.source_ip and address[0] != self.source_ip:
                    rospy.logwarn_throttle(
                        2.0, "GPS unexpected source %s; expected %s",
                        address[0], self.source_ip)
                    continue
                self.parse_and_publish(packet)
            except socket.timeout:
                continue
            except Exception as error:
                rospy.logwarn_throttle(
                    1.0, "GPS UDP 오류: %s", str(error))


if __name__ == "__main__":
    MoraiGpsUDP()
