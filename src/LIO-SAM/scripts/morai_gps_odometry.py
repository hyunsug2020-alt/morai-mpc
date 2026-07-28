#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import rospy
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry


def latlon_to_utm(latitude, longitude, zone):
    """Convert WGS84 latitude/longitude to UTM coordinates."""
    semi_major = 6378137.0
    eccentricity_sq = 0.00669438
    scale = 0.9996
    eccentricity_prime_sq = (
        eccentricity_sq / (1.0 - eccentricity_sq))

    latitude_rad = math.radians(latitude)
    longitude_rad = math.radians(longitude)
    longitude_origin = math.radians(
        (zone - 1) * 6 - 180 + 3)

    sin_latitude = math.sin(latitude_rad)
    cos_latitude = math.cos(latitude_rad)
    tan_latitude = math.tan(latitude_rad)
    radius = semi_major / math.sqrt(
        1.0 - eccentricity_sq * sin_latitude * sin_latitude)
    tangent_sq = tan_latitude * tan_latitude
    correction = eccentricity_prime_sq * cos_latitude * cos_latitude
    longitude_delta = cos_latitude * (
        longitude_rad - longitude_origin)

    meridian = semi_major * (
        (1.0 - eccentricity_sq / 4.0
         - 3.0 * eccentricity_sq**2 / 64.0
         - 5.0 * eccentricity_sq**3 / 256.0) * latitude_rad
        - (3.0 * eccentricity_sq / 8.0
           + 3.0 * eccentricity_sq**2 / 32.0
           + 45.0 * eccentricity_sq**3 / 1024.0)
        * math.sin(2.0 * latitude_rad)
        + (15.0 * eccentricity_sq**2 / 256.0
           + 45.0 * eccentricity_sq**3 / 1024.0)
        * math.sin(4.0 * latitude_rad)
        - (35.0 * eccentricity_sq**3 / 3072.0)
        * math.sin(6.0 * latitude_rad)
    )

    easting = scale * radius * (
        longitude_delta
        + (1.0 - tangent_sq + correction)
        * longitude_delta**3 / 6.0
        + (5.0 - 18.0 * tangent_sq + tangent_sq**2
           + 72.0 * correction - 58.0 * eccentricity_prime_sq)
        * longitude_delta**5 / 120.0
    ) + 500000.0

    northing = scale * (
        meridian + radius * tan_latitude * (
            longitude_delta**2 / 2.0
            + (5.0 - tangent_sq + 9.0 * correction
               + 4.0 * correction**2)
            * longitude_delta**4 / 24.0
            + (61.0 - 58.0 * tangent_sq + tangent_sq**2
               + 600.0 * correction
               - 330.0 * eccentricity_prime_sq)
            * longitude_delta**6 / 720.0
        )
    )
    if latitude < 0.0:
        northing += 10000000.0
    return easting, northing


class MoraiGpsOdometry:
    def __init__(self):
        rospy.init_node("morai_gps_odometry")
        self.zone = int(rospy.get_param("~utm_zone", 52))
        self.variance = float(rospy.get_param("~variance", 1.0))
        self.origin = None
        self.publisher = rospy.Publisher(
            "/lio_sam/gps/odometry", Odometry, queue_size=10)
        self.subscriber = rospy.Subscriber(
            "/gps", GPSMessage, self.callback, queue_size=20)
        rospy.loginfo(
            "MORAI GPS -> LIO odometry: zone=%d variance=%.3f",
            self.zone, self.variance)

    def callback(self, message):
        if message.status < 0:
            rospy.logwarn_throttle(
                2.0, "유효하지 않은 GPS fix: status=%d", message.status)
            return
        values = (message.latitude, message.longitude, message.altitude)
        if not all(math.isfinite(value) for value in values):
            return

        easting, northing = latlon_to_utm(
            message.latitude, message.longitude, self.zone)
        if self.origin is None:
            self.origin = (easting, northing, message.altitude)
            rospy.loginfo(
                "LIO GPS 로컬 원점: %.3f, %.3f, %.3f",
                easting, northing, message.altitude)

        output = Odometry()
        output.header = message.header
        output.header.frame_id = "odom"
        output.child_frame_id = "gps"
        output.pose.pose.position.x = easting - self.origin[0]
        output.pose.pose.position.y = northing - self.origin[1]
        output.pose.pose.position.z = message.altitude - self.origin[2]
        output.pose.pose.orientation.w = 1.0
        output.pose.covariance[0] = self.variance
        output.pose.covariance[7] = self.variance
        output.pose.covariance[14] = max(self.variance, 4.0)
        self.publisher.publish(output)


if __name__ == "__main__":
    MoraiGpsOdometry()
    rospy.spin()
