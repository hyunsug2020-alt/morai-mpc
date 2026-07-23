#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def latlon_to_utm(latitude, longitude, zone):
    """Convert WGS84 latitude/longitude to UTM without an external dependency."""
    a = 6378137.0
    ecc_sq = 0.00669438
    k0 = 0.9996
    ecc_prime_sq = ecc_sq / (1.0 - ecc_sq)

    lat = math.radians(latitude)
    lon = math.radians(longitude)
    lon_origin = math.radians((zone - 1) * 6 - 180 + 3)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    tan_lat = math.tan(lat)
    n = a / math.sqrt(1.0 - ecc_sq * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ecc_prime_sq * cos_lat * cos_lat
    aa = cos_lat * (lon - lon_origin)

    m = a * (
        (1.0 - ecc_sq / 4.0 - 3.0 * ecc_sq**2 / 64.0
         - 5.0 * ecc_sq**3 / 256.0) * lat
        - (3.0 * ecc_sq / 8.0 + 3.0 * ecc_sq**2 / 32.0
           + 45.0 * ecc_sq**3 / 1024.0) * math.sin(2.0 * lat)
        + (15.0 * ecc_sq**2 / 256.0
           + 45.0 * ecc_sq**3 / 1024.0) * math.sin(4.0 * lat)
        - (35.0 * ecc_sq**3 / 3072.0) * math.sin(6.0 * lat)
    )

    easting = k0 * n * (
        aa + (1.0 - t + c) * aa**3 / 6.0
        + (5.0 - 18.0 * t + t**2 + 72.0 * c
           - 58.0 * ecc_prime_sq) * aa**5 / 120.0
    ) + 500000.0

    northing = k0 * (
        m + n * tan_lat * (
            aa**2 / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c**2) * aa**4 / 24.0
            + (61.0 - 58.0 * t + t**2 + 600.0 * c
               - 330.0 * ecc_prime_sq) * aa**6 / 720.0
        )
    )
    if latitude < 0.0:
        northing += 10000000.0
    return easting, northing


class ESKFNode:
    """Planar GPS/IMU localization filter for MORAI map coordinates."""

    def __init__(self):
        rospy.init_node("eskf_node")

        self.imu_topic = rospy.get_param("~imu_topic", "/imu")
        self.gps_topic = rospy.get_param("~gps_topic", "/gps")
        self.odom_topic = rospy.get_param("~odom_topic", "/eskf/odom")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.utm_zone = int(rospy.get_param("~utm_zone", 52))
        self.publish_tf = bool(rospy.get_param("~publish_tf", False))
        self.sensor_config_file = rospy.get_param("~sensor_config_file", "")
        self.vehicle_model = rospy.get_param(
            "~vehicle_model", "2023_Hyundai_ioniq5")
        self.wheelbase = float(rospy.get_param("~wheelbase", 3.0))
        self.vehicle_length = float(
            rospy.get_param("~vehicle_length", 4.635))
        self.vehicle_width = float(
            rospy.get_param("~vehicle_width", 1.892))
        self.front_overhang = float(
            rospy.get_param("~front_overhang", 0.845))
        self.rear_overhang = float(
            rospy.get_param("~rear_overhang", 0.790))

        self.gps_pos_variance = float(
            rospy.get_param("~gps_position_variance", 2.0))
        self.gps_vel_variance = float(
            rospy.get_param("~gps_velocity_variance", 1.5))
        self.imu_yaw_variance = float(
            rospy.get_param("~imu_yaw_variance", 0.02))
        self.accel_noise = float(rospy.get_param("~accel_noise", 0.8))
        self.gyro_noise = float(rospy.get_param("~gyro_noise", 0.03))
        self.max_predict_dt = float(rospy.get_param("~max_predict_dt", 0.2))
        self.speed_deadband = float(
            rospy.get_param("~speed_deadband_mps", 0.3))

        self.gps_lever = np.array([
            float(rospy.get_param("~gps_x", 3.232)),
            float(rospy.get_param("~gps_y", 0.037)),
        ])
        self.imu_lever = np.array([
            float(rospy.get_param("~imu_x", 3.423)),
            float(rospy.get_param("~imu_y", 0.012)),
        ])
        self._load_sensor_extrinsics()
        self.gps_from_imu = self.gps_lever - self.imu_lever
        self._validate_vehicle_geometry()

        # State is [imu_map_x, imu_map_y, imu_map_vx, imu_map_vy, ROS_yaw].
        self.x = np.zeros((5, 1), dtype=float)
        self.p = np.diag([4.0, 4.0, 9.0, 9.0, 0.5])
        self.initialized = False
        self.last_predict_stamp = None
        self.last_gps_xy = None
        self.last_gps_stamp = None
        self.last_imu_yaw = None
        self.last_yaw_rate = 0.0

        self.odom_pub = rospy.Publisher(
            self.odom_topic, Odometry, queue_size=10)
        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster() if self.publish_tf else None)
        self.imu_sub = rospy.Subscriber(
            self.imu_topic, Imu, self.imu_callback, queue_size=50)
        self.gps_sub = rospy.Subscriber(
            self.gps_topic, GPSMessage, self.gps_callback, queue_size=10)

        rospy.loginfo(
            "[ESKF] GPS+IMU localization ready: %s + %s -> %s",
            self.gps_topic, self.imu_topic, self.odom_topic)
        rospy.loginfo(
            "[ESKF] sensor lever arms: GPS=(%.3f, %.3f), "
            "IMU=(%.3f, %.3f), source=%s",
            self.gps_lever[0], self.gps_lever[1],
            self.imu_lever[0], self.imu_lever[1],
            self.sensor_config_file or "launch defaults")
        rospy.loginfo(
            "[ESKF] vehicle=%s wheelbase=%.3fm size=%.3fx%.3fm",
            self.vehicle_model, self.wheelbase,
            self.vehicle_length, self.vehicle_width)

    @staticmethod
    def _sensor_xy(sensor_list):
        if not sensor_list:
            raise ValueError("sensor list is empty")
        position = sensor_list[0]["pos"]
        return np.array([float(position["x"]), float(position["y"])])

    def _load_sensor_extrinsics(self):
        if not self.sensor_config_file:
            return
        path = os.path.expanduser(self.sensor_config_file)
        try:
            with open(path, "r", encoding="utf-8-sig") as stream:
                config = json.load(stream)
            self.gps_lever = self._sensor_xy(config.get("GPSList", []))
            self.imu_lever = self._sensor_xy(config.get("IMUList", []))
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn(
                "[ESKF] cannot load sensor config %s: %s; using defaults",
                path, exc)

    def _validate_vehicle_geometry(self):
        expected_length = (
            self.front_overhang + self.wheelbase + self.rear_overhang)
        if abs(expected_length - self.vehicle_length) > 0.05:
            rospy.logwarn(
                "[ESKF] vehicle geometry mismatch: overhangs + wheelbase "
                "= %.3fm, length=%.3fm",
                expected_length, self.vehicle_length)
        lateral_limit = self.vehicle_width * 0.5 + 0.4
        for name, lever in (("GPS", self.gps_lever), ("IMU", self.imu_lever)):
            if abs(lever[1]) > lateral_limit:
                rospy.logwarn(
                    "[ESKF] %s lateral position %.3fm exceeds vehicle envelope",
                    name, lever[1])

    @staticmethod
    def _stamp_or_now(header):
        if header.stamp != rospy.Time():
            return header.stamp
        return rospy.Time.now()

    @staticmethod
    def _valid_quaternion(msg):
        norm_sq = (
            msg.orientation.x**2 + msg.orientation.y**2
            + msg.orientation.z**2 + msg.orientation.w**2)
        return norm_sq > 1e-8

    def _correct(self, z, h, r, expected=None):
        innovation = z - (h @ self.x if expected is None else expected)
        s = h @ self.p @ h.T + r
        try:
            k = self.p @ h.T @ np.linalg.inv(s)
        except np.linalg.LinAlgError:
            rospy.logwarn_throttle(2.0, "[ESKF] correction matrix is singular")
            return
        self.x += k @ innovation
        identity = np.eye(5)
        # Joseph form keeps covariance positive under floating point error.
        ikh = identity - k @ h
        self.p = ikh @ self.p @ ikh.T + k @ r @ k.T

    def gps_callback(self, msg):
        if not (math.isfinite(msg.latitude)
                and math.isfinite(msg.longitude)):
            rospy.logwarn_throttle(2.0, "[ESKF] invalid GPS coordinate")
            return

        utm_x, utm_y = latlon_to_utm(
            msg.latitude, msg.longitude, self.utm_zone)
        map_x = utm_x - msg.eastOffset
        map_y = utm_y - msg.northOffset
        stamp = self._stamp_or_now(msg.header)

        if not self.initialized:
            if self.last_imu_yaw is not None:
                self.x[4, 0] = self.last_imu_yaw
            yaw = float(self.x[4, 0])
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            rel_x, rel_y = self.gps_from_imu
            self.x[0, 0] = map_x - (
                cos_yaw * rel_x - sin_yaw * rel_y)
            self.x[1, 0] = map_y - (
                sin_yaw * rel_x + cos_yaw * rel_y)
            self.initialized = True
            self.last_predict_stamp = stamp
            self.last_gps_xy = np.array([map_x, map_y])
            self.last_gps_stamp = stamp
            imu_world_x = (
                cos_yaw * self.imu_lever[0]
                - sin_yaw * self.imu_lever[1])
            imu_world_y = (
                sin_yaw * self.imu_lever[0]
                + cos_yaw * self.imu_lever[1])
            rospy.loginfo(
                "[ESKF] initialized base_link=(%.3f, %.3f), yaw=%.2f deg",
                self.x[0, 0] - imu_world_x,
                self.x[1, 0] - imu_world_y,
                math.degrees(self.x[4, 0]))
            self.publish(stamp)
            return

        yaw = float(self.x[4, 0])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rel_x, rel_y = self.gps_from_imu
        rel_world_x = cos_yaw * rel_x - sin_yaw * rel_y
        rel_world_y = sin_yaw * rel_x + cos_yaw * rel_y

        values = [map_x, map_y]
        rows = [
            [1.0, 0.0, 0.0, 0.0, -rel_world_y],
            [0.0, 1.0, 0.0, 0.0, rel_world_x],
        ]
        expected = [
            self.x[0, 0] + rel_world_x,
            self.x[1, 0] + rel_world_y,
        ]
        variances = [self.gps_pos_variance, self.gps_pos_variance]

        if self.last_gps_xy is not None and self.last_gps_stamp is not None:
            gps_dt = (stamp - self.last_gps_stamp).to_sec()
            if 0.05 <= gps_dt <= 2.0:
                gps_velocity = (
                    np.array([map_x, map_y]) - self.last_gps_xy) / gps_dt
                if np.linalg.norm(gps_velocity) < 80.0:
                    # Remove velocity caused only by rotating the GPS lever arm.
                    gps_velocity[0] += self.last_yaw_rate * rel_world_y
                    gps_velocity[1] -= self.last_yaw_rate * rel_world_x
                    values.extend(gps_velocity.tolist())
                    rows.extend([
                        [0.0, 0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0, 0.0],
                    ])
                    expected.extend([self.x[2, 0], self.x[3, 0]])
                    variances.extend([
                        self.gps_vel_variance, self.gps_vel_variance])

        self._correct(
            np.asarray(values, dtype=float).reshape((-1, 1)),
            np.asarray(rows, dtype=float),
            np.diag(variances),
            np.asarray(expected, dtype=float).reshape((-1, 1)))
        self.last_gps_xy = np.array([map_x, map_y])
        self.last_gps_stamp = stamp
        self.publish(stamp)

    def imu_callback(self, msg):
        stamp = self._stamp_or_now(msg.header)
        yaw_measurement = None
        if self._valid_quaternion(msg):
            quaternion = [
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w,
            ]
            _, _, yaw_measurement = euler_from_quaternion(quaternion)
            yaw_measurement = wrap_angle(yaw_measurement)
            self.last_imu_yaw = yaw_measurement

        if not self.initialized:
            return

        if self.last_predict_stamp is None:
            self.last_predict_stamp = stamp
            return
        dt = (stamp - self.last_predict_stamp).to_sec()
        self.last_predict_stamp = stamp
        if dt <= 0.0:
            return
        dt = min(dt, self.max_predict_dt)

        yaw = float(self.x[4, 0])
        ax_body = float(msg.linear_acceleration.x)
        ay_body = float(msg.linear_acceleration.y)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        ax_map = cos_yaw * ax_body - sin_yaw * ay_body
        ay_map = sin_yaw * ax_body + cos_yaw * ay_body
        yaw_rate = float(msg.angular_velocity.z)
        self.last_yaw_rate = yaw_rate

        self.x[0, 0] += self.x[2, 0] * dt + 0.5 * ax_map * dt * dt
        self.x[1, 0] += self.x[3, 0] * dt + 0.5 * ay_map * dt * dt
        self.x[2, 0] += ax_map * dt
        self.x[3, 0] += ay_map * dt
        self.x[4, 0] = wrap_angle(self.x[4, 0] + yaw_rate * dt)

        f = np.eye(5)
        f[0, 2] = dt
        f[1, 3] = dt
        f[2, 4] = (-sin_yaw * ax_body - cos_yaw * ay_body) * dt
        f[3, 4] = (cos_yaw * ax_body - sin_yaw * ay_body) * dt
        process_var = np.diag([
            0.25 * self.accel_noise * dt**4,
            0.25 * self.accel_noise * dt**4,
            self.accel_noise * dt**2,
            self.accel_noise * dt**2,
            self.gyro_noise * dt**2,
        ])
        self.p = f @ self.p @ f.T + process_var

        if yaw_measurement is not None:
            h = np.zeros((1, 5))
            h[0, 4] = 1.0
            innovation = wrap_angle(yaw_measurement - self.x[4, 0])
            s = float(self.p[4, 4] + self.imu_yaw_variance)
            k = self.p[:, 4:5] / s
            self.x += k * innovation
            self.x[4, 0] = wrap_angle(self.x[4, 0])
            identity = np.eye(5)
            ikh = identity - k @ h
            self.p = (
                ikh @ self.p @ ikh.T
                + k * self.imu_yaw_variance @ k.T)

        self.publish(stamp)

    def publish(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        quaternion = quaternion_from_euler(0.0, 0.0, float(self.x[4, 0]))
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Internal velocity is map-frame. Odometry twist is expressed in base_link.
        yaw = float(self.x[4, 0])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        imu_world_x = (
            cos_yaw * self.imu_lever[0]
            - sin_yaw * self.imu_lever[1])
        imu_world_y = (
            sin_yaw * self.imu_lever[0]
            + cos_yaw * self.imu_lever[1])
        odom.pose.pose.position.x = float(self.x[0, 0]) - imu_world_x
        odom.pose.pose.position.y = float(self.x[1, 0]) - imu_world_y

        vx_map = float(self.x[2, 0])
        vy_map = float(self.x[3, 0])
        # State velocity is at the IMU; publish velocity at base_link.
        vx_map += self.last_yaw_rate * imu_world_y
        vy_map -= self.last_yaw_rate * imu_world_x
        if math.hypot(vx_map, vy_map) < self.speed_deadband:
            vx_map = 0.0
            vy_map = 0.0
        odom.twist.twist.linear.x = cos_yaw * vx_map + sin_yaw * vy_map
        odom.twist.twist.linear.y = -sin_yaw * vx_map + cos_yaw * vy_map
        odom.twist.twist.angular.z = self.last_yaw_rate

        odom.pose.covariance[0] = float(self.p[0, 0])
        odom.pose.covariance[1] = float(self.p[0, 1])
        odom.pose.covariance[6] = float(self.p[1, 0])
        odom.pose.covariance[7] = float(self.p[1, 1])
        odom.pose.covariance[35] = float(self.p[4, 4])
        odom.twist.covariance[0] = float(self.p[2, 2])
        odom.twist.covariance[7] = float(self.p[3, 3])
        odom.twist.covariance[35] = self.gyro_noise
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    try:
        ESKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
