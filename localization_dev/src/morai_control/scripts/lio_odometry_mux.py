#!/usr/bin/env python3
"""Continuous LIO aiding odometry with ESKF priority and pure-odom fallback."""

import json
import math
import threading

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class LioOdometryMux:
    def __init__(self):
        rospy.init_node("lio_odometry_mux")
        self.primary_topic = rospy.get_param(
            "~primary_topic", "/eskf/odom")
        self.fallback_topic = rospy.get_param(
            "~fallback_topic", "/odometry/pure")
        self.output_topic = rospy.get_param(
            "~output_topic", "/lio/aiding/odometry")
        self.diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "/lio_odometry_mux/diagnostics")
        self.primary_timeout = float(rospy.get_param(
            "~primary_timeout_s", 0.30))
        self.max_jump = float(rospy.get_param(
            "~max_continuous_jump_m", 5.0))
        self.max_yaw_jump = float(rospy.get_param(
            "~max_continuous_yaw_jump_rad", 0.8))
        self.max_header_age = float(rospy.get_param(
            "~max_header_age_s", 1.0))
        self.future_tolerance = float(rospy.get_param(
            "~future_tolerance_s", 0.10))
        self.cycle_guard_param = rospy.get_param(
            "~cycle_guard_param", "/eskf_node/slam_aiding_enabled")
        self.primary_covariance_scale = max(1.0, float(rospy.get_param(
            "~primary_covariance_scale", 4.0)))
        self.primary_position_variance = max(0.0, float(rospy.get_param(
            "~primary_position_variance", 0.0625)))
        self.primary_yaw_variance = max(0.0, float(rospy.get_param(
            "~primary_yaw_variance", 0.01)))
        self.switch_position_variance = max(0.0, float(rospy.get_param(
            "~switch_position_variance", 0.04)))
        self.switch_yaw_variance = max(0.0, float(rospy.get_param(
            "~switch_yaw_variance", 0.01)))
        self.primary_min_samples = max(1, int(rospy.get_param(
            "~primary_min_samples", 3)))
        self.primary_max_position_std = max(0.0, float(rospy.get_param(
            "~primary_max_position_std", 5.0)))
        self.primary_max_yaw_std = max(0.0, float(rospy.get_param(
            "~primary_max_yaw_std", 1.0)))

        self.lock = threading.RLock()
        self.latest = {"eskf": None, "pure": None}
        self.receipt = {"eskf": None, "pure": None}
        self.last_source_stamp = {"eskf": None, "pure": None}
        self.active_source = None
        self.alignment_yaw = 0.0
        self.alignment_x = 0.0
        self.alignment_y = 0.0
        self.last_output = None
        self.last_output_stamp = None
        self.alignment_position_variance = 0.0
        self.alignment_yaw_variance = 0.0
        self.primary_consecutive = 0
        self.published = 0
        self.switches = 0
        self.realignments = 0
        self.nonmonotonic = 0
        self.invalid = 0
        self.stale = 0
        self.unsafe_primary = 0
        self.resets = 0

        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=100)
        self.diagnostics_publisher = rospy.Publisher(
            self.diagnostics_topic, String,
            queue_size=2, latch=True)
        self.primary_subscriber = rospy.Subscriber(
            self.primary_topic, Odometry,
            lambda message: self.callback("eskf", message),
            queue_size=100, tcp_nodelay=True)
        self.fallback_subscriber = rospy.Subscriber(
            self.fallback_topic, Odometry,
            lambda message: self.callback("pure", message),
            queue_size=100, tcp_nodelay=True)
        self.timer = rospy.Timer(
            rospy.Duration(0.5), self.publish_diagnostics)
        rospy.loginfo(
            "[LIO odom mux] primary=%s fallback=%s output=%s",
            self.primary_topic, self.fallback_topic, self.output_topic)

    @staticmethod
    def valid(message):
        pose = message.pose.pose
        quaternion = pose.orientation
        values = (
            message.header.stamp.to_sec(),
            pose.position.x, pose.position.y,
            quaternion.x, quaternion.y,
            quaternion.z, quaternion.w,
        )
        quaternion_norm = math.sqrt(sum(
            value * value for value in values[3:7]))
        return (
            values[0] > 0.0
            and all(math.isfinite(value) for value in values)
            and quaternion_norm > 1.0e-8
            and message.child_frame_id == "base_link")

    def primary_cycle_safe(self):
        if not rospy.has_param(self.cycle_guard_param):
            return False
        return not bool(rospy.get_param(self.cycle_guard_param))

    def primary_fresh(self, now):
        if not self.primary_cycle_safe():
            return False
        if self.primary_consecutive < self.primary_min_samples:
            return False
        receipt = self.receipt["eskf"]
        if receipt is None:
            return False
        return 0.0 <= (now - receipt).to_sec() <= self.primary_timeout

    def timely(self, message, now):
        age = (now - message.header.stamp).to_sec()
        return -self.future_tolerance <= age <= self.max_header_age

    def primary_covariance_valid(self, message):
        covariance = message.pose.covariance
        values = (covariance[0], covariance[7], covariance[35])
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            return False
        position_std = math.sqrt(max(values[0], values[1]))
        yaw_std = math.sqrt(values[2])
        return (position_std <= self.primary_max_position_std
                and yaw_std <= self.primary_max_yaw_std)

    def reset_stream_state(self):
        self.latest = {"eskf": None, "pure": None}
        self.receipt = {"eskf": None, "pure": None}
        self.last_source_stamp = {"eskf": None, "pure": None}
        self.active_source = None
        self.last_output = None
        self.last_output_stamp = None
        self.alignment_yaw = 0.0
        self.alignment_x = 0.0
        self.alignment_y = 0.0
        self.alignment_position_variance = 0.0
        self.alignment_yaw_variance = 0.0
        self.primary_consecutive = 0
        self.resets += 1

    @staticmethod
    def source_pose(message):
        return (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw_from_quaternion(message.pose.pose.orientation),
        )

    def align_source(self, source, message):
        x, y, yaw = self.source_pose(message)
        if self.last_output is None:
            self.alignment_yaw = 0.0
            self.alignment_x = 0.0
            self.alignment_y = 0.0
        else:
            output_x, output_y, output_yaw = self.last_output
            self.alignment_yaw = wrap_angle(output_yaw - yaw)
            cosine = math.cos(self.alignment_yaw)
            sine = math.sin(self.alignment_yaw)
            self.alignment_x = output_x - (cosine * x - sine * y)
            self.alignment_y = output_y - (sine * x + cosine * y)
        if self.active_source is not None and self.active_source != source:
            self.switches += 1
            self.alignment_position_variance = self.switch_position_variance
            self.alignment_yaw_variance = self.switch_yaw_variance
        self.active_source = source

    def transformed_pose(self, message):
        x, y, yaw = self.source_pose(message)
        cosine = math.cos(self.alignment_yaw)
        sine = math.sin(self.alignment_yaw)
        return (
            self.alignment_x + cosine * x - sine * y,
            self.alignment_y + sine * x + cosine * y,
            wrap_angle(yaw + self.alignment_yaw),
        )

    def transformed_covariance(self, source, covariance):
        # Odometry pose covariance order: x, y, z, roll, pitch, yaw. Rotate
        # both vector blocks into lio_aiding_odom while preserving cross terms.
        cosine = math.cos(self.alignment_yaw)
        sine = math.sin(self.alignment_yaw)
        transform = [
            [cosine, -sine, 0.0, 0.0, 0.0, 0.0],
            [sine, cosine, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, cosine, -sine, 0.0],
            [0.0, 0.0, 0.0, sine, cosine, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]
        matrix = [
            [value if math.isfinite(value) else 0.0
             for value in covariance[row * 6:(row + 1) * 6]]
            for row in range(6)]
        for index in range(6):
            matrix[index][index] = max(0.0, matrix[index][index])
        rotated = [[0.0] * 6 for _ in range(6)]
        for row in range(6):
            for column in range(6):
                rotated[row][column] = sum(
                    transform[row][left] * matrix[left][right]
                    * transform[column][right]
                    for left in range(6) for right in range(6))
        scale = self.primary_covariance_scale if source == "eskf" else 1.0
        output = [value * scale for row in rotated for value in row]
        if source == "eskf":
            output[0] += self.primary_position_variance
            output[7] += self.primary_position_variance
            output[35] += self.primary_yaw_variance
        output[0] += self.alignment_position_variance
        output[7] += self.alignment_position_variance
        output[35] += self.alignment_yaw_variance
        return output

    def callback(self, source, message):
        with self.lock:
            now = rospy.Time.now()
            if not self.valid(message):
                self.invalid += 1
                return
            if not self.timely(message, now):
                self.stale += 1
                return
            if source == "eskf" and not self.primary_cycle_safe():
                self.unsafe_primary += 1
                self.primary_consecutive = 0
                return
            stamp = message.header.stamp.to_sec()
            previous_source_stamp = self.last_source_stamp[source]
            if (previous_source_stamp is not None
                    and stamp < previous_source_stamp - 0.5):
                self.reset_stream_state()
                previous_source_stamp = None
            if (previous_source_stamp is not None
                    and stamp <= previous_source_stamp):
                self.nonmonotonic += 1
                return
            if source == "eskf":
                if not self.primary_covariance_valid(message):
                    self.invalid += 1
                    self.primary_consecutive = 0
                    return
                previous_receipt = self.receipt["eskf"]
                if (previous_receipt is None or
                        (now - previous_receipt).to_sec()
                        > self.primary_timeout):
                    self.primary_consecutive = 0
                self.primary_consecutive += 1
            self.latest[source] = message
            self.receipt[source] = now
            self.last_source_stamp[source] = stamp

            desired_source = (
                "eskf" if self.primary_fresh(now) else "pure")
            if source != desired_source:
                return
            if (
                    self.last_output_stamp is not None
                    and stamp <= self.last_output_stamp):
                self.nonmonotonic += 1
                return
            if self.active_source != source:
                self.align_source(source, message)

            transformed = self.transformed_pose(message)
            if self.last_output is not None:
                distance = math.hypot(
                    transformed[0] - self.last_output[0],
                    transformed[1] - self.last_output[1])
                yaw_step = abs(wrap_angle(
                    transformed[2] - self.last_output[2]))
                if distance > self.max_jump or yaw_step > self.max_yaw_jump:
                    self.align_source(source, message)
                    transformed = self.transformed_pose(message)
                    self.realignments += 1

            output = Odometry()
            output.header.stamp = message.header.stamp
            output.header.frame_id = "lio_aiding_odom"
            output.child_frame_id = "base_link"
            output.pose.pose.position.x = transformed[0]
            output.pose.pose.position.y = transformed[1]
            output.pose.pose.position.z = message.pose.pose.position.z
            half_yaw = 0.5 * transformed[2]
            output.pose.pose.orientation.z = math.sin(half_yaw)
            output.pose.pose.orientation.w = math.cos(half_yaw)
            output.pose.covariance = self.transformed_covariance(
                source, message.pose.covariance)
            output.twist = message.twist
            self.publisher.publish(output)
            self.last_output = transformed
            self.last_output_stamp = stamp
            self.published += 1

    def publish_diagnostics(self, _event):
        with self.lock:
            now = rospy.Time.now()
            ages = {}
            for source in ("eskf", "pure"):
                receipt = self.receipt[source]
                ages[source] = (
                    None if receipt is None
                    else max(0.0, (now - receipt).to_sec()))
            payload = {
                "active_source": self.active_source,
                "primary_fresh": self.primary_fresh(now),
                "source_age_sec": ages,
                "published": self.published,
                "switches": self.switches,
                "realignments": self.realignments,
                "nonmonotonic": self.nonmonotonic,
                "invalid": self.invalid,
                "stale": self.stale,
                "unsafe_primary": self.unsafe_primary,
                "primary_consecutive": self.primary_consecutive,
                "primary_min_samples": self.primary_min_samples,
                "timestamp_resets": self.resets,
                "cycle_guard_param": self.cycle_guard_param,
                "cycle_safe": self.primary_cycle_safe(),
            }
        self.diagnostics_publisher.publish(
            String(data=json.dumps(payload, sort_keys=True)))


if __name__ == "__main__":
    try:
        LioOdometryMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
