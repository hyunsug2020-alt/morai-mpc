#!/usr/bin/env python3
"""Densify low-rate MORAI IMU samples for LIO-SAM timestamp integration.

This node does not invent new sensor information. It interpolates measurements
between two received samples after the newer endpoint arrives. LIO-SAM delays
point-cloud processing internally, so the resulting ordered timestamps are
available before the matching scan is deskewed.
"""

import json
import math
import threading

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class ImuResampler:
    def __init__(self):
        rospy.init_node("imu_resampler")
        self.input_topic = rospy.get_param("~input_topic", "/imu/data")
        self.output_topic = rospy.get_param(
            "~output_topic", "/imu/data_lio")
        self.target_rate = max(
            1.0, float(rospy.get_param("~target_rate_hz", 100.0)))
        self.max_gap = max(
            1.0 / self.target_rate,
            float(rospy.get_param("~max_gap_s", 0.30)))
        self.max_segments = max(
            1, int(rospy.get_param("~max_segments", 100)))

        self.lock = threading.Lock()
        self.previous = None
        self.received = 0
        self.published = 0
        self.invalid = 0
        self.nonmonotonic = 0
        self.gap_resets = 0
        self.last_gap = None
        self.last_segments = 0
        self.last_receipt = None

        self.publisher = rospy.Publisher(
            self.output_topic, Imu, queue_size=1000)
        self.diagnostics_publisher = rospy.Publisher(
            "/imu_resampler/diagnostics", String, queue_size=2,
            latch=True)
        self.subscriber = rospy.Subscriber(
            self.input_topic, Imu, self.callback,
            queue_size=200, tcp_nodelay=True)
        self.timer = rospy.Timer(
            rospy.Duration(1.0), self.publish_diagnostics)
        rospy.loginfo(
            "[IMU resampler] %s -> %s, target=%.1f Hz",
            self.input_topic, self.output_topic, self.target_rate)

    @staticmethod
    def finite_message(message):
        values = (
            message.header.stamp.to_sec(),
            message.orientation.x, message.orientation.y,
            message.orientation.z, message.orientation.w,
            message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        quaternion_norm = math.sqrt(sum(value * value for value in values[1:5]))
        return (
            values[0] > 0.0
            and all(math.isfinite(value) for value in values)
            and quaternion_norm > 1.0e-8)

    @staticmethod
    def slerp(first, second, alpha):
        q0 = [first.x, first.y, first.z, first.w]
        q1 = [second.x, second.y, second.z, second.w]
        n0 = math.sqrt(sum(value * value for value in q0))
        n1 = math.sqrt(sum(value * value for value in q1))
        q0 = [value / n0 for value in q0]
        q1 = [value / n1 for value in q1]
        dot = sum(a * b for a, b in zip(q0, q1))
        if dot < 0.0:
            q1 = [-value for value in q1]
            dot = -dot
        dot = max(-1.0, min(1.0, dot))
        if dot > 0.9995:
            result = [
                a + alpha * (b - a) for a, b in zip(q0, q1)]
            norm = math.sqrt(sum(value * value for value in result))
            return [value / norm for value in result]
        theta = math.acos(dot)
        sine = math.sin(theta)
        first_weight = math.sin((1.0 - alpha) * theta) / sine
        second_weight = math.sin(alpha * theta) / sine
        return [
            first_weight * a + second_weight * b
            for a, b in zip(q0, q1)]

    @staticmethod
    def lerp(first, second, alpha):
        return first + alpha * (second - first)

    def interpolate_covariance(self, first, second, alpha):
        # ROS uses covariance[0] == -1 to mean that the estimate is absent.
        # Interpolating that sentinel would turn "unknown" into fake certainty.
        if first[0] < 0.0 or second[0] < 0.0:
            return [-1.0] + [0.0] * 8
        return [
            self.lerp(a, b, alpha) for a, b in zip(first, second)]

    def interpolate(self, first, second, alpha, stamp):
        output = Imu()
        output.header.stamp = stamp
        output.header.frame_id = second.header.frame_id
        quaternion = self.slerp(
            first.orientation, second.orientation, alpha)
        output.orientation.x = quaternion[0]
        output.orientation.y = quaternion[1]
        output.orientation.z = quaternion[2]
        output.orientation.w = quaternion[3]
        for axis in ("x", "y", "z"):
            setattr(
                output.angular_velocity, axis,
                self.lerp(
                    getattr(first.angular_velocity, axis),
                    getattr(second.angular_velocity, axis), alpha))
            setattr(
                output.linear_acceleration, axis,
                self.lerp(
                    getattr(first.linear_acceleration, axis),
                    getattr(second.linear_acceleration, axis), alpha))
        output.orientation_covariance = self.interpolate_covariance(
            first.orientation_covariance,
            second.orientation_covariance, alpha)
        output.angular_velocity_covariance = self.interpolate_covariance(
            first.angular_velocity_covariance,
            second.angular_velocity_covariance, alpha)
        output.linear_acceleration_covariance = self.interpolate_covariance(
            first.linear_acceleration_covariance,
            second.linear_acceleration_covariance, alpha)
        return output

    def publish_endpoint(self, message):
        output = self.interpolate(
            message, message, 1.0, message.header.stamp)
        self.publisher.publish(output)
        self.published += 1

    def callback(self, message):
        with self.lock:
            self.received += 1
            self.last_receipt = rospy.Time.now()
            if not self.finite_message(message):
                self.invalid += 1
                return
            if self.previous is None:
                self.publish_endpoint(message)
                self.previous = message
                self.last_segments = 1
                return

            first_stamp = self.previous.header.stamp.to_sec()
            second_stamp = message.header.stamp.to_sec()
            gap = second_stamp - first_stamp
            self.last_gap = gap
            if gap <= 0.0:
                self.nonmonotonic += 1
                return
            if gap > self.max_gap:
                self.gap_resets += 1
                self.publish_endpoint(message)
                self.previous = message
                self.last_segments = 1
                return

            segments = min(
                self.max_segments,
                max(1, int(math.ceil(gap * self.target_rate))))
            for index in range(1, segments + 1):
                alpha = float(index) / float(segments)
                stamp = rospy.Time.from_sec(
                    first_stamp + alpha * gap)
                self.publisher.publish(self.interpolate(
                    self.previous, message, alpha, stamp))
                self.published += 1
            self.previous = message
            self.last_segments = segments

    def publish_diagnostics(self, _event):
        with self.lock:
            source_age = None
            if self.last_receipt is not None:
                source_age = max(
                    0.0,
                    (rospy.Time.now() - self.last_receipt).to_sec())
            payload = {
                "mode": "tracking"
                if source_age is not None and source_age < 1.0
                else "waiting",
                "input_topic": self.input_topic,
                "output_topic": self.output_topic,
                "target_rate_hz": self.target_rate,
                "source_age_sec": source_age,
                "raw_received": self.received,
                "dense_published": self.published,
                "invalid": self.invalid,
                "nonmonotonic": self.nonmonotonic,
                "gap_resets": self.gap_resets,
                "last_raw_gap_s": self.last_gap,
                "last_segments": self.last_segments,
                "interpolation_only": True,
            }
        self.diagnostics_publisher.publish(
            String(data=json.dumps(payload, sort_keys=True)))


if __name__ == "__main__":
    try:
        ImuResampler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
