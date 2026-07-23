#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy

import rospy
from sensor_msgs.msg import Imu


class MoraiROS1Bridge:
    """Normalize MORAI's raw IMU topic for LIO-SAM."""

    def __init__(self):
        rospy.init_node("morai_ros1_bridge")
        self.input_topic = rospy.get_param("~input_topic", "/imu")
        self.output_topic = rospy.get_param("~output_topic", "/imu/data")
        self.frame_id = rospy.get_param("~frame_id", "imu_link")
        self.imu_pub = rospy.Publisher(
            self.output_topic, Imu, queue_size=50)
        rospy.Subscriber(
            self.input_topic, Imu, self.imu_callback, queue_size=50)
        rospy.loginfo(
            "[MORAI IMU bridge] %s -> %s",
            self.input_topic, self.output_topic)

    @staticmethod
    def _ensure_covariance(values, diagonal):
        if any(abs(value) > 0.0 for value in values):
            return values
        result = [0.0] * 9
        result[0] = diagonal
        result[4] = diagonal
        result[8] = diagonal
        return result

    def imu_callback(self, msg):
        output = copy.deepcopy(msg)
        output.header.frame_id = self.frame_id
        output.orientation_covariance = self._ensure_covariance(
            output.orientation_covariance, 0.02)
        output.angular_velocity_covariance = self._ensure_covariance(
            output.angular_velocity_covariance, 0.03)
        output.linear_acceleration_covariance = self._ensure_covariance(
            output.linear_acceleration_covariance, 0.8)
        self.imu_pub.publish(output)


if __name__ == "__main__":
    try:
        MoraiROS1Bridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
