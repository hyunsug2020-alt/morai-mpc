#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


class EgoStatusFromESKF:
    """Expose ESKF localization through the legacy ego-status message shape."""

    def __init__(self):
        rospy.init_node("ego_status_from_eskf")

        self.odom_topic = rospy.get_param("~odom_topic", "/eskf/odom")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.ego_topic = rospy.get_param(
            "~ego_topic", "/localization/ego_status")
        self.latest_imu = None

        self.ego_pub = rospy.Publisher(
            self.ego_topic, EgoVehicleStatus, queue_size=10)
        self.filtered_odom_pub = rospy.Publisher(
            "/odometry/filtered", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher(
            "/Ego_pose", PoseStamped, queue_size=10)

        rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber(
            self.imu_topic, Imu, self.imu_callback, queue_size=20)
        rospy.loginfo(
            "[Localization] %s -> %s (legacy-compatible)",
            self.odom_topic, self.ego_topic)

    def imu_callback(self, msg):
        self.latest_imu = msg

    def odom_callback(self, odom):
        quaternion = [
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ]
        _, _, yaw = euler_from_quaternion(quaternion)

        ego = EgoVehicleStatus()
        ego.header = odom.header
        ego.position.x = odom.pose.pose.position.x
        ego.position.y = odom.pose.pose.position.y
        ego.position.z = odom.pose.pose.position.z
        ego.velocity.x = odom.twist.twist.linear.x
        ego.velocity.y = odom.twist.twist.linear.y
        ego.velocity.z = odom.twist.twist.linear.z
        ego.angular_velocity = odom.twist.twist.angular

        # This project stores heading as ROS yaw degrees: East=0, CCW positive.
        ego.heading = math.degrees(yaw)
        if self.latest_imu is not None:
            ego.acceleration = self.latest_imu.linear_acceleration
            ego.angular_velocity = self.latest_imu.angular_velocity
        self.ego_pub.publish(ego)

        filtered = Odometry()
        filtered.header = odom.header
        filtered.child_frame_id = odom.child_frame_id
        filtered.pose = odom.pose
        filtered.twist = odom.twist
        self.filtered_odom_pub.publish(filtered)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)


if __name__ == "__main__":
    try:
        EgoStatusFromESKF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
