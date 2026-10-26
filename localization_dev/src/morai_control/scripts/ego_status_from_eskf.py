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
        self.vehicle_state_topic = rospy.get_param(
            "~vehicle_state_topic", "/Competition_topic")
        self.ego_topic = rospy.get_param(
            "~ego_topic", "/localization/ego_status")
        self.latest_imu = None
        self.latest_vehicle_state = None
        # MORAI's rosbridge /Competition_topic reports wheel_angle with the opposite sign
        # of /Ego_topic (measured 2026-09-30: Ego +12.35 vs Competition -12.35). Set -1
        # there so the output matches the /Ego_topic convention consumers were built on.
        self.wheel_angle_sign = float(rospy.get_param("~wheel_angle_sign", 1.0))
        # Added to the published heading so it matches /Ego_topic's true-north convention
        # when the ESKF runs in grid yaw (see eskf_node ~imu_yaw_offset_deg).
        self.heading_offset_deg = float(rospy.get_param("~heading_offset_deg", 0.0))

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
        rospy.Subscriber(
            self.vehicle_state_topic, EgoVehicleStatus,
            self.vehicle_state_callback, queue_size=20)
        rospy.loginfo(
            "[Localization] %s -> %s (legacy-compatible)",
            self.odom_topic, self.ego_topic)

    def imu_callback(self, msg):
        self.latest_imu = msg

    def vehicle_state_callback(self, msg):
        # Raw vehicle telemetry from the sim/vehicle, not GPS/IMU-derived --
        # passthrough only, no fusion needed.
        self.latest_vehicle_state = msg

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
        ego.heading = math.degrees(yaw) + self.heading_offset_deg
        if self.latest_imu is not None:
            ego.acceleration = self.latest_imu.linear_acceleration
            ego.angular_velocity = self.latest_imu.angular_velocity
        if self.latest_vehicle_state is not None:
            # accel/brake/wheel_angle are genuinely consumed downstream:
            # student_v3/gui.py reads them into the model's input feature
            # vector, and moraimpc's path_follower_node reads wheel_angle
            # for its own LTV-MPC steering-state tracking. gear/ctrl_mode
            # are not currently read by any known consumer, but are passed
            # through too so this topic remains a faithful mirror of the
            # native /Ego_topic-shaped message for any future consumer.
            ego.accel = self.latest_vehicle_state.accel
            ego.brake = self.latest_vehicle_state.brake
            ego.wheel_angle = self.wheel_angle_sign * self.latest_vehicle_state.wheel_angle
            ego.gear = self.latest_vehicle_state.gear
            ego.ctrl_mode = self.latest_vehicle_state.ctrl_mode
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
