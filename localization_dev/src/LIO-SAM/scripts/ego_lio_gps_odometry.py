#!/usr/bin/env python3
"""Use MORAI Ego ground truth as LIO-SAM's absolute XY position anchor."""

import math

import rospy
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


def convert_ego_to_odometry(
        message,
        stamp,
        frame_id="odom",
        child_frame_id="ego_lio_anchor",
        xy_variance=1.0,
        z_variance=4.0,
        lidar_forward=1.045,
        lidar_left=0.0):
    """Convert Ego pose to the LiDAR-origin position used by LIO-SAM."""
    values = (
        message.position.x,
        message.position.y,
        message.position.z,
        message.heading,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Ego pose contains a non-finite value")

    yaw = math.radians(message.heading)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    output = Odometry()
    output.header.stamp = stamp
    output.header.frame_id = frame_id
    output.child_frame_id = child_frame_id
    output.pose.pose.position.x = (
        message.position.x + lidar_forward * cos_yaw - lidar_left * sin_yaw
    )
    output.pose.pose.position.y = (
        message.position.y + lidar_forward * sin_yaw + lidar_left * cos_yaw
    )
    output.pose.pose.position.z = message.position.z
    quaternion = quaternion_from_euler(0.0, 0.0, yaw)
    output.pose.pose.orientation.x = quaternion[0]
    output.pose.pose.orientation.y = quaternion[1]
    output.pose.pose.orientation.z = quaternion[2]
    output.pose.pose.orientation.w = quaternion[3]
    output.pose.covariance[0] = xy_variance
    output.pose.covariance[7] = xy_variance
    output.pose.covariance[14] = z_variance
    output.pose.covariance[21] = 1.0e6
    output.pose.covariance[28] = 1.0e6
    output.pose.covariance[35] = 1.0e6
    return output


class EgoLioGpsOdometry:
    def __init__(self):
        rospy.init_node("ego_lio_gps_odometry")
        self.input_topic = rospy.get_param("~input_topic", "/Ego_topic")
        self.output_topic = rospy.get_param(
            "~output_topic", "/lio_sam/gps/odometry")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.xy_variance = float(rospy.get_param("~xy_variance", 1.0))
        self.z_variance = float(rospy.get_param("~z_variance", 4.0))
        self.lidar_forward = float(rospy.get_param("~lidar_forward", 1.045))
        self.lidar_left = float(rospy.get_param("~lidar_left", 0.0))
        if self.xy_variance <= 0.0 or self.z_variance <= 0.0:
            raise ValueError("Ego anchor variances must be positive")

        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=50)
        self.subscriber = rospy.Subscriber(
            self.input_topic, EgoVehicleStatus, self.callback, queue_size=50)
        rospy.loginfo(
            "Ego -> LIO absolute anchor: %s -> %s, lever=(%.3f, %.3f)m",
            self.input_topic,
            self.output_topic,
            self.lidar_forward,
            self.lidar_left,
        )

    def callback(self, message):
        try:
            # WebSocket Ego and UDP sensors can use different source clocks.
            # Arrival time keeps the GPS factor inside LIO-SAM's +/-0.2 s gate.
            output = convert_ego_to_odometry(
                message,
                rospy.Time.now(),
                frame_id=self.frame_id,
                xy_variance=self.xy_variance,
                z_variance=self.z_variance,
                lidar_forward=self.lidar_forward,
                lidar_left=self.lidar_left,
            )
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "%s", error)
            return
        self.publisher.publish(output)


if __name__ == "__main__":
    EgoLioGpsOdometry()
    rospy.spin()
