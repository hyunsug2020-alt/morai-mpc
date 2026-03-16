#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64


class PathFollowerNode(Node):
    """ROS 2 Humble path follower converted from ROS1 gen_planner."""

    def __init__(self):
        super().__init__("path_follower_node")

        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("forward_speed_kmh", 8.0)
        self.declare_parameter("reverse_speed_kmh", 5.0)
        self.declare_parameter("steer_gain", 1.0)
        self.declare_parameter("reverse_steer_gain", 1.2)
        self.declare_parameter("max_steer_deg", 55.0)
        self.declare_parameter("arrival_threshold_forward", 1.0)
        self.declare_parameter("arrival_threshold_reverse", 0.6)
        self.declare_parameter("use_imu_heading_fallback", True)
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("path_topic", "/global_path")
        self.declare_parameter("heading_topic", "/ublox_gps/heading")
        self.declare_parameter("imu_topic", "/ublox_gps/imu")

        self.forward_speed_kmh = float(self.get_parameter("forward_speed_kmh").value)
        self.reverse_speed_kmh = float(self.get_parameter("reverse_speed_kmh").value)
        self.steer_gain = float(self.get_parameter("steer_gain").value)
        self.reverse_steer_gain = float(self.get_parameter("reverse_steer_gain").value)
        self.max_steer_deg = float(self.get_parameter("max_steer_deg").value)
        self.arrival_threshold_forward = float(self.get_parameter("arrival_threshold_forward").value)
        self.arrival_threshold_reverse = float(self.get_parameter("arrival_threshold_reverse").value)
        self.use_imu_heading_fallback = bool(self.get_parameter("use_imu_heading_fallback").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        heading_topic = str(self.get_parameter("heading_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)

        self.global_path = Path()
        self.has_path = False
        self.has_odom = False
        self.has_heading = False
        self.teleop_mode = False
        self.obstacle_stop = False
        self.slope_factor = 1.0
        self.fused_heading_deg = 0.0
        self.target_index = 0
        self.current_driving_direction = 1
        self.direction_profile: List[int] = []
        self.latest_odom = Odometry()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.create_subscription(Path, path_topic, self.global_path_cb, 10)
        self.create_subscription(Float64, heading_topic, self.heading_cb, 20)
        self.create_subscription(Imu, imu_topic, self.imu_cb, 20)
        self.create_subscription(Bool, "/teleop_mode", self.teleop_cb, 10)
        self.create_subscription(Bool, "/obstacle_stop", self.obstacle_stop_cb, 10)
        self.create_subscription(Float64, "/slope_factor", self.slope_factor_cb, 10)

        hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / max(hz, 1.0), self.control_loop)

        self.get_logger().info(
            f"Path follower node started (ROS 2 Humble) | "
            f"odom={odom_topic} path={path_topic} heading={heading_topic} imu={imu_topic}"
        )

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.has_odom = True

    def heading_cb(self, msg: Float64) -> None:
        self.fused_heading_deg = float(msg.data)
        self.has_heading = True

    def imu_cb(self, msg: Imu) -> None:
        if self.has_heading and not self.use_imu_heading_fallback:
            return
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self.fused_heading_deg = math.degrees(yaw)
        self.has_heading = True

    def teleop_cb(self, msg: Bool) -> None:
        self.teleop_mode = bool(msg.data)

    def obstacle_stop_cb(self, msg: Bool) -> None:
        self.obstacle_stop = bool(msg.data)

    def slope_factor_cb(self, msg: Float64) -> None:
        self.slope_factor = float(msg.data)

    def global_path_cb(self, msg: Path) -> None:
        self.global_path = msg
        self.has_path = len(msg.poses) > 0
        self.target_index = 0
        self.current_driving_direction = 1
        self.analyze_path_direction()
        self.get_logger().info(f"New global path received: {len(msg.poses)} points")

    def analyze_path_direction(self) -> None:
        path_len = len(self.global_path.poses)
        if path_len < 3:
            self.direction_profile = [1] * path_len
            return

        cusp_indices = []
        cusp_threshold = -0.5
        for i in range(1, path_len - 1):
            p_prev = self.global_path.poses[i - 1].pose.position
            p_curr = self.global_path.poses[i].pose.position
            p_next = self.global_path.poses[i + 1].pose.position
            v1x = p_curr.x - p_prev.x
            v1y = p_curr.y - p_prev.y
            v2x = p_next.x - p_curr.x
            v2y = p_next.y - p_curr.y
            mag_v1 = math.hypot(v1x, v1y)
            mag_v2 = math.hypot(v2x, v2y)
            if mag_v1 <= 0.01 or mag_v2 <= 0.01:
                continue
            cosine_angle = (v1x * v2x + v1y * v2y) / (mag_v1 * mag_v2)
            if cosine_angle < cusp_threshold:
                cusp_indices.append(i)

        self.direction_profile = [1] * path_len
        direction = 1
        cusp_ptr = 0
        for i in range(path_len):
            if cusp_ptr < len(cusp_indices) and i >= cusp_indices[cusp_ptr]:
                direction *= -1
                cusp_ptr += 1
            self.direction_profile[i] = direction

    def get_pwm_speed(self, target_kmh: float) -> float:
        pwm = abs(target_kmh) * (255.0 / 25.5) * self.slope_factor
        return max(0.0, min(255.0, pwm))

    def constrain_steer(self, steer_deg: float) -> float:
        return max(-self.max_steer_deg, min(self.max_steer_deg, steer_deg))

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def control_loop(self) -> None:
        if self.teleop_mode:
            return
        if self.obstacle_stop:
            self.publish_stop()
            return
        if not (self.has_path and self.has_odom and self.has_heading):
            return
        if not self.global_path.poses:
            self.publish_stop()
            return
        if self.target_index >= len(self.global_path.poses):
            self.publish_stop()
            return

        pos = self.latest_odom.pose.pose.position
        target_pos = self.global_path.poses[self.target_index].pose.position

        is_reversing = False
        if self.direction_profile and self.target_index < len(self.direction_profile):
            is_reversing = self.direction_profile[self.target_index] == -1
        arrival_threshold = self.arrival_threshold_reverse if is_reversing else self.arrival_threshold_forward

        dx = target_pos.x - pos.x
        dy = target_pos.y - pos.y
        distance = math.hypot(dx, dy)
        has_passed_target = False
        if self.target_index < len(self.global_path.poses) - 1:
            next_target = self.global_path.poses[self.target_index + 1].pose.position
            v_tvx = pos.x - target_pos.x
            v_tvy = pos.y - target_pos.y
            v_tnx = next_target.x - target_pos.x
            v_tny = next_target.y - target_pos.y
            dot = v_tvx * v_tnx + v_tvy * v_tny
            if dot > 0.0:
                has_passed_target = True

        if distance < arrival_threshold or has_passed_target:
            cusp_point = False
            if self.target_index < len(self.direction_profile) - 1:
                cusp_point = self.direction_profile[self.target_index] != self.direction_profile[self.target_index + 1]
            self.target_index += 1
            if cusp_point:
                self.current_driving_direction *= -1
                self.publish_stop()
                return
            if self.target_index >= len(self.global_path.poses):
                self.publish_stop()
                return
            target_pos = self.global_path.poses[self.target_index].pose.position
            dx = target_pos.x - pos.x
            dy = target_pos.y - pos.y

        target_heading = math.atan2(dy, dx)
        ego_heading = math.radians(self.fused_heading_deg)
        cmd = Twist()

        if self.current_driving_direction == -1:
            steer_error = ((target_heading - (ego_heading + math.pi) + math.pi) % (2.0 * math.pi)) - math.pi
            steer_deg = math.degrees(steer_error) * self.reverse_steer_gain
            cmd.linear.x = -self.get_pwm_speed(self.reverse_speed_kmh)
        else:
            steer_error = ((target_heading - ego_heading + math.pi) % (2.0 * math.pi)) - math.pi
            steer_deg = math.degrees(steer_error) * self.steer_gain
            cmd.linear.x = self.get_pwm_speed(self.forward_speed_kmh)

        cmd.angular.z = self.constrain_steer(steer_deg)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
