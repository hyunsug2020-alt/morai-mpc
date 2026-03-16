#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import re
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from std_msgs.msg import Float64


class PathMakerNode(Node):
    """ROS 2 Humble path recorder (legacy path_maker equivalent)."""

    def __init__(self):
        super().__init__('path_maker_node')

        self.declare_parameter('output_dir', str(Path.home() / 'henes_ws_ros2' / 'paths'))
        self.declare_parameter('path_name', 'recorded_path')
        self.declare_parameter('timestamped_filename', True)
        self.declare_parameter('use_numbered_filename', True)
        self.declare_parameter('use_filter', True)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('record_rate_hz', 30.0)
        self.declare_parameter('min_record_distance_default', 0.2)
        self.declare_parameter('require_speed_for_record', False)

        output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()
        path_name = str(self.get_parameter('path_name').value)
        timestamped_filename = bool(self.get_parameter('timestamped_filename').value)
        use_numbered_filename = bool(self.get_parameter('use_numbered_filename').value)
        self.use_filter = bool(self.get_parameter('use_filter').value)
        self.min_record_distance_default = float(self.get_parameter('min_record_distance_default').value)
        self.min_record_distance = self.min_record_distance_default
        self.require_speed_for_record = bool(self.get_parameter('require_speed_for_record').value)

        output_dir.mkdir(parents=True, exist_ok=True)
        suffix = '_filtered' if self.use_filter else '_original'
        if use_numbered_filename:
            next_idx = self._next_path_index(output_dir)
            self.output_file = output_dir / f'{next_idx}{suffix}.txt'
        elif timestamped_filename:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.output_file = output_dir / f'{path_name}_{ts}{suffix}.txt'
        else:
            self.output_file = output_dir / f'{path_name}{suffix}.txt'
        self.fp = self.output_file.open('w', encoding='utf-8')

        self.global_path = NavPath()
        self.global_path.header.frame_id = 'map'

        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.gps_quality = 1.0
        self.vel_from_gps = (0.0, 0.0)
        self.have_odom = False
        self.last_local_x = 0.0
        self.last_local_y = 0.0
        self.record_count = 0
        self.quality_counter = 0
        self.position_history: List[Tuple[float, float]] = []
        self.history_size = 5

        self.path_pub = self.create_publisher(NavPath, '/global_path', 10)
        odom_topic = str(self.get_parameter('odom_topic').value)

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.create_subscription(Point, '/utm_origin', self.origin_cb, 10)
        self.create_subscription(Float64, '/gps/quality', self.quality_cb, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/ublox_gps/fix_velocity', self.gps_vel_cb, 10)

        self.latest_odom = Odometry()

        pub_hz = float(self.get_parameter('publish_rate_hz').value)
        rec_hz = float(self.get_parameter('record_rate_hz').value)
        self.create_timer(1.0 / max(pub_hz, 0.1), self.publish_path)
        self.create_timer(1.0 / max(rec_hz, 1.0), self.record_step)

        self.get_logger().info(f'Path maker started. Odom: {odom_topic} | Output: {self.output_file}')

    def _next_path_index(self, output_dir: Path) -> int:
        max_idx = 0
        for p in output_dir.glob('*.txt'):
            m = re.match(r'^(\d+)(?:_.*)?\.txt$', p.name)
            if not m:
                continue
            idx = int(m.group(1))
            if idx > max_idx:
                max_idx = idx
        return max_idx + 1

    def origin_cb(self, msg: Point) -> None:
        self.origin_x = float(msg.x)
        self.origin_y = float(msg.y)
        self.origin_set = True

    def quality_cb(self, msg: Float64) -> None:
        self.gps_quality = float(msg.data)
        if self.gps_quality > 0.8:
            self.min_record_distance = 0.1
        elif self.gps_quality > 0.6:
            self.min_record_distance = 0.15
        elif self.gps_quality > 0.4:
            self.min_record_distance = 0.25
        else:
            self.min_record_distance = 0.5

    def gps_vel_cb(self, msg: TwistWithCovarianceStamped) -> None:
        self.vel_from_gps = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.have_odom = True

    def publish_path(self) -> None:
        if not self.global_path.poses:
            return
        self.global_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.global_path)

    def record_step(self) -> None:
        if not self.have_odom:
            return
        if not self.origin_set:
            return

        p = self.latest_odom.pose.pose.position
        local_x = float(p.x)
        local_y = float(p.y)
        local_z = float(p.z)

        self.position_history.append((local_x, local_y))
        if len(self.position_history) > self.history_size:
            self.position_history.pop(0)

        if len(self.position_history) >= 3:
            w = [0.2, 0.3, 0.5]
            sm_x = sum(ww * pp[0] for ww, pp in zip(w, self.position_history[-3:]))
            sm_y = sum(ww * pp[1] for ww, pp in zip(w, self.position_history[-3:]))
            local_x, local_y = sm_x, sm_y

        distance = math.hypot(local_x - self.last_local_x, local_y - self.last_local_y)
        if distance < self.min_record_distance:
            return

        if self.gps_quality < 0.4:
            self.quality_counter = 0
            return
        self.quality_counter += 1
        required_count = 1 if self.gps_quality > 0.8 else 2
        if self.quality_counter < required_count:
            return

        if self.require_speed_for_record:
            speed = math.hypot(self.vel_from_gps[0], self.vel_from_gps[1])
            min_speed = 0.3 if self.gps_quality > 0.8 else 0.5
            if speed < min_speed:
                return

        global_x = local_x + self.origin_x
        global_y = local_y + self.origin_y

        self.fp.write(f'{global_x}\t{global_y}\t{local_z}\n')
        self.fp.flush()
        self.last_local_x = local_x
        self.last_local_y = local_y
        self.record_count += 1

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = local_x
        pose.pose.position.y = local_y
        pose.pose.position.z = local_z
        pose.pose.orientation = self.latest_odom.pose.pose.orientation
        self.global_path.poses.append(pose)

    def destroy_node(self):
        try:
            self.fp.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathMakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
