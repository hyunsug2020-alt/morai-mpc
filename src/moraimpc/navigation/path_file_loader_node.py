#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class PathFileLoaderNode(Node):
    """Load a saved path text file and publish /global_path."""

    def __init__(self):
        super().__init__('path_file_loader_node')

        self.declare_parameter('path_dir', str(Path.home() / 'henes_ws_ros2' / 'paths'))
        self.declare_parameter('path_id', '')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)

        path_dir = Path(str(self.get_parameter('path_dir').value)).expanduser()
        path_id = str(self.get_parameter('path_id').value).strip()
        path_topic = str(self.get_parameter('path_topic').value)
        frame_id = str(self.get_parameter('frame_id').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if not path_id:
            raise RuntimeError('path_id parameter is empty')

        file_path = self._resolve_path_file(path_dir, path_id)
        if file_path is None:
            raise RuntimeError(f'path file not found for id={path_id} in {path_dir}')

        self.path_msg = self._load_path(file_path, frame_id)
        if not self.path_msg.poses:
            raise RuntimeError(f'path file has no valid points: {file_path}')

        # Transient local keeps the last /global_path for late subscribers.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(NavPath, path_topic, qos)

        self.create_timer(1.0 / max(0.2, publish_rate_hz), self._publish_path)
        self.get_logger().info(
            f'Loaded path id={path_id} points={len(self.path_msg.poses)} file={file_path}'
        )

    def _resolve_path_file(self, path_dir: Path, path_id: str):
        candidates = [
            path_dir / f'{path_id}_filtered.txt',
            path_dir / f'{path_id}.txt',
            path_dir / f'{path_id}_original.txt',
        ]
        for c in candidates:
            if c.exists():
                return c
        return None

    def _load_path(self, file_path: Path, frame_id: str) -> NavPath:
        msg = NavPath()
        msg.header.frame_id = frame_id

        points: List[List[float]] = []
        with file_path.open('r', encoding='utf-8') as fp:
            for raw in fp:
                line = raw.strip()
                if not line:
                    continue
                parts = line.replace(',', ' ').split()
                if len(parts) < 2:
                    continue
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2]) if len(parts) >= 3 else 0.0
                except ValueError:
                    continue
                points.append([x, y, z])

        for x, y, z in points:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        return msg

    def _publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self.path_msg.poses:
            pose.header.stamp = self.path_msg.header.stamp
        self.pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFileLoaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
