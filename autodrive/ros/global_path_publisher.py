#!/usr/bin/env python3
"""Publish the competition global path (x y per line text file) as a latched nav_msgs/Path in frame 'map'.

Same topic/frame the collector's record_ego.py expects: /planning/global_path. Orientation per pose is the
heading toward the next point so downstream consumers get a yaw as well.
"""
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def load(path_file):
    pts = []
    with open(path_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 2:
                continue
            pts.append((float(parts[0]), float(parts[1]), float(parts[2]) if len(parts) > 2 else 0.0))
    if len(pts) < 2:
        raise ValueError(f"{path_file}: fewer than 2 points")
    return pts


def main():
    rospy.init_node("global_path_publisher")
    path_file = rospy.get_param("~path_file")
    topic = rospy.get_param("~output_topic", "/planning/global_path")
    frame = rospy.get_param("~frame_id", "map")
    pts = load(path_file)
    msg = Path()
    msg.header.frame_id = frame
    msg.header.stamp = rospy.Time.now()
    for i, (x, y, z) in enumerate(pts):
        nx, ny, _ = pts[min(i + 1, len(pts) - 1)] if i + 1 < len(pts) else pts[i]
        px, py, _ = pts[i - 1] if i + 1 >= len(pts) else pts[i]
        yaw = math.atan2(ny - py, nx - px)
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.poses.append(ps)
    pub = rospy.Publisher(topic, Path, queue_size=1, latch=True)
    pub.publish(msg)
    rospy.loginfo("global path: %d points from %s -> %s (latched, %s)", len(pts), path_file, topic, frame)
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():  # re-publish every 5 s so late subscribers on rosbridge also get it
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        sys.exit(0)
