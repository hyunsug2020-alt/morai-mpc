#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


DEFAULT_SENSORS = {
    "GPS-2": ((3.232, 0.037, 0.658), (0.0, 0.0, 0.0)),
    "IMU-1": ((3.423, 0.012, 0.614), (0.0, 0.0, 0.0)),
    "Lidar3D-3": ((1.676, 0.005, 1.201), (0.0, 0.0, 0.0)),
}


def sensor_entry(items, config_key, fallback_frame):
    item = items[0]
    position = item["pos"]
    rotation = item["rot"]
    ros_config = item.get(config_key, {}).get("rosConfig", {})
    frame = ros_config.get("frameID") or fallback_frame
    xyz = tuple(float(position[key]) for key in ("x", "y", "z"))
    rpy = tuple(
        math.radians(float(rotation[key]))
        for key in ("roll", "pitch", "yaw"))
    return frame, (xyz, rpy)


def load_sensors(path):
    sensors = dict(DEFAULT_SENSORS)
    if not path:
        return sensors
    try:
        with open(os.path.expanduser(path), "r", encoding="utf-8-sig") as stream:
            config = json.load(stream)
        if config.get("GPSList"):
            frame, value = sensor_entry(config["GPSList"], "gc", "GPS-2")
            sensors[frame] = value
        if config.get("IMUList"):
            frame, value = sensor_entry(config["IMUList"], "ic", "IMU-1")
            sensors[frame] = value
        if config.get("Lidar3DList"):
            frame, value = sensor_entry(
                config["Lidar3DList"], "lc", "Lidar3D-3")
            sensors[frame] = value
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        rospy.logwarn(
            "[SensorTF] cannot load %s: %s; using asdf defaults", path, exc)
    return sensors


def make_transform(parent, child, xyz, rpy):
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = xyz[0]
    transform.transform.translation.y = xyz[1]
    transform.transform.translation.z = xyz[2]
    quaternion = quaternion_from_euler(*rpy)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    return transform


if __name__ == "__main__":
    rospy.init_node("sensor_tf_from_config")
    config_file = rospy.get_param("~sensor_config_file", "")
    parent_frame = rospy.get_param("~parent_frame", "base_link")
    sensors = load_sensors(config_file)
    transforms = [
        make_transform(parent_frame, frame, xyz, rpy)
        for frame, (xyz, rpy) in sensors.items()
    ]
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    rospy.loginfo(
        "[SensorTF] %s -> %s, source=%s",
        parent_frame, ", ".join(sorted(sensors)),
        config_file or "asdf defaults")
    rospy.spin()
