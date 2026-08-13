#!/usr/bin/env python3
"""Convert Velodyne end-stamped point times to LIO-SAM start-relative time."""

import json
import math
import threading

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


class VelodyneTimeNormalizer:
    def __init__(self):
        rospy.init_node("velodyne_time_normalizer")
        self.input_topic = rospy.get_param(
            "~input_topic", "/velodyne_points")
        self.output_topic = rospy.get_param(
            "~output_topic", "/velodyne_points_lio")
        self.max_scan_duration = float(rospy.get_param(
            "~max_scan_duration_s", 0.30))

        self.lock = threading.Lock()
        self.received = 0
        self.published = 0
        self.rejected = 0
        self.last_receipt = None
        self.last_min_time = None
        self.last_max_time = None
        self.last_duration = None

        self.publisher = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=2)
        self.diagnostics_publisher = rospy.Publisher(
            "/velodyne_time_normalizer/diagnostics",
            String, queue_size=2)
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointCloud2, self.cloud_callback,
            queue_size=2, buff_size=16 * 1024 * 1024,
            tcp_nodelay=True)
        self.timer = rospy.Timer(
            rospy.Duration(1.0), self.publish_diagnostics)
        rospy.loginfo(
            "[Velodyne time] %s -> %s", self.input_topic,
            self.output_topic)

    @staticmethod
    def time_field(message):
        for field in message.fields:
            if field.name == "time":
                return field
        return None

    def reject(self, reason):
        with self.lock:
            self.rejected += 1
        rospy.logwarn_throttle(2.0, "[Velodyne time] %s", reason)

    def cloud_callback(self, message):
        receipt = rospy.Time.now()
        with self.lock:
            self.received += 1
            self.last_receipt = receipt

        field = self.time_field(message)
        if field is None:
            self.reject("PointCloud2 has no 'time' field")
            return
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            self.reject("'time' field must be one FLOAT32 value")
            return
        if message.width == 0 or message.height == 0:
            self.reject("empty point cloud")
            return

        mutable_data = bytearray(message.data)
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        try:
            point_times = np.ndarray(
                shape=(message.height, message.width), dtype=dtype,
                buffer=mutable_data, offset=field.offset,
                strides=(message.row_step, message.point_step))
        except (TypeError, ValueError) as exc:
            self.reject("cannot map point time field: {}".format(exc))
            return

        finite = np.isfinite(point_times)
        if not np.any(finite):
            self.reject("all point times are non-finite")
            return
        minimum = float(np.min(point_times[finite]))
        maximum = float(np.max(point_times[finite]))
        duration = maximum - minimum
        if (
                not math.isfinite(duration)
                or duration <= 0.0
                or duration > self.max_scan_duration):
            self.reject("invalid scan duration {:.6f}s".format(duration))
            return

        # Preserve each point's absolute acquisition time:
        # old_header + old_time == new_header + normalized_time.
        point_times[finite] -= minimum
        output = PointCloud2()
        output.header = message.header
        output.header.stamp = (
            message.header.stamp + rospy.Duration.from_sec(minimum))
        output.height = message.height
        output.width = message.width
        output.fields = message.fields
        output.is_bigendian = message.is_bigendian
        output.point_step = message.point_step
        output.row_step = message.row_step
        output.data = bytes(mutable_data)
        output.is_dense = message.is_dense
        self.publisher.publish(output)

        with self.lock:
            self.published += 1
            self.last_min_time = minimum
            self.last_max_time = maximum
            self.last_duration = duration

    def publish_diagnostics(self, _event):
        with self.lock:
            age = None
            if self.last_receipt is not None:
                age = max(
                    0.0, (rospy.Time.now() - self.last_receipt).to_sec())
            payload = {
                "mode": "tracking" if age is not None and age < 1.0
                else "waiting",
                "input_topic": self.input_topic,
                "output_topic": self.output_topic,
                "source_age_sec": age,
                "received": self.received,
                "published": self.published,
                "rejected": self.rejected,
                "raw_time_min_s": self.last_min_time,
                "raw_time_max_s": self.last_max_time,
                "scan_duration_s": self.last_duration,
                "absolute_point_time_preserved": True,
            }
        self.diagnostics_publisher.publish(
            String(data=json.dumps(payload, sort_keys=True)))


if __name__ == "__main__":
    try:
        VelodyneTimeNormalizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
