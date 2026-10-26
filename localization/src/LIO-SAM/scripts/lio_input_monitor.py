#!/usr/bin/env python3
"""Live timestamp/rate contract monitor for MORAI LIO-SAM inputs."""

import bisect
import csv
import hashlib
import json
import math
import os
import statistics
import threading
from collections import deque

import numpy as np
import rospy
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String


class LioInputMonitor:
    CSV_FIELDS = [
        "scan_index",
        "receipt_time_s",
        "header_seq",
        "header_stamp_s",
        "scan_start_s",
        "scan_end_s",
        "scan_duration_s",
        "point_count",
        "finite_xyz_count",
        "intra_frame_duplicate_points",
        "intra_frame_duplicate_ratio",
        "previous_frame_overlap_points",
        "previous_frame_overlap_ratio",
        "duplicate_header_stamp",
        "duplicate_full_frame",
        "frame_hash",
    ]

    def __init__(self):
        rospy.init_node("lio_input_monitor")
        self.lock = threading.RLock()
        self.window = float(rospy.get_param("~window_s", 20.0))
        self.raw_imu = deque(maxlen=5000)
        self.dense_imu = deque(maxlen=10000)
        self.ego = deque(maxlen=5000)
        self.gps = deque(maxlen=1000)
        self.scans = deque(maxlen=300)
        self.receipt = {}
        self.latencies = {
            name: deque(maxlen=5000)
            for name in ("raw_imu", "dense_imu", "ego", "gps", "lidar")}
        self.csv_path = os.path.abspath(os.path.expanduser(
            rospy.get_param("~csv_path", ""))) if rospy.get_param(
                "~csv_path", "") else ""
        self.csv_stream = None
        self.csv_writer = None
        self.scan_index = 0
        self.recent_lidar_stamps = deque(maxlen=300)
        self.recent_lidar_hashes = deque(maxlen=300)
        self.previous_unique_xyz = None
        self.duplicate_stamp_count = 0
        self.duplicate_frame_count = 0
        self.max_intra_duplicate_ratio = 0.0
        self.last_duplicate_metrics = None
        self.open_csv()

        raw_imu_topic = rospy.get_param("~raw_imu_topic", "/imu/data")
        dense_imu_topic = rospy.get_param(
            "~dense_imu_topic", "/imu/data_lio")
        ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        gps_topic = rospy.get_param("~gps_topic", "/gps")
        lidar_topic = rospy.get_param(
            "~lidar_topic", "/velodyne_points_lio")

        rospy.Subscriber(
            raw_imu_topic, Imu,
            lambda message: self.stamp_callback("raw_imu", message),
            queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(
            dense_imu_topic, Imu,
            lambda message: self.stamp_callback("dense_imu", message),
            queue_size=3000, tcp_nodelay=True)
        rospy.Subscriber(
            ego_topic, EgoVehicleStatus,
            lambda message: self.stamp_callback("ego", message),
            queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(
            gps_topic, GPSMessage,
            lambda message: self.stamp_callback("gps", message),
            queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(
            lidar_topic, PointCloud2,
            self.cloud_callback, queue_size=30,
            buff_size=32 * 1024 * 1024, tcp_nodelay=True)
        self.publisher = rospy.Publisher(
            "/lio_sam/input_diagnostics", String,
            queue_size=2, latch=True)
        self.timer = rospy.Timer(
            rospy.Duration(1.0), self.publish_diagnostics)
        rospy.on_shutdown(self.close_csv)

    def open_csv(self):
        if not self.csv_path:
            return
        try:
            output_dir = os.path.dirname(self.csv_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            # One fixed path per launch. A new run intentionally replaces the
            # previous run so stale and current samples cannot be mixed.
            self.csv_stream = open(
                self.csv_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.DictWriter(
                self.csv_stream, fieldnames=self.CSV_FIELDS)
            self.csv_writer.writeheader()
            self.csv_stream.flush()
            rospy.loginfo(
                "[LIO input] LiDAR duplicate CSV (overwrite): %s",
                self.csv_path)
        except OSError as exc:
            rospy.logerr(
                "[LIO input] cannot open LiDAR duplicate CSV %s: %s",
                self.csv_path, exc)
            self.csv_stream = None
            self.csv_writer = None

    def close_csv(self):
        with self.lock:
            if self.csv_stream is not None and not self.csv_stream.closed:
                self.csv_stream.flush()
                self.csv_stream.close()

    @staticmethod
    def field_array(message, field):
        if field.datatype != field.FLOAT32 or field.count != 1:
            raise ValueError("{} must be FLOAT32[1]".format(field.name))
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        return np.ndarray(
            (message.height, message.width), dtype=dtype,
            buffer=message.data, offset=field.offset,
            strides=(message.row_step, message.point_step)).reshape(-1)

    @classmethod
    def unique_xyz(cls, message):
        fields = {field.name: field for field in message.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            raise ValueError("PointCloud2 has no complete x/y/z fields")
        xyz = np.ascontiguousarray(np.column_stack([
            cls.field_array(message, fields[name])
            for name in ("x", "y", "z")]), dtype=np.float32)
        finite_xyz = xyz[np.isfinite(xyz).all(axis=1)]
        packed = finite_xyz.view(np.dtype(
            (np.void, finite_xyz.dtype.itemsize * 3))).reshape(-1)
        return finite_xyz.shape[0], np.unique(packed)

    def duplicate_metrics(self, message, receipt, start, end):
        finite_count, unique_xyz = self.unique_xyz(message)
        unique_count = unique_xyz.size
        intra_duplicates = max(0, finite_count - unique_count)
        intra_ratio = intra_duplicates / max(1, finite_count)
        previous_overlap = 0
        previous_overlap_ratio = 0.0
        if self.previous_unique_xyz is not None:
            previous_overlap = np.intersect1d(
                unique_xyz, self.previous_unique_xyz,
                assume_unique=True).size
            previous_overlap_ratio = previous_overlap / max(1, unique_count)

        stamp_ns = message.header.stamp.to_nsec()
        frame_hash = hashlib.blake2b(
            message.data, digest_size=16).hexdigest()
        duplicate_stamp = stamp_ns in self.recent_lidar_stamps
        duplicate_frame = frame_hash in self.recent_lidar_hashes
        self.recent_lidar_stamps.append(stamp_ns)
        self.recent_lidar_hashes.append(frame_hash)
        self.previous_unique_xyz = unique_xyz
        self.scan_index += 1
        if duplicate_stamp:
            self.duplicate_stamp_count += 1
        if duplicate_frame:
            self.duplicate_frame_count += 1
        self.max_intra_duplicate_ratio = max(
            self.max_intra_duplicate_ratio, intra_ratio)
        metrics = {
            "scan_index": self.scan_index,
            "receipt_time_s": "{:.9f}".format(receipt),
            "header_seq": message.header.seq,
            "header_stamp_s": "{:.9f}".format(
                message.header.stamp.to_sec()),
            "scan_start_s": "{:.9f}".format(start),
            "scan_end_s": "{:.9f}".format(end),
            "scan_duration_s": "{:.9f}".format(end - start),
            "point_count": message.width * message.height,
            "finite_xyz_count": finite_count,
            "intra_frame_duplicate_points": intra_duplicates,
            "intra_frame_duplicate_ratio": "{:.9f}".format(intra_ratio),
            "previous_frame_overlap_points": previous_overlap,
            "previous_frame_overlap_ratio": "{:.9f}".format(
                previous_overlap_ratio),
            "duplicate_header_stamp": int(duplicate_stamp),
            "duplicate_full_frame": int(duplicate_frame),
            "frame_hash": frame_hash,
        }
        self.last_duplicate_metrics = {
            "intra_frame_duplicate_points": intra_duplicates,
            "intra_frame_duplicate_ratio": intra_ratio,
            "previous_frame_overlap_points": previous_overlap,
            "previous_frame_overlap_ratio": previous_overlap_ratio,
            "duplicate_header_stamp": duplicate_stamp,
            "duplicate_full_frame": duplicate_frame,
        }
        return metrics

    def stamp_callback(self, source, message):
        stamp = message.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp <= 0.0:
            return
        with self.lock:
            getattr(self, source).append(stamp)
            receipt = rospy.Time.now().to_sec()
            self.receipt[source] = receipt
            self.latencies[source].append(receipt - stamp)

    def cloud_callback(self, message):
        field = next(
            (item for item in message.fields if item.name == "time"), None)
        if field is None or message.width == 0 or message.height == 0:
            return
        try:
            times = self.field_array(message, field)
            finite = times[np.isfinite(times)]
            if finite.size == 0:
                return
            start = message.header.stamp.to_sec() + float(finite.min())
            end = message.header.stamp.to_sec() + float(finite.max())
        except (TypeError, ValueError):
            return
        if end <= start:
            return
        with self.lock:
            self.scans.append((start, end))
            receipt = rospy.Time.now().to_sec()
            self.receipt["lidar"] = receipt
            self.latencies["lidar"].append(receipt - end)
            try:
                metrics = self.duplicate_metrics(
                    message, receipt, start, end)
                if self.csv_writer is not None:
                    self.csv_writer.writerow(metrics)
                    self.csv_stream.flush()
            except (TypeError, ValueError) as exc:
                rospy.logwarn_throttle(
                    2.0, "[LIO input] duplicate check failed: %s", exc)

    @staticmethod
    def rate(stamps):
        unique = []
        for stamp in stamps:
            if not unique or stamp > unique[-1]:
                unique.append(stamp)
        if len(unique) < 2 or unique[-1] <= unique[0]:
            return None
        return (len(unique) - 1) / (unique[-1] - unique[0])

    @staticmethod
    def nearest_offset(stamps, target):
        if not stamps:
            return None
        index = bisect.bisect_left(stamps, target)
        candidates = []
        if index < len(stamps):
            candidates.append(stamps[index] - target)
        if index > 0:
            candidates.append(stamps[index - 1] - target)
        return min(candidates, key=abs)

    def publish_diagnostics(self, _event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            sources = {
                "raw_imu": list(self.raw_imu),
                "dense_imu": list(self.dense_imu),
                "ego": list(self.ego),
                "gps": list(self.gps),
            }
            scans = list(self.scans)
            receipt = dict(self.receipt)
            latencies = {
                name: list(values)
                for name, values in self.latencies.items()}
            duplicate_summary = {
                "scans_checked": self.scan_index,
                "duplicate_header_stamps": self.duplicate_stamp_count,
                "duplicate_full_frames": self.duplicate_frame_count,
                "max_intra_frame_duplicate_ratio": (
                    self.max_intra_duplicate_ratio),
                "last": self.last_duplicate_metrics,
                "csv_path": self.csv_path or None,
                "csv_mode": "overwrite_each_launch",
            }
        for name, stamps in sources.items():
            sources[name] = [
                stamp for stamp in stamps if now - stamp <= self.window]
        scans = [scan for scan in scans if now - scan[1] <= self.window]

        raw = sorted(sources["raw_imu"])
        dense = sorted(sources["dense_imu"])
        usable_scans = [
            scan for scan in scans if dense and scan[1] <= dense[-1]]
        raw_counts = []
        dense_counts = []
        start_offsets = []
        end_offsets = []
        durations = []
        for start, end in usable_scans:
            raw_counts.append(
                bisect.bisect_right(raw, end)
                - bisect.bisect_left(raw, start))
            dense_counts.append(
                bisect.bisect_right(dense, end)
                - bisect.bisect_left(dense, start))
            start_offset = self.nearest_offset(dense, start)
            end_offset = self.nearest_offset(dense, end)
            if start_offset is not None and end_offset is not None:
                start_offsets.append(abs(start_offset))
                end_offsets.append(abs(end_offset))
            durations.append(end - start)

        rates = {
            name + "_hz": self.rate(stamps)
            for name, stamps in sources.items()}
        rates["lidar_hz"] = self.rate(
            [scan[0] for scan in scans])
        raw_rate = rates["raw_imu_hz"] or 0.0
        dense_rate = rates["dense_imu_hz"] or 0.0
        lidar_rate = rates["lidar_hz"] or 0.0
        dense_coverage = (
            statistics.median(dense_counts) if dense_counts else 0.0)
        endpoint_p95 = (
            max(float(np.percentile(start_offsets, 95)),
                float(np.percentile(end_offsets, 95)))
            if start_offsets and end_offsets else None)
        if raw_rate < 5.0 or lidar_rate < 2.0:
            status = "critical"
        elif (
                dense_rate >= 80.0 and dense_coverage >= 10.0
                and endpoint_p95 is not None and endpoint_p95 <= 0.02):
            status = (
                "coverage_compensated"
                if raw_rate >= 40.0 and lidar_rate >= 8.0
                else "coverage_compensated_low_raw_rate")
        else:
            status = "degraded"

        payload = {
            "status": status,
            "rates": rates,
            "sensor_limits_hz": {
                "gps": 30.0, "raw_imu": 50.0,
                "lidar": 15.0, "lidar_recommended": 10.0},
            "configured_target_hz": {
                "gps": 30.0, "raw_imu": 50.0, "lidar": 10.0},
            "scan_duration_median_s": (
                statistics.median(durations) if durations else None),
            "raw_imu_samples_per_scan_median": (
                statistics.median(raw_counts) if raw_counts else None),
            "dense_imu_samples_per_scan_median": (
                dense_coverage if dense_counts else None),
            "dense_timestamp_grid_endpoint_offset_p95_s": endpoint_p95,
            "clock_offset_validated": False,
            "receipt_header_latency_median_s": {
                name: (statistics.median(values) if values else None)
                for name, values in latencies.items()},
            "receipt_header_latency_p95_s": {
                name: (float(np.percentile(values, 95)) if values else None)
                for name, values in latencies.items()},
            "source_age_sec": {
                name: (
                    None if name not in receipt
                    else max(0.0, now - receipt[name]))
                for name in (
                    "raw_imu", "dense_imu", "ego", "gps", "lidar")},
            "gps_present": bool(sources["gps"]),
            "lidar_duplicates": duplicate_summary,
            "window_s": self.window,
        }
        self.publisher.publish(
            String(data=json.dumps(payload, sort_keys=True)))


if __name__ == "__main__":
    try:
        LioInputMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
