#!/usr/bin/env python3
"""Replay recorded MORAI motion through the production ESKF ROS wrapper.

Ground truth is used only by this validation process to synthesize a noisy GPS
receiver and score output. The estimator receives recorded IMU and pure
odometry, continuous additional sensor noise, GPS outliers, and deterministic
random GPS-shadow intervals.
"""

import argparse
import bisect
import copy
import glob
import json
import math
import os
import threading
import time

import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import Quaternion
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Transformer
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def random_nonoverlapping_intervals(rng, duration, count=6):
    intervals = []
    attempts = 0
    while len(intervals) < count and attempts < 500:
        attempts += 1
        length = float(rng.uniform(8.0, min(30.0, duration * 0.12)))
        start = float(rng.uniform(18.0, duration - length - 8.0))
        candidate = (start, start + length)
        if any(
                candidate[0] < old_end + 5.0
                and candidate[1] > old_start - 5.0
                for old_start, old_end in intervals):
            continue
        intervals.append(candidate)
    return tuple(sorted(intervals))


def in_intervals(value, intervals):
    return any(start <= value <= end for start, end in intervals)


class OutputMetrics:
    def __init__(self, truth_stamps, truth_positions, truth_yaws,
                 start_stamp, dropouts):
        self.truth_stamps = truth_stamps
        self.truth_positions = truth_positions
        self.truth_yaws = truth_yaws
        self.start_stamp = start_stamp
        self.dropouts = dropouts
        self.lock = threading.Lock()
        self.errors = []
        self.outage_errors = []
        self.healthy_errors = []
        self.yaw_errors = []
        self.trace = []
        self.timeline = []
        self.last_timeline_stamp = -math.inf
        self.latest_diagnostics = None

    def _nearest_truth(self, stamp):
        index = bisect.bisect_left(self.truth_stamps, stamp)
        candidates = []
        if index < len(self.truth_stamps):
            candidates.append(index)
        if index > 0:
            candidates.append(index - 1)
        if not candidates:
            return None
        best = min(candidates, key=lambda item: abs(
            self.truth_stamps[item] - stamp))
        if abs(self.truth_stamps[best] - stamp) > 0.12:
            return None
        return best

    def odometry_callback(self, message):
        stamp = message.header.stamp.to_sec()
        index = self._nearest_truth(stamp)
        if index is None or stamp - self.start_stamp < 5.0:
            return
        position = np.array([
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        ])
        quaternion = message.pose.pose.orientation
        yaw = euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]
        position_error = float(np.linalg.norm(
            position - self.truth_positions[index]))
        yaw_error = abs(wrap_angle(yaw - self.truth_yaws[index]))
        speed = math.hypot(
            message.twist.twist.linear.x,
            message.twist.twist.linear.y)
        relative_time = stamp - self.start_stamp
        with self.lock:
            self.errors.append(position_error)
            self.yaw_errors.append(yaw_error)
            is_outage = in_intervals(relative_time, self.dropouts)
            diagnostics = self.latest_diagnostics or {}
            counters = diagnostics.get("counters", {})
            sample = (
                relative_time, position_error, is_outage, speed,
                diagnostics.get("gps_mode"),
                counters.get("gps_reacquired"),
                counters.get("velocity_reanchored"),
                counters.get("gps_rejected"),
            )
            self.trace.append(sample)
            if relative_time - self.last_timeline_stamp >= 1.0:
                self.timeline.append(sample)
                self.last_timeline_stamp = relative_time
            if is_outage:
                self.outage_errors.append(position_error)
            else:
                self.healthy_errors.append(position_error)

    def diagnostics_callback(self, message):
        with self.lock:
            self.latest_diagnostics = json.loads(message.data)

    @staticmethod
    def _summary(values):
        array = np.asarray(values, dtype=float)
        if not len(array):
            return None
        return {
            "samples": int(len(array)),
            "rmse_m": float(np.sqrt(np.mean(array**2))),
            "p95_m": float(np.percentile(array, 95.0)),
            "max_m": float(np.max(array)),
        }

    def result(self):
        with self.lock:
            yaw = np.asarray(self.yaw_errors, dtype=float)
            interval_summaries = []
            for start, end in self.dropouts:
                values = [
                    sample[1] for sample in self.trace
                    if start <= sample[0] <= end]
                summary = self._summary(values)
                interval_summaries.append({
                    "start_s": start,
                    "end_s": end,
                    "duration_s": end - start,
                    "metrics": summary,
                })
            top_errors = sorted(
                self.trace, key=lambda sample: sample[1], reverse=True)[:20]
            timeline = [
                sample for sample in self.timeline
                if sample[1] >= 5.0
                or in_intervals(sample[0], self.dropouts)]
            return {
                "overall": self._summary(self.errors),
                "gps_shadow": self._summary(self.outage_errors),
                "gps_available": self._summary(self.healthy_errors),
                "yaw_rmse_deg": (
                    None if not len(yaw) else math.degrees(float(
                        np.sqrt(np.mean(yaw**2))))),
                "dropout_metrics": interval_summaries,
                "top_errors": [
                    {
                        "elapsed_s": sample[0],
                        "error_m": sample[1],
                        "gps_shadow": sample[2],
                        "eskf_speed_mps": sample[3],
                        "gps_mode": sample[4],
                        "gps_reacquired": sample[5],
                        "velocity_reanchored": sample[6],
                        "gps_rejected": sample[7],
                    }
                    for sample in top_errors],
                "fault_timeline_1hz": [
                    {
                        "elapsed_s": sample[0],
                        "error_m": sample[1],
                        "gps_shadow": sample[2],
                        "eskf_speed_mps": sample[3],
                        "gps_mode": sample[4],
                        "gps_reacquired": sample[5],
                        "velocity_reanchored": sample[6],
                        "gps_rejected": sample[7],
                    }
                    for sample in timeline],
                "diagnostics": copy.deepcopy(self.latest_diagnostics),
            }


def load_truth(bag_paths):
    samples = []
    for bag_path in bag_paths:
        with rosbag.Bag(bag_path) as bag:
            for _, message, _ in bag.read_messages(topics=["/Ego_topic"]):
                stamp = message.header.stamp.to_sec()
                samples.append((
                    stamp,
                    np.array([
                        float(message.position.x),
                        float(message.position.y),
                    ]),
                    math.radians(float(message.heading))))
    samples.sort(key=lambda sample: sample[0])
    unique = []
    for sample in samples:
        if not unique or sample[0] > unique[-1][0]:
            unique.append(sample)
    return (
        [sample[0] for sample in unique],
        np.asarray([sample[1] for sample in unique]),
        np.asarray([sample[2] for sample in unique]))


def replay(args):
    bag_paths = sorted(glob.glob(args.bag_glob))
    if not bag_paths:
        raise RuntimeError("no bags match: " + args.bag_glob)
    truth_stamps, truth_positions, truth_yaws = load_truth(bag_paths)
    if len(truth_stamps) < 100:
        raise RuntimeError("insufficient ground-truth samples")
    start_stamp = truth_stamps[0]
    duration = truth_stamps[-1] - start_stamp
    rng = np.random.default_rng(args.seed)
    dropouts = random_nonoverlapping_intervals(rng, duration)
    if len(dropouts) < 3:
        raise RuntimeError("could not construct enough dropout intervals")

    with open(args.sensor_config, encoding="utf-8-sig") as stream:
        sensor_config = json.load(stream)
    gps_position = sensor_config["GPSList"][0]["pos"]
    gps_lever = np.array([
        float(gps_position["x"]),
        float(gps_position["y"]),
    ])

    rospy.init_node("eskf_rosbag_fault_stimulus")
    imu_publisher = rospy.Publisher("/imu", Imu, queue_size=100)
    odometry_publisher = rospy.Publisher(
        "/odometry/pure", Odometry, queue_size=100)
    gps_publisher = rospy.Publisher("/gps", GPSMessage, queue_size=30)
    odometry_diagnostics_publisher = rospy.Publisher(
        "/pure_odometry/diagnostics", String, queue_size=10)
    metrics = OutputMetrics(
        truth_stamps,
        truth_positions,
        truth_yaws,
        start_stamp,
        dropouts)
    rospy.Subscriber(
        "/eskf/odom", Odometry, metrics.odometry_callback,
        queue_size=500, tcp_nodelay=True)
    rospy.Subscriber(
        "/eskf/diagnostics", String, metrics.diagnostics_callback,
        queue_size=10)
    time.sleep(1.0)

    forward = Transformer.from_crs(
        "EPSG:4326", "EPSG:32652", always_xy=True)
    inverse = Transformer.from_crs(
        "EPSG:32652", "EPSG:4326", always_xy=True)
    base_easting, base_northing = forward.transform(126.7745, 37.2425)
    last_gps_stamp = -math.inf
    topic_order = [
        "/imu/data",
        "/odometry/pure",
        "/pure_odometry/diagnostics",
        "/Ego_topic",
    ]
    gps_outliers = 0
    imu_spikes = 0

    for bag_path in bag_paths:
        with rosbag.Bag(bag_path) as bag:
            for topic, original, _ in bag.read_messages(topics=topic_order):
                if rospy.is_shutdown():
                    break
                if topic == "/pure_odometry/diagnostics":
                    odometry_diagnostics_publisher.publish(original)
                    time.sleep(args.publish_delay)
                    continue
                stamp = original.header.stamp.to_sec()
                relative_time = stamp - start_stamp
                if topic == "/imu/data":
                    message = copy.deepcopy(original)
                    message.linear_acceleration.x += float(
                        rng.normal(0.0, 0.18))
                    message.linear_acceleration.y += float(
                        rng.normal(0.0, 0.18))
                    message.angular_velocity.z += float(
                        rng.normal(0.0, 0.008))
                    quaternion = message.orientation
                    roll, pitch, yaw = euler_from_quaternion([
                        quaternion.x, quaternion.y,
                        quaternion.z, quaternion.w])
                    yaw = wrap_angle(yaw + float(rng.normal(0.0, 0.025)))
                    if rng.random() < 0.004:
                        message.linear_acceleration.x += float(
                            rng.normal(0.0, 8.0))
                        message.linear_acceleration.y += float(
                            rng.normal(0.0, 8.0))
                        message.angular_velocity.z += float(
                            rng.normal(0.0, 0.8))
                        yaw = wrap_angle(yaw + float(rng.normal(0.0, 0.8)))
                        imu_spikes += 1
                    noisy_quaternion = quaternion_from_euler(roll, pitch, yaw)
                    message.orientation = Quaternion(*noisy_quaternion)
                    imu_publisher.publish(message)
                elif topic == "/odometry/pure":
                    odometry_publisher.publish(original)
                elif stamp - last_gps_stamp >= 0.195:
                    last_gps_stamp = stamp
                    if in_intervals(relative_time, dropouts):
                        continue
                    base_position = np.array([
                        float(original.position.x),
                        float(original.position.y),
                    ])
                    yaw = math.radians(float(original.heading))
                    rotation = np.array([
                        [math.cos(yaw), -math.sin(yaw)],
                        [math.sin(yaw), math.cos(yaw)],
                    ])
                    sensor_local = (
                        base_position + rotation @ gps_lever
                        + rng.normal(0.0, 1.5, 2))
                    if rng.random() < 0.03:
                        direction = float(rng.uniform(-math.pi, math.pi))
                        sensor_local += float(rng.uniform(18.0, 35.0)) * np.array([
                            math.cos(direction), math.sin(direction)])
                        gps_outliers += 1
                    longitude, latitude = inverse.transform(
                        base_easting + sensor_local[0],
                        base_northing + sensor_local[1])
                    gps = GPSMessage()
                    gps.header.stamp = original.header.stamp
                    gps.latitude = latitude
                    gps.longitude = longitude
                    gps.eastOffset = base_easting
                    gps.northOffset = base_northing
                    gps.status = 0
                    gps_publisher.publish(gps)
                time.sleep(args.publish_delay)

    time.sleep(3.0)
    result = metrics.result()
    diagnostics = result.pop("diagnostics")
    if diagnostics is None:
        raise RuntimeError("no ESKF diagnostics received")
    counters = diagnostics["counters"]
    result.update({
        "bags": bag_paths,
        "duration_s": duration,
        "dropouts": dropouts,
        "injected_gps_outliers": gps_outliers,
        "injected_imu_spikes": imu_spikes,
        "gps_reacquired": counters["gps_reacquired"],
        "gps_rejected": counters["gps_rejected"],
        "odometry_position_accepted": counters["odometry_accepted"],
        "odometry_speed_accepted": counters["wheel_speed_accepted"],
        "invalid_measurements": counters["invalid_measurements"],
        "odometry_alignment_ready": diagnostics[
            "odometry_alignment_ready"],
        "position_std_m": diagnostics["position_std_m"],
    })
    overall = result["overall"] or {}
    shadow = result["gps_shadow"] or {}
    healthy = result["gps_available"] or {}
    result["passed"] = bool(
        overall.get("samples", 0) >= 1000
        and shadow.get("samples", 0) >= 200
        and overall.get("rmse_m", math.inf) < 5.0
        and shadow.get("p95_m", math.inf) < 8.0
        and shadow.get("max_m", math.inf) < 20.0
        and healthy.get("p95_m", math.inf) < 4.0
        and result["yaw_rmse_deg"] < 3.0
        and result["odometry_position_accepted"] > 10
        and result["odometry_speed_accepted"] > 100
        and result["invalid_measurements"] == 0
        and result["odometry_alignment_ready"])
    return result


def main():
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag-glob",
        default=os.path.join(
            package_dir, "logs", "bags",
            "pure_odometry_debug_2026-08-03-16-*-53_*.bag"))
    parser.add_argument(
        "--sensor-config",
        default=os.path.join(package_dir, "config", "morai_sensor.json"))
    parser.add_argument("--seed", type=int, default=20260803)
    parser.add_argument("--publish-delay", type=float, default=0.001)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    result = replay(args)
    payload = json.dumps(result, indent=2, sort_keys=True)
    print(payload)
    if args.output:
        output_path = os.path.abspath(os.path.expanduser(args.output))
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as stream:
            stream.write(payload + "\n")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
