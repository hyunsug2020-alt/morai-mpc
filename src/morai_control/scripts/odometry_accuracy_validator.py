#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import time

import rospy
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


NOTION_SPEC_URL = (
    "https://app.notion.com/p/35e04a348f1981acb3fbd30381dae434")
EXPECTED_VEHICLE_SPECS = {
    "mass": (1905.0, 0.1),
    "wheelbase": (3.000, 0.001),
    "vehicle_length": (4.635, 0.001),
    "vehicle_width": (1.892, 0.001),
    "front_overhang": (0.845, 0.001),
    "rear_overhang": (0.790, 0.001),
    "min_turn_radius": (5.87, 0.01),
    "physical_max_steer_deg": (40.0, 0.01),
    "wheel_radius": (0.36, 0.001),
    "lateral_accel_limit_mps2": (2.26, 0.01),
}


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odometry(msg):
    quaternion = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    ]
    return euler_from_quaternion(quaternion)[2]


class RunningMetrics:
    def __init__(self, name, align_to_ground_truth):
        self.name = name
        self.align_to_ground_truth = align_to_ground_truth
        self.alignment = None
        self.count = 0
        self.position_sq_sum = 0.0
        self.position_abs_sum = 0.0
        self.position_max = 0.0
        self.yaw_sq_sum = 0.0
        self.speed_sq_sum = 0.0
        self.first_stamp = None
        self.last_stamp = None

    def align(self, x, y, yaw, truth):
        if not self.align_to_ground_truth:
            return x, y, yaw
        if self.alignment is None:
            yaw_offset = wrap_angle(truth["yaw"] - yaw)
            cos_yaw = math.cos(yaw_offset)
            sin_yaw = math.sin(yaw_offset)
            tx = truth["x"] - (cos_yaw * x - sin_yaw * y)
            ty = truth["y"] - (sin_yaw * x + cos_yaw * y)
            self.alignment = {
                "x": tx,
                "y": ty,
                "yaw_rad": yaw_offset,
            }
        cos_yaw = math.cos(self.alignment["yaw_rad"])
        sin_yaw = math.sin(self.alignment["yaw_rad"])
        aligned_x = cos_yaw * x - sin_yaw * y + self.alignment["x"]
        aligned_y = sin_yaw * x + cos_yaw * y + self.alignment["y"]
        aligned_yaw = wrap_angle(yaw + self.alignment["yaw_rad"])
        return aligned_x, aligned_y, aligned_yaw

    def update(self, msg, truth, stamp):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_odometry(msg)
        x, y, yaw = self.align(x, y, yaw, truth)

        position_error = math.hypot(x - truth["x"], y - truth["y"])
        yaw_error = wrap_angle(yaw - truth["yaw"])
        estimate_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        speed_error = estimate_speed - truth["speed"]

        self.count += 1
        self.position_sq_sum += position_error * position_error
        self.position_abs_sum += position_error
        self.position_max = max(self.position_max, position_error)
        self.yaw_sq_sum += yaw_error * yaw_error
        self.speed_sq_sum += speed_error * speed_error
        self.first_stamp = stamp if self.first_stamp is None else self.first_stamp
        self.last_stamp = stamp

    def summary(self):
        if self.count == 0:
            return {
                "status": "no_samples",
                "sample_count": 0,
                "alignment": self.alignment,
            }
        return {
            "status": "measured",
            "sample_count": self.count,
            "position_rmse_m": math.sqrt(
                self.position_sq_sum / self.count),
            "position_mean_abs_m": self.position_abs_sum / self.count,
            "position_max_m": self.position_max,
            "yaw_rmse_deg": math.degrees(math.sqrt(
                self.yaw_sq_sum / self.count)),
            "speed_rmse_mps": math.sqrt(self.speed_sq_sum / self.count),
            "duration_s": max(0.0, self.last_stamp - self.first_stamp),
            "alignment": self.alignment,
        }


class OdometryAccuracyValidator:
    """Compare localization outputs with MORAI ground truth for validation only."""

    def __init__(self):
        rospy.init_node("odometry_accuracy_validator")
        self.truth_topic = rospy.get_param("~ground_truth_topic", "/Ego_topic")
        self.eskf_topic = rospy.get_param("~eskf_topic", "/eskf/odom")
        self.lio_topic = rospy.get_param(
            "~lio_sam_topic", "/lio_sam/mapping/odometry")
        self.output_file = os.path.expanduser(rospy.get_param(
            "~output_file", "/tmp/morai_odometry_accuracy.json"))
        self.max_time_delta = float(rospy.get_param(
            "~max_time_delta_s", 0.25))
        self.report_period = float(rospy.get_param("~report_period_s", 5.0))
        self.truth_heading_mode = rospy.get_param(
            "~ground_truth_heading_mode", "ros_yaw_deg")
        self.latest_truth = None
        self.metrics = {
            "eskf": RunningMetrics("eskf", align_to_ground_truth=False),
            "lio_sam": RunningMetrics(
                "lio_sam", align_to_ground_truth=True),
        }
        self.vehicle_checks = self._validate_vehicle_specs()

        rospy.Subscriber(
            self.truth_topic, EgoVehicleStatus, self.truth_callback,
            queue_size=50)
        rospy.Subscriber(
            self.eskf_topic, Odometry,
            lambda msg: self.estimate_callback("eskf", msg),
            queue_size=100)
        rospy.Subscriber(
            self.lio_topic, Odometry,
            lambda msg: self.estimate_callback("lio_sam", msg),
            queue_size=100)
        rospy.Timer(rospy.Duration(self.report_period), self.report)
        rospy.on_shutdown(self.write_summary)
        rospy.logwarn(
            "[Odom validation only] ground truth %s is not used by control",
            self.truth_topic)
        rospy.loginfo(
            "[Odom validation] ESKF=%s LIO-SAM=%s output=%s",
            self.eskf_topic, self.lio_topic, self.output_file)

    @staticmethod
    def _stamp(header):
        if header.stamp != rospy.Time():
            return header.stamp.to_sec()
        return rospy.Time.now().to_sec()

    def _validate_vehicle_specs(self):
        checks = {}
        all_passed = True
        for name, (expected, tolerance) in EXPECTED_VEHICLE_SPECS.items():
            configured = float(rospy.get_param("~" + name, float("nan")))
            error = abs(configured - expected)
            passed = math.isfinite(configured) and error <= tolerance
            checks[name] = {
                "configured": configured if math.isfinite(configured) else None,
                "expected": expected,
                "tolerance": tolerance,
                "passed": passed,
            }
            all_passed = all_passed and passed

        length_sum = (
            float(rospy.get_param("~front_overhang", 0.0))
            + float(rospy.get_param("~wheelbase", 0.0))
            + float(rospy.get_param("~rear_overhang", 0.0)))
        configured_length = float(
            rospy.get_param("~vehicle_length", float("nan")))
        geometry_passed = (
            math.isfinite(configured_length)
            and abs(length_sum - configured_length) <= 0.001)
        checks["geometry_sum"] = {
            "configured": length_sum,
            "expected": configured_length if math.isfinite(
                configured_length) else None,
            "tolerance": 0.001,
            "passed": geometry_passed,
        }
        checks["all_passed"] = all_passed and geometry_passed
        if not checks["all_passed"]:
            rospy.logerr("[Odom validation] vehicle specification mismatch")
        return checks

    def truth_callback(self, msg):
        yaw_deg = float(msg.heading)
        if self.truth_heading_mode == "morai_north_cw_deg":
            yaw = math.radians(90.0 - yaw_deg)
        else:
            yaw = math.radians(yaw_deg)
        self.latest_truth = {
            "source_stamp": self._stamp(msg.header),
            "arrival_time": time.monotonic(),
            "x": float(msg.position.x),
            "y": float(msg.position.y),
            "yaw": wrap_angle(yaw),
            "speed": math.hypot(msg.velocity.x, msg.velocity.y),
        }

    def estimate_callback(self, name, msg):
        if self.latest_truth is None:
            return
        # MORAI rosbridge and UDP sensors can use different header clocks.
        # Compare callback arrival times so valid pairs are not discarded due
        # to a constant or drifting inter-source timestamp offset.
        arrival_time = time.monotonic()
        if (
                arrival_time - self.latest_truth["arrival_time"]
                > self.max_time_delta):
            return
        self.metrics[name].update(msg, self.latest_truth, arrival_time)

    def report(self, _event):
        parts = []
        for name, metric in self.metrics.items():
            summary = metric.summary()
            if summary["status"] == "measured":
                parts.append(
                    "%s n=%d pos=%.3fm yaw=%.2fdeg speed=%.3fm/s"
                    % (name, summary["sample_count"],
                       summary["position_rmse_m"],
                       summary["yaw_rmse_deg"],
                       summary["speed_rmse_mps"]))
            else:
                parts.append("%s waiting" % name)
        rospy.loginfo("[Odom validation] %s", " | ".join(parts))
        self.write_summary()

    def write_summary(self):
        result = {
            "purpose": "validation_only_not_used_for_control",
            "notion_vehicle_spec": NOTION_SPEC_URL,
            "vehicle_spec_checks": self.vehicle_checks,
            "ground_truth_topic": self.truth_topic,
            "ground_truth_heading_mode": self.truth_heading_mode,
            "synchronization": "callback_arrival_monotonic",
            "estimates": {
                name: metric.summary()
                for name, metric in self.metrics.items()
            },
        }
        output_dir = os.path.dirname(self.output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        temporary = self.output_file + ".tmp"
        try:
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump(result, stream, indent=2, sort_keys=True)
                stream.write("\n")
            os.replace(temporary, self.output_file)
        except OSError as exc:
            rospy.logerr_throttle(
                5.0, "[Odom validation] cannot write summary: %s", exc)


if __name__ == "__main__":
    try:
        OdometryAccuracyValidator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
