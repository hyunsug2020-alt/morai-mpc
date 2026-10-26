#!/usr/bin/env python3
"""Replay recorded wheel/IMU inputs with deterministic random IMU faults.

The estimator-facing vehicle message contains only usable velocity and wheel
angle information; ground-truth pose and heading are zeroed.  The unmodified
vehicle message is published on a separate validation-only topic so an
external recorder can score the resulting pure odometry.
"""

import argparse
import copy
import json
import math
import os
import time

import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import Quaternion
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def wait_for_subscribers(publishers, timeout_sec=10.0):
    deadline = time.monotonic() + timeout_sec
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if all(publisher.get_num_connections() > 0
               for publisher in publishers):
            return
        time.sleep(0.05)
    missing = [publisher.resolved_name for publisher in publishers
               if publisher.get_num_connections() <= 0]
    raise RuntimeError("no subscribers for: {}".format(
        ", ".join(missing)))


def zero_forbidden_vehicle_fields(message):
    message.position.x = 0.0
    message.position.y = 0.0
    message.position.z = 0.0
    message.heading = 0.0


def perturb_imu(message, rng, args):
    for axis in ("x", "y", "z"):
        value = getattr(message.linear_acceleration, axis)
        setattr(message.linear_acceleration, axis, value + float(
            rng.normal(0.0, args.accel_noise_std)))
        value = getattr(message.angular_velocity, axis)
        setattr(message.angular_velocity, axis, value + float(
            rng.normal(0.0, args.gyro_noise_std)))

    quaternion = message.orientation
    roll, pitch, yaw = euler_from_quaternion([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    yaw += float(rng.normal(0.0, args.orientation_yaw_noise_std))

    spike = bool(rng.random() < args.spike_probability)
    if spike:
        for axis in ("x", "y", "z"):
            value = getattr(message.linear_acceleration, axis)
            setattr(message.linear_acceleration, axis, value + float(
                rng.normal(0.0, args.accel_spike_std)))
        message.angular_velocity.z += float(
            rng.normal(0.0, args.gyro_spike_std))
        yaw += float(rng.normal(0.0, args.orientation_yaw_spike_std))

    noisy_quaternion = quaternion_from_euler(roll, pitch, yaw)
    message.orientation = Quaternion(*noisy_quaternion)
    return spike


def replay(args):
    rng = np.random.default_rng(args.seed)
    rospy.init_node("pure_odometry_noise_replay", anonymous=True)
    vehicle_publisher = rospy.Publisher(
        args.vehicle_topic, EgoVehicleStatus, queue_size=1000)
    imu_publisher = rospy.Publisher(
        args.imu_topic, Imu, queue_size=1000)
    truth_publisher = rospy.Publisher(
        args.truth_topic, EgoVehicleStatus, queue_size=1000)
    wait_for_subscribers([
        vehicle_publisher, imu_publisher, truth_publisher])

    counts = {
        "vehicle_published": 0,
        "imu_published": 0,
        "imu_spikes": 0,
    }
    with rosbag.Bag(args.bag) as bag:
        for topic, original, _ in bag.read_messages(
                topics=["/Ego_topic", "/imu/data"]):
            if rospy.is_shutdown():
                break
            if topic == "/Ego_topic":
                truth_publisher.publish(original)
                vehicle = copy.deepcopy(original)
                zero_forbidden_vehicle_fields(vehicle)
                vehicle_publisher.publish(vehicle)
                counts["vehicle_published"] += 1
            else:
                imu = copy.deepcopy(original)
                if perturb_imu(imu, rng, args):
                    counts["imu_spikes"] += 1
                imu_publisher.publish(imu)
                counts["imu_published"] += 1
            if args.publish_delay > 0.0:
                time.sleep(args.publish_delay)
    time.sleep(args.settle_time)
    return {
        "bag": os.path.abspath(args.bag),
        "seed": args.seed,
        "noise": {
            "accel_std_mps2": args.accel_noise_std,
            "gyro_std_radps": args.gyro_noise_std,
            "orientation_yaw_std_rad": args.orientation_yaw_noise_std,
            "spike_probability": args.spike_probability,
            "accel_spike_std_mps2": args.accel_spike_std,
            "gyro_spike_std_radps": args.gyro_spike_std,
            "orientation_yaw_spike_std_rad": (
                args.orientation_yaw_spike_std),
        },
        "estimator_vehicle_fields": ["velocity.x", "wheel_angle"],
        "ground_truth_pose_zeroed_for_estimator": True,
        **counts,
    }


def nonnegative(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and >= 0")
    return parsed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--seed", type=int, default=20260826)
    parser.add_argument("--vehicle-topic", default="/Competition_topic_noise")
    parser.add_argument("--imu-topic", default="/imu/data_noise")
    parser.add_argument("--truth-topic", default="/validation/ego_truth")
    parser.add_argument("--accel-noise-std", type=nonnegative, default=0.18)
    parser.add_argument("--gyro-noise-std", type=nonnegative, default=0.008)
    parser.add_argument(
        "--orientation-yaw-noise-std", type=nonnegative, default=0.025)
    parser.add_argument("--spike-probability", type=nonnegative, default=0.004)
    parser.add_argument("--accel-spike-std", type=nonnegative, default=8.0)
    parser.add_argument("--gyro-spike-std", type=nonnegative, default=0.8)
    parser.add_argument(
        "--orientation-yaw-spike-std", type=nonnegative, default=0.8)
    parser.add_argument("--publish-delay", type=nonnegative, default=0.0005)
    parser.add_argument("--settle-time", type=nonnegative, default=2.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    if args.spike_probability > 1.0:
        parser.error("--spike-probability must be <= 1")

    result = replay(args)
    payload = json.dumps(result, indent=2, sort_keys=True)
    print(payload)
    if args.output:
        output = os.path.abspath(os.path.expanduser(args.output))
        os.makedirs(os.path.dirname(output), exist_ok=True)
        with open(output, "w", encoding="utf-8") as stream:
            stream.write(payload + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
