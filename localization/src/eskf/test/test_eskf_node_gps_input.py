#!/usr/bin/env python3

import importlib.util
import json
import math
import os
import threading
import unittest
from collections import deque
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np


SCRIPT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "scripts", "eskf_node.py"))
SPEC = importlib.util.spec_from_file_location("eskf_node_gps_input_test", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DummyGpsMode:
    def __init__(self):
        self.rejections = 0

    def rejected(self, _stamp=None):
        self.rejections += 1


class DummyWheelFilter:
    def __init__(self):
        self.initialized = True
        self.counters = {
            "invalid_measurements": 0,
            "wheel_speed_rejected": 0,
        }
        self.speed = None

    def update_wheel_speed(self, speed, _variance):
        self.speed = speed
        return False, None


class EskfImuOrientationTest(unittest.TestCase):
    @staticmethod
    def message(stamp=1.02, yaw=0.0, covariance=0.01):
        message = MODULE.Imu()
        message.header.stamp = MODULE.rospy.Time.from_sec(stamp)
        quaternion = MODULE.quaternion_from_euler(0.0, 0.0, yaw)
        (message.orientation.x, message.orientation.y,
         message.orientation.z, message.orientation.w) = quaternion
        message.orientation_covariance[0] = covariance
        message.orientation_covariance[8] = 0.01
        message.linear_acceleration.x = 0.2
        message.angular_velocity.z = 0.1
        return message

    def node(self, initialized=True):
        # Exercise the production callback/filter without starting ROS nodes
        # or connecting to the user's active ROS master.
        with ExitStack() as stack:
            stack.enter_context(patch.object(
                MODULE.rospy, "get_param", side_effect=lambda key, default: default))
            stack.enter_context(patch.object(
                MODULE.rospy, "has_param", return_value=False))
            for name in ("init_node", "Publisher", "Subscriber", "Timer",
                         "loginfo", "logwarn"):
                stack.enter_context(patch.object(MODULE.rospy, name))
            node = MODULE.ESKFNode()
        node.publish = Mock()
        if initialized:
            node.filter.initialize([0.0, 0.0], 0.1)
            node.last_imu_yaw = 0.1
            node.last_predict_stamp = MODULE.rospy.Time.from_sec(1.0)
            node.gps_replay.seed(1.0, node.filter.snapshot())
        return node

    def test_unavailable_identity_orientation_is_rejected(self):
        self.assertFalse(MODULE.ESKFNode._valid_quaternion(
            self.message(covariance=-1.0)))

    def test_zero_and_positive_covariance_remain_compatible(self):
        for covariance in (0.0, 0.01):
            with self.subTest(covariance=covariance):
                node = self.node()
                node.imu_callback(self.message(yaw=0.12, covariance=covariance))
                self.assertEqual(node.filter.counters["yaw_accepted"], 1)
                self.assertAlmostEqual(node.last_imu_yaw, 0.12)
                node.publish.assert_called_once()

    def test_unavailable_orientation_preserves_prediction_and_replay(self):
        node = self.node()
        before = node.filter.snapshot()
        node.imu_callback(self.message(covariance=-1.0))
        after = node.filter.snapshot()

        self.assertEqual(node.filter.counters["yaw_accepted"], 0)
        self.assertEqual(node.filter.counters["yaw_rejected"], 0)
        self.assertEqual(node.last_imu_yaw, 0.1)
        self.assertAlmostEqual(node.filter.x[node.filter.YAW], 0.102)
        self.assertGreater(node.filter.x[node.filter.VX], 0.0)
        node.publish.assert_called_once()
        event = node.gps_replay.frames[-1].event
        self.assertIsNone(event.yaw_measurement)
        self.assertIsNone(event.yaw_variance)
        node.filter.restore(before)
        node._apply_replay_event(event)
        np.testing.assert_allclose(node.filter.x, after["x"])
        np.testing.assert_allclose(node.filter.p, after["p"])

        # The first valid orientation resumes normal correction immediately.
        node.imu_callback(self.message(stamp=1.04, yaw=0.12))
        self.assertEqual(node.filter.counters["yaw_accepted"], 1)

    def test_unavailable_orientation_does_not_seed_initial_heading(self):
        node = self.node(initialized=False)
        node.imu_callback(self.message(covariance=-1.0))
        self.assertIsNone(node.last_imu_yaw)
        self.assertAlmostEqual(node.last_imu_stamp_sec, 1.02)
        np.testing.assert_allclose(node.last_imu_input, [0.2, 0.0, 0.1])
        self.assertFalse(node.filter.initialized)

    def test_valid_but_outlying_orientation_still_uses_nis_gate(self):
        node = self.node()
        with patch.object(MODULE.rospy, "logwarn_throttle"):
            node.imu_callback(self.message(yaw=3.0))
        self.assertEqual(node.filter.counters["yaw_accepted"], 0)
        self.assertEqual(node.filter.counters["yaw_rejected"], 1)
        self.assertAlmostEqual(node.filter.x[node.filter.YAW], 0.102)


class EskfGpsInputTest(unittest.TestCase):
    def test_zero_zero_fix_is_rejected_before_filter_initialization(self):
        node = MODULE.ESKFNode.__new__(MODULE.ESKFNode)
        node.lock = threading.RLock()
        node.invalid_gps_status = -1
        node.minimum_abs_gps_latitude = 1.0
        node.minimum_abs_gps_longitude = 1.0
        node.maximum_local_gps_radius = 100000.0
        node.utm_zone = 52
        node.filter = SimpleNamespace(counters={
            "invalid_measurements": 0,
            "gps_rejected": 0,
        })
        node.gps_mode = DummyGpsMode()
        message = SimpleNamespace(
            latitude=0.0,
            longitude=0.0,
            eastOffset=302595.0,
            northOffset=4124145.0,
            status=0,
        )

        with patch.object(MODULE.rospy, "logwarn_throttle"):
            node.gps_callback(message)

        self.assertEqual(node.filter.counters["invalid_measurements"], 1)
        self.assertEqual(node.filter.counters["gps_rejected"], 1)
        self.assertEqual(node.gps_mode.rejections, 1)

    def test_competition_wheel_update_never_reads_lateral_velocity(self):
        class CompetitionVelocity:
            x = -7.5

            @property
            def y(self):
                raise AssertionError("competition velocity.y is unavailable")

        class Stamp:
            def to_sec(self):
                return 1.0

        node = MODULE.ESKFNode.__new__(MODULE.ESKFNode)
        node.lock = threading.RLock()
        node.wheel_aiding_enabled = True
        node.filter = DummyWheelFilter()
        node.last_wheel_source_stamp = None
        node.last_predict_stamp = None
        node.wheel_max_message_lag = 0.5
        node.wheel_speed_scale = 1.0
        node.wheel_speed_variance = 0.1
        node.wheel_stationary_since = None
        node.wheel_stationary_last_stamp = None
        node.wheel_motion_since = None
        node.wheel_motion_sign = 0
        node.wheel_motion_last_stamp = None
        node._stamp_or_now = lambda _header: Stamp()
        message = SimpleNamespace(
            velocity=CompetitionVelocity(),
            wheel_angle=0.0,
            header=SimpleNamespace(),
        )

        node.vehicle_state_callback(message)

        self.assertEqual(node.filter.speed, -7.5)
        self.assertEqual(node.last_wheel_speed, -7.5)

    def test_reacquisition_does_not_apply_gps_before_stable_window(self):
        node = MODULE.ESKFNode.__new__(MODULE.ESKFNode)
        node.lock = threading.RLock()
        node.invalid_gps_status = -1
        node.minimum_abs_gps_latitude = 1.0
        node.minimum_abs_gps_longitude = 1.0
        node.maximum_local_gps_radius = 100000.0
        node.utm_zone = 52
        node.filter = SimpleNamespace(
            initialized=True,
            counters={"invalid_measurements": 0, "gps_rejected": 0})
        node.gps_mode = SimpleNamespace(received=lambda _stamp: None)
        node.gps_reacquisition_active = True
        node.gps_reacquisition_samples = 20
        node.gps_reacquisition_buffer = deque(maxlen=40)
        node.last_gps_message_time = 0.0
        node.last_gps_accept_time = 0.0
        node._stamp_or_now = lambda _header: SimpleNamespace(
            to_sec=lambda: 10.0)
        message = SimpleNamespace(
            latitude=37.0, longitude=127.0,
            eastOffset=0.0, northOffset=0.0, status=0,
            header=SimpleNamespace())

        with patch.object(
                MODULE, "latlon_to_utm", return_value=(10.0, 20.0)):
            node.gps_callback(message)

        self.assertEqual(len(node.gps_reacquisition_buffer), 1)
        self.assertTrue(node.gps_reacquisition_active)


class EskfWheelMotionTest(unittest.TestCase):
    @staticmethod
    def node():
        return EskfImuOrientationTest().node()

    @staticmethod
    def wheel_message(stamp, speed):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=MODULE.rospy.Time.from_sec(stamp)),
            velocity=SimpleNamespace(x=speed), wheel_angle=0.0)

    @staticmethod
    def imu_message(stamp, yaw=0.0):
        message = EskfImuOrientationTest.message(stamp=stamp, yaw=yaw)
        message.linear_acceleration.x = 0.0
        message.angular_velocity.z = 0.0
        return message

    def test_wheel_and_imu_track_reverse_without_gps(self):
        # Real callbacks, filter and replay; only ROS transport is mocked.
        for wheel_speed in (-2.0, 2.0):
            with self.subTest(wheel_speed=wheel_speed):
                node = self.node()
                node.filter.initialize([0.0, 0.0], math.pi / 2.0)
                node.gps_replay.seed(1.0, node.filter.snapshot())
                node.vehicle_state_callback(self.wheel_message(1.0, wheel_speed))
                start, _ = node.filter.base_state()
                for index in range(1, 151):
                    stamp = 1.0 + index * 0.02
                    node.imu_callback(self.imu_message(stamp, math.pi / 2.0))
                    node.vehicle_state_callback(self.wheel_message(stamp, wheel_speed))
                position, velocity = node.filter.base_state()
                self.assertAlmostEqual(position[1] - start[1], wheel_speed * 3.0, delta=0.1)
                self.assertAlmostEqual(position[0] - start[0], 0.0, delta=0.01)
                self.assertAlmostEqual(velocity[1], wheel_speed, delta=0.01)
                self.assertAlmostEqual(
                    node.filter.x[node.filter.YAW], math.pi / 2.0, delta=1e-5)
                self.assertEqual(node.last_wheel_speed, wheel_speed)
                self.assertEqual(node.filter.counters["gps_accepted"], 0)
                self.assertEqual(node.filter.counters["zupt_accepted"], 0)
                self.assertEqual(node.gps_replay.frames[-1].event.forward_speed, wheel_speed)

                with patch.object(MODULE.rospy.Time, "now", return_value=(
                        MODULE.rospy.Time.from_sec(4.0))):
                    node.publish_diagnostics(None)
                payload = json.loads(node.diagnostics_pub.publish.call_args[0][0].data)
                self.assertEqual(payload["wheel_speed_mps"], wheel_speed)
                self.assertEqual(payload["zupt_blocked_by_wheel"], 0)

    def test_rejected_moving_wheel_blocks_zupt_including_slam_hint(self):
        for wheel_speed in (-2.0, 2.0, -0.25, 0.25):
            for slam_hint in (False, True):
                with self.subTest(wheel_speed=wheel_speed, slam_hint=slam_hint):
                    node = self.node()
                    node.filter.initialize([0.0, 0.0], 0.0)
                    node.gps_replay.seed(1.0, node.filter.snapshot())
                    node.zupt_candidate_since = 0.0
                    node.slam_stationary_hint_time = 1.0 if slam_hint else None
                    with patch.object(node.filter, "update_wheel_speed", return_value=(False, None)):
                        for index in range(50):
                            stamp = 1.02 + index * 0.02
                            node.vehicle_state_callback(self.wheel_message(stamp, wheel_speed))
                            node.imu_callback(self.imu_message(stamp))
                    self.assertEqual(node.zupt_blocked_by_wheel, 50)
                    self.assertIsNone(node.zupt_candidate_since)
                    self.assertIsNone(node.last_wheel_accept_time)
                    self.assertEqual(node.filter.counters["zupt_accepted"], 0)
                    self.assertTrue(all(not frame.event.apply_zupt for frame in node.gps_replay.frames
                                        if frame.event is not None))

    def test_wheel_zupt_guard_freshness_threshold_and_enabled_flag(self):
        cases = (
            (0.52, -2.0, True, True),  # Exactly 0.5 s old: still moving.
            (0.519, -2.0, True, False),
            (1.021, -2.0, True, False),  # Future stamp is not fresh.
            (None, -2.0, True, False),
            (1.02, -0.249, True, False),
            (1.02, 0.0, True, False),
            (1.02, -2.0, False, False),
        )
        for stamp, speed, enabled, blocked in cases:
            with self.subTest(stamp=stamp, speed=speed, enabled=enabled):
                node = self.node()
                node.filter.initialize([0.0, 0.0], 0.0)
                node.last_wheel_message_time = stamp
                node.last_wheel_speed = speed
                node.wheel_aiding_enabled = enabled
                node.zupt_candidate_since = 0.0
                node.imu_callback(self.imu_message(1.02))
                self.assertEqual(node.zupt_blocked_by_wheel, int(blocked))
                self.assertEqual(node.filter.counters["zupt_accepted"], int(not blocked))
                self.assertEqual(node.gps_replay.frames[-1].event.apply_zupt, not blocked)

    def test_odometry_zupt_guard_remains_active(self):
        node = self.node()
        node.filter.initialize([0.0, 0.0], 0.0)
        node.last_odometry_message_time = 1.02
        node.last_odometry_speed = -2.0
        node.zupt_candidate_since = 0.0
        node.imu_callback(self.imu_message(1.02))
        self.assertEqual(node.zupt_blocked_by_odometry, 1)
        self.assertEqual(node.zupt_blocked_by_wheel, 0)
        self.assertEqual(node.filter.counters["zupt_accepted"], 0)


if __name__ == "__main__":
    unittest.main()
