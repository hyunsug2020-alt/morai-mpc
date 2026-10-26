import unittest
from unittest.mock import Mock, patch

import numpy as np

import test_eskf_node_gps_input as node_test


MODULE = node_test.MODULE


class SlowWheelMotionTest(unittest.TestCase):
    def node(self):
        node = node_test.EskfImuOrientationTest().node()
        node.filter.initialize([0.0, 0.0], 0.0)
        node.tf_broadcaster = None
        node.odom_pub = Mock()
        node.publish = MODULE.ESKFNode.publish.__get__(node)
        return node

    @staticmethod
    def wheel(node, stamp, speed):
        node.vehicle_state_callback(node_test.EskfWheelMotionTest.wheel_message(stamp, speed))

    def test_sustained_slow_motion_blocks_zupt_and_preserves_output(self):
        for speed in (-0.05, 0.05, -0.06, 0.09, -0.16, -0.24, -0.6, 2.0, -2.0):
            with self.subTest(speed=speed):
                node = self.node()
                node.slam_stationary_hint_time = 1.0
                # A gated wheel update must still veto a stationary hint.
                node.filter.x[node.filter.VX] = np.sign(speed) * 0.08
                with patch.object(node.filter, "update_wheel_speed", return_value=(False, None)):
                    for index in range(1, 101):
                        stamp = 1.0 + index * 0.02
                        self.wheel(node, stamp, speed)
                        node.imu_callback(node_test.EskfWheelMotionTest.imu_message(stamp))
                self.assertTrue(node._wheel_motion_confirmed(stamp))
                self.assertEqual(node.filter.counters["zupt_accepted"], 0)
                self.assertIsNone(node.zupt_candidate_since)
                self.assertFalse(node.wheel_stationary_output_active)
                self.assertAlmostEqual(node.odom_pub.publish.call_args[0][0].twist.twist.linear.x,
                                       np.sign(speed) * 0.08)

    def test_confirmation_requires_sign_duration_freshness_and_valid_samples(self):
        node = self.node()
        node.last_predict_stamp = None
        self.wheel(node, 1.0, -0.16)
        self.assertFalse(node._wheel_motion_confirmed(1.4))  # Silence is not persistence.
        self.wheel(node, 1.29, -0.16)
        self.assertFalse(node._wheel_motion_confirmed(1.4))
        self.wheel(node, 1.32, -0.16)
        self.assertTrue(node._wheel_motion_confirmed(1.32))
        self.assertFalse(node._wheel_motion_confirmed(1.83))
        self.wheel(node, 1.9, -0.16)
        self.assertFalse(node._wheel_motion_confirmed(1.9))
        self.wheel(node, 2.0, 0.16)
        self.assertEqual(node.wheel_motion_since, 2.0)
        self.wheel(node, 2.0, -0.16)  # Duplicate cannot change sign/history.
        self.assertEqual(node.wheel_motion_sign, 1)
        self.wheel(node, 2.1, 0.049)
        self.assertIsNone(node.wheel_motion_since)

    def test_alternating_noise_allows_standstill_recovery(self):
        node = self.node()
        node.filter.x[node.filter.VX] = 0.8
        with patch.object(node.filter, "update_wheel_speed", return_value=(False, None)):
            for index in range(1, 101):
                stamp = 1.0 + index * 0.02
                self.wheel(node, stamp, (-1)**index * 0.06)
                node.imu_callback(node_test.EskfWheelMotionTest.imu_message(stamp))
        self.assertFalse(node._wheel_motion_confirmed(stamp))
        self.assertGreater(node.zupt_reacquired_by_wheel, 0)
        self.assertTrue(node.wheel_stationary_output_active)

    def test_stationary_output_releases_before_next_imu(self):
        node = self.node()
        for index in range(1, 101):
            stamp = 1.0 + index * 0.02
            self.wheel(node, stamp, 0.0)
            node.imu_callback(node_test.EskfWheelMotionTest.imu_message(stamp))
        self.assertTrue(node.wheel_stationary_output_active)
        node.last_predict_stamp = None
        node.filter.x[node.filter.VX] = -0.08
        with patch.object(node.filter, "update_wheel_speed", return_value=(True, None)):
            for index in range(1, 18):
                stamp = 3.0 + index * 0.02
                node.last_imu_stamp_sec = stamp
                self.wheel(node, stamp, -0.06)
        self.assertFalse(node.wheel_stationary_output_active)
        self.assertAlmostEqual(node.odom_pub.publish.call_args[0][0].twist.twist.linear.x, -0.08)

    def test_deadband_requires_fresh_wheel_below_noise_floor(self):
        for wheel, fresh, enabled, expected in ((0.049, True, True, 0.0),
                                              (0.05, True, True, -0.08),
                                              (-0.16, True, True, -0.08),
                                              (0.0, False, True, -0.08),
                                              (0.0, True, False, -0.08)):
            with self.subTest(wheel=wheel, fresh=fresh, enabled=enabled):
                node = self.node()
                node.filter.x[node.filter.VX] = -0.08
                node.last_wheel_speed = wheel
                node.wheel_stationary_last_stamp = 1.0 if fresh else 0.0
                node.wheel_aiding_enabled = enabled
                node.publish(MODULE.rospy.Time.from_sec(1.0))
                self.assertAlmostEqual(node.odom_pub.publish.call_args[0][0].twist.twist.linear.x,
                                       expected)

    def test_rejected_slow_motion_releases_stationary_flag(self):
        node = self.node()
        node.last_predict_stamp = None
        node.wheel_stationary_output_active = True
        with patch.object(node.filter, "update_wheel_speed", return_value=(False, None)):
            for stamp in (1.0, 1.2, 1.32):
                self.wheel(node, stamp, -0.06)
        self.assertFalse(node.wheel_stationary_output_active)

    def test_slow_reverse_without_gps_constrains_lateral_drift(self):
        node = self.node()
        node.filter.initialize([0.0, 0.0], np.pi / 2.0)
        initial_position = node.filter.base_state()[0]
        node.filter.x[node.filter.VX] = 0.2  # Unobserved lateral velocity.
        for index in range(1, 151):
            stamp = 1.0 + index * 0.02
            self.wheel(node, stamp, -0.16)
            imu = node_test.EskfWheelMotionTest.imu_message(stamp, yaw=np.pi / 2.0)
            node.imu_callback(imu)
        position, velocity = node.filter.base_state()
        position -= initial_position
        self.assertEqual(node.filter.counters["zupt_accepted"], 0)
        self.assertGreater(node.filter.counters["nhc_accepted"], 0)
        self.assertLess(abs(velocity[0]), 0.01)
        self.assertLess(abs(position[0]), 0.1)
        self.assertAlmostEqual(position[1], -0.48, delta=0.05)


class OdometryRejectionDiagnosticTest(unittest.TestCase):
    def node(self):
        node = node_test.EskfImuOrientationTest().node()
        node.odometry_aiding_enabled = True
        node.odometry_speed_aiding_enabled = False
        node.last_gps_accept_time = 0.0
        node.last_predict_stamp = MODULE.rospy.Time.from_sec(200.0)
        node.odometry_alignment.anchor_stamp = 199.0
        node.odometry_shadow_anchor_samples = 10
        node.odometry_shadow_anchor_local_position = np.zeros(2)
        node.odometry_shadow_anchor_global_position = np.zeros(2)
        node.odometry_shadow_anchor_yaw = 0.0
        return node

    @staticmethod
    def message(stamp=200.0, position=0.0):
        msg = MODULE.Odometry()
        msg.header.stamp = MODULE.rospy.Time.from_sec(stamp)
        msg.pose.pose.position.x = position
        msg.pose.pose.orientation.w = 1.0
        return msg

    def test_each_pre_update_rejection_has_its_own_reason(self):
        for reason in ("message_lag", "local_pose_jump", "missing_anchor", "expired_anchor"):
            with self.subTest(reason=reason):
                node = self.node()
                msg = self.message()
                if reason == "message_lag":
                    msg.header.stamp = MODULE.rospy.Time.from_sec(199.0)
                elif reason == "local_pose_jump":
                    node.last_odometry_local_stamp = 199.99
                    node.last_odometry_local_position = np.array([100.0, 0.0])
                elif reason == "missing_anchor":
                    node.odometry_shadow_anchor_samples = 0
                else:
                    node.odometry_alignment.anchor_stamp = 0.0
                node.odometry_callback(msg)
                self.assertEqual(node.filter.counters["odometry_rejected"], 1)
                self.assertEqual(node.odometry_rejection_reasons[reason], 1)
                self.assertEqual(sum(node.odometry_rejection_reasons.values()), 1)
                self.assertEqual(node.last_odometry_rejection_reason, reason)
                self.assertIsNone(node.filter.last_nis["odometry_pose"])

    def test_innovation_gate_and_numerical_failure_are_distinct(self):
        node = self.node()
        node.odometry_shadow_anchor_global_position = np.array([10000.0, 0.0])
        with patch.object(MODULE.rospy, "logwarn_throttle"):
            node.odometry_callback(self.message())
        self.assertEqual(node.filter.counters["odometry_rejected"], 1)
        self.assertEqual(node.odometry_rejection_reasons["innovation_gate"], 1)
        node = self.node()
        with patch.object(node.filter, "update_odometry_position", return_value=(False, float("inf"))), \
                patch.object(MODULE.rospy, "logwarn_throttle"):
            node.odometry_callback(self.message())
        self.assertEqual(node.filter.counters["odometry_rejected"], 0)
        self.assertEqual(node.odometry_rejection_reasons["numerical_failure"], 1)

    def test_diagnostics_include_rejection_age_and_slow_motion_state(self):
        node = self.node()
        node.odometry_alignment.anchor_stamp = 0.0
        node.odometry_callback(self.message())
        with patch.object(MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time.from_sec(200.0)):
            node.publish_diagnostics(None)
        diagnostic = MODULE.json.loads(node.diagnostics_pub.publish.call_args[0][0].data)
        self.assertEqual(diagnostic["last_odometry_rejection_reason"], "expired_anchor")
        self.assertEqual(diagnostic["odometry_alignment_age_sec"], 200.0)
        self.assertEqual(diagnostic["odometry_message_lag_sec"], 0.0)
        self.assertFalse(diagnostic["wheel_motion_confirmed"])


if __name__ == "__main__":
    unittest.main()
