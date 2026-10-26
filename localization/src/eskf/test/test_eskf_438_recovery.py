import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

import numpy as np

from eskf.output import PositionCorrectionLimiter
import test_eskf_node_gps_input as node_test


MODULE = node_test.MODULE


class PositionCorrectionTest(unittest.TestCase):
    def test_high_speed_jump_is_limited_and_error_recovers(self):
        limiter = PositionCorrectionLimiter(2.0)
        velocity = np.array([0.0, -23.0])
        limiter.update([0.0, 0.0], velocity, 0.0, 0.0)
        previous = limiter.position.copy()
        steps = []
        for index in range(1, 101):
            stamp = index * 0.05
            raw = velocity * stamp + [0.0, 3.56]
            position = limiter.update(raw, velocity, stamp, 0.05)
            correction = position - previous - velocity * 0.05
            steps.append(correction)
            self.assertLessEqual(np.linalg.norm(correction), 0.1000001)
            previous = position
        for start in range(91):
            self.assertLessEqual(np.linalg.norm(np.sum(steps[start:start + 10], axis=0)), 1.000001)
        np.testing.assert_allclose(limiter.remaining, 0.0, atol=1e-10)
        np.testing.assert_allclose(position, raw)

    def test_same_and_out_of_order_stamp_cannot_spend_budget(self):
        limiter = PositionCorrectionLimiter()
        limiter.update([0.0, 0.0], [2.0, 0.0], 1.0, 0.0)
        for stamp in (1.0, 0.9, 1.0):
            np.testing.assert_allclose(
                limiter.update([10.0, 0.0], [2.0, 0.0], stamp, 0.0), 0.0)
        np.testing.assert_allclose(
            limiter.update([10.0, 0.0], [2.0, 0.0], 1.1, 0.1), [0.4, 0.0])

    def test_physics_time_scales_motion_not_correction_budget(self):
        limiter = PositionCorrectionLimiter()
        limiter.update([0.0, 0.0], [20.0, 0.0], 1.0, 0.0)
        position = limiter.update([10.0, 0.0], [20.0, 0.0], 1.5, 0.2)
        np.testing.assert_allclose(position, [5.0, 0.0])

    def test_explicit_reset_allows_stationary_scenario_relocation(self):
        limiter = PositionCorrectionLimiter()
        limiter.update([0.0, 0.0], [0.0, 0.0], 1.0, 0.0)
        limiter.reset()
        np.testing.assert_allclose(
            limiter.update([100.0, 0.0], [0.0, 0.0], 1.0, 0.0), [100.0, 0.0])


class GpsRecoveryOutputTest(unittest.TestCase):
    def node(self, speed=2.0, stationary=False):
        node = node_test.EskfImuOrientationTest().node()
        node.filter = MODULE.RobustPlanarESKF()
        node.filter.initialize([0.0, 0.0], 0.0)
        node.filter.x[node.filter.VX] = speed
        node.gps_replay.seed(1.0, node.filter.snapshot())
        node._trusted_odometry_base_velocity = lambda _: np.array([speed, 0.0])
        node.gps_reacquisition_active = True
        node.gps_reacquisition_samples = 2
        node.tf_broadcaster = None
        node.odom_pub = Mock()
        node.publish = MODULE.ESKFNode.publish.__get__(node)
        if stationary:
            node.last_wheel_speed = 0.0
            node.wheel_stationary_since = 0.0
            node.wheel_stationary_last_stamp = 1.0
        node.publish(MODULE.rospy.Time.from_sec(1.0))
        return node

    @staticmethod
    def message(stamp):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=MODULE.rospy.Time.from_sec(stamp)),
            latitude=37.0, longitude=127.0, eastOffset=0.0,
            northOffset=0.0, status=0)

    def recover(self, node, distance):
        with patch.object(MODULE, "latlon_to_utm", return_value=(distance, 0.0)):
            node.gps_callback(self.message(1.01))
            node.gps_callback(self.message(1.02))

    def test_moving_internal_reanchor_preserves_control_pose(self):
        for speed in (2.0, -2.0):
            with self.subTest(speed=speed):
                node = self.node(speed)
                self.recover(node, 10.0)
                odom = node.odom_pub.publish.call_args[0][0]
                self.assertGreater(node.filter.base_state()[0][0], 9.0)
                self.assertLess(abs(odom.pose.pose.position.x), 0.1)
                self.assertEqual(node.gps_recovery_event["reason"], "moving_internal_reanchor")
                self.assertGreater(odom.pose.covariance[0], 90.0)
                self.assertGreater(np.linalg.norm(node.output_position.remaining), 9.0)

    def test_stationary_reset_requires_confirmed_fresh_wheel(self):
        for confirmed in (False, True):
            with self.subTest(confirmed=confirmed):
                node = self.node(0.0, stationary=confirmed)
                self.recover(node, 100.0)
                odom = node.odom_pub.publish.call_args[0][0]
                if confirmed:
                    self.assertAlmostEqual(odom.pose.pose.position.x, 100.0)
                    self.assertEqual(node.gps_recovery_event["reason"], "stationary_scenario_reset")
                else:
                    self.assertLess(odom.pose.pose.position.x, 0.05)
                    self.assertEqual(node.gps_recovery_event["reason"], "moving_internal_reanchor")

    def test_delayed_consensus_is_projected_to_filter_state_epoch(self):
        node = self.node()
        node.last_predict_stamp = MODULE.rospy.Time.from_sec(1.2)
        with patch.object(MODULE, "latlon_to_utm", side_effect=[(10.0, 0.0), (10.2, 0.0)]):
            node.gps_callback(self.message(1.0))
            node.gps_callback(self.message(1.1))
        self.assertAlmostEqual(node.filter.base_state()[0][0], 10.4)
        self.assertAlmostEqual(node.gps_history[-1][0], 1.2)
        self.assertAlmostEqual(node.gps_recovery_event["gps_lag_sec"], 0.1)
        self.assertAlmostEqual(node.odom_pub.publish.call_args[0][0].header.stamp.to_sec(), 1.2)


class WheelStandstillRecoveryTest(unittest.TestCase):
    def test_quiet_wheel_and_imu_recover_velocity_and_bias_drift(self):
        for drift in (0.8, -0.8):
            with self.subTest(drift=drift):
                node = node_test.EskfImuOrientationTest().node()
                node.filter.initialize([0.0, 0.0], 0.0)
                node.filter.x[node.filter.VX] = drift
                node.filter.x[node.filter.BAX] = 1.0
                with patch.object(node.filter, "update_wheel_speed", return_value=(False, None)):
                    for index in range(1, 101):
                        stamp = 1.0 + index * 0.02
                        node.vehicle_state_callback(node_test.EskfWheelMotionTest.wheel_message(stamp, 0.0))
                        node.imu_callback(node_test.EskfWheelMotionTest.imu_message(stamp))
                        if index < 65:
                            self.assertEqual(node.filter.counters["zupt_accepted"], 0)
                self.assertGreater(node.zupt_reacquired_by_wheel, 0)
                self.assertLess(np.linalg.norm(node.filter.base_state()[1]), 0.1)

    def test_motion_or_imu_activity_prevents_false_zupt(self):
        for wheel, accel, gyro in ((2.0, 0.0, 0.0), (-2.0, 0.0, 0.0),
                                   (0.0, 0.5, 0.0), (0.0, 0.0, 0.1)):
            with self.subTest(wheel=wheel, accel=accel, gyro=gyro):
                node = node_test.EskfImuOrientationTest().node()
                node.filter.initialize([0.0, 0.0], 0.0)
                node.filter.x[node.filter.VX] = 0.8
                with patch.object(node.filter, "update_wheel_speed", return_value=(False, None)):
                    for index in range(1, 101):
                        stamp = 1.0 + index * 0.02
                        node.vehicle_state_callback(node_test.EskfWheelMotionTest.wheel_message(stamp, wheel))
                        message = node_test.EskfWheelMotionTest.imu_message(stamp)
                        message.linear_acceleration.x = accel
                        message.angular_velocity.z = gyro
                        node.imu_callback(message)
                self.assertEqual(node.filter.counters["zupt_accepted"], 0)

    def test_stationary_confirmation_expires_and_restarts_after_gap(self):
        node = node_test.EskfImuOrientationTest().node()
        node.last_predict_stamp = None
        for stamp in (1.0, 1.25, 1.5):
            node.vehicle_state_callback(node_test.EskfWheelMotionTest.wheel_message(stamp, 0.0))
        self.assertTrue(node._wheel_stationary_confirmed(1.5))
        node.wheel_stationary_since = 0.9
        self.assertTrue(node._wheel_stationary_confirmed(1.49))
        self.assertFalse(node._wheel_stationary_confirmed(2.01))
        node.vehicle_state_callback(node_test.EskfWheelMotionTest.wheel_message(2.1, 0.0))
        self.assertFalse(node._wheel_stationary_confirmed(2.1))
        node.vehicle_state_callback(node_test.EskfWheelMotionTest.wheel_message(2.2, 0.1))
        self.assertIsNone(node.wheel_stationary_since)

    def test_confirmed_stationary_output_is_released_on_wheel_motion(self):
        node = node_test.EskfImuOrientationTest().node()
        node.filter.initialize([0.0, 0.0], 0.0)
        node.filter.x[node.filter.VX] = 0.8
        node.wheel_stationary_since = 0.0
        node.wheel_stationary_last_stamp = 2.0
        node.last_imu_stamp_sec = 2.0
        node.zupt_candidate_since = 0.0
        node.last_predict_stamp = MODULE.rospy.Time.from_sec(2.0)
        node.tf_broadcaster = None
        node.odom_pub = Mock()
        node.publish = MODULE.ESKFNode.publish.__get__(node)
        node.publish(MODULE.rospy.Time.from_sec(2.0))
        self.assertTrue(node.wheel_stationary_output_active)
        self.assertEqual(node.odom_pub.publish.call_args[0][0].twist.twist.linear.x, 0.0)
        with patch.object(MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time.from_sec(2.0)):
            node.publish_diagnostics(None)
        diagnostic = MODULE.json.loads(node.diagnostics_pub.publish.call_args[0][0].data)
        self.assertAlmostEqual(diagnostic["filter_forward_speed_mps"], 0.8)
        self.assertTrue(diagnostic["wheel_stationary_output_active"])
        self.assertEqual(diagnostic["output_base_position_m"], node.output_position.position.tolist())
        node.filter.x[node.filter.VX] = -2.0
        node.vehicle_state_callback(node_test.EskfWheelMotionTest.wheel_message(2.02, -2.0))
        self.assertFalse(node.wheel_stationary_output_active)
        self.assertAlmostEqual(node.odom_pub.publish.call_args[0][0].twist.twist.linear.x, -2.0)


if __name__ == "__main__":
    unittest.main()
