#!/usr/bin/env python3

import math
import unittest

import numpy as np

from eskf import (
    FixedLagHistory,
    GpsMode,
    GpsModeMachine,
    ImuReplayEvent,
    RobustPlanarESKF,
    SpeedReplayEvent,
    gps_fix_is_valid,
    local_gps_position_is_valid,
)
from eskf.timescale import GpsWheelTimeScale


class GpsInputValidationTest(unittest.TestCase):
    def test_morai_zero_fix_is_rejected(self):
        self.assertFalse(gps_fix_is_valid(0.0, 0.0))
        self.assertFalse(gps_fix_is_valid(float("nan"), 127.0))

    def test_korean_fix_and_local_position_are_accepted(self):
        self.assertTrue(gps_fix_is_valid(37.2391, 126.7734))
        self.assertTrue(local_gps_position_is_valid([25.0, -495.0]))

    def test_position_far_outside_local_map_is_rejected(self):
        self.assertFalse(local_gps_position_is_valid(
            [-42025234.0, -4124143.0], 100000.0))


class CoreSnapshotTest(unittest.TestCase):
    def test_snapshot_restore_is_deep_and_numerically_valid(self):
        eskf = RobustPlanarESKF()
        eskf.initialize([10.0, -2.0], 0.3)
        snapshot = eskf.snapshot()
        expected_state = snapshot["x"].copy()
        eskf.predict([1.0, 0.2], 0.1, 0.1)
        eskf.restore(snapshot)
        np.testing.assert_allclose(eskf.x, expected_state)
        self.assertTrue(eskf.is_numerically_valid())
        snapshot["x"][0] += 100.0
        self.assertNotAlmostEqual(eskf.x[0], snapshot["x"][0])


class FixedLagReplayTest(unittest.TestCase):
    @staticmethod
    def apply_event(eskf, event):
        if isinstance(event, SpeedReplayEvent):
            eskf.update_wheel_speed(
                event.forward_speed, event.variance)
            return
        eskf.predict(
            event.acceleration_body, event.yaw_rate, event.dt)
        if event.yaw_measurement is not None:
            eskf.update_yaw(event.yaw_measurement, event.yaw_variance)
        if event.apply_nhc:
            eskf.update_nonholonomic_constraint(event.yaw_rate)
        if event.apply_zupt:
            eskf.update_zero_velocity()

    def test_delayed_gps_matches_in_order_filter(self):
        in_order = RobustPlanarESKF()
        delayed = RobustPlanarESKF()
        in_order.initialize([0.0, 0.0], 0.0)
        delayed.initialize([0.0, 0.0], 0.0)
        history = FixedLagHistory(max_age=2.0, min_delay=0.01)
        history.seed(0.0, delayed.snapshot())
        measurement = np.array([0.20, -0.05])

        for index in range(1, 11):
            event = ImuReplayEvent(0.1, [0.8, 0.0], 0.01)
            self.apply_event(in_order, event)
            self.apply_event(delayed, event)
            history.record(index * 0.1, event, delayed.snapshot())
            if index == 5:
                accepted, _ = in_order.update_gps_position(measurement)
                self.assertTrue(accepted)

        result = history.replay(
            0.5,
            1.0,
            delayed.snapshot,
            delayed.restore,
            lambda: delayed.update_gps_position(measurement),
            lambda event: self.apply_event(delayed, event))
        self.assertTrue(result.attempted)
        self.assertTrue(result.accepted)
        self.assertEqual(result.replayed_events, 5)
        np.testing.assert_allclose(delayed.x, in_order.x, atol=1e-10)
        np.testing.assert_allclose(delayed.p, in_order.p, atol=1e-10)
        self.assertTrue(delayed.is_numerically_valid())

    def test_external_correction_barrier_blocks_unsafe_rewind(self):
        eskf = RobustPlanarESKF()
        eskf.initialize([0.0, 0.0], 0.0)
        history = FixedLagHistory(max_age=2.0)
        history.seed(0.0, eskf.snapshot())
        history.mark_barrier(0.8, eskf.snapshot())
        possible, reason = history.can_replay(0.7, 1.0)
        self.assertFalse(possible)
        self.assertEqual(reason, "before_external_correction_barrier")

    def test_delayed_gps_replays_interleaved_speed_updates(self):
        in_order = RobustPlanarESKF()
        delayed = RobustPlanarESKF()
        in_order.initialize([0.0, 0.0], 0.0)
        delayed.initialize([0.0, 0.0], 0.0)
        history = FixedLagHistory(max_age=2.0, min_delay=0.01)
        history.seed(0.0, delayed.snapshot())
        measurement = np.array([0.4, -0.1])

        for index in range(1, 11):
            imu_event = ImuReplayEvent(0.1, [0.4, 0.0], 0.01)
            self.apply_event(in_order, imu_event)
            self.apply_event(delayed, imu_event)
            history.record(index * 0.1, imu_event, delayed.snapshot())
            if index % 2 == 0:
                speed_event = SpeedReplayEvent(1.2 + 0.05 * index, 0.09)
                self.apply_event(in_order, speed_event)
                self.apply_event(delayed, speed_event)
                history.record(
                    index * 0.1, speed_event, delayed.snapshot())
            if index == 5:
                accepted, _ = in_order.update_gps_position(measurement)
                self.assertTrue(accepted)

        result = history.replay(
            0.5,
            1.0,
            delayed.snapshot,
            delayed.restore,
            lambda: delayed.update_gps_position(measurement),
            lambda event: self.apply_event(delayed, event))
        self.assertTrue(result.attempted)
        self.assertTrue(result.accepted)
        self.assertEqual(result.replayed_events, 8)
        np.testing.assert_allclose(delayed.x, in_order.x, atol=1e-10)
        np.testing.assert_allclose(delayed.p, in_order.p, atol=1e-10)

    def test_rejected_replay_restores_current_state(self):
        eskf = RobustPlanarESKF()
        eskf.initialize([0.0, 0.0], 0.0)
        history = FixedLagHistory(max_age=2.0, min_delay=0.01)
        history.seed(0.0, eskf.snapshot())
        event = ImuReplayEvent(0.1, [0.0, 0.0], 0.0)
        for index in range(1, 6):
            self.apply_event(eskf, event)
            history.record(index * 0.1, event, eskf.snapshot())
        before = eskf.snapshot()
        result = history.replay(
            0.2,
            0.5,
            eskf.snapshot,
            eskf.restore,
            lambda: eskf.update_gps_position([1000.0, 1000.0]),
            lambda item: self.apply_event(eskf, item))
        self.assertFalse(result.accepted)
        np.testing.assert_allclose(eskf.x, before["x"])
        np.testing.assert_allclose(eskf.p, before["p"])


class OdometryUpdateTest(unittest.TestCase):
    def test_base_covariance_includes_yaw_lever_arm(self):
        eskf = RobustPlanarESKF(imu_lever=[3.4, 0.0])
        eskf.initialize([0.0, 0.0], 0.0)
        eskf.p[:] = 0.0
        eskf.p[eskf.YAW, eskf.YAW] = 0.25
        pose_covariance, twist_covariance = eskf.base_covariance()
        self.assertAlmostEqual(pose_covariance[1, 1], 3.4**2 * 0.25)
        self.assertAlmostEqual(pose_covariance[2, 2], 0.25)
        self.assertTrue(np.all(np.isfinite(twist_covariance)))

    def test_velocity_reanchor_preserves_requested_base_link_velocity(self):
        eskf = RobustPlanarESKF(
            imu_lever=[3.4, 0.2], gps_from_imu=[-0.2, 0.03])
        eskf.initialize([10.0, -4.0], 0.7)
        eskf.last_corrected_yaw_rate = 0.35
        eskf.x[eskf.VX:eskf.VY + 1] = [22.0, -9.0]
        requested_velocity = np.array([1.2, 0.8])

        eskf.reanchor_base_velocity(requested_velocity, variance=0.25)

        _, actual_velocity = eskf.base_state()
        np.testing.assert_allclose(
            actual_velocity, requested_velocity, atol=1e-12)
        self.assertAlmostEqual(eskf.p[eskf.VX, eskf.VX], 0.25)
        self.assertAlmostEqual(eskf.p[eskf.VY, eskf.VY], 0.25)
        self.assertEqual(eskf.counters["velocity_reanchored"], 1)
        self.assertTrue(eskf.is_numerically_valid())

    def test_gps_reanchor_can_reset_position_and_velocity_together(self):
        eskf = RobustPlanarESKF(
            imu_lever=[3.4, 0.0], gps_from_imu=[-0.2, 0.0])
        eskf.initialize([0.0, 0.0], -0.4)
        eskf.x[eskf.VX:eskf.VY + 1] = [25.0, 4.0]
        gps_position = np.array([120.0, -30.0])
        base_velocity = np.array([0.5, -0.2])

        eskf.reanchor_gps_position(
            gps_position,
            base_velocity=base_velocity,
            velocity_variance=0.36)

        expected_imu_position = (
            gps_position
            - eskf._rotation(eskf.x[eskf.YAW]) @ eskf.gps_from_imu)
        np.testing.assert_allclose(eskf.x[0:2], expected_imu_position)
        np.testing.assert_allclose(eskf.base_state()[1], base_velocity)
        self.assertAlmostEqual(eskf.p[eskf.VX, eskf.VX], 0.36)
        self.assertEqual(eskf.counters["gps_reacquired"], 1)
        self.assertEqual(eskf.counters["velocity_reanchored"], 1)
        self.assertTrue(eskf.is_numerically_valid())

    def test_gps_recovery_correction_is_position_and_heading_bounded(self):
        eskf = RobustPlanarESKF(gps_from_imu=[0.2, 0.0])
        eskf.initialize([0.0, 0.0], 0.0)

        result = eskf.apply_bounded_gps_recovery(
            [6.5, 0.0], max_position_step=0.35,
            heading_measurement=math.radians(20.0),
            max_heading_step=math.radians(1.5))
        sensor_position = (
            eskf.x[0:2] + eskf._rotation(eskf.x[eskf.YAW]) @
            eskf.gps_from_imu)

        self.assertLessEqual(
            np.linalg.norm(sensor_position - np.array([0.2, 0.0])),
            0.3500001)
        self.assertAlmostEqual(
            eskf.x[eskf.YAW], math.radians(1.5), places=10)
        self.assertGreater(result["position_remaining_m"], 5.0)
        self.assertEqual(eskf.counters["gps_recovery_steps"], 1)
        self.assertTrue(eskf.is_numerically_valid())

    def test_aligned_odometry_position_updates_base_link_state(self):
        eskf = RobustPlanarESKF(imu_lever=[2.0, 0.0])
        eskf.initialize([0.0, 0.0], 0.0)
        before_error = np.linalg.norm(
            np.array([1.0, 0.25]) - eskf.base_state()[0])
        accepted, nis = eskf.update_odometry_position(
            [1.0, 0.25], 0.25)
        after_error = np.linalg.norm(
            np.array([1.0, 0.25]) - eskf.base_state()[0])
        self.assertTrue(accepted)
        self.assertTrue(math.isfinite(nis))
        self.assertLess(after_error, before_error)
        self.assertEqual(eskf.counters["odometry_accepted"], 1)
        self.assertTrue(eskf.is_numerically_valid())

    def test_odometry_position_outlier_is_rejected(self):
        eskf = RobustPlanarESKF()
        eskf.initialize([0.0, 0.0], 0.0)
        before = eskf.snapshot()
        accepted, nis = eskf.update_odometry_position(
            [1000.0, -1000.0], 0.25)
        self.assertFalse(accepted)
        self.assertGreater(nis, eskf.odometry_nis_hard)
        np.testing.assert_allclose(eskf.x, before["x"])
        self.assertEqual(eskf.counters["odometry_rejected"], 1)


class YawReanchorTest(unittest.TestCase):
    def test_locked_out_yaw_is_adopted_and_velocity_turns_with_it(self):
        # 2026-10-03: after a divergence the filter rejected the noise-free
        # IMU heading for minutes. Adopting it must keep body-frame motion.
        eskf = RobustPlanarESKF()
        eskf.initialize([0.0, 0.0], 0.0)
        eskf.x[eskf.VX:eskf.VY + 1] = [10.0, 0.0]
        accepted, _ = eskf.update_yaw(math.radians(120.), 1e-4)
        self.assertFalse(accepted)
        eskf.reanchor_yaw(math.radians(120.), 1e-4)
        self.assertAlmostEqual(eskf.x[eskf.YAW], math.radians(120.), places=6)
        np.testing.assert_allclose(eskf.x[eskf.VX:eskf.VY + 1],
                                   [10. * math.cos(math.radians(120.)), 10. * math.sin(math.radians(120.))],
                                   atol=1e-6)
        accepted, _ = eskf.update_yaw(math.radians(120.5), 1e-4)
        self.assertTrue(accepted)


class LaneConstraintUpdateTest(unittest.TestCase):
    def test_lane_update_reduces_cross_track_and_yaw_without_along_track_fix(self):
        eskf = RobustPlanarESKF(imu_lever=[0.0, 0.0])
        eskf.initialize([5.0, 1.0], 0.10)
        before_x = float(eskf.x[eskf.PX])
        before_lateral = abs(float(eskf.x[eskf.PY]))
        before_yaw = abs(float(eskf.x[eskf.YAW]))

        accepted, nis = eskf.update_lane_constraint(
            map_point=[15.0, 0.0],
            map_heading=0.0,
            body_point=[10.0, 0.0],
            yaw_measurement=0.0,
            lateral_variance=0.16,
            yaw_variance=math.radians(4.0) ** 2)

        self.assertTrue(accepted)
        self.assertTrue(math.isfinite(nis))
        self.assertLess(abs(float(eskf.x[eskf.PY])), before_lateral)
        self.assertLess(abs(float(eskf.x[eskf.YAW])), before_yaw)
        self.assertAlmostEqual(float(eskf.x[eskf.PX]), before_x, places=12)
        self.assertEqual(eskf.counters["lane_accepted"], 1)
        self.assertTrue(eskf.is_numerically_valid())

    def test_lane_outlier_is_rejected(self):
        eskf = RobustPlanarESKF()
        eskf.initialize([0.0, 0.0], 0.0)
        before = eskf.snapshot()
        accepted, nis = eskf.update_lane_constraint(
            map_point=[10.0, 100.0],
            map_heading=0.0,
            body_point=[10.0, 0.0],
            yaw_measurement=0.0,
            lateral_variance=0.16,
            yaw_variance=math.radians(4.0) ** 2)
        self.assertFalse(accepted)
        self.assertGreater(nis, eskf.lane_nis_hard)
        np.testing.assert_allclose(eskf.x, before["x"])
        self.assertEqual(eskf.counters["lane_rejected"], 1)

class GpsModeMachineTest(unittest.TestCase):
    def test_loss_recovery_and_degraded_transitions(self):
        modes = GpsModeMachine(
            outage_timeout=1.0, rejection_limit=2, recovery_accepts=2)
        self.assertEqual(modes.mode, GpsMode.STARTING)
        modes.initialized(0.0)
        self.assertEqual(modes.mode, GpsMode.NORMAL)
        modes.tick(1.1)
        self.assertEqual(modes.mode, GpsMode.GPS_LOST)
        modes.received(1.2)
        self.assertEqual(modes.mode, GpsMode.REACQUIRING)
        transition_count = modes.transitions
        modes.received(1.25)
        self.assertEqual(modes.mode, GpsMode.REACQUIRING)
        self.assertEqual(modes.transitions, transition_count)
        modes.accepted(1.2)
        self.assertEqual(modes.mode, GpsMode.REACQUIRING)
        modes.accepted(1.3)
        self.assertEqual(modes.mode, GpsMode.NORMAL)
        modes.rejected(1.4)
        modes.rejected(1.5)
        self.assertEqual(modes.mode, GpsMode.DEGRADED)
        diagnostics = modes.diagnostics(1.5)
        self.assertEqual(diagnostics["gps_mode"], "degraded")
        self.assertGreaterEqual(diagnostics["gps_mode_transitions"], 4)



class GpsWheelTimeScaleTest(unittest.TestCase):
    """Wall-clock stamps, physics speed: GPS/wheel ratio is the clock ratio."""

    def _drive(self, ratio, yaw_rate=0.0, speed=10.0, seconds=6.0):
        estimator = GpsWheelTimeScale(gps_lever=(3.28, -0.52))
        position = np.zeros(2)
        yaw = 0.3
        wall = 100.0
        result = None
        for step in range(int(seconds / 0.05)):
            wall += 0.05
            yaw += yaw_rate * 0.05 * ratio
            position = position + speed * ratio * 0.05 * np.array(
                [math.cos(yaw), math.sin(yaw)])
            estimator.add_speed(wall, speed)
            estimator.add_yaw(wall, yaw)
            if step % 2 == 0:
                c, s = math.cos(yaw), math.sin(yaw)
                antenna = position + np.array(
                    [c * 3.28 + s * 0.52, s * 3.28 - c * 0.52])
                result = estimator.add_gps(wall, antenna) or result
        return estimator, wall, result

    def test_straight_line_ratio(self):
        estimator, wall, _ = self._drive(0.65)
        self.assertAlmostEqual(estimator.get(wall), 0.65, places=3)

    def test_turning_ratio(self):
        estimator, wall, _ = self._drive(0.85, yaw_rate=0.4)
        self.assertAlmostEqual(estimator.get(wall), 0.85, delta=0.01)

    def test_standstill_gives_no_measurement_and_expires(self):
        estimator, wall, _ = self._drive(0.7, speed=0.0)
        self.assertIsNone(estimator.get(wall))


class TimeScaleProcessNoiseTest(unittest.TestCase):
    def test_noise_added_along_velocity_only(self):
        legacy = RobustPlanarESKF({})
        noisy = RobustPlanarESKF({"time_scale_relative_std": 0.1})
        for filt in (legacy, noisy):
            filt.initialize([0.0, 0.0], 0.0)
            filt.x[filt.VX] = 20.0
            filt.p[:] = np.eye(filt.SIZE) * 1e-6
            filt.predict(np.zeros(2), 0.0, 0.1)
        extra = noisy.p[0:2, 0:2] - legacy.p[0:2, 0:2]
        self.assertAlmostEqual(extra[0, 0], (20.0 * 0.1 * 0.1) ** 2, places=6)
        self.assertAlmostEqual(extra[1, 1], 0.0, places=9)

    def test_gps_adaptive_scale_cap(self):
        filt = RobustPlanarESKF({"gps_position_adaptive_scale_max": 2.0})
        filt.initialize([0.0, 0.0], 0.0)
        for _ in range(200):
            filt.update_gps_position([3.0, 0.0], 0.03)
            filt.x[0:2] = 0.0
        self.assertLessEqual(filt.measurement_scales["gps_position"], 2.0)

if __name__ == "__main__":
    unittest.main()
