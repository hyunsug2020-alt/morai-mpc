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
)


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


if __name__ == "__main__":
    unittest.main()
