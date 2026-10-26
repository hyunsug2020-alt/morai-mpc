"""Check measured-speed recovery after a transient stop with inert ROS publishers."""
import unittest
from unittest.mock import patch

import numpy as np

import test_avoidance_gate as avoidance_fixture
from gui import LAUNCH_ACCEL_MPS2


class PublishedSpeedRecoveryTests(unittest.TestCase):
    def setUp(self):
        runtime = avoidance_fixture.AvoidanceGateTests().runtime()
        self.publisher = runtime.path_publisher
        self.route = runtime.test_route
        self.ego = dict(runtime.test_status['ego'])
        self.postprocess = dict(mode='HIGHWAY', reason='checked highway cruise', validated_static=True)

    def publish(self, clock, measured_kmh, cap_kmh, emergency=False, path=None):
        self.ego['speed_kmh'] = measured_kmh
        self.publisher.zone_mps = cap_kmh / 3.6
        self.publisher.emergency_stop = emergency
        with patch('gui.time.monotonic', return_value=clock):
            self.publisher.publish(self.ego, self.route if path is None else path,
                                   self.route, self.postprocess)
        return self.publisher.velocity_pub.items[-1].data * 3.6

    def test_moving_car_recovers_from_one_stop_pulse_at_measured_speed(self):
        self.assertEqual(self.publish(100., 68.1, 63.7, emergency=True), 0.)
        actual = self.publish(100.2, 60.3, 63.7)
        self.assertGreaterEqual(actual, 60.3)
        self.assertLessEqual(actual, 60.3 + LAUNCH_ACCEL_MPS2 * .2 * 3.6 + 1e-9)

    def test_recovery_never_exceeds_new_checked_safety_cap(self):
        self.publish(100., 52.6, 90., emergency=True)
        self.assertAlmostEqual(self.publish(100.2, 52.6, 12.), 12.)

    def test_standstill_restart_retains_acceleration_limit(self):
        self.publish(100., 0., 28.94, emergency=True)
        self.assertAlmostEqual(self.publish(100.2, 0., 28.94), LAUNCH_ACCEL_MPS2 * .2 * 3.6)

    def test_repeated_stop_keeps_zero_even_with_high_measured_speed(self):
        self.publish(100., 68.1, 90., emergency=True)
        self.assertEqual(self.publish(100.2, 60.3, 90., emergency=True), 0.)

    def test_valid_path_recovers_after_invalid_path_stop(self):
        self.assertEqual(self.publish(100., 21., 28.94, path=np.zeros((1, 2))), 0.)
        self.assertGreaterEqual(self.publish(100.2, 20., 28.94), 20.)

    def test_planned_zero_cap_recovers_at_measured_speed(self):
        self.assertEqual(self.publish(100., 21., 0.), 0.)
        self.assertGreaterEqual(self.publish(100.2, 20., 28.94), 20.)

    def test_regular_speed_increase_retains_command_acceleration_limit(self):
        self.publish(100., 20., 20.)
        self.assertAlmostEqual(self.publish(100.2, 20., 28.94), 20. + LAUNCH_ACCEL_MPS2 * .2 * 3.6)

    def test_checked_highway_reference_is_published_without_a_second_ramp(self):
        self.publish(100., 60., 20.)
        self.postprocess['highway'] = dict(speed_plan=dict(reference_checked=True))
        self.assertAlmostEqual(self.publish(100.1, 60., 61.44), 61.44)

    def test_emergency_overrides_a_checked_highway_reference(self):
        self.postprocess['highway'] = dict(speed_plan=dict(reference_checked=True))
        self.assertEqual(self.publish(100., 60., 61.44, emergency=True), 0.)

    def test_other_modes_keep_their_ramp_even_with_old_highway_metadata(self):
        self.publish(100., 20., 20.)
        self.postprocess.update(mode='LATTICE', highway=dict(speed_plan=dict(reference_checked=True)))
        self.assertAlmostEqual(self.publish(100.2, 20., 28.94), 20. + LAUNCH_ACCEL_MPS2 * .2 * 3.6)


if __name__ == '__main__':
    unittest.main()
