"""Measured-pose and optional-map failure paths for the raw LiDAR guard."""
from collections import deque
from contextlib import redirect_stdout
import io
import math
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import gui


def sample(stamp, x, *, heading=0., measured=True, src='ros'):
    return dict(stamp=stamp, x=x, y=0., heading_deg=heading,
                speed_kmh=36., measured=measured, src=src)


class MapRoadPoseContextTests(unittest.TestCase):
    def receiver(self, samples):
        receiver = gui.RosEgoReceiver.__new__(gui.RosEgoReceiver)
        receiver.lock = threading.Lock()
        receiver.samples = deque(samples)
        receiver.imu_samples = deque([
            dict(stamp=stamp, quaternion=[0., 0., 0., 1.],
                 covariance=[.01, 0., 0., 0., .01, 0., 0., 0., .01],
                 angular_velocity=[0., 0., 0.],
                 measured=True, frame_id='imu_link') for stamp in (100., 100.02, 100.04)])
        return receiver

    def test_bracketed_ros_measurements_interpolate_without_velocity_extrapolation(self):
        receiver = self.receiver([sample(100., 0., heading=359.),
                                  sample(100.04, .24, heading=1.)])
        with patch('gui.time.time', return_value=100.06):
            pose = receiver.measured_at_stamp(100.02)
        self.assertTrue(pose['measured'])
        self.assertAlmostEqual(pose['stamp'], 100.02)
        self.assertAlmostEqual(pose['x'], .12)
        self.assertAlmostEqual((pose['heading_deg']+180.) % 360.-180., 0.)
        self.assertAlmostEqual(pose['age_s'], .04)

    def test_exact_measured_source_stamp_is_accepted(self):
        receiver = self.receiver([sample(100., 3.)])
        with patch('gui.time.time', return_value=100.08):
            self.assertEqual(receiver.measured_at_stamp(100.)['x'], 3.)

    def test_file_fallback_cannot_supply_a_pose_or_complete_a_bracket(self):
        for samples in ([sample(100., 0., src='file')],
                        [sample(100., 0., measured=False)],
                        [sample(100., 0.), sample(100.04, .24, src='file')]):
            receiver = self.receiver(samples)
            with self.subTest(samples=samples), patch('gui.time.time', return_value=100.08):
                self.assertIsNone(receiver.measured_at_stamp(100.02))

    def test_nearby_measurement_is_not_extrapolated_into_a_source_pose(self):
        receiver = self.receiver([sample(100., 0.)])
        with patch('gui.time.time', return_value=100.08):
            self.assertIsNone(receiver.measured_at_stamp(100.001))

    def test_source_age_future_stamp_and_nonfinite_context_fail_closed(self):
        receiver = self.receiver([sample(100., 0.)])
        for now, stamp in ((100.201, 100.), (99.999, 100.), (100.1, math.nan)):
            with self.subTest(now=now, stamp=stamp), patch('gui.time.time', return_value=now):
                self.assertIsNone(receiver.measured_at_stamp(stamp))

    def test_large_pose_gap_and_conflicting_stamp_revision_fail_closed(self):
        for samples, query, now in (([sample(100., 0.), sample(100.25, 1.5)], 100.1, 100.28),
                                    ([sample(100., 0.), sample(100., .2)], 100., 100.18)):
            receiver = self.receiver(samples)
            with self.subTest(samples=samples), patch('gui.time.time', return_value=now):
                self.assertIsNone(receiver.measured_at_stamp(query))

    def test_shared_pose_bracket_limit_does_not_renew_source_age(self):
        receiver = self.receiver([sample(100., 0.), sample(100.12, .72)])
        receiver.imu_samples = deque([dict(receiver.imu_samples[0], stamp=s)
                                     for s in (100.04, 100.06, 100.08)])
        with patch('gui.time.time', return_value=100.13):
            value = receiver.measured_at_stamp(100.06)
        self.assertIsNotNone(value)
        self.assertAlmostEqual(value['age_s'], .07)

    def test_delayed_bracketing_measurement_can_enable_the_same_scan(self):
        receiver = self.receiver([sample(100., 0.)])
        with patch('gui.time.time', return_value=100.08):
            self.assertIsNone(receiver.measured_at_stamp(100.02))
            receiver.samples.append(sample(100.04, .24))
            self.assertIsNotNone(receiver.measured_at_stamp(100.02))

    def test_future_header_cannot_supply_a_measured_bracket(self):
        receiver = self.receiver([sample(100., 0.), sample(100.04, .24)])
        with patch('gui.time.time', return_value=100.03):
            self.assertIsNone(receiver.measured_at_stamp(100.02))
        receiver = self.receiver([sample(100.02, .12)])
        receiver.imu_samples = deque([r for r in receiver.imu_samples if r['stamp'] != 100.02])
        with patch('gui.time.time', return_value=100.03):
            self.assertIsNone(receiver.measured_at_stamp(100.02))

    def test_missing_imu_or_conflicting_source_stamp_keeps_the_raw_guard(self):
        receiver = self.receiver([sample(100., 0.)])
        receiver.imu_samples.clear()
        with patch('gui.time.time', return_value=100.08):
            self.assertIsNone(receiver.measured_at_stamp(100.))

    def test_native_ros_callback_marks_only_its_own_samples_as_measured(self):
        receiver = self.receiver([])
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:100.)),
            position=SimpleNamespace(x=3., y=4.), heading=20.,
            velocity=SimpleNamespace(x=10., y=0.), accel=.2, brake=0., wheel_angle=0.)
        receiver.on_ego(message)
        self.assertIs(receiver.samples[-1]['measured'], True)
        with patch('gui.time.time', return_value=100.05):
            pose = receiver.measured_at_stamp(100.)
        self.assertEqual((pose['x'], pose['y']), (3., 4.))

    def test_missing_or_uncertified_map_retains_original_raw_configuration(self):
        for factory in (lambda *a: SimpleNamespace(certified=False),
                        lambda *a: (_ for _ in ()).throw(ValueError('hash mismatch'))):
            module = SimpleNamespace(MappedRoadFloor=factory)
            with self.subTest(factory=factory), patch.dict(sys.modules,
                    {'perception.mapped_road_floor': module}), redirect_stdout(io.StringIO()):
                self.assertIsNone(gui.mapped_road_floor('invalid-map', 'invalid-route'))

    def test_certified_map_is_passed_through_unchanged(self):
        profile = SimpleNamespace(certified=True)
        module = SimpleNamespace(MappedRoadFloor=lambda *a: profile)
        with patch.dict(sys.modules, {'perception.mapped_road_floor': module}):
            self.assertIs(gui.mapped_road_floor('map', 'route'), profile)


if __name__ == '__main__':
    unittest.main()
