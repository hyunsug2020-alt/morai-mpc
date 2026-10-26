"""Source identity, timing and quaternion failures must retain raw obstacles."""
import math
import json
from pathlib import Path
import unittest

import numpy as np

from perception.imu_attitude import measured_imu_at


def sample(stamp, pitch=0., sign=1.):
    return dict(stamp=stamp, quaternion=[0., sign*math.sin(pitch/2.), 0.,
                                        sign*math.cos(pitch/2.)],
                covariance=[.01, 0., 0., 0., .01, 0., 0., 0., .01],
                angular_velocity=[0., 0., 0.], frame_id='imu_link', measured=True)


class MeasuredImuAttitudeTests(unittest.TestCase):
    def test_exact_and_nearby_attitudes_keep_actual_source_headers(self):
        before, after = sample(100., .002), sample(100.04, .004, sign=-1.)
        exact = measured_imu_at([before, after], 100.)
        self.assertEqual(exact['imu_source_stamps'], [100.])
        value = measured_imu_at([before, after], 100.001)
        self.assertEqual(value['imu_source_stamps'], [100.])
        self.assertEqual(value['imu_stamp'], 100.)
        self.assertAlmostEqual(np.linalg.norm(value['imu_quaternion']), 1.)
        self.assertAlmostEqual(abs(value['imu_quaternion'][1]), math.sin(.002/2.))
        self.assertLessEqual(value['imu_adjacent_rate_rad_s'], .2)

    def test_missing_wide_bracket_and_extrapolation_are_rejected(self):
        for samples, stamp in (([], 100.), ([sample(100.)], 100.001),
                               ([sample(100.), sample(100.2)], 100.1)):
            with self.subTest(samples=samples, stamp=stamp):
                self.assertIsNone(measured_imu_at(samples, stamp))

    def test_invalid_orientation_and_untrusted_source_are_rejected(self):
        baseline = sample(100.)
        invalid = [dict(baseline, quaternion=[0., 0., 0., 0.]),
                   dict(baseline, quaternion=[0., math.nan, 0., 1.]),
                   dict(baseline, covariance=[-1.]+[0.]*8),
                   dict(baseline, covariance=[math.nan]*9),
                   dict(baseline, measured=False), dict(baseline, src='file'),
                   dict(baseline, frame_id='different_sensor')]
        for record in invalid:
            with self.subTest(record=record):
                self.assertIsNone(measured_imu_at([record], 100.))

    def test_same_stamp_conflict_is_rejected_even_when_the_first_record_is_valid(self):
        before = sample(100.)
        revised = sample(100., .001)
        self.assertIsNone(measured_imu_at([before, revised], 100.))
        self.assertIsNone(measured_imu_at([revised, before, sample(100.04)], 100.02))

    def test_sign_equivalent_duplicate_is_the_same_attitude(self):
        samples = [sample(100., .002), sample(100., .002, sign=-1.), sample(100.04, .002)]
        self.assertIsNotNone(measured_imu_at(samples, 100.))

    def test_covariance_revision_at_the_same_stamp_is_not_a_new_source(self):
        before = sample(100.)
        revised = dict(before, covariance=[.02, 0., 0., 0., .02, 0., 0., 0., .02])
        self.assertIsNone(measured_imu_at([before, revised], 100.))

    def test_attitude_jump_does_not_authorize_interpolation(self):
        self.assertIsNone(measured_imu_at([sample(100.), sample(100.04, .1)], 100.001))

    def test_midpoint_interpolation_counterexample_is_rejected(self):
        self.assertIsNone(measured_imu_at([sample(100.), sample(100.098, math.radians(1.))], 100.049))

    def test_gyro_spike_and_same_stamp_gyro_revision_are_rejected(self):
        before = sample(100.)
        revised = dict(before, angular_velocity=[.15, 0., 0.])
        neighbour = sample(100.04)
        self.assertIsNone(measured_imu_at([revised, neighbour], 100.))
        self.assertIsNone(measured_imu_at([before, revised, neighbour], 100.))

    def test_neighbour_gyro_does_not_replace_the_selected_sample_limit(self):
        selected = dict(sample(100.), angular_velocity=[.09, 0., 0.])
        neighbour = dict(sample(100.04, .004), angular_velocity=[.186, 0., 0.])
        value = measured_imu_at([selected, neighbour], 100.001)
        self.assertIsNotNone(value)
        self.assertEqual(value['imu_angular_velocity'], [.09, 0., 0.])
        self.assertAlmostEqual(value['imu_adjacent_rate_rad_s'], .1, places=6)
        self.assertIsNone(measured_imu_at([selected, neighbour], 100.04))

    def test_neighbour_revisions_must_agree_for_either_input_order(self):
        selected, neighbour = sample(100.), sample(100.04, .004)
        revisions = [sample(100.04, .005),
                     dict(neighbour, covariance=[.02]+[0.]*8),
                     dict(neighbour, angular_velocity=[.15, 0., 0.])]
        for revised in revisions:
            for records in ([selected, neighbour, revised],
                            [revised, selected, neighbour]):
                with self.subTest(records=records):
                    self.assertIsNone(measured_imu_at(records, 100.001))
        equivalent = sample(100.04, .004, sign=-1.)
        self.assertIsNotNone(measured_imu_at([selected, neighbour, equivalent], 100.001))

    def test_invalid_neighbour_remains_fail_closed(self):
        selected, neighbour = sample(100.), sample(100.04, .004)
        invalid = [dict(neighbour, quaternion=[0., 0., 0., 0.]),
                   dict(neighbour, covariance=[-1.]+[0.]*8),
                   dict(neighbour, angular_velocity=[math.nan, 0., 0.]),
                   dict(neighbour, measured=False),
                   dict(neighbour, frame_id='other_sensor'), sample(100.04, .02)]
        for value in invalid:
            with self.subTest(neighbour=value):
                self.assertIsNone(measured_imu_at([selected, value], 100.001))

    def test_time_window_keeps_measured_headers_and_rejects_unsupported_timing(self):
        samples = [sample(100.), sample(100.04)]
        value = measured_imu_at(samples, 100.0059)
        self.assertIsNotNone(value)
        self.assertEqual(value['imu_stamp'], 100.)
        self.assertIsNone(measured_imu_at(samples, 100.0061))
        self.assertIsNone(measured_imu_at([sample(100.)], 100.))

    def test_observed_ten_to_fifteen_hundredths_second_neighbours_keep_current_attitude(self):
        selected = dict(sample(100., .002), angular_velocity=[.09, 0., 0.])
        for span in (.10, .1271049976348877, .15016531944274902):
            with self.subTest(span=span):
                neighbour = sample(100.+span, .002+.1*span)
                value = measured_imu_at([selected, neighbour], 100.0059)
                self.assertIsNotNone(value)
                self.assertEqual(value['imu_stamp'], 100.)
                self.assertEqual(value['imu_quaternion'], selected['quaternion'])
                self.assertLessEqual(value['imu_adjacent_rate_rad_s'], .2)
                self.assertIsNone(measured_imu_at([selected, neighbour], 100.0061))
        self.assertIsNone(measured_imu_at([selected, sample(100.201, .002)], 100.001))

    def test_actual_recorded_sparse_neighbour_is_available_before_the_next_sample(self):
        source_path = Path(__file__).parent/'test_data/mapped_road_floor_cases.json'
        case = next(case for case in json.loads(source_path.read_text()) if case['native'] == 65)
        scan_stamp = case['pose']['stamp']
        samples = []
        for record in case['imu_source_records']:
            if record['bag_t'] > scan_stamp+.02:
                continue
            samples.append(dict(stamp=record['stamp'], quaternion=record['quaternion'],
                covariance=record['orientation_covariance'],
                angular_velocity=record['angular_velocity'], frame_id='imu_link', measured=True))
        value = measured_imu_at(samples, scan_stamp)
        self.assertIsNotNone(value)
        self.assertEqual(value['imu_stamp'], 1791968045.6547143)
        self.assertTrue(all(t < value['imu_stamp'] for t in value['imu_neighbour_stamps']))
        self.assertLessEqual(value['imu_adjacent_rate_rad_s'], .2)


if __name__ == '__main__':
    unittest.main()
