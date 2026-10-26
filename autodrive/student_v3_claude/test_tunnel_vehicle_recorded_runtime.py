"""Replay real tunnel vehicle gaps, measured memory, and surveyed exit support."""
import unittest
import numpy as np

import test_tunnel_latest_recorded_runtime as recorded_fixture


class VehicleRecordedTunnelRuntimeTests(unittest.TestCase):
    receiver = staticmethod(recorded_fixture.LatestRecordedTunnelRuntimeTests.receiver)
    make_runtime = classmethod(recorded_fixture.LatestRecordedTunnelRuntimeTests.make_runtime.__func__)
    records = classmethod(recorded_fixture.LatestRecordedTunnelRuntimeTests.records.__func__)

    @classmethod
    def setUpClass(cls):
        recorded_fixture.LatestRecordedTunnelRuntimeTests.setUpClass.__func__(cls)
        capture = cls.root/'data/autodrive/follow_runs/follow-20261003T014329Z-2388302/tunnel_analysis/tunnel_vehicle_representative.npz'
        with np.load(capture) as archive:
            cls.data = {key: archive[key] for key in archive.files}
        cls._records = cls._runtime = None

    def test_recorded_vehicle_memory_pose_and_carry_never_publish_a_false_stop(self):
        for index, row in self.records().items():
            with self.subTest(source_index=index):
                self.assertTrue(row['tunnel']['valid'], row['reason'])
                self.assertFalse(row['emergency_stop'], row.get('stop_reason'))
                self.assertGreater(row['planner_cap_kmh'], 0.)
                self.assertTrue(np.isfinite(row['published_path']).all())

    def test_early_vehicle_bypass_preserves_the_twenty_kmh_target(self):
        for index in (69, 78, 90):
            row = self.records()[index]
            with self.subTest(source_index=index):
                self.assertEqual(row['mode'], 'TUNNEL BYPASS')
                self.assertGreaterEqual(row['planner_cap_kmh'], 20.)
                self.assertGreater(row['tunnel']['geometry_clear_distance_m'], 20.)

    def test_static_vehicle_face_and_surveyed_exit_keep_fresh_collision_evidence(self):
        records = self.records()
        for index in (127, 128):
            self.assertEqual(records[index]['tunnel']['moving_objects'], 0)
        for index in range(1181, 1187):
            row = records[index]
            with self.subTest(source_index=index):
                stats = row['tunnel']['wall_stats']
                self.assertTrue(stats['surveyed_exit_support'])
                self.assertGreaterEqual(min(stats['left_n'], stats['right_n']), 100)
                self.assertGreaterEqual(max(stats['left_span_m'], stats['right_span_m']), 14.)
                self.assertGreaterEqual(min(stats['left_span_m'], stats['right_span_m']), 10.)
                self.assertGreater(row['tunnel']['observed_clear_distance_m'], 15.)


if __name__ == '__main__':
    unittest.main()
