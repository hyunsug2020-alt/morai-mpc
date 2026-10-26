"""Measured portal barriers survive bounded scan gaps in their actual frame."""
import json
import math
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from planning.tunnel.approach_memory import TunnelApproachMemory, update_approach_points


def pose(stamp=100., x=0., y=0., heading=0., speed=0.):
    return dict(stamp=stamp, x=x, y=y, heading_deg=heading, speed_kmh=speed)


class TunnelApproachMemoryTests(unittest.TestCase):
    def test_first_unknown_return_is_immediately_available(self):
        memory = TunnelApproachMemory()
        points = np.array([[40., .1], [40.1, -.2]])
        actual = memory.update(points, pose(), 100.)
        self.assertEqual(set(map(tuple, actual)), set(map(tuple, points)))

    def test_empty_valid_scan_keeps_barrier_through_1_1_second_gap(self):
        memory = TunnelApproachMemory()
        memory.update([[40., 0.]], pose(speed=36.), 100.)
        actual = memory.update([], pose(101.1, x=11., speed=36.), 101.1)
        np.testing.assert_allclose(actual, [[29., 0.]])
        self.assertEqual(len(memory.snapshot(pose(101.51, x=15.1, speed=36.), 101.51)), 0)

    def test_duplicate_stamp_cannot_refresh_barrier_age(self):
        memory = TunnelApproachMemory()
        memory.update([[40., 0.]], pose(), 100.)
        memory.update([[40., 0.]], pose(), 100.)
        self.assertEqual(memory.cells[next(iter(memory.cells))][0], 100.)
        self.assertEqual(len(memory.snapshot(pose(101.51), 101.51)), 0)

    def test_lane_change_and_rotation_use_world_evidence(self):
        memory = TunnelApproachMemory()
        memory.update([[40., 0.]], pose(speed=36.), 100.)
        current = pose(101., x=10., y=3.5, heading=30., speed=36.)
        angle = math.radians(30.)
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        expected = (np.array([[40., 0.]])-[10., 3.5])@rotation
        np.testing.assert_allclose(memory.snapshot(current, 101.), expected)

    def test_clock_reset_and_pose_jump_clear_the_old_world(self):
        for operation in ('update', 'snapshot'):
            memory = TunnelApproachMemory()
            memory.update([[40., 0.]], pose(), 100.)
            if operation == 'update':
                actual = memory.update([], pose(50.), 50.)
            else:
                actual = memory.snapshot(pose(50.), 50.)
            self.assertEqual(len(actual), 0)
            self.assertEqual(memory.cells, {})
        memory.update([[40., 0.]], pose(50.), 50.)
        self.assertEqual(len(memory.snapshot(pose(50.1, x=30.), 50.1)), 0)

    @staticmethod
    def car(stamp=101., confirmed=True, age=0.):
        return (42., 0., 4., 'measured_car', 'npc', 2., 4., 1.8, 0., 2., 0., 12,
                dict(source_stamp=stamp, age_s=age), None, confirmed, age)

    def test_fresh_confirmed_mover_removes_its_current_and_old_footprints(self):
        memory = TunnelApproachMemory()
        memory.update([[40., 0.], [25., 5.]], pose(), 100.)
        actual = memory.update([[42., 0.]], pose(101.), 101., [self.car()])
        np.testing.assert_allclose(actual, [[25., 5.]])

    def test_unknown_or_stale_motion_cannot_erase_a_barrier(self):
        for car in (self.car(confirmed=False), self.car(stamp=100.), self.car(age=.31)):
            memory = TunnelApproachMemory()
            memory.update([[40., 0.]], pose(), 100.)
            np.testing.assert_allclose(memory.update([], pose(101.), 101., [car]), [[40., 0.]])

    def test_large_cloud_is_bounded_without_losing_sparse_far_barrier(self):
        memory = TunnelApproachMemory(max_points=256)
        x = np.linspace(1., 30., 180)
        y = np.linspace(-3., 3., 100)
        cloud = np.column_stack((np.repeat(x, len(y)), np.tile(y, len(x))))
        cloud = np.vstack((cloud, [[65., 0.]]))
        actual = memory.update(cloud, pose(), 100.)
        self.assertLessEqual(len(memory.cells), 256)
        self.assertTrue(np.any(np.all(actual == [65., 0.], axis=1)))

    def test_invalid_pose_source_contract_is_rejected(self):
        memory = TunnelApproachMemory()
        with self.assertRaises(ValueError):
            memory.update([[40., 0.]], pose(100.5), 100.)
        with self.assertRaises(ValueError):
            memory.snapshot(pose(), float('nan'))
        with self.assertRaises(ValueError):
            memory.snapshot(dict(pose(), age_s=.26), 100.)

    def test_recorded_eight_low_barrier_returns_survive_the_measured_gap(self):
        from lidar_corridor import LidarCorridor
        from tunnel_runtime import swept_clearance
        root = Path(__file__).resolve().parents[2]
        capture = root/'data/autodrive/follow_runs/follow-20261009T102648Z-28688/raw_sensor_audit'
        if not (capture/'scans.jsonl').exists():
            self.skipTest('Raw portal audit is unavailable')
        records = {row['file']: row for row in map(json.loads, (capture/'scans.jsonl').read_text().splitlines())}
        barrier = np.array([18.5709991455, -494.2879943848])
        memory = TunnelApproachMemory()
        selected = []
        for filename in ('00403.npy', '00410.npy'):
            row = records[filename]
            lidar = LidarCorridor()
            xyz = np.load(capture/filename)
            lidar.ingest(xyz, stamp=row['stamp'])
            body = xyz+lidar.mount
            ego = dict(row['ego'], stamp=row['stamp'])
            position, rotation = memory._pose(ego, row['stamp'])
            world = body[:, :2]@rotation.T+position
            height = body[:, 2]-(body[:, :2]@lidar.ground_plane[:2]+lidar.ground_plane[2])
            keep = (np.linalg.norm(world-barrier, axis=1) < 2.5) & (height > .3) & (height < 2.2)
            selected.append(body[keep, :2])
            remembered = memory.update(selected[-1], ego, row['stamp'])
        self.assertEqual(len(selected[0]), 8)
        self.assertEqual(len(selected[1]), 0)
        self.assertGreater(len(remembered), 0)
        local_barrier = (barrier-position)@rotation
        path = np.linspace([0., 0.], local_barrier*1.8, 150)
        self.assertGreater(swept_clearance(path, selected[1], 60.), 55.)
        self.assertLess(swept_clearance(path, remembered, 60.), 40.)


class TunnelApproachAdapterTests(unittest.TestCase):
    def setUp(self):
        self.scan = dict(stamp=100., age_s=.01, xyz=np.ones((1, 3)))
        self.ego = dict(pose(100., x=10., speed=36.), age_s=.01)
        self.points = np.array([[40., 0.]])
        self.runtime = SimpleNamespace(_transition_entry_obstacles=True,
            ego_receiver=SimpleNamespace(at_stamp=Mock(return_value=self.ego)))

    def test_source_measurement_is_returned_in_the_current_control_frame(self):
        query = dict(self.ego, stamp=100.3, x=13.)
        actual = update_approach_points(self.runtime, self.scan, self.points, query)
        np.testing.assert_allclose(actual, [[37., 0.]])
        self.runtime.ego_receiver.at_stamp.assert_called_once_with(100.)
        status = self.runtime._portal_memory_status
        self.assertTrue(status['valid'])
        self.assertEqual(status['source_count'], 1)
        self.assertEqual(status['retained_count'], 1)
        self.assertEqual(status['max_age_s'], 1.5)

    def test_inactive_passthrough_resets_portal_evidence(self):
        update_approach_points(self.runtime, self.scan, self.points, self.ego)
        self.runtime._transition_entry_obstacles = False
        self.assertIs(update_approach_points(self.runtime, self.scan, self.points, self.ego), self.points)
        self.assertEqual(self.runtime._portal_approach_memory.cells, {})
        self.assertFalse(self.runtime._portal_memory_status['active'])

    def test_missing_legacy_interface_preserves_the_callers_points(self):
        for runtime, scan in ((SimpleNamespace(_transition_entry_obstacles=True), self.scan),
                              (self.runtime, {})):
            self.assertIs(update_approach_points(runtime, scan, self.points, self.ego), self.points)
            self.assertIsNone(runtime._portal_memory_status['valid'])
            self.assertEqual(runtime._portal_memory_status['quality'], 'legacy unchecked')

    def test_actual_stale_or_invalid_sensors_return_none(self):
        variants = [(dict(self.scan, age_s=.61), self.ego),
                    (dict(self.scan, age_s=-.01), self.ego),
                    (dict(self.scan, xyz=None), self.ego),
                    (dict(self.scan, stamp=float('nan')), self.ego),
                    (self.scan, dict(self.ego, age_s=.26)),
                    (self.scan, dict(self.ego, stamp=100.61)),
                    (self.scan, dict(self.ego, stamp=99.87))]
        for scan, ego in variants:
            with self.subTest(scan=scan['stamp'], ego=ego['stamp']):
                self.assertIsNone(update_approach_points(self.runtime, scan, self.points, ego))
                self.assertFalse(self.runtime._portal_memory_status['valid'])
                self.assertTrue(self.runtime._portal_memory_status['reason'])

    def test_missing_or_stale_source_pose_rejects_and_fresh_pose_recovers(self):
        for source_pose in (None, dict(self.ego, stamp=99.), dict(self.ego, age_s=.26)):
            self.runtime.ego_receiver.at_stamp.return_value = source_pose
            self.assertIsNone(update_approach_points(self.runtime, self.scan, self.points, self.ego))
        self.runtime.ego_receiver.at_stamp.return_value = self.ego
        np.testing.assert_allclose(update_approach_points(self.runtime, self.scan, self.points, self.ego), self.points)

    def test_same_source_skips_pose_lookup_and_voxelization(self):
        update_approach_points(self.runtime, self.scan, self.points, self.ego)
        memory = self.runtime._portal_approach_memory
        memory.update = Mock(wraps=memory.update)
        actual = update_approach_points(self.runtime, self.scan, self.points,
                                       dict(self.ego, stamp=100.1, x=11.))
        np.testing.assert_allclose(actual, [[39., 0.]])
        memory.update.assert_not_called()
        self.runtime.ego_receiver.at_stamp.assert_called_once_with(100.)
        self.assertEqual(next(iter(memory.cells.values()))[0], 100.)

    def test_full_current_cloud_survives_the_memory_bound_and_crop(self):
        points = np.column_stack((np.repeat(np.linspace(-5., 35., 80), 75),
                                  np.tile(np.linspace(-8., 8., 75), 80)))
        points[:2] = [[-20., -30.], [100., 30.]]
        query = dict(self.ego, stamp=100.1, x=11., heading_deg=30.)
        actual = update_approach_points(self.runtime, self.scan, points, query)
        position, rotation = TunnelApproachMemory._pose(query, query['stamp'])
        expected = (points+[10., 0.]-position)@rotation
        np.testing.assert_allclose(actual, expected)
        self.assertEqual(len(actual), 6000)
        self.assertEqual(len(self.runtime._portal_approach_memory.cells), 4096)

    def test_empty_new_scan_retains_previous_barrier_in_current_frame(self):
        update_approach_points(self.runtime, self.scan, self.points, self.ego)
        query = dict(self.ego, stamp=101.1, x=21.)
        self.runtime.ego_receiver.at_stamp.return_value = query
        actual = update_approach_points(self.runtime, dict(self.scan, stamp=101.1), [], query)
        np.testing.assert_allclose(actual, [[29., 0.]])
        self.assertEqual(self.runtime._portal_memory_status['remembered_count'], 1)

    def test_late_scan_cannot_delete_newer_evidence_but_clock_reset_does(self):
        update_approach_points(self.runtime, self.scan, self.points, self.ego)
        current = dict(self.ego, stamp=100.2, x=12.)
        self.runtime.ego_receiver.at_stamp.return_value = current
        update_approach_points(self.runtime, dict(self.scan, stamp=100.2), [], current)
        late = dict(self.ego, stamp=100.1, x=11.)
        self.runtime.ego_receiver.at_stamp.return_value = late
        actual = update_approach_points(self.runtime, dict(self.scan, stamp=100.1), [], current)
        np.testing.assert_allclose(actual, [[38., 0.]])
        self.assertEqual(self.runtime._portal_approach_memory.last_scan, 100.2)
        self.assertTrue(self.runtime._portal_memory_status['out_of_order'])
        reset = dict(self.ego, stamp=50., x=0.)
        self.runtime.ego_receiver.at_stamp.return_value = reset
        actual = update_approach_points(self.runtime, dict(self.scan, stamp=50.), [], reset)
        self.assertEqual(len(actual), 0)
        self.assertEqual(self.runtime._portal_approach_memory.cells, {})


if __name__ == '__main__':
    unittest.main()
