"""Pinned-map ground bounds and optional LiDAR snapshot integration."""
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from perception.lidar_geometry import fit_road_plane
from perception.lidar_receiver import LidarCorridor
from perception.mapped_road_floor import MappedRoadFloor

ROOT = Path('/home/david/morai_competition/autodrive/student_v3_claude')
DATA = Path(__file__).parent/'test_data'
CASES = json.loads((DATA/'mapped_road_floor_cases.json').read_text())


def fixture(index=0):
    case = CASES[index]
    with np.load(DATA/case['array']) as data:
        source = np.column_stack([data[k] for k in ('x', 'y', 'z')]).astype(np.float32)
    points = source+np.array([1.05, 0., 1.35], np.float32)
    near = points[(points[:, 0] > 2.) & (points[:, 0] < 15.) & (abs(points[:, 1]) < 3.)]
    plane = fit_road_plane(points, np.percentile(near[:, 2], 5))
    pose = dict(case['pose'], age_s=.12, measured=True)
    return source, points, plane, pose


class MappedRoadFloorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.floor = MappedRoadFloor(ROOT/'link_set.json', ROOT/'global_path.json')

    def test_actual65_66_67_low_road_returns_are_not_obstacles(self):
        for index in range(3):
            with self.subTest(native=CASES[index]['native']):
                _, points, plane, pose = fixture(index)
                old = points[:, 2]-points[:, :2] @ plane[:2]-plane[2]
                mask = ((points[:, 0] > 30.) & (points[:, 0] < 42.) &
                        (abs(points[:, 1]) < 1.3) & (old > .3) & (old < .5))
                self.assertEqual(mask.sum(), 14)
                ground, diagnostic = self.floor.heights(points, plane, pose)
                self.assertTrue(diagnostic['applied'])
                self.assertTrue((points[mask, 2]-ground[mask] < .3).all())
                self.assertEqual(diagnostic['map_mesh_lower_m'], .2)

    def test_low_boxes_and_vertical_bodies_keep_their_height(self):
        _, road, plane, pose = fixture()
        for height in (.31, .4, 1.2, 1.8):
            with self.subTest(height=height):
                x, y = np.meshgrid(np.arange(37.8, 39.1, .2), np.arange(-.9, 1., .2))
                obstacle = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.6+height)))
                points = np.vstack((road, obstacle))
                ground, _ = self.floor.heights(points, plane, pose)
                self.assertTrue((points[-len(obstacle):, 2]-ground[-len(obstacle):] > .3).all())

    def test_unknown_far_surface_cannot_raise_the_calibration(self):
        _, points, plane, pose = fixture()
        far = ((points[:, 0] > 30.) & (points[:, 0] < 45.) & (abs(points[:, 1]) < 6.))
        ground, baseline = self.floor.heights(points, plane, pose)
        for rise in (.31, .4):
            changed = points.copy()
            changed[far, 2] += rise
            elevated_ground, diagnostic = self.floor.heights(changed, plane, pose)
            np.testing.assert_array_equal(elevated_ground, ground)
            self.assertEqual(diagnostic['calibration_offset_m'], baseline['calibration_offset_m'])
            leading = far & (abs(points[:, 1]) < 1.3)
            self.assertTrue((changed[leading, 2]-elevated_ground[leading] > .3).all())

    def test_missing_stale_predicted_or_wrong_route_pose_keeps_original(self):
        _, points, plane, pose = fixture()
        wrong = [None, dict(pose, age_s=.201), dict(pose, age_s=-.1),
                 dict(pose, measured=False), dict(pose, src='file'),
                 dict(pose, heading_deg=pose['heading_deg']+180.),
                 dict(pose, x=pose['x']+5.), dict(pose, y=float('nan'))]
        for value in wrong:
            with self.subTest(pose=value):
                ground, diagnostic = self.floor.heights(points, plane, value)
                self.assertFalse(diagnostic['applied'])
                np.testing.assert_array_equal(ground, points[:, :2] @ plane[:2]+plane[2])

    def test_attitude_must_be_an_actual_nearby_sample_in_the_operating_rate(self):
        _, points, plane, pose = fixture()
        original = points[:, :2] @ plane[:2]+plane[2]
        cases = [dict(pose, imu_stamp=pose['stamp']-.0061,
                      imu_source_stamps=[pose['stamp']-.0061]),
                 dict(pose, imu_stamp=pose['stamp']+.0061,
                      imu_source_stamps=[pose['stamp']+.0061]),
                 dict(pose, imu_source_stamps=[pose['stamp']-.049,
                                               pose['stamp']+.049]),
                 dict(pose, imu_angular_velocity=[.101, 0., 0.]),
                 dict(pose, imu_adjacent_rate_rad_s=.2001),
                 dict(pose, imu_adjacent_rate_rad_s=float('nan')),
                 dict(pose, imu_age_s=pose['age_s']+.02),
                 dict(pose, imu_age_s=0., age_s=0.,
                      imu_stamp=pose['stamp']+.001,
                      imu_source_stamps=[pose['stamp']+.001])]
        for invalid in cases:
            with self.subTest(pose=invalid):
                ground, diagnostic = self.floor.heights(points, plane, invalid)
                self.assertFalse(diagnostic['applied'])
                np.testing.assert_array_equal(ground, original)

    def test_both_sample_time_offsets_lower_the_floor_by_their_full_leverage(self):
        _, points, plane, pose = fixture()
        exact = dict(pose, imu_stamp=pose['stamp'], imu_age_s=pose['age_s'],
                     imu_source_stamps=[pose['stamp']])
        _, reference = self.floor.heights(points, plane, exact)
        self.assertEqual(reference['attitude_timing_lower_max_m'], 0.)
        exact_ground, _ = self.floor.heights(points, plane, exact)
        for delta in (-.0059, -.0019, .0019, .0059):
            sample = pose['stamp']+delta
            timed = dict(pose, imu_stamp=sample, imu_source_stamps=[sample],
                         imu_age_s=pose['age_s']-delta)
            ground, diagnostic = self.floor.heights(points, plane, timed)
            self.assertTrue(diagnostic['applied'])
            self.assertGreater(diagnostic['attitude_timing_lower_max_m'], .018)
            self.assertAlmostEqual(diagnostic['attitude_rate_bound_rad_s'], .2)
            self.assertTrue((ground <= exact_ground+1e-12).all())
            if abs(delta) > .005:
                closer = pose['stamp']+math.copysign(.0019, delta)
                closer_ground, closer_diagnostic = self.floor.heights(points, plane,
                    dict(pose, imu_stamp=closer, imu_source_stamps=[closer],
                         imu_age_s=pose['age_s']-math.copysign(.0019, delta)))
                self.assertGreater(diagnostic['attitude_timing_lower_max_m'],
                                   closer_diagnostic['attitude_timing_lower_max_m'])
                self.assertTrue((ground <= closer_ground+1e-12).all())

    def test_low_box_is_preserved_at_both_six_millisecond_operating_edges(self):
        _, road, plane, pose = fixture()
        x, y = np.meshgrid(np.arange(37.8, 39.1, .2), np.arange(-.9, 1., .2))
        obstacle = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.6+.31)))
        points = np.vstack((road, obstacle))
        for delta in (-.0059, .0059):
            sample = pose['stamp']+delta
            ground, diagnostic = self.floor.heights(points, plane,
                dict(pose, imu_stamp=sample, imu_source_stamps=[sample],
                     imu_age_s=pose['age_s']-delta))
            self.assertTrue(diagnostic['applied'])
            self.assertTrue((points[-len(obstacle):, 2]-ground[-len(obstacle):] > .3).all())

    def test_weak_or_biased_near_calibration_fails_closed(self):
        _, original, plane, pose = fixture()
        for mode in ('narrow', 'sparse', 'raised'):
            with self.subTest(mode=mode):
                points = original.copy()
                if mode == 'narrow':
                    points = points[(points[:, 0] > 15.) | (abs(points[:, 1]) < .2)]
                elif mode == 'sparse':
                    points = points[(points[:, 0] > 15.) | (points[:, 0] < 8.)]
                else:
                    points[points[:, 0] < 15., 2] += .4
                ground, diagnostic = self.floor.heights(points, plane, pose)
                self.assertFalse(diagnostic['applied'])
                np.testing.assert_array_equal(ground, points[:, :2] @ plane[:2]+plane[2])

    def test_map_or_route_revision_is_not_accepted(self):
        for name in ('link_set.json', 'global_path.json'):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temp:
                changed = Path(temp)/name
                changed.write_bytes((ROOT/name).read_bytes()+b' ')
                with self.assertRaisesRegex(ValueError, 'source hash mismatch'):
                    MappedRoadFloor(changed if name == 'link_set.json' else ROOT/'link_set.json',
                                    changed if name == 'global_path.json' else ROOT/'global_path.json')

    def test_near_and_outside_target_returns_are_unchanged(self):
        _, points, plane, pose = fixture()
        ground, _ = self.floor.heights(points, plane, pose)
        near = points[:, 0] <= 20.
        np.testing.assert_array_equal(ground[near], points[near, :2] @ plane[:2]+plane[2])
        far = np.array([[55., 0., .1], [38., 8., .2]])
        points = np.vstack((points, far))
        ground, _ = self.floor.heights(points, plane, pose)
        np.testing.assert_array_equal(ground[-2:], far[:, :2] @ plane[:2]+plane[2])

    def test_deferred_snapshot_preserves_source_time_and_runs_once(self):
        source, points, plane, pose = fixture()
        available = [False]
        provider = lambda stamp: dict(pose, stamp=stamp, imu_stamp=stamp, imu_age_s=pose['age_s'], imu_source_stamps=[stamp]) if available[0] else None
        corridor = LidarCorridor(x_range=(-75., 100.), road_floor=self.floor, pose_at_stamp=provider)
        corridor.ingest(source, stamp=pose['stamp'])
        first = corridor.xy_snapshot()
        available[0] = True
        with patch.object(self.floor, 'heights', wraps=self.floor.heights) as evaluate:
            current = corridor.scan_snapshot()
            corridor.snapshot()
            corridor.xy_snapshot()
            self.assertEqual(evaluate.call_count, 1)
        self.assertEqual(current['stamp'], first['stamp'])
        self.assertEqual(current['sequence'], first['sequence'])
        self.assertEqual(corridor.xy_snapshot()['received'], first['received'])
        np.testing.assert_array_equal(current['xyz'], points)
        np.testing.assert_array_equal(current['ground_plane'], plane)
        self.assertTrue(current['mapped_road_floor']['applied'])
        self.assertGreaterEqual(current['age_s'], first['age_s'])

    def test_source_stamp_mismatch_does_not_apply_a_floor(self):
        source, _, _, pose = fixture()
        corridor = LidarCorridor(road_floor=self.floor,
            pose_at_stamp=lambda stamp: dict(pose, stamp=stamp+.005))
        corridor.ingest(source, stamp=pose['stamp'])
        scan = corridor.scan_snapshot()
        self.assertFalse(scan['mapped_road_floor']['applied'])
        self.assertIsNone(corridor._road_floor_sequence)

    def test_pending_source_pose_never_exposes_a_temporary_road_obstacle_scan(self):
        for index in range(3):
            with self.subTest(native=CASES[index]['native']):
                source, _, _, pose = fixture(index)
                # A visible roadside return is not a newly appearing body in
                # the footprint corridor and must not bypass synchronization.
                source = np.vstack((source, np.array([[8., 1.7, .4]])-[1.05, 0., 1.35]))
                first_stamp, next_stamp = pose['stamp'], pose['stamp']+.07
                entered, ready = threading.Event(), threading.Event()
                entered_at = []
                def provider(stamp):
                    if stamp == next_stamp and not ready.is_set():
                        entered_at.append(time.monotonic())
                        entered.set()
                        return None
                    return dict(pose, stamp=stamp, age_s=.01, imu_stamp=stamp,
                                imu_age_s=.01, imu_source_stamps=[stamp])
                corridor = LidarCorridor(x_range=(-75., 100.), road_floor=self.floor, pose_at_stamp=provider)
                corridor.ingest(source, stamp=first_stamp)
                before = corridor.xy_snapshot()
                self.assertTrue(before['mapped_road_floor']['applied'])
                # The previous cloud may precede the calibrated road section.
                # Its classification must not decide the new cloud's wait.
                with corridor.lock:
                    corridor.road_diagnostics = dict(applied=False, reason='outside source section')
                with patch('perception.lidar_receiver.MAPPED_POSE_WAIT_S', .5):
                    worker = threading.Thread(target=lambda: corridor.ingest(source, stamp=next_stamp))
                    worker.start()
                    self.assertTrue(entered.wait(1.))
                    try:
                        during = corridor.xy_snapshot()
                        self.assertEqual(during['stamp'], first_stamp)
                        np.testing.assert_array_equal(during['points'], before['points'])
                    finally:
                        ready.set()
                        worker.join(2.)
                self.assertFalse(worker.is_alive())
                after = corridor.xy_snapshot()
                self.assertEqual(after['stamp'], next_stamp)
                self.assertTrue(after['mapped_road_floor']['applied'])
                np.testing.assert_array_equal(after['points'], before['points'])
                self.assertLessEqual(after['received'], entered_at[0])

    def test_a_new_near_body_keeps_immediate_raw_protection_without_waiting(self):
        source, _, _, pose = fixture()
        available = [True]
        def provider(stamp):
            return (dict(pose, stamp=stamp, age_s=.01, imu_stamp=stamp,
                         imu_age_s=.01, imu_source_stamps=[stamp]) if available[0] else None)
        corridor = LidarCorridor(x_range=(-75., 100.), road_floor=self.floor, pose_at_stamp=provider)
        corridor.ingest(source, stamp=pose['stamp'])
        self.assertTrue(corridor.xy_snapshot()['mapped_road_floor']['applied'])
        available[0] = False
        obstacle = np.array([[8., 0., .2]])-corridor.mount
        with patch('perception.lidar_receiver.time.sleep', side_effect=AssertionError('near body delayed')):
            corridor.ingest(np.vstack((source, obstacle)), stamp=pose['stamp']+.07)
        scan = corridor.xy_snapshot()
        self.assertFalse(scan['mapped_road_floor']['applied'])
        self.assertTrue(np.any(np.linalg.norm(scan['points']-[8., 0.], axis=1) < .1))

    def test_new_scan_during_deferred_compute_discards_the_old_result(self):
        source, _, _, pose = fixture()
        available = [False]
        corridor = LidarCorridor(road_floor=self.floor,
            pose_at_stamp=lambda stamp: dict(pose, stamp=stamp, imu_stamp=stamp, imu_age_s=pose['age_s'], imu_source_stamps=[stamp]) if available[0] else None)
        corridor.ingest(source, stamp=pose['stamp'])
        entered, release = threading.Event(), threading.Event()
        original = self.floor.heights
        def blocked(*args):
            entered.set()
            self.assertTrue(release.wait(2.))
            return original(*args)
        available[0] = True
        with patch.object(self.floor, 'heights', side_effect=blocked):
            worker = threading.Thread(target=corridor.scan_snapshot)
            worker.start()
            self.assertTrue(entered.wait(1.))
            available[0] = False
            corridor.ingest(source, stamp=pose['stamp']+.1)
            release.set()
            worker.join(2.)
            self.assertFalse(worker.is_alive())
        scan = corridor.scan_snapshot()
        self.assertEqual(scan['stamp'], pose['stamp']+.1)
        self.assertEqual(scan['sequence'], 2)
        self.assertFalse(scan['mapped_road_floor']['applied'])
        self.assertIsNone(corridor._road_floor_sequence)


if __name__ == '__main__':
    unittest.main()
