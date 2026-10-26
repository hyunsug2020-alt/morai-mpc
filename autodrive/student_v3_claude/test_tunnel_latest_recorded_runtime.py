"""Regress the latest tunnel's wall visibility, gentle bypass and surface jitter."""
import json
import os
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

import test_avoidance_gate as avoidance_fixture
from gui import HdMapLanes, Runtime, path_curvatures, smooth_control_route
from lidar_corridor import LidarCorridor
from tunnel_map import TunnelMap
from tunnel_runtime import pose_at_stamp


class LatestRecordedTunnelRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets = Path(__file__).resolve().parent
        cls.root = cls.assets.parents[1]
        capture = cls.root/'data/autodrive/follow_runs/follow-20261002T165436Z-1549766/tunnel_analysis/tunnel_scans_representative.npz'
        with np.load(capture) as archive:
            cls.data = {key: archive[key] for key in archive.files}
        cls.waypoints = json.loads((cls.assets/'global_path.json').read_text())['waypoints']
        cls.world = np.asarray([[p['x'], p['y']] for p in cls.waypoints])
        cls.control_world = smooth_control_route(cls.world)
        cls.rules = json.loads((cls.root/'data/autodrive/drivable_rules.json').read_text())
        cls._records = cls._runtime = None

    @staticmethod
    def receiver(values, capacity):
        poses = sorted([dict(stamp=e[1], x=e[2], y=e[3], heading_deg=e[4], speed_kmh=e[5],
                             age_s=.01, **(dict(accel=e[6], brake=e[7], wheel_angle=e[8])
                                           if len(e) > 8 else {})) for e in values],
                       key=lambda pose: pose['stamp'])
        times = np.asarray([pose['stamp'] for pose in poses])

        def at_stamp(stamp):
            index = int(np.searchsorted(times, stamp))
            return pose_at_stamp(poses[max(0, index-capacity+2):min(len(poses), index+2)], stamp)
        return SimpleNamespace(at_stamp=at_stamp)

    @classmethod
    def make_runtime(cls):
        runtime = avoidance_fixture.AvoidanceGateTests().runtime()
        # 2026-10-02 capture: ESKF poses in the frame the walls were surveyed in.
        runtime.tunnel = TunnelMap.load(cls.assets/'tunnel_wall_map.json', survey_frame=True)
        runtime.config.update(route_points=20)
        runtime.waypoints, runtime.waypoint_xy = cls.waypoints, cls.world
        runtime.control_waypoint_xy = cls.control_world
        with patch.dict(os.environ, {'STUDENT_DRIVABLE_RULES': ''}):
            runtime.lanes = HdMapLanes(cls.assets/'link_set.json')
        runtime.lanes.rules_cache = SimpleNamespace(value=cls.rules)
        runtime.lanes.set_route(cls.world)
        runtime.route_tensor = lambda ego, status: Runtime.route_tensor(runtime, ego, status)
        runtime.lidar = LidarCorridor(x_range=(-75., 100.), mount=cls.data['sensor_mount'])
        runtime.ego_receiver = cls.receiver(cls.data['ego'], 400)
        runtime.tunnel_odometry = cls.receiver(cls.data['odometry'], 800)
        runtime.receiver.snapshot = Mock(side_effect=AssertionError('Camera input used inside tunnel'))
        runtime.model = Mock(side_effect=AssertionError('Camera model used inside tunnel'))
        runtime.object_receiver.snapshot = Mock(side_effect=AssertionError('Semantic objects used inside tunnel'))
        runtime.path_publisher.tunnel_constraint_pub = avoidance_fixture.Sink()
        runtime.path_publisher._tunnel_constraint_stamp = 0.
        return runtime

    @classmethod
    def records(cls):
        if cls._records is None:
            runtime, records = cls.make_runtime(), {}
            for index, (_, stamp) in enumerate(cls.data['stamps']):
                ego = runtime.ego_receiver.at_stamp(float(stamp))
                if ego is None or not runtime.tunnel.active(ego):
                    raise AssertionError('Representative scan lacks an active tunnel pose')
                nearest = int(np.argmin(np.linalg.norm(cls.world-[ego['x'], ego['y']], axis=1)))
                runtime.test_status = dict(ego=ego, path=dict(nearest_idx=nearest, distance_m=0.))
                a, b = cls.data['offsets'][index:index+2]
                clock = 1000.+float(stamp)-float(cls.data['stamps'][0, 1])
                with patch('tunnel_runtime.time.monotonic', return_value=clock):
                    runtime.lidar.ingest(cls.data['xyz'][a:b], stamp=float(stamp))
                    if not runtime.infer():
                        raise AssertionError('Recorded tunnel produced no decision')
                record = dict(runtime._web_record)
                record['published_path'] = runtime.last_result['path'].copy()
                records[int(cls.data['original_scan_indices'][index])] = record
            runtime.receiver.snapshot.assert_not_called()
            runtime.model.assert_not_called()
            runtime.object_receiver.snapshot.assert_not_called()
            cls._records, cls._runtime = records, runtime
        return cls._records

    def test_recorded_entry_boundaries_surface_jitter_and_stopped_exit_stay_driveable(self):
        records = self.records()
        self.assertEqual(len(records), 18)
        for source_index, row in records.items():
            with self.subTest(source_index=source_index):
                self.assertTrue(row['tunnel']['valid'], row['reason'])
                self.assertFalse(row['emergency_stop'], row.get('stop_reason'))
                if source_index == 250 and row['tunnel']['wall_stats'].get('extraction') == 'background':
                    # M7: the leaning upper wall (0.3 m inward, 1.4-2.1 m high) no longer
                    # reads as obstacles, so the car keeps the route instead of the old
                    # 0.5 m kerb-triggered bypass. The route passes the right exit portal
                    # 10 m ahead, which bounds ray support (not obstacles) at 11 m.
                    self.assertAlmostEqual(row['tunnel']['clearance_components_m']['obstacle'], 60., delta=.001)
                    self.assertGreaterEqual(row['tunnel']['observed_clear_distance_m'], 10.)
                    self.assertGreaterEqual(row['planner_cap_kmh'], 20.)
                else:
                    self.assertGreaterEqual(row['tunnel']['observed_clear_distance_m'], 15.)
                self.assertGreater(row['planner_cap_kmh'], 0.)
                self.assertTrue(np.isfinite(row['published_path']).all())
        # The four consecutive measurements previously misclassified a static
        # vehicle face as a pedestrian crossing at source scan 206.
        self.assertEqual(records[206]['tunnel']['moving_objects'], 0)

    def test_recorded_small_entry_object_has_a_gentle_measured_bypass(self):
        row = self.records()[7]
        self.assertEqual(row['mode'], 'TUNNEL BYPASS')
        self.assertGreater(abs(row['applied_peak_m']), .1)
        # Wide pass preferred in the tunnel (user 2026-10-04); still gentle (curvature below).
        self.assertLessEqual(abs(row['applied_peak_m']), 2.0)
        self.assertLess(float(np.max(path_curvatures(row['published_path']))), .035)   # ~1.9 m/s^2 at 30 km/h
        self.assertFalse(row['emergency_stop'])

    def test_recorded_stopped_exit_keeps_a_usable_speed_cap(self):
        row = self.records()[1296]
        self.assertGreater(row['tunnel']['geometry_clear_distance_m'], 20.)
        self.assertGreater(row['tunnel']['observed_clear_distance_m'], 20.)
        self.assertFalse(row['emergency_stop'])
        self.assertGreaterEqual(row['planner_cap_kmh'], 6.)


if __name__ == '__main__':
    unittest.main()
