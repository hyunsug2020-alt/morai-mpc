"""Replay recorded VLP16 scans through the competition tunnel Runtime with inert sinks."""
import json
import math
import os
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

import test_avoidance_gate as avoidance_fixture
from gui import HdMapLanes, Runtime, smooth_control_route
from lidar_corridor import LidarCorridor
from tunnel_map import TunnelMap
from tunnel_runtime import pose_at_stamp, swept_clearance
from ego_body import REAR_AXLE_SHIFT_M


class RecordedTunnelRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets = Path(__file__).resolve().parent
        cls.root = cls.assets.parents[1]
        capture = cls.root/'data/autodrive/follow_runs/follow-20261002T152952Z-1309248/tunnel_analysis/tunnel_scans.npz'
        with np.load(capture) as data:
            cls.xyz, cls.offsets, cls.stamps, ego = (data[key] for key in ('xyz', 'offsets', 'stamps', 'ego'))
        cls.poses = [dict(stamp=float(e[1]), x=float(e[2]), y=float(e[3]),
                          heading_deg=float(e[4]), speed_kmh=math.hypot(e[5], e[6])*3.6,
                          age_s=.01) for e in ego]
        # Runtime tracks measured surfaces in wheel/IMU odometry. Using the
        # corrected ESKF pose here fabricates object motion during localization
        # updates, so replay the same uncorrected source recorded in this bag.
        with np.load(capture.with_name('tunnel_odometry.npz')) as data:
            cls.odometry_poses = [dict(stamp=float(e[1]), x=float(e[2]), y=float(e[3]),
                heading_deg=float(e[4]), speed_kmh=float(e[5]), age_s=.01)
                for e in data['odometry']]
        cls.waypoints = json.loads((cls.assets/'global_path.json').read_text())['waypoints']
        cls.world = np.array([[p['x'], p['y']] for p in cls.waypoints])
        cls.control_world = smooth_control_route(cls.world)
        cls.rules = json.loads((cls.root/'data/autodrive/drivable_rules.json').read_text())
        cls._empty_records = None

    @classmethod
    def make_runtime(cls):
        runtime = avoidance_fixture.AvoidanceGateTests().runtime()
        # 2026-10-02 capture: ESKF poses in the frame the walls were surveyed in.
        runtime.tunnel = TunnelMap.load(cls.assets/'tunnel_wall_map.json', survey_frame=True)
        runtime.config.update(route_points=20)
        runtime.waypoints, runtime.waypoint_xy = cls.waypoints, cls.world
        runtime.control_waypoint_xy = cls.control_world
        # Freeze operator rules for deterministic replay; no polling thread or live ROS node.
        with patch.dict(os.environ, {'STUDENT_DRIVABLE_RULES': ''}):
            runtime.lanes = HdMapLanes(cls.assets/'link_set.json')
        runtime.lanes.rules_cache = SimpleNamespace(value=cls.rules)
        runtime.lanes.set_route(cls.world)
        runtime.route_tensor = lambda current, status: Runtime.route_tensor(runtime, current, status)
        runtime.lidar = LidarCorridor(x_range=(-75., 100.))
        runtime.ego_receiver = SimpleNamespace(at_stamp=lambda stamp: pose_at_stamp(cls.poses, stamp))
        runtime.tunnel_odometry = SimpleNamespace(
            at_stamp=lambda stamp: pose_at_stamp(cls.odometry_poses, stamp))
        runtime.receiver.snapshot = Mock(side_effect=AssertionError('Camera input used inside tunnel'))
        runtime.model = Mock(side_effect=AssertionError('Camera model used inside tunnel'))
        runtime.object_receiver.snapshot = Mock(side_effect=AssertionError('Semantic objects used inside tunnel'))
        runtime.path_publisher.tunnel_constraint_pub = avoidance_fixture.Sink()
        runtime.path_publisher._tunnel_constraint_stamp = 0.
        return runtime

    @classmethod
    def run_frame(cls, runtime, index, injected_body_points=None):
        stamp = float(cls.stamps[index, 1])
        ego = pose_at_stamp(cls.poses, stamp)
        if ego is None or not runtime.tunnel.active(ego):
            return None
        a, b = cls.offsets[index:index+2]
        raw = cls.xyz[a:b]
        if injected_body_points is not None:
            raw = np.vstack((raw, np.asarray(injected_body_points)-runtime.lidar.mount))
        nearest = int(np.argmin(np.linalg.norm(cls.world-[ego['x'], ego['y']], axis=1)))
        runtime.test_status = dict(ego=ego, path=dict(nearest_idx=nearest, distance_m=0.))
        # Advance by recorded source time, preserving the 0.1 s motion-tracker baseline.
        clock = 1000.+stamp-float(cls.stamps[0, 1])
        with patch('tunnel_runtime.time.monotonic', return_value=clock):
            runtime.lidar.ingest(raw, stamp=stamp)
            if not runtime.infer():
                raise AssertionError('Tunnel Runtime did not publish a decision')
        return runtime._web_record

    @classmethod
    def empty_records(cls):
        if cls._empty_records is None:
            runtime = cls.make_runtime()
            records = {}
            for index in range(len(cls.stamps)):
                record = cls.run_frame(runtime, index)
                if record is not None:
                    records[index] = record
            runtime.receiver.snapshot.assert_not_called()
            runtime.model.assert_not_called()
            runtime.object_receiver.snapshot.assert_not_called()
            cls._empty_records = records
        return cls._empty_records

    def test_entire_empty_record_has_no_camera_false_bypass_or_emergency_stop(self):
        records = self.empty_records()
        self.assertEqual(len(records), 137)
        invalid = [(index, row['tunnel']['reason']) for index, row in records.items()
                   if not row['tunnel']['valid']]
        bypasses = [(index, row['mode']) for index, row in records.items() if row['mode'] != 'TUNNEL']
        stops = [(index, row['reason']) for index, row in records.items() if row['emergency_stop']]
        self.assertEqual(invalid, [])
        self.assertEqual(bypasses, [])
        self.assertEqual(stops, [])
        self.assertTrue(all(math.isfinite(row['planner_cap_kmh']) and row['planner_cap_kmh'] > 0.
                            for row in records.values()))

    def test_recorded_exit_uses_wall_space_without_an_artificial_lane_recovery(self):
        records = self.empty_records()
        for index in (213, 214):
            with self.subTest(scan=index):
                row = records[index]
                self.assertTrue(row['tunnel']['valid'])
                self.assertFalse(row['tunnel']['boundary_recovery'])
                self.assertGreaterEqual(row['tunnel']['geometry_clear_distance_m'], 20.)
                self.assertGreaterEqual(row['tunnel']['clearance_components_m']['wall'], 20.)
                self.assertGreater(row['planner_cap_kmh'], 0.)
                self.assertFalse(row['emergency_stop'])

    def test_injected_centre_box_produces_a_nonzero_measured_safe_bypass(self):
        box = np.array([[x, y, z] for x in np.arange(22., 23., .15)
                        for y in np.arange(-.5, .5, .15) for z in np.arange(.3, 1.2, .15)])
        for index in (100, 120, 140):
            with self.subTest(scan=index):
                runtime = self.make_runtime()
                record = self.run_frame(runtime, index, box)
                self.assertIsNotNone(record)
                self.assertTrue(record['tunnel']['valid'])
                self.assertEqual(record['mode'], 'TUNNEL BYPASS')
                self.assertGreater(abs(record['applied_peak_m']), .1)
                self.assertFalse(record['emergency_stop'], record['reason'])
                self.assertTrue(math.isfinite(record['planner_cap_kmh']))
                self.assertGreater(record['planner_cap_kmh'], 0.)
                self.assertLessEqual(record['planner_cap_kmh'], 28.94+1e-6)
                path = runtime.last_result['path']
                self.assertTrue(np.isfinite(path).all())
                horizon = float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum())
                aligned = record['tunnel']['aligned_ego']
                self.assertGreaterEqual(runtime.tunnel.path_clearance(path, aligned, horizon), horizon-.3)
                measured_points = runtime._tunnel_cache['match']['obstacle_points']
                # The real body on the rear-axle pose, as the planner checks it (ego_body).
                self.assertGreaterEqual(swept_clearance(path, measured_points, horizon,
                                                        body_shift_m=REAR_AXLE_SHIFT_M), horizon-.3)
                runtime.receiver.snapshot.assert_not_called()
                runtime.model.assert_not_called()
                runtime.object_receiver.snapshot.assert_not_called()


if __name__ == '__main__':
    unittest.main()
