"""Actual-map regression for the 2026-10-16 stopped on-ramp vehicle.

The replay uses production inference/publication with synthetic fresh sensors
and a simple forward follower. It does not replace a MORAI dynamics test.
"""
import copy
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

import numpy as np

import gui
from highway_planner import HighwayPlanner, NearFieldMemory, local_points, world_points
from lidar_corridor import LidarCorridor
from planning.highway.lane_map import HighwayLaneMap
from planning.highway.policy import (ALIGN_ESCAPE_M, corridor_overrun,
                                     inward_recovery_clearance, keep_path)
import test_highway_integration as fixtures


HERE = Path(__file__).parent


class RecordedHighwayAlignmentStall(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(HERE/'test_data/highway_alignment_2155_20261016.npz') as frame:
            cls.frame = {key: frame[key].copy() for key in frame.files}
        cls.ego = json.loads(str(cls.frame['ego']))
        cls.mission = json.loads(str(cls.frame['mission']))
        cls.rules = json.loads(str(cls.frame['rules']))
        cls.world = np.array([[p['x'], p['y']] for p in json.loads(
            (HERE/'global_path.json').read_text())['waypoints']])
        cls.smooth = gui.smooth_control_route(cls.world)

    def lanes(self):
        with patch.dict(os.environ, {'STUDENT_DRIVABLE_RULES': ''}):
            lanes = gui.HdMapLanes(HERE/'link_set.json')
        lanes.rules_cache = SimpleNamespace(value=copy.deepcopy(self.rules))
        lanes.set_route(self.world)
        return lanes

    def geometry(self, ego=None):
        ego = self.ego if ego is None else ego
        index = int(np.argmin(np.linalg.norm(self.world-[ego['x'], ego['y']], axis=1)))
        route = local_points(self.smooth[(index+np.arange(640)) % len(self.smooth)], ego)
        lane_map = HighwayLaneMap(self.lanes(), copy.deepcopy(self.mission))
        options = lane_map.candidates(ego, route)
        return index, route, options, lane_map.corridor(ego, route, options)

    def runtime(self):
        runtime = gui.Runtime.__new__(gui.Runtime)
        runtime.highway = HighwayPlanner(copy.deepcopy(self.mission), self.world)
        runtime.lanes = self.lanes()
        runtime.near_field_memory = NearFieldMemory()
        runtime.lidar = LidarCorridor()
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
            [], True, dict(ready=True, vehicles=[], stamp=runtime.test_ego['stamp'], age_s=.01)))
        runtime.object_receiver = SimpleNamespace(snapshot=lambda: [], age_s=.01)
        runtime.ego_receiver = SimpleNamespace(at_stamp=lambda stamp: dict(runtime.test_ego, stamp=stamp))
        runtime.receiver = SimpleNamespace(rates=lambda: dict(front=30., left=30., right=30.))
        runtime.actions, runtime.gallery, runtime.gallery_matrix = ['DRIVE', 'STOP'], [], np.empty((0, 2))
        runtime.args = SimpleNamespace(status=Path('/tmp/highway_alignment_test_status.json'))
        runtime._decision_log = SimpleNamespace(write=lambda line: None, dropped=0)
        publisher = gui.RosPathPublisher.__new__(gui.RosPathPublisher)
        publisher.Float32 = publisher.String = fixtures.Message
        publisher.PoseStamped, publisher.PathMessage = fixtures.Pose, fixtures.PathMessage
        publisher.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: 1.))
        for name in ('waypoints_pub', 'velocity_pub', 'mode_pub', 'path_pub', 'status_pub'):
            setattr(publisher, name, fixtures.Sink())
        publisher.cruise_mps = 120./3.6
        publisher.publish_camera_rates = lambda *args: None
        publisher.publish_corridor = lambda *args: None
        runtime.path_publisher = publisher
        return runtime

    def infer(self, runtime, ego, now, points=()):
        ego = dict(ego, stamp=now, received=now, age_s=.01)
        runtime.test_ego = ego
        runtime.lidar.received, runtime.lidar._scan_stamp = now, now
        runtime.lidar.pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        runtime.object_receiver.observation_stamp = now
        runtime.object_receiver.received = now
        index = int(np.argmin(np.linalg.norm(self.world-[ego['x'], ego['y']], axis=1)))
        route = local_points(self.smooth[(index+np.arange(640)) % len(self.smooth)], ego)
        status = dict(ego=ego, path=dict(nearest_idx=index,
            distance_m=float(np.linalg.norm(self.world[index]-[ego['x'], ego['y']]))))
        self.assertTrue(runtime.infer_highway(status, ego, route, route[:20], route[::10][:20],
            route[:20], np.empty(0), np.empty(0), np.empty(0), 0., time.perf_counter()))
        payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        return payload, runtime._web_record['highway']

    def test_recorded_path_reuses_existing_bounded_inward_recovery(self):
        index, route, options, corridor = self.geometry()
        self.assertEqual(index, 2155)
        self.assertEqual([lane.link_id for lane in options], ['A2256W000846'])
        path = keep_path(options[0].centre, 12., 0., corridor.clear_distance)
        self.assertEqual(corridor.clear_distance(path, 170.), 1.5)
        checked = inward_recovery_clearance(path, options[0].centre, corridor.clear_distance)
        self.assertIsNotNone(checked)
        self.assertEqual(checked(path, 170.), 170.)
        sample, overrun = corridor_overrun(corridor, path, ALIGN_ESCAPE_M)
        self.assertLessEqual(max(overrun), max(0., overrun[0])+.03)
        self.assertLess(overrun[-1], 0.)
        planner = HighwayPlanner(copy.deepcopy(self.mission), self.world)
        plan = planner.decide(index, route, self.ego, [], {}, corridor.clear_distance,
            lambda path, horizon: horizon, lane_options=options)
        self.assertTrue(plan['status']['low_speed_alignment_recovery'])
        self.assertTrue(plan['status']['alignment_escape_join'])
        self.assertEqual(plan['status']['clearance_components_m']['map'], 170.)
        self.assertGreater(plan['target_mps'], 0.)
        self.assertLessEqual(plan['target_mps'], 4./3.6)

    def test_actual_inference_and_publisher_release_the_recorded_standstill(self):
        for heading_delta in (-.1, 0., .1):
            for lateral_delta in (-.01, 0., .01):
                with self.subTest(heading_delta=heading_delta, lateral_delta=lateral_delta):
                    ego = dict(self.ego, heading_deg=self.ego['heading_deg']+heading_delta)
                    yaw = math.radians(self.ego['heading_deg'])
                    ego['x'] -= lateral_delta*math.sin(yaw)
                    ego['y'] += lateral_delta*math.cos(yaw)
                    runtime, now = self.runtime(), 1_000_000.
                    with patch('time.monotonic', return_value=now), patch('time.time', return_value=now):
                        payload, info = self.infer(runtime, ego, now)
                    self.assertGreater(payload['target_speed_mps'], 0., info)
                    self.assertLessEqual(payload['target_speed_mps'], 4./3.6)
                    self.assertTrue(info['low_speed_alignment_recovery'])
                    self.assertFalse(info['emergency_stop'])
                    self.assertEqual(info['static_clearance_sources']['raw_clear_m'], 170.)
                    self.assertGreater(info['clearance_components_m']['map'], 100.)
                    self.assertIsNone(getattr(runtime.path_publisher, 'reverse_state', None))

    def test_raw_front_collision_and_operator_no_go_still_stop(self):
        for blocked in ('raw', 'zone'):
            with self.subTest(blocked=blocked):
                runtime, now = self.runtime(), 1_000_000.
                points = [[3.9, 0.], [4., .1]] if blocked == 'raw' else []
                if blocked == 'zone':
                    polygon = world_points(np.array([[-10., -10.], [15., -10.],
                                                      [15., 10.], [-10., 10.]]), self.ego)
                    runtime.lanes.rules_cache.value['zones'] = [{'polygon': polygon.tolist()}]
                with patch('time.monotonic', return_value=now), patch('time.time', return_value=now):
                    payload, _ = self.infer(runtime, self.ego, now, points)
                self.assertEqual(payload['target_speed_mps'], 0.)
                self.assertIsNone(getattr(runtime.path_publisher, 'reverse_state', None))

    def test_published_forward_paths_leave_the_edge_and_keep_moving_for_30m(self):
        runtime, ego = self.runtime(), dict(self.ego)
        now, speed, travelled, recovery_frames = 1_000_000., 0., 0., 0
        with patch('time.monotonic', side_effect=lambda: now), \
                patch('time.time', side_effect=lambda: now):
            for frame in range(300):
                ego['speed_kmh'] = speed*3.6
                payload, info = self.infer(runtime, ego, now)
                target = float(payload['target_speed_mps'])
                self.assertGreater(target, 0., (frame, ego, info))
                self.assertIsNone(getattr(runtime.path_publisher, 'reverse_state', None))
                self.assertTrue(all(p.get('gear') == 'D' for p in payload['waypoints']))
                recovery_frames += info['alignment_escape_join']
                path_world = np.array([[p['x'], p['y']] for p in payload['waypoints']])
                path = local_points(path_world, ego)
                self.assertTrue(np.isfinite(path).all())
                self.assertTrue(np.all(np.diff(path[:, 0]) > -.05))
                self.assertLessEqual(max(gui.path_curvatures(path)), gui.MAX_CONTROL_CURVATURE+1e-5)
                # This is a synthetic path follower, not the simulator's MPC.
                look = max(3., .45*speed+2.)
                ahead = np.flatnonzero((path[:, 0] > 0.) &
                                      (np.linalg.norm(path, axis=1) >= look))
                self.assertGreater(len(ahead), 0)
                q = path[ahead[0]]
                curvature = float(2.*q[1]/(q@q))
                speed = max(0., speed+float(np.clip((target-speed)/.25, -3.5, 1.5))*.1)
                yaw = math.radians(ego['heading_deg'])
                ego['x'] += speed*math.cos(yaw)*.1
                ego['y'] += speed*math.sin(yaw)*.1
                ego['heading_deg'] += math.degrees(speed*curvature*.1)
                travelled += speed*.1
                now += .1
                if travelled > 30.:
                    break
        self.assertGreater(travelled, 30.)
        self.assertGreater(recovery_frames, 2)
        self.assertFalse(info['alignment_escape_join'])
        self.assertGreater(ego['speed_kmh'], 5.)


if __name__ == '__main__':
    unittest.main()
