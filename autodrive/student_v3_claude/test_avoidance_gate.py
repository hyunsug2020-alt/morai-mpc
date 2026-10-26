"""Exercise normal-road inference and published control paths without a simulator."""
import json
import threading
from collections import deque
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

import numpy as np
import torch

from gui import Runtime, RosPathPublisher, RosEgoReceiver
from highway_planner import NearFieldMemory
from lane_corridor import LaneCorridor
from lidar_corridor import LidarCorridor
from traffic_planner import GapPlanner


class Message:
    def __init__(self, data=None):
        self.data = data


class Sink:
    def __init__(self):
        self.items = []

    def publish(self, value):
        self.items.append(value)


class Pose:
    def __init__(self):
        self.pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0.))


class PathMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id=None)
        self.poses = []


class AvoidanceGateTests(unittest.TestCase):
    def runtime(self, action="YIELD", force=True, objects=(), points=()):
        runtime = Runtime.__new__(Runtime)
        route = np.column_stack((np.arange(0., 65.5, .5), np.zeros(131)))
        runtime.test_route = route
        stamp = time.time()
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=27., age_s=.01,
                   stamp=stamp, measured=True)
        runtime.test_status = dict(ego=ego, path=dict(nearest_idx=0, distance_m=0.))
        runtime.update_ego = lambda: runtime.test_status
        runtime.receiver = SimpleNamespace(snapshot=lambda count: [], rates=lambda: {})
        runtime.config = dict(frames=1)
        runtime.input_error, runtime.highway, runtime.stop_line_controller = None, None, None
        runtime.image_tensor = lambda rows: (torch.zeros(1), None)
        runtime.ego_tensor = lambda current: torch.zeros(1)
        runtime.route_tensor = lambda current, status: (torch.zeros(1), route[:20], route[:40], route)
        runtime.device = torch.device("cpu")
        runtime.latencies = deque(maxlen=100)
        runtime.actions = ["KEEP", "YIELD", "LEFT_CHANGE", "RIGHT_CHANGE"]
        logits = torch.full((1, 4), -10.)
        logits[0, runtime.actions.index(action)] = 10.
        x = np.linspace(0., 20., 41)
        prediction = np.column_stack((x, -2.2 * np.minimum(x / 10., 1.)))
        runtime.model = lambda *args: dict(action_logits=logits, concept_logits=torch.zeros(1, 1),
                                          scene=torch.zeros(1, 1), path=torch.tensor(prediction)[None])
        runtime.args = SimpleNamespace(force_student_avoidance=force, max_route_distance_m=100.)
        runtime.clearance = "both"
        runtime.object_receiver = SimpleNamespace(objects=list(objects), age_s=.01,
                                                  received=time.monotonic(), src="test")
        runtime.object_receiver.snapshot = lambda: runtime.object_receiver.objects
        runtime.lidar = LidarCorridor()
        runtime.lidar.pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        runtime.lidar.received = time.monotonic()
        runtime.test_scan = dict(stamp=stamp, sequence=1, age_s=.01,
                                 xyz=np.empty((0, 3)), obstacle_points=runtime.lidar.pts)
        runtime.ego_receiver = SimpleNamespace(lock=threading.RLock(), samples=deque([dict(ego)]))
        runtime.ego_receiver.snapshot = lambda count: (dict(runtime.test_status['ego']), [], 20.)
        runtime.ego_receiver.measured_xy_at_stamp = lambda value: RosEgoReceiver.measured_xy_at_stamp(
            runtime.ego_receiver, value)
        runtime.lidar.scan_snapshot = lambda: dict(runtime.test_scan,
            obstacle_points=runtime.lidar.pts.copy(),
            age_s=max(0., time.monotonic()-runtime.lidar.received))
        runtime.near_field_memory = NearFieldMemory()
        runtime.lanes = SimpleNamespace(
            corridor=lambda ego, path, **kwargs: LaneCorridor(path, np.full(len(path), 5.), np.full(len(path), -5.)),
            adjacent_offsets=lambda ego, path: {-1: -3.5, 1: 3.5})
        runtime.speed_zones = SimpleNamespace(kmh_at=lambda index: (28.94, {}))
        runtime.gap_planner = GapPlanner()
        runtime.gallery_matrix, runtime.gallery = np.empty((0, 1)), []
        runtime.avoidance_direction, runtime.avoidance_latch_until = 0., 0.
        runtime._decision_log = SimpleNamespace(write=lambda line: None, dropped=0)
        publisher = RosPathPublisher.__new__(RosPathPublisher)
        publisher.Float32 = publisher.String = Message
        publisher.PoseStamped, publisher.PathMessage = Pose, PathMessage
        publisher.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: 1.))
        for name in ("waypoints_pub", "velocity_pub", "mode_pub", "path_pub", "status_pub"):
            setattr(publisher, name, Sink())
        publisher.cruise_mps = 28.94 / 3.6
        publisher.publish_corridor = lambda *args: None
        publisher.publish_camera_rates = lambda *args: None
        runtime.path_publisher = publisher
        return runtime

    def assert_route_only(self, runtime):
        self.assertTrue(runtime.infer())
        self.assertEqual(runtime._web_record["mode"], "ROUTE BASE")
        self.assertEqual(runtime._web_record["applied_peak_m"], 0.)
        payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        self.assertTrue(all(abs(point["y"]) < 1e-6 for point in payload["waypoints"]))
        self.assertEqual(runtime.path_publisher.mode_pub.items[-1].data, "CRUISE")

    def test_empty_road_ignores_high_confidence_model_action_with_both_options(self):
        for force in (False, True):
            for action in ("YIELD", "LEFT_CHANGE", "RIGHT_CHANGE"):
                with self.subTest(force=force, action=action):
                    self.assert_route_only(self.runtime(action, force))

    def test_side_object_and_old_model_direction_cannot_start_bypass(self):
        side = (12., 4., .8, "side_structure", "obstacle", 0., .8, .8, 0.)
        runtime = self.runtime(objects=[side])
        runtime.avoidance_direction, runtime.avoidance_latch_until = -1., time.monotonic() + 100.
        self.assert_route_only(runtime)
        self.assertIsNone(runtime._web_record["hazard"])

    def test_real_object_starts_checked_bypass_even_if_model_says_keep(self):
        box = (18., 0., 1., "box", "obstacle", 0., 1., 1., 0.)
        runtime = self.runtime(action="KEEP", objects=[box])
        self.assertTrue(runtime.infer())
        self.assertEqual(runtime._web_record["mode"], "STUDENT FORCED")
        self.assertTrue(runtime.last_result["postprocess"]["validated_static"])
        self.assertIsNotNone(runtime._static_plan_saved)
        self.assertEqual(runtime.path_publisher.mode_pub.items[-1].data, "AVOID")

    def test_stale_object_cannot_start_bypass_and_retains_stream_stop(self):
        box = (18., 0., 1., "box", "obstacle", 0., 1., 1., 0.)
        runtime = self.runtime(objects=[box])
        runtime.object_receiver.age_s = .6
        self.assert_route_only(runtime)
        self.assertTrue(runtime.path_publisher.emergency_stop)
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_moving_pedestrian_requires_yield_without_starting_bypass(self):
        pedestrian = (6., 0., .6, "person", "pedestrian", 1., .6, .6, 90., 0., 1.)
        runtime = self.runtime(objects=[pedestrian])
        self.assert_route_only(runtime)
        self.assertTrue(runtime.path_publisher.emergency_stop)

    def test_checked_static_bypass_can_rejoin_after_object_disappears(self):
        box = (18., 0., 1., "box", "obstacle", 0., 1., 1., 0.)
        runtime = self.runtime(action="KEEP", objects=[box])
        self.assertTrue(runtime.infer())
        runtime.object_receiver.objects = []
        self.assertTrue(runtime.infer())
        self.assertEqual(runtime._web_record["mode"], "STUDENT REJOIN")
        self.assertTrue(runtime.last_result["postprocess"]["validated_static"])

    def test_lidar_only_permission_expires_without_model_refresh(self):
        runtime = self.runtime(action="YIELD", points=[(18., 0.), (18., .1), (18., -.1)])
        self.assertTrue(runtime.infer())
        self.assertEqual(runtime._web_record["mode"], "STUDENT FORCED")
        latch = dict(runtime._sensor_bypass_latch)
        runtime.lidar.pts = np.zeros((0, 2), np.float32)
        runtime.near_field_memory = NearFieldMemory()
        self.assertTrue(runtime.infer())
        self.assertEqual(runtime._web_record["mode"], "STUDENT FORCED")
        self.assertEqual(runtime._sensor_bypass_latch, latch)
        later = latch["until"] + .1
        with patch("gui.time.monotonic", return_value=later):
            runtime.lidar.received = later
            self.assert_route_only(runtime)
        self.assertIsNone(runtime._sensor_bypass_latch)


if __name__ == "__main__":
    unittest.main()
