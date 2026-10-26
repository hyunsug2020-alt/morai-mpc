"""The current network may propose paths, but cannot bypass measured constraints."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import torch

from tunnel_model_guidance import checked_model_path, propose, route_features
from tunnel_runtime import swept_clearance
from tunnel_no_go import TunnelNoGoReceiver, make_record


class TunnelModelGuidanceTests(unittest.TestCase):
    def setUp(self):
        self.reference = np.column_stack((np.arange(0., 60.5, .5), np.zeros(121)))
        self.raw = self.reference[:61]
        self.ego = dict(x=0., y=0., heading_deg=0., wheel_angle=0., speed_kmh=20.)

    def constraints(self, points=()):
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        return SimpleNamespace(valid=True, reason="fresh",
            clear_distance=lambda path, horizon: swept_clearance(path, pts, horizon))

    def check(self, raw=None, points=(), current_clear=60.):
        return checked_model_path(self.raw if raw is None else raw, self.reference,
            self.ego, self.constraints(points), lambda path, horizon: min(horizon, current_clear),
            required_m=25., maintain_mps=20./3.6)

    def test_valid_model_path_keeps_speed_after_topic_and_current_scan_checks(self):
        path, info = self.check()
        self.assertTrue(info["accepted"], info)
        self.assertGreater(info["speed_cap_mps"], 20./3.6)
        self.assertGreater(path[-1, 0], 40.)

    def test_topic_obstacle_and_newer_raw_obstacle_each_reject(self):
        path, info = self.check(points=[(12., 0.)])
        self.assertIsNone(path)
        self.assertIn("no-go", info["reason"])
        path, info = self.check(current_clear=12.)
        self.assertIsNone(path)
        self.assertIn("current LiDAR", info["reason"])

    def test_actual_no_go_wire_and_body_geometry_are_checked_before_model_path_acceptance(self):
        ego = dict(self.ego, stamp=100.)
        walls = [[[-10., 5.], [65., 5.]], [[-10., -5.], [65., -5.]]]
        for occupied in ([], [[12., 0.]], [[0., 1.1]]):
            with self.subTest(occupied=occupied):
                receiver = TunnelNoGoReceiver(topic=None, clock=lambda: 50., source_clock=lambda: 100.)
                self.assertTrue(receiver.ingest(make_record(occupied, walls, ego, 100., True)))
                snapshot = receiver.snapshot(ego)
                self.assertTrue(snapshot.valid)
                path, info = checked_model_path(self.raw, self.reference, ego, snapshot,
                    lambda p, h: h, 25., 20./3.6)
                self.assertEqual(path is not None, not bool(occupied), info)
                self.assertEqual(info['accepted'], not bool(occupied))

    def test_short_backward_and_nonfinite_model_paths_are_not_extended_as_valid(self):
        for raw in (self.reference[:7], self.raw[::-1], self.raw*np.nan):
            with self.subTest(raw=raw.shape):
                path, info = self.check(raw=raw)
                self.assertIsNone(path)
                self.assertFalse(info["accepted"])

    def test_model_path_requiring_deceleration_is_rejected(self):
        raw = self.raw.copy()
        raw[:, 1] = 2.*np.sin(raw[:, 0])
        path, info = self.check(raw=raw)
        self.assertIsNone(path)
        self.assertIn("maintain checked speed", info["reason"])

    def test_existing_route_tensor_shape_and_spacing_are_preserved(self):
        features = route_features(self.reference, 20)
        self.assertEqual(features.shape, (20, 4))
        np.testing.assert_allclose(features[:, 0], np.arange(20)*.5)
        np.testing.assert_allclose(features[:, 2], np.arange(20)*.5)

    def test_nonfinite_topic_or_current_clearance_never_authorizes_a_path(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value):
                constraints = SimpleNamespace(valid=True, clear_distance=lambda p, h: value)
                path, info = checked_model_path(self.raw, self.reference, self.ego,
                    constraints, lambda p, h: h, 25., 20./3.6)
                self.assertIsNone(path)
                self.assertFalse(info['accepted'])
                self.assertIn('no-go clearance invalid', info['reason'])
                path, info = checked_model_path(self.raw, self.reference, self.ego,
                    self.constraints(), lambda p, h: value, 25., 20./3.6)
                self.assertIsNone(path)
                self.assertFalse(info['accepted'])

    def test_nonfinite_wheel_angle_or_feasibility_requirement_is_rejected(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            for field in ('wheel_angle', 'required_m', 'maintain_mps'):
                with self.subTest(value=value, field=field):
                    ego = dict(self.ego, wheel_angle=value) if field == 'wheel_angle' else self.ego
                    path, info = checked_model_path(self.raw, self.reference, ego,
                        self.constraints(), lambda p, h: h,
                        value if field == 'required_m' else 25.,
                        value if field == 'maintain_mps' else 20./3.6)
                    self.assertIsNone(path)
                    self.assertFalse(info['accepted'])
                    self.assertIn('input invalid', info['reason'])

    def test_existing_model_output_is_used_and_bad_output_remains_explicit_fallback(self):
        runtime = SimpleNamespace(config=dict(frames=4, route_points=20), device=torch.device("cpu"),
            model_camera_fresh=lambda: True, receiver=SimpleNamespace(snapshot=lambda count: []),
            image_tensor=lambda rows: (torch.zeros(1), []), ego_tensor=lambda ego: torch.zeros(1),
            actions=["KEEP_LANE", "YIELD"])
        def infer(*inputs):
            self.assertEqual(inputs[2].shape, (1, 20, 4))
            return dict(path=torch.tensor(self.raw)[None], action_logits=torch.tensor([[0., 1.]]))
        runtime.model = infer
        path, info = propose(runtime, self.ego, self.reference, self.constraints(),
                             lambda path, horizon: horizon, 25., 20./3.6)
        self.assertIsNotNone(path)
        self.assertEqual(info["source"], "model")
        self.assertEqual(info["action"], "YIELD")
        self.raw = self.raw[:5]
        path, info = propose(runtime, self.ego, self.reference, self.constraints(),
                             lambda path, horizon: horizon, 25., 20./3.6)
        self.assertIsNone(path)
        self.assertEqual(info["source"], "lidar_fallback")
        self.assertIn("insufficient", info["reason"])

    def test_nonfinite_model_probability_remains_a_fallback_with_valid_path_geometry(self):
        runtime = SimpleNamespace(config=dict(frames=4, route_points=20), device=torch.device('cpu'),
            model_camera_fresh=lambda: True, receiver=SimpleNamespace(snapshot=lambda count: []),
            image_tensor=lambda rows: (torch.zeros(1), []), ego_tensor=lambda ego: torch.zeros(1),
            actions=['KEEP_LANE', 'YIELD'], model=lambda *args: dict(
                path=torch.tensor(self.raw)[None], action_logits=torch.tensor([[float('nan'), 1.]])))
        path, info = propose(runtime, self.ego, self.reference, self.constraints(),
                             lambda path, horizon: horizon, 25., 20./3.6)
        self.assertIsNone(path)
        self.assertFalse(info['accepted'])
        self.assertEqual(info['source'], 'lidar_fallback')
        self.assertEqual(info['reason'], 'model action probabilities invalid')

    def test_malformed_model_path_is_not_exposed_as_runtime_display_metadata(self):
        malformed = (np.full((3, 2), np.nan), np.full((3, 2), np.inf),
                     np.arange(3.), np.empty((0, 2)), np.zeros((3, 1)),
                     np.zeros((3, 2, 1)))
        for raw in malformed:
            with self.subTest(shape=raw.shape):
                runtime = SimpleNamespace(config=dict(frames=4, route_points=20),
                    device=torch.device('cpu'), model_camera_fresh=lambda: True,
                    receiver=SimpleNamespace(snapshot=lambda count: []),
                    image_tensor=lambda rows: (torch.zeros(1), []),
                    ego_tensor=lambda ego: torch.zeros(1), actions=['KEEP_LANE'],
                    model=lambda *args: dict(path=torch.tensor(raw)[None],
                                              action_logits=torch.tensor([[1.]])))
                path, info = propose(runtime, self.ego, self.reference, self.constraints(),
                                     lambda p, h: h, 25., 20./3.6)
                self.assertIsNone(path)
                self.assertFalse(info['accepted'])
                self.assertEqual(info['source'], 'lidar_fallback')
                self.assertNotIn('_raw_path', info)

    def test_model_inference_or_candidate_check_cannot_overrun_its_admission_budget(self):
        for slow_stage in ('model', 'check'):
            with self.subTest(stage=slow_stage), patch('tunnel_model_guidance.time.perf_counter', return_value=0.) as clock:
                runtime = SimpleNamespace(config=dict(frames=4, route_points=20), device=torch.device('cpu'),
                    model_camera_fresh=lambda: True, receiver=SimpleNamespace(snapshot=lambda count: []),
                    image_tensor=lambda rows: (torch.zeros(1), []), ego_tensor=lambda ego: torch.zeros(1),
                    actions=['KEEP_LANE', 'YIELD'])
                def infer(*args):
                    if slow_stage == 'model':
                        clock.return_value = 2.
                    return dict(path=torch.tensor(self.raw)[None], action_logits=torch.tensor([[1., 0.]]))
                def clear(path, horizon):
                    if slow_stage == 'check':
                        clock.return_value = 2.
                    return horizon
                runtime.model = infer
                path, info = propose(runtime, self.ego, self.reference, self.constraints(),
                                     clear, 25., 20./3.6, deadline=1.)
                self.assertIsNone(path)
                self.assertFalse(info['accepted'])
                self.assertEqual(info['source'], 'lidar_fallback')
                self.assertEqual(info['reason'], 'model proposal budget exhausted')

    def test_stale_camera_invalid_topic_or_missing_history_does_not_run_the_model(self):
        for stage in ('camera', 'topic', 'history'):
            with self.subTest(stage=stage):
                runtime = SimpleNamespace(model=Mock(), config=dict(frames=4),
                    model_camera_fresh=lambda: stage != 'camera',
                    receiver=SimpleNamespace(snapshot=lambda count: None if stage == 'history' else []))
                constraints = self.constraints()
                if stage == 'topic':
                    constraints.valid, constraints.reason = False, 'topic stale'
                path, info = propose(runtime, self.ego, self.reference, constraints,
                                     lambda p, h: h, 25., 20./3.6)
                self.assertIsNone(path)
                self.assertFalse(info['accepted'])
                self.assertEqual(info['source'], 'lidar_fallback')
                runtime.model.assert_not_called()

    def test_optional_camera_decode_or_model_schema_failure_keeps_the_lidar_fallback(self):
        for error in (OSError('invalid JPEG'), IndexError('missing camera'), KeyError('path')):
            with self.subTest(error=type(error).__name__):
                runtime = SimpleNamespace(config=dict(frames=4, route_points=20), device=torch.device('cpu'),
                    model_camera_fresh=lambda: True, receiver=SimpleNamespace(snapshot=lambda count: []),
                    image_tensor=Mock(side_effect=error), model=Mock(), actions=['KEEP_LANE'])
                path, info = propose(runtime, self.ego, self.reference, self.constraints(),
                                     lambda p, h: h, 25., 20./3.6)
                self.assertIsNone(path)
                self.assertFalse(info['accepted'])
                self.assertEqual(info['source'], 'lidar_fallback')
                self.assertIn('model proposal failed:', info['reason'])


if __name__ == "__main__":
    unittest.main()
