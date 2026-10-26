"""Recorded tunnel Runtime consumes real protocol records with inert transport.

The stub model isolates path-head proposal plumbing; it is explicitly not a
checkpoint accuracy or closed-loop steering evaluation. Cameras and ROS are
never opened, and original bags are never read.
"""
import json
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
import torch

import test_tunnel_recorded_runtime as recorded_fixture
from tunnel_no_go import TunnelNoGoPublisher, TunnelNoGoReceiver


class TunnelTopicRuntimeTests(unittest.TestCase):
    # These cover the camera-model proposal, which only runs when the tunnel
    # uses the geometric planner (the lattice mode skips it).
    def setUp(self):
        patcher = patch("tunnel_runtime.TUNNEL_LATTICE", False)
        patcher.start()
        self.addCleanup(patcher.stop)

    @classmethod
    def setUpClass(cls):
        recorded_fixture.RecordedTunnelRuntimeTests.setUpClass()

    def runtime(self, unsafe=False, malformed=False, model_path=None, loopback=True):
        runtime = recorded_fixture.RecordedTunnelRuntimeTests.make_runtime()
        runtime.model_camera_fresh = lambda: True
        runtime.receiver.snapshot = Mock(return_value=[])
        runtime.model_calls = []

        def model(images, ego, route):
            runtime.model_calls.append(route.detach().cpu().numpy())
            if model_path is not None:
                path = torch.as_tensor(model_path, dtype=torch.float32)[None]
            elif unsafe:
                x = torch.arange(0., 60.01, .5)
                path = torch.stack((x, torch.zeros_like(x)), dim=1)[None]
            else:
                # Copy the existing route-input reference, then exercise the
                # production extension and all downstream feasibility checks.
                path = route[:, :, :2].clone()
            return dict(path=path, action_logits=torch.tensor([[10., -10., -10., -10.]]))

        runtime.model = model
        publisher = TunnelNoGoPublisher(topic=None)
        receiver = TunnelNoGoReceiver(topic=None, clock=lambda: time.monotonic(),
            source_clock=lambda: runtime.test_status['ego']['stamp'])
        actual_update = publisher.update
        runtime.no_go_wire_messages = []

        def update(*args, **kwargs):
            record = actual_update(*args, **kwargs)
            wire = json.dumps(record, separators=(',', ':'), allow_nan=False)
            runtime.no_go_wire_messages.append(wire)
            # Both producer and consumer use the actual wire validation path.
            # A deliberately broken active packet tests integration failure.
            if loopback:
                receiver.ingest('{broken' if malformed and record['active'] else wire)
            # The runtime validates the returned record directly; a broken
            # active record must fail that same validation.
            return {'broken': True} if malformed and record['active'] else record

        publisher.update = update
        runtime.tunnel_no_go_publisher, runtime.tunnel_no_go_receiver = publisher, receiver
        return runtime

    def run_frame(self, runtime, index, box=None):
        # This integration check does not measure performance. A deterministic
        # compute clock prevents unrelated CPU contention from substituting a
        # model-budget fallback for the specific geometry branch under test.
        with patch('tunnel_runtime.time.perf_counter', return_value=0.):
            return recorded_fixture.RecordedTunnelRuntimeTests.run_frame(runtime, index, box)

    def test_recorded_free_model_proposal_is_checked_and_accepted(self):
        for index in (100, 120):
            with self.subTest(index=index):
                runtime = self.runtime()
                record = self.run_frame(runtime, index)
                self.assertTrue(record['tunnel']['no_go']['valid'])
                guidance = record['tunnel']['model_guidance']
                self.assertTrue(guidance['enabled'])
                self.assertTrue(runtime.model_calls)
                self.assertTrue(guidance['accepted'], guidance)
                self.assertEqual(guidance['source'], 'model')
                self.assertGreaterEqual(guidance['no_go_clear_m'], 8.)
                self.assertFalse(record['emergency_stop'], record['reason'])
                self.assertTrue(np.isfinite(runtime.last_result['path']).all())
                wire = json.loads(runtime.no_go_wire_messages[-1])
                self.assertTrue(wire['active'])
                self.assertEqual(len(wire['walls']), 2)
                runtime.object_receiver.snapshot.assert_not_called()

    def test_own_record_is_used_without_ros_loopback(self):
        # 2026-10-03: the receiver only saw the previous frame's loopback
        # copy, older than 0.25 s, and the model path was never adopted.
        runtime = self.runtime(loopback=False)
        record = self.run_frame(runtime, 100)
        self.assertTrue(record['tunnel']['no_go']['valid'], record['tunnel']['no_go'])
        self.assertTrue(record['tunnel']['model_guidance']['accepted'])

    def test_obstacle_through_model_path_is_rejected_and_safe_bypass_kept(self):
        box = np.asarray([[x, y, z] for x in np.arange(22., 23., .15)
                          for y in np.arange(-.5, .5, .15) for z in np.arange(.3, 1.2, .15)])
        runtime = self.runtime(unsafe=True)
        record = self.run_frame(runtime, 120, box)
        self.assertTrue(record['tunnel']['no_go']['valid'])
        guidance = record['tunnel']['model_guidance']
        self.assertTrue(guidance['enabled'])
        self.assertTrue(runtime.model_calls)
        self.assertFalse(guidance['accepted'], guidance)
        self.assertEqual(guidance['reason'], 'model trajectory intersects tunnel no-go area')
        self.assertEqual(guidance['source'], 'lidar_fallback')
        self.assertEqual(record['mode'], 'TUNNEL BYPASS')
        self.assertFalse(record['emergency_stop'], record['reason'])
        self.assertGreater(abs(record['applied_peak_m']), .1)
        self.assertGreater(record['tunnel']['geometry_clear_distance_m'], 20.)

    def test_invalid_topic_prevents_model_adoption_without_hiding_safe_fallback(self):
        runtime = self.runtime(malformed=True)
        record = self.run_frame(runtime, 100)
        self.assertFalse(record['tunnel']['no_go']['valid'])
        guidance = record['tunnel']['model_guidance']
        self.assertTrue(guidance['enabled'])
        self.assertFalse(guidance['accepted'])
        self.assertEqual(guidance['source'], 'lidar_fallback')
        self.assertEqual(runtime.model_calls, [])
        self.assertFalse(record['emergency_stop'], record['reason'])

    def test_invalid_model_path_keeps_finite_fallback_and_json_diagnostics(self):
        invalid_paths = {
            'nan': np.asarray([[0., 0.], [5., float('nan')], [10., 0.]]),
            'one_dimensional': np.asarray([0., 5., 10.]),
            'empty': np.empty((0, 2)),
            'three_dimensional': np.zeros((2, 3, 2)),
            'missing_coordinate': np.zeros((4, 1)),
        }
        for name, invalid_path in invalid_paths.items():
            with self.subTest(name=name):
                runtime = self.runtime(model_path=invalid_path)
                record = self.run_frame(runtime, 100)
                guidance = record['tunnel']['model_guidance']
                self.assertTrue(record['tunnel']['no_go']['valid'])
                self.assertTrue(guidance['enabled'])
                self.assertTrue(runtime.model_calls)
                self.assertFalse(guidance['accepted'])
                self.assertEqual(guidance['source'], 'lidar_fallback')
                self.assertEqual(guidance['reason'], 'model trajectory invalid')
                self.assertFalse(record['emergency_stop'], record['reason'])
                self.assertGreater(record['planner_cap_kmh'], 0.)
                self.assertTrue(np.isfinite(runtime.last_result['path']).all())
                self.assertEqual(runtime.last_result['raw_path'].shape, (0, 2))
                self.assertEqual(runtime.last_result['trajectory']['plan'], 'NO TRAJECTORY')
                self.assertEqual(record['raw_peak_m'], 0.)
                # NaN output must neither break Runtime nor contaminate the
                # web/archival JSON contract while the safe path is published.
                json.dumps(record, allow_nan=False)

    def test_exit_clears_actual_topic_consumer_snapshot(self):
        runtime = self.runtime()
        record = self.run_frame(runtime, 140)
        self.assertTrue(record['tunnel']['no_go']['active'])
        runtime.leave_tunnel()
        wire = json.loads(runtime.no_go_wire_messages[-1])
        self.assertFalse(wire['active'])
        snapshot = runtime.tunnel_no_go_receiver.snapshot(runtime.test_status['ego'])
        self.assertFalse(snapshot.active)
        self.assertFalse(snapshot.valid)
        runtime.tunnel_no_go_receiver.close()
        runtime.tunnel_no_go_publisher.close()


if __name__ == '__main__':
    unittest.main()
