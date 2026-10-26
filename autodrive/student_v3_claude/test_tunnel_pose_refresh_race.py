"""Source-time odometry refresh races and fail-closed recovery contracts."""
from collections import deque
import threading
import time
import unittest

import numpy as np

import test_tunnel_integration as fixture
from tunnel_runtime import TunnelOdometryReceiver


class TunnelPoseRefreshRaceTests(unittest.TestCase):
    def runtime(self, fault=None):
        runtime = fixture.TunnelIntegrationTests().runtime()
        ego = dict(runtime.test_status['ego'], speed_kmh=0.)
        runtime.test_status['ego'] = ego
        original = dict(runtime.test_scan)
        fresh = dict(ego, stamp=ego['stamp']+.4, x=.3, age_s=.01)
        latest = dict(original, stamp=fresh['stamp'], sequence=2, age_s=.01)
        receiver = TunnelOdometryReceiver.__new__(TunnelOdometryReceiver)
        receiver.lock = threading.Lock()
        receiver.samples = deque([dict(ego, received=time.monotonic()-.4)], maxlen=800)

        def update():
            received = time.monotonic()-(.26 if fault == 'stale_odometry' else .01)
            stamp = fresh['stamp']-(.13 if fault == 'missing_source_pose' else 0.)
            receiver.samples.append(dict(fresh, stamp=stamp, received=received))
            return dict(ego=dict(fresh, age_s=.26 if fault == 'stale_ego' else .01))

        if fault == 'stale_scan':
            latest['age_s'] = .61
        elif fault == 'clock_reset':
            latest['stamp'] = original['stamp']-.1
        elif fault == 'missing_scan':
            latest['xyz'] = None
        runtime.update_ego = update
        runtime.tunnel_odometry = receiver
        runtime.lidar.scan_snapshot = lambda: latest
        return runtime, ego, original, latest

    def check(self, runtime, ego, original):
        return runtime.checked_tunnel_plan(runtime.test_route, ego, ego, original,
            np.empty((0, 2)), 4., 40., False,
            dict(valid=True, pose_valid=True, wall=60., lane=60., obstacle=60.))[0]

    def test_old_lookup_failure_is_replaced_by_the_new_fresh_history(self):
        runtime, ego, original, latest = self.runtime()
        self.assertIsNone(runtime.tunnel_odometry.at_stamp(ego['stamp']))
        decision = self.check(runtime, ego, original)
        self.assertTrue(decision.validation['pose_refreshed'])
        self.assertTrue(decision.validation['valid'])
        self.assertTrue(decision.fresh())
        self.assertTrue(decision.checks_match_path())
        self.assertFalse(decision.emergency)
        self.assertGreater(decision.allowed_mps, 0.)
        self.assertEqual(decision.control_ego['stamp'], latest['stamp'])

    def test_actual_stale_or_missing_sources_keep_zero(self):
        faults = ('stale_scan', 'stale_odometry', 'stale_ego', 'missing_source_pose',
                  'clock_reset', 'missing_scan')
        for fault in faults:
            with self.subTest(fault=fault):
                runtime, ego, original, _ = self.runtime(fault)
                decision = self.check(runtime, ego, original)
                self.assertTrue(decision.emergency)
                self.assertEqual(decision.allowed_mps, 0.)
                self.assertFalse(decision.validation['valid'])

    def test_next_fresh_observation_recovers_after_rejection(self):
        runtime, ego, original, latest = self.runtime('stale_odometry')
        self.assertEqual(self.check(runtime, ego, original).allowed_mps, 0.)
        runtime.update_ego = lambda: dict(ego=dict(ego, stamp=latest['stamp'], age_s=.01, x=.3))
        runtime.tunnel_odometry.samples.append(dict(ego, stamp=latest['stamp'], x=.3,
                                                   received=time.monotonic()))
        decision = self.check(runtime, ego, original)
        self.assertTrue(decision.validation['valid'])
        self.assertFalse(decision.emergency)
        self.assertGreater(decision.allowed_mps, 0.)

    def test_receiver_rejects_future_receipt_and_unbounded_prediction(self):
        runtime, ego, _, _ = self.runtime()
        receiver = runtime.tunnel_odometry
        receiver.samples[-1]['received'] = time.monotonic()+.01
        self.assertIsNone(receiver.at_stamp(ego['stamp']))
        receiver.samples[-1]['received'] = time.monotonic()
        self.assertIsNone(receiver.at_stamp(ego['stamp']+.13))
        self.assertIsNotNone(receiver.at_stamp(ego['stamp']+.11))

    def test_invalid_scan_receipt_age_cannot_authorize_refresh(self):
        for age in (-.01, float('nan')):
            with self.subTest(age=age):
                runtime, ego, original, latest = self.runtime()
                latest['age_s'] = age
                decision = self.check(runtime, ego, original)
                self.assertEqual(decision.allowed_mps, 0.)
                self.assertTrue(decision.emergency)
                self.assertFalse(decision.validation['valid'])


if __name__ == '__main__':
    unittest.main()
