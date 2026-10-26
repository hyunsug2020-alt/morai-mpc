"""A transient sensor rejection may recover only by rechecking issued geometry."""
from dataclasses import replace
import time
import unittest
from unittest.mock import patch

import numpy as np

from test_live_path_safety import LivePathSafetyTests
from test_tunnel_integration import TunnelIntegrationTests


class TunnelSensorRecoveryTests(unittest.TestCase):
    def runtime(self):
        helper = LivePathSafetyTests()
        runtime = helper.attach(TunnelIntegrationTests().runtime())
        runtime.infer()
        self.assertGreater(helper.payload(runtime)['target_speed_mps'], 0.)
        return helper, runtime

    def test_fresh_sensor_rechecks_resume_after_transient_incoming_rejection(self):
        helper, runtime = self.runtime()
        publisher = runtime.path_publisher
        baseline = publisher._safety_baseline
        old = baseline['decision']
        invalid = replace(old, checked_at=time.monotonic()-1.6)
        self.assertFalse(publisher.publish(invalid.control_ego, invalid.path, invalid.path,
            dict(mode='TUNNEL', reason='old search'), decision_plan=invalid))
        self.assertTrue(publisher._planner_failed)
        self.assertTrue(publisher._planner_sensor_recheck)
        helper.advance(runtime, .1, [])
        with patch('time.monotonic', return_value=time.monotonic()+.1):
            publisher.recheck_active_path()
        self.assertFalse(publisher._planner_failed)
        self.assertGreater(helper.payload(runtime)['target_speed_mps'], 0.)

    def test_invalid_new_geometry_cannot_enable_sensor_retry(self):
        helper, runtime = self.runtime()
        publisher = runtime.path_publisher
        plan = publisher._safety_baseline['decision']
        changed = plan.path.copy()
        changed[3, 1] += .1
        self.assertFalse(publisher.publish(plan.control_ego, changed, changed,
            dict(mode='TUNNEL', reason='geometry mismatch'), decision_plan=plan))
        self.assertFalse(publisher._planner_sensor_recheck)
        helper.advance(runtime, .1, [])
        self.assertFalse(publisher.recheck_active_path())
        self.assertEqual(helper.payload(runtime)['target_speed_mps'], 0.)

    def test_stale_unsafe_plan_cannot_resume_previous_safe_path(self):
        cases = (
            dict(validation=dict(valid=False, pose_valid=True, steering_valid=False),
                 emergency=True, allowed_mps=0., stop_reason='tunnel final path steering infeasible'),
            dict(validation=dict(valid=False, pose_valid=False, steering_valid=True),
                 emergency=True, allowed_mps=0., stop_reason='tunnel pose invalid'),
            dict(validation=dict(valid=False, pose_valid=True, steering_valid=True),
                 stop_reason='tunnel geometry invalid'),
            dict(validation=dict(valid=True, pose_valid=True, steering_valid=True, blocked=True)),
            dict(emergency=True, allowed_mps=0., stop_reason='tunnel obstacle on path'),
            dict(allowed_mps=0.),
            dict(allowed_mps=float('nan')),
            dict(validation=dict(valid=False, pose_valid=False, steering_valid=True),
                 emergency=True, allowed_mps=0., stop_reason='tunnel plan sensor stale'),
        )
        for changes in cases:
            with self.subTest(changes=changes):
                helper, runtime = self.runtime()
                publisher = runtime.path_publisher
                old = publisher._safety_baseline['decision']
                changes = dict(changes)
                validation = dict(old.validation, **changes.pop('validation', {}))
                invalid = replace(old, checked_at=time.monotonic()-1.6,
                                  validation=validation, **changes)
                self.assertFalse(publisher.publish(invalid.control_ego, invalid.path, invalid.path,
                    dict(mode='TUNNEL', reason='unsafe delayed search'), decision_plan=invalid))
                self.assertFalse(publisher._planner_sensor_recheck)
                helper.advance(runtime, .1, [])
                self.assertFalse(publisher.recheck_active_path())
                self.assertEqual(helper.payload(runtime)['target_speed_mps'], 0.)

    def test_sensor_only_stale_stop_wrapper_can_recheck_safe_geometry(self):
        helper, runtime = self.runtime()
        publisher = runtime.path_publisher
        old = publisher._safety_baseline['decision']
        invalid = replace(old, checked_at=time.monotonic()-1.6, allowed_mps=0.,
            emergency=True, stop_reason='tunnel plan sensor stale',
            validation=dict(old.validation, valid=False))
        self.assertFalse(publisher.publish(invalid.control_ego, invalid.path, invalid.path,
            dict(mode='TUNNEL', reason='sensor-only expired search'), decision_plan=invalid))
        self.assertTrue(publisher._planner_sensor_recheck)
        helper.advance(runtime, .1, [])
        with patch('time.monotonic', return_value=time.monotonic()+.1):
            self.assertTrue(publisher.recheck_active_path())
        self.assertGreater(helper.payload(runtime)['target_speed_mps'], 0.)

    def test_hazard_stop_cannot_be_reclassified_by_next_stale_safe_plan(self):
        helper, runtime = self.runtime()
        publisher = runtime.path_publisher
        old = publisher._safety_baseline['decision']
        publisher.publish_stop('tunnel final path steering infeasible')
        stale = replace(old, checked_at=time.monotonic()-1.6)
        self.assertFalse(publisher.publish(stale.control_ego, stale.path, stale.path,
            dict(mode='TUNNEL', reason='following delayed search'), decision_plan=stale))
        self.assertFalse(publisher._planner_sensor_recheck)
        helper.advance(runtime, .1, [])
        self.assertFalse(publisher.recheck_active_path())
        self.assertEqual(helper.payload(runtime)['target_speed_mps'], 0.)
        fresh = replace(old, checked_at=time.monotonic())
        self.assertTrue(publisher.publish(fresh.control_ego, fresh.path, fresh.path,
            dict(mode='TUNNEL', reason='fresh checked replacement'), decision_plan=fresh))
        self.assertFalse(publisher._planner_failed)
        self.assertGreater(helper.payload(runtime)['target_speed_mps'], 0.)

    def test_real_obstacle_and_stale_observation_keep_retry_stopped(self):
        helper, runtime = self.runtime()
        publisher = runtime.path_publisher
        publisher.publish_stop('sensor rejection', sensor_recheck=True)
        runtime.test_scan = dict(runtime.test_scan, age_s=.7)
        publisher.recheck_active_path()
        self.assertEqual(helper.payload(runtime)['target_speed_mps'], 0.)
        self.assertTrue(publisher._planner_failed)
        runtime.test_scan = dict(runtime.test_scan, age_s=.01)
        runtime.tunnel.path_clearance = lambda *args, **kwargs: .5
        helper.advance(runtime, .1, [])
        publisher.recheck_active_path()
        self.assertEqual(helper.payload(runtime)['target_speed_mps'], 0.)
        self.assertTrue(publisher._planner_failed)

    def test_recovery_does_not_extend_geometry_lifetime(self):
        helper, runtime = self.runtime()
        publisher = runtime.path_publisher
        publisher.publish_stop('sensor rejection', sensor_recheck=True)
        created = publisher._safety_baseline['created']
        with patch('time.monotonic', return_value=created+2.1):
            self.assertFalse(publisher.recheck_active_path())
        self.assertEqual(helper.payload(runtime)['target_speed_mps'], 0.)


if __name__ == '__main__':
    unittest.main()
