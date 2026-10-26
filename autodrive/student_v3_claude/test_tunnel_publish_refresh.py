"""A slow search must revalidate fresh sensors before publishing its geometry."""
from dataclasses import replace
import json
import time
import unittest
from unittest.mock import patch

import test_live_path_safety as live_fixture
import test_tunnel_integration as tunnel_fixture


class TunnelPublishRefreshTests(unittest.TestCase):
    def runtime(self):
        runtime = tunnel_fixture.TunnelIntegrationTests().runtime()
        live_fixture.LivePathSafetyTests().attach(runtime)
        runtime.infer()
        return runtime

    def test_delayed_tunnel_geometry_gets_new_sensor_proof_before_publication(self):
        runtime = self.runtime()
        pub = runtime.path_publisher
        old = runtime._tunnel_decision_plan
        old = replace(old, checked_at=time.monotonic()-.7)
        runtime.test_scan = dict(runtime.test_scan, stamp=runtime.test_scan['stamp']+.2, sequence=2)
        runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
        runtime.object_receiver.observation_stamp = runtime.test_scan['stamp']
        self.assertFalse(old.fresh())
        self.assertTrue(pub.publish(old.control_ego, old.path, old.path,
            dict(mode='TUNNEL', reason='delayed search', validated_static=True), decision_plan=old))
        active = pub._active_decision_plan
        self.assertTrue(active.fresh())
        self.assertGreater(active.sensor_stamp, old.sensor_stamp)
        self.assertFalse(active.emergency, active.stop_reason)
        self.assertGreater(pub.velocity_pub.items[-1].data, 0.)

    def test_delayed_geometry_with_latest_blocker_stays_stopped(self):
        runtime = self.runtime()
        pub = runtime.path_publisher
        old = replace(runtime._tunnel_decision_plan, checked_at=time.monotonic()-.7)
        import numpy as np
        runtime.test_scan = dict(runtime.test_scan, stamp=runtime.test_scan['stamp']+.2, sequence=2,
            xyz=np.vstack((runtime.test_scan['xyz'], [[7., 0., .7], [7., .2, .7]])))
        runtime.test_status['ego']['stamp'] = runtime.test_scan['stamp']
        runtime.test_status['ego']['speed_kmh'] = 30.
        runtime.tunnel.obstacle_points = np.array([[7., 0.], [7., .2]])
        runtime.object_receiver.observation_stamp = runtime.test_scan['stamp']
        pub.publish(old.control_ego, old.path, old.path,
            dict(mode='TUNNEL', reason='delayed search', validated_static=True), decision_plan=old)
        self.assertEqual(pub.velocity_pub.items[-1].data, 0.)
        self.assertTrue(pub._active_decision_plan.emergency)

    def test_expired_geometry_and_changed_path_cannot_use_revalidation_exception(self):
        for kind in ('expired', 'changed'):
            runtime = self.runtime()
            pub = runtime.path_publisher
            old = replace(runtime._tunnel_decision_plan, checked_at=time.monotonic()-.7)
            path = old.path.copy()
            if kind == 'expired': old = replace(old, checked_at=time.monotonic()-2.)
            else: path[:, 1] += 1.
            pub._safety_provider = lambda *args: self.fail('unchecked geometry reached revalidator')
            self.assertFalse(pub.publish(old.control_ego, path, path,
                dict(mode='TUNNEL', reason='invalid', validated_static=True), decision_plan=old))
            self.assertEqual(pub.velocity_pub.items[-1].data, 0.)

    def test_live_clearance_records_shared_final_path_checks(self):
        runtime = self.runtime()
        pub = runtime.path_publisher
        original = runtime._tunnel_decision_plan
        helper = runtime.tunnel_path_checks
        with patch.object(runtime, 'tunnel_path_checks', wraps=helper) as checks:
            active = pub._safety_provider(
                pub._published_plan['world_path'], original.control_ego, 'TUNNEL', original)
        self.assertTrue(checks.called)
        live = active.validation['live_safety']['raw_clear_m']
        self.assertAlmostEqual(live, active.validation['geometry_clearance_m'])

    def test_blind_zone_returns_used_by_live_stop_also_constrain_bypass_search(self):
        import numpy as np
        runtime = tunnel_fixture.TunnelIntegrationTests().runtime()
        measured = tunnel_fixture.TunnelIntegrationTests.cluster(10.)
        runtime._live_tunnel_obstacles = dict(points=measured,
            ego=dict(runtime.test_odometry), stamp=runtime.test_scan['stamp'],
            received=time.monotonic())
        self.assertTrue(runtime.infer())
        self.assertTrue(np.any(np.linalg.norm(runtime._tunnel_view_points, axis=1) < 11.))
        decision = runtime._tunnel_decision_plan
        clearance = runtime.tunnel_path_checks(decision.path, decision.validation_ego,
            measured, None, validation=decision.validation)['obstacle']
        self.assertTrue(decision.emergency or clearance >= 8., clearance)
        self.assertGreater(np.max(np.abs(decision.path[:, 1])), .1)

    def test_old_live_obstacle_snapshot_cannot_block_a_new_tunnel_search(self):
        import numpy as np
        runtime = tunnel_fixture.TunnelIntegrationTests().runtime()
        runtime._live_tunnel_obstacles = dict(points=np.array([[2.5, 0.]]),
            ego=dict(runtime.test_odometry), stamp=runtime.test_scan['stamp']-2.,
            received=time.monotonic()-2.)
        self.assertTrue(runtime.infer())
        self.assertFalse(runtime._tunnel_decision_plan.emergency)
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)
        runtime.leave_tunnel()
        self.assertFalse(hasattr(runtime, '_live_tunnel_obstacles'))


if __name__ == '__main__':
    unittest.main()
