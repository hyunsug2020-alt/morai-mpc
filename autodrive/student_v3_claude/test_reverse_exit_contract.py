"""The real path publisher must preserve forward proof across every R exit."""
import json
import time
import unittest
from contextlib import nullcontext
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

import gui
from driving_plan import DrivingPlan


class StaticReverseExitContract(unittest.TestCase):
    def runtime(self):
        import test_tunnel_plan_contract as fixtures
        runtime = fixtures.TunnelPlanContractTests().runtime()
        runtime.test_status['ego'].update(stamp=100., speed_kmh=0.)
        runtime.test_scan['stamp'] = 100.
        runtime.path_publisher.start_reverse(runtime.test_status['ego'], static_recovery=True)
        return runtime

    def plan(self, runtime, stamp=100., *, complete=True):
        runtime.test_status['ego']['stamp'] = stamp
        ego = dict(runtime.test_status['ego'])
        return DrivingPlan(1, runtime.test_route, ego, ego, stamp, .01, time.monotonic(),
            2., 60. if complete else 4., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False,
                 static_rear_axle=True, static_complete=complete), mode='STUDENT FORCED')

    def publish(self, runtime, plan):
        publisher = runtime.path_publisher
        ego = dict(runtime.test_status['ego'])
        path = runtime.test_route if plan is None else plan.path
        publisher.publish(ego, path, runtime.test_route,
            dict(mode='STUDENT FORCED', reason='checked bypass', validated_static=True,
                 forward_clear_m=60.), decision_plan=plan)
        return json.loads(publisher.waypoints_pub.items[-1].data)

    def expire(self, runtime, reason, *, runtime_first=False):
        publisher, ego = runtime.path_publisher, runtime.test_status['ego']
        if reason == 'timeout':
            publisher.reverse_state['until'] = time.monotonic()-1.
        elif reason == 'distance':
            ego['x'] = -6.1
        elif reason == 'abort':
            publisher.reverse_abort = True
        else:
            self.fail(reason)
        if runtime_first:
            gui.Runtime.reverse_escape_check(runtime, ego, False, 60., np.empty((0, 2)),
                                             time.monotonic())

    def assert_stopped_in_r(self, payload):
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertEqual({point['gear'] for point in payload['waypoints']}, {'R'})

    def test_every_exit_keeps_partial_forward_plan_stopped(self):
        for reason in ('timeout', 'distance', 'abort'):
            for runtime_first in (False, True):
                with self.subTest(reason=reason, runtime_first=runtime_first):
                    runtime = self.runtime()
                    self.expire(runtime, reason, runtime_first=runtime_first)
                    payload = self.publish(runtime, self.plan(runtime, complete=False))
                    self.assert_stopped_in_r(payload)
                    self.assertIsNone(runtime.path_publisher.reverse_state)
                    self.assertTrue(runtime.path_publisher._reverse_exit_state['static_recovery'])
                    for stamp in (100.1, 100.2):
                        self.assert_stopped_in_r(self.publish(runtime,
                            self.plan(runtime, stamp, complete=False)))

    def test_every_exit_releases_on_three_fresh_complete_scans(self):
        for reason in ('timeout', 'distance', 'abort'):
            with self.subTest(reason=reason):
                runtime = self.runtime()
                self.expire(runtime, reason, runtime_first=True)
                for stamp in (100., 100.1):
                    self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, stamp)))
                payload = self.publish(runtime, self.plan(runtime, 100.2))
                self.assertEqual({point['gear'] for point in payload['waypoints']}, {'D'})
                self.assertGreater(payload['target_speed_mps'], 0.)
                self.assertIsNone(runtime.path_publisher._reverse_exit_state)

    def test_missing_nonstatic_or_stale_plan_cannot_remove_static_release_policy(self):
        for fault in ('missing', 'nonstatic', 'stale', 'blocked', 'mismatched_geometry'):
            with self.subTest(fault=fault):
                runtime = self.runtime()
                self.expire(runtime, 'timeout')
                plan = self.plan(runtime)
                if fault == 'missing':
                    plan = None
                elif fault == 'nonstatic':
                    plan = plan.decided(static_rear_axle=False)
                elif fault == 'stale':
                    plan = replace(plan, checked_at=time.monotonic()-1.)
                elif fault == 'blocked':
                    plan = plan.decided(blocked=True)
                else:
                    plan = plan.decided(geometry_path_digest='a different path')
                for _ in range(4):
                    self.assert_stopped_in_r(self.publish(runtime, plan))
                self.assertTrue(runtime.path_publisher._reverse_exit_state['static_recovery'])

    def test_repeated_old_scan_and_rewind_cannot_count_as_three_proofs(self):
        runtime = self.runtime()
        self.expire(runtime, 'timeout')
        for _ in range(5):
            self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, 100.)))
        self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, 100.1)))
        self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, 99.9)))
        self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, 100.)))
        payload = self.publish(runtime, self.plan(runtime, 100.1))
        self.assertGreater(payload['target_speed_mps'], 0.)

    def test_reverse_must_stop_and_current_proof_must_survive_until_selecting_d(self):
        runtime = self.runtime()
        self.expire(runtime, 'timeout')
        runtime.test_status['ego']['speed_kmh'] = -1.5
        for stamp in (100., 100.1, 100.2):
            self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, stamp)))
        runtime.test_status['ego']['speed_kmh'] = 0.
        self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, 100.3, complete=False)))
        for stamp in (100.4, 100.5):
            self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, stamp)))
        self.assertGreater(self.publish(runtime, self.plan(runtime, 100.6))['target_speed_mps'], 0.)

    def test_new_reverse_cannot_downgrade_an_unreleased_static_episode(self):
        runtime = self.runtime()
        self.expire(runtime, 'timeout', runtime_first=True)
        runtime.path_publisher.start_reverse(runtime.test_status['ego'], static_recovery=False)
        self.assertTrue(runtime.path_publisher.reverse_state['static_recovery'])

    def test_live_safety_cannot_reissue_a_previous_forward_baseline(self):
        import test_live_path_safety as fixtures
        fixture = fixtures.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        runtime.infer()
        publisher = runtime.path_publisher
        baseline = publisher._safety_baseline
        self.assertIsNotNone(baseline)
        publisher.start_reverse(runtime.test_status['ego'], static_recovery=True)
        self.assertIsNone(publisher._safety_baseline)
        # Even a baseline captured by a caller before start_reverse is not an
        # authority to restart D during or after the escape.
        publisher._safety_baseline = baseline
        published = len(publisher.waypoints_pub.items)
        self.assertFalse(publisher.recheck_active_path())
        publisher.reverse_state['until'] = time.monotonic()-1.
        publisher.reverse_active(runtime.test_status['ego'])
        self.assertFalse(publisher.recheck_active_path())
        self.assertEqual(len(publisher.waypoints_pub.items), published)

    def test_approaching_person_retains_its_rear_checked_protective_backoff(self):
        starts = []
        publisher = SimpleNamespace(start_reverse=lambda *args, **kwargs: starts.append(kwargs),
                                    reverse_active=lambda ego: False)
        runtime = SimpleNamespace(path_publisher=publisher, _contact=None)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        self.assertTrue(gui.Runtime.pedestrian_reverse(runtime, ego, 1.5, np.empty((0, 2)), [],
                                                     time.monotonic()))
        self.assertEqual(starts[0]['distance_m'], 1.5)
        self.assertIs(starts[0]['static_recovery'], False)
        starts.clear()
        runtime._ped_reverse_tries = {}
        self.assertFalse(gui.Runtime.pedestrian_reverse(runtime, ego, 1.5, np.array([[-1.5, 0.]]), [],
                                                      time.monotonic()))
        self.assertEqual(starts, [])

    def test_static_planning_still_aborts_an_active_protective_backoff_on_new_rear_return(self):
        import test_tunnel_plan_contract as fixtures
        runtime = fixtures.TunnelPlanContractTests().runtime()
        runtime.test_status['ego'].update(stamp=100., speed_kmh=-1.5)
        publisher = runtime.path_publisher
        publisher.start_reverse(runtime.test_status['ego'], static_recovery=False)
        self.assertTrue(gui.Runtime.reverse_escape_check(runtime, runtime.test_status['ego'],
            True, 0., np.array([[-1.2, 0.]]), time.monotonic(),
            swing_check=True, static_recovery=True))
        self.assertTrue(publisher.reverse_abort)
        self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, complete=False)))

    def test_zero_forward_publication_does_not_discard_static_exit_proof(self):
        runtime = self.runtime()
        self.expire(runtime, 'timeout')
        runtime.path_publisher.lap_cap_mps = 0.
        for stamp in (100., 100.1):
            self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, stamp)))
        payload = self.publish(runtime, self.plan(runtime, 100.2))
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertIsNotNone(runtime.path_publisher._reverse_exit_state)
        runtime.path_publisher.lap_cap_mps = None
        self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, 100.3, complete=False)))

    def test_active_reverse_monitors_rear_before_ready_or_stale_forward_plan(self):
        for stale in (False, True):
            for contact in (False, True):
                with self.subTest(stale=stale, contact=contact):
                    runtime = self.runtime()
                    plan = self.plan(runtime, 100.)
                    if stale:
                        plan = replace(plan, checked_at=time.monotonic()-1.)
                    points = np.empty((0, 2)) if contact else np.array([[-1.2, 0.]])
                    runtime._contact = dict(block_reverse=True) if contact else None
                    self.assertTrue(gui.Runtime.reverse_escape_check(runtime,
                        runtime.test_status['ego'], True, 0., points, time.monotonic(),
                        decision_plan=plan, swing_check=True, static_recovery=True))
                    self.assertTrue(runtime.path_publisher.reverse_abort)
                    self.assert_stopped_in_r(self.publish(runtime, plan))

    def test_provider_early_rejection_breaks_consecutive_forward_evidence(self):
        for fault in ('stale', 'mismatched_geometry'):
            with self.subTest(fault=fault):
                runtime = self.runtime()
                self.expire(runtime, 'timeout')
                publisher = runtime.path_publisher
                publisher._safety_provider = lambda world, ego, mode, plan: plan
                self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, 100.)))
                rejected = self.plan(runtime, 100.1)
                if fault == 'stale':
                    rejected = replace(rejected, checked_at=time.monotonic()-1.)
                else:
                    rejected = rejected.decided(geometry_path_digest='a different path')
                self.assert_stopped_in_r(self.publish(runtime, rejected))
                self.assertNotIn('static_clear_scans', publisher._reverse_exit_state)
                for stamp in (100.2, 100.3):
                    self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, stamp)))
                self.assertGreater(self.publish(runtime, self.plan(runtime, 100.4))['target_speed_mps'], 0.)

    def test_source_rejected_zero_command_requires_three_new_forward_proofs(self):
        class Provider:
            accepted = False

            def __call__(self, world, ego, mode, plan):
                return plan

            def publication_source_guard(self, plan):
                return nullcontext(self.accepted)

        runtime = self.runtime()
        self.expire(runtime, 'timeout')
        publisher = runtime.path_publisher
        publisher._safety_provider = provider = Provider()
        for stamp in (100., 100.1):
            self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, stamp)))
        rejected = self.publish(runtime, self.plan(runtime, 100.2))
        self.assertEqual(rejected['target_speed_mps'], 0.)
        self.assertNotIn('static_clear_scans', publisher._reverse_exit_state)
        provider.accepted = True
        for stamp in (100.3, 100.4):
            self.assert_stopped_in_r(self.publish(runtime, self.plan(runtime, stamp)))
        self.assertGreater(self.publish(runtime, self.plan(runtime, 100.5))['target_speed_mps'], 0.)


if __name__ == '__main__':
    unittest.main()
