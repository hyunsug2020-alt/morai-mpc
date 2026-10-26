"""Normal-road static passes agree with the final contact and timing guards."""
import unittest
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from ped_guard import contact_guard, sweep_ok
from static_lane_pass import StaticLanePass
import test_avoidance_gate as avoidance_fixtures


class StaticSteadyPassTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        self.route = np.column_stack((np.arange(0., 80.1, .5), np.zeros(161)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=20., stamp=100.)
        self.box = (20., -.6, .6, 'barrel', 'obstacle', 0., .6, .6, 0.)
        self.points = np.array([[x, y] for x in (19.7, 20., 20.3) for y in (-.9, -.6, -.3)])
        self.corridor = LaneCorridor(self.route, np.full(161, 3.5), np.full(161, -3.5),
                                    body_shift_m=REAR_AXLE_SHIFT_M)

    def run_plan(self, *, scope=True, contact=None, points=None, people=()):
        points = self.points if points is None else points
        objects = [self.box, *people]
        runtime = avoidance_fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=objects, points=points)
        runtime._contact = contact
        lane, clearance = ma.road_static_checks(self.corridor, points)
        with patch('static_lane_pass.normal_static_area', return_value=scope), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            plan, _, _ = runtime.model_static_plan(self.ego, self.route, objects, points,
                self.corridor, lane, clearance, None, {}, 100., True, True, 1.)
        return plan

    def test_a_full_pass_prefers_tracking_room_and_the_published_body_geometry(self):
        plan = self.run_plan()
        self.assertTrue(ma.static_plan_complete(plan), plan)
        self.assertTrue(plan['canonical_control'])
        _, comfortable = ma.road_static_checks(self.corridor, self.points, margin_m=.45)
        self.assertGreaterEqual(comfortable(plan['path'], 35.), 35.)
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], self.ego, [self.box],
            35., body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        np.testing.assert_allclose(plan['path'][0], [0., 0.])
        self.assertLessEqual(max(gui.path_curvatures(plan['path'])), gui.MAX_CONTROL_CURVATURE)
        self.assertGreater(gui.model_static_speed_cap_mps(20./3.6, plan, speed_mps=20./3.6), 0.)

    def test_remembered_side_return_gets_a_straight_departure_before_the_turn(self):
        # The current scan can lose a nearby return while contact memory keeps
        # it. The original planner approved a path the final sweep then stops.
        contact = contact_guard(np.array([[3., 1.2]]))
        distance = max(1.5, contact['ahead_m']+.5)
        old = self.run_plan(scope=False, contact=contact)
        self.assertTrue(ma.static_plan_complete(old), old)
        self.assertFalse(sweep_ok(contact, old['path'], distance))
        plan = self.run_plan(contact=contact)
        self.assertTrue(ma.static_plan_complete(plan), plan)
        self.assertTrue(plan['contact_delayed'])
        self.assertTrue(sweep_ok(contact, plan['path'], distance))
        straight = plan['path'][plan['path'][:, 0] < distance-.5]
        self.assertGreater(len(straight), 5)
        self.assertLess(np.max(np.abs(straight[:, 1])), .01)

    def test_a_new_actual_front_blocker_cannot_be_removed_by_the_smooth_pass(self):
        plan = self.run_plan(points=np.vstack((self.points, [[3.9, 0.]])))
        self.assertTrue(plan is None or plan.get('blocked'), plan)

    def test_person_rows_and_person_contact_memory_keep_the_existing_policy(self):
        person = (30., 2., .6, 'person', 'pedestrian', 0., .6, .6, 0.)
        for people, contact in (([person], None), ([], contact_guard(None, [(3., 1.5)]))):
            with self.subTest(people=bool(people)), \
                    patch.object(ma, 'contact_static_pass', side_effect=AssertionError('person policy changed')):
                self.run_plan(people=people, contact=contact)

    def test_expired_lattice_deadline_does_not_publish_an_unchecked_positive_path(self):
        from lattice_planner import plan
        with patch('time.perf_counter', return_value=12.):
            result = plan(self.route, self.ego, self.points, [self.box], self.corridor,
                candidate_targets=(0., .75), candidate_lengths=(14.,),
                paved_road_only=True, road_corridor=self.corridor, deadline=11.)
        self.assertTrue(result['blocked'])
        self.assertEqual(result['speed_cap_mps'], 0.)
        self.assertEqual(result['evaluated'], 0)
        self.assertTrue(result['rejected']['budget_exhausted'])

    def test_shared_portable_deadline_also_caps_the_no_candidate_extension(self):
        now = 100.
        budget = ma.Budget(.09, extra_s=.06, clock=lambda: now, deadline=100.02)
        self.assertFalse(budget.expired())
        now = 100.021
        self.assertTrue(budget.expired())

    def exhausted_search(self, previous, points=None, objects=None, saved=None):
        points = self.points if points is None else points
        objects = [self.box] if objects is None else objects
        runtime = avoidance_fixtures.AvoidanceGateTests().runtime(
            action='KEEP', objects=objects, points=points)
        runtime._contact = None
        runtime._static_plan_saved = saved
        lane, clearance = ma.road_static_checks(self.corridor, points)
        failed = dict(blocked=True, kind='bypass', safe_horizon_m=4., activation_m=25.)
        with patch('static_lane_pass.normal_static_area', return_value=True), \
                patch.object(gui, 'model_avoidance', ma, create=True), \
                patch.object(ma, 'road_rejoin_plan', return_value=None), \
                patch.object(ma, 'plan_bypass',
                    return_value=(failed, [], dict(budget_exhausted=True))), \
                patch.object(StaticLanePass, 'plan',
                    return_value=(None, dict(reason='budget exhausted'))):
            plan, _, _ = runtime.model_static_plan(self.ego, self.route, objects, points,
                self.corridor, lane, clearance, previous, {}, 100., True, True, 1.)
        return plan

    def test_search_expiry_keeps_a_full_pass_rechecked_against_current_geometry(self):
        previous = self.run_plan()
        plan = self.exhausted_search(previous)
        self.assertTrue(ma.static_plan_complete(plan), plan)
        self.assertTrue(plan['kept_checked_static'])
        self.assertTrue(plan['reused'])
        np.testing.assert_allclose(plan['path'], previous['path'])

    def test_gentler_saved_join_survives_an_untrackable_short_join_without_lowering_clearance(self):
        previous = self.run_plan()
        saved = dict(stamp=100., plan=previous,
            world_path=[gui.RosPathPublisher.global_xy(self.ego, *p[:2]) for p in previous['path']])
        abrupt = dict(previous, path=previous['path'].copy())
        abrupt['path'][1, 1] += .8
        plan = self.exhausted_search(abrupt, saved=saved)
        self.assertTrue(ma.static_plan_complete(plan), plan)
        self.assertTrue(plan['kept_checked_static'])
        self.assertIn(plan['continuation_join_m'], (8., 10., 12.))
        _, clear = ma.road_static_checks(self.corridor, self.points)
        self.assertGreaterEqual(clear(plan['path'], 35.), 35.)
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], self.ego,
            [self.box], 35., body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        blocked = self.exhausted_search(abrupt,
            points=np.vstack((self.points, [[3.9, 0.]])), saved=saved)
        self.assertTrue(blocked is None or blocked.get('blocked'), blocked)
        self.assertGreater(gui.model_static_speed_cap_mps(20./3.6, plan, speed_mps=20./3.6), 0.)

    def test_expiry_cannot_retain_a_pass_after_a_new_raw_front_blocker(self):
        previous = self.run_plan()
        plan = self.exhausted_search(previous, points=np.vstack((self.points, [[3.9, 0.]])))
        self.assertTrue(plan is None or plan.get('blocked'), plan)

    def test_expiry_cannot_retain_a_pass_after_a_new_semantic_front_body(self):
        previous = self.run_plan()
        front = (4., 0., 1., 'new box', 'obstacle', 0., 1., 1., 0.)
        plan = self.exhausted_search(previous, objects=[self.box, front])
        self.assertTrue(plan is None or plan.get('blocked'), plan)

    def test_checked_continuation_rejects_incomplete_and_untrackable_old_paths(self):
        previous = self.run_plan()
        for change in (dict(clear_distance_m=0.), dict(standing_bypass_ids=[1]),
                       dict(static_borrow=True), dict(bounded_static_pass=True),
                       dict(path=previous['path'][:10])):
            with self.subTest(change=list(change)):
                self.assertIsNone(ma.road_checked_continuation(dict(previous, **change), self.ego,
                    lambda p: p, lambda p, h: h))
        self.assertIsNone(ma.road_checked_continuation(previous,
            dict(self.ego, wheel_angle=float('nan')), lambda p: p, lambda p, h: h))


if __name__ == '__main__':
    unittest.main()
