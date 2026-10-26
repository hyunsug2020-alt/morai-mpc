"""A stopped car may gain speed on a checked in-lane prefix without borrowing."""
import json
import unittest

import numpy as np

import gui
from ego_body import REAR_AXLE_SHIFT_M
from static_lane_pass import StaticLanePass, body_stations, PROFILE_RETURN_BUDGET_S
from test_static_initial_join_regression import ROOT, corridor
import test_static_initial_join_regression as initial
from tunnel_runtime import swept_clearance


class StoppedApproachTests(unittest.TestCase):
    setUp = initial.RecordedInitialJoinTests.setUp

    def plan(self, *, raw_contact=False, person=False, measured=True):
        with np.load(ROOT/'test_data/static_stopped_approach_20261023.npz', allow_pickle=False) as data:
            r = json.loads(str(data['record']))
        r['ego']['measured'] = measured
        policy = StaticLanePass()
        for k, v in r['policy'].items():
            setattr(policy, k, corridor(v) if k in ('legal', 'physical') else v)
        before = (policy.entered_at, policy.return_station_m)
        points = np.asarray(r['points'])
        if raw_contact:
            points = np.vstack((points, [[3.9, 0.]]))
        if person:
            e = r['ego']
            r['objects'].append([e['x'], e['y'], .6, 'person', 'pedestrian', 0., .6, .6, 0.])
        previous = dict(r['previous'], path=np.asarray(r['previous']['path']))
        plan, proof = policy.plan(self.lanes, r['ego'], np.asarray(r['route']), r['objects'],
            r['static'], points, corridor(r['legal']), previous, r['now'], r['time_scale'],
            r['ready'], budget_s=1., **{k: r[k] for k in ('side_margin_m', 'contact',
                'legal_checked', 'reserve_source_bounds', 'slalom_first')})
        self.assertEqual(policy.entered_at, before[0])
        if plan and plan.get('static_motion_profile'):
            self.assertTrue(proof['launch_ready'])
            self.assertLessEqual(proof['contact_s'], PROFILE_RETURN_BUDGET_S)
            self.assertEqual(policy.return_station_m, proof['return_station_m'])
        else:
            self.assertEqual(policy.return_station_m, before[1])
        legal = policy.corridor(r['ego'], corridor(r['legal']))
        physical = policy.physical if policy.physical is not None else legal
        return plan, proof, r, points, legal, physical

    def test_recorded_standstill_gets_a_complete_stoppable_path_with_no_line_contact(self):
        plan, proof, r, points, legal, physical = self.plan()
        self.assertIsNotNone(plan, {k: v for k, v in proof.items() if k != 'replay'})
        timed = plan.get('static_motion_profile', False)
        self.assertTrue(timed or plan.get('approach_only'))
        self.assertEqual(plan['static_borrow'], timed)
        self.assertGreater(plan['speed_cap_mps'], .5)
        path = plan['path']
        horizon = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        self.assertGreater(horizon, 8.)
        _, left, right = body_stations(path, legal)
        self.assertLessEqual(float(left.max()), .78 if timed else .03)
        self.assertLessEqual(float(right.max()), .03)
        self.assertGreaterEqual(physical.clear_distance(path, horizon), horizon-.25)
        self.assertGreaterEqual(swept_clearance(path, points, horizon, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.25)
        self.assertGreaterEqual(gui.static_clear_distance(path, r['ego'], r['static'],
            horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.25)
        v = plan['speed_cap_mps']
        self.assertLessEqual(v*.7+v*v/3.+2., horizon+.01)

    def test_actual_contact_person_and_unmeasured_pose_cannot_gain_an_approach(self):
        for change in (dict(raw_contact=True), dict(person=True), dict(measured=False)):
            with self.subTest(change=change):
                plan, _, _, _, _, _ = self.plan(**change)
                self.assertIsNone(plan)

    def test_close_captured_stop_has_a_complete_profile_and_keeps_final_speed_constraints(self):
        with np.load(ROOT/'test_data/static_close_stopped_20261023.npz', allow_pickle=False) as data:
            r = json.loads(str(data['record']))
        policy = StaticLanePass()
        plan, proof = policy.plan(self.lanes, r['ego'], np.asarray(r['route']), r['objects'],
            r['static'], np.asarray(r['points']), corridor(r['legal']), None, r['now'],
            r['time_scale'], True, budget_s=1., **{k: r[k] for k in ('side_margin_m',
                'contact', 'legal_checked', 'reserve_source_bounds', 'slalom_first')})
        self.assertIsNotNone(plan, {k: v for k, v in proof.items() if k != 'replay'})
        self.assertTrue(plan['static_motion_profile'])
        self.assertLessEqual(proof['contact_s'], PROFILE_RETURN_BUDGET_S)
        self.assertLessEqual(proof['encroachment_m'], .75)
        ok = policy.command_check(plan['path'], r['ego'], plan['speed_cap_mps'],
                                  r['now'], motion_profile=True)
        self.assertTrue(ok['ok'], ok)
        lagged = policy.command_check(plan['path'], r['ego'], .97*plan['speed_cap_mps'],
                                      r['now'], motion_profile=True)
        self.assertTrue(lagged['ok'], lagged)
        self.assertLessEqual(lagged['contact_s'], PROFILE_RETURN_BUDGET_S)
        slow = policy.command_check(plan['path'], r['ego'], .1, r['now'], motion_profile=True)
        self.assertFalse(slow['ok'], slow)

    def test_profile_continuation_keeps_the_world_curve_after_a_near_vertex_projection(self):
        from scipy.interpolate import CubicSpline
        from highway_planner import world_points
        with np.load(ROOT/'test_data/static_close_stopped_20261023.npz', allow_pickle=False) as data:
            r = json.loads(str(data['record']))
        policy = StaticLanePass()
        plan, proof = policy.plan(self.lanes, r['ego'], np.asarray(r['route']), r['objects'],
            r['static'], np.asarray(r['points']), corridor(r['legal']), None, r['now'],
            r['time_scale'], True, budget_s=1., **{k: r[k] for k in ('side_margin_m',
                'contact', 'legal_checked', 'reserve_source_bounds', 'slalom_first')})
        self.assertIsNotNone(plan)
        path = plan['path']
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        curve = CubicSpline(arc, path)
        at = arc[1]-.001
        xy = world_points(curve(at)[None], r['ego'])[0]
        tangent = curve(at, 1)
        ego = dict(r['ego'], x=xy[0], y=xy[1], speed_kmh=4., stamp=r['ego']['stamp']+.5,
            heading_deg=r['ego']['heading_deg']+float(np.degrees(np.arctan2(tangent[1], tangent[0]))))
        saved = dict(plan=plan, world_path=world_points(path, r['ego']), stamp=r['now'])
        continued = gui.continue_static_plan(saved, ego, r['now']+.5)
        self.assertIsNotNone(continued)
        steps = np.linalg.norm(np.diff(continued['path'], axis=0), axis=1)
        self.assertGreater(float(steps[:-1].min()), .4)
        self.assertTrue(continued['static_motion_profile'])


if __name__ == '__main__':
    unittest.main()
