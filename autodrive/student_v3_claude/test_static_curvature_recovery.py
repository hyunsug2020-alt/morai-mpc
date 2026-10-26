"""Recorded random scenarios retain complete, checked forward recovery paths."""
import json
import unittest

import numpy as np

import gui
from ego_body import REAR_AXLE_SHIFT_M
from static_lane_pass import StaticLanePass, body_stations, PROFILE_RETURN_BUDGET_S
from test_static_initial_join_regression import ROOT, corridor
import test_static_initial_join_regression as initial
from tunnel_runtime import swept_clearance


class RecordedCurvatureRecoveryTests(unittest.TestCase):
    setUp = initial.RecordedInitialJoinTests.setUp

    def recover(self, number, *, contact=False, person=False, live=False):
        suffix = '_live' if live else ''
        with np.load(ROOT/f'test_data/static_random_{number:02d}{suffix}_20261023.npz',
                     allow_pickle=False) as fixture:
            r = json.loads(str(fixture['record']))
        points = np.asarray(r['points'])
        if contact:
            points = np.vstack((points, [[3.9, 0.]]))
        if person:
            r['objects'].append([r['ego']['x'], r['ego']['y'], .6,
                                 'person', 'pedestrian', 0., .6, .6, 0.])
        policy = StaticLanePass()
        previous = (dict(r['previous'], path=np.asarray(r['previous']['path']))
                    if r['previous'] is not None else None)
        plan, proof = policy.plan(self.lanes, r['ego'], np.asarray(r['route']),
            r['objects'], r['static'], points, corridor(r['legal']), previous,
            r['now'], r['time_scale'], True, budget_s=.11,
            **{key: r[key] for key in ('side_margin_m', 'contact', 'legal_checked',
                                       'reserve_source_bounds', 'slalom_first')})
        return plan, proof, r, points

    def test_captured_deadlocks_have_complete_forward_paths_with_unchanged_margins(self):
        for number, live in ((4, False), (4, True), (5, False)):
            with self.subTest(scenario=number, live=live):
                plan, proof, r, points = self.recover(number, live=live)
                self.assertIsNotNone(plan, {k: v for k, v in proof.items() if k != 'replay'})
                self.assertGreater(plan['speed_cap_mps'], .5)
                self.assertLessEqual(proof['contact_s'], PROFILE_RETURN_BUDGET_S)
                path = plan['path']
                horizon = np.linalg.norm(np.diff(path, axis=0), axis=1).sum()
                self.assertGreaterEqual(horizon, 18.-1e-3)
                self.assertGreaterEqual(swept_clearance(path, points, horizon, .25,
                    lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.25)
                self.assertGreaterEqual(gui.static_clear_distance(path, r['ego'], r['static'],
                    horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.25)
                _, left, right = body_stations(path, corridor(r['legal']))
                self.assertLessEqual(max(left), .78)
                self.assertLessEqual(max(right), .03)

    def test_a_current_contact_or_person_cannot_be_released_by_an_extra_proposal(self):
        for number in (4, 5):
            for change in (dict(contact=True), dict(person=True)):
                with self.subTest(scenario=number, change=change):
                    plan, _, _, _ = self.recover(number, **change)
                    self.assertIsNone(plan)

    def test_stopped_recovery_does_not_count_producer_padding_as_a_second_body(self):
        with np.load(ROOT/'test_data/static_stopped_source_envelope_20261023.npz',
                     allow_pickle=False) as fixture:
            r = json.loads(str(fixture['record']))
        for contact in (False, True):
            points = np.asarray(r['points'])
            if contact:
                points = np.vstack((points, [[3.8, 0.]]))
            policy = StaticLanePass()
            plan, proof = policy.plan(self.lanes, r['ego'], np.asarray(r['route']),
                r['objects'], r['static'], points, corridor(r['legal']), None,
                r['now'], r['time_scale'], r['ready'], budget_s=.11,
                **{key: r[key] for key in ('side_margin_m', 'contact', 'legal_checked',
                                         'reserve_source_bounds', 'slalom_first')})
            if contact:
                self.assertIsNone(plan)
                continue
            self.assertIsNotNone(plan, {k: v for k, v in proof.items() if k != 'replay'})
            self.assertTrue(plan['static_motion_profile'])
            self.assertLessEqual(proof['contact_s'], PROFILE_RETURN_BUDGET_S-.25)
            path = plan['path']
            horizon = np.linalg.norm(np.diff(path, axis=0), axis=1).sum()
            self.assertGreaterEqual(gui.static_clear_distance(path, r['ego'], r['static'],
                horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.25)
            self.assertGreaterEqual(swept_clearance(path, points, horizon, .25,
                lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.25)

    def test_recorded_rolling_approaches_keep_a_clear_in_lane_path_above_20_kmh(self):
        for index in range(2):
            with self.subTest(capture=index):
                with np.load(ROOT/f'test_data/static_random_10_rolling_{index}_20261023.npz',
                             allow_pickle=False) as fixture:
                    r = json.loads(str(fixture['record']))
                policy = StaticLanePass()
                for key, value in r['policy'].items():
                    setattr(policy, key, corridor(value) if key in ('legal', 'physical') else value)
                previous = (dict(r['previous'], path=np.asarray(r['previous']['path']))
                            if r['previous'] is not None else None)
                plan, proof = policy.plan(self.lanes, r['ego'], np.asarray(r['route']),
                    r['objects'], r['static'], np.asarray(r['points']), corridor(r['legal']),
                    previous, r['now'], r['time_scale'], r['ready'], budget_s=.11,
                    **{key: r[key] for key in ('side_margin_m', 'contact', 'legal_checked',
                                               'reserve_source_bounds', 'slalom_first')})
                self.assertIsNotNone(plan, {k: v for k, v in proof.items() if k != 'replay'})
                self.assertFalse(plan['static_borrow'])
                self.assertGreater(plan['speed_cap_mps']*3.6, 20.)
                self.assertEqual(proof['contact_s'], 0.)
                path = plan['path']
                horizon = np.linalg.norm(np.diff(path, axis=0), axis=1).sum()
                self.assertGreaterEqual(swept_clearance(path, np.asarray(r['points']),
                    horizon, .25, lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.25)
                self.assertGreaterEqual(gui.static_clear_distance(path, r['ego'], r['static'],
                    horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.25)

    def test_measured_lane_gap_keeps_the_existing_time_and_body_bounds(self):
        with np.load(ROOT/'test_data/static_surveyed_gap_20261023.npz',
                     allow_pickle=False) as fixture:
            r = json.loads(str(fixture['record']))
        policy = StaticLanePass()
        for key, value in r['policy'].items():
            setattr(policy, key, corridor(value) if key in ('legal', 'physical') else value)
        previous = dict(r['previous'], path=np.asarray(r['previous']['path']))
        plan, proof = policy.plan(self.lanes, r['ego'], np.asarray(r['route']),
            r['objects'], r['static'], np.asarray(r['points']), corridor(r['legal']),
            previous, r['now'], r['time_scale'], r['ready'], budget_s=.11,
            **{key: r[key] for key in ('side_margin_m', 'contact', 'legal_checked',
                                       'reserve_source_bounds', 'slalom_first')})
        self.assertIsNotNone(plan, {k: v for k, v in proof.items() if k != 'replay'})
        self.assertLessEqual(proof['contact_s'], PROFILE_RETURN_BUDGET_S)
        self.assertLessEqual(proof['encroachment_m'], .75)
        self.assertGreater(plan['speed_cap_mps']*3.6, 20.)


if __name__ == '__main__':
    unittest.main()
