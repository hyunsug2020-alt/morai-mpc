"""Finish an observed lane return before searching the next distant obstacle."""
import json
import unittest

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from static_lane_pass import StaticLanePass
from test_static_initial_join_regression import ROOT, corridor
import test_static_initial_join_regression as initial
from tunnel_runtime import swept_clearance


class ObservedReturnTests(unittest.TestCase):
    setUp = initial.RecordedInitialJoinTests.setUp

    def plan(self, *, raw_contact=False, person=False, measured=True):
        with np.load(ROOT/'test_data/static_observed_return_20261023.npz', allow_pickle=False) as data:
            record = json.loads(str(data['record']))
        record['ego']['measured'] = measured
        policy = StaticLanePass()
        for key, value in record['policy'].items():
            setattr(policy, key, corridor(value) if key in ('legal', 'physical') else value)
        frozen = (policy.entered_at, policy.return_station_m)
        points = np.asarray(record['points'])
        if raw_contact:
            points = np.vstack((points, [[3.9, 0.]]))
        if person:
            ego = record['ego']
            record['objects'].append([ego['x'], ego['y'], .6, 'person', 'pedestrian',
                                      0., .6, .6, ego['heading_deg']])
        previous = dict(record['previous'], path=np.asarray(record['previous']['path']))
        plan, proof = policy.plan(self.lanes, record['ego'], np.asarray(record['route']),
            record['objects'], record['static'], points, corridor(record['legal']), previous,
            record['now'], record['time_scale'], record['ready'], budget_s=1.,
            **{key: record[key] for key in ('side_margin_m', 'contact', 'legal_checked',
                                          'reserve_source_bounds', 'slalom_first')})
        self.assertEqual((policy.entered_at, policy.return_station_m), frozen)
        return plan, proof, record, points

    def test_recorded_return_is_complete_without_waiting_for_the_next_obstacle(self):
        plan, proof, record, points = self.plan()
        self.assertIsNotNone(plan, {k: v for k, v in proof.items() if k != 'replay'})
        self.assertTrue(ma.static_plan_complete(plan))
        self.assertTrue(plan['static_borrow'])
        self.assertTrue(proof['ok'])
        self.assertLessEqual(proof['return_station_m'], record['policy']['return_station_m']+.25)
        length = float(np.linalg.norm(np.diff(plan['path'], axis=0), axis=1).sum())
        self.assertGreaterEqual(length, 8.)
        self.assertLessEqual(length, 12.01)
        self.assertGreater(plan['speed_cap_mps'], .5)
        self.assertGreaterEqual(swept_clearance(plan['path'], points, length, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), length-.01)
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], record['ego'],
            record['static'], length, body_shift_m=REAR_AXLE_SHIFT_M), length-.01)

    def test_return_does_not_remove_raw_person_or_measured_pose_requirements(self):
        for changes in (dict(raw_contact=True), dict(person=True), dict(measured=False)):
            with self.subTest(changes=changes):
                plan, _, _, _ = self.plan(**changes)
                self.assertIsNone(plan)


if __name__ == '__main__':
    unittest.main()
