"""The recorded static pass must not brake for an artificial origin kink."""
import copy
import json
import math
from pathlib import Path
import unittest

import numpy as np

import gui
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
import model_avoidance as ma
from static_lane_pass import StaticLanePass, _path_steering_profile
from tunnel_runtime import swept_clearance


ROOT = Path(__file__).parent
DATA = json.loads((ROOT / 'test_data' / 'static_initial_join_20261021.json').read_text())


def corridor(record):
    if record is None:
        return None
    return LaneCorridor(np.asarray(record['route']), np.asarray(record['left']),
        np.asarray(record['right']), zones=record['zones'],
        intersection_mask=np.asarray(record['intersection_mask']),
        body_shift_m=REAR_AXLE_SHIFT_M)


class RecordedInitialJoinTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        self.lanes = gui.HdMapLanes(ROOT / 'link_set.json')
        route = json.loads((ROOT / 'global_path.json').read_text())['waypoints']
        self.lanes.set_route(np.array([[p['x'], p['y']] for p in route]))

    def plan(self, extra_point=None, measured=True):
        record = copy.deepcopy(DATA)
        policy = StaticLanePass()
        for key, value in record['policy'].items():
            setattr(policy, key, corridor(value) if key in ('legal', 'physical') else value)
        record['ego']['measured'] = measured
        previous = dict(record['previous'], path=np.asarray(record['previous']['path']))
        points = np.asarray(record['points'])
        if extra_point is not None:
            points = np.vstack((points, [extra_point]))
        return policy.plan(self.lanes, record['ego'], np.asarray(record['route']),
            record['objects'], record['static'], points, corridor(record['legal']),
            previous, record['now'], record['time_scale'], record['ready'],
            # Keep geometry verification independent of host scheduling.
            budget_s=1., **{key: record[key] for key in ('side_margin_m', 'contact',
                'legal_checked', 'reserve_source_bounds', 'slalom_first')})

    def test_recorded_rolling_pass_keeps_a_complete_reachable_return(self):
        plan, proof = self.plan()
        self.assertIsNotNone(plan, {k: v for k, v in proof.items() if k != 'replay'})
        self.assertTrue(ma.static_plan_complete(plan))
        self.assertGreater(plan['speed_cap_mps'] * 3.6, 15.)
        self.assertLessEqual(plan['speed_cap_mps'] * 3.6, 25.)
        self.assertTrue(proof['ok'])
        self.assertLess(proof['contact_s'], 2.5)
        path, ego = plan['path'], DATA['ego']
        horizon = plan['clear_distance_m']
        self.assertGreaterEqual(swept_clearance(path, DATA['points'], horizon, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(gui.static_clear_distance(path, ego, DATA['static'],
            horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        arc, gradient = _path_steering_profile(path, ego)
        speed = ego['speed_kmh'] / 3.6
        reachable = np.sqrt(np.maximum(0., speed**2 - 5.*np.maximum(0., arc[1:]-speed*.2)))
        self.assertLessEqual(float(np.max(reachable*gradient)), math.radians(30.)+1e-6)
        self.assertLessEqual(max(gui.path_curvatures(path)), gui.MAX_CONTROL_CURVATURE)

    def test_measured_speed_fallback_keeps_current_raw_contact_and_pose_requirements(self):
        plan, _ = self.plan(extra_point=[3.9, 0.])
        self.assertIsNone(plan)
        plan, _ = self.plan(measured=False)
        self.assertIsNone(plan)


if __name__ == '__main__':
    unittest.main()
