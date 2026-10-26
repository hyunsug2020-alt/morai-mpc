"""A departing car's curved-path projection must retain measured body clearance."""
import unittest

import numpy as np

from ego_body import EGO_FRONT_M
from traffic_planner import GapPlanner
from test_roundabout_entry_standoff import car


class DepartingFollowProjectionTests(unittest.TestCase):
    def evaluate(self, *, lateral=2.15, speed=3., valid=True):
        path = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))
        planner = GapPlanner([dict(points=path.tolist(), from_node_idx='a', to_node_idx='b')])
        planner.junction = dict(yield_only=True, constant_ring_speed=True,
                                held=False, ready=True)
        planner.junction_committed = True
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        obj = car(EGO_FRONT_M+2.3, lateral, speed, 0., 7,
                  length=4.6, width=1.9, heading=0., valid=valid)
        result = planner.evaluate([obj], ego, path, 30./3.6, 100., fresh=True)
        return result, planner.status

    def test_clear_departing_measured_body_does_not_cancel_the_committed_launch(self):
        result, status = self.evaluate()
        self.assertGreater(result['v_mps'], 0.)
        self.assertLessEqual(result['v_mps'], 3.)
        self.assertGreater(status['leader']['measured_body_clear_m'], .1)
        self.assertLessEqual(status['leader']['gap_m'], .1)

    def test_real_contact_unmeasured_motion_and_stopped_leaders_still_stop(self):
        for kwargs in (dict(lateral=0.), dict(valid=False), dict(speed=0.)):
            with self.subTest(**kwargs):
                result, _ = self.evaluate(**kwargs)
                self.assertEqual(result['v_mps'], 0.)


if __name__ == '__main__':
    unittest.main()
