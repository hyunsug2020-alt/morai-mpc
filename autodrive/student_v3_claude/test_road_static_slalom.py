"""Replay the alternating barrels that stopped MORAI at route index 661."""
import json
from pathlib import Path
import time
import unittest
from unittest.mock import patch

import numpy as np

import gui
import lattice_planner
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from lidar_corridor import LidarCorridor
import test_avoidance_gate as fixtures
from tunnel_runtime import swept_clearance


class RecordedStaticSlalomTests(unittest.TestCase):
    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        with np.load(Path(__file__).parent/'test_data/road_static_slalom_20261021.npz') as data:
            self.ego = json.loads(str(data['ego']))
            self.objects = json.loads(str(data['objects']))
            self.route, self.points = data['route'].copy(), data['points'].copy()
            self.raw_xyz, self.x_range = data['raw_xyz'].copy(), tuple(data['x_range'])
            self.corridor = LaneCorridor(self.route, data['left'], data['right'],
                zones=data['zones'].tolist(), intersection_mask=data['intersection_mask'],
                body_shift_m=REAR_AXLE_SHIFT_M)

    def test_replay_uses_the_live_lidar_range_and_keeps_the_following_row(self):
        lidar = LidarCorridor(x_range=self.x_range)
        lidar.ingest(self.raw_xyz, stamp=self.ego['stamp'])
        np.testing.assert_array_equal(lidar.pts, self.points)
        self.assertEqual(self.x_range, (-75., 100.))
        self.assertTrue(np.any((self.points[:, 0] > 48.) & (self.points[:, 0] < 50.)))

    def run_plan(self, points=None, previous=None):
        points = self.points if points is None else points
        runtime = fixtures.AvoidanceGateTests().runtime(
            action='KEEP', objects=self.objects, points=points)
        runtime._contact, runtime._lidar_log = None, dict(age_s=.01)
        runtime.lanes.n_links = 0
        runtime.model_road_offsets = lambda *args: ma.Offsets({1: 3.2, -1: 3.2})
        lane, clear = ma.road_static_checks(self.corridor, points)
        with patch('static_lane_pass.normal_static_area', return_value=True), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            plan, _, context = runtime.model_static_plan(self.ego, self.route, self.objects,
                points, self.corridor, lane, clear, previous, {1: 3.2, -1: 3.2},
                time.monotonic(), True, True, 1.)
        return plan, context, runtime

    def test_recorded_fourteen_metre_cutoff_excluded_the_only_complete_family(self):
        original = lattice_planner.slalom_profiles
        def old_rows(*args, **kwargs):
            kwargs['max_row_gap_m'] = 14.
            return original(*args, **kwargs)
        # Isolate the old row-family cutoff from the additional fully checked
        # measured-steering recovery proposals, which can now solve this stop.
        with patch.object(lattice_planner, 'slalom_profiles', side_effect=old_rows), \
                patch('static_lane_pass.measured_curvature_paths', return_value=iter(())):
            plan, _, _ = self.run_plan()
        self.assertFalse(ma.static_plan_complete(plan), plan)

    def test_recorded_standstill_gets_a_complete_legal_sweep_through_both_barrels(self):
        plan, context, runtime = self.run_plan()
        self.assertTrue(ma.static_plan_complete(plan), runtime._model_avoidance_log)
        self.assertTrue(runtime._model_avoidance_log['slalom_priority'])
        self.assertFalse(plan['static_borrow'])
        self.assertGreater(plan['speed_cap_mps'], .5)
        required = min(55., plan['clear_distance_m'])
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], self.ego,
            context['static'], required, body_shift_m=REAR_AXLE_SHIFT_M), required)
        self.assertGreaterEqual(swept_clearance(plan['path'], self.points, required, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), required)
        self.assertGreaterEqual(context['corridor'].clear_distance(plan['path'], required), required)
        self.assertLessEqual(max(gui.path_curvatures(plan['path'])), gui.MAX_CONTROL_CURVATURE)

    def test_current_front_return_still_vetoes_the_recorded_sweep(self):
        plan, _, _ = self.run_plan(points=np.vstack((self.points, [[3.9, 0.]])))
        self.assertFalse(ma.static_plan_complete(plan), plan)

    def test_reused_slalom_recomputes_pace_after_a_transient_low_cap(self):
        plan, _, _ = self.run_plan()
        self.assertTrue(ma.static_plan_complete(plan), plan)
        repeated, _, _ = self.run_plan(previous=dict(plan, speed_cap_mps=.05))
        self.assertTrue(ma.static_plan_complete(repeated), repeated)
        self.assertGreater(repeated['speed_cap_mps'], .5)


if __name__ == '__main__':
    unittest.main()
