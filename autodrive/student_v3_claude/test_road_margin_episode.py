"""Recorded current-pose recovery across consecutive frames 197 and 198.

The 243421 capture begins after producer-only geometry entered stationary
memory. Its original bound row is resealed through Runtime's storage API at
the recorded producer clock; no missing ego pose or raw observation is invented.
Both later frames use only their recorded current pose, map, points and prior
path. This replay establishes admission for these inputs, not native stability
or a measurement-noise bound.
"""
import copy
import json
import math
from pathlib import Path
import time
import unittest

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M
from lane_corridor import LaneCorridor
from perception.static_geometry_contract import STORED_PROOF_KEY, validate_stored_static_geometry
from perception.stationary_semantics import metadata, with_metadata
from static_lane_pass import measured_initial_join, _reachable_join_dynamics
from tunnel_runtime import swept_clearance


class RecordedMarginEpisode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(Path(__file__).parent/'test_data/road_margin_episode_20261019.npz') as data:
            cls.data = {key: data[key].copy() for key in data.files}

    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    def frame(self, sequence):
        prefix = f'frame{sequence}_'
        frame = {key: json.loads(str(self.data[prefix+key]))
                 for key in ('ego', 'static', 'previous', 'memory', 'objects', 'proof', 'info')}
        frame['static'] = [tuple(row) for row in frame['static']]
        for key in ('route', 'points'):
            frame[key] = self.data[prefix+key].copy()
        frame['now'] = float(self.data[prefix+'now'])
        frame['corridor'] = LaneCorridor(**{key: self.data[prefix+'corridor_'+key]
            for key in ('route', 'left', 'right', 'intersection_mask', 'body_shift_m')},
            zones=json.loads(str(self.data[prefix+'corridor_zones'])))
        frame['corridor'] = ma.road_white_line_corridor(frame['corridor'],
            frame['info']['white_line_right_m'])
        return frame

    def seeded_memory(self):
        runtime = gui.Runtime.__new__(gui.Runtime)
        for inserted, row in self.frame(197)['memory']:
            row = tuple(row)
            source_stamp = metadata(row)['producer_static_geometry']['source_stamp']
            runtime.remember_static_objects([row], inserted, True,
                measured_stamp=source_stamp)
        return runtime

    def recover(self, runtime, frame, ledger):
        self.assertEqual(frame['objects'], [])
        remembered = runtime.remember_static_objects([], frame['now'], True,
            tracked_objects=frame['objects'], measured_stamp=frame['ego']['stamp'])
        return ma.road_static_margin_recovery(remembered, frame['ego'], frame['points'],
            recovery_ledger=ledger)

    def callbacks(self, frame, objects):
        ego, route, corridor = frame['ego'], frame['route'], frame['corridor']
        _, raw = ma.road_static_checks(corridor, frame['points'])
        continued = np.asarray(frame['previous']['path'], float)[:, :2]
        checked = measured_initial_join(continued, ego,
            max_curvature=gui.MAX_CONTROL_CURVATURE, lateral_accel=gui.BYPASS_ALAT_MPS2)
        cache = {continued.tobytes(): checked, checked.tobytes(): checked}

        def canonical(path):
            path = np.asarray(path, float)[:, :2]
            key = path.tobytes()
            if key not in cache:
                ahead = np.flatnonzero(path[:, 0] > .05)
                joined = np.vstack((np.zeros((1, 2)), path[ahead[0]:]))
                current = gui.RosPathPublisher.control_geometry(joined, route,
                    dict(mode='LATTICE', validated_static=True))
                current = measured_initial_join(current, ego,
                    max_curvature=gui.MAX_CONTROL_CURVATURE, lateral_accel=gui.BYPASS_ALAT_MPS2)
                cache[key] = current
                cache[current.tobytes()] = current
            return cache[key]

        def clearance(path, horizon):
            path = canonical(path)
            if len(path) < 3 or max(gui.path_curvatures(path), default=math.inf) > gui.MAX_CONTROL_CURVATURE:
                return 0.
            return min(raw(path, horizon), gui.static_clear_distance(path, ego, objects,
                horizon, body_shift_m=REAR_AXLE_SHIFT_M))
        return canonical, clearance

    def plan(self, frame, objects):
        canonical, clearance = self.callbacks(frame, objects)
        return ma.road_rejoin_plan(frame['route'], frame['ego'], objects,
            frame['previous'], canonical, clearance, time.perf_counter()+.025)

    def assert_full_current_guards(self, plan, frame, objects):
        self.assertTrue(ma.static_plan_complete(plan), plan)
        path = plan['path']
        horizon = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        canonical, clearance = self.callbacks(frame, objects)
        self.assertGreater(horizon, 48.)
        self.assertGreaterEqual(clearance(path, horizon), horizon-.01)
        self.assertGreaterEqual(frame['corridor'].clear_distance(path, horizon), horizon-.01)
        self.assertGreaterEqual(swept_clearance(path, frame['points'], horizon, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(gui.static_clear_distance(path, frame['ego'], objects,
            horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertTrue(_reachable_join_dynamics(path, frame['ego'],
            gui.MAX_CONTROL_CURVATURE, gui.BYPASS_ALAT_MPS2)[0])
        self.assertIsNotNone(ma._road_steering_limit(path, frame['ego'], frame['ego']['speed_kmh']/3.6))
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        speed = frame['ego']['speed_kmh']/3.6
        lateral = np.maximum(0., speed*speed-3.*np.maximum(0., arc[1:-1]-speed*.7))*gui.path_curvatures(path)
        self.assertTrue(np.all(lateral <= gui.BYPASS_ALAT_MPS2+.05))

    def test_current_recovery_in_197_resets_old_credit_before_198_first_deficit(self):
        for old_anchor in (None, .12, .1999):
            runtime, ledger = self.seeded_memory(), {} if old_anchor is None else {10: old_anchor}
            initial_memory = copy.deepcopy(runtime._static_object_memory)
            first = self.frame(197)
            objects = self.recover(runtime, first, ledger)
            self.assertNotIn(10, ledger)
            plan = self.plan(first, objects)
            self.assertTrue(plan['kept_hard_return'])
            self.assert_full_current_guards(plan, first, objects)
            second = self.frame(198)
            objects = self.recover(runtime, second, ledger)
            self.assertAlmostEqual(ledger[10], .19646438983958842-.0049)
            plan = self.plan(second, objects)
            self.assertTrue(plan['kept_hard_return'])
            self.assertEqual((plan['offset_m'], plan['transition_m']), (.45, 22.))
            self.assert_full_current_guards(plan, second, objects)
            self.assertEqual(runtime._static_object_memory, initial_memory)
            row = next(obj for obj in objects if obj[11] == 10)
            geometry = metadata(row)['road_static_geometry']
            self.assertEqual(geometry['observed_points'], 0)
            self.assertEqual(metadata(row)['producer_static_geometry']['source_stamp'],
                1792408688.4173462)
            self.assertIsNotNone(validate_stored_static_geometry(metadata(row)[STORED_PROOF_KEY],
                row, second['ego']['stamp']))
            self.assertEqual(ma._road_rejoin_preference_objects([row], second['ego'],
                second['route'], include_near=True), [])

    def test_recorded_old_proof_and_exact_floor_both_reject_the_previous_path(self):
        frame = self.frame(198)
        unmarked = ma.road_static_margin_recovery(frame['static'], frame['ego'], frame['points'])
        row = next(obj for obj in unmarked if obj[11] == 10)
        self.assertNotIn('initial_margin_floor_m', metadata(row)['road_static_geometry'])
        canonical, clearance = self.callbacks(frame, unmarked)
        self.assertEqual(clearance(canonical(frame['previous']['path']), 48.), 0.)
        self.assertIsNone(self.plan(frame, unmarked))
        runtime = self.seeded_memory()
        marked = runtime.remember_static_objects([], frame['now'], True,
            measured_stamp=frame['ego']['stamp'])
        exact = ma.road_static_margin_recovery(marked, frame['ego'], frame['points'])
        canonical, clearance = self.callbacks(frame, exact)
        self.assertEqual(clearance(canonical(frame['previous']['path']), 48.), .5)
        self.assertIsNone(self.plan(frame, exact))

    def test_recorded_complete_path_keeps_physical_boxes_over_24cm_away(self):
        frame = self.frame(198)
        objects = self.recover(self.seeded_memory(), frame, {})
        plan = self.plan(frame, objects)
        path, ego = plan['path'], frame['ego']
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        sample = np.arange(0., arc[-1]+.001, .01)
        centres = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
        tangent = np.gradient(centres, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        tangent[0] = [1., 0.]
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        row = next(obj for obj in objects if obj[11] == 10)
        yaw = math.radians(row[8]-ego['heading_deg'])
        axis = np.array([math.cos(yaw), math.sin(yaw)])
        cross = np.array([-axis[1], axis[0]])
        hl, hw = np.asarray(metadata(row)['road_static_geometry']['physical_size_m'])/2.
        delta = np.asarray(ma._static.local_xy(row[0], row[1], ego))-centres-REAR_AXLE_SHIFT_M*tangent
        gaps = [np.abs(np.sum(delta*a, axis=1))-(
            EGO_HALF_LENGTH_M*np.abs(np.sum(tangent*a, axis=1))+
            EGO_HALF_WIDTH_M*np.abs(np.sum(normal*a, axis=1))+
            hl*np.abs(np.sum(axis*a, axis=-1))+hw*np.abs(np.sum(cross*a, axis=-1)))
            for a in (tangent, normal, axis, cross)]
        self.assertGreater(float(np.maximum.reduce(gaps).min()), .243)

    def test_current_new_raw_block_and_no_go_polygon_still_reject_the_prior(self):
        for changed in ('raw', 'map'):
            frame = self.frame(198)
            if changed == 'raw':
                frame['points'] = np.vstack((frame['points'], [8., 0.]))
            else:
                frame['corridor'].zones.append(np.array([[5., -6.], [9., -6.], [9., 6.], [5., 6.]]))
            objects = self.recover(self.seeded_memory(), frame, {})
            self.assertIsNone(self.plan(frame, objects), changed)

    def test_people_and_changed_motion_never_receive_the_recorded_floor(self):
        frame = self.frame(198)
        for index, value in ((4, 'pedestrian'), (5, .2), (9, .01)):
            objects = self.seeded_memory().remember_static_objects([], frame['now'], True,
                measured_stamp=frame['ego']['stamp'])
            changed = list(next(obj for obj in objects if obj[11] == 10))
            changed[index] = value
            checked, = ma.road_static_margin_recovery([tuple(changed)], frame['ego'],
                frame['points'], recovery_ledger={})
            self.assertNotIn('initial_margin_floor_m', metadata(checked)['road_static_geometry'])


if __name__ == '__main__':
    unittest.main()
