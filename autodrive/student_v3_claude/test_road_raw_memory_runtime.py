"""Recorded current-only road memory integration, independent of source preferences.

These replays run the real host planner at recorded measured poses. They prove
geometry and observation ownership, not a simulated or native closed-loop drive.
"""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from perception.road_raw_memory import MEMORY_KEY
from tunnel_runtime import swept_clearance
import test_avoidance_gate as fixtures


class RecordedRawMemoryRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(Path(__file__).parent / 'test_data/road_raw_hazard_sequence_20261017.npz') as data:
            cls.data = {key: data[key].copy() for key in data.files}

    def setUp(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    def frame(self, number):
        prefix = f'frame{number}_'
        row = {key: json.loads(str(self.data[prefix + key])) for key in
               ('ego', 'objects', 'static', 'previous', 'memory', 'contact',
                'info', 'proof', 'now', 'zones', 'captured_plan')}
        row.update({key: self.data[prefix + key].copy() for key in
                    ('route', 'points', 'input_points', 'left', 'right', 'intersection_mask')})
        for key in ('previous', 'captured_plan'):
            if row[key] is not None and 'path' in row[key]:
                row[key]['path'] = np.asarray(row[key]['path'], dtype=float)
        row['body_shift_m'] = float(self.data[prefix + 'body_shift_m'])
        frame = row['proof']['road_scan_frame']
        row['check_wall'] = float(frame['stamp']) + float(frame['wall_age_s'])
        return row

    @staticmethod
    def offsets(row):
        # Preserve the integer keys and attributes of road_offsets, rather than
        # accidentally turning permitted adjacent lanes into an empty palette.
        offsets = ma.Offsets({int(k): float(v) for k, v in row['info']['offsets'].items()})
        offsets.start_m_by_side = {
            int(k): tuple(v) for k, v in row['info']['offset_starts'].items()}
        offsets.open_road_sides = tuple(sorted(offsets))
        return offsets

    def runtime(self, row):
        runtime = fixtures.AvoidanceGateTests().runtime(
            action='KEEP', objects=row['objects'], points=row['input_points'])
        runtime.zone_kmh = 55.
        runtime.tunnel = SimpleNamespace(active=lambda ego: False)
        return runtime

    def plan(self, row, runtime=None):
        runtime = runtime or self.runtime(row)
        # Each frame contributes only its captured current semantic observation.
        # Raw observation history is deliberately retained across these calls.
        runtime._contact = copy.deepcopy(row['contact'])
        runtime._static_object_memory = copy.deepcopy(row['memory'])
        runtime._lidar_log = copy.deepcopy(row['proof'])
        offsets = self.offsets(row)
        runtime.model_road_offsets = lambda *args: offsets
        right = row['right'] + np.where(
            row['intersection_mask'], 0., row['info']['white_line_right_m'])
        corridor = LaneCorridor(row['route'], row['left'], right, row['zones'],
            row['intersection_mask'], body_shift_m=row['body_shift_m'])
        lane, clear = ma.road_static_checks(corridor, row['input_points'])
        with patch.object(gui, 'model_avoidance', ma, create=True), \
                patch.object(ma, 'v6_decision', return_value=(row['info'].get('v6'), None)), \
                patch('static_lane_pass.normal_static_area', return_value=True), \
                patch('gui.time.time', return_value=row['check_wall']):
            result = runtime.model_static_plan(row['ego'], row['route'], row['objects'],
                row['input_points'], corridor, lane, clear, row['previous'], {},
                row['now'], True, True, 1.)
        return result, runtime

    @staticmethod
    def save(plan, row):
        yaw = math.radians(row['ego']['heading_deg'])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        return dict(stamp=row['now'],
            world_path=np.asarray(plan['path']) @ rotation.T + [row['ego']['x'], row['ego']['y']],
            plan={key: value for key, value in plan.items() if key != 'path'})

    def replay(self, numbers):
        runtime, saved, results = None, None, {}
        for number in numbers:
            row = self.frame(number)
            # Earlier inputs preserve the actual captured incoming manoeuvre.
            # After the changed dropout decision, continue the newly selected
            # world path instead of injecting the recording's old rejoin.
            if number >= 207 and saved is not None:
                row['previous'] = ma.portable.continue_static_plan(saved, row['ego'], row['now'])
                self.assertIsNotNone(row['previous'])
            result, runtime = self.plan(row, runtime)
            plan, _, context = result
            results[number] = dict(row=row, plan=plan, context=context,
                diagnostic=copy.deepcopy(runtime._committed_road_raw_memory.last_diagnostics),
                tracks=copy.deepcopy(runtime._committed_road_raw_memory._tracks))
            if plan and not plan.get('blocked'):
                saved = self.save(plan, row)
        return results, runtime

    def assert_full_checks(self, frame):
        row, plan, context = frame['row'], frame['plan'], frame['context']
        self.assertTrue(ma.static_plan_complete(plan), plan)
        required = max(35., plan['safe_horizon_m'], plan['hold_until_m'])
        self.assertGreaterEqual(context['corridor'].clear_distance(plan['path'], required), required-.01)
        self.assertGreaterEqual(swept_clearance(plan['path'], context['points'], required, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), required-.01)
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], row['ego'],
            context['static'], required, body_shift_m=REAR_AXLE_SHIFT_M), required-.01)
        self.assertLessEqual(max(gui.path_curvatures(plan['path'])), gui.MAX_CONTROL_CURVATURE)
        self.assertGreater(plan['speed_cap_mps'], 0.)

    def test_next_raw_hazard_is_confirmed_before_the_nearer_semantic_target_passes(self):
        frames, _ = self.replay((199, 201))
        for number, expected in ((199, 1), (201, 2)):
            frame = frames[number]
            self.assertLess(frame['row']['info']['hazard_front_m'], 2.)
            self.assertEqual(len(frame['tracks']), 1)
            track = frame['tracks'][0]
            self.assertGreater(gui.local_xy(*track['obj'][:2], frame['row']['ego'])[0], 20.)
            self.assertEqual(track['observations'], expected)
            self.assertEqual(track['sequence'], frame['row']['proof']['road_scan_frame']['sequence'])

    def test_one_missing_physical_scan_cannot_start_the_recorded_premature_rejoin(self):
        frames, _ = self.replay((199, 201, 203, 205, 207, 209))
        measured = frames[203]['tracks'][0]
        self.assertEqual(frames[205]['diagnostic']['source_sequence'],
                         frames[207]['diagnostic']['source_sequence'])
        for number in (205, 207):
            frame = frames[number]
            self.assert_full_checks(frame)
            self.assertNotEqual(frame['plan']['kind'], 'rejoin')
            self.assertEqual(frame['diagnostic']['retained_n'], 1)
            self.assertEqual(frame['tracks'][0]['stamp'], measured['stamp'])
            self.assertEqual(frame['tracks'][0]['sequence'], measured['sequence'])
            self.assertEqual(frame['tracks'][0]['observations'], 2)
            retained = [obj for obj in frame['context']['static'] if len(obj) > 12
                        and isinstance(obj[12], dict) and MEMORY_KEY in obj[12]]
            self.assertEqual(len(retained), 1)
            np.testing.assert_array_equal(retained[0][:2], measured['obj'][:2])
            np.testing.assert_array_equal(retained[0][6:9], measured['obj'][6:9])
            self.assertAlmostEqual(retained[0][12][MEMORY_KEY]['age_s'],
                                   frame['row']['check_wall']-measured['stamp'])
        captured = frames[205]['row']['captured_plan']
        self.assertEqual(captured['kind'], 'rejoin')
        required = max(captured['safe_horizon_m'], captured['hold_until_m'])
        self.assertLess(gui.static_clear_distance(np.asarray(captured['path']),
            frames[205]['row']['ego'], frames[205]['context']['static'], required,
            body_shift_m=REAR_AXLE_SHIFT_M), required)
        self.assert_full_checks(frames[209])
        self.assertEqual(frames[209]['diagnostic']['retained_n'], 0)
        self.assertEqual(frames[209]['tracks'][0]['sequence'],
                         frames[209]['row']['proof']['road_scan_frame']['sequence'])

    def test_fresh_semantic_geometry_supersedes_the_retained_raw_target(self):
        frames, runtime = self.replay((199, 201, 203, 205, 207, 209, 211, 213))
        current = frames[213]
        self.assertEqual(current['diagnostic']['retained_n'], 1)
        self.assertTrue(any(obj[3] == 'fusion_obstacle' and obj[11] == 8
                            for obj in current['context']['static']))
        self.assertFalse(any(len(obj) > 12 and isinstance(obj[12], dict)
                             and MEMORY_KEY in obj[12] for obj in current['context']['static']))
        self.assertFalse(any(str(obj[3]).startswith('tunnel_lidar_')
                             for _, obj in runtime._static_object_memory))

    def test_reused_scan_cannot_extend_the_half_second_observation_expiry(self):
        frames, runtime = self.replay((199, 201, 203, 205, 207))
        stamp = frames[203]['tracks'][0]['stamp']
        memory = runtime._committed_road_raw_memory
        current = self.frame(209)
        for delta, expected in ((.499, 1), (.501, 0)):
            retained = memory.update([], current['ego'],
                proof=current['proof']['road_scan_frame'], now_wall=stamp+delta,
                active=True, committed=True)
            self.assertEqual(len(retained), expected)
            self.assertIsNone(memory.last_diagnostics['reason'])
            if retained:
                self.assertEqual(retained[0][12][MEMORY_KEY]['source_stamp'], stamp)

    def test_a_current_raw_box_outside_the_confirmed_horizon_is_not_remembered(self):
        row = self.frame(301)
        result, runtime = self.plan(row)
        self.assertGreater(runtime._model_avoidance_log['lidar_boxes_n'], 0)
        self.assertIsNone(runtime._committed_road_raw_memory.last_diagnostics['reason'])
        self.assertEqual(runtime._committed_road_raw_memory._tracks, [])
        self.assertTrue(any(str(obj[3]).startswith('tunnel_lidar_') for obj in result[2]['static']))

    def test_person_or_moving_traffic_revokes_raw_memory_ownership(self):
        for kind in ('pedestrian', 'npc'):
            with self.subTest(kind=kind):
                _, runtime = self.replay((199, 201))
                row = self.frame(203)
                row['objects'].append((1000., 1000., .6, 'person' if kind == 'pedestrian' else 'lidar_car',
                    kind, 2., .6, .6, 0., 2., 0., 999))
                self.plan(row, runtime)
                self.assertEqual(runtime._committed_road_raw_memory._tracks, [])
                self.assertEqual(runtime._committed_road_raw_memory.last_diagnostics['reason'],
                                 'outside ordinary committed static policy')


if __name__ == '__main__':
    unittest.main()
