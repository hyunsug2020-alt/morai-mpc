"""Fixed-map entry reference and state-independent semantic body regressions."""
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from gui import HdMapLanes, RosPathPublisher, smooth_control_route
from roundabout_empty import (EntryReference, STOP_STATION_M, certified_geometry,
                             semantic_entry_clearance, raw_entry_clearance, braking_reserve,
                             MAX_ENTRY_SOURCE_AGE_S, ROLL_MPS)
from stop_line import competition_stop_controller


class EntryReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).parent
        cls.route = np.asarray([[p['x'], p['y']] for p in
            json.loads((root/'global_path.json').read_text())['waypoints']])
        cls.lanes = HdMapLanes(root/'link_set.json')
        cls.lanes.set_route(cls.route)
        cls.control_route = smooth_control_route(cls.route)

    def fixture(self, speed=2.5):
        ego = dict(x=-99.404371236545, y=312.851757325084,
                   heading_deg=89.7081825492, speed_kmh=speed,
                   stamp=100., received=99.99, age_s=.01)
        yaw = math.radians(ego['heading_deg'])
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        nearest = int(self.lanes.route_tree.query([ego['x'], ego['y']])[1])
        # route_tensor supplies the smoothed control waypoints to this helper.
        world = self.control_route[nearest:nearest+120]
        local = (world-[ego['x'], ego['y']])@rot
        path = np.column_stack((local, np.arange(len(local))*.5, np.ones(len(local))))
        return ego, path, rot

    def test_selected_reference_preserves_inputs_and_metadata(self):
        ego, path, rot = self.fixture()
        before = path.copy()
        ref = EntryReference()
        changed = ref.apply(self.lanes, ego, path, enabled=True, now=100.)
        self.assertTrue(ref.active)
        np.testing.assert_array_equal(path, before)
        np.testing.assert_array_equal(changed[:, 2:], path[:, 2:].astype(np.float32))
        displacement = np.linalg.norm(changed[:, :2]-path[:, :2], axis=1)
        self.assertLessEqual(displacement.max(), .05001)
        self.assertGreater(displacement[0], .04999)
        control = RosPathPublisher.control_geometry(changed, changed, dict(mode='ROUTE BASE'))
        world = control[:, :2]@rot.T+[ego['x'], ego['y']]
        self.assertTrue(certified_geometry(self.lanes, ego, world))

    def test_speed_interruptions_keep_fixed_world_geometry(self):
        ego, path, rot = self.fixture()
        ref = EntryReference()
        first = ref.apply(self.lanes, ego, path, enabled=True, now=100.)
        interrupted = ref.apply(self.lanes, dict(ego, speed_kmh=5.), path,
                                enabled=True, now=100.)
        np.testing.assert_array_equal(interrupted, first)
        self.assertTrue(ref.active)

    def test_constant_speed_mode_retires_the_empty_roll_reference_at_the_recorded_stop(self):
        ego, path, _ = self.fixture()
        reference = EntryReference()
        reference.apply(self.lanes, ego, path, enabled=True, now=100.)
        self.assertTrue(reference.active)
        stopped = dict(ego, x=-98.8852207083503, y=318.51077340459744,
                       heading_deg=78.20883558805065, speed_kmh=0.)
        from roundabout_empty import certified_pose_geometry
        self.assertFalse(certified_pose_geometry(self.lanes, stopped))
        original = path.copy()
        result = reference.apply(self.lanes, stopped, original, enabled=True,
                                 now=100., constant_speed=True)
        self.assertIs(result, original)
        self.assertFalse(reference.active)
        self.assertIsNone(reference.last_station)

    def test_corner_correction_does_not_survive_the_reference_exit(self):
        ego, _, rotation = self.fixture()
        nearest = int(self.lanes.route_tree.query([ego['x'], ego['y']])[1])
        keep = np.flatnonzero(self.lanes.route_station >= EntryReference.HOLD_END_M+EntryReference.RETURN_M)
        world = self.control_route[keep[0]:keep[0]+80].copy()
        # A future smoothing deviation must remain an ordinary path at exit,
        # rather than gaining a correction that disappears on the next frame.
        world[0, 0] += .12
        local = (world-[ego['x'], ego['y']])@rotation
        reference = EntryReference(); reference.active = True
        active = reference.apply(self.lanes, ego, local, enabled=True, now=100.)
        with patch.object(self.lanes, 'route_progress',
                          return_value=reference.HOLD_END_M+reference.RETURN_M+.01):
            ended = reference.apply(self.lanes, ego, local, enabled=True, now=100.)
        self.assertFalse(reference.active)
        np.testing.assert_allclose(active, ended, atol=1e-5)

    def test_missing_stale_fast_far_or_other_map_does_not_start(self):
        for changes in ({'speed_kmh':3.01}, {'age_s':.201}, {'stamp':0.},
                        {'x':math.nan}, {'received':99.799}, {'received':100.01},
                        {'received':math.nan}, {'age_s':-.01}):
            ego, path, _ = self.fixture()
            ref = EntryReference()
            with self.subTest(changes=changes):
                result = ref.apply(self.lanes, dict(ego, **changes), path, enabled=True, now=100.)
                self.assertIs(result, path)
                self.assertFalse(ref.active)
        ego, path, _ = self.fixture()
        missing_receipt = dict(ego)
        del missing_receipt['received']
        self.assertIs(EntryReference().apply(self.lanes, missing_receipt, path,
                      enabled=True, now=100.), path)
        ref = EntryReference()
        self.assertIs(ref.apply(self.lanes, ego, path, enabled=False, now=100.), path)
        with patch('roundabout_empty.certified_pose_geometry', return_value=False):
            self.assertIs(ref.apply(self.lanes, ego, path, enabled=True, now=100.), path)
        with patch.object(self.lanes, 'route_progress', return_value=STOP_STATION_M-30.):
            self.assertIs(ref.apply(self.lanes, ego, path, enabled=True, now=100.), path)

    def test_disabled_finished_or_rewound_entry_resets_reference(self):
        for reason in ('disabled', 'finished', 'rewind'):
            ego, path, _ = self.fixture()
            ref = EntryReference()
            ref.apply(self.lanes, ego, path, enabled=True, now=100.)
            with self.subTest(reason=reason):
                if reason == 'finished':
                    with patch.object(self.lanes, 'route_progress', return_value=STOP_STATION_M+12.1):
                        result = ref.apply(self.lanes, ego, path, enabled=True, now=100.)
                elif reason == 'disabled':
                    result = ref.apply(self.lanes, ego, path, enabled=False, now=100.)
                else:
                    # A clock reset requires a new current observation; a fast
                    # rewound frame cannot carry the old low-speed selection.
                    result = ref.apply(self.lanes, dict(ego, stamp=99., speed_kmh=10.),
                                       path, enabled=True, now=100.)
                self.assertIs(result, path)
                self.assertFalse(ref.active)

    def test_reference_finishes_after_its_smooth_return(self):
        ego, path, _ = self.fixture()
        ref = EntryReference()
        ref.apply(self.lanes, ego, path, enabled=True, now=100.)
        with patch.object(self.lanes, 'route_progress',
                          return_value=ref.HOLD_END_M+ref.RETURN_M+.01):
            self.assertIs(ref.apply(self.lanes, ego, path, enabled=True, now=100.), path)
        self.assertFalse(ref.active)

    def test_pose_jitter_does_not_toggle_selected_reference_or_its_speed_contract(self):
        ego, path, _ = self.fixture()
        ref = EntryReference()
        ref.apply(self.lanes, ego, path, enabled=True, now=100.)
        self.assertTrue(ref.active)
        # Native run 12 reached this pose after the ENTER cap jumped to 30
        # km/h. Its measured 0.262 m cross-track error exceeds the 0.2 m bound.
        stopped = dict(ego, x=-99.05271183090376, y=317.72652660205625,
                       heading_deg=80.87356626288846, speed_kmh=0.)
        result = ref.apply(self.lanes, stopped, path, enabled=True, now=100.)
        self.assertIsNot(result, path)
        self.assertTrue(ref.active)

    def test_native_smoothed_corner_uses_only_the_remaining_map_offset_room(self):
        root = Path(__file__).parent
        with np.load(root/'test_data/roundabout_reference_boundary_20261013.npz') as data:
            source, old_world = data['ego'], data['world_path']
        ego = dict(x=source[1], y=source[2], heading_deg=math.degrees(source[3]),
                   speed_kmh=source[4]*3.6, stamp=100., received=99.99, age_s=.01)
        c, s = math.cos(source[3]), math.sin(source[3])
        rotation = np.array([[c, -s], [s, c]])
        _, nearest = self.lanes.route_tree.query(old_world)
        tangent = np.gradient(self.route, axis=0)[nearest]
        right = np.column_stack((tangent[:, 1], -tangent[:, 0]))/np.linalg.norm(tangent, axis=1)[:, None]
        t = np.clip((self.lanes.route_station[nearest]-EntryReference.HOLD_END_M)/EntryReference.RETURN_M, 0., 1.)
        old_amount = EntryReference.OFFSET_M*(1.-t**3*(10.-15.*t+6.*t*t))
        base = (old_world-old_amount[:, None]*right-source[1:3])@rotation
        reference = EntryReference(); reference.active = True
        shifted = reference.apply(self.lanes, ego, base, enabled=True, now=100.)
        world = shifted@rotation.T+source[1:3]
        self.assertEqual(raw_entry_clearance(self.lanes, ego, np.empty((0, 2)), old_world), 0.)
        self.assertGreater(raw_entry_clearance(self.lanes, ego, np.empty((0, 2)), world), 0.)
        self.assertEqual(raw_entry_clearance(self.lanes, ego, [[1., 0.]], world), 0.)

    def test_native_box_and_checked_reference_keep_the_collision_guards(self):
        ego, path, rot = self.fixture()
        changed = EntryReference().apply(self.lanes, ego, path, enabled=True, now=100.)
        control = RosPathPublisher.control_geometry(changed, changed,
            dict(mode='ROUTE BASE', canonical_control=False))
        world = control[:, :2]@rot.T+[ego['x'], ego['y']]
        original = (-101.743206163536, 316.1936736479171, 6.337804794311523,
            'fusion_obstacle', 'obstacle', 0., 6.337804794311523, 1.9469873905181885,
            89.70578375653446, 0., 0., 40)
        refit = (-101.9741203, 316.2538691, 6.3775695,
            'fusion_obstacle', 'obstacle', 0., 6.3775695, 1.9297863,
            84.759212, 0., 0., 40)
        self.assertEqual(semantic_entry_clearance(control, ego, [original]), 0.)
        self.assertGreater(semantic_entry_clearance(control, ego, [refit]),
                           braking_reserve(2.5/3.6, MAX_ENTRY_SOURCE_AGE_S))
        self.assertTrue(certified_geometry(self.lanes, ego, world))
        for kmh in (0., 2.5, ROLL_MPS*3.6):
            current = dict(ego, speed_kmh=kmh)
            self.assertEqual(raw_entry_clearance(self.lanes, current, np.empty((0, 2)), world),
                             braking_reserve(kmh/3.6, MAX_ENTRY_SOURCE_AGE_S))
            # A real current body return always defeats the selected candidate.
            self.assertEqual(raw_entry_clearance(self.lanes, current, [[1., 0.]], world), 0.)


if __name__ == '__main__':
    unittest.main()
