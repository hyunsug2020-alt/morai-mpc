"""Atomic fusion geometry through reception, planning and fresh road guards."""
import copy
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from perception.static_geometry_contract import (CONTRACT_NAME, MAX_SOURCE_AGE_S,
    STORED_PROOF_KEY, validate_stored_static_geometry)
from perception.stationary_semantics import metadata, with_metadata
from perception.road_static_calibration import RoadStaticCalibration


class AtomicRoadGeometryPipeline(unittest.TestCase):
    def setUp(self):
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.,
                        stamp=100.1, measured=True)
        self.path = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))
        self.unassociated = np.array([[50., 8.]])
        self.receiver = gui.RosObjectReceiver.__new__(gui.RosObjectReceiver)
        self.receiver.lock = threading.Lock()
        self.receiver.objects, self.receiver.received = [], 0.
        self.receiver._tracks, self.receiver._motion = {}, {}
        self.receiver._turn, self.receiver._position_history = {}, {}
        self.receiver.file_cache = SimpleNamespace(value={})
        self.clock = 100.1
        self.receiver._source_clock = lambda: self.clock
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    def receive(self, *, name=CONTRACT_NAME, x=10., y=1.65, speed=0.,
                kind='obstacle', object_type=2, stamp=100., mono=1000.):
        obj = SimpleNamespace(unique_id=9, name=name, type=object_type, heading=0.,
            position=SimpleNamespace(x=x, y=y),
            size=SimpleNamespace(x=1.1, y=1.1, z=.7),
            velocity=SimpleNamespace(x=speed*3.6, y=0.))
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)),
            npc_list=[], pedestrian_list=[], obstacle_list=[])
        getattr(message, kind+'_list').append(obj)
        with patch('gui.time.monotonic', return_value=mono):
            self.receiver.on_objects(message)
        return self.receiver.objects[0]

    def normalize(self, obj, *, ego=None, points=None, enabled=True):
        pose = self.ego if ego is None else ego
        source = ma.validate_road_geometry_sources([obj], pose)
        stationary, _ = ma.split_static(source)
        return ma.road_static_objects(stationary, pose,
            self.unassociated if points is None else points,
            allow_producer_geometry=enabled)[0]

    def test_atomic_receiver_preserves_padded_source_and_truthful_point_counts(self):
        source = self.receive()
        self.assertEqual(source[3], 'fusion_obstacle')
        self.assertEqual(source[6:8], (1.1, 1.1))
        proof = metadata(source)['producer_static_geometry']
        self.assertEqual(proof['source_stamp'], 100.)
        self.assertEqual(proof['source_points_min'], 4)
        for points in (self.unassociated, np.empty((0, 2))):
            with self.subTest(points_n=len(points)):
                checked = self.normalize(source, points=points)
                np.testing.assert_allclose(checked[6:8], (.9, .9), atol=1e-12)
                geometry = metadata(checked)['road_static_geometry']
                self.assertEqual(geometry['observed_points'], 0)
                self.assertTrue(geometry['producer_geometry'])
        self.assertEqual(self.normalize(source, enabled=False)[6:8], (1.1, 1.1))

    def test_legacy_unknown_type_people_and_motion_do_not_gain_producer_permission(self):
        for changes in (dict(name='fusion_obstacle'), dict(name='fusion_obstacle_padded_v2'),
                        dict(object_type=1), dict(speed=.01), dict(kind='pedestrian')):
            with self.subTest(changes=changes):
                self.setUp()
                source = self.receive(**changes)
                self.assertNotIn('producer_static_geometry', metadata(source))
                if source[4] == 'obstacle':
                    self.assertEqual(self.normalize(source)[6:8], (1.1, 1.1))

    def test_changed_or_stale_raw_proof_is_revoked_before_static_motion_is_zeroed(self):
        source = self.receive()
        for index, value in ((0, 10.01), (5, .01), (9, .01), (10, float('nan')),
                             (6, 1.2), (8, .01), (11, 10)):
            with self.subTest(index=index):
                changed = list(source)
                changed[index] = value
                validated = ma.validate_road_geometry_sources([tuple(changed)], self.ego)[0]
                self.assertIsNone(metadata(validated).get('producer_static_geometry'))
                static, _ = ma.split_static([validated])
                self.assertEqual(static[0][5], 0.)
                checked = ma.road_static_objects(static, self.ego, self.unassociated,
                                                 allow_producer_geometry=True)[0]
                self.assertNotIn('road_static_geometry', metadata(checked))
        for stamp in (100.+MAX_SOURCE_AGE_S+.001, 99.949):
            with self.subTest(stamp=stamp):
                checked = self.normalize(source, ego=dict(self.ego, stamp=stamp))
                self.assertEqual(checked[6:8], (1.1, 1.1))
        self.assertEqual(self.normalize(source, ego=dict(self.ego, measured=False))[6:8], (1.1, 1.1))

    def test_repeated_receipt_does_not_extend_original_source_clock(self):
        self.receive()
        self.clock = 100.2
        self.receive(stamp=100., mono=1000.2)
        self.assertEqual(self.receiver.received, 1000.)
        source = self.receiver.objects[0]
        self.assertEqual(metadata(source)['producer_static_geometry']['source_stamp'], 100.)
        checked = self.normalize(source, ego=dict(self.ego, stamp=100.+MAX_SOURCE_AGE_S+.001))
        self.assertEqual(checked[6:8], (1.1, 1.1))

    def test_current_proof_supports_current_pose_recovery_and_strict_source_preference(self):
        source = self.receive(x=3., y=-1.55)
        checked = self.normalize(source)
        geometry = metadata(checked)['road_static_geometry']
        self.assertEqual(geometry['observed_points'], 0)
        self.assertAlmostEqual(geometry['initial_margin_floor_m'], .154)
        preferred, = ma._road_rejoin_preference_objects([checked], self.ego, self.path,
                                                       include_near=True)
        self.assertEqual(preferred[6:8], (1.1, 1.1))
        for key in ('initial_margin_floor_m', 'initial_margin_restore_start_m'):
            self.assertNotIn(key, metadata(preferred)['road_static_geometry'])
        self.assertEqual(gui.static_clear_distance(self.path, self.ego, [preferred], 35.,
                         body_shift_m=REAR_AXLE_SHIFT_M), 0.)
        stale = dict(self.ego, stamp=100.+MAX_SOURCE_AGE_S+.001)
        self.assertEqual(ma._road_rejoin_preference_objects([checked], stale, self.path,
                                                           include_near=True), [])
        recovered = ma.road_static_margin_recovery([checked], stale, self.unassociated)[0]
        self.assertNotIn('initial_margin_floor_m', metadata(recovered)['road_static_geometry'])

    def test_producer_physical_memory_uses_the_existing_ttl_without_renewing_source_time(self):
        host = gui.Runtime.__new__(gui.Runtime)
        checked = self.normalize(self.receive())
        original = copy.deepcopy(checked)
        current = host.remember_static_objects([checked], 1000., True)
        self.assertEqual(current[0], original)
        with patch.object(gui, 'STATIC_OBJECT_MEMORY_S', 3.):
            for now in (1000.1, 1000.4, 1001.2, 1002.999, 1003.):
                remembered, = host.remember_static_objects([], now, True)
                np.testing.assert_allclose(remembered[6:8], (.9, .9), atol=1e-12)
                self.assertEqual(metadata(remembered)['road_static_geometry']['observed_points'], 0)
                self.assertEqual(metadata(remembered)['producer_static_geometry']['source_stamp'], 100.)
                self.assertEqual(host._static_object_memory[0][0], 1000.)
            self.assertEqual(host.remember_static_objects([], 1003.000001, True), [])
            self.assertEqual(host._static_object_memory, [])
        # A genuinely associated current return keeps the existing memory policy.
        raw_checked = self.normalize(self.receiver.objects[0], points=np.array([[10., 1.65]]))
        host = gui.Runtime.__new__(gui.Runtime)
        host.remember_static_objects([raw_checked], 1000., True)
        raw_memory = host.remember_static_objects([], 1000.1, True)[0]
        self.assertEqual(metadata(raw_memory)['road_static_geometry']['observed_points'], 1)
        np.testing.assert_allclose(raw_memory[6:8], (.9, .9), atol=1e-12)

    def test_retained_physical_bounds_recompute_pose_floors_without_fresh_producer_permissions(self):
        source = self.receive(x=3., y=-1.55)
        checked = self.normalize(source)
        self.assertIn('initial_margin_floor_m', metadata(checked)['road_static_geometry'])
        host = gui.Runtime.__new__(gui.Runtime)
        host.remember_static_objects([checked], 1000., True, measured_stamp=self.ego['stamp'])
        remembered, = host.remember_static_objects([], 1000.4, True)
        geometry = metadata(remembered)['road_static_geometry']
        self.assertNotIn('initial_margin_floor_m', geometry)
        self.assertNotIn('initial_margin_restore_start_m', geometry)
        self.assertEqual(geometry['observed_points'], 0)
        marker = metadata(remembered)[STORED_PROOF_KEY]
        self.assertEqual(marker[2], 100.)
        stale = dict(self.ego, stamp=100.4, x=.01)
        self.assertEqual(ma._road_rejoin_preference_objects([remembered], stale, self.path,
                                                           include_near=True), [])
        recovered = ma.road_static_margin_recovery([remembered], stale, self.unassociated)[0]
        self.assertAlmostEqual(metadata(recovered)['road_static_geometry']['initial_margin_floor_m'], .154)
        self.assertAlmostEqual(metadata(recovered)['road_static_geometry']['initial_margin_restore_start_m'], 4.43)
        self.assertNotIn('initial_margin_floor_m', metadata(remembered)['road_static_geometry'])
        np.testing.assert_allclose(recovered[6:8], (.9, .9), atol=1e-12)
        # An expired source cannot normalize a new observation, even though
        # the existing stationary physical estimate still lives in memory.
        self.assertEqual(self.normalize(source, ego=stale)[6:8], (1.1, 1.1))
        cache = RoadStaticCalibration()
        proof = dict(valid=True, stamp=100.4, sequence=1, source_pose=[100.4, .01, 0., 0.],
            target_pose=[100.4, .01, 0., 0.], age_s=0., points_n=len(self.unassociated))
        cache.update([source], [remembered], stale, self.unassociated, proof=proof,
                     now_wall=100.4, active=True, retained_near_field_n=0)
        self.assertEqual(cache._tracks, {})
        self.assertEqual(cache.last_diagnostics['calibrated_n'], 0)
        self.assertEqual(metadata(remembered)['producer_static_geometry']['source_stamp'], 100.)

    def test_history_marker_is_storage_only_and_cannot_be_minted_from_expired_or_unmeasured_input(self):
        checked = self.normalize(self.receive(x=3., y=-1.55))
        for stamp in (None, 100.301):
            host = gui.Runtime.__new__(gui.Runtime)
            returned, = host.remember_static_objects([checked], 1000., True, measured_stamp=stamp)
            self.assertNotIn(STORED_PROOF_KEY, metadata(returned))
            remembered, = host.remember_static_objects([], 1000.4, True)
            self.assertNotIn(STORED_PROOF_KEY, metadata(remembered))
            recovered = ma.road_static_margin_recovery([remembered],
                dict(self.ego, stamp=100.4), self.unassociated)[0]
            self.assertNotIn('initial_margin_floor_m', metadata(recovered)['road_static_geometry'])
        host = gui.Runtime.__new__(gui.Runtime)
        returned, = host.remember_static_objects([checked], 1000., True, measured_stamp=100.1)
        self.assertNotIn(STORED_PROOF_KEY, metadata(returned))
        stored = host._static_object_memory[0][1]
        self.assertIn(STORED_PROOF_KEY, metadata(stored))

    def test_stored_history_age_and_current_raw_collision_keep_original_recovery_boundaries(self):
        host = gui.Runtime.__new__(gui.Runtime)
        source = self.receive(x=3., y=-1.55)
        host.remember_static_objects([self.normalize(source)], 1000., True, measured_stamp=100.1)
        remembered, = host.remember_static_objects([], 1000.4, True)
        marker = metadata(remembered)[STORED_PROOF_KEY]
        for stamp in (100.4, 102.999, 103.):
            recovered = ma.road_static_margin_recovery([remembered],
                dict(self.ego, stamp=stamp), self.unassociated)[0]
            self.assertAlmostEqual(metadata(recovered)['road_static_geometry']['initial_margin_floor_m'], .154)
            self.assertEqual(metadata(recovered)[STORED_PROOF_KEY], marker)
            self.assertEqual(metadata(recovered)['road_static_geometry']['observed_points'], 0)
        for ego, points in ((dict(self.ego, stamp=103.00001), self.unassociated),
                            (dict(self.ego, stamp=99.949), self.unassociated),
                            (dict(self.ego, stamp=100.4, measured=False), self.unassociated),
                            (dict(self.ego, stamp=100.4, y=-.1), self.unassociated),
                            (dict(self.ego, stamp=100.4), np.array([[3.8, 0.]]))):
            recovered = ma.road_static_margin_recovery([remembered], ego, points)[0]
            self.assertNotIn('initial_margin_floor_m', metadata(recovered)['road_static_geometry'])
        changed = list(remembered)
        changed[5], changed[9] = .01, .01
        validated, = ma.validate_road_geometry_sources([tuple(changed)], dict(self.ego, stamp=100.4))
        stationary, _ = ma.split_static([validated])
        recovered, = ma.road_static_margin_recovery(stationary, dict(self.ego, stamp=100.4), self.unassociated)
        self.assertNotIn('initial_margin_floor_m', metadata(recovered)['road_static_geometry'])
        self.assertIsNone(validate_stored_static_geometry(marker, stationary[0], 100.4))

    def test_history_recovery_never_authorizes_a_path_that_approaches_the_same_body(self):
        host = gui.Runtime.__new__(gui.Runtime)
        host.remember_static_objects([self.normalize(self.receive(x=3., y=-1.55))],
            1000., True, measured_stamp=100.1)
        remembered, = host.remember_static_objects([], 1000.4, True)
        recovered, = ma.road_static_margin_recovery([remembered], dict(self.ego, stamp=100.4), self.unassociated)
        self.assertAlmostEqual(gui.static_clear_distance(self.path, self.ego, [recovered], 35.,
            body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        approaches = self.path.copy()
        approaches[:, 1] = -.04*np.minimum(approaches[:, 0], 1.)
        self.assertLess(gui.static_clear_distance(approaches, self.ego, [recovered], 35.,
            body_shift_m=REAR_AXLE_SHIFT_M), 1.)

    def stored_margin_box(self):
        host = gui.Runtime.__new__(gui.Runtime)
        host.remember_static_objects([self.normalize(self.receive(x=3., y=-1.55))],
            1000., True, measured_stamp=100.1)
        return host._static_object_memory[0][1]

    def test_episode_anchor_allows_four_mm_but_never_recharges_five_mm_per_frame(self):
        obj, ledger = self.stored_margin_box(), {}
        first, = ma.road_static_margin_recovery([obj], self.ego, self.unassociated,
                                               recovery_ledger=ledger)
        anchor = ledger[9]
        self.assertAlmostEqual(anchor, .154-.0049)
        self.assertAlmostEqual(metadata(first)['road_static_geometry']['initial_margin_floor_m'], anchor)
        for approach in (.00399, .005-1e-7):
            checked, = ma.road_static_margin_recovery([first],
                dict(self.ego, y=-approach), self.unassociated, recovery_ledger=ledger)
            self.assertEqual(metadata(checked)['road_static_geometry']['initial_margin_floor_m'], anchor)
        for approach in (.005+1e-7, .01, .015):
            checked, = ma.road_static_margin_recovery([first],
                dict(self.ego, y=-approach), self.unassociated, recovery_ledger=ledger)
            self.assertNotIn('initial_margin_floor_m', metadata(checked)['road_static_geometry'])
            self.assertEqual(ledger, {9: anchor})
        # Without the explicit Runtime opt-in the original exact floor remains.
        exact, = ma.road_static_margin_recovery([obj], self.ego, self.unassociated)
        self.assertAlmostEqual(metadata(exact)['road_static_geometry']['initial_margin_floor_m'], .154)

    def test_raw_observations_never_start_an_anchor_but_cannot_recharge_an_existing_episode(self):
        producer = self.stored_margin_box()
        raw = copy.deepcopy(producer)
        metadata(raw)['road_static_geometry']['observed_points'] = 1
        legacy = self.normalize(self.receive(name='fusion_obstacle', x=3., y=-1.55,
                                            stamp=100.01, mono=1000.01),
                                points=np.array([[3., -1.95]]))
        self.assertNotIn('producer_static_geometry', metadata(legacy))
        for obj in (raw, legacy):
            ledger = {}
            checked, = ma.road_static_margin_recovery([obj], self.ego, self.unassociated,
                                                     recovery_ledger=ledger)
            self.assertAlmostEqual(metadata(checked)['road_static_geometry']['initial_margin_floor_m'], .154)
            self.assertEqual(ledger, {})
        ledger = {}
        ma.road_static_margin_recovery([producer], self.ego, self.unassociated, recovery_ledger=ledger)
        anchor = ledger[9]
        checked, = ma.road_static_margin_recovery([raw], dict(self.ego, y=-.004),
            self.unassociated, recovery_ledger=ledger)
        self.assertEqual(metadata(checked)['road_static_geometry']['initial_margin_floor_m'], anchor)
        for approach in (.005+1e-7, .01, .015):
            checked, = ma.road_static_margin_recovery([raw], dict(self.ego, y=-approach),
                self.unassociated, recovery_ledger=ledger)
            self.assertNotIn('initial_margin_floor_m', metadata(checked)['road_static_geometry'])
            self.assertEqual(ledger, {9: anchor})

    def test_anchor_survives_fresh_source_geometry_and_missing_proof_until_real_clearance(self):
        obj, ledger = self.stored_margin_box(), {}
        ma.road_static_margin_recovery([obj], self.ego, self.unassociated, recovery_ledger=ledger)
        anchor = ledger[9]
        changed = copy.deepcopy(list(obj))
        changed[0] += .01
        proof = changed[17]['producer_static_geometry']
        proof['source_stamp'] = 100.4
        signature = list(proof['signature'])
        signature[2] = (changed[0], changed[1])
        proof['signature'] = tuple(signature)
        changed = tuple(changed)
        resized = copy.deepcopy(list(changed))
        resized[1], resized[2], resized[7] = -1.6, 1., 1.
        shape = resized[17]['road_static_geometry']
        shape['physical_size_m'], shape['source_size_m'] = [.8, .9], [1.1, 1.2]
        resized[17]['raw_size_m'] = [1.1, 1.2]
        proof = resized[17]['producer_static_geometry']
        proof['physical_size_m'] = [.8, .9]
        signature = list(proof['signature'])
        signature[2], signature[3] = (resized[0], resized[1]), (1.1, 1.2)
        proof['signature'] = tuple(signature)
        resized = tuple(resized)
        for row in (obj, changed, resized,
                    with_metadata(obj, producer_static_geometry=None), changed):
            checked, = ma.road_static_margin_recovery([row],
                dict(self.ego, stamp=100.5, y=-.01), self.unassociated, recovery_ledger=ledger)
            self.assertNotIn('initial_margin_floor_m', metadata(checked)['road_static_geometry'])
            self.assertEqual(ledger, {9: anchor})
        # Proof expiry cannot erase the episode either.
        ma.road_static_margin_recovery([obj], dict(self.ego, stamp=103.001),
            self.unassociated, recovery_ledger=ledger)
        self.assertEqual(ledger, {9: anchor})
        ma.road_static_margin_recovery([obj], dict(self.ego, y=.046),
            self.unassociated, recovery_ledger=ledger, read_only=True)
        self.assertEqual(ledger, {9: anchor})
        ma.road_static_margin_recovery([obj], dict(self.ego, y=.046),
            self.unassociated, recovery_ledger=ledger)
        self.assertEqual(ledger, {})
        ma.road_static_margin_recovery([obj], dict(self.ego, y=-.01),
            self.unassociated, recovery_ledger=ledger)
        self.assertAlmostEqual(ledger[9], .144-.0049)

    def test_anchor_capacity_invalid_uid_duplicates_and_raw_body_guards_fail_conservatively(self):
        obj = self.stored_margin_box()
        full = {uid: .15 for uid in range(20, 84)}
        checked, = ma.road_static_margin_recovery([obj], self.ego, self.unassociated,
                                               recovery_ledger=full)
        self.assertEqual(len(full), 64)
        self.assertNotIn(9, full)
        self.assertAlmostEqual(metadata(checked)['road_static_geometry']['initial_margin_floor_m'], .154)
        for uid in (True, np.bool_(True), 9.5, float('nan'), float('inf'), -1):
            row = list(obj)
            row[11] = uid
            ledger = {}
            ma.road_static_margin_recovery([tuple(row)], self.ego, self.unassociated,
                                           recovery_ledger=ledger)
            self.assertEqual(ledger, {})
        ledger = {9: .1491}
        far = list(obj)
        far[1] = -2.
        for rows in ([obj, tuple(far)], [tuple(far), obj]):
            checked = ma.road_static_margin_recovery(rows, self.ego, self.unassociated,
                                                     recovery_ledger=ledger)
            self.assertEqual(ledger, {9: .1491})
            self.assertTrue(all('initial_margin_floor_m' not in metadata(r)['road_static_geometry'] for r in checked))
        for ego, points in ((dict(self.ego, y=-.035), self.unassociated),
                            (dict(self.ego, y=-.3), self.unassociated),
                            (self.ego, np.array([[3.8, 0.]]))):
            checked, = ma.road_static_margin_recovery([obj], ego, points, recovery_ledger=ledger)
            self.assertNotIn('initial_margin_floor_m', metadata(checked)['road_static_geometry'])
            self.assertEqual(ledger, {9: .1491})

    def test_fresh_guard_consults_existing_anchors_read_only_across_frames(self):
        source = self.receive(x=3., y=-1.55)
        obj, ledger = self.normalize(source), {}
        ma.road_static_margin_recovery([obj], self.ego, self.unassociated, recovery_ledger=ledger)
        original = dict(ledger)
        path = self.path.copy()
        path[:, 1] = -.00025*path[:, 0]
        self.assertIsNotNone(gui.fresh_road_static_hazard([source], self.ego, path, 25., 1.15,
            self.unassociated, enabled=True)[0])
        for pose in (self.ego, dict(self.ego, y=-.003, stamp=100.2)):
            hit, _ = gui.fresh_road_static_hazard([source], pose, path, 25., 1.15,
                self.unassociated, enabled=True, margin_recovery_state=ledger)
            self.assertIsNone(hit)
            self.assertEqual(ledger, original)
        hit, _ = gui.fresh_road_static_hazard([source], dict(self.ego, y=-.006), path,
            25., 1.15, self.unassociated, enabled=True, margin_recovery_state=ledger)
        self.assertIsNotNone(hit)
        self.assertEqual(ledger, original)
        empty = {}
        self.assertIsNotNone(gui.fresh_road_static_hazard([source], self.ego, path, 25., 1.15,
            self.unassociated, enabled=True, margin_recovery_state=empty)[0])
        self.assertEqual(empty, {})
        gui.fresh_road_static_hazard([source], dict(self.ego, y=.1), path, 25., 1.15,
            self.unassociated, enabled=True, margin_recovery_state=ledger)
        self.assertEqual(ledger, original)
        for pose, points in ((dict(self.ego, stamp=100.301), self.unassociated),
                             (self.ego, np.array([[3., -2.05]]))):
            hit, _ = gui.fresh_road_static_hazard([source], pose, path, 25., 1.15,
                points, enabled=True, margin_recovery_state=ledger)
            self.assertIsNotNone(hit)
            self.assertEqual(ledger, original)

    def test_new_current_growth_and_changed_motion_replace_old_stationary_geometry(self):
        source = self.receive()
        initial = self.normalize(source)
        larger = list(source)
        larger[2], larger[6], larger[7] = 1.3, 1.3, 1.3
        larger = with_metadata(tuple(larger), raw_size_m=[1.3, 1.3])
        moving = list(source)
        moving[5], moving[9] = .01, .01
        for current in (larger, tuple(moving)):
            with self.subTest(size=current[6:8], motion=current[5]):
                host = gui.Runtime.__new__(gui.Runtime)
                host.remember_static_objects([initial], 1000., True, measured_stamp=100.1)
                checked = self.normalize(current)
                self.assertNotIn('road_static_geometry', metadata(checked))
                self.assertIsNone(metadata(checked).get('producer_static_geometry'))
                host.remember_static_objects([checked], 1000.1, True, [current])
                remembered, = host.remember_static_objects([], 1000.2, True)
                self.assertEqual(remembered[6:8], current[6:8])
                self.assertNotIn(STORED_PROOF_KEY, metadata(remembered))
        # New actual returns can grow the current physical estimate and replace
        # memory, while a current person still removes the overlapping old box.
        host = gui.Runtime.__new__(gui.Runtime)
        host.remember_static_objects([initial], 1000., True, measured_stamp=100.1)
        grown = self.normalize(source, points=np.array([[10.5, 1.65]]))
        host.remember_static_objects([grown], 1000.1, True, [source])
        remembered, = host.remember_static_objects([], 1000.2, True)
        np.testing.assert_allclose(remembered[6:8], (1.1, .9), atol=1e-12)
        person = list(source)
        person[3:5], person[11] = ['fusion_pedestrian', 'pedestrian'], 12
        self.assertEqual(host.remember_static_objects([], 1000.3, True, [tuple(person)]), [])

    def test_current_raw_sweep_still_blocks_beside_a_remembered_physical_box(self):
        host = gui.Runtime.__new__(gui.Runtime)
        host.remember_static_objects([self.normalize(self.receive())], 1000., True)
        remembered, = host.remember_static_objects([], 1001., True)
        self.assertEqual(metadata(remembered)['road_static_geometry']['observed_points'], 0)
        corridor = LaneCorridor(self.path, np.full(len(self.path), 3.5), np.full(len(self.path), -3.5))
        _, clearance = ma.road_static_checks(corridor, np.array([[3.8, 0.]]))
        self.assertEqual(clearance(self.path, 35.), 0.)

    def test_current_raw_growth_and_independent_front_collision_remain_authoritative(self):
        source = self.receive()
        checked = self.normalize(source, points=np.array([[10.5, 1.65]]))
        np.testing.assert_allclose(checked[6:8], (1.1, .9), atol=1e-12)
        self.assertEqual(metadata(checked)['road_static_geometry']['observed_points'], 1)
        overflow = self.normalize(source, points=np.array([[10.55, 1.65]]))
        self.assertEqual(overflow[6:8], (1.1, 1.1))
        self.assertNotIn('road_static_geometry', metadata(overflow))
        corridor = LaneCorridor(self.path, np.full(len(self.path), 3.5), np.full(len(self.path), -3.5))
        _, clearance = ma.road_static_checks(corridor, np.array([[3.8, 0.]]))
        self.assertEqual(clearance(self.path, 35.), 0.)
        front = self.receive(y=0., stamp=100.01, mono=1000.01)
        hit, info = gui.fresh_road_static_hazard([front], self.ego, self.path, 25., 1.15,
                                               self.unassociated, enabled=True)
        self.assertIsNotNone(hit)
        self.assertLess(info['clear_m'], 10.)

    def test_fresh_guard_and_outside_scope_share_the_same_received_source(self):
        source = self.receive()
        expected = gui.route_hazard([source], self.ego, self.path, 25., 1.15)
        self.assertIsNotNone(expected)
        hit, info = gui.fresh_road_static_hazard([source], self.ego, self.path, 25., 1.15,
                                               self.unassociated, enabled=True)
        self.assertIsNone(hit)
        self.assertEqual(info['normalized_n'], 1)
        hit, info = gui.fresh_road_static_hazard([source], self.ego, self.path, 25., 1.15,
                                               self.unassociated, enabled=False)
        self.assertEqual(hit, expected)
        self.assertFalse(info['active'])

    def test_producer_only_normalization_cannot_issue_a_new_raw_calibration(self):
        source = self.receive()
        cache = RoadStaticCalibration()
        for sequence, now in enumerate((100.1, 100.2), 1):
            ego = dict(self.ego, stamp=now)
            checked = self.normalize(source, ego=ego)
            proof = dict(valid=True, stamp=now, sequence=sequence,
                source_pose=[now, 0., 0., 0.], target_pose=[now, 0., 0., 0.],
                age_s=0., points_n=len(self.unassociated))
            result = cache.update([source], [checked], ego, self.unassociated,
                proof=proof, now_wall=now, active=True, retained_near_field_n=0)[0]
            self.assertEqual(metadata(result)['road_static_geometry']['observed_points'], 0)
            self.assertEqual(cache._tracks, {})
            self.assertEqual(cache.last_diagnostics['calibrated_n'], 0)

    def test_file_fallback_restores_the_name_without_granting_missing_provenance(self):
        source = self.receive()
        for file_row in (list(source[:12]), copy.deepcopy(list(source))):
            with self.subTest(tuple_fields=len(file_row)):
                file_row[3] = CONTRACT_NAME
                self.receiver.received = 999.
                self.receiver.file_cache.value = dict(t=100.1, source_stamp=100., source_lag_s=.1,
                                                       objects=[file_row])
                with patch('gui.time.monotonic', return_value=1000.), \
                        patch('gui.time.time', return_value=100.1):
                    received, = self.receiver.snapshot()
                self.assertEqual(self.receiver.src, 'file')
                self.assertEqual(received[3], 'fusion_obstacle')
                self.assertNotIn('producer_static_geometry', metadata(received))
                self.assertEqual(self.normalize(received)[6:8], (1.1, 1.1))
                if len(file_row) > 17:
                    self.assertEqual(metadata(received)['height_m'], .7)

    def test_runtime_producer_permission_is_limited_to_the_normal_static_scope(self):
        import test_avoidance_gate as fixtures
        source = self.receive()
        person = list(source)
        person[3:5] = ['fusion_pedestrian', 'pedestrian']
        person[11] = 12
        for scope in ('normal', 'outside', 'tunnel', 'person'):
            with self.subTest(scope=scope):
                objects = [source]+([tuple(person)] if scope == 'person' else [])
                runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=objects)
                runtime._contact = None
                if scope == 'tunnel':
                    runtime.tunnel = SimpleNamespace(active=lambda ego: True)
                corridor = runtime.lanes.corridor(self.ego, self.path)
                lane, clearance = ma.road_static_checks(corridor, self.unassociated)
                with patch.object(gui, 'model_avoidance', ma, create=True), \
                        patch('static_lane_pass.normal_static_area', return_value=scope != 'outside'), \
                        patch.object(ma, 'plan_bypass', return_value=(None, [], {})), \
                        patch.object(ma, 'road_rejoin_plan', return_value=None):
                    _, _, context = runtime.model_static_plan(self.ego, self.path, objects,
                        self.unassociated, corridor, lane, clearance, None, {}, 1000., True, True, .946)
                checked = next(obj for obj in context['static'] if obj[11] == source[11])
                np.testing.assert_allclose(checked[6:8], (.9, .9) if scope == 'normal' else (1.1, 1.1),
                                           atol=1e-12)
                self.assertEqual(hasattr(runtime, '_road_margin_recovery_ledger'), scope == 'normal')


if __name__ == '__main__':
    unittest.main()
