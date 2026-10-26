"""Physical scan certificates outlive a brief raw dropout, never their source."""
import copy
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from perception.road_static_calibration import RoadStaticCalibration
from perception.stationary_semantics import metadata, with_metadata
from test_road_static_fusion import obstacle
from lane_corridor import LaneCorridor
import test_avoidance_gate as fixture


class RoadStaticCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.cache = RoadStaticCalibration()
        self.source = with_metadata(obstacle(), raw_size_m=[1.1, 1.1], height_m=.4)
        self.ego = dict(x=0., y=0., heading_deg=0., stamp=100., measured=True)
        self.points = np.array([[4.9, -1.8], [4.9, -1.6], [5.1, -1.4]])
        self.proof = dict(valid=True, stamp=100., sequence=1, source_pose=[100., 0., 0., 0.],
            target_pose=[100., 0., 0., 0.], age_s=0., points_n=3)

    def observe(self, *, source=None, points=None, now=100., stamp=None, sequence=None,
                proof=None, active=True, retained=0):
        source = self.source if source is None else source
        points = self.points if points is None else points
        stamp = now if stamp is None else stamp
        ego = dict(self.ego, stamp=now)
        cert = dict(self.proof, stamp=stamp, sequence=round((stamp-100.)*100)+1 if sequence is None else sequence,
            source_pose=[stamp, 0., 0., 0.], target_pose=[now, 0., 0., 0.], age_s=now-stamp,
            points_n=len(points)) if proof is None else proof
        checked = ma.road_static_objects([source], ego, points)
        return self.cache.update([source], checked, ego, points, proof=cert, now_wall=now,
            active=active, retained_near_field_n=retained)[0]

    def test_dropout_reuses_only_the_same_source_and_preserves_current_fields(self):
        checked = self.observe()
        current = list(self.source)
        current[12:] = [dict(unrelated='current')]
        current = with_metadata(tuple(current), **metadata(self.source))
        restored = self.observe(source=current, points=np.empty((0, 2)), now=100.2)
        np.testing.assert_allclose(restored[6:8], [.9, .9])
        self.assertEqual(restored[:2], current[:2])
        self.assertEqual(restored[3:6], current[3:6])
        self.assertEqual(restored[8:17], current[8:17])
        self.assertEqual(self.source[6:8], (1.1, 1.1))
        self.assertEqual(self.cache._tracks[7]['frame'], (100., 1))

    def test_semantic_and_same_scan_repetitions_never_renew_the_raw_ttl(self):
        self.observe()
        for now in (100.1, 100.2, 100.3):
            self.observe(now=now, stamp=100., sequence=1)
            self.assertEqual(self.cache._tracks[7]['frame'], (100., 1))
        for now in (100.4, 100.5):
            result = self.observe(points=np.empty((0, 2)), now=now)
            np.testing.assert_allclose(result[6:8], [.9, .9])
            self.assertEqual(self.cache._tracks[7]['frame'], (100., 1))
        expired = self.observe(points=np.empty((0, 2)), now=100.500001)
        self.assertEqual(expired[6:8], (1.1, 1.1))

    def test_new_physical_scan_can_certify_and_renew_unchanged_geometry(self):
        self.observe()
        self.observe(now=100.2)
        self.assertEqual(self.cache._tracks[7]['frame'], (100.2, 21))

    def test_retained_or_unproved_point_counts_cannot_issue_or_renew_calibration(self):
        for retained in (1, -1, None, True, float('nan')):
            self.cache.reset()
            self.observe(retained=retained)
            self.assertEqual(self.cache._tracks, {})
        self.cache.reset()
        self.observe()
        self.observe(now=100.2, retained=1)
        self.assertEqual(self.cache._tracks[7]['frame'], (100., 1))
        cert = dict(self.proof, points_n=2)
        self.cache.reset()
        self.observe(proof=cert)
        self.assertEqual(self.cache._tracks, {})

    def test_associated_current_surface_rejected_by_normalizer_cannot_use_old_geometry(self):
        self.observe()
        edge = np.array([[5.55, -1.6]])
        current = self.observe(points=edge, now=100.2)
        self.assertEqual(current[6:8], (1.1, 1.1))
        self.assertNotIn('road_static_geometry', metadata(current))
        self.assertNotIn(7, self.cache._tracks)
        self.assertEqual(self.observe(points=np.empty((0, 2)), now=100.3)[6:8], (1.1, 1.1))

    def test_changed_physical_surface_without_a_new_certificate_revokes_the_old_size(self):
        for retained, stamp, sequence in ((1, 100.2, 21), (0, 100., 1)):
            self.cache.reset(); self.observe()
            larger = self.observe(points=np.array([[5.46, -1.6]]), now=100.2,
                stamp=stamp, sequence=sequence, retained=retained)
            self.assertGreater(larger[6], 1.)
            self.assertNotIn(7, self.cache._tracks)
            self.assertEqual(self.observe(points=np.empty((0, 2)), now=100.3)[6:8], (1.1, 1.1))

    def test_changed_source_pose_size_class_motion_or_identity_cancels_old_calibration(self):
        for fields in ({0: 5.00001}, {1: -1.60001}, {8: .00001}, {6: 1.2},
                {3: 'fusion_parked_vehicle'}, {4: 'pedestrian'}, {5: .01}, {9: .01}):
            self.cache.reset(); self.observe()
            row = list(self.source)
            for index, value in fields.items(): row[index] = value
            result = self.observe(source=tuple(row), points=np.empty((0, 2)), now=100.2)
            self.assertEqual(result[6:8], tuple(row[6:8]))
            self.assertNotIn(7, self.cache._tracks)
        self.cache.reset(); self.observe()
        row = list(self.source); row[11] = 8
        self.assertEqual(self.observe(source=tuple(row), points=np.empty((0, 2)), now=100.2)[6:8], (1.1, 1.1))
        self.assertNotIn(7, self.cache._tracks)

    def test_bad_proof_clock_and_scope_reset_without_restoring(self):
        changes = (dict(valid=False), dict(age_s=float('nan')), dict(sequence=True),
            dict(target_pose=[100.2, 1., 0., 0.]), dict(source_pose=[99., 0., 0., 0.]))
        for change in changes:
            self.cache.reset(); self.observe()
            cert = dict(self.proof, stamp=100.2, sequence=2, source_pose=[100.2, 0., 0., 0.],
                target_pose=[100.2, 0., 0., 0.], **{})
            cert.update(change)
            result = self.observe(points=np.empty((0, 2)), now=100.2, proof=cert)
            self.assertEqual(result[6:8], (1.1, 1.1)); self.assertEqual(self.cache._tracks, {})
        self.observe()
        self.assertEqual(self.observe(points=np.empty((0, 2)), now=99.9)[6:8], (1.1, 1.1))
        self.observe()
        self.assertEqual(self.observe(points=np.empty((0, 2)), now=100.2, active=False)[6:8], (1.1, 1.1))

    def test_three_second_semantic_memory_and_pose_recovery_never_issue_a_raw_certificate(self):
        checked = self.observe()
        geometry = dict(metadata(checked)['road_static_geometry'], initial_margin_floor_m=.15,
            initial_margin_restore_start_m=4.)
        stored = with_metadata(checked, road_static_geometry=geometry)
        cache = RoadStaticCalibration()
        result = cache.update([stored], [stored], self.ego, self.points, proof=self.proof,
            now_wall=100., active=True, retained_near_field_n=0)
        self.assertEqual(cache._tracks, {})
        self.assertEqual(result, [stored])
        restored = self.observe(points=np.empty((0, 2)), now=100.2)
        self.assertNotIn('initial_margin_floor_m', metadata(restored)['road_static_geometry'])

    def test_model_host_restores_before_memory_and_rejects_each_scope_exit(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        route = np.column_stack((np.arange(0., 80.1, .5), np.zeros(161)))
        corridor = LaneCorridor(route, np.full(161, 3.5), np.full(161, -3.5))
        for scope in ('normal', 'person', 'moving', 'contact', 'tunnel', 'outside', 'junction'):
            runtime = fixture.AvoidanceGateTests().runtime(action='KEEP', objects=[self.source],
                points=self.points)
            runtime._contact = None
            for now, points in ((100., self.points), (100.2, np.array([[40., 8.]]))):
                ego = dict(self.ego, stamp=now, speed_kmh=15.)
                cert = dict(self.proof, stamp=now, sequence=round((now-100.)*100)+1,
                    source_pose=[now, 0., 0., 0.], target_pose=[now, 0., 0., 0.], points_n=len(points))
                runtime._lidar_log = dict(road_scan_frame=cert, retained_near_field_n=0)
                objects = [self.source]
                if now>100.:
                    if scope=='person': objects.append(obstacle(x=50., name='fusion_pedestrian', kind='pedestrian'))
                    if scope=='moving': objects.append(obstacle(x=50., speed=3.))
                    if scope=='contact': runtime._contact = dict(block_forward=True, points=[])
                    if scope=='tunnel': runtime.tunnel = SimpleNamespace(active=lambda ego: True)
                    if scope=='junction': runtime._intersection_vehicle_status = {}
                lane, clear = ma.road_static_checks(corridor, points)
                with patch('static_lane_pass.normal_static_area', return_value=scope!='outside' or now==100.), \
                        patch.object(gui, 'model_avoidance', ma, create=True), \
                        patch('gui.time.time', return_value=now):
                    _, _, context = runtime.model_static_plan(ego, route, objects, points, corridor,
                        lane, clear, None, {}, now, True, True, 1.)
                if now==100.:
                    self.assertIn(7, runtime._road_static_calibration._tracks)
                elif scope=='normal':
                    current = next(o for o in context['static'] if o[3]=='fusion_obstacle' and o[11]==7)
                    np.testing.assert_allclose(current[6:8], [.9, .9])
                    self.assertEqual(runtime._lidar_log['road_static_calibration']['restored_n'], 1)
                else:
                    self.assertEqual(runtime._road_static_calibration._tracks, {}, scope)

    def test_actual_117_to_120_retains_physical_box_without_renewing_semantic_memory(self):
        path = Path(__file__).parent/'test_data/road_static_calibration_20261019.npz'
        runtime = gui.Runtime.__new__(gui.Runtime)
        with np.load(path) as data:
            first_stamp = None
            for seq in (117, 118, 119, 120):
                prefix = f'frame{seq}_'
                ego, objects, log = (json.loads(str(data[prefix+key])) for key in ('ego', 'objects', 'proof'))
                sources = ma.split_static(objects)[0]
                points = data[prefix+'points']; cert = log['road_scan_frame']
                wall = cert['stamp']+cert['wall_age_s']
                normalized = ma.road_static_objects(sources, ego, points)
                current = self.cache.update(sources, normalized, ego, points, proof=cert,
                    now_wall=wall, active=True, retained_near_field_n=log['retained_near_field_n'])
                current = ma.road_static_margin_recovery(current, ego, points)
                memory = runtime.remember_static_objects(current, float(data[prefix+'now']), True, objects)
                box = next(o for o in memory if o[3]=='fusion_obstacle' and o[11]==6)
                np.testing.assert_allclose(box[6:8], [.9, .9], atol=1e-6)
                if seq==117: first_stamp = self.cache._tracks[6]['frame'][0]
                if 6 in self.cache._tracks: self.assertEqual(self.cache._tracks[6]['frame'][0], first_stamp)
                if seq==118: self.assertEqual(self.cache.last_diagnostics['restored_n'], 1)

    def test_model_host_uses_original_motion_to_revoke_calibration_before_static_zeroing(self):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        route = np.column_stack((np.arange(0., 80.1, .5), np.zeros(161)))
        corridor = LaneCorridor(route, np.full(161, 3.5), np.full(161, -3.5))
        other = list(with_metadata(obstacle(x=20., y=1.6), raw_size_m=[1.1, 1.1], height_m=.4))
        other[11] = 8
        other = tuple(other)
        for field in (5, 9, 10):
            with self.subTest(motion_field=field):
                runtime = fixture.AvoidanceGateTests().runtime(action='KEEP')
                runtime._contact = None
                cache = runtime._road_static_calibration = RoadStaticCalibration()
                for index, motion in enumerate((0., .01, 0.)):
                    now = 100.+index*.1
                    source = list(self.source)
                    source[field] = motion
                    source = tuple(source)
                    objects = [other, source]
                    points = (np.vstack((self.points, [[20., 1.6]])) if index==0
                              else np.array([[40., 8.]]))
                    ego = dict(self.ego, stamp=now, speed_kmh=15.)
                    cert = dict(self.proof, stamp=now, sequence=index+1,
                        source_pose=[now, 0., 0., 0.], target_pose=[now, 0., 0., 0.],
                        points_n=len(points))
                    runtime._lidar_log = dict(road_scan_frame=cert, retained_near_field_n=0)
                    lane, clear = ma.road_static_checks(corridor, points)
                    with patch('static_lane_pass.normal_static_area', return_value=True), \
                            patch.object(gui, 'model_avoidance', ma, create=True), \
                            patch('gui.time.time', return_value=now), \
                            patch.object(cache, 'update', wraps=cache.update) as update:
                        _, _, context = runtime.model_static_plan(ego, route, objects, points,
                            corridor, lane, clear, None, {}, now, True, True, 1.)
                    # The cache sees original semantic rows in the same order;
                    # the planner retains its existing stationary representation.
                    current = next(o for o in context['static'] if o[3]=='fusion_obstacle' and o[11]==7)
                    self.assertEqual(tuple(current[i] for i in (5, 9, 10)), (0., 0., 0.))
                    if index==0:
                        np.testing.assert_allclose(current[6:8], [.9, .9])
                        self.assertIn(7, cache._tracks)
                    else:
                        self.assertEqual(current[6:8], (1.1, 1.1))
                        self.assertNotIn(7, cache._tracks)
                        self.assertEqual(cache.last_diagnostics['restored_n'], 1)
                        self.assertIn(8, cache._tracks)
                    self.assertEqual(update.call_args.args[0], objects)


if __name__ == '__main__': unittest.main()
