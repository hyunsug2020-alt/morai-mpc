"""A versioned producer name binds fresh physical geometry to one raw row."""
import copy
import math
import unittest

import numpy as np

from perception.static_geometry_contract import (CANONICAL_NAME, CONTRACT_NAME,
    canonical_static_name, producer_geometry_name, decode_static_geometry,
    validate_static_geometry, mark_stored_static_geometry, validate_stored_static_geometry)
from perception.stationary_semantics import with_metadata
from semantic_fusion import Tracker, PedestrianPartMerger


class StaticGeometryContractTests(unittest.TestCase):
    def setUp(self):
        physical = (.8, 1.)
        self.detection = dict(pos=np.array([10., 2.]), size=np.array(physical)+.3,
            yaw=.2, local=np.array([10., 2.]), height=.8, n=8,
            _static_geometry_evidence=dict(position_xy=(10., 2.), size_xy=(1.1, 1.3),
                physical_size_m=physical, yaw_rad=.2, height_m=.8,
                points_n=8, source_stamp=100.))
        self.detection = dict(Tracker().update('obstacle', [self.detection], 100.)[0],
                              kind='obstacle')
        self.alignment = dict(source_pose_valid=True, legacy_fallback=False,
                              mode='measured_pose', source_stamp=100.)

    def attrs(self, detection=None):
        d = self.detection if detection is None else detection
        return dict(uid=d['uid'], kind=d['kind'], position_xy=d['pos'], size_xy=d['size'],
            heading_deg=math.degrees(d['yaw']), velocity_xy=d['vel'],
            speed_mps=float(np.linalg.norm(d['vel'])), height_m=d['height'])

    def tag(self, detection=None, **changes):
        d = self.detection if detection is None else detection
        kwargs = dict(self.attrs(d), alignment=self.alignment, timing_ready=True)
        kwargs.update(changes)
        return producer_geometry_name(d, **kwargs)

    def proof(self, **changes):
        kwargs = dict(self.attrs(), source_stamp=100., measured_stamp=100.1)
        kwargs.update(changes)
        return decode_static_geometry(CONTRACT_NAME, **kwargs)

    def row(self):
        a = self.attrs()
        return with_metadata((a['position_xy'][0], a['position_xy'][1], max(a['size_xy']),
            CANONICAL_NAME, a['kind'], a['speed_mps'], *a['size_xy'], a['heading_deg'],
            *a['velocity_xy'], a['uid'], 0., None, None, 0., None),
            height_m=a['height_m'], raw_size_m=list(a['size_xy']))

    def test_current_measured_cluster_survives_tracking_without_changing_its_dimensions(self):
        original = copy.deepcopy(self.detection)
        self.assertEqual(self.tag(), CONTRACT_NAME)
        np.testing.assert_array_equal(self.detection['pos'], original['pos'])
        np.testing.assert_array_equal(self.detection['size'], original['size'])
        self.assertEqual(self.detection['_static_geometry_evidence'],
                         original['_static_geometry_evidence'])
        proof = self.proof()
        self.assertEqual(proof['source_points_min'], 4)
        self.assertNotIn('observed_points', proof)
        np.testing.assert_allclose(proof['physical_size_m'], [.8, 1.])
        self.assertIsNotNone(validate_static_geometry(proof, self.row(), 100.2))

    def test_only_the_known_version_is_canonicalized(self):
        self.assertEqual(canonical_static_name(CONTRACT_NAME), CANONICAL_NAME)
        for name in (CANONICAL_NAME, 'fusion_obstacle_padded_v2', 'fusion_parked_vehicle',
                     'fusion_pedestrian', 'custom', None):
            self.assertEqual(canonical_static_name(name), name)
            self.assertIsNone(decode_static_geometry(name,
                **dict(self.attrs(), source_stamp=100., measured_stamp=100.1)))

    def test_missing_original_evidence_and_insufficient_points_cannot_tag(self):
        d = copy.deepcopy(self.detection)
        del d['_static_geometry_evidence']
        self.assertEqual(self.tag(d), CANONICAL_NAME)
        for count in (0, 3, True, 4.5):
            d = copy.deepcopy(self.detection)
            d['_static_geometry_evidence']['points_n'] = d['n'] = count
            self.assertEqual(self.tag(d), CANONICAL_NAME)
        d = copy.deepcopy(self.detection)
        d['n'] -= 1
        self.assertEqual(self.tag(d), CANONICAL_NAME)

    def test_failed_alignment_or_final_timing_cannot_tag(self):
        for change in (dict(source_pose_valid=False), dict(legacy_fallback=True),
                       dict(mode='legacy_motion_fallback'), dict(source_stamp=99.9)):
            self.assertEqual(self.tag(alignment=dict(self.alignment, **change)), CANONICAL_NAME)
        self.assertEqual(self.tag(timing_ready=False), CANONICAL_NAME)

    def test_motion_class_merge_and_publication_candidates_cannot_tag(self):
        for change in (dict(vel=np.array([.01, 0.])), dict(moving=True),
                       dict(kind='npc'), dict(kind='pedestrian'), dict(merged_part_ids=[7]),
                       dict(publish_geometry=object()), dict(publication_geometry=object())):
            d = dict(self.detection, **change)
            self.assertEqual(self.tag(d), CANONICAL_NAME)
        self.assertEqual(self.tag(name='fusion_parked_vehicle'), 'fusion_parked_vehicle')

    def test_changed_detection_geometry_cannot_reuse_original_physical_evidence(self):
        for change in (dict(pos=np.array([10.01, 2.])), dict(size=np.array([1.2, 1.3])),
                       dict(yaw=.21), dict(height=.81)):
            self.assertEqual(self.tag(dict(self.detection, **change)), CANONICAL_NAME)
        for change in (dict(physical_size_m=(.7, 1.)), dict(physical_size_m=(.9, 1.)),
                       dict(source_stamp=99.9), dict(height_m=.9)):
            d = copy.deepcopy(self.detection)
            d['_static_geometry_evidence'].update(change)
            self.assertEqual(self.tag(d), CANONICAL_NAME)

    def test_changed_final_published_fields_cannot_tag(self):
        for changes in (dict(uid=8), dict(kind='npc'), dict(position_xy=(10.01, 2.)),
                        dict(size_xy=(1.2, 1.3)), dict(heading_deg=0.),
                        dict(velocity_xy=(.01, 0.)), dict(speed_mps=.01), dict(height_m=.9)):
            self.assertEqual(self.tag(**changes), CANONICAL_NAME)

    def test_a_real_pedestrian_part_merge_does_not_inherit_an_obstacle_contract(self):
        obstacle = dict(self.detection, vel=np.array([1., 0.]), moving=True,
                        pos=np.array([10.2, 2.]), height=1.2)
        person = dict(obstacle, uid=20, kind='pedestrian', pos=np.array([10., 2.]),
                      size=np.array([.6, .6]), height=1.3)
        merger = PedestrianPartMerger()
        for stamp in (100., 100.1, 100.2):
            result = merger.merge([person, obstacle], stamp)
        self.assertEqual(len(result), 1)
        self.assertTrue(result[0]['merged_part_ids'])
        self.assertEqual(self.tag(result[0], name='fusion_pedestrian'), 'fusion_pedestrian')

    def test_source_clock_age_and_future_bound_do_not_renew_the_physical_frame(self):
        self.assertIsNotNone(self.proof(measured_stamp=100.3))
        self.assertIsNotNone(self.proof(measured_stamp=99.95))
        for changes in (dict(measured_stamp=100.30001), dict(measured_stamp=99.94999),
                        dict(source_stamp=0.), dict(source_stamp=float('nan')),
                        dict(measured_stamp=float('inf'))):
            self.assertIsNone(self.proof(**changes))
        proof = self.proof()
        self.assertIsNotNone(validate_static_geometry(proof, self.row(), 100.29))
        self.assertIsNone(validate_static_geometry(proof, self.row(), 100.31))
        self.assertEqual(proof['source_stamp'], 100.)

    def test_invalid_row_fields_cannot_decode(self):
        for change in (dict(uid=True), dict(uid=-1), dict(uid=1.5), dict(kind='pedestrian'),
                       dict(position_xy=(10., float('nan'))), dict(size_xy=(.9, 1.3)),
                       dict(size_xy=(1.1, float('inf'))), dict(heading_deg=float('nan')),
                       dict(velocity_xy=(.01, 0.)), dict(speed_mps=.01), dict(height_m=0.)):
            self.assertIsNone(self.proof(**change))

    def test_current_raw_row_changes_invalidate_the_bound_signature(self):
        proof = self.proof()
        for index, value in ((0, 10.01), (1, 2.01), (2, 2.), (3, 'custom'),
                             (4, 'npc'), (5, .01), (6, 1.2), (7, 1.4),
                             (8, 0.), (9, .01), (10, .01), (11, 9)):
            row = list(self.row())
            row[index] = value
            self.assertIsNone(validate_static_geometry(proof, tuple(row), 100.1))
        self.assertIsNone(validate_static_geometry(proof,
            with_metadata(self.row(), height_m=.9), 100.1))

    def test_normalized_row_and_forged_proof_fields_are_not_raw_source_evidence(self):
        proof = self.proof()
        row = list(self.row())
        row[2], row[6], row[7] = 1.1, .9, 1.1
        self.assertIsNone(validate_static_geometry(proof, tuple(row), 100.1))
        for changes in (dict(version=True), dict(version=2), dict(source='other'),
                        dict(contract='fusion_obstacle_padded_v2'), dict(source_points_min=3),
                        dict(source_padding_m=.2), dict(physical_size_m=[.9, 1.]),
                        dict(signature=())):
            self.assertIsNone(validate_static_geometry(dict(proof, **changes), self.row(), 100.1))

    def stored_row(self):
        proof = self.proof()
        physical = np.asarray(proof['physical_size_m'])
        row = list(self.row())
        row[6:8], row[2] = physical+.1, max(physical+.1)
        return with_metadata(tuple(row), producer_static_geometry=proof,
            road_static_geometry=dict(version=1, source='semantic_fusion',
                producer_geometry=True, observed_points=0, source_size_m=[1.1, 1.3],
                physical_size_m=physical.tolist(), source_padding_m=.3, applied_padding_m=.1))

    def test_only_fresh_normalized_producer_storage_can_issue_immutable_history(self):
        row = self.stored_row()
        marker = mark_stored_static_geometry(row, 100.1)
        self.assertIsInstance(marker, tuple)
        self.assertEqual(marker[2], 100.)
        self.assertEqual(marker, mark_stored_static_geometry(row, 100.2))
        for stamp in (100.30001, 99.94999, None, float('nan')):
            self.assertIsNone(mark_stored_static_geometry(row, stamp))
        self.assertIsNone(mark_stored_static_geometry(self.row(), 100.1))
        observed = copy.deepcopy(row)
        observed[17]['road_static_geometry']['observed_points'] = 1
        self.assertIsNone(mark_stored_static_geometry(observed, 100.1))
        self.assertIsNone(validate_stored_static_geometry(None, row, 100.4))

    def test_history_is_bounded_by_original_source_clock_without_current_proof_renewal(self):
        row = self.stored_row()
        marker = mark_stored_static_geometry(row, 100.1)
        for stamp in (100., 100.4, 102.999, 103.):
            self.assertEqual(validate_stored_static_geometry(marker, row, stamp), marker)
        for stamp in (99.99999, 103.00001, float('inf'), None):
            self.assertIsNone(validate_stored_static_geometry(marker, row, stamp))
        raw = self.row()
        self.assertIsNone(validate_static_geometry(row[17]['producer_static_geometry'], raw, 100.4))
        self.assertEqual(marker[2], 100.)
        changed = copy.deepcopy(row)
        changed[17]['producer_static_geometry']['source_stamp'] = 100.1
        self.assertIsNone(validate_stored_static_geometry(marker, changed, 100.4))

    def test_history_rejects_any_changed_bound_motion_geometry_class_or_marker(self):
        row = self.stored_row()
        marker = mark_stored_static_geometry(row, 100.1)
        for index, value in ((0, 10.01), (1, 2.01), (2, 1.3), (3, 'custom'),
                             (4, 'pedestrian'), (5, .01), (6, 1.), (7, 1.2),
                             (8, 0.), (9, .01), (10, .01), (11, 20)):
            with self.subTest(index=index):
                changed = list(row)
                changed[index] = value
                self.assertIsNone(validate_stored_static_geometry(marker, tuple(changed), 100.4))
        for changes in (dict(height_m=.9), dict(raw_size_m=[1.2, 1.3])):
            self.assertIsNone(validate_stored_static_geometry(marker,
                with_metadata(row, **changes), 100.4))
        for changes in (dict(observed_points=1), dict(physical_size_m=[.9, 1.]),
                        dict(source_size_m=[1.2, 1.3]), dict(applied_padding_m=.2)):
            changed = copy.deepcopy(row)
            changed[17]['road_static_geometry'].update(changes)
            self.assertIsNone(validate_stored_static_geometry(marker, changed, 100.4))
        for index, value in ((0, True), (1, 'unknown'), (2, 100.1),
                             (4, (.9, 1.)), (5, (1., 1.1)), (6, .2)):
            changed = list(marker)
            changed[index] = value
            self.assertIsNone(validate_stored_static_geometry(tuple(changed), row, 100.4))
        self.assertIsNone(validate_stored_static_geometry(list(marker), row, 100.4))


if __name__ == '__main__':
    unittest.main()
