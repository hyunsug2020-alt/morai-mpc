"""Preserve independent obstacles and all footprints when joining walking-person fragments."""
import json
import math
from pathlib import Path
import unittest

import numpy as np

from semantic_fusion import PedestrianPartMerger


class PedestrianPartTests(unittest.TestCase):
    @staticmethod
    def detection(uid, kind, position, velocity=(1., 0.), size=(.6, .6), height=1.6):
        return dict(uid=uid, kind=kind, pos=np.asarray(position), vel=np.asarray(velocity),
                    size=np.asarray(size), height=height, n=10, moving=True)

    def pair(self):
        return [self.detection(1, 'pedestrian', [5., 0.]),
                self.detection(2, 'obstacle', [5.1, .1], size=(1.1, 1.1))]

    def merge_confirmed(self, rows):
        merger = PedestrianPartMerger()
        first, second = merger.merge(rows, 10.), merger.merge(rows, 10.08)
        self.assertEqual(len(first), len(rows))
        self.assertEqual(len(second), len(rows))
        return merger.merge(rows, 10.16)

    def assert_corners_preserved(self, source, merged):
        corners = np.vstack([PedestrianPartMerger.corners(row) for row in source])
        heading = math.atan2(merged['vel'][1], merged['vel'][0])
        rotation = np.array([[math.cos(heading), -math.sin(heading)],
                             [math.sin(heading), math.cos(heading)]])
        relative = (corners-merged['pos']) @ rotation
        self.assertTrue(np.all(np.abs(relative) <= np.asarray(merged['size']) / 2. + 1e-8))

    def test_coherent_fragments_join_only_after_repeated_observation(self):
        rows = self.pair()
        result = self.merge_confirmed(rows)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]['uid'], 1)
        self.assertEqual(result[0]['kind'], 'pedestrian')
        self.assertEqual(result[0]['merged_part_ids'], [2])
        self.assert_corners_preserved(rows, result[0])

    def test_distinct_nearby_objects_remain_separate(self):
        for changes in [dict(pos=np.array([5., .65])), dict(vel=np.zeros(2)),
                        dict(vel=np.array([0., 1.])), dict(size=np.array([2., 1.1])),
                        dict(height=.4), dict(kind='npc')]:
            with self.subTest(changes=changes):
                rows = self.pair()
                rows[1].update(changes)
                result = self.merge_confirmed(rows)
                self.assertEqual(len(result), 2)

    def test_motion_warmup_and_repeated_source_stamp_do_not_confirm_duplicate(self):
        merger = PedestrianPartMerger()
        rows = self.pair()
        for _ in range(10):
            self.assertEqual(len(merger.merge(rows, 10.)), 2)
        rows[1]['vel'] = np.zeros(2)
        self.assertEqual(len(merger.merge(rows, 10.2)), 2)

    def test_missing_fragment_or_clock_reset_requires_new_confirmation(self):
        merger = PedestrianPartMerger()
        rows = self.pair()
        merger.merge(rows, 10.)
        merger.merge(rows, 10.08)
        merger.merge(rows[:1], 10.16)
        self.assertEqual(len(merger.merge(rows, 10.24)), 2)
        self.assertEqual(len(merger.merge(rows, 9.)), 2)

    def test_invalid_measurement_keeps_independent_obstacle(self):
        rows = self.pair()
        rows[1]['vel'] = np.array([float('nan'), 0.])
        self.assertEqual(len(self.merge_confirmed(rows)), 2)
        self.assertEqual(len(PedestrianPartMerger().merge(self.pair(), float('nan'))), 2)

    def test_recorded_two_crossings_confirm_both_pairs_and_preserve_neighbors(self):
        fixture = json.loads((Path(__file__).parent/'test_data/run_20261003_pedestrian_parts.json').read_text())
        merger = PedestrianPartMerger()
        counts = {(28, 29): 0, (47, 48): 0}
        for frame in fixture['frames']:
            rows = frame['detections']
            result = merger.merge(rows, frame['stamp'])
            before = {row['uid']: row for row in rows}
            after = {row['uid']: row for row in result}
            removed = set(before)-set(after)
            self.assertTrue(removed <= {29, 48}, (frame['elapsed'], removed))
            for pair in counts:
                person, fragment = pair
                if fragment in removed:
                    counts[pair] += 1
                    self.assertIn(person, after)
                    self.assert_corners_preserved([before[person], before[fragment]], after[person])
            # Unknown/initial stationary pieces retain their independent row.
            for row in rows:
                if row['kind'] == 'obstacle' and np.linalg.norm(row['vel']) <= .5:
                    self.assertIn(row['uid'], after)
        self.assertEqual(counts, {(28, 29): 14, (47, 48): 15})


if __name__ == '__main__':
    unittest.main()
