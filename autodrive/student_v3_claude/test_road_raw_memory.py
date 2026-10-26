"""Physical-scan and recorded road-target dropout contracts."""
import copy
import json
from pathlib import Path
import unittest

import numpy as np

from perception.road_raw_memory import CommittedRoadRawMemory, MEMORY_KEY


def box(x=10., y=0., uid=0, *, speed=0., length=.3, width=.3):
    return (x, y, max(length, width), f'tunnel_lidar_{uid}', 'obstacle', speed,
            length, width, 0., 0., 0., uid, {'point_count': 2})


def observation(stamp, sequence, *, x=0., y=0., heading=0., lag=.02):
    ego = dict(measured=True, stamp=stamp, x=x, y=y, heading_deg=heading)
    pose = [stamp, x, y, heading]
    return ego, dict(valid=True, stamp=stamp, sequence=sequence, source_pose=pose,
                     target_pose=pose, age_s=lag), stamp+lag


class RoadRawMemoryTests(unittest.TestCase):
    def setUp(self):
        self.memory = CommittedRoadRawMemory()

    def update(self, targets, stamp, sequence, *, active=True, committed=True, **pose):
        ego, proof, now = observation(stamp, sequence, **pose)
        return self.memory.update(targets, ego, proof=proof, now_wall=now,
                                  active=active, committed=committed)

    def confirm(self):
        self.update([box()], 100., 1)
        self.update([box()], 100.1, 2)

    def test_confirmation_needs_two_distinct_physical_scans(self):
        self.update([box()], 100., 1)
        self.update([box()], 100., 1, lag=.1)
        self.assertEqual(self.update([], 100.2, 2), [])
        self.update([box()], 100.3, 3)
        held = self.update([], 100.4, 4)
        self.assertEqual(len(held), 1)
        self.assertEqual(held[0][12][MEMORY_KEY]['observations'], 2)

    def test_observations_before_commit_do_not_authorize_a_manoeuvre(self):
        self.update([box()], 100., 1, committed=False)
        self.update([box()], 100.1, 2, committed=False)
        self.assertEqual(self.update([], 100.2, 3, committed=False), [])
        self.assertEqual(len(self.update([], 100.2, 3, committed=True)), 1)

    def test_reused_scan_and_retained_feedback_never_renew_source_or_bounds(self):
        self.confirm()
        self.update([box(x=10.2, uid=99)], 100.1, 2, lag=.12)
        held = self.update([], 100.25, 3)
        self.assertEqual(held[0][:2], (10., 0.))
        self.assertEqual(held[0][12][MEMORY_KEY]['source_stamp'], 100.1)
        again = self.update(held, 100.4, 4)
        self.assertEqual(again[0][12][MEMORY_KEY]['source_stamp'], 100.1)
        self.assertEqual(self.update([], 100.7, 5), [])

    def test_spatial_match_survives_uid_change_but_not_a_different_target(self):
        self.update([box(uid=0)], 100., 1)
        self.update([box(x=10.247, uid=2)], 100.1, 2)
        held = self.update([], 100.2, 3)
        self.assertEqual(held[0][11], 2)
        self.assertEqual(held[0][0], 10.247)
        other = CommittedRoadRawMemory()
        ego, proof, now = observation(100., 1)
        other.update([box()], ego, proof=proof, now_wall=now, active=True, committed=True)
        ego, proof, now = observation(100.1, 2)
        other.update([box(x=12., uid=0)], ego, proof=proof, now_wall=now, active=True, committed=True)
        ego, proof, now = observation(100.2, 3)
        self.assertEqual(other.update([], ego, proof=proof, now_wall=now, active=True, committed=True), [])

    def test_world_body_is_unchanged_and_releases_only_after_actual_rear_pass(self):
        self.confirm()
        held = self.update([], 100.2, 3, x=9.5, heading=90.)
        self.assertEqual(held[0][:12], box()[:12])
        self.assertEqual(len(self.update([], 100.3, 4, x=10.5)), 1)
        self.assertEqual(self.update([], 100.4, 5, x=11.3), [])
        self.assertEqual(self.update([], 100.45, 6, x=0.), [])

    def test_inactive_ordinary_road_scope_resets_and_does_not_resurrect(self):
        for excluded_scope in ('tunnel', 'junction', 'highway', 'person', 'standing bypass'):
            with self.subTest(scope=excluded_scope):
                self.memory.reset()
                self.confirm()
                self.assertEqual(self.update([], 100.2, 3, active=False), [])
                self.assertEqual(self.update([], 100.3, 4), [])

    def test_new_measured_motion_cancels_the_matching_static_memory(self):
        self.confirm()
        self.assertEqual(self.update([box(speed=.4)], 100.2, 3), [])
        self.assertEqual(self.update([], 100.3, 4), [])

    def test_no_wall_noise_person_or_nonfinite_box_is_retained(self):
        person = list(box())
        person[3], person[4] = 'fusion_pedestrian', 'pedestrian'
        empty = list(box())
        empty[12] = {'point_count': 0}
        for obj in (box(length=9.), box(length=6., width=.3), box(x=float('nan')), person, empty):
            with self.subTest(obj=obj):
                self.memory.reset()
                self.update([obj], 100., 1)
                self.update([obj], 100.1, 2)
                self.assertEqual(self.update([], 100.2, 3), [])

    def test_invalid_source_pose_age_and_clock_reset_fail_closed(self):
        for failure in ('invalid', 'unmeasured', 'pose mismatch', 'old source', 'nan', 'sequence restart'):
            with self.subTest(failure=failure):
                self.memory.reset()
                self.confirm()
                ego, proof, now = observation(100.2, 3)
                if failure == 'invalid':
                    proof['valid'] = False
                elif failure == 'unmeasured':
                    ego['measured'] = False
                elif failure == 'pose mismatch':
                    ego['x'] = 1.
                elif failure == 'old source':
                    now += .4
                elif failure == 'nan':
                    now = float('nan')
                else:
                    proof['sequence'] = 1
                self.assertEqual(self.memory.update([], ego, proof=proof, now_wall=now,
                                                   active=True, committed=True), [])
                self.assertEqual(self.update([], 100.3, 4), [])

    def test_returned_metadata_is_a_copy_of_measured_geometry(self):
        original = box()
        self.update([original], 100., 1)
        self.update([original], 100.1, 2)
        held = self.update([], 100.2, 3)
        held[0][12]['point_count'] = 999
        again = self.update([], 100.3, 4)
        self.assertEqual(again[0][12]['point_count'], 2)
        self.assertNotIn(MEMORY_KEY, original[12])


class RecordedRawDropoutTests(unittest.TestCase):
    def test_303_305_307_309_preserve_only_last_measured_world_box(self):
        path = Path(__file__).parent/'test_data/road_raw_dropout_20261017.npz'
        memory = CommittedRoadRawMemory()
        result = {}
        with np.load(path, allow_pickle=False) as data:
            for sequence in (301, 303, 305, 307, 309):
                prefix = f'frame{sequence}_'
                ego = json.loads(str(data[prefix+'ego']))
                proof = json.loads(str(data[prefix+'proof']))
                targets = json.loads(str(data[prefix+'raw_boxes']))
                result[sequence] = memory.update(targets, ego, proof=proof,
                    now_wall=float(data[prefix+'check_wall']), active=True, committed=True)
            measured = json.loads(str(data['frame303_raw_boxes']))[0]
            returned = json.loads(str(data['frame309_raw_boxes']))[0]
            self.assertEqual(measured[11], 0)
            self.assertEqual(returned[11], 2)
            self.assertAlmostEqual(np.linalg.norm(np.asarray(measured[:2])-returned[:2]), .24715066, places=5)
            self.assertEqual(result[301], [])
            self.assertEqual(result[303], [])
            self.assertEqual(result[309], [])  # Fresh reappearance takes precedence.
            for sequence in (305, 307):
                self.assertEqual(len(result[sequence]), 1)
                np.testing.assert_allclose(result[sequence][0][:2], measured[:2], rtol=0., atol=0.)
                np.testing.assert_allclose(result[sequence][0][6:9], measured[6:9], rtol=0., atol=0.)
                evidence = result[sequence][0][12][MEMORY_KEY]
                self.assertEqual(evidence['source_stamp'], 1792233452.2411165)
                self.assertEqual(evidence['sequence'], 457)
                self.assertEqual(evidence['observations'], 2)
                self.assertLess(evidence['age_s'], .5)
            self.assertEqual(memory.last_diagnostics['confirmed_n'], 1)


if __name__ == '__main__':
    unittest.main()
