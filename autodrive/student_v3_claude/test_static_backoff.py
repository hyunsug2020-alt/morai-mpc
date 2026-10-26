"""A recorded close static deadlock needs a checked, bounded backoff."""
import copy
import json
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

import gui
from static_lane_pass import checked_static_backoff, PROFILE_RETURN_BUDGET_S
from test_static_initial_join_regression import ROOT
import test_static_initial_join_regression as initial


class StaticBackoffTests(unittest.TestCase):
    setUp = initial.RecordedInitialJoinTests.setUp

    def record(self):
        with np.load(ROOT/'test_data/static_forward_deadlock_20261024.npz',
                     allow_pickle=False) as fixture:
            return json.loads(str(fixture['record']))

    def test_recorded_deadlock_has_a_short_backoff_with_a_complete_forward_return(self):
        record = self.record()
        before = copy.deepcopy(record)
        proof = checked_static_backoff(self.lanes, record)
        self.assertIsNotNone(proof)
        self.assertEqual(proof['distance_m'], 1.5)
        self.assertGreaterEqual(proof['forward_clear_m'], 18.-1e-3)
        self.assertLessEqual(proof['forward_contact_s'], PROFILE_RETURN_BUDGET_S-.25)
        self.assertEqual(proof['source_stamp'], record['ego']['stamp'])
        self.assertEqual(record, before)

    def test_rear_and_front_swing_contacts_cannot_start_a_backoff(self):
        for point in ([-1.2, 0.], [3.8, 1.1]):
            with self.subTest(point=point):
                record = self.record()
                record['points'].append(point)
                self.assertIsNone(checked_static_backoff(self.lanes, record))

    def test_missing_evidence_motion_people_and_an_active_lane_excursion_keep_the_stop(self):
        for case in ('stale', 'unmeasured', 'moving', 'person', 'npc', 'entered', 'no_map', 'nan', 'clear_raw'):
            with self.subTest(case=case):
                record = self.record()
                if case == 'stale':
                    record['ego']['age_s'] = .3
                elif case == 'unmeasured':
                    record['ego']['measured'] = False
                elif case == 'moving':
                    record['ego']['speed_kmh'] = 1.
                elif case in ('person', 'npc'):
                    row = list(record['objects'][0])
                    row[4] = 'pedestrian' if case == 'person' else 'npc'
                    record['objects'].append(row)
                elif case == 'entered':
                    record['policy']['entered_at'] = record['now']-.1
                elif case == 'no_map':
                    record['legal'] = None
                elif case == 'nan':
                    record['points'][0][0] = float('nan')
                elif case == 'clear_raw':
                    record['points'] = []
                self.assertIsNone(checked_static_backoff(self.lanes, record))

    def host(self):
        record = self.record()
        now = time.monotonic()
        record['now'] = now
        current = record['ego']
        publisher = SimpleNamespace(reverse_active=lambda ego: False, start_reverse=Mock())
        host = SimpleNamespace(path_publisher=publisher, lanes=self.lanes,
            _model_avoidance_log=dict(bounded_pass=dict(replay=record)),
            _reverse_wait_since=now-4., _contact=None,
            object_receiver=SimpleNamespace(snapshot=lambda: record['objects'], age_s=.05,
                                            observation_stamp=current['stamp']))
        frame = (current, np.asarray(record['route']), np.asarray(record['points']), dict(valid=True))
        return host, current, frame, now

    def test_runtime_starts_only_the_checked_distance_and_keeps_static_forward_release(self):
        host, ego, frame, now = self.host()
        with patch.object(gui, 'current_static_guard_frame', return_value=frame):
            self.assertTrue(gui.Runtime.reverse_escape_check(host, ego, True, 1.25,
                frame[2], now, swing_check=True, static_recovery=True))
        call = host.path_publisher.start_reverse.call_args
        self.assertEqual(call.kwargs['distance_m'], 1.5)
        self.assertTrue(call.kwargs['static_recovery'])

    def test_new_rear_return_after_feasibility_search_vetoes_the_actual_reverse(self):
        host, ego, frame, now = self.host()
        newer = (frame[0], frame[1], np.vstack((frame[2], [[-1.2, 0.]])), frame[3])
        with patch.object(gui, 'current_static_guard_frame', side_effect=[frame, newer]):
            self.assertFalse(gui.Runtime.reverse_escape_check(host, ego, True, 1.25,
                frame[2], now, swing_check=True, static_recovery=True))
        host.path_publisher.start_reverse.assert_not_called()


if __name__ == '__main__':
    unittest.main()
