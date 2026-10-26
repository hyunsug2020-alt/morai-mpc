"""Mandatory stop plus measured traffic clearance, with advisory model labels."""
import unittest

from stop_line import StopLineController, intersection_entry_decision


ACTIONS = ['KEEP_LANE', 'LEFT_CHANGE', 'RIGHT_CHANGE', 'TURN_LEFT',
           'TURN_RIGHT', 'STOP', 'YIELD', 'UNKNOWN']
LINE = dict(id='709:1781', node_id='A1256W000709', station_m=50., xy=[50., 0.])
STATION = LINE['station_m'] - StopLineController.FRONT_M - 1.


def probabilities(action):
    values = [.08] * len(ACTIONS)
    values[ACTIONS.index(action)] = .44
    return values


class IntersectionModelGateTests(unittest.TestCase):
    def decision(self, action='KEEP_LANE', **kwargs):
        return intersection_entry_decision(ACTIONS, probabilities(action),
                                           lidar_ready=True, **kwargs)

    def test_low_confidence_argmax_consents_without_arbitrary_half_threshold(self):
        # The recorded model's largest probability was approximately 32%.
        record = intersection_entry_decision(
            ACTIONS, [.32, .16, .12, .11, .1, .08, .06, .05], lidar_ready=True)
        self.assertTrue(record['clear'])
        self.assertEqual(record['action'], 'KEEP_LANE')
        self.assertEqual(record['probability'], .32)
        for action in ('TURN_LEFT', 'TURN_RIGHT'):
            self.assertTrue(self.decision(action)['clear'])

    def test_wait_unknown_and_lane_change_actions_do_not_veto_clear_traffic(self):
        for action in ('STOP', 'YIELD', 'UNKNOWN', 'LEFT_CHANGE', 'RIGHT_CHANGE'):
            with self.subTest(action=action):
                record = self.decision(action)
                self.assertTrue(record['valid_model'])
                self.assertTrue(record['clear'])
                self.assertFalse(record['model_permits_entry'])

    def test_invalid_model_output_is_advisory_but_cannot_override_bad_lidar(self):
        valid = probabilities('KEEP_LANE')
        bad_vectors = [None, [], [.9], [0.] * 8, [.2] * 8,
                       [float('nan')] + valid[1:], [float('inf')] + valid[1:],
                       [-.1] + valid[1:], [1.1] + valid[1:],
                       ['bad'] + valid[1:]]
        for values in bad_vectors:
            with self.subTest(values=values):
                record = intersection_entry_decision(ACTIONS, values, lidar_ready=True)
                self.assertTrue(record['clear'])
                self.assertFalse(record['valid_model'])
                self.assertFalse(intersection_entry_decision(ACTIONS, values, lidar_ready=False)['clear'])
        record = intersection_entry_decision(['MISSING'] + ACTIONS[1:], valid,
                                              lidar_ready=True)
        self.assertTrue(record['clear'])
        self.assertFalse(record['valid_model'])

    def test_ambiguous_model_stays_advisory(self):
        values = [.05] * 8
        values[0], values[5] = .35, .35
        record = intersection_entry_decision(ACTIONS, values, lidar_ready=True)
        self.assertTrue(record['valid_model'])
        self.assertTrue(record['clear'])
        self.assertFalse(record['model_permits_entry'])

    def test_lidar_stale_and_traffic_override_every_model_action(self):
        cases = [dict(lidar_ready=False), dict(fresh=False),
                 dict(emergency=True), dict(yielding=dict(kind='stop')),
                 dict(yielding=dict(kind='slow')),
                 dict(yielding=dict(kind='follow', source='crossing_caution', v_mps=1.))]
        for case in cases:
            with self.subTest(case=case):
                params = dict(lidar_ready=True)
                params.update(case)
                record = intersection_entry_decision(
                    ACTIONS, probabilities('KEEP_LANE'), **params)
                self.assertFalse(record['clear'])
        record = self.decision(yielding=dict(kind='follow', source='traffic',
                                             reason='lead vehicle headway', v_mps=2.))
        self.assertTrue(record['clear'])
        self.assertTrue(self.decision('STOP', model_fresh=False)['clear'])

    def test_one_second_stop_and_clear_observations_run_in_parallel(self):
        gate = StopLineController([LINE])
        decision = self.decision()
        for step in range(10):
            record = gate.update(STATION, 0., step / 10., decision['clear'])
            self.assertTrue(record['stop'])
        record = gate.update(STATION, 0., 1., decision['clear'])
        self.assertEqual(record['state'], 'ENTER')
        self.assertFalse(record['stop'])

    def test_creeping_and_sensor_observation_gaps_do_not_count_as_full_stop(self):
        gate = StopLineController([LINE])
        for step in range(21):
            self.assertTrue(gate.update(STATION, .16, step / 10., True)['stop'])
        gate.update(STATION, 0., 2.1, True)
        self.assertTrue(gate.update(STATION, 0., 2.8, True)['stop'])
        for step in range(1, 10):
            self.assertTrue(gate.update(STATION, 0., 2.8 + step / 10., True)['stop'])
        self.assertEqual(gate.update(STATION, 0., 3.8, True)['state'], 'ENTER')

    def test_crossing_car_then_clear_gap_releases_without_another_dwell(self):
        gate = StopLineController([LINE])
        wait = self.decision('KEEP_LANE', yielding=dict(kind='stop'))
        go = self.decision('STOP')
        for step in range(21):
            record = gate.update(STATION, 0., step / 10., wait['clear'])
            self.assertTrue(record['stop'])
        self.assertEqual(gate.update(STATION, 0., 2.1, go['clear'])['state'], 'ENTER')
        # Mandatory-stop release stays latched; measured collision veto remains
        # the Runtime's independent emergency input, not the model's label.
        for action in ('STOP', 'UNKNOWN', 'LEFT_CHANGE'):
            clear = self.decision(action)['clear']
            self.assertEqual(gate.update(STATION, 0., 3.2, clear)['state'], 'ENTER')


if __name__ == '__main__':
    unittest.main()
