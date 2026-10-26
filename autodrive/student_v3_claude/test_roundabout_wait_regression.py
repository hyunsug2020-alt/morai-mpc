"""Replay the four-car standstill from competition_08 on 2026-10-21."""
import json
import math
from pathlib import Path
import unittest

import numpy as np

from sim_time import to_sim_objects
from stop_line import INTERSECTION_ACCEL_MPS2, yield_line_cap
from traffic_planner import GapPlanner
from test_roundabout_entry_standoff import car, roundabout_map


DATA = json.loads((Path(__file__).parent / 'test_data' /
                   'roundabout_wait_20261021.json').read_text())


def recorded_scene(following=True, advance=0.):
    planner = GapPlanner(roundabout_map()['links'])
    planner.ACCEL_MPS2 = INTERSECTION_ACCEL_MPS2
    planner.junction = dict(held=True, ready=True, following_only=following,
        yield_only=not following, empty_actual_body=True, unknown_ids=[],
        line_gap_m=.949-advance)
    ego = dict(DATA['ego'])
    ego['y'] += advance
    yaw = math.radians(ego['heading_deg'])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    path = (np.asarray(DATA['world_path'])-[ego['x'], ego['y']])@rotation
    objects = []
    for vehicle in DATA['vehicles']:
        row = list(car(*vehicle['position'], *vehicle['velocity'], vehicle['uid'],
            length=vehicle['size'][0], width=vehicle['size'][1],
            heading=vehicle['heading_deg'], valid=vehicle['velocity_valid']))
        row[15] = vehicle.get('prediction_age_s', 0.)
        objects.append(tuple(row))
        planner.speed_ratio[vehicle['uid']] = 1.1
    return planner, ego, path, to_sim_objects(objects, DATA['rtf'])


class RecordedRoundaboutWaitTests(unittest.TestCase):
    def evaluate(self, following=True, advance=0., fresh=True):
        planner, ego, path, objects = recorded_scene(following, advance)
        result = planner.evaluate(objects, ego, path, 30./3.6, DATA['wall'], fresh=fresh)
        return planner, ego, result

    def test_recorded_line_wait_keeps_rejecting_the_gap(self):
        planner, _, _ = self.evaluate(False)
        self.assertEqual(planner.status['ring']['action'], 'WAIT')
        self.assertLess(planner.status['ring']['margin_s'], 0.)

    def test_occupied_ring_allows_only_a_brakeable_approach(self):
        planner, ego, result = self.evaluate()
        self.assertEqual(planner.status['ring']['action'], 'WAIT')
        self.assertEqual(planner.state, 'CROSSING_CAUTION')
        self.assertGreater(result['v_mps'], .5)
        gap = planner.status['contact_gap_m']
        self.assertGreater(gap, 7.)
        self.assertLessEqual(result['v_mps'], yield_line_cap(gap, 0., ego['age_s']))
        self.assertFalse(planner.junction_committed)

    def test_occupied_physical_corridor_stops_before_entry(self):
        planner, _, result = self.evaluate(advance=9.)
        self.assertEqual(planner.status['ring']['action'], 'WAIT')
        self.assertEqual(result['v_mps'], 0.)
        self.assertFalse(planner.junction_committed)

    def test_lost_measured_stream_stops_the_recorded_approach(self):
        _, _, result = self.evaluate(fresh=False)
        self.assertEqual(result['v_mps'], 0.)


if __name__ == '__main__':
    unittest.main()
