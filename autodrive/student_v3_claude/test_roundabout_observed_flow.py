"""Measured circulation forecasts retain cold-start and crossing safeguards."""
import json
import math
from pathlib import Path
import unittest

import numpy as np

import ring_entry
from junction_vehicle_motion import source_motion_rows
from sim_time import to_sim_objects
from traffic_planner import GapPlanner, JunctionTrackMemory
from test_roundabout_entry_standoff import car, roundabout_map


class ObservedFlowTests(unittest.TestCase):
    def test_position_fit_cannot_drop_the_upper_bound_below_recent_measured_ring_speed(self):
        tracker = ring_entry.RingMotionTracker()
        for t in np.arange(0., 2.01, .1):
            tracker.update([car(3.*t, 0., 4., 0., 7)], {7:100.+t})
        self.assertAlmostEqual(tracker.fitted[7], 3.)
        self.assertGreaterEqual(tracker.fitted[7]*tracker.ratio[7], 4.)

    def test_constant_ring_speed_uses_its_existing_measurement_uncertainty(self):
        obj = car(0., 0., 4., 0., 7)
        conflict = dict(arc_cp=19.)
        circulating = ring_entry.RingCar(obj, 4., 1.1, 'moving', conflict, observed_flow=True)
        front = .5*math.hypot(4.9, 2.09)+.1
        self.assertAlmostEqual(circulating.earliest_cp(), (19.-front)/4.4)
        ordinary = ring_entry.RingCar(obj, 4., 1.1, 'moving', conflict)
        self.assertLess(ordinary.earliest_cp(), circulating.earliest_cp())
        stopped = ring_entry.RingCar(obj, 0., 1.1, 'standing', conflict, observed_flow=True)
        self.assertAlmostEqual(stopped.earliest_cp(), ring_entry.resume_time(19., 0.))

    def test_straight_crossing_does_not_become_observed_circulation(self):
        frame = ring_entry.PathFrame(np.column_stack((np.arange(0., 40.01, .5), np.zeros(81))))
        lane = np.column_stack((np.full(121, 10.), np.arange(30., -30.01, -.5)))
        arc = np.arange(0., 60.01, .5)
        cars, _ = ring_entry.classify([car(10., 30., 0., -3., 7)], frame, 0.,
            lambda *args:[(lane, arc)], observed_flow_ids={7})
        self.assertFalse(cars[0].observed_flow)

    def test_recorded_close_gap_is_rejected_with_front_body_arrival_and_speed_uncertainty(self):
        data = json.loads((Path(__file__).parent/'test_data'/'roundabout_observed_flow_20261021.json').read_text())
        world = np.asarray(data['world_path'])
        planner = GapPlanner(roundabout_map()['links'])
        tracker, memory = ring_entry.RingMotionTracker(), JunctionTrackMemory()
        opened = 0
        for sample in data['frames']:
            rows, stamps = [], {}
            for vehicle in sample['vehicles']:
                row = list(car(*vehicle['position'], *vehicle['velocity'], vehicle['uid'],
                    length=vehicle['size'][0], width=vehicle['size'][1],
                    heading=vehicle['heading_deg'], valid=vehicle['velocity_valid']))
                row[15] = vehicle.get('prediction_age_s', 0.)
                rows.append(tuple(row))
                if not row[15]:
                    stamps[vehicle['uid']] = vehicle['stamp']
            rows, _ = source_motion_rows(rows, tracker, stamps)
            measured = [row for row in rows if not row[15]]
            forecasts = [row for row in rows if row[15]]
            rows = memory.update(measured, sample['t'], sample['ego'], planner.map_predictor)
            known = {row[11] for row in rows}
            rows += [row for row in forecasts if row[11] not in known]
            planner.speed_ratio = dict(tracker.ratio)
            planner.junction = dict(following_only=True, ready=True, empty_actual_body=True,
                constant_ring_speed=True,
                unknown_ids=memory.unknown_ids, ghost_ids=memory.ghost_ids,
                inherited_ids=memory.inherited_ids)
            ego = dict(x=sample['ego'][0], y=sample['ego'][1], heading_deg=sample['heading_deg'],
                       speed_kmh=0., age_s=.02)
            yaw = math.radians(ego['heading_deg'])
            rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            result = planner.evaluate(to_sim_objects(rows, sample['rtf']), ego,
                (world-sample['ego'])@rotation, 30./3.6, sample['t'])
            ring = planner.status.get('ring', {})
            if ring.get('action')=='GO' and result['v_mps']>0.:
                opened += 1
                self.assertTrue(any(check['arrival_model']=='constant_velocity' for check in ring['checks']))
                self.assertTrue(all(check['margin_s']>=ring_entry.NEXT_MARGIN_S for check in ring['checks']))
        self.assertEqual(opened, 0)

    def test_physical_entry_accepts_a_wide_constant_speed_gap_and_rejects_a_close_one(self):
        frame = ring_entry.PathFrame(np.column_stack((np.arange(0., 40.01, .5), np.zeros(81))))
        obj = car(-10., 10., 4.17, 0., 7)
        for distance, expected in ((23., 'GO'), (10., 'WAIT')):
            conflict = dict(arc_cp=distance, arc_lane=distance-2., arc_exit=None,
                            s_lane=4.6, s_cp=6.5)
            vehicle = ring_entry.RingCar(obj, 4.17, 1.1, 'moving', conflict, observed_flow=True)
            result = ring_entry.entry_decision([vehicle], 0., 0., 30./3.6,
                frame=frame, yield_only=True, body_geometry=True)
            self.assertEqual(result['action'], expected)
            if expected=='GO':
                self.assertTrue(all(c['margin_s']>=ring_entry.NEXT_MARGIN_S for c in result['checks']))

    def test_constant_speed_model_is_explicit_and_excludes_inherited_sources(self):
        from test_roundabout_wait_regression import recorded_scene
        for enabled, inherited in ((False, False), (True, False), (True, True)):
            planner, ego, path, objects = recorded_scene(False)
            planner.junction['constant_ring_speed'] = enabled
            if inherited:
                planner.junction['inherited_ids'] = {o[11] for o in objects}
            planner.evaluate(objects, ego, path, 30./3.6, 100.)
            models = {check['arrival_model'] for check in planner.status['ring']['checks']}
            self.assertEqual('constant_velocity' in models, enabled and not inherited)

    def test_physical_yield_boundary_keeps_the_original_source_and_stops_before_ring(self):
        import gui
        from unittest.mock import patch
        route = roundabout_map()['route']
        station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        line = dict(id='A1256W000709:1781', node_id='A1256W000709',
                    station_m=885.1006875597407, xy=[-99.31267240812303,317.7989547071047])
        with patch.object(gui, 'YIELD_AT_JUNCTION', True), patch.object(gui, 'RING_CONSTANT_SPEED', True):
            controller = gui.junction_controller([line], route)
        self.assertEqual(line['station_m'], 885.1006875597407)
        shifted = controller.lines[0]
        self.assertAlmostEqual(shifted['station_m']-line['station_m'], 7.)
        # The recorded physical hold point has 0.75 m remaining contact gap;
        # the new buffered target stays behind that axle station.
        physical_hold = np.array([-98.49, 320.26])
        nearest = int(np.argmin(np.linalg.norm(route-physical_hold, axis=1)))
        from ego_body import EGO_FRONT_M
        self.assertLess(shifted['station_m']-EGO_FRONT_M-controller.BUFFER_M, station[nearest])


if __name__=='__main__':
    unittest.main()
