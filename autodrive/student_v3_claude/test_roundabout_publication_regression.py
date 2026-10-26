"""A departing ring car cannot become a frozen obstacle on the launch path."""
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from driving_plan import DrivingPlan
import ring_entry
from stop_line import competition_stop_controller, intersection_front_station, yield_line_cap
import test_junction_live_profile as profile_fixture
from test_roundabout_entry_standoff import car, roundabout_map
from traffic_planner import GapPlanner


DATA = json.loads((Path(__file__).parent / 'test_data' /
                   'roundabout_publication_20261021.json').read_text())


class RecordedPublicationTests(unittest.TestCase):
    def scene(self, *, departing_only=False, known=True):
        helper = profile_fixture.JunctionLiveProfileTests()
        runtime, _, _ = helper.fixture(following=False)
        world = np.asarray(DATA['world_path'])
        ego = dict(DATA['ego'], stamp=DATA['wall'], age_s=.02, received=99.98)
        runtime.test_status['ego'].update(ego)
        runtime.test_route = world
        line_station = 892.1006875597407
        runtime.gap_planner = GapPlanner(roundabout_map()['links'])
        runtime.gap_planner.junction = dict(yield_only=True, constant_ring_speed=True,
            ready=True, held=True, empty_actual_body=True,
            line_station_m=line_station, line_id='entry')
        frame = ring_entry.PathFrame(world)
        origin = float(frame.project([[ego['x'], ego['y']]])[0][0])
        runtime.lanes.route_progress = lambda pose: (887.50765 +
            float(frame.project([[pose['x'], pose['y']]])[0][0]) - origin)
        line = dict(id='entry', node_id='A1256W000709', station_m=line_station,
                    xy=[-96.686, 324.217])
        runtime.stop_line_controller = competition_stop_controller([line])
        front = intersection_front_station(ego, runtime.lanes.route_progress)
        runtime.stop_line_controller.update(runtime.lanes.route_progress(ego), 0.,
            100., False, observation_age_s=.02, empty_front_station=front)
        runtime.sim_clock = SimpleNamespace(rtf=.988)
        runtime.object_receiver.objects = [tuple(row) for row in DATA['objects']]
        semantic_stamp = DATA['semantic_stamp']
        runtime.object_receiver.observation_stamp = semantic_stamp
        runtime.object_receiver.snapshot_with_time = lambda: (
            runtime.object_receiver.objects, semantic_stamp, .2)
        vehicles = [car(*v['position'], *v['velocity'], v['uid'],
            length=v['size'][0], width=v['size'][1], heading=v['heading_deg'],
            valid=known and v['velocity_valid']) for v in DATA['vehicles']['vehicles']
            if not departing_only or v['uid'] == 500004]
        vehicle_stamp = DATA['vehicles']['stamp']
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
            vehicles, True, dict(DATA['vehicles'], age_s=.12)))
        runtime.test_scan.update(stamp=vehicle_stamp, age_s=.12)
        plan = DrivingPlan(1, world, ego, ego, vehicle_stamp, .12, 100.,
            30./3.6, 60., False, dict(valid=True, pose_valid=True,
                steering_valid=True, blocked=False,
                junction_body_cap_mps=30./3.6, junction_gap_checked=True), 'ROUTE BASE')
        return helper, runtime, plan, world

    def check(self, scene):
        helper, runtime, plan, world = scene
        with patch('time.monotonic', return_value=100.):
            return helper.check(runtime, plan, world)

    def test_recorded_departing_bodies_do_not_block_the_whole_future_corridor(self):
        result = self.check(self.scene())
        self.assertGreater(result.validation['empty_entry_semantic_clear_m'], 50.)
        self.assertGreater(result.validation['live_safety']['current_body_cap_mps'], 0.)
        self.assertIn(('fusion_npc', 50), result.validation['junction_moving_body_keys'])
        # The cold incoming speed bounds still reject this particular gap.
        self.assertEqual(result.validation['live_safety']['junction_traffic']['ring']['action'], 'WAIT')
        traffic = result.validation['live_safety']['junction_traffic']
        self.assertLessEqual(result.allowed_mps, yield_line_cap(traffic['front_gap_m'], 0., .02))
        self.assertFalse(traffic['gap_checked'])

    def test_departed_ring_car_without_incoming_traffic_allows_a_fresh_launch(self):
        result = self.check(self.scene(departing_only=True))
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)
        self.assertTrue(result.validation['live_safety']['junction_traffic']['gap_checked'])

    def test_unmeasured_vehicle_cannot_lose_its_frozen_body_guard(self):
        result = self.check(self.scene(departing_only=True, known=False))
        self.assertNotIn(('lidar_car', 500004), result.validation['junction_moving_body_keys'])
        self.assertNotIn(('fusion_npc', 50), result.validation['junction_moving_body_keys'])
        self.assertEqual(result.allowed_mps, 0.)

    def test_raw_contact_still_stops_the_departing_car_launch(self):
        scene = self.scene(departing_only=True)
        scene[1].test_scan['xyz'] = np.array([[2.5, 0., .6]])
        result = self.check(scene)
        self.assertTrue(result.emergency)
        self.assertEqual(result.allowed_mps, 0.)

    def test_committed_launch_keeps_departing_vehicle_motion_ownership(self):
        scene = self.scene(departing_only=True)
        scene[1].gap_planner.junction_committed = True
        result = self.check(scene)
        self.assertGreater(result.validation['empty_entry_semantic_clear_m'], 50.)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)

    def test_committed_launch_retains_unknown_and_current_contact_guards(self):
        scene = self.scene(departing_only=True, known=False)
        scene[1].gap_planner.junction_committed = True
        self.assertEqual(self.check(scene).allowed_mps, 0.)
        scene = self.scene(departing_only=True)
        scene[1].gap_planner.junction_committed = True
        scene[1].test_scan['xyz'] = np.array([[2.5, 0., .6]])
        self.assertEqual(self.check(scene).allowed_mps, 0.)

    def test_departing_camera_alias_returns_do_not_create_a_false_launch_ceiling(self):
        for committed in (False, True):
            with self.subTest(committed=committed):
                scene = self.scene(departing_only=True)
                scene[1].gap_planner.junction_committed = committed
                # A real return inside the fresh matched camera footprint,
                # outside the narrower PP body and outside the actual ego.
                scene[1].test_scan['xyz'] = np.array([[5.25, -1.5, .6]])
                result = self.check(scene)
                self.assertGreater(result.validation['live_safety']['raw_clear_m'], 50.)
                self.assertFalse(result.emergency, result.stop_reason)
                self.assertGreater(result.allowed_mps, 5.)

    def shifted_committed_scene(self, distance, known=True):
        scene = self.scene(departing_only=True, known=known)
        scene[1].gap_planner.junction_committed = True
        ego = scene[1].test_status['ego']
        heading = np.radians(ego['heading_deg'])
        ego['x'] += distance*np.cos(heading)
        ego['y'] += distance*np.sin(heading)
        return scene

    def test_inflated_camera_alias_cannot_cancel_a_clear_measured_committed_launch(self):
        result = self.check(self.shifted_committed_scene(.25))
        self.assertGreater(result.validation['junction_current_semantic_clear_m'], .1)
        self.assertIn(['fusion_npc', 50, 500004], result.validation['junction_current_body_aliases'])
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertGreater(result.allowed_mps, 0.)

    def test_fused_body_contact_and_unmeasured_alias_still_stop(self):
        for distance, known in ((.5, True), (.25, False)):
            with self.subTest(distance=distance, known=known):
                result = self.check(self.shifted_committed_scene(distance, known))
                self.assertEqual(result.allowed_mps, 0.)


if __name__ == '__main__':
    unittest.main()
