"""Observed highway traffic keeps its road independently of ego admission."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

from gui import HdMapLanes
from highway_planner import HighwayPlanner, local_points, timed_gap, world_points
from highway_sensor_alignment import align_measured_cars
from highway_speed_plan import clearance_forecast_bound
from planning.highway.lane_map import HighwayLaneMap
from planning.highway.motion_evidence import HighwayMotionEvidence
from planning.highway.policy import ThreeLanePolicy
from sim_time import to_sim_objects


HERE = Path(__file__).parent


def car(x, y, vx=10., vy=0., uid=1):
    return (x, y, 4.9, 'lidar_car', 'npc', math.hypot(vx, vy),
            4.9, 2.1, math.degrees(math.atan2(vy, vx)), vx, vy,
            uid, 0., None, True, 0.)


def synthetic_map(actor_y=4., children=()):
    """An actor road disconnected from the mission's available ego road."""
    exit_points = np.column_stack((np.arange(-200., 1001.), np.zeros(1201)))
    actor_points = np.column_stack((np.arange(-200., 101.), np.full(301, actor_y)))
    geometry = {'exit': exit_points, 'actor': actor_points}
    links = [dict(idx='exit', from_node_idx='e0', to_node_idx='e1',
                  width_start=3.5, width_end=3.5),
             dict(idx='actor', from_node_idx='a0', to_node_idx='a1',
                  width_start=3.5, width_end=3.5)]
    successors = {'e0': ['exit'], 'a0': ['actor']}
    for key, points in children:
        geometry[key] = np.asarray(points, dtype=float)
        links.append(dict(idx=key, from_node_idx='a1', to_node_idx=key+'end',
                          width_start=3.5, width_end=3.5))
        successors.setdefault('a1', []).append(key)
    points = np.vstack([geometry[l['idx']] for l in links])
    link_of = np.concatenate([np.full(len(geometry[l['idx']]), k, dtype=int)
                              for k, l in enumerate(links)])
    rules = {'mode': 'whitelist', 'allowed_links': ['exit']}
    lanes = SimpleNamespace(links=links, geometry=geometry, successors=successors,
        predecessors={'e0': [], 'e1': ['exit']}, tree=cKDTree(points), link_of=link_of,
        link_by_id={l['idx']: l for l in links}, route_xy=exit_points,
        blocked=lambda key: key != 'exit', rules=lambda: rules,
        adjacent_link=lambda link, side: None, local_zones=lambda ego: [])
    return HighwayLaneMap(lanes, dict(end_link='exit', excluded_lane_links=['actor']))


class ActorLaneTopologyTests(unittest.TestCase):
    def setUp(self):
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=100.)

    def test_excluded_whitelist_actor_remains_visible_without_ego_admission(self):
        native = synthetic_map()
        route = np.column_stack((np.arange(0., 300.), np.zeros(300)))
        before = native.candidates(self.ego, route)
        corridor = native.corridor(self.ego, route, before)
        self.assertTrue(native.blocked('actor'))
        self.assertIsNone(native.chain('actor'))
        motion = native.motion_candidates(self.ego, [car(20., 4.)])
        self.assertEqual([lane.link_id for lane in motion], ['actor'])
        self.assertEqual(motion[0].chain, ('actor',))
        self.assertLess(motion[0].centre[0, 0], 0.)
        after = native.candidates(self.ego, route)
        self.assertEqual([lane.link_id for lane in before], ['exit'])
        self.assertEqual([lane.link_id for lane in after], ['exit'])
        for a, b in zip(before, after):
            np.testing.assert_array_equal(a.centre, b.centre)
        self.assertEqual(corridor.clear_distance(route, 170.),
                         native.corridor(self.ego, route, after).clear_distance(route, 170.))

    def test_unique_continuous_native_successor_is_retained_and_cached(self):
        child = np.column_stack((np.arange(100., 301.), np.full(201, 4.)))
        native = synthetic_map(children=(('continue', child),))
        first = native.observed_motion_chain('actor')
        self.assertEqual(first[0], ('actor', 'continue'))
        self.assertIs(native.observed_motion_chain('actor'), first)
        np.testing.assert_array_equal(first[1][-1], child[-1])

    def test_ambiguous_successors_are_not_guessed(self):
        straight = np.column_stack((np.arange(100., 201.), np.full(101, 4.)))
        turning = np.column_stack((np.arange(100., 201.), 4.+np.arange(101)*.1))
        native = synthetic_map(children=(('straight', straight), ('turn', turning)))
        chain, geometry = native.observed_motion_chain('actor')
        self.assertEqual(chain, ('actor',))
        np.testing.assert_array_equal(geometry, native.lanes.geometry['actor'])

    def test_disconnected_or_sharp_successor_does_not_create_a_join(self):
        disconnected = np.column_stack((np.arange(101., 202.), np.full(101, 4.)))
        sharp = np.column_stack((np.full(101, 100.), np.arange(4., 105.)))
        for child in (disconnected, sharp):
            with self.subTest(child=child[-1].tolist()):
                native = synthetic_map(children=(('child', child),))
                self.assertEqual(native.observed_motion_chain('actor')[0], ('actor',))

    def test_small_survey_seam_preserves_a_unique_successor(self):
        child = np.column_stack((np.arange(100., 201.)+.034, np.full(101, 4.)))
        native = synthetic_map(children=(('child', child),))
        self.assertEqual(native.observed_motion_chain('actor')[0], ('actor', 'child'))

    def test_opposite_or_invalid_survey_keeps_linear_forecasting(self):
        for kind in ('opposite', 'nan', 'degenerate'):
            with self.subTest(kind=kind):
                native = synthetic_map()
                if kind == 'opposite':
                    native.lanes.link_by_id['actor']['opp_traffic'] = True
                elif kind == 'nan':
                    native.lanes.geometry['actor'][0, 0] = float('nan')
                else:
                    native.lanes.geometry['actor'] = np.array([[1., 4.], [1., 4.]])
                self.assertIsNone(native.observed_motion_chain('actor'))

    def test_fallback_does_not_accept_unknown_or_opposite_vehicle_motion(self):
        native = synthetic_map()
        route = np.column_stack((np.arange(0., 1000.), np.zeros(1000)))
        config = dict(from_idx=0, to_idx=900, kmh=90., lane_change_policy='free_three_lane')
        unknown = list(car(20., 4.)); unknown[14] = False
        for obj in (tuple(unknown), car(20., 4., vx=-10.)):
            with self.subTest(velocity=obj[9:11], valid=obj[14]):
                planner = HighwayPlanner(config, route)
                ThreeLanePolicy(config).update_motion_paths(planner,
                    native.motion_candidates(self.ego, [obj]), self.ego, [obj])
                self.assertNotIn(obj[11], planner.longitudinal.motion_paths)


class RecordedActorLaneReplay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads((HERE/'test_data/highway_actor_lane_20261012.json').read_text())
        cls.route = np.array([[p['x'], p['y']] for p in
            json.loads((HERE/'global_path.json').read_text())['waypoints']])
        cls.lanes = HdMapLanes(HERE/'link_set.json')
        cls.lanes.set_route(cls.route)

    def replay(self, fallback):
        config = self.fixture['config']
        native = HighwayLaneMap(self.lanes, config)
        evidence = HighwayMotionEvidence()
        results = []
        for case in self.fixture['cases']:
            ego = case['ego']
            source = tuple(case['source_actor'])
            # Isolate the newly mapped actor. Other cars retain the exact
            # planning-time tuples already checked by the recorded runtime.
            with patch.object(native, 'observed_motion_chain',
                    wraps=native.observed_motion_chain) as observed:
                if not fallback:
                    observed.side_effect = lambda seed: None
                lanes = native.motion_candidates(ego, [source])
            cleaned, diagnostics = evidence.update([source], case['source_stamp'], lanes, ego)
            aligned, _ = align_measured_cars(cleaned, case['source_stamp'], ego['stamp'])
            objects = [aligned[0] if obj[11] == 500015 else tuple(obj)
                       for obj in case['objects']]
            before = np.asarray(source[6:8])
            np.testing.assert_array_equal(aligned[0][6:8], before)
            objects = to_sim_objects(objects, case['rtf'])
            planner = HighwayPlanner(config, self.route)
            planner.longitudinal.time_scale = case['rtf']
            planner.longitudinal.sensor_age_s = case['sensor_age_s']
            planner.longitudinal.fusion_sensor_age_s = case['fusion_sensor_age_s']
            planner.longitudinal.begin(ego)
            world = np.asarray(case['world_path'])
            planner.saved_world = world.copy()
            planner.status['command_footprint_margin_m'] = case['command_footprint_margin_m']
            motion = native.motion_candidates(ego, objects)
            if not fallback:
                motion = [lane for lane in motion if lane.link_id != 'A2256W000179']
            ThreeLanePolicy(config).update_motion_paths(planner, motion, ego, objects)
            result = planner.plan_speed(local_points(world, ego), ego, objects, 90./3.6,
                static_clear_m=clearance_forecast_bound(case['static_clear_m'], case['query_m']),
                ceiling_mps=120./3.6)
            results.append((result, planner, diagnostics))
        return results

    def test_four_recorded_false_intrusions_release_after_measured_motion_evidence(self):
        baseline = self.replay(False)
        corrected = self.replay(True)
        self.assertEqual(len(baseline), 4)
        for case, old, new in zip(self.fixture['cases'], baseline, corrected):
            with self.subTest(stamp=case['log_stamp']):
                self.assertTrue(old[0]['emergency'])
                self.assertEqual(old[0]['reference_mps'], 0.)
                self.assertNotIn(500015, old[1].longitudinal.motion_paths)
                self.assertFalse(new[0]['emergency'])
                self.assertTrue(new[0]['diagnostics']['reference_checked'])
                self.assertGreater(new[0]['reference_mps'], 0.)
                self.assertIn(500015, new[1].longitudinal.motion_paths)
                self.assertEqual(new[0]['diagnostics']['command_footprint_margin_m'], .6)
        self.assertEqual(corrected[-1][2]['vehicles'][0]['evidence'], 'uncertain lateral motion')
        self.assertEqual(corrected[-1][2]['vehicles'][0]['used_lateral_mps'], 0.)

    def test_existing_ego_chain_motion_geometry_is_unchanged(self):
        case = self.fixture['cases'][0]
        native = HighwayLaneMap(self.lanes, self.fixture['config'])
        objects = [tuple(obj) for obj in case['objects'] if obj[11] != 500015]
        motion = native.motion_candidates(case['ego'], objects)
        self.assertEqual(len(motion), 2)
        for lane in motion:
            chain, geometry, _ = native.chain(lane.link_id)
            self.assertEqual(lane.chain, chain)
            np.testing.assert_array_equal(lane.centre, local_points(geometry, case['ego']))


class ActorLaneSafetyTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 1000.), np.zeros(1000)))
        self.config = dict(from_idx=0, to_idx=900, kmh=90.,
            lane_change_policy='free_three_lane', comfort_decel_mps2=2.5)

    def plan(self, ego, objects, native, static_clear=float('inf')):
        planner = HighwayPlanner(self.config, self.route)
        planner.longitudinal.begin(ego)
        planner.saved_world = world_points(self.route, ego)
        planner.status['command_footprint_margin_m'] = .6
        ThreeLanePolicy(self.config).update_motion_paths(planner,
            native.motion_candidates(ego, objects), ego, objects)
        trajectory = planner.longitudinal.preview(self.route, ego, objects, 25.).prefix(3.)
        risk = timed_gap(self.route, ego, objects, 25., lane_change=True,
            speed_trajectory=trajectory, object_positions=planner.longitudinal.object_positions,
            object_yaws=planner.longitudinal.object_yaws, ignore_rear_approach=False,
            footprint_margin_m=.6)
        result = planner.plan_speed(self.route, ego, objects, 25., static_clear_m=static_clear)
        return result, risk, planner

    def test_real_cut_in_keeps_observed_turn_and_future_body_collision(self):
        native = synthetic_map()
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=101.4)
        evidence = HighwayMotionEvidence()
        for i in range(15):
            t=i*.1
            obj = car(8.+8.3*t, 4.-1.2*t, vx=8.3, vy=-1.2)
            cleaned, diagnostic = evidence.update([obj], 100.+t,
                native.motion_candidates(ego, [obj]), ego)
        self.assertEqual(diagnostic['vehicles'][0]['evidence'], 'observed turn')
        self.assertAlmostEqual(cleaned[0][10], -1.2)
        result, risk, planner = self.plan(ego, cleaned, native)
        self.assertIn(obj[11], planner.longitudinal.motion_paths)
        self.assertIsNotNone(risk['physical_collision_when'])
        self.assertGreater(risk['physical_collision_when'], 0.)
        self.assertLessEqual(risk['physical_collision_when'], 3.)
        self.assertTrue(result['emergency'])
        self.assertEqual(result['reference_mps'], 0.)

    def test_current_body_overlap_on_actor_road_still_stops(self):
        native = synthetic_map()
        ego = dict(x=0., y=2., heading_deg=0., speed_kmh=90., stamp=100.)
        result, risk, planner = self.plan(ego, [car(2., 3.9)], native)
        self.assertIn(1, planner.longitudinal.motion_paths)
        self.assertEqual(risk['physical_collision_when'], 0.)
        self.assertTrue(result['emergency'])
        self.assertEqual(result['reference_mps'], 0.)

    def test_fast_native_rear_body_collision_cannot_be_waived(self):
        native = synthetic_map()
        ego = dict(x=0., y=2.2, heading_deg=0., speed_kmh=90., stamp=100.)
        result, risk, planner = self.plan(ego, [car(-12., 4., vx=40.)], native)
        self.assertIn(1, planner.longitudinal.motion_paths)
        self.assertTrue(risk['rear_body_conflict'])
        self.assertIsNotNone(risk['physical_collision_when'])
        self.assertTrue(result['emergency'])
        self.assertEqual(result['reference_mps'], 0.)

    def test_actual_static_hit_still_stops_with_the_actor_fallback(self):
        native = synthetic_map()
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=100.)
        result, _, planner = self.plan(ego, [car(20., 4.)], native, static_clear=0.)
        self.assertIn(1, planner.longitudinal.motion_paths)
        self.assertTrue(result['emergency'])
        self.assertEqual(result['reference_mps'], 0.)


if __name__ == '__main__':
    unittest.main()
