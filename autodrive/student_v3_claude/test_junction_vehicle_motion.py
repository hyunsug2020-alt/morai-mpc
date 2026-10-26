"""Publication-bound map timing cannot erase current contact or unknown motion."""
from dataclasses import FrozenInstanceError, replace
import copy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from junction_vehicle_motion import (certify_moving_cars, confirm_position_motion,
                                     proof_matches, source_motion_rows)
from ring_entry import FALLBACK_RATIO, RingMotionTracker
from roundabout_empty import semantic_entry_clearance
from sim_time import to_sim_objects
from test_junction_entry import car
from traffic_planner import GapPlanner


class JunctionVehicleMotionTests(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0., stamp=100., age_s=.02, received=9.98)
        self.vehicle = car(15., 0., 3., 0., 7)
        self.planner = self.mapped_planner(self.path)

    def mapped_planner(self, points):
        planner = GapPlanner([dict(points=points.tolist(), from_node_idx='a', to_node_idx='b')])
        planner.junction = dict(following_only=True, ready=True, empty_actual_body=True)
        return planner

    def certify(self, *, planner=None, ego=None, path=None, cars=None, batches=None, **overrides):
        arguments = dict(rtf=.8, now=10., vehicle_age_s=.1, lidar_age_s=.1, requested_mps=24./3.6)
        arguments.update(overrides)
        return certify_moving_cars(planner or self.planner, self.ego if ego is None else ego,
            self.path if path is None else path, [self.vehicle] if cars is None else cars,
            [('vehicles', [self.vehicle], 99.9, .1)] if batches is None else batches, **arguments)

    def test_actual_planner_go_is_rechecked_without_mutating_original(self):
        self.planner.status = {'state': 'WAIT'}
        self.planner.junction_committed = False
        self.planner.speed_ratio = {7: 1.1}
        proof = self.certify()
        self.assertIsNotNone(proof)
        self.assertEqual(proof.covered_keys, frozenset({('lidar_car', 7)}))
        self.assertGreater(proof.cap_mps, 0.)
        self.assertGreater(proof.current_clear_m, 7.)
        self.assertEqual(self.planner.status, {'state': 'WAIT'})
        self.assertFalse(self.planner.junction_committed)
        self.assertEqual(self.planner.speed_ratio, {7: 1.1})
        with self.assertRaises(FrozenInstanceError):
            proof.cap_mps = 100.

    def test_constant_ring_yield_uses_a_new_assessment_without_consuming_entry(self):
        self.planner.junction = dict(yield_only=True, constant_ring_speed=True,
            ready=True, held=True, empty_actual_body=True, line_gap_m=.9)
        proof = self.certify()
        self.assertIsNotNone(proof)
        self.assertEqual(proof.covered_keys, frozenset({('lidar_car', 7)}))
        self.assertFalse(self.planner.junction_committed)
        self.assertEqual(self.planner.status, {})
        for change in (dict(constant_ring_speed=False), dict(recovery_only=True)):
            self.planner.junction.update(change)
            self.assertIsNone(self.certify())

    def test_constant_ring_yield_cannot_cover_contact_unknown_or_unassessed_bodies(self):
        self.planner.junction = dict(yield_only=True, constant_ring_speed=True,
            ready=True, held=True, empty_actual_body=True, line_gap_m=.9)
        for vehicle in (car(5., 0., 3., 0., 7), car(15., 0., 0., 0., 7)):
            self.assertIsNone(self.certify(cars=[vehicle],
                batches=[('vehicles', [vehicle], 99.9, .1)]))
        self.planner.junction['unknown_ids'] = {7}
        self.assertIsNone(self.certify())

    def test_binding_rejects_pose_speed_stamp_path_expiry_and_clock_rewind(self):
        proof = self.certify()
        self.assertTrue(proof_matches(proof, self.ego, self.path, now=10.))
        for key in ('x', 'y', 'heading_deg', 'speed_kmh', 'stamp'):
            self.assertFalse(proof_matches(proof, dict(self.ego, **{key: self.ego[key]+.001}), self.path, now=10.))
        changed = self.path.copy()
        changed[50, 1] += .001
        self.assertFalse(proof_matches(proof, self.ego, changed, now=10.))
        self.assertFalse(proof_matches(proof, self.ego, self.path, now=proof.expires_at+1e-8))
        self.assertFalse(proof_matches(proof, self.ego, self.path, now=9.99))

    def test_same_call_evaluation_is_available_after_wait_without_mutating_planner(self):
        crossing = np.column_stack((np.full(161, 10.), np.arange(40., -40.01, -.5)))
        planner = self.mapped_planner(crossing)
        vehicle = car(10., 5., 0., -3., 9)
        evaluation = {'old': 'must not survive'}
        proof = self.certify(planner=planner, cars=[vehicle],
            batches=[('vehicles', [vehicle], 99.9, .1)], evaluation_out=evaluation)
        self.assertIsNone(proof)
        self.assertEqual(evaluation['status']['ring']['action'], 'WAIT')
        self.assertEqual(evaluation['decision']['state'], 'CROSSING_CAUTION')
        self.assertNotIn('old', evaluation)
        self.assertEqual(planner.status, {})
        self.assertIsNone(self.certify(evaluation_out=evaluation, vehicle_age_s=.36))
        self.assertEqual(evaluation, {})

    def test_current_launch_state_is_used_by_both_gp_and_coverage_reassessment(self):
        crossing = np.column_stack((np.full(161, 10.), np.arange(40., -40.01, -.5)))
        planner = self.mapped_planner(crossing)
        vehicle = car(10., 22., 0., -3., 9)
        ego = dict(self.ego, speed_kmh=7.2, brake=0., accel=.3)
        evaluation = {}
        self.assertIsNone(self.certify(planner=planner, ego=ego, cars=[vehicle],
            batches=[('vehicles', [vehicle], 99.9, .1)], evaluation_out=evaluation))
        pending = evaluation['status']['ring']
        self.assertEqual(pending['action'], 'WAIT')
        ego['_junction_launch'] = dict(v_mps=2., dead_s=0., stamp=ego['stamp'])
        proof = self.certify(planner=planner, ego=ego, cars=[vehicle],
            batches=[('vehicles', [vehicle], 99.9, .1)], evaluation_out=evaluation)
        self.assertIsNotNone(proof)
        launched = evaluation['status']['ring']
        self.assertEqual(launched['action'], 'GO')
        self.assertGreaterEqual(launched['checks'][0]['margin_s'], .55)
        self.assertLess(launched['checks'][0]['rear_clear_s'], pending['checks'][0]['rear_clear_s'])
        self.assertEqual(proof.required_rear_travel_m, launched['checks'][0]['required_rear_travel_m'])
        self.assertEqual(launched['checks'][0]['earliest_s'], pending['checks'][0]['earliest_s'])
        self.assertTrue(proof_matches(proof, ego, self.path, now=10.))

    def test_invalid_or_old_launch_state_cannot_gain_publication_permission(self):
        crossing = np.column_stack((np.full(161, 10.), np.arange(40., -40.01, -.5)))
        planner = self.mapped_planner(crossing)
        vehicle = car(10., 22., 0., -3., 9)
        for state in (dict(v_mps=2., dead_s=0., stamp=99.999999),
                      dict(v_mps=2.01, dead_s=0., stamp=100.),
                      dict(v_mps=2., dead_s=.501, stamp=100.),
                      dict(v_mps=2., dead_s=math.nan, stamp=100.),
                      dict(v_mps=0., dead_s=.5, stamp=100.)):
            with self.subTest(state=state):
                ego = dict(self.ego, speed_kmh=7.2, brake=0., accel=.3, _junction_launch=state)
                evaluation = {}
                self.assertIsNone(self.certify(planner=planner, ego=ego, cars=[vehicle],
                    batches=[('vehicles', [vehicle], 99.9, .1)], evaluation_out=evaluation))
                self.assertEqual(evaluation['status']['ring']['action'], 'WAIT')

    def test_pending_actual_brake_rejects_a_forged_current_launch_state(self):
        crossing = np.column_stack((np.full(161, 10.), np.arange(40., -40.01, -.5)))
        planner = self.mapped_planner(crossing)
        vehicle = car(10., 22., 0., -3., 9)
        ego = dict(self.ego, speed_kmh=7.2, brake=1., accel=.3,
            _junction_launch=dict(v_mps=2., dead_s=0., stamp=100.))
        evaluation = {}
        self.assertIsNone(self.certify(planner=planner, ego=ego, cars=[vehicle],
            batches=[('vehicles', [vehicle], 99.9, .1)], evaluation_out=evaluation))
        self.assertEqual(evaluation['status']['ring']['action'], 'WAIT')

    def test_ages_rtf_and_malformed_path_fail_closed(self):
        for arguments in (dict(vehicle_age_s=.351), dict(lidar_age_s=.351), dict(vehicle_age_s=-.1),
                          dict(rtf=0.), dict(rtf=1.01), dict(rtf=math.nan), dict(requested_mps=0.)):
            self.assertIsNone(self.certify(**arguments))
        self.assertIsNone(self.certify(ego=dict(self.ego, age_s=.201)))
        self.assertIsNone(self.certify(ego=dict(self.ego, received=9.7)))
        self.assertIsNone(self.certify(ego=dict(self.ego, received=10.01)))
        self.assertIsNone(self.certify(path=self.path[:3]))
        path = self.path.copy()
        path[8, 1] = math.nan
        self.assertIsNone(self.certify(path=path))

    def test_source_batch_must_own_a_fresh_measured_observation(self):
        self.assertIsNone(self.certify(batches=[]))
        self.assertIsNone(self.certify(batches=[('vehicles', [self.vehicle], 99.9, .36)]))
        self.assertIsNone(self.certify(batches=[('vehicles', [self.vehicle], 0., .1)]))
        self.assertIsNone(self.certify(batches=[('vehicles', [self.vehicle], 100.2, .1)]))
        self.assertIsNone(self.certify(batches=[('vehicles', [self.vehicle], 99.6, .1)]))
        bad = list(self.vehicle)
        bad[14] = False
        self.assertIsNone(self.certify(batches=[('vehicles', [tuple(bad)], 99.9, .1)]))

    def test_unmapped_straight_fallback_and_wrong_direction_cannot_own_future(self):
        planner = GapPlanner()
        planner.junction = dict(following_only=True, ready=True)
        self.assertTrue(planner._lane_paths(self.vehicle, 0., np.array([3., 0.])))
        self.assertIsNone(self.certify(planner=planner))
        backward = car(15., 0., -3., 0., 7)
        self.assertIsNone(self.certify(cars=[backward], batches=[('vehicles', [backward], 99.9, .1)]))

    def test_unknown_predicted_excluded_and_nonfollowing_cannot_own(self):
        self.assertIsNone(self.certify(excluded_uids={7}))
        for field in ('unknown_ids', 'ghost_ids', 'inherited_ids'):
            self.planner.junction[field] = {7}
            self.assertIsNone(self.certify())
            self.planner.junction.pop(field)
        predicted = tuple(self.vehicle)+(.01,)
        self.assertIsNone(self.certify(cars=[predicted]))
        unknown = list(self.vehicle)
        unknown[14] = False
        self.assertIsNone(self.certify(cars=[tuple(unknown)]))
        self.planner.junction['following_only'] = False
        self.assertIsNone(self.certify())

    def test_large_measured_vehicle_retains_generic_forecast_and_body_guards(self):
        for length, width in ((6.00001, 1.9), (4.6, 2.40001)):
            oversized = list(self.vehicle)
            oversized[6:8] = [length, width]
            oversized = tuple(oversized)
            self.assertIsNone(self.certify(cars=[oversized],
                batches=[('vehicles', [oversized], 99.9, .1)]))
            self.assertIsNone(self.certify(cars=[self.vehicle],
                batches=[('vehicles', [oversized], 99.9, .1)]))
        bounded = list(self.vehicle)
        bounded[6:8] = [6., 2.4]
        bounded = tuple(bounded)
        self.assertIsNotNone(self.certify(cars=[bounded],
            batches=[('vehicles', [bounded], 99.9, .1)]))

    def test_mapped_crossing_beyond_entry_scope_keeps_generic_future_collision_guard(self):
        from path_safety import PathSafety
        from ego_body import EGO_CENTRE_M

        lane = np.column_stack((np.full(161, 37.), np.arange(40., -40.01, -.5)))
        planner = self.mapped_planner(lane)
        vehicle = car(37., 7.6, 0., -3., 8)
        ego = dict(self.ego, speed_kmh=60.)
        batches = [('vehicles', [vehicle], 99.9, .1)]
        decision = planner.evaluate(to_sim_objects([vehicle], .8), ego, self.path, 30./3.6, 10.)
        self.assertEqual(planner.status['state'], 'CRUISE')
        self.assertGreater(decision['v_mps'], 0.)
        generic = PathSafety(body_shift_m=EGO_CENTRE_M).check(self.path, ego, batches, 10.,
            requested_mps=30./3.6, motion_time_scale=.8, dynamic_only=True)
        self.assertTrue(generic.emergency)
        self.assertIsNone(self.certify(planner=planner, ego=ego, cars=[vehicle], batches=batches,
                                      requested_mps=30./3.6))

    def test_one_assessed_leader_does_not_cover_an_unclassified_mapped_vehicle(self):
        crossing_lane = np.column_stack((np.full(161, 37.), np.arange(40., -40.01, -.5)))
        planner = GapPlanner([
            dict(points=self.path.tolist(), from_node_idx='a', to_node_idx='b'),
            dict(points=crossing_lane.tolist(), from_node_idx='c', to_node_idx='d')])
        planner.junction = dict(following_only=True, ready=True, empty_actual_body=True)
        unclassified = car(37., 7.6, 0., -3., 8)
        vehicles = [self.vehicle, unclassified]
        proof = self.certify(planner=planner, cars=vehicles, batches=[('vehicles', vehicles, 99.9, .1)])
        self.assertIsNotNone(proof)
        self.assertIn(('lidar_car', 7), proof.covered_keys)
        self.assertNotIn(('lidar_car', 8), proof.covered_keys)

    def test_classified_beside_or_behind_vehicle_is_not_a_timed_entry_participant(self):
        behind = car(-6., 0., 3., 0., 8)
        world_map = np.column_stack((np.arange(-20., 80.01, .5), np.zeros(201)))
        planner = self.mapped_planner(world_map)
        vehicles = [self.vehicle, behind]
        proof = self.certify(planner=planner, cars=vehicles, batches=[('vehicles', vehicles, 99.9, .1)])
        self.assertIsNotNone(proof)
        self.assertNotIn(('lidar_car', 8), proof.covered_keys)

    def test_positive_contact_approach_is_not_a_go_certificate(self):
        lane = np.column_stack((np.full(161, 10.), np.arange(40., -40.01, -.5)))
        crossing = car(10., 13., 0., -3., 7)
        planner = self.mapped_planner(lane)
        self.assertIsNone(self.certify(planner=planner, cars=[crossing],
            batches=[('vehicles', [crossing], 99.9, .1)]))

    def test_independent_launch_profile_keeps_the_current_emitted_cap(self):
        lane = np.column_stack((np.full(161, 10.), np.arange(40., -40.01, -.5)))
        crossing = car(10., 35., 0., -3., 7)
        planner = self.mapped_planner(lane)
        arguments = dict(planner=planner, cars=[crossing],
            batches=[('vehicles', [crossing], 99.9, .1)], requested_mps=1.2826)
        self.assertIsNone(self.certify(**arguments))
        proof = self.certify(**arguments, profile_ceiling_mps=24./3.6)
        self.assertIsNotNone(proof)
        self.assertLessEqual(proof.cap_mps, 1.2826)
        self.assertEqual(proof.covered_keys, frozenset({('lidar_car', 7)}))

    def incoming_fixture(self):
        lanes = [np.column_stack((np.full(201, x), np.arange(50., -50.01, -.5)))
                 for x in (10., 20.)]
        planner = GapPlanner([dict(points=lane.tolist(), from_node_idx=str(index),
                                   to_node_idx=str(index+2)) for index, lane in enumerate(lanes)])
        planner.junction = dict(following_only=True, ready=True, empty_actual_body=True)
        vehicles = [car(10., 35., 0., -3., 7), car(20., 44., 0., -3., 8)]
        return planner, vehicles, [('vehicles', vehicles, 99.9, .1)]

    def test_actual_incoming_map_proof_carries_the_longest_unrounded_rear_clearance(self):
        planner, vehicles, batches = self.incoming_fixture()
        decision = planner.evaluate(to_sim_objects(vehicles, .8), self.ego, self.path, 24./3.6, 10.)
        self.assertGreater(decision['v_mps'], 0.)
        checks = planner.status['ring']['checks']
        self.assertEqual({check['object_id'] for check in checks}, {7, 8})
        required = max(check['required_rear_travel_m'] for check in checks)
        proof = self.certify(planner=planner, cars=vehicles, batches=batches,
                             requested_mps=1.2826, profile_ceiling_mps=24./3.6)
        self.assertIsNotNone(proof)
        self.assertEqual(proof.required_rear_travel_m, required)
        self.assertGreater(required, 23.)
        self.assertNotEqual(required, round(required, 2))
        self.assertLessEqual(proof.cap_mps, 1.2826)

    def test_successful_certificate_runs_the_expensive_map_assessment_once(self):
        import ring_entry
        planner, vehicles, batches = self.incoming_fixture()
        with patch('ring_entry.classify', wraps=ring_entry.classify) as classify, \
                patch('ring_entry.entry_decision', wraps=ring_entry.entry_decision) as decide:
            proof = self.certify(planner=planner, cars=vehicles, batches=batches)
        self.assertIsNotNone(proof)
        self.assertEqual(classify.call_count, 1)
        self.assertEqual(decide.call_count, 1)

    def test_copied_old_assessment_cannot_authorize_an_overridden_evaluator(self):
        planner, vehicles, batches = self.incoming_fixture()
        decision = planner.evaluate(to_sim_objects(vehicles, .8), self.ego, self.path, 24./3.6, 10.)
        previous = planner._junction_assessment
        status = copy.deepcopy(planner.status)
        self.assertIsNotNone(previous)
        def fake(instance, *args, **kwargs):
            instance.status = copy.deepcopy(status)
            return copy.deepcopy(decision)
        with patch.object(GapPlanner, 'evaluate', fake):
            self.assertIsNone(self.certify(planner=planner, cars=vehicles, batches=batches))
        self.assertIs(planner._junction_assessment, previous)

    def test_restoring_an_old_assessment_after_a_real_evaluation_fails_its_epoch(self):
        planner, vehicles, batches = self.incoming_fixture()
        planner.evaluate(to_sim_objects(vehicles, .8), self.ego, self.path, 24./3.6, 10.)
        previous = planner._junction_assessment
        evaluate = GapPlanner.evaluate
        def stale(instance, *args, **kwargs):
            decision = evaluate(instance, *args, **kwargs)
            instance._junction_assessment = previous
            return decision
        with patch.object(GapPlanner, 'evaluate', stale):
            self.assertIsNone(self.certify(planner=planner, cars=vehicles, batches=batches))

    def test_private_assessment_is_immutable_and_is_cleared_on_an_invalid_next_frame(self):
        planner, vehicles, _ = self.incoming_fixture()
        planner.evaluate(to_sim_objects(vehicles, .8), self.ego, self.path, 24./3.6, 10.)
        assessed = planner._junction_assessment
        self.assertIsInstance(assessed.moving_ids, frozenset)
        self.assertIsInstance(assessed.incoming, tuple)
        with self.assertRaises(FrozenInstanceError):
            assessed.incoming = ()
        planner.evaluate(to_sim_objects(vehicles, .8), self.ego, self.path[:2], 24./3.6, 10.1)
        self.assertIsNone(planner._junction_assessment)

    def test_a_followed_leader_without_incoming_checks_needs_no_rear_reservation(self):
        proof = self.certify()
        self.assertIsNotNone(proof)
        self.assertEqual(proof.required_rear_travel_m, 0.)

    def certify_with_changed_incoming_status(self, change):
        planner, vehicles, batches = self.incoming_fixture()
        evaluate = GapPlanner.evaluate

        def changed(instance, *args, **kwargs):
            result = evaluate(instance, *args, **kwargs)
            change(instance.status)
            return result

        with patch.object(GapPlanner, 'evaluate', changed):
            return self.certify(planner=planner, cars=vehicles, batches=batches)

    def test_missing_incoming_distance_or_erased_checks_fail_closed(self):
        changes = (lambda status: status.pop('ring'),
                   lambda status: status['ring'].pop('checks'),
                   lambda status: status['ring'].update(checks=[]),
                   lambda status: status['ring'].update(checks=status['ring']['checks'][:1]),
                   lambda status: status['ring']['checks'][0].pop('required_rear_travel_m'))
        for change in changes:
            with self.subTest(change=change):
                self.assertIsNone(self.certify_with_changed_incoming_status(change))

    def test_even_a_single_float_step_distance_edit_cannot_change_private_evidence(self):
        def change(status):
            check = status['ring']['checks'][0]
            check['required_rear_travel_m'] = float(np.nextafter(check['required_rear_travel_m'], -np.inf))
        self.assertIsNone(self.certify_with_changed_incoming_status(change))

    def test_one_invalid_or_underreported_incoming_distance_cannot_be_ignored(self):
        for value in (math.nan, math.inf, -.1, None, False, 'invalid', 0.):
            with self.subTest(value=value):
                def change(status):
                    status['ring']['checks'][0]['required_rear_travel_m'] = value
                self.assertIsNone(self.certify_with_changed_incoming_status(change))

    def test_bound_proof_rejects_invalid_rear_travel(self):
        proof = self.certify()
        for value in (math.nan, math.inf, -.1):
            self.assertFalse(proof_matches(replace(proof, required_rear_travel_m=value),
                                          self.ego, self.path, now=10.))

    def test_invalid_independent_profile_never_raises_a_zero_or_invalid_request(self):
        for profile in (0., -.1, math.nan, math.inf):
            self.assertIsNone(self.certify(profile_ceiling_mps=profile))
        self.assertIsNone(self.certify(requested_mps=0., profile_ceiling_mps=24./3.6))
        self.assertIsNone(self.certify(vehicle_age_s=.351, profile_ceiling_mps=24./3.6))
        contact = car(5., 0., 3., 0., 7)
        self.assertIsNone(self.certify(cars=[contact], batches=[('vehicles', [contact], 99.9, .1)],
            requested_mps=1.2826, profile_ceiling_mps=24./3.6))

    def test_actual_front_and_source_body_contact_refuse_certificate(self):
        contact = car(5., 0., 3., 0., 7)
        self.assertIsNone(self.certify(cars=[contact], batches=[('vehicles', [contact], 99.9, .1)]))
        # The aligned body is clear, but the source body overlaps the real
        # front3.845m. The symmetric legacy body would miss this contact.
        aligned = car(6.5, 0., 3., 0., 7)
        source = car(5.5, 0., 3., 0., 7)
        self.assertIsNone(self.certify(cars=[aligned], batches=[('vehicles', [source], 99.9, .1)]))

    def alias(self, uid=12, **fields):
        row = list(car(fields.get('x', 15.4), fields.get('y', 0.), fields.get('vx', 2.8), fields.get('vy', 0.), uid))
        row[3], row[7] = 'fusion_npc', fields.get('width', 3.2)
        return tuple(row)

    def parked_alias(self, uid=12, **fields):
        row = list(car(fields.get('x', 15.25), fields.get('y', 0.), 0., 0., uid,
                       heading=fields.get('heading', 30.)))
        row[3:5] = ['fusion_parked_vehicle', 'obstacle']
        row[7] = fields.get('width', 3.2)
        row.append(0.)
        return tuple(row)

    def test_exact_parked_camera_alias_keeps_its_original_body_and_shorter_deadline(self):
        self.vehicle = tuple(self.vehicle)+(0.,)
        camera = self.parked_alias()
        original = tuple(camera)
        proof = self.certify(batches=[('vehicles', [self.vehicle], 99.9, .1),
                                     ('semantic', [camera], 99.7, .3)])
        self.assertIn(('fusion_parked_vehicle', 12), proof.covered_keys)
        self.assertGreater(proof.current_clear_m, .1)
        self.assertAlmostEqual(proof.expires_at, 10.05)
        self.assertEqual(camera, original)
        self.assertEqual(camera[4], 'obstacle')
        self.assertEqual(camera[9:11], (0., 0.))

    def test_generic_static_people_moving_or_predicted_labels_never_gain_parked_ownership(self):
        self.vehicle = tuple(self.vehicle)+(0.,)
        camera = self.parked_alias()
        for index, value in ((3, 'fusion_obstacle'), (4, 'pedestrian'),
                             (5, .01), (9, .01), (15, .01), (15, math.nan)):
            row = list(camera)
            row[index] = value
            proof = self.certify(batches=[('vehicles', [self.vehicle], 99.9, .1),
                                         ('semantic', [tuple(row)], 99.9, .1)])
            self.assertIsNotNone(proof)
            self.assertNotIn(('fusion_parked_vehicle', 12), proof.covered_keys)
        stale = self.certify(batches=[('vehicles', [self.vehicle], 99.9, .1),
                                     ('semantic', [camera], 99.649, .351)])
        self.assertNotIn(('fusion_parked_vehicle', 12), stale.covered_keys)

    def test_stationary_camera_beside_the_car_or_outside_shape_bounds_remains_independent(self):
        self.vehicle = tuple(self.vehicle)+(0.,)
        for camera, age in ((self.parked_alias(x=15.9), .1),
                            (self.parked_alias(x=16.21), .3),
                            (self.parked_alias(width=3.81), .1),
                            (self.parked_alias(heading=40.01), .1)):
            proof = self.certify(batches=[('vehicles', [self.vehicle], 100.-age, age),
                                         ('semantic', [camera], 99.9, .1)])
            self.assertIsNotNone(proof)
            self.assertNotIn(('fusion_parked_vehicle', 12), proof.covered_keys)

    def test_a_nearby_unknown_pp_or_two_camera_candidates_prevents_unique_ownership(self):
        self.vehicle = tuple(self.vehicle)+(0.,)
        camera = self.parked_alias()
        unknown = list(car(15.8, 0., 0., 0., 8, valid=False))
        unknown.append(0.)
        proof = self.certify(batches=[('vehicles', [self.vehicle, tuple(unknown)], 99.9, .1),
                                     ('semantic', [camera], 99.9, .1)])
        self.assertNotIn(('fusion_parked_vehicle', 12), proof.covered_keys)
        other = self.parked_alias(uid=13, x=15.3)
        proof = self.certify(batches=[('vehicles', [self.vehicle], 99.9, .1),
                                     ('semantic', [camera, other], 99.9, .1)])
        self.assertEqual(proof.covered_keys, frozenset({('lidar_car', 7)}))

    def test_original_inflated_parked_camera_contact_still_vetoes_a_clear_pp_body(self):
        pp = tuple(car(7., 0., 3., 0., 7))+(0.,)
        camera = self.parked_alias(x=6.9, width=3.8, heading=40.)
        self.assertIsNotNone(self.certify(cars=[pp], batches=[('vehicles', [pp], 99.95, .05)]))
        self.assertIsNone(self.certify(cars=[pp], batches=[('vehicles', [pp], 99.95, .05),
            ('semantic', [camera], 99.95, .05)]))

    def test_recorded_native39_parked_camera_and_pp_pair_is_unique_and_currently_clear(self):
        fixture = json.loads((Path(__file__).parent/'test_data/junction_native39_parked_alias.json').read_text())
        pp = fixture['pp']
        row = list(car(*pp['position'], *pp['velocity'], pp['uid'], heading=pp['heading_deg']))
        row[6:8] = pp['size'][:2]
        row.append(0.)
        camera = fixture['camera']
        alias = list(self.parked_alias(uid=camera['uid'], x=camera['position'][0],
                                      y=camera['position'][1], heading=camera['heading_deg'],
                                      width=camera['size'][1]))
        alias[6] = camera['size'][0]
        world = np.asarray(fixture['world_path'])
        links = json.loads((Path(__file__).parent/'link_set.json').read_text())
        planner = GapPlanner(links)
        planner.junction = dict(following_only=True, ready=True, empty_actual_body=True)
        now = fixture['capture_wall']
        pose_age = now-fixture['ego']['stamp']
        ego = dict(fixture['ego'], age_s=pose_age, received=10.-pose_age)
        proof = self.certify(planner=planner, ego=ego, path=world, cars=[tuple(row)],
            batches=[('vehicles', [tuple(row)], fixture['pp_batch_stamp'], now-fixture['pp_batch_stamp']),
                     ('semantic', [tuple(alias)], fixture['camera_stamp'], now-fixture['camera_stamp'])],
            rtf=fixture['rtf'])
        self.assertIsNotNone(proof)
        self.assertEqual(proof.covered_keys,
                         frozenset({('lidar_car', 500017), ('fusion_parked_vehicle', 23)}))
        self.assertAlmostEqual(proof.current_clear_m, 1.3039856076, places=8)
        # The old source was already stale when native39 published later.
        publication_delay = fixture['actual_publication_wall']-now
        self.assertFalse(proof_matches(proof, ego, world, now=10.+publication_delay))

    def test_one_to_one_coherent_camera_alias_keeps_its_complete_current_box(self):
        alias = self.alias()
        batches = [('vehicles', [self.vehicle], 99.9, .1), ('semantic', [alias], 99.85, .15)]
        proof = self.certify(batches=batches)
        self.assertEqual(proof.covered_keys, frozenset({('lidar_car', 7), ('fusion_npc', 12)}))
        self.assertGreater(proof.current_clear_m, .1)
        duplicate = self.alias(uid=13, x=15.8)
        proof = self.certify(batches=[batches[0], ('semantic', [alias, duplicate], 99.85, .15)])
        self.assertEqual(len(proof.covered_keys), 2)
        self.assertNotIn(('fusion_npc', 13), proof.covered_keys)

    def test_unmatched_stale_opposite_and_oversize_aliases_keep_generic_guards(self):
        for alias, age in ((self.alias(x=25.), .1), (self.alias(), .351),
                           (self.alias(vx=-3.), .1), (self.alias(width=3.81), .1)):
            proof = self.certify(batches=[('vehicles', [self.vehicle], 99.9, .1),
                                         ('semantic', [alias], 100.-age, age)])
            self.assertEqual(proof.covered_keys, frozenset({('lidar_car', 7)}))

    def test_alias_current_contact_is_not_discarded_even_after_matching(self):
        # Map ownership of the measured car cannot erase an associated camera
        # box which touches the actual ego right now.
        measured = car(7., 0., 3., 0., 7)
        alias = self.alias(x=5.5, width=1.9)
        self.assertIsNone(self.certify(cars=[measured], batches=[('vehicles', [measured], 99.9, .1),
            ('semantic', [alias], 99.9, .1)]))

    def test_npc_velocity_converts_once_and_pose_age_owns_expiry(self):
        with patch('junction_vehicle_motion.to_sim_objects', wraps=to_sim_objects) as convert:
            proof = self.certify()
        self.assertEqual(convert.call_count, 1)
        self.assertAlmostEqual(proof.expires_at, 10.18)
        self.assertIsNone(self.certify(rtf=.2))  # Outside production converter's valid RTF range.

    def test_recorded_native32_curve_static_tube_does_not_replace_actual_t0_body(self):
        # Recorded plan3520 WORLD prefix and source pose, rounded below1um.
        # The full map retains plausible branches; a single straight fallback
        # would not be an equivalent ring timing proof.
        world = np.array([
            (-98.771276, 319.723568), (-98.628015, 320.202604), (-98.469957, 320.676961),
            (-98.296614, 321.145947), (-98.107098, 321.608631), (-97.900437, 322.063912),
            (-97.675906, 322.510646), (-97.433245, 322.947793), (-97.172684, 323.374510),
            (-96.894902, 323.790221), (-96.600863, 324.194596), (-96.291692, 324.587522),
            (-95.968608, 324.969093), (-95.632931, 325.339638), (-95.286121, 325.699788),
            (-94.929787, 326.050523), (-94.565623, 326.393127), (-94.195305, 326.729074),
            (-93.820398, 327.059894), (-93.442286, 327.387049), (-93.062160, 327.711863),
            (-92.681061, 328.035535), (-92.299952, 328.359195), (-91.919791, 328.683967),
            (-91.541575, 329.011001)])
        links = json.loads((Path(__file__).parent/'link_set.json').read_text())
        if isinstance(links, dict):
            links = links.get('links', links.get('data', []))
        planner = GapPlanner(links)
        planner.junction = dict(following_only=True, ready=True, empty_actual_body=True)
        ego = dict(self.ego, x=-98.635511, y=319.4533, heading_deg=76.96074)
        row = list(car(-97.858072, 325.639954, 1.370972, .314921, 500008, heading=374.124065))
        row[6:8] = [4.918751, 2.100781]
        row = tuple(row)
        yaw = math.radians(ego['heading_deg'])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        static_clear = semantic_entry_clearance((world-[ego['x'], ego['y']])@rotation, ego, [row], 60.)
        self.assertLessEqual(static_clear, .05+1e-8)
        proof = self.certify(planner=planner, ego=ego, path=world, cars=[row],
            batches=[('vehicles', [row], 99.9, .1)], rtf=.716)
        # GP ignores this partially beside on-path car. It has no assessed
        # timing witness, so a raw GO alone cannot own its future/static tube.
        self.assertIsNone(proof)

        # A passing recorded PP body (native32 t1166.103, uid500007) is actually
        # assessed as an ahead car on the nearest physical lane seed. Isolate
        # this body here; the other native
        # traffic is deliberately absent, so this is not a full-scene GO claim.
        ahead = list(car(-95.944479579, 326.548303307, 3.255026172, 1.523709088,
                         500007, heading=380.7666365))
        ahead[6:8] = [4.932184219, 2.095575333]
        ahead = tuple(ahead)
        age = .1
        current = list(ahead)
        current[0] += current[9]*age
        current[1] += current[10]*age
        current = tuple(current)
        self.assertLess(semantic_entry_clearance((world-[ego['x'], ego['y']])@rotation, ego, [ahead], 60.), 1.)
        proof = self.certify(planner=planner, ego=ego, path=world, cars=[current],
            batches=[('vehicles', [ahead], 99.9, age)], rtf=.716, vehicle_age_s=age)
        self.assertIsNotNone(proof)
        self.assertIn(('lidar_car', 500007), proof.covered_keys)
        self.assertGreater(proof.current_clear_m, .8)
        self.assertGreater(proof.cap_mps, 0.)


class PositionMotionConfirmationTests(unittest.TestCase):
    def setUp(self):
        self.tracker = RingMotionTracker()

    def source(self, stamp, x, y=0., *, vx=3., vy=0., valid=True, uid=7,
               prediction_age=0., length=4.6, width=1.9, heading=10.):
        row = list(car(x, y, vx, vy, uid, valid=valid, heading=heading))
        row[6:8] = [length, width]
        row.append(prediction_age)
        return float(stamp), tuple(row)

    def observe(self, observation, tracker=None):
        stamp, row = observation
        tracker = self.tracker if tracker is None else tracker
        stamps = {row[11]: stamp}
        tracker.update([row], stamps)
        return confirm_position_motion([row], tracker, stamps)

    def moving_dropout(self, *, offset=15., uid=7, tracker=None):
        self.observe(self.source(99.5, offset, uid=uid), tracker)
        self.observe(self.source(99.7, offset+.6, uid=uid), tracker)
        final = self.source(99.9, offset+1.2, uid=uid, valid=False)
        return final, self.observe(final, tracker)

    def test_short_measured_dropout_preserves_pose_shape_yaw_and_input(self):
        final, (rows, promoted) = self.moving_dropout()
        original = final[1]
        self.assertEqual(promoted, frozenset({7}))
        row = rows[0]
        self.assertTrue(row[14])
        self.assertAlmostEqual(row[9], 3.)
        self.assertAlmostEqual(row[10], 0.)
        self.assertAlmostEqual(row[5], 3.)
        for index in (0, 1, 2, 3, 4, 6, 7, 8, 11, 12, 13, 15):
            self.assertEqual(row[index], original[index])
        self.assertFalse(original[14])
        self.assertEqual(original[9:11], (0., 0.))
        self.assertGreaterEqual(self.tracker.ratio[7], FALLBACK_RATIO)
        self.assertAlmostEqual(self.tracker.fitted[7], 3.)
        self.assertTrue(math.isnan(self.tracker.history[7][-1][3]))

    def test_new_unmeasured_track_never_creates_its_own_trusted_motion(self):
        for stamp, x in ((99.5, 15.), (99.7, 15.6), (99.9, 16.2)):
            rows, promoted = self.observe(self.source(stamp, x, valid=False))
            self.assertFalse(rows[0][14])
            self.assertFalse(promoted)
        fresh = RingMotionTracker()
        rows, promoted = confirm_position_motion([self.source(99.9, 16.2, valid=False)[1]], fresh, {7: 99.9})
        self.assertFalse(promoted)
        self.assertFalse(rows[0][14])

    def test_repeated_source_time_does_not_add_measurements_or_renew_trust(self):
        self.observe(self.source(99.5, 15.))
        final = self.source(99.9, 16.2, valid=False)
        for _ in range(4):
            rows, promoted = self.observe(final)
            self.assertFalse(promoted)
        self.assertEqual(len(self.tracker.history[7]), 2)
        self.assertEqual(self.tracker._junction_position_trusted[7][0], 99.5)
        self.tracker = RingMotionTracker()
        final, (rows, promoted) = self.moving_dropout()
        original_history = list(self.tracker.history[7])
        repeated, again = self.observe(final)
        self.assertEqual(again, promoted)
        self.assertEqual(repeated, rows)
        self.assertEqual(self.tracker.history[7], original_history)
        self.assertEqual(self.tracker._junction_position_trusted[7][0], 99.7)

    def test_trusted_history_reset_or_source_conflict_cannot_bridge(self):
        self.observe(self.source(99.5, 15.))
        self.tracker.reset()
        self.observe(self.source(99.6, 15.3, valid=False))
        self.observe(self.source(99.7, 15.6, valid=False))
        self.assertFalse(self.observe(self.source(99.9, 16.2, valid=False))[1])
        self.tracker = RingMotionTracker()
        self.moving_dropout()
        wrong = self.source(99.9, 16.4, valid=False)[1]
        self.assertFalse(confirm_position_motion([wrong], self.tracker, {7: 99.9})[1])

    def test_standing_jitter_and_abrupt_deceleration_are_not_motion(self):
        for positions, measured_speed in (([15., 15., 15.], 0.),
                                          ([15., 15.09, 14.95], 3.),
                                          ([15., 15.6, 15.66], 3.),
                                          ([15., 15.6, 15.1], 3.)):
            tracker = RingMotionTracker()
            self.observe(self.source(99.5, positions[0], vx=measured_speed), tracker)
            self.observe(self.source(99.7, positions[1], valid=False), tracker)
            self.assertFalse(self.observe(self.source(99.9, positions[2], valid=False), tracker)[1])

    def test_newer_measured_standstill_revokes_previous_trusted_movement(self):
        self.observe(self.source(99.5, 15.))
        self.observe(self.source(99.7, 15.6, vx=0.))
        self.assertNotIn(7, self.tracker._junction_position_trusted)
        self.assertFalse(self.observe(self.source(99.9, 16.2, valid=False))[1])

    def test_fitted_true_velocity_cannot_overwrite_original_source_velocity(self):
        stamp, raw = self.source(99.5, 15.)
        self.tracker.update([raw], {7: stamp})
        fitted = list(raw)
        fitted[5], fitted[9] = 5., 5.
        confirm_position_motion([tuple(fitted)], self.tracker, {7: stamp})
        self.assertNotIn(7, self.tracker._junction_position_trusted)

    def test_ghost_prediction_inherited_vector_and_large_shape_are_rejected(self):
        for change in ('prediction', 'inherited', 'shape', 'large', 'nan', 'semantic'):
            tracker = RingMotionTracker()
            self.observe(self.source(99.5, 15.), tracker)
            self.observe(self.source(99.7, 15.6), tracker)
            stamp, raw = self.source(99.9, 16.2, valid=False)
            row = list(raw)
            if change == 'prediction':
                row[15] = .01
            elif change == 'inherited':
                row[5], row[9] = 3., 3.
            elif change == 'shape':
                row[7] *= 1.26
            elif change == 'large':
                row[6] = 6.1
            elif change == 'nan':
                row[0] = math.nan
            else:
                row[3] = 'fusion_npc'
            self.assertFalse(confirm_position_motion([tuple(row)], tracker, {7: stamp})[1], change)

    def test_inherited_true_cannot_replace_an_original_false_history_witness(self):
        raw = self.source(99.5, 15., valid=False)[1]
        self.tracker.update([raw], {7: 99.5})
        inherited = self.source(99.5, 15.)[1]
        confirm_position_motion([inherited], self.tracker, {7: 99.5})
        self.assertNotIn(7, self.tracker._junction_position_trusted)
        self.observe(self.source(99.7, 15.6, valid=False))
        self.assertFalse(self.observe(self.source(99.9, 16.2, valid=False))[1])

    def test_old_sparse_duplicate_and_out_of_order_sources_are_rejected(self):
        self.observe(self.source(99.1, 15.))
        self.observe(self.source(99.6, 16.5, valid=False))
        self.assertFalse(self.observe(self.source(99.9, 17.4, valid=False))[1])
        self.tracker = RingMotionTracker()
        final, _ = self.moving_dropout()
        self.assertFalse(confirm_position_motion([final[1]], self.tracker, {7: 99.9, 8: 100.251})[1])
        self.assertFalse(confirm_position_motion([final[1], final[1]], self.tracker, {7: 99.9})[1])
        self.assertFalse(confirm_position_motion([self.source(99.8, 15.9, valid=False)[1]], self.tracker, {7: 99.8})[1])
        tracker = RingMotionTracker()
        self.observe(self.source(99.0, 15.), tracker)
        for stamp, x in ((99.3, 15.9), (99.6, 16.8)):
            self.observe(self.source(stamp, x, valid=False), tracker)
        self.assertFalse(self.observe(self.source(99.9, 17.7, valid=False), tracker)[1])

    def test_direction_change_and_discontinuous_track_centre_are_rejected(self):
        for observations in (
                [self.source(99.5, 15.), self.source(99.7, 15., .6, valid=False), self.source(99.9, 15., 1.2, valid=False)],
                [self.source(99.5, 15.), self.source(99.7, 20.6, valid=False), self.source(99.9, 21.2, valid=False)]):
            tracker = RingMotionTracker()
            for observation in observations:
                result = self.observe(observation, tracker)
            self.assertFalse(result[1])

    def test_confirmation_does_not_relax_actual_contact_or_entry_coverage(self):
        base = JunctionVehicleMotionTests()
        base.setUp()
        _, (rows, _) = self.moving_dropout(offset=3.8)
        self.assertIsNone(base.certify(cars=rows, batches=[('vehicles', rows, 99.9, .1)]))
        self.tracker = RingMotionTracker()
        lane = np.column_stack((np.full(161, 37.), np.arange(40., -40.01, -.5)))
        planner = base.mapped_planner(lane)
        for stamp, y, valid in ((99.5, 8.8, True), (99.7, 8.2, True), (99.9, 7.6, False)):
            rows, promoted = self.observe(self.source(stamp, 37., y, vx=0., vy=-3., valid=valid, uid=8))
        self.assertEqual(promoted, frozenset({8}))
        self.assertIsNone(base.certify(planner=planner, ego=dict(base.ego, speed_kmh=60.), cars=rows,
            batches=[('vehicles', rows, 99.9, .1)], requested_mps=30./3.6))

    def test_recorded_native33_original_pp_dropout_is_position_confirmed(self):
        # Original /perception/lidar_vehicles source positions, not Object_topic
        # ground truth. GUI omitted .915 and .216; the live source saw both.
        observations = [
            (1791925266.6955404, -98.3220141097, 325.5368640053, 2.7886676622, .2668573615, True),
            (1791925266.9150674, -98.5638441988, 325.6836168181, 2.5274677892, .3300014119, True),
            (1791925267.0538962, -98.3300058973, 325.8085584918, 1.8961794331, .4383803849, True),
            (1791925267.2158050, -97.9095558662, 325.7287227953, 1.8049596778, .3841436364, True),
            (1791925267.3629985, -97.3907831105, 326.0194600813, 0., 0., False),
            (1791925267.5016317, -97.0307357240, 326.1195300128, 0., 0., False),
        ]
        for stamp, x, y, vx, vy, valid in observations:
            final = self.source(stamp, x, y, vx=vx, vy=vy, valid=valid, uid=500011,
                                length=4.877139568, width=2.072130203, heading=376.450370712)
            rows, promoted = self.observe(final)
            if not valid:
                self.assertEqual(promoted, frozenset({500011}))
                self.assertTrue(rows[0][14])
                self.assertGreater(rows[0][9], 2.9)
                self.assertGreater(rows[0][10], .6)
                self.assertEqual(rows[0][:2], final[1][:2])
                self.assertEqual(rows[0][8], final[1][8])
        self.assertGreaterEqual(self.tracker.ratio[500011], FALLBACK_RATIO)
        self.tracker = RingMotionTracker()
        gui_observations = [observations[i] for i in (0, 2, 4, 5)]
        for stamp, x, y, vx, vy, valid in gui_observations:
            rows, promoted = self.observe(self.source(stamp, x, y, vx=vx, vy=vy,
                valid=valid, uid=500011, length=4.88, width=2.07))
            if stamp == observations[4][0]:
                self.assertFalse(promoted)  # Its sparse first estimate is inconsistent.
        self.assertEqual(promoted, frozenset({500011}))


class SourceMotionRowsTests(unittest.TestCase):
    def setUp(self):
        self.tracker = RingMotionTracker()

    def observe(self, stamp, x, *, speed=3., valid=True, uid=7):
        row = tuple(car(x, 0., speed, 0., uid, valid=valid, heading=12.)) + (0.,)
        output, promoted = source_motion_rows([row], self.tracker, {uid: stamp})
        return row, output[0], promoted

    def moving_history(self):
        for index in range(5):
            self.observe(99.+.2*index, 15.+.6*index)

    def test_measured_zero_overrides_old_fit_and_preserves_current_body(self):
        self.moving_history()
        original, output, promoted = self.observe(100., 17.4, speed=0.)
        self.assertEqual(output, original)
        self.assertEqual(output[5], 0.)
        self.assertTrue(output[14])
        self.assertFalse(promoted)
        self.assertEqual(self.tracker.fitted[7], 0.)
        self.assertNotIn(7, self.tracker._junction_position_trusted)

    def test_measured_near_standstill_keeps_its_original_velocity(self):
        self.moving_history()
        original, output, promoted = self.observe(100., 17.49, speed=.49)
        self.assertEqual(output, original)
        self.assertFalse(promoted)
        self.assertAlmostEqual(output[9], .49)
        self.assertGreaterEqual(self.tracker.ratio[7], 1.3)

    def test_confirmed_invalid_velocity_bridge_remains_available(self):
        self.observe(99.5, 15.)
        self.observe(99.7, 15.6)
        original, output, promoted = self.observe(99.9, 16.2, speed=0., valid=False)
        self.assertEqual(promoted, frozenset({7}))
        self.assertTrue(output[14])
        self.assertAlmostEqual(output[9], 3.)
        for index in (0, 1, 6, 7, 8, 11, 15):
            self.assertEqual(output[index], original[index])
        before = list(self.tracker.history[7])
        _, repeated, repeated_promoted = self.observe(99.9, 16.2, speed=0., valid=False)
        self.assertEqual(repeated, output)
        self.assertEqual(repeated_promoted, promoted)
        self.assertEqual(len(self.tracker.history[7]), len(before))

    def test_measured_stop_revokes_the_next_invalid_velocity_bridge(self):
        self.observe(99.5, 15.)
        self.observe(99.7, 15.6, speed=0.)
        original, output, promoted = self.observe(99.9, 16.2, speed=0., valid=False)
        self.assertEqual(output, original)
        self.assertFalse(promoted)
        self.assertFalse(output[14])

    def test_tracker_reset_drops_private_trust_before_reusing_uid(self):
        self.observe(99.5, 15.)
        self.tracker.reset()
        original, output, promoted = self.observe(99.7, 15.6, speed=0., valid=False)
        self.assertEqual(output, original)
        self.assertFalse(promoted)
        self.assertNotIn(7, self.tracker._junction_position_trusted)

    def test_source_without_prediction_age_also_preserves_measured_stop(self):
        self.moving_history()
        original = tuple(car(17.4, 0., 0., 0., 7, valid=True))
        output, promoted = source_motion_rows([original], self.tracker, {7: 100.})
        self.assertEqual(output, (original,))
        self.assertFalse(promoted)

    def change_speed(self, old, current):
        x, old_tracker = 15., RingMotionTracker()
        final = None
        for index in range(12):
            stamp = 98.+.2*index
            speed = current if index >= 9 else old
            row = tuple(car(x, 0., speed, 0., 7, valid=True)) + (0.,)
            before = old_tracker.update([row], {7: stamp})[0]
            output, promoted = source_motion_rows([row], self.tracker, {7: stamp})
            final = row, before, output[0], promoted
            x += .2*speed
        return final

    def test_recent_measured_acceleration_is_not_pinned_to_the_two_second_fit(self):
        original, old, new, promoted = self.change_speed(2., 4.8)
        self.assertLess(old[5], 4.)
        self.assertAlmostEqual(new[5], 4.8)
        self.assertFalse(promoted)
        self.assertTrue(new[14])
        for index in (0, 1, 2, 3, 4, 6, 7, 8, 11, 12, 13, 14, 15):
            self.assertEqual(new[index], original[index])

    def test_recent_measured_deceleration_is_not_replaced_by_old_fast_motion(self):
        _, old, new, _ = self.change_speed(5.5, 2.4)
        self.assertGreater(old[5], 4.)
        self.assertAlmostEqual(new[5], 2.4)
        self.assertGreaterEqual(self.tracker.ratio[7], 1.1)

    def test_partial_box_centroid_shift_cannot_double_speed_against_both_other_estimates(self):
        for index in range(22):
            t = .1*index
            offset = max(0., t-1.8)*3.
            original, output, _ = self.observe(100.+t, 3.*t+offset, speed=3.)
        self.assertLess(output[5], 4.)
        self.assertLess(self.tracker.fitted[7], 4.)
        self.assertEqual(output[:2], original[:2])
        self.assertGreaterEqual(output[5]*self.tracker.ratio[7], 3.)

    def test_current_raw_speed_remains_inside_the_short_fit_speed_bounds(self):
        for stamp, x in ((99.5, 15.), (99.7, 15.6), (99.9, 16.2)):
            self.observe(stamp, x, speed=2.)
        speed, ratio = self.tracker.fitted[7], self.tracker.ratio[7]
        self.assertAlmostEqual(speed, 3.)
        self.assertLessEqual(speed/ratio, 2.+1e-9)
        self.assertGreaterEqual(speed*ratio, 2.-1e-9)

    def test_stale_duplicate_or_conflicting_sources_cannot_gain_short_motion(self):
        self.change_speed(2., 4.8)
        raw = tuple(car(21., 0., 4.8, 0., 7, valid=True)) + (0.,)
        # Timestamp rewinds do not append a new trusted source position.
        output, promoted = source_motion_rows([raw], self.tracker, {7: 99.8})
        self.assertFalse(promoted)
        self.assertNotEqual(output[0][5], 4.8)
        tracker = RingMotionTracker()
        for stamp, x in ((99.5, 15.), (99.7, 15.6)):
            row = tuple(car(x, 0., 2., 0., 7, valid=True)) + (0.,)
            source_motion_rows([row], tracker, {7: stamp})
        raw = tuple(car(16.2, 0., 2., 0., 7, valid=True)) + (0.,)
        output, _ = source_motion_rows([raw], tracker, {7: 99.9, 8: 100.251})
        self.assertNotAlmostEqual(output[0][5], 3.)
        output, _ = source_motion_rows([raw, raw], tracker, {7: 99.9})
        self.assertTrue(all(row[5] != 3. for row in output))

    def test_recorded_native36_accelerating_leader_and_jittering_ghost(self):
        fixture = json.loads((Path(__file__).parent/'test_data/junction_native36_source_motion.json').read_text())
        old = RingMotionTracker()
        final = {}
        for frame in fixture['frames']:
            rows = []
            for value in frame['cars']:
                row = list(car(*value['position'], *value['velocity'], value['uid'],
                               valid=value['velocity_valid'], heading=value['heading_deg']))
                row[6:8] = value['size'][:2]
                row.append(value.get('prediction_age_s', 0.))
                rows.append(tuple(row))
            stamps = {row[11]: frame['stamp'] for row in rows}
            before = old.update(rows, stamps)
            after, _ = source_motion_rows(rows, self.tracker, stamps)
            for source, previous, current in zip(rows, before, after):
                final[source[11]] = source, previous, current
                if source[11] == 500008:
                    self.assertEqual(current, previous)  # Jitter/forecast never gains a short-fit row.
                for index in (0, 1, 6, 7, 8, 11, 14, 15):
                    self.assertEqual(current[index], source[index])
        source, previous, current = final[500007]
        self.assertLess(previous[5], 3.5)
        self.assertAlmostEqual(current[5], 4.6634388263, places=7)
        self.assertGreaterEqual(self.tracker.ratio[500007], 1.1)
        self.assertLessEqual(current[5]/self.tracker.ratio[500007], source[5]+1e-8)
        history = list(self.tracker.history[500007])
        output, _ = source_motion_rows([source], self.tracker, {500007: fixture['frames'][-1]['stamp']})
        self.assertAlmostEqual(output[0][5], current[5])
        self.assertEqual(self.tracker.history[500007], history)


if __name__ == '__main__':
    unittest.main()
