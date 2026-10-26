"""Cold inside occupancy never removes the live forecast's emergency authority."""
from dataclasses import replace
import json
import math
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

import numpy as np

from driving_plan import DrivingPlan
from gui import HdMapLanes
from junction_recovery import JunctionRecovery
from live_path_safety import LivePathSafety
from path_safety import PathSafety
from stop_line import competition_stop_controller
from test_junction_entry import car
from test_ped_guard import person
import test_live_path_safety as fixtures
from traffic_planner import GapPlanner


class JunctionRecoveryTests(unittest.TestCase):
    def layout(self):
        lanes = HdMapLanes.__new__(HdMapLanes)
        lanes.points, lanes.links, lanes.stop_line_nodes = np.empty((0, 2)), [], {}
        lanes.set_route(np.column_stack((np.arange(0., 201., .5), np.zeros(402))))
        line = dict(id='entry', node_id='A1256W000709', station_m=100., xy=[100., 0.])
        region = dict(node_id=line['node_id'], line_id=line['id'], stop_station_m=100.,
                      start_station_m=20., end_station_m=150.)
        controller = competition_stop_controller([line])
        ego = dict(x=104., y=0., heading_deg=0., speed_kmh=0., stamp=100., age_s=.01,
                   received=99.99)
        return lanes, controller, [region], ego

    def provider(self, *, cars=(), objects=(), points=(), speed=0.):
        helper = fixtures.LivePathSafetyTests()
        runtime = helper.attach(helper.runtime())
        lanes, controller, regions, ego = self.layout()
        ego['speed_kmh'] = speed
        runtime.test_status['ego'] = ego
        runtime.lanes, runtime.stop_line_controller, runtime.intersection_regions = lanes, controller, regions
        runtime.junction_recovery = JunctionRecovery()
        self.assertTrue(runtime.junction_recovery.observe(ego, controller, lanes, regions, 100.))
        controller.update(104., speed/3.6, 100., True, observation_age_s=.01,
                          empty_front_station=107.845)
        world = np.column_stack((np.arange(104., 164.01, .5), np.zeros(121)))
        runtime.test_route = world
        runtime.gap_planner = GapPlanner()
        runtime.gap_planner.junction = dict(line_id='entry', line_station_m=100.,
            line_gap_m=-7.845, recovery_only=True, yield_only=True, following_only=False,
            empty_actual_body=True, ready=True, held=True)
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
            list(cars), True, dict(stamp=99.9, age_s=.1)))
        runtime.object_receiver.objects = list(objects)
        runtime.test_scan['xyz'] = np.asarray(points, float).reshape(-1, 3)
        runtime.sim_clock = SimpleNamespace(rtf=1.)
        local = world-[ego['x'], ego['y']]
        plan = DrivingPlan(1, local, ego, ego, 99.9, .1, 100., 8., 60., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False), 'LATTICE')
        return runtime, plan, world

    def check(self, runtime, plan, world):
        return runtime.path_publisher._safety_provider(world, plan.control_ego, plan.mode, plan)

    def test_actual_map_inside_pose_is_eligible_but_does_not_commit_a_gap(self):
        root = Path(__file__).parent
        lanes = HdMapLanes(root/'link_set.json')
        route = np.array([[p['x'], p['y']] for p in json.loads((root/'global_path.json').read_text())['waypoints']])
        lanes.set_route(route)
        controller = competition_stop_controller(lanes.stop_lines)
        line = controller.lines[0]
        region = dict(node_id=line['node_id'], line_id=line['id'], stop_station_m=line['station_m'],
                      start_station_m=line['station_m']-80., end_station_m=line['station_m']+55.)
        ego = dict(x=-98., y=321.4, heading_deg=67.9, speed_kmh=0., stamp=100., age_s=.01)
        memory = JunctionRecovery()
        self.assertTrue(memory.observe(ego, controller, lanes, [region], 100.))
        self.assertFalse(memory.active)
        context = memory.context(ego, controller, lanes, [region], 100.)
        self.assertEqual(context['phase'], 'RECOVERY_FOLLOW')
        self.assertFalse(controller.released)

    def test_bootstrap_excludes_preline_corner_overlap_wrong_route_and_heading(self):
        for changes in (dict(x=98.), dict(x=101.3, heading_deg=5.), dict(y=.501),
                        dict(heading_deg=6.01), dict(x=112.01), dict(x=float('nan'))):
            with self.subTest(changes=changes):
                lanes, controller, regions, ego = self.layout()
                memory = JunctionRecovery()
                self.assertFalse(memory.observe(dict(ego, **changes), controller, lanes, regions, 100.))
                self.assertFalse(memory.active)

    def test_previous_approach_completed_region_and_teleport_cannot_rearm_bootstrap(self):
        for variant in ('approach', 'completed', 'teleport', 'existing_wait'):
            with self.subTest(variant=variant):
                lanes, controller, regions, ego = self.layout()
                memory = JunctionRecovery()
                if variant == 'completed':
                    controller.completed.add('entry')
                elif variant == 'existing_wait':
                    controller.update(104., 0., 99.9, False, empty_front_station=107.845)
                elif variant == 'approach':
                    self.assertFalse(memory.observe(dict(ego, x=95.), controller, lanes, regions, 99.99))
                else:
                    self.assertTrue(memory.observe(ego, controller, lanes, regions, 100.))
                    self.assertFalse(memory.observe(dict(ego, x=108., stamp=100.1, received=100.1),
                                                    controller, lanes, regions, 100.1))
                self.assertIsNone(memory.context(ego, controller, lanes, regions, 100.))

    def test_positive_recovery_keeps_gp_uncommitted_and_checks_every_npc(self):
        runtime, plan, world = self.provider(cars=[car(124., 0., 3., 0., 7)])
        original = PathSafety.check
        checks = []
        def checked(guard, *args, **kwargs):
            checks.append(kwargs)
            return original(guard, *args, **kwargs)
        with patch('time.monotonic', return_value=100.), patch.object(PathSafety, 'check', checked):
            result = self.check(runtime, plan, world)
        self.assertEqual(len(checks), 1)
        self.assertFalse(checks[0]['timed_vehicles'])
        self.assertFalse(checks[0]['timed_vehicle_keys'])
        self.assertGreater(result.allowed_mps, 0.)
        self.assertFalse(result.emergency, result.stop_reason)
        self.assertTrue(runtime.junction_recovery.active)
        self.assertTrue(runtime.stop_line_controller.released)
        self.assertFalse(runtime.stop_line_controller.yield_launch_pending)
        self.assertFalse(runtime.gap_planner.junction_committed)
        self.assertFalse(runtime.path_publisher._safety_provider.traffic.junction_committed)
        self.assertEqual(result.validation['live_safety']['junction_traffic']['state'], 'RECOVERY_FOLLOW')
        self.assertNotIn('junction_motion', result.validation)
        self.assertIn('junction_recovery', result.validation)

    def test_future_npc_and_named_p_person_emergencies_remain_authoritative(self):
        for kind in ('npc', 'person'):
            with self.subTest(kind=kind):
                obj = car(110., 3., 0., -2., 7) if kind == 'npc' else person(110., 3., 0., -2.)
                runtime, plan, world = self.provider(
                    cars=[obj] if kind == 'npc' else [], objects=[obj] if kind == 'person' else [], speed=12.)
                with patch('time.monotonic', return_value=100.):
                    result = self.check(runtime, plan, world)
                self.assertGreater(result.validation['junction_current_semantic_clear_m'], .1)
                self.assertFalse(result.validation.get('junction_current_contact', False))
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)
                self.assertIn('live path collision:', result.stop_reason)
                if kind == 'person':
                    self.assertTrue(result.validation['live_safety']['people_emergency'])
                self.assertFalse(runtime.junction_recovery.active)
                self.assertFalse(runtime.stop_line_controller.released)
                self.assertFalse(runtime.gap_planner.junction_committed)

    def test_raw_current_npc_and_unknown_contact_always_stop_recovery(self):
        for hazard in ('raw', 'moving_npc', 'unknown_npc'):
            with self.subTest(hazard=hazard):
                runtime, plan, world = self.provider(
                    cars=[] if hazard == 'raw' else [car(109.5, 0., 3., 0., 7, valid=hazard != 'unknown_npc')],
                    points=[[3.8, 0., .6]] if hazard == 'raw' else [])
                with patch('time.monotonic', return_value=100.):
                    result = self.check(runtime, plan, world)
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)
                self.assertTrue(result.validation['junction_current_contact'])
                self.assertFalse(runtime.junction_recovery.active)
                self.assertFalse(runtime.gap_planner.junction_committed)

    def test_active_recovery_still_stops_for_a_new_current_body_contact(self):
        runtime, plan, world = self.provider()
        with patch('time.monotonic', return_value=100.):
            positive = self.check(runtime, plan, world)
        self.assertGreater(positive.allowed_mps, 0.)
        self.assertTrue(runtime.junction_recovery.active)
        ego = dict(plan.control_ego, stamp=100.1, received=100.1)
        runtime.test_status['ego'] = ego
        runtime.test_scan.update(stamp=100., age_s=.1)
        runtime.object_receiver.observation_stamp = 100.
        runtime.vehicle_receiver.snapshot = lambda: (
            [car(109.5, 0., 3., 0., 7)], True, dict(stamp=100., age_s=.1))
        current = replace(plan, control_ego=ego, validation_ego=ego,
                          checked_at=100.1, sensor_stamp=100.)
        with patch('time.monotonic', return_value=100.1):
            stopped = self.check(runtime, current, world)
        self.assertTrue(stopped.validation['junction_current_contact'])
        self.assertEqual(stopped.allowed_mps, 0.)
        self.assertTrue(stopped.emergency)
        self.assertFalse(runtime.gap_planner.junction_committed)

    def test_stale_sources_pose_and_forged_mode_cannot_authorize_recovery(self):
        for variant in ('source', 'pose', 'no_bootstrap', 'blocked', 'wrong_path'):
            with self.subTest(variant=variant):
                runtime, plan, world = self.provider()
                if variant == 'source':
                    runtime.vehicle_receiver.snapshot = lambda: ([], True, dict(stamp=99.649, age_s=.351))
                elif variant == 'pose':
                    runtime.test_status['ego'].update(age_s=.201, received=99.799)
                elif variant == 'no_bootstrap':
                    runtime.junction_recovery = JunctionRecovery()
                elif variant == 'wrong_path':
                    world = world + [0., 2.]
                    plan = replace(plan, path=world-[104., 0.])
                else:
                    plan = plan.decided(blocked=True)
                with patch('time.monotonic', return_value=100.):
                    result = self.check(runtime, plan, world)
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)
                self.assertFalse(runtime.junction_recovery.active)

    def test_reported_pose_age_and_receipt_clock_fail_closed_independently(self):
        invalid = (dict(age_s=-.001), dict(age_s=float('nan')),
                   dict(received=100.001), dict(received=float('nan')),
                   dict(received=float('inf')))
        for changes in invalid:
            with self.subTest(changes=changes):
                lanes, controller, regions, ego = self.layout()
                self.assertFalse(JunctionRecovery().observe(
                    dict(ego, **changes), controller, lanes, regions, 100.))
                runtime, plan, world = self.provider()
                runtime.test_status['ego'].update(changes)
                with patch('time.monotonic', return_value=100.):
                    result = self.check(runtime, plan, world)
                self.assertEqual(result.allowed_mps, 0.)
                self.assertTrue(result.emergency)
                self.assertFalse(runtime.junction_recovery.active)
                self.assertFalse(runtime.stop_line_controller.released)

    def test_real_runtime_bootstrap_publishes_recovery_without_a_gap_commit(self):
        import threading
        import virtual_sim as simulator
        world = simulator.load_world()
        simulation = simulator.Sim(world, 1785, speed_kmh=0., actors=[], end_idx=1840,
            duration_s=.01, start_pose=(-98., 321.4, 67.9), highway=False, launch_lag=True)
        runtime, sensors = simulation.rt.r, simulation.rt
        # SimulatedRuntime bypasses Runtime.__init__; opt in to the same boot
        # object that the production constructor always installs.
        runtime.junction_recovery = JunctionRecovery()
        runtime.object_receiver.snapshot_with_time = lambda: (list(sensors.objects), sensors.clock.now, .01)
        runtime.lidar.scan_snapshot = lambda: dict(obstacle_points=runtime.lidar.pts,
            stamp=sensors.clock.now, age_s=.01, sequence=int(sensors.clock.now/simulator.DT))
        runtime.vehicle_receiver.snapshot = lambda: (list(sensors.cars), sensors.vehicle_ready,
            dict(stamp=sensors.clock.now, age_s=.01, ready=sensors.vehicle_ready, vehicles=[]))
        runtime.path_publisher._command_lock = threading.RLock()
        runtime.path_publisher._safety_provider = LivePathSafety(runtime)
        simulation.run()
        plan = runtime.path_publisher._active_decision_plan
        self.assertGreater(plan.allowed_mps, 0., plan.stop_reason)
        self.assertFalse(plan.emergency)
        self.assertEqual(runtime.gap_planner.status['state'], 'RECOVERY_FOLLOW')
        self.assertFalse(runtime.gap_planner.junction_committed)
        self.assertTrue(runtime.stop_line_controller.released)
        self.assertTrue(plan.validation['live_safety']['recovery_full_npc_forecast'])
        self.assertIn('junction_recovery', plan.validation)

    def test_body_compute_and_controller_lock_cannot_extend_recovery_deadline(self):
        runtime, plan, world = self.provider()
        clock = [100.]
        original = PathSafety.check
        def delayed(guard, *args, **kwargs):
            result = original(guard, *args, **kwargs)
            clock[0] += .191
            return result
        with patch('time.monotonic', side_effect=lambda: clock[0]), patch.object(PathSafety, 'check', delayed):
            result = self.check(runtime, plan, world)
        self.assertAlmostEqual(result.validation['junction_recovery']['expires_at'], 100.19)
        self.assertEqual(result.allowed_mps, 0.)
        self.assertTrue(result.emergency)
        self.assertFalse(runtime.stop_line_controller.released)

    def test_proof_binds_full_pose_path_and_finite_locked_publication_clock(self):
        runtime, plan, world = self.provider()
        memory = runtime.junction_recovery
        proof = memory.prepare(plan.control_ego, world, runtime.stop_line_controller,
            runtime.lanes, runtime.intersection_regions, 100., ready=True, source_ages=[.1])
        self.assertIsNotNone(proof)
        self.assertFalse(memory.matches(proof, dict(plan.control_ego, x=104.01), world, 100.))
        self.assertFalse(memory.matches(proof, plan.control_ego, world+[0., .01], 100.))
        self.assertFalse(memory.matches(proof, plan.control_ego, world, 99.99))
        for clock in (float('nan'), 99.99, proof['expires_at']+.000001):
            with self.subTest(clock=clock), patch('time.monotonic', return_value=clock):
                self.assertFalse(runtime.stop_line_controller.consume_inside_recovery(
                    'entry', 100., 104., checked_at=100., expires_at=proof['expires_at']))
                self.assertFalse(runtime.stop_line_controller.released)

    def test_controller_completes_at_old_boundary_but_phase_carries_to_region_end(self):
        runtime, plan, world = self.provider()
        with patch('time.monotonic', return_value=100.):
            result = self.check(runtime, plan, world)
        self.assertGreater(result.allowed_mps, 0.)
        memory = runtime.junction_recovery
        # A sequence of physical samples, rather than a teleport beyond the line.
        for step in range(1, 11):
            ego = dict(plan.control_ego, x=104.+step, speed_kmh=36., stamp=100.+step*.1,
                       received=100.+step*.1)
            self.assertTrue(memory.observe(ego, runtime.stop_line_controller, runtime.lanes,
                                           runtime.intersection_regions, 100.+step*.1))
        runtime.stop_line_controller.update(114., 10., 101., True, empty_front_station=117.845)
        self.assertIn('entry', runtime.stop_line_controller.completed)
        self.assertIsNotNone(memory.context(ego, runtime.stop_line_controller, runtime.lanes,
                                           runtime.intersection_regions, 101.))
        for step in range(11, 48):
            ego = dict(ego, x=104.+step, stamp=100.+step*.1, received=100.+step*.1)
            memory.observe(ego, runtime.stop_line_controller, runtime.lanes,
                           runtime.intersection_regions, 100.+step*.1)
        self.assertFalse(memory.active)
        self.assertEqual(memory.reason, 'recovery region complete')

    def test_expired_recovery_is_stopped_at_final_timer_command_serialization(self):
        helper = fixtures.LivePathSafetyTests()
        runtime = helper.attach(helper.runtime())
        runtime.infer()
        publisher, clock = runtime.path_publisher, [time.monotonic()+.1]
        original_provider = publisher._safety_provider
        def expiring(*args, **kwargs):
            result = original_provider(*args, **kwargs)
            return result.decided(junction_recovery=dict(expires_at=clock[0]+.01))
        publisher._safety_provider = expiring
        publisher._last_target_mps = 0.
        original_summary = DrivingPlan.summary
        def delayed_summary(plan):
            answer = original_summary(plan)
            if plan.allowed_mps > 0. and 'junction_recovery' in plan.validation:
                clock[0] += .02
            return answer
        with patch('time.monotonic', side_effect=lambda: clock[0]), patch.object(DrivingPlan, 'summary', delayed_summary):
            publisher.recheck_active_path()
        payload = helper.payload(runtime)
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertEqual(payload['decision']['allowed_mps'], 0.)
        self.assertTrue(payload['decision']['emergency'])
        self.assertEqual(publisher._last_target_mps, 0.)


if __name__ == '__main__':
    unittest.main()
