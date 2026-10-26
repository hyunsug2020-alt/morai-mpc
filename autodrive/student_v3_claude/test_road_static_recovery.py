"""Static bypass body geometry and bounded recovery from the 2026-10-15 stall."""
import time
import unittest
import json
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
from ego_body import EGO_FRONT_M, EGO_REAR_M, REAR_AXLE_SHIFT_M
from lane_corridor import LaneCorridor
from tunnel_runtime import swept_clearance
from driving_plan import DrivingPlan, static_reverse_release_ready


class RecordedRoadStaticPadding(unittest.TestCase):
    def setUp(self):
        with np.load(Path(__file__).parent/'test_data/road_static_padding_20261015.npz') as frame:
            self.ego = json.loads(str(frame['ego']))
            self.route, self.path, self.points = (frame[k].copy() for k in ('route', 'path', 'points'))
            self.objects = json.loads(str(frame['objects']))
            self.corridor = LaneCorridor(self.route, frame['left'].copy(), frame['right'].copy())

    def test_recorded_clear_bypass_uses_one_static_margin_budget(self):
        from tunnel_runtime import lidar_objects, corridor_points
        old = gui.static_clear_distance(self.path, self.ego, self.objects, 60.,
                                       body_shift_m=REAR_AXLE_SHIFT_M)
        lane, clear = ma.road_static_checks(self.corridor, self.points)
        self.assertEqual(old, .5)
        self.assertGreater(clear(self.path, 60.), 58.)
        boxes = ma.road_lidar_objects(corridor_points(self.points, self.corridor), self.ego)
        static = self.objects[:1]+ma.merge_lidar_obstacles(self.objects[:1], boxes)
        self.assertGreater(gui.static_clear_distance(self.path, self.ego, static, 60.,
                           body_shift_m=REAR_AXLE_SHIFT_M), 58.)
        previous = dict(path=self.path, direction=-1., offset_m=.45, transition_m=22.,
                        hold_m=25., hold_until_m=30., kind='bypass',
                        curvature=float(max(gui.path_curvatures(self.path))))
        plan, _, _ = ma.plan_bypass(self.route, self.ego, static, lane, clear, -1., previous, {},
                                   body_shift_m=REAR_AXLE_SHIFT_M, budget_s=2.)
        self.assertFalse(plan.get('blocked'), plan)
        self.assertTrue(ma.static_plan_complete(plan))

    def test_collision_and_unknown_geometry_remain_blocked(self):
        from tunnel_runtime import corridor_points
        unsafe = np.vstack((self.points, [3.9, 0.]))
        lane, clear = ma.road_static_checks(self.corridor, unsafe)
        self.assertEqual(clear(self.path, 60.), 0.)
        boxes = ma.road_lidar_objects(corridor_points(unsafe, self.corridor), self.ego)
        plan, _, _ = ma.plan_bypass(self.route, self.ego, boxes, lane, clear, -1.,
                                   adjacent_offsets={}, body_shift_m=REAR_AXLE_SHIFT_M, budget_s=2.)
        self.assertTrue(plan and plan.get('blocked'))

    def test_road_boxes_still_enclose_every_measured_return_at_oblique_angles(self):
        from tunnel_runtime import lidar_objects
        grid = np.array([(x, y) for x in (-.2, 0., .2) for y in (-.4, -.2, 0., .2, .4)])
        ego = dict(x=0., y=0., heading_deg=0.)
        for degrees in (0., 22.5, 45.):
            angle = np.deg2rad(degrees)
            rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            points = grid@rot.T+[15., 1.4]
            # The clustering API quantizes inputs to its existing 20 cm cells.
            measured = np.unique(np.round(points/.2), axis=0)*.2
            road = ma.road_lidar_objects(points, ego)
            tunnel = lidar_objects(points, ego)
            self.assertEqual(len(road), 1)
            self.assertEqual(len(tunnel), 1)
            b = road[0]
            yaw = np.deg2rad(b[8])
            axes = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            local = (measured-np.array(b[:2]))@axes
            self.assertTrue((abs(local) <= np.array(b[6:8])/2.+1e-6).all())
            self.assertTrue(all(road[0][i] <= tunnel[0][i] for i in (6, 7)))


class RoadStaticGeometry(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 60.01, .25), np.zeros(241)))
        self.corridor = LaneCorridor(self.route, np.full(241, 3.5), np.full(241, -3.5))

    def test_raw_clearance_uses_actual_front_without_mutating_shared_corridor(self):
        points = np.array([[5.5, 0.]])
        _, clear = ma.road_static_checks(self.corridor, points)
        actual = clear(self.route, 30.)
        self.assertEqual(actual, swept_clearance(self.route, points, 30., .25,
                         lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M))
        self.assertLess(actual, swept_clearance(self.route, points, 30., .25))
        self.assertEqual(self.corridor.body_shift_m, 0.)

    def test_recorded_front_left_returns_cannot_approve_a_new_straight_departure(self):
        # Logged at (-60.04, -119.78), heading 86.98 degrees, body gap 0.054 m.
        points = np.array([[3.8, 1.], [3.8, 1.2], [3.8, 1.4], [4., 1.]])
        _, clear = ma.road_static_checks(self.corridor, points)
        self.assertEqual(clear(self.route, 30.), 0.)

    def test_missing_map_or_scan_never_establishes_free_space(self):
        for corridor, points in [(None, np.empty((0, 2))), (self.corridor, None)]:
            with self.subTest(corridor=corridor is not None):
                _, clear = ma.road_static_checks(corridor, points)
                self.assertEqual(clear(self.route, 20.), 0.)
        lane, clear = ma.road_static_checks(self.corridor, np.empty((0, 2)))
        self.assertTrue(lane(self.route))
        self.assertEqual(clear(self.route, 20.), 20.)

    def test_bypass_is_clear_for_the_real_body_and_measured_obstacle(self):
        points = np.array([[15., y] for y in np.arange(.8, 1.61, .1)])
        lane, clear = ma.road_static_checks(self.corridor, points)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=10.)
        box = (15.4, 1.2, .8, 'box', 'obstacle', 0., .8, .8, 0.)
        plan, _, _ = ma.plan_bypass(self.route, ego, [box], lane, clear,
            committed_direction=-1., adjacent_offsets={}, body_shift_m=REAR_AXLE_SHIFT_M)
        self.assertIsNotNone(plan)
        self.assertFalse(plan.get('blocked'), plan)
        end = plan['hold_until_m']
        self.assertGreaterEqual(clear(plan['path'], end), end-.01)
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], ego, [box], end,
                                body_shift_m=REAR_AXLE_SHIFT_M), end-.01)

    def test_live_publication_preserves_static_body_and_restores_other_road_modes(self):
        import test_live_path_safety as fixtures
        harness = fixtures.LivePathSafetyTests()
        runtime = harness.attach(harness.runtime())
        runtime.infer()
        publisher = runtime.path_publisher
        baseline = publisher._active_decision_plan
        provider = publisher._safety_provider
        world = np.array([[p['x'], p['y']] for p in harness.payload(runtime)['waypoints']])
        checked = baseline.decided(static_rear_axle=True)
        result = provider(world, baseline.control_ego, baseline.mode, checked)
        self.assertTrue(result.validation['static_rear_axle'])
        self.assertAlmostEqual(provider.guard.front_m, EGO_FRONT_M)
        self.assertAlmostEqual(provider.guard.rear_m, EGO_REAR_M)
        ordinary = checked.decided(static_rear_axle=False)
        provider(world, baseline.control_ego, baseline.mode, ordinary)
        self.assertAlmostEqual(provider.guard.front_m, 2.3175)


class StaticReverseRecovery(unittest.TestCase):
    def setUp(self):
        self.starts = []
        publisher = SimpleNamespace(start_reverse=lambda *a, **k: self.starts.append((a, k)),
                                    reverse_active=lambda ego: False)
        self.host = SimpleNamespace(path_publisher=publisher, object_receiver=None, _contact=None,
                                    _reverse_wait_since=time.monotonic()-10.)
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)

    def attempt(self, x, *, static=True, points=None):
        self.host._reverse_wait_since = time.monotonic()-10.
        return gui.Runtime.reverse_escape_check(self.host, dict(self.ego, x=x), True, 0.,
            np.empty((0, 2)) if points is None else points, time.monotonic(),
            static_recovery=static, swing_check=True)

    def test_static_obstacle_never_starts_a_timed_reverse_even_after_progress(self):
        for x in (0., -6., 0., 2.1, 10., 0.):
            self.assertFalse(self.attempt(x))
        self.assertEqual(self.starts, [])
        self.assertIn('checked forward bypass', self.host._reverse_veto)

    def test_static_planner_does_not_cancel_a_protective_reverse(self):
        # A real approaching person may already have started its own backoff.
        self.host.path_publisher.reverse_active = lambda ego: True
        self.host.path_publisher.reverse_abort = False
        self.assertTrue(self.attempt(-2.))
        self.assertFalse(self.host.path_publisher.reverse_abort)
        self.assertEqual(self.starts, [])

    def test_rear_obstacle_still_vetoes_the_first_escape(self):
        self.assertFalse(self.attempt(0., static=False, points=np.array([[-1.2, 0.]])))
        self.assertEqual(self.starts, [])

    def test_other_recovery_policy_retains_its_existing_budget(self):
        self.assertTrue(self.attempt(0., static=False))
        self.assertTrue(self.attempt(-6., static=False))


class ModelRoadRuntime(unittest.TestCase):
    def test_empty_model_road_can_release_on_a_fresh_clear_route(self):
        import test_avoidance_gate as fixtures
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP')
        runtime.test_status['path']['nearest_idx'] = 550
        with patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            self.assertTrue(runtime.infer())
        self.assertTrue(runtime.last_result['postprocess']['static_complete'])

    def test_blocked_saved_model_path_rechecks_actual_body_without_terminating_inference(self):
        import test_avoidance_gate as fixtures
        box = (18., 0., 1., 'box', 'obstacle', 0., 1., 1., 0.)
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=[box])
        runtime.test_status['path']['nearest_idx'] = 550
        with patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            self.assertTrue(runtime.infer())
            self.assertIsNotNone(runtime._static_plan_saved)
            previous = runtime.model_static_plan

            def blocked(*args, **kwargs):
                plan, hazards, context = previous(*args, **kwargs)
                return dict(blocked=True, kind='bypass', rejected={}), hazards, context

            with patch.object(runtime, 'model_static_plan', side_effect=blocked), \
                    patch.object(ma.portable, 'recover_static_prefix', return_value=None) as recover:
                self.assertTrue(runtime.infer())
                recover.assert_called_once()
                self.assertEqual(recover.call_args.kwargs['body_shift_m'], REAR_AXLE_SHIFT_M)
            self.assertEqual(runtime._web_record['mode'], 'STUDENT HOLD')


class RecordedCanonicalRejoinMemory(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(Path(__file__).parent/'test_data/road_static_return_fallback_20261019.npz') as data:
            cls.frames = {key: data[key].copy() for key in data.files}
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    def frame(self, sequence, key):
        value = self.frames[f'frame{sequence}_{key}']
        return json.loads(str(value)) if value.ndim == 0 else value.copy()

    def infer_recorded(self, sequence, *, model=True, context_changes=None, plan_changes=None):
        import test_avoidance_gate as fixtures
        ego, plan, proof, now = [self.frame(sequence, key) for key in ('ego', 'plan', 'proof', 'now')]
        plan.update(plan_changes or {})
        points = self.frame(sequence, 'points')
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP')
        runtime.test_status = dict(ego=dict(ego, age_s=.01),
                                   path=dict(nearest_idx=800, distance_m=.1))
        runtime.lidar.pts, runtime.lidar.received = points.astype(np.float32), now
        runtime.test_scan = dict(stamp=proof['road_scan_frame']['stamp'],
            sequence=proof['road_scan_frame']['sequence'], obstacle_points=runtime.lidar.pts,
            age_s=.01, xyz=np.empty((0, 3)))
        runtime._contact = None
        context = dict(static=self.frame(sequence, 'static'), clearance=lambda *args: 60.,
            points=points, planning=True, steady_static=True, direction=0.,
            route_return_ready=True, standing_bypass=False, hazard_m=None)
        context.update(context_changes or {})
        wall = proof['road_scan_frame']['stamp']+proof['road_scan_frame']['wall_age_s']
        with ExitStack() as stack:
            for target, name, value in ((gui, 'MODEL_AVOID', model), (gui, 'USE_LATTICE', False),
                                       (gui, 'model_avoidance', ma)):
                stack.enter_context(patch.object(target, name, value, create=True))
            stack.enter_context(patch.object(runtime, 'model_static_plan', return_value=(plan, [], context)))
            stack.enter_context(patch.object(gui, 'plan_static_bypass', return_value=(plan, [])))
            stack.enter_context(patch('static_lane_pass.normal_static_area', return_value=True))
            stack.enter_context(patch('perception.road_scan_frame.align_road_scan_wait',
                return_value=dict(points=points, proof=proof['road_scan_frame'])))
            stack.enter_context(patch('gui.time.time', return_value=wall))
            stack.enter_context(patch('gui.time.monotonic', return_value=now))
            self.assertTrue(runtime.infer(), runtime.input_error)
        return runtime, ego, plan, now

    def test_recorded_zero_direction_rejoins_keep_world_geometry_for_the_next_pose(self):
        # The native run saved frame 126/128 but lost it on 127/129 solely
        # because the route-centre side latch was released.
        for sequence in (126, 128):
            with self.subTest(sequence=sequence):
                self.assertIsNotNone(self.frame(sequence, 'saved'))
                self.assertIsNone(self.frame(sequence+1, 'saved'))
                runtime, ego, plan, now = self.infer_recorded(sequence)
                self.assertEqual(runtime.avoidance_direction, 0.)
                self.assertEqual(runtime.avoidance_latch_until, 0.)
                self.assertEqual(runtime._web_record['mode'], 'STUDENT REJOIN')
                saved = runtime._static_plan_saved
                self.assertIsNotNone(saved)
                self.assertEqual(saved['stamp'], now)
                expected = [gui.RosPathPublisher.global_xy(ego, float(p[0]), float(p[1]))
                            for p in plan['path']]
                np.testing.assert_allclose(saved['world_path'], expected, atol=1e-12)
                continued = gui.continue_static_plan(saved, self.frame(sequence+1, 'ego'),
                                                     self.frame(sequence+1, 'now'))
                self.assertIsNotNone(continued)
                self.assertGreater(len(continued['path']), 100)
                self.assertTrue(ma.static_plan_complete(continued))

    def test_zero_direction_memory_permission_does_not_expand_other_modes(self):
        cases = [dict(model=False), dict(context_changes=dict(steady_static=False)),
                 dict(context_changes=dict(standing_bypass=True)),
                 dict(plan_changes=dict(canonical_control=False)),
                 dict(plan_changes=dict(standing_bypass_ids=[1])),
                 dict(plan_changes=dict(bounded_static_pass=True)),
                 dict(plan_changes=dict(static_borrow=True)),
                 dict(plan_changes=dict(clear_distance_m=0.))]
        for changes in cases:
            with self.subTest(changes=changes):
                runtime, _, _, _ = self.infer_recorded(126, **changes)
                self.assertEqual(runtime.avoidance_direction, 0.)
                self.assertIsNone(runtime._static_plan_saved)


class RoadReturnCompletion(unittest.TestCase):
    def check(self, *, points=(), objects=(), normal=True, previous_changes=None):
        import test_avoidance_gate as fixtures
        runtime = fixtures.AvoidanceGateTests().runtime(action='KEEP', objects=objects, points=points)
        route = runtime.test_route
        ego = dict(runtime.test_status['ego'], speed_kmh=0.)
        corridor = runtime.lanes.corridor(ego, route)
        previous = dict(path=route.copy(), direction=-1., offset_m=.5, kind='rejoin',
            transition_m=18., hold_m=25., hold_until_m=30., safe_horizon_m=35.,
            clear_distance_m=60., curvature=0., speed_cap_mps=2., canonical_control=True)
        previous.update(previous_changes or {})
        with patch.object(gui, 'model_avoidance', ma, create=True), \
                patch('static_lane_pass.normal_static_area', return_value=normal), \
                patch.object(ma, 'choose_direction', wraps=ma.choose_direction) as direction, \
                patch.object(ma, 'road_rejoin_plan', return_value=None) as rejoin, \
                patch.object(ma, 'plan_bypass', return_value=(None, [], {})) as bypass:
            _, _, context = runtime.model_static_plan(ego, route, list(objects),
                np.asarray(points, dtype=float).reshape(-1, 2), corridor, lambda *a: True,
                lambda path, horizon=60.: horizon, previous, {}, time.monotonic(), True, True, .946)
        return runtime, context, direction, rejoin, bypass

    def test_full_raw_or_object_hazard_keeps_geometry_but_releases_the_old_side(self):
        box = (20., 0., 1., 'box', 'obstacle', 0., 1., 1., 0.)
        for inputs in (dict(points=[(20., 0.)]), dict(objects=[box])):
            with self.subTest(inputs=inputs):
                runtime, context, direction, rejoin, _ = self.check(**inputs)
                self.assertTrue(context['route_return_ready'])
                self.assertFalse(context['route_geometry_return_ready'])
                self.assertIsNone(direction.call_args.args[0])
                self.assertIsNotNone(rejoin.call_args.args[3])
                self.assertFalse(runtime._model_avoidance_log['route_geometry_return_ready'])

    def test_full_clear_route_discards_geometry_and_other_areas_keep_the_immediate_policy(self):
        exceptions = [dict(points=[(20., 0.)], previous_changes={key: value})
                      for key, value in (('bounded_static_pass', True), ('static_borrow', True),
                                         ('standing_bypass_ids', [1]))]
        for inputs in ({}, dict(points=[(20., 0.)], normal=False), *exceptions):
            with self.subTest(inputs=inputs):
                _, context, direction, rejoin, bypass = self.check(**inputs)
                self.assertTrue(context['route_return_ready'])
                self.assertTrue(context['route_geometry_return_ready'])
                self.assertIsNone(direction.call_args.args[0])
                planner_previous = (rejoin.call_args.args[3] if rejoin.called else bypass.call_args.args[6])
                self.assertIsNone(planner_previous)


class StaticReverseRelease(unittest.TestCase):
    def plan(self, stamp=100., complete=True):
        path = np.column_stack((np.arange(0., 30.1, .25), np.zeros(121)))
        ego = dict(x=0., y=0., heading_deg=0., stamp=stamp, speed_kmh=0.)
        return DrivingPlan(1, path, ego, ego, stamp, .01, time.monotonic(), 2., 30., False,
            dict(valid=True, pose_valid=True, steering_valid=True, blocked=False,
                 static_rear_axle=True, static_complete=complete), mode='STUDENT FORCED')

    def test_braking_prefix_cannot_release_reverse(self):
        p = self.plan(complete=False)
        self.assertFalse(p.forward_ready(p.control_ego))
        self.assertFalse(static_reverse_release_ready({}, p, True))
        p = self.plan()
        self.assertTrue(p.forward_ready(p.control_ego))

    def test_repeated_old_scan_cannot_release_reverse(self):
        state = {}
        for _ in range(10):
            self.assertFalse(static_reverse_release_ready(state, self.plan(), True))
        self.assertFalse(static_reverse_release_ready(state, self.plan(100.1), True))
        self.assertTrue(static_reverse_release_ready(state, self.plan(100.2), True))

    def test_failed_or_rewound_observation_resets_clear_history(self):
        state = {}
        for stamp in (100., 100.1):
            self.assertFalse(static_reverse_release_ready(state, self.plan(stamp), True))
        self.assertFalse(static_reverse_release_ready(state, self.plan(100.2), False))
        self.assertFalse(static_reverse_release_ready(state, self.plan(100.3), True))
        self.assertFalse(static_reverse_release_ready(state, self.plan(99.), True))
        self.assertEqual(state['static_clear_scans'], [99.])

    def test_other_reverse_policies_keep_their_existing_release(self):
        p = self.plan().decided(static_rear_axle=False)
        self.assertTrue(static_reverse_release_ready({}, p, True))
        self.assertFalse(static_reverse_release_ready({}, p, False))

    def test_real_publisher_keeps_reverse_until_three_complete_sensor_proofs(self):
        import test_live_path_safety as fixtures
        harness = fixtures.LivePathSafetyTests()
        runtime = harness.attach(harness.runtime())
        runtime.test_status['ego']['speed_kmh'] = 0.
        runtime.infer()
        publisher = runtime.path_publisher
        base = publisher._active_decision_plan
        publisher.start_reverse(base.control_ego, static_recovery=True)
        post = dict(mode='STUDENT FORCED', reason='checked static bypass', validated_static=True,
                    applied_peak_m=-.45, static_rear_axle=True, static_complete=True)
        for i in range(3):
            if i:
                harness.advance(runtime, .1, [])
            ego = dict(runtime.test_status['ego'])
            plan = DrivingPlan(i+100, base.path, ego, ego, runtime.test_scan['stamp'], .1,
                time.monotonic(), 2., 60., False,
                dict(valid=True, pose_valid=True, steering_valid=True, blocked=False,
                     static_rear_axle=True, static_complete=True), mode='STUDENT FORCED')
            self.assertTrue(publisher.publish(ego, plan.path, plan.path, dict(post), plan))
            if i < 2:
                self.assertIsNotNone(publisher.reverse_state)
                self.assertEqual(publisher._published_plan['gear'], 'R')
            else:
                self.assertIsNone(publisher.reverse_state)


if __name__ == '__main__':
    unittest.main()
