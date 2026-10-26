"""Static lane exceptions retain full body, sensor, traffic and return checks."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

from ego_body import REAR_AXLE_SHIFT_M, body_corners
from highway_planner import local_points
from lane_corridor import LaneCorridor
from static_lane_pass import (StaticLanePass, opposing_corridor, normal_static_area,
                             RETURN_BUDGET_S, measured_curvature_paths)


class StaticLanePassTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 100.1, .5), np.zeros(201)))
        self.legal = LaneCorridor(self.route, np.full(201, 1.75), np.full(201, -1.75),
                                  body_shift_m=REAR_AXLE_SHIFT_M)
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=20., stamp=100., wheel_angle=0.)
        opposite = self.route[::-1]+[0., 3.5]
        self.lanes = SimpleNamespace(n_links=2,
            links=[dict(idx='opposite', link_type='0', width_start=3.5, width_end=3.5)],
            geometry={'opposite': opposite}, link_trees={'opposite': cKDTree(opposite)},
            link_rule=lambda k: {}, route_progress=lambda ego: ego['x'],
            rules=lambda: {'sections': []})
        self.box = (20., -.8, .6, 'barrel', 'obstacle', 0., .6, .6, 0.)
        self.points = np.array([[x, y] for x in (19.7, 20., 20.3) for y in (-1.1, -.8, -.5)])

    def plan(self, policy=None, *, objects=None, points=None, ready=True, ego=None):
        return (policy or StaticLanePass()).plan(self.lanes, ego or self.ego, self.route,
            [self.box] if objects is None else objects, [self.box],
            self.points if points is None else points, self.legal, None, 100., .75, ready)

    def test_unknown_steering_cannot_generate_a_measured_launch(self):
        for value in (None, math.nan, math.inf, 'unknown'):
            with self.subTest(wheel=value):
                ego = dict(self.ego, wheel_angle=value)
                self.assertEqual(list(measured_curvature_paths(self.route, ego, [self.box])), [])
        ego = {key: value for key, value in self.ego.items() if key != 'wheel_angle'}
        self.assertEqual(list(measured_curvature_paths(self.route, ego, [self.box])), [])

    def test_delayed_planner_pose_cannot_undo_a_newer_observed_return(self):
        policy = StaticLanePass()
        policy.legal, policy.origin = self.legal, dict(self.ego)
        policy.observe(dict(self.ego, stamp=100.), 100.)
        policy.observe(dict(self.ego, y=1., stamp=101.), 101.)
        entered_at = policy.entered_at
        self.assertIsNotNone(entered_at)
        # The live checker has advanced while a slower planner still carries
        # an older pose. Repeated old inside poses cannot finish the excursion.
        for stamp in (100.5, 100.6):
            policy.observe(dict(self.ego, stamp=stamp), 102.)
        self.assertEqual(policy.entered_at, entered_at)
        self.assertEqual(policy.inside_frames, 0)
        self.assertEqual(policy.last_stamp, 101.)
        policy.observe(dict(self.ego, stamp=102.), 102.)
        self.assertEqual(policy.inside_frames, 1)
        # An old outside pose also cannot discard the first real inside fix.
        policy.observe(dict(self.ego, y=1., stamp=101.5), 103.)
        self.assertEqual(policy.inside_frames, 1)
        self.assertEqual(policy.last_stamp, 102.)
        policy.observe(dict(self.ego, stamp=103.), 103.)
        self.assertTrue(policy.finished)
        self.assertIsNone(policy.entered_at)

    def test_small_excursion_passes_the_barrel_and_returns_with_time_to_spare(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        self.assertTrue(plan['static_borrow'])
        self.assertLessEqual(plan['speed_cap_mps']*3.6, 25.)
        self.assertGreater(proof['contact_s'], 0.)
        self.assertLessEqual(proof['contact_s'], RETURN_BUDGET_S)
        self.assertLessEqual(proof['encroachment_m'], .75)
        self.assertGreaterEqual(plan['clear_distance_m'], plan['safe_horizon_m'])
        self.assertGreaterEqual(plan['clear_distance_m'], plan['hold_until_m'])
        self.assertAlmostEqual(plan['path'][-1, 1], 0., places=4)
        from gui import static_clear_distance
        from tunnel_runtime import swept_clearance
        self.assertGreater(static_clear_distance(plan['path'], self.ego, [self.box], 55.,
            body_shift_m=REAR_AXLE_SHIFT_M), 54.9)
        self.assertGreater(swept_clearance(plan['path'], self.points, 55., .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), 54.9)

    def test_wide_in_lane_return_can_use_thirty_without_changing_the_borrow_limit(self):
        from gui import model_static_speed_cap_mps
        from model_avoidance import fast_static_clearance
        points = np.empty((0, 2))
        plan, proof = StaticLanePass().plan(self.lanes, self.ego, self.route,
            [], [], points, self.legal, None, 100., .75, True)
        self.assertIsNotNone(plan, proof)
        self.assertFalse(plan['static_borrow'])
        plan['fast_static_clear_m'] = fast_static_clearance(plan, self.ego, [], points)
        cap = model_static_speed_cap_mps(55./3.6, plan, speed_mps=20./3.6)
        self.assertAlmostEqual(cap*3.6, 30.)
        plan['fast_static_clear_m'] = 0.
        plan['static_speed_profile']['raw_clear_m'] = 0.
        self.assertAlmostEqual(model_static_speed_cap_mps(55./3.6, plan,
            speed_mps=20./3.6)*3.6, 25.)

    def test_missing_geometry_sensor_and_rolling_speed_never_invent_a_pass(self):
        for change in ('opposite_map', 'sensor', 'slow', 'stale'):
            self.setUp()
            if change == 'opposite_map':
                self.lanes.n_links = 0
            ego = dict(self.ego, speed_kmh=2.) if change == 'slow' else self.ego
            points = np.array([[float('nan'), 0.]]) if change == 'sensor' else self.points
            with self.subTest(change=change):
                plan, _ = self.plan(ego=ego, points=points, ready=change != 'stale')
                self.assertIsNone(plan)

    def test_checked_tight_pass_keeps_twenty_five_until_wide_clearance_is_proven(self):
        from gui import static_clear_distance, model_static_speed_cap_mps
        from model_avoidance import fast_static_clearance
        from tunnel_runtime import swept_clearance
        boxes = [(8., side*1.7, .6, 'barrel', 'obstacle', 0., .6, .6, 0.)
                 for side in (-1., 1.)]
        points = np.array([[x, side*1.4] for x in (7.7, 8., 8.3) for side in (-1., 1.)])
        self.lanes.n_links = 0
        plan, proof = StaticLanePass().plan(self.lanes, self.ego, self.route,
            boxes, boxes, points, self.legal, None, 100., budget_s=1.)
        self.assertIsNotNone(plan, proof)
        self.assertFalse(plan['static_borrow'])
        plan['fast_static_clear_m'] = fast_static_clearance(plan, self.ego, boxes, points)
        self.assertLess(plan['fast_static_clear_m'], 35.)
        self.assertAlmostEqual(model_static_speed_cap_mps(55./3.6, plan,
            speed_mps=self.ego['speed_kmh']/3.6)*3.6, 25.)
        horizon = plan['clear_distance_m']
        self.assertGreaterEqual(static_clear_distance(plan['path'], self.ego, boxes,
            horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(swept_clearance(plan['path'], points, horizon, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(self.legal.clear_distance(plan['path'], horizon), horizon-.01)

    def test_oncoming_unknown_vehicle_and_nearby_person_veto_the_exception(self):
        for kind, speed in (('npc', 20.), ('npc', 0.), ('pedestrian', 0.)):
            obj = (40., 3.5, 4., 'track', kind, speed, 4., 1.8, 180., -speed, 0., 9)
            with self.subTest(kind=kind, speed=speed):
                plan, _ = self.plan(objects=[self.box, obj])
                self.assertIsNone(plan)

    def test_intersection_block_override_and_road_on_the_wrong_side_are_not_borrowed(self):
        for change in ('junction', 'blocked', 'right', 'same_direction'):
            self.setUp()
            if change == 'junction':
                self.lanes.links[0]['link_type'] = '1'
            elif change == 'blocked':
                self.lanes.link_rule = lambda k: dict(block=True)
            else:
                points = self.lanes.geometry['opposite']
                points = points-[0., 7.] if change == 'right' else points[::-1]
                self.lanes.geometry['opposite'] = points
                self.lanes.link_trees['opposite'] = cKDTree(points)
            with self.subTest(change=change):
                self.assertIsNone(opposing_corridor(self.lanes, self.ego, self.route, self.legal))

    def test_surveyed_gap_within_existing_encroachment_does_not_hide_an_opposing_lane(self):
        for gap in (.45, .75, .8, 1.2):
            with self.subTest(surveyed_gap_m=gap):
                self.setUp()
                points = self.lanes.geometry['opposite']+[0., gap]
                self.lanes.geometry['opposite'] = points
                self.lanes.link_trees['opposite'] = cKDTree(points)
                expanded = opposing_corridor(self.lanes, self.ego, self.route, self.legal)
                if gap > .75:
                    self.assertIsNone(expanded)
                else:
                    self.assertIsNotNone(expanded)
                    self.assertLessEqual(float(np.max(expanded.left-self.legal.left)), .75+1e-9)
                    np.testing.assert_array_equal(expanded.right, self.legal.right)

    def test_a_no_go_polygon_still_blocks_the_otherwise_clear_borrow(self):
        self.legal.zones = [np.array([[14., 1.2], [25., 1.2], [25., 2.6], [14., 2.6]])]
        plan, _ = self.plan()
        self.assertIsNone(plan)

    def test_source_padding_restoration_does_not_close_a_selected_static_pass(self):
        import gui
        import model_avoidance as ma
        from tunnel_runtime import swept_clearance
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)
        source = (20., -1.65, 1.1, 'fusion_obstacle', 'obstacle', 0.,
                  1.1, 1.1, 0., 0., 0., 7)
        points = np.array([[19.9, -1.65], [20., -1.65], [20.1, -1.65]])
        normalized, = ma.road_static_objects([source], self.ego, points)
        self.assertLess(normalized[7], source[7])
        plan, info = StaticLanePass().plan(self.lanes, self.ego, self.route,
            [source], [normalized], points, self.legal, None, 1., budget_s=1.)
        self.assertIsNotNone(plan, info)
        self.assertGreater(info['source_envelope_n'], 0)
        horizon = plan['clear_distance_m']
        self.assertGreaterEqual(gui.static_clear_distance(plan['path'], self.ego,
            [source], horizon, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(swept_clearance(plan['path'], points, horizon, .25,
            lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), horizon-.01)
        self.assertGreaterEqual(self.legal.clear_distance(plan['path'], horizon), horizon-.01)
        self.assertEqual(normalized[6:8], (.9, .9))

        # An actually occupied front corridor still invalidates the pass.
        blocked, _ = StaticLanePass().plan(self.lanes, self.ego, self.route,
            [source], [normalized], np.vstack((points, [[3.9, 0.]])),
            self.legal, None, 1., budget_s=1.)
        self.assertIsNone(blocked)

    def test_actual_contact_clock_survives_repeated_checks_and_one_clear_frame(self):
        policy = StaticLanePass()
        policy.legal, policy.origin = self.legal, dict(self.ego)
        outside = dict(self.ego, y=1., stamp=101.)
        policy.observe(outside, 10.)
        entered = policy.entered_at
        policy.observe(outside, 11.)
        policy.observe(dict(self.ego, stamp=102.), 11.1)
        policy.observe(dict(outside, stamp=103.), 11.2)
        self.assertEqual(policy.entered_at, entered)
        policy.observe(dict(self.ego, stamp=104.), 11.3)
        policy.observe(dict(self.ego, stamp=105.), 11.4)
        self.assertTrue(policy.finished)
        self.assertIsNone(policy.entered_at)

    def test_sensor_blackout_counts_from_the_last_distinct_inside_observation(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        # Re-reading the same old inside pose cannot renew the deadline.
        policy.observe(self.ego, 101.2)
        self.assertEqual(policy.last_inside_at, 100.)
        policy.observe(dict(self.ego, y=1., stamp=101.4), 101.4)
        self.assertEqual(policy.entered_at, 100.)
        self.assertGreater(103.-policy.entered_at, RETURN_BUDGET_S)

    def test_tunnel_and_highway_exclusion_covers_the_full_return_and_portal_buffer(self):
        for kind in ('tunnel', 'overpass'):
            self.lanes.rules = lambda: {'sections': [dict(type=kind, start_m=100., end_m=200.)]}
            for station, expected in ((19., True), (20., False), (80., False),
                                      (150., False), (220., False), (221., True)):
                with self.subTest(kind=kind, station=station):
                    self.assertEqual(normal_static_area(self.lanes, dict(self.ego, x=station)), expected)
            self.lanes.route_progress = lambda ego: 90.
            plan, proof = self.plan()
            self.assertIsNone(plan)
            self.assertIn('excluded', proof['reason'])
            self.lanes.route_progress = lambda ego: ego['x']
        self.assertFalse(normal_static_area(SimpleNamespace(), self.ego))

    def test_ideal_recorded_path_progress_and_actual_body_contact_finish_inside_budget(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        world = plan['path'][:, :2]
        stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(world, axis=0), axis=1))]
        now = 100.
        active = None
        durations = []
        for distance in np.arange(0., 40., .2):
            at = min(len(world)-2, max(1, np.searchsorted(stations, distance)))
            tangent = world[at+1]-world[at-1]
            xy = np.array([np.interp(distance, stations, world[:, k]) for k in (0, 1)])
            yaw = math.atan2(tangent[1], tangent[0])
            ego = dict(self.ego, x=xy[0], y=xy[1], heading_deg=math.degrees(yaw), stamp=now)
            check = policy.command_check(local_points(world, ego), ego, plan['speed_cap_mps'], now)
            self.assertTrue(check['ok'], (distance, check))
            outside = body_corners(*xy, yaw)[:, 1].max() > 1.75
            if outside and active is None:
                active = now
            elif not outside and active is not None:
                durations.append(now-active)
                active = None
            now += .2/(20./3.6*.75)
        self.assertIsNone(active)
        self.assertTrue(durations)
        self.assertLess(max(durations), 3.)

    def test_final_command_cannot_reuse_a_faster_plan_to_authorize_a_slow_excursion(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        check = policy.command_check(plan['path'], self.ego, .5, 100.)
        self.assertFalse(check['ok'])

    def test_expired_contact_budget_keeps_the_checked_return_moving_without_a_new_budget(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        path = plan['path'][:, :2]
        for i in range(1, len(path)-1):
            tangent = path[i+1]-path[i-1]
            yaw = math.atan2(tangent[1], tangent[0])
            if body_corners(*path[i], yaw, lateral_margin=.05)[:, 1].max() > 1.79:
                break
        else:
            self.fail('fixture contains no physical contact interval')
        ego = dict(self.ego, x=path[i, 0], y=path[i, 1], heading_deg=math.degrees(yaw), stamp=100.1)
        policy.observe(ego, 100.1)
        entered = policy.entered_at
        self.assertIsNotNone(entered)
        check = policy.command_check(local_points(path, ego), dict(ego, stamp=103.1),
                                      plan['speed_cap_mps'], 103.1)
        self.assertEqual(policy.entered_at, entered)
        self.assertTrue(check['ok'], check)
        self.assertTrue(check['late_return'], check)
        self.assertGreater(check['contact_s'], RETURN_BUDGET_S)
        resumed = policy.command_check(local_points(path, ego),
            dict(ego, speed_kmh=0., stamp=103.2), .2, 103.2)
        self.assertTrue(resumed['ok'], resumed)
        self.assertTrue(resumed['late_return'], resumed)
        person = (ego['x'], ego['y'], .6, 'person', 'pedestrian', 0., .6, .6, 0.)
        guarded = policy.assess(local_points(path, ego), ego, plan['speed_cap_mps'],
            policy.corridor(ego), 103.2, objects=[person])
        self.assertFalse(guarded['ok'], guarded)
        self.assertIn('person', guarded['reason'])
        # An overdue return still cannot leave the surveyed road.
        blocked = local_points(path, ego)
        blocked[(blocked[:, 0] > .1) & (blocked[:, 0] < 6.), 1] += 3.
        self.assertFalse(policy.command_check(blocked, dict(ego, stamp=103.2),
            plan['speed_cap_mps'], 103.2)['ok'])

    def test_an_active_pass_cannot_extend_its_committed_return_point(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        path = plan['path'][:, :2].copy()
        policy.entered_at = 100.
        # Push the same safe lateral excursion farther down the lane.
        path[:, 1] = np.interp(path[:, 0]-5., path[:, 0], path[:, 1])
        check = policy.assess(path, self.ego, plan['speed_cap_mps'], self.legal, 100.1)
        self.assertFalse(check['ok'], check)
        self.assertIn('committed return', check['reason'])

    def test_unentered_proposal_can_be_replaced_without_an_obsolete_return_deadline(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        original = policy.return_station_m
        self.assertIsNone(policy.entered_at)
        path = plan['path'][:, :2].copy()
        path[:, 1] = np.interp(path[:, 0]-5., path[:, 0], path[:, 1])
        check = policy.assess(path, self.ego, plan['speed_cap_mps'], self.legal, 100.1)
        self.assertTrue(check['ok'], check)
        self.assertGreater(check['return_station_m'], original+.25)
        self.assertLessEqual(check['contact_s'], RETURN_BUDGET_S)
        self.assertEqual(policy.return_station_m, original)  # assessment alone never commits

    def test_main_and_refresh_publications_recheck_the_actual_post_ramp_target(self):
        import json
        import time
        import gui
        import test_live_path_safety as fixtures
        harness = fixtures.LivePathSafetyTests()
        for refresh in (False, True):
            runtime = harness.attach(harness.runtime())
            runtime.infer()
            publisher = runtime.path_publisher
            calls = []
            def command_check(path, ego, target, now):
                calls.append(target)
                return dict(ok=target >= 5., reason='bounded return cannot use a slow command')
            runtime._static_lane_pass = SimpleNamespace(entered_at=1., finished=False, observe=lambda *args: None,
                corridor=lambda *args: None, assess=lambda *args, **kwargs: dict(ok=True),
                command_check=command_check)
            publisher._last_target_mps = 1.
            now = time.monotonic()
            publisher._last_target_time = now-.01
            with patch.object(gui, 'MODEL_AVOID', False), patch.object(gui, 'USE_LATTICE', False), \
                    patch('time.monotonic', return_value=now):
                if refresh:
                    publisher.recheck_active_path()
                else:
                    runtime.infer()
            with self.subTest(refresh=refresh):
                self.assertTrue(calls)
                payload = json.loads(publisher.waypoints_pub.items[-1].data)
                self.assertEqual(payload['target_speed_mps'], 0.)
                self.assertTrue(payload['decision']['emergency'])

    def test_reused_pass_keeps_its_return_and_still_rejects_a_new_raw_blocker(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        with patch('lattice_planner.plan', side_effect=AssertionError('safe reuse must not replan')):
            reused, check = policy.plan(self.lanes, self.ego, self.route, [self.box], [self.box],
                self.points, self.legal, plan, 100.1, .75, True)
        self.assertIsNotNone(reused, check)
        self.assertTrue(reused['reused'])
        np.testing.assert_allclose(reused['path'][-1], plan['path'][-1], atol=.25)
        blocked = np.vstack((self.points, [[3.9, 0.]]))
        rejected, _ = policy.plan(self.lanes, self.ego, self.route, [self.box], [self.box],
            blocked, self.legal, reused, 100.2, .75, True)
        self.assertIsNone(rejected)

    def test_an_unobserved_existing_encroachment_does_not_receive_a_fresh_budget(self):
        route = self.route-[0., 1.]
        legal = LaneCorridor(route, np.full(201, 1.75), np.full(201, -1.75),
                             body_shift_m=REAR_AXLE_SHIFT_M)
        path = self.route.copy()
        path[:, 1] = -np.clip(path[:, 0]/8., 0., 1.)
        proof = StaticLanePass().assess(path, self.ego, 5., legal, 100.)
        self.assertFalse(proof['ok'])

    def test_unused_future_excursion_retires_only_after_distinct_inside_observations(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        self.assertIsNone(policy.entered_at)
        policy.release_unused(self.ego, 100.)
        policy.release_unused(self.ego, 100.1)
        policy.release_unused(dict(self.ego, stamp=99.), 100.1)
        policy.release_unused(dict(self.ego, stamp=101., age_s=1.), 101.)
        self.assertFalse(policy.finished)
        policy.release_unused(dict(self.ego, stamp=101.), 101.)
        self.assertTrue(policy.finished)
        # An unrelated later bend is outside the old, finite straight corridor.
        policy.observe(dict(self.ego, x=90., y=5., heading_deg=30., stamp=102.), 102.)
        self.assertIsNone(policy.entered_at)

    def test_pending_and_actual_excursions_keep_their_proof_until_unused_or_returned(self):
        policy = StaticLanePass()
        plan, proof = self.plan(policy)
        self.assertIsNotNone(plan, proof)
        for stamp in (101., 102.):
            policy.release_unused(dict(self.ego, stamp=stamp), stamp, using_exception=True)
        self.assertFalse(policy.finished)
        outside = dict(self.ego, y=1., stamp=103.)
        policy.release_unused(outside, 103.)
        entered = policy.entered_at
        self.assertIsNotNone(entered)
        policy.release_unused(dict(outside, stamp=104.), 104.)
        self.assertFalse(policy.finished)
        self.assertEqual(policy.entered_at, entered)
        policy.release_unused(dict(self.ego, stamp=105.), 105.)
        self.assertFalse(policy.finished)
        policy.release_unused(dict(self.ego, stamp=106.), 106.)
        self.assertTrue(policy.finished)

    def test_recorded_unused_proposal_does_not_stop_the_later_s_bend(self):
        import json
        from pathlib import Path
        with np.load(Path(__file__).parent/'test_data/static_unused_proposal_20261016.npz') as data:
            policy = StaticLanePass()
            policy.legal = LaneCorridor(data['route'], data['left'], data['right'],
                                        body_shift_m=REAR_AXLE_SHIFT_M)
            policy.origin = json.loads(str(data['origin']))
            policy.last_stamp = policy.last_inside_at = policy.origin['stamp']
            for stamp, x, y, heading, speed, borrowed in data['poses']:
                ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed, stamp=stamp, age_s=.01)
                policy.release_unused(ego, stamp, using_exception=bool(borrowed))
            self.assertGreater(y, 88.)
            self.assertTrue(policy.finished)
            self.assertIsNone(policy.entered_at)


if __name__ == '__main__':
    unittest.main()
