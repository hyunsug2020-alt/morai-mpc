"""Pace batching preserves highway forecasts and admission decisions."""
import json
from dataclasses import replace
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from highway_planner import HighwayPlanner, local_points, timed_gap, world_points, _GapForecastContext
from highway_speed_plan import HighwaySpeedPlanner, SpeedTrajectory, _cached_command_preview
import highway_planner as planner_module
from highway_three_lane import HighwayLaneMap, ThreeLanePolicy
import planning.highway.policy as policy_module
from sim_time import to_sim_objects
from ego_body import EGO_CENTRE_M, EGO_FRONT_M


HERE = Path(__file__).parent


def car(x, y=0., speed=10., uid=1):
    return (x, y, 4.9, 'lidar_car', 'npc', speed, 4.9, 2.1, 0.,
            speed, 0., uid, 0., None, True, 0.)


def scalar_paces(model, path, ego, objects, paces, horizon, **kwargs):
    for pace in paces:
        yield pace, model.preview(path, ego, objects, float(pace), horizon, **kwargs)


def uncached_gap_checker(path, ego, positions, yaws, body_shift_m=EGO_CENTRE_M,
                         gap_function=None):
    stats = dict(queries=0, computations=0, hits=0)
    gap_function = timed_gap if gap_function is None else gap_function

    def check(objects, goal, trajectory, lane_change=True, ignore_rear_approach=False,
              footprint_margin_m=.6):
        stats['queries'] += 1
        stats['computations'] += 1
        return gap_function(path, ego, objects, goal, lane_change=lane_change,
            speed_trajectory=trajectory, object_positions=positions, object_yaws=yaws,
            ignore_rear_approach=ignore_rear_approach, footprint_margin_m=footprint_margin_m, body_shift_m=body_shift_m)

    check.stats = stats
    return check


def uncached_command_preview(model):
    stats = dict(queries=0, computations=0, hits=0)

    def check(trajectory):
        stats['queries'] += 1
        stats['computations'] += 1
        return model.command_preview(trajectory)

    check.stats = stats
    return check


class ForecastBatchTests(unittest.TestCase):
    def setUp(self):
        x = np.arange(0., 321., .5)
        self.path = np.column_stack((x, np.zeros(len(x))))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=10.)

    def assert_trajectory_equal(self, scalar, batch):
        for field in ('times', 'distances', 'speeds', 'accelerations'):
            np.testing.assert_allclose(getattr(scalar, field), getattr(batch, field),
                                       rtol=0., atol=1e-12)
        for field in ('goal_mps', 'ceiling_mps', 'braking_limit_mps2'):
            self.assertEqual(getattr(scalar, field), getattr(batch, field))
        self.assertAlmostEqual(scalar.reference_mps, batch.reference_mps, places=12)
        self.assertAlmostEqual(scalar.reference_time_s, batch.reference_time_s, places=12)

    def test_every_forecast_constraint_matches_scalar(self):
        goals = [0., 4., 12., 20., 33., float('nan'), float('inf'), -1.]
        for acceleration in (-6., -4.5, 0., 3.):
            for decel in (None, 5.):
                for constrained in (False, True):
                    with self.subTest(acceleration=acceleration, decel=decel,
                                      constrained=constrained):
                        model = HighwaySpeedPlanner(2.5, 4., 12.)
                        model.acceleration = acceleration
                        model.sensor_age_s, model.fusion_sensor_age_s = .2, .3
                        if constrained:
                            model.end_distance_m = 100.
                            model.end_buffer_m, model.end_speed_mps = 6., 8.
                        objects = [car(45., speed=9.), car(20., 3.5, 14., uid=2)]
                        kwargs = dict(decel_mps2=decel, ceiling_mps=30.,
                            static_clear_m=80. if constrained else float('inf'),
                            static_mover_mps=3. if constrained else 0.,
                            speed_profile=(np.array([0., 30., 70., 321.]),
                                           np.array([20., 12., 30., 30.])) if constrained else None)
                        batch = model.preview_many(self.path, self.ego, objects, goals, 8., **kwargs)
                        for goal, trajectory in zip(goals, batch):
                            scalar = model.preview(self.path, self.ego, objects, goal, 8., **kwargs)
                            self.assert_trajectory_equal(scalar, trajectory)

    def test_active_strong_braking_is_retained(self):
        model = HighwaySpeedPlanner(2.5, 4., 12.)
        model.acceleration = -6.
        trajectories = model.preview_many(self.path, self.ego, [], [20., 30.])
        for trajectory in trajectories:
            self.assertEqual(trajectory.accelerations[0], -6.)
            self.assertLess(trajectory.speeds[1], trajectory.speeds[0])
            self.assertLess(trajectory.accelerations[1], -2.5)

    def test_inactive_and_partly_active_leaders_preserve_scalar_motion(self):
        times = np.arange(0., 3.01, .1)
        n = len(times)
        cases = [
            (np.full(n, np.nan), np.full(n, np.nan), np.zeros(n, bool),
             np.full(n, np.nan), float('inf')),
            (np.full(n, -10.), np.full(n, np.nan), np.ones(n, bool),
             np.full(n, 2.5), float('inf')),
            (np.full(n, np.nan), np.full(n, 5.), np.ones(n, bool),
             np.full(n, 2.5), 0.),
            (30.+times*5., np.full(n, 5.), np.ones(n, bool),
             np.full(n, 2.5), float('inf')),
            (np.full(n, 30.), np.full(n, 5.), np.ones(n, bool),
             np.full(n, 2.5), .2),
        ]
        goals = [0., 8., 20., 33.]
        for index, leader in enumerate(cases):
            for acceleration in (-6., 3.):
                with self.subTest(leader=index, acceleration=acceleration):
                    model = HighwaySpeedPlanner(2.5, 4., 12.)
                    model.acceleration = acceleration
                    with patch.object(model, 'leaders', return_value=[leader]), \
                            np.errstate(all='ignore'):
                        batch = model.preview_many(self.path, self.ego, [], goals, 3.)
                        for goal, trajectory in zip(goals, batch):
                            scalar = model.preview(self.path, self.ego, [], goal, 3.)
                            self.assert_trajectory_equal(scalar, trajectory)

    def test_results_have_independent_writable_arrays(self):
        model = HighwaySpeedPlanner()
        trajectories = model.preview_many(self.path, self.ego, [], [20., 25.])
        for field in ('times', 'distances', 'speeds', 'accelerations'):
            original = getattr(trajectories[1], field).copy()
            getattr(trajectories[0], field)[:] = -100.
            np.testing.assert_array_equal(getattr(trajectories[1], field), original)

    def test_empty_batch_does_not_query_leaders(self):
        model = HighwaySpeedPlanner()
        with patch.object(model, 'leaders', side_effect=AssertionError('unused batch')):
            self.assertEqual(model.preview_many(self.path, self.ego, [], []), [])

    def test_chunk_order_and_early_exit_preserve_pace_grid(self):
        model = HighwaySpeedPlanner()
        paces = np.arange(25., 5., -.5)
        with patch.object(model, 'preview_many', wraps=model.preview_many) as batches, \
                patch.object(model, 'preview', wraps=model.preview) as scalars:
            iterator = policy_module._paced_previews(model, self.path, self.ego, [], paces, 7.)
            first = [next(iterator)[0] for _ in range(3)]
            np.testing.assert_array_equal(first, paces[:3])
            self.assertEqual(batches.call_count, 0)
            self.assertEqual(scalars.call_count, 3)
            following = [next(iterator)[0] for _ in range(6)]
            np.testing.assert_array_equal(following, paces[3:9])
            self.assertEqual(scalars.call_count, 4)
            self.assertEqual(batches.call_count, 1)
            np.testing.assert_array_equal(batches.call_args.args[3], paces[4:])

    def test_search_beyond_one_batch_keeps_every_pace(self):
        model = HighwaySpeedPlanner()
        paces = np.arange(100., 0., -.5)
        result = list(policy_module._paced_previews(
            model, self.path, self.ego, [], paces, .2))
        np.testing.assert_array_equal([pace for pace, _ in result], paces)

    def test_footprint_boundary_epsilon_matches_scalar(self):
        model = HighwaySpeedPlanner()
        ego = dict(self.ego, speed_kmh=0.)
        for epsilon in (-1e-9, 1e-9):
            objects = [car(EGO_FRONT_M+2.45+.2+epsilon, speed=0.)]
            batch = model.preview_many(self.path, ego, objects, [0., 0.], 3.)[0]
            scalar = model.preview(self.path, ego, objects, 0., 3.)
            actual = timed_gap(self.path, ego, objects, 0., speed_trajectory=batch,
                               lane_change=False, footprint_margin_m=.2)
            expected = timed_gap(self.path, ego, objects, 0., speed_trajectory=scalar,
                                 lane_change=False, footprint_margin_m=.2)
            self.assertEqual(actual, expected)
            self.assertEqual(actual is not None, epsilon < 0.)


class GapCacheTests(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 300.), np.zeros(300)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60.)
        self.model = HighwaySpeedPlanner()
        self.trajectory = self.model.preview(self.path, self.ego, [], 20., 3.)

    def checker(self):
        return policy_module._cached_gap_checker(self.path, self.ego,
            self.model.object_positions, self.model.object_yaws)

    def test_same_distance_forecast_reuses_risk_and_copies_annotations(self):
        risk = dict(when=1., object_id=1, rear_approach=True)
        with patch.object(policy_module, 'timed_gap', return_value=risk) as compute:
            check, objects = self.checker(), [car(10.)]
            first = check(objects, 20., self.trajectory)
            first['object_id'] = 999
            first['annotation'] = True
            other = SpeedTrajectory(self.trajectory.times.copy(), self.trajectory.distances.copy(),
                self.trajectory.speeds*2., self.trajectory.accelerations*2., 30.)
            self.assertEqual(check(objects, 30., other), risk)
            self.assertEqual(compute.call_count, 1)
            self.assertEqual(check.stats, dict(queries=2, computations=1, hits=1))

    def test_objects_margin_rear_policy_lane_change_and_arrays_have_separate_keys(self):
        with patch.object(policy_module, 'timed_gap', return_value=None) as compute:
            check, objects = self.checker(), [car(10.)]
            check(objects, 20., self.trajectory)
            check(objects, 20., self.trajectory, footprint_margin_m=.9)
            check(objects, 20., self.trajectory, ignore_rear_approach=True)
            check(objects, 20., self.trajectory, lane_change=False)
            check(list(objects), 20., self.trajectory)
            for field in ('times', 'distances'):
                changed = SpeedTrajectory(self.trajectory.times.copy(), self.trajectory.distances.copy(),
                    self.trajectory.speeds.copy(), self.trajectory.accelerations.copy(), 20.)
                getattr(changed, field)[1] += 1e-9
                check(objects, 20., changed)
            self.assertEqual(compute.call_count, 7)
            self.assertEqual(check.stats['hits'], 0)

    def test_cache_is_local_to_one_evaluation(self):
        with patch.object(policy_module, 'timed_gap', return_value=None) as compute:
            objects = [car(10.)]
            self.checker()(objects, 20., self.trajectory)
            self.checker()(objects, 20., self.trajectory)
            self.assertEqual(compute.call_count, 2)


class PreparedGapTests(unittest.TestCase):
    def setUp(self):
        x = np.arange(0., 301., .5)
        self.path = np.column_stack((x, 3.5*np.clip(x/60., 0., 1.)**2))
        self.ego = dict(x=25., y=-40., heading_deg=-93.1, speed_kmh=60.)
        self.model = HighwaySpeedPlanner()

    def assert_checks_equal(self, objects, **kwargs):
        context = _GapForecastContext(self.model.object_positions, self.model.object_yaws)
        for goal in (20., 8., 0.):
            trajectory = self.model.preview(self.path, self.ego, objects, goal, 3.)
            options = dict(speed_trajectory=trajectory, object_positions=self.model.object_positions,
                           object_yaws=self.model.object_yaws, **kwargs)
            self.assertEqual(timed_gap(self.path, self.ego, objects, goal, **options),
                timed_gap(self.path, self.ego, objects, goal, _prepared=context, **options))
        return context

    def test_native_rear_and_crossing_bodies_keep_all_safety_gates(self):
        for heading in (0., 37., -93.1, 180.):
            self.ego['heading_deg'] = heading
            lane = world_points(self.path, self.ego)
            stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(lane, axis=0), axis=1))]
            for along, lateral in ((-12., 0.), (-8., 3.5), (9., 3.5), (18., 0.)):
                xy = world_points(np.array([[along, lateral]]), self.ego)[0]
                h = np.deg2rad(heading)
                obj = tuple([*xy, 4.9, 'lidar_car', 'npc', 23., 4.9, 2.1, heading,
                             23.*np.cos(h), 23.*np.sin(h), 1, 0., None, True, 0.])
                self.model.motion_paths = {1: (lane, stations, along, 23., lateral, -.3)}
                for margin in (0., .2, .6, .9):
                    for ignore_rear in (False, True):
                        with self.subTest(heading=heading, along=along, lateral=lateral,
                                          margin=margin, ignore_rear=ignore_rear):
                            context = self.assert_checks_equal([obj], lane_change=True,
                                ignore_rear_approach=ignore_rear, footprint_margin_m=margin)
                            self.assertEqual(len(context.forecasts), 1)

    def test_lazy_model_snapshot_owns_motion_arrays_and_stable_yaw(self):
        context = _GapForecastContext(self.model.object_positions, self.model.object_yaws)
        lane = world_points(self.path, self.ego)
        stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(lane, axis=0), axis=1))]
        self.model.motion_paths[1] = (lane, stations, 15., 12., 0., 0.)
        self.model.stable_yaws[1] = self.ego['heading_deg']
        xy = world_points(np.array([[15., 0.]]), self.ego)[0]
        obj = car(*xy, uid=1)
        trajectory = self.model.preview(self.path, self.ego, [], 20., 3.)
        options = dict(speed_trajectory=trajectory, object_positions=self.model.object_positions,
                       object_yaws=self.model.object_yaws)
        expected = timed_gap(self.path, self.ego, [obj], 20., **options)
        actual = timed_gap(self.path, self.ego, [obj], 20., _prepared=context, **options)
        self.assertEqual(actual, expected)
        lane[:] += 100.
        stations[:] += 100.
        self.model.stable_yaws[1] += 90.
        context.forecasts.clear()
        self.assertEqual(timed_gap(self.path, self.ego, [obj], 20., _prepared=context, **options), expected)

    def test_invalid_inputs_keep_validation_order_and_risk(self):
        trajectory = self.model.preview(self.path, self.ego, [], 20., 3.)
        cases = [dict(path=[[float('nan'), 0.], [1., 0.]], ego={}),
                 dict(ego=dict(self.ego, x=float('nan')), body_shift_m=float('nan')),
                 dict(body_shift_m=float('nan')),
                 dict(speed_trajectory=replace(trajectory, times=trajectory.times[::-1])),
                 dict(path=np.array([[0., 0.], [1., 0.]])),
                 dict(object_positions=lambda obj, times: np.full((len(times), 3), np.nan))]
        for changes in cases:
            path, ego = changes.get('path', self.path), changes.get('ego', self.ego)
            options = dict(speed_trajectory=trajectory, object_positions=self.model.object_positions,
                           object_yaws=self.model.object_yaws)
            options.update({key: value for key, value in changes.items() if key not in ('path', 'ego')})
            with self.subTest(changes=changes.keys()):
                context = _GapForecastContext(options['object_positions'], options['object_yaws'])
                self.assertEqual(timed_gap(path, ego, [car(35.)], 20., **options),
                    timed_gap(path, ego, [car(35.)], 20., _prepared=context, **options))

    def test_stationary_actual_heading_keeps_contact_boundary(self):
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        for epsilon in (-1e-9, 1e-9):
            obj = car(EGO_FRONT_M+2.45+.2+epsilon, speed=0.)
            context = self.assert_checks_equal([obj], lane_change=False, footprint_margin_m=.2)
            trajectory = self.model.preview(self.path, self.ego, [], 0., 3.)
            risk = timed_gap(self.path, self.ego, [obj], 0., speed_trajectory=trajectory,
                             lane_change=False, footprint_margin_m=.2, _prepared=context)
            self.assertEqual(risk is not None, epsilon < 0.)


class RecordedYieldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from gui import HdMapLanes, smooth_control_route
        cls.fixture = json.loads((HERE/'test_data/highway_yield_latency_20261012.json').read_text())
        cls.route = np.array([[p['x'], p['y']] for p in
                             json.loads((HERE/'global_path.json').read_text())['waypoints']])
        cls.control = smooth_control_route(cls.route)
        cls.lanes = HdMapLanes(HERE/'link_set.json')
        cls.lanes.set_route(cls.route)

    def run_candidates(self):
        fixture, ego = self.fixture, self.fixture['ego']
        index = int(np.argmin(np.linalg.norm(self.route-[ego['x'], ego['y']], axis=1)))
        native = HighwayLaneMap(self.lanes, fixture['config'])
        options = native.candidates(ego, local_points(self.control[index:index+640], ego))
        objects = to_sim_objects(fixture['objects'], fixture['rtf'])
        planner = HighwayPlanner(fixture['config'], self.route)
        planner.longitudinal.time_scale = fixture['rtf']
        planner.longitudinal.sensor_age_s = fixture['sensor_age_s']
        planner.longitudinal.begin(ego)
        policy = ThreeLanePolicy(fixture['config'])
        policy.update_motion_paths(planner, native.motion_candidates(ego, objects), ego, objects)
        output = []
        for logged in fixture['checks']:
            if 'transition_m' not in logged:
                continue
            destinations = [next(l for l in options if l.link_id == link)
                            for link in logged['lanes']]
            transition = logged['transition_m']
            base = (transition-(len(destinations)-1)*max(3., ego['speed_kmh']/7.2))/len(destinations)
            path = policy.sequence_path(destinations, base, ego['speed_kmh']/3.6)
            result, check = policy.evaluate(planner, path, ego, objects,
                lambda p, h: logged['map_clear_m'], lambda p, h: logged['static_clear_m'],
                changing=True, destinations=destinations, transition=transition,
                goal_limit=logged['free_goal_kmh']/3.6,
                admission=logged.get('gap_risk', {}).get('footprint_margin_m', .9) > .6)
            output.append((result, check))
        return output

    def test_recorded_candidates_match_scalar_uncached_decisions(self):
        with patch.object(policy_module, '_paced_previews', scalar_paces), \
                patch.object(policy_module, '_cached_gap_checker', uncached_gap_checker):
            baseline = self.run_candidates()
        batched = self.run_candidates()
        queries, computations = 0, 0
        for (old_result, old), (new_result, new) in zip(baseline, batched):
            self.assertEqual(old_result is None, new_result is None)
            old_stats, new_stats = old.pop('traffic_gap_queries'), new.pop('traffic_gap_queries')
            self.assertEqual(old_stats['queries'], new_stats['queries'])
            queries += new_stats['queries']
            computations += new_stats['computations']
            self.assertEqual(old, new)
            if old_result is not None:
                self.assertEqual(old_result['goal'], new_result['goal'])
                np.testing.assert_allclose(old_result['trajectory'].distances,
                    new_result['trajectory'].distances, rtol=0., atol=1e-12)
        # Both implementations must visit the same full candidate/pace grid.
        # Measured-pose joins can change the number of rejected pace probes.
        self.assertGreaterEqual(queries, 400)
        self.assertLess(computations, queries)


class CommandCacheTests(unittest.TestCase):
    def setUp(self):
        self.model = HighwaySpeedPlanner()
        path = np.column_stack((np.arange(300.), np.zeros(300)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60.)
        self.trajectory = self.model.preview(path, ego, [], 0., 3., decel_mps2=5.)

    def test_only_execution_inputs_determine_reuse_and_outputs_are_independent(self):
        with patch.object(self.model, 'command_preview', wraps=self.model.command_preview) as compute:
            check = _cached_command_preview(self.model)
            expected = check(self.trajectory)
            changed = replace(self.trajectory, distances=self.trajectory.distances+100.,
                              speeds=self.trajectory.speeds.copy())
            changed.speeds[1:] += 10.
            changed.reference_override_s = 0.
            reference = check(changed)
            self.assertNotEqual(reference.goal_mps, expected.goal_mps)
            check(changed)
            self.assertEqual(compute.call_count, 2)
            reference.distances[:] = -100.
            reference.times[:] = -100.
            restored = check(changed)
            self.assertGreaterEqual(restored.distances.min(), 0.)
            np.testing.assert_array_equal(restored.times, changed.times)
            self.assertEqual(compute.call_count, 2)

    def test_initial_motion_time_grid_and_metadata_have_separate_keys(self):
        with patch.object(self.model, 'command_preview', wraps=self.model.command_preview) as compute:
            check = _cached_command_preview(self.model)
            check(self.trajectory)
            for field in ('speeds', 'accelerations', 'times'):
                changed = replace(self.trajectory,
                                  **{field: getattr(self.trajectory, field).copy()})
                getattr(changed, field)[0 if field != 'times' else 1] += .01
                check(changed)
            check(replace(self.trajectory, ceiling_mps=24.))
            check(replace(self.trajectory, braking_limit_mps2=7.))
            self.assertEqual(compute.call_count, 6)
            self.assertEqual(check.stats['hits'], 0)
            _cached_command_preview(self.model)(self.trajectory)
            self.assertEqual(compute.call_count, 7)

    def test_input_mutation_cannot_change_a_cached_snapshot(self):
        pristine = replace(self.trajectory, times=self.trajectory.times.copy())
        check = _cached_command_preview(self.model)
        expected = check(self.trajectory)
        self.trajectory.times[1] += 10.
        restored = check(pristine)
        np.testing.assert_array_equal(restored.times, expected.times)
        np.testing.assert_array_equal(restored.distances, expected.distances)
        self.assertEqual(check.stats, dict(queries=2, computations=1, hits=1))


class SpeedDecisionCacheTests(unittest.TestCase):
    def run_decision(self, acceleration=0., objects=(), static=float('inf'), changing=False,
                     goal=30., held=None, speed=80.):
        path = np.column_stack((np.arange(400.), np.zeros(400)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=speed, stamp=100.)
        planner = HighwayPlanner(dict(from_idx=0, to_idx=390, kmh=120.,
            lane_change_policy='free_three_lane'), path)
        planner.longitudinal.acceleration = acceleration
        if changing:
            planner.saved_world = path.copy()
        if held is not None:
            planner._speed_hold_goal, planner._speed_hold_until = held, 100.5
        return planner.plan_speed(path, ego, objects, goal, static_clear_m=static)

    def test_cached_batch_decisions_match_scalar_uncached_including_all_failures(self):
        cases = [dict(), dict(acceleration=-6.), dict(acceleration=3., static=30.75),
                 dict(objects=[car(60.2, speed=8./3.6)]),
                 dict(objects=[car(45., speed=8./3.6)]),
                 dict(objects=[car(6., speed=0.)]), dict(static=3.75),
                 dict(changing=True, objects=[car(-30., speed=30.)]),
                 dict(objects=[car(-8., speed=25.), car(60.2, speed=8./3.6)]),
                 dict(objects=[car(-12., speed=27.)], held=15.),
                 dict(objects=[car(30., 3.5, speed=10.)], changing=True)]
        queries = computations = 0
        for case in cases:
            with self.subTest(case=case):
                with patch.object(policy_module, '_paced_previews', scalar_paces), \
                        patch.object(policy_module, '_cached_gap_checker', uncached_gap_checker), \
                        patch.object(planner_module, '_cached_command_preview', uncached_command_preview):
                    old = self.run_decision(**case)
                new = self.run_decision(**case)
                for field in ('traffic_gap_queries', 'command_preview_queries'):
                    old_stats = old['diagnostics'].pop(field)
                    new_stats = new['diagnostics'].pop(field)
                    self.assertEqual(old_stats['queries'], new_stats['queries'])
                    queries += new_stats['queries']
                    computations += new_stats['computations']
                self.assertEqual(old, new)
        self.assertLess(computations, queries)


if __name__ == '__main__':
    unittest.main()
