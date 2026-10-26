"""Highway MOBIL selection, follower demand and admission regressions."""
import math
import unittest
from unittest.mock import patch

import numpy as np

import test_highway_free_lane as free_lane
import test_highway_three_lane as three_lane
from ego_body import EGO_CENTRE_M, EGO_HALF_LENGTH_M
from highway_planner import HighwayPlanner
from highway_speed_plan import HighwaySpeedPlanner, SpeedTrajectory
from planning.highway.mobil import MobilEvaluator, following_acceleration, incentive


class MobilModelTests(unittest.TestCase):
    def test_new_follower_disadvantage_can_remove_a_small_ego_advantage(self):
        before = following_acceleration(25., 120./3.6)
        after = following_acceleration(25., 120./3.6, leader=25., gap=18.75)
        self.assertEqual(before, 3.)
        self.assertEqual(after, 0.)
        selfish = incentive(0., .35, 0., 0., before, after, 0.)
        courteous = incentive(0., .35, 0., 0., before, after, .1)
        self.assertGreater(selfish, .2)
        self.assertLess(courteous, .2)

    def test_releasing_old_follower_can_make_a_change_worthwhile(self):
        before = following_acceleration(25., 120./3.6, leader=20., gap=18.75)
        after = following_acceleration(25., 120./3.6, leader=25., gap=60.)
        self.assertLess(before, -2.5)
        self.assertGreater(incentive(0., .1, before, after, 0., 0., .1), .2)

    def test_braking_request_is_not_hidden_by_saturation(self):
        value = following_acceleration(25., 120./3.6, leader=10., gap=12.)
        self.assertLess(value, -2.5)
        self.assertLess(value, -6.)

    def test_safe_braking_boundary(self):
        # Equal-speed rear: target falls by .45 * headway deficit. Dividing
        # by the reference response .5 gives the exact -2.5 m/s² boundary.
        gap = 20.*.75-2.5*.5/.45
        self.assertAlmostEqual(following_acceleration(20., 30., 20., gap), -2.5)
        self.assertLess(following_acceleration(20., 30., 20., gap-1e-4), -2.5)
        self.assertGreater(following_acceleration(20., 30., 20., gap+1e-4), -2.5)

    def test_unknown_or_overlapping_gap_does_not_supply_an_incentive(self):
        self.assertIsNone(following_acceleration(float('nan'), 30.))
        self.assertIsNone(following_acceleration(20., 30., 20., 0.))
        self.assertIsNone(following_acceleration(20., 30., 20., -1.))

    def test_invalid_model_settings_fail(self):
        f = free_lane.FreeLaneChoice(); f.setUp()
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=10.)
        for key, value in [('mobil_politeness', -.1), ('mobil_politeness', float('nan')),
                           ('mobil_threshold_mps2', 3.), ('mobil_safe_decel_mps2', 0.),
                           ('mobil_safe_decel_mps2', 3.)]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                MobilEvaluator(ego, [], f.lanes, f.lanes, HighwaySpeedPlanner(), {key: value})

    def test_inferred_speed_is_not_promoted_to_measurement(self):
        f = free_lane.FreeLaneChoice(); f.setUp()
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=10.)
        for source in ['tracked', 'assumed']:
            obj = tuple(list(free_lane.car(30., 3.5, 20.))+[source])
            context = MobilEvaluator(ego, [obj], f.lanes, f.lanes, HighwaySpeedPlanner(), {})
            self.assertIsNone(context.snapshot(f.lanes[1], 0.)[0]['speed'])

    def test_retained_alignment_lineage_cannot_be_bypassed_by_a_young_age(self):
        from highway_sensor_alignment import align_highway_predictions
        from copy import deepcopy
        for age in (.2, .95):
            source = list(free_lane.car(30., 3.5, 20., 7)); source[15] = age
            aligned, _ = align_highway_predictions([tuple(source)], 100., 100.2)
            self.assertTrue(MobilEvaluator.measured(aligned[0]))
            if age == .95:
                self.assertGreater(aligned[0][15], 1.)
        # Young age remains insufficient when explicit alignment evidence is
        # invalid; it cannot renew, rewind or hide excessive transport delay.
        source = list(free_lane.car(30., 3.5, 20., 7)); source[15] = .2
        aligned, _ = align_highway_predictions([tuple(source)], 100., 100.2)
        for field, value in [('target_stamp', 101.), ('source_age_s', .1),
                             ('target_age_s', .1), ('source_stamp', 0.)]:
            damaged = deepcopy(list(aligned[0]))
            damaged[18]['highway_prediction_alignment'][field] = value
            with self.subTest(field=field):
                self.assertFalse(MobilEvaluator.measured(tuple(damaged)))
        rewound, _ = align_highway_predictions(aligned, 100., 100.1)
        self.assertFalse(MobilEvaluator.measured(rewound[0]))
        renewed = list(aligned[0]); renewed[15] = 0.
        self.assertFalse(MobilEvaluator.measured(tuple(renewed)))

    def test_duplicate_observation_does_not_double_count_a_follower(self):
        f = free_lane.FreeLaneChoice(); f.setUp()
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=10.)
        car = free_lane.car(30., 3.5, 20., uid=7)
        duplicate = list(car); duplicate[3], duplicate[11] = 'fusion_npc', 42
        context = MobilEvaluator(ego, [car, car, tuple(duplicate)], f.lanes, f.lanes,
                                 HighwaySpeedPlanner(), {})
        self.assertEqual(len(context.objects), 1)

    def test_rear_axle_is_shifted_to_the_real_body_centre(self):
        from highway_speed_plan import SpeedTrajectory
        path = np.array([[0., 0.], [20., 0.]])
        trajectory = SpeedTrajectory(np.array([0., 1.]), np.array([0., 10.]),
            np.array([10., 10.]), np.zeros(2), 10.)
        points, speeds, _ = MobilEvaluator.sample_ego(dict(path=path, trajectory=trajectory), [0., 1.])
        np.testing.assert_allclose(points[:, 0], [EGO_CENTRE_M, 10.+EGO_CENTRE_M])
        self.assertAlmostEqual(points[0, 0]-EGO_HALF_LENGTH_M, -.79)
        self.assertAlmostEqual(points[0, 0]+EGO_HALF_LENGTH_M, 3.845)


class MobilForecastTests(unittest.TestCase):
    def setUp(self):
        self.f = free_lane.FreeLaneChoice(); self.f.setUp()
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=72., stamp=10.)
        self.model = HighwaySpeedPlanner()

    @staticmethod
    def forecast(speed, lateral=0., path_length=400.):
        times = np.arange(0., 6.01, .1)
        speeds = np.asarray(speed(times) if callable(speed) else np.full(len(times), speed))
        distances = np.r_[0., np.cumsum((speeds[:-1]+speeds[1:])*.05)]
        trajectory = SpeedTrajectory(times, distances, speeds, np.gradient(speeds, times), 30.)
        x = np.arange(0., path_length+.01, .5)
        z = np.clip(x/100., 0., 1.)
        path = np.column_stack((x, lateral*(10.*z**3-15.*z**4+6.*z**5)))
        return dict(path=path, trajectory=trajectory, ceiling_mps=30.)

    def context(self, objects=()):
        return MobilEvaluator(self.ego, objects, self.f.lanes, self.f.lanes, self.model, {})

    def test_reaching_the_same_cruise_sooner_retains_ego_advantage(self):
        changed = self.forecast(lambda t: 20.+3.*np.clip(t, 0., 2.), 3.5)
        stayed = self.forecast(lambda t: 20.+3.*np.clip(t-3., 0., 2.))
        self.assertEqual(changed['trajectory'].speeds[-1], stayed['trajectory'].speeds[-1])
        length = changed['trajectory'].distances[-1]
        assessment = self.context().assess(self.f.lanes[0], [self.f.lanes[1]], changed, stayed, length)
        expected = 2.*(length-stayed['trajectory'].distances[-1])/6.**2
        self.assertTrue(assessment['available'])
        self.assertAlmostEqual(assessment['incentive_mps2'], expected)
        self.assertGreater(expected, .2)

    def test_released_old_follower_is_integrated_over_actual_occupancy(self):
        changed, stayed = self.forecast(20., 3.5), self.forecast(20.)
        assessment = self.context([free_lane.car(-15., 0., 20., 7)]).assess(
            self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
        self.assertTrue(assessment['available'])
        self.assertGreater(assessment['old_follower_delta_mps2'], 0.)
        self.assertAlmostEqual(assessment['new_follower_delta_mps2'], 0.)
        self.assertAlmostEqual(assessment['incentive_mps2'], .1*assessment['old_follower_delta_mps2'])

    def test_known_unsafe_prefix_is_rejected_before_incomplete_completion(self):
        changed, stayed = self.forecast(20., 3.5), self.forecast(20.)
        assessment = self.context([free_lane.car(-10., 3.5, 20., 7)]).assess(
            self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 200.)
        self.assertFalse(assessment['safe'])
        self.assertEqual(assessment['reason'], 'MOBIL target follower braking exceeds limit')
        self.assertLess(assessment['follower_checks'][0]['acceleration_mps2'], -2.5)

    def test_intermediate_lane_rear_is_checked_for_a_two_leg_candidate(self):
        changed, stayed = self.forecast(20., 7.), self.forecast(20.)
        assessment = self.context([free_lane.car(-10., 3.5, 20., 7)]).assess(
            self.f.lanes[0], self.f.lanes[1:], changed, stayed, 55.)
        self.assertFalse(assessment['safe'])
        self.assertEqual(assessment['follower_checks'][0]['lane'], '1')

    def test_saved_next_leg_safety_extends_past_the_original_length(self):
        from highway_three_lane import ThreeLanePolicy, HighwayLane
        from planning.highway.trajectory_geometry import normalize_path
        # First-leg completion is detected by lateral alignment before the
        # quintic waypoint. The saved suffix still contains its end and pause.
        original = ThreeLanePolicy(self.f.config).sequence_path(self.f.lanes[1:], 90., 20.)
        at = np.searchsorted(original[:, 0], 68.)
        origin = original[at].copy()
        saved = original[at:]-origin
        lanes = [HighwayLane(l.link_id, l.rank, l.centre-origin, l.chain, 700.) for l in self.f.lanes]
        changed = self.forecast(20.); changed['path'] = saved
        stayed = self.forecast(20.)
        rear = free_lane.car(-19.59, 7.-origin[1], 20.5, 7)
        context = MobilEvaluator(self.ego, [rear], lanes, lanes, self.model, {})
        # The original length ends at t=4.5. The actual next leg ends after
        # t=6.1; geometry there must still enforce the rear braking bound.
        short = dict(changed, trajectory=changed['trajectory'].prefix(4.5))
        earlier = context.assess(lanes[1], [lanes[2]], short, stayed, 90.)
        full = context.assess(lanes[1], [lanes[2]], changed, stayed, 90.)
        self.assertTrue(earlier['safe'])
        self.assertFalse(full['safe'])
        self.assertGreater(full['safety_prefix_s'], 4.5)
        self.assertLess(full['follower_checks'][0]['acceleration_mps2'], -2.5)
        path, stations = normalize_path(saved)
        self.assertGreater(stations[np.searchsorted(path[:, 0], 190.-origin[0])], 90.)

    def test_closing_response_age_belongs_to_the_leader_observation(self):
        context = self.context()
        rear = dict(s=0., speed=25., half=2.45, age=.8)
        lead = dict(s=45., speed=20., half=2.45, age=.2)
        value = context.acceleration(rear, lead, 33.)
        expected = following_acceleration(25., 33., 20., 40.1, age=.2)
        self.assertAlmostEqual(value, expected)
        self.assertAlmostEqual(context.acceleration(dict(rear, age=0.), lead, 33.), value)

    def test_short_changed_path_does_not_invent_a_late_stationary_ego(self):
        changed, stayed = self.forecast(20., 3.5), self.forecast(20.)
        changed['path'] = np.array([[0., 0.], [10., 2.], [20., 3.5]])
        # Rear is more than 100 m away throughout the real path prefix. If
        # the final t=6 sample is clamped to the path end, it invents a severe
        # insertion demand next to the later rear position.
        context = self.context([free_lane.car(-130., 3.5, 24., 7)])
        with patch.object(context, 'prepare_future', wraps=context.prepare_future) as future:
            assessment = context.assess(self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
        self.assertTrue(assessment['safe'])
        self.assertFalse(assessment['available'])
        self.assertLess(assessment['safety_prefix_s'], 1.1)
        self.assertEqual(assessment['reason'], 'MOBIL lane completion outside prediction')
        self.assertLessEqual(max(context.future), assessment['safety_prefix_s']+1e-8)

    def test_stay_path_endpoint_cannot_clamp_a_moving_forecast(self):
        changed, stayed = self.forecast(20., 3.5), self.forecast(20., path_length=20.)
        assessment = self.context().assess(self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
        self.assertFalse(assessment['available'])
        self.assertEqual(assessment['reason'], 'MOBIL stay prediction unavailable')

    def test_measured_semantic_duplicate_wins_over_inferred_lidar(self):
        lidar = list(free_lane.car(30., 3.5, 20., 7)); lidar.append('assumed')
        semantic = list(free_lane.car(30., 3.5, 20., 42)); semantic[3] = 'fusion_npc'
        context = self.context([tuple(lidar), tuple(semantic)])
        self.assertEqual(len(context.objects), 1)
        self.assertEqual(context.objects[0][11], 42)
        self.assertAlmostEqual(context.snapshot(self.f.lanes[1], 0.)[0]['speed'], 20.)

    def test_curve_forecast_speed_is_invariant_under_world_rotation(self):
        radius = 120.
        distance = np.arange(0., 401., .25)
        local = np.column_stack((radius*np.sin(distance/radius), radius*(1.-np.cos(distance/radius))))
        from highway_three_lane import HighwayLane
        lane = HighwayLane('curve', 0, local, ('curve',), 400.)
        for heading in (0., 45., 135.):
            angle = math.radians(heading)
            rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
            origin = np.array([10., -5.])
            world = local @ rotation.T+origin
            obj = list(free_lane.car(*origin, 20., 7))
            obj[8] = heading; obj[9:11] = [20.*math.cos(angle), 20.*math.sin(angle)]
            self.model.motion_paths = {7: (world, distance, 0., 20., 0., 0.)}
            context = MobilEvaluator(dict(self.ego, x=10., y=-5., heading_deg=heading),
                [tuple(obj)], [lane], [lane], self.model, {})
            with self.subTest(heading=heading):
                # After six seconds the lane tangent has turned about 57°.
                state = context.snapshot(lane, 6.)[0]
                self.assertIsNotNone(state['speed'])
                self.assertAlmostEqual(state['speed'], 20., places=3)

    def test_t0_pose_uses_measured_axle_and_heading_with_residual_vertex(self):
        result = self.forecast(20.)
        result['path'] = np.array([[-.1, .3], [2., 1.], [20., 2.], [160., 2.]])
        points, _, direction = MobilEvaluator.sample_ego(result, [0.])
        np.testing.assert_allclose(direction, [[1., 0.]])
        np.testing.assert_allclose(points, [[EGO_CENTRE_M, 0.]])

    def test_ego_follower_demand_uses_lane_projected_speed(self):
        angle = math.radians(10.)
        direction = np.array([[math.cos(angle), math.sin(angle)]])
        context = self.context()
        states, _ = context.ego_states(self.f.lanes[1], np.array([[20., 3.5]]),
                                     np.array([30.]), direction)
        ego = states[0]
        self.assertAlmostEqual(ego['speed'], 30.*math.cos(angle))
        # Same projected body gap, opposite b_safe decisions if the lateral
        # part of ego velocity is incorrectly presented as forward motion.
        rear = dict(s=ego['s']-ego['half']-2.45-31.*.75, speed=31., half=2.45, age=0.)
        self.assertLess(context.acceleration(rear, ego, 33.), -2.5)
        self.assertGreater(context.acceleration(rear, dict(ego, speed=30.), 33.), -2.5)

    def test_snapshots_share_vectorized_motion_forecasts_across_lanes(self):
        objects = [free_lane.car(20., 0., 20., 7), free_lane.car(-30., 3.5, 22., 8)]
        times = np.arange(0., 6.01, .1)
        context = self.context(objects)
        with patch.object(self.model, 'object_positions', wraps=self.model.object_positions) as positions:
            for lane in self.f.lanes:
                context.prepare_snapshots(lane, times)
                for t in times:
                    context.snapshot(lane, t)
            self.assertEqual(positions.call_count, len(objects))

    def test_only_a_valid_estimated_front_allows_forecast_progress_fallback(self):
        changed, stayed = self.forecast(26., 3.5), self.forecast(20.)
        for source in ('tracked', 'assumed'):
            front = tuple(list(free_lane.car(100., 0., 20., 7))+[source])
            assessment = self.context([front]).assess(
                self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
            self.assertFalse(assessment['available'])
            self.assertTrue(assessment['safe'])
            self.assertTrue(assessment['forecast_progress_fallback'])
            self.assertEqual(assessment['unavailable_reasons'], ['estimated_leader'])
            self.assertEqual(assessment['unavailable_evidence'][0]['object_id'], '7')
            self.assertIsNone(self.context([front]).snapshot(self.f.lanes[0], 0.)[0]['speed'])

    def test_invalid_estimate_cannot_use_forecast_progress_fallback(self):
        changed, stayed = self.forecast(26., 3.5), self.forecast(20.)
        original = list(free_lane.car(100., 0., 20., 7))+['assumed']
        for index, value, expected in [(15, 1.001, 'invalid_age'), (15, -.1, 'invalid_age'),
                (15, float('nan'), 'invalid_age'), (6, -4.9, 'invalid_geometry'),
                (14, False, 'unobserved_motion'), (14, float('nan'), 'unobserved_motion'),
                (14, 'True', 'unobserved_motion')]:
            obj = original.copy(); obj[index] = value
            with self.subTest(index=index, value=value):
                assessment = self.context([tuple(obj)]).assess(
                    self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
                self.assertFalse(assessment['forecast_progress_fallback'])
                self.assertIn(expected, assessment['unavailable_reasons'])

    def test_alignment_lineage_and_lane_direction_override_estimated_front(self):
        from highway_sensor_alignment import align_highway_predictions
        changed, stayed = self.forecast(26., 3.5), self.forecast(20.)
        source = list(free_lane.car(100., 0., 20., 7)); source[15] = .2
        aligned, _ = align_highway_predictions([tuple(source)], 100., 100.2)
        inferred = list(aligned[0]); inferred[16] = 'assumed'
        assessment = self.context([tuple(inferred)]).assess(
            self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
        self.assertIn('invalid_alignment', assessment['unavailable_reasons'])
        self.assertFalse(assessment['forecast_progress_fallback'])
        backwards = list(free_lane.car(200., 0., 20., 7))+['assumed']
        backwards[9] = -20.
        assessment = self.context([tuple(backwards)]).assess(
            self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
        self.assertIn('unaligned_motion', assessment['unavailable_reasons'])
        self.assertFalse(assessment['forecast_progress_fallback'])

    def test_every_unknown_reason_is_collected_across_entered_lanes(self):
        changed, stayed = self.forecast(26., 3.5), self.forecast(20.)
        estimated = tuple(list(free_lane.car(100., 0., 20., 7))+['assumed'])
        unobserved = free_lane.car(100., 3.5, 20., 8, valid=False)
        assessment = self.context([estimated, unobserved]).assess(
            self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
        self.assertEqual(set(assessment['unavailable_reasons']),
                         {'estimated_leader', 'unobserved_motion'})
        self.assertFalse(assessment['forecast_progress_fallback'])

    def test_unknown_old_follower_balance_prevents_estimated_front_fallback(self):
        changed, stayed = self.forecast(26., 3.5), self.forecast(20.)
        estimated = tuple(list(free_lane.car(100., 0., 20., 7))+['assumed'])
        assessment = self.context([estimated, free_lane.car(-30., 0., 20., 8)]).assess(
            self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
        self.assertTrue(assessment['safe'])
        self.assertIn('follower_balance', assessment['unavailable_reasons'])
        self.assertFalse(assessment['forecast_progress_fallback'])

    def test_unknown_target_rear_prevents_estimated_front_fallback(self):
        changed, stayed = self.forecast(26., 3.5), self.forecast(20.)
        estimated = tuple(list(free_lane.car(100., 0., 20., 7))+['assumed'])
        rear = tuple(list(free_lane.car(-30., 3.5, 20., 8))+['assumed'])
        assessment = self.context([estimated, rear]).assess(
            self.f.lanes[0], [self.f.lanes[1]], changed, stayed, 120.)
        self.assertIsNone(assessment['safe'])
        self.assertIn('target_follower', assessment['unavailable_reasons'])
        self.assertFalse(assessment['forecast_progress_fallback'])

    def test_estimated_front_does_not_bypass_completion_or_stay_horizon(self):
        estimated = tuple(list(free_lane.car(100., 0., 20., 7))+['assumed'])
        for length, path_length, reason in [(200., 400., 'completion_horizon'),
                                            (120., 20., 'stay_horizon')]:
            changed, stayed = self.forecast(26., 3.5), self.forecast(20., path_length=path_length)
            assessment = self.context([estimated]).assess(
                self.f.lanes[0], [self.f.lanes[1]], changed, stayed, length)
            self.assertIn(reason, assessment['unavailable_reasons'])
            self.assertFalse(assessment['forecast_progress_fallback'])


class MobilAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.f = free_lane.FreeLaneChoice(); self.f.setUp()
        self.f.config.update(lane_selection_model='mobil', mobil_politeness=.1,
                             mobil_threshold_mps2=.2, mobil_safe_decel_mps2=2.5)
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=10.)

    def settle(self, objects=(), **kwargs):
        self.f.decide(dict(self.ego, stamp=9.6), objects, **kwargs)
        return self.f.decide(self.ego, objects, **kwargs)

    def test_empty_road_keeps_current_lane(self):
        r = self.settle()
        self.assertEqual(r['status']['lane_selection_model'], 'mobil')
        self.assertEqual(r['status']['target_lane'], '0')
        self.assertFalse(r['emergency'])

    def test_slow_leader_can_trigger_an_admitted_mobil_change(self):
        r = self.settle([free_lane.car(30., 0., 8.)])
        accepted = [x for x in r['status']['candidate_checks'] if x.get('accepted')]
        self.assertTrue(accepted)
        self.assertTrue(accepted[0]['mobil']['safe'])
        self.assertGreater(accepted[0]['mobil']['incentive_mps2'], .2)
        self.assertEqual(r['status']['target_lane'], '1')

    def test_fast_rear_and_blocked_middle_lane_still_prevent_a_far_change(self):
        cars = [free_lane.car(30., 0., 8., 1), free_lane.car(-10., 3.5, 30., 2)]
        r = self.settle(cars)
        self.assertEqual(r['status']['target_lane'], '0')
        self.assertIsNone(self.f.planner.saved_world)

    def test_estimated_slow_front_uses_checked_progress_without_promoting_speed(self):
        obj = tuple(list(free_lane.car(30., 0., 8.))+['assumed'])
        first = self.f.decide(dict(self.ego, stamp=9.6), [obj])
        self.assertEqual(first['status']['target_lane'], '0')
        r = self.f.decide(self.ego, [obj])
        self.assertEqual(r['status']['target_lane'], '1')
        accepted = next(c for c in r['status']['candidate_checks'] if c.get('accepted'))
        self.assertTrue(accepted['mobil']['safe'])
        self.assertTrue(accepted['mobil']['forecast_progress_fallback'])
        self.assertFalse(accepted['mobil']['available'])
        self.assertGreaterEqual(accepted['average_kmh']-r['status']['current_average_kmh'], 2.)
        self.assertGreater(accepted['command_gap_checks'], 0)

    def test_estimated_front_fallback_cannot_bypass_footprint_veto(self):
        import planning.highway.policy as module
        real = module.timed_gap
        def veto(*args, **kwargs):
            if kwargs.get('lane_change'):
                return dict(when=1., name='lidar_car', object_id=99,
                            reason='occupied adjacent lane', rear_approach=False)
            return real(*args, **kwargs)
        obj = tuple(list(free_lane.car(30., 0., 8.))+['assumed'])
        with patch.object(module, 'timed_gap', side_effect=veto):
            r = self.settle([obj])
        self.assertEqual(r['status']['target_lane'], '0')
        self.assertIsNone(self.f.planner.saved_world)

    def test_fallback_without_forecast_gain_does_not_start_a_change(self):
        fallback = dict(available=False, safe=True, incentive_mps2=None,
            reason='MOBIL motion or follower gap unavailable',
            forecast_progress_fallback=True, unavailable_reasons=['estimated_leader'])
        with patch.object(MobilEvaluator, 'assess', return_value=fallback):
            r = self.settle()
        self.assertEqual(r['status']['target_lane'], '0')
        self.assertIsNone(self.f.planner.saved_world)
        self.assertIn('MOBIL insufficient forecast progress',
                      [c['reason'] for c in r['status']['candidate_checks']])

    def test_unknown_rear_keeps_existing_safety_rejection(self):
        r = self.settle([free_lane.car(30., 0., 8., 1),
                         free_lane.car(-30., 3.5, 0., 2, valid=False)])
        self.assertEqual(r['status']['target_lane'], '0')

    def test_mobil_does_not_replace_existing_footprint_veto(self):
        import planning.highway.policy as module
        real = module.timed_gap
        def veto(*args, **kwargs):
            if kwargs.get('lane_change'):
                return dict(when=1., collision_when=1., name='lidar_car', object_id=99,
                            reason='test occupied adjacent lane', rear_approach=False)
            return real(*args, **kwargs)
        with patch.object(module, 'timed_gap', side_effect=veto):
            r = self.settle([free_lane.car(30., 0., 8.)])
        self.assertEqual(r['status']['target_lane'], '0')
        self.assertIsNone(self.f.planner.saved_world)

    def test_an_unknown_mobil_score_does_not_create_an_emergency(self):
        unknown = dict(available=False, safe=None, incentive_mps2=None, reason='unknown motion')
        with patch.object(MobilEvaluator, 'assess', return_value=unknown):
            r = self.settle([free_lane.car(110., 0., 10.)])
        self.assertFalse(r['emergency'])
        self.assertEqual(r['status']['target_lane'], '0')

    def test_observation_gain_does_not_bypass_stability_time(self):
        cars = [free_lane.car(30., 0., 8.)]
        r = self.f.decide(dict(self.ego, stamp=9.6), cars)
        self.assertEqual(r['status']['target_lane'], '0')
        r = self.f.decide(self.ego, cars)
        self.assertEqual(r['status']['target_lane'], '1')

    def test_next_leg_rechecks_mobil_even_when_footprint_check_passes(self):
        f = three_lane.ThreeLaneTests(); f.setUp()
        f.planner = HighwayPlanner(dict(f.config, lane_selection_model='mobil',
            comfort_decel_mps2=2.5), f.route)
        first = f.settle([f.car(65., 0., 15.), f.car(110., 3.5, 15., 2)])
        self.assertEqual(first['status']['planned_lane_sequence'], ['1', '2'])
        length = first['status']['selected_transition_m']
        from highway_three_lane import HighwayLane
        lanes = [HighwayLane(l.link_id, l.rank,
            l.centre[l.centre[:, 0] >= length]-[length, 3.5], l.chain, 800.-length) for l in f.lanes]
        unsafe = dict(available=True, safe=False, incentive_mps2=1.,
                      reason='MOBIL target follower braking exceeds limit')
        with patch.object(MobilEvaluator, 'assess', return_value=unsafe) as assess:
            result = f.decide([f.car(length+40., 3.5, 15., 3)], lanes=lanes,
                             ego=dict(f.ego, x=length, y=3.5, stamp=16.))
        self.assertGreater(assess.call_count, 0)
        self.assertEqual(f.planner.completed, 1)
        self.assertIsNone(f.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '1')
        self.assertFalse(result['emergency'])
        self.assertIn('next leg MOBIL admission blocked', result['status']['continuation_replan_reasons'])

    def test_progress_rejection_does_not_claim_incentive_satisfied(self):
        high = dict(available=True, safe=True, incentive_mps2=1., reason='MOBIL incentive satisfied')
        with patch.object(MobilEvaluator, 'assess', return_value=high):
            r = self.settle()
        reasons = [x['reason'] for x in r['status']['candidate_checks'] if 'mobil' in x]
        self.assertTrue(reasons)
        self.assertIn('MOBIL insufficient forecast progress', reasons)
        self.assertNotIn('MOBIL incentive satisfied', reasons)

    def test_opt_in_does_not_change_other_policy_configurations(self):
        self.f.config.pop('lane_selection_model')
        self.f.planner = HighwayPlanner(self.f.config, self.f.route)
        r = self.settle([free_lane.car(30., 0., 8.)])
        self.assertEqual(r['status']['lane_selection_model'], 'forecast_progress')
        self.assertEqual(r['status']['target_lane'], '1')


class MobilRecordedAdmissionTests(unittest.TestCase):
    """Replay the missed opportunity in follow-20261012T162958Z-4120230.

    These two frames had zero filtered LiDAR points and no retained near-field
    points; the original cloud had ten unconfirmed far returns. Final object
    tracks are already aligned and need only the wall-to-simulator conversion.
    GUI poses are rounded. This verifies planner admission and its command
    checks, rather than reproducing the later physical or sensor evolution.
    """
    FRAMES = [
        dict(ego=dict(x=64.99, y=147.15, heading_deg=-88.28, speed_kmh=59.2,
                      stamp=1791822816.7492757), moving_age=.178, static_age=.045,
             fusion_age=.147, objects=[
                 (66.04111642637723, 105.87461829303862, 4.932117462158203,
                  'lidar_car', 'npc', 8.632370724578628, 4.932117462158203,
                  1.9850480556488037, 269.8827043632054, .516725035440624,
                  -8.616891525620538, 500003, 0., None, True, 0., 'assumed')]),
        dict(ego=dict(x=65.1, y=143.43, heading_deg=-88.3, speed_kmh=50.5,
                      stamp=1791822817.0547733), moving_age=.003, static_age=.02,
             fusion_age=.119, objects=[
                 (66.10270650321071, 102.06754678413208, 4.6, 'fusion_parked_vehicle',
                  'npc', 8.630598876697297, 4.6, 1.9, -86.41110135274234,
                  .5402681626203578, -8.613672113733346, 55, 0., None, True, 0.,
                  'assumed', {'height_m': 1.2326449784315692, 'raw_size_m': [4.6, 1.9]}),
                 (65.38493137177016, 103.20685097178496, 5.4597079336311936,
                  'lidar_car', 'npc', 8.630598876697297, 5.4597079336311936,
                  2.635177808966887, 269.4342782193241, .5402025743907062,
                  -8.61367622731841, 500003, 0., None, True, .30573248863220215,
                  'assumed')])]

    @classmethod
    def setUpClass(cls):
        free_lane.RecordedFreeLaneReplay.setUpClass()
        for name in ('route', 'smooth', 'lanes'):
            setattr(cls, name, getattr(free_lane.RecordedFreeLaneReplay, name))

    def replay(self, modifier=None):
        from highway_three_lane import HighwayLaneMap
        from planning.highway.object_policy import highway_static_objects
        from sim_time import to_sim_objects
        from gui import static_clear_distance
        planner = HighwayPlanner(dict(free_lane.MISSION), self.route)
        native = HighwayLaneMap(self.lanes, planner.config)
        original = MobilEvaluator.assess
        def assess(evaluator, current, sequence, *args):
            result = original(evaluator, current, sequence, *args)
            return modifier(result, sequence) if modifier else result
        results = []
        with patch.object(MobilEvaluator, 'assess', new=assess):
            for frame in self.FRAMES:
                ego, rtf = frame['ego'], .777
                index = 1800+int(np.argmin(np.linalg.norm(
                    self.route[1800:3600]-[ego['x'], ego['y']], axis=1)))
                local = free_lane.local_points(self.smooth[index:index+640], ego)
                objects = to_sim_objects(frame['objects'], rtf)
                lanes = native.candidates(ego, local)
                corridor = native.corridor(ego, local, lanes)
                model = planner.longitudinal
                model.time_scale, model.sensor_age_s = rtf, frame['moving_age']
                model.static_sensor_age_s, model.fusion_sensor_age_s = frame['static_age'], frame['fusion_age']
                bodies = highway_static_objects(objects)
                results.append(planner.decide(index, local, ego, objects, {},
                    corridor.clear_distance,
                    lambda path, horizon: static_clear_distance(path, ego, bodies, horizon),
                    perception_ready=True, lane_options=lanes,
                    motion_options=native.motion_candidates(ego, objects)))
        return results

    def test_missed_two_leg_opportunity_passes_existing_command_and_body_guards(self):
        def old_admission(result, sequence):
            return dict(result, forecast_progress_fallback=False)
        before, after = self.replay(old_admission), self.replay()
        self.assertEqual(before[-1]['status']['planned_lane_sequence'], [])
        self.assertEqual(after[0]['status']['planned_lane_sequence'], [])  # stability
        final = after[-1]
        self.assertFalse(final['emergency'])
        self.assertEqual(final['status']['planned_lane_sequence'],
                         ['A2256W000409', 'A2256W000410'])
        admitted = next(c for c in final['status']['candidate_checks'] if c['accepted'])
        self.assertTrue(admitted['mobil']['safe'])
        self.assertTrue(admitted['mobil']['forecast_progress_fallback'])
        self.assertGreater(admitted['average_kmh']-final['status']['current_average_kmh'], 30.)
        self.assertGreater(admitted['command_gap_checks'], 0)
        self.assertEqual(admitted['command_check_horizon_s'], 3.)
        self.assertEqual(admitted['traffic_check_horizon_s'], 14.)
        self.assertGreaterEqual(admitted['command_footprint_margin_m'], .89)

    def test_mixed_known_utility_and_estimated_leader_candidates_rank_by_progress(self):
        def known_far_lane(result, sequence):
            if len(sequence) == 3 and result.get('forecast_progress_fallback'):
                return dict(result, available=True, incentive_mps2=2.,
                            forecast_progress_fallback=False)
            return result
        final = self.replay(known_far_lane)[-1]
        self.assertEqual(final['status']['planned_lane_sequence'],
                         ['A2256W000409', 'A2256W000410'])


if __name__ == '__main__':
    unittest.main()
