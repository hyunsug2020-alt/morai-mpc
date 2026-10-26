"""Following, temporal collision checks and the published speed share a plan."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from highway_planner import HighwayPlanner, timed_gap, world_points
from highway_speed_plan import (HighwaySpeedPlanner, SpeedTrajectory,
                               ACCEL_CONTROL_LOOKAHEAD_S, JERK_MPS3, STEP_S)
from gui import highway_objects
from ego_body import EGO_FRONT_M
from test_highway_speed_recovery import (independent_command_motion,
                                          independent_physical_contacts)


class HighwaySpeedPlanTests(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 321., .5), np.zeros(642)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=100.)
        self.planner = HighwayPlanner(dict(from_idx=0, to_idx=600, kmh=90.), self.path)
        self.planner.phase, self.planner.completed = 'DONE', 2

    @staticmethod
    def car(x, y=0., speed=40. / 3.6, heading=0., valid=True, uid=1):
        yaw = np.radians(heading)
        return (x, y, 4.9, 'car', 'npc', speed, 4.9, 2.1, heading,
                speed * np.cos(yaw), speed * np.sin(yaw), uid, 0., None, valid)

    def plan(self, stamp=100., goal=25., objects=(), **kwargs):
        ego = dict(self.ego, stamp=stamp)
        self.planner.longitudinal.begin(ego)
        return self.planner.plan_speed(self.path, ego, objects, goal, **kwargs)

    def test_acceleration_and_jerk_are_bounded_when_goal_changes(self):
        model = HighwaySpeedPlanner()
        for goal in (0., 25., 10.):
            for acceleration in (-2.5, 0., 2.5):
                model.acceleration = acceleration
                trajectory = model.preview(self.path, self.ego, (), goal)
                self.assertLessEqual(np.max(trajectory.accelerations), 2.5 + 1e-9)
                self.assertGreaterEqual(np.min(trajectory.accelerations), -2.5 - 1e-9)
                self.assertLessEqual(np.max(np.abs(np.diff(trajectory.accelerations))) / STEP_S,
                                     JERK_MPS3 + 1e-9)
                self.assertTrue(np.all(np.diff(trajectory.distances) >= 0.))

    def test_future_headway_avoids_the_cruise_prediction_collision(self):
        cars = [self.car(70.)]
        self.assertIsNotNone(timed_gap(self.path, self.ego, cars, 25., lane_change=False))
        self.planner.longitudinal.begin(self.ego)
        following = self.planner.longitudinal.preview(self.path, self.ego, cars, 25.)
        self.assertIsNone(timed_gap(self.path, self.ego, cars, 25., lane_change=False,
                                    speed_trajectory=following))
        self.assertLess(following.speeds[-1], 15.)
        with patch('highway_planner.timed_gap', wraps=timed_gap) as checked:
            result = self.planner.plan_speed(self.path, self.ego, cars, 25.)
            profile = checked.call_args.kwargs['speed_trajectory']
            self.assertAlmostEqual(result['reference_mps'], profile.reference_mps)
            self.assertFalse(result['emergency'])

    def test_different_speed_goals_keep_different_acceleration_plans(self):
        ego = dict(self.ego, speed_kmh=60.)
        model = HighwaySpeedPlanner()
        references = [model.preview(self.path, ego, (), goal / 3.6).reference_mps
                      for goal in (40., 60., 90.)]
        self.assertLess(references[0], references[1])
        self.assertLess(references[1], references[2])
        self.assertAlmostEqual(references[1] * 3.6, 60.)

    def test_distant_crossing_stays_a_lane_prediction_without_a_speed_dip(self):
        crossing = self.car(105., 31.5, speed=7.5, heading=-90.)
        risk = timed_gap(self.path, self.ego, [crossing], 25., lane_change=False)
        self.assertGreater(risk['collision_when'], 3.)
        result = self.planner.decide(0, self.path, self.ego, [crossing], {},
                                     lambda path, horizon: horizon, lambda path, horizon: horizon)
        self.assertFalse(result['emergency'])
        self.assertAlmostEqual(result['target_mps'], 25.)
        command = self.planner.plan_speed(result['path'], self.ego, [crossing], result['target_mps'])
        self.assertFalse(command['emergency'])
        self.assertAlmostEqual(command['reference_mps'], 25.)

    def test_all_cruise_lane_options_are_tried_before_a_slower_change(self):
        self.planner.phase, self.planner.completed = 'WAIT_GAP', 0
        self.planner.change_until = 600
        def gap(path, ego, objects, pace, **kwargs):
            if path[-1, 1] > 3. and pace > 20.:
                return dict(when=5., collision_when=5., reason='left lane occupied')
            return None
        with patch.object(self.planner, 'predicted_gap', side_effect=gap):
            result = self.planner.decide(0, self.path, self.ego, [], {1:3.5, -1:3.5},
                                         lambda path, horizon: horizon, lambda path, horizon: horizon)
        self.assertEqual(self.planner.selected_side, -1)
        self.assertAlmostEqual(result['target_mps'], 25.)
        self.assertFalse(result['emergency'])

    def test_measured_acceleration_never_raises_command_above_mission_limit(self):
        self.planner.longitudinal.begin(self.ego)
        self.planner.longitudinal.acceleration = 2.5
        result = self.planner.plan_speed(self.path, self.ego, (), 25.)
        self.assertFalse(result['emergency'])
        self.assertLessEqual(result['reference_mps'], 25.)

    def test_throttle_reference_recovers_cruise_without_extending_brake_lookahead(self):
        model = HighwaySpeedPlanner()
        accelerating = model.preview(self.path, dict(self.ego, speed_kmh=60.), (), 25.)
        braking = model.preview(self.path, self.ego, (), 15.)
        self.assertGreater(accelerating.reference_mps * 3.6, 65.)
        self.assertLess(accelerating.reference_mps * 3.6, 75.)
        self.assertGreater(accelerating.reference_time_s, .9)
        self.assertLessEqual(accelerating.reference_time_s, ACCEL_CONTROL_LOOKAHEAD_S)
        self.assertEqual(braking.reference_time_s, .4)
        self.assertGreater(braking.reference_mps * 3.6, 88.)

    def test_acceleration_reference_stops_before_the_checked_braking_switch(self):
        times = np.arange(0., 3.01, .1)
        acceleration = np.where(times <= .8, 2., -2.)
        speeds = 20. + np.cumsum(acceleration) * .1
        speeds[0] = 20.
        trajectory = SpeedTrajectory(times, times * 20., speeds, acceleration, 30.)
        self.assertGreater(trajectory.reference_mps, speeds[0])
        self.assertLessEqual(trajectory.reference_time_s, .8)
        self.assertGreaterEqual(np.interp(trajectory.reference_time_s,
                                        times, acceleration), 0.)

    def test_sensor_hold_and_tight_clearance_keep_their_hard_command_ceiling(self):
        for measured_kmh, ceiling_kmh in ((60., 60.), (2.9, 5.)):
            ego = dict(self.ego, speed_kmh=measured_kmh)
            self.planner.longitudinal.begin(ego)
            self.planner.longitudinal.acceleration = 2.5
            result = self.planner.plan_speed(self.path, ego, (), ceiling_kmh / 3.6,
                                            ceiling_mps=ceiling_kmh / 3.6)
            self.assertFalse(result['emergency'])
            self.assertLessEqual(result['reference_mps'] * 3.6, ceiling_kmh + 1e-9)

    def test_following_uses_world_velocity_in_the_ego_frame(self):
        ego = dict(self.ego, heading_deg=-90., x=10., y=20.)
        xy = world_points(np.array([[70., 0.]]), ego)[0]
        car = self.car(*xy, heading=-90.)
        model = HighwaySpeedPlanner()
        original = model.preview(self.path, self.ego, [self.car(70.)], 25.)
        rotated = model.preview(self.path, ego, [car], 25.)
        np.testing.assert_allclose(rotated.speeds, original.speeds, atol=1e-9)

    def test_side_and_rear_cars_do_not_become_front_following_targets(self):
        model = HighwaySpeedPlanner()
        free = model.preview(self.path, self.ego, (), 25.)
        traffic = model.preview(self.path, self.ego,
                                [self.car(30., 3.5), self.car(-30.)], 25.)
        np.testing.assert_allclose(traffic.speeds, free.speeds)

    def test_unknown_velocity_does_not_invent_a_moving_leader(self):
        model = HighwaySpeedPlanner()
        free = model.preview(self.path, self.ego, (), 25.)
        unknown = model.preview(self.path, self.ego, [self.car(70., valid=False)], 25.)
        np.testing.assert_allclose(unknown.speeds, free.speeds)

    def test_short_goal_hold_expires_and_does_not_lock_a_recovered_cap(self):
        self.plan(goal=25.)
        self.plan(stamp=100.1, goal=15.)
        held = self.plan(stamp=100.2, goal=25.)
        self.assertTrue(held['diagnostics']['held'])
        self.assertEqual(held['diagnostics']['goal_kmh'], 54.)
        self.plan(stamp=100.5, goal=25.)
        recovered = self.plan(stamp=100.8, goal=25.)
        self.assertFalse(recovered['diagnostics']['held'])
        self.assertEqual(recovered['diagnostics']['goal_kmh'], 90.)

    def test_lower_safety_goal_is_applied_during_the_hold(self):
        self.plan(goal=15.)
        result = self.plan(stamp=100.1, goal=10.)
        self.assertEqual(result['diagnostics']['goal_kmh'], 36.)
        self.assertLess(result['reference_mps'], self.ego['speed_kmh'] / 3.6)

    def test_clock_restart_clears_acceleration_and_old_goal_hold(self):
        self.plan(goal=10.)
        self.planner.longitudinal.acceleration = -2.5
        result = self.plan(stamp=5., goal=25.)
        self.assertEqual(self.planner.longitudinal.acceleration, 0.)
        self.assertFalse(result['diagnostics']['held'])
        self.assertEqual(result['diagnostics']['goal_kmh'], 90.)

    def test_emergency_bypasses_gradual_braking_and_releases_old_hold(self):
        self.plan(goal=15.)
        result = self.plan(stamp=100.1, emergency=True)
        self.assertEqual(result['reference_mps'], 0.)
        self.assertTrue(result['emergency'])
        self.assertFalse(result['diagnostics']['reference_checked'])
        self.assertIsNone(self.planner._speed_hold_goal)

    def test_no_safe_gradual_trajectory_retains_immediate_stop(self):
        result = self.plan(objects=[self.car(4., speed=0.)], static_clear_m=4.)
        self.assertTrue(result['emergency'])
        self.assertEqual(result['reference_mps'], 0.)

    def test_recorded_late_cut_in_uses_checked_braking_before_emergency_stop(self):
        fixture = json.loads((Path(__file__).parent /
                              'test_data/highway_speed_cut_in_20261007.json').read_text())
        ego, objects = fixture['ego'], fixture['objects']
        path = np.asarray(fixture['path'])
        self.planner.longitudinal.begin(ego)
        self.planner.longitudinal.acceleration = fixture['acceleration']
        if fixture['lane_change']:
            self.planner.saved_world = world_points(path, ego)
        with patch('highway_planner.timed_gap', wraps=timed_gap) as checked, \
                patch.object(self.planner.longitudinal, 'command_preview',
                             wraps=self.planner.longitudinal.command_preview) as commands:
            result = self.planner.plan_speed(path, ego, objects, fixture['goal_mps'])
            self.assertFalse(result['emergency'])
            self.assertGreater(result['reference_mps'] * 3.6, 75.)
            profile = commands.call_args.args[0]
            self.assertEqual(profile.braking_limit_mps2,
                             result['diagnostics']['braking_limit_mps2'])
            self.assertLess(result['reference_mps'], ego['speed_kmh']/3.6)
            self.assertLess(result['diagnostics']['acceleration_mps2'], 0.)
            self.assertTrue(result['diagnostics']['execution_reference_checked'])
            self.assertIsNone(timed_gap(path, ego, objects, profile.goal_mps,
                                       lane_change=fixture['lane_change'],
                                       speed_trajectory=profile,
                                       footprint_margin_m=result['diagnostics']['footprint_margin_m']))
            # Keep every recorded coordinate and box. Safety, rather than the
            # former exact escalation level, is checked on independent corner
            # polygons for both nominal and scalar-command motion.
            self.assertEqual(independent_physical_contacts(path,ego,objects,profile),[])
            executed=independent_command_motion(profile)
            self.assertEqual(independent_physical_contacts(path,ego,objects,executed),[])
            self.assertIsNone(timed_gap(path,ego,objects,profile.reference_mps,
                lane_change=fixture['lane_change'],speed_trajectory=executed,
                footprint_margin_m=.2))

    def test_recorded_camera_centroid_spikes_do_not_duplicate_a_measured_car(self):
        fixture = json.loads((Path(__file__).parent /
                              'test_data/highway_semantic_car_spikes_20261007.json').read_text())
        for frame in fixture['frames']:
            ego = frame['ego']
            fused, cars = [tuple(o) for o in frame['fused']], [tuple(o) for o in frame['cars']]
            self.assertEqual(len(fused), 1)
            self.assertEqual(len(cars), 1)
            before = timed_gap(self.path, ego, fused + cars, 50. / 3.6, lane_change=False)
            self.assertIsNotNone(before)
            self.assertLess(before['collision_when'], 1.)
            merged = highway_objects(fused, cars)
            self.assertEqual(merged, cars)
            after = timed_gap(self.path, ego, merged, 50. / 3.6,
                              lane_change=False, horizon_s=3.)
            if after is not None:
                self.assertGreaterEqual(after['collision_when'], 2.)
            self.planner.longitudinal.begin(ego)
            command = self.planner.plan_speed(self.path, ego, merged, 50. / 3.6)
            self.assertFalse(command['emergency'])
            self.assertGreater(command['reference_mps'] * 3.6, ego['speed_kmh'] - 5.)

    def test_association_margin_preserves_separate_adjacent_and_following_cars(self):
        lidar = self.car(40.)
        shifted = list(lidar); shifted[1] = 1.4; shifted[3] = 'fusion_npc'
        neighbor = list(shifted); neighbor[1] = 3.5
        following = list(shifted); following[0] = 45.
        self.assertEqual(highway_objects([tuple(shifted), tuple(neighbor), tuple(following)],
                                        [lidar]), [tuple(neighbor), tuple(following), lidar])
        unknown = list(lidar); unknown[14] = False
        self.assertEqual(len(highway_objects([tuple(shifted)], [tuple(unknown)])), 2)
        predicted = tuple(list(lidar) + [.2])
        self.assertEqual(len(highway_objects([tuple(shifted)], [predicted])), 2)

    def test_static_obstacle_stopping_distance_is_checked_before_smoothing(self):
        with patch('highway_planner.timed_gap', wraps=timed_gap) as checked:
            result = self.plan(static_clear_m=120.)
            self.assertFalse(result['emergency'])
            trajectory = checked.call_args.kwargs['speed_trajectory']
            self.assertLessEqual(np.max(trajectory.distances), 120. - .25)
            self.assertLess(result['reference_mps'], 25.)

    def test_following_settles_behind_a_slow_leader_with_three_car_traffic(self):
        # Pedal gains match the follower; braking strength and 0.25 s lag are
        # an explicit approximate plant, not a substitute for the MORAI drive.
        x, speed, acceleration, lead_x, lead_speed = 0., 25., 0., 70., 40. / 3.6
        references, gaps, brakes, emergencies = [], [], [], []
        for i in range(350):
            ego = dict(self.ego, x=x, speed_kmh=speed * 3.6, stamp=100. + i * .1)
            cars = [self.car(lead_x), self.car(lead_x + 20., 3.5, uid=2),
                    self.car(lead_x - 25., -3.5, speed=50. / 3.6, uid=3)]
            self.planner.longitudinal.begin(ego)
            result = self.planner.plan_speed(self.path, ego, cars, 25.)
            reference = result['reference_mps']
            error = reference - speed
            brake = min(1., max(0., -.45 * error)) if error < -.15 else 0.
            throttle = min(1., max(0., .35 * error)) if error > .15 else 0.
            acceleration += .1 / .25 * (-8. * brake + 3. * throttle - acceleration)
            speed = max(0., speed + acceleration * .1)
            x += speed * .1
            lead_x += lead_speed * .1
            references.append(reference * 3.6)
            gaps.append(lead_x - x - EGO_FRONT_M - 2.45)
            brakes.append(brake)
            emergencies.append(result['emergency'])
        self.assertFalse(any(emergencies))
        self.assertGreater(min(gaps), 5.)
        self.assertLess(max(brakes), .65)
        self.assertLess(np.max(np.abs(np.diff(references))), 3.)
        self.assertAlmostEqual(speed * 3.6, 40., delta=1.)


if __name__ == '__main__':
    unittest.main()
