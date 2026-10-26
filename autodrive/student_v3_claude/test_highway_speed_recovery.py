"""Measured braking recovery and independent highway collision vetoes."""
import json
import math
import unittest
from unittest.mock import patch

import numpy as np

from highway_planner import HighwayPlanner, timed_gap, local_points
from highway_speed_plan import (HighwaySpeedPlanner, SpeedTrajectory, STEP_S, TRAFFIC_CONTROL_HORIZON_S,
                               FOLLOWER_ACCEL_GAIN, FOLLOWER_FULL_ACCEL_MPS2, HIGHWAY_ACCEL_MPS2)
from ego_body import EGO_CENTRE_M, EGO_FRONT_M, EGO_REAR_M, EGO_HALF_WIDTH_M
from test_highway_rear_axle import polygon, polygons_overlap, vehicle_polygon


def independent_command_motion(trajectory):
    """Explicit follower/plant model, independent of command_preview()."""
    times = trajectory.times[trajectory.times<=TRAFFIC_CONTROL_HORIZON_S+1e-9]
    speed, acceleration = trajectory.speeds[0], trajectory.accelerations[0]
    emitted, reference = speed, trajectory.reference_mps
    speeds, distances, accelerations = [speed], [0.], [acceleration]
    for elapsed in np.diff(times):
        emitted = min(reference, emitted+3.*elapsed)
        error = emitted-speed
        throttle = min(1., max(0., .35*error)) if error>.15 else 0.
        brake = min(1., max(0., -.45*error)) if error<-.15 else 0.
        acceleration += elapsed/.25*(3.*throttle-8.*brake-acceleration)
        next_speed = max(0., speed+acceleration*elapsed)
        distances.append(distances[-1]+(speed+next_speed)*elapsed/2.)
        speeds.append(next_speed)
        accelerations.append(acceleration)
        speed = next_speed
    return SpeedTrajectory(times, np.asarray(distances), np.asarray(speeds),
                           np.asarray(accelerations), reference)


def independent_physical_contacts(path, ego, objects, trajectory, footprint_margin_m=0.):
    """Corner-polygon witnesses for recorded linear motion and piecewise paths.

    This oracle has no rear exemptions and does not call the production SAT,
    path-pose sampler or body-support helpers. A positive margin enlarges all
    four body sides conservatively, including along oblique projection axes.
    Native curved forecasts require a separate recorded world-motion oracle
    and are outside this helper's scope.
    """
    path = np.asarray(path, dtype=float)
    if np.linalg.norm(path[0])>1e-9:
        path = np.vstack(([0.,0.],path))
    path = path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-9]]
    stations = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    angle = math.radians(ego['heading_deg'])
    rotation = np.array([[math.cos(angle),-math.sin(angle)],
                         [math.sin(angle),math.cos(angle)]])
    contacts = []
    for stamp,distance in zip(trajectory.times,trajectory.distances):
        xy = np.array([np.interp(distance,stations,path[:,k]) for k in (0,1)])
        at = min(len(path)-1,max(1,int(np.searchsorted(stations,distance))))
        tangent = path[at]-path[at-1]
        heading = ego['heading_deg']+math.degrees(math.atan2(tangent[1],tangent[0]))
        if distance<=1e-9:
            xy,heading = np.zeros(2),ego['heading_deg']
        axle = xy @ rotation.T+[ego['x'],ego['y']]
        front,rear,half_width = 3.845+footprint_margin_m,.79+footprint_margin_m,.946+footprint_margin_m
        body = polygon([(front,half_width),(-rear,half_width),
                        (-rear,-half_width),(front,-half_width)],axle,heading)
        for obj in objects:
            other = list(obj)
            other[:2] = np.asarray(obj[:2])+stamp*np.asarray(obj[9:11])
            if polygons_overlap(body,vehicle_polygon(other)):
                contacts.append(dict(time_s=float(stamp),uid=obj[11]))
    return contacts


class HighwaySpeedRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 501., .5), np.zeros(1002)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=30., stamp=100.)
        self.planner = HighwayPlanner(dict(from_idx=0, to_idx=900, kmh=90.,
            lane_change_policy='free_three_lane'), self.path)

    @staticmethod
    def car(x, speed=22., uid=1):
        return (x, 0., 5., 'car', 'npc', speed, 5., 2., 0.,
                speed, 0., uid, 0., None, True, 0.)

    def install_rear_motion(self):
        native = np.column_stack((np.arange(-200., 501.), np.zeros(701)))
        self.planner.longitudinal.motion_paths[1] = (
            native, np.arange(701.), 190., 22., 0., 0.)
        self.planner.saved_world = self.path.copy()

    def test_clear_road_recovery_does_not_command_another_brake(self):
        model = self.planner.longitudinal
        model.acceleration = -1.2
        trajectory = model.preview(self.path, self.ego, (), 25.,
                                   static_clear_m=float('inf'))
        self.assertEqual(trajectory.accelerations[0], -1.2)
        self.assertLess(trajectory.speeds[4], trajectory.speeds[0])
        self.assertGreater(trajectory.reference_mps, trajectory.speeds[0])
        self.assertGreater(trajectory.reference_time_s, .9)
        self.assertLessEqual(trajectory.reference_time_s, TRAFFIC_CONTROL_HORIZON_S)
        self.assertAlmostEqual(trajectory.reference_mps, np.interp(
            trajectory.reference_time_s, trajectory.times, trajectory.speeds))
        self.assertLessEqual(np.max(np.abs(np.diff(trajectory.accelerations))) / STEP_S,
                             2. + 1e-9)
        command = self.planner.plan_speed(self.path, self.ego, (), 25.,
                                        static_clear_m=float('inf'))
        self.assertFalse(command['emergency'])
        self.assertGreater(command['reference_mps'], 30. / 3.6)

    def test_braking_and_leader_constraints_do_not_invent_recovery(self):
        model = self.planner.longitudinal
        model.acceleration = -1.2
        for objects, goal in (((), 0.), ([self.car(15., speed=2.)], 25.)):
            trajectory = model.preview(self.path, self.ego, objects, goal)
            self.assertEqual(trajectory.reference_time_s, .4)
            self.assertLess(trajectory.reference_mps, trajectory.speeds[0])
        stopped = self.planner.plan_speed(self.path, self.ego,
            [self.car(4., speed=0.)], 25., static_clear_m=4.)
        self.assertTrue(stopped['emergency'])
        self.assertEqual(stopped['reference_mps'], 0.)
        bounded = self.planner.plan_speed(self.path, self.ego, (), 25.,
            static_clear_m=float('inf'), ceiling_mps=30. / 3.6)
        self.assertLessEqual(bounded['reference_mps'], 30. / 3.6)

    def test_committed_unknown_car_retains_lane_admission_margin(self):
        ego = dict(x=68.03, y=-44.21, heading_deg=-89.16,
                   speed_kmh=98.6, stamp=100.)
        world = np.column_stack((np.full(1002, ego['x']),
                                 ego['y']-np.arange(0., 501., .5)))
        path = local_points(world, ego)
        unknown = (70.58726052755695, -73.51413440631981, 5.05068826675415,
                   'lidar_car', 'npc', 0., 5.05068826675415, 2.1242709159851074,
                   265.5011664207111, 0., 0., 500012, 0., None, False, .1508)
        self.planner.target_mps = self.planner.overtake_mps = 120./3.6
        self.planner.saved_world = world.copy()
        trajectory = self.planner.longitudinal.preview(path, ego, [unknown], 120./3.6)
        admission = timed_gap(path, ego, [unknown], 120./3.6,
                              speed_trajectory=trajectory.prefix(3.))
        self.assertIsNotNone(admission)
        self.assertIsNotNone(admission['collision_when'])
        result = self.planner.plan_speed(path, ego, [unknown], 120./3.6,
                                        static_clear_m=float('inf'))
        self.assertTrue(result['emergency'])
        self.assertEqual(result['reference_mps'], 0.)
        self.assertIsNone(admission['physical_collision_when'])
        self.assertEqual(admission['footprint_margin_m'], .6)
        self.assertEqual(result['diagnostics']['command_footprint_margin_m'], .6)
        self.planner.saved_world = None
        current = self.planner.plan_speed(path, ego, [unknown], 120./3.6,
                                         static_clear_m=float('inf'))
        self.assertFalse(current['emergency'])
        self.assertGreater(current['reference_mps'], ego['speed_kmh']/3.6)
        self.assertEqual(current['diagnostics']['command_footprint_margin_m'], .2)

    def test_unknown_car_body_on_the_current_path_still_requires_emergency_braking(self):
        ego = dict(self.ego, speed_kmh=98.6)
        unknown = (29., 1.8, 5., 'lidar_car', 'npc', 0.,
                   5., 2., 0., 0., 0., 500012, 0., None, False, .15)
        self.planner.target_mps = self.planner.overtake_mps = 120./3.6
        self.planner.saved_world = self.path.copy()
        result = self.planner.plan_speed(self.path, ego, [unknown], 120./3.6,
                                        static_clear_m=float('inf'))
        self.assertTrue(result['emergency'])
        self.assertEqual(result['reference_mps'], 0.)
        self.assertIsNotNone(result['diagnostics']['risk']['physical_collision_when'])

    @staticmethod
    def command_motion(trajectory):
        """Independent explicit follower/plant model for scalar-reference checks."""
        return independent_command_motion(trajectory)

    def test_recovery_command_cannot_skip_braking_needed_for_a_current_crossing(self):
        # The synthetic crossing was specified relative to the old body centre.
        crossing = (11.+EGO_CENTRE_M, -2.8, .6, 'crossing', 'pedestrian', 4.,
                    .6, .6, 90., 0., 4., 42)
        model = self.planner.longitudinal
        model.acceleration = -1.2
        before = model.preview(self.path, self.ego, [crossing], 25.)
        self.assertGreater(before.reference_mps, before.speeds[0])
        self.assertIsNone(timed_gap(self.path, self.ego, [crossing], 25., lane_change=False,
                                   speed_trajectory=before.prefix(3.), footprint_margin_m=.2))
        executed_risk = timed_gap(self.path, self.ego, [crossing], 25., lane_change=False,
                                 speed_trajectory=self.command_motion(before), footprint_margin_m=.2)
        self.assertIsNotNone(executed_risk)
        self.assertIsNotNone(executed_risk['physical_collision_when'])
        result = self.planner.plan_speed(self.path, self.ego, [crossing], 25.,
                                        static_clear_m=float('inf'))
        self.assertFalse(result['emergency'])
        self.assertLess(result['reference_mps'], self.ego['speed_kmh']/3.6)
        self.assertTrue(result['diagnostics']['recovery_brake_retained'])
        self.assertTrue(result['diagnostics']['execution_reference_checked'])
        # The status topic serialises this (a numpy bool stopped the planner, follow-20261010T055542Z).
        self.assertIs(type(result['diagnostics']['recovery_brake_retained']), bool)
        json.dumps(result['diagnostics'])
        before.reference_override_s = result['diagnostics']['reference_time_s']
        self.assertIsNone(timed_gap(self.path, self.ego, [crossing], 25., lane_change=False,
                                   speed_trajectory=self.command_motion(before), footprint_margin_m=.2))

    def test_unknown_rear_shell_cannot_hide_a_crossing_body_from_command_validation(self):
        # Preserve the synthetic body-centre clearance in rear-axle coordinates.
        crossing = (11.+EGO_CENTRE_M, -2.8, .6, 'crossing', 'pedestrian', 4.,
                    .6, .6, 90., 0., 4., 42)
        rear = (-4., 2., 5., 'lidar_car', 'npc', 0.,
                5., 2., 0., 0., 0., 43, 0., None, False, .1)
        self.planner.saved_world = self.path.copy()
        self.planner.longitudinal.acceleration = -1.2
        trajectory = self.planner.longitudinal.preview(self.path, self.ego, [rear, crossing], 25.)
        risk = timed_gap(self.path, self.ego, [rear, crossing], 25.,
                         speed_trajectory=self.command_motion(trajectory),
                         ignore_rear_approach=True, footprint_margin_m=.2)
        self.assertEqual(risk['object_id'], 42)
        self.assertIsNotNone(risk['physical_collision_when'])
        self.assertEqual(independent_physical_contacts(self.path,self.ego,[rear,crossing],
            self.command_motion(trajectory)),[dict(time_s=1.,uid=42)])
        with patch('highway_planner.timed_gap', wraps=timed_gap) as checked, \
                patch.object(self.planner.longitudinal, 'command_preview',
                             wraps=self.planner.longitudinal.command_preview) as commands:
            result = self.planner.plan_speed(self.path, self.ego, [rear, crossing], 25.,
                                            static_clear_m=float('inf'))
            final = commands.call_args.args[0]
        self.assertFalse(result['emergency'])
        self.assertLess(result['reference_mps'], self.ego['speed_kmh']/3.6)
        self.assertLess(result['diagnostics']['acceleration_mps2'],0.)
        self.assertTrue(result['diagnostics']['execution_reference_checked'])
        self.assertEqual(result['diagnostics']['command_footprint_margin_m'],.6)
        self.assertAlmostEqual(final.reference_mps,result['reference_mps'])
        executed = self.command_motion(final)
        self.assertIsNone(timed_gap(self.path,self.ego,[rear,crossing],final.reference_mps,
            speed_trajectory=executed,footprint_margin_m=.6))
        self.assertEqual(independent_physical_contacts(self.path,self.ego,[rear,crossing],
            executed,footprint_margin_m=.6),[])

    def test_recovery_breaks_the_measured_braking_feedback_loop(self):
        speed, acceleration = 30. / 3.6, -1.2
        for i in range(30):
            ego = dict(self.ego, speed_kmh=speed*3.6, stamp=100.+i*.1)
            self.planner.longitudinal.begin(ego)
            if i == 0:
                self.planner.longitudinal.acceleration = acceleration
            command = self.planner.plan_speed(self.path, ego, (), 25.,
                                             static_clear_m=float('inf'))
            self.assertFalse(command['emergency'])
            error = command['reference_mps']-speed
            self.assertGreaterEqual(error, 0.)
            throttle = min(1., .35*error) if error > .15 else 0.
            acceleration += .1/.25*(3.*throttle-acceleration)
            speed = max(0., speed+acceleration*.1)
        self.assertGreater(speed*3.6, 35.)

    def test_distant_leader_keeps_full_cruise_acceleration_when_stopping_room_exists(self):
        ego = dict(self.ego, speed_kmh=60.)
        model = self.planner.longitudinal
        model.sensor_age_s = .12
        leader = tuple(list(self.car(300., speed=50./3.6))[:3] +
                       ['lidar_car'] + list(self.car(300., speed=50./3.6))[4:])
        empty = model.preview(self.path, ego, (), 120./3.6)
        distant = model.preview(self.path, ego, [leader], 120./3.6)
        np.testing.assert_allclose(distant.speeds[distant.times<=1.6],
                                   empty.speeds[empty.times<=1.6])
        self.assertGreater(distant.reference_mps*3.6, 68.)
        implied_acceleration = FOLLOWER_ACCEL_GAIN * (
            distant.reference_mps-ego['speed_kmh']/3.6) * FOLLOWER_FULL_ACCEL_MPS2
        # The follower's full-pedal model equals the trajectory's highway
        # acceleration assumption (2026-10-10: full throttle on the highway).
        self.assertLessEqual(implied_acceleration, HIGHWAY_ACCEL_MPS2+1e-9)

    def test_actual_moving_source_age_changes_approach_but_empty_road_stays_free(self):
        model = self.planner.longitudinal
        ego = dict(self.ego, speed_kmh=80.)
        leader = list(self.car(95., speed=50./3.6)); leader[3] = 'lidar_car'
        recent = model.preview(self.path, ego, [tuple(leader)], 120./3.6)
        model.cache.clear()
        model.sensor_age_s = .6
        delayed = model.preview(self.path, ego, [tuple(leader)], 120./3.6)
        self.assertLess(delayed.speeds[-1], recent.speeds[-1])
        delayed_free = model.preview(self.path, ego, (), 120./3.6)
        model.sensor_age_s = 0.
        free = model.preview(self.path, ego, (), 120./3.6)
        np.testing.assert_allclose(delayed_free.speeds, free.speeds)

    def test_120_cruise_approaches_then_matches_slow_leader_without_stops(self):
        planner = HighwayPlanner(dict(from_idx=0, to_idx=900, kmh=120.,
            lane_change_policy='free_three_lane'), self.path)
        speed, acceleration, position = 80./3.6, 0., 0.
        leader_x, leader_speed = 180., 50./3.6
        references, gaps, speeds, emergencies = [], [], [], []
        emitted = speed
        for i in range(400):
            ego = dict(self.ego, x=position, speed_kmh=speed*3.6, stamp=100.+i*.1)
            leader = list(self.car(leader_x, speed=leader_speed)); leader[3] = 'lidar_car'
            planner.longitudinal.begin(ego)
            planner.longitudinal.sensor_age_s = .1
            command = planner.plan_speed(self.path, ego, [tuple(leader)], 120./3.6,
                static_clear_m=float('inf'), validation_stamp=ego['stamp']-.1)
            # The C++ command ramp rises by at most 3 m/s². Its pedal gains,
            # 0.15 m/s deadband and 0.25 s plant lag remain explicit assumptions.
            emitted = min(command['reference_mps'], emitted+3.*.1)
            error = emitted-speed
            throttle = min(1., max(0., .35*error)) if error>.15 else 0.
            brake = min(1., max(0., -.45*error)) if error<-.15 else 0.
            acceleration += .1/.25*(3.*throttle-8.*brake-acceleration)
            speed = max(0., speed+acceleration*.1)
            position += speed*.1
            leader_x += leader_speed*.1
            references.append(command['reference_mps']*3.6)
            gaps.append(leader_x-position-EGO_FRONT_M-2.5)
            speeds.append(speed*3.6)
            emergencies.append(command['emergency'])
        self.assertFalse(any(emergencies))
        self.assertGreater(min(references), 40.)
        self.assertGreater(max(speeds), 90.)
        self.assertLessEqual(max(speeds), 120.)
        self.assertGreater(min(gaps), 5.)
        self.assertLess(np.max(np.abs(np.diff(references))), 3.)
        self.assertAlmostEqual(speed*3.6, 50., delta=1.)

    def test_rear_headway_cannot_hide_an_independent_crossing(self):
        self.install_rear_motion()
        ego = dict(self.ego, speed_kmh=72.)
        rear = self.car(-10.)
        crossing = (30., 5., .6, 'crossing', 'pedestrian', 3.,
                    .6, .6, 90., 0., -3., 2)
        trajectory = self.planner.longitudinal.preview(self.path, ego,
                                                       [rear, crossing], 20.).prefix(3.)
        kwargs = dict(lane_change=True, speed_trajectory=trajectory,
            object_positions=self.planner.longitudinal.object_positions,
            object_yaws=self.planner.longitudinal.object_yaws)
        lane_risk = timed_gap(self.path, ego, [rear, crossing], 20., **kwargs)
        speed_risk = timed_gap(self.path, ego, [rear, crossing], 20.,
                               ignore_rear_approach=True, **kwargs)
        self.assertEqual(lane_risk['object_id'], 1)
        self.assertTrue(lane_risk['rear_approach'])
        self.assertEqual(speed_risk['object_id'], 2)
        # Independent axis-aligned bumper intervals determine the actual
        # contact sample; the longer real front reaches this crossing sooner.
        crossing_y=crossing[1]+trajectory.times*crossing[10]
        physical=((trajectory.distances+EGO_FRONT_M>=crossing[0]-.3) &
                  (trajectory.distances-EGO_REAR_M<=crossing[0]+.3) &
                  (np.abs(crossing_y)<=EGO_HALF_WIDTH_M+.3))
        contact=trajectory.times[np.flatnonzero(physical)[0]]
        self.assertAlmostEqual(speed_risk['physical_collision_when'],contact)
        self.assertLessEqual(speed_risk['collision_when'],contact)
        for objects in ([rear, crossing], [crossing, rear]):
            result = self.planner.plan_speed(self.path, ego, objects, 20.,
                                            static_clear_m=float('inf'))
            self.assertTrue(result['emergency'] or result['reference_mps'] < 20.)

    def test_current_rear_body_overlap_is_not_filtered(self):
        self.install_rear_motion()
        ego = dict(self.ego, speed_kmh=72.)
        rear = self.car(-3.)
        native = self.planner.longitudinal.motion_paths[1]
        self.planner.longitudinal.motion_paths[1] = (*native[:2], 197., *native[3:])
        result = self.planner.plan_speed(self.path, ego, [rear], 20.,
                                        static_clear_m=float('inf'))
        self.assertTrue(result['emergency'])
        self.assertEqual(result['diagnostics']['risk']['object_id'], 1)

    def test_only_distinct_confirmed_clear_scans_release_a_goal_hold(self):
        self.planner.plan_speed(self.path, self.ego, (), 15.,
                                static_clear_m=float('inf'))
        def recover(stamp, source):
            return self.planner.plan_speed(self.path, dict(self.ego, stamp=stamp),
                (), 25., static_clear_m=float('inf'), validation_stamp=source)
        first = recover(100.1, 100.1)
        reused = recover(100.3, 100.1)
        self.assertTrue(first['diagnostics']['held'])
        self.assertTrue(reused['diagnostics']['held'])
        cleared = recover(100.4, 100.4)
        self.assertFalse(cleared['diagnostics']['held'])
        self.assertTrue(cleared['diagnostics']['hold_released'])
        self.assertEqual(cleared['diagnostics']['goal_kmh'], 90.)

    def test_unsafe_scan_resets_early_hold_release_confirmation(self):
        self.planner.plan_speed(self.path, self.ego, (), 15.,
                                static_clear_m=float('inf'))
        self.planner.plan_speed(self.path, dict(self.ego, stamp=100.1), (), 25.,
            static_clear_m=float('inf'), validation_stamp=100.1)
        blocked = self.planner.plan_speed(self.path, dict(self.ego, stamp=100.2), (), 25.,
            static_clear_m=15., validation_stamp=100.2)
        self.assertFalse(blocked['diagnostics'].get('hold_released', False))
        self.assertTrue(blocked['emergency'] or blocked['reference_mps'] <= 30./3.6)
        first_clear = self.planner.plan_speed(self.path, dict(self.ego, stamp=100.3), (), 25.,
            static_clear_m=float('inf'), validation_stamp=100.3)
        self.assertFalse(first_clear['diagnostics'].get('hold_released', False))


if __name__ == '__main__':
    unittest.main()
