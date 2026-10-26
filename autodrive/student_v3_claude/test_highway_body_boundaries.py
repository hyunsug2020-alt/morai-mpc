"""Highway body placement at shared static/map boundaries and lane admission."""
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

from ego_body import EGO_CENTRE_M, EGO_FRONT_M, EGO_REAR_M
from gui import static_clear_distance
from lane_corridor import LaneCorridor
from highway_planner import HighwayPlanner, lane_change_path
from highway_planner import timed_gap
from highway_speed_plan import clearance_forecast_bound
from planning.highway.trajectory_geometry import normalize_path, sample_pose
from highway_three_lane import HighwayLane, ThreeLanePolicy
from test_highway_rear_axle import ego_polygon, vehicle_polygon, polygons_overlap
from planning.highway.body_geometry import (HighwayLaneCorridor,
    highway_boxed_clearance, highway_raw_clearance, highway_side_gaps)


def box(x, y=0., length=.4, width=.4, yaw=0., kind='static'):
    return (x, y, max(length, width), 'box', kind, 0., length, width, yaw,
            0., 0., 1, 0., None, True, 0.)


class HighwayStaticBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 301., .25), np.zeros(1204)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)

    def raw(self, points, path=None):
        points = np.asarray(points).reshape(-1, 2)
        return highway_raw_clearance(self.path if path is None else path, points, 170.,
            extra_margin=.25, lateral_margin=.15,
            tree=cKDTree(points) if len(points) and np.isfinite(points).all() else None)

    def test_current_front_and_rear_points_have_actual_bumper_limits(self):
        for x in (EGO_FRONT_M+.25-1e-9, -EGO_REAR_M-.25+1e-9):
            self.assertEqual(self.raw([[x, 0.]]), 0.)
        self.assertGreater(self.raw([[EGO_FRONT_M+.25+1e-9, 0.]]), 0.)
        self.assertEqual(self.raw([[-EGO_REAR_M-.25-1e-9, 0.]]), 170.)

    def test_proposed_initial_heading_cannot_hide_the_actual_front_point(self):
        vertical = self.path[:, ::-1]
        self.assertEqual(self.raw([[EGO_FRONT_M, 0.]], vertical), 0.)

    def test_current_rear_box_survives_shared_rear_object_skip(self):
        obstacles = [box(-.85)]
        self.assertEqual(static_clear_distance(self.path, self.ego, obstacles, 170.,
                                               body_shift_m=EGO_CENTRE_M), 170.)
        self.assertEqual(highway_boxed_clearance(self.path, self.ego, obstacles, 170.), 0.)

    def test_current_box_uses_world_pose_and_observed_rotation(self):
        ego = dict(self.ego, x=10., y=20., heading_deg=90.)
        obstacles = [box(10., 24., yaw=90.)]
        self.assertEqual(highway_boxed_clearance(self.path, ego, obstacles, 170.), 0.)

    def test_future_static_bound_is_axle_station_and_empty_query_completes(self):
        obstacles = [box(20.)]
        actual = highway_boxed_clearance(self.path, self.ego, obstacles, 170.)
        self.assertGreaterEqual(actual, 20.-EGO_FRONT_M-.2-.2)
        self.assertLess(actual, 20.-EGO_FRONT_M-.2-.2+.25)
        self.assertEqual(self.raw([]), 170.)
        self.assertEqual(highway_boxed_clearance(self.path, self.ego, [], 170.), 170.)

    def test_invalid_geometry_never_becomes_clear(self):
        self.assertEqual(self.raw([[float('nan'), 0.]]), 0.)
        self.assertEqual(self.raw([], np.zeros((4, 2))), 0.)
        self.assertEqual(highway_boxed_clearance(self.path, self.ego,
            [box(float('nan'))], 170.), 0.)

    def test_endpoint_raw_and_box_contacts_are_not_query_completion(self):
        horizon = 10.
        raw = highway_raw_clearance(self.path,
            np.array([[horizon+EGO_FRONT_M+.25, 0.]]), horizon)
        boxed = highway_boxed_clearance(self.path, self.ego,
            [box(horizon+EGO_FRONT_M+.2+.2-1e-9)], horizon)
        for bound in (raw, boxed):
            self.assertGreater(bound, horizon-.001)
            self.assertTrue(np.isfinite(clearance_forecast_bound(bound, horizon)))
        self.assertEqual(clearance_forecast_bound(
            highway_raw_clearance(self.path, np.empty((0, 2)), horizon), horizon),
            float('inf'))

    def test_side_gaps_include_points_beside_actual_front(self):
        points = np.array([[3.7, 1.2], [3.7, -1.5]])
        _, left, right = highway_side_gaps(self.path, points, cKDTree(points))
        self.assertAlmostEqual(left[0], 1.2-.946)
        self.assertAlmostEqual(right[0], 1.5-.946)


class HighwayCorridorBoundaryTests(unittest.TestCase):
    def setUp(self):
        x = np.arange(-10., 301., .25)
        self.route = np.column_stack((x, np.zeros(len(x))))
        self.path = self.route[self.route[:, 0] >= 0.]

    def corridor(self, zones=()):
        return HighwayLaneCorridor(self.route, np.full(len(self.route), 4.),
            np.full(len(self.route), -4.), zones=list(zones))

    def test_clone_changes_only_highway_body_ownership(self):
        original = LaneCorridor(self.route, np.full(len(self.route), 4.),
                                np.full(len(self.route), -4.))
        copied = HighwayLaneCorridor.from_corridor(original)
        self.assertEqual(original.body_shift_m, 0.)
        self.assertEqual(copied.body_shift_m, EGO_CENTRE_M)
        self.assertIs(copied.tree, original.tree)
        self.assertEqual(copied.clear_distance(self.path, 170.), 170.)

    def test_no_go_inside_current_front_body_is_blocked(self):
        tiny = np.array([[3.5, -.05], [3.6, -.05], [3.6, .05], [3.5, .05]])
        self.assertEqual(self.corridor([tiny]).clear_distance(self.path, 170.), 0.)

    def test_no_go_crossing_edges_is_blocked_without_corner_inside(self):
        stripe = np.array([[3.4, -3.], [3.5, -3.], [3.5, 3.], [3.4, 3.]])
        self.assertEqual(self.corridor([stripe]).clear_distance(self.path, 170.), 0.)

    def test_tail_does_not_check_or_prepend_current_pose(self):
        tiny = np.array([[3.5, -.05], [3.6, -.05], [3.6, .05], [3.5, .05]])
        corridor = self.corridor([tiny])
        tail = self.path[self.path[:, 0] >= 10.]
        self.assertEqual(corridor.clear_distance(tail, 170.), 0.)
        self.assertEqual(corridor.clear_distance(tail, 170., include_current=False), 170.)

    def test_endpoint_no_go_touch_remains_a_finite_bound(self):
        front = 10.+EGO_FRONT_M-1e-9
        zone = np.array([[front, -.05], [front+.1, -.05],
                         [front+.1, .05], [front, .05]])
        bound = self.corridor([zone]).clear_distance(self.path, 10.)
        self.assertGreater(bound, 9.999)
        self.assertTrue(np.isfinite(clearance_forecast_bound(bound, 10.)))


class HighwayStartPoseTests(unittest.TestCase):
    def test_initial_behind_vertices_cannot_create_reverse_motion(self):
        route = np.column_stack((np.arange(-1., 30., .5), np.full(62, .3)))
        for path in (route, np.vstack(([0., 0.], route)),
                     lane_change_path(route, 0., 12., match_heading=True)):
            normalized, stations = normalize_path(path)
            np.testing.assert_array_equal(normalized[0], [0., 0.])
            self.assertTrue(np.all(normalized[1:, 0] >= 0.))
            self.assertTrue(np.all(np.diff(stations) > 0.))

    def test_duplicate_origins_do_not_hide_a_backward_prefix(self):
        path, _ = normalize_path([[0., 0.], [0., 0.], [-.5, 0.], [1., 0.], [2., 0.]])
        np.testing.assert_array_equal(path, [[0., 0.], [1., 0.], [2., 0.]])

    def test_centimetre_tracking_errors_rejoin_measured_heading(self):
        for offset in (.1, .3, .35, 1.5):
            route = np.column_stack((np.arange(.001, 150., .5), np.full(300, offset)))
            path = lane_change_path(route, 0., 50., match_heading=True)
            normalized, stations = normalize_path(path)
            _, heading = sample_pose(normalized, stations, np.array([0., .25]), 0.)
            self.assertEqual(heading[0], 0.)
            self.assertLess(abs(np.degrees(heading[1])), .01)


class HighwayFinalPublicationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from test_highway_unmeasured_vehicles import MeasuredSlowCarsStillBrake
        MeasuredSlowCarsStillBrake.setUpClass()
        cls.scene = MeasuredSlowCarsStillBrake

    def test_invalid_final_paths_publish_stop_without_claiming_validation(self):
        from test_highway_unmeasured_vehicles import RuntimeHarness
        scene = self.scene
        x, y = scene.world[2800]
        ego = dict(x=x, y=y, heading_deg=-90., speed_kmh=30., stamp=100.)
        invalid = (np.full((4, 2), np.nan), np.zeros((4, 2)),
                   np.array([[-4., 0.], [-3., 0.], [-2., 0.], [-1., 0.]]),
                   np.array([[0., 0.]]))
        for path in invalid:
            with self.subTest(path=path.tolist()):
                harness = RuntimeHarness(scene.lanes, scene.world, scene.smooth)
                try:
                    runtime = harness.runtime
                    runtime._web_record = None
                    decide = runtime.highway.decide
                    def malformed(*args, **kwargs):
                        return dict(decide(*args, **kwargs), path=path)
                    with patch.object(runtime.highway, 'decide', side_effect=malformed), \
                            patch.object(runtime.path_publisher, 'publish_stop') as stop, \
                            patch.object(runtime.path_publisher, 'publish') as publish:
                        harness.step(ego, [], 99.95)
                    stop.assert_called_once_with('invalid highway final path')
                    publish.assert_not_called()
                    self.assertTrue(runtime.path_publisher.emergency_stop)
                    self.assertIsNone(runtime.last_result)
                finally:
                    harness.close()

    def test_inward_escape_survives_canonical_freeze_and_publication(self):
        from types import SimpleNamespace
        import json
        from test_highway_unmeasured_vehicles import RuntimeHarness
        ego = dict(x=75.28, y=-38.3, heading_deg=-90., speed_kmh=0., stamp=100.)
        scene = self.scene
        harness = RuntimeHarness(scene.lanes, scene.world, scene.smooth)
        payloads = []
        harness.runtime.path_publisher.waypoints_pub = SimpleNamespace(
            publish=lambda message: payloads.append(json.loads(message.data)))
        try:
            record = harness.step(ego, [], 99.95)
            runtime = harness.runtime
            info = record['highway']
            self.assertTrue(info['low_speed_alignment_recovery'])
            self.assertTrue(info['alignment_escape_join'])
            self.assertFalse(record['emergency_stop'])
            self.assertGreater(record['planner_cap_kmh'], 0.)
            self.assertLessEqual(record['planner_cap_kmh'], 4.+1e-6)
            path = runtime.last_result['path']
            self.assertTrue(runtime.last_result['postprocess']['canonical_control'])
            np.testing.assert_array_equal(path, path.astype(np.float32).astype(float))
            emitted = np.array([[row['x'], row['y']] for row in payloads[-1]['waypoints']])
            expected = np.array([runtime.path_publisher.global_xy(ego, *point) for point in path])
            np.testing.assert_array_equal(emitted, expected)
            self.assertEqual(len(emitted), len(path))
        finally:
            harness.close()


class HighwayCommandAdmissionTests(unittest.TestCase):
    def candidate(self, rear_x, rear_speed, goal_kmh):
        x = np.arange(0., 501., .5)
        route = np.column_stack((x, np.zeros(len(x))))
        config = dict(from_idx=0, to_idx=900, kmh=goal_kmh, lane_change_policy='free_three_lane',
                      lane_change_goal=0, comfort_decel_mps2=2.5, comfort_jerk_mps3=4.)
        planner = HighwayPlanner(config, route)
        policy = ThreeLanePolicy(config)
        lane = HighwayLane('adjacent', 1, route+[0., 3.5], ('adjacent', 'exit'), 500.)
        line = np.column_stack((np.arange(-100., 501.), np.full(601, 3.5)))
        planner.longitudinal.motion_paths = {1: (line, np.arange(601.),
                                                 100.+rear_x, rear_speed, 0., 0.)}
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=10.)
        obj = (rear_x, 3.5, 4.9, 'lidar_car', 'npc', rear_speed,
               4.9, 2.1, 0., rear_speed, 0., 1, 0., None, True, 0.)
        path = lane_change_path(lane.centre, 0., 45., match_heading=True)
        return policy.evaluate(planner, path, ego, [obj], lambda p, h: h,
            lambda p, h: h, changing=True, destinations=[lane], transition=45.,
            goal_limit=goal_kmh/3.6, admission=True)

    def test_real_rear_room_can_admit_a_command_safe_change(self):
        result, check = self.candidate(-9., 60./3.6, 60.)
        self.assertIsNotNone(result, check)
        self.assertEqual(check['command_check_horizon_s'], 3.)
        self.assertGreaterEqual(check['command_gap_checks'], 1)

    def test_nominal_safe_acceleration_is_vetoed_by_actual_command_rear_gap(self):
        result, check = self.candidate(-25., 23., 100.)
        self.assertIsNone(result)
        self.assertEqual(check['command_check_horizon_s'], 3.)
        self.assertIsNone(check['gap_risk']['physical_collision_when'])
        self.assertTrue(check['gap_risk']['rear_approach'])

    def test_rear_contact_risk_can_release_a_slow_hold_only_with_a_safe_command(self):
        route = np.column_stack((np.arange(0., 1000.), np.zeros(1000)))
        config = dict(from_idx=0, to_idx=900, kmh=120., lane_change_policy='free_three_lane',
                      comfort_decel_mps2=2.5, comfort_jerk_mps3=4.)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=10.)
        rear = (-12., 0., 4.9, 'lidar_car', 'npc', 20., 4.9, 2.1, 0.,
                20., 0., 1, 0., None, True, 0.)
        planner = HighwayPlanner(config, route)
        planner._speed_hold_goal, planner._speed_hold_until = 0., 20.
        planner._speed_hold_stamp = 9.9
        command = planner.plan_speed(route, ego, [rear], 30., static_clear_m=float('inf'))
        self.assertFalse(command['emergency'])
        self.assertTrue(command['diagnostics']['rear_escape'])
        self.assertGreater(command['reference_mps'], 60./3.6)
        self.assertLessEqual(command['reference_mps'], 30.)
        trajectory = planner.longitudinal.preview(route, ego, [rear], 30., 3.)
        from test_highway_speed_recovery import HighwaySpeedRecoveryTests
        execution = HighwaySpeedRecoveryTests.command_motion(trajectory)
        for time, distance in zip(execution.times, execution.distances):
            other = list(rear)
            other[0] += 20.*time
            self.assertFalse(polygons_overlap(ego_polygon(dict(ego, x=distance)),
                                              vehicle_polygon(other)))

    def test_front_contact_cannot_be_waived_as_a_rear_escape(self):
        route = np.column_stack((np.arange(0., 1000.), np.zeros(1000)))
        planner = HighwayPlanner(dict(from_idx=0, to_idx=900, kmh=120.,
            lane_change_policy='free_three_lane'), route)
        planner._speed_hold_goal, planner._speed_hold_until = 0., 20.
        planner._speed_hold_stamp = 9.9
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=10.)
        touching = (6., 0., 4.9, 'lidar_car', 'npc', 0., 4.9, 2.1, 0.,
                    0., 0., 1, 0., None, True, 0.)
        self.assertTrue(polygons_overlap(ego_polygon(ego), vehicle_polygon(touching)))
        command = planner.plan_speed(route, ego, [touching], 30., static_clear_m=float('inf'))
        self.assertTrue(command['emergency'])
        self.assertEqual(command['reference_mps'], 0.)


class HighwayBrakingReferenceTests(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 1000.), np.zeros(1000)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=80., stamp=100.)
        self.planner = HighwayPlanner(dict(from_idx=0, to_idx=900, kmh=120.,
            lane_change_policy='free_three_lane'), self.path)
        self.front = (60.2, 0., 4.9, 'lidar_car', 'npc', 8./3.6,
                      4.9, 2.1, 0., 8./3.6, 0., 1, 0., None, True, 0.)

    def plan(self, objects=None, **kwargs):
        return self.planner.plan_speed(self.path, self.ego,
            [self.front] if objects is None else objects, 30., **kwargs)

    def test_safe_brake_reference_matches_actual_plant_and_highest_checked_pace(self):
        from dataclasses import replace
        from test_highway_speed_recovery import independent_command_motion, independent_physical_contacts
        model = self.planner.longitudinal
        originals = []
        preview = model.preview
        def record(*args, **kwargs):
            trajectory = preview(*args, **kwargs)
            originals.append(trajectory)
            return trajectory
        with patch.object(model, 'preview', side_effect=record), \
                patch.object(model, 'command_preview', wraps=model.command_preview) as commands:
            result = self.plan(static_clear_m=float('inf'))
        self.assertFalse(result['emergency'])
        self.assertTrue(result['diagnostics']['braking_reference_extended'])
        self.assertAlmostEqual(result['reference_mps']*3.6, 72.08)
        selected = commands.call_args.args[0]
        self.assertTrue(selected.braking_reference_extended)
        self.assertTrue(selected.prefix(2.).braking_reference_extended)
        self.assertEqual(selected.reference_override_s, result['diagnostics']['reference_time_s'])
        self.assertAlmostEqual(selected.reference_mps, result['reference_mps'])
        self.assertTrue(all(trajectory.reference_override_s is None for trajectory in originals))
        executed = independent_command_motion(selected)
        self.assertLessEqual(-executed.accelerations.min(), 5.)
        self.assertEqual(independent_physical_contacts(self.path, self.ego, [self.front], executed), [])
        np.testing.assert_allclose(model.command_preview(selected).distances,
                                   executed.distances, rtol=0., atol=1e-12)
        nominal = preview(self.path, self.ego, [self.front], 0., decel_mps2=5.)
        safe = []
        for at in np.arange(.45, 3.025, .05):
            candidate = replace(nominal, reference_override_s=float(at))
            command = independent_command_motion(candidate)
            if (-command.accelerations.min() <= 5.+1e-9 and
                    timed_gap(self.path, self.ego, [self.front], candidate.reference_mps,
                        lane_change=False, speed_trajectory=command, ignore_rear_approach=True,
                        footprint_margin_m=.2) is None):
                safe.append(candidate.reference_mps)
        self.assertAlmostEqual(result['reference_mps'], max(safe))

    def test_extension_cannot_waive_current_contact_short_path_or_static_bound(self):
        contact = list(self.front)
        contact[0] = 6.
        self.assertTrue(self.plan([tuple(contact)], static_clear_m=float('inf'))['emergency'])
        self.planner.reset()
        self.path = self.path[:6]
        self.assertTrue(self.plan([], static_clear_m=float('inf'))['emergency'])
        self.setUp()
        self.assertTrue(self.plan([], static_clear_m=3.75)['emergency'])

    def test_front_safe_braking_must_also_keep_the_actual_rear_body_clear(self):
        rear = (-8., 0., 4.9, 'lidar_car', 'npc', 25., 4.9, 2.1, 0.,
                25., 0., 2, 0., None, True, 0.)
        self.assertTrue(self.plan([self.front, rear], static_clear_m=float('inf'))['emergency'])

    def test_model_peak_limit_includes_measured_acceleration_and_keeps_ceiling(self):
        from test_highway_speed_recovery import independent_command_motion
        for acceleration in (-6., -4., 0., 3.):
            self.setUp()
            self.planner.longitudinal.acceleration = acceleration
            with patch.object(self.planner.longitudinal, 'command_preview',
                              wraps=self.planner.longitudinal.command_preview) as commands:
                result = self.plan(static_clear_m=float('inf'), ceiling_mps=24.)
            self.assertLessEqual(result['reference_mps'], 24.)
            if result['diagnostics'].get('braking_reference_extended'):
                selected = commands.call_args.args[0]
                executed = independent_command_motion(selected)
                self.assertLessEqual(-executed.accelerations.min(), selected.braking_limit_mps2+1e-9)


if __name__ == '__main__':
    unittest.main()
