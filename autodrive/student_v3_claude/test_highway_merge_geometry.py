"""Replay the empty highway merge that previously imposed a 35 km/h path cap."""
import unittest
import math
from unittest.mock import Mock, patch

import numpy as np

import test_highway_three_lane as fixture
from highway_planner import (HighwayPlanner, local_points, world_points, geometry_speed_cap,
                             path_curvature)
from lane_corridor import LaneCorridor
from planning.highway.lane_map import HighwayLaneMap, arc
from planning.highway.merge_geometry import gentle_merge_geometry, recover_merge_tracking


class HighwayMergeGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.SurveyedHighwayTests.setUpClass()
        cls.survey = fixture.SurveyedHighwayTests()

    def test_recorded_merge_rounds_corner_inside_original_surveyed_corridor(self):
        for y in (-185., -205., -214.):
            ego = dict(x=75. if y < -205. else 74., y=y, heading_deg=-90.,
                       speed_kmh=70., stamp=100.)
            index = int(np.argmin(np.linalg.norm(self.survey.route-[ego['x'], ego['y']], axis=1)))
            _, _, native, _ = self.survey.frame(index)
            route = local_points(self.survey.smooth[index:index+640], ego)
            lanes = native.candidates(ego, route)
            lane = next(lane for lane in lanes if lane.link_id == 'A2256W000445')
            corridor = native.corridor(ego, route, lanes)
            self.assertLessEqual(np.linalg.norm(lane.centre-lane.surveyed_centre, axis=1).max(), .600001)
            # A lane primitive is a future geometric tail beside the ego;
            # actual selected joins are checked with the measured t0 below.
            self.assertGreaterEqual(corridor.clear_distance(lane.centre, 170., include_current=False), 169.9)
            self.assertGreaterEqual(geometry_speed_cap(lane.centre, 25.)*3.6, 70.)
            if y <= -205.:
                self.assertLess(geometry_speed_cap(lane.surveyed_centre, 25.)*3.6, 40.)

    def test_bounds_endpoints_and_distant_road_remain_original(self):
        _, _, native, _ = self.survey.frame(3000)
        _, original, merge = native.chain('A2256W000445')
        proposed = gentle_merge_geometry(original, merge)
        np.testing.assert_array_equal(proposed[[0, -1]], original[[0, -1]])
        station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(original, axis=0), axis=1))]
        at = int(np.argmin(np.linalg.norm(original-merge, axis=1)))
        far = abs(station-station[at]) >= 45.
        np.testing.assert_array_equal(proposed[far], original[far])

    def test_road_bounds_are_not_recentered_on_smoothing_proposal(self):
        _, route, native, lanes = self.survey.frame(3060)
        ego, _, _, _ = self.survey.frame(3060)
        checked = native.corridor(ego, route, lanes)
        for lane in lanes:
            lane.centre = lane.centre+[0., 15.]
        after = native.corridor(ego, route, lanes)
        np.testing.assert_array_equal(after.left, checked.left)
        np.testing.assert_array_equal(after.right, checked.right)

    def test_followed_merge_survives_seed_handoff_with_fresh_road_checks(self):
        native = HighwayLaneMap(self.survey.lanes, self.survey.config)
        _, surveyed, merge = native.chain('A2256W000445')
        proposed = gentle_merge_geometry(surveyed, merge)
        at = int(np.argmin(abs(proposed[:, 1]+205.)))
        position = proposed[at].copy()
        tangent = proposed[at+1]-proposed[at-1]
        heading = math.atan2(tangent[1], tangent[0])
        planner = HighwayPlanner(self.survey.config, self.survey.route)
        planner.completed, planner.phase = 2, 'CRUISE'
        seen, goals = set(), []
        for frame in range(65):
            ego = dict(x=position[0], y=position[1], heading_deg=math.degrees(heading),
                       speed_kmh=70., stamp=100.+frame*.05)
            index = int(np.argmin(np.linalg.norm(self.survey.route-position, axis=1)))
            route = local_points(self.survey.smooth[index:index+640], ego)
            lanes = native.candidates(ego, route)
            corridor = native.corridor(ego, route, lanes)
            result = planner.decide(index, route, ego, [], {}, corridor.clear_distance,
                lambda path, horizon: horizon, lane_options=lanes, motion_options=[])
            self.assertFalse(result['emergency'], index)
            self.assertGreaterEqual(corridor.clear_distance(result['path'], 170.), 169.9, index)
            for lane in lanes:
                self.assertLessEqual(np.linalg.norm(lane.centre-lane.surveyed_centre, axis=1).max(), .600001)
            if 3199 <= index <= 3207:
                seen.add(index)
                goals.append(result['target_mps']*3.6)
            path = world_points(result['path'], ego)
            stations = arc(path)
            advance = 70./3.6*.05
            position = np.array([np.interp(advance, stations, path[:, axis]) for axis in (0, 1)])
            ahead = np.array([np.interp(advance+.5, stations, path[:, axis]) for axis in (0, 1)])
            heading = math.atan2(*(ahead-position)[::-1])
        self.assertTrue(seen and min(seen) <= 3201 and max(seen) >= 3205, seen)
        # This checks continuity of an already legal empty-road path; traffic
        # safety and curvature caps remain free to lower the requested speed.
        # The 5 km/h bound was set at 3.5 m/s² lateral acceleration; curve
        # caps scale with its square root.
        from highway_planner import HIGHWAY_CURVE_ALAT_MPS2
        self.assertLess(max(goals)-min(goals), 5.*math.sqrt(HIGHWAY_CURVE_ALAT_MPS2/3.5), goals)

    def test_handoff_never_changes_native_actor_geometry(self):
        native = HighwayLaneMap(self.survey.lanes, self.survey.config)
        _, surveyed, merge = native.chain('A2256W000445')
        proposed = gentle_merge_geometry(surveyed, merge)
        for frame, y in enumerate((-217., -218., -219.)):
            at = int(np.argmin(abs(proposed[:, 1]-y)))
            ego = dict(x=proposed[at, 0], y=proposed[at, 1], heading_deg=-90.,
                       speed_kmh=70., stamp=100.+frame*.1)
            index = int(np.argmin(np.linalg.norm(self.survey.route-proposed[at], axis=1)))
            native.candidates(ego, local_points(self.survey.smooth[index:index+640], ego))
        at = int(np.argmin(abs(surveyed[:, 1]+205.)))
        obj = fixture.ThreeLaneTests.car(*surveyed[at], 10.)
        motion = native.motion_candidates(ego, [obj])
        self.assertTrue(motion)
        for lane in motion:
            _, original, _ = native.chain(lane.link_id)
            np.testing.assert_allclose(world_points(lane.centre, ego), original, atol=1e-12)

    def test_recorded_tracking_error_at_common_entry_keeps_same_road_bounds(self):
        # Run 907403: no traffic; the rounded A445 entry and measured pose
        # made the original checked road distance jump 170 -> 2 -> 170 m.
        poses = ((74.63, -213.55, -88.53, 76.5),
                 (74.65, -216.76, -89.89, 75.6),
                 (74.67, -217.99, -90.44, 74.7))
        before, after = [], []
        bounds = []
        from gui import highway_nudge
        for enabled, distances in ((False, before), (True, after)):
            native = HighwayLaneMap(self.survey.lanes, self.survey.config)
            planner = HighwayPlanner(self.survey.config, self.survey.route)
            planner.completed, planner.phase = 2, 'CRUISE'
            for frame, (x, y, heading, speed) in enumerate(poses):
                ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed, stamp=100.+frame*.1)
                index = int(np.argmin(np.linalg.norm(self.survey.route-[x, y], axis=1)))
                route = local_points(self.survey.smooth[index:index+640], ego)
                if enabled:
                    lanes = native.candidates(ego, route)
                else:
                    with patch('planning.highway.lane_map.recover_merge_tracking',
                               side_effect=lambda surveyed, proposed, *a, **k: proposed.copy()):
                        lanes = native.candidates(ego, route)
                corridor = native.corridor(ego, route, lanes)
                if not enabled:
                    bounds.append((corridor.left.copy(), corridor.right.copy()))
                else:
                    np.testing.assert_array_equal(corridor.left, bounds[frame][0])
                    np.testing.assert_array_equal(corridor.right, bounds[frame][1])
                result = planner.decide(index, route, ego, [], {}, corridor.clear_distance,
                    lambda path, horizon: horizon, lane_options=lanes, motion_options=[],
                    path_transform=lambda path: highway_nudge(path, np.empty((0, 2)), None, speed/3.6))
                # The ordinary policy rejoins the candidate with measured
                # heading, and Runtime's final geometry transform runs too.
                distances.append(corridor.clear_distance(result['path'], 170.))
                if enabled:
                    self.assertLessEqual(np.max(path_curvature(result['path'])),
                                         math.tan(math.radians(35.))/3.9)
                    if frame < 2:
                        self.assertGreaterEqual(geometry_speed_cap(result['path'], 120./3.6)*3.6, 70.)
        self.assertGreaterEqual(before[0], 169.9)
        # The actual rear-axle body and measured-pose join also keep the
        # unrepaired curve clear; the former centred footprint falsely stopped.
        self.assertEqual(before, [170.]*len(poses))
        self.assertGreaterEqual(before[2], 169.9)
        self.assertTrue(all(distance >= 169.9 for distance in after), after)

    def test_merge_recovery_does_not_open_narrow_or_prohibited_road(self):
        route = np.column_stack((np.arange(0., 180.01, .5), np.zeros(361)))
        for corridor in (LaneCorridor(route, np.ones(361), -np.ones(361)),
                         LaneCorridor(route, np.full(361, 1.75), np.full(361, -1.75),
                            zones=[[[2., -3.], [8., -3.], [8., 3.], [2., 3.]]])):
            original_left, original_right = corridor.left.copy(), corridor.right.copy()
            candidate = recover_merge_tracking(route, route, 75./3.6, corridor.clear_distance)
            np.testing.assert_array_equal(candidate, route)
            np.testing.assert_array_equal(corridor.left, original_left)
            np.testing.assert_array_equal(corridor.right, original_right)
            self.assertLess(corridor.clear_distance(candidate, 170.), 12.)

    def test_recorded_earlier_entry_uses_a_gentler_checked_tracking_repair(self):
        # Run 984404 hit the same native seam earlier: a fixed 6 m repair
        # passed the road but imposed 68.3 km/h and was correctly rejected.
        ego = dict(x=74.49, y=-212.14, heading_deg=-87.36, speed_kmh=78.8, stamp=100.)
        index = int(np.argmin(np.linalg.norm(self.survey.route-[ego['x'], ego['y']], axis=1)))
        route = local_points(self.survey.smooth[index:index+640], ego)
        results = []
        for enabled in (False, True):
            native = HighwayLaneMap(self.survey.lanes, self.survey.config)
            if enabled:
                lanes = native.candidates(ego, route)
            else:
                with patch('planning.highway.lane_map.recover_merge_tracking',
                           side_effect=lambda surveyed, proposed, *a, **k: proposed.copy()):
                    lanes = native.candidates(ego, route)
            corridor = native.corridor(ego, route, lanes)
            planner = HighwayPlanner(self.survey.config, self.survey.route)
            planner.completed, planner.phase = 2, 'CRUISE'
            decision = planner.decide(index, route, ego, [], {}, corridor.clear_distance,
                lambda path, horizon: horizon, lane_options=lanes, motion_options=[])
            results.append((corridor.clear_distance(decision['path'], 170.),
                            geometry_speed_cap(decision['path'], 120./3.6)*3.6))
        self.assertLess(results[0][0], 6.)
        self.assertGreaterEqual(results[1][0], 169.9)
        self.assertGreaterEqual(results[1][1], 70.)

    def test_clear_merge_skips_extra_whole_horizon_search(self):
        route = np.column_stack((np.arange(0., 180.01, .5), np.zeros(361)))
        clearance = Mock(side_effect=lambda path, horizon: horizon)
        np.testing.assert_array_equal(recover_merge_tracking(route, route, 75./3.6, clearance), route)
        self.assertEqual(clearance.call_count, 1)
        self.assertEqual(clearance.call_args.args[1], 12.)

    def test_invalid_speed_cannot_authorize_a_tracking_repair(self):
        route = np.column_stack((np.arange(0., 180.01, .5), np.zeros(361)))
        for speed in (float('nan'), float('inf'), -float('inf'), -1., None):
            with self.subTest(speed=speed):
                clearance = Mock(side_effect=AssertionError('invalid-speed repair query'))
                candidate = recover_merge_tracking(route, route, speed, clearance)
                np.testing.assert_array_equal(candidate, route)
                clearance.assert_not_called()


if __name__ == '__main__':
    unittest.main()
