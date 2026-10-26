"""A started lane change must survive its own every-frame recheck.

Run follow-20261009T204445Z (2 laps): 7 cancels of a started change (4 static
relief, one of them re-rooting it; 3 unsafe change). The recheck (3 s horizon,
0.6 m footprint margin, static relief) often cancelled for what the start knew:
a coasting track read as leaving the target lane, a cut-in whose slack vanished
once the slower car sat at its lane centre, or (object policy) a camera
duplicate of a moving car. The start now needs the recheck's clearance plus
hysteresis; the recheck itself is unchanged.
Recorded numbers are in the ego frame of the start frame, the target lane left.
"""
import math
import unittest

import numpy as np

from gui import static_clear_distance
from highway_planner import HighwayPlanner, local_points, world_points
from highway_three_lane import HighwayLane, HighwayLaneMap, ThreeLanePolicy, arc
from planning.highway.object_policy import highway_static_objects
import test_highway_three_lane as surveyed_fixture


class ChangeAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        self.config = dict(from_idx=0, to_idx=1800, kmh=120.,
            lane_change_policy="free_three_lane", lane_change_goal=0, end_link="exit")
        self.planner = HighwayPlanner(self.config, self.route)
        self.lanes = [HighwayLane(str(k), k, self.route+np.array([0., k*3.5]),
                                  (str(k), "exit"), 600.) for k in range(3)]
        behind = np.column_stack((np.arange(-200., 0., .5), np.zeros(400)))
        self.motion = [HighwayLane(l.link_id, -1, np.vstack((behind+l.centre[0], l.centre)),
                                   l.chain, 0.) for l in self.lanes]

    @staticmethod
    def car(x, y, along, uid, length=4.9, width=2.1, yaw=0., lateral=0., valid=True, age=0.):
        return (x, y, max(length, width), 'lidar_car', 'npc', math.hypot(along, lateral),
                length, width, yaw, along, lateral, uid, 0., None, valid, age)

    def ego(self, speed_kmh, stamp=10.):
        return dict(x=0., y=0., heading_deg=0., speed_kmh=speed_kmh, stamp=stamp)

    def decide(self, objects, speed_kmh, static=None, stamp=10.):
        return self.planner.decide(0, self.route[:640], self.ego(speed_kmh, stamp), objects, {},
            lambda p, h: h, static or (lambda p, h: h), perception_ready=True,
            lane_options=self.lanes, motion_options=self.motion)

    def candidate(self, objects, speed_kmh, length, static=None, admission=True, target=1):
        """The recorded single-leg candidate under the start (admission) or recheck test."""
        policy = self.planner.three_lane_policy = ThreeLanePolicy(self.config)
        ego = self.ego(speed_kmh)
        self.planner.longitudinal.begin(ego)
        policy.update_motion_paths(self.planner, self.lanes+self.motion, ego, objects)
        path = policy.sequence_path([self.lanes[target]], length, speed_kmh/3.6)
        return policy.evaluate(self.planner, path, ego, objects, lambda p, h: h,
            static or (lambda p, h: h), changing=True, destinations=[self.lanes[target]],
            transition=length, goal_limit=self.planner.target_mps, admission=admission)

    def static_on(self, objects):
        def clearance(path, horizon):
            return static_clear_distance(path, self.ego(0.), highway_static_objects(objects), horizon)
        return clearance

    def test_recorded_coasting_truck_in_target_lane_does_not_start_an_escape(self):
        # t=494.27 at 67 km/h: current lane hit 52 m ahead; slow truck 500057
        # 45.4 m ahead in the target lane, last measured 0.18 s earlier with a
        # 1.04 m/s exit toward the next lane. The 67 m escape started; 0.18 s
        # later its forecast body had grown to 9.0x4.2 m and a fresh unknown-speed
        # detection of the same truck appeared: "unsafe change" cancel.
        truck = self.car(45.4, 3.5+.36, 2.42, 500057, 7.76, 2.98, -4.2, lateral=1.04, age=.18)
        def static(path, horizon):
            path = np.asarray(path)[:, :2]
            return 52.25 if abs(np.interp(52.25, arc(path), path[:, 1])) < 1.75 else horizon
        recheck, _ = self.candidate([truck], 67., 66.98, static, admission=False)
        self.assertIsNotNone(recheck)
        start, check = self.candidate([truck], 67., 66.98, static)
        self.assertIsNone(start)
        self.assertEqual(check['reason'], 'target-lane body motion not currently observed')
        self.assertEqual(check['target_lane_body_age_s'], .18)
        self.planner = HighwayPlanner(self.config, self.route)
        result = self.decide([truck], 67., static)
        self.assertEqual(result['status']['target_lane'], '0')
        self.assertIsNone(self.planner.saved_world)
        # Freshly measured, the exit is still not assumed: at its lane centre the
        # truck blocks the escape path.
        fresh = self.car(45.4, 3.5+.36, 2.42, 500057, 7.57, 2.79, -4.2, lateral=1.04)
        start, check = self.candidate([fresh], 67., 66.98, static)
        self.assertIsNone(start)
        self.assertTrue(check['gap_risk']['lane_centred_pose'])
        self.assertEqual(check['gap_risk']['object_id'], 500057)

    def test_recorded_cut_in_escape_needs_the_slower_car_at_its_lane_centre(self):
        # t=234.40 at 97 km/h: unknown-speed car 500028 39.1 m ahead in our lane;
        # the 64.7 m escape cut in front of 500027 (11.9 m/s, 8.8 m ahead in the
        # target lane, 0.17 m beyond its centre). The recheck margin had 0.3-0.35 m
        # slack; braking for 500028 left the ego beside 500027 0.59 s later.
        slower = self.car(8.8, 3.5+.17, 11.91, 500027, 5.0, 2.09)
        unknown = self.car(39.1, -.36, 0., 500028, 5.0, 2.01, valid=False)
        objects, static = [slower, unknown], self.static_on([unknown])
        recheck, _ = self.candidate(objects, 97., 64.66, static, admission=False)
        self.assertIsNotNone(recheck)
        start, check = self.candidate(objects, 97., 64.66, static)
        self.assertIsNone(start)
        self.assertEqual(check['reason'], 'lane-change traffic gap occupied')
        self.assertEqual(check['gap_risk']['object_id'], 500027)
        self.assertAlmostEqual(check['gap_risk']['footprint_margin_m'], .9)
        self.planner = HighwayPlanner(self.config, self.route)
        result = self.decide(objects, 97., static)
        self.assertNotIn(result['status']['target_lane'], ('1', '2'))

    def test_centred_wide_truck_remains_overtakable(self):
        # The 8.9x3.1 m truck box of t=220.4 centred in our lane: passing it in
        # the next 3.5 m lane leaves 0.95 m, above the 0.9 m start margin. Its
        # +-0.35 m/+-4.4 deg box jitter can still exceed the 0.6 m recheck margin.
        truck = self.car(32.5, 0., 8.32, 500013, 8.92, 3.14)
        start, check = self.candidate([truck], 43.9, 77.78)
        self.assertIsNotNone(start, check)

    def test_next_leg_recheck_keeps_the_recheck_test(self):
        # First leg done (ego centred in lane '1'); the admitted next leg into
        # '2' is rechecked as a continuation. A car there coasting 0.2 s would
        # refuse a new start but must not cancel the admitted sequence.
        self.lanes = [HighwayLane(str(k), k, self.route+np.array([0., (k-1)*3.5]),
                                  (str(k), "exit"), 600.) for k in range(3)]
        policy = self.planner.three_lane_policy = ThreeLanePolicy(self.config)
        policy.target_chain, policy.change_chains = ('1', 'exit'), [('1', 'exit'), ('2', 'exit')]
        policy.change_length, policy.change_started = 90., 9.
        leg = policy.sequence_path([self.lanes[2]], 90., 25.)
        self.planner.saved_world, self.planner.phase = world_points(leg, self.ego(90.)), 'CHANGE'
        coasting = self.car(60., 3.5, 25., 9, age=.2)
        # A slower car ahead in the reached lane keeps the next leg worth it.
        slower = self.car(80., 0., 15., 11)
        result = self.decide([coasting, slower], 90.)
        self.assertEqual(result['status']['lane_changes'], 1)
        self.assertEqual(result['status']['target_lane'], '2')
        self.assertNotIn('next leg admission blocked', result['status']['continuation_replan_reasons'])
        self.assertIsNotNone(self.planner.saved_world)
        start, check = self.candidate([coasting], 90., 90., target=2)
        self.assertEqual(check['reason'], 'target-lane body motion not currently observed')

    def test_start_is_refused_when_static_relief_would_cancel_it(self):
        relief = ThreeLanePolicy.static_relief_at_start
        speed, reserve = 50./3.6, 106.4
        hit = dict(static_m=100., query_m=170.)
        self.assertTrue(relief(hit, 170., speed, reserve))
        # Not clearer by the 10 m relief margin less hysteresis: no cancel follows.
        self.assertFalse(relief(hit, 104., speed, reserve))
        self.assertFalse(relief(dict(static_m=170., query_m=170.), 170., speed, reserve))
        self.assertFalse(relief(dict(static_m=150., query_m=170.), 170., speed, reserve))
        self.assertTrue(relief(dict(static_m=110., query_m=170.), 170., speed, reserve))

    def test_obstacle_appearing_after_the_start_still_cancels(self):
        # t=491.39->492.04: the clear 144 m entry change into lane 409 met a far
        # LiDAR return (a slow truck's rear, unconfirmed beyond 71 m at the start)
        # that blocked every lane-409 path. The static-relief cancel is unchanged.
        lead = [self.car(70., 0., 15., 7)]
        for stamp in (9.5, 10.):
            result = self.planner.decide(0, self.route[:640], self.ego(90., stamp), lead, {},
                lambda p, h: h, lambda p, h: h, perception_ready=True, lane_options=self.lanes)
        self.assertEqual(result['status']['target_lane'], '1')
        saved = self.planner.saved_world
        def static(path, horizon):
            path = np.asarray(path)[:, :2]
            return 67.5 if np.interp(67.5, arc(path), path[:, 1]) > .5 else horizon
        result = self.planner.decide(0, self.route[:640], self.ego(90., 10.07), lead, {},
            lambda p, h: h, static, perception_ready=True, lane_options=self.lanes)
        self.assertIsNotNone(saved)
        self.assertEqual(result['status']['current_lane_recheck']['cancel_basis'], 'static relief')
        self.assertEqual(result['status']['target_lane'], '0')
        self.assertIsNone(self.planner.saved_world)


class LatestRecordedAdmissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        surveyed_fixture.SurveyedHighwayTests.setUpClass()
        cls.survey = surveyed_fixture.SurveyedHighwayTests()

    def test_recorded_lane_match_stays_on_the_approached_lane(self):
        # Same run, 226.732--227.857 s: ego approaches lane 409 monotonically,
        # but the current-lane label alternated 409/411, including after arrival.
        poses = [(65.73, 126.24, -88.04), (65.79, 124.41, -88.09),
                 (65.86, 122.10, -88.15), (65.91, 120.11, -88.20),
                 (65.97, 118.26, -88.24), (66.15, 111.44, -88.33),
                 (66.40, 102.85, -88.39), (66.61, 95.44, -88.49)]
        native = HighwayLaneMap(self.survey.lanes, self.survey.config)
        policy = ThreeLanePolicy(self.survey.config)
        for n, (x, y, heading) in enumerate(poses):
            with self.subTest(pose=(x, y, heading)):
                ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=110., stamp=100.+n*.1)
                index = int(np.argmin(np.linalg.norm(self.survey.route-[x, y], axis=1)))
                route = local_points(self.survey.smooth[index:index+640], ego)
                lanes = native.candidates(ego, route)
                current = policy.match_current_lane(lanes)
                self.assertEqual(current.link_id, 'A2256W000409',
                    [(lane.link_id, ThreeLanePolicy.lateral(lane), lane.centre[0].tolist())
                     for lane in lanes])
        # A clear movement to the next separate lane must still change the match.
        ego = dict(ego, x=ego['x']+3.5, stamp=ego['stamp']+.1)
        route = local_points(self.survey.smooth[index:index+640], ego)
        self.assertEqual(policy.match_current_lane(native.candidates(ego, route)).link_id,
                         'A2256W000410')

    def test_recorded_fast_escape_checks_the_target_car_before_starting(self):
        # follow-20261010T062150Z 487.850 s: the lane-430 escape started at
        # 117.4 km/h with this already visible slower car, then cancelled
        # as unsafe 0.224 s later. Use its surveyed road and the start frame.
        ego = dict(x=67.23, y=12.54, heading_deg=-89.92, speed_kmh=117.4, stamp=100.)
        car = (70.2372930391329, -32.49558452971676, 4.979358196258545, 'lidar_car', 'npc',
               12.465403790829821, 4.979358196258545, 2.0885634422302246, 270.76999607018263,
               .5574198245836122, -12.452934385420798, 500050, 0., None, True, 0.)
        box = (65.36803629451606, -28.772615033366524, 1.707721471786499, 'fusion_obstacle',
               'obstacle', 0., 1.100000023841858, 1.707721471786499, -89.81070540935589,
               0., 0., 213, 0., None)
        index = int(np.argmin(np.linalg.norm(self.survey.route-[ego['x'], ego['y']], axis=1)))
        route = local_points(self.survey.smooth[index:index+640], ego)
        native = HighwayLaneMap(self.survey.lanes, self.survey.config)
        lanes = native.candidates(ego, route)
        corridor = native.corridor(ego, route, lanes)
        planner = HighwayPlanner(self.survey.config, self.survey.route)
        policy = planner.three_lane_policy = ThreeLanePolicy(self.survey.config)
        planner.longitudinal.begin(ego)
        policy.update_motion_paths(planner, native.motion_candidates(ego, [car]), ego, [car, box])
        target = next(lane for lane in lanes if lane.link_id == 'A2256W000430')
        path = policy.sequence_path([target], 200., ego['speed_kmh']/3.6)
        result, check = policy.evaluate(planner, path, ego, [car, box], corridor.clear_distance,
            lambda path, horizon: static_clear_distance(path, ego, highway_static_objects([car, box]), horizon),
            changing=True, destinations=[target], transition=200., admission=True)
        self.assertTrue(result is None, check)
        self.assertEqual(check['reason'], 'lane-change traffic gap occupied')
        self.assertEqual(check['gap_risk']['object_id'], 500050)

    def test_completed_recorded_change_keeps_its_destination_match(self):
        native = HighwayLaneMap(self.survey.lanes, self.survey.config)
        planner = HighwayPlanner(self.survey.config, self.survey.route)
        policy = planner.three_lane_policy = ThreeLanePolicy(self.survey.config)
        policy.change_started = 99.
        # Before arrival the matched source is the converging link 411.
        policy.current_chain = ('A2256W000411',)
        for n, (x, y, heading, speed) in enumerate([(66.40, 102.85, -88.39, 111.7),
                                                   (66.61, 95.44, -88.49, 105.5)]):
            ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed, stamp=100.+n*.13)
            index = int(np.argmin(np.linalg.norm(self.survey.route-[x, y], axis=1)))
            route = local_points(self.survey.smooth[index:index+640], ego)
            lanes = native.candidates(ego, route)
            corridor = native.corridor(ego, route, lanes)
            if n == 0:
                target = next(lane for lane in lanes if lane.link_id == 'A2256W000409')
                policy.target_chain = target.chain
                policy.change_chains = [target.chain]
                planner.saved_world = world_points(policy.sequence_path([target], 200., speed/3.6), ego)
                planner.phase = 'CHANGE'
            result = planner.decide(index, route, ego, [], {}, corridor.clear_distance,
                lambda path, horizon: horizon, perception_ready=True, lane_options=lanes,
                motion_options=[])
            self.assertEqual(result['status']['current_lane'], 'A2256W000409')
            self.assertEqual(result['status']['target_lane'], 'A2256W000409')
            self.assertEqual(result['status']['lane_changes'], 1)
            self.assertIsNone(planner.saved_world)


if __name__ == '__main__':
    unittest.main()
