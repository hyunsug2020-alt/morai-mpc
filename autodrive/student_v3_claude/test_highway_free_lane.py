"""Free lane choice on the highway (user 2026-10-11): every lane is usable.

Going to the far lane is fine, but once it is blocked the car must be free to
move back (and out again) to whichever lane is fastest. Lane changes keep the
start margin (lane_gain_kmh) and stability time, a committed change is only
replaced by the larger cancel margin, and every safety check still applies.
"""
import json
import math
from pathlib import Path
import unittest

import numpy as np

from gui import HdMapLanes, smooth_control_route, static_clear_distance
from highway_planner import HighwayPlanner, local_points
from highway_three_lane import HighwayLane, HighwayLaneMap
from planning.highway.object_policy import highway_static_objects
from sim_time import to_sim_objects


HERE = Path(__file__).parent
MISSION = json.loads((HERE/'highway_mission.json').read_text())


def car(x, y, speed, uid=1, valid=True):
    return (x, y, 4.9, "lidar_car", "npc", speed, 4.9, 2.1, 0., speed, 0., uid, 0., None, valid, 0.)


class FreeLaneChoice(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        self.config = dict(from_idx=0, to_idx=1800, kmh=120., lane_change_policy="free_three_lane",
                           lane_change_goal=0, end_link="exit", comfort_decel_mps2=2.5,
                           comfort_jerk_mps3=4., lane_gain_kmh=2., lane_gain_hold_s=.3,
                           lane_cancel_gain_kmh=3.)
        self.planner = HighwayPlanner(self.config, self.route)
        self.lanes = [HighwayLane(str(k), k, self.route+np.array([0., k*3.5]),
                                  (str(k), "exit"), 800.) for k in range(3)]

    def decide(self, ego, objects, lanes=None):
        lanes = self.lanes if lanes is None else lanes
        behind = np.column_stack((np.arange(-200., 0., .5), np.zeros(400)))
        motion = [HighwayLane(l.link_id, -1, np.vstack((behind+l.centre[0], l.centre)),
                              l.chain, 0.) for l in lanes]
        return self.planner.decide(0, self.route[:640], ego, objects, {}, lambda p, h: h,
            lambda p, h: h, perception_ready=True, lane_options=lanes, motion_options=motion)

    def shifted(self, x, y):
        """Lanes in the ego frame of a pose (x, y) on the straight road."""
        return [HighwayLane(l.link_id, l.rank, l.centre[l.centre[:, 0] >= x]-[x, y], l.chain,
                            800.-x) for l in self.lanes]

    def commit_left(self, speed_kmh=60., objects=None):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=speed_kmh, stamp=10.)
        objects = [car(30., 0., 8., uid=1)] if objects is None else objects
        self.decide(dict(ego, stamp=9.6), objects)
        result = self.decide(ego, objects)
        self.assertIsNotNone(self.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '1')
        return ego

    def test_blocked_target_lane_returns_after_the_cancel_margin_persists(self):
        ego = self.commit_left()
        saved = self.planner.saved_world.copy()
        # The lane we are entering is now blocked by a slower car (no near-term
        # conflict: 55 m ahead); the lane we left has cleared.
        blocked = [car(60., 3.5, 10., uid=2)]
        first = self.decide(dict(ego, stamp=10.1), blocked)
        recheck = first['status']['current_lane_recheck']
        self.assertTrue(recheck['free_recheck'])
        self.assertIsNone(recheck['cancel_basis'])               # one frame is not enough
        self.assertEqual(recheck['progress_held_s'], 0.)
        np.testing.assert_array_equal(self.planner.saved_world, saved)
        later = self.decide(dict(ego, stamp=10.45), blocked)
        self.assertEqual(later['status']['current_lane_recheck']['cancel_basis'], 'progress')
        self.assertIsNone(self.planner.saved_world)
        self.assertEqual(later['status']['target_lane'], '0')

    def test_move_back_in_front_of_a_closing_rear_car_is_refused(self):
        from unittest.mock import patch
        import planning.highway.policy as policy_module
        ego = self.commit_left()
        blocked = [car(60., 3.5, 10., uid=2)]
        original = policy_module.timed_gap
        def rear_conflict(*args, **kwargs):
            # The return check (current lane, rear traffic included) sees a
            # car closing from behind in the lane we would go back to.
            if kwargs.get('lane_change') is False and 'ignore_rear_approach' not in kwargs:
                return dict(when=1.6, collision_when=1.6, name='lidar_car', object_id=9,
                            rear_approach=True, reason='predicted lane-change gap occupied')
            return original(*args, **kwargs)
        with patch.object(policy_module, 'timed_gap', side_effect=rear_conflict):
            for stamp in (10.1, 10.45, 10.8):
                result = self.decide(dict(ego, stamp=stamp), blocked)
                self.assertIsNone(result['status']['current_lane_recheck']['cancel_basis'])
                # Never back into lane 0; moving on (to the free far lane) is allowed.
                self.assertIsNotNone(self.planner.saved_world)
                self.assertNotEqual(result['status']['target_lane'], '0')

    def test_committed_search_runs_at_a_bounded_rate(self):
        from planning.highway.policy import FREE_SEARCH_PERIOD_S
        ego = self.commit_left()
        blocked = [car(60., 3.5, 10., uid=2)]
        searched = []
        for stamp in (10.05, 10.10, 10.21, 10.26):
            result = self.decide(dict(ego, stamp=stamp), blocked)
            searched.append(bool(result['status']['candidate_checks']))
            # The current-lane recheck itself runs on every frame.
            self.assertTrue(result['status']['current_lane_recheck']['free_recheck'])
        self.assertEqual(searched, [True, False, True, False])
        self.assertAlmostEqual(FREE_SEARCH_PERIOD_S, .15)

    def test_completed_leg_stops_when_the_next_one_is_not_faster(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=10.)
        slow = [car(65., 0., 15.), car(110., 3.5, 15., 2)]
        self.decide(dict(ego, stamp=9.5), slow)
        first = self.decide(ego, slow)
        self.assertEqual(first['status']['planned_lane_sequence'], ['1', '2'])
        length = first['status']['selected_transition_m']
        # First leg done: the middle lane's car has gone, the far lane now has
        # a slower one. Continuing would only be a change for its own sake.
        result = self.decide(dict(ego, x=length, y=3.5, stamp=15.),
                             [car(length+90., 7., 20., 4)], lanes=self.shifted(length, 3.5))
        self.assertEqual(self.planner.completed, 1)
        self.assertEqual(result['status']['current_lane_recheck']['cancel_basis'], 'next leg not faster')
        self.assertIsNone(self.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '1')

    def test_far_lane_car_moves_back_when_its_lane_is_blocked(self):
        # Already in the far lane '2', blocked by a slow car; the middle lane
        # is clear. The ordinary start test (2 km/h, 0.3 s) moves it back.
        ego = dict(x=0., y=7., heading_deg=0., speed_kmh=70., stamp=10.)
        lanes = self.shifted(0., 7.)
        blocked = [car(30., 7., 11., uid=5)]
        self.decide(dict(ego, stamp=9.65), blocked, lanes=lanes)
        result = self.decide(ego, blocked, lanes=lanes)
        self.assertIsNotNone(self.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '1')

    def test_move_back_beside_a_car_is_refused(self):
        ego = dict(x=0., y=7., heading_deg=0., speed_kmh=70., stamp=10.)
        lanes = self.shifted(0., 7.)
        beside = [car(30., 7., 11., uid=5), car(0.5, 3.5, 70./3.6, uid=6)]
        self.decide(dict(ego, stamp=9.65), beside, lanes=lanes)
        result = self.decide(ego, beside, lanes=lanes)
        self.assertIsNone(self.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '2')
        middle = [c for c in result['status']['candidate_checks'] if c.get('lanes') == ['1']]
        self.assertTrue(middle)
        self.assertFalse(any(c['accepted'] for c in middle))

    def test_stability_timer_survives_a_link_boundary(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=10.)
        slow = [car(30., 0., 8., uid=1)]
        first = self.decide(ego, slow)
        self.assertIsNone(self.planner.saved_world)
        self.assertTrue(first['status']['further_overtake_requested'])
        # Same lanes, new link ids (a native link boundary).
        renamed = [HighwayLane(l.link_id+'b', l.rank, l.centre, (l.link_id+'b', 'exit'), 800.)
                   for l in self.lanes]
        result = self.decide(dict(ego, stamp=10.31), slow, lanes=renamed)
        self.assertIsNotNone(self.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '1b')


class RecordedFreeLaneReplay(unittest.TestCase):
    """Recorded decision frames (ego, RTF, planned object tracks) replayed
    through the production policy on the surveyed lanes. LiDAR returns are not
    recorded; static clearance uses the recorded boxed bodies only."""
    @classmethod
    def setUpClass(cls):
        cls.route = np.array([[p['x'], p['y']] for p in
                              json.loads((HERE/'global_path.json').read_text())['waypoints']])
        cls.smooth = smooth_control_route(cls.route)
        cls.lanes = HdMapLanes(HERE/'link_set.json')
        cls.lanes.set_route(cls.route)

    def replay(self, name):
        frames = json.loads((HERE/'test_data'/name).read_text())['frames']
        planner = HighwayPlanner(dict(MISSION), self.route)
        native = HighwayLaneMap(self.lanes, planner.config)
        out = []
        for frame in frames:
            ego, rtf = dict(frame['ego']), frame['rtf'] or 1.
            index = 1800+int(np.argmin(np.linalg.norm(self.route[1800:3600]-[ego['x'], ego['y']], axis=1)))
            local = local_points(self.smooth[index:index+640], ego)
            objects = to_sim_objects([tuple(o) for o in frame['objects']], rtf)
            lanes = native.candidates(ego, local)
            corridor = native.corridor(ego, local, lanes)
            planner.longitudinal.time_scale = rtf
            bodies = highway_static_objects(objects)
            result = planner.decide(index, local, ego, objects, {}, corridor.clear_distance,
                lambda p, h, ego=ego, bodies=bodies: static_clear_distance(p, ego, bodies, h),
                perception_ready=True, lane_options=lanes,
                motion_options=native.motion_candidates(ego, objects))
            out.append((frame, result['status']))
        return out

    def test_blocked_far_lane_sequence_moves_back_to_the_clear_lane(self):
        # follow-20261010T151125Z lap 2, 32.5-41.2 s: an escape [430, 431]
        # past a standing car. After its first leg the right lane (420) became
        # clear (forecast 76-81 km/h against 62 km/h for continuing), but the
        # admitted sequence went on into the far lane, which then stayed
        # blocked to the common entrance (61-70 km/h behind a car, s 487-677).
        steps = self.replay('highway_free_lane_151125Z_lap2.json')
        start = steps[0][0]['t']
        commit = next(status for frame, status in steps if status['phase'] == 'CHANGE')
        self.assertEqual(commit['planned_lane_sequence'], ['A2256W000430', 'A2256W000431'])
        back = [(frame['t']-start, status) for frame, status in steps
                if status['target_lane'] == 'A2256W000420' and status['phase'] == 'CHANGE']
        self.assertTrue(back, 'no move back to the clear lane')
        when, status = back[0]
        self.assertGreater(when, 8.3+.3-.05)                 # after the 0.3 s stable margin
        accepted = [c for c in status['candidate_checks'] if c.get('accepted')]
        self.assertEqual(accepted[0]['lanes'], ['A2256W000420'])
        self.assertGreaterEqual(accepted[0]['average_kmh'],
                                status['current_average_kmh']+MISSION['lane_cancel_gain_kmh'])
        # The recorded run (no recheck while committed) kept the far lane.
        logged = next(frame['logged'] for frame, s in steps if frame['t']-start >= when-1e-6)
        self.assertEqual(logged['target_lane'], 'A2256W000431')

    def test_admitted_change_is_kept_while_its_lane_stays_fastest(self):
        # follow-20261010T180455Z lap 2 (52.9/64.8 s drive), 33.0-39.7 s: the
        # same kind of sequence while the far lane stayed the fastest. Free
        # rechecks run every committed frame but change nothing.
        steps = self.replay('highway_free_lane_180455Z_lap2.json')
        committed = next(i for i, (_, status) in enumerate(steps) if status['phase'] == 'CHANGE')
        # Real front reach keeps a nearby side car in the fixed entry gate
        # longer. Start promptly once a body-safe candidate persists.
        first_lane = steps[committed][1]['target_lane']
        eligible = next(i for i, (_, status) in enumerate(steps) if any(
            c.get('reason') == 'safe lane candidate' and c.get('lanes', [])[:1] == [first_lane]
            for c in status['candidate_checks']))
        # Admission uses the ego source clock. Log writes can be delayed or
        # repeat an observation; their wall timestamps do not measure the hold.
        waited = steps[committed][0]['ego']['stamp']-steps[eligible][0]['ego']['stamp']
        self.assertGreaterEqual(waited, MISSION['lane_gain_hold_s']-1e-6)
        self.assertLess(waited, MISSION['lane_gain_hold_s']+.2)
        rechecked = 0
        for frame, status in steps[committed:]:
            self.assertEqual(status['target_lane'], frame['logged']['target_lane'], frame['t'])
            recheck = status.get('current_lane_recheck') or {}
            rechecked += bool(recheck.get('free_recheck'))
            self.assertNotEqual(recheck.get('cancel_basis'), 'progress')
        self.assertGreater(rechecked, 30)


if __name__ == '__main__':
    unittest.main()
