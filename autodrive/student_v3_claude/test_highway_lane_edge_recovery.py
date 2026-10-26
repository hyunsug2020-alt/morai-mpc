"""Lane-edge stall after a change into the leftmost lane (run follow-20261010T052911Z).

t=270.655 s (48.6 km/h, lane A2256W000435): 0.23-0.36 m left of the centre with a
2.7-3.7 deg leftward heading after a 434->435 change. The keep path matched that
heading over 34 m, bulged 0.62 m out and left the mapped road 6.25 m ahead: speed-
plan emergency, stop 0.68-0.72 m left of the centre (corner 0.08 m over the 0.2 m
margin). t=280.016 s: the 12 m inward join swung its rear corner out (0.080 ->
0.145 m over the margin) and the escape returned None: 5+ min standstill.
Rebuilt with the surveyed map, HighwayLaneMap and LaneCorridor at the logged poses.
"""
import json
from pathlib import Path
import unittest

import numpy as np

from gui import HdMapLanes, smooth_control_route, local_xy
from highway_planner import HighwayPlanner, geometry_speed_cap, lane_change_path
from highway_speed_plan import clearance_forecast_bound
from lane_corridor import LaneCorridor
from planning.highway.lane_map import HighwayLaneMap, arc
from planning.highway.policy import ThreeLanePolicy, corridor_overrun
from planning.highway.trajectory_geometry import normalize_path


HERE = Path(__file__).parent
MOVING = dict(x=74.72, y=-23.52, heading_deg=-86., speed_kmh=48.6, stamp=100.)
STANDING = dict(x=75.24, y=-38.3, heading_deg=-88.36, speed_kmh=0., stamp=100.)


class RecordedLaneEdgeStall(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        waypoints = json.loads((HERE/'global_path.json').read_text())['waypoints']
        cls.route = np.array([[p['x'], p['y']] for p in waypoints])
        cls.control = smooth_control_route(cls.route)
        cls.lanes = HdMapLanes(HERE/'link_set.json')
        cls.lanes.set_route(cls.route)
        cls.config = json.loads((HERE/'highway_mission.json').read_text())

    def scene(self, ego):
        planner = HighwayPlanner(dict(self.config), self.route)
        lane_map = HighwayLaneMap(self.lanes, planner.config)
        index = 1800+int(np.argmin(np.linalg.norm(self.route[1800:3600]-[ego['x'], ego['y']], axis=1)))
        dense = np.array([local_xy(*self.control[(index+i) % len(self.route)], ego) for i in range(640)])
        candidates = lane_map.candidates(ego, dense)
        return planner, index, dense, candidates, lane_map.corridor(ego, dense, candidates)

    def decide(self, ego, corridor=None):
        planner, index, dense, candidates, recorded = self.scene(ego)
        corridor = corridor or recorded
        result = planner.decide(index, dense, ego, [], {}, corridor.clear_distance, lambda p, h: h,
                                perception_ready=True, lane_options=candidates)
        final = (result.get('map_clearance') or corridor.clear_distance)(result['path'], 170.)
        speed = planner.plan_speed(result['path'], ego, [], result['target_mps'],
                                   static_clear_m=clearance_forecast_bound(final, 170.),
                                   ceiling_mps=planner.overtake_mps)
        return result, final, speed, candidates, corridor

    def test_recorded_heading_after_a_change_keeps_a_clear_correction(self):
        _, _, _, candidates, corridor = self.scene(MOVING)
        lane = min(candidates, key=lambda l: np.linalg.norm(l.centre[0]))
        self.assertEqual(lane.link_id, 'A2256W000435')
        speed = MOVING['speed_kmh']/3.6
        recorded = lane_change_path(lane.centre, 0., speed*2.5, match_heading=True)
        self.assertLess(corridor.clear_distance(recorded, 170.), 10.)       # 6.25 m in the run
        result, final, plan, _, _ = self.decide(MOVING)
        self.assertEqual(final, 170.)
        self.assertGreaterEqual(geometry_speed_cap(result['path'], speed), speed-.1)
        self.assertFalse(plan['emergency'])
        self.assertGreater(plan['reference_mps']*3.6, 45.)
        self.assertFalse(result['status']['low_speed_alignment_recovery'])

    def test_recorded_standstill_actual_body_uses_existing_inward_allowance(self):
        _, _, _, candidates, corridor = self.scene(STANDING)
        lane = min(candidates, key=lambda l: np.linalg.norm(l.centre[0]))
        aligned = lane_change_path(lane.centre, 0., 12.)
        self.assertEqual(corridor.clear_distance(aligned, 170.), 170.)
        # The measured current pose and actual rear bumper stay within the
        # existing shrinking allowance; a centred rear falsely required escape.
        stations, overrun = corridor_overrun(corridor, aligned, 170.)
        allowance = overrun[0]*np.clip(1.-stations/6., 0., 1.)+.03
        self.assertTrue(np.all(overrun <= allowance))
        result, final, plan, _, _ = self.decide(STANDING)
        self.assertFalse(result['status']['low_speed_alignment_recovery'])
        self.assertFalse(result['status']['alignment_escape_join'])
        self.assertEqual(final, 170.)
        self.assertFalse(plan['emergency'])
        self.assertGreater(plan['reference_mps'], .5)
        stations, overrun = corridor_overrun(corridor, result['path'], 6.)
        self.assertAlmostEqual(overrun[0], .170052, delta=.001)
        allowance = overrun[0]*np.clip(1.-stations/6., 0., 1.)+.03
        self.assertTrue(np.all(overrun <= allowance))
        # Obstacle and traffic checks are unchanged: plain clearance elsewhere.
        self.assertIsNone(result.get('map_clearance'))

    def test_genuinely_closed_road_still_stops(self):
        _, _, _, _, corridor = self.scene(MOVING)
        start = int(np.argmin(np.linalg.norm(corridor.route, axis=1)))
        ahead = arc(corridor.route)-arc(corridor.route)[start]
        left, right = corridor.left.copy(), corridor.right.copy()
        closed = ahead > 25.
        left[closed] = right[closed] = (corridor.left[closed]+corridor.right[closed])/2.
        result, final, plan, _, _ = self.decide(MOVING, LaneCorridor(corridor.route, left, right))
        self.assertLess(final, 26.)          # no corrective span routes around a real closure
        if not plan['emergency']:            # escalated braking stops before it
            self.assertLess(plan['reference_mps'], MOVING['speed_kmh']/3.6-1.)
            self.assertLessEqual(plan['diagnostics']['distance_m'], final)

    def test_latest_recorded_common_entry_keeps_a_checked_path(self):
        # follow-20261010T062150Z 496.267--496.682 s: six emergency frames
        # while the common-entry margin was still 21--31 m ahead.
        poses = [(71.90, -186.70, -83.82, 76.7), (72.21, -189.46, -83.68, 75.4),
                 (72.44, -191.52, -83.69, 74.9), (72.60, -193.00, -83.74, 73.0),
                 (72.77, -194.58, -83.76, 71.0), (72.98, -196.53, -83.77, 67.5)]
        for x, y, heading, speed in poses:
            with self.subTest(pose=(x, y, heading, speed)):
                ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed, stamp=100.)
                result, final, plan, candidates, corridor = self.decide(ego)
                self.assertEqual(final, 170., result['status']['current_lane'])
                self.assertFalse(plan['emergency'])
                for lane in candidates:
                    self.assertLessEqual(np.max(np.linalg.norm(lane.centre-lane.surveyed_centre, axis=1)),
                                         .600001)

    def test_latest_recorded_pose_at_the_seam_has_a_checked_actual_body_path(self):
        # Same run 249.319/249.486 s. The actual body, measured t0 and unchanged
        # decreasing boundary allowance clear the formerly centred-body veto.
        for x, y, heading, speed in [(74.58, -214.66, -88.06, 68.4),
                                     (74.63, -216.57, -88.91, 68.2)]:
            with self.subTest(pose=(x, y, heading, speed)):
                ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed, stamp=100.)
                result, final, plan, _, corridor = self.decide(ego)
                self.assertEqual(final, 170.)
                stations, overrun = corridor_overrun(corridor, result['path'], 12.)
                allowance = max(0., overrun[0])*np.clip(1.-stations/6., 0., 1.)+.03
                self.assertTrue(np.all(overrun <= allowance))
                self.assertFalse(plan['emergency'])

    def test_recorded_slow_entry_keeps_the_measured_lane_without_emergency_braking(self):
        # follow-20261012T091249Z, 18:16:40.918/41.004 KST: the selected A422
        # path left the 20 cm margin 8.25/6.25 m ahead at only 61--62 km/h.
        # Preserve the recorded lane history: a fresh planner instead picks
        # nearby A445 and would miss the A422 tracking-repair failure.
        poses = ((74.2476, -207.7266, -86.47, 61.1),
                 (74.3595, -209.6353, -86.65, 62.0))
        for x, y, heading, speed in poses:
            with self.subTest(pose=(x, y, heading, speed)):
                ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=speed, stamp=100.)
                planner, index, dense, candidates, corridor = self.scene(ego)
                lane = next(l for l in candidates if l.link_id == 'A2256W000422')
                planner.completed, planner.phase = 3, 'MERGE'
                planner.three_lane_policy = ThreeLanePolicy(planner.config)
                planner.three_lane_policy.current_chain = lane.chain
                result = planner.decide(index, dense, ego, [], {}, corridor.clear_distance,
                    lambda path, horizon: horizon, perception_ready=True, lane_options=candidates)
                self.assertEqual(result['status']['current_lane'], lane.link_id)
                path = normalize_path(result['path'])[0].astype(np.float32).astype(float)
                final = corridor.clear_distance(path, 170.)
                self.assertEqual(final, 170.)
                stations, overrun = corridor_overrun(corridor, path, 170.)
                allowance = max(0., overrun[0])*np.clip(1.-stations/6., 0., 1.)+.03
                self.assertTrue(np.all(overrun <= allowance))
                for candidate in candidates:
                    self.assertLessEqual(np.max(np.linalg.norm(
                        candidate.centre-candidate.surveyed_centre, axis=1)), .600001)
                plan = planner.plan_speed(path, ego, [], result['target_mps'],
                    static_clear_m=clearance_forecast_bound(final, 170.),
                    ceiling_mps=planner.overtake_mps)
                self.assertFalse(plan['emergency'])
                self.assertGreaterEqual(plan['reference_mps']*3.6, speed-1.)


if __name__ == '__main__':
    unittest.main()
