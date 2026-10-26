"""Long lane followers are checked with their lane's heading, not box-yaw jitter.

Run follow-20261009T204445Z t=220.43-223.72: a lane change passing a 9x3.2 m
truck (uid 500013) was cancelled ("unsafe change") when its LiDAR box yaw jumped
from 269.9 to 274.3 deg; it jittered +-4.4 deg (+-0.35 m at its ends) while its
centre followed the lane. test_data/highway_truck_pass_20261009.json holds its
fresh boxes, the surveyed lanes (5 m samples) and the ego states of that pass.
"""
import json
import math
from pathlib import Path
import unittest

import numpy as np

from highway_planner import HighwayPlanner, TRAFFIC_CONTROL_HORIZON_S, local_points, world_points
from highway_three_lane import HighwayLane, ThreeLanePolicy
from planning.highway.motion_evidence import HighwayMotionEvidence


FIXTURE = json.loads((Path(__file__).parent/'test_data'/'highway_truck_pass_20261009.json').read_text())
WORLD = dict(x=0., y=0., heading_deg=0.)
TRUCK = FIXTURE['truck_uid']


def dense(points, step=.5):
    """Native lanes are sampled every 0.5 m; lane matching needs that density."""
    points = np.asarray(points, dtype=float)
    stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    samples = np.arange(0., stations[-1], step)
    return np.column_stack([np.interp(samples, stations, points[:, k]) for k in (0, 1)])


def car(x, y, length, width, yaw, vx, vy, uid=TRUCK, age=0.):
    return (x, y, max(length, width), 'lidar_car', 'npc', math.hypot(vx, vy), length, width,
            yaw, vx, vy, uid, 0., None, True, age)


class LaneHeadingTests(unittest.TestCase):
    def evidence(self, observations, lanes=None, length=None, until=None):
        lanes = ([HighwayLane('409', -1, dense(FIXTURE['lanes']['409']), ('409', 'exit'), 0.)]
                 if lanes is None else lanes)
        evidence, headings = HighwayMotionEvidence(), []
        for stamp, x, y, body, width, yaw, vx, vy in observations:
            if until is not None and stamp > until+1e-6:
                break
            evidence.update([car(x, y, body if length is None else length, width, yaw, vx, vy)],
                            1000.+stamp, lanes, WORLD)
            headings.append(evidence.lane_headings.get(TRUCK))
        return evidence, headings

    def recheck(self, frame, stable_yaws):
        """The committed change's 3 s recheck at a recorded frame (a risk cancels it)."""
        config = dict(from_idx=0, to_idx=1800, kmh=120., lane_change_policy='free_three_lane',
                      lane_change_goal=0, end_link='exit', comfort_decel_mps2=1.2,
                      comfort_jerk_mps3=2., emergency_jerk_mps3=12.)
        planner = HighwayPlanner(config, np.column_stack((np.arange(0., 1000., .5), np.zeros(2000))))
        policy = planner.three_lane_policy = ThreeLanePolicy(config)
        egos = {int(k): dict(v, stamp=1000.+v['stamp']) for k, v in FIXTURE['egos'].items()}
        def lanes(ego):
            result = []
            for rank, key in enumerate(FIXTURE['lane_order']):
                centre = local_points(dense(FIXTURE['lanes'][key]), ego)
                at = int(np.argmin(np.linalg.norm(centre, axis=1)))
                result.append(HighwayLane(key, rank, centre[at:], (key, 'exit'), 300.))
            return result
        start = egos[FIXTURE['start']['frame']]
        first = lanes(start)
        path = policy.sequence_path(first[1:3], FIXTURE['start']['leg_m'], start['speed_kmh']/3.6)
        planner.saved_world = world_points(path, start)
        for k in range(2889, frame+1):
            planner.longitudinal.begin(egos[k])
        ego, speed = egos[frame], egos[frame]['speed_kmh']/3.6
        objects = [tuple(o) for o in ego['truck']]
        motion = [HighwayLane(key, -1, local_points(dense(FIXTURE['lanes'][key]), ego), (key, 'exit'), 0.)
                  for key in FIXTURE['lane_order']]
        policy.update_motion_paths(planner, motion, ego, objects)
        planner.longitudinal.end_buffer_m = speed*1.2
        planner.longitudinal.end_distance_m = ((ego['exit_cap_kmh']/3.6)**2-(60./3.6)**2)/2.4+speed*1.2
        planner.longitudinal.stable_yaws = stable_yaws
        local = local_points(planner.saved_world, ego)
        base, _ = policy.evaluate(planner, local[int(np.argmin(np.linalg.norm(local, axis=1))):], ego,
            objects, lambda p, h: h, lambda p, h: h, check_horizon_s=TRAFFIC_CONTROL_HORIZON_S,
            goal_limit=FIXTURE['start']['goal_kmh']/3.6)
        return base['risk']

    def test_recorded_truck_keeps_its_lane_heading_through_the_pass(self):
        observations = FIXTURE['truck_observations']
        _, headings = self.evidence(observations)
        self.assertTrue(all(h is not None for h in headings[2:]))
        jump = next(i for i, o in enumerate(observations) if abs(o[0]-223.602) < 1e-3)
        self.assertAlmostEqual(observations[jump][5], 274.326, places=2)
        self.assertLess(abs((headings[jump]+90.) % 360.-0.2), .5)    # lane -89.8 deg
        # Sizes and centres stay measured; only the footprint heading changes.
        evidence, _ = self.evidence(observations[:3])
        row = evidence.update([car(*observations[3][1:])], 1000.+observations[3][0],
                              [HighwayLane('409', -1, dense(FIXTURE['lanes']['409']), ('409', 'exit'), 0.)],
                              WORLD)[0][0]
        self.assertEqual(row[:2], tuple(observations[3][1:3]))
        self.assertEqual((row[6], row[7], row[8]), tuple(observations[3][3:6]))

    def test_recorded_recheck_no_longer_cancels_the_pass(self):
        evidence, _ = self.evidence(FIXTURE['truck_observations'], until=223.602)
        for frame in (2893, 2894, 2895):
            with self.subTest(frame=frame):
                measured = self.recheck(frame, {})
                self.assertIsNotNone(measured)
                self.assertEqual(measured['object_id'], TRUCK)
                self.assertIsNone(self.recheck(frame, dict(evidence.lane_headings)))

    def cut_in(self, lateral_mps):
        """A 9x3.2 m truck in lane 2 (y=7) at 6.4 m/s, its centre moving toward lane 1."""
        lanes = [HighwayLane(str(k), k, np.column_stack((np.arange(-50., 950., .5), np.full(2000, k*3.5))),
                             (str(k), 'exit'), 600.) for k in range(3)]
        yaw = math.degrees(math.atan2(-lateral_mps, 6.4))
        observations = [(t, 20.+6.4*t, 7.-lateral_mps*t, 8.96, 3.19, yaw, 6.4, -lateral_mps)
                        for t in np.arange(0., 1.31, .13)]
        evidence, headings = self.evidence(observations, lanes=lanes[2:])
        return dict(evidence.lane_headings), headings, car(*observations[-1][1:]), lanes

    @staticmethod
    def committed_change_past(truck, stable_yaws, lanes, admission=None):
        """Lane 0 -> 1 change past ``truck``: its recheck, or a new start if ``admission``."""
        config = dict(from_idx=0, to_idx=1800, kmh=120., lane_change_policy='free_three_lane',
                      lane_change_goal=0, end_link='exit')
        planner = HighwayPlanner(config, np.column_stack((np.arange(0., 1000., .5), np.zeros(2000))))
        policy = planner.three_lane_policy = ThreeLanePolicy(config)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=55., stamp=20.)
        # Native lane centres start at the ego; motion lanes also extend behind.
        ahead = [HighwayLane(l.link_id, l.rank, l.centre[l.centre[:, 0] >= 0.], l.chain, 600.) for l in lanes]
        path = policy.sequence_path([ahead[1]], 60., 55./3.6)
        if not admission:
            planner.saved_world = world_points(path, ego)
        policy.update_motion_paths(planner, lanes, ego, [truck])
        planner.longitudinal.stable_yaws = stable_yaws
        if admission:
            return policy.evaluate(planner, path, ego, [truck], lambda p, h: h, lambda p, h: h,
                changing=True, destinations=[ahead[1]], transition=60., goal_limit=planner.target_mps)
        base, _ = policy.evaluate(planner, path, ego, [truck], lambda p, h: h, lambda p, h: h,
            check_horizon_s=TRAFFIC_CONTROL_HORIZON_S, goal_limit=planner.target_mps)
        return base['risk']

    def test_cut_in_keeps_its_measured_heading_and_still_cancels_or_refuses(self):
        stable, headings, truck, lanes = self.cut_in(.4)
        self.assertTrue(all(h is None for h in headings))
        risk = self.committed_change_past(truck, stable, lanes)
        self.assertIsNotNone(risk)
        self.assertEqual(risk['object_id'], TRUCK)
        start, check = self.committed_change_past(truck, stable, lanes, admission=True)
        self.assertIsNone(start)
        self.assertEqual(check['reason'], 'lane-change traffic gap occupied')
        # The same truck following its lane gets the lane heading and is passed.
        stable, headings, truck, lanes = self.cut_in(0.)
        self.assertIsNotNone(headings[-1])
        self.assertIsNone(self.committed_change_past(truck, stable, lanes))

    def test_no_lane_match_short_body_or_jump_keeps_the_measured_heading(self):
        observations = FIXTURE['truck_observations']
        lane = [HighwayLane('409', -1, dense(FIXTURE['lanes']['409']), ('409', 'exit'), 0.)]
        far = [HighwayLane('far', -1, dense(FIXTURE['lanes']['418']), ('far', 'exit'), 0.)]
        _, headings = self.evidence(observations, lanes=far)
        self.assertTrue(all(h is None for h in headings))
        _, headings = self.evidence(observations, length=4.9)
        self.assertTrue(all(h is None for h in headings))
        # A 3 m centre jump along the lane (re-associated body) restarts the history.
        jumped = [list(o) for o in observations]
        for o in jumped[20:]:
            o[2] -= 3.
        _, headings = self.evidence(jumped, lanes=lane)
        self.assertIsNotNone(headings[19])
        self.assertEqual(headings[20:22], [None, None])
        self.assertIsNotNone(headings[22])


if __name__ == '__main__':
    unittest.main()
