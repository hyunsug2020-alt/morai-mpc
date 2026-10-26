"""Aggressive highway tuning (2026-10-10): every relaxed limit keeps its collision check.

Time-loss source -> change under test:
* plant acceleration: full-throttle reference, forecast bound = full-pedal model
* exit / leader braking: 2.5 m/s² comfort braking, 4 m/s³ jerk, 58 km/h boundary
* sensor jitter: mildly late scans no longer freeze the speed
* overtakes: brisk change length behind a leader, first-leg stability timer,
  2 km/h start / 3 km/h cancel margins
* rule: 60 km/h before the exemption link A2256W000411 starts
"""
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from gui import Runtime, HdMapLanes, RosPathPublisher, smooth_control_route
from highway_planner import (HighwayPlanner, NearFieldMemory, geometry_speed_cap, local_points,
                             HIGHWAY_CURVE_ALAT_MPS2, geometry_speed_profile, lane_change_path)
from highway_speed_plan import (HighwaySpeedPlanner, HIGHWAY_ACCEL_MPS2, FOLLOWER_ACCEL_GAIN,
                                FOLLOWER_FULL_ACCEL_MPS2, MIN_GAP_M)
from highway_three_lane import HighwayLane, HighwayLaneMap, ThreeLanePolicy
from lidar_corridor import LidarCorridor
from motion_limits import LAUNCH_ACCEL_MPS2
from planning.highway import policy as policy_module


HERE = Path(__file__).parent
MISSION = json.loads((HERE/'highway_mission.json').read_text())


def car(x, y, speed, uid=1, valid=True):
    return (x, y, 4.9, "lidar_car", "npc", speed, 4.9, 2.1, 0., speed, 0., uid, 0., None, valid)


class MissionParameters(unittest.TestCase):
    def test_production_mission_uses_the_aggressive_values(self):
        self.assertEqual(MISSION['comfort_decel_mps2'], 2.5)
        self.assertEqual(MISSION['comfort_jerk_mps3'], 4.)
        self.assertEqual(MISSION['lane_gain_kmh'], 2.)
        self.assertEqual(MISSION['lane_gain_hold_s'], .15)
        self.assertTrue(MISSION['brisk_lane_changes'])
        self.assertEqual(MISSION['lane_cancel_gain_kmh'], 3.)
        self.assertEqual(MISSION['exit_speed_kmh'], 58.)
        self.assertEqual(MISSION['rule_speed_limit']['until_idx'], 2255)
        self.assertEqual(MISSION['rule_speed_limit']['kmh'], 58.)
        # Unchanged: cruise, emergency jerk and the overtaking ceiling.
        self.assertEqual(MISSION['kmh'], 120.)
        self.assertEqual(MISSION['emergency_jerk_mps3'], 12.)

    def test_rule_zone_ends_at_the_exemption_link_start(self):
        lanes = json.loads((HERE/'link_set.json').read_text())
        route = np.array([[p['x'], p['y']] for p in
                          json.loads((HERE/'global_path.json').read_text())['waypoints']])
        link = next(l for l in lanes if l['idx'] == MISSION['rule_speed_limit']['link'])
        start = int(np.argmin(np.linalg.norm(route-np.asarray(link['points'][0])[:2], axis=1)))
        self.assertEqual(start, 2251)
        until = MISSION['rule_speed_limit']['until_idx']
        stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        self.assertGreater(stations[until]-stations[start], 1.)     # margin past the link start
        self.assertLess(stations[until]-stations[start], 3.)


class FullThrottleReference(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 501., .5), np.zeros(1002)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=100.)

    def model(self, acceleration):
        model = HighwaySpeedPlanner(2.5, 4., 12.)
        model.begin(self.ego)
        model.acceleration = acceleration
        return model

    def test_clear_road_reference_asks_for_full_throttle(self):
        trajectory = self.model(HIGHWAY_ACCEL_MPS2).preview(self.path, self.ego, (), 120./3.6)
        lead = trajectory.reference_mps-60./3.6
        self.assertAlmostEqual(FOLLOWER_ACCEL_GAIN*lead, 1., places=6)       # throttle 1.0
        # The shared 2.5 m/s² bound held the pedal at 0.83.
        self.assertGreater(lead, LAUNCH_ACCEL_MPS2/(FOLLOWER_ACCEL_GAIN*FOLLOWER_FULL_ACCEL_MPS2)+.4)

    def test_forecast_and_command_preview_stay_inside_the_full_pedal_bound(self):
        model = self.model(5.)                 # an implausible measurement is clipped
        trajectory = model.preview(self.path, self.ego, (), 120./3.6)
        self.assertLessEqual(float(trajectory.accelerations.max()), HIGHWAY_ACCEL_MPS2+1e-9)
        executed = model.command_preview(trajectory)
        self.assertLessEqual(float(executed.accelerations.max()), FOLLOWER_FULL_ACCEL_MPS2+1e-9)
        self.assertEqual(HIGHWAY_ACCEL_MPS2, FOLLOWER_FULL_ACCEL_MPS2)

    def test_close_leader_still_removes_the_throttle(self):
        leader = car(18., 0., 50./3.6)
        trajectory = self.model(0.).preview(self.path, self.ego, [leader], 120./3.6)
        self.assertLess(trajectory.reference_mps, 60./3.6)

    def test_runtime_planner_publishes_the_full_throttle_reference(self):
        planner = HighwayPlanner(dict(MISSION), self.path)
        planner.longitudinal.begin(self.ego)
        planner.longitudinal.acceleration = HIGHWAY_ACCEL_MPS2
        command = planner.plan_speed(self.path, self.ego, (), 120./3.6, static_clear_m=float('inf'),
                                     ceiling_mps=120./3.6)
        self.assertFalse(command['emergency'])
        self.assertAlmostEqual(command['reference_mps']-60./3.6, 1./FOLLOWER_ACCEL_GAIN, places=6)


class FirmerComfortBraking(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 501., .5), np.zeros(1002)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=100.)

    def approach(self, decel):
        model = HighwaySpeedPlanner(decel, 4., 12.)
        model.begin(self.ego)
        leader = car(90., 0., 40./3.6)
        trajectory = model.preview(self.path, self.ego, [leader], 120./3.6, horizon_s=12.)
        gaps = 90.+40./3.6*trajectory.times-trajectory.distances-2.3175-4.9/2.
        return trajectory, gaps

    def test_slow_leader_is_approached_later_and_never_reached(self):
        gentle, gentle_gaps = self.approach(1.2)
        firm, firm_gaps = self.approach(2.5)
        early = firm.times <= 2.
        self.assertGreater(float(firm.distances[early][-1]), float(gentle.distances[early][-1])+2.)
        self.assertGreaterEqual(float(firm.accelerations.min()), -2.5-1e-9)
        # The firmer forecast still keeps the following gap; the 1.2 m/s²
        # forecast could not (3.5 m) and relied on the braking escalation.
        self.assertGreater(float(firm_gaps.min()), MIN_GAP_M)
        self.assertGreater(float(firm_gaps.min()), float(gentle_gaps.min()))

    def test_escalation_only_tries_braking_firmer_than_comfort(self):
        planner = HighwayPlanner(dict(MISSION), self.path)
        planner.longitudinal.begin(self.ego)
        tried = []
        original = planner.longitudinal.preview
        def preview(*args, **kwargs):
            tried.append(kwargs.get('decel_mps2'))
            return original(*args, **kwargs)
        with patch.object(planner.longitudinal, 'preview', side_effect=preview):
            command = planner.plan_speed(self.path, self.ego, (), 90./3.6, static_clear_m=60.)
        firmer = [d for d in tried if d is not None]
        self.assertTrue(firmer)
        self.assertTrue(all(d > 2.5 for d in firmer), firmer)
        # A 60 m stop line at 90 km/h still gets a checked stop or an emergency.
        self.assertTrue(command['emergency'] or
                        command['diagnostics']['braking_limit_mps2'] >= 4.)


class ExitAndRuleZone(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.route = np.array([[p['x'], p['y']] for p in
                              json.loads((HERE/'global_path.json').read_text())['waypoints']])
        cls.smooth = smooth_control_route(cls.route)
        cls.lanes = HdMapLanes(HERE/'link_set.json')
        cls.lanes.set_route(cls.route)

    def decide(self, index, speed_kmh):
        planner = HighwayPlanner(dict(MISSION), self.route)
        p = self.route[index]
        tangent = self.smooth[index+3]-self.smooth[index]
        ego = dict(x=p[0], y=p[1], heading_deg=math.degrees(math.atan2(tangent[1], tangent[0])),
                   speed_kmh=speed_kmh, stamp=float(index))
        native = HighwayLaneMap(self.lanes, planner.config)
        local = local_points(self.smooth[index:index+640], ego)
        candidates = native.candidates(ego, local)
        corridor = native.corridor(ego, local, candidates)
        result = planner.decide(index, local, ego, [], {}, corridor.clear_distance,
            lambda path, horizon: horizon, lane_options=candidates, motion_options=[])
        return planner, result

    def test_exit_cap_brakes_at_comfort_rate_to_58_kmh(self):
        planner, result = self.decide(3300, 100.)
        status = result['status']
        remaining = planner.station[planner.end]-planner.station[3300]
        buffer = 100./3.6*(.6+2.5/4.)
        expected = math.sqrt((58./3.6)**2+2.*2.5*(remaining-buffer))*3.6
        self.assertAlmostEqual(status['exit_cap_kmh'], expected, places=6)
        self.assertEqual(status['exit_speed_kmh'], 58.)
        self.assertEqual(planner.longitudinal.end_speed_mps, 58./3.6)
        _, last = self.decide(3499, 60.)
        self.assertLessEqual(last['target_mps']*3.6, 58.+1e-6)

    def test_rule_zone_caps_the_on_ramp_and_lifts_at_the_exemption(self):
        _, ramp = self.decide(2200, 57.)
        self.assertEqual(ramp['status']['rule_speed_cap_kmh'], 58.)
        self.assertLessEqual(ramp['target_mps']*3.6, 58.+1e-6)
        for check in ramp['status']['candidate_checks']:
            self.assertLessEqual(check['speed_ceiling_kmh'], 58.+1e-6)
        _, highway = self.decide(2260, 57.)
        self.assertIsNone(highway['status']['rule_speed_cap_kmh'])
        self.assertGreater(highway['target_mps']*3.6, 70.)


class Message:
    def __init__(self, data=None):
        self.data = data


class Sink:
    def __init__(self):
        self.items = []

    def publish(self, message):
        self.items.append(message)


class Pose:
    def __init__(self):
        self.header = None
        self.pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0.))


class PathMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id=None)
        self.poses = []


class RuntimeLimits(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = np.array([[p['x'], p['y']] for p in
                              json.loads((HERE/'global_path.json').read_text())['waypoints']])
        cls.smooth = smooth_control_route(cls.world)
        cls.lanes = HdMapLanes(HERE/'link_set.json')
        cls.lanes.set_route(cls.world)

    def run_branch(self, index=2611, speed_kmh=60., lidar_age=0., was_ready=False):
        runtime = Runtime.__new__(Runtime)
        runtime.highway = HighwayPlanner(dict(MISSION), self.world)
        runtime.vehicle_receiver = SimpleNamespace(
            snapshot=lambda: ([], True, dict(ready=True, vehicles=[])))
        runtime.object_receiver = SimpleNamespace(snapshot=lambda: [], age_s=.01)
        runtime.near_field_memory = NearFieldMemory()
        runtime.lidar = LidarCorridor()
        runtime.lidar.received = time.monotonic()-lidar_age
        runtime.lanes = self.lanes
        runtime.receiver = SimpleNamespace(rates=lambda: dict(front=30., left=30., right=30.))
        runtime.actions = ['DRIVE', 'STOP']
        runtime.gallery_matrix, runtime.gallery = np.empty((0, 2)), []
        if was_ready:
            runtime._highway_was_ready = True
        publisher = RosPathPublisher.__new__(RosPathPublisher)
        publisher.Float32 = publisher.String = Message
        publisher.PoseStamped, publisher.PathMessage = Pose, PathMessage
        publisher.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: 1.))
        for name in ('waypoints_pub', 'velocity_pub', 'mode_pub', 'path_pub', 'status_pub'):
            setattr(publisher, name, Sink())
        publisher.cruise_mps, publisher.collision_stop = 25., False
        publisher.publish_camera_rates = lambda rates: None
        publisher.publish_corridor = lambda ego, corridor: None
        runtime.path_publisher = publisher
        x, y = self.world[index]
        yaw = math.degrees(math.atan2(*(self.smooth[index+3]-self.smooth[index])[::-1]))
        ego = dict(x=x, y=y, heading_deg=yaw, speed_kmh=speed_kmh, age_s=.01)
        status = dict(ego=ego, path=dict(nearest_idx=index, distance_m=0.))
        dense = local_points(self.smooth[index:index+640], ego)
        prediction = np.column_stack((np.arange(1., 21.), np.zeros(20)))
        with tempfile.TemporaryDirectory() as directory, \
                patch.dict('os.environ', {'STUDENT_LOG_DIR': directory}):
            runtime.args = SimpleNamespace(status=Path(directory)/'status.json')
            self.assertTrue(runtime.infer_highway(status, ego, dense, dense[:20], dense[::10],
                prediction, np.array([.01, .99]), np.empty(0), np.empty(0), 3., time.perf_counter()))
            runtime._decision_log.close()
        return runtime._web_record, publisher.velocity_pub.items[-1].data

    def test_mildly_late_scan_keeps_the_checked_acceleration(self):
        record, published = self.run_branch(lidar_age=.32, was_ready=True)
        self.assertNotEqual(record['reason'], 'highway sensor late: hold speed')
        self.assertTrue(record['highway']['sensor_late_mild'])
        self.assertTrue(record['highway']['sensor_failure']['lidar_stale'])
        self.assertFalse(record['emergency_stop'])
        self.assertGreater(published, 60./3.6+.5)
        # Lane-change starts still wait for fresh data.
        self.assertFalse(record['highway']['perception_ready'])

    def test_late_scan_beyond_the_mild_bound_still_holds_speed(self):
        record, published = self.run_branch(lidar_age=.5, was_ready=True)
        self.assertEqual(record['reason'], 'highway sensor late: hold speed')
        self.assertFalse(record['highway']['sensor_late_mild'])
        self.assertLessEqual(published, 60./3.6+1e-6)

    def test_late_scan_without_a_previous_ready_frame_still_stops(self):
        record, published = self.run_branch(lidar_age=.32, was_ready=False)
        self.assertTrue(record['emergency_stop'])
        self.assertEqual(published, 0.)

    def test_rule_zone_is_a_hard_ceiling_below_measured_momentum(self):
        record, published = self.run_branch(index=2200, speed_kmh=63.)
        self.assertEqual(record['highway']['rule_speed_cap_kmh'], 58.)
        self.assertLessEqual(published, 58./3.6+1e-6)
        record, published = self.run_branch(index=2611, speed_kmh=63.)
        self.assertIsNone(record['highway']['rule_speed_cap_kmh'])
        self.assertGreater(published, 63./3.6)


class LaneChangePolicy(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        self.config = dict(from_idx=0, to_idx=1800, kmh=90., lane_change_policy="free_three_lane",
                           lane_change_goal=0, end_link="exit", comfort_decel_mps2=2.5,
                           comfort_jerk_mps3=4., lane_gain_kmh=2., lane_gain_hold_s=.3,
                           lane_cancel_gain_kmh=3.)
        self.planner = HighwayPlanner(self.config, self.route)
        self.lanes = [HighwayLane(str(k), k, self.route+np.array([0., k*3.5]),
                                  (str(k), "exit"), 800.) for k in range(3)]

    def decide(self, ego, objects, lanes=None, static=None):
        return self.planner.decide(0, self.route[:640], ego, objects, {}, lambda p, h: h,
            static or (lambda p, h: h), lane_options=self.lanes if lanes is None else lanes)

    def test_margins_and_stability_come_from_the_mission(self):
        policy = ThreeLanePolicy(self.config)
        self.assertEqual((policy.gain_kmh, policy.cancel_gain_kmh, policy.stable_s), (2., 3., .3))
        # The cancel margin is never below the start margin, and defaults to it.
        self.assertEqual(ThreeLanePolicy(dict(lane_gain_kmh=3., lane_cancel_gain_kmh=1.)).cancel_gain_kmh, 3.)
        self.assertEqual(ThreeLanePolicy(dict(lane_gain_kmh=2.5)).cancel_gain_kmh, 2.5)

    def test_brisk_change_leaves_a_slow_leaders_lane(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=45., stamp=10.)
        leader = car(22., 0., 30./3.6)
        first = self.decide(dict(ego, stamp=9.6), [leader])
        lengths = {round(c['transition_m'], 1) for c in first['status']['candidate_checks']
                   if c.get('lanes') == ['1'] and c.get('transition_m')}
        self.assertIn(round(max(policy_module.BRISK_CHANGE_MIN_M,
                                45./3.6*policy_module.BRISK_CHANGE_S), 1), lengths)
        result = self.decide(ego, [leader])
        self.assertIsNotNone(self.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '1')
        # Shorter than the 78 m four-second option used before.
        self.assertLess(result['status']['selected_transition_m'], 75.)
        # The brisk path is drivable well inside the highway curve limit.
        self.assertGreaterEqual(geometry_speed_cap(result['path'], 120./3.6), 45./3.6)

    def test_brisk_change_is_still_rejected_beside_a_target_lane_car(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=45., stamp=10.)
        objects = [car(22., 0., 30./3.6, uid=1), car(-1., 3.5, 45./3.6, uid=2),
                   car(-1., 7., 45./3.6, uid=3)]
        self.decide(dict(ego, stamp=9.6), objects)
        result = self.decide(ego, objects)
        self.assertIsNone(self.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '0')

    def test_stability_timer_belongs_to_the_first_leg(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=60., stamp=10.)
        objects = [car(45., 0., 25./3.6, uid=1), car(90., 3.5, 45./3.6, uid=2)]
        first = self.decide(ego, objects)
        self.assertIsNone(self.planner.saved_world)
        self.assertTrue(first['status']['further_overtake_requested'])
        self.assertEqual(self.planner.three_lane_policy.pending, 1)          # left
        # Next frame: the far lane is no longer offered, so the best sequence
        # changes from (1, 2) to (1,). The same first leg keeps its timer.
        later = dict(ego, stamp=10.+.31)
        result = self.decide(later, objects, lanes=self.lanes[:2])
        self.assertIsNotNone(self.planner.saved_world)
        self.assertEqual(result['status']['target_lane'], '1')

    def test_static_escape_reserve_keeps_gentle_planning_braking(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90., stamp=10.)
        result = self.decide(ego, [])
        speed = 25.
        reaction = .4+max(0., self.planner.longitudinal.static_sensor_age_s)+1.2/2.
        expected = speed*reaction+speed*speed/(2.*1.2)+12.
        self.assertAlmostEqual(result['status']['static_replan_reserve_m'], expected, places=6)


class CurvatureForecast(unittest.TestCase):
    def setUp(self):
        straight = np.column_stack((np.arange(0., 400., .5), np.zeros(800)))
        self.path = lane_change_path(straight, 3.5, 40.)       # brisk change, then straight
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=45., stamp=100.)

    def test_profile_starts_at_the_current_cap_and_releases_after_the_curve(self):
        stations, speeds = geometry_speed_profile(self.path, 120./3.6)
        self.assertAlmostEqual(speeds[0], geometry_speed_cap(self.path, 120./3.6), places=9)
        self.assertLess(speeds[0], 100./3.6)
        self.assertAlmostEqual(float(np.interp(80., stations, speeds)), 120./3.6, places=6)
        self.assertTrue(np.all(np.diff(stations) > 0.))

    def test_forecast_passes_the_curve_before_accelerating(self):
        model = HighwaySpeedPlanner(2.5, 4., 12.)
        model.begin(self.ego)
        cap = geometry_speed_cap(self.path, 120./3.6)
        profile = geometry_speed_profile(self.path, 120./3.6)
        held = model.preview(self.path, self.ego, (), cap, horizon_s=14.)
        released = model.preview(self.path, self.ego, (), 120./3.6, horizon_s=14.,
                                 speed_profile=profile)
        self.assertLessEqual(float(held.speeds.max()), cap+1e-6)
        self.assertGreater(float(released.speeds.max()), cap+5.)
        # Inside the curved part the forecast respects the envelope.
        inside = released.distances <= 30.
        limit = np.interp(released.distances[inside], *profile)
        self.assertTrue(np.all(released.speeds[inside] <= limit+2.5*.1+1e-6))


class CurveLimits(unittest.TestCase):
    def test_highway_curve_cap_uses_four_mps2(self):
        self.assertEqual(HIGHWAY_CURVE_ALAT_MPS2, 4.)
        radius = 60.
        angle = np.linspace(0., 1.5, 400)
        arc = np.column_stack((radius*np.sin(angle), radius-radius*np.cos(angle)))
        self.assertAlmostEqual(geometry_speed_cap(arc, 40.), math.sqrt(4.*radius), delta=.3)

    def test_lane_keeping_heading_correction_keeps_its_validated_limit(self):
        from planning.highway import merge_geometry
        self.assertEqual(policy_module.KEEP_PATH_CURVE_ALAT_MPS2, 3.5)
        self.assertEqual(merge_geometry.REPAIR_CURVE_ALAT_MPS2, 3.5)


class SurveyedBriskChange(unittest.TestCase):
    """follow-20261010T140051Z lap 2, 34-45 s: stuck in the on-ramp lane 411
    behind a 25 km/h car; every 78-200 m change to 409 left the mapped road
    15-23 m ahead ("lane change cannot finish inside mapped road")."""
    def test_brisk_change_from_the_ramp_lane_stays_on_the_mapped_road(self):
        route = np.array([[p['x'], p['y']] for p in
                          json.loads((HERE/'global_path.json').read_text())['waypoints']])
        smooth = smooth_control_route(route)
        lanes = HdMapLanes(HERE/'link_set.json')
        lanes.set_route(route)
        native = HighwayLaneMap(lanes, MISSION)
        geometry = lanes.geometry['A2256W000411']
        at = int(np.argmin(np.linalg.norm(geometry-route[2480], axis=1)))
        tangent = geometry[at+3]-geometry[at]
        ego = dict(x=geometry[at][0], y=geometry[at][1], speed_kmh=45., stamp=1.,
                   heading_deg=math.degrees(math.atan2(tangent[1], tangent[0])))
        local = local_points(smooth[2480:2480+640], ego)
        candidates = native.candidates(ego, local)
        corridor = native.corridor(ego, local, candidates)
        target = next(l for l in candidates if l.link_id == 'A2256W000409')
        gentle = lane_change_path(target.centre, 0., 78., match_heading=True)
        brisk = lane_change_path(target.centre, 0., max(policy_module.BRISK_CHANGE_MIN_M,
                                 45./3.6*policy_module.BRISK_CHANGE_S), match_heading=True)
        self.assertLess(corridor.clear_distance(gentle, 170.), 30.)
        self.assertGreaterEqual(corridor.clear_distance(brisk, 170.), 169.9)


if __name__ == '__main__':
    unittest.main()
