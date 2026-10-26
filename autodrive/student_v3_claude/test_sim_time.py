"""Simulator-time units: measured motion vs ego motion on a slow MORAI (RTF < 1).

follow-20261010T101327Z highway 19:18:06-22: RTF ~0.57. The 40 km/h lead 30 m
ahead read 23.1 km/h (LiDAR, wall-clock velocity) while the ego reported 40 km/h
(simulator speed) and the gap stayed constant for 15 s; the speed plan held the
ego at 40.5 km/h behind a car it believed 17 km/h slower.
"""
import math
import unittest
from types import SimpleNamespace

import numpy as np

import gui  # noqa: F401  (puts the shared autodrive modules on sys.path)
from highway_speed_plan import HEADWAY_S, MIN_GAP_M, HighwaySpeedPlanner, following_gap_m
from ego_body import EGO_FRONT_M
from lead_follow import (EMERGENCY_GAP_M, STANDSTILL_GAP_M, LeadTracker, follow_cap_mps,
                         lead_cap, needs_emergency_stop)
from path_safety import PathSafety
from sim_time import (MEASURED_OBSTACLE_MOTION_MPS, SimClock, clip_rtf, sim_mps, sim_seconds,
                      to_sim_objects)
from stop_line import StopLineController
from test_ped_guard import EGO, PATH

RTF = .57


def car(x, y, measured_mps, *, name='lidar_car', heading=0., uid=500021, age=0.):
    h = math.radians(heading)
    return (x, y, 4.9, name, 'npc', measured_mps, 4.9, 2.1, heading,
            measured_mps*math.cos(h), measured_mps*math.sin(h), uid, 0., None, True, age)


class ClockTests(unittest.TestCase):
    def drive(self, clock, rtf, speed_mps=11.1, seconds=4., dt=1/15., stamp0=1000.):
        x = 0.
        for k in range(int(seconds/dt)+1):
            clock.update(dict(x=x, y=0., speed_kmh=speed_mps*3.6, stamp=stamp0+k*dt), 5.+k*dt)
            x += speed_mps*rtf*dt     # the pose moves the wall-clock distance
        return x

    def test_unknown_factor_is_one_until_measured(self):
        clock = SimClock()
        self.assertEqual(clock.rtf, 1.)
        self.assertFalse(clock.valid)
        for k in range(30):   # standing since the start: no evidence
            clock.update(dict(x=0., y=0., speed_kmh=0., stamp=10.+k*.1), k*.1)
        self.assertEqual(clock.rtf, 1.)
        self.assertFalse(clock.record()['rtf_measured'])

    def test_measures_the_factor_from_ego_motion_and_keeps_it_at_rest(self):
        clock = SimClock()
        self.drive(clock, RTF)
        self.assertTrue(clock.valid)
        self.assertAlmostEqual(clock.rtf, RTF, delta=.02)
        for k in range(40):   # stop-line hold: no new evidence, last factor kept
            clock.update(dict(x=50., y=0., speed_kmh=0., stamp=1010.+k*.1), 20.+k*.1)
        self.assertAlmostEqual(clock.rtf, RTF, delta=.02)

    def test_factor_is_clipped_to_real_time(self):
        clock = SimClock()
        self.drive(clock, 1.15)
        self.assertEqual(clock.rtf, 1.)
        self.assertEqual(clip_rtf(0.), 1.)
        self.assertEqual(clip_rtf(float('nan')), 1.)
        self.assertEqual(clip_rtf(.1), .3)

    def test_stalled_pose_cannot_pull_the_factor_down(self):
        clock = SimClock()
        x = self.drive(clock, RTF)
        for k in range(45):   # new stamps, speed 40 km/h, pose frozen
            clock.update(dict(x=x, y=0., speed_kmh=40., stamp=1004.1+k/15.), 10.+k/15.)
        self.assertAlmostEqual(clock.rtf, RTF, delta=.02)

    def test_a_drop_is_followed_more_slowly_than_a_rise(self):
        rising, falling = SimClock(.5), SimClock(.9)
        rising.valid = falling.valid = True
        for clock in (rising, falling):
            self.drive(clock, .7, seconds=2.5)
        self.assertGreater(rising.rtf-.5, .9-falling.rtf)
        self.assertGreater(falling.rtf, .7)

    def test_pose_gap_restarts_the_window(self):
        clock = SimClock()
        self.drive(clock, RTF)
        before = clock.rtf
        # A 5 s pose outage (or a teleport) is not driven distance.
        clock.update(dict(x=500., y=0., speed_kmh=40., stamp=1010.), 30.)
        self.assertEqual(len(clock._samples), 1)
        self.assertEqual(clock.rtf, before)


class ConversionTests(unittest.TestCase):
    def test_vehicle_velocity_and_turn_rate_become_simulator_units(self):
        row = list(car(30., 0., 6.33, heading=10.))
        row[12] = .2
        (converted,) = to_sim_objects([tuple(row)], RTF)
        self.assertAlmostEqual(converted[5], 6.33/RTF)
        self.assertAlmostEqual(math.hypot(converted[9], converted[10]), 6.33/RTF)
        self.assertAlmostEqual(converted[12], .2/RTF)
        # Positions, size, heading and the detector's wall prediction age stay.
        self.assertEqual(converted[:5], tuple(row[:5]))
        self.assertEqual(converted[6:9], tuple(row[6:9]))
        self.assertEqual(converted[15], row[15])

    def test_propagation_is_the_same_displacement_in_both_units(self):
        measured = car(30., 0., 6.33)
        (converted,) = to_sim_objects([measured], RTF)
        age = .18
        self.assertAlmostEqual(measured[9]*age, converted[9]*sim_seconds(age, RTF))
        self.assertAlmostEqual(sim_mps(6.33, RTF), converted[5])

    def test_refined_people_are_never_converted_twice(self):
        person = (5., 2., .6, 'fusion_pedestrian', 'pedestrian', 1.4, .6, .6, 0., 0., -1.4, 7)
        self.assertEqual(to_sim_objects([person], RTF, people=False), [person])
        (raw,) = to_sim_objects([person], RTF)
        self.assertAlmostEqual(raw[10], -1.4/RTF)

    def test_box_jitter_stays_static_and_tunnel_metadata_is_kept(self):
        jitter = (12., 3., 1., 'fusion_obstacle', 'obstacle', .9, 1., 1., 0., .9, 0., 155)
        (static,) = to_sim_objects([jitter], RTF)
        self.assertEqual((static[5], static[9], static[10]), (0., 0., 0.))
        self.assertLess(.9, MEASURED_OBSTACLE_MOTION_MPS)
        mover = (12., 3., 1., 'fusion_obstacle', 'obstacle', 1.2, 1., 1., 0., 1.2, 0., 156,
                 dict(point_count=9))
        (moving,) = to_sim_objects([mover], RTF)
        self.assertAlmostEqual(moving[5], 1.2/RTF)
        self.assertEqual(moving[12], dict(point_count=9))

    def test_real_time_is_identity_for_movers(self):
        rows = [car(30., 0., 11.1), car(-20., 3.5, 20., uid=2)]
        self.assertEqual(to_sim_objects(rows, 1.), rows)


class HighwayFollowingTests(unittest.TestCase):
    PATH = np.column_stack((np.arange(0., 400., .5), np.zeros(800)))

    def preview(self, speed, gap, measured_lead_mps, rtf):
        planner = HighwaySpeedPlanner(1.2, 2., 12.)
        # Highway localization is the rear axle: build the requested bumper
        # gap at the actual front, while keeping the RTF assertions unchanged.
        lead = car(EGO_FRONT_M+gap+4.9/2., 0., measured_lead_mps)
        objects = to_sim_objects([lead], rtf)
        return planner.preview(self.PATH, dict(EGO, speed_kmh=speed*3.6), objects, 120./3.6)

    def test_equal_speed_lead_at_rtf_057_is_followed_at_the_headway_gap(self):
        for kmh in (40., 60., 90.):
            with self.subTest(kmh=kmh):
                speed = kmh/3.6
                gap = following_gap_m(speed)
                trajectory = self.preview(speed, gap, speed*RTF, RTF)
                # Steady following: neither closing in nor dropping back.
                self.assertLess(np.max(np.abs(trajectory.speeds-speed)), .05)
                self.assertAlmostEqual(trajectory.reference_mps, speed, delta=.05)
                lead = gap+speed*trajectory.times
                np.testing.assert_allclose(lead-trajectory.distances, gap, atol=.1)

    def test_steady_gaps_at_40_60_90_kmh(self):
        expected = {40.: 7.22, 60.: 10.83, 90.: 16.25}   # 0.65 s settled headway
        for kmh, gap in expected.items():
            self.assertAlmostEqual(following_gap_m(kmh/3.6), gap, places=2)
        self.assertEqual((HEADWAY_S, MIN_GAP_M), (.75, 4.))
        self.assertEqual(following_gap_m(0.), 4.)
        # A gap 2 m longer is closed (the lead is not believed slower) ...
        speed = 60./3.6
        closing = self.preview(speed, following_gap_m(speed)+2., speed*RTF, RTF)
        self.assertGreater(closing.reference_mps, speed)
        # ... and 2 m shorter opens with a gentle brake, no emergency.
        opening = self.preview(speed, following_gap_m(speed)-2., speed*RTF, RTF)
        self.assertLess(opening.reference_mps, speed)
        self.assertGreater(float(opening.accelerations.min()), -1.21)

    def test_unconverted_lead_speed_reproduces_the_40kmh_hold(self):
        # The recorded case: 30 m gap at 40 km/h, lead measured 23 km/h.
        speed = 40./3.6
        held = self.preview(speed, 30., 23.1/3.6, 1.)
        fixed = self.preview(speed, 30., 23.1/3.6, RTF)
        self.assertLess(held.speeds[-1], speed-2.)        # believed closing: slows down
        self.assertGreater(fixed.reference_mps, speed)     # true equal speed: closes the gap
        self.assertGreater(fixed.speeds[-1], speed+1.)

    def test_ego_acceleration_is_per_simulated_second(self):
        planner = HighwaySpeedPlanner()
        planner.time_scale = RTF
        planner.begin(dict(stamp=10., speed_kmh=36.))
        planner.begin(dict(stamp=10.2, speed_kmh=36.-1.*3.6*.2*RTF))   # -1 m/s^2 simulated
        planner.begin(dict(stamp=10.4, speed_kmh=36.-2.*1.*3.6*.2*RTF))
        self.assertAlmostEqual(planner.acceleration, -1., delta=.05)


class HighwayRuntimeUnitsTests(unittest.TestCase):
    def test_planner_gets_simulator_units_once_and_the_record_logs_the_factor(self):
        from unittest.mock import patch
        import highway_runtime
        from highway_planner import HighwayPlanner, world_points
        from test_highway_integration import HighwayIntegration
        HighwayIntegration.setUpClass()
        case = HighwayIntegration('test_three_lane_runtime_publishes_highway_mode_and_same_speed_reference')
        seen = {}
        decide, plan_speed = HighwayPlanner.decide, HighwayPlanner.plan_speed

        def capture_decide(planner, index, route, ego, objects, *args, **kwargs):
            seen['decide'] = list(objects)
            return decide(planner, index, route, ego, objects, *args, **kwargs)

        def capture_speed(planner, path, ego, objects, *args, **kwargs):
            seen['speed'] = list(objects)
            return plan_speed(planner, path, ego, objects, *args, **kwargs)

        def lead(ego):
            x, y = world_points(np.array([[30., 0.]]), ego)[0]
            h = math.radians(ego['heading_deg'])
            v = 40./3.6*RTF
            return [(x, y, 4.6, 'fusion_npc', 'npc', v, 4.6, 1.9, ego['heading_deg'],
                     v*math.cos(h), v*math.sin(h), 116)]
        with patch.object(highway_runtime, 'runtime_rtf', return_value=RTF), \
                patch.object(HighwayPlanner, 'decide', capture_decide), \
                patch.object(HighwayPlanner, 'plan_speed', capture_speed):
            runtime, _ = case.run_branch(highway_policy='free_three_lane', objects=lead, speed_kmh=40.)
        record = runtime._web_record
        logged = next(o for o in record['object_tracks'] if o[3] == 'fusion_npc')
        for stage in ('decide', 'speed'):
            planned = next(o for o in seen[stage] if o[3] == 'fusion_npc')
            self.assertAlmostEqual(planned[5], logged[5]/RTF)
            self.assertAlmostEqual(planned[9], logged[9]/RTF)
            self.assertEqual(tuple(planned[:2]), tuple(logged[:2]))
        self.assertAlmostEqual(logged[5], 40./3.6*RTF)   # the log keeps measured units
        self.assertEqual(record['rtf'], RTF)
        self.assertEqual(record['highway']['speed_plan']['rtf'], RTF)
        self.assertEqual(runtime.highway.longitudinal.time_scale, RTF)


class PathSafetyUnitTests(unittest.TestCase):
    def check(self, obj, rtf, speed_kmh=40., requested=None):
        requested = speed_kmh/3.6+3. if requested is None else requested
        return PathSafety().check(PATH, dict(EGO, speed_kmh=speed_kmh),
            [('vehicles', [obj], 100., 0.)], 10., requested_mps=requested,
            dynamic_only=True, motion_time_scale=rtf)

    def test_lead_speed_is_compared_in_simulator_units_once(self):
        speed = 40./3.6
        measured = car(14., 0., speed*RTF)
        simulated = car(14., 0., speed)
        converted = self.check(measured, RTF)
        reference = self.check(simulated, 1.)
        # One conversion: wall-clock motion at RTF 0.57 equals the same motion
        # expressed in simulator units (no second scaling of the ego).
        self.assertAlmostEqual(converted.cap_mps, reference.cap_mps, places=6)
        self.assertGreaterEqual(converted.cap_mps, speed)
        self.assertFalse(converted.emergency)
        self.assertEqual(converted.validation['time_scale'], RTF)
        # The old reading (measured speed as simulator speed) lowered the cap.
        self.assertLess(self.check(measured, 1.).cap_mps, converted.cap_mps-1.)

    def test_crossing_time_matches_between_unit_systems(self):
        from test_ped_guard import person
        batch_wall = [('semantic', [person(12., 5., 0., -4.*.5)], 100., 0.)]
        batch_sim = [('semantic', [person(12., 5., 0., -4.)], 100., 0.)]
        ego = dict(EGO, speed_kmh=30.)
        wall = PathSafety().check(PATH, ego, batch_wall, 10., requested_mps=30./3.6,
                                  motion_time_scale=.5)
        sim = PathSafety().check(PATH, ego, batch_sim, 10., requested_mps=30./3.6,
                                 motion_time_scale=1.)
        self.assertEqual(wall.emergency, sim.emergency)
        self.assertTrue(sim.emergency)


class StopLineTests(unittest.TestCase):
    LINE = dict(id='entry', station_m=50., xy=[50., 0.])

    def release_time(self, rtf, step=.05):
        gate = StopLineController([self.LINE])
        station = 50.-gate.FRONT_M-1.
        now = 0.
        while now < 5.:
            result = gate.update(station, 0., now, True, time_scale=rtf)
            if result['state'] == 'ENTER':
                return now, result
            self.assertTrue(result['stop'])
            now = round(now+step, 6)
        return None, result

    def test_one_simulated_second_at_rtf_057(self):
        released, _ = self.release_time(RTF)
        self.assertGreaterEqual(released*RTF, StopLineController.HOLD_S-1e-9)
        self.assertLess(released, StopLineController.HOLD_S/RTF+.05+1e-9)

    def test_never_shorter_than_one_wall_second(self):
        released, _ = self.release_time(1.4)
        self.assertAlmostEqual(released, StopLineController.HOLD_S)
        released, _ = self.release_time(1.)
        self.assertAlmostEqual(released, StopLineController.HOLD_S)

    def test_hold_progress_is_reported_in_simulator_seconds(self):
        gate = StopLineController([self.LINE])
        station = 50.-gate.FRONT_M-1.
        for k in range(11):
            result = gate.update(station, 0., k*.1, False, time_scale=RTF)
        self.assertEqual(result['state'], 'HOLD')
        self.assertAlmostEqual(result['held_sim_s'], RTF, places=3)
        self.assertEqual(result['time_scale'], RTF)


class LeadFollowTests(unittest.TestCase):
    def test_standstill_gap(self):
        self.assertEqual(STANDSTILL_GAP_M, 3.5)
        self.assertEqual(follow_cap_mps(STANDSTILL_GAP_M, 0.), 0.)
        self.assertGreater(follow_cap_mps(STANDSTILL_GAP_M+.5, 0.), 0.)
        self.assertLess(EMERGENCY_GAP_M, STANDSTILL_GAP_M)
        # Equal-speed steady gap 3.5 + 0.5 v + 0.075 v^2: 18.3 m at 40 km/h.
        v = 40./3.6
        self.assertAlmostEqual(follow_cap_mps(STANDSTILL_GAP_M+.5*v+.075*v*v, v), v, places=6)

    def test_lidar_gap_slope_and_odometry_use_simulated_time(self):
        # Ego at 11.1 m/s (simulator), RTF 0.57; the LiDAR gap shrinks 1 m per wall
        # second: the lead's true speed is 11.1 - 1/0.57 m/s.
        tracker = LeadTracker()
        ego, state = 11.1, None
        for k in range(16):
            now = k*.1
            state = tracker.update(now, ego, dict(dist_m=30.-1.*now), (), scan_stamp=now, time_scale=RTF)
        self.assertTrue(state['known'])
        self.assertAlmostEqual(state['v_lead'], ego-1./RTF, delta=.05)

    def test_runtime_converts_the_measured_lead(self):
        runtime = gui.Runtime.__new__(gui.Runtime)
        runtime.vehicle_receiver = None
        runtime.lidar = None
        path = np.column_stack((np.arange(0., 80., .5), np.zeros(160)))
        v = 40./3.6
        steady = STANDSTILL_GAP_M+.5*v+.075*v*v
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=40.)
        fused = car(2.3175+steady+4.9/2., 0., v*RTF, name='fusion_npc', uid=116)
        for rtf, closes in ((RTF, False), (1., True)):
            with self.subTest(rtf=rtf):
                runtime.sim_clock = SimClock(rtf)
                runtime._lead_tracker = None
                result = runtime.lead_follow_step(ego, 10., [fused], None, path, False, False)
                self.assertTrue(result['state']['known'])
                if closes:
                    self.assertLess(result['cap_mps'], v-2.)          # the old reading
                else:
                    self.assertAlmostEqual(result['state']['v_lead'], v, places=3)
                    self.assertAlmostEqual(result['cap_mps'], v, delta=.05)
                    self.assertFalse(result['emergency'])
                    self.assertEqual(runtime._lead_state['rtf'], RTF)

    def test_moving_conflict_follows_the_simulator_speed(self):
        v = 30./3.6
        lead = car(8., 0., v*RTF, name='fusion_npc')
        result = gui.moving_conflict(to_sim_objects([lead], RTF), EGO, PATH, v)
        self.assertEqual(result['kind'], 'follow')
        self.assertAlmostEqual(result['v_mps'], v, places=6)


class TunnelUnitsTests(unittest.TestCase):
    def test_tunnel_timed_conflict_and_lead_use_simulator_units(self):
        from unittest.mock import patch
        import test_tunnel_plan_contract as contract
        fixture = contract.TunnelPlanContractTests('test_six_meter_limit_also_stops_before_selecting_d')
        runtime = fixture.runtime()
        runtime.sim_clock = SimClock(RTF)
        decision = fixture.plan(runtime)
        cluster = (15., 0., .8, 'tunnel_lidar_dynamic_7', 'pedestrian', 1., .8, .6, 0., 1., 0., 7,
                   dict(point_count=12))
        detector_car = car(25., 0., 5.)
        seen = {}

        def capture(objects, *args, **kwargs):
            seen['moving'] = objects
            return None
        with patch('gui.moving_conflict', capture):
            runtime.tunnel_traffic_checks(decision, [detector_car], np.empty((0, 2)),
                                          [cluster, detector_car])
        moving = {o[11]: o for o in seen['moving']}
        self.assertAlmostEqual(moving[7][5], 1./RTF)                  # cluster, converted once
        self.assertEqual(moving[7][12], dict(point_count=12))
        self.assertAlmostEqual(moving[500021][5], 5./RTF)
        state = runtime._lead_state
        self.assertAlmostEqual(state['v_lead_kmh'], 5./RTF*3.6, places=1)
        self.assertEqual(state['rtf'], RTF)


class JunctionUnitsTests(unittest.TestCase):
    def test_live_junction_check_times_ring_cars_in_simulator_units(self):
        from unittest.mock import patch
        import ring_entry
        from test_live_path_safety import LivePathSafetyTests
        from test_junction_entry import car as ring_car
        runtime, plan = LivePathSafetyTests().junction([ring_car(20., 12., 0., -4., 14)])
        runtime.sim_clock = SimClock(RTF)
        seen = {}
        follow_cap = ring_entry.follow_cap

        def capture(objects, *args, **kwargs):
            seen['objects'] = list(objects)
            return follow_cap(objects, *args, **kwargs)
        with patch.object(ring_entry, 'follow_cap', capture):
            runtime.path_publisher._safety_provider(plan.path, plan.control_ego, plan.mode, plan)
        (planned,) = [o for o in seen['objects'] if o[11] == 14]
        self.assertAlmostEqual(planned[5], 4./RTF)
        self.assertAlmostEqual(planned[10], -4./RTF)


class NormalRoadUnitsTests(unittest.TestCase):
    def test_timed_traffic_checks_get_simulator_units_and_the_record_logs_rtf(self):
        from unittest.mock import patch
        from test_avoidance_gate import AvoidanceGateTests
        from traffic_planner import GapPlanner
        lead = car(20., 0., 6., name='fusion_npc', uid=116)
        walker = (12., 4., .6, 'fusion_pedestrian', 'pedestrian', 1.2, .6, .6, 0., 0., -1.2, 9)
        runtime = AvoidanceGateTests().runtime(action='KEEP', objects=[lead, walker])
        runtime.sim_clock = SimClock(RTF)
        seen = {}
        evaluate, conflict = GapPlanner.evaluate, gui.moving_conflict

        def capture_gap(planner, objects, *args, **kwargs):
            seen['gap'] = list(objects)
            return evaluate(planner, objects, *args, **kwargs)

        def capture_conflict(objects, *args, **kwargs):
            seen.setdefault('conflict', list(objects))
            return conflict(objects, *args, **kwargs)
        with patch.object(GapPlanner, 'evaluate', capture_gap), \
                patch.object(gui, 'moving_conflict', capture_conflict):
            self.assertTrue(runtime.infer())
        (planned,) = [o for o in seen['gap'] if o[11] == 116]
        self.assertAlmostEqual(planned[5], 6./RTF)
        # People are converted by PedTracker with the same factor (never again
        # at the timed checks: to_sim_objects(people=False)).
        self.assertAlmostEqual(runtime._ped_tracker.rtf, RTF)
        self.assertTrue(all(o[4] != 'pedestrian' or abs(o[10]) <= 1.2/RTF+1e-6
                            for o in seen.get('conflict', ())))
        self.assertEqual(runtime._web_record['rtf'], RTF)
        self.assertAlmostEqual(runtime._web_record['lead']['v_lead_kmh'], 6./RTF*3.6, places=1)


class LiveCheckerClockTests(unittest.TestCase):
    def test_live_checker_uses_the_runtime_clock(self):
        from live_path_safety import LivePathSafety
        runtime = SimpleNamespace(sim_clock=SimClock(RTF))
        checker = LivePathSafety.__new__(LivePathSafety)
        checker.runtime, checker.motion_clock = runtime, SimClock()
        self.assertEqual(checker.motion_rtf(dict(x=0., y=0., speed_kmh=40.), 1.), RTF)
        checker.runtime = SimpleNamespace()
        self.assertEqual(checker.motion_rtf(dict(x=0., y=0., speed_kmh=40.), 1.), 1.)


if __name__ == '__main__':
    unittest.main()
