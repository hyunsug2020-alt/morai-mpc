"""Actual rear-axle body clearance in the opt-in moving junction policy."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from ego_body import body_corners
import ring_entry
from traffic_planner import GapPlanner


ROOT = Path(__file__).parent


def straight_frame():
    return ring_entry.PathFrame(np.column_stack((np.arange(0., 40.01, .5), np.zeros(81))))


def crossing_car(arc):
    obj = (12., -arc, 4.9, 'lidar_car', 'npc', 5., 4.9, 2.09, 90.,
           0., 5., 1, 0., None, True, 0.)
    conflict = dict(s_lane=12., s_cp=12., arc_lane=arc-2.4, arc_cp=arc, arc_exit=arc+2.4)
    return ring_entry.RingCar(obj, 5., 1.1, 'moving', conflict)


class ActualRearGeometryTests(unittest.TestCase):
    def assert_body_is_past_boundary(self, frame, boundary, target):
        """Independent dense body-edge projections, not the proof's half planes."""
        queries = []
        stations = np.r_[target, np.arange(target+.025, boundary+2.3175, .025), boundary+2.3175]
        for station in stations:
            xy = [np.interp(station, frame.s, frame.xy[:, axis]) for axis in (0, 1)]
            index = int(np.clip(np.searchsorted(frame.s, station), 1, len(frame.s)-1))
            direction = frame.xy[index]-frame.xy[index-1]
            heading = math.atan2(direction[1], direction[0])
            for error in (-6., 0., 6.):
                corners = body_corners(*xy, heading+math.radians(error))
                edges = np.vstack([corners[index]+np.linspace(0., 1., 9)[:, None]
                                   *(corners[(index+1) % 4]-corners[index]) for index in range(4)])
                for angle in np.arange(0., 2.*math.pi, math.pi/4.):
                    queries.append(edges+.1*np.array([math.cos(angle), math.sin(angle)]))
        projected, _, _ = frame.project(np.vstack(queries))
        self.assertGreaterEqual(float(np.min(projected)), boundary-1e-9)

    def test_straight_rear_bound_contains_heading_and_tracking_error(self):
        frame, boundary = straight_frame(), 10.
        target = ring_entry.body_rear_clear_station(frame, boundary)
        self.assertGreater(target-boundary, .79+.1)
        self.assertLess(target-boundary, 1.1)
        self.assert_body_is_past_boundary(frame, boundary, target)

    def test_native_surveyed_merge_clears_with_actual_body_corners(self):
        route = json.loads((ROOT/'global_path.json').read_text())['waypoints']
        frame = ring_entry.PathFrame([[p['x'], p['y']] for p in route[1700:1900]])
        links = json.loads((ROOT/'link_set.json').read_text())
        lane = np.asarray(next(link for link in links if link['idx']=='A2256W000134')['points'])[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(lane, axis=0), axis=1))]
        line = float(frame.project([[-99.3127, 317.799]])[0][0])
        conflict = ring_entry.lane_conflict(lane, arc, frame, line-5., line+30.)
        self.assertIsNotNone(conflict)
        boundary = conflict['s_cp']
        target = ring_entry.body_rear_clear_station(frame, boundary)
        self.assertLess(target-boundary, 1.5)
        self.assert_body_is_past_boundary(frame, boundary, target)

    def test_left_and_right_curves_keep_corner_clearance(self):
        for radius in (6., 10., 18.):
            for direction in (-1., 1.):
                arc = np.arange(0., 40.01, .25)
                frame = ring_entry.PathFrame(np.column_stack((radius*np.sin(arc/radius),
                    direction*radius*(1.-np.cos(arc/radius)))))
                boundary = float(frame.s[40])
                target = ring_entry.body_rear_clear_station(frame, boundary)
                with self.subTest(radius=radius, direction=direction):
                    self.assertLess(target-boundary, 2.3175)
                    self.assert_body_is_past_boundary(frame, boundary, target)

    def test_missing_sparse_nonfinite_and_self_crossing_frames_fall_back(self):
        sparse = ring_entry.PathFrame(np.column_stack((np.arange(0., 42., 2.), np.zeros(21))))
        invalid = SimpleNamespace(xy=np.array([[0., 0.], [math.nan, 0.], [20., 0.]]),
                                  s=np.array([0., 10., 20.]))
        loop_x = np.r_[np.arange(0., 10.01, .5), np.arange(9.5, -1.01, -.5),
                       np.arange(-.5, 30.01, .5)]
        loop = np.column_stack((loop_x, np.zeros(len(loop_x))))
        crossing = ring_entry.PathFrame(loop)
        for frame, boundary in ((None, 10.), (sparse, 10.), (invalid, 10.), (crossing, 31.)):
            with self.subTest(frame=type(frame).__name__):
                self.assertEqual(ring_entry.body_rear_clear_station(frame, boundary), boundary+2.3175)

    def test_sharp_corner_does_not_shorten_the_legacy_envelope(self):
        xy = np.vstack((np.column_stack((np.arange(0., 10.01, .5), np.zeros(21))),
                        np.column_stack((np.full(40, 10.), np.arange(.5, 20.01, .5)))))
        self.assertEqual(ring_entry.body_rear_clear_station(ring_entry.PathFrame(xy), 10.), 10.+2.3175)


class ActualRearPolicyTests(unittest.TestCase):
    def decide(self, arc, *, speed=0., geometry=True, legacy=False, frame=None):
        return ring_entry.entry_decision([crossing_car(arc)], 0., speed, 8.,
            frame=straight_frame() if frame is None else frame,
            yield_only=not legacy, body_geometry=geometry)

    def test_geometric_clearance_restores_a_slot_without_reducing_margin(self):
        checked = self.decide(32.)
        conservative = self.decide(32., geometry=False)
        self.assertEqual(conservative['action'], 'WAIT')
        self.assertEqual(checked['action'], 'GO')
        self.assertGreaterEqual(checked['checks'][0]['margin_s'], .55)
        self.assertEqual(ring_entry.NEXT_MARGIN_S, .55)
        self.assertEqual(ring_entry.LAUNCH_SLOW, (2.5, .5))
        self.assertEqual(self.decide(31.)['action'], 'WAIT')

    def test_restored_crossing_slot_has_no_actual_body_contact(self):
        self.assertEqual(self.decide(32.)['action'], 'GO')
        gaps = []
        for time in np.arange(0., 6.001, .005):
            moving = max(0., time-.5)
            distance = 1.25*moving*moving if moving <= 3.2 else 12.8+8.*(moving-3.2)
            ego = body_corners(distance, 0., 0.)
            npc_travel = 5.*time+time*time if time <= 1. else 6.+7.*(time-1.)
            npc_y = -32.+npc_travel
            npc = np.array([[12.-1.045, npc_y-2.45], [12.+1.045, npc_y+2.45]])
            lower, upper = np.min(ego, axis=0), np.max(ego, axis=0)
            across = np.maximum(np.maximum(npc[0]-upper, lower-npc[1]), 0.)
            gaps.append(float(np.linalg.norm(across)))
        self.assertGreater(min(gaps), .5)

    def test_stopped_and_moving_entry_keep_the_same_slow_rear_bound(self):
        times = [self.decide(32., speed=kmh/3.6)['checks'][0]['rear_clear_s'] for kmh in (0., 12., 20.)]
        self.assertEqual(times, [times[0]]*3)

    def test_required_rear_travel_matches_the_unrounded_body_target(self):
        frame, centre = straight_frame(), 3.
        car = crossing_car(32.)
        boundary = car.conflict['s_cp']+ring_entry.LANE_CONTACT_M
        for geometry in (False, True):
            with self.subTest(body_geometry=geometry):
                result = ring_entry.entry_decision([car], centre, 0., 8., frame=frame,
                    yield_only=True, body_geometry=geometry)
                target = (ring_entry.body_rear_clear_station(frame, boundary) if geometry
                          else boundary+ring_entry.EGO_HALF_LENGTH_M)
                self.assertEqual(result['checks'][0]['required_rear_travel_m'], target-centre)

    def test_legacy_entry_never_uses_the_new_body_proof(self):
        with patch('ring_entry.body_rear_clear_station', side_effect=AssertionError('legacy proof changed')):
            self.assertEqual(self.decide(32., legacy=True), self.decide(32., legacy=True, geometry=False))

    def test_unknown_motion_cannot_gain_entry_from_body_geometry(self):
        planner = GapPlanner()
        planner.junction = dict(held=True, ready=True, yield_only=True, empty_actual_body=True,
            line_gap_m=1., unknown_ids={7})
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        unknown = (10., 3., 4.9, 'lidar_car', 'npc', 0., 4.9, 2.09, 90.,
                   0., 0., 7, 0., None, False, 0.)
        with patch('ring_entry.body_rear_clear_station', side_effect=AssertionError('unknown got a proof')):
            result = planner.evaluate([unknown], ego, straight_frame().xy, 8., 100.)
        self.assertEqual(planner.status['state'], 'YIELD_WAIT')
        self.assertIn('without measured velocity', result['reason'])
        self.assertFalse(planner.junction_committed)


class LeaderSpeedBoundTests(unittest.TestCase):
    def decide(self, speed, ratio):
        lane = np.column_stack((np.arange(8., 80.01, .5), np.zeros(145)))
        arc = np.arange(0., 72.01, .5)
        obj = (8., 0., 4.9, 'lidar_car', 'npc', speed, 4.9, 2.09, 0.,
               speed, 0., 2, 0., None, True, 0.)
        conflict = dict(s_lane=10., s_cp=12., arc_lane=2., arc_cp=4.,
                        arc_exit=None, lane=(lane, arc))
        leader = ring_entry.RingCar(obj, speed, ratio, 'moving', conflict,
                                   on_path_along=8.)
        return ring_entry.entry_decision([leader, crossing_car(36.)], 0., 0., 8.,
            frame=straight_frame(), yield_only=True, body_geometry=True)

    def test_poor_quality_fit_cannot_clear_earlier_than_its_measured_lower_bound(self):
        # A 5.5 m/s fit with this quality bound permits a current 2 m/s leader.
        # A separate 2.6 m/s, ratio-1.3 observation has the same predicted speed.
        uncertain = self.decide(5.5, 2.75)
        measured_lower = self.decide(2.6, 1.3)
        self.assertEqual(uncertain['checks'], measured_lower['checks'])
        self.assertEqual(uncertain['action'], 'WAIT')
        self.assertGreater(uncertain['checks'][0]['rear_clear_s'], 7.)

    def test_good_fit_and_legacy_fallback_keep_the_future_slowdown(self):
        good_fit = self.decide(5.5, 1.1)
        fallback = self.decide(5.5, 1.3)
        self.assertEqual(good_fit, fallback)
        self.assertEqual(good_fit['action'], 'GO')
        self.assertGreaterEqual(good_fit['checks'][0]['margin_s'], .55)
        self.assertLess(good_fit['checks'][0]['rear_clear_s'], 5.)

    def test_wider_speed_uncertainty_never_shortens_rear_clearance(self):
        times = [self.decide(5.5, ratio)['checks'][0]['rear_clear_s']
                 for ratio in (1.1, 1.3, 1.8, 2.75)]
        self.assertEqual(times, sorted(times))
        self.assertGreater(times[-1], times[0]+3.)


class MeasuredLaunchStateTests(unittest.TestCase):
    def decide(self, arc=24., *, speed=2., launch=None, legacy=False):
        return ring_entry.entry_decision([crossing_car(arc)], 0., speed, 8.,
            frame=straight_frame(), yield_only=not legacy, body_geometry=True,
            launch_state=launch)

    def test_continuing_measured_launch_does_not_restart_the_full_rest_delay(self):
        pending = self.decide()
        launched = self.decide(launch=dict(v_mps=2., dead_s=0., stamp=100.))
        self.assertEqual(pending['action'], 'WAIT')
        self.assertEqual(launched['action'], 'GO')
        self.assertGreater(pending['checks'][0]['rear_clear_s'], launched['checks'][0]['rear_clear_s']+.9)
        self.assertEqual(pending['checks'][0]['earliest_s'], launched['checks'][0]['earliest_s'])
        self.assertEqual(pending['checks'][0]['required_rear_travel_m'],
                         launched['checks'][0]['required_rear_travel_m'])
        self.assertGreaterEqual(launched['checks'][0]['margin_s'], .55)
        self.assertEqual(ring_entry.NEXT_MARGIN_S, .55)
        self.assertEqual(ring_entry.LAUNCH_SLOW, (2.5, .5))

    def test_pending_braking_and_invalid_lower_envelopes_retain_the_rest_model(self):
        default = self.decide()
        self.assertEqual(default, self.decide(launch=dict(v_mps=0., dead_s=.5, stamp=100.)))
        for state in (dict(v_mps=-.01, dead_s=0., stamp=100.),
                      dict(v_mps=2.00001, dead_s=0., stamp=100.),
                      dict(v_mps=2., dead_s=-.01, stamp=100.),
                      dict(v_mps=2., dead_s=.50001, stamp=100.),
                      dict(v_mps=2., dead_s=math.nan, stamp=100.),
                      dict(v_mps=True, dead_s=0., stamp=100.),
                      dict(v_mps=2., dead_s=0., stamp=math.inf),
                      dict(v_mps=2., dead_s=0.)):
            with self.subTest(state=state):
                self.assertEqual(default, self.decide(launch=state))

    def test_state_must_match_the_exact_current_pose_stamp(self):
        ego = dict(speed_kmh=7.2, stamp=100., brake=0., accel=.3,
                   _junction_launch=dict(v_mps=2., dead_s=.1, stamp=100.))
        self.assertEqual(ring_entry.pose_launch_state(ego), ego['_junction_launch'])
        for stamp in (99.999999, 100.000001, math.nan, 0.):
            with self.subTest(stamp=stamp):
                self.assertIsNone(ring_entry.pose_launch_state(dict(ego, stamp=stamp)))
        self.assertIsNone(ring_entry.pose_launch_state(dict(ego, speed_kmh=7.19)))
        self.assertIsNone(ring_entry.pose_launch_state(dict(speed_kmh=7.2, stamp=100.)))
        for changes in (dict(brake=1.), dict(brake=math.nan), dict(accel=0.), dict(accel=math.nan)):
            self.assertIsNone(ring_entry.pose_launch_state(dict(ego, **changes)))
        for missing in ('brake', 'accel'):
            incomplete = dict(ego)
            incomplete.pop(missing)
            self.assertIsNone(ring_entry.pose_launch_state(incomplete))

    def test_legacy_entry_ignores_optional_launch_state(self):
        default = self.decide(legacy=True)
        self.assertEqual(default,
            self.decide(legacy=True, launch=dict(v_mps=2., dead_s=0., stamp=100.)))

    def test_repeated_current_pose_checks_preserve_a_continuous_safe_launch(self):
        frame, initial_arc = straight_frame(), 26.
        target = ring_entry.body_rear_clear_station(frame, 12.+ring_entry.LANE_CONTACT_M)
        actual_clear_time = 2.4+(target-12.)/8.
        default_waits = 0
        for elapsed in np.arange(0., 2.701, .1):
            speed = min(8., 2.+2.5*elapsed)
            station = (2.*elapsed+1.25*elapsed*elapsed if elapsed <= 2.4
                       else 12.+8.*(elapsed-2.4))
            npc_travel = 5.*elapsed+elapsed*elapsed if elapsed <= 1. else 6.+7.*(elapsed-1.)
            car = crossing_car(initial_arc-npc_travel)
            state = dict(v_mps=speed, dead_s=0., stamp=100.+elapsed)
            result = ring_entry.entry_decision([car], station, speed, 8., frame=frame,
                yield_only=True, body_geometry=True, launch_state=state)
            pending = ring_entry.entry_decision([car], station, speed, 8., frame=frame,
                yield_only=True, body_geometry=True)
            default_waits += pending['action'] == 'WAIT'
            self.assertEqual(result['action'], 'GO', (elapsed, result))
            self.assertGreaterEqual(result['checks'][0]['margin_s'], .55)
            self.assertGreaterEqual(elapsed+result['checks'][0]['rear_clear_s'], actual_clear_time-.011)
        self.assertGreater(default_waits, 0)
        distances = []
        for elapsed in np.arange(0., 5.001, .005):
            station = (2.*elapsed+1.25*elapsed*elapsed if elapsed <= 2.4
                       else 12.+8.*(elapsed-2.4))
            ego = body_corners(station, 0., 0.)
            travel = 5.*elapsed+elapsed*elapsed if elapsed <= 1. else 6.+7.*(elapsed-1.)
            npc_y = -initial_arc+travel
            npc_lower, npc_upper = np.array([10.955, npc_y-2.45]), np.array([13.045, npc_y+2.45])
            gap = np.maximum(np.maximum(npc_lower-np.max(ego, axis=0),
                                         np.min(ego, axis=0)-npc_upper), 0.)
            distances.append(float(np.linalg.norm(gap)))
        self.assertGreater(min(distances), .5)


if __name__ == '__main__':
    unittest.main()
