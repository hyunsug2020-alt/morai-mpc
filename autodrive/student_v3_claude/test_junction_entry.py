"""Roundabout entry ("끼어들기"): track dropouts, wait/go at the line, committed entry.

Live run 222 (t 252-264 s): a circulating car 10 m to our left vanished from the
LiDAR vehicle list for 1.1 s, came back under a new id without velocity, and the
gap planner read both as a gap; we drove into its path and stopped beside it.
User rules 2026-10-10: after the 1 s stop enter right behind the passing car's rear,
never in front of an approaching one; once committed only decelerate (ring_entry).
"""
import json
from pathlib import Path
import unittest

import numpy as np

from traffic_planner import GapPlanner, JunctionTrackMemory


def car(x, y, vx, vy, uid, valid=True, heading=None):
    heading = np.degrees(np.arctan2(vy, vx)) if heading is None else heading
    return (float(x), float(y), 4.6, "lidar_car", "npc", float(np.hypot(vx, vy)) if valid else 0.,
            4.6, 1.9, float(heading), float(vx) if valid else 0., float(vy) if valid else 0., uid, 0., None, bool(valid))


class JunctionTrackMemoryTests(unittest.TestCase):
    def test_lost_car_is_kept_as_predicted_ghost(self):
        memory = JunctionTrackMemory()
        memory.update([car(0., 10., 0., -3., 7)], 0.)
        out = memory.update([], 1.0)
        self.assertEqual(memory.ghost_ids, {7})
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0][1], 7.0, places=3)      # moved on at 3 m/s
        self.assertTrue(out[0][14])
        self.assertEqual(memory.update([], 2.5), [])            # forgotten after HOLD_S

    def test_new_track_without_velocity_inherits_lost_velocity(self):
        memory = JunctionTrackMemory()
        memory.update([car(0., 10., 0., -3., 7)], 0.)
        memory.update([], .6)
        out = memory.update([car(.5, 6.9, 0., 0., 8, valid=False)], 1.1)
        self.assertEqual(memory.inherited_ids, {8})
        self.assertEqual(memory.ghost_ids, set())               # re-identified, no duplicate
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0][10], -3.)
        self.assertTrue(out[0][14])

    def test_unmatched_new_track_is_unknown(self):
        memory = JunctionTrackMemory()
        out = memory.update([car(20., 20., 0., 0., 9, valid=False)], 0.)
        self.assertEqual(memory.unknown_ids, {9})
        self.assertFalse(out[0][14])


class JunctionEntryTests(unittest.TestCase):
    def test_recorded_ring_car_retains_its_nearby_lane_and_crossing_check(self):
        root = Path(__file__).parent
        data = json.loads((root/'test_data/roundabout_native32_map_go.json').read_text())
        npc = data['npc']
        obj = list(car(npc['x'], npc['y'], npc['vx'], npc['vy'], npc['uid'],
                       heading=npc['heading_deg']))
        obj[6:8] = [npc['length_m'], npc['width_m']]
        links = json.loads((root/'link_set.json').read_text())
        planner = GapPlanner(links)
        planner.junction = dict(following_only=True, ready=True, empty_actual_body=True)
        ego = dict(data['ego'], stamp=100., age_s=.02)
        yaw = np.radians(ego['heading_deg'])
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        local = (np.asarray(data['world_path'])-[ego['x'], ego['y']])@rotation
        result = planner.evaluate([tuple(obj)], ego, local, 24./3.6, 10.)
        self.assertEqual(planner.status['state'], 'CROSSING_CAUTION')
        checks = planner.status['ring']['checks']
        self.assertIn(npc['uid'], [item['object_id'] for item in checks])
        self.assertLess(result['v_mps'], 24./3.6)
        # Uncertain track identity must retain all plausible map hypotheses.
        heading = np.arctan2(npc['vy'], npc['vx'])
        for field in ('unknown_ids', 'ghost_ids', 'inherited_ids'):
            planner.junction[field] = {npc['uid']}
            original = planner.map_predictor.lane_paths(obj[:2], heading, 70.)
            retained = planner._lane_paths(tuple(obj), heading, np.array(obj[9:11]))
            self.assertEqual(len(retained), len(original))
            planner.junction.pop(field)

    """Straight entry path along +x; a car approaching our path from the left (+y)."""
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 40.01, .5), np.zeros(81)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)

    def planner(self, held=False, unknown=()):
        planner = GapPlanner()
        planner.junction = dict(held=held, ready=True, unknown_ids=set(unknown), line_gap_m=None)
        return planner

    def run_until(self, planner, objects, t0=0., t1=1.0, ego=None):
        result = None
        for t in np.arange(t0, t1 + 1e-9, 1. / 15.):
            result = planner.evaluate(objects, ego or self.ego, self.path, 8., float(t))
        return result

    def test_unknown_velocity_track_blocks_entry(self):
        planner = self.planner(unknown={5})
        result = planner.evaluate([car(6., 12., 0., 0., 5, valid=False)], self.ego, self.path, 8., 0.)
        self.assertEqual(result["kind"], "stop")
        self.assertIn("without measured velocity", result["reason"])

    def test_not_ready_detector_blocks_entry(self):
        planner = self.planner()
        planner.junction["ready"] = False
        result = planner.evaluate([], self.ego, self.path, 8., 0.)
        self.assertEqual(result["kind"], "stop")

    def test_empty_ring_goes_at_once_after_the_hold(self):
        # No ring car approaching: go immediately (no extra clear dwell).
        planner = self.planner()
        first = planner.evaluate([], self.ego, self.path, 8., 0.)
        self.assertTrue(first is None or first["kind"] != "stop", first)
        self.assertTrue(planner.junction_committed)

    def test_car_arriving_just_after_our_slow_launch_is_rejected(self):
        # Crosses our path 10 m ahead at 3 m/s, ~4.3 s from now: our slow launch
        # (ring_entry.LAUNCH_SLOW) clears its lane ~3.6 s from now, less than
        # ring_entry.NEXT_MARGIN_S before its (conservative) arrival.
        planner = self.planner()
        result = self.run_until(planner, [car(10., 13., 0., -3., 11)], 0., 1.)
        self.assertEqual(result["kind"], "stop", result)
        self.assertIn("wait at the line", result["reason"])
        self.assertFalse(planner.junction_committed)

    def test_committed_entry_does_not_stop_for_car_reaching_our_side(self):
        planner = self.planner()
        planner.junction_committed, planner.state = True, "ENTER"
        moving = dict(self.ego, speed_kmh=12.)
        side = car(1., 5., 0., -3., 12)        # comes into our left side
        result = planner.evaluate([side], moving, self.path, 8., 0.)
        self.assertTrue(result is None or result["kind"] != "stop", result)
        self.assertGreater(result["v_mps"], 3.)

    def test_committed_entry_only_brakes_for_car_crossing_ahead(self):
        # User rule 2026-10-10: once committed a car really ahead only slows us
        # (here to a standstill behind it, as following does) - never a stop rule.
        planner = self.planner()
        planner.junction_committed, planner.state = True, "ENTER"
        moving = dict(self.ego, speed_kmh=12.)
        ahead = car(8., 4., 0., -4., 13)       # crosses 8 m ahead within ~1 s
        result = planner.evaluate([ahead], moving, self.path, 8., 0.)
        self.assertIsNotNone(result)
        self.assertEqual(result["kind"], "follow", result)
        self.assertLess(result["v_mps"], 8., result)
        self.assertTrue(planner.junction_committed)
        # Right in front of the bumper: braked to a standstill behind it.
        close = car(5.5, 1., 0., -4., 13)
        result = planner.evaluate([close], moving, self.path, 8., .05)
        self.assertEqual(result["kind"], "follow", result)
        self.assertEqual(result["v_mps"], 0., result)
        # Once it has crossed, we drive on at once (no hold, no new entry decision).
        gone = car(8., -6., 0., -4., 13)
        result = planner.evaluate([gone], dict(moving, speed_kmh=2.), self.path, 8., .1)
        self.assertNotEqual(result["kind"], "stop", result)
        self.assertGreater(result["v_mps"], 3.)

    def test_stop_while_committed_does_not_reopen_the_entry_decision(self):
        planner = self.planner()
        planner.junction_committed, planner.state = True, "ENTER"
        standing = dict(self.ego, speed_kmh=0.)
        arriving = car(10., 13., 0., -3., 11)  # would block a fresh entry decision
        result = planner.evaluate([arriving], standing, self.path, 8., 0.)
        self.assertNotEqual(result["kind"], "stop", result)
        self.assertTrue(planner.junction_committed)

    def test_held_at_line_with_traffic_drives_up_to_the_line(self):
        planner = self.planner(held=True)
        planner.junction["line_gap_m"] = 6.
        moving = dict(self.ego, speed_kmh=10.)
        result = planner.evaluate([car(10., 9., 0., -3., 14)], moving, self.path, 8., 0.)
        self.assertEqual(result["kind"], "follow", result)
        self.assertFalse(planner.junction_committed)

    def test_no_junction_keeps_previous_behaviour(self):
        planner = GapPlanner()
        result = planner.evaluate([], self.ego, self.path, 8., 0.)
        self.assertIsNone(result)
        self.assertFalse(planner.junction_committed)

    def test_following_only_has_no_line_stop_or_commit(self):
        for gap in (80., 5., .5, -.1, -8.):
            planner = self.planner(held=True)
            planner.junction.update(following_only=True, line_gap_m=gap,
                                    entry_m=-gap, ready=True)
            result = planner.evaluate([], self.ego, self.path, 8., 0.)
            self.assertEqual(result['kind'], 'enter', result)
            self.assertGreater(result['v_mps'], 0.)
            self.assertFalse(planner.junction_committed)
            self.assertEqual(planner.status['ring']['action'], 'GO')
            self.assertEqual(planner.status['ring']['checks'], [])

    def test_following_only_still_brakes_for_actual_front_body(self):
        planner = self.planner(held=True)
        planner.junction.update(following_only=True, ready=True)
        result = planner.evaluate([car(7., 0., 0., 0., 21)],
                                  dict(self.ego, speed_kmh=12.), self.path, 8., 0.)
        self.assertEqual(result['kind'], 'follow', result)
        self.assertEqual(result['v_mps'], 0., result)
        self.assertFalse(planner.junction_committed)

    def test_following_only_keeps_sensor_failure_stop(self):
        for ready, fresh in ((False, True), (True, False)):
            planner = self.planner()
            planner.junction.update(following_only=True, ready=ready)
            result = planner.evaluate([], self.ego, self.path, 8., 0., fresh=fresh)
            self.assertEqual(result['kind'], 'stop', result)
            self.assertEqual(result['v_mps'], 0.)

    def test_following_only_traffic_braking_does_not_use_stop_line_position(self):
        caps = []
        for line_gap in (80., .5, -8.):
            planner = self.planner()
            planner.junction.update(following_only=True, line_gap_m=line_gap)
            result = planner.evaluate([car(12., 9., 0., -3., 22)],
                                      dict(self.ego, speed_kmh=20.), self.path, 8., 0.)
            self.assertEqual(result['reason'], 'junction: brake before occupied ring corridor')
            self.assertLess(result['v_mps'], 8.)
            self.assertFalse(planner.junction_committed)
            caps.append(result['v_mps'])
        self.assertEqual(caps, [caps[0]] * 3)

    def test_following_only_keeps_unmapped_unknown_vehicle_stop(self):
        planner = self.planner()
        planner.junction.update(following_only=True)
        result = planner.evaluate([car(8., 6., 0., 0., 23, valid=False)],
                                  self.ego, self.path, 8., 0.)
        self.assertEqual(result['kind'], 'stop', result)
        self.assertEqual(result['v_mps'], 0.)
        self.assertFalse(planner.junction_committed)

    def test_occupied_corridor_tiny_crawl_stops_then_fresh_gap_releases_at_same_pose(self):
        planner = self.planner()
        planner.junction.update(following_only=True)
        ego = dict(self.ego, x=5.)
        blocked = planner.evaluate([car(12., 9., 0., -3., 22)], ego, self.path, 8., 0.)
        self.assertEqual(planner.status['state'], 'CROSSING_CAUTION')
        self.assertEqual(planner.status['ring']['action'], 'WAIT')
        self.assertAlmostEqual(planner.status['contact_gap_m'], .755)
        self.assertEqual(blocked['kind'], 'stop')
        self.assertEqual(blocked['v_mps'], 0.)
        at_buffer = planner.evaluate([car(12., 9., 0., -3., 22)],
            dict(ego, x=5.3), self.path, 8., .05)
        self.assertEqual(at_buffer['v_mps'], 0.)
        # The same observed car has passed the corridor and keeps moving away;
        # neither the ego pose nor a surveyed-line parameter changes.
        released = planner.evaluate([car(12., -12., 0., -3., 22)], ego, self.path, 8., .1)
        self.assertEqual(planner.status['ring']['action'], 'GO')
        self.assertGreater(released['v_mps'], .5)
        self.assertFalse(planner.junction_committed)

    def test_occupied_corridor_above_capture_speed_retains_its_braking_envelope(self):
        from stop_line import yield_line_cap
        planner = self.planner()
        planner.junction.update(following_only=True)
        ego = dict(self.ego, x=4.9, speed_kmh=.6*3.6)
        path = np.column_stack((np.arange(0., 40.01, .05), np.zeros(801)))
        result = planner.evaluate([car(12., 9., 0., -3., 22)], ego, path, 8., 0.)
        gap = planner.status['contact_gap_m']
        self.assertEqual(planner.status['ring']['action'], 'WAIT')
        self.assertGreater(result['v_mps'], 0.)
        self.assertLess(result['v_mps'], .5)
        self.assertAlmostEqual(result['v_mps'], yield_line_cap(gap, .6, 0.))

    def test_occupied_corridor_uses_the_physical_gap_with_one_braking_reserve(self):
        from stop_line import (yield_line_cap, YIELD_BUFFER_M, StopLineController,
                               INTERSECTION_ACCEL_MPS2)
        planner = self.planner()
        planner.junction.update(following_only=True)
        ego = dict(self.ego, x=4., age_s=.05)
        result = planner.evaluate([car(12., 9., 0., -3., 22)], ego, self.path, 8., 0.)
        gap = planner.status['contact_gap_m']
        self.assertEqual(planner.status['ring']['action'], 'WAIT')
        self.assertAlmostEqual(gap, 1.755)
        self.assertGreater(result['v_mps'], .5)
        self.assertAlmostEqual(result['v_mps'], yield_line_cap(gap, 0., .05))
        cap = result['v_mps']
        travel = .5 * INTERSECTION_ACCEL_MPS2 * .05**2
        reserved = (travel + YIELD_BUFFER_M + cap * StopLineController.REACTION_S
                    + cap**2 / (2. * StopLineController.BRAKE_MPS2))
        self.assertLessEqual(reserved, gap + 1e-9)


if __name__ == "__main__":
    unittest.main()
