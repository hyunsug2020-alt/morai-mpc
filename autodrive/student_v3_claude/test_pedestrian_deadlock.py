"""Standing-person deadlock search and decision-log replays; no simulator.

Log replays reconstruct perception positions, not missing LiDAR or selected
paths. Geometry tests separately supply complete obstacle/corridor inputs.
"""
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

import model_avoidance as ma
import ped_guard as pg
import gui
from gui import Runtime, RosPathPublisher, MAX_CONTROL_CURVATURE, static_speed_cap_mps
from lane_corridor import LaneCorridor
from path_safety import PathSafety
from test_ped_guard import EGO, person
from test_pedestrian_release import Replay, ROUTE, local

DATA = json.loads((Path(__file__).parent/'test_data/pedestrian_deadlock_20261011.json').read_text())
C_RUN, HARD08_RUN, *CROSSINGS = DATA['runs']
PATH = np.column_stack((np.arange(0., 60.01, .25), np.zeros(241)))


class Standing:
    rtf = 1.

    def state(self, uid):
        return dict(state='standing', age=20.)


def corridor(left=5.25, right=-5.25, route=PATH, zones=None):
    return LaneCorridor(route, np.full(len(route), left), np.full(len(route), right), zones=zones)


def host():
    runtime = Runtime.__new__(Runtime)
    runtime._ped_tracker = Standing()
    runtime.zone_kmh = 30.
    runtime.path_publisher = SimpleNamespace(zone_mps=30./3.6, _safety_provider=object())
    runtime.object_receiver = SimpleNamespace(age_s=0.)
    runtime.clearance = 'objects'
    runtime.lanes = SimpleNamespace(n_links=0)
    runtime.model_road_offsets = lambda *args: {}
    runtime._ped_waiting = {16: 0.}
    return runtime


def earned_wait(runtime, obj, rtf=1.):
    runtime._standing_bypass_wait = pg.StandingBypassWait()
    for t in np.arange(0., 7.3/rtf, .1):
        runtime._standing_bypass_wait.update([obj], dict(EGO, speed_kmh=0.), float(t), rtf,
                                              runtime._ped_tracker, {obj[11]})
    return float(t)


class StandingWaitTests(unittest.TestCase):
    def test_bounded_centroid_jitter_pauses_but_real_translation_resets_wait(self):
        wait = pg.StandingBypassWait()
        first_ready = None
        for i in range(240):
            # The C log's alternate body surfaces; only close, confirmed
            # standing observations may count or authorize a search.
            obj = person(7.2+(0.4 if i%4==0 else 0.), .4, 0., 0.)
            ready = wait.update([obj], EGO, i*.1, .67, Standing(), {16})
            if i%4==0:
                self.assertFalse(ready)
            elif ready and first_ready is None:
                first_ready = i*.1
        self.assertIsNotNone(first_ready)
        moved = person(8., .4, 0., 0.)
        self.assertFalse(wait.update([moved], EGO, 24., .67, Standing(), {16}))
        self.assertEqual(wait.tracks[16]['elapsed'], 0.)

    def test_wait_uses_simulator_seconds_and_only_stopped_time(self):
        for rtf in (.6, .85, 1.):
            wait = pg.StandingBypassWait()
            first = None
            for t in np.arange(0., 13., .1):
                ready = wait.update([person(7.2, .4, 0., 0.)], dict(EGO, speed_kmh=0.),
                                    float(t), rtf, Standing(), {16})
                if ready and first is None:
                    first = t
            self.assertGreaterEqual(first*rtf, 7.-1e-6)
            self.assertLess(first*rtf, 7.2)
        wait = pg.StandingBypassWait()
        for t in np.arange(0., 15., .1):
            self.assertFalse(wait.update([person(7.2, .4, 0., 0.)], dict(EGO, speed_kmh=3.),
                                         float(t), 1., Standing(), {16}))

    def test_another_person_wait_cannot_authorize_a_new_person(self):
        runtime = host()
        obj = person(7.2, .4, 0., 0.)
        t = earned_wait(runtime, obj)
        other = person(10., -1., 0., 0., uid=17)
        ready = runtime._standing_bypass_wait.update([obj, other], dict(EGO, speed_kmh=0.),
                                                     t+.1, 1., Standing(), {16, 17})
        self.assertEqual(ready, {16})

    def test_motion_and_first_restart_displacement_revoke_permission(self):
        for changed in (person(7.2, .4, 0., -.6), person(7.2, .15, 0., 0.)):
            runtime = host()
            t = earned_wait(runtime, person(7.2, .4, 0., 0.))
            self.assertFalse(runtime._standing_bypass_wait.update([changed], dict(EGO, speed_kmh=0.),
                              t+.1, 1., Standing(), {16}))

    def test_missing_observation_clock_jump_and_long_gap_reset_wait(self):
        for gap in ('missing', 'gap', 'rewind'):
            runtime = host()
            obj = person(7.2, .4, 0., 0.)
            t = earned_wait(runtime, obj)
            if gap == 'missing':
                runtime._standing_bypass_wait.update([], EGO, t+.1, 1., Standing(), {16})
            when = t+.2 if gap == 'missing' else t+1. if gap == 'gap' else t-2.
            self.assertFalse(runtime._standing_bypass_wait.update([obj], EGO, when, 1., Standing(), {16}))

    def test_id_churn_at_the_same_spot_keeps_the_uninterrupted_wait(self):
        runtime = host()
        t = earned_wait(runtime, person(7.2, .4, 0., 0.))
        ready = runtime._standing_bypass_wait.update([person(7.2, .42, 0., 0., uid=68)], EGO,
                                                     t+.1, 1., Standing(), {68})
        self.assertEqual(ready, {68})

    def test_unknown_motion_cannot_earn_a_bypass(self):
        tracker = SimpleNamespace(state=lambda uid: dict(state='unknown'))
        wait = pg.StandingBypassWait()
        for t in np.arange(0., 15., .1):
            self.assertFalse(wait.update([person(7.2, .4, 0., 0.)], EGO, float(t), 1., tracker, {16}))


class StandingGeometryTests(unittest.TestCase):
    def test_continuation_after_person_loss_rechecks_actual_body_without_a_new_search(self):
        obj = person(10., 0., 0., 0.)
        old = self.plan(obj)
        kept, _ = ma.plan_standing_bypass(PATH, EGO, [], corridor(), lambda p, h: h, [],
            previous=old, allow_search=False)
        self.assertTrue(kept.get('reused'))
        self.assertIsNone(kept['person_gap_m'])
        self.assertLessEqual(static_speed_cap_mps(30./3.6, kept), 8./3.6)
        point = old['path'][30]
        obstacle = (point[0], point[1], 1., 'box', 'obstacle', 0., 1., 1., 0.)
        blocked, _ = ma.plan_standing_bypass(PATH, EGO, [obstacle], corridor(), lambda p, h: h, [],
            previous=old, allow_search=False)
        self.assertTrue(blocked.get('blocked'))

    def plan(self, obj, boxes=(), legal=None, road=None, raw=None):
        return ma.plan_standing_bypass(PATH, dict(EGO, speed_kmh=0.), [obj]+list(boxes),
            legal or corridor(), raw or (lambda path, horizon: horizon), [obj],
            road=road, budget_s=2.)[0]

    def assert_safe(self, plan, obj):
        self.assertFalse(plan.get('blocked'), plan)
        self.assertGreaterEqual(ma.standing_person_gap(plan['path'], EGO, [obj]), 1.05)
        self.assertLessEqual(plan['curvature'], MAX_CONTROL_CURVATURE)
        self.assertLessEqual(static_speed_cap_mps(30./3.6, plan), 8./3.6)
        # Publication uses exactly the geometry that passed the checks.
        np.testing.assert_array_equal(plan['path'], RosPathPublisher.control_geometry(
            plan['path'], PATH, dict(mode='STUDENT FORCED', canonical_control=True)))
        self.assertEqual(pg.plan([obj], EGO, plan['path'], 0., 8./3.6, Standing())['kind'], 'clear')

    def test_person_and_three_close_boxes_share_one_complete_bypass(self):
        obj = person(10., 0., 0., 0.)
        boxes = [(f, .1, 1.1, 'box', 'obstacle', 0., 1.1, 1.1, 0.) for f in (16., 22., 28.)]
        plan = self.plan(obj, boxes)
        self.assert_safe(plan, obj)
        self.assertEqual(plan['cluster_count'], 4)
        self.assertGreater(plan['hold_until_m'], 30.)
        self.assertGreaterEqual(ma._static.static_clear_distance(plan['path'], EGO, boxes, 60.,
            body_shift_m=ma.REAR_AXLE_SHIFT_M), 59.)

    def test_hard08_near_stander_has_a_slow_checked_path(self):
        obj = person(7.2, .4, 0., 0.)
        self.assert_safe(self.plan(obj), obj)

    def test_same_lane_path_wins_before_any_boundary_excursion(self):
        obj = person(10., 1.5, 0., 0.)
        plan = self.plan(obj, legal=corridor(2.25, -2.25), road=corridor())
        self.assert_safe(plan, obj)
        self.assertEqual(plan['boundary_contact_s'], 0.)
        self.assertEqual(plan['boundary_penalty_s'], 0.)

    def test_necessary_excursion_reports_contact_time_and_penalty(self):
        obj = person(10., 0., 0., 0.)
        plan = self.plan(obj, legal=corridor(1.75, -1.75), road=corridor())
        self.assert_safe(plan, obj)
        self.assertGreater(plan['boundary_contact_s'], 0.)
        self.assertEqual(plan['boundary_penalty_s'], 5.*math.ceil(plan['boundary_contact_s']/3.))

    def test_no_space_no_scan_and_no_go_zone_keep_stop(self):
        obj = person(10., 0., 0., 0.)
        for kwargs in (dict(legal=corridor(1.75, -1.75)), dict(raw=lambda p, h: 0.),
                       dict(legal=corridor(zones=[[[3., -6.], [20., -6.], [20., 6.], [3., 6.]]]))):
            self.assertTrue(self.plan(obj, **kwargs).get('blocked'))

    def test_c_last_close_frame_cannot_trade_person_clearance_for_progress(self):
        obj = person(5.8, -.2, 0., 0.)
        self.assertTrue(self.plan(obj).get('blocked'))
        self.assertEqual(pg.plan([obj], EGO, PATH, 0., 8./3.6, Standing())['kind'], 'stop')


class StandingRoadTests(unittest.TestCase):
    def map(self, other_direction=-1., blocked=False, lateral=3.5):
        other = PATH.copy()
        other[:, 1] = lateral
        if other_direction < 0.:
            other = other[::-1]
        links = [dict(idx='route', road_id='A', link_type='0', width_start=3.5, width_end=3.5),
                 dict(idx='other', road_id='B', link_type='0', width_start=3.5, width_end=3.5)]
        points = np.vstack((PATH, other))
        return SimpleNamespace(n_links=2, links=links, geometry={'route': PATH, 'other': other},
            tree=cKDTree(points), link_of=np.r_[np.zeros(len(PATH), int), np.ones(len(other), int)],
            link_trees={'route': cKDTree(PATH), 'other': cKDTree(other)}, blocked=lambda uid: blocked)

    def test_only_mapped_near_parallel_oncoming_lane_may_be_borrowed(self):
        legal = corridor(1.75, -1.75)
        widened = ma.standing_road_corridor(self.map(), EGO, PATH, legal)
        self.assertAlmostEqual(float(widened.left[40]), 5.25)
        self.assertAlmostEqual(float(widened.right[40]), -1.75)
        for lanes in (self.map(blocked=True), self.map(other_direction=1.), self.map(lateral=8.)):
            unchanged = ma.standing_road_corridor(lanes, EGO, PATH, legal)
            np.testing.assert_array_equal(unchanged.left, legal.left)
        self.assertEqual(widened.body_shift_m, ma.REAR_AXLE_SHIFT_M)


class RuntimeBypassTests(unittest.TestCase):
    def setUp(self):
        model = patch.object(gui, 'model_avoidance', ma, create=True)
        model.start()
        self.addCleanup(model.stop)

    def plan(self, runtime, objects, now):
        lanes = corridor()
        return runtime.model_static_plan(dict(EGO, speed_kmh=0.), PATH, objects,
            np.empty((0, 2)), lanes, lambda *args: True, lanes.clear_distance,
            None, {}, now, True, True, 1.2)[0]

    def test_released_person_cannot_start_an_unrelated_bypass_from_old_wait(self):
        runtime = host()
        obj = person(7.2, .4, 0., 0.)
        now = earned_wait(runtime, obj)
        runtime._ped_waiting = {}
        self.assertIsNone(self.plan(runtime, [obj], now))

    def test_guard_wait_turns_into_checked_bypass_and_clears_only_its_stop_latch(self):
        runtime = host()
        obj = person(7.2, .4, 0., 0.)
        now = earned_wait(runtime, obj)
        plan = self.plan(runtime, [obj], now)
        self.assertFalse(plan.get('blocked'), plan)
        runtime.last_yield = dict(kind='stop', name='pedestrian_guard:p')
        runtime._yield_stop_until = now+1.
        decision = runtime.pedestrian_guard([obj], EGO, plan['path'], now,
            checked_standing=plan['standing_bypass_ids'], points=np.empty((0, 2)))
        self.assertIn(16, decision['ids'])
        self.assertEqual(decision['kind'], 'clear')
        self.assertIsNone(runtime.apply_pedestrian_guard(decision, runtime.last_yield, EGO,
            np.empty((0, 2)), [obj], now, plan['path']))
        self.assertEqual(runtime._yield_stop_until, 0.)
        independent = dict(kind='stop', name='obstacle', reason='box on path')
        self.assertEqual(runtime.apply_pedestrian_guard(decision, independent, EGO,
            np.empty((0, 2)), [obj], now, plan['path']), independent)

    def test_walking_into_path_keeps_the_existing_stop_even_after_another_person_wait(self):
        runtime = host()
        obj = person(7.2, .4, 0., 0.)
        now = earned_wait(runtime, obj)
        moving = person(7., -.8, 0., 1., uid=17)
        runtime._ped_tracker = SimpleNamespace(rtf=1., state=lambda uid:
            dict(state='standing' if uid==16 else 'moving', age=20.))
        plan = self.plan(runtime, [obj, moving], now)
        self.assertTrue(plan.get('blocked'))
        self.assertIn('moving person', runtime._model_avoidance_log['standing_bypass_veto'])
        decision = runtime.pedestrian_guard([moving], EGO, PATH, now)
        self.assertTrue(decision['kind']=='stop' or decision.get('stopping'))

    def test_live_checker_rechecks_restart_and_raw_overlap_on_selected_path(self):
        obj = person(10., 0., 0., 0.)
        plan = ma.plan_standing_bypass(PATH, dict(EGO, speed_kmh=0.), [obj], corridor(),
            lambda p, h: h, [obj], budget_s=2.)[0]
        guard = PathSafety()
        for t in np.arange(0., 2., .1):
            guard.check(plan['path'], EGO, [('semantic', [obj], float(t), 0.)], float(t),
                requested_mps=plan['speed_cap_mps'], raw_clear_m=60.)
        stopped = guard.check(plan['path'], EGO, [], 2., requested_mps=plan['speed_cap_mps'], raw_clear_m=.2)
        self.assertTrue(stopped.emergency)
        self.assertEqual(stopped.cap_mps, 0.)
        walker = person(7., -.8, 0., 1.)
        restarted = pg.plan([walker], EGO, PATH, 0., 8./3.6,
                            SimpleNamespace(state=lambda uid: dict(state='moving', age=20.)))
        self.assertTrue(restarted['kind']=='stop' or restarted.get('stopping'))

    def test_lost_person_keeps_one_second_lidar_clear_protection(self):
        runtime = host()
        obj = person(6.3, 0., 0., 0.)
        runtime.pedestrian_guard([obj], EGO, PATH, 10., points=np.empty((0, 2)))
        self.assertEqual(runtime.pedestrian_guard([], EGO, PATH, 10.1,
                         points=np.array([[6.3, 0.]]))['kind'], 'stop')
        for now in (10.2, 10.8, 11.19):
            self.assertEqual(runtime.pedestrian_guard([], EGO, PATH, now,
                             points=np.empty((0, 2)))['kind'], 'stop')
        self.assertEqual(runtime.pedestrian_guard([], EGO, PATH, 11.21,
                         points=np.empty((0, 2)))['kind'], 'clear')


class RecordedDeadlockTests(unittest.TestCase):
    def test_frozen_logs_earn_per_person_search_after_sim_wait(self):
        for name in (C_RUN, HARD08_RUN):
            runtime = host()
            runtime._ped_tracker = pg.PedTracker()
            wait = pg.StandingBypassWait()
            first_ready = None
            for frame in DATA['runs'][name]['frames']:
                t, x, y, h, kmh, rtf, cap, mode, obs = frame
                ego = dict(x=x, y=y, heading_deg=h, speed_kmh=kmh)
                rows = [(ox, oy, max(length, width), 'fusion_pedestrian', kind, speed,
                         length, width, heading, 0., 0., uid)
                        for uid, kind, ox, oy, length, width, heading, speed in obs if kind=='pedestrian']
                runtime._ped_tracker.rtf = rtf
                rows = runtime._ped_tracker.update(rows, t, t)
                ready = wait.update(rows, ego, t, rtf, runtime._ped_tracker,
                                    set(getattr(runtime, '_ped_waiting', {})))
                straight = PATH
                runtime.pedestrian_guard(rows, ego, straight, t)
                if ready and first_ready is None:
                    first_ready = t
            self.assertIsNotNone(first_ready, name)
            self.assertLess(first_ready-DATA['runs'][name]['frames'][0][0], 22., name)

    def test_three_crosswalk_runs_keep_checked_decisions_and_under_four_second_stops(self):
        for name in CROSSINGS:
            runner, start, holds = Replay(), None, []
            for frame, expected in zip(DATA['runs'][name]['frames'], DATA['runs'][name]['expected']):
                t, x, y, h, v, rtf, cap, mode, objects = frame
                obs = [[uid, 'p' if kind=='pedestrian' else 'c', ox, oy]
                       for uid, kind, ox, oy, *_ in objects if kind in ('pedestrian', 'npc')]
                guard_caps = []
                guard_decisions = []
                before_brakes = set(getattr(runner.runtime, '_ped_crossing_brakes', {}))

                def observed_guard(*args, **kwargs):
                    decision = pg.plan(*args, **kwargs)
                    guard_caps.append(decision['cap_mps'])
                    guard_decisions.append(decision)
                    return decision

                with patch('gui.ped_guard_plan', side_effect=observed_guard):
                    result = runner.step([t, x, y, h, v, rtf, cap, obs])
                d = result['decision']
                actual = [d['kind'], None if d['cap_mps'] is None else round(d['cap_mps'], 6),
                          sorted(result['released']), result['go']]
                if (actual[0] == 'stop' and expected[0] == 'yield' and v > .5
                        and before_brakes & set(d.get('blocking_ids', ()))):
                    # The old fixture released a crossing stop while still
                    # braking at 6–14 km/h. Retain the same walker's stop until
                    # the temporal guard proves a pass, including this decel.
                    self.assertEqual(guard_decisions[0]['kind'], 'yield')
                    self.assertTrue(guard_decisions[0]['stopping'])
                    self.assertEqual(actual, ['stop', 0., expected[2], False])
                elif d.get('approach_cap_mps') is not None:
                    # The fixture includes the old live-provider override.
                    # Keep every recorded kind/release/go decision, but the
                    # published cap must now retain the guard's approach bound.
                    self.assertEqual([actual[0], actual[2], actual[3]],
                                     [expected[0], expected[2], expected[3]], (name, t))
                    self.assertEqual(actual[1], round(guard_caps[0], 6), (name, t))
                    old_cap = runner.runtime.path_publisher.zone_mps if expected[1] is None else expected[1]
                    self.assertLessEqual(actual[1], old_cap + 1e-6, (name, t))
                else:
                    self.assertEqual(actual, expected, (name, t))
                stopped = d['kind']=='stop' and v<.5
                if stopped and start is None:
                    start = t
                if not stopped and start is not None:
                    holds.append(t-start)
                    start = None
            self.assertLess(max(holds, default=0.), 4., name)


if __name__ == '__main__':
    unittest.main()
