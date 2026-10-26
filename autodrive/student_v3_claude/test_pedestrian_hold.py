"""Hold ~2.5 m before a crossing or frozen person, never creep closer (user 2026-10-11).

follow-20261011T044808Z 13:50:25-43 (global_hard_08): a walker crossed our path
while we rejoined after a box bypass. The temporal gap released them at 13:50:28.5
with our bumper 3.9 m from their walk while they were still mid-path ("arrive after
crossing"); we rolled up to 10 km/h, they froze 1.4 m ahead of our left front, and
standing people were nobody's job: ped_guard skipped them, the older 4 s wait passed
them on to the bypass planner, the route-relative checks (ROUTE BASE path 1.6 m right
of the car) put them 2.5 m beside the path, and the 2 km/h caution crawl crept on
until the raw contact guard held the bumper 0.2-0.35 m from them for 8 min (MORAI
walkers wait while a car blocks them).

Open-loop replays of the logged decision frames (people rebuilt from the logged near
list, as test_pedestrian_release) through Runtime.pedestrian_guard, plus synthetic
checks of the stand-off, the pose-anchored path and the lost-person hold.
"""
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import ped_guard as pg
from gui import Runtime, PED_LOST_BLIND_S, PED_LOST_CLEAR_S
from motion_filter import measured_velocity
from test_ped_guard import EGO, PATH, Known, person
from test_pedestrian_release import ROUTE, ZONE_MPS, local, path_lateral

HERE = Path(__file__).parent
DATA = json.loads((HERE/'test_data/pedestrian_hold_20261011.json').read_text())
WALKER = 20                        # the person who crossed and froze in front of us
WALKER_SPOT = (-62.80, -14.60)     # where they stood (world), 13:50:31-43
NO_SCAN = object()


class Standing(Known):
    def state(self, uid):
        return {"state": "standing", "age": 5.}


def route_path(ego):
    """ROUTE BASE control path: the route itself, wherever the car is."""
    route = local(ROUTE, ego)
    k = int(np.argmin(np.hypot(route[:, 0], route[:, 1])))
    return route[max(0, k-10):k+160].copy()


def drive_path(ego):
    """The rejoin / bypass paths of these frames are not logged; the car drove
    straight on its heading through them (x -61.7..-61.9 over 13:50:28-40)."""
    return PATH


class HoldReplay:
    def __init__(self):
        self.runtime = Runtime.__new__(Runtime)
        self.runtime.path_publisher = SimpleNamespace(zone_mps=ZONE_MPS, _safety_provider=object())
        self.runtime.zone_kmh = 30.
        self.runtime._ped_tracker = pg.PedTracker()
        self.history = {}

    def step(self, frame, drop=(), points=NO_SCAN):
        t, x, y, heading, kmh, rtf, logged_cap, mode, objects = frame
        ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=kmh)
        rows = []
        for uid, kind, ox, oy in objects:
            if uid in drop:
                continue
            track = self.history.setdefault(uid, [])
            track.append((t, ox, oy))
            track[:] = [s for s in track if t-s[0] <= 1.]
            vx, vy = measured_velocity(track, 4.) or (0., 0.)
            rows.append((ox, oy, .6, 'fusion_pedestrian', 'pedestrian', math.hypot(vx, vy), .6, .6,
                         math.degrees(math.atan2(vy, vx)), vx, vy, uid))
        path = route_path(ego) if mode == 'ROUTE BASE' else drive_path(ego)
        tracker = self.runtime._ped_tracker
        tracker.rtf = rtf or 1.
        refined = tracker.update(rows, t, t)
        extra = {} if points is NO_SCAN else dict(points=points(ego) if callable(points) else points)
        decision = self.runtime.pedestrian_guard(refined, ego, path, t, anticipate_crossing=True, **extra)
        people = {}
        for o in refined:
            f, l = local([o[:2]], ego)[0]
            people[int(o[11])] = dict(f=float(f), l=float(l), lat=path_lateral(np.array([f, l]), path),
                                      speed=float(o[5]))
        released = {uid for uid, until in self.runtime._ped_released.items() if until > t}
        go = decision['kind'] != 'stop' and not decision.get('stopping') and (
            decision['cap_mps'] is None or decision['cap_mps'] > .3)
        return dict(t=t, mode=mode, kmh=kmh, logged_cap=logged_cap, decision=decision, people=people,
                    released=released, go=go)


def replay(name, **kwargs):
    run = HoldReplay()
    return [run.step(frame, **kwargs) for frame in DATA['windows'][name]['frames']], run


class CrossingApproachReplay(unittest.TestCase):
    """13:50:24.8-31.3: approach, logged release mid-path, roll-up, freeze."""

    def test_walker_is_not_released_while_still_in_our_path(self):
        rows, _ = replay('crossing_approach_20261011T044808Z')
        in_path = [r for r in rows if WALKER in r['people'] and
                   abs(r['people'][WALKER]['lat']) < pg.CORRIDOR_HALF_M-.3 and r['people'][WALKER]['f'] > pg.EGO_FRONT_M]
        self.assertGreater(len(in_path), 25)
        # Logged: released at 13:50:28.47 ("arrive after crossing", 3.9 m to their
        # walk) and "clear" up to 10 km/h until they froze 1.4 m ahead.
        self.assertTrue(all(not r['go'] for r in in_path), [round(r['t']-rows[0]['t'], 2) for r in in_path if r['go']])
        self.assertTrue(all(WALKER not in r['released'] for r in rows))
        # Every approach targets a stop point STOP_CLEAR_M before their walk.
        for r in in_path:
            d = r['decision']
            if d['kind'] == 'yield':
                self.assertLessEqual(d['along']+pg.EGO_FRONT_M+pg.STOP_CLEAR_M, r['people'][WALKER]['f']+.4)

    def test_logged_release_frame_now_holds(self):
        rows, _ = replay('crossing_approach_20261011T044808Z')
        frame = next(r for r in rows if r['t'] >= 1791694228.47)
        decision = frame['decision']
        self.assertTrue(decision['kind'] == 'stop' or decision.get('stopping'), decision)
        self.assertIn(WALKER, decision['blocking_ids'])


class FrozenWalkerReplay(unittest.TestCase):
    """13:50:31.3-43: the walker stood 1.4-0.5 m ahead of our left front (l = 0.8-1.0)."""

    def test_frozen_walker_in_front_holds_us_at_zero(self):
        rows, _ = replay('frozen_walker_20261011T044808Z')
        frozen = [r for r in rows if WALKER in r['people']]
        self.assertGreater(len(frozen), 120)
        # Logged: "pedestrian moving nearby: wait" for 4 s, then 1.5-2 km/h creep
        # (rejoin path, then ROUTE BASE) into the raw contact guard.
        stops = [r for r in frozen if r['decision']['kind'] == 'stop' and r['decision']['cap_mps'] == 0.]
        self.assertEqual(len(stops), len(frozen), [round(r['t']-rows[0]['t'], 2) for r in frozen if r not in stops][:5])
        self.assertTrue(all(WALKER in r['decision']['blocking_ids'] and WALKER not in r['released'] for r in frozen))
        # Including the ROUTE BASE frames whose path (the route) ran 1.6 m right of the car.
        route_base = [r for r in frozen if r['mode'] == 'ROUTE BASE']
        self.assertGreater(len(route_base), 20)
        self.assertTrue(all(abs(r['people'][WALKER]['lat']) > pg.CORRIDOR_HALF_M for r in route_base))
        self.assertTrue(any(r['logged_cap'] and r['logged_cap'] > 1. for r in route_base))

    def lost_at(self, points):
        """Fusion loses the frozen walker 2 s into the window; LiDAR per ``points``."""
        frames = DATA['windows']['frozen_walker_20261011T044808Z']['frames']
        lost_from = frames[0][0]+2.
        run = HoldReplay()
        rows = []
        for frame in frames:
            drop = (WALKER,) if frame[0] >= lost_from else ()
            rows.append(run.step(frame, drop=drop, points=points if drop else
                                 (points if points is NO_SCAN else (lambda ego: np.empty((0, 2))))))
        return [r for r in rows if r['t'] >= lost_from], lost_from

    def test_lost_walker_with_lidar_returns_keeps_the_hold(self):
        def walker_returns(ego):
            centre = local([WALKER_SPOT], ego)[0]
            ring = np.array([[math.cos(a)*.25, math.sin(a)*.25] for a in np.linspace(0., 2*math.pi, 8)])
            return centre+ring
        after, _ = self.lost_at(walker_returns)
        self.assertGreater(after[-1]['t']-after[0]['t'], 8.)
        self.assertTrue(all(r['decision']['kind'] == 'stop' and WALKER in r['decision']['blocking_ids']
                            for r in after))
        self.assertIn('person lost', after[-1]['decision']['reason'])

    def test_lost_walker_with_clear_lidar_releases_after_confirmation(self):
        after, lost_from = self.lost_at(lambda ego: np.empty((0, 2)))
        held = [r for r in after if WALKER in r['decision']['blocking_ids']]
        self.assertTrue(held)
        free = next(r['t'] for r in after if WALKER not in r['decision']['blocking_ids'])
        self.assertGreaterEqual(free-lost_from, PED_LOST_CLEAR_S-.05)
        self.assertLess(free-lost_from, PED_LOST_CLEAR_S+.4)
        self.assertTrue(all(WALKER not in r['decision']['blocking_ids'] for r in after if r['t'] >= free))

    def test_lost_walker_without_scan_holds_only_briefly(self):
        after, lost_from = self.lost_at(None)
        free = next(r['t'] for r in after if WALKER not in r['decision']['blocking_ids'])
        self.assertLess(free-lost_from, PED_LOST_BLIND_S+.4)


class StandOffTests(unittest.TestCase):
    def plan(self, o, kmh, path=PATH, tracker=None, **kwargs):
        return pg.plan([o], dict(EGO, speed_kmh=kmh), path, kmh/3.6, ZONE_MPS, tracker or Known(), {}, 0., **kwargs)

    def test_standing_person_in_path_gets_approach_then_hold(self):
        o = person(25., .3, 0., 0.)
        far = self.plan(o, 30., tracker=Standing())
        self.assertEqual((far['kind'], far['stopping']), ('yield', True))
        self.assertAlmostEqual(far['along'], 25.-pg.EGO_FRONT_M-pg.STOP_CLEAR_M, places=2)
        self.assertIn(16, far['blocking_ids'])
        self.assertNotIn(16, far['ids'])      # the older rules keep seeing them too
        at_point = self.plan(person(pg.EGO_FRONT_M+pg.STOP_CLEAR_M, .3, 0., 0.), 0., tracker=Standing())
        self.assertEqual((at_point['kind'], at_point['cap_mps']), ('stop', 0.))
        short = person(pg.EGO_FRONT_M+pg.STOP_CLEAR_M+.6, .3, 0., 0.)
        self.assertEqual(self.plan(short, 0., tracker=Standing(), holding={16})['kind'], 'stop')
        closer = self.plan(person(pg.EGO_FRONT_M+1., -.8, 0., 0.), 0., tracker=Standing())
        self.assertEqual((closer['kind'], closer['cap_mps']), ('stop', 0.))

    def test_standing_people_beside_the_corridor_or_body_are_not_waited_for(self):
        for o in (person(8., 1.75, 0., 0.), person(8., -2.2, 0., 0.), person(3., 1.4, 0., 0.)):
            with self.subTest(o=o[:2]):
                r = self.plan(o, 0., tracker=Standing())
                self.assertEqual(r['kind'], 'clear')
                self.assertEqual(r['blocking_ids'], set())

    def test_route_base_path_beside_the_car_is_judged_from_the_pose(self):
        # 13:50:40.6: the route (published path) 1.6 m right of the car, walker
        # standing 1.45 m ahead of our left front: 2.5 m beside "the path".
        route = np.column_stack((np.arange(-5., 80., .5), np.full(170, -1.6)))
        anchored = pg.anchor_path(route)
        frame = pg.PathFrame(anchored)
        self.assertAlmostEqual(float(frame.project([[0., 0.]])[1][0]), 0., places=2)
        self.assertAlmostEqual(float(np.interp(pg.ANCHOR_MERGE_M+1., anchored[:, 0], anchored[:, 1])), -1.6, places=2)
        np.testing.assert_allclose(pg.anchor_path(PATH), PATH)   # on the path: unchanged
        held = self.plan(person(5.3, .8, 0., 0.), 0., path=route, tracker=Standing())
        self.assertEqual((held['kind'], held['cap_mps']), ('stop', 0.))
        # Someone 2.5 m left of the car is beside the body's way back to the route.
        self.assertEqual(self.plan(person(5.3, 2.5, 0., 0.), 0., path=route, tracker=Standing())['kind'], 'clear')

    def test_checked_bypass_around_a_stander_is_not_held(self):
        x = np.arange(-5., 80., .5)
        u = np.clip(x/7., 0., 1.)
        bypass = np.column_stack((x, -2.*u*u*(3.-2.*u)))   # swings 2 m right by x = 7
        o = person(10., 0., 0., 0.)
        self.assertEqual(self.plan(o, 5., path=bypass, tracker=Standing())['blocking_ids'], set())
        self.assertIn(16, self.plan(o, 5., tracker=Standing())['blocking_ids'])

    def test_arrive_after_crossing_keeps_the_stand_off(self):
        # Stopped 1.4 m short of the stop point, walker mid-path (13:50:28.47).
        o = person(pg.EGO_FRONT_M+pg.STOP_CLEAR_M+1.4, -.6, 0., 1.36)
        r = self.plan(o, 1.8)
        self.assertTrue(r['kind'] == 'stop' or r.get('stopping'), r)
        self.assertIn(16, r['blocking_ids'])
        # Nearly out of the corridor and walking on: go (the clean departure).
        out = self.plan(person(pg.EGO_FRONT_M+pg.STOP_CLEAR_M+.3, 1.35, 0., 1.36), 0.)
        self.assertNotEqual(out['kind'], 'stop')
        self.assertFalse(out.get('stopping', False))


if __name__ == '__main__':
    unittest.main()
