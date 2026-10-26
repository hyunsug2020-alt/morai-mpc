"""Stop for a crossing person, then release once (user 2026-10-11: "멈칫멈칫").

Replays of logged decision frames (follow-20261010T162527Z, 113545Z) through the
decision thread's pedestrian guard with its release commit (Runtime.pedestrian_guard)
and the live checker's PathSafety, open loop on the logged ego states. People and
the fusion_npc car box grown from a person's returns are rebuilt from the logged
near list. No simulator; the ego motion is the recorded one.
"""
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import ped_guard as pg
from gui import Runtime
from motion_filter import measured_velocity
from path_safety import MotionMemory, PathSafety
from test_ped_guard import EGO, PATH, Known, person

HERE = Path(__file__).parent
DATA = json.loads((HERE/'test_data/pedestrian_release_20261010.json').read_text())
ROUTE = np.array([[w['x'], w['y']] for w in
                  json.loads((HERE/'global_path.json').read_text())['waypoints']])
ZONE_MPS = 30./3.6


def local(points, ego):
    h = math.radians(ego['heading_deg'])
    d = np.asarray(points, float).reshape(-1, 2)-[ego['x'], ego['y']]
    return np.column_stack((d[:, 0]*math.cos(h)+d[:, 1]*math.sin(h),
                            -d[:, 0]*math.sin(h)+d[:, 1]*math.cos(h)))


def control_path(ego):
    """Route ahead in the ego frame; an offset ego rejoins it over 15 m (bypass exit)."""
    route = local(ROUTE, ego)
    k = int(np.argmin(np.hypot(route[:, 0], route[:, 1])))
    path = route[max(0, k-10):k+160].copy()
    offset = -float(np.interp(0., path[:, 0], path[:, 1]))
    if abs(offset) > .5:
        u = np.clip(path[:, 0]/15., 0., 1.)
        path[:, 1] += np.where(path[:, 0] > 0., offset*(1.-u*u*(3.-2.*u)), offset)
    return path


def path_lateral(point, path):
    j = int(np.clip(np.argmin(np.hypot(*(path-point).T)), 1, len(path)-2))
    tangent = path[j+1]-path[j-1]
    tangent /= np.linalg.norm(tangent)
    return float((point-path[j]) @ np.array([-tangent[1], tangent[0]]))


class Replay:
    def __init__(self):
        self.runtime = Runtime.__new__(Runtime)
        # As on the road: the live checker is attached to the publisher.
        self.runtime.path_publisher = SimpleNamespace(zone_mps=ZONE_MPS, _safety_provider=object())
        self.runtime.zone_kmh = 30.
        self.runtime._ped_tracker = pg.PedTracker()
        self.live = PathSafety()
        self.decision_ghosts, self.live_ghosts = pg.PersonGhostFilter(), pg.PersonGhostFilter()
        self.history = {}
        self.ghosts_dropped = 0

    def step(self, frame):
        t, x, y, heading, kmh, rtf, logged_cap, objects = frame
        ego = dict(x=x, y=y, heading_deg=heading, speed_kmh=kmh)
        rows = []
        for uid, kind, ox, oy in objects:
            track = self.history.setdefault(uid, [])
            track.append((t, ox, oy))
            track[:] = [s for s in track if t-s[0] <= 1.]
            # Receiver/fusion velocity: the robust history fit (wall units).
            vx, vy = measured_velocity(track, 4. if kind == 'p' else 25.) or (0., 0.)
            speed = math.hypot(vx, vy)
            if kind == 'p':
                rows.append((ox, oy, .6, 'fusion_pedestrian', 'pedestrian', speed, .6, .6,
                             math.degrees(math.atan2(vy, vx)), vx, vy, uid))
            elif speed >= .5:   # slower "vehicle" boxes are listed as parked obstacles
                rows.append((ox, oy, 4.6, 'fusion_npc', 'npc', speed, 4.6, 1.9,
                             math.degrees(math.atan2(vy, vx)), vx, vy, uid))
        path = control_path(ego)
        tracker = self.runtime._ped_tracker
        tracker.rtf = rtf or 1.
        refined = tracker.update(rows, t, t)
        kept = self.decision_ghosts(refined, ego, t, tracker.rtf)
        self.ghosts_dropped += len(refined)-len(kept)
        decision = self.runtime.pedestrian_guard(kept, ego, path, t, anticipate_crossing=True)
        semantic = self.live_ghosts(rows, ego, t)
        released = {uid for uid, until in self.runtime._ped_released.items() if until > t}
        live = self.live.check(path, ego, [('semantic', semantic, t-.1, .1)], t,
                               requested_mps=ZONE_MPS, dynamic_only=True, motion_time_scale=rtf or 1.,
                               released_people=released)
        cap = min(ZONE_MPS, live.cap_mps, decision['cap_mps'] if decision['cap_mps'] is not None else ZONE_MPS)
        go = decision['kind'] != 'stop' and not live.emergency and cap > .3
        people = []
        for o in refined:
            if o[4] == 'pedestrian':
                f, l = local([o[:2]], ego)[0]
                people.append(dict(f=float(f), lat=path_lateral(np.array([f, l]), path), speed=float(o[5])))
        return dict(t=t, go=go, cap=cap, decision=decision, live=live, people=people,
                    released=released, logged_cap=logged_cap, kmh=kmh)


def replay(name):
    run = Replay()
    return [run.step(frame) for frame in DATA['windows'][name]['frames']], run


def in_path_ahead(row):
    """A person whose body is in our swept corridor ahead of the true front bumper."""
    return any(abs(p['lat']) < pg.CORRIDOR_HALF_M and pg.EGO_FRONT_M-.3 < p['f'] < 12. for p in row['people'])


def leaving_path(row):
    """Everyone still in our corridor ahead is within 0.5 m of its edge (walking out)."""
    return all(abs(p['lat']) >= pg.CORRIDOR_HALF_M-.5 for p in row['people']
               if pg.EGO_FRONT_M-.3 < p['f'] < 12.)


def go_to_stop(rows):
    return [b['t'] for a, b in zip(rows, rows[1:]) if a['go'] and not b['go']]


def logged_cycles(rows, after):
    caps = [r['logged_cap'] or 0. for r in rows if r['t'] >= after]
    return sum(1 for a, b in zip(caps, caps[1:]) if a > .5 and b <= .05)


class RecordedReleaseTests(unittest.TestCase):
    def assert_single_departure(self, name, min_logged_cycles):
        rows, _ = replay(name)
        entered = next(r for r in rows if in_path_ahead(r))
        # The person entering our path ahead stops us in that frame.
        self.assertFalse(entered['go'], name)
        stop = entered['t']
        release = next(r['t'] for r in rows if r['t'] > stop and r['go'])
        after = [r for r in rows if r['t'] >= release]
        # One decisive departure: no frame after the release stops us again.
        self.assertTrue(all(r['go'] for r in after), [r['t'] for r in after if not r['go']][:5])
        # ...and only once the person has (nearly) left our path: no release while
        # they still walk through our corridor ahead (follow-20261011T044808Z).
        self.assertTrue(leaving_path(after[0]), after[0]['people'])
        # While stopped for them we hold: no creep cap toward the walk line.
        held = [r for r in rows if stop <= r['t'] < release]
        self.assertTrue(held)
        self.assertTrue(all(r['decision']['kind'] == 'stop' or r['live'].emergency
                            or r['cap'] <= .3 for r in held))
        # The recorded drive went and stopped again many times after this point.
        self.assertGreaterEqual(logged_cycles(rows, stop), min_logged_cycles)
        last_logged_stop = max(r['t'] for r in rows if (r['logged_cap'] or 0.) <= .05)
        return rows, stop, release, last_logged_stop

    def test_site2_crossing_then_kerb_stander_releases_once(self):
        # 162527Z 01:27:41.9-28:01: stopped for the crossing, then 8 go/stop
        # cycles (live "collision"/departure for the person standing 2.5 m right
        # of the path beside the body, the fusion_npc box) until 01:27:59.4.
        rows, stop, release, last_stop = self.assert_single_departure('site2_cross_20261010T162527Z', 6)
        self.assertLess(release-stop, 2.5)
        self.assertGreater(last_stop-release, 14.)
        # Released while the person was still walking out, at walking pace.
        mover = [p for r in rows if r['t'] == release for p in r['people']]
        self.assertTrue(mover and all(p['speed'] > 1. for p in mover))

    def test_site2_long_crawl_releases_once(self):
        # 113545Z 20:38:08-38:34: 13 go/stop cycles over 26 s after one crossing.
        rows, stop, release, last_stop = self.assert_single_departure('site2_cross_20261010T113545Z', 8)
        self.assertGreater(last_stop-release, 20.)

    def test_site1_rejoin_crossing_with_vehicle_box_releases_once(self):
        # 162527Z 01:27:23-34: the person's own returns became a moving fusion_npc
        # car box 2.1 m beyond them; it stopped us after the person was clear.
        rows, run = replay('site1_rejoin_20261010T162527Z')
        self.assertGreaterEqual(run.ghosts_dropped, 3)
        stop = next(r['t'] for r in rows if in_path_ahead(r))
        release = next(r['t'] for r in rows if r['t'] > stop and r['go'] and
                       all(q['go'] for q in rows if q['t'] >= r['t']))
        # Held until the person walked out of our corridor (they crossed it in
        # ~3 s; the old 1.7 s release came with them still mid-path 3 m ahead).
        self.assertLess(release-stop, 3.5)
        self.assertTrue(leaving_path(next(r for r in rows if r['t'] == release)))
        self.assertGreaterEqual(logged_cycles(rows, stop), 4)

    def test_standing_person_beside_the_body_never_stops_us(self):
        # Second lap, 162527Z 01:32:34-49: the person stood 2.5 m right of the
        # path the whole time; the drive crawled 0-4 km/h with 8 stop pulses.
        rows, _ = replay('site2_beside_20261010T162527Z')
        beside = [r for r in rows if r['people'] and
                  all(p['f'] <= pg.EGO_FRONT_M-.4 and abs(p['lat']) > 2. for p in r['people'])]
        self.assertGreater(len(beside), 40)
        self.assertTrue(all(r['go'] and r['cap'] >= ZONE_MPS-.01 for r in beside))
        self.assertGreaterEqual(logged_cycles(rows, rows[0]['t']), 6)
        self.assertLessEqual(len(go_to_stop(rows)), 2)   # approach pulses only (open loop)


class ReleaseCommitTests(unittest.TestCase):
    """Runtime.pedestrian_guard release commit on synthetic crossings."""

    def guard(self, runtime, o, kmh, now):
        return runtime.pedestrian_guard([o], dict(EGO, speed_kmh=kmh), PATH, now)

    def runtime(self):
        runtime = Runtime.__new__(Runtime)
        runtime.path_publisher, runtime.zone_kmh, runtime._ped_tracker = None, 30., Known()
        return runtime

    def test_release_once_then_nominal_reentry_stops_again(self):
        runtime = self.runtime()
        lateral, kinds, released_at = -.5, [], None
        for k in range(30):                       # crossing 3.2 m ahead at 0.8 m/s
            r = self.guard(runtime, person(7., lateral, 0., .8), 0., .1*k)
            kinds.append(r['kind'])
            if k == 0:
                self.assertEqual(r['blocking_ids'], {16})
            elif released_at is None and 16 in runtime._ped_waiting:
                # Already stopping for them: hold, never creep toward the walk.
                self.assertEqual((r['kind'], r['cap_mps']), ('stop', 0.))
            if released_at is None and 16 in runtime._ped_released:
                released_at = k
                self.assertNotIn(16, runtime._ped_waiting)
                # Released while still in our corridor, predicted clear on arrival.
                self.assertLess(abs(lateral), pg.CORRIDOR_HALF_M)
            lateral += .08
        self.assertIsNotNone(released_at)
        self.assertTrue(all(kind in ('clear', 'yield') for kind in kinds[released_at:]), kinds)
        # Turning back into our path ahead is a re-entry: full rules, stop, commit ended.
        back = self.guard(runtime, person(7., lateral, 0., -1.2), 7.2, 3.)
        self.assertEqual(back['kind'], 'stop')
        self.assertIn(16, back['blocking_ids'])
        self.assertNotIn(16, runtime._ped_released)
        self.assertIn(16, runtime._ped_waiting)

    def test_unknown_hold_does_not_release_a_crossing_at_standstill(self):
        class Mixed(Known):
            def state(self, uid):
                return {"state": "unknown", "age": .2} if uid == 1 else super().state(uid)
        crossing, unknown = person(7., -.6, 0., 1.2), person(22., -3., 0., 0., uid=1)
        r = pg.plan([crossing, unknown], dict(EGO, speed_kmh=0.), PATH, 0., ZONE_MPS, Mixed(), {}, 0.)
        self.assertNotIn(r['kind'], ('clear', 'pass', 'hold'))
        self.assertIn(16, r['conflict_ids'])

    def test_released_rolling_ego_slows_instead_of_stopping(self):
        # Bumper 5.65 m from the walk line: room to roll behind the 2.5 m stand-off.
        o = person(9.5, .2, 0., .8)
        stop = pg.plan([o], dict(EGO, speed_kmh=7.), PATH, 7./3.6, ZONE_MPS, Known(), {}, 0.)
        self.assertTrue(stop['kind'] == 'stop' or stop.get('stopping'), stop)
        self.assertIn(16, stop['blocking_ids'])
        rolled = pg.plan([o], dict(EGO, speed_kmh=7.), PATH, 7./3.6, ZONE_MPS, Known(), {}, 0.,
                         released={16: 5.})
        self.assertEqual(rolled['kind'], 'yield')
        self.assertFalse(rolled.get('stopping', False))
        self.assertGreaterEqual(rolled['cap_mps'], pg.ROLL_MIN_MPS)
        self.assertLess(rolled['cap_mps'], pg.MIN_YIELD_MPS)
        t = np.arange(0., 20., .01)
        travel = pg._profile('speed', 7./3.6, t, v_target=rolled['cap_mps'])
        for factor in pg.RELEASED_FACTORS:
            lateral = .2+.8*factor*t
            self.assertGreater(pg._clearance(np.full_like(t, 9.5), lateral, travel).min(), 0.)
            # The front stays STOP_CLEAR_M short of their walk while they are in our corridor.
            inside = np.abs(lateral) <= pg.CORRIDOR_HALF_M
            self.assertLessEqual((travel[inside]+pg.EGO_FRONT_M+pg.STOP_CLEAR_M).max(), 9.5+.02)
        # From a standstill a released walker-out is simply clear; unreleased,
        # the launch waits for the full 0.8x walker.
        self.assertEqual(pg.plan([o], dict(EGO, speed_kmh=0.), PATH, 0., ZONE_MPS, Known(), {}, 0.,
                                 released={16: 5.})['kind'], 'clear')
        launch = pg.plan([o], dict(EGO, speed_kmh=0.), PATH, 0., ZONE_MPS, Known(), {}, 0.)
        self.assertEqual(launch['kind'], 'yield')
        self.assertLess(launch['cap_mps'], ZONE_MPS)
        # Rolling 3.65 m from the walk of a released person still mid-path: stop.
        close = pg.plan([person(7.5, .2, 0., .8)], dict(EGO, speed_kmh=7.), PATH, 7./3.6, ZONE_MPS,
                        Known(), {}, 0., released={16: 5.})
        self.assertEqual(close['kind'], 'stop')

    def test_holding_stop_does_not_creep_to_the_walk_line(self):
        o = person(pg.EGO_FRONT_M+pg.STOP_CLEAR_M+.6, 0., 0., .3)
        creep = pg.plan([o], dict(EGO, speed_kmh=0.), PATH, 0., ZONE_MPS, Known(), {}, 0.)
        self.assertEqual(creep['kind'], 'yield')        # approach contract unchanged
        held = pg.plan([o], dict(EGO, speed_kmh=0.), PATH, 0., ZONE_MPS, Known(), {}, 0., holding={16})
        self.assertEqual(held['kind'], 'stop')
        self.assertEqual(held['cap_mps'], 0.)
        self.assertEqual(held['blocking_ids'], {16})


class BesideTheBodyTests(unittest.TestCase):
    def test_person_beside_body_is_decided_not_stopped_for(self):
        for o in (person(2., -2.5, 0., .4), person(3.3, 2.6, .3, -.3), person(-.5, -3., 0., -1.)):
            with self.subTest(o=o[:2]):
                r = pg.plan([o], dict(EGO, speed_kmh=0.), PATH, 0., ZONE_MPS, Known(), {}, 0.)
                self.assertEqual(r['kind'], 'clear')
                self.assertIn(16, r['ids'])   # owned: the older position rules skip them

    def test_walking_into_our_side_or_close_to_it_keeps_every_check(self):
        for o in (person(2.5, -2.5, 0., 1.2), person(3.2, -1.6, 0., .3), person(4.6, -2.5, 0., .4)):
            with self.subTest(o=o[:2]):
                r = pg.plan([o], dict(EGO, speed_kmh=0.), PATH, 0., ZONE_MPS, Known(), {}, 0.)
                self.assertNotEqual(r['kind'], 'clear')

    def test_live_check_keeps_a_kerb_person_ahead_but_not_beside(self):
        def check(o, kmh=8.):
            return PathSafety().check(PATH, dict(EGO, speed_kmh=kmh), [('semantic', [o], 100., .1)], 10.,
                                      requested_mps=ZONE_MPS, dynamic_only=True)
        ahead = check(person(6., -2.5, 0., 0.))
        self.assertLess(ahead.cap_mps, 12./3.6)
        beside = check(person(2.5, -2.5, 0., 0.))
        self.assertAlmostEqual(beside.cap_mps, ZONE_MPS)
        self.assertFalse(beside.emergency)
        # Released people keep only the forecast checks: stepping back in stops us.
        released = PathSafety().check(PATH, dict(EGO, speed_kmh=8.),
            [('semantic', [person(6., -2.5, 0., 0.)], 100., .1)], 10., requested_mps=ZONE_MPS,
            dynamic_only=True, released_people={16})
        self.assertAlmostEqual(released.cap_mps, ZONE_MPS)
        back = PathSafety().check(PATH, dict(EGO, speed_kmh=15.),
            [('semantic', [person(6., -2., 0., 1.3)], 100., .1)], 10., requested_mps=ZONE_MPS,
            dynamic_only=True, released_people={16})
        self.assertTrue(back.emergency)
        self.assertEqual(back.cap_mps, 0.)
        # Farther ahead the same entry is a deceleration, not a stop.
        farther = PathSafety().check(PATH, dict(EGO, speed_kmh=15.),
            [('semantic', [person(7.5, -2., 0., 1.3)], 100., .1)], 10., requested_mps=ZONE_MPS,
            dynamic_only=True, released_people={16})
        self.assertFalse(farther.emergency)
        self.assertLess(farther.cap_mps, 15./3.6)

    def test_walking_away_departure_is_later_than_standing(self):
        from path_safety import departure_time
        self.assertAlmostEqual(departure_time(1., 0.), .45+(1.-.405)/1.8)
        self.assertGreater(departure_time(1., -1.2), departure_time(1., 0.)+.3)
        self.assertLess(departure_time(1., 1.2), departure_time(1., 0.))


class MotionEvidenceTests(unittest.TestCase):
    def test_kerb_jitter_is_not_a_walk_into_the_path(self):
        rng = np.random.default_rng(3)
        memory = MotionMemory()
        for k in range(12):
            jitter = rng.normal(0., .08, 2)
            vy = .45 if k % 2 else -.4      # fusion's own 0.3 s velocity on a stander
            obj = (6.+jitter[0], -2.5+jitter[1], .6, 'fusion_pedestrian', 'pedestrian', abs(vy),
                   .6, .6, 0., 0., vy, 9)
            memory.update('semantic', [obj], 100.+.1*k, .1, 10.+.1*k)
        self.assertLess(np.linalg.norm(memory.tracks[('semantic', 9)]['velocity']), .3)
        r = PathSafety()
        for k in range(12):
            obj = (2.5+rng.normal(0., .08), -2.5+rng.normal(0., .08), .6, 'fusion_pedestrian',
                   'pedestrian', .45, .6, .6, 0., 0., .45 if k % 2 else -.45, 9)
            result = r.check(PATH, dict(EGO, speed_kmh=4.), [('semantic', [obj], 100.+.1*k, .1)],
                             10.+.1*k, requested_mps=ZONE_MPS, dynamic_only=True)
            self.assertFalse(result.emergency, k)

    def test_real_start_from_the_kerb_is_still_measured(self):
        memory = MotionMemory()
        for k in range(12):
            y = -2.5+max(0., k-7)*.12           # walks toward the path at 1.2 m/s from k = 8
            obj = (6., y, .6, 'fusion_pedestrian', 'pedestrian', 0., .6, .6, 0., 0., 0., 9)
            memory.update('semantic', [obj], 100.+.1*k, .1, 10.+.1*k)
        self.assertGreater(memory.tracks[('semantic', 9)]['velocity'][1], .5)

    def test_tracker_settles_a_jittering_stander(self):
        tracker = pg.PedTracker()
        rng = np.random.default_rng(5)
        for k in range(25):
            t = .1*k
            out = tracker.update([(10.+rng.normal(0., .06), -2.6+rng.normal(0., .06), .6, 'p', 'pedestrian',
                                   .6, .6, .6, 0., 0., .6, 7)], t, t)
        self.assertEqual(tracker.state(7)['state'], 'standing')
        self.assertEqual(out[0][5], 0.)
        for k in range(25, 32):                 # starts walking at 1.2 m/s
            t = .1*k
            out = tracker.update([(10., -2.6+1.2*(t-2.45), .6, 'p', 'pedestrian', 1.2, .6, .6, 0., 0., 1.2, 7)],
                                 t, t)
        self.assertEqual(tracker.state(7)['state'], 'moving')
        self.assertGreater(out[0][10], .8)


class RuntimeGhostTests(unittest.TestCase):
    def test_decision_ignores_the_car_box_grown_on_a_released_walker(self):
        # 162527Z 01:27:43.4-45.6: walker clear of our path, its fusion_npc box held
        # the gap planner at "predicted footprint time overlap" for 1.4 s.
        from unittest.mock import patch
        from test_avoidance_gate import AvoidanceGateTests
        walker = person(8.5, 1.9, 0., 1.2)
        ghost = (10.6, 2.0, 4.6, 'fusion_npc', 'npc', 1.1, 4.6, 1.9, 90., 0., 1.1, 45)
        records = {}
        for name, filtered in (('raw', False), ('filtered', True)):
            runtime = AvoidanceGateTests().runtime(objects=[walker, ghost])
            runtime.test_status['path']['nearest_idx'] = 900
            runtime.test_status['ego']['speed_kmh'] = 0.
            keep_all = lambda: (lambda objects, ego, now, scale=1.: list(objects))
            with patch('gui.USE_LATTICE', True), \
                    patch('gui.PersonGhostFilter', pg.PersonGhostFilter if filtered else keep_all):
                self.assertTrue(runtime.infer())
            records[name] = runtime._web_record
        # Unfiltered, the box alone reproduces the recorded gap-planner stop.
        self.assertEqual((records['raw'].get('yield') or {}).get('name'), 'fusion_npc')
        self.assertEqual(records['raw']['published_cap_kmh'], 0.)
        self.assertNotIn('fusion_npc', json.dumps(records['filtered'].get('yield')))
        self.assertNotIn('fusion_npc', str(records['filtered'].get('stop_reason')))
        self.assertGreater(records['filtered']['published_cap_kmh'], 0.)


class PersonVehicleGhostTests(unittest.TestCase):
    EGO = dict(x=0., y=0., heading_deg=0.)

    def test_grown_car_box_on_a_walker_is_dropped_and_remembered(self):
        walker = person(6.5, .2, 0., -1.3)
        ghost = (8.6, .1, 4.6, 'fusion_npc', 'npc', .9, 4.6, 1.9, -90., 0., -.9, 45)
        self.assertEqual(pg.person_vehicle_ghosts([walker, ghost], self.EGO, .74), {1})
        flt = pg.PersonGhostFilter()
        self.assertEqual(flt([walker, ghost], self.EGO, 0., .74), [walker])
        self.assertEqual(flt([ghost], self.EGO, .5), [])         # person missed for a frame
        self.assertEqual(flt([ghost], self.EGO, 1.2), [ghost])   # memory expires

    def test_real_vehicles_near_a_person_are_kept(self):
        walker = person(6.5, .2, 0., -1.3)
        for car in ((8.6, .1, 4.6, 'fusion_npc', 'npc', 4., 4.6, 1.9, 0., 4., 0., 45),      # faster
                    (8.6, 1.9, 4.6, 'fusion_npc', 'npc', 1., 4.6, 1.9, -90., 0., -1., 45),  # beside
                    (8.6, .1, 4.6, 'lidar_car', 'npc', .9, 4.6, 1.9, -90., 0., -.9, 45),    # measured car
                    (4.4, .2, 4.6, 'fusion_npc', 'npc', .9, 4.6, 1.9, -90., 0., -.9, 45)):  # nearer than them
            with self.subTest(car=car[:2]+car[3:4]):
                self.assertEqual(pg.person_vehicle_ghosts([walker, car], self.EGO, .74), set())


if __name__ == '__main__':
    unittest.main()
