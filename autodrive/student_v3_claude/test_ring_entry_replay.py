"""Roundabout entry replayed on the logged ring traffic (user rules 2026-10-10).

Rules: stop 1 s at the line once (never redone); then enter right behind the rear of
the passing ring car; never in front of an approaching one; once committed only
decelerate behind what is ahead (no stop that waits for a gap).

test_data/ring_entry_replay_20261010.json holds the LiDAR vehicle rows the planner
saw (intersection_lidar of the decision records) for:
  * follow-20261010T113545Z lap 1: the three aborted launches (twitch: 20:40:01.55,
    20:40:21.07, 20:40:28.37) and the entry that worked (20:40:59.55);
  * the four entries of follow-20261010T101327Z / 092032Z (no rtf logged then: the
    replay uses the RTF estimated from ego motion, 0.76-0.82);
  * lap 2 of 113545Z approaching the line (stopped 8.4 m short at 20:46:23.5 by a
    "LiDAR not ready" for new tracks 40-50 m away);
  * both laps of follow-20261010T130317Z and 140051Z (ring_entry in the live loop);
plus every ring car's logged passage of our conflict point (CP, 9.5 m past the line).
Each window's rtf_true (ring NPCs cruise 5.75 m/s sim) times our physics; the planner
gets the logged rtf, which SimClock holds while we stand (0.824 vs 0.733 in 140051Z
lap 2, 0.636 vs ~0.75 in 113545Z lap 1).

The replay keeps the ego standing at its logged stop pose until the planner goes,
then moves it along the route with the measured launch (dead time, acceleration)
under the planner's caps while the logged cars drive on. NPC reactions to that
counterfactual ego are not modelled (they did not happen).
"""
import copy
import json
import math
from pathlib import Path
import unittest

import numpy as np

import ring_entry
from live_path_safety import LivePathSafety
from sim_time import to_sim_objects
from stop_line import INTERSECTION_ACCEL_MPS2, INTERSECTION_ENTRY_MPS, STOP_CAPTURE_M, StopLineController
from traffic_planner import GapPlanner, JunctionTrackMemory, MapVehiclePredictor

ROOT = Path(__file__).parent
DATA = json.loads((ROOT / 'test_data' / 'ring_entry_replay_20261010.json').read_text())
_CACHE = {}


def world():
    if not _CACHE:
        links = json.loads((ROOT / 'link_set.json').read_text())
        route = np.asarray([[p['x'], p['y']] for p in
                            json.loads((ROOT / 'global_path.json').read_text())['waypoints']])
        frame = ring_entry.PathFrame(route[1700:1900])
        line_s = float(frame.project(np.array([-99.3127, 317.799]))[0][0])   # A1256W000709:1781
        ring = next(link for link in links if link['idx'] == 'A2256W000134')
        points = np.asarray(ring['points'], float)[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        conflict = ring_entry.lane_conflict(points, arc, frame, line_s - 5., line_s + 30.)
        _CACHE.update(predictor=MapVehiclePredictor(links), frame=frame, line_s=line_s,
                      s_lane=conflict['s_lane'], s_cp=conflict['s_cp'])
    return _CACHE


def pose(station):
    frame = world()['frame']
    xy = np.array([np.interp(station, frame.s, frame.xy[:, i]) for i in (0, 1)])
    i = int(np.clip(np.searchsorted(frame.s, station), 1, len(frame.s) - 1))
    tangent = frame.xy[i] - frame.xy[i - 1]
    return xy, math.degrees(math.atan2(tangent[1], tangent[0]))


def detector_rows(frame):
    """VehicleReceiver rows (lidar_vehicle_detection) and measured stamps of one frame."""
    rows, stamps = [], {}
    for v in frame['vehicles']:
        length, width = v['size']
        vx, vy = v['velocity']
        rows.append((v['position'][0], v['position'][1], max(length, width), 'lidar_car', 'npc',
                     math.hypot(vx, vy), length, width, v['heading_deg'], vx, vy, v['uid'], 0., None,
                     v['velocity_valid'], v['prediction_age_s']))
        if not v['prediction_age_s'] > 0.:
            stamps[v['uid']] = v['stamp']
    return rows, stamps


# Measured launch (follow-20261010T130317Z, MPC log): first motion 0.36-0.40 s sim after the
# release, 2.5-2.7 m/s^2 sim while the cap is >= 5 km/h above the speed, then tracking the
# cap at ~2 /s. Replaying the planner caps with it reproduces the logged rear-clear times
# (4.79 / 4.31 s sim with the lead-follow ACC cap vs 4.78 / 4.51 s measured).
VEHICLE_LAUNCH = (2.7, .38)
VEHICLE_GAIN = 2.
# The follower's own curve limit through the entry: MPC max_kappa_ahead 0.104 from the line
# to CP+20 m at curve alat 4.2 -> 22.9 km/h (follow-20261010T140051Z: 21.5-22.4 km/h driven
# while the published cap was 24-27 km/h). With it and the window's simulator speed
# (rtf_true) the replay puts our rear past CP 5.08-5.18 / 5.46 s wall after the release
# against 5.14 / 5.48 s measured (140051Z laps 1 / 2).
VEHICLE_VMAX_MPS = 22.9 / 3.6


class EntryReplay:
    """The gui junction pipeline (tracker, track memory, sim units, planner) on logged frames."""
    def __init__(self, name):
        data = world()
        self.window = DATA['windows'][name]
        self.passes = {int(uid): t for uid, t in DATA['ring_cp_passes'][self.window['run']].items()}
        self.planner = GapPlanner()
        self.planner.map_predictor = data['predictor']
        self.planner.ACCEL_MPS2 = INTERSECTION_ACCEL_MPS2
        self.tracker, self.memory = ring_entry.RingMotionTracker(), JunctionTrackMemory()
        # Simulator seconds per wall second of this window for our physics and the reported
        # times; the planner sees the logged (SimClock) rtf like the live runtime did.
        self.rtf = self.window.get('rtf_true') or self.window['frames'][0]['rtf']

    def observe(self, frame, ego_xy):
        rows, stamps = detector_rows(frame)
        rows = self.tracker.update(rows, stamps)
        self.planner.speed_ratio = dict(self.tracker.ratio)
        forecasts = [c for c in rows if c[15] > 0.]
        measured = [c for c in rows if not c[15] > 0.]
        rows = self.memory.update(measured, frame['t'], ego_xy, world()['predictor'])
        known = {c[11] for c in rows}
        return rows + [c for c in forecasts if c[11] not in known]

    def decide(self, frame, station, speed_mps, held, ready=True):
        data = world()
        xy, heading = pose(station)
        rows = self.observe(frame, xy)
        ego = dict(x=float(xy[0]), y=float(xy[1]), heading_deg=heading, speed_kmh=speed_mps * 3.6)
        front = station + ring_entry.EGO_HALF_LENGTH_M
        line_s = data['line_s']
        self.planner.entry_cap_mps = INTERSECTION_ENTRY_MPS if front < line_s + 3. else None
        self.planner.entry_cap_distance_m = max(0., line_s + 12. - station)
        self.planner.junction = dict(held=held, ready=ready, unknown_ids=set(self.memory.unknown_ids),
                                     line_id='entry', line_gap_m=(line_s - front) if held else None,
                                     entry_m=front - line_s, stop_station_m=line_s)
        yaw = math.radians(heading)
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        frame_xy = data['frame'].xy
        i = int(np.argmin(np.linalg.norm(frame_xy - xy, axis=1)))
        path = (frame_xy[max(0, i - 2):i + 120] - xy) @ rotation
        return self.planner.evaluate(to_sim_objects(rows, frame['rtf']), ego, path, 55. / 3.6, frame['t'])

    def stand(self):
        """Index of the first frame standing at the line and our station there (frames
        before it only feed the trackers)."""
        frames = self.window['frames']
        first = next(i for i, f in enumerate(frames)
                     if f['speed_kmh'] <= .5 and f['line_gap_m'] is not None and f['line_gap_m'] <= 1.45)
        station = world()['line_s'] - ring_entry.EGO_HALF_LENGTH_M - frames[first]['line_gap_m']
        for f in frames[:first]:
            self.observe(f, pose(station)[0])
        return first, station

    def run(self, launch=VEHICLE_LAUNCH, brake=2.5, gain=VEHICLE_GAIN):
        """Stand at the logged stop pose until the planner goes, then drive the entry.

        Vehicle: ``launch`` = (acceleration, dead time) in simulator units, then it tracks
        the planner's cap with ``gain`` (1/s), braking at most ``brake``."""
        return self.drive(*self.stand(), launch=launch, brake=brake, gain=gain)

    def drive(self, start, station, launch=VEHICLE_LAUNCH, brake=2.5, gain=VEHICLE_GAIN):
        data = world()
        frames = self.window['frames']
        log, go, speed, moved, previous, model_rear = [], None, 0., 0., None, None
        t_nose = t_rear = None
        for f in frames[start:]:
            if t_rear is not None and f['t'] > t_rear + 2.:
                break             # the entry is done (the replay path ends ~60 m on)
            if go is not None and previous is not None:
                dt = (f['t'] - previous) * self.rtf             # simulator seconds
                active = min(dt, max(0., moved + dt - launch[1]))
                moved += dt
                cap = min(log[-1]['cap'], VEHICLE_VMAX_MPS)
                rate = float(np.clip(gain * (cap - speed), -brake, launch[0]))
                new = max(0., speed + rate * active)
                station += .5 * (speed + new) * dt
                speed = max(0., new)
                if t_nose is None and station + ring_entry.EGO_HALF_LENGTH_M >= data['s_lane']:
                    t_nose = f['t']
                if t_rear is None and station >= data['s_cp'] + ring_entry.EGO_HALF_LENGTH_M:
                    t_rear = f['t']
            previous = f['t']
            result = self.decide(f, station, speed, held=go is None)
            kind = result['kind'] if result else None
            if go is None and kind != 'stop':
                go = f            # stop line released: committed, following only
                checks = (self.planner.status.get('ring') or {}).get('checks') or []
                model_rear = max([c['rear_clear_s'] for c in checks], default=None)
                self.planner.latch_junction_commit()
            log.append(dict(hms=f['hms'], t=f['t'], kind=kind, reason=result and result['reason'],
                            cap=result['v_mps'] if result else 55. / 3.6, speed=speed, station=station,
                            committed=self.planner.junction_committed,
                            ring=dict(self.planner.status.get('ring') or {})))
        return dict(log=log, go=go, t_nose=t_nose, t_rear=t_rear, model_rear=model_rear)

    def every_go(self):
        """The entry the planner would start at every frame it says go while the logged car
        still stood at the line (later frames carry the logged car's own launch)."""
        data = world()
        first, station = self.stand()
        frames = self.window['frames']
        keep = {id(data['predictor']): data['predictor'], id(DATA): DATA, id(self.window): self.window,
                id(data['frame']): data['frame']}
        launched = logged_launch_t(self.window)
        entries = []
        for i in range(first, len(frames)):
            if frames[i]['t'] >= launched:
                break
            fork = copy.deepcopy(self, dict(keep))
            result = self.decide(frames[i], station, 0., held=True)
            if result and result['kind'] != 'stop':
                entries.append((frames[i], fork.drive(i, station)))
        return entries

    def next_ring_car(self, out):
        """(uid, CP passage) of the first ring car to reach our CP after our nose entered."""
        later = sorted((t, uid) for uid, t in self.passes.items() if t > out['t_nose'] - .3)
        return (later[0][1], later[0][0]) if later else (None, None)

    def post_encroachment_s(self, out, uid, step=.02, span=2.5):
        """Simulator seconds our replayed entry could start later before our body touches
        car ``uid`` on its logged boxes (negative: it touches; how much earlier we had to be)."""
        boxes = {}
        for f in self.window['frames']:
            for v in f['vehicles']:
                if v['uid'] == uid and not v['prediction_age_s'] > 0 and \
                        out['go']['t'] - 1. <= v['stamp'] <= out['t_rear'] + 4.:
                    length, width = v['size']
                    boxes[v['stamp']] = (v['stamp'], v['position'][0], v['position'][1],
                                         math.radians(v['heading_deg']), max(length, width), min(length, width))
        theirs = np.asarray(sorted(boxes.values()))
        driven = [row for row in out['log'] if row['t'] >= out['go']['t']]
        t = np.array([row['t'] for row in driven])
        station = np.array([row['station'] for row in driven])
        shifts = np.round(np.arange(-span, span + 1e-9, step), 6)
        when = theirs[None, :, 0] + shifts[:, None]
        along = np.where(when > t[-1], station[-1] + driven[-1]['speed'] * self.rtf * (when - t[-1]),
                         np.interp(when, t, station))
        frame = world()['frame']
        i = np.clip(np.searchsorted(frame.s, along), 1, len(frame.s) - 1)
        tangent = frame.xy[i] - frame.xy[i - 1]
        ours = body_corners(np.interp(along, frame.s, frame.xy[:, 0]), np.interp(along, frame.s, frame.xy[:, 1]),
                            np.arctan2(tangent[..., 1], tangent[..., 0]), 2 * ring_entry.EGO_HALF_LENGTH_M,
                            2 * ring_entry.EGO_HALF_WIDTH_M)
        hit = bodies_overlap(ours, np.broadcast_to(
            body_corners(theirs[:, 1], theirs[:, 2], theirs[:, 3], theirs[:, 4], theirs[:, 5]), ours.shape)).any(1)
        zero = int(np.argmin(np.abs(shifts)))
        if not hit[zero]:
            touch = np.flatnonzero(hit[:zero][::-1])         # starting later = earlier stations
            return (touch[0] + 1) * step * self.rtf if len(touch) else span * self.rtf
        free = np.flatnonzero(~hit[zero:])
        return -free[0] * step * self.rtf if len(free) else -span * self.rtf


def body_corners(x, y, heading, length, width):
    """Corners (..., 4, 2) of rectangles centred at x, y."""
    x, y, heading = np.asarray(x, float), np.asarray(y, float), np.asarray(heading, float)
    length, width = np.broadcast_to(length, x.shape), np.broadcast_to(width, x.shape)
    c, s = np.cos(heading)[..., None], np.sin(heading)[..., None]
    dx = np.array([.5, .5, -.5, -.5]) * length[..., None]
    dy = np.array([.5, -.5, -.5, .5]) * width[..., None]
    return np.stack([x[..., None] + c * dx - s * dy, y[..., None] + s * dx + c * dy], -1)


def bodies_overlap(a, b):
    """Separating-axis test of rectangle arrays a, b (..., 4, 2)."""
    apart = np.zeros(a.shape[:-2], bool)
    for poly in (a, b):
        for k in (0, 1):
            edge = poly[..., k + 1, :] - poly[..., k, :]
            normal = np.stack([-edge[..., 1], edge[..., 0]], -1)[..., None, :]
            pa, pb = (a * normal).sum(-1), (b * normal).sum(-1)
            apart |= (pa.max(-1) < pb.min(-1)) | (pb.max(-1) < pa.min(-1))
    return ~apart


def logged_launch_t(window):
    """When the logged car left its stand at the line (moved 1 m or 3 km/h): later frames
    show ring cars reacting to it, not to a counterfactual entry."""
    frames = window['frames']
    first = next(i for i, f in enumerate(frames)
                 if f['speed_kmh'] <= .5 and f['line_gap_m'] is not None and f['line_gap_m'] <= 1.45)
    x0 = frames[first]['ego']
    for f in frames[first:]:
        if math.hypot(f['ego'][0] - x0[0], f['ego'][1] - x0[1]) > 1. or f['speed_kmh'] > 3.:
            return f['t']
    return math.inf


def hms_time(window, hms):
    frame = next(f for f in DATA['windows'][window]['frames'] if f['hms'] >= hms)
    return frame['t']


class RingEntryReplayTests(unittest.TestCase):
    ABORTED = (('twitch1', '20:40:01.55'), ('twitch2', '20:40:21.07'), ('twitch4', '20:40:28.37'))
    SUCCESSFUL = ('success_lap1', 'success_101327_lap1', 'success_101327_lap2',
                  'success_092032_lap1', 'success_092032_lap2')

    def check_entry(self, name, launch=ring_entry.LAUNCH_NOMINAL, minimum_margin_s=ring_entry.NEXT_MARGIN_S):
        replay = EntryReplay(name)
        out = replay.run(launch)
        self.assertIsNotNone(out['go'], name)
        after = [row for row in out['log'] if row['t'] >= out['go']['t']]
        # One committed entry: never a stop again, never back to the entry decision.
        self.assertTrue(all(row['kind'] != 'stop' and row['committed'] for row in after),
                        [(row['hms'], row['kind'], row['reason']) for row in after])
        self.assertIsNotNone(out['t_rear'], name)
        rtf = replay.rtf
        passes = sorted(replay.passes.values())
        # Nobody reaches our conflict point while our body occupies the ring lane ...
        self.assertFalse([p for p in passes if out['t_nose'] - .3 < p < out['t_rear']], name)
        # ... the car we followed was ahead of our nose ...
        before = [p for p in passes if p <= out['t_nose']]
        self.assertTrue(before and out['t_nose'] - before[-1] >= .5, name)
        # ... and the next one arrives NEXT_MARGIN_S (sim) after our rear has cleared it.
        later = [p for p in passes if p >= out['t_rear']]
        if later:
            self.assertGreaterEqual((later[0] - out['t_rear']) * rtf, minimum_margin_s, name)
        return replay, out

    def test_aborted_launches_become_one_correctly_timed_committed_entry(self):
        for name, aborted in self.ABORTED:
            with self.subTest(name=name):
                replay, out = self.check_entry(name)
                aborted_t = hms_time(name, aborted)
                # The logged launch moment (the passing car already at our CP, the next one
                # 23-29 m behind) is not an entry gap any more.
                at_abort = [row for row in out['log'] if abs(row['t'] - aborted_t) < .05]
                if out['go']['t'] > aborted_t:
                    self.assertEqual(at_abort[0]['kind'], 'stop', at_abort)
                    self.assertIn('wait at the line', at_abort[0]['reason'])

    def test_twitch1_waits_while_the_next_ring_car_is_too_close(self):
        # 20:40:01.55: 500007 at our CP, 500008 29 m up the ring - it reached our CP
        # 0.30 s wall before our rear would have cleared it (the logged abort).
        replay, out = self.check_entry('twitch1')
        rows = [row for row in out['log'] if row['hms'] in ('20:40:01.55', '20:40:01.87', '20:40:02.21')]
        self.assertEqual(len(rows), 3)
        for row in rows:
            self.assertEqual(row['kind'], 'stop', row)
            self.assertFalse(row['committed'])
            self.assertEqual(row['ring']['blocker'], 500008)
            self.assertLess(row['ring']['margin_s'], ring_entry.NEXT_MARGIN_S)
        self.assertGreaterEqual(out['go']['t'], hms_time('twitch1', '20:40:05.0'))

    def test_successful_entries_stay_single_committed_entries(self):
        for name in self.SUCCESSFUL:
            with self.subTest(name=name):
                self.check_entry(name)

    def test_entries_keep_a_margin_with_a_slower_launch(self):
        # Slower than measured (0.6 s dead time, 2.4 m/s^2): still no car at our CP while
        # we occupy the ring lane.
        for name in [name for name, _ in self.ABORTED] + list(self.SUCCESSFUL):
            with self.subTest(name=name):
                self.check_entry(name, launch=(2.4, .6), minimum_margin_s=.8)


class Lap2ApproachTests(unittest.TestCase):
    def test_far_unknown_tracks_do_not_stop_the_approach(self):
        # 20:46:22.4-25.3: new/standing tracks at (-113.9, 356.3), (-91.3, 357.1) - 40-50 m
        # away, 50+ m up the ring - made the live check stop us 8.4 m before the line.
        replay = EntryReplay('lap2_approach')
        line_s = world()['line_s']
        for f in replay.window['frames']:
            if f['line_gap_m'] is None or f['line_gap_m'] < 2.:
                continue
            station = line_s - ring_entry.EGO_HALF_LENGTH_M - f['line_gap_m']
            speed = min(f['speed_kmh'], 1.) / 3.6       # also at the slow end of the braking
            result = replay.decide(f, station, speed, held=True)
            self.assertEqual(result['kind'], 'follow', (f['hms'], result))
            self.assertEqual(result['reason'], 'junction: approach mandatory stop line')
            self.assertGreater(result['v_mps'], 1.)
        self.assertTrue(replay.memory.unknown_ids or any(not v['velocity_valid']
                        for f in replay.window['frames'] for v in f['vehicles']))

    def test_live_check_has_no_readiness_stop_before_the_commit(self):
        # Before the commit the published decision owns waiting at the line; the live
        # check adds no readiness/unknown/overlap stop of its own.
        from types import SimpleNamespace
        from path_safety import SafetyResult
        guard = LivePathSafety.__new__(LivePathSafety)
        guard.traffic = GapPlanner()
        runtime = SimpleNamespace(gap_planner=GapPlanner())
        path = np.column_stack((np.arange(0., 30., .5), np.zeros(60)))
        unknown = (6., 12., 4.6, 'lidar_car', 'npc', 0., 4.6, 1.9, 0., 0., 0., 5, 0., None, False)
        out = guard.junction_follow(SafetyResult(5., False, 30., None, {}), runtime, [unknown], 1.,
                                    path, dict(x=0., y=0., speed_kmh=3.))
        self.assertFalse(out.emergency)
        self.assertEqual(out.cap_mps, 5.)
        self.assertEqual(out.validation['junction_traffic']['state'], 'WAIT')


class CommittedFollowTests(unittest.TestCase):
    PATH = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))

    @staticmethod
    def car(x, y, vx, vy, uid=7):
        heading = math.degrees(math.atan2(vy, vx)) if (vx or vy) else 0.
        return (x, y, 4.9, 'lidar_car', 'npc', math.hypot(vx, vy), 4.9, 2.1, heading, vx, vy, uid, 0., None, True)

    def test_committed_entry_only_decelerates_behind_a_slowing_leader(self):
        planner = GapPlanner()
        planner.junction = dict(held=False, ready=False, unknown_ids={99}, line_gap_m=None, entry_m=1.)
        planner.junction_committed = True
        speed, gap, lead, now, caps = 6., 14., 6., 100., []
        for step in range(80):
            lead = max(0., lead - .1)                # leader brakes to a standstill
            leader = self.car(gap + 2.3175 + 2.45, 0., lead, 0.)
            ego = dict(x=0., y=0., heading_deg=0., speed_kmh=speed * 3.6)
            # unknown/unready tracks and a stale camera stream must not stop us now
            result = planner.evaluate([leader, self.car(5., 6., 0., 0., 99)], ego, self.PATH, 8.3,
                                      now, fresh=step % 3 != 0)
            self.assertNotEqual(result['kind'], 'stop', result)
            self.assertTrue(planner.junction_committed)
            caps.append(result['v_mps'])
            speed = max(0., min(result['v_mps'], speed + .3) if result['v_mps'] >= speed else
                        max(result['v_mps'], speed - 2.5 * .1))
            gap += (lead - speed) * .1
            now += .1
            self.assertGreater(gap, 0., step)
        self.assertLess(caps[-1], .5)                 # behind the stopped leader, as following does
        self.assertGreater(gap, .3)

    def test_car_beside_us_does_not_stop_the_committed_entry(self):
        planner = GapPlanner()
        planner.junction = dict(held=False, ready=True, unknown_ids=set(), line_gap_m=None, entry_m=2.)
        planner.junction_committed = True
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=10.)
        side = self.car(-1., 3., 6., -2.)              # ring car closing on our left rear
        result = planner.evaluate([side], ego, self.PATH, 8.3, 100.)
        self.assertNotEqual(result['kind'], 'stop', result)
        self.assertAlmostEqual(result['v_mps'], planner.status['target_mps'])
        self.assertGreater(result['v_mps'], 5.)


class StopLineHoldLatchTests(unittest.TestCase):
    LINE = dict(id='entry', station_m=50., xy=[50., 0.])
    STATION = 50. - StopLineController.FRONT_M - 1.

    def test_hold_latch_survives_long_ticks_and_stale_observations(self):
        # follow-20261010T113545Z 39:47.74-40:00.99: the completed hold was reset by
        # 0.62/0.49/0.72 s decision gaps and 0.43-0.60 s pose ages (three times).
        gate = StopLineController([self.LINE])
        now, result = 0., None
        for _ in range(12):                          # 1.1 s of regular, fresh standstill
            result = gate.update(self.STATION, 0., now, clear=False, observation_age_s=.2)
            now += .1
        self.assertTrue(result['hold_latched'])
        self.assertEqual(result['state'], 'WAIT_CLEAR')
        for gap, age in ((.62, .558), (.49, .43), (.72, .598), (1.1, .74), (.6, .2)):
            now += gap
            result = gate.update(self.STATION, 0., now, clear=False, observation_age_s=age)
            self.assertEqual(result['state'], 'WAIT_CLEAR', (gap, age))
            self.assertTrue(result['hold_latched'])
        result = gate.update(self.STATION, 0., now + .1, clear=True, observation_age_s=.2)
        self.assertEqual(result['state'], 'ENTER')   # the first fresh clear gap: no second hold

    def test_long_tick_before_the_latch_keeps_the_time_held(self):
        gate = StopLineController([self.LINE])
        for i in range(6):                           # 0.5 s held
            gate.update(self.STATION, 0., i * .1, clear=True, observation_age_s=.2)
        result = gate.update(self.STATION, 0., 1.2, clear=True, observation_age_s=.2)   # 0.7 s gap
        self.assertEqual(result['state'], 'HOLD')
        self.assertAlmostEqual(result['held_sim_s'], .5)
        result = gate.update(self.STATION, 0., 1.3, clear=True, observation_age_s=.7)  # stale pose
        self.assertAlmostEqual(result['held_sim_s'], .5)
        for i in range(1, 6):
            result = gate.update(self.STATION, 0., 1.3 + i * .1, clear=True, observation_age_s=.2)
        self.assertEqual(result['state'], 'ENTER')

    def test_hold_counts_simulator_seconds(self):
        gate = StopLineController([self.LINE])
        released = None
        for i in range(40):
            result = gate.update(self.STATION, 0., i * .1, clear=True, observation_age_s=.2, time_scale=.636)
            if result['state'] == 'ENTER':
                released = i * .1
                break
        self.assertAlmostEqual(released, 1.6, places=6)    # 1.0 s sim at RTF 0.636 (+1 tick)


class EntrySpeedTests(unittest.TestCase):
    """follow-20261010T130317Z (first drive with ring_entry): no twitch, no stop in the ring,
    but the entry took 4.78 / 4.51 s sim from the release to our rear past the CP."""
    WINDOWS = ('drive130317_lap1', 'drive130317_lap2')

    def test_replayed_entries_clear_the_ring_lane_within_about_four_seconds(self):
        # Planner caps only (the lead-follow ACC cap no longer applies once committed) with
        # the measured launch: 4.07 / 4.03 s sim (4.43 / 4.07 at the logged release times).
        for name in self.WINDOWS:
            with self.subTest(name=name):
                replay, out = RingEntryReplayTests().check_entry(name)
                rtf = replay.rtf
                self.assertLessEqual((out['t_rear'] - out['go']['t']) * rtf, 4.25)

    def test_rear_clear_model_is_not_optimistic_against_the_replayed_launch(self):
        # The model's rear-clear time at the go (entry_decision) against the replayed launch.
        # Not 113545Z lap 1 (twitch2: -0.51 s): its planner converted with a held RTF of 0.636
        # against ~0.75, so the slow leader it followed in read 17 % fast; that entry's
        # next ring car still came 1.87 s sim body to body after us (NextMarginTests).
        for name in ('twitch4', 'success_101327_lap1', 'success_101327_lap2', 'success_092032_lap1',
                     'success_092032_lap2', 'drive140051_lap1', 'drive140051_lap2') + self.WINDOWS:
            with self.subTest(name=name):
                replay = EntryReplay(name)
                out = replay.run()
                rtf = replay.rtf
                self.assertIsNotNone(out['model_rear'])
                self.assertGreaterEqual(out['model_rear'], (out['t_rear'] - out['go']['t']) * rtf - .2, name)

    def test_lap2_goes_on_the_first_safe_cycle(self):
        # Logged: rear model 4.5-5.8 s vs the next car's 3.6-4.7 s, waited until 22:12:17.45.
        replay, out = RingEntryReplayTests().check_entry('drive130317_lap2')
        self.assertLess(out['go']['t'], hms_time('drive130317_lap2', '22:12:12.0'))

    def test_exit_curve_does_not_cap_the_turn_in(self):
        # A curve 25 m ahead is reached braking comfortably, not driven at from the line.
        planner = GapPlanner()
        planner.junction = dict(held=False, ready=True, unknown_ids=set(), line_gap_m=None, entry_m=0.)
        planner.junction_committed = True
        straight = np.column_stack((np.arange(0., 25.01, .5), np.zeros(51)))
        angle = np.linspace(0., math.pi / 2., 40)[1:]
        curve = np.column_stack((25. + 8. * np.sin(angle), 8. * (1. - np.cos(angle))))   # R 8 m
        path = np.vstack((straight, curve))
        result = planner.evaluate([], dict(x=0., y=0., heading_deg=0., speed_kmh=0.), path, 15., 100.)
        bend = math.sqrt(4.2 * 8.)
        self.assertGreater(result['v_mps'], bend + 2.)
        self.assertLessEqual(result['v_mps'], math.sqrt(bend ** 2 + 2. * ring_entry.CURVE_BRAKE_MPS2 * 28.))
        # In the bend it is the bend's speed (2 m chord curvature).
        limit = planner._path_geometry(dict(x=0., y=0., heading_deg=0.), path)[3]
        self.assertLess(limit(30.), bend + .3)
        self.assertGreater(limit(10.), bend + 3.)


_EVERY_GO = {}


def every_go(name):
    if name not in _EVERY_GO:
        replay = EntryReplay(name)
        _EVERY_GO[name] = (replay, replay.every_go())
    return _EVERY_GO[name]


class NextMarginTests(unittest.TestCase):
    """ring_entry.NEXT_MARGIN_S on the logged ring traffic. follow-20261010T140051Z waited
    5.4 s in lap 1 at a model margin of 0.98 < 1.0 and 5.1 s in lap 2 at 0.16-0.17."""
    WINDOWS = ('twitch2', 'twitch4', 'success_lap1', 'success_101327_lap2', 'success_092032_lap1',
               'success_092032_lap2', 'drive130317_lap1', 'drive130317_lap2', 'drive140051_lap1',
               'drive140051_lap2')

    def test_every_go_at_the_line_leaves_the_next_ring_car_half_a_second(self):
        # Every frame the planner says go while the logged car still stood at the line,
        # replayed as a committed entry; the next ring car's logged boxes against ours:
        # 99 goes, worst 0.66 s sim body to body (140051Z 23:04:38.22, 23:10:00.52). At a
        # margin of 0.16 (130317Z 22:12:06.47) our bodies would have touched: still a wait.
        checked = 0
        for name in self.WINDOWS:
            replay, goes = every_go(name)
            for frame, out in goes:
                with self.subTest(name=name, hms=frame['hms']):
                    self.assertIsNotNone(out['t_rear'])
                    uid, _ = replay.next_ring_car(out)
                    if uid is None:
                        continue
                    self.assertGreaterEqual(replay.post_encroachment_s(out, uid), .5)
                    checked += 1
        self.assertGreater(checked, 80)

    def test_140051_lap1_goes_when_the_hold_completes(self):
        # Logged: hold complete 23:04:37.91 at a margin of 0.98, released 23:04:43.27.
        _, goes = every_go('drive140051_lap1')
        done = hms_time('drive140051_lap1', '23:04:37.91')
        self.assertTrue(any(abs(frame['t'] - done) < .05 for frame, _ in goes))


class StopLineApproachTests(unittest.TestCase):
    """follow-20261010T130317Z: HOLD commanded at 1.61 / 1.71 m (~5 km/h), rest at 1.37-1.54 m,
    then APPROACH/HOLD toggled at 0.3-3 km/h down to 1.18-1.23 m; the 1 s hold completed at
    22:07:01.13 / 22:12:07.05."""
    CASES = (('drive130317_lap1', '22:06:57.0', '22:07:01.13'), ('drive130317_lap2', '22:12:03.4', '22:12:07.05'))

    def replay(self, name, start):
        frames = [f for f in DATA['windows'][name]['frames'] if f['hms'] >= start and f['line_gap_m'] is not None]
        gate = StopLineController([dict(id='entry', station_m=50., xy=[50., 0.])])
        states, rest, held_at = [], None, None
        for f in frames:
            gap, speed = f['line_gap_m'], f['speed_kmh'] / 3.6
            commanded = any(state != 'APPROACH' for _, state, _ in states)
            if rest is None and commanded and speed <= .15:
                rest = gap
            if rest is not None:            # the stop stays commanded: the car stays put
                gap, speed = rest, 0.
            result = gate.update(50. - StopLineController.FRONT_M - gap, speed, f['t'], clear=False,
                                 observation_age_s=f['obs_age_s'] or .1, time_scale=f['rtf'])
            states.append((f['hms'], result['state'], gap))
            if held_at is None and result.get('hold_latched'):
                held_at = f
        return states, rest, held_at

    def test_one_clean_stop_inside_the_capture_window(self):
        for name, start, logged in self.CASES:
            with self.subTest(name=name):
                states, rest, held_at = self.replay(name, start)
                first = next(i for i, (_, state, _) in enumerate(states) if state != 'APPROACH')
                self.assertTrue(all(state != 'APPROACH' for _, state, _ in states[first:]), states[first:])
                self.assertLessEqual(rest, STOP_CAPTURE_M)
                self.assertIsNotNone(held_at)
                # The 1 s (sim) hold is done well before the logged completion.
                self.assertLess(held_at['t'], hms_time(name, logged) - 1.)


class AtTheLineTests(unittest.TestCase):
    def test_commanded_stop_short_of_the_rest_window_is_at_the_line(self):
        # Lap 2 rested at 1.54 m (rest window 1.40 m): the stop line holds us there, so the
        # entry decision is made there - not "approach mandatory stop line".
        planner = GapPlanner()
        planner.junction = dict(held=True, ready=True, unknown_ids=set(), line_gap_m=1.54, entry_m=-1.54)
        path = np.column_stack((np.arange(0., 40.01, .5), np.zeros(81)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        self.assertEqual(planner.evaluate([], ego, path, 8., 100.)['reason'], 'junction: approach mandatory stop line')
        planner.junction['at_line'] = True
        result = planner.evaluate([], ego, path, 8., 100.1)
        self.assertEqual(result['reason'], 'gap accepted with acceleration and clearance')


class QueueAtTheLineTests(unittest.TestCase):
    def test_car_queued_behind_us_never_holds_the_entry(self):
        planner = GapPlanner()
        planner.junction = dict(held=True, ready=True, unknown_ids={31}, line_gap_m=1., entry_m=-1.)
        path = np.column_stack((np.arange(-1., 40.01, .5), np.zeros(83)))
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=0.)
        behind = (-7., .2, 4.6, 'lidar_car', 'npc', 0., 4.6, 1.9, 0., 0., 0., 31, 0., None, False)
        standing = (-7.5, -.1, 4.6, 'lidar_car', 'npc', 0., 4.6, 1.9, 0., 0., 0., 32, 0., None, True)
        result = planner.evaluate([behind, standing], ego, path, 8., 100.)
        self.assertNotEqual(result['kind'], 'stop', result)


class RingMotionTrackerTests(unittest.TestCase):
    def test_turning_ring_car_speed_from_positions_not_detector_spikes(self):
        # 500014 at the ring node, 20:40:35.7-39.0 (113545Z): detector speeds up to 7.0 m/s
        # wall (11 m/s sim) while its boxes moved 3.3-4.4 m/s wall (true 5.75 m/s sim).
        track = ring_entry.RingMotionTracker()
        samples = ((34.859, -118.6, 338.19, .57, -4.21), (35.741, -116.5, 334.1, 2.31, -4.83),
                   (36.791, -112.1, 328.97, 4.13, -5.66), (38.076, -107.22, 326.03, 3.89, -2.18),
                   (38.406, -106.58, 325.85, 3.19, -1.27), (38.956, -104.36, 325.59, 3.01, -.5))
        speed = None
        for stamp, x, y, vx, vy in samples:
            row = (x, y, 5., 'lidar_car', 'npc', math.hypot(vx, vy), 5., 2.1, 0., vx, vy, 500014, 0., None, True, 0.)
            (out,) = track.update([row], {500014: stamp})
            speed = out[5]
        self.assertTrue(3. <= speed <= 4.6, speed)
        self.assertEqual(track.ratio[500014], ring_entry.FIT_RATIO)

    def test_spawned_ring_car_without_velocity_is_not_relevant(self):
        # Every ring NPC appears standing at the spawn point (-110.8, 357.9), 52 m up the
        # ring from our CP: its unmeasured first frames no longer hold the entry.
        data = world()
        planner = GapPlanner()
        planner.map_predictor = data['predictor']
        planner.junction = dict(held=True, ready=True, unknown_ids={500020}, line_gap_m=.9,
                                entry_m=-.9, stop_station_m=data['line_s'])
        station = data['line_s'] - ring_entry.EGO_HALF_LENGTH_M - .9
        xy, heading = pose(station)
        frame_xy = data['frame'].xy
        i = int(np.argmin(np.linalg.norm(frame_xy - xy, axis=1)))
        yaw = math.radians(heading)
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        path = (frame_xy[max(0, i - 2):i + 120] - xy) @ rotation
        spawn = (-110.8, 357.9, 5., 'lidar_car', 'npc', 0., 5., 2.1, 224., 0., 0., 500020, 0., None, False, 0.)
        ego = dict(x=float(xy[0]), y=float(xy[1]), heading_deg=heading, speed_kmh=0.)
        result = planner.evaluate([spawn], ego, path, 55. / 3.6, 100.)
        self.assertNotEqual(result['kind'], 'stop', result)
        # The same unmeasured car 15 m up the ring still holds us at the line.
        near = (-112.1, 328.97, 5., 'lidar_car', 'npc', 0., 5., 2.1, 126., 0., 0., 500021, 0., None, False, 0.)
        planner.junction['unknown_ids'] = {500021}
        result = planner.evaluate([near], ego, path, 55. / 3.6, 100.1)
        self.assertEqual(result['kind'], 'stop', result)
        self.assertIn('without measured velocity', result['reason'])


if __name__ == '__main__':
    unittest.main()
