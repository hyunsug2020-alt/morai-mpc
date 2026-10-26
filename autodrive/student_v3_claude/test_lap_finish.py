"""One-lap runs (lap_finish.py): lap detection on the real closed route and the stop cap."""
import json
import math
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

import lap_finish
from lap_finish import (ARM_FRACTION, FINISH_WINDOW_M, OVERSHOOT_M, LapTracker,
                        lap_marker_path, lap_tracker_from_env, write_lap_marker)

ROUTE = np.asarray([(p["x"], p["y"]) for p in
                    json.loads((Path(__file__).parent / "global_path.json").read_text())["waypoints"]])
CONTEST_404_START = (-137.6, -439.5)       # 13 m before the route's index wrap
HARD_START = (-131.5, -428.0)              # right after the wrap (route idx 1)


class RouteWalker:
    """Moves a pose along the closed route by station (both directions)."""

    def __init__(self, route):
        xy = np.asarray(route, float)
        if np.hypot(*(xy[0] - xy[-1])) < 1e-6:
            xy = xy[:-1]
        self.xy = np.vstack((xy, xy[:1]))
        seg = np.hypot(*np.diff(self.xy, axis=0).T)
        self.cum = np.concatenate(([0.], np.cumsum(seg)))
        self.length = float(self.cum[-1])

    def at(self, station, lateral=0.):
        s = station % self.length
        i = min(int(np.searchsorted(self.cum, s, side="right")) - 1, len(self.xy) - 2)
        t = (s - self.cum[i]) / max(self.cum[i + 1] - self.cum[i], 1e-9)
        p = self.xy[i] + t * (self.xy[i + 1] - self.xy[i])
        d = self.xy[i + 1] - self.xy[i]
        n = np.array((-d[1], d[0])) / max(np.hypot(*d), 1e-9)
        return p + lateral * n

    def station_of(self, xy):
        i = int(np.argmin(np.hypot(*(self.xy[:-1] - np.asarray(xy)).T)))
        return float(self.cum[i])


class Drive:
    """Kinematic car following the tracker's cap (decel <= 3 m/s^2, as the follower)."""

    def __init__(self, tracker, start_station, cruise=13.9, dt=.1, now=1000.):
        self.tracker, self.walker = tracker, RouteWalker(ROUTE)
        self.station, self.speed, self.cruise, self.dt, self.now = start_station, 0., cruise, dt, now
        self.history = []
        self.status = tracker.update(*self.walker.at(self.station), 0., self.now)

    def step(self, target=None, direction=1., noise=0., rng=None):
        cap = self.status["cap_mps"]
        want = self.cruise if target is None else target
        if cap is not None:
            want = min(want, cap)
        if want < self.speed:
            self.speed = max(want, self.speed - 3. * self.dt)
        else:
            self.speed = min(want, self.speed + 2. * self.dt)
        self.station += direction * self.speed * self.dt
        self.now += self.dt
        xy = self.walker.at(self.station)
        if noise and rng is not None:
            xy = xy + rng.normal(0., noise, 2)
        self.status = self.tracker.update(xy[0], xy[1], self.speed, self.now)
        self.history.append(dict(self.status, speed=self.speed, station=self.station))
        return self.status

    def run_until_done(self, limit_s=600.):
        steps = int(limit_s / self.dt)
        for _ in range(steps):
            if self.step()["done"]:
                return True
        return False


class LapTrackerTests(unittest.TestCase):
    def setUp(self):
        self.walker = RouteWalker(ROUTE)

    def lap_from(self, xy):
        tracker = LapTracker(ROUTE)
        drive = Drive(tracker, self.walker.station_of(xy))
        return tracker, drive

    def assert_clean_lap(self, tracker, drive, start_station):
        self.assertTrue(drive.run_until_done(), drive.status)
        self.assertFalse(tracker.overshoot)
        # Never complete or brake before the far side of the loop has been driven.
        first_cap = next(i for i, h in enumerate(drive.history) if h["cap_mps"] is not None)
        self.assertGreaterEqual(drive.history[first_cap]["progress_m"], ARM_FRACTION * tracker.length - 2.)
        # Rest at the start point, at a smooth braking rate (no emergency stop).
        driven = drive.station - start_station
        self.assertAlmostEqual(driven, tracker.length, delta=FINISH_WINDOW_M)
        stop_xy = self.walker.at(drive.station)
        start_xy = self.walker.at(start_station)
        self.assertLess(np.hypot(*(stop_xy - start_xy)), FINISH_WINDOW_M)
        # The finishing stop brakes along the lap profile (1.5 m/s^2 + the sqrt tail
        # near zero), well below the follower's 3 m/s^2 limit an abrupt zero would hit.
        speeds = [h["speed"] for h in drive.history[first_cap:]]
        decel = max(a - b for a, b in zip(speeds, speeds[1:])) / drive.dt
        self.assertLessEqual(decel, 2. + 1e-6)
        self.assertGreater(speeds[0], 10.)       # cruising when the finish was armed
        self.assertEqual(drive.status["cap_mps"], 0.)
        # The marker payload.
        record = tracker.record(*stop_xy, wall_time=123., run="follow-test")
        self.assertTrue(record["lap_complete"])
        self.assertLess(record["stop"]["distance_from_start_m"], FINISH_WINDOW_M)
        self.assertGreater(record["lap_time_s"], tracker.length / drive.cruise)
        self.assertLess(record["lap_time_s"], tracker.length / drive.cruise + 60.)
        json.dumps(record)

    def test_route_is_one_closed_loop(self):
        tracker = LapTracker(ROUTE)
        self.assertAlmostEqual(tracker.length, 2184.54, delta=.1)
        self.assertEqual(len(tracker.xy), len(ROUTE) - 1)

    def test_start_just_before_the_index_wrap(self):
        start = self.walker.station_of(CONTEST_404_START)
        self.assertGreater(start, self.walker.length - 20.)
        tracker, drive = self.lap_from(CONTEST_404_START)
        # Crossing the wrap a few metres after the start is ordinary progress.
        for _ in range(100):
            drive.step()
        self.assertGreater(drive.status["progress_m"], 30.)
        self.assertLess(drive.status["progress_m"], 120.)
        self.assertFalse(drive.status["armed"])
        self.assert_clean_lap(tracker, drive, start)

    def test_start_right_after_the_wrap(self):
        tracker, drive = self.lap_from(HARD_START)
        self.assert_clean_lap(tracker, drive, self.walker.station_of(HARD_START))

    def test_start_mid_route(self):
        start = 1000.
        tracker = LapTracker(ROUTE)
        drive = Drive(tracker, start)
        self.assert_clean_lap(tracker, drive, start)

    def test_localization_jitter_at_the_start_never_counts(self):
        tracker = LapTracker(ROUTE)
        rng = np.random.default_rng(3)
        start = self.walker.at(self.walker.station_of(CONTEST_404_START))
        for k in range(3000):
            xy = start + rng.normal(0., .5, 2)
            status = tracker.update(xy[0], xy[1], abs(rng.normal(0., .1)), 1000. + .066 * k)
            self.assertFalse(status["armed"])
            self.assertFalse(status["done"])
            self.assertIsNone(status["cap_mps"])
        self.assertLess(abs(status["progress_m"]), 2.5)
        self.assertEqual(status["restarts"], 0)

    def test_jittery_localization_still_finishes_at_the_start(self):
        tracker = LapTracker(ROUTE)
        start = self.walker.station_of(CONTEST_404_START)
        drive = Drive(tracker, start)
        rng = np.random.default_rng(7)
        done = False
        for _ in range(6000):
            if drive.step(noise=.3, rng=rng)["done"]:
                done = True
                break
        self.assertTrue(done)
        self.assertFalse(tracker.overshoot)
        self.assertAlmostEqual(drive.station - start, tracker.length, delta=FINISH_WINDOW_M)

    def test_reverse_manoeuvre_does_not_count(self):
        tracker = LapTracker(ROUTE)
        start = 500.
        drive = Drive(tracker, start)
        while drive.status["progress_m"] < 600.:
            drive.step()
        while drive.speed > 0.:                    # stop
            drive.step(target=0.)
        before = drive.status["progress_m"]
        reversed_m = 0.
        while reversed_m < 9.:                     # 9 m reverse escape at ~2 km/h
            drive.speed = .55
            drive.station -= drive.speed * drive.dt
            drive.now += drive.dt
            reversed_m += drive.speed * drive.dt
            drive.status = tracker.update(*self.walker.at(drive.station), drive.speed, drive.now)
        drive.speed = 0.
        self.assertAlmostEqual(drive.status["progress_m"], before - 9., delta=.6)
        self.assert_clean_lap(tracker, drive, start)

    def test_reverse_behind_the_start_finishes_at_the_start_not_before(self):
        tracker = LapTracker(ROUTE)
        start = self.walker.station_of(CONTEST_404_START)
        drive = Drive(tracker, start)
        for _ in range(100):                       # 5.5 m behind the start point
            drive.station -= .055
            drive.now += .1
            drive.status = tracker.update(*self.walker.at(drive.station), .55, drive.now)
        self.assertAlmostEqual(drive.status["progress_m"], -5.5, delta=.3)
        self.assertTrue(drive.run_until_done())
        self.assertAlmostEqual(drive.station - start, tracker.length, delta=FINISH_WINDOW_M)

    def test_back_and_forth_before_half_lap_never_arms(self):
        tracker = LapTracker(ROUTE)
        drive = Drive(tracker, 200.)
        while drive.status["progress_m"] < ARM_FRACTION * tracker.length - 20.:
            drive.step()
        for _ in range(20):                        # 20 x (forward 10 m, back 10 m)
            for direction in (1., -1.):
                for _ in range(10):
                    drive.station += direction
                    drive.now += .2
                    drive.status = tracker.update(*self.walker.at(drive.station), 5., drive.now)
        self.assertFalse(drive.status["armed"])
        self.assertIsNone(drive.status["cap_mps"])

    def test_stop_mid_route_is_not_a_lap(self):
        tracker = LapTracker(ROUTE)
        drive = Drive(tracker, 1500.)
        while drive.status["progress_m"] < .8 * tracker.length:
            drive.step()
        for _ in range(300):                       # 30 s wait at a stop line / for traffic
            drive.step(target=0.)
        self.assertTrue(drive.status["armed"])
        self.assertFalse(drive.status["done"])

    def test_teleport_restarts_the_lap(self):
        tracker = LapTracker(ROUTE)
        drive = Drive(tracker, 300.)
        while drive.status["progress_m"] < 1300.:
            drive.step()
        self.assertTrue(drive.status["armed"])
        drive.station = 300.                       # MORAI scenario reset to the start pose
        drive.speed = 0.
        drive.now += .1
        drive.status = tracker.update(*self.walker.at(drive.station), 0., drive.now)
        self.assertEqual(drive.status["event"], "restart")
        self.assertEqual(drive.status["progress_m"], 0.)
        self.assertFalse(drive.status["armed"])
        self.assertIsNone(drive.status["cap_mps"])
        self.assertTrue(drive.run_until_done())
        self.assertAlmostEqual(drive.station - 300., tracker.length, delta=FINISH_WINDOW_M)

    def test_localization_jump_after_the_tunnel_is_counted(self):
        tracker = LapTracker(ROUTE)
        drive = Drive(tracker, 1800.)
        while drive.status["progress_m"] < 100.:
            drive.step()
        before = drive.status["progress_m"]
        drive.station += 12.                       # GPS back: 12 m correction in one fix
        drive.now += .066
        drive.status = tracker.update(*self.walker.at(drive.station), drive.speed, drive.now)
        self.assertNotEqual(drive.status["event"], "restart")
        self.assertAlmostEqual(drive.status["progress_m"], before + 12., delta=.3)

    def test_fixes_far_off_the_route_are_ignored(self):
        tracker = LapTracker(ROUTE)
        drive = Drive(tracker, 700.)
        for _ in range(50):
            drive.step()
        before = drive.status["progress_m"]
        far = self.walker.at(drive.station, lateral=40.)
        status = tracker.update(far[0], far[1], 5., drive.now + .1)
        self.assertEqual(status["progress_m"], before)
        self.assertEqual(status["off_route_fixes"], 1)
        # A bypass 4 m off the centreline still counts.
        side = self.walker.at(drive.station + 1., lateral=4.)
        status = tracker.update(side[0], side[1], 5., drive.now + .2)
        self.assertAlmostEqual(status["progress_m"], before + 1., delta=.3)

    def test_overshoot_ends_the_lap(self):
        tracker = LapTracker(ROUTE)
        start = 900.
        walker = self.walker
        now, station = 1000., start
        tracker.update(*walker.at(station), 0., now)
        status = None
        while station - start < tracker.length + OVERSHOOT_M + 5.:   # a car ignoring the cap
            station += 1.4
            now += .1
            status = tracker.update(*walker.at(station), 14., now)
            if status["done"]:
                break
        self.assertTrue(status["done"])
        self.assertTrue(tracker.overshoot)
        self.assertGreater(station - start, tracker.length + OVERSHOOT_M)

    def test_cap_reaches_zero_at_the_start_station(self):
        tracker = LapTracker(ROUTE)
        tracker.update(*self.walker.at(0.), 0., 0.)
        for k, s in enumerate(np.arange(1., tracker.length - 50., 2.)):
            tracker.update(*self.walker.at(s), 10., 1. + .2 * k)
        self.assertTrue(tracker.armed)
        cap = tracker.cap_mps(0.)
        self.assertAlmostEqual(cap, math.sqrt(2. * lap_finish.LAP_DECEL_MPS2 * (tracker.length - tracker.progress)), places=6)
        self.assertLess(tracker.cap_mps(10.), cap)   # reaction reserve at speed

    def test_invalid_fixes_change_nothing(self):
        tracker = LapTracker(ROUTE)
        status = tracker.update(float("nan"), 0., 0., 0.)
        self.assertEqual(status["state"], "WAIT_START")
        status = tracker.update("x", 0., 0., 0.)
        self.assertEqual(status["state"], "WAIT_START")


class LapEnvironmentTests(unittest.TestCase):
    def test_disabled_unless_requested(self):
        self.assertIsNone(lap_tracker_from_env(ROUTE, {}))
        self.assertIsNone(lap_tracker_from_env(ROUTE, {"STUDENT_ONE_LAP": "0"}))
        self.assertIsInstance(lap_tracker_from_env(ROUTE, {"STUDENT_ONE_LAP": "1"}), LapTracker)
        with self.assertRaises(ValueError):
            lap_tracker_from_env(ROUTE, {"STUDENT_ONE_LAP": "1", "STUDENT_LAP_DECEL_MPS2": "9"})

    def test_marker_path_and_atomic_write(self):
        self.assertIsNone(lap_marker_path({}))
        self.assertEqual(lap_marker_path({"STUDENT_LOG_DIR": "/tmp/run"}), Path("/tmp/run/lap_complete.json"))
        self.assertEqual(lap_marker_path({"STUDENT_LOG_DIR": "/tmp/run", "STUDENT_LAP_MARKER": "/x/m.json"}),
                         Path("/x/m.json"))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "run" / "lap_complete.json"
            write_lap_marker(path, dict(lap_complete=True))
            self.assertEqual(json.loads(path.read_text()), dict(lap_complete=True))
            self.assertEqual(sorted(p.name for p in path.parent.iterdir()), ["lap_complete.json"])


class PublisherLapCapTests(unittest.TestCase):
    """The lap cap limits every published target speed, including the safety recheck."""

    def setUp(self):
        import test_avoidance_gate as avoidance_fixture
        runtime = avoidance_fixture.AvoidanceGateTests().runtime()
        self.publisher = runtime.path_publisher
        self.route = runtime.test_route
        self.ego = dict(runtime.test_status["ego"])

    def publish(self, clock, cap_kmh, mode="HIGHWAY"):
        self.ego["speed_kmh"] = 40.
        self.publisher.zone_mps = cap_kmh / 3.6
        self.publisher.emergency_stop = False
        postprocess = dict(mode=mode, reason="test", validated_static=True,
                           highway=dict(speed_plan=dict(reference_checked=True)))
        with patch("gui.time.monotonic", return_value=clock):
            self.publisher.publish(self.ego, self.route, self.route, postprocess)
        return self.publisher.velocity_pub.items[-1].data

    def test_no_lap_cap_leaves_the_target(self):
        self.assertAlmostEqual(self.publish(100., 60.), 60. / 3.6)

    def test_lap_cap_limits_the_published_target(self):
        self.publisher.lap_cap_mps = 4.
        self.assertAlmostEqual(self.publish(100., 60.), 4.)
        payload = json.loads(self.publisher.waypoints_pub.items[-1].data)
        self.assertAlmostEqual(payload["target_speed_mps"], 4.)
        self.publisher.lap_cap_mps = 0.
        self.assertEqual(self.publish(100.1, 60.), 0.)

    def test_lap_cap_above_the_target_changes_nothing(self):
        self.publisher.lap_cap_mps = 50.
        self.assertAlmostEqual(self.publish(100., 60.), 60. / 3.6)


class LiveRecheckLapCapTests(unittest.TestCase):
    def test_safety_recheck_never_raises_above_the_lap_cap(self):
        from test_live_path_safety import LivePathSafetyTests
        fixture = LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        runtime.infer()
        first = fixture.payload(runtime)["target_speed_mps"]
        self.assertGreater(first, 1.)
        runtime.path_publisher.lap_cap_mps = .8
        fixture.advance(runtime, .05, [])
        runtime.path_publisher.recheck_active_path()
        self.assertLessEqual(fixture.payload(runtime)["target_speed_mps"], .8 + 1e-9)
        self.assertLessEqual(runtime.path_publisher.velocity_pub.items[-1].data, .8 + 1e-9)


class RuntimeLapTests(unittest.TestCase):
    """gui.Runtime.update_lap: cap hand-off and the lap_complete.json marker."""

    def runtime(self, tracker):
        import test_avoidance_gate as avoidance_fixture
        runtime = avoidance_fixture.AvoidanceGateTests().runtime()
        runtime.lap_tracker = tracker
        return runtime

    def test_update_lap_hands_the_cap_and_writes_the_marker_once(self):
        tracker = LapTracker(ROUTE, hold_s=.3)
        runtime = self.runtime(tracker)
        walker = RouteWalker(ROUTE)
        start = walker.station_of(CONTEST_404_START)
        clock = [5000.]

        def feed(station, speed):
            x, y = walker.at(station)
            runtime.input_error = None
            clock[0] += .1
            with patch("gui.time.monotonic", return_value=clock[0]):
                return runtime.update_lap(dict(ego=dict(x=x, y=y, speed_kmh=speed * 3.6)))

        with tempfile.TemporaryDirectory() as tmp, \
                patch.dict(os.environ, {"STUDENT_LAP_MARKER": str(Path(tmp) / "lap_complete.json"),
                                        "STUDENT_LOG_DIR": str(Path(tmp))}):
            marker = Path(tmp) / "lap_complete.json"
            self.assertEqual(feed(start, 0.)["event"], "start")
            self.assertIsNone(runtime.path_publisher.lap_cap_mps)
            station = start
            while station < start + tracker.length - 60.:
                station += 1.
                lap = feed(station, 10.)
            self.assertTrue(lap["armed"])
            self.assertLess(runtime.path_publisher.lap_cap_mps, 15.)
            self.assertFalse(marker.exists())
            while station < start + tracker.length - .5:
                station += .5
                feed(station, 3.)
            for _ in range(5):
                lap = feed(station, 0.)
            self.assertTrue(tracker.done)
            self.assertEqual(runtime.path_publisher.lap_cap_mps, 0.)
            record = json.loads(marker.read_text())
            self.assertTrue(record["lap_complete"])
            self.assertLess(record["stop"]["distance_from_start_m"], 1.)
            stamp = marker.stat().st_mtime_ns
            self.assertIsNone(feed(station + 3., 0.))        # done: nothing more, cap stays zero
            self.assertEqual(runtime.path_publisher.lap_cap_mps, 0.)
            self.assertEqual(marker.stat().st_mtime_ns, stamp)

    def test_stale_pose_keeps_the_last_cap(self):
        runtime = self.runtime(LapTracker(ROUTE))
        runtime.path_publisher.lap_cap_mps = 3.
        runtime.input_error = "MORAI ROS/Ego disconnected - stale coordinates blocked"
        self.assertIsNone(runtime.update_lap(dict(ego=dict(x=0., y=0., speed_kmh=0.))))
        self.assertEqual(runtime.path_publisher.lap_cap_mps, 3.)

    def test_without_a_tracker_nothing_changes(self):
        runtime = self.runtime(None)
        self.assertIsNone(runtime.update_lap(dict(ego=dict(x=0., y=0., speed_kmh=0.))))
        self.assertIsNone(getattr(runtime.path_publisher, "lap_cap_mps", None))


if __name__ == "__main__":
    unittest.main()
