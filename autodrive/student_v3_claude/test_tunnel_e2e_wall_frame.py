"""Tunnel planning stays in the measured wall frame while the ESKF pose and the wall
map disagree (E2E mode).

With the 2026-10-02 wall map 0.544 deg off the world, the measured correction grew
~1 m per 100 m driven (E2E runs
follow-20261009T185440Z / 191227Z: 0.18 -> 1.24 m, then "pose correction exceeds
validated limit" > 1.2 m and "both supported tunnel walls required" once the walls
left the +-1.4 m fit window centred on the raw ESKF pose, then the degraded fallback
and LivePathSafety stops). STUDENT_TUNNEL_POLICY=model turns on TUNNEL_RELATIVE_FRAME.
No ROS, simulator or GPU needed.
"""
import json
import math
import os
import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from tunnel_map import TunnelMap  # noqa: E402

RECORDED = ('tunnel_pose_rejected_20261008.npz', 'tunnel_semantic_stop_20261008.npz',
            'tunnel_wide_pass_entry_20261008.npz')
SEARCH = (1.25, -1.25, 2.5, -2.5)
# Synthetic tunnel with the surveyed asymmetric offsets and extent, axis along +x.
PAYLOAD = dict(format='morai_fixed_tunnel_walls_v1', origin_xy=[0., 0.], axis_heading_deg=0.,
               left_offset_m=8.37, right_offset_m=-3.56, longitudinal_min_m=2.5, longitudinal_max_m=117.65)
PLANE = [0., 0., -.4]
BIAS_DEG = .55   # ESKF heading minus the walls' heading (logged yaw correction 0.48-0.60 deg)


def synthetic_scan(along, lateral, heading_deg, walls=True, kerbs=True):
    """Vehicle-frame returns of both walls (and kerbs) seen from a true map pose."""
    world = []
    if walls:
        s = np.arange(max(PAYLOAD['longitudinal_min_m'], along-40.),
                      min(PAYLOAD['longitudinal_max_m'], along+60.), .25)
        for wall, side, kerb in ((8.37, 1, .75), (-3.56, -1, .65)):
            for h in np.arange(.5, 3.01, .25):
                world.append(np.column_stack((s, np.full(len(s), wall), np.full(len(s), h))))
            for h in ((.5, .7, .9, 1.05) if kerbs else ()):
                world.append(np.column_stack((s, np.full(len(s), wall-side*kerb), np.full(len(s), h))))
    angle = np.radians(np.arange(0., 360., .5))
    world.append(np.column_stack((along+7.*np.cos(angle), lateral+7.*np.sin(angle), np.zeros(len(angle)))))
    world = np.vstack(world)
    t = math.radians(heading_deg)
    d = world[:, :2]-[along, lateral]
    body = np.column_stack((math.cos(t)*d[:, 0]+math.sin(t)*d[:, 1], -math.sin(t)*d[:, 0]+math.cos(t)*d[:, 1]))
    return np.column_stack((body, world[:, 2]+PLANE[2]))


def wall_frame_lateral(model, pose):
    return float((np.array([pose['x'], pose['y']])-model.origin) @ model.normal)


def usable(model, xyz, ego, plane, **window):
    """process() and the uncapped relative projection, as the relative-frame runtime uses them."""
    match = model.process(xyz, ego, plane, **window)
    if match['valid']:
        return match, match
    return match, model.relative_projection(xyz, ego, plane, match, max_correction_m=None)


class RecordedScansUnderDrift(unittest.TestCase):
    """Real tunnel scans, wall map and background survey with extra ESKF lateral error."""
    @classmethod
    def setUpClass(cls):
        from lidar_corridor import LidarCorridor
        from tunnel_runtime import pose_at_stamp
        # The recorded 2026-10-08 disagreement: these scans against the map then live.
        cls.model = TunnelMap.load(HERE/'tunnel_wall_map.json', survey_frame=True)
        cls.scans = {}
        for name in RECORDED:
            data = np.load(HERE/'test_data'/name)
            meta = json.loads(str(data['meta']))
            lidar = LidarCorridor(x_range=(-75., 100.))
            lidar.ingest(data['xyz'], stamp=meta['stamp'])
            snapshot = lidar.scan_snapshot()
            cls.scans[name] = (snapshot['xyz'], snapshot['ground_plane'], pose_at_stamp(meta['poses'], meta['stamp']))

    def drifted(self, ego, metres):
        out = dict(ego)
        out['x'], out['y'] = (np.array([ego['x'], ego['y']])-metres*self.model.normal).tolist()
        return out

    def test_old_window_and_cap_lose_the_walls_once_the_drift_grows(self):
        for name, (xyz, plane, ego) in self.scans.items():
            with self.subTest(name=name):
                match = self.model.process(xyz, self.drifted(ego, 1.5), plane)
                self.assertFalse(match['valid'])
                self.assertEqual(match['reason'], 'both supported tunnel walls required')
                self.assertIsNone(self.model.relative_projection(xyz, self.drifted(ego, 1.5), plane, match))

    def test_prior_or_search_window_keeps_the_same_wall_frame_at_any_drift(self):
        for name, (xyz, plane, ego) in self.scans.items():
            base, reference = usable(self.model, xyz, ego, plane)
            self.assertIsNotNone(reference, name)
            measured = base['wall_stats']['lateral_correction_m']
            lateral = wall_frame_lateral(self.model, reference['aligned_ego'])
            for extra in (.5, 1., 1.5, 2.5):
                for window in (dict(lateral_prior_m=measured+extra-.1), dict(search_m=SEARCH)):
                    with self.subTest(name=name, extra=extra, window=sorted(window)):
                        ego_drifted = self.drifted(ego, extra)
                        match, used = usable(self.model, xyz, ego_drifted, plane, **window)
                        self.assertIsNotNone(used, match['reason'])
                        self.assertTrue(used['valid'])
                        self.assertAlmostEqual(match['wall_stats']['lateral_correction_m'], measured+extra, delta=.05)
                        self.assertAlmostEqual(wall_frame_lateral(self.model, used['aligned_ego']), lateral, delta=.05)
                        self.assertAlmostEqual(used['aligned_ego']['heading_deg'],
                                               reference['aligned_ego']['heading_deg'], delta=.2)
                        if measured+extra > self.model.max_lateral+.05:
                            # Beyond max_lateral this is planning only: never a map-aiding constraint.
                            self.assertIsNone(used['constraint'])
                            self.assertTrue(used['relative_geometry'])
                        elif measured+extra < self.model.max_lateral-.05:
                            self.assertEqual(used['constraint']['source'], 'tunnel_lidar_wall')
                        # Same obstacle extraction in the same wall frame.
                        self.assertLessEqual(abs(len(used['obstacle_points'])-len(reference['obstacle_points'])),
                                             max(3, .05*len(reference['obstacle_points'])))

    def test_map_aiding_limit_is_unchanged_by_the_window(self):
        xyz, plane, ego = self.scans['tunnel_wide_pass_entry_20261008.npz']
        for window in ({}, dict(lateral_prior_m=.3), dict(lateral_prior_m=.3, search_m=SEARCH)):
            match = self.model.process(xyz, self.drifted(ego, .3), plane, **window)
            self.assertTrue(match['valid'], match['reason'])
            self.assertEqual(match['constraint']['source'], 'tunnel_lidar_wall')
            beyond = self.model.process(xyz, self.drifted(ego, 1.), plane, **window)
            self.assertEqual(beyond['reason'], 'pose correction exceeds validated limit')
            self.assertIsNone(beyond['constraint'])


class SyntheticWindowGates(unittest.TestCase):
    def setUp(self):
        self.model = TunnelMap(PAYLOAD)

    def test_defaults_keep_the_raw_window_and_the_relative_cap(self):
        xyz = synthetic_scan(101., .6, -18.)
        eskf = dict(x=101., y=.6-1.6, heading_deg=-18.-BIAS_DEG)
        self.assertEqual(self.model.process(xyz, eskf, PLANE)['reason'], 'both supported tunnel walls required')
        near = self.model.process(xyz, dict(eskf, y=.6-1.3), PLANE)
        self.assertEqual(near['reason'], 'pose correction exceeds validated limit')
        self.assertIsNone(self.model.relative_projection(xyz, dict(eskf, y=.6-1.3), PLANE, near))
        relative = self.model.relative_projection(xyz, dict(eskf, y=.6-1.3), PLANE, near, max_correction_m=None)
        self.assertAlmostEqual(relative['aligned_ego']['y'], .6, delta=.02)

    def test_search_never_pairs_a_kerb_or_a_single_wall(self):
        eskf = dict(x=60., y=1.2, heading_deg=-BIAS_DEG)
        xyz = synthetic_scan(60., 2.8, 0.)
        right_wall = (np.abs(xyz[:, 1]-(-3.56-2.8)) < .01) & (xyz[:, 2] > .3+PLANE[2])
        only_kerb = xyz[~right_wall]          # right wall gone, its kerb face still there
        result = self.model.process(only_kerb, eskf, PLANE, lateral_prior_m=1.6, search_m=SEARCH)
        self.assertFalse(result['valid'])
        self.assertIsNone(self.model.relative_projection(only_kerb, eskf, PLANE, result, max_correction_m=None))
        left = synthetic_scan(60., 2.8, 0.)
        left = left[left[:, 1] > 0.]
        result = self.model.process(left, eskf, PLANE, lateral_prior_m=1.6, search_m=SEARCH)
        self.assertFalse(result['valid'])
        self.assertNotIn('body_walls', result)


class Drive:
    """True car on the bypass side (2.8 m), ESKF pose vs walls diverging at 0.55 deg,
    pure odometry in its own frame (rotated 30 deg, shifted)."""
    ODOMETRY_ROTATION = math.radians(30.)

    def __init__(self, lateral=2.8, initial_error=.3):
        self.lateral, self.initial_error = lateral, initial_error

    def eskf(self, along, heading_deg=0.):
        error = self.initial_error+(along-5.)*math.tan(math.radians(BIAS_DEG))
        return dict(x=along, y=self.lateral-error, heading_deg=heading_deg-BIAS_DEG, speed_kmh=30.)

    def odometry(self, along, lateral=None, heading_deg=0.):
        c, s = math.cos(self.ODOMETRY_ROTATION), math.sin(self.ODOMETRY_ROTATION)
        y = self.lateral if lateral is None else lateral
        return dict(x=100.+c*along-s*y, y=-50.+s*along+c*y,
                    heading_deg=heading_deg+math.degrees(self.ODOMETRY_ROTATION), speed_kmh=30.)


class RelativeFrameRuntime(unittest.TestCase):
    """TunnelRuntime's per-scan matching on a real TunnelMap (method level)."""
    def setUp(self):
        import tunnel_runtime
        from tunnel_runtime import TunnelRuntime
        self.tr = tunnel_runtime
        host = SimpleNamespace(tunnel=TunnelMap(PAYLOAD), _tunnel_wall_anchor=None, _tunnel_lateral_prior=None)
        host.tunnel_wall_prior = lambda: TunnelRuntime.tunnel_wall_prior(host)
        host.tunnel_relative_frame_match = lambda *a: TunnelRuntime.tunnel_relative_frame_match(host, *a)
        host.tunnel_exit_anchor = lambda *a: TunnelRuntime.tunnel_exit_anchor(host, *a)
        self.host, self.drive = host, Drive()
        patcher = patch.multiple(tunnel_runtime, TUNNEL_EXIT_ANCHOR=True, TUNNEL_RELATIVE_FRAME=True)
        patcher.start()
        self.addCleanup(patcher.stop)

    def step(self, stamp, along, xyz, eskf=None, odometry=None):
        host = self.host
        scan = dict(xyz=xyz, ground_plane=PLANE)
        eskf = eskf or self.drive.eskf(along)
        odometry = odometry or self.drive.odometry(along)
        match = host.tunnel.process(xyz, eskf, PLANE, lateral_prior_m=host.tunnel_wall_prior(),
                                    search_m=self.tr.TUNNEL_WALL_SEARCH_M)
        match, relative = host.tunnel_relative_frame_match(match, scan, stamp, eskf, odometry)
        return match if match.get('valid') else relative, match

    def test_whole_tunnel_and_portal_stay_in_the_measured_wall_frame(self):
        stamp, worst = 0., 0.
        for along in np.arange(5., 104.1, 1.5):
            stamp += .1
            used, match = self.step(stamp, along, synthetic_scan(along, 2.8, 0.))
            self.assertIsNotNone(used, f'{along}: {match["reason"]}')
            self.assertAlmostEqual(used['aligned_ego']['y'], 2.8, delta=.03)
            worst = max(worst, match['wall_stats']['lateral_correction_m'])
        self.assertGreater(worst, 1.2)        # past the old relative cap
        self.assertAlmostEqual(self.host.tunnel_wall_prior(), worst, delta=.05)
        # Portal: the walls vanish (fit fails) 13 m before the exit; the last relative
        # match is carried by odometry, net of its 1.25 m correction.
        for i in range(30):                    # 60 s stop at the portal (last match at 104 m)
            stamp += 2.
            used, match = self.step(stamp, 104., synthetic_scan(104., 2.8, 0., walls=False))
            self.assertIsNotNone(used, match['reason'])
            self.assertEqual(used['reason'], 'tunnel exit: last wall match carried by odometry')
            self.assertIsNone(used['constraint'])
            self.assertAlmostEqual(used['aligned_ego']['y'], 2.8, delta=.05)
        self.assertEqual(self.host._tunnel_wall_anchor['moving_s'], 0.)
        for along in np.arange(106., 130.1, 2.):    # drive out of the tunnel on the carried pose
            stamp += .15
            used, match = self.step(stamp, along, synthetic_scan(along, 2.8, 0., walls=False))
            self.assertIsNotNone(used, match['reason'])
            self.assertAlmostEqual(used['aligned_ego']['y'], 2.8, delta=.1)

    def approach_exit(self):
        stamp = 0.
        for along in np.arange(80., 100.1, 1.5):
            stamp += .1
            used, match = self.step(stamp, along, synthetic_scan(along, 2.8, 0.))
            self.assertIsNotNone(used, match['reason'])
        return stamp

    def test_carried_anchor_still_rejects_odometry_disagreement_and_far_occlusions(self):
        stamp = self.approach_exit()
        lost = synthetic_scan(101.5, 2.8, 0., walls=False)
        # Odometry 1 m sideways of where the ESKF moved: not carried (0.25 s rule nor exit).
        jumped = self.drive.odometry(101.5, lateral=2.8+1.)
        used, match = self.step(stamp+.1, 101.5, lost, odometry=jumped)
        self.assertIsNone(used)
        self.assertFalse(match['valid'])
        # Far from the exit nothing is carried beyond the 0.25 s occlusion rule.
        self.host._tunnel_wall_anchor = None
        for along in np.arange(30., 40.1, 1.5):
            stamp += .1
            self.step(stamp, along, synthetic_scan(along, 2.8, 0.))
        used, _ = self.step(stamp+.2, 40.5, synthetic_scan(40.5, 2.8, 0., walls=False))
        self.assertIsNotNone(used)             # brief occlusion: carried, net of the correction
        self.assertEqual(used['reason'], 'recent tunnel wall pose propagated')
        used, _ = self.step(stamp+.5, 41., synthetic_scan(41., 2.8, 0., walls=False))
        self.assertIsNone(used)

    def test_exit_anchor_ages_by_driving_time_and_keeps_old_rules_without_relative_fields(self):
        stamp = self.approach_exit()
        anchor = self.host._tunnel_wall_anchor
        failed = dict(valid=False, reason='both supported tunnel walls required')
        scan = dict(xyz=synthetic_scan(101., 2.8, 0., walls=False), ground_plane=PLANE)
        eskf, odometry = self.drive.eskf(101.), self.drive.odometry(101.)
        exit_anchor = self.host.tunnel_exit_anchor
        # Lattice-mode anchors (no scan_ego / moving_s): raw 0.8 m rule, 1.2 m ESKF offset.
        legacy = dict(stamp=anchor['stamp'], match=anchor['match'], odometry=anchor['odometry'])
        self.assertFalse(exit_anchor(failed, legacy, scan, stamp+.1, eskf, odometry)['valid'])
        self.assertTrue(exit_anchor(failed, anchor, scan, stamp+.1, eskf, odometry)['valid'])
        # Without moving_s the wall-clock 20 s rule applies; with it only driving time counts.
        aged = {key: value for key, value in anchor.items() if key != 'moving_s'}
        self.assertFalse(exit_anchor(failed, aged, scan, stamp+25., eskf, odometry)['valid'])
        self.assertTrue(exit_anchor(failed, dict(anchor, moving_s=0.), scan, stamp+25., eskf, odometry)['valid'])
        self.assertFalse(exit_anchor(failed, dict(anchor, moving_s=21.), scan, stamp+25., eskf, odometry)['valid'])
        # More than 55 m of odometry since the anchor: not carried.
        far_eskf, far_odometry = self.drive.eskf(156.), self.drive.odometry(156.)
        self.assertFalse(exit_anchor(failed, anchor, scan, stamp+5., far_eskf, far_odometry)['valid'])


class InferTunnelUnderDrift(unittest.TestCase):
    """Full infer() on a recorded tunnel scan whose ESKF pose carries 1.0 m more drift."""
    def recorded(self, metres):
        from test_tunnel_integration import TunnelIntegrationTests
        fixture = TunnelIntegrationTests()
        runtime = fixture.recorded_runtime('tunnel_semantic_stop_20261008.npz')
        normal = runtime.tunnel.normal
        shift = lambda pose: None if pose is None else dict(
            pose, x=float(pose['x']-metres*normal[0]), y=float(pose['y']-metres*normal[1]))
        original = runtime.ego_receiver.at_stamp
        runtime.ego_receiver = SimpleNamespace(at_stamp=lambda stamp: shift(original(stamp)))
        runtime.test_status = dict(runtime.test_status, ego=shift(runtime.test_status['ego']))
        return fixture, runtime

    def test_default_mode_still_falls_back_and_relative_frame_keeps_planning(self):
        import tunnel_runtime
        fixture, runtime = self.recorded(1.)
        with patch('tunnel_runtime.time.monotonic', return_value=runtime.lidar.received):
            fixture.assert_fallback_drive(runtime)
        self.assertIn('both supported tunnel walls required', runtime._web_record['reason'])

        fixture, runtime = self.recorded(1.)
        with patch.multiple(tunnel_runtime, TUNNEL_RELATIVE_FRAME=True, TUNNEL_EXIT_ANCHOR=True), \
                patch('tunnel_runtime.time.monotonic', return_value=runtime.lidar.received):
            fixture.assert_positive_target(runtime)
        row = runtime._web_record
        self.assertTrue(row['tunnel']['valid'], row['reason'])
        self.assertTrue(row['tunnel']['relative_geometry'])
        self.assertNotIn('fallback', row['reason'])
        self.assertGreater(row['tunnel']['wall_stats']['lateral_correction_m'], 1.9)
        runtime.path_publisher.publish_tunnel_constraint.assert_not_called()
        self.assertFalse(row['emergency_stop'])
        self.assertGreaterEqual(row['tunnel']['geometry_clear_distance_m'], 30.)


class ModeFlag(unittest.TestCase):
    def test_relative_frame_is_on_in_model_mode_only(self):
        code = "import tunnel_runtime as t; print(t.TUNNEL_RELATIVE_FRAME, t.TUNNEL_EXIT_ANCHOR)"
        out = {}
        for policy in ("model", "lattice"):
            env = dict(os.environ, STUDENT_TUNNEL_POLICY=policy)
            env.pop("STUDENT_TUNNEL_RELATIVE_FRAME", None)
            env.pop("STUDENT_TUNNEL_EXIT_ANCHOR", None)
            result = subprocess.run([sys.executable, "-c", code], cwd=str(HERE), capture_output=True,
                                    text=True, env=env, timeout=240)
            self.assertEqual(result.returncode, 0, result.stderr[-2000:])
            out[policy] = result.stdout.strip().splitlines()[-1]
        self.assertEqual(out, dict(model="True True", lattice="False False"))


if __name__ == "__main__":
    unittest.main()
