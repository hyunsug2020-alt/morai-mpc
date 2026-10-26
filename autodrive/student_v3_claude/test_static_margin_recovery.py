"""Replay the captured 2026-10-16 road stop with real inference/publication."""
import json
import math
from pathlib import Path
from unittest.mock import patch
import unittest

import numpy as np

import gui
import model_avoidance as ma
from ego_body import REAR_AXLE_SHIFT_M, body_gap
from tunnel_runtime import swept_clearance
import test_road_forward_progress as recorded


class CapturedMarginStopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        recorded.RecordedRoadForwardProgress.setUpClass()
        with np.load(Path(__file__).parent/'test_data/road_static_margin_20261016.npz') as data:
            cls.frame = {key: data[key].copy() for key in data.files}
        cls.initial = json.loads(str(cls.frame['ego']))
        cls.objects = json.loads(str(cls.frame['objects']))

    def fixture(self):
        f = recorded.RecordedRoadForwardProgress()
        f.initial, f.objects = dict(self.initial), list(self.objects)
        f.point_world = self.frame['points'] @ recorded.rotation(self.initial['heading_deg']).T + [
            self.initial['x'], self.initial['y']]
        return f

    def test_the_actual_stopped_pose_can_leave_without_reverse_or_stop_pulses(self):
        f = self.fixture()
        runtime = f.runtime()
        ego, now, speed, travelled = dict(self.initial), 1_000_000., 0., 0.
        with patch('time.monotonic', side_effect=lambda: now), patch('time.time', side_effect=lambda: now), \
                patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            for sequence in range(1, 150):
                points = f.observe(runtime, ego, now, sequence)
                self.assertTrue(runtime.infer())
                payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
                self.assertGreater(payload['target_speed_mps'], .1, runtime._web_record)
                self.assertFalse(payload['decision']['emergency'], payload['decision'])
                self.assertNotEqual(payload.get('gear'), 'R')
                path_world = np.array([[p['x'], p['y']] for p in payload['waypoints']])
                path = (path_world-[ego['x'], ego['y']]) @ recorded.rotation(ego['heading_deg'])
                required = max(4., speed*.7+speed*speed/5.+2.)
                self.assertGreaterEqual(swept_clearance(path, points, required, .25,
                    lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M), required-.01)
                self.assertGreater(body_gap(points), .30)
                look = max(3., .45*speed+2.)
                ahead = np.flatnonzero((path[:, 0] > 0.) & (np.linalg.norm(path, axis=1) >= look))
                q = path[ahead[0]]
                curvature = float(2.*q[1]/(q@q))
                speed = max(0., speed+float(np.clip((payload['target_speed_mps']-speed)/.25, -3.5, 2.1))*.1)
                yaw = math.radians(ego['heading_deg'])
                ego['x'] += speed*math.cos(yaw)*.1
                ego['y'] += speed*math.sin(yaw)*.1
                ego['heading_deg'] += math.degrees(speed*curvature*.1)
                ego['speed_kmh'] = speed*3.6
                travelled += speed*.1
                now += .1
                if travelled > 20.:
                    break
        self.assertGreater(travelled, 20.)

    def test_recovery_never_authorizes_an_actual_new_front_collision(self):
        f, now = self.fixture(), 1_000_000.
        runtime = f.runtime()
        blocker = np.array([[3.9, 0.]]) @ recorded.rotation(self.initial['heading_deg']).T + [
            self.initial['x'], self.initial['y']]
        with patch('time.monotonic', return_value=now), patch('time.time', return_value=now), \
                patch.object(gui, 'MODEL_AVOID', True), patch.object(gui, 'USE_LATTICE', False), \
                patch.object(gui, 'model_avoidance', ma, create=True):
            f.observe(runtime, self.initial, now, 1, point_world=np.vstack((f.point_world, blocker)))
            self.assertTrue(runtime.infer())
        payload = json.loads(runtime.path_publisher.waypoints_pub.items[-1].data)
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertNotEqual(payload.get('gear'), 'R')

    def test_padding_recovery_allows_leaving_but_never_approaching_the_same_box(self):
        static = ma.road_static_objects(self.objects, self.initial, self.frame['points'])
        path = np.column_stack((np.arange(0., 60.01, .25), np.zeros(241)))
        self.assertGreaterEqual(gui.static_clear_distance(path, self.initial, static, 30.,
            body_shift_m=REAR_AXLE_SHIFT_M), 29.9)
        toward = path.copy()
        u = np.clip(toward[:, 0]/3., 0., 1.)
        toward[:, 1] = -.15*u**3*(10.-15.*u+6.*u*u)
        self.assertLess(gui.static_clear_distance(toward, self.initial, static, 30.,
            body_shift_m=REAR_AXLE_SHIFT_M), 3.)


if __name__ == '__main__':
    unittest.main()
