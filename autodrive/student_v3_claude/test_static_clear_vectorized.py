"""Compare vectorized SAT with the original measured static-clearance implementation."""
import math
import time
import unittest
import numpy as np
from gui import local_xy, static_clear_distance
from ego_body import REAR_AXLE_SHIFT_M, ROAD_LEGACY_SHIFT_M


def reference_static_clear_distance(path, ego, objects, horizon_m=35.0, body_shift_m=ROAD_LEGACY_SHIFT_M):
    """Return the first overlapping vehicle pose distance, or the checked path length."""
    route = np.asarray(path, dtype=float)[:, :2]
    if len(route) < 2:
        return 0.0
    if np.linalg.norm(route[0]) > 0.01:
        route = np.vstack(([0.0, 0.0], route))
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    samples = np.arange(0.0, min(horizon_m, arc[-1]) + 0.01, 0.25)
    centres = np.column_stack([np.interp(samples, arc, route[:, i]) for i in (0, 1)])
    if len(centres) < 2:
        return 0.0
    tang = np.gradient(centres, axis=0)
    tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-6)
    tang[0] = (1.0, 0.0)
    normal = np.column_stack((-tang[:, 1], tang[:, 0]))
    clearance = min(float(horizon_m), float(arc[-1]))
    forward_prefix = centres[:max(2, min(len(centres), int(5.0 / 0.25) + 1)), 0]
    centres = centres + body_shift_m * tang   # box centres ahead of the samples (ego_body)
    for obj in objects:
        position = np.asarray(local_xy(obj[0], obj[1], ego))
        yaw = math.radians((obj[8] if len(obj) > 8 else ego["heading_deg"]) - ego["heading_deg"])
        axis = np.array([math.cos(yaw), math.sin(yaw)])
        cross = np.array([-axis[1], axis[0]])
        length = max(0.3, float(obj[6] if len(obj) > 6 else obj[2])) / 2
        width = max(0.3, float(obj[7] if len(obj) > 7 else obj[2])) / 2
        # An object already behind the ego origin may overlap the car's rear
        # footprint at s=0. It recedes on a forward path and must not invalidate
        # every new bypass (run 36: bollard behind ego blocked both free lanes).
        rear_face = position[0] + length * abs(axis[0]) + width * abs(cross[0])
        if rear_face < -0.25 and np.all(np.diff(forward_prefix) >= -0.05):
            future = centres[samples >= 5.0]
            radius = math.hypot(2.3175, 0.946) + math.hypot(length, width) + 0.2
            if not len(future) or np.linalg.norm(future - position, axis=1).min() > radius:
                continue
        delta = position - centres
        overlap = np.ones(len(centres), dtype=bool)
        for a in (tang, normal, axis, cross):
            projected = np.abs(np.sum(delta * a, axis=1))
            # Keep the same lateral pedestrian margin as moving_conflict.
            # Otherwise a rejoin path is accepted here and the yield rule
            # stops the car beside a standing person with no way to replan.
            margin = 0.65 if obj[4] == "pedestrian" else 0.2
            radius = (2.3175 * np.abs(np.sum(tang * a, axis=1)) +
                      0.946 * np.abs(np.sum(normal * a, axis=1)) +
                      length * np.abs(np.sum(axis * a, axis=-1)) +
                      width * np.abs(np.sum(cross * a, axis=-1)) + margin)
            overlap &= projected <= radius
        if np.any(overlap):
            clearance = min(clearance, float(samples[np.flatnonzero(overlap)[0]]))
    return clearance


class StaticClearVectorizedTests(unittest.TestCase):
    @staticmethod
    def objects(local_rows, ego):
        yaw = math.radians(ego['heading_deg'])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        objects = []
        for x, y, length, width, heading, kind in local_rows:
            world = rotation @ [x, y] + [ego['x'], ego['y']]
            objects.append((world[0], world[1], max(length, width), 'measured',
                            kind, 0., length, width, heading + ego['heading_deg']))
        return objects

    def assert_equivalent(self, path, ego, objects, horizon=35., shift=ROAD_LEGACY_SHIFT_M):
        with np.errstate(invalid='ignore', divide='ignore', over='ignore'):
            try:
                expected = reference_static_clear_distance(path, ego, objects, horizon, body_shift_m=shift)
            except Exception as error:
                with self.assertRaises(type(error)):
                    static_clear_distance(path, ego, objects, horizon, body_shift_m=shift)
            else:
                actual = static_clear_distance(path, ego, objects, horizon, body_shift_m=shift)
                if math.isnan(expected):
                    self.assertTrue(math.isnan(actual))
                else:
                    self.assertEqual(actual, expected)

    def test_seeded_rotated_curved_and_short_paths_match_original(self):
        rng = np.random.RandomState(20261003)
        for case in range(240):
            ego = dict(x=rng.uniform(-700., 700.), y=rng.uniform(-700., 700.),
                       heading_deg=rng.uniform(-180., 180.))
            count = int(rng.randint(2, 226))
            x = np.cumsum(rng.uniform(.02, .55, count))
            path = np.column_stack((x, rng.uniform(-4., 4.) * np.sin(x/rng.uniform(5., 18.))))
            if case % 3 == 0:
                path = np.column_stack((path, np.zeros(count)))
            rows = [(rng.uniform(-8., 70.), rng.uniform(-12., 12.),
                     rng.uniform(.05, 8.), rng.uniform(.05, 4.),
                     rng.uniform(-180., 180.), rng.choice(['obstacle', 'npc', 'pedestrian']))
                    for _ in range(int(rng.randint(0, 66)))]
            with self.subTest(case=case):
                self.assert_equivalent(path, ego, self.objects(rows, ego), rng.uniform(.1, 60.))
                # The real body on the rear-axle pose (tunnel, ego_body).
                self.assert_equivalent(path, ego, self.objects(rows, ego), rng.uniform(.1, 60.),
                                       shift=REAR_AXLE_SHIFT_M)

    def test_rear_object_skip_and_future_reentry_are_preserved(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        objects = self.objects([(-2., 0., .5, .5, 0., 'obstacle')], ego)
        straight = np.column_stack((np.arange(0., 41., .5), np.zeros(82)))
        self.assert_equivalent(straight, ego, objects)
        self.assertEqual(static_clear_distance(straight, ego, objects), 35.)
        returning = np.column_stack((np.r_[np.arange(0., 10.5, .5), np.arange(10., -5.5, -.5)],
                                      np.zeros(52)))
        self.assert_equivalent(returning, ego, objects)
        self.assertEqual(static_clear_distance(returning, ego, objects), 0.)
        short = straight[:8]
        self.assert_equivalent(short, ego, objects)

    def test_real_body_rear_object_and_reentry(self):
        # The pose is the rear axle: 2 m behind it lies 1.2 m behind the real rear bumper.
        ego = dict(x=0., y=0., heading_deg=0.)
        objects = self.objects([(-2., 0., .5, .5, 0., 'obstacle')], ego)
        straight = np.column_stack((np.arange(0., 41., .5), np.zeros(82)))
        self.assertEqual(static_clear_distance(straight, ego, objects, body_shift_m=REAR_AXLE_SHIFT_M), 35.)
        returning = np.column_stack((np.r_[np.arange(0., 10.5, .5), np.arange(10., -5.5, -.5)],
                                      np.zeros(52)))
        # Driven along the returning path the real front reaches it at 17.75 m (x = 2.25).
        self.assertEqual(static_clear_distance(returning, ego, objects, body_shift_m=REAR_AXLE_SHIFT_M), 17.75)
        self.assert_equivalent(returning, ego, objects, shift=REAR_AXLE_SHIFT_M)
        # Beside the real body (it reaches 3.845 m ahead of the pose) it blocks at once.
        beside = self.objects([(3.2, 1.2, .4, .4, 0., 'obstacle')], ego)
        self.assertEqual(static_clear_distance(straight, ego, beside, body_shift_m=REAR_AXLE_SHIFT_M), 0.)

    def test_stationary_and_pedestrian_margins_at_boundaries_match(self):
        ego = dict(x=12., y=-31., heading_deg=117.)
        path = np.column_stack((np.arange(0., 56.25, .25), np.zeros(225)))
        for kind, margin in (('obstacle', .2), ('npc', .2), ('pedestrian', .65)):
            for angle in (0., 30., 90., 180.):
                for epsilon in (-1e-8, 0., 1e-8):
                    objects = self.objects([(8., .946+.4+margin+epsilon,
                                              1.2, .8, angle, kind)], ego)
                    self.assert_equivalent(path, ego, objects, 50.)

    def test_empty_legacy_fields_and_first_tangent_match(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        for path in (np.empty((0, 2)), np.array([[0., 0.]]),
                     np.array([[.5, 1.], [2., 3.], [4., 4.]])):
            self.assert_equivalent(path, ego, [])
            self.assert_equivalent(path, ego, [(2., 0., .4, 'legacy', 'obstacle', 0.)])

    def test_nonfinite_values_preserve_existing_result_or_exception(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        path = np.column_stack((np.arange(0., 36., .5), np.zeros(72)))
        base = list(self.objects([(8., 0., 1., 1., 0., 'obstacle')], ego)[0])
        for index in (0, 1, 2, 6, 7, 8):
            for value in (float('nan'), float('inf'), float('-inf')):
                invalid = base.copy()
                invalid[index] = value
                with self.subTest(index=index, value=value):
                    self.assert_equivalent(path, ego, [tuple(invalid)])
        invalid_path = path.copy()
        invalid_path[4, 1] = float('nan')
        self.assert_equivalent(invalid_path, ego, [tuple(base)])
        self.assert_equivalent(path, dict(ego, heading_deg=float('nan')), [tuple(base)])

    def test_recorded_tunnel_geometry_matches_original(self):
        import json
        from pathlib import Path
        fixture = Path('/tmp/tunnel_new_stop_fixture_121.npz')
        if not fixture.exists():
            self.skipTest('Recorded near-object fixture unavailable')
        with np.load(fixture) as data:
            ego = json.loads(str(data['ego']))
            objects = json.loads(str(data['objects']))
            previous = json.loads(str(data['previous']))
            for path in (data['dense'], np.asarray(previous['path'])):
                self.assert_equivalent(path, ego, objects, 60.)


def benchmark(iterations=80):
    rng = np.random.RandomState(121)
    ego = dict(x=31., y=-7., heading_deg=-132.)
    path = np.column_stack((np.arange(0., 56.25, .25), 3.*np.sin(np.arange(225)/45.)))
    rows = [(rng.uniform(-10., 70.), rng.uniform(-12., 12.), rng.uniform(.3, 5.),
             rng.uniform(.3, 2.5), rng.uniform(-180., 180.), 'obstacle') for _ in range(50)]
    objects = StaticClearVectorizedTests.objects(rows, ego)
    results = {}
    for name, function in (('original', reference_static_clear_distance), ('vectorized', static_clear_distance)):
        for _ in range(5):
            function(path, ego, objects, 55.)
        elapsed = []
        for _ in range(iterations):
            started = time.perf_counter()
            function(path, ego, objects, 55.)
            elapsed.append((time.perf_counter()-started)*1000.)
        results[name+'_median_ms'] = float(np.median(elapsed))
        results[name+'_p95_ms'] = float(np.percentile(elapsed, 95))
    results['speedup'] = results['original_median_ms']/results['vectorized_median_ms']
    results['objects'], results['path_points'] = len(objects), len(path)
    return results


if __name__ == '__main__':
    unittest.main()
