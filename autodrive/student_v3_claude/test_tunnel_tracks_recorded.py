"""Recorded static tunnel bodies stay static; measured movers still yield (no ROS, no simulator)."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from motion_filter import measured_velocity
from tunnel_runtime import lidar_objects
from tunnel_tracks import (DETECTOR_BOX_DRIFT_MPS, RADIAL_MOTION_RANGE_M, TunnelTracks,
                           measured_motion_supports, measured_vehicle)

# TunnelTracks inputs replayed from tunnel_scans_full.npz captures through the
# Runtime: (stamp, rect x, y, length, width, heading, voxels, motion centre x, y,
# ego-axis extent x, y, observer x, y, heading), uncorrected odometry frame.
# follow-20261003T032751Z, 8.8-10.5 s: scenario car 40100008 (UNIQUEID 31), the
# one that stopped follow-20261009T204445Z at 296.8-300.6 s, passed at 30 km/h.
# The pre-fix rule read 1.16 m/s and published "tunnel moving object conflict".
PARKED_CAR_PASS = (
    (1790998395.8879, 426.668, 46.444, 0.87, 1.97, 128.02, 10, 426.566, 46.632, 0.8, 1.6, 444.373, 30.127, 148.57),
    (1790998395.9985, 426.722, 46.519, 0.78, 1.94, 134.47, 12, 426.603, 46.640, 0.6, 1.6, 443.794, 30.481, 148.51),
    (1790998396.2197, 426.772, 46.599, 0.60, 1.86, 141.22, 11, 426.704, 46.654, 0.4, 1.6, 442.572, 31.232, 148.35),
    (1790998396.5538, 425.133, 47.623, 0.30, 0.40, 148.09, 2, 425.133, 47.623, 0.0, 0.2, 440.474, 32.532, 148.09),
    (1790998396.6664, 425.228, 47.597, 3.80, 2.00, 148.01, 24, 425.228, 47.597, 3.6, 1.8, 439.700, 33.015, 148.01),
    (1790998396.7783, 425.044, 46.866, 4.31, 1.73, 171.88, 23, 425.182, 47.544, 3.8, 1.6, 438.922, 33.502, 147.92),
    (1790998396.8895, 424.968, 47.110, 4.36, 1.77, 166.26, 28, 425.180, 47.579, 4.0, 1.6, 438.137, 33.995, 147.83),
    (1790998397.0008, 424.724, 47.252, 4.86, 1.92, 163.68, 41, 424.855, 47.700, 4.4, 1.8, 437.336, 34.500, 147.74),
    (1790998397.1130, 424.925, 47.209, 4.53, 1.92, 163.60, 45, 425.068, 47.596, 4.0, 1.8, 436.514, 35.019, 147.65),
    (1790998397.2212, 424.833, 47.650, 4.40, 1.80, 147.58, 40, 424.833, 47.650, 4.2, 1.6, 435.726, 35.519, 147.58),
    (1790998397.3343, 425.029, 47.675, 4.20, 2.00, 147.48, 47, 425.029, 47.675, 4.0, 1.8, 434.920, 36.032, 147.48),
    (1790998397.4456, 425.037, 47.695, 4.40, 2.00, 147.39, 54, 425.037, 47.695, 4.2, 1.8, 434.153, 36.521, 147.39),
    (1790998397.5565, 424.980, 47.629, 4.60, 1.80, 147.33, 52, 424.980, 47.629, 4.4, 1.6, 433.381, 37.016, 147.33),
)
# Same capture, 19.4-20.4 s: a parked car's rear face 30 m ahead, ego stopped. One
# ring re-samples it; the pre-fix rule read 3.65 m/s along the line of sight.
FAR_FACE_EGO_STOPPED = (
    (1790998406.5251, 395.838, 60.814, 0.40, 1.60, 152.55, 8, 395.838, 60.814, 0.2, 1.4, 421.034, 44.456, 152.55),
    (1790998406.6334, 395.843, 60.791, 0.40, 1.64, 144.43, 8, 395.774, 60.841, 0.2, 1.4, 420.973, 44.488, 152.56),
    (1790998406.7444, 396.631, 60.243, 0.77, 1.84, 145.44, 9, 396.584, 60.305, 0.6, 1.6, 420.943, 44.504, 152.56),
    (1790998406.8532, 396.911, 60.111, 0.60, 1.84, 145.43, 10, 396.846, 60.171, 0.4, 1.6, 420.938, 44.506, 152.56),
    (1790998406.9644, 396.449, 60.338, 0.77, 1.84, 145.43, 10, 396.402, 60.401, 0.6, 1.6, 420.938, 44.506, 152.56),
    (1790998407.0754, 395.754, 60.758, 1.07, 2.06, 126.00, 11, 395.560, 60.949, 1.0, 1.8, 420.938, 44.506, 152.56),
    (1790998407.1826, 395.884, 60.758, 0.57, 2.03, 146.23, 11, 395.826, 60.810, 0.4, 1.8, 420.938, 44.506, 152.57),
    (1790998407.2933, 395.883, 60.757, 0.57, 2.03, 146.23, 11, 395.825, 60.809, 0.4, 1.8, 420.938, 44.506, 152.57),
    (1790998407.4030, 395.883, 60.757, 0.57, 2.03, 146.23, 10, 395.825, 60.809, 0.4, 1.8, 420.938, 44.506, 152.57),
)
# follow-20261002T165436Z, 16.4-19.1 s: the same car beside the ego at 16 km/h,
# then cut by the x > -5 m clustering window: the rear edge follows the ego.
PARKED_CAR_CUT_BEHIND = (
    (1790960450.2545, -40.584, -129.586, 4.70, 1.71, 150.82, 74, -40.594, -129.588, 4.4, 1.6, -39.161, -135.791, 146.74),
    (1790960450.3657, -40.588, -129.611, 4.89, 1.65, 150.84, 82, -40.581, -129.598, 4.6, 1.6, -39.565, -135.526, 146.75),
    (1790960450.4744, -40.647, -129.556, 4.86, 1.62, 149.63, 76, -40.647, -129.556, 4.6, 1.6, -39.964, -135.264, 146.76),
    (1790960450.5839, -40.654, -129.545, 4.64, 1.62, 149.61, 72, -40.648, -129.554, 4.4, 1.6, -40.384, -134.989, 146.75),
    (1790960450.6943, -40.896, -129.875, 5.04, 1.65, 160.75, 69, -40.737, -129.494, 4.8, 1.6, -40.811, -134.709, 146.72),
    (1790960450.8031, -41.001, -129.811, 5.00, 1.65, 160.71, 69, -40.819, -129.438, 4.8, 1.6, -41.231, -134.433, 146.67),
    (1790960450.9127, -40.895, -129.558, 4.83, 1.42, 150.71, 74, -40.855, -129.531, 4.6, 1.4, -41.634, -134.167, 146.62),
    (1790960451.0243, -40.935, -129.552, 4.87, 1.41, 150.65, 79, -40.916, -129.489, 4.6, 1.4, -42.034, -133.904, 146.56),
    (1790960451.1344, -40.942, -129.590, 5.00, 1.40, 146.50, 79, -40.942, -129.590, 4.8, 1.2, -42.426, -133.644, 146.50),
    (1790960451.2446, -40.909, -129.611, 4.80, 1.40, 146.40, 81, -40.909, -129.611, 4.6, 1.2, -42.816, -133.386, 146.40),
    (1790960451.3561, -40.824, -129.547, 5.00, 1.60, 146.32, 80, -40.824, -129.547, 4.8, 1.4, -43.208, -133.125, 146.32),
    (1790960451.4653, -40.967, -129.665, 5.00, 1.81, 137.48, 91, -40.816, -129.432, 4.8, 1.6, -43.594, -132.867, 146.23),
    (1790960451.5755, -40.731, -129.369, 4.80, 2.00, 146.15, 94, -40.731, -129.369, 4.6, 1.8, -43.985, -132.605, 146.15),
    (1790960451.6852, -40.704, -129.332, 4.99, 2.02, 150.17, 101, -40.704, -129.389, 4.8, 1.8, -44.377, -132.342, 146.08),
    (1790960451.7934, -40.671, -129.413, 4.80, 2.00, 146.04, 99, -40.671, -129.413, 4.6, 1.8, -44.761, -132.084, 146.04),
    (1790960451.9048, -40.731, -129.375, 4.80, 2.00, 145.99, 99, -40.731, -129.375, 4.6, 1.8, -45.154, -131.819, 145.99),
    (1790960452.0164, -40.785, -129.274, 4.77, 2.01, 149.54, 93, -40.792, -129.335, 4.6, 1.8, -45.548, -131.553, 145.97),
    (1790960452.1278, -40.937, -129.238, 4.20, 2.00, 145.94, 88, -40.937, -129.238, 4.0, 1.8, -45.943, -131.286, 145.94),
    (1790960452.2403, -41.172, -129.081, 3.80, 2.00, 145.92, 82, -41.172, -129.081, 3.6, 1.8, -46.344, -131.015, 145.92),
    (1790960452.3519, -41.374, -128.902, 3.19, 2.20, 155.38, 72, -41.264, -128.898, 3.0, 2.0, -46.740, -130.747, 145.92),
    (1790960452.4628, -41.524, -128.784, 2.63, 2.14, 155.38, 57, -41.406, -128.802, 2.4, 2.0, -47.131, -130.482, 145.92),
    (1790960452.5748, -41.633, -128.648, 2.20, 2.20, 145.91, 52, -41.633, -128.648, 2.0, 2.0, -47.524, -130.216, 145.91),
    (1790960452.6843, -41.909, -128.582, 1.80, 2.00, 145.91, 38, -41.909, -128.582, 1.6, 1.8, -47.910, -129.955, 145.91),
    (1790960452.7966, -42.299, -128.475, 1.10, 2.23, 169.11, 27, -42.059, -128.481, 1.0, 1.8, -48.308, -129.686, 145.91),
    (1790960452.9095, -42.239, -128.271, 0.62, 1.84, 154.03, 16, -42.156, -128.295, 0.4, 1.6, -48.710, -129.414, 145.90),
)


def recorded_cluster(row, views=True):
    stamp, x, y, length, width, heading, count, mx, my, ex, ey, ox, oy, oh = row
    metadata = {"point_count": count, "motion_center": [mx, my]}
    if views:
        metadata.update(motion_extent=[ex, ey], observer=[ox, oy, oh])
    return (x, y, max(length, width), "tunnel_lidar_0", "obstacle", 0., length, width,
            heading, 0., 0., 0, metadata)


def replay(rows, views=True, **options):
    tracks = TunnelTracks(**options)
    return [tracks.update([recorded_cluster(row, views)], row[0])[0] for row in rows]


# The former rules had no car-length body gate (tunnel_tracks.VEHICLE_BODY_MOVING_MPS).
WITHOUT_BODY_GATE = patch('tunnel_tracks.VEHICLE_BODY_MOVING_MPS', 0.)


def pre_fix(rows):
    # Former rule: box-centre motion above 0.5 m/s, released below 0.3 m/s.
    with WITHOUT_BODY_GATE:
        return replay(rows, views=False, moving_threshold_mps=.5, release_mps=.3)


def box_outline(x, y, length, width, heading_deg=0.):
    yaw = math.radians(heading_deg)
    axes = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    corners = np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]])*[.5*length, .5*width]
    return corners @ axes.T+[x, y]


def person(x, y):
    # A walking person's legs and arms span about 0.6 m in every direction.
    return box_outline(x, y, .6, .6, 45.)


def crosses(p, q, a, b):
    side = lambda u, v, w: (v[0]-u[0])*(w[1]-u[1])-(v[1]-u[1])*(w[0]-u[0])
    return side(p, q, a)*side(p, q, b) < 0. and side(a, b, p)*side(a, b, q) < 0.


def surface_points(outline, sensor, occluders=(), spacing=.1):
    """Faces of a convex body that face the sensor and are not hidden by occluders."""
    outline = np.asarray(outline, dtype=float)
    centre, points = outline.mean(axis=0), []
    for a, b in zip(outline, np.roll(outline, -1, axis=0)):
        mid, edge = .5*(a+b), b-a
        normal = np.array([edge[1], -edge[0]])
        if normal @ (mid-centre) < 0.:
            normal = -normal
        if normal @ (sensor-mid) > 0.:
            points.extend(a+np.linspace(0., 1., max(2, int(np.linalg.norm(edge)/spacing)+1))[:, None]*edge)
    return np.asarray([point for point in points if not any(
        crosses(sensor, point, c, d) for occluder in occluders
        for c, d in zip(occluder, np.roll(occluder, -1, axis=0)))]).reshape(-1, 2)


def drive(bodies, ego_speed, steps=8, dt=.1, pre_fix=False):
    """Track bodies through lidar_objects views from an ego driving along +x."""
    if pre_fix:
        with WITHOUT_BODY_GATE:
            return _drive(bodies, ego_speed, steps, dt, pre_fix)
    return _drive(bodies, ego_speed, steps, dt, pre_fix)


def _drive(bodies, ego_speed, steps, dt, pre_fix):
    tracks = TunnelTracks(moving_threshold_mps=.5, release_mps=.3) if pre_fix else TunnelTracks()
    results = []
    for i in range(steps):
        t = i*dt
        ego = dict(x=ego_speed*t, y=0., heading_deg=0.)
        sensor = np.array([ego["x"]+1.05, 0.])
        outlines = [body(t) for body in bodies]
        world = np.vstack([surface_points(outline, sensor, outlines[:k])
                           for k, outline in enumerate(outlines)])
        objects = lidar_objects(world-[ego["x"], ego["y"]], ego)
        if pre_fix:
            objects = [o[:12]+({key: o[12][key] for key in ("point_count", "motion_center")},)
                       for o in objects]
        results.append((ego, tracks.update(objects, 1000.+t)))
    return results


def movers(results):
    return [[o for o in objects if o[4] == "pedestrian"] for _, objects in results]


class RecordedStaticBodies(unittest.TestCase):
    def test_parked_car_passed_at_30_kmh_is_not_a_moving_conflict(self):
        self.assertEqual([round(o[5], 2) for o in pre_fix(PARKED_CAR_PASS) if o[4] == "pedestrian"], [1.16])
        # A higher walking floor alone does not explain the reveal of its far end.
        with WITHOUT_BODY_GATE:
            self.assertTrue(any(o[4] == "pedestrian" for o in replay(PARKED_CAR_PASS, views=False)))
        # The car-length body gate alone keeps it static as well.
        self.assertFalse(any(o[4] == "pedestrian" for o in replay(PARKED_CAR_PASS, views=False)))
        for out in replay(PARKED_CAR_PASS):
            self.assertEqual(out[4], "obstacle")
            self.assertEqual(out[5], 0.)

    def test_far_face_resampled_while_stopped_is_not_moving(self):
        self.assertEqual([round(o[5], 2) for o in pre_fix(FAR_FACE_EGO_STOPPED) if o[4] == "pedestrian"], [3.65])
        for out in replay(FAR_FACE_EGO_STOPPED):
            self.assertEqual(out[4], "obstacle")

    def test_parked_car_cut_behind_the_ego_is_not_moving(self):
        tail = PARKED_CAR_CUT_BEHIND[-8:]
        drift = measured_velocity([(row[0]-tail[-1][0], row[7], row[8]) for row in tail], max_speed_mps=15.)
        self.assertGreater(math.hypot(*drift), 1.)   # the box centre follows the cut
        for out in replay(PARKED_CAR_CUT_BEHIND):
            self.assertEqual(out[4], "obstacle")

    def test_static_car_passed_on_either_side_never_moves(self):
        for lateral in (-3.3, 3.3):
            for skew in (0., 8.):
                with self.subTest(lateral=lateral, skew=skew):
                    car = lambda t, lateral=lateral, skew=skew: box_outline(20., lateral, 4.6, 1.9, skew)
                    if not skew:   # cut by the x > -5 m window, as at 300.5 s and 581.7 s
                        self.assertTrue(any(movers(drive([car], 8.3, steps=40, pre_fix=True))))
                    self.assertFalse(any(movers(drive([car], 8.3, steps=40))))


class MeasuredMoversStillYield(unittest.TestCase):
    route = np.column_stack((np.arange(0., 60., .5), np.zeros(120)))

    def test_walking_pedestrian_crossing_ahead_stops_the_car(self):
        from gui import moving_conflict
        results = drive([lambda t: person(20., -2.+1.2*t)], 7.)
        found = movers(results)
        self.assertEqual(len(found[-1]), 1)
        walker = found[-1][0]
        self.assertAlmostEqual(walker[10], 1.2, delta=.15)
        self.assertLess(abs(walker[9]), .3)
        ego = results[-1][0]
        self.assertEqual(moving_conflict([walker], ego, self.route+[ego["x"], 0.], 7.)["kind"], "stop")

    def test_pedestrian_walking_along_the_lane_is_confirmed(self):
        found = movers(drive([lambda t: person(18.-1.*t, .5)], 5.5))
        self.assertEqual(len(found[-1]), 1)
        self.assertAlmostEqual(found[-1][0][9], -1., delta=.15)

    def test_pedestrian_stepping_out_behind_a_parked_car_is_not_delayed(self):
        bodies = [lambda t: box_outline(16., -3.3, 4.6, 1.9), lambda t: person(20.5, -3.6+1.4*t)]
        for speed in (0., 7.):
            with self.subTest(ego_speed=speed):
                before = movers(drive(bodies, speed, 14, pre_fix=True))
                found = movers(drive(bodies, speed, 14))
                first = next(i for i, step in enumerate(found) if step)
                self.assertLessEqual(first, next(i for i, step in enumerate(before) if step))
                self.assertGreater(found[-1][0][10], 1.)

    def test_car_driving_several_mps_ahead_is_confirmed(self):
        found = movers(drive([lambda t: box_outline(15.+4.*t, -3.5, 4.5, 1.8)], 10.))
        self.assertEqual(len(found[-1]), 1)
        self.assertAlmostEqual(found[-1][0][9], 4., delta=.4)

    def test_oncoming_car_beyond_single_ring_range_is_confirmed_once_it_moved(self):
        found = movers(drive([lambda t: box_outline(30.-5.*t, 3.5, 4.5, 1.8)], 8.))
        first = next(i for i, step in enumerate(found) if step)
        self.assertLessEqual(first, 5)
        self.assertAlmostEqual(found[-1][0][9], -5., delta=.5)

    def test_far_crossing_pedestrian_keeps_its_tangential_motion(self):
        results = drive([lambda t: person(40., -2.+1.2*t)], 5.)
        ego, found = results[-1][0], movers(results)[-1]
        self.assertEqual(len(found), 1)
        self.assertGreater(math.hypot(found[0][0]-ego["x"], found[0][1]-ego["y"]), RADIAL_MOTION_RANGE_M)
        self.assertAlmostEqual(found[0][10], 1.2, delta=.15)


class DetectorCarMotionTests(unittest.TestCase):
    # npc/500032 on parked car 40100008, follow-20261009T204445Z 293.5-294.3 s:
    # 2.89-2.92 m/s with velocity_valid. Pointed here at the path, 3.3 m to its right.
    parked = (10., -3.3, 4.93, "lidar_car", "npc", 2.9, 4.93, 2.09, 0., 0., 2.9, 500032, 0., None, True, 0.)

    def test_measured_mover_must_lie_in_the_detector_box(self):
        inside = (11.4, -2.4, .6, "tunnel_lidar_dynamic_1", "pedestrian", 1.2)
        outside = (10., 1., .6, "tunnel_lidar_dynamic_2", "pedestrian", 1.2)
        self.assertTrue(measured_motion_supports(self.parked, [inside]))
        self.assertFalse(measured_motion_supports(self.parked, [outside]))
        self.assertEqual(measured_vehicle(self.parked, [inside]), self.parked)
        static = measured_vehicle(self.parked, [outside])
        self.assertEqual((static[5], static[9], static[10]), (0., 0., 0.))
        self.assertEqual(static[:5]+static[6:9]+static[11:], self.parked[:5]+self.parked[6:9]+self.parked[11:])

    def test_unknown_or_beyond_drift_detector_motion_is_kept(self):
        unknown = self.parked[:14]+(False, 0.)
        self.assertEqual(measured_vehicle(unknown, []), unknown)
        fast = self.parked[:5]+(DETECTOR_BOX_DRIFT_MPS,)+self.parked[6:10]+(DETECTOR_BOX_DRIFT_MPS,)+self.parked[11:]
        self.assertEqual(measured_vehicle(fast, []), fast)

    def test_publish_check_takes_unknown_velocity_as_static_without_a_measured_mover(self):
        unknown = self.parked[:5]+(0.,)+self.parked[6:9]+(0., 0.)+self.parked[11:14]+(False, 0.)
        static = measured_vehicle(unknown, [], unknown_static=True)
        self.assertEqual((static[5], static[9], static[10], static[14]), (0., 0., 0., True))
        inside = (11.4, -2.4, .6, "tunnel_lidar_dynamic_1", "pedestrian", 1.2)
        self.assertEqual(measured_vehicle(unknown, [inside], unknown_static=True), unknown)
        fast = self.parked[:5]+(DETECTOR_BOX_DRIFT_MPS,)+self.parked[6:10]+(DETECTOR_BOX_DRIFT_MPS,)+self.parked[11:]
        self.assertEqual(measured_vehicle(fast, [], unknown_static=True), fast)

    def runtime(self, points, car):
        from types import SimpleNamespace
        from unittest.mock import Mock
        import test_tunnel_integration as fixture
        runtime = fixture.TunnelIntegrationTests().runtime(points)
        runtime.highway = SimpleNamespace(config=dict(lidar_regions=[dict(from_idx=0, to_idx=10000)]),
                                         active=lambda index: False, phase='OUTSIDE')
        runtime.latest_status = runtime.test_status
        runtime.vehicle_receiver = SimpleNamespace(snapshot=Mock(return_value=([car], True, {})))
        return runtime

    @staticmethod
    def scan(runtime, points, stamp, sequence):
        runtime.tunnel.obstacle_points = np.asarray(points, dtype=np.float32)
        xyz = np.column_stack((points, np.full(len(points), .6))).astype(np.float32)
        runtime.test_scan = dict(runtime.test_scan, xyz=np.vstack((runtime.test_background_xyz, xyz)),
                                 stamp=stamp, sequence=sequence)
        runtime.test_status["ego"]["stamp"] = stamp
        runtime.infer()
        return runtime._web_record

    def test_parked_detector_car_with_drifting_box_is_bypassed_without_a_stop(self):
        from unittest.mock import patch
        face = np.array([(x, -2.3) for x in np.arange(7.6, 12.45, .2)] +
                        [(7.6, y) for y in np.arange(-4.3, -2.2, .2)])
        for corroborated in (True, False):
            with self.subTest(corroborated=corroborated), \
                    patch('tunnel_tracks.measured_motion_supports', return_value=corroborated):
                runtime = self.runtime(face, self.parked)
                first = runtime.test_scan["stamp"]
                for i in range(5):
                    row = self.scan(runtime, face, first+.1*i, i+1)
                car = next(o for o in row['tunnel']['objects'] if o['name'] == 'lidar_car')
                if corroborated:   # former rule: detector velocity alone stopped the car
                    self.assertEqual(row['stop_reason'], 'tunnel moving object conflict')
                    continue
                self.assertEqual(row['tunnel']['moving_objects'], 0)
                self.assertEqual(car['speed_mps'], 0.)
                self.assertNotEqual(row['stop_reason'], 'tunnel moving object conflict')
                self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_detector_car_with_measured_crossing_motion_still_stops_the_car(self):
        def face(t):   # door panels and the window line of a car side, 0.4 m apart
            return np.array([(x, y) for x in (11.1, 11.5) for y in np.arange(-4.8+3.*t, -1.25+3.*t, .2)])
        car = lambda t: (12., -3.+3.*t, 4.5, "lidar_car", "npc", 3., 4.5, 1.8, 90., 0., 3.,
                         500040, 0., None, True, 0.)
        runtime = self.runtime(face(0.), car(0.))
        first = runtime.test_scan["stamp"]
        for i in range(5):
            runtime.vehicle_receiver.snapshot.return_value = ([car(.1*i)], True, {})
            row = self.scan(runtime, face(.1*i), first+.1*i, i+1)
        logged = next(o for o in row['tunnel']['objects'] if o['name'] == 'lidar_car')
        self.assertEqual(logged['speed_mps'], 3.)
        self.assertEqual(row['tunnel']['moving_objects'], 2)
        self.assertEqual(row['stop_reason'], 'tunnel moving object conflict')
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)


if __name__ == "__main__":
    unittest.main()
