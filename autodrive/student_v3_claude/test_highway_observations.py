"""Recorded stop pulses, temporal confirmation and perception source freshness."""
import json
import importlib.util
import math
from collections import deque
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from gui import RosObjectReceiver, highway_static_objects, static_clear_distance
from highway_observations import HighwayObservationFilter, source_lag_s
from highway_planner import local_points, timed_gap
from tunnel_runtime import swept_clearance


def obstacle(x=5., y=0., speed=0., heading=0., object_id=1,
             name="fusion_obstacle", kind="obstacle"):
    yaw = math.radians(heading)
    return (x, y, 1.1, name, kind, speed, 1.1, 1.1, heading,
            speed*math.cos(yaw), speed*math.sin(yaw), object_id)


class HighwayObservations(unittest.TestCase):
    def setUp(self):
        self.filter = HighwayObservationFilter()
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=70.)
        self.points = np.empty((0, 2))

    def update(self, objects, stamp, points=None, **kwargs):
        return self.filter.update(objects, stamp, self.points if points is None else points,
                                  self.ego, **kwargs)

    def test_one_or_two_frames_cannot_confirm(self):
        for stamp in (100., 100.1):
            accepted, pending, _ = self.update([obstacle()], stamp)
            self.assertEqual(accepted, [])
            self.assertEqual(len(pending), 1)
        self.assertEqual(self.update([], 100.2)[0], [])

    def test_three_distinct_frames_confirm_with_minimum_elapsed_time(self):
        for stamp in (100., 100.01, 100.02):
            self.assertEqual(self.update([obstacle()], stamp)[0], [])
        accepted, pending, _ = self.update([obstacle()], 100.15)
        self.assertEqual(accepted, [obstacle()])
        self.assertEqual(pending, [])

    def test_repeated_decisions_do_not_count_as_new_frames(self):
        for _ in range(20):
            accepted, pending, _ = self.update([obstacle()], 100.)
            self.assertEqual(accepted, [])
        self.assertEqual(len(pending), 1)
        self.assertEqual(self.filter.tracks[1]["count"], 1)

    def test_missing_frame_resets_unconfirmed_sequence(self):
        self.update([obstacle()], 100.)
        self.update([obstacle()], 100.1)
        self.update([], 100.2)
        self.assertEqual(self.update([obstacle()], 100.3)[0], [])
        self.assertEqual(self.filter.tracks[1]["count"], 1)

    def test_expired_or_clock_reset_track_requires_new_confirmation(self):
        for stamp in (100., 100.1, 100.2):
            self.update([obstacle()], stamp)
        self.assertEqual(self.update([obstacle()], 100.6)[0], [])
        self.assertEqual(self.update([obstacle()], 1.)[0], [])

    def test_current_lidar_confirms_first_frame_in_rotated_world_coordinates(self):
        self.ego.update(x=10., y=20., heading_deg=90.)
        obj = obstacle(10., 25., heading=90.)
        accepted, pending, _ = self.update([obj], 100., np.array([[5., -.1], [5., .1]]))
        self.assertEqual(accepted, [obj])
        self.assertEqual(pending, [])

    def test_low_speed_pedestrians_vehicles_and_legacy_stamps_keep_protection(self):
        self.ego["speed_kmh"] = 10.
        self.assertEqual(self.update([obstacle()], 100.)[0], [obstacle()])
        self.ego["speed_kmh"] = 70.
        for name, kind in (("fusion_pedestrian", "pedestrian"), ("lidar_car", "npc"),
                           ("TrafficBarrel", "obstacle")):
            obj = obstacle(name=name, kind=kind)
            self.assertEqual(self.update([obj], 101.)[0], [obj])
        self.assertEqual(self.update([obstacle()], None)[0], [obstacle()])
        self.assertEqual(self.update([obstacle()], 0.)[0], [obstacle()])

    def test_confirmed_static_world_position_is_never_shifted_by_ego_speed_or_age(self):
        obj = obstacle()
        for stamp in (100., 100.1, 100.2):
            accepted, _, _ = self.update([obj], stamp, age_s=.25)
        self.assertEqual(accepted[0][:2], obj[:2])

    def test_single_motion_heading_spike_is_stationary_until_position_history_supports_it(self):
        for i in range(7):
            self.update([obstacle(20., 3.+.03*(i % 2), heading=0.)], 100.+i*.1)
        accepted, _, info = self.update([obstacle(20., 3., speed=1.4, heading=-90.)], 100.7)
        self.assertEqual(info["motion_rejected"], 1)
        self.assertEqual(accepted[0][5], 0.)
        self.assertEqual(accepted[0][8:11], (0., 0., 0.))

    def test_real_consistent_motion_is_retained_and_propagated_by_own_velocity(self):
        for i in range(6):
            obj = obstacle(20., 3.-i*.1, speed=1., heading=-90.)
            accepted, _, _ = self.update([obj], 100.+i*.1, age_s=.2)
        self.assertAlmostEqual(accepted[0][5], 1.)
        self.assertAlmostEqual(accepted[0][0], 20.)
        self.assertAlmostEqual(accepted[0][1], 2.3)

    def test_two_speed_spikes_are_rejected_even_if_centroid_history_suggests_motion(self):
        for i in range(6):
            self.update([obstacle(20., 3.-i*.1)], 100.+i*.1)
        for i in (6, 7):
            obj = obstacle(20., 3.-i*.1, speed=1., heading=-90.)
            accepted, _, info = self.update([obj], 100.+i*.1,
                                            np.array([[20., obj[1]], [20.1, obj[1]]]))
            self.assertEqual(accepted[0][5], 0.)
            self.assertEqual(info["motion_rejected"], 1)
        accepted, _, _ = self.update([obstacle(20., 2.2, speed=1., heading=-90.)], 100.8)
        self.assertAlmostEqual(accepted[0][5], 1.)

    def test_real_pedestrian_crossing_preserves_first_frame_prediction(self):
        person = obstacle(30., 2.3, speed=1., heading=-90.,
                          name="fusion_pedestrian", kind="pedestrian")
        accepted, _, _ = self.update([person], 100.)
        path = np.column_stack((np.arange(0., 170.5, .5), np.zeros(341)))
        before = timed_gap(path, self.ego, [person], 70./3.6)
        self.assertIsNotNone(before)
        self.assertEqual(timed_gap(path, self.ego, accepted, 70./3.6), before)

    def test_position_outlier_does_not_replace_or_refresh_confirmed_track(self):
        for stamp in (100., 100.1, 100.2):
            self.update([obstacle(20.)], stamp)
        accepted, _, info = self.update([obstacle(2.)], 100.3)
        self.assertEqual(info["position_rejected"], 1)
        self.assertEqual(accepted[0][0], 20.)
        self.assertEqual(self.filter.tracks[1]["seen"], 100.2)
        self.assertEqual(self.update([obstacle(2.)], 100.6)[0], [])

    def test_position_jump_with_current_lidar_is_not_rejected(self):
        for stamp in (100., 100.1, 100.2):
            self.update([obstacle(20.)], stamp)
        accepted, _, info = self.update([obstacle(3.)], 100.3, np.array([[3., 0.], [3.1, 0.]]))
        self.assertEqual(info["position_rejected"], 0)
        self.assertEqual(accepted[0][0], 3.)

    def test_rejected_position_preserves_confirmed_motion_to_decision_time(self):
        for i in range(6):
            self.update([obstacle(20., 3.-i*.1, speed=1., heading=-90.)], 100.+i*.1)
        accepted, _, info = self.update([obstacle(2., speed=1., heading=-90.)], 100.6, age_s=.2)
        self.assertEqual(info["position_rejected"], 1)
        self.assertAlmostEqual(accepted[0][0], 20.)
        self.assertAlmostEqual(accepted[0][1], 2.2)

    def test_confirmed_shape_rejects_two_orientation_spikes_even_with_lidar_support(self):
        for stamp in (100., 100.1, 100.2):
            self.update([obstacle()], stamp)
        for stamp in (100.3, 100.4):
            accepted, pending, info = self.update([obstacle(heading=45.)], stamp,
                                            np.array([[5., 0.], [5.1, 0.]]))
            self.assertEqual(accepted, [])
            self.assertEqual(pending[0][8], 0.)
            self.assertEqual(info['shape_rejected'], 1)
        self.assertEqual(self.update([obstacle()], 100.5)[0][0][8], 0.)

    def test_persistent_new_orientation_is_accepted_after_three_source_frames(self):
        for stamp in (100., 100.1, 100.2):
            self.update([obstacle()], stamp)
        for stamp in (100.3, 100.4, 100.5):
            accepted, _, _ = self.update([obstacle(heading=45.)], stamp)
        self.assertEqual(accepted[0][8], 45.)

    def test_size_expansion_requires_confirmation_and_keeps_current_position(self):
        for stamp in (100., 100.1, 100.2):
            self.update([obstacle()], stamp)
        obj = list(obstacle(5.1)); obj[2] = obj[6] = 4.4
        for stamp in (100.3, 100.4):
            accepted, pending, info = self.update([tuple(obj)], stamp)
            self.assertEqual(accepted, [])
            self.assertEqual(pending[0][6], 1.1)
            self.assertEqual(pending[0][0], 5.1)
            self.assertEqual(info['shape_rejected'], 1)
        self.assertEqual(self.update([tuple(obj)], 100.5)[0][0][6], 4.4)

    def test_smaller_nested_measurement_does_not_retain_an_inflated_first_box(self):
        wide = list(obstacle())
        wide[6:9] = [3.25, 2.42, 0.]
        self.update([tuple(wide)], 100., np.array([[5., 0.], [5.1, 0.]]))
        narrow = list(wide)
        narrow[6:9] = [2.39, 1.1, .06]
        accepted, _, info = self.update([tuple(narrow)], 100.1)
        self.assertEqual(accepted[0][6:9], tuple(narrow[6:9]))
        self.assertEqual(info['shape_rejected'], 0)

    def test_equivalent_pca_axis_reversal_or_exchange_does_not_delay_geometry(self):
        original = list(obstacle()); original[2] = original[6] = 3.
        self.update([tuple(original)], 100., np.array([[5., 0.], [5.1, 0.]]))
        for stamp, heading, length, width in ((100.1, 180., 3., 1.1),
                                               (100.2, 90., 1.1, 3.)):
            obj = list(original); obj[6:9] = [length, width, heading]
            accepted, _, info = self.update([tuple(obj)], stamp)
            self.assertEqual(info['shape_rejected'], 0)
            self.assertEqual(accepted[0][6:8], (3., 1.1))

    def test_duplicate_capture_cannot_confirm_new_shape(self):
        for stamp in (100., 100.1, 100.2):
            self.update([obstacle()], stamp)
        for _ in range(10):
            accepted, pending, _ = self.update([obstacle(heading=45.)], 100.3)
            self.assertEqual(accepted, [])
            self.assertEqual(pending[0][8], 0.)


class RecordedHighwayStops(unittest.TestCase):
    def test_recorded_full_brake_triggers_are_removed(self):
        cases = json.loads((Path(__file__).parent / "test_data" /
                            "highway_observation_spikes_20261007.json").read_text())
        self.assertEqual(len(cases), 5)
        for case in cases:
            with self.subTest(run=case["run"]):
                gate = HighwayObservationFilter()
                ego = case["ego"]
                path = local_points(np.asarray(case["path"]), ego)
                # Replay distinct captured observations, rather than treating
                # repeated controller reads of one message as confirmations.
                for frame in case["frames"]:
                    raw = [tuple(obj) for obj in frame["objects"]]
                    scans = [s for s in case["lidar_scans"] if s["received"] <= frame["received"]]
                    points = np.asarray(scans[-1]["points"] if scans else [], dtype=float).reshape(-1, 2)
                    accepted, pending, info = gate.update(
                        raw, frame["stamp"], points, ego,
                        age_s=frame["received"]-frame["stamp"])
                    if frame["stamp"] != case["target_stamp"]:
                        continue
                    if case["target_id"] == 14:
                        conflict = timed_gap(path, ego, raw, ego["speed_kmh"]/3.6)
                        self.assertIsNotNone(conflict)
                        # The actual nose reaches this rejected ghost at least as early.
                        self.assertLessEqual(conflict["when"], .5)
                        self.assertIsNone(timed_gap(path, ego, accepted, ego["speed_kmh"]/3.6))
                        self.assertGreaterEqual(info["motion_rejected"]+info["position_rejected"], 1)
                    elif case['target_id'] == 23:
                        self.assertEqual(static_clear_distance(path, ego, raw, 170.), 3.)
                        self.assertGreaterEqual(info['shape_rejected'], 1)
                    elif case['target_id'] == 86:
                        self.assertEqual(static_clear_distance(path, ego, raw, 170.), 170.)
                        self.assertEqual(info['shape_rejected'], 0)
                    else:
                        self.assertEqual(static_clear_distance(path, ego, raw, 170.), 0.)
                        self.assertEqual(len(pending), 1)
                        self.assertEqual(accepted, [])
                    self.assertEqual(static_clear_distance(path, ego, highway_static_objects(accepted), 170.), 170.)
                    self.assertEqual(swept_clearance(path, points, 170., extra_margin=.25,
                                                    lateral_margin=.15), 170.)
                    break
                else:
                    self.fail("Recorded event observation was not replayed")


class ObjectSourceFreshness(unittest.TestCase):
    def setUp(self):
        self.receiver = RosObjectReceiver.__new__(RosObjectReceiver)
        self.receiver.lock = threading.Lock()
        self.receiver.objects, self.receiver.received = [], 0.
        self.receiver._tracks, self.receiver._motion = {}, {}
        self.receiver._turn, self.receiver._position_history = {}, {}
        self.receiver.file_cache = SimpleNamespace(value={})
        self.clock = 100.
        self.receiver._source_clock = lambda: self.clock

    def receive(self, stamp, mono=1000.):
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)),
                                  npc_list=[], pedestrian_list=[], obstacle_list=[])
        with patch("gui.time.monotonic", return_value=mono):
            self.receiver.on_objects(message)

    def snapshot(self, mono=1000., wall=100.):
        with patch("gui.time.monotonic", return_value=mono), patch("gui.time.time", return_value=wall):
            return self.receiver.snapshot()

    def test_source_delay_adds_to_receipt_age(self):
        self.receive(99.85)
        self.snapshot(mono=1000.1)
        self.assertAlmostEqual(self.receiver.age_s, .25)
        self.assertAlmostEqual(self.receiver.receive_age_s, .1)
        self.assertEqual(self.receiver.observation_stamp, 99.85)

    def test_duplicate_or_older_source_cannot_refresh_receipt_time(self):
        self.receive(99.9)
        self.receive(99.9, mono=1000.2)
        self.receive(99.8, mono=1000.3)
        self.snapshot(mono=1000.4)
        self.assertEqual(self.receiver.received, 1000.)
        self.assertAlmostEqual(self.receiver.age_s, .5)

    def test_sim_clock_reset_accepts_new_epoch(self):
        self.receive(99.9)
        self.clock = 1.1
        self.receive(1., mono=1000.2)
        self.snapshot(mono=1000.3)
        self.assertEqual(self.receiver.observation_stamp, 1.)
        self.assertAlmostEqual(self.receiver.age_s, .2)

    def test_file_fallback_preserves_capture_age_stamp_and_identity(self):
        self.receive(99.4)
        obj = obstacle(object_id=7)
        self.receiver.file_cache.value = dict(t=99.98, source_stamp=99.8,
                                             source_lag_s=.18, objects=[list(obj)])
        self.assertEqual(self.snapshot(mono=1000.1), [obj])
        self.assertEqual(self.receiver.src, "file")
        self.assertAlmostEqual(self.receiver.age_s, .2)
        self.assertEqual(self.receiver.observation_stamp, 99.8)

    def test_old_capture_in_fresh_file_is_not_used(self):
        self.receive(99.7)
        self.receiver.file_cache.value = dict(t=100., source_stamp=98., source_lag_s=2.,
                                             objects=[list(obstacle())])
        self.assertEqual(self.snapshot(), [])
        self.assertEqual(self.receiver.src, "ros")

    def test_invalid_or_future_file_age_cannot_override_ros(self):
        self.receive(99.7)
        for lag in (float("nan"), float("inf"), -.2):
            self.receiver.file_cache.value = dict(t=100., source_lag_s=lag,
                                                 objects=[list(obstacle())])
            self.assertEqual(self.snapshot(), [])
            self.assertEqual(self.receiver.src, "ros")

    def test_invalid_source_time_fails_closed(self):
        for stamp in (float("nan"), float("inf"), -1., 100.1):
            with self.subTest(stamp=stamp):
                self.assertEqual(source_lag_s(stamp, 100.), 999.)
                self.receive(stamp)
                self.snapshot()
                self.assertGreater(self.receiver.age_s, 1.)
        self.assertEqual(source_lag_s(0., 100.), 0.)
        self.assertEqual(source_lag_s(100.01, 100.), 0.)


class StatusFileFreshness(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            "highway_status_test", Path(__file__).parents[1] / "ros" / "status_node.py")
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def write_observation(self, stamp):
        status = self.module.Status.__new__(self.module.Status)
        status.obj_t = deque()
        writes = []
        status.write_sample = lambda name, data: writes.append((name, data))
        obj = SimpleNamespace(unique_id=7, name="fusion_obstacle", heading=90.,
                              position=SimpleNamespace(x=5., y=0.),
                              size=SimpleNamespace(x=1., y=1.),
                              velocity=SimpleNamespace(x=3.6, y=0.))
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)),
                                  npc_list=[], pedestrian_list=[], obstacle_list=[obj])
        with patch.object(self.module.rospy, "get_time", return_value=100.), \
                patch.object(self.module.time, "time", return_value=100.):
            status.on_objects(message)
        self.assertEqual(writes[0][0], "objects.json")
        return writes[0][1]

    def test_status_file_preserves_identity_capture_time_and_velocity_units(self):
        row = self.write_observation(99.8)
        self.assertEqual(row["source_stamp"], 99.8)
        self.assertAlmostEqual(row["source_lag_s"], .2)
        self.assertEqual(row["objects"][0][11], 7)
        self.assertAlmostEqual(row["objects"][0][5], 1.)
        self.assertAlmostEqual(row["objects"][0][10], 1.)

    def test_status_file_invalid_stamps_fail_closed(self):
        for stamp in (float("nan"), float("inf"), -1., 100.1):
            self.assertEqual(self.write_observation(stamp)["source_lag_s"], 999.)
        self.assertEqual(self.write_observation(0.)["source_lag_s"], 0.)


if __name__ == "__main__":
    unittest.main()
