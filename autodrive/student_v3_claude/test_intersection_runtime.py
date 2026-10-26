"""Exercise advisory model outputs, LiDAR traffic and the mandatory full stop."""
from collections import deque
from types import SimpleNamespace
from unittest.mock import patch
import threading
import unittest

import numpy as np
import torch

import test_avoidance_gate as fixture
import gui
from gui import Runtime, VIEWS
from stop_line import INTERSECTION_ACCEL_MPS2, INTERSECTION_ENTRY_MPS, StopLineController


class IntersectionRuntimeTests(unittest.TestCase):
    def runtime(self):
        runtime = fixture.AvoidanceGateTests().runtime(action="KEEP")
        runtime.actions = ["KEEP_LANE", "YIELD", "LEFT_CHANGE", "RIGHT_CHANGE"]
        runtime.test_status["ego"].update(x=46.6825, speed_kmh=0.)
        runtime.lanes.route_progress = lambda ego: ego["x"]
        runtime.stop_line_controller = StopLineController([dict(id="entry", station_m=50., xy=[50., 0.])])
        runtime.intersection_regions = [dict(node_id="test", start_station_m=0.,
            stop_station_m=50., exit_station_m=80., end_station_m=105.)]
        runtime.test_cars, runtime.test_lidar_ready = [], True
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda: (
            runtime.test_cars, runtime.test_lidar_ready,
            dict(ready=runtime.test_lidar_ready, age_s=.01)))
        runtime.receiver.stale_views = lambda max_age_s: []
        return runtime

    def step(self, runtime, step):
        now = 100. + step / 15.
        with patch("gui.time.monotonic", return_value=now):
            runtime.lidar.received = now
            runtime.object_receiver.received = now
            self.assertTrue(runtime.infer())
        return runtime._web_record

    @staticmethod
    def car(x, y, vx=0., vy=0., uid=500001, velocity_valid=True, heading=0.):
        return (x, y, 4.5, "lidar_car", "npc", (vx*vx+vy*vy)**.5,
                4.5, 1.8, heading, vx, vy, uid, 0., None, velocity_valid)

    def test_model_go_and_clear_lidar_release_only_after_one_second(self):
        runtime = self.runtime()
        for step in range(15):
            row = self.step(runtime, step)
            self.assertTrue(row["emergency_stop"])
            self.assertEqual(row["published_cap_kmh"], 0.)
        row = self.step(runtime, 15)
        self.assertEqual(row["stop_line"]["state"], "ENTER")
        self.assertFalse(row["emergency_stop"])
        self.assertTrue(row["stop_line"]["entry_decision"]["clear"])
        self.assertEqual(runtime.gap_planner.ACCEL_MPS2, INTERSECTION_ACCEL_MPS2)
        self.assertAlmostEqual(runtime.gap_planner.entry_cap_mps, INTERSECTION_ENTRY_MPS)
        self.assertAlmostEqual(runtime.gap_planner.entry_cap_distance_m, 15.3175)

    def test_zero_dwell_first_checked_gap_is_published_and_committed_immediately(self):
        runtime = self.runtime()
        runtime.stop_line_controller = StopLineController(runtime.stop_line_controller.lines, hold_s=0.)
        row = self.step(runtime, 0)
        self.assertEqual(row['stop_line']['state'], 'ENTER')
        self.assertEqual(row['stop_line']['required_hold_s'], 0.)
        self.assertTrue(row['stop_line']['entry_decision']['clear'])
        self.assertFalse(row['emergency_stop'])
        self.assertGreater(row['published_cap_kmh'], 0.)
        self.assertTrue(runtime.gap_planner.junction_committed)

    def test_zero_dwell_retains_lidar_readiness_unknown_motion_and_crossing_vetoes(self):
        for ready, cars in ((False, []), (True, [self.car(55., 10., velocity_valid=False)]),
                            (True, [self.car(54.6825, 8., vy=-8., heading=90.)])):
            with self.subTest(ready=ready, cars=cars):
                runtime = self.runtime()
                runtime.stop_line_controller = StopLineController(runtime.stop_line_controller.lines, hold_s=0.)
                runtime.test_lidar_ready, runtime.test_cars = ready, cars
                row = self.step(runtime, 0)
                self.assertTrue(row['emergency_stop'])
                self.assertEqual(row['published_cap_kmh'], 0.)
                self.assertNotEqual(row['stop_line']['state'], 'ENTER')
                self.assertFalse(runtime.gap_planner.junction_committed)

    def test_model_stop_cannot_block_a_measured_clear_gap_after_full_stop(self):
        runtime = self.runtime()
        runtime.actions[0] = "STOP"
        for step in range(15):
            row = self.step(runtime, step)
            self.assertTrue(row["emergency_stop"])
        row = self.step(runtime, 15)
        self.assertEqual(row["stop_line"]["state"], "ENTER")
        self.assertFalse(row["emergency_stop"])
        self.assertTrue(row["stop_line"]["entry_decision"]["model_advisory"])
        self.assertFalse(row["stop_line"]["entry_decision"]["model_permits_entry"])

    def test_first_safe_follow_gap_releases_without_another_one_second_wait(self):
        runtime = self.runtime()
        runtime.actions[0] = "STOP"
        with patch("gui.GapPlanner.evaluate", return_value=dict(kind="stop", source="traffic",
                name="crossing_car", along=8., t=1.,
                reason="predicted footprint time overlap", v_mps=0.)):
            for step in range(20):
                row = self.step(runtime, step)
                self.assertTrue(row["emergency_stop"])
        with patch("gui.GapPlanner.evaluate", return_value=dict(kind="follow", source="traffic",
                name="departing_car", along=15., t=0.,
                reason="follow merging lead with checked headway", v_mps=3.)):
            row = self.step(runtime, 20)
        self.assertEqual(row["stop_line"]["state"], "ENTER")
        self.assertFalse(row["emergency_stop"])

    def test_lidar_crossing_car_overrides_model_go(self):
        runtime = self.runtime()
        runtime.test_cars = [self.car(54.6825, 8., vy=-8., heading=90.)]
        for step in range(18):
            row = self.step(runtime, step)
            self.assertTrue(row["emergency_stop"])
            self.assertNotEqual(row["stop_line"]["state"], "ENTER")
        self.assertFalse(row["stop_line"]["entry_decision"]["traffic_clear"])
        self.assertTrue(row["traffic_risks"])

    def test_unready_or_unmeasured_vehicle_cannot_be_treated_as_empty(self):
        for detector_ready, cars in ((False, []), (True, [self.car(55., 10., velocity_valid=False)])):
            with self.subTest(detector_ready=detector_ready):
                runtime = self.runtime()
                runtime.test_lidar_ready, runtime.test_cars = detector_ready, cars
                for step in range(18):
                    row = self.step(runtime, step)
                    self.assertTrue(row["emergency_stop"])
                decision = row["stop_line"]["entry_decision"]
                self.assertEqual(decision["lidar_ready"], detector_ready)
                self.assertFalse(decision["clear"])
                if detector_ready:
                    # A nearby car without measured velocity (no lane to judge it by) holds us.
                    self.assertFalse(decision["traffic_clear"])
                    self.assertIn("without measured velocity", row["stop_reason"])

    def test_stale_model_view_does_not_veto_other_fresh_sensor_clearance(self):
        runtime = self.runtime()
        runtime.receiver.stale_views = lambda max_age_s: ["left"]
        for step in range(16):
            row = self.step(runtime, step)
        self.assertFalse(row["emergency_stop"])
        self.assertEqual(row["stop_line"]["state"], "ENTER")
        self.assertFalse(row["stop_line"]["entry_decision"]["model_fresh"])

    def test_committed_entry_ignores_model_jitter_and_detector_outage(self):
        runtime = self.runtime()
        for step in range(16):
            self.step(runtime, step)
        runtime.actions[0] = "STOP"
        self.assertFalse(self.step(runtime, 16)["emergency_stop"])
        self.assertTrue(runtime.gap_planner.junction_committed)
        # User rule 2026-10-10: once committed, following only - a detector outage is no
        # stop (run 222: stopping in the circulating lane is what got us hit); the raw
        # LiDAR sweep and the remembered tracks still brake us behind what is ahead.
        runtime.test_lidar_ready = False
        for step in range(17, 17 + 30):
            row = self.step(runtime, step)
            self.assertFalse(row["emergency_stop"], row["stop_reason"])
        runtime.test_status["ego"]["x"] = 75.
        self.assertFalse(self.step(runtime, 47)["emergency_stop"])
        self.assertTrue(runtime.gap_planner.junction_committed)
        runtime.test_status["ego"]["x"] = 120.
        self.assertFalse(self.step(runtime, 48)["emergency_stop"])
        self.assertFalse(runtime.gap_planner.junction_committed)      # junction left
        self.assertEqual(runtime.gap_planner.ACCEL_MPS2, 3.)
        self.assertIsNone(runtime.gap_planner.entry_cap_mps)

    def test_vehicle_fusion_keeps_people_obstacles_and_separate_cars(self):
        runtime = self.runtime()
        runtime.test_cars = [self.car(55., 3.)]
        fused = [(55.2, 3.1, 4., "camera_car", "npc", 0.),
                 (55., 6.5, 4., "other_car", "npc", 0.),
                 (55., 3., .6, "person", "pedestrian", 0.),
                 (55., 3., .4, "box", "obstacle", 0.)]
        merged = runtime.intersection_objects(fused, runtime.test_status["ego"])
        self.assertEqual([obj[3] for obj in merged], ["other_car", "person", "box", "lidar_car"])
        runtime.test_status["ego"]["x"] = 200.
        self.assertEqual(runtime.intersection_objects(fused, runtime.test_status["ego"]), fused)
        self.assertIsNone(runtime._intersection_vehicle_status)

    def test_real_camera_receiver_requires_every_recent_view(self):
        runtime = Runtime.__new__(Runtime)
        runtime.receiver = SimpleNamespace(lock=threading.Lock(),
            counts={view: deque([100.]) for view, _ in VIEWS})
        with patch("gui.time.monotonic", return_value=100.2):
            self.assertTrue(runtime.model_camera_fresh())
            runtime.receiver.counts["left"].clear()
            self.assertFalse(runtime.model_camera_fresh())
            runtime.receiver.counts["left"].append(99.)
            self.assertFalse(runtime.model_camera_fresh())

    def test_departing_tracked_car_is_not_also_a_static_lidar_stop(self):
        runtime = self.runtime()
        for step in range(16):
            self.step(runtime, step)
        ego_x = runtime.test_status["ego"]["x"]
        runtime.test_cars = [self.car(ego_x+4., 3.7, vy=5., heading=90.)]
        runtime.lidar.pts = np.asarray([(3.3, 1.09), (3.4, 1.09)], dtype=np.float32)
        row = self.step(runtime, 16)
        self.assertFalse(row["emergency_stop"], row["stop_reason"])
        self.assertGreater(row["lidar"]["timed_vehicle_points_n"], 0)
        # A point outside that measured vehicle still brakes us to a standstill behind
        # it; committed, that is the braking cap, not a stop rule (user rule 2026-10-10).
        runtime.lidar.pts = np.asarray([(2.5, 0.), (3.3, .2)], dtype=np.float32)
        row = self.step(runtime, 17)
        self.assertEqual(row["published_cap_kmh"], 0.)
        self.assertFalse(row["emergency_stop"])
        self.assertEqual(row["yield"]["name"], "obstacle_on_path")

    def test_unmeasured_car_returns_keep_static_stop(self):
        runtime = self.runtime()
        for step in range(16):
            self.step(runtime, step)
        runtime.test_cars = [self.car(runtime.test_status["ego"]["x"]+4., 2.,
                                     vy=5., heading=90., velocity_valid=False)]
        runtime.lidar.pts = np.asarray([(3.3, .2), (3.4, .3)], dtype=np.float32)
        row = self.step(runtime, 16)
        self.assertEqual(row["lidar"]["timed_vehicle_points_n"], 0)
        self.assertEqual(row["published_cap_kmh"], 0.)        # braked to a standstill behind it

    def test_committed_entry_follows_with_the_entry_planner_not_the_acc_law(self):
        # follow-20261010T130317Z: the normal-road lead-follow cap held 11.6-15.3 km/h behind
        # the ring car we followed in while the entry planner allowed 18-22 km/h.
        runtime = self.runtime()
        for step in range(16):
            self.step(runtime, step)
        self.assertTrue(runtime.gap_planner.junction_committed)
        ego_x = runtime.test_status["ego"]["x"]
        runtime.test_cars = [self.car(ego_x + 10., 0., vx=3.)]
        row = self.step(runtime, 16)
        self.assertFalse(row["emergency_stop"])
        self.assertFalse(row["lead"]["cap_applied"])
        self.assertGreater(row["published_cap_kmh"], row["lead"]["cap_kmh"] + 3.)
        self.assertEqual(row["traffic_plan"]["reason"], "junction committed: follow the car ahead")

    def test_released_entry_publishes_the_full_entry_cap_at_once(self):
        # User rule 2026-10-10: launch firmly behind the passing ring car - no slow
        # wall-clock target ramp (the follower's own ramp and the car's response remain,
        # ring_entry.LAUNCH_* model them).
        runtime = self.runtime()
        for step in range(16):
            row = self.step(runtime, step)
        self.assertEqual(row["stop_line"]["state"], "ENTER")
        self.assertGreater(runtime.path_publisher._last_target_mps, 5.)
        self.assertEqual(INTERSECTION_ACCEL_MPS2, 3.)


if __name__ == "__main__":
    unittest.main()
