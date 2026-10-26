"""Run-428 duplicate semantics, motion dropout and pedestrian waiting contracts."""
import time
import unittest
from unittest.mock import patch

import numpy as np

from gui import PEDESTRIAN_WAIT_S, Runtime, deduplicate_obstacles, route_hazard
from lattice_planner import plan
from lidar_corridor import PedestrianMotionHistory, without_moving_objects


def person(x=10.4, y=-1.6, speed=.89, uid=1):
    return (x, y, .6, "fusion_pedestrian", "pedestrian", speed,
            .6, .6, 90., 0., speed, uid)


def obstacle(x=10.5, y=-1.7, length=1.42, width=.6, heading=45.):
    return (x, y, max(length, width), "fusion_obstacle", "obstacle", 0., length, width, heading)


class ObjectDedupeTests(unittest.TestCase):
    def setUp(self):
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=30.)
        self.route = np.column_stack((np.arange(0., 80.5, .5), np.zeros(161)))

    def test_run428_t96_person_removes_obstacle_and_preserves_neighbor(self):
        # The log contains ext=0.71, not full dimensions. This oriented compact
        # footprint has the same lateral extent; unknown shape is tested below.
        rows = [obstacle(3.6, -4.1), person(), obstacle()]
        self.assertEqual(deduplicate_obstacles(rows), rows[:2])
        self.assertEqual(len(rows), 3)
        before = route_hazard(rows, self.ego, self.route, max_speed_mps=1.5)
        after = route_hazard(deduplicate_obstacles(rows), self.ego, self.route, max_speed_mps=1.5)
        self.assertEqual(before[3], "obstacle")
        self.assertEqual(after[3], "pedestrian")

    def test_boundary_vehicle_overlap_and_independent_obstacle(self):
        tracked = (0., 0., 2., "car", "npc", 0., 4., 2., 0.)
        boundary, separate = obstacle(1.5, 0.), obstacle(1.5001, 0.)
        self.assertEqual(deduplicate_obstacles([boundary, separate, tracked]), [separate, tracked])
        self.assertEqual(deduplicate_obstacles([boundary]), [boundary])

    def test_duplicate_static_memory_is_purged_when_person_appears(self):
        runtime = Runtime.__new__(Runtime)
        runtime._static_object_memory = [(10., obstacle())]
        self.assertEqual(runtime.remember_static_objects([], 10.1, True, [person()]), [])
        self.assertEqual(runtime._static_object_memory, [])

    def test_independent_box_beside_person_is_preserved_in_detection_and_memory(self):
        p = person(x=10., y=2., speed=1.)
        box = obstacle(x=10., y=.6, length=1., width=1., heading=0.)
        self.assertEqual(deduplicate_obstacles([p, box]), [p, box])
        runtime = Runtime.__new__(Runtime)
        runtime._static_object_memory = [(10., box)]
        self.assertEqual(runtime.remember_static_objects([], 10.1, True, [p]), [box])
        hazard = route_hazard([box], self.ego, self.route)
        self.assertEqual(hazard[3], "obstacle")

    def test_large_box_is_preserved_even_with_near_identical_centre(self):
        p = person()
        for length, width in ((1.42, 1.42), (2., .6), (1.3, 1.3)):
            with self.subTest(length=length, width=width):
                box = obstacle(length=length, width=width)
                self.assertEqual(deduplicate_obstacles([p, box]), [p, box])

    def test_compact_duplicate_needs_close_centre_or_oriented_overlap(self):
        p = person(x=10., y=0., speed=1.)
        overlap = obstacle(x=10., y=.9, length=1.6, width=.6, heading=90.)
        separate = obstacle(x=10., y=.9, length=1.6, width=.6, heading=0.)
        close = obstacle(x=10.7, y=0., length=.3, width=.3, heading=0.)
        self.assertEqual(deduplicate_obstacles([p, overlap, separate, close]), [p, separate])

    def test_missing_or_invalid_shape_keeps_obstacle(self):
        p = person()
        for box in (obstacle()[:6], obstacle(width=float('nan')), obstacle(width=0.)):
            with self.subTest(box=box):
                self.assertEqual(deduplicate_obstacles([p, box]), [p, box])

    def test_wider_oriented_person_mask_survives_zero_speed_for_two_seconds(self):
        history = PedestrianMotionHistory()
        moving = person(x=10., y=-2.)      # walked 2 m in 2 s (displacement, run 440)
        standing = person(x=10., y=0., speed=0.)
        points = np.array([[11.2, 0.], [10., 1.2], [11.31, 0.], [10., 1.31], [20., 0.]])
        history.update([moving], 10.)
        history.update([standing], 12.)
        np.testing.assert_array_equal(without_moving_objects(points, [standing], self.ego, history), points[2:])
        history.update([standing], 12.01)
        np.testing.assert_array_equal(without_moving_objects(points, [standing], self.ego, history), points)

    def test_vehicle_mask_keeps_original_padding(self):
        car = (10., 0., 4., "car", "npc", 1., 4., 2., 0.)
        points = np.array([[12.3, 1.3], [12.41, 0.], [10., 1.36]])
        np.testing.assert_array_equal(without_moving_objects(points, [car], self.ego), points[1:])

    def test_track_isolation_clock_reset_and_id_reuse(self):
        history = PedestrianMotionHistory()
        history.update([person(), person(x=20., speed=0., uid=2)], 10.)
        self.assertEqual(history.max_speed(person(uid=2, speed=0.)), 0.)
        history.update([person(x=30., speed=0.)], 10.1)
        self.assertEqual(history.max_speed(person(x=30., speed=0.)), 0.)
        history.update([person(speed=1.)], 11.)
        history.update([person(speed=0.)], 9.)
        self.assertEqual(history.max_speed(person(speed=0.)), 0.)

    def test_only_continuously_standing_person_is_bypass_target_after_wait(self):
        runtime = Runtime.__new__(Runtime)
        runtime._pedestrian_wait_since = 10.
        p = person(y=0., speed=0.)
        W = PEDESTRIAN_WAIT_S
        for t in np.arange(10., 10. + W + .01, 1.):
            runtime.deduplicate_objects([p], float(t))
            if t < 10. + W:
                self.assertFalse(runtime.person_blocks(p, self.ego, self.route, t))
        self.assertTrue(runtime.pedestrian_wait_expired(10. + W))
        self.assertTrue(runtime.person_blocks(p, self.ego, self.route, 10. + W))
        runtime.deduplicate_objects([person(y=.6, speed=1.)], 10. + W + .1)   # really moved 0.6 m
        runtime.deduplicate_objects([p], 10. + W + .2)
        self.assertFalse(runtime.person_blocks(p, self.ego, self.route, 10. + W + .2))

    def test_stander_speed_jitter_and_long_wait_still_allow_bypass(self):
        # Run 433: refined speed flickered to 0.4-0.6 m/s; after twice the wait
        # with the car stopped the person is passed regardless of the timer.
        runtime = Runtime.__new__(Runtime)
        runtime._pedestrian_wait_since = 10.
        W = PEDESTRIAN_WAIT_S
        for t in np.arange(10., 10. + W + .01, 1.):
            runtime.deduplicate_objects([person(y=0., speed=.45 if int(t) % 5 == 0 else 0.)], float(t))
        self.assertTrue(runtime.person_blocks(person(y=0., speed=0.), self.ego, self.route, 10. + W))
        runtime = Runtime.__new__(Runtime)
        runtime._pedestrian_wait_since = 10.
        for t in np.arange(10., 10. + 2 * W + .01, 1.):
            runtime.deduplicate_objects([person(y=0., speed=.6 if int(t) % 3 == 0 else 0.)], float(t))
        self.assertTrue(runtime.person_blocks(person(y=0., speed=0.), self.ego, self.route, 10. + 2 * W))

    def test_raw_speed_noise_does_not_block_bypass_after_refined_standing_15_seconds(self):
        from test_avoidance_gate import AvoidanceGateTests
        raw = person(x=10., y=0., speed=.37)
        runtime = AvoidanceGateTests().runtime(objects=[raw])
        runtime.test_status["path"]["nearest_idx"] = 900
        runtime.test_status["ego"]["speed_kmh"] = 0.
        runtime._pedestrian_wait_since = 10.
        # Exercise both real inference snapshots: each repeats the same raw
        # measurement, while PedTracker confirms fixed positions as standing.
        for t in np.arange(10., 27.01, .25):
            with patch("gui.time.monotonic", return_value=float(t)), patch("gui.USE_LATTICE", True):
                runtime.object_receiver.received = float(t)
                runtime.lidar.received = float(t)
                self.assertTrue(runtime.infer())
        refined = runtime.refine_pedestrians([raw], self.ego)[0]
        self.assertEqual(refined[5], 0.)
        self.assertEqual(runtime._ped_motion_history.max_speed(refined), 0.)
        self.assertTrue(runtime.pedestrian_wait_expired(27.))
        self.assertTrue(runtime.person_blocks(refined, self.ego, self.route, 27.))
        self.assertNotEqual(runtime._web_record["lattice"]["kind"], "pedestrian_wait")

    def test_wait_discards_old_bypass_model_swerve_and_still_brakes_for_real_box(self):
        previous = dict(target=-3.5, world_path=self.route + [0., -3.5])
        result = plan(self.route, self.ego, None, [obstacle(20., 0.)], None,
                      previous=previous, model_path=self.route + [0., 3.5], wait_for_pedestrian=True)
        self.assertEqual(result["target"], 0.)
        self.assertEqual(result["kind"], "pedestrian_wait")
        self.assertTrue(result["blocked"])
        self.assertLess(result["speed_cap_mps"], 10.)

    def test_empty_wait_path_retains_cruise_cap_for_pedestrian_guard(self):
        result = plan(self.route, self.ego, None, [], None, wait_for_pedestrian=True, cruise_mps=12.)
        self.assertEqual(result["target"], 0.)
        self.assertEqual(result["kind"], "pedestrian_wait")
        self.assertFalse(result["blocked"])
        self.assertEqual(result["speed_cap_mps"], 12.)

    def test_runtime_duplicate_and_lidar_dropout_keep_lane_centre_without_hazard(self):
        from test_avoidance_gate import AvoidanceGateTests
        moving = person(x=12., y=0.)
        runtime = AvoidanceGateTests().runtime(objects=[moving, obstacle(12.1, -.1)],
                                               points=[(12., 0.), (13.2, 0.), (12., 1.2)])
        runtime.test_status["path"]["nearest_idx"] = 900
        for p in (moving, person(x=12., y=0., speed=0.)):
            runtime.object_receiver.objects = [p, obstacle(12.1, -.1)]
            with patch("gui.USE_LATTICE", True):
                self.assertTrue(runtime.infer())
            record = runtime._web_record
            self.assertEqual(record["lattice"]["target"], 0.)
            self.assertIn(record["lattice"]["kind"], ("lattice", "previous"))
            # The crossing itself can require emergency braking; duplicates must
            # still produce neither a static bypass nor an obstacle hazard.
            self.assertTrue(record["emergency_stop"])
            self.assertEqual(record["ped_guard"]["kind"], "stop")
            self.assertIn("stopping envelope predicts contact", record["stop_reason"])
            self.assertIsNone(record["hazard"])
            self.assertFalse(any(o["k"] == "obstacle" for o in record["near"]))


if __name__ == "__main__":
    unittest.main()


class GhostVehicleTests(unittest.TestCase):
    def test_parked_vehicle_box_beside_walker_is_dropped_but_far_one_kept(self):
        p = person(x=10., y=0., speed=.4)
        ghost = (10.8, 2.1, 4.6, "fusion_parked_vehicle", "obstacle", 0., 4.6, 1.9, 90.)
        real = (10.8, 2.6, 4.6, "fusion_parked_vehicle", "obstacle", 0., 4.6, 1.9, 90.)
        self.assertEqual(deduplicate_obstacles([p, ghost]), [p])
        self.assertEqual(deduplicate_obstacles([p, real]), [p, real])
        box = (10.8, 2.1, 1.0, "fusion_obstacle", "obstacle", 0., 1.0, 1.0, 0.)
        self.assertIn(box, deduplicate_obstacles([p, box]))


if __name__ == "__main__":
    unittest.main()
