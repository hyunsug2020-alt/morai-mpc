"""Far LiDAR returns must stay at one map position before they become stop lines.

Runs follow-20261009T185440Z (t=54-62 s) and 191227Z (t=68-71 s): 1-4 returns
86-95 m ahead slid along with the ego for seconds (an undetected car or ground
hits at constant range). Each frame they made a "static" stop line, so the car
braked at 1.5-2 m/s^2 again and again and could not pass 66 km/h.
"""
import math
import unittest

import numpy as np

from highway_planner import HighwayPlanner, local_points, world_points
from planning.highway.static_returns import FarReturnPersistence
from tunnel_runtime import swept_clearance


class FarReturnPersistenceTests(unittest.TestCase):
    @staticmethod
    def ego(t, speed_mps=18.5, heading_deg=-87.):
        h = math.radians(heading_deg)
        return dict(x=74.+speed_mps*t*math.cos(h), y=-100.+speed_mps*t*math.sin(h),
                    heading_deg=heading_deg, speed_kmh=speed_mps*3.6, stamp=1000.+t)

    def test_return_sliding_with_the_ego_never_becomes_static(self):
        persistence = FarReturnPersistence()
        for k in range(30):                      # 2 s of 15 Hz scans
            t = k/15.
            ego = self.ego(t)
            points = np.array([[88.+.5*math.sin(k), 1.7-.1*k/30.], [89., 1.4]])
            kept = persistence.update(points, ego, ego['stamp'])
            self.assertEqual(len(kept), 0, t)

    def test_fixed_far_obstacle_is_confirmed_after_two_scans(self):
        persistence = FarReturnPersistence()
        obstacle = np.array([[80., .3], [80.2, -.4]])
        world = world_points(obstacle, self.ego(0.))
        kept = []
        for k in range(4):
            t = k/15.
            ego = self.ego(t)
            kept.append(len(persistence.update(local_points(world, ego), ego, ego['stamp'])))
        # First seen at 1000.0; 1000.067 is still too soon, 1000.133 confirms both.
        self.assertEqual(kept, [0, 0, 2, 2])

    def test_near_side_and_rear_returns_pass_unchanged(self):
        persistence = FarReturnPersistence()
        points = np.array([[34.9, 0.], [10., 1.], [-40., 3.5], [2.5, -1.2]])
        for k in range(3):
            ego = self.ego(k/15.)
            np.testing.assert_array_equal(persistence.update(points, ego, ego['stamp']), points)

    def test_missing_or_invalid_stamp_fails_safe_unfiltered(self):
        persistence = FarReturnPersistence()
        points = np.array([[90., 0.]])
        for stamp in (None, float('nan'), 0., -1.):
            with self.subTest(stamp=stamp):
                np.testing.assert_array_equal(persistence.update(points, self.ego(0.), stamp), points)

    def test_reused_scan_does_not_confirm_itself(self):
        persistence = FarReturnPersistence()
        ego = self.ego(0.)
        points = np.array([[70., 0.]])
        for _ in range(5):
            self.assertEqual(len(persistence.update(points, ego, ego['stamp'])), 0)

    def test_older_scan_resets_history(self):
        persistence = FarReturnPersistence()
        world = world_points(np.array([[60., 0.]]), self.ego(0.))
        for t in (0., .2):
            ego = self.ego(t)
            persistence.update(local_points(world, ego), ego, ego['stamp'])
        ego = self.ego(.1)                       # clock went back: new history
        self.assertEqual(len(persistence.update(local_points(world, ego), ego, ego['stamp'])), 0)


class FarReturnBrakingTests(unittest.TestCase):
    """The filter output feeds the unchanged clearance and speed checks."""
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        self.planner = HighwayPlanner(dict(from_idx=0, to_idx=1800, kmh=120.,
            lane_change_policy="free_three_lane", lane_change_goal=0, end_link="exit",
            comfort_decel_mps2=1.2, comfort_jerk_mps3=2., emergency_jerk_mps3=12.), self.route)
        self.path = np.column_stack((np.arange(0., 200., .5), np.zeros(400)))

    def command(self, points, ego):
        clear = swept_clearance(self.path, points, 170., extra_margin=.25, lateral_margin=.15)
        self.planner.longitudinal.begin(ego)
        return clear, self.planner.plan_speed(self.path, ego, (), 120./3.6,
            static_clear_m=clear if clear < 170. else float('inf'))

    def test_recorded_phantom_at_88_m_no_longer_brakes_at_66_kmh(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=66.2, stamp=1000.)
        phantom = np.array([[88., .6], [88.4, .2], [88.8, -.3]])
        clear, before = self.command(phantom, ego)
        self.assertLess(clear, 90.)
        self.assertLess(before['reference_mps']*3.6, 66.2)      # recorded: goal 0, -2 m/s^2
        self.planner.reset()
        persistence = FarReturnPersistence()
        for k in range(15):
            frame = dict(ego, x=18.4*k/15., stamp=1000.+k/15.)
            kept = persistence.update(phantom, frame, frame['stamp'])
            clear, after = self.command(kept, dict(frame, x=0.))
            self.assertEqual(clear, 170.)
            self.assertGreater(after['reference_mps']*3.6, 66.2)

    def test_confirmed_static_return_keeps_the_stop_line(self):
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=66.2, stamp=1000.)
        persistence = FarReturnPersistence()
        world = np.array([[70., 0.], [70., .5]])
        for k in range(3):
            frame = dict(ego, x=18.4*k/15., stamp=1000.+k/15.)
            kept = persistence.update(local_points(world, frame), frame, frame['stamp'])
        self.assertEqual(len(kept), 2)
        clear, command = self.command(kept, dict(ego, stamp=frame['stamp']))
        self.assertLess(clear, 70.)
        self.assertLess(command['reference_mps']*3.6, 66.2)


if __name__ == '__main__':
    unittest.main()


class FarReturnRuntimeTests(unittest.TestCase):
    """infer_highway applies the filter to timestamped, pose-aligned scans."""
    @classmethod
    def setUpClass(cls):
        from test_highway_integration import HighwayIntegration
        HighwayIntegration.setUpClass()
        cls.integration = HighwayIntegration('test_three_lane_runtime_publishes_highway_mode_and_same_speed_reference')

    def run_scan(self, points, index=2611):
        from types import SimpleNamespace
        from unittest.mock import patch
        from gui import Runtime
        from lidar_corridor import LidarCorridor
        world, smooth = self.integration.world, self.integration.smooth
        x, y = world[index]
        yaw = math.degrees(math.atan2(*(smooth[index+3]-smooth[index])[::-1]))
        pose = dict(x=x, y=y, heading_deg=yaw, speed_kmh=66., stamp=100.)
        original = LidarCorridor.xy_snapshot
        def timestamped(receiver):
            return dict(original(receiver), stamp=100.)
        with patch.object(LidarCorridor, 'xy_snapshot', timestamped), \
                patch.object(Runtime, 'ego_receiver', SimpleNamespace(at_stamp=lambda stamp: dict(pose)),
                             create=True):
            return self.integration.run_branch(highway_policy='free_three_lane', points=points,
                                               speed_kmh=66., ego_stamp=100.05, index=index)

    def test_first_far_return_in_the_lane_is_not_a_stop_line(self):
        runtime, publisher = self.run_scan([[88., 0.], [88.3, .3]])
        info = runtime._web_record['highway']
        self.assertTrue(info['sensor_alignment']['scan_frame_aligned'])
        self.assertEqual(info['sensor_alignment']['far_returns']['dropped_n'], 2)
        self.assertEqual(info['clearance_components_m']['static'], 170.)
        self.assertFalse(runtime._web_record['emergency_stop'])
        self.assertGreater(publisher.velocity_pub.items[-1].data*3.6, 60.)

    def test_near_return_counts_on_the_first_frame(self):
        runtime, publisher = self.run_scan([[30., 0.], [30.3, .3]])
        info = runtime._web_record['highway']
        self.assertEqual(info['sensor_alignment']['far_returns']['dropped_n'], 0)
        self.assertLess(info['bounded_static_hit_m'], 31.)          # seen in the lane...
        self.assertEqual(info['reason'], 'highway checked escape: near-term conflict or static hit')

    def test_near_blockage_across_the_road_still_brakes_on_the_first_frame(self):
        wall = [[30., y] for y in np.arange(-8., 8.01, .5)]
        runtime, publisher = self.run_scan(wall)
        info = runtime._web_record['highway']
        self.assertEqual(info['sensor_alignment']['far_returns']['dropped_n'], 0)
        self.assertLess(info['clearance_components_m']['static'], 31.)
        self.assertLess(publisher.velocity_pub.items[-1].data*3.6, 66.)
