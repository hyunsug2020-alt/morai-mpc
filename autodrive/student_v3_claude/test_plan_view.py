"""Read-only decision visualization, including actual rotated vehicle footprints."""
import unittest

import numpy as np

from gui import Runtime
from lane_corridor import LaneCorridor
from plan_view import CHECKED, RISK, object_polygon, render_plan_view
import test_avoidance_gate as avoidance_fixture


class PlanViewTests(unittest.TestCase):
    def test_lidar_car_box_axis_is_preserved_when_velocity_points_the_other_way(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        car = (15., 3., 4., 'lidar_car', 'npc', 8., 4., 2., 90., 8., 0., 42, 0., None, True)
        box = object_polygon(car, ego)
        self.assertAlmostEqual(float(np.ptp(box[:, 0])), 2.)
        self.assertAlmostEqual(float(np.ptp(box[:, 1])), 4.)

    def test_actual_risk_and_checked_path_render_without_mutating_the_decision(self):
        x = np.arange(0., 65.5, .5)
        path = np.column_stack((x, np.full(len(x), 3.)))
        route = np.column_stack((x, np.zeros(len(x))))
        result = dict(path=path, local_route=route, raw_path=route)
        record = dict(action='KEEP_LANE', motion_state='DRIVE', published_cap_kmh=20.,
                      static_clear_distance_m=40., traffic_risks=[dict(object_id=42,
                          when=2., object_xy=[15., 3.], ego_xy=[15., 3.])])
        ego = dict(x=0., y=0., heading_deg=0.)
        car = (15., 3., 4., 'lidar_car', 'npc', 8., 4., 2., 90., 8., 0., 42, 0., None, True)
        before = path.copy()
        canvas = np.zeros((550, 800, 3), np.uint8)
        corridor = LaneCorridor(route, np.full(len(x), 5.), np.full(len(x), -5.))
        render_plan_view(canvas, (0, 0), (800, 550), result=result, record=record,
                         ego=ego, objects=[car], corridor=corridor)
        self.assertGreater(np.count_nonzero(np.all(canvas == CHECKED, axis=2)), 10)
        self.assertGreater(np.count_nonzero(np.all(canvas == RISK, axis=2)), 10)
        np.testing.assert_array_equal(path, before)
        self.assertEqual(record['published_cap_kmh'], 20.)

    def test_runtime_draw_uses_checked_snapshot_without_publishing_or_reading_new_objects(self):
        runtime = avoidance_fixture.AvoidanceGateTests().runtime(points=[(10., 4.), (12., -5.)])
        self.assertTrue(runtime.infer())
        publisher = runtime.path_publisher
        count = len(publisher.velocity_pub.items)
        runtime.object_receiver.snapshot = lambda: (_ for _ in ()).throw(AssertionError('A new object frame was read'))
        canvas = np.zeros((800, 1280, 3), np.uint8)
        Runtime.draw_plan(runtime, canvas, (0, 250), (800, 550))
        self.assertEqual(len(publisher.velocity_pub.items), count)
        self.assertGreater(np.count_nonzero(canvas[250:800, :800]), 1000)

    def test_missing_first_frame_remains_a_valid_waiting_view(self):
        canvas = np.zeros((550, 800, 3), np.uint8)
        render_plan_view(canvas, (0, 0), (800, 550), result=None, record={})
        self.assertGreater(np.count_nonzero(canvas), 1000)


if __name__ == '__main__':
    unittest.main()
