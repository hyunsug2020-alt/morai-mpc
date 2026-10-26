import unittest
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from tunnel_runtime import TunnelRuntime, swept_clearance
from lattice_planner import object_points


def car(x=12., y=0., velocity_valid=False):
    return (x,y,4.8,'lidar_car','npc',0.,4.8,1.9,0.,0.,0.,9,0.,None,velocity_valid,0.)


class TunnelVehicleTests(unittest.TestCase):
    def test_parallel_semantic_car_does_not_occupy_the_empty_lane(self):
        runtime = self.runtime()
        runtime.object_receiver = SimpleNamespace(received=time.monotonic(),
            snapshot=lambda: [(2., -3.3, 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 0.)])
        ego = dict(x=0., y=0., heading_deg=0.)
        points, count = runtime.semantic_obstacle_points(ego)
        self.assertEqual(count, 1)
        x = np.arange(0., 30., .25)
        path = np.column_stack((x, np.zeros(len(x))))
        self.assertGreaterEqual(swept_clearance(path, points, 25., lateral_margin=.3), 25.)
        # A real turn towards its full body remains blocked.
        towards = np.column_stack((x, -.3*x))
        self.assertLess(swept_clearance(towards, points, 25., lateral_margin=.3), 5.)

    def test_crosswise_semantic_car_keeps_its_full_rotated_body(self):
        runtime = self.runtime()
        runtime.object_receiver = SimpleNamespace(received=time.monotonic(),
            snapshot=lambda: [(12., 2., 4.6, 'fusion_parked_vehicle', 'obstacle', 0., 4.6, 1.9, 90.)])
        ego = dict(x=0., y=0., heading_deg=0.)
        points, count = runtime.semantic_obstacle_points(ego)
        path = np.column_stack((np.arange(0., 30., .25), np.zeros(120)))
        self.assertEqual(count, 1)
        self.assertLess(swept_clearance(path, points, 25., lateral_margin=.3), 12.)

    def runtime(self, index=3700, ready=True):
        runtime=TunnelRuntime()
        runtime.highway=SimpleNamespace(config=dict(lidar_regions=[dict(from_idx=3500,to_idx=4080)]))
        runtime.latest_status=dict(path=dict(nearest_idx=index))
        runtime.vehicle_receiver=SimpleNamespace(snapshot=Mock(return_value=([car()],ready,{})))
        return runtime

    def test_approach_adds_unknown_velocity_vehicle_body_and_preserves_nonvehicles(self):
        runtime=self.runtime()
        obstacle=(20.,3.,.5,'box','obstacle',0.)
        camera_car=(12.2,.2,2.,'fusion_parked_vehicle','obstacle',0.)
        result=runtime.tunnel_vehicle_objects([obstacle,camera_car],{})
        self.assertEqual(result,[obstacle,car()])
        self.assertFalse(result[-1][14])

    def test_stale_stream_cannot_replace_existing_camera_observations(self):
        original=[car(x=20.)]
        self.assertEqual(self.runtime(ready=False).tunnel_vehicle_objects(original,{}),original)

    def test_highway_and_roundabout_keep_their_existing_object_handling(self):
        for index in (1800,2500,4081):
            runtime=self.runtime(index)
            original=[car(x=20.)]
            self.assertEqual(runtime.tunnel_vehicle_objects(original,{}),original)
            runtime.vehicle_receiver.snapshot.assert_not_called()

    def test_recognized_body_prevents_a_path_through_an_unobserved_vehicle_side(self):
        ego=dict(x=0.,y=0.,heading_deg=0.)
        x=np.arange(0.,30.1,.25)
        path=np.column_stack((x,np.full(len(x),1.6)))
        # A sparse far-side return misses the proposed near-side pass.
        self.assertGreaterEqual(swept_clearance(path,np.array([[12.,-.95]]),25.),25.)
        body=object_points([car()],ego)
        self.assertLess(swept_clearance(path,body,25.),10.)


if __name__ == '__main__':
    unittest.main()
