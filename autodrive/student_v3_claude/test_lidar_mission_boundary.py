"""Keep measured vehicle perception ready during the highway controller handoff."""
import threading
import unittest
from unittest.mock import patch

from lidar_vehicle_detection import VehicleReceiver, detector_mission_active


class LidarMissionBoundaryTests(unittest.TestCase):
    mission = dict(from_idx=2251, to_idx=3500)

    def test_newer_detector_pose_remains_active_until_gui_has_left_highway(self):
        # Recorded run: GUI nearest=3496 while the detector's newer pose had
        # already passed 3500, publishing inactive and causing a zero target.
        receiver = VehicleReceiver.__new__(VehicleReceiver)
        receiver.lock = threading.Lock()
        receiver.received = 100.
        receiver.record = dict(active=detector_mission_active(3504, self.mission),
                               ready=True, vehicles=[])
        with patch('lidar_vehicle_detection.time.monotonic', return_value=100.01):
            cars, ready, status = receiver.snapshot()
        self.assertEqual(cars, [])
        self.assertTrue(ready)
        self.assertTrue(status['active'])

    def test_warmup_and_exit_padding_have_finite_bounds(self):
        for index in (2051, 2251, 3500, 3504, 3600):
            self.assertTrue(detector_mission_active(index, self.mission), index)
        for index in (2050, 3601):
            self.assertFalse(detector_mission_active(index, self.mission), index)

    def test_boundary_padding_does_not_override_stale_stream_stop(self):
        receiver = VehicleReceiver.__new__(VehicleReceiver)
        receiver.lock = threading.Lock()
        receiver.received = 100.
        receiver.record = dict(active=detector_mission_active(3504, self.mission),
                               ready=True, vehicles=[])
        with patch('lidar_vehicle_detection.time.monotonic', return_value=100.4):
            _, ready, _ = receiver.snapshot()
        self.assertFalse(ready)

    def test_tunnel_approach_keeps_detector_running_without_changing_highway_end(self):
        mission = dict(self.mission, lidar_regions=[dict(from_idx=3500, to_idx=4080)])
        for index in (3601, 3700, 3777, 4016, 4080):
            self.assertTrue(detector_mission_active(index, mission), index)
        self.assertFalse(detector_mission_active(4081, mission))
        self.assertEqual(mission['to_idx'], 3500)


if __name__ == '__main__':
    unittest.main()
