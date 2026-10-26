#!/usr/bin/env python3

import importlib.util
import os
import struct
import unittest
from unittest.mock import Mock, patch
import rospy


SCRIPT_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "scripts", "morai_imu_udp.py"))
SPEC = importlib.util.spec_from_file_location("morai_imu_udp", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MoraiImuUdpTest(unittest.TestCase):
    def test_arrival_clock_aligns_gps_and_keeps_wire_duplicate_filter(self):
        node = MODULE.MoraiImuUDP.__new__(MODULE.MoraiImuUDP)
        node.use_sim_time = False
        node.timestamp_source = 'arrival'
        node.last_sensor_stamp_ns = node.last_wire_stamp_ns = None
        node.pub = Mock()
        packet = (b'#IMUData$' + struct.pack('<I', 88) + bytes(12) +
                  struct.pack('<II', 999, 720000000) +
                  struct.pack('<10d', 1., 0., 0., 0., 0., 0., 0., 0., 0., 9.81) + b'\r\n')
        with patch.object(MODULE.rospy.Time, 'now', return_value=rospy.Time(1000)), \
                patch.object(MODULE.rospy, 'loginfo_throttle'), \
                patch.object(MODULE.rospy, 'logwarn_throttle'):
            node.parse_and_publish(packet)
            node.parse_and_publish(packet)
        self.assertEqual(node.pub.publish.call_count, 1)
        self.assertEqual(node.pub.publish.call_args[0][0].header.stamp, rospy.Time(1000))

    def test_two_pc_wire_clock_validation(self):
        usable = MODULE.wire_timestamp_is_usable
        self.assertTrue(usable(1000.0, 999, 500000000, 10.0))
        self.assertFalse(usable(1000.0, 900, 0, 10.0))
        self.assertFalse(usable(1000.0, 0, 0, 10.0))
        self.assertFalse(usable(1000.0, 999, 1000000000, 10.0))
        self.assertTrue(usable(1000.0, 900, 0, 0.0))


if __name__ == "__main__":
    unittest.main()
