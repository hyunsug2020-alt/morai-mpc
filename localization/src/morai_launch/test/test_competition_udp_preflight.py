#!/usr/bin/env python3

import importlib.util
import os
import unittest


SCRIPT_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "scripts", "competition_udp_preflight.py"))
SPEC = importlib.util.spec_from_file_location(
    "competition_udp_preflight", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CompetitionUdpPreflightTest(unittest.TestCase):
    def test_camera_image_packet_is_allowed(self):
        self.assertEqual(MODULE.classify_camera_packet(b"MOR\x00image"),
                         "image")

    def test_camera_ground_truth_packet_is_identified_for_runtime_discard(self):
        self.assertEqual(MODULE.classify_camera_packet(b"BOX\x00labels"),
                         "ground_truth")

    def test_unknown_camera_packet_is_not_accepted(self):
        self.assertEqual(MODULE.classify_camera_packet(b"bad"), "unknown")


if __name__ == "__main__":
    unittest.main()
