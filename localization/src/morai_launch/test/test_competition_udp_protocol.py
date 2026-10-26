#!/usr/bin/env python3

import os
import ctypes
import math
import struct
import sys
import unittest


PACKAGE_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "src"))
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from morai_launch.competition_udp_protocol import (  # noqa: E402
    COLLISION_DATA_LENGTH,
    COLLISION_HEADER,
    COLLISION_PACKET_SIZE,
    PacketError,
    STATUS_DATA_LENGTH,
    STATUS_HEADER,
    STATUS_PACKET_SIZE,
    TAIL,
    pack_ctrl_cmd,
    normalize_ctrl_steer,
    parse_collision_data,
    parse_ctrl_cmd,
    parse_vehicle_status,
    speed_kmh_to_mps,
    vehicle_status_si_values,
    control_watchdog_action,
)


class CompetitionUdpProtocolTest(unittest.TestCase):
    def test_control_watchdog_brakes_then_releases_external_mode(self):
        decide = control_watchdog_action
        self.assertEqual(decide(False, 0.0, None, 0.5, 0.25, 1.0), "wait")
        self.assertEqual(decide(False, 0.0, 0.0, 0.5, 0.25, 1.0), "command")
        self.assertEqual(decide(True, 0.0, 0.4, 0.5, 0.25, 1.0), "brake")
        self.assertEqual(decide(True, 0.0, 1.1, 0.5, 0.25, 1.0), "release")

    def test_ctrl_packet_is_byte_identical_to_official_ctypes_layout(self):
        class OfficialEgoCtrlCmd(ctypes.Structure):
            _pack_ = 1
            _fields_ = [
                ("header", ctypes.c_char * 14),
                ("data_length", ctypes.c_int),
                ("aux_data", ctypes.c_int * 3),
                ("ctrl_mode", ctypes.c_int8),
                ("gear", ctypes.c_int8),
                ("cmd_type", ctypes.c_int8),
                ("velocity", ctypes.c_float),
                ("acceleration", ctypes.c_float),
                ("accel", ctypes.c_float),
                ("brake", ctypes.c_float),
                ("steer", ctypes.c_float),
                ("tail", ctypes.c_char * 2),
            ]

        official = OfficialEgoCtrlCmd()
        official.header = b"#MoraiCtrlCmd$"
        official.data_length = 23
        official.aux_data = (0, 0, 0)
        official.ctrl_mode = 2
        official.gear = 4
        official.cmd_type = 1
        official.velocity = 0.0
        official.acceleration = 0.0
        official.accel = 0.25
        official.brake = 0.5
        official.steer = -1.0
        official.tail = b"\r\n"

        packet = pack_ctrl_cmd(0.25, 0.5, -1.0)
        self.assertEqual(len(packet), ctypes.sizeof(official))
        self.assertEqual(packet, bytes(official))

    def test_ros_steering_angle_is_normalized_for_udp(self):
        maximum = math.radians(35.0)
        self.assertAlmostEqual(normalize_ctrl_steer(0.0, maximum), 0.0)
        self.assertAlmostEqual(
            normalize_ctrl_steer(math.radians(17.5), maximum), 0.5)
        self.assertAlmostEqual(
            normalize_ctrl_steer(math.radians(-35.0), maximum), -1.0)
        self.assertAlmostEqual(
            normalize_ctrl_steer(math.radians(70.0), maximum), 1.0)
        with self.assertRaises(PacketError):
            normalize_ctrl_steer(0.0, 0.0)

    def test_ctrl_cmd_is_competition_format_and_clamped(self):
        packet = pack_ctrl_cmd(accel=1.4, brake=-0.2, steer=-1.5)
        self.assertEqual(len(packet), 55)
        decoded = parse_ctrl_cmd(packet)
        self.assertEqual(decoded["ctrl_mode"], 2)
        self.assertEqual(decoded["gear"], 4)
        self.assertEqual(decoded["cmd_type"], 1)
        self.assertAlmostEqual(decoded["accel"], 1.0)
        self.assertAlmostEqual(decoded["brake"], 0.0)
        self.assertAlmostEqual(decoded["steer"], -1.0)

    def test_ctrl_cmd_supports_allowed_reverse_gear(self):
        packet = pack_ctrl_cmd(
            accel=0.2, brake=0.0, steer=0.1,
            ctrl_mode=2, gear=2, cmd_type=1)
        decoded = parse_ctrl_cmd(packet)
        self.assertEqual(decoded["ctrl_mode"], 2)
        self.assertEqual(decoded["gear"], 2)
        self.assertEqual(decoded["cmd_type"], 1)

    def test_ctrl_cmd_rejects_noncompetition_control_fields(self):
        with self.assertRaises(PacketError):
            pack_ctrl_cmd(0.0, 1.0, 0.0, ctrl_mode=3)
        with self.assertRaises(PacketError):
            pack_ctrl_cmd(0.0, 1.0, 0.0, gear=3)
        with self.assertRaises(PacketError):
            pack_ctrl_cmd(0.0, 1.0, 0.0, cmd_type=2)

    def test_vehicle_status_layout(self):
        packet = bytearray(STATUS_PACKET_SIZE)
        struct.pack_into(
            "<11sIiiiIIBBfi", packet, 0,
            STATUS_HEADER, STATUS_DATA_LENGTH, 0, 0, 0,
            10, 20, 2, 4, 12.5, 7)
        offset = struct.calcsize("<11sIiiiIIBBfi")
        values = [
            0.3, 0.1,
            4.8, 1.9, 1.6, 0.9, 3.5, 1.1,
            101.0, 202.0, 3.0,
            1.0, 2.0, 90.0,
            12.5, -0.2, 0.0,
            0.01, 0.02, 0.03,
            0.4, 0.5, 0.6,
            -13.6,
        ]
        struct.pack_into("<24f", packet, offset, *values)
        offset += struct.calcsize("<24f")
        struct.pack_into("<38s", packet, offset, b"link-123")
        packet[-2:] = TAIL

        decoded = parse_vehicle_status(bytes(packet))
        self.assertEqual(decoded["sec"], 10)
        self.assertEqual(decoded["ctrl_mode"], 2)
        self.assertAlmostEqual(decoded["signed_velocity_kmh"], 12.5)
        self.assertAlmostEqual(
            speed_kmh_to_mps(decoded["signed_velocity_kmh"]),
            12.5 / 3.6)
        self.assertNotIn("position", decoded)
        self.assertNotIn("velocity_kmh", decoded)
        self.assertNotIn("acceleration", decoded)
        self.assertNotIn("tire_data", decoded)
        self.assertAlmostEqual(decoded["angular_velocity"][2], 0.03)
        self.assertAlmostEqual(decoded["steer_deg"], -13.6, places=4)
        self.assertEqual(decoded["link_id"], "link-123")
        self.assertEqual(decoded["packet_size"], 181)

    def test_rejects_noncompetition_extended_vehicle_status(self):
        packet = bytearray(229)
        struct.pack_into("<11sI", packet, 0, STATUS_HEADER, 200)
        packet[-2:] = TAIL
        with self.assertRaises(PacketError):
            parse_vehicle_status(bytes(packet))

    def test_vehicle_speed_conversion_is_explicit_si_boundary(self):
        self.assertAlmostEqual(speed_kmh_to_mps(36.0), 10.0)
        self.assertAlmostEqual(speed_kmh_to_mps(-18.0), -5.0)
        with self.assertRaises(PacketError):
            speed_kmh_to_mps(float("nan"))

        values = vehicle_status_si_values({
            "signed_velocity_kmh": 36.0,
            "steer_deg": -35.0,
        })
        self.assertEqual(values["signed_velocity_mps"], 10.0)
        self.assertEqual(values["steer_deg"], -35.0)

    def test_collision_omits_zero_padding(self):
        packet = bytearray(COLLISION_PACKET_SIZE)
        struct.pack_into(
            "<15sIiiiII", packet, 0,
            COLLISION_HEADER, COLLISION_DATA_LENGTH, 0, 0, 0, 11, 22)
        offset = struct.calcsize("<15sIiiiII")
        struct.pack_into(
            "<hhffffff", packet, offset,
            1, 42, 1.0, 2.0, 3.0, 100.0, 200.0, 0.0)
        packet[-2:] = TAIL

        decoded = parse_collision_data(bytes(packet))
        self.assertEqual(len(decoded["records"]), 1)
        self.assertEqual(decoded["records"][0]["object_id"], 42)
        self.assertEqual(decoded["records"][0]["position"], (1.0, 2.0, 3.0))

    def test_competition_ego_placeholder_is_not_a_collision(self):
        packet = bytearray(COLLISION_PACKET_SIZE)
        struct.pack_into("<15sIiiiII", packet, 0,
                         COLLISION_HEADER, COLLISION_DATA_LENGTH, 0, 0, 0, 11, 22)
        offset = struct.calcsize("<15sIiiiII")
        struct.pack_into("<hhffffff", packet, offset,
                         -1, 0, -143.228, -449.397, 28.855, 0., 0., 0.)
        packet[-2:] = TAIL
        self.assertEqual(parse_collision_data(bytes(packet))["records"], [])
        struct.pack_into("<hhffffff", packet, offset + 28,
                         1, 38, -97.752, 326.338, 28.693, 0., 0., 0.)
        self.assertEqual([r["object_id"] for r in parse_collision_data(bytes(packet))["records"]], [38])

    def test_rejects_wrong_length_header_and_tail(self):
        with self.assertRaises(PacketError):
            parse_vehicle_status(b"short")

        packet = bytearray(STATUS_PACKET_SIZE)
        struct.pack_into("<11sI", packet, 0, b"#WrongInfo$", 152)
        packet[-2:] = TAIL
        with self.assertRaises(PacketError):
            parse_vehicle_status(bytes(packet))

        mismatched_extended = bytearray(229)
        struct.pack_into(
            "<11sI", mismatched_extended, 0,
            STATUS_HEADER, STATUS_DATA_LENGTH)
        mismatched_extended[-2:] = TAIL
        with self.assertRaises(PacketError):
            parse_vehicle_status(bytes(mismatched_extended))

        ctrl = bytearray(pack_ctrl_cmd(0.0, 1.0, 0.0))
        ctrl[-2:] = b"xx"
        with self.assertRaises(PacketError):
            parse_ctrl_cmd(bytes(ctrl))


if __name__ == "__main__":
    unittest.main()
