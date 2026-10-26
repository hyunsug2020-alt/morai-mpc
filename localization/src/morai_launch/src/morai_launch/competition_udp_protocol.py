#!/usr/bin/env python3
"""MORAI competition UDP packet encoder/decoders.

Only the competition-permitted packets are implemented:

* Ego Ctrl Cmd (User -> Sim)
* Competition Vehicle Status / MoraiInfo (Sim -> User)
* CollisionData (Sim -> User)

MORAI packets are little-endian and use one-byte packing.  Keeping this module
free of ROS imports makes the wire format independently testable.
"""

import math
import struct


CTRL_HEADER = b"#MoraiCtrlCmd$"
STATUS_HEADER = b"#MoraiInfo$"
COLLISION_HEADER = b"#CollisionData$"
TAIL = b"\r\n"

CTRL_PACKET_SIZE = 55
STATUS_PACKET_SIZE = 181
COLLISION_PACKET_SIZE = 181

CTRL_DATA_LENGTH = 23
STATUS_DATA_LENGTH = 152
COLLISION_DATA_LENGTH = 148

_CTRL = struct.Struct("<14sIiiiBBBfffff2s")
_STATUS_PREFIX = struct.Struct("<11sIiiiIIBBfi")
_COLLISION_PREFIX = struct.Struct("<15sIiiiII")
_COLLISION_RECORD = struct.Struct("<hhffffff")


class PacketError(ValueError):
    """Raised for a malformed or unsupported MORAI UDP packet."""


def _finite(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise PacketError("{} must be finite".format(name))
    return result


def _clamp(value, minimum, maximum, name):
    return min(max(_finite(value, name), minimum), maximum)


def normalize_ctrl_steer(steer_rad, max_steer_rad):
    """Convert the ROS steering angle [rad] to MORAI UDP's [-1, 1]."""
    maximum = _finite(max_steer_rad, "max_steer_rad")
    if maximum <= 0.0:
        raise PacketError("max_steer_rad must be positive")
    return _clamp(
        _finite(steer_rad, "steer_rad") / maximum,
        -1.0, 1.0, "normalized_steer")


def speed_kmh_to_mps(speed_kmh):
    """Convert a MORAI UDP vehicle speed from km/h to ROS SI units."""
    return _finite(speed_kmh, "speed_kmh") / 3.6


def vehicle_status_si_values(status):
    """Return only competition-provided kinematics in ROS units."""
    return {
        "signed_velocity_mps": speed_kmh_to_mps(
            status["signed_velocity_kmh"]),
        # ROS EgoVehicleStatus variants represent this field in degrees.
        "steer_deg": _finite(status["steer_deg"], "steer_deg"),
    }


def control_watchdog_action(
        started, status_age_s, command_age_s,
        status_timeout_s, command_timeout_s, release_timeout_s):
    """Choose command, temporary brake, wait, or UDP control release."""
    status_fresh = (status_age_s is not None and
                    status_age_s <= status_timeout_s)
    command_fresh = (command_age_s is not None and
                     command_age_s <= command_timeout_s)
    if not started:
        return "command" if status_fresh and command_fresh else "wait"
    if ((status_age_s is None or status_age_s > release_timeout_s) or
            (command_age_s is None or command_age_s > release_timeout_s)):
        return "release"
    return "command" if status_fresh and command_fresh else "brake"


def _validate_packet(packet, size, header, data_length):
    if not isinstance(packet, (bytes, bytearray, memoryview)):
        raise PacketError("packet must be bytes-like")
    packet = bytes(packet)
    if len(packet) != size:
        raise PacketError(
            "unexpected packet size: expected {}, got {}".format(
                size, len(packet)))
    if not packet.startswith(header):
        raise PacketError("unexpected packet header")
    actual_length = struct.unpack_from("<I", packet, len(header))[0]
    if actual_length != data_length:
        raise PacketError(
            "unexpected data length: expected {}, got {}".format(
                data_length, actual_length))
    if packet[-2:] != TAIL:
        raise PacketError("missing CRLF packet tail")
    return packet


def pack_ctrl_cmd(accel, brake, steer, ctrl_mode=2, gear=4, cmd_type=1,
                  velocity=0.0, acceleration=0.0):
    """Build the 55-byte competition Ego Ctrl Cmd packet.

    The caller may pass arbitrary numeric input; pedal and steering values are
    bounded to MORAI's documented [-1, 1] ranges. Competition launch code
    fixes ctrl_mode=2 and cmd_type=1, and limits gear to R=2 or D=4.
    """
    ctrl_mode = int(ctrl_mode)
    gear = int(gear)
    cmd_type = int(cmd_type)
    for value, name in ((ctrl_mode, "ctrl_mode"), (gear, "gear"),
                        (cmd_type, "cmd_type")):
        if value < 0 or value > 255:
            raise PacketError("{} must fit uint8".format(name))
    if ctrl_mode != 2:
        raise PacketError("competition ctrl_mode must be 2")
    if gear not in (2, 4):
        raise PacketError("competition gear must be R=2 or D=4")
    if cmd_type != 1:
        raise PacketError("competition cmd_type must be 1")
    return _CTRL.pack(
        CTRL_HEADER,
        CTRL_DATA_LENGTH,
        0, 0, 0,
        ctrl_mode,
        gear,
        cmd_type,
        _finite(velocity, "velocity"),
        _finite(acceleration, "acceleration"),
        _clamp(accel, 0.0, 1.0, "accel"),
        _clamp(brake, 0.0, 1.0, "brake"),
        _clamp(steer, -1.0, 1.0, "steer"),
        TAIL)


def parse_ctrl_cmd(packet):
    """Decode a Ctrl Cmd packet.  Used by tests and packet capture checks."""
    packet = _validate_packet(
        packet, CTRL_PACKET_SIZE, CTRL_HEADER, CTRL_DATA_LENGTH)
    values = _CTRL.unpack(packet)
    return {
        "ctrl_mode": values[5],
        "gear": values[6],
        "cmd_type": values[7],
        "velocity": values[8],
        "acceleration": values[9],
        "accel": values[10],
        "brake": values[11],
        "steer": values[12],
    }


def parse_vehicle_status(packet):
    """Decode the 181-byte Competition Vehicle Status packet.

    Fields excluded by the competition contract are consumed only to preserve
    their fixed wire offsets and are never returned to runtime callers.
    """
    packet = _validate_packet(
        packet, STATUS_PACKET_SIZE, STATUS_HEADER, STATUS_DATA_LENGTH)
    prefix = _STATUS_PREFIX.unpack_from(packet, 0)
    offset = _STATUS_PREFIX.size

    # accel/brake, dimensions, pose, velocity, angular velocity,
    # acceleration, steering.  All are IEEE-754 float32 on the wire.
    floats = struct.unpack_from("<24f", packet, offset)
    offset += struct.calcsize("<24f")
    link_id = struct.unpack_from("<38s", packet, offset)[0]

    return {
        "packet_size": len(packet),
        "data_length": STATUS_DATA_LENGTH,
        "sec": prefix[5],
        "nsec": prefix[6],
        "ctrl_mode": prefix[7],
        "gear": prefix[8],
        "signed_velocity_kmh": prefix[9],
        "map_data_id": prefix[10],
        "accel": floats[0],
        "brake": floats[1],
        "size": floats[2:5],
        "overhang": floats[5],
        "wheelbase": floats[6],
        "rear_overhang": floats[7],
        "rotation": floats[11:14],
        "angular_velocity": floats[17:20],
        "steer_deg": floats[23],
        "link_id": link_id.split(b"\0", 1)[0].decode(
            "ascii", errors="replace"),
    }


def parse_collision_data(packet):
    """Decode MORAI's fixed five-slot CollisionData packet."""
    packet = _validate_packet(
        packet, COLLISION_PACKET_SIZE, COLLISION_HEADER,
        COLLISION_DATA_LENGTH)
    prefix = _COLLISION_PREFIX.unpack_from(packet, 0)
    offset = _COLLISION_PREFIX.size
    records = []
    for _index in range(5):
        values = _COLLISION_RECORD.unpack_from(packet, offset)
        offset += _COLLISION_RECORD.size
        record = {
            "object_type": values[0],
            "object_id": values[1],
            "position": values[2:5],
            "global_offset": values[5:8],
        }
        # Newer competition builds also include the ego's own pose with
        # object_type=-1 even when no collision occurred. Never expose that
        # placeholder as a collision or as simulator position information.
        if values[0] >= 0 and any(values):
            records.append(record)
    return {
        "sec": prefix[5],
        "nsec": prefix[6],
        "records": records,
    }
