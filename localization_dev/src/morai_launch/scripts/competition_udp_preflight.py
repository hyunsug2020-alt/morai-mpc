#!/usr/bin/env python3
"""Read-only MORAI UDP status and existing-sensor handshake."""

import argparse
import socket
import sys
import time

from morai_launch.competition_udp_protocol import (
    PacketError,
    parse_vehicle_status,
    speed_kmh_to_mps,
)


def classify_camera_packet(packet):
    """Classify MORAI camera datagrams needed by the competition gate."""
    if packet.startswith(b"MOR"):
        return "image"
    if packet.startswith(b"BOX"):
        return "ground_truth"
    return "unknown"


def probe_camera_port(bind_ip, simulator_ip, port, duration_s):
    """Require image UDP; report BOX packets that runtime receivers discard."""
    deadline = time.monotonic() + max(0.1, float(duration_s))
    saw_image = False
    saw_ground_truth = False
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.05)
    try:
        sock.bind((bind_ip, int(port)))
        while time.monotonic() < deadline:
            try:
                packet, address = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if address[0] != simulator_ip:
                continue
            packet_class = classify_camera_packet(packet)
            if packet_class == "ground_truth":
                saw_ground_truth = True
            if packet_class == "image":
                saw_image = True
    except OSError as error:
        return False, "UDP port open failed: {}".format(error)
    finally:
        sock.close()
    if not saw_image:
        return False, "no MOR image packet"
    if saw_ground_truth:
        return True, "image (BOX ignored by runtime receiver)"
    return True, "image"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator-ip", required=True)
    parser.add_argument("--bind-ip", required=True)
    parser.add_argument("--status-port", required=True, type=int)
    parser.add_argument("--camera-port", action="append", type=int,
                        default=[])
    parser.add_argument("--camera-probe-s", type=float, default=0.35)
    parser.add_argument("--timeout", type=float, default=15.0)
    arguments = parser.parse_args()

    deadline = time.monotonic() + max(0.1, arguments.timeout)
    status = None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.25)
    try:
        sock.bind((arguments.bind_ip, arguments.status_port))
        while time.monotonic() < deadline:
            try:
                packet, address = sock.recvfrom(4096)
            except socket.timeout:
                continue
            if address[0] != arguments.simulator_ip:
                continue
            try:
                status = parse_vehicle_status(packet)
            except PacketError as error:
                print(
                    "오류: MORAI 상태 UDP 형식 불일치: {}".format(error),
                    file=sys.stderr)
                return 3
            break
    except OSError as error:
        print("오류: 상태 UDP 포트 열기 실패: {}".format(error), file=sys.stderr)
        return 4
    finally:
        sock.close()

    if status is None:
        print(
            "오류: {}초 동안 {}:{}의 #MoraiInfo$ UDP가 없음. "
            "MORAI Play/Publisher/Destination IP/Port 확인 필요".format(
                arguments.timeout, arguments.bind_ip, arguments.status_port),
            file=sys.stderr)
        return 5

    camera_results = []
    for camera_port in arguments.camera_port:
        accepted, reason = probe_camera_port(
            arguments.bind_ip, arguments.simulator_ip, camera_port,
            arguments.camera_probe_s)
        if not accepted:
            print(
                "오류: Camera UDP {} 설정 불일치: {}. "
                "기존 MORAI Sensor UDP port 확인 필요".format(
                    camera_port, reason), file=sys.stderr)
            return 6
        camera_results.append("{}:{}".format(camera_port, reason))

    print(
        "MORAI UDP 확인 완료: source={} packet={}bytes "
        "speed={:.3f}km/h({:.3f}m/s) ctrl_mode={} gear={} "
        "cameras={} [{}]".format(
            arguments.simulator_ip, status["packet_size"],
            status["signed_velocity_kmh"],
            speed_kmh_to_mps(status["signed_velocity_kmh"]),
            status["ctrl_mode"], status["gear"], len(arguments.camera_port),
            ", ".join(camera_results)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
