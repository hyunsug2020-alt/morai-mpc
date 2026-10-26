"""UDP source isolation, complete JPEGs and receiver handoff regression checks."""
import socket
import struct
import threading
import time
import unittest

from web_sensor_preview import CameraFrame, SensorPreview


def packet(payload, index=0, stamp=10, end=True):
    return b"MOR" + struct.pack("<IIII", stamp, 1, index, len(payload)) + payload + (b"EI" if end else b"\0\0")


class FrameTests(unittest.TestCase):
    def test_complete_fragmented_frame_and_next_frame(self):
        parser = CameraFrame()
        self.assertIsNone(parser.feed(packet(b"\xff\xd8first", end=False)))
        self.assertEqual(parser.feed(packet(b"second\xff\xd9", index=1)), b"\xff\xd8firstsecond\xff\xd9")
        self.assertEqual(parser.feed(packet(b"\xff\xd8new\xff\xd9", stamp=11)), b"\xff\xd8new\xff\xd9")

    def test_missing_fragment_or_changed_timestamp_never_displays_partial_image(self):
        for index, stamp in ((2, 10), (1, 11)):
            with self.subTest(index=index, stamp=stamp):
                parser = CameraFrame()
                parser.feed(packet(b"\xff\xd8first", end=False))
                self.assertIsNone(parser.feed(packet(b"tail\xff\xd9", index=index, stamp=stamp)))
                self.assertIsNone(parser.feed(packet(b"tail\xff\xd9", index=3)))

    def test_malformed_and_oversized_images_are_rejected(self):
        parser = CameraFrame()
        for value in (b"MOR", packet(b"not a jpeg"), b"MOR" + struct.pack("<IIII", 1, 1_000_000_000, 0, 2) + b"xxEI"):
            self.assertIsNone(parser.feed(value))
        parser.MAX_BYTES = 5
        self.assertIsNone(parser.feed(packet(b"\xff\xd8too large\xff\xd9")))
        self.assertEqual(len(parser.buffer), 0)


class PreviewTests(unittest.TestCase):
    def profile(self):
        reservations = []
        for _ in range(9):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", 0))
            reservations.append(sock)
        ports = [s.getsockname()[1] for s in reservations]
        for sock in reservations:
            sock.close()
        p = {"id": "test", "local_ip": "127.0.0.1", "simulator_ip": "127.0.0.2",
             "cameras": {name: {"udp_port": ports[i]} for i, name in enumerate(("front", "left", "right"))},
             "aux_camera": {"udp_port": ports[3]}}
        p.update(dict(zip(("gps_port", "imu_port", "lidar_port", "status_port", "collision_port"), ports[4:])))
        return p, ports

    def test_correct_source_only_and_all_ports_released_after_preview(self):
        profile, ports = self.profile()
        monitor = SensorPreview()
        stopped = threading.Event()
        wanted = b"\xff\xd8correct source\xff\xd9"
        unwanted = b"\xff\xd8other profile\xff\xd9"

        def send():
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as good, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bad:
                good.bind(("127.0.0.2", 0)); bad.bind(("127.0.0.3", 0))
                while not stopped.is_set():
                    good.sendto(packet(wanted), ("127.0.0.1", ports[0]))
                    good.sendto(b"gps", ("127.0.0.1", ports[4]))
                    bad.sendto(packet(unwanted), ("127.0.0.1", ports[0]))
                    bad.sendto(packet(unwanted), ("127.0.0.1", ports[1]))
                    stopped.wait(.01)

        thread = threading.Thread(target=send); thread.start()
        try:
            result = monitor.sample(profile, seconds=.2, ttl=0)
        finally:
            stopped.set(); thread.join()
        self.assertTrue(result["streams"]["front"]["connected"])
        self.assertTrue(result["streams"]["gps"]["connected"])
        self.assertFalse(result["streams"]["left"]["connected"])
        self.assertEqual(monitor.image("test", "front"), wanted)
        self.assertIsNone(monitor.image("other", "front"))
        for port in ports:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind(("127.0.0.1", port))

    def test_busy_port_is_never_shared(self):
        profile, ports = self.profile()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as owner:
            owner.bind(("127.0.0.1", ports[0]))
            result = SensorPreview().sample(profile, seconds=.02, ttl=0)
            self.assertIsNotNone(result["streams"]["front"]["error"])
            self.assertFalse(result["streams"]["front"]["connected"])

    def test_drive_handoff_interrupts_sampling_and_closes_sockets(self):
        profile, ports = self.profile()
        monitor = SensorPreview()
        allowed = threading.Event(); allowed.set()
        thread = threading.Thread(target=lambda: monitor.sample(profile, can_read=allowed.is_set, seconds=5))
        thread.start(); time.sleep(.05); allowed.clear(); thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
        for port in ports:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind(("127.0.0.1", port))
        self.assertIsNone(monitor.sample(profile, can_read=lambda: False))

    def test_stale_frames_are_hidden_and_launch_clear_removes_cached_images(self):
        monitor = SensorPreview()
        monitor.frames[("two_pc", "front")] = (time.time() - 10, b"old")
        monitor.frames[("single_pc", "front")] = (time.time(), b"fresh")
        self.assertIsNone(monitor.image("two_pc", "front"))
        self.assertEqual(monitor.image("single_pc", "front"), b"fresh")
        monitor.clear()
        self.assertIsNone(monitor.image("single_pc", "front"))


if __name__ == "__main__":
    unittest.main()
