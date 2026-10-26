"""Read-only, profile-scoped UDP previews while the driving stack is idle."""
import select
import socket
import struct
import threading
import time


class CameraFrame:
    """Assemble bounded MORAI JPEG fragments; discard incomplete frames."""
    MAX_BYTES = 8 * 1024 * 1024

    def __init__(self):
        self.reset()

    def reset(self):
        self.stamp = None
        self.expected = 0
        self.buffer = bytearray()

    def feed(self, packet):
        if len(packet) < 21 or packet[:3] != b"MOR":
            return None
        sec, nsec, index, size = struct.unpack_from("<IIII", packet, 3)
        if nsec >= 1_000_000_000 or not size or 19 + size > len(packet) - 2:
            self.reset()
            return None
        stamp = (sec, nsec)
        if index == 0:
            self.reset()
            self.stamp = stamp
        elif stamp != self.stamp or index != self.expected:
            self.reset()
            return None
        self.buffer.extend(packet[19:19 + size])
        self.expected = index + 1
        if len(self.buffer) > self.MAX_BYTES:
            self.reset()
            return None
        if packet[-2:] != b"EI":
            return None
        jpeg = bytes(self.buffer)
        self.reset()
        end = jpeg.find(b"\xff\xd9")
        return jpeg[:end + 2] if jpeg.startswith(b"\xff\xd8") and end >= 0 else None


class SensorPreview:
    """Temporary receivers release every socket before a dashboard drive starts."""
    def __init__(self):
        self.lock = threading.RLock()
        self.frames = {}
        self.samples = {}

    def clear(self):
        with self.lock:
            self.frames.clear()
            self.samples.clear()

    def image(self, profile_id, view, max_age=4.0):
        with self.lock:
            received, jpeg = self.frames.get((profile_id, view), (0, None))
            return jpeg if time.time() - received <= max_age else None

    def sample(self, profile, can_read=lambda: True, seconds=0.65, ttl=1.5):
        with self.lock:
            if not can_read():
                return None
            now = time.monotonic()
            cached, sampled = self.samples.get(profile["id"], (None, 0))
            if cached is not None and now - sampled < ttl:
                return cached
            ports = {view: spec["udp_port"] for view, spec in profile["cameras"].items()}
            ports["front_low"] = profile["aux_camera"]["udp_port"]
            camera_views = set(ports)
            ports.update({name: profile[key] for name, key in (
                ("gps", "gps_port"), ("imu", "imu_port"), ("lidar", "lidar_port"),
                ("status", "status_port"), ("collision", "collision_port"))})
            streams = {name: {"port": port, "packets": 0, "frames": 0, "error": None}
                       for name, port in ports.items()}
            sockets = {}
            assemblers = {view: CameraFrame() for view in camera_views}
            start = time.monotonic()
            try:
                for name, port in ports.items():
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    try:
                        # No SO_REUSEADDR: a preview must never share a receiver's port.
                        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
                        sock.bind((profile["local_ip"], port))
                        sock.setblocking(False)
                        sockets[sock] = name
                    except OSError as error:
                        streams[name]["error"] = str(error)
                        sock.close()
                while sockets and time.monotonic() - start < seconds and can_read():
                    ready, _, _ = select.select(list(sockets), [], [], min(.05, max(0, seconds - (time.monotonic() - start))))
                    for sock in ready:
                        name = sockets[sock]
                        for _ in range(128):
                            try:
                                packet, address = sock.recvfrom(65535)
                            except BlockingIOError:
                                break
                            if address[0] != profile["simulator_ip"]:
                                continue
                            streams[name]["packets"] += 1
                            if name in assemblers:
                                jpeg = assemblers[name].feed(packet)
                                if jpeg:
                                    streams[name]["frames"] += 1
                                    self.frames[(profile["id"], name)] = (time.time(), jpeg)
            finally:
                for sock in sockets:
                    sock.close()
            elapsed = max(.001, time.monotonic() - start)
            for name, stream in streams.items():
                stream["connected"] = bool(stream["frames"] if name in camera_views else stream["packets"])
                stream["hz"] = round(stream["frames"] / elapsed, 1) if name in camera_views else None
                stream["packets_per_s"] = round(stream["packets"] / elapsed, 1)
            result = {"profile": profile["id"], "simulator_ip": profile["simulator_ip"],
                      "local_ip": profile["local_ip"], "sampled_at": time.time(), "streams": streams}
            self.samples[profile["id"]] = (result, time.monotonic())
            return result
