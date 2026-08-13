"""MORAI UDP JPEG packet decoding and multi-camera reception."""

import socket
import struct
import threading
import time

import cv2
import numpy as np


class MoraiPacketDecoder:
    """Assemble JPEG frames from MORAI camera UDP chunks.

    MORAI installations in this workspace use one of two packet layouts:

    * legacy: 11-byte header, 1-byte chunk index at offset 3
    * current: 19-byte header, uint32 chunk index at offset 11

    ``packet_format="auto"`` validates the current layout first and falls
    back to the legacy layout.
    """

    VALID_FORMATS = ("auto", "legacy11", "current19")

    def __init__(self, packet_format="auto"):
        if packet_format not in self.VALID_FORMATS:
            raise ValueError(
                "packet_format must be one of {}".format(self.VALID_FORMATS)
            )
        self.packet_format = packet_format
        self.buffer = bytearray()
        self.expected_chunk = None

    @staticmethod
    def _parse_legacy11(data):
        if len(data) < 11:
            return None
        chunk_index = data[3]
        payload_length = struct.unpack("<I", data[7:11])[0]
        payload_end = 11 + payload_length
        if payload_length <= 0 or payload_end > len(data):
            return None
        return chunk_index, data[11:payload_end]

    @staticmethod
    def _parse_current19(data):
        if len(data) < 19:
            return None
        chunk_index = struct.unpack("<I", data[11:15])[0]
        payload_length = struct.unpack("<I", data[15:19])[0]
        payload_end = 19 + payload_length
        if (
            chunk_index > 100000
            or payload_length <= 0
            or payload_end > len(data)
        ):
            return None
        return chunk_index, data[19:payload_end]

    def _parse(self, data):
        if not data.startswith(b"MOR"):
            return None

        if self.packet_format == "legacy11":
            return self._parse_legacy11(data)
        if self.packet_format == "current19":
            return self._parse_current19(data)

        parsed = self._parse_current19(data)
        if parsed is not None:
            return parsed
        return self._parse_legacy11(data)

    def feed(self, data):
        """Consume one datagram and return a decoded BGR frame when complete."""
        parsed = self._parse(data)
        if parsed is None:
            return None

        chunk_index, payload = parsed
        if chunk_index == 0:
            self.buffer = bytearray(payload)
            self.expected_chunk = 1
        elif not self.buffer or chunk_index != self.expected_chunk:
            self.buffer = bytearray()
            self.expected_chunk = None
            return None
        else:
            self.buffer.extend(payload)
            self.expected_chunk += 1

        if not data.endswith(b"EI"):
            return None

        jpeg = bytes(self.buffer)
        self.buffer = bytearray()
        self.expected_chunk = None

        end_of_image = jpeg.find(b"\xff\xd9")
        if end_of_image >= 0:
            jpeg = jpeg[: end_of_image + 2]

        image_bytes = np.frombuffer(jpeg, dtype=np.uint8)
        return cv2.imdecode(image_bytes, cv2.IMREAD_COLOR)


class UdpCameraReceiver:
    """Receive multiple MORAI UDP camera streams in background threads."""

    def __init__(
        self,
        ports,
        bind_host="0.0.0.0",
        packet_format="auto",
        receive_buffer_bytes=1024 * 1024,
        socket_timeout=0.5,
        logger=None,
    ):
        self.ports = [int(port) for port in ports]
        self.bind_host = bind_host
        self.packet_format = packet_format
        self.receive_buffer_bytes = int(receive_buffer_bytes)
        self.socket_timeout = float(socket_timeout)
        self.logger = logger or print

        self._frames = {port: None for port in self.ports}
        self._frame_times = {port: None for port in self.ports}
        self._sequences = {port: 0 for port in self.ports}
        self._locks = {port: threading.Lock() for port in self.ports}
        self._threads = []
        self._sockets = {}
        self._stop_event = threading.Event()

    def _log(self, message):
        self.logger(message)

    def start(self):
        if self._threads:
            return
        self._stop_event.clear()
        for port in self.ports:
            thread = threading.Thread(
                target=self._receive_loop,
                args=(port,),
                name="udp-camera-{}".format(port),
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)

    def stop(self):
        self._stop_event.set()
        for sock in list(self._sockets.values()):
            try:
                sock.close()
            except OSError:
                pass
        for thread in self._threads:
            thread.join(timeout=self.socket_timeout + 0.5)
        self._threads = []
        self._sockets = {}

    def get_frame(self, port, copy=True):
        port = int(port)
        with self._locks[port]:
            frame = self._frames[port]
            if frame is not None and copy:
                frame = frame.copy()
            return frame, self._frame_times[port], self._sequences[port]

    def _receive_loop(self, port):
        decoder = MoraiPacketDecoder(self.packet_format)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sockets[port] = sock

        try:
            sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_RCVBUF, self.receive_buffer_bytes
            )
            sock.bind((self.bind_host, port))
            sock.settimeout(self.socket_timeout)
        except OSError as error:
            self._log("[UDP:{}] bind failed: {}".format(port, error))
            try:
                sock.close()
            except OSError:
                pass
            return

        self._log(
            "[UDP:{}] listening on {}:{} ({})".format(
                port, self.bind_host, port, self.packet_format
            )
        )
        received_first_packet = False
        invalid_packets = 0

        while not self._stop_event.is_set():
            try:
                data, address = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as error:
                if not self._stop_event.is_set():
                    self._log("[UDP:{}] receive failed: {}".format(port, error))
                break

            if not received_first_packet:
                self._log(
                    "[UDP:{}] first packet from {}:{} ({} bytes)".format(
                        port, address[0], address[1], len(data)
                    )
                )
                received_first_packet = True

            frame = decoder.feed(data)
            if frame is None:
                if not data.startswith(b"MOR"):
                    invalid_packets += 1
                    if invalid_packets % 30 == 0:
                        self._log(
                            "[UDP:{}] {} packets had an invalid header".format(
                                port, invalid_packets
                            )
                        )
                continue

            with self._locks[port]:
                self._frames[port] = frame
                self._frame_times[port] = time.time()
                self._sequences[port] += 1

        try:
            sock.close()
        except OSError:
            pass
