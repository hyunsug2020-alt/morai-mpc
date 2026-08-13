#!/usr/bin/env python3

import struct
import unittest

import cv2
import numpy as np

from around_view.udp_receiver import MoraiPacketDecoder


def make_packet(packet_format, chunk_index, payload, final, padded=False):
    suffix = b"EI" if final else b"AI"
    if packet_format == "legacy11":
        header = bytearray(11)
        header[0:3] = b"MOR"
        header[3] = chunk_index
        header[7:11] = struct.pack("<I", len(payload))
    else:
        header = bytearray(19)
        header[0:3] = b"MOR"
        header[11:15] = struct.pack("<I", chunk_index)
        header[15:19] = struct.pack("<I", len(payload))
    padding = b""
    if padded:
        padding = bytes(max(0, 65000 - len(header) - len(payload) - len(suffix)))
    return bytes(header) + payload + padding + suffix


class MoraiPacketDecoderTest(unittest.TestCase):
    def setUp(self):
        source = np.zeros((24, 32, 3), dtype=np.uint8)
        source[:, :, 1] = 180
        success, encoded = cv2.imencode(".jpg", source)
        self.assertTrue(success)
        self.jpeg = encoded.tobytes()

    def _decode(self, packet_format):
        decoder = MoraiPacketDecoder("auto")
        split = len(self.jpeg) // 2
        first = make_packet(packet_format, 0, self.jpeg[:split], False)
        second = make_packet(packet_format, 1, self.jpeg[split:], True)
        self.assertIsNone(decoder.feed(first))
        frame = decoder.feed(second)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.shape, (24, 32, 3))

    def test_legacy_11_byte_header(self):
        self._decode("legacy11")

    def test_current_19_byte_header(self):
        self._decode("current19")

    def test_padded_final_packet(self):
        decoder = MoraiPacketDecoder("auto")
        split = len(self.jpeg) // 2
        first = make_packet("current19", 0, self.jpeg[:split], False)
        final = make_packet(
            "current19", 1, self.jpeg[split:], True, padded=True
        )
        self.assertIsNone(decoder.feed(first))
        self.assertIsNotNone(decoder.feed(final))

    def test_out_of_order_chunk_is_discarded(self):
        decoder = MoraiPacketDecoder("auto")
        packet = make_packet("current19", 2, self.jpeg, True)
        self.assertIsNone(decoder.feed(packet))


if __name__ == "__main__":
    unittest.main()
