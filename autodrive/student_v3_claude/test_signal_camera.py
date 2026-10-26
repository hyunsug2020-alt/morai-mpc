"""Verify camera 4 reaches signal perception without changing model inputs."""
import copy
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2

import gui
from signal_control.runtime import SignalFrameWorker
from signal_control.signal_config import DEFAULT_CONFIG, validate_config
from signal_control.debug_store import SignalDebugStore
from test_signal_control import lamps


class SignalCameraTests(unittest.TestCase):
    def test_udp_camera4_drives_signal_and_preview_with_conflicting_front_input(self):
        # Use isolated ephemeral ports, never the simulator's real receivers.
        holders = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(4)]
        for sock in holders:
            sock.bind(("127.0.0.1", 0))
        ports = [sock.getsockname()[1] for sock in holders]
        profile = copy.deepcopy(gui.SENSOR_PROFILE)
        profile.update(local_ip="127.0.0.1", simulator_ip="127.0.0.1")
        profile["aux_camera"]["udp_port"] = ports[3]
        views = tuple(zip(("front", "left", "right"), ports[:3]))
        mask = cv2.resize(lamps()[1], (640, 480), interpolation=cv2.INTER_NEAREST)
        with tempfile.TemporaryDirectory() as directory, \
             patch.dict(os.environ, STUDENT_SIGNAL_GUARD="1", STUDENT_SESSION_DIR=""), \
             patch.object(gui, "SENSOR_PROFILE", profile), patch.object(gui, "VIEWS", views):
            receiver = gui.CameraReceiver()
            self.assertIsNone(receiver.session_dir)
            store = SignalDebugStore(directory)
            worker = SignalFrameWorker(receiver, "test", validate_config(DEFAULT_CONFIG),
                SimpleNamespace(mask=lambda *_: mask), debug_store=store)
            worker.select("signal")
            for sock in holders:
                sock.close()
            receiver.start()
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    for i in range(6):
                        state = "RED" if i < 3 else "GREEN"
                        other = "GREEN" if i < 3 else "RED"
                        stamp = 1_000_000_000 + i*100_000_000
                        for name, port in receiver.capture_views:
                            image = lamps(state if name == "front_low" else other)[0]
                            if name == "front_low":
                                image = cv2.resize(image, (640, 480))
                            ok, jpeg = cv2.imencode(".jpg", image)
                            self.assertTrue(ok)
                            payload = jpeg.tobytes()
                            packet = b"MOR" + struct.pack("<IIII", 1, i*100_000_000, 0, len(payload)) + payload + b"EI"
                            deadline = time.monotonic()+2.
                            while True:
                                sender.sendto(packet, ("127.0.0.1", port))
                                time.sleep(.01)
                                with receiver.lock:
                                    arrived = bool(receiver.frames[name] and receiver.frames[name][-1][0] == stamp)
                                if arrived:
                                    break
                                self.assertLess(time.monotonic(), deadline, "camera receive timeout: "+name)
                        self.assertTrue(worker.process_once())
                        report = worker.snapshot()
                        self.assertEqual(report["camera_id"], 4)
                        self.assertEqual(report["camera_view"], "front_low")
                        self.assertEqual(report["raw_state"], state)
                        self.assertEqual(report["camera_stamp_ns"], stamp)
                    self.assertTrue(worker.snapshot()["permission_stable"])
                    debug = store.latest("test")
                    self.assertEqual(debug["camera_id"], 4)
                    image = store.image("test", debug["frame_id"], debug["slot"], "original")
                    import numpy as np
                    self.assertEqual(cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR).shape[:2], (480, 640))
                    rows = receiver.snapshot(count=1)
                    self.assertEqual(len(rows[0]), 3)
                    self.assertNotIn(receiver.frames["front_low"][-1][1], rows[0])
            finally:
                receiver.stop_event.set()
                receiver.join(timeout=3.)
            self.assertFalse(receiver.is_alive())

    def test_disabled_signals_keep_three_model_views_and_collection_still_has_camera4(self):
        with patch.dict(os.environ, STUDENT_SIGNAL_GUARD="0", STUDENT_SESSION_DIR=""):
            receiver = gui.CameraReceiver()
            self.assertEqual(receiver.capture_views, gui.VIEWS)
            self.assertEqual(set(receiver.frames), {name for name, _ in gui.VIEWS})
            self.assertIn("front_low", dict(gui.camera_input_views(recording=True)))

    def test_ros_subscribes_to_camera4_and_keeps_it_out_of_model_snapshot(self):
        topics = []
        rospy = SimpleNamespace(Subscriber=lambda topic, *args, **kwargs: topics.append(topic))
        with patch.dict(os.environ, STUDENT_SIGNAL_GUARD="1"), \
             patch.dict(sys.modules, {"rospy": rospy, "sensor_msgs.msg": SimpleNamespace(Image=object)}):
            receiver = gui.RosCameraReceiver()
        self.assertIn("/around_view/camera/front_low/image_raw", topics)
        image = lamps()[0]
        msg = SimpleNamespace(encoding="bgr8", data=image.tobytes(), height=image.shape[0], width=image.shape[1],
            header=SimpleNamespace(stamp=SimpleNamespace(secs=1, nsecs=0)))
        for name, _ in receiver.input_views:
            receiver._on_image(msg, name)
        self.assertTrue(receiver.frames["front_low"])
        self.assertEqual(len(receiver.snapshot(count=1)[0]), 3)

    def test_file_receiver_reads_camera4_without_requiring_it_for_model_snapshot(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.dict(os.environ, STUDENT_SIGNAL_GUARD="1", STUDENT_CAMERA_CACHE=directory):
            receiver = gui.RecordedCameraReceiver(directory)
            _, jpeg = cv2.imencode(".jpg", lamps()[0])
            stamp = time.time_ns()
            payload = struct.pack("!QQ", stamp, stamp) + jpeg.tobytes()
            for name, _ in receiver.input_views:
                (Path(directory)/(name+".frame")).write_bytes(payload)
            receiver.read_live_frames()
            self.assertTrue(receiver.frames["front_low"])
            self.assertEqual(len(receiver.snapshot(count=1)[0]), 3)


if __name__ == "__main__":
    unittest.main()
