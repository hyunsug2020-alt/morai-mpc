"""HSV previews stay coherent, bounded and independent of driving decisions."""
import copy
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from signal_control.debug_store import SignalDebugStore, SLOTS, VIEWS
from signal_control.pipeline import SignalFramePipeline
from signal_control.runtime import SignalFrameWorker
from signal_control.signal_config import DEFAULT_CONFIG, validate_config
from signal_control.visualization import signal_debug_images
import test_signal_control as signal_tests


class SignalDebugStoreTests(unittest.TestCase):
    def metadata(self, now=100., run="run"):
        return dict(run_id=run, camera_stamp_ns=10, line_id="signal", fresh=True,
                    source="live", time=now, frame_wall=now, camera_id=4, camera_view="front_low")

    def images(self, prefix=b"frame"):
        return {view: prefix+b"-"+view.encode() for view in VIEWS}

    def test_all_images_and_metadata_belong_to_one_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            store = SignalDebugStore(directory)
            images = self.images()
            metadata = store.publish(self.metadata(), DEFAULT_CONFIG, images)
            self.assertEqual(store.latest("run", now=100.1), metadata)
            for view in VIEWS:
                self.assertEqual(store.image("run", metadata["frame_id"], metadata["slot"], view,
                                             now=100.1, line_id="signal"), images[view])

    def test_stale_wrong_run_frame_line_and_non_live_source_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            store = SignalDebugStore(directory)
            metadata = store.publish(self.metadata(), DEFAULT_CONFIG, self.images())
            for run, frame, slot, view, now, line in (
                ("other", metadata["frame_id"], metadata["slot"], "original", 100.1, "signal"),
                ("run", "wrong", metadata["slot"], "original", 100.1, "signal"),
                ("run", metadata["frame_id"], metadata["slot"], "original", 102., "signal"),
                ("run", metadata["frame_id"], metadata["slot"], "original", 99., "signal"),
                ("run", metadata["frame_id"], metadata["slot"], "original", 100.1, "other"),
                ("run", metadata["frame_id"], -1, "original", 100.1, "signal"),
                ("run", metadata["frame_id"], metadata["slot"], "../metadata", 100.1, "signal"),
            ):
                self.assertIsNone(store.image(run, frame, slot, view, now, line))
            self.assertIsNone(store.latest("run", now=102.))
            self.assertIsNone(store.latest("other", now=100.1))
            store.publish(dict(self.metadata(), source="recording"), DEFAULT_CONFIG, self.images())
            self.assertIsNone(store.latest("run", now=100.1))
            store.publish(dict(self.metadata(), camera_id=1, camera_view="front"), DEFAULT_CONFIG, self.images())
            self.assertIsNone(store.latest("run", now=100.1))

    def test_fixed_ring_never_grows_and_overwritten_frames_cannot_be_mixed(self):
        with tempfile.TemporaryDirectory() as directory:
            store = SignalDebugStore(directory)
            first = store.publish(self.metadata(), DEFAULT_CONFIG, self.images(b"old"))
            for i in range(SLOTS+2):
                latest = store.publish(dict(self.metadata(), camera_stamp_ns=20+i),
                                       DEFAULT_CONFIG, self.images(str(i).encode()))
            self.assertEqual(len(list(Path(directory).glob("frame-*.zip"))), SLOTS)
            self.assertEqual(len(list(Path(directory).iterdir())), SLOTS+1)
            self.assertIsNone(store.image("run", first["frame_id"], first["slot"], "original", now=100.1))
            self.assertEqual(store.latest("run", now=100.1), latest)

    def test_corrupt_or_malformed_artifacts_are_hidden(self):
        with tempfile.TemporaryDirectory() as directory:
            store = SignalDebugStore(directory)
            meta = store.publish(self.metadata(), DEFAULT_CONFIG, self.images())
            (Path(directory)/("frame-%s.zip" % meta["slot"])).write_bytes(b"incomplete archive")
            self.assertIsNone(store.image("run", meta["frame_id"], meta["slot"], "original", now=100.1))
            for value in ("broken", "null", "[]", '{"fresh":true}', json.dumps(dict(meta, slot=True))):
                (Path(directory)/"latest.json").write_text(value)
                self.assertIsNone(store.latest("run", now=100.1))


class SignalVisualizationTests(unittest.TestCase):
    def frame(self, state="GREEN"):
        config = validate_config(DEFAULT_CONFIG)
        image, mask = signal_tests.lamps(state)
        pipeline = SignalFramePipeline(config)
        report, crop, masks = pipeline.process(image, mask, "run")
        return image, mask, report, crop, masks, config

    def test_green_hsv_pixels_are_visible_and_inputs_do_not_change(self):
        values = self.frame()
        image, mask, report, crop, masks, config = values
        before = [image.copy(), mask.copy(), crop.copy(), copy.deepcopy(report)]
        images = signal_debug_images(*values)
        self.assertEqual(set(images), set(VIEWS))
        decoded = {key: cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                   for key, data in images.items()}
        self.assertTrue(all(image is not None for image in decoded.values()))
        self.assertGreater(int(decoded["green"][:, :, 1].max()), 200)
        self.assertEqual(int(decoded["red"].max()), 0)
        self.assertEqual(int(decoded["yellow"].max()), 0)
        np.testing.assert_array_equal(image, before[0])
        np.testing.assert_array_equal(mask, before[1])
        np.testing.assert_array_equal(crop, before[2])
        self.assertEqual(report, before[3])

    def test_missing_housing_produces_empty_hsv_masks(self):
        image, mask, _, _, _, config = self.frame()
        mask.fill(0)
        report, crop, masks = SignalFramePipeline(config).process(image, mask, "run")
        images = signal_debug_images(image, mask, report, crop, masks, config)
        for color in ("red", "yellow", "green"):
            decoded = cv2.imdecode(np.frombuffer(images[color], np.uint8), cv2.IMREAD_COLOR)
            self.assertEqual(int(decoded.max()), 0)

    def worker(self, store):
        image, mask = signal_tests.lamps()
        _, jpeg = cv2.imencode(".jpg", image)
        receiver = SimpleNamespace(lock=threading.Lock(), frames={"front_low": []}, counts={"front_low": []})
        worker = SignalFrameWorker(receiver, "run", validate_config(DEFAULT_CONFIG),
            SimpleNamespace(mask=lambda *_: mask), debug_store=store)
        worker.select("signal")
        def advance(stamp):
            receiver.frames["front_low"].append((stamp, jpeg.tobytes()))
            receiver.counts["front_low"].append(time.monotonic())
        return worker, advance

    def test_worker_preview_uses_actual_report_and_does_not_repeat_inference(self):
        with tempfile.TemporaryDirectory() as directory:
            store = SignalDebugStore(directory)
            worker, advance = self.worker(store)
            for stamp in range(3):
                advance(stamp)
                self.assertTrue(worker.process_once())
            report, debug = worker.snapshot(), store.latest("run")
            self.assertTrue(report["permission_stable"])
            self.assertEqual(debug["camera_stamp_ns"], report["camera_stamp_ns"])
            self.assertEqual(debug["slots"], report["slots"])
            self.assertFalse(worker.process_once())
            self.assertEqual(store.sequence, 3)

    def test_preview_encoding_or_storage_failure_does_not_reset_permission(self):
        for failure in (OSError("disk full"), ValueError("bad preview")):
            worker, advance = self.worker(SimpleNamespace(publish=lambda *_: (_ for _ in ()).throw(failure)))
            for stamp in range(3):
                advance(stamp)
                self.assertTrue(worker.process_once())
            self.assertTrue(worker.snapshot()["permission_stable"])
            self.assertEqual(worker.snapshot()["raw_state"], "GREEN")
        worker, advance = self.worker(SimpleNamespace(publish=lambda *_: None))
        with patch("signal_control.runtime.signal_debug_images", side_effect=RuntimeError("preview failed")):
            for stamp in range(3):
                advance(stamp)
                self.assertTrue(worker.process_once())
        self.assertTrue(worker.snapshot()["permission_stable"])

    def test_slow_preview_does_not_hold_control_report_lock(self):
        worker, advance = self.worker(None)
        for stamp in range(2):
            advance(stamp)
            worker.process_once()
        worker.debug_store = SimpleNamespace(publish=lambda *_: None)
        entered, release, readable = threading.Event(), threading.Event(), threading.Event()
        def slow(*_):
            entered.set()
            release.wait(2.)
            return {}
        advance(2)
        with patch("signal_control.runtime.signal_debug_images", side_effect=slow):
            thread = threading.Thread(target=worker.process_once)
            thread.start()
            self.assertTrue(entered.wait(1.))
            def read():
                if worker.snapshot()["permission_stable"]:
                    readable.set()
            reader = threading.Thread(target=read)
            reader.start()
            try:
                self.assertTrue(readable.wait(.5))
            finally:
                release.set()
                thread.join(2.)
                reader.join(2.)


if __name__ == "__main__":
    unittest.main()
