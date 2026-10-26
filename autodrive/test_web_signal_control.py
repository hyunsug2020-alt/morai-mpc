"""Signal choice reaches both HTTP entry points without starting a vehicle."""
import json
from contextlib import contextmanager
from pathlib import Path
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from urllib.parse import urlencode
from http.server import ThreadingHTTPServer
from unittest.mock import patch

import web_app
from student_v3_claude.signal_control.debug_store import SignalDebugStore, VIEWS
from student_v3_claude.signal_control.signal_config import DEFAULT_CONFIG


class WebSignalControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), web_app.Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = "http://127.0.0.1:%d" % cls.server.server_address[1]

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def post(self, endpoint, payload):
        request = urllib.request.Request(self.base+endpoint, json.dumps(payload).encode(),
                                         {"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=3) as response:
            return json.load(response)

    def test_on_off_and_safe_default_reach_process_for_both_profiles(self):
        with patch.object(web_app.JOBS, "start") as start:
            for name in ("single_pc", "two_pc"):
                for options, expected in (({}, "1"), ({"signal_enabled": True}, "1"),
                                          ({"signal_enabled": False}, "0")):
                    with self.subTest(name=name, options=options):
                        self.post("/api/drive/start", {"profile": name, "options": options})
                        command = start.call_args.args[1]
                        self.assertEqual(command[:2], ["env", "STUDENT_SIGNAL_GUARD="+expected])
                        self.assertEqual(web_app.job_network_profile({"command": command}), name)
                        self.assertEqual(command[2:4], ["bash", str(web_app.LAUNCH)])

    def test_collection_and_legacy_endpoint_preserve_signal_choice(self):
        with patch.object(web_app.JOBS, "start") as start:
            self.post("/api/drive/start", {"profile": "single_pc", "collect": True,
                                         "options": {"signal_enabled": False}})
            action, command = start.call_args.args
            self.assertEqual(action, "autodrive_collect")
            self.assertEqual(command[:2], ["env", "STUDENT_SIGNAL_GUARD=0"])
            self.assertEqual(web_app.job_network_profile({"command": command}), "single_pc")
            self.assertTrue(command[-1].isdecimal())
            self.post("/api/start", {"action": "autodrive_follow",
                                     "options": {"network_profile": "two_pc", "signal_enabled": False}})
            self.assertEqual(start.call_args.args[1][1], "STUDENT_SIGNAL_GUARD=0")

    def test_non_boolean_cannot_launch_and_options_must_be_object(self):
        with patch.object(web_app.JOBS, "start") as start:
            for endpoint in ("/api/drive/start", "/api/start"):
                for options in ([{"signal_enabled": v} for v in ("0", "false", 0, 1, None, [], {})]
                                + [None, [], False, ""]):
                    with self.subTest(endpoint=endpoint, options=options):
                        with self.assertRaises(urllib.error.HTTPError) as error:
                            self.post(endpoint, {"profile": "two_pc", "action": "autodrive_follow",
                                                 "options": options})
                        self.assertEqual(error.exception.code, 400)
            start.assert_not_called()

    def test_signal_control_present_on_both_dashboards(self):
        for endpoint in ("/single-pc", "/competition"):
            with urllib.request.urlopen(self.base+endpoint) as response:
                html = response.read().decode()
            self.assertIn('id="signalEnabled" type="checkbox" checked', html)
            self.assertIn('id="signalStatus"', html)
            self.assertIn('/signals?profile=', html)

    def get(self, path):
        with urllib.request.urlopen(self.base+path, timeout=3) as response:
            return response.headers.get("Content-Type"), response.read()

    @contextmanager
    def live_view(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root/"follow-signal-test"
            run.mkdir()
            now = time.time()
            (run/"run.json").write_text(json.dumps(dict(run=run.name, signal_enabled=True)))
            control = dict(run_id=run.name, time=now, active=True, selected_line_id="line",
                           raw_state="GREEN", phase="ALLOW", camera_stamp_ns=10)
            (run/"signal_status.json").write_text(json.dumps(control))
            store = SignalDebugStore(root/"debug")
            report = dict(run_id=run.name, camera_stamp_ns=10, line_id="line", fresh=True,
                source="live", time=now, frame_wall=now, raw_state="GREEN", slots={},
                camera_id=4, camera_view="front_low")
            images = {view: b"one-frame-"+view.encode() for view in VIEWS}
            snapshot = store.publish(report, DEFAULT_CONFIG, images)
            job = dict(running=True, action="autodrive_follow", started_at=now-1.,
                command=["env", "STUDENT_SIGNAL_GUARD=1", "bash", str(web_app.LAUNCH),
                         "0", "90.0", "localization", "two_pc"])
            with patch.object(web_app.JOBS, "snapshot", return_value=job), \
                 patch.object(web_app, "active_drive_directory", side_effect=lambda _job, profile: run if profile == "two_pc" else None), \
                 patch.object(web_app, "SIGNAL_DEBUG", store), \
                 patch.object(web_app.SENSOR_PREVIEW, "sample") as sample:
                yield run, store, report, snapshot, images
                sample.assert_not_called()

    def image_path(self, meta, view="original", profile="two_pc", **overrides):
        query = dict(profile=profile, view=view, slot=meta["slot"], frame=meta["frame_id"])
        query.update(overrides)
        return "/api/signal/image?"+urlencode(query)

    def test_signal_pages_have_profile_controls_and_all_hsv_panels(self):
        for profile in ("single_pc", "two_pc"):
            _, body = self.get("/signals?profile="+profile)
            html = body.decode()
            self.assertIn('data-profile="'+profile+'"', html)
            self.assertNotIn("__PROFILE__", html)
            for view in VIEWS:
                self.assertIn('id="'+view+'"', html)
            self.assertIn('id="signalEnabled"', html)
            self.assertIn('4번 신호등 카메라', html)
            self.assertIn('id="start"', html)
            self.assertIn('id="stop"', html)

    def test_current_profile_serves_same_frame_images_without_camera_binding(self):
        with self.live_view() as (_, _, _, snapshot, images):
            _, data = self.get("/api/signal/status?profile=two_pc")
            result = json.loads(data)
            self.assertTrue(result["available"])
            self.assertEqual(result["perception"]["frame_id"], snapshot["frame_id"])
            for view in VIEWS:
                content_type, image = self.get(self.image_path(snapshot, view))
                self.assertEqual(content_type, "image/jpeg")
                self.assertEqual(image, images[view])

    def test_other_profile_cannot_see_current_run_images(self):
        with self.live_view() as (_, _, _, snapshot, _):
            _, data = self.get("/api/signal/status?profile=single_pc")
            result = json.loads(data)
            self.assertFalse(result["available"])
            self.assertFalse(result["running"])
            self.assertIsNone(result["perception"])
            with self.assertRaises(urllib.error.HTTPError) as error:
                self.get(self.image_path(snapshot, profile="single_pc"))
            self.assertEqual(error.exception.code, 404)

    def test_disabled_or_different_selected_line_hides_previous_hsv_images(self):
        with self.live_view() as (run, _, _, snapshot, _):
            (run/"run.json").write_text('{"signal_enabled":false}')
            _, data = self.get("/api/signal/status?profile=two_pc")
            self.assertFalse(json.loads(data)["available"])
            with self.assertRaises(urllib.error.HTTPError) as error:
                self.get(self.image_path(snapshot))
            self.assertEqual(error.exception.code, 404)
            (run/"run.json").write_text('{"signal_enabled":true}')
            control = json.loads((run/"signal_status.json").read_text())
            control["selected_line_id"] = "new-line"
            (run/"signal_status.json").write_text(json.dumps(control))
            _, data = self.get("/api/signal/status?profile=two_pc")
            self.assertFalse(json.loads(data)["available"])
            with self.assertRaises(urllib.error.HTTPError):
                self.get(self.image_path(snapshot))

    def test_stale_wrong_run_and_old_frame_are_not_served_as_live(self):
        for change in (dict(run_id="previous"), dict(frame_wall=time.time()-5), dict(source="recording"),
                       dict(camera_id=1, camera_view="front"), dict(camera_id=None)):
            with self.subTest(change=change), self.live_view() as (_, store, report, _, images):
                meta = store.publish(dict(report, **change), DEFAULT_CONFIG, images)
                _, data = self.get("/api/signal/status?profile=two_pc")
                self.assertFalse(json.loads(data)["available"])
                with self.assertRaises(urllib.error.HTTPError) as error:
                    self.get(self.image_path(meta))
                self.assertEqual(error.exception.code, 404)
        with self.live_view() as (_, _, _, snapshot, _):
            with self.assertRaises(urllib.error.HTTPError) as error:
                self.get(self.image_path(snapshot, frame="old-frame"))
            self.assertEqual(error.exception.code, 404)

    def test_view_endpoint_validation_and_idle_state(self):
        for path in ("/signals", "/signals?profile=unknown", "/api/signal/status", "/api/signal/image?profile=two_pc&slot=0&view=../file",
                     "/api/signal/image?profile=two_pc&slot=-1&view=red", "/api/signal/image?profile=two_pc&slot=bad&view=red"):
            with self.subTest(path=path), self.assertRaises(urllib.error.HTTPError) as error:
                self.get(path)
            self.assertEqual(error.exception.code, 400)
        with patch.object(web_app.JOBS, "snapshot", return_value={"running": False, "command": []}), \
             patch.object(web_app, "drive_lock_held", return_value=False):
            _, data = self.get("/api/signal/status?profile=two_pc")
            self.assertFalse(json.loads(data)["available"])
            self.assertFalse(json.loads(data)["busy"])

    def test_malformed_or_stale_gate_status_does_not_expose_images(self):
        for timestamp in (None, "invalid", time.time()-5, time.time()+5):
            with self.subTest(timestamp=timestamp), self.live_view() as (run, _, _, _, _):
                control = json.loads((run/"signal_status.json").read_text())
                control["time"] = timestamp
                (run/"signal_status.json").write_text(json.dumps(control))
                _, data = self.get("/api/signal/status?profile=two_pc")
                self.assertEqual(json.loads(data)["control"], {})
                self.assertFalse(json.loads(data)["available"])


if __name__ == "__main__":
    unittest.main()
