"""HTTP contracts for separate dashboards and profile-owned telemetry/controls."""
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from unittest.mock import patch

import competition_profiles
import web_app
from web_sensor_preview import SensorPreview


def job(profile=None, running=False, started=None):
    command = ["bash", "/project/student_v3_follow_claude.sh", "0", "90.0", "localization", profile] if profile else []
    return {"running": running, "action": "autodrive_follow" if profile else None, "command": command,
            "started_at": started, "finished_at": None, "exit_code": None, "elapsed_seconds": 0,
            "logs": ["private " + profile] if profile else []}


class DriveHTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), web_app.Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = "http://127.0.0.1:%d" % cls.server.server_address[1]

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown(); cls.server.server_close(); cls.thread.join()

    def post(self, path, value):
        request = urllib.request.Request(self.base + path, json.dumps(value).encode(), {"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=3) as response:
            return json.load(response)

    def test_separate_pages_and_selection_home(self):
        with urllib.request.urlopen(self.base + "/") as response:
            html = response.read().decode()
        self.assertIn('href="/single-pc"', html)
        self.assertIn('href="/competition"', html)
        for route, name in (("/single-pc", "single_pc"), ("/competition", "two_pc")):
            with self.subTest(route=route), urllib.request.urlopen(self.base + route) as response:
                html = response.read().decode()
                self.assertIn('data-profile="' + name + '"', html)
                self.assertNotIn("__PROFILE__", html)

    def test_explicit_start_profile_and_single_pc_only_collection(self):
        with patch.object(web_app.JOBS, "start") as start:
            for name in ("single_pc", "two_pc"):
                self.assertEqual(self.post("/api/drive/start", {"profile": name})["profile"], name)
                self.assertEqual(start.call_args[0][1][-1], name)
            self.post("/api/drive/start", {"profile": "single_pc", "collect": True})
            self.assertEqual(start.call_args[0][0], "autodrive_collect")
            self.assertEqual(start.call_args[0][1][7], "single_pc")
            count = start.call_count
            for value in ({}, {"profile": "unknown"}, {"profile": "two_pc", "collect": True}, {"profile": "single_pc", "collect": "yes"}):
                with self.subTest(value=value), self.assertRaises(urllib.error.HTTPError) as error:
                    self.post("/api/drive/start", value)
                self.assertEqual(error.exception.code, 400)
            self.assertEqual(start.call_count, count)

    def test_other_profile_cannot_stop_active_drive(self):
        with patch.object(web_app.JOBS, "snapshot", return_value=job("two_pc", True)), \
                patch.object(web_app.JOBS, "stop", return_value=True) as stop:
            with self.assertRaises(urllib.error.HTTPError) as error:
                self.post("/api/drive/stop", {"profile": "single_pc"})
            self.assertEqual(error.exception.code, 400)
            stop.assert_not_called()
            self.assertTrue(self.post("/api/drive/stop", {"profile": "two_pc"})["ok"])
            stop.assert_called_once_with()

    def test_telemetry_requires_profile_and_camera_never_falls_back_to_recordings(self):
        with patch.object(web_app.JOBS, "snapshot", return_value=job()), \
                patch.object(web_app, "drive_lock_held", return_value=False), \
                patch.object(web_app, "SENSOR_PREVIEW", SensorPreview()), \
                patch.object(web_app, "latest_preview") as recording:
            for route in ("/api/drive-status", "/api/drive-status?profile=unknown", "/api/drive-camera?profile=unknown&view=front"):
                with self.subTest(route=route), self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(self.base + route)
                self.assertEqual(error.exception.code, 400)
            with self.assertRaises(urllib.error.HTTPError) as error:
                urllib.request.urlopen(self.base + "/api/drive-camera?profile=two_pc&view=front")
            self.assertEqual(error.exception.code, 404)
            recording.assert_not_called()


class ProfileStatusTests(unittest.TestCase):
    def test_idle_page_hides_other_profiles_logs_and_ros(self):
        profile = competition_profiles.profile("two_pc")
        with patch.object(web_app.JOBS, "snapshot", return_value=job("single_pc")), \
                patch.object(web_app, "drive_lock_held", return_value=False), \
                patch.object(web_app.SENSOR_PREVIEW, "sample", return_value={"profile": "two_pc"}) as sample, \
                patch.object(web_app, "drive_camera", return_value=None):
            result = web_app.drive_status(profile)
        self.assertEqual(result["ros"], {})
        self.assertEqual(result["job"]["logs"], [])
        self.assertIsNone(result["job"]["action"])
        self.assertEqual(result["sensor_input"]["profile"], "two_pc")
        self.assertEqual(sample.call_args.args[0]["id"], "two_pc")

    def test_running_profile_reads_own_run_and_blocks_other_page(self):
        now = time.time()
        with tempfile.TemporaryDirectory() as directory:
            data = Path(directory)
            run = data / "autodrive/follow_runs/run-test"
            run.mkdir(parents=True)
            (data / "autodrive/drive_run.json").write_text(json.dumps({"nonce": "run-test", "started_wall_ns": int((now - 1) * 1e9)}))
            (run / "network_profile.json").write_text('{"id":"two_pc"}')
            (run / "loc_status.json").write_text(json.dumps({"time": now, "ego_hz": 15, "ego": {"speed_kmh": 12}}))
            with patch.object(web_app, "DATA", data), \
                    patch.object(web_app.JOBS, "snapshot", return_value=job("two_pc", True, now - 2)), \
                    patch.object(web_app, "drive_lock_held", return_value=True), \
                    patch.object(web_app.SENSOR_PREVIEW, "sample") as sample:
                own = web_app.drive_status(competition_profiles.profile("two_pc"))
                other = web_app.drive_status(competition_profiles.profile("single_pc"))
                self.assertEqual(own["ros"]["ego"]["speed_kmh"], 12)
                self.assertEqual(own["monitor_mode"], "driving")
                self.assertEqual(other["ros"], {})
                self.assertEqual(other["job"]["logs"], [])
                self.assertEqual(other["other_job"]["profile"], "two_pc")
                self.assertTrue(other["busy"])
                sample.assert_not_called()
                (run / "loc_status.json").write_text(json.dumps({"time": now - 10, "ego_hz": 15}))
                self.assertEqual(web_app.drive_status(competition_profiles.profile("two_pc"))["ros"], {})

    def test_cameras_do_not_cross_profile_boundaries(self):
        preview = SensorPreview()
        preview.frames[("single_pc", "front")] = (time.time(), b"local")
        preview.frames[("two_pc", "front")] = (time.time(), b"competition")
        with patch.object(web_app, "SENSOR_PREVIEW", preview), \
                patch.object(web_app, "drive_lock_held", return_value=False):
            for name, expected in (("single_pc", b"local"), ("two_pc", b"competition")):
                self.assertEqual(web_app.drive_camera(competition_profiles.profile(name), "front", job()), expected)
            self.assertIsNone(web_app.drive_camera(competition_profiles.profile("two_pc"), "../front", job()))


if __name__ == "__main__":
    unittest.main()
