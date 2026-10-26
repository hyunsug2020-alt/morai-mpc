"""Integration checks for explicit profiles and sensor-only web collection."""
import fcntl
import importlib.util
import json
import os
import shutil
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import competition_profiles as profiles
import web_app


class ProfileTests(unittest.TestCase):
    def test_one_pc_changes_only_ip_fields(self):
        sensor, network = profiles.originals()
        one = profiles.profile("single_pc")
        two = profiles.profile("two_pc")
        self.assertEqual(json.loads(Path(one["sensor_file"]).read_bytes()), profiles.loopback(sensor))
        self.assertEqual(json.loads(Path(one["network_file"]).read_bytes()), profiles.single_pc_network(network, sensor))
        self.assertEqual(json.loads(Path(two["sensor_file"]).read_bytes()), sensor)
        self.assertEqual(json.loads(Path(two["network_file"]).read_bytes()), network)
        for key in ("gps_port", "imu_port", "lidar_port", "status_port", "collision_port", "control_port", "control_bind_port", "cameras", "aux_camera"):
            self.assertEqual(one[key], two[key])
        self.assertEqual((two["simulator_ip"], two["local_ip"]), ("192.168.0.1", "192.168.0.10"))
        self.assertEqual((one["simulator_ip"], one["local_ip"]), ("127.0.0.1", "127.0.0.1"))

    def test_registered_sensor_bytes_preserve_mounts_and_15hz_rate(self):
        expected = {"single_pc": "bisa_real_sensor_single.json",
                    "two_pc": "bisa_real_sensor_two_pc.json"}
        for name, filename in expected.items():
            with self.subTest(profile=name):
                p = profiles.profile(name)
                path = Path(p["sensor_file"])
                self.assertEqual(path.name, filename)
                self.assertEqual(path.read_bytes(), profiles.original_bytes(filename))
                sensor = json.loads(path.read_bytes())
                self.assertEqual(len(sensor["Lidar3DList"]), 1)
                lidar = sensor["Lidar3DList"][0]["lc"]
                self.assertEqual(lidar["lidar3DType"], 0)
                self.assertEqual(lidar["frequencyIndex"], 2)
                self.assertAlmostEqual(lidar["sensorPeriod"], 1/15)
                self.assertEqual(p["lidar_rpm"], 900)
                aux = next(c for c in sensor["cameraList"] if c["m_SensorUniqueID"] == 4)
                self.assertEqual([float(aux["pos"][k]) for k in ("x", "y", "z")], [.5, 0, 1.25])
                self.assertEqual(float(aux["rot"]["pitch"]), 330)

    def test_ports_and_view_id_swap_match_zip(self):
        p = profiles.profile("two_pc")
        self.assertEqual(p["cameras"], {"front": {"camera_id": 1, "udp_port": 9202},
                                         "left": {"camera_id": 3, "udp_port": 9203},
                                         "right": {"camera_id": 2, "udp_port": 9204}})
        self.assertEqual((p["gps_port"], p["imu_port"], p["status_port"], p["collision_port"]), (9090, 9091, 9099, 9092))

    def test_changed_original_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "originals").mkdir()
            for name in profiles.SOURCE_FILES:
                shutil.copy2(profiles.PRESETS / "originals" / name, root / "originals" / name)
            (root / "originals/bisa_real_sensor_two_pc.json").write_text("{}")
            with patch.object(profiles, "PRESETS", root), self.assertRaisesRegex(ValueError, "원본 변경"):
                profiles.originals()

    def test_generation_contains_exactly_two_files_per_kind(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copytree(profiles.PRESETS / "originals", root / "originals")
            with patch.object(profiles, "PRESETS", root):
                profiles.materialize()
                expected = {name for files in profiles.PROFILE_FILES.values() for name in files.values()}
                self.assertEqual({p.name for p in root.glob("*.json")}, expected)
                before = {p.name: p.read_bytes() for p in root.glob("*.json")}
                profiles.materialize()
                self.assertEqual({p.name: p.read_bytes() for p in root.glob("*.json")}, before)
                for name in profiles.PROFILE_NAMES:
                    profiles.profile(name)

    def test_stale_sensor_is_rejected_and_generation_restores_registered_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copytree(profiles.PRESETS / "originals", root / "originals")
            with patch.object(profiles, "PRESETS", root):
                profiles.materialize()
                for name, files in profiles.PROFILE_FILES.items():
                    with self.subTest(profile=name):
                        path = root / files["sensor_file"]
                        sensor = json.loads(path.read_bytes())
                        sensor["cameraList"][3]["pos"]["x"] = "3.735"
                        path.write_text(json.dumps(sensor))
                        with self.assertRaisesRegex(ValueError, "원본 기준과 다름"):
                            profiles.profile(name)
                        profiles.materialize()
                        self.assertEqual(path.read_bytes(), profiles.original_bytes(path.name))
                        profiles.profile(name)

    def test_install_archives_old_files_and_keeps_exactly_two_per_kind(self):
        spec = importlib.util.spec_from_file_location("install_final_sensor", profiles.ROOT / "morai/install_final_sensor.py")
        installer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(installer)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            presets = root / "morai/final_sensor"
            shutil.copytree(profiles.PRESETS / "originals", presets / "originals")
            (presets / "bisa_real_sensor_single_pc.json").write_bytes(b"obsolete source")
            savefile = root / "SaveFile"
            old = {}
            for kind in ("Network", "Sensor"):
                target = savefile / kind / "25.S4.MolitComp03"
                target.mkdir(parents=True)
                for name in ("legacy.json", "legacy.json.bak.123"):
                    old[target / name] = (kind + name).encode()
                    (target / name).write_bytes(old[target / name])
            changed = savefile / "Sensor/25.S4.MolitComp03" / profiles.PROFILE_FILES["two_pc"]["sensor_file"]
            changed.write_bytes(b"old selected sensor")
            with patch.object(profiles, "PRESETS", presets), patch.object(installer, "ROOT", root):
                installer.install(savefile, prune=True)
                for key, kind in (("network_file", "Network"), ("sensor_file", "Sensor")):
                    target = savefile / kind / "25.S4.MolitComp03"
                    expected = {files[key] for files in profiles.PROFILE_FILES.values()}
                    self.assertEqual({p.name for p in target.iterdir()}, expected)
                    for name in profiles.PROFILE_NAMES:
                        source = Path(profiles.profile(name)[key])
                        self.assertEqual((target / source.name).read_bytes(), source.read_bytes())
                archived = {p.read_bytes() for p in (savefile / "PresetArchive").rglob("*") if p.is_file()}
                self.assertTrue(set(old.values()) <= archived)
                self.assertIn(b"old selected sensor", archived)
                self.assertFalse((presets / "bisa_real_sensor_single_pc.json").exists())
                archived_sources = [p for p in (root / "morai/preset_archive").rglob("*") if p.is_file()]
                self.assertEqual([p.read_bytes() for p in archived_sources], [b"obsolete source"])
                previous = {str(p): p.read_bytes() for p in (savefile / "PresetArchive").rglob("*") if p.is_file()}
                installer.install(savefile, prune=True)
                self.assertEqual({str(p): p.read_bytes() for p in (savefile / "PresetArchive").rglob("*") if p.is_file()}, previous)

    def test_ip_not_assigned_prevents_launch(self):
        with patch.object(profiles.socket, "socket") as factory:
            factory.return_value.__enter__.return_value.bind.side_effect = OSError("not assigned")
            with self.assertRaisesRegex(ValueError, "192.168.0.10.*시작하지 않음"):
                profiles.check_local_ip(profiles.profile("two_pc"))

    def test_calibration_follows_yaw_instead_of_sorted_ids(self):
        from semantic_fusion import load_cameras
        cameras, mount = load_cameras(profiles.profile("single_pc")["sensor_file"])
        self.assertAlmostEqual(cameras["left"]["pos"][1], .65)
        self.assertAlmostEqual(cameras["right"]["pos"][1], -.65)
        self.assertGreater(cameras["left"]["R"].T[1, 0], 0)
        self.assertLess(cameras["right"]["R"].T[1, 0], 0)
        self.assertEqual(list(mount), [1.05, 0.0, 1.35])


class WebTests(unittest.TestCase):
    def test_explicit_buttons_and_collection_route_through_http(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), web_app.Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = "http://127.0.0.1:%d/api/start" % server.server_address[1]

        def start(options, action="autodrive_follow"):
            request = urllib.request.Request(url, json.dumps({"action": action, "options": options}).encode(),
                                             {"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=3) as response:
                return json.load(response)

        try:
            with patch.object(web_app, "MODE", "collect"), patch.object(web_app.JOBS, "start") as job:
                for name in ("single_pc", "two_pc"):
                    # A stale "rule" drive_mode from an old browser is ignored: E2E is the only launch.
                    self.assertTrue(start({"minutes": 1, "network_profile": name, "drive_mode": "rule"})["ok"])
                    command = job.call_args[0][1]
                    self.assertEqual(command[:2], ["env", "STUDENT_SIGNAL_GUARD=1"])
                    command = command[2:]
                    self.assertTrue(command[1].endswith("student_v3_follow_claude.sh"))
                    self.assertEqual(command[2:], ["0", "90.0", "localization", name])
                self.assertTrue(start({"minutes": 1, "ego_source": "ego", "network_profile": "two_pc"}, "autodrive_collect")["ok"])
                command = job.call_args[0][1]
                self.assertEqual(command[:2], ["env", "STUDENT_SIGNAL_GUARD=1"])
                command = command[2:]
                self.assertTrue(command[1].endswith("student_v3_follow_claude.sh"))
                self.assertEqual(command[2:6], ["0", "90.0", "localization", "single_pc"])
                self.assertTrue(command[6].isdecimal())
                self.assertTrue(start({"minutes": 1})["ok"])
                self.assertEqual(job.call_args[0][1][-1], "two_pc")
                calls = job.call_count
                with self.assertRaises(urllib.error.HTTPError) as error:
                    start({"network_profile": "invalid"})
                self.assertEqual(error.exception.code, 400)
                self.assertEqual(job.call_count, calls)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_preset_downloads_match_profiles_and_reject_unknown_paths(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), web_app.Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = "http://127.0.0.1:%d/api/preset?" % server.server_address[1]
        try:
            for name in profiles.PROFILE_NAMES:
                for kind in ("sensor", "network"):
                    path = Path(profiles.profile(name)[kind + "_file"])
                    with urllib.request.urlopen(base + "profile=" + name + "&kind=" + kind) as response:
                        self.assertEqual(response.read(), path.read_bytes())
                        self.assertEqual(response.headers["Content-Disposition"], 'attachment; filename="' + path.name + '"')
            with urllib.request.urlopen(base + "kind=network") as response:
                self.assertEqual(response.read(), Path(profiles.profile("two_pc")["network_file"]).read_bytes())
            for query in ("profile=../originals&kind=network", "profile=two_pc&kind=../network", "profile=two_pc"):
                with self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(base + query)
                self.assertEqual(error.exception.code, 400)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


class CaptureTests(unittest.TestCase):
    def test_four_cameras_saved_with_correct_ids_and_sensor_sources(self):
        sys.path.insert(0, str(HERE / "student_v3_claude"))
        from gui import CameraReceiver
        with tempfile.TemporaryDirectory() as directory, (profiles.ROOT / "data/autodrive/drive.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.dict(os.environ, {"STUDENT_SESSION_DIR": directory, "STUDENT_NETWORK_PROFILE": "single_pc"}):
                receiver = CameraReceiver()
            receiver.start()
            jpeg = b"\xff\xd8test-image\xff\xd9"
            sec = int(time.time())
            packet = b"MOR" + struct.pack("<IIII", sec, 100, 0, len(jpeg)) + jpeg + b"EI"
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    sender.bind(("127.0.0.1", 0))
                    deadline = time.monotonic() + 2
                    while time.monotonic() < deadline and not all(receiver.saved.values()):
                        for _view, port in receiver.capture_views:
                            sender.sendto(packet, ("127.0.0.1", port))
                        time.sleep(.05)
                self.assertTrue(all(receiver.saved.values()), receiver.saved)
            finally:
                receiver.stop_event.set()
                receiver.join(timeout=3)
            self.assertFalse(receiver.is_alive())
            metadata = json.loads((Path(directory) / "session.json").read_text())
            self.assertEqual(metadata["views"]["left"]["camera_id"], 3)
            self.assertEqual(metadata["views"]["right"]["camera_id"], 2)
            self.assertEqual(metadata["views"]["front_low"]["camera_id"], 4)
            self.assertEqual(metadata["ego_source"], "/localization/ego_status")
            self.assertEqual(metadata["object_source"], "/perception/objects")
            self.assertIn("finished_at", metadata)


if __name__ == "__main__":
    unittest.main()
