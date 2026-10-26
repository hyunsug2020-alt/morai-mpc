"""Deletion must protect active runs and stay out of Drive backup queues."""
import fcntl
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import web_app


class SessionDeletionTests(unittest.TestCase):
    def test_finished_capture_and_mpc_deleted_and_number_not_reused(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            raw = root / "raw"
            capture = raw / "19"
            capture.mkdir(parents=True)
            (capture / "session.json").write_text(json.dumps(dict(finished_at="done")))
            (capture / "camera.jpg").write_bytes(b"image")
            mpc = root / "autodrive/mpc_runs/19/run"
            mpc.mkdir(parents=True)
            (mpc / "run.json").write_text(json.dumps(dict(ended_wall_ns=1)))
            (mpc / "mpc.csv").write_text("t,target_vel\n")
            legacy = root / "autodrive/student_gui_19.jsonl"
            legacy.write_text('{"mode":"STUDENT FORCED"}\n')
            result = web_app.delete_local_session("19", data=root, raw=raw)
            self.assertTrue(result["capture"] and result["mpc"])
            self.assertFalse(capture.exists())
            self.assertFalse(mpc.parent.exists())
            self.assertFalse(legacy.exists())
            self.assertTrue((root / "autodrive/cloud_backup/deleted_sessions/19.json").is_file())
            with patch.object(web_app, "RAW", raw), patch.object(web_app, "DATA", root):
                self.assertEqual(web_app.next_session_id(), "20")

    def test_active_and_busy_sessions_are_preserved(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            raw = root / "raw"
            capture = raw / "20"
            capture.mkdir(parents=True)
            (capture / "session.json").write_text(json.dumps(dict(finished_at="done")))
            drive = root / "autodrive/drive_run.json"
            drive.parent.mkdir(parents=True)
            drive.write_text(json.dumps(dict(session_id="20", owner_pid=os.getpid())))
            with self.assertRaisesRegex(RuntimeError, "주행 중"):
                web_app.delete_local_session("20", data=root, raw=raw)
            self.assertTrue(capture.exists())
            drive.unlink()
            lock_path = root / "autodrive/cloud_backup/session_locks/20.lock"
            lock_path.parent.mkdir(parents=True, exist_ok=True)
            with lock_path.open("a") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX)
                with self.assertRaisesRegex(RuntimeError, "Drive 백업 중"):
                    web_app.delete_local_session("20", data=root, raw=raw)
            status = root / "autodrive/cloud_backup/status.json"
            status.write_text(json.dumps(dict(session="20", phase="uploading")))
            with self.assertRaisesRegex(RuntimeError, "Drive 백업 중"):
                web_app.delete_local_session("20", data=root, raw=raw)
            self.assertTrue(capture.exists())


if __name__ == "__main__":
    unittest.main()
