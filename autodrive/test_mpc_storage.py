"""Per-run controller downloads (the Drive archive/prune half left with cloud_backup.py, 2026-10-11)."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import web_app


class MpcStorageTests(unittest.TestCase):
    def test_runs_keep_separate_files_and_download_the_requested_run(self):
        with tempfile.TemporaryDirectory() as td, patch.object(web_app, "DATA", Path(td)):
            root = Path(td) / "autodrive/mpc_runs"
            for run, stamp in (("run-a", 100), ("run-b", 200)):
                folder = root / "19" / run
                folder.mkdir(parents=True)
                (folder / "run.json").write_text(json.dumps(dict(started_wall_ns=stamp)))
                (folder / "mpc.csv").write_text("t,target_vel\n0," + str(stamp))
            self.assertEqual([r["run"] for r in web_app.mpc_runs()], ["run-b", "run-a"])
            self.assertIn("100", web_app.mpc_file("19", "run-a", "mpc.csv").read_text())
            self.assertIn("200", web_app.mpc_file("19", "run-b", "mpc.csv").read_text())
            for args in (("..", "run-a", "mpc.csv"), ("19", "../../outside", "mpc.csv"),
                         ("19", "run-a", "secret"), ("19", "missing", "mpc.csv")):
                with self.assertRaises(ValueError):
                    web_app.mpc_file(*args)


if __name__ == "__main__":
    unittest.main()
