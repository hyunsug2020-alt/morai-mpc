"""A future speed setting must not change scoring of an ongoing run."""
import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import competition_speed as speed


class CompetitionSpeedTests(unittest.TestCase):
    def evaluate_clean_run(self, run_kmh):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            state_path = root / "state.json"
            state = dict(speed.DEFAULT, level=3, base_kmh=30., history=[])
            state_path.write_text(json.dumps(state))
            (root / "speed_plan.json").write_text(json.dumps(dict(
                level=3, normal_kmh=run_kmh, curve_alat_max=3.029)))
            with (root / "mpc.csv").open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(["mode", "t", "v_kmh", "cte", "steer_cmd",
                                 "path_curvature", "x", "y"])
                for index in range(651):
                    writer.writerow(["NORMAL", index * .2, run_kmh, .01, 0., 0., 0., 0.])
            with patch.object(speed, "STATE", str(state_path)), \
                    patch.object(speed, "section_metrics", return_value={}), \
                    contextlib.redirect_stdout(io.StringIO()):
                speed.evaluate(SimpleNamespace(run_dir=str(root)))
            return json.loads((root / "speed_eval.json").read_text()), json.loads(state_path.read_text())

    def test_old_run_keeps_its_target_and_cannot_promote_future_speed(self):
        result, state = self.evaluate_clean_run(28.94)
        self.assertEqual(result["verdict"], "pass")
        self.assertEqual(result["normal_kmh"], 28.94)
        self.assertEqual(result["next_normal_kmh"], 34.73)
        self.assertEqual(state["level"], 3)
        self.assertEqual(state["base_kmh"], 30.)

    def test_new_run_can_still_advance_the_existing_ladder(self):
        result, state = self.evaluate_clean_run(34.73)
        self.assertEqual(result["verdict"], "pass")
        self.assertEqual(result["normal_kmh"], 34.73)
        self.assertEqual(state["level"], 4)
        self.assertEqual(result["next_normal_kmh"], 35.)


if __name__ == "__main__":
    unittest.main()
