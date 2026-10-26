"""v6 judgement child for driving use (STUDENT_PLANNER=model in the Claude working copy).

The generated v6/ package (from the v6.3 model bundle) is used unchanged. Its child
writes only {action, p} to latest.json; the planner also needs the full predicted
path, the class probabilities and the ego pose the path is relative to. This wrapper
reuses v6.shadow_child.Child and only extends what it records:

  latest.json  = the original fields + action_full, probabilities, path (64 x 2,
                 0.1 s steps), dt, pose {x, y, heading_deg, t_mono}
  <log>.jsonl  = the original record + pose (the 0.5 s path_xy is kept as before)

The pose is the resampled ego pose at the model's reference time, captured from the
same resample_history call V6Shadow.inputs makes (time base: time.monotonic(), the
GUI ego receiver's "received" clock). Launched by v6.shadow_runtime with
child_cmd=[python, "-m", "v6_ext_child"].
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import numpy as np

from v6 import shadow as _shadow
from v6 import shadow_child as _child


class PoseRecorder:
    """Wrap resample_history as V6Shadow.inputs calls it; keep the last decision pose."""

    def __init__(self, function):
        self.function = function
        self.pose = None

    def __call__(self, times_s, x, y, yaw_rad, t_end_s, *args, **kwargs):
        xy, yaw = self.function(times_s, x, y, yaw_rad, t_end_s, *args, **kwargs)
        times = np.asarray(times_s, dtype=float)
        reference = float(t_end_s)
        if len(times):
            reference = float(min(max(reference, times[0]), times[-1]))   # resample_history clips the same way
        heading = (math.degrees(float(yaw[-1])) + 180.) % 360. - 180.
        self.pose = dict(x=float(xy[-1][0]), y=float(xy[-1][1]), heading_deg=heading, t_mono=reference)
        return xy, yaw


RECORDER = PoseRecorder(_shadow.resample_history)
_shadow.resample_history = RECORDER


class DrivingChild(_child.Child):
    def step(self):
        samples = self.ego_samples()
        if not samples:
            return {"ok": False, "reason": "no_ego"}
        RECORDER.pose = None
        packed, why = self.shadow.inputs(samples, time_key="received", now=time.monotonic(), **self.checks)   # CPU
        if packed is None:
            return {"ok": False, "reason": why}
        pose = RECORDER.pose
        if not self.in_window():                         # GPU window passed: try the next idle slot
            return None
        started = time.monotonic()
        result = self.shadow.infer_packed(packed)        # GPU
        result["slot_delay_ms"] = round((started - self.last_idle) * 1e3, 2)
        path = np.asarray(result.pop("path"), dtype=float).reshape(-1, 3)
        result["path_xy"] = [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in path[4::5]]   # 0.5 s, as before
        result["path_full"] = [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in path]
        config = getattr(getattr(self.shadow, "runner", None), "config", None)
        result["dt"] = float(getattr(config, "trajectory_dt", .1))
        result["pose"] = pose
        result["action_probabilities"] = {k: round(float(v), 4) for k, v in result["action_probabilities"].items()}
        result["concepts"] = {k: round(float(v), 3) for k, v in result["concepts"].items()}
        result["infer_ms"] = round(float(result["infer_ms"]), 2)
        return result

    def write(self, result):
        path_full = result.pop("path_full", None)        # latest.json only; keeps the log line small
        try:
            self.log.write(json.dumps(result, ensure_ascii=False) + "\n")
        except (OSError, ValueError, TypeError):
            pass
        ok = bool(result.get("ok"))
        probabilities = result.get("action_probabilities") or {}
        latest = {"t": result["t"], "ok": ok,
                  "action": _child.ACTION_SHORT.get(result.get("action"), result.get("action")) if ok else None,
                  "p": round(max(probabilities.values()), 3) if ok and probabilities else None,
                  "reason": None if ok else result.get("reason"),
                  "action_full": result.get("action") if ok else None,
                  "probabilities": probabilities if ok else None,
                  "path": path_full if ok else None,
                  "dt": result.get("dt"),
                  "pose": result.get("pose") if ok else None,
                  "infer_ms": result.get("infer_ms"),
                  "slot_delay_ms": result.get("slot_delay_ms")}
        tmp = self.a.latest + ".tmp"
        try:
            with open(tmp, "w") as handle:
                json.dump(latest, handle)
            os.replace(tmp, self.a.latest)
        except (OSError, ValueError, TypeError):
            pass


def main(argv=None, shadow_factory=None):
    return DrivingChild(_child.parse(argv), shadow_factory).run()


if __name__ == "__main__":
    sys.exit(main())
