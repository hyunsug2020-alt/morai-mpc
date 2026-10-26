"""v6 그림자 실행 검사.

1) 단위 (어디서나): 체크포인트 미설정/잘못된 경로 → GUI에 영향 없이 꺼짐, 오류 반복 시 스스로 꺼짐, 기록 요약 형식.
2) 재생 (데이터 있을 때): 녹화 회차 프레임을 sidecar와 같은 생성기로 masks.npz 에 계속 써 넣고, 녹화 ego를 GUI ego
   샘플 형식(monotonic received)으로 넣어 실제 워커를 돌린다 → 정상 판단(UNKNOWN 없음), 마스크 멈추면 판단 보류.
   환경변수: V6_TEST_SESSION(녹화 회차 dir), V6_TEST_CHECKPOINT(v6 best.pt), V6_TEST_SEMANTIC_MODEL(best_fp16.onnx),
            V6_TEST_PIPELINE(semantic/pipeline dir)
"""
import csv
import json
import os
import sys
import tempfile
import threading
import time
import unittest
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from v6 import shadow_runtime as sr  # noqa: E402


class FakeReceiver:
    def __init__(self, samples=()):
        self.lock = threading.Lock()
        self.samples = deque(samples, maxlen=400)


class FakeRuntime:
    def __init__(self, samples=()):
        self.ego_receiver = FakeReceiver(samples)
        self.waypoint_xy = np.array([[0.0, 0.0], [100.0, 0.0]])


FAKE_CHILD = """
import sys, time, json
from v6 import shadow_child

class Fake:
    def __init__(self, *a, **k): pass
    def start(self): pass
    def close(self): pass
    def inputs(self, samples, **k):
        return ("packed",), None
    def infer_packed(self, packed):
        return {"ok": True, "action": "KEEP_LANE", "action_probabilities": {"KEEP_LANE": 1.0}, "concepts": {},
                "path": [[0.0, 0.0, 0.0]] * 64, "infer_ms": 1.0}

class Boom(Fake):
    def infer_packed(self, *a, **k): raise RuntimeError("boom")

sys.exit(shadow_child.main(sys.argv[2:], shadow_factory={"fake": Fake, "boom": Boom}[sys.argv[1]]))
"""


def fake_child_cmd(tmpdir, kind="fake"):
    path = os.path.join(tmpdir, "fake_child.py")
    with open(path, "w") as f:
        f.write(FAKE_CHILD)
    return [sys.executable, path, kind]


def tick(rt, seconds, hz=15.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        sr.notify_v6_idle(rt); time.sleep(1 / hz)


def read_log(path):
    try:
        return [json.loads(l) for l in open(path)]
    except FileNotFoundError:
        return []


class InputGuardTests(unittest.TestCase):
    """V6Shadow.inputs 의 멈춤 감지 (모델 없이)."""
    def make(self, shm):
        from v6.shadow import V6Shadow, MaskBuffer
        from v6.runtime_inputs import route_cumulative
        sh = V6Shadow.__new__(V6Shadow)
        sh.masks = MaskBuffer(shm)
        sh.route_xy = np.stack([np.linspace(0, 200, 101), np.zeros(101)], -1)
        sh.route_cum = route_cumulative(sh.route_xy)
        now_ns = time.time_ns()
        ids = np.zeros((180, 320), np.uint8)
        mono = time.monotonic()
        for v in ("left", "front", "right"):
            for k in range(8):
                sh.masks.frames[v].append((now_ns - (7 - k) * 66_000_000, ids))
            sh.masks.view_advance_mono[v] = mono
        sh.masks.last_update_mono = mono
        return sh

    def ego(self, last_age):
        now = time.monotonic()
        return [{"received": now - last_age - 0.05 * (40 - i), "x": 0.5 * i, "y": 0.0, "heading_deg": 0.0} for i in range(41)]

    def test_ok_when_fresh(self):
        with tempfile.TemporaryDirectory() as d:
            sh = self.make(d)
            packed, why = sh.inputs(self.ego(0.02), now=time.monotonic(), max_mask_age_s=0.5, max_ego_age_s=0.3,
                                    frame_lag_correction=True)
            self.assertIsNone(why); self.assertEqual(packed[0].shape, (4, 3, 180, 320))

    def test_ego_stale(self):
        with tempfile.TemporaryDirectory() as d:
            _, why = self.make(d).inputs(self.ego(1.0), now=time.monotonic(), max_mask_age_s=0.5, max_ego_age_s=0.3)
            self.assertEqual(why, "ego_stale")

    def test_single_camera_stale(self):
        with tempfile.TemporaryDirectory() as d:
            sh = self.make(d)
            sh.masks.view_advance_mono["left"] = time.monotonic() - 2.0
            _, why = sh.inputs(self.ego(0.02), now=time.monotonic(), max_mask_age_s=0.5, max_ego_age_s=0.3)
            self.assertEqual(why, "camera_stale:left")

    def test_sidecar_stopped(self):
        with tempfile.TemporaryDirectory() as d:
            sh = self.make(d)
            sh.masks.last_update_mono = time.monotonic() - 2.0
            _, why = sh.inputs(self.ego(0.02), now=time.monotonic(), max_mask_age_s=0.5)
            self.assertEqual(why, "masks_stale")

    def test_mask_buffer_reads_only_new_files(self):
        from v6.shadow import MaskBuffer
        with tempfile.TemporaryDirectory() as d:
            mb = MaskBuffer(d)

            def write(seq, stamp):
                out = {"seq": np.int64(seq), "wall": np.float64(time.time())}
                for v in ("front", "left", "right"):
                    out[f"{v}_mask"] = np.zeros((384, 640), np.uint8)
                    out[f"{v}_place"] = np.array([0, 12, 640, 360, 640, 360], np.int32)
                    out[f"{v}_stamp_ns"] = np.int64(stamp)
                np.savez(Path(d) / ".t.npz", **out); os.replace(Path(d) / ".t.npz", Path(d) / "masks.npz")
            write(1, 1_000_000_000); mb.poll_once(); mb.poll_once()
            self.assertEqual([len(mb.frames[v]) for v in ("left", "front", "right")], [1, 1, 1])
            write(2, 1_000_000_000); mb.poll_once()          # 새 seq 지만 같은 프레임 → 추가 안 함
            self.assertEqual(len(mb.frames["front"]), 1)
            write(3, 1_066_000_000); time.sleep(0.01); mb.poll_once()
            self.assertEqual(len(mb.frames["front"]), 2)
            # Atomic replacement may share the old modification timestamp.
            # Its new inode must still deliver the new camera observation.
            old = (Path(d) / 'masks.npz').stat()
            write(4, 1_132_000_000)
            os.utime(Path(d) / 'masks.npz', ns=(old.st_atime_ns, old.st_mtime_ns))
            mb.poll_once()
            self.assertEqual(len(mb.frames['front']), 3)


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.env = {"STUDENT_V6_CHECKPOINT": "x", "STUDENT_LOG_DIR": self.tmp.name, "STUDENT_V6_HZ": "30"}
        samples = [{"received": time.monotonic(), "x": 0.0, "y": 0.0, "heading_deg": 0.0}]
        self.rt = FakeRuntime(samples)

    def tearDown(self):
        sr.stop_v6_shadow(self.rt)
        self.tmp.cleanup()

    def test_disabled_without_checkpoint(self):
        rt = FakeRuntime()
        self.assertIsNone(sr.start_v6_shadow(rt, environ={}))
        self.assertIsNone(rt.v6_latest)
        self.assertEqual(sr.v6_record_fields(rt), {})
        sr.notify_v6_idle(rt); sr.stop_v6_shadow(rt)

    def test_no_gpu_work_without_gui_signal(self):
        b = sr.start_v6_shadow(self.rt, environ=self.env, child_cmd=fake_child_cmd(self.tmp.name))
        time.sleep(1.5)                                          # 신호 없음 → 적재·추론 없음
        self.assertEqual(read_log(b.log_path), [])
        self.assertIsNone(b.child.poll())

    def test_runs_only_right_after_gui_signal(self):
        b = sr.start_v6_shadow(self.rt, environ=self.env, child_cmd=fake_child_cmd(self.tmp.name))
        threading.Thread(target=lambda: [self.rt.ego_receiver.samples.append(
            {"received": time.monotonic(), "x": 0.0, "y": 0.0, "heading_deg": 0.0}) or time.sleep(0.02) for _ in range(150)],
            daemon=True).start()
        tick(self.rt, 3.0)
        log = read_log(b.log_path)
        self.assertGreater(len(log), 20, log[:3])
        self.assertTrue(all(r["slot_delay_ms"] <= 15.0 for r in log), [r["slot_delay_ms"] for r in log])
        self.assertTrue(all(r["ok"] for r in log))
        time.sleep(0.2)
        self.assertTrue(sr.v6_record_fields(self.rt)["v6"]["ok"])
        self.assertEqual(sr.v6_record_fields(self.rt)["v6"]["action"], "KEEP")

    def test_stop_cleans_up(self):
        b = sr.start_v6_shadow(self.rt, environ=self.env, child_cmd=fake_child_cmd(self.tmp.name))
        tick(self.rt, 0.5)
        sr.stop_v6_shadow(self.rt)
        self.assertIsNotNone(b.child.poll())
        self.assertFalse(os.path.exists(b.dir))
        self.assertFalse(b.is_alive())
        self.assertIsNone(self.rt.v6_worker)

    def test_repeated_errors_end_child(self):
        b = sr.start_v6_shadow(self.rt, environ=self.env, child_cmd=fake_child_cmd(self.tmp.name, "boom"))
        tick(self.rt, 3.0, hz=30)
        time.sleep(0.5)
        self.assertEqual(b.child.poll(), 3)
        log = read_log(b.log_path)
        self.assertTrue(log[-1].get("disabled"))
        self.assertTrue(self.rt.v6_latest["reason"].startswith(("child_exit", "error")))

    def test_bad_checkpoint(self):
        env = dict(self.env, STUDENT_V6_CHECKPOINT="/nonexistent/best.pt", STUDENT_V6_DEVICE="cpu")
        t0 = time.monotonic()
        b = sr.start_v6_shadow(self.rt, environ=env)
        self.assertLess(time.monotonic() - t0, 1.0)              # GUI 시작을 늦추지 않음
        stop = threading.Event()
        threading.Thread(target=lambda: [sr.notify_v6_idle(self.rt) or time.sleep(1 / 15) for _ in iter(lambda: stop.is_set(), True)],
                         daemon=True).start()
        try:
            self.assertEqual(b.child.wait(timeout=60), 2)
        finally:
            stop.set()
        self.assertEqual(read_log(b.log_path)[-1]["reason"], "load_failed")

    def test_child_exits_when_parent_gone(self):
        from v6 import shadow_child
        d = self.tmp.name
        idle = os.path.join(d, "idle.bin"); open(idle, "wb").write(b"\0" * 32)
        np.save(os.path.join(d, "r.npy"), np.zeros((2, 2)))
        rc = shadow_child.main(["--checkpoint", "x", "--route", os.path.join(d, "r.npy"), "--ego", os.path.join(d, "e.npz"),
                                "--idle", idle, "--latest", os.path.join(d, "l.json"), "--log", os.path.join(d, "c.jsonl"),
                                "--shm", d, "--parent-pid", "999999999"])
        self.assertEqual(rc, 0)


def _require(*names):
    missing = [n for n in names if not os.environ.get(n)]
    return unittest.skipIf(missing, f"replay data not configured: {missing}")


@_require("V6_TEST_SESSION", "V6_TEST_CHECKPOINT", "V6_TEST_SEMANTIC_MODEL", "V6_TEST_PIPELINE")
class ReplayTest(unittest.TestCase):
    def test_worker_on_recorded_session(self):
        from v6.shadow import V6Shadow  # noqa: F401  (패키지 import 확인)
        sys.path.insert(0, os.environ["V6_TEST_PIPELINE"])
        session = Path(os.environ["V6_TEST_SESSION"])
        gen = _mask_generator()
        # 녹화 프레임 (시각순) 과 ego
        frames = {}
        for line in open(session / "manifest.jsonl"):
            r = json.loads(line)
            if r.get("view") in ("front", "left", "right") and r.get("rgb") and (session / r["rgb"]).is_file():
                frames.setdefault(r["view"], []).append((int(r["rgb_timestamp_ns"]), r["rgb"]))
        for v in frames:
            frames[v].sort()
        ego = list(csv.DictReader(open(session / "ego.csv")))
        start_us = int(ego[0]["timestamp_us"]) + 60_000_000          # 회차 시작 60 s 뒤부터 재생
        ego_rows = [e for e in ego if int(e["timestamp_us"]) >= start_us - 5_000_000]
        route = json.load(open(session / "route.json"))
        rt = FakeRuntime()
        rt.waypoint_xy = np.asarray([p[:2] for p in route["poses_xyz_qxyzw"]], np.float64)
        with tempfile.TemporaryDirectory() as shm, tempfile.TemporaryDirectory() as logs:
            env = {"STUDENT_V6_CHECKPOINT": os.environ["V6_TEST_CHECKPOINT"], "STUDENT_LOG_DIR": logs, "SEMANTIC_FEED_DIR": shm,
                   "STUDENT_V6_HZ": "10"}
            wall0, mono0, rec0 = time.time(), time.monotonic(), start_us / 1e6
            stop = threading.Event()

            def feed():   # 녹화 시간 그대로 ego·마스크 공급 (실시간)
                i, seq = 0, 0
                idx = {v: 0 for v in frames}
                while not stop.is_set():
                    rec_now = rec0 + (time.monotonic() - mono0)
                    while i < len(ego_rows) and int(ego_rows[i]["timestamp_us"]) / 1e6 <= rec_now:
                        e = ego_rows[i]
                        with rt.ego_receiver.lock:
                            rt.ego_receiver.samples.append({"received": mono0 + int(e["timestamp_us"]) / 1e6 - rec0,
                                                            "x": float(e["x"]), "y": float(e["y"]), "heading_deg": float(e["heading_deg"])})
                        i += 1
                    latest = {}
                    for v, lst in frames.items():
                        while idx[v] + 1 < len(lst) and lst[idx[v] + 1][0] / 1e9 <= rec_now:
                            idx[v] += 1
                        latest[v] = lst[idx[v]]
                    if all(st / 1e9 <= rec_now for st, _ in latest.values()):
                        seq += 1
                        out = {"seq": np.int64(seq), "wall": np.float64(time.time())}
                        for (v, (st, rel)), (mask, place) in zip(latest.items(), gen.run([(v, (session / rel).read_bytes()) for v, (st, rel) in latest.items()])):
                            out[f"{v}_mask"] = mask; out[f"{v}_place"] = place; out[f"{v}_stamp_ns"] = np.int64(st)
                        tmp = Path(shm) / ".masks.tmp.npz"
                        np.savez(tmp, **out); os.replace(tmp, Path(shm) / "masks.npz")
                    time.sleep(0.03)

            feeder = threading.Thread(target=feed, daemon=True)
            feeder.start()
            time.sleep(2.0)                                   # 이력 2 s 쌓기
            w = sr.start_v6_shadow(rt, environ=env)
            self.assertIsNotNone(w)
            log_path = w.log_path
            tick_stop = threading.Event()

            def gui_ticks():   # GUI infer 루프 흉내: 15 Hz 로 '실시간 추론 끝' 신호
                while not tick_stop.is_set():
                    sr.notify_v6_idle(rt); tick_stop.wait(1 / 15)
            ticker = threading.Thread(target=gui_ticks, daemon=True); ticker.start()
            time.sleep(10.0)                                  # 로드(백그라운드) 포함
            stop.set(); feeder.join(timeout=3)
            time.sleep(1.5)                                   # 마스크 멈춤 → 판단 보류 확인
            tick_stop.set(); sr.stop_v6_shadow(rt)
            self.assertIsNotNone(w.child.poll())
            lines = read_log(log_path)
        ok = [r for r in lines if r.get("ok")]
        self.assertGreater(len(ok), 40, f"too few decisions: {len(ok)} / {len(lines)}; first={lines[:2]}")
        self.assertTrue(all(r["action"] != "UNKNOWN" for r in ok))
        self.assertTrue(all(len(r["path_xy"]) == 12 for r in ok))   # 0.5 s ~ 6.0 s
        self.assertTrue(any((r.get("reason") or "").startswith(("masks_stale", "camera_stale", "ego_stale")) for r in lines[-5:]), lines[-5:])
        self.assertTrue(all(r["slot_delay_ms"] <= 15.0 for r in ok))
        ms = sorted(r["infer_ms"] for r in ok)
        print(json.dumps({"decisions": len(ok), "actions": {a: sum(r["action"] == a for r in ok) for a in {r["action"] for r in ok}},
                          "infer_ms_p50": ms[len(ms) // 2], "infer_ms_p95": ms[int(len(ms) * 0.95)]}))


def _mask_generator():
    """sidecar 와 같은 마스크 생성 (semantic_sidecar.py 와 동일 전처리)."""
    import importlib.util
    here = Path(os.environ.get("V6_TEST_MASKS_PY", Path(__file__).resolve().parent / "v6" / "_test_semantic_masks.py"))
    spec = importlib.util.spec_from_file_location("semantic_masks", here)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod.SidecarMasks(os.environ["V6_TEST_SEMANTIC_MODEL"], os.environ["V6_TEST_PIPELINE"])


if __name__ == "__main__":
    unittest.main()
