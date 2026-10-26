"""Real camera regressions for semantic signal/map association and lamp layout."""
import copy
import json
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from gui import HdMapLanes
from competition_profiles import profile
from signal_control.mapped_housing import MappedSignalHousing
from signal_control.runtime import SignalController, SignalFrameWorker, mapped_signal_lines
from signal_control.pipeline import SignalFramePipeline
from signal_control.signal_config import DEFAULT_CONFIG
from signal_control.signal_logic import mapped_lamp_bounds
from test_signal_control import lamps

HERE = Path(__file__).resolve().parent


class MappedHousingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lanes = HdMapLanes(HERE/"link_set.json")
        route = json.loads((HERE/"global_path.json").read_bytes())
        cls.lanes.set_route(np.array([[p["x"], p["y"]] for p in route["waypoints"]]))
        cls.lines = mapped_signal_lines(cls.lanes)
        cls.projector = MappedSignalHousing(cls.lanes, profile("single_pc")["sensor_file"])

    def context(self):
        ego = dict(x=-60.18, y=-38.75, heading_deg=89.9, speed_kmh=0.)
        line = next(l for l in self.lines if l["traffic_light_id"] == "C1256W000016")
        return ego, line, self.projector.project(ego, line)

    def test_all_five_map_signals_validate_with_their_actual_lamp_counts(self):
        self.projector.validate(self.lines)
        layouts = [self.projector.project(dict(x=l["stop_line_xy"][0], y=l["stop_line_xy"][1],
                    heading_deg=90.), l)["layout"] for l in self.lines]
        self.assertEqual(layouts, ["three", "three", "four", "four", "four"])
        other = MappedSignalHousing(self.lanes, profile("two_pc")["sensor_file"])
        ego, line, context = self.context()
        self.assertEqual(other.project(ego, line), context)

    def test_projection_includes_real_signal_and_excludes_larger_road_false_positive(self):
        _, _, context = self.context()
        x1, y1, x2, y2 = np.array(context["window"])*[640, 480, 640, 480]
        self.assertTrue(x1 < 227 < x2 and y1 < 309 < y2)
        self.assertFalse(x1 < 212 < x2 and y1 < 426 < y2)
        self.assertFalse(x1 < 299 < x2 and y1 < 356 < y2)

    def test_missing_or_inconsistent_map_cannot_silently_select_arbitrary_signal(self):
        original = copy.deepcopy(self.projector.lights)
        try:
            for invalid in ({}, {key: dict(value, link_id_list=[]) for key, value in original.items()},
                            {key: dict(value, point=[0, 0, float("nan")]) for key, value in original.items()}):
                self.projector.lights = invalid
                with self.assertRaises(ValueError):
                    self.projector.validate(self.lines)
        finally:
            self.projector.lights = original

    def test_out_of_view_signal_cannot_be_replaced_by_green_hood_reflection(self):
        line = next(l for l in self.lines if l["traffic_light_id"] == "C1256W000091")
        context = self.projector.project(dict(x=-59.64, y=-156.59, heading_deg=85.8), line)
        self.assertTrue(context["window"] is None or context["window"][3] < .05)
        pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, layout="three", roi=[0., 0., 1., 1.]))
        for _ in range(3):
            report, _, _ = pipeline.process(*lamps("GREEN"), "offline", source="recording", target=context)
            self.assertEqual(report["raw_state"], "UNKNOWN")
            self.assertIsNone(report["target_box"])
            self.assertTrue(report["stop_required"])

    def test_real_semantic_masks_choose_mapped_red_over_larger_yellow_false_positive(self):
        _, _, context = self.context()
        pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, roi=[0., 0., 1., 1.]))
        with np.load(HERE/"test_data/signal_road_reflections_20261020.npz", allow_pickle=False) as data:
            for i in range(3):
                image = cv2.imdecode(data["jpeg_%d" % i], cv2.IMREAD_COLOR)
                report, _, _ = pipeline.process(image, data["mask_%d" % i], "offline",
                    source="recording", target=context)
                self.assertEqual(report["raw_state"], "RED")
                self.assertEqual(report["target_box"], [208, 302, 39, 15])
                self.assertEqual(report["traffic_light_id"], "C1256W000016")
                self.assertTrue(report["stop_required"])

    def test_no_semantic_mask_at_mapped_signal_is_unknown_despite_green_elsewhere(self):
        _, _, context = self.context()
        image, mask = lamps("GREEN")
        pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, roi=[0., 0., 1., 1.]))
        for _ in range(3):
            report, _, _ = pipeline.process(image, mask, "offline", source="recording", target=context)
            self.assertEqual(report["raw_state"], "UNKNOWN")
            self.assertTrue(report["stop_required"])

    def test_same_mapped_housing_can_turn_green_without_selecting_reflections(self):
        _, _, context = self.context()
        with np.load(HERE/"test_data/signal_road_reflections_20261020.npz", allow_pickle=False) as data:
            image = cv2.imdecode(data["jpeg_0"], cv2.IMREAD_COLOR)
            mask = data["mask_0"].copy()
        image[302:317, 208:247] = 25
        cv2.circle(image, (241, 309), 3, (0, 255, 0), -1)
        pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, roi=[0., 0., 1., 1.]))
        for i in range(3):
            report, _, _ = pipeline.process(image, mask, "offline", source="recording", target=context)
            self.assertEqual(report["raw_state"], "GREEN")
            self.assertEqual(report["target_box"], [208, 302, 39, 15])
            self.assertEqual(report["permission_stable"], i == 2)
        image[302:317, 208:247] = 25
        report, _, _ = pipeline.process(image, mask, "offline", source="recording", target=context)
        self.assertEqual(report["raw_state"], "UNKNOWN")
        self.assertTrue(report["stop_required"])

    def small_context(self):
        ego = dict(x=-59.93306915788369, y=-73.35447561867387, heading_deg=90.51808672702289)
        line = next(l for l in self.lines if l["traffic_light_id"] == "C1256W000016")
        return self.projector.project(ego, line)

    def test_first_green_survives_small_semantic_edges_and_braking_pitch(self):
        line = next(l for l in self.lines if l['signal_index'] == 1)
        config = dict(DEFAULT_CONFIG, layout='three', roi=[0., 0., 1., 1.],
                      stable_frames=1, permission_policy='green')
        pipeline = SignalFramePipeline(config)
        with np.load(HERE/'test_data/signal_first_green_20261023.npz', allow_pickle=False) as data:
            for i in (11, 12, 13, 18):
                with self.subTest(frame=i):
                    image = cv2.imdecode(data['jpeg_%d' % i], cv2.IMREAD_COLOR)
                    x, y, heading = data['ego_%d' % i]
                    context = self.projector.project(dict(x=x, y=y, heading_deg=heading), line)
                    report, _, _ = pipeline.process(image, data['mask_%d' % i], 'offline',
                        source='recording', target=context)
                    self.assertEqual(report['raw_state'], 'GREEN')
                    self.assertFalse(report['stop_required'])
                    # Losing semantics or the actual colored lamp still stops.
                    missing, _, _ = pipeline.process(image, np.zeros_like(data['mask_%d' % i]),
                        'offline', source='recording', target=context)
                    self.assertTrue(missing['stop_required'])
                    dark, _, _ = pipeline.process(np.zeros_like(image), data['mask_%d' % i],
                        'offline', source='recording', target=context)
                    self.assertTrue(dark['stop_required'])

    def test_unresolved_far_first_signal_cannot_inherit_green_permission(self):
        pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, layout='three', roi=[0., 0., 1., 1.],
            stable_frames=1, permission_policy='green'))
        line = next(l for l in self.lines if l['signal_index'] == 1)
        with np.load(HERE/'test_data/signal_first_green_20261023.npz', allow_pickle=False) as data:
            for i, expected in ((11, 'GREEN'), (0, 'UNKNOWN')):
                x, y, heading = data['ego_%d' % i]
                context = self.projector.project(dict(x=x, y=y, heading_deg=heading), line)
                report, _, _ = pipeline.process(cv2.imdecode(data['jpeg_%d' % i], cv2.IMREAD_COLOR),
                    data['mask_%d' % i], 'offline', source='recording', target=context)
                self.assertEqual(report['raw_state'], expected)
                self.assertEqual(report['stop_required'], expected != 'GREEN')

    def test_small_first_housing_stops_immediately_for_red_yellow_and_stale_frames(self):
        pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, layout='three', roi=[0., 0., 1., 1.],
            stable_frames=1, permission_policy='green'))
        line = next(l for l in self.lines if l['signal_index'] == 1)
        with np.load(HERE/'test_data/signal_first_green_20261023.npz', allow_pickle=False) as data:
            x, y, heading = data['ego_11']
            context = self.projector.project(dict(x=x, y=y, heading_deg=heading), line)
            image = cv2.imdecode(data['jpeg_11'], cv2.IMREAD_COLOR)
            mask = data['mask_11']
            green, _, _ = pipeline.process(image, mask, 'run', target=context, now=100.)
            a, b, w, h = green['target_box']
            for state, color in (('RED', (0, 0, 255)), ('YELLOW', (0, 255, 255))):
                changed = image.copy()
                changed[b:b+h, a:a+w] = 20
                dx, dy, dw, dh = green['slots'][state.lower()]['box']
                changed[b+dy:b+dy+dh, a+dx:a+dx+dw] = color
                report, _, _ = pipeline.process(changed, mask, 'run', target=context, now=100.)
                self.assertEqual(report['raw_state'], state)
                self.assertTrue(report['stop_required'])
            report, _, _ = pipeline.process(image, mask, 'run', target=context,
                                            now=102., frame_wall=100.)
            self.assertEqual(report['raw_state'], 'UNKNOWN')
            self.assertTrue(report['stop_required'])

    def test_real_eleven_pixel_green_uses_mapped_lamp_positions_not_jittered_semantic_edges(self):
        context = self.small_context()
        pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, layout="four", roi=[0., 0., 1., 1.],
                                             stable_frames=1, permission_policy="green"))
        with np.load(HERE/"test_data/signal_small_housing_20261020.npz", allow_pickle=False) as data:
            for i in range(4):
                image = cv2.imdecode(data["jpeg_%d" % i], cv2.IMREAD_COLOR)
                report, _, _ = pipeline.process(image, data["mask_%d" % i], "offline",
                    source="recording", target=context)
                self.assertEqual(report["target_box"], [288, 385, 11, 4])
                self.assertIsNotNone(report["lamp_bounds"])
                self.assertEqual(report["stop_required"], i == 0)
                if i:
                    self.assertEqual(report["raw_state"], "GREEN")
                    # No semantic component must still mean UNKNOWN at the same map position.
                    missing, _, _ = pipeline.process(image, np.zeros_like(data["mask_%d" % i]),
                        "offline", source="recording", target=context)
                    self.assertTrue(missing["stop_required"])

    def test_map_alignment_cannot_turn_arrow_only_into_straight_or_red_green_into_red_left(self):
        context = self.small_context()
        with np.load(HERE/"test_data/signal_small_housing_20261020.npz", allow_pickle=False) as data:
            image = cv2.imdecode(data["jpeg_1"], cv2.IMREAD_COLOR)
            mask = data["mask_1"]
        for policy in ("green", "red_left_only"):
            pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, layout="four", roi=[0., 0., 1., 1.],
                                                 stable_frames=1, permission_policy=policy))
            for state, red, gx in (("LEFT", False, 293), ("GREEN", False, 295),
                                    ("RED_LEFT", True, 293), ("UNKNOWN", True, 295)):
                image[385:389, 288:299] = 20
                image[386:388, gx:gx+2] = (0, 255, 0)
                if red:
                    image[386:388, 288:290] = (0, 0, 255)
                report, _, _ = pipeline.process(image, mask, "offline", source="recording", target=context)
                self.assertEqual(report["raw_state"], state)
                self.assertEqual(report["stop_required"], state != ("GREEN" if policy == "green" else "RED_LEFT"))

    def test_large_map_semantic_disagreement_never_changes_lamp_slots(self):
        self.assertIsNone(mapped_lamp_bounds([20., 0., 31., 4.], 288, 11, "four"))
        self.assertIsNone(mapped_lamp_bounds([float("nan"), 0., 299., 4.], 288, 11, "four"))

    def test_real_small_green_remains_straight_when_semantic_box_gains_background_pixels(self):
        context = self.small_context()
        pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, layout="four", roi=[0., 0., 1., 1.],
                                             stable_frames=1, permission_policy="green"))
        with np.load(HERE/"test_data/signal_small_housing_20261020.npz", allow_pickle=False) as data:
            for i in (1, 2, 3):
                image = cv2.imdecode(data["jpeg_%d" % i], cv2.IMREAD_COLOR)
                for extra in (0, 1, 2, 3):
                    mask = data["mask_%d" % i].copy()
                    mask[385:389, 299:299+extra] = 255
                    report, _, _ = pipeline.process(image, mask, "offline", source="recording", target=context)
                    self.assertEqual(report["raw_state"], "GREEN")
                    self.assertFalse(report["stop_required"])

    def test_colored_pixels_in_extra_semantic_padding_do_not_count_as_lamps(self):
        context = self.small_context()
        pipeline = SignalFramePipeline(dict(DEFAULT_CONFIG, layout="four", roi=[0., 0., 1., 1.],
                                             stable_frames=1, permission_policy="green"))
        with np.load(HERE/"test_data/signal_small_housing_20261020.npz", allow_pickle=False) as data:
            image = cv2.imdecode(data["jpeg_1"], cv2.IMREAD_COLOR)
            mask = data["mask_1"].copy()
        image[385:389, 288:302] = 20
        image[386:388, 299:302] = (0, 255, 0)
        mask[385:389, 299:302] = 255
        report, _, _ = pipeline.process(image, mask, "offline", source="recording", target=context)
        self.assertEqual(report["raw_state"], "UNKNOWN")
        self.assertTrue(report["stop_required"])

    def test_controller_installs_projection_and_three_lamp_config_reaches_debug_snapshot(self):
        receiver = SimpleNamespace(lock=threading.Lock(), frames={}, counts={})
        controller = SignalController(self.lanes, receiver, run_id="test", model=SimpleNamespace(mask=lambda *_: None))
        self.assertIsNotNone(controller.projector)
        ego, line, _ = self.context()
        controller.evaluate(ego)
        self.assertEqual(controller.worker.target_context["traffic_light_id"], line["traffic_light_id"])
        image, mask = lamps("GREEN")
        _, jpeg = cv2.imencode(".jpg", image)
        receiver.frames["front_low"] = [(1, jpeg.tobytes())]
        receiver.counts["front_low"] = [time.monotonic()]
        worker = SignalFrameWorker(receiver, "test", DEFAULT_CONFIG, SimpleNamespace(mask=lambda *_: mask))
        worker.select("three", context=dict(traffic_light_id="three", layout="three",
            window=[.4, .05, .6, .3], expected_box=[600/1280, 100/720, 700/1280, 124/720]))
        self.assertTrue(worker.process_once())
        self.assertEqual(worker.pipeline.config["layout"], "three")
        self.assertEqual(set(worker.snapshot()["slots"]), {"red", "yellow", "green"})


if __name__ == "__main__":
    unittest.main()
