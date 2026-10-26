#!/usr/bin/env python3
"""Semantic ONNX sidecar for the Alpamayo (Student v5) drive.

gui.py owns the MORAI camera UDP ports and overwrites <shm>/frames.npz at 20 Hz with the
latest JPEG + MORAI frame stamp per view. Every new frame set is segmented here:
  * <shm>/masks.npz      model-resolution class masks + letterbox placement + stamps, every frame
                         (read by semantic_fusion.py, which projects LiDAR points onto them)
  * <live>/<view>.jpg    dashboard overlays, throttled (drawing costs as much as inference)
  * <live>/status.json   rates and pixel counts for the dashboard
It never touches driving; with no fresh frames it idles.
"""
import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import torch  # Load CUDA/cuDNN shared libraries used by ONNX Runtime.
import onnxruntime as ort

cv2.setNumThreads(1)
torch.set_num_threads(1)
torch.set_num_interop_threads(1)

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--model", type=Path, required=True)
parser.add_argument("--pipeline", type=Path, required=True, help="dir with run_onnx.py / semantic_common.py")
parser.add_argument("--live", type=Path, required=True, help="dashboard overlay dir")
parser.add_argument("--shm", type=Path, default=Path("/dev/shm/alpamayo_semantic"))
parser.add_argument("--overlay-hz", type=float, default=5.0)
parser.add_argument("--gpu-postprocess", choices=("on", "off"), default="on")
parser.add_argument("--max-fps", type=float, default=10.0, help="GPU frame-set inference rate; keeps renderer headroom")
args = parser.parse_args()
if not 1 <= args.max_fps <= 30:
    parser.error("--max-fps must be between 1 and 30")

# Only background CPU work is moved to E-cores; CUDA submission/ORT stays on fast cores.
original_allowed_cpus = set(os.sched_getaffinity(0))
cpu_thread_registry = {}
def pin_thread(spec, stage):
    if not spec:
        return
    cpus = set()
    for part in spec.split(","):
        bounds = part.strip().split("-")
        lo, hi = int(bounds[0]), int(bounds[-1])
        cpus.update(range(lo, hi + 1))
    if not cpus or not cpus.issubset(original_allowed_cpus):
        raise RuntimeError("Unavailable CPU placement for " + stage + ": " + spec)
    os.sched_setaffinity(0, cpus)
    cpu_thread_registry[stage] = threading.get_native_id()
    print(stage + " CPU placement: " + spec + " thread=" + str(threading.get_native_id()), flush=True)

pin_thread(os.environ.get("SEMANTIC_MAIN_CPUS", ""), "semantic GPU submission")

sys.path.insert(0, str(args.pipeline))
from run_onnx import preprocess, prediction_masks, build_visualization  # noqa: E402
from semantic_common import letterbox, load_class_config  # noqa: E402
from semantic_person_refinement import (small_person_component, context_rois, merge_contexts,
                                       forward_person_rois, recover_person_contexts)  # noqa: E402

VIEWS = ("front", "left", "right")
FRESH_S = 1.0

model = args.model.resolve()
meta = json.loads(model.with_suffix(model.suffix + ".json").read_text(encoding="utf-8"))
width, height = int(meta["input_width"]), int(meta["input_height"])
names = dict(meta.get("view_exclusion_masks") or {})
if meta.get("front_exclusion_mask"):
    names.setdefault("front", meta["front_exclusion_mask"])
masks_raw = {v: cv2.imread(str(model.parent / n), cv2.IMREAD_GRAYSCALE) for v, n in names.items()}
config, classes, _palette, _ids = load_class_config(model.parent / "classes.yaml")
colors = np.asarray([c["color"] for c in classes], np.uint8)[:, ::-1]
class_names = [str(c["name"]) for c in classes]
excluded = tuple(class_names.index(n) for n in (config.get("excluded_classes") or []) if n in class_names)
providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in ort.get_available_providers()]
if "CUDAExecutionProvider" in providers and hasattr(ort, "preload_dlls"):
    ort.preload_dlls()
opts = ort.SessionOptions()
opts.intra_op_num_threads = 2
opts.inter_op_num_threads = 1
opts.add_session_config_entry("session.intra_op.allow_spinning", "0")
opts.add_session_config_entry("session.inter_op.allow_spinning", "0")
session = ort.InferenceSession(str(model), sess_options=opts, providers=providers)
if session.get_providers()[0] != "CUDAExecutionProvider":
    raise RuntimeError("장애물 시맨틱 GPU 추론 초기화 실패")
output_name = session.get_outputs()[0].name
threshold = float(meta.get("threshold", 0.5))
gpu_masks = None
refinement_gpu = None
if args.gpu_postprocess == "on":
    from semantic_gpu_masks import GpuMaskInference
    from camera_config import CAMERA_PAIRS
    active_ids = [[i for i in CAMERA_PAIRS[v]["supervised_class_ids"] if i not in excluded] for v in VIEWS]
    gpu_masks = GpuMaskInference(session, output_name, active_ids, len(class_names), width, height, threshold)
    refinement_gpu = GpuMaskInference(session, output_name, [active_ids[0]]*2,
                                      len(class_names), width, height, threshold)


args.shm.mkdir(parents=True, exist_ok=True)
args.live.mkdir(parents=True, exist_ok=True)
frames_path = args.shm / "frames.npz"
cv2.setNumThreads(2)
MEAN = np.asarray((0.485, 0.456, 0.406), np.float32) * 255.0
STD = np.asarray((0.229, 0.224, 0.225), np.float32) * 255.0
exclusion_cache = {}
ex_small = {}
status = {"model": model.parent.name, "providers": session.get_providers(), "class_names": class_names, "views": {}}


def exclusion_for(view, shape):
    key = (view, shape)
    if key not in exclusion_cache:
        mask = masks_raw.get(view)
        # The drive rig may stream a different resolution than the training rig (and front is decoded at 1/2).
        exclusion_cache[key] = None if mask is None else (
            mask if mask.shape == shape else cv2.resize(mask, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST))
    return exclusion_cache[key]


def fast_preprocess(view, image):
    """Letterbox first, then blank the ego-body mask at model resolution (same result as run_onnx.preprocess
    up to edge pixels, without full-resolution copies)."""
    canvas, place = letterbox(image, width, height, cv2.INTER_LINEAR, 0)
    ex = exclusion_for(view, image.shape[:2])
    if ex is not None:
        small, _ = letterbox(ex, width, height, cv2.INTER_NEAREST, 0)
        canvas[small > 0] = 0
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32)
    return ((rgb - MEAN) / STD).transpose(2, 0, 1), place


def write_atomic(path, data):
    tmp = path.with_name("." + path.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def refine_front_person(item, masks):
    """Use current full-resolution RGB to disambiguate small red posts/people."""
    component = small_person_component(masks[0], item['prepared'][0][1],
                                      item['images'][0].shape)
    if component is None:
        return dict(relabelled_pixels=0, contexts=0)
    image = cv2.imdecode(item['frames']['front_jpeg'], cv2.IMREAD_COLOR)
    if image is None:
        return dict(relabelled_pixels=0, contexts=0)
    item['native_front_image'] = image
    component = small_person_component(masks[0], item['prepared'][0][1], image.shape)
    if component is None:
        return dict(relabelled_pixels=0, contexts=0)
    rois = context_rois(component)
    if len(rois) != 2:
        return dict(relabelled_pixels=0, contexts=0)
    exclusion = exclusion_for('front', image.shape[:2])
    prepared = []
    for x, y, w, h in rois:
        crop = image[y:y+h, x:x+w].copy()
        if exclusion is not None:
            crop[exclusion[y:y+h, x:x+w] > 0] = 0
        canvas, place = letterbox(crop, width, height, cv2.INTER_LINEAR, 0)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32)
        prepared.append((((rgb-MEAN)/STD).transpose(2, 0, 1), place))
    batch = np.stack([p[0] for p in prepared])
    if refinement_gpu is not None:
        refined = refinement_gpu.run(batch)
    else:
        prediction = session.run([output_name], {'input': batch})[0]
        refined = prediction_masks(prediction, ('front', 'front'), output_name, threshold, excluded)
    contexts = [dict(mask=mask, placement=prepared[i][1], roi=rois[i])
                for i, mask in enumerate(refined)]
    masks[0], evidence = merge_contexts(masks[0], component, contexts)
    if evidence['relabelled_pixels']:
        protected = np.zeros_like(masks[0], dtype=bool)
        protected[component['yy'], component['xx']] = True
        item['confirmed_post_pixels'] = protected
    return dict(evidence, contexts=2)


def recover_front_people(item, masks):
    """Observe distant people even when the wide base view missed their class."""
    image = item.get('native_front_image')
    if image is None:
        image = cv2.imdecode(item['frames']['front_jpeg'], cv2.IMREAD_COLOR)
    if image is None:
        return dict(added_person_pixels=0, contexts=0)
    rois = forward_person_rois(image.shape)
    exclusion = exclusion_for('front', image.shape[:2])
    prepared = []
    for x, y, w, h in rois:
        crop = image[y:y+h, x:x+w].copy()
        if exclusion is not None:
            crop[exclusion[y:y+h, x:x+w] > 0] = 0
        canvas, place = letterbox(crop, width, height, cv2.INTER_LINEAR, 0)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32)
        prepared.append((((rgb-MEAN)/STD).transpose(2, 0, 1), place))
    batch = np.stack([p[0] for p in prepared])
    if refinement_gpu is not None:
        predictions = refinement_gpu.run(batch)
    else:
        prediction = session.run([output_name], {'input': batch})[0]
        predictions = prediction_masks(prediction, ('front', 'front'), output_name, threshold, excluded)
    contexts = [dict(mask=mask, placement=prepared[i][1], roi=rois[i])
                for i, mask in enumerate(predictions)]
    masks[0], evidence = recover_person_contexts(masks[0], item['prepared'][0][1], image.shape, contexts,
                                                protected=item.get('confirmed_post_pixels'))
    return evidence


class Slot:
    """Latest-only hand-off between pipeline stages (old items are dropped, never queued)."""
    def __init__(self):
        self.cond, self.item = threading.Condition(), None

    def put(self, item):
        with self.cond:
            self.item = item
            self.cond.notify()

    def take(self, timeout=0.5):
        with self.cond:
            if self.item is None:
                self.cond.wait(timeout)
            item, self.item = self.item, None
            return item


prepared_slot, overlay_slot = Slot(), Slot()


def prep_loop():
    """Stage 1 (CPU): read the newest frame set, decode, letterbox, normalise."""
    pin_thread(os.environ.get("SEMANTIC_BACKGROUND_CPUS", ""), "semantic preparation")
    last_seq = None
    next_frame = 0.0
    while True:
        remaining = next_frame - time.monotonic()
        if remaining > 0:
            time.sleep(min(remaining, 0.02))
            continue
        try:
            fresh = time.time() - frames_path.stat().st_mtime < FRESH_S
            frames = dict(np.load(frames_path)) if fresh else None
        except (FileNotFoundError, ValueError, OSError, EOFError):
            frames = None
        if frames is None or int(frames["seq"]) == last_seq:
            time.sleep(0.004)
            continue
        last_seq = int(frames["seq"])
        next_frame = time.monotonic() + 1.0 / args.max_fps
        # Front 1280x720 -> decode at 1/2 (640x360): the model input is 640x384, so no detail is lost.
        prep_started = time.perf_counter()
        images = [cv2.imdecode(frames[f"{v}_jpeg"], cv2.IMREAD_REDUCED_COLOR_2 if v == "front" else cv2.IMREAD_COLOR) for v in VIEWS]
        if any(img is None for img in images):
            continue
        prepared = [fast_preprocess(v, img) for v, img in zip(VIEWS, images)]
        prepared_slot.put(dict(seq=last_seq, frames=frames, images=images, prepared=prepared, prep_ms=(time.perf_counter()-prep_started)*1000))


def overlay_loop():
    """Stage 3 (CPU, throttled): dashboard overlays + status."""
    pin_thread(os.environ.get("SEMANTIC_DISPLAY_CPUS", os.environ.get("SEMANTIC_BACKGROUND_CPUS", "")), "semantic display")
    while True:
        time.sleep(1.0 / args.overlay_hz)
        item = overlay_slot.take(0)
        if item is None:
            continue
        for view, img, mask, (_, place) in zip(VIEWS, item["images"], item["masks"], item["prepared"]):
            crop = mask[place["top"]:place["top"] + place["height"], place["left"]:place["left"] + place["width"]]
            crop = cv2.resize(crop, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
            ex = exclusion_for(view, img.shape[:2])
            if ex is not None:
                crop[ex > 0] = 255
            vis = build_visualization(view, img, crop, colors, class_names, item["inference_ms"])
            ok, enc = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                write_atomic(args.live / f"{view}.jpg", enc.tobytes())
            status["views"][view] = {"time": time.time(), "inference_ms": round(item["inference_ms"], 1),
                                     "pixel_counts": {class_names[i]: int((crop == i).sum())
                                                      for i in range(len(class_names)) if i not in excluded}}
        status.update(time=time.time(), fps=item["fps"], inference_ms=round(item["inference_ms"], 1), prep_ms=round(item["prep_ms"], 2), gpu_postprocess=gpu_masks is not None)
        write_atomic(args.live / "status.json", json.dumps(status, ensure_ascii=False).encode())


threading.Thread(target=prep_loop, name="prep", daemon=True).start()
threading.Thread(target=overlay_loop, name="overlay", daemon=True).start()
done, rate_t0, fps = 0, time.monotonic(), 0.0
while True:
    # Stage 2 (GPU): inference + masks for the fusion node, every frame set.
    item = prepared_slot.take()
    if item is None:
        continue
    started = time.perf_counter()
    batch = np.stack([p[0] for p in item["prepared"]])
    if gpu_masks is not None:
        masks = gpu_masks.run(batch)
    else:
        prediction = session.run([output_name], {"input": batch})[0]
        masks = prediction_masks(prediction, VIEWS, output_name, threshold, excluded)
    refinement = dict(relabelled_pixels=0, contexts=0)
    try:
        refinement = refine_front_person(item, masks)
    except Exception as error:
        # A failed extra observation preserves every original person pixel.
        refinement['error'] = str(error)[:200]
    try:
        person_recovery = recover_front_people(item, masks)
    except Exception as error:
        person_recovery = dict(added_person_pixels=0, contexts=0, error=str(error)[:200])
    inference_ms = (time.perf_counter() - started) * 1000.0
    out = {"seq": np.int64(item["seq"]), "wall": np.float64(time.time())}
    for view, img, mask, (_, place) in zip(VIEWS, item["images"], masks, item["prepared"]):
        ex = exclusion_for(view, img.shape[:2])
        if ex is not None:
            # ego body/hood: no class there, for the fusion and for every display
            small = ex_small.get((view, img.shape[:2]))
            if small is None:
                small = ex_small[(view, img.shape[:2])] = letterbox(ex, width, height, cv2.INTER_NEAREST, 0)[0] > 0
            mask = mask.copy()
            mask[small] = 254             # 254 = hidden by the ego body (camera blind), 255 = background
        out[f"{view}_mask"] = mask
        out[f"{view}_stamp_ns"] = item["frames"][f"{view}_stamp_ns"]
        out[f"{view}_place"] = np.array([place["left"], place["top"], place["width"], place["height"],
                                         img.shape[1], img.shape[0]], np.int32)
    tmp = args.shm / ".masks.npz.tmp"
    with open(tmp, "wb") as handle:
        np.savez(handle, **out)
    os.replace(tmp, args.shm / "masks.npz")
    write_atomic(args.shm / "performance.json", json.dumps({
        "seq": int(item["seq"]), "wall": time.time(), "prep_ms": item["prep_ms"],
        "inference_post_ms": inference_ms,
        "pipeline_latency_ms": (time.time()-float(item["frames"]["wall"]))*1000,
        "gpu_postprocess": gpu_masks is not None,
        "small_person_refinement": refinement,
        "distant_person_recovery": person_recovery,
        "cpu_threads": {stage: {"tid": tid, "cpus": sorted(os.sched_getaffinity(tid))}
                        for stage, tid in cpu_thread_registry.items()},
    }).encode())
    done += 1
    now = time.monotonic()
    if now - rate_t0 >= 1.0:
        fps, done, rate_t0 = round(done / (now - rate_t0), 1), 0, now
    overlay_slot.put(dict(item, masks=masks, inference_ms=inference_ms, fps=fps))
