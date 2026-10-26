"""Use the existing Student path head with tunnel perception constraints.

The network and checkpoint are unchanged. LiDAR supplies a feasible reference
through the existing route input, and independently checks the returned path.
An invalid network output is reported explicitly instead of being presented as
a model-generated fallback trajectory.
"""
import math
import time

import numpy as np


def route_features(reference, count):
    reference = np.asarray(reference, dtype=float)[:, :2]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
    keep = np.r_[True, np.diff(arc) > 1e-4]
    arc, reference = arc[keep], reference[keep]
    if len(arc) < 3 or arc[-1] < (count-1)*.5:
        raise ValueError("tunnel reference too short")
    sample = np.arange(count)*.5
    xy = np.column_stack([np.interp(sample, arc, reference[:, axis]) for axis in (0, 1)])
    distance = np.r_[np.linalg.norm(xy[0]), np.linalg.norm(xy[0])+
                     np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    return np.column_stack((xy, distance, np.ones(count))).astype(np.float32)


def checked_model_path(raw, reference, ego, constraints, clear, required_m,
                       maintain_mps):
    """Return a physically reachable path only after both perception checks."""
    from gui import (RosPathPublisher, smooth_control_route, path_curvatures,
                     MAX_CONTROL_CURVATURE, BYPASS_ALAT_MPS2)
    info = dict(accepted=False, reason="model trajectory invalid")
    try:
        required_m, maintain_mps = float(required_m), float(maintain_mps)
        wheel_angle = ego.get("wheel_angle")
        if (not math.isfinite(required_m) or required_m <= 0. or
                not math.isfinite(maintain_mps) or maintain_mps < 0. or
                wheel_angle is not None and not math.isfinite(float(wheel_angle))):
            return None, dict(info, reason="model trajectory feasibility input invalid")
    except (TypeError, ValueError):
        return None, dict(info, reason="model trajectory feasibility input invalid")
    raw = np.asarray(raw, dtype=float)
    if raw.ndim != 2 or raw.shape[0] < 3 or raw.shape[1] < 2 or not np.isfinite(raw).all():
        return None, info
    raw = raw[:, :2]
    reference = np.asarray(reference, dtype=float)[:, :2]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
    nearest = np.argmin(np.sum((raw[:, None]-reference[None])**2, axis=2), axis=1)
    progress = arc[nearest]
    if (raw[0, 0] < -.5 or np.max(np.diff(progress)*-1., initial=0.) > .5 or
            progress[-1]-progress[0] < 8.):
        info["reason"] = "model trajectory has insufficient ordered progress"
        return None, info
    if np.linalg.norm(raw[0]) > 4. or np.max(np.linalg.norm(np.diff(raw, axis=0), axis=1)) > 4.:
        info["reason"] = "model trajectory discontinuity"
        return None, info
    raw = np.vstack(([0., 0.], raw))
    keep = np.r_[True, np.linalg.norm(np.diff(raw, axis=0), axis=1) > .05]
    path = smooth_control_route(raw[keep], max_shift_m=.15)
    path = RosPathPublisher.extend_with_route(path, reference)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    if arc[-1] < required_m:
        info["reason"] = "model trajectory horizon too short"
        return None, info
    sample = np.arange(0., min(60., arc[-1])+.001, .5)
    path = np.column_stack([np.interp(sample, arc, path[:, axis]) for axis in (0, 1)])
    curvature = float(np.max(path_curvatures(path), initial=0.))
    a, b = np.diff(path, axis=0)[:-1], np.diff(path, axis=0)[1:]
    denominator = np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)*np.linalg.norm(a+b, axis=1)
    signed = np.divide(2.*(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0]), denominator,
                       out=np.zeros(len(a)), where=denominator > 1e-8)
    steering = np.arctan(3.9*signed)
    if ego.get("wheel_angle") is not None:
        steering = np.r_[math.radians(float(ego["wheel_angle"])), steering]
    gradient = float(np.max(np.abs(np.diff(steering))/.5, initial=0.))
    cap = min(math.sqrt(BYPASS_ALAT_MPS2/max(curvature, .001)),
              math.radians(30.)*.8/max(gradient, 1e-6))
    info.update(curvature_1pm=curvature, speed_cap_mps=cap,
                steering_gradient_radpm=gradient)
    if curvature > MAX_CONTROL_CURVATURE or cap+1e-6 < maintain_mps:
        info["reason"] = "model trajectory cannot maintain checked speed"
        return None, info
    topic_clear = float(constraints.clear_distance(path, min(60., arc[-1])))
    if not math.isfinite(topic_clear):
        return None, dict(info, reason="model trajectory no-go clearance invalid")
    info["no_go_clear_m"] = float(topic_clear)
    if topic_clear < required_m:
        info["reason"] = "model trajectory intersects tunnel no-go area"
        return None, info
    clearance = float(clear(path, min(60., arc[-1])))
    if not math.isfinite(clearance):
        return None, dict(info, reason="model trajectory current LiDAR clearance invalid")
    info["clear_distance_m"] = clearance
    if clearance < required_m:
        info["reason"] = "model trajectory lacks current LiDAR clearance"
        return None, info
    info.update(accepted=True, reason="model trajectory passed tunnel no-go check")
    return path, info


def propose(runtime, ego, reference, constraints, clear, required_m, maintain_mps,
            deadline=None):
    """One measured-frame proposal per update; no forced speed floor or retry loop."""
    info = dict(enabled=True, accepted=False, source="lidar_fallback")
    if not getattr(runtime, "model", None):
        return None, dict(info, reason="model unavailable")
    if not constraints.valid:
        return None, dict(info, reason=constraints.reason)
    if deadline is not None and time.perf_counter() >= deadline:
        return None, dict(info, reason="model proposal budget exhausted")
    if not runtime.model_camera_fresh():
        return None, dict(info, reason="model camera frames stale")
    import torch
    rows = runtime.receiver.snapshot(int(runtime.config["frames"]))
    if rows is None:
        return None, dict(info, reason="model camera history unavailable")
    started = time.perf_counter()
    try:
        images, runtime.last_images = runtime.image_tensor(rows)
        route = torch.from_numpy(route_features(reference, int(runtime.config["route_points"])))[None]
        with torch.inference_mode(), torch.autocast(device_type=runtime.device.type,
                                                   enabled=runtime.device.type == "cuda"):
            output = runtime.model(images.to(runtime.device),
                runtime.ego_tensor(ego).to(runtime.device), route.to(runtime.device))
        raw = output["path"][0].float().cpu().numpy()
        probabilities = torch.softmax(output["action_logits"].float(), dim=-1)[0].cpu().numpy()
        if (probabilities.ndim != 1 or len(probabilities) != len(runtime.actions) or
                not np.isfinite(probabilities).all()):
            return None, dict(info, reason="model action probabilities invalid",
                              latency_ms=(time.perf_counter()-started)*1000.)
        info.update(action=runtime.actions[int(np.argmax(probabilities))],
                    probability=float(np.max(probabilities)))
        # Display metadata follows the same finite XY contract as a trajectory.
        # Malformed outputs remain explicit fallbacks without a drawable path.
        info["_probabilities"] = probabilities
        if (raw.ndim == 2 and len(raw) >= 1 and raw.shape[1] >= 2 and
                np.isfinite(raw).all()):
            info["_raw_path"] = raw[:, :2]
        if deadline is not None and time.perf_counter() >= deadline:
            return None, dict(info, reason="model proposal budget exhausted",
                              latency_ms=(time.perf_counter()-started)*1000.)
        candidate, checked = checked_model_path(raw, reference, ego, constraints,
                                                clear, required_m, maintain_mps)
        info.update(checked,
                    latency_ms=(time.perf_counter()-started)*1000.)
        if deadline is not None and time.perf_counter() >= deadline:
            return None, dict(info, accepted=False, reason="model proposal budget exhausted")
        if candidate is not None:
            info["source"] = "model"
        return candidate, info
    except (ValueError, RuntimeError, TypeError, OSError, IndexError, KeyError) as exc:
        return None, dict(info, reason="model proposal failed: "+str(exc),
                          latency_ms=(time.perf_counter()-started)*1000.)
