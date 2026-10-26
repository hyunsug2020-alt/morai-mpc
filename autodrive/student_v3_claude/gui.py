#!/usr/bin/env python3
"""Three-camera MORAI Student shadow GUI. It never publishes vehicle controls."""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import queue
import select
import socket
import struct
import sys
import threading
import time
from collections import OrderedDict, deque
from pathlib import Path

import cv2
import numpy as np
from json_safe import json_default
import torch
from PIL import Image
from scipy.spatial import cKDTree
from lane_corridor import LaneCorridor

from model import load_checkpoint
try:   # v6 judgement model (child process; only with STUDENT_V6_CHECKPOINT). Never stops the GUI.
    from v6 import stop_v6_shadow, notify_v6_idle, v6_record_fields
except Exception as _v6_error:
    print(f"[v6] unavailable: {_v6_error!r}", flush=True)
    stop_v6_shadow = notify_v6_idle = None
    def v6_record_fields(runtime):
        return {}
from traffic_planner import (GapPlanner, JunctionTrackMemory, fit_circle_motion, vehicle_motion_heading,
                             lidar_vehicle_velocity)
from ring_entry import RingMotionTracker, following_speed_cap
from lidar_corridor import LidarCorridor, PedestrianMotionHistory, margin_for_speed, without_moving_objects
from motion_filter import measured_velocity
from motion_limits import LAUNCH_ACCEL_MPS2
from highway_planner import HighwayPlanner, NearFieldMemory
from highway_runtime import HighwayRuntime
from highway_observations import HighwayObservationFilter, source_lag_s
from planning.highway.object_policy import (highway_objects, highway_static_objects,
                                           highway_measured_objects)
from lattice_planner import plan as lattice_plan
from ped_guard import PedTracker, plan as ped_guard_plan, EGO_FRONT_M as PED_FRONT_M, EGO_REAR_M as PED_REAR_M
from ego_body import (EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M, ROAD_LEGACY_SHIFT_M, REAR_AXLE_SHIFT_M, body_centres,
                      body_gap as ego_body_gap, reverse_sweep_hits, EGO_FRONT_M)
from ped_guard import contact_guard, sweep_ok as contact_sweep_ok, CORRIDOR_HALF_M, PersonGhostFilter
from ped_guard import PED_R as PED_RADIUS_M, StandingBypassWait
from perception.stationary_semantics import (StaticPedestrianEvidence, annotate_static_candidates,
                                            as_obstacle as static_person_obstacle, metadata as object_metadata)
from perception.static_geometry_contract import canonical_static_name, decode_static_geometry
from stop_line import (COMPETITION_STOP_NODE_IDS, INTERSECTION_ACCEL_MPS2,
                       INTERSECTION_ENTRY_MPS, INTERSECTION_ENTRY_CAP_FRONT_GAP_M,
                       intersection_entry_cap_mps, competition_stop_controller,
                       entry_is_clear, intersection_entry_decision, EMPTY_ENTRY_REASON,
                       empty_intersection_observation, intersection_front_station)
from intersection_perception import (build_intersection_regions, active_intersection,
                                     load_intersection_regions)
from roundabout_empty import (EntryReference, certified_path_geometry, certified_pose_geometry,
    raw_entry_clearance, braking_reserve, entry_scan_points,
    fresh_entry_pose, entry_proof_fresh,
    ROLL_MPS, MAX_POSE_AGE_S, MAX_ENTRY_SOURCE_AGE_S, ROLL_CHECK_FRONT_GAP_M,
    CLEARANCE_M as EMPTY_ENTRY_CLEARANCE_M)
from tunnel_runtime import TunnelRuntime, pose_at_stamp
from planner_transition import PlannerTransition
from driving_plan import DrivingPlan, static_reverse_release_ready
from junction_entry_profile import bind_entry_profile, entry_profile_ceiling
from contextlib import nullcontext
from live_path_safety import LivePathSafety, serialized_command, bind_plan_id
from planning.tunnel.approach_memory import update_approach_points
from sim_time import SimClock, runtime_rtf, to_sim_objects
from lap_finish import lap_marker_path, lap_tracker_from_env, write_lap_marker, signal_waiting
from signal_control.runtime import SignalController, signal_enabled_from_env
from signal_control.camera import CAMERA_VIEW as SIGNAL_CAMERA_VIEW
from lead_follow import (HeadingBias, LeadTracker, body_aligned_path, lead_cap, lidar_lead,
                         needs_emergency_stop, object_leads, path_coordinates, path_heading_rad, STANDSTILL_GAP_M,
                         STOPPED_MPS, MOVING_MPS)

sys.path.insert(1, str(Path(__file__).resolve().parents[1]))
from competition_profiles import profile as competition_profile
from lidar_vehicle_detection import VehicleReceiver
SENSOR_PROFILE = competition_profile(os.environ.get("STUDENT_NETWORK_PROFILE", "single_pc"))


ROS_PYTHON_PATHS = (
    "/opt/ros/noetic/lib/python3/dist-packages",
    str(Path(__file__).resolve().parents[2] / "localization/devel/lib/python3/dist-packages"),
    "/usr/lib/python3/dist-packages",
)
for ros_path in reversed(ROS_PYTHON_PATHS):
    if ros_path in sys.path:
        sys.path.remove(ros_path)
    sys.path.insert(0, ros_path)


VIEWS = tuple((view, spec["udp_port"]) for view, spec in SENSOR_PROFILE["cameras"].items())
COLORS = {"route": (30, 205, 255), "student": (255, 230, 60), "raw": (210, 110, 240)}
IMAGE_MEAN = np.asarray((0.485, 0.456, 0.406), dtype=np.float32)
IMAGE_STD = np.asarray((0.229, 0.224, 0.225), dtype=np.float32)
# Existing normal-road bypass curvature bound; highway validation uses 3.90 m explicitly.
MAX_CONTROL_CURVATURE = math.tan(math.radians(35.0)) / 3.90
HIGHWAY_SIDE_MARGIN_M = .15
# Highway: obstacles close beside the body (auto run 188: exit drum taper 0.25 m
# beside the body at 85 km/h, a full brake 1.5 s before it still knocked drums over).
# The path is shifted inside the lane toward the roomier side, and the speed is
# capped by the remaining side gap early enough to brake at 3 m/s^2.
HIGHWAY_NUDGE_MAX_M = .6
HIGHWAY_NUDGE_WANT_M = .6
HIGHWAY_SIDE_GAP_KMH = ((.2, 30.), (.45, 50.), (.75, 70.))
cv2.setNumThreads(1)



import sys as _performance_sys
from pathlib import Path as _PerformancePath
_performance_sys.path.insert(0, str(_PerformancePath(__file__).resolve().parent.parent if _PerformancePath(__file__).resolve().parent.name == "student_v3" else _PerformancePath(__file__).resolve().parent))
from cpu_placement import pin_io_thread, pin_compute_thread

class AsyncJsonCache:
    """Refresh optional disk-backed inputs without blocking ROS/control readers."""
    def __init__(self, path, interval=.1):
        self.path, self.interval = Path(path), interval
        self.value = None
        self.stopping = threading.Event()
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def _run(self):
        pin_io_thread("status JSON input")
        while not self.stopping.is_set():
            try:
                self.value = json.loads(self.path.read_text())
            except (OSError, ValueError):
                pass
            self.stopping.wait(self.interval)

    def close(self):
        self.stopping.set()
        self.worker.join(timeout=.2)


def speed_label(record):
    """One training label per decision: why the car stops / slows (2026-10-04, v6 data).

    action STOP | SLOW | CRUISE from the published speed cap vs the zone speed;
    cause pedestrian | vehicle | static | stop_line | curve | junction | highway_speed | other | none, and
    dynamic=True when a moving road user (pedestrian or vehicle) caused it."""
    zone = record.get("target_kmh") or 0.
    cap = record.get("published_cap_kmh")
    cap = zone if cap is None else cap
    if record.get("emergency_stop") or cap < 1.:
        action = "STOP"
    elif cap < zone - 3.:
        action = "SLOW"
    else:
        action = "CRUISE"
    hold = record.get("yield") if isinstance(record.get("yield"), dict) else None
    hazard = record.get("hazard") if isinstance(record.get("hazard"), dict) else None
    text = " ".join(str(v) for v in (record.get("stop_reason"), record.get("reason"),
                                     (hold or {}).get("name"), (hazard or {}).get("name")) if v).lower()
    caution = record.get("dynamic_caution_kmh")
    if action == "CRUISE":
        cause = "none"
    elif "stop line" in text or "signal" in text:
        cause = "stop_line"
    elif "pedestrian" in text or "person" in text or (caution is not None and caution < zone - 3.):
        cause = "pedestrian"
    elif any(k in text for k in ("car", "vehicle", "npc", "crossing", "collision", "lane-change", "intersection", "lead")):
        cause = "vehicle"
    elif any(k in text for k in ("obstacle", "static", "lidar", "bypass", "tunnel")):
        cause = "static"
    elif "curv" in text:
        cause = "curve"
    elif str(record.get("mode", "")).startswith("HIGHWAY"):
        cause = "highway_speed"       # highway exit / sensor-late hold / speed ramp
    elif isinstance(record.get("traffic_plan"), dict) and record["traffic_plan"].get("reason"):
        cause = "junction"            # junction turn / merge speed plan without a conflicting car
    else:
        cause = "other"
    source = hold if hold and hold.get("along_m") else hazard
    obj = None
    if source:
        obj = {"name": source.get("name"), "along_m": source.get("along_m"), "lat_m": source.get("lat_m")}
    return {"action": action, "cause": cause, "dynamic": cause in ("pedestrian", "vehicle"),
            "cap_kmh": round(float(cap), 1), "zone_kmh": round(float(zone), 1), "object": obj}


class AsyncDecisionLog:
    """Keep diagnostic disk stalls outside the control loop with bounded memory."""
    def __init__(self, path):
        self.path = Path(path)
        self.pending = queue.Queue(maxsize=512)
        self.stopping = threading.Event()
        self.dropped = 0
        self.error = None
        self.worker = threading.Thread(target=self._run, name="student-decision-log", daemon=True)
        self.worker.start()

    def write(self, line):
        try:
            self.pending.put_nowait(line)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        pin_io_thread("decision log output")
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", buffering=1) as stream:
                while not self.stopping.is_set() or not self.pending.empty():
                    try:
                        line = self.pending.get(timeout=.1)
                    except queue.Empty:
                        continue
                    stream.write(line)
        except Exception as exc:
            self.error = repr(exc)

    def close(self):
        self.stopping.set()
        self.worker.join(timeout=2.)


def cosine_gallery_matches(scene, gallery_matrix, gallery, count=3):
    """Return nearest supervised scene descriptions; similarity is not probability."""
    scene = np.asarray(scene, dtype=np.float32)
    norm = float(np.linalg.norm(scene))
    if norm < 1e-8 or not len(gallery):
        return []
    similarities = gallery_matrix @ (scene / norm)
    order = np.argsort(similarities)[::-1][:count]
    return [{"similarity": float(similarities[index]), **gallery[index]} for index in order]


def trajectory_diagnostics(path, route, min_lateral_m=0.70):
    """Describe the Student path itself relative to the local HD-map route."""
    path = np.asarray(path, dtype=np.float32)
    route = np.asarray(route, dtype=np.float32)
    if len(path) == 0 or len(route) < 2:
        return {"plan": "NO TRAJECTORY", "max_offset_m": 0.0, "end_left_m": 0.0}
    route = np.vstack((np.zeros((1, route.shape[1]), np.float32), route))
    order = np.argsort(route[:, 0])
    forward, unique = np.unique(route[order, 0], return_index=True)
    route_left = route[order, 1][unique]
    expected_left = np.interp(path[:, 0], forward, route_left)
    offsets = path[:, 1] - expected_left
    usable = offsets[min(4, len(offsets) - 1):]
    peak = float(usable[np.argmax(np.abs(usable))])
    if abs(peak) < min_lateral_m:
        plan = "FOLLOW ROUTE"
    else:
        plan = "NUDGE LEFT" if peak > 0 else "NUDGE RIGHT"
    return {
        "plan": plan,
        "max_offset_m": peak,
        "end_left_m": float(offsets[-1]),
        "forward_m": float(np.max(path[:, 0])),
    }


def path_curvatures(points):
    """Discrete planar curvature in 1/m for path feasibility checks."""
    points = np.asarray(points, dtype=np.float64)[:, :2]
    if len(points) < 3:
        return np.zeros(0, dtype=np.float64)
    first_edge = points[1:-1] - points[:-2]
    second_edge = points[2:] - points[1:-1]
    chord = points[2:] - points[:-2]
    denominator = (np.linalg.norm(first_edge, axis=1)
                   * np.linalg.norm(second_edge, axis=1)
                   * np.linalg.norm(chord, axis=1))
    cross = np.abs(first_edge[:, 0] * chord[:, 1]
                   - first_edge[:, 1] * chord[:, 0])
    return np.divide(2.0 * cross, denominator,
                     out=np.zeros_like(denominator), where=~(denominator < 1e-8))


def smooth_control_route(points, max_shift_m=0.25):
    """Round map-link sampling corners within a bounded distance of the route."""
    points = np.asarray(points, dtype=float)[:, :2]
    if len(points) < 5:
        return points.copy()
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    unique, keep = np.unique(arc, return_index=True)
    if len(unique) < 5:
        return points.copy()
    sample = np.arange(0., unique[-1] + .01, .5)
    if len(sample) < 5:
        return points.copy()
    uniform = np.column_stack([np.interp(sample, unique, points[keep, i]) for i in (0, 1)])
    radius = 9
    kernel = np.exp(-.5 * (np.arange(-radius, radius + 1) / 3.) ** 2)
    kernel /= kernel.sum()
    before = uniform[0] - np.arange(radius, 0, -1)[:, None] * (uniform[1] - uniform[0])
    after = uniform[-1] + np.arange(1, radius + 1)[:, None] * (uniform[-1] - uniform[-2])
    extended = np.vstack((before, uniform, after))
    filtered = np.column_stack([np.convolve(extended[:, i], kernel, mode="valid") for i in (0, 1)])
    candidate = np.column_stack([np.interp(arc, sample, filtered[:, i]) for i in (0, 1)])
    delta = candidate - points
    delta *= np.minimum(1., max_shift_m / np.maximum(1e-9, np.linalg.norm(delta, axis=1)))[:, None]
    return points + delta


def build_forced_bypass(dense_route, direction, offset_m=2.2, hold_m=6.0,
                        path_length_m=60.0, max_curvature_1pm=0.16, transitions=(10.0, 12.0, 14.0, 16.0, 18.0, 20.0)):
    """Build a vehicle-feasible S-curve using only Student's avoidance direction."""
    route = np.asarray(dense_route, dtype=np.float32)
    route_xy = route[:, :2]
    route_s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(route_xy, axis=0), axis=1))))
    route_s, unique = np.unique(route_s, return_index=True)
    route_xy = route_xy[unique]
    usable = route_s <= min(float(path_length_m), float(route_s[-1]))
    route_s, map_base = route_s[usable], route_xy[usable]
    if len(route_s) < 3 or route_s[-1] < 25.0:
        return None, float("inf"), 0.0

    tangent = np.gradient(map_base, route_s, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-6)
    normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))

    def quintic(value):
        value = np.clip(value, 0.0, 1.0)
        return value ** 3 * (10.0 - 15.0 * value + 6.0 * value ** 2)

    best = None
    for transition_m in transitions:
        return_start = transition_m + hold_m
        profile = float(direction) * float(offset_m) * (1.0 - quintic(
            (route_s - return_start) / transition_m))
        # Offset the map lane first, then join to it from the actual pose and
        # heading. Offsetting the normals of an already joined path makes a
        # second S bend when ego is already in the adjacent lane.
        reference = map_base + normals * profile[:, None]
        initial_tangent = (reference[1] - reference[0]) / (route_s[1] - route_s[0])
        correction = reference[0] + route_s[:, None] * (initial_tangent - np.array([1., 0.]))
        candidate = np.zeros((len(reference), 3), dtype=np.float32)
        candidate[:, :2] = reference - correction * (1.0 - quintic(route_s / transition_m))[:, None]
        curvature = path_curvatures(candidate)
        peak_curvature = float(np.max(curvature)) if len(curvature) else float("inf")
        if best is None or peak_curvature < best[1]:
            best = (candidate, peak_curvature, transition_m)
        if peak_curvature <= max_curvature_1pm:
            return candidate, peak_curvature, transition_m
    return best


def postprocess_trajectory(raw_path, dense_route, max_lateral_m=4.0,
                           force_student_avoidance=False, student_enabled=True,
                           forced_min_offset_m=2.2, forced_direction=None):
    """Anchor the trajectory to HD-map geometry and retain only a rejoining lateral residual.

    This is deliberately conservative: a persistent one-sided error (for example a straight
    Student path through a map curve) is rejected instead of being turned into fake avoidance.
    """
    raw_path = np.asarray(raw_path, dtype=np.float32)
    route = np.asarray(dense_route, dtype=np.float32)
    empty_info = {"mode": "ROUTE FALLBACK", "reason": "invalid input", "raw_peak_m": 0.0,
                  "raw_end_m": 0.0, "applied_peak_m": 0.0}
    if len(raw_path) < 3 or len(route) < 3:
        return raw_path.copy(), empty_info

    # The supervised tensor contains ego-local poses, including points at/behind the
    # current pose. Keep the forward, ordered branch. Summing these poses as deltas or
    # accumulating their zig-zag distance creates a false lateral residual on curves.
    all_xy = raw_path[:, :2]
    anchor = int(np.argmin(np.linalg.norm(all_xy, axis=1)))
    candidate = raw_path[anchor:]
    progress = candidate[:, 0] - candidate[0, 0]
    keep = []
    furthest = -0.05
    for index, value in enumerate(progress):
        if float(value) >= furthest + 0.05:
            keep.append(index)
            furthest = float(value)
    if len(keep) < 3 or furthest < 1.0:
        empty_info["reason"] = "no ordered forward Student path"
        return raw_path.copy(), empty_info
    working_path = candidate[keep].copy()
    query_s = progress[keep].astype(np.float32)

    route_xy = route[:, :2]
    route_s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(route_xy, axis=0), axis=1))))
    route_s, unique = np.unique(route_s, return_index=True)
    route_xy = route_xy[unique]
    raw_xy = working_path[:, :2]
    query_s = np.clip(query_s, route_s[0], route_s[-1])
    map_base = np.column_stack((np.interp(query_s, route_s, route_xy[:, 0]),
                                np.interp(query_s, route_s, route_xy[:, 1]))).astype(np.float32)
    # A nearest HD-map point can be behind or beside ego. Connecting to it in one short segment
    # creates a false 60~90 degree heading and forces MPC into RECOV. Start on ego heading and
    # merge smoothly onto the map over 12 m with zero slope at both ends.
    join_u = np.clip(query_s / 12.0, 0.0, 1.0).astype(np.float32)
    join_weight = join_u * join_u * (3.0 - 2.0 * join_u)
    ego_heading_base = np.column_stack((query_s, np.zeros_like(query_s))).astype(np.float32)
    base = ego_heading_base * (1.0 - join_weight[:, None]) + map_base * join_weight[:, None]

    tangent = np.gradient(base, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-6)
    normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    route_turn_deg = abs(math.degrees(math.atan2(
        tangent[-1, 1] * tangent[0, 0] - tangent[-1, 0] * tangent[0, 1],
        float(np.dot(tangent[-1], tangent[0])))))
    lateral = np.sum((raw_xy - base) * normals, axis=1)
    kernel = np.ones(5, dtype=np.float32) / 5.0
    lateral = np.convolve(np.pad(lateral, (2, 2), mode="edge"), kernel, mode="valid")
    lateral -= lateral[0]

    usable_start = min(4, len(lateral) - 1)
    peak_index = usable_start + int(np.argmax(np.abs(lateral[usable_start:])))
    peak = float(lateral[peak_index])
    end = float(np.mean(lateral[-min(5, len(lateral)) :]))
    initial = float(np.max(np.abs(lateral[: min(6, len(lateral))])))
    peak_abs = abs(peak)
    detection_threshold = 0.25 if force_student_avoidance else 0.70
    has_residual = student_enabled and peak_abs >= detection_threshold
    returns_to_route = abs(end) <= max(0.70, 0.45 * peak_abs)
    peak_before_end = peak_index <= int(0.85 * (len(lateral) - 1))
    begins_on_route = initial <= 1.0
    rejoining_residual = has_residual and returns_to_route and peak_before_end and begins_on_route
    # A demonstrated lane offset may legitimately continue beyond the 6.4 s horizon. Preserve it only
    # on a nearly straight map segment and within one lane; this excludes straight-through-map-curve errors.
    persistent_residual = (has_residual and not returns_to_route and begins_on_route
                           and peak_abs >= 1.5 and peak_abs <= max_lateral_m
                           and route_turn_deg <= 8.0)
    direction_latched = forced_direction in (-1, 1, -1.0, 1.0)
    forced_residual = (force_student_avoidance and (has_residual or direction_latched)
                       and np.all(np.isfinite(lateral)))
    accepted = forced_residual or rejoining_residual or persistent_residual

    if accepted:
        if forced_residual:
            direction = float(forced_direction) if direction_latched else (1.0 if peak >= 0.0 else -1.0)
            forced_path, forced_curvature, transition_m = build_forced_bypass(
                route, direction, offset_m=min(float(forced_min_offset_m), float(max_lateral_m)))
            if forced_path is not None and forced_curvature <= 0.16:
                return forced_path, {
                    "mode": "STUDENT FORCED",
                    "reason": "feasible Student-direction S-curve",
                    "raw_peak_m": peak,
                    "raw_end_m": end,
                    "applied_peak_m": direction * min(float(forced_min_offset_m), float(max_lateral_m)),
                    "route_turn_deg": route_turn_deg,
                    "max_curvature_1pm": forced_curvature,
                    "transition_m": transition_m,
                }
            applied = np.zeros_like(lateral)
            mode = "ROUTE BASE"
            reason = "forced path curvature rejected"
            forced_residual = False
        else:
            applied = lateral.copy()
            mode = "STUDENT RESIDUAL"
            reason = ("depart-and-rejoin shape accepted" if rejoining_residual
                      else "bounded offset on straight route accepted")
        fade_count = min(6, len(applied))
        u = np.linspace(0.0, 1.0, fade_count, dtype=np.float32)
        applied[:fade_count] *= u * u * (3.0 - 2.0 * u)
        applied = np.clip(applied, -max_lateral_m, max_lateral_m)
    else:
        applied = np.zeros_like(lateral)
        mode = "ROUTE BASE"
        if not student_enabled:
            reason = "no avoidance action intent"
        elif not has_residual:
            reason = "no material avoidance residual"
        elif not returns_to_route:
            reason = "persistent/curved or excessive offset rejected"
        elif not peak_before_end:
            reason = "late divergence rejected"
        else:
            reason = "unanchored start rejected"

    hybrid = working_path.copy()
    hybrid[:, :2] = base + normals * applied[:, None]
    return hybrid, {"mode": mode, "reason": reason, "raw_peak_m": peak,
                    "raw_end_m": end, "applied_peak_m": float(applied[np.argmax(np.abs(applied))]),
                    "route_turn_deg": route_turn_deg}


def put_wrapped(canvas, text, origin, max_width, color, scale=.40, thickness=1, max_lines=2):
    words = str(text).replace("\n", " \n ").split()
    lines, line = [], ""
    for word in words:
        if word == "\\n":
            if line: lines.append(line)
            line = ""
            continue
        candidate = f"{line} {word}".strip()
        if cv2.getTextSize(candidate, 0, scale, thickness)[0][0] <= max_width:
            line = candidate
        else:
            if line: lines.append(line)
            line = word
    if line: lines.append(line)
    x, y = origin
    for value in lines[:max_lines]:
        cv2.putText(canvas, value, (x, y), 0, scale, color, thickness, cv2.LINE_AA)
        y += 20
    return y


class FrameAssembler:
    def __init__(self):
        self.reset()

    def reset(self):
        self.ts = self.expected = None
        self.buf = bytearray()

    def feed(self, datagram):
        if len(datagram) < 21 or not datagram.startswith(b"MOR"):
            return None
        sec, nsec, index, size = struct.unpack("<IIII", datagram[3:19])
        end = 19 + size
        if nsec >= 1_000_000_000 or size <= 0 or end > len(datagram) - 2:
            self.reset(); return None
        stamp = sec * 1_000_000_000 + nsec
        payload = datagram[19:end]
        if index == 0:
            self.ts, self.expected, self.buf = stamp, 1, bytearray(payload)
        elif self.expected is None or index != self.expected or stamp != self.ts:
            self.reset(); return None
        else:
            self.buf.extend(payload); self.expected += 1
        if datagram[-2:] != b"EI":
            return None
        jpeg, stamp = bytes(self.buf), self.ts
        self.reset()
        end_jpeg = jpeg.find(b"\xff\xd9")
        if end_jpeg >= 0:
            jpeg = jpeg[:end_jpeg + 2]
        return (stamp, jpeg) if jpeg.startswith(b"\xff\xd8") else None


CAMERA_IDS = {view: spec["camera_id"] for view, spec in SENSOR_PROFILE["cameras"].items()}


def camera_input_views(recording=False):
    """Keep the driving model's three views separate from signal camera 4."""
    if recording or signal_enabled_from_env():
        return VIEWS + ((SIGNAL_CAMERA_VIEW, SENSOR_PROFILE["aux_camera"]["udp_port"]),)
    return VIEWS


class CameraReceiver(threading.Thread):
    """UDP camera receiver. When STUDENT_SESSION_DIR is set (DAgger recording) every frame is also written in the
    collect/collect_rgb.py raw-session format: raw/<view>/rgb/NNNNNNNN_<ts_ns>.jpg + manifest.jsonl + session.json."""
    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        import os
        self.session_dir = Path(os.environ["STUDENT_SESSION_DIR"]) if os.environ.get("STUDENT_SESSION_DIR") else None
        self.capture_views = camera_input_views(recording=self.session_dir is not None)
        self.frames = {name: deque(maxlen=120) for name, _ in self.capture_views}
        self.counts = {name: deque(maxlen=300) for name, _ in self.capture_views}
        self.manifest = None
        self.saved = {name: 0 for name, _ in self.capture_views}
        if self.session_dir:
            for name, _ in self.capture_views:
                (self.session_dir / "raw" / name / "rgb").mkdir(parents=True, exist_ok=True)
            self.manifest = (self.session_dir / "manifest.jsonl").open("a", buffering=1)
            self.started = time.time()
            self.write_session_json()

    def write_session_json(self, finished=False):
        import datetime, os
        meta = {"format_version": 2, "kind": "student_gui_drive", "session_id": self.session_dir.name,
                "started_at": datetime.datetime.fromtimestamp(self.started, datetime.timezone.utc).isoformat(), "started_wall_ns": int(self.started * 1e9),
                "views": {v: {"camera_id": CAMERA_IDS.get(v, 4), "udp_port": port} for v, port in self.capture_views},
                "network_profile": SENSOR_PROFILE,
                "ego_source": "/localization/ego_status", "object_source": "/perception/objects",
                "duration_seconds": round(time.time() - self.started, 1), "every_frame": True,
                "timestamp_source": "MORAI UDP packet header (host wallclock, verified ~1ms offset)",
                "driver": {"type": "student_gui", "checkpoint": os.environ.get("STUDENT_CHECKPOINT"), "cruise_kmh": os.environ.get("STUDENT_CRUISE_KMH")}}
        if finished:
            meta["finished_at"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
            meta["finished_wall_ns"] = time.time_ns()
            meta["saved_per_view"] = dict(self.saved)
        (self.session_dir / "session.json").write_text(json.dumps(meta, indent=2))

    def save_frame(self, name, stamp_ns, jpeg):
        # Disk writes happen on a writer thread: a slow write (page-cache flush while the Drive
        # uploader zips a session) must never stop this thread from draining the camera sockets.
        if not self.manifest:
            return
        if getattr(self, "save_queue", None) is None:
            self.save_queue, self.save_dropped = queue.Queue(maxsize=3000), 0
            self.save_writer = threading.Thread(target=self._save_worker, name="camera-save", daemon=True)
            self.save_writer.start()
        try:
            self.save_queue.put_nowait((name, stamp_ns, jpeg))
        except queue.Full:
            self.save_dropped += 1

    def _save_worker(self):
        while True:
            item = self.save_queue.get()
            if item is None:
                return
            try:
                self._write_frame(*item)
            except OSError:
                pass

    def _write_frame(self, name, stamp_ns, jpeg):
        import os
        n = self.saved[name]
        rel = Path("raw") / name / "rgb" / f"{n:08d}_{stamp_ns}.jpg"
        tmp = self.session_dir / (str(rel) + ".tmp")
        tmp.write_bytes(jpeg)
        os.replace(tmp, self.session_dir / rel)
        self.manifest.write(json.dumps({"view": name, "camera_id": CAMERA_IDS.get(name, 4), "frame_index": n, "rgb_timestamp_ns": stamp_ns,
                                        "received_wall_ns": time.time_ns(), "rgb": str(rel), "bytes": len(jpeg)}) + "\n")
        self.saved[name] = n + 1

    def run(self):
        pin_io_thread("camera UDP input")
        sockets, assemblers = {}, {}
        try:
            for name, port in self.capture_views:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
                sock.bind((SENSOR_PROFILE["local_ip"], port)); sock.setblocking(False)
                sockets[sock], assemblers[name] = name, FrameAssembler()
            while not self.stop_event.is_set():
                ready, _, _ = select.select(list(sockets), [], [], 0.1)
                for sock in ready:
                    name = sockets[sock]
                    while True:
                        try:
                            packet, address = sock.recvfrom(65535)
                        except BlockingIOError:
                            break
                        if address[0] != SENSOR_PROFILE["simulator_ip"]:
                            continue
                        result = assemblers[name].feed(packet)
                        if result:
                            now = time.monotonic()
                            with self.lock:
                                if self.frames[name] and result[0] <= self.frames[name][-1][0]:
                                    continue
                                self.frames[name].append(result)
                                self.counts[name].append(now)
                            self.save_frame(name, result[0], result[1])
        finally:
            for sock in sockets:
                sock.close()
            if self.manifest:
                if getattr(self, "save_queue", None) is not None:
                    self.save_queue.put(None)
                    self.save_writer.join(timeout=60)
                self.manifest.close()
                self.write_session_json(finished=True)

    def snapshot(self, count=4, interval_ns=100_000_000):
        with self.lock:
            copies = {name: list(self.frames[name]) for name, _ in VIEWS}
        if any(not copies[name] for name, _ in VIEWS):
            return None
        latest = min(copies[name][-1][0] for name, _ in VIEWS)
        targets = [latest - interval_ns * i for i in reversed(range(count))]
        selected = []
        for target in targets:
            row = []
            for name, _ in VIEWS:
                row.append(min(copies[name], key=lambda item: abs(item[0] - target))[1])
            selected.append(row)
        return selected

    def rates(self):
        now = time.monotonic()
        with self.lock:
            return {name: sum(t >= now - 2.0 for t in self.counts[name]) / 2.0 for name, _ in VIEWS}


class RosCameraReceiver:
    """CameraReceiver drop-in for environments where MORAI's camera UDP is
    already consumed by another node (e.g. around_view) and republished as
    sensor_msgs/Image ROS topics, instead of being free for our own raw UDP
    socket on VIEWS' ports. Re-encodes each frame back to JPEG bytes so
    snapshot() output matches CameraReceiver's (stamp, jpeg) contract exactly,
    with no changes needed anywhere else in this file."""

    def __init__(self, topic_prefix="/around_view/camera", topic_suffix="image_raw"):
        import rospy
        from sensor_msgs.msg import Image
        self.lock = threading.Lock()
        self.input_views = camera_input_views()
        self.frames = {name: deque(maxlen=120) for name, _ in self.input_views}
        self.counts = {name: deque(maxlen=300) for name, _ in self.input_views}
        self.stop_event = threading.Event()
        self._warned_encoding = set()
        self._subs = [
            rospy.Subscriber(f"{topic_prefix}/{name}/{topic_suffix}", Image,
                              self._on_image, callback_args=name, queue_size=1, buff_size=2 ** 24)
            for name, _ in self.input_views
        ]

    def _on_image(self, msg, name):
        if msg.encoding != "bgr8":
            if name not in self._warned_encoding:
                self._warned_encoding.add(name)
                sys.stderr.write(f"[RosCameraReceiver] {name}: 지원하지 않는 encoding {msg.encoding!r} (bgr8만 처리)\n")
            return
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        ok, jpeg = cv2.imencode(".jpg", array, (cv2.IMWRITE_JPEG_QUALITY, 85))
        if not ok:
            return
        stamp = msg.header.stamp.secs * 1_000_000_000 + msg.header.stamp.nsecs
        now = time.monotonic()
        with self.lock:
            self.frames[name].append((stamp, jpeg.tobytes()))
            self.counts[name].append(now)

    def start(self):
        pass  # subscribers are already live from __init__

    def join(self, timeout=None):
        for sub in self._subs:
            sub.unregister()

    def snapshot(self, count=4, interval_ns=100_000_000):
        with self.lock:
            copies = {name: list(self.frames[name]) for name, _ in VIEWS}
        if any(not copies[name] for name, _ in VIEWS):
            return None
        latest = min(copies[name][-1][0] for name, _ in VIEWS)
        targets = [latest - interval_ns * i for i in reversed(range(count))]
        selected = []
        for target in targets:
            row = []
            for name, _ in VIEWS:
                row.append(min(copies[name], key=lambda item: abs(item[0] - target))[1])
            selected.append(row)
        return selected

    def rates(self):
        now = time.monotonic()
        with self.lock:
            return {name: sum(t >= now - 2.0 for t in self.counts[name]) / 2.0 for name, _ in VIEWS}


class RecordedCameraReceiver(threading.Thread):
    """Read live tmpfs frames; archived-file playback is retained for offline use."""
    def __init__(self, session_dir):
        super().__init__(daemon=True)
        self.session_dir = Path(session_dir)
        self.lock = threading.Lock()
        self.input_views = camera_input_views()
        self.frames = {name: deque(maxlen=120) for name, _ in self.input_views}
        self.counts = {name: deque(maxlen=300) for name, _ in self.input_views}
        self.latest_file = {name: None for name, _ in self.input_views}
        self.latest_received = {name: 0.0 for name, _ in self.input_views}
        self.cache_dir = Path(os.environ["STUDENT_CAMERA_CACHE"]) if os.environ.get("STUDENT_CAMERA_CACHE") else None
        self.stop_event = threading.Event()

    def read_live_frames(self):
        for name, _ in self.input_views:
            try:
                payload = (self.cache_dir / (name + ".frame")).read_bytes()
                stamp, received_ns = struct.unpack("!QQ", payload[:16])
                jpeg = payload[16:]
                wall_ns = time.time_ns()
                age = max((wall_ns - received_ns) / 1e9, (wall_ns - stamp) / 1e9)
                if not 0 <= age <= .5 or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
                    continue
                if self.latest_file[name] == (stamp, received_ns):
                    continue
            except (OSError, ValueError, struct.error):
                continue
            now = time.monotonic()
            with self.lock:
                self.frames[name].append((stamp, jpeg))
                self.counts[name].append(now)
                self.latest_received[name] = now - age
                self.latest_file[name] = (stamp, received_ns)

    def run(self):
        if self.cache_dir is not None:
            while not self.stop_event.is_set():
                self.read_live_frames()
                self.stop_event.wait(.01)
            return
        manifest_path = self.session_dir / "manifest.jsonl"
        manifest = None
        while not self.stop_event.is_set():
            if manifest is None:
                try:
                    manifest = manifest_path.open()
                    initial = manifest.readlines()[-120:]
                except OSError:
                    self.stop_event.wait(0.05)
                    continue
            else:
                initial = []
            lines = initial + manifest.readlines()
            for line in lines:
                try:
                    item = json.loads(line)
                    name = item["view"]
                    if name not in self.frames:
                        continue
                    newest = self.session_dir / item["rgb"]
                    if newest == self.latest_file[name]:
                        continue
                    stamp = int(item["rgb_timestamp_ns"])
                    jpeg = newest.read_bytes()
                    if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
                        continue
                except (OSError, ValueError, KeyError, TypeError):
                    continue
                now = time.monotonic()
                with self.lock:
                    self.frames[name].append((stamp, jpeg))
                    self.counts[name].append(now)
                    self.latest_received[name] = now - max(0.0, (time.time_ns() - int(item.get("received_wall_ns", time.time_ns()))) / 1e9)
                    self.latest_file[name] = newest
            self.stop_event.wait(0.05)
        if manifest is not None:
            manifest.close()

    def stale_views(self, max_age_s=0.5):
        now = time.monotonic()
        with self.lock:
            return [name for name, _ in VIEWS
                    if now - self.latest_received[name] > max_age_s]

    def snapshot(self, count=4, interval_ns=100_000_000):
        with self.lock:
            now = time.monotonic()
            if any(now - self.latest_received[name] > 0.5 for name, _ in VIEWS):
                return None
            copies = {name: list(self.frames[name]) for name, _ in VIEWS}
        if any(not copies[name] for name, _ in VIEWS):
            return None
        latest = min(copies[name][-1][0] for name, _ in VIEWS)
        return [[min(copies[name], key=lambda item: abs(item[0] - target))[1]
                 for name, _ in VIEWS]
                for target in (latest - interval_ns * i for i in reversed(range(count)))]

    def rates(self):
        now = time.monotonic()
        with self.lock:
            return {name: sum(t >= now - 2.0 for t in self.counts[name]) / 2.0 for name, _ in VIEWS}


class RosEgoReceiver:
    """Receive the training-time EgoVehicleStatus fields directly at MORAI rate."""
    def __init__(self, topic):
        import rospy
        from morai_msgs.msg import EgoVehicleStatus
        from sensor_msgs.msg import Imu

        # The installed Noetic rospy still calls Thread.isAlive on shutdown.
        if not hasattr(threading.Thread, "isAlive"):
            threading.Thread.isAlive = threading.Thread.is_alive
        self.lock = threading.Lock()
        self.samples = deque(maxlen=400)
        self.imu_samples = deque(maxlen=400)
        self.file_cache = AsyncJsonCache(self.EGO_FILE)
        if not rospy.core.is_initialized():
            rospy.init_node("morai_student_shadow", anonymous=True, disable_signals=True)
        self.subscriber = rospy.Subscriber(topic, EgoVehicleStatus, self.on_ego, queue_size=50)
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.on_imu, queue_size=50)

    def on_imu(self, message):
        sample = dict(stamp=message.header.stamp.to_sec(), frame_id=message.header.frame_id,
            quaternion=[message.orientation.x, message.orientation.y,
                        message.orientation.z, message.orientation.w],
            covariance=list(message.orientation_covariance), measured=True)
        sample['angular_velocity'] = [message.angular_velocity.x,
                                      message.angular_velocity.y,
                                      message.angular_velocity.z]
        with self.lock:
            self.imu_samples.append(sample)

    def on_ego(self, message):
        sample = {
            "measured": True,
            "received": time.monotonic(),
            "stamp": message.header.stamp.to_sec(),
            "x": float(message.position.x),
            "y": float(message.position.y),
            "heading_deg": float(message.heading),
            "speed_kmh": math.hypot(float(message.velocity.x), float(message.velocity.y)) * 3.6,
            "accel": float(message.accel),
            "brake": float(message.brake),
            "wheel_angle": float(message.wheel_angle),
        }
        with self.lock:
            self.samples.append(sample)

    EGO_FILE = os.environ.get("STUDENT_EGO_FILE", "/data/autodrive/ego.json")

    def pull_file(self):
        """The in-process rospy subscription was seen frozen for 4.5 s while the car moved (2026-09-28 00:20, hs_2 t+291).
        status_node (ROS container, reliable) writes ego.json at 20 Hz; take it whenever our own stream is older than 0.25 s."""
        try:
            latest = self.samples[-1] if self.samples else None
            if latest and time.monotonic() - latest["received"] < 0.25:
                return
            d = self.file_cache.value
            if not isinstance(d, dict):
                return
            age = time.time() - float(d["t"])
            if age > 0.5:
                return
            if latest and abs(float(d["x"]) - latest["x"]) < 1e-6 and abs(float(d["y"]) - latest["y"]) < 1e-6 and time.monotonic() - latest["received"] < 0.6:
                return   # same sample as our own latest
            sample = {"received": time.monotonic() - age, "stamp": float(d.get('stamp', 0.)), "x": float(d["x"]), "y": float(d["y"]), "heading_deg": float(d["heading_deg"]),
                      "speed_kmh": float(d["speed_kmh"]), "accel": float(d.get("accel", 0)), "brake": float(d.get("brake", 0)), "wheel_angle": float(d.get("wheel_angle", 0)), "src": "file"}
            with self.lock:
                self.samples.append(sample)
        except (OSError, ValueError, KeyError):
            pass

    def snapshot(self, count, interval_s=0.1):
        self.pull_file()
        now = time.monotonic()
        with self.lock:
            samples = list(self.samples)
        if not samples:
            return None, [], 0.0
        latest = samples[-1]
        targets = [latest["received"] - interval_s * i for i in reversed(range(count))]
        history = [min(samples, key=lambda item: abs(item["received"] - target)) for target in targets]
        hz = sum(item["received"] >= now - 2.0 for item in samples) / 2.0
        ego = dict(latest)
        ego["age_s"] = now - latest["received"]
        return ego, history, hz

    def at_stamp(self, stamp):
        with self.lock:
            samples = list(self.samples)
        return pose_at_stamp(samples, stamp)

    def measured_at_stamp(self, stamp):
        """Supply only source-stamped ROS measurements for mapped ground.

        File fallback and velocity extrapolation cannot authorize removing a
        raw return. A delayed cloud may become eligible once its measured
        bracketing pose arrives; the LiDAR source timestamp remains unchanged.
        """
        from perception.source_pose import measured_pose_at
        from perception.imu_attitude import measured_imu_at
        try:
            stamp = float(stamp)
            now = time.time()
            age = now-stamp
            if not math.isfinite(age) or not 0. <= age <= .2:
                return None
            with self.lock:
                records = [(r['stamp'], r['x'], r['y'], math.radians(r['heading_deg']),
                            r['speed_kmh']/3.6, 0.) for r in self.samples
                           if r.get('measured') is True and r.get('src') != 'file'
                           and r['stamp'] <= now]
                imu_samples = [r for r in self.imu_samples if r['stamp'] <= now]
            pose = measured_pose_at(records, stamp, nearest_limit=0., bracket_limit=.2)
            if pose is None or abs(pose[0]-stamp) > 1e-9:
                return None
            attitude = measured_imu_at(imu_samples, stamp)
            if attitude is None:
                return None
            return dict(stamp=stamp, x=float(pose[1]), y=float(pose[2]),
                        heading_deg=math.degrees(pose[3]), age_s=age, measured=True,
                        imu_age_s=now-attitude['imu_stamp'], **attitude)
        except (KeyError, TypeError, ValueError, OverflowError):
            return None

    def measured_xy_at_stamp(self, stamp):
        """Measured planar ROS bracket only; no IMU requirement or extrapolation.

        Scan age is checked by the road-frame caller. Keeping that separate
        allows a just-arrived pose bracket to validate the original scan
        without changing its source time or treating a file pose as measured.
        """
        from perception.source_pose import measured_pose_at
        try:
            with self.lock:
                records = [(r['stamp'], r['x'], r['y'], math.radians(r['heading_deg']),
                            r['speed_kmh']/3.6, 0.) for r in self.samples
                           if r.get('measured') is True and r.get('src') != 'file']
            pose = measured_pose_at(records, stamp, nearest_limit=0., bracket_limit=.2)
            if pose is None or abs(pose[0]-float(stamp)) > 1e-9:
                return None
            return dict(stamp=float(stamp), x=float(pose[1]), y=float(pose[2]),
                        heading_deg=math.degrees(pose[3]), speed_kmh=float(pose[4])*3.6,
                        measured=True)
        except (KeyError, TypeError, ValueError, OverflowError):
            return None

    def close(self):
        self.subscriber.unregister()
        if getattr(self, 'imu_subscriber', None) is not None:
            self.imu_subscriber.unregister()
        self.file_cache.close()


def pedestrian_velocity(samples):
    """Estimate measured motion without projecting short-baseline position noise."""
    return measured_velocity(samples)


class RosObjectReceiver:
    """MORAI /Object_topic (npc + pedestrian + obstacle lists, world frame). Used by the hazard rule, not by the model."""
    def __init__(self, topic="/Object_topic"):
        import rospy
        from morai_msgs.msg import ObjectStatusList
        self.lock = threading.Lock()
        self.objects, self.received = [], 0.0
        self._tracks = {}
        self._motion = {}
        self._turn = {}
        self._position_history = {}
        self.file_cache = AsyncJsonCache(self.OBJ_FILE)
        if not rospy.core.is_initialized():
            rospy.init_node("morai_student_shadow", anonymous=True, disable_signals=True)
        self._source_clock = rospy.get_time
        self._snapshot_lock = threading.RLock()
        self.subscriber = rospy.Subscriber(topic, ObjectStatusList, self.on_objects, queue_size=2)

    def on_objects(self, m):
        objs = []
        now = time.monotonic()
        header_stamp = getattr(getattr(m, "header", None), "stamp", None)
        stamp = float(header_stamp.to_sec()) if header_stamp is not None else 0.
        source_now = getattr(self, "_source_clock", time.time)()
        previous_stamp = getattr(self, "_source_stamp", 0.)
        if stamp > 0. and previous_stamp > 0. and stamp <= previous_stamp:
            if source_now >= previous_stamp-.05:
                return  # Repeated/older frames cannot refresh freshness or confirmation.
            self._tracks, self._motion, self._turn, self._position_history = {}, {}, {}, {}
        lag = source_lag_s(stamp, source_now)
        tracks = {}
        motion = {}
        turn = {}
        # A missed semantic frame must not reset a pedestrian's motion history.
        histories = {key: history for key, history in self._position_history.items()
                     if key[0] == "pedestrian" and history and now - history[-1][0] <= 1.0}
        for kind, lst in (("npc", m.npc_list), ("pedestrian", m.pedestrian_list), ("obstacle", m.obstacle_list)):
            for o in lst:
                spd = math.hypot(o.velocity.x, o.velocity.y) / 3.6   # MORAI ObjectStatus velocity is km/h (verified from recorded positions)
                hd = math.radians(float(o.heading))
                vx, vy = spd * math.cos(hd), spd * math.sin(hd)
                key = (kind, o.unique_id)
                previous = self._tracks.get(key)
                anchor = (now, float(o.position.x), float(o.position.y), hd)
                if previous is not None:
                    dt = now - previous[0]
                    if dt < 0.12:
                        anchor = previous
                old_turn = getattr(self, "_turn", {}).get(key)
                if previous is not None and 0.12 <= now - previous[0] <= 0.6 and len(previous) >= 4:
                    angle = (hd - previous[3] + math.pi) % (2 * math.pi) - math.pi
                    rate = angle / (now - previous[0])
                    if abs(rate) <= 1.2:
                        rate = .5 * rate + .5 * old_turn[1] if old_turn else rate
                        turn[key] = (now, rate)
                if key not in turn and old_turn and now - old_turn[0] <= .3:
                    turn[key] = old_turn
                tracks[key] = anchor
                history = list(getattr(self, "_position_history", {}).get(key, []))
                history = [sample for sample in history if now - sample[0] <= 10.0]
                if history and math.hypot(o.position.x-history[-1][1], o.position.y-history[-1][2]) > max(3., spd * (now-history[-1][0]) * 2):
                    history = []
                if not history or now - history[-1][0] >= .08:
                    history.append((now, float(o.position.x), float(o.position.y)))
                histories[key] = history
                if kind == "pedestrian" and spd < 0.3:
                    measured = pedestrian_velocity(history)
                    if measured is not None:
                        vx, vy = measured
                        spd = math.hypot(vx, vy)
                        motion[key] = (now, vx, vy)
                circle = fit_circle_motion(history, float(o.heading)) if kind == "npc" and spd > 1. else None
                # Missing pedestrian dimensions must not become a point obstacle.
                length = max(float(o.size.x), 0.6 if kind == "pedestrian" else 0.3)
                width = max(float(o.size.y), 0.6 if kind == "pedestrian" else 0.3)
                shape = dict(height_m=float(getattr(o.size, 'z', 0.)),
                             raw_size_m=[float(o.size.x), float(o.size.y)])
                proof = decode_static_geometry(o.name, uid=o.unique_id, kind=kind,
                    position_xy=(o.position.x, o.position.y), size_xy=(o.size.x, o.size.y),
                    heading_deg=float(o.heading), velocity_xy=(vx, vy), speed_mps=spd,
                    height_m=shape['height_m'], source_stamp=stamp, measured_stamp=source_now)
                if proof is not None and getattr(o, 'type', None) == 2:
                    shape['producer_static_geometry'] = proof
                objs.append((o.position.x, o.position.y, max(length, width), canonical_static_name(o.name), kind, spd,
                             length, width, float(o.heading), vx, vy, int(o.unique_id),
                             turn.get(key, (now, 0.0))[1], circle, None, 0., None,
                             shape))
        self._tracks, self._motion, self._turn, self._position_history = tracks, motion, turn, histories
        with self.lock:
            self.objects, self.received = objs, time.monotonic()
            self._source_stamp, self._source_lag_s = stamp, lag

    OBJ_FILE = os.environ.get("STUDENT_OBJECTS_FILE", "/data/autodrive/objects.json")

    def snapshot(self, max_age_s=1.0):
        if not hasattr(self, '_snapshot_lock'):
            self._snapshot_lock = threading.RLock()
        with self._snapshot_lock:
            return self._snapshot(max_age_s)

    def snapshot_with_time(self, max_age_s=1.0):
        if not hasattr(self, '_snapshot_lock'):
            self._snapshot_lock = threading.RLock()
        with self._snapshot_lock:
            objects = self._snapshot(max_age_s)
            return objects, self.observation_stamp, self.age_s

    def _snapshot(self, max_age_s=1.0):
        with self.lock:
            own, age = list(self.objects), time.monotonic() - self.received
            stamp = getattr(self, "_source_stamp", 0.)
            lag = getattr(self, "_source_lag_s", 0.)
        self.receive_age_s = age
        age += lag
        self.observation_stamp = stamp or None
        self.src = "ros"
        if age > 0.25:   # our rospy stream is late: use status_node's 10 Hz objects.json (same tuple format) when it is fresher
            try:
                d = self.file_cache.value
                if not isinstance(d, dict):
                    d = {}
                fage = time.time() - float(d["t"]) + float(d.get("source_lag_s", 0.))
                if math.isfinite(fage) and 0. <= fage < 0.5 and fage < age:
                    self.src = "file"
                    self.age_s = fage
                    self.observation_stamp = float(d.get("source_stamp", 0.)) or None
                    rows = []
                    for original in d["objects"]:
                        row = list(original)
                        if len(row) > 3:
                            row[3] = canonical_static_name(row[3])
                        if len(row) > 17 and isinstance(row[17], dict):
                            row[17] = {key: value for key, value in row[17].items()
                                       if key not in ('producer_static_geometry', 'road_static_geometry')}
                        rows.append(tuple(row))
                    return rows
            except (OSError, ValueError, KeyError):
                pass
        self.age_s = age
        return own if age <= max_age_s else []

    def close(self):
        self.subscriber.unregister()
        self.file_cache.close()


def deduplicate_obstacles(objects, tracked_objects=None, radius_m=1.5):
    """Remove compact semantic duplicates, retaining separate nearby boxes.

    Distance is only a candidate gate. Pedestrian fragments must have a short
    side <= 1.2 m and a long side <= 1.8 m, then be within 0.8 m or overlap
    the person's oriented footprint. Missing/invalid dimensions keep the box.
    """
    tracked = [o for o in highway_measured_objects(objects if tracked_objects is None else tracked_objects)
               if o[4] in ("pedestrian", "npc")]

    def footprint(obj):
        if len(obj) < 9:
            return None
        values = np.asarray([obj[0], obj[1], obj[6], obj[7], obj[8]], dtype=float)
        if not np.isfinite(values).all() or np.min(values[2:4]) <= 0.:
            return None
        h = math.radians(values[4])
        axes = np.array([[math.cos(h), math.sin(h)], [-math.sin(h), math.cos(h)]])
        return values[:2], values[2:4] / 2., axes

    def duplicate(obj, target):
        distance = math.hypot(obj[0] - target[0], obj[1] - target[1])
        if distance > radius_m:
            return False
        box, body = footprint(obj), footprint(target)
        if box is None or body is None:
            return False
        centre, half, axes = box
        other_centre, other_half, other_axes = body
        if target[4] == "pedestrian":
            # Run 438 inspector: the walker's own obstacle box measured 1.87 x 1.10 m with its
            # centre 0.10 m from the person; the padded long side may pass 1.8 m when the
            # centres coincide.
            long_limit = 1.95 if distance <= .3 else 1.8
            if min(half) * 2 > 1.2 or max(half) * 2 > long_limit:
                return False
        elif np.any(half > other_half):
            return False
        if distance <= .8:
            return True
        # Separating-axis test, without padding: adjacency is not overlap.
        for axis in np.vstack((axes, other_axes)):
            extent = np.abs(axes @ axis) @ half + np.abs(other_axes @ axis) @ other_half
            if abs((centre - other_centre) @ axis) >= extent:
                return False
        return True

    people = [t for t in tracked if t[4] == "pedestrian"]

    def ghost_vehicle(obj):
        # A walker's LiDAR cluster painted "vehicle" is grown to a 4.6 x 1.9 m car box
        # beside the crossing; the car then queued behind that "parked vehicle" for
        # 165 s (auto run 466). A standing car box born within 2.5 m of a tracked person
        # is dropped here; its real LiDAR points still reach the static planner.
        if obj[4] != "obstacle" or "parked_vehicle" not in str(obj[3]):
            return False
        return (float(obj[5]) < .5 and
                any(math.hypot(obj[0] - t[0], obj[1] - t[1]) <= 2.5 for t in people))

    return [o for o in objects if o[4] != "obstacle" or
            not (ghost_vehicle(o) or any(duplicate(o, t) for t in tracked))]


def route_hazard(objects, ego, dense_route, ahead_m=25.0, half_width_m=1.8, max_speed_mps=None, debug=None):
    """Nearest object sitting on the route ahead: (along_m, lateral_m, name, kind) or None. lateral > 0 = object left of route.
    max_speed_mps: ignore objects moving faster than this (a bypass is only planned around (near-)static objects)."""
    route = np.asarray(dense_route, dtype=np.float32)[:, :2]
    if len(route) < 3 or not objects:
        return None
    tang = np.gradient(route, axis=0); tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-6)
    normal = np.column_stack((-tang[:, 1], tang[:, 0]))
    s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))))
    best = None
    yaw = math.radians(float(ego["heading_deg"]))
    for x, y, size, name, kind, speed, *extra in objects:
        if max_speed_mps is not None and speed > max_speed_mps:
            continue
        f, l = local_xy(x, y, ego)
        min_ahead = 1.0 if ahead_m <= 10.0 else 3.0   # stop check: anything from 1 m ahead; bypass trigger: from 3 m
        if f < min_ahead - 2.0:    # behind the car: not a hazard for the path ahead
            continue
        p = np.array([f, l], dtype=np.float32)
        k = int(np.argmin(((route - p) ** 2).sum(axis=1)))
        along, lat = float(s[k]), float(((p - route[k]) * normal[k]).sum())
        if len(extra) >= 3:   # oriented footprint: extent across / along the route direction
            sx, sy, hd = float(extra[0]), float(extra[1]), math.radians(float(extra[2]))
            d = hd - (yaw + math.atan2(float(tang[k, 1]), float(tang[k, 0])))
            ext_lat = 0.5 * (abs(sx * math.sin(d)) + abs(sy * math.cos(d)))
            ext_along = 0.5 * (abs(sx * math.cos(d)) + abs(sy * math.sin(d)))
        else:
            ext_lat = ext_along = 0.5 * float(size)
        ext_lat = max(0.15, min(ext_lat, 0.5 * float(size)))
        front = along - min(ext_along, 3.0)   # nearest face, not the centre
        back = along + min(ext_along, 3.0)
        lat_ok = abs(lat) - ext_lat <= half_width_m
        # a long object whose near face is already beside the bumper still blocks the path while its far face is ahead
        along_ok = back >= min_ahead and front <= ahead_m and f >= min_ahead - 2.0
        if debug is not None and f < max(30.0, ahead_m + 5.0):
            debug.append({"n": name[:18], "k": kind, "f": round(f, 1), "l": round(l, 1), "along": round(along, 1), "lat": round(lat, 2),
                          "ext": round(ext_lat, 2), "front": round(front, 1), "v": round(float(speed), 2), "ok": bool(lat_ok and along_ok)})
        if along_ok and lat_ok:
            if best is None or front < best[0]:
                best = (max(front, min_ahead), lat, name, kind, float(speed))
    return best


def fresh_road_static_hazard(objects, ego, path, horizon_m, half_width_m, points, *, enabled=False,
                             margin_recovery_state=None, source_age_s=0.):
    """Recheck fresh generic static bodies in the normal-road planner's geometry.

    The caller owns the ordinary-road scope. People, motion and other semantic
    classes retain the original route check. No prior planning snapshot can
    replace a newly observed body, and missing raw evidence retains its full
    source dimensions.
    """
    objects = list(objects)
    if not enabled:
        return route_hazard(objects, ego, path, horizon_m, half_width_m), dict(active=False)
    import model_avoidance as ma
    generic, other = [], []
    for obj in objects:
        try:
            known = (len(obj) >= 9 and obj[3] == 'fusion_obstacle' and obj[4] == 'obstacle'
                     and np.isfinite(np.asarray([obj[i] for i in (0, 1, 2, 5, 6, 7, 8)], float)).all()
                     and 0. <= float(obj[5]) < ma.OBSTACLE_STATIC_MPS
                     and min(float(obj[6]), float(obj[7])) > 0.)
        except (TypeError, ValueError, OverflowError):
            known = False
        (generic if known else other).append(obj)
    on_path = route_hazard(other, ego, path, horizon_m, half_width_m) if other else None
    info = dict(active=True, static_n=len(generic), normalized_n=0, clear_m=None,
                horizon_m=float(horizon_m))
    if not generic:
        return on_path, info
    generic = ma.validate_road_geometry_sources(generic, ego)
    stationary, _ = ma.split_static(generic)
    checked = ma.road_static_objects(stationary, ego, points, allow_producer_geometry=True)
    if margin_recovery_state is not None:
        checked = ma.road_static_margin_recovery(checked, ego, points,
            recovery_ledger=margin_recovery_state, read_only=True)
    info['normalized_n'] = sum('road_static_geometry' in object_metadata(o) for o in checked)
    local = np.asarray(path, dtype=float)
    valid_path = bool(local.ndim == 2 and local.shape[1] >= 2 and len(local) >= 2
                      and np.isfinite(local).all())
    length = float(np.linalg.norm(np.diff(local[:, :2], axis=0), axis=1).sum()) if valid_path else 0.
    limit = float(horizon_m)
    covered = bool(valid_path and math.isfinite(limit) and limit > 0. and length >= limit-1e-6)
    info['path_horizon_covered'] = covered
    checked_limit = min(limit, length) if valid_path and math.isfinite(limit) and limit > 0. else 0.
    clear = (static_clear_distance(local[:, :2], ego, checked, checked_limit,
                                  body_shift_m=REAR_AXLE_SHIFT_M) if checked_limit > 0. else 0.)
    info['clear_m'] = float(clear) if math.isfinite(clear) else None
    try:
        observed_age = float(source_age_s)
        pose_age = float(ego.get('age_s', 0.))
        speed = abs(float(ego.get('speed_kmh', math.inf)))/3.6
    except (ValueError, TypeError, OverflowError):
        observed_age, pose_age, speed = math.inf, math.inf, math.inf
    if (not covered and valid_path and length >= 8. and points is not None
            and np.isfinite(np.asarray(points, float)).all()
            and 0. <= observed_age <= .3 and 0. <= pose_age <= .2
            and clear >= checked_limit-1e-6):
        from path_safety import stop_cap
        from lead_follow import EMERGENCY_DECEL_MPS2
        age = max(pose_age, observed_age)
        lag = age+.15
        gap = max(0., length-1.)
        room = gap-speed*lag
        if (math.isfinite(speed) and math.isfinite(age) and age >= 0. and room > .1
                and speed*speed/(2.*room) <= EMERGENCY_DECEL_MPS2):
            # The end of a checked prefix is a stopping boundary, not a body
            # at the bumper. All checked geometry remains clear; issue its
            # braking cap while the full manoeuvre is searched again.
            info['checked_path_end_cap_mps'] = stop_cap(gap, lag, 2.5)
            return on_path, info
    if (covered and math.isfinite(clear) and clear < limit-1e-6
            and points is not None and np.isfinite(np.asarray(points, float)).all()
            and 0. <= observed_age <= .3 and 0. <= pose_age <= .2):
        from path_safety import stop_cap
        from lead_follow import EMERGENCY_DECEL_MPS2
        lag = max(pose_age, observed_age)+.15
        gap = max(0., clear-1.)
        room = gap-speed*lag
        if (math.isfinite(speed) and room > .1
                and speed*speed/(2.*room) <= EMERGENCY_DECEL_MPS2):
            # A measured static body at braking distance owns a speed cap.
            # The independent live raw/body sweep still vetoes actual contact.
            info['static_braking_cap_mps'] = stop_cap(gap, lag, 2.5)
    if not covered or not math.isfinite(clear) or clear < limit-1e-6:
        static_hit = (max(0., float(clear)) if covered and math.isfinite(clear) else 0.,
                      0., 'fusion_obstacle', 'obstacle', 0.)
        if on_path is None or static_hit[0] < on_path[0]:
            on_path = static_hit
    return on_path, info


def current_static_guard_frame(runtime, ego, path):
    """Check a late object snapshot in a matching measured pose/scan frame.

    Model search can span several sensor publications. Reusing its local scan
    with newly received world boxes creates a different physical footprint.
    Preserve the selected world curve and transform an atomic current scan;
    missing measurements remain a stop, without extrapolating the vehicle.
    """
    from perception.road_scan_frame import align_road_scan_wait
    from planning.highway.path_continuity import projected_suffix
    deadline = time.monotonic()+.1
    first_sequence = None
    while True:
        current, _, _ = runtime.ego_receiver.snapshot(1)
        captured_at = time.monotonic()
        scan = runtime.lidar.scan_snapshot() if runtime.lidar is not None else {}
        if first_sequence is None:
            first_sequence = scan.get('sequence')
        frame = align_road_scan_wait(scan, current,
            getattr(runtime.ego_receiver, 'measured_xy_at_stamp', None),
            captured_at=captured_at, wait_s=max(0., deadline-captured_at))
        if frame['proof']['valid']:
            if scan.get('sequence') != first_sequence:
                frame['proof']['replaced_sequence'] = first_sequence
            break
        # A new atomic scan can arrive while the search or source-pose check
        # finishes. Re-read it within the existing 100 ms synchronization
        # budget; never extend the rejected scan's lifetime or mask malformed
        # measurements and discontinuous poses.
        transient = frame['proof']['reason'] in (
            'road scan source stale', 'road scan source pose unavailable',
            'road scan target pose stale', 'road scan target measurement unavailable')
        remaining = deadline-time.monotonic()
        if not transient or remaining <= 0.:
            break
        time.sleep(min(.002, remaining))
    if not frame['proof']['valid']:
        return None, None, None, frame['proof']
    local = np.asarray(path, float)[:, :2]
    def axes(pose):
        yaw = math.radians(float(pose['heading_deg']))
        return np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    world = local @ axes(ego).T + [ego['x'], ego['y']]
    try:
        current_path = projected_suffix((world-[current['x'], current['y']]) @ axes(current))
    except (ValueError, TypeError, IndexError, OverflowError) as error:
        return None, None, None, dict(frame['proof'], valid=False,
            reason='static guard world path does not match current measured pose',
            projection_error=str(error))
    return current, current_path, frame['points'], frame['proof']


def highway_side_gaps(path, points, tree, horizon=90.):
    """Arc, left and right gap (m) from the 1.892 m body to LiDAR points beside each path sample.

    Points within the body length beside a sample count; 2.0 when nothing is near."""
    path = np.asarray(path, dtype=float)[:, :2]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    n = int(np.searchsorted(arc, horizon))
    left, right = np.full(n, 2.), np.full(n, 2.)
    if points is None or tree is None or not len(points) or n < 2:
        return arc[:n], left, right
    tangent = np.gradient(path, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    for i, near in enumerate(tree.query_ball_point(path[:n], 3.4)):
        if not near:
            continue
        rel = np.asarray(points)[near, :2] - path[i]
        lon = rel @ tangent[i]
        lat = rel @ np.array([-tangent[i, 1], tangent[i, 0]])
        beside = np.abs(lon) < 2.4
        if np.any(beside & (lat > .3)):
            left[i] = min(2., float(lat[beside & (lat > .3)].min()) - .946)
        if np.any(beside & (lat < -.3)):
            right[i] = min(2., float(-lat[beside & (lat < -.3)].max()) - .946)
    return arc[:n], left, right


def highway_nudge(path, points, tree, speed_mps, horizon=90.):
    """Propose a smooth, bounded shift toward measured roadside clearance."""
    from planning.highway.nudge_geometry import nudge_geometry
    path = np.asarray(path, dtype=float)[:, :2]
    arc, left, right = highway_side_gaps(path, points, tree, horizon)
    tight = np.flatnonzero(np.minimum(left, right) < HIGHWAY_NUDGE_WANT_M)
    if not len(tight):
        return path
    want = np.clip((left[tight] - right[tight]) / 2., -HIGHWAY_NUDGE_MAX_M, HIGHWAY_NUDGE_MAX_M)
    keep = np.abs(want) >= .05
    if not keep.any():
        return path
    stations, want = arc[tight][keep], want[keep]
    return nudge_geometry(path, stations, want, speed_mps, HIGHWAY_NUDGE_MAX_M)








def highway_side_gap_cap(path, points, tree, horizon=90., decel=3.):
    """Speed (m/s) from which we can brake to the side-gap speed before a tight passage."""
    arc, left, right = highway_side_gaps(path, points, tree, horizon)
    gap = np.minimum(left, right)
    cap = float("inf")
    for limit, kmh in HIGHWAY_SIDE_GAP_KMH:
        hit = np.flatnonzero(gap < limit)
        if len(hit):
            cap = min(cap, math.sqrt((kmh / 3.6) ** 2 + 2. * decel * max(0., float(arc[hit[0]]) - 3.)))
    return cap


def static_clear_distance(path, ego, objects, horizon_m=35.0, body_shift_m=ROAD_LEGACY_SHIFT_M):
    """Return the first overlapping vehicle pose distance, or the checked path length.

    body_shift_m: the body box centre ahead of each path sample along its heading.
    The pose is the rear axle: the tunnel passes ego_body.REAR_AXLE_SHIFT_M (the real
    body); the road/highway default keeps the legacy box centred on the sample."""
    route = np.asarray(path, dtype=float)[:, :2]
    if len(route) < 2:
        return 0.0
    if np.linalg.norm(route[0]) > 0.01:
        route = np.vstack(([0.0, 0.0], route))
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    samples = np.arange(0.0, min(horizon_m, arc[-1]) + 0.01, 0.25)
    centres = np.column_stack([np.interp(samples, arc, route[:, i]) for i in (0, 1)])
    if len(centres) < 2:
        return 0.0
    tang = np.gradient(centres, axis=0)
    tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-6)
    tang[0] = (1.0, 0.0)
    normal = np.column_stack((-tang[:, 1], tang[:, 0]))
    clearance = min(float(horizon_m), float(arc[-1]))
    forward_prefix = centres[:max(2, min(len(centres), int(5.0 / 0.25) + 1)), 0]
    centres = body_centres(centres, tang, body_shift_m)
    objects = list(objects)
    if not objects:
        return clearance
    # Parse scalar fields with the same bounds and math operations as the
    # object-wise implementation, then test all object/pose pairs at once.
    positions, axes, lengths, widths = [], [], [], []
    for obj in objects:
        positions.append(local_xy(obj[0], obj[1], ego))
        yaw = math.radians((obj[8] if len(obj) > 8 else ego["heading_deg"]) - ego["heading_deg"])
        axes.append((math.cos(yaw), math.sin(yaw)))
        lengths.append(max(0.3, float(obj[6] if len(obj) > 6 else obj[2])) / 2)
        widths.append(max(0.3, float(obj[7] if len(obj) > 7 else obj[2])) / 2)
    positions, axes = np.asarray(positions), np.asarray(axes)
    lengths, widths = np.asarray(lengths), np.asarray(widths)
    cross = np.column_stack((-axes[:, 1], axes[:, 0]))
    # Keep the original rear-object exception, including its future re-entry
    # check. A path that returns towards a rear object must still test it.
    rear_face = positions[:, 0] + lengths * abs(axes[:, 0]) + widths * abs(cross[:, 0])
    keep = np.ones(len(objects), dtype=bool)
    behind = np.flatnonzero((rear_face < -0.25) & np.all(np.diff(forward_prefix) >= -0.05))
    if len(behind):
        future = centres[samples >= 5.0]
        if not len(future):
            keep[behind] = False
        else:
            delta = future[None, :, :] - positions[behind, None, :]
            radius = (math.hypot(EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M) +
                      np.array([math.hypot(lengths[i], widths[i]) for i in behind]) + 0.2)
            keep[behind] = ~(np.linalg.norm(delta, axis=2).min(axis=1) > radius)
    if not keep.any():
        return clearance
    # NaN comparisons retain the old failure behavior. In particular, a NaN
    # future distance does not establish that a rear object can be discarded.
    positions, axes, cross = positions[keep], axes[keep], cross[keep]
    lengths, widths = lengths[keep, None], widths[keep, None]
    margins = np.array([0.65 if obj[4] == "pedestrian" else 0.2
                        for obj, retained in zip(objects, keep) if retained])[:, None]
    if abs(body_shift_m-REAR_AXLE_SHIFT_M) < 1e-6:
        retained_objects = [obj for obj, retained in zip(objects, keep) if retained]
        floors = [(object_metadata(obj).get('road_static_geometry') or {}).get('initial_margin_floor_m')
                  if obj[4] == 'obstacle' else None for obj in retained_objects]
        if any(value is not None for value in floors):
            margins = np.broadcast_to(margins, (len(floors), len(samples))).copy()
            for i, floor in enumerate(floors):
                if floor is not None and math.isfinite(float(floor)) and .12 <= floor < .2:
                    # Never shrink the box. Recover only its existing padding
                    # deficit without approaching it; restore the full margin
                    # after the rear body has cleared this same object.
                    start = float((object_metadata(retained_objects[i]).get('road_static_geometry') or {}).get(
                        'initial_margin_restore_start_m', 0.))
                    if not math.isfinite(start) or not 0. <= start <= 10.:
                        continue
                    u = np.clip((samples-start)/2., 0., 1.)
                    restored = u**3*(10.-15.*u+6.*u*u)
                    margins[i] = .2-(.2-float(floor)+1e-4)*(1.-restored)
    dx = positions[:, 0, None] - centres[None, :, 0]
    dy = positions[:, 1, None] - centres[None, :, 1]
    tx, ty = tang[:, 0], tang[:, 1]
    nx, ny = normal[:, 0], normal[:, 1]
    ax, ay = axes[:, 0, None], axes[:, 1, None]
    cx, cy = cross[:, 0, None], cross[:, 1, None]
    at, an = abs(ax * tx + ay * ty), abs(ax * nx + ay * ny)
    ct, cn = abs(cx * tx + cy * ty), abs(cx * nx + cy * ny)
    tn = abs(tx * nx + ty * ny)
    ac = abs(ax * cx + ay * cy)
    # The same four separating axes: ego tangent/normal and measured box axes.
    hl, hw = EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M
    overlap = abs(dx * tx + dy * ty) <= (
        hl * abs(tx * tx + ty * ty) + hw * tn + lengths * at + widths * ct + margins)
    overlap &= abs(dx * nx + dy * ny) <= (
        hl * tn + hw * abs(nx * nx + ny * ny) + lengths * an + widths * cn + margins)
    overlap &= abs(dx * ax + dy * ay) <= (
        hl * at + hw * an + lengths * abs(ax * ax + ay * ay) + widths * ac + margins)
    overlap &= abs(dx * cx + dy * cy) <= (
        hl * ct + hw * cn + lengths * ac + widths * abs(cx * cx + cy * cy) + margins)
    hit = np.flatnonzero(overlap.any(axis=0))
    return min(clearance, float(samples[hit[0]])) if len(hit) else clearance


def static_path_clear(path, ego, objects, horizon_m=35.0):
    return static_clear_distance(path, ego, objects, horizon_m) >= horizon_m - 1e-6


def static_approach_cap_mps(distance_m):
    """Slow before a static obstacle so a feasible bypass or stop can be followed."""
    return max(8.0, min(25.0, 8.0 + 0.5 * (float(distance_m) - 10.0))) / 3.6


def blocked_static_stop_cap_mps(distance_m, decel_mps2=1.5, clearance_m=4.0):
    """Speed envelope for a checked stop before an obstacle with no bypass yet."""
    return math.sqrt(2.0 * decel_mps2 * max(0.0, float(distance_m) - clearance_m))


def continue_static_plan(saved, ego, now, join_m=6.):
    """Carry the remaining world-frame path into the new ego frame, if still trackable."""
    if not saved or not 0.0 <= now - saved["stamp"] <= 1.0:
        return None
    local = np.array([local_xy(x, y, ego) for x, y in saved["world_path"]])
    if saved['plan'].get('static_motion_profile') or saved['plan'].get('bounded_static_pass'):
        from planning.highway.path_continuity import projected_suffix
        try:
            # Keep the same world curve at metric spacing. A projection a
            # millimetre before a saved vertex is not a real wheel-rate step.
            path = RosPathPublisher.resample(projected_suffix(local))
            length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
            if np.linalg.norm(path[0]) > .2 or length < 8.:
                return None
            return dict(saved['plan'], path=path, reused=True, fixed_world_curve=True,
                        clear_distance_m=min(length, saved['plan']['clear_distance_m']),
                        hold_until_m=min(length, saved['plan']['hold_until_m']))
        except (TypeError, ValueError, IndexError):
            return None
    nearest = int(np.argmin(np.linalg.norm(local, axis=1)))
    path = local[nearest:]
    if len(path) < 3 or np.linalg.norm(path[0]) > 0.8:
        return None
    tangent = path[min(3, len(path) - 1)] - path[0]
    if abs(math.atan2(tangent[1], tangent[0])) > math.radians(35):
        return None
    # Join from the actual ego pose. A nearest waypoint can be behind the car;
    # prepending the origin to that point creates a reverse segment and a sharp
    # curvature spike in the follower even though the saved path itself is smooth.
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    # Keep the checked vertices. Linear upsampling adds straight midpoints
    # between them and doubles the discrete curvature at each original turn.
    # The short pose/heading join still undergoes every ordinary safety check.
    sample, unique = np.unique(arc, return_index=True)
    if len(sample) < 3:
        return None
    reference = path[unique]
    u = np.clip(sample / float(join_m), 0., 1.)
    weight = u ** 3 * (10 - 15 * u + 6 * u ** 2)
    initial_tangent = (reference[1] - reference[0]) / (sample[1] - sample[0])
    correction = reference[0] + sample[:, None] * (initial_tangent - np.array([1., 0.]))
    joined = reference - correction * (1 - weight[:, None])
    return dict(saved["plan"], path=joined, reused=True)


# Trial stop applies only to the explicitly selected collision intersection.
YIELD_AT_JUNCTION = os.environ.get("STUDENT_INTERSECTION_YIELD",
                                 os.environ.get("STUDENT_STOP_AT_STOP_LINE", "0")) == "1"
RING_CONSTANT_SPEED = os.environ.get('STUDENT_RING_CONSTANT_SPEED', '1') == '1'


def junction_controller(lines, route_xy=None):
    """Keep fresh gap release and ongoing following without a mandatory stop."""
    if RING_CONSTANT_SPEED and route_xy is not None:
        route = np.asarray(route_xy, float)
        if (route.ndim == 2 and route.shape[1]>=2 and len(route)>=2
                and np.isfinite(route[:, :2]).all()):
            stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1))]
            adjusted = []
            for line in lines:
                current = dict(line)
                if str(current.get('id', '')).startswith('A1256W000709:'):
                    # This surveyed line is ~7.5 m before the actual ring body
                    # corridor. Approach its physical yield boundary while
                    # retaining the controller's stop buffer and atomic launch.
                    surveyed = float(current['station_m'])
                    station = surveyed+7.
                    if station<=stations[-1]:
                        current.update(station_m=station, surveyed_station_m=surveyed,
                            yield_advance_m=7., yield_launch_capture_m=3.,
                            xy=[float(np.interp(station, stations, route[:, axis]))
                                                   for axis in (0, 1)])
                adjusted.append(current)
            lines = adjusted
    return (competition_stop_controller(lines, require_full_stop=False)
            if YIELD_AT_JUNCTION else None)


# Brake for a static obstacle as if blocked until a bypass is validated (competition launch sets 1).
# Normal-road static avoidance: "lattice" (candidate paths, collision-only
# stops), "legacy" (geometric bypass with map-corridor stops) or "model" (the
# legacy host with the portable static planner and the v6 model's bypass side;
# see model_avoidance.py - Claude working copy, launched as the web's E2E mode).
PLANNER_MODE = os.environ.get("STUDENT_PLANNER", "legacy")
USE_LATTICE = PLANNER_MODE == "lattice"
MODEL_AVOID = PLANNER_MODE == "model"
if MODEL_AVOID:
    import model_avoidance
    model_avoidance.configure(MAX_CONTROL_CURVATURE, static_clear_distance)
TIGHT_PASS_KMH = float(os.environ.get("STUDENT_TIGHT_PASS_KMH", "5"))     # stopped, no path: creep with 0.12 m margin
STALL_RELAX_S = float(os.environ.get("STUDENT_STALL_RELAX_S", "8"))
# Leaving the legal lanes (shoulder, centre line) only after this long stalled with no
# in-lane path: line contact is a penalty, a car parked for good is a failed run.
STALL_LANE_ESCAPE_S = float(os.environ.get("STUDENT_STALL_LANE_ESCAPE_S", "25"))
# Moving and every in-lane path blocked: re-plan at once with the 1 m line allowance.
LANE_ESCAPE_ON_BLOCK = os.environ.get("STUDENT_LANE_ESCAPE_ON_BLOCK", "1") != "0"       # then 0.05 m margin + 2.5 m shoulder
# Reverse escape (competition: never park for good). Blocked within REVERSE_TRIGGER_M for
# REVERSE_AFTER_S with a LiDAR-clear rear: back up REVERSE_ESCAPE_M in R, then replan.
REVERSE_TRIGGER_M = float(os.environ.get("STUDENT_REVERSE_TRIGGER_M", "6"))
REVERSE_AFTER_S = float(os.environ.get("STUDENT_REVERSE_AFTER_S", "3"))
REVERSE_ESCAPE_M = float(os.environ.get("STUDENT_REVERSE_ESCAPE_M", "6"))
REVERSE_MAX_TRIES = 3
REVERSE_REFILL_S = 30.     # standstill after exhausting the escapes: grant one more (run 502)
REVERSE_MAX_REFILLS = 3
CRAWL_KMH = float(os.environ.get("STUDENT_CRAWL_KMH", "5"))   # below the lattice low-speed threshold (1.5 m/s)
OFF_LANE_CAR_M = 1.8
OFF_LANE_CAR_SPEED_MPS = 2.0
# Route index ranges where obstacles and people are perceived ("a-b,c-d").
# From the rule-book mission map (3-1): the static + dynamic obstacle
# sections between the signals at idx 635 and 879 (generous 480-1100), and
# the GPS-shadow tunnel (blackout 3777-4016). Everywhere else only vehicles:
# curbs, guard rails and painted islands stopped the car there.
STATIC_MISSION_IDX = [tuple(int(v) for v in part.split("-")) for part in
                      os.environ.get("STUDENT_STATIC_MISSION_IDX", "480-1100,3760-4030").split(",") if "-" in part]


def in_obstacle_mission(route_index):
    return any(a <= route_index <= b for a, b in STATIC_MISSION_IDX)


# Pedestrian (dynamic-obstacle) section speed. Fusion found a crossing person
# only 4-12 m ahead; at 40 km/h that forced a 7 m/s^2 stop (and an ESKF
# divergence) on 2026-10-03. "a-b:kmh" ranges.
# A separate 30 km/h static-pass preview below reaches the requested pace
# before admitting a lateral manoeuvre, rather than braking after a failed join.
# Static boxes are remembered (STATIC_OBJECT_MEMORY_S) so a bypass starts early.
DYNAMIC_ZONES = [(int(r.split("-")[0]), int(r.split("-")[1].split(":")[0]), float(r.split(":")[1]))
                 for r in os.environ.get("STUDENT_DYNAMIC_ZONE", "480-800:40,800-1100:30,3700-4030:40").split(",") if ":" in r]
# Tunnel: GPS blackout from route idx 3777, wall map 3782-4012. The launcher's
# "3700-4030:40" zone above is the 2026-10-03 pedestrian-section cap; inside the
# tunnel nothing is seen by camera and the LiDAR planner owns the speed (observed
# floor and obstacle clearance braked at 4 m/s^2, bypass curvature). It held the
# planner cap at <= 40 km/h for every tunnel frame of follow-20261010T151125Z. From
# the blackout start to the zone end the cap is TUNNEL_ZONE_KMH (competition limit
# 60 km/h on Ego status; <= 58 leaves the follower's overshoot room). The 41 m portal
# approach keeps its own cap: the first tunnel obstacle stands 8 m past the portal.
TUNNEL_ZONE_IDX = tuple(int(v) for v in os.environ.get("STUDENT_TUNNEL_ZONE_IDX", "3777-4030").split("-"))
TUNNEL_ZONE_KMH = min(58., float(os.environ.get("STUDENT_TUNNEL_ZONE_KMH", "55")))
# Surveyed physical tunnel only; preserve the existing portal/exit zone speeds.
TUNNEL_STATIC_SPEED_IDX = (3782, 4012)
TUNNEL_STATIC_SPEED_KMH = min(58., float(os.environ.get("STUDENT_TUNNEL_STATIC_SPEED_KMH", "58")))


def with_tunnel_zone(zones, span=TUNNEL_ZONE_IDX, kmh=TUNNEL_ZONE_KMH):
    """Dynamic zones with the tunnel span set to the tunnel cap (parts outside kept)."""
    lo, hi = span
    result = []
    for start, end, cap in zones:
        if end < lo or start > hi:
            result.append((start, end, cap))
            continue
        if start < lo:
            result.append((start, lo - 1, cap))
        result.append((max(start, lo), min(end, hi), kmh))
        if end > hi:
            result.append((hi + 1, end, cap))
    return result


DYNAMIC_ZONES = with_tunnel_zone(DYNAMIC_ZONES)
DYNAMIC_ZONES = with_tunnel_zone(DYNAMIC_ZONES, TUNNEL_STATIC_SPEED_IDX, TUNNEL_STATIC_SPEED_KMH)
DYNAMIC_ZONES.append((480, 770, 30.))
# Static boxes flicker in fusion (seen at 32 m, gone, back at 12 m): remember
# them so the bypass starts at first sight.
STATIC_OBJECT_MEMORY_S = float(os.environ.get("STUDENT_STATIC_MEMORY_S", "3.0"))


DYNAMIC_CAUTION_MAX_KMH = float(os.environ.get("STUDENT_DYNAMIC_CAUTION_KMH", "25"))
# Route span with walls/barriers beside the lane (tunnel approach + tunnel).
NO_SHOULDER_IDX = tuple(int(v) for v in os.environ.get("STUDENT_NO_SHOULDER_IDX", "3650-4060").split("-"))
# How long to wait for a person who does not clear the way, then drive around.
PEDESTRIAN_STOP_GAP_M = float(os.environ.get("STUDENT_PEDESTRIAN_STOP_GAP_M", "4"))
PEDESTRIAN_WAIT_S = max(3., float(os.environ.get("STUDENT_PEDESTRIAN_WAIT_S", "4")))   # 433: 15 s + flicker = stuck 90 s; 497: 10 s re-armed each creep -> 4 s
STANDER_S = 3.   # displacement-confirmed standing before the lattice owns the person
PEDESTRIAN_PASS_KMH = float(os.environ.get("STUDENT_PEDESTRIAN_PASS_KMH", "10"))
# Release commit (user 2026-10-11: "stop for them, then go once - no stop-go"). A
# person the guard stopped us for is released once they no longer force a stop
# (out of our path, beside us, or predicted clear before we arrive). For
# PED_RELEASE_S the guard and the live checker then judge them at their measured
# pace, so only a nominal re-entry stops us again (follow-20261010T162527Z
# 01:27:23-34: 5 go/stop cycles over 11 s for one crossing).
PED_RELEASE_S = 6.
PED_WAITING_FORGET_S = 10.
# A person we are stopping for, last seen in our corridor within PED_LOST_WATCH_M of
# the front bumper, who then drops out of perception (fusion loses people at close
# range; single-frame misses released a crossing in replays) still holds the stop
# until raw LiDAR shows the front corridor up to where they stood clear for
# PED_LOST_CLEAR_S. Without a scan the hold lasts PED_LOST_BLIND_S from the last
# sighting (the stale-LiDAR stop owns the rest). A person seen again within
# PED_LOST_REID_M of that spot under another id is judged as seen.
PED_LOST_WATCH_M = 6.
PED_LOST_CLEAR_S = 1.
PED_LOST_BLIND_S = 1.
PED_LOST_REID_M = 1.2
FRONT_BUMPER_M = 2.3175   # ego reference to front bumper
# Moving pedestrian within WALKER_PASS_SIDE_M of our path and WALKER_PASS_AHEAD_M ahead:
# pass at WALKER_PASS_KMH (no stop, no path change).
WALKER_PASS_KMH = float(os.environ.get("STUDENT_WALKER_PASS_KMH", "20"))
WALKER_PASS_SIDE_M = 3.0
WALKER_PASS_AHEAD_M = 25.
WALKER_PASS_MIN_MPS = .4
PASS_CLEAR_M = 1.9            # person centre this far from our path: body .946 + person .3 + .65 m room
PEDESTRIAN_PASS_BY_KMH = 15.  # passing a standing person beside the path (at PASS_CLEAR_M)
# More room, more speed (user 2026-10-05: too cautious with plenty of space): +10 km/h
# per metre of path clearance beyond PASS_CLEAR_M, up to 30 km/h at 3.4 m.
PEDESTRIAN_PASS_BY_KMH_PER_M = 10.
PEDESTRIAN_PASS_BY_MAX_KMH = 30.
# Lead-vehicle following on the normal road (lead_follow.py; live run 264 rear-end at
# idx 400). "0" disables. LiDAR clusters count outside the obstacle sections and the
# surveyed junction regions (there the lattice / detector + gap planner own them).
LEAD_FOLLOW = os.environ.get("STUDENT_LEAD_FOLLOW", "1") != "0"
# A lead standing still in front of us is waited for. Queue at the first signal junction
# (idx ~400, stop line 10 m ahead of the lead) stood 56-60+ s in runs 166/244/254, so a
# lead within LEAD_QUEUE_ZONE_M before a junction stop line gets LEAD_QUEUE_WAIT_S; a car
# standing anywhere else (broken down, parked in the lane) LEAD_STALLED_WAIT_S. After
# that the lattice gets the LiDAR returns and plans around it (same-direction lanes
# first, then the usual stall / lane-escape steps).
LEAD_QUEUE_ZONE_M = float(os.environ.get("STUDENT_LEAD_QUEUE_ZONE_M", "100"))
LEAD_QUEUE_WAIT_S = float(os.environ.get("STUDENT_LEAD_QUEUE_WAIT_S", "120"))
LEAD_STALLED_WAIT_S = float(os.environ.get("STUDENT_LEAD_STALLED_WAIT_S", "20"))


def is_dynamic_obstacle(obj):
    """People, and small things that move on their own (e.g. animals).

    Vehicles ("npc") are excluded, and so is anything longer than 2 m: a car
    mislabelled as an obstacle whose position jitters must not slow us here.
    """
    if obj[4] == "pedestrian":
        return True
    size = max(float(obj[2]), float(obj[6]) if len(obj) > 6 else 0., float(obj[7]) if len(obj) > 7 else 0.)
    # 1 m/s: a static drum's fused position jittered at 0.6-0.9 m/s and held
    # the car at 2 km/h in the box section (live 2026-10-04 00:08).
    return obj[4] == "obstacle" and float(obj[5]) >= 1. and size <= 2.


def dynamic_caution_mps(objects, ego, ahead_m=40., side_m=6., decel=2.5, standoff=4.):
    """Speed from which we can stop `standoff` short of a person/animal near the road."""
    cap = None
    for obj in objects:
        if not is_dynamic_obstacle(obj):
            continue
        forward, lateral = local_xy(obj[0], obj[1], ego)
        # Only ahead of the front bumper: a person we are passing or have
        # passed stays in the side cameras and kept us crawling (live
        # 2026-10-03). Path intrusion from the side still stops us through
        # moving_conflict.
        if not (2.3175 + .5 < forward < ahead_m and abs(lateral) < side_m):
            continue
        reach = math.sqrt(2. * decel * max(0., forward - 2.3175 - standoff))
        value = min(DYNAMIC_CAUTION_MAX_KMH / 3.6, max(reach, 2. / 3.6))
        toward = 0.
        if len(obj) > 10:
            h = math.radians(float(ego["heading_deg"]))
            toward = -(-math.sin(h) * float(obj[9]) + math.cos(h) * float(obj[10])) * math.copysign(1., lateral)
        if (obj[4] == "pedestrian" and abs(lateral) >= PASS_CLEAR_M and float(obj[5]) >= WALKER_PASS_MIN_MPS
                and abs(lateral) < WALKER_PASS_SIDE_M and forward < WALKER_PASS_AHEAD_M):
            # A person walking just beside the path: pass without stopping or swerving,
            # but slow enough to stop if they step in (auto run 243: a walker 2.1 m
            # beside turned into the lane; we braked from 33 km/h at 6 m and hit them).
            value = WALKER_PASS_KMH / 3.6
            cap = value if cap is None else min(cap, value)
            continue
        if obj[4] == "pedestrian" and abs(lateral) >= PASS_CLEAR_M and (float(obj[5]) < .5 or toward < .3):
            # Standing beside the road: a person 3.7 m off the lane held us at
            # 2 km/h for 20 s (live 2026-10-03). Walking along it too: one 2.9 m
            # beside the path kept ~6 m ahead and held us at 2 km/h for 35 s
            # (auto run 186); 2.1 m beside, 2 km/h behind a shoulder walker (run 206):
            # same PASS_CLEAR_M band as the pass-by rule. Entering the lane is handled by moving_conflict /
            # on_path, which still stop.
            continue
        cap = value if cap is None else min(cap, value)
    return cap


# Guardrail S-curve before the junction (route idx 1270-1720, user 2026-10-05: "55
# there"): the follower's curve speed comes from curve_alat_max (ladder 4.2 m/s^2 ->
# 42 km/h at R32). Raise it inside this section only; restored on leaving.
CURVE_ALAT_ZONE = (1270, 1720)
CURVE_ALAT_ZONE_MPS2 = float(os.environ.get("STUDENT_ZONE_CURVE_ALAT", "5.0"))

# Same S-curve: offline minimum-curvature racing line through both same-direction
# lanes (racing_line.json, user 2026-10-05: "use the 2nd lane, never the centre
# line / guardrail side"). Offsets from the control route; never left of our lane
# centre (LiDAR: guardrail face 1.5-2.0 m left of it; 9 of 18 runs 180-197 stopped
# at idx 1490-1515 when the car drifted 0.4 m toward it). Used only while no
# vehicle/object/person is near and the LiDAR corridor of the line is clear;
# otherwise the lattice drives the lane centre as before. "off" disables.
RACING_LINE_FILE = os.environ.get("STUDENT_RACING_LINE", str(Path(__file__).resolve().parent / "racing_line.json"))
RACING_AHEAD_M = float(os.environ.get("STUDENT_RACING_AHEAD_M", "90"))
RACING_BEHIND_M = 30.             # a car closing in the right lane from behind
RACING_BAND_M = (-7.5, 2.0)       # route lateral of our two lanes (+ margin); oncoming lanes excluded
RACING_HOLD_S = 2.0               # stay on the lane centre this long after the last conflict
RACING_LIDAR_HALF_M = .946 + .35  # body half width + lattice side margin
RACING_APPROACH_IDX = 120         # route points (~60 m) before the line starts
RACING_MIN_OFFSET_M = .05         # below this the line is the lane centre
# Guardrail line (user 2026-10-05, run 203 touched the guardrail at idx ~1510, 48 km/h,
# +0.65 m drift): whenever the racing line is off in this section, drive the
# guardrail-safe line of racing_line.json instead of the lane centre: body left edge
# >= 1.0 m + the measured MPC drift (scaled to the expected speed) from the LiDAR
# guardrail. "safe_wide" may put the body up to ~1 m into lane 2; with a vehicle in
# lane 2 "safe_lane" stays in lane 1 and the speed is capped where its clearance
# would fall under 0.8 m with the drift. "off" disables (lane centre as before).
GUARD_LINE = os.environ.get("STUDENT_GUARD_LINE", "on") != "off"
GUARD_LANE2_BAND_M = (-5.6, -1.2)    # route lateral of lane 2 (+ vehicle half width margin)
GUARD_LANE2_LIDAR_BAND_M = (-4.6, -1.2)   # LiDAR returns: lane 2 inside its right edge (-5.25)
GUARD_LANE2_LIDAR_AHEAD_M = 35.     # guardrail face ~+2.0 m: a 5 deg heading error moves it to
                                     # -1.2 m only beyond ~37 m (run 227)
GUARD_LANE2_LIDAR_BEHIND_M = 3.5    # alongside only: the route starts at the car, so returns
                                     # further behind cannot be placed in a bend
GUARD_LANE2_BEHIND_M = 25.           # along the route: alongside, behind, or ahead within the
                                     # racing look-ahead (the join back takes 30-40 m)
LINE_JOIN_ALAT_MPS2 = float(os.environ.get("STUDENT_LINE_JOIN_ALAT", "1.2"))   # line changes mid-curve
LINE_JOIN_S = 4.0                    # ... for this long after the switch; then the normal pull-back
GUARD_SIDE = 1                       # the median guardrail is left (+) of the route: the gentle join
                                     # applies toward it only (run 227)
GUARD_CAP_DECEL_MPS2 = 2.0
RACING_LATE_ENTRY_IDX = 1600         # past the right bend the racing line only returns to the
                                     # lane centre: do not switch onto it there (sim: a late
                                     # switch swung the car 0.5 m toward the guardrail)


def load_racing_lines(path, n_waypoints):
    """Per-waypoint arrays of racing_line.json: offsets of the racing line ("racing")
    and the guardrail lines ("safe_wide", "safe_lane"; 0 outside), their speed caps
    ("lane_vcap", "wide_vcap" km/h; 999 = none). None when unavailable."""
    if not path or path == "off" or not os.path.isfile(path):
        return None
    try:
        with open(path) as source:
            payload = json.load(source)
        first = int(payload["first_idx"])
        lines = {}
        for key, name, fill in (("racing", "offsets_m", 0.), ("safe_wide", "safe_wide_m", 0.),
                                ("safe_lane", "safe_lane_m", 0.), ("lane_vcap", "lane_vcap_kmh", 999.),
                                ("wide_vcap", "wide_vcap_kmh", 999.)):
            values = payload.get(name)
            if values is None:
                continue
            values = np.asarray(values, dtype=float)
            if first < 0 or first + len(values) > int(n_waypoints) or not np.all(np.isfinite(values)):
                return None
            array = np.full(int(n_waypoints), fill)
            array[first:first + len(values)] = values
            lines[key] = array
        return lines if "racing" in lines else None
    except Exception as exc:
        print(f"[gui] racing line unavailable: {exc}", flush=True)
        return None


def load_racing_offsets(path, n_waypoints):
    """Per-waypoint lateral offsets of the racing line (0 outside it), or None."""
    lines = load_racing_lines(path, n_waypoints)
    return None if lines is None else lines["racing"]


CONTACT_MEMORY_S = 3.


def body_gap_m(lidar):
    """Smallest distance (m) from the 4.635 x 1.892 m body to a current LiDAR obstacle
    point (0.3-2.2 m above the road): map scenery (drums, walls, kerbs) is never in
    MORAI's collision list, so contacts are found from this (<= 0.1 m). The body
    spans 0.79 m behind to 3.845 m ahead of the rear-axle origin (ego_body)."""
    try:
        pts, _ = lidar.snapshot() if lidar else (None, 99.)
        if pts is None or not len(pts):
            return None
        return round(ego_body_gap(np.asarray(pts, dtype=float)[:, :2]), 2)
    except Exception:
        return None


def reverse_swing_points(points, ego, road_heading_deg, distance_m):
    """Vehicle-frame points (N x 2) the body would reach on a reverse of distance_m that
    turns it to road_heading_deg (gui.publish_reverse / ego_body.reverse_sweep_hits)."""
    if points is None or len(points) == 0 or not math.isfinite(float(distance_m)) or float(distance_m) <= 0.:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)[:, :2]
    error = (float(road_heading_deg) - float(ego["heading_deg"]) + 180.) % 360. - 180.
    return pts[reverse_sweep_hits(pts, math.radians(error), float(distance_m))]


def zone_speed_kmh(speed_zones, route_index):
    if getattr(speed_zones, "station", None) is not None:
        return speed_zones.braking_kmh(route_index, DYNAMIC_ZONES)
    kmh, zone = speed_zones.kmh_at(route_index)
    for a, b, cap in DYNAMIC_ZONES:
        if a <= route_index <= b:
            kmh = min(kmh, cap)
    return kmh, zone
STRICT_STATIC_STOP = os.environ.get("STUDENT_STRICT_STATIC_STOP", "0") == "1"
# Lateral acceleration allowed on a bypass path (m/s^2). Competition launch raises it.
BYPASS_ALAT_MPS2 = float(os.environ.get("STUDENT_BYPASS_ALAT", "1.7"))


def static_speed_cap_mps(cruise_mps, plan, approach_distance=None):
    """Use the validated driving path for speed; approach limits apply until it exists."""
    if plan is not None and not plan.get("blocked"):
        return min(cruise_mps, math.sqrt(BYPASS_ALAT_MPS2 / max(plan["curvature"], 1e-3)),
                   plan["speed_cap_mps"], plan.get("ordinary_speed_cap_mps", float('inf')),
                   plan.get("standing_bypass_cap_mps", float('inf')))
    return min(cruise_mps, static_approach_cap_mps(approach_distance)) if approach_distance is not None else cruise_mps


def model_static_speed_cap_mps(cruise_mps, plan, approach_distance=None, *, speed_mps=None):
    """E2E (STUDENT_PLANNER=model) speed for static obstacles.

    A complete static bypass with measured speed uses a braking envelope to
    each curve. Normal-road plans can reach 30 km/h only with full raw and
    object clearance at the wider margin; otherwise their 25 km/h ceiling
    remains. Partial and standing-person bypasses keep their existing caps.
    None yet:
    only the checked-stop envelope (1.5 m/s^2, 4 m short) to approach_distance, the
    first thing the planner itself must go around or a LiDAR block on the route. No
    8-25 km/h approach cap: it braked the car before the planner engaged and shrank
    the speed-scaled activation distance with it (run 191227Z: 34 -> 16 km/h from
    44 m with no plan tried), and the envelope already guarantees the stop."""
    if plan is not None and not plan.get("blocked"):
        from model_avoidance import static_plan_complete
        if speed_mps is not None and plan.get('static_motion_profile') and static_plan_complete(plan):
            # The checked profile already owns local curve/wheel braking.
            # A second discretization must not become a new future ceiling.
            return min(cruise_mps, plan['speed_cap_mps'])
        if (speed_mps is not None and static_plan_complete(plan)
                and plan.get('generator') == 'road_static_pass'
                and plan.get('static_speed_profile') and not plan.get('static_borrow')
                and not plan.get('standing_bypass_ids')):
            # This complete in-lane pass includes distance-based curvature,
            # wheel-rate and wide-clearance braking. Reapplying its worst
            # future curvature here would recreate the upstream crawl.
            # A nearer width constraint still lowers the command smoothly to
            # the ordinary pass pace, using this frame's current clearance.
            raw_reserve = plan['static_speed_profile'].get('raw_clear_m',
                plan.get('fast_static_clear_m', 0.))
            wide_room = max(0., float(raw_reserve)-
                            max(float(speed_mps), 30./3.6)*.7)
            width_cap = math.sqrt((25./3.6)**2+3.*wide_room)
            return min(cruise_mps, plan['speed_cap_mps'], width_cap)
        if (speed_mps is not None and static_plan_complete(plan)
                and plan.get('path') is not None and not plan.get('standing_bypass_ids')
                and plan.get('fast_static_clear_m', 0.) >= max(
                    35., float(plan['safe_horizon_m']), float(plan['hold_until_m']))):
            # A distant return bend limits speed at that bend, rather than
            # throughout the straight pass. Keep the existing lateral-accel,
            # clear-distance and steering-rate limits; reserve 0.7 s followed
            # by gentle braking to reach each local curvature limit.
            from tunnel_runtime import curve_speed_cap
            curve_cap, _ = curve_speed_cap(RosPathPublisher.resample(plan['path']),
                speed_mps, alat_mps2=BYPASS_ALAT_MPS2, decel_mps2=1.5, reaction_s=.7)
            return min(cruise_mps, curve_cap, plan['speed_cap_mps'])
        return static_speed_cap_mps(cruise_mps, plan)
    if approach_distance is not None:
        return min(cruise_mps, blocked_static_stop_cap_mps(approach_distance))
    return cruise_mps


def recover_static_prefix(previous, ego, objects, corridor_clearance):
    """Continue a checked bypass prefix while a longer replan is unavailable."""
    if previous is None:
        return None
    path = previous["path"]
    speed = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
    required = max(4., speed * .7 + speed * speed / 5. + 2.)
    horizon = 35.
    stationary = [o for o in objects if o[4] == "obstacle" or
                  (o[4] == "pedestrian" and o[5] < .3) or
                  (o[4] == "npc" and o[5] < .5)]
    clear = min(static_clear_distance(path, ego, stationary, horizon),
                corridor_clearance(path, horizon))
    if clear < required:
        return None
    curvature = float(np.max(path_curvatures(path)))
    usable = max(0., clear - 2.)
    cap = math.sqrt((2.5 * .7) ** 2 + 5. * usable) - 2.5 * .7
    return dict(previous, kind="hold", curvature=curvature, speed_cap_mps=cap,
                safe_horizon_m=required, clear_distance_m=clear,
                replan_needed=True, reused=True)


def plan_static_bypass(dense_route, ego, objects, lane_ok, committed_direction=0,
                       previous_plan=None, adjacent_offsets=None, corridor_clearance=None,
                       first_complete=False, full_width=False, deadline=None,
                       body_shift_m=ROAD_LEGACY_SHIFT_M):
    """Keep one lane through a connected obstacle group and validate a braking-safe prefix.
    body_shift_m: body placement in the footprint test (static_clear_distance)."""
    stationary = [o for o in objects if o[4] == "obstacle" or
                  (o[4] == "pedestrian" and o[5] < 0.3) or (o[4] == "npc" and o[5] < 0.5)]
    debug = []
    route_hazard(stationary, ego, dense_route, ahead_m=65.0, half_width_m=1.4,
                 max_speed_mps=1.5, debug=debug)
    hazards = sorted((d for d in debug if d["ok"]), key=lambda d: d["front"])
    speed = max(0.0, float(ego.get("speed_kmh", 0))) / 3.6
    planning_speed = max(speed, 20. / 3.6) if full_width else speed
    tracking_speed = max(planning_speed, 2.5)
    comfortable_curvature = BYPASS_ALAT_MPS2 / tracking_speed ** 2
    # Do not shrink the planning window down to 14 m as approach braking starts.
    activation = max(35.0 if full_width else 25.0, min(45.0, planning_speed * 3.5 + 6.0))
    required = max(4.0, speed * .7 + speed ** 2 / 5.0 + 2.0)
    side_locked = committed_direction in (-1, 1)
    committed = previous_plan is not None and side_locked
    # The route projection drops an object once its centre passes the ego,
    # although the rear bumper is still alongside it. Keep the manoeuvre
    # active until the whole object clears the vehicle's rear footprint.
    if side_locked and abs(float(np.asarray(dense_route)[0, 1])) > 1.0:
        route = np.asarray(dense_route, dtype=float)[:, :2]
        route_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        for obj in stationary:
            forward, lateral = local_xy(obj[0], obj[1], ego)
            extent = min(3., max(float(obj[2]), float(obj[6] if len(obj) > 6 else 0.)) / 2.)
            if not (-2.5 < forward + extent and forward < 4. and 1.8 < abs(lateral) < 8.):
                continue
            index = int(np.argmin(np.sum((route - [forward, lateral]) ** 2, axis=1)))
            if route_s[index] > 8. or np.linalg.norm(route[index] - [forward, lateral]) > 2.0:
                continue
            if any(h["n"] == obj[3] for h in hazards):
                continue
            hazards.append({"n": obj[3], "front": max(0., forward - extent),
                            "along": max(0., forward), "back": max(0., forward + extent),
                            "f": forward, "lat": lateral, "ok": True})
        hazards.sort(key=lambda d: d["front"])
    transition_hint = max(10., min(24., planning_speed * 3.0),
                          float(previous_plan.get("transition_m", 0.)) if committed else 0.)
    # Returning and leaving again both need a complete lateral transition plus
    # room for the vehicle. This is a geometric decision, not a timer or route ID.
    rejoin_gap = 2.0 * transition_hint + 5.0
    engagement = max(activation, required, rejoin_gap if committed else 0.)
    active = bool(hazards and hazards[0]["front"] <= engagement)
    if not active and not committed:
        return None, []
    group = []
    if active:
        for hazard in hazards:
            back = hazard.get("back", 2.0 * hazard["along"] - hazard["front"])
            if group and hazard["front"] - group[-1]["back"] > rejoin_gap:
                break
            if back + 3.0 > 57.0:
                break
            group.append(dict(hazard, back=back))
    hold_until = max([0.] + [d["back"] + 3.0 for d in group])
    horizon = min(60., max(35., hold_until + transition_hint))
    route_array = np.asarray(dense_route, dtype=float)[:, :2]
    tangent = route_array[min(4, len(route_array)-1)] - route_array[0]
    tangent /= max(float(np.linalg.norm(tangent)), 1e-9)
    # Offsets are measured from the route, not from the current vehicle pose.
    # In a tunnel, choosing .45 m when ego is already 1.2 m to that side is
    # an early return toward the obstacle, despite being called a bypass.
    ego_route_lateral = float(np.dot(-route_array[0], [-tangent[1], tangent[0]]))
    candidates = []
    rejected = {"curvature": 0, "clearance": 0, "lane": 0}
    budget_hit, evaluated = False, 0

    def budget_exhausted():
        nonlocal budget_hit
        if deadline is not None:
            current = time.perf_counter()
            # A first safe path can require reaching a wider candidate. Allow
            # a bounded extra 60 ms while none exists; after finding one, spend
            # no more time improving it beyond the normal search budget.
            if current >= deadline and (candidates or current >= deadline+.06):
                budget_hit = True
        return budget_hit

    def finish(plan):
        if deadline is None:
            return plan
        return dict(plan, planning_budget_exhausted=budget_hit, candidates_evaluated=evaluated)

    def consider(plan, lane_tol):
        nonlocal evaluated
        evaluated += 1
        path = plan["path"]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
        if arc[-1] < required or plan["curvature"] > MAX_CONTROL_CURVATURE:
            rejected["curvature"] += 1
            return False
        # Geometric feasibility also needs the controller's signed steering
        # rate. Missing measurements are unknown, not a zero road-wheel angle.
        # Use physical arc spacing rather than the source vertex spacing. HD
        # links may add 9 mm endpoint gaps, and dense scans can sample at 1 cm;
        # neither should change the actuator limit for the same geometric curve.
        rate_s = np.linspace(0., arc[-1], max(3, int(math.ceil(arc[-1] / .5)) + 1))
        xy = np.column_stack([np.interp(rate_s, arc, path[:, i]) for i in (0, 1)])
        edge, following = xy[1:-1]-xy[:-2], xy[2:]-xy[1:-1]
        chord = xy[2:]-xy[:-2]
        denominator = (np.linalg.norm(edge, axis=1) * np.linalg.norm(following, axis=1)
                       * np.linalg.norm(chord, axis=1))
        signed_curvature = np.divide(2. * (edge[:, 0]*chord[:, 1] - edge[:, 1]*chord[:, 0]),
            denominator, out=np.zeros_like(denominator), where=denominator >= 1e-8)
        steering = np.arctan(3.90 * signed_curvature)
        steering_s = rate_s[1:-1]
        initial_angle = ego.get("wheel_angle")
        if initial_angle is not None and math.isfinite(float(initial_angle)):
            steering = np.r_[math.radians(float(initial_angle)), steering]
            steering_s = np.r_[0., steering_s]
        interval = np.diff(steering_s)
        gradient = np.divide(np.diff(steering), interval,
            out=np.zeros_like(interval), where=interval >= 1e-8)
        peak_gradient = float(np.max(np.abs(gradient))) if len(gradient) else 0.
        # Reserve 20% in the target speed for actuator lag and feedback corrections.
        physical_rate = math.radians(30.)
        steering_rate_cap = physical_rate * .8 / max(peak_gradient, 1e-6)
        segment_cap = physical_rate / np.maximum(np.abs(gradient), 1e-6)
        # Test current-speed reachability against the physical limit, not the
        # target reserve. Braking cannot instantly attain a newly commanded cap.
        segment_s = steering_s[1:]
        minimum_speed = np.sqrt(np.maximum(0., speed ** 2 - 5. * np.maximum(
            0., segment_s - speed * .2)))
        if np.any((segment_s <= 5.) & (minimum_speed > segment_cap + 1e-6)):
            rejected["curvature"] += 1
            return False
        clear = static_clear_distance(path, ego, stationary, horizon, body_shift_m=body_shift_m)
        # An uncommitted manoeuvre must pass the complete obstacle group.
        # Reject a known collision before invoking the costlier raw-LiDAR and
        # lane checks; such a partial path cannot be selected below anyway.
        minimum_clear = max(required, hold_until) if active and previous_plan is None else required
        if clear < minimum_clear:
            rejected["clearance"] += 1
            return False
        if corridor_clearance is not None:
            clear = min(clear, corridor_clearance(path, horizon))
        if clear < minimum_clear:
            rejected["clearance"] += 1
            return False
        end = min(clear, max(required, hold_until))
        prefix = path[:min(len(path), int(np.searchsorted(arc, end)) + 1)]
        if not lane_ok(prefix, lane_tol):
            rejected["lane"] += 1
            return False
        usable = max(0., clear - 2.)
        cap = min(math.sqrt((2.5 * .7) ** 2 + 5.0 * usable) - 2.5 * .7,
                  steering_rate_cap)
        complete = clear >= max(required, hold_until)
        plan = dict(plan, safe_horizon_m=required, clear_distance_m=clear,
                    speed_cap_mps=cap, activation_m=activation, manoeuvre_horizon_m=hold_until,
                    steering_rate_cap_mps=steering_rate_cap, steering_gradient_radpm=peak_gradient,
                    cluster_count=len(group), hold_until_m=hold_until,
                    replan_needed=clear < horizon - .5, reused=bool(plan.get("reused")))
        continuity = 0.
        if previous_plan is not None and not plan["reused"]:
            old = previous_plan["path"][:, :2]
            old_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(old, axis=0), axis=1))]
            sample = np.linspace(0., min(12., old_s[-1], arc[-1]), 24)
            current = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
            previous = np.column_stack([np.interp(sample, old_s, old[:, i]) for i in (0, 1)])
            continuity = float(np.mean(np.linalg.norm(current - previous, axis=1)))
        score = ((4. + .1 * speed ** 2) * plan["curvature"] + 2. * continuity
                 + .08 * (horizon - clear) + .15 * abs(plan.get("offset_m", 0.))
                 + (0. if plan["reused"] else .15))
        # A path that reaches only the braking-safe prefix still approaches a
        # known obstruction. Prefer a clear replan over reusing that path, even
        # when the new path differs from the previous steering trajectory.
        reachable_cap = min(cap, math.sqrt(BYPASS_ALAT_MPS2 / max(plan["curvature"], 1e-3)))
        candidates.append(((not complete, plan["replan_needed"],
                            max(0., tracking_speed - reachable_cap), score), plan))
        return complete

    if previous_plan is not None and previous_plan["direction"] == committed_direction:
        old = dict(previous_plan, curvature=float(np.max(path_curvatures(previous_plan["path"]))))
        # Reuse only if the saved path still stays out through the next group.
        # A safe short prefix alone must not endorse its old, premature return.
        if not active:
            old["kind"] = "rejoin"
        premature_return = active and old.get("kind") == "rejoin"
        if active and not premature_return:
            route = np.asarray(dense_route)[:, :2]
            route_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
            tangent = np.gradient(route, axis=0)
            tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
            normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
            _, at = cKDTree(route).query(old["path"][:, :2])
            lateral = committed_direction * np.sum((old["path"][:, :2] - route[at]) * normal[at], axis=1)
            section = lateral[route_s[at] <= hold_until]
            if len(section):
                peak = int(np.argmax(section))
                premature_return = bool(section[peak] > .4 and
                                        np.min(section[peak:]) < .75 * section[peak])
        if not premature_return and consider(old, 1.9):
            checked_old = candidates[-1][1]
            # A sharp carried join may be safe only at crawling speed. Keep
            # it as a fallback while checking a fresh, gentler continuation.
            if (not checked_old["replan_needed"] and checked_old["curvature"] <= comfortable_curvature
                    and checked_old["speed_cap_mps"] >= tracking_speed):
                return finish(checked_old), hazards
    if not active and committed:
        # Finishing a bypass is also a lateral manoeuvre. The builder's short
        # default transition needlessly made an empty-road return abrupt.
        for transition in (26., 22., 18., 14., 12., 10., 8., 6.):
            if budget_exhausted():
                break
            path, curvature, _ = build_forced_bypass(dense_route, 0, offset_m=0, hold_m=0,
                max_curvature_1pm=MAX_CONTROL_CURVATURE, transitions=(transition,))
            if path is not None:
                complete = consider(dict(path=path, direction=committed_direction, offset_m=0., hold_m=0.,
                                         curvature=curvature, transition_m=transition, kind="rejoin"), 1.9)
                if (first_complete and complete and not candidates[-1][1]["replan_needed"]
                        and curvature <= comfortable_curvature
                        and candidates[-1][1]["speed_cap_mps"] >= tracking_speed):
                    return finish(candidates[-1][1]), hazards
    else:
        sides = (float(committed_direction), -float(committed_direction)) if side_locked else (1., -1.)
        trials = []
        for side in sides:
            if side_locked and side != float(committed_direction) and speed > 1.5:
                continue
            lane_offset = (adjacent_offsets or {}).get(int(side))
            # Small edge adjustments precede full lane changes. Without a
            # committed side, check both directions at each width before
            # approving a wider manoeuvre in the first direction.
            choices = ([(.45, False), (.6, False), (.9, False), (1.2, False)]
                       if adjacent_offsets is not None else
                       [(1.0, False), (1.6, False), (2.2, False)])
            # A permitted neighbouring lane also permits checked intermediate
            # widths; a narrow centre obstacle need not cause a full lane change.
            if lane_offset and adjacent_offsets is not None:
                choices += [(offset, False) for offset in (1.6, 2.2, 2.8)
                            if offset < abs(lane_offset)-.05]
                if full_width:
                    # Tunnels permit all measured space between their walls,
                    # including widths between ordinary lane-change candidates.
                    choices += [(float(offset), False) for offset in
                                np.arange(3.4, abs(lane_offset)-.05, .6)]
            choices += [(lane_offset, True)] if lane_offset else []
            if full_width and active and side * ego_route_lateral > .4:
                minimum_offset = side * ego_route_lateral - .05
                choices = [(offset, adjacent) for offset, adjacent in choices
                           if abs(offset) >= minimum_offset]
                if lane_offset and side * ego_route_lateral < abs(lane_offset):
                    choices.append((side * ego_route_lateral, False))
                choices.sort(key=lambda choice: abs(choice[0]))
            trials.extend((side, offset, adjacent) for offset, adjacent in choices)
        if not side_locked:
            trials.sort(key=lambda trial: abs(trial[1]))
        transitions = (6., 8., 10., 12., 14., 18., 22.) if speed <= 2.5 else (10., 12., 14., 18., 22., 26.)
        if full_width:
            # Plan for a rolling 20 km/h bypass even after a prior slowdown.
            # Keep short options as checked fallbacks; never raise a physical
            # curvature/steering/clearance speed cap to manufacture this target.
            transitions = (6., 8., 10., 12., 14., 18., 22., 26., 30., 34.)
        if first_complete:
            transitions = tuple(reversed(transitions))
        for side, offset, adjacent in trials:
            if budget_exhausted():
                break
            if side_locked and side != float(committed_direction) and candidates:
                break
            for transition in transitions:
                if budget_exhausted():
                    break
                hold = max(6., hold_until - transition)
                path, curvature, _ = build_forced_bypass(dense_route, side, offset_m=offset,
                    hold_m=hold, max_curvature_1pm=MAX_CONTROL_CURVATURE, transitions=(transition,))
                if path is not None:
                    complete = consider(dict(path=path, direction=side, offset_m=offset, hold_m=hold,
                                  curvature=curvature, transition_m=transition, kind="bypass",
                                  adjacent_lane=adjacent), 1.5 if adjacent else 1.9)
                    # A narrow curve must also be gentle enough for the current
                    # speed before the early-exit optimization can accept it.
                    if (first_complete and complete and not candidates[-1][1]["replan_needed"]
                            and curvature <= comfortable_curvature
                            and candidates[-1][1]["speed_cap_mps"] >= tracking_speed):
                        return finish(candidates[-1][1]), hazards
    # A braking-safe prefix is useful only after a bypass is already under way.
    # Before commitment it can end beside the obstacle, and latching its side
    # prevents a full adjacent-lane route found a few metres later from being used.
    if not candidates or (active and previous_plan is None and
                          not any(not rank[0] for rank, _ in candidates)):
        return finish({"blocked": True, "kind": "bypass" if active else "rejoin",
                "activation_m": activation, "safe_horizon_m": required, "rejected": rejected}), hazards
    return finish(min(candidates, key=lambda pair: pair[0])[1]), hazards


def moving_conflict(objects, ego, path_local, ego_mps, horizon_s=3.0, lat_m=1.6, body_shift_m=ROAD_LEGACY_SHIFT_M):
    """Traffic rule for MOVING objects and pedestrians, judged on the path we will drive. Returns None or a dict:
      kind="stop"   : pedestrian on the path ahead / crossing vehicle that reaches our path within ~1.5 s / stopped leader < 8 m
      kind="slow"   : crossing vehicle that reaches our path in 1.5-3 s (target 6 km/h until it clears)
      kind="follow" : vehicle ahead in our direction inside 15 m -> cap our speed to its speed (v_mps)
    Oncoming vehicles (heading opposite to our path) are ignored unless they are actually inside our lane (|lat| <= 0.9 m).
    body_shift_m: the body's front / rear bumpers lie EGO_HALF_LENGTH_M +/- body_shift_m ahead of / behind
    the pose (ego_body: the tunnel passes REAR_AXLE_SHIFT_M, the road keeps its legacy +-2.3175 m)."""
    front_m, rear_m = EGO_HALF_LENGTH_M + body_shift_m, EGO_HALF_LENGTH_M - body_shift_m
    route = np.asarray(path_local, dtype=np.float32)[:, :2]
    if len(route) < 3 or not objects:
        return None
    tang = np.gradient(route, axis=0); tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-6)
    normal = np.column_stack((-tang[:, 1], tang[:, 0]))
    s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))))
    yaw = math.radians(float(ego["heading_deg"]))
    best = None
    def better(cand):
        order = {"stop": 0, "slow": 1, "follow": 2}
        return best is None or (order[cand["kind"]], cand["along"]) < (order[best["kind"]], best["along"])
    for x, y, size, name, kind, speed, *extra in objects:
        vx, vy = (float(extra[3]), float(extra[4])) if len(extra) >= 5 else (0.0, 0.0)
        hd = (vehicle_motion_heading((x, y, size, name, kind, speed, *extra))
              if len(extra) >= 3 else None)
        half = 0.5 * float(size)
        f, l = local_xy(x, y, ego)
        p = np.array([f, l], dtype=np.float32); k = int(np.argmin(((route - p) ** 2).sum(axis=1)))
        along, lat = float(s[k]), float(((p - route[k]) * normal[k]).sum())
        if kind == "pedestrian":
            # Include reaction time and braking distance; predict crossing before
            # the pedestrian enters the vehicle's current lateral corridor.
            radius = max(0.6, half)
            stopping = ego_mps * 0.7 + ego_mps ** 2 / (2.0 * 2.5) + 3.0
            if speed < .3 and abs(vx) + abs(vy) < .3:
                # Standing still: brake by distance, not by "arrival vs crossing time".
                # The timing rule ordered a stop 31 m short at 25 km/h, released it
                # once stopped (arrival = along / 1 m/s), and repeated: stop-go all
                # the way up to a person standing in our lane (auto run 145).
                along0 = along + float(np.dot(p - route[k], tang[k]))
                # Lateral band from the body, not the 0.6 m radius: the bypass keeps
                # 0.946 + 0.35 + (0.3 + 0.1) = 1.7 m from a standing person, and a 1.9 m
                # band called that bypass "on the path" and stopped it (sim of run 145).
                if (-rear_m - radius < along0 and abs(lat) <= 0.946 + half + 0.35
                        and not (along0 < front_m and abs(lat) > 0.946 + half + .1)):
                    if along0 <= stopping - 1.5:
                        cand = {"kind": "stop", "name": name, "along": along0, "t": 0., "v_mps": 0.}
                    else:
                        cap = math.sqrt(max(0., 2. * 2.0 * (along0 - front_m - 2.5)))
                        cand = {"kind": "follow", "name": name, "along": along0, "t": 0., "v_mps": cap}
                    if better(cand): best = cand
                continue
            for t in np.arange(0.0, max(horizon_s, ego_mps / 2.5 + 0.7) + 0.01, 0.15):
                f_t, l_t = local_xy(x + vx * t, y + vy * t, ego)
                point = np.array([f_t, l_t])
                j = int(np.argmin(((route - point) ** 2).sum(axis=1)))
                along_t = float(s[j])
                lat_t = float(np.dot(point - route[j], normal[j]))
                # Include longitudinal projection, since argmin clamps objects
                # behind the route to index zero.
                along_t += float(np.dot(point - route[j], tang[j]))
                # A side camera can still see a person after the car passes.
                # Compare both positions at the predicted instant; a crossing
                # behind the rear bumper is not a collision with our path.
                rear = rear_m + radius + 0.35
                if along_t - ego_mps * t < -rear or abs(lat_t) > 0.946 + radius + 0.35:
                    continue
                if speed < .3 and along_t < front_m and abs(lat_t) > 0.946 + half + .1:
                    # Standing beside or behind the body: driving forward cannot
                    # reach them; stopping here only parks us (live 2026-10-03).
                    continue
                arrival = along_t / max(ego_mps, 1.0)
                if along_t <= stopping or abs(arrival - t) <= 1.5:
                    cand = {"kind": "stop", "name": name, "along": along_t,
                            "t": float(t), "v_mps": float(speed)}
                    if better(cand): best = cand
                    break
                if along_t <= stopping + 8.0:
                    cand = {"kind": "slow", "name": name, "along": along_t,
                            "t": float(t), "v_mps": float(speed)}
                    if better(cand): best = cand
            continue
        if speed <= 1.0:
            continue
        rel = None
        if hd is not None:
            d = hd - (yaw + math.atan2(float(tang[k, 1]), float(tang[k, 0])))
            d = (d + math.pi) % (2 * math.pi) - math.pi
            rel = "same" if abs(d) < math.radians(45) else "oncoming" if abs(d) > math.radians(135) else "cross"
        if rel == "same":
            if 1.0 <= along <= 15.0 and abs(lat) - min(half, 1.2) <= 1.2:
                if speed < 0.5 and along <= 8.0:
                    cand = {"kind": "stop", "name": name, "along": along, "t": 0.0, "v_mps": float(speed)}
                else:
                    cand = {"kind": "follow", "name": name, "along": along, "t": 0.0, "v_mps": float(speed)}
                if better(cand): best = cand
            continue
        if rel == "oncoming" and abs(lat) - min(half, 1.2) > 0.9:
            continue   # opposite lane traffic
        for t in np.arange(0.0, horizon_s + 1e-6, 0.25):
            f, l = local_xy(x + vx * t, y + vy * t, ego)
            p = np.array([f, l], dtype=np.float32); k = int(np.argmin(((route - p) ** 2).sum(axis=1)))
            along_t, lat_t = float(s[k]), float(((p - route[k]) * normal[k]).sum())
            if abs(lat_t) - min(half, 1.2) > lat_m:
                continue
            s_ego = ego_mps * t + 2.0   # our front bumper at time t
            if -2.0 <= along_t - s_ego <= 6.0 and along_t >= 0.5:
                cand = {"kind": "stop" if (t <= 1.5 or along_t <= 6.0) else "slow", "name": name, "along": along_t, "t": float(t), "v_mps": float(speed)}
                if better(cand): best = cand
                break
    return best


def crossing_caution(objects, ego, followable_ids=()):
    """Fallback for approaching cross traffic; leave classified merge leaders to GapPlanner."""
    yaw_e = math.radians(float(ego["heading_deg"]))
    for o in objects:
        if o[4] != "npc" or float(o[5]) < 1.0 or len(o) < 12:
            continue
        if int(o[11]) in followable_ids:
            continue
        f, l = local_xy(o[0], o[1], ego)
        dist = math.hypot(f, l)
        if dist > 20.0 or f < -2.0:
            continue
        motion_heading = vehicle_motion_heading(o)
        if motion_heading is None:
            continue
        d_hd = (motion_heading - yaw_e + math.pi) % (2 * math.pi) - math.pi
        if abs(d_hd) < math.radians(30) or abs(d_hd) > math.radians(150):
            continue
        vx, vy = float(o[9]), float(o[10])
        closing = (vx * (ego["x"] - o[0]) + vy * (ego["y"] - o[1])) / max(dist, 1e-3)
        if closing <= 0.5:
            continue
        near_hit = dist <= 10.0
        return {"kind": "stop" if near_hit else "follow", "name": f"crossing_caution:{o[3]}",
                "along": f, "t": dist / max(float(o[5]), 0.1),
                "v_mps": 0.0 if near_hit else 8.0 / 3.6, "source": "crossing_caution",
                "reason": "crossing vehicle closing within 10 m" if near_hit else "crossing vehicle closing within 20 m"}
    return None


class HdMapLanes:
    """All HD-map link centrelines (MORAI link_set.json) for the top view: every lane, not just the global route."""
    def __init__(self, path):
        self.links = []
        self.stop_line_nodes = {}
        self.stop_lines = []
        self.geometry = {}
        self.successors = {}
        self.predecessors = {}
        self.points = np.zeros((0, 2), np.float32); self.link_of = np.zeros(0, np.int32); self.n_links = 0
        # Operator rules from the dashboard's 주행 가능 영역 page (reloaded every 2 s):
        # {"mode": "whitelist", "allowed_links": [ids], "zones": [{"polygon": [[x, y], ...]}], "sections": [...]}
        # whitelist: bypasses may use only allowed_links besides the route lane. Without "mode",
        # legacy per-link overrides {"links": {id: {"block", "left", "right"}}} apply.
        rules_path = os.environ.get("STUDENT_DRIVABLE_RULES")
        self.rules_cache = AsyncJsonCache(rules_path, interval=2.) if rules_path else None
        try:
            with open(path) as source:
                links = json.load(source)
            self.links = links
            for link in links:
                self.successors.setdefault(link.get("from_node_idx"), []).append(link.get("idx"))
                self.predecessors.setdefault(link.get("to_node_idx"), []).append(link.get("idx"))
            pts, ids = [], []
            for i, link in enumerate(links):
                p = link.get("points")
                if isinstance(p, str):
                    import ast; p = ast.literal_eval(p)
                p = np.asarray(p, dtype=np.float32)[:, :2]
                if len(p) < 2:
                    continue
                self.geometry[link.get("idx")] = p
                pts.append(p); ids.append(np.full(len(p), i, np.int32))
            if pts:
                self.points, self.link_of, self.n_links = np.vstack(pts), np.concatenate(ids), len(links)
                self.tree = cKDTree(self.points)
                self.link_by_id = {link["idx"]: link for link in links}
                self.link_trees = {key: cKDTree(points) for key, points in self.geometry.items()}
                self._load_stop_line_nodes(Path(path))
        except Exception as exc:
            print(f"[gui] HD-map lanes unavailable: {exc}", flush=True)

    def _load_stop_line_nodes(self, link_path):
        """Read surveyed stop nodes from the same map or its packaged archive."""
        import zipfile
        candidates = [link_path.parent/"node_set.json", Path("/hdmap/node_set.json"),
                      Path(__file__).resolve().parents[1]/"ws/src/moraimpc/data/hdmap.zip",
                      Path("/opt/autodrive/ws/src/moraimpc/data/hdmap.zip")]
        configured = os.environ.get("STUDENT_NODE_SET") or os.environ.get("STUDENT_HDMAP_ZIP")
        if configured:
            candidates.insert(0, Path(configured))
        known_nodes = {link.get("from_node_idx") for link in self.links}
        for candidate in candidates:
            if not candidate.is_file():
                continue
            try:
                if candidate.suffix == ".zip":
                    with zipfile.ZipFile(candidate) as archive:
                        nodes = json.loads(archive.read("node_set.json"))
                else:
                    nodes = json.loads(candidate.read_text())
                matched = {str(node["idx"]): node for node in nodes
                           if node.get("on_stop_line") and node.get("idx") in known_nodes
                           and len(node.get("point", ())) >= 2}
                if matched:
                    self.stop_line_nodes = matched
                    self.stop_line_map_source = str(candidate)
                    return
            except (OSError, ValueError, KeyError, zipfile.BadZipFile) as exc:
                print(f"[gui] Stop-line map unavailable at {candidate}: {exc}", flush=True)

    def rules(self):
        value = self.rules_cache.value if self.rules_cache is not None else None
        return value if isinstance(value, dict) else {}

    def link_rule(self, link_id):
        rule = (self.rules().get("links") or {}).get(link_id)
        return rule if isinstance(rule, dict) else {}

    def blocked(self, link_id):
        rules = self.rules()
        if rules.get("mode") == "whitelist":
            # Operator-drawn drivable area: only the lanes chosen on the dashboard (plus the route lane).
            return link_id not in set(rules.get("allowed_links") or ())
        return bool(self.link_rule(link_id).get("block"))

    def set_route(self, route_xy):
        """Global route, for mapping corridor points to route distance (mission sections)."""
        route = np.asarray(route_xy, dtype=float)[:, :2]
        self.route_xy = route.copy()
        self.route_station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        self.route_tree = cKDTree(route)
        # HD-map intersections: route points on junction links (link_type "1", the links that carry
        # traffic-light relations). No lanes to move into there, so they get open-space bypasses.
        self.route_junction = np.zeros(len(route), dtype=bool)
        if len(self.points):
            distance, index = self.tree.query(route)
            kinds = np.array([str(self.links[int(k)].get("link_type")) == "1" for k in self.link_of[index]])
            self.route_junction = kinds & (distance <= 1.0)
        # Select surveyed nodes only at the route's entrance into a junction.
        # Some junction exits and tunnel gates also carry on_stop_line=True.
        entries = np.flatnonzero(self.route_junction[1:] & ~self.route_junction[:-1]) + 1
        self.stop_lines = []
        for entry in entries:
            link = self.links[int(self.link_of[index[entry]])]
            node = self.stop_line_nodes.get(str(link.get("from_node_idx")))
            if node is None:
                continue
            xy = np.asarray(node["point"][:2], dtype=float)
            lo, hi = max(0, int(entry)-12), min(len(route)-1, int(entry)+12)
            segments = np.arange(lo, hi)
            station, distance, segment = self._project_route(xy, segments)
            if distance > 1.5:
                continue
            route_index = int(segment + (np.linalg.norm(xy-route[segment+1]) <
                                          np.linalg.norm(xy-route[segment])))
            self.stop_lines.append(dict(id=f"{node['idx']}:{route_index}", node_id=str(node["idx"]),
                xy=xy.tolist(), station_m=station, route_idx=route_index,
                source="hdmap_node", map_source=self.stop_line_map_source,
                entry_link_id=link["idx"], traffic_light_id=node.get("traffic_light_id")))
        self.stop_lines.sort(key=lambda line: line["station_m"])
        self.stop_line_stations = np.asarray([line["station_m"] for line in self.stop_lines])

    def _project_route(self, xy, segments, heading_deg=None):
        """Return continuous station, distance and selected route segment."""
        starts = self.route_xy[segments]
        delta = self.route_xy[segments+1]-starts
        length_sq = np.maximum(1e-12, np.sum(delta*delta, axis=1))
        fraction = np.clip(np.sum((xy-starts)*delta, axis=1)/length_sq, 0., 1.)
        projected = starts+fraction[:, None]*delta
        distance_sq = np.sum((projected-xy)**2, axis=1)
        if heading_deg is not None:
            yaw = math.radians(float(heading_deg))
            alignment = delta @ np.array([math.cos(yaw), math.sin(yaw)]) / np.sqrt(length_sq)
            aligned = alignment >= .5
            if aligned.any():
                distance_sq = np.where(aligned, distance_sq, float("inf"))
        at = int(np.argmin(distance_sq))
        segment = int(segments[at])
        station = self.route_station[segment]+fraction[at]*math.sqrt(length_sq[at])
        return float(station), math.sqrt(float(distance_sq[at])), segment

    def route_progress(self, ego):
        """Project the measured body centre onto the route without 0.5 m quantization."""
        if len(self.route_xy) < 2:
            return 0.
        xy = np.array([ego["x"], ego["y"]], dtype=float)
        _, nearby = self.route_tree.query(xy, k=min(8, len(self.route_xy)))
        nearby = np.atleast_1d(nearby)
        segments = np.unique(np.clip(np.r_[nearby-1, nearby], 0, len(self.route_xy)-2))
        return self._project_route(xy, segments, ego.get("heading_deg"))[0]

    def next_stop_line(self, station, front_m=2.3175, horizon_m=40.0):
        """Distance from the front bumper to the next stop line ahead, or None."""
        lines = getattr(self, "stop_line_stations", ())
        ahead = [s - station - front_m for s in lines if -0.3 <= s - station - front_m <= horizon_m]
        return min(ahead) if ahead else None

    def open_road_mask(self, world):
        """True where the route runs through a section whose whole road may be used
        (type "tunnel", or "all_lanes": true): no lane split, either direction."""
        spans = [(float(sec["start_m"]), float(sec["end_m"])) for sec in self.rules().get("sections") or []
                 if isinstance(sec, dict) and (sec.get("type") == "tunnel" or sec.get("all_lanes"))]
        if not spans or getattr(self, "route_tree", None) is None or not len(self.points):
            return np.zeros(len(world), dtype=bool)
        distance, index = self.route_tree.query(world)
        station = self.route_station[index]
        inside = np.zeros(len(world), dtype=bool)
        for start, end in spans:
            inside |= (station >= start) & (station <= end)
        return inside & (distance <= 6.0)

    def intersection_width(self, world):
        """Per-point free half-width (m) where the route crosses an intersection: HD-map junction
        links automatically (rules "auto_intersections": false turns that off) plus sections of type
        "intersection". No lanes there, so a bypass is generated in open space and checked only
        against objects/LiDAR. 0 elsewhere."""
        width = np.zeros(len(world))
        if getattr(self, "route_tree", None) is None:
            return width
        rules = self.rules()
        spans = [(float(sec["start_m"]), float(sec["end_m"]), float(sec.get("width_m") or 4.5))
                 for sec in rules.get("sections") or []
                 if isinstance(sec, dict) and sec.get("type") == "intersection"]
        distance, index = self.route_tree.query(world)
        station = self.route_station[index]
        for start, end, half in spans:
            inside = (station >= start) & (station <= end) & (distance <= 6.0)
            width[inside] = np.maximum(width[inside], half)
        if rules.get("auto_intersections", True):
            auto = self.route_junction[index] & (distance <= 6.0)
            width[auto] = np.maximum(width[auto], 4.5)
        return width

    def widen_to_road(self, world, normal, left, right, mask):
        """Extend the corridor over every HD-map lane beside the route, whatever its direction."""
        if getattr(self, "point_half_width", None) is None:
            widths = np.array([min(float(l.get("width_start") or 3.5), float(l.get("width_end") or 3.5))
                               for l in self.links]) if self.links else np.zeros(0)
            self.point_half_width = np.clip(widths, 2.5, 4.5)[self.link_of] / 2.
        for i in np.flatnonzero(mask):
            near = self.tree.query_ball_point(world[i], 6.0)
            if not near:
                continue
            lateral = (self.points[near] - world[i]) @ normal[i]
            half = self.point_half_width[near]
            beside = (np.abs(lateral) > .5) & (np.abs(lateral) <= 5.5)
            if np.any(beside & (lateral > 0)):
                left[i] = max(left[i], float(np.max((lateral + half)[beside & (lateral > 0)])))
            if np.any(beside & (lateral < 0)):
                right[i] = min(right[i], float(np.min((lateral - half)[beside & (lateral < 0)])))

    def local_zones(self, ego):
        """No-go polygons converted from map coordinates into the ego-local route frame."""
        zones = []
        yaw = math.radians(float(ego["heading_deg"]))
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        for zone in self.rules().get("zones") or []:
            try:
                polygon = np.asarray(zone["polygon"], dtype=float)[:, :2]
            except (KeyError, TypeError, ValueError, IndexError):
                continue
            if len(polygon) >= 3:
                zones.append((polygon - [ego["x"], ego["y"]]) @ rotation)
        return zones

    def adjacent_link(self, link, side):
        """Return a same-direction neighbour across a shared internal lane mark."""
        key = "left_lane_change_dst_link_idx" if side > 0 else "right_lane_change_dst_link_idx"
        target = self.link_by_id.get(link.get(key))
        if target is None or self.blocked(target.get("idx")):
            return None
        if self.rules().get("mode") == "whitelist":
            return target
        override = self.link_rule(link.get("idx")).get("left" if side > 0 else "right")
        if override == "deny":
            return None
        if override == "allow" and not target.get("opp_traffic"):
            return target
        allowed = link.get("can_move_left_lane" if side > 0 else "can_move_right_lane")
        if allowed:
            return target
        reverse = "right_lane_change_dst_link_idx" if side > 0 else "left_lane_change_dst_link_idx"
        edge = "lane_mark_left" if side > 0 else "lane_mark_right"
        other_edge = "lane_mark_right" if side > 0 else "lane_mark_left"
        shared = set(link.get(edge) or ()) & set(target.get(other_edge) or ())
        if (shared and link.get("road_id") and link.get("road_id") == target.get("road_id")
                and target.get(reverse) == link.get("idx")
                and not link.get("opp_traffic") and not target.get("opp_traffic")
                and abs(int(link.get("ego_lane") or 0) - int(target.get("ego_lane") or 0)) == 1):
            return target
        return None

    def road_corridor(self, ego, route_local, max_side_m=12., step_m=.25):
        """Physical road extent (union of every HD-map lane, both directions).

        The legal corridor is a cost in the lattice (slight encroachment into
        another lane beats stopping), but the road edge is not: a 3 m bypass
        put the body 0.43 m off the road (live 2026-10-03 20:35, idx 733-759).
        """
        route = np.asarray(route_local, dtype=float)[:, :2]
        if not self.n_links or len(route) < 3:
            return None
        if not hasattr(self, "_link_half_width"):
            self._link_half_width = np.array([min(float(l.get("width_start", 3.5)), float(l.get("width_end", 3.5))) / 2.
                                              for l in self.links])
        yaw = math.radians(float(ego["heading_deg"]))
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        world = route @ rotation.T + [ego["x"], ego["y"]]
        tangent = np.gradient(world, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        offsets = np.arange(step_m, max_side_m + 1e-6, step_m)
        extents = []
        for side in (1., -1.):
            probes = world[:, None, :] + side * offsets[None, :, None] * normal[:, None, :]
            distance, index = self.tree.query(probes.reshape(-1, 2), k=4)
            # +0.3 m bridges the 0.5 m centreline sampling at shared lane
            # edges (otherwise the road "ended" between two lanes); removed
            # again from the outer extent below.
            inside = (distance <= self._link_half_width[self.link_of[index]] + .3).any(axis=1)
            inside = inside.reshape(len(world), len(offsets))
            # Contiguous road from the route outwards.
            first_out = np.where(inside.all(axis=1), len(offsets), np.argmin(inside, axis=1))
            extents.append(np.maximum(0., np.where(first_out > 0, offsets[np.maximum(first_out - 1, 0)], 0.)
                                      + step_m / 2 - .3))
        from lane_corridor import LaneCorridor
        return LaneCorridor(route, extents[0], -extents[1])

    def corridor(self, ego, route_local, allow_second_same_direction=False):
        """Road bounds from the route lane and explicitly permitted same-direction neighbours."""
        route = np.asarray(route_local, dtype=float)[:, :2]
        if not self.n_links or len(route) < 3:
            return None
        yaw = math.radians(float(ego["heading_deg"]))
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        world = route @ rotation.T + [ego["x"], ego["y"]]
        tangent = np.gradient(world, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        distance, index = self.tree.query(world)
        link_ids = self.link_of[index]
        left, right = np.zeros(len(route)), np.zeros(len(route))
        adjacent_roots = set()
        for link_index in np.unique(link_ids):
            mask = np.flatnonzero((link_ids == link_index) & (distance <= 1.5))
            link = self.links[int(link_index)]
            # The global route may change link at a junction before a vehicle
            # finishes returning from a legal neighbouring lane. Retain that
            # neighbour's connected continuation for the short merge area.
            current_points = self.geometry.get(link.get("idx"))
            if (len(mask) and mask[0] < 90 and current_points is not None and
                    np.linalg.norm(world[mask[0]] - current_points[0]) <= 12.0):
                current_heading = current_points[min(3, len(current_points) - 1)] - current_points[0]
                current_heading /= max(np.linalg.norm(current_heading), 1e-9)
                for predecessor_id in self.predecessors.get(link.get("from_node_idx"), ()):
                    predecessor = self.link_by_id.get(predecessor_id)
                    prior_points = self.geometry.get(predecessor_id)
                    if predecessor is None or prior_points is None or predecessor.get("opp_traffic"):
                        continue
                    prior_heading = prior_points[-1] - prior_points[max(0, len(prior_points) - 4)]
                    prior_heading /= max(np.linalg.norm(prior_heading), 1e-9)
                    if (np.linalg.norm(prior_points[-1] - current_points[0]) > 2.0 or
                            float(prior_heading @ current_heading) < math.cos(math.radians(35))):
                        continue
                    for side in (1, -1):
                        neighbour = self.adjacent_link(predecessor, side)
                        if neighbour is not None and neighbour.get("idx") in self.geometry:
                            adjacent_roots.add((side, neighbour["idx"]))
            width = min(float(link.get("width_start", 3.5)), float(link.get("width_end", 3.5)))
            width = float(np.clip(width, 2.5, 4.5))
            base_lateral = np.sum((self.points[index[mask]] - world[mask]) * normal[mask], axis=1)
            left[mask], right[mask] = base_lateral + width / 2., base_lateral - width / 2.
            for side, key, allowed in ((1, "left_lane_change_dst_link_idx", "can_move_left_lane"),
                                       (-1, "right_lane_change_dst_link_idx", "can_move_right_lane")):
                target_link = self.adjacent_link(link, side)
                target = target_link.get("idx") if target_link else None
                if target not in self.geometry:
                    continue
                neighbor = self.geometry[target]
                _, at = self.link_trees[target].query(world[mask])
                nt = neighbor[np.minimum(at + 1, len(neighbor) - 1)] - neighbor[np.maximum(at - 1, 0)]
                nt /= np.maximum(np.linalg.norm(nt, axis=1, keepdims=True), 1e-9)
                lateral = np.sum((neighbor[at] - world[mask]) * normal[mask], axis=1)
                gap = np.linalg.norm(neighbor[at] - world[mask], axis=1)
                # A connected adjacent lane remains usable as its centreline
                # converges toward the route in a merge.
                valid = ((gap <= 5.5) & ((nt * tangent[mask]).sum(axis=1) >= math.cos(math.radians(35))) &
                         (side * lateral >= 0.0) & (side * lateral <= 5.))
                if np.any(valid & (mask < min(len(route), 90))):
                    adjacent_roots.add((side, target))
                half = min(float(target_link.get("width_start", 3.5)), float(target_link.get("width_end", 3.5))) / 2.
                selected = mask[valid]
                if side > 0:
                    left[selected] = np.maximum(left[selected], lateral[valid] + half)
                else:
                    right[selected] = np.minimum(right[selected], lateral[valid] - half)
        # The lane the car already occupies stays drivable where changing into
        # it is no longer permitted (solid line). Without this the corridor
        # narrowed from 5.0 m to 2.1 m around a car in the neighbour lane and
        # stopped it there (virtual highway run, 2026-10-03).
        ego_xy = np.array([ego["x"], ego["y"]])
        occupied_roots = set()
        ego_distance, ego_point = self.tree.query(ego_xy)
        if ego_distance <= 1.5:
            occupied = self.links[int(self.link_of[ego_point])]
            occupied_id = occupied.get("idx")
            occupied_points = self.geometry.get(occupied_id)
            if occupied_points is not None and not occupied.get("opp_traffic") and occupied_id not in {
                    self.links[int(i)].get("idx") for i in np.unique(link_ids[:20])}:
                at = int(np.argmin(np.linalg.norm(occupied_points - ego_xy, axis=1)))
                heading = occupied_points[min(at + 1, len(occupied_points) - 1)] - occupied_points[max(at - 1, 0)]
                heading /= max(np.linalg.norm(heading), 1e-9)
                lateral0 = float((occupied_points[at] - world[0]) @ normal[0])
                # Distinct lanes can converge to less than 1.5 m separation.
                # Dropping the occupied lane at that threshold removed its
                # permitted outer neighbour for two frames during a return,
                # causing a false 0.5 m map clearance and full braking.
                if float(heading @ tangent[0]) >= math.cos(math.radians(35)):
                    side0 = 1 if lateral0 > 0 else -1
                    adjacent_roots.add((side0, occupied_id))
                    occupied_roots.add((side0, occupied_id))
                    # Where the occupied lane merges into the route lane, the
                    # way past is its own outer neighbour (highway on-ramp).
                    neighbour = self.adjacent_link(occupied, side0)
                    if neighbour is not None and neighbour.get("idx") in self.geometry:
                        adjacent_roots.add((side0, neighbour["idx"]))
                        occupied_roots.add((side0, neighbour["idx"]))
        # A permitted adjacent lane can continue onto its own next link while
        # the global route takes a different link. Preserve that lane corridor
        # through the merge instead of treating its end as a curb.
        for side, root in adjacent_roots:
            # Lanes reached from the car's own lane may lie two lanes out.
            reach = 9.5 if (side, root) in occupied_roots else 5.0
            pending, visited = [(root, 0)], set()
            while pending:
                link_id, depth = pending.pop(0)
                if link_id in visited or depth > 2 or link_id not in self.geometry:
                    continue
                visited.add(link_id)
                link = self.link_by_id[link_id]
                if link.get("opp_traffic") or self.blocked(link_id):
                    continue
                points = self.geometry[link_id]
                _, at = self.link_trees[link_id].query(world)
                nearest = points[at]
                gap = np.linalg.norm(nearest - world, axis=1)
                nt = points[np.minimum(at + 1, len(points) - 1)] - points[np.maximum(at - 1, 0)]
                nt /= np.maximum(np.linalg.norm(nt, axis=1, keepdims=True), 1e-9)
                lateral = np.sum((nearest - world) * normal, axis=1)
                valid = ((gap <= reach + .5) & ((nt * tangent).sum(axis=1) >= math.cos(math.radians(35))) &
                         (side * lateral >= 0.0) & (side * lateral <= reach))
                half = min(float(link.get("width_start", 3.5)), float(link.get("width_end", 3.5))) / 2.
                if side > 0:
                    left[valid] = np.maximum(left[valid], lateral[valid] + half)
                else:
                    right[valid] = np.minimum(right[valid], lateral[valid] - half)
                if depth < 2:
                    for successor in self.successors.get(link.get("to_node_idx"), ()):
                        next_points = self.geometry.get(successor)
                        if next_points is not None and np.linalg.norm(points[-1] - next_points[0]) <= 2.0:
                            pending.append((successor, depth + 1))
        if allow_second_same_direction:
            # Recovery only: a car already in the first neighbouring lane can
            # briefly occupy the next same-direction lane while straightening.
            # Keep the topology, road, heading, and opposite-traffic checks;
            # lane-change flags alone are incomplete in this HD map.
            for side, root in adjacent_roots:
                first = self.link_by_id.get(root)
                if first is None:
                    continue
                key = "left_lane_change_dst_link_idx" if side > 0 else "right_lane_change_dst_link_idx"
                reverse = "right_lane_change_dst_link_idx" if side > 0 else "left_lane_change_dst_link_idx"
                second = self.link_by_id.get(first.get(key))
                if (second is None or second.get("opp_traffic") or self.blocked(second.get("idx")) or
                        not first.get("road_id") or first.get("road_id") != second.get("road_id") or
                        second.get(reverse) != root or
                        abs(int(first.get("ego_lane") or 0) - int(second.get("ego_lane") or 0)) != 1):
                    continue
                pending, visited = [(second["idx"], 0)], set()
                while pending:
                    link_id, depth = pending.pop(0)
                    if link_id in visited or depth > 2 or link_id not in self.geometry:
                        continue
                    visited.add(link_id)
                    link = self.link_by_id[link_id]
                    if link.get("opp_traffic") or self.blocked(link_id):
                        continue
                    points = self.geometry[link_id]
                    _, at = self.link_trees[link_id].query(world)
                    nearest = points[at]
                    gap = np.linalg.norm(nearest - world, axis=1)
                    nt = points[np.minimum(at + 1, len(points) - 1)] - points[np.maximum(at - 1, 0)]
                    nt /= np.maximum(np.linalg.norm(nt, axis=1, keepdims=True), 1e-9)
                    lateral = np.sum((nearest - world) * normal, axis=1)
                    valid = ((gap <= 11.5) & ((nt * tangent).sum(axis=1) >= math.cos(math.radians(35))) &
                             (side * lateral >= 5.) & (side * lateral <= 11.))
                    half = min(float(link.get("width_start", 3.5)), float(link.get("width_end", 3.5))) / 2.
                    if side > 0:
                        left[valid] = np.maximum(left[valid], lateral[valid] + half)
                    else:
                        right[valid] = np.minimum(right[valid], lateral[valid] - half)
                    if depth < 2:
                        for successor in self.successors.get(link.get("to_node_idx"), ()):
                            next_points = self.geometry.get(successor)
                            if next_points is not None and np.linalg.norm(points[-1] - next_points[0]) <= 2.0:
                                pending.append((successor, depth + 1))
        open_road = self.open_road_mask(world)
        if open_road.any():
            self.widen_to_road(world, normal, left, right, open_road)
        free = self.intersection_width(world)
        if free.any():
            left[:] = np.maximum(left, free)
            right[:] = np.minimum(right, -free)
        return LaneCorridor(route, left, right, zones=self.local_zones(ego),
                            intersection_mask=free > 0.)

    def adjacent_offsets(self, ego, route_local):
        """Lateral offsets to connected, same-direction lane centres near the route."""
        if not len(self.points):
            return {}
        route = np.asarray(route_local, dtype=np.float32)[:, :2]
        if len(route) < 3:
            return {}
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        k = min(len(route) - 2, int(np.searchsorted(arc, min(8., arc[-1]))))
        direction = route[k + 1] - route[max(0, k - 1)]
        if np.linalg.norm(direction) < 0.1:
            return {}
        yaw = math.radians(float(ego["heading_deg"]))
        c, s = math.cos(yaw), math.sin(yaw)
        q = np.array([ego["x"] + c * route[k, 0] - s * route[k, 1],
                      ego["y"] + s * route[k, 0] + c * route[k, 1]])
        route_heading = yaw + math.atan2(direction[1], direction[0])
        d2 = ((self.points - q) ** 2).sum(axis=1)
        free = float(self.intersection_width(q[None, :])[0])
        if free > 0:
            # Intersection: no lane to move into; offer an open-space sidestep that keeps the
            # car body (half width ~0.95 m + 0.35 m margin) inside the marked free width.
            offset = min(3.5, free - 1.3)
            if offset >= 1.5:
                return {1: offset, -1: offset}
        if self.open_road_mask(q[None, :])[0]:
            # Open road (tunnel): a full bypass may use the neighbouring lane in either direction.
            normal = np.array([-math.sin(route_heading), math.cos(route_heading)])
            lateral = (self.points[d2 <= 36.] - q) @ normal
            offsets = {}
            for side in (1, -1):
                candidates = side * lateral[(side * lateral >= 2.5) & (side * lateral <= 5.0)]
                if len(candidates):
                    offsets[side] = float(candidates[np.argmin(np.abs(candidates - 3.5))])
            if offsets:
                return offsets
        # Same 16 nearest points in the same order; a full argsort of every
        # map point cost ~2.5 ms per call.
        nearest = np.argpartition(d2, 16)[:16]
        for point_index in nearest[np.argsort(d2[nearest])]:
            if d2[point_index] > 1.5 ** 2:
                break
            link = self.links[int(self.link_of[point_index])]
            points = self.geometry.get(link.get("idx"))
            if points is None:
                continue
            at = int(np.argmin(((points - q) ** 2).sum(axis=1)))
            tangent = points[min(at + 1, len(points) - 1)] - points[max(0, at - 1)]
            link_heading = math.atan2(tangent[1], tangent[0])
            if abs((link_heading - route_heading + math.pi) % (2 * math.pi) - math.pi) > math.radians(30):
                continue
            offsets = {}
            normal = np.array([-math.sin(route_heading), math.cos(route_heading)])
            for side, key, allowed in ((1, "left_lane_change_dst_link_idx", "can_move_left_lane"),
                                       (-1, "right_lane_change_dst_link_idx", "can_move_right_lane")):
                target_link = self.adjacent_link(link, side)
                neighbor = self.geometry.get(target_link.get("idx")) if target_link else None
                if neighbor is None:
                    continue
                n = int(np.argmin(((neighbor - q) ** 2).sum(axis=1)))
                if np.linalg.norm(neighbor[n] - q) > 5.5:
                    continue
                neighbor_tangent = neighbor[min(n + 1, len(neighbor) - 1)] - neighbor[max(0, n - 1)]
                neighbor_heading = math.atan2(neighbor_tangent[1], neighbor_tangent[0])
                if abs((neighbor_heading - route_heading + math.pi) % (2 * math.pi) - math.pi) > math.radians(35):
                    continue
                lateral = float(np.dot(neighbor[n] - q, normal))
                if 2.5 <= side * lateral <= 5.0:
                    offsets[side] = side * lateral
            if offsets:
                return offsets
        return {}

    def local_links(self, ego, radius_m=90.0):
        if not len(self.points):
            return []
        d2 = ((self.points - np.array([ego["x"], ego["y"]], np.float32)) ** 2).sum(axis=1)
        mask = d2 <= radius_m * radius_m
        if not mask.any():
            return []
        yaw = math.radians(float(ego["heading_deg"])); c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = self.points[mask, 0] - ego["x"], self.points[mask, 1] - ego["y"]
        loc = np.column_stack((c * dx + s * dy, -s * dx + c * dy))
        ids = self.link_of[mask]
        out = []
        for lid in np.unique(ids):
            seg = loc[ids == lid]
            if len(seg) >= 2:
                out.append(seg)
        return out


class SpeedZones:
    """Cruise speed per route section from data/autodrive/speed_zones.json (reloaded every 2 s):
       [{"from_pct": 10, "to_pct": 25, "kmh": 25}, {"from_idx": 900, "to_idx": 1200, "kmh": 20}]"""
    def __init__(self, path, base_kmh, route_len):
        self.path, self.base, self.route_len = path, float(base_kmh), max(1, int(route_len))
        self.zones, self.loaded = [], 0.0
        self.file_cache = AsyncJsonCache(path, interval=2.)

    def reload(self):
        if time.monotonic() - self.loaded < 2.0:
            return
        self.loaded = time.monotonic()
        value = self.file_cache.value
        if isinstance(value, list):
            self.zones = value

    def kmh_at(self, nearest_idx):
        self.reload()
        pct = 100.0 * nearest_idx / self.route_len
        for z in self.zones:
            try:
                if "from_idx" in z and z["from_idx"] <= nearest_idx <= z["to_idx"]:
                    return float(z["kmh"]), z
                if "from_pct" in z and z["from_pct"] <= pct <= z["to_pct"]:
                    return float(z["kmh"]), z
            except (KeyError, TypeError):
                continue
        return self.base, None

    def set_route(self, points):
        points = np.asarray(points, dtype=float)[:, :2]
        self.station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        self.loop_length = float(self.station[-1] + np.linalg.norm(points[-1] - points[0]))

    def set_junction_previews(self, regions, limit_kmh):
        """Preview an existing entry cap without extending its active region."""
        limits = set()
        for region in regions:
            try:
                station = float(region['stop_station_m'])-EGO_FRONT_M-INTERSECTION_ENTRY_CAP_FRONT_GAP_M
                limit = float(limit_kmh)
                if (math.isfinite(station) and 0. <= station < self.loop_length
                        and math.isfinite(limit) and limit > 0.):
                    limits.add((station, limit))
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
        self.junction_previews = tuple(sorted(limits))

    def braking_kmh(self, nearest_idx, dynamic_zones=()):
        """Reach an upcoming speed limit with 2 m/s² deceleration before entry."""
        nearest_idx = int(nearest_idx) % self.route_len
        kmh, zone = self.kmh_at(nearest_idx)

        def cap_at(index):
            cap, _ = self.kmh_at(index)
            for start, end, limit in dynamic_zones:
                if start <= index <= end:
                    cap = min(cap, limit)
            return max(0., cap)

        def preview_cap(limit_kmh, distance):
            # Include 0.5 s response time and finish braking 3 m before entry.
            limit = limit_kmh/3.6
            return (math.sqrt((limit+1.)**2+4.*max(0., distance-3.))-1.)*3.6

        kmh = cap_at(nearest_idx)
        boundaries = {0}
        for start, end, _ in dynamic_zones:
            boundaries.update((int(start), int(end) + 1))
        for item in self.zones:
            try:
                if "from_idx" in item:
                    boundaries.update((int(item["from_idx"]), int(item["to_idx"]) + 1))
                elif "from_pct" in item:
                    boundaries.update((int(math.ceil(item["from_pct"] * self.route_len / 100.)),
                                       int(math.floor(item["to_pct"] * self.route_len / 100.)) + 1))
            except (KeyError, TypeError, ValueError):
                continue
        for boundary in boundaries:
            if not 0 <= boundary < self.route_len or boundary == nearest_idx:
                continue
            distance = float(self.station[boundary] - self.station[nearest_idx])
            if distance < 0.:
                distance += self.loop_length
            kmh = min(kmh, preview_cap(cap_at(boundary), distance))
        for station, limit_kmh in getattr(self, 'junction_previews', ()):
            distance = station-float(self.station[nearest_idx])
            if -3. <= distance < 0.:
                # Bridge route-index rounding and one fresh-pose update into
                # the unchanged entry cap; no state or entry permission.
                distance = 0.
            elif distance < 0.:
                distance += self.loop_length
            # The same preview envelope as ordinary speed-zone entries. Beyond
            # the short bridge only the existing traffic planner owns its cap.
            kmh = min(kmh, preview_cap(limit_kmh, distance))
        return kmh, zone


class RosPathPublisher:
    """Publish a validated rolling path in the existing MPC follower format."""
    def __init__(self, cruise_kmh):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as PathMessage
        from std_msgs.msg import Float32, String

        self.PoseStamped = PoseStamped
        self.PathMessage = PathMessage
        self.Float32 = Float32
        self.String = String
        self.cruise_mps = float(cruise_kmh) / 3.6
        import rospy
        self.rospy = rospy
        self.waypoints_pub = rospy.Publisher("/avoid_waypoints", String, queue_size=1)
        self.velocity_pub = rospy.Publisher("/avoid_target_vel", Float32, queue_size=1)
        self.mode_pub = rospy.Publisher("/lane_change_mode", String, queue_size=1)
        self.path_pub = rospy.Publisher("/student/processed_path", PathMessage, queue_size=1)
        self.status_pub = rospy.Publisher("/student/path_status", String, queue_size=1)
        # Legal driving corridor edges the planner is checking against (RViz).
        self.corridor_left_pub = rospy.Publisher("/student/corridor_left", PathMessage, queue_size=1)
        self.corridor_right_pub = rospy.Publisher("/student/corridor_right", PathMessage, queue_size=1)
        self._corridor_stamp = 0.0
        # Camera receive rates (the cameras arrive as UDP here, not as ROS topics), 1 Hz, for the sensor panel.
        self.camera_rates_pub = rospy.Publisher("/student/camera_rates", String, queue_size=1)
        self._camera_rates_stamp = 0.0
        self.tunnel_constraint_pub = rospy.Publisher("/localization/lane_constraint", String, queue_size=1)
        self._tunnel_constraint_stamp = 0.0
        self._command_lock = threading.RLock()
        self._safety_stop = threading.Event()
        self._safety_provider = None

    def start_safety(self, provider):
        self._safety_provider = provider
        if not hasattr(self, '_safety_stop'):
            self._safety_stop = threading.Event()
        def monitor():
            while not self._safety_stop.wait(.05):
                try:
                    self.recheck_active_path()
                except Exception as error:
                    self.publish_stop('live path safety failed: '+str(error)[:160])
        self._safety_thread = threading.Thread(target=monitor, name='live-path-safety', daemon=True)
        self._safety_thread.start()

    @serialized_command
    def recheck_active_path(self):
        """A sensor-driven stop reaches control while the planner is busy."""
        if (getattr(self, 'reverse_state', None) is not None or
                getattr(self, '_reverse_exit_state', None) is not None):
            # A previous D baseline cannot restart motion during an escape or
            # while its forward release is still waiting for checked geometry.
            return False
        baseline = getattr(self, '_safety_baseline', None)
        provider = getattr(self, '_safety_provider', None)
        if baseline is None or provider is None or baseline['mode'] == 'HIGHWAY' or baseline.get('gear') == 'R':
            return False
        now = time.monotonic()
        # Geometry may remain usable during a slow search. Sensors and the
        # complete issued path are revalidated below on every timer tick;
        # their .6 s age limit is unchanged. Bound this bridge to two seconds.
        failed = getattr(self, '_planner_failed', False)
        if hasattr(provider, 'expire_yield_launch'):
            provider.expire_yield_launch(now)
        sensor_retry = failed and getattr(self, '_planner_sensor_recheck', False)
        if now-baseline['created'] > 2. or (failed and not sensor_retry):
            if getattr(self, '_last_target_mps', 0.) > 0.:
                self.publish_stop('live path geometry stale')
            return False
        decision = provider(baseline['world_path'], baseline['ego'], baseline['mode'], baseline['decision'])
        checked = (decision.fresh() and decision.checks_match_path() and
                   decision.validation.get('valid') and decision.validation.get('pose_valid') and
                   decision.validation.get('steering_valid') and not decision.emergency)
        entry_profile = (entry_profile_ceiling(decision, now) if checked and
            decision.validation.get('junction_entry_profile_accepted') is True else None)
        command_ceiling = baseline['target'] if entry_profile is None else entry_profile
        if checked and self._static_launch_ready(decision):
            command_ceiling = decision.allowed_mps
        target = min(command_ceiling, decision.allowed_mps) if checked else 0.
        lap_cap = getattr(self, 'lap_cap_mps', None)
        if lap_cap is not None:
            target = min(target, max(0., float(lap_cap)))
        if sensor_retry and checked:
            self._planner_failed = False
            self._planner_sensor_recheck = False
            self.emergency_stop = False
        previous = float(getattr(self, '_last_target_mps', 0.))
        if target > previous:
            if (getattr(self, '_speed_recovery_pending', False) or
                    str(baseline.get('mode', '')).startswith('TUNNEL')):
                # Tunnel: a one-frame cap dip also recovers from the measured speed
                # (see publish); a stop pulse everywhere, as before.
                measured = abs(float(decision.control_ego.get('speed_kmh', 0.)))/3.6
                if math.isfinite(measured):
                    previous = max(previous, min(target, measured))
            acceleration = baseline.get('launch_accel_mps2', LAUNCH_ACCEL_MPS2)
            map_launch = bool((decision.validation.get('junction_motion') or entry_profile is not None)
                             and decision.junction_motion_fresh(now))
            if map_launch or self._static_launch_ready(decision):
                acceleration = None
            elif baseline.get('junction_motion_launch') or baseline.get('static_motion_launch'):
                # A preceding permission cannot remove the ramp from a later
                # observation whose measured-motion proof is no longer valid.
                acceleration = LAUNCH_ACCEL_MPS2
            if acceleration is not None:       # None: released junction entry, no ramp
                target = min(target, previous+float(acceleration)*max(0., now-self._last_target_time))
        if hasattr(provider, "bound_static_lane_pass"):
            decision, target = provider.bound_static_lane_pass(decision, baseline["world_path"], target)
        signal_control_source = (target, decision)
        target, decision = self._signal_command(decision.control_ego, target, decision)
        if (abs(target-previous) < .001 and target > .05
                and not decision.validation.get('junction_source_launch')
                and not self._static_launch_ready(decision)):
            return False
        self._plan_sequence = getattr(self, '_plan_sequence', 0)+1
        decision = bind_plan_id(decision, self._plan_sequence, target)
        source_guard = (provider.publication_source_guard(decision) if
            hasattr(provider, 'publication_source_guard') else nullcontext(True))
        with source_guard as source_ok:
            source_publication_ok = bool(source_ok)
            if not source_ok:
                target = 0.
                decision = decision.decided(allowed_mps=0., emergency=True,
                    stop_reason='junction vehicle source changed before command publication')
            target, decision = self._signal_command(decision.control_ego, target, decision,
                control_before_signal=signal_control_source if source_ok else None)
            if hasattr(provider, 'bound_static_lane_pass'):
                decision, target = provider.bound_static_lane_pass(decision, baseline['world_path'], target)
            payload = dict(baseline['payload'], plan_id=self._plan_sequence,
                source_stamp=float(decision.control_ego.get('stamp', 0.)),
                target_speed_mps=target, decision=decision.summary())
            encoded = json.dumps(payload, separators=(',', ':'))
            if target > 0. and not decision.junction_motion_fresh():
                source_publication_ok = False
                target = 0.
                if hasattr(provider, 'reject_source_publication'):
                    provider.reject_source_publication(decision,
                        reason='junction motion proof expired before command publication')
                decision = decision.decided(allowed_mps=0., emergency=True,
                    stop_reason='junction motion proof expired before command publication')
                payload.update(target_speed_mps=0., decision=decision.summary())
                encoded = json.dumps(payload, separators=(',', ':'))
            self.waypoints_pub.publish(self.String(data=encoded))
            self.velocity_pub.publish(self.Float32(data=target))
            if hasattr(provider, 'note_published'):
                provider.note_published(decision, baseline['world_path'], target, time.monotonic())
        if source_publication_ok and (self._refreshed_entry_baseline(decision)
                                     or self._static_launch_ready(decision)):
            # Refresh sensor/control evidence without renewing the fixed world
            # geometry's two-second lifetime or retaining an earlier WAIT zero.
            baseline.update(decision=decision, ego=dict(decision.control_ego),
                target=0. if decision.emergency else decision.allowed_mps)
        self._last_waypoint_payload = payload
        self._last_target_mps, self._last_target_time = target, now
        self._speed_recovery_pending = target <= .05
        self._active_decision_plan = decision
        self._published_plan = dict(self._published_plan, plan_id=self._plan_sequence,
            received=now, target_mps=target, allowed_mps=decision.allowed_mps,
            emergency=decision.emergency, sensor_stamp=decision.sensor_stamp,
            validation=decision.validation, decision=decision.summary())
        return True

    @staticmethod
    def _refreshed_entry_baseline(plan):
        return bool(plan is not None
            and plan.validation.get('junction_entry_profile_refreshed') is True
            and not plan.validation.get('junction_source_rejected')
            and entry_profile_ceiling(plan) is not None)

    @classmethod
    def control_geometry(cls, local_path, dense_route, postprocess):
        """Canonicalize before traffic checks, then publish without edits."""
        path = np.asarray(local_path, float)[:, :2]
        if postprocess.get('canonical_control'):
            return path
        if postprocess.get('mode') == 'ROUTE BASE' and len(dense_route) >= 16:
            path = np.asarray(dense_route, float)[:120, :2]
        elif not postprocess.get('validated_static'):
            path = cls.extend_with_route(path, dense_route)
        if postprocess.get('mode') == 'LATTICE':
            ahead = np.flatnonzero(path[:, 0] > -.5)
            path = path[ahead[0]:] if len(ahead) else path
        if postprocess.get('mode') not in ('ROUTE BASE', 'HIGHWAY', 'LATTICE'):
            path = np.vstack((np.zeros((1, 2)), path))
        return cls.resample(path)

    @staticmethod
    def global_xy(current, forward, left):
        yaw = math.radians(float(current["heading_deg"]))
        x = float(current["x"]) + math.cos(yaw) * forward - math.sin(yaw) * left
        y = float(current["y"]) + math.sin(yaw) * forward + math.cos(yaw) * left
        return x, y

    @staticmethod
    def resample(points, spacing=0.5):
        points = np.asarray(points, dtype=np.float64)
        segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
        keep = np.concatenate(([True], segment > 0.05))
        points = points[keep]
        if len(points) < 2:
            return points
        distance = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))
        targets = np.arange(0.0, distance[-1] + spacing * 0.5, spacing)
        return np.column_stack((np.interp(targets, distance, points[:, 0]),
                                np.interp(targets, distance, points[:, 1])))

    @staticmethod
    def extend_with_route(local_path, dense_route, target_length_m=60.0, merge_length_m=12.0):
        """Extend the short learned horizon and smoothly return its final offset to the map."""
        local_path = np.asarray(local_path, dtype=np.float32)
        route = np.asarray(dense_route, dtype=np.float32)[:, :2]
        if len(local_path) < 2 or len(route) < 3:
            return local_path
        current_length = float(np.sum(np.linalg.norm(np.diff(
            np.vstack((np.zeros((1, 2), np.float32), local_path[:, :2])), axis=0), axis=1)))
        if current_length >= target_length_m:
            return local_path[:, :2]
        route_progress = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))))
        nearest = min(int(np.searchsorted(route_progress, current_length)), len(route) - 2)
        tail = route[nearest:]
        if len(tail) < 3:
            return local_path[:, :2]
        tail_s = route_progress[nearest:nearest + len(tail)] - route_progress[nearest]
        needed = target_length_m - current_length
        tail = tail[tail_s <= needed + merge_length_m]
        tail_s = tail_s[:len(tail)]
        if len(tail) < 2:
            return local_path[:, :2]
        offset = local_path[-1, :2] - tail[0]
        u = np.clip(tail_s / merge_length_m, 0.0, 1.0).astype(np.float32)
        weight = u * u * (3.0 - 2.0 * u)
        tail = tail + offset[None, :] * (1.0 - weight[:, None])
        return np.vstack((local_path[:, :2], tail[1:]))

    @serialized_command
    def start_reverse(self, current, timeout_s=14., road_heading_deg=None, distance_m=None,
                      static_recovery=None):
        # Back out along the road, not along the car: a car stuck 29 deg off the lane
        # reversed straight, came back still angled and stalled again (auto run 88).
        previous = getattr(self, '_reverse_exit_state', None) or {}
        if static_recovery is None:
            plan = getattr(self, '_active_decision_plan', None)
            static_recovery = bool(plan is not None and plan.validation.get('static_rear_axle'))
        self.reverse_state = dict(x=float(current["x"]), y=float(current["y"]),
                                  heading=float(current["heading_deg"] if road_heading_deg is None else road_heading_deg),
                                  until=time.monotonic() + timeout_s,
                                  distance=REVERSE_ESCAPE_M if distance_m is None else float(distance_m),
                                  static_recovery=bool(static_recovery or previous.get('static_recovery')))
        self._reverse_exit_state = self.reverse_state
        self.reverse_abort = False
        self._safety_baseline = None

    @serialized_command
    def reverse_active(self, current):
        state = getattr(self, "reverse_state", None)
        if state is None:
            return False
        moved = math.hypot(float(current["x"]) - state["x"], float(current["y"]) - state["y"])
        if moved >= state.get("distance", REVERSE_ESCAPE_M) or time.monotonic() > state["until"] or getattr(self, "reverse_abort", False):
            # Runtime checks this before publish(). Preserve the ended escape
            # so timeout, distance and abort cannot bypass forward release.
            self._reverse_exit_state = state
            self._reverse_stopping_state = state
            self.reverse_state, self.reverse_abort = None, False
            return False
        return True

    @serialized_command
    def publish_stop(self, reason='planner input unavailable', *, sensor_recheck=False):
        """Stop the last geometry atomically, including when no new path exists."""
        self._reset_reverse_forward_proof()
        provider = getattr(self, '_safety_provider', None)
        if provider is not None and hasattr(provider, 'clear_published_launch'):
            provider.clear_published_launch(reason)
        if provider is not None and hasattr(provider, 'update_yield_launch'):
            provider.update_yield_launch(stopped=True)
        self.emergency_stop = True
        self._planner_failed = True
        self._planner_sensor_recheck = bool(sensor_recheck)
        stamp = time.monotonic()
        held = getattr(self, '_last_waypoint_payload', None)
        if held is not None:
            self._plan_sequence = getattr(self, '_plan_sequence', 0)+1
            payload = dict(held, plan_id=self._plan_sequence, target_speed_mps=0.)
            if payload.get('decision') is not None:
                decision = dict(payload['decision'], plan_id=self._plan_sequence,
                    allowed_mps=0., emergency=True, forward_clear_m=0., stop_reason=reason,
                    validation=dict(payload['decision']['validation'], valid=False))
                payload['decision'] = decision
            self.waypoints_pub.publish(self.String(data=json.dumps(payload, separators=(',', ':'))))
            self._last_waypoint_payload = payload
            command = getattr(self, '_published_plan', None)
            if command is not None:
                self._published_plan = dict(command, plan_id=self._plan_sequence, received=stamp,
                    target_mps=0., allowed_mps=0., emergency=True, decision=payload.get('decision'),
                    validation=dict(command.get('validation') or {}, valid=False))
        self.velocity_pub.publish(self.Float32(data=0.))
        self.mode_pub.publish(self.String(data='SAFE_STOP'))
        self._last_target_mps, self._last_target_time = 0., stamp
        self._speed_recovery_pending = True

    def _reset_reverse_forward_proof(self):
        """A rejected publication breaks consecutive static forward evidence."""
        for name in ('reverse_state', '_reverse_exit_state'):
            state = getattr(self, name, None)
            if state is not None:
                state.pop('static_clear_scans', None)

    def _signal_command(self, ego, target_mps, decision=None, *, control_before_signal=None):
        """Recheck current signal permission at every forward command boundary."""
        controller = getattr(self, '_signal_controller', None)
        if controller is None or not controller.enabled:
            return target_mps, decision
        if control_before_signal is not None:
            return controller.apply(ego, target_mps, decision,
                                    control_before_signal=control_before_signal)
        return controller.apply(ego, target_mps, decision)

    @staticmethod
    def _static_launch_ready(plan):
        if plan is not None and plan.validation.get('static_profile_deadline') is not None:
            try:
                deadline = float(plan.validation['static_profile_deadline'])
                if not math.isfinite(deadline) or time.monotonic() > deadline:
                    return False
            except (TypeError, ValueError, OverflowError):
                return False
        return bool(plan is not None and not plan.emergency and plan.allowed_mps > 0.
            and plan.validation.get('static_motion_profile') is True
            and plan.validation.get('static_lane_borrow') is True
            and (plan.validation.get('static_lane_pass') or {}).get('launch_ready') is True
            and plan.fresh() and plan.checks_match_path())

    @serialized_command
    def publish_reverse(self, current, stop=False, decision_plan=None):
        """Straight 9 m path behind the car in R (follower: dynamic dead-end recovery, <= 2 km/h)."""
        provider = getattr(self, '_safety_provider', None)
        if provider is not None and hasattr(provider, 'clear_published_launch'):
            provider.clear_published_launch('reverse command')
        if provider is not None and hasattr(provider, 'update_yield_launch'):
            provider.update_yield_launch(stopped=True)
        state = self.reverse_state or getattr(self, '_reverse_stopping_state', {})
        h = math.radians(state.get("heading", float(current["heading_deg"])))
        points = [(float(current["x"]) - d * math.cos(h), float(current["y"]) - d * math.sin(h))
                  for d in np.arange(0., 9.01, .5)]
        waypoints = [{"x": float(x), "y": float(y), "heading": math.radians(float(current["heading_deg"])), "gear": "R"}
                     for x, y in points]
        self._plan_sequence = getattr(self, '_plan_sequence', 0)+1
        payload = dict(waypoints=waypoints, plan_id=self._plan_sequence,
                       source_stamp=float(current.get('stamp', 0.)), target_speed_mps=0. if stop else .55)
        if decision_plan is not None:
            payload['decision'] = decision_plan.summary()
        self._last_waypoint_payload = payload
        self.waypoints_pub.publish(self.String(data=json.dumps(payload, separators=(",", ":"))))
        self.velocity_pub.publish(self.Float32(data=payload['target_speed_mps']))
        self._last_target_mps, self._last_target_time = 0., time.monotonic()
        self._speed_recovery_pending = True
        self.mode_pub.publish(self.String(data="AVOID"))
        self._published_plan = dict(plan_id=self._plan_sequence, received=time.monotonic(),
            source_stamp=payload['source_stamp'], world_path=np.asarray(points),
            target_mps=payload['target_speed_mps'], emergency=stop, mode='AVOID', gear='R',
            decision=decision_plan.summary() if decision_plan is not None else None)
        self._safety_baseline = None
        return True

    @serialized_command
    def publish(self, current, local_path, dense_route, postprocess, decision_plan: DrivingPlan = None):
        provider = getattr(self, '_safety_provider', None)
        if provider is not None and hasattr(provider, 'expire_yield_launch'):
            provider.expire_yield_launch(time.monotonic())
        guard_world = None
        if provider is not None and postprocess.get('mode') != 'HIGHWAY':
            sensor_retry_safe = False
            if decision_plan is not None:
                validation = decision_plan.validation
                # A freshness-only wrapper clears valid/cap after geometry
                # checks. Other stops must retain their rejection, including
                # when a later stale search still looks geometrically safe.
                sensor_only_stop = (decision_plan.stop_reason == 'tunnel plan sensor stale' and
                    not validation.get('valid') and decision_plan.emergency and
                    decision_plan.allowed_mps == 0.)
                safe_authority = (validation.get('valid') and not decision_plan.emergency and
                    math.isfinite(decision_plan.allowed_mps) and decision_plan.allowed_mps > 0.)
                hazard_latched = (getattr(self, '_planner_failed', False) and
                    not getattr(self, '_planner_sensor_recheck', False))
                sensor_retry_safe = bool(not hazard_latched and
                    validation.get('pose_valid') and validation.get('steering_valid') and
                    not validation.get('blocked') and np.isfinite(decision_plan.path).all() and
                    math.isfinite(decision_plan.sensor_stamp) and decision_plan.sensor_stamp > 0. and
                    math.isfinite(float(decision_plan.control_ego.get('stamp', 0.))) and
                    float(decision_plan.control_ego.get('stamp', 0.)) > 0. and
                    math.isfinite(decision_plan.sensor_age_s) and decision_plan.sensor_age_s >= 0. and
                    (safe_authority or sensor_only_stop))
            # Slow tunnel searches must refresh their sensor proof before use.
            # Only bounded, previously checked geometry reaches revalidation.
            recheck_tunnel = bool(decision_plan is not None and
                str(postprocess.get('mode', '')).startswith('TUNNEL') and
                sensor_retry_safe and
                decision_plan.validation.get('valid') and
                decision_plan.validation.get('pose_valid') and
                decision_plan.validation.get('steering_valid') and
                decision_plan.sensor_stamp > 0. and
                0. <= time.monotonic()-decision_plan.checked_at <= 1.5 and
                0. <= decision_plan.sensor_age_s <= .6 and
                -.12 <= float(decision_plan.control_ego.get('stamp', 0.))-decision_plan.sensor_stamp <= .6)
            if decision_plan is not None and (not decision_plan.matches(current, local_path)
                    or not decision_plan.checks_match_path()
                    or (not decision_plan.fresh() and not recheck_tunnel)):
                sensor_retry = (str(postprocess.get('mode', '')).startswith('TUNNEL') and
                    decision_plan.matches(current, local_path) and decision_plan.checks_match_path() and
                    not decision_plan.fresh() and sensor_retry_safe)
                self.publish_stop('incoming decision geometry or sensor invalid', sensor_recheck=sensor_retry)
                return False
            canonical = (decision_plan.path if decision_plan is not None else
                         self.control_geometry(local_path, dense_route, postprocess))
            guard_world = np.asarray([self.global_xy(current, *point) for point in canonical])
            if decision_plan is None:
                decision_plan = DrivingPlan(getattr(self, '_plan_sequence', 0)+1,
                    canonical, current, current, float(current.get('stamp', 0.)), 0., time.monotonic(),
                    float(getattr(self, 'zone_mps', self.cruise_mps)), 60.,
                    bool(getattr(self, 'emergency_stop', False)),
                    dict(valid=True, pose_valid=True, steering_valid=True,
                         blocked=bool((postprocess.get('lattice') or {}).get('blocked')
                                      or postprocess.get('static_blocked')),
                         generator='lattice' if postprocess.get('mode') == 'LATTICE' else None,
                         dynamic_recovery_ids=postprocess.get('dynamic_recovery_ids', []),
                         standing_bypass_ids=postprocess.get('standing_bypass_ids', []),
                         static_rear_axle=bool(postprocess.get('static_rear_axle')),
                         road_scan_frame=postprocess.get('road_scan_frame'),
                         static_complete=bool(postprocess.get('static_complete')),
                         static_lane_borrow=bool(postprocess.get("static_lane_borrow")),
                         static_motion_profile=bool(postprocess.get('static_motion_profile')),
                         static_profile_ceiling_mps=postprocess.get('static_profile_ceiling_mps'),
                         static_profile_source_limit_mps=(float(getattr(self, 'zone_mps', self.cruise_mps))
                             if postprocess.get('static_motion_profile')
                             and postprocess.get('static_profile_command_mps') is not None
                             and float(getattr(self, 'zone_mps', self.cruise_mps))+1e-6
                                 < float(postprocess['static_profile_command_mps']) else None),
                         entry_reference=bool(postprocess.get('entry_reference')),
                         object_stream_recheck=postprocess.get('object_stream_recheck'),
                         junction_body_cap_mps=postprocess.get('junction_body_cap_mps'),
                         junction_gap_checked=bool((postprocess.get('stop_line') or {}).get('gap_checked')),
                         side_margin_m=float((postprocess.get('lattice') or {}).get('side_margin_m', .25)),
                         end_margin_m=float((postprocess.get('lattice') or {}).get('end_margin_m', .25))),
                    postprocess['mode'], postprocess.get('stop_reason'))
            entry_profile = postprocess.get('junction_entry_profile')
            if isinstance(entry_profile, dict):
                decision_plan = bind_entry_profile(decision_plan, entry_profile.get('ceiling_mps'),
                    entry_profile.get('line_id'), entry_profile.get('line_station_m'))
            self._safety_input_decision = decision_plan
            decision_plan = provider(guard_world, current, postprocess['mode'], decision_plan)
            current, local_path = decision_plan.control_ego, decision_plan.path
            postprocess = dict(postprocess, canonical_control=True, decision=decision_plan.summary())
        if decision_plan is not None:
            # The checked plan owns these fields. A previous branch's stop/cap
            # or unrelated clearance must never override this decision.
            usable = (decision_plan.matches(current, local_path) and decision_plan.fresh()
                      and decision_plan.checks_match_path()
                      and decision_plan.validation.get('valid') and decision_plan.validation.get('pose_valid')
                      and decision_plan.validation.get('steering_valid') and np.isfinite(decision_plan.path).all()
                      and math.isfinite(decision_plan.allowed_mps) and decision_plan.allowed_mps >= 0.)
            if not usable:
                decision_plan = decision_plan.decided(allowed_mps=0., emergency=True,
                    stop_reason='decision geometry or sensor invalid', valid=False)
            self.zone_mps = decision_plan.allowed_mps if usable else 0.
            self.emergency_stop = decision_plan.emergency or not usable
            postprocess = dict(postprocess, forward_clear_m=decision_plan.forward_clear_m if usable else 0.)
            self._active_decision_plan = decision_plan
        was_reversing = getattr(self, 'reverse_state', None)
        reversing = self.reverse_active(current)
        reverse_exit = getattr(self, '_reverse_exit_state', None) or was_reversing
        if reversing or (reverse_exit and reverse_exit.get('static_recovery')):
            forward_clear = float(postprocess.get('forward_clear_m') or 0.)
            required = max(8., float(current.get('speed_kmh', 0.))/3.6*1.5+4.)
            forward_ready = (usable and decision_plan.forward_ready(current) if decision_plan is not None else
                postprocess.get('validated_static') and not getattr(self, 'emergency_stop', False)
                    and float(getattr(self, 'zone_mps', 0.) or 0.) > 0. and forward_clear >= required)
            forward_ready = static_reverse_release_ready(reverse_exit, decision_plan, forward_ready)
            if forward_ready:
                # A fresh checked forward plan ends the escape; the follower
                # stops before changing R -> D, rather than waiting 14 seconds.
                self.reverse_state = None
                self.reverse_abort = False
                self._reverse_release_stop = True
                self._reverse_stopping_state = reverse_exit
            else:
                return self.publish_reverse(current,
                                            stop=not reversing or (decision_plan is not None and
                                                (not usable or not decision_plan.fresh(max_age_s=.25))),
                                            decision_plan=decision_plan)
        if reverse_exit and self.reverse_state is None:
            self._reverse_release_stop = True
            self._reverse_stopping_state = reverse_exit
        if (getattr(self, '_reverse_release_stop', False)
                and abs(float(current.get('speed_kmh', 0.))) > .5):
            return self.publish_reverse(current, stop=True, decision_plan=decision_plan)
        if decision_plan is not None and not usable:
            self.publish_stop(decision_plan.stop_reason)
            return False
        def publish_zero():
            self.publish_stop('planner path invalid or too short')

        # A stop must reach the follower even when the path is too short/invalid.
        local_path = np.asarray(local_path, dtype=float if decision_plan is not None else np.float32)
        if local_path.ndim != 2 or local_path.shape[1] < 2 or len(local_path) < 3 or not np.all(np.isfinite(local_path[:, :2])):
            publish_zero()
            return False
        # Static bypass candidates carry a third placeholder column. ROS waypoints
        # use x/y only; keep every published branch in the same 2-D format.
        local_path = local_path[:, :2]
        if decision_plan is not None:
            control_path = decision_plan.path
        elif postprocess.get('canonical_control'):
            control_path = local_path
        elif postprocess.get("mode") == "ROUTE BASE" and len(dense_route) >= 16:
            # 2026-09-27 (this PC): the ego-anchored base path (starts at the car, merges into the map over 12 m, re-sent 8x/s)
            # kept the LTV-MPC ~1.2 m left of the route for whole sessions. With exact MORAI ego there is no reason to anchor at
            # the car: hand the map route itself (from the nearest waypoint, 60 m) to the follower, like the plain follow mode.
            route = np.asarray(dense_route, dtype=np.float32)[:, :2]
            control_path = route[:int(min(len(route), 120))]
        else:
            control_path = local_path if postprocess.get("validated_static") else self.extend_with_route(local_path, dense_route)
        if decision_plan is None and not postprocess.get('canonical_control') and postprocess.get("mode") == "LATTICE":
            # The lattice path already starts at the car's route projection.
            # Prepending the origin made a back-and-forth kink (follower saw
            # 0.9 1/m on a straight road, capped 10 km/h and wobbled; live
            # 2026-10-03). Drop points behind the rear axle instead.
            ahead = np.flatnonzero(control_path[:, 0] > -.5)
            control_path = control_path[ahead[0]:] if len(ahead) else control_path
        local_xy_points = (control_path if decision_plan is not None or postprocess.get('canonical_control') or postprocess.get("mode") in ("ROUTE BASE", "HIGHWAY", "LATTICE")
                           else np.vstack((np.zeros((1, 2), np.float32), control_path)))
        global_points = [self.global_xy(current, float(point[0]), float(point[1]))
                         for point in local_xy_points]
        global_points = (guard_world if guard_world is not None else np.asarray(global_points)
                         if decision_plan is not None or postprocess.get('canonical_control') else self.resample(global_points))
        path_length = (float(np.sum(np.linalg.norm(np.diff(global_points, axis=0), axis=1)))
                       if len(global_points) >= 2 else 0.0)
        if len(global_points) < 16 or path_length < 8.0:
            publish_zero()
            return False

        waypoints = []
        for index, (x, y) in enumerate(global_points):
            other = global_points[min(index + 1, len(global_points) - 1)]
            if index == len(global_points) - 1:
                other = global_points[index - 1]
                heading = math.atan2(y - other[1], x - other[0])
            else:
                heading = math.atan2(other[1] - y, other[0] - x)
            waypoints.append({"x": float(x), "y": float(y), "heading": heading, "gear": "D"})

        mode = ("HIGHWAY" if postprocess["mode"] == "HIGHWAY" else
                "AVOID" if postprocess["mode"].startswith(("STUDENT", "TUNNEL")) else "CRUISE")
        target = getattr(self, "zone_mps", None)
        requested = 0.0 if getattr(self, "emergency_stop", False) else (self.cruise_mps if target is None else target)
        lap_cap = getattr(self, "lap_cap_mps", None)
        if lap_cap is not None:
            # One-lap run (lap_finish.py): brake smoothly to rest back at the start point.
            requested = min(requested, max(0., float(lap_cap)))
        if getattr(self, '_reverse_release_stop', False):
            if abs(float(current.get('speed_kmh', 0.))) > .5:
                requested = 0.
            else:
                self._reverse_release_stop = False
        stamp = time.monotonic()
        map_launch = bool(decision_plan is not None and not decision_plan.emergency
            and decision_plan.allowed_mps > 0.
            and decision_plan.validation.get('junction_motion')
            and decision_plan.junction_motion_fresh(stamp))
        profile_launch = bool(decision_plan is not None and not decision_plan.emergency
            and decision_plan.allowed_mps > 0.
            and decision_plan.validation.get('junction_entry_profile_accepted') is True
            and entry_profile_ceiling(decision_plan, stamp) is not None)
        static_launch = self._static_launch_ready(decision_plan)
        previous = getattr(self, "_last_target_mps", None)
        planned_speed = ((postprocess.get("highway") or {}).get("speed_plan") or {}
                         if postprocess.get("mode") == "HIGHWAY" else {})
        if previous is not None and requested > previous and not planned_speed.get("reference_checked"):
            # Tunnel: a one-frame dip of the planner cap (8 km/h on a single "no safe
            # bypass" frame) restarted this 2.5 m/s^2 ramp from the dip while the car
            # still drove 39-45 km/h: 3 s to recover, the car braked to 9-19 km/h
            # (follow-20261010T162527Z 01:30:25.0, 01:35:21.9, 01:35:28.0). There the
            # ramp starts from the measured speed, as after a stop pulse; it still
            # limits any rise above the speed actually driven.
            tunnel_dip = str(postprocess.get("mode", "")).startswith("TUNNEL")
            if getattr(self, "_speed_recovery_pending", False) or tunnel_dip:
                # A brief stop pulse must not restart the target ramp at zero
                # while the car is still moving. Recover from measured speed,
                # bounded by the newly checked cap; standstill keeps the ramp.
                measured = float(current.get("speed_kmh", 0.)) / 3.6
                if math.isfinite(measured):
                    previous = max(previous, min(requested, max(0., measured)))
            stop_line = postprocess.get("stop_line") or {}
            # Released into the roundabout: the full entry cap at once (user rule 2026-10-10,
            # launch firmly). The old wall-clock ramp (2.8 m/s per wall s) was already faster
            # in sim time than the car (RTF 0.64: 4.4 m/s^2 sim vs 2.85-3.1 measured); the
            # follower's own 3 m/s^2 wall ramp and the vehicle's response remain, and the
            # entry forecast (ring_entry.LAUNCH_*) is fitted to that measured launch.
            if (stop_line.get("state") != "ENTER" and not stop_line.get('gap_checked')
                    and not map_launch and not profile_launch and not static_launch):
                requested = min(requested, previous + LAUNCH_ACCEL_MPS2 * max(0.0, stamp - self._last_target_time))
        if provider is not None and hasattr(provider, "bound_static_lane_pass"):
            decision_plan, requested = provider.bound_static_lane_pass(decision_plan, global_points, requested)
        signal_control_source = (requested, decision_plan)
        requested, decision_plan = self._signal_command(current, requested, decision_plan)
        self._speed_recovery_pending = requested <= 0.
        self._last_target_mps, self._last_target_time = requested, stamp
        self._plan_sequence = getattr(self, '_plan_sequence', 0)+1
        if decision_plan is not None:
            decision_plan = bind_plan_id(decision_plan, self._plan_sequence, requested)
            self._active_decision_plan = decision_plan
        source_guard = (provider.publication_source_guard(decision_plan) if provider is not None
            and hasattr(provider, 'publication_source_guard') else nullcontext(True))
        with source_guard as source_ok:
            source_publication_ok = bool(source_ok)
            if not source_ok:
                requested = 0.
                decision_plan = decision_plan.decided(allowed_mps=0., emergency=True,
                    stop_reason='junction vehicle source changed before command publication')
            requested, decision_plan = self._signal_command(current, requested, decision_plan,
                control_before_signal=signal_control_source if source_ok else None)
            if provider is not None and hasattr(provider, 'bound_static_lane_pass'):
                decision_plan, requested = provider.bound_static_lane_pass(decision_plan, global_points, requested)
            self._last_target_mps = requested
            payload = dict(waypoints=waypoints, plan_id=self._plan_sequence,
                           source_stamp=float(current.get('stamp', 0.)), target_speed_mps=requested)
            if decision_plan is not None:
                payload['decision'] = decision_plan.summary()
            encoded = json.dumps(payload, separators=(",", ":"))
            if requested > 0. and decision_plan is not None and not decision_plan.junction_motion_fresh():
                source_publication_ok = False
                requested = 0.
                if provider is not None and hasattr(provider, 'reject_source_publication'):
                    provider.reject_source_publication(decision_plan,
                        reason='junction motion proof expired before command publication')
                decision_plan = decision_plan.decided(allowed_mps=0., emergency=True,
                    stop_reason='junction motion proof expired before command publication')
                payload.update(target_speed_mps=0., decision=decision_plan.summary())
                encoded = json.dumps(payload, separators=(",", ":"))
            if requested <= 0. and decision_plan is not None:
                self._active_decision_plan = decision_plan
                self._last_target_mps, self._last_target_time = 0., time.monotonic()
                self._speed_recovery_pending = True
            self._last_waypoint_payload = payload
            self.waypoints_pub.publish(self.String(data=encoded))
            self.velocity_pub.publish(self.Float32(data=requested))
            if provider is not None and hasattr(provider, 'note_published'):
                provider.note_published(decision_plan, global_points, requested, time.monotonic())
        self._last_target_mps = requested
        if decision_plan is not None:
            self._active_decision_plan = decision_plan
        self.mode_pub.publish(self.String(data=mode))
        self._published_plan = dict(plan_id=self._plan_sequence, received=stamp,
            source_stamp=payload['source_stamp'], world_path=np.asarray(global_points),
            target_mps=requested, offset_m=float(postprocess.get('applied_peak_m', 0.)),
            emergency=bool(getattr(self, 'emergency_stop', False) or
                           (decision_plan is not None and decision_plan.emergency)), mode=mode, gear='D',
            allowed_mps=float(getattr(self, 'zone_mps', requested) or 0.),
            sensor_stamp=decision_plan.sensor_stamp if decision_plan is not None else postprocess.get('sensor_stamp'),
            validation=dict(decision_plan.validation) if decision_plan is not None else postprocess.get('validation', {}),
            decision=decision_plan.summary() if decision_plan is not None else None)
        if source_publication_ok and requested > 0.:
            self._reverse_exit_state = None
        else:
            self._reset_reverse_forward_proof()
        self._planner_failed = False
        self._planner_sensor_recheck = False
        base = getattr(self, '_safety_input_decision', None) if provider is not None else decision_plan
        if source_publication_ok and (self._refreshed_entry_baseline(decision_plan)
                or (decision_plan is not None and decision_plan.validation.get('object_stream_recovered'))):
            base = decision_plan
        self._safety_baseline = dict(payload=payload, world_path=global_points.copy(),
            ego=dict(base.control_ego if base is not None else current),
            mode=postprocess['mode'], target=(0. if base.emergency else base.allowed_mps) if base is not None else requested,
            decision=base, created=time.monotonic(), gear='D',
            junction_motion_launch=map_launch or profile_launch,
            static_motion_launch=static_launch,
            launch_accel_mps2=(None if (map_launch or profile_launch or static_launch or
                                      (postprocess.get('stop_line') or {}).get('state') == 'ENTER'
                                      or (postprocess.get('stop_line') or {}).get('gap_checked'))
                               else LAUNCH_ACCEL_MPS2))

        stamp = self.rospy.Time.now()
        path_message = self.PathMessage()
        path_message.header.stamp = stamp
        path_message.header.frame_id = "map"
        for x, y in global_points:
            pose = self.PoseStamped()
            pose.header = path_message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_message.poses.append(pose)
        self.path_pub.publish(path_message)
        self.status_pub.publish(self.String(data=json.dumps({
            "mode": mode, "postprocess": postprocess["mode"], "reason": postprocess["reason"],
            "points": len(waypoints), "length_m": path_length, "target_speed_mps": requested, "highway": postprocess.get("highway"),
        }, separators=(",", ":"), default=json_default)))
        return True

    def publish_camera_rates(self, rates):
        now = time.monotonic()
        if now - self._camera_rates_stamp < 1.0:
            return
        self._camera_rates_stamp = now
        self.camera_rates_pub.publish(self.String(data=json.dumps({k: round(float(v), 1) for k, v in rates.items()})))

    def publish_tunnel_constraint(self, constraint):
        """The ESKF independently gates these lateral/yaw observations on GPS loss."""
        stamp = float(constraint.get("stamp", 0.))
        if stamp < self._tunnel_constraint_stamp:
            self._tunnel_constraint_stamp = 0.
        if not constraint.get("valid") or stamp <= self._tunnel_constraint_stamp + .2:
            return
        self._tunnel_constraint_stamp = stamp
        self.tunnel_constraint_pub.publish(self.String(data=json.dumps(constraint, default=json_default)))

    def publish_corridor(self, ego, lane_corridor):
        """Left/right edges of the permitted lane corridor in map coordinates, at most 5 Hz."""
        now = time.monotonic()
        if lane_corridor is None or now - self._corridor_stamp < .2:
            return
        self._corridor_stamp = now
        yaw = math.radians(float(ego["heading_deg"]))
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        stamp = self.rospy.Time.now()
        for publisher, offset in ((self.corridor_left_pub, lane_corridor.left),
                                  (self.corridor_right_pub, lane_corridor.right)):
            local = lane_corridor.route + lane_corridor.normal * np.asarray(offset)[:, None]
            world = local[::4] @ rotation.T + [ego["x"], ego["y"]]
            message = self.PathMessage()
            message.header.stamp = stamp
            message.header.frame_id = "map"
            for x, y in world:
                pose = self.PoseStamped()
                pose.header = message.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                message.poses.append(pose)
            publisher.publish(message)

    def close(self):
        if hasattr(self, '_safety_stop'):
            self._safety_stop.set()
        if hasattr(self, '_safety_thread'):
            self._safety_thread.join(timeout=1.)
        for publisher in (self.waypoints_pub, self.velocity_pub, self.mode_pub,
                          self.path_pub, self.status_pub, self.corridor_left_pub, self.corridor_right_pub,
                          self.camera_rates_pub, self.tunnel_constraint_pub):
            publisher.unregister()


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


def mapped_road_floor(map_path, route_path):
    """A missing or uncertified road reference retains the original raw guard."""
    try:
        from perception.mapped_road_floor import MappedRoadFloor
        profile = MappedRoadFloor(map_path, route_path)
        return profile if profile.certified else None
    except Exception as exc:
        print(f"[lidar] mapped ground unavailable; retaining raw filter: {exc!r}", flush=True)
        return None


def local_xy(x, y, ego):
    yaw = math.radians(float(ego["heading_deg"]))
    dx, dy = x - float(ego["x"]), y - float(ego["y"])
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


class Runtime(TunnelRuntime, HighwayRuntime):
    def __init__(self, args):
        self.args = args
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model, self.checkpoint = load_checkpoint(args.checkpoint, self.device)
        self.config = self.checkpoint["config"]
        self.actions = self.checkpoint["actions"]
        self.concepts = self.checkpoint["concepts"]
        self.gallery = self.checkpoint.get("gallery", [])
        gallery_matrix = np.asarray([item["signature"] for item in self.gallery], dtype=np.float32)
        gallery_norms = np.linalg.norm(gallery_matrix, axis=1, keepdims=True)
        self.gallery_matrix = gallery_matrix / np.maximum(gallery_norms, 1e-8)
        self.concept_supervision = self.checkpoint.get("concept_supervision", {})
        self.receiver = (RosCameraReceiver(args.camera_ros_prefix) if args.camera_source == "ros"
                         else RecordedCameraReceiver(args.camera_files_dir) if args.camera_source == "files"
                         else CameraReceiver())
        self.ego_receiver = RosEgoReceiver(args.ego_topic)
        self.path_publisher = RosPathPublisher(args.cruise_kmh) if args.publish_control_path else None
        # Object_topic hazard rule (outside the network): MORAI's object list now, a perception module later.
        import os
        self.hazard_enabled = os.environ.get("STUDENT_OBJECT_GUARD", "1") != "0"
        self.object_receiver = RosObjectReceiver(os.environ.get("STUDENT_OBJECT_TOPIC", "/Object_topic")) if self.hazard_enabled else None
        self.hazard_stop_m = float(os.environ.get("STUDENT_HAZARD_STOP_M", "6.0"))
        self.lanes = HdMapLanes(os.environ.get("STUDENT_HDMAP_LINKS", "/hdmap/link_set.json"))
        # LiDAR free-space check (competition: no object list). STUDENT_CLEARANCE: objects (MORAI list drives the rules, default for
        # DAgger here) | lidar (point cloud drives bypass trigger, candidate clearance and the stop) | both (objects drive, lidar logged)
        self.clearance = os.environ.get("STUDENT_CLEARANCE", "both")
        lidar_topic = os.environ.get("STUDENT_LIDAR_TOPIC", "/lidar/points")
        mount = tuple(float(v) for v in os.environ.get("STUDENT_LIDAR_MOUNT", "1.05,0,1.35").split(","))
        try:
            self.lidar = LidarCorridor(lidar_topic if lidar_topic and lidar_topic != "-" else None, mount=mount, z_min=float(os.environ.get("STUDENT_LIDAR_ZMIN", "0.30")), x_range=(-75., 100.),
                road_floor=mapped_road_floor(os.environ.get("STUDENT_HDMAP_LINKS", "/hdmap/link_set.json"), args.route),
                pose_at_stamp=self.ego_receiver.measured_at_stamp)
        except Exception as exc:
            print(f"[lidar] disabled: {exc!r}", flush=True); self.lidar = None
        # Until the route-zone file loads, retain the normal 25 km/h cap.
        self.zone_kmh, self.zone = min(float(args.cruise_kmh), 25.0), None
        self.hazard_bypass_m = float(os.environ.get("STUDENT_HAZARD_BYPASS_M", "15.0"))
        self.last_hazard = None
        self.gap_planner = GapPlanner(self.lanes.links)
        route_obj = read_json(args.route)
        self.waypoints = route_obj.get("waypoints", []) if isinstance(route_obj, dict) else []
        if len(self.waypoints) < 20:
            raise ValueError("글로벌 경로 포인트 부족")
        self.speed_zones = SpeedZones(os.environ.get("STUDENT_SPEED_ZONES", "/data/autodrive/speed_zones.json"), min(args.cruise_kmh, 25.0), len(self.waypoints))   # needs the route length
        self.waypoint_xy = np.asarray(
            [(float(point["x"]), float(point["y"])) for point in self.waypoints], dtype=np.float64)
        self.speed_zones.set_route(self.waypoint_xy)
        self.lanes.set_route(self.waypoint_xy)
        signal_log = os.environ.get("STUDENT_LOG_DIR")
        self.signal_controller = SignalController(
            self.lanes, self.receiver, enabled=signal_enabled_from_env(),
            source="live" if args.camera_source in ("udp", "ros") else "recording",
            status_path=Path(signal_log)/"signal_status.json" if signal_log else None)
        if self.path_publisher is not None:
            self.path_publisher._signal_controller = self.signal_controller
        self.stop_line_controller = junction_controller(self.lanes.stop_lines, self.waypoint_xy)
        from junction_recovery import JunctionRecovery
        self.junction_recovery = JunctionRecovery()
        self.intersection_regions = (build_intersection_regions(
            self.lanes.stop_lines, self.lanes.route_junction, self.lanes.route_station,
            COMPETITION_STOP_NODE_IDS) if YIELD_AT_JUNCTION else [])
        intersection_file = os.environ.get("STUDENT_INTERSECTION_PERCEPTION")
        if intersection_file and YIELD_AT_JUNCTION:
            detector_regions = load_intersection_regions(intersection_file, self.waypoint_xy)
            if detector_regions != self.intersection_regions:
                raise ValueError("GUI and LiDAR intersection regions differ")
        self.speed_zones.set_junction_previews(self.intersection_regions, INTERSECTION_ENTRY_MPS*3.6)
        self._intersection_vehicle_status = None
        mission_file = os.environ.get("STUDENT_HIGHWAY_MISSION")
        self.highway = HighwayPlanner(mission_file, self.waypoint_xy) if mission_file else None
        if self.highway and not self.highway.config.get('use_mission_speed', False):
            self.highway.target_mps = min(self.highway.target_mps, args.cruise_kmh/3.6)
        self.vehicle_receiver = VehicleReceiver() if self.highway or self.intersection_regions else None
        # Measured points: 0.4 m cell centres moved roadside cones into the body corridor.
        self.near_field_memory = NearFieldMemory(retain_measured_points=True)
        self.control_waypoint_xy = smooth_control_route(self.waypoint_xy)
        segments = np.linalg.norm(np.diff(self.waypoint_xy, axis=0), axis=1)
        self.route_cumulative = np.concatenate(([0.0], np.cumsum(segments)))
        # One-lap run (STUDENT_ONE_LAP=1, E2E launch): stop back at the start pose.
        self.lap_tracker = lap_tracker_from_env(self.waypoint_xy) if self.path_publisher is not None else None
        if self.lap_tracker is not None:
            print(f"[lap] one-lap run: route {self.lap_tracker.length:.1f} m; the car stops back at its "
                  f"start pose (marker {lap_marker_path()})", flush=True)
        self.ego_history = []
        self.latest_status = {}
        # Simulator real-time factor (sim_time): measured object motion is in
        # wall-clock units, ego speed in simulator units. One estimate per
        # decision, shared by every planner and the live path checker.
        self.sim_clock = SimClock()
        self.input_error = "Ego 수신 대기"
        self.last_result = None
        self.last_images = None
        self.latencies = deque(maxlen=100)
        self.avoidance_direction = 0.0
        self.avoidance_latch_until = 0.0
        self.init_tunnel()
        if self.path_publisher is not None:
            self.path_publisher.start_safety(LivePathSafety(self))
        self.v6_latest = None
        self.v6_worker = None
        if os.environ.get("STUDENT_V6_CHECKPOINT"):
            try:
                import model_avoidance as _model_avoidance
                _model_avoidance.start_v6(self)
            except Exception as error:   # the drive continues; model mode falls back to the planner's side
                print(f"[v6] not started: {error!r}", flush=True)

    def remember_static_objects(self, objects, now, active, tracked_objects=(), *, measured_stamp=None):
        """Keep stationary boxes (world frame) for STATIC_OBJECT_MEMORY_S."""
        memory = [m for m in getattr(self, "_static_object_memory", [])
                  if active and 0. <= now - m[0] <= STATIC_OBJECT_MEMORY_S
                  and deduplicate_obstacles([m[1]], tracked_objects)]
        current = [o for o in objects if o[4] == "obstacle" and float(o[5]) < .3]
        merged = list(objects)
        for _, obj in memory:
            if all(math.hypot(obj[0] - o[0], obj[1] - o[1]) > 1. for o in current):
                merged.append(obj)
        if active:
            memory = [m for m in memory
                      if all(math.hypot(m[1][0] - o[0], m[1][1] - o[1]) > 1. for o in current)]
            for obj in current:
                geometry = object_metadata(obj).get('road_static_geometry')
                if geometry and 'initial_margin_floor_m' in geometry:
                    # Retain the verified physical box through a blind spot,
                    # but never retain a recovery allowance tied to this pose.
                    from perception.stationary_semantics import with_metadata
                    persistent = {k: v for k, v in geometry.items() if k not in
                        ('initial_margin_floor_m', 'initial_margin_restore_start_m')}
                    obj = with_metadata(obj, road_static_geometry=persistent)
                if measured_stamp is not None:
                    from perception.static_geometry_contract import (STORED_PROOF_KEY,
                        mark_stored_static_geometry)
                    if STORED_PROOF_KEY not in object_metadata(obj):
                        marker = mark_stored_static_geometry(obj, measured_stamp)
                        if marker is not None:
                            from perception.stationary_semantics import with_metadata
                            obj = with_metadata(obj, **{STORED_PROOF_KEY: marker})
                memory.append((now, obj))
        self._static_object_memory = memory
        return merged

    def model_static_plan(self, ego, dense_route, objs_snap, lid_static_pts, lane_corridor, map_ok,
                          map_clearance, previous_static, adjacent_offsets, now, in_obstacle_zone,
                          in_static_obstacle_zone, hw):
        """STUDENT_PLANNER=model (model_avoidance.py) for the normal road.

        Same planning area and LiDAR point set as the lattice branch: obstacle sections
        and a waited-out standing lead only, never a junction; people still waited for
        lose their returns. Only what the host's motion evidence calls stationary is
        planned around; LiDAR-only boxes cover objects fusion missed. The package planner
        then runs with the v6 model's side once it sees a static obstacle itself.
        Returns (plan, hazards, context)."""
        ma = model_avoidance
        in_junction = getattr(self, "_intersection_vehicle_status", None) is not None
        lead_bypass = bool(not in_junction and not in_obstacle_zone and self.lead_bypass_active(ego, now))
        planning = not in_junction and (in_static_obstacle_zone or lead_bypass)

        wait = getattr(self, '_standing_bypass_wait', None)
        if wait is None:
            wait = self._standing_bypass_wait = StandingBypassWait()
        tracker = getattr(self, '_ped_tracker', None)
        rtf = runtime_rtf(self, getattr(tracker, 'rtf', 1.))
        wait_objects = [object_metadata(o).get('semantic_person', o) for o in objs_snap]
        ready = wait.update(wait_objects if planning else [], ego, now, rtf, tracker,
                            set(getattr(self, '_ped_waiting', {})),
                            stamp=getattr(self.object_receiver, 'observation_stamp', None))
        def standing_person(obj):
            f, _ = local_xy(obj[0], obj[1], ego)
            return (len(obj) > 11 and obj[11] is not None and int(obj[11]) in ready
                    and (obj[11] in getattr(self, '_ped_waiting', {}) or
                         obj[11] in (previous_static or {}).get('standing_bypass_ids', ()))
                    and -PED_REAR_M-.5 < f < 30.)
        # Apply the forty-second timeout after this fresh wait update too.
        objs_snap = Runtime.standing_obstacle_objects(self, objs_snap, now) if planning else objs_snap
        classified = [o for o in objs_snap if object_metadata(o).get('pedestrian_static_reason')
                      and -PED_REAR_M-.5 < local_xy(o[0], o[1], ego)[0] < 30.]
        bypass_people = [o for o in objs_snap if o[4] == 'pedestrian' and standing_person(o)] + classified
        points = None
        if planning and self.clearance != "objects" and lid_static_pts is not None:
            points = np.asarray(lid_static_pts, dtype=float).reshape(-1, 2)
            if len(points):
                keep = np.ones(len(points), dtype=bool)
                for f, l in [local_xy(o[0], o[1], ego) for o in objs_snap
                             if o[4] == "pedestrian" and not standing_person(o)]:
                    keep &= np.hypot(points[:, 0] - f, points[:, 1] - l) > 1.2
                touching = getattr(self, "_contact", None)
                if touching and not touching["block_forward"]:
                    keep &= ~((points[:, 0] < 1.0) & (points[:, 0] > -PED_REAR_M - 1.)
                              & (np.abs(points[:, 1]) > .946) & (np.abs(points[:, 1]) < .946 + .7))
                points = points[keep]
        objs_snap = ma.validate_road_geometry_sources(objs_snap, ego)
        static, dynamic = ma.split_static(objs_snap, standing_person) if planning else ([], list(objs_snap))
        # Calibration identity includes measured motion before the planner zeros it.
        sources = [o for o in objs_snap if ma.is_static(o, standing_person)] if planning else []
        from perception.road_static_calibration import RoadStaticCalibration
        from static_lane_pass import normal_static_area
        calibration = getattr(self, '_road_static_calibration', None)
        if calibration is None:
            calibration = self._road_static_calibration = RoadStaticCalibration()
        calibration_active = bool(planning and in_static_obstacle_zone and lane_corridor is not None
            and points is not None and self.object_receiver and not dynamic
            and not any(o[4] == 'pedestrian' for o in objs_snap) and not bypass_people
            and not (previous_static or {}).get('standing_bypass_ids')
            and not getattr(self, '_contact', None)
            and not (getattr(self, 'tunnel', None) and self.tunnel.active(ego))
            and normal_static_area(self.lanes, ego))
        if planning:
            static = ma.road_static_objects(static, ego, points,
                allow_producer_geometry=calibration_active)
        static = calibration.update(sources, static, ego, points,
            proof=(getattr(self, '_lidar_log', {}) or {}).get('road_scan_frame', {}),
            now_wall=time.time(), active=calibration_active,
            retained_near_field_n=(getattr(self, '_lidar_log', {}) or {}).get('retained_near_field_n'))
        if calibration.last_diagnostics.get('restored_n'):
            static = ma.road_static_margin_recovery(static, ego, points)
        if hasattr(self, '_lidar_log'):
            self._lidar_log['road_static_calibration'] = dict(calibration.last_diagnostics)
        static = self.remember_static_objects(static, now, planning, objs_snap,
            measured_stamp=ego.get('stamp') if ego.get('measured') is True else None)
        lidar_boxes = []
        if points is not None and len(points) and lane_corridor is not None:
            from tunnel_runtime import corridor_points
            boxes = [box for box in ma.road_lidar_objects(corridor_points(points, lane_corridor), ego)
                     if not ma.wall_like(box)]
            lidar_boxes = ma.merge_lidar_obstacles(list(objs_snap) + static, boxes)
            static = static + lidar_boxes
        if planning:
            # A remembered physical box keeps its size, but recovery of an
            # existing margin deficit must be proved again at this pose.
            recovery_ledger = None
            if calibration_active:
                recovery_ledger = getattr(self, '_road_margin_recovery_ledger', None)
                if recovery_ledger is None:
                    recovery_ledger = self._road_margin_recovery_ledger = {}
            static = ma.road_static_margin_recovery(static, ego, points,
                recovery_ledger=recovery_ledger)
        check_points = points if planning else lid_static_pts
        from static_lane_pass import normal_static_area
        ordinary_static = bool(planning and in_static_obstacle_zone and lane_corridor is not None
            and check_points is not None and self.object_receiver
            and not any(o[4] == 'pedestrian' for o in objs_snap) and not bypass_people
            and not (previous_static or {}).get('standing_bypass_ids')
            and not any(float(p[2]) > 0. for p in (getattr(self, '_contact', None) or {}).get('points', ()))
            and not (getattr(self, 'tunnel', None) and self.tunnel.active(ego))
            and normal_static_area(self.lanes, ego))
        from perception.road_raw_memory import CommittedRoadRawMemory
        raw_memory = getattr(self, '_committed_road_raw_memory', None)
        if raw_memory is None:
            raw_memory = self._committed_road_raw_memory = CommittedRoadRawMemory()
        raw_targets = []
        raw_memory_active = bool(ordinary_static and not dynamic)
        if raw_memory_active and lidar_boxes:
            observed_hazards, _ = ma.confirmed_hazards(dense_route, ego, static,
                ma.footprint_hazards(dense_route, ego, static, body_shift_m=ma.REAR_AXLE_SHIFT_M))
            if observed_hazards:
                targets = [hazard['world_xy'] for hazard in observed_hazards]
                raw_targets = [box for box in lidar_boxes
                               if any(math.hypot(box[0]-target[0], box[1]-target[1]) < 1e-6
                                      for target in targets)]
        committed_raw = bool(previous_static and ma.static_plan_complete(previous_static)
            and not previous_static.get('bounded_static_pass') and not previous_static.get('static_borrow')
            and not previous_static.get('standing_bypass_ids'))
        retained_raw = raw_memory.update(raw_targets, ego,
            proof=(getattr(self, '_lidar_log', {}) or {}).get('road_scan_frame', {}),
            now_wall=time.time(), active=raw_memory_active, committed=committed_raw)
        if retained_raw:
            # Only this call's planning geometry receives the missing measured
            # target. Fresh fusion/raw observations win; semantic memory above
            # never receives these half-second raw boxes.
            static = static+ma.merge_lidar_obstacles(static, retained_raw)
        if hasattr(self, '_lidar_log'):
            self._lidar_log['road_raw_memory'] = dict(raw_memory.last_diagnostics)
        white_line_right_m = 0.
        if (planning and in_static_obstacle_zone and static and lane_corridor is not None
                and check_points is not None and not any(o[4] == 'pedestrian' for o in objs_snap)
                and not bypass_people and not (previous_static or {}).get('standing_bypass_ids')
                and not (getattr(self, 'tunnel', None) and self.tunnel.active(ego))
                and normal_static_area(self.lanes, ego)):
            # The operator permits a small white-line crossing. Keep this
            # permission independent of the timed left centre-line exception;
            # every candidate still needs map/no-go, body, SAT and raw checks.
            white_line_right_m = .5
            lane_corridor = ma.road_white_line_corridor(lane_corridor, white_line_right_m)

        def clearance(path_local, horizon=60.):
            available = map_clearance(path_local, horizon)
            if self.clearance != "objects":
                if lid_static_pts is None:
                    return 0.
                if check_points is not None and len(check_points):
                    hit = self.lidar.blocked(path_local, hw, horizon + 2.3175, pts=check_points)
                    if hit is not None:
                        available = min(available, max(0., hit[0] - 2.3175))
            return available
        if planning:
            map_ok, clearance = ma.road_static_checks(lane_corridor,
                check_points if self.clearance != 'objects' else np.empty((0, 2)))
        footprint = ma.footprint_hazards(dense_route, ego, static,
                                         body_shift_m=ma.REAR_AXLE_SHIFT_M)
        hazards_seen, engagement = ma.confirmed_hazards(dense_route, ego, static, footprint)
        decision, why = ma.v6_decision(self)
        # Near the route centre is not the end of a small bypass. The object's
        # centre may already be behind the route trigger while its body is still
        # beside our front/rear corners. Keep the checked rejoin until the route
        # itself is safe for the current body and its immediate join. This is
        # only a side-completion check: a farther obstacle must choose a new
        # side. Keep the checked geometry until the ordinary road's full return
        # horizon is clear, independently of that direction latch.
        return_horizon = 1.
        route_return_ready = bool(
            clearance(dense_route, return_horizon) >= return_horizon and
            static_clear_distance(dense_route, ego, static, return_horizon,
                                  body_shift_m=ma.REAR_AXLE_SHIFT_M) >= return_horizon)
        full_geometry_return = bool(ordinary_static
            and not (previous_static or {}).get('bounded_static_pass')
            and not (previous_static or {}).get('static_borrow')
            and not (previous_static or {}).get('standing_bypass_ids'))
        route_geometry_return_ready = route_return_ready
        if full_geometry_return:
            route_geometry_return_ready = bool(clearance(dense_route, 35.) >= 35.
                and static_clear_distance(dense_route, ego, static, 35.,
                                          body_shift_m=ma.REAR_AXLE_SHIFT_M) >= 35.)
        profile_policy = getattr(self, '_static_lane_pass', None)
        if profile_policy is not None:
            profile_policy.observe(ego, now)
        profile_owned = bool(previous_static and previous_static.get('static_motion_profile')
            and profile_policy is not None and profile_policy.legal is not None and not profile_policy.finished)
        release_return_side = bool(not profile_owned and ma.manoeuvre_complete(previous_static, dense_route)
                                   and route_return_ready)
        if release_return_side and route_geometry_return_ready:
            previous_static = None
        standing_continuation = bool(previous_static and previous_static.get('standing_bypass_ids'))
        direction, source = ma.choose_direction(None if release_return_side else previous_static,
                                                hazards_seen, decision)
        offsets = adjacent_offsets
        if hazards_seen and lane_corridor is not None:
            offsets = self.model_road_offsets(ego, dense_route, lane_corridor, adjacent_offsets, hazards_seen[0])
        if not profile_owned and ma.kept_side_closed(source, direction, offsets, dense_route, hazards_seen):
            previous_static = None   # the kept side has no room beside this obstacle
            direction, source = ma.choose_direction(None, hazards_seen, decision)
        from static_lane_pass import normal_static_area
        steady_static = bool(planning and in_static_obstacle_zone and lane_corridor is not None
            and check_points is not None and not any(o[4] == 'pedestrian' for o in objs_snap)
            and not any(float(p[2]) > 0. for p in (getattr(self, '_contact', None) or {}).get('points', ()))
            and not (getattr(self, 'tunnel', None) and self.tunnel.active(ego))
            and normal_static_area(self.lanes, ego) and not standing_continuation)
        # Prepare the S path from every measured footprint row ahead. The
        # immediate activation window otherwise hides the next opposite-side
        # row until the ego has already committed to a large held offset.
        near_rows = [hazard for hazard in footprint if hazard['front'] <= 45.]
        prefer_slalom = bool(ordinary_static and any(
            first['lat']*second['lat'] < 0.
            and abs(first['lat']-second['lat']) >= .6
            and 0. < second['along']-first['along'] <= 25.
            for first, second in zip(near_rows, near_rows[1:])))
        if profile_owned:
            # The accepted world path owns the manoeuvre until the measured
            # body returns. Revalidate it below and at publication; replacing
            # its near join every frame caused stop pulses during line contact.
            plan, hazards, info = dict(previous_static, reused=True), hazards_seen, dict(profile_continuation=True)
        elif self.object_receiver and not (bypass_people or standing_continuation) and steady_static:
            # Evaluate the exact published rear-axle path, including its join.
            # An origin -> rear vertex -> forward kink otherwise reaches the
            # contact guard after an apparently safe portable-plan check.
            canonical_cache = {}
            from static_lane_pass import measured_initial_join
            if previous_static and (previous_static.get('canonical_control')
                                    or previous_static.get('bounded_static_pass')):
                # continue_static_plan already joins these checked vertices
                # to the measured pose. Resampling that join again can double
                # its discrete curvature and reject a still trackable pass.
                continued = np.asarray(previous_static['path'], dtype=float)[:, :2]
                checked = continued if previous_static.get('fixed_world_curve') else measured_initial_join(continued, ego,
                    max_curvature=MAX_CONTROL_CURVATURE, lateral_accel=BYPASS_ALAT_MPS2,
                    target_mps=ma.ROAD_STATIC_FAST_MPS)
                canonical_cache[continued.tobytes()] = checked
                canonical_cache[checked.tobytes()] = checked
            def canonical(path):
                path = np.asarray(path, dtype=float)[:, :2]
                key = path.tobytes()
                if key not in canonical_cache:
                    ahead = np.flatnonzero(path[:, 0] > .05)
                    p = (np.vstack((np.zeros((1, 2)), path[ahead[0]:])) if len(ahead)
                         else np.zeros((2, 2)))
                    checked = RosPathPublisher.control_geometry(p, dense_route,
                        dict(mode='LATTICE', validated_static=True))
                    checked = measured_initial_join(checked, ego,
                        max_curvature=MAX_CONTROL_CURVATURE, lateral_accel=BYPASS_ALAT_MPS2,
                        target_mps=ma.ROAD_STATIC_FAST_MPS)
                    canonical_cache[key] = checked
                    canonical_cache[checked.tobytes()] = checked
                return canonical_cache[key]
            old_clearance, old_map_ok = clearance, map_ok
            _, comfortable = ma.road_static_checks(lane_corridor, check_points, margin_m=.45)
            comfortable_static = ma.road_comfort_objects(static)
            touching = getattr(self, '_contact', None) or ma.road_side_watch(check_points)
            def checked_clearance(path, horizon=60., *, comfortable_gap=False):
                p = canonical(path)
                if (len(p) < 3 or max(path_curvatures(p), default=math.inf) > MAX_CONTROL_CURVATURE
                        or (touching and (touching['block_forward'] or not contact_sweep_ok(
                            touching, p, max(1.5, touching['ahead_m']+.5))))):
                    return 0.
                return min((comfortable if comfortable_gap else old_clearance)(p, horizon),
                    static_clear_distance(p, ego, comfortable_static if comfortable_gap else static,
                                          horizon, body_shift_m=ma.REAR_AXLE_SHIFT_M))
            def checked_lane(path, tolerance=1.5):
                return old_map_ok(canonical(path), tolerance)
            def wide_clearance(path, horizon=60.):
                return checked_clearance(path, horizon, comfortable_gap=True)
            prefer_comfort = bool(hazards_seen and hazards_seen[0]['front'] >= 8.
                                  and float(ego.get('speed_kmh', 0.)) >= 8.)
            deadline = time.perf_counter()+.11
            lane_exception = getattr(self, '_static_lane_pass', None)
            checked_continuation = (ma.road_checked_continuation(
                previous_static, ego, canonical, checked_clearance)
                if lane_exception is None or lane_exception.entered_at is None or lane_exception.finished
                else None)
            if (checked_continuation is None and previous_static is not None
                    and (lane_exception is None or lane_exception.entered_at is None or lane_exception.finished)):
                # A six-metre measured-pose correction can be too abrupt for
                # a still usable world path. Search gentler joins to those same
                # saved vertices; each gets the full current body/map/raw and
                # measured steering proof before it may survive search expiry.
                saved = getattr(self, '_static_plan_saved', None)
                for join_m in (8., 10., 12.):
                    if time.perf_counter() >= deadline:
                        break
                    continued = continue_static_plan(saved, ego, now, join_m=join_m)
                    checked_continuation = ma.road_checked_continuation(
                        continued, ego, canonical, checked_clearance)
                    if checked_continuation is not None:
                        checked_continuation['continuation_join_m'] = join_m
                        break
            returned = (ma.road_rejoin_plan(dense_route, ego,
                comfortable_static if prefer_comfort else static, previous_static, canonical,
                wide_clearance if prefer_comfort else checked_clearance,
                min(deadline, time.perf_counter()+.025))
                if not prefer_slalom and (lane_exception is None or lane_exception.entered_at is None or lane_exception.finished) else None)
            previous_full_safe = bool(previous_static and ma.static_plan_complete(previous_static)
                and checked_clearance(previous_static['path'], max(35., float(previous_static['hold_until_m'])))
                    >= max(35., float(previous_static['hold_until_m'])))
            previous_contact_safe = bool(touching and previous_full_safe)
            release_unbounded_side = bool(previous_static and hazards_seen and not previous_full_safe
                and not previous_static.get('bounded_static_pass') and not previous_static.get('static_borrow')
                and (lane_exception is None or lane_exception.entered_at is None or lane_exception.finished))
            release_pending_bounded = bool(previous_static and hazards_seen and not previous_full_safe
                and ma.pending_bounded_search_release(previous_static, ego, lane_exception, lane_corridor))
            release_side = release_unbounded_side or release_pending_bounded
            search_previous = None if release_side else previous_static
            search_direction = (-direction if release_pending_bounded else 0.) if release_side else direction
            delayed = (ma.contact_static_pass(dense_route, ego, static, touching, canonical,
                checked_clearance, min(deadline, time.perf_counter()+.035), preferred_side=direction)
                if not prefer_slalom and not previous_contact_safe and returned is None else None)
            if returned is not None:
                plan, hazards, info = returned, hazards_seen, dict(road_return=True)
            elif delayed is not None:
                plan, hazards, info = delayed, hazards_seen, dict(contact_delayed=True)
            elif previous_contact_safe and previous_static.get('contact_delayed'):
                plan, hazards, info = dict(previous_static, reused=True), hazards_seen, dict(contact_delayed=True)
            elif prefer_comfort and not prefer_slalom:
                comfort_direction = search_direction or (-1. if hazards_seen[0]['lat'] > 0. else 1.)
                plan, hazards, info = ma.plan_bypass(dense_route, ego, comfortable_static, checked_lane, wide_clearance,
                    comfort_direction, search_previous, offsets, body_shift_m=ma.REAR_AXLE_SHIFT_M,
                    deadline=min(deadline, time.perf_counter()+.025))
            else:
                plan, hazards, info = None, [], {}
            if not prefer_slalom and not ma.static_plan_complete(plan):
                fallback, fallback_hazards, fallback_info = ma.plan_bypass(dense_route, ego, static,
                    checked_lane, checked_clearance, search_direction, search_previous, offsets,
                    body_shift_m=ma.REAR_AXLE_SHIFT_M, deadline=deadline)
                if fallback is not None and (not fallback.get('blocked') or plan is None or plan.get('blocked')):
                    plan, hazards, info = fallback, fallback_hazards, fallback_info
            if not ma.static_plan_complete(plan) and checked_continuation is not None:
                # A search timeout cannot revoke a full current body/map/raw
                # proof. Fresh objects and the live publisher still recheck
                # the selected geometry before any positive command is sent.
                plan, hazards = checked_continuation, hazards_seen
            if prefer_slalom and plan is None:
                # A single held offset cannot traverse alternating rows.
                # Preserve the bounded search budget for complete S paths.
                plan = dict(blocked=True, kind='bypass', safe_horizon_m=4., activation_m=35.)
                hazards = hazards_seen
            info['slalom_priority'] = prefer_slalom
            if plan is not None and not plan.get('blocked'):
                plan = dict(plan, path=canonical(plan['path']), canonical_control=True)
                if any((object_metadata(o).get('road_static_geometry') or {}).get('initial_margin_floor_m') is not None
                       for o in static):
                    plan['speed_cap_mps'] = min(plan['speed_cap_mps'], 15./3.6)
                    plan['initial_margin_recovery'] = True
                plan['curvature'] = float(max(path_curvatures(plan['path']), default=0.))
                plan['steady_clear_distance_m'] = wide_clearance(plan['path'], max(
                    35., float(plan['safe_horizon_m']), float(plan['hold_until_m'])))
            clearance, map_ok = checked_clearance, checked_lane
            info['steady_static_checked'] = True
            info['released_static_side'] = release_side
            info['released_pending_bounded'] = release_pending_bounded
        else:
            plan, hazards, info = ((ma.plan_bypass(dense_route, ego, static, map_ok, clearance, direction,
                                                   previous_static, offsets, body_shift_m=ma.REAR_AXLE_SHIFT_M))
                                   if self.object_receiver and not (bypass_people or standing_continuation) else (None, [], {}))
        if bypass_people or standing_continuation:
            # This person earned a search after a simulator-time stop. Keep the
            # ordinary crossing guard and live motion check authoritative; never
            # clear a stop merely because the timer expired.
            from tunnel_runtime import swept_clearance
            road = ma.standing_road_corridor(self.lanes, ego, dense_route, lane_corridor)
            def raw_clearance(path, horizon):
                if self.clearance != 'objects':
                    if lid_static_pts is None:
                        return 0.
                    return swept_clearance(path, check_points, horizon, .25,
                        lateral_margin=.25, body_shift_m=ma.REAR_AXLE_SHIFT_M)
                return horizon
            movers = [o for o in objs_snap if o[4] == 'pedestrian' and float(o[5]) >= .2]
            moving_guard = ped_guard_plan(movers, ego, dense_route,
                float(ego.get('speed_kmh', 0.))/3.6, 8./3.6, tracker, now=now)
            if moving_guard['kind'] == 'stop' or moving_guard.get('stopping'):
                plan = dict(blocked=True, kind='bypass', safe_horizon_m=4., activation_m=25.)
                hazards = footprint
                info = dict(standing_bypass_veto='moving person entering the path')
            else:
                people_to_check = ([o for o in bypass_people if o[4]=='pedestrian'] if bypass_people else
                    [o for o in objs_snap if o[4]=='pedestrian' and len(o)>11
                     and o[11] in previous_static['standing_bypass_ids']])
                policies = {int(o[11]): wait.policy(o, now) for o in people_to_check}
                plan, hazards = ma.plan_standing_bypass(dense_route, ego, static,
                    lane_corridor, raw_clearance, people_to_check, previous_static, road,
                    allow_search=bool(bypass_people),
                    person_clearances={uid: p['gap_m'] if p else 1.05 for uid, p in policies.items()},
                    controlled_ids=[int(o[11]) for o in bypass_people],
                    static_person_ids=[int(o[11]) for o in classified], speed_cap_mps=5./3.6)
                info = {k: plan.get(k) for k in ('person_gap_m', 'boundary_contact_s', 'boundary_penalty_s',
                                                'best_person_gap_m', 'reason', 'person_clearances_m')}
            info['standing_bypass_ids'] = sorted(int(o[11]) for o in bypass_people)
        pass_policy = getattr(self, '_static_lane_pass', None)
        if pass_policy is not None and (not pass_policy.finished
                or bool(previous_static and previous_static.get('bounded_static_pass'))):
            pass_policy.observe(ego, now)
        pass_info = None
        from static_lane_pass import StaticLanePass, normal_static_area
        if (planning and in_static_obstacle_zone and not bypass_people and not standing_continuation
                and not (getattr(self, 'tunnel', None) and self.tunnel.active(ego))
                and normal_static_area(self.lanes, ego)
                and (bool(plan and plan.get('blocked')) or
                     bool(plan and not plan.get('blocked') and plan.get('kind') != 'rejoin'
                          and not ma.static_plan_complete(plan)) or
                     bool(previous_static and previous_static.get('bounded_static_pass')) or
                     bool(pass_policy and pass_policy.entered_at is not None))):
            if pass_policy is None:
                pass_policy = self._static_lane_pass = StaticLanePass()
            object_age = float(getattr(self.object_receiver, 'age_s', 999.))
            scan_age_value = (getattr(self, '_lidar_log', {}) or {}).get('age_s')
            scan_age = float(scan_age_value) if scan_age_value is not None else 999.
            static_sources_ready = bool(0. <= object_age <= .3 and 0. <= scan_age <= .3)
            object_stamp = float(getattr(self.object_receiver, 'observation_stamp', 0.) or 0.)
            if not static_sources_ready and object_stamp > 0. and check_points is not None:
                # Planning and publication share one stationary-label contract.
                # A fresh raw association never refreshes the semantic stamp.
                from live_path_safety import static_motion_source_freshness
                source_now = time.monotonic()
                source_proof = static_motion_source_freshness(check_points, ego,
                    [('semantic', objs_snap, object_stamp, object_age)], scan_age,
                    source_now, source_now)
                static_sources_ready = source_proof['ready']
            replacement, pass_info = pass_policy.plan(self.lanes, ego, dense_route, objs_snap,
                static, check_points, lane_corridor, previous_static, now, rtf,
                ready=static_sources_ready,
                budget_s=.11,
                contact=getattr(self, '_contact', None), legal_checked=True,
                reserve_source_bounds=not prefer_slalom or float(ego.get('speed_kmh', 0.)) > .5,
                slalom_first=prefer_slalom)
            if replacement is not None:
                plan = replacement
        if pass_policy is not None and plan is not None and not plan.get('blocked'):
            # A failed search has not replaced the issued excursion. Retiring
            # its still-pending lane frame while braking loses the observed
            # crossing origin if the car rolls onto the line afterward.
            pass_policy.release_unused(ego, now,
                using_exception=bool(plan and not plan.get('blocked') and plan.get('static_borrow')))
        if plan is not None and not plan.get('blocked'):
            plan['fast_static_clear_m'] = ma.fast_static_clearance(plan, ego, static, check_points)
        self._model_avoidance_log = dict(
            planning_area=planning, junction=in_junction, lead_bypass=lead_bypass,
            lane_exception_state=(None if pass_policy is None else 'finished' if pass_policy.finished
                                  else 'active' if pass_policy.entered_at is not None else 'pending'),
            fast_static_clear_m=(plan or {}).get('fast_static_clear_m'),
            preferred_raw_gap_m=(plan or {}).get('preferred_raw_gap_m'),
            fresh_return_candidates_n=(plan or {}).get('fresh_return_candidates_n'),
            cached_return_fallback=bool((plan or {}).get('cached_return_fallback')),
            kept_comfortable_return=bool((plan or {}).get('kept_comfortable_return')),
            kept_hard_return=bool((plan or {}).get('kept_hard_return')),
            kept_checked_static=bool((plan or {}).get('kept_checked_static')),
            continuation_join_m=(plan or {}).get('continuation_join_m'),
            bounded_pass=pass_info, white_line_right_m=white_line_right_m,
            lidar_box_padding_m=ma.ROAD_STATIC_MARGIN_M-ma.PORTABLE_STATIC_MARGIN_M,
            raw_static_margin_m=ma.ROAD_STATIC_MARGIN_M,
            fusion_boxes_normalized_n=sum('road_static_geometry' in object_metadata(o) for o in static),
            initial_margin_recovery=bool(plan and plan.get('initial_margin_recovery')),
            road_margin_anchors_n=len(getattr(self, '_road_margin_recovery_ledger', {})),
            road_margin_floor_min_m=min(((object_metadata(o).get('road_static_geometry') or {})[
                'initial_margin_floor_m'] for o in static if (object_metadata(o).get('road_static_geometry')
                or {}).get('initial_margin_floor_m') is not None), default=None),
            route_return_ready=route_return_ready,
            route_geometry_return_ready=route_geometry_return_ready,
            static_n=len(static), lidar_boxes_n=len(lidar_boxes), dynamic_n=len(dynamic),
            standing_wait_sim_s={str(uid): round(row['elapsed'], 2) for uid, row in wait.tracks.items()},
            standing_ready_ids=sorted(ready),
            stationary_semantic_ids={str(o[11]): object_metadata(o)['pedestrian_static_reason'] for o in classified},
            hazard_front_m=round(float(hazards_seen[0]["front"]), 1) if hazards_seen else None,
            footprint_hazard_m=round(float(footprint[0]["front"]), 1) if footprint else None,
            engagement_m=round(float(engagement), 1),
            offsets={str(k): round(float(v), 2) for k, v in (offsets or {}).items()},
            offset_starts={str(k): list(v) if isinstance(v, (list, tuple)) else [v]
                           for k, v in (getattr(offsets, "start_m_by_side", None) or {}).items()},
            plan_offset_m=round(float(plan["offset_m"]), 2) if plan and not plan.get("blocked") and "offset_m" in plan else None,
            plan_start_m=round(float(plan.get("start_m") or 0.), 1) if plan and not plan.get("blocked") else None,
            v6=(dict(action=decision["action"], p=round(float(decision["p"]), 3),
                     age_s=round(float(decision["age_s"]), 3)) if decision else None),
            v6_reason=why, direction=direction, direction_source=source,
            plan_kind=(plan or {}).get("kind"), blocked=bool(plan and plan.get("blocked")), **info)
        return plan, hazards, dict(static=static, clearance=clearance, points=check_points,
                                   corridor=lane_corridor,
                                   planning=planning, steady_static=steady_static, direction=direction,
                                   route_return_ready=route_return_ready,
                                   route_geometry_return_ready=route_geometry_return_ready,
                                   standing_bypass=bool(bypass_people or standing_continuation),
                                   hazard_m=float(footprint[0]["front"]) if footprint else None)

    def model_road_offsets(self, ego, dense_route, lane_corridor, near, hazard):
        """Room to go around a confirmed obstacle on the normal road (model mode).

        HdMapLanes.adjacent_offsets looks 8 m ahead of the car only. The static boxes
        inside the junction (route idx ~636-697) and before the lane that widens in on
        the right (idx ~738-804) had no room there until the car was ~16 m away, so the
        planner was blocked and braked (follow-20261009T191227Z/185440Z). Look up the
        room beside the obstacle too; model_avoidance.road_offsets offers it with starts
        where the lane corridor on that side begins to widen."""
        route = np.asarray(dense_route, dtype=float)[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        ahead = {}
        for station in (float(hazard["front"]), float(hazard.get("along", hazard["front"]))):
            first = int(np.searchsorted(arc, max(0., station - 8.)))
            if len(route) - first >= 3:
                for side, value in (self.lanes.adjacent_offsets(ego, route[first:]) or {}).items():
                    ahead.setdefault(int(side), float(value))
            if ahead:
                break
        return model_avoidance.road_offsets(near, ahead, arc, lane_corridor.left, lane_corridor.right)

    def pedestrian_lattice_wait(self, objects, ego, route):
        """Disable bypass candidates only for people already in the near corridor."""
        for obj in objects:
            f, l = local_xy(obj[0], obj[1], ego)
            if 0. < f <= 10. and abs(route_lateral(route, f, l)) <= CORRIDOR_HALF_M:
                return True
        return False

    def pedestrian_wait_expired(self, now):
        since = getattr(self, "_pedestrian_wait_since", None)
        return since is not None and now - since >= PEDESTRIAN_WAIT_S

    def deduplicate_objects(self, objects, now):
        """Consume PedTracker-refined objects; raw velocity must not start this history."""
        objects = annotate_static_candidates(objects)
        evidence = getattr(self, '_static_pedestrian_evidence', None)
        if evidence is None:
            evidence = self._static_pedestrian_evidence = StaticPedestrianEvidence()
        receiver = getattr(self, 'object_receiver', None)
        stamp = float(getattr(receiver, 'observation_stamp', None) or
                      getattr(receiver, 'received', None) or now)
        if float(getattr(receiver, 'age_s', 0.)) <= .5:
            evidence.update(objects, stamp, now, runtime_rtf(self, getattr(getattr(self, '_ped_tracker', None), 'rtf', 1.)))
        else:
            evidence.update([], stamp, now, 1.)
        objects = self.standing_obstacle_objects(objects, now)
        history = getattr(self, "_ped_motion_history", None)
        if history is None:
            history = self._ped_motion_history = PedestrianMotionHistory()
        history.update(objects, now)
        return deduplicate_obstacles(objects)

    def standing_obstacle_objects(self, objects, now):
        """Read-only live revalidation of road classification and wait permission."""
        evidence = getattr(self, '_static_pedestrian_evidence', None)
        wait = getattr(self, '_standing_bypass_wait', None)
        out = []
        for obj in annotate_static_candidates(objects):
            reason = evidence.reason(obj, now) if evidence is not None else None
            policy = wait.policy(obj, now) if wait is not None else None
            if reason is None and policy is not None and policy['static']:
                reason = policy['reason']
            out.append(static_person_obstacle(obj, reason) if reason else obj)
        return out

    def person_blocks(self, obj, ego, route, now):
        """Only a person standing in our lane for 15 s may become a bypass target."""
        f, l = local_xy(obj[0], obj[1], ego)
        history = self._ped_motion_history
        # Run 433: a stander's refined speed flickered to 0.4-0.6 m/s a few times in
        # 90 s, so the standing timer never reached the wait and the car sat
        # "waiting for pedestrian" until it crept into them. Walking is >= 0.5 m/s,
        # and after twice the wait with the car stopped the person is passed anyway.
        since = getattr(self, "_pedestrian_wait_since", None)
        long_wait = since is not None and now - since >= 2. * PEDESTRIAN_WAIT_S
        still = history.max_speed(obj) < .5 and history.standing_for(obj, now) >= PEDESTRIAN_WAIT_S
        return ((still or long_wait)
                and 0. < f < 15. and (abs(l) < 2. or abs(route_lateral(route, f, l)) < 2.))

    def racing_lines(self):
        if not hasattr(self, "_racing_lines"):
            n_waypoints = len(getattr(self, "waypoints", None) or ())
            self._racing_lines = load_racing_lines(RACING_LINE_FILE, n_waypoints) if n_waypoints else None
        return self._racing_lines

    def racing_base_offsets(self, nearest, count, kind="racing"):
        """Line offsets ("racing", "safe_wide", "safe_lane") for the dense route points
        (nearest + i), or None."""
        lines = self.racing_lines()
        offsets = lines.get(kind) if lines is not None else None
        if offsets is None or count < 3:
            return None
        if not hasattr(self, "_racing_window"):
            used = np.flatnonzero(np.abs(lines["racing"]) > 1e-6)
            self._racing_window = (int(used[0]) - RACING_APPROACH_IDX, int(used[-1])) if len(used) else (1, 0)
        # Only on the approach to and inside the S-curve (never in the box section
        # before it), and only while the line ahead really leaves the lane centre.
        if not self._racing_window[0] <= int(nearest) <= self._racing_window[1]:
            return None
        base = offsets[(int(nearest) + np.arange(int(count))) % len(offsets)]
        return base if np.any(np.abs(base) > RACING_MIN_OFFSET_M) else None

    def lane2_vehicle(self, ego, dense_route, base=None, lid_pts=None):
        """A vehicle (or anything) in lane 2 alongside/behind/ahead, or LiDAR returns on
        the wide guardrail line (base), or None."""
        route = np.asarray(dense_route, dtype=float)[:, :2]
        window = (-GUARD_LANE2_BEHIND_M, max(RACING_AHEAD_M, max(0., float(ego.get("speed_kmh", 0.))) / 3.6 * 6.))
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        objects = list(self.object_receiver.snapshot()) if self.object_receiver else []
        receiver = getattr(self, "vehicle_receiver", None)
        if receiver is not None:
            try:
                objects += list(receiver.snapshot()[0])
            except Exception:
                pass
        for o in objects:
            f, l = local_xy(float(o[0]), float(o[1]), ego)
            if not window[0] - 5. <= f <= window[1] + 5.:
                continue
            j = int(np.argmin(np.hypot(route[:, 0] - f, route[:, 1] - l)))
            along = float(arc[j]) if f > route[0, 0] else f
            lateral = route_lateral(route, f, l)
            half = .5 * min(float(o[6]) if len(o) > 6 else 1., float(o[7]) if len(o) > 7 else 1.)
            if (window[0] <= along <= window[1]
                    and GUARD_LANE2_BAND_M[0] - half <= lateral <= GUARD_LANE2_BAND_M[1] + half):
                return f"{o[4]} {along:+.0f} m / {lateral:+.1f} m"
        if base is not None and lid_pts is not None and len(lid_pts) and getattr(self, "lidar", None) is not None:
            # LiDAR returns inside lane 2 itself, near enough that a 5 deg pose/scan heading
            # error cannot put the median guardrail there. Run 227: the localization heading
            # was 2-5 deg off in the right bend and the left guardrail's returns 28-45 m ahead
            # fell on the guardrail line's corridor (it reaches 0.45 m into lane 1), so the
            # car took the lane-bounded line next to the guardrail with nothing in lane 2.
            pts = np.asarray(lid_pts, dtype=float)[:, :2]
            tangent = np.gradient(route, axis=0)
            tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
            _, j = cKDTree(route).query(pts)
            rel = pts - route[j]
            lateral = rel[:, 0] * -tangent[j, 1] + rel[:, 1] * tangent[j, 0]
            along = arc[j] + np.sum(rel * tangent[j], axis=1)
            inside = ((lateral >= GUARD_LANE2_LIDAR_BAND_M[0]) & (lateral <= GUARD_LANE2_LIDAR_BAND_M[1])
                      & (along >= -GUARD_LANE2_LIDAR_BEHIND_M) & (along <= GUARD_LANE2_LIDAR_AHEAD_M))
            if np.count_nonzero(inside) >= 2:
                return f"lidar {float(np.min(along[inside])):.0f} m in lane 2"
        return None

    def guard_line_cap_mps(self, nearest, dense_route, kind):
        """Speed that can still slow to the line's guardrail cap at every point ahead."""
        lines = self.racing_lines()
        caps = lines.get("lane_vcap" if kind == "safe_lane" else "wide_vcap") if lines else None
        if caps is None:
            return None
        route = np.asarray(dense_route, dtype=float)[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        ahead = caps[(int(nearest) + np.arange(len(route))) % len(caps)] / 3.6
        limited = ahead < 900. / 3.6
        if not limited.any():
            return None
        return float(np.min(np.sqrt(ahead[limited] ** 2 + 2. * GUARD_CAP_DECEL_MPS2 * arc[limited])))

    def racing_gate(self, ego, dense_route, base, lid_pts, now):
        """(allowed, reason): the racing line only with nothing near on our side of the road."""
        route = np.asarray(dense_route, dtype=float)[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        v = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
        ahead = max(RACING_AHEAD_M, v * 6.)
        objects = list(self.object_receiver.snapshot()) if self.object_receiver else []
        receiver = getattr(self, "vehicle_receiver", None)
        if receiver is not None:
            try:
                objects += list(receiver.snapshot()[0])
            except Exception:
                pass
        reason = None
        for o in objects:
            f, l = local_xy(float(o[0]), float(o[1]), ego)
            if not -RACING_BEHIND_M <= f <= ahead + 5.:
                continue
            j = int(np.argmin(np.hypot(route[:, 0] - f, route[:, 1] - l)))
            along = float(arc[j]) if f > route[0, 0] else f
            lateral = route_lateral(route, f, l)
            half = .5 * max(float(o[6]) if len(o) > 6 else 1., float(o[7]) if len(o) > 7 else 1.)
            if (-RACING_BEHIND_M <= along <= ahead and
                    RACING_BAND_M[0] - half <= lateral <= RACING_BAND_M[1] + half):
                reason = f"{o[4]} {along:+.0f} m / {lateral:+.1f} m"
                break
        if reason is None and lid_pts is not None and self.lidar is not None:
            tangent = np.gradient(route, axis=0)
            tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
            line = route + np.column_stack((-tangent[:, 1], tangent[:, 0])) * np.asarray(base)[:len(route), None]
            hit = self.lidar.blocked(line, RACING_LIDAR_HALF_M, min(ahead, float(arc[-1])), pts=lid_pts)
            if hit is not None:
                reason = f"lidar {hit[0]:.0f} m on the racing line"
        if reason is not None:
            self._racing_hold_until = now + RACING_HOLD_S
            self._racing_gate_reason = reason
            return False, reason
        if now < getattr(self, "_racing_hold_until", -1.):
            return False, "hold: " + str(getattr(self, "_racing_gate_reason", ""))
        return True, "clear"

    def apply_zone_curve_alat(self, route_idx):
        """Follower curve_alat_max (live-reloaded ROS param): CURVE_ALAT_ZONE_MPS2 inside
        CURVE_ALAT_ZONE, the launch value elsewhere. No-op without a ROS follower."""
        if self.path_publisher is None or getattr(self, "_alat_param", "") is None:
            return
        try:
            import rospy
            if not hasattr(self, "_alat_param"):
                names = [n for n in rospy.get_param_names() if n.endswith("path_follower_node/curve_alat_max")]
                self._alat_param = names[0] if names else None
                if self._alat_param is None:
                    return
                self._alat_base = float(rospy.get_param(self._alat_param))
                self._alat_set = self._alat_base
            inside = CURVE_ALAT_ZONE[0] <= route_idx <= CURVE_ALAT_ZONE[1]
            want = max(self._alat_base, CURVE_ALAT_ZONE_MPS2) if inside else self._alat_base
            if abs(want - self._alat_set) > 1e-6:
                rospy.set_param(self._alat_param, want)
                self._alat_set = want
        except Exception:
            self._alat_param = None

    def on_road_vehicles(self, cars):
        """Drop near-stationary 'cars' that sit off every lane.

        Live 2026-10-03 roundabout: PointPillars boxed the yellow island
        structure (2.3 m from any lane centre, ~0.8 m/s jitter) as a car and
        the gap planner waited for it. Real cars were 0.1-0.5 m from a lane
        centre; moving cars (lane changes) are always kept.
        """
        lanes = getattr(self, "lanes", None)
        if not cars or lanes is None or not getattr(lanes, "n_links", 0) or not hasattr(lanes, "tree"):
            return cars
        kept = []
        for car in cars:
            velocity = lidar_vehicle_velocity(car)
            speed = float(np.linalg.norm(velocity)) if velocity is not None else float(car[5])
            distance, _ = lanes.tree.query(np.asarray(car[:2], dtype=float))
            if speed < OFF_LANE_CAR_SPEED_MPS and float(distance) > OFF_LANE_CAR_M:
                continue
            kept.append(car)
        return kept

    def contact_check(self, points, objects, ego, in_zone):
        """ped_guard.contact_guard for slow driving in the pedestrian/obstacle sections.
        LiDAR returns explained by a listed obstacle/vehicle are left to the static
        planner (its tight passes keep 0.05-0.12 m from boxes on purpose)."""
        if not in_zone or float(ego.get("speed_kmh", 0.)) > 12.:
            return None
        known = [(local_xy(o[0], o[1], ego), .5 * max(float(o[2]), float(o[6]) if len(o) > 6 else 0.,
                                                       float(o[7]) if len(o) > 7 else 0.))
                 for o in objects if o[4] != "pedestrian"]
        pts = None
        if points is not None and len(points):
            pts = np.asarray(points, dtype=float)[:, :2]
            pts = pts[(pts[:, 0] > -PED_REAR_M - 1.) & (pts[:, 0] < PED_FRONT_M + 1.) & (np.abs(pts[:, 1]) < 2.)]
            for (f, l), half in known:
                pts = pts[np.hypot(pts[:, 0] - f, pts[:, 1] - l) > half + .5]
        people = [local_xy(o[0], o[1], ego) for o in objects if o[4] == "pedestrian"]
        try:
            guard = contact_guard(pts, people)
        except Exception:
            guard = None
        now = time.monotonic()
        if guard is not None:
            guard["points_world"] = [(ego["x"] + x * math.cos(math.radians(ego["heading_deg"])) - y * math.sin(math.radians(ego["heading_deg"])),
                                      ego["y"] + x * math.sin(math.radians(ego["heading_deg"])) + y * math.cos(math.radians(ego["heading_deg"])), r)
                                     for x, y, r in guard["points"]]
            self._contact_memory = (now, guard)
            return guard
        # A person right beside the LiDAR mount drops in and out of its near-field
        # returns: remember the contact for CONTACT_MEMORY_S (as world points).
        memory = getattr(self, "_contact_memory", None)
        if memory is None or now - memory[0] > CONTACT_MEMORY_S:
            return None
        old = memory[1]
        pts_now = np.array([(*local_xy(x, y, ego), r) for x, y, r in old["points_world"]])
        try:
            guard = contact_guard(pts_now[:, :2][pts_now[:, 2] == 0.], [(p[0], p[1]) for p in pts_now if p[2] > 0.])
        except Exception:
            guard = None
        if guard is not None:
            guard["remembered"] = True
        return guard

    def refine_pedestrians(self, objects, ego):
        """Pedestrian velocity from the position history, in simulator time (ped_guard.PedTracker)."""
        tracker = getattr(self, "_ped_tracker", None)
        if tracker is None:
            tracker = self._ped_tracker = PedTracker()
        now = time.monotonic()
        clock = getattr(self, "sim_clock", None)
        if clock is not None:
            tracker.rtf = clock.rtf      # one factor for people, vehicles and the live checker
        elif getattr(self, "_ped_rtf_t", None) != now:
            self._ped_rtf_t = now
            tracker.update_rtf(ego, now)
        if not any(o[4] == "pedestrian" for o in objects):
            return objects
        stamp = float(getattr(self.object_receiver, "received", 0.) or now) if self.object_receiver else now
        if getattr(self.object_receiver, "src", "ros") == "file":   # status_node's objects.json: its own age
            stamp = now - float(getattr(self.object_receiver, "age_s", 0.) or 0.)
        extra = getattr(self.object_receiver, "_position_history", None) if self.object_receiver else None
        return tracker.update(objects, stamp, now, extra if isinstance(extra, dict) else None)

    def lost_front_people(self, objects, ego, now, candidates, points):
        """People we stop for who vanished from perception just ahead of the bumper.

        objects: every row seen this frame; candidates: ids we are stopping for
        (blocking or still waiting). Remembers such a person while they stand in our
        corridor within PED_LOST_WATCH_M of the front bumper; once they are missing,
        they hold us until raw LiDAR (vehicle-frame points, None = no scan) shows the
        corridor from the bumper to just past their spot clear for PED_LOST_CLEAR_S.
        Returns {uid: (forward_m, evidence)} of the people still holding us."""
        memory = dict(getattr(self, "_ped_front_memory", None) or {})
        seen = {int(o[11]): o for o in objects
                if o[4] == "pedestrian" and len(o) > 11 and o[11] is not None}
        for uid, o in seen.items():
            f, l = local_xy(o[0], o[1], ego)
            if (uid in candidates and f + PED_RADIUS_M > PED_FRONT_M - .1
                    and f - PED_FRONT_M <= PED_LOST_WATCH_M and abs(l) <= CORRIDOR_HALF_M):
                memory[uid] = dict(xy=(float(o[0]), float(o[1])), seen=now, clear_since=None)
            else:
                memory.pop(uid, None)
        missing = [uid for uid in memory if uid not in seen]
        if not missing:
            self._ped_front_memory = memory
            return {}
        pts = None
        if points is not None:
            pts = np.asarray(points, dtype=float)
            pts = pts[:, :2] if pts.ndim == 2 and len(pts) else np.empty((0, 2))
            # Returns explained by a listed obstacle / vehicle are not the person.
            for o in objects:
                if o[4] == "pedestrian":
                    continue
                kf, kl = local_xy(o[0], o[1], ego)
                half = .5 * max(float(o[2]), float(o[6]) if len(o) > 6 else 0., float(o[7]) if len(o) > 7 else 0.)
                pts = pts[np.hypot(pts[:, 0] - kf, pts[:, 1] - kl) > half + .5]
        holding = {}
        for uid in missing:
            entry = dict(memory[uid])
            f, l = local_xy(entry["xy"][0], entry["xy"][1], ego)
            reidentified = any(math.hypot(float(o[0]) - entry["xy"][0], float(o[1]) - entry["xy"][1]) < PED_LOST_REID_M
                               for o in seen.values())
            if reidentified or f + PED_RADIUS_M < PED_FRONT_M - .5:
                del memory[uid]
                continue
            if pts is None:
                if now - entry["seen"] > PED_LOST_BLIND_S:
                    del memory[uid]
                    continue
                holding[uid] = (f, "no LiDAR scan")
                continue
            corridor = ((pts[:, 0] >= PED_FRONT_M - .2) & (pts[:, 0] <= f + .8)
                        & (np.abs(pts[:, 1]) <= CORRIDOR_HALF_M + .2))
            spot = np.hypot(pts[:, 0] - f, pts[:, 1] - l) <= .8
            if bool((corridor | spot).any()):
                entry["clear_since"] = None
                evidence = "LiDAR returns there"
            else:
                entry["clear_since"] = entry["clear_since"] if entry["clear_since"] is not None else now
                if now - entry["clear_since"] >= PED_LOST_CLEAR_S:
                    del memory[uid]
                    continue
                evidence = "LiDAR clear %.1f s" % (now - entry["clear_since"])
            memory[uid] = entry
            holding[uid] = (f, evidence)
        self._ped_front_memory = memory
        return holding

    def pedestrian_guard(self, objects, ego, path_local, now, anticipate_crossing=False,
                         points=None, seen_objects=None, checked_standing=()):
        """ped_guard.plan with the release commit and the lost-person hold.

        points: raw LiDAR obstacle points (vehicle frame) for lost_front_people, None
        when there is no scan; seen_objects: every row seen this frame (default
        objects), so a person handled elsewhere is not taken for lost."""
        objects = self.standing_obstacle_objects(objects, now)
        static_ids = {int(o[11]) for o in objects if object_metadata(o).get('pedestrian_static_reason')}
        if static_ids:
            # The body still exists and is owned by static collision checks;
            # changing its semantic label is not a lost-person observation.
            memory = getattr(self, '_ped_front_memory', {})
            self._ped_front_memory = {uid: row for uid, row in memory.items() if uid not in static_ids}
        tracker = getattr(self, "_ped_tracker", None)
        committed = getattr(self, "_ped_pass_until", {})
        released = {k: v for k, v in getattr(self, "_ped_released", {}).items() if v > now}
        release_motion = {k: v for k, v in getattr(self, "_ped_release_motion", {}).items() if k in released}
        waiting = {k: v for k, v in getattr(self, "_ped_waiting", {}).items()
                   if now - v <= PED_WAITING_FORGET_S}
        stop_hold_ids = set(getattr(self, '_ped_stop_hold_ids', ())) & set(waiting)
        crossing_brakes = {uid: stamp for uid, stamp in
                           getattr(self, '_ped_crossing_brakes', {}).items()
                           if uid not in static_ids and 0. <= now-stamp <= PED_WAITING_FORGET_S}
        zone = self.path_publisher.zone_mps if self.path_publisher else self.zone_kmh / 3.6
        try:
            result = ped_guard_plan(objects, ego, np.asarray(path_local)[:, :2] if path_local is not None else None,
                                    float(ego.get("speed_kmh", 0.)) / 3.6, float(zone), tracker, committed, now,
                                    anticipate_crossing=anticipate_crossing, released=released,
                                    holding=set(waiting), release_motion=release_motion,
                                    stop_hold_ids=stop_hold_ids)
        except Exception as exc:   # never lose the older rules because of the guard
            if not getattr(self, "_ped_guard_err", False):
                self._ped_guard_err = True
                print(f"[ped_guard] error, falling back to the older rules: {exc!r}", flush=True)
            result = dict(kind="clear", ids=set(), pass_ids=set(), cap_mps=None, reason=None)
        # Once emergency braking starts for this exact walker, a stopping
        # approach is not a release, including while we are still decelerating.
        # Only a checked pass/arrival-after gap may authorize acceleration.
        brake_blockers = crossing_brakes.keys() & set(result.get('blocking_ids', ()))
        if brake_blockers and result['kind'] == 'yield' and result.get('stopping'):
            result = dict(result, kind='stop', cap_mps=0., stopping=True,
                          reason='person crossing: remain stopped until checked crossing gap')
        if result['kind'] == 'stop' and result.get('stopping'):
            crossing_brakes.update({uid: now for uid in result.get('blocking_ids', ())
                                    if uid in result.get('ids', ())})
        # A complete standing-person bypass owns its cleared people, so the
        # old stop latch cannot re-arm after this guard releases them. Fresh
        # movement, a new conflict, or a vanished person retains every stop.
        wait = getattr(self, '_standing_bypass_wait', None)
        result['ids'].update(static_ids)
        for obj in objects:
            policy = wait.policy(obj, now) if wait is not None else None
            if (obj[4] == 'pedestrian' and len(obj) > 11 and obj[11] in checked_standing
                    and policy is not None
                    and float(obj[5]) < .2 and obj[11] not in result.get('conflict_ids', ())
                    and model_avoidance.standing_person_gap(path_local, ego, [obj]) >= policy['gap_m']):
                result['ids'].add(int(obj[11]))
        if (result['kind'] == 'hold' and not result.get('stopping') and
                result.get('approach_cap_mps') is None and
                getattr(self.path_publisher, '_safety_provider', None) is not None):
            # The live checker owns unknown departure on the final control path.
            # A second unknown-motion "hold current speed" at standstill would
            # turn its positive cap into zero forever. Real crossing stops and
            # unowned people keep the original guard; publication rechecks the
            # same path against fresh sensors before allowing any movement.
            # An explicit stopping-readiness approach cap is independent of
            # this unknown-motion hold and must remain in the published plan.
            result = dict(result, cap_mps=float(zone),
                          reason='motion checked on selected bypass')
        committed = {k: v for k, v in committed.items() if v > now}
        for uid in result["pass_ids"]:
            committed[uid] = max(committed.get(uid, 0.), now + 4.)
        self._ped_pass_until = committed
        # Release commit: waited-for people that no longer force a stop are released
        # once; a stop for a released person (nominal re-entry) ends its commit.
        blocking = set(result.get("blocking_ids", ()))
        seen = {int(o[11]): (float(o[9]), float(o[10])) for o in objects
                if o[4] == "pedestrian" and len(o) > 11 and o[11] is not None}
        # Not seen is not gone: a person we stop for who drops out of perception just
        # ahead keeps the stop until LiDAR shows the front clear (no creep re-arm).
        lost = self.lost_front_people(objects if seen_objects is None else seen_objects, ego, now,
                                      blocking | set(waiting), points)
        if lost:
            forward = min(f for f, _ in lost.values())
            reason = "person lost %.1f m ahead: hold until LiDAR shows the front clear (%s)" % (
                max(0., forward - PED_FRONT_M), ", ".join(sorted({why for _, why in lost.values()})))
            blocking |= set(lost)
            result = dict(result, blocking_ids=set(blocking),
                          conflict_ids=set(result.get("conflict_ids", ())) | set(lost))
            if result["kind"] != "stop":
                result = dict(result, kind="stop", cap_mps=0., stopping=False, reason=reason,
                              along=max(0., forward - PED_FRONT_M - PED_RADIUS_M),
                              name=", ".join(n for n in (result.get("name"), "lost person") if n))
        for uid in blocking:
            waiting[uid] = now
            released.pop(uid, None)
        # Release only on a decision that lets us drive on (not an unknown-person
        # "hold current speed"): clear, pass, or arrive after their crossing.
        go = result["kind"] in ("clear", "pass") or (result["kind"] == "yield" and not result.get("stopping"))
        front_memory = getattr(self, "_ped_front_memory", None) or {}
        for uid in [uid for uid in waiting if go and uid not in blocking and uid in seen]:
            released[uid] = now + PED_RELEASE_S
            release_motion[uid] = seen[uid]
            del waiting[uid]
            front_memory.pop(uid, None)   # released: a later miss is not a lost blocker
        self._ped_waiting = waiting
        self._ped_crossing_brakes = {uid: now if uid in blocking else stamp
                                    for uid, stamp in crossing_brakes.items()
                                    if uid in blocking or uid in lost or
                                    (not go and uid not in seen)}
        # The planner removes a hold once that person's stop point leaves the
        # resume band or their checked gap releases. A missing person retains
        # the hold only while the existing front-memory/LiDAR rule holds them.
        self._ped_stop_hold_ids = set(result.get('stop_hold_ids', ())) | (stop_hold_ids & set(lost))
        self._ped_release_motion = {k: v for k, v in release_motion.items() if k in released}
        self._ped_released = released      # replaced, never mutated: read by the live checker thread
        self._ped_guard_log = (None if result["kind"] == "clear" and not result["ids"] and not released else
                               dict(kind=result["kind"], reason=result.get("reason"), n=len(result["ids"]), who=result.get("name"),
                                    released=sorted(int(u) for u in released), waiting=sorted(int(u) for u in waiting),
                                    lost=sorted(int(u) for u in lost),
                                    crossing_brakes=sorted(self._ped_crossing_brakes),
                                    cap_kmh=round(result["cap_mps"] * 3.6, 1) if result.get("cap_mps") is not None else None,
                                    approach_cap_kmh=(round(result['approach_cap_mps'] * 3.6, 1)
                                                      if result.get('approach_cap_mps') is not None else None),
                                    clr=round(result["clearance"], 2) if result.get("clearance") is not None else None,
                                    rtf=round(tracker.rtf, 2) if tracker else None,
                                    stopping=bool(result.get("stopping", result["kind"] == "stop")),
                                    required_decel_mps2=result.get("required_decel_mps2"),
                                    timing=[{k: (round(v, 3) if math.isfinite(v) else None)
                                             for k, v in row.items()} for row in result.get("timing", [])]))
        return result

    def apply_pedestrian_guard(self, result, yielding, ego, points, objects, now, path_local=None):
        kind = result["kind"]

        def decided_or_off_path(obj):
            if len(obj) > 11 and obj[11] is not None and int(obj[11]) in result.get("ids", ()):
                return True
            # An unowned moving person may enter later. A stationary person
            # outside the checked path corridor cannot keep an old stop latched.
            if abs(float(obj[5])) >= .2:
                return False
            f, l = local_xy(obj[0], obj[1], ego)
            return abs(route_lateral(path_local, f, l)) > CORRIDOR_HALF_M

        owns_people = bool(result.get("ids")) and all(
            decided_or_off_path(o) for o in objects if o[4] == "pedestrian")
        previous_yield = getattr(self, "last_yield", None)
        pedestrian_latch = previous_yield is None or "pedestrian" in str(previous_yield.get("name", ""))
        if owns_people and kind in ("pass", "clear", "yield", "hold"):
            # The temporal planner owns these walkers, including cap recovery.
            # Preserve independent obstacle/vehicle stops.
            self._pedestrian_hold_until = 0.
            if pedestrian_latch and (yielding is None or "pedestrian" in str(yielding.get("name", ""))):
                self._yield_stop_until = 0.
                if yielding and yielding.get("kind") == "stop":
                    yielding = None
        if kind == "pass":
            # A checked temporal pass clears only the pedestrian stop it owns.
            # Another person's approach bound still limits that checked pass.
            if owns_people:
                if pedestrian_latch:
                    self._yield_stop_until = 0.
                if yielding and "pedestrian" in str(yielding.get("name", "")) and yielding.get("kind") == "stop":
                    yielding = None
            if result.get('approach_cap_mps') is None:
                return yielding
        if kind in ("hold", "pass"):
            cap = float(result.get("cap_mps") or 0.)
            if yielding is None or (yielding.get("kind") != "stop" and cap < float(yielding.get("v_mps", 99.))):
                yielding = {"kind": "follow", "name": "pedestrian_guard:" + str(result.get("name")), "along": 0.,
                            "t": 0., "v_mps": cap, "reason": result.get("reason")}
            return yielding
        if kind not in ("yield", "stop", "reverse"):
            return yielding
        cand = {"kind": "follow" if kind == "yield" else "stop", "name": "pedestrian_guard:" + str(result.get("name")),
                "along": float(result.get("along") or 0.), "t": 0., "v_mps": float(result.get("cap_mps") or 0.),
                "reason": result.get("reason")}
        if kind == "reverse":
            self.pedestrian_reverse(ego, float(result["reverse_m"]), points, objects, now)
        if (yielding is None or cand["kind"] == "stop"
                or (yielding.get("kind") != "stop" and cand["v_mps"] < float(yielding.get("v_mps", 99.)))):
            return cand
        return yielding

    def pedestrian_reverse(self, ego, distance_m, points, objects, now):
        """Standing, and a person's walk reaches our body: back off if the rear is clear
        (LiDAR + objects), like the [reverse] escape but at once and only as far as needed."""
        publisher = getattr(self, "path_publisher", None)
        if publisher is None or not hasattr(publisher, "start_reverse") or publisher.reverse_active(ego):
            return False
        if float(ego.get("speed_kmh", 0.)) > 1.5:
            return False
        spot = (round(float(ego["x"]) / 10.), round(float(ego["y"]) / 10.))
        tries = getattr(self, "_ped_reverse_tries", {})
        if tries.get(spot, 0) >= 4:
            self._reverse_veto = "pedestrian reverse tries exhausted"
            return False
        touching = getattr(self, "_contact", None)
        if touching and touching["block_reverse"]:
            self._reverse_veto = "contact guard: %s within %.2f m" % ("/".join(touching["where"]), touching["gap"])
            return False
        reach = PED_REAR_M + distance_m + 1.5
        if points is not None and len(points):
            pts = np.asarray(points, dtype=float)[:, :2]
            rear = (pts[:, 0] < -PED_REAR_M - .2) & (pts[:, 0] > -reach) & (np.abs(pts[:, 1]) < 1.3)
            if rear.any():
                near = pts[rear][np.argmax(pts[rear][:, 0])]
                self._reverse_veto = "pedestrian reverse: rear point at %.1f,%.1f" % (near[0], near[1])
                return False
        for obj in objects:
            f, l = local_xy(obj[0], obj[1], ego)
            if -reach - 2.5 < f < -PED_REAR_M + .3 and abs(l) < 2.:
                self._reverse_veto = "pedestrian reverse: rear %s at %.1f,%.1f" % (obj[4], f, l)
                return False
        tries[spot] = tries.get(spot, 0) + 1
        self._ped_reverse_tries = tries
        road_heading = None
        route_xy = getattr(self, "waypoint_xy", None)
        if route_xy is not None and len(route_xy) > 2:
            route_xy = np.asarray(route_xy, dtype=float)[:, :2]
            j = int(np.argmin(np.hypot(route_xy[:, 0] - ego["x"], route_xy[:, 1] - ego["y"])))
            a, b = route_xy[max(j - 2, 0)], route_xy[min(j + 2, len(route_xy) - 1)]
            road_heading = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        publisher.start_reverse(ego, timeout_s=10., road_heading_deg=road_heading, distance_m=distance_m,
                                static_recovery=False)
        print(f"[reverse] pedestrian {tries[spot]}/4: back off {distance_m:.1f} m, rear clear", flush=True)
        return True

    def reverse_escape_check(self, ego, blocked, clear_m, points, now, decision_plan=None, swing_check=False,
                             static_recovery=False):
        """Blocked close ahead and parked: start a reverse escape when the rear is LiDAR-clear.
        points: obstacle points in the vehicle frame (x forward, y left).
        swing_check (tunnel): also veto / abort a reverse whose turn to the road heading
        swings the body (its front corners 3.8 m ahead of the rear axle) into a point."""
        publisher = getattr(self, "path_publisher", None)
        if publisher is None or not hasattr(publisher, "start_reverse"):
            return False
        recovery_distance = REVERSE_ESCAPE_M
        static_backoff = None
        if publisher.reverse_active(ego):
            # Forward proofs may need several scans before releasing R. A
            # ready or stale forward plan never skips current rear monitoring.
            # Abort when something appears behind while reversing (true rear bumper is
            # 0.79 m behind the rear axle, judge.py; -2.0 missed a walker right behind it),
            # or something now touches the side we would drag along (auto run 268).
            if points is not None and len(points):
                pts = np.asarray(points, dtype=float)[:, :2]
                behind = (((pts[:, 0] < -PED_REAR_M + .1) & (np.abs(pts[:, 1]) < 1.1))
                          | ((pts[:, 0] < -2.0) & (np.abs(pts[:, 1]) < 1.2))) & (pts[:, 0] > -4.5)
                swing = np.empty((0, 2))
                if swing_check:
                    # The rest of the reverse turns the body to the road heading: its front
                    # corners swing sideways (ego_body.reverse_sweep_hits).
                    state = getattr(publisher, "reverse_state", None) or {}
                    remaining = (float(state.get("distance", REVERSE_ESCAPE_M)) -
                                 math.hypot(float(ego["x"]) - float(state.get("x", ego["x"])),
                                            float(ego["y"]) - float(state.get("y", ego["y"]))))
                    swing = reverse_swing_points(pts, ego, state.get("heading", ego["heading_deg"]), remaining)
                publisher.reverse_abort = bool(behind.any() or len(swing))
                if len(swing):
                    self._reverse_veto = "reverse swing abort: point at %.1f,%.1f" % tuple(swing[0])
            touching = getattr(self, "_contact", None)
            if touching and touching["block_reverse"]:
                publisher.reverse_abort = True
            return True
        if decision_plan is not None:
            # A stale stop or a different path cannot start a new reverse.
            if not decision_plan.fresh(now):
                self._reverse_wait_since = None
                return False
            clear_m = decision_plan.forward_clear_m
            blocked = bool(decision_plan.emergency or decision_plan.validation.get('blocked'))
            if decision_plan.forward_ready(ego, now):
                self._reverse_wait_since = None
                self._reverse_veto = None
                return False
        if static_recovery:
            self._reverse_veto = 'normal-road static recovery requires a checked forward bypass'
            record = (((getattr(self, '_model_avoidance_log', None) or {}).get(
                'bounded_pass') or {}).get('replay'))
            if (not blocked or abs(float(ego.get('speed_kmh', 0.))) > .5
                    or not isinstance(record, dict) or clear_m is None
                    or clear_m > REVERSE_TRIGGER_M):
                self._reverse_wait_since = None
                return False
            if getattr(self, '_reverse_wait_since', None) is None:
                self._reverse_wait_since = now
                return False
            if now-self._reverse_wait_since < REVERSE_AFTER_S:
                return False
            try:
                from static_lane_pass import checked_static_backoff
                from tunnel_runtime import corridor_at_pose
                import model_avoidance as ma
                observed = record['ego']
                current, route, fresh_points, frame = current_static_guard_frame(
                    self, observed, np.asarray(record['route']))
                if (not frame['valid'] or current is None or
                        abs(float(current['speed_kmh'])) > .5 or
                        math.hypot(current['x']-observed['x'], current['y']-observed['y']) > .2
                        or time.monotonic()-float(record['now']) > 1.5):
                    return False
                receiver = self.object_receiver
                objects = receiver.snapshot()
                if (not 0. <= float(getattr(receiver, 'age_s', math.inf)) <= .3
                        or float(getattr(receiver, 'observation_stamp', 0.)) <= 0.):
                    return False
                static, dynamic = ma.split_static(objects)
                if dynamic or any(o[4] in ('pedestrian', 'npc') for o in objects):
                    return False
                static = ma.road_static_objects(static, current, fresh_points,
                                                allow_producer_geometry=True)
                value = record['legal']
                legal = LaneCorridor(np.asarray(value['route']), np.asarray(value['left']),
                    np.asarray(value['right']), zones=value['zones'],
                    intersection_mask=np.asarray(value['intersection_mask']),
                    body_shift_m=REAR_AXLE_SHIFT_M)
                legal = corridor_at_pose(legal, observed, current)
                source = dict(record, ego=dict(current), route=route.tolist(),
                    points=fresh_points.tolist(), objects=objects, static=static,
                    ready=True, legal=dict(route=legal.route.tolist(), left=legal.left.tolist(),
                        right=legal.right.tolist(), zones=legal.zones,
                        intersection_mask=legal.intersection_mask.tolist()))
                static_backoff = checked_static_backoff(self.lanes, source)
                if static_backoff is None:
                    return False
                # Feasibility search can span a new scan. The actual reverse
                # receives a separate, current measured rear/swing check.
                latest, latest_route, fresh_points, frame = current_static_guard_frame(self, current, route)
                if (not frame['valid'] or latest is None or
                        abs(float(latest['speed_kmh'])) > .5 or
                        math.hypot(latest['x']-current['x'], latest['y']-current['y']) > .1 or
                        abs((latest['heading_deg']-current['heading_deg']+180.) % 360.-180.) > .5):
                    return False
                from tunnel_runtime import swept_clearance
                if swept_clearance(latest_route, fresh_points, 7., .25,
                        lateral_margin=.25, body_shift_m=REAR_AXLE_SHIFT_M) > REVERSE_TRIGGER_M:
                    return False
                ego, points = latest, fresh_points
                recovery_distance = static_backoff['distance_m']
                self._static_backoff_proof = static_backoff
            except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
                return False
        speed = float(ego.get("speed_kmh", 0.))
        if not blocked or speed > 3. or clear_m is None or clear_m > REVERSE_TRIGGER_M:
            # (> 3 km/h, not > 1: a stalled car inching 1-3 km/h reset the timer
            # forever and the second reverse never came - auto run 78)
            self._reverse_wait_since = None
            self._reverse_veto = None
            return False
        if getattr(self, "_reverse_wait_since", None) is None:
            self._reverse_wait_since = now
            return False
        if now - self._reverse_wait_since < REVERSE_AFTER_S:
            return False
        spot = (round(float(ego["x"]) / 10.), round(float(ego["y"]) / 10.))
        tries = getattr(self, "_reverse_tries", {})
        # Why a due reverse did not start, for the decision log (auto run 127 sat
        # 60 s blocked 3 m short of a standing person with no reverse and no clue).
        if tries.get(spot, 0) >= REVERSE_MAX_TRIES:
            # Exhausted is not forever: run 502 stood 150 s boxed at 23 deg beside a
            # parked car with "tries exhausted". After REVERSE_REFILL_S of standstill
            # one more escape is granted (the reverse aligns to the road heading).
            refills = getattr(self, "_reverse_refills", {})
            if (now - self._reverse_wait_since >= REVERSE_REFILL_S
                    and refills.get(spot, 0) < REVERSE_MAX_REFILLS):
                refills[spot] = refills.get(spot, 0) + 1
                self._reverse_refills = refills
                tries[spot] = REVERSE_MAX_TRIES - 1
                self._reverse_tries = tries
                print(f"[reverse] budget refill {refills[spot]}/{REVERSE_MAX_REFILLS} after {now - self._reverse_wait_since:.0f} s standstill", flush=True)
            else:
                self._reverse_veto = "tries exhausted here"
                return False
        touching = getattr(self, "_contact", None)
        if touching and touching["block_reverse"]:
            self._reverse_veto = "contact guard: %s within %.2f m" % ("/".join(touching["where"]), touching["gap"])
            return False
        road_heading = None
        route_xy = getattr(self, "waypoint_xy", None)
        if route_xy is not None and len(route_xy) > 2:
            route_xy = np.asarray(route_xy, dtype=float)[:, :2]
            j = int(np.argmin(np.hypot(route_xy[:, 0] - ego["x"], route_xy[:, 1] - ego["y"])))
            a, b = route_xy[max(j - 2, 0)], route_xy[min(j + 2, len(route_xy) - 1)]
            road_heading = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        if static_backoff is not None:
            road_heading = static_backoff['heading_deg']
        if points is not None and len(points):
            pts = np.asarray(points, dtype=float)[:, :2]
            # Directly behind the true rear bumper (0.79 m behind the rear axle; -2.0 let a
            # reverse back toward a walker right behind us, auto run 268), and the old band.
            rear = (((pts[:, 0] < -PED_REAR_M + .1) & (np.abs(pts[:, 1]) < 1.1))
                    | ((pts[:, 0] < -2.0) & (np.abs(pts[:, 1]) < 1.3))) & (pts[:, 0] > -2.3 - REVERSE_ESCAPE_M - 2.2)
            if rear.any():
                near = pts[rear][np.argmax(pts[rear][:, 0])]
                self._reverse_veto = "rear point at %.1f,%.1f" % (near[0], near[1])
                return False
            # Turning the body to the road heading while backing swings its front corners
            # 3.8 m ahead of the rear axle sideways (follow-20261010T200235Z 05:09:05: a
            # drum beside the front-left corner was touched; ego_body.reverse_sweep_hits).
            swing = (reverse_swing_points(pts, ego, road_heading if road_heading is not None
                                          else ego["heading_deg"], recovery_distance)
                     if swing_check else ())
            if len(swing):
                self._reverse_veto = "reverse swing point at %.1f,%.1f" % tuple(swing[0])
                return False
        objects = self.object_receiver.snapshot() if getattr(self, "object_receiver", None) else []
        for obj in objects:
            f, l = local_xy(obj[0], obj[1], ego)
            if -2.3 - REVERSE_ESCAPE_M - 4. < f < 0. and abs(l) < 2.:
                self._reverse_veto = "rear %s at %.1f,%.1f" % (obj[4], f, l)
                return False
        self._reverse_veto = None
        tries[spot] = tries.get(spot, 0) + 1
        self._reverse_tries = tries
        self._reverse_wait_since = None
        publisher.start_reverse(ego, road_heading_deg=road_heading,
            distance_m=recovery_distance, static_recovery=static_backoff is not None)
        print(f"[reverse] escape {tries[spot]}/{REVERSE_MAX_TRIES}: blocked {clear_m:.1f} m ahead, rear clear", flush=True)
        return True

    def surround_cars(self, ego):
        """All LiDAR-tracked cars (rear included) in the ego frame, for training data."""
        receiver = getattr(self, "vehicle_receiver", None)
        if receiver is None:
            return None
        try:
            cars, ready, _ = receiver.snapshot()
        except Exception:
            return None
        h = math.radians(ego.get("heading_deg", 0.))
        c, s = math.cos(h), math.sin(h)
        out = []
        for car in cars:
            dx, dy = car[0] - ego["x"], car[1] - ego["y"]
            vx, vy = (car[9], car[10]) if len(car) > 10 else (0., 0.)
            out.append(dict(id=car[11] if len(car) > 11 else None,
                            along=round(c*dx + s*dy, 2), lat=round(-s*dx + c*dy, 2),
                            v_along=round(c*vx + s*vy, 2), v_lat=round(-s*vx + c*vy, 2),
                            len=round(float(car[6]), 2), wid=round(float(car[7]), 2),
                            yaw=round(float(car[8]) - ego.get("heading_deg", 0.), 1),
                            v_ok=bool(car[14]) if len(car) > 14 else False))
        return dict(ready=bool(ready), list=out)

    def lead_follow_step(self, ego, now, objects, lidar_pts, path_local, use_lidar, in_junction,
                         lidar_cars_in_objects=False, decision_plan=None):
        """Lead vehicle on the path we drive -> speed cap, wait timer, bypass request.

        objects: the vehicle objects of this frame (fused cars; detector cars inside the
        junction regions). Outside them the LiDAR detector's cars are added when its
        stream is ready (it runs only where lidar_vehicle_detection enables it)."""
        if not LEAD_FOLLOW or path_local is None or len(path_local) < 3:
            self._lead_state = None
            return None
        if decision_plan is not None:
            ego, path_local = decision_plan.validation_ego, decision_plan.path
        tracker_name = '_tunnel_lead_tracker' if decision_plan is not None else '_lead_tracker'
        tracker = getattr(self, tracker_name, None)
        if tracker is None:
            tracker = LeadTracker()
            setattr(self, tracker_name, tracker)
        if not hasattr(self, '_lead_bias'):
            self._lead_bias = HeadingBias()
        path = np.asarray(path_local, dtype=float)[:, :2]
        # A tunnel bypass already includes the checked pose correction. Rotating
        # it a second time creates a different collision corridor.
        bias = 0. if decision_plan is not None else self._lead_bias.update(now, path_heading_rad(path))
        ego_mps = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
        lidar = None
        if use_lidar and lidar_pts is not None and len(lidar_pts):
            found = [lidar_lead(path, lidar_pts)]
            if abs(bias) > math.radians(.5):
                # Heading bias: the map path drawn in the vehicle frame swings off the
                # road the car physically drives (run 264: 3 deg). Check both.
                found.append(lidar_lead(body_aligned_path(path, bias), lidar_pts))
            found = [d for d in found if d is not None]
            lidar = min(found, key=lambda d: d["dist_m"]) if found else None
        cars = [o for o in (objects or ()) if o[4] == "npc"]
        receiver = getattr(self, "vehicle_receiver", None)
        if receiver is not None and not in_junction and not lidar_cars_in_objects:
            try:
                detected, ready, _ = receiver.snapshot()
                if ready:
                    # Tracked cars only (velocity measured over >= 3 scans): a one-scan
                    # PointPillars box must not stop us. Used for the lead, not by the
                    # gap planner (that stays junction-only).
                    cars += [c for c in self.on_road_vehicles(detected) if len(c) > 14 and c[14]]
            except Exception:
                pass
        # Callers pass measured vehicle rows; the lead speed meets ego speed and
        # braking here, so it is converted once to simulator units.
        rtf = runtime_rtf(self)
        cars = to_sim_objects(cars, rtf)
        leads = object_leads(cars, ego, path)
        lidar_obj = getattr(self, "lidar", None)
        scan = (decision_plan.sensor_stamp if decision_plan is not None else
                getattr(lidar_obj, "_scan_stamp", None) or getattr(lidar_obj, "received", None))
        if decision_plan is not None and lidar is None and not leads:
            held = getattr(self, '_tunnel_lead_world', None)
            if held is not None:
                held_local = local_xy(*held, ego)
                s, lateral, _ = path_coordinates(path, [held_local])
                if not len(s) or not np.isfinite(s[0]) or abs(lateral[0]) > 1.296:
                    tracker.reset()
                    self._tunnel_lead_world = None
                    self._lead_wait_since = None
        state = tracker.update(now, ego_mps, lidar, leads, scan_stamp=scan, time_scale=rtf)
        if decision_plan is not None and state and not state['held']:
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
            point = [np.interp(state['dist_m'], arc, path[:, i]) for i in (0, 1)]
            self._tunnel_lead_world = RosPathPublisher.global_xy(ego, *point)
        cap = lead_cap(state, ego_mps)
        # Waiting behind a standing lead: queue or broken down?
        standing = bool(state and cap is not None and state["confirmed"]
                        and (not state["known"] or state["v_lead"] < STOPPED_MPS)
                        and state["gap_m"] < STANDSTILL_GAP_M + 8.)
        wait_s, limit_s, queue = 0., None, None
        if standing and ego_mps < .3:
            if getattr(self, "_lead_wait_since", None) is None:
                self._lead_wait_since = now
            wait_s = now - self._lead_wait_since
        elif ego_mps > 1. or not state or (state["known"] and state["v_lead"] > MOVING_MPS):
            self._lead_wait_since = None
        elif getattr(self, "_lead_wait_since", None) is not None:
            wait_s = now - self._lead_wait_since
        if wait_s > 0. and state is not None:
            lead_station = float(self.lanes.route_progress(ego)) + float(state["dist_m"])
            lines = np.asarray(getattr(self.lanes, "stop_line_stations", ()), dtype=float)
            ahead = lines[lines >= lead_station - 5.] - lead_station
            queue = bool(len(ahead) and float(ahead.min()) <= LEAD_QUEUE_ZONE_M)
            limit_s = LEAD_QUEUE_WAIT_S if queue else LEAD_STALLED_WAIT_S
            if wait_s >= limit_s and getattr(self, "_lead_bypass", None) is None:
                self._lead_bypass = dict(station_end=lead_station + 12., until=now + 90.,
                                         since=now, queue=queue)
                print(f"[lead] standing {wait_s:.0f} s ({'queue' if queue else 'not at a junction'}): "
                      f"plan around it", flush=True)
        self._lead_state = dict(
            gap_m=round(state["gap_m"], 1) if state else None,
            v_lead_kmh=round(state["v_lead"] * 3.6, 1) if state and state["known"] else None,
            source=state["source"] if state else None, held=bool(state and state["held"]),
            confirmed=bool(state and state["confirmed"]),
            cap_kmh=round(cap * 3.6, 1) if cap is not None else None,
            lidar_m=round(lidar["dist_m"], 1) if lidar else None,
            lidar_span_m=lidar["span_m"] if lidar else None,
            bias_deg=round(math.degrees(bias), 1), wait_s=round(wait_s, 1),
            wait_limit_s=limit_s, queue=queue,
            bypass=getattr(self, "_lead_bypass", None) is not None, rtf=round(rtf, 3))
        emergency = needs_emergency_stop(state, ego_mps) if cap is not None else False
        self._lead_state["emergency"] = emergency
        if decision_plan is not None:
            self._lead_state.update(plan_id=decision_plan.plan_id, sensor_stamp=decision_plan.sensor_stamp,
                                    path_digest=decision_plan.summary()['path_digest'])
        return dict(cap_mps=cap, state=state, emergency=emergency)

    def lead_bypass_active(self, ego, now):
        """A standing lead waited out (lead_follow_step): until we are past it."""
        bypass = getattr(self, "_lead_bypass", None)
        if bypass is None:
            return False
        if now > bypass["until"] or float(self.lanes.route_progress(ego)) > bypass["station_end"]:
            self._lead_bypass = None
            self._lead_wait_since = None
            return False
        return True

    def intersection_objects(self, objects, ego):
        """Use measured LiDAR cars in the selected junction, keeping other detections."""
        self._intersection_vehicle_status = None
        regions = getattr(self, "intersection_regions", ())
        if not regions:
            return objects
        station = self.lanes.route_progress(ego)
        region = active_intersection(station, regions)
        memory = getattr(self, "junction_tracks", None)
        if memory is None:
            memory = self.junction_tracks = JunctionTrackMemory()
        ring_motion = getattr(self, "ring_motion", None)
        if ring_motion is None:
            ring_motion = self.ring_motion = RingMotionTracker()
        if region is None:
            memory.reset()
            ring_motion.reset()
            return objects
        receiver = getattr(self, "vehicle_receiver", None)
        cars, ready, status = receiver.snapshot() if receiver is not None else ([], False, {})
        cars = self.on_road_vehicles(cars)
        planner = getattr(self, "gap_planner", None)
        # Robust ring-car speed from the measured positions at their detector stamps
        # (the detector's own 0.3 s slopes read one ring car at 0-11 m/s sim in 2 s).
        stamps = {}
        for item in (status.get("vehicles") or ()) if isinstance(status, dict) else ():
            try:
                if not float(item.get("prediction_age_s") or 0.) > 0.:
                    stamps[int(item["uid"])] = float(item["stamp"])
            except (KeyError, TypeError, ValueError):
                continue
        if getattr(self, 'stop_line_controller', None) is None:
            from junction_vehicle_motion import source_motion_rows
            cars, _ = source_motion_rows(cars, ring_motion, stamps)
        else:
            cars = ring_motion.update(cars, stamps)
        if planner is not None:
            planner.speed_ratio = dict(ring_motion.ratio)
        # Tracker dropouts (0.9-2.6 s, new id, no velocity for ~0.5 s; run 222): keep
        # lost cars as predicted ghosts and give a re-found car its old velocity.
        forecasts = [car for car in cars if len(car) > 15 and car[15] > 0.]
        measured = [car for car in cars if len(car) <= 15 or car[15] <= 0.]
        cars = memory.update(measured, time.monotonic(), (ego["x"], ego["y"]),
                             getattr(planner, "map_predictor", None))
        represented = {car[11] for car in cars}
        cars += [car for car in forecasts if car[11] not in represented]
        # A new track has no measured velocity yet. Whether it matters (ring arc to our
        # conflict point) is the entry planner's call (ring_entry.unknown_relevant): new
        # tracks at the spawn point 52 m up the ring are not a reason to wait.
        motion_ready = all(len(car) > 14 and car[14] for car in cars)
        self._intersection_vehicle_status = dict(status, active_region=region,
            detector_ready=bool(ready), motion_ready=bool(motion_ready),
            ready=bool(ready), car_count=len(cars), station_m=station,
            ghost_ids=sorted(memory.ghost_ids), inherited_ids=sorted(memory.inherited_ids),
            unknown_ids=sorted(memory.unknown_ids),
            fitted_mps={str(uid): round(v, 2) for uid, v in ring_motion.fitted.items()})
        if not ready and not memory.ghost_ids:
            return objects
        # Match one camera car to one LiDAR car. Do not remove people, generic
        # obstacles, or a neighbouring vehicle merely because it is nearby.
        merged, unused = [], set(range(len(cars)))
        for obj in objects:
            is_vehicle = obj[4] == "npc" or obj[3] == "fusion_parked_vehicle"
            match = None
            if is_vehicle and unused:
                candidate = min(unused, key=lambda j: math.hypot(obj[0]-cars[j][0], obj[1]-cars[j][1]))
                if ((len(cars[candidate]) <= 15 or cars[candidate][15] <= 0.) and
                        math.hypot(obj[0]-cars[candidate][0], obj[1]-cars[candidate][1]) < 2.):
                    match = candidate
            if match is None:
                merged.append(obj)
            else:
                unused.remove(match)
        return merged + list(cars)

    def model_camera_fresh(self, max_age_s=.5):
        """Require recent images in every model view before granting entry."""
        receiver = self.receiver
        if hasattr(receiver, "stale_views"):
            return not receiver.stale_views(max_age_s)
        if not hasattr(receiver, "counts") or not hasattr(receiver, "lock"):
            return False
        now = time.monotonic()
        with receiver.lock:
            return all(receiver.counts.get(view) and
                       0. <= now - receiver.counts[view][-1] <= max_age_s
                       for view, _ in VIEWS)

    def refresh_intersection_objects(self, objects, planned_ego):
        """Refresh late traffic evidence while retaining the fixed planned world path."""
        controller = getattr(self, 'stop_line_controller', None)
        previous = getattr(self, '_intersection_vehicle_status', None)
        receiver = getattr(self, 'vehicle_receiver', None)
        if controller is None or not controller.allow_empty_entry or not previous or receiver is None:
            return objects
        _, _, latest = receiver.snapshot()
        try:
            newest, old = float(latest.get('stamp') or 0.), float(previous.get('stamp') or 0.)
        except (TypeError, ValueError):
            return objects
        if not math.isfinite(newest) or newest <= old:
            return objects
        pose = fresh_entry_pose(self, planned_ego)
        if pose is None:
            return objects
        # Re-run the existing ring-motion/ghost policy on the newer sample;
        # retained people and camera bodies keep their earlier safety evidence.
        original = [o for o in objects if o[3] != 'lidar_car']
        return self.intersection_objects(original, pose)

    def apply_stop_line(self, ego, now, yielding, emergency, fresh,
                        model_actions=None, model_probabilities=None, model_fresh=False,
                        entry_pose=None, traffic_stop_only=False, committed_traffic_check=None):
        self._intersection_sensor_stop = False
        controller = self.stop_line_controller
        if controller is None:
            return emergency, None
        if controller.allow_empty_entry and entry_pose is not None:
            # This same decision already checked continuity to the original
            # planning pose. Continue from that measured pose when refreshing
            # at application, rather than rejecting a current sample solely
            # because model/planning work outlived the original 0.35 s frame.
            # The fixed world path and current sensor/body proofs are unchanged.
            ego = entry_pose
        # Planning can take longer than one sensor period. Use the actual
        # application clock and preserve the observed pose's receive time.
        applied_now = time.monotonic() if "received" in ego else now
        try:
            observation_age = (applied_now - float(ego["received"]) if "received" in ego
                               else float(ego.get("age_s", 0.)))
        except (TypeError, ValueError):
            observation_age = float("nan")
        clear = entry_is_clear(yielding, emergency, fresh)
        entry = None
        if model_actions is not None:
            vehicles = getattr(self, "_intersection_vehicle_status", None) or {}
            entry = intersection_entry_decision(model_actions, model_probabilities,
                lidar_ready=bool(vehicles.get("detector_ready", vehicles.get("ready"))), yielding=yielding,
                emergency=emergency, fresh=fresh, model_fresh=model_fresh)
            clear = entry["clear"]
            # Model classification is advisory. Loss of the measured LiDAR
            # traffic stream remains a real perception fault during entry.
            region = vehicles.get("active_region") or {}
            # Once committed to the entry the detector's readiness no longer stops us
            # (user rule 2026-10-10: following only; the remembered tracks and the raw
            # LiDAR body sweep still brake us behind what is on our path).
            committed = bool(getattr(getattr(self, "gap_planner", None), "junction_committed", False))
            if vehicles and not vehicles.get("detector_ready"):
                if getattr(self, "_junction_not_ready_since", None) is None:
                    self._junction_not_ready_since = applied_now
            else:
                self._junction_not_ready_since = None
            bridged = committed
            if (vehicles and not vehicles.get("detector_ready") and not bridged and
                    vehicles["station_m"] >= region["stop_station_m"]-controller.FRONT_M-controller.BUFFER_M-1.):
                self._intersection_sensor_stop = True
                emergency = True
        empty_requested = bool(controller.allow_empty_entry and yielding
            and yielding.get("reason") == EMPTY_ENTRY_REASON)
        if (controller.allow_empty_entry and not empty_requested
                and getattr(self, '_intersection_vehicle_status', None)):
            pose = fresh_entry_pose(self, ego)
            if pose is not None:
                ego = pose
                applied_now = time.monotonic()
                observation_age = max(float(ego['age_s']), applied_now-float(ego['received']))
        empty_clear = bool(empty_requested and clear
            and empty_intersection_observation(getattr(self, "_intersection_vehicle_status", None)))
        context = getattr(getattr(self, 'gap_planner', None), 'junction', None) or {}
        proof = None
        if empty_clear:
            path_geometry = certified_path_geometry(self.lanes,
                getattr(self, '_empty_entry_world_path', None))
            probe = fresh_entry_pose(self, ego)
            empty_clear = bool(path_geometry and probe is not None
                and certified_pose_geometry(self.lanes, probe))
            if probe is not None:
                # Refresh this method's controller inputs together. The world
                # publication remains fixed in the original planning frame.
                ego = probe
                applied_now = time.monotonic()
                observation_age = max(float(ego['age_s']), applied_now-float(ego['received']))
            front_gap = float(context.get('line_station_m', math.inf))-intersection_front_station(ego, self.lanes.route_progress)
            empty_clear = empty_clear and math.isfinite(front_gap)
            if empty_clear and front_gap <= ROLL_CHECK_FRONT_GAP_M and abs(float(ego.get('speed_kmh', 0.)))/3.6 <= ROLL_MPS:
                # Recheck at commit time, after planning: neither the old scan's
                # grace period nor a newer traffic sample may bypass the proof.
                proof = {}
                raw = entry_scan_points(self, ego, self._intersection_vehicle_status, applied_now, proof=proof)
                empty_clear = bool(raw is not None and raw_entry_clearance(self.lanes, ego, raw,
                    getattr(self, '_empty_entry_world_path', None)) >=
                    braking_reserve(abs(float(ego.get('speed_kmh', 0.)))/3.6, MAX_ENTRY_SOURCE_AGE_S)
                    and entry_proof_fresh(proof, time.monotonic()))
            if probe is not None:
                applied_now = time.monotonic()
                observation_age = max(float(ego['age_s']), applied_now-float(ego['received']))
                empty_clear = (empty_clear and observation_age <= MAX_POSE_AGE_S
                    and (proof is None or entry_proof_fresh(proof, applied_now)))
        if empty_requested and not empty_clear:
            # A failed current proof cannot reuse the old empty decision to
            # release through the legacy hold-zero branch. A later measured
            # normal gap decision may still release the mandatory stop.
            clear = False
        decision = controller.update(
            self.lanes.route_progress(ego), float(ego.get("speed_kmh", 0.)) / 3.6,
            applied_now, clear=clear,
            observation_age_s=observation_age, time_scale=runtime_rtf(self),
            empty_entry_clear=empty_clear,
            empty_entry_prepare=empty_intersection_observation(
                getattr(self, '_intersection_vehicle_status', None)),
            empty_front_station=(intersection_front_station(ego, self.lanes.route_progress)
                                 if controller.allow_empty_entry else None),
            empty_entry_clearance_m=(getattr(getattr(self, 'gap_planner', None), 'junction', None)
                                    or {}).get('empty_entry_clearance_m', 0.))
        recovery = getattr(self, 'junction_recovery', None)
        recovery_context = (recovery.context(ego, controller, self.lanes,
            self.intersection_regions, applied_now, observe_pose=False)
            if recovery is not None and context.get('recovery_only') else None)
        if (decision is not None and recovery_context is not None and fresh
                and context.get('ready') is True and not emergency):
            # The whole measured body already occupied this junction at boot.
            # Keep controller bookkeeping, but do not ask that body to wait at
            # a line behind it. Only the full live guard can approve movement;
            # this is deliberately not a gap_checked/GO permission.
            decision = dict(decision, state='RECOVERY_FOLLOW', stop=False,
                cap_mps=INTERSECTION_ENTRY_MPS, gap_checked=False,
                reason='observed inside junction requires full live checks')
            if entry is not None:
                entry = dict(entry, clear=False, authority='live_recovery',
                             reason=decision['reason'])
        planner = getattr(self, "gap_planner", None)
        if (decision is not None and decision.get("state") == "ENTER" and planner is not None
                and getattr(self, "_intersection_vehicle_status", None) is not None):
            # Released at the line: the entry is committed (following only) until the
            # junction region ends; no later rule may send us back to waiting.
            if not controller.yield_only:
                planner.latch_junction_commit()
            if (controller.released and planner.junction_committed
                    and traffic_stop_only and callable(committed_traffic_check) and yielding
                    and yielding.get('source') == 'traffic'):
                # A publication may have committed while this GUI calculation
                # still held an older gap-wait result. Recompute following;
                # do not carry that pre-entry stop into the accepted entry.
                following = committed_traffic_check()
                if following and following.get('kind') in ('enter', 'follow'):
                    yielding.clear()
                    yielding.update(following)
                    self.yield_cap_mps = max(0., float(following['v_mps']))
                    emergency = False
        if decision is not None:
            if entry is not None:
                decision["entry_decision"] = entry
                if decision["state"] == "WAIT_CLEAR" and not entry["clear"]:
                    decision["reason"] = entry["reason"]
            cap = decision["cap_mps"]
            if decision.get("state") == "ENTER":
                # Released at the line: leave the junction at zone speed; the
                # gap planner, curvature and leader headway still limit it.
                cap = max(cap, self.zone_kmh / 3.6)
            self.yield_cap_mps = cap if self.yield_cap_mps is None else min(self.yield_cap_mps, cap)
            emergency = emergency or decision["stop"]
        return emergency, decision

    def update_ego(self):
        # Live ROS snapshots supply all ego/path fields below. Reading status.json
        # here makes the control loop wait on the archival recording volume.
        status = dict(self.latest_status)
        ego, history, ego_hz = self.ego_receiver.snapshot(int(self.config["ego_steps"]))
        if ego:
            delta = self.waypoint_xy - np.asarray((ego["x"], ego["y"]), dtype=np.float64)
            nearest = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
            distance = float(np.linalg.norm(delta[nearest]))
            status["time"] = time.time()
            status["ego_hz"] = ego_hz
            status["ego"] = ego
            status["path"] = {
                "nearest_idx": nearest,
                "distance_m": distance,
                "progress_pct": 100.0 * self.route_cumulative[nearest] / max(self.route_cumulative[-1], 1e-6),
            }
        self.latest_status = status
        fresh = bool(ego and float(status.get("ego_hz", 0)) > 0 and float(ego.get("age_s", 999)) < 1.0)
        if not fresh:
            self.input_error = "MORAI ROS/Ego disconnected - stale coordinates blocked"
            return status
        self.input_error = None
        self.ego_history = history
        return status

    def ego_tensor(self, current):
        history = list(self.ego_history)
        if not history:
            raise RuntimeError("Ego 수신 대기")
        history = [history[0]] * max(0, int(self.config["ego_steps"]) - len(history)) + history
        values = []
        for item in history[-int(self.config["ego_steps"]):]:
            forward, left = local_xy(float(item["x"]), float(item["y"]), current)
            values.append([forward, left, float(item.get("speed_kmh", 0)) / 3.6,
                           float(item.get("accel", 0)), float(item.get("brake", 0)),
                           float(item.get("wheel_angle", 0))])
        return torch.tensor(values, dtype=torch.float32).unsqueeze(0)

    def route_tensor(self, current, status):
        nearest = int((status.get("path") or {}).get("nearest_idx", 0))
        count = int(self.config["route_points"])
        n_wp = len(self.waypoints)

        def sample(stride, sample_count=count):
            # global_path.json is a closed loop (first/last waypoint coincide, see
            # __init__). Wrap past the last index instead of clamping to it, so the
            # sampled path keeps extending into the start of the lap rather than
            # collapsing to a single repeated point that stalls the controller.
            indices = [(nearest + i * stride) % n_wp for i in range(sample_count)]
            values, distance = [], 0.0
            previous = (float(current["x"]), float(current["y"]))
            for index in indices:
                point = self.waypoints[index]
                xy = (float(point["x"]), float(point["y"]))
                distance += math.hypot(xy[0] - previous[0], xy[1] - previous[1])
                forward, left = local_xy(xy[0], xy[1], current)
                values.append([forward, left, distance, 1.0])
                previous = xy
            return values

        # The training route uses 20 consecutive ~0.5 m points (about 10 m total).
        model_values = sample(1)
        # Keep the 100 m sample only for the GUI overview.
        display_values = sample(10)
        dense_values = sample(1, 640 if self.highway and self.highway.active(nearest) else 240)
        for i, value in enumerate(dense_values):
            x, y = self.control_waypoint_xy[(nearest + i) % n_wp]
            value[0], value[1] = local_xy(x, y, current)
        return (torch.tensor(model_values, dtype=torch.float32).unsqueeze(0),
                model_values, display_values, dense_values)

    def image_tensor(self, rows):
        decoded, latest = [], []
        width, height = int(self.config["width"]), int(self.config["height"])
        # Adjacent model windows share most JPEG objects. Preserve the exact PIL
        # resize/normalization, but decode each immutable frame only once.
        cache = getattr(self, "_image_cache", None)
        if cache is None:
            cache = self._image_cache = OrderedDict()
        for ti, row in enumerate(rows):
            decoded_row = []
            for jpeg in row:
                key = (width, height, id(jpeg))
                entry = cache.get(key)
                if entry is None or entry[0] is not jpeg:
                    image = Image.open(io.BytesIO(jpeg)).convert("RGB").resize((width, height), Image.Resampling.BILINEAR)
                    array = np.asarray(image, dtype=np.float32) / 255.0
                    normalized = (array - IMAGE_MEAN) / IMAGE_STD
                    tensor = torch.from_numpy(normalized).permute(2, 0, 1)
                    bgr = cv2.cvtColor(np.asarray(image, dtype=np.uint8), cv2.COLOR_RGB2BGR)
                    entry = (jpeg, tensor, bgr)
                    cache[key] = entry
                    while len(cache) > 64:
                        cache.popitem(last=False)
                cache.move_to_end(key)
                decoded_row.append(entry[1])
                if ti == len(rows) - 1:
                    latest.append(entry[2].copy())
            decoded.append(torch.stack(decoded_row))
        return torch.stack(decoded).unsqueeze(0), latest

    def update_lap(self, status):
        """One-lap run: track route progress and hand the stop cap to the publisher."""
        tracker = getattr(self, "lap_tracker", None)
        if tracker is None or (tracker.done and getattr(self, "_lap_marker_written", False)):
            return None            # done: the publisher keeps its zero cap
        ego = status.get("ego")
        if not ego or self.input_error:
            return None            # stale pose: keep the last cap
        try:
            applied = getattr(self.path_publisher, '_active_decision_plan', None)
            signal = (applied.validation.get('traffic_signal') or {}) if applied is not None else {}
            lap = tracker.update(ego["x"], ego["y"], float(ego.get("speed_kmh", 0.)) / 3.6,
                                 time.monotonic(), rtf=runtime_rtf(self),
                                 signal_wait=signal_waiting(ego, signal, time.time()))
        except Exception as error:   # never stop the drive over lap bookkeeping
            if not getattr(self, "_lap_error", False):
                self._lap_error = True
                print(f"[lap] tracker error: {error!r}", flush=True)
            return None
        if self.path_publisher is not None:
            self.path_publisher.lap_cap_mps = lap["cap_mps"]
        self._lap_status = lap
        event = lap.get("event")
        if event in ("start", "restart", "armed"):
            print(f"[lap] {event}: progress {lap['progress_m']:.1f}/{lap['route_length_m']:.1f} m "
                  f"at ({float(ego['x']):.1f}, {float(ego['y']):.1f})", flush=True)
        if lap["just_done"]:
            record = tracker.record(ego["x"], ego["y"], wall_time=time.time(),
                                    run=os.path.basename(os.environ.get("STUDENT_LOG_DIR", "")) or None)
            path = lap_marker_path()
            print(f"[lap] complete: {json.dumps(record)}", flush=True)
            if path is not None:
                try:
                    write_lap_marker(path, record)
                except OSError as error:
                    print(f"[lap] marker not written: {error!r}", flush=True)
            self._lap_marker_written = True
        return lap

    def infer(self):
        status = self.update_ego()
        recovery = getattr(self, 'junction_recovery', None)
        if recovery is not None and not recovery.seen_start and status.get('ego'):
            recovery.observe(status['ego'], self.stop_line_controller, self.lanes,
                             self.intersection_regions, time.monotonic())
        if status.get("ego") and not self.input_error and getattr(self, "sim_clock", None) is not None:
            self.sim_clock.update(status["ego"], time.monotonic())
        self.update_lap(status)
        transition = getattr(self, '_planner_transition', None)
        if transition is None:
            transition = self._planner_transition = PlannerTransition()
        owner = transition.before(self, status)
        if owner == 'TUNNEL':
            self._tunnel_was_active = True
            self._transition_approach = not self.tunnel.active(status['ego'])
            result = self.infer_tunnel(status, status['ego'])
        else:
            self._transition_approach = False
            result = self._infer_current(status)
        transition.after(self)
        return result

    def _infer_current(self, status):
        lattice = None
        ego = status.get("ego")
        nearest = int((status.get("path") or {}).get("nearest_idx", 0))
        if ego and self.highway and self.highway.active(nearest):
            return self.infer_highway_only(status, ego)
        if self.highway and self.highway.phase != "OUTSIDE":
            self.highway.reset()
        if ego and getattr(self, "tunnel", None) and self.tunnel.active(ego):
            self._tunnel_was_active = True
            return self.infer_tunnel(status, ego)
        if getattr(self, "_tunnel_was_active", False):
            self.leave_tunnel()
            self._tunnel_was_active = False
        if isinstance(self.receiver, RecordedCameraReceiver):
            stale = self.receiver.stale_views()
            if stale:
                self.input_error = "Camera stale: " + ", ".join(stale)
                self.last_result = None
                if self.path_publisher:
                    self.path_publisher.publish_stop(self.input_error)
                return False
        rows = self.receiver.snapshot(int(self.config["frames"]))
        if self.input_error or not ego or rows is None:
            self.last_result = None
            if self.path_publisher:
                self.path_publisher.publish_stop(self.input_error or 'planner input unavailable')
            return False
        images, self.last_images = self.image_tensor(rows)
        ego_input = self.ego_tensor(ego)
        route_input, model_route, display_route, dense_route = self.route_tensor(ego, status)
        reference = getattr(self, '_entry_reference', None)
        if reference is None:
            reference = self._entry_reference = EntryReference()
        line_controller = getattr(self, 'stop_line_controller', None)
        dense_route = reference.apply(self.lanes, ego, dense_route,
            enabled=bool(line_controller is not None and line_controller.allow_empty_entry),
            now=time.monotonic(), constant_speed=RING_CONSTANT_SPEED)
        images = images.to(self.device, non_blocking=True)
        ego_input = ego_input.to(self.device, non_blocking=True)
        route_input = route_input.to(self.device, non_blocking=True)
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        started = time.perf_counter()
        with torch.inference_mode(), torch.autocast(device_type=self.device.type, enabled=self.device.type == "cuda"):
            output = self.model(images, ego_input, route_input)
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        latency = (time.perf_counter() - started) * 1000.0
        self.latencies.append(latency)
        probabilities = torch.softmax(output["action_logits"].float(), dim=-1)[0].cpu().numpy()
        concept_values = torch.sigmoid(output["concept_logits"].float())[0].cpu().numpy()
        scene = output["scene"].float()[0].cpu().numpy()
        # Training supervision is already an ego-local trajectory. Integrating it again turns
        # metre-valued poses into impossible tens-of-metres offsets.
        predicted_path = output["path"].float()[0].cpu().numpy()
        action_index = int(np.argmax(probabilities))
        action_name = self.actions[action_index]
        action_probability = float(probabilities[action_index])
        model_avoidance_intent = (action_name in {"YIELD", "LEFT_CHANGE", "RIGHT_CHANGE"}
                                  and action_probability >= 0.50)
        now = time.monotonic()
        saved_static = getattr(self, "_static_plan_saved", None)
        previous_static = continue_static_plan(saved_static, ego, now)
        sensor_latch = getattr(self, "_sensor_bypass_latch", None)
        if sensor_latch and not 0. <= sensor_latch["until"] - now <= 2.:
            sensor_latch = self._sensor_bypass_latch = None
        # Only a sensor-started manoeuvre may carry its direction forward.
        # A model action or an old direction timer is not obstacle evidence.
        direction_latched = (previous_static["direction"] if previous_static is not None else
                             sensor_latch["direction"] if sensor_latch else None)
        if direction_latched is None:
            self.avoidance_direction, self.avoidance_latch_until = 0., 0.
        committed_direction = direction_latched or 0.0
        # hazard rule: an object on the route ahead forces a bypass away from it (if the model did not already decide),
        # and a stop when it is too close to pass. Overrides nothing when the route ahead is clear.
        objs_snap = self.object_receiver.snapshot() if self.object_receiver else []
        if self.object_receiver and getattr(self.object_receiver, "age_s", 999.) > .5:
            # Stale detections still trigger the stream safety stop below, but
            # cannot authorize a new lateral manoeuvre.
            objs_snap = []
        objs_snap = self.intersection_objects(objs_snap, ego)
        objs_snap = self.tunnel_vehicle_objects(objs_snap, ego)
        route_index = int((status.get("path") or {}).get("nearest_idx", 0))
        in_obstacle_zone = in_obstacle_mission(route_index)
        entry_obstacles = (bool(getattr(self, '_transition_entry_obstacles', False))
                           and getattr(self, '_intersection_vehicle_status', None) is None)
        in_static_obstacle_zone = in_obstacle_zone or entry_obstacles
        if (USE_LATTICE or MODEL_AVOID) and not in_static_obstacle_zone:
            # Rule map (3-1): obstacles and people only in the avoidance
            # sections and the GPS-shadow tunnel; everywhere else vehicles.
            objs_snap = [o for o in objs_snap if o[4] == "npc"]
        objs_snap = self.refine_pedestrians(objs_snap, ego)
        objs_snap = self.deduplicate_objects(objs_snap, now)
        self._obj_debug = []
        v_now = float(ego.get("speed_kmh", 0.0)) / 3.6
        trig_m = float(np.clip(3.5 * v_now, 8.0, 25.0))     # bypass trigger: 3.5 s of travel (15 m at 15 km/h, 25 m at 26+)
        stop_m = float(np.clip(1.5 * v_now, 4.0, 8.0))      # stop distance on the path we drive: 1.5 s
        hazard = route_hazard(objs_snap, ego, dense_route, trig_m, half_width_m=1.4, max_speed_mps=1.5, debug=self._obj_debug) if self.object_receiver else None
        bypass_objects = [o for o in objs_snap if o[4] != "pedestrian" or
                          (self.pedestrian_wait_expired(now) and self.person_blocks(o, ego, dense_route, now))]
        if self.object_receiver:
            hazard = route_hazard(bypass_objects, ego, dense_route, trig_m, half_width_m=1.4, max_speed_mps=1.5)
        self._obj_debug.sort(key=lambda d: d["f"])
        self._n_obj = len(objs_snap)
        # LiDAR corridor on the route ahead (and a clearance helper used for every candidate path below)
        hw = 0.95 + margin_for_speed(v_now)
        from static_lane_pass import normal_static_area
        road_scan_required = bool(MODEL_AVOID and not USE_LATTICE and in_obstacle_zone
            and not entry_obstacles and getattr(self, '_intersection_vehicle_status', None) is None
            and normal_static_area(self.lanes, ego))
        road_scan_frame = None
        near_field_stamp = getattr(self.lidar, 'received', 0.)
        if road_scan_required:
            from perception.road_scan_frame import align_road_scan_wait
            captured_at = time.monotonic()
            scan = self.lidar.scan_snapshot() if self.lidar is not None else {}
            frame = align_road_scan_wait(scan, ego,
                getattr(getattr(self, 'ego_receiver', None), 'measured_xy_at_stamp', None),
                captured_at=captured_at)
            road_scan_frame = self._road_scan_frame = frame['proof']
            if not road_scan_frame['valid']:
                self.input_error = road_scan_frame['reason']
                self.last_result = None
                if self.path_publisher is not None:
                    self.path_publisher.publish_stop(self.input_error)
                return False
            lid_pts, lid_age = frame['points'], road_scan_frame['age_s']
            near_field_stamp = road_scan_frame['stamp']
        else:
            lid_pts, lid_age = self.lidar.snapshot() if self.lidar else (None, 99.0)
        if entry_obstacles and lid_pts is not None:
            scan = self.lidar.scan_snapshot() if hasattr(self.lidar, 'scan_snapshot') else {}
            raw_points = scan.get('obstacle_points')
            if raw_points is None:
                raw_points = lid_pts
            current_movers = (highway_measured_objects(objs_snap) if
                float(getattr(self.object_receiver, 'age_s', 999.)) <= .3 else [])
            lid_pts = update_approach_points(self, scan, raw_points, ego, current_movers)
            lid_age = float(scan.get('age_s', lid_age)) if lid_pts is not None else 999.
        elif hasattr(self, '_portal_approach_memory'):
            self._portal_approach_memory.reset()
        self._contact = self.contact_check(lid_pts, objs_snap, ego, in_static_obstacle_zone)
        if lid_pts is not None:
            lid_pts = self.near_field_memory.update(lid_pts, ego, near_field_stamp,
                                                   highway_measured_objects(objs_snap))
        lid_static_pts = without_moving_objects(lid_pts, highway_measured_objects(objs_snap), ego, self._ped_motion_history) if self.clearance != "objects" else lid_pts
        lid_route = self.lidar.blocked(np.asarray(dense_route)[:, :2], 1.5, 30.0, pts=lid_static_pts) if lid_pts is not None else None   # trigger corridor 1.5 m
        self._lidar_log = {"n": int(len(lid_pts)) if lid_pts is not None else None, "age_s": round(float(lid_age), 2) if lid_pts is not None else None,
                           "retained_near_field_n": self.near_field_memory.retained_n,
                           "road_plane": [round(float(v), 4) for v in self.lidar.ground_plane] if self.lidar else None,
                           "masked_moving_n": int(len(lid_pts) - len(lid_static_pts)) if lid_pts is not None else None,
                           "route_block_m": round(lid_route[0], 1) if lid_route else None, "side": lid_route[1] if lid_route else None, "path_block_m": None}
        if road_scan_required:
            road_context = dict(scan.get('mapped_road_floor', {}))
            road_context.update(scan_stamp=road_scan_frame['stamp'],
                                scan_sequence=road_scan_frame['sequence'])
            self._lidar_log['mapped_road_floor'] = road_context
            self._lidar_log['road_scan_frame'] = road_scan_frame
        elif self.lidar is not None and hasattr(self.lidar, 'road_diagnostics'):
            with self.lidar.lock:
                road_context = dict(self.lidar.road_diagnostics)
                road_context['scan_stamp'] = self.lidar._scan_stamp
                road_context['scan_sequence'] = self.lidar._scan_sequence
            self._lidar_log['mapped_road_floor'] = road_context
        if entry_obstacles:
            self._lidar_log['portal_memory'] = getattr(self, '_portal_memory_status', None)
        lid_hazard = (max(lid_route[0], 3.0), lid_route[1], "lidar", "obstacle", 0.0) if (lid_route is not None and lid_route[0] <= trig_m) else None
        if self.clearance == "lidar":
            hazard = lid_hazard
        elif self.clearance == "both" and lid_hazard is not None and (hazard is None or lid_hazard[0] < hazard[0] - 1.0):
            hazard = lid_hazard   # both: the object list OR the point cloud triggers (hs_6 16:47: median barrier is not in Object_topic)
        self.last_hazard = hazard
        if hazard and hazard[3] == "pedestrian" and (len(hazard) < 5 or hazard[4] >= 0.3):
            hazard = None   # never plan a bypass around a MOVING person: the yield rule below stops for them (standing: bypass)
        sensor_avoidance = hazard is not None or sensor_latch is not None
        avoidance_intent = sensor_avoidance
        if hazard:
            # the model decides WHEN, the geometry decides WHICH SIDE: 2026-09-27 23:10 log showed the model's left/right sign
            # pointing toward the object in 178 of 286 bypass frames (collisions with barrels). Always pass on the far side.
            away = (float(direction_latched) if direction_latched in (-1, 1, -1.0, 1.0)
                    else (-1.0 if hazard[1] > 0 else 1.0))
            direction_latched = away
        force_avoidance = self.args.force_student_avoidance and sensor_avoidance
        hybrid_path, postprocess = postprocess_trajectory(
            predicted_path, dense_route,
            force_student_avoidance=force_avoidance,
            student_enabled=sensor_avoidance,
            forced_direction=direction_latched)
        avoidance_evidence = ("sensor_path_blocked" if hazard is not None else
                              "sensor_manoeuvre_continuation" if sensor_latch else "none")
        if not sensor_avoidance:
            hybrid_path = np.asarray(dense_route, dtype=np.float32)[:, :2]
            postprocess = dict(postprocess, mode="ROUTE BASE", reason="no sensor-confirmed path obstruction",
                               applied_peak_m=0.)
        if postprocess["mode"] == "ROUTE FALLBACK":
            # the model's raw path was unusable ("no ordered forward Student path"): drive the map route, never the raw path
            # (2026-09-27 23:22: publishing the raw path here produced a 27->0 km/h stop and 46 km/h overspeed at the start)
            hybrid_path = np.asarray(dense_route, dtype=np.float32)[:len(dense_route), :2]
            postprocess = dict(postprocess, mode="ROUTE BASE", reason="fallback: " + str(postprocess.get("reason")), applied_peak_m=0.0)
        # --- drivable-lane check for any forced S-curve: the hold section must lie on some HD-map lane centreline ------
        lane_corridor = self.lanes.corridor(ego, dense_route)
        if self.path_publisher is not None:
            self.path_publisher.publish_corridor(ego, lane_corridor)
        def map_clearance(path_local, horizon=60.):
            return lane_corridor.clear_distance(path_local, horizon) if lane_corridor is not None else 0.
        def map_ok(path_local, tol_m=1.5):
            path = np.asarray(path_local)[:, :2]
            length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
            return map_clearance(path, length) >= length - .3
        def corridor_clearance(path_local, horizon=60.):
            available = map_clearance(path_local, horizon)
            if self.clearance != "objects":
                if lid_pts is None:
                    return 0.
                hit = self.lidar.blocked(path_local, hw, horizon + 2.3175, pts=lid_pts)
                if hit is not None:
                    available = min(available, max(0., hit[0] - 2.3175))
            return available
        def lane_ok(path_local, tol_m=1.5):
            length = min(25., float(np.linalg.norm(np.diff(np.asarray(path_local)[:, :2], axis=0), axis=1).sum()))
            return corridor_clearance(path_local, length) >= length - .3
        if postprocess["mode"] != "ROUTE BASE" and (
                float(np.max(path_curvatures(hybrid_path))) > MAX_CONTROL_CURVATURE or not lane_ok(hybrid_path)):
            hybrid_path = np.asarray(dense_route, dtype=np.float32)[:, :2]
            postprocess = dict(postprocess, mode="ROUTE BASE", reason="Student path outside feasible lane corridor", applied_peak_m=0.)
        if not USE_LATTICE:
            planning_started = time.perf_counter()
            adjacent_offsets = self.lanes.adjacent_offsets(ego, dense_route)
            if MODEL_AVOID:
                # Portable static planner + v6 side, host static/dynamic split (model_avoidance.py).
                static_plan, static_hazards, model_context = self.model_static_plan(
                    ego, dense_route, objs_snap, lid_static_pts, lane_corridor, map_ok, map_clearance,
                    previous_static, adjacent_offsets, now, in_obstacle_zone, in_static_obstacle_zone, hw)
                plan_objects, plan_clearance = model_context["static"], model_context["clearance"]
                recovery_points = model_context["points"]
                lane_corridor = model_context.get('corridor', lane_corridor)
                if self.path_publisher is not None:
                    self.path_publisher.publish_corridor(ego, lane_corridor)
                committed_direction = model_context["direction"]
            else:
                plan_objects, plan_clearance, recovery_points = objs_snap, corridor_clearance, lid_pts
                static_plan, static_hazards = (plan_static_bypass(
                    dense_route, ego, objs_snap, map_ok, committed_direction, previous_static,
                    adjacent_offsets, corridor_clearance)
                    if self.object_receiver else (None, []))
            static_plan_blocked = bool((static_hazards and static_plan is None) or
                                       (static_plan and static_plan.get("blocked")))
            soft_lane_recovery = False
            route_distance = float((status.get("path") or {}).get("distance_m", 0.))
            if (static_plan_blocked and not (MODEL_AVOID and model_context.get('standing_bypass'))
                    and saved_static and 0. <= now - saved_static["stamp"] <= 1. and
                    route_distance >= 2.0 and lane_corridor is not None):
                # Keep a committed bypass moving through same-direction road space
                # when strict lane-change metadata rejects every otherwise clear
                # path. Never use this fallback to start a new bypass.
                recovery_corridor = self.lanes.corridor(ego, dense_route, allow_second_same_direction=True)
                if recovery_corridor is not None:
                    def recovery_clearance(path_local, horizon=60.):
                        available = recovery_corridor.clear_distance(path_local, horizon)
                        if self.clearance != "objects":
                            if lid_pts is None:
                                return 0.
                            if recovery_points is not None and len(recovery_points):
                                hit = self.lidar.blocked(path_local, hw, horizon + 2.3175, pts=recovery_points)
                                if hit is not None:
                                    available = min(available, max(0., hit[0] - 2.3175))
                        return available

                    def recovery_lane_ok(path_local, tol_m=1.5):
                        length = min(25., float(np.linalg.norm(np.diff(np.asarray(path_local)[:, :2], axis=0), axis=1).sum()))
                        return recovery_clearance(path_local, length) >= length - .3

                    if MODEL_AVOID:
                        recovery_lane_ok, recovery_clearance = model_avoidance.road_static_checks(
                            recovery_corridor, recovery_points if self.clearance != 'objects' else np.empty((0, 2)))
                        recovery_plan, recovery_hazards, _ = model_avoidance.plan_bypass(
                            dense_route, ego, plan_objects, recovery_lane_ok, recovery_clearance,
                            committed_direction, previous_static, adjacent_offsets,
                            body_shift_m=REAR_AXLE_SHIFT_M)
                    else:
                        recovery_plan, recovery_hazards = plan_static_bypass(
                            dense_route, ego, objs_snap, recovery_lane_ok, committed_direction,
                            previous_static, adjacent_offsets, recovery_clearance)
                    if (recovery_plan and not recovery_plan.get("blocked") and
                            recovery_plan["clear_distance_m"] >= max(
                                recovery_plan["safe_horizon_m"], recovery_plan["hold_until_m"])):
                        static_plan, static_hazards = recovery_plan, recovery_hazards
                        static_plan_blocked, soft_lane_recovery = False, True
            static_rejected = static_plan.get("rejected") if static_plan else None
            static_prefix_recovered = False
            if (static_plan_blocked and previous_static is not None and
                    not (MODEL_AVOID and model_context.get('standing_bypass'))):
                recovered = (model_avoidance.portable.recover_static_prefix(
                                 previous_static, ego, plan_objects, plan_clearance,
                                 body_shift_m=REAR_AXLE_SHIFT_M) if MODEL_AVOID else
                             recover_static_prefix(previous_static, ego, objs_snap, corridor_clearance))
                if recovered is not None:
                    static_plan, static_plan_blocked, static_prefix_recovered = recovered, False, True
            planning_ms = (time.perf_counter() - planning_started) * 1000.
            if static_plan_blocked:
                static_plan = None
            if static_plan is not None:
                avoidance_intent = True
                avoidance_evidence = "validated_static_rejoin" if static_plan["kind"] == "rejoin" else "validated_static_bypass"
                hybrid_path = static_plan["path"]
                self.avoidance_direction = static_plan["direction"]
                self.avoidance_latch_until = now + 3.0
                rejoining = static_plan["kind"] == "rejoin"
                if (rejoining and float((status.get("path") or {}).get("distance_m", 999)) < 0.5
                        and (not MODEL_AVOID or model_context.get('route_return_ready'))):
                    self.avoidance_direction, self.avoidance_latch_until = 0.0, 0.0
                postprocess = dict(postprocess, mode="STUDENT REJOIN" if rejoining else "STUDENT FORCED",
                                   reason=("same-direction lane recovery" if soft_lane_recovery else
                                           "collision-checked route rejoin" if rejoining else
                                           "braking-safe static path with lookahead replanning"),
                                   applied_peak_m=static_plan["direction"] * static_plan["offset_m"],
                                   max_curvature_1pm=static_plan["curvature"], transition_m=static_plan["transition_m"],
                                   hold_m=static_plan["hold_m"], validated_static=True,
                                   static_lane_borrow=bool(static_plan.get("static_borrow")),
                                   static_motion_profile=bool(static_plan.get('static_motion_profile')),
                                   static_profile_ceiling_mps=static_plan.get('static_profile_ceiling_mps'),
                                   static_profile_command_mps=static_plan.get('speed_cap_mps'),
                                   canonical_control=bool(static_plan.get("standing_bypass_ids") or static_plan.get("bounded_static_pass")
                                       or static_plan.get('canonical_control')),
                                   standing_bypass_ids=static_plan.get('standing_bypass_ids', []))
            elif static_plan_blocked:
                if direction_latched in (-1, 1, -1.0, 1.0):
                    self.avoidance_direction = float(direction_latched)
                    self.avoidance_latch_until = now + 3.0
                # Keep the steering path during braking instead of snapping to the
                # global centreline while the car is still alongside the obstacle.
                hybrid_path = (previous_static["path"] if previous_static is not None else
                               np.asarray(dense_route, dtype=np.float32)[:, :2])
                postprocess = dict(postprocess, mode="STUDENT HOLD" if previous_static is not None else "ROUTE BASE",
                                   reason="no braking-safe static path; stop", applied_peak_m=0.0,
                                   validated_static=previous_static is not None)
            keep_checked_static = bool(MODEL_AVOID and static_plan is not None
                and model_context.get('steady_static') and not model_context.get('standing_bypass')
                and static_plan.get('canonical_control') and not static_plan.get('standing_bypass_ids')
                and not static_plan.get('bounded_static_pass') and not static_plan.get('static_borrow')
                and model_avoidance.static_plan_complete(static_plan))
            if static_plan is not None and (self.avoidance_direction or keep_checked_static):
                self._static_plan_saved = {
                    "stamp": now, "plan": {k: v for k, v in static_plan.items() if k != "path"},
                    "world_path": [RosPathPublisher.global_xy(ego, float(p[0]), float(p[1]))
                                   for p in static_plan["path"]]}
            elif static_plan_blocked and previous_static is not None:
                # Keep the committed steering path while stopped. Letting its
                # timestamp expire switches to the global route beside the object.
                self._static_plan_saved = {
                    "stamp": now, "plan": {k: v for k, v in previous_static.items() if k != "path"},
                    "world_path": [RosPathPublisher.global_xy(ego, float(p[0]), float(p[1]))
                                   for p in previous_static["path"]]}
            else:
                self._static_plan_saved = None
            if MODEL_AVOID:
                postprocess['static_rear_axle'] = bool(model_context.get('planning'))
                postprocess['static_complete'] = (model_avoidance.static_plan_complete(static_plan)
                    or (static_plan is None and not static_plan_blocked and not static_hazards))
                postprocess["static_blocked"] = static_plan_blocked   # DrivingPlan blocked flag
                if static_plan is None and not static_plan_blocked:
                    # No validated bypass and nothing to stop for: the global route.
                    # The best_v5 trajectory never steers in this mode.
                    hybrid_path = np.asarray(dense_route, dtype=np.float32)[:, :2]
                    postprocess = dict(postprocess, mode="ROUTE BASE", applied_peak_m=0.,
                                       reason="no validated static bypass needed")
        else:
            planning_started = time.perf_counter()
            static_objects = [o for o in objs_snap if o[4] == "obstacle" or
                              (o[4] == "pedestrian" and o[5] < .3) or (o[4] == "npc" and o[5] < .5)]
            static_objects = self.remember_static_objects(static_objects, now, in_static_obstacle_zone, objs_snap)
            # People are waited for, not driven around: a person crossing
            # turned into a -3.5 m "bypass" that then sat beside them for 50 s
            # (live 2026-10-04). Only someone who stays in our lane past
            # PEDESTRIAN_WAIT_S becomes an obstacle to pass (user rule).
            def _person_blocks(o):
                # Route-relative too: after a skewed reverse the car-relative |l| < 2
                # left a person standing in our lane out of the plan and the
                # pass-around drove into them (sim of run 145).
                # Standing people only: someone walking (crossing) is waited for, not
                # planned around - a bypass toward their destination was boxed in by
                # their heading (user 2026-10-05).
                return self.person_blocks(o, ego, dense_route, now)
            static_objects = [o for o in static_objects if o[4] != "pedestrian"
                              or (self.pedestrian_wait_expired(now) and _person_blocks(o))]
            waiting_people = [o for o in objs_snap if o[4] == "pedestrian"
                              and not (self.pedestrian_wait_expired(now) and _person_blocks(o))]
            pedestrian_wait = self.pedestrian_lattice_wait(waiting_people, ego, dense_route)
            previous_lattice = getattr(self, "_lattice_prev", None)
            entry_corridor = bool(getattr(self, '_transition_entry_corridor', False))
            if entry_corridor:
                from planner_transition import approach_corridor
                lane_corridor = approach_corridor(self.tunnel, self.lanes, ego, dense_route)
            if previous_lattice is not None and not 0. <= now - previous_lattice["stamp"] <= 1.:
                previous_lattice = None
            out_of_junction = getattr(self, "_intersection_vehicle_status", None) is None
            racing_base = self.racing_base_offsets(route_index, len(dense_route)) if out_of_junction else None
            if (racing_base is not None and route_index >= RACING_LATE_ENTRY_IDX
                    and getattr(self, "_line_kind", None) != "racing"):
                racing_base = None
            racing_gate = None
            if racing_base is not None:
                racing_ok, racing_gate = self.racing_gate(ego, dense_route, racing_base, lid_pts, now)
                if not racing_ok:
                    racing_base = None
            line_kind = "racing" if racing_base is not None else None
            line_cap_mps = None
            if racing_base is None and GUARD_LINE and out_of_junction:
                # Racing off (traffic / just switched off): the guardrail line, not the lane centre.
                lane2 = self.lane2_vehicle(ego, dense_route,
                                           self.racing_base_offsets(route_index, len(dense_route), "safe_wide"), lid_pts)
                kind = "safe_lane" if lane2 else "safe_wide"
                guard_base = self.racing_base_offsets(route_index, len(dense_route), kind)
                if guard_base is not None:
                    racing_base, line_kind = guard_base, kind
                    line_cap_mps = self.guard_line_cap_mps(route_index, dense_route, kind)
                    if lane2:
                        racing_gate = f"{racing_gate}; lane 2: {lane2}"
            if line_kind != getattr(self, "_line_kind", None):
                self._line_kind, self._line_switch_at = line_kind, now
            # Gentle join only right after a line switch (no swing toward the guardrail);
            # otherwise the usual pull-back, which keeps the follower's drift small.
            line_join_alat = LINE_JOIN_ALAT_MPS2 if now - getattr(self, "_line_switch_at", -99.) < LINE_JOIN_S else None
            # The plan the follower is tracking: a line plan re-starts from it, not from the
            # car (run 227), also across a line switch (only the hysteresis below drops it).
            anchor_lattice = getattr(self, "_lattice_prev", None)
            anchor_offsets = None
            if (anchor_lattice is not None and anchor_lattice.get("offsets") is not None
                    and 0. <= now - anchor_lattice["stamp"] <= .5 and abs(anchor_lattice["target"]) < .01
                    and not anchor_lattice.get("blocked")):
                shift = route_index - int(anchor_lattice["route_index"])
                kept = anchor_lattice["offsets"][shift:] if 0 <= shift < len(anchor_lattice["offsets"]) else ()
                if len(kept) >= 3:
                    anchor_offsets = np.full(len(dense_route), np.nan)
                    anchor_offsets[:min(len(kept), len(dense_route))] = kept[:len(dense_route)]
            if previous_lattice is not None and previous_lattice.get("line") != line_kind:
                # Line changed (racing / guardrail / lane centre): the previous path must
                # not be kept by the hysteresis, e.g. staying in the 2nd lane behind a car.
                previous_lattice = None
            # A lead that stood in our lane past its wait (lead_follow_step) is planned
            # around like an obstacle-section blocker: the lattice gets the LiDAR returns.
            lead_bypass = (getattr(self, "_intersection_vehicle_status", None) is None and not in_obstacle_zone
                           and self.lead_bypass_active(ego, now))
            if not lead_bypass and (getattr(self, "_intersection_vehicle_status", None) is not None
                                    or not in_static_obstacle_zone):
                # Junction: only vehicles matter (timed by the gap planner).
                # Curbs, painted islands and poles stopped the car in the
                # roundabout (live 2026-10-03, "yellow" island edge).
                junction_points, static_objects = None, []
            else:
                junction_points = lid_static_pts if self.clearance != "objects" else None
                if junction_points is not None and len(junction_points):
                    # LiDAR returns of people are waited for (yield logic), not
                    # planned around; they kept the bypass "blocked" and the car
                    # inching (live 2026-10-04). Kept once the standing wait expired
                    # for a person in our lane.
                    ped_xy = [local_xy(o[0], o[1], ego) for o in objs_snap if o[4] == "pedestrian"
                              and not (self.pedestrian_wait_expired(now) and _person_blocks(o))]
                    if ped_xy:
                        pts_arr = np.asarray(junction_points, dtype=float)
                        keep = np.ones(len(pts_arr), dtype=bool)
                        for f, l in ped_xy:
                            keep &= np.hypot(pts_arr[:, 0] - f, pts_arr[:, 1] - l) > 1.2
                        junction_points = pts_arr[keep]
                touching = getattr(self, "_contact", None)
                if touching and junction_points is not None and len(junction_points) and not touching["block_forward"]:
                    # Something against the rear half of our side (auto run 268): driving
                    # forward is the way away from it, so it must not block every candidate
                    # at its first pose; contact_sweep_ok rejects paths that swing toward it.
                    pts_arr = np.asarray(junction_points, dtype=float)
                    beside = ((pts_arr[:, 0] < 1.0) & (pts_arr[:, 0] > -PED_REAR_M - 1.)
                              & (np.abs(pts_arr[:, 1]) > .946) & (np.abs(pts_arr[:, 1]) < .946 + .7))
                    junction_points = pts_arr[~beside]
            if junction_points is not None:
                # People beside the lane (shoulder walkers) are keep-out zones for the
                # candidate paths: excluded entirely, a -1.5 m bypass swerved toward a
                # walker on the shoulder (auto run 102). Outside the lane band they never
                # trigger a bypass; in-lane people stay with the wait rules above.
                keep_out = []
                for o in objs_snap:
                    if o[4] != "pedestrian":
                        continue
                    f, l = local_xy(o[0], o[1], ego)
                    if not (-3. < f < 40.) or abs(route_lateral(dense_route, f, l)) < 1.6:
                        continue
                    vx, vy = (float(o[9]), float(o[10])) if len(o) > 10 else (0., 0.)
                    h = math.radians(float(ego["heading_deg"]))
                    vf, vl = math.cos(h) * vx + math.sin(h) * vy, -math.sin(h) * vx + math.cos(h) * vy
                    # Walkers along the road only. A crossing person's predicted path ran
                    # into our lane and the lattice "bypassed" their destination, boxed in by
                    # their heading (user 2026-10-05); crossing people are waited for (yield
                    # rules), never steered around. Predicted points inside the lane are
                    # dropped for the same reason.
                    if abs(vl) > .3:
                        continue
                    for t in (0., 1., 2., 3.):
                        cf, cl = f + vf * t, l + vl * t
                        if abs(route_lateral(dense_route, cf, cl)) < 1.6:
                            continue
                        for a in np.linspace(0., 2. * math.pi, 8, endpoint=False):
                            # Person-sized (0.4 m): 0.8 m made the lane-centre path step away
                            # from walkers 2 m beside it (user: never re-route for walkers).
                            keep_out.append((cf + .4 * math.cos(a), cl + .4 * math.sin(a)))
                if keep_out:
                    junction_points = (np.vstack((np.asarray(junction_points, dtype=float).reshape(-1, 2),
                                                  np.asarray(keep_out)))
                                       if len(junction_points) else np.asarray(keep_out))
            paved_road_only = 480 <= route_index <= 1100
            try:
                road = self.lanes.road_corridor(ego, dense_route) if getattr(self, "lanes", None) else None
                if not paved_road_only and road is not None and lane_corridor is not None and len(lane_corridor.left) == len(road.left):
                    # Legal lanes are road by definition.
                    road.left = np.maximum(road.left, lane_corridor.left)
                    road.right = np.minimum(road.right, lane_corridor.right)
            except Exception:
                road = None
            if entry_corridor:
                road = lane_corridor
            lattice = lattice_plan(dense_route, ego, junction_points,
                                   static_objects, lane_corridor, previous_lattice, model_path=predicted_path,
                                   road_corridor=road, wait_for_pedestrian=pedestrian_wait,
                                   paved_road_only=paved_road_only,
                                   pedestrian_inflate_m=.1 if self.pedestrian_wait_expired(now) else None,
                                   # No shoulder at the tunnel mouth: the wall is right there
                                   # (live 2026-10-04: pushed -2 m right at idx 3762, wedged
                                   # against the tunnel wall).
                                   right_shoulder_m=0. if NO_SHOULDER_IDX[0] <= int((status.get("path") or {}).get("nearest_idx", 0)) <= NO_SHOULDER_IDX[1] else None,
                                   cruise_mps=zone_speed_kmh(self.speed_zones, int((status.get("path") or {}).get("nearest_idx", 0)))[0] / 3.6,
                                   max_curvature=MAX_CONTROL_CURVATURE, base_offset=racing_base,
                                   max_offset_m=5. if entry_corridor else 3.5,
                                   extra_clearance=(lambda p, h: self.tunnel.path_clearance(p, ego, h))
                                       if entry_corridor else None,
                                   base_label="racing line" if line_kind == "racing" else "guardrail line",
                                   join_alat_mps2=line_join_alat, anchor_offsets=anchor_offsets,
                                   anchor_same_line=anchor_lattice is not None and anchor_lattice.get("line") == line_kind,
                                   gentle_side=GUARD_SIDE)
            if lattice["blocked"] and LANE_ESCAPE_ON_BLOCK and not pedestrian_wait:
                # No in-lane path: take the line penalty and keep going instead of
                # stopping (user 2026-10-05: "the goal is to finish; with no legal
                # answer, take the penalty and go"). Legal paths still win whenever
                # one exists - this re-plan only runs when all of them are blocked.
                escape = lattice_plan(dense_route, ego, junction_points,
                                      static_objects, lane_corridor, previous_lattice, model_path=predicted_path,
                                      road_corridor=road, outside_max_m=1.0,
                                      paved_road_only=paved_road_only,
                                      pedestrian_inflate_m=.1 if self.pedestrian_wait_expired(now) else None,
                                      right_shoulder_m=0. if NO_SHOULDER_IDX[0] <= int((status.get("path") or {}).get("nearest_idx", 0)) <= NO_SHOULDER_IDX[1] else None,
                                      cruise_mps=zone_speed_kmh(self.speed_zones, int((status.get("path") or {}).get("nearest_idx", 0)))[0] / 3.6,
                                      max_curvature=MAX_CONTROL_CURVATURE,
                                      extra_clearance=(lambda p, h: self.tunnel.path_clearance(p, ego, h))
                                          if entry_corridor else None)
                if not escape["blocked"]:
                    escape["reason"] = "no in-lane path, leaving the lane: " + str(escape.get("reason"))
                    lattice = escape
            lattice["racing_gate"] = racing_gate
            lattice["line"] = line_kind
            if line_cap_mps is not None and line_kind is not None:
                lattice["speed_cap_mps"] = min(lattice["speed_cap_mps"], line_cap_mps)
                lattice["line_cap_kmh"] = round(line_cap_mps * 3.6, 1)
            # Never park for good (competition, user 2026-10-04): stopped with no
            # path, retry with a tight margin and finer offsets and creep through;
            # after STALL_RELAX_S also use the shoulder beyond its allowance.
            ego_kmh = float(ego.get("speed_kmh", 0.))
            if not lattice["blocked"] or ego_kmh > 3.:
                self._stall_since = None
            elif getattr(self, "_stall_since", None) is None:
                self._stall_since = now
            creeping = now < getattr(self, "_lattice_tight_until", -1.)
            # Never squeeze past people: the tight creep found a 0.05 m gap beside a
            # crossing pedestrian and pushed into them at 3-5 km/h (auto run 61).
            person_near = any(o[4] == "pedestrian" and -2. < local_xy(o[0], o[1], ego)[0] < 10.
                              and abs(local_xy(o[0], o[1], ego)[1]) < 3. for o in objs_snap)
            # The tight/relaxed re-plan is the expensive one (0.7-0.8 s per frame with
            # dense LiDAR while stalled: 1 Hz decisions, auto run 156). Standing still it
            # cannot change much in 0.5 s, so retry at 2 Hz; while creeping, every frame.
            retry_due = creeping or now - getattr(self, "_tight_retry_at", -9.) >= .5
            touching = getattr(self, "_contact", None)
            if lattice["blocked"] and (ego_kmh < 3. or creeping) and not person_near and not pedestrian_wait and retry_due \
                    and not (touching and touching["block_forward"]):
                self._tight_retry_at = now
                stalled = now - (getattr(self, "_stall_since", None) or now)
                relaxed = stalled >= STALL_RELAX_S
                tight = lattice_plan(dense_route, ego, junction_points, static_objects, lane_corridor, None,
                                     road_corridor=road, pedestrian_inflate_m=.1 if self.pedestrian_wait_expired(now) else None,
                                     paved_road_only=paved_road_only,
                                     outside_max_m=1.0 if stalled >= STALL_LANE_ESCAPE_S else None,
                                     right_shoulder_m=2.5 if relaxed else (
                                         0. if NO_SHOULDER_IDX[0] <= int((status.get("path") or {}).get("nearest_idx", 0)) <= NO_SHOULDER_IDX[1] else None),
                                     cruise_mps=TIGHT_PASS_KMH / 3.6, max_curvature=MAX_CONTROL_CURVATURE,
                                     side_margin_m=.05 if relaxed else .12,
                                     extra_clearance=(lambda p, h: self.tunnel.path_clearance(p, ego, h))
                                         if entry_corridor else None)
                if not tight["blocked"]:
                    tight["speed_cap_mps"] = min(tight["speed_cap_mps"], TIGHT_PASS_KMH / 3.6)
                    tight["reason"] = ("stall escape (relaxed): " if relaxed else "tight pass: ") + str(tight.get("reason"))
                    lattice = tight
                    self._lattice_tight_until = now + 2.
            # Same crawl hold as the tunnel (auto run 58): "no path" near an obstacle
            # means approach at <= CRAWL_KMH until it stays clear for 3 s.
            self._lattice_blocked_n = (getattr(self, "_lattice_blocked_n", 0) + 1
                                       if lattice["blocked"] and lattice.get("clear_m", 99.) < 20. else 0)
            if self._lattice_blocked_n >= 2:     # one flickering frame must not cost a crawl
                # No re-acceleration until clear for 3 s (floor CRAWL_KMH).
                held = (getattr(self, "_lattice_hold_mps", 99.)
                        if now < getattr(self, "_lattice_crawl_until", -1.) else 99.)
                self._lattice_hold_mps = max(CRAWL_KMH / 3.6, min(held, float(ego.get("speed_kmh", 0.)) / 3.6))
                self._lattice_crawl_until = now + 3.
            self._lattice_clear_n = 0 if lattice["blocked"] else getattr(self, "_lattice_clear_n", 0) + 1
            if self._lattice_clear_n >= 2:   # path checked clear twice: only curves limit speed
                self._lattice_crawl_until = -1.
            if now < getattr(self, "_lattice_crawl_until", -1.):
                lattice["speed_cap_mps"] = min(lattice["speed_cap_mps"], self._lattice_hold_mps)
            planning_ms = (time.perf_counter() - planning_started) * 1000.
            hybrid_path = np.asarray(lattice["path"], dtype=np.float32)
            self._lattice_prev = dict(stamp=now, target=lattice["target"], racing=line_kind == "racing", line=line_kind,
                                      blocked=bool(lattice["blocked"]), route_index=route_index,
                                      offsets=(None if lattice.get("offsets") is None
                                               else np.asarray(lattice["offsets"], dtype=float)),
                                      world_path=[RosPathPublisher.global_xy(ego, float(p[0]), float(p[1]))
                                                  for p in hybrid_path])
            static_plan, static_hazards, static_rejected = None, [], None
            static_plan_blocked = lattice["blocked"]
            soft_lane_recovery = static_prefix_recovered = False
            self._static_plan_saved = None
            avoidance_intent = abs(lattice["target"]) > .01
            avoidance_evidence = "lattice_lidar_checked" if avoidance_intent else "none"
            self.avoidance_direction = float(np.sign(lattice["target"]))
            postprocess = dict(postprocess, mode="LATTICE", reason=lattice["reason"],
                               applied_peak_m=lattice["target"], validated_static=True,
                               lattice={k: v for k, v in lattice.items() if k not in ("path", "offsets")})
        # emergency stop is judged against the path the follower will actually drive (the bypass, if any), not the base route:
        # an object beside the route that the S-curve already clears must not freeze the car (deadlock seen 2026-09-27 21:46)
        self.zone_kmh, self.zone = zone_speed_kmh(self.speed_zones, int((status.get("path") or {}).get("nearest_idx", 0)))
        # Model mode: obstacles and LiDAR route blocks count only where it plans (obstacle
        # sections, a waited-out lead, never a junction); elsewhere roadside returns
        # within 1.5 m of the route would brake the car for nothing.
        route_block = None if (MODEL_AVOID and not model_context["planning"]) else lid_route
        if self.path_publisher and MODEL_AVOID:
            # E2E: approach distance = what the planner itself must go around (footprint
            # hazard, any distance) or a LiDAR block on the route; objects beside the
            # footprint impose nothing (model_static_speed_cap_mps).
            approach_distance = min([d for d in (model_context["hazard_m"],
                                                  route_block[0] if route_block else None)
                                     if d is not None], default=None)
            self.path_publisher.zone_mps = model_static_speed_cap_mps(
                self.zone_kmh / 3.6, static_plan, approach_distance,
                speed_mps=max(0., float(ego.get('speed_kmh', 0.)))/3.6)
            if soft_lane_recovery:
                self.path_publisher.zone_mps = min(self.path_publisher.zone_mps, 12. / 3.6)
        elif self.path_publisher:
            stationary_ahead = [o for o in objs_snap if o[4] == "obstacle" or
                                (o[4] == "pedestrian" and o[5] < 0.3) or
                                (o[4] == "npc" and o[5] < 0.5)]
            early_hazard = route_hazard(stationary_ahead, ego, dense_route, 45.0, 1.4)
            approach_distance = min([d for d in (
                early_hazard[0] if early_hazard else None,
                route_block[0] if route_block else None) if d is not None], default=None)
            self.path_publisher.zone_mps = static_speed_cap_mps(
                self.zone_kmh / 3.6, static_plan, approach_distance)
            if USE_LATTICE:
                # The lattice speed already brakes for the first obstacle on
                # the chosen path; obstacles beside it impose nothing.
                self.path_publisher.zone_mps = min(self.zone_kmh / 3.6, lattice["speed_cap_mps"])
            elif (STRICT_STATIC_STOP and approach_distance is not None and
                    (static_plan is None or static_plan.get("blocked"))):
                # No validated bypass yet: never approach faster than a stop 4 m short of the
                # obstacle allows (competition run hit a CargoBox when "no bypass" came at 3.6 m).
                self.path_publisher.zone_mps = min(self.path_publisher.zone_mps,
                                                   blocked_static_stop_cap_mps(approach_distance))
            if soft_lane_recovery:
                self.path_publisher.zone_mps = min(self.path_publisher.zone_mps, 12. / 3.6)
        static_stop_distance = (lattice["clear_m"] if USE_LATTICE and lattice["blocked"] else None) if USE_LATTICE else min(
            [d for d in (static_hazards[0]["front"] if static_hazards else None,
                         route_block[0] if route_block else None) if d is not None], default=None)
        if static_plan_blocked and self.path_publisher and static_stop_distance is not None:
            self.path_publisher.zone_mps = min(
                self.path_publisher.zone_mps, blocked_static_stop_cap_mps(static_stop_distance))
        bypassing = postprocess["mode"] != "ROUTE BASE"
        path_for_stop = hybrid_path if bypassing else dense_route
        path_for_stop = RosPathPublisher.control_geometry(path_for_stop, dense_route, postprocess)
        postprocess['canonical_control'] = True
        # A live dynamic stop must feed the existing micro-bypass generator,
        # rather than repeatedly publishing its original blocked centre path.
        # Its candidate callback checks actual motion without changing the
        # publication stop latch; all normal traffic/stop checks still follow.
        provider = getattr(self.path_publisher, '_safety_provider', None)
        recovery = (provider.recovery_inputs(ego, 'LATTICE', requested_mps=self.zone_kmh/3.6)
                    if USE_LATTICE and provider is not None else None)
        if recovery is not None:
            def recovery_geometry(candidate):
                # Match publication's dtype and rear-point trimming before
                # projecting any stopping distance. Checking the generator's
                # untrimmed path gave a positive gap that disappeared on the
                # final control path, returning every accepted bypass to zero.
                return RosPathPublisher.control_geometry(np.asarray(candidate, dtype=np.float32),
                    dense_route, dict(mode='LATTICE', validated_static=True, canonical_control=False))
            def recovery_clearance(candidate, horizon):
                checked = recovery_geometry(candidate)
                clear = recovery['clearance'](checked, horizon)
                return min(clear, self.tunnel.path_clearance(checked, ego, horizon)) if entry_corridor else clear
            motion_pace = recovery.get('speed_cap')
            def recovery_speed_cap(candidate, horizon):
                return motion_pace(recovery_geometry(candidate), horizon)
            replacement = lattice_plan(dense_route, ego, lid_static_pts, static_objects+recovery['objects'],
                lane_corridor, previous_lattice, model_path=predicted_path, road_corridor=road,
                paved_road_only=paved_road_only,
                right_shoulder_m=0. if NO_SHOULDER_IDX[0] <= route_index <= NO_SHOULDER_IDX[1] else None,
                cruise_mps=self.zone_kmh/3.6, max_curvature=MAX_CONTROL_CURVATURE,
                max_offset_m=5. if entry_corridor else 3.5, extra_clearance=recovery_clearance,
                extra_speed_cap=recovery_speed_cap if motion_pace is not None else None)
            if not replacement['blocked']:
                lattice = replacement
                static_plan_blocked = False
                hybrid_path = np.asarray(lattice['path'], dtype=np.float32)
                postprocess = dict(postprocess, mode='LATTICE', validated_static=True,
                    reason='live dynamic stop: checked micro-bypass; '+lattice['reason'],
                    applied_peak_m=lattice['target'],
                    lattice={k: v for k, v in lattice.items() if k not in ('path', 'offsets')},
                    dynamic_recovery_ids=recovery['ids'])
                path_for_stop = RosPathPublisher.control_geometry(hybrid_path, dense_route,
                    dict(postprocess, canonical_control=False))
                self.path_publisher.zone_mps = min(self.zone_kmh/3.6, lattice['speed_cap_mps'])
                self._lattice_prev = dict(stamp=now, target=lattice['target'], line=None,
                    blocked=False, route_index=route_index, offsets=lattice.get('offsets'),
                    world_path=[RosPathPublisher.global_xy(ego, *p) for p in path_for_stop])
                bypassing = True
        # Judge collision on the actual path at a braking distance, including
        # controller latency; a fixed 4 m check is too late at 20-25 km/h.
        objs_now = self.object_receiver.snapshot() if self.object_receiver else []
        objs_now = self.intersection_objects(objs_now, ego)
        if (USE_LATTICE or MODEL_AVOID) and not in_static_obstacle_zone:
            objs_now = [o for o in objs_now if o[4] == "npc"]
        objs_now = self.refine_pedestrians(objs_now, ego)
        objs_now = self.deduplicate_objects(objs_now, now)
        # A walker's returns painted "vehicle" become a fusion_npc car box moving with
        # them: the gap planner, lead follower and live check each stopped for it
        # after the person was released. The person row owns that body.
        ghost_filter = getattr(self, "_person_ghost_filter", None)
        if ghost_filter is None:
            ghost_filter = self._person_ghost_filter = PersonGhostFilter()
        objs_now = ghost_filter(objs_now, ego, now, runtime_rtf(self))
        checked_people = {int(o[11]) for o in objs_now if o[4] == 'pedestrian' and len(o)>11
                          and int(o[11]) in postprocess.get('dynamic_recovery_ids', ())}
        objs_all = list(objs_now)
        if checked_people:
            # These people already constrained this selected path's geometry
            # and pace. A second position-only hold on the old route re-armed
            # a stop after every successful micro-bypass. The live publisher
            # still checks all people, fresh motion and raw body clearance.
            objs_now = [o for o in objs_now if o[4] != 'pedestrian' or int(o[11]) not in checked_people]
            if not any(o[4] == 'pedestrian' for o in objs_now):
                self._pedestrian_hold_until = 0.
                previous_yield = getattr(self, 'last_yield', None)
                if previous_yield is not None and 'pedestrian' in str(previous_yield.get('name', '')):
                    self._yield_stop_until = 0.
        # Moving people (and people not tracked long enough to tell) are decided by the
        # pedestrian guard; the older per-case rules below only see standing people.
        ped_guard = self.pedestrian_guard(objs_now, ego, path_for_stop, now,
                                          anticipate_crossing=in_obstacle_zone,
                                          points=lid_pts, seen_objects=objs_all,
                                          checked_standing=(static_plan or {}).get('standing_bypass_ids', ())
                                              if MODEL_AVOID and not USE_LATTICE else ())
        if not checked_people:
            objs_all = objs_now
        objs_now = [o for o in objs_now if not (o[4] == "pedestrian" and int(o[11]) in ped_guard["ids"])]
        brake_horizon = max(4.0, v_now * 0.7 + v_now * v_now / (2.0 * 2.5) + 2.0)
        in_junction = getattr(self, "_intersection_vehicle_status", None) is not None
        path_objects = ([o for o in objs_now if o[4] != "npc" and (not USE_LATTICE or o[4] == "pedestrian")
                         and not ((USE_LATTICE or MODEL_AVOID) and in_junction)]
                        if self.object_receiver else [])
        normal_static_guard = bool(MODEL_AVOID and not USE_LATTICE
            and model_context.get('steady_static') and postprocess.get('static_rear_axle')
            and not in_junction and not any(o[4] == 'pedestrian' for o in objs_all))
        guard_ego, guard_path, guard_points = ego, path_for_stop, lid_static_pts
        guard_frame = None
        if normal_static_guard:
            guard_ego, guard_path, guard_points, guard_frame = current_static_guard_frame(
                self, ego, path_for_stop)
        on_path, fresh_static_guard = fresh_road_static_hazard(path_objects,
            guard_ego if guard_ego is not None else ego,
            guard_path if guard_path is not None else np.empty((0, 2)),
            brake_horizon if bypassing else max(stop_m, brake_horizon),
            .95 if bypassing else 1.15, guard_points, enabled=normal_static_guard,
            margin_recovery_state=(getattr(self, '_road_margin_recovery_ledger', None)
                                   if normal_static_guard else None),
            source_age_s=float((guard_frame or {}).get('age_s', 0.)))
        if guard_frame is not None:
            fresh_static_guard['road_scan_frame'] = guard_frame
            if not guard_frame['valid']:
                on_path = (0., 0., 'static guard measured frame unavailable', 'obstacle', 0.)
        checked_end_cap = fresh_static_guard.get('checked_path_end_cap_mps')
        if self.path_publisher is not None and checked_end_cap is not None:
            self.path_publisher.zone_mps = min(self.path_publisher.zone_mps, checked_end_cap)
        static_braking_cap = fresh_static_guard.get('static_braking_cap_mps')
        if (on_path is not None and on_path[2] == 'fusion_obstacle'
                and static_braking_cap is not None and self.path_publisher is not None
                and guard_frame is not None and guard_frame['valid']):
            self.path_publisher.zone_mps = min(self.path_publisher.zone_mps, static_braking_cap)
            on_path = None
        object_stale = bool(self.object_receiver and getattr(self.object_receiver, "age_s", 999.0) > 0.5) and self.clearance != "lidar"
        # Measured moving cars are checked below with timed full footprints.
        # Applying the raw static corridor to the same returns parks the ego
        # behind a car that has already been checked to clear before arrival.
        # Unknown cars and every unassociated point keep the static guard.
        intersection_status = getattr(self, "_intersection_vehicle_status", None) or {}
        timed_cars = [o for o in objs_now if o[3] == "lidar_car" and o[5] > .5
                      and lidar_vehicle_velocity(o) is not None] if intersection_status.get("detector_ready") else []
        stop_points = without_moving_objects(lid_pts, highway_measured_objects(timed_cars), ego) if timed_cars else lid_pts
        self._lidar_log["timed_vehicle_points_n"] = int(len(lid_pts) - len(stop_points)) if lid_pts is not None else 0
        lid_path = self.lidar.blocked(np.asarray(path_for_stop)[:, :2], 1.1, 30.0, pts=stop_points) if stop_points is not None else None
        self._lidar_log["path_block_m"] = round(lid_path[0], 1) if lid_path else None
        if self.clearance != "objects":
            lim = brake_horizon if bypassing else max(stop_m, brake_horizon)
            if USE_LATTICE:
                lim = 0. if in_junction else 3.0   # last-resort contact guard; cars only in a junction
            elif MODEL_AVOID and (in_junction or not model_context["planning"]):
                lim = 0. if in_junction else 3.0   # outside the planning area: the lattice's contact guard
            lid_stop = (lid_path[0], lid_path[1], "lidar", "obstacle", 0.0) if (lid_path is not None and lid_path[0] <= lim) else None
            on_path = lid_stop if self.clearance == "lidar" else (on_path or lid_stop)
        lidar_stale = self.clearance != "objects" and lid_pts is None
        # A failed candidate is a planned stop while the known obstacle still
        # leaves braking room. Keep the independent path/stream safety stops.
        critical_static = (lattice["emergency"] if USE_LATTICE else
                           static_plan_blocked and
                           (static_stop_distance is None or
                            static_stop_distance <= max(4.0, v_now * .25 + v_now * v_now / 6.0 + 2.5)))
        # Committed roundabout entry (user rule 2026-10-10): following only. Something on
        # our path ahead brakes us down behind it (to a standstill only as following does);
        # people still stop us, and a lost LiDAR scan / pose keeps its sensor stop.
        committed_junction = bool(in_junction and getattr(self.gap_planner, "junction_committed", False))
        self._committed_brake_mps = None
        if committed_junction:
            if on_path and str(on_path[3]) != "pedestrian":
                self._committed_brake_mps = following_speed_cap(
                    max(0., float(on_path[0]) - FRONT_BUMPER_M), 0., v_now)
                on_path = None
            object_stale = False      # the LiDAR vehicle tracks and raw sweep still guard us
        emergency = bool(on_path) or critical_static or object_stale or lidar_stale
        ego_mps_now = float(ego.get("speed_kmh", 0.0)) / 3.6
        entry_ego = None
        live_junction, junction_mode = False, {}
        junction_body_cap = (float(self.path_publisher.zone_mps) if self.path_publisher
                             else self.zone_kmh/3.6)
        try:
            refreshed = self.refresh_intersection_objects(objs_now, ego)
            if refreshed is not objs_now:
                objs_now = refreshed
                objs_all = [o for o in objs_all if o[3] != 'lidar_car'] + [
                    o for o in refreshed if o[3] == 'lidar_car']
            intersection_status = getattr(self, "_intersection_vehicle_status", None)
            self.gap_planner.ACCEL_MPS2 = INTERSECTION_ACCEL_MPS2 if intersection_status else GapPlanner.ACCEL_MPS2
            self.gap_planner.entry_cap_mps = INTERSECTION_ENTRY_MPS if intersection_status else None
            if intersection_status and getattr(self.stop_line_controller, 'yield_only', False):
                entry_front_gap = float(intersection_status['active_region']['stop_station_m']) \
                    - intersection_front_station(ego, self.lanes.route_progress)
                self.gap_planner.entry_cap_mps = intersection_entry_cap_mps(entry_front_gap)
            if intersection_status and intersection_status.get("station_m", -1e9) >= (
                    (intersection_status.get("active_region") or {}).get("stop_station_m", 1e9) - 2.3175 + 3.):
                self.gap_planner.entry_cap_mps = None   # inside: accelerate out
            self.gap_planner.entry_cap_distance_m = (max(0.,
                intersection_status["active_region"]["stop_station_m"] + 12. -
                intersection_status["station_m"]) if intersection_status else 0.)
            # Junction entry mode: wait at the line / follow the passing ring car in
            # (ring_entry via GapPlanner._evaluate_junction), following only once committed.
            line_ctrl = getattr(self, "stop_line_controller", None)
            self._empty_entry_world_path = np.asarray([RosPathPublisher.global_xy(ego, *p[:2]) for p in path_for_stop])
            empty_candidate = bool(line_ctrl is not None and line_ctrl.allow_empty_entry
                and not line_ctrl.yield_only
                and not objs_all and empty_intersection_observation(intersection_status)
                and postprocess.get('mode') == 'ROUTE BASE')
            entry_ego = fresh_entry_pose(self, ego) if empty_candidate else None
            entry_front_gap = (float((intersection_status.get('active_region') or {})['stop_station_m'])
                - intersection_front_station(entry_ego, self.lanes.route_progress)) if entry_ego is not None else math.inf
            rolling_available = bool(entry_ego is not None and line_ctrl.rolling_entry_available(
                entry_front_gap, abs(float(entry_ego['speed_kmh']))/3.6))
            # The rolling proof owns only the last five metres. Requiring its
            # tighter source deadline along the entire approach can repeatedly
            # brake a clear lane while the ordinary stop controller is enough.
            empty_geometry = False
            if rolling_available and entry_front_gap <= ROLL_CHECK_FRONT_GAP_M:
                # Map/path hashing is independent of pose. Take the coherent
                # entry sensor frame after this work, keeping the world path fixed.
                path_geometry = certified_path_geometry(self.lanes, self._empty_entry_world_path)
                entry_ego = fresh_entry_pose(self, ego)
                if entry_ego is not None:
                    entry_front_gap = (float(intersection_status['active_region']['stop_station_m'])
                        - intersection_front_station(entry_ego, self.lanes.route_progress))
                    empty_geometry = bool(path_geometry
                        and entry_front_gap <= ROLL_CHECK_FRONT_GAP_M
                        and line_ctrl.rolling_entry_available(entry_front_gap,
                            abs(float(entry_ego['speed_kmh']))/3.6)
                        and certified_pose_geometry(self.lanes, entry_ego))
            entry_speed = abs(float(entry_ego['speed_kmh']))/3.6 if entry_ego is not None else math.inf
            empty_front_gap = entry_front_gap if empty_geometry else math.inf
            if empty_geometry and empty_front_gap <= ROLL_CHECK_FRONT_GAP_M and entry_speed <= ROLL_MPS:
                # A failed rolling certificate falls back to the normal stop/gap
                # policy. It must not latch an unrelated zero-clearance raw stop.
                proof = {}
                raw = entry_scan_points(self, entry_ego, intersection_status, time.monotonic(), proof=proof)
                empty_geometry = bool(raw is not None and raw_entry_clearance(self.lanes, entry_ego, raw,
                    self._empty_entry_world_path) >= braking_reserve(entry_speed, MAX_ENTRY_SOURCE_AGE_S)
                    and entry_proof_fresh(proof, time.monotonic()))
            junction_line = line_ctrl.active if line_ctrl is not None else None
            if junction_line is None and line_ctrl is not None and intersection_status:
                line_id = (intersection_status.get('active_region') or {}).get('line_id')
                junction_line = next((line for line in line_ctrl.lines if line['id'] == line_id), None)
            line_gap = None
            if line_ctrl is not None and not line_ctrl.released and intersection_status:
                stop_station = (junction_line['station_m'] if junction_line is not None
                    else (intersection_status.get('active_region') or {}).get('stop_station_m'))
                if stop_station is not None:
                    front_station = (intersection_front_station(ego, self.lanes.route_progress)
                        if line_ctrl.allow_empty_entry else
                        float(intersection_status['station_m'])+line_ctrl.FRONT_M)
                    line_gap = float(stop_station)-front_station
            recovery = getattr(self, 'junction_recovery', None)
            recovery_context = (recovery.context(ego, line_ctrl, self.lanes,
                self.intersection_regions, time.monotonic(), observe_pose=False)
                if recovery is not None else None)
            self.gap_planner.junction = (dict(
                line_id=(junction_line['id'] if junction_line is not None
                         else (intersection_status.get('active_region') or {}).get('line_id')),
                line_station_m=(float(junction_line['station_m']) if junction_line is not None else
                                (intersection_status.get('active_region') or {}).get('stop_station_m')),
                held=bool(line_ctrl is not None and not line_ctrl.released
                    and (line_ctrl.active is not None or (line_ctrl.allow_empty_entry
                        and (intersection_status.get('active_region') or {}).get('line_id')
                        not in line_ctrl.completed))),
                yield_only=bool(line_ctrl is not None and line_ctrl.yield_only),
                yield_advance_m=float((junction_line or {}).get('yield_advance_m', 0.)),
                recovery_only=bool(recovery_context and recovery_context['line_id'] ==
                    (intersection_status.get('active_region') or {}).get('line_id')),
                following_only=line_ctrl is None,
                constant_ring_speed=RING_CONSTANT_SPEED,
                allow_empty_entry=bool(line_ctrl is not None and line_ctrl.allow_empty_entry),
                empty_observation=empty_geometry,
                # Use the physical body throughout this opt-in entry, including
                # mandatory fallback and committed following. A state change
                # cannot make the same observed obstacle disappear from safety.
                empty_actual_body=bool(line_ctrl is None or line_ctrl.allow_empty_entry),
                empty_entry_clearance_m=EMPTY_ENTRY_CLEARANCE_M if empty_geometry else 0.,
                # The stop line has commanded (or completed) the stop: at the line, not approaching.
                at_line=bool(line_ctrl is not None and getattr(line_ctrl, "at_line", False)),
                line_gap_m=line_gap,
                ready=bool(intersection_status.get("detector_ready")),
                unknown_ids=set(intersection_status.get("unknown_ids") or ()),
                ghost_ids=set(intersection_status.get("ghost_ids") or ()),
                inherited_ids=set(intersection_status.get("inherited_ids") or ()),
                # Front bumper past the surveyed stop line (also after the line is released).
                stop_station_m=(intersection_status.get("active_region") or {}).get("stop_station_m"),
                entry_m=((float(intersection_status["station_m"]) + FRONT_BUMPER_M -
                          float(intersection_status["active_region"]["stop_station_m"]))
                         if (intersection_status.get("active_region") or {}).get("stop_station_m") is not None
                         and intersection_status.get("station_m") is not None else None))
                if intersection_status else None)
            use_objs = bool(self.object_receiver) and self.clearance != "lidar"
            # Timed checks against ego speed use simulator units. People were
            # already converted by refine_pedestrians (PedTracker).
            rtf = runtime_rtf(self)
            objs_sim = to_sim_objects(objs_now, rtf, people=False)
            yielding = moving_conflict([o for o in objs_sim if o[4] != "npc"], ego, path_for_stop, ego_mps_now) if use_objs else None
            if yielding is not None:
                junction_body_cap = min(junction_body_cap, max(0., float(yielding.get('v_mps', 0.))))
            # vehicles BEHIND us going our way are never a reason to stop: hs_7 t+554 the planner braked for a truck accelerating
            # behind us (predicted footprint overlap) and got rear-ended at 28 km/h. Only objects ahead of the rear bumper are planned on.
            def _ahead_or_crossing(o):
                if o[4] != "npc" or len(o) < 9:
                    return True
                f, l = local_xy(o[0], o[1], ego)
                motion_heading = vehicle_motion_heading(o)
                if motion_heading is None:
                    return True
                d_hd = (motion_heading - math.radians(float(ego["heading_deg"])) + math.pi) % (2 * math.pi) - math.pi
                return not (f < -2.0 and abs(d_hd) < math.radians(35))
            live_provider = getattr(self.path_publisher, '_safety_provider', None)
            junction_mode = self.gap_planner.junction or {}
            live_junction = bool((junction_mode.get('following_only')
                or (junction_mode.get('yield_only') and junction_mode.get('constant_ring_speed')
                    and junction_mode.get('yield_advance_m', 0.) > 0.
                    and not junction_mode.get('recovery_only')))
                and isinstance(live_provider, LivePathSafety) and live_provider.runtime is self)
            if live_junction:
                if junction_mode.get('following_only'):
                    self.gap_planner.junction_committed = False
                self.gap_planner.junction_wait_since = None
                self.gap_planner.followable_ids = set()
                self.gap_planner.risk_details = []
                self.gap_planner.state = 'LIVE_CHECK'
                self.gap_planner.status = dict(state='LIVE_CHECK',
                    reason='junction timing belongs to the current published path')
            # The live publisher owns current ring timing. An older GUI WAIT
            # must not revoke a source-checked launch during actuator dead time.
            # Avoid a second map prediction before the fresh publication check.
            traffic = self.gap_planner.evaluate(
                [o for o in objs_sim if _ahead_or_crossing(o)], ego, path_for_stop,
                min(self.zone_kmh / 3.6, self.path_publisher.zone_mps if self.path_publisher else self.zone_kmh / 3.6),
                now, fresh=not object_stale) if use_objs and not live_junction else None
            # Pedestrian stops and slower pedestrian caps retain priority.
            if traffic is not None and not (self.gap_planner.junction or {}).get('following_only'):
                # No-line junction timing is evaluated on the final published
                # path with the latest sensor frame by LivePathSafety. Keeping
                # this older GUI cap as well would hide a newly opened gap.
                if yielding is None or traffic["kind"] == "stop":
                    yielding = traffic
                elif yielding["kind"] != "stop" and yielding["kind"] == "slow":
                    traffic["v_mps"] = min(traffic["v_mps"], 6.0 / 3.6)
                    yielding = traffic
        except Exception as exc:   # a malformed object row must never stop the decision loop
            yielding = {"kind": "stop", "name": "prediction_error", "along": 0.0, "t": 0.0, "v_mps": 0.0}
            if not getattr(self, "_yield_err", False):
                self._yield_err = True; print(f"[yield] stopping after error: {exc!r}", flush=True)
        # The fallback checks unclassified cross traffic. A vehicle already
        # classified as our merge leader remains under the planner's timed
        # footprint and headway checks, even while its heading is turning.
        try:
            # Committed junction entry: the timed footprint check above owns cross
            # traffic. This position-only fallback stopped us at 16 km/h for a car still
            # 10 m to our left - parking us in its path (roundabout sim, run 222 pattern).
            committed_entry = bool(getattr(self, "_intersection_vehicle_status", None)
                                   and getattr(self.gap_planner, "junction_committed", False))
            if ((yielding is None or yielding.get("kind") != "stop") and not committed_entry
                    and not (live_junction and junction_mode.get('yield_only'))
                    and not (self.gap_planner.junction or {}).get('following_only')):
                caution = crossing_caution(to_sim_objects(objs_now, runtime_rtf(self), people=False),
                                           ego, self.gap_planner.followable_ids)
                if caution is not None and (
                        not (self.gap_planner.junction or {}).get('following_only')
                        or yielding is None
                        or float(caution['v_mps']) < float(yielding.get('v_mps', 0.))):
                    # The fallback may tighten a timed junction command. It
                    # must never raise its braking cap toward an occupied lane.
                    yielding = caution
        except Exception:
            pass
        # Preserve every detected conflict. The independent stop-line gate below
        # adds a mandatory stop even when no object is currently classified.
        self.yield_cap_mps = None
        junction = getattr(self, "_intersection_vehicle_status", None) or {}
        region = junction.get("active_region") or {}
        # Front bumper 3 m past the stop line: inside the circulating lane.
        inside_junction = bool(region and junction.get("station_m", -1e9) >=
                               region.get("stop_station_m", 1e9) - 2.3175 + 3.)
        junction_committed = bool(junction and getattr(self.gap_planner, "junction_committed", False))
        if (inside_junction and not junction_committed
                and not (self.gap_planner.junction or {}).get('following_only')
                and not getattr(getattr(self, 'stop_line_controller', None), 'yield_only', False)):
            # Past the line we only get by a release; never fall back to waiting there.
            self.gap_planner.latch_junction_commit()
            junction_committed = True
        if (junction_committed and yielding and yielding.get("kind") in ("stop", "slow")
                and "pedestrian" not in str(yielding.get("name"))):
            # Committed: no stop/hold that waits for a gap (the planner itself only
            # returns following caps then). Any other vehicle/obstacle stop becomes the
            # braking cap behind it on our path.
            along = float(yielding.get("along", 0.) or 0.)
            cap = following_speed_cap(max(0., along - FRONT_BUMPER_M), 0., ego_mps_now)
            yielding = dict(yielding, kind="follow", v_mps=cap,
                            reason="junction committed: brake behind (was " + str(yielding.get("reason")) + ")")
            self._yield_stop_until = 0.
        if self._committed_brake_mps is not None:
            brake = self._committed_brake_mps
            if yielding is None or (yielding.get("kind") != "stop" and float(yielding.get("v_mps", 1e9)) > brake):
                yielding = {"kind": "follow", "name": "obstacle_on_path", "along": 0., "t": 0., "v_mps": brake,
                            "reason": "junction committed: brake behind what is on our path"}
        # A person who stays put (not crossing) gets PEDESTRIAN_WAIT_S, then we
        # steer around them (user rule 2026-10-03: stood 20+ s beside a
        # standing cyclist). Someone directly ahead in our lane still stops us.
        # Lateral from the path we will drive (a checked bypass), not from the car:
        # the car-relative 2.8 m band held a bypass that kept 1.7 m from a standing
        # person to 2 km/h for 15 s, and the stall reverse restarted it (sim of run 145).
        _path_xy = np.asarray(path_for_stop, dtype=float)[:, :2] if path_for_stop is not None and len(path_for_stop) > 2 else None
        def _path_lateral(f, l):
            return l if _path_xy is None or not bypassing else route_lateral(_path_xy, f, l)
        people = [local_xy(o[0], o[1], ego) for o in objs_now if o[4] == "pedestrian"]
        # Path-relative too, as the hold below: a person 6 m beside the car but on a
        # path curving back to the lane held us while this wait timer never started
        # (car-relative 4.5 m band) - stopped for good (sim of run 145).
        near_people = [(f, l) for f, l in people if -3. < f < 15. and min(abs(l), abs(_path_lateral(f, l))) < 4.5]
        slow_now = float(ego.get("speed_kmh", 0.)) < 5.
        if near_people and slow_now:
            if getattr(self, "_pedestrian_wait_since", None) is None:
                self._pedestrian_wait_since = now
        elif not near_people or float(ego.get("speed_kmh", 0.)) > 12.:
            self._pedestrian_wait_since = None
        if (self.pedestrian_wait_expired(now) and yielding and yielding["kind"] in ("stop", "slow")
                and "pedestrian" in str(yielding.get("name"))
                and not any(0. < f < 8. and abs(l) < 1.3 for f, l in near_people)):
            yielding = dict(yielding, kind="follow", v_mps=PEDESTRIAN_PASS_KMH / 3.6,
                            reason="pedestrian waited %.0f s: pass around" % PEDESTRIAN_WAIT_S)
            self._yield_stop_until = 0.
        if (yielding and yielding["kind"] == "stop" and "pedestrian" in str(yielding.get("name"))
                and float(yielding.get("along", 0.)) > PEDESTRIAN_STOP_GAP_M + 2.3175):
            # Slow early, stop late: full stops 13-18 m short of a crossing
            # person (live 2026-10-04). Approach at a speed that can still stop
            # PEDESTRIAN_STOP_GAP_M short of where they cross; stop only there.
            room = float(yielding["along"]) - 2.3175 - PEDESTRIAN_STOP_GAP_M
            yielding = dict(yielding, kind="follow",
                            v_mps=max(3. / 3.6, math.sqrt(2. * 2.0 * max(0., room))))
        # A person moving near the road just ahead: hold a steady stop until
        # they have moved on, instead of stop-and-creep (user 2026-10-04).
        def _heading_into_lane(o, l, f=0.):
            # People walking along the shoulder are not "crossing": stopping for them
            # parked us in their path mid-bypass and they walked into us (run 102 / sim).
            l = route_lateral(dense_route, f, l)
            if abs(l) < 2.2:
                return True
            if len(o) <= 10:
                return False
            h = math.radians(float(ego["heading_deg"]))
            vl = -math.sin(h) * float(o[9]) + math.cos(h) * float(o[10])
            return vl * math.copysign(1., l) < -.3
        def _lateral_reach(o, l):
            # Where a walker will be within 2 s: one 4 m to the side walking at us was
            # outside the 3 m band, so we crept 3 m into its line (sim of auto run 107).
            if len(o) <= 10:
                return abs(l)
            h = math.radians(float(ego["heading_deg"]))
            vl = -math.sin(h) * float(o[9]) + math.cos(h) * float(o[10])
            return min(abs(l), abs(l + 2. * vl))
        moving_people = [(local_xy(o[0], o[1], ego)[0],
                          _lateral_reach(o, _path_lateral(*local_xy(o[0], o[1], ego))))   # from the path we drive
                         for o in objs_now
                         if o[4] == "pedestrian" and float(o[5]) > .4   # fused speed jitters to 0.37
                         and _heading_into_lane(o, local_xy(o[0], o[1], ego)[1], local_xy(o[0], o[1], ego)[0])]
        # Ahead of our front bumper only: a person already beside the body is passed,
        # not waited for - stop-go beside walkers let them walk into our side at
        # 1-2 km/h (auto run 138, twice).
        # Hold only for someone on our path or reaching within PASS_CLEAR_M of it in
        # 2 s; the 3 m band stopped us for people walking well beside the path.
        if (any(FRONT_BUMPER_M < f < 10. and abs(l) < PASS_CLEAR_M for f, l in moving_people)
                and float(ego.get("speed_kmh", 0.)) < 8. and not self.pedestrian_wait_expired(now)):
            self._pedestrian_hold_until = now + 1.0
        # By position, not fused speed: a walking person read 0.3 m/s, so the rule above
        # never fired and we drove up to 11 km/h into one 3 m ahead (auto runs 94/95).
        # Room to pass (user 2026-10-05: "pass where there is room, stop the stop-go"):
        # a person standing >= PASS_CLEAR_M from our path is passed at
        # PEDESTRIAN_PASS_BY_KMH, not waited for. 450 of the pedestrian stop frames in
        # runs 163-171 were people 1.6-3 m to the side, mostly standing.
        _people = [(o, local_xy(o[0], o[1], ego)) for o in objs_now if o[4] == "pedestrian"]
        def _toward_path(o, lat):
            if len(o) <= 10:
                return False
            h = math.radians(float(ego["heading_deg"]))
            vl = -math.sin(h) * float(o[9]) + math.cos(h) * float(o[10])
            return vl * math.copysign(1., lat) < -.3
        # People beside the path (>= PASS_CLEAR_M) who are not walking toward it are just
        # passed - standing or walking along the shoulder (user 2026-10-05: "just pass;
        # stop and go only for people crossing in front"). Run 206 crawled at 2 km/h
        # behind a shoulder walker 2.1 m to the side.
        # A confirmed stander (no displacement for STANDER_S) is the lattice's job: it is
        # planned around with the pedestrian inflation. Holding for them here re-armed a
        # 10 s wait every time the car crept past (run 497: 11+12+8+8 s beside one
        # person with a free half lane; user: "plenty of room, waits far too long").
        history = getattr(self, "_ped_motion_history", None)
        def _stander(o):
            return (history is not None and history.standing_for(o, now) >= STANDER_S
                    and history.max_speed(o) < .5)
        close_people = [(f, l) for o, (f, l) in _people
                        if FRONT_BUMPER_M < f < 10. and abs(_path_lateral(f, l)) < 2.8
                        and (abs(_path_lateral(f, l)) < PASS_CLEAR_M or _toward_path(o, _path_lateral(f, l)))
                        and not _stander(o)]
        passing_by = [(f, l) for o, (f, l) in _people
                      if -2. < f < 12. and PASS_CLEAR_M <= abs(_path_lateral(f, l)) < 3.5 and float(o[5]) < .4]
        # passing_by people get no cap of their own any more (zone speed applies).
        if close_people and not self.pedestrian_wait_expired(now):
            room = min(f for f, _ in close_people) - 2.3175 - 1.5
            if room <= 2.:
                self._pedestrian_hold_until = now + 1.0
            else:
                cap = math.sqrt(2. * 2. * room)
                if yielding is None or (yielding["kind"] != "stop" and cap < float(yielding.get("v_mps", 99.))):
                    yielding = {"kind": "follow", "name": "pedestrian_near_path", "along": room, "t": 0.,
                                "v_mps": cap, "reason": "pedestrian near our path: approach to stop short"}
        # Preserve a previously detected vehicle/obstacle stop and its source.
        # Replacing it with a pedestrian hold would let TTC cap recovery erase it.
        if (now < getattr(self, "_pedestrian_hold_until", 0.)
                and (yielding is None or yielding.get("kind") != "stop")):
            yielding = {"kind": "stop", "name": "pedestrian", "along": 0., "t": 0., "v_mps": 0.,
                        "reason": "pedestrian moving nearby: wait"}
        yielding = self.apply_pedestrian_guard(ped_guard, yielding, ego, lid_pts, objs_all, now,
                                              path_local=path_for_stop)
        touching = getattr(self, "_contact", None)
        if touching and not touching["block_forward"] and not contact_sweep_ok(
                touching, path_for_stop, max(1.5, touching["ahead_m"] + .5)):
            # The way out is ahead, but the planned path swings the body toward what
            # touches us (auto run 268: a relaxed swerve hit the person at our side).
            touching = dict(touching, block_forward=True, where=touching["where"] + ["path swings toward it"])
            self._contact = touching
        if touching and touching["block_forward"]:
            # Nothing may move us toward / along what touches our front: stop, and back
            # straight off when the rear is free (same checks as the pedestrian reverse).
            yielding = {"kind": "stop", "name": "contact_guard", "along": 0., "t": 0., "v_mps": 0.,
                        "reason": "contact guard: %s within %.2f m" % ("/".join(touching["where"]), touching["gap"])}
            static_contact = bool(MODEL_AVOID and model_context.get('planning')
                                  and not any(o[4] == 'pedestrian' and
                                      math.hypot(*local_xy(o[0], o[1], ego)) < 8. for o in objs_all))
            if not touching["block_reverse"] and not static_contact:
                self.pedestrian_reverse(ego, max(1.0, min(REVERSE_ESCAPE_M, touching["back_m"] or 1.5)),
                                        lid_pts, objs_all, now)
        retained_nontraffic_hold = now < getattr(self, '_yield_stop_until', 0.) and not committed_junction
        traffic_stop_only = bool(not emergency and not retained_nontraffic_hold
                                 and yielding and yielding.get('kind') == 'stop'
                                 and yielding.get('source') == 'traffic')
        traffic_profile_wait = bool(not emergency and not retained_nontraffic_hold
            and getattr(self.stop_line_controller, 'yield_only', False)
            and not committed_junction and yielding
            and yielding.get('source') == 'traffic'
            and yielding.get('kind') in ('stop', 'follow')
            and (self.gap_planner.status.get('ring') or {}).get('action') == 'WAIT')
        if yielding:
            if yielding["kind"] == "stop":
                emergency = True
            elif yielding["kind"] == "slow":
                self.yield_cap_mps = 6.0 / 3.6
            else:
                self.yield_cap_mps = max(0.0, yielding["v_mps"])
        if yielding and yielding["kind"] == "stop" and yielding.get("source") != "traffic":
            self._yield_stop_until = now + 0.7
        if now < getattr(self, "_yield_stop_until", 0.0) and not committed_junction:
            emergency = True
        emergency, stop_line = self.apply_stop_line(
            ego, now, yielding, emergency,
            fresh=not (object_stale or lidar_stale) and float(ego.get("age_s", 999.)) < .5,
            model_actions=self.actions, model_probabilities=probabilities,
            model_fresh=time.perf_counter() - started <= .25 and self.model_camera_fresh(),
            entry_pose=entry_ego, traffic_stop_only=traffic_stop_only,
            committed_traffic_check=lambda: self.gap_planner.evaluate(
                [o for o in objs_sim if _ahead_or_crossing(o)], ego, path_for_stop,
                self.zone_kmh/3.6, time.monotonic(), fresh=not object_stale))
        self.last_yield = yielding
        route_idx = int((status.get("path") or {}).get("nearest_idx", 0))
        self.apply_zone_curve_alat(route_idx)
        in_box_section = any(a <= route_idx <= b and cap >= 30. for a, b, cap in DYNAMIC_ZONES)
        objs_caution = [o for o in objs_all if not (o[4] == "pedestrian" and
                        int(o[11]) in (set(ped_guard["ids"]) | checked_people))]
        caution = dynamic_caution_mps([o for o in objs_caution if o[4] == "pedestrian"] if in_box_section
                                      else objs_caution, ego)
        if caution is not None:
            # People/animals near the road: slow so we could stop short of
            # where they are now (fusion often finds them only 4-12 m ahead).
            self.yield_cap_mps = caution if self.yield_cap_mps is None else min(self.yield_cap_mps, caution)
            self._dynamic_caution_mps = caution
            junction_body_cap = min(junction_body_cap, caution)
        else:
            self._dynamic_caution_mps = None
        # Lead vehicle on the path we drive (any section of the normal road): stopping-
        # distance cap; LiDAR clusters too outside the obstacle sections and junction
        # regions, where nothing else looked at them (run 264).
        lead_hold = None
        try:
            acc = self.lead_follow_step(ego, now, objs_now, lid_pts, path_for_stop,
                                        use_lidar=not in_obstacle_zone and not in_junction,
                                        in_junction=in_junction)
        except Exception as exc:     # never let the follow rule take the decision loop down
            acc = None
            if not getattr(self, "_lead_err", False):
                self._lead_err = True; print(f"[lead] disabled after error: {exc!r}", flush=True)
        map_following = bool(in_junction and
            ((self.gap_planner.junction or {}).get('following_only')
             or (self.gap_planner.junction or {}).get('recovery_only')
             or live_junction))
        if (acc is not None and acc["cap_mps"] is not None and not acc["emergency"] and in_junction
                and (map_following
                     or getattr(self.gap_planner, "junction_committed", False)
                     or (getattr(self.stop_line_controller, 'yield_only', False)
                         and bool((stop_line or {}).get('gap_checked'))))):
            # Committed roundabout entry: the entry planner's following cap (ring_entry.follow_cap)
            # owns the car ahead. This normal-road ACC law (3.5 m standstill gap, 0.5 s
            # reaction, lead braking 4 vs ours 2.5, lead speed projected on our turning path)
            # held 11.6-15.3 km/h behind the ring car we followed in, 5.4-7 m ahead at
            # 12-16 km/h along our path (follow-20261010T130317Z 22:07:03.1-04.3,
            # 22:12:18.7-19.5) while the entry planner allowed 18-22 km/h. Its emergency
            # check below still applies.
            self._lead_state = dict(self._lead_state or {}, cap_applied=False)
        elif acc is not None and acc["cap_mps"] is not None:
            self.yield_cap_mps = acc["cap_mps"] if self.yield_cap_mps is None else min(self.yield_cap_mps, acc["cap_mps"])
            if acc["cap_mps"] < .3:
                lead_hold = "lead vehicle ahead: wait behind it (gap %.1f m)" % acc["state"]["gap_m"]
            if acc["emergency"]:
                emergency = True
                lead_hold = "lead vehicle ahead: emergency brake (gap %.1f m)" % acc["state"]["gap_m"]
        if map_following:
            for person_cap in (ped_guard.get('cap_mps'),
                    yielding.get('v_mps') if yielding is not None and
                    'pedestrian' in str(yielding.get('name')) else None):
                if person_cap is not None:
                    junction_body_cap = min(junction_body_cap, max(0., float(person_cap)))
            postprocess['junction_body_cap_mps'] = max(0., min(junction_body_cap,
                self.zone_kmh/3.6, INTERSECTION_ENTRY_MPS))
        if self.yield_cap_mps is not None and self.path_publisher:
            if in_junction and (map_following or
                                getattr(self.stop_line_controller, 'yield_only', False)):
                # Current ring headway is a command cap, not a permanent ceiling
                # for the future leader trajectory. Preserve the independently
                # checked geometry/static/person ceiling for live gap prediction.
                postprocess['junction_body_cap_mps'] = max(0., min(junction_body_cap,
                    self.zone_kmh/3.6, INTERSECTION_ENTRY_MPS))
            self.path_publisher.zone_mps = min(self.path_publisher.zone_mps, self.yield_cap_mps)
        constant_ring_candidate = bool(RING_CONSTANT_SPEED and in_junction
            and getattr(self.stop_line_controller, 'yield_only', False)
            and (getattr(self.stop_line_controller, 'active', None) or {}).get('yield_advance_m', 0.) > 0.
            and not committed_junction)
        if ((traffic_profile_wait or constant_ring_candidate) and not self._intersection_sensor_stop
                and (constant_ring_candidate or not (acc is not None and acc.get('emergency')))
                and (self.gap_planner.junction or {}).get('ready') is True
                and not (self.gap_planner.junction or {}).get('recovery_only')
                and stop_line is not None
                and (constant_ring_candidate or not stop_line.get('gap_checked'))
                and stop_line.get('distance_m', math.inf) <= INTERSECTION_ENTRY_CAP_FRONT_GAP_M):
            # Preserve the current WAIT, including a controller-captured zero.
            # Only a new full live check may use this independent ceiling to
            # assess a prospective launch instead of forecasting WAIT forever.
            # A GUI WAIT/ACC cap is the current stop command, not the next
            # launch trajectory. With the explicit constant ring policy, ask
            # the independent publisher to check the normal entry ceiling.
            # This proposal retains zero/emergency until fresh raw, person,
            # body, traffic and source-publication checks all accept it.
            profile_cap = (min(self.zone_kmh/3.6, INTERSECTION_ENTRY_MPS)
                           if constant_ring_candidate else junction_body_cap)
            person_cap = ped_guard.get('cap_mps')
            if person_cap is not None:
                profile_cap = min(profile_cap, max(0., float(person_cap)))
            context = self.gap_planner.junction
            postprocess['junction_entry_profile'] = dict(
                ceiling_mps=max(0., min(profile_cap, self.zone_kmh/3.6, INTERSECTION_ENTRY_MPS)),
                line_id=context.get('line_id'), line_station_m=context.get('line_station_m'))
        # The static planner owns low-speed recovery too. A second timer-based
        # S-curve generator used to replace its checked lane path and speed cap.
        if self.path_publisher:
            self.path_publisher.emergency_stop = emergency
        if static_plan is not None or static_plan_blocked:
            self._sensor_bypass_latch = None  # the checked static path owns continuation
        elif postprocess["mode"] in ("STUDENT FORCED", "STUDENT RESIDUAL") and sensor_avoidance:
            direction = 1. if postprocess["applied_peak_m"] > 0. else -1.
            if hazard is not None:
                # Brief LiDAR dropouts may continue an accepted manoeuvre, but
                # model probabilities can never start or renew this timer.
                self._sensor_bypass_latch = dict(direction=direction, until=now + 2.)
            self.avoidance_direction = direction
            self.avoidance_latch_until = self._sensor_bypass_latch["until"]
        else:
            self._sensor_bypass_latch = None
            self.avoidance_direction = 0.0
            self.avoidance_latch_until = 0.0
        postprocess["action_intent"] = action_name if avoidance_intent else None
        postprocess["avoidance_evidence"] = avoidance_evidence
        postprocess["stop_line"] = stop_line
        postprocess['road_scan_frame'] = road_scan_frame
        stop_reasons = []
        if stop_line and stop_line["stop"]: stop_reasons.append(stop_line["reason"])
        if emergency and getattr(self, "_intersection_sensor_stop", False):
            stop_reasons.append("intersection LiDAR not ready")
        if object_stale: stop_reasons.append("object stream stale")
        if static_plan_blocked: stop_reasons.append("no safe static bypass")
        if on_path: stop_reasons.append("obstacle on control path: " + str(on_path[2]))
        if yielding and yielding["kind"] == "stop":
            stop_reasons.append(yielding.get("reason", "yield to " + str(yielding["name"])))
        if emergency and lead_hold: stop_reasons.append(lead_hold)
        if emergency and not stop_reasons: stop_reasons.append("waiting for crossing to clear")
        if (object_stale and not in_junction
                and postprocess.get('mode') == 'ROUTE BASE' and not objs_all
                and not static_plan_blocked and on_path is None and not critical_static
                and not lidar_stale and not retained_nontraffic_hold and not lead_hold
                and not (stop_line or {}).get('stop')
                and yielding and yielding.get('name') == 'object_stream'
                and set(stop_reasons) <= {'object stream stale', 'objects stale'}
                and self.path_publisher is not None):
            # Only a newer complete semantic observation may resolve this
            # freshness-only stop in the final whole-path live check. No age
            # allowance, contact/traffic permission or cached empty frame does.
            postprocess['object_stream_recheck'] = dict(
                source_stamp=getattr(self.object_receiver, 'observation_stamp', None),
                ceiling_mps=float(self.path_publisher.zone_mps))
        planned_static_stop = bool(static_plan_blocked and self.path_publisher and
                                   self.path_publisher.zone_mps <= .05)
        postprocess["motion_state"] = "STOP" if emergency or planned_static_stop or lead_hold else "DRIVE"
        postprocess["stop_reason"] = ("; ".join(stop_reasons) if emergency else
                                      "waiting for safe static bypass" if planned_static_stop else lead_hold)
        # Show the same route-extended path that MPC receives. The learned branch can
        # contain only a handful of forward points and was nearly invisible in the GUI.
        control_path = path_for_stop
        postprocess['entry_reference'] = bool(reference.active)
        model_route = np.asarray(model_route, dtype=np.float32)
        self.last_result = {
            "path": control_path,
            "raw_path": predicted_path,
            "probabilities": probabilities,
            "concept_values": concept_values,
            "scene_matches": cosine_gallery_matches(scene, self.gallery_matrix, self.gallery),
            "trajectory": trajectory_diagnostics(predicted_path, np.asarray(display_route, dtype=np.float32)),
            "hybrid_trajectory": trajectory_diagnostics(
                control_path, np.asarray(display_route, dtype=np.float32),
                min_lateral_m=0.25 if self.args.force_student_avoidance else 0.70),
            "postprocess": postprocess,
            "local_route": np.asarray(display_route, dtype=np.float32),
            "model_route": model_route,
            "latency_ms": latency,
            "status": status,
            "objects": objs_now,
            "lane_corridor": lane_corridor,
        }
        # A forced bypass intentionally leaves the global centreline by up to 4 m.
        # Keep publishing through the manoeuvre and return; the old 2 m gate cut the
        # path stream mid-avoidance and made MPC apply its stale-path safety brake.
        route_distance = float((status.get("path") or {}).get("distance_m", 999))
        if self.path_publisher:
            if (USE_LATTICE and postprocess.get("mode") == "LATTICE") or MODEL_AVOID:
                # MORAI people never avoid us: one walked into our stopped car's front
                # corner and stood there, and we waited for it for good (auto run 107).
                # Backing off gives it room to walk on (rear checked as for any reverse).
                # Only someone on the path we will drive pins us: a person 1.7 m beside
                # a checked bypass, passed at the 2 km/h caution crawl, read as "pinned"
                # (car-relative |l| < 1.9) and the reverse fired mid-pass after 3 s,
                # leaving the car skewed and oscillating for 40 s (sim of run 145).
                pinned = any(o[4] == "pedestrian" and float(o[5]) < .3
                             and 0. < local_xy(o[0], o[1], ego)[0] < 4.5 and abs(local_xy(o[0], o[1], ego)[1]) < 1.9
                             # >= the standing-person stop band in moving_conflict (.946+.3+.35):
                             # a person at 1.55-1.6 m held us there but never counted as pinning,
                             # so no reverse - stopped for good (auto run 180).
                             and abs(route_lateral(hybrid_path, *local_xy(o[0], o[1], ego))) < 0.946 + .3 + .35 + .05   # < the 1.7 m a checked bypass keeps
                             for o in objs_now)
                if MODEL_AVOID:
                    # A static backoff needs a checked forward return at its
                    # endpoint. Every reverse still monitors rear/swing returns.
                    self.reverse_escape_check(ego, bool(static_plan_blocked) or pinned,
                                              0. if pinned else static_stop_distance, lid_static_pts, now,
                                              swing_check=True,
                                              static_recovery=bool(model_context.get('planning') and not pinned))
                else:
                    self.reverse_escape_check(ego, bool(lattice.get("blocked")) or pinned,
                                              0. if pinned else lattice.get("clear_m"), junction_points, now)
            # always publish: the follower's speed cap (/avoid_target_vel) expires after 0.5 s and it would run at its own
            # cruise speed (46 km/h seen 2026-09-27 23:22 while the car was > 6 m from the route after a scenario reset)
            if lattice is not None and not lattice.get('blocked'):
                postprocess['forward_clear_m'] = float(lattice.get('clear_m', 0.))
            signals = getattr(self, 'signal_controller', None)
            if signals is not None:
                signal_plan = signals.evaluate(ego)
                postprocess['traffic_signal'] = signal_plan
                # Keep the safety baseline free of the signal's temporary cap.
                # Final publisher checks below own signal braking, so a fresh
                # permission can release HOLD even while model inference waits.
            self.path_publisher.publish(ego, control_path, dense_route, postprocess)
            if getattr(self.path_publisher, '_safety_provider', None) is not None:
                active = getattr(self.path_publisher, '_active_decision_plan', None)
                if active is not None:
                    self._normal_decision_plan = active
                    postprocess['decision'] = active.summary()
                    if active.emergency:
                        postprocess.update(motion_state='STOP', stop_reason=active.stop_reason)
            self.path_publisher.publish_camera_rates(self.receiver.rates())
            if route_distance > self.args.max_route_distance_m and not getattr(self, "_far_warned", False):
                self._far_warned = True; print(f"[route] ego {route_distance:.1f} m from the route; still publishing the route path", flush=True)
        # decision log (one line per inference) -> /data/autodrive/student_gui_<session|nosave>.jsonl
        try:
            import os
            if not hasattr(self, "_decision_log"):
                sid = os.path.basename(os.environ.get("STUDENT_SESSION_DIR", "")) or "nosave"
                log_dir = os.environ.get("STUDENT_LOG_DIR") or ("/data/autodrive" if os.path.isdir("/data/autodrive") else os.path.dirname(str(self.args.status)) or ".")
                self._decision_log = AsyncDecisionLog(os.path.join(log_dir, f"student_gui_{sid}.jsonl"))
            record = {"t": time.time(), "ego": [round(ego["x"], 2), round(ego["y"], 2)], "speed_kmh": round(ego.get("speed_kmh", 0), 1),
                                                 "target_decision_hz": getattr(self, "decision_target_hz", 15.0),
                                                 "decision_hz": round(getattr(self, "decision_hz", 0.0), 1),
                                                 "action": action_name, "p": round(action_probability, 3), "intent": avoidance_intent,
                                                 "model_avoidance_intent": model_avoidance_intent,
                                                 "avoidance_evidence": avoidance_evidence,
                                                 "motion_state": "STOP" if self.path_publisher and getattr(self.path_publisher, "_last_target_mps", 0.) <= .05 else postprocess.get("motion_state"),
                                                 "traffic_risks": self.gap_planner.risk_details[:6],
                                                 "mode": postprocess.get("mode"), "reason": postprocess.get("reason"),
                                                 "stop_reason": postprocess.get("stop_reason"), "applied_peak_m": round(float(postprocess.get("applied_peak_m", 0) or 0), 2),
                                                 "raw_peak_m": round(float(postprocess.get("raw_peak_m", 0) or 0), 2), "latency_ms": round(latency, 1),
                                                 "hazard": ({"along_m": round(self.last_hazard[0], 1), "lat_m": round(self.last_hazard[1], 2), "name": self.last_hazard[2]} if self.last_hazard else None),
                                                 "static_corridor": [{"name": d["n"], "front_m": d["front"]} for d in static_hazards[:6]],
                                                 "bypass_hold_m": static_plan["hold_m"] if static_plan else None,
                                                 "bypass_blocked": static_plan_blocked,
                                                 "fresh_static_guard": fresh_static_guard,
                                                 "object_stale": object_stale,
                                                 "clearance": self.clearance, "lidar": getattr(self, "_lidar_log", None), "trig_m": round(trig_m, 1), "stop_m": round(stop_m, 1),
                                                 "traffic_plan": ((postprocess.get('decision') or {}).get('validation', {})
                                                     .get('live_safety', {}).get('junction_traffic', self.gap_planner.status)
                                                     if (self.gap_planner.junction or {}).get('following_only')
                                                     else self.gap_planner.status),
                                                 "stop_line": stop_line,
                                                 "intersection_lidar": getattr(self, "_intersection_vehicle_status", None),
                                                 "decision_log_dropped": self._decision_log.dropped,
                                                 "lattice": postprocess.get("lattice"),
                                                 "dynamic_caution_kmh": round(self._dynamic_caution_mps * 3.6, 1) if getattr(self, "_dynamic_caution_mps", None) else None,
                                                 "static_plan_kind": static_plan.get("kind") if static_plan else None,
                                                 "static_safe_horizon_m": static_plan.get("safe_horizon_m") if static_plan else None,
                                                 "static_clear_distance_m": static_plan.get("clear_distance_m") if static_plan else None,
                                                 "static_path_reused": static_plan.get("reused") if static_plan else False,
                                                 "static_prefix_recovered": static_prefix_recovered,
                                                 "same_direction_recovery": soft_lane_recovery,
                                                 "static_replan_needed": static_plan.get("replan_needed") if static_plan else False,
                                                 "activation_m": static_plan.get("activation_m") if static_plan else None,
                                                 "planning_ms": round(planning_ms, 1), "decision_ms": round((time.perf_counter() - started) * 1000., 1),
                                                 "static_rejected": static_rejected,
                                                 "static_stop_distance_m": round(static_stop_distance, 1) if static_stop_distance is not None else None,
                                                 "critical_static": bool(critical_static),
                                                 "static_cluster_count": static_plan.get("cluster_count") if static_plan else 0,
                                                 "static_hold_until_m": static_plan.get("hold_until_m") if static_plan else None,
                                                 "path_curvature_1pm": static_plan.get("curvature") if static_plan else None,
                                                 "planner_cap_kmh": round(self.path_publisher.zone_mps * 3.6, 1) if self.path_publisher else None,
                                                 "published_cap_kmh": round(getattr(self.path_publisher, "_last_target_mps", 0.0) * 3.6, 1) if self.path_publisher else None,
                                                 "decision": postprocess.get('decision'),
                                                 "emergency_stop": bool(getattr(self.path_publisher, "emergency_stop", False)) if self.path_publisher else False,
                                                 "yield": ({"name": self.last_yield["name"], "kind": self.last_yield["kind"], "along_m": round(self.last_yield["along"], 1), "t_s": round(self.last_yield["t"], 2), "v_kmh": round(self.last_yield["v_mps"] * 3.6, 1)} if getattr(self, "last_yield", None) else None),
                                                 "cap_kmh": round(self.yield_cap_mps * 3.6, 1) if getattr(self, "yield_cap_mps", None) else None, "route_dist_m": round(float((status.get("path") or {}).get("distance_m", -1)), 2),
                                                 "n_obj": getattr(self, "_n_obj", None), "obj_age_s": round(time.monotonic() - self.object_receiver.received, 2) if self.object_receiver else None,
                                                 "obj_src": getattr(self.object_receiver, "src", None) if self.object_receiver else None, "ego_src": ego.get("src", "ros"), "ego_age_s": round(float(ego.get("age_s", 0)), 2),
                                                 "near": [d for d in getattr(self, "_obj_debug", []) if d["f"] > -3.0][:3],
                                                 "target_kmh": self.zone_kmh, "zone": self.zone}
            record["speed_label"] = speed_label(record)
            signals = getattr(self, 'signal_controller', None)
            record['signal_enabled'] = bool(signals and signals.enabled)
            record['traffic_signal'] = dict(signals.last_plan) if signals else {"active": False}
            record["cars"] = self.surround_cars(ego)
            record["ego_h"] = round(float(ego.get("heading_deg", 0.)), 2)
            record["reverse_veto"] = getattr(self, "_reverse_veto", None)
            pg = getattr(self, "_ped_guard_log", None)
            if pg:
                record["ped_guard"] = pg
            if getattr(self, "_contact", None):
                record["contact_guard"] = self._contact
            record["body_gap_m"] = body_gap_m(self.lidar)
            record["lead"] = getattr(self, "_lead_state", None)
            # Logged object velocities are measured units; planners used / rtf.
            record.update(self.sim_clock.record() if getattr(self, "sim_clock", None) is not None
                          else dict(rtf=1., rtf_measured=False))
            record['transition'] = dict(getattr(getattr(self, '_planner_transition', None), 'info', {}))
            record.update(v6_record_fields(self))
            if MODEL_AVOID:
                record["model_avoidance"] = getattr(self, "_model_avoidance_log", None)
            self._web_record = record
            self._decision_log.write(json.dumps(record, default=json_default) + "\n")
        except Exception:
            pass
        return True

    def draw_plan(self, canvas, origin, size):
        from plan_view import render_plan_view

        result = self.last_result
        record = dict(getattr(self, "_web_record", None) or {})
        tunnel_view = bool(result and result.get("postprocess", {}).get("mode", "").startswith("TUNNEL"))
        ego = (result.get("status") or {}).get("ego") if result else None
        points, age = self.lidar.snapshot() if getattr(self, "lidar", None) else (None, None)
        corridor = result.get("lane_corridor") if result else None
        objects = result.get("objects", ()) if result else ()
        if tunnel_view:
            ego = getattr(self, "_tunnel_view_pose", None) or ego
            points = getattr(self, "_tunnel_view_points", None)
            age = (record.get("lidar") or {}).get("age_s")
            corridor = self.tunnel.corridor(ego) if ego and getattr(self, "tunnel", None) else None
            objects = [(o["x"], o["y"], max(o["length_m"], o["width_m"]),
                        o["name"], o["kind"], o.get("speed_mps", 0.),
                        o["length_m"], o["width_m"], o.get("heading_deg", 0.),
                        o.get("vx_mps", 0.), o.get("vy_mps", 0.), o.get("id", o["name"]))
                       for o in (record.get("tunnel") or {}).get("objects", ())]
        lanes = getattr(self, "lanes", None)
        links = lanes.local_links(ego, 70.) if ego and lanes and hasattr(lanes, "local_links") else ()
        render_plan_view(canvas, origin, size, result=result, record=record,
                         lidar_points=points, lidar_age_s=age, links=links,
                         corridor=corridor, objects=objects, ego=ego)

    # Semantic overlay for the window: masks.npz from semantic_sidecar.py (class ids -> BGR, 6-class palette).
    SEMANTIC_BGR = {0: (60, 20, 220), 1: (255, 2, 98), 2: (2, 255, 236), 3: (240, 74, 255), 5: (255, 255, 255)}

    def semantic_tint(self, name, display):
        if os.environ.get("SEMANTIC_OVERLAY", "1") != "1":
            return display
        path = Path(os.environ.get("SEMANTIC_FEED_DIR", "/dev/shm/alpamayo_semantic")) / "masks.npz"
        try:
            mtime = path.stat().st_mtime
            if time.time() - mtime > 1.0:
                return display
            if getattr(self, "_semantic_mtime", None) != mtime:
                self._semantic_masks, self._semantic_mtime = dict(np.load(path)), mtime
            masks = self._semantic_masks
            left, top, pw, ph = (int(v) for v in masks[f"{name}_place"][:4])
            crop = masks[f"{name}_mask"][top:top + ph, left:left + pw]
            crop = cv2.resize(crop, (display.shape[1], display.shape[0]), interpolation=cv2.INTER_NEAREST)
        except Exception:
            return display
        color = np.zeros_like(display)
        for class_id, bgr in self.SEMANTIC_BGR.items():
            color[crop == class_id] = bgr
        painted = crop < 6
        out = display.copy()
        out[painted] = cv2.addWeighted(display, 0.45, color, 0.55, 0)[painted]
        return out

    def draw(self):
        last_result = self.last_result
        canvas = np.full((800, 1280, 3), (22, 25, 29), np.uint8)
        rates = self.receiver.rates()
        if not hasattr(self, "_display_frames"):
            self._display_frames = {}
        with self.receiver.lock:
            frames = {name: self.receiver.frames[name][-1] for name, _ in VIEWS if self.receiver.frames[name]}
        for name, (stamp, jpeg) in frames.items():
            if self._display_frames.get(name, (None,))[0] != stamp:
                decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if decoded is not None:
                    self._display_frames[name] = (stamp, decoded)
        images = [self._display_frames[name][1] for name, _ in VIEWS if name in self._display_frames]
        if len(images) == len(VIEWS):
            for index, ((name, _), image) in enumerate(zip(VIEWS, images)):
                widths = (426, 426, 428)
                x = sum(widths[:index])
                display = cv2.resize(image, (widths[index], 240))
                display = self.semantic_tint(name, display)
                canvas[:240, x:x + widths[index]] = display
                cv2.rectangle(canvas, (x, 0), (x + widths[index], 30), (0, 0, 0), -1)
                cv2.putText(canvas, f"{name.upper()}  {rates[name]:.1f} Hz  SEMANTIC", (x + 10, 21), 0, .55, (255, 255, 255), 2)
        else:
            cv2.putText(canvas, "Waiting for front / left / right cameras", (330, 115), 0, .8, (180, 180, 180), 2)
        self.draw_plan(canvas, (0, 250), (800, 550))
        cv2.rectangle(canvas, (810, 250), (1279, 799), (37, 41, 47), -1)
        y = 282
        title = ("STUDENT PATH -> MPC CONTROL" if self.path_publisher
                 else "STUDENT SHADOW - NO CONTROL OUTPUT")
        title_color = (80, 80, 255) if self.path_publisher else (80, 180, 255)
        cv2.putText(canvas, title, (825, y), 0, .53, title_color, 2); y += 38
        checkpoint_label = f"{self.args.checkpoint.name} | epoch {self.checkpoint.get('epoch', '?')} | 3 cameras x 4 frames"
        cv2.putText(canvas, checkpoint_label, (825, y), 0, .48, (210, 210, 210), 1); y += 25
        if self.input_error:
            cv2.putText(canvas, "EGO INPUT INVALID", (825, y), 0, .62, (70, 70, 255), 2); y += 30
            cv2.putText(canvas, "Connect ws://127.0.0.1:9091", (825, y), 0, .46, (90, 160, 255), 1); y += 24
            cv2.putText(canvas, "Ego Vehicle Status Publish ON", (825, y), 0, .43, (90, 160, 255), 1); y += 30
        if last_result:
            probabilities = last_result["probabilities"]
            order = np.argsort(probabilities)[::-1]
            cv2.putText(canvas, f"latency {last_result['latency_ms']:.1f} ms  GPU {torch.cuda.get_device_name(0) if self.device.type == 'cuda' else 'CPU'}", (825, y), 0, .40, (185, 185, 185), 1); y += 29

            trajectory = last_result["hybrid_trajectory"]
            postprocess = last_result["postprocess"]
            plan_color = (70, 190, 255) if trajectory["plan"] != "FOLLOW ROUTE" else (90, 220, 140)
            cv2.putText(canvas, "POSTPROCESSED SHADOW TRAJECTORY", (825, y), 0, .46, (210, 210, 210), 1); y += 23
            cv2.putText(canvas, postprocess.get("motion_state", "") + " | " + postprocess["mode"], (825, y), 0, .58,
                        (60, 60, 255) if postprocess.get("motion_state") == "STOP" else plan_color, 2); y += 23
            cv2.putText(canvas, postprocess.get("stop_reason") or postprocess["reason"], (825, y), 0, .38, (180, 190, 200), 1); y += 19
            cv2.putText(canvas, f"raw peak {postprocess['raw_peak_m']:+.2f}m end {postprocess['raw_end_m']:+.2f}m | applied {postprocess['applied_peak_m']:+.2f}m", (825, y), 0, .37, (190, 200, 210), 1); y += 24

            matches = last_result["scene_matches"]
            cv2.putText(canvas, "LEARNED SCENE / PLAN RETRIEVAL", (825, y), 0, .43, (210, 210, 210), 1); y += 21
            if matches:
                match = matches[0]
                cv2.putText(canvas, f"cosine similarity {match['similarity']:.3f}  {match['sample_id']}", (825, y), 0, .37, (160, 170, 180), 1); y += 19
                reason = [line.strip() for line in match["text"].splitlines() if line.strip()]
                y = put_wrapped(canvas, reason[-1] if reason else "", (825, y), 435, (100, 210, 255), .39, 1, 3) + 5

            cv2.putText(canvas, "ACTION OUTPUT", (825, y), 0, .43, (210, 210, 210), 1); y += 21
            for index in order[:4]:
                probability = float(probabilities[index])
                color = (80, 220, 130) if index == order[0] else (130, 140, 150)
                cv2.putText(canvas, f"{self.actions[index]:<14} {probability * 100:5.1f}%", (825, y), 0, .43, color, 1)
                cv2.rectangle(canvas, (1025, y - 11), (1025 + int(probability * 180), y), color, -1)
                y += 22

            concept_values = last_result["concept_values"]
            if len(concept_values):
                obstacle_index = self.concepts.index("obstacle")
                road_index = self.concepts.index("road_blocked")
                cv2.putText(canvas, "AUX CONCEPT HEAD: UNUSED / 0 labels", (825, y), 0, .40, (80, 150, 255), 1); y += 20
                cv2.putText(canvas, f"raw only: obstacle {concept_values[obstacle_index]*100:.1f}%  road_blocked {concept_values[road_index]*100:.1f}%", (825, y), 0, .38, (130, 140, 150), 1); y += 24
            ego = last_result["status"].get("ego") or {}
            cv2.putText(canvas, f"speed {ego.get('speed_kmh', 0):.1f} km/h  Ego {last_result['status'].get('ego_hz', 0):.1f} Hz", (825, y), 0, .43, (220, 220, 220), 1); y += 22
            path = last_result["status"].get("path") or {}
            cv2.putText(canvas, f"HD-map idx {path.get('nearest_idx', 0)}  distance {path.get('distance_m', 0):.2f}m", (825, y), 0, .40, (220, 220, 220), 1)
        cv2.putText(canvas, "Q / ESC: close GUI", (825, 780), 0, .45, (160, 160, 160), 1)
        return canvas

    def run(self):
        self.receiver.start()
        self.signal_controller.start()
        if not self.args.headless:
            cv2.namedWindow("MORAI Student v5", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("MORAI Student v5", 1280, 800)
        inference_stop = threading.Event()
        failures = []
        self.decision_target_hz = float(os.environ.get("STUDENT_DECISION_HZ", "15"))
        semantic_hz = float(os.environ.get("SEMANTIC_FEED_HZ", "15"))
        if any(not math.isfinite(hz) or not 1 <= hz <= 60
               for hz in (self.decision_target_hz, semantic_hz)):
            raise ValueError("Student and semantic feed rates must be finite and within 1–60 Hz")
        decision_period = 1.0 / self.decision_target_hz
        self.decision_hz = 0.0
        def infer_loop():
            try:
                count, rate_started = 0, time.monotonic()
                while not inference_stop.is_set():
                    began = time.monotonic()
                    self.infer()
                    if notify_v6_idle:
                        notify_v6_idle(self)   # the v6 child uses the GPU only right after this
                    count += 1
                    elapsed = time.monotonic() - rate_started
                    if elapsed >= 1.0:
                        self.decision_hz = count / elapsed
                        count, rate_started = 0, time.monotonic()
                    inference_stop.wait(max(0., decision_period-(time.monotonic()-began)))
            except Exception as error:
                failures.append(error)
                if self.path_publisher:
                    self.path_publisher.publish_stop('planner inference failed: '+str(error))
                inference_stop.set()
        inference = threading.Thread(target=infer_loop, name="student-inference", daemon=True)
        inference.start()
        # Semantic sidecar feed: hand the latest camera JPEGs (+ MORAI frame stamps) to the semantic
        # ONNX container, which cannot bind the same UDP ports. One atomic file in RAM (/dev/shm),
        # Feed only the latest camera set at the configured rate; never queue old frames.
        semantic_dir = Path(os.environ.get("SEMANTIC_FEED_DIR", "/dev/shm/alpamayo_semantic"))
        def semantic_feed():
            pin_io_thread("semantic frame handoff")
            semantic_dir.mkdir(parents=True, exist_ok=True)
            seq, last = 0, None
            while not inference_stop.is_set():
                began = time.monotonic()
                try:
                    with self.receiver.lock:
                        latest = {name: self.receiver.frames[name][-1] for name, _ in VIEWS if self.receiver.frames[name]}
                    if len(latest) != len(VIEWS):
                        continue
                    stamps = tuple(int(latest[name][0]) for name, _ in VIEWS)
                    if stamps == last:
                        continue
                    last, seq = stamps, seq + 1
                    payload = {"seq": np.int64(seq), "wall": np.float64(time.time())}
                    for name, _ in VIEWS:
                        payload[f"{name}_stamp_ns"] = np.int64(latest[name][0])
                        payload[f"{name}_jpeg"] = np.frombuffer(latest[name][1], np.uint8)
                    tmp = semantic_dir / ".frames.npz.tmp"
                    with open(tmp, "wb") as handle:
                        np.savez(handle, **payload)
                    os.replace(tmp, semantic_dir / "frames.npz")
                except Exception:
                    pass
                finally:
                    inference_stop.wait(max(0., 1.0 / semantic_hz - (time.monotonic() - began)))
        if hasattr(self.receiver, "frames") and hasattr(self.receiver, "lock"):
            threading.Thread(target=semantic_feed, name="semantic-feed", daemon=True).start()

        # Web view (replaces the OpenCV window): camera row with semantic tint + route/plan view as JPEGs and the
        # latest decision as JSON in RAM; the dashboard renders them (decision text translated there).
        def web_view():
            semantic_dir.mkdir(parents=True, exist_ok=True)
            # 7.5 Hz (was 5 Hz): smoother dashboard view (user 2026-10-04).
            while not inference_stop.wait(float(os.environ.get("STUDENT_WEB_VIEW_PERIOD_S", ".133"))):
                try:
                    canvas = self.draw()
                    for part, image in (("gui_cams", canvas[:240]), ("gui_plan", canvas[250:800, :800])):
                        ok, enc = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if ok:
                            tmp = semantic_dir / f".{part}.jpg.tmp"
                            tmp.write_bytes(enc.tobytes())
                            os.replace(tmp, semantic_dir / f"{part}.jpg")
                    record = dict(getattr(self, "_web_record", None) or {})
                    result = self.last_result
                    if result is not None and record.get("action") != "LIDAR":
                        record["probs"] = {str(a): round(float(p), 3) for a, p in zip(self.actions, result["probabilities"])}
                    record["web_time"] = time.time()
                    record["objects_topic"] = os.environ.get("STUDENT_OBJECT_TOPIC", "/Object_topic")
                    record["n_objects"] = getattr(self, "_n_obj", 0)
                    record["model_checkpoint"] = os.path.basename(str(getattr(self.args, "checkpoint", "") or ""))
                    record["planner"] = PLANNER_MODE
                    if getattr(self, "_lap_status", None):
                        record["lap"] = self._lap_status
                    tmp = semantic_dir / ".decision.json.tmp"
                    tmp.write_text(json.dumps(record, default=str))
                    os.replace(tmp, semantic_dir / "decision.json")
                except Exception:
                    pass
        threading.Thread(target=web_view, name="web-view", daemon=True).start()
        try:
            while not inference_stop.is_set():
                if self.args.headless:
                    time.sleep(0.03)
                else:
                    cv2.imshow("MORAI Student v5", self.draw())
                    key = cv2.waitKey(30) & 0xff
                    if key in (27, ord("q"), ord("Q")):
                        break
            if failures:
                raise failures[0]
        finally:
            inference_stop.set()
            inference.join(timeout=3.)
            self.receiver.stop_event.set(); self.receiver.join(timeout=3)  # lets the recorder close manifest/session.json
            if self.path_publisher:
                self.path_publisher.close()
            self.signal_controller.close()
            self.ego_receiver.close()
            if stop_v6_shadow:
                stop_v6_shadow(self)
            if getattr(self, "tunnel_odometry", None):
                self.tunnel_odometry.close()
            if getattr(self, "tunnel_no_go_publisher", None):
                self.tunnel_no_go_publisher.update([], [], None, 0., False)
                self.tunnel_no_go_publisher.close()
            if getattr(self, "tunnel_no_go_receiver", None):
                self.tunnel_no_go_receiver.close()
            if self.object_receiver:
                self.object_receiver.close()
            if getattr(self, "vehicle_receiver", None):
                self.vehicle_receiver.close()
            if getattr(self, "lidar", None):
                self.lidar.close()
            self.speed_zones.file_cache.close()
            if hasattr(self, "_decision_log"):
                self._decision_log.close()
            if not self.args.headless:
                cv2.destroyAllWindows()



def route_lateral(route_local, f, l):
    """Lateral offset of vehicle-frame point (f, l) from the route centreline
    (route_local: dense route in the vehicle frame). Mid-bypass the car is itself
    offset, so car-relative lateral misplaced shoulder walkers (auto run 102 / sim)."""
    if route_local is None or len(route_local) < 2:
        return l
    route_local = np.asarray(route_local, dtype=float)[:, :2]
    j = int(np.argmin(np.hypot(route_local[:, 0] - f, route_local[:, 1] - l)))
    k, i = min(j + 1, len(route_local) - 1), max(j - 1, 0)
    t = route_local[k] - route_local[i]
    t = t / max(1e-6, float(np.hypot(*t)))
    return float((np.array([f, l]) - route_local[j]) @ np.array([-t[1], t[0]]))

def _sigterm(*_):
    raise SystemExit(0)


def main():
    pin_compute_thread("driving compute")
    import signal
    signal.signal(signal.SIGTERM, _sigterm)
    # OpenCV otherwise starts a machine-sized pool for camera/web operations,
    # competing with the 15 Hz LiDAR decision thread even in headless mode.
    cv2.setNumThreads(1)
    torch.set_num_threads(1)
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, default=Path(__file__).resolve().parent / "last.pt")
    parser.add_argument("--route", type=Path, default=root / "data/autodrive/global_path.json")
    parser.add_argument("--status", type=Path, default=root / "data/autodrive/status.json")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument("--camera-source", choices=("udp", "ros", "files"), default="udp",
                        help="udp: MORAI direct JPEG UDP on VIEWS ports (semantic-training rig). "
                             "ros: subscribe sensor_msgs/Image topics instead (competition rig, "
                             "where those UDP ports are already owned by around_view)")
    parser.add_argument("--camera-ros-prefix", default="/around_view/camera",
                        help="used only with --camera-source ros; topics are <prefix>/<front|left|right>/image_raw")
    parser.add_argument("--camera-files-dir", type=Path,
                        help="active collect_session.sh output; required with --camera-source files")
    parser.add_argument("--publish-control-path", action="store_true",
                        help="publish processed path for the MPC follower")
    parser.add_argument("--force-student-avoidance", action="store_true",
                        help="apply every material Student lateral path, retaining only numeric/offset limits")
    parser.add_argument("--headless", action="store_true",
                        help="run inference and ROS publishing without an OpenCV window")
    parser.add_argument("--cruise-kmh", type=float, default=10.0)
    parser.add_argument("--max-route-distance-m", type=float, default=6.0,
                        help="continue publishing while an intentional bypass is off the global route")
    args = parser.parse_args()
    if args.camera_source == "files" and not args.camera_files_dir:
        parser.error("--camera-files-dir required with --camera-source files")
    if args.publish_control_path and not 5.0 <= args.cruise_kmh <= 90.0:
        parser.error("Student MPC control cruise-kmh must be 5~90")
    if not 4.0 <= args.max_route_distance_m <= 10.0:
        parser.error("max-route-distance-m must be 4~10")
    Runtime(args).run()


if __name__ == "__main__":
    main()
