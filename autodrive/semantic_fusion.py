#!/usr/bin/env python3
"""Semantic camera x LiDAR object fusion for the Alpamayo competition launch.

The camera says WHAT (semantic_sidecar.py masks: vehicle / person_animal / obstacle), the LiDAR says
WHERE. Each LiDAR point is projected into the front/left/right cameras, takes the class under it,
same-class points are clustered on a ground grid, tracked for velocity, and published as a
morai_msgs/ObjectStatusList on /perception/objects — the same message gui.py already reads from
/Object_topic, so the launch switches source with STUDENT_OBJECT_TOPIC alone.

Timing: masks carry the MORAI frame stamp of their camera frame. The LiDAR scan nearest to that stamp is
used and the remaining gap is removed with the ego speed/yaw-rate (points are moved to the camera instant).
Calibration: MORAI rgb_cam.json (pinhole, horizontal FOV, no distortion; vehicle frame x fwd, y left, z up).
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, os.environ.get("STUDENT_V3_DIR", str(HERE / "student_v3_claude")))   # the launch exports its copy
from lidar_corridor import cloud_xyz, fit_road_plane, outside_ego_footprint  # noqa: E402
from motion_filter import measured_velocity  # noqa: E402

VIEWS = ("front", "left", "right")
CLASS_KIND = {0: "npc", 1: "pedestrian", 2: "obstacle"}      # semantic class id -> ObjectStatusList list
KIND_TYPE = {"npc": 1, "pedestrian": 0, "obstacle": 2}
MIN_POINTS = {"npc": 6, "pedestrian": 3, "obstacle": 4}



import sys as _performance_sys
from pathlib import Path as _PerformancePath
_performance_sys.path.insert(0, str(_PerformancePath(__file__).resolve().parent.parent if _PerformancePath(__file__).resolve().parent.name == "student_v3" else _PerformancePath(__file__).resolve().parent))
from cpu_placement import pin_io_thread, pin_compute_thread

def load_cameras(sensor_json):
    """MORAI rig -> per-view calibration, classified by camera yaw rather than ID order."""
    data = json.loads(Path(sensor_json).read_text(encoding="utf-8"))
    cams = sorted(data["cameraList"], key=lambda c: int(c["m_SensorUniqueID"]))
    classified = {}
    for cam in cams:
        yaw = (float(cam["rot"]["yaw"]) + 180.0) % 360.0 - 180.0
        view = "left" if 45 < yaw < 135 else "right" if -135 < yaw < -45 else "front" if abs(yaw) <= 45 else None
        if view is not None:
            classified.setdefault(view, cam)
    if set(classified) != set(VIEWS):
        raise ValueError("sensor preset requires front, left and right cameras")
    out = {}
    for view in VIEWS:
        cam = classified[view]
        cc = cam["cc"]
        w, h = int(cc["cameraResWidth"]), int(cc["cameraResHeight"])
        fx = (w / 2.0) / math.tan(math.radians(float(cc["cameraFOV"])) / 2.0)   # MORAI FOV is horizontal
        yaw, pitch, roll = (math.radians(float(cam["rot"][k])) for k in ("yaw", "pitch", "roll"))
        cy_, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        rz = np.array([[cy_, -sy, 0], [sy, cy_, 0], [0, 0, 1]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])       # +pitch tilts the optical axis down
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        cam_to_vehicle = rz @ ry @ rx
        pos = np.array([float(cam["pos"][k]) for k in ("x", "y", "z")])
        out[view] = dict(pos=pos, R=cam_to_vehicle.T, fx=fx, cx=w / 2.0, cy=h / 2.0, w=w, h=h)
    lidar = data.get("Lidar3DList") or []
    mount = np.array([float(lidar[0]["pos"][k]) for k in ("x", "y", "z")]) if lidar else np.array([1.05, 0.0, 1.35])
    return out, mount


def split_vehicles(points, max_len=6.5, car_len=5.0, gap=0.8):
    """A cluster longer than one car is usually a queue / parked row: cut it along its long axis at gaps in
    the points, and if a piece is still too long, into ~car-length chunks."""
    xy = points[:, :2]
    if len(xy) < 2:
        return [points]
    centered = xy - xy.mean(axis=0)
    axis = np.linalg.svd(centered, full_matrices=False)[2][0]
    t = centered @ axis
    if t.max() - t.min() <= max_len:
        return [points]
    order = np.argsort(t)
    ts = t[order]
    cuts = np.nonzero(np.diff(ts) > gap)[0]
    pieces = np.split(order, cuts + 1)
    out = []
    for piece in pieces:
        tp = t[piece]
        length = tp.max() - tp.min()
        if length <= max_len:
            out.append(points[piece])
            continue
        n = int(math.ceil(length / car_len))
        edges = np.linspace(tp.min(), tp.max() + 1e-6, n + 1)
        for i in range(n):
            sub = piece[(tp >= edges[i]) & (tp < edges[i + 1])]
            if len(sub):
                out.append(points[sub])
    return out


class Tracker:
    """Nearest-neighbour tracks per kind for stable ids and velocity (world frame, sim time)."""
    def __init__(self):
        self.tracks, self.next_id = {}, 1

    def update(self, kind, detections, stamp):
        alive = {k: t for k, t in self.tracks.items() if t["kind"] == kind and 0 <= stamp - t["stamp"] < 1.0}
        used, result = set(), []
        for det in sorted(detections, key=lambda d: -d["n"]):
            best, best_d = None, 2.5 if kind == "npc" else 1.5
            for key, t in alive.items():
                if key in used:
                    continue
                pred = t["pos"] + t["vel"] * (stamp - t["stamp"])
                d = float(np.hypot(*(det["pos"] - pred)))
                if d < best_d:
                    best, best_d = key, d
            if best is None:
                best = self.next_id
                self.next_id += 1
            used.add(best)
            history = (alive[best]["history"] if best in alive else deque(maxlen=40))
            bounds_history = (alive[best]["bounds_history"] if best in alive else deque(maxlen=40))
            bounds = det.get("motion_bounds")
            motion_pos = np.mean(bounds, axis=0) if bounds is not None else det["pos"]
            if not history or stamp > history[-1][0]:
                history.append((stamp, float(motion_pos[0]), float(motion_pos[1])))
                if bounds is not None:
                    bounds_history.append((stamp, np.asarray(bounds).copy()))
            # Robust displacement slopes preserve slow crossings and suppress a
            # single changing cluster face. A moving car is confirmed after
            # 0.3 s rather than being published as parked for its first second.
            estimated = measured_velocity(list(history), 25.0 if kind == "npc" else 4.0)
            if estimated is not None and bounds is not None and kind != "pedestrian":
                # Box padding and a changing visible face are not object motion.
                # Translation must be supported by both observed world bounds.
                edges = [measured_velocity([(t, *box[i]) for t, box in bounds_history],
                                           25.0 if kind == "npc" else 4.0) for i in (0, 1)]
                speed = float(np.linalg.norm(estimated))
                if any(v is None for v in edges) or np.linalg.norm(np.asarray(edges[0]) - edges[1]) > max(.4, speed * .25):
                    estimated = (0.0, 0.0)
            vel = np.asarray(estimated if estimated is not None else (0.0, 0.0))
            moving = float(np.linalg.norm(vel)) > (0.5 if kind == "npc" else 0.15)
            self.tracks[best] = dict(kind=kind, pos=det["pos"], vel=vel, stamp=stamp,
                                     history=history, bounds_history=bounds_history)
            result.append(dict(det, uid=best, vel=vel, moving=moving))
        self.tracks = {k: t for k, t in self.tracks.items() if 0 <= stamp - t["stamp"] < 1.0}
        return result


class PedestrianPartMerger:
    """Join coherently moving semantic fragments while preserving both footprints.

    A walking person's visible body and camera-blind legs can be classified
    separately. Nearby stationary boxes and unconfirmed motion remain separate.
    """
    def __init__(self):
        self.pairs = {}
        self.last_stamp = None

    @staticmethod
    def corners(detection):
        velocity = np.asarray(detection["vel"], dtype=float)
        heading = math.atan2(velocity[1], velocity[0])
        c, s = math.cos(heading), math.sin(heading)
        rotation = np.array([[c, -s], [s, c]])
        half = np.asarray(detection["size"], dtype=float)[:2] / 2.
        local = np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]]) * half
        return local @ rotation.T + np.asarray(detection["pos"])

    def merge(self, detections, stamp):
        if not math.isfinite(stamp):
            self.pairs.clear()
            return list(detections)
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.pairs.clear()
        self.last_stamp = stamp
        pedestrians = [d for d in detections if d["kind"] == "pedestrian"]
        obstacles = [d for d in detections if d["kind"] == "obstacle"]
        valid_pairs, replacements, removed = {}, {}, set()
        for pedestrian in pedestrians:
            for obstacle in obstacles:
                if obstacle["uid"] in removed:
                    continue
                pv, ov = np.asarray(pedestrian["vel"]), np.asarray(obstacle["vel"])
                measured = np.r_[pv, ov, pedestrian["pos"], obstacle["pos"],
                                 pedestrian["size"][:2], obstacle["size"][:2],
                                 pedestrian["height"], obstacle["height"]]
                if not np.isfinite(measured).all():
                    continue
                # These are the measured positions/velocities already used by
                # the publisher. A single nearby detection cannot change class.
                if (np.linalg.norm(pv) <= .5 or np.linalg.norm(ov) <= .5 or
                        np.linalg.norm(pv-ov) > .35 or
                        np.linalg.norm(np.asarray(pedestrian["pos"])-obstacle["pos"]) > .4 or
                        np.max(obstacle["size"][:2]) > 1.7 or
                        pedestrian["height"] < .75 or obstacle["height"] > 2.2 or
                        abs(pedestrian["height"]-obstacle["height"]) > .7):
                    continue
                key = (pedestrian["uid"], obstacle["uid"])
                previous = self.pairs.get(key)
                if previous is None or stamp-previous["last"] > .25:
                    state = dict(since=stamp, last=stamp, count=1)
                else:
                    state = dict(previous)
                    if stamp > state["last"]:
                        state["count"] += 1
                        state["last"] = stamp
                valid_pairs[key] = state
                if state["count"] < 3 or stamp-state["since"] < .15:
                    continue
                merged = replacements.get(pedestrian["uid"], pedestrian)
                corners = np.vstack((self.corners(merged), self.corners(obstacle)))
                heading = math.atan2(pv[1], pv[0])
                c, s = math.cos(heading), math.sin(heading)
                rotation = np.array([[c, -s], [s, c]])
                relative = (corners-np.asarray(pedestrian["pos"])) @ rotation
                lo, hi = relative.min(axis=0), relative.max(axis=0)
                replacements[pedestrian["uid"]] = dict(
                    merged, pos=np.asarray(pedestrian["pos"])+(lo+hi)/2. @ rotation.T,
                    size=hi-lo, height=max(merged["height"], obstacle["height"]),
                    n=merged["n"]+obstacle["n"],
                    merged_part_ids=list(merged.get("merged_part_ids", []))+[obstacle["uid"]])
                removed.add(obstacle["uid"])
        self.pairs = valid_pairs
        return [replacements.get(d["uid"], d) for d in detections if d["uid"] not in removed]


def main():
    pin_compute_thread("fusion compute")
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sensor", type=Path, default=Path.home() / "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Sensor/25.S4.MolitComp03/rgb_cam.json")
    p.add_argument("--shm", type=Path, default=Path("/dev/shm/alpamayo_semantic"))
    p.add_argument("--lidar-topic", default=os.environ.get("STUDENT_LIDAR_TOPIC", "/lidar/points"))
    p.add_argument("--ego-topic", default="/localization/ego_status")
    p.add_argument("--out-topic", default="/perception/objects")
    p.add_argument("--hz", type=float, default=float(os.environ.get("SEMANTIC_FUSION_HZ", "15")),
                   help="maximum rate for fresh semantic mask sets")
    p.add_argument("--obj-min-height", type=float, default=0.15, help="points this far above the estimated road surface count")
    p.add_argument("--z-max", type=float, default=3.5)
    p.add_argument("--range", type=float, default=50.0)
    p.add_argument("--grid", type=float, default=0.4)
    p.add_argument("--blind-range", type=float, default=12.0, help="LiDAR-only obstacle zone ahead (m) where cameras cannot see")
    p.add_argument("--blind-half-width", type=float, default=2.5)
    p.add_argument("--route", type=Path, default=HERE / "student_v3_claude/global_path.json")
    p.add_argument("--barrier-min-len", type=float, default=3.0)
    p.add_argument("--barrier-span", type=float, default=6.0, help="along-route length of a same-offset obstacle line = barrier")
    p.add_argument("--barrier-min-points", type=int, default=15)
    p.add_argument("--barrier-max-width", type=float, default=1.0)
    p.add_argument("--route-half-width", type=float, default=float(os.environ.get("SEMANTIC_ROUTE_HALF_WIDTH", "4.5")),
                   help="publish only objects within this lateral distance of the global route (own + adjacent lane); "
                        "roadside structures outside the lanes are ignored")
    args = p.parse_args()
    if not math.isfinite(args.hz) or not 1 <= args.hz <= 60:
        p.error("--hz must be finite and within 1–60 Hz")

    import rospy
    from morai_msgs.msg import EgoVehicleStatus, ObjectStatus, ObjectStatusList
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import Vector3
    if not hasattr(threading.Thread, "isAlive"):
        threading.Thread.isAlive = threading.Thread.is_alive
    rospy.init_node("semantic_fusion", anonymous=False, disable_signals=True)

    cams, mount = load_cameras(args.sensor)
    route = np.array([[w["x"], w["y"]] for w in json.loads(args.route.read_text())["waypoints"]], np.float64)

    route_dir = np.arctan2(np.gradient(route[:, 1]), np.gradient(route[:, 0]))
    route_s = np.r_[0.0, np.cumsum(np.hypot(np.diff(route[:, 0]), np.diff(route[:, 1])))]
    from scipy.spatial import cKDTree
    route_tree = cKDTree(route)

    def barrier_points(world_xy):
        """Guardrails / median barriers: obstacle points that keep the same lateral offset from the route over a
        long stretch (>= --barrier-span m along it). Per point -> True if it belongs to such a line."""
        if len(world_xy) < args.barrier_min_points:
            return np.zeros(len(world_xy), bool)
        _, idx = route_tree.query(world_xy)
        rel = world_xy - route[idx]
        lat = -np.sin(route_dir[idx]) * rel[:, 0] + np.cos(route_dir[idx]) * rel[:, 1]
        along = route_s[idx]
        out = np.zeros(len(world_xy), bool)
        bins = np.round(lat / 0.3).astype(np.int64)
        for b in np.unique(bins):
            sel = np.abs(bins - b) <= 1                       # +-0.45 m band around this offset
            if sel.sum() >= args.barrier_min_points and np.ptp(along[sel]) >= args.barrier_span:
                out |= sel
        return out

    def near_route(xy):
        return float(np.min(np.hypot(route[:, 0] - xy[0], route[:, 1] - xy[1]))) <= args.route_half_width

    def route_heading(xy):
        return float(route_dir[int(np.argmin(np.hypot(route[:, 0] - xy[0], route[:, 1] - xy[1])))])

    def is_barrier(member, world):
        """Guardrail / median barrier / kerb wall: long, thin and running along the route."""
        xy = member[:, :2].astype(np.float64)
        if len(xy) < 4:
            return False
        centered = xy - xy.mean(axis=0)
        axes = np.linalg.svd(centered, full_matrices=False)[2]
        t, w = centered @ axes[0], centered @ axes[1]
        length, width = float(t.max() - t.min()), float(w.max() - w.min())
        if length < args.barrier_min_len or width > args.barrier_max_width:
            return False
        along = math.atan2(axes[0][1], axes[0][0]) + ego[3]          # vehicle frame -> world heading
        diff = abs((along - route_heading(world) + math.pi / 2) % math.pi - math.pi / 2)
        return diff < math.radians(25)
    clouds = deque(maxlen=12)        # (stamp_s, xyz vehicle frame)
    egos = deque(maxlen=200)         # (stamp_s, x, y, heading_rad, speed_mps, yaw_rate)
    lock = threading.Lock()

    def on_cloud(msg):
        try:
            xyz = cloud_xyz(msg) + mount.astype(np.float32)
            with lock:
                clouds.append((msg.header.stamp.to_sec(), xyz))
        except Exception as exc:
            rospy.logwarn_throttle(10, f"cloud: {exc!r}")

    def on_ego(msg):
        stamp = msg.header.stamp.to_sec() or rospy.get_time()
        speed = math.hypot(msg.velocity.x, msg.velocity.y)
        yaw_rate = float(getattr(msg.angular_velocity, "z", 0.0))
        with lock:
            egos.append((stamp, msg.position.x, msg.position.y, math.radians(msg.heading), speed, yaw_rate))

    rospy.Subscriber(args.lidar_topic, PointCloud2, on_cloud, queue_size=2, buff_size=2 ** 24)
    rospy.Subscriber(args.ego_topic, EgoVehicleStatus, on_ego, queue_size=20)
    pub = rospy.Publisher(args.out_topic, ObjectStatusList, queue_size=2)
    tracker = Tracker()
    pedestrian_parts = PedestrianPartMerger()
    debug_path = args.shm / "fusion.json"
    masks_path = args.shm / "masks.npz"
    last_seq, n_rate, t_rate, rate = None, 0, time.monotonic(), 0.0
    last_started = 0.0
    nx = int(round((args.range + 5.0) / args.grid))
    ny = int(round(2 * args.range / args.grid))

    last_masks_version = None
    while not rospy.is_shutdown():
        remaining = 1.0 / args.hz - (time.monotonic() - last_started)
        if remaining > 0:
            time.sleep(min(remaining, .005))
            continue
        try:
            info = masks_path.stat()
            version = (info.st_ino, info.st_mtime_ns)
            if time.time()-info.st_mtime >= 1.0 or version == last_masks_version:
                time.sleep(0.005)
                continue
            with np.load(masks_path, allow_pickle=False) as archive:
                masks = dict(archive)
            last_masks_version = version
        except (FileNotFoundError, ValueError, OSError, EOFError):
            masks = None
        if masks is None or int(masks["seq"]) == last_seq:
            time.sleep(0.005)
            continue
        last_started = time.monotonic()
        last_seq = int(masks["seq"])
        cam_stamp = int(masks["front_stamp_ns"]) * 1e-9
        with lock:
            cloud_list, ego_list = list(clouds), list(egos)
        if not cloud_list or not ego_list:
            continue
        cloud_stamp, pts = min(cloud_list, key=lambda c: abs(c[0] - cam_stamp))
        ego = min(ego_list, key=lambda e: abs(e[0] - cam_stamp))
        if abs(ego[0] - cam_stamp) > 1.0:      # localization stamped on another clock: use the newest pose
            ego = ego_list[-1]
        # Move the scan to the camera instant using ego motion (points are vehicle-frame).
        dt = cam_stamp - cloud_stamp
        if abs(dt) < 0.3:
            dyaw = ego[5] * dt
            c, s = math.cos(-dyaw), math.sin(-dyaw)
            shifted = pts[:, :2] - np.array([ego[4] * dt, 0.0], np.float32)
            pts = np.column_stack((c * shifted[:, 0] - s * shifted[:, 1], s * shifted[:, 0] + c * shifted[:, 1], pts[:, 2])).astype(np.float32)
        rng = np.hypot(pts[:, 0], pts[:, 1])
        # Ground height from the road around the car (the vehicle-frame origin is not at road level, and a fixed
        # z cut drops low objects such as 0.4 m cargo boxes). Objects = points above ground + --obj-min-height.
        road = pts[(rng > 3.0) & (rng < 15.0) & (np.abs(pts[:, 1]) < 4.0)]
        ground = float(np.percentile(road[:, 2], 20)) if len(road) > 30 else -0.25
        ground_plane = fit_road_plane(pts, ground)
        ground_heights = pts[:, :2] @ ground_plane[:2] + ground_plane[2]
        keep = (pts[:, 2] > ground_heights + args.obj_min_height) & (pts[:, 2] < ground_heights + args.z_max) & outside_ego_footprint(pts) & (rng < args.range)
        pts = pts[keep]
        labels = np.full(len(pts), 255, np.uint8)
        seen = np.zeros(len(pts), bool)
        for view in VIEWS:
            cam = cams[view]
            mask = masks[f"{view}_mask"]
            left, top, pw, ph, sw, sh = (int(v) for v in masks[f"{view}_place"])
            local = (pts - cam["pos"]) @ cam["R"].T                   # camera frame: x fwd, y left, z up
            front = local[:, 0] > 0.5
            u = cam["cx"] - cam["fx"] * local[:, 1] / np.where(front, local[:, 0], 1)
            v = cam["cy"] - cam["fx"] * local[:, 2] / np.where(front, local[:, 0], 1)
            # calibration pixels -> model-resolution letterboxed mask (stream res may differ from calibration res)
            mu = np.floor(left + u * pw / cam["w"]).astype(np.int64)
            mv = np.floor(top + v * ph / cam["h"]).astype(np.int64)
            inside = (front & (u >= 0) & (u < cam["w"]) & (v >= 0) & (v < cam["h"])
                      & (mu >= 0) & (mu < mask.shape[1]) & (mv >= 0) & (mv < mask.shape[0]))
            idx = np.nonzero(inside & ~seen)[0]
            got = mask[mv[idx], mu[idx]]
            visible = got != 254                        # 254 = under the ego body/hood in this camera
            labels[idx[visible]] = got[visible]
            seen[idx[visible]] = True
        out = ObjectStatusList()
        out.header.stamp = rospy.Time.from_sec(cam_stamp)
        out.header.frame_id = "map"
        c, s = math.cos(ego[3]), math.sin(ego[3])
        debug = []
        barriers = 0
        barrier_dropped_pts = 0
        low_vehicle = []
        tracked_detections = []
        # Camera blind zone (outside every image or under the ego body/hood): the LiDAR alone decides there.
        # Anything above the road in the driving corridor ahead is an obstacle, every frame (no memory).
        blind = (~seen) & (pts[:, 0] > 0.5) & (pts[:, 0] < args.blind_range) & (np.abs(pts[:, 1]) < args.blind_half_width)
        labels[blind] = 2
        for cls, kind in ((0, "npc"), (1, "pedestrian"), (2, "obstacle")):
            sel = pts[labels == cls]
            if kind == "obstacle" and low_vehicle:
                sel = np.vstack([sel] + low_vehicle)             # low "vehicle" clusters are obstacles
            if kind == "obstacle" and len(sel):
                wxy = np.column_stack((ego[1] + c * sel[:, 0] - s * sel[:, 1], ego[2] + s * sel[:, 0] + c * sel[:, 1]))
                line = barrier_points(wxy)
                barrier_dropped_pts = int(line.sum())
                sel = sel[~line]
            if len(sel) < MIN_POINTS[kind]:
                tracker.update(kind, [], cam_stamp)
                continue
            gx = np.clip(((sel[:, 0] + 5.0) / args.grid).astype(np.int32), 0, nx - 1)
            gy = np.clip(((sel[:, 1] + args.range) / args.grid).astype(np.int32), 0, ny - 1)
            occ = np.zeros((nx, ny), np.uint8)
            occ[gx, gy] = 1
            occ = cv2.dilate(occ, np.ones((3, 3), np.uint8))
            count, comp = cv2.connectedComponents(occ, connectivity=8)
            ids = comp[gx, gy]
            dets = []
            groups = []
            for k in range(1, count):
                member = sel[ids == k]
                groups.extend(split_vehicles(member) if kind == "npc" else [member])
            for member in groups:
                if len(member) < MIN_POINTS[kind]:
                    continue
                lo, hi = member.min(axis=0), member.max(axis=0)
                center = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2])
                extent = np.maximum(hi[:2] - lo[:2], 0.3)
                member_height = float(np.max(member[:, 2] - (member[:, :2] @ ground_plane[:2] + ground_plane[2])))
                if kind == "npc" and member_height < 1.0:
                    # lower than any car (e.g. a cargo box painted "vehicle"): publish it as a plain obstacle, unscaled
                    low_vehicle.append(member)
                    continue
                yaw_local = 0.0
                if kind == "npc":
                    # LiDAR sees only the near faces of a car: keep those where they are and grow the box away
                    # from the ego to a real car footprint (otherwise the bypass planner under-sizes it).
                    # Grow it along the ROAD, not along the ego axis: with the ego turned 16 deg during a
                    # bypass the ego-aligned 4.6 m box projected 3.1 m across the lane (0.95 -> 1.55 m
                    # half-width), the lane closed and the car sat blocked (auto run 437, user 2026-10-07).
                    world0 = np.array([ego[1] + c * center[0] - s * center[1], ego[2] + s * center[0] + c * center[1]])
                    yaw_local = route_heading(world0) - ego[3]
                    cd, sd = math.cos(yaw_local), math.sin(yaw_local)
                    rel = member[:, :2] - center
                    t, w = rel @ np.array([cd, sd]), rel @ np.array([-sd, cd])
                    extent = np.maximum(np.array([t.max() - t.min(), w.max() - w.min()]), 0.3)
                    center_r = np.array([center @ np.array([cd, sd]), center @ np.array([-sd, cd])])
                    full = np.array([4.6, 1.9])
                    grow = np.maximum(full - extent, 0.0)
                    center_r = center_r + np.sign(np.where(np.abs(center_r) < 1e-3, 1.0, center_r)) * grow / 2
                    center = np.array([center_r[0] * cd - center_r[1] * sd, center_r[0] * sd + center_r[1] * cd])
                    extent = np.maximum(extent, full)
                world = np.array([ego[1] + c * center[0] - s * center[1], ego[2] + s * center[0] + c * center[1]])
                if not near_route(world):
                    continue
                if kind == "obstacle" and is_barrier(member, world):
                    barriers += 1
                    continue
                if kind == "obstacle":
                    # A long barrier seen at an angle (curved tunnel mouth, run 49) fills a much wider
                    # vehicle-aligned box: 3.6 m concrete barrier -> 2.1 m sideways, leaving no legal
                    # bypass. Use the cluster's own principal axes when that box is clearly tighter.
                    xy = member[:, :2].astype(np.float64)
                    if len(xy) >= 6:
                        centred = xy - xy.mean(axis=0)
                        axes = np.linalg.svd(centred, full_matrices=False)[2]
                        t, w = centred @ axes[0], centred @ axes[1]
                        oriented = np.array([t.max() - t.min(), w.max() - w.min()])
                        if np.prod(np.maximum(oriented, 0.3)) < 0.75 * np.prod(extent):
                            center = xy.mean(axis=0) + axes[0] * (t.max() + t.min()) / 2 + axes[1] * (w.max() + w.min()) / 2
                            extent = np.maximum(oriented, 0.3)
                            yaw_local = math.atan2(axes[0][1], axes[0][0])
                            world = np.array([ego[1] + c * center[0] - s * center[1], ego[2] + s * center[0] + c * center[1]])
                    # LiDAR often sees only the top/near face of a low box: never smaller than 0.8 m, plus 0.3 m margin
                    extent = np.maximum(extent, 0.8) + 0.3
                observed_world = np.column_stack((ego[1] + c * member[:, 0] - s * member[:, 1],
                                                  ego[2] + s * member[:, 0] + c * member[:, 1]))
                motion_bounds = np.stack((observed_world.min(axis=0), observed_world.max(axis=0)))
                dets.append(dict(pos=world, local=center, size=extent, height=member_height,
                                 n=len(member), motion_bounds=motion_bounds, yaw=ego[3] + yaw_local))
            tracked_detections.extend(dict(d, kind=kind) for d in tracker.update(kind, dets, cam_stamp))
        for d in pedestrian_parts.merge(tracked_detections, cam_stamp):
            kind = d["kind"]
            o = ObjectStatus()
            o.unique_id, o.type, o.name = int(d["uid"]), KIND_TYPE[kind], f"fusion_{kind}"
            speed = float(np.hypot(*d["vel"]))
            o.heading = (math.degrees(math.atan2(d["vel"][1], d["vel"][0])) if speed > 0.5
                         else math.degrees(d.get("yaw", ego[3])))
            o.velocity = Vector3(speed * 3.6, 0.0, 0.0)            # MORAI convention: km/h along heading
            o.acceleration = Vector3(0.0, 0.0, 0.0)
            o.size = Vector3(float(d["size"][0]), float(d["size"][1]), d["height"])
            o.position = Vector3(float(d["pos"][0]), float(d["pos"][1]), 0.0)
            # MORAI lists parked cars as obstacles; the planner bypasses obstacles but queues behind npcs.
            listed = "obstacle" if kind == "npc" and not d["moving"] else kind
            if listed != kind:
                o.type, o.name = KIND_TYPE["obstacle"], "fusion_parked_vehicle"
            getattr(out, {"npc": "npc_list", "pedestrian": "pedestrian_list", "obstacle": "obstacle_list"}[listed]).append(o)
            debug.append(dict(kind=kind, id=o.unique_id, x=round(o.position.x, 2), y=round(o.position.y, 2), size=[round(float(v), 2) for v in d["size"]], height=round(d["height"], 2),
                              local=[round(float(v), 2) for v in d["local"]], n=d["n"], speed_kmh=round(speed * 3.6, 1),
                              merged_part_ids=d.get("merged_part_ids", [])))
        out.num_of_npcs, out.num_of_pedestrian, out.num_of_obstacle = len(out.npc_list), len(out.pedestrian_list), len(out.obstacle_list)
        pub.publish(out)
        n_rate += 1
        now = time.monotonic()
        if now - t_rate >= 1.0:
            rate, n_rate, t_rate = n_rate / (now - t_rate), 0, now
            tmp = debug_path.with_name(".fusion.json.tmp")
            tmp.write_text(json.dumps(dict(time=time.time(), hz=round(rate, 1), target_hz=args.hz, cam_lidar_dt_ms=round(dt * 1000, 1),
                                           labelled_points=int((labels != 255).sum()), points=int(len(pts)), barriers_dropped=barriers, barrier_points_dropped=barrier_dropped_pts, ground_z=round(ground, 2), objects=debug)))
            os.replace(tmp, debug_path)


if __name__ == "__main__":
    main()
