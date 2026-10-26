"""LiDAR tunnel planning in the same vehicle frame used by the MPC follower."""
import json
import copy
import math
import os
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
from json_safe import json_default
from scipy.spatial import cKDTree
from driving_plan import DrivingPlan
from sim_time import runtime_rtf, to_sim_objects
from ego_body import (EGO_FRONT_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M, REAR_AXLE_SHIFT_M,
                      ROAD_LEGACY_SHIFT_M, body_centres, self_return_mask)

# Preserve the flat module's resource anchor without changing planning methods.
__file__ = str(Path(__file__).resolve().parents[2] / "tunnel_runtime.py")


# Beyond this a frame is too old to steer by; below it the late travel is
# subtracted from the checked clearance.
TUNNEL_PLANNING_ABORT_S = .6   # (no longer stops: late frames are charged against clearance)
TUNNEL_FALLBACK_KMH = float(os.environ.get("STUDENT_TUNNEL_FALLBACK_KMH", "40"))
TUNNEL_TIGHT_PASS_KMH = float(os.environ.get("STUDENT_TUNNEL_TIGHT_PASS_KMH", "5"))
TUNNEL_PEEK_KMH = float(os.environ.get("STUDENT_TUNNEL_PEEK_KMH", "3"))
CRAWL_KMH = float(os.environ.get("STUDENT_CRAWL_KMH", "5"))   # below the lattice low-speed threshold (1.5 m/s)
TUNNEL_BYPASS_ALAT_MPS2 = float(os.environ.get("STUDENT_TUNNEL_BYPASS_ALAT", "3.5"))   # bypass curve limit (was 1.7)
TUNNEL_FALLBACK_MAX_OFFSET_M = 4.0   # fallback keeps the current lane: 1.5 pulled a bypassing car back into a parked one (auto run 77)
FALLBACK_STALL_S = 6.   # crawling < 3 km/h this long with an obstacle within 25 m: reverse and re-plan
# Run 222: at 1.2-7 Hz decisions the car covers 1-4 m between looks. Above this
# interval the speed must leave room for the travel until the next decision plus
# the stop (TUNNEL_BRAKE_MPS2) inside the checked clear distance.
# Seconds of travel (at least 15 m) within which the side of each obstacle we pass is kept (lattice
# side_commit_s; runs 222/225/271 crossed between S-row drums and hit the next one).
TUNNEL_SIDE_COMMIT_S = float(os.environ.get("STUDENT_TUNNEL_SIDE_COMMIT_S", "2.0"))
TUNNEL_SLOW_DECISION_S = float(os.environ.get("STUDENT_TUNNEL_SLOW_DECISION_S", ".3"))
FALLBACK_EXIT_REJOIN_M = 30.   # fallback path back on the route this far before the tunnel exit (auto run 167)
TUNNEL_LATTICE_MAX_OFFSET_M = 5.0   # wall-to-wall candidates (walls checked); 3.5 left a 6 m gap unused (auto run 118)
# Braking assumed for the clear-distance speed caps. 2.5 m/s^2 held the car at
# 20-30 km/h behind the ~25-33 m the VLP16 sees of the tunnel floor (live
# 2026-10-03); the highway uses 4.
TUNNEL_BRAKE_MPS2 = float(os.environ.get("STUDENT_TUNNEL_BRAKE_MPS2", "4.0"))
# Hold the bypass offset until this far before the tunnel exit. Off by
# default (1e9 = always rejoin): it stalled near the exit in the 40 km/h
# virtual run; set STUDENT_TUNNEL_HOLD_UNTIL_EXIT_M=30 to enable.
TUNNEL_REJOIN_BEFORE_EXIT_M = float(os.environ.get('STUDENT_TUNNEL_HOLD_UNTIL_EXIT_M', '1e9'))
TUNNEL_HOLD_MAX_M = 2.0   # keep both walls in view for the wall match
# Last stretch before the exit (auto runs 185/187/188): the open road beyond the
# exit and the portal returns shorten the observed LiDAR coverage, and returns
# just past the exit at the right wall line (the road bends right there) made the
# lattice offset the path with nothing on the road. In this zone the coverage
# only lowers the speed (never below the cap for this distance, never a stop),
# and an offset is kept only when the centre path itself is obstructed.
TUNNEL_EXIT_ZONE_M = float(os.environ.get("STUDENT_TUNNEL_EXIT_ZONE_M", "30"))
TUNNEL_EXIT_OBSERVED_FLOOR_M = float(os.environ.get("STUDENT_TUNNEL_EXIT_OBSERVED_FLOOR_M", "10"))
# Route joins in that zone keep the body this far (instead of 0.25 m) inside the
# kerb-corrected walls: a 26 m join from a 12 deg heading error swung 1.1 m past
# the route toward the right kerb 4 m before the exit (run 188).
TUNNEL_EXIT_WALL_MARGIN_M = float(os.environ.get("STUDENT_TUNNEL_EXIT_WALL_MARGIN_M", "1.0"))
TUNNEL_JOIN_TRANSITIONS = (26., 22., 18., 14., 12., 10., 8., 6.)
# Exit-zone return to the centre: at least the comfortable join for the offset and
# speed (lattice RETURN_ALAT_MPS2) and no steeper than this to the route. Longer than
# the distance left, it simply continues past the exit (the tunnel is two lanes wide).
TUNNEL_EXIT_RETURN_MAX_DEG = float(os.environ.get("STUDENT_TUNNEL_EXIT_RETURN_MAX_DEG", "12"))
TUNNEL_EXIT_RETURN_MAX_M = 55.   # build_forced_bypass plans 60 m
# Lattice candidates across the wall-to-wall width (wider, smoother bypasses:
# 0.91 m vs 0.28 m clearance, no dip below 29 km/h in the virtual tunnel).
# Raw LiDAR points only, body-width trigger, walls are hard limits; passes the
# recorded-scan tunnel tests. STUDENT_TUNNEL_LATTICE=0 restores the old planner.
TUNNEL_LATTICE = os.environ.get("STUDENT_TUNNEL_LATTICE", "1") == "1"
# Claude working copy, web E2E mode (model_avoidance.py): inside the tunnel the v6
# model's checked path, the portable static planner with its route scope (rejoin
# 45 m before the exit) and lane hold. Reuses the non-lattice static and model slots.
TUNNEL_POLICY = os.environ.get("STUDENT_TUNNEL_POLICY", "lattice")
TUNNEL_MODEL = TUNNEL_POLICY == "model"
if TUNNEL_MODEL:
    TUNNEL_LATTICE = False
# The walls end at the portal, so the wall match fails in the last metres by design
# (E2E run follow-20261009T185440Z: 199 of 381 tunnel frames, then 30 s of degraded
# crawl at the exit). Within this distance of the exit a failed match keeps driving on
# the last verified match carried by wheel/IMU odometry; it never aids localization.
TUNNEL_EXIT_ANCHOR = TUNNEL_MODEL or os.environ.get("STUDENT_TUNNEL_EXIT_ANCHOR", "0") == "1"
TUNNEL_EXIT_ANCHOR_M = float(os.environ.get("STUDENT_TUNNEL_EXIT_ANCHOR_M", "40"))
TUNNEL_EXIT_ANCHOR_S = float(os.environ.get("STUDENT_TUNNEL_EXIT_ANCHOR_S", "20"))
# The wall map was 0.544 deg / 0.177 m off the world (built 2026-10-02 from ESKF poses
# before imu_yaw_variance_override; realigned 2026-10-10, see tunnel_wall_map.json), so
# the measured pose-to-map correction grew ~1 m per 100 m driven (0.18 -> 1.24 m over
# 97 m, flat while stopped) although the ESKF matched withheld raw GPS. Planning already
# runs in the measured wall frame; with this flag it keeps doing so however large any
# pose-to-map disagreement grows: the wall-fit window follows the last measured lateral,
# validated walls plan without the 1.2 m relative cap, relative matches also anchor the
# exit carry, anchors are checked net of the correction measured with them, and a stop
# does not age the exit anchor. Map aiding (0.8 m) and every clearance check unchanged.
TUNNEL_RELATIVE_FRAME = TUNNEL_MODEL or os.environ.get("STUDENT_TUNNEL_RELATIVE_FRAME", "0") == "1"
TUNNEL_WALL_SEARCH_M = (1.25, -1.25, 2.5, -2.5)   # extra window shifts after the prior and raw windows
# Model policy: re-shape the portable planner's bypass for curvature speed and keep its
# offset and commitment (model_avoidance.smooth_tunnel_plan). "0" restores its own shape.
TUNNEL_SHAPE = os.environ.get("STUDENT_TUNNEL_SHAPE", "1") != "0"
TUNNEL_COVERAGE_ESCAPE_M = 12.   # observed floor this short on clear walls/returns: change the line
LATTICE_NEAR_SIDE_MARGIN_M = .1


def rotation(ego):
    yaw = math.radians(ego["heading_deg"])
    return np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])


def pose_at_stamp(samples, stamp, max_extrapolation=.12):
    """Interpolate poses; bound forward prediction and internal-gap retrodiction."""
    # ESKF may publish a corrected state at the same source timestamp.
    samples = list({p["stamp"]: p for p in samples if p.get("stamp", 0.) > 0.}.values())
    if not samples or not math.isfinite(stamp) or stamp <= 0.:
        return None
    samples = sorted(samples, key=lambda p: p["stamp"])
    times = np.array([p["stamp"] for p in samples])
    index = int(np.searchsorted(times, stamp))
    if index < len(samples) and abs(times[index] - stamp) < 1e-6:
        return dict(samples[index])
    if index == len(samples):
        result = dict(samples[-1])
        dt = stamp - result["stamp"]
        if dt > max_extrapolation:
            return None
        delta = rotation(result)[:, 0] * result.get("speed_kmh", 0.) / 3.6 * dt
        result.update(x=result["x"] + delta[0], y=result["y"] + delta[1], stamp=stamp)
        return result
    if index == 0:
        return dict(samples[0]) if abs(times[0] - stamp) < .015 else None
    a, b = samples[index - 1], samples[index]
    dt = b["stamp"] - a["stamp"]
    if dt <= 0.:
        return None
    if dt > .2:
        # A later sample exposes a publication gap; the same short prediction
        # allowed at the end of the queue remains valid near its leading edge.
        predicted = pose_at_stamp([a], stamp, max_extrapolation=max_extrapolation)
        if predicted is not None:
            return predicted
        # Both bounding poses have already arrived. A recent following sample
        # also supports a short retrodiction into this internal publication gap.
        # This does not authorize prediction before the receiver's first sample.
        reverse_dt = stamp-b["stamp"]
        if -max_extrapolation <= reverse_dt <= 0.:
            result = dict(b)
            delta = rotation(b)[:, 0] * b.get("speed_kmh", 0.) / 3.6 * reverse_dt
            result.update(x=b["x"]+delta[0], y=b["y"]+delta[1], stamp=stamp)
            return result
        return None
    u = (stamp - a["stamp"]) / dt
    result = dict(a)
    for key in ("x", "y", "speed_kmh"):
        result[key] = a.get(key, 0.) + u * (b.get(key, 0.) - a.get(key, 0.))
    delta = (b["heading_deg"] - a["heading_deg"] + 180.) % 360. - 180.
    result.update(heading_deg=a["heading_deg"] + u * delta, stamp=stamp)
    return result


class TunnelOdometryReceiver:
    """Uncorrected wheel/IMU motion avoids feeding ESKF map corrections back twice."""
    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        self.lock = threading.Lock()
        self.samples = deque(maxlen=800)
        self.subscriber = rospy.Subscriber("/odometry/pure", Odometry, self.on_odometry, queue_size=100)

    def on_odometry(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y*q.y + q.z*q.z))
        v = message.twist.twist.linear
        sample = dict(stamp=message.header.stamp.to_sec(), x=p.x, y=p.y,
                      heading_deg=math.degrees(yaw), speed_kmh=math.hypot(v.x, v.y)*3.6,
                      received=time.monotonic())
        with self.lock:
            if self.samples and sample["stamp"] < self.samples[-1]["stamp"]:
                self.samples.clear()
            self.samples.append(sample)

    def at_stamp(self, stamp):
        return self.at_stamps((stamp,))[0]

    def at_stamps(self, stamps):
        """Resolve one set of source times against one fresh odometry history."""
        with self.lock:
            samples = list(self.samples)
        age = time.monotonic() - samples[-1]["received"] if samples else math.inf
        if not 0. <= age <= .25:
            return [None for _ in stamps]
        return [pose_at_stamp(samples, stamp) for stamp in stamps]

    def close(self):
        self.subscriber.unregister()


def propagate_match(match, scan_ego, current_ego):
    """Carry a scan's wall correction forward using the intervening odometry."""
    alignment = rotation(match) @ rotation(scan_ego).T
    delta = np.array([current_ego["x"] - scan_ego["x"],
                      current_ego["y"] - scan_ego["y"]])
    xy = np.array([match["x"], match["y"]]) + alignment @ delta
    return dict(current_ego, x=float(xy[0]), y=float(xy[1]),
                heading_deg=current_ego["heading_deg"] + match["heading_deg"] - scan_ego["heading_deg"])


def anchor_disagreement(anchor, predicted, scan_ego):
    """Distance (m) and yaw (deg) between an odometry-carried wall pose and the ESKF scan
    pose, net of the wall correction measured with the anchor when it recorded its ESKF
    pose (relative frame); otherwise the raw difference, as before."""
    offset, yaw_offset = np.zeros(2), 0.
    source = anchor.get("scan_ego")
    if source is not None:
        aligned = anchor["match"]["aligned_ego"]
        offset = np.array([aligned["x"]-source["x"], aligned["y"]-source["y"]], dtype=float)
        yaw_offset = float(aligned["heading_deg"])-float(source["heading_deg"])
    error = np.array([predicted["x"]-scan_ego["x"], predicted["y"]-scan_ego["y"]], dtype=float)-offset
    yaw = abs((predicted["heading_deg"]-scan_ego["heading_deg"]-yaw_offset+180.) % 360.-180.)
    return float(math.hypot(error[0], error[1])), float(yaw)


def points_at_pose(points, old_ego, ego):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    world = points @ rotation(old_ego).T + [old_ego["x"], old_ego["y"]]
    return (world - [ego["x"], ego["y"]]) @ rotation(ego)


def corridor_at_pose(corridor, old_ego, ego):
    """Carry the road and every no-go polygon into the path's current frame."""
    if corridor is None:
        return None
    from lane_corridor import LaneCorridor
    result = LaneCorridor(points_at_pose(corridor.route, old_ego, ego),
        corridor.left, corridor.right,
        zones=[points_at_pose(zone, old_ego, ego) for zone in corridor.zones],
        intersection_mask=corridor.intersection_mask,
        body_shift_m=getattr(corridor, 'body_shift_m', ROAD_LEGACY_SHIFT_M))
    result.wall_only = bool(getattr(corridor, 'wall_only', False))
    result.measured_walls = bool(getattr(corridor, 'measured_walls', False))
    return result


def objects_at_pose(objects, old_ego, ego):
    """Transform measured boxes and velocity into an independent odometry world."""
    transform = rotation(ego) @ rotation(old_ego).T
    yaw_delta = ego["heading_deg"]-old_ego["heading_deg"]
    result = []
    for obj in objects:
        values = list(obj)
        point = np.asarray(values[:2])-[old_ego["x"], old_ego["y"]]
        values[:2] = (transform @ point+[ego["x"], ego["y"]]).tolist()
        values[8] += yaw_delta
        values[9:11] = (transform @ np.asarray(values[9:11])).tolist()
        if len(values) > 12 and isinstance(values[12], dict) and "motion_center" in values[12]:
            values[12] = dict(values[12])
            centre = np.asarray(values[12]["motion_center"])-[old_ego["x"], old_ego["y"]]
            values[12]["motion_center"] = (transform @ centre+[ego["x"], ego["y"]]).tolist()
            if "observer" in values[12]:
                x, y, heading = values[12]["observer"]
                observer = transform @ (np.array([x, y])-[old_ego["x"], old_ego["y"]])+[ego["x"], ego["y"]]
                values[12]["observer"] = [float(observer[0]), float(observer[1]), float(heading)+yaw_delta]
        result.append(tuple(values))
    return result


def lidar_objects(points, ego, radius=.65, *, bounds_padding_m=None):
    """Connected measured returns become conservative boxes, without a class model.

    Every return remains in the independent swept-body collision check, including
    singletons. Boxes only supply the existing static planner's bypass triggers.
    """
    if bounds_padding_m is not None and (not math.isfinite(bounds_padding_m) or bounds_padding_m < 0.):
        raise ValueError('LiDAR bounds padding must be finite and nonnegative')
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    points = points[(points[:, 0] > -5.) & (points[:, 0] < 70.) & (abs(points[:, 1]) < 14.)]
    if not len(points):
        return []
    points = np.unique(np.round(points / .2), axis=0) * .2
    tree, seen, objects = cKDTree(points), set(), []
    for i in range(len(points)):
        if i in seen:
            continue
        pending, group = [i], []
        seen.add(i)
        while pending:
            j = pending.pop()
            group.append(j)
            for neighbor in tree.query_ball_point(points[j], radius):
                if neighbor not in seen:
                    seen.add(neighbor)
                    pending.append(neighbor)
        p = points[group]
        angle = 0.
        if len(p) >= 4:
            import cv2
            # A box aligned to the ego can invent occupied corners on a
            # slanted vehicle face. Fit an enclosing measured orientation.
            # Normalize the equivalent axes to keep longitudinal/width stable.
            _, _, degrees = cv2.minAreaRect(p.astype(np.float32))
            angle = math.radians((float(degrees)+45.) % 90.-45.)
        axes = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
        projected = p @ axes
        # Default bounds cover each 20 cm cell at the fitted orientation.
        # Road callers budget explicit bounds padding with their SAT margin.
        margin = (.1*(abs(math.cos(angle))+abs(math.sin(angle)))
                  if bounds_padding_m is None else float(bounds_padding_m))
        lo, hi = projected.min(axis=0) - margin, projected.max(axis=0) + margin
        centre = (lo + hi) / 2.
        world = rotation(ego) @ (axes @ centre) + [ego["x"], ego["y"]]
        # Orientation changes can move an enclosing rectangle's centre without
        # moving any surface. Track the original axis bounds independently.
        motion_world = rotation(ego) @ (.5*(p.min(axis=0)+p.max(axis=0))) + [ego["x"], ego["y"]]
        length, width = np.maximum(hi - lo, .3)
        # TunnelTracks credits travel-axis motion only when both edges of these
        # bounds move, and line-of-sight motion by the observer's range.
        objects.append((float(world[0]), float(world[1]), float(max(length, width)),
                        "tunnel_lidar_%d" % i, "obstacle", 0., float(length),
                        float(width), ego["heading_deg"]+math.degrees(angle), 0., 0., i,
                        {"point_count": len(p), "motion_center": motion_world.tolist(),
                         "motion_extent": (p.max(axis=0)-p.min(axis=0)).tolist(),
                         "observer": [float(ego["x"]), float(ego["y"]), float(ego["heading_deg"])]}))
    return objects


# A standing detector (PointPillars) box is refitted to whatever part of the car the LiDAR
# sees, and it can be placed in front of that car's own returns. follow-20261010T200235Z
# 05:09:44-05:12:30 (random layout global_hard_13, 20 m before the exit): a car standing
# ~65 deg across the left lane, 4.9 x 2.1 m box. From 5-30 m away its box sat on its returns;
# alongside it, the box was refitted 1.0-1.6 m toward us, its corner inside our own body
# (gap 0.00-0.15 m), while the car's own returns (40-80 in its cluster, the end and side
# faces) stayed 0.9-1.6 m from our side (body_gap_m 1.45-1.65 m; no contact in 2 min). The
# box outline zeroed the obstacle clearance of every path from the car's pose: "no safe
# bypass", a reverse, a bypass found 3 m back, the same stop 1-2 m later, for 2 min 45 s.
# The box is moved (never turned or shrunk) back onto the faces the car shows the sensor.
# Its low parts near our body are below the lowest ring (1.75 m above the road, -15 deg),
# but the tunnel's near-field memory keeps what was measured of them on the way in.
DETECTOR_FACE_TOLERANCE_M = .2    # a face may stand this far in front of the car's returns
DETECTOR_ASSOCIATION_M = 1.2      # returns this close to the box can be the car's; the largest move
DETECTOR_MIN_RETURNS = 8
DETECTOR_FACE_MIN_M = 1.5         # a cluster this long is the car, not a drum in front of it
DETECTOR_RAY_DEG = 2.             # a nearer return this close in bearing could hide part of the car
DETECTOR_FACE_VIEW_M = .5         # the sensor at most this far inside a face's plane still sees it
DETECTOR_ANCHOR_RANGE_M = 30.     # boxes refit near the car; far views keep the detector's
LIDAR_ORIGIN_XY = (1.05, 0.)      # VLP16 mount in the vehicle frame (lidar_mount)


def _box_frame(points, centre, heading_rad):
    axis = np.array([math.cos(heading_rad), math.sin(heading_rad)])
    relative = np.asarray(points, dtype=float).reshape(-1, 2) - centre
    return relative @ axis, relative @ np.array([-axis[1], axis[0]])


def _inside_box(points, centre, heading_rad, length, width, margin=0.):
    u, v = _box_frame(points, centre, heading_rad)
    return (np.abs(u) <= .5 * length + margin) & (np.abs(v) <= .5 * width + margin)


def _box_corners(centre, heading_rad, length, width):
    axis = np.array([math.cos(heading_rad), math.sin(heading_rad)])
    cross = np.array([-axis[1], axis[0]])
    return np.array([centre + p * .5 * length * axis + q * .5 * width * cross
                     for p, q in ((1, 1), (1, -1), (-1, -1), (-1, 1))])


def _boxes_overlap(a, b):
    """Separating-axis test of two (centre, heading_rad, length, width) rectangles."""
    corners = [_box_corners(*a), _box_corners(*b)]
    for box in corners:
        for k in range(2):
            edge = box[k + 1] - box[k]
            normal = np.array([-edge[1], edge[0]])
            pa, pb = corners[0] @ normal, corners[1] @ normal
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def anchor_detector_box(car, points, ego, sensor_xy=LIDAR_ORIGIN_XY, tolerance_m=DETECTOR_FACE_TOLERANCE_M,
                        association_m=DETECTOR_ASSOCIATION_M, min_returns=DETECTOR_MIN_RETURNS, debug=None):
    """(car, info): a standing detector car box moved back onto its car's measured faces.

    car: world tuple (x, y, size, name, kind, speed, length, width, heading, ...); points:
    measured returns (with the near-field memory) in ego's vehicle frame. The returns near
    the box are clustered as in lidar_objects and the largest cluster overlapping it, at
    least DETECTOR_FACE_MIN_M long, is taken as the car. Along each box axis whose face the
    sensor looks at (or grazes, DETECTOR_FACE_VIEW_M), that face may stand at most
    tolerance_m in front of the car's nearest return; a box standing further forward is
    moved back (by at most association_m), keeping its size and heading - the unseen
    remainder stays behind the measured faces.

    The detector box is kept when the strip it would give up holds a return (2 or more), or
    a nearer return on a ray into it could hide part of the car, or without such a cluster.
    info: None (unchanged), {"kept": why} or the move. Every return stays in the raw-point
    checks either way. debug (dict): filled with the strip given up and the move."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2) if points is not None else np.empty((0, 2))
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(car) < 9 or len(pts) < min_returns:
        return car, None
    length, width = max(.3, float(car[6])), max(.3, float(car[7]))
    yaw = math.radians(float(ego["heading_deg"]))
    turn = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    origin = np.array([float(ego["x"]), float(ego["y"])])
    centre = (np.array([float(car[0]), float(car[1])]) - origin) @ turn
    heading = math.radians(float(car[8])) - yaw
    if not (np.isfinite(centre).all() and math.isfinite(heading)):
        return car, None
    associated = _inside_box(pts, centre, heading, length, width, association_m)
    near = pts[associated]
    if len(near) < min_returns:
        return car, None
    best = None
    for obj in lidar_objects(near, ego):
        count = int((obj[12] or {}).get("point_count", 0))
        c = (np.array(obj[:2], dtype=float) - origin) @ turn
        h = math.radians(float(obj[8])) - yaw
        if (count >= min_returns and max(float(obj[6]), float(obj[7])) >= DETECTOR_FACE_MIN_M
                and _boxes_overlap((c, h, obj[6], obj[7]), (centre, heading, length, width))
                and (best is None or count > best[0])):
            best = (count, obj, c, h)
    if best is None:
        return car, None
    count, cluster, m_centre, m_heading = best
    own_mask = associated & _inside_box(pts, m_centre, m_heading, float(cluster[6]), float(cluster[7]), .15)
    own = pts[own_mask]
    if len(own) < min_returns:
        return car, None
    own_u, own_v = _box_frame(own, centre, heading)
    sensor = np.asarray(sensor_xy, dtype=float)
    seen_u, seen_v = _box_frame(sensor, centre, heading)
    shift = []
    for coordinate, half, seen in ((own_u, .5 * length, float(seen_u[0])), (own_v, .5 * width, float(seen_v[0]))):
        move = 0.
        # Only a face the sensor looks at (or grazes): seen side-on, an axis keeps its fit.
        if seen < -(half - DETECTOR_FACE_VIEW_M):
            move = max(0., float(coordinate.min()) - tolerance_m + half)
        elif seen > half - DETECTOR_FACE_VIEW_M:
            move = -max(0., half - float(coordinate.max()) - tolerance_m)
        shift.append(float(np.clip(move, -association_m, association_m)))
    if not any(shift):
        return car, None
    axis = np.array([math.cos(heading), math.sin(heading)])
    new_centre = centre + shift[0] * axis + shift[1] * np.array([-axis[1], axis[0]])
    # The strip given up, sampled every ~0.15 m.
    steps_u = np.linspace(-.5, .5, max(2, int(math.ceil(length / .15)) + 1))
    steps_v = np.linspace(-.5, .5, max(2, int(math.ceil(width / .15)) + 1))
    samples = (centre + steps_u[:, None, None] * length * axis
               + steps_v[None, :, None] * width * np.array([-axis[1], axis[0]])).reshape(-1, 2)
    given_up = samples[~_inside_box(samples, new_centre, heading, length, width, .05)]
    if debug is not None:
        debug.update(given_up=given_up, own=own, shift=shift, new=(new_centre, heading, length, width))
    stray = (_inside_box(pts, centre, heading, length, width)
             & ~_inside_box(pts, new_centre, heading, length, width, .1))
    if int(stray.sum()) >= 2:
        return car, dict(kept="returns in the strip given up", returns=int(stray.sum()))
    if len(given_up):
        ray_p = given_up - sensor
        reach = float(np.linalg.norm(ray_p, axis=1).max())
        others = pts[~own_mask & ~_inside_box(pts, new_centre, heading, length, width, .3)]
        others = others[np.linalg.norm(others - sensor, axis=1) < reach]
        if len(others):
            ray_q = others - sensor
            gap = np.abs((np.arctan2(ray_p[:, 1], ray_p[:, 0])[:, None]
                          - np.arctan2(ray_q[:, 1], ray_q[:, 0])[None, :] + math.pi) % (2. * math.pi) - math.pi)
            if np.any((gap <= math.radians(DETECTOR_RAY_DEG))
                      & (np.linalg.norm(ray_q, axis=1)[None, :] < np.linalg.norm(ray_p, axis=1)[:, None])):
                return car, dict(kept="a nearer return could hide part of the car")
    world = new_centre @ turn.T + origin
    values = list(car)
    values[0], values[1] = float(world[0]), float(world[1])
    return tuple(values), dict(moved_m=round(float(np.hypot(*shift)), 2), cluster_returns=count)


def standing_cars_on_returns(cars, points, ego):
    """(cars, log): anchor_detector_box for every standing detector car within
    DETECTOR_ANCHOR_RANGE_M (moving ones keep the detector box their motion belongs to);
    log (decision record tunnel.detector_anchor) lists the boxes it moved or kept."""
    if points is None or not len(points):
        return list(cars), None
    out, log = [], []
    for car in cars:
        if (len(car) > 14 and car[14] and float(car[5]) > .5) or math.hypot(
                float(car[0]) - float(ego["x"]), float(car[1]) - float(ego["y"])) > DETECTOR_ANCHOR_RANGE_M:
            out.append(car)
            continue
        anchored, info = anchor_detector_box(car, points, ego)
        out.append(anchored)
        if info is not None:
            log.append(dict(info, id=car[11] if len(car) > 11 else None))
    return out, log or None


def measured_wall_corridor(route, walls, source_ego, current_ego):
    """Preserve fresh asymmetric LiDAR wall bounds without correcting map pose."""
    from lane_corridor import LaneCorridor
    if not walls or not walls.get('validated'):
        return None
    route = np.asarray(route, dtype=float)[:, :2]
    if len(route) < 3:
        return None
    lines = []
    for side in ('left', 'right'):
        slope, intercept = float(walls[side]['slope']), float(walls[side]['intercept_m'])
        if not math.isfinite(slope) or not math.isfinite(intercept):
            return None
        line = np.array([[-80., intercept-80.*slope], [160., intercept+160.*slope]])
        world = line @ rotation(source_ego).T + [source_ego['x'], source_ego['y']]
        line = (world-[current_ego['x'], current_ego['y']]) @ rotation(current_ego)
        lines.append(line)
    # Use the measured wall axis as the corridor reference. Projecting walls
    # against the curved road after the portal discarded an otherwise valid fit.
    centre = .5*(lines[0]+lines[1])
    corridor_route = np.linspace(centre[0], centre[1], 481)
    tangent = centre[1]-centre[0]
    tangent /= np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]])
    bounds = []
    for line in lines:
        along = line[1]-line[0]
        axis = np.array([-along[1], along[0]])/np.linalg.norm(along)
        denominator = normal @ axis
        if abs(denominator) < .5:
            return None
        bounds.append((line[0] @ axis-corridor_route @ axis)/denominator)
    left, right = np.maximum(*bounds), np.minimum(*bounds)
    if not np.isfinite(left-right).all() or np.any((left-right < 6.) | (left-right > 18.)):
        return None
    return LaneCorridor(corridor_route, left, right, body_shift_m=REAR_AXLE_SHIFT_M)


def corridor_points(points, corridor):
    """Limit bypass triggers to the mapped road; raw points still check every path."""
    if corridor is None or not len(points):
        return np.empty((0, 2))
    points = np.asarray(points)
    _, index = corridor.tree.query(points)
    lateral = np.sum((points - corridor.route[index]) * corridor.normal[index], axis=1)
    keep = ((lateral <= corridor.left[index] + .25) &
            (lateral >= corridor.right[index] - .25))
    return points[keep]


def kerb_room(tunnel, scan_aligned):
    """ObservedLidarCoverage lateral_room: distance from path stations (scan vehicle frame)
    to the tunnel's kerb faces, inside the mapped section only; None without a map pose."""
    names = ("origin", "axis", "normal", "left", "right", "start", "end")
    if not scan_aligned or not all(hasattr(tunnel, name) for name in names):
        return None
    turn = rotation(scan_aligned)
    ledge = getattr(tunnel, "ledge", None) or {1: 0., -1: 0.}
    face_left = float(tunnel.left)-max(0., float(ledge.get(1, 0.)))
    face_right = float(tunnel.right)+max(0., float(ledge.get(-1, 0.)))

    def room(centres, normals):
        world = np.asarray(centres, dtype=float) @ turn.T+[scan_aligned["x"], scan_aligned["y"]]
        relative = world-tunnel.origin
        along, lateral = relative @ tunnel.axis, relative @ tunnel.normal
        side = (np.asarray(normals, dtype=float) @ turn.T) @ tunnel.normal   # + : our left is the tunnel's
        cos = np.maximum(np.abs(side), .5)
        to_left, to_right = (face_left-lateral)/cos, (lateral-face_right)/cos
        inside = (along >= tunnel.start) & (along <= tunnel.end)
        left = np.where(inside, np.where(side >= 0., to_left, to_right), np.inf)
        right = np.where(inside, np.where(side >= 0., to_right, to_left), np.inf)
        return left, right
    return room


def latency_speed_cap(distance_m, lag_s, brake_mps2):
    """Highest speed with lag_s of travel plus a brake_mps2 stop inside distance_m-2."""
    if lag_s <= 0. or not math.isfinite(distance_m):
        return math.inf
    room = max(0., float(distance_m) - 2.)
    a = float(brake_mps2)
    return max(0., -a * lag_s + math.sqrt((a * lag_s) ** 2 + 2. * a * room))


def coverage_search_distance(speed_mps, decision_lag_s=0.):
    """Start finding a visible line before coverage reaches the braking envelope.

    Invert the existing clearance speed cap, with one source-refresh interval
    of travel in reserve. This changes candidate search, never the stop checks.
    """
    speed = max(0., float(speed_mps))
    reaction = max(1.75 / TUNNEL_BRAKE_MPS2, float(decision_lag_s) + .25)
    return max(TUNNEL_COVERAGE_ESCAPE_M,
               2. + speed * reaction + speed * speed / (2. * TUNNEL_BRAKE_MPS2))


# Bypass curve speed. The cap was sqrt(alat / the peak curvature of the whole ~60 m
# path), so the return S of a bypass still 20-40 m ahead (0.053-0.099 1/m) held
# 20-27 km/h from the frame the bypass was planned (follow-20261010T151125Z lap 1
# 00:16:35.0-35.3 and 00:16:42.2-46.3, lap 2 00:21:52.7-56.6; 3.0 m/s^2 of
# STUDENT_BYPASS_ALAT through static_speed_cap_mps). Each curvature now limits the
# speed at its own station, reached by braking at TUNNEL_CURVE_DECEL_MPS2 after a
# reaction: min over s of sqrt(alat/k(s) + 2 a (s - v t)). The curve at the car
# itself (s ~ 0) keeps the plain sqrt(alat/k) limit.
TUNNEL_CURVE_DECEL_MPS2 = float(os.environ.get("STUDENT_TUNNEL_CURVE_DECEL_MPS2", "2.0"))
TUNNEL_CURVE_REACTION_S = .4


def curve_speed_cap(path, speed_mps=0., alat_mps2=None, decel_mps2=None,
                    reaction_s=TUNNEL_CURVE_REACTION_S):
    """(cap m/s, station m of the binding curvature) along a vehicle-frame path."""
    from gui import path_curvatures
    alat = TUNNEL_BYPASS_ALAT_MPS2 if alat_mps2 is None else float(alat_mps2)
    decel = TUNNEL_CURVE_DECEL_MPS2 if decel_mps2 is None else float(decel_mps2)
    path = np.asarray(path, dtype=float)[:, :2]
    if len(path) < 3:
        return math.inf, 0.
    curvature = path_curvatures(path)
    if not np.isfinite(curvature).all():
        return 0., 0.
    station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))][1:-1]
    room = np.maximum(0., station - max(0., float(speed_mps)) * float(reaction_s))
    allowed = alat / np.maximum(curvature, 1e-4) + 2. * decel * room
    index = int(np.argmin(allowed))
    return math.sqrt(float(allowed[index])), float(station[index])


def tunnel_route_alignment(route):
    """Measured rear-axle distance and heading to the original local route."""
    route = np.asarray(route, dtype=float)
    if route.ndim != 2 or route.shape[1] < 2 or len(route) < 2 or not np.isfinite(route).all():
        return dict(offset_m=None, heading_error_deg=None, complete=False)
    route = route[:, :2]
    delta = np.diff(route, axis=0)
    length2 = np.sum(delta*delta, axis=1)
    fraction = np.clip(-np.sum(route[:-1]*delta, axis=1)/np.maximum(length2, 1e-12), 0., 1.)
    nearest = route[:-1]+fraction[:, None]*delta
    distance = np.linalg.norm(nearest, axis=1)
    distance[length2 < 1e-8] = math.inf
    index = int(np.argmin(distance))
    heading = abs(math.degrees(math.atan2(delta[index, 1], delta[index, 0])))
    offset = float(distance[index])
    return dict(offset_m=offset, heading_error_deg=heading,
                complete=bool(offset <= .25 and heading <= 3.))


def tunnel_exit_completion(path, dense, ego, tunnel):
    """First checked path station after which route position and heading stay aligned."""
    path, dense = np.asarray(path, dtype=float)[:, :2], np.asarray(dense, dtype=float)[:, :2]
    if len(path) < 3 or len(dense) < 3 or not np.isfinite(path).all() or not np.isfinite(dense).all():
        return None
    segments = np.diff(dense, axis=0)
    length2 = np.sum(segments*segments, axis=1)
    delta = path[:, None, :]-dense[None, :-1, :]
    fractions = np.clip(np.sum(delta*segments[None, :, :], axis=2)/
                        np.maximum(length2, 1e-12)[None, :], 0., 1.)
    separation = delta-fractions[:, :, None]*segments[None, :, :]
    distance2 = np.sum(separation*separation, axis=2)
    distance2[:, length2 < 1e-8] = math.inf
    nearest = np.argmin(distance2, axis=1)
    tangent = np.gradient(path, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    route_tangent = segments[nearest]/np.maximum(np.sqrt(length2[nearest, None]), 1e-9)
    angle = np.arccos(np.clip(np.sum(tangent*route_tangent, axis=1), -1., 1.))
    aligned = ((distance2[np.arange(len(path)), nearest] <= .25**2) &
               (angle <= math.radians(3.)))
    bad = np.flatnonzero(~aligned)
    index = int(bad[-1]+1) if len(bad) else 0
    station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    if index >= len(path) or station[-1]-station[index] < 2.:
        return None
    world = path[index]@rotation(ego).T+[ego['x'], ego['y']]
    remaining = float(tunnel.end-(world-tunnel.origin)@tunnel.axis)
    return dict(station_m=float(station[index]), to_exit_m=remaining, world_xy=world.tolist())


def tunnel_exit_rejoin(dense, ego, tunnel, objects, hazards, clearance, active_path=None,
                       *, wall_clearance=None, centre_limit=math.inf,
                       previous_saved=None, now=None, budget_s=.04, clock=time.perf_counter):
    """Prefer a fully checked return before the portal, then the earliest safe return.

    The return is a tunnel-only zero-offset candidate. Raw returns, lane bounds,
    physical walls and static SAT all check its canonical publication vertices.
    A prospective return after the last route obstacle supplies a curvature-only
    preparation speed; it never authorizes that unexecuted path.
    """
    from gui import (build_forced_bypass, MAX_CONTROL_CURVATURE, RosPathPublisher,
                     static_clear_distance, path_curvatures, continue_static_plan)
    from ego_body import EGO_REAR_M
    started = clock()
    dense = np.asarray(dense, dtype=float)
    info = dict(selected=False, reused=False, preparation_cap_kmh=None, candidates_checked=0,
                rejected=dict(curvature=0, geometry=0, dynamics=0), budget_exhausted=False)
    if dense.ndim != 2 or dense.shape[1] < 2 or len(dense) < 3 or not np.isfinite(dense).all():
        return None, dict(info, reason='original tunnel route unavailable')
    dense = dense[:, :2]
    pose = [float(ego.get(key, math.nan)) for key in ('x', 'y', 'heading_deg', 'speed_kmh')]
    if not np.isfinite(pose).all():
        return None, dict(info, reason='exit return pose unavailable')
    to_exit = float(tunnel.end-(np.array([ego['x'], ego['y']])-tunnel.origin)@tunnel.axis)
    info.update(to_exit_m=to_exit, measured=tunnel_route_alignment(dense))
    if info['measured']['complete']:
        return None, dict(info, reason='already aligned with original lane')
    if not 0. <= to_exit <= TUNNEL_EXIT_ZONE_M:
        return None, dict(info, reason='outside tunnel exit zone')
    speed = abs(float(ego.get('speed_kmh', 0.)))/3.6
    route_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]

    stationary = [o for o in objects if o[4] == 'obstacle' or
                  (o[4] == 'npc' and float(o[5]) < .5) or
                  (o[4] == 'pedestrian' and float(o[5]) < .3)]

    def checked_return(candidate, peak):
        horizon = min(60., float(np.linalg.norm(np.diff(candidate, axis=0), axis=1).sum()))
        required = max(35., speed*.7+speed*speed/5.+2.)
        if horizon < required:
            return None
        components = dict(clearance(candidate, horizon))
        components['static_sat'] = static_clear_distance(candidate, ego, stationary, horizon,
                                                         body_shift_m=REAR_AXLE_SHIFT_M)
        exit_room = float(wall_clearance(candidate, horizon)) if wall_clearance is not None else horizon
        info['candidates_checked'] += 1
        if (not components or not all(math.isfinite(float(v)) and float(v) >= horizon-.3
                                     for v in components.values()) or
                not math.isfinite(exit_room) or exit_room < min(horizon, centre_limit)-.3):
            return None
        return horizon, components, exit_room

    # Keep a checked exit return's world geometry once it meets the deadline.
    # Starting a fresh six-metre heading join every frame moves its completion
    # beyond the portal. Shorter continuation joins preserve that fixed target
    # only when the same full geometry, steering and speed checks still pass.
    saved_plan = (previous_saved or {}).get('plan') or {}
    if (saved_plan.get('exit_rejoin') and saved_plan.get('exit_rejoin_goal_xy') is not None
            and now is not None):
        info['committed_attempted'] = True
        for join_m in (6., 4., 3., 2., 1.):
            if clock()-started > budget_s:
                info['budget_exhausted'] = True
                break
            continued = continue_static_plan(previous_saved, ego, now, join_m=join_m)
            if continued is None:
                break
            candidate = np.asarray(continued['path'])[:, :2]
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(candidate, axis=0), axis=1))]
            if arc[-1] < 50.:
                # Keep the return vertices and replenish only its already
                # aligned route tail. Otherwise trimming a saved sixty-metre
                # path eventually fails the unchanged full-horizon check.
                tail = int(np.argmin(np.linalg.norm(dense-candidate[-1], axis=1)))
                if np.linalg.norm(dense[tail]-candidate[-1]) <= .25:
                    candidate = np.vstack((candidate, dense[tail+1:]))
                    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(candidate, axis=0), axis=1))]
                    candidate = candidate[:min(len(candidate), int(np.searchsorted(arc, 60.))+1)]
            candidate = RosPathPublisher.resample(candidate)
            peak = float(np.max(path_curvatures(candidate)))
            if not math.isfinite(peak) or peak > MAX_CONTROL_CURVATURE:
                info['rejected']['curvature'] += 1
                continue
            curve_cap, _ = curve_speed_cap(candidate, speed)
            if speed > curve_cap+.5:
                info['rejected']['dynamics'] += 1
                continue
            completion = tunnel_exit_completion(candidate, dense, ego, tunnel)
            if completion is None or completion['to_exit_m'] < 1.95:
                continue
            checked = checked_return(candidate, peak)
            if checked is None:
                info['rejected']['geometry'] += 1
                continue
            horizon, components, exit_room = checked
            info.update(selected=True, reused=True, reason='kept checked return before portal',
                transition_m=float(continued.get('transition_m', join_m)), pose_join_m=join_m,
                completion_to_exit_m=completion['to_exit_m'], completes_before_exit=True,
                curve_cap_kmh=curve_cap*3.6, checked_clearance_m=components,
                exit_wall_clearance_m=exit_room, completion_station_m=completion['station_m'],
                committed_goal_xy=list(saved_plan['exit_rejoin_goal_xy']))
            return dict(continued, path=candidate, curvature=peak, speed_cap_mps=curve_cap,
                safe_horizon_m=horizon, clear_distance_m=min(components.values()), reused=True), info
        info['committed_invalidated'] = True

    # Reserve the last two metres at the portal. Obstacles still in the route
    # predict when the rear body can clear them; their measured geometry stays
    # in every actual path check below. This slows the valid bypass in advance
    # instead of discovering a tight return at 35 km/h with only 11 m left.
    backs = [float(h.get('back', 2.*float(h.get('along', 0.))-float(h['front'])))
             for h in hazards if float(h['front']) < to_exit-2. and
             float(h.get('back', 2.*float(h.get('along', 0.))-float(h['front']))) > -EGO_REAR_M]
    backs = [back for back in backs if math.isfinite(back)]
    future_s = max([0.]+backs)+(EGO_REAR_M+.25 if backs else 0.)
    future_path = np.asarray(active_path if active_path is not None else dense, dtype=float)
    if (future_path.ndim == 2 and future_path.shape[1] >= 2 and len(future_path) > 2
            and np.isfinite(future_path).all()):
        future_path = future_path[:, :2]
        path_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(future_path, axis=0), axis=1))]
        if future_s < path_s[-1]-25.:
            point = np.array([np.interp(future_s, path_s, future_path[:, k]) for k in (0, 1)])
            tangent = future_path[min(int(np.searchsorted(path_s, future_s))+1, len(future_path)-1)]-point
            if np.linalg.norm(tangent) > .01:
                future = dict(ego, heading_deg=float(ego['heading_deg'])+
                              math.degrees(math.atan2(tangent[1], tangent[0])))
                future['x'], future['y'] = point@rotation(ego).T+[ego['x'], ego['y']]
                world = dense@rotation(ego).T+[ego['x'], ego['y']]
                nearest = int(np.argmin(np.linalg.norm(world-[future['x'], future['y']], axis=1)))
                future_dense = (world[nearest:]-[future['x'], future['y']])@rotation(future)
                remaining = float(tunnel.end-(np.array([future['x'], future['y']])-tunnel.origin)@tunnel.axis)
                target = min(TUNNEL_EXIT_RETURN_MAX_M, remaining-2.)
                prep_transitions = sorted({max(6., target)} |
                    {t for t in TUNNEL_JOIN_TRANSITIONS if t >= target})
                for transition in prep_transitions:
                    if clock()-started > budget_s:
                        info['budget_exhausted'] = True
                        break
                    trial, peak, _ = build_forced_bypass(future_dense, 0., offset_m=0., hold_m=0.,
                        max_curvature_1pm=MAX_CONTROL_CURVATURE, transitions=(transition,))
                    if trial is not None:
                        trial = RosPathPublisher.resample(trial)
                        peak = float(np.max(path_curvatures(trial)))
                    if trial is not None and math.isfinite(peak) and 1e-5 < peak <= MAX_CONTROL_CURVATURE:
                        prep = math.sqrt(TUNNEL_BYPASS_ALAT_MPS2/peak)
                        info.update(preparation_cap_kmh=prep*3.6, preparation_ahead_m=future_s,
                            preparation_to_exit_m=remaining, preparation_transition_m=transition,
                            preparation_curvature_1pm=peak, preparation_steerable=True,
                            preparation_deadline_reachable=bool(transition <= target))
                        break
    transitions = set(TUNNEL_JOIN_TRANSITIONS)|{TUNNEL_EXIT_RETURN_MAX_M}
    available = min(TUNNEL_EXIT_RETURN_MAX_M, to_exit-2.)
    if available >= 6.:
        transitions.add(round(available, 2))
    candidates = []
    for transition in sorted(transitions):
        if clock()-started > budget_s:
            info['budget_exhausted'] = True
            break
        candidate, peak, _ = build_forced_bypass(dense, 0., offset_m=0., hold_m=0.,
            max_curvature_1pm=MAX_CONTROL_CURVATURE, transitions=(transition,))
        if candidate is None or not math.isfinite(peak) or peak > MAX_CONTROL_CURVATURE:
            info['rejected']['curvature'] += 1
            continue
        candidate = RosPathPublisher.resample(candidate)
        peak = float(np.max(path_curvatures(candidate)))
        if not math.isfinite(peak) or peak > MAX_CONTROL_CURVATURE:
            info['rejected']['curvature'] += 1
            continue
        curve_cap, _ = curve_speed_cap(candidate, speed)
        if speed > curve_cap+.5:
            info['rejected']['dynamics'] += 1
            continue
        completion = np.array([np.interp(transition, route_s, dense[:, k]) for k in (0, 1)])
        completion_remaining = float(tunnel.end-((completion@rotation(ego).T+
            [ego['x'], ego['y']])-tunnel.origin)@tunnel.axis)
        before = completion_remaining >= 2.-.05
        # Before the deadline keep the gentlest candidate. If the deadline is
        # unreachable, finish the safe return as early as the current speed allows.
        rank = (not before, -transition if before else transition, peak)
        candidates.append((rank, candidate, peak, transition, completion_remaining, curve_cap))
    for _, candidate, peak, transition, completion_remaining, curve_cap in sorted(candidates, key=lambda c:c[0]):
        if clock()-started > budget_s:
            info['budget_exhausted'] = True
            break
        checked = checked_return(candidate, peak)
        if checked is None:
            info['rejected']['geometry'] += 1
            continue
        horizon, components, exit_room = checked
        info.update(selected=True, reason='return before portal' if completion_remaining >= 1.95 else
                    'earliest reachable safe return; portal deadline unreachable',
                    transition_m=transition, completion_to_exit_m=completion_remaining,
                    completes_before_exit=bool(completion_remaining >= 1.95),
                    curve_cap_kmh=curve_cap*3.6, checked_clearance_m=components,
                    exit_wall_clearance_m=exit_room)
        if info['completes_before_exit']:
            info['preparation_cap_kmh'] = None   # this return's own checked curve now owns its speed
        goal_local = np.array([np.interp(transition, route_s, dense[:, k]) for k in (0, 1)])
        return dict(path=candidate, kind='rejoin', generator='portable', direction=1., offset_m=0.,
                    hold_m=0., hold_until_m=0., transition_m=transition, curvature=peak,
                    safe_horizon_m=horizon, clear_distance_m=min(components.values()),
                    speed_cap_mps=curve_cap, exit_rejoin=True, reused=False,
                    exit_rejoin_goal_xy=(goal_local@rotation(ego).T+[ego['x'], ego['y']]).tolist()
                        if info['completes_before_exit'] else None), info
    return None, dict(info, reason='no reachable fully checked exit return')


# Moving objects in the tunnel (user 2026-10-11: decelerate for a mover unless it is
# predicted into the path within the stopping distance). A moving_conflict "stop"
# whose conflict point can still be stopped short of at TUNNEL_YIELD_BRAKE_MPS2 after
# TUNNEL_YIELD_REACTION_S (moving_conflict's own stopping terms) is approached on that
# braking envelope, re-checked every frame, instead of a full stop; one the current
# speed cannot stop short of keeps the emergency stop.
TUNNEL_YIELD_BRAKE_MPS2 = 2.5
TUNNEL_YIELD_REACTION_S = .7
TUNNEL_YIELD_STANDOFF_M = EGO_FRONT_M + 2.5   # front bumper (rear axle + 3.845 m), then 2.5 m short of the conflict
TUNNEL_YIELD_TOLERANCE_MPS = .5          # follower lag behind a falling envelope


def yield_envelope_mps(along_m):
    """Highest speed that still stops TUNNEL_YIELD_STANDOFF_M short of a conflict at along_m."""
    a, t = TUNNEL_YIELD_BRAKE_MPS2, TUNNEL_YIELD_REACTION_S
    room = float(along_m) - TUNNEL_YIELD_STANDOFF_M
    if not math.isfinite(room) or room <= 0.:
        return 0.
    return -a * t + math.sqrt((a * t) ** 2 + 2. * a * room)


def tunnel_conflict_objects(moving):
    """Tunnel movers as moving_conflict reads them. A car-length LiDAR cluster
    (tunnel_tracks.vehicle_body, confirmed only above VEHICLE_BODY_MOVING_MPS) is a
    vehicle heading along its measured velocity: followed in our lane, ignored in the
    other lane, timed when crossing - not a pedestrian disc as wide as its length
    (a 4.3 m box made a 3.5 m band each side of the path)."""
    from tunnel_tracks import vehicle_body
    rows = []
    for obj in moving:
        row = list(obj)
        if (str(row[3]).startswith('tunnel_lidar_dynamic') and row[4] == 'pedestrian' and len(row) > 10
                and vehicle_body((float(row[6]), float(row[7])))):
            if math.hypot(float(row[9]), float(row[10])) > .3:
                row[8] = math.degrees(math.atan2(float(row[10]), float(row[9])))
            row[4] = 'npc'
        rows.append(tuple(row))
    return rows


def tunnel_parallel_path(route, max_offset=TUNNEL_FALLBACK_MAX_OFFSET_M, change_m=15., to_exit_m=None):
    """Path from the car that settles parallel to the route (vehicle frame).

    to_exit_m: distance to the tunnel exit. The road outside is narrower than the
    12 m tunnel: holding a 4 m bypass offset through the exit left the car 4.7 m off
    the route against the exit kerb, and three reverses replayed the same approach
    (auto run 167). Back to the route centre FALLBACK_EXIT_REJOIN_M before the exit."""
    from lattice_planner import offset_profile, _arc, _frame
    route = np.asarray(route, dtype=float)[:, :2]
    if len(route) < 3:
        return route
    arc = _arc(route)
    tangent, normal = _frame(route)
    d0 = -float(route[0] @ normal[0])
    slope0 = float(np.clip(math.tan(-math.atan2(tangent[0, 1], tangent[0, 0])), -.35, .35))
    target = float(np.clip(d0, -max_offset, max_offset))
    if to_exit_m is not None and abs(target) > .3:
        hold = max(0., float(to_exit_m) - FALLBACK_EXIT_REJOIN_M)
        d = offset_profile(arc, d0, target, change_m, hold_until=max(hold, 1.), return_m=FALLBACK_EXIT_REJOIN_M * .8,
                           slope0=slope0)
    else:
        d = offset_profile(arc, d0, target, change_m, slope0=slope0)
    return route + normal * d[:, None]


def partition_lattice_points(points, body_shift_m=ROAD_LEGACY_SHIFT_M):
    """Separate returns already beside the body using the planner's vehicle frame
    (front bumper EGO_HALF_LENGTH_M + body_shift_m ahead of the pose; ego_body)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    beside = ((points[:, 0] < EGO_HALF_LENGTH_M + float(body_shift_m) + .5) &
              (np.abs(points[:, 1]) > EGO_HALF_WIDTH_M + LATTICE_NEAR_SIDE_MARGIN_M))
    return points[beside], points[~beside]


def lattice_swept_clearance(path, points, horizon, side_margin, end_margin, body_shift_m=ROAD_LEGACY_SHIFT_M):
    """Recheck a lattice path with the same margins and beside-body partition."""
    if points is None:
        return 0.
    near, ahead = partition_lattice_points(points, body_shift_m)
    clear = swept_clearance(path, ahead, horizon, end_margin, lateral_margin=side_margin,
                            body_shift_m=body_shift_m)
    if len(near):
        clear = min(clear, swept_clearance(path, near, horizon, end_margin,
                                          lateral_margin=LATTICE_NEAR_SIDE_MARGIN_M,
                                          body_shift_m=body_shift_m))
    return clear


def swept_clearance(path, points, horizon, extra_margin=.25, tree=None, lateral_margin=None,
                    body_shift_m=ROAD_LEGACY_SHIFT_M):
    """Check front, sides and rear at 25 cm intervals, including curved paths.

    body_shift_m: box centre ahead of each path sample along its heading. The pose
    is the rear axle, so the tunnel passes REAR_AXLE_SHIFT_M (the real body: its
    front corners swing out on curves as the real front does); the road/highway
    default keeps the legacy box centred on the sample (ego_body).
    Returns the travel (m) to the first sample whose body (+margins) holds a point."""
    lateral_margin = extra_margin if lateral_margin is None else lateral_margin
    path = np.asarray(path, dtype=float)[:, :2]
    if len(path) < 3 or points is None:
        return 0.
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    sample = np.arange(0., min(horizon, arc[-1]) + .001, .25)
    if len(sample) < 3:
        return 0.
    centres = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
    tangent = np.gradient(centres, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    points = np.asarray(points).reshape(-1, 2)
    if not len(points):
        return float(sample[-1])
    tree = tree if tree is not None else cKDTree(points)
    centres = body_centres(centres, tangent, body_shift_m)
    radius = math.hypot(EGO_HALF_LENGTH_M + extra_margin, EGO_HALF_WIDTH_M + lateral_margin)
    # Vectorised pre-check: most stations have no point within the footprint
    # radius. Per-station Python lists for every 25 cm made this 60 % of the
    # tunnel decision time (300-480 ms per frame, 2.4 s when stalled; run 126).
    nearest, _ = tree.query(centres, k=1, distance_upper_bound=radius + 1e-9)
    occupied = np.flatnonzero(np.isfinite(nearest))
    if not len(occupied):
        return min(float(horizon), float(arc[-1]))
    # Query only the small neighborhood of each footprint, not an M x N matrix.
    nearby = tree.query_ball_point(centres[occupied], radius, return_sorted=False)
    counts = np.fromiter((len(ids) for ids in nearby), dtype=int, count=len(nearby))
    if counts.sum():
        station = np.repeat(occupied, counts)
        ids = np.concatenate([ids for ids in nearby if len(ids)])
        delta = points[ids] - centres[station]
        along = np.sum(delta*tangent[station], axis=1)
        lateral = np.sum(delta*normal[station], axis=1)
        hit = ((abs(along) <= EGO_HALF_LENGTH_M + extra_margin) &
               (abs(lateral) <= EGO_HALF_WIDTH_M + lateral_margin))
        first = np.flatnonzero(hit)
        if len(first):
            return float(sample[station[first[0]]])
    return min(float(horizon), float(arc[-1]))



def swept_clearances(path, points, horizon, extra_margin, lateral_margins, tree=None,
                     body_shift_m=ROAD_LEGACY_SHIFT_M):
    """swept_clearance for several lateral margins with one neighbour query (the
    lattice asked 3x per candidate with the same path and points)."""
    path = np.asarray(path, dtype=float)[:, :2]
    if len(path) < 3 or points is None:
        return [0.] * len(lateral_margins)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    sample = np.arange(0., min(horizon, arc[-1]) + .001, .25)
    if len(sample) < 3:
        return [0.] * len(lateral_margins)
    points = np.asarray(points).reshape(-1, 2)
    if not len(points):
        return [float(sample[-1])] * len(lateral_margins)
    centres = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
    tangent = np.gradient(centres, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    centres = body_centres(centres, tangent, body_shift_m)   # box centres (swept_clearance)
    tree = tree if tree is not None else cKDTree(points)
    end = min(float(horizon), float(arc[-1]))
    radius = math.hypot(EGO_HALF_LENGTH_M + extra_margin, EGO_HALF_WIDTH_M + max(lateral_margins))
    nearest, _ = tree.query(centres, k=1, distance_upper_bound=radius + 1e-9)
    occupied = np.flatnonzero(np.isfinite(nearest))
    if not len(occupied):
        return [end] * len(lateral_margins)
    # Fixed-k neighbour arrays (no per-station Python lists); stations whose k-th
    # neighbour is still inside the radius may hold more points: ball query there.
    k = min(24, len(points))
    dist, idx = tree.query(centres[occupied], k=k, distance_upper_bound=radius + 1e-9)
    dist, idx = dist.reshape(len(occupied), k), idx.reshape(len(occupied), k)
    valid = np.isfinite(dist)
    station = np.repeat(occupied, valid.sum(axis=1))
    ids = idx[valid]
    full = occupied[valid[:, -1]] if k < len(points) else occupied[:0]
    if len(full):
        extra = tree.query_ball_point(centres[full], radius, return_sorted=False)
        counts = np.fromiter((len(i) for i in extra), dtype=int, count=len(extra))
        station = np.r_[station, np.repeat(full, counts)]
        ids = np.r_[ids, np.concatenate([np.asarray(i, dtype=int) for i in extra])]
        order = np.argsort(station, kind="stable")
        station, ids = station[order], ids[order]
    if not len(ids):
        return [end] * len(lateral_margins)
    delta = points[ids] - centres[station]
    along = np.abs(np.sum(delta*tangent[station], axis=1)) <= EGO_HALF_LENGTH_M + extra_margin
    lateral = np.abs(np.sum(delta*normal[station], axis=1))
    out = []
    for margin in lateral_margins:
        # Same as swept_clearance: only points inside its smaller radius count.
        r = math.hypot(EGO_HALF_LENGTH_M + extra_margin, EGO_HALF_WIDTH_M + margin)
        inside = np.hypot(delta[:, 0], delta[:, 1]) <= r
        first = np.flatnonzero(along & (lateral <= EGO_HALF_WIDTH_M + margin) & inside)
        out.append(float(sample[station[first[0]]]) if len(first) else end)
    return out

class TunnelRuntime:
    def tunnel_path_checks(self, path, aligned, points, corridor, horizon=60., validation=None, tree=None):
        """Use one final geometry and its footprint margins for every check."""
        validation = validation or {}
        end = float(validation.get('end_margin_m', .25))
        side = float(validation.get('side_margin_m', .25))
        obstacle = (lattice_swept_clearance(path, points, horizon, side, end, REAR_AXLE_SHIFT_M)
                    if validation.get('generator') == 'lattice' else
                    swept_clearance(path, points, horizon, end, tree=tree, lateral_margin=side,
                                    body_shift_m=REAR_AXLE_SHIFT_M))
        if corridor is None:
            return dict(obstacle=obstacle)
        wall = (self.tunnel.path_clearance(path, aligned, horizon)
                if validation.get('mapped_walls', True) else corridor.clear_distance(path, horizon))
        lane = (wall if getattr(corridor, 'wall_only', False) and not corridor.zones
                else corridor.clear_distance(path, horizon))
        return dict(wall=wall, lane=lane, obstacle=obstacle)

    def tunnel_traffic_checks(self, decision, objects, points, moving=()):
        """Lead and motion decisions consume the same immutable checked path."""
        from gui import moving_conflict
        if not decision.validation.get('valid') or not decision.validation.get('pose_valid'):
            self._lead_state = None
            return decision, None, None
        lead = (self.lead_follow_step(decision.validation_ego, decision.checked_at,
            objects, points, decision.path, True, False, lidar_cars_in_objects=True,
            decision_plan=decision) if hasattr(self, 'lead_follow_step') else None)
        cap, emergency = decision.allowed_mps, decision.emergency
        if lead and lead.get('cap_mps') is not None:
            cap = min(cap, lead['cap_mps'])
            emergency = emergency or bool(lead.get('emergency'))
        # TunnelTracks clusters and detector cars carry measured (wall-clock)
        # velocities; the timed conflict check runs against simulator ego speed.
        speed = abs(float(decision.control_ego.get('speed_kmh', 0.)))/3.6
        yielding = moving_conflict(to_sim_objects(tunnel_conflict_objects(moving), runtime_rtf(self)),
                                  decision.validation_ego, decision.path, speed, body_shift_m=REAR_AXLE_SHIFT_M)
        if yielding:
            envelope = yield_envelope_mps(yielding.get('along', 0.)) if yielding['kind'] == 'stop' else 0.
            if yielding['kind'] == 'stop' and envelope > .5 and speed <= envelope+TUNNEL_YIELD_TOLERANCE_MPS:
                # Still stoppable short of the conflict: decelerate on the envelope.
                yielding = dict(yielding, kind='brake', cap_mps=envelope)
                cap = min(cap, envelope)
            elif yielding['kind'] == 'stop':
                emergency = True
            else:
                cap = min(cap, 6./3.6 if yielding['kind'] == 'slow' else yielding['v_mps'])
        reason = (decision.stop_reason or ('tunnel moving object conflict' if yielding and
                  yielding['kind'] == 'stop' else 'tunnel lead conflict' if emergency else None))
        decision = decision.decided(allowed_mps=cap, emergency=emergency, stop_reason=reason,
            lead=getattr(self, '_lead_state', None), moving_conflict=yielding,
            traffic_path_digest=decision.path_digest(), traffic_frame_digest=decision.frame_digest())
        return decision, lead, yielding

    def retry_tunnel_plan(self, decision, route, corridor, source_frame, points, objects,
                          moving, cruise_mps, decision_lag=0., exit_zone=False, mapped_walls=True):
        """Try one replacement, then recheck geometry and traffic before adoption.

        An imminent conflict while moving is stopped by the current decision;
        its next normal planning cycle performs the search. A stationary car or
        a newly obstructed distant path can use this bounded same-cycle retry.
        """
        from gui import RosPathPublisher
        lead = decision.validation.get('lead') or {}
        conflict = decision.validation.get('moving_conflict')
        horizon = min(60., float(np.linalg.norm(np.diff(decision.path, axis=0), axis=1).sum()))
        geometry = decision.validation.get('geometry_clearance_m', decision.forward_clear_m)
        lead_stop = (lead.get('cap_kmh') is not None and lead['cap_kmh'] <= .18
                     and not lead.get('held') and not (lead.get('v_lead_kmh') or 0.) > 3.6)
        risky = (decision.validation.get('latest_obstacle', 60.) < horizon-.3 or
                 geometry < min(horizon, decision.validation.get('initial_geometry_m', geometry))-.3 or lead_stop)
        if (not risky or decision.validation.get('blocked') or conflict or (mapped_walls and not TUNNEL_LATTICE)
                or not decision.fresh(max_age_s=.25) or not decision.validation.get('pose_valid')
                or not decision.validation.get('valid')):
            return decision, points, None
        now = time.monotonic()
        if ((decision.emergency and abs(float(decision.control_ego.get('speed_kmh', 0.))) > .5)
                or now-getattr(self, '_tunnel_final_retry_at', -9.) < .25):
            return decision.decided(stop_reason=decision.stop_reason, replan_deferred=True), points, None
        scan = self.lidar.scan_snapshot()
        if scan.get('stamp') != decision.sensor_stamp:
            return decision.decided(stop_reason=decision.stop_reason, replan_deferred=True), points, None
        pose = self.tunnel_odometry.at_stamp(float(decision.control_ego.get('stamp', 0.)))
        scan_pose = self.tunnel_odometry.at_stamp(decision.sensor_stamp)
        if pose is None or scan_pose is None:
            return decision, points, None
        self._tunnel_final_retry_at = now
        frame = decision.validation_ego
        route = points_at_pose(route, source_frame, frame)
        corridor = corridor_at_pose(corridor, source_frame, frame)
        from lattice_planner import object_points
        cars = [obj for obj in objects if obj[4] == 'npc']
        retry_points = points if not cars else np.vstack((points, object_points(cars, frame)))
        names = ('_tunnel_lattice_prev', '_fallback_lattice_prev', '_tunnel_retry_at', '_tunnel_tight_until',
                 '_tunnel_lead_tracker', '_tunnel_lead_world', '_lead_wait_since', '_lead_bypass', '_lead_state')
        saved = {name: copy.deepcopy(getattr(self, name)) for name in names if hasattr(self, name)}
        accepted = False
        try:
            if mapped_walls:
                plan, _ = self.tunnel_lattice_plan(route, frame, retry_points, objects, corridor, now)
            else:
                plan = self.fallback_lattice(route, frame, retry_points, corridor=corridor)
            if plan is None or plan.get('blocked'):
                return decision.decided(stop_reason=decision.stop_reason,
                    replan_attempted=True, replan_accepted=False), points, None
            path = RosPathPublisher.resample(plan['path'])
            components = self.tunnel_path_checks(path, frame, retry_points, corridor,
                                                 validation=dict(plan, mapped_walls=mapped_walls))
            from tunnel_visibility import ObservedLidarCoverage
            observed = ObservedLidarCoverage(scan['xyz'], scan['ground_plane']).clear_distance(
                points_at_pose(path, pose, scan_pose), 60., exclude_initial_footprint=True)
            raw_observed = observed
            if exit_zone:
                observed = max(observed, TUNNEL_EXIT_OBSERVED_FLOOR_M)
            clear = min(min(components.values()), observed)
            trial, trial_points = self.checked_tunnel_plan(path, decision.control_ego, frame, scan,
                retry_points, min(cruise_mps, plan['speed_cap_mps'],
                    latency_speed_cap(clear, decision_lag, TUNNEL_BRAKE_MPS2)), clear, False,
                dict(valid=True, pose_valid=True, blocked=False, observed=observed,
                    observed_raw=raw_observed, mapped_walls=mapped_walls,
                    generator=plan.get('generator'), side_margin_m=plan['side_margin_m'],
                    end_margin_m=plan['end_margin_m'], **components), corridor=corridor)
            trial, _, _ = self.tunnel_traffic_checks(trial, objects, trial_points, moving)
            required = max(8., float(plan.get('safe_horizon_m', 0.)))
            accepted = bool(trial.forward_ready(trial.control_ego) and trial.forward_clear_m >= required-.3)
            if accepted:
                return trial.decided(replan_attempted=True, replan_accepted=True), trial_points, plan
            return decision.decided(stop_reason=decision.stop_reason, replan_attempted=True,
                replan_accepted=False, replan_reason=trial.stop_reason), points, None
        except Exception as error:
            return decision.decided(stop_reason=decision.stop_reason, replan_attempted=True,
                replan_accepted=False, replan_reason=str(error)[:256]), points, None
        finally:
            if not accepted:
                for name in names:
                    if name in saved:
                        setattr(self, name, saved[name])
                    elif hasattr(self, name):
                        delattr(self, name)

    def init_tunnel(self):
        from tunnel_map import TunnelMap
        from tunnel_no_go import TunnelNoGoPublisher, TunnelNoGoReceiver
        filename = os.environ.get("STUDENT_TUNNEL_MAP", str(Path(__file__).with_name("tunnel_wall_map.json")))
        self.tunnel = TunnelMap.load(filename)
        self._tunnel_cache = None
        self._tunnel_plan_saved = None
        self._tunnel_wall_anchor = None
        self._tunnel_lateral_prior = None
        self.tunnel_odometry = TunnelOdometryReceiver()
        # Fed directly with this process's own record each frame (see
        # infer_tunnel); a ROS loopback copy would arrive one frame late.
        self.tunnel_no_go_receiver = TunnelNoGoReceiver(topic=None)
        self.tunnel_no_go_publisher = TunnelNoGoPublisher()
        self.tunnel_no_go_publisher.update([], [], None, 0., False)
        self.init_tunnel_model_policy()

    def init_tunnel_model_policy(self):
        """STUDENT_TUNNEL_POLICY=model: route scope and model checks over the route
        indices the wall map's own tunnel test calls inside (3782..4012 today)."""
        self.tunnel_route_policy = self.tunnel_model_policy = None
        self._tunnel_lane_hold = None
        if not TUNNEL_MODEL or getattr(self, "waypoint_xy", None) is None:
            return
        import model_avoidance
        from tunnel_static_portable import TunnelLaneHold
        bounds = model_avoidance.tunnel_route_bounds(self.waypoint_xy, self.tunnel.active)
        if bounds is None:
            print("[tunnel] model policy: route never enters the wall map", flush=True)
            return
        self.tunnel_route_policy, self.tunnel_model_policy = model_avoidance.make_tunnel_policies(
            self.waypoint_xy, bounds)
        self._tunnel_lane_hold = TunnelLaneHold()
        print(f"[tunnel] model policy over route idx {bounds[0]}..{bounds[1]}", flush=True)

    def tunnel_exit_anchor(self, match, anchor, scan, stamp, scan_ego, scan_odometry):
        """Near the exit, carry the last verified wall match forward by odometry."""
        to_exit = float(self.tunnel.end - (np.array([scan_ego["x"], scan_ego["y"]]) -
                                           self.tunnel.origin) @ self.tunnel.axis)
        origin = anchor["odometry"]
        travelled = math.hypot(scan_odometry["x"]-origin["x"], scan_odometry["y"]-origin["y"])
        age = stamp-anchor["stamp"]
        # Relative-frame anchors count only the time spent driving (a stop at the
        # portal does not move the carried pose); others their full age, as before.
        driven_s = float(anchor.get("moving_s", age))
        if not (to_exit <= TUNNEL_EXIT_ANCHOR_M and 0. <= age and driven_s <= TUNNEL_EXIT_ANCHOR_S
                and travelled <= TUNNEL_EXIT_ANCHOR_M+15.):
            return match
        predicted = propagate_match(anchor["match"]["aligned_ego"], origin, scan_odometry)
        # Odometry and ESKF must agree on the motion since the anchor; the ESKF's
        # offset from the walls measured with the anchor is not a disagreement.
        correction, yaw_error = anchor_disagreement(anchor, predicted, scan_ego)
        if correction > self.tunnel.max_lateral or yaw_error > self.tunnel.max_yaw:
            return match
        projected = self.tunnel.project_scan(scan["xyz"], predicted, scan["ground_plane"],
                                             anchor["match"]["wall_stats"])
        if not projected.get("valid"):
            return match
        return dict(projected, reason="tunnel exit: last wall match carried by odometry",
                    exit_anchor=dict(failed=match.get("reason"), age_s=round(age, 2), driven_s=round(driven_s, 2),
                                     travelled_m=round(travelled, 1), to_exit_m=round(to_exit, 1)))

    def tunnel_wall_prior(self):
        """Wall-fit window shift (m): the last measured lateral correction, else 0."""
        prior = getattr(self, "_tunnel_lateral_prior", None)
        return float(prior["correction_m"]) if prior else 0.

    def tunnel_relative_frame_match(self, match, scan, stamp, scan_ego, scan_odometry):
        """TUNNEL_RELATIVE_FRAME: plan on fresh validated walls whatever the ESKF error.

        Returns (match, relative projection or None). Validated walls update the
        fit-window prior and the anchor (map-valid or relative alike); without them
        the anchor is carried only by the existing short-occlusion and exit-zone
        rules, checked net of the correction measured with it. No localization
        constraint is produced here.
        """
        stats = match.get("wall_stats") or {}
        walls_measured = bool(match.get("valid") or (match.get("body_walls") or {}).get("validated") is True)
        correction = stats.get("lateral_correction_m")
        if walls_measured and correction is not None and math.isfinite(float(correction)):
            self._tunnel_lateral_prior = dict(correction_m=float(correction), stamp=stamp)
        relative = None
        if not match.get("valid") and hasattr(self.tunnel, "relative_projection"):
            relative = self.tunnel.relative_projection(scan["xyz"], scan_ego, scan["ground_plane"], match,
                                                       max_correction_m=None)
        measured = match if match.get("valid") else relative
        anchor = getattr(self, "_tunnel_wall_anchor", None)
        if measured is not None and measured.get("valid"):
            self._tunnel_wall_anchor = dict(stamp=stamp, match=measured, odometry=scan_odometry,
                                            scan_ego=dict(scan_ego), moving_s=0., tick=(stamp, scan_odometry))
            return match, relative
        if anchor is None:
            return match, relative
        tick_stamp, tick_odometry = anchor.get("tick") or (anchor["stamp"], anchor["odometry"])
        if stamp > tick_stamp:
            moved = math.hypot(scan_odometry["x"]-tick_odometry["x"], scan_odometry["y"]-tick_odometry["y"])
            if "moving_s" in anchor and moved > .02+.1*(stamp-tick_stamp):
                anchor["moving_s"] = float(anchor["moving_s"])+stamp-tick_stamp
            anchor["tick"] = (stamp, scan_odometry)
        if (match.get("reason") == "both supported tunnel walls required"
                and hasattr(self.tunnel, "project_scan") and 0. <= stamp-anchor["stamp"] <= .25):
            origin = anchor["odometry"]
            travelled = math.hypot(scan_odometry["x"]-origin["x"], scan_odometry["y"]-origin["y"])
            turn = abs((scan_odometry["heading_deg"]-origin["heading_deg"]+180.) % 360.-180.)
            predicted = propagate_match(anchor["match"]["aligned_ego"], origin, scan_odometry)
            offset, yaw_error = anchor_disagreement(anchor, predicted, scan_ego)
            if travelled <= 2. and turn <= 3. and offset <= self.tunnel.max_lateral and yaw_error <= self.tunnel.max_yaw:
                match = self.tunnel.project_scan(scan["xyz"], predicted, scan["ground_plane"],
                                                 anchor["match"]["wall_stats"])
        if (not match.get("valid") and TUNNEL_EXIT_ANCHOR
                and hasattr(self.tunnel, "project_scan") and hasattr(self.tunnel, "end")):
            match = self.tunnel_exit_anchor(match, anchor, scan, stamp, scan_ego, scan_odometry)
        return match, relative

    def tunnel_model_static(self, dense, aligned, objects, lane_ok, clear, adjacent_offsets, previous,
                            nearest_idx, preview, exit_zone=False):
        """STUDENT_TUNNEL_POLICY=model static slot, in example_integration.on_frame order.

        Moving clusters (TunnelTracks: kind pedestrian) and moving cars stay with the
        lead / moving-conflict checks; wall-like clusters stay in the raw clearance only.
        The v6 LEFT/RIGHT is latched (TunnelLaneHold) only while the package itself sees
        a static obstacle; the preview copy never changes the hold.
        Returns (plan, hazards, confirmed hazards, v6 decision, v6 reason)."""
        import model_avoidance as ma
        policy = getattr(self, "tunnel_route_policy", None)
        state = (policy.state(nearest_idx) if policy is not None else
                 dict(tunnel_active=True, cross_center_allowed=True, defer_rejoin=False,
                      rejoin_phase=False, rejoin_distance_m=0.))
        static, dynamic = ma.split_static([o for o in objects if not ma.wall_like(o)], in_tunnel=True)
        hazards_seen, engagement = ma.confirmed_hazards(dense, aligned, static, body_shift_m=REAR_AXLE_SHIFT_M)
        decision, why = ma.v6_decision(self, refresh=not preview)
        hold = None if preview else getattr(self, "_tunnel_lane_hold", None)
        if exit_zone and hold is not None:
            hold.value = None   # the exit zone owns the path: no lane hold or deferred return there
        held = hold.value if hold is not None else None
        if hold is not None and hazards_seen and decision is not None and not exit_zone:
            held = hold.observe_model(decision["action"], decision["p"], state)
        direction, source = ma.choose_direction(previous, hazards_seen, decision, held)
        defer = bool(state["defer_rejoin"] and held is not None)
        plan, hazards, info = ma.plan_bypass(
            dense, aligned, static, lane_ok, clear, direction, previous, ma.tunnel_offsets(adjacent_offsets),
            defer_rejoin=defer, deferred_offset_m=float(held["offset_m"]) if held else 0.,
            deferred_distance_m=float(state["rejoin_distance_m"]), body_shift_m=REAR_AXLE_SHIFT_M)
        shape, bridge = None, None
        if (TUNNEL_SHAPE and not preview and
                ((plan is not None and plan.get("blocked")) or (plan is None and info.get("blocked_without_hazard")))):
            # No plan this frame: keep the committed path while it is still clear.
            plan, bridge = ma.bridge_tunnel_plan(plan, previous, aligned, static, clear,
                                                 getattr(self, "_tunnel_bridge_n", 0))
        self._tunnel_bridge_n = getattr(self, "_tunnel_bridge_n", 0) + 1 if (bridge or {}).get("applied") else 0
        if TUNNEL_SHAPE and not preview and not exit_zone and not (bridge or {}).get("applied"):
            # Same side and checks, a faster and steadier shape (model_avoidance).
            plan, shape = ma.smooth_tunnel_plan(
                plan, hazards, dense, aligned, static, lane_ok, clear, previous,
                wall_room=lambda path, horizon: self.tunnel.path_clearance(
                    path, aligned, horizon, margin=ma.TUNNEL_SHAPE_WALL_MARGIN_M))
        self._tunnel_model_log = dict(
            route_state={k: state[k] for k in ("tunnel_active", "defer_rejoin", "rejoin_phase")},
            rejoin_distance_m=round(float(state["rejoin_distance_m"]), 1),
            static_n=len(static), dynamic_n=len(dynamic),
            hazard_front_m=round(float(hazards_seen[0]["front"]), 1) if hazards_seen else None,
            engagement_m=round(float(engagement), 1),
            v6=(dict(action=decision["action"], p=round(float(decision["p"]), 3),
                     age_s=round(float(decision["age_s"]), 3)) if decision else None),
            v6_reason=why, direction=direction, direction_source=source, lane_hold=held,
            defer_rejoin=defer, plan_kind=(plan or {}).get("kind"),
            blocked=bool(plan and plan.get("blocked")), preview=bool(preview), shape=shape, bridge=bridge, **info)
        return plan, hazards, hazards_seen, decision, why, state

    def observe_tunnel_lane_hold(self, plan, state, decision_valid):
        """After the final checks: the side actually driven updates the lane hold."""
        hold = getattr(self, "_tunnel_lane_hold", None)
        if hold is None or state is None or not decision_valid:
            return
        executed = plan if plan and not plan.get("blocked") and plan.get("generator") == "portable" else None
        if plan is None and state.get("rejoin_phase"):
            hold.value = None   # back on the centre route in the rejoin phase: the return is complete
            return
        if (executed is not None and hold.value and executed.get("kind") != "rejoin"
                and float(executed.get("direction", 0.)) * float(hold.value["direction"]) < 0.):
            hold.value = None   # the model's latch never overrides the side being driven
        hold.observe_plan(executed, state)

    TUNNEL_FALLBACK_ERRORS = ("tunnel wall match unavailable", "tunnel scan pose unavailable",
                              "tunnel odometry discontinuity", "tunnel route join infeasible",
                              # one-frame timing glitches under CPU load sent stop pulses at 37-39 km/h
                              "tunnel lidar stale", "tunnel scan pose time mismatch", "tunnel lidar stamp frozen")

    def tunnel_coverage_escape(self, plan, hazards, dense, aligned, objects, clear, observed_of, observed_now,
                               budget_s=.04, required_observed_m=None):
        """(plan or None, info): leave a line whose own footprint the LiDAR floor coverage
        cannot confirm for a nearby line that it can, on the planner's side and checks.

        follow-20261010T173230Z 02:41:45-02:42:11: a bypass 0.3 m from the left kerb read
        "observed" 1 m with walls and returns clear for 60 m; the coverage cap stopped the
        car and the 3 km/h peek crawled 12 m along the kerb on the same line for 25 s."""
        import model_avoidance as ma
        from gui import RosPathPublisher
        started = time.perf_counter()
        info = dict(applied=False, observed_m=round(float(observed_now), 1))
        direction = float(plan.get("direction") or 1.)
        offset = abs(float(plan.get("offset_m", 0.)))
        static, _ = ma.split_static([o for o in objects if not ma.wall_like(o)], in_tunnel=True)
        hold_until = ma._group_hold_until(hazards) or 0.
        needed = ma.tunnel_required_m(abs(float(aligned.get("speed_kmh", 0.)))/3.6)
        best = None
        for value in sorted({round(max(0., offset-1.), 2), round(max(0., offset-2.), 2),
                             round(min(offset, .9), 2), 0.}, reverse=True):
            for transition in (10., 14., 20.):
                if time.perf_counter()-started > budget_s:
                    info["budget_exhausted"] = True
                    break
                hold = max(6., hold_until-transition) if value > 0. else 0.
                candidate, peak, _ = ma._geometry.build_forced_bypass(
                    dense, direction if value > 0. else 0., offset_m=value, hold_m=hold,
                    max_curvature_1pm=ma._static.MAX_CONTROL_CURVATURE, transitions=(transition,))
                if candidate is None or peak > ma._static.MAX_CONTROL_CURVATURE:
                    continue
                local = np.asarray(candidate, dtype=float)[:, :2]
                horizon = min(60., float(np.linalg.norm(np.diff(local, axis=0), axis=1).sum()))
                need = max(needed, hold_until if value > 0. else 0.)
                if horizon < need:
                    continue
                geometric = min(ma._static.static_clear_distance(candidate, aligned, static, horizon,
                                                                 body_shift_m=REAR_AXLE_SHIFT_M),
                                float(clear(local, horizon)))
                if geometric < need:
                    continue
                resampled = RosPathPublisher.resample(local)
                if np.linalg.norm(resampled[0]) > .01:
                    resampled = RosPathPublisher.resample(np.vstack(([0., 0.], resampled)))
                observed = float(observed_of(resampled))
                score = min(geometric, observed)
                if best is None or score > best[0]:
                    best = (score, observed, value, transition, hold, candidate, peak, geometric)
        info["ms"] = round((time.perf_counter()-started)*1000., 1)
        improvement = max(observed_now+2., 1.5*observed_now)
        if required_observed_m is not None:
            # A nearby line that fully supports the current braking envelope
            # need not add 50 percent more range. Retain the two-metre minimum
            # gain so tiny changes in coverage cannot alternate the path.
            improvement = max(observed_now+2., min(improvement, required_observed_m))
        if best is None or best[1] < improvement:
            return None, dict(info, best_observed_m=round(best[1], 1) if best else None)
        score, observed, value, transition, hold, candidate, peak, geometric = best
        escaped = dict(plan, path=candidate, kind="bypass" if value > 0. else "rejoin", direction=direction,
                       offset_m=value, transition_m=transition, hold_m=hold, curvature=peak,
                       clear_distance_m=geometric, hold_until_m=hold_until, reused=False, reshaped=True,
                       coverage_escape=True, generator="portable")
        return escaped, dict(info, applied=True, offset_m=value, transition_m=transition,
                             best_observed_m=round(observed, 1))

    def tunnel_vehicle_objects(self, objects, ego):
        """Use fresh full-size LiDAR car boxes before the tunnel and through it.

        The configured detector region overlaps the highway handoff. Camera
        cars matched to a measured box are replaced once; unrelated obstacles
        and pedestrians remain. Unknown vehicle velocity never removes its body.
        """
        config = getattr(getattr(self, 'highway', None), 'config', {})
        index = int(((getattr(self, 'latest_status', None) or {}).get('path') or {}).get('nearest_idx', -1))
        regions = config.get('lidar_regions', ())
        self._tunnel_vehicle_status = None
        if not any(region['from_idx'] <= index <= region['to_idx'] for region in regions):
            return objects
        receiver = getattr(self, 'vehicle_receiver', None)
        cars, ready, status = receiver.snapshot() if receiver is not None else ([], False, {})
        self._tunnel_vehicle_status = dict(status, detector_ready=bool(ready), car_count=len(cars))
        if not ready:
            return objects
        # Tunnel vehicles can legitimately occupy the adjacent wall-to-wall
        # road; do not discard them using a single HD-map lane centre.
        cars = [car for car in cars if len(car) >= 11 and car[4] == 'npc']
        merged = [obj for obj in objects if not ((obj[4] == 'npc' or obj[3] == 'fusion_parked_vehicle')
                  and any(math.hypot(obj[0]-car[0], obj[1]-car[1]) < 2 for car in cars))]
        return merged + cars

    def semantic_obstacle_points(self, ego, max_age_s=.5):
        """Vehicle-frame outline points of fused camera-semantic + LiDAR objects
        (obstacles, people, cars) ahead inside the tunnel width; (points, count)."""
        receiver = getattr(self, "object_receiver", None)
        try:
            if receiver is None or time.monotonic() - float(receiver.received) > max_age_s:
                return None, 0
            objects = receiver.snapshot()
            if not isinstance(objects, (list, tuple)):
                return None, 0
            from gui import local_xy
            pts, n = [], 0
            ring = np.linspace(0., 2. * math.pi, 12, endpoint=False)
            for o in objects:
                if o[4] not in ("obstacle", "pedestrian", "npc"):
                    continue
                f, l = local_xy(o[0], o[1], ego)
                if not (-3. < f < 45.) or abs(l) > 3.5:   # walls stay with the LiDAR wall model
                    continue
                if len(o) > 8 and np.isfinite(np.asarray(o[6:9], dtype=float)).all() and min(o[6:8]) > 0.:
                    from lattice_planner import object_points
                    # A 4.6 x 1.9 m parallel car must not become a 4.6 m wide
                    # disc that falsely occupies the neighbouring empty lane.
                    pts.append(object_points([o], ego))
                else:
                    r = max(.3, float(o[2]) / 2.)
                    pts.append(np.column_stack((f + r * np.cos(ring), l + r * np.sin(ring))))
                pts.append(np.array([[f, l]]))
                n += 1
            return (np.vstack(pts), n) if pts else (None, 0)
        except Exception:
            return None, 0

    @staticmethod
    def fallback_obstacle_points(scan):
        """Vehicle-frame xy of raw returns 0.3-2.5 m above the road, ego body removed."""
        if scan.get("xyz") is None:
            return None
        xyz = np.asarray(scan["xyz"], dtype=float).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        plane = scan.get("ground_plane")
        ground = (xyz[:, :2] @ np.asarray(plane[:2], float) + float(plane[2])) if plane is not None else -1.7
        height = xyz[:, 2] - ground
        keep = (height > .3) & (height < 2.5) & self_return_mask(xyz)
        return xyz[keep, :2]

    def fallback_lattice(self, route_path, ego, points, walls=None, source_ego=None, to_exit_m=None, corridor=None):
        """Raw-return bypass within fresh measured walls, or conservative bounds."""
        try:
            from lattice_planner import plan as lattice_plan, SIDE_MARGIN_M
            from planning.tunnel.free_space import free_space_candidates
            from lane_corridor import LaneCorridor
            from gui import MAX_CONTROL_CURVATURE
            route = np.asarray(route_path, dtype=float)[:, :2]
            n = len(route)
            if corridor is None:
                corridor = measured_wall_corridor(route, walls, source_ego or ego, ego)
                measured = corridor is not None
            else:
                measured = bool(getattr(corridor, 'measured_walls', False))
            if corridor is None:
                corridor = LaneCorridor(route, np.full(n, 4.0), np.full(n, -4.0), body_shift_m=REAR_AXLE_SHIFT_M)
            provider = getattr(getattr(self, 'path_publisher', None), '_safety_provider', None)
            recovery = (provider.recovery_inputs(ego, 'TUNNEL', requested_mps=TUNNEL_FALLBACK_KMH/3.6)
                        if provider is not None else None)
            now = time.monotonic()
            previous = max((p for p in (getattr(self, '_fallback_lattice_prev', None),
                                        getattr(self, '_tunnel_lattice_prev', None))
                            if p is not None and 0. <= now-p['stamp'] <= 1.),
                           key=lambda p: p['stamp'], default=None)
            objects = recovery['objects'] if recovery else []
            max_offset = TUNNEL_LATTICE_MAX_OFFSET_M if measured else 3.5
            legal_horizon = to_exit_m if measured else None
            common = dict(cruise_mps=TUNNEL_FALLBACK_KMH / 3.6, max_curvature=MAX_CONTROL_CURVATURE,
                          outside_max_m=0., edge_margin_m=.25, blocked_standoff_m=6.,
                          wide_side_m=1.6, w_lateral=.3, side_commit_s=TUNNEL_SIDE_COMMIT_S,
                          extra_clearance=recovery['clearance'] if recovery else None,
                          extra_speed_cap=recovery.get('speed_cap') if recovery else None,
                          legal_horizon_m=legal_horizon, max_offset_m=max_offset,
                          body_shift_m=REAR_AXLE_SHIFT_M)
            spaces = free_space_candidates(route, ego, points, corridor, objects=objects,
                legal_horizon_m=legal_horizon, side_margin_m=SIDE_MARGIN_M, edge_margin_m=.25,
                max_offset_m=max_offset, max_curvature=MAX_CONTROL_CURVATURE,
                previous_target=previous['target'] if previous is not None else None)
            result = lattice_plan(route, ego, points, objects, corridor, previous,
                candidate_targets=spaces['targets'], candidate_lengths=spaces['lengths'],
                candidate_quality=spaces['quality'], reuse_first=True,
                reuse_min_quality=spaces['reuse_min_quality'], **common)
            if result['blocked'] and now-getattr(self, '_fallback_full_retry_at', -9.) >= .5:
                self._fallback_full_retry_at = now
                expanded = lattice_plan(route, ego, points, objects, corridor, previous, **common)
                expanded['evaluated'] += result['evaluated']
                expanded['full_search_fallback'] = True
                result = expanded
        except Exception:
            return None
        self._fallback_lattice_prev = dict(stamp=time.monotonic(), target=result["target"],
            world_path=np.asarray(result['path'])[:, :2] @ rotation(ego).T+[ego['x'], ego['y']])
        if result["blocked"]:
            return None
        return dict(path=np.asarray(result["path"], dtype=float)[:, :2], speed_cap_mps=result["speed_cap_mps"],
                    clear_m=result['clear_m'], target=result['target'],
                    safe_horizon_m=result['horizon_m'], side_margin_m=result['side_margin_m'],
                    end_margin_m=result['end_margin_m'], generator='fallback', blocked=False,
                    candidates_evaluated=result['evaluated'], reused=result.get('reused', False),
                    full_search_fallback=result.get('full_search_fallback', False),
                    free_space={key: spaces[key] for key in ('passages', 'reason')},
                    dynamic_recovery_ids=recovery['ids'] if recovery else [],
                    reason=('measured wall corridor: ' if measured else '')+str(result.get("reason")))

    def tunnel_degraded_drive(self, error, ego, scan, route_path):
        """(speed cap, clear distance) for driving on ESKF + raw LiDAR, or None."""
        if not str(error).startswith(self.TUNNEL_FALLBACK_ERRORS):
            return None
        if float(ego.get("age_s", 999.)) > .25 or scan.get("xyz") is None or scan.get("age_s", 999.) > .6:
            return None
        if route_path is None or len(route_path) < 2:
            return None
        xyz = np.asarray(scan["xyz"], dtype=float).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        plane = scan.get("ground_plane")
        ground = (xyz[:, :2] @ np.asarray(plane[:2], float) + float(plane[2])) if plane is not None else -1.7
        height = xyz[:, 2] - ground
        keep = (height > .3) & (height < 2.5) & self_return_mask(xyz)
        distance = swept_clearance(np.asarray(route_path)[:, :2], xyz[keep, :2], 40., lateral_margin=.3,
                                   body_shift_m=REAR_AXLE_SHIFT_M)
        # Older scan: charge the distance driven since it was taken.
        travelled = max(0., float(ego.get("speed_kmh", 0.))) / 3.6 * float(scan.get("age_s", 0.))
        distance = max(0., distance - travelled)
        cap = min(TUNNEL_FALLBACK_KMH / 3.6, max(0., math.sqrt(1.75 ** 2 + 2. * TUNNEL_BRAKE_MPS2 * max(0., distance - 2.)) - 1.75))
        return cap, float(distance)

    def tunnel_lattice_plan(self, dense, aligned, points, objects, corridor, now):
        """Recheck the issued path, then propose the widest reachable passages."""
        from lattice_planner import plan as lattice_plan, SIDE_MARGIN_M, END_MARGIN_M
        from planning.tunnel.free_space import free_space_candidates
        from gui import MAX_CONTROL_CURVATURE
        previous = getattr(self, "_tunnel_lattice_prev", None)
        if previous is not None and not 0. <= now - previous["stamp"] <= 1.:
            previous = None
        # Raw LiDAR points only: cluster boxes of long structures (a 20 m box
        # at the exit) cut into the lane although every return was outside it.
        static_objects = []
        provider = getattr(getattr(self, 'path_publisher', None), '_safety_provider', None)
        zone_kmh, _ = self.speed_zones.kmh_at(int(((getattr(self, "latest_status", None) or {}).get("path") or {}).get("nearest_idx", 0)))
        if getattr(self, '_transition_approach', False):
            from gui import zone_speed_kmh
            zone_kmh = min(zone_kmh, zone_speed_kmh(self.speed_zones,
                int(((getattr(self, 'latest_status', None) or {}).get('path') or {}).get('nearest_idx', 0)))[0])
        recovery = (provider.recovery_inputs(aligned, 'TUNNEL', requested_mps=zone_kmh/3.6)
                    if provider is not None else None)
        if recovery is not None:
            static_objects = recovery['objects']
        # The wall corridor is straight and ends at the tunnel exit; the road
        # beyond curves away. Check lane legality only up to the exit, or every
        # candidate "left the corridor" near the end (virtual run).
        to_exit = (self.tunnel.end - (np.array([aligned["x"], aligned["y"]]) -
                   self.tunnel.origin) @ self.tunnel.axis) if hasattr(self.tunnel, "end") else None
        def control_path(candidate):
            candidate = np.asarray(candidate, dtype=float)[:, :2]
            ahead = candidate[candidate[:, 0] > -.5]
            candidate = ahead if len(ahead) >= 3 else candidate
            candidate = candidate[int(np.argmin(np.linalg.norm(candidate, axis=1))):].copy()
            if len(candidate) >= 3:
                arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(candidate, axis=0), axis=1))]
                candidate -= candidate[0] * np.clip(1. - arc / 3., 0., 1.)[:, None]
                candidate[0] = 0.
            return candidate
        def extra_clearance(path, horizon):
            clear = self.tunnel.path_clearance(path, aligned, horizon)
            return min(clear, recovery['clearance'](path, horizon)) if recovery is not None else clear
        common = dict(max_curvature=MAX_CONTROL_CURVATURE, legal_horizon_m=to_exit, outside_max_m=0.,
                      wide_side_m=1.6, w_lateral=.3, side_commit_s=TUNNEL_SIDE_COMMIT_S,
                      path_transform=control_path,
                      extra_clearance=extra_clearance, body_shift_m=REAR_AXLE_SHIFT_M)
        if recovery is not None:
            common['extra_speed_cap'] = recovery.get('speed_cap')
        def passage_plan(side, edge, cruise, prior, **kwargs):
            spaces = free_space_candidates(dense, aligned, points, corridor,
                objects=static_objects, legal_horizon_m=to_exit,
                side_margin_m=side, edge_margin_m=edge,
                max_offset_m=TUNNEL_LATTICE_MAX_OFFSET_M,
                max_curvature=MAX_CONTROL_CURVATURE,
                previous_target=prior['target'] if prior is not None else None)
            result = lattice_plan(dense, aligned, points, static_objects, corridor, prior,
                cruise_mps=cruise, side_margin_m=side, edge_margin_m=edge,
                max_offset_m=TUNNEL_LATTICE_MAX_OFFSET_M,
                candidate_targets=spaces['targets'], candidate_lengths=spaces['lengths'],
                candidate_quality=spaces['quality'], reuse_first=True,
                reuse_min_quality=spaces['reuse_min_quality'], **common, **kwargs)
            result['free_space'] = {key: spaces[key] for key in ('passages', 'reason')}
            # Complex geometry can require another return profile. Reserve the
            # full fan for a failed small search, and avoid repeating it at 15 Hz.
            if result['blocked'] and now-getattr(self, '_tunnel_full_retry_at', -9.) >= .5:
                self._tunnel_full_retry_at = now
                expanded = lattice_plan(dense, aligned, points, static_objects, corridor, prior,
                    cruise_mps=cruise, side_margin_m=side, edge_margin_m=edge,
                    max_offset_m=TUNNEL_LATTICE_MAX_OFFSET_M, **common, **kwargs)
                expanded['evaluated'] += result['evaluated']
                expanded['free_space'] = result['free_space']
                expanded['reused'] = False
                expanded['full_search_fallback'] = True
                result = expanded
            return result
        result = passage_plan(SIDE_MARGIN_M, .25, zone_kmh/3.6, previous,
                              blocked_standoff_m=6.)
        # A stopped bottleneck cannot change much in 0.5 s. Retry the tighter
        # margins at 2 Hz, without repeating the same full-width normal search.
        creeping = now < getattr(self, "_tunnel_tight_until", -1.)
        moving = float(aligned.get("speed_kmh", 0.)) >= 3.
        retry_due = moving or creeping or now - getattr(self, "_tunnel_retry_at", -9.) >= .5
        if result["blocked"] and retry_due and (not moving or creeping):
            self._tunnel_retry_at = now
            # Stopped in a bottleneck (studio "passage" 2.4-2.6 m, auto run 52 waited
            # 60 s behind a parked car): creep through with a tight margin.
            tight = passage_plan(.12, .12, TUNNEL_TIGHT_PASS_KMH/3.6, previous)
            if not tight["blocked"]:
                tight["speed_cap_mps"] = min(tight["speed_cap_mps"], TUNNEL_TIGHT_PASS_KMH / 3.6)
                tight["reason"] = "tight bottleneck pass: " + str(tight.get("reason"))
                result = tight
                self._tunnel_tight_until = now + 2.   # keep creeping instead of stop-go at 3 km/h
        path = np.asarray(result["path"], dtype=float)[:, :2]
        side_margin = float(result.get("side_margin_m", SIDE_MARGIN_M))
        end_margin = float(result.get("end_margin_m", END_MARGIN_M))
        tight_pass = side_margin < SIDE_MARGIN_M - 1e-6
        # The tunnel branch prepends the origin (0, 0); a first point slightly
        # behind it made a reversal kink ("route join infeasible", virtual run).
        # Origin joining happened before collision/curvature checks above.
        rotation_matrix = rotation(aligned)
        self._tunnel_lattice_prev = dict(stamp=now, target=result["target"],
                                         world_path=path @ rotation_matrix.T + [aligned["x"], aligned["y"]])
        hazards = ([dict(n="lidar", front=result["clear_m"], along=result["clear_m"], back=result["clear_m"])]
                   if result["blocked"] else [])
        if (abs(result["target"]) < .01 and not result["blocked"] and
                not tight_pass and not result.get("near_obstacle_points", 0)):
            # Nothing to avoid: the tunnel's own smooth route join applies
            # (a lattice "return" was reported as a bypass on recorded scans).
            self._tunnel_lattice_prev = None
            return None, hazards
        plan = dict(path=path, kind="bypass" if abs(result["target"]) > .01 else "rejoin",
                    direction=float(np.sign(result["target"])) or 1., offset_m=abs(result["target"]),
                    transition_m=30., hold_m=0., hold_until_m=float(result["hold_until"] or 0.),
                    safe_horizon_m=result["horizon_m"], clear_distance_m=result["clear_m"],
                    curvature=result["curvature"], speed_cap_mps=result["speed_cap_mps"],
                    steering_rate_cap_mps=result["speed_cap_mps"], steering_gradient_radpm=0.,
                    blocked=result["blocked"], generator="lattice", reused=result.get('reused', False),
                    selection_reason=result["reason"], search_max_offset_m=TUNNEL_LATTICE_MAX_OFFSET_M,
                    side_margin_m=side_margin, end_margin_m=end_margin, tight_pass=tight_pass,
                    planning_budget_exhausted=False, candidates_evaluated=result["evaluated"],
                    free_space=result.get('free_space'),
                    full_search_fallback=result.get('full_search_fallback', False))
        if recovery is not None:
            plan['dynamic_recovery_ids'] = recovery['ids']
        return plan, hazards

    def leave_tunnel(self):
        if getattr(self, "tunnel_no_go_publisher", None) is not None:
            self.tunnel_no_go_publisher.update([], [], None, 0., False)
        self._tunnel_cache = None
        self._tunnel_plan_saved = None
        self._tunnel_wall_anchor = None
        self._tunnel_lateral_prior = None
        self._tunnel_hold_offset = 0.
        if getattr(self, "_tunnel_lane_hold", None) is not None:
            self._tunnel_lane_hold.value = None   # a lap's lane hold never carries into the next
        for name in ("_tunnel_tracks", "_tunnel_memory", "_live_tunnel_obstacles", "_tunnel_view_points", "_tunnel_view_pose",
                     "_tunnel_decision_plan", "_tunnel_lead_tracker", "_tunnel_lead_world",
                     "_tunnel_detector_motion"):
            if hasattr(self, name):
                delattr(self, name)

    def checked_tunnel_plan(self, path, ego, aligned, scan, points, cap, clear_m,
                            emergency, validation, mode='TUNNEL', stop_reason=None, corridor=None):
        """Recheck a final path against any scan that arrived during its search.

        Wall/road/steering checks remain those of this path, never of the route
        it bypasses. A new scan is projected with wheel/IMU odometry into that
        same frame before lead following and the reverse decision consume it.
        """
        from gui import path_curvatures, MAX_CONTROL_CURVATURE
        from lattice_planner import LOW_SPEED_MAX_CURVATURE
        now = time.monotonic()
        age = max(0., float(scan.get('age_s', 999.)))
        sensor_stamp = float(scan.get('stamp') or 0.)
        latest = self.lidar.scan_snapshot() if self.lidar is not None else {}
        latest_stamp = float(latest.get('stamp') or 0.)
        # scan_snapshot reports age from the original receipt, even when the
        # source stamp is unchanged; rereading it must not reset that age.
        if latest_stamp == sensor_stamp:
            age = max(age, float(latest.get('age_s', 999.)))
        changed = (latest_stamp != sensor_stamp or latest.get('sequence') != scan.get('sequence'))
        validation = dict(validation, revalidated=True)
        if changed:
            # A search can outlive the pose it started with. Refresh the pose
            # first: that observation can arrive between the old and new pose
            # lookups. An earlier failed old-pose lookup must not veto the new
            # fresh history. Resolve all source times together afterwards.
            refresh_pose = latest_stamp > float(ego.get('stamp', 0.))+.12 and latest_stamp >= sensor_stamp
            fresh_ego = ego
            if refresh_pose:
                fresh_status = self.update_ego()
                fresh_ego = fresh_status.get('ego') or {}
            stamps = (float(ego.get('stamp', 0.)), float(fresh_ego.get('stamp', 0.)), latest_stamp)
            receiver = self.tunnel_odometry
            if hasattr(receiver, 'at_stamps'):
                old_pose, fresh_pose, scan_pose = receiver.at_stamps(stamps)
            else:
                old_pose, fresh_pose, scan_pose = [receiver.at_stamp(stamp) for stamp in stamps]
            validation['pose_refresh_sources'] = dict(
                original_ego=stamps[0], fresh_ego=stamps[1], lidar=latest_stamp,
                fresh_ego_age_s=float(fresh_ego.get('age_s', 999.)),
                scan_age_s=float(latest.get('age_s', 999.)),
                original_pose_available=old_pose is not None,
                fresh_pose_available=fresh_pose is not None,
                scan_pose_available=scan_pose is not None)
            if refresh_pose:
                if (old_pose is not None and fresh_pose is not None and fresh_ego.get('age_s', 999.) <= .25
                        and -.12 <= float(fresh_ego.get('stamp', 0.))-latest_stamp <= .6
                        and 0. <= float(latest.get('age_s', 999.)) <= .6
                        and fresh_ego.get('stamp', 0.) > ego.get('stamp', 0.)):
                    from gui import continue_static_plan, RosPathPublisher
                    fresh_aligned = dict(fresh_ego, **{key: value for key, value in
                        propagate_match(aligned, old_pose, fresh_pose).items() if key in ('x', 'y', 'heading_deg')})
                    local = points_at_pose(path, old_pose, fresh_pose)
                    world = local @ rotation(fresh_aligned).T+[fresh_aligned['x'], fresh_aligned['y']]
                    saved = dict(stamp=now, world_path=world, plan={})
                    check_points = (points_at_pose(points, old_pose, fresh_pose)
                                    if points is not None else None)
                    check_corridor = corridor_at_pose(corridor, old_pose, fresh_pose) if corridor is not None else None
                    steering_limit = (max(MAX_CONTROL_CURVATURE, LOW_SPEED_MAX_CURVATURE)
                                      if abs(float(fresh_ego.get('speed_kmh', 0.))) < 5.4 else MAX_CONTROL_CURVATURE)
                    continued = None
                    # A fixed six-metre heading join can turn a checked gentle
                    # curve into an infeasible sharp S after a pose refresh.
                    # Search longer joins with the same body/wall/width gates.
                    for join_m in (6., 8., 10., 12., 16., 20., 26.):
                        trial = continue_static_plan(saved, fresh_aligned, now, join_m=join_m)
                        if trial is None:
                            break
                        candidate = RosPathPublisher.resample(trial['path'])
                        peak = float(np.max(path_curvatures(candidate)))
                        if not math.isfinite(peak) or peak > steering_limit*1.15:
                            continue
                        if check_points is not None:
                            components = self.tunnel_path_checks(candidate, fresh_aligned,
                                check_points, check_corridor, validation=validation)
                            required = min(clear_m, max(8., abs(float(fresh_ego.get('speed_kmh', 0.)))/3.6*1.5+4.))
                            if min(components.values()) < required-.3:
                                continue
                        continued = dict(trial, path=candidate)
                        validation['pose_join_m'] = join_m
                        break
                    if continued is None:
                        validation.update(valid=False, pose_valid=False)
                        cap, emergency, stop_reason = 0., True, 'tunnel latest pose cannot join current path'
                    else:
                        path = RosPathPublisher.resample(continued['path'])
                        travelled = math.hypot(fresh_pose['x']-old_pose['x'], fresh_pose['y']-old_pose['y'])
                        clear_m = max(0., clear_m-travelled)
                        if 'observed' in validation:
                            validation['observed'] = max(0., validation['observed']-travelled)
                        if points is not None:
                            points = points_at_pose(points, old_pose, fresh_pose)
                        if corridor is not None:
                            corridor = corridor_at_pose(corridor, old_pose, fresh_pose)
                        validation['pose_advanced_s'] = float(fresh_ego['stamp'])-float(ego['stamp'])
                        ego, aligned, old_pose = fresh_ego, fresh_aligned, fresh_pose
                        validation['pose_refreshed'] = True
            source_age = float(ego.get('stamp', 0.))-latest_stamp
            raw = self.fallback_obstacle_points(latest)
            context = getattr(self, '_live_tunnel_corridor', None)
            if (old_pose is not None and scan_pose is not None and context is not None
                    and context.get('wall_stats') is not None and hasattr(self.tunnel, 'project_scan')):
                scan_aligned = propagate_match(aligned, old_pose, scan_pose)
                projected = self.tunnel.project_scan(latest['xyz'], scan_aligned,
                    latest['ground_plane'], context['wall_stats'])
                if projected.get('valid'):
                    raw = projected['obstacle_points']
            if (latest_stamp < sensor_stamp or raw is None or old_pose is None or scan_pose is None
                    or not -.12 <= source_age <= .6 or not 0. <= float(latest.get('age_s', 999.)) <= .6):
                emergency, cap, clear_m = True, 0., 0.
                validation.update(valid=False, pose_valid=False)
                stop_reason = 'tunnel latest scan cannot validate current path'
            else:
                raw = points_at_pose(raw, scan_pose, old_pose)
                points = raw if points is None else np.vstack((np.asarray(points)[:, :2], raw))
                guard_clear = self.tunnel_path_checks(path, aligned, raw, None,
                                                       validation=validation)['obstacle']
                clear_m = min(clear_m, guard_clear)
                validation.update(latest_obstacle=guard_clear, latest_scan_sequence=latest.get('sequence'))
                v = abs(float(ego.get('speed_kmh', 0.)))/3.6
                cap = min(cap, max(0., math.sqrt(1.75**2+2.*TUNNEL_BRAKE_MPS2*max(0., clear_m-2.))-1.75))
                stop_m = max(2., v*.25+v*v/(2.*TUNNEL_BRAKE_MPS2))
                if clear_m <= stop_m:
                    # Name the cause: a return on the path, the checked geometry carried in
                    # clear_m, or only the unseen floor (the coverage limit carried in
                    # clear_m; follow-20261010T173230Z 02:41:44.4). A detector box at our side
                    # read "coverage insufficient" with observed 10 m and obstacle 0 m for
                    # 2 min (follow-20261010T200235Z 05:09:44-05:12:30).
                    geometry = {key: float(validation[key]) for key in ('obstacle', 'wall', 'lane')
                                if key in validation}
                    limiting = min(geometry, key=geometry.get) if geometry else None
                    if guard_clear <= stop_m:
                        cause = 'tunnel latest obstacle on final path'
                    elif limiting is not None and geometry[limiting] <= stop_m:
                        cause = stop_reason or dict(obstacle='tunnel obstacle on path', wall='tunnel wall clearance',
                                                    lane='tunnel lane boundary')[limiting]
                    else:
                        cause = stop_reason or 'tunnel lidar coverage insufficient'
                    emergency, stop_reason = True, cause
            sensor_stamp = latest_stamp
            age = max(0., float(latest.get('age_s', 999.)))
        if validation.get('valid'):
            before = min((validation.get(key, 60.) for key in ('wall', 'lane', 'obstacle')))
            components = self.tunnel_path_checks(path, aligned, points, corridor, validation=validation)
            geometry_clear = min(components.values())
            validation.update(components, initial_geometry_m=before, geometry_clearance_m=geometry_clear)
            clear_m = min(clear_m, geometry_clear)
            cap = min(cap, max(0., math.sqrt(1.75**2+2.*TUNNEL_BRAKE_MPS2*max(0., clear_m-2.))-1.75))
            v = abs(float(ego.get('speed_kmh', 0.)))/3.6
            if geometry_clear <= max(2., v*.25+v*v/(2.*TUNNEL_BRAKE_MPS2)):
                reasons = dict(obstacle='tunnel obstacle on path', wall='tunnel wall clearance',
                               lane='tunnel lane boundary')
                emergency, stop_reason = True, stop_reason or reasons[min(components, key=components.get)]
        peak = float(np.max(path_curvatures(path))) if len(path) >= 3 else math.inf
        limit = max(MAX_CONTROL_CURVATURE, LOW_SPEED_MAX_CURVATURE) if abs(float(ego.get('speed_kmh', 0.))) < 5.4 else MAX_CONTROL_CURVATURE
        steering_valid = math.isfinite(peak) and peak <= limit*1.15
        cap = (min(cap, curve_speed_cap(path, abs(float(ego.get('speed_kmh', 0.)))/3.6)[0])
               if steering_valid else 0.)
        validation.update(steering_valid=steering_valid, curvature_1pm=peak if math.isfinite(peak) else None,
            sensor_times=dict(lidar=sensor_stamp, ego=ego.get('stamp'),
                objects=getattr(getattr(self, 'object_receiver', None), 'observation_stamp', None),
                vehicles=(getattr(self, '_tunnel_vehicle_status', None) or {}).get('stamp')))
        if not steering_valid:
            cap, emergency, stop_reason = 0., True, 'tunnel final path steering infeasible'
        publisher = getattr(self, 'path_publisher', None)
        plan_id = getattr(publisher, '_plan_sequence', 0)+1
        decision = DrivingPlan(plan_id, path, ego, aligned, sensor_stamp, age, now,
            max(0., float(cap)), max(0., float(clear_m)), bool(emergency), validation, mode, stop_reason)
        decision = decision.decided(stop_reason=stop_reason, geometry_path_digest=decision.path_digest(),
                                    geometry_frame_digest=decision.frame_digest())
        if not decision.fresh(now):
            decision = decision.decided(allowed_mps=0., emergency=True,
                stop_reason=stop_reason or 'tunnel plan sensor stale', valid=False)
        return decision, points

    def infer_tunnel(self, status, ego):
        # Import lazily: gui owns the shared planner, while Runtime mixes in this branch.
        from gui import (AsyncDecisionLog, RosPathPublisher, continue_static_plan,
                         plan_static_bypass, static_speed_cap_mps, trajectory_diagnostics,
                         USE_LATTICE, MAX_CONTROL_CURVATURE as LATTICE_MAX_CURVATURE)
        started, now = time.perf_counter(), time.monotonic()
        # Decision interval (run 222: 3-7 Hz, 1.2-1.8 Hz when stopped, on a loaded PC).
        last_decision = getattr(self, "_tunnel_last_decision", None)
        interval = now - last_decision if last_decision is not None else 0.
        self._tunnel_last_decision = now
        if not hasattr(self, "_tunnel_intervals") or not 0. <= interval <= 2.:
            self._tunnel_intervals = deque(maxlen=6)
        if 0. < interval <= 2.:
            self._tunnel_intervals.append(interval)
        decision_lag = max(self._tunnel_intervals, default=0.)
        decision_lag = decision_lag if decision_lag > TUNNEL_SLOW_DECISION_S else 0.
        scan = self.lidar.scan_snapshot() if self.lidar else {}
        stamp = float(scan.get("stamp") or 0.)
        ego_stamp = float(ego.get("stamp") or 0.)
        error = None
        if self.input_error or float(ego.get("age_s", 999.)) > .25:
            error = "tunnel localization stale"
        elif scan.get("xyz") is None or scan.get("age_s", 999.) > .25:
            error = "tunnel lidar stale"
        elif stamp <= 0. or ego_stamp <= 0. or not -.12 <= ego_stamp - stamp <= .25:
            error = "tunnel scan pose time mismatch"
        cached = getattr(self, "_tunnel_cache", None)
        fresh_scan = not cached or stamp != cached["stamp"]
        if not error and cached and not fresh_scan and now - cached["received"] > .25:
            error = "tunnel lidar stamp frozen"
        if cached and stamp < cached["stamp"]:
            # A reset cannot carry a manoeuvre or scan from the old timeline.
            self._tunnel_cache = cached = None
            self._tunnel_plan_saved = None
            self._tunnel_wall_anchor = None
            self._tunnel_lateral_prior = None
            if getattr(self, "_tunnel_lane_hold", None) is not None:
                self._tunnel_lane_hold.value = None
            if hasattr(self, "_tunnel_memory"):
                del self._tunnel_memory
            if hasattr(self, "_tunnel_tracks"):
                self._tunnel_tracks.reset()
            error = "tunnel scan clock reset"
        if not error and fresh_scan:
            scan_ego = self.ego_receiver.at_stamp(stamp)
            scan_odometry = self.tunnel_odometry.at_stamp(stamp)
            if scan_ego is None and scan_odometry is not None and abs(ego_stamp-stamp) <= .12:
                pose_odometry = self.tunnel_odometry.at_stamp(ego_stamp)
                if pose_odometry is not None:
                    scan_ego = propagate_match(ego, pose_odometry, scan_odometry)
            if scan_ego is None or scan_odometry is None:
                error = "tunnel scan pose unavailable"
            else:
                # Relative frame: centre the wall-fit window on the last measured
                # wall-frame lateral (and search further) instead of the raw ESKF.
                window = (dict(lateral_prior_m=self.tunnel_wall_prior(), search_m=TUNNEL_WALL_SEARCH_M)
                          if TUNNEL_RELATIVE_FRAME else {})
                if getattr(self, '_transition_approach', False):
                    match = self.tunnel.process(scan["xyz"], scan_ego, scan["ground_plane"], approach=True, **window)
                else:
                    match = self.tunnel.process(scan["xyz"], scan_ego, scan["ground_plane"], **window)
                anchor = getattr(self, "_tunnel_wall_anchor", None)
                relative_frame = None
                if TUNNEL_RELATIVE_FRAME:
                    match, relative_frame = self.tunnel_relative_frame_match(
                        match, scan, stamp, scan_ego, scan_odometry)
                elif match.get("valid"):
                    self._tunnel_wall_anchor = dict(stamp=stamp, match=match, odometry=scan_odometry)
                elif (match.get("reason") == "both supported tunnel walls required" and anchor
                        and hasattr(self.tunnel, "project_scan") and 0. <= stamp-anchor["stamp"] <= .25):
                    origin = anchor["odometry"]
                    travelled = math.hypot(scan_odometry["x"]-origin["x"], scan_odometry["y"]-origin["y"])
                    turn = abs((scan_odometry["heading_deg"]-origin["heading_deg"]+180.) % 360.-180.)
                    predicted = propagate_match(anchor["match"]["aligned_ego"], origin, scan_odometry)
                    correction = math.hypot(predicted["x"]-scan_ego["x"], predicted["y"]-scan_ego["y"])
                    yaw_error = abs((predicted["heading_deg"]-scan_ego["heading_deg"]+180.) % 360.-180.)
                    if travelled <= 2. and turn <= 3. and correction <= self.tunnel.max_lateral and yaw_error <= self.tunnel.max_yaw:
                        match = self.tunnel.project_scan(scan["xyz"], predicted, scan["ground_plane"],
                                                        anchor["match"]["wall_stats"])
                if (not TUNNEL_RELATIVE_FRAME and not match.get("valid") and TUNNEL_EXIT_ANCHOR and anchor
                        and hasattr(self.tunnel, "project_scan") and hasattr(self.tunnel, "end")):
                    match = self.tunnel_exit_anchor(match, anchor, scan, stamp, scan_ego, scan_odometry)
                from tunnel_visibility import ObservedLidarCoverage
                cached = self._tunnel_cache = dict(stamp=stamp, scan_ego=scan_ego, match=match,
                    scan_odometry=scan_odometry, received=now,
                    coverage=ObservedLidarCoverage(scan["xyz"], scan["ground_plane"]))
                if TUNNEL_RELATIVE_FRAME:
                    # Already projected without the 1.2 m cap; the lazy default below must not redo it.
                    cached['relative_projection'] = relative_frame
        match = cached["match"] if cached else {}
        map_match = match
        if not error and cached and not match.get('valid') and hasattr(self.tunnel, 'relative_projection'):
            if 'relative_projection' not in cached:
                cached['relative_projection'] = self.tunnel.relative_projection(
                    scan['xyz'], cached['scan_ego'], scan['ground_plane'], match)
            relative = cached['relative_projection']
            if relative is not None:
                match = relative
        if not error and not match.get("valid"):
            error = "tunnel wall match unavailable: " + str(match.get("reason", "no scan"))
        current_odometry = self.tunnel_odometry.at_stamp(ego_stamp) if not error else None
        if not error and current_odometry is None:
            error = "tunnel scan pose unavailable"
        if not error:
            od = cached["scan_odometry"]
            travelled = math.hypot(current_odometry["x"]-od["x"], current_odometry["y"]-od["y"])
            dt = abs(ego_stamp - stamp)
            expected = max(ego.get("speed_kmh", 0.), current_odometry.get("speed_kmh", 0.),
                           od.get("speed_kmh", 0.)) / 3.6 * dt
            turn = abs((current_odometry["heading_deg"]-od["heading_deg"]+180.) % 360.-180.)
            if travelled > expected + .35 or turn > 3. + 70. * dt:
                error = "tunnel odometry discontinuity"
        aligned = ego
        if not error:
            predicted = propagate_match(match["aligned_ego"], cached["scan_odometry"], current_odometry)
            aligned = dict(ego, **{k: predicted[k] for k in ("x", "y", "heading_deg")})
        # Wall aiding must not wait behind avoidance search or rendering. Its
        # validity depends on the matched scan and odometry, not route clearance.
        if (not error and fresh_scan and match.get("constraint") and self.path_publisher
                and not getattr(self, '_transition_approach', False)
                and hasattr(self.path_publisher, "publish_tunnel_constraint")):
            constraint = dict(match.get("constraint") or {}, stamp=ego_stamp,
                map_point=[aligned["x"], aligned["y"]], body_point=[0., 0.],
                yaw_measurement_rad=math.radians(aligned["heading_deg"]), scan_stamp=stamp)
            self.path_publisher.publish_tunnel_constraint(constraint)
        localization_ms = (time.perf_counter() - started) * 1000.
        planning_status = dict(status, ego=aligned, path=dict(status["path"]))
        if hasattr(self, "waypoint_xy"):
            distances = np.linalg.norm(self.waypoint_xy - [aligned["x"], aligned["y"]], axis=1)
            nearest = int(np.argmin(distances))
            planning_status["path"].update(nearest_idx=nearest, distance_m=float(distances[nearest]))
        _, model_route, display_route, dense = self.route_tensor(aligned, planning_status)
        dense = np.asarray(dense, dtype=float)[:, :2]
        # Inside the tunnel the whole wall-to-wall width is free road. After a
        # bypass the reference lane moves to where the car is, so it drives
        # on in parallel and returns only TUNNEL_REJOIN_BEFORE_EXIT_M before
        # the exit (user rule 2026-10-03). Later obstacles are bypassed from
        # the shifted reference on either side.
        to_exit_ref = (self.tunnel.end - (np.array([aligned["x"], aligned["y"]]) -
                       self.tunnel.origin) @ self.tunnel.axis) if hasattr(self.tunnel, "end") else 0.
        hold = float(getattr(self, "_tunnel_hold_offset", 0.))
        if error or to_exit_ref <= TUNNEL_REJOIN_BEFORE_EXIT_M:
            hold = self._tunnel_hold_offset = 0.
        original_dense = dense
        if abs(hold) > 1e-3 and len(dense) > 2:
            tangent_r = np.gradient(dense, axis=0)
            tangent_r /= np.maximum(np.linalg.norm(tangent_r, axis=1, keepdims=True), 1e-9)
            dense = dense + np.column_stack((-tangent_r[:, 1], tangent_r[:, 0])) * hold
        # Both the planner and publisher see the same actual origin and join.
        from gui import build_forced_bypass, MAX_CONTROL_CURVATURE
        route_path, _, _ = build_forced_bypass(dense, 0., offset_m=0., hold_m=0.,
            max_curvature_1pm=MAX_CONTROL_CURVATURE, transitions=(26., 22., 18., 14., 12., 10., 8., 6.))
        if route_path is None:
            route_path = dense
            error = error or "tunnel route join infeasible"
        route_path = np.asarray(route_path)[:, :2]
        exit_zone = bool(not error and hasattr(self.tunnel, "end") and to_exit_ref <= TUNNEL_EXIT_ZONE_M)
        def exit_wall_clear(candidate, horizon):
            return self.tunnel.path_clearance(candidate, aligned, horizon, margin=TUNNEL_EXIT_WALL_MARGIN_M)
        exit_transitions, centre_limit = TUNNEL_JOIN_TRANSITIONS, math.inf
        if exit_zone and len(dense) > 2:
            # Exit: the gentlest join that stays off the kerbs (route centre at the exit).
            # Run 202: the route itself bends toward the right kerb at the portal, so with
            # the 1.0 m margin every join "failed" ~31 m out; ranked by that distance the
            # 10-14 m joins (0.18 1/m) won: heading -33 deg, 4.4 -> 0.9 m in 10 m, 7 s stop
            # at the right wall. Judge a join against the route centre's own limit, and
            # put the gentle length for this offset and speed first.
            from lattice_planner import comfortable_join_m
            tangent0 = dense[1] - dense[0]
            tangent0 = tangent0 / max(float(np.linalg.norm(tangent0)), 1e-9)
            d0 = -float(dense[0] @ np.array([-tangent0[1], tangent0[0]]))
            v_now = max(0., float(aligned.get("speed_kmh", 0.))) / 3.6
            gentle = max(comfortable_join_m(d0, v_now),
                         1.875 * abs(d0) / math.tan(math.radians(TUNNEL_EXIT_RETURN_MAX_DEG)))
            exit_transitions = tuple(sorted(set(TUNNEL_JOIN_TRANSITIONS) |
                                            {round(min(gentle, TUNNEL_EXIT_RETURN_MAX_M), 1)}, reverse=True))
            centre_limit = exit_wall_clear(dense, min(60., float(np.linalg.norm(np.diff(dense, axis=0), axis=1).sum())))
            for transition in exit_transitions:
                candidate, peak, _ = build_forced_bypass(dense, 0., offset_m=0., hold_m=0.,
                    max_curvature_1pm=MAX_CONTROL_CURVATURE, transitions=(transition,))
                if candidate is None or peak > MAX_CONTROL_CURVATURE:
                    continue
                candidate = np.asarray(candidate)[:, :2]
                horizon = min(60., float(np.linalg.norm(np.diff(candidate, axis=0), axis=1).sum()))
                if exit_wall_clear(candidate, horizon) >= min(horizon, centre_limit) - .3:
                    route_path = candidate
                    break
        if np.linalg.norm(route_path[0]) > .01:
            route_path = np.vstack(([0., 0.], route_path))
        # Tunnel rules permit the whole width between the physical walls.
        # Lane-change metadata remains relevant to the ordinary-road branch.
        corridor = (self.tunnel.corridor(aligned) if hasattr(self.tunnel, "corridor")
                    else self.lanes.corridor(aligned, dense))
        entry_side = (not hasattr(self.tunnel, "end") or
                      (np.array([aligned["x"], aligned["y"]])-self.tunnel.origin) @ self.tunnel.axis <
                      .5*(self.tunnel.start+self.tunnel.end))
        if getattr(self, '_transition_approach', False) and entry_side:
            # The portal approach only. Past the exit the same "approach" flag is set for
            # the frame that leaves the wall map; the exit road's lane bound then read 0.5 m
            # on the straight tunnel path (inside the GPS shadow, where lanes are not
            # scored): "live lidar collision on published path", cap 0 at 39 km/h
            # (follow-20261010T173230Z 02:42:22.4).
            from planner_transition import approach_corridor
            corridor = approach_corridor(self.tunnel, self.lanes, aligned, dense, body_shift_m=REAR_AXLE_SHIFT_M)
        if corridor is not None:
            # Every tunnel footprint is the real body on the rear-axle pose (ego_body),
            # also a lane corridor taken from the map (tunnels without a wall model).
            corridor.body_shift_m = REAR_AXLE_SHIFT_M
        adjacent_offsets = (self.tunnel.adjacent_offsets(aligned, dense)
                            if hasattr(self.tunnel, "adjacent_offsets")
                            else self.lanes.adjacent_offsets(aligned, dense))
        points = None if error else points_at_pose(match["obstacle_points"], cached["scan_odometry"], current_odometry)
        provider = getattr(getattr(self, 'path_publisher', None), '_safety_provider', None)
        if points is not None and provider is not None and hasattr(provider, 'tunnel_near_points'):
            retained = provider.tunnel_near_points(current_odometry)
            if len(retained):
                points = np.vstack((points, retained))
        objects = []
        moving = []
        self._tunnel_detector_anchor = None
        if points is not None:
            from tunnel_memory import TunnelNearFieldMemory
            from tunnel_tracks import TunnelTracks
            from lidar_corridor import without_moving_objects
            if not hasattr(self, "_tunnel_memory"):
                self._tunnel_memory = TunnelNearFieldMemory()
                self._tunnel_tracks = TunnelTracks()
            if self.tunnel.active(ego) and getattr(self, 'test_transition_status', None) is None:
                self._tunnel_memory.observe_scan(scan, cached['scan_odometry'],
                    getattr(self.lidar, 'mount', (1.05, 0., 1.35)))
            else:
                self._tunnel_memory.stop_observing()
            measured = lidar_objects(corridor_points(points, corridor), aligned)
            measured_odometry = objects_at_pose(measured, aligned, current_odometry)
            tracked_odometry = self._tunnel_tracks.update(measured_odometry, stamp)
            tracked = objects_at_pose(tracked_odometry, current_odometry, aligned)
            moving = [o for o in tracked if o[4] == "pedestrian"]
            # Wall aiding can change ESKF/map coordinates with no physical
            # motion. Store measured returns in uncorrected wheel/IMU odometry;
            # use grid cells only to count support, never to invent point centres.
            memory_moving = [o for o in tracked_odometry if o[4] == "pedestrian"]
            points = self._tunnel_memory.update(points, current_odometry, stamp, memory_moving)
            live_obstacles = getattr(self, '_live_tunnel_obstacles', None)
            if (live_obstacles is not None and
                    -.12 <= ego_stamp-live_obstacles['stamp'] <= .6 and
                    0. <= now-live_obstacles['received'] <= .6):
                # A low box can leave the scan while the live guard still
                # retains its measured face. Include it in candidate search,
                # rather than repeatedly issuing a path the guard must stop.
                retained = points_at_pose(live_obstacles['points'],
                                          live_obstacles['ego'], current_odometry)
                points = np.vstack((points, retained))
            static_points = without_moving_objects(points, moving, aligned)
            objects = lidar_objects(corridor_points(static_points, corridor), aligned) + moving
            cars = objects_at_pose(self.tunnel_vehicle_objects([], ego), ego, aligned)
            if cars:
                from lattice_planner import object_points
                from tunnel_tracks import DetectorMotion, measured_vehicle
                # Detector boxes drift with the visible part of a parked car: below
                # that drift a car moves only with a confirmed measured mover in its
                # box; otherwise it is bypassed as static (no moving conflict).
                if not hasattr(self, "_tunnel_detector_motion"):
                    self._tunnel_detector_motion = DetectorMotion()
                vehicle_stamp = float((getattr(self, "_tunnel_vehicle_status", None) or {}).get("stamp") or 0.)
                cars = [measured_vehicle(car, moving, persisted=self._tunnel_detector_motion.persisted(
                    car, vehicle_stamp)) for car in cars]
                # A standing car's box sits on its own measured faces (anchor_detector_box):
                # a refit in front of them must not close the road beside it.
                cars, self._tunnel_detector_anchor = standing_cars_on_returns(cars, points, aligned)
                # A partial visible bumper must not authorize passing through
                # the unobserved remainder of a recognized car's body.
                points = np.vstack((points, object_points(cars, aligned)))
                objects += cars
                moving += [car for car in cars if float(car[5]) > .5 and len(car) > 14 and car[14]]
        exit_previous_saved = getattr(self, "_tunnel_plan_saved", None)
        previous = continue_static_plan(exit_previous_saved, aligned, now)
        obstacle_tree = cKDTree(points) if points is not None and len(points) else None
        last_clearance_query = None
        def clearance_components(path, horizon=60., lane=corridor, checked_plan=None):
            if error or corridor is None:
                return dict(lane=0., wall=0., obstacle=0.)
            path = np.asarray(path)[:, :2]
            return self.tunnel_path_checks(path, aligned, points, lane, horizon,
                                           validation=checked_plan, tree=obstacle_tree)
        def clear(path, horizon=60.):
            nonlocal last_clearance_query
            distance = min(clearance_components(path, horizon).values())
            last_clearance_query = (np.asarray(path)[:, :2], float(horizon), distance)
            return distance
        def lane_ok(path, tolerance=1.5):
            local = np.asarray(path)[:, :2]
            length = float(np.linalg.norm(np.diff(local, axis=0), axis=1).sum())
            if last_clearance_query is not None and getattr(corridor, "wall_only", False):
                checked, horizon, distance = last_clearance_query
                if (len(local) <= len(checked) and horizon >= length
                        and np.shares_memory(local, checked)
                        and np.array_equal(local, checked[:len(local)])):
                    # Identical prefixes share every interior 25 cm pose and
                    # tangent. Only the final station's one-sided tangent can
                    # differ; that station lies inside the existing 30 cm
                    # endpoint tolerance and cannot change this predicate.
                    return min(distance, length) >= length-.3
            return clear(path, length) >= length - .3
        transition_seed = getattr(self, '_transition_seed', None)
        handoff_reused = False
        handoff_plan = None
        if transition_seed is not None and not error:
            local_seed = (np.asarray(transition_seed['world_path'])-[ego['x'], ego['y']]) @ rotation(ego)
            self._tunnel_lattice_prev = dict(stamp=now, target=transition_seed['target'],
                world_path=local_seed @ rotation(aligned).T+[aligned['x'], aligned['y']])
            alternatives = [transition_seed, getattr(self, '_transition_alternative', None)]
            for seed in alternatives:
                if not seed or not 0. <= now-seed.get('stamp', -math.inf) <= 1.5:
                    continue
                local = (np.asarray(seed['world_path'])-[ego['x'], ego['y']]) @ rotation(ego)
                world = local @ rotation(aligned).T+[aligned['x'], aligned['y']]
                continued = continue_static_plan(dict(stamp=now, world_path=world, plan={}), aligned, now)
                if continued is None:
                    continue
                candidate = RosPathPublisher.resample(continued['path'])
                horizon = min(60., float(np.linalg.norm(np.diff(candidate, axis=0), axis=1).sum()))
                margins = dict(generator='lattice',
                    side_margin_m=float((seed.get('validation') or {}).get('side_margin_m', .25)),
                    end_margin_m=float((seed.get('validation') or {}).get('end_margin_m', .25)))
                checked = clearance_components(candidate, horizon, checked_plan=margins)
                from gui import path_curvatures, zone_speed_kmh
                peak = float(np.max(path_curvatures(candidate)))
                required = max(12., abs(float(ego.get('speed_kmh', 0.)))/3.6*1.5+4.)
                if horizon < required or min(checked.values()) < horizon-.3 or peak > MAX_CONTROL_CURVATURE:
                    continue
                target = float(seed.get('target', 0.))
                cruise = zone_speed_kmh(self.speed_zones, int(planning_status['path']['nearest_idx']))[0]/3.6
                handoff_plan = dict(path=candidate, kind='bypass' if abs(target) > .01 else 'rejoin',
                    direction=float(np.sign(target)) or 1., offset_m=abs(target), transition_m=30.,
                    hold_m=0., hold_until_m=horizon, safe_horizon_m=horizon,
                    clear_distance_m=min(checked.values()), curvature=peak, blocked=False,
                    speed_cap_mps=min(cruise, math.sqrt(TUNNEL_BYPASS_ALAT_MPS2/max(peak, 1e-3))),
                    steering_rate_cap_mps=cruise, steering_gradient_radpm=0., reused=True,
                    selection_reason='freshly revalidated transition path', **margins)
                handoff_reused = True
                break
            self._transition_seed = None
            self._transition_alternative = None
        # Background entry preview: a shallow copy without the underscore state.
        preview = getattr(self, "test_transition_status", None) is not None
        model_hazards, v6_now, v6_why, route_state = [], None, None, None
        self._tunnel_model_log = None
        if handoff_plan is not None:
            plan, hazards = handoff_plan, []
        elif TUNNEL_LATTICE and not error:
            plan, hazards = self.tunnel_lattice_plan(dense, aligned, points, objects, corridor, now)
        elif TUNNEL_MODEL and not error:
            plan, hazards, model_hazards, v6_now, v6_why, route_state = self.tunnel_model_static(
                dense, aligned, objects, lane_ok, clear, adjacent_offsets, previous,
                int(planning_status["path"]["nearest_idx"]), preview, exit_zone)
        else:
            plan, hazards = (plan_static_bypass(dense, aligned, objects, lane_ok,
                previous["direction"] if previous else 0., previous,
                adjacent_offsets, clear, first_complete=True, full_width=True,
                deadline=started+.09, body_shift_m=REAR_AXLE_SHIFT_M) if not error else (None, []))
        # Exit zone: keep the route centre unless the centre itself is obstructed.
        # (Run 188: "lidar checked bypass" +1.0 -> +2.5 m with no hazard, 4 m before
        # the exit, triggered by portal returns past the exit; the car ended at the
        # right kerb.) Same raw points and lattice margins, plus the tunnel walls.
        exit_centre = False
        if exit_zone and plan is not None and not plan.get("blocked") and not hazards and points is not None:
            from lattice_planner import SIDE_MARGIN_M, END_MARGIN_M, COMFORT_SIDE_M
            centre_length = float(np.linalg.norm(np.diff(route_path, axis=0), axis=1).sum())
            horizon_c = min(max(float(plan.get("safe_horizon_m") or 0.), 40.), centre_length)
            # Returns inside the tunnel (a drum before the exit) keep the lattice's
            # comfortable pass; returns past the exit (portal, roadside) only need
            # the ordinary side margin.
            world_points = np.asarray(points)[:, :2] @ rotation(aligned).T + [aligned["x"], aligned["y"]]
            beyond = (world_points - self.tunnel.origin) @ self.tunnel.axis > self.tunnel.end
            centre_clear = min(self.tunnel.path_clearance(route_path, aligned, horizon_c),
                               swept_clearance(route_path, np.asarray(points)[~beyond, :2], horizon_c,
                                               END_MARGIN_M, lateral_margin=COMFORT_SIDE_M,
                                               body_shift_m=REAR_AXLE_SHIFT_M),
                               swept_clearance(route_path, np.asarray(points)[beyond, :2], horizon_c,
                                               END_MARGIN_M, lateral_margin=SIDE_MARGIN_M,
                                               body_shift_m=REAR_AXLE_SHIFT_M))
            if centre_clear >= horizon_c - .3:
                plan = previous = None
                self._tunnel_plan_saved = self._static_plan_saved = self._tunnel_lattice_prev = None
                exit_centre = True
        exit_rejoin_info = dict(active=False)
        exit_reference_world = dense @ rotation(aligned).T + [aligned['x'], aligned['y']]
        if exit_zone and not error and points is not None:
            active_exit_path = (plan['path'] if plan and not plan.get('blocked') else
                                previous['path'] if previous is not None else route_path)
            return_plan, exit_rejoin_info = tunnel_exit_rejoin(
                dense, aligned, self.tunnel, objects, hazards, clearance_components, active_exit_path,
                wall_clearance=exit_wall_clear, centre_limit=centre_limit,
                previous_saved=exit_previous_saved, now=now)
            exit_rejoin_info['active'] = True
            if return_plan is not None:
                plan, previous = return_plan, None
                self._tunnel_plan_saved = self._static_plan_saved = self._tunnel_lattice_prev = None
                exit_centre = True
        if (not error and not hazards and to_exit_ref > TUNNEL_REJOIN_BEFORE_EXIT_M
                and (plan is None or plan.get("kind") == "rejoin") and len(original_dense) > 2):
            # Freeze the reference at the car's lateral position (bounded by
            # the walls via the same clearance check) instead of returning.
            tangent_o = np.gradient(original_dense, axis=0)
            tangent_o /= np.maximum(np.linalg.norm(tangent_o, axis=1, keepdims=True), 1e-9)
            normal_o = np.column_stack((-tangent_o[:, 1], tangent_o[:, 0]))
            car_offset = float(np.clip(-float(original_dense[0] @ normal_o[0]), -TUNNEL_HOLD_MAX_M, TUNNEL_HOLD_MAX_M))
            candidate_ref = original_dense + normal_o * car_offset
            horizon_ref = min(60., float(np.linalg.norm(np.diff(candidate_ref, axis=0), axis=1).sum()))
            if abs(car_offset - hold) > .2 and clear(candidate_ref, horizon_ref) >= horizon_ref - .3:
                self._tunnel_hold_offset = car_offset
        # A completed return must release its saved manoeuvre. Re-saving it every
        # cycle otherwise keeps BYPASS active indefinitely on an unobstructed road.
        rejoin_complete = False
        if plan and plan.get("kind") == "rejoin" and not hazards and len(dense) > 3:
            tangent = dense[min(4, len(dense)-1)] - dense[0]
            heading_error = abs(math.atan2(tangent[1], tangent[0]))
            if abs(dense[0, 1]) <= .25 and heading_error <= math.radians(3.):
                length = min(60., float(np.linalg.norm(np.diff(route_path, axis=0), axis=1).sum()))
                if length >= 20. and clear(route_path, length) >= length - .3:
                    plan = previous = None
                    self._tunnel_plan_saved = self._static_plan_saved = None
                    rejoin_complete = True
        blocked = bool(plan and plan.get("blocked"))
        to_exit = (self.tunnel.end - (np.array([aligned["x"], aligned["y"]])-
                    self.tunnel.origin) @ self.tunnel.axis) if hasattr(self.tunnel, "end") else math.inf
        recovery_corridor = None
        if not error and plan is None and previous is None and corridor is not None and 0. <= to_exit <= 15.:
            from lane_corridor import LaneCorridor
            recovery_corridor = LaneCorridor(corridor.route, corridor.left + .15,
                                             corridor.right - .15, zones=corridor.zones,
                                             body_shift_m=REAR_AXLE_SHIFT_M)
        if not error and (plan is None or blocked) and previous is None:
            # A long join avoids an unnecessary sharp return, but a narrowing
            # lane may require an earlier return. Validate each shorter option
            # against the complete body, walls and raw returns before selecting.
            best_join = None
            for transition in (exit_transitions if exit_zone else (26., 22., 18., 14., 12., 10., 8., 6.)):
                candidate, peak, _ = build_forced_bypass(dense, 0., offset_m=0., hold_m=0.,
                    max_curvature_1pm=MAX_CONTROL_CURVATURE, transitions=(transition,))
                if candidate is None or peak > MAX_CONTROL_CURVATURE:
                    continue
                horizon = min(60., float(np.linalg.norm(np.diff(candidate[:, :2], axis=0), axis=1).sum()))
                candidate_clear = clear(candidate, horizon)
                if exit_zone:
                    candidate_clear = min(candidate_clear, exit_wall_clear(candidate[:, :2], horizon))
                if exit_zone:
                    # Up to the centre's own limit every join is equally clear: prefer the gentle one.
                    candidate_clear = min(candidate_clear, centre_limit)
                rank = (candidate_clear, -peak)
                if best_join is None or rank > best_join[0]:
                    best_join = rank, candidate[:, :2]
                if candidate_clear >= min(horizon, centre_limit) - .3:
                    best_join = rank, candidate[:, :2]
                    break
                if recovery_corridor is not None and candidate_clear < 20.:
                    recovery_clear = min(clearance_components(candidate, lane=recovery_corridor).values())
                    if exit_zone:
                        recovery_clear = min(recovery_clear, exit_wall_clear(candidate[:, :2], horizon))
                    if recovery_clear >= 20.:
                        best_join = rank, candidate[:, :2]
                        break
            if best_join is not None:
                route_path = best_join[1]
        path = plan["path"][:, :2] if plan and not blocked else previous["path"][:, :2] if previous else route_path
        if np.linalg.norm(path[0]) > .01:
            path = np.vstack(([0., 0.], path))
        # This transport and consumer exist only in the tunnel branch. Geometry
        # is expressed through the wall-matched pose; final control retains the
        # established relative-pose publication below.
        model_info = dict(enabled=False, accepted=False, source="lidar_fallback",
                          reason="tunnel model guidance unavailable")
        model_raw, model_probabilities = np.empty((0, 2)), None
        no_go_info = dict(active=False, valid=False, reason="topic unavailable")
        no_go_pub = getattr(self, "tunnel_no_go_publisher", None)
        no_go_rx = getattr(self, "tunnel_no_go_receiver", None)
        if no_go_pub is not None and no_go_rx is not None:
            walls = []
            if all(hasattr(self.tunnel, key) for key in ("origin", "axis", "normal", "start", "end", "left", "right")):
                for offset in (self.tunnel.left, self.tunnel.right):
                    world = self.tunnel.origin + offset*self.tunnel.normal + np.asarray(
                        [self.tunnel.start, self.tunnel.end])[:, None]*self.tunnel.axis
                    walls.append((world-[aligned["x"], aligned["y"]]) @ rotation(aligned))
            record = no_go_pub.update(points if points is not None else [], walls, aligned, stamp,
                                      True, valid=not bool(error))
            # Validate this frame's own record directly. Waiting for the ROS
            # loopback always read the previous frame's record, older than the
            # 0.25 s limit at 130 ms per frame, so the model path was never
            # used in the tunnel (2026-10-03: "occupancy source stale" in
            # every tunnel frame).
            held = getattr(no_go_rx, "record", None)
            if record is not None and not (held and held.get("sequence") == record.get("sequence")):
                try:
                    wire = json.dumps(record, separators=(",", ":"), allow_nan=False)
                except (TypeError, ValueError):
                    wire = "{invalid"
                no_go_rx.ingest(wire)
            constraints = no_go_rx.snapshot(aligned)
            no_go_info = dict(active=constraints.active, valid=constraints.valid,
                reason=constraints.reason, source_stamp=constraints.source_stamp,
                topic="/perception/no_go_areas")
            if (not error and TUNNEL_MODEL and not preview and plan is None and not blocked
                    and previous is None and getattr(self, "tunnel_model_policy", None) is not None):
                # v6 path (model_avoidance.v6_tunnel_candidate): only with no static plan
                # running, moved into the current body frame by odometry, then the same
                # ordering / curvature / steering / no-go / LiDAR checks as below.
                import model_avoidance
                from tunnel_model_guidance import checked_model_path
                v_model = max(20./3.6, float(ego.get("speed_kmh", 0.))/3.6)
                required = max(8., v_model*.7+v_model*v_model/5.+2.)
                def model_clear(candidate, horizon):
                    observed = cached["coverage"].clear_distance(
                        points_at_pose(candidate, current_odometry, cached["scan_odometry"]), horizon)
                    return min(clear(candidate, horizon), observed)
                path_now, path_why = None, None
                if v6_now is not None:
                    receiver = self.ego_receiver
                    with receiver.lock:
                        samples = list(receiver.samples)
                    path_now, path_why = model_avoidance.v6_path_now(
                        v6_now, samples, self.tunnel_odometry, current_odometry, points_at_pose)
                candidate, model_info = model_avoidance.v6_tunnel_candidate(
                    v6_now, v6_why, self.tunnel_model_policy, path_now, path_why, path, aligned,
                    constraints, model_clear, required, v_model, model_hazards, exit_zone,
                    checked_model_path)
                model_raw = model_info.pop("_raw_path", np.empty((0, 2)))
                if v6_now is not None:
                    v6_probabilities = v6_now.get("probabilities") or {}
                    model_probabilities = np.array([float(v6_probabilities.get(a, 0.)) for a in self.actions])
                if candidate is not None:
                    path = candidate
                    blocked = False
                    peak = float(path[np.argmax(np.abs(path[:, 1])), 1])
                    plan = dict(path=path, generator="v6", blocked=False, kind="model",
                        direction=float(np.sign(peak)) or 1., offset_m=abs(peak), transition_m=30.,
                        hold_m=0., hold_until_m=required, safe_horizon_m=required,
                        clear_distance_m=model_info["clear_distance_m"],
                        curvature=model_info["curvature_1pm"],
                        speed_cap_mps=model_info["speed_cap_mps"],
                        steering_rate_cap_mps=model_info["speed_cap_mps"],
                        steering_gradient_radpm=model_info["steering_gradient_radpm"])
            elif not error and not TUNNEL_LATTICE and not TUNNEL_MODEL:
                # Lattice mode: the model proposal never fitted the 0.16 s
                # budget on the car (live 2026-10-03) and only slowed frames.
                from tunnel_model_guidance import propose
                v_model = max(20./3.6, float(ego.get("speed_kmh", 0.))/3.6)
                required = max(8., v_model*.7+v_model*v_model/5.+2.,
                    float((plan or {}).get("safe_horizon_m", 0.)),
                    float((plan or {}).get("hold_until_m", 0.)))
                def model_clear(candidate, horizon):
                    observed = cached["coverage"].clear_distance(
                        points_at_pose(candidate, current_odometry, cached["scan_odometry"]), horizon)
                    return min(clear(candidate, horizon), observed)
                candidate, model_info = propose(self, aligned, path, constraints, model_clear,
                    required, v_model, deadline=started+.16)
                model_raw = model_info.pop("_raw_path", np.empty((0, 2)))
                model_probabilities = model_info.pop("_probabilities", None)
                if candidate is not None:
                    path = candidate
                    blocked = False
                    peak = float(path[np.argmax(np.abs(path[:, 1])), 1])
                    plan = dict(plan or {}, path=path, generator="model", blocked=False,
                        kind=(plan or {}).get("kind", "model"),
                        direction=(plan or {}).get("direction", float(np.sign(peak))),
                        offset_m=(plan or {}).get("offset_m", abs(peak)),
                        transition_m=(plan or {}).get("transition_m", 30.),
                        hold_m=(plan or {}).get("hold_m", 0.),
                        hold_until_m=max(required, float((plan or {}).get("hold_until_m", 0.))),
                        safe_horizon_m=required, clear_distance_m=model_info["clear_distance_m"],
                        curvature=model_info["curvature_1pm"],
                        speed_cap_mps=model_info["speed_cap_mps"],
                        steering_rate_cap_mps=model_info["speed_cap_mps"],
                        steering_gradient_radpm=model_info["steering_gradient_radpm"])
        from gui import zone_speed_kmh
        # Canonical publication geometry is finalized before every safety check.
        # The publisher will not rotate, prepend or resample this checked path.
        path = (np.asarray(path)[:, :2] if plan and plan.get('exit_rejoin')
                else RosPathPublisher.resample(path))
        zone_kmh, zone = zone_speed_kmh(self.speed_zones, int(planning_status["path"]["nearest_idx"]))
        self.zone_kmh, self.zone = zone_kmh, zone
        # A hazard already beside the car (front <= 0.5 m: the parked car being passed,
        # added by the planner as a rear-footprint hazard) is no distance to approach;
        # it made the blocked-frame cap 8 km/h at full speed (follow-20261010T162527Z).
        approach = min((h["front"] for h in hazards if float(h["front"]) > .5), default=None) if blocked else None
        if plan and not blocked and plan.get("generator") == "portable":
            # The portable plan's own cap brakes at 2.5 m/s^2 inside its check horizon
            # (35 m whenever no hazard is active: 11.2 m/s = 40 km/h, every "rejoin"
            # frame of follow-20261010T151125Z), not inside a measured clearance. The
            # final path's walls, returns, observed floor and curvature below own it.
            cap = zone_kmh / 3.6
        else:
            cap = static_speed_cap_mps(zone_kmh / 3.6, plan if not blocked else None, approach)
        from gui import path_curvatures, BYPASS_ALAT_MPS2
        curvature = float(np.max(path_curvatures(path)))
        # Below ~5 km/h the lattice may use its full-lock escape candidates
        # (LOW_SPEED_MAX_CURVATURE); judging them by the cruise limit threw the
        # tunnel into the no-bypass fallback, parked behind a drum (auto run 57).
        from lattice_planner import LOW_SPEED_MAX_CURVATURE
        limit = (max(MAX_CONTROL_CURVATURE, LOW_SPEED_MAX_CURVATURE)
                 if float(aligned.get("speed_kmh", 0.)) < 5.4 else MAX_CONTROL_CURVATURE)
        if not math.isfinite(curvature) or curvature > limit * 1.15:   # small overshoot != infeasible
            error = error or "tunnel route join infeasible"
        # Tunnel: the bypass curve is the only limit once a path exists (user
        # 2026-10-05); 1.7 m/s^2 held 15-20 km/h through every held bypass.
        # Each curvature limits the speed where it is (curve_speed_cap).
        curve_cap, curve_station = curve_speed_cap(path, abs(float(ego.get('speed_kmh', 0.)))/3.6)
        cap = min(cap, curve_cap)
        preparation_cap = exit_rejoin_info.get('preparation_cap_kmh')
        if exit_zone and preparation_cap is not None:
            cap = min(cap, max(0., float(preparation_cap))/3.6)
        if model_info.get("accepted"):
            cap = min(cap, model_info["speed_cap_mps"])
        checked_plan = plan if plan and not blocked else previous
        if checked_plan and checked_plan.get("tight_pass"):
            # Retain the creep cap even for a zero-offset path or a saved plan.
            cap = min(cap, TUNNEL_TIGHT_PASS_KMH / 3.6)
        components = clearance_components(path, checked_plan=checked_plan)
        geometric_distance = min(components.values())
        boundary_recovery = False
        # At the tunnel exit the map abruptly removes the adjacent lane. The
        # recorded car was still wholly inside its own lane, but the extra 20 cm
        # lane-marking margin rejected a gentle return. For that base return only,
        # retain 5 cm at the lane marking and the full 25 cm at walls/obstacles.
        # This never authorizes a new bypass or relaxes a no-go polygon.
        if (not error and plan is None and previous is None and corridor is not None and
                0. <= to_exit <= 15. and geometric_distance < 20.):
            recovery_components = clearance_components(path, lane=recovery_corridor)
            recovery_distance = min(recovery_components.values())
            if recovery_distance >= 20.:
                components = recovery_components
                geometric_distance = recovery_distance
                boundary_recovery = True
                cap = min(cap, 12. / 3.6)
        # Coverage is not asked beyond the kerb faces: a line beside the kerb read 1 m
        # (follow-20261010T173230Z 02:41:45, kerb_room / tunnel_visibility).
        side_room = kerb_room(self.tunnel, match.get("aligned_ego")) if not error else None
        observed_distance = (cached["coverage"].clear_distance(
            points_at_pose(path, current_odometry, cached["scan_odometry"]), 60.,
            exclude_initial_footprint=True, lateral_room=side_room) if not error else 0.)
        coverage_escape = None
        physical_tunnel = not preview and self.tunnel.active(ego)
        coverage_required = (coverage_search_distance(abs(float(ego.get('speed_kmh', 0.)))/3.6,
                                                     decision_lag)
                             if physical_tunnel else TUNNEL_COVERAGE_ESCAPE_M)
        if (TUNNEL_SHAPE and TUNNEL_MODEL and not error and not exit_zone and plan and not blocked
                and plan.get("generator") == "portable" and plan.get("kind") in ("bypass", "rejoin")
                and observed_distance < coverage_required and geometric_distance >= 20.):
            # Walls and returns are clear but the line itself leaves the floor unseen
            # (beside a kerb): take a nearby line the coverage confirms, never crawl on it.
            def observed_of(candidate):
                return cached["coverage"].clear_distance(
                    points_at_pose(candidate, current_odometry, cached["scan_odometry"]), 60.,
                    exclude_initial_footprint=True, lateral_room=side_room)
            escaped, coverage_escape = self.tunnel_coverage_escape(
                plan, hazards, dense, aligned, objects, clear, observed_of, observed_distance,
                required_observed_m=coverage_required if physical_tunnel else None)
            if escaped is not None:
                plan = checked_plan = escaped
                path = RosPathPublisher.resample(np.asarray(escaped["path"], dtype=float)[:, :2])
                if np.linalg.norm(path[0]) > .01:
                    path = RosPathPublisher.resample(np.vstack(([0., 0.], path)))
                curvature = float(np.max(path_curvatures(path)))
                curve_cap, curve_station = curve_speed_cap(path, abs(float(ego.get('speed_kmh', 0.)))/3.6)
                cap = min(zone_kmh / 3.6, curve_cap)
                components = clearance_components(path, checked_plan=checked_plan)
                geometric_distance = min(components.values())
                observed_distance = observed_of(path)
        # Short coverage is not an obstacle: it lowers the speed only. At the exit
        # the open road beyond reads short; it must not stop the car there
        # (runs 185/187/188 braked hard 3-4 m before the exit).
        effective_observed = (max(observed_distance, TUNNEL_EXIT_OBSERVED_FLOOR_M)
                              if exit_zone else observed_distance)
        distance = min(geometric_distance, effective_observed)
        # Stops there follow walls/obstacles only (the floor alone is ~one braking distance).
        hard_distance = geometric_distance if exit_zone else distance
        # Speed must leave braking room within the whole checked footprint corridor.
        cap = min(cap, max(0., math.sqrt(1.75 ** 2 + 2. * TUNNEL_BRAKE_MPS2 * max(0., distance - 2.)) - 1.75))
        cap = min(cap, latency_speed_cap(distance, decision_lag, TUNNEL_BRAKE_MPS2))
        v = abs(float(ego.get('speed_kmh', 0.)))/3.6
        initial_emergency = bool(error or hard_distance <= max(2., v*.25+v*v/(2.*TUNNEL_BRAKE_MPS2)))
        source_frame = dict(aligned)
        self._live_tunnel_corridor = dict(ego=dict(aligned),
            corridor=recovery_corridor if boundary_recovery else corridor,
            wall_stats=dict(match.get('wall_stats') or {}))
        decision, points = self.checked_tunnel_plan(path, ego, aligned, scan, points, cap, distance,
            initial_emergency, dict(valid=not bool(error), pose_valid=not bool(error), blocked=blocked,
                handoff_reused=handoff_reused,
                generator=(checked_plan or {}).get('generator'),
                dynamic_recovery_ids=(checked_plan or {}).get('dynamic_recovery_ids', []),
                **components, observed=effective_observed,
                side_margin_m=float((checked_plan or {}).get('side_margin_m', .25)),
                end_margin_m=float((checked_plan or {}).get('end_margin_m', .25))),
            corridor=recovery_corridor if boundary_recovery else corridor)
        decision, lead_info, yielding = self.tunnel_traffic_checks(decision, objects, points, moving)
        decision, points, replacement = self.retry_tunnel_plan(decision, dense,
            recovery_corridor if boundary_recovery else corridor, source_frame,
            points, objects, moving, zone_kmh/3.6, decision_lag, exit_zone)
        if replacement is not None:
            plan, blocked, previous, hazards = replacement, False, None, []
            model_info.update(accepted=False, source='lidar_revalidation', reason='replanned after final path checks')
            lead_info = decision.validation.get('lead')
            yielding = decision.validation.get('moving_conflict')
            curvature = decision.validation['curvature_1pm']
            observed_distance = decision.validation['observed_raw']
        path, ego, aligned = decision.path, decision.control_ego, decision.validation_ego
        status = dict(status, ego=ego)
        v = abs(float(ego.get('speed_kmh', 0.)))/3.6
        distance = decision.forward_clear_m
        curvature = decision.validation.get('curvature_1pm', curvature)
        curvature = math.inf if curvature is None else curvature
        components = {key: decision.validation.get(key, value) for key, value in components.items()}
        geometric_distance = min(min(components.values()), decision.validation.get('latest_obstacle', math.inf))
        effective_observed = decision.validation.get('observed', effective_observed)
        hard_distance = geometric_distance if exit_zone else distance
        cap, emergency = decision.allowed_mps, decision.emergency
        elapsed = max(0., time.perf_counter()-started-decision.validation.get('pose_advanced_s', 0.))
        if elapsed > .25 and not error:
            # A late frame is still a checked plan; charge the travel since its
            # scan against the clearance instead of a stop pulse that restarts
            # the speed ramp (2026-10-03: stopped every ~2 s at 130 ms median).
            late_distance = max(0., distance - v * elapsed)
            cap = min(cap, max(0., math.sqrt(1.75 ** 2 + 2. * TUNNEL_BRAKE_MPS2 * max(0., late_distance - 2.)) - 1.75))
            emergency = emergency or max(0., hard_distance - v * elapsed) <= max(2., v * .25 + v * v / (2. * TUNNEL_BRAKE_MPS2))
            distance = late_distance
        # Crawl once "no safe bypass" has appeared near an obstacle, until it stays
        # clear for 3 s: at <=5 km/h the lattice's sharp low-speed paths make the
        # S around a slalom feasible. Flip-flopping found/blocked at 25 km/h ended
        # 3 m behind a parked K5 with no way out (auto run 58).
        mono = time.monotonic()
        if ((handoff_reused or decision.validation.get('replan_accepted')) and
                not blocked and not emergency and decision.validation.get('valid')):
            # A fully revalidated handoff owns the current path. Crawl latches
            # from the previous path cannot hold its speed down for one frame.
            self._tunnel_crawl_until = -1.
            self._tunnel_blocked_n = 0
        self._tunnel_blocked_n = getattr(self, "_tunnel_blocked_n", 0) + 1 if (blocked and distance < 20.) else 0
        if self._tunnel_blocked_n >= 2:          # one flickering frame must not cost a crawl
            # No re-acceleration until clear for 3 s: never faster than at the
            # last blocked frame (floor CRAWL_KMH so the creep logic can work).
            self._tunnel_hold_mps = max(CRAWL_KMH / 3.6, min(getattr(self, "_tunnel_hold_mps", 99.)
                                        if mono < getattr(self, "_tunnel_crawl_until", -1.) else 99., v))
            self._tunnel_crawl_until = mono + 3.
        # Release as soon as the bypass is checked clear on 2 frames: once a path
        # exists only the curve limits speed (user 2026-10-05; the 3 s hold made
        # 55 % of the slowdowns inside held tunnel bypasses).
        self._tunnel_clear_n = 0 if blocked else getattr(self, "_tunnel_clear_n", 0) + 1
        if self._tunnel_clear_n >= 2:
            self._tunnel_crawl_until = -1.
        if mono < getattr(self, "_tunnel_crawl_until", -1.):
            cap = min(cap, self._tunnel_hold_mps)
        if (emergency and not error and decision.validation.get('valid') and
                not (lead_info and lead_info.get('emergency')) and
                decision.validation.get('latest_obstacle', math.inf) > max(2., v*.25+v*v/(2.*TUNNEL_BRAKE_MPS2)) and
                not (yielding and yielding["kind"] == "stop") and
                effective_observed < geometric_distance and
                geometric_distance >= 3. and v < 1.5):
            # (No minimum observed distance: the VLP16's lowest beam meets the road
            # ~5 m out, so the ground just ahead of a stopped car is never observed
            # and coverage reads 0 m for good - auto run 65. Anything tall enough to
            # hit (>0.5 m) is still seen by the upper beams.)
            # Only LiDAR coverage is short (the bypass lane is hidden behind the
            # obstacle). Stopping never reveals it (auto run 55 waited 71 s):
            # creep to see past it; a 15 Hz scan stops this within ~0.3 m.
            emergency = False
            cap = min(max(cap, TUNNEL_PEEK_KMH / 3.6), TUNNEL_PEEK_KMH / 3.6)
        reason = error or ("tunnel no safe bypass" if blocked else
                           "tunnel model path checked by LiDAR" if model_info.get("accepted") else
                           "tunnel lidar checked rejoin" if plan and plan.get("kind") == "rejoin" else
                           "tunnel lidar checked bypass" if plan else
                           "tunnel lane boundary recovery" if boundary_recovery else
                           "tunnel exit centre route" if exit_centre else "tunnel clear route")
        if not error and match.get('relative_geometry'):
            reason += '; relative wall geometry; map aiding rejected'
        if plan and not blocked and plan.get("generator") != "v6":
            # (A v6 path is not a committed manoeuvre: saved, the static planner would
            # read its side as a bypass to return from.)
            world = path @ rotation(aligned).T + [aligned["x"], aligned["y"]]
            self._tunnel_plan_saved = dict(stamp=decision.checked_at, world_path=world, plan=plan)
            # Preserve the checked return when the next decision leaves the tunnel.
            control_world = path @ rotation(ego).T + [ego["x"], ego["y"]]
            self._static_plan_saved = dict(stamp=decision.checked_at, world_path=control_world, plan=plan)
            if getattr(self, '_tunnel_lattice_prev', None) is not None:
                self._tunnel_lattice_prev.update(stamp=decision.checked_at, world_path=world)
        elif previous is None:
            self._tunnel_plan_saved = None
        mode = ("TUNNEL MODEL" if model_info.get("accepted") else
                "TUNNEL BYPASS" if plan and not blocked else "TUNNEL")
        clearances = dict(components, observed=effective_observed)
        geometric_stop = geometric_distance <= max(2., v * .25 + v * v / (2. * TUNNEL_BRAKE_MPS2))
        limiting_source = (min(components, key=components.get)
                           if geometric_stop else min(clearances, key=clearances.get))
        blocked_reasons = dict(observed="tunnel lidar coverage insufficient",
            obstacle="tunnel obstacle on path", wall="tunnel wall clearance",
            lane="tunnel lane boundary")
        stop_reason = error or ("tunnel moving object conflict" if yielding and yielding["kind"] == "stop" else
                                decision.stop_reason or blocked_reasons[limiting_source] if emergency else None)
        # A final same-cycle retry must keep the exit preparation envelope too.
        if exit_zone and preparation_cap is not None:
            cap = min(cap, max(0., float(preparation_cap))/3.6)
        decision = decision.decided(allowed_mps=cap, emergency=emergency, forward_clear_m=distance,
            mode=mode, stop_reason=stop_reason,
            lead=getattr(self, '_lead_state', None), moving_conflict=yielding)
        self._tunnel_decision_plan = decision
        if TUNNEL_MODEL and not preview:
            self.observe_tunnel_lane_hold(plan, route_state,
                bool(decision.validation.get('valid')) and not emergency and not error)
        post = dict(mode=mode, reason=reason, validated_static=True,
            forward_clear_m=distance if not error and not blocked and not emergency else 0.,
            raw_peak_m=float(np.max(np.abs(model_raw[:, 1]))) if len(model_raw) else 0.,
            raw_end_m=float(model_raw[-1, 1]) if len(model_raw) else 0.,
            applied_peak_m=float(plan["direction"] * plan["offset_m"]) if plan and not blocked else 0.,
            motion_state="STOP" if emergency else "DRIVE", stop_reason=stop_reason,
            decision=decision.summary(),
            avoidance_evidence="tunnel_model_no_go_checked" if model_info.get("accepted") else "tunnel_lidar" if plan else "none")
        probabilities = model_probabilities if model_probabilities is not None else np.zeros(len(self.actions), dtype=float)
        self.last_result = dict(path=path, raw_path=model_raw, probabilities=probabilities,
            concept_values=np.zeros(len(getattr(self, "concepts", ()))), scene_matches=[],
            trajectory=trajectory_diagnostics(model_raw, np.asarray(display_route)),
            hybrid_trajectory=trajectory_diagnostics(path, np.asarray(display_route)),
            postprocess=post, local_route=np.asarray(display_route), model_route=np.asarray(model_route),
            latency_ms=model_info.get("latency_ms", 0.), status=status)
        self.last_hazard = None
        self.last_yield = yielding
        self._n_obj = len(objects)
        self._tunnel_view_points, self._tunnel_view_pose = points, aligned
        self._lidar_log = dict(n=len(points) if points is not None else None,
            age_s=scan.get("age_s"), path_block_m=distance,
            retained_near_field_n=getattr(getattr(self, "_tunnel_memory", None), "retained_n", 0),
            observed_free_memory_cleared_total=getattr(getattr(self, "_tunnel_memory", None), "cleared_total", 0))
        degraded = None
        fallback_plan_info = None
        if self.path_publisher:
            self.path_publisher.zone_mps = cap
            self.path_publisher.emergency_stop = emergency
            # Map conversion used the matched pose; the controller still consumes ESKF.
            # Publish local geometry through that same ESKF pose to preserve relative error.
            # Age of the scan's data, not of its receipt: a source stamp 1 s old
            # or ahead of localization is not a usable scan.
            source_age = ego_stamp - stamp if stamp > 0. and ego_stamp > 0. else 999.
            if cached and not fresh_scan:
                source_age = max(source_age, now - cached["received"])
            usable = -.12 <= source_age
            # Fallback path: stay parallel to the tunnel at the current offset
            # (clamped off the walls) instead of joining the route. The route
            # join swung a car 3 m off-centre across the tunnel until it faced
            # the wall (live 2026-10-04, idx 3904-3927).
            # Run 222: the wall match failed twice in the middle of a bypass. Keep the
            # avoidance the last good decision was flying (its world path, re-joined
            # from the actual pose) instead of settling parallel at the half-way offset,
            # and keep the lattice's side commitment alive for when the match returns.
            inherited = None
            if error and str(error).startswith(self.TUNNEL_FALLBACK_ERRORS):
                saved = getattr(self, "_tunnel_last_good", None)
                cont = continue_static_plan(saved, ego, now) if saved else None
                if cont is not None and len(cont["path"]) >= 3 and float(
                        np.linalg.norm(np.diff(cont["path"], axis=0), axis=1).sum()) >= 15.:
                    inherited = np.asarray(cont["path"], dtype=float)[:, :2]
                    saved["stamp"] = now
                    if getattr(self, "_tunnel_lattice_prev", None) is not None:
                        self._tunnel_lattice_prev["stamp"] = now
            fallback_path = ((inherited if inherited is not None else
                              tunnel_parallel_path(dense, to_exit_m=to_exit_ref if 'to_exit_ref' in locals() else None))
                             if error else route_path)
            degraded = (self.tunnel_degraded_drive(error, ego, dict(scan, age_s=max(float(scan.get("age_s", 999.)), source_age)),
                                                   fallback_path) if error and usable else None)
            if degraded is not None:
                # Wall matching failed but ESKF odometry is fresh: keep going on
                # the route at reduced speed, braking only for raw LiDAR returns
                # inside the swept body (user rule: never sit in the tunnel).
                cap, distance_raw = degraded
                drive_path, fallback_reason = fallback_path, "tunnel odometry fallback: " + error
                raw_pts = self.fallback_obstacle_points(scan)
                if provider is not None and hasattr(provider, 'tunnel_near_points'):
                    fallback_odometry = self.tunnel_odometry.at_stamp(float(ego.get('stamp', 0.)))
                    if fallback_odometry is not None:
                        retained = provider.tunnel_near_points(fallback_odometry)
                        if len(retained):
                            raw_pts = np.vstack((raw_pts, retained))
                cars = self.tunnel_vehicle_objects([], ego)
                if cars:
                    from lattice_planner import object_points
                    # Standing boxes on their cars' measured faces, as in the matched branch.
                    cars, self._tunnel_detector_anchor = standing_cars_on_returns(
                        cars, None if raw_pts is None else np.asarray(raw_pts)[:, :2], ego)
                    car_points = object_points(cars, ego)
                    raw_pts = car_points if raw_pts is None else np.vstack((np.asarray(raw_pts)[:, :2], car_points))
                    distance_raw = min(distance_raw, float(swept_clearance(fallback_path, car_points, 40., lateral_margin=.3,
                                                                           body_shift_m=REAR_AXLE_SHIFT_M)))
                    fallback_reason += '; measured LiDAR cars %d' % len(cars)
                # Same near-field memory as the wall-matched branch: a low obstacle seen
                # before the match failed may now be below the lowest beam (read only:
                # raw returns include the walls, which must not enter the memory).
                memory = getattr(self, "_tunnel_memory", None)
                odometry = self.tunnel_odometry.at_stamp(ego_stamp) if memory is not None and ego_stamp > 0. else None
                if odometry is not None:
                    remembered = np.asarray(memory.update(np.empty((0, 2)), odometry, stamp, ()), dtype=float).reshape(-1, 2)
                    if len(remembered):
                        raw_pts = remembered if raw_pts is None else np.vstack((np.asarray(raw_pts)[:, :2], remembered))
                        distance_raw = min(distance_raw, float(swept_clearance(fallback_path, remembered, 40., lateral_margin=.3,
                                                                               body_shift_m=REAR_AXLE_SHIFT_M)))
                        fallback_reason += "; near-field memory %d" % len(remembered)
                if inherited is not None:
                    from gui import path_curvatures
                    fallback_reason += "; kept avoidance path"
                    curvature_inherited = float(np.max(path_curvatures(inherited[:80]))) if len(inherited) >= 3 else 0.
                    cap = min(cap, math.sqrt(TUNNEL_BYPASS_ALAT_MPS2 / max(curvature_inherited, 1e-3)))
                cap = min(cap, max(0., math.sqrt(1.75 ** 2 + 2. * TUNNEL_BRAKE_MPS2 * max(0., distance_raw - 2.)) - 1.75))
                cap = min(cap, latency_speed_cap(distance_raw, decision_lag, TUNNEL_BRAKE_MPS2))
                emergency = distance_raw <= max(2., v * .25 + v * v / (2. * TUNNEL_BRAKE_MPS2))
                # Camera-semantic objects too (user 2026-10-05): without the wall match
                # the raw LiDAR check missed a drum until 1.25 m at 17 km/h (auto run 158).
                sem_pts, sem_n = self.semantic_obstacle_points(ego)
                if sem_n:
                    raw_pts = sem_pts if raw_pts is None else np.vstack((np.asarray(raw_pts)[:, :2], sem_pts))
                    sem_clear = float(swept_clearance(fallback_path, sem_pts, 40., .25, body_shift_m=REAR_AXLE_SHIFT_M))
                    if sem_clear < distance_raw:
                        distance_raw = sem_clear
                        cap = min(cap, math.sqrt(2. * 2. * max(0., sem_clear - 3.)))
                        emergency = distance_raw <= max(2., v * .25 + v * v / (2. * TUNNEL_BRAKE_MPS2))
                    fallback_reason += "; camera-semantic objects %d" % sem_n
                    cap = min(cap, latency_speed_cap(distance_raw, decision_lag, TUNNEL_BRAKE_MPS2))
                provider = getattr(self.path_publisher, '_safety_provider', None)
                dynamic_recovery = (provider.recovery_inputs(ego, 'TUNNEL', requested_mps=TUNNEL_FALLBACK_KMH/3.6)
                                    if provider is not None else None)
                if (distance_raw < 25. or dynamic_recovery is not None) and raw_pts is not None:
                    # No wall match does not mean no bypass: plan around the raw LiDAR
                    # returns (walls included) instead of parking behind the obstacle
                    # for the rest of the run (auto run 87: 60 s in fallback).
                    fresh_walls = (cached and cached['stamp'] == stamp and -.12 <= source_age <= .25
                                   and 0. <= now-cached['received'] <= .25)
                    escape = self.fallback_lattice(fallback_path, ego, raw_pts,
                        walls=match.get('body_walls') if fresh_walls else None,
                        source_ego=cached.get('scan_ego') if cached else None,
                        to_exit_m=to_exit_ref)
                    if escape is not None:
                        drive_path = escape['path']
                        # The blocked reference path's zero cap belongs to
                        # that path. Recompute braking on the checked bypass.
                        travelled = v * max(0., float(scan.get('age_s', 0.)), source_age)
                        distance_raw = max(0., float(escape['clear_m'])-travelled)
                        cap = min(TUNNEL_FALLBACK_KMH/3.6, escape['speed_cap_mps'],
                                  max(0., math.sqrt(1.75**2+2.*TUNNEL_BRAKE_MPS2*max(0., distance_raw-2.))-1.75),
                                  latency_speed_cap(distance_raw, decision_lag, TUNNEL_BRAKE_MPS2))
                        emergency = distance_raw <= max(2., v*.25+v*v/(2.*TUNNEL_BRAKE_MPS2))
                        fallback_plan_info = dict(target_m=escape['target'], clear_m=distance_raw,
                                                  speed_cap_kmh=cap*3.6, reason=escape.get('reason'),
                                                  dynamic_recovery_ids=escape.get('dynamic_recovery_ids', []))
                        fallback_reason += "; raw-LiDAR bypass " + str(escape.get("reason"))
                drive_path = RosPathPublisher.resample(drive_path)
                distance_raw = min(distance_raw, float(swept_clearance(drive_path, raw_pts, 40., lateral_margin=.3,
                                                                       body_shift_m=REAR_AXLE_SHIFT_M)))
                emergency = distance_raw <= max(2., v*.25+v*v/(2.*TUNNEL_BRAKE_MPS2))
                source_frame = dict(ego)
                fresh_walls = (cached and cached['stamp'] == stamp and -.12 <= source_age <= .25
                               and 0. <= now-cached['received'] <= .25)
                fallback_corridor = measured_wall_corridor(fallback_path,
                    match.get('body_walls') if fresh_walls else None,
                    cached['scan_ego'] if cached else ego, ego)
                if fallback_corridor is None:
                    from lane_corridor import LaneCorridor
                    fallback_corridor = LaneCorridor(fallback_path, np.full(len(fallback_path), 4.),
                                                    np.full(len(fallback_path), -4.), body_shift_m=REAR_AXLE_SHIFT_M)
                else:
                    fallback_corridor.measured_walls = True
                # The live checker must inherit this fallback's measured walls,
                # not the last wall-matched branch's map corridor and pose.
                self._live_tunnel_corridor = dict(ego=dict(ego), corridor=fallback_corridor)
                fallback_decision, raw_pts = self.checked_tunnel_plan(drive_path, ego, ego, scan,
                    raw_pts, cap, distance_raw, emergency,
                    dict(valid=True, pose_valid=float(ego.get('age_s', 999.)) <= .25, blocked=False,
                         obstacle=distance_raw, side_margin_m=.3, end_margin_m=.25,
                         dynamic_recovery_ids=(fallback_plan_info or {}).get('dynamic_recovery_ids', []),
                         mapped_walls=False,
                         wall_source='raw LiDAR and bounded fallback corridor'),
                    stop_reason='tunnel raw lidar obstacle' if emergency else None,
                    corridor=fallback_corridor)
                fallback_moving = [car for car in cars if len(car) > 14 and car[14] and car[5] > .5]
                fallback_decision, follow, _ = self.tunnel_traffic_checks(fallback_decision, cars, raw_pts, fallback_moving)
                fallback_decision, raw_pts, replacement = self.retry_tunnel_plan(fallback_decision,
                    fallback_path, fallback_corridor, source_frame, raw_pts, cars, fallback_moving,
                    TUNNEL_FALLBACK_KMH/3.6, decision_lag, mapped_walls=False)
                if replacement is not None:
                    fallback_reason += '; replanned after final path checks'
                    fallback_plan_info = dict(target_m=replacement['target'],
                        clear_m=fallback_decision.forward_clear_m,
                        speed_cap_kmh=fallback_decision.allowed_mps*3.6, reason=replacement['reason'])
                ego = fallback_decision.control_ego
                v = abs(float(ego.get('speed_kmh', 0.)))/3.6
                cap, distance_raw, emergency = (fallback_decision.allowed_mps,
                    fallback_decision.forward_clear_m, fallback_decision.emergency)
                # A bypass the car cannot steer (0.31 1/m at a skewed tunnel mouth) kept
                # it crawling in place for 50 s (auto run 111): no progress for
                # FALLBACK_STALL_S counts as blocked, so the reverse straightens us out.
                if v > .8 or not distance_raw < 25.:
                    self._fallback_slow_since = None
                elif getattr(self, "_fallback_slow_since", None) is None:
                    self._fallback_slow_since = time.monotonic()
                stalled = (getattr(self, "_fallback_slow_since", None) is not None
                           and time.monotonic() - self._fallback_slow_since > FALLBACK_STALL_S)
                # A checked steerable forward plan also clears a previous stall;
                # an old timer must not turn a newly opened route into a reverse.
                fallback_decision = fallback_decision.decided(allowed_mps=cap, emergency=emergency,
                    stop_reason=fallback_decision.stop_reason or ('tunnel lead conflict' if emergency else None),
                    lead=getattr(self, '_lead_state', None))
                if fallback_decision.forward_ready(ego):
                    self._fallback_slow_since = None
                    self._reverse_wait_since = None
                    self._reverse_veto = None
                    stalled = False
                self._tunnel_decision_plan = decision = fallback_decision
                aligned, path = fallback_decision.validation_ego, fallback_decision.path
                curvature = fallback_decision.validation.get('curvature_1pm', curvature)
                curvature = math.inf if curvature is None else curvature
                yielding = fallback_decision.validation.get('moving_conflict')
                self.last_yield = yielding
                self._tunnel_view_points, self._tunnel_view_pose = raw_pts, aligned
                if (emergency or stalled) and hasattr(self, "reverse_escape_check"):
                    self.reverse_escape_check(ego, True, distance_raw, raw_pts, time.monotonic(),
                                              decision_plan=fallback_decision, swing_check=True)
                self.path_publisher.zone_mps = cap
                self.path_publisher.emergency_stop = emergency
                post = dict(post, mode='TUNNEL', reason=fallback_reason,
                    forward_clear_m=distance_raw if not emergency else 0.,
                    motion_state='STOP' if emergency else 'DRIVE',
                    stop_reason=fallback_decision.stop_reason, decision=fallback_decision.summary())
                self.last_result.update(path=fallback_decision.path, postprocess=post, status=dict(status, ego=ego),
                    hybrid_trajectory=trajectory_diagnostics(fallback_decision.path, np.asarray(display_route)))
                published = self.path_publisher.publish(ego, fallback_decision.path, fallback_decision.path, post,
                                                        decision_plan=fallback_decision)
                if published and getattr(self.path_publisher, '_safety_provider', None) is not None:
                    active = getattr(self.path_publisher, '_active_decision_plan', None)
                    if active is not None:
                        fallback_decision = decision = self._tunnel_decision_plan = active
                        cap, distance_raw, emergency = active.allowed_mps, active.forward_clear_m, active.emergency
                        post.update(decision=active.summary(), stop_reason=active.stop_reason,
                                    motion_state='STOP' if emergency else 'DRIVE')
                record_override = dict(reason=fallback_reason,
                                       stop_reason=fallback_decision.stop_reason,
                                       emergency_stop=emergency, planner_cap_kmh=cap * 3.6)
                distance = distance_raw
                clearances = {key: fallback_decision.validation[key] for key in ('wall', 'lane', 'obstacle')
                              if key in fallback_decision.validation}
                geometric_distance = min(clearances.values())
                limiting_source = min(clearances, key=clearances.get)
                self._lidar_log.update(path_block_m=distance_raw, age_s=fallback_decision.sensor_age_s,
                                       n=len(raw_pts) if raw_pts is not None else None)
            elif error:
                self.path_publisher.publish_stop(error)
            else:
                # Parked against a blocker with no bypass (runs 52/57/58): back out.
                if hasattr(self, "reverse_escape_check"):
                    # Judge by real obstacle/wall distance, not LiDAR coverage.
                    self.reverse_escape_check(ego, bool(blocked or (emergency and geometric_stop)),
                                              geometric_distance,
                                              static_points if 'static_points' in locals() else None,
                                              time.monotonic(), decision_plan=decision, swing_check=True)
                published = self.path_publisher.publish(ego, decision.path, decision.path, post, decision_plan=decision)
                if published and getattr(self.path_publisher, '_safety_provider', None) is not None:
                    active = getattr(self.path_publisher, '_active_decision_plan', None)
                    if active is not None:
                        decision = self._tunnel_decision_plan = active
                        post['decision'] = decision.summary()
                # The avoidance a later fallback frame keeps flying (control frame).
                if decision.validation.get('valid') and not decision.emergency and not blocked and decision.allowed_mps > 0.:
                    self._tunnel_last_good = dict(stamp=decision.checked_at, plan={}, decision=decision.summary(),
                        world_path=decision.path @ rotation(ego).T+[ego['x'], ego['y']])
            self.path_publisher.publish_camera_rates(self.receiver.rates())
            self.path_publisher.publish_corridor(ego, corridor)
            stop_reason = decision.stop_reason
            emergency = decision.emergency
            post.update(stop_reason=stop_reason, motion_state='STOP' if emergency else 'DRIVE')
        if exit_rejoin_info.get('active'):
            measured_exit_route = (exit_reference_world-[aligned['x'], aligned['y']]) @ rotation(aligned)
            exit_rejoin_info['measured'] = tunnel_route_alignment(measured_exit_route)
            exit_rejoin_info['to_exit_m'] = float(self.tunnel.end-
                (np.array([aligned['x'], aligned['y']])-self.tunnel.origin) @ self.tunnel.axis)
            rejoin_complete = bool(exit_rejoin_info['measured']['complete'] and not emergency and not error)
            if rejoin_complete:
                self._tunnel_plan_saved = self._static_plan_saved = self._tunnel_lattice_prev = None
        record = dict(t=time.time(), ego=[ego["x"], ego["y"]], speed_kmh=ego.get("speed_kmh", 0.),
            decision=decision.summary(),
            transition=dict(getattr(getattr(self, '_planner_transition', None), 'info', {})),
            action=model_info.get("action", "LIDAR"), p=model_info.get("probability"), mode=mode, reason=reason, stop_reason=stop_reason,
            motion_state=post["motion_state"], target_kmh=zone_kmh, zone=zone,
            applied_peak_m=post["applied_peak_m"], raw_peak_m=post["raw_peak_m"], n_obj=len(objects),
            emergency_stop=emergency, object_stale=False, lidar=self._lidar_log,
            lidar_vehicles=getattr(self, '_tunnel_vehicle_status', None), lead=getattr(self, '_lead_state', None),
            reverse_veto=getattr(self, "_reverse_veto", None),
            avoidance_evidence=post["avoidance_evidence"],
            tunnel=dict(valid=not bool(error), reason=match.get("reason"), fresh_scan=fresh_scan,
                map_match_valid=bool(map_match.get('valid')),
                exit_anchor=match.get('exit_anchor'),
                map_match_reason=map_match.get('reason'),
                relative_geometry=bool(match.get('relative_geometry')),
                body_walls=match.get('body_walls'), fallback_plan=fallback_plan_info,
                scan_stamp=decision.sensor_stamp, wall_stats=match.get("wall_stats"), aligned_ego=aligned,
                clear_distance_m=distance, geometry_clear_distance_m=geometric_distance,
                observed_clear_distance_m=observed_distance, bypass_blocked=blocked,
                clearance_components_m=clearances, limiting_source=limiting_source,
                localization_ms=localization_ms, rejoin_complete=rejoin_complete,
                exit_zone=exit_zone, exit_centre=exit_centre, exit_rejoin=exit_rejoin_info,
                model_guidance=model_info, no_go=no_go_info,
                model_policy=getattr(self, "_tunnel_model_log", None) if TUNNEL_MODEL else None,
                path_curvature_1pm=curvature if math.isfinite(curvature) else None,
                curve_cap_kmh=round(curve_cap * 3.6, 1) if math.isfinite(curve_cap) else None,
                curve_cap_station_m=round(curve_station, 1),
                roadwheel_peak_deg=math.degrees(math.atan(3.90 * curvature)) if math.isfinite(curvature) else None,
                plan={key: plan[key] for key in (
                    "kind", "transition_m", "offset_m", "reused", "speed_cap_mps",
                    "steering_rate_cap_mps", "steering_gradient_radpm", "planning_budget_exhausted",
                    "candidates_evaluated", "side_margin_m", "end_margin_m", "tight_pass",
                    "selection_reason", "search_max_offset_m", "reshaped", "hold_until_m", "direction", "bridged",
                    "coverage_escape")
                    if plan and key in plan},
                boundary_recovery=boundary_recovery, coverage_escape=coverage_escape, moving_objects=len(moving),
                detector_anchor=getattr(self, "_tunnel_detector_anchor", None),
                hazards=hazards,
                objects=[dict(name=o[3], kind=o[4], x=o[0], y=o[1], length_m=o[6], width_m=o[7],
                    heading_deg=o[8], id=o[11] if len(o)>11 else o[3],
                    vx_mps=o[9] if len(o)>9 else 0., vy_mps=o[10] if len(o)>10 else 0.,
                    speed_mps=o[5], point_count=(o[12] if len(o)>12 and isinstance(o[12], dict) else {}).get("point_count"))
                    for o in objects]),
            planner_cap_kmh=cap * 3.6,
            published_cap_kmh=getattr(self.path_publisher, "_last_target_mps", 0.) * 3.6,
            latency_ms=model_info.get("latency_ms", 0.), decision_ms=(time.perf_counter() - started) * 1000.,
            target_decision_hz=getattr(self, "decision_target_hz", 15.),
            decision_hz=getattr(self, "decision_hz", 0.), route_dist_m=planning_status["path"].get("distance_m"),
            ego_age_s=ego.get("age_s"), obj_src="tunnel_lidar", clearance="tunnel_lidar")
        # Logged object velocities are measured units; planners used / rtf.
        record.update(self.sim_clock.record() if getattr(self, 'sim_clock', None) is not None
                      else dict(rtf=round(runtime_rtf(self), 3), rtf_measured=False))
        if self.path_publisher and degraded is not None:
            record.update(record_override)
        try:   # v6 summary (Claude working copy)
            from gui import v6_record_fields
            record.update(v6_record_fields(self))
        except Exception:
            pass
        self._web_record = record
        if not hasattr(self, "_decision_log"):
            sid = os.path.basename(os.environ.get("STUDENT_SESSION_DIR", "")) or "nosave"
            directory = os.environ.get("STUDENT_LOG_DIR", str(Path(self.args.status).parent))
            self._decision_log = AsyncDecisionLog(os.path.join(directory, f"student_gui_{sid}.jsonl"))
        try:
            from gui import body_gap_m
            record["body_gap_m"] = body_gap_m(self.lidar)
        except Exception:
            pass
        self._decision_log.write(json.dumps(record, default=json_default) + "\n")
        return True
