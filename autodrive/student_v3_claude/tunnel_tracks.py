"""Measured motion of unlabeled tunnel LiDAR clusters in uncorrected odometry."""
from collections import deque
import math

import numpy as np

from motion_filter import measured_velocity

# A static body's returns slide over it while the ego passes: the far end is
# revealed, the near end leaves the clustering window behind the car, sparse rings
# re-sample a face. Run follow-20261009T204445Z (no movers in the tunnel) read
# 0.50-2.65 m/s on parked cars passed 3.2-3.8 m away and 1.1-3.6 m/s on car faces
# 40-50 m ahead: six moving-conflict stops (cap 0) in two passes. Beyond 20 m the
# -3 deg ring (1.35 m mount, +-0.5 deg pitch) meets a body below the 0.14 m ground
# cut, so one ring samples it and its returns slide over the face's depth (bumper
# to rear window, < 2 m): recorded static faces moved up to 1.34 m along the line
# of sight within 0.8 s, also with the ego stopped (10-02/10-03 captures, 23-55 m).
RADIAL_MOTION_RANGE_M = 20.
FACE_RESAMPLE_DEPTH_M = 2.
# Nearer, each ring that starts to hit a body re-samples its face as well: a
# parked car's rear face 22 -> 19 m ahead stepped 0.5-0.8 m towards the ego per
# ring entry (1.7 m in 0.8 s), read as 2.3-3.5 m/s and stopped the car
# (follow-20261010T062150Z 279.6-280.3 s). A face wider than a walking person
# and at most half as deep moves along the line of sight only beyond that depth.
FACE_WIDTH_M = 1.2


# PointPillars boxes of the scenario's parked cars drifted with their visible part
# at up to 3.8 m/s with velocity_valid (follow-20261009T191227Z 348.6 s, 45 m ahead;
# 20261009T204445Z: 0.5-3.65 m/s). Slower detector motion needs a measured mover.
DETECTOR_BOX_DRIFT_MPS = 5.

# A cluster as long as a car (0.1 m box padding included) is no walking person or
# cyclist: a parked car's visible part (its whole body, a corner or only the side
# facing the ego) slides along it while the ego passes. The 17 E2E drives of
# 404_contest.json from follow-20261009T171938Z to
# 20261010T151125Z had no tunnel mover (every tunnel object there is static), yet
# TunnelTracks confirmed 46 moving tracks; 26 were car-body clusters of the two
# parked Grandeurs (UNIQUEID 31/32: 4.2-5.1 x 1.0-2.4 m, 20-109 points, 3-5 m beside
# the path) at 0.51-2.65 m/s, the cause of 19 of the 21 planner "tunnel moving object
# conflict" stops and of the live "collision: tunnel_lidar_dynamic" stops beside those
# cars (follow-20261010T151125Z 00:16:40.5: 38 -> 12 km/h; 00:22:00.7: cap 0 at the
# exit). A body that long starts moving only above the fastest such slide (2.65 m/s;
# 24 of the 26 at most 1.82 m/s, 2.43 m/s in the 10-10 drives); slower it stays a
# static obstacle for the bypass and the raw-return body sweep, like a parked car.
# A car really moving is still confirmed: crossing at 3 m/s into the path
# (test_tunnel_tracks_recorded), driving at 4-5 m/s; a confirmed body keeps the
# common 0.3 m/s release.
VEHICLE_BODY_LENGTH_M = 2.5
VEHICLE_BODY_MOVING_MPS = 2.8


def vehicle_body(size):
    """Measured cluster box (length, width) as long as a car: not a person or a bike."""
    return float(max(size[0], size[1])) >= VEHICLE_BODY_LENGTH_M


def measured_motion_supports(car, movers, margin_m=1.):
    """True when a confirmed measured mover lies inside a detector car's box."""
    yaw = math.radians(float(car[8]))
    along = np.array([math.cos(yaw), math.sin(yaw)])
    for mover in movers:
        delta = np.asarray(mover[:2], dtype=float)-np.asarray(car[:2], dtype=float)
        if (abs(float(delta @ along)) <= .5*float(car[6])+margin_m and
                abs(float(delta @ [-along[1], along[0]])) <= .5*float(car[7])+margin_m):
            return True
    return False


def measured_vehicle(car, movers, unknown_static=False, persisted=None):
    """Keep a detector car's motion only beyond box drift or with a measured mover.

    Below the box drift a valid velocity needs a confirmed measured mover in the
    box. The publish-time check (unknown_static) applies the same to an unknown
    velocity, which it would otherwise fit from the drifting box itself
    (follow-20261009T204445Z 294.8-295.1 s; follow-20261010T060515Z 281.8 s and
    560.0 s: parked cars passed in the tunnel, "stopping envelope"/"collision:
    lidar_car", 38 -> 0 km/h). The car's body stays an obstacle either way.
    persisted: whether the box moved with that velocity since the previous detector
    frame; a box re-fitted around a parked car's other end read 5.9 m/s and was
    0 m/s one frame later (follow-20261010T062150Z 281.5 s, "collision:
    lidar_car" at 6 km/h).
    """
    valid = len(car) > 14 and bool(car[14])
    fast = valid and float(car[5]) >= DETECTOR_BOX_DRIFT_MPS and persisted is not False
    if (len(car) <= 14 or measured_motion_supports(car, movers) or fast or
            not (valid or unknown_static)):
        return car
    values = list(car)
    values[5], values[9], values[10], values[14] = 0., 0., 0., True
    return tuple(values)


# The 281.5 s box jumped 1.7 m in 0.14 s, 0.85 m beyond its reported 5.9 m/s;
# a box tracking a moving car follows its own velocity within its fit noise.
BOX_TRAVEL_TOLERANCE_M = .5


class DetectorMotion:
    """Previous distinct detector frame of each car, for measured_vehicle(persisted)."""
    def __init__(self):
        self.frames = {}

    def persisted(self, car, stamp):
        rows = self.frames.get(car[11], ())
        if not rows or rows[-1][0] != stamp:
            rows = (*rows, (float(stamp), float(car[0]), float(car[1])))[-2:]
            self.frames[car[11]] = rows
        elapsed = rows[-1][0]-rows[0][0]
        if len(rows) < 2 or not 0. < elapsed <= .5:
            return False
        moved = np.asarray(rows[1][1:])-rows[0][1:]
        return bool(np.linalg.norm(moved-np.asarray(car[9:11], float)*elapsed) <= BOX_TRAVEL_TOLERANCE_M)


def static_body(obj, keep_person=False):
    """Zero an object's motion; a non-person body is left to the raw-point check."""
    values = list(obj)
    values[5] = 0.
    if len(values) > 10:
        values[9] = values[10] = 0.
    if not (keep_person and values[4] == 'pedestrian'):
        values[4] = 'obstacle'
    return tuple(values)


class TunnelTracks:
    """Keep stable cluster IDs and expose only confirmed motion to yield rules.

    ``pedestrian`` is the existing planner's generic yielding category here;
    this tracker performs no semantic classification. Dynamic names explicitly
    describe a LiDAR moving object. Independent raw-point collision checks must
    continue to cover untracked returns and motion warmup.
    """

    def __init__(self, max_gap_s=.5, max_speed_mps=15., moving_threshold_mps=.8, release_mps=.3):
        self.max_gap_s = float(max_gap_s)
        self.max_speed_mps = float(max_speed_mps)
        # Walking floor (user rule: a person walking >= ~0.8 m/s makes the car
        # yield); the live path check already treats slower clusters as static.
        # A confirmed mover keeps yielding until it slows below the release speed.
        self.moving_threshold_mps = float(moving_threshold_mps)
        self.release_mps = min(float(release_mps), self.moving_threshold_mps)
        self.tracks = {}
        self.last_stamp = None
        self.last_output = []
        self.next_id = 800000

    def reset(self):
        self.tracks.clear()
        self.last_stamp = None
        self.last_output = []

    @staticmethod
    def _shape_changed(old, new):
        # Changing visible faces can move a centroid without physical motion.
        return bool(np.any(np.abs(new - old) > np.maximum(.4, .25 * np.maximum(old, new))))

    @staticmethod
    def _wall_like(size):
        longest, shortest = float(np.max(size)), float(np.min(size))
        return longest > 8. or (longest >= 6. and shortest <= .35)

    @staticmethod
    def _motion_supported(obj, size):
        # Cluster boxes include 100 mm padding on each face. Require at least
        # 300 mm of measured extent on both axes: isolated beam returns and
        # single rows of wall/road edges shift with viewpoint, even in map space.
        if np.any(size < .5 - 1e-9):
            return False
        if len(obj) > 12 and isinstance(obj[12], dict):
            try:
                count = float(obj[12].get("point_count", 0))
            except (TypeError, ValueError):
                return False
            if not math.isfinite(count) or count < 4:
                return False
        return True

    @staticmethod
    def _coherent_motion(history):
        # At the shortest eligible baseline there is only one long endpoint
        # pair. A visible vehicle face can jump back and forth while that pair
        # suggests translation. Require most measured travel to contribute to
        # net motion; smooth turns and ordinary centroid noise remain eligible.
        xy = np.asarray(history, dtype=float)[:, 1:3]
        if len(xy) < 4:
            return False
        travel = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
        net = float(np.linalg.norm(xy[-1] - xy[0]))
        return travel <= 1e-9 or net >= .6 * travel

    @staticmethod
    def _recent_motion_supported(history, velocity, minimum_speed):
        # A long-baseline fit can confirm motion after the visible surface has
        # already stopped shifting. Require its direction to remain supported
        # over the latest 150 ms; this also rejects a settling box centroid.
        samples = np.asarray(history, dtype=float)
        previous = np.flatnonzero(samples[:, 0] <= samples[-1, 0] - .15 + 1e-9)
        if not len(previous):
            return False
        first = samples[previous[-1]]
        elapsed = samples[-1, 0] - first[0]
        recent = (samples[-1, 1:3] - first[1:3]) / elapsed
        direction = np.array(velocity, dtype=float, copy=True)
        direction /= max(float(np.linalg.norm(direction)), 1e-9)
        return float(recent @ direction) > minimum_speed

    @staticmethod
    def _median_slope(times, values):
        dt = times[None, :]-times[:, None]
        pairs = dt >= .3-1e-9
        return float(np.median((values[None, :]-values[:, None])[pairs]/dt[pairs])) if pairs.any() else None

    def _view_credited_velocity(self, history, views, velocity):
        """Credit motion of the measured body, not of its sampled surface patch.

        Views hold the ego-axis extent of the returns and the observer pose.
        Along the travel axis a revealed far end or the clustering-window cut
        behind the car moves one edge only; a moving body carries both. Across
        it a revealed or occluded far side moves only the far edge, while the
        edge nearest the observer leads a person stepping out from behind a
        parked car: that edge must carry lateral motion.
        """
        samples, views = np.asarray(history, dtype=float), np.asarray(views, dtype=float)
        times = samples[:, 0]
        heading = math.radians(views[-1, 4])
        forward = np.array([math.cos(heading), math.sin(heading)])
        left = np.array([-forward[1], forward[0]])
        along, across = float(np.dot(velocity, forward)), float(np.dot(velocity, left))
        credited = np.zeros(2)
        centre = samples[:, 1:3] @ forward
        edges = [self._median_slope(times, centre+side*.5*views[:, 0]) for side in (-1., 1.)]
        if all(edge is not None and edge*along > 0. and abs(edge) >= .5*abs(along) for edge in edges):
            credited += along*forward
        offset = float((samples[-1, 1:3]-views[-1, 2:4]) @ left)
        near = (None if abs(offset) <= .5*views[-1, 1] else self._median_slope(
            times, samples[:, 1:3] @ left-math.copysign(.5, offset)*views[:, 1]))
        if abs(offset) <= .5*views[-1, 1] or (near is not None and near*across > 0. and
                                             abs(near) >= .5*abs(across)):
            credited += across*left
        sight = samples[-1, 1:3]-views[-1, 2:4]
        distance = float(np.linalg.norm(sight))
        sight /= max(distance, 1e-9)
        cos, sin = abs(float(sight @ forward)), abs(float(sight @ left))
        depth, width = views[-1, 0]*cos+views[-1, 1]*sin, views[-1, 0]*sin+views[-1, 1]*cos
        face = width >= FACE_WIDTH_M and depth <= .5*width
        if ((distance > RADIAL_MOTION_RANGE_M or face) and
                abs(float((samples[-1, 1:3]-samples[0, 1:3]) @ sight)) < FACE_RESAMPLE_DEPTH_M):
            credited -= float(credited @ sight)*sight
        return float(credited[0]), float(credited[1])

    @staticmethod
    def _measurement(obj):
        if len(obj) < 9:
            raise ValueError("Tunnel cluster requires position and box dimensions")
        metadata = obj[12] if len(obj) > 12 and isinstance(obj[12], dict) else {}
        centre = np.asarray(metadata.get("motion_center", obj[:2]), dtype=float)
        size = np.asarray(obj[6:8], dtype=float)
        if centre.shape != (2,) or not np.isfinite(centre).all() or not np.isfinite(size).all() or np.any(size <= 0.):
            raise ValueError("Tunnel cluster position and dimensions must be finite and positive")
        view = None
        if "motion_extent" in metadata or "observer" in metadata:
            extent = np.asarray(metadata.get("motion_extent", ()), dtype=float)
            observer = np.asarray(metadata.get("observer", ()), dtype=float)
            if (extent.shape != (2,) or observer.shape != (3,) or not np.isfinite(extent).all() or
                    not np.isfinite(observer).all() or np.any(extent < 0.)):
                raise ValueError("Tunnel cluster view requires a finite extent and observer pose")
            view = np.r_[extent, observer]
        return centre, np.maximum(size, .3), view

    def update(self, objects, stamp):
        """Return confirmed velocity and stable IDs, preserving measured box axes.

        Input coordinates must already share the uncorrected odometry world.
        The tunnel caller converts confirmed boxes back to the matched map
        frame; localization feedback therefore cannot fabricate object motion.
        Unique source timestamps define the history; repeated calls never create
        extra motion evidence. Confirmed moving clusters survive at most 250 ms
        of missing returns by bounded velocity prediction; other missing clusters
        are kept only for ID association.
        """
        stamp = float(stamp)
        if not math.isfinite(stamp) or stamp <= 0.:
            raise ValueError("Tunnel tracking stamp must be finite and positive")
        objects = list(objects)
        measurements = [self._measurement(obj) for obj in objects]
        if self.last_stamp is not None:
            if stamp < self.last_stamp or stamp - self.last_stamp > self.max_gap_s:
                self.reset()
            elif abs(stamp - self.last_stamp) < 1e-9:
                return list(self.last_output)
        self.last_stamp = stamp
        self.tracks = {uid: track for uid, track in self.tracks.items()
                       if stamp - track["stamp"] <= self.max_gap_s + 1e-9}

        centres = np.asarray([measurement[0] for measurement in measurements]).reshape(-1, 2)
        sizes = np.asarray([measurement[1] for measurement in measurements]).reshape(-1, 2)
        candidates = []
        for uid, track in self.tracks.items():
            dt = stamp - track["stamp"]
            predicted = track["centre"] + track["velocity"] * dt
            gate = min(4.5, .65 + self.max_speed_mps * dt)
            distances = np.linalg.norm(centres - predicted, axis=1)
            for index in np.flatnonzero(distances <= gate):
                shape_penalty = min(1., float(np.linalg.norm(sizes[index] - track["size"]))) * .25
                candidates.append((float(distances[index]) + shape_penalty, uid, int(index)))
        assigned, used_tracks = {}, set()
        for _, uid, index in sorted(candidates):
            if uid not in used_tracks and index not in assigned:
                assigned[index] = uid
                used_tracks.add(uid)

        output, observed_ids = [], set()
        for index, (obj, measurement) in enumerate(zip(objects, measurements)):
            centre, size, view = measurement
            uid = assigned.get(index)
            if uid is None:
                uid = self.next_id
                self.next_id += 1
                self.tracks[uid] = dict(centre=centre, size=size, stamp=stamp,
                                       velocity=np.zeros(2), moving=False,
                                       history=deque(maxlen=32), sizes=deque(maxlen=32),
                                       views=deque(maxlen=32))
            track = self.tracks[uid]
            observed_ids.add(uid)
            supported = self._motion_supported(obj, size)
            discontinuous = (stamp - track["stamp"] > .25 + 1e-9 or
                             self._shape_changed(track["size"], size) or
                             not supported)
            if discontinuous:
                track["history"].clear()
                track["sizes"].clear()
                track["views"].clear()
                track["moving"] = False
            if supported:
                track["history"].append((stamp, float(centre[0]), float(centre[1])))
                track["sizes"].append((stamp, size.copy()))
                track["views"].append(view)
            while track["history"] and stamp - track["history"][0][0] > .8 + 1e-9:
                track["history"].popleft()
                track["sizes"].popleft()
                track["views"].popleft()

            # Epoch-second float stamps lose fractions of a microsecond. Use
            # relative microsecond times so a nominal 300 ms baseline remains
            # eligible instead of occasionally requiring an extra scan.
            relative_history = [(round(t - stamp, 6), x, y)
                                for t, x, y in track["history"]]
            velocity = None if not supported or self._wall_like(size) else measured_velocity(
                relative_history, max_speed_mps=self.max_speed_mps)
            if velocity is not None and len(track["sizes"]) >= 4:
                sizes = np.asarray([row[1] for row in track["sizes"]])
                reference = np.median(sizes, axis=0)
                if any(self._shape_changed(reference, value) for value in sizes):
                    velocity = None
            if velocity is not None and not self._coherent_motion(relative_history):
                velocity = None
            views = list(track["views"])
            if velocity is not None and views and all(view is not None for view in views):
                velocity = self._view_credited_velocity(relative_history, views, velocity)
            # A car-sized body starts moving only at vehicle speed (see
            # VEHICLE_BODY_MOVING_MPS); once confirmed it keeps the common release.
            body = vehicle_body(size) or any(vehicle_body(row[1]) for row in track["sizes"])
            threshold = (self.release_mps if track["moving"] else
                         max(self.moving_threshold_mps, VEHICLE_BODY_MOVING_MPS) if body else
                         self.moving_threshold_mps)
            if velocity is not None and not self._recent_motion_supported(
                    relative_history, velocity, threshold):
                velocity = None
            vx, vy = velocity if velocity is not None else (0., 0.)
            speed = math.hypot(vx, vy)
            moving = velocity is not None and speed > threshold
            track.update(centre=centre, size=size, stamp=stamp,
                         velocity=np.asarray((vx, vy)), moving=moving)

            values = list(obj)
            while len(values) < 12:
                values.append(0.)
            values[11] = uid
            if moving:
                values[3] = "tunnel_lidar_dynamic_%d" % uid
                values[4], values[5] = "pedestrian", speed
                # Velocity direction is independent of the measured box axes.
                # Rotating a long box into its travel direction can mask returns
                # from another object and misrepresent the moving footprint.
                values[9], values[10] = vx, vy
            else:
                values[4], values[5], values[9], values[10] = "obstacle", 0., 0., 0.
            track["object"] = tuple(values)
            output.append(track["object"])
        for uid, track in self.tracks.items():
            elapsed = stamp - track["stamp"]
            if uid in observed_ids or not track["moving"] or elapsed > min(.25, self.max_gap_s) + 1e-9:
                continue
            values = list(track["object"])
            centre = np.asarray(track["object"][:2]) + track["velocity"] * elapsed
            values[0], values[1] = float(centre[0]), float(centre[1])
            output.append(tuple(values))
        self.last_output = output
        return list(output)
