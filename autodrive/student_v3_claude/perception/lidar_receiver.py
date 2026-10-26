"""LiDAR free-space check for candidate paths (competition: no ground-truth object list; a semantic camera detector decides WHAT is
there later, the LiDAR measures WHERE). Pure numpy; rospy is used only when a topic is given.

Frame: vehicle frame (x forward, y left, origin = localized base_link pose),
matching the GUI's local route and candidate paths. Points are shifted by the
selected sensor mount (competition default: x=1.05, y=0, z=1.35, rot 0)."""
import copy
import math, os, threading, time
import numpy as np
from ego_body import EGO_HALF_WIDTH_M

from .lidar_geometry import _DT, outside_ego_footprint, fit_road_plane, cloud_xyz
from .pedestrian_motion import PedestrianMotionHistory, without_moving_objects

MAPPED_POSE_WAIT_S = .08


class LidarCorridor:
    def __init__(self, topic=None, mount=(1.05, 0.0, 1.35), z_min=0.30, z_max=2.2, x_range=(-3.0, 40.0), y_max=12.0, grid_m=0.2, max_age_s=0.6, road_floor=None, pose_at_stamp=None):
        self.mount = np.asarray(mount, np.float32); self.z_min, self.z_max = z_min, z_max
        self.x_range, self.y_max, self.grid_m, self.max_age_s = x_range, y_max, grid_m, max_age_s
        self.lock = threading.Lock()
        self._ingest_lock = threading.Lock()
        self._road_floor_lock = threading.Lock()
        self.road_floor, self.pose_at_stamp = road_floor, pose_at_stamp
        self._road_floor_sequence = None
        self.road_diagnostics = dict(applied=False, reason='unconfigured')
        self.pts = np.zeros((0, 2), np.float32); self.received = 0.0; self.n_raw = 0; self.ground_z = 0.0
        self.ground_plane = np.zeros(3)
        self._raw_xyz, self._scan_stamp, self._scan_sequence = None, None, 0
        self.sub = None
        if topic:
            import rospy
            from sensor_msgs.msg import PointCloud2
            if not rospy.core.is_initialized():
                rospy.init_node("morai_student_shadow", anonymous=True, disable_signals=True)
            self.sub = rospy.Subscriber(topic, PointCloud2, self.on_cloud, queue_size=1, buff_size=2 ** 24)

    def on_cloud(self, msg):
        try:
            stamp = float(msg.header.stamp.to_sec())
            self.ingest(cloud_xyz(msg), stamp=stamp)
        except Exception as exc:   # a malformed scan must never take the decision loop down
            with self.lock:
                self._raw_xyz = None
            if not getattr(self, "_err", False):
                self._err = True; print(f"[lidar] ingest error: {exc!r}", flush=True)

    def ingest(self, xyz, stamp=None):
        """Publish one atomic scan while preserving the existing XY clearance API.

        ``stamp`` is the source ROS time when supplied. Direct offline callers
        retain the one-argument API and receive a wall-clock timestamp.
        """
        with self._ingest_lock:
            try:
                received = time.monotonic()
                source_stamp = time.time() if stamp is None else float(stamp)
                xyz = np.asarray(xyz, dtype=np.float32)
                if xyz.ndim != 2 or xyz.shape[1] != 3:
                    raise ValueError("LiDAR XYZ must have shape (N, 3)")
                if not math.isfinite(source_stamp) or source_stamp <= 0.:
                    raise ValueError("LiDAR source stamp must be finite and positive")
                xyz = xyz[np.isfinite(xyz).all(axis=1)]
                p = xyz + self.mount               # rot = 0: sensor axes == vehicle axes
                with self.lock:
                    ground_z = self.ground_z
                near = p[(p[:, 0] > 2.0) & (p[:, 0] < 15.0) & (np.abs(p[:, 1]) < 3.0)]
                if len(near) > 50:
                    ground_z = float(np.percentile(near[:, 2], 5))
                plane = fit_road_plane(p, ground_z)
                ground = p[:, :2] @ plane[:2] + plane[2]
                z = p[:, 2] - ground
                keep = (z > self.z_min) & (z < self.z_max) & outside_ego_footprint(p) & (p[:, 0] > self.x_range[0]) & (p[:, 0] < self.x_range[1]) & (np.abs(p[:, 1]) < self.y_max)
                q = p[keep, :2]
                if len(q):
                    q = np.unique(np.round(q / self.grid_m).astype(np.int32), axis=0).astype(np.float32) * self.grid_m
                # A new cloud normally precedes its measured pose bracket.
                # At the certified rising-road patch, exposing its temporary
                # near-plane classification produces a false collision pulse.
                # Keep the preceding scan visible while synchronizing this
                # cloud, then publish its classification atomically. New near
                # bodies skip this wait; all source ages include processing.
                # The first scan entering a calibrated road section needs
                # the same source-pose synchronization as later scans. The
                # preceding scan may still be outside the calibrated section.
                wait_for_pose = bool(self.road_floor is not None
                    and getattr(self.road_floor, 'certified', False))
                # A roadside kerb is outside the current body corridor; it
                # must not disable synchronization for distant road returns.
                # New bodies in the footprint corridor retain immediate raw
                # protection, including while the measured pose is pending.
                near_body = bool(len(q) and np.any((q[:, 0] > -1.) & (q[:, 0] < 20.)
                    & (abs(q[:, 1]) < EGO_HALF_WIDTH_M+.25)))
                mapped = None
                if self.road_floor is not None and self.pose_at_stamp is not None and getattr(self.road_floor, 'certified', False):
                    deadline = received+(MAPPED_POSE_WAIT_S if wait_for_pose and not near_body else 0.)
                    while True:
                        if self._road_floor_lock.acquire(blocking=False):
                            try:
                                mapped = self._mapped_points(p, plane, source_stamp)
                            finally:
                                self._road_floor_lock.release()
                        if mapped is not None or time.monotonic() >= deadline:
                            break
                        time.sleep(.002)
                with self.lock:
                    self.pts, self.received = (mapped[0] if mapped is not None else q), received
                    self.n_raw, self.n_kept = len(p), int(keep.sum())
                    self.ground_z, self.ground_plane = ground_z, plane
                    self._raw_xyz, self._scan_stamp = p, source_stamp
                    self._scan_sequence += 1
                    self.road_diagnostics = (copy.deepcopy(mapped[2]) if mapped is not None else
                        dict(applied=False, reason='awaiting measured source pose'))
                    if mapped is not None:
                        self.n_kept = mapped[1]
                        self._road_floor_sequence = self._scan_sequence
                self._refresh_road_floor()
            except Exception:
                with self.lock:
                    self._raw_xyz = None
                raise

    def _mapped_points(self, points, plane, stamp):
        """Classify one cloud only with its measured, time-aligned context."""
        try:
            started = time.monotonic()
            pose = self.pose_at_stamp(stamp)
            if (not isinstance(pose, dict) or pose.get('measured') is not True or
                    pose.get('imu_measured') is not True or
                    abs(float(pose.get('stamp', 0.))-float(stamp)) > 1e-6):
                return None
            age = max(float(pose.get('age_s', float('inf'))), float(pose.get('imu_age_s', float('inf'))))
            ground, diagnostic = self.road_floor.heights(points, plane, pose)
            ground = np.asarray(ground, dtype=float)
            if ground.shape != (len(points),) or not np.isfinite(ground).all():
                return None
            elapsed = time.monotonic()-started
            if elapsed < 0. or not math.isfinite(age) or age+elapsed > .2:
                return None
            height = points[:, 2]-ground
            keep = ((height > self.z_min) & (height < self.z_max) & outside_ego_footprint(points) &
                    (points[:, 0] > self.x_range[0]) & (points[:, 0] < self.x_range[1]) & (abs(points[:, 1]) < self.y_max))
            q = points[keep, :2]
            if len(q):
                q = np.unique(np.round(q/self.grid_m).astype(np.int32), axis=0).astype(np.float32)*self.grid_m
            return q, int(keep.sum()), diagnostic
        except Exception:
            return None

    def _refresh_road_floor(self):
        """Complete a deferred calibration without refreshing the captured scan."""
        if (self.road_floor is None or self.pose_at_stamp is None or
                not getattr(self.road_floor, 'certified', False) or
                not self._road_floor_lock.acquire(blocking=False)):
            return
        try:
            with self.lock:
                sequence = self._scan_sequence
                if (self._raw_xyz is None or sequence == self._road_floor_sequence or
                        not 0. <= time.monotonic()-self.received <= self.max_age_s):
                    return
                points = self._raw_xyz.copy()
                plane = self.ground_plane.copy()
                stamp, received = self._scan_stamp, self.received
            try:
                capture_at = time.monotonic()
                pose = self.pose_at_stamp(stamp)
                if (not isinstance(pose, dict) or pose.get('measured') is not True or
                        pose.get('imu_measured') is not True or
                        abs(float(pose.get('stamp', 0.))-float(stamp)) > 1e-6):
                    return
                pose_age = max(float(pose.get('age_s', float('inf'))),
                               float(pose.get('imu_age_s', float('inf'))))
                ground, diagnostic = self.road_floor.heights(points, plane, pose)
                ground = np.asarray(ground, dtype=float)
                if ground.shape != (len(points),) or not np.isfinite(ground).all():
                    raise ValueError('mapped road floor returned malformed heights')
                height = points[:, 2]-ground
                keep = ((height > self.z_min) & (height < self.z_max) &
                        outside_ego_footprint(points) & (points[:, 0] > self.x_range[0]) &
                        (points[:, 0] < self.x_range[1]) & (np.abs(points[:, 1]) < self.y_max))
                q = points[keep, :2]
                if len(q):
                    q = np.unique(np.round(q/self.grid_m).astype(np.int32), axis=0).astype(np.float32)*self.grid_m
            except Exception:
                # Optional map support cannot invalidate the original scan.
                return
            with self.lock:
                commit_at = time.monotonic()
                elapsed = commit_at-capture_at
                if (sequence != self._scan_sequence or stamp != self._scan_stamp or
                        received != self.received or self._raw_xyz is None or
                        not 0. <= elapsed or pose_age+elapsed > .2 or
                        not 0. <= commit_at-received <= self.max_age_s):
                    return
                self.pts, self.n_kept = q, int(keep.sum())
                self.road_diagnostics = copy.deepcopy(diagnostic)
                self._road_floor_sequence = sequence
        finally:
            self._road_floor_lock.release()

    def snapshot(self):
        self._refresh_road_floor()
        with self.lock:
            age = time.monotonic() - self.received
            return (self.pts if age <= self.max_age_s else None), age

    def xy_snapshot(self):
        """Read the XY scan and its source time atomically without copying XYZ."""
        self._refresh_road_floor()
        with self.lock:
            age = time.monotonic() - self.received
            return dict(points=self.pts if age <= self.max_age_s else None,
                        age_s=age, stamp=self._scan_stamp, received=self.received,
                        sequence=self._scan_sequence,
                        mapped_road_floor=copy.deepcopy(self.road_diagnostics))

    def scan_snapshot(self):
        """Return one timestamped vehicle-frame XYZ scan for wall localization.

        Arrays are independent read-only copies. Malformed input invalidates
        this raw scan immediately; stale scans also return ``xyz=None``.
        The existing XY snapshot keeps its established last-valid-scan grace.
        """
        self._refresh_road_floor()
        with self.lock:
            age = time.monotonic() - self.received
            xyz = (self._raw_xyz.copy() if self._raw_xyz is not None and
                   0. <= age <= self.max_age_s else None)
            obstacles = self.pts.copy() if xyz is not None else None
            plane = self.ground_plane.copy()
            stamp, sequence = self._scan_stamp, self._scan_sequence
            road_diagnostics = copy.deepcopy(self.road_diagnostics)
        if xyz is not None:
            xyz.setflags(write=False)
            obstacles.setflags(write=False)
        plane.setflags(write=False)
        return dict(xyz=xyz, obstacle_points=obstacles, ground_plane=plane, stamp=stamp,
                    sequence=sequence, age_s=age, mapped_road_floor=road_diagnostics)

    def close(self):
        if self.sub is not None:
            self.sub.unregister()
            self.sub = None

    def blocked(self, path_xy, half_width_m, max_s_m=30.0, pts=None, min_s_m=2.0):
        """First arc length (m) along `path_xy` (vehicle frame) where an obstacle point lies within half_width_m of the centreline,
        plus the side of the blocking points (+1 left / -1 right of the path) and their count. None if the corridor is clear
        (or no fresh scan: caller checks `snapshot()[0] is None`)."""
        if pts is None:
            pts, _ = self.snapshot()
        if pts is None or len(pts) == 0:
            return None
        path = np.asarray(path_xy, np.float32)[:, :2]
        if len(path) < 2:
            return None
        s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))))
        sel = (s <= max_s_m) & (s >= min_s_m)   # the vehicle origin is mid-car: arc lengths < 2 m are beside the body, not ahead of the bumper
        path, s = path[sel], s[sel]
        if len(path) < 2:
            return None
        # only points near the path's bounding box matter
        lo, hi = path.min(axis=0) - half_width_m, path.max(axis=0) + half_width_m
        cand = pts[(pts[:, 0] >= lo[0]) & (pts[:, 0] <= hi[0]) & (pts[:, 1] >= lo[1]) & (pts[:, 1] <= hi[1])]
        if len(cand) == 0:
            return None
        d = np.sqrt(((path[:, None, :] - cand[None, :, :]) ** 2).sum(axis=2))    # (M, N)
        hit = d <= half_width_m
        rows = np.where(hit.any(axis=1))[0]
        if len(rows) == 0:
            return None
        i = int(rows[0])
        tang = path[min(i + 1, len(path) - 1)] - path[max(i - 1, 0)]
        blockers = cand[hit[i]]
        side = float(np.sign(np.mean((blockers[:, 0] - path[i, 0]) * -tang[1] + (blockers[:, 1] - path[i, 1]) * tang[0])) or 1.0)
        return float(s[i]), side, int(hit.any(axis=0).sum())


def margin_for_speed(v_mps):
    """extra lateral margin beyond the car half width (0.95 m): 0.25 m when creeping, 0.4 m at 15 km/h, 0.55 m at 30 km/h"""
    return float(np.clip(0.25 + 0.036 * v_mps, 0.25, 0.6))
