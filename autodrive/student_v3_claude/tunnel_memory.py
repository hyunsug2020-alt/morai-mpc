"""Retain blind-zone obstacles until fresh measured rays prove them absent."""
import math

import numpy as np
from scipy.spatial import cKDTree

from highway_planner import NearFieldMemory, local_points, world_points
from ego_body import self_return_mask


class TunnelNearFieldMemory(NearFieldMemory):
    """Add observed-free clearing without an age-based blind-zone expiry.

    A retained XY point alone cannot establish free space. Preserve its measured
    height above the road, then require rays on both sides to pass at or below
    that height in two fresh scans. An occluded low barrier remains occupied.
    Callers outside the physical tunnel retain NearFieldMemory's behavior.
    """

    def __init__(self):
        super().__init__(retain_measured_points=True, hold_blind=True)
        self.heights = {}
        self.free_counts = {}
        self.recently_freed = {}
        self.geometry = None
        self.geometry_stamp = None
        self.cleared_n = 0
        self.cleared_total = 0

    def stop_observing(self):
        self.geometry = None
        self.free_counts.clear()

    def observe_scan(self, scan, pose, mount):
        self.cleared_n = 0
        try:
            stamp = float(scan['stamp'])
            age = float(scan['age_s'])
            xyz = np.asarray(scan['xyz'], dtype=float).reshape(-1, 3)
            plane = np.asarray(scan['ground_plane'], dtype=float)
            mount = np.asarray(mount, dtype=float)
            valid = (pose is not None and math.isfinite(stamp) and stamp > 0.
                     and abs(float(pose['stamp'])-stamp) < 1e-6
                     and 0. <= age <= .25 and plane.shape == (3,) and mount.shape == (3,)
                     and np.isfinite(plane).all() and np.isfinite(mount).all()
                     and all(math.isfinite(float(pose[k])) for k in ('x', 'y', 'heading_deg')))
        except (KeyError, TypeError, ValueError):
            valid = False
        if valid and stamp == self.geometry_stamp:
            return
        if not valid or (self.geometry_stamp is not None and stamp < self.geometry_stamp):
            self.geometry = None
            self.free_counts.clear()
            return
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz) < 100:
            self.geometry = None
            self.free_counts.clear()
            return
        if self.geometry_stamp is not None and stamp-self.geometry_stamp > .4:
            self.free_counts.clear()
        self.geometry_stamp = stamp
        height = xyz[:, 2]-(xyz[:, :2] @ plane[:2]+plane[2])
        measured = xyz[self_return_mask(xyz) & (height > .14) & (height < 2.2)]
        tree = cKDTree(measured[:, :2]) if len(measured) else None
        rays = xyz-mount
        ranges = np.linalg.norm(rays[:, :2], axis=1)
        usable = ranges > .5
        rays, ranges = rays[usable], ranges[usable]
        unit = rays[:, :2]/ranges[:, None]
        self.geometry = dict(stamp=stamp, pose=dict(pose), plane=plane, mount=mount,
                             measured=measured, tree=tree, rays=rays, ranges=ranges)
        for key, observation in list(self.cells.items()):
            previous_height = self.heights.get(key)
            if previous_height is None or observation[0] >= stamp or observation[1] < 2:
                continue
            target = local_points(observation[2][None, :], pose)[0]
            if not (-2.5 < target[0] < 6.5 and abs(target[1]) < 3.):
                continue
            # A current return at the old face height vetoes clearing, even
            # when a neighboring beam sees beyond a narrow obstacle.
            neighbors = [] if tree is None else tree.query_ball_point(target, .15)
            occupied = any(measured[i, 2]-(target @ plane[:2]+plane[2]) >= previous_height-.1
                           for i in neighbors)
            vector = target-mount[:2]
            distance = float(np.linalg.norm(vector))
            free = False
            if not occupied and distance > .5:
                along = unit @ vector
                cross = unit[:, 0]*vector[1]-unit[:, 1]*vector[0]
                z_at_target = mount[2]+rays[:, 2]*along/ranges
                h_at_target = z_at_target-(target @ plane[:2]+plane[2])
                through = ((along > 0.) & (ranges > along+.5) & (abs(cross) <= .06)
                           & (h_at_target >= max(.14, previous_height-.08))
                           & (h_at_target <= previous_height+.005))
                free = bool(np.any(through & (cross < -.002))
                            and np.any(through & (cross > .002)))
            self.free_counts[key] = self.free_counts.get(key, 0)+1 if free else 0
            if self.free_counts[key] >= 2:
                del self.cells[key]
                self.heights.pop(key, None)
                self.free_counts.pop(key, None)
                self.recently_freed[key] = stamp
                self.cleared_n += 1
                self.cleared_total += 1

    def update(self, points, ego, scan_stamp, moving_objects=()):
        geometry = self.geometry
        if geometry is None or geometry['stamp'] != scan_stamp:
            return super().update(points, ego, scan_stamp, moving_objects)
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        # Other planners can append retained points or detector-box edges.
        # These are constraints, not new measurements that renew a blind cell.
        tree = geometry['tree']
        self.recently_freed = {key: when for key, when in self.recently_freed.items()
                               if 0. <= scan_stamp-when <= .8}
        learn = np.ones(len(points), dtype=bool)
        if self.recently_freed and len(points):
            world = world_points(points, ego)
            source = local_points(world, geometry['pose'])
            distances = tree.query(source)[0] if tree is not None else np.full(len(points), np.inf)
            keys = np.round(world/.4).astype(int)
            learn = np.array([tuple(key) not in self.recently_freed or distance <= .12
                              for key, distance in zip(keys, distances)])
        previous = dict(self.cells)
        result = super().update(points[learn], ego, scan_stamp, moving_objects)
        for key, observation in self.cells.items():
            if observation[0] != scan_stamp or tree is None:
                continue
            point = local_points(observation[2][None, :], geometry['pose'])[0]
            distance, index = tree.query(point)
            if distance <= .12:
                measured = geometry['measured'][index]
                self.heights[key] = float(measured[2]-(measured[:2] @ geometry['plane'][:2]+geometry['plane'][2]))
            elif key in previous and previous[key][1] >= 2:
                # An exported blind cell cannot become its own new evidence.
                # Unassociated new constraints still receive the legacy two
                # observations and remain occupied without a height certificate.
                self.cells[key] = previous[key]
        self.heights = {key: value for key, value in self.heights.items() if key in self.cells}
        self.free_counts = {key: value for key, value in self.free_counts.items() if key in self.cells}
        return np.vstack((points[~learn], result)) if np.any(~learn) else result
