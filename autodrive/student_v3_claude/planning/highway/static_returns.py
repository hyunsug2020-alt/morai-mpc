"""World-fixed confirmation for far highway LiDAR returns."""
from collections import deque
import math

import numpy as np
from scipy.spatial import cKDTree

from highway_planner import world_points


class FarReturnPersistence:
    """Count a far return as static only after it stays at one world position.

    A stationary obstacle returns at the same map position in every scan. Far
    returns that slide with traffic (rear faces of an undetected or partly
    masked car, ground hits at a constant range) never do, yet each became a
    30-95 m stop line and repeated 1.5-5 m/s^2 braking (E2E runs 2026-10-09
    185440Z/191227Z: 1-4 points 86-92 m ahead, moving with the ego for 8 s).

    Only returns more than ``near_m`` ahead are filtered; near, beside and rear
    returns pass unchanged. A far return counts once an earlier scan,
    ``min_age_s``-``max_age_s`` old, had a return within ``radius_m`` of it:
    real static obstacles are confirmed after about two scans, movers above
    radius_m/min_age_s (about 4 m/s) are not. Missing or reused stamps fail safe
    by returning every point.
    """
    def __init__(self, near_m=35., radius_m=.5, min_age_s=.12, max_age_s=.7):
        self.near_m = float(near_m)
        self.radius_m = float(radius_m)
        self.min_age_s = float(min_age_s)
        self.max_age_s = float(max_age_s)
        self.history = deque()
        self.info = {}

    def reset(self):
        self.history.clear()

    def update(self, points, ego, stamp):
        self.info = dict(far_n=0, confirmed_n=0, dropped_n=0, near_m=self.near_m)
        if points is None:
            return None
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or len(points) == 0:
            return points
        try:
            stamp = float(stamp)
        except (TypeError, ValueError):
            stamp = float('nan')
        far = points[:, 0] > self.near_m
        self.info['far_n'] = int(np.count_nonzero(far))
        if not math.isfinite(stamp) or stamp <= 0.:
            self.history.clear()
            self.info['reason'] = 'no scan stamp: unfiltered'
            return points
        if self.history and stamp < self.history[-1][0]:
            self.history.clear()
        while self.history and stamp-self.history[0][0] > self.max_age_s:
            self.history.popleft()
        world = world_points(points[far, :2], ego) if far.any() else np.empty((0, 2))
        earlier = [xy for t, xy in self.history
                   if self.min_age_s-1e-9 <= stamp-t <= self.max_age_s and len(xy)]
        confirmed = np.zeros(len(world), dtype=bool)
        if earlier and len(world):
            distance, _ = cKDTree(np.vstack(earlier)).query(world)
            confirmed = distance <= self.radius_m
        if not self.history or stamp > self.history[-1][0]:
            self.history.append((stamp, world))
        keep = ~far
        keep[np.flatnonzero(far)[confirmed]] = True
        self.info.update(confirmed_n=int(np.count_nonzero(confirmed)),
                         dropped_n=int(np.count_nonzero(~keep)))
        return points[keep]
