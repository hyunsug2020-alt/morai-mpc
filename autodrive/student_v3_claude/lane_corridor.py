"""Vehicle footprint checks inside the route lane and permitted adjacent lanes."""
import numpy as np
from scipy.spatial import cKDTree

from ego_body import EGO_HALF_LENGTH_M, ROAD_LEGACY_SHIFT_M


def points_in_polygon(points, polygon):
    """Even-odd rule for many points against one polygon (N x 2, M x 2)."""
    x, y = points[:, 0], points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    xj, yj = polygon[-1]
    for xi, yi in polygon:
        crosses = (yi > y) != (yj > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_at = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= crosses & (x < x_at)
        xj, yj = xi, yi
    return inside


class LaneCorridor:
    def __init__(self, route, left, right, zones=None, intersection_mask=None,
                 body_shift_m=ROAD_LEGACY_SHIFT_M):
        # body_shift_m: body centre ahead of each path sample (ego_body). The tunnel
        # passes REAR_AXLE_SHIFT_M (the pose is the rear axle); the road and highway
        # keep the legacy box centred on the pose.
        self.body_shift_m = float(body_shift_m)
        # zones: no-go polygons in the same (ego-local) frame as route, from drivable_rules.json.
        self.zones = [np.asarray(z, dtype=float) for z in (zones or []) if len(z) >= 3]
        self.route = np.asarray(route, dtype=float)[:, :2]
        self.left, self.right = np.asarray(left), np.asarray(right)
        self.intersection_mask = (np.zeros(len(self.route), dtype=bool) if intersection_mask is None
                                  else np.asarray(intersection_mask, dtype=bool))
        if self.intersection_mask.shape != (len(self.route),):
            raise ValueError("Intersection mask must match the route length")
        tangent = np.gradient(self.route, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        self.normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        self.tree = cKDTree(self.route)

    def clear_distance(self, path, horizon_m=60.0):
        """First pose whose footprint leaves the legal lane corridor, in metres."""
        path = np.asarray(path, dtype=float)[:, :2]
        if len(path) < 3:
            return 0.0
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        sample = np.arange(0., min(horizon_m, arc[-1]) + .01, .25)
        centres = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
        if len(centres) < 3:
            return 0.0
        tangent = np.gradient(centres, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        # The body and its front/rear corners must fit, including a 20 cm margin.
        # The body centre lies body_shift_m ahead of each sample along its heading.
        shift = self.body_shift_m
        corners = np.stack([centres + a * tangent + b * normal
                            for a in (shift - EGO_HALF_LENGTH_M, shift, shift + EGO_HALF_LENGTH_M)
                            for b in (-1.146, 1.146)], axis=1)
        _, index = self.tree.query(corners.reshape(-1, 2))
        lateral = np.sum((corners.reshape(-1, 2) - self.route[index]) * self.normal[index], axis=1)
        violation = np.maximum(lateral - self.left[index], self.right[index] - lateral)
        violation = violation.reshape(len(centres), -1).max(axis=1)
        for zone in self.zones:
            inside = points_in_polygon(corners.reshape(-1, 2), zone).reshape(len(centres), -1).any(axis=1)
            violation[inside] = 1e3
        # Permit recovery from an existing small boundary overrun only while
        # every next pose reduces it. Never widen the corridor for a new bypass.
        initial = max(0., float(violation[0]))
        allowance = initial * np.clip(1. - sample / 6., 0., 1.) + .03
        hits = np.flatnonzero(violation > allowance)
        return float(sample[hits[0]]) if len(hits) else min(float(horizon_m), float(arc[-1]))
