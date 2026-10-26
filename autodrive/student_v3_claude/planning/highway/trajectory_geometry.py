"""Rear-axle poses and oriented box support for highway forecasts only."""
import math

import numpy as np


def normalize_path(path):
    """Keep axle stations rooted at the measured pose; remove repeated vertices."""
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] < 2 or len(path) < 2:
        raise ValueError("invalid highway path")
    path = path[:, :2]
    if not np.isfinite(path).all():
        raise ValueError("non-finite highway path")
    path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-9]]
    if len(path) < 2:
        raise ValueError("zero-length highway path")
    # A nearest-map vertex can lie just behind the axle. Prepending the
    # measured origin before it would introduce a reverse segment and a false
    # 180-degree footprint swing. Highway control follows the forward suffix.
    prefix = 1 if np.linalg.norm(path[0]) <= 1e-9 and len(path) > 1 else 0
    if path[prefix, 0] < 0.:
        ahead = np.flatnonzero(path[prefix:, 0] >= 0.) + prefix
        if not len(ahead):
            raise ValueError("highway path has no forward suffix")
        path = path[ahead[0]:]
    if np.linalg.norm(path[0]) > 1e-9:
        path = np.vstack(([0., 0.], path))
    path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-9]]
    if len(path) < 2:
        raise ValueError("zero-length highway path")
    stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    return path, stations


def sample_pose(path, stations, distances, heading_deg, path_angles=None):
    """Sample geometric tangents even after stopping; a standing car cannot rotate."""
    xy = np.column_stack([np.interp(distances, stations, path[:, k]) for k in (0, 1)])
    if path_angles is None:
        tangent = np.gradient(path, stations, axis=0)
        path_angles = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
    yaw = np.interp(distances, stations, path_angles) + math.radians(heading_deg)
    stopped = np.asarray(distances) <= 1e-9
    xy[stopped] = 0.
    yaw[stopped] = math.radians(heading_deg)
    return xy, yaw


def projected_halves(length, width, relative_yaw):
    """OBB support along and across the reference tangent, without shrinking boxes."""
    cosine, sine = np.abs(np.cos(relative_yaw)), np.abs(np.sin(relative_yaw))
    return (length*.5*cosine + width*.5*sine,
            length*.5*sine + width*.5*cosine)
