"""Smooth roadside clearance proposals; existing road/body checks own safety."""
import numpy as np
from scipy.ndimage import gaussian_filter1d


def nudge_geometry(path, stations, demand, speed_mps, max_shift_m=.6):
    """Spread nudge ramp corners and the initial join over travel distance.

    A three-metre ramp corner can undo a gentle native merge at highway speed.
    Spatial smoothing follows 0.8 seconds of travel, and the origin join follows
    three seconds. Close obstacles still require the final raw footprint check;
    this proposal neither changes the road bounds nor proves a safe passage.
    """
    path = np.asarray(path, dtype=float)[:, :2]
    full = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    speed = max(0., float(speed_mps))
    rate = max_shift_m/max(12., speed*2.)
    distance = np.abs(full[:, None]-np.asarray(stations)[None, :])
    demand = np.asarray(demand)
    left = np.max(np.clip(demand, 0., max_shift_m)[None, :]-rate*distance, axis=1).clip(min=0.)
    right = np.max(np.clip(-demand, 0., max_shift_m)[None, :]-rate*distance, axis=1).clip(min=0.)
    shift = left-right
    if len(full) > 4:
        # Uniform distance sampling gives the same profile when native links
        # change their waypoint spacing at a shared endpoint.
        sample = np.arange(0., full[-1]+.5, .5)
        uniform = np.interp(sample, full, shift)
        filtered = gaussian_filter1d(uniform, max(1.5, speed*.8)/.5, mode='nearest')
        shift = np.interp(full, sample, filtered)
    if np.max(np.abs(shift)) < .05:
        return path
    u = np.clip(full/max(12., speed*3.), 0., 1.)
    shift *= u**3*(10.-15.*u+6.*u*u)
    tangent = np.gradient(path, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    return path+normal*shift[:, None]
