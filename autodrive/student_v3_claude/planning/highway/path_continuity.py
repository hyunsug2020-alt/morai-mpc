"""Join highway plans to the last published geometry before checking safety.

The previous reference is a steering reference, not a measurement of the
vehicle's steering state. Every joined path still needs the ordinary body,
traffic, raw-scan and map checks, including its retained prefix.
"""
import math

import numpy as np
from scipy.interpolate import CubicSpline


def _curve(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] < 2 or len(points) < 4:
        raise ValueError('invalid highway continuation reference')
    points = points[:, :2]
    if not np.isfinite(points).all():
        raise ValueError('non-finite highway continuation reference')
    points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-6]]
    if len(points) < 4:
        raise ValueError('short highway continuation reference')
    stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    return points, stations


def projected_suffix(points):
    """Project the measured axle onto a forward segment, rather than a vertex."""
    points, _ = _curve(points)
    edges = np.diff(points, axis=0)
    squared = np.sum(edges*edges, axis=1)
    fraction = np.clip(-np.sum(points[:-1]*edges, axis=1)/squared, 0., 1.)
    projected = points[:-1]+fraction[:, None]*edges
    distance = np.sum(projected*projected, axis=1)
    # Reject a wrong branch or a stale reference which is no longer near the
    # measured forward heading. This is only a reference-ownership check.
    usable = edges[:, 0]/np.sqrt(squared) >= math.cos(math.radians(30.))
    distance = np.where(usable, distance, np.inf)
    at = int(np.argmin(distance))
    if distance[at] > 1.5**2:
        raise ValueError('highway continuation reference does not match ego')
    return _curve(np.vstack((projected[at], points[at+1:])))[0]


def published_reference(command, ego, now_monotonic, to_local):
    """Use only a recent, valid forward HIGHWAY publication in the same epoch."""
    try:
        if (not isinstance(command, dict) or command.get('mode') != 'HIGHWAY' or
                command.get('gear') != 'D' or command.get('emergency') is not False):
            return None
        target = float(command.get('target_mps', 0.))
        if not math.isfinite(target) or target <= 0.:
            return None
        validation = command.get('validation') or {}
        if (not isinstance(validation, dict) or any(validation.get(key) is not True for key in
                ('valid', 'pose_valid', 'steering_valid')) or validation.get('blocked')):
            return None
        age = float(now_monotonic)-float(command['received'])
        source = float(command['source_stamp'])
        gap = float(ego['stamp'])-source
        if (not all(math.isfinite(v) for v in (age, source, gap)) or
                source <= 0. or not 0. <= age <= 1. or not 0. <= gap <= 1.):
            return None
        return projected_suffix(to_local(np.array(command['world_path'], copy=True), ego))
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return None


def join_reference(candidate, reference, speed_mps, time_scale=1., *, brisk=False):
    """Keep the recent reference ahead, then blend to a checked candidate.

    0.4 wall seconds covers the recorded 395 ms highway decision; two 0.25 s
    actuator time constants reserve response travel without assuming immediate
    tracking. These are geometry reservations, not a proof of actuator state.
    Quintic blend weights have zero first and second derivatives at both ends.
    The axle/heading correction also returns to the old reference with zero
    first/second derivatives, rather than moving the saved destination each tick.
    """
    old, old_s = _curve(reference)
    new, new_s = _curve(candidate)
    speed = float(speed_mps)
    scale = float(time_scale)
    if not math.isfinite(speed) or speed < 0. or not math.isfinite(scale) or scale <= 0.:
        raise ValueError('invalid highway continuation timing')
    prefix = max(12., speed*(.4*scale+.5))
    # A 35 m blend reversed the recorded turn within a short lateral-accel
    # envelope (65.8 km/h cap at 100.2 km/h). A 90 m blend kept the same
    # measured case reachable at 106.8 km/h; safety still checks its full span.
    blend = max(90., speed*3.)
    if brisk:
        edges = np.diff(old, axis=0)
        lengths = np.linalg.norm(edges, axis=1)
        turns = np.diff(np.unwrap(np.arctan2(edges[:, 1], edges[:, 0])))
        curvature = np.abs(turns)/np.maximum(.5*(lengths[:-1]+lengths[1:]), 1e-6)
        near = old_s[1:-1] <= prefix+blend
        # Starting a pass from a nearly straight reference needs no turn
        # reversal. Preserve the same response prefix, then try a shorter
        # blend. Curved references and every ongoing change keep the long
        # blend; the caller still checks the entire joined path and traffic.
        if len(curvature[near]) and float(curvature[near].max()) <= .0005:
            blend = max(45., speed*3.)
    end = prefix+blend
    if min(old_s[-1], new_s[-1]) < end+3.:
        raise ValueError('highway continuation reference too short for join')
    # Uniform metric spacing also avoids tiny native endpoint gaps in the
    # follower's signed curvature differences. Keep the destination exactly.
    stations = np.linspace(0., new_s[-1], int(math.ceil(new_s[-1]/.5))+1)
    new_xy = np.column_stack([np.interp(stations, new_s, new[:, i]) for i in (0, 1)])
    u = np.clip((stations-prefix)/blend, 0., 1.)
    weight = u**3*(10.-15.*u+6.*u*u)
    # Fit derivatives over physical metres, independent of HD-map vertex
    # spacing. Preserve the previous *reference* curvature; never invent a
    # missing or stale physical wheel measurement as zero steering.
    fit_s = np.linspace(0., 3., 13)
    fit_xy = np.column_stack([np.interp(fit_s, old_s, old[:, i]) for i in (0, 1)])
    coefficients = np.polynomial.polynomial.polyfit(fit_s, fit_xy, 2)
    # Interpolate the old prefix with the same differentiable representation
    # used by the boundary correction. A fit subtracted from a polyline would
    # leave that polyline's different initial tangent in the published path.
    old_curve = CubicSpline(old_s, old, bc_type=((2, 2.*coefficients[2]), (2, np.zeros(2))))
    old_xy = old_curve(np.minimum(stations, old_s[-1]))
    path = old_xy+(new_xy-old_xy)*weight[:, None]
    tangent, second = old_curve(0., 1), old_curve(0., 2)
    norm = float(np.linalg.norm(tangent))
    if norm < .5:
        raise ValueError('invalid highway continuation tangent')
    curvature = float(np.cross(tangent, second))/norm**3
    q = np.clip(stations/prefix, 0., 1.)
    h0 = 1.-q**3*(10.-15.*q+6.*q*q)
    h1 = q-6.*q**3+8.*q**4-3.*q**5
    h2 = .5*q*q*(1.-q)**3
    path += (-old[0]*h0[:, None] +
             prefix*(np.array([1., 0.])-tangent)*h1[:, None] +
             prefix**2*(np.array([0., curvature])-second)*h2[:, None])
    path[0] = 0.
    path[-1] = new[-1]
    active = stations <= end
    if (not np.isfinite(path).all() or
            np.any(np.diff(path[active, 0]) <= 0.)):
        raise ValueError('non-forward highway continuation join')
    return path, dict(applied=True, prefix_m=round(prefix, 3),
                      blend_m=round(blend, 3), reference_curvature_1pm=curvature)
