"""Round highway link joins while retaining their surveyed road envelope."""
import numpy as np
from scipy.ndimage import gaussian_filter1d

# Lateral acceleration at which seam-tracking repairs were validated.
REPAIR_CURVE_ALAT_MPS2 = 3.5


def gentle_merge_geometry(points, merge_xy, max_shift_m=.6, sigma_m=9.):
    """Spread a native merge corner over distance, before cutting at the ego.

    This proposes control geometry only. Road legality, raw returns, moving
    bodies and steering still validate the final path against surveyed geometry.
    Other road sections and the chain endpoints retain their original points.
    """
    points = np.asarray(points, dtype=float)[:, :2]
    if len(points) < 5:
        return points.copy()
    station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    unique, keep = np.unique(station, return_index=True)
    if len(unique) < 5:
        return points.copy()
    sample = np.arange(0., unique[-1]+.5, .5)
    uniform = np.column_stack([np.interp(sample, unique, points[keep, axis]) for axis in (0, 1)])
    radius = int(np.ceil(4.*sigma_m/.5))
    before = uniform[0]-np.arange(radius, 0, -1)[:, None]*(uniform[1]-uniform[0])
    after = uniform[-1]+np.arange(1, radius+1)[:, None]*(uniform[-1]-uniform[-2])
    extended = np.vstack((before, uniform, after))
    filtered = gaussian_filter1d(extended, sigma_m/.5, axis=0, mode='nearest')[radius:-radius]
    candidate = np.column_stack([np.interp(station, sample, filtered[:, axis]) for axis in (0, 1)])
    delta = candidate-points
    norm = np.linalg.norm(delta, axis=1)
    # If this join needs more space, retain its original checked geometry.
    # Pointwise clipping a large smoothing displacement introduces new corners.
    at = int(np.argmin(np.linalg.norm(points-np.asarray(merge_xy), axis=1)))
    distance = abs(station-station[at])
    u = np.clip((distance-25.)/20., 0., 1.)
    weight = 1.-u**3*(10.-15.*u+6.*u*u)
    scale = min(1., max_shift_m/max(1e-9, float(np.max(norm*weight))))
    result = points+delta*(weight*scale)[:, None]
    result[[0, -1]] = points[[0, -1]]
    return result


def continue_merge_geometry(surveyed, proposed, previous_surveyed, previous_control,
                            merge_xy, max_shift_m=.6):
    """Carry a recently followed merge displacement across a native seed join.

    The new surveyed road remains the reference. Only the checked control
    displacement is transported, and it rejoins the new proposal by 45 m.
    A different branch without a nearby common envelope is left unchanged.
    """
    from scipy.spatial import cKDTree
    surveyed, proposed = np.asarray(surveyed), np.asarray(proposed)
    previous_surveyed, previous_control = np.asarray(previous_surveyed), np.asarray(previous_control)
    stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(previous_surveyed, axis=0), axis=1))]
    distance, at = cKDTree(previous_surveyed).query(surveyed)
    tangent = np.gradient(previous_surveyed, axis=0)
    tangent /= np.maximum(1e-9, np.linalg.norm(tangent, axis=1, keepdims=True))
    along = stations[at]+np.sum((surveyed-previous_surveyed[at])*tangent[at], axis=1)
    merge = stations[int(np.argmin(np.linalg.norm(previous_surveyed-merge_xy, axis=1)))]
    relative = along-merge
    relevant = relative < 45.
    if not relevant.any() or np.any(distance[relevant] > .5):
        return proposed.copy()
    # Continue the actual world curve, not only its old displacement: successor
    # links can sample the same surveyed join differently at their endpoints.
    carried = np.column_stack([np.interp(along, stations, previous_control[:, axis])
                               for axis in (0, 1)])-surveyed
    u = np.clip((relative-25.)/20., 0., 1.)
    weight = 1.-u**3*(10.-15.*u+6.*u*u)
    displacement = carried*weight[:, None]+(proposed-surveyed)*(1.-weight[:, None])
    # One scale preserves smoothness; pointwise clipping would create corners.
    displacement *= min(1., max_shift_m/max(1e-9, float(np.linalg.norm(displacement, axis=1).max())))
    return surveyed+displacement


def recover_merge_tracking(surveyed, proposed, speed_mps, clearance, *,
                           horizon_m=170., max_shift_m=.6, max_recovery_m=.08,
                           merge_remaining_m=0.):
    """Repair a small merge tracking overrun without changing the road bounds.

    The native corner rounding and the ego's heading join can put a body
    corner a few centimetres beyond the checked margin near a common entry.
    Test a bounded inward correction on the joined control path, then return
    its centre proposal only if the original corridor checks the full path.
    Traffic and raw-obstacle validation still belong to the highway planner.
    """
    from highway_planner import lane_change_path, geometry_speed_cap, path_curvature

    surveyed = np.asarray(surveyed, dtype=float)[:, :2]
    proposed = np.asarray(proposed, dtype=float)[:, :2]
    if surveyed.shape != proposed.shape or len(proposed) < 5:
        return proposed.copy()
    try:
        speed_mps = float(speed_mps)
    except (TypeError, ValueError, OverflowError):
        return proposed.copy()
    if not np.isfinite(speed_mps) or speed_mps < 0.:
        return proposed.copy()
    station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(proposed, axis=0), axis=1))]
    transition = max(12., max(0., float(speed_mps))*2.5)

    def joined(centre):
        return lane_change_path(centre, 0., transition, match_heading=True)

    original = joined(proposed)
    required = min(float(horizon_m), float(np.linalg.norm(np.diff(original, axis=0), axis=1).sum()))
    # Inspect the upcoming seam before reaching it. A local-only 12 m query
    # missed its 3 cm margin violation at 21--31 m in the latest recorded run.
    # Clear candidates skip every extra full-horizon repair search.
    check_distance = min(required, max(12., float(merge_remaining_m)+3.))
    clear = float(clearance(original, check_distance))
    if required < 12. or clear >= check_distance-.01:
        return proposed.copy()
    tangent = np.gradient(proposed, axis=0)
    tangent /= np.maximum(1e-9, np.linalg.norm(tangent, axis=1, keepdims=True))
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))

    def smooth(value):
        u = np.clip(value, 0., 1.)
        return u**3*(10.-15.*u+6.*u*u)

    # Match position and heading at the origin; return to the original curve
    # gradually. A uniform bound prevents accumulated seed-handoff shifts.
    # An earlier boundary hit permits a longer, gentler correction. Keeping
    # every repair at six metres needlessly rejected the otherwise safe
    # recorded entry at 78.8 km/h because its curve cap fell below 70 km/h.
    recovery_span = float(np.clip(3.*clear, 6., 12.))
    weight = smooth(station/recovery_span)*(1.-smooth((station-10.)/35.))
    native_delta = surveyed-proposed
    inward = float(np.mean(np.sum(native_delta[station <= 12.]*normal[station <= 12.], axis=1)))
    directions = (np.sign(inward) or 1., -(np.sign(inward) or 1.))
    # Repairs are judged at the 3.5 m/s² they were validated with against the
    # recorded seam stops; the highway speed caps themselves use 4.0 m/s².
    # A car already below 70 km/h may use a repair that supports its current
    # speed. Requiring 70 km/h rejected a clear 69.16 km/h proposal at 61.1
    # km/h and left a mapped boundary only 7.75 m ahead. Faster approaches
    # retain the original floor; the final curve and traffic speed plans still
    # validate the selected path before it reaches the controller.
    cap_floor = min(70./3.6, speed_mps, geometry_speed_cap(original, 120./3.6,
                                                           lateral_accel=REPAIR_CURVE_ALAT_MPS2))
    steering_limit = np.tan(np.radians(35.))/3.9
    for distance in (.02, .04, .06, .08):
        if distance > max_recovery_m+1e-9:
            break
        for direction in directions:
            candidate = proposed+normal*(direction*distance*weight)[:, None]
            if np.max(np.linalg.norm(candidate-surveyed, axis=1)) > max_shift_m+1e-6:
                continue
            path = joined(candidate)
            if (float(clearance(path, horizon_m)) < required-.01 or
                    np.max(path_curvature(path)) > steering_limit or
                    geometry_speed_cap(path, 120./3.6,
                                       lateral_accel=REPAIR_CURVE_ALAT_MPS2) < cap_floor-1e-6):
                continue
            return candidate
    return proposed.copy()
