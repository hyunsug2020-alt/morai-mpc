"""Highway-only footprints rooted at the measured rear axle.

Shared road and tunnel checkers keep their own defaults. The highway's raw,
boxed and mapped checks use the same actual pose and path-station tangents as
its timed traffic forecast, without a rear-object discard exception.
"""
import math

import numpy as np
from scipy.spatial import cKDTree

from ego_body import (EGO_CENTRE_M, EGO_FRONT_M, EGO_REAR_M,
                      EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M)
from lane_corridor import LaneCorridor, points_in_polygon
from .trajectory_geometry import normalize_path, sample_pose


HIGHWAY_NUDGE_MAX_M = .6
HIGHWAY_NUDGE_WANT_M = .6
HIGHWAY_SIDE_GAP_KMH = ((.2, 30.), (.45, 50.), (.75, 70.))
CORRIDOR_SIDE_MARGIN_M = .2


def hit_clearance(station, end):
    """Keep a sampled endpoint hit distinct from a completed clear query.

    The legacy float API uses the exact query distance for completion. Its
    consumer has a 1e-6 m tolerance, so an actual hit keeps a conservative
    1e-5 m separation from that sentinel; bumper offsets are already applied.
    """
    return max(0., min(float(station), float(end)-1e-5))


def sample_body_poses(path, horizon_m, include_current=True):
    """Quarter-metre axle samples, including the query endpoint and actual t0."""
    if include_current:
        path, stations = normalize_path(path)
    else:
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[1] < 2 or len(path) < 2:
            raise ValueError('invalid highway corridor tail')
        path = path[:, :2]
        if not np.isfinite(path).all():
            raise ValueError('non-finite highway corridor tail')
        path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-9]]
        if len(path) < 2:
            raise ValueError('zero-length highway corridor tail')
        stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    horizon_m = float(horizon_m)
    if not math.isfinite(horizon_m) or horizon_m <= 0.:
        raise ValueError('invalid highway body horizon')
    end = min(horizon_m, float(stations[-1]))
    sample = np.r_[np.arange(0., end, .25), end]
    if len(sample) < 2:
        raise ValueError('short highway body path')
    if include_current:
        axle, yaw = sample_pose(path, stations, sample, 0.)
    else:
        axle = np.column_stack([np.interp(sample, stations, path[:, k]) for k in (0, 1)])
        tangent = np.gradient(path, stations, axis=0)
        yaw = np.interp(sample, stations, np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0])))
    tangent = np.column_stack((np.cos(yaw), np.sin(yaw)))
    return sample, axle, tangent, end


def polygon_body_overlap(rectangles, polygon):
    """Inclusive rectangle/non-convex polygon contact, with a cheap bounds gate."""
    rectangles = np.asarray(rectangles, dtype=float)
    polygon = np.asarray(polygon, dtype=float)
    if rectangles.ndim != 3 or rectangles.shape[1:] != (4, 2):
        raise ValueError('invalid highway body rectangles')
    hit = np.zeros(len(rectangles), dtype=bool)
    if not np.isfinite(rectangles).all():
        return np.ones(len(rectangles), dtype=bool)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3 or not np.isfinite(polygon).all():
        return np.ones(len(rectangles), dtype=bool)
    tolerance = 1e-12
    lower, upper = rectangles.min(axis=1), rectangles.max(axis=1)
    polygon_lower, polygon_upper = polygon.min(axis=0), polygon.max(axis=0)
    indices = np.flatnonzero(np.all(upper >= polygon_lower-tolerance, axis=1) &
                             np.all(lower <= polygon_upper+tolerance, axis=1))
    if not len(indices):
        return hit
    bodies = rectangles[indices]
    inside = points_in_polygon(bodies.reshape(-1, 2), polygon).reshape(len(bodies), 4).any(axis=1)
    # Polygon vertices inside the body cover a small zone enclosed by the car.
    a, b = bodies[:, 1]-bodies[:, 0], bodies[:, 3]-bodies[:, 0]
    delta = polygon[None, :, :]-bodies[:, :1, :]
    along = np.sum(delta*a[:, None, :], axis=2)/np.sum(a*a, axis=1)[:, None]
    across = np.sum(delta*b[:, None, :], axis=2)/np.sum(b*b, axis=1)[:, None]
    inside |= ((along >= -tolerance) & (along <= 1.+tolerance) &
               (across >= -tolerance) & (across <= 1.+tolerance)).any(axis=1)
    # Inclusive segment intersection also catches edge-only crossings and touch.
    start = bodies[:, :, None, :]
    end = np.roll(bodies, -1, axis=1)[:, :, None, :]
    other_start = polygon[None, None, :, :]
    other_end = np.roll(polygon, -1, axis=0)[None, None, :, :]
    edge, other_edge = end-start, other_end-other_start
    def cross(x, y):
        return x[..., 0]*y[..., 1]-x[..., 1]*y[..., 0]
    first = cross(edge, other_start-start)
    second = cross(edge, other_end-start)
    third = cross(other_edge, start-other_start)
    fourth = cross(other_edge, end-other_start)
    straddle = (((first <= tolerance) & (second >= -tolerance)) |
                ((first >= -tolerance) & (second <= tolerance)))
    straddle &= (((third <= tolerance) & (fourth >= -tolerance)) |
                 ((third >= -tolerance) & (fourth <= tolerance)))
    bounds = np.all(np.maximum(start, end) >= np.minimum(other_start, other_end)-tolerance, axis=-1)
    bounds &= np.all(np.minimum(start, end) <= np.maximum(other_start, other_end)+tolerance, axis=-1)
    inside |= (straddle & bounds).any(axis=(1, 2))
    hit[indices] = inside
    return hit


def current_box_contact(objects, ego):
    """Current static-box SAT witness, including the shared checker's margins."""
    try:
        pose = np.asarray([ego['x'], ego['y'], ego['heading_deg']], dtype=float)
        if not np.isfinite(pose).all():
            return dict(reason='invalid highway current pose')
    except (KeyError, TypeError, ValueError, OverflowError):
        return dict(reason='invalid highway current pose')
    yaw = math.radians(pose[2])
    tangent = np.array([math.cos(yaw), math.sin(yaw)])
    normal = np.array([-tangent[1], tangent[0]])
    centre = pose[:2] + EGO_CENTRE_M*tangent
    for obj in objects:
        try:
            values = np.asarray([obj[0], obj[1],
                obj[6] if len(obj) > 6 else obj[2],
                obj[7] if len(obj) > 7 else obj[2],
                obj[8] if len(obj) > 8 else pose[2]], dtype=float)
            if not np.isfinite(values).all() or min(values[2:4]) <= 0.:
                return dict(reason='invalid highway current obstacle footprint')
            other_yaw = math.radians(values[4])
            other_tangent = np.array([math.cos(other_yaw), math.sin(other_yaw)])
            other_normal = np.array([-other_tangent[1], other_tangent[0]])
            delta = values[:2]-centre
            margin = .65 if obj[4] == 'pedestrian' else .2
            separating_gap = -float('inf')
            for axis in (tangent, normal, other_tangent, other_normal):
                reach = (EGO_HALF_LENGTH_M*abs(float(tangent @ axis)) +
                         EGO_HALF_WIDTH_M*abs(float(normal @ axis)) +
                         max(.3, values[2])/2.*abs(float(other_tangent @ axis)) +
                         max(.3, values[3])/2.*abs(float(other_normal @ axis)))
                separating_gap = max(separating_gap, abs(float(delta @ axis))-reach-margin)
            if separating_gap <= 0.:
                return dict(object_id=obj[11] if len(obj) > 11 else None,
                            name=obj[3], when=0.,
                            separating_axis_gap_m=separating_gap,
                            reason='static box touches current rear-axle body')
        except (TypeError, ValueError, OverflowError, IndexError):
            return dict(reason='invalid highway current obstacle footprint')
    return None


def highway_boxed_clearance(path, ego, objects, horizon_m):
    """Full static OBB sweep; every rear box keeps its current and future SAT."""
    objects = list(objects)
    if current_box_contact(objects, ego) is not None:
        return 0.
    try:
        sample, axle, tangent, end = sample_body_poses(path, horizon_m)
    except (TypeError, ValueError, IndexError, OverflowError):
        return 0.
    if not objects:
        return end
    pose = np.asarray([ego['x'], ego['y'], ego['heading_deg']], dtype=float)
    yaw = math.radians(pose[2])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    positions, axes, lengths, widths, margins = [], [], [], [], []
    for obj in objects:
        positions.append((np.asarray(obj[:2], dtype=float)-pose[:2]) @ rotation)
        angle = math.radians(float(obj[8] if len(obj) > 8 else pose[2])-pose[2])
        axes.append((math.cos(angle), math.sin(angle)))
        lengths.append(max(.3, float(obj[6] if len(obj) > 6 else obj[2]))/2.)
        widths.append(max(.3, float(obj[7] if len(obj) > 7 else obj[2]))/2.)
        margins.append(.65 if obj[4] == 'pedestrian' else .2)
    positions, axes = np.asarray(positions), np.asarray(axes)
    cross = np.column_stack((-axes[:, 1], axes[:, 0]))
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    centres = axle+EGO_CENTRE_M*tangent
    delta = positions[:, None, :]-centres[None, :, :]
    lengths, widths, margins = np.asarray(lengths)[:, None], np.asarray(widths)[:, None], np.asarray(margins)[:, None]
    overlap = np.ones((len(objects), len(sample)), dtype=bool)
    for axis in (tangent[None, :, :], normal[None, :, :], axes[:, None, :], cross[:, None, :]):
        reach = (EGO_HALF_LENGTH_M*np.abs(np.sum(tangent[None, :, :]*axis, axis=2)) +
                 EGO_HALF_WIDTH_M*np.abs(np.sum(normal[None, :, :]*axis, axis=2)) +
                 lengths*np.abs(np.sum(axes[:, None, :]*axis, axis=2)) +
                 widths*np.abs(np.sum(cross[:, None, :]*axis, axis=2)))
        overlap &= np.abs(np.sum(delta*axis, axis=2)) <= reach+margins
    hit = np.flatnonzero(overlap.any(axis=0))
    return hit_clearance(sample[hit[0]], end) if len(hit) else end


def highway_raw_clearance(path, points, horizon_m, extra_margin=.25,
                          tree=None, lateral_margin=None):
    """KD-bounded full rectangle sweep using the same highway body poses."""
    if points is None:
        return 0.
    try:
        sample, axle, tangent, end = sample_body_poses(path, horizon_m)
        points = np.asarray(points, dtype=float).reshape(-1, 2)
    except (TypeError, ValueError, IndexError, OverflowError):
        return 0.
    if not np.isfinite(points).all():
        return 0.
    try:
        extra_margin = float(extra_margin)
        side = extra_margin if lateral_margin is None else float(lateral_margin)
        if not math.isfinite(extra_margin) or not math.isfinite(side) or min(extra_margin, side) < 0.:
            return 0.
    except (TypeError, ValueError, OverflowError):
        return 0.
    inside = ((points[:, 0] >= -EGO_REAR_M-extra_margin) &
              (points[:, 0] <= EGO_FRONT_M+extra_margin) &
              (np.abs(points[:, 1]) <= EGO_HALF_WIDTH_M+side))
    if inside.any():
        return 0.
    if not len(points):
        return end
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    centres = axle+EGO_CENTRE_M*tangent
    tree = tree if tree is not None else cKDTree(points)
    radius = math.hypot(EGO_HALF_LENGTH_M+extra_margin, EGO_HALF_WIDTH_M+side)
    nearest, _ = tree.query(centres, k=1, distance_upper_bound=radius+1e-9)
    occupied = np.flatnonzero(np.isfinite(nearest))
    if not len(occupied):
        return end
    nearby = tree.query_ball_point(centres[occupied], radius+1e-9, return_sorted=False)
    counts = np.fromiter((len(ids) for ids in nearby), dtype=int, count=len(nearby))
    if counts.sum():
        station = np.repeat(occupied, counts)
        ids = np.concatenate([ids for ids in nearby if len(ids)])
        delta = points[ids]-centres[station]
        along = np.sum(delta*tangent[station], axis=1)
        across = np.sum(delta*normal[station], axis=1)
        # Inclusive contact must survive subtraction roundoff at exact decimal
        # bumper coordinates. This numerical tolerance is far below the margin.
        hit = np.flatnonzero((np.abs(along) <= EGO_HALF_LENGTH_M+extra_margin+1e-10) &
                             (np.abs(across) <= EGO_HALF_WIDTH_M+side+1e-10))
        if len(hit):
            return hit_clearance(sample[station[hit[0]]], end)
    return end


class HighwayLaneCorridor(LaneCorridor):
    """Actual highway body, with the existing shrinking boundary allowance.

    A tail checked by inward recovery is a future segment, not a new measured
    ego pose. Callers select ``include_current=False`` for that explicit case.
    """
    def __init__(self, route, left, right, zones=None, intersection_mask=None,
                 body_shift_m=EGO_CENTRE_M):
        super().__init__(route, left, right, zones, intersection_mask,
                         body_shift_m=body_shift_m)

    @classmethod
    def from_corridor(cls, corridor):
        """Clone ownership without changing shared geometry or its body shift."""
        if not isinstance(corridor, LaneCorridor):
            return None
        clone = cls.__new__(cls)
        clone.__dict__ = corridor.__dict__.copy()
        clone.body_shift_m = EGO_CENTRE_M
        return clone

    def _overrun(self, path, horizon_m, include_current):
        try:
            sample, centres, tangent, end = sample_body_poses(path, horizon_m, include_current)
            normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
            side = EGO_HALF_WIDTH_M+CORRIDOR_SIDE_MARGIN_M
            shift = self.body_shift_m
            corners = np.stack([centres+a*tangent+b*normal
                for a in (shift-EGO_HALF_LENGTH_M, shift, shift+EGO_HALF_LENGTH_M)
                for b in (-side, side)], axis=1)
            flat = corners.reshape(-1, 2)
            _, index = self.tree.query(flat)
            lateral = np.sum((flat-self.route[index])*self.normal[index], axis=1)
            overrun = np.maximum(lateral-self.left[index], self.right[index]-lateral)
            overrun = overrun.reshape(len(sample), -1).max(axis=1)
            zones = np.zeros(len(sample), dtype=bool)
            body_centres = centres+shift*tangent
            rectangles = np.stack([body_centres+a*tangent+b*normal
                for a, b in ((EGO_HALF_LENGTH_M, side), (-EGO_HALF_LENGTH_M, side),
                             (-EGO_HALF_LENGTH_M, -side), (EGO_HALF_LENGTH_M, -side))], axis=1)
            for zone in self.zones:
                zones |= polygon_body_overlap(rectangles, zone)
            overrun[zones] = 1e3
            if not np.isfinite(overrun).all():
                raise ValueError('non-finite highway corridor boundary')
            return sample, overrun, zones, end
        except (TypeError, ValueError, IndexError, OverflowError):
            return np.array([0.]), np.array([float('inf')]), np.array([True]), 0.

    def sample_overrun(self, path, horizon_m, include_current=True):
        sample, overrun, _, _ = self._overrun(path, horizon_m, include_current)
        return sample, overrun

    def clear_distance(self, path, horizon_m=60., include_current=True):
        sample, overrun, zones, end = self._overrun(path, horizon_m, include_current)
        initial = max(0., float(overrun[0]))
        allowance = initial*np.clip(1.-sample/6., 0., 1.)+.03
        # No-go polygons never inherit the lane-edge recovery allowance.
        hit = np.flatnonzero(zones | (overrun > allowance))
        return hit_clearance(sample[hit[0]], end) if len(hit) else end


def highway_side_gaps(path, points, tree, horizon=90.):
    """Measured side gaps around actual body centres, preserving legacy limits."""
    try:
        path, arc = normalize_path(path)
    except (TypeError, ValueError, IndexError):
        return np.array([0.]), np.array([-float('inf')]), np.array([-float('inf')])
    n = int(np.searchsorted(arc, horizon))
    left, right = np.full(n, 2.), np.full(n, 2.)
    if points is None or tree is None or not len(points) or n < 2:
        return arc[:n], left, right
    _, yaw = sample_pose(path, arc, arc, 0.)
    tangent = np.column_stack((np.cos(yaw), np.sin(yaw)))
    centres = path[:n]+EGO_CENTRE_M*tangent[:n]
    for i, near in enumerate(tree.query_ball_point(centres, 3.4)):
        if not near:
            continue
        relative = np.asarray(points)[near, :2]-centres[i]
        along = relative @ tangent[i]
        lateral = relative @ np.array([-tangent[i, 1], tangent[i, 0]])
        beside = np.abs(along) < 2.4
        if np.any(beside & (lateral > .3)):
            left[i] = min(2., float(lateral[beside & (lateral > .3)].min())-EGO_HALF_WIDTH_M)
        if np.any(beside & (lateral < -.3)):
            right[i] = min(2., float(-lateral[beside & (lateral < -.3)].max())-EGO_HALF_WIDTH_M)
    return arc[:n], left, right


def highway_nudge(path, points, tree, speed_mps, horizon=90.):
    """Keep the existing bounded nudge proposal, using actual-body side gaps."""
    from .nudge_geometry import nudge_geometry
    try:
        path, _ = normalize_path(path)
    except (TypeError, ValueError, IndexError):
        return path
    arc, left, right = highway_side_gaps(path, points, tree, horizon)
    if not np.isfinite(left).all() or not np.isfinite(right).all():
        return path
    tight = np.flatnonzero(np.minimum(left, right) < HIGHWAY_NUDGE_WANT_M)
    if not len(tight):
        return path
    want = np.clip((left[tight]-right[tight])/2., -HIGHWAY_NUDGE_MAX_M, HIGHWAY_NUDGE_MAX_M)
    keep = np.abs(want) >= .05
    if not keep.any():
        return path
    return nudge_geometry(path, arc[tight][keep], want[keep], speed_mps, HIGHWAY_NUDGE_MAX_M)


def highway_side_gap_cap(path, points, tree, horizon=90., decel=3.):
    """Preserve the existing side-gap speeds and approach deceleration."""
    arc, left, right = highway_side_gaps(path, points, tree, horizon)
    if not np.isfinite(left).all() or not np.isfinite(right).all():
        return 0.
    gap, cap = np.minimum(left, right), float('inf')
    for limit, kmh in HIGHWAY_SIDE_GAP_KMH:
        hit = np.flatnonzero(gap < limit)
        if len(hit):
            cap = min(cap, math.sqrt((kmh/3.6)**2 + 2.*decel*max(0., float(arc[hit[0]])-3.)))
    return cap
