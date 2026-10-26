"""The ego body in the vehicle frame, for every footprint built from a pose.

MORAI's ego pose (ego_status x/y, the ESKF pose, the origin of every vehicle-frame
path, scan and object) is the REAR AXLE, not the body centre. The 4.635 x 1.892 m
body reaches EGO_REAR_M behind it and EGO_FRONT_M ahead of it (judge.py), so its
centre lies EGO_CENTRE_M = 1.5275 m ahead of the pose.

Checks that placed a symmetric +-2.3175 m box on the pose (or on each path sample)
checked a body 1.53 m behind the real one: the real front bumper reached obstacles
1.53 m before the check did, and objects 0.8-2.3 m behind the rear axle "touched" a
rear that is not there (follow-20261010T210040Z: 5.5 min parked beside a wreck the
real body cleared by 1.06 m; follow-20261010T211143Z: a bollard read 2.0 m from the
bumper at a real 0.4 m). Its front corners on curves and in reverse also swing
EGO_FRONT_M, not 2.3 m, from the rear axle on the path.

A footprint at a path sample is the box of half extents (EGO_HALF_LENGTH_M,
EGO_HALF_WIDTH_M) centred EGO_CENTRE_M ahead of the sample along the path heading
there: the rear axle follows the path and the body points along its tangent.

Scope (2026-10-11): the tunnel planner, its final/live checks, coverage, no-go,
walls and the reverse escape use REAR_AXLE_SHIFT_M. The shared checks also used on
the normal road and highway (swept_clearance, static_clear_distance, LaneCorridor,
the lattice, PathSafety) take the shift as a parameter whose default,
ROAD_LEGACY_SHIFT_M, keeps the box centred on the pose there: the same 1.53 m error,
but road/highway behaviour (pedestrian departure caps, lane-edge joins, slalom and
barrel bypasses) was tuned and replay-tested with it; fixing it needs its own retune.
"""
import math
import os

import numpy as np

EGO_FRONT_M = float(os.environ.get("STUDENT_EGO_FRONT_M", "3.845"))   # rear axle -> front bumper
EGO_REAR_M = float(os.environ.get("STUDENT_EGO_REAR_M", "0.79"))      # rear axle -> rear bumper
EGO_HALF_WIDTH_M = .946
EGO_HALF_LENGTH_M = .5 * (EGO_FRONT_M + EGO_REAR_M)                   # 2.3175
EGO_CENTRE_M = .5 * (EGO_FRONT_M - EGO_REAR_M)                        # body centre ahead of the rear axle
# Box centre ahead of the pose / path sample in the checks that take body_shift_m.
REAR_AXLE_SHIFT_M = EGO_CENTRE_M   # the truth: the pose is the rear axle (tunnel)
ROAD_LEGACY_SHIFT_M = 0.           # road/highway: box centred on the pose (known 1.53 m error)
# Ioniq5 wheelbase and road-wheel lock (lattice_planner.LOW_SPEED_MAX_CURVATURE keeps
# a margin below tan(35 deg) / 3.0 = 0.233 1/m).
EGO_MAX_CURVATURE = math.tan(math.radians(35.)) / 3.0


def body_centres(samples, tangent, shift_m=REAR_AXLE_SHIFT_M):
    """Body centre for each rear-axle sample with its heading (unit tangent)."""
    return np.asarray(samples, dtype=float) + float(shift_m) * np.asarray(tangent, dtype=float)


def body_extent(points, margin=0., lateral_margin=None):
    """(along, lateral) excess (>= 0) of vehicle-frame points beyond the body at the pose."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    lateral_margin = margin if lateral_margin is None else lateral_margin
    along = np.maximum(np.maximum(p[:, 0] - (EGO_FRONT_M + margin), -(EGO_REAR_M + margin) - p[:, 0]), 0.)
    across = np.maximum(np.abs(p[:, 1]) - (EGO_HALF_WIDTH_M + lateral_margin), 0.)
    return along, across


def inside_body(points, margin=0., lateral_margin=None):
    """Vehicle-frame points inside the body (+margin) at the pose."""
    along, across = body_extent(points, margin, lateral_margin)
    return (along <= 0.) & (across <= 0.)


def body_gap(points):
    """Smallest distance (m) from the body at the pose to vehicle-frame points (inf if none)."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(p):
        return math.inf
    along, across = body_extent(p)
    return float(np.min(np.hypot(along, across)))


def self_return_mask(points, margin=.05):
    """True for returns outside the vehicle's own body/housing (keep them).

    Only the body from the rear bumper to the old 2.3675 m mask edge is masked:
    every self return seen in recorded scans (the roof around the VLP16) lies there,
    and a return inside the front of the body is a contact, never masked. The rear
    zone 0.84-2.37 m behind the rear axle was masked before although it lies
    behind the real rear bumper (reverse checks could not see it)."""
    p = np.asarray(points)
    return ((p[:, 0] < -(EGO_REAR_M + margin)) | (p[:, 0] > EGO_HALF_LENGTH_M + margin) |
            (np.abs(p[:, 1]) > EGO_HALF_WIDTH_M + margin))


def body_corners(x, y, heading_rad, margin=0., lateral_margin=None):
    """World corners (4 x 2, counter-clockwise) of the body at a rear-axle pose."""
    lateral_margin = margin if lateral_margin is None else lateral_margin
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    front, rear = EGO_FRONT_M + margin, -(EGO_REAR_M + margin)
    half = EGO_HALF_WIDTH_M + lateral_margin
    local = np.array([(front, half), (rear, half), (rear, -half), (front, -half)])
    return local @ np.array([[c, s], [-s, c]]) + [x, y]


REVERSE_TURN_RATES = (1., .5, .25)   # fractions of EGO_MAX_CURVATURE the follower may turn at


def reverse_poses(heading_error_rad, distance_m, step_m=.25, curvature=EGO_MAX_CURVATURE):
    """Rear-axle poses (x, y, yaw), vehicle frame, of a reverse that aligns the body
    with the road heading.

    The published reverse runs straight back from the rear axle along the road
    heading (gui.publish_reverse); heading_error_rad = road heading - body heading.
    The follower turns the body toward it while backing, which swings the front
    corners EGO_FRONT_M ahead of the turning rear axle sideways (follow-20261010T200235Z
    05:09:01-05:09:06: 24 deg error, the front-left corner reached a drum 0.75 m beside
    it). Poses: straight back without turning; the body turning toward the road
    heading from the start at each REVERSE_TURN_RATES fraction of the steering limit,
    then straight back; and the published line at the road heading (the end state)
    from where the fastest turn is complete."""
    distance = max(0., float(distance_m))
    d = np.arange(0., distance + 1e-9, step_m)
    if not len(d) or d[-1] < distance - 1e-9:
        d = np.r_[d, distance]
    poses = [np.column_stack((-d, np.zeros(len(d)), np.zeros(len(d))))]
    error = float(heading_error_rad)
    if abs(error) > 1e-3:
        for rate in REVERSE_TURN_RATES:
            # Backing with steering: the yaw changes by the curvature per metre travelled.
            yaw = np.sign(error) * np.minimum(abs(error), d * curvature * rate)
            dx, dy = -np.cos(yaw[:-1]) * np.diff(d), -np.sin(yaw[:-1]) * np.diff(d)
            poses.append(np.column_stack((np.r_[0., np.cumsum(dx)], np.r_[0., np.cumsum(dy)], yaw)))
        # The line itself only once the turn can be complete (no turning in place).
        on_line = d[d >= min(distance, abs(error) / curvature)]
        poses.append(np.column_stack((-on_line * math.cos(error), -on_line * math.sin(error),
                                      np.full(len(on_line), error))))
    return np.vstack(poses)


def reverse_sweep_hits(points, heading_error_rad, distance_m, margin=.15, contact_m=.03, step_m=.25):
    """Vehicle-frame points a reverse (reverse_poses) would bring the body to, as a bool mask.

    A point counts when the body (+margin) reaches it at some pose although it lies
    outside that band now, or when the body itself (+contact_m) reaches it: returns
    already inside the margin band beside a standing car (a wall, a parked car
    alongside) block a reverse only if the motion actually touches them."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    hit = np.zeros(len(p), dtype=bool)
    if not len(p) or not np.isfinite(float(distance_m)) or float(distance_m) <= 0.:
        return hit
    reach = math.hypot(EGO_FRONT_M + margin, EGO_HALF_WIDTH_M + margin) + float(distance_m) + .5
    near = np.flatnonzero(np.isfinite(p).all(axis=1) & (np.linalg.norm(p, axis=1) <= reach))
    if not len(near):
        return hit
    q = p[near]
    poses = reverse_poses(heading_error_rad, distance_m, step_m)
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    dx, dy = q[None, :, 0] - poses[:, 0, None], q[None, :, 1] - poses[:, 1, None]
    along, across = c * dx + s * dy, np.abs(-s * dx + c * dy)

    def inside(extra):
        return ((along <= EGO_FRONT_M + extra) & (along >= -(EGO_REAR_M + extra)) &
                (across <= EGO_HALF_WIDTH_M + extra))
    banded = inside(margin).any(axis=0) & ~inside_body(q, margin)
    hit[near] = banded | inside(contact_m).any(axis=0)
    return hit
