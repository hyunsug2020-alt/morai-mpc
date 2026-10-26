"""Small tunnel path proposals from current-frame measured free intervals.

These intervals rank and prune proposals. They do not certify a path: the
lattice planner still checks the complete swept body, walls, curvature and
moving-object forecast before selecting or reusing any proposal.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree

from ego_body import EGO_FRONT_M, EGO_HALF_WIDTH_M, EGO_REAR_M


def _arc(points):
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]


def _subtract(lower, upper, occupied):
    """Free centre intervals after merging inflated measured returns."""
    if upper <= lower:
        return []
    result, cursor = [], float(lower)
    for lo, hi in sorted(occupied):
        if hi <= cursor or lo >= upper:
            continue
        if lo > cursor:
            result.append((cursor, min(float(lo), float(upper))))
        cursor = max(cursor, float(hi))
        if cursor >= upper:
            break
    if cursor < upper:
        result.append((cursor, float(upper)))
    return result


@dataclass
class _PassageQuality:
    route: np.ndarray
    stations: np.ndarray
    tangent: np.ndarray
    normal: np.ndarray
    sample: np.ndarray
    intervals: list
    body_clearance: float

    def __post_init__(self):
        self.tree = cKDTree(self.route)

    def __call__(self, path, horizon):
        """Minimum balanced passage width along the proposed current-frame path."""
        path = np.asarray(path, dtype=float)[:, :2]
        if len(path) < 3 or not np.isfinite(path).all():
            return 0.
        travel = _arc(path)
        _, at = self.tree.query(path)
        along = self.stations[at] + np.sum((path-self.route[at])*self.tangent[at], axis=1)
        lateral = np.sum((path-self.route[at])*self.normal[at], axis=1)
        selected = (travel <= horizon) & (along >= 3.) & (along <= self.sample[-1])
        if not selected.any():
            return 0.
        row = np.clip(np.searchsorted(self.sample, along[selected]), 0, len(self.sample)-1)
        before = np.maximum(0, row-1)
        row = np.where(abs(self.sample[before]-along[selected]) < abs(self.sample[row]-along[selected]),
                       before, row)
        clearance = float('inf')
        for index, value in zip(row, lateral[selected]):
            widths = [2.*min(value-lo, hi-value)+2.*self.body_clearance
                      for lo, hi in self.intervals[int(index)] if lo-1e-6 <= value <= hi+1e-6]
            if not widths:
                return 0.
            clearance = min(clearance, max(widths))
        return max(0., float(clearance))


def free_space_candidates(route, ego, points, corridor, *, horizon_m=None,
                          legal_horizon_m=None, side_margin_m=.35, edge_margin_m=.25,
                          end_margin_m=.3, near_side_margin_m=.1, max_offset_m=5.,
                          max_curvature=.18, previous_target=None, max_targets=5,
                          max_lengths=3, station_step_m=2., half_width_m=EGO_HALF_WIDTH_M,
                          front_m=EGO_FRONT_M, rear_m=EGO_REAR_M, objects=(), pedestrian_inflate_m=None):
    """Return bounded lattice options and current-source passage diagnostics.

    All geometry must already share the current scan/validation vehicle frame.
    Returns ``targets``, ``lengths``, ``quality(path, horizon)`` and
    ``reuse_min_quality`` for lattice.plan, plus serializable ``passages``.
    Connectivity propagates the ego's reachable lateral interval through
    successive measured rows. Rotated body and steering checks remain in plan.
    Stations are rear-axle positions: the body reaches front_m ahead of and rear_m
    behind each (ego_body).
    """
    route = np.asarray(route, dtype=float)[:, :2]
    if len(route) < 3 or not np.isfinite(route).all():
        raise ValueError('Free-space route must have at least three finite points')
    if station_step_m <= 0. or not math.isfinite(station_step_m):
        raise ValueError('station_step_m must be finite and positive')
    stations = _arc(route)
    if stations[-1] <= 1.:
        raise ValueError('Free-space route is too short')
    speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
    horizon = min(stations[-1]-1., max(40., speed*4.+20.) if horizon_m is None else float(horizon_m))
    wall_horizon = min(horizon, max(0., float(legal_horizon_m))) if legal_horizon_m is not None else horizon
    measured_horizon = horizon
    tangent = np.gradient(route, axis=0)
    tangent /= np.maximum(1e-9, np.linalg.norm(tangent, axis=1, keepdims=True))
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    origin_offset = -float(route[0] @ normal[0])
    reach = max(0., float(max_offset_m))
    target_limit, length_limit = max(1, min(5, int(max_targets))), max(1, min(3, int(max_lengths)))
    sample = np.unique(np.r_[np.arange(0., measured_horizon, station_step_m), measured_horizon])
    centres = np.column_stack([np.interp(sample, stations, route[:, axis]) for axis in (0, 1)])
    _, route_at = cKDTree(route).query(centres)
    body_clearance = half_width_m+side_margin_m
    fallback = dict(targets=(float(np.clip(origin_offset, -reach, reach)),),
                    lengths=(max(8., speed*1.6),), passages=(),
                    quality=None, reuse_min_quality=0., horizon_m=horizon,
                    reason='measured corridor unavailable')
    if corridor is None or measured_horizon <= 0.:
        return fallback
    _, wall_at = corridor.tree.query(centres)
    projection = np.sum(normal[route_at]*corridor.normal[wall_at], axis=1)
    displacement = np.sum((centres-corridor.route[wall_at])*corridor.normal[wall_at], axis=1)
    wall_margin = half_width_m+edge_margin_m
    lower = (np.asarray(corridor.right)[wall_at]+wall_margin-displacement)/np.maximum(.5, projection)
    upper = (np.asarray(corridor.left)[wall_at]-wall_margin-displacement)/np.maximum(.5, projection)
    lower, upper = np.maximum(lower, -reach), np.minimum(upper, reach)
    upper[projection <= .5] = lower[projection <= .5]
    # Exit geometry is checked by the consumer's physical wall callback. The
    # straight tunnel corridor must not close a curved road beyond its end.
    lower[sample > wall_horizon], upper[sample > wall_horizon] = -reach, reach
    cloud = np.asarray(points, dtype=float).reshape(-1, 2) if points is not None else np.empty((0, 2))
    if objects is not None and len(objects):
        # Reuse the lattice's exact frozen/semantic box conversion. The lazy
        # import keeps this proposal helper out of the runtime import cycle.
        from lattice_planner import object_points
        options = {} if pedestrian_inflate_m is None else dict(pedestrian_inflate=pedestrian_inflate_m)
        measured_boxes = object_points(objects, ego, **options)
        if len(measured_boxes):
            cloud = np.vstack((cloud, measured_boxes))
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    if len(cloud):
        _, point_at = cKDTree(route).query(cloud)
        point_along = stations[point_at]+np.sum((cloud-route[point_at])*tangent[point_at], axis=1)
        point_lateral = np.sum((cloud-route[point_at])*normal[point_at], axis=1)
        beside = ((cloud[:, 0] < front_m+.5) &
                  (abs(cloud[:, 1]) > half_width_m+near_side_margin_m))
        # At the origin the ego points along +x, even when the route turns.
        # Projecting returns already beside it into the route frame invents
        # blockers ahead of a turn. Keep those in the mandatory near-body
        # swept check, while proposals use the forward obstacle rows.
        point_along, point_lateral = point_along[~beside], point_lateral[~beside]
        inflation = np.full(len(point_along), half_width_m+side_margin_m)
    else:
        point_along = point_lateral = inflation = np.empty(0)
    rows, constrained = [], []
    for index, station in enumerate(sample):
        ahead = point_along-station
        nearby = ((ahead <= front_m+end_margin_m+station_step_m/2.) &
                  (ahead >= -(rear_m+end_margin_m+station_step_m/2.)))
        occupied = [(float(lo), float(hi)) for lo, hi in
                    zip(point_lateral[nearby]-inflation[nearby], point_lateral[nearby]+inflation[nearby])
                    if hi > lower[index] and lo < upper[index]]
        intervals = _subtract(lower[index], upper[index], occupied)
        rows.append(intervals)
        if occupied:
            constrained.append(index)
    quality = _PassageQuality(route, stations, tangent, normal, sample, rows, body_clearance)

    # Keep connectivity separate from gap width: an isolated wide pocket must
    # not outrank a narrower passage that the ego can actually enter.
    layers = []
    previous = [dict(lo=origin_offset, hi=origin_offset, width=float('inf'), chain=())]
    for index, intervals in enumerate(rows):
        # Rows inflate longitudinal obstacle extent by the front/rear body.
        # A fixed lane-change slope of .45 prematurely disconnected a wide
        # entry gap that a checked short quintic turn reaches. This generous
        # proposal envelope retains that branch; the lattice's actual rotated
        # sweep and curvature limit decide whether it is drivable.
        movement = .9*(sample[index]-sample[index-1]) if index else 0.
        layer = []
        for lo, hi in intervals:
            options = []
            for parent in previous:
                reachable_lo = max(lo, parent['lo']-movement)
                reachable_hi = min(hi, parent['hi']+movement)
                if reachable_lo <= reachable_hi+1e-9:
                    width = min(parent['width'], hi-lo)
                    options.append(dict(lo=reachable_lo, hi=reachable_hi, width=width,
                        chain=parent['chain']+((index, lo, hi),)))
            # Multiple predecessor branches retain distinct reachable ranges.
            layer.extend(sorted(options, key=lambda item: (-item['width'], abs((item['lo']+item['hi'])/2.-origin_offset)))[:5])
        if not layer:
            break
        layers.append(layer)
        previous = layer
    passages = []
    if layers:
        for node in layers[-1]:
            chain = node['chain']
            # Select the constriction that requires a lateral change. A
            # narrow row the ego can already follow may precede the actual
            # obstacle; targeting that row's midpoint hides a later wide pass.
            obstacle_rows = [row for row in chain if row[0] in constrained
                             and not row[1]-1e-6 <= origin_offset <= row[2]+1e-6]
            if not obstacle_rows:
                target = float(np.clip(origin_offset, node['lo'], node['hi']))
                entry = 0.
                lo, hi = node['lo'], node['hi']
            else:
                index, lo, hi = min(obstacle_rows, key=lambda row: (row[2]-row[1], row[0]))
                target = float((lo+hi)/2.)
                entry = float(sample[obstacle_rows[0][0]])
            passages.append(dict(target=target, width_m=float(node['width']+2.*body_clearance),
                reachable_m=float(sample[len(layers)-1]), entry_m=entry,
                lower_m=float(lo), upper_m=float(hi),
                complete=len(layers)==len(rows)))
    passages.sort(key=lambda item: (-int(item['complete']), -item['reachable_m'],
        -item['width_m'], abs(item['target']-origin_offset)))
    # Keep a centre/rejoin fallback, but reserve room for each measured side
    # before adding continuity and edge alternatives.
    targets = []
    def add(value):
        value = float(np.clip(value, -reach, reach))
        if len(targets) < target_limit and all(abs(value-other) > .1 for other in targets):
            targets.append(value)
    # Always keep the exact route target. Deduplicating a small ego offset
    # against zero otherwise labels an empty-road join as a fresh bypass.
    add(0.)
    for passage in passages:
        add(passage['target'])
    for passage in passages:
        if passage['entry_m'] > 0.:
            near = (passage['lower_m'] + .4 if passage['target'] >= origin_offset
                    else passage['upper_m'] - .4)
            add(np.clip(near, passage['lower_m'], passage['upper_m']))
    if previous_target is not None and math.isfinite(float(previous_target)):
        add(float(previous_target))
    for passage in passages:
        # A constant offset at a gap's midpoint may still enter too late.
        # Include the far side within the measured free interval, so a short
        # join can obtain the required clearance before its final offset.
        add(passage['upper_m'] if passage['target'] >= origin_offset else passage['lower_m'])
    for passage in passages:
        add((passage['target']+origin_offset)/2.)
    if not targets:
        add(origin_offset)
    if constrained:
        # Retain the proven lattice's medium/long joins. A curvature-derived
        # minimum alone discarded the recorded +5 m / 18 m wide bypass.
        lengths = ([5. if objects is not None and any(o[4]=='pedestrian' for o in objects) else 6., 12., 18.]
                   if speed < 1.5 else [max(8., speed*1.6), max(12., speed*2.5), max(18., speed*3.5)])
    else:
        minimum_join = math.sqrt(5.77*abs(origin_offset)*max(3., speed)**2/2.)
        lengths = [max(8., speed*1.6, minimum_join)]
    lengths = tuple(dict.fromkeys(round(value, 2) for value in lengths))[:length_limit]
    complete = [passage['width_m'] for passage in passages if passage['complete']]
    return dict(targets=tuple(targets), lengths=lengths, passages=tuple(passages), quality=quality,
                reuse_min_quality=max(0., max(complete, default=0.)-.5),
                horizon_m=horizon, reason='connected measured passages' if complete else 'no complete measured passage')
