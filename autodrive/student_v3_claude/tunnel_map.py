"""Two-wall tunnel geometry, bounded transverse alignment and strict surface filtering."""
import json
import math
import os
from pathlib import Path

import numpy as np

from ego_body import EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M, REAR_AXLE_SHIFT_M, body_centres, self_return_mask


def _dot(points, vector):
    """Small 2D products avoid multithreaded BLAS overhead on every scan."""
    return points[..., 0]*vector[0]+points[..., 1]*vector[1]


def _rotation(heading_deg):
    angle = math.radians(float(heading_deg))
    return np.array([[math.cos(angle), -math.sin(angle)],
                     [math.sin(angle), math.cos(angle)]])


LEDGE_MAX_M = .75       # widest kerb folded into the wall
LEDGE_TOP_M = 1.4       # kerb top (measured 1.0-1.25 m)
LEDGE_MIN_SPAN_M = 6.   # continuous along the wall; a drum or car at the wall is shorter
LEDGE_CREDIT_M = .2     # kerb width already covered by the 0.25 m wall margin (>= 0.05 m left)

# M7 (2026-10-05): surveyed empty-tunnel background. Inside the mapped section wall
# and kerb are removed by a per-height wall-section profile (+0.05 m); everywhere a
# 0.1 m voxel map (dilated +-1 voxel) of the empty pass removes the portal, exit and
# ceiling returns -- except in the protected road band (> kerb face + 0.15 m inward,
# > 0.3 m high), where the map never deletes anything.
BACKGROUND_FORMAT = 'morai_tunnel_background_v1'
OBSTACLE_H_MIN, OBSTACLE_H_MAX = .14, 2.2
PROFILE_TOL_M = .05
ROAD_PROTECT_M = .15
ROAD_PROTECT_MIN_H = .3
EXIT_RESIDUE_M = 10.   # unsurveyed road past the exit treated like the surveyed road band
OUTPUT_CELL_M = .1
# Propagated (wall-occluded) scans carry odometry pose error: wider tolerances.
PROJECTED_PROFILE_TOL_M = .15
PROJECTED_VOXEL_SPREAD_M = .1
# Road-plane band (kerb widths of the survey + margin), cross-slope and grade limits.
ROAD_KERB_M = {1: .75, -1: .65}
ROAD_MARGIN_M = .6
ROAD_MAX_CROSS = .04
ROAD_MAX_GRADE = .12


def voxel_ds2d(points, cell=OUTPUT_CELL_M):
    """One real return per XY cell (first in scan order; not the cell centre)."""
    if not len(points):
        return points
    _, keep = np.unique(np.floor(points/cell).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(keep)]


def _lowest_per_cell(q):
    cell = np.floor(q[:, :2]).astype(np.int64)
    key = (cell[:, 0]+1000)*4096+(cell[:, 1]+2048)
    order = np.lexsort((q[:, 2], key))
    ks = key[order]
    first = np.r_[True, ks[1:] != ks[:-1]]
    counts = np.diff(np.r_[np.flatnonzero(first), len(ks)])
    return q[order[first]], counts


def road_plane(p, lateral, left, right, fallback):
    """G1: robust plane z = a x + b y + c over the visible kerb-to-kerb tunnel road
    (lowest return per 1 m cell). Out-of-limit or unsupported fits keep ``fallback``."""
    fallback = np.asarray(fallback, dtype=float)
    band = ((lateral < left-ROAD_KERB_M[1]-ROAD_MARGIN_M) & (lateral > right+ROAD_KERB_M[-1]+ROAD_MARGIN_M) &
            (p[:, 0] > -40.) & (p[:, 0] < 60.) & ((np.abs(p[:, 0]) > 2.6) | (np.abs(p[:, 1]) > 1.2)))
    q = p[band]
    if len(q) < 60:
        return fallback
    anchors, counts = _lowest_per_cell(q)
    anchors = anchors[counts >= 2]
    if len(anchors) < 15:
        return fallback
    A = np.column_stack((anchors[:, :2], np.ones(len(anchors))))
    z = anchors[:, 2]
    plane = np.array([0., 0., np.percentile(z, 20)])
    for it in range(5):
        weight = (np.abs(z-A @ plane) < (.25 if it == 0 else .08)).astype(float)
        if weight.sum() < 12:
            return fallback
        plane = np.linalg.lstsq(A*weight[:, None], z*weight, rcond=None)[0]
    if not np.isfinite(plane).all() or abs(plane[1]) > ROAD_MAX_CROSS or abs(plane[0]) > ROAD_MAX_GRADE:
        return fallback
    return plane


def road_ground(p, lateral, left, right, fallback, bin_m=5., clamp=.35):
    """G2: G1 plane plus a piecewise-linear longitudinal correction (vertical curves)
    from the 30th-percentile residual of the lowest road return per 1 m cell in 5 m
    bins of vehicle x. Returns the ground height under every point."""
    plane = road_plane(p, lateral, left, right, fallback)
    base = _dot(p[:, :2], plane[:2])+plane[2]
    band = ((lateral < left-ROAD_KERB_M[1]-ROAD_MARGIN_M) & (lateral > right+ROAD_KERB_M[-1]+ROAD_MARGIN_M) &
            (p[:, 0] > -60.) & (p[:, 0] < 80.) & ((np.abs(p[:, 0]) > 2.6) | (np.abs(p[:, 1]) > 1.2)))
    q = p[band]
    if len(q) < 60:
        return base
    anchors, _ = _lowest_per_cell(q)
    residual = anchors[:, 2]-(_dot(anchors[:, :2], plane[:2])+plane[2])
    ok = np.abs(residual) < clamp
    anchors, residual = anchors[ok], residual[ok]
    edges = np.arange(-60., 80.1, bin_m)
    index = np.digitize(anchors[:, 0], edges)-1
    centres, offsets = [], []
    for b in range(len(edges)-1):
        values = residual[index == b]
        if len(values) >= 3:
            centres.append(edges[b]+bin_m/2.)
            offsets.append(float(np.percentile(values, 30)))
    if len(centres) < 2:
        return base
    return base+np.interp(p[:, 0], centres, offsets)


class TunnelBackground:
    """Static empty-tunnel survey in the tunnel frame (along, lateral, height above road)."""
    def __init__(self, data):
        self.shape = tuple(int(v) for v in data['grid_shape'])
        self.bits = np.asarray(data['grid_bits'], dtype=np.uint8)
        self.voxel = float(data['voxel_m'])
        self.along0, self.lateral0 = float(data['along_min_m']), float(data['lateral_min_m'])
        self.height_bin = float(data['profile_bin_m'])
        self.profile = {1: np.asarray(data['profile_left_m'], dtype=float),
                        -1: np.asarray(data['profile_right_m'], dtype=float)}
        self.kerb_face = {1: float(data['kerb_face_left_m']), -1: float(data['kerb_face_right_m'])}
        if (len(self.shape) != 3 or min(self.shape) < 1 or self.bits.size*8 < int(np.prod(self.shape)) or
                not 0. < self.voxel <= .3 or not 0. < self.height_bin <= .2):
            raise ValueError('malformed tunnel background grid')
        for side in (1, -1):
            if (self.profile[side].ndim != 1 or len(self.profile[side]) < 10 or
                    not np.isfinite(self.profile[side]).all() or not 0. <= self.kerb_face[side] <= LEDGE_MAX_M):
                raise ValueError('malformed tunnel background profile')

    @classmethod
    def load(cls, path, payload):
        with np.load(path) as data:
            data = {key: data[key] for key in data.files}
        if str(data.get('format')) != BACKGROUND_FORMAT:
            raise ValueError('unsupported tunnel background')
        # The survey is only meaningful in the frame of the wall map it was built for.
        # It is wall-aligned (built from wall-matched scans), so a rigid world
        # re-alignment of the same walls ('realigned_from': same offsets and extent)
        # keeps it valid.
        frames = [payload]+([payload['realigned_from']] if 'realigned_from' in payload else [])
        expected = [list(frame['origin_xy'])+[frame[key] for key in (
            'axis_heading_deg', 'left_offset_m', 'right_offset_m', 'longitudinal_min_m', 'longitudinal_max_m')]
            for frame in frames]
        survey = np.asarray(data['map_frame'], dtype=float)
        if not (np.allclose(expected[0][3:], expected[-1][3:], atol=1e-6, rtol=0.) and
                any(np.allclose(survey, frame, atol=1e-6, rtol=0.) for frame in expected)):
            raise ValueError('tunnel background was built for a different wall map')
        return cls(data)

    def occupied(self, along, lateral, height):
        i = np.floor((along-self.along0)/self.voxel).astype(np.int64)
        j = np.floor((lateral-self.lateral0)/self.voxel).astype(np.int64)
        k = np.floor(height/self.voxel).astype(np.int64)
        inside = ((i >= 0) & (i < self.shape[0]) & (j >= 0) & (j < self.shape[1]) &
                  (k >= 0) & (k < self.shape[2]))
        out = np.zeros(len(along), dtype=bool)
        flat = (i[inside]*self.shape[1]+j[inside])*self.shape[2]+k[inside]
        out[inside] = ((self.bits[flat >> 3] >> (7-(flat & 7))) & 1).astype(bool)
        return out

    def wall_section(self, lateral, height, left, right, tolerance):
        """Within the surveyed section: behind the measured wall/kerb face at this height."""
        b = np.clip((height/self.height_bin).astype(np.int64), 0, len(self.profile[1])-1)
        return (((left-lateral) <= self.profile[1][b]+tolerance) |
                ((lateral-right) <= self.profile[-1][b]+tolerance))


def background_path_for(map_path):
    return Path(os.environ.get('STUDENT_TUNNEL_BACKGROUND', str(Path(map_path).with_name('tunnel_background.npz'))))


def fit_wall(points, expected_offset=None, expected_slope=0., side=1, x_min=-12., min_span=14.):
    """Fit a long supported vertical wall; isolated returns never define a wall."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    selected = ((points[:, 0] > x_min) & (points[:, 0] < 60.) &
                (points[:, 2] > .45) & (points[:, 2] < 3.3))
    if expected_offset is None:
        selected &= (side*points[:, 1] > 2.) & (side*points[:, 1] < 13.)
    else:
        selected &= np.abs(points[:, 1]-expected_slope*points[:, 0]-expected_offset) < 1.4
    p = points[selected]
    if len(p) < 35:
        return None
    # One point per small XYZ cell prevents nearby beams dominating the fit.
    _, keep = np.unique(np.floor(p/[.3, .03, .2]).astype(np.int32), axis=0, return_index=True)
    p = p[keep]
    if len(p) < 30:
        return None
    rng = np.random.default_rng(42)
    pairs = rng.integers(0, len(p), size=(72, 2))
    delta = p[pairs[:, 1], :2]-p[pairs[:, 0], :2]
    good = (np.abs(delta[:, 0]) > 10.)
    slopes = delta[good, 1]/delta[good, 0]
    intercepts = p[pairs[good, 0], 1]-slopes*p[pairs[good, 0], 0]
    good = np.abs(slopes-expected_slope) < .15
    slopes, intercepts = slopes[good], intercepts[good]
    if not len(slopes):
        return None
    distance = np.abs(p[:, 1, None]-p[:, 0, None]*slopes-intercepts)/np.sqrt(1+slopes*slopes)
    counts = np.sum(distance < .045, axis=0)
    best = int(np.argmax(counts))
    mask = distance[:, best] < .045
    if mask.sum() < 30:
        return None
    for _ in range(3):
        slope, intercept = np.linalg.lstsq(np.column_stack((p[mask, 0], np.ones(mask.sum()))),
                                          p[mask, 1], rcond=None)[0]
        residual = (p[:, 1]-slope*p[:, 0]-intercept)/math.sqrt(1+slope*slope)
        mask = np.abs(residual) < .045
        if mask.sum() < 30:
            return None
    accepted = p[mask]
    span = float(np.percentile(accepted[:, 0], 95)-np.percentile(accepted[:, 0], 5))
    x_bins = len(np.unique(np.floor(accepted[:, 0]/2.).astype(int)))
    height_bins = len(np.unique(np.floor(accepted[:, 2]/.35).astype(int)))
    if span < min_span or x_bins < 8 or height_bins < 3:
        return None
    normal = np.array([-slope, 1.])/math.sqrt(1+slope*slope)
    return dict(slope=float(slope), intercept=float(intercept), normal=normal,
                offset=float(intercept/math.sqrt(1+slope*slope)), points=accepted,
                n=len(accepted), span_m=span, residual_p95_m=float(np.percentile(np.abs(residual[mask]), 95)))


class TunnelMap:
    """A surveyed parallel-wall section with no longitudinal pose correction."""
    def __init__(self, payload, background=None):
        self.payload = payload
        self.origin = np.asarray(payload['origin_xy'], dtype=float)
        self.heading = float(payload['axis_heading_deg'])
        self.axis = _rotation(self.heading)[:, 0]
        self.normal = _rotation(self.heading)[:, 1]
        self.left = float(payload['left_offset_m'])
        self.right = float(payload['right_offset_m'])
        # Measured kerb/ledge at the wall base (m inward); see process().
        self.ledge = {1: 0., -1: 0.}
        self.start = float(payload['longitudinal_min_m'])
        self.end = float(payload['longitudinal_max_m'])
        self.max_lateral = float(payload.get('max_lateral_correction_m', .8))
        self.max_yaw = float(payload.get('max_yaw_correction_deg', 3.))
        self.wall_tolerance = float(payload.get('wall_filter_tolerance_m', .03))
        # M7 extraction when a surveyed background matches this map; otherwise the
        # vertical-wall + kerb-fold filter below (synthetic maps, missing file).
        self.background = background
        if background is not None:
            # Surveyed kerb faces (bumper height, p99). LEDGE_CREDIT_M still applies.
            self.ledge = {1: background.kerb_face[1], -1: background.kerb_face[-1]}

    @classmethod
    def load(cls, path, background='auto', survey_frame=False):
        payload = json.loads(Path(path).read_text())
        if survey_frame and 'realigned_from' in payload:
            # Recordings made before the re-alignment (ESKF poses of 2026-10-02/03) are
            # in the frame the walls were surveyed in.
            payload = dict(payload, **payload['realigned_from'])
        if payload.get('format') != 'morai_fixed_tunnel_walls_v1':
            raise ValueError('unsupported tunnel wall map')
        values = list(payload['origin_xy'])+[payload[key] for key in (
            'axis_heading_deg', 'left_offset_m', 'right_offset_m',
            'longitudinal_min_m', 'longitudinal_max_m')]
        if len(payload['origin_xy']) != 2 or not np.isfinite(values).all():
            raise ValueError('tunnel wall coordinates must be finite')
        if not 6. <= payload['left_offset_m']-payload['right_offset_m'] <= 18.:
            raise ValueError('tunnel width outside supported range')
        if payload['longitudinal_max_m'] <= payload['longitudinal_min_m']:
            raise ValueError('tunnel map extent must be ordered')
        for key, default, maximum in (('max_lateral_correction_m', .8, 1.),
                                      ('max_yaw_correction_deg', 3., 5.),
                                      ('wall_filter_tolerance_m', .03, .04)):
            value = float(payload.get(key, default))
            if not math.isfinite(value) or not 0. < value <= maximum:
                raise ValueError(f'invalid tunnel map bound: {key}')
        if background == 'auto':
            background = None
            filename = background_path_for(path)
            try:
                background = TunnelBackground.load(filename, payload)
            except Exception as exc:   # missing/corrupt/mismatched: legacy extraction
                print(f'[tunnel] background {filename} unavailable ({exc}); using wall/kerb filter', flush=True)
        elif background is not None and not isinstance(background, TunnelBackground):
            background = TunnelBackground.load(background, payload)
        return cls(payload, background)

    @property
    def inner_left(self):
        """Left wall face for planning: the surveyed wall minus its kerb beyond the
        first LEDGE_CREDIT_M. The 0.25 m body margin to a wall face already covers
        that much; the full kerb width braked the car on the empty default route,
        which runs 0.35 m from the right kerb (recorded tunnel test)."""
        return self.left-max(0., self.ledge[1]-LEDGE_CREDIT_M)

    @property
    def inner_right(self):
        return self.right+max(0., self.ledge[-1]-LEDGE_CREDIT_M)

    def active(self, ego):
        delta = np.array([ego['x'], ego['y']])-self.origin
        along, lateral = delta @ self.axis, delta @ self.normal
        return self.start <= along <= self.end and self.right-2. <= lateral <= self.left+2.

    def corridor(self, ego):
        """The competition permits the complete measured space between tunnel walls."""
        from lane_corridor import LaneCorridor
        rotation = _rotation(ego['heading_deg'])
        centre = self.origin + .5*(self.inner_left+self.inner_right)*self.normal
        along = (np.array([ego['x'], ego['y']])-centre) @ self.axis
        world = centre + (along+np.arange(-80., 160.1, .5))[:, None]*self.axis
        route = (world-[ego['x'], ego['y']]) @ rotation
        half_width = .5*(self.inner_left-self.inner_right)
        corridor = LaneCorridor(route, np.full(len(route), half_width),
                                np.full(len(route), -half_width), body_shift_m=REAR_AXLE_SHIFT_M)
        # The physical tunnel ends; its straight sides must not extend across
        # the curved exit road merely because the query path is 60 m long.
        corridor.clear_distance = lambda path, horizon_m=60.: self.path_clearance(path, ego, horizon_m)
        corridor.wall_only = True
        return corridor

    def adjacent_offsets(self, ego, route):
        """Offer both wall-bounded sides; each candidate still checks every body corner."""
        route = np.asarray(route, dtype=float)[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        world = route[arc <= 60.] @ _rotation(ego['heading_deg']).T + [ego['x'], ego['y']]
        along = _dot(world-self.origin, self.axis)
        covered = (along >= self.start) & (along <= self.end)
        if covered.any():
            world = world[covered]
        lateral = _dot(world-self.origin, self.normal)
        margin = .946+.25+.15
        available = {1: float(self.inner_left-lateral.max()-margin),
                     -1: float(lateral.min()-self.inner_right-margin)}
        return {side: width for side, width in available.items() if width >= .45}

    def process(self, vehicle_xyz, ego, ground_plane, *, approach=False, lateral_prior_m=0., search_m=()):
        """Fit both walls and return the bounded map-pose correction.

        ``lateral_prior_m`` centres the +-1.4 m wall-fit window on the wall-frame
        lateral last measured (ESKF + that correction) instead of on the raw ESKF
        pose; ``search_m`` adds further window shifts (relative to the prior) tried
        only when the earlier windows fail. With the defaults the window, gates and
        result are unchanged. Every attempt passes the same span, width and
        parallelism gates, and the reported correction is always relative to the
        raw ESKF pose, so the map-aiding limit keeps its meaning.
        """
        original = np.asarray(vehicle_xyz, dtype=float).reshape(-1, 3)
        invalid = dict(valid=False, reason='tunnel wall match unavailable', aligned_ego=dict(ego),
                       obstacle_points=np.empty((0, 2)), wall_stats={}, constraint=None)
        delta = np.array([ego['x'], ego['y']])-self.origin
        along, lateral = delta @ self.axis, delta @ self.normal
        prior = float(lateral_prior_m) if math.isfinite(float(lateral_prior_m)) else 0.
        approach_limit = 60. if approach else 2.5675
        if not (self.start-approach_limit <= along <= self.end+2.5675 and
                self.right-2. <= lateral+prior <= self.left+2.):
            invalid['reason'] = 'outside mapped tunnel section'
            return invalid
        if len(original) < 100 or ground_plane is None:
            invalid['reason'] = 'missing scan or ground plane'
            return invalid
        plane = np.asarray(ground_plane, dtype=float)
        if plane.shape != (3,) or not np.isfinite(plane).all() or np.linalg.norm(plane[:2]) > .22:
            invalid['reason'] = 'invalid ground plane'
            return invalid
        p = original[np.isfinite(original).all(axis=1)]
        height = p[:, 2]-(_dot(p[:, :2], plane[:2])+plane[2])
        body_free = self_return_mask(p)
        structure = np.column_stack((p[:, :2], height))
        rotation = _rotation(ego['heading_deg'])
        map_normal_body = rotation.T @ self.normal
        # cos 35 deg: after a stop with a bypass offset the car stood 18-20 deg
        # off the axis and this guard (was cos 18) froze it for minutes
        # (live 2026-10-03). The wall fit already uses the tilted slope.
        if map_normal_body[1] < .82:
            invalid['reason'] = 'heading inconsistent with tunnel map'
            return invalid
        lateral = (np.array([ego['x'], ego['y']])-self.origin) @ self.normal
        slope = -map_normal_body[0]/map_normal_body[1]
        along = (np.array([ego['x'], ego['y']])-self.origin) @ self.axis
        approximate_point_along = along+_dot(p[:, :2], rotation.T @ self.axis)
        fitting = body_free & (approximate_point_along >= self.start) & (approximate_point_along <= self.end)
        to_exit = self.end-along
        # A surveyed wall terminates at the exit rather than continuing into
        # the next road. The closer wall's forward returns shorten accordingly.
        # Keep two fresh, dense vertical surfaces and a full-length opposite
        # wall; only their span requirement follows this known physical end.
        boundary_span = max(10., 9.+to_exit) if 0. <= to_exit <= 5. else 14.

        def wall_pair(shift):
            """Both walls inside one window (centre: ESKF lateral + shift) and their gates."""
            expected = lateral+shift
            pair = [fit_wall(structure[fitting], (offset-expected)/map_normal_body[1], slope, side,
                             x_min=-40., min_span=boundary_span)
                    for offset, side in ((self.left, 1), (self.right, -1))]
            if any(wall is None for wall in pair):
                return None, None, 'both supported tunnel walls required'
            left, right = pair
            shortened_exit_support = min(left['span_m'], right['span_m']) < 14.
            if shortened_exit_support and (min(left['n'], right['n']) < 100 or
                                            max(left['span_m'], right['span_m']) < 14.):
                return None, None, 'both supported tunnel walls required'
            mismatch = abs(math.degrees(math.atan(left['slope'])-math.atan(right['slope'])))
            width = left['offset']-right['offset']
            stats = dict(left_n=left['n'], right_n=right['n'], left_span_m=left['span_m'],
                         right_span_m=right['span_m'], width_m=width, parallel_error_deg=mismatch,
                         fit_residual_p95_m=max(left['residual_p95_m'], right['residual_p95_m']))
            if shortened_exit_support:
                stats['surveyed_exit_support'] = True
            if mismatch > .6 or abs(width-(self.left-self.right)) > .16:
                return None, stats, 'wall width or parallelism inconsistent'
            return pair, stats, None

        # Prior window first, then the raw ESKF window (the original behaviour),
        # then the search shifts. The first pair passing every gate is used.
        shifts = [prior]
        for shift in (0., *(prior+float(value) for value in search_m)):
            if math.isfinite(shift) and all(abs(shift-other) > 1e-6 for other in shifts):
                shifts.append(shift)
        first_failure = None
        for shift in shifts:
            walls, stats, reason = wall_pair(shift)
            if walls is not None:
                break
            if first_failure is None:
                first_failure = (stats, reason)
        if walls is None:
            stats, reason = first_failure
            if stats is not None:
                invalid['wall_stats'] = stats
            invalid['reason'] = reason
            return invalid
        left, right = walls
        if abs(shift) > 1e-6:
            stats['window_lateral_m'] = float(shift)
        invalid['wall_stats'] = stats
        # The fresh vehicle-frame wall fits remain usable as physical bounds
        # even when their absolute map-pose correction is rejected below.
        invalid['body_walls'] = dict(validated=True, **{
            name: dict(slope=float(wall['slope']),
                       intercept_m=float(wall['offset']*math.sqrt(1.+wall['slope']**2)))
            for name, wall in [('left', left), ('right', right)]})
        measured_angle = math.degrees(math.atan((left['slope']+right['slope'])/2.))
        aligned_heading = self.heading-measured_angle
        yaw_delta = (aligned_heading-ego['heading_deg']+180.) % 360.-180.
        corrected_lateral = ((self.left-left['offset'])+(self.right-right['offset']))/2.
        correction = corrected_lateral-lateral
        stats.update(lateral_correction_m=float(correction), yaw_correction_deg=float(yaw_delta),
                     longitudinal_correction_m=0.)
        if abs(correction) > self.max_lateral or abs(yaw_delta) > self.max_yaw:
            invalid['reason'] = 'pose correction exceeds validated limit'
            return invalid
        aligned = dict(ego)
        aligned['x'], aligned['y'] = (np.array([ego['x'], ego['y']])+correction*self.normal).tolist()
        aligned['heading_deg'] = ego['heading_deg']+yaw_delta
        constraint = dict(valid=True, active=True, map_point=[aligned['x'], aligned['y']],
                          map_heading_rad=math.radians(self.heading), body_point=[0., 0.],
                          yaw_measurement_rad=math.radians(aligned['heading_deg']),
                          lateral_variance_m2=.25, yaw_variance_rad2=math.radians(3.)**2,
                          source='tunnel_lidar_wall')
        if self.background is not None:
            obstacle_points, removed = self.background_obstacles(p, aligned, plane, body_free)
            return dict(valid=True, reason='two tunnel walls matched', aligned_ego=aligned,
                        obstacle_points=obstacle_points,
                        wall_stats=dict(stats, filtered_wall_n=removed, extraction='background'),
                        constraint=constraint)
        normal_body = _rotation(aligned['heading_deg']).T @ self.normal
        signed = _dot(p[:, :2], normal_body)
        along = (np.array([aligned['x'], aligned['y']])-self.origin) @ self.axis
        point_along = along+_dot(p[:, :2], _rotation(aligned['heading_deg']).T @ self.axis)
        mapped = (point_along >= self.start) & (point_along <= self.end)
        wall_mask = np.zeros(len(p), dtype=bool)
        for offset, wall in zip((self.left, self.right), walls):
            measured_residual = np.abs(_dot(p[:, :2], wall['normal'])-wall['offset'])
            mapped_residual = np.abs(signed-(offset-corrected_lateral))
            wall_mask |= (mapped & (measured_residual <= self.wall_tolerance) &
                          (mapped_residual <= self.wall_tolerance) & (height > .14) & (height < 3.4))
        # The tunnel walls stand on a 0.4-0.6 m kerb (height < 1.25 m) that the wall
        # map does not hold: every kerb return became an "obstacle" (2,000-5,800 points
        # per scan, 2.5-4.7 s decisions, and the reverse vetoed by the kerb behind;
        # auto runs 164/165). A kerb is continuous along the wall: fold it into the wall
        # and move that wall's effective face inward by its measured width instead.
        tunnel_lateral = signed+corrected_lateral
        point_along_map = point_along
        for offset, side in ((self.left, 1), (self.right, -1)):
            inward = (offset-tunnel_lateral) if side == 1 else (tunnel_lateral-offset)
            candidate = (mapped & body_free & (inward > self.wall_tolerance) & (inward <= LEDGE_MAX_M) &
                         (height > .1) & (height < LEDGE_TOP_M))
            # Above the kerb the wall face leans/varies by 0.1-0.3 m (car-roof height
            # and up): within 0.15 m of the surveyed face it is wall, not an obstacle.
            wall_mask |= mapped & (np.abs(inward) <= .15) & (height >= LEDGE_TOP_M-.1) & (height < 3.4)
            if candidate.sum() >= 40:
                span = float(np.ptp(point_along_map[candidate]))
                if span >= LEDGE_MIN_SPAN_M:
                    width = float(min(LEDGE_MAX_M, np.percentile(inward[candidate], 90)))
                    # Smoothed: a width jittering 0.55-0.59 m between scans moved the
                    # cut through the kerb, and the leftover fragment's jumping centroid
                    # read as a 4.8 m/s "pedestrian" (recorded tunnel replay, scan 81).
                    previous = self.ledge[side]
                    width = width if previous <= 0. else .8*previous+.2*width
                    self.ledge[side] = width
                    wall_mask |= candidate & (inward <= width+.1)
        keep = body_free & (height > .14) & (height < 2.2) & ~wall_mask
        # Every unidentified non-surface return remains in the original body frame.
        obstacle_points = p[keep, :2]
        return dict(valid=True, reason='two tunnel walls matched', aligned_ego=aligned,
                    obstacle_points=obstacle_points, wall_stats=dict(stats, filtered_wall_n=int(wall_mask.sum())),
                    constraint=constraint)

    def relative_projection(self, vehicle_xyz, ego, ground_plane, rejected_match, max_correction_m=1.2):
        """Use freshly supported walls for planning without accepting map aiding.

        A rejected absolute correction does not invalidate the measured local
        wall geometry. Keep its projection separate from process() and never
        return a localization constraint or a propagated wall anchor.
        ``max_correction_m=None`` accepts any absolute ESKF error: the walls were
        fitted in the vehicle frame and passed every span/width/parallelism gate,
        so the pose relative to them is measured whatever the dead-reckoning drift.
        """
        walls = rejected_match.get('body_walls') or {}
        stats = rejected_match.get('wall_stats') or {}
        if (rejected_match.get('reason') != 'pose correction exceeds validated limit'
                or walls.get('validated') is not True):
            return None
        correction = float(stats.get('lateral_correction_m', float('nan')))
        yaw = float(stats.get('yaw_correction_deg', float('nan')))
        bound = math.inf if max_correction_m is None else float(max_correction_m)
        # The raw-ESKF fitting window admits 1.2 m, but map aiding still requires
        # max_lateral (0.8 m). Heading remains within the original validated gate.
        if (not math.isfinite(correction) or not math.isfinite(yaw)
                or abs(correction) > bound or abs(yaw) > self.max_yaw):
            return None
        aligned = dict(ego)
        aligned['x'], aligned['y'] = (np.asarray([ego['x'], ego['y']]) + correction*self.normal).tolist()
        aligned['heading_deg'] += yaw
        projected = self.project_scan(vehicle_xyz, aligned, ground_plane, stats,
                                      profile_tolerance=PROFILE_TOL_M, voxel_spread=0.)
        if not projected.get('valid'):
            return None
        projected.update(reason='fresh relative tunnel wall geometry', relative_geometry=True,
                         body_walls=walls, constraint=None)
        projected['wall_stats'].pop('propagated', None)
        projected['wall_stats']['relative_geometry'] = True
        return projected

    def project_scan(self, vehicle_xyz, ego, ground_plane, previous_stats,
                     profile_tolerance=PROJECTED_PROFILE_TOL_M, voxel_spread=PROJECTED_VOXEL_SPREAD_M):
        """Carry a recently verified pose through a brief wall occlusion.

        The caller bounds age, odometry, and correction. All non-ground returns,
        including walls, remain collision evidence; no new localization aid is
        fabricated from the propagated pose.
        """
        p = np.asarray(vehicle_xyz, dtype=float).reshape(-1, 3)
        plane = np.asarray(ground_plane, dtype=float)
        if len(p) < 100 or plane.shape != (3,) or not np.isfinite(plane).all() or np.linalg.norm(plane[:2]) > .22:
            return dict(valid=False, reason='invalid scan for recent wall propagation',
                        aligned_ego=dict(ego), obstacle_points=np.empty((0, 2)),
                        wall_stats={}, constraint=None)
        p = p[np.isfinite(p).all(axis=1)]
        height = p[:, 2]-(_dot(p[:, :2], plane[:2])+plane[2])
        body_free = self_return_mask(p)
        if self.background is not None:
            # Same survey as process(), with tolerances widened for the propagated pose.
            obstacle_points, removed = self.background_obstacles(
                p, ego, plane, body_free, profile_tolerance=profile_tolerance,
                voxel_spread=voxel_spread)
            return dict(valid=True, reason='recent tunnel wall pose propagated', aligned_ego=dict(ego),
                        obstacle_points=obstacle_points,
                        wall_stats=dict(previous_stats, propagated=True, filtered_wall_n=removed,
                                        extraction='background'), constraint=None)
        keep = body_free & (height > .14) & (height < 2.2)
        return dict(valid=True, reason='recent tunnel wall pose propagated', aligned_ego=dict(ego),
                    obstacle_points=p[keep, :2],
                    wall_stats=dict(previous_stats, propagated=True, filtered_wall_n=0), constraint=None)

    def background_obstacles(self, p, ego, plane, body_free, profile_tolerance=PROFILE_TOL_M,
                             voxel_spread=0.):
        """M7 obstacle returns (vehicle frame XY) and the number of removed structure returns.

        ``ego`` is the wall-aligned (or propagated) scan pose. Heights use the G2
        tunnel-road model; it falls back to ``plane`` when the road fit fails.
        """
        bg = self.background
        rotation = _rotation(ego['heading_deg'])
        delta = np.array([ego['x'], ego['y']])-self.origin
        axis_body, normal_body = rotation.T @ self.axis, rotation.T @ self.normal
        lateral = delta @ self.normal+_dot(p[:, :2], normal_body)
        height = p[:, 2]-road_ground(p, lateral, self.left, self.right, plane)
        index = np.flatnonzero(body_free & (height > OBSTACLE_H_MIN) & (height < OBSTACLE_H_MAX))
        q, lateral, height = p[index], lateral[index], height[index]
        along = delta @ self.axis+_dot(q[:, :2], axis_body)
        mapped = (along >= self.start) & (along <= self.end)
        drop = mapped & bg.wall_section(lateral, height, self.left, self.right, profile_tolerance)
        road = (mapped & (height > ROAD_PROTECT_MIN_H) &
                (self.left-lateral > bg.kerb_face[1]+ROAD_PROTECT_M) &
                (lateral-self.right > bg.kerb_face[-1]+ROAD_PROTECT_M))
        test = np.flatnonzero(~drop & ~road)
        if len(test):
            a, l, h = along[test], lateral[test], height[test]
            hit = bg.occupied(a, l, h)
            if voxel_spread > 0.:
                for da, dl in ((voxel_spread, 0.), (-voxel_spread, 0.), (0., voxel_spread), (0., -voxel_spread)):
                    rest = ~hit
                    if not rest.any():
                        break
                    hit[rest] = bg.occupied(a[rest]+da, l[rest]+dl, h[rest])
            drop[test] = hit
        # The survey holds road-surface voxels 0.14-0.3 m high all along the road band
        # (ground-model residue, ~18,700 cells). Below ROAD_PROTECT_MIN_H they may delete
        # the lower part of a real object; never where it adjoins a protected return in
        # the road band (the same object, e.g. a 0.8 m concrete barrier, auto run 196).
        low = np.flatnonzero(drop & mapped & (height <= ROAD_PROTECT_MIN_H) &
                             (self.left-lateral > bg.kerb_face[1]+ROAD_PROTECT_M) &
                             (lateral-self.right > bg.kerb_face[-1]+ROAD_PROTECT_M))
        # Just past the surveyed exit the survey has no road-residue voxels, so the same
        # 0.14-0.3 m ground-model residue stays as an obstacle: a 5 x 0.6 m band across
        # the road 2 m past the end (12 voxels, the lowest ring's ground range 6 m ahead)
        # held the car 10 s, 4 m short of the exit (follow-20261010T162527Z 01:35:29.8-
        # 37.2, "tunnel no safe bypass"; it vanished when the car rolled back 0.4 m). There
        # such low returns in the road band count only where they adjoin a return above
        # ROAD_PROTECT_MIN_H (the lower part of a real object), as inside.
        exit_band = (~mapped & (along > self.end) & (along <= self.end+EXIT_RESIDUE_M) &
                     (height <= ROAD_PROTECT_MIN_H) &
                     (self.left-lateral > bg.kerb_face[1]+ROAD_PROTECT_M) &
                     (lateral-self.right > bg.kerb_face[-1]+ROAD_PROTECT_M))
        exit_low = np.flatnonzero(exit_band & ~drop)
        drop[exit_low] = True
        low = np.r_[low, exit_low]
        road |= (~mapped & (along > self.end) & (along <= self.end+EXIT_RESIDUE_M) & (height > ROAD_PROTECT_MIN_H) &
                 (self.left-lateral > bg.kerb_face[1]+ROAD_PROTECT_M) &
                 (lateral-self.right > bg.kerb_face[-1]+ROAD_PROTECT_M))
        anchor = q[road & ~drop, :2]
        if len(low) and len(anchor):
            from scipy.spatial import cKDTree
            near, _ = cKDTree(anchor).query(q[low, :2], k=1, distance_upper_bound=.4)
            drop[low[np.isfinite(near)]] = False
        return voxel_ds2d(q[~drop, :2]), int(drop.sum())

    def path_clearance(self, path_local, aligned_ego, horizon, margin=.25):
        """First station whose body (+margin) leaves the inner walls. A larger
        margin keeps a path off the kerb (tunnel exit join, tunnel_runtime).
        Path samples are rear-axle positions; the body is centred EGO_CENTRE_M
        ahead along the path heading (ego_body)."""
        path = np.asarray(path_local, dtype=float)
        if path.ndim != 2 or path.shape[1] < 2:
            return 0.
        path = path[:, :2]
        if len(path) < 2 or not np.isfinite(path).all() or not math.isfinite(horizon):
            return 0.
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        arc, keep = np.unique(arc, return_index=True)
        if len(arc) < 2:
            return 0.
        sample = np.arange(0., min(float(horizon), float(arc[-1]))+.001, .25)
        path = np.column_stack([np.interp(sample, arc, path[keep, k]) for k in (0, 1)])
        if len(path) < 2:
            return 0.
        rotation = _rotation(aligned_ego['heading_deg'])
        world = path @ rotation.T+[aligned_ego['x'], aligned_ego['y']]
        delta = world-self.origin
        tangent = np.gradient(path, axis=0)
        tangent /= np.maximum(1e-9, np.linalg.norm(tangent, axis=1, keepdims=True))
        tangent_world = tangent @ rotation.T
        transverse_world = np.column_stack((-tangent_world[:, 1], tangent_world[:, 0]))
        delta = body_centres(delta, tangent_world)
        corners = np.stack([delta+along*(EGO_HALF_LENGTH_M+.25)*tangent_world+
                            side*(EGO_HALF_WIDTH_M+margin)*transverse_world
                            for along in (-1., 1.) for side in (-1., 1.)], axis=1)
        corner_along, corner_lateral = _dot(corners, self.axis), _dot(corners, self.normal)
        covered = (corner_along.max(1) >= self.start) & (corner_along.min(1) <= self.end)
        valid = ~covered | ((corner_lateral.max(1) <= self.inner_left) &
                            (corner_lateral.min(1) >= self.inner_right))
        bad = np.flatnonzero(~valid)
        return float(sample[bad[0]]) if len(bad) else float(horizon)
