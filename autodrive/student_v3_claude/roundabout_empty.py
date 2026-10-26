"""Bounded empty-entry certificate for the surveyed competition roundabout."""
import hashlib
import json
import math
import time

import numpy as np
import ego_body
from scipy.spatial import cKDTree

from stop_line import StopLineController, INTERSECTION_ENTRY_MPS, EMPTY_ENTRY_ROLL_MPS

# Offline body SAT on the saved map: 8.9 x 3.2 m traffic within its 3.5 m
# lane, ego lateral error <= .2 m and heading error <= 10 degrees. A
# support-envelope probe found 4.950 m from the latest front-centre line
# projection to first ring contact; subtract .015 m sampling error and use
# only 4.9 m. At 2.6 km/h, source age .25 s and acceleration 3.2 m/s²,
# braking_reserve is 4.8788 m.
# This is a map/model bound, not a guarantee about arbitrary off-lane actors.
ROUTE_SHA256 = '47c35f037a853123abf86a5e5c0f48562330f698351cfe02a55686a5253a4e2d'
MAP_SHA256 = 'aa6e967c70f59d563d1f455224af3a90a27201bc139dcee38af071cae20b74bf'
STOP_STATION_M = 885.1006875597407
CLEARANCE_M = 4.9
ROLL_MPS = EMPTY_ENTRY_ROLL_MPS
ROLL_TARGET_MPS = 2.5 / 3.6
ROLL_PREPARATION_BRAKE_MPS2 = 1.
MAX_POSE_AGE_S = .2
# The measured detector/scan pipeline can span just over .2 s. Its separate
# .25 s budget still fits the same 4.9 m body bound at 2.6 km/h; .26 s does not.
# Localization and both source deadlines must survive the entire body check.
MAX_ENTRY_SOURCE_AGE_S = .25
# Native launch acceleration reached 3.1 m/s². Use a separate conservative
# entry-proof model; the ordinary controller and traffic launch retain 3.0.
ENTRY_ACCEL_BOUND_MPS2 = 3.2
ROLL_CHECK_FRONT_GAP_M = 5.
# Saved publication braking prefixes differ from the raw map tangent by up to
# 5.044 degrees (three entry scans); 6 degrees includes those rounded chords.
ROLL_HEADING_ERROR_DEG = 6.
ROLL_ROUTE_ERROR_M = .1
TRACK_HEADING_ERROR_DEG = 3.
TRACK_ERROR_M = .1


class EntryReference:
    """A small, fixed-map reference candidate; existing guards authorize it.

    Once selected at measured low speed, keep the same world geometry through
    a traffic interruption. Changing speed must not toggle the lateral target.
    The reference returns smoothly after the surveyed stop station; it does
    not alter map geometry, route progress, model inputs, or safety margins.
    """
    OFFSET_M = .05
    # Endpoint rounding alone leaves no room to certify an interval straddling
    # two curved map segments. Keep one centimetre inside the .1 m path bound.
    PATH_PROOF_SLACK_M = .01
    HOLD_END_M = STOP_STATION_M+1.
    RETURN_M = 4.9

    def __init__(self):
        self.active = False
        self.last_station = self.last_stamp = None

    def apply(self, lanes, ego, local_path, *, enabled, now, constant_speed=False):
        from stop_line import intersection_front_station
        original = local_path
        if constant_speed:
            # The bounded 2.5 km/h empty-roll reference has a different pose
            # certificate from normal constant-speed, full-traffic entry.
            # Retire that optional reference; the ordinary surveyed path and
            # its current raw/body/traffic publication checks remain active.
            self.active = False
            self.last_station = self.last_stamp = None
            return original
        try:
            station = float(lanes.route_progress(ego))
            stamp = float(ego['stamp'])
            speed = abs(float(ego['speed_kmh']))/3.6
            values = [station, stamp, speed, float(ego['x']), float(ego['y']),
                      float(ego['heading_deg']), float(ego['age_s'])]
            received = float(ego['received'])
            receipt_age = float(now)-received
            age = max(values[-1], receipt_age)
            if (not all(math.isfinite(v) for v in values+[received, receipt_age])
                    or stamp <= 0. or values[-1] < 0. or receipt_age < 0.):
                raise ValueError('invalid reference pose')
            if self.last_station is not None:
                dt = stamp-self.last_stamp
                jump = station-self.last_station
                if dt < 0. or jump < -20. or abs(jump) > max(30., 2.*speed*max(dt, 0.)+10.):
                    self.active = False
            self.last_station, self.last_stamp = station, stamp
            if not enabled or station > self.HOLD_END_M+self.RETURN_M:
                self.active = False
            if not enabled:
                return original
            route = np.asarray(lanes.route_xy, dtype='<f8')
            if not self.active:
                gap = STOP_STATION_M-intersection_front_station(ego, lanes.route_progress)
                if (not 0. <= gap <= ROLL_CHECK_FRONT_GAP_M or speed > ROLL_MPS
                        or age > MAX_POSE_AGE_S
                        or hashlib.sha256(route.tobytes()).hexdigest() != ROUTE_SHA256
                        or not certified_pose_geometry(lanes, ego)):
                    return original
                self.active = True
            path = np.array(local_path, dtype=np.float32, copy=True)
            if path.ndim != 2 or len(path) < 3 or path.shape[1] < 2 or not np.isfinite(path).all():
                raise ValueError('invalid reference path')
            yaw = math.radians(values[5])
            rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            xy = np.array(values[3:5])
            world = path[:, :2]@rotation.T+xy
            _, nearest = lanes.route_tree.query(world)
            # Use the fixed map tangent, never an already biased publication.
            tangent = np.gradient(route, axis=0)[nearest]
            norms = np.linalg.norm(tangent, axis=1)
            if (norms <= 1e-8).any():
                raise ValueError('degenerate reference map')
            right = np.column_stack((tangent[:, 1], -tangent[:, 0]))/norms[:, None]
            stations = lanes.route_station[nearest]
            t = np.clip((stations-self.HOLD_END_M)/self.RETURN_M, 0., 1.)
            amount = self.OFFSET_M*(1.-t**3*(10.-15.*t+6.*t*t))
            # Route smoothing already consumes part of the rolling path's
            # lateral bound. Add only the remaining room, rather than pushing
            # a 5.6 cm corner cut to 10.6 cm and failing the next entry proof.
            segments = np.clip(nearest[:, None]+np.arange(-1, 2), 0, len(route)-2)
            starts, delta = route[segments], route[segments+1]-route[segments]
            length_sq = np.maximum(np.sum(delta*delta, axis=2), 1e-12)
            fraction = np.clip(np.sum((world[:, None, :]-starts)*delta, axis=2)/length_sq, 0., 1.)
            projections = starts+fraction[:, :, None]*delta
            distances = np.linalg.norm(world[:, None, :]-projections, axis=2)
            rows, closest = np.arange(len(world)), np.argmin(distances, axis=1)
            distance = distances[rows, closest]
            limit = ROLL_ROUTE_ERROR_M-self.PATH_PROOF_SLACK_M
            # The longer stopping prefix can reach a smoothed corner already
            # outside the path bound before any bias is added. Correct only a
            # small entry-window deviation; larger errors still fail the final
            # path/pose/raw certificates instead of becoming a new route.
            # This correction ends well before the reference's spatial return
            # ends, so deactivating the reference cannot remove a live offset.
            window = (stations >= STOP_STATION_M-10.) & (stations <= self.HOLD_END_M)
            correction = np.where(window, np.minimum(self.OFFSET_M, np.maximum(0., distance-limit)), 0.)
            world += ((projections[rows, closest]-world)*
                      (correction/np.maximum(distance, 1e-12))[:, None])
            distance -= correction
            # Bound the local float32 publication's component rounding in metres.
            rounding = np.linalg.norm(np.spacing(path[:, :2]).astype(float), axis=1)
            amount = np.minimum(amount, np.maximum(0.,
                limit-distance-rounding))
            world += amount[:, None]*right
            path[:, :2] = (world-xy)@rotation
            return path
        except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
            self.active = False
            return original


def fresh_entry_pose(runtime, planned):
    """Read a coherent current pose for the fixed world entry path.

    Planning may span multiple source frames. Do not match its cached pose to
    a newer scan, or move the world path when refreshing the entry proof.
    Continuity is only an admission check; all body/path proofs still apply.
    """
    try:
        current, _, _ = runtime.ego_receiver.snapshot(1)
        pose = dict(current)
        now = time.monotonic()
        values = [float(pose[k]) for k in
                  ('x', 'y', 'heading_deg', 'speed_kmh', 'stamp', 'received', 'age_s')]
        old = [float(planned[k]) for k in ('x', 'y', 'speed_kmh', 'stamp')]
        if not all(math.isfinite(v) for v in values+old):
            return None
        x, y, _, speed, stamp, received, reported_age = values
        receipt_age = now-received
        age = max(reported_age, receipt_age)
        delta = stamp-old[3]
        if (stamp <= 0. or old[3] <= 0. or reported_age < 0. or receipt_age < 0.
                or age > MAX_POSE_AGE_S or not 0. <= delta <= .35):
            return None
        travel = max(abs(speed), abs(old[2]))/3.6*delta + 1.5*delta*delta + .1
        if math.hypot(x-old[0], y-old[1]) > travel:
            return None
        pose['age_s'] = age
        return pose
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return None


def entry_publication_can_commit(original, ego, now):
    """A stopped, invalid, old or discontinuous publication grants no entry."""
    try:
        flags = original.validation
        if (original.emergency or not math.isfinite(original.allowed_mps)
                or original.allowed_mps <= 0. or not original.fresh(now)
                or not original.checks_match_path()
                or any(flags.get(k) is not True for k in ('valid', 'pose_valid', 'steering_valid'))
                or flags.get('blocked') is not False):
            return False
        old = original.control_ego
        values = [float(p[k]) for p in (old, ego) for k in ('x', 'y', 'speed_kmh', 'stamp')]
        if not all(math.isfinite(v) for v in values) or min(values[3], values[7]) <= 0.:
            return False
        delta = values[7]-values[3]
        travel = max(abs(values[2]), abs(values[6]))/3.6*delta + 1.5*delta*delta + .1
        return (0. <= delta <= .35
                and math.hypot(values[4]-values[0], values[5]-values[1]) <= travel)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return False


def entry_scan_points(runtime, ego, status, now, *, proof=None):
    """Read one fresh empty-traffic/scan pair and align returns to this ego pose.

    This is checked again immediately before a rolling commit. The generic
    LiDAR grace period remains separate from the bounded entry source budget.
    """
    from stop_line import empty_intersection_observation
    from tunnel_runtime import points_at_pose
    try:
        cars, ready, latest = runtime.vehicle_receiver.snapshot()
        pose_age = max(float(ego.get('age_s', 0.)),
                       now-float(ego.get('received', now)))
        pose_stamp = float(ego['stamp'])
        car_stamp = float(latest['stamp'])
        car_age = max(float(latest['age_s']), max(0., pose_stamp-car_stamp)+pose_age)
        current = dict(status, detector_ready=ready, car_count=len(cars),
                       vehicles=latest['vehicles'], stamp=car_stamp, age_s=car_age)
        for key in ('ghost_ids', 'inherited_ids', 'unknown_ids'):
            if latest.get(key):
                return None
        if (not empty_intersection_observation(current) or cars
                or not 0. <= pose_age <= MAX_POSE_AGE_S
                or not 0. <= car_age <= MAX_ENTRY_SOURCE_AGE_S
                or not -.12 <= pose_stamp-car_stamp <= MAX_ENTRY_SOURCE_AGE_S):
            return None
        scan = runtime.lidar.scan_snapshot()
        stamp = float(scan['stamp'])
        age = max(float(scan['age_s']), max(0., pose_stamp-stamp)+pose_age)
        raw = scan['obstacle_points']
        if (stamp <= 0. or not 0. <= age <= MAX_ENTRY_SOURCE_AGE_S
                or not -.12 <= pose_stamp-stamp <= MAX_ENTRY_SOURCE_AGE_S or raw is None):
            return None
        scan_pose = runtime.ego_receiver.at_stamp(stamp)
        if scan_pose is None:
            return None
        if proof is not None:
            proof.update(checked_at=now,
                expires_at=min(now+MAX_POSE_AGE_S-pose_age,
                               now+MAX_ENTRY_SOURCE_AGE_S-max(car_age, age)),
                car_stamp=car_stamp, scan_stamp=stamp)
        return points_at_pose(raw, scan_pose, ego)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return None


def entry_proof_fresh(proof, now):
    """The same inspected source pair must remain fresh after body checking."""
    try:
        checked, expires, current = float(proof['checked_at']), float(proof['expires_at']), float(now)
        return bool(all(math.isfinite(v) for v in (checked, expires, current))
                    and checked <= current <= expires)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def raw_entry_clearance(lanes, ego, local_points, world_path=None):
    """Certify the low-speed stopping prefix, or return zero to require a stop.

    Sweep the actual published prefix, with tracking error <=3 degrees/.1 m.
    Its map error <=6 degrees/.1 m keeps the combined body within the offline
    map certificate's 10 degrees/.2 m. Include both incident path tangents,
    existing .25 m margins and .025 m sample translation. This remains a
    conditional tracking model, rechecked live. The extra metre is a buffer.
    """
    try:
        route = np.asarray(lanes.route_xy)
        def inside_route(points, tangents, neighbours, *, normalize_tangent):
            # Keep adjacent segments from both nearest vertices. Taking only
            # the smaller vertex index can omit the closest outgoing segment
            # at a curved corner, unlike the pose/path geometry certificates.
            neighbours = np.asarray(neighbours).reshape(len(points), -1)
            candidates = np.clip((neighbours[:, :, None]+np.arange(-1, 2)).reshape(len(points), -1),
                                 0, len(route)-2)
            candidates.sort(axis=1)
            starts = route[candidates]
            references = route[candidates+1]-starts
            length_sq = np.maximum(1e-12, np.sum(references*references, axis=2))
            fraction = np.clip(np.sum((points[:, None, :]-starts)*references, axis=2)/length_sq, 0., 1.)
            projected = starts+fraction[:, :, None]*references
            distance_sq = np.sum((projected-points[:, None, :])**2, axis=2)
            rows, at = np.arange(len(points)), np.argmin(distance_sq, axis=1)
            distance = np.sqrt(distance_sq[rows, at])
            reference = references[rows, at]
            norm = np.linalg.norm(reference, axis=1)
            if normalize_tangent:
                norm *= np.linalg.norm(tangents, axis=1)
            alignment = np.sum(reference*tangents, axis=1)/norm
            minimum = math.cos(math.radians(ROLL_HEADING_ERROR_DEG))
            invalid = (distance > ROLL_ROUTE_ERROR_M) | (alignment < minimum)
            if normalize_tangent:
                invalid |= norm <= 1e-8
            # Scalar dot/norm can round differently at a decision boundary.
            # Re-evaluate only those rows with the unchanged original formula.
            boundary = ((abs(distance-ROLL_ROUTE_ERROR_M) <= 1e-12) |
                        (abs(alignment-minimum) <= 1e-12))
            if normalize_tangent:
                boundary |= abs(norm-1e-8) <= 1e-12
            for i in np.flatnonzero(boundary):
                _, scalar_distance, segment = lanes._project_route(points[i], candidates[i])
                scalar_reference = route[segment+1]-route[segment]
                scalar_norm = np.linalg.norm(scalar_reference)
                if normalize_tangent:
                    scalar_norm *= np.linalg.norm(tangents[i])
                invalid[i] = (scalar_distance > ROLL_ROUTE_ERROR_M or
                    (normalize_tangent and scalar_norm <= 1e-8) or
                    scalar_reference@tangents[i]/scalar_norm < minimum)
            return not invalid.any()

        xy = np.array([float(ego['x']), float(ego['y'])])
        yaw = math.radians(float(ego['heading_deg']))
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        speed = abs(float(ego.get('speed_kmh', 0.)))/3.6
        if not math.isfinite(speed) or speed > ROLL_MPS:
            return 0.
        reserve = braking_reserve(speed, MAX_ENTRY_SOURCE_AGE_S)
        path = np.asarray(world_path, float)[:, :2]
        if len(path) < 3 or not np.isfinite(path).all():
            return 0.
        lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        if (lengths <= 1e-8).any():
            return 0.
        directions = np.diff(path, axis=0)/lengths[:, None]
        fractions = np.sum((xy-path[:-1])*directions, axis=1)
        projected = path[:-1]+np.clip(fractions, 0., lengths)[:, None]*directions
        nearest = int(np.argmin(np.linalg.norm(xy-projected, axis=1)))
        start = float(np.r_[0., np.cumsum(lengths)][nearest]+np.clip(fractions[nearest], 0., lengths[nearest]))
        if nearest == 0 and -.2 <= fractions[0] < 0.:
            # Publications may begin a few centimetres ahead of the rear axle.
            # Extend only that first tangent to the measured orthogonal start.
            projected[0] = path[0]+fractions[0]*directions[0]
            path = np.vstack((projected[0], path))
            lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
            directions = np.diff(path, axis=0)/lengths[:, None]
            start = 0.
        if (np.linalg.norm(xy-projected[nearest]) > TRACK_ERROR_M or
                directions[nearest]@heading < math.cos(math.radians(TRACK_HEADING_ERROR_DEG))):
            return 0.
        arc = np.r_[0., np.cumsum(lengths)]
        end = start+reserve-1.
        if arc[-1] < end:
            return 0.
        # A large tangent jump needs a rotation sweep; fall back to a stop.
        relevant = (arc[1:-1] >= start) & (arc[1:-1] <= end)
        if (np.sum(directions[:-1]*directions[1:], axis=1)[relevant] <
                math.cos(math.radians(2.*TRACK_HEADING_ERROR_DEG))).any():
            return 0.
        vertices = np.concatenate((path[:-1], .5*(path[:-1]+path[1:]), path[1:]))
        vertex_arc = np.concatenate((arc[:-1], .5*(arc[:-1]+arc[1:]), arc[1:]))
        tangents = np.tile(directions, (3, 1))
        _, nearby = lanes.route_tree.query(vertices, k=2)
        window = (vertex_arc >= start-.025) & (vertex_arc <= end+.025)
        if not inside_route(vertices[window], tangents[window], nearby[window],
                            normalize_tangent=True):
            return 0.
        samples = np.unique(np.r_[np.arange(start, end, .05), end,
            arc[(arc >= start) & (arc <= end)]])
        prefix = np.column_stack([np.interp(samples, arc, path[:, i]) for i in (0, 1)])
        # Distance to one convex map segment is convex along a straight path
        # interval. Certifying both endpoints against that SAME segment bounds
        # the whole interval, including the gaps between sampled points.
        _, near = lanes.route_tree.query(prefix, k=2)
        candidates = np.clip(np.concatenate((near[:-1]-1, near[:-1], near[1:]-1, near[1:]), axis=1), 0, len(route)-2)
        starts, lines = route[candidates], route[candidates+1]-route[candidates]
        norms = np.linalg.norm(lines, axis=2)
        distances = []
        for endpoint in (prefix[:-1], prefix[1:]):
            delta = endpoint[:, None, :]-starts
            fraction = np.clip(np.sum(delta*lines, axis=2)/np.maximum(norms**2, 1e-12), 0., 1.)
            distances.append(np.linalg.norm(delta-fraction[:, :, None]*lines, axis=2))
        interval = np.diff(prefix, axis=0)
        alignment = np.sum(lines*interval[:, None, :], axis=2)/np.maximum(norms*np.linalg.norm(interval, axis=1)[:, None], 1e-12)
        common = ((distances[0] <= ROLL_ROUTE_ERROR_M) & (distances[1] <= ROLL_ROUTE_ERROR_M)
                  & (alignment >= math.cos(math.radians(ROLL_HEADING_ERROR_DEG))))
        if not common.any(axis=1).all():
            return 0.
        # At vertices both adjacent segment headings are admissible.
        along = np.r_[samples, samples]
        segments = np.r_[np.searchsorted(arc, samples, side='left')-1,
                         np.searchsorted(arc, samples, side='right')-1]
        segments = np.clip(segments, 0, len(path)-2)
        tangent = directions[segments]
        centres = np.column_stack([np.interp(along, arc, path[:, i]) for i in (0, 1)])
        # Certify uniform prefix samples as well as original path vertices.
        _, nearby = lanes.route_tree.query(centres, k=2)
        if not inside_route(centres, tangent, nearby, normalize_tangent=False):
            return 0.
        angle = math.radians(TRACK_HEADING_ERROR_DEG)
        front = 3.845*math.cos(angle)+.946*math.sin(angle)
        rear = .79*math.cos(angle)+.946*math.sin(angle)
        width = 3.845*math.sin(angle)+.946*math.cos(angle)
        centres += .5*(front-rear)*tangent
        half_along = .5*(front+rear)+.25+TRACK_ERROR_M+.025+1.
        half_width = width+.25+TRACK_ERROR_M
        points = np.asarray(local_points, dtype=float).reshape(-1, 2)
        if not np.isfinite(points).all():
            return 0.
        if not len(points):
            return reserve
        points = points@np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])+xy
        neighbours = cKDTree(points).query_ball_point(centres, math.hypot(half_along, half_width))
        counts = np.fromiter((len(ids) for ids in neighbours), int, count=len(neighbours))
        if not counts.sum():
            return reserve
        poses = np.repeat(np.arange(len(centres)), counts)
        ids = np.concatenate([ids for ids in neighbours if ids])
        delta, direction = points[ids]-centres[poses], tangent[poses]
        distance = np.abs(np.sum(delta*direction, axis=1))
        across = np.abs(delta[:, 0]*direction[:, 1]-delta[:, 1]*direction[:, 0])
        hit = (distance <= half_along) & (across <= half_width)
        return 0. if hit.any() else reserve
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return 0.


def map_geometry_digest(links):
    """Recheck mutable map content, using only fields that define the certificate."""
    digest = hashlib.sha256()
    for link in sorted(links, key=lambda item: item['idx']):
        fields = {key: link.get(key) for key in ('idx', 'from_node_idx', 'to_node_idx',
            'link_type', 'road_type', 'width_start', 'width_end', 'force_width_start', 'force_width_end')}
        digest.update(json.dumps(fields, sort_keys=True, separators=(',', ':')).encode())
        points = np.asarray(link['points'], dtype='<f8')[:, :2]
        digest.update(str(points.shape).encode())
        digest.update(points.tobytes())
    return digest.hexdigest()


def semantic_entry_clearance(path, ego, objects, horizon_m=60.):
    """Current semantic bodies against the same tracking tube as raw returns.

    Original segment tangents, both incident headings, and the actual t=0
    heading prevent a resampled gradient from narrowing a corner's envelope.
    Compare the returned contact distance with braking_reserve: that already
    includes a metre beyond actual stopping travel as a longitudinal buffer.
    """
    try:
        path = np.asarray(path, float)[:, :2]
        if len(path) < 2 or not np.isfinite(path).all():
            return 0.
        lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        path = path[np.r_[True, lengths > 1e-8]]
        if len(path) < 2:
            return 0.
        lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        arc = np.r_[0., np.cumsum(lengths)]
        directions = np.diff(path, axis=0)/lengths[:, None]
        fraction = -np.sum(path[:-1]*directions, axis=1)
        projected = path[:-1]+np.clip(fraction, 0., lengths)[:, None]*directions
        nearest = int(np.argmin(np.linalg.norm(projected, axis=1)))
        start = arc[nearest]+np.clip(fraction[nearest], 0., lengths[nearest])
        if nearest == 0 and -.2 <= fraction[0] < 0.:
            path = np.vstack((path[0]+fraction[0]*directions[0], path))
            lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
            arc = np.r_[0., np.cumsum(lengths)]
            start = 0.
        horizon = min(float(horizon_m), float(arc[-1]-start))
        if not math.isfinite(horizon) or horizon <= 0.:
            return 0.
        boxes = list(objects)
        if not boxes:
            return horizon
        end = start+horizon
        samples = np.unique(np.r_[np.arange(start, end, .05), end,
                                  arc[(arc >= start) & (arc <= end)]])
        station = np.r_[samples-start, samples-start, 0.]
        segments = np.clip(np.r_[np.searchsorted(arc, samples, side='left')-1,
            np.searchsorted(arc, samples, side='right')-1], 0, len(path)-2)
        tangent = np.diff(path, axis=0)[segments]/lengths[segments, None]
        tangent = np.vstack((tangent, [1., 0.]))
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        angle = math.radians(TRACK_HEADING_ERROR_DEG)
        front = 3.845*math.cos(angle)+.946*math.sin(angle)
        rear = .79*math.cos(angle)+.946*math.sin(angle)
        width = 3.845*math.sin(angle)+.946*math.cos(angle)
        along = np.r_[samples, samples]
        centres = np.vstack((np.column_stack([np.interp(along, arc, path[:, i]) for i in (0, 1)]), [0., 0.]))
        centres += .5*(front-rear)*tangent
        half_length = .5*(front+rear)+.25+TRACK_ERROR_M+.025
        half_width = width+.25+TRACK_ERROR_M+.025
        yaw = math.radians(float(ego['heading_deg']))
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        positions = (np.asarray([o[:2] for o in boxes], float)-[ego['x'], ego['y']])@rotation
        headings = np.radians([float(o[8])-float(ego['heading_deg']) for o in boxes])
        axes = np.column_stack((np.cos(headings), np.sin(headings)))
        cross = np.column_stack((-axes[:, 1], axes[:, 0]))
        sizes = np.asarray([o[6:8] for o in boxes], float)/2.
        if (not np.isfinite(np.r_[positions.ravel(), axes.ravel(), sizes.ravel()]).all()
                or (sizes <= 0.).any()):
            return 0.
        delta = positions[:, None, :]-centres
        at, an = abs(axes@tangent.T), abs(axes@normal.T)
        ct, cn = abs(cross@tangent.T), abs(cross@normal.T)
        lengths, widths = sizes[:, 0, None], sizes[:, 1, None]
        overlap = abs(np.sum(delta*tangent, axis=2)) <= half_length+lengths*at+widths*ct
        overlap &= abs(np.sum(delta*normal, axis=2)) <= half_width+lengths*an+widths*cn
        overlap &= abs(np.sum(delta*axes[:, None, :], axis=2)) <= half_length*at+half_width*an+lengths
        overlap &= abs(np.sum(delta*cross[:, None, :], axis=2)) <= half_length*ct+half_width*cn+widths
        hit = overlap.any(axis=0)
        return float(station[hit].min()) if hit.any() else horizon
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return 0.


def braking_reserve(speed, age):
    """Allow acceleration during observation/response delay, then brake at 2 m/s²."""
    delay = float(age) + StopLineController.REACTION_S
    speed = abs(float(speed))
    acceleration = max(ENTRY_ACCEL_BOUND_MPS2, StopLineController.ACCEL_MPS2)
    future = speed + acceleration * delay
    return (speed*delay + .5*acceleration*delay*delay
            + future*future/(2.*StopLineController.BRAKE_MPS2) + 1.)


def rolling_preparation_cap(front_gap):
    """Prepare a low-speed approach; this limit grants no crossing permission."""
    return math.sqrt(ROLL_TARGET_MPS**2 + 2.*ROLL_PREPARATION_BRAKE_MPS2
                     * max(0., float(front_gap)-ROLL_CHECK_FRONT_GAP_M))


def approach_cap(front_gap, speed, age, clearance=0.):
    """Retain braking until a measured low-speed, certified empty roll is feasible."""
    try:
        gap, speed, age, clearance = map(float, (front_gap, speed, age, clearance))
    except (TypeError, ValueError):
        return 0.
    if not all(math.isfinite(v) for v in (gap, speed, age, clearance)) or speed < 0. or age < 0.:
        return 0.
    travelled = speed*age + .5*StopLineController.ACCEL_MPS2*age*age
    room = max(0., gap-travelled-StopLineController.BUFFER_M-speed*StopLineController.REACTION_S)
    cap = min(INTERSECTION_ENTRY_MPS, math.sqrt(2.*StopLineController.BRAKE_MPS2*room))
    # Reach the rolling target before its check window instead of dropping
    # abruptly to it at five metres. The ordinary stop-line envelope still
    # bounds every preparation command; only the certified floor below can
    # lift a zero cap.
    cap = min(cap, rolling_preparation_cap(gap))
    if (speed <= ROLL_MPS and age <= MAX_POSE_AGE_S and clearance > 0.
            and braking_reserve(speed, MAX_ENTRY_SOURCE_AGE_S) <= clearance + max(0., gap)):
        # Leave measured-speed headroom below the certificate ceiling.
        cap = max(cap, ROLL_TARGET_MPS)
    return cap


def certified_path_geometry(lanes, world_path):
    """Check body, mutable map and publication bounds without reading ego pose.

    Run this for each checked publication; it is not a reusable certificate.
    The current pose must then pass certified_pose_geometry in the same call.
    """
    try:
        if (ego_body.EGO_FRONT_M, ego_body.EGO_REAR_M, ego_body.EGO_HALF_WIDTH_M) != (3.845, .79, .946):
            return False
        route = np.asarray(lanes.route_xy, dtype='<f8')
        if (hashlib.sha256(route.tobytes()).hexdigest() != ROUTE_SHA256 or
                map_geometry_digest(lanes.links) != MAP_SHA256):
            return False
        path = np.asarray(world_path, dtype=float)[:, :2]
        if len(path) < 3 or not np.isfinite(path).all():
            return False
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        keep = np.r_[True, np.diff(arc) > 1e-4]
        path, arc = path[keep], arc[keep]
        if len(path) < 3:
            return False
        # Inspect every original segment as well as uniform samples. A brief
        # reversal or lateral spike must not disappear between sampled points.
        vertices = np.concatenate((path[:-1], .5*(path[:-1]+path[1:]), path[1:]))
        directions = np.tile(np.diff(path, axis=0), (3, 1))
        _, nearby = lanes.route_tree.query(vertices, k=2)
        nearest = np.min(nearby, axis=1)
        station = lanes.route_station[nearest]
        window = (station >= STOP_STATION_M-10.) & (station <= STOP_STATION_M+7.)
        vertices, directions, nearest = vertices[window], directions[window], nearest[window]
        segments = np.clip(nearest[:, None]+np.arange(-1, 2), 0, len(route)-2)
        starts, along = route[segments], route[segments+1]-route[segments]
        length = np.linalg.norm(along, axis=2)
        delta = vertices[:, None, :]-starts
        fraction = np.clip(np.sum(delta*along, axis=2)/np.maximum(length**2, 1e-12), 0., 1.)
        distance = np.linalg.norm(delta-fraction[:, :, None]*along, axis=2)
        norm = length*np.linalg.norm(directions, axis=1)[:, None]
        alignment = np.sum(along*directions[:, None, :], axis=2)/np.maximum(norm, 1e-12)
        # Retain every incident candidate within the original 1e-7 tie band.
        closest = distance.min(axis=1)
        incident = distance <= closest[:, None]+1e-7
        best_alignment = np.where(incident, alignment, -np.inf).max(axis=1)
        largest_norm = norm.max(axis=1)
        minimum = math.cos(math.radians(10.))
        invalid = (closest > .2) | (largest_norm <= 1e-8) | (best_alignment < minimum)
        boundary = ((abs(closest-.2) <= 1e-12) | (abs(largest_norm-1e-8) <= 1e-12) |
                    (abs(best_alignment-minimum) <= 1e-12))
        boundary |= (abs(distance-(closest[:, None]+1e-7)) <= 1e-12).any(axis=1)
        for i in np.flatnonzero(boundary):
            scalar_starts, scalar_along = route[segments[i]], route[segments[i]+1]-route[segments[i]]
            scalar_length = np.linalg.norm(scalar_along, axis=1)
            scalar_fraction = np.clip(np.sum((vertices[i]-scalar_starts)*scalar_along, axis=1)/
                np.maximum(scalar_length**2, 1e-12), 0., 1.)
            scalar_distance = np.linalg.norm(vertices[i]-scalar_starts-scalar_fraction[:, None]*scalar_along, axis=1)
            scalar_norm = scalar_length*np.linalg.norm(directions[i])
            scalar_alignment = (scalar_along@directions[i])/np.maximum(scalar_norm, 1e-12)
            scalar_incident = scalar_distance <= scalar_distance.min()+1e-7
            invalid[i] = (scalar_distance.min() > .2 or scalar_norm.max() <= 1e-8 or
                          scalar_alignment[scalar_incident].max() < minimum)
        if invalid.any():
            return False
        samples = np.arange(0., arc[-1], .5)
        points = np.column_stack([np.interp(samples, arc, path[:, i]) for i in (0, 1)])
        _, neighbours = lanes.route_tree.query(points, k=2)
        nearest = np.min(neighbours, axis=1)
        station = lanes.route_station[nearest]
        window = (station >= STOP_STATION_M-10.) & (station <= STOP_STATION_M+7.)
        if not window.any() or station.max() < STOP_STATION_M+7.:
            return False
        tangent = np.gradient(points, axis=0)
        checked_points, checked_tangent = points[window], tangent[window]
        segments = np.clip(nearest[window, None]+np.arange(-1, 2), 0, len(route)-2)
        starts, along = route[segments], route[segments+1]-route[segments]
        length_sq = np.maximum(1e-12, np.sum(along*along, axis=2))
        fraction = np.clip(np.sum((checked_points[:, None, :]-starts)*along, axis=2)/length_sq, 0., 1.)
        projected = starts+fraction[:, :, None]*along
        distance_sq = np.sum((projected-checked_points[:, None, :])**2, axis=2)
        rows, at = np.arange(len(checked_points)), np.argmin(distance_sq, axis=1)
        distance = np.sqrt(distance_sq[rows, at])
        reference = along[rows, at]
        norm = np.linalg.norm(reference, axis=1)*np.linalg.norm(checked_tangent, axis=1)
        alignment = np.sum(reference*checked_tangent, axis=1)/norm
        invalid = (distance > .2) | (norm <= 1e-8) | (alignment < minimum)
        boundary = ((abs(distance-.2) <= 1e-12) | (abs(norm-1e-8) <= 1e-12) |
                    (abs(alignment-minimum) <= 1e-12))
        for i in np.flatnonzero(boundary):
            _, scalar_distance, segment = lanes._project_route(checked_points[i], segments[i])
            scalar_reference = route[segment+1]-route[segment]
            scalar_norm = np.linalg.norm(scalar_reference)*np.linalg.norm(checked_tangent[i])
            invalid[i] = (scalar_distance > .2 or scalar_norm <= 1e-8 or
                          float(scalar_reference@checked_tangent[i])/scalar_norm < minimum)
        if invalid.any():
            return False
        return True
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return False


def certified_pose_geometry(lanes, ego):
    """Check the current pose after the same-call path/map certificate."""
    try:
        route = np.asarray(lanes.route_xy, dtype='<f8')
        point = np.array([float(ego['x']), float(ego['y'])])
        _, nearby = lanes.route_tree.query(point, k=2)
        segments = np.clip(np.r_[nearby-1, nearby], 0, len(route)-2)
        _, distance, segment = lanes._project_route(point, segments, ego['heading_deg'])
        yaw = math.radians(float(ego['heading_deg']))
        along = route[segment+1]-route[segment]
        alignment = along@np.array([math.cos(yaw), math.sin(yaw)])/np.linalg.norm(along)
        return bool(distance <= .2 and alignment >= math.cos(math.radians(10.)))
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return False


def certified_geometry(lanes, ego, world_path):
    """Check the actual publication against the fixed map certificate's bounds."""
    return certified_path_geometry(lanes, world_path) and certified_pose_geometry(lanes, ego)
